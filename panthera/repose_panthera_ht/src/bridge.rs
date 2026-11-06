//! The `CuBridge`: standard Repose channels plus the vendor `panthera_status`.
use crate::{
    mapping,
    params::{CuPantheraHtParams, duration},
    protocol::CuPantheraStatus,
    transport::{self, Shared},
};
use cu29::prelude::*;
#[cfg(feature = "control")]
use cu29::units::si::angle::radian;
#[cfg(feature = "control")]
use repose_controller::{CuControllerMode, CuJointCommandPayload};
#[cfg(feature = "control")]
use repose_kinematics::CuJointPosition;
use repose_kinematics::CuJointStatePayload;
#[cfg(feature = "control")]
use repose_robot_interface::CuSessionCommand;
use repose_robot_interface::CuSessionState;
#[cfg(feature = "control")]
use std::time::Instant;
use std::{
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
    },
    thread::JoinHandle,
    time::Duration,
};

#[cfg(not(feature = "control"))]
pub struct PantheraTxChannels;
#[cfg(not(feature = "control"))]
pub type PantheraTxId = core::convert::Infallible;
#[cfg(not(feature = "control"))]
impl BridgeChannelSet for PantheraTxChannels {
    type Id = PantheraTxId;
    const STATIC_CHANNELS: &'static [&'static dyn BridgeChannelInfo<Self::Id>] = &[];
}
#[cfg(feature = "control")]
tx_channels! {
    pub struct PantheraTxChannels: PantheraTxId {
        joint_command => CuJointCommandPayload<6> = "joint_command",
        session => CuSessionCommand = "session",
    }
}
rx_channels! {
    pub struct PantheraRxChannels: PantheraRxId {
        joint_state => CuJointStatePayload<6> = "joint_state",
        panthera_status => CuPantheraStatus = "panthera_status",
        session_state => CuSessionState = "session_state",
    }
}

/// Copper bridge to a HighTorque Panthera HT over the v6 comm board.
///
/// Each motor-state frame is published once per Rx channel. Empty, contended,
/// or stale slots clear the output. Transport/protocol failure returns an
/// error; restart explicitly to reconnect.
#[derive(Reflect)]
pub struct CuPantheraHtBridge {
    params: CuPantheraHtParams,
    seen: [u64; 2],
    #[reflect(ignore)]
    shared: Arc<Mutex<Shared>>,
    #[reflect(ignore)]
    shutdown: Arc<AtomicBool>,
    #[reflect(ignore)]
    worker: Option<JoinHandle<()>>,
    #[reflect(ignore)]
    stale_after: Duration,
}
impl CuPantheraHtBridge {
    pub fn from_params(params: CuPantheraHtParams) -> CuResult<Self> {
        params.validate()?;
        Ok(Self {
            stale_after: duration(params.stale_after)?,
            params,
            seen: [0; 2],
            shared: Arc::new(Mutex::new(Shared::default())),
            shutdown: Arc::new(AtomicBool::new(false)),
            worker: None,
        })
    }
    fn close(&mut self) {
        self.shutdown.store(true, Ordering::Release);
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }
}
impl Drop for CuPantheraHtBridge {
    fn drop(&mut self) {
        self.close();
    }
}
impl Freezable for CuPantheraHtBridge {}
impl CuBridge for CuPantheraHtBridge {
    type Tx = PantheraTxChannels;
    type Rx = PantheraRxChannels;
    type Resources<'r> = ();
    fn new(
        config: Option<&ComponentConfig>,
        _tx: &[BridgeChannelConfig<PantheraTxId>],
        _rx: &[BridgeChannelConfig<PantheraRxId>],
        _resources: (),
    ) -> CuResult<Self> {
        Self::from_params(
            config
                .map(ComponentConfig::deserialize_into)
                .transpose()?
                .unwrap_or_default(),
        )
    }
    fn start(&mut self, _ctx: &CuContext) -> CuResult<()> {
        self.close();
        let port = transport::connect(&self.params)?;
        self.shared = Arc::new(Mutex::new(Shared::default()));
        self.seen = [0; 2];
        self.shutdown.store(false, Ordering::Release);
        let shared = self.shared.clone();
        let shutdown = self.shutdown.clone();
        let params = self.params.clone();
        self.worker = Some(
            std::thread::Builder::new()
                .name("panthera-serial".into())
                .spawn(move || {
                    let result = transport::run(port, &params, &shared, &shutdown);
                    if let Err(error) = &result {
                        eprintln!("Panthera transport stopped: {error}");
                    }
                    if let Ok(mut state) = shared.lock() {
                        state.failed = result.is_err();
                        state.latest = None;
                        state.session.on_disconnected();
                    }
                })
                .map_err(|e| CuError::new_with_cause("Panthera worker startup failed", e))?,
        );
        Ok(())
    }
    #[cfg(not(feature = "control"))]
    fn send<'a, P: CuMsgPayload + 'a>(
        &mut self,
        _ctx: &CuContext,
        _channel: &'static BridgeChannel<PantheraTxId, P>,
        _msg: &CuMsg<P>,
    ) -> CuResult<()> {
        Err(CuError::from(
            "Panthera monitoring bridge has no transmit channels",
        ))
    }
    #[cfg(feature = "control")]
    fn send<'a, P: CuMsgPayload + 'a>(
        &mut self,
        _ctx: &CuContext,
        channel: &'static BridgeChannel<PantheraTxId, P>,
        msg: &CuMsg<P>,
    ) -> CuResult<()> {
        if channel.id == PantheraTxId::Session {
            let Some(&command) = msg.downcast_ref::<CuSessionCommand>()?.payload() else {
                return Ok(());
            };
            if !self.params.allow_control {
                return Err(CuError::from("Panthera session requires allow_control"));
            }
            if self.worker.is_none() {
                return Err(CuError::from("Panthera bridge not started"));
            }
            let mut state = self
                .shared
                .try_lock()
                .map_err(|_| CuError::from("Panthera command state busy"))?;
            if state.failed {
                return Err(CuError::from("Panthera failed; restart bridge"));
            }
            state.session.request(command);
            return Ok(());
        }
        let Some(command) = msg.downcast_ref::<CuJointCommandPayload<6>>()?.payload() else {
            return Ok(());
        };
        if !self.params.allow_motion {
            return Err(CuError::from(
                "Panthera motion requires allow_motion = true",
            ));
        }
        if self.worker.is_none() {
            return Err(CuError::from("Panthera bridge is not started"));
        }
        if command.mode != CuControllerMode::Position {
            return Err(CuError::from(
                "Panthera supports position commands only (MIT frame)",
            ));
        }
        let mut target = [0.0; 6];
        for (out, position) in target.iter_mut().zip(command.positions) {
            let CuJointPosition::Revolute(angle) = position else {
                return Err(CuError::from("Panthera requires six revolute joints"));
            };
            *out = angle.get::<radian>();
            if !out.is_finite() {
                return Err(CuError::from("Panthera nonfinite joint target"));
            }
        }
        let mut state = self
            .shared
            .try_lock()
            .map_err(|_| CuError::from("Panthera command state busy; retry explicitly"))?;
        if state.failed {
            return Err(CuError::from("Panthera failed; restart bridge"));
        }
        let Some((_, status)) = &state.latest else {
            return Err(CuError::from("Panthera has no telemetry yet"));
        };
        if status.motors.iter().any(|m| m.fault != 0) {
            return Err(CuError::from(
                "Panthera motor fault active; clear faults first",
            ));
        }
        let max_step = self.params.max_step.get::<radian>();
        for (motor, goal) in status.motors.iter().zip(target) {
            let actual = motor.position.get::<radian>();
            if (actual - goal).abs() > max_step {
                return Err(CuError::from("Panthera target exceeds max_step"));
            }
        }
        if state.last_command == Some(target) {
            return Ok(()); // duplicate is idempotent
        }
        state.pending = Some((target, Instant::now()));
        Ok(())
    }
    fn receive<'a, P: CuMsgPayload + 'a>(
        &mut self,
        ctx: &CuContext,
        channel: &'static BridgeChannel<PantheraRxId, P>,
        msg: &mut CuMsg<P>,
    ) -> CuResult<()> {
        msg.clear_payload();
        msg.tov = Tov::None;
        if self.worker.is_none() {
            return Err(CuError::from("Panthera bridge is not started"));
        }
        let mut shared = match self.shared.try_lock() {
            Ok(s) => s,
            Err(std::sync::TryLockError::WouldBlock) => return Ok(()),
            Err(_) => return Err(CuError::from("Panthera state poisoned")),
        };
        if shared.failed {
            return Err(CuError::from(
                "Panthera transport/protocol failed; restart bridge",
            ));
        }
        if channel.id == PantheraRxId::SessionState {
            if shared.session.take_changed() {
                msg.tov = Tov::Time(ctx.now());
                msg.downcast_mut::<CuSessionState>()?
                    .set_payload(shared.session.state().clone());
            }
            return Ok(());
        }
        let index = match channel.id {
            PantheraRxId::JointState => 0,
            PantheraRxId::PantheraStatus => 1,
            PantheraRxId::SessionState => unreachable!(),
        };
        if self.seen[index] == shared.generation {
            return Ok(());
        }
        self.seen[index] = shared.generation;
        let Some((at, status)) = &shared.latest else {
            return Ok(());
        };
        if at.elapsed() > self.stale_after {
            return Ok(());
        }
        msg.tov = Tov::Time(
            ctx.now()
                - CuDuration::from_nanos(at.elapsed().as_nanos().min(u64::MAX as u128) as u64),
        );
        match channel.id {
            PantheraRxId::JointState => msg
                .downcast_mut::<CuJointStatePayload<6>>()?
                .set_payload(mapping::joint_state(status)),
            PantheraRxId::PantheraStatus => msg
                .downcast_mut::<CuPantheraStatus>()?
                .set_payload(status.clone()),
            PantheraRxId::SessionState => unreachable!(),
        }
        Ok(())
    }
    fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
        self.close();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(feature = "control")]
    use cu29::units::si::f64::Angle;
    #[test]
    fn monitoring_bridge_has_no_tx() {
        let bridge = CuPantheraHtBridge::from_params(Default::default()).unwrap();
        assert!(bridge.worker.is_none());
    }
    #[cfg(feature = "control")]
    #[test]
    fn motion_gates() {
        let mut bridge = CuPantheraHtBridge::from_params(Default::default()).unwrap();
        bridge.worker = Some(std::thread::spawn(|| {}));
        let (ctx, _) = CuContext::new_mock_clock();
        let mut msg = CuMsg::default();
        let command = CuJointCommandPayload::<6> {
            positions: [CuJointPosition::Revolute(Angle::new::<radian>(0.1)); 6],
            ..Default::default()
        };
        msg.set_payload(command.clone());
        // allow_motion off.
        assert!(
            bridge
                .send(&ctx, &PantheraTxChannels::JOINT_COMMAND, &msg)
                .is_err()
        );
        bridge.params.allow_motion = true;
        bridge.params.allow_control = true;
        // No telemetry yet.
        assert!(
            bridge
                .send(&ctx, &PantheraTxChannels::JOINT_COMMAND, &msg)
                .is_err()
        );
        bridge.shared.lock().unwrap().latest = Some((Instant::now(), CuPantheraStatus::default()));
        // Velocity mode refused.
        let mut velocity = command.clone();
        velocity.mode = CuControllerMode::Velocity;
        msg.set_payload(velocity);
        assert!(
            bridge
                .send(&ctx, &PantheraTxChannels::JOINT_COMMAND, &msg)
                .is_err()
        );
        // Position accepted, duplicate idempotent.
        msg.set_payload(command.clone());
        bridge
            .send(&ctx, &PantheraTxChannels::JOINT_COMMAND, &msg)
            .unwrap();
        bridge
            .send(&ctx, &PantheraTxChannels::JOINT_COMMAND, &msg)
            .unwrap();
        assert!(bridge.shared.lock().unwrap().pending.is_some());
    }
}
