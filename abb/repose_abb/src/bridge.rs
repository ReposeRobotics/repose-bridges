use crate::{
    mapping,
    params::{CuAbbParams, duration},
    protocol::CuAbbStatus,
    transport::{self, Shared},
};
use cu29::prelude::*;
use repose_kinematics::CuJointStatePayload;
use repose_robot_interface::CuSessionState;
use std::{
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
    },
    thread::JoinHandle,
    time::Duration,
};
#[cfg(feature = "control")]
use {
    crate::transport::Target,
    cu29::units::si::{angle::radian, angular_velocity::radian_per_second},
    repose_controller::{CuControllerMode, CuJointCommandPayload},
    repose_kinematics::{CuJointPosition, CuJointVelocity},
    repose_robot_interface::CuSessionCommand,
    std::time::Instant,
};

#[cfg(not(feature = "control"))]
pub struct AbbTxChannels;
#[cfg(not(feature = "control"))]
pub type AbbTxId = core::convert::Infallible;
#[cfg(not(feature = "control"))]
impl BridgeChannelSet for AbbTxChannels {
    type Id = AbbTxId;
    const STATIC_CHANNELS: &'static [&'static dyn BridgeChannelInfo<Self::Id>] = &[];
}
#[cfg(feature = "control")]
tx_channels! { pub struct AbbTxChannels: AbbTxId {
    joint_command => CuJointCommandPayload<6> = "joint_command",
    session => CuSessionCommand = "session",
} }
rx_channels! { pub struct AbbRxChannels: AbbRxId {
    joint_state => CuJointStatePayload<6> = "joint_state",
    abb_status => CuAbbStatus = "abb_status",
    session_state => CuSessionState = "session_state",
} }

#[derive(Reflect)]
pub struct CuAbbBridge {
    params: CuAbbParams,
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
impl CuAbbBridge {
    pub fn from_params(params: CuAbbParams) -> CuResult<Self> {
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
impl Drop for CuAbbBridge {
    fn drop(&mut self) {
        self.close();
    }
}
impl Freezable for CuAbbBridge {}
impl CuBridge for CuAbbBridge {
    type Tx = AbbTxChannels;
    type Rx = AbbRxChannels;
    type Resources<'r> = ();
    fn new(
        config: Option<&ComponentConfig>,
        _tx: &[BridgeChannelConfig<AbbTxId>],
        _rx: &[BridgeChannelConfig<AbbRxId>],
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
        let socket = transport::bind(&self.params)?;
        self.shared = Arc::new(Mutex::new(Shared::default()));
        self.seen = [0; 2];
        self.shutdown.store(false, Ordering::Release);
        let shared = self.shared.clone();
        let shutdown = self.shutdown.clone();
        let params = self.params.clone();
        self.worker = Some(
            std::thread::Builder::new()
                .name("abb-egm".into())
                .spawn(move || {
                    let result = transport::run(socket, &params, &shared, &shutdown);
                    if let Err(error) = &result {
                        eprintln!("ABB transport stopped: {error}");
                    }
                    if let Ok(mut state) = shared.lock() {
                        state.failed = result.is_err();
                        state.latest = None;
                        state.session.on_disconnected();
                    }
                })
                .map_err(|e| CuError::new_with_cause("ABB worker startup failed", e))?,
        );
        Ok(())
    }
    #[cfg(not(feature = "control"))]
    fn send<'a, P: CuMsgPayload + 'a>(
        &mut self,
        _ctx: &CuContext,
        _channel: &'static BridgeChannel<AbbTxId, P>,
        _msg: &CuMsg<P>,
    ) -> CuResult<()> {
        Err(CuError::from(
            "ABB monitoring bridge has no transmit channels",
        ))
    }
    #[cfg(feature = "control")]
    fn send<'a, P: CuMsgPayload + 'a>(
        &mut self,
        _ctx: &CuContext,
        channel: &'static BridgeChannel<AbbTxId, P>,
        msg: &CuMsg<P>,
    ) -> CuResult<()> {
        if self.worker.is_none() {
            return Err(CuError::from("ABB bridge is not started"));
        }
        if channel.id == AbbTxId::Session {
            let Some(&command) = msg.downcast_ref::<CuSessionCommand>()?.payload() else {
                return Ok(());
            };
            if !self.params.allow_control {
                return Err(CuError::from("ABB session requires allow_control"));
            }
            let mut state = self
                .shared
                .try_lock()
                .map_err(|_| CuError::from("ABB command state busy"))?;
            if state.failed {
                return Err(CuError::from("ABB failed; restart bridge"));
            }
            state.session.request(command);
            return Ok(());
        }
        let Some(command) = msg.downcast_ref::<CuJointCommandPayload<6>>()?.payload() else {
            return Ok(());
        };
        if !self.params.allow_motion {
            return Err(CuError::from("ABB motion requires allow_motion = true"));
        }
        if command.mode == CuControllerMode::Effort {
            return Err(CuError::from("ABB EGM does not support effort control"));
        }
        let maximum = self.params.speed.get::<radian_per_second>();
        let mut position = [0.0; 6];
        let mut velocity = [0.0; 6];
        for i in 0..6 {
            let CuJointPosition::Revolute(angle) = command.positions[i] else {
                return Err(CuError::from("ABB requires six revolute joints"));
            };
            position[i] = angle.get::<radian>();
            if command.mode == CuControllerMode::Velocity {
                let CuJointVelocity::Revolute(v) = command.velocities[i] else {
                    return Err(CuError::from("ABB requires revolute velocities"));
                };
                velocity[i] = v.get::<radian_per_second>().clamp(-maximum, maximum);
            }
            if !(position[i].is_finite() && velocity[i].is_finite()) {
                return Err(CuError::from("ABB nonfinite joint command"));
            }
        }
        let mut state = self
            .shared
            .try_lock()
            .map_err(|_| CuError::from("ABB command state busy; retry explicitly"))?;
        if state.failed {
            return Err(CuError::from("ABB failed; restart bridge"));
        }
        let session = state.session.state();
        if !(session.control_acquired && session.drives_enabled) {
            return Err(CuError::from("ABB streaming session is not enabled"));
        }
        let Some((at, status)) = &state.latest else {
            return Err(CuError::from("ABB has no feedback"));
        };
        if at.elapsed() > self.stale_after || !status.ready() {
            return Err(CuError::from("ABB feedback stale or robot not ready"));
        }
        let max_step = self.params.max_step.get::<radian>();
        if position
            .iter()
            .zip(status.joints_rad())
            .any(|(goal, actual)| (goal - actual).abs() > max_step)
        {
            return Err(CuError::from("ABB target exceeds max_step from measured"));
        }
        state.target = Some(Target {
            position,
            velocity,
            at: Instant::now(),
        });
        Ok(())
    }
    fn receive<'a, P: CuMsgPayload + 'a>(
        &mut self,
        ctx: &CuContext,
        channel: &'static BridgeChannel<AbbRxId, P>,
        msg: &mut CuMsg<P>,
    ) -> CuResult<()> {
        msg.clear_payload();
        msg.tov = Tov::None;
        if self.worker.is_none() {
            return Err(CuError::from("ABB bridge is not started"));
        }
        let mut shared = match self.shared.try_lock() {
            Ok(s) => s,
            Err(std::sync::TryLockError::WouldBlock) => return Ok(()),
            Err(_) => return Err(CuError::from("ABB state poisoned")),
        };
        if shared.failed {
            return Err(CuError::from(
                "ABB transport/protocol failed; restart bridge",
            ));
        }
        let index = match channel.id {
            AbbRxId::SessionState => {
                if shared.session.take_changed() {
                    msg.tov = Tov::Time(ctx.now());
                    msg.downcast_mut::<CuSessionState>()?
                        .set_payload(shared.session.state().clone());
                }
                return Ok(());
            }
            AbbRxId::JointState => 0,
            AbbRxId::AbbStatus => 1,
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
            AbbRxId::JointState => msg
                .downcast_mut::<CuJointStatePayload<6>>()?
                .set_payload(mapping::joint_state(status)),
            AbbRxId::AbbStatus => msg
                .downcast_mut::<CuAbbStatus>()?
                .set_payload(status.clone()),
            AbbRxId::SessionState => unreachable!(),
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
    use crate::protocol::tests::Robot;
    use std::{net::UdpSocket, time::Instant};

    fn local() -> CuAbbParams {
        CuAbbParams {
            bind: "127.0.0.1:0".into(),
            robot_ip: "127.0.0.1".into(),
            ..Default::default()
        }
    }

    #[test]
    fn publishes_each_sample_once_per_channel() {
        // Reserve a port, then let the bridge bind it.
        let port = UdpSocket::bind("127.0.0.1:0")
            .unwrap()
            .local_addr()
            .unwrap();
        let mut bridge = CuAbbBridge::from_params(CuAbbParams {
            bind: port.to_string(),
            ..local()
        })
        .unwrap();
        let (ctx, _) = CuContext::new_mock_clock();
        bridge.start(&ctx).unwrap();
        let robot = UdpSocket::bind("127.0.0.1:0").unwrap();
        robot.send_to(&Robot::default().encode(), port).unwrap();
        let deadline = Instant::now() + Duration::from_secs(2);
        while bridge.shared.lock().unwrap().generation == 0 {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(1));
        }
        let mut joints = CuMsg::<CuJointStatePayload<6>>::default();
        let mut raw = CuMsg::<CuAbbStatus>::default();
        bridge
            .receive(&ctx, &AbbRxChannels::JOINT_STATE, &mut joints)
            .unwrap();
        bridge
            .receive(&ctx, &AbbRxChannels::ABB_STATUS, &mut raw)
            .unwrap();
        assert!(joints.payload().is_some());
        assert_eq!(raw.payload().unwrap().sequence, 1);
        bridge
            .receive(&ctx, &AbbRxChannels::JOINT_STATE, &mut joints)
            .unwrap();
        assert!(joints.payload().is_none());
        // Contended state yields no sample rather than blocking.
        bridge.shared.lock().unwrap().generation += 1;
        let shared = bridge.shared.clone();
        let guard = shared.lock().unwrap();
        bridge
            .receive(&ctx, &AbbRxChannels::ABB_STATUS, &mut raw)
            .unwrap();
        assert!(raw.payload().is_none());
        drop(guard);
        bridge.stop(&ctx).unwrap();
    }

    #[cfg(feature = "control")]
    #[test]
    fn motion_gates() {
        let mut bridge = CuAbbBridge::from_params(local()).unwrap();
        bridge.worker = Some(std::thread::spawn(|| {}));
        let (ctx, _) = CuContext::new_mock_clock();
        let angle = |r| CuJointPosition::Revolute(cu29::units::si::f64::Angle::new::<radian>(r));
        let mut command = CuJointCommandPayload::<6> {
            mode: CuControllerMode::Velocity,
            positions: [angle(0.1); 6],
            ..Default::default()
        };
        let mut msg = CuMsg::default();
        msg.set_payload(command.clone());
        let send = |bridge: &mut CuAbbBridge, msg: &CuMsg<CuJointCommandPayload<6>>| {
            bridge.send(&ctx, &AbbTxChannels::JOINT_COMMAND, msg)
        };
        assert!(send(&mut bridge, &msg).is_err()); // allow_motion off
        bridge.params.allow_control = true;
        bridge.params.allow_motion = true;
        assert!(send(&mut bridge, &msg).is_err()); // no session
        {
            let mut shared = bridge.shared.lock().unwrap();
            shared.session.on_control_report(true);
            shared.session.request(CuSessionCommand::EnableDrives);
            let out = shared.session.take_outgoing().unwrap();
            shared.session.mark_sent(out, 1);
            shared.session.on_acknowledged(1);
            shared.latest = Some((
                Instant::now(),
                CuAbbStatus {
                    motors: crate::CuAbbMotorState::On,
                    rapid: crate::CuAbbRapidState::Running,
                    ..Default::default()
                },
            ));
        }
        send(&mut bridge, &msg).unwrap();
        assert_eq!(
            bridge.shared.lock().unwrap().target.unwrap().position,
            [0.1; 6]
        );
        command.positions[2] = angle(1.0);
        msg.set_payload(command.clone());
        assert!(send(&mut bridge, &msg).is_err()); // max_step
        command.positions[2] = angle(0.1);
        command.mode = CuControllerMode::Effort;
        msg.set_payload(command);
        assert!(send(&mut bridge, &msg).is_err());
    }
}
