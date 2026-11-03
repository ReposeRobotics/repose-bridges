use crate::{
    mapping,
    params::{CuUrParams, duration},
    protocol::CuUrStatus,
    transport::{self, Shared},
};
use cu29::prelude::*;
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
#[cfg(feature = "control")]
use {
    cu29::units::si::{angle::radian, angular_velocity::radian_per_second},
    repose_controller::{CuControllerMode, CuJointCommandPayload},
    repose_kinematics::CuJointPosition,
};

#[cfg(not(feature = "control"))]
pub struct UrTxChannels;
#[cfg(not(feature = "control"))]
pub type UrTxId = core::convert::Infallible;
#[cfg(not(feature = "control"))]
impl BridgeChannelSet for UrTxChannels {
    type Id = UrTxId;
    const STATIC_CHANNELS: &'static [&'static dyn BridgeChannelInfo<Self::Id>] = &[];
}
#[cfg(feature = "control")]
tx_channels! { pub struct UrTxChannels: UrTxId { joint_command => CuJointCommandPayload<6> = "joint_command", session => CuSessionCommand = "session", } }
rx_channels! { pub struct UrRxChannels: UrRxId {
    joint_state => CuJointStatePayload<6> = "joint_state",
    ur_status => CuUrStatus = "ur_status",
    session_state => CuSessionState = "session_state",
} }

#[derive(Reflect)]
pub struct CuUrBridge {
    params: CuUrParams,
    seen: [u64; 2],
    #[reflect(ignore)]
    shared: Arc<Mutex<Shared>>,
    #[reflect(ignore)]
    shutdown: Arc<AtomicBool>,
    #[reflect(ignore)]
    worker: Option<JoinHandle<()>>,
    #[reflect(ignore)]
    stale_after: Duration,
    /// PID state and the time of the last velocity command it saw.
    #[cfg(feature = "control")]
    #[reflect(ignore)]
    pid: repose_joint_pid::JointPid<6>,
    #[cfg(feature = "control")]
    #[reflect(ignore)]
    last_velocity: Option<Instant>,
}
impl CuUrBridge {
    pub fn from_params(params: CuUrParams) -> CuResult<Self> {
        params.validate()?;
        Ok(Self {
            stale_after: duration(params.stale_after)?,
            #[cfg(feature = "control")]
            pid: repose_joint_pid::JointPid::new(params.joint_gains),
            #[cfg(feature = "control")]
            last_velocity: None,
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
impl Drop for CuUrBridge {
    fn drop(&mut self) {
        self.close();
    }
}
impl Freezable for CuUrBridge {}
impl CuBridge for CuUrBridge {
    type Tx = UrTxChannels;
    type Rx = UrRxChannels;
    type Resources<'r> = ();
    fn new(
        config: Option<&ComponentConfig>,
        _tx: &[BridgeChannelConfig<UrTxId>],
        _rx: &[BridgeChannelConfig<UrRxId>],
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
        #[cfg(feature = "control")]
        {
            self.last_velocity = None;
        }
        let (stream, recipe, input) = transport::connect(&self.params)?;
        self.shared = Arc::new(Mutex::new(Shared::default()));
        self.seen = [0; 2];
        self.shutdown.store(false, Ordering::Release);
        let shared = self.shared.clone();
        let shutdown = self.shutdown.clone();
        let params = self.params.clone();
        self.worker = Some(
            std::thread::Builder::new()
                .name("ur-rtde".into())
                .spawn(move || {
                    let result = transport::run(stream, recipe, input, &params, &shared, &shutdown);
                    if let Err(error) = &result {
                        eprintln!("UR transport stopped: {error}");
                    }
                    if let Ok(mut state) = shared.lock() {
                        state.failed = result.is_err();
                        state.latest = None;
                        state.session.on_disconnected();
                    }
                })
                .map_err(|e| CuError::new_with_cause("UR worker startup failed", e))?,
        );
        Ok(())
    }
    #[cfg(not(feature = "control"))]
    fn send<'a, P: CuMsgPayload + 'a>(
        &mut self,
        _ctx: &CuContext,
        _channel: &'static BridgeChannel<UrTxId, P>,
        _msg: &CuMsg<P>,
    ) -> CuResult<()> {
        Err(CuError::from(
            "UR monitoring bridge has no transmit channels",
        ))
    }
    #[cfg(feature = "control")]
    fn send<'a, P: CuMsgPayload + 'a>(
        &mut self,
        _ctx: &CuContext,
        channel: &'static BridgeChannel<UrTxId, P>,
        msg: &CuMsg<P>,
    ) -> CuResult<()> {
        if channel.id == UrTxId::Session {
            let Some(&command) = msg.downcast_ref::<CuSessionCommand>()?.payload() else {
                return Ok(());
            };
            if !self.params.allow_control {
                return Err(CuError::from("UR session requires allow_control"));
            }
            if self.worker.is_none() {
                return Err(CuError::from("UR bridge not started"));
            }
            let mut state = self
                .shared
                .try_lock()
                .map_err(|_| CuError::from("UR command state busy"))?;
            if state.failed {
                return Err(CuError::from("UR failed; restart bridge"));
            }
            if state.pending.is_some() || state.active.is_some() {
                return Err(CuError::from("UR point move in progress"));
            }
            state.session.request(command);
            return Ok(());
        }
        let Some(command) = msg.downcast_ref::<CuJointCommandPayload<6>>()?.payload() else {
            return Ok(());
        };
        if !self.params.allow_motion {
            return Err(CuError::from("UR motion requires allow_motion = true"));
        }
        if self.worker.is_none() {
            return Err(CuError::from("UR bridge is not started"));
        }
        if command.mode == CuControllerMode::Effort {
            return Err(CuError::from("UR does not support effort control"));
        }
        let mut target = [0.0; 6];
        for (out, position) in target.iter_mut().zip(command.positions) {
            let CuJointPosition::Revolute(angle) = position else {
                return Err(CuError::from("UR requires six revolute joints"));
            };
            *out = angle.get::<radian>();
            if !out.is_finite() {
                return Err(CuError::from("UR nonfinite joint target"));
            }
        }
        let mut state = self
            .shared
            .try_lock()
            .map_err(|_| CuError::from("UR command state busy or poisoned; retry explicitly"))?;
        if state.failed {
            return Err(CuError::from("UR failed; restart bridge"));
        }
        let Some((at, status)) = &state.latest else {
            return Err(CuError::from("UR has no telemetry"));
        };
        if at.elapsed() > self.stale_after || !status.ready() {
            return Err(CuError::from("UR telemetry stale or robot not ready"));
        }
        if command.mode == CuControllerMode::Velocity {
            if !state.session.state().control_acquired || !state.session.state().motion_allowed {
                return Err(CuError::from("UR streaming session is not ready"));
            }
            if state.pending.is_some() || state.active.is_some() {
                return Err(CuError::from("UR point move in progress"));
            }
            let maximum = self.params.speed.get::<radian_per_second>();
            let mut feedforward = [0.0; 6];
            let mut error = [0.0; 6];
            for i in 0..6 {
                let repose_kinematics::CuJointVelocity::Revolute(v) = command.velocities[i] else {
                    return Err(CuError::from("UR requires revolute velocities"));
                };
                feedforward[i] = v.get::<radian_per_second>();
                error[i] = target[i] - status.joints[i].get::<radian>();
                if !feedforward[i].is_finite()
                    || error[i].abs() > self.params.max_step.get::<radian>()
                {
                    return Err(CuError::from(
                        "UR invalid velocity or tracking error exceeds max_step",
                    ));
                }
            }
            // A gap longer than the command timeout restarts the PID.
            let now = Instant::now();
            let timeout = duration(self.params.command_timeout)?;
            let dt = self
                .last_velocity
                .map(|t| now.duration_since(t))
                .filter(|gap| *gap <= timeout)
                .map_or(0.0, |gap| gap.as_secs_f64());
            self.last_velocity = Some(now);
            let feedback = self.pid.update(error, dt, maximum);
            let velocity: [f64; 6] =
                std::array::from_fn(|i| (feedforward[i] + feedback[i]).clamp(-maximum, maximum));
            state.velocity = Some((velocity, Instant::now()));
            return Ok(());
        }
        if state.session.state().control_acquired {
            return Err(CuError::from(
                "UR position move unavailable during streaming session",
            ));
        }
        if state.last_target == Some(target) {
            return Ok(());
        }
        if state.pending.is_some() || state.active.is_some() || status.runtime_state != 1 {
            return Err(CuError::from("UR move or another program in progress"));
        }
        if status
            .velocities
            .iter()
            .any(|v| v.get::<radian_per_second>().abs() >= 0.001)
        {
            return Err(CuError::from("UR must be stationary before movej"));
        }
        if status.joints.iter().zip(target).any(|(actual, goal)| {
            (actual.get::<radian>() - goal).abs() > self.params.max_step.get::<radian>()
        }) {
            return Err(CuError::from("UR target exceeds max_step"));
        }
        state.pending = Some((target, Instant::now()));
        state.last_target = Some(target);
        Ok(())
    }
    fn receive<'a, P: CuMsgPayload + 'a>(
        &mut self,
        ctx: &CuContext,
        channel: &'static BridgeChannel<UrRxId, P>,
        msg: &mut CuMsg<P>,
    ) -> CuResult<()> {
        msg.clear_payload();
        msg.tov = Tov::None;
        if self.worker.is_none() {
            return Err(CuError::from("UR bridge is not started"));
        }
        let mut shared = match self.shared.try_lock() {
            Ok(s) => s,
            Err(std::sync::TryLockError::WouldBlock) => return Ok(()),
            Err(_) => return Err(CuError::from("UR state poisoned")),
        };
        if shared.failed {
            return Err(CuError::from(
                "UR transport/protocol failed; restart bridge",
            ));
        }
        if channel.id == UrRxId::SessionState {
            if shared.session.take_changed() {
                msg.tov = Tov::Time(ctx.now());
                msg.downcast_mut::<CuSessionState>()?
                    .set_payload(shared.session.state().clone());
            }
            return Ok(());
        }
        let index = match channel.id {
            UrRxId::JointState => 0,
            UrRxId::UrStatus => 1,
            UrRxId::SessionState => unreachable!(),
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
            UrRxId::JointState => msg
                .downcast_mut::<CuJointStatePayload<6>>()?
                .set_payload(mapping::joint_state(status)),
            UrRxId::UrStatus => msg
                .downcast_mut::<CuUrStatus>()?
                .set_payload(status.clone()),
            UrRxId::SessionState => unreachable!(),
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
    use std::{
        io::{Read, Write},
        net::TcpListener,
        time::Instant,
    };
    fn request(peer: &mut std::net::TcpStream, kind: u8) -> Vec<u8> {
        let mut header = [0; 3];
        peer.read_exact(&mut header).unwrap();
        assert_eq!(header[2], kind);
        let size = u16::from_be_bytes([header[0], header[1]]) as usize;
        let mut data = vec![0; size - 3];
        peer.read_exact(&mut data).unwrap();
        data
    }
    fn packet(peer: &mut std::net::TcpStream, kind: u8, data: &[u8]) {
        peer.write_all(&((data.len() + 3) as u16).to_be_bytes())
            .unwrap();
        peer.write_all(&[kind]).unwrap();
        peer.write_all(data).unwrap();
    }
    #[test]
    fn fragmented_transport_and_independent_channels() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let address = listener.local_addr().unwrap().to_string();
        let peer = std::thread::spawn(move || {
            let (mut peer, _) = listener.accept().unwrap();
            peer.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
            assert_eq!(request(&mut peer, b'V'), [0, 2]);
            packet(&mut peer, b'V', &[1]);
            let recipe = request(&mut peer, b'O');
            assert_eq!(&recipe[8..], crate::protocol::FIELDS);
            let mut reply = vec![7];
            reply.extend_from_slice(crate::protocol::TYPES);
            packet(&mut peer, b'O', &reply);
            assert!(request(&mut peer, b'S').is_empty());
            packet(&mut peer, b'S', &[1]);
            let mut body = vec![7];
            for n in 0..13 {
                body.extend_from_slice(&(n as f64).to_be_bytes());
            }
            body.extend_from_slice(&7i32.to_be_bytes());
            body.extend_from_slice(&1i32.to_be_bytes());
            body.extend_from_slice(&1u32.to_be_bytes());
            body.extend_from_slice(&0i32.to_be_bytes());
            let mut frame = vec![0, 124, b'U'];
            frame.extend_from_slice(&body);
            for part in frame.chunks(7) {
                peer.write_all(part).unwrap();
                std::thread::sleep(Duration::from_millis(1));
            }
            let mut sink = [0];
            let _ = peer.read(&mut sink);
        });
        let mut bridge = CuUrBridge::from_params(CuUrParams {
            address,
            ..Default::default()
        })
        .unwrap();
        let (ctx, _) = CuContext::new_mock_clock();
        bridge.start(&ctx).unwrap();
        let deadline = Instant::now() + Duration::from_secs(2);
        while bridge.shared.lock().unwrap().generation == 0 {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(2));
        }
        let mut joints = CuMsg::<CuJointStatePayload<6>>::default();
        let mut raw = CuMsg::<CuUrStatus>::default();
        bridge
            .receive(&ctx, &UrRxChannels::JOINT_STATE, &mut joints)
            .unwrap();
        bridge
            .receive(&ctx, &UrRxChannels::UR_STATUS, &mut raw)
            .unwrap();
        assert!(joints.payload().is_some());
        assert_eq!(raw.payload().unwrap().robot_mode, 7);
        bridge
            .receive(&ctx, &UrRxChannels::JOINT_STATE, &mut joints)
            .unwrap();
        assert!(joints.payload().is_none());
        let shared = bridge.shared.clone();
        let guard = shared.lock().unwrap();
        bridge
            .receive(&ctx, &UrRxChannels::UR_STATUS, &mut raw)
            .unwrap();
        assert!(raw.payload().is_none());
        drop(guard);
        bridge.stop(&ctx).unwrap();
        peer.join().unwrap();
    }
    #[cfg(feature = "control")]
    #[test]
    fn motion_gates_and_single_slot() {
        let mut bridge = CuUrBridge::from_params(Default::default()).unwrap();
        bridge.worker = Some(std::thread::spawn(|| {}));
        let (ctx, _) = CuContext::new_mock_clock();
        let mut msg = CuMsg::default();
        let mut command = CuJointCommandPayload::<6> {
            positions: [CuJointPosition::Revolute(cu29::units::si::f64::Angle::new::<radian>(0.1));
                6],
            ..Default::default()
        };
        msg.set_payload(command.clone());
        assert!(
            bridge
                .send(&ctx, &UrTxChannels::JOINT_COMMAND, &msg)
                .is_err()
        );
        bridge.params.allow_motion = true;
        bridge.params.allow_control = true;
        assert!(
            bridge
                .send(&ctx, &UrTxChannels::JOINT_COMMAND, &msg)
                .is_err()
        );
        bridge.shared.lock().unwrap().latest = Some((
            Instant::now(),
            CuUrStatus {
                robot_mode: 7,
                safety_mode: 1,
                runtime_state: 1,
                ..Default::default()
            },
        ));
        command.mode = CuControllerMode::Velocity;
        msg.set_payload(command.clone());
        assert!(
            bridge
                .send(&ctx, &UrTxChannels::JOINT_COMMAND, &msg)
                .is_err()
        );
        command.mode = CuControllerMode::Position;
        msg.set_payload(command.clone());
        bridge
            .send(&ctx, &UrTxChannels::JOINT_COMMAND, &msg)
            .unwrap();
        bridge
            .send(&ctx, &UrTxChannels::JOINT_COMMAND, &msg)
            .unwrap(); // duplicate is idempotent
        command.positions[0] =
            CuJointPosition::Revolute(cu29::units::si::f64::Angle::new::<radian>(0.2));
        msg.set_payload(command);
        assert!(
            bridge
                .send(&ctx, &UrTxChannels::JOINT_COMMAND, &msg)
                .is_err()
        );
    }
    #[cfg(feature = "control")]
    #[test]
    fn streamed_velocity_uses_standard_session_and_caps_feedback() {
        let mut bridge = CuUrBridge::from_params(CuUrParams {
            allow_control: true,
            allow_motion: true,
            ..Default::default()
        })
        .unwrap();
        bridge.worker = Some(std::thread::spawn(|| {}));
        {
            let mut shared = bridge.shared.lock().unwrap();
            shared.latest = Some((
                Instant::now(),
                CuUrStatus {
                    robot_mode: 7,
                    safety_mode: 1,
                    runtime_state: 2,
                    ..Default::default()
                },
            ));
            shared.session.on_control_report(true);
            shared.session.on_drive_report(true, true);
        }
        let command = CuJointCommandPayload::<6> {
            mode: CuControllerMode::Velocity,
            positions: [CuJointPosition::Revolute(cu29::units::si::f64::Angle::new::<radian>(0.2));
                6],
            ..Default::default()
        };
        let (ctx, _) = CuContext::new_mock_clock();
        let mut msg = CuMsg::default();
        msg.set_payload(command);
        bridge
            .send(&ctx, &UrTxChannels::JOINT_COMMAND, &msg)
            .unwrap();
        assert_eq!(bridge.shared.lock().unwrap().velocity.unwrap().0, [0.1; 6]);
        bridge.shared.lock().unwrap().session.on_disconnected();
        assert!(
            bridge
                .send(&ctx, &UrTxChannels::JOINT_COMMAND, &msg)
                .is_err()
        );
    }
}
