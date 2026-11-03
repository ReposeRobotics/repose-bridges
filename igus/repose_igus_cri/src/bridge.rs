//! The `CuBridge`: standard Repose channels plus the vendor `cri_status` channel.

#[cfg(feature = "control")]
use crate::jog::JogLimits;
#[cfg(feature = "control")]
use crate::jog::jog_percent;
use crate::mapping;
#[cfg(feature = "control")]
use crate::motion::MoveContext;
use crate::params::CuIgusCriParams;
use crate::params::duration;
use crate::protocol::CuCriMotionState;
use crate::protocol::CuCriStatus;
use crate::transport;
use crate::transport::Shared;
use cu29::prelude::*;
#[cfg(feature = "control")]
use repose_controller::CuControllerMode;
#[cfg(feature = "control")]
use repose_controller::CuJointCommandPayload;
#[cfg(feature = "control")]
use repose_kinematics::CuJointPosition;
use repose_kinematics::CuJointStatePayload;
#[cfg(feature = "control")]
use repose_robot_interface::CuSessionCommand;
use repose_robot_interface::CuSessionState;
use std::net::TcpStream;
use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;
use std::thread::JoinHandle;
use std::time::Duration;
use std::time::Instant;

/// How long `stop` waits for each wind-down command.
const WIND_DOWN: Duration = Duration::from_millis(500);

/// Empty transmit set: no command channel can be constructed.
#[cfg(not(feature = "control"))]
pub struct IgusCriTxChannels;
/// Uninhabited identifier: without `control` the bridge has no Tx channel.
#[cfg(not(feature = "control"))]
pub type IgusCriTxId = core::convert::Infallible;
#[cfg(not(feature = "control"))]
impl BridgeChannelSet for IgusCriTxChannels {
    type Id = IgusCriTxId;
    const STATIC_CHANNELS: &'static [&'static dyn BridgeChannelInfo<Self::Id>] = &[];
}
#[cfg(feature = "control")]
tx_channels! {
    pub struct IgusCriTxChannels: IgusCriTxId {
        session => CuSessionCommand = "session",
        joint_command => CuJointCommandPayload<6> = "joint_command",
    }
}
rx_channels! {
    pub struct IgusCriRxChannels: IgusCriRxId {
        joint_state => CuJointStatePayload<6> = "joint_state",
        session_state => CuSessionState = "session_state",
        cri_status => CuCriStatus = "cri_status",
        cri_motion => CuCriMotionState = "cri_motion",
    }
}

/// Copper bridge to an igus ReBeL over CRI. Call `start` before `receive`.
///
/// Each `STATUS` frame is published once per Rx channel. Empty, contended, or
/// stale slots clear the output. Transport/protocol failure returns an error;
/// restart explicitly to reconnect. External I/O is recorded through Copper
/// messages, so freeze/thaw is a no-op.
#[derive(Reflect)]
pub struct CuIgusCriBridge {
    params: CuIgusCriParams,
    seen_joint_state: u64,
    seen_cri_status: u64,
    #[reflect(ignore)]
    shared: Arc<Mutex<Shared>>,
    #[reflect(ignore)]
    shutdown: Arc<AtomicBool>,
    #[reflect(ignore)]
    worker: Option<JoinHandle<()>>,
    #[reflect(ignore)]
    stale_after: Duration,
    #[cfg(feature = "control")]
    #[reflect(ignore)]
    jog_limits: JogLimits,
    #[cfg(feature = "control")]
    #[reflect(ignore)]
    command_timeout: Duration,
    /// PID state and the time of the last velocity command it saw.
    #[cfg(feature = "control")]
    #[reflect(ignore)]
    pid: repose_joint_pid::JointPid<6>,
    #[cfg(feature = "control")]
    #[reflect(ignore)]
    last_velocity: Option<Instant>,
}

impl CuIgusCriBridge {
    /// Construct without opening a connection; `start` owns connection setup.
    pub fn from_params(params: CuIgusCriParams) -> CuResult<Self> {
        params.validate()?;
        let stale_after = duration(params.stale_after)?;
        #[cfg(feature = "control")]
        let jog_limits = JogLimits {
            full_scale: params.jog_full_scale,
            max_percent: params.max_jog_percent,
        };
        #[cfg(feature = "control")]
        let pid = repose_joint_pid::JointPid::new(params.joint_gains);
        #[cfg(feature = "control")]
        let command_timeout = duration(params.command_timeout)?;
        Ok(Self {
            #[cfg(feature = "control")]
            jog_limits,
            #[cfg(feature = "control")]
            command_timeout,
            #[cfg(feature = "control")]
            pid,
            #[cfg(feature = "control")]
            last_velocity: None,
            params,
            seen_joint_state: 0,
            seen_cri_status: 0,
            shared: Arc::new(Mutex::new(Shared::default())),
            shutdown: Arc::new(AtomicBool::new(false)),
            worker: None,
            stale_after,
        })
    }

    fn running(&self) -> CuResult<()> {
        if self.worker.is_none() {
            return Err(CuError::from("CRI bridge is not started"));
        }
        Ok(())
    }

    /// Undo what this session changed (drives, then control), each step
    /// bounded by [`WIND_DOWN`], then join the worker.
    fn close(&mut self) {
        for _ in 0..2 {
            let queued =
                self.shared
                    .lock()
                    .is_ok_and(|mut shared| match shared.session.shutdown_step() {
                        Some(step) if !shared.failed && !shared.session.is_busy() => {
                            shared.session.request(step);
                            true
                        }
                        _ => false,
                    });
            if !queued {
                break;
            }
            let deadline = Instant::now() + WIND_DOWN;
            while Instant::now() < deadline
                && self.worker.as_ref().is_some_and(|w| !w.is_finished())
                && self.shared.lock().is_ok_and(|s| s.session.is_busy())
            {
                std::thread::sleep(Duration::from_millis(5));
            }
        }
        self.shutdown.store(true, Ordering::Release);
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }
}

impl Drop for CuIgusCriBridge {
    fn drop(&mut self) {
        self.close();
    }
}

impl Freezable for CuIgusCriBridge {}

impl CuBridge for CuIgusCriBridge {
    type Tx = IgusCriTxChannels;
    type Rx = IgusCriRxChannels;
    type Resources<'r> = ();

    fn new(
        config: Option<&ComponentConfig>,
        _tx: &[BridgeChannelConfig<IgusCriTxId>],
        _rx: &[BridgeChannelConfig<IgusCriRxId>],
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
        #[cfg(feature = "control")]
        {
            self.last_velocity = None;
        }
        self.close();
        let stream = TcpStream::connect_timeout(
            &self.params.socket_address()?,
            duration(self.params.connect_timeout)?,
        )
        .map_err(|e| CuError::new_with_cause("CRI connection failed", e))?;
        stream
            .set_read_timeout(Some(transport::POLL))
            .map_err(|e| CuError::new_with_cause("CRI read timeout setup failed", e))?;
        stream
            .set_write_timeout(Some(Duration::from_millis(100)))
            .map_err(|e| CuError::new_with_cause("CRI write timeout setup failed", e))?;
        self.shared = Arc::new(Mutex::new(Shared {
            velocity_percent: self.params.velocity_percent,
            goal_tolerance: self.params.goal_tolerance,
            ..Shared::default()
        }));
        self.seen_joint_state = 0;
        self.seen_cri_status = 0;
        self.shutdown.store(false, Ordering::Release);
        let shared = Arc::clone(&self.shared);
        let shutdown = Arc::clone(&self.shutdown);
        self.worker = Some(
            std::thread::Builder::new()
                .name("igus-cri".into())
                .spawn(move || {
                    if transport::run(stream, &shared, &shutdown).is_err()
                        && let Ok(mut shared) = shared.lock()
                    {
                        shared.fail();
                    }
                })
                .map_err(|e| CuError::new_with_cause("CRI worker startup failed", e))?,
        );
        Ok(())
    }

    #[cfg(not(feature = "control"))]
    fn send<'a, P: CuMsgPayload + 'a>(
        &mut self,
        _ctx: &CuContext,
        _channel: &'static BridgeChannel<IgusCriTxId, P>,
        _msg: &CuMsg<P>,
    ) -> CuResult<()> {
        Err(CuError::from("CRI bridge has no transmit channels"))
    }

    /// Queue a session command. Refusals (busy, not in control) are reported
    /// on `session_state`, not as errors.
    #[cfg(feature = "control")]
    fn send<'a, P: CuMsgPayload + 'a>(
        &mut self,
        _ctx: &CuContext,
        channel: &'static BridgeChannel<IgusCriTxId, P>,
        msg: &CuMsg<P>,
    ) -> CuResult<()> {
        match channel.id {
            IgusCriTxId::Session => {
                let Some(&command) = msg.downcast_ref::<CuSessionCommand>()?.payload() else {
                    return Ok(());
                };
                if !self.params.allow_control {
                    return Err(CuError::from(
                        "CRI control requires allow_control = true in the bridge config",
                    ));
                }
                self.running()?;
                let mut shared = self
                    .shared
                    .lock()
                    .map_err(|_| CuError::from("CRI worker state poisoned"))?;
                if shared.failed {
                    return Err(CuError::from(
                        "CRI transport or protocol failed; restart bridge",
                    ));
                }
                shared.session.request(command);
                Ok(())
            }
            IgusCriTxId::JointCommand => {
                let Some(command) = msg.downcast_ref::<CuJointCommandPayload<6>>()?.payload()
                else {
                    return Ok(());
                };
                if !self.params.allow_motion {
                    return Err(CuError::from(
                        "CRI motion requires allow_motion = true in the bridge config",
                    ));
                }
                if command.mode == CuControllerMode::Velocity {
                    self.running()?;
                    let mut shared = self
                        .shared
                        .lock()
                        .map_err(|_| CuError::from("CRI worker state poisoned"))?;
                    if shared.failed {
                        return Err(CuError::from(
                            "CRI transport or protocol failed; restart bridge",
                        ));
                    }
                    // Without a fresh measurement there is no feedback: send nothing.
                    let Some((_, measured)) = shared.latest.as_ref() else {
                        return Ok(());
                    };
                    // A gap longer than the command timeout restarts the PID.
                    let now = Instant::now();
                    let dt = self
                        .last_velocity
                        .map(|t| now.duration_since(t))
                        .filter(|gap| *gap <= self.command_timeout)
                        .map_or(0.0, |gap| gap.as_secs_f64());
                    self.last_velocity = Some(now);
                    let jog = jog_percent(command, measured, &self.jog_limits, &mut self.pid, dt)
                        .map_err(CuError::from)?;
                    shared.jog = jog;
                    shared.jog_until = Some(Instant::now() + self.command_timeout);
                    return Ok(());
                }
                if command.mode != CuControllerMode::Position {
                    return Err(CuError::from(
                        "CRI bridge supports position (Move Joint) and velocity (jog) commands",
                    ));
                }
                let mut target = [cu29::units::si::f64::Angle::default(); 6];
                for (out, position) in target.iter_mut().zip(command.positions) {
                    let CuJointPosition::Revolute(angle) = position else {
                        return Err(CuError::from("ReBeL joints are revolute"));
                    };
                    *out = angle;
                }
                self.running()?;
                let mut shared = self
                    .shared
                    .lock()
                    .map_err(|_| CuError::from("CRI worker state poisoned"))?;
                if shared.failed {
                    return Err(CuError::from(
                        "CRI transport or protocol failed; restart bridge",
                    ));
                }
                let Shared {
                    session,
                    motion,
                    latest,
                    ..
                } = &mut *shared;
                motion.request(
                    target,
                    &MoveContext {
                        session: session.state(),
                        measured: latest.as_ref().map(|(_, status)| status),
                        max_step: self.params.max_step,
                    },
                );
                Ok(())
            }
        }
    }

    fn receive<'a, P: CuMsgPayload + 'a>(
        &mut self,
        ctx: &CuContext,
        channel: &'static BridgeChannel<IgusCriRxId, P>,
        msg: &mut CuMsg<P>,
    ) -> CuResult<()> {
        msg.clear_payload();
        msg.tov = Tov::None;
        self.running()?;
        let mut shared = match self.shared.try_lock() {
            Ok(shared) => shared,
            Err(std::sync::TryLockError::WouldBlock) => return Ok(()),
            Err(_) => return Err(CuError::from("CRI worker state poisoned")),
        };
        if shared.failed {
            return Err(CuError::from(
                "CRI transport or protocol failed; restart bridge",
            ));
        }
        match channel.id {
            IgusCriRxId::JointState => {
                let output = msg.downcast_mut::<CuJointStatePayload<6>>()?;
                if let Some((received, status)) =
                    fresh(&shared, &mut self.seen_joint_state, self.stale_after)
                {
                    let mut state = CuJointStatePayload::<6>::default();
                    mapping::joint_state(status, &mut state);
                    stamp(ctx, received, output);
                    output.set_payload(state);
                }
            }
            IgusCriRxId::CriStatus => {
                let output = msg.downcast_mut::<CuCriStatus>()?;
                if let Some((received, status)) =
                    fresh(&shared, &mut self.seen_cri_status, self.stale_after)
                {
                    stamp(ctx, received, output);
                    output.set_payload(status.clone());
                }
            }
            IgusCriRxId::CriMotion => {
                let output = msg.downcast_mut::<CuCriMotionState>()?;
                if shared.motion.take_changed() {
                    output.tov = Tov::Time(ctx.now());
                    output.set_payload(shared.motion.state().clone());
                }
            }
            IgusCriRxId::SessionState => {
                let output = msg.downcast_mut::<CuSessionState>()?;
                if shared.session.take_changed() {
                    output.tov = Tov::Time(ctx.now());
                    output.set_payload(shared.session.state().clone());
                }
            }
        }
        Ok(())
    }

    fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
        self.close();
        Ok(())
    }
}

/// The latest status if this channel has not seen it and it is fresh.
fn fresh<'s>(
    shared: &'s Shared,
    seen: &mut u64,
    stale_after: Duration,
) -> Option<(Instant, &'s CuCriStatus)> {
    if *seen == shared.generation {
        return None;
    }
    *seen = shared.generation;
    let (received, status) = shared.latest.as_ref()?;
    (received.elapsed() <= stale_after).then_some((*received, status))
}

/// Receipt-time estimate; CRI does not provide a measurement timestamp.
fn stamp<P: CuMsgPayload>(ctx: &CuContext, received: Instant, output: &mut CuMsg<P>) {
    let age = received.elapsed();
    output.tov =
        Tov::Time(ctx.now() - CuDuration::from_nanos(age.as_nanos().min(u64::MAX as u128) as u64));
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::tests::STATUS;
    use crate::protocol::tests::status;
    use crate::transport::FRAME_CAPACITY;
    use repose_kinematics::CuJointPosition;
    use std::io::Write;
    use std::net::TcpListener;

    fn parked() -> CuIgusCriBridge {
        let mut bridge = CuIgusCriBridge::from_params(Default::default()).unwrap();
        // A parked worker models the live lifecycle without a socket.
        bridge.worker = Some(std::thread::spawn(|| {}));
        bridge
    }

    fn publish(bridge: &CuIgusCriBridge, received: Instant) {
        let mut shared = bridge.shared.lock().unwrap();
        shared.latest = Some((received, status(STATUS).unwrap().unwrap()));
        shared.generation += 1;
    }

    #[test]
    fn test_channels_consume_independently() {
        let mut bridge = parked();
        publish(&bridge, Instant::now());
        let (ctx, _) = CuContext::new_mock_clock();
        let mut joints = CuMsg::<CuJointStatePayload<6>>::default();
        let mut raw = CuMsg::<CuCriStatus>::default();
        bridge
            .receive(&ctx, &IgusCriRxChannels::JOINT_STATE, &mut joints)
            .unwrap();
        bridge
            .receive(&ctx, &IgusCriRxChannels::CRI_STATUS, &mut raw)
            .unwrap();
        let raw_joints = raw.payload().unwrap().joints;
        assert_eq!(
            joints.payload().unwrap().positions,
            raw_joints.map(CuJointPosition::Revolute)
        );
        assert!(raw.payload().unwrap().cartesian.is_some());
        bridge
            .receive(&ctx, &IgusCriRxChannels::CRI_STATUS, &mut raw)
            .unwrap();
        assert!(raw.payload().is_none());
    }

    #[test]
    fn test_receive_consumes_and_rejects_stale_samples() {
        let mut bridge = parked();
        let (ctx, _) = CuContext::new_mock_clock();
        let mut msg = CuMsg::<CuJointStatePayload<6>>::default();
        publish(&bridge, Instant::now());
        bridge
            .receive(&ctx, &IgusCriRxChannels::JOINT_STATE, &mut msg)
            .unwrap();
        assert!(msg.payload().is_some());
        bridge
            .receive(&ctx, &IgusCriRxChannels::JOINT_STATE, &mut msg)
            .unwrap();
        assert!(msg.payload().is_none());
        publish(&bridge, Instant::now() - Duration::from_secs(1));
        bridge
            .receive(&ctx, &IgusCriRxChannels::JOINT_STATE, &mut msg)
            .unwrap();
        assert!(msg.payload().is_none());
        bridge.shared.lock().unwrap().failed = true;
        assert!(
            bridge
                .receive(&ctx, &IgusCriRxChannels::JOINT_STATE, &mut msg)
                .is_err()
        );
    }

    #[test]
    fn test_transport_failure_and_frame_capacity() {
        for oversized in [false, true] {
            let listener = TcpListener::bind("127.0.0.1:0").unwrap();
            let mut bridge = CuIgusCriBridge::from_params(CuIgusCriParams {
                address: listener.local_addr().unwrap().to_string(),
                ..Default::default()
            })
            .unwrap();
            let (ctx, _) = CuContext::new_mock_clock();
            bridge.start(&ctx).unwrap();
            let (mut peer, _) = listener.accept().unwrap();
            if oversized {
                peer.write_all(&[b'x'; FRAME_CAPACITY + 1]).unwrap();
            } else {
                drop(peer);
            }
            let deadline = Instant::now() + Duration::from_secs(2);
            while !bridge.shared.lock().unwrap().failed {
                assert!(Instant::now() < deadline);
                std::thread::sleep(Duration::from_millis(2));
            }
            let mut msg = CuMsg::<CuJointStatePayload<6>>::default();
            assert!(
                bridge
                    .receive(&ctx, &IgusCriRxChannels::JOINT_STATE, &mut msg)
                    .is_err()
            );
            bridge.stop(&ctx).unwrap();
        }
    }

    #[cfg(feature = "control")]
    mod control {
        use super::*;
        use repose_controller::CuControllerMode;
        use repose_controller::CuJointCommandPayload;
        use repose_robot_interface::CuSessionRejection;
        use repose_robot_interface::CuSessionResult;
        use std::io::Read;

        fn read_line(peer: &mut TcpStream) -> String {
            let mut line = Vec::new();
            loop {
                let mut byte = [0];
                peer.read_exact(&mut byte).unwrap();
                if byte[0] == b'\n' {
                    return String::from_utf8(line).unwrap();
                }
                line.push(byte[0]);
            }
        }

        /// Next `CMD` line as (counter, command text), skipping heartbeats.
        fn next_command(peer: &mut TcpStream) -> (u16, String) {
            loop {
                let line = read_line(peer);
                assert!(!line.contains("ALIVEJOG") || line.contains("ALIVEJOG 0 0 0 0 0 0 0 0 0"));
                if let Some((head, command)) = line.split_once(" CMD ") {
                    let counter = head
                        .split_ascii_whitespace()
                        .nth(1)
                        .unwrap()
                        .parse()
                        .unwrap();
                    return (counter, command.trim_end_matches(" CRIEND").to_string());
                }
            }
        }

        fn send(bridge: &mut CuIgusCriBridge, ctx: &CuContext, command: CuSessionCommand) {
            let mut msg = CuMsg::<CuSessionCommand>::default();
            msg.set_payload(command);
            bridge.send(ctx, &IgusCriTxChannels::SESSION, &msg).unwrap();
        }

        fn wait(bridge: &CuIgusCriBridge, done: impl Fn(&CuSessionState) -> bool) {
            let deadline = Instant::now() + Duration::from_secs(2);
            while !done(bridge.shared.lock().unwrap().session.state()) {
                assert!(Instant::now() < deadline);
                std::thread::sleep(Duration::from_millis(2));
            }
        }

        fn status_frame(a6: f64) -> String {
            format!(
                "CRISTART 9 STATUS MODE joint POSJOINTCURRENT 0 0 0 0 0 {a6} 0 0 0 0 0 0 0 0 0 0 ERROR NoError 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 KINSTATE 0 CRIEND"
            )
        }

        fn move_command(a6: f64, mode: CuControllerMode) -> CuMsg<CuJointCommandPayload<6>> {
            let payload = CuJointCommandPayload::<6> {
                mode,
                positions: [0.0, 0.0, 0.0, 0.0, 0.0, a6].map(|d| {
                    CuJointPosition::Revolute(cu29::units::si::f64::Angle::new::<
                        cu29::units::si::angle::degree,
                    >(d))
                }),
                ..Default::default()
            };
            let mut msg = CuMsg::default();
            msg.set_payload(payload);
            msg
        }

        fn motion(bridge: &CuIgusCriBridge) -> CuCriMotionState {
            bridge.shared.lock().unwrap().motion.state().clone()
        }

        #[test]
        fn test_move_joint_against_fake_controller() {
            use crate::protocol::CuCriMotionRejection;
            use crate::protocol::CuCriMotionStatus;
            let listener = TcpListener::bind("127.0.0.1:0").unwrap();
            let params = CuIgusCriParams {
                address: listener.local_addr().unwrap().to_string(),
                allow_control: true,
                ..Default::default()
            };
            let (ctx, _) = CuContext::new_mock_clock();

            // Motion refused without the third opt-in.
            let mut locked = CuIgusCriBridge::from_params(params.clone()).unwrap();
            locked.start(&ctx).unwrap();
            let (_peer, _) = listener.accept().unwrap();
            let msg = move_command(5.0, CuControllerMode::Position);
            assert!(
                locked
                    .send(&ctx, &IgusCriTxChannels::JOINT_COMMAND, &msg)
                    .is_err()
            );
            locked.stop(&ctx).unwrap();

            let mut bridge = CuIgusCriBridge::from_params(CuIgusCriParams {
                allow_motion: true,
                max_step: Some(cu29::units::si::f64::Angle::new::<
                    cu29::units::si::angle::degree,
                >(10.0)),
                ..params
            })
            .unwrap();
            bridge.start(&ctx).unwrap();
            let (mut peer, _) = listener.accept().unwrap();
            peer.set_read_timeout(Some(Duration::from_secs(2))).unwrap();

            // Effort mode is not supported.
            let jog = move_command(5.0, CuControllerMode::Effort);
            assert!(
                bridge
                    .send(&ctx, &IgusCriTxChannels::JOINT_COMMAND, &jog)
                    .is_err()
            );

            // Not in control yet: refused locally.
            writeln!(peer, "{}", status_frame(0.0)).unwrap();
            wait(&bridge, |_| true);
            let deadline = Instant::now() + Duration::from_secs(2);
            while bridge.shared.lock().unwrap().latest.is_none() {
                assert!(Instant::now() < deadline);
                std::thread::sleep(Duration::from_millis(2));
            }
            bridge
                .send(&ctx, &IgusCriTxChannels::JOINT_COMMAND, &msg)
                .unwrap();
            assert_eq!(
                motion(&bridge).rejection,
                CuCriMotionRejection::NotInControl
            );

            // Acquire control; the status frame already reports drives ready.
            writeln!(peer, "CRISTART 1 CMD Active true CRIEND").unwrap();
            writeln!(peer, "{}", status_frame(0.0)).unwrap();
            wait(&bridge, |s| s.control_acquired && s.motion_allowed);

            let start = move_command(5.0, CuControllerMode::Position);
            // A different target counts as new; the earlier rejection was for this one,
            // so nudge it to force a fresh request.
            let start_b = move_command(5.1, CuControllerMode::Position);
            bridge
                .send(&ctx, &IgusCriTxChannels::JOINT_COMMAND, &start_b)
                .unwrap();
            let (counter, command) = next_command(&mut peer);
            assert_eq!(command, "Move Joint 0.0 0.0 0.0 0.0 0.0 5.1 0 0 0 5");
            // Resend: no-op. Different target in flight: refused.
            bridge
                .send(&ctx, &IgusCriTxChannels::JOINT_COMMAND, &start_b)
                .unwrap();
            bridge
                .send(&ctx, &IgusCriTxChannels::JOINT_COMMAND, &start)
                .unwrap();
            assert_eq!(motion(&bridge).refused, 1);
            writeln!(peer, "CRISTART 2 CMDACK {counter} CRIEND").unwrap();
            writeln!(peer, "{}", status_frame(2.0)).unwrap();
            writeln!(peer, "CRISTART 3 EXECEND 1 move CRIEND").unwrap();
            let deadline = Instant::now() + Duration::from_secs(2);
            while motion(&bridge).last_exec.as_str() != "EXECEND 1 move" {
                assert!(Instant::now() < deadline);
                std::thread::sleep(Duration::from_millis(2));
            }
            assert_eq!(motion(&bridge).status, CuCriMotionStatus::Moving);
            writeln!(peer, "{}", status_frame(4.8)).unwrap();
            let deadline = Instant::now() + Duration::from_secs(2);
            while motion(&bridge).status != CuCriMotionStatus::Reached {
                assert!(Instant::now() < deadline);
                std::thread::sleep(Duration::from_millis(2));
            }
            let mut out = CuMsg::<CuCriMotionState>::default();
            bridge
                .receive(&ctx, &IgusCriRxChannels::CRI_MOTION, &mut out)
                .unwrap();
            assert_eq!(out.payload().unwrap().status, CuCriMotionStatus::Reached);

            // Beyond max_step from the measured pose: refused, nothing sent.
            let far = move_command(30.0, CuControllerMode::Position);
            bridge
                .send(&ctx, &IgusCriTxChannels::JOINT_COMMAND, &far)
                .unwrap();
            assert_eq!(
                motion(&bridge).rejection,
                CuCriMotionRejection::MaxStepExceeded
            );

            // Control lost mid-move aborts tracking.
            let back = move_command(0.0, CuControllerMode::Position);
            bridge
                .send(&ctx, &IgusCriTxChannels::JOINT_COMMAND, &back)
                .unwrap();
            let (_, command) = next_command(&mut peer);
            assert!(command.starts_with("Move Joint"));
            writeln!(peer, "CRISTART 4 CMD Active false CRIEND").unwrap();
            writeln!(peer, "{}", status_frame(4.0)).unwrap();
            let deadline = Instant::now() + Duration::from_secs(2);
            while motion(&bridge).status != CuCriMotionStatus::Aborted {
                assert!(Instant::now() < deadline);
                std::thread::sleep(Duration::from_millis(2));
            }
            bridge.stop(&ctx).unwrap();
        }

        #[test]
        fn test_velocity_streams_jog_and_times_out() {
            let listener = TcpListener::bind("127.0.0.1:0").unwrap();
            let mut bridge = CuIgusCriBridge::from_params(CuIgusCriParams {
                address: listener.local_addr().unwrap().to_string(),
                allow_control: true,
                allow_motion: true,
                ..Default::default()
            })
            .unwrap();
            let (ctx, _) = CuContext::new_mock_clock();
            bridge.start(&ctx).unwrap();
            let (mut peer, _) = listener.accept().unwrap();
            peer.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
            writeln!(peer, "CRISTART 1 CMD Active true CRIEND").unwrap();
            writeln!(peer, "{}", status_frame(0.0)).unwrap();
            wait(&bridge, |s| s.control_acquired && s.motion_allowed);
            let deadline = Instant::now() + Duration::from_secs(2);
            while bridge.shared.lock().unwrap().latest.is_none() {
                assert!(Instant::now() < deadline);
                std::thread::sleep(Duration::from_millis(2));
            }
            // 3 deg/s on every joint, positions on target: 5 % jog.
            let mut payload = CuJointCommandPayload::<6> {
                mode: CuControllerMode::Velocity,
                ..Default::default()
            };
            payload.velocities = [repose_kinematics::CuJointVelocity::Revolute(
                cu29::units::si::f64::AngularVelocity::new::<
                    cu29::units::si::angular_velocity::degree_per_second,
                >(3.0),
            ); 6];
            let mut msg = CuMsg::<CuJointCommandPayload<6>>::default();
            msg.set_payload(payload);
            bridge
                .send(&ctx, &IgusCriTxChannels::JOINT_COMMAND, &msg)
                .unwrap();
            let line = loop {
                let line = read_line(&mut peer);
                if line.contains("ALIVEJOG") && !line.contains("ALIVEJOG 0 0") {
                    break line;
                }
            };
            assert!(
                line.contains("ALIVEJOG 5.000 5.000 5.000 5.000 5.000 5.000 0 0 0"),
                "{line}"
            );
            // No new command: back to zero after the 100 ms timeout.
            std::thread::sleep(Duration::from_millis(250));
            let mut zero = false;
            for _ in 0..10 {
                let line = read_line(&mut peer);
                if line.contains("ALIVEJOG 0 0 0 0 0 0 0 0 0") {
                    zero = true;
                    break;
                }
            }
            assert!(zero);
            bridge.stop(&ctx).unwrap();
        }

        #[test]
        fn test_session_against_fake_controller() {
            let listener = TcpListener::bind("127.0.0.1:0").unwrap();
            let params = CuIgusCriParams {
                address: listener.local_addr().unwrap().to_string(),
                ..Default::default()
            };
            let (ctx, _) = CuContext::new_mock_clock();

            // Refused without the second opt-in.
            let mut locked = CuIgusCriBridge::from_params(params.clone()).unwrap();
            locked.start(&ctx).unwrap();
            let (_peer, _) = listener.accept().unwrap();
            let mut msg = CuMsg::<CuSessionCommand>::default();
            msg.set_payload(CuSessionCommand::EnableDrives);
            assert!(
                locked
                    .send(&ctx, &IgusCriTxChannels::SESSION, &msg)
                    .is_err()
            );
            locked.stop(&ctx).unwrap();

            let mut bridge = CuIgusCriBridge::from_params(CuIgusCriParams {
                allow_control: true,
                ..params
            })
            .unwrap();
            bridge.start(&ctx).unwrap();
            let (mut peer, _) = listener.accept().unwrap();
            peer.set_read_timeout(Some(Duration::from_secs(2))).unwrap();

            // Passive: refused locally, nothing sent.
            writeln!(peer, "CRISTART 0 CMD Active false CRIEND").unwrap();
            send(&mut bridge, &ctx, CuSessionCommand::EnableDrives);
            wait(&bridge, |s| s.rejection == CuSessionRejection::NotInControl);

            send(&mut bridge, &ctx, CuSessionCommand::AcquireControl);
            let (_, command) = next_command(&mut peer);
            assert_eq!(command, "SetActive true");
            writeln!(peer, "CRISTART 1 CMD Active true CRIEND").unwrap();
            wait(&bridge, |s| {
                s.control_acquired && s.result == CuSessionResult::Acknowledged
            });

            send(&mut bridge, &ctx, CuSessionCommand::EnableDrives);
            let (counter, command) = next_command(&mut peer);
            assert_eq!(command, "Enable");
            // A second request while one is pending is refused, not queued.
            send(&mut bridge, &ctx, CuSessionCommand::ClearFaults);
            wait(&bridge, |s| s.refused == 1);
            writeln!(peer, "CRISTART 2 CMDACK {} CRIEND", counter - 1).unwrap();
            writeln!(peer, "CRISTART 3 CMDACK {counter} CRIEND").unwrap();
            wait(&bridge, |s| s.drives_enabled);
            let mut state = CuMsg::<CuSessionState>::default();
            bridge
                .receive(&ctx, &IgusCriRxChannels::SESSION_STATE, &mut state)
                .unwrap();
            assert_eq!(
                state.payload().unwrap().command,
                Some(CuSessionCommand::EnableDrives)
            );

            // Controller rejection carries its text.
            send(&mut bridge, &ctx, CuSessionCommand::ClearFaults);
            let (counter, _) = next_command(&mut peer);
            writeln!(peer, "CRISTART 4 CMDERROR {counter} not available CRIEND").unwrap();
            wait(&bridge, |s| s.rejection == CuSessionRejection::Controller);
            assert_eq!(
                bridge
                    .shared
                    .lock()
                    .unwrap()
                    .session
                    .state()
                    .detail
                    .as_str(),
                "not available"
            );

            // Stop disables the drives, then releases control.
            let responder = std::thread::spawn(move || {
                let (counter, disable) = next_command(&mut peer);
                writeln!(peer, "CRISTART 5 CMDACK {counter} CRIEND").unwrap();
                let (_, release) = next_command(&mut peer);
                writeln!(peer, "CRISTART 6 CMD Active false CRIEND").unwrap();
                (disable, release)
            });
            bridge.stop(&ctx).unwrap();
            let (disable, release) = responder.join().unwrap();
            assert_eq!(disable, "Disable");
            assert_eq!(release, "SetActive false");
        }
    }
}
