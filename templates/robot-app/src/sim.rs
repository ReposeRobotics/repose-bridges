//! `SimRobot<N>`: the standard robot channels (joint_state, session_state,
//! session, joint_command) over the MuJoCo sidecar in `sim/`, so the same
//! runner and program drive the simulated arm.
//!
//! Joint traffic goes to `repose_mujoco_sim::CuMujocoSimBridge<N>`. Its wire
//! protocol is lockstep (one command in, one state out), and Copper does not
//! call `send` for an empty message, so when no joint command went out since
//! the last state, `receive` first sends a hold at the last measured
//! positions. A simulator has no controller or drives: every session command
//! is acknowledged at once, and `EnableDrives` allows motion.
//!
//! Config is the MuJoCo bridge's: `host`, `port`, `read_timeout`.

use cu29::prelude::*;
use repose_controller::CuControllerMode;
use repose_controller::CuJointCommandPayload;
use repose_kinematics::CuJointPosition;
use repose_kinematics::CuJointStatePayload;
use repose_mujoco_sim::CuMujocoSimBridge;
use repose_mujoco_sim::MujocoRxChannels;
use repose_mujoco_sim::MujocoRxId;
use repose_mujoco_sim::MujocoTxChannels;
use repose_mujoco_sim::MujocoTxId;
use repose_robot_interface::CuSessionCommand;
use repose_robot_interface::CuSessionResult;
use repose_robot_interface::CuSessionState;
use serde::Deserialize;
use serde::Serialize;

#[derive(Copy, Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[repr(usize)]
#[serde(rename_all = "snake_case")]
pub enum SimTxId {
    Session,
    JointCommand,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[repr(usize)]
#[serde(rename_all = "snake_case")]
pub enum SimRxId {
    JointState,
    SessionState,
}

pub struct SimTxChannels<const N: usize>;

impl<const N: usize> SimTxChannels<N> {
    pub const SESSION: BridgeChannel<SimTxId, CuSessionCommand> =
        BridgeChannel::with_channel(SimTxId::Session, "session");
    pub const JOINT_COMMAND: BridgeChannel<SimTxId, CuJointCommandPayload<N>> =
        BridgeChannel::with_channel(SimTxId::JointCommand, "joint_command");
}

impl<const N: usize> BridgeChannelSet for SimTxChannels<N> {
    type Id = SimTxId;
    const STATIC_CHANNELS: &'static [&'static dyn BridgeChannelInfo<Self::Id>] =
        &[&Self::SESSION, &Self::JOINT_COMMAND];
}

pub struct SimRxChannels<const N: usize>;

impl<const N: usize> SimRxChannels<N> {
    pub const JOINT_STATE: BridgeChannel<SimRxId, CuJointStatePayload<N>> =
        BridgeChannel::with_channel(SimRxId::JointState, "joint_state");
    pub const SESSION_STATE: BridgeChannel<SimRxId, CuSessionState> =
        BridgeChannel::with_channel(SimRxId::SessionState, "session_state");
}

impl<const N: usize> BridgeChannelSet for SimRxChannels<N> {
    type Id = SimRxId;
    const STATIC_CHANNELS: &'static [&'static dyn BridgeChannelInfo<Self::Id>] =
        &[&Self::JOINT_STATE, &Self::SESSION_STATE];
}

#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct SimRobot<const N: usize> {
    #[reflect(ignore)]
    mujoco: CuMujocoSimBridge<N>,
    // The inner bridge takes `&'static` channels; generic consts cannot be
    // promoted, so these are leaked once in `new`.
    #[reflect(ignore)]
    mujoco_tx: &'static BridgeChannel<MujocoTxId, CuJointCommandPayload<N>>,
    #[reflect(ignore)]
    mujoco_rx: &'static BridgeChannel<MujocoRxId, CuJointStatePayload<N>>,
    #[reflect(ignore)]
    session: CuSessionState,
    #[reflect(ignore)]
    measured: Option<[CuJointPosition; N]>,
    /// A command was written since the last state was read.
    #[reflect(ignore)]
    pending: bool,
    #[reflect(ignore)]
    hold: CuMsg<CuJointCommandPayload<N>>,
}

impl<const N: usize> Freezable for SimRobot<N> {}

impl<const N: usize> SimRobot<N> {
    fn apply(&mut self, command: CuSessionCommand) {
        let s = &mut self.session;
        s.command = Some(command);
        s.sequence = s.sequence.wrapping_add(1);
        s.result = CuSessionResult::Acknowledged;
        match command {
            CuSessionCommand::AcquireControl => s.control_acquired = true,
            CuSessionCommand::ReleaseControl => s.control_acquired = false,
            CuSessionCommand::ClearFaults => {}
            CuSessionCommand::EnableDrives => s.drives_enabled = true,
            CuSessionCommand::DisableDrives => s.drives_enabled = false,
        }
        s.drives_ready = s.drives_enabled;
        s.motion_allowed = s.control_acquired && s.drives_enabled;
    }
}

impl<const N: usize> CuBridge for SimRobot<N> {
    type Tx = SimTxChannels<N>;
    type Rx = SimRxChannels<N>;
    type Resources<'r> = ();

    fn new(
        config: Option<&ComponentConfig>,
        _tx: &[BridgeChannelConfig<SimTxId>],
        _rx: &[BridgeChannelConfig<SimRxId>],
        _resources: Self::Resources<'_>,
    ) -> CuResult<Self> {
        let mujoco_tx = Box::leak(Box::new(MujocoTxChannels::<N>::JOINT_COMMAND));
        let mujoco_rx = Box::leak(Box::new(MujocoRxChannels::<N>::JOINT_STATE));
        let mujoco = CuMujocoSimBridge::new(
            config,
            &[BridgeChannelConfig::from_static(mujoco_tx, None, None)],
            &[BridgeChannelConfig::from_static(mujoco_rx, None, None)],
            (),
        )?;
        Ok(Self {
            mujoco,
            mujoco_tx,
            mujoco_rx,
            session: CuSessionState::default(),
            measured: None,
            pending: false,
            hold: CuMsg::new(None),
        })
    }

    fn start(&mut self, ctx: &CuContext) -> CuResult<()> {
        self.mujoco.start(ctx)
    }

    fn send<'a, Payload>(
        &mut self,
        ctx: &CuContext,
        channel: &'static BridgeChannel<SimTxId, Payload>,
        msg: &CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
        match channel.id {
            SimTxId::Session => {
                if let Some(&command) = msg.downcast_ref::<CuSessionCommand>()?.payload() {
                    self.apply(command);
                }
                Ok(())
            }
            SimTxId::JointCommand => {
                let command = msg.downcast_ref::<CuJointCommandPayload<N>>()?;
                if command.payload().is_none() {
                    return Ok(());
                }
                self.mujoco.send(ctx, self.mujoco_tx, command)?;
                self.pending = true;
                Ok(())
            }
        }
    }

    fn receive<'a, Payload>(
        &mut self,
        ctx: &CuContext,
        channel: &'static BridgeChannel<SimRxId, Payload>,
        msg: &mut CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
        match channel.id {
            SimRxId::JointState => {
                // Keep the lockstep: hold the last measured pose. Before the
                // first state (and after a dropped connection) the sidecar
                // sends its initial state unasked, so nothing is sent then.
                if !core::mem::take(&mut self.pending)
                    && let Some(positions) = self.measured
                {
                    self.hold.set_payload(CuJointCommandPayload {
                        mode: CuControllerMode::Position,
                        positions,
                        ..Default::default()
                    });
                    self.mujoco.send(ctx, self.mujoco_tx, &self.hold)?;
                }
                let state = msg.downcast_mut::<CuJointStatePayload<N>>()?;
                let received = self.mujoco.receive(ctx, self.mujoco_rx, state);
                self.measured = match received {
                    Ok(()) => state.payload().map(|s| s.positions),
                    Err(_) => None,
                };
                received
            }
            SimRxId::SessionState => {
                let state = msg.downcast_mut::<CuSessionState>()?;
                state.tov = Tov::Time(ctx.now());
                state.set_payload(self.session.clone());
                Ok(())
            }
        }
    }

    fn stop(&mut self, ctx: &CuContext) -> CuResult<()> {
        self.mujoco.stop(ctx)
    }
}
