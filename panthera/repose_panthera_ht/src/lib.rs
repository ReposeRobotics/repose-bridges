//! Copper bridge for the HighTorque Panthera HT 6-DOF arm.
//!
//! The arm's smart actuators sit behind HighTorque's v6 communication board
//! (USB serial, VID:PID `CAF1:FFFF`, 4 Mbaud). The wire protocol is the open
//! one from `HighTorque-Robotics/hightorque_robot` (`serial_struct.h`,
//! `canport.cpp`): 0xF7-headed frames with two CRC-8/CCITT bytes, carrying
//! either comm-board housekeeping or per-motor MIT-style commands.
//!
//! | Dir | Channel | Payload |
//! |---|---|---|
//! | Rx | `joint_state` | `repose_kinematics::CuJointStatePayload<6>` |
//! | Rx | `session_state` | `repose_robot_interface::CuSessionState` |
//! | Rx | `panthera_status` | [`CuPantheraStatus`] (per-motor mode/fault/state) |
//! | Tx | `session` | `repose_robot_interface::CuSessionCommand` (`control` feature) |
//! | Tx | `joint_command` | `repose_controller::CuJointCommandPayload<6>` (`control` feature) |
//!
//! Motion commands need two opt-ins: the `control` cargo feature and
//! `allow_motion` in [`CuPantheraHtParams`]. Position commands are sent as
//! `MODE_POS_VEL_TQE_KP_KD` frames; velocity/effort modes are refused. The
//! serial link is single-master, so `AcquireControl` is acknowledged locally
//! while the link is healthy. See the crate README for wiring and lifecycle.
//!
//! Modules: `params` (config), `protocol` (pure frame codec), `mapping`
//! (motor state → Repose contract), `transport` (serial worker), `bridge`
//! (`CuBridge`).

mod bridge;
mod mapping;
mod params;
mod protocol;
mod transport;

pub use bridge::{
    CuPantheraHtBridge, PantheraRxChannels, PantheraRxId, PantheraTxChannels, PantheraTxId,
};
pub use params::CuPantheraHtParams;
pub use protocol::{CuPantheraMotor, CuPantheraStatus};
