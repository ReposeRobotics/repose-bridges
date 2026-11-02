//! Copper bridge for the igus ReBeL over CRI (TCP), implementing the
//! vendor-neutral Repose robot contract (`repose-robot-interface`).
//!
//! | Dir | Channel | Payload |
//! |---|---|---|
//! | Rx | `joint_state` | `repose_kinematics::CuJointStatePayload<6>` |
//! | Rx | `session_state` | `repose_robot_interface::CuSessionState` |
//! | Rx | `cri_status` | [`CuCriStatus`] (vendor: raw STATUS fields) |
//! | Tx | `session` | `repose_robot_interface::CuSessionCommand` (`control` feature) |
//!
//! Session commands need two opt-ins: the `control` cargo feature and
//! `allow_control` in [`CuIgusCriParams`]. No build sends motion; the CRI
//! heartbeat's jog values are always zero. See the crate README for wiring.
//!
//! Modules: `params` (config), `protocol` (pure CRI text encode/decode),
//! `mapping` (CRI → Repose contract), `transport` (TCP worker), `bridge`
//! (`CuBridge`).

mod bridge;
#[cfg(feature = "control")]
mod jog;
mod mapping;
mod motion;
mod params;
mod protocol;
mod transport;

pub use bridge::CuIgusCriBridge;
pub use bridge::IgusCriRxChannels;
pub use bridge::IgusCriRxId;
pub use bridge::IgusCriTxChannels;
pub use bridge::IgusCriTxId;
pub use params::CuIgusCriParams;
pub use protocol::CuCriCartesianPose;
pub use protocol::CuCriMotionRejection;
pub use protocol::CuCriMotionState;
pub use protocol::CuCriMotionStatus;
pub use protocol::CuCriStatus;
pub use protocol::CuCriText;
