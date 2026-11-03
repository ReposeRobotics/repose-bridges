use crate::protocol::{CuAbbMciState, CuAbbStatus};
use repose_kinematics::{CuJointPosition, CuJointStatePayload, CuJointVelocity, CuRobotStatus};

/// EGM error is the only fault EGM reports; its code is the raw MCI state (1).
pub(crate) fn joint_state(status: &CuAbbStatus) -> CuJointStatePayload<6> {
    CuJointStatePayload {
        positions: status.joints.map(CuJointPosition::Revolute),
        velocities: status.velocities.map(CuJointVelocity::Revolute),
        status: if status.mci == CuAbbMciState::Error {
            CuRobotStatus::Fault(1)
        } else if status.ready() {
            CuRobotStatus::Ready
        } else {
            CuRobotStatus::NotReady
        },
    }
}
