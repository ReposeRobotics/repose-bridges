use crate::protocol::CuUrStatus;
use repose_kinematics::{CuJointPosition, CuJointStatePayload, CuJointVelocity, CuRobotStatus};
pub(crate) fn joint_state(status: &CuUrStatus) -> CuJointStatePayload<6> {
    CuJointStatePayload {
        positions: status.joints.map(CuJointPosition::Revolute),
        velocities: status.velocities.map(CuJointVelocity::Revolute),
        status: if !matches!(status.safety_mode, 1 | 2) {
            CuRobotStatus::Fault(status.safety_mode as u32)
        } else if status.ready() {
            CuRobotStatus::Ready
        } else {
            CuRobotStatus::NotReady
        },
    }
}
