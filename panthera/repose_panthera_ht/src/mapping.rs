use crate::protocol::CuPantheraStatus;
use repose_kinematics::{CuJointPosition, CuJointStatePayload, CuJointVelocity, CuRobotStatus};

pub(crate) fn joint_state(status: &CuPantheraStatus) -> CuJointStatePayload<6> {
    let any_fault = status.motors.iter().any(|m| m.fault != 0);
    let can_bad = status.can_fault < 0 || status.can_fault > 1;
    CuJointStatePayload {
        positions: core::array::from_fn(|i| CuJointPosition::Revolute(status.motors[i].position)),
        velocities: core::array::from_fn(|i| CuJointVelocity::Revolute(status.motors[i].velocity)),
        status: if any_fault {
            CuRobotStatus::Fault(u32::from(
                status.motors.iter().map(|m| m.fault).max().unwrap_or(0),
            ))
        } else if can_bad {
            CuRobotStatus::Fault(u32::from(status.can_fault as u8))
        } else {
            CuRobotStatus::Ready
        },
    }
}
