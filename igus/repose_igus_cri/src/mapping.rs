//! CRI observations mapped onto the vendor-neutral Repose contract.

use crate::protocol::CuCriStatus;
use repose_kinematics::CuJointPosition;
use repose_kinematics::CuJointStatePayload;
use repose_kinematics::CuJointVelocity;
use repose_kinematics::CuRobotStatus;

/// ReBeL joints are revolute; CRI reports no joint velocities, so they are zero.
pub(crate) fn joint_state(status: &CuCriStatus, out: &mut CuJointStatePayload<6>) {
    for (position, angle) in out.positions.iter_mut().zip(status.joints) {
        *position = CuJointPosition::Revolute(angle);
    }
    out.velocities = [CuJointVelocity::default(); 6];
    out.status = robot_status(status);
}

/// `Fault` for any joint code other than ready / not enabled / enabling,
/// `Ready` when the controller would accept motion, otherwise `NotReady`.
/// CRI's `ESTOP` value is not mapped: its encoding is undocumented.
pub(crate) fn robot_status(status: &CuCriStatus) -> CuRobotStatus {
    if let Some(code) = status.joint_fault() {
        // Bit-preserving: vendor codes are reported as the controller sent them.
        CuRobotStatus::Fault(code as u32)
    } else if status.motion_allowed() {
        CuRobotStatus::Ready
    } else {
        CuRobotStatus::NotReady
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::tests::FULL_STATUS;
    use crate::protocol::tests::status;
    use cu29::units::si::angle::degree;

    #[test]
    fn test_robot_status() {
        let mut cri = status(FULL_STATUS).unwrap().unwrap();
        let mut out = CuJointStatePayload::<6>::default();
        joint_state(&cri, &mut out);
        assert_eq!(out.status, CuRobotStatus::NotReady);
        let CuJointPosition::Revolute(a6) = out.positions[5] else {
            panic!("revolute expected");
        };
        assert!((a6.get::<degree>() - 6.0).abs() < 1e-10);
        cri.joint_errors = [0; 6];
        assert_eq!(robot_status(&cri), CuRobotStatus::NotReady); // KINSTATE 13
        cri.kinstate = Some(0);
        assert_eq!(robot_status(&cri), CuRobotStatus::Ready);
        cri.joint_errors[2] = 0x40;
        assert_eq!(robot_status(&cri), CuRobotStatus::Fault(0x40));
    }
}
