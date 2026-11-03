//! Streaming velocity (jog) commands: CRI `ALIVEJOG` values are percent of
//! each joint's maximum speed. Pure computation, no I/O.

use crate::protocol::CuCriStatus;
use cu29::units::si::angle::degree;
use cu29::units::si::angular_velocity::degree_per_second;
use cu29::units::si::f64::AngularVelocity;
use repose_controller::CuJointCommandPayload;
use repose_joint_pid::JointPid;
use repose_kinematics::CuJointPosition;
use repose_kinematics::CuJointVelocity;
use repose_kinematics::igus_rebel_chain;

/// How far ahead a jog may not cross a joint limit.
const LIMIT_LOOKAHEAD_S: f64 = 0.2;

/// Jog tuning taken from the bridge params.
#[derive(Debug, Clone, Copy, Default)]
pub(crate) struct JogLimits {
    /// Joint speed at 100 % jog.
    pub full_scale: AngularVelocity,
    /// Largest jog magnitude sent, percent.
    pub max_percent: f64,
}

/// Jog percent per joint for a velocity command: feed-forward velocity plus
/// per-joint PID feedback on (commanded − measured) in degrees, clamped to
/// `max_percent`. `dt` is seconds since the previous command (non-positive
/// resets the PID). A joint whose motion would cross its limit within the
/// lookahead is zeroed.
pub(crate) fn jog_percent(
    command: &CuJointCommandPayload<6>,
    measured: &CuCriStatus,
    limits: &JogLimits,
    pid: &mut JointPid<6>,
    dt: f64,
) -> Result<[f64; 6], &'static str> {
    let full = limits.full_scale.get::<degree_per_second>();
    let chain = igus_rebel_chain();
    let mut error = [0.0; 6];
    for (i, slot) in error.iter_mut().enumerate() {
        let CuJointPosition::Revolute(target) = command.positions[i] else {
            return Err("ReBeL joints are revolute");
        };
        *slot = target.get::<degree>() - measured.joints[i].get::<degree>();
    }
    let feedback = pid.update(error, dt, full * limits.max_percent / 100.0);
    let mut out = [0.0; 6];
    for (i, slot) in out.iter_mut().enumerate() {
        let CuJointVelocity::Revolute(v) = command.velocities[i] else {
            return Err("ReBeL joints are revolute");
        };
        let at = measured.joints[i].get::<degree>();
        let speed = v.get::<degree_per_second>() + feedback[i];
        let percent = (speed / full * 100.0).clamp(-limits.max_percent, limits.max_percent);
        if !percent.is_finite() {
            return Err("non-finite velocity command");
        }
        let ahead = at + percent / 100.0 * full * LIMIT_LOOKAHEAD_S;
        let joint = &chain.joints()[i];
        let next = CuJointPosition::Revolute(cu29::units::si::f64::Angle::new::<degree>(ahead));
        *slot = if joint.limits.contains(next) {
            percent
        } else {
            0.0
        };
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu29::units::si::f64::Angle;
    use repose_controller::CuControllerMode;

    fn command(pos_deg: f64, vel_dps: f64) -> CuJointCommandPayload<6> {
        CuJointCommandPayload {
            mode: CuControllerMode::Velocity,
            positions: [CuJointPosition::Revolute(Angle::new::<degree>(pos_deg)); 6],
            velocities: [CuJointVelocity::Revolute(AngularVelocity::new::<degree_per_second>(
                vel_dps,
            )); 6],
            ..Default::default()
        }
    }

    fn pid() -> JointPid<6> {
        JointPid::new([repose_joint_pid::CuJointGains::default(); 6])
    }

    fn limits() -> JogLimits {
        JogLimits {
            full_scale: AngularVelocity::new::<degree_per_second>(60.0),
            max_percent: 10.0,
        }
    }

    #[test]
    fn test_feed_forward_feedback_and_clamp() {
        let measured = CuCriStatus::default();
        // 3 °/s at 60 °/s full scale is 5 %.
        let out = jog_percent(&command(0.0, 3.0), &measured, &limits(), &mut pid(), 0.0).unwrap();
        assert!((out[0] - 5.0).abs() < 1e-9);
        // 1° behind adds kp × 1° = 2 °/s.
        let out = jog_percent(&command(1.0, 3.0), &measured, &limits(), &mut pid(), 0.0).unwrap();
        assert!((out[0] - 25.0 / 3.0).abs() < 1e-9);
        // Clamped to the cap.
        let out = jog_percent(&command(0.0, 100.0), &measured, &limits(), &mut pid(), 0.0).unwrap();
        assert_eq!(out[0], 10.0);
    }

    #[test]
    fn test_zero_near_limit() {
        let chain = igus_rebel_chain();
        let CuJointPosition::Revolute(upper) = chain.joints()[0].limits.upper() else {
            unreachable!()
        };
        let mut measured = CuCriStatus::default();
        measured.joints[0] = upper;
        let out = jog_percent(
            &command(upper.get::<degree>(), 5.0),
            &measured,
            &limits(),
            &mut pid(),
            0.0,
        )
        .unwrap();
        assert_eq!(out[0], 0.0);
        let out = jog_percent(
            &command(upper.get::<degree>(), -5.0),
            &measured,
            &limits(),
            &mut pid(),
            0.0,
        )
        .unwrap();
        assert!(out[0] < 0.0);
    }
}
