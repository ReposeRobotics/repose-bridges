//! Per-joint PID position feedback for velocity-streaming bridges.
//!
//! A bridge streaming velocity commands adds `kp·e + ki·∫e dt + kd·de/dt` to
//! the command's feed-forward velocity, where `e` is commanded minus measured
//! position. The math is unit-agnostic: callers pass error in one angle unit
//! and receive velocity in that unit per second. No I/O and no allocation.

use cu29::prelude::*;
use serde::{Deserialize, Serialize};

/// Gains for one joint; one entry of a bridge's `joint_gains` RON list.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Reflect)]
#[serde(default, deny_unknown_fields)]
pub struct CuJointGains {
    /// Proportional gain, 1/s.
    pub kp: f64,
    /// Integral gain, 1/s².
    pub ki: f64,
    /// Derivative gain, dimensionless (s·1/s).
    pub kd: f64,
    /// Largest |ki·∫e dt| contribution, as a fraction of the bridge's speed
    /// cap (0..=1). Bounds integral windup.
    pub integral_limit: f64,
}

impl Default for CuJointGains {
    fn default() -> Self {
        Self {
            kp: 2.0,
            ki: 0.0,
            kd: 0.0,
            integral_limit: 0.5,
        }
    }
}

impl CuJointGains {
    /// Reject non-finite or negative gains and a limit outside 0..=1.
    pub fn validate(&self) -> CuResult<()> {
        for gain in [self.kp, self.ki, self.kd] {
            if !(gain.is_finite() && gain >= 0.0) {
                return Err(CuError::from("joint gains must be finite and >= 0"));
            }
        }
        if !(self.integral_limit.is_finite() && (0.0..=1.0).contains(&self.integral_limit)) {
            return Err(CuError::from("joint integral_limit must be in 0..=1"));
        }
        Ok(())
    }
}

/// Validate every joint's gains.
pub fn validate_gains(gains: &[CuJointGains]) -> CuResult<()> {
    gains.iter().try_for_each(CuJointGains::validate)
}

/// Integral and derivative state for `N` joints.
///
/// The first update after construction or [`reset`](Self::reset) is
/// proportional-only; later updates use the elapsed time `dt` passed in.
#[derive(Debug, Clone, Copy)]
pub struct JointPid<const N: usize> {
    gains: [CuJointGains; N],
    integral: [f64; N],
    previous: Option<[f64; N]>,
}

impl<const N: usize> Default for JointPid<N> {
    fn default() -> Self {
        Self::new([CuJointGains::default(); N])
    }
}

impl<const N: usize> JointPid<N> {
    pub fn new(gains: [CuJointGains; N]) -> Self {
        Self {
            gains,
            integral: [0.0; N],
            previous: None,
        }
    }

    /// Forget accumulated integral and previous error, e.g. after a gap in
    /// the command stream or a mode change.
    pub fn reset(&mut self) {
        self.integral = [0.0; N];
        self.previous = None;
    }

    /// Feedback velocity per joint for position `error`, after `dt` seconds
    /// since the previous update. `max_speed` (same unit as the result) scales
    /// each joint's `integral_limit`. Non-finite or non-positive `dt` resets
    /// the state and returns the proportional term only.
    pub fn update(&mut self, error: [f64; N], dt: f64, max_speed: f64) -> [f64; N] {
        let Some(previous) = self.previous.filter(|_| dt.is_finite() && dt > 0.0) else {
            self.reset();
            self.previous = Some(error);
            return std::array::from_fn(|i| self.gains[i].kp * error[i]);
        };
        self.previous = Some(error);
        std::array::from_fn(|i| {
            let g = &self.gains[i];
            let limit = g.integral_limit * max_speed;
            if g.ki > 0.0 {
                // Store ki·∫e so the clamp applies directly to its velocity.
                self.integral[i] = (self.integral[i] + g.ki * error[i] * dt).clamp(-limit, limit);
            }
            g.kp * error[i] + self.integral[i] + g.kd * (error[i] - previous[i]) / dt
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn gains(kp: f64, ki: f64, kd: f64) -> [CuJointGains; 2] {
        [CuJointGains {
            kp,
            ki,
            kd,
            integral_limit: 0.5,
        }; 2]
    }

    #[test]
    fn test_validation() {
        assert!(CuJointGains::default().validate().is_ok());
        for bad in [
            CuJointGains {
                kp: -1.0,
                ..Default::default()
            },
            CuJointGains {
                ki: f64::NAN,
                ..Default::default()
            },
            CuJointGains {
                kd: f64::INFINITY,
                ..Default::default()
            },
            CuJointGains {
                integral_limit: 1.5,
                ..Default::default()
            },
        ] {
            assert!(bad.validate().is_err());
        }
    }

    #[test]
    fn test_proportional_first_then_pid() {
        let mut pid = JointPid::new(gains(2.0, 1.0, 0.5));
        assert_eq!(pid.update([1.0, -1.0], 0.02, 10.0), [2.0, -2.0]);
        let out = pid.update([0.8, -1.0], 0.1, 10.0);
        // 2·0.8 + 1·0.8·0.1 + 0.5·(−0.2)/0.1
        assert!((out[0] - (1.6 + 0.08 - 1.0)).abs() < 1e-12);
        assert!((out[1] - (-2.0 - 0.1)).abs() < 1e-12);
    }

    #[test]
    fn test_integral_clamped_and_reset() {
        let mut pid = JointPid::new(gains(0.0, 10.0, 0.0));
        pid.update([1.0; 2], 0.0, 1.0);
        for _ in 0..100 {
            pid.update([1.0; 2], 0.1, 1.0);
        }
        assert_eq!(pid.update([1.0; 2], 0.1, 1.0), [0.5; 2]);
        pid.reset();
        assert_eq!(pid.update([1.0; 2], 0.1, 1.0), [0.0; 2]);
    }
}
