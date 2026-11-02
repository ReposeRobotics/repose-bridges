use cu29::prelude::*;
use cu29::units::si::{
    angle::radian,
    angular_acceleration::radian_per_second_squared,
    angular_velocity::radian_per_second,
    f64::{Angle, AngularAcceleration, AngularVelocity, Time},
    time::second,
};
use serde::{Deserialize, Serialize};
use std::{net::SocketAddr, time::Duration};

/// Shared Rust / RON configuration. Durations and angles use Copper SI units.
#[derive(Debug, Clone, Serialize, Deserialize, Reflect)]
#[serde(default, deny_unknown_fields)]
pub struct CuUrParams {
    pub address: String,
    pub script_port: u16,
    pub dashboard_port: u16,
    pub allow_control: bool,
    pub command_timeout: Time,
    pub position_gain: f64,
    pub frequency: f64,
    pub connect_timeout: Time,
    pub stale_after: Time,
    pub allow_motion: bool,
    pub speed: AngularVelocity,
    /// Leading-axis acceleration in rad/s².
    pub acceleration: AngularAcceleration,
    pub max_step: Angle,
    pub goal_tolerance: Angle,
    pub move_timeout: Time,
}
impl Default for CuUrParams {
    fn default() -> Self {
        Self {
            address: "127.0.0.1:30004".into(),
            script_port: 30002,
            dashboard_port: 29999,
            allow_control: false,
            command_timeout: Time::new::<second>(0.1),
            position_gain: 2.0,
            frequency: 50.0,
            connect_timeout: Time::new::<second>(2.0),
            stale_after: Time::new::<second>(0.5),
            allow_motion: false,
            speed: AngularVelocity::new::<radian_per_second>(0.1),
            acceleration: AngularAcceleration::new::<radian_per_second_squared>(0.2),
            max_step: Angle::new::<radian>(0.25),
            goal_tolerance: Angle::new::<radian>(0.001),
            move_timeout: Time::new::<second>(30.0),
        }
    }
}
impl CuUrParams {
    pub fn validate(&self) -> CuResult<()> {
        self.socket_address()?;
        duration(self.command_timeout)?;
        if self.allow_motion && !self.allow_control {
            return Err(CuError::from("UR allow_motion requires allow_control"));
        }
        if self.dashboard_port == 0 || !self.position_gain.is_finite() || self.position_gain < 0.0 {
            return Err(CuError::from("UR invalid dashboard port or position gain"));
        }
        if self.allow_control && self.frequency < 25.0 {
            return Err(CuError::from("UR control requires at least 25 Hz feedback"));
        }
        for t in [self.connect_timeout, self.stale_after, self.move_timeout] {
            duration(t)?;
        }
        if !(self.frequency.is_finite() && (1.0..=125.0).contains(&self.frequency)) {
            return Err(CuError::from("UR frequency must be 1..=125 Hz"));
        }
        if self.script_port == 0 {
            return Err(CuError::from("UR script_port must be nonzero"));
        }
        for (value, maximum) in [
            (self.speed.get::<radian_per_second>(), 1.0),
            (self.acceleration.get::<radian_per_second_squared>(), 2.0),
            (self.max_step.get::<radian>(), std::f64::consts::PI),
            (self.goal_tolerance.get::<radian>(), 0.1),
        ] {
            if !(value.is_finite() && value > 0.0 && value <= maximum) {
                return Err(CuError::from("UR motion parameter outside permitted range"));
            }
        }
        Ok(())
    }
    pub(crate) fn socket_address(&self) -> CuResult<SocketAddr> {
        self.address
            .parse()
            .map_err(|e| CuError::new_with_cause("UR requires numeric socket address", e))
    }
}
pub(crate) fn duration(value: Time) -> CuResult<Duration> {
    let seconds = value.get::<second>();
    if !(seconds.is_finite() && seconds > 0.0 && seconds <= 3600.0) {
        return Err(CuError::from("UR duration must be in 0..=3600 seconds"));
    }
    Ok(Duration::from_secs_f64(seconds))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn validation() {
        assert!(CuUrParams::default().validate().is_ok());
        for value in [f64::NAN, f64::INFINITY, 0.0, -1.0, 126.0] {
            assert!(
                CuUrParams {
                    frequency: value,
                    ..Default::default()
                }
                .validate()
                .is_err()
            );
        }
        assert!(
            CuUrParams {
                address: "robot.local:30004".into(),
                ..Default::default()
            }
            .validate()
            .is_err()
        );
        assert!(
            CuUrParams {
                stale_after: Time::new::<second>(f64::NAN),
                ..Default::default()
            }
            .validate()
            .is_err()
        );
        assert!(
            CuUrParams {
                max_step: Angle::new::<radian>(0.0),
                ..Default::default()
            }
            .validate()
            .is_err()
        );
    }
}
