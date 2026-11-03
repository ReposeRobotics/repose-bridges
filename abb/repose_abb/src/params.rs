use cu29::prelude::*;
use cu29::units::si::{
    angle::radian,
    angular_velocity::radian_per_second,
    f64::{Angle, AngularVelocity, Time},
    time::second,
};
use serde::{Deserialize, Serialize};
use std::{
    net::{IpAddr, SocketAddr},
    time::Duration,
};

/// Shared Rust / RON configuration. Durations and angles use Copper SI units.
#[derive(Debug, Clone, Serialize, Deserialize, Reflect)]
#[serde(default, deny_unknown_fields)]
pub struct CuAbbParams {
    /// Local UDP address the controller's UDPUC device sends EGM to.
    pub bind: String,
    /// Numeric controller IP. Datagrams from any other source are ignored.
    pub robot_ip: String,
    /// Allow session commands (requires the `control` feature).
    pub allow_control: bool,
    /// Allow joint corrections; requires `allow_control`.
    pub allow_motion: bool,
    /// Feedback older than this is not published; while streaming it faults.
    pub stale_after: Time,
    /// A joint command older than this is replaced by holding position.
    pub command_timeout: Time,
    /// Longest wait for a session command's observed outcome.
    pub session_timeout: Time,
    /// Largest commanded joint speed sent to EGM.
    pub speed: AngularVelocity,
    /// Largest distance of a commanded position from measured position.
    pub max_step: Angle,
}
impl Default for CuAbbParams {
    fn default() -> Self {
        Self {
            bind: "0.0.0.0:6510".into(),
            robot_ip: "192.168.125.1".into(),
            allow_control: false,
            allow_motion: false,
            stale_after: Time::new::<second>(0.1),
            command_timeout: Time::new::<second>(0.1),
            session_timeout: Time::new::<second>(10.0),
            speed: AngularVelocity::new::<radian_per_second>(0.5),
            max_step: Angle::new::<radian>(0.25),
        }
    }
}
impl CuAbbParams {
    pub fn validate(&self) -> CuResult<()> {
        self.bind_address()?;
        self.robot_address()?;
        if self.allow_motion && !self.allow_control {
            return Err(CuError::from("ABB allow_motion requires allow_control"));
        }
        for t in [self.stale_after, self.command_timeout, self.session_timeout] {
            duration(t)?;
        }
        for (value, maximum) in [
            (self.speed.get::<radian_per_second>(), 2.0),
            (self.max_step.get::<radian>(), std::f64::consts::PI),
        ] {
            if !(value.is_finite() && value > 0.0 && value <= maximum) {
                return Err(CuError::from(
                    "ABB motion parameter outside permitted range",
                ));
            }
        }
        Ok(())
    }
    pub(crate) fn bind_address(&self) -> CuResult<SocketAddr> {
        self.bind
            .parse()
            .map_err(|e| CuError::new_with_cause("ABB bind must be a numeric socket address", e))
    }
    pub(crate) fn robot_address(&self) -> CuResult<IpAddr> {
        self.robot_ip
            .parse()
            .map_err(|e| CuError::new_with_cause("ABB robot_ip must be a numeric IP address", e))
    }
}
pub(crate) fn duration(value: Time) -> CuResult<Duration> {
    let seconds = value.get::<second>();
    if !(seconds.is_finite() && seconds > 0.0 && seconds <= 3600.0) {
        return Err(CuError::from("ABB duration must be in 0..=3600 seconds"));
    }
    Ok(Duration::from_secs_f64(seconds))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn validation() {
        assert!(CuAbbParams::default().validate().is_ok());
        let bad = [
            CuAbbParams {
                robot_ip: "robot.local".into(),
                ..Default::default()
            },
            CuAbbParams {
                bind: "6510".into(),
                ..Default::default()
            },
            CuAbbParams {
                allow_motion: true,
                ..Default::default()
            },
            CuAbbParams {
                stale_after: Time::new::<second>(f64::NAN),
                ..Default::default()
            },
            CuAbbParams {
                speed: AngularVelocity::new::<radian_per_second>(3.0),
                ..Default::default()
            },
            CuAbbParams {
                max_step: Angle::new::<radian>(0.0),
                ..Default::default()
            },
        ];
        for params in bad {
            assert!(params.validate().is_err(), "{params:?}");
        }
    }

    #[test]
    fn from_ron_rejects_unknown_fields() {
        let ron = |config: &str| {
            format!(
                r#"(tasks: [], cnx: [], bridges: [(id: "robot", type: "repose_abb::CuAbbBridge",
                config: {config}, channels: [])])"#
            )
        };
        let config = cu29::config::read_configuration_str(
            ron(r#"{"robot_ip": "10.0.0.5", "speed": 0.2}"#),
            None,
        )
        .unwrap();
        let params: CuAbbParams = config.bridges[0]
            .config
            .as_ref()
            .unwrap()
            .deserialize_into()
            .unwrap();
        assert_eq!(params.robot_ip, "10.0.0.5");
        assert_eq!(params.speed.get::<radian_per_second>(), 0.2);
        let config =
            cu29::config::read_configuration_str(ron(r#"{"robot": "10.0.0.5"}"#), None).unwrap();
        assert!(
            config.bridges[0]
                .config
                .as_ref()
                .unwrap()
                .deserialize_into::<CuAbbParams>()
                .is_err()
        );
    }
}
