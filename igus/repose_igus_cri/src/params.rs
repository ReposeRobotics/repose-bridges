use cu29::prelude::*;
use cu29::units::si::angle::degree;
use cu29::units::si::angular_velocity::degree_per_second;
use cu29::units::si::f64::Angle;
use cu29::units::si::f64::AngularVelocity;
use cu29::units::si::f64::Time;
use cu29::units::si::time::second;
use serde::Deserialize;
use serde::Serialize;
use std::net::SocketAddr;
use std::time::Duration;

/// Bridge parameters; the bridge's RON `config:` block.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Reflect)]
#[serde(default, deny_unknown_fields)]
pub struct CuIgusCriParams {
    /// Numeric socket address of the controller (no DNS lookup).
    pub address: String,
    /// Maximum age of an observation published to Copper.
    pub stale_after: Time,
    /// Startup connection timeout.
    pub connect_timeout: Time,
    /// Second opt-in for the `control` feature: session commands are refused
    /// unless this is true. Ignored without the feature.
    pub allow_control: bool,
    /// Third opt-in: motion commands are refused unless this is true.
    /// Requires `allow_control = true`.
    pub allow_motion: bool,
    /// Velocity percent cap for Move Joint commands (0 < v <= 25.0).
    pub velocity_percent: f64,
    /// Optional largest step between target and measured angle per joint.
    /// `None` (default): no step limit; the ReBeL joint limits still apply.
    pub max_step: Option<Angle>,
    /// Goal tolerance: every joint must be within this tolerance for a move
    /// to be considered reached.
    pub goal_tolerance: Angle,
    /// Joint speed at 100 % CRI jog. Measured on a ReBeL (V980-14): 60 °/s on
    /// every axis, linear.
    pub jog_full_scale: AngularVelocity,
    /// Largest jog sent for velocity commands, percent of `jog_full_scale`
    /// (0 < p <= 100).
    pub max_jog_percent: f64,
    /// Velocity commands older than this are replaced by zero jog.
    pub command_timeout: Time,
    /// Position-error feedback for velocity commands, 1/s (>= 0).
    pub position_gain: f64,
}

impl Default for CuIgusCriParams {
    fn default() -> Self {
        Self {
            address: "192.168.3.11:3920".into(),
            stale_after: Time::new::<second>(0.5),
            connect_timeout: Time::new::<second>(2.0),
            allow_control: false,
            allow_motion: false,
            velocity_percent: 5.0,
            max_step: None,
            goal_tolerance: Angle::new::<degree>(0.5),
            jog_full_scale: AngularVelocity::new::<degree_per_second>(60.0),
            max_jog_percent: 10.0,
            command_timeout: Time::new::<second>(0.1),
            position_gain: 2.0,
        }
    }
}

impl CuIgusCriParams {
    /// Reject invalid addresses, non-positive durations, or out-of-bound motion params.
    pub fn validate(&self) -> CuResult<()> {
        self.socket_address()?;
        duration(self.stale_after)?;
        duration(self.connect_timeout)?;
        if self.allow_motion && !self.allow_control {
            return Err(CuError::from(
                "CRI allow_motion requires allow_control = true",
            ));
        }
        if !(self.velocity_percent > 0.0 && self.velocity_percent <= 25.0) {
            return Err(CuError::from(
                "CRI velocity_percent must be in 0 < v <= 25.0",
            ));
        }
        if let Some(step) = self.max_step {
            let step_deg = step.get::<degree>();
            if !step_deg.is_finite() || step_deg <= 0.0 {
                return Err(CuError::from("CRI max_step must be finite and positive"));
            }
        }
        let tol_deg = self.goal_tolerance.get::<degree>();
        if !tol_deg.is_finite() || tol_deg <= 0.0 {
            return Err(CuError::from(
                "CRI goal_tolerance must be finite and positive",
            ));
        }
        let full = self.jog_full_scale.get::<degree_per_second>();
        if !full.is_finite() || full <= 0.0 {
            return Err(CuError::from(
                "CRI jog_full_scale must be finite and positive",
            ));
        }
        if !(self.max_jog_percent > 0.0 && self.max_jog_percent <= 100.0) {
            return Err(CuError::from("CRI max_jog_percent must be in 0 < p <= 100"));
        }
        duration(self.command_timeout)?;
        if !(self.position_gain.is_finite() && self.position_gain >= 0.0) {
            return Err(CuError::from("CRI position_gain must be finite and >= 0"));
        }
        Ok(())
    }

    pub(crate) fn socket_address(&self) -> CuResult<SocketAddr> {
        self.address
            .parse::<SocketAddr>()
            .map_err(|e| CuError::new_with_cause("CRI requires a numeric socket address", e))
    }
}

pub(crate) fn duration(value: Time) -> CuResult<Duration> {
    let result = Duration::try_from_secs_f64(value.get::<second>())
        .map_err(|e| CuError::new_with_cause("CRI duration must be finite and positive", e))?;
    if result.is_zero() {
        return Err(CuError::from("CRI duration must be positive"));
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_validation() {
        assert!(CuIgusCriParams::default().validate().is_ok());
        for params in [
            CuIgusCriParams {
                stale_after: Time::new::<second>(f64::NAN),
                ..Default::default()
            },
            CuIgusCriParams {
                connect_timeout: Time::new::<second>(0.0),
                ..Default::default()
            },
            CuIgusCriParams {
                address: "rebel.local:3920".into(),
                ..Default::default()
            },
            CuIgusCriParams {
                allow_motion: true,
                allow_control: false,
                ..Default::default()
            },
            CuIgusCriParams {
                velocity_percent: 0.0,
                ..Default::default()
            },
            CuIgusCriParams {
                velocity_percent: -5.0,
                ..Default::default()
            },
            CuIgusCriParams {
                velocity_percent: 25.1,
                ..Default::default()
            },
            CuIgusCriParams {
                velocity_percent: f64::NAN,
                ..Default::default()
            },
            CuIgusCriParams {
                max_step: Some(Angle::new::<degree>(0.0)),
                ..Default::default()
            },
            CuIgusCriParams {
                max_step: Some(Angle::new::<degree>(f64::NAN)),
                ..Default::default()
            },
            CuIgusCriParams {
                goal_tolerance: Angle::new::<degree>(0.0),
                ..Default::default()
            },
            CuIgusCriParams {
                goal_tolerance: Angle::new::<degree>(-1.0),
                ..Default::default()
            },
        ] {
            assert!(params.validate().is_err());
        }
    }
}
