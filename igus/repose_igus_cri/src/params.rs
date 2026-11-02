use cu29::prelude::*;
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
}

impl Default for CuIgusCriParams {
    fn default() -> Self {
        Self {
            address: "192.168.3.11:3920".into(),
            stale_after: Time::new::<second>(0.5),
            connect_timeout: Time::new::<second>(2.0),
            allow_control: false,
        }
    }
}

impl CuIgusCriParams {
    /// Reject invalid addresses and non-positive or unrepresentable durations.
    pub fn validate(&self) -> CuResult<()> {
        self.socket_address()?;
        duration(self.stale_after)?;
        duration(self.connect_timeout)?;
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
        ] {
            assert!(params.validate().is_err());
        }
    }
}
