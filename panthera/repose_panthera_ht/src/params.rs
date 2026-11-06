use cu29::prelude::*;
use cu29::units::si::{
    angle::radian,
    f64::{Angle, Time},
    time::second,
};
use serde::{Deserialize, Serialize};
use std::time::Duration;

/// Number of arm joints (the Panthera HT gripper is a separate optional motor).
pub const JOINTS: usize = 6;
/// Largest motor CAN id the protocol supports (`serial_struct.h`).
pub const MAX_MOTOR_ID: u8 = 30;

/// Shared Rust / RON configuration. Durations and angles use Copper SI units.
#[derive(Debug, Clone, Serialize, Deserialize, Reflect)]
#[serde(default, deny_unknown_fields)]
pub struct CuPantheraHtParams {
    /// Serial device of the comm board; empty means auto-detect the first
    /// USB port with VID:PID `CAF1:FFFF`.
    pub serial_port: String,
    /// 1-based index into the detected comm boards when auto-detecting.
    pub can_port_id: u8,
    /// Serial baud rate; the v6 comm board uses 4 Mbaud.
    pub baud: u32,
    /// CAN ids of the six arm joints, slot order = joint order.
    pub joint_ids: [u8; JOINTS],
    /// Motor state request rate in Hz.
    pub frequency: f64,
    /// Maximum age of an observation published to Copper.
    pub stale_after: Time,
    /// Startup connection timeout.
    pub connect_timeout: Time,
    /// Position commands older than this stop being re-sent.
    pub command_timeout: Time,
    /// Second opt-in for the `control` feature: session and motion commands
    /// are refused unless this is true.
    pub allow_control: bool,
    /// Third opt-in: motion commands are refused unless this is true.
    /// Requires `allow_control = true`.
    pub allow_motion: bool,
    /// Largest step between target and measured angle per joint.
    pub max_step: Angle,
    /// MIT-mode position gain sent with every motion frame (wire scale 100/rad).
    pub kp: f64,
    /// MIT-mode damping gain sent with every motion frame (wire scale 100/rad).
    pub kd: f64,
}
impl Default for CuPantheraHtParams {
    fn default() -> Self {
        Self {
            serial_port: String::new(),
            can_port_id: 1,
            baud: 4_000_000,
            joint_ids: [1, 2, 3, 4, 5, 6],
            frequency: 50.0,
            stale_after: Time::new::<second>(0.5),
            connect_timeout: Time::new::<second>(2.0),
            command_timeout: Time::new::<second>(0.1),
            allow_control: false,
            allow_motion: false,
            max_step: Angle::new::<radian>(std::f64::consts::PI),
            kp: 20.0,
            kd: 2.0,
        }
    }
}
impl CuPantheraHtParams {
    pub fn validate(&self) -> CuResult<()> {
        if self.serial_port.len() > 256 {
            return Err(CuError::from("Panthera serial_port path too long"));
        }
        if self.can_port_id == 0 {
            return Err(CuError::from(
                "Panthera can_port_id is 1-based, must be >= 1",
            ));
        }
        if !(self.baud == 4_000_000 || (1_000..=12_000_000).contains(&self.baud)) {
            return Err(CuError::from("Panthera baud must be 1000..=12000000"));
        }
        for id in self.joint_ids {
            if id == 0 || id > MAX_MOTOR_ID {
                return Err(CuError::from("Panthera joint_ids must be in 1..=30"));
            }
        }
        let mut sorted = self.joint_ids;
        sorted.sort_unstable();
        if sorted.windows(2).any(|w| w[0] == w[1]) {
            return Err(CuError::from("Panthera joint_ids must be distinct"));
        }
        duration(self.stale_after)?;
        duration(self.connect_timeout)?;
        duration(self.command_timeout)?;
        if self.allow_motion && !self.allow_control {
            return Err(CuError::from(
                "Panthera allow_motion requires allow_control = true",
            ));
        }
        if !(self.frequency.is_finite() && (1.0..=500.0).contains(&self.frequency)) {
            return Err(CuError::from("Panthera frequency must be 1..=500 Hz"));
        }
        let max_step = self.max_step.get::<radian>();
        if !(max_step.is_finite() && max_step > 0.0 && max_step <= std::f64::consts::PI) {
            return Err(CuError::from("Panthera max_step must be in 0..=pi rad"));
        }
        // Wire kp/kd are int16 with scale PID_SCALE=100 per radian (see protocol).
        for (name, value) in [("kp", self.kp), ("kd", self.kd)] {
            if !(value.is_finite()
                && (0.0..=32765.0 / (100.0 * std::f64::consts::TAU)).contains(&value))
            {
                return Err(CuError::from(format!(
                    "Panthera {name} outside permitted range"
                )));
            }
        }
        Ok(())
    }
}
pub(crate) fn duration(value: Time) -> CuResult<Duration> {
    let seconds = value.get::<second>();
    if !(seconds.is_finite() && seconds > 0.0 && seconds <= 3600.0) {
        return Err(CuError::from(
            "Panthera duration must be in 0..=3600 seconds",
        ));
    }
    Ok(Duration::from_secs_f64(seconds))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn validation() {
        assert!(CuPantheraHtParams::default().validate().is_ok());
        assert!(
            CuPantheraHtParams {
                joint_ids: [1, 1, 3, 4, 5, 6],
                ..Default::default()
            }
            .validate()
            .is_err()
        );
        assert!(
            CuPantheraHtParams {
                joint_ids: [0, 2, 3, 4, 5, 6],
                ..Default::default()
            }
            .validate()
            .is_err()
        );
        assert!(
            CuPantheraHtParams {
                joint_ids: [1, 2, 3, 4, 5, 31],
                ..Default::default()
            }
            .validate()
            .is_err()
        );
        assert!(
            CuPantheraHtParams {
                allow_motion: true,
                ..Default::default()
            }
            .validate()
            .is_err()
        );
        assert!(
            CuPantheraHtParams {
                frequency: f64::NAN,
                ..Default::default()
            }
            .validate()
            .is_err()
        );
        assert!(
            CuPantheraHtParams {
                baud: 0,
                ..Default::default()
            }
            .validate()
            .is_err()
        );
        assert!(
            CuPantheraHtParams {
                kp: -1.0,
                ..Default::default()
            }
            .validate()
            .is_err()
        );
    }
}
