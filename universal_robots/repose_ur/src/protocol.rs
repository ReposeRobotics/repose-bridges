//! Pure, bounded RTDE v2 decoding. All numbers on the wire are big endian.
use bincode::{Decode, Encode};
use cu29::prelude::*;
use cu29::units::si::{
    angle::radian,
    angular_velocity::radian_per_second,
    f64::{Angle, AngularVelocity, Time},
    time::second,
};
use serde::{Deserialize, Serialize};

pub(crate) const CAPACITY: usize = 2048;
pub(crate) const FIELDS: &[u8] =
    b"timestamp,actual_q,actual_qd,robot_mode,safety_mode,runtime_state,output_int_register_24";
pub(crate) const TYPES: &[u8] = b"DOUBLE,VECTOR6D,VECTOR6D,INT32,INT32,UINT32,INT32";

/// Raw controller status; controller uptime is deliberately separate from CuMsg time.
#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct CuUrStatus {
    pub controller_uptime: Time,
    pub joints: [Angle; 6],
    pub velocities: [AngularVelocity; 6],
    pub robot_mode: i32,
    pub safety_mode: i32,
    pub runtime_state: u32,
    pub control_marker: i32,
}
impl CuUrStatus {
    pub(crate) fn ready(&self) -> bool {
        self.robot_mode == 7 && matches!(self.safety_mode, 1 | 2)
    }
}

pub(crate) fn decode(body: &[u8], recipe: u8) -> CuResult<CuUrStatus> {
    if body.len() != 121 || body[0] != recipe {
        return Err(CuError::from("UR invalid RTDE data length or recipe"));
    }
    let mut values = [0.0; 13];
    for (out, bytes) in values.iter_mut().zip(body[1..105].as_chunks::<8>().0) {
        *out = f64::from_be_bytes(*bytes);
        if !out.is_finite() {
            return Err(CuError::from("UR nonfinite telemetry"));
        }
    }
    if values[0] < 0.0 {
        return Err(CuError::from("UR negative uptime"));
    }
    Ok(CuUrStatus {
        controller_uptime: Time::new::<second>(values[0]),
        joints: std::array::from_fn(|i| Angle::new::<radian>(values[i + 1])),
        velocities: std::array::from_fn(|i| {
            AngularVelocity::new::<radian_per_second>(values[i + 7])
        }),
        robot_mode: i32::from_be_bytes(body[105..109].try_into().unwrap()),
        safety_mode: i32::from_be_bytes(body[109..113].try_into().unwrap()),
        runtime_state: u32::from_be_bytes(body[113..117].try_into().unwrap()),
        control_marker: i32::from_be_bytes(body[117..121].try_into().unwrap()),
    })
}

#[cfg(feature = "control")]
pub(crate) fn move_script(target: [f64; 6], speed: f64, acceleration: f64) -> String {
    format!(
        "def repose_move():\n  movej([{},{},{},{},{},{}], a={}, v={}, t=0, r=0)\nend\n",
        target[0], target[1], target[2], target[3], target[4], target[5], acceleration, speed
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn decode_rejects_bad_data() {
        let mut data = [0; 121];
        data[0] = 3;
        assert!(decode(&data, 3).is_ok());
        assert!(decode(&data, 2).is_err());
        for n in 0..121 {
            assert!(decode(&data[..n], 3).is_err());
        }
        data[1..9].copy_from_slice(&f64::NAN.to_be_bytes());
        assert!(decode(&data, 3).is_err());
    }
}
