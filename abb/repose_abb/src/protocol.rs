//! Pure, bounded EGM protobuf (proto2, `egm.proto`) decoding and encoding.
//!
//! Only the fields this bridge uses are interpreted; unknown fields are
//! skipped. Joint values on the wire are degrees; this module converts.
use bincode::{Decode, Encode};
use cu29::prelude::*;
use cu29::units::si::{
    angle::{degree, radian},
    angular_velocity::radian_per_second,
    f64::{Angle, AngularVelocity, Time},
    time::{millisecond, second},
};
use serde::{Deserialize, Serialize};

/// Largest accepted `EgmRobot` datagram.
pub(crate) const CAPACITY: usize = 1400;
/// Encoded `EgmSensor` upper bound.
#[cfg(any(feature = "control", test))]
pub(crate) const SENSOR_CAPACITY: usize = 192;
const MAX_JOINTS: usize = 12;

#[derive(
    Default, Debug, Clone, Copy, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
pub enum CuAbbMotorState {
    #[default]
    Undefined,
    On,
    Off,
}
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
pub enum CuAbbMciState {
    #[default]
    Undefined,
    Error,
    Stopped,
    Running,
}
#[derive(
    Default, Debug, Clone, Copy, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect,
)]
pub enum CuAbbRapidState {
    #[default]
    Undefined,
    Stopped,
    Running,
}

/// One decoded `EgmRobot` message. Controller times are separate from CuMsg time.
#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct CuAbbStatus {
    /// `header.seqno`.
    pub sequence: u32,
    /// `header.tm`: controller milliseconds, wraps at 2³².
    pub controller_time: Time,
    /// `feedBack.time` (sec + usec) when the controller sends it, else zero.
    pub feedback_time: Time,
    pub joints: [Angle; 6],
    /// Host-side finite difference of consecutive feedback; zero on the first sample.
    pub velocities: [AngularVelocity; 6],
    /// `planned.joints`; zero when absent.
    pub planned: [Angle; 6],
    pub motors: CuAbbMotorState,
    pub mci: CuAbbMciState,
    pub rapid: CuAbbRapidState,
    pub convergence_met: bool,
    pub utilization: f64,
}
impl CuAbbStatus {
    /// Motors on, RAPID running and EGM not in error.
    pub(crate) fn ready(&self) -> bool {
        self.motors == CuAbbMotorState::On
            && self.rapid == CuAbbRapidState::Running
            && self.mci != CuAbbMciState::Error
    }
    pub(crate) fn joints_rad(&self) -> [f64; 6] {
        self.joints.map(|a| a.get::<radian>())
    }
    /// Seconds used to differentiate consecutive samples.
    pub(crate) fn sample_seconds(&self) -> f64 {
        let feedback = self.feedback_time.get::<second>();
        if feedback > 0.0 {
            feedback
        } else {
            self.controller_time.get::<second>()
        }
    }
    pub(crate) fn set_velocities(&mut self, previous: Option<&CuAbbStatus>) {
        let Some(previous) = previous else { return };
        let dt = self.sample_seconds() - previous.sample_seconds();
        if !(dt > 0.0 && dt <= 0.5) {
            return;
        }
        let (now, before) = (self.joints_rad(), previous.joints_rad());
        self.velocities = std::array::from_fn(|i| {
            AngularVelocity::new::<radian_per_second>((now[i] - before[i]) / dt)
        });
    }
}

enum Value<'a> {
    Varint(u64),
    Fixed64([u8; 8]),
    Bytes(&'a [u8]),
}
struct Reader<'a> {
    data: &'a [u8],
    at: usize,
}
fn malformed() -> CuError {
    CuError::from("ABB malformed EGM message")
}
impl<'a> Reader<'a> {
    fn new(data: &'a [u8]) -> Self {
        Self { data, at: 0 }
    }
    fn varint(&mut self) -> CuResult<u64> {
        let mut value = 0u64;
        for shift in (0..70).step_by(7) {
            let byte = *self.data.get(self.at).ok_or_else(malformed)?;
            self.at += 1;
            if shift == 63 && byte > 1 {
                return Err(malformed());
            }
            value |= u64::from(byte & 0x7f) << shift;
            if byte & 0x80 == 0 {
                return Ok(value);
            }
        }
        Err(malformed())
    }
    fn take(&mut self, n: usize) -> CuResult<&'a [u8]> {
        let end = self.at.checked_add(n).ok_or_else(malformed)?;
        let bytes = self.data.get(self.at..end).ok_or_else(malformed)?;
        self.at = end;
        Ok(bytes)
    }
    fn next(&mut self) -> CuResult<Option<(u64, Value<'a>)>> {
        if self.at == self.data.len() {
            return Ok(None);
        }
        let key = self.varint()?;
        let field = key >> 3;
        if field == 0 {
            return Err(malformed());
        }
        let value = match key & 7 {
            0 => Value::Varint(self.varint()?),
            1 => Value::Fixed64(self.take(8)?.try_into().unwrap()),
            2 => {
                let n = usize::try_from(self.varint()?).map_err(|_| malformed())?;
                Value::Bytes(self.take(n)?)
            }
            5 => {
                self.take(4)?;
                return self.next();
            }
            _ => return Err(malformed()),
        };
        Ok(Some((field, value)))
    }
}

/// Repeated double field 1 (`EgmJoints.joints`), packed or not.
fn joints(data: &[u8]) -> CuResult<([f64; MAX_JOINTS], usize)> {
    let mut out = [0.0; MAX_JOINTS];
    let mut count = 0;
    let mut push = |bytes: [u8; 8]| {
        let value = f64::from_le_bytes(bytes);
        if count == MAX_JOINTS || !value.is_finite() {
            return Err(malformed());
        }
        out[count] = value;
        count += 1;
        Ok(())
    };
    let mut reader = Reader::new(data);
    while let Some((field, value)) = reader.next()? {
        match (field, value) {
            (1, Value::Fixed64(bytes)) => push(bytes)?,
            (1, Value::Bytes(packed)) => {
                let (chunks, rest) = packed.as_chunks::<8>();
                if !rest.is_empty() {
                    return Err(malformed());
                }
                for bytes in chunks {
                    push(*bytes)?;
                }
            }
            (1, _) => return Err(malformed()),
            _ => {}
        }
    }
    Ok((out, count))
}
fn six(data: &[u8]) -> CuResult<[Angle; 6]> {
    let (values, count) = joints(data)?;
    if count != 6 {
        return Err(CuError::from("ABB EGM requires exactly six robot joints"));
    }
    Ok(std::array::from_fn(|i| Angle::new::<degree>(values[i])))
}
fn nested_varint(data: &[u8], wanted: u64) -> CuResult<Option<u64>> {
    let mut reader = Reader::new(data);
    let mut found = None;
    while let Some((field, value)) = reader.next()? {
        if field == wanted {
            let Value::Varint(v) = value else {
                return Err(malformed());
            };
            found = Some(v);
        }
    }
    Ok(found)
}
fn bytes(value: Value<'_>) -> CuResult<&[u8]> {
    match value {
        Value::Bytes(b) => Ok(b),
        _ => Err(malformed()),
    }
}
/// `EgmFeedBack` / `EgmPlanned`: joints (1) and clock (4).
fn joint_message(data: &[u8]) -> CuResult<(Option<[Angle; 6]>, Option<Time>)> {
    let (mut angles, mut time) = (None, None);
    let mut reader = Reader::new(data);
    while let Some((field, value)) = reader.next()? {
        match field {
            1 => angles = Some(six(bytes(value)?)?),
            4 => {
                let clock = bytes(value)?;
                let sec = nested_varint(clock, 1)?.unwrap_or(0);
                let usec = nested_varint(clock, 2)?.unwrap_or(0);
                if usec >= 1_000_000 {
                    return Err(malformed());
                }
                time = Some(Time::new::<second>(sec as f64 + usec as f64 * 1e-6));
            }
            _ => {}
        }
    }
    Ok((angles, time))
}

/// Decode one `EgmRobot` datagram. Feedback joints are required.
pub(crate) fn decode(data: &[u8]) -> CuResult<CuAbbStatus> {
    if data.len() > CAPACITY {
        return Err(CuError::from("ABB EGM datagram exceeds capacity"));
    }
    let mut status = CuAbbStatus::default();
    let mut have_feedback = false;
    let mut reader = Reader::new(data);
    while let Some((field, value)) = reader.next()? {
        match field {
            1 => {
                let header = bytes(value)?;
                let seq = nested_varint(header, 1)?.unwrap_or(0);
                let tm = nested_varint(header, 2)?.unwrap_or(0);
                status.sequence = u32::try_from(seq).map_err(|_| malformed())?;
                status.controller_time =
                    Time::new::<millisecond>(u32::try_from(tm).map_err(|_| malformed())? as f64);
            }
            2 => {
                let (angles, time) = joint_message(bytes(value)?)?;
                status.joints =
                    angles.ok_or_else(|| CuError::from("ABB EGM feedback lacks joints"))?;
                status.feedback_time = time.unwrap_or_default();
                have_feedback = true;
            }
            3 => {
                if let (Some(angles), _) = joint_message(bytes(value)?)? {
                    status.planned = angles;
                }
            }
            4 => {
                status.motors = match nested_varint(bytes(value)?, 1)? {
                    Some(1) => CuAbbMotorState::On,
                    Some(2) => CuAbbMotorState::Off,
                    _ => CuAbbMotorState::Undefined,
                }
            }
            5 => {
                status.mci = match nested_varint(bytes(value)?, 1)? {
                    Some(1) => CuAbbMciState::Error,
                    Some(2) => CuAbbMciState::Stopped,
                    Some(3) => CuAbbMciState::Running,
                    _ => CuAbbMciState::Undefined,
                }
            }
            6 => {
                let Value::Varint(v) = value else {
                    return Err(malformed());
                };
                status.convergence_met = v != 0;
            }
            8 => {
                status.rapid = match nested_varint(bytes(value)?, 1)? {
                    Some(1) => CuAbbRapidState::Stopped,
                    Some(2) => CuAbbRapidState::Running,
                    _ => CuAbbRapidState::Undefined,
                }
            }
            10 => {
                let Value::Fixed64(b) = value else {
                    return Err(malformed());
                };
                let value = f64::from_le_bytes(b);
                if !value.is_finite() {
                    return Err(malformed());
                }
                status.utilization = value;
            }
            _ => {}
        }
    }
    if !have_feedback {
        return Err(CuError::from("ABB EGM message lacks feedback"));
    }
    Ok(status)
}

#[cfg(any(feature = "control", test))]
struct Writer<'a> {
    out: &'a mut [u8; SENSOR_CAPACITY],
    len: usize,
}
#[cfg(any(feature = "control", test))]
impl Writer<'_> {
    fn byte(&mut self, b: u8) {
        self.out[self.len] = b;
        self.len += 1;
    }
    fn varint(&mut self, mut v: u64) {
        while v >= 0x80 {
            self.byte(v as u8 | 0x80);
            v >>= 7;
        }
        self.byte(v as u8);
    }
    /// Length-delimited submessage; every submessage here is < 128 bytes.
    fn nested(&mut self, field: u8, body: impl FnOnce(&mut Self)) {
        self.byte(field << 3 | 2);
        let length_at = self.len;
        self.byte(0);
        body(self);
        let length = self.len - length_at - 1;
        debug_assert!(length < 128);
        self.out[length_at] = length as u8;
    }
    fn joints(&mut self, field: u8, values: &[f64; 6]) {
        self.nested(field, |w| {
            w.nested(1, |w| {
                for v in values {
                    w.byte(1 << 3 | 1);
                    for b in v.to_le_bytes() {
                        w.byte(b);
                    }
                }
            })
        });
    }
}

#[cfg(any(feature = "control", test))]
/// Encode an `EgmSensor` correction: planned joints (deg) and speed reference (deg/s).
pub(crate) fn encode_sensor(
    sequence: u32,
    time_ms: u32,
    position_rad: [f64; 6],
    speed_rad_s: [f64; 6],
    out: &mut [u8; SENSOR_CAPACITY],
) -> usize {
    let mut w = Writer { out, len: 0 };
    w.nested(1, |w| {
        w.byte(1 << 3);
        w.varint(sequence.into());
        w.byte(2 << 3);
        w.varint(time_ms.into());
        w.byte(3 << 3);
        w.varint(3); // MSGTYPE_CORRECTION
    });
    w.joints(2, &position_rad.map(f64::to_degrees));
    w.joints(3, &speed_rad_s.map(f64::to_degrees));
    w.len
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// Minimal encoder for test `EgmRobot` messages.
    pub(crate) struct Robot {
        pub seq: u32,
        pub tm: u32,
        pub joints_deg: Vec<f64>,
        pub packed: bool,
        pub motors: u64,
        pub mci: u64,
        pub rapid: u64,
    }
    impl Default for Robot {
        fn default() -> Self {
            Self {
                seq: 1,
                tm: 1000,
                joints_deg: vec![0.0; 6],
                packed: false,
                motors: 1,
                mci: 2,
                rapid: 2,
            }
        }
    }
    fn varint(out: &mut Vec<u8>, mut v: u64) {
        while v >= 0x80 {
            out.push(v as u8 | 0x80);
            v >>= 7;
        }
        out.push(v as u8);
    }
    fn nested(out: &mut Vec<u8>, field: u8, body: &[u8]) {
        out.push(field << 3 | 2);
        varint(out, body.len() as u64);
        out.extend_from_slice(body);
    }
    fn enum_msg(out: &mut Vec<u8>, field: u8, value: u64) {
        let mut body = vec![8];
        varint(&mut body, value);
        nested(out, field, &body);
    }
    impl Robot {
        pub(crate) fn encode(&self) -> Vec<u8> {
            let mut out = Vec::new();
            let mut header = vec![8];
            varint(&mut header, self.seq.into());
            header.push(16);
            varint(&mut header, self.tm.into());
            header.extend_from_slice(&[24, 2]);
            nested(&mut out, 1, &header);
            let mut joints = Vec::new();
            if self.packed {
                let data: Vec<u8> = self
                    .joints_deg
                    .iter()
                    .flat_map(|v| v.to_le_bytes())
                    .collect();
                nested(&mut joints, 1, &data);
            } else {
                for v in &self.joints_deg {
                    joints.push(9);
                    joints.extend_from_slice(&v.to_le_bytes());
                }
            }
            let mut feedback = Vec::new();
            nested(&mut feedback, 1, &joints);
            nested(&mut out, 2, &feedback);
            nested(&mut out, 3, &feedback);
            enum_msg(&mut out, 4, self.motors);
            enum_msg(&mut out, 5, self.mci);
            out.extend_from_slice(&[6 << 3, 1]);
            enum_msg(&mut out, 8, self.rapid);
            out.push(10 << 3 | 1);
            out.extend_from_slice(&0.25f64.to_le_bytes());
            // Unknown field 99 (varint) must be skipped.
            varint(&mut out, 99 << 3);
            out.push(5);
            out
        }
    }

    #[test]
    fn decodes_packed_and_unpacked() {
        for packed in [false, true] {
            let robot = Robot {
                joints_deg: vec![10.0, 20.0, 30.0, 40.0, 50.0, 180.0],
                packed,
                ..Default::default()
            };
            let status = decode(&robot.encode()).unwrap();
            assert_eq!(status.sequence, 1);
            assert_eq!(status.controller_time.get::<millisecond>(), 1000.0);
            assert!((status.joints[5].get::<radian>() - std::f64::consts::PI).abs() < 1e-12);
            assert_eq!(status.planned[0], status.joints[0]);
            assert_eq!(status.motors, CuAbbMotorState::On);
            assert_eq!(status.mci, CuAbbMciState::Stopped);
            assert_eq!(status.rapid, CuAbbRapidState::Running);
            assert!(status.convergence_met);
            assert_eq!(status.utilization, 0.25);
            assert!(status.ready());
        }
    }

    #[test]
    fn rejects_malformed() {
        let good = Robot::default().encode();
        for n in 0..good.len() {
            let _ = decode(&good[..n]); // never panics
        }
        assert!(decode(&good[..good.len() - 1]).is_err()); // truncated trailing field
        assert!(decode(&good[..20]).is_err()); // inside feedback
        for joints in [vec![0.0; 5], vec![0.0; 7], vec![f64::NAN; 6]] {
            let robot = Robot {
                joints_deg: joints,
                ..Default::default()
            };
            assert!(decode(&robot.encode()).is_err());
        }
        assert!(decode(&[0x0b, 0]).is_err()); // group wire type
        assert!(decode(&[0xff; 11]).is_err()); // overlong varint
        assert!(decode(&vec![0; CAPACITY + 1]).is_err());
    }

    #[test]
    fn sensor_round_trips_through_reader() {
        let mut out = [0; SENSOR_CAPACITY];
        let position = [0.1, -0.2, 0.3, 0.0, 1.0, -1.0];
        let speed = [0.5; 6];
        let n = encode_sensor(7, 40, position, speed, &mut out);
        let mut reader = Reader::new(&out[..n]);
        let mut fields = Vec::new();
        while let Some((field, value)) = reader.next().unwrap() {
            fields.push(field);
            let data = bytes(value).unwrap();
            match field {
                1 => {
                    assert_eq!(nested_varint(data, 1).unwrap(), Some(7));
                    assert_eq!(nested_varint(data, 3).unwrap(), Some(3));
                }
                2 => {
                    let (angles, _) = joint_message(data).unwrap();
                    let angles = angles.unwrap().map(|a| a.get::<radian>());
                    for (a, b) in angles.iter().zip(position) {
                        assert!((a - b).abs() < 1e-12);
                    }
                }
                3 => {
                    let (angles, _) = joint_message(data).unwrap();
                    assert!(
                        (angles.unwrap()[0].get::<degree>() - 0.5f64.to_degrees()).abs() < 1e-9
                    );
                }
                _ => panic!("unexpected field"),
            }
        }
        assert_eq!(fields, [1, 2, 3]);
        assert!(n < SENSOR_CAPACITY);
    }
}
