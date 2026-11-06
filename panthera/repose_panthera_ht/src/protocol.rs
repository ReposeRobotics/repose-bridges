//! Pure, bounded codec for the v6 comm-board serial protocol.
//!
//! Wire format (from `HighTorque-Robotics/hightorque_robot`, `serial_struct.h`
//! and `canport.cpp`): every frame is
//! `0xF7 | cmd(u8) | len(u16 LE) | crc_head(u8) | crc_data(u8) | data[len]`
//! where `crc_head` covers `cmd,len` and `crc_data` covers `data`, both with
//! the CRC-8/CCITT table below (init 0xFF). RX frames prefix `data` with the
//! 3-byte CAN-FD port state; TX frames carry the bare command body.
//! All multi-byte fields are little endian; angles are int16 turns
//! (rad / 2π) times a scale.
use bincode::{Decode, Encode};
use cu29::prelude::*;
use cu29::units::si::{
    angle::radian,
    angular_velocity::radian_per_second,
    f64::{Angle, AngularVelocity, Torque},
    torque::newton_meter,
};
use serde::{Deserialize, Serialize};

/// Frame header byte.
pub(crate) const PROT_HEAD: u8 = 0xF7;
/// Maximum data segment of a frame (`PROT_DATA_LEN`).
pub(crate) const PROT_DATA_LEN: usize = 506;
/// Largest frame on the wire: header + data.
pub(crate) const CAPACITY: usize = 6 + PROT_DATA_LEN;

pub(crate) const MODE_COMM_VERSION: u8 = 1;
pub(crate) const MODE_COMM_INIT: u8 = 2;
pub(crate) const MODE_CACHE_NUM: u8 = 4;
pub(crate) const MODE_MOTOR_STATE: u8 = 20;
#[cfg(any(feature = "control", test))]
pub(crate) const MODE_POS_VEL_TQE_KP_KD: u8 = 0x98;

/// `contr_t.data_type` for int16 payloads without per-value headers.
pub(crate) const TINT16_NOHDR: u8 = 0;
/// `contr_t.query`: motor mode/fault/pos/vel/tqe records.
pub(crate) const QUERY_MODE_FAULT_POS_VEL_TQE: u8 = 11;

/// int16 turns per radian for positions (`POS_SCALE`).
const POS_SCALE: f64 = 10_000.0;
/// int16 turns per rad/s for velocities (`VEL_SCALE`).
const VEL_SCALE: f64 = 4_000.0;
/// int16 per Nm for torque (`TQE_SCALE`).
const TQE_SCALE: f64 = 100.0;
/// int16 per rad for kp/kd (`PID_SCALE`); the SDK also multiplies by 2π.
#[cfg(any(feature = "control", test))]
const PID_SCALE: f64 = 100.0;
/// int16 clamp used by the SDK (`int16_limit`).
#[cfg(any(feature = "control", test))]
const INT16_LIMIT: f64 = 32765.0;

/// One motor's state as reported in a `MODE_MOTOR_STATE` frame.
#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct CuPantheraMotor {
    pub id: u8,
    pub mode: u8,
    pub fault: u8,
    pub position: Angle,
    pub velocity: AngularVelocity,
    pub torque: Torque,
}

/// Raw comm-board status; angles/velocities are motor-side, CuMsg time is separate.
#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct CuPantheraStatus {
    /// Comm-board firmware version as (major, minor, patch).
    pub comm_version: [u8; 3],
    /// CAN-FD port fault code (`fdcan_state_s.fault`, -1 = unknown).
    pub can_fault: i8,
    pub tx_err: u8,
    pub rx_err: u8,
    /// Arm joints in `joint_ids` order.
    pub motors: [CuPantheraMotor; crate::params::JOINTS],
}

const CRC8_TABLE: [u8; 256] = [
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
];

/// CRC-8/CCITT as table-driven in the vendor SDK (`crc8_ccitt`, init 0xFF).
pub(crate) fn crc8(data: &[u8]) -> u8 {
    let mut crc = 0xFFu8;
    for &byte in data {
        crc = CRC8_TABLE[(crc ^ byte) as usize];
    }
    crc
}

#[cfg(any(feature = "control", test))]
fn clamp_i16(value: f64) -> i16 {
    let clamped = value.clamp(-INT16_LIMIT, INT16_LIMIT);
    clamped as i16
}

/// Radians → int16 turns (position), truncating like the SDK's cast.
#[cfg(any(feature = "control", test))]
pub(crate) fn pos_to_i16(rad: f64) -> i16 {
    clamp_i16(rad / std::f64::consts::TAU * POS_SCALE)
}
/// int16 turns → radians (position).
pub(crate) fn pos_from_i16(raw: i16) -> f64 {
    f64::from(raw) / POS_SCALE * std::f64::consts::TAU
}
/// Rad/s → int16 turns/s (velocity).
#[cfg(any(feature = "control", test))]
#[allow(dead_code)]
pub(crate) fn vel_to_i16(rad_s: f64) -> i16 {
    clamp_i16(rad_s / std::f64::consts::TAU * VEL_SCALE)
}
/// int16 turns/s → rad/s (velocity).
pub(crate) fn vel_from_i16(raw: i16) -> f64 {
    f64::from(raw) / VEL_SCALE * std::f64::consts::TAU
}
/// Nm → int16 torque.
#[cfg(any(feature = "control", test))]
#[allow(dead_code)]
pub(crate) fn tqe_to_i16(nm: f64) -> i16 {
    clamp_i16(nm * TQE_SCALE)
}
/// int16 → Nm torque.
pub(crate) fn tqe_from_i16(raw: i16) -> f64 {
    f64::from(raw) / TQE_SCALE
}
/// MIT kp/kd (per rad) → int16, matching the SDK's `kp_float2int`.
#[cfg(any(feature = "control", test))]
pub(crate) fn pid_to_i16(value: f64) -> i16 {
    clamp_i16(value * std::f64::consts::TAU * PID_SCALE)
}

/// Data length of a TX motor-command frame, including the 2-byte `contr_t`
/// header and CAN-FD DLC padding (`CanPort::get_data_len`).
#[cfg(any(feature = "control", test))]
pub(crate) fn command_data_len(mode: u8, id_max: u8) -> CuResult<usize> {
    let per_motor = match mode {
        0x85 | 0x86 => 1, // STOP, BRAKE
        0x80..=0x84 => 2, // POSITION..CURRENT
        0x90 => 4,        // VEL_ACC
        0x92 | 0x95 => 6, // POS_VEL_TQE, POS_VEL_ACC
        MODE_POS_VEL_TQE_KP_KD => 10,
        _ => return Err(CuError::from("Panthera unknown command mode")),
    };
    let len = per_motor as usize * id_max as usize;
    let mul = len / 60;
    let rem = len % 60;
    let rem_len = match rem {
        0..=7 => rem,
        8..=11 => 11,
        12..=15 => 15,
        16..=19 => 19,
        20..=23 => 23,
        24..=31 => 31,
        32..=47 => 47,
        _ => 60,
    };
    Ok(mul * 60 + rem_len + 2)
}

/// Serialize a frame into `out` (`out.len()` must be exactly `6 + data.len()`).
pub(crate) fn encode_frame(out: &mut [u8], cmd: u8, data: &[u8]) -> CuResult<()> {
    if data.len() > PROT_DATA_LEN || out.len() != 6 + data.len() {
        return Err(CuError::from("Panthera frame outside capacity"));
    }
    out[0] = PROT_HEAD;
    out[1] = cmd;
    out[2..4].copy_from_slice(&(data.len() as u16).to_le_bytes());
    out[4] = crc8(&out[1..4]);
    out[5] = crc8(data);
    out[6..].copy_from_slice(data);
    Ok(())
}

/// A decoded `MODE_MOTOR_STATE` frame body.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct MotorStateFrame {
    pub can_fault: i8,
    pub tx_err: u8,
    pub rx_err: u8,
    pub query: u8,
    /// (id, fault, mode, pos, vel, tqe) raw records.
    pub records: [(u8, u8, u8, i16, i16, i16); 30],
    pub count: usize,
}

/// A decoded RX frame body.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum Frame {
    CommVersion([u8; 3]),
    CommInit(u8),
    CacheNum(u8),
    MotorState(Box<MotorStateFrame>),
    Other,
}

/// Parse one complete frame (`frame.len() == 6 + len`), rejecting bad CRC or size.
pub(crate) fn decode_frame(frame: &[u8]) -> CuResult<(u8, Frame)> {
    if frame.len() < 6 || frame[0] != PROT_HEAD {
        return Err(CuError::from("Panthera frame header invalid"));
    }
    let cmd = frame[1];
    let len = u16::from_le_bytes([frame[2], frame[3]]) as usize;
    if frame.len() != 6 + len || len > PROT_DATA_LEN {
        return Err(CuError::from("Panthera frame length outside capacity"));
    }
    if frame[4] != crc8(&frame[1..4]) || frame[5] != crc8(&frame[6..]) {
        return Err(CuError::from("Panthera frame CRC mismatch"));
    }
    let data = &frame[6..];
    // RX frames prefix the 3-byte CAN-FD port state.
    if data.len() < 3 {
        return Ok((cmd, Frame::Other));
    }
    let can_fault = data[0] as i8;
    let tx_err = data[1];
    let rx_err = data[2];
    let body = &data[3..];
    let parsed = match cmd {
        MODE_COMM_VERSION if body.len() >= 3 => {
            Frame::CommVersion([body[2], body[1], body[0]]) // major, minor, patch
        }
        MODE_COMM_INIT if !body.is_empty() => Frame::CommInit(body[0]),
        MODE_CACHE_NUM if !body.is_empty() => Frame::CacheNum(body[0]),
        MODE_MOTOR_STATE if !body.is_empty() => {
            // Packed `query_mode_fault_pos_vel_tqe_t` is 9 bytes per motor.
            let query = body[0];
            let mut records = [(0u8, 0u8, 0u8, 0i16, 0i16, 0i16); 30];
            let count = (body.len() - 1) / 9;
            if count > 30 || !(body.len() - 1).is_multiple_of(9) {
                return Err(CuError::from(
                    "Panthera motor state record outside capacity",
                ));
            }
            for (i, chunk) in body[1..].as_chunks::<9>().0.iter().enumerate() {
                records[i] = (
                    chunk[0],
                    chunk[1],
                    chunk[2],
                    i16::from_le_bytes([chunk[3], chunk[4]]),
                    i16::from_le_bytes([chunk[5], chunk[6]]),
                    i16::from_le_bytes([chunk[7], chunk[8]]),
                );
            }
            Frame::MotorState(Box::new(MotorStateFrame {
                can_fault,
                tx_err,
                rx_err,
                query,
                records,
                count,
            }))
        }
        _ => Frame::Other,
    };
    Ok((cmd, parsed))
}

/// Build the data segment of a `MODE_MOTOR_STATE` request (2 bytes).
pub(crate) fn encode_state_request() -> [u8; 2] {
    [TINT16_NOHDR, QUERY_MODE_FAULT_POS_VEL_TQE]
}

/// Build a `MODE_POS_VEL_TQE_KP_KD` data segment. `targets[i]` is the desired
/// joint position in radians for `joint_ids[i]`; velocity/torque are zero.
/// Unused motor slots are `i16::MIN`, matching the SDK's `0x8000` fill.
#[cfg(any(feature = "control", test))]
pub(crate) fn encode_position_command(
    targets: &[f64; crate::params::JOINTS],
    joint_ids: &[u8; crate::params::JOINTS],
    kp: f64,
    kd: f64,
) -> CuResult<Vec<u8>> {
    let id_max = joint_ids.iter().copied().max().unwrap_or(0);
    let len = command_data_len(MODE_POS_VEL_TQE_KP_KD, id_max)?;
    let mut data = vec![0u8; len];
    // Unused i16 slots are 0x8000, matching the SDK's raw16 fill.
    for chunk in data[2..].as_chunks_mut::<2>().0 {
        chunk.copy_from_slice(&i16::MIN.to_le_bytes());
    }
    data[0] = TINT16_NOHDR;
    data[1] = QUERY_MODE_FAULT_POS_VEL_TQE;
    let kp_raw = pid_to_i16(kp);
    let kd_raw = pid_to_i16(kd);
    for (&id, &target) in joint_ids.iter().zip(targets) {
        if !target.is_finite() {
            return Err(CuError::from("Panthera nonfinite joint target"));
        }
        let base = 2 + (id as usize - 1) * 10;
        let entry = [pos_to_i16(target), 0, 0, kp_raw, kd_raw];
        for (i, value) in entry.iter().enumerate() {
            data[base + 2 * i..base + 2 * i + 2].copy_from_slice(&value.to_le_bytes());
        }
    }
    Ok(data)
}

/// Build a `MODE_CACHE_NUM` data segment.
pub(crate) fn encode_cache_num(num: u8) -> [u8; 1] {
    [num]
}

/// Convert raw motor-state records into a status payload for the given joints.
pub(crate) fn status_from_records(
    comm_version: [u8; 3],
    can_fault: i8,
    tx_err: u8,
    rx_err: u8,
    query: u8,
    records: &[(u8, u8, u8, i16, i16, i16)],
    joint_ids: &[u8; crate::params::JOINTS],
) -> CuResult<CuPantheraStatus> {
    if query != QUERY_MODE_FAULT_POS_VEL_TQE {
        return Err(CuError::from("Panthera unexpected motor state query"));
    }
    let mut motors = core::array::from_fn(|_| CuPantheraMotor::default());
    for (slot, &id) in joint_ids.iter().enumerate() {
        let Some(&(rid, fault, mode, pos, vel, tqe)) = records.iter().find(|r| r.0 == id) else {
            return Err(CuError::from("Panthera missing motor in state frame"));
        };
        if rid != id {
            return Err(CuError::from("Panthera motor id mismatch"));
        }
        let position = pos_from_i16(pos);
        let velocity = vel_from_i16(vel);
        if !position.is_finite() || !velocity.is_finite() {
            return Err(CuError::from("Panthera nonfinite telemetry"));
        }
        motors[slot] = CuPantheraMotor {
            id,
            mode,
            fault,
            position: Angle::new::<radian>(position),
            velocity: AngularVelocity::new::<radian_per_second>(velocity),
            torque: Torque::new::<newton_meter>(tqe_from_i16(tqe)),
        };
    }
    Ok(CuPantheraStatus {
        comm_version,
        can_fault,
        tx_err,
        rx_err,
        motors,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc8_matches_sdk_vectors() {
        // crc8_ccitt of empty input is the init value.
        assert_eq!(crc8(&[]), 0xFF);
        // Single byte: crc = table[init ^ byte].
        assert_eq!(crc8(b"1"), CRC8_TABLE[(0xFF ^ b'1') as usize]);
    }

    #[test]
    fn frame_roundtrip_and_crc_rejection() {
        let data = encode_state_request();
        let mut frame = [0u8; 8];
        encode_frame(&mut frame, MODE_MOTOR_STATE, &data).unwrap();
        assert_eq!(frame[0], PROT_HEAD);
        assert_eq!(frame[1], MODE_MOTOR_STATE);
        assert_eq!(u16::from_le_bytes([frame[2], frame[3]]) as usize, 2);
        let (cmd, parsed) = decode_frame(&frame).unwrap();
        assert_eq!(cmd, MODE_MOTOR_STATE);
        assert_eq!(parsed, Frame::Other); // too short for the fdcan prefix
        frame[5] ^= 0xFF;
        assert!(decode_frame(&frame).is_err());
        assert!(decode_frame(&frame[..5]).is_err());
    }

    #[test]
    fn motor_state_frame_decodes() {
        // data = fdcan_state(3) + query(1) + one 9-byte record for motor 2.
        let mut data = vec![0u8; 3 + 1 + 9];
        data[0] = 0; // can fault OK
        data[3] = QUERY_MODE_FAULT_POS_VEL_TQE;
        data[4] = 2; // id
        data[5] = 0; // fault
        data[6] = 3; // mode
        data[7..9].copy_from_slice(&100i16.to_le_bytes()); // pos
        data[9..11].copy_from_slice(&(-50i16).to_le_bytes()); // vel
        data[11..13].copy_from_slice(&25i16.to_le_bytes()); // tqe
        let mut frame = vec![0u8; 6 + data.len()];
        encode_frame(&mut frame, MODE_MOTOR_STATE, &data).unwrap();
        let (cmd, Frame::MotorState(state)) = decode_frame(&frame).unwrap() else {
            panic!("expected motor state");
        };
        let MotorStateFrame {
            can_fault,
            query,
            records,
            count,
            ..
        } = *state;
        assert_eq!(cmd, MODE_MOTOR_STATE);
        assert_eq!(can_fault, 0);
        assert_eq!(query, QUERY_MODE_FAULT_POS_VEL_TQE);
        assert_eq!(count, 1);
        assert_eq!(records[0], (2, 0, 3, 100, -50, 25));
    }

    #[test]
    fn scaling_roundtrip() {
        for rad in [-6.0, -0.5, 0.0, 1.234, std::f64::consts::TAU] {
            let back = pos_from_i16(pos_to_i16(rad));
            assert!((back - rad).abs() < 0.001, "{rad} -> {back}");
        }
        assert_eq!(tqe_from_i16(tqe_to_i16(1.5)), 1.5);
        assert_eq!(pos_to_i16(1e9), INT16_LIMIT as i16);
        assert_eq!(pos_to_i16(-1e9), -(INT16_LIMIT as i16));
    }

    #[test]
    fn command_frame_layout() {
        // id 6 is skipped, so its slot stays 0x8000; id_max = 7 pads the DLC.
        let joint_ids = [1u8, 2, 3, 4, 5, 7];
        let targets = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6];
        let data = encode_position_command(&targets, &joint_ids, 20.0, 2.0).unwrap();
        assert_eq!(
            data.len(),
            command_data_len(MODE_POS_VEL_TQE_KP_KD, 7).unwrap()
        );
        assert_eq!(data.len(), 73);
        assert_eq!(data[0], TINT16_NOHDR);
        assert_eq!(data[1], QUERY_MODE_FAULT_POS_VEL_TQE);
        // Motor id 3 sits at slot index 2: offset 2 + 2*10 = 22.
        let pos = i16::from_le_bytes([data[22], data[23]]);
        assert!((pos_from_i16(pos) - 0.3).abs() < 0.001);
        let kp = i16::from_le_bytes([data[22 + 6], data[22 + 7]]);
        assert_eq!(kp, pid_to_i16(20.0));
        // Unused slot for id 6 (index 5) stays 0x8000.
        assert_eq!(&data[2 + 5 * 10..2 + 5 * 10 + 2], &i16::MIN.to_le_bytes());
    }
}
