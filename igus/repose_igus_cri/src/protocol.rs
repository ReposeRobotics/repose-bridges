//! Pure CRI text protocol: frame decoding and command encoding. No I/O.

use cu29::prelude::*;
use cu29::units::si::angle::degree;
use cu29::units::si::f64::Angle;
use cu29::units::si::f64::Length;
use cu29::units::si::length::millimeter;
use repose_robot_interface::CuSessionCommand;
use serde::Deserialize;
use serde::Serialize;

/// Per-joint `ERROR` code: drive not enabled.
pub(crate) const JOINT_NOT_ENABLED: i32 = 4;
/// Per-joint `ERROR` code: drive enabling.
pub(crate) const JOINT_ENABLING: i32 = 4096;
/// Largest client counter; counters run 1..=9999 and wrap to 1.
pub(crate) const MAX_COUNTER: u16 = 9999;

/// Fixed-capacity ASCII token from a CRI frame, truncated to 32 bytes.
#[derive(
    Default,
    Debug,
    Clone,
    PartialEq,
    Eq,
    bincode::Encode,
    bincode::Decode,
    Serialize,
    Deserialize,
    Reflect,
)]
pub struct CuCriText {
    bytes: [u8; 32],
    len: u8,
}

impl CuCriText {
    pub(crate) fn new(text: &str) -> Self {
        let mut take = text.len().min(32);
        while !text.is_char_boundary(take) {
            take -= 1;
        }
        let mut bytes = [0; 32];
        bytes[..take].copy_from_slice(&text.as_bytes()[..take]);
        Self {
            bytes,
            len: take as u8,
        }
    }

    /// The stored text; empty when the field was absent.
    pub fn as_str(&self) -> &str {
        core::str::from_utf8(&self.bytes[..usize::from(self.len.min(32))]).unwrap_or("")
    }
}

/// Cartesian TCP coordinates reported by `POSCARTROBOT`.
///
/// ABC are retained as controller angles; their rotation convention is not
/// inferred. Tool and base frame settings belong to the controller.
#[derive(
    Default, Debug, Clone, bincode::Encode, bincode::Decode, Serialize, Deserialize, Reflect,
)]
pub struct CuCriCartesianPose {
    /// Cartesian XYZ position (wire millimetres converted to Copper units).
    pub translation: [Length; 3],
    /// Controller A, B, C orientation angles (wire degrees).
    pub abc: [Angle; 3],
}

/// Everything this bridge reads from one CRI `STATUS` frame, as the
/// controller sent it. Payload of the vendor `cri_status` channel.
///
/// Fields absent from the frame stay at their defaults.
#[derive(
    Default, Debug, Clone, bincode::Encode, bincode::Decode, Serialize, Deserialize, Reflect,
)]
pub struct CuCriStatus {
    /// Measured joints A1–A6 from `POSJOINTCURRENT`.
    pub joints: [Angle; 6],
    /// Controller TCP pose from `POSCARTROBOT`.
    pub cartesian: Option<CuCriCartesianPose>,
    /// Motion mode token, for example `joint` or `cartbase`.
    pub mode: CuCriText,
    /// Error summary token from `ERROR`, for example `NoError` or `_MNE`.
    pub error_summary: CuCriText,
    /// Per-joint error codes A1–A6 from `ERROR`: 0 ready, 4 not enabled,
    /// 4096 enabling.
    pub joint_errors: [i32; 6],
    /// `KINSTATE` code; 0 no kinematic error, 99 motion not allowed.
    pub kinstate: Option<i32>,
    /// Raw `ESTOP` value.
    pub estop: Option<i32>,
    /// Velocity override in percent.
    pub override_percent: Option<f64>,
}

impl CuCriStatus {
    /// Every joint reports error code 0. After `Enable` the controller brings
    /// the drives up one axis at a time (about 0.4–0.7 s each on a ReBeL).
    pub fn drives_ready(&self) -> bool {
        self.joint_errors.iter().all(|&code| code == 0)
    }

    /// Drives are ready and `KINSTATE` reports no kinematic error.
    pub fn motion_allowed(&self) -> bool {
        self.drives_ready() && self.kinstate == Some(0)
    }

    /// First joint code that is neither ready, not-enabled, nor enabling.
    pub(crate) fn joint_fault(&self) -> Option<i32> {
        self.joint_errors
            .iter()
            .copied()
            .find(|&code| !matches!(code, 0 | JOINT_NOT_ENABLED | JOINT_ENABLING))
    }
}

/// Status of a Move Joint command.
#[derive(
    Default,
    Debug,
    Clone,
    Copy,
    PartialEq,
    Eq,
    bincode::Encode,
    bincode::Decode,
    Serialize,
    Deserialize,
    Reflect,
)]
#[serde(rename_all = "snake_case")]
pub enum CuCriMotionStatus {
    /// No motion commanded.
    #[default]
    Idle,
    /// Command queued or sent to the controller; waiting for acknowledgement.
    Pending,
    /// Controller acknowledged move; robot moving toward target.
    Moving,
    /// Target reached within configured goal tolerance.
    Reached,
    /// Command rejected locally or by the controller.
    Rejected,
    /// Motion aborted mid-flight (control lost, drives not ready, fault, or disconnect).
    Aborted,
}

/// Why a motion request was rejected or aborted.
#[derive(
    Default,
    Debug,
    Clone,
    Copy,
    PartialEq,
    Eq,
    bincode::Encode,
    bincode::Decode,
    Serialize,
    Deserialize,
    Reflect,
)]
#[serde(rename_all = "snake_case")]
pub enum CuCriMotionRejection {
    #[default]
    None,
    /// Refused locally: session does not hold control.
    NotInControl,
    /// Refused locally: drives are not ready or motion is not allowed.
    MotionNotAllowed,
    /// Refused locally: target exceeds configured joint limits.
    JointLimits,
    /// Refused locally: step from current position exceeds `max_step`.
    MaxStepExceeded,
    /// Refused locally: another move is currently in flight.
    MoveInFlight,
    /// Rejected by controller via CMDERROR; `detail` holds the controller message.
    Controller,
    /// Motion aborted mid-flight.
    Aborted,
}

/// Move tracking state published on the `cri_motion` Rx channel.
#[derive(
    Default,
    Debug,
    Clone,
    PartialEq,
    bincode::Encode,
    bincode::Decode,
    Serialize,
    Deserialize,
    Reflect,
)]
pub struct CuCriMotionState {
    /// Client sequence counter the move was sent with.
    pub sequence: u32,
    /// Commanded target joint angles.
    pub target: [Angle; 6],
    /// Current motion execution status.
    pub status: CuCriMotionStatus,
    /// Rejection or abort reason, when `status` is `Rejected` or `Aborted`.
    pub rejection: CuCriMotionRejection,
    /// Controller text accompanying a rejection or abort detail.
    pub detail: CuCriText,
    /// Different targets refused because a move was in flight.
    pub refused: u32,
    /// Last `EXECACK`/`EXECEND`/`EXECERROR` frame, verbatim (informational:
    /// their format is unverified on this firmware).
    pub last_exec: CuCriText,
}

/// One decoded server frame.
#[derive(Debug)]
pub(crate) enum CriFrame<'a> {
    /// Written into the caller's `CuCriStatus`.
    Status,
    Acknowledged(u32),
    Rejected(u32, &'a str),
    /// `CMD Active true|false`: whether this connection is in control.
    Active(bool),
    /// Informational program execution frames (unverified on this firmware).
    ExecAck(&'a str),
    ExecEnd(&'a str),
    ExecError(&'a str),
    Other,
}

/// CRI keyword for a session command.
pub(crate) fn command_keyword(command: CuSessionCommand) -> &'static str {
    match command {
        CuSessionCommand::AcquireControl => "SetActive true",
        CuSessionCommand::ReleaseControl => "SetActive false",
        CuSessionCommand::ClearFaults => "Reset",
        CuSessionCommand::EnableDrives => "Enable",
        CuSessionCommand::DisableDrives => "Disable",
    }
}

/// Format a `CMD Move Joint` CRI command line.
pub(crate) fn format_move_joint(
    sequence: u16,
    target: &[Angle; 6],
    velocity_percent: f64,
) -> String {
    let [a1, a2, a3, a4, a5, a6] = target;
    format!(
        "CRISTART {sequence} CMD Move Joint {:.1} {:.1} {:.1} {:.1} {:.1} {:.1} 0 0 0 {velocity_percent} CRIEND",
        a1.get::<degree>(),
        a2.get::<degree>(),
        a3.get::<degree>(),
        a4.get::<degree>(),
        a5.get::<degree>(),
        a6.get::<degree>(),
    )
}

/// Decode one frame that ends in `CRIEND`. A `STATUS` frame is written into
/// `status` (fully replaced). Malformed `STATUS` is an error; unknown frame
/// types are `Other`.
pub(crate) fn decode<'a>(frame: &'a str, status: &mut CuCriStatus) -> CuResult<CriFrame<'a>> {
    let mut tokens = frame.split_ascii_whitespace();
    if tokens.next() != Some("CRISTART") {
        return Err(CuError::from("Invalid CRI frame start"));
    }
    tokens
        .next()
        .and_then(|v| v.parse::<u16>().ok())
        .ok_or_else(|| CuError::from("Invalid CRI counter"))?;
    let body = |keyword: &str| {
        frame
            .find(keyword)
            .map(|i| {
                frame[i + keyword.len()..]
                    .trim()
                    .trim_end_matches("CRIEND")
                    .trim()
            })
            .unwrap_or("")
    };
    Ok(match tokens.next() {
        Some("STATUS") => {
            *status = CuCriStatus::default();
            decode_status(tokens.peekable(), status)?;
            CriFrame::Status
        }
        Some(kind @ ("CMDACK" | "CMDERROR")) => {
            let Some(counter) = tokens.next().and_then(|t| t.parse::<u32>().ok()) else {
                return Ok(CriFrame::Other);
            };
            if kind == "CMDACK" {
                CriFrame::Acknowledged(counter)
            } else {
                let detail = body("CMDERROR");
                let detail = detail.split_once(' ').map(|(_, rest)| rest).unwrap_or("");
                CriFrame::Rejected(counter, detail)
            }
        }
        Some("CMD") => match (tokens.next(), tokens.next()) {
            (Some("Active"), Some("true")) => CriFrame::Active(true),
            (Some("Active"), Some("false")) => CriFrame::Active(false),
            _ => CriFrame::Other,
        },
        Some("EXECACK") => CriFrame::ExecAck(body("EXECACK")),
        Some("EXECEND") => CriFrame::ExecEnd(body("EXECEND")),
        Some("EXECERROR") => CriFrame::ExecError(body("EXECERROR")),
        _ => CriFrame::Other,
    })
}

fn decode_status<'a>(
    mut tokens: core::iter::Peekable<impl Iterator<Item = &'a str>>,
    status: &mut CuCriStatus,
) -> CuResult<()> {
    let mut found_joints = false;
    while let Some(token) = tokens.next() {
        match token {
            "POSJOINTCURRENT" => {
                if found_joints {
                    return Err(CuError::from("Duplicate CRI joint positions"));
                }
                for position in &mut status.joints {
                    *position = Angle::new::<degree>(read_number(&mut tokens)?);
                }
                found_joints = true;
            }
            "POSCARTROBOT" => {
                if status.cartesian.is_some() {
                    return Err(CuError::from("Duplicate CRI Cartesian pose"));
                }
                let mut pose = CuCriCartesianPose::default();
                for value in &mut pose.translation {
                    *value = Length::new::<millimeter>(read_number(&mut tokens)?);
                }
                for value in &mut pose.abc {
                    *value = Angle::new::<degree>(read_number(&mut tokens)?);
                }
                status.cartesian = Some(pose);
            }
            "MODE" => {
                if let Some(mode) = tokens.next() {
                    status.mode = CuCriText::new(mode);
                }
            }
            "ERROR" => {
                if let Some(summary) = tokens.next() {
                    status.error_summary = CuCriText::new(summary);
                }
                let mut index = 0;
                while let Some(code) = read_int(&mut tokens) {
                    if let Some(slot) = status.joint_errors.get_mut(index) {
                        *slot = code;
                    }
                    index += 1;
                }
            }
            "KINSTATE" => status.kinstate = read_int(&mut tokens),
            "ESTOP" => status.estop = read_int(&mut tokens),
            "OVERRIDE" => {
                status.override_percent = tokens
                    .peek()
                    .and_then(|v| v.parse::<f64>().ok())
                    .filter(|v| v.is_finite());
                if status.override_percent.is_some() {
                    tokens.next();
                }
            }
            _ => {}
        }
    }
    if !found_joints {
        return Err(CuError::from("CRI STATUS missing POSJOINTCURRENT"));
    }
    Ok(())
}

fn read_int<'a>(tokens: &mut core::iter::Peekable<impl Iterator<Item = &'a str>>) -> Option<i32> {
    let value = tokens.peek()?.parse::<i32>().ok()?;
    tokens.next();
    Some(value)
}

fn read_number<'a>(tokens: &mut impl Iterator<Item = &'a str>) -> CuResult<f64> {
    tokens
        .next()
        .and_then(|v| v.parse::<f64>().ok())
        .filter(|v| v.is_finite())
        .ok_or_else(|| CuError::from("Invalid CRI coordinate"))
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    pub(crate) const STATUS: &str = "CRISTART 42 STATUS MODE joint POSJOINTCURRENT 0 90 -90 180 45 -45 0 0 0 0 0 0 0 0 0 0 POSCARTROBOT 0 0 0 0 0 0 CRIEND";
    pub(crate) const FULL_STATUS: &str = "CRISTART 7 STATUS MODE joint POSJOINTSETPOINT 1 2 3 4 5 6 0 0 0 0 0 0 0 0 0 0 POSJOINTCURRENT 1 2 3 4 5 6 0 0 0 0 0 0 0 0 0 0 POSCARTROBOT 1 2 3 4 5 6 POSCARTPLATTFORM 0 0 0 OVERRIDE 80.0 DIN 0 DOUT 0 ESTOP 3 SUPPLY 24000 CURRENTALL 500 CURRENTJOINTS 1 2 3 4 5 6 0 0 0 0 0 0 0 0 0 0 ERROR NoError 0 0 4 0 0 0 0 0 0 0 0 0 0 0 0 0 KINSTATE 13 CRIEND";

    pub(crate) fn status(frame: &str) -> CuResult<Option<CuCriStatus>> {
        let mut out = CuCriStatus::default();
        Ok(match decode(frame, &mut out)? {
            CriFrame::Status => Some(out),
            _ => None,
        })
    }

    #[test]
    fn test_status_units_and_validation() {
        let output = status(STATUS).unwrap().unwrap();
        assert!((output.joints[1].get::<degree>() - 90.0).abs() < 1e-10);
        assert!(status("CRISTART 1 STATUS POSJOINTCURRENT 0 NaN 0 0 0 0 CRIEND").is_err());
        assert!(status("CRISTART 1 STATUS POSJOINTCURRENT 0 1 CRIEND").is_err());
        assert!(status("CRISTART 1 STATUS MODE joint CRIEND").is_err());
        assert!(
            status("CRISTART 1 RUNSTATE ignored CRIEND")
                .unwrap()
                .is_none()
        );
        assert!(status("CRISTOP 1 STATUS CRIEND").is_err());
    }

    #[test]
    fn test_cartesian_units_missing_fields_and_malformed_pose() {
        let frame = "CRISTART 1 STATUS POSCARTROBOT 123 -456 789 10 -20 30 POSJOINTCURRENT 1 2 3 4 5 6 CRIEND";
        let decoded = status(frame).unwrap().unwrap();
        let pose = decoded.cartesian.as_ref().unwrap();
        assert!((pose.translation[0].get::<millimeter>() - 123.0).abs() < 1e-10);
        assert!(
            (pose.translation[1].get::<cu29::units::si::length::meter>() + 0.456).abs() < 1e-10
        );
        assert!((pose.abc[1].get::<degree>() + 20.0).abs() < 1e-10);
        assert!((decoded.joints[5].get::<degree>() - 6.0).abs() < 1e-10);
        let without = status("CRISTART 2 STATUS POSJOINTCURRENT 1 2 3 4 5 6 CRIEND")
            .unwrap()
            .unwrap();
        assert!(without.cartesian.is_none());
        for tail in ["1 2", "1 2 3 4 NaN 6", "1 2 3 4 5 inf"] {
            assert!(
                status(&format!(
                    "CRISTART 3 STATUS POSJOINTCURRENT 1 2 3 4 5 6 POSCARTROBOT {tail} CRIEND"
                ))
                .is_err()
            );
        }
    }

    #[test]
    fn test_health_fields() {
        let health = status(FULL_STATUS).unwrap().unwrap();
        assert_eq!(health.mode.as_str(), "joint");
        assert_eq!(health.error_summary.as_str(), "NoError");
        assert_eq!(health.joint_errors, [0, 0, 4, 0, 0, 0]);
        assert_eq!(health.kinstate, Some(13));
        assert_eq!(health.estop, Some(3));
        assert_eq!(health.override_percent, Some(80.0));
        assert!(!health.drives_ready() && !health.motion_allowed());
        assert_eq!(health.joint_fault(), None);
        assert!((health.joints[5].get::<degree>() - 6.0).abs() < 1e-10);
    }

    fn decode_any(frame: &str) -> CuResult<CriFrame<'_>> {
        decode(frame, &mut CuCriStatus::default())
    }

    #[test]
    fn test_move_joint_format_and_exec_frames() {
        let target = [
            Angle::new::<degree>(0.0),
            Angle::new::<degree>(10.5),
            Angle::new::<degree>(-20.0),
            Angle::new::<degree>(30.12),
            Angle::new::<degree>(0.0),
            Angle::new::<degree>(-5.0),
        ];
        let line = format_move_joint(42, &target, 5.0);
        assert_eq!(
            line,
            "CRISTART 42 CMD Move Joint 0.0 10.5 -20.0 30.1 0.0 -5.0 0 0 0 5 CRIEND"
        );

        match decode_any("CRISTART 10 EXECACK 1 0 CRIEND").unwrap() {
            CriFrame::ExecAck(detail) => assert_eq!(detail, "1 0"),
            other => panic!("{other:?}"),
        }
        match decode_any("CRISTART 11 EXECEND 1 0 PLAN CRIEND").unwrap() {
            CriFrame::ExecEnd(detail) => assert_eq!(detail, "1 0 PLAN"),
            other => panic!("{other:?}"),
        }
        match decode_any("CRISTART 12 EXECERROR 1 0 collision CRIEND").unwrap() {
            CriFrame::ExecError(detail) => assert_eq!(detail, "1 0 collision"),
            other => panic!("{other:?}"),
        }
    }
    #[test]
    fn test_replies() {
        assert!(matches!(
            decode_any("CRISTART 9 CMDACK 5 CRIEND").unwrap(),
            CriFrame::Acknowledged(5)
        ));
        // Format observed on RobotControl V980-14.
        match decode_any("CRISTART 114 CMDERROR 12 not available CRIEND").unwrap() {
            CriFrame::Rejected(12, detail) => assert_eq!(detail, "not available"),
            other => panic!("{other:?}"),
        }
        assert!(matches!(
            decode_any("CRISTART 5 CMD Active true CRIEND").unwrap(),
            CriFrame::Active(true)
        ));
        assert!(matches!(
            decode_any("CRISTART 4 CMD Active false CRIEND").unwrap(),
            CriFrame::Active(false)
        ));
        assert!(matches!(
            decode_any("CRISTART 1 MESSAGE RobotControl Version CRIEND").unwrap(),
            CriFrame::Other
        ));
    }
}
