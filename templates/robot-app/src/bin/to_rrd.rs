//! Converts the Copper log of a run (`robot.copper`) into a Rerun recording
//! (`robot.rrd`). Offline: the robot program itself never logs to Rerun.
//!
//!     cargo run --features rrd --bin to_rrd                  # robot.copper -> robot.rrd
//!     cargo run --features rrd --bin to_rrd -- in.copper out.rrd
//!     rerun robot.rrd
//!
//! Entities, on the `cu_time` timeline (Copper clock) and the `copperlist`
//! sequence:
//! - `joints/measured/<n>`, `joints/commanded/<n>`: positions, degrees
//! - `joints/velocity/<n>`: measured velocities, degrees per second
//! - `world/arm/measured`, `world/arm/commanded`: the arm as a line through
//!   its joint frames, from the kinematic chain in `copperconfig.ron`
//! - `world/tcp/<k>`: measured tool path, in segments of up to 256 points
//! - `session`: session commands and their results, as text
//!
//! The joint count comes from the message types in `copperconfig.ron`.

use cu_spatial_payloads::Transform3D;
use cu29::bincode::config::standard;
use cu29::bincode::decode_from_std_read;
use cu29::bincode::error::DecodeError;
use cu29::copperlist::CopperList;
use cu29::prelude::*;
use cu29::units::si::angle::degree;
use cu29::units::si::angular_velocity::degree_per_second;
use repose_controller::CuJointCommandPayload;
use repose_kinematics::CuChainSpec;
use repose_kinematics::CuJointPosition;
use repose_kinematics::CuJointStatePayload;
use repose_kinematics::CuJointVelocity;
use repose_kinematics::CuKinematicChain;
use repose_robot_interface::CuSessionState;
use rerun::RecordingStream;
use rerun::blueprint::Blueprint;
use rerun::blueprint::Horizontal;
use rerun::blueprint::Spatial3DView;
use rerun::blueprint::TextLogView;
use rerun::blueprint::TimeSeriesView;
use rerun::blueprint::Vertical;
use serde::Deserialize;
use std::io::Read;
use std::path::Path;
use std::path::PathBuf;

gen_cumsgs!("copperconfig.ron");

const MEASURED_COLOR: [u8; 3] = [80, 170, 255];
const COMMANDED_COLOR: [u8; 3] = [255, 150, 60];
/// Points per logged tool-path segment; bounds the work per cycle.
const TCP_SEGMENT_POINTS: usize = 256;

/// The measured tool path, logged as consecutive bounded segments.
#[derive(Default)]
struct ToolPath {
    segment: usize,
    points: Vec<[f32; 3]>,
}

impl ToolPath {
    fn log(&mut self, rec: &RecordingStream, tool: [f32; 3]) -> CuResult<()> {
        if self.points.len() == TCP_SEGMENT_POINTS {
            // Start the next segment where this one ended, so the path stays joined.
            let last = self.points[TCP_SEGMENT_POINTS - 1];
            self.points.clear();
            self.points.push(last);
            self.segment += 1;
        }
        self.points.push(tool);
        rec.log(
            format!("world/tcp/{}", self.segment),
            &rerun::LineStrips3D::new([self.points.clone()])
                .with_colors([MEASURED_COLOR])
                .with_radii([0.002]),
        )
        .map_err(rerun_error)
    }
}

/// Counts the bytes read, to tell a clean end of log from a truncated record.
struct CountingReader<R> {
    inner: R,
    bytes: u64,
}

impl<R: Read> Read for CountingReader<R> {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        let n = self.inner.read(buf)?;
        self.bytes += n as u64;
        Ok(n)
    }
}

/// Next copper list, `None` at a clean end of the log. A malformed or
/// truncated record, or a read error, is an error.
fn next_copperlist<R: Read>(src: &mut CountingReader<R>) -> CuResult<Option<CopperList<CuMsgs>>> {
    let start = src.bytes;
    match decode_from_std_read::<CopperList<CuMsgs>, _, _>(src, standard()) {
        Ok(list) => Ok(Some(list)),
        Err(e) => {
            let at_end = match &e {
                DecodeError::UnexpectedEnd { .. } => true,
                DecodeError::Io { inner, .. } => inner.kind() == std::io::ErrorKind::UnexpectedEof,
                _ => false,
            };
            if at_end && src.bytes == start {
                Ok(None)
            } else {
                Err(CuError::from(format!(
                    "to_rrd: cannot decode the copper list at byte {start} of the log: {e}"
                )))
            }
        }
    }
}

/// The `program` task config; only the chain is needed here.
#[derive(Deserialize)]
struct ChainConfig<const N: usize> {
    chain: CuChainSpec<N>,
}

fn rerun_error(e: impl std::error::Error + Send + Sync + 'static) -> CuError {
    CuError::new_with_cause("to_rrd: Rerun", e)
}

fn position_degrees(p: CuJointPosition) -> f64 {
    match p {
        CuJointPosition::Revolute(a) => a.get::<degree>(),
        // Only revolute joints are plotted.
        _ => f64::NAN,
    }
}

fn velocity_degrees(v: CuJointVelocity) -> f64 {
    match v {
        CuJointVelocity::Revolute(w) => w.get::<degree_per_second>(),
        _ => f64::NAN,
    }
}

fn translation(t: &Transform3D<f64>) -> [f32; 3] {
    // Row-major homogeneous matrix: the translation is the last column.
    let m = t.to_matrix();
    [m[0][3] as f32, m[1][3] as f32, m[2][3] as f32]
}

/// Kinematic chain of the `program` task, sized by the logged joint state.
fn load_chain<const N: usize>(
    _joint_state: &CuMsg<CuJointStatePayload<N>>,
) -> CuResult<CuKinematicChain<N>> {
    let config_path = Path::new(env!("CARGO_MANIFEST_DIR")).join("copperconfig.ron");
    let config = read_configuration(&config_path.to_string_lossy())?;
    let params: ChainConfig<N> = config
        .find_task_node(None, "program")
        .and_then(|node| node.get_instance_config())
        .ok_or_else(|| CuError::from("to_rrd: no `program` task config in copperconfig.ron"))?
        .deserialize_into()?;
    params.chain.resolve()
}

/// Base origin, every joint frame and the tool, as one line strip.
fn arm_points<const N: usize>(
    chain: &CuKinematicChain<N>,
    q: &[CuJointPosition; N],
) -> CuResult<Vec<[f32; 3]>> {
    let mut frames = [Transform3D::<f64>::identity(); N];
    let mut tool = Transform3D::<f64>::identity();
    chain.forward_frames(q, &mut frames, &mut tool)?;
    let mut points = Vec::with_capacity(N + 2);
    points.push([0.0; 3]);
    points.extend(frames.iter().map(translation));
    points.push(translation(&tool));
    Ok(points)
}

fn log_arm<const N: usize>(
    rec: &RecordingStream,
    entity: &str,
    chain: &CuKinematicChain<N>,
    q: &[CuJointPosition; N],
    color: [u8; 3],
) -> CuResult<Option<[f32; 3]>> {
    // A pose the chain rejects (non-finite) is skipped, not fatal.
    let Ok(points) = arm_points(chain, q) else {
        return Ok(None);
    };
    let tool = points.last().copied();
    rec.log(
        entity,
        &rerun::LineStrips3D::new([points.clone()])
            .with_colors([color])
            .with_radii([0.006]),
    )
    .map_err(rerun_error)?;
    rec.log(
        format!("{entity}/joints"),
        &rerun::Points3D::new(points)
            .with_colors([color])
            .with_radii([0.012]),
    )
    .map_err(rerun_error)?;
    Ok(tool)
}

fn log_cycle<const N: usize>(
    rec: &RecordingStream,
    chain: &CuKinematicChain<N>,
    state: &CuMsg<CuJointStatePayload<N>>,
    command: &CuMsg<CuJointCommandPayload<N>>,
    tcp_path: &mut ToolPath,
) -> CuResult<()> {
    if let Some(state) = state.payload() {
        for i in 0..N {
            let n = i + 1;
            rec.log(
                format!("joints/measured/{n}"),
                &rerun::Scalars::single(position_degrees(state.positions[i])),
            )
            .map_err(rerun_error)?;
            rec.log(
                format!("joints/velocity/{n}"),
                &rerun::Scalars::single(velocity_degrees(state.velocities[i])),
            )
            .map_err(rerun_error)?;
        }
        if let Some(tool) = log_arm(
            rec,
            "world/arm/measured",
            chain,
            &state.positions,
            MEASURED_COLOR,
        )? {
            tcp_path.log(rec, tool)?;
        }
    }
    if let Some(command) = command.payload() {
        for i in 0..N {
            rec.log(
                format!("joints/commanded/{}", i + 1),
                &rerun::Scalars::single(position_degrees(command.positions[i])),
            )
            .map_err(rerun_error)?;
        }
        log_arm(
            rec,
            "world/arm/commanded",
            chain,
            &command.positions,
            COMMANDED_COLOR,
        )?;
    }
    Ok(())
}

/// Default viewer layout: the arm in 3D beside the joint plots and session log.
fn blueprint() -> Blueprint {
    let plots = Vertical::new([
        TimeSeriesView::new("Joint positions (deg)")
            .with_origin("joints")
            .with_contents(["/joints/measured/**", "/joints/commanded/**"])
            .into(),
        TimeSeriesView::new("Joint velocities (deg/s)")
            .with_origin("joints/velocity")
            .into(),
        TextLogView::new("Session").with_origin("session").into(),
    ])
    .with_row_shares([2.0, 2.0, 1.0]);
    Blueprint::new(
        Horizontal::new([
            Spatial3DView::new("Arm").with_origin("world").into(),
            plots.into(),
        ])
        .with_column_shares([3.0, 2.0]),
    )
}

/// Plot names, so measured and commanded series can be told apart.
fn log_series_names<const N: usize>(
    rec: &RecordingStream,
    _chain: &CuKinematicChain<N>,
) -> CuResult<()> {
    for n in 1..=N {
        for kind in ["measured", "commanded", "velocity"] {
            rec.log_static(
                format!("joints/{kind}/{n}"),
                &rerun::SeriesLines::new().with_names([format!("{kind} {n}")]),
            )
            .map_err(rerun_error)?;
        }
    }
    Ok(())
}

/// Copper time of a cycle: when the joint state was valid, else when it was
/// processed.
fn cycle_time<T: CuMsgPayload>(msg: &CuMsg<T>) -> Option<CuTime> {
    match msg.tov {
        Tov::Time(t) => Some(t),
        Tov::Range(r) => Some(r.end),
        Tov::None => Option::<CuTime>::from(msg.metadata.process_time.end),
    }
}

fn main() -> CuResult<()> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let mut args = std::env::args_os().skip(1).map(PathBuf::from);
    let input = args.next().unwrap_or_else(|| root.join("robot.copper"));
    let output = args.next().unwrap_or_else(|| input.with_extension("rrd"));

    let UnifiedLogger::Read(log) = UnifiedLoggerBuilder::new()
        .file_base_name(&input)
        .build()
        .map_err(|e| CuError::new_with_cause("to_rrd: cannot open the Copper log", e))?
    else {
        return Err(CuError::from("to_rrd: expected a readable Copper log"));
    };
    let mut reader = CountingReader {
        inner: UnifiedLoggerIOReader::new(log, UnifiedLogType::CopperList),
        bytes: 0,
    };

    let rec = rerun::RecordingStreamBuilder::new("{{project-name}}")
        .with_blueprint(blueprint())
        .save(&output)
        .map_err(rerun_error)?;
    rec.log_static("world", &rerun::ViewCoordinates::RIGHT_HAND_Z_UP())
        .map_err(rerun_error)?;

    let mut chain = None;
    let mut tcp_path = ToolPath::default();
    let mut session = CuSessionState::default();
    let mut cycles = 0u64;
    let mut failure = None;
    loop {
        let list = match next_copperlist(&mut reader) {
            Ok(Some(list)) => list,
            Ok(None) => break,
            Err(e) => {
                failure = Some(e);
                break;
            }
        };
        let msgs = &list.msgs;
        let state = msgs.get_robot_rx_joint_state();
        rec.set_time_sequence("copperlist", list.id as i64);
        if let Some(t) = cycle_time(state) {
            rec.set_duration_secs("cu_time", t.as_nanos() as f64 * 1e-9);
        }
        let chain = match &chain {
            Some(chain) => chain,
            None => {
                let loaded = chain.insert(load_chain(state)?);
                log_series_names(&rec, loaded)?;
                &*loaded
            }
        };
        log_cycle(
            &rec,
            chain,
            state,
            msgs.get_program_output_1(),
            &mut tcp_path,
        )?;
        if let Some(s) = msgs.get_robot_rx_session_state().payload()
            && (s.command, s.result) != (session.command, session.result)
        {
            session = s.clone();
            if let Some(command) = s.command {
                rec.log(
                    "session",
                    &rerun::TextLog::new(format!("{command:?}: {:?}", s.result)),
                )
                .map_err(rerun_error)?;
            }
        }
        cycles += 1;
    }
    // Keep what was converted, but do not report a cut-short log as success.
    rec.flush_blocking().map_err(rerun_error)?;
    println!(
        "to_rrd: {cycles} cycles from {} -> {}",
        input.display(),
        output.display()
    );
    match failure {
        Some(e) => {
            eprintln!(
                "to_rrd: the log ended early; {} holds only the cycles before it",
                output.display()
            );
            Err(e)
        }
        None => Ok(()),
    }
}
