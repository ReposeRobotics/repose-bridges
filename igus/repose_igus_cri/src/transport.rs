//! TCP worker thread: owns the socket, decodes frames, and shares the latest
//! observation and session with the Copper side.

use crate::motion::MotionTracker;
use crate::protocol::CriFrame;
use crate::protocol::CuCriStatus;
use crate::protocol::MAX_COUNTER;
use crate::protocol::command_keyword;
use crate::protocol::decode;
use crate::protocol::format_move_joint;
use cu29::prelude::*;
use cu29::units::si::f64::Angle;
use repose_robot_interface::CuSessionTracker;
use std::io::Read;
use std::io::Write;
use std::net::TcpStream;
use std::sync::Mutex;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;
use std::time::Duration;
use std::time::Instant;

pub(crate) const FRAME_CAPACITY: usize = 8192;
/// ALIVEJOG period; also the jog streaming rate (the vendor driver uses 50 ms).
pub(crate) const HEARTBEAT: Duration = Duration::from_millis(50);
pub(crate) const POLL: Duration = Duration::from_millis(20);

/// State shared between the worker and the bridge.
#[derive(Default)]
pub(crate) struct Shared {
    /// Latest `STATUS` and its host receipt time.
    pub latest: Option<(Instant, CuCriStatus)>,
    /// Incremented per `STATUS`; channels consume each generation once.
    pub generation: u64,
    pub session: CuSessionTracker,
    pub motion: MotionTracker,
    /// Move Joint velocity in percent of the controller maximum.
    pub velocity_percent: f64,
    /// Per-joint tolerance for a move to count as reached.
    pub goal_tolerance: Angle,
    /// Jog percent per joint from the latest velocity command.
    pub jog: [f64; 6],
    /// The jog stays valid until this instant (command timeout).
    pub jog_until: Option<Instant>,
    pub failed: bool,
}

impl Shared {
    /// Jog to send now: the latest velocity command while it is fresh and the
    /// session allows motion and no `Move Joint` is in flight, else zero.
    pub(crate) fn active_jog(&mut self) -> [f64; 6] {
        let state = self.session.state();
        let fresh = self.jog_until.is_some_and(|until| Instant::now() < until);
        if fresh && state.control_acquired && state.motion_allowed && !self.motion.in_flight() {
            self.jog
        } else {
            self.jog_until = None;
            [0.0; 6]
        }
    }

    pub(crate) fn fail(&mut self) {
        self.latest = None;
        self.session.on_disconnected();
        self.motion.on_disconnected();
        self.jog_until = None;
        self.failed = true;
    }
}

/// Client counter running 1..=9999, wrapping to 1, as CRI requires.
struct Counter(u16);

impl Counter {
    fn next(&mut self) -> u16 {
        let current = self.0;
        self.0 = self.0 % MAX_COUNTER + 1;
        current
    }
}

fn lock(shared: &Mutex<Shared>) -> CuResult<std::sync::MutexGuard<'_, Shared>> {
    shared
        .lock()
        .map_err(|_| CuError::from("CRI worker state poisoned"))
}

/// Run until `shutdown` or a transport/protocol error.
pub(crate) fn run(
    mut stream: TcpStream,
    shared: &Mutex<Shared>,
    shutdown: &AtomicBool,
) -> CuResult<()> {
    let mut frame = [0; FRAME_CAPACITY];
    let mut used = 0;
    let mut heartbeat = None;
    let mut counter = Counter(1);
    let mut bytes = [0; 1024];
    let mut status = CuCriStatus::default();
    while !shutdown.load(Ordering::Acquire) {
        if heartbeat.is_none_or(|last: Instant| last.elapsed() >= HEARTBEAT) {
            let jog = lock(shared)?.active_jog();
            let sequence = counter.next();
            if jog.iter().all(|v| *v == 0.0) {
                writeln!(
                    stream,
                    "CRISTART {sequence} ALIVEJOG 0 0 0 0 0 0 0 0 0 CRIEND"
                )
            } else {
                writeln!(
                    stream,
                    "CRISTART {sequence} ALIVEJOG {:.3} {:.3} {:.3} {:.3} {:.3} {:.3} 0 0 0 CRIEND",
                    jog[0], jog[1], jog[2], jog[3], jog[4], jog[5]
                )
            }
            .map_err(|e| CuError::new_with_cause("CRI heartbeat failed", e))?;
            heartbeat = Some(Instant::now());
        }
        let outgoing = lock(shared)?.session.take_outgoing();
        if let Some(command) = outgoing {
            let sequence = counter.next();
            writeln!(
                stream,
                "CRISTART {sequence} CMD {} CRIEND",
                command_keyword(command)
            )
            .map_err(|e| CuError::new_with_cause("CRI command failed", e))?;
            lock(shared)?
                .session
                .mark_sent(command, u32::from(sequence));
        }
        let (target, velocity) = {
            let mut shared = lock(shared)?;
            (shared.motion.take_outgoing(), shared.velocity_percent)
        };
        if let Some(target) = target {
            let sequence = counter.next();
            writeln!(stream, "{}", format_move_joint(sequence, &target, velocity))
                .map_err(|e| CuError::new_with_cause("CRI move failed", e))?;
            lock(shared)?.motion.mark_sent(u32::from(sequence));
        }
        match stream.read(&mut bytes) {
            Ok(0) => return Err(CuError::from("CRI peer disconnected")),
            Ok(count) => {
                for &byte in &bytes[..count] {
                    if used == frame.len() {
                        return Err(CuError::from("CRI frame exceeds capacity"));
                    }
                    frame[used] = byte;
                    used += 1;
                    if frame[..used].ends_with(b"CRIEND") {
                        let text = core::str::from_utf8(&frame[..used])
                            .map_err(|e| CuError::new_with_cause("Invalid CRI UTF-8", e))?;
                        apply(decode(text, &mut status)?, &status, &mut *lock(shared)?);
                        used = 0;
                    }
                }
            }
            Err(e)
                if matches!(
                    e.kind(),
                    std::io::ErrorKind::WouldBlock
                        | std::io::ErrorKind::TimedOut
                        | std::io::ErrorKind::Interrupted
                ) => {}
            Err(e) => return Err(CuError::new_with_cause("CRI read failed", e)),
        }
    }
    Ok(())
}

fn apply(frame: CriFrame<'_>, status: &CuCriStatus, shared: &mut Shared) {
    match frame {
        CriFrame::Status => {
            shared
                .session
                .on_drive_report(status.drives_ready(), status.motion_allowed());
            let tolerance = shared.goal_tolerance;
            let Shared {
                session, motion, ..
            } = shared;
            motion.on_status(status, session.state(), tolerance);
            shared.latest = Some((Instant::now(), status.clone()));
            shared.generation = shared.generation.wrapping_add(1);
        }
        CriFrame::Acknowledged(sequence) => {
            shared.session.on_acknowledged(sequence);
            shared.motion.on_acknowledged(sequence);
        }
        CriFrame::Rejected(sequence, detail) => {
            shared.session.on_rejected(sequence, detail);
            shared.motion.on_rejected(sequence, detail);
        }
        // Informational; their format is unverified on this firmware.
        CriFrame::ExecAck(body) => shared.motion.on_exec("EXECACK", body),
        CriFrame::ExecEnd(body) => shared.motion.on_exec("EXECEND", body),
        CriFrame::ExecError(body) => shared.motion.on_exec("EXECERROR", body),
        CriFrame::Active(active) => shared.session.on_control_report(active),
        CriFrame::Other => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::tests::STATUS;
    use cu29::units::si::angle::degree;
    use std::net::TcpListener;
    use std::sync::Arc;

    #[test]
    fn test_counter_wraps_to_one() {
        let mut counter = Counter(MAX_COUNTER);
        assert_eq!(counter.next(), MAX_COUNTER);
        assert_eq!(counter.next(), 1);
    }

    #[test]
    fn test_fragmentation_coalescing_and_heartbeat() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let stream = TcpStream::connect(listener.local_addr().unwrap()).unwrap();
        stream.set_read_timeout(Some(POLL)).unwrap();
        stream.set_write_timeout(Some(POLL)).unwrap();
        let (mut peer, _) = listener.accept().unwrap();
        peer.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
        let shared = Arc::new(Mutex::new(Shared::default()));
        let shutdown = Arc::new(AtomicBool::new(false));
        let slot = Arc::clone(&shared);
        let stop = Arc::clone(&shutdown);
        let worker = std::thread::spawn(move || run(stream, &slot, &stop));
        let mut heartbeat = Vec::new();
        loop {
            let mut byte = [0];
            peer.read_exact(&mut byte).unwrap();
            heartbeat.push(byte[0]);
            if byte[0] == b'\n' {
                break;
            }
        }
        assert_eq!(
            core::str::from_utf8(&heartbeat).unwrap(),
            "CRISTART 1 ALIVEJOG 0 0 0 0 0 0 0 0 0 CRIEND\n"
        );
        peer.write_all(b"CRISTART 0 RUNSTATE ignored CRIEND\nCRISTA")
            .unwrap();
        peer.write_all(&STATUS.as_bytes()[6..]).unwrap();
        let deadline = Instant::now() + Duration::from_secs(2);
        while shared.lock().unwrap().latest.is_none() {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(2));
        }
        let guard = shared.lock().unwrap();
        assert_eq!(guard.generation, 1);
        assert!((guard.latest.as_ref().unwrap().1.joints[2].get::<degree>() + 90.0).abs() < 1e-10);
        drop(guard);
        shutdown.store(true, Ordering::Release);
        worker.join().unwrap().unwrap();
    }
}
