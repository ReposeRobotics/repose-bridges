//! UDP I/O lives exclusively on a worker thread.
use crate::{
    params::{CuAbbParams, duration},
    protocol::{self, CAPACITY, CuAbbStatus},
};
use cu29::prelude::*;
use std::{
    net::UdpSocket,
    sync::{
        Mutex,
        atomic::{AtomicBool, Ordering},
    },
    time::{Duration, Instant},
};

/// A streamed joint target: radians and radians per second.
#[cfg(feature = "control")]
#[derive(Debug, Clone, Copy)]
pub(crate) struct Target {
    pub position: [f64; 6],
    pub velocity: [f64; 6],
    pub at: Instant,
}

#[derive(Default)]
pub(crate) struct Shared {
    pub latest: Option<(Instant, CuAbbStatus)>,
    pub generation: u64,
    pub failed: bool,
    pub session: repose_robot_interface::CuSessionTracker,
    #[cfg(feature = "control")]
    pub target: Option<Target>,
}
pub(crate) fn lock(shared: &Mutex<Shared>) -> CuResult<std::sync::MutexGuard<'_, Shared>> {
    shared
        .lock()
        .map_err(|_| CuError::from("ABB worker state poisoned"))
}
fn io(error: std::io::Error) -> CuError {
    CuError::new_with_cause("ABB UDP I/O failed", error)
}

pub(crate) fn bind(params: &CuAbbParams) -> CuResult<UdpSocket> {
    let socket = UdpSocket::bind(params.bind_address()?).map_err(io)?;
    socket
        .set_read_timeout(Some(Duration::from_millis(10)))
        .map_err(io)?;
    Ok(socket)
}

pub(crate) fn run(
    socket: UdpSocket,
    params: &CuAbbParams,
    shared: &Mutex<Shared>,
    shutdown: &AtomicBool,
) -> CuResult<()> {
    let robot = params.robot_address()?;
    let stale_after = duration(params.stale_after)?;
    #[cfg(feature = "control")]
    let mut control = crate::control::Control::new();
    let mut buffer = [0; CAPACITY + 1];
    let mut previous: Option<CuAbbStatus> = None;
    let mut last_sample: Option<Instant> = None;
    while !shutdown.load(Ordering::Acquire) {
        let fresh = last_sample.is_some_and(|at| at.elapsed() <= stale_after);
        #[cfg(feature = "control")]
        {
            control.session(params, shared, fresh)?;
            if control.streaming() && !fresh {
                return Err(CuError::from("ABB EGM feedback timed out while streaming"));
            }
        }
        #[cfg(not(feature = "control"))]
        let _ = fresh;
        let (n, from) = match socket.recv_from(&mut buffer) {
            Ok(received) => received,
            Err(e)
                if matches!(
                    e.kind(),
                    std::io::ErrorKind::WouldBlock
                        | std::io::ErrorKind::TimedOut
                        | std::io::ErrorKind::Interrupted
                ) =>
            {
                continue;
            }
            Err(e) => return Err(io(e)),
        };
        if from.ip() != robot {
            continue;
        }
        if n > CAPACITY {
            return Err(CuError::from("ABB EGM datagram exceeds capacity"));
        }
        let mut status = protocol::decode(&buffer[..n])?;
        status.set_velocities(previous.as_ref());
        previous = Some(status.clone());
        let now = Instant::now();
        last_sample = Some(now);
        {
            let mut state = lock(shared)?;
            state.latest = Some((now, status.clone()));
            state.generation = state.generation.wrapping_add(1);
        }
        #[cfg(feature = "control")]
        if let Some(reply) = control.reply(&status, params, shared)? {
            socket.send_to(reply, from).map_err(io)?;
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::tests::Robot;
    #[test]
    fn ignores_foreign_sources_and_publishes_robot_feedback() {
        let params = CuAbbParams {
            bind: "127.0.0.1:0".into(),
            robot_ip: "127.0.0.1".into(),
            ..Default::default()
        };
        let socket = bind(&params).unwrap();
        let address = socket.local_addr().unwrap();
        let shared = Mutex::new(Shared::default());
        let shutdown = AtomicBool::new(false);
        std::thread::scope(|scope| {
            let worker = scope.spawn(|| run(socket, &params, &shared, &shutdown));
            let robot = UdpSocket::bind("127.0.0.1:0").unwrap();
            for (seq, tm, deg) in [(1, 1000, 0.0), (2, 1004, 0.04)] {
                let message = Robot {
                    seq,
                    tm,
                    joints_deg: vec![deg; 6],
                    ..Default::default()
                };
                robot.send_to(&message.encode(), address).unwrap();
            }
            let deadline = Instant::now() + Duration::from_secs(2);
            while shared.lock().unwrap().generation < 2 {
                assert!(Instant::now() < deadline);
                std::thread::sleep(Duration::from_millis(1));
            }
            let (_, status) = shared.lock().unwrap().latest.clone().unwrap();
            assert_eq!(status.sequence, 2);
            // 0.04 deg in 4 ms = 10 deg/s.
            let v =
                status.velocities[0].get::<cu29::units::si::angular_velocity::degree_per_second>();
            assert!((v - 10.0).abs() < 1e-6, "{v}");
            shutdown.store(true, Ordering::Release);
            worker.join().unwrap().unwrap();
        });
    }

    #[test]
    fn malformed_datagram_faults() {
        let params = CuAbbParams {
            bind: "127.0.0.1:0".into(),
            robot_ip: "127.0.0.1".into(),
            ..Default::default()
        };
        let socket = bind(&params).unwrap();
        let address = socket.local_addr().unwrap();
        let shared = Mutex::new(Shared::default());
        let shutdown = AtomicBool::new(false);
        let robot = UdpSocket::bind("127.0.0.1:0").unwrap();
        robot.send_to(&[0xff, 0xff], address).unwrap();
        assert!(run(socket, &params, &shared, &shutdown).is_err());
    }
    #[cfg(feature = "control")]
    #[test]
    fn replies_to_robot_only_while_streaming_and_faults_on_silence() {
        use repose_robot_interface::{CuSessionCommand, CuSessionResult};
        let params = CuAbbParams {
            bind: "127.0.0.1:0".into(),
            robot_ip: "127.0.0.1".into(),
            allow_control: true,
            allow_motion: true,
            ..Default::default()
        };
        let socket = bind(&params).unwrap();
        let address = socket.local_addr().unwrap();
        let shared = Mutex::new(Shared::default());
        let shutdown = AtomicBool::new(false);
        let robot = UdpSocket::bind("127.0.0.1:0").unwrap();
        robot
            .set_read_timeout(Some(Duration::from_millis(50)))
            .unwrap();
        let mut reply = [0; 256];
        std::thread::scope(|scope| {
            let worker = scope.spawn(|| run(socket, &params, &shared, &shutdown));
            let send = |mci| {
                let message = Robot {
                    mci,
                    ..Default::default()
                };
                robot.send_to(&message.encode(), address).unwrap();
            };
            send(2);
            assert!(robot.recv_from(&mut reply).is_err()); // monitoring: silent
            shared
                .lock()
                .unwrap()
                .session
                .request(CuSessionCommand::AcquireControl);
            let deadline = Instant::now() + Duration::from_secs(2);
            while shared.lock().unwrap().session.state().result != CuSessionResult::Acknowledged {
                assert!(Instant::now() < deadline);
                std::thread::sleep(Duration::from_millis(1));
            }
            shared
                .lock()
                .unwrap()
                .session
                .request(CuSessionCommand::EnableDrives);
            let mut answered = false;
            while !answered {
                assert!(Instant::now() < deadline);
                send(3);
                answered = robot.recv_from(&mut reply).is_ok();
            }
            // Stop sending feedback: streaming must fault, not hold forever.
            assert!(worker.join().unwrap().is_err());
        });
    }
}
