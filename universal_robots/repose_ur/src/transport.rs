//! Socket I/O lives exclusively on a worker after startup negotiation.
use crate::{
    params::{CuUrParams, duration},
    protocol::{self, CAPACITY, CuUrStatus},
};
use cu29::prelude::*;
use std::{
    io::{Read, Write},
    net::TcpStream,
    sync::{
        Mutex,
        atomic::{AtomicBool, Ordering},
    },
    time::{Duration, Instant},
};

#[derive(Default)]
pub(crate) struct Shared {
    pub latest: Option<(Instant, CuUrStatus)>,
    pub generation: u64,
    pub failed: bool,
    pub session: repose_robot_interface::CuSessionTracker,
    #[cfg(feature = "control")]
    pub velocity: Option<([f64; 6], Instant)>,
    #[cfg(feature = "control")]
    pub pending: Option<([f64; 6], Instant)>,
    #[cfg(feature = "control")]
    pub active: Option<([f64; 6], Instant)>,
    #[cfg(feature = "control")]
    pub last_target: Option<[f64; 6]>,
}
fn io(error: std::io::Error) -> CuError {
    CuError::new_with_cause("UR socket I/O failed", error)
}
fn lock(shared: &Mutex<Shared>) -> CuResult<std::sync::MutexGuard<'_, Shared>> {
    shared
        .lock()
        .map_err(|_| CuError::from("UR worker state poisoned"))
}
pub(crate) fn write_packet(stream: &mut TcpStream, kind: u8, body: &[u8]) -> CuResult<()> {
    let size = u16::try_from(body.len() + 3).map_err(|_| CuError::from("UR oversized request"))?;
    stream.write_all(&size.to_be_bytes()).map_err(io)?;
    stream.write_all(&[kind]).map_err(io)?;
    stream.write_all(body).map_err(io)
}
fn response(stream: &mut TcpStream, kind: u8) -> CuResult<Vec<u8>> {
    let mut header = [0; 3];
    stream.read_exact(&mut header).map_err(io)?;
    let size = usize::from(u16::from_be_bytes([header[0], header[1]]));
    if !(3..=CAPACITY).contains(&size) {
        return Err(CuError::from("UR invalid negotiation response"));
    }
    let mut body = vec![0; size - 3];
    stream.read_exact(&mut body).map_err(io)?;
    if header[2] == b'M' {
        let length = body.first().copied().unwrap_or(0) as usize;
        let message = body
            .get(1..1 + length)
            .and_then(|s| std::str::from_utf8(s).ok())
            .unwrap_or("malformed controller message");
        return Err(CuError::from(format!("UR controller: {message}")));
    }
    if header[2] != kind {
        return Err(CuError::from("UR unexpected negotiation packet"));
    }
    Ok(body)
}
pub(crate) fn connect(params: &CuUrParams) -> CuResult<(TcpStream, u8, u8)> {
    let timeout = duration(params.connect_timeout)?;
    let mut stream = TcpStream::connect_timeout(&params.socket_address()?, timeout).map_err(io)?;
    stream.set_read_timeout(Some(timeout)).map_err(io)?;
    stream.set_write_timeout(Some(timeout)).map_err(io)?;
    write_packet(&mut stream, b'V', &2u16.to_be_bytes())?;
    if response(&mut stream, b'V')? != [1] {
        return Err(CuError::from("UR requires RTDE protocol v2"));
    }
    let mut recipe = params.frequency.to_be_bytes().to_vec();
    recipe.extend_from_slice(protocol::FIELDS);
    write_packet(&mut stream, b'O', &recipe)?;
    let result = response(&mut stream, b'O')?;
    if result.first().is_none_or(|id| *id == 0) || result[1..] != *protocol::TYPES {
        return Err(CuError::from("UR unsupported RTDE output recipe"));
    }
    let id = result[0];
    #[allow(unused_mut)]
    let mut input = 0;
    #[cfg(feature = "control")]
    if params.allow_control {
        write_packet(&mut stream, b'I', crate::control::INPUT_FIELDS)?;
        let result = response(&mut stream, b'I')?;
        if result.first().is_none_or(|id| *id == 0) || result[1..] != *crate::control::INPUT_TYPES {
            return Err(CuError::from("UR control registers unavailable or in use"));
        }
        input = result[0];
    }
    write_packet(&mut stream, b'S', &[])?;
    if response(&mut stream, b'S')? != [1] {
        return Err(CuError::from("UR refused RTDE start"));
    }
    stream
        .set_read_timeout(Some(Duration::from_millis(20)))
        .map_err(io)?;
    Ok((stream, id, input))
}

#[cfg(feature = "control")]
pub(crate) fn script(params: &CuUrParams, text: &str) -> CuResult<()> {
    let mut address = params.socket_address()?;
    address.set_port(params.script_port);
    let mut stream =
        TcpStream::connect_timeout(&address, duration(params.connect_timeout)?).map_err(io)?;
    stream
        .set_write_timeout(Some(Duration::from_millis(100)))
        .map_err(io)?;
    stream.write_all(text.as_bytes()).map_err(io)
}

pub(crate) fn run(
    mut stream: TcpStream,
    recipe: u8,
    _input: u8,
    params: &CuUrParams,
    shared: &Mutex<Shared>,
    shutdown: &AtomicBool,
) -> CuResult<()> {
    #[cfg(feature = "control")]
    let mut owned_motion = false;
    #[cfg(feature = "control")]
    let mut control = crate::control::Control::default();
    let result = (|| {
        let mut frame = [0; CAPACITY];
        let mut used = 0;
        let mut last_sample = Instant::now();
        while !shutdown.load(Ordering::Acquire) {
            if last_sample.elapsed() > duration(params.stale_after)? {
                return Err(CuError::from("UR telemetry timed out"));
            }
            #[cfg(feature = "control")]
            control.tick(&mut stream, _input, params, shared)?;
            #[cfg(feature = "control")]
            {
                let mut state = lock(shared)?;
                if let Some((target, queued)) = state.pending.take() {
                    let fresh = queued.elapsed() <= duration(params.stale_after)?;
                    let ready = state.latest.as_ref().is_some_and(|(at, s)| {
                        at.elapsed() <= duration(params.stale_after).unwrap()
                            && s.ready()
                            && s.runtime_state == 1
                    });
                    if !(fresh && ready) {
                        return Err(CuError::from(
                            "UR became unavailable before motion dispatch",
                        ));
                    }
                    // Never hold the shared mutex across network I/O.
                    drop(state);
                    owned_motion = true;
                    script(
                            params,
                            &protocol::move_script(
                                target,
                                params
                                    .speed
                                    .get::<cu29::units::si::angular_velocity::radian_per_second>(),
                                params.acceleration.get::<cu29::units::si::angular_acceleration::radian_per_second_squared>(),
                            ),
                        )?;
                    state = lock(shared)?;
                    state.active = Some((target, Instant::now()));
                }
                if state
                    .active
                    .as_ref()
                    .is_some_and(|(_, at)| at.elapsed() > duration(params.move_timeout).unwrap())
                {
                    return Err(CuError::from("UR move timed out"));
                }
            }
            match stream.read(&mut frame[used..]) {
                Ok(0) => return Err(CuError::from("UR disconnected")),
                Ok(n) => used += n,
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
            }
            while used >= 3 {
                let size = usize::from(u16::from_be_bytes([frame[0], frame[1]]));
                if !(3..=CAPACITY).contains(&size) {
                    return Err(CuError::from("UR frame outside capacity"));
                }
                if used < size {
                    break;
                }
                if frame[2] != b'U' {
                    return Err(CuError::from("UR unexpected RTDE packet"));
                }
                let status = protocol::decode(&frame[3..size], recipe)?;
                let mut state = lock(shared)?;
                #[cfg(feature = "control")]
                if let Some((target, _)) = state.active {
                    if !status.ready() {
                        return Err(CuError::from("UR motion interrupted by controller"));
                    }
                    let reached = status.joints.iter().zip(target).all(|(actual, goal)| {
                        (actual.get::<cu29::units::si::angle::radian>() - goal).abs()
                            <= params
                                .goal_tolerance
                                .get::<cu29::units::si::angle::radian>()
                    });
                    let stopped = status.velocities.iter().all(|v| {
                        v.get::<cu29::units::si::angular_velocity::radian_per_second>()
                            .abs()
                            < 0.001
                    });
                    if reached && stopped && status.runtime_state == 1 {
                        state.active = None;
                        owned_motion = false;
                    }
                }
                last_sample = Instant::now();
                state.latest = Some((last_sample, status));
                state.generation = state.generation.wrapping_add(1);
                drop(state);
                frame.copy_within(size..used, 0);
                used -= size;
            }
        }
        Ok(())
    })();
    #[cfg(feature = "control")]
    if owned_motion {
        let _ = script(params, "stopj(2.0)\n");
    }
    #[cfg(feature = "control")]
    control.stop(params);
    result
}

#[cfg(all(test, feature = "control"))]
mod tests {
    use super::*;
    use std::net::TcpListener;
    #[test]
    fn dispatches_move_and_stops_owned_motion() {
        let rtde = TcpListener::bind("127.0.0.1:0").unwrap();
        let scripts = TcpListener::bind("127.0.0.1:0").unwrap();
        let client = TcpStream::connect(rtde.local_addr().unwrap()).unwrap();
        client
            .set_read_timeout(Some(Duration::from_millis(20)))
            .unwrap();
        let (_rtde_peer, _) = rtde.accept().unwrap();
        let params = CuUrParams {
            script_port: scripts.local_addr().unwrap().port(),
            ..Default::default()
        };
        let shared = Mutex::new(Shared {
            latest: Some((
                Instant::now(),
                CuUrStatus {
                    robot_mode: 7,
                    safety_mode: 1,
                    runtime_state: 1,
                    ..Default::default()
                },
            )),
            pending: Some(([0.1; 6], Instant::now())),
            ..Default::default()
        });
        let shutdown = AtomicBool::new(false);
        std::thread::scope(|scope| {
            let worker = scope.spawn(|| run(client, 1, 0, &params, &shared, &shutdown));
            let (mut peer, _) = scripts.accept().unwrap();
            peer.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
            let mut text = String::new();
            peer.read_to_string(&mut text).unwrap();
            assert_eq!(
                text,
                "def repose_move():\n  movej([0.1,0.1,0.1,0.1,0.1,0.1], a=0.2, v=0.1, t=0, r=0)\nend\n"
            );
            shutdown.store(true, Ordering::Release);
            let (mut peer, _) = scripts.accept().unwrap();
            peer.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
            text.clear();
            peer.read_to_string(&mut text).unwrap();
            assert_eq!(text, "stopj(2.0)\n");
            worker.join().unwrap().unwrap();
        });
    }
}
