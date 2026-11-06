//! Serial I/O lives exclusively on a worker after startup negotiation.
use crate::{
    params::{CuPantheraHtParams, JOINTS, duration},
    protocol::{self, CAPACITY},
};
use cu29::prelude::*;
use std::{
    sync::{
        Mutex,
        atomic::{AtomicBool, Ordering},
    },
    time::{Duration, Instant},
};

/// USB VID:PID of the v6 comm board.
const COMM_BOARD_VID: u16 = 0xCAF1;
const COMM_BOARD_PID: u16 = 0xFFFF;

#[derive(Default)]
pub(crate) struct Shared {
    pub latest: Option<(Instant, protocol::CuPantheraStatus)>,
    pub generation: u64,
    pub failed: bool,
    pub session: repose_robot_interface::CuSessionTracker,
    /// Latest position command (rad per joint) and when it was queued.
    #[cfg(feature = "control")]
    pub pending: Option<([f64; JOINTS], Instant)>,
    #[cfg(feature = "control")]
    pub last_command: Option<[f64; JOINTS]>,
}

fn io(error: std::io::Error) -> CuError {
    CuError::new_with_cause("Panthera serial I/O failed", error)
}
fn lock(shared: &Mutex<Shared>) -> CuResult<std::sync::MutexGuard<'_, Shared>> {
    shared
        .lock()
        .map_err(|_| CuError::from("Panthera worker state poisoned"))
}

/// Pick the comm-board serial device: explicit `serial_port`, else the
/// `can_port_id`-th detected port with the comm-board VID:PID.
pub(crate) fn resolve_port(params: &CuPantheraHtParams) -> CuResult<String> {
    if !params.serial_port.is_empty() {
        return Ok(params.serial_port.clone());
    }
    let mut ports = serialport::available_ports()
        .map_err(|e| CuError::new_with_cause("Panthera port scan failed", e))?;
    ports.retain(|p| {
        matches!(&p.port_type, serialport::SerialPortType::UsbPort(info)
            if info.vid == COMM_BOARD_VID && info.pid == COMM_BOARD_PID)
    });
    ports.sort_by(|a, b| a.port_name.cmp(&b.port_name));
    let index = usize::from(params.can_port_id.saturating_sub(1));
    ports
        .get(index)
        .map(|p| p.port_name.clone())
        .ok_or_else(|| CuError::from("Panthera comm board serial port not found"))
}

fn send_frame<P: serialport::SerialPort + ?Sized>(
    port: &mut P,
    cmd: u8,
    data: &[u8],
) -> CuResult<()> {
    let mut frame = [0u8; CAPACITY];
    let len = 6 + data.len();
    protocol::encode_frame(&mut frame[..len], cmd, data)?;
    port.write_all(&frame[..len]).map_err(io)?;
    port.flush().map_err(io)
}

/// Open the port and run the comm-board handshake: version (>= 6.0.0), init,
/// cache size, then two state requests.
pub(crate) fn connect(params: &CuPantheraHtParams) -> CuResult<Box<dyn serialport::SerialPort>> {
    let port_name = resolve_port(params)?;
    let mut port = serialport::new(&port_name, params.baud)
        .timeout(Duration::from_millis(100))
        .open()
        .map_err(|e| CuError::new_with_cause("Panthera serial open failed", e))?;
    let deadline = Instant::now() + duration(params.connect_timeout)?;
    // Comm-board version.
    let mut version = [0u8; 3];
    while Instant::now() < deadline {
        send_frame(&mut *port, protocol::MODE_COMM_VERSION, &[])?;
        std::thread::sleep(Duration::from_millis(2));
        if let Some(v) = read_version(&mut *port)? {
            version = v;
            break;
        }
    }
    if version[0] < 6 {
        return Err(CuError::from(
            "Panthera comm board version too low: require >= v6.0.0",
        ));
    }
    // Comm init.
    let mut inited = false;
    while Instant::now() < deadline {
        send_frame(&mut *port, protocol::MODE_COMM_INIT, &[])?;
        std::thread::sleep(Duration::from_millis(2));
        if matches!(read_flag(&mut *port)?, Some(protocol::Frame::CommInit(1))) {
            inited = true;
            break;
        }
    }
    if !inited {
        return Err(CuError::from("Panthera comm board init failed"));
    }
    // Cache size = number of configured motors.
    send_frame(
        &mut *port,
        protocol::MODE_CACHE_NUM,
        &protocol::encode_cache_num(JOINTS as u8),
    )?;
    // Prime motor state.
    let request = protocol::encode_state_request();
    send_frame(&mut *port, protocol::MODE_MOTOR_STATE, &request)?;
    send_frame(&mut *port, protocol::MODE_MOTOR_STATE, &request)?;
    Ok(port)
}

/// Read and parse frames until one matches the wanted predicate or timeout.
fn read_frame<P: serialport::SerialPort + ?Sized>(
    port: &mut P,
    buf: &mut [u8],
    used: &mut usize,
    timeout: Duration,
) -> CuResult<Option<(u8, protocol::Frame)>> {
    let deadline = Instant::now() + timeout;
    loop {
        if Instant::now() >= deadline {
            return Ok(None);
        }
        match port.read(&mut buf[*used..]) {
            Ok(0) => return Err(CuError::from("Panthera comm board disconnected")),
            Ok(n) => *used += n,
            Err(e)
                if matches!(
                    e.kind(),
                    std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                ) => {}
            Err(e) => return Err(io(e)),
        }
        while *used >= 6 {
            if buf[0] != protocol::PROT_HEAD {
                // Resynchronize: drop one byte.
                buf.copy_within(1..*used, 0);
                *used -= 1;
                continue;
            }
            let len = u16::from_le_bytes([buf[2], buf[3]]) as usize;
            if len > protocol::PROT_DATA_LEN {
                return Err(CuError::from("Panthera frame length outside capacity"));
            }
            let total = 6 + len;
            if *used < total {
                break;
            }
            let frame = protocol::decode_frame(&buf[..total])?;
            buf.copy_within(total..*used, 0);
            *used -= total;
            return Ok(Some(frame));
        }
    }
}

fn read_version<P: serialport::SerialPort + ?Sized>(port: &mut P) -> CuResult<Option<[u8; 3]>> {
    let mut buf = [0u8; CAPACITY];
    let mut used = 0;
    while let Some((cmd, frame)) = read_frame(port, &mut buf, &mut used, Duration::from_millis(50))?
    {
        if cmd == protocol::MODE_COMM_VERSION
            && let protocol::Frame::CommVersion(v) = frame
        {
            return Ok(Some(v));
        }
    }
    Ok(None)
}

fn read_flag<P: serialport::SerialPort + ?Sized>(
    port: &mut P,
) -> CuResult<Option<protocol::Frame>> {
    let mut buf = [0u8; CAPACITY];
    let mut used = 0;
    while let Some((_, frame)) = read_frame(port, &mut buf, &mut used, Duration::from_millis(50))? {
        match frame {
            protocol::Frame::CommInit(_) | protocol::Frame::CacheNum(_) => return Ok(Some(frame)),
            _ => {}
        }
    }
    Ok(None)
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn run(
    mut port: Box<dyn serialport::SerialPort>,
    params: &CuPantheraHtParams,
    shared: &Mutex<Shared>,
    shutdown: &AtomicBool,
) -> CuResult<()> {
    let request = protocol::encode_state_request();
    let tick = Duration::from_secs_f64(1.0 / params.frequency);
    let stale_after = duration(params.stale_after)?;
    #[cfg(feature = "control")]
    let command_timeout = duration(params.command_timeout)?;
    let mut buf = [0u8; CAPACITY];
    let mut used = 0;
    let mut last_request = Instant::now() - tick;
    let mut comm_version = [0u8; 3];
    let result = (|| {
        while !shutdown.load(Ordering::Acquire) {
            if last_request.elapsed() >= tick {
                send_frame(&mut *port, protocol::MODE_MOTOR_STATE, &request)?;
                last_request = Instant::now();
            }
            #[cfg(feature = "control")]
            {
                let mut state = lock(shared)?;
                if let Some((target, queued)) = state.pending
                    && queued.elapsed() <= command_timeout
                {
                    let data = protocol::encode_position_command(
                        &target,
                        &params.joint_ids,
                        params.kp,
                        params.kd,
                    )?;
                    drop(state);
                    send_frame(&mut *port, protocol::MODE_POS_VEL_TQE_KP_KD, &data)?;
                    state = lock(shared)?;
                    state.last_command = Some(target);
                }
                if let Some(command) = state.session.take_outgoing() {
                    let sequence = state.session.state().sequence;
                    state.session.mark_sent(command, sequence);
                    state.session.on_acknowledged(sequence);
                }
                state.session.on_control_report(true);
                drop(state);
            }
            while let Some((cmd, frame)) = read_frame(&mut *port, &mut buf, &mut used, tick)? {
                match frame {
                    protocol::Frame::CommVersion(v) => comm_version = v,
                    protocol::Frame::MotorState(state) => {
                        let protocol::MotorStateFrame {
                            can_fault,
                            tx_err,
                            rx_err,
                            query,
                            records,
                            count,
                        } = *state;
                        let records = &records[..count];
                        let status = protocol::status_from_records(
                            comm_version,
                            can_fault,
                            tx_err,
                            rx_err,
                            query,
                            records,
                            &params.joint_ids,
                        )?;
                        #[cfg(feature = "control")]
                        let drives_ready = status.motors.iter().all(|m| m.fault == 0);
                        let mut state = lock(shared)?;
                        state.latest = Some((Instant::now(), status));
                        state.generation = state.generation.wrapping_add(1);
                        #[cfg(feature = "control")]
                        state
                            .session
                            .on_drive_report(drives_ready, params.allow_motion);
                    }
                    _ => {}
                }
                let _ = cmd;
            }
            // Stale telemetry fails the worker so the bridge reports an error.
            let state = lock(shared)?;
            if let Some((at, _)) = &state.latest
                && at.elapsed() > stale_after
            {
                return Err(CuError::from("Panthera telemetry timed out"));
            }
            drop(state);
        }
        Ok(())
    })();
    #[cfg(feature = "control")]
    if let Ok(mut state) = shared.lock() {
        state.session.on_disconnected();
    }
    result
}
