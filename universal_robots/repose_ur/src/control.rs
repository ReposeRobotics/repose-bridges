//! Session lifecycle and velocity streaming, entirely on the transport worker.
use crate::{
    params::{CuUrParams, duration},
    protocol::CuUrStatus,
    transport::{Shared, script, write_packet},
};
use cu29::prelude::*;
use repose_robot_interface::CuSessionCommand;
use std::{
    io::{Read, Write},
    net::TcpStream,
    time::{Duration, Instant},
};

pub(crate) const INPUT_FIELDS: &[u8] = b"input_int_register_24,input_double_register_24,input_double_register_25,input_double_register_26,input_double_register_27,input_double_register_28,input_double_register_29";
pub(crate) const INPUT_TYPES: &[u8] = b"INT32,DOUBLE,DOUBLE,DOUBLE,DOUBLE,DOUBLE,DOUBLE";
pub(crate) const MARKER: i32 = 19729;

fn line(stream: &mut TcpStream) -> CuResult<String> {
    let mut bytes = [0; 512];
    for i in 0..bytes.len() {
        stream
            .read_exact(&mut bytes[i..=i])
            .map_err(|e| CuError::new_with_cause("UR dashboard read", e))?;
        if bytes[i] == b'\n' {
            return String::from_utf8(bytes[..i].to_vec())
                .map_err(|e| CuError::new_with_cause("UR dashboard UTF-8", e));
        }
    }
    Err(CuError::from("UR dashboard line exceeds capacity"))
}
fn dashboard(params: &CuUrParams, command: &str) -> CuResult<String> {
    let mut address = params.socket_address()?;
    address.set_port(params.dashboard_port);
    let timeout = duration(params.connect_timeout)?;
    let mut stream = TcpStream::connect_timeout(&address, timeout)
        .map_err(|e| CuError::new_with_cause("UR dashboard connect", e))?;
    stream
        .set_read_timeout(Some(timeout))
        .map_err(|e| CuError::new_with_cause("UR dashboard timeout", e))?;
    stream
        .set_write_timeout(Some(timeout))
        .map_err(|e| CuError::new_with_cause("UR dashboard timeout", e))?;
    line(&mut stream)?;
    writeln!(stream, "{command}").map_err(|e| CuError::new_with_cause("UR dashboard write", e))?;
    line(&mut stream)
}
pub(crate) fn resident(params: &CuUrParams) -> String {
    format!(
        r#"def repose_stream():
  rtde_set_watchdog("input_int_register_24", 8, "stop")
  write_output_integer_register(24, 19729)
  while read_input_integer_register(24) == 1:
    qd = [read_input_float_register(24),read_input_float_register(25),read_input_float_register(26),read_input_float_register(27),read_input_float_register(28),read_input_float_register(29)]
    speedj(qd, a={}, t=0.02)
  end
  stopj(2.0)
  write_output_integer_register(24, 0)
end
"#,
        params
            .acceleration
            .get::<cu29::units::si::angular_acceleration::radian_per_second_squared>()
    )
}

#[derive(Default)]
pub(crate) struct Control {
    pub streaming: bool,
    launched: bool,
    enabled_here: bool,
    sequence: u32,
    pending: Option<(CuSessionCommand, Instant)>,
    stage: u8,
    last_send: Option<Instant>,
}
impl Control {
    pub(crate) fn tick(
        &mut self,
        stream: &mut TcpStream,
        input: u8,
        params: &CuUrParams,
        shared: &std::sync::Mutex<Shared>,
    ) -> CuResult<()> {
        let status = shared
            .lock()
            .map_err(|_| CuError::from("UR state poisoned"))?
            .latest
            .as_ref()
            .map(|(_, s)| s.clone());
        if self.pending.is_none() {
            let command = shared
                .lock()
                .map_err(|_| CuError::from("UR state poisoned"))?
                .session
                .take_outgoing();
            if let Some(command) = command {
                self.sequence = self.sequence.wrapping_add(1);
                shared
                    .lock()
                    .unwrap()
                    .session
                    .mark_sent(command, self.sequence);
                self.pending = Some((command, Instant::now()));
                self.stage = 0;
            }
        }
        if let Some((command, started)) = self.pending {
            if started.elapsed() > duration(params.move_timeout)? {
                return Err(CuError::from("UR session command timed out"));
            }
            if let Some(status) = &status {
                let outcome = self.session(command, status, params);
                match outcome {
                    Ok(true) => {
                        shared
                            .lock()
                            .unwrap()
                            .session
                            .on_acknowledged(self.sequence);
                        self.pending = None;
                    }
                    Ok(false) => {}
                    Err(error) => {
                        shared
                            .lock()
                            .unwrap()
                            .session
                            .on_rejected(self.sequence, &error.to_string());
                        self.pending = None;
                    }
                }
            }
        }
        if self.launched
            && self.pending.is_none()
            && self.streaming
            && let Some(status) = &status
            && (status.control_marker != MARKER || status.runtime_state != 2 || !status.ready())
        {
            return Err(CuError::from("UR streaming control program interrupted"));
        }
        if input != 0
            && self
                .last_send
                .is_none_or(|at| at.elapsed() >= Duration::from_millis(20))
        {
            let velocity = {
                let mut state = shared
                    .lock()
                    .map_err(|_| CuError::from("UR state poisoned"))?;
                let ready = status
                    .as_ref()
                    .is_some_and(|s| s.ready() && s.control_marker == MARKER);
                state.session.on_drive_report(
                    status.as_ref().is_some_and(CuUrStatus::ready),
                    self.streaming && ready,
                );
                if self.streaming && ready {
                    state
                        .velocity
                        .filter(|(_, at)| at.elapsed() <= duration(params.command_timeout).unwrap())
                        .map(|(v, _)| v)
                        .unwrap_or([0.0; 6])
                } else {
                    [0.0; 6]
                }
            };
            let mut body = [0; 53];
            body[0] = input;
            body[1..5].copy_from_slice(&i32::from(self.streaming).to_be_bytes());
            for (bytes, value) in body[5..].as_chunks_mut::<8>().0.iter_mut().zip(velocity) {
                *bytes = value.to_be_bytes();
            }
            write_packet(stream, b'U', &body)?;
            self.last_send = Some(Instant::now());
        }
        Ok(())
    }
    fn session(
        &mut self,
        command: CuSessionCommand,
        status: &CuUrStatus,
        params: &CuUrParams,
    ) -> CuResult<bool> {
        match command {
            CuSessionCommand::AcquireControl => {
                if dashboard(params, "is in remote control")?.trim() != "true" {
                    return Err(CuError::from("UR requires remote control"));
                }
                if status.runtime_state != 1 {
                    return Err(CuError::from("UR another program is running"));
                }
                Ok(true)
            }
            CuSessionCommand::ClearFaults => {
                if !matches!(status.safety_mode, 1 | 2) {
                    return Err(CuError::from("UR safety fault needs manual reset"));
                }
                Ok(true)
            }
            CuSessionCommand::EnableDrives => {
                if self.stage == 0 {
                    self.enabled_here = true;
                    if status.robot_mode == 3 {
                        let answer = dashboard(params, "power on")?;
                        if !answer.starts_with("Powering on") {
                            return Err(CuError::from(answer));
                        }
                    }
                    self.stage = 1;
                }
                if status.robot_mode == 5 && self.stage == 1 {
                    let answer = dashboard(params, "brake release")?;
                    if !answer.starts_with("Brake releasing") {
                        return Err(CuError::from(answer));
                    }
                    self.stage = 2;
                }
                if status.ready() && self.stage < 3 {
                    if status.runtime_state != 1 {
                        return Err(CuError::from("UR another program started"));
                    }
                    if !params.allow_motion {
                        return Err(CuError::from("UR streaming requires allow_motion"));
                    }
                    self.streaming = true;
                    self.last_send = None;
                    self.launched = true;
                    // tick publishes a fresh run flag before launching on the next cycle.
                    self.stage = 3;
                    return Ok(false);
                }
                if self.stage == 3 {
                    script(params, &resident(params))?;
                    self.stage = 4;
                    return Ok(false);
                }
                Ok(self.stage == 4 && status.control_marker == MARKER && status.runtime_state == 2)
            }
            CuSessionCommand::DisableDrives => {
                self.streaming = false;
                if self.stage == 0 {
                    // Stop only our resident program; never replace an unrelated one.
                    if self.launched {
                        let answer = dashboard(params, "stop")?;
                        if !answer.starts_with("Stopped") {
                            return Err(CuError::from(answer));
                        }
                    }
                    let answer = dashboard(params, "power off")?;
                    if !answer.starts_with("Powering off") {
                        return Err(CuError::from(answer));
                    }
                    self.stage = 1;
                    self.launched = false;
                }
                if status.robot_mode == 3 {
                    self.enabled_here = false;
                }
                Ok(status.robot_mode == 3)
            }
            CuSessionCommand::ReleaseControl => {
                self.stop(params);
                Ok(true)
            }
        }
    }
    pub(crate) fn stop(&mut self, params: &CuUrParams) {
        if self.launched {
            let _ = dashboard(params, "stop");
        }
        if self.enabled_here {
            let _ = dashboard(params, "power off");
        }
        self.enabled_here = false;
        self.streaming = false;
        self.launched = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{net::TcpListener, sync::Mutex};
    #[test]
    fn expired_velocity_is_replaced_with_zero_on_wire() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let mut stream = TcpStream::connect(listener.local_addr().unwrap()).unwrap();
        let (mut peer, _) = listener.accept().unwrap();
        peer.set_read_timeout(Some(Duration::from_secs(1))).unwrap();
        let params = CuUrParams::default();
        let shared = Mutex::new(Shared {
            latest: Some((
                Instant::now(),
                CuUrStatus {
                    robot_mode: 7,
                    safety_mode: 1,
                    runtime_state: 2,
                    control_marker: MARKER,
                    ..Default::default()
                },
            )),
            velocity: Some(([0.1; 6], Instant::now())),
            ..Default::default()
        });
        let mut control = Control {
            streaming: true,
            ..Default::default()
        };
        control.tick(&mut stream, 3, &params, &shared).unwrap();
        let mut packet = [0; 56];
        peer.read_exact(&mut packet).unwrap();
        assert_eq!(&packet[..8], &[0, 56, b'U', 3, 0, 0, 0, 1]);
        assert_eq!(f64::from_be_bytes(packet[8..16].try_into().unwrap()), 0.1);
        shared.lock().unwrap().velocity = Some(([0.1; 6], Instant::now() - Duration::from_secs(1)));
        control.last_send = None;
        control.tick(&mut stream, 3, &params, &shared).unwrap();
        peer.read_exact(&mut packet).unwrap();
        assert!(packet[8..].iter().all(|b| *b == 0));
        control.streaming = false;
        control.last_send = None;
        control.tick(&mut stream, 3, &params, &shared).unwrap();
        peer.read_exact(&mut packet).unwrap();
        assert_eq!(&packet[4..8], &[0, 0, 0, 0]);
    }
}
