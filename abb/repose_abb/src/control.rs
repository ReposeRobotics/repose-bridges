//! Session rules and EGM correction replies, entirely on the transport worker.
//!
//! EGM has no remote power or program control: the operator switches motors
//! on and runs the RAPID EGM program. Session commands here gate whether this
//! bridge answers EGM feedback with corrections, and are acknowledged from
//! observed EGM state.
use crate::{
    params::{CuAbbParams, duration},
    protocol::{
        self, CuAbbMciState, CuAbbMotorState, CuAbbRapidState, CuAbbStatus, SENSOR_CAPACITY,
    },
    transport::{Shared, lock},
};
use cu29::prelude::*;
use repose_robot_interface::CuSessionCommand;
use std::{sync::Mutex, time::Instant};

pub(crate) struct Control {
    streaming: bool,
    sequence: u32,
    pending: Option<(CuSessionCommand, Instant)>,
    /// Position held while no fresh target exists, latched from feedback.
    hold: Option<[f64; 6]>,
    reply_sequence: u32,
    started: Instant,
    buffer: [u8; SENSOR_CAPACITY],
}
impl Control {
    pub(crate) fn new() -> Self {
        Self {
            streaming: false,
            sequence: 0,
            pending: None,
            hold: None,
            reply_sequence: 0,
            started: Instant::now(),
            buffer: [0; SENSOR_CAPACITY],
        }
    }
    pub(crate) fn streaming(&self) -> bool {
        self.streaming
    }

    pub(crate) fn session(
        &mut self,
        params: &CuAbbParams,
        shared: &Mutex<Shared>,
        fresh: bool,
    ) -> CuResult<()> {
        let mut state = lock(shared)?;
        let status = state
            .latest
            .as_ref()
            .filter(|_| fresh)
            .map(|(_, s)| s.clone());
        if self.streaming
            && let Some(s) = &status
            && (s.motors != CuAbbMotorState::On
                || s.mci == CuAbbMciState::Error
                || s.rapid != CuAbbRapidState::Running)
        {
            self.streaming = false;
            return Err(CuError::from("ABB EGM motion stopped by controller"));
        }
        state.session.on_drive_report(
            status
                .as_ref()
                .is_some_and(|s| s.motors == CuAbbMotorState::On),
            self.streaming
                && status
                    .as_ref()
                    .is_some_and(|s| s.mci == CuAbbMciState::Running),
        );
        if self.pending.is_none()
            && let Some(command) = state.session.take_outgoing()
        {
            self.sequence = self.sequence.wrapping_add(1);
            state.session.mark_sent(command, self.sequence);
            self.pending = Some((command, Instant::now()));
        }
        let Some((command, since)) = self.pending else {
            return Ok(());
        };
        let outcome = if since.elapsed() > duration(params.session_timeout)? {
            Err(CuError::from("ABB session command timed out"))
        } else {
            self.evaluate(command, status.as_ref(), params)
        };
        match outcome {
            Ok(false) => {}
            Ok(true) => {
                state.session.on_acknowledged(self.sequence);
                self.pending = None;
            }
            Err(error) => {
                if command == CuSessionCommand::EnableDrives {
                    self.streaming = false;
                }
                state.session.on_rejected(self.sequence, &error.to_string());
                self.pending = None;
            }
        }
        Ok(())
    }

    /// Ok(true) acknowledges, Ok(false) keeps waiting, Err rejects.
    fn evaluate(
        &mut self,
        command: CuSessionCommand,
        status: Option<&CuAbbStatus>,
        params: &CuAbbParams,
    ) -> CuResult<bool> {
        match command {
            CuSessionCommand::ReleaseControl | CuSessionCommand::DisableDrives => {
                self.streaming = false;
                Ok(true)
            }
            _ if status.is_none() => Ok(false),
            CuSessionCommand::AcquireControl => Ok(true),
            CuSessionCommand::ClearFaults => {
                if status.unwrap().mci == CuAbbMciState::Error {
                    return Err(CuError::from("ABB EGM error: reset RAPID program"));
                }
                Ok(true)
            }
            CuSessionCommand::EnableDrives => {
                let status = status.unwrap();
                if !params.allow_motion {
                    return Err(CuError::from("ABB streaming requires allow_motion"));
                }
                if status.motors != CuAbbMotorState::On {
                    return Err(CuError::from("ABB motors off; switch on at controller"));
                }
                if status.rapid != CuAbbRapidState::Running {
                    return Err(CuError::from("ABB RAPID EGM program not running"));
                }
                if !self.streaming {
                    self.streaming = true;
                    self.hold = None;
                }
                Ok(status.mci == CuAbbMciState::Running)
            }
        }
    }

    /// The correction for one feedback message, or None when not streaming.
    pub(crate) fn reply(
        &mut self,
        status: &CuAbbStatus,
        params: &CuAbbParams,
        shared: &Mutex<Shared>,
    ) -> CuResult<Option<&[u8]>> {
        if !self.streaming {
            return Ok(None);
        }
        let timeout = duration(params.command_timeout)?;
        let target = lock(shared)?.target.filter(|t| t.at.elapsed() <= timeout);
        let (position, velocity) = match target {
            Some(t) => {
                self.hold = None;
                (t.position, t.velocity)
            }
            None => (*self.hold.get_or_insert(status.joints_rad()), [0.0; 6]),
        };
        self.reply_sequence = self.reply_sequence.wrapping_add(1);
        let time_ms = self.started.elapsed().as_millis() as u32;
        let n = protocol::encode_sensor(
            self.reply_sequence,
            time_ms,
            position,
            velocity,
            &mut self.buffer,
        );
        Ok(Some(&self.buffer[..n]))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::Target;
    use repose_robot_interface::CuSessionResult;

    fn status(mci: CuAbbMciState) -> CuAbbStatus {
        CuAbbStatus {
            motors: CuAbbMotorState::On,
            rapid: CuAbbRapidState::Running,
            mci,
            ..Default::default()
        }
    }
    fn params() -> CuAbbParams {
        CuAbbParams {
            allow_control: true,
            allow_motion: true,
            ..Default::default()
        }
    }
    fn request(shared: &Mutex<Shared>, command: CuSessionCommand) {
        shared.lock().unwrap().session.request(command);
    }
    fn result(shared: &Mutex<Shared>) -> CuSessionResult {
        shared.lock().unwrap().session.state().result
    }

    #[test]
    fn enable_waits_for_egm_running_then_streams_hold_and_targets() {
        let params = params();
        let shared = Mutex::new(Shared {
            latest: Some((Instant::now(), status(CuAbbMciState::Stopped))),
            ..Default::default()
        });
        let mut control = Control::new();
        assert!(
            control
                .reply(&status(CuAbbMciState::Stopped), &params, &shared)
                .unwrap()
                .is_none()
        );
        request(&shared, CuSessionCommand::AcquireControl);
        control.session(&params, &shared, true).unwrap();
        assert_eq!(result(&shared), CuSessionResult::Acknowledged);
        request(&shared, CuSessionCommand::EnableDrives);
        control.session(&params, &shared, true).unwrap();
        assert_eq!(result(&shared), CuSessionResult::Pending);
        assert!(control.streaming());
        // Holding measured position while EGM ramps in.
        let held = control
            .reply(&status(CuAbbMciState::Stopped), &params, &shared)
            .unwrap()
            .unwrap()
            .to_vec();
        assert!(!held.is_empty());
        shared.lock().unwrap().latest = Some((Instant::now(), status(CuAbbMciState::Running)));
        control.session(&params, &shared, true).unwrap();
        assert_eq!(result(&shared), CuSessionResult::Acknowledged);
        control.session(&params, &shared, true).unwrap();
        assert!(shared.lock().unwrap().session.state().motion_allowed);
        shared.lock().unwrap().target = Some(Target {
            position: [0.1; 6],
            velocity: [0.2; 6],
            at: Instant::now(),
        });
        let moving = control
            .reply(&status(CuAbbMciState::Running), &params, &shared)
            .unwrap()
            .unwrap()
            .to_vec();
        assert_ne!(moving[8..], held[8..]);
        request(&shared, CuSessionCommand::DisableDrives);
        control.session(&params, &shared, true).unwrap();
        assert!(!control.streaming());
        assert!(
            control
                .reply(&status(CuAbbMciState::Running), &params, &shared)
                .unwrap()
                .is_none()
        );
    }

    #[test]
    fn enable_rejected_when_motors_off_and_streaming_faults_on_stop() {
        let params = params();
        let mut off = status(CuAbbMciState::Stopped);
        off.motors = CuAbbMotorState::Off;
        let shared = Mutex::new(Shared {
            latest: Some((Instant::now(), off)),
            ..Default::default()
        });
        let mut control = Control::new();
        shared.lock().unwrap().session.on_control_report(true);
        request(&shared, CuSessionCommand::EnableDrives);
        control.session(&params, &shared, true).unwrap();
        assert_eq!(result(&shared), CuSessionResult::Rejected);
        assert!(!control.streaming());

        shared.lock().unwrap().latest = Some((Instant::now(), status(CuAbbMciState::Running)));
        request(&shared, CuSessionCommand::EnableDrives);
        control.session(&params, &shared, true).unwrap();
        assert!(control.streaming());
        shared.lock().unwrap().latest = Some((Instant::now(), status(CuAbbMciState::Error)));
        assert!(control.session(&params, &shared, true).is_err());
    }
}
