//! Acquire control, enable the drives, move one joint by `step` and back,
//! disable, release, then signal [`DONE`].

use cu29::prelude::*;
use cu29::units::si::angle::degree;
use cu29::units::si::f64::Angle;
use repose_controller::CuControllerMode;
use repose_controller::CuJointCommandPayload;
use repose_kinematics::CuJointPosition;
use repose_kinematics::CuJointStatePayload;
use repose_robot_interface::CuSessionCommand;
use repose_robot_interface::CuSessionResult;
use repose_robot_interface::CuSessionState;
use serde::Deserialize;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;

/// Set when the sequence is finished (or on Ctrl-C); `main` then stops the app.
pub static DONE: AtomicBool = AtomicBool::new(false);

/// A move counts as reached within this many degrees of the target.
const REACHED_DEG: f64 = 0.5;
/// Give up on a phase after this long.
const PHASE_TIMEOUT_S: u64 = 120;

#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, bincode::Encode, bincode::Decode, Reflect,
)]
enum Phase {
    Acquire,
    ClearFaults,
    Enable,
    MoveOut,
    MoveBack,
    Disable,
    Release,
    Finished,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Params {
    /// Joint to move, 1-6.
    joint: usize,
    /// Step, SI radians.
    step: Angle,
}

#[derive(Reflect)]
pub struct RebelSequence {
    joint: usize,
    step: Angle,
    phase: Phase,
    cycles: u32,
    #[reflect(ignore)]
    phase_started: Option<CuTime>,
    session: CuSessionState,
    #[reflect(ignore)]
    start: Option<[CuJointPosition; 6]>,
}

impl Freezable for RebelSequence {}

impl RebelSequence {
    fn advance(&mut self, next: Phase) {
        println!("sequence: {:?} -> {next:?}", self.phase);
        self.phase = next;
        self.cycles = 0;
        self.phase_started = None;
    }

    /// Send `command` until the bridge reports it acknowledged; then advance.
    fn session_step(
        &mut self,
        command: CuSessionCommand,
        next: Phase,
        out: &mut CuMsg<CuSessionCommand>,
    ) {
        let s = &self.session;
        if s.command == Some(command) && s.result == CuSessionResult::Acknowledged {
            self.advance(next);
        } else if s.command == Some(command) && s.result == CuSessionResult::Rejected {
            println!("sequence: {command:?} rejected: {}", s.detail.as_str());
            self.advance(Phase::Disable);
        } else {
            out.set_payload(command);
        }
    }

    fn target(&self, start: &[CuJointPosition; 6], out: bool) -> [CuJointPosition; 6] {
        let mut target = *start;
        if out && let CuJointPosition::Revolute(a) = start[self.joint - 1] {
            target[self.joint - 1] = CuJointPosition::Revolute(a + self.step);
        }
        target
    }
}

fn reached(measured: &[CuJointPosition; 6], target: &[CuJointPosition; 6]) -> bool {
    measured.iter().zip(target).all(|(m, t)| match (m, t) {
        (CuJointPosition::Revolute(m), CuJointPosition::Revolute(t)) => {
            (*m - *t).get::<degree>().abs() <= REACHED_DEG
        }
        _ => false,
    })
}

impl CuTask for RebelSequence {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, CuJointStatePayload<6>, CuSessionState);
    type Output<'m> = output_msg!(CuSessionCommand, CuJointCommandPayload<6>);

    fn new(cfg: Option<&ComponentConfig>, _res: Self::Resources<'_>) -> CuResult<Self> {
        let params: Params = cfg
            .ok_or_else(|| CuError::from("sequence: missing config"))?
            .deserialize_into()?;
        if !(1..=6).contains(&params.joint) {
            return Err(CuError::from("sequence: joint must be 1-6"));
        }
        Ok(Self {
            joint: params.joint,
            step: params.step,
            phase: Phase::Acquire,
            cycles: 0,
            phase_started: None,
            session: CuSessionState::default(),
            start: None,
        })
    }

    fn process<'i, 'o>(
        &mut self,
        ctx: &CuContext,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        let (joints, session) = *input;
        let (session_out, command_out) = output;
        session_out.clear_payload();
        command_out.clear_payload();
        if let Some(s) = session.payload() {
            self.session = s.clone();
        }
        let measured = joints.payload().map(|j| j.positions);
        self.cycles += 1;
        let now = ctx.now();
        let started = *self.phase_started.get_or_insert(now);
        let timed_out = now - started > CuDuration::from_secs(PHASE_TIMEOUT_S);
        if timed_out && self.phase < Phase::Disable {
            println!("sequence: {:?} timed out", self.phase);
            self.advance(Phase::Disable);
        }
        match self.phase {
            Phase::Acquire => self.session_step(
                CuSessionCommand::AcquireControl,
                Phase::ClearFaults,
                session_out,
            ),
            Phase::ClearFaults => {
                self.session_step(CuSessionCommand::ClearFaults, Phase::Enable, session_out)
            }
            Phase::Enable => {
                if self.session.motion_allowed {
                    if let Some(m) = measured {
                        self.start = Some(m);
                        self.advance(Phase::MoveOut);
                    }
                } else if !self.session.drives_enabled {
                    self.session_step(CuSessionCommand::EnableDrives, Phase::Enable, session_out);
                }
            }
            Phase::MoveOut | Phase::MoveBack => {
                let (Some(start), Some(measured)) = (self.start, measured) else {
                    return Ok(());
                };
                let target = self.target(&start, self.phase == Phase::MoveOut);
                if reached(&measured, &target) && self.cycles > 5 {
                    let next = if self.phase == Phase::MoveOut {
                        Phase::MoveBack
                    } else {
                        Phase::Disable
                    };
                    self.advance(next);
                } else {
                    // Hold the target every cycle; the bridge sends it once.
                    command_out.set_payload(CuJointCommandPayload {
                        mode: CuControllerMode::Position,
                        positions: target,
                        ..Default::default()
                    });
                }
            }
            Phase::Disable => {
                if self.session.drives_enabled {
                    self.session_step(CuSessionCommand::DisableDrives, Phase::Release, session_out);
                } else {
                    self.advance(Phase::Release);
                }
            }
            Phase::Release => {
                if self.session.control_acquired {
                    self.session_step(
                        CuSessionCommand::ReleaseControl,
                        Phase::Finished,
                        session_out,
                    );
                } else {
                    self.advance(Phase::Finished);
                }
            }
            Phase::Finished => DONE.store(true, Ordering::Release),
        }
        Ok(())
    }
}
