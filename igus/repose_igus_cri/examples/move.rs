//! First motion: move A6 by +5° and back with controller-planned `Move Joint`.
//!
//! ```sh
//! move [ADDRESS]                 # print the plan only; does not connect
//! move [ADDRESS] --allow-motion  # run it: the arm MOVES
//! ```
//!
//! Sequence: AcquireControl, ClearFaults, EnableDrives, wait for drives ready,
//! move A6 +5° at 5 % velocity, wait until reached, move back, DisableDrives,
//! ReleaseControl. Any failure stops; `stop` disables the drives.
use cu29::prelude::*;
use cu29::units::si::angle::degree;
use cu29::units::si::f64::Angle;
use repose_controller::CuControllerMode;
use repose_controller::CuJointCommandPayload;
use repose_igus_cri::CuCriMotionState;
use repose_igus_cri::CuCriMotionStatus;
use repose_igus_cri::CuCriStatus;
use repose_igus_cri::CuIgusCriBridge;
use repose_igus_cri::CuIgusCriParams;
use repose_igus_cri::IgusCriRxChannels;
use repose_igus_cri::IgusCriTxChannels;
use repose_kinematics::CuJointPosition;
use repose_robot_interface::CuSessionCommand;
use repose_robot_interface::CuSessionResult;
use repose_robot_interface::CuSessionState;
use std::time::Duration;
use std::time::Instant;

const STEP_DEG: f64 = 5.0;

struct Run {
    bridge: CuIgusCriBridge,
    ctx: CuContext,
    status: CuMsg<CuCriStatus>,
    state: CuMsg<CuSessionState>,
    motion: CuMsg<CuCriMotionState>,
    session: CuSessionState,
    last_motion: CuCriMotionState,
    joints: Option<[Angle; 6]>,
}

impl Run {
    fn poll(&mut self) -> CuResult<()> {
        self.bridge
            .receive(&self.ctx, &IgusCriRxChannels::CRI_STATUS, &mut self.status)?;
        if let Some(s) = self.status.payload() {
            self.joints = Some(s.joints);
        }
        self.bridge.receive(
            &self.ctx,
            &IgusCriRxChannels::SESSION_STATE,
            &mut self.state,
        )?;
        if let Some(s) = self.state.payload() {
            println!(
                "  session: {:?} → {:?} {:?} {} | control={} enabled={} ready={} motion_allowed={}",
                s.command,
                s.result,
                s.rejection,
                s.detail.as_str(),
                s.control_acquired,
                s.drives_enabled,
                s.drives_ready,
                s.motion_allowed
            );
            self.session = s.clone();
        }
        self.bridge
            .receive(&self.ctx, &IgusCriRxChannels::CRI_MOTION, &mut self.motion)?;
        if let Some(m) = self.motion.payload() {
            println!(
                "  motion: #{} {:?} {:?} {} | exec: {} | refused={}",
                m.sequence,
                m.status,
                m.rejection,
                m.detail.as_str(),
                m.last_exec.as_str(),
                m.refused
            );
            self.last_motion = m.clone();
        }
        Ok(())
    }

    fn wait(&mut self, seconds: f64, done: impl Fn(&Self) -> bool) -> CuResult<bool> {
        let end = Instant::now() + Duration::from_secs_f64(seconds);
        while Instant::now() < end {
            self.poll()?;
            if done(self) {
                return Ok(true);
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        Ok(false)
    }

    fn session_command(&mut self, command: CuSessionCommand) -> CuResult<bool> {
        println!("Sending {command:?}");
        let mut msg = CuMsg::<CuSessionCommand>::default();
        msg.set_payload(command);
        self.bridge
            .send(&self.ctx, &IgusCriTxChannels::SESSION, &msg)?;
        self.wait(3.0, |r| {
            r.session.command == Some(command) && r.session.result != CuSessionResult::Pending
        })?;
        let ok = self.session.command == Some(command)
            && self.session.result == CuSessionResult::Acknowledged;
        if !ok {
            println!("Stopping: {command:?} ended as {:?}", self.session.result);
        }
        Ok(ok)
    }

    /// Command `target` every cycle (as a controller task would) until reached.
    fn move_to(&mut self, target: [Angle; 6]) -> CuResult<bool> {
        let degrees = target.map(|a| a.get::<degree>());
        println!("Moving to {degrees:.2?}");
        let payload = CuJointCommandPayload::<6> {
            mode: CuControllerMode::Position,
            positions: target.map(CuJointPosition::Revolute),
            ..Default::default()
        };
        let mut msg = CuMsg::<CuJointCommandPayload<6>>::default();
        msg.set_payload(payload);
        let started = Instant::now();
        let end = started + Duration::from_secs(30);
        while Instant::now() < end {
            self.bridge
                .send(&self.ctx, &IgusCriTxChannels::JOINT_COMMAND, &msg)?;
            self.poll()?;
            match self.last_motion.status {
                CuCriMotionStatus::Reached => {
                    println!("Reached in {:.1} s", started.elapsed().as_secs_f64());
                    return Ok(true);
                }
                CuCriMotionStatus::Rejected | CuCriMotionStatus::Aborted => {
                    println!("Stopping: move ended as {:?}", self.last_motion.status);
                    return Ok(false);
                }
                _ => {}
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        println!("Stopping: move not reached within 30 s");
        Ok(false)
    }
}

fn main() -> CuResult<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let address = args
        .iter()
        .find(|a| !a.starts_with("--"))
        .cloned()
        .unwrap_or_else(|| "192.168.3.11:3920".into());
    let params = CuIgusCriParams {
        address,
        allow_control: true,
        allow_motion: true,
        velocity_percent: 5.0,
        ..Default::default()
    };
    println!(
        "Plan: A6 +{STEP_DEG}° and back at {}% velocity, goal tolerance {:.1}°.",
        params.velocity_percent,
        params.goal_tolerance.get::<degree>()
    );
    if !args.iter().any(|a| a == "--allow-motion") {
        println!("Dry run: pass --allow-motion to move the arm.");
        return Ok(());
    }
    println!("WARNING: the arm will move. Keep the e-stop in hand and the workspace clear.");

    let mut run = Run {
        bridge: CuIgusCriBridge::from_params(params)?,
        ctx: CuContext::from_clock(RobotClock::default()),
        status: CuMsg::default(),
        state: CuMsg::default(),
        motion: CuMsg::default(),
        session: CuSessionState::default(),
        last_motion: CuCriMotionState::default(),
        joints: None,
    };
    run.bridge.start(&run.ctx)?;
    run.wait(2.0, |r| r.joints.is_some())?;

    let ok = run.session_command(CuSessionCommand::AcquireControl)?
        && run.session_command(CuSessionCommand::ClearFaults)?
        && run.session_command(CuSessionCommand::EnableDrives)?;
    if ok {
        println!("Waiting for drives ready (15 s limit)");
        if run.wait(15.0, |r| r.session.drives_ready && r.session.motion_allowed)? {
            run.wait(0.3, |_| false)?; // fresh joints after enabling
            if let Some(start) = run.joints {
                let start_deg = start.map(|a| a.get::<degree>());
                println!("Start joints {start_deg:.2?}");
                let mut target = start;
                target[5] = Angle::new::<degree>(start_deg[5] + STEP_DEG);
                println!("Motion starts in 5 s. Press the e-stop to abort.");
                run.wait(5.0, |_| false)?;
                if run.move_to(target)? {
                    run.wait(1.0, |_| false)?;
                    run.move_to(start)?;
                }
            }
        } else {
            println!("Drives not ready after 15 s");
        }
        run.session_command(CuSessionCommand::DisableDrives)?;
    }
    if run.session.control_acquired {
        run.session_command(CuSessionCommand::ReleaseControl)?;
    }
    run.wait(1.0, |_| false)?;
    // `stop` also winds down anything still held (drives, then control).
    run.bridge.stop(&run.ctx)
}
