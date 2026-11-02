//! Exercise the robot session on a live ReBeL through the Repose contract.
//!
//! ```sh
//! session [ADDRESS]            # observe only, send no commands
//! session [ADDRESS] --activate # AcquireControl, ClearFaults, ReleaseControl
//! session [ADDRESS] --enable   # AcquireControl, ClearFaults, EnableDrives,
//!                              # wait for drives ready, hold 2 s,
//!                              # DisableDrives, ReleaseControl
//! ```
//!
//! No mode commands motion. `--enable` energises the drives: keep the e-stop
//! within reach.
use cu29::prelude::*;
use repose_igus_cri::CuCriStatus;
use repose_igus_cri::CuIgusCriBridge;
use repose_igus_cri::CuIgusCriParams;
use repose_igus_cri::IgusCriRxChannels;
use repose_igus_cri::IgusCriTxChannels;
use repose_robot_interface::CuSessionCommand;
use repose_robot_interface::CuSessionResult;
use repose_robot_interface::CuSessionState;
use std::time::Duration;
use std::time::Instant;

struct Session {
    bridge: CuIgusCriBridge,
    ctx: CuContext,
    status: CuMsg<CuCriStatus>,
    state: CuMsg<CuSessionState>,
    latest: CuSessionState,
    last_print: Instant,
}

impl Session {
    /// Poll both channels for up to `seconds`, returning early once `done`.
    fn observe(&mut self, seconds: f64, done: impl Fn(&CuSessionState) -> bool) -> CuResult<bool> {
        let end = Instant::now() + Duration::from_secs_f64(seconds);
        while Instant::now() < end {
            self.bridge
                .receive(&self.ctx, &IgusCriRxChannels::CRI_STATUS, &mut self.status)?;
            if let Some(s) = self.status.payload()
                && self.last_print.elapsed() >= Duration::from_millis(500)
            {
                println!(
                    "  cri: error={} joints={:?} kinstate={:?} estop={:?} override={:?}",
                    s.error_summary.as_str(),
                    s.joint_errors,
                    s.kinstate,
                    s.estop,
                    s.override_percent
                );
                self.last_print = Instant::now();
            }
            self.bridge.receive(
                &self.ctx,
                &IgusCriRxChannels::SESSION_STATE,
                &mut self.state,
            )?;
            if let Some(s) = self.state.payload() {
                println!(
                    "  session: {:?} #{} → {:?} {:?} {} | control={} enabled={} ready={} motion_allowed={} refused={}",
                    s.command,
                    s.sequence,
                    s.result,
                    s.rejection,
                    s.detail.as_str(),
                    s.control_acquired,
                    s.drives_enabled,
                    s.drives_ready,
                    s.motion_allowed,
                    s.refused
                );
                self.latest = s.clone();
            }
            if done(&self.latest) {
                return Ok(true);
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        Ok(false)
    }

    /// Send one command; true once acknowledged within 3 s.
    fn command(&mut self, command: CuSessionCommand) -> CuResult<bool> {
        println!("Sending {command:?}");
        let mut msg = CuMsg::<CuSessionCommand>::default();
        msg.set_payload(command);
        self.bridge
            .send(&self.ctx, &IgusCriTxChannels::SESSION, &msg)?;
        let finished = |s: &CuSessionState| {
            s.command == Some(command) && !matches!(s.result, CuSessionResult::Pending)
        };
        self.observe(3.0, finished)?;
        let ok = finished(&self.latest) && self.latest.result == CuSessionResult::Acknowledged;
        if !ok {
            println!("Stopping: {command:?} ended as {:?}", self.latest.result);
        }
        Ok(ok)
    }
}

fn main() -> CuResult<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let flag = |name: &str| args.iter().any(|a| a == name);
    let address = args
        .iter()
        .find(|a| !a.starts_with("--"))
        .cloned()
        .unwrap_or_else(|| "192.168.3.11:3920".into());
    let enable = flag("--enable");
    let activate = enable || flag("--activate");

    let mut session = Session {
        bridge: CuIgusCriBridge::from_params(CuIgusCriParams {
            address,
            allow_control: activate,
            ..Default::default()
        })?,
        ctx: CuContext::from_clock(RobotClock::default()),
        status: CuMsg::default(),
        state: CuMsg::default(),
        latest: CuSessionState::default(),
        last_print: Instant::now() - Duration::from_secs(1),
    };
    session.bridge.start(&session.ctx)?;
    println!("Observing");
    session.observe(2.0, |_| false)?;

    if activate
        && session.command(CuSessionCommand::AcquireControl)?
        && session.command(CuSessionCommand::ClearFaults)?
        && enable
        && session.command(CuSessionCommand::EnableDrives)?
    {
        // Drives come up one axis at a time after the acknowledgement.
        println!("Waiting for drives ready (15 s limit)");
        let started = Instant::now();
        if session.observe(15.0, |s| s.drives_ready && s.motion_allowed)? {
            println!(
                "Drives ready after {:.1} s; holding 2 s",
                started.elapsed().as_secs_f64()
            );
            session.observe(2.0, |_| false)?;
        } else {
            println!("Drives not ready after 15 s");
        }
        session.command(CuSessionCommand::DisableDrives)?;
    }
    if activate && session.latest.control_acquired {
        session.command(CuSessionCommand::ReleaseControl)?;
    }
    session.observe(1.0, |_| false)?;
    // `stop` also winds down anything still held (drives, then control).
    session.bridge.stop(&session.ctx)
}
