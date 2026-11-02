//! Streaming velocity control: A6 at +3 °/s for 2 s, then −3 °/s for 2 s,
//! commanded every 20 ms as a Copper controller would (velocity mode).
//!
//! ```sh
//! jog [ADDRESS] --allow-motion   # the arm MOVES
//! ```
use cu29::prelude::*;
use cu29::units::si::angle::degree;
use cu29::units::si::angular_velocity::degree_per_second;
use cu29::units::si::f64::Angle;
use cu29::units::si::f64::AngularVelocity;
use repose_controller::CuControllerMode;
use repose_controller::CuJointCommandPayload;
use repose_igus_cri::CuCriStatus;
use repose_igus_cri::CuIgusCriBridge;
use repose_igus_cri::CuIgusCriParams;
use repose_igus_cri::IgusCriRxChannels;
use repose_igus_cri::IgusCriTxChannels;
use repose_kinematics::CuJointPosition;
use repose_kinematics::CuJointVelocity;
use repose_robot_interface::CuSessionCommand;
use repose_robot_interface::CuSessionResult;
use repose_robot_interface::CuSessionState;
use std::time::Duration;
use std::time::Instant;

const SPEED_DPS: f64 = 3.0;
const LEG_S: f64 = 2.0;
/// Hold the final position (zero feed-forward) this long so feedback settles it.
const HOLD_S: f64 = 1.0;
const PERIOD: Duration = Duration::from_millis(20);

struct Io {
    bridge: CuIgusCriBridge,
    ctx: CuContext,
    status: CuMsg<CuCriStatus>,
    state_msg: CuMsg<CuSessionState>,
    session: CuSessionState,
    joints: Option<[Angle; 6]>,
}

impl Io {
    fn poll(&mut self) -> CuResult<()> {
        self.bridge
            .receive(&self.ctx, &IgusCriRxChannels::CRI_STATUS, &mut self.status)?;
        if let Some(s) = self.status.payload() {
            self.joints = Some(s.joints);
        }
        self.bridge.receive(
            &self.ctx,
            &IgusCriRxChannels::SESSION_STATE,
            &mut self.state_msg,
        )?;
        if let Some(s) = self.state_msg.payload() {
            self.session = s.clone();
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

    fn session(&mut self, command: CuSessionCommand) -> CuResult<bool> {
        let mut msg = CuMsg::<CuSessionCommand>::default();
        msg.set_payload(command);
        self.bridge
            .send(&self.ctx, &IgusCriTxChannels::SESSION, &msg)?;
        self.wait(3.0, |io| {
            io.session.command == Some(command) && io.session.result != CuSessionResult::Pending
        })?;
        let ok = self.session.result == CuSessionResult::Acknowledged;
        println!("{command:?}: {:?}", self.session.result);
        Ok(ok)
    }
}

fn main() -> CuResult<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let address = args
        .iter()
        .find(|a| !a.starts_with("--"))
        .cloned()
        .unwrap_or_else(|| "192.168.3.11:3920".into());
    println!("Plan: A6 +{SPEED_DPS} °/s for {LEG_S} s, then back, streamed at 50 Hz.");
    if !args.iter().any(|a| a == "--allow-motion") {
        println!("Dry run: pass --allow-motion to move the arm.");
        return Ok(());
    }
    let mut io = Io {
        bridge: CuIgusCriBridge::from_params(CuIgusCriParams {
            address,
            allow_control: true,
            allow_motion: true,
            ..Default::default()
        })?,
        ctx: CuContext::from_clock(RobotClock::default()),
        status: CuMsg::default(),
        state_msg: CuMsg::default(),
        session: CuSessionState::default(),
        joints: None,
    };
    io.bridge.start(&io.ctx)?;
    io.wait(2.0, |io| io.joints.is_some())?;
    if io.session(CuSessionCommand::AcquireControl)?
        && io.session(CuSessionCommand::ClearFaults)?
        && io.session(CuSessionCommand::EnableDrives)?
        && io.wait(15.0, |io| io.session.motion_allowed)?
    {
        io.wait(0.3, |_| false)?;
        if let Some(start) = io.joints {
            let a6 = start[5].get::<degree>();
            println!("Start A6 {a6:.2}°. Motion in 3 s.");
            io.wait(3.0, |_| false)?;
            let t0 = Instant::now();
            let mut last_print = 0.0;
            loop {
                let t = t0.elapsed().as_secs_f64();
                if t >= 2.0 * LEG_S + HOLD_S {
                    break;
                }
                // Triangle: out for one leg, back for the next, then hold.
                let (v, p) = if t < LEG_S {
                    (SPEED_DPS, a6 + SPEED_DPS * t)
                } else if t < 2.0 * LEG_S {
                    (-SPEED_DPS, a6 + SPEED_DPS * (2.0 * LEG_S - t))
                } else {
                    (0.0, a6)
                };
                let mut positions = start.map(CuJointPosition::Revolute);
                positions[5] = CuJointPosition::Revolute(Angle::new::<degree>(p));
                let mut velocities = [CuJointVelocity::Revolute(AngularVelocity::default()); 6];
                velocities[5] =
                    CuJointVelocity::Revolute(AngularVelocity::new::<degree_per_second>(v));
                let mut msg = CuMsg::<CuJointCommandPayload<6>>::default();
                msg.set_payload(CuJointCommandPayload {
                    mode: CuControllerMode::Velocity,
                    positions,
                    velocities,
                    ..Default::default()
                });
                io.bridge
                    .send(&io.ctx, &IgusCriTxChannels::JOINT_COMMAND, &msg)?;
                io.poll()?;
                if t - last_print >= 0.5 {
                    if let Some(j) = io.joints {
                        let at = j[5].get::<degree>();
                        println!(
                            "t={t:.1}s target {p:.2}° measured {at:.2}° error {:+.2}°",
                            p - at
                        );
                    }
                    last_print = t;
                }
                std::thread::sleep(PERIOD);
            }
            io.wait(1.0, |_| false)?;
            if let Some(j) = io.joints {
                println!("End A6 {:.2}° (start {a6:.2}°)", j[5].get::<degree>());
            }
        }
        io.session(CuSessionCommand::DisableDrives)?;
    }
    if io.session.control_acquired {
        io.session(CuSessionCommand::ReleaseControl)?;
    }
    io.bridge.stop(&io.ctx)
}
