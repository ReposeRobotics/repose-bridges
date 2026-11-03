//! Explicitly requested relative wrist move streamed over EGM; no default motion.
//!
//! Usage: abb_move --move-wrist-radians <delta> <robot-ip> [bind-address]
//! Motors must be on and the ReposeEgm RAPID program running.
use cu29::prelude::*;
use cu29::units::si::{
    angle::radian,
    angular_velocity::radian_per_second,
    f64::{Angle, AngularVelocity},
};
use repose_abb::{AbbRxChannels, AbbTxChannels, CuAbbBridge, CuAbbParams};
use repose_controller::{CuControllerMode, CuJointCommandPayload};
use repose_kinematics::{CuJointPosition, CuJointStatePayload, CuJointVelocity};
use repose_robot_interface::{CuSessionCommand, CuSessionResult, CuSessionState};
use std::time::{Duration, Instant};

const SPEED: f64 = 0.1; // rad/s

fn session(bridge: &mut CuAbbBridge, ctx: &CuContext, command: CuSessionCommand) -> CuResult<()> {
    let mut msg = CuMsg::default();
    msg.set_payload(command);
    bridge.send(ctx, &AbbTxChannels::SESSION, &msg)?;
    let deadline = Instant::now() + Duration::from_secs(12);
    loop {
        let mut state = CuMsg::<CuSessionState>::default();
        bridge.receive(ctx, &AbbRxChannels::SESSION_STATE, &mut state)?;
        if let Some(state) = state.payload().filter(|s| s.command == Some(command)) {
            match state.result {
                CuSessionResult::Acknowledged => return Ok(()),
                CuSessionResult::Rejected => {
                    return Err(CuError::from(format!(
                        "{command:?} rejected: {}",
                        state.detail.as_str()
                    )));
                }
                _ => {}
            }
        }
        if Instant::now() > deadline {
            return Err(CuError::from(format!("{command:?} timed out")));
        }
        std::thread::sleep(Duration::from_millis(5));
    }
}

fn wrist(bridge: &mut CuAbbBridge, ctx: &CuContext) -> CuResult<CuJointStatePayload<6>> {
    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        let mut msg = CuMsg::<CuJointStatePayload<6>>::default();
        bridge.receive(ctx, &AbbRxChannels::JOINT_STATE, &mut msg)?;
        if let Some(state) = msg.payload() {
            return Ok(state.clone());
        }
        if Instant::now() > deadline {
            return Err(CuError::from("no EGM feedback"));
        }
        std::thread::sleep(Duration::from_millis(2));
    }
}

fn main() -> CuResult<()> {
    let args: Vec<_> = std::env::args().collect();
    if !(4..=5).contains(&args.len()) || args[1] != "--move-wrist-radians" {
        return Err(CuError::from(
            "usage: abb_move --move-wrist-radians <delta> <robot-ip> [bind-address]",
        ));
    }
    let delta: f64 = args[2]
        .parse()
        .map_err(|e| CuError::new_with_cause("invalid angle", e))?;
    if !delta.is_finite() || delta.abs() > 0.2 {
        return Err(CuError::from("delta must be finite and <= 0.2 radians"));
    }
    let mut bridge = CuAbbBridge::from_params(CuAbbParams {
        robot_ip: args[3].clone(),
        bind: args.get(4).cloned().unwrap_or(CuAbbParams::default().bind),
        allow_control: true,
        allow_motion: true,
        ..Default::default()
    })?;
    let (ctx, _) = CuContext::new_mock_clock();
    bridge.start(&ctx)?;
    let initial = wrist(&mut bridge, &ctx)?;
    session(&mut bridge, &ctx, CuSessionCommand::AcquireControl)?;
    session(&mut bridge, &ctx, CuSessionCommand::EnableDrives)?;
    let CuJointPosition::Revolute(start) = initial.positions[5] else {
        return Err(CuError::from("expected revolute wrist"));
    };
    let start = start.get::<radian>();
    let duration = delta.abs() / SPEED;
    let begun = Instant::now();
    let result = (|| loop {
        let t = begun.elapsed().as_secs_f64();
        let fraction = (t / duration).min(1.0);
        let mut command = CuJointCommandPayload::<6> {
            mode: CuControllerMode::Velocity,
            positions: initial.positions,
            ..Default::default()
        };
        command.positions[5] =
            CuJointPosition::Revolute(Angle::new::<radian>(start + delta * fraction));
        let speed = if fraction < 1.0 {
            SPEED * delta.signum()
        } else {
            0.0
        };
        command.velocities =
            [CuJointVelocity::Revolute(AngularVelocity::new::<radian_per_second>(0.0)); 6];
        command.velocities[5] =
            CuJointVelocity::Revolute(AngularVelocity::new::<radian_per_second>(speed));
        let mut msg = CuMsg::default();
        msg.set_payload(command);
        bridge.send(&ctx, &AbbTxChannels::JOINT_COMMAND, &msg)?;
        let state = wrist(&mut bridge, &ctx)?;
        let CuJointPosition::Revolute(actual) = state.positions[5] else {
            unreachable!()
        };
        let error = actual.get::<radian>() - start - delta;
        if fraction >= 1.0 && error.abs() <= 0.002 {
            println!(
                "Reached wrist offset: {} rad",
                actual.get::<radian>() - start
            );
            return Ok(());
        }
        if t > duration + 5.0 {
            return Err(CuError::from("move did not reach target"));
        }
        std::thread::sleep(Duration::from_millis(4));
    })();
    session(&mut bridge, &ctx, CuSessionCommand::DisableDrives)?;
    session(&mut bridge, &ctx, CuSessionCommand::ReleaseControl)?;
    bridge.stop(&ctx)?;
    result
}
