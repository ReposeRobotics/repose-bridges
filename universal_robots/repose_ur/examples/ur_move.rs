//! Explicitly requested relative wrist move; no default motion.
use cu29::prelude::*;
use cu29::units::si::{angle::radian, f64::Angle};
use repose_controller::CuJointCommandPayload;
use repose_kinematics::{CuJointPosition, CuJointStatePayload};
use repose_ur::{CuUrBridge, CuUrParams, UrRxChannels, UrTxChannels};
fn main() -> CuResult<()> {
    let args: Vec<_> = std::env::args().collect();
    if args.len() != 4 || args[1] != "--move-wrist-radians" {
        return Err(CuError::from(
            "usage: ur_move --move-wrist-radians <delta> <numeric-ip:rtde-port>",
        ));
    }
    let delta: f64 = args[2]
        .parse()
        .map_err(|e| CuError::new_with_cause("invalid angle", e))?;
    if !delta.is_finite() || delta.abs() > 0.2 {
        return Err(CuError::from("delta must be finite and <= 0.2 radians"));
    }
    let mut bridge = CuUrBridge::from_params(CuUrParams {
        address: args[3].clone(),
        allow_motion: true,
        allow_control: true,
        ..Default::default()
    })?;
    let (ctx, _) = CuContext::new_mock_clock();
    bridge.start(&ctx)?;
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(3);
    let initial = loop {
        let mut msg = CuMsg::<CuJointStatePayload<6>>::default();
        bridge.receive(&ctx, &UrRxChannels::JOINT_STATE, &mut msg)?;
        if let Some(state) = msg.payload() {
            break state.clone();
        }
        if std::time::Instant::now() > deadline {
            return Err(CuError::from("no UR feedback"));
        }
        std::thread::sleep(std::time::Duration::from_millis(10));
    };
    let mut command = CuJointCommandPayload {
        positions: initial.positions,
        ..Default::default()
    };
    let CuJointPosition::Revolute(wrist) = command.positions[5] else {
        return Err(CuError::from("expected revolute wrist"));
    };
    command.positions[5] = CuJointPosition::Revolute(wrist + Angle::new::<radian>(delta));
    let mut msg = CuMsg::default();
    msg.set_payload(command);
    bridge.send(&ctx, &UrTxChannels::JOINT_COMMAND, &msg)?;
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
    let started = std::time::Instant::now();
    loop {
        let mut msg = CuMsg::<CuJointStatePayload<6>>::default();
        bridge.receive(&ctx, &UrRxChannels::JOINT_STATE, &mut msg)?;
        if let Some(state) = msg.payload() {
            let CuJointPosition::Revolute(actual) = state.positions[5] else {
                unreachable!()
            };
            if started.elapsed().as_secs_f64() > 0.5
                && ((actual - wrist).get::<radian>() - delta).abs() <= 0.001
            {
                println!(
                    "Reached wrist offset: {} rad",
                    (actual - wrist).get::<radian>()
                );
                break;
            }
        }
        if std::time::Instant::now() > deadline {
            return Err(CuError::from("move did not reach target"));
        }
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
    bridge.stop(&ctx)
}
