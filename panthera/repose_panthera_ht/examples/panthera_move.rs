use cu29::prelude::*;
use cu29::units::si::{angle::radian, f64::Angle};
use repose_controller::CuJointCommandPayload;
use repose_kinematics::CuJointPosition;
use repose_panthera_ht::{CuPantheraHtBridge, CuPantheraHtParams, PantheraTxChannels};
fn main() -> CuResult<()> {
    let serial_port = std::env::args().nth(1).unwrap_or_default();
    let mut bridge = CuPantheraHtBridge::from_params(CuPantheraHtParams {
        serial_port,
        allow_control: true,
        allow_motion: true,
        ..Default::default()
    })?;
    let (ctx, _) = CuContext::new_mock_clock();
    bridge.start(&ctx)?;
    std::thread::sleep(std::time::Duration::from_secs(1));
    let mut msg = CuMsg::default();
    msg.set_payload(CuJointCommandPayload::<6> {
        positions: [CuJointPosition::Revolute(Angle::new::<radian>(0.1)); 6],
        ..Default::default()
    });
    bridge.send(&ctx, &PantheraTxChannels::JOINT_COMMAND, &msg)?;
    println!("sent position command");
    std::thread::sleep(std::time::Duration::from_secs(2));
    bridge.stop(&ctx)
}
