//! Compute the ReBeL base-to-link6 pose from live measured joints for ten seconds.
use cu_spatial_payloads::Transform3D;
use cu29::prelude::*;
use cu29::units::si::length::meter;
use repose_igus_cri::CuIgusCriBridge;
use repose_igus_cri::CuIgusCriParams;
use repose_igus_cri::IgusCriRxChannels;
use repose_kinematics::CuJointStatePayload;
use repose_kinematics::igus_rebel_chain;

fn main() -> CuResult<()> {
    let params = CuIgusCriParams {
        address: std::env::args()
            .nth(1)
            .unwrap_or_else(|| "192.168.3.11:3920".into()),
        ..Default::default()
    };
    let mut bridge = CuIgusCriBridge::from_params(params)?;
    let clock = RobotClock::default();
    let ctx = CuContext::from_clock(clock);
    let chain = igus_rebel_chain();
    let mut pose = Transform3D::<f64>::identity();
    bridge.start(&ctx)?;
    let mut msg = CuMsg::<CuJointStatePayload<6>>::default();
    for _ in 0..1000 {
        bridge.receive(&ctx, &IgusCriRxChannels::JOINT_STATE, &mut msg)?;
        if let Some(state) = msg.payload() {
            // Controller A1…A6 correspond to preset joints 1…6.
            chain.forward_kinematics(&state.positions, &mut pose)?;
            let xyz = pose.translation().map(|value| value.get::<meter>());
            println!("Base → link6 xyz (m): {xyz:.4?} R: {:.4?}", pose.rotation());
        }
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
    bridge.stop(&ctx)
}
