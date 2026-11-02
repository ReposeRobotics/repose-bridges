//! Observe the six measured joint angles for ten seconds.
use cu29::prelude::*;
use cu29::units::si::angle::degree;
use repose_igus_cri::CuIgusCriBridge;
use repose_igus_cri::CuIgusCriParams;
use repose_igus_cri::IgusCriRxChannels;
use repose_kinematics::CuJointPosition;
use repose_kinematics::CuJointStatePayload;

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
    bridge.start(&ctx)?;
    let mut msg = CuMsg::<CuJointStatePayload<6>>::default();
    for _ in 0..1000 {
        bridge.receive(&ctx, &IgusCriRxChannels::JOINT_STATE, &mut msg)?;
        if let Some(state) = msg.payload() {
            let degrees = state.positions.map(|p| match p {
                CuJointPosition::Revolute(angle) => angle.get::<degree>(),
                _ => f64::NAN,
            });
            println!("A1…A6 (°): {degrees:.3?}  status: {:?}", state.status);
        }
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
    bridge.stop(&ctx)
}
