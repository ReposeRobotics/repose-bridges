//! Observe the six measured joint angles for ten seconds.
use cu29::prelude::*;
use cu29::units::si::angle::degree;
use repose_igus_cri::CuIgusCriBridge;
use repose_igus_cri::CuIgusCriParams;
use repose_igus_cri::CuRebelState;
use repose_igus_cri::IgusCriRxChannels;

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
    let mut msg = CuMsg::<CuRebelState>::default();
    for _ in 0..1000 {
        bridge.receive(&ctx, &IgusCriRxChannels::JOINT_STATE, &mut msg)?;
        if let Some(state) = msg.payload() {
            println!(
                "A1…A6 (°): {:?}",
                state.positions.map(|p| p.get::<degree>())
            );
        }
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
    bridge.stop(&ctx)
}
