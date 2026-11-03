//! Print EGM feedback. Usage: abb_monitor [robot-ip] [bind-address]
use cu29::prelude::*;
use repose_abb::{AbbRxChannels, CuAbbBridge, CuAbbParams, CuAbbStatus};
fn main() -> CuResult<()> {
    let mut args = std::env::args().skip(1);
    let defaults = CuAbbParams::default();
    let mut bridge = CuAbbBridge::from_params(CuAbbParams {
        robot_ip: args.next().unwrap_or(defaults.robot_ip.clone()),
        bind: args.next().unwrap_or(defaults.bind.clone()),
        ..defaults
    })?;
    let (ctx, _) = CuContext::new_mock_clock();
    bridge.start(&ctx)?;
    for _ in 0..100 {
        let mut msg = CuMsg::<CuAbbStatus>::default();
        bridge.receive(&ctx, &AbbRxChannels::ABB_STATUS, &mut msg)?;
        match msg.payload() {
            Some(status) => println!("{status:?}"),
            None => println!("no fresh EGM feedback"),
        }
        std::thread::sleep(std::time::Duration::from_millis(100));
    }
    bridge.stop(&ctx)
}
