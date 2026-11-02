use cu29::prelude::*;
use repose_ur::{CuUrBridge, CuUrParams, CuUrStatus, UrRxChannels};
fn main() -> CuResult<()> {
    let address = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "127.0.0.1:30004".into());
    let mut bridge = CuUrBridge::from_params(CuUrParams {
        address,
        ..Default::default()
    })?;
    let (ctx, _) = CuContext::new_mock_clock();
    bridge.start(&ctx)?;
    for _ in 0..100 {
        let mut msg = CuMsg::<CuUrStatus>::default();
        bridge.receive(&ctx, &UrRxChannels::UR_STATUS, &mut msg)?;
        if let Some(status) = msg.payload() {
            println!("{status:?}");
        }
        std::thread::sleep(std::time::Duration::from_millis(100));
    }
    bridge.stop(&ctx)
}
