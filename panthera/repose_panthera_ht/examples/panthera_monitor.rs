use cu29::prelude::*;
use repose_panthera_ht::{
    CuPantheraHtBridge, CuPantheraHtParams, CuPantheraStatus, PantheraRxChannels,
};
fn main() -> CuResult<()> {
    let serial_port = std::env::args().nth(1).unwrap_or_default();
    let mut bridge = CuPantheraHtBridge::from_params(CuPantheraHtParams {
        serial_port,
        ..Default::default()
    })?;
    let (ctx, _) = CuContext::new_mock_clock();
    bridge.start(&ctx)?;
    for _ in 0..100 {
        let mut msg = CuMsg::<CuPantheraStatus>::default();
        bridge.receive(&ctx, &PantheraRxChannels::PANTHERA_STATUS, &mut msg)?;
        if let Some(status) = msg.payload() {
            println!("{status:?}");
        }
        std::thread::sleep(std::time::Duration::from_millis(100));
    }
    bridge.stop(&ctx)
}
