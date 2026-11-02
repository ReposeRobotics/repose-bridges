//! Minimal Copper program for an igus ReBeL: the graph is `copperconfig.ron`,
//! the behaviour is the `sequence` task. Runs until the sequence finishes or
//! Ctrl-C, then stops the app so the bridge disables the drives and releases
//! control.
//!
//! The arm MOVES: keep the e-stop in hand.

mod sequence;

use cu29::prelude::*;
use std::sync::atomic::Ordering;
use std::time::Duration;
use std::time::Instant;

/// `run_one_iteration` does not pace itself; run the graph at 50 Hz.
const PERIOD: Duration = Duration::from_millis(20);

#[copper_runtime(config = "copperconfig.ron")]
struct RebelApp {}

fn main() -> CuResult<()> {
    ctrlc::set_handler(|| sequence::DONE.store(true, Ordering::Release))
        .map_err(|e| CuError::new_with_cause("Ctrl-C handler", e))?;
    let log = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("rebel_app.copper");
    let mut app = RebelApp::builder()
        .with_log_path(log, None)?
        .build()?
        .start()
        .map_err(|fault| fault.error)?;
    while !sequence::DONE.load(Ordering::Acquire) {
        let started = Instant::now();
        app.run_one_iteration()?;
        std::thread::sleep(PERIOD.saturating_sub(started.elapsed()));
    }
    app.stop().map_err(|fault| fault.error)?;
    Ok(())
}
