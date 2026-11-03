//! Robot program: the robot was chosen at `cargo generate`; your steps are in
//! `src/program.rs`. Runs until the program finishes or Ctrl-C, then stops
//! the app so the bridge disables the drives and releases control.
//!
//! The arm MOVES: keep the e-stop in hand.

mod program;
mod runner;

use cu29::prelude::*;
use std::sync::atomic::Ordering;
use std::time::Duration;
use std::time::Instant;

#[copper_runtime(config = "copperconfig.ron")]
struct RobotApp {}

/// `run_one_iteration` does not pace itself; run the graph at 50 Hz.
const PERIOD: Duration = Duration::from_millis(20);

fn main() -> CuResult<()> {
    ctrlc::set_handler(|| runner::DONE.store(true, Ordering::Release))
        .map_err(|e| CuError::new_with_cause("Ctrl-C handler", e))?;
    let log = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("robot.copper");
    let mut app = RobotApp::builder()
        .with_log_path(log, None)?
        .build()?
        .start()
        .map_err(|fault| fault.error)?;
    while !runner::DONE.load(Ordering::Acquire) {
        let started = Instant::now();
        app.run_one_iteration()?;
        std::thread::sleep(PERIOD.saturating_sub(started.elapsed()));
    }
    app.stop().map_err(|fault| fault.error)?;
    Ok(())
}
