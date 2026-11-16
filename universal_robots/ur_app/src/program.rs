//! Same Step API and runner as the igus robot-app template.
use crate::runner::Step;
use crate::runner::Step::*;
pub fn steps<const N: usize>() -> Vec<Step<N>> {
    // First three joints out and back; wrist joints stay put.
    let out: [f64; N] = core::array::from_fn(|i| if i < 3 { 5.0 } else { 0.0 });
    vec![JRelative(out), Wait(1.0), JRelative(out.map(|d| -d))]
}
