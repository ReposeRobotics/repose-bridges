//! Same Step API and runner as the igus robot-app template.
use crate::runner::Step;
use crate::runner::Step::*;
pub fn steps() -> Vec<Step> {
    vec![
        Offset([5.0, 5.0, 5.0, 0.0, 0.0, 0.0]),
        Wait(1.0),
        Offset([-5.0, -5.0, -5.0, 0.0, 0.0, 0.0]),
    ]
}
