//! Your robot program. Steps run in order; each motion step finishes when the
//! robot reaches it. Afterwards the drives are disabled and control released.
//!
//! - `Joints([..])`     absolute joint angles, degrees (A1..A6)
//! - `Offset([..])`     joint offsets from the previous target, degrees
//! - `Tool([x, y, z])`  move the tool by x/y/z metres in the base frame,
//!   keeping its orientation (solved with Repose IK)
//! - `Plan([..])`       absolute joint angles, degrees, reached along a
//!   collision-free path from the planner (see copperconfig.ron)
//! - `Wait(s)`          hold for `s` seconds
//!
//! Before anything moves, every step is solved and collision-checked; the
//! program refuses to start if one fails.

use crate::runner::Step;
use crate::runner::Step::*;

pub fn steps() -> Vec<Step> {
    vec![
        // all joints at once, out and back
        Offset([5.0, 5.0, 5.0, 5.0, 5.0, 5.0]),
        Offset([-5.0, -5.0, -5.0, -5.0, -5.0, -5.0]),
        Wait(1.0),
        // bend the arm (collision-free path), then tool 3 cm up and back via IK
        Plan([0.0, 30.0, 60.0, 0.0, 60.0, 0.0]),
        Tool([0.0, 0.0, 0.03]),
        Tool([0.0, 0.0, -0.03]),
    ]
}
