//! Your robot program. Steps run in order; each motion step is timed into a
//! smooth trajectory (speed and acceleration in copperconfig.ron), streamed to
//! the robot, and finishes when the robot settles on its goal. Afterwards the
//! drives are disabled and control released.
//!
//! - `Joints([..])`     absolute joint angles, degrees (A1..A6)
//! - `Offset([..])`     joint offsets from the previous target, degrees
//! - `Tool([x, y, z])`  move the tool by x/y/z metres in the base frame,
//!   keeping its orientation (solved with Repose IK)
//! - `Plan([..])`       absolute joint angles, degrees, reached along a
//!   collision-free path from the planner (`"planner"` in copperconfig.ron:
//!   rrt_connect, rrt_star, prm, a_star, stomp, path_optimizer, direct)
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
        // Add robot-appropriate absolute targets with Joints([...]) or Plan([...]).
        // Tool([0.0, 0.0, 0.03]) moves 3 cm up using the selected model's IK.
        // Absolute targets and reachable tool moves depend on the robot/start pose.
    ]
}
