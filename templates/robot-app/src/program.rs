//! Your robot program. Steps run in order; each motion step is timed into a
//! smooth trajectory (speed and acceleration in copperconfig.ron), streamed to
//! the robot, and finishes when the robot settles on its goal. Afterwards the
//! drives are disabled and control released.
//!
//! - `JAbsolut([..])`     absolute joint angles, degrees (A1..A6)
//! - `JRelative([..])`     joint offsets from the previous target, degrees
//! - `TCPRelative([x, y, z])`  move the tool by x/y/z metres in the base frame,
//!   keeping its orientation along a collision-checked straight TCP path
//! - `TCPAbsolut([x, y, z, roll, pitch, yaw])` absolute base-frame pose,
//!   metres and degrees; straight TCP translation and shortest-arc rotation
//! - `JPlanned([..])`       absolute joint angles, degrees, reached along a
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
        JRelative([5.0, 5.0, 5.0, 5.0, 5.0, 5.0]),
        JRelative([-5.0, -5.0, -5.0, -5.0, -5.0, -5.0]),
        Wait(1.0),
        // Add robot-appropriate absolute targets with JAbsolut([...]) or JPlanned([...]).
        // TCPRelative([0.0, 0.0, 0.03]) moves 3 cm straight up while holding orientation.
        // Absolute targets and reachable tool moves depend on the robot/start pose.
    ]
}
