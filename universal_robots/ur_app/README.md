# UR robot program

Run from the workspace root with `cargo run -p ur-app`. This executes
`src/program.rs` against URSim: first three joints +5°, wait, then return.
The example connects to localhost RTDE 30004 and Dashboard 29999. Start URSim
with the bridge's compose file, complete its initial setup, enable and select
Remote Control. The program acquires control, clears healthy fault state,
powers on/releases brakes, streams the program, powers off and releases control.

The `Step` API is exactly the robot-app template's API:

```rust
pub fn steps() -> Vec<Step> {
    vec![
        JRelative([5.0, 5.0, 5.0, 0.0, 0.0, 0.0]),
        Wait(1.0),
        JRelative([-5.0, -5.0, -5.0, 0.0, 0.0, 0.0]),
    ]
}
```

`JAbsolut`, `JRelative`, `TCPRelative`, `JPlanned` and `Wait` have the same names, units and
implementation: the app compiles the template's **same runner and planner source
files**, with no UR-specific program logic. Copy `program.rs` between programs;
select the bridge/address and the appropriate kinematic/collision models in RON.
The generated template includes both bridge dependencies.

The example's inline UR5e nominal chain comes from
[Universal Robots' description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/rolling/config/ur5e/default_kinematics.yaml).
Its capsule collision geometry is an approximate example, not a validated
hardware collision model. It omits environment and tool geometry. Use a model
and calibration appropriate to the actual robot before running TCPRelative/JPlanned on
hardware; other UR models need their own configuration. This example targets
UR5e URSim, not all UR robots.
