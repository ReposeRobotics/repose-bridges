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

`JAbsolut`, `JRelative`, `TCPRelative`, `TCPAbsolut`, `JPlanned` and `Wait` have the same names, units and
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

`TCPRelative([x, y, z])` moves the TCP by a base-frame displacement in metres,
keeping orientation fixed. `TCPAbsolut([x, y, z, roll, pitch, yaw])` targets a
base-frame pose in metres and degrees. Both use Repose's Cartesian planning
and timing task: straight translation, shortest-arc rotation, and full-path
preflight checks. A blocked or unreachable line stops the program; it does
not fall back to a joint move. Fixed-capacity paths can also fail explicitly.

Optional `scene` and `exclusions` in the runner's RON config are shared by
joint planning and Cartesian motion. Cartesian results must match that scene
revision and exclusions before following. These are discrete collision checks,
not continuous swept-volume certification. Hardware behavior of this new TCP
execution path has not been verified. Planner scratch requires an 8 MiB thread
stack (the examples run on the main thread).
