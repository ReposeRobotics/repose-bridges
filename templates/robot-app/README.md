# {{project-name}}

One program, two robot configurations. Keep your steps in `src/program.rs`:
`Joints`, `Offset`, `Tool`, `Plan` and `Wait` use the same API on either robot.

Choose the robot in `robot.ron`:

```ron
(robot: igus_rebel) // or: (robot: ur5e)
```

Then run `cargo run`. The build selects `robots/igus_rebel.ron` or
`robots/ur5e.ron` and generates `copperconfig.ron` for Copper. Change `robot.ron`
and run again to switch; no Cargo features or program changes are needed.
Edit each robot's profile for its address, planner, speed and model parameters.
`copperconfig.ron` is generated and ignored by Git; edit the source profiles.

The default program offsets all joints by 5° and back, then waits. Absolute
targets and tool moves must suit the selected robot and its starting pose.

`Plan` uses `repose::planners::CuAnyPlanner` and `CuPlannerChoice` from Repose
v0.7.1. Set `"planner"` in the selected robot profile to `rrt_connect` (default),
`rrt_star`, `prm`, `a_star`, `stomp`, `path_optimizer` or `direct`. The same runner
performs preflight checks and streams timed trajectories for both robots.

For URSim, start the repository's `universal_robots/repose_ur/compose.yaml`,
complete initial PolyScope setup, and enable **and select** Remote Control.
The default UR address is localhost:30004; Dashboard uses port 29999. For a
remote simulator, run this app on the same host or forward RTDE, Dashboard
and secondary port 30002. The UR5e configuration contains a nominal chain and
approximate collision capsules for simulation. Real hardware needs matching
robot calibration, tool and collision geometry; other UR models need their own
model configuration.

Both configurations request control and motion. Running the program moves the
selected robot. Ctrl-C stops the app and winds down the bridge's session.
