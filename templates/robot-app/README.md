# {{project-name}}

Your steps are in `src/program.rs`: `Joints`, `Offset`, `Tool`, `Plan` and
`Wait` use the same API on the igus ReBeL and the UR5e.

`cargo generate` asked which robot to use, so this project depends only on
that robot's bridge. Its whole Copper graph is in `copperconfig.ron`: the
bridge address, planner, `speed`, per-joint PID gains (`joint_gains`) and the
kinematic and collision models. Edit it there, then `cargo run`. To switch
robots, generate a new project and copy `src/program.rs` across.

The default program offsets all joints by 5° and back, then waits. Absolute
targets and tool moves must suit the robot and its starting pose.

`Plan` uses `repose::planners::CuAnyPlanner` and `CuPlannerChoice` from Repose
v0.7.1. Set `"planner"` in `copperconfig.ron` to `rrt_connect` (default),
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

Both robot configurations request control and motion. Running the program
moves the robot. Ctrl-C stops the app and winds down the bridge's session.
