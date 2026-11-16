# {{project-name}}

Your steps are in `src/program.rs`: `JAbsolut`, `JRelative`, `TCPRelative`, `TCPAbsolut`, `JPlanned` and
`Wait` use the same API on the igus ReBeL, the UR5e and the simulated SO-ARM100.

`cargo generate` asked which robot to use, so this project depends only on
that robot's bridge. The runner is generic over the joint count: the profile
sets it with the `runner::RobotProgram<N>` task type and the `<N>` channel
payload types (6 for the igus ReBeL and the UR5e, 5 for the SO-ARM100), and `src/program.rs` is
written against `Step<N>` so the same program runs on any joint count.
Its whole Copper graph is in `copperconfig.ron`: the
bridge address, planner, `speed`, per-joint PID gains (`joint_gains`) and the
kinematic and collision models. Edit it there, then `cargo run`. To switch
robots, generate a new project and copy `src/program.rs` across.

The default program offsets all joints by 5° and back, then waits. Absolute
targets and tool moves must suit the robot and its starting pose.

`JPlanned` uses `repose::planners::CuAnyPlanner` and `CuPlannerChoice` from Repose
commit `e554cca` (pose and Cartesian planning). Set `"planner"` in `copperconfig.ron` to `rrt_connect` (default),
`rrt_star`, `prm`, `a_star`, `stomp`, `path_optimizer` or `direct`. The same runner
performs preflight checks and streams timed trajectories for both robots.

The SO-ARM100 profile (`so_arm`) runs a simulated 5-DOF arm, not hardware.
Start the MuJoCo sidecar first, then the app:

    python -m pip install mujoco numpy
    python sim/sidecar.py            # headless, listens on 127.0.0.1:55091
    python sim/sidecar.py --viewer   # or with a MuJoCo window (needs a display)
    cargo run

`src/sim.rs` (`sim::SimRobot<5>`) gives the runner the standard robot
channels over `repose_mujoco_sim::CuMujocoSimBridge<5>`: line-delimited JSON
over TCP, one joint command in and one state out per 20 ms cycle, positions in
radians. It sends a hold at the measured pose in cycles without a command, and
acknowledges every session command at once (a simulator has no drives);
joint-state timestamps are the Copper receive time. The bundled
`sim/so_arm100.urdf` and the chain in `copperconfig.ron` are a nominal
straight-arm model with approximate collision capsules, not a calibrated
SO-ARM100 or SO-101; the gripper is not simulated. With five joints a TCP pose
fixes more than the arm can reach, so `TCPRelative`/`TCPAbsolut` targets must
be reachable with the held orientation. This profile uses the newer Repose
commit `54e23b9` (joint-count-generic MuJoCo bridge); the hardware profiles
stay on `e554cca`.

For URSim, start the repository's `universal_robots/repose_ur/compose.yaml`,
complete initial PolyScope setup, and enable **and select** Remote Control.
The default UR address is localhost:30004; Dashboard uses port 29999. For a
remote simulator, run this app on the same host or forward RTDE, Dashboard
and secondary port 30002. The UR5e configuration contains a nominal chain and
approximate collision capsules for simulation. Real hardware needs matching
robot calibration, tool and collision geometry; other UR models need their own
model configuration.

The igus and UR configurations request control and motion. Running the program
moves the robot. Ctrl-C stops the app and winds down the bridge's session.

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
