# Repose Bridges

## Start a robot program

```sh
cargo install cargo-generate
cargo generate gh:ReposeRobotics/repose-bridges templates/robot-app --name my_robot
cd my_robot
cargo run          # the robot (igus_rebel or ur5e) is asked by cargo generate
```

Write your program in `src/program.rs` as a list of steps: absolute joint
moves, joint offsets, tool moves in metres (solved with Repose IK),
collision-free planned moves (`Plan`, Repose RRT-Connect), and waits. Every
move is timed into a smooth trajectory (Repose trajectory timing) and streamed
to the robot; set `speed` and `acceleration` in `copperconfig.ron`. Before
anything moves, every step is solved and checked for joint limits and
self-collision; the program refuses to start if one fails.
`cargo generate` asks for the robot and its address, then includes only that
robot's bridge crate and writes its Copper graph to `copperconfig.ron`
(bridge, address, speed, PID gains, model, planner). Both robots run the same
`src/program.rs`. Ctrl-C stops cleanly: drives off, control released.

Copper-native hardware bridges, grouped by manufacturer. Bridges use the
vendor-neutral Repose robot contract from `repose-robot-interface`, with the
following standard channels. Check each bridge's supported command modes before
swapping it into an application:

| Dir | Channel | Payload | Required |
|---|---|---|---|
| Rx | `joint_state` | `repose_kinematics::CuJointStatePayload<N>` | always |
| Tx | `joint_command` | `repose_controller::CuJointCommandPayload<N>` | bridges that move the robot |
| Tx | `session` | `repose_robot_interface::CuSessionCommand` | bridges with a control session |
| Rx | `session_state` | `repose_robot_interface::CuSessionState` | bridges with a control session |

Vendor extras use vendor-prefixed channel names (`cri_status`). Within a crate,
keep the module split `params` / `protocol` (pure, no I/O) / `mapping` /
`transport` / `bridge`, and reuse `CuSessionTracker` for session rules.

| Manufacturer | Crate | Device | Interface |
|---|---|---|---|
| Universal Robots | [repose-ur](universal_robots/repose_ur) | e-Series / URSim | RTDE feedback, opt-in URScript joint moves (unverified on hardware) |
| ABB | [repose-abb](abb/repose_abb) | OmniCore, six axes | EGM UDP: joint feedback, opt-in streamed joint corrections (unverified on hardware) |
| igus | [repose-igus-cri](igus/repose_igus_cri) | ReBeL, six axes | CRI TCP: joint state, session control (no motion yet) |

Requires Rust 1.95 or newer and Copper 1.3.1. Host transports use `std`.
New manufacturers get their own top-level directory (for example `abb/`).
