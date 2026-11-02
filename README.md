# Repose Bridges

## Start a robot program

```sh
cargo install cargo-generate
cargo generate gh:ReposeRobotics/repose-bridges templates/robot-app --name my_robot
cd my_robot
cargo run          # the arm MOVES: keep the e-stop in hand
```

Write your program in `src/program.rs` as a list of steps: absolute joint
moves, joint offsets, tool moves in metres (solved with Repose IK), and waits.
Pick the robot (bridge, address, speed) and its kinematic model in
`copperconfig.ron`. Ctrl-C stops cleanly: drives off, control released.

Copper-native hardware bridges, grouped by manufacturer. Every bridge
implements the vendor-neutral Repose robot contract from
`repose-robot-interface`, so applications swap robots (or the MuJoCo simulator)
by changing one bridge entry in their RON config:

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
| igus | [repose-igus-cri](igus/repose_igus_cri) | ReBeL, six axes | CRI TCP: joint state, session control (no motion yet) |

Requires Rust 1.95 or newer and Copper 1.3.1. Host transports use `std`.
New manufacturers get their own top-level directory (for example `abb/`).
