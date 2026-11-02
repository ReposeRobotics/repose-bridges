# Repose Bridges

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
