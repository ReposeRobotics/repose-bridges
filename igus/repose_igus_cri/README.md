# igus ReBeL CRI bridge

`CuIgusCriBridge` connects Copper to an igus ReBeL over CRI (TCP) and speaks
the vendor-neutral Repose robot contract (`repose-robot-interface`). The same
channels and payloads are used by the MuJoCo simulator bridge and future vendor
bridges, so an application changes robots by changing one bridge entry.

Connect the host to the controller's Ethernet network. From the repo root:

```sh
cargo session                 # observe health and session state, no commands
cargo move --allow-motion     # A6 +5° and back (the arm moves)
cargo run -p rebel-app        # the minimal Copper program (igus/rebel_app)
```

## Channels

| Dir | Channel | Payload | Notes |
|---|---|---|---|
| Rx | `joint_state` | `repose_kinematics::CuJointStatePayload<6>` | standard; revolute A1–A6, velocities zero (CRI reports none) |
| Rx | `session_state` | `repose_robot_interface::CuSessionState` | standard; published on change |
| Rx | `cri_status` | `repose_igus_cri::CuCriStatus` | vendor: joints, TCP pose, mode, error codes, KINSTATE, ESTOP, override |
| Tx | `session` | `repose_robot_interface::CuSessionCommand` | standard; `control` feature only |
| Tx | `joint_command` | `repose_controller::CuJointCommandPayload<6>` | standard; `control` feature + `allow_motion`; `position` (Move Joint) or `velocity` (streamed jog) |
| Rx | `cri_motion` | `repose_igus_cri::CuCriMotionState` | vendor: move status, rejection reason, last `EXEC*` frame |

`joint_state.status` (`CuRobotStatus`) is `Ready` when every drive reports code
0 and `KINSTATE` is 0, `Fault(code)` for any per-joint code other than 0, 4 (not
enabled) or 4096 (enabling), and `NotReady` otherwise. CRI's `ESTOP` value is
reported raw on `cri_status` and not mapped: its encoding is undocumented.

## Module layout

| Module | Job |
|---|---|
| `params` | `CuIgusCriParams` and validation |
| `protocol` | pure CRI text decode/encode: STATUS, CMDACK/CMDERROR, `CMD Active`, command keywords |
| `mapping` | CRI → Repose contract (`CuJointStatePayload`, `CuRobotStatus`) |
| `transport` | TCP worker thread, heartbeat, frame assembly, shared latest-value slot |
| `bridge` | the `CuBridge`: channels, session gating, wind-down on stop |

Session rules (one command in flight, refuse while not in control, sequence
matching, wind-down order) live in `repose_robot_interface::CuSessionTracker`
and are shared by every bridge. This crate only maps them to CRI.

## Copper wiring

```ron
bridges: [
    (
        id: "rebel",
        type: "repose_igus_cri::CuIgusCriBridge",
        config: {
            "address": "192.168.3.11:3920",
            "stale_after": 0.5,
            "connect_timeout": 2.0,
            "allow_control": false,
        },
        channels: [Rx(id: "joint_state")],
    ),
],
cnx: [
    (src: "rebel/joint_state", dst: "fk",
     msg: "repose_kinematics::CuJointStatePayload<6>"),
],
```

The config map is `CuIgusCriParams`; times are SI seconds. Unknown keys are
rejected. Defaults match the snippet.

## Session control (`control` feature)

Off by default; needs the cargo feature `control` **and** `"allow_control": true`.
Then the `session` Tx channel takes `CuSessionCommand`:

| `CuSessionCommand` | CRI | Answered by |
|---|---|---|
| `acquire_control` | `CMD SetActive true` | `CMD Active true` |
| `release_control` | `CMD SetActive false` | `CMD Active false` |
| `clear_faults` | `CMD Reset` | `CMDACK` / `CMDERROR` |
| `enable_drives` | `CMD Enable` | `CMDACK` / `CMDERROR` |
| `disable_drives` | `CMD Disable` | `CMDACK` / `CMDERROR` |

**There is no motion command.** The heartbeat's jog values are zero in every
build. `enable_drives` energises the drives and releases the brakes while the
arm holds position. Keep the e-stop within reach.

- The controller ignores commands from a passive CRI connection. While
  `session_state.control_acquired` is false, every command except
  `acquire_control` is refused locally (`rejection: not_in_control`).
- One command is in flight at a time; extra requests are refused and counted
  in `refused`. Refusals are reported on `session_state`, never as errors.
- After the `Enable` acknowledgement the controller brings the axes up one at
  a time (per-joint code 4 → 4096 → 0, about 0.4–0.7 s each).
  `session_state.drives_ready` and `motion_allowed` follow the STATUS frames.
- `stop` and drop send `disable_drives` if this session enabled the drives,
  then `release_control` if it acquired control, waiting up to 500 ms each.
- A request without `allow_control`, or before `start`, returns an error.

```ron
config: { "address": "192.168.3.11:3920", "allow_control": true },
channels: [Rx(id: "joint_state"), Rx(id: "session_state"), Tx(id: "session")],
```

Verified on RobotControl V980-14 (ReBeL 6DOF-03): acquire, clear faults, enable
(all six axes `NoError`, `KINSTATE 0` after about 2.4 s), disable, and release
were acknowledged, and `CMDERROR <client counter> <text>` replies matched. CRI
`Connect`/`Disconnect` are not exposed: this firmware answers `not available`.

```sh
cargo run -p repose-igus-cri --features control --example session -- 192.168.3.11:3920 --activate
cargo run -p repose-igus-cri --features control --example session -- 192.168.3.11:3920 --enable
```

## Motion (stage 2a: point-to-point)

`joint_command` with `mode: position` sends one controller-planned
`CMD Move Joint <A1…A6 °> 0 0 0 <velocity %>`. Three opt-ins: the `control`
feature, `allow_control`, and `allow_motion` (which requires `allow_control`).
`velocity` and `effort` modes return an error (streaming jog is stage 2b).

A target is refused, reported on `cri_motion` and never sent, unless this
session holds control and the controller reports `motion_allowed`, the target
is within the ReBeL preset's joint limits, and every joint is within
`max_step` of the measured pose, if one is set (off by default). One move is in flight at a time:
re-sending the same target is a no-op, so a controller task can hold its
setpoint every cycle, and a different target while moving is counted in
`refused`. A move is `reached` only when every measured joint is within
`goal_tolerance` (default 0.5°). Losing control, drives, or a joint fault
mid-move marks it `aborted`; tracking stops, and `stop` disables the drives.

| Param | Default | Bounds |
|---|---|---|
| `allow_motion` | `false` | requires `allow_control` |
| `velocity_percent` | `5.0` | 0 < v ≤ 25 |
| `max_step` | none | > 0 when set |
| `goal_tolerance` | 0.5° | > 0 |

Verified on RobotControl V980-14 with `examples/move.rs`: A6 +5° and back at
5 %, each move reached in about 1.6 s. Observed frames: `CMDACK <counter>`
for the move, then `EXECACK 0 0 MoveTo` at start and `EXECEND -1 0 MoveTo PLAN`
at the end. After `Disable` the controller also reported
`EXECERROR 0 0 MoveTo Hardware er…` (truncated). That most likely means the
finished move program saw the drives go off, but it is unconfirmed. `EXEC*`
frames are informational only; completion is judged from measured joints.

```sh
cargo run -p repose-igus-cri --features control --example move -- 192.168.3.11:3920 --allow-motion
```

## Motion (stage 2b: streaming velocity)

`joint_command` with `mode: velocity` streams CRI jog: the bridge sends
`ALIVEJOG` every 50 ms with per-joint percent of `jog_full_scale`. The jog is
the command's feed-forward velocity plus per-joint PID feedback on
(commanded − measured position, in degrees), clamped to `max_jog_percent`; a joint whose jog would
cross its limit within 0.2 s is zeroed. Same opt-ins and gates as Move Joint
(in control, motion allowed, no Move Joint in flight). A command older than
`command_timeout` drops the jog to zero. Send commands every cycle and end a
motion with zero velocity on the final position, so feedback settles it.

| Param | Default | Notes |
|---|---|---|
| `jog_full_scale` | 60 °/s | measured: CRI jog is percent of max joint speed, linear, the same on A1–A6 |
| `max_jog_percent` | 10 | 6 °/s |
| `command_timeout` | 0.1 s | |
| `joint_gains` | 6 × `(kp: 2.0, ki: 0.0, kd: 0.0, integral_limit: 0.5)` | per joint A1–A6, see below |

`joint_gains` is a list of six entries, A1 first. `kp` is 1/s, `ki` 1/s²,
`kd` dimensionless; all finite and >= 0. `integral_limit` (0–1) caps the
integral term at that fraction of the jog cap (`max_jog_percent` of
`jog_full_scale`). The derivative is taken on the position error between
consecutive commands. The PID restarts (proportional only) on the first
command and after any gap longer than `command_timeout`. Omitted fields in an
entry keep their defaults:

```ron
"joint_gains": [
    {"kp": 2.0, "ki": 0.5},
    {"kp": 2.0, "ki": 0.5},
    {"kp": 2.5},
    {}, {}, {"kp": 1.5, "kd": 0.05},
],
```

Only the defaults (P = 2.0) have been verified on hardware.
Verified on RobotControl V980-14 with `examples/jog.rs` (A6 ±3 °/s, 2 s legs):
tracking error ≤ 0.02° at constant speed, about 1° lag for 0.5 s after a
direction reversal (controller acceleration ramp), and 0.03° final error
after a 1 s hold. The idle heartbeat stays `ALIVEJOG 0 0 0 0 0 0 0 0 0`.

## Connection behavior

`start` connects with a bounded timeout and launches a TCP worker. That worker
sends an all-zero `ALIVEJOG` heartbeat every 200 ms, as required by CRI, and
the client counter runs 1–9999. Without the `control` feature it sends nothing
else and the transmit channel set is empty.

The worker handles fragmented and coalesced CRI frames in a fixed 8192-byte
buffer. A latest-value slot replaces older observations when Copper runs slower
than the incoming stream. `receive` uses a nonblocking lock and publishes each
STATUS frame once per channel. No sample, contention, or an observation older
than `stale_after` produces an empty message. Socket I/O runs outside the
Copper loop.

`Tov` estimates host receipt time in Copper's clock; CRI STATUS has no robot
measurement timestamp.

Malformed status, oversized frames, and transport disconnect terminate the
worker and make `receive` return an error. Call `stop` then `start` to
reconnect. Freeze/thaw is a no-op for the external transport; use recorded
Copper messages for replay.

## Protocol references

- [CRI client structure](https://wiki.cpr-robots.com/index.php/CRI_Client_Structure)
- [Vendor CRI specification](https://cpr-robots.com/download/CRI/CPR_RobotInterfaceCRI.pdf)
- [CRI interface setup](https://wiki.cpr-robots.com/index.php/CRI_Ethernet_Interface)
