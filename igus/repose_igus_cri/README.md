# igus ReBeL CRI bridge

`CuIgusCriBridge` connects Copper to an igus ReBeL over CRI (TCP) and speaks
the vendor-neutral Repose robot contract (`repose-robot-interface`). The same
channels and payloads are used by the MuJoCo simulator bridge and future vendor
bridges, so an application changes robots by changing one bridge entry.

Connect the host to the controller's Ethernet network, then run:

```sh
cargo run -p repose-igus-cri --example monitor -- 192.168.3.11:3920
```

The monitor prints observations for ten seconds. For an iRC simulator, pass its
numeric IP address and configured CRI port instead.

## Verify against the running ROS model

This check uses the exact `robot_description` parameter currently served by ROS.
A separate ROS `robot_state_publisher` computes reference transforms for the same
joint inputs that Repose receives. It publishes only on unique validation topics.
The observed ROS joint state and synthetic numerical test angles never become
robot commands. The controller's XYZ/ABC values and provisional offsets are not
used in this check.

With the robot's ROS stack running, capture a reference and verify it:

```sh
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 igus/repose_igus_cri/examples/ros_reference/capture.py --output artifacts/ros_fk
cargo run -p repose-igus-cri --example verify_ros_fk -- artifacts/ros_fk/reference.json
```

Python requires ROS `rclpy`, message/service packages, NumPy, SciPy, and PyYAML.
The reference publisher path defaults to the Jazzy installation and can be
changed with `--publisher`. Node and topic overrides are available with `--help`.

The capture contains one observed joint state, zero, positive/negative changes to
each joint, and sixteen seeded configurations. Dynamic TF must have the exact
input timestamp; fixed TF comes from that isolated publisher. The original URDF,
a collision-free import copy, SHA-256 hashes, input angles, named frames, and ROS
reference matrices are saved under the output directory. Only collision elements
are removed for the import copy; its joint XML is checked unchanged. This avoids
requiring mesh collision support for a kinematics-only test.

`verify_ros_fk` checks both captured URDF hashes, verifies frame names/joint order
and reference rotations, imports that URDF into Repose, and compares FK without
adjustable offsets. It writes `comparison.json`. Numerical tolerances are fixed
at 1e-9 m and 1e-7 rad; they describe implementation agreement, not physical robot
accuracy. Failure of the imported model returns an error. At `link6`, the preset
is also compared and reported separately as MATCH or DIFF; a preset DIFF remains
visible even if the imported model matches.

Repeat with `--tip tool0 --output artifacts/ros_fk_tool0` and
`--tip TCPGripper --output artifacts/ros_fk_tcp`, passing each resulting reference
to `verify_ros_fk`. These endpoints are imported from their actual fixed URDF
joints, including tool translations and rotations, with no fitted correction.

On the locally running FHTW description, all three endpoints agree at numerical
precision across thirty configurations each. The preset differs slightly because
it uses π where the ROS URDF uses `3.1416` for the base yaw. This establishes
agreement with the ROS model. Physical calibration and the controller's Cartesian
frame/ABC convention remain separate, unverified questions. The active controller
tool configuration is not available in this workspace.

## Live Repose forward kinematics

Run the Repose example to compute a pose from each fresh observation:

```sh
cargo run -p repose-igus-cri --example forward_kinematics -- 192.168.3.11:3920
```

It runs for ten seconds and prints the model endpoint (`link6`) position in metres and its 3×3
rotation matrix relative to the Repose model's base frame. The rotation maps
endpoint-frame vectors into the base frame. The example passes measured A1–A6
angles directly into `igus_rebel_chain()` and writes FK into a reused transform.
It uses the preset's default tool offset; an attached gripper's TCP requires its
own offset. Compare against ROS using the same base and endpoint frames, joint
zeros, and robot model before interpreting differences as pose error.

Repose is pinned to a Git revision as an example-only development dependency;
the bridge library keeps its Copper-only runtime dependencies. This example
uses the kinematics API directly and keeps control with the existing ROS driver.

## Compare Repose with the controller

```sh
cargo run -p repose-igus-cri --example compare_kinematics -- igus/repose_igus_cri/examples/compare_kinematics.ron
```

Edit the RON file for your controller address, ABC convention, and frame offsets.
The example runs for ten seconds, reporting Euclidean position error in mm and
shortest relative rotation angle in degrees, then mean/max errors and sample count.
The first observation also prints the joint angles, raw controller XYZ/ABC,
Repose XYZ, signed XYZ residuals, both rotation matrices, and orientation errors
under each supported ABC hypothesis. Capture this snapshot when diagnosing large
errors. Candidate errors are diagnostic only; they do not change the configured
convention or establish which convention is correct.

It reads measured joints and `POSCARTROBOT` from one STATUS frame through the
vendor `cri_status` channel (`CuCriStatus`), so the pair is always consistent.

The configuration requires an explicit `abc` choice. The supplied file selects
`xyz` as an assumption, **not a confirmed controller convention**:

- `xyz`: `Rz(C) Ry(B) Rx(A)` (A=roll, B=pitch, C=yaw).
- `zyx`: `Rz(A) Ry(B) Rx(C)` (A=yaw, B=pitch, C=roll).

Confirm the convention for your controller before interpreting orientation errors.
The bridge preserves ABC angles without assuming a convention. The vendor
[CRI specification](https://wiki.cpr-robots.com/images/2/23/CPR_RobotInterfaceCRI.pdf)
identifies `POSCARTROBOT` as the robot TCP in mm and degrees; it does not establish
the Euler sequence used here. Do not select a convention solely because it gives
a smaller error at one pose.

Offsets use SI metres and radians; `rpy` means `Rz(yaw) Ry(pitch) Rx(roll)`.
`base` maps the Repose model base into the controller reference frame, and `tool`
maps the Repose model endpoint (`link6`) to the controller's configured TCP. The comparison is
`base × FK(joints) × tool` versus the controller TCP. Identity offsets are suitable
only when those frames already match. For a TCP 100 mm along endpoint Z, set
`tool.translation` to `(0.0, 0.0, 0.1)`. Use independently known offsets rather than
fitting away an unexplained model error.

The preset is tied to its source robot model; the name ReBeL alone does not
establish matching geometry, joint zeros/signs, base frame, or flange frame.
Other ReBeL descriptions use different joint transforms. The captured ROS model,
rather than the robot's product name, is the reference for model equivalence. An ABC-only change cannot correct the translational
residual because the Cartesian XYZ values are unchanged.

First compare several stationary configurations spanning the joints, using the
existing ROS controls to reposition the arm. Same-frame transport avoids pairing
separate messages, but does not guarantee that the controller calculates Cartesian
and measured joints from the same internal instant or feedback source. Motion can
therefore add tracking or timing differences. CRI joint/Cartesian rounding also
limits the achievable agreement. Missing Cartesian data or no fresh samples returns
an error; it never counts as a successful comparison.

Agreement checks model and frame consistency against the controller. Physical
accuracy still requires an independent measurement, and no universal pass threshold
is imposed by this diagnostic.

### Provisional FHTW frame alignment

The locally running FHTW `igus_rebel_description_real` arm geometry and axes
match the Repose preset, apart from the rounded base-yaw constant noted above. Its `link6` endpoint has a fixed +52.2 mm local-Z translation to ROS
`tool0`. The preset ends at `link6`; the default comparison's identity tool offset
therefore omits that translation. The `TCPGripper` frame is a different endpoint.

A separate diagnostic configuration accounts for this known length and tests a
controller-orientation hypothesis:

```sh
cargo run -p repose-igus-cri --example compare_kinematics -- igus/repose_igus_cri/examples/compare_fhtw_provisional.ron
```

This selects ZYX ABC and adds a local-Z half-turn to the model endpoint. The
half-turn is inferred from observed controller poses; it is **not** specified by
the ROS `link6` → `tool0` joint. Two recorded stationary observations exercise this
hypothesis in regression tests, but both have almost identical orientations.
Validate several distinct wrist orientations before accepting the controller
convention. Remaining translation discrepancies have not been calibrated away.
This configuration is specific to the observed setup, not a universal ReBeL preset.

## Channels

| Dir | Channel | Payload | Notes |
|---|---|---|---|
| Rx | `joint_state` | `repose_kinematics::CuJointStatePayload<6>` | standard; revolute A1–A6, velocities zero (CRI reports none) |
| Rx | `session_state` | `repose_robot_interface::CuSessionState` | standard; published on change |
| Rx | `cri_status` | `repose_igus_cri::CuCriStatus` | vendor: joints, TCP pose, mode, error codes, KINSTATE, ESTOP, override |
| Tx | `session` | `repose_robot_interface::CuSessionCommand` | standard; `control` feature only |
| Tx | `joint_command` | `repose_controller::CuJointCommandPayload<6>` | standard; `control` feature + `allow_motion`; `position` mode only |
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
`max_step` (default 10°) of the measured pose. One move is in flight at a time:
re-sending the same target is a no-op, so a controller task can hold its
setpoint every cycle, and a different target while moving is counted in
`refused`. A move is `reached` only when every measured joint is within
`goal_tolerance` (default 0.5°). Losing control, drives, or a joint fault
mid-move marks it `aborted`; tracking stops, and `stop` disables the drives.

| Param | Default | Bounds |
|---|---|---|
| `allow_motion` | `false` | requires `allow_control` |
| `velocity_percent` | `5.0` | 0 < v ≤ 25 |
| `max_step` | 10° | > 0 |
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
