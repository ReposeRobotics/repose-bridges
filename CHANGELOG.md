# Changelog

## Unreleased

- **Breaking:** Rename robot-program steps: `Joints` → `JAbsolut`, `Offset` →
  `JRelative`, `Plan` → `JPlanned`, and `Tool` → `TCPRelative`. Update programs
  to the new variants; their units and motion behavior are unchanged.
  `TCPRelative` solves a relative tool target with endpoint IK and follows a
  joint-space trajectory; it does not promise a straight TCP path.


- Add `repose-abb` under `abb/`: ABB OmniCore bridge over Externally Guided
  Motion (EGM). Default builds only receive EGM feedback (joint state, motor,
  MCI and RAPID state); the `control` feature adds the standard session and
  `joint_command` channels, answering feedback with joint corrections inside
  an explicitly enabled session. Includes a bounded protobuf codec, monitor and
  move examples, and a sample RAPID module. Not yet verified on RobotStudio or
  hardware.

- Template: `cargo generate` now asks for the robot (`igus_rebel` or `ur5e`)
  and only that robot's address. The project depends only on that robot's
  bridge crate, and its profile becomes `copperconfig.ron` directly; `robot.ron`
  and the profile-selecting build script are gone. Switching robots now means
  generating a new project.

- Per-joint PID gains for streamed velocity commands on `repose-igus-cri` and
  `repose-ur`: `joint_gains` in RON takes six `{kp, ki, kd, integral_limit}`
  entries (shared `repose-joint-pid` crate). **Breaking:** replaces the scalar
  `position_gain`; the default (`kp: 2.0`, no I/D) behaves as before. I/D
  terms are not yet verified on hardware. The template's robot profiles list
  `joint_gains` with the defaults, ready to tune.

- Template: select igus ReBeL or UR5e/URSim in `robot.ron`, then `cargo run`
  the same program with the Repose v0.7.1 planner API. The build selects the
  matching bridge/model RON profile. Include separate address prompts and
  relative moves in the default program for either robot.

- Template: use `repose::planners` (`CuPlannerChoice` / `CuAnyPlanner`) from
  Repose v0.7.1 instead of the template's own `planner.rs`, and pin all Repose
  dependencies (template, `repose-igus-cri`, `rebel_app`) to the `v0.7.1` tag.
  The `planner` key in copperconfig.ron is unchanged.
- Add `repose-ur` under `universal_robots/`: RTDE v2 joint feedback for URSim
  and Universal Robots, plus opt-in controller-planned URScript `movej`
  commands. Includes bounded worker transport, stale feedback and command
  gates, monitor/move examples, and a pinned URSim Docker Compose setup.
  Verified RTDE feedback and ±0.05 rad wrist moves on URSim 5.25.1 (UR5e).
  Add the standard Repose session channels and streamed velocity control with
  position feedback, command expiry, and a resident robot-side RTDE watchdog.
  `ur-app` uses the exact same Step API, runner and planners as the igus template;
  only robot configuration and program targets differ. Physical hardware remains
  unverified.

- Add streaming velocity control: `joint_command` in `velocity` mode streams
  CRI jog every 50 ms (feed-forward plus position feedback, speed cap, limit
  lookahead, 100 ms command timeout). Jog units measured on hardware: percent
  of 60 °/s, linear, on all axes. The heartbeat now runs at 50 ms, as in the
  vendor driver. Adds a `jog` example and a `cargo jog` alias.

- Add point-to-point motion: Tx `joint_command` (`CuJointCommandPayload<6>`,
  position mode) sends controller-planned `Move Joint`, behind a third opt-in
  `allow_motion`. The bridge applies local gates (in control, motion allowed,
  joint limits, `max_step`), keeps one move in flight, treats a repeated
  target as a no-op, and reports reached/rejected/aborted on the vendor Rx
  `cri_motion` channel. Adds a `move` example; first motion verified on
  hardware (A6 ±5° at 5 %).

- **Breaking:** the bridge implements the vendor-neutral Repose robot contract
  (`repose-robot-interface`). `joint_state` now carries
  `repose_kinematics::CuJointStatePayload<6>` (was `CuRebelState`), with
  `CuRobotStatus` derived from CRI error codes and KINSTATE. The `status` and
  `health` channels merge into the vendor `cri_status` channel (`CuCriStatus`,
  was `CuRebelStatus` + `CuCriHealth`). The session channels are now Tx
  `session` (`CuSessionCommand`) and Rx `session_state` (`CuSessionState`).
- Add opt-in session control (`control` feature plus `allow_control`):
  acquire/release control (`SetActive`), clear faults, enable/disable drives,
  with one command in flight, local refusal while passive, sequence-matched
  CMDACK/CMDERROR, and disable + release on stop. Verified on RobotControl
  V980-14. No motion commands. Adds a `session` example.
- Split the crate into `params`, `protocol`, `mapping`, `transport` and
  `bridge` modules.
- CRI client counter now runs 1–9999 as in the vendor driver (was 0–9999).

- Add `rebel-app`, a minimal Copper program driving the ReBeL through the
  bridge, and `cargo move` / `cargo session` aliases.
- Remove the FK verification and comparison examples (`verify_ros_fk`,
  `compare_kinematics`, `forward_kinematics`, `monitor`, ROS capture script).

- Add the independent manufacturer-organized Repose Bridges workspace.
- Add a receive-only igus ReBeL CRI Copper bridge, measured six-axis joint
  positions, mandatory zero-jog heartbeat, and bounded TCP worker.
