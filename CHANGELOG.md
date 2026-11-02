# Changelog

## Unreleased

- Template: use `repose::planners` (`CuPlannerChoice` / `CuAnyPlanner`) from
  Repose v0.7.1 instead of the template's own `planner.rs`, and pin all Repose
  dependencies (template, `repose-igus-cri`, `rebel_app`) to the `v0.7.1` tag.
  The `planner` key in copperconfig.ron is unchanged.

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
