# Changelog

## Unreleased

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

- Add a same-input ROS/Repose FK verifier using an isolated robot_state_publisher,
  the active ROS URDF, timestamp-matched references, and saved SHA-256 provenance.
- Validate named link6, tool0, and TCPGripper endpoints without fitted offsets;
  report the preset's rounded-base-yaw discrepancy separately.
- Clarify that the preset endpoint is link6 and controller alignment is unverified.

- Add a provisional FHTW comparison configuration with the known ROS link6-to-tool0
  length and an explicitly unverified controller orientation hypothesis, covered
  by two recorded hardware snapshots.

- Print a first-observation snapshot and both ABC orientation hypotheses in the
  FK comparison to diagnose model, frame, and convention mismatches.

- Add a paired CRI status channel with optional unit-typed Cartesian TCP readings,
  preserving the existing joint-state payload and channel.
- Add a controller-versus-Repose FK comparison example with explicit ABC convention,
  base/tool offsets, and position/orientation error statistics.
- Include example tests in the local PR gate.

- Add a live ReBeL forward-kinematics example using Repose to report the
  base-to-flange position and rotation from measured CRI joint angles.

- Add the independent manufacturer-organized Repose Bridges workspace.
- Add a receive-only igus ReBeL CRI Copper bridge, measured six-axis joint
  positions, mandatory zero-jog heartbeat, bounded TCP worker, and monitor example.
