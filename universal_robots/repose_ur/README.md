# Universal Robots / URSim

`repose-ur` connects directly to UR's controller interfaces. The same bridge and
payloads target URSim or a physical robot by changing the numeric IP address.
Initial target: e-Series running PolyScope 5 with RTDE protocol v2. CB3 and
PolyScope X have not been validated.

| Direction | Channel | Payload |
|---|---|---|
| Rx | `joint_state` | `repose_kinematics::CuJointStatePayload<6>` |
| Rx | `ur_status` | `CuUrStatus` |
| Tx, `control` feature only | `joint_command` | `repose_controller::CuJointCommandPayload<6>` |

Default builds expose **no Tx channels**. The `control` feature exposes the
standard `session` and `joint_command` channels; `session_state` is always an Rx
channel carrying `repose_robot_interface::CuSessionState`. Session operations
require `allow_control: true`; motion additionally requires `allow_motion: true`.

The shared [robot program](../ur_app) uses exactly the igus template's runner,
planner and `program::steps()` API: `Joints`, `Offset`, `Tool`, `Plan`, `Wait`.
Velocity mode streams feed-forward joint velocity plus position-error feedback
through a resident URScript loop. Position mode still supports individual
controller-planned `movej` commands outside a streaming session. Effort is refused.

## URSim

From this directory:

```sh
docker compose up -d
# Open http://localhost:6080/vnc.html in a browser.
# Complete initial PolyScope setup, confirm the simulator safety configuration,
# enable Remote Control in Settings, then select Remote Control.
# For motion: power on and release brakes from PolyScope first.
cargo run -p repose-ur --example ur_monitor
# Explicitly moves wrist 3 by +0.05 radians on the selected controller:
cargo run -p repose-ur --features control --example ur_move -- \
  --move-wrist-radians 0.05 127.0.0.1:30004
docker compose down
```

The compose file pins URSim 5.25.1, binds ports to localhost, and persists the
simulator's programs. Initial setup may need repeating after recreating the
container. On a physical robot enable the RTDE and secondary client services
where required by its security settings, select remote mode, and substitute its
address. No External Control URCap or ROS installation is needed for this
point-to-point path.

## Configuration

`CuUrParams` is the sole Rust/RON configuration schema. Missing fields use its
defaults and unknown fields are rejected. A bridge config block (SI units):

```ron
config: {
    "address": "127.0.0.1:30004",
    "script_port": 30002,
    "dashboard_port": 29999,
    "allow_control": false,
    "command_timeout": 0.1,
    "position_gain": 2.0,
    "frequency": 50.0,
    "connect_timeout": 2.0,
    "stale_after": 0.5,
    "allow_motion": false,
    "speed": 0.1,
    "acceleration": 0.2,
    "max_step": 0.25,
    "goal_tolerance": 0.001,
    "move_timeout": 30.0,
},
```

Frequency is 1–125 Hz (at least 25 Hz with control enabled). All durations are positive, finite, at most one hour.
Speed is >0 and <=1 rad/s, acceleration >0 and <=2 rad/s², max_step >0 and <=π
radians, tolerance >0 and <=0.1 radians. `max_step` is an application gate from
measured joint position, not a robot model's joint limit or collision check.
The robot's configured limits remain authoritative; the caller must plan a
valid path. No model-specific limits or kinematics are embedded here.

## Wire traffic and lifecycle

`start` connects to RTDE TCP port 30004, requests protocol v2, negotiates an
output recipe and starts synchronization. The recipe is `timestamp,actual_q,
actual_qd,robot_mode,safety_mode,runtime_state,output_int_register_24`; the exact type list and recipe
ID are checked. Monitoring sends only these setup requests; it writes no RTDE
inputs or URScript. With control enabled, startup additionally reserves input
integer register 24 and input double registers 24–29; registration fails if
another client owns them. Each data frame is exactly 124 bytes including its header.
The parser limits any frame to 2048 bytes and rejects invalid lengths, recipe
IDs, unexpected packets and nonfinite measurements. Controller text during
negotiation is reported as an error.

After setup a worker owns all network I/O. Copper receives use `try_lock`, a
single latest-sample slot, independent per-channel generations, and fixed-size
payloads. Empty, contended, previously consumed or stale samples clear output.
Successful receives allocate no memory and never wait for I/O. CuMsg `tov` is
estimated **host receipt time** in Copper's clock domain. Controller uptime
(seconds since controller boot) stays in `ur_status`; no controller/host clock
synchronization is claimed. Normal/reduced safety mode plus robot mode RUNNING
maps to Ready; other safety modes map to Fault carrying the raw mode code.
Ready alone does not mean a command will be accepted.

The standard session commands use Dashboard TCP 29999 and `CuSessionTracker`:
AcquireControl verifies Remote Control and a stopped program, then claims a local
session (UR provides no exclusive network-client lock). ClearFaults acknowledges
healthy normal/reduced safety state; other safety states require manual reset.
EnableDrives sends power-on/brake-release as needed, uploads the resident program
and waits for its output-register marker and running state before acknowledging.
DisableDrives stops the resident loop, powers off, and waits for power-off feedback.
ReleaseControl winds down a remaining owned streaming session and releases the
local claim. Replies and observed state drive acknowledgement; session commands
time out rather than being assumed successful. Other clients must not control
the robot concurrently.

The resident program consumes run flag input int 24 and velocities input doubles
24–29 using `speedj` at 20 ms. It reports marker 19729 through output int 24. The
worker sends one RTDE input packet at most every 20 ms; an expired application
command becomes zero velocity after `command_timeout`. Targets must be finite
revolute coordinates within `max_step` of measured positions. Feed-forward plus
`position_gain` times position error is clamped to configured joint `speed`.
No network I/O happens in Copper send/receive. The robot-side RTDE watchdog stops
the program when input packets fall below 8 Hz, including a killed host process.
See [UR's watchdog reference](https://www.universal-robots.com/manuals/EN/HTML/SW10_12_1/Content/prod-scriptmanual/all_scripts/rtde_set_watchdog_variable_name.htm).

Position-mode commands enqueue one fixed-size target; the worker sends a complete
`movej` program to secondary TCP 30002. They require a stopped program, stationary
fresh measurements and no active streaming session. Identical targets are
idempotent until restart. Completion is inferred from target tolerance,
near-zero speed and stopped runtime state; rejected scripts can time out.

Disconnect, stale feedback, interrupted streaming program or malformed protocol
faults the bridge. Stop/start explicitly to reconnect; queued targets are never
replayed. Stop/drop joins the worker, attempts a stop for owned motion and powers
off a streaming session this bridge enabled. These network cleanup requests are
best effort. The robot-side watchdog covers streaming communication loss;
standalone `movej` can continue to its target if connectivity is lost.

## Verification

Automated tests cover fragmented RTDE TCP traffic, negotiation, independent
channel consumption, contention, malformed data, parameter validation, local
motion gates, exact move program dispatch and best-effort stop on shutdown.

Verified on URSim 5.25.1 (UR5e), 2026-11-02, after initial PolyScope setup:
RTDE feedback reported robot mode 7, safety mode 1 and stopped runtime state 1.
The bridge moved wrist 3 by +0.05 rad, then -0.05 rad; both examples reached
within the configured 0.001 rad tolerance. The shared program runner also completed
a three-step streamed program (first three joints +5°, wait, then −5°),
including session bring-up, preflight, drive disable and release. Physical hardware remains unverified.

Protocol references: [UR RTDE guide](https://docs.universal-robots.com/tutorials/communication-protocol-tutorials/rtde-guide.html),
[UR interfaces](https://www.universal-robots.com/articles/ur/interface-communication/overview-of-client-interfaces/),
[movej](https://www.universal-robots.com/manuals/EN/HTML/SW10_10/Content/prod-scriptmanual/all_scripts/movej_qa14v105t0r.htm),
[official URSim setup](https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/setup/ursim_docker.rst).
