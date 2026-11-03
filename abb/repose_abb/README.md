# ABB OmniCore (EGM)

`repose-abb` talks to ABB controllers through Externally Guided Motion (EGM):
the controller streams `EgmRobot` protobuf datagrams over UDP and, while a
RAPID `EGMRunJoint` is active, follows `EgmSensor` joint corrections sent back.
Initial target: OmniCore controllers on RobotWare 7 with the EGM option, six
revolute axes (for example GoFa CRB 15000). IRC5, seven-axis robots and
external axes are not supported.

| Direction | Channel | Payload |
|---|---|---|
| Rx | `joint_state` | `repose_kinematics::CuJointStatePayload<6>` |
| Rx | `abb_status` | `CuAbbStatus` |
| Rx | `session_state` | `repose_robot_interface::CuSessionState` |
| Tx, `control` feature only | `joint_command` | `repose_controller::CuJointCommandPayload<6>` |
| Tx, `control` feature only | `session` | `repose_robot_interface::CuSessionCommand` |

Default builds expose **no Tx channels** and never send a datagram. Session
commands require `allow_control: true`; joint commands additionally require
`allow_motion: true`.

## Controller setup

1. Configuration → Communication → Transmission Protocol: add `ROB_1`, type
   `UDPUC`, remote address = the Repose host, remote port = the bridge's bind
   port (default 6510).
2. Load [`rapid/ReposeEgm.modx`](rapid/ReposeEgm.modx) into a motion task
   and review it. It starts feedback streaming (`EGMStreamStart`), then loops
   `EGMRunJoint` with position-correction gain 1 and a one-second
   communication timeout.
3. Switch motors on and run the program (auto mode, or manual with the
   enabling device held).

```sh
cargo run -p repose-abb --example abb_monitor -- 192.168.125.1 0.0.0.0:6510
# Explicitly moves joint 6 by +0.05 rad, then disables and releases:
cargo run -p repose-abb --features control --example abb_move -- \
  --move-wrist-radians 0.05 192.168.125.1
```

## Configuration

`CuAbbParams` is the sole Rust/RON configuration schema; missing fields use
defaults and unknown fields are rejected. SI units:

```ron
config: {
    "bind": "0.0.0.0:6510",
    "robot_ip": "192.168.125.1",
    "allow_control": false,
    "allow_motion": false,
    "stale_after": 0.1,
    "command_timeout": 0.1,
    "session_timeout": 10.0,
    "speed": 0.5,
    "max_step": 0.25,
},
```

Addresses must be numeric. Durations are positive, finite, at most one hour.
`speed` is >0 and <=2 rad/s; `max_step` is >0 and <=π rad. `max_step` is an
application gate from measured position, not a joint limit or collision
check; the controller's configured limits and `MaxSpeedDeviation` remain
authoritative.

## Wire traffic and lifecycle

`start` binds the UDP socket; afterwards a worker thread owns all I/O.
Datagrams from any address other than `robot_ip` are ignored. Each datagram
is limited to 1400 bytes and decoded by a bounded protobuf reader (proto2
`egm.proto`): header (`seqno`, `tm`), feedback and planned joints (packed or
unpacked, exactly six, finite, degrees on the wire), feedback clock, motor,
MCI and RAPID execution state, convergence flag and utilization. Unknown
fields are skipped; truncated fields, bad wire types, overlong varints and
missing feedback fault the bridge.

Copper receives use `try_lock`, one latest-sample slot and per-channel
generations: contended, consumed or stale samples clear the output, and
successful receives allocate nothing and never wait for I/O. CuMsg `tov` is
estimated **host receipt time**. Controller time (`header.tm`, ms) and the
feedback clock stay in `abb_status`; no clock synchronization is claimed.
Velocities are a host-side finite difference of consecutive feedback (using
the feedback clock when present). Motors on + RAPID running + MCI not in error
maps to Ready; MCI error maps to `Fault(1)`.

### Session and motion (`control`)

EGM offers no remote motor or program control, so the session gates only
whether this bridge answers. Outcomes come from observed EGM state, through
`CuSessionTracker`:

- **AcquireControl** acknowledges on fresh feedback. It claims a local
  session; EGM has no exclusive client lock, so no other sensor may answer
  the same UDPUC device.
- **ClearFaults** acknowledges unless MCI reports error (reset the RAPID
  program instead).
- **EnableDrives** requires `allow_motion`, motors on and RAPID running, then
  starts answering every feedback message by holding the measured position,
  and acknowledges once MCI reports running. Motors are not switched.
- **DisableDrives** / **ReleaseControl** stop answering at once. EGM then
  holds and ends on its communication timeout; the sample program catches
  `ERR_UDPUC_COMM` and restarts.

Commands not observed to complete within `session_timeout` are rejected.
While streaming, every feedback datagram gets one `EgmSensor` reply (header,
planned joints, joint speed reference; degrees) to its source address. A
`joint_command` in Velocity mode sends its positions with its velocities as
feed-forward (clamped to `speed`); Position mode sends the positions with zero
speed. EGM closes the position loop itself, so no host PID is applied. Targets
must be within `max_step` of measured and need an enabled session with fresh,
ready feedback. A target older than `command_timeout` is replaced by holding
the position measured at that moment. Effort is refused.

Feedback silence longer than `stale_after`, motors off, RAPID stopped or MCI
error while streaming faults the bridge; stop/start to reconnect. Nothing is
replayed. The controller's `CommTimeout` covers a killed host process.

## Verification

Automated tests cover protobuf decoding (packed/unpacked, malformed input,
capacity), `EgmSensor` encoding, source filtering, velocity estimation,
channel consumption and contention, parameter validation, motion gates,
session acknowledgement and rejection, replies only while streaming, held
position on command expiry, and faulting on feedback loss.

**Not yet verified on RobotStudio or hardware.** The EGM field numbers follow
ABB's `egm.proto` as used by `abb_libegm`; the RAPID module is a starting
point. Record the RobotWare version and results here once verified.

References: ABB Application manual — Externally Guided Motion (3HAC073318,
RobotWare 7); [abb_libegm](https://github.com/ros-industrial/abb_libegm).
