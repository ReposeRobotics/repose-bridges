# repose-panthera-ht

Copper bridge for the [HighTorque Panthera HT](https://hightorquerobotics.com/Panthera-HT_Hub/index_en.html)
6-DOF arm. The vendor wire protocol is reimplemented in pure Rust from the
open-source SDK [`HighTorque-Robotics/hightorque_robot`](https://github.com/HighTorque-Robotics/hightorque_robot)
(`serial_struct.h`, `canport.cpp`, `crc.cpp`, `convert.cpp`); the lerobot
plugin `HighTorque-Robotics/Panthera-HT_lerobot` wraps the same SDK.

## Wire traffic

USB serial to the v6 communication board (VID:PID `CAF1:FFFF`), 4 Mbaud.
Every frame is `0xF7 | cmd | len(u16 LE) | crc_head | crc_data | data[len]`
with CRC-8/CCITT (init 0xFF, table from the SDK) over `cmd,len` and `data`.
RX frames prefix `data` with the 3-byte CAN-FD port state
(fault, tx_err, rx_err). Relevant commands:

| cmd | dir | meaning |
|---|---|---|
| `0x01` | both | comm-board version (major, minor, patch) |
| `0x02` | both | comm-board init handshake |
| `0x04` | both | set cached motor count |
| `0x14` | both | motor state request / reply (query 11: id, fault, mode, pos, vel, tqe per motor, int16 turns × scale) |
| `0x98` | TX | MIT position command `pos_vel_tqe_kp_kd`, 10 bytes per motor slot (indexed by CAN id − 1), unused slots `0x8000` |

Angle wire scale: int16 turns = rad / 2π × 10000 (pos), × 4000 (vel/s);
torque × 100 per Nm; kp/kd × 100 per rad after a 2π factor (SDK quirk).

## Lifecycle

`start` opens the port, requires comm-board firmware ≥ v6.0.0, runs the
init/cache handshake, and spawns the serial worker. The worker sends a motor
state request at `frequency` Hz and, with the `control` feature, re-sends the
latest position command while it is younger than `command_timeout`. All
serial I/O stays on the worker; `receive` only reads shared state.

## Timestamp meaning

`joint_state` / `panthera_status` messages carry `tov` = Copper time of
receipt minus the age of the motor-state frame, i.e. the frame's sampling
time on the host clock. `session_state` is event-driven.

## Configuration

All parameters live in `CuPantheraHtParams` (RON `config:` block,
`deny_unknown_fields`): `serial_port` (empty = auto-detect the
`can_port_id`-th comm board), `baud`, `joint_ids` (six CAN ids 1..=30),
`frequency`, `stale_after`, `connect_timeout`, `command_timeout`,
`allow_control`, `allow_motion`, `max_step`, `kp`, `kd`.

## Channels

Rx: `joint_state` (`CuJointStatePayload<6>`), `panthera_status`
(`CuPantheraStatus`), `session_state` (`CuSessionState`).
Tx (feature `control`, plus `allow_control`/`allow_motion`): `session`
(`CuSessionCommand`, acknowledged locally — the serial link is
single-master), `joint_command` (`CuJointCommandPayload<6>`, position mode
only, sent as a `0x98` MIT frame with the configured `kp`/`kd`).

## Hardware verification

Not yet verified on a physical Panthera HT. The codec is unit-tested against
the SDK's frame layout and CRC; motion on hardware should be attempted only
after monitor-mode telemetry is confirmed with `panthera_monitor`.
