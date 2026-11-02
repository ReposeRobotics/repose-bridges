# igus ReBeL CRI bridge

`CuIgusCriBridge` observes the six measured ReBeL joint angles over CRI TCP.
The `joint_state` Rx channel carries `CuRebelState`, with Copper `Angle` values
in A1–A6 controller order. CRI degrees are converted at the protocol boundary.

Connect the host to the controller's Ethernet network, then run:

```sh
cargo run -p repose-igus-cri --example monitor -- 192.168.3.11:3920
```

The monitor prints observations for ten seconds. For an iRC simulator, pass its
numeric IP address and configured CRI port instead.

## Copper wiring

Add this crate to your application's dependencies, then declare a bridge and
connect its receive channel to your consumer task:

```ron
bridges: [
    (
        id: "rebel",
        type: "repose_igus_cri::CuIgusCriBridge",
        config: {
            "address": "192.168.3.11:3920",
            "stale_after": 0.5,
            "connect_timeout": 2.0,
        },
        channels: [Rx(id: "joint_state")],
    ),
],
cnx: [
    (src: "rebel/joint_state", dst: "consumer",
     msg: "repose_igus_cri::CuRebelState"),
],
```

The config map is `CuIgusCriParams`; times are SI seconds. Unknown keys are
rejected. Defaults match the snippet. The crate does not depend on Repose:
applications can adapt the joint angles to their own state payload.

## Connection behavior

`start` connects with a bounded timeout and launches a TCP worker. That worker
sends an all-zero `ALIVEJOG` heartbeat every 200 ms, as required by CRI. It never
requests active control, enables motors, resets errors, or sends motion commands.
The transmit channel set is empty. Read-only describes the exposed capability;
TCP traffic includes the mandatory heartbeat.

The worker handles fragmented and coalesced CRI frames in a fixed 8192-byte
buffer. A latest-value slot replaces older observations when Copper runs slower
than the incoming stream. `receive` uses a nonblocking lock and consumes each
observation once. No sample, contention, or an observation older than
`stale_after` produces an empty message. Socket I/O runs outside the Copper loop.

`Tov` estimates host receipt time in Copper's clock; CRI STATUS has no robot
measurement timestamp. Network delay and controller sampling delay remain
unknown. Only measured position is exposed; velocities are not synthesized.

Malformed status, oversized frames, and transport disconnect terminate the
worker and make `receive` return an error. Call `stop` then `start` to reconnect.
`stop` and drop shut down and join the worker. Freeze/thaw is a no-op for the
external transport; use recorded Copper messages for replay.

This first version has localhost protocol tests. Hardware acceptance on a ReBeL
and controller firmware compatibility are still to be verified.

## Protocol references

- [CRI client structure](https://wiki.cpr-robots.com/index.php/CRI_Client_Structure)
- [Vendor CRI specification](https://cpr-robots.com/download/CRI/CPR_RobotInterfaceCRI.pdf)
- [CRI interface setup](https://wiki.cpr-robots.com/index.php/CRI_Ethernet_Interface)
