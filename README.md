# Repose Bridges

Copper-native hardware bridges, grouped by manufacturer. This workspace is
independent of Repose's planning and kinematics workspace. Vendor bridges depend
on Copper directly; applications connect their measured state to Repose tasks.

| Manufacturer | Crate | Device | Interface |
|---|---|---|---|
| igus | [repose-igus-cri](igus/repose_igus_cri) | ReBeL, six axes | CRI TCP, measured joint angles |

Requires Rust 1.95 or newer and Copper 1.3.1. Host transports use `std`.
New manufacturers get their own top-level directory (for example `abb/`).
