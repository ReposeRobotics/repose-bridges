# {{project-name}}

A Repose robot program on Copper.

- `src/program.rs` — your steps (joint moves, offsets, tool moves via IK, waits)
- `copperconfig.ron` — which robot (bridge), its address, speed, kinematic model
- `src/runner.rs` — the generic engine (session bring-up, step execution, wind-down)

```sh
cargo run      # the arm MOVES: keep the e-stop in hand
```

Ctrl-C stops cleanly: drives are disabled and control released.
