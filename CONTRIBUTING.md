# Development

Run `just fmt` after edits and `just pr-check` before committing. The PR gate
checks formatting, all-target clippy with warnings denied, unit and example tests, and
doctests. Transport tests use localhost sockets and require no robot.

Use manufacturer directories and underscore crate directory names. Crates use
hyphenated package names and Copper-native bridges with typed channels. Keep
socket I/O outside the Copper loop, bound protocol buffers, and use Copper units
for physical quantities. New command capabilities need explicit API and protocol
review. Record user-visible changes under Unreleased in CHANGELOG.md.

The ROS reference capture is an opt-in integration check documented in the bridge
README. It needs a running robot_description parameter service and joint-state
publisher. The standard PR gate does not require a live ROS environment. Generated
URDF/reference/report artifacts are ignored by Git; preserve their hashes when
sharing results.
