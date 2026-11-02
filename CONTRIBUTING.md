# Development

Run `just fmt` after edits and `just pr-check` before committing. The PR gate
checks formatting, all-target clippy with warnings denied, unit tests, and
doctests. Transport tests use localhost sockets and require no robot.

Use manufacturer directories and underscore crate directory names. Crates use
hyphenated package names and Copper-native bridges with typed channels. Keep
socket I/O outside the Copper loop, bound protocol buffers, and use Copper units
for physical quantities. New command capabilities need explicit API and protocol
review. Record user-visible changes under Unreleased in CHANGELOG.md.
