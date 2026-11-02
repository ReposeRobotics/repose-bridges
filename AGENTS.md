# Repose Bridges

Manufacturer-organized Copper (`cu29`) hardware bridges. Keep this workspace
independent of the Repose planning workspace. Vendor directories contain crates
and device integration examples; share code only when multiple drivers need it.

Use `CuBridge` with typed static channel sets. Payloads derive Default, Debug,
Clone, Encode, Decode, Serialize, Deserialize, and Reflect. Use Copper physical
units and put timestamps on CuMsg envelopes. Fallible APIs return CuResult.

Keep network I/O outside the runtime loop. Runtime receives must be bounded,
nonblocking, and allocation-free on success. Bound protocol frame capacity and
reject malformed input. Device commands require an explicit requested scope;
monitoring bridges expose no transmit channels.

Document each bridge's wire traffic, lifecycle, timestamp meaning, configuration,
and hardware verification status. Keep Rust and RON configuration in one params
struct with validation and deny_unknown_fields. Run `just fmt` after editing and
`just pr-check` before committing. Add user-visible changes to CHANGELOG.md.
