default: pr-check

fmt:
    cargo fmt --all

lint:
    cargo fmt --all -- --check
    cargo clippy --workspace --all-targets -- --deny warnings
    cargo clippy -p repose-ur --all-targets --features control -- --deny warnings
    cargo clippy -p repose-igus-cri --all-targets --features control -- --deny warnings

test:
    cargo test --workspace --all-targets
    cargo test -p repose-ur --all-targets --features control
    cargo test -p repose-igus-cri --all-targets --features control
    cargo test --workspace --doc

pr-check: fmt lint test
