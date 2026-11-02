default: pr-check

fmt:
    cargo fmt --all

lint:
    cargo fmt --all -- --check
    cargo clippy --workspace --all-targets -- --deny warnings

test:
    cargo test --workspace

pr-check: fmt lint test
