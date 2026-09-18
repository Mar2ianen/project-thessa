## What changes

<!-- Describe the change and why it is needed. -->

## Checks

- [ ] `cargo fmt --all -- --check`
- [ ] `cargo clippy --workspace --all-targets --all-features -- -D warnings`
- [ ] `cargo test --workspace`
- [ ] Numerical tolerances or conserved quantities were checked when physics changed
- [ ] Current documentation/ADR was updated when a contract or license changed

## License and boundaries

- [ ] New dependencies are compatible with the target crate license
- [ ] Code does not flow from GPL game crates into MIT engine crates
- [ ] Platform-specific APIs did not enter domain or simulation code
