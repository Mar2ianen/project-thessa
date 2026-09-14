# Contributing

Project Thessa is still a design and engineering prototype. Evaluate changes
against three questions:

1. Does the change strengthen the `factory -> logistics -> aerospace -> factory`
   loop or a documented foundation for it?
2. Does it preserve physical causality instead of hiding gameplay bonuses in
   coefficients?
3. Does it keep the simulation core independent of a renderer or network
   runtime?

## Minimum requirements for simulation changes

- use SI units inside authoritative state;
- add known-case tests for every new solver;
- keep wall-clock time out of physics hot paths;
- make results independent of hash-map iteration order;
- review the license and reason for every new dependency;
- support performance claims with a benchmark or trace.

Numerical regression tests should compare physically meaningful tolerances or
conserved quantities rather than use `==` for floating-point values.

## Cross-platform baseline

- route new platform APIs through an adapter crate/module;
- domain and simulation code must not accept Windows or DirectX types;
- reject shader features without a clear Vulkan/Metal/WebGPU path or graceful
  fallback;
- once targets exist, CI must include native Linux builds plus compile/smoke
  checks for Windows, macOS, and WASM.

## Licensing baseline

- an MIT engine crate must not depend on a GPL game crate;
- every package has an explicit SPDX `license`;
- a new copyleft dependency requires an ADR;
- do not copy reference implementation code into MIT engine crates without a
  compatible license; reimplement ideas and algorithms independently and test
  against public results.

## Before opening a change

Run:

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets --all-features -- -D warnings
cargo test --workspace
```

Update the relevant current implementation document or ADR when a contract,
crate boundary, wire format, or license boundary changes. Keep proposals and
roadmap items labelled as future work.
