# Project Thessa

> Factory, logistics, and aerospace engineering in one physical world.

[![CI](https://github.com/Mar2ianen/project-thessa/actions/workflows/ci.yml/badge.svg)](https://github.com/Mar2ianen/project-thessa/actions/workflows/ci.yml)
![Rust](https://img.shields.io/badge/Rust-1.95%2B-000000?logo=rust&logoColor=white)
![Engine license](https://img.shields.io/badge/engine-MIT-blue)
![Game license](https://img.shields.io/badge/game-GPL--3.0--or--later-blue)

Project Thessa is a cross-platform Rust prototype for a factory and aerospace
sandbox. The intended game loop connects production, surface logistics, vehicle
design, atmospheric flight, orbital mechanics, and interplanetary logistics.

The repository is currently a **pre-alpha engineering prototype**. Internal APIs,
data formats, and design values may change before `0.1.0`; no compatibility
promise is made for them.

## Current implementation

The checked-in vertical slice currently contains:

- `thessa-sim-core`: compatibility facade for deterministic baked ephemerides,
  full multi-body test-particle gravity, atmosphere and panel aerodynamics,
  rigid-body flight, on-rails coast caches, cohort gravity patches, piecewise
  analytic affine propagation, cabin pressure and crew authority, part
  assembly connectivity, installed reaction-wheel banks, deployable
  parachutes, and a parameterized wire-free vehicle power bus with batteries,
  ultracapacitors, fission reactors, sun-tracking cell-grid solar arrays with
  geometric occlusion, and prioritized consumers. The
  implementation is separated into `thessa-aero-core`,
  `thessa-celestial`, `thessa-trajectory`, `thessa-propulsion`, and
  `thessa-vehicle-core`;
- `thessa-aero-surfaces`, `thessa-fuselage`, and `thessa-vehicle-baker`:
  procedural lifting-surface/body compilers, interior-volume and mass
  properties, cabin layouts and presets, static exits, controls, and assembled
  vehicle asset compilation including electrical power hardware;
- `thessa-plume-core`: backend-neutral engine-exhaust profiles, optics, and
  participating-medium evaluation, consumed by the client plume renderer;
- `thessa-flight-control` and `thessa-flight-authority`: typed guidance,
  aircraft/spacecraft/direct control laws, policy limits, physical allocation,
  actuator dynamics, and the authoritative flight stepper;
- `thessa-autopilot` and `thessa-autopilot-js`: validated typed graph IR,
  sequence/parallel/wait execution, simulation-time scheduling, sandboxed
  QuickJS blocks, and typed trajectory-plan execution;
- `thessa-maneuver`: two-body planning helpers (circularization, Hohmann,
  Lambert, plane change, velocity matching) plus candidate search. Planning
  approximations are revalidated through the exact field before execution;
- `thessa-flight-net` and `thessa-protocol`: versioned framed input/snapshot
  transport with strict validation and ordered typed part commands;
- `apps/server`: headless authoritative simulation over stdio or TCP;
- `apps/client`: Bevy 0.19 map, pilot HUD, atmospheric and field-first plume
  rendering, terrain streaming, and an embedded authoritative-server path;
- `thessa-worldgen-rocky`: deterministic rocky-world fields, geology, climate,
  landmarks, LOD, obstacle reports, and client texture export;
- `thessa-rcbt-core`, `thessa-rcbt-ffi`, `thessa-rcbt-large-ffi`,
  `thessa-bevy-rcbt`, and `thessa-rcbt-wgpu`:
  backend-neutral adaptive terrain topology, a fast pure Rust implementation,
  optional verified upstream `libcbt` and `large_cbt` OCBT implementations,
  universal Bevy frame scheduling, compact height pages, and a portable GPU
  adapter. An experimental microscaled material-page codec and LOD/mip path is
  available for render-world A/B; it currently pre-decodes to RGBA and raw
  storage remains the default. The client keeps the CPU terrain renderer as the
  visual fallback while CBT topology is exercised against live terrain selection;
- isolated validation harnesses for orbital and aerodynamic reference checks.

Factory gameplay, structural fracture, thermal networks, save persistence, and
production multiplayer are still design/future work. The design documents keep
these areas explicitly marked as planned rather than presenting them as shipped.

## Physical and architectural invariants

- There is one gravity model: physical sources contribute simultaneously;
  sphere-of-influence boundaries are UI or optimization hints, never physics.
- The server is authoritative. Clients send commands and consume snapshots;
  they do not send arbitrary world state.
- `sim-core` is independent of Bevy, Tokio, networking, and renderer APIs.
- Authoritative spatial state uses `f64` SI values and `SimTime` rather than
  wall-clock time.
- Guidance produces physical demands. Control laws, policies, allocators, and
  actuators realize those demands; no hidden yaw/stability/drag multipliers or
  direct craft rotation are used to fake control authority.
- CPU geometry/BVH queries are the canonical server path. Hardware ray tracing
  may accelerate client work but is never the only authoritative path.
- Engine and reusable tooling crates are MIT; game-specific crates and apps are
  GPL-3.0-or-later. See [`LICENSING.md`](LICENSING.md).

## Repository layout

```text
apps/client/             GPL Bevy client
apps/server/             GPL authoritative server shell
crates/sim-core/         MIT compatibility facade for simulation-domain crates
crates/aero-core/        MIT atmosphere and aerodynamic models
crates/celestial/        MIT ephemerides, gravity fields, and simulation time
crates/trajectory/       MIT propagation and coast-cache algorithms
crates/propulsion/       MIT propulsion models and feed systems
crates/vehicle-core/     MIT rigid-body, vehicle, and mechanism models
crates/simd/             MIT optional numeric kernels
crates/fuselage/         MIT procedural body compiler
crates/plume-core/       MIT backend-neutral plume field
crates/flight-*          GPL flight authority, control, and transport
crates/autopilot*        GPL graph and JavaScript automation
crates/maneuver/         MIT trajectory planning primitives
crates/protocol/         MIT shared wire framing
crates/graphics/         MIT renderer-independent graphics settings
crates/atmosphere/       MIT shared visual atmosphere optics
crates/perf/             MIT performance capture model
crates/rcbt-*            MIT adaptive terrain topology and backend adapters
tools/                   MIT offline bakers and world generation
data/                    system, vehicle, aero, material, and worldgen inputs
docs/                    current architecture, design, validation, and ADRs
validation/              isolated external-reference workspaces
logs/                    intentionally preserved diagnostic captures
```

## Build and test

Rust `1.95+` is required by the workspace.

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets --all-features -- -D warnings
cargo test --workspace
cargo test -p thessa-sim-core --release
```

Build the checked-in system and vehicle descriptors:

```bash
cargo run -p thessa-system-baker -- \
  --input data/system.toml \
  --output data/system.baked.json

cargo run -p thessa-vehicle-baker -- \
  --input data/vehicles/example_aircraft.toml \
  --output /tmp/example_aircraft.baked.json
```

Run the client:

```bash
cargo run -p thessa-client
```

The client normally starts an embedded authoritative server process. Use
`--local` only when you specifically need the legacy in-process stepping path.
The headless server also supports stdio and TCP transports; run
`cargo run -p thessa-server -- --help` for the current CLI.

## Pilot controls

| Key | Action |
| --- | --- |
| `M` | Map / Pilot |
| `W/S` | pitch down / up |
| `A/D` | yaw left / right |
| `Q/E` | roll left / right |
| `Caps Lock` | precision control, 25% input |
| `Shift/Ctrl` | throttle up / down |
| `Z` / `X` | full throttle / cutoff |
| `Space` | engine on / off |
| `T` / hold `F` | toggle / invert SAS |
| `R` / `Y` | RCS / reaction wheels |
| `G` / `P` | landing gear / parachutes |
| `V` | free / follow camera |
| `` ` `` | reset camera |
| `Escape` / `F8` / `Pause` | pause |
| `F1` | controls and HUD legend |
| `F2` | hide / show UI |
| `F3` | extra telemetry |
| RMB / MMB / wheel | orbit / pan / zoom |

The pilot interface contract is documented in
[`docs/10_PILOT_INTERFACE.md`](docs/10_PILOT_INTERFACE.md).

## Validation and benchmarks

Orbital reference harness:

```bash
cargo run --manifest-path validation/nyx-compare/Cargo.toml --release
```

Aerodynamic comparison without external packages:

```bash
cargo run --manifest-path validation/aero-compare/Cargo.toml --release
```

Optional local JSBSim/RocketPy comparison:

```bash
python -m venv .venv-aero
.venv-aero/bin/pip install jsbsim rocketpy

THESSA_AERO_PYTHON=.venv-aero/bin/python \
  cargo run --manifest-path validation/aero-compare/Cargo.toml --release -- \
  --require-external
```

Selected benchmarks:

```bash
cargo bench -p thessa-sim-core --bench gravity
cargo bench -p thessa-sim-core --bench affine_prop
cargo bench -p thessa-sim-core --bench aero
cargo bench -p thessa-sim-core --bench flight
cargo bench -p thessa-maneuver --bench porkchop
```

The CI workflow runs formatting, Clippy, cross-platform builds/tests, release
simulation checks, benchmark compilation, and isolated reference validation.

## Documentation map

Start here:

1. [`docs/00_STATUS.md`](docs/00_STATUS.md) — living implementation/future-work
   index; start here for the current status of each major document.
2. [`docs/03_PHYSICS_ENGINE.md`](docs/03_PHYSICS_ENGINE.md) — physics contracts
   and implemented numerical paths.
3. [`docs/04_RUNTIME_ARCHITECTURE.md`](docs/04_RUNTIME_ARCHITECTURE.md) — current
   process, crate, threading, and transport boundaries.
4. [`docs/07_AUTOPILOT.md`](docs/07_AUTOPILOT.md) — current graph and scripting
   layer plus future standard-library work.
5. [`docs/08_NUMERICAL_VERTICAL_SLICE.md`](docs/08_NUMERICAL_VERTICAL_SLICE.md)
   — numerical validation and known limits.
6. [`docs/10_PILOT_INTERFACE.md`](docs/10_PILOT_INTERFACE.md) — pilot HUD and
   control contract.
7. [`docs/11_AERODYNAMICS.md`](docs/11_AERODYNAMICS.md) — aero model and
   reference matrix.
8. [`docs/details/README.md`](docs/details/README.md) — vehicle subsystem and
   authoring specifications, including implemented surface/body compilers,
   landing gear, reaction wheels, and parachutes.
9. [`docs/21_TERRAIN_STREAMING_THROUGHPUT.md`](docs/21_TERRAIN_STREAMING_THROUGHPUT.md)
   — current terrain streaming and CBT boundary.
10. [`docs/22_RCBT_GPU_TERRAIN.md`](docs/22_RCBT_GPU_TERRAIN.md) — CBT GPU
    boundary and replacement gates.
11. [`docs/42_EPHEMERIS_RESIDUAL_STORAGE.md`](docs/42_EPHEMERIS_RESIDUAL_STORAGE.md)
    — bounded residual compression for ephemeris/gravity caches.
12. [`docs/43_AERO_COEFFICIENT_RESIDUAL_STORAGE.md`](docs/43_AERO_COEFFICIENT_RESIDUAL_STORAGE.md)
    — bounded residual storage for large aerodynamic coefficient fields.
13. [`docs/05_ROADMAP.md`](docs/05_ROADMAP.md) — dependency-ordered future work.
14. [`CHANGELOG.md`](CHANGELOG.md) — notable changes.

ADRs live in [`docs/adr/`](docs/adr/). Design-only documents are labelled as
vision, proposal, or future work; implemented behavior is described in the
current implementation documents and source-level API comments.

## Rocky world generator

`thessa-worldgen-rocky` is part of the workspace. It generates deterministic
rocky-world fields and exports client textures; the current runtime contact
boundary remains spherical.

```bash
cargo run -p thessa-worldgen-rocky -- \
  check --manifest data/worldgen/thessa_demo.toml

cargo run --release -p thessa-worldgen-rocky -- \
  export-client-texture \
  --recipe data/worldgen/worldgen_recipe.toml \
  --body-file data/worldgen/thessa_v02.toml \
  --out /tmp/thessa-preview.png --width 2048
```

Flight instrument SVG sources and their 4x PNG exports are in
`assets/ui/flight/`; F1 includes their illustrated legend.

## Contributing

Read [`AGENTS.md`](AGENTS.md) and [`CONTRIBUTING.md`](CONTRIBUTING.md) before
changing simulation code. Physics changes should include a known-case test,
an error envelope against a decision-relevant scale, and a benchmark when the
change affects a hot path.
