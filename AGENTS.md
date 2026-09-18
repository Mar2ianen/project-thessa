# AGENTS.md

Rules for code and design agents working on Project Thessa.

## 0. Documentation language

All project documentation is written in English.

This includes `docs/`, ADRs, architecture/design notes, agent instructions, README-style design prose, and new documentation embedded in configuration examples. Existing non-English documentation should be translated when it is materially edited; do not introduce new non-English documentation.

## 1. Main invariant

**Do not replace physical causality with game coefficients when the effect can be obtained from geometry, material, a field, or an actuator.**

Examples:

- forbidden: `yaw_power`, `magic_drag`, `stability_bonus`, or directly rotating the craft;
- allowed: control-surface area, hinge position, actuator moment, local flow, force, lever arm, and an FBW allocator;
- forbidden: teleporting cargo between spaceports;
- allowed: physical transport plus schedules, buffers, and warp.

## 2. Module boundaries

### `sim-core`

- owns authoritative simulation state;
- has no dependency on Bevy, Tokio, Lightyear, or Avian;
- may use reviewed math, serialization, or data-parallel crates;
- stores authoritative spatial state as `f64`;
- uses SI as the canonical unit system;
- uses `SimTime` for physics time, never `Instant` or `SystemTime`.

## 2.1. Cross-platform invariant

- no DirectX/Vulkan/Metal/wgpu-specific API in domain, simulation, or gameplay code;
- shaders and render data must not be designed around DX-only semantics;
- current native client integration is Bevy + wgpu, but reusable GPU algorithm cores must not expose Bevy/wgpu types as their semantic API;
- preferred layering for reusable GPU subsystems is: backend-agnostic core -> portable wgpu backend -> optional measured native backend -> Bevy adapter;
- supported backend families are Vulkan, Metal, WebGPU, and the backend selected internally by wgpu on a supported platform;
- a direct Vulkan backend is allowed inside an isolated renderer/GPU subsystem after profiling; it must not change simulation/gameplay APIs;
- wgpu choosing D3D12 internally on Windows does not make DirectX part of the architecture API; direct DX12/DXR calls require a separate ADR and are forbidden by default;
- capability checks are preferred over vendor checks (`if AMD/NVIDIA/...`);
- Linux is a first-class development and runtime target, not a post-Windows port;
- the portable client/render path must be regularly compile-checked for WASM/WebGPU after a web target exists.

### Bevy client

Bevy owns the current client integration:

- rendering integration / default wgpu backend;
- input;
- UI;
- assets;
- client ECS;
- debug gizmos and tooling;
- visual interpolation and extraction.

Bevy does not own the semantic API of reusable renderer algorithms. If a subsystem can be reused outside Bevy (`rcbt`, compression/streaming kernels, plume fields, etc.), its core types/traits must not accept `Entity`, `RenderWorld`, `wgpu::Device`, or similar integration handles as domain API.

`bevy::Transform` is never an authoritative astronomical coordinate.

## 2.2. Graphics feature policy

Every user-visible graphics effect must be integrated through the shared graphics-settings pipeline and must have explicit quality/fallback levels.

Required rules:

- every effect is represented in `crates/graphics` requested/resolved settings and is configurable through `graphics.toml`; do not hide production graphics features behind ad-hoc environment variables, hard-coded local toggles, or plugin-presence side effects;
- the normal flow is `RequestedGraphics -> capability resolution -> ResolvedGraphicsSettings -> renderer`; effect systems consume resolved settings instead of bypassing the resolver;
- every effect has an explicit disabled state and at least one cheaper fallback/quality level; where appropriate use the common `Low` / `Medium` / `High` quality vocabulary, with additional capability-driven backends behind the same semantic effect;
- quality levels change representation accuracy, sample count, resolution, residency budget, update cadence, or backend choice; they must not change authoritative physics or invent/remove physically caused phenomena merely because quality is lower;
- capability checks choose what can run; quality/policy chooses what should run. Do not infer capabilities from OS or GPU vendor names;
- explicitly forced unsupported modes may fail fast with a clear error; automatic/default modes must degrade to a supported fallback;
- `enabled = false` must make the effect effectively free: no meaningful per-frame work, hidden prepasses, global renderer-mode changes, large persistent allocations, or unrelated side effects;
- merely registering a graphics plugin must not silently switch global render architecture. Expensive prerequisites are enabled only when the resolved effect/backend needs them;
- a renderer-specific mesh, texture, particle system, light, or acceleration structure is a consumer of semantic effect state, not the semantic source, when the phenomenon has a backend-neutral model;
- new effects must document their quality tiers, fallback behavior, capability requirements, and performance counters/budget;
- reusable effect cores follow the layering rule in §2.1 and keep Bevy/wgpu/Vulkan handles out of their semantic API.

For the engine-plume reference architecture, see `docs/38_ENGINE_PLUME_RENDERING.md`.

## 3. Celestial mechanics

- stars, planets, moons, and canonical minor bodies follow baked deterministic ephemerides;
- runtime does not integrate their mutual dynamics;
- ships, stations, debris, and temporary objects receive the sum of gravity from all relevant bodies;
- SOI switching is never part of physics;
- SOI may exist only as a UI or optimization hint;
- gravity harmonics are evaluated in a body-fixed frame;
- Lagrange points are not hard-coded: they emerge from fields and ephemerides.

## 4. Vehicles

`VehicleDesign` should compile at least into:

- render representation;
- collision representation;
- aerodynamic zones or panels;
- structural graph;
- thermal graph;
- mass and inertia model;
- actuator graph;
- fluid and electrical connectivity.

Do not store hundreds of fixed parts only to aggregate them back later.

### Rigid-body policy

One connected structural cluster may be integrated as one rigid body while that approximation remains valid. Moving hinges and doors may have explicit DOFs. When the structural graph fails, split it into connected components and make each component a separate cluster/body.

## 5. Aerodynamics and control

- compute forces locally for aerodynamic zones or panels;
- local velocity includes translational velocity, atmospheric motion, and `omega x r`;
- control surfaces have hinge geometry, limits, rate, and actuator torque;
- an actuator may fail to reach a commanded deflection under aerodynamic load;
- FBW emits actuator commands and does not add a craft moment directly;
- low-level/manual control must remain possible.

## 6. Temperature and failure

- temperature is not reduced to one value for the whole craft;
- thermal nodes are connected by conductance and radiation edges;
- aerodynamic, engine, and solar heating enter the same thermal graph;
- material strength may depend on temperature;
- damage must not automatically mean explosion or despawn;
- structural failure changes topology, mass, inertia, aero, and thermal graphs.

## 7. Ray queries

Physics ray tracing means **geometric ray/BVH queries**, not necessarily DXR/Vulkan RT.

The canonical server path must work without a GPU. Hardware RT or compute may accelerate client/local calculations but must not be the only way to obtain an authoritative result.

## 8. Multiplayer and warp

- the server is authoritative;
- clients send inputs/commands, not arbitrary world state;
- the controlling client may predict its own craft;
- other craft interpolate snapshots;
- warp is one server simulation time scale;
- warp changes follow a consensual/shared policy;
- alarm and autopilot event schedulers operate in simulation time.

## 9. Autopilot scripts

The UX reference is MechJeb-like ready operations (`Ascent Guidance`, `Maneuver Planner`, `Landing Guidance`, `Rendezvous`, `Docking`, and attitude helpers), but they are not isolated modes in Thessa. Every high-level action is a typed block with inputs and outputs that can be nested in a reusable graph or subprogram.

Required combinators:

- sequence;
- condition or switch;
- wait/event;
- loop/retry/fallback;
- parallel/fork/join;
- reusable parameterized subgraph;
- explicit abort/failure path.

`stage/separate` may create multiple `VehicleId` values; the graph must be able to route different control branches to the booster and upper stage.

Prefer an event-driven VM:

- `WAIT UNTIL` compiles to a wake condition;
- sleeping programs are not polled every physics tick;
- high-level blocks (`point prograde`, `land at pad`, `target orbit`) use the standard guidance/control systems;
- low-level sensors and actuators remain available to advanced programs;
- the same script layer serves logistics and flight automation.

## 10. Performance

Optimize representation, not physical laws. Where an effect is demonstrably negligible under the policy in §10.1, reduce the model instead of integrating noise.

For low-level CPU/GPU code, performance is part of the architecture, but unsafe/layout/platform tricks are allowed only with a benchmark and a clear fallback. Idiomatic Rust is not a goal when it measurably blocks throughput; a dirty layout trick is not a goal when counters do not show a problem.

## 10.1. Bounded model reduction

The main invariant forbids replacing causality with coefficients where the effect changes a decision such as trajectory, control, or failure. Where the influence is demonstrably bounded by an error envelope, reduction is allowed and preferred, especially for large fleets.

Every reduction needs:

- an explicit config boundary such as density, altitude, or mode, not a magic number in code;
- calibration of reference parameters against the full model;
- an absolute error envelope against a decision-relevant scale such as weight, thrust, or batch tolerance, rather than a misleading relative error near zero;
- a regression test pinning the envelope;
- a benchmark showing the gain.

Current reductions include:

- `vacuum_cutoff_density_kg_m3`: below the threshold, the medium is declared vacuum exactly and exact-vacuum fast paths / rails batches may run;
- upper-band drag (`cutoff < rho < COAST`) uses reference-area drag instead of the panel loop and sets aerodynamic moment to zero where RCS dominates by orders of magnitude;
- batches run only in declared vacuum and cap the coast jump at `MAX_COAST_BATCH_JUMP_S` for responsiveness and wake latency;
- batch proximity checks sample exact ephemerides at five craft/body points rather than a chain-speed worst-case margin, and revalidate the current state within 5 m before each jump.

Prefer:

- SoA/AoSoA for mass numeric kernels;
- batch gravity, aero, atmosphere, and thermal evaluation;
- AVX2 as the native baseline and optional AVX-512 builds;
- adaptive simulation rates/steps for different modes;
- expensive contacts only where actual contacts exist;
- profiling and benchmarks before hand-written intrinsics;
- packed bitsets/bitplanes and word-wise operations for topology/state machines when they improve the measured workload;
- checking false sharing separately: per-thread hot mutable state may be cache-padded/aligned, but padding is not a blanket rule;
- `#[repr(C)]` for FFI/GPU ABI/layout contracts, not as a universal performance annotation;
- `#[repr(align(N))]` or padded wrappers only with a benchmark on the target workload and an explicit working-set cost assessment;
- separating hot read-only state from frequently written counters/queue heads where possible;
- comparing every native GPU fast path against the portable backend with identical telemetry.

## 10.2. Reusable GPU algorithms

For `rcbt` and future reusable GPU subsystems:

- the core semantic API does not depend on Bevy/wgpu/Vulkan;
- backend traits describe operations/capabilities, not thin wrappers around `Device` / `CommandEncoder`;
- wgpu/WGSL is the portable default;
- a native Vulkan/SPIR-V path is allowed as an optional backend for a measured reason;
- subgroup/bit operations are preferred over artificial use of matrix units for bit-tree work;
- cooperative/matrix hardware may be used by an optional decoder for compressed data only after a dedicated benchmark;
- an upstream/reference implementation may be bound as an oracle/benchmark baseline, but production does not need to preserve its internal layout;
- the performance target may be more aggressive than the reference implementation; observable semantics matter more than internal compatibility;
- see `docs/22_RCBT_GPU_TERRAIN.md`.

## 11. Licenses

Until the project license decision changes:

- do not add AGPL dependencies to runtime;
- LGPL Rust dependencies require an explicit decision;
- do not copy code from `nyx-space` or `avian_fdm` into the project;
- their documentation and algorithmic ideas may be used as references with an independent implementation and primary sources.

## 12. Definition of done for a physics feature

A feature is not done without:

1. a formal description of state/inputs/outputs;
2. a known-special-case test;
3. a regression case;
4. a numerical error estimate;
5. a benchmark on at least the target-size batch;
6. debug visualization/telemetry when the effect cannot otherwise be verified directly.

## 13. Licensing boundary

- engine/reusable crates use SPDX `MIT`;
- game apps/game-specific crates use SPDX `GPL-3.0-or-later`;
- do not move GPL-only game code into MIT engine crates;
- permissive dependencies are preferred for engine crates; a copyleft dependency in an engine crate requires an ADR analyzing redistribution/linking boundaries;
- generated code and asset licenses remain explicit;
- see `LICENSING.md`.
