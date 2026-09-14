# AGENTS.md

Rules for code and design agents working on Project Thessa.

## 1. Main invariant

**Do not replace physical causality with game coefficients when the effect can
be obtained from geometry, material, a field, or an actuator.**

Examples:

- forbidden: `yaw_power`, `magic_drag`, `stability_bonus`, or directly rotating
  the craft;
- allowed: control-surface area, hinge position, actuator moment, local flow,
  force, lever arm, and an FBW allocator;
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

- no DirectX-specific API in domain, simulation, or gameplay code;
- shaders and render data must not be designed around DX-only semantics;
- the native rendering boundary is Bevy/wgpu;
- supported backend families are Vulkan, Metal, WebGPU, and the backend chosen
  by wgpu on a supported platform;
- wgpu choosing D3D12 internally on Windows does not make DirectX part of the
  architecture API; direct DX12/DXR calls require a separate ADR and are
  forbidden by default;
- Linux is a first-class development and runtime target, not a post-Windows
  port;
- the client layer must regularly be compile-checked for WASM/WebGPU after a
  web target exists.

### Bevy client

Bevy owns:

- rendering and wgpu;
- input;
- UI;
- assets;
- client ECS;
- debug gizmos and tooling;
- visual interpolation and extraction.

`bevy::Transform` is never an authoritative astronomical coordinate.

### Server

- Tokio owns networking, async I/O, persistence orchestration, and
  administration/metrics;
- Rayon or a custom fixed worker pool owns CPU-heavy simulation batches;
- a CPU-heavy solver is not launched as an ordinary `tokio::spawn` future;
- the simulation tick has one authoritative phase order.

## 3. Celestial mechanics

- stars, planets, moons, and canonical minor bodies follow baked deterministic
  ephemerides;
- runtime does not integrate their mutual dynamics;
- ships, stations, debris, and temporary objects receive the sum of gravity
  from all relevant bodies;
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

One connected structural cluster may be integrated as one rigid body while that
approximation remains valid. Moving hinges and doors may have explicit DOFs.
When the structural graph fails, split it into connected components and make
each component a separate cluster/body.

## 5. Aerodynamics and control

- compute forces locally for aerodynamic zones or panels;
- local velocity includes translational velocity, atmospheric motion, and
  `omega x r`;
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

Physics ray tracing means **geometric ray/BVH queries**, not necessarily
DXR/Vulkan RT.

The canonical server path must work without a GPU. Hardware RT or compute may
accelerate client/local calculations but must not be the only way to obtain an
authoritative result.

## 8. Multiplayer and warp

- the server is authoritative;
- clients send inputs/commands, not arbitrary world state;
- the controlling client may predict its own craft;
- other craft interpolate snapshots;
- warp is one server simulation time scale;
- warp changes follow a consensual/shared policy;
- alarm and autopilot event schedulers operate in simulation time.

## 9. Autopilot scripts

The UX reference is MechJeb-like ready operations (`Ascent Guidance`,
`Maneuver Planner`, `Landing Guidance`, `Rendezvous`, `Docking`, and attitude
helpers), but they are not isolated modes in Thessa. Every high-level action
is a typed block with inputs and outputs that can be nested in a reusable graph
or subprogram.

Required combinators:

- sequence;
- condition or switch;
- wait/event;
- loop/retry/fallback;
- parallel/fork/join;
- reusable parameterized subgraph;
- explicit abort/failure path.

`stage/separate` may create multiple `VehicleId` values; the graph must be
able to route different control branches to the booster and upper stage.

Prefer an event-driven VM:

- `WAIT UNTIL` compiles to a wake condition;
- sleeping programs are not polled every physics tick;
- high-level blocks (`point prograde`, `land at pad`, `target orbit`) use the
  standard guidance/control systems;
- low-level sensors and actuators remain available to advanced programs;
- the same script layer serves logistics and flight automation.

## 10. Performance

Optimize representation, not physical laws. Where the effect is demonstrably
negligible under the policy in §10.1, reduce the model instead of integrating
noise.

### 10.1. Bounded model reduction

The main invariant forbids replacing causality with coefficients when the
effect changes a decision (trajectory, control, or failure). Where an error
envelope is demonstrably bounded, reduction is allowed and preferred,
especially for large fleets. Every reduction needs:

- an explicit config boundary (density, altitude, or mode), not a magic number;
- calibration against the full model;
- an absolute error envelope against a decision-relevant scale (weight, thrust,
  or batch tolerance), not a misleading relative error;
- a regression test pinning the envelope;
- a benchmark showing the gain.

Current reductions include:

- `vacuum_cutoff_density_kg_m3`: below the threshold, the medium is declared
  vacuum exactly and vacuum fast paths/rails batches may run;
- upper-band drag (`cutoff < rho < COAST`) uses reference-area drag instead of
  the panel loop and sets aerodynamic moment to zero where RCS dominates;
- batches run only in declared vacuum and cap the coast jump at
  `MAX_COAST_BATCH_JUMP_S` for responsiveness and wake latency;
- batch proximity checks sample the exact ephemerides at five craft/body
  points and revalidate the current state within 5 m before each jump.

Prefer:

- SoA/AoSoA for mass numeric kernels;
- batch gravity, aero, atmosphere, and thermal evaluation;
- AVX2 as the native baseline and optional AVX-512 builds;
- adaptive rates/steps for different modes;
- expensive contacts only where actual contacts exist;
- profiling and benchmarks before hand-written intrinsics.

## 11. Licenses

Until the project license decision changes:

- do not add AGPL dependencies to runtime;
- LGPL Rust dependencies require an explicit decision;
- do not copy code from `nyx-space` or `avian_fdm` into the project;
- their documentation and algorithmic ideas may be used as references, with an
  independent implementation and primary sources.

## 12. Licensing boundary

- engine and reusable crates: SPDX `MIT`;
- game apps and game-specific crates: SPDX `GPL-3.0-or-later`;
- do not move GPL-only game code into MIT engine crates;
- preserve generated-code and asset licenses explicitly;
- see [`LICENSING.md`](LICENSING.md).

## 13. Definition of done for a physics feature

A physics feature is not complete without:

1. a formal description of state, inputs, and outputs;
2. a known-case test;
3. a regression case;
4. a numerical error estimate;
5. a benchmark at target batch size;
6. debug visualization or telemetry when the effect cannot otherwise be
   inspected directly.
