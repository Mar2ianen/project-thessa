# 04 — Runtime architecture

## Status

**Implemented prototype, with explicit future boundaries.** This document
describes the current workspace as of 2026-09-14. Sections labelled future do
not describe a shipped subsystem.

## 4.0. Cross-platform contract

Cross-platform support is an architectural constraint, not a post-release port.

- simulation, protocol, persistence, and gameplay code do not know DirectX;
- the native renderer boundary is Bevy/wgpu;
- Linux/Vulkan, macOS/Metal, and browser/WebGPU remain first-class targets;
- Windows may use the backend selected internally by wgpu, including D3D12,
  but no DirectX types or calls cross the renderer boundary;
- physics ray/BVH queries have a CPU path and do not require DXR;
- WGSL and Bevy render abstractions are preferred to platform shader forks;
- platform-specific optimization requires an adapter/feature boundary and a
  benchmark.

A future Vulkan-only Windows packaging policy would require a separate ADR and
would not change the simulation API.

## 4.1. Current process split

```text
┌──────────────────────────────┐
│ apps/client                  │
│ Bevy render, input, UI, map  │
└───────────────┬──────────────┘
                │ framed commands/snapshots
                ▼
┌──────────────────────────────┐
│ apps/server                  │
│ authoritative driver         │
│ Tokio transport/orchestration│
│ dedicated simulation thread  │
└───────────────┬──────────────┘
                ▼
       flight-authority
                ▼
   sim-core / flight-control
```

The normal client path starts a separate local server process and exchanges
framed messages over stdio. The server also has a TCP transport. `--local`
keeps an in-process legacy path for diagnostics; it is not the architectural
authority model.

## 4.2. Current workspace boundary

```text
crates/sim-core/          MIT numerical state, time, gravity, aero, flight
crates/simd/              MIT optional numeric kernels
crates/atmosphere/        MIT shared atmosphere optics
crates/graphics/          MIT graphics settings resolution
crates/perf/              MIT performance capture model
crates/protocol/          MIT wire envelope and framing
crates/flight-control/    GPL guidance, control laws, policy, allocation
crates/flight-authority/  GPL authoritative vehicle runtime adapter
crates/flight-net/        GPL game input/snapshot messages
crates/autopilot/         GPL typed graph IR and runner
crates/autopilot-js/      GPL sandboxed QuickJS blocks
crates/maneuver/          MIT typed maneuver planning prototype

apps/client/              GPL Bevy client
apps/server/              GPL headless authoritative shell

tools/system-baker/       MIT system TOML to baked JSON
tools/vehicle-baker/      MIT vehicle TOML to baked JSON
tools/worldgen-rocky/     MIT offline rocky-world generator
```

`validation/nyx-compare` and `validation/aero-compare` are separate Cargo
workspaces. Their reference dependencies do not enter the root runtime graph.
The RCBT boundary is intentionally different: `thessa-rcbt-core` is the
portable pure-Rust runtime implementation, while `thessa-rcbt-ffi` is an
optional vendored `libcbt` backend that may be selected by a client or tool
that accepts its single-threaded handle and depth limit. Neither backend is
required by the authoritative server simulation.

The following boundaries are future work rather than missing hidden crates:
factory/logistics state, persistence/migrations, structural fracture, thermal
networks, fluid/electrical networks, and an optional web client.

## 4.3. Threading

### Server

```text
Tokio runtime
  ├ stdio/TCP receive and send
  ├ connection lifecycle
  └ async orchestration

Authoritative driver thread
  ├ input coalescing and ordered edge events
  ├ simulation-time waits and autopilot execution
  ├ flight authority tick
  └ bounded worker/bake requests
```

CPU-heavy flight and gravity work stays in the authority/core path or in a
bounded worker operation. Do not spawn one blocking task per aero job.

The current server loop uses a wall-time driver quantum to service inputs and
snapshots. That quantum is a responsiveness budget, not a physics-rate limit.
Requested warp is capped and the effective warp reports work actually served;
on-rails batches provide the high-throughput path.

### Client

Bevy owns render/client ECS schedules, input, visual interpolation, map state,
terrain streaming, atmosphere presentation, and the HUD. Background previews
may use Bevy task pools, but authoritative numerical state is not owned by
`bevy::Transform` or a render task.

## 4.4. Authoritative tick and time

The current authority advances `SimTime` on a fixed 120 Hz lattice for active
flight. A bounded duration can contain deterministic substeps. The driver
services commands, schedules, and snapshots around those steps; it does not
replace them with wall-clock integration.

The conceptual order is:

```text
1. accept and validate ordered client commands
2. wake simulation-time graph/script/plan waits
3. resolve guidance and flight policy
4. allocate physical control demands to actuators
5. advance flight dynamics and contacts
6. update on-rails/terrain wake state and bake requests
7. commit events and snapshot state
```

Some subsystems still have their own internal phase details. New phases must
preserve one deterministic authority order.

## 4.5. Warp and on-rails scheduling

`SimTime` is independent of `Instant`. Warp is a shared server policy. When
multiple clients vote, the effective value is constrained by the most
restrictive vote; a pause vote pauses the simulation.

The authority may switch an unpowered, vacuum-safe craft to a cached sampled or
piecewise analytic path. The path has explicit horizon, contact, atmosphere,
obstacle, and wake conditions. It is invalidated by control/thrust or other
state changes and revalidated before a jump. A failed or expired approximation
falls back to exact stepping; it must not silently continue outside its bound.

Autopilot waits use simulation time or named domain events. Sleeping programs
are held in a scheduler and are not polled at every physics tick.

## 4.6. Networking and authority

The client sends validated commands and guidance/autopilot submissions. It does
not submit arbitrary craft transforms. The server owns craft state, time warp,
script continuations, plan cursors, and event order.

Snapshots use a versioned envelope and bounded framed transport. Continuous
pilot input is latest-value-wins per client; edge commands remain ordered, and
leave cleanup is retained even when an input queue is saturated. Outbound
snapshots use a latest-wins slot while reliable welcome/control frames stay
ordered.

The current protocol is designed for a local/server prototype. Production
prediction, interpolation policy for remote craft, authentication, persistence,
and fleet replication tiers are future work.

## 4.7. Rendering coordinates

Authoritative spatial values remain `f64` SI. The client maps them into an
anchored render-local frame near the camera or selected body. The render origin
may move and visual radii may be exaggerated for readability, but neither
`Transform` nor the HUD feeds coordinates back into physics.

Terrain and atmosphere rendering consume interpolated visual poses. Telemetry,
control, and contact decisions use authority snapshots/state.

## 4.8. Persistence

No production save database or migration system is implemented yet. Current
persisted-like inputs are checked-in TOML/JSON design data and diagnostic
captures. A future save layer must version formats, keep simulation time
explicit, and preserve the MIT/GPL boundary.

## 4.9. Vehicle design and duplication

The implemented `VehicleDefinition`/`vehicle-baker` path compiles a serializable
vehicle asset with mass/inertia, geometry, aero panels, control surfaces, and
starter propulsion/control data. The active flight slice uses this compiled
definition and the X-15 adapter.

The full parametric editor, shared immutable design storage for large fleets,
structural graph compilation, thermal graph compilation, and fluid/electrical
connectivity remain future work.

## 4.10. Autopilot and planning boundary

`thessa-autopilot` defines typed graph values, ports, graph validation,
sequence/parallel/wait behavior, and failure/abort paths. `thessa-autopilot-js`
produces those values through a capability-restricted QuickJS host. The server
owns continuations and wakes them from `SimTime` or domain events.

`thessa-maneuver` produces typed plans and plain physical guidance commands. A
two-body planner is allowed to prune/search candidates, but plan execution
revalidates against the exact multi-body field and realizes burns through the
authority/allocator path. No planner teleports velocity or craft state.

## 4.11. Build targets

The root workspace is tested on the CI matrix documented in `.github/workflows`.
Native Linux is the development baseline. Windows/macOS builds are supported
through Bevy/wgpu. A WASM/WebGPU client target is not yet part of the workspace;
it must be added with compile checks before being called shipped.

## 4.12. Dependency and license policy

Reusable engine/tooling packages are MIT. Game-specific control, automation,
network, authority, client, and server packages are GPL-3.0-or-later. Reference
solvers remain in isolated validation workspaces. See `LICENSING.md` and the
ADR index before adding a dependency. The optional `libcbt` adapter is the
explicit exception for RCBT: it is kept in the root workspace as a selectable
runtime backend, with the foreign code and its boundary documented separately.

## 4.13. Observability

`thessa-perf` records frame, CPU scope, GPU availability, memory, world, server,
and on-rails metrics. The client can export JSON/CSV captures and the server
reports effective warp separately from requested warp. Physics reductions expose
error/coverage information in tests and benchmark reports.

## 4.14. What this document does not promise

It does not claim that the complete game loop, factory, thermal/structural
failure, production persistence, internet multiplayer, or web client already
exists. Those systems belong in the roadmap and design documents until their
crate/API/test paths land in the workspace.
