# 04 — Runtime architecture

## Status

**Implemented prototype, with explicit future boundaries.** This document
describes the current workspace as of 2026-09-27. Sections labelled future do
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
crates/sim-core/          MIT aggregate compatibility facade
crates/aero-core/         MIT atmosphere and aerodynamic models
crates/celestial/         MIT ephemerides, gravity fields, and simulation time
crates/trajectory/       MIT propagation, gravity patches, and coast caches
crates/propulsion/       MIT propulsion models and feed systems
crates/vehicle-core/     MIT rigid-body, vehicle, and mechanism models
crates/simd/              MIT optional numeric kernels
crates/aero-surfaces/     MIT procedural lifting-surface authoring/compiler
crates/fuselage/          MIT procedural body compiler
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
crates/collision/         MIT Rapier contact backend (no Bevy dependency)
crates/bevy-rcbt/         MIT universal Bevy scheduling adapter for RCBT
                          (render-world bridge + material pages + presentation)
crates/plume-core/        MIT backend-neutral engine-plume field/medium model
crates/rcbt-core/         MIT backend-agnostic RCBT topology runtime
crates/rcbt-ref/          MIT independent RCBT topology oracle (tests)
crates/rcbt-ffi/          MIT optional vendored libcbt backend adapter
crates/rcbt-large-ffi/    MIT optional vendored large CBT backend adapter
crates/rcbt-wgpu/         MIT portable wgpu RCBT compute backend

apps/client/              GPL Bevy client (terrain, water, beauty, plume,
                          contact gizmos)
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
factory/logistics state, persistence/migrations, structural fracture and
vehicle-topology changes, fluid/resource networks, full electrical circuits
and docked-bus exchange, thermal/structural failure coupling, and an optional
web client. A lumped vehicle thermal network and an ideal shared electrical bus
already exist as bounded vehicle-system slices.

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
pilot input is latest-value-wins per client; edge commands, including the
version-5 `Command::Part` group and named subsystem commands, remain ordered.
The server decodes each input envelope once and dispatches payload
deserialization by its message kind. Client-observed tick fields are advisory;
state is applied on the next available authority step without rewind or
tick-based rejection. Valid non-unit SAS quaternions are normalized before use.
Keyboard/HUD part commands apply immediately to local prediction and are sent
to the authoritative server; legacy boolean state echoes cannot overwrite a
part command unless the echoed value actually changes. Leave cleanup is
retained even when an input queue is saturated. Outbound snapshots use a
latest-wins slot while reliable welcome/control frames stay ordered. One
encoded snapshot frame is Arc-shared across subscriber mailboxes; socket
writers consume the shared bytes without a per-client payload copy.

`FleetSnapshot` carries complete non-primary membership in ascending stable-ID
order on its own existing message kind. The server publishes it even when
empty; silence cannot acknowledge removal. The shared capacity is 64 vehicles
including PRIMARY (at most 63 secondaries), checked before a split mutates
authority. Primary and fleet outbound slots are independently latest-wins,
not an atomic common-epoch scene packet. Secondary simulation debt can differ.
The native embedded link demultiplexes these kinds with one reader and the same
bounded two-arrival inbox mechanism. Its read-only fleet inspector validates
each pose at that vehicle's own time; stale fleet observations are withheld,
not inferred to have despawned. Kinematics alone do not define appearance:
bounded retained meshes come from the revision-matched catalogue below, not
inference from the stable ID or name. Full designs/resources and
prediction/reconciliation remain absent. The old Bevy host still ignores fleet
observations.

Wire v9 extends `FleetGeometry` (kind 10, introduced in v8) and retains the exact
`geometry_revision` reference in `FleetSnapshot`; v7/v8 peers fail version
checking. The complete, sorted
non-primary catalogue contains bounded display labels and actual authoritative
contact geometry, including the recentered geometry of split clusters. It is
neither full `VehicleDefinition` replication nor a beauty mesh/model registry.
The server retains one encoded Arc-shared catalogue, compares existing source
parts without copying on ordinary snapshots, and publishes only on change or
to a new subscriber. Each mailbox retains one latest geometry slot plus its
last published revision, so draining does not cause periodic retransmission.
Coalescing can skip revisions. Readers keep independently bounded catalogue and
snapshot inboxes; an on-change catalogue waits for the short inbox lock instead
of being silently lost under contention. No extra reader or asset IO is added.

Only matching revisions and identical member IDs attach geometry to observations.
In-flight revisions show waiting, never a guessed model. Same-revision changes,
backwards revisions, invalid part poses/shapes/materials or over-budget received
catalogues are errors. The producer limits labels to 128 UTF-8 bytes (control
characters removed) and geometry to 64 contact parts per secondary. A definition
with more parts publishes explicit budget omission, not a truncated hull;
missing contact geometry is also explicit. These policies do not change physics.
The native inspector draws a diagnostic auto-fit contact wireframe, not an
appearance proxy. Geometry storage is Arc-shared through the HUD rather
than deep-copied per frame; `--no-hud` omits projection/UI work. Stale membership
still withholds all rows. The same catalogue also carries optional retained
appearance, limited to 8 parts / 1,024 vertices / 4,096 indices per secondary;
part origins and local vertices are bounded to ±1,024 m per component. Oversized
sources are explicitly omitted, never partly rendered or replaced by a contact
hull/X-15. Authority resolves/remaps assembly ownership; the display catalogue
strips ownership indices and retains actual meshes and body-local rigid poses.
Source comparison uses Arc identity before value comparison, with no periodic
mesh buffer copy/encode. No paths, loaders, textures or full definitions arrive.

Native combines admitted appearance and the compatibility PRIMARY source on
render-layout change in the existing renderer/device/targets/pipeline/pass. Static
source replacement currently reuploads PRIMARY as well as admitted shells;
it does not claim an incremental GPU residency/asset registry. Ordinary frames
update only matrices/view/lighting and per-primitive instance ranges. A second
two-pose presentation history exists only for available world meshes, not
physics or authoritative prediction. It samples inertial linear positions and
slerped attitudes at PRIMARY's scene epoch, subtracting the ephemeris there;
without a bracket/exact sample it withholds that world mesh. Render-layout changes
reset histories; label/contact-only changes and common part-origin translations
preserve them. For an unchanged layout, snapshots are transformed to the retained
publication's body frame before interpolation. The first part identifies the
common translation; its offset is not used as an interpolation origin, avoiding
an artificial change to the arc of a rotating body. Mesh/normal/index or part
rotation/relative-position changes rebuild the combined source, including PRIMARY.
Relative-position equality permits a 1 nm subtraction envelope against the
retained publication, never the preceding packet, so errors cannot accumulate.
Narrowing reserves that envelope within the existing 1 mm local vertex budget.
Observer rows still show their own authoritative times.
Changing a part origin, including a resource-driven COM shift, revises the whole
catalogue even when shared vertex buffers are unchanged; an incremental network
mesh dictionary and general incremental GPU residency remain future work. COM-only
translations no longer cause GPU source replacement, but still retransmit meshes.
Stale/revision-mismatched membership withholds draws, never invents despawn.

`renderer.fleet_meshes=false` allocates/uploads no fleet GPU source or track
history. Enabled Low/Medium/High resolutions choose 8/2/0.5-pixel estimated
angular-footprint culling through `ResolvedGraphicsSettings`, with identical
mesh geometry. The basic opaque triangle path needs no optional GPU feature;
there is no hidden device or global architecture switch. Shells use an explicitly
neutral untextured finish, not authored/PBR material parity. Counters expose
source publications, out-of-epoch withholding, size culls and submitted mesh
instances/draws/triangles/upload bytes. Complete resources, render designs,
PRIMARY shape replacement and production prediction/reconciliation remain gaps.

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

The implemented `VehicleDefinition`/`vehicle-baker` path compiles serializable
vehicle assets with mass/inertia, lofted bodies, aero panels/body controls,
propulsion mounts, articulated landing gear, reaction-wheel banks, parachute
packs, a shared electrical bus with batteries, reactors, solar arrays, and
prioritized loads, and a lumped thermal-node network with links and radiators.
The active flight slice uses this compiled definition and the X-15 adapter.
The full parametric in-game editor is still future work.

The full parametric editor, shared immutable design storage for large fleets,
structural graph compilation, grid-resolved thermal coupling, shield
thermal-protection/ablation coupling, and fluid connectivity remain future
work. Electrical load allocation is
implemented as a wire-free shared bus; voltage/current dynamics and automatic
demand coupling for all powered vehicle subsystems remain future work.

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
