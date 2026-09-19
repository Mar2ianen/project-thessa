# Project Thessa — Bevy Exit and Engine Migration Plan

**Status:** Active migration  
**Language:** English  
**Updated:** 2026-09-18  
**Review snapshot:** repository architecture known through the full-stack merge (`main` past `40aed83`), plus renderer/RCBT/Vulkan decisions made through 2026-09-16.

> This document is intentionally operational. The goal is to stop adding Bevy-specific work that will have to be rewritten during the engine extraction.
>
> Carve-out: the `docs/38` indexed-terrain repairs are disposable-bridge work
> (shared WGSL/geometry and material-ancestor mapping contain no Bevy API;
> acknowledgement belongs to the bridge) — see §38:249-252.

## 1. Decision

Bevy is no longer a long-term architectural foundation for Project Thessa.

It is now treated as a **bootstrap host** while Thessa grows its own engine through a strangler-fig migration:

1. Keep the game runnable through Bevy.
2. Move engine-owned subsystems behind Thessa-owned APIs.
3. Replace Bevy subsystems one at a time.
4. Reduce Bevy to a compatibility/host bridge.
5. Remove the bridge when no engine subsystem depends on it.

The primary invariant is:

> **New engine code must not require a rewrite solely because Bevy is removed.**

This is not a rewrite project. Existing working code remains usable until a replacement exists.

---

## 2. Review of current systems and Bevy coupling

This review is based on the current architecture known from the repository snapshot and the subsequent renderer work.

| System | Current coupling | Assessment | Required direction |
| --- | --- | --- | --- |
| `sim-core` / gravity / orbital simulation | Low / none | Good boundary already. Simulation is conceptually independent of rendering. | Keep engine-agnostic. Do not introduce Bevy ECS types into the simulation API. |
| Headless/server-side simulation | Low | Good candidate for the canonical simulation runtime. | Keep using the same simulation crates as the client. No renderer/window dependencies. |
| Client app bootstrap | High | Bevy currently owns lifecycle, world, scheduling and renderer startup. | Reduce this to `bevy_bridge`; application logic must move behind Thessa-owned APIs. |
| Terrain cube-sphere / tile streaming | Medium–high | Core policy is Thessa-specific, but the current mesh/entity/material delivery path is tied to Bevy. | Split topology/generation/streaming from presentation. Renderer backends consume neutral terrain descriptors. |
| Terrain cache/jobs/wanted/visible state | Medium | The state model is reusable; execution and delivery are the likely coupling points. | Move jobs to Rayon and keep cache/state structures free of Bevy task/entity types. |
| RCBT / CBT topology | Low by design, if kept separate | This should become one of the cleanest reusable engine subsystems. | Keep `rcbt-core` backend-neutral. GPU support lives separately. Bevy integration must remain a thin adapter. |
| Procedural mesh generation | Medium–high | Current path is likely shaped around Bevy `Mesh`/asset upload semantics. | Define Thessa-native procedural output and GPU-ready layouts; convert to Bevy only in the bridge. |
| `thessa-atmosphere` / atmosphere logic | Medium | Physical/stellar logic is reusable; render integration and shader resources are Bevy-specific. | Separate physical atmosphere state from backend resources/passes. Multiple stars are a core invariant. |
| Stellar irradiance / angular size / eclipses | Low conceptually | This is already a Thessa world-model feature, not a Bevy feature. | Keep as engine/world data; renderer consumes `StarEmitter[]`. Never encode a single-Sun assumption. |
| Atmosphere shader path | High | Shader registration, resources and render passes are tied to the current renderer path. | Move under `thessa-render`; expose Vulkan and wgpu implementations behind the same engine-owned contract. |
| Auto exposure / temporal render state | High | Usually tied to Bevy render graph/resources. | Move into renderer-owned passes/resources. |
| Solari | Very high | Solari dictates RT scene construction and lighting architecture, and its cost model does not match Thessa. | Freeze integration. Use only as temporary/reference implementation while FastRT is built. |
| Ray tracing scene / BLAS / TLAS | High | Currently inherited from Solari/Bevy. | Move to a standalone RT infrastructure layer owned by Thessa. |
| Temporal AA / upscaling | High | Existing Bevy/DLSS plumbing is useful as reference but should not define the engine API. | Create a backend-neutral temporal contract; DLSS/OSS/native TAAU are implementations. |
| wgpu GPU resources | High where exposed | wgpu is useful as a portability backend but its memory model is too restrictive for the reference path. | Do not expose `wgpu::*` outside backend/bridge code. |
| Vulkan | New primary backend | Needed for explicit memory topology, RT control and UMA fast paths. | Treat native Vulkan as the reference/high-performance backend. |
| UMA shared buffers | Not expressible well enough through current high-level path | Important for APU/laptop performance and procedural workloads. | Add shared/persistently mapped allocations to the engine memory API. |
| Bevy Tasks | Replaceable | No reason to preserve them as an engine dependency. | Rayon becomes the CPU execution engine; Tokio remains for async I/O/network work. |
| Bevy ECS | Replaceable | Once renderer/tasks/assets leave Bevy, retaining Bevy only for ECS has little value. | Introduce a Thessa ECS/scheduler later in the migration, after subsystem boundaries stabilize. |
| Bevy Assets | Medium | Useful during bootstrap but should not own runtime engine state. | Introduce engine asset/cache interfaces gradually; bridge to Bevy while needed. |
| Window/input/platform | High but low priority | Rewriting platform plumbing gives little value. | Replace late with GPUI when sufficiently stable; do not write custom Wayland/X11/Win32 plumbing. |
| UI/HUD/debug tooling | Medium–high | Currently likely follows the Bevy host. | Future GPUI layer; keep game/simulation state independent from UI entities. |

### Immediate conclusion

The largest migration risk is **not** `sim-core`. It is continuing to implement renderer, terrain presentation, temporal, RT, GPU memory and task orchestration directly against Bevy/wgpu APIs.

That work must stop leaking upward now.

---

## 3. Freeze rules — effective immediately

Do not add new architectural dependencies on:

- `bevy_render` as a public engine API;
- Solari as a long-term lighting/RT API;
- `wgpu::*` types outside renderer backend or Bevy bridge modules;
- `bevy_tasks` for new engine-level scheduling;
- Bevy `Entity`, `World`, `Commands`, `Query`, `Res`, etc. inside engine-owned crates;
- Bevy `Mesh` as the canonical output of procedural geometry;
- Bevy asset handles as the canonical identity/storage model for engine runtime resources;
- Bevy render-graph nodes as the only implementation of a new rendering feature.

Bevy-specific code is allowed only when it is one of:

1. a thin adapter to an engine-owned subsystem;
2. a temporary implementation that can be deleted as one unit;
3. minimal glue required to keep the current game runnable.

**Do not polish temporary Bevy paths.**

---

## 4. Target dependency direction

```text
game/
    ↓
thessa-engine APIs
    ↓
engine subsystems
    ↓
backend interfaces
    ├── Vulkan
    ├── wgpu
    ├── GPUI
    └── Bevy bridge (temporary)
```

Forbidden direction:

```text
engine subsystem
    ↓
Bevy type
    ↓
"we will abstract it later"
```

The bridge adapts Thessa to Bevy, never the opposite.

---

## 5. Target engine layout

The exact crate names can change, but the dependency boundaries should converge toward:

```text
engine/
├── app/
│   ├── plugin registry
│   ├── resources
│   ├── events
│   └── lifecycle
│
├── ecs/
│   ├── world
│   ├── query
│   └── schedule graph
│
├── sim/
│   ├── gravity
│   ├── orbital
│   ├── vehicles
│   ├── colonies
│   └── gameplay simulation
│
├── jobs/
│   └── Rayon execution backend
│
├── render/
│   ├── api/
│   ├── graph/
│   ├── memory/
│   ├── vulkan/
│   ├── wgpu/
│   ├── terrain/
│   ├── atmosphere/
│   ├── lighting/
│   ├── fast_rt/
│   └── temporal/
│
├── assets/
│
└── platform/
    └── gpui/

bridges/
└── bevy/
```

Reusable engine crates remain MIT.

Game-specific code remains GPL-3.0-or-later.

No DirectX-facing engine API is introduced.

---

## 6. Plugin-first architecture

The engine core should stay small.

Most functionality should be installable as plugins:

```text
Core
├── App
├── PluginRegistry
├── World
├── Scheduler
└── Render API

Plugins
├── VulkanPlugin
├── WgpuPlugin
├── TerrainPlugin
├── AtmospherePlugin
├── FastRtPlugin
├── TemporalPlugin
├── NetworkingPlugin
├── PhysicsPlugin
└── AssetPlugin
```

The API should remain **semantically close to Bevy where that is useful**, because the migration is a strangler-fig replacement and we want Bevy plugins to be mechanically portable.

Example:

```rust
pub trait Plugin: Send + Sync + 'static {
    fn build(&self, app: &mut App);

    fn finish(&self, _app: &mut App) {}
    fn cleanup(&self, _app: &mut App) {}
}
```

This is not an attempt at source compatibility.

The goal is:

> Porting a Bevy plugin should normally mean replacing glue and registration code, not rewriting the subsystem.

Keep the good architectural ideas:

- plugins register systems/resources instead of owning a virtual `update()` loop;
- declarative scheduling;
- main-world/render-world separation;
- explicit extraction;
- pipelined CPU simulation and rendering.

---

## 7. ECS and scheduling

Bevy ECS is not a long-term dependency.

However, replacing ECS is **not** the first migration step. First stabilize subsystem boundaries so the ECS replacement does not force another rewrite.

The target scheduler has two layers:

```text
Thessa scheduler
    dependency graph / phases / declared access

Rayon
    actual CPU execution / work stealing
```

Do not write another general-purpose thread pool.

Work classes should be explicit:

```text
Simulation
ProceduralGeneration
Streaming
RenderExtraction
GpuPreparation
Background
```

Typical frame/tick flow:

```text
input snapshot
      ↓
simulation
      ↓
barrier
      ↓
gameplay / post-physics
      ↓
barrier
      ↓
render extraction
      ↓
GPU submit
```

The dedicated server uses the same simulation/ECS/scheduler crates without platform/render/UI crates.

Tokio remains appropriate for:

- network I/O;
- filesystem I/O where async is useful;
- HTTP/services;
- multiplayer control-plane work.

---

## 8. Renderer policy

### 8.1 Native Vulkan is the reference backend

Native Vulkan is the high-performance/reference renderer.

New renderer capabilities are designed against:

- the real GPU;
- Vulkan capabilities;
- explicit synchronization;
- explicit memory topology;
- explicit RT and queue control.

They are **not** designed around the lowest common denominator of WebGPU.

### 8.2 wgpu remains useful

wgpu remains:

- a portability backend;
- a convenient fallback;
- a possible macOS path;
- a fast way to keep broad hardware coverage.

But wgpu limitations must not shape engine-owned APIs.

Never expose these outside the backend boundary:

```rust
wgpu::Buffer
wgpu::Texture
wgpu::BindGroup
wgpu::Device
wgpu::Queue
```

Engine code uses Thessa descriptors/handles/capabilities.

---

## 9. Memory model and UMA

The memory API describes **intent**, not wgpu semantics.

Minimum memory classes:

```text
DeviceOnly
CpuToGpu
GpuToCpu
Shared
```

The Vulkan backend chooses the actual memory type and strategy.

On UMA, `Shared` should permit a true fast path:

```text
Rayon / CPU procedural work
          ↓
persistently mapped shared allocation
          ↓
flush/barrier if required
          ↓
GPU consumes the same allocation
```

Do not force:

```text
CPU allocation
→ staging allocation
→ GPU copy
→ final allocation
```

when the hardware exposes unified memory.

This matters especially for:

- RCBT/CBT state;
- terrain patch descriptors;
- generated vertex/index data;
- instance data;
- indirect draw/dispatch arguments;
- streaming metadata;
- RT instance metadata;
- dynamic procedural tables.

On discrete GPUs the same engine intent may transparently select a staged path.

The allocator/capability layer should be able to distinguish at least:

```text
SharedCoherent
SharedNonCoherent
Staged
DeviceOnly
```

---

## 10. Procedural-first engine policy

Procedural generation is not an asset-preprocessing feature in Thessa. It is a runtime architectural requirement.

A subsystem must support cheap local changes to the world.

RCBT/CBT already demonstrates the desired cost model:

> The cost of changing the world should scale with the amount of topology actually changed, not with the maximum size of the representation.

CPU procedural work is first-class.

Do not move work to GPU merely because it concerns rendering.

On APU/UMA systems the preferred path can intentionally be:

```text
available CPU budget
      ↓
procedural generation / topology / culling hints
      ↓
shared GPU-readable data
      ↓
GPU spends budget on raster/shading/RT
```

Use profiling, not ideology, to choose CPU vs GPU execution.

---

## 11. RCBT ownership

RCBT should remain independent from Bevy and from Project Thessa-specific policy.

Preferred split:

```text
rcbt-core
    topology, updates, invariants, CPU implementation

rcbt-wgpu / GPU backend(s)
    optional GPU implementations

bevy-rcbt
    thin Bevy adapter only

Project Thessa
    terrain policy, planetary semantics, streaming decisions
```

If native Vulkan-specific RCBT paths become useful, add them without changing the core data model.

The Bevy adapter is disposable.

---

## 12. Terrain migration

Terrain should stop producing Bevy rendering objects as its canonical output.

Target flow:

```text
planet / terrain state
        ↓
RCBT + procedural topology
        ↓
terrain generation / streaming
        ↓
backend-neutral patch/render descriptors
        ↓
renderer backend
        ├── Vulkan
        └── wgpu
```

The Bevy bridge may temporarily convert those descriptors into Bevy meshes/materials/entities.

That conversion must not leak back into the terrain core.

Move the existing cache/jobs/wanted/visible model behind a terrain-owned interface. Replace Bevy task execution with Rayon where applicable.

---

## 13. Atmosphere and stellar lighting

A single-Sun model is forbidden as a fundamental renderer assumption.

The renderer consumes:

```text
StarEmitter[]
```

with at least:

```text
direction
angular radius
radiometric / spectral contribution
visibility / eclipse factor
```

`N = 1` may have an optimized fast path, but never a different architecture.

Atmosphere and lighting must naturally support:

- multiple stars;
- different spectra/colors;
- different angular sizes;
- partial and total eclipses;
- surface views;
- atmospheric flight;
- orbit;
- deep-space views;
- planet-scale coordinate ranges.

Separate:

1. physical atmosphere/world state;
2. derived LUT/data preparation;
3. backend-specific GPU resources/passes.

Only layer 3 is renderer-backend-specific.

---

## 14. Solari policy

Solari is now a **temporary/reference implementation**, not a foundation.

Do not expand Solari-specific architecture.

Reasons:

- measured overhead is too large for the target performance model;
- its path-tracing-first lighting architecture does not match FastRT;
- RT work is insufficiently sparse/adaptive for Thessa's goals;
- RT scene and lighting ownership are too tightly inherited from Bevy/Solari.

Keep it only until equivalent functionality is replaced.

Use it for:

- visual comparison;
- regression/reference scenes;
- temporary lighting while the replacement is incomplete.

---

## 15. FastRT

FastRT is not a path tracer and not simply a "hybrid renderer".

Its core rule is:

> **Use ray tracing only where cheaper methods are not good enough.**

The generic path is:

```text
cheap estimate
      ↓
validity / confidence
      ├── good enough → keep result
      └── insufficient
               ↓
              RT
```

This can operate at sample/event granularity, not only effect granularity.

Examples:

```text
SSR hit valid
    → use SSR

SSR invalid / off-screen / unstable
    → trace only those reflection rays
```

```text
shadow map result reliable
    → keep it

thin geometry / difficult penumbra / invalid approximation
    → ray visibility correction
```

```text
cache/probe/history reliable
    → reuse

uncertain GI sample
    → sparse RT correction
```

RT is treated as a limited compute budget.

---

## 16. FastRT infrastructure

`FastRtPlugin` should provide infrastructure, not an all-or-nothing lighting solution:

```text
FastRT
├── BLAS/TLAS service
├── RT geometry classification
├── fallback/candidate queues
├── confidence/validity data
├── ray-budget allocator
├── temporal feedback
└── indirect/adaptive dispatch
```

Effects consume that service independently:

```text
ReflectionPlugin
ShadowPlugin
GiPlugin
EmissiveLightingPlugin
```

Critical performance invariant:

```text
ray_count == 0
    ⇒
RT-specific frame cost ≈ 0
```

Connecting the RT subsystem must not itself cause a large frame-time penalty.

The current ~15% FPS loss observed with Solari present but without useful RT work is an **anti-baseline**: the replacement must not reproduce this behavior.

---

## 17. RT scene ownership

The raster scene and RT scene are not required to match.

Geometry may be classified as:

```text
RasterOnly
RtFull
RtProxy
RtConditional
Ignore
```

Procedural systems can provide cheaper RT proxy geometry independently of raster LOD.

This is especially important for planetary terrain, where the engine already knows:

- topology;
- LOD structure;
- static/dynamic state;
- distance;
- procedural source;
- whether a full triangle representation is useful for the current RT effect.

Do not blindly mirror all visible raster geometry into the TLAS.

---

## 18. Unreal / Lumen as a reference implementation

Unreal Engine is not a migration target.

Lumen/Nanite/UE renderer are useful as:

- production reference implementations;
- architecture references;
- performance/quality baselines;
- benchmark targets;
- a source of already explored trade-offs.

Study concepts such as:

- screen-space first, fallback later;
- cached surface lighting;
- software vs hardware tracing;
- selective hit-lighting;
- RT instance culling;
- far-field approximations;
- async compute;
- dynamic-geometry budgets;
- temporal reuse.

Then implement the concept independently for:

```text
Rust
Vulkan
Thessa procedural workloads
FastRT
Thessa world model
```

The objective is to reuse engineering knowledge, not implementation code.

---

## 19. Temporal rendering and upscaling

Do not build the next temporal system directly around a Bevy/DLSS-specific API.

Define a backend-neutral contract:

```text
TemporalInputs
├── color
├── depth
├── motion vectors
├── jitter
├── exposure
├── history validity/reset
├── render extent
└── output extent
```

Possible implementations:

```text
Native TAA / TAAU
DLSS
OpenSuperSampling
future FSR / XeSS / other backends
```

Bevy's current DLSS integration is useful as a migration reference because it already demonstrates the required temporal inputs and device-level integration, but its API must not become the Thessa API.

---

## 20. Assets

Bevy assets may remain during the bootstrap period, but engine-owned runtime state must not require Bevy handles.

Introduce engine-owned resource identities and cache/streaming APIs for:

- procedural resources;
- shader resources;
- streamed terrain data;
- generated meshes/data;
- network/server-visible content;
- headless tools.

Bevy `AssetServer` and `Handle<T>` become bridge concepts.

---

## 21. Platform and UI

Do **not** write a custom window/input platform layer.

There is little value in owning:

- Wayland/X11 plumbing;
- Win32 plumbing;
- IME;
- clipboard;
- DPI handling;
- focus/input-method integration.

The intended future shell is GPUI once it is sufficiently stable for the project.

GPUI should own:

- application/window lifecycle;
- input;
- focus;
- clipboard;
- IME;
- DPI;
- HUD;
- editor/debug panels.

GPUI's entity/state model is not the game ECS.

```text
GPUI state != simulation world
```

Platform migration is deliberately late and independent from renderer/ECS migration.

---

## 22. Migration phases

### Phase A — Freeze Bevy debt

Effective immediately:

- no new engine APIs exposing Bevy types;
- no new engine APIs exposing wgpu types;
- no new Solari-specific architecture;
- no new Bevy Tasks dependency for engine work;
- no new procedural subsystem whose canonical output is a Bevy object.

This is the highest-priority phase because it prevents double work.

### Phase B — Engine-owned boundaries

Create or stabilize backend-neutral boundaries for:

```text
app/plugins
jobs
terrain/procedural
render
GPU memory/resources
temporal
RT
```

Implementations may still internally call Bevy/wgpu.

Public interfaces may not.

### Phase C — Native Vulkan memory/resource path

Start where Vulkan gives immediate value:

1. allocator;
2. shared/UMA buffers;
3. procedural/streaming buffers;
4. indirect argument buffers;
5. GPU resource lifetime/synchronization;
6. RT infrastructure.

Benchmark each path against the existing wgpu implementation.

### Phase D — FastRT infrastructure

Implement:

1. AS management;
2. RT geometry classification;
3. sparse candidate queues;
4. ray budgeting;
5. GPU timing/feedback;
6. effect-independent ray-query services.

Then migrate effects incrementally away from Solari.

### Phase E — Renderer ownership

Move behind Thessa-owned APIs:

- terrain rendering;
- atmosphere;
- stellar lighting;
- temporal/upscaling;
- materials;
- render extraction;
- streaming;
- RT.

At the end of this phase, `bevy_render` is an implementation detail of `bevy_bridge`, not part of engine architecture.

### Phase F — ECS and scheduler

After subsystem boundaries are stable:

- introduce Thessa ECS/world;
- introduce dependency scheduler;
- run CPU work through Rayon;
- keep a temporary Bevy ECS adapter if useful.

Simulation code must already be independent enough that this is mostly integration work.

### Phase G — Platform shell

When GPUI is sufficiently stable:

```text
Bevy window/input/UI
        ↓
GPUI
```

Do not block renderer migration on this.

### Phase H — Remove Bevy

Final state:

```text
project-thessa
├── thessa-engine
├── thessa-game
└── optional bevy_bridge
```

Deleting `bevy_bridge` should be dependency cleanup, not another engine rewrite.

---

## 23. Pull-request policy during migration

Every PR adding a substantial engine feature should answer:

### A. Does this code survive removal of Bevy?

If not, explain why the code belongs specifically in `bevy_bridge`.

### B. Does the public API describe the Thessa problem or the current library?

Bad:

```text
API mirrors wgpu/Bevy because that is what currently implements it.
```

Good:

```text
API expresses memory, render, topology or scheduling intent;
the backend chooses the implementation.
```

### C. Can the temporary integration be deleted as one unit?

If a Bevy integration cannot later be removed without modifying the subsystem core, the boundary is wrong.

### D. Is the work likely to be rewritten by a scheduled migration phase?

If yes, implement only the minimum bridge needed now.

---

## 24. Work that should be avoided right now

Until the relevant engine boundary exists, avoid spending significant time on:

- richer Solari integration;
- Bevy-specific RT abstractions;
- Bevy-only temporal/upscaler APIs;
- complex Bevy render-graph architecture for new systems;
- Bevy-specific procedural mesh ownership;
- optimization work whose result cannot be used by the native Vulkan backend;
- adding gameplay/simulation logic directly to Bevy ECS if it belongs in shared simulation;
- expanding Bevy Tasks usage.

The intent is not to stop development.

The intent is to ensure new work lands on the side of the strangler fig that survives.

---

## 25. Definition of done

Bevy is removable when all of the following are true:

- simulation runs without `bevy_ecs`;
- CPU scheduling runs through the Thessa scheduler + Rayon;
- renderer runs without `bevy_render`;
- native Vulkan backend is fully functional;
- wgpu backend does not require Bevy;
- terrain/RCBT/procedural systems expose engine-owned data;
- atmosphere and stellar lighting use engine-owned render APIs;
- FastRT no longer depends on Solari;
- temporal/upscaling uses a backend-neutral contract;
- assets/runtime resources do not require Bevy handles;
- platform/windowing runs independently from Bevy;
- headless server uses the same simulation crates;
- plugins register through the Thessa application layer;
- remaining Bevy code is contained in a dedicated bridge crate/module.

At that point:

```text
remove bevy_bridge
```

must not trigger a rewrite of any engine subsystem.

---

## 26. Short version for contributors

> Bevy bootstrapped Project Thessa and remains a temporary host while the game stays runnable.  
> It is no longer the architecture that new engine systems target.  
> New renderer, procedural, scheduling, RT, temporal and resource work must use Thessa-owned APIs, with Bevy implemented only as a bridge.  
> Native Vulkan is the reference GPU backend, wgpu is the portability backend, Rayon is the CPU execution backend, and GPUI is the intended future platform/UI shell.  
> The migration is incremental: replace one subsystem at a time and never create work that must be rewritten merely to remove Bevy.
