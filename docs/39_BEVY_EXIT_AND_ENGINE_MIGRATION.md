# Project Thessa — Bevy Exit and Engine Migration Plan

**Status:** Active migration  
**Language:** English  
**Updated:** 2026-10-15 (isolated renderer/streaming checkpoints and winit + egui shell selection)
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
| Window/input/platform | High but low priority | Rewriting platform plumbing gives little value. | winit lifecycle/input and system decorations; do not write custom Wayland/X11/Win32 plumbing. |
| UI/HUD/debug tooling | Medium–high | Production UI still follows the Bevy host. | egui overlay on the owned renderer; keep game/simulation state independent from UI state. |

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
    ├── winit / egui
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
    └── winit_egui/

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

## Implementation checkpoint (2026-10-15, `feat/fastrt-stage1`)

This isolated branch is not a replacement of the production Bevy client yet.
`thessa-fastrt` supplies classification, including deterministic duplicate-key
ordering by `(key, RtClass declaration order)`. `thessa-fastrt-wgpu` supplies
Cartesian ray × AABB compute, **not** a BVH/BLAS/TLAS. Its pool independently
sizes rays, boxes and output/readback, checks negotiated device buffer,
binding and dispatch limits before allocation, and guards logical dispatch
tails. `pooled_bytes()` exposes actual persistent memory use.

`thessa-render-wgpu` owns headless or surface-compatible instance/adapter/device/queue, capability
probing, explicitly opt-in experimental features, byte roundtrips and clear
readback. Texture readback uses 256-byte-aligned rows and returns tightly
packed RGBA8, including non-aligned widths. Hardware probe capabilities are
not a promise that default device limits support every experimental pipeline.

`ViewUniformBlock` retains the 464-byte CBT ABI but shader world coordinates
mean **camera-relative render space**: subtract the camera/render origin in
f64 before narrowing; upload zero camera position; reject non-finite f32
matrices as well as singular/non-finite f64 inputs, inconsistent camera origins
and non-affine camera transforms. Tile frames must use the
same `TileAnchor::to_gpu(camera_body_m)` origin. The body transform is a
rotation in this space, never a second absolute translation.

Portable WGSL assemblies, geometry capacities and tile anchors now live in `rcbt-core`; the Bevy
adapter re-exports its old precision path. Tile transport also rejects f32
narrowing overflow. The fourth-order normal stencil was rejected by an
actual quantized HeightPage GPU regression: a 100 m step causes ~1.2° of
false slope two cells before the cliff. Production now uses bounded two-tap
secants (five bilinear height evaluations including the center rather than
nine for interior fourth-order). Tests exercise all six cube faces, borders,
and a planar parent/child LOD seam; this is not full visual acceptance on
arbitrary game terrain. The analytic smooth-field test is explicitly only a
mathematical comparison, not a terrain-quality claim.

`terrain::TerrainRenderer` now executes the **same** geometry, classifier and
raster WGSL as the Bevy adapter, using persistent packed pages, tile-local
geometry, triangle compaction/history and GPU indirect draw arguments.
Camera-only frames do not regenerate geometry. `update_pages` validates all
ordinals, duplicates and stable-slot capacities before publication and
regenerates only dirty patches, without CPU meshes or readback. Camera
coordinates remain body-fixed. Bounded raw material residency, direct stellar
lighting and native migration-viewer presentation are now available (below);
body motion, networked game-world streaming and full scene/effect integration remain unported.
Byte roundtrip is still not production snapshot publication parity.

Run the dependency-independent screenshot/latency harness:

```sh
cargo run --release -p thessa-render-wgpu --example terrain_frame -- terrain.png
```

Radeon 880M / RADV, 1280×720, 64 synthetic HeightPages, 129,024 active triangles,
120 measured frames after 30 warm-up frames (2026-10-15, one run): 0.997 ms
median, 1.219 ms p95, reciprocal median 1003.4 offscreen frames/s. Timing
includes camera upload, classification, indirect raster and GPU completion;
excludes initial setup (92.32 ms) and screenshot readback. Dirty publication
+ geometry + GPU completion measured 0.062 ms median for one patch and
0.552 ms for all 64. This is **not game FPS**, a Bevy comparison, or a complete
client replacement. Hardware regressions also check CPU position/height
parity, unchanged clean geometry, static pixel determinism, nonempty raster,
unaligned screenshot extent, and looking-away culling.

![Owned CBT frame, synthetic quantized terrain fixture](artifacts/owned-terrain-2026-10-15.png)

Normal-stencil screenshot A/B on this smooth fixture (same code at `015aabd`,
isolated reference worktree changes only the stencil back to fourth order):
mean absolute RGB difference 0.00174/255, p99 0, max 2; 4,272 of 921,600 pixels
change. [Fourth-order reference](artifacts/owned-terrain-fourth-order-reference.png)
and [difference ×16](artifacts/owned-terrain-normal-diff-x16.png).
This near-equality does not excuse the cliff ringing: the quantized step GPU
test, not this smooth screenshot, establishes that regression. These A/B
captures do not establish an FPS gain; ordinary camera frames do not rerun
geometry and timing variance must not be attributed to the normal stencil.

Three later alternating runs per stencil, 100 dirty-publication samples per
run after 30 warm-up iterations (same 64-page fixture):

| Dirty upload + geometry + completion | Fourth-order median of run medians | Secant median of run medians |
| --- | ---: | ---: |
| 1 page | 0.044 ms | 0.044 ms |
| 64 pages | 0.507 ms | 0.441 ms |

The 64-page result is ~13% lower in this sample, but ranges overlap
(0.450–0.560 ms vs 0.436–0.544 ms). Do not extrapolate to game FPS or claim
a stable universal speedup. Static-frame medians of run medians were
1.030 ms vs 1.034 ms; one reference run had a 120.791 ms p95 outlier.
[Raw measurements](artifacts/owned-terrain-normal-ab.json) retain that outlier.

Previous checkpoint verification: workspace fmt and all-target/all-feature clippy,
workspace lib/bin and integration tests, then all 25 Bevy-CBT ignored GPU
tests passed (1,770 successful tests across these invocations). Own backend,
quantized cliff/LOD, dirty update, pixel/culling, view and slab regressions
are included. The embedded-server cleanup fixture now keeps PID scratch in
the crate's `target/test-tmp`, exposes shell stderr and allows 250 ms rather
than 25 ms for the timeout fixture to start under parallel load; child-exit
and reap assertions remain intact. No production server behavior changed.

Measured on Radeon 880M / RADV, release `batch_slab`, synchronous end-to-end
packing/upload/dispatch/readback (2026-10-15, one run):

| Rays × boxes | Persistent pool | Scalar CPU | GPU end-to-end |
| --- | ---: | ---: | ---: |
| 4096 × 256 | 33,693,712 B | 7.46 ms | 3.94 ms |
| 16384 × 256 | 134,750,224 B | 25.30 ms | 15.89 ms |

For 4,194,304 pairs the old six pair-sized buffers used 402,653,184 B;
the new pool is 128.51 MiB instead of 384 MiB (~66.5% reduction). These
are batch query timings, **not** game FPS or a before/after latency claim.
Regression tests cover device-limit planning, non-power-of-two tails,
changing/shrinking batch shapes, unaligned rows and astronomical camera
translation invariance.

### Canonical material/presentation slice (2026-10-15)

`rcbt-core::material_page` now owns the immutable `CbtMaterialPage` payload
and `MaterialPages` source map; `material_cache` owns the existing slot policy,
ancestor mapping and world-addressed directory. Bevy re-exports the old paths
and `CbtRenderMaterialPages::source_pages()` borrows the exact neutral snapshot,
without repacking/cloning pixels. `lighting::SurfaceLighting` owns the shared
seven-vec4 direct-light prefix, with finite/nonnegative/unit-direction checks.

`TerrainRenderer::with_material_capacity` explicitly bounds raw texture-array
residency; `update_materials` consumes independent demand and immutable source
generations. Resident ancestors are protected first under exhaustion; layers
stay stable across priority changes; warm revisits do not reupload; changed
generations upload only dirty pages. Removed/stale sources disappear from the
GPU directory before sampling. Fixed-capacity directory hashing uses the bound
array's actual length (padding a smaller hash table would be incorrect).
Invalid cube IDs/incomplete mip chains fail before any publication. The raw
backend requires all eight mips; compact microstore upload is **not ported**.
Default construction reserves one material layer; callers choose larger budgets
explicitly. `update_lighting` validates then uploads direct illumination without
turning on ocean/cloud/city/RT effects. Material upload counters expose page,
byte and currently published resident counts.

`apps/terrain-native` is a GPL migration viewer with **no Bevy runtime
dependency**. It reuses `canonical_launch_setup` (source-checked frozen erosion
and the game's HIGHLANDS bookmark), the real bounded LOD selector, quantized
height builder, material builder and ancestor pages. The initial material slice
baked one static resident partition; the live-streaming continuation below
replaces that startup snapshot. Ephemeris-derived directions/irradiance and the
existing atmosphere blackbody/inverse-square helpers supply up to three stars
at EPOCH; the terrain axes match the launch survey. There is no fixed fake sun.
Atmospheric transport/eclipses are not represented in this slice. Manual EV100
and vsync flow through `RequestedGraphics -> ResolvedGraphicsSettings`; the
viewer explicitly warns that other graphics settings/effects are unported.

The same scene drives offscreen readback and a winit-owned native surface:
compatible-adapter selection, sRGB format negotiation, resize/depth recreation,
zero-size suspension, surface-lost recreation, outdated/suboptimal recovery,
timeout/occlusion skips, close and basic keyboard orbit/zoom. There is no CPU
frame copy/blit, CPU mesh or Bevy render world. The later egui checkpoint below
extends this shell; it does not replace the complete production client.

```sh
cargo run --release -p thessa-terrain-native
cargo run --release -p thessa-terrain-native -- --headless terrain.png --frames 120
```

Window controls: Left/Right orbit; Up/Down zoom; Esc exit. `--frames N` bounds
either mode for smoke runs. `--graphics PATH` selects settings (default
`graphics.toml` from the working directory). Only manual exposure and vsync are
currently consumed. This path intentionally does not claim HDR/tonemapping,
auto-exposure, atmosphere/clouds/ocean/shadows, RT, ships, HUD/UI, networking,
moving-body parity with `thessa-client`.
The screenshot is coarse and lacks the game's effect stack: it demonstrates
canonical page/material transport, **not visual acceptance or client parity**.

Historical static-slice measurement (not the current live workload): Radeon
880M / RADV, 1280×720, 64 canonical height tiles, 85 material pages
(7,427,300 payload bytes across eight mips), 48,384 drawn triangles, 120 measured
frames after 30 warm-up frames: 1.291 ms median / 1.651 ms p95 in one run
(774.7 reciprocal median offscreen frames/s). Timing is camera upload +
classification + raster + GPU completion, **not GPU-only timing**; excludes
startup (2.03 s canonical CPU bake), initial material publication and screenshot
readback. A native Wayland smoke run presented 120 frames in 1.286 s (93.3
presentation requests/s, including compositor/presentation pacing; individual
display feedback was not measured). Neither number
is game FPS, a Bevy comparison, or an improvement over the different synthetic
fixture above.

[Raw capture metadata](artifacts/owned-canonical-terrain-2026-10-15.json) records
the timing boundary and screenshot digest.

![Owned canonical frozen Thessa terrain, static HIGHLANDS snapshot](artifacts/owned-canonical-terrain-2026-10-15.png)

Regression coverage includes raw material pixel changes and ancestor fallback,
unchanged-generation zero uploads, dirty reupload, atomic invalid publication,
removal back to fallback, coarse-first exhaustion, warm reuse, stale-generation
masking, fixed directory hashing, shared linear-light mip parity and checked
orbit camera uniforms. The original Bevy material/microstore consumers use the
same payload via re-exports; they have not been removed or silently disabled.

Historical verification for the material slice: workspace fmt, all-target/all-feature clippy,
workspace lib/bin and integration tests, plus the separate Bevy-CBT `render`
ignored-test invocation all passed (1,775 successful tests across these runs;
24 hardware tests in that final invocation). Native material/raster hardware
regressions ran as ordinary tests. The normal runtime dependency tree for
`thessa-terrain-native` contains no Bevy crates. The canonical offscreen capture
and 120-frame native Wayland smoke run also completed successfully.

### Live terrain streaming continuation (2026-10-15)

`crates/terrain-stream` (GPL game code, no renderer/ECS dependencies) now owns
the extracted two-tier selector, motion trigger/debounce, velocity bias,
fine-selection hysteresis, complete-cover construction, ancestor dependencies,
height priority, material refinement/dependency/priority rules, capacity-one
mailboxes and Rayon executor. **Both** `thessa-client` and the native viewer
consume those policies/executor. The existing `worldgen-rocky::streaming`
resident-cover and deterministic LRU functions remain the shared cover policy.
No selector or terrain scheduler lives in the native application's scene code.
The Bevy client still owns its mesh fallback, cache/assets, topology-planning
and resource-publication glue; complete `WorldTerrain` lifecycle extraction is
not finished. The new renderer-free runtime does not yet replace that glue.

The native runtime synchronously bootstraps six height/material roots, then
polls at most eight height and four material builds without waiting for them.
Only quantized heights are built for GPU geometry, not CPU meshes. Camera-only
frames poll bounded mailboxes without rebuilding dependencies/LRU priorities.
Coverage uses ready families or cached ancestors/previous finer cover while
replacements load. Height-cache admission counts both residents and in-flight
results against 1,024 entries and protects the drawn cover and bootstrap roots.
If exhausted, refinement stalls and is counted rather than evicting drawn
pages or allocating without bounds. Material sources retain at most 1,024 pages
after each tick (up to four completed pages transiently before eviction).
Material generations are monotonic across rebuild/eviction; material refinement
is independent of matching height keys. The old client's material completion
path no longer discards finer pages just because no height page shares the key;
its existing source-budget trimming still applies. The GPU array reserves at most 256 raw
layers, with the shared coarse-ancestor-first fallback under exhaustion.
These are diagnostic viewer budgets, not newly supported `graphics.toml`
quality tiers or a replacement for production graphics capability resolution.

`TerrainRenderer::with_capacity` reserves a fixed geometry/page pool;
`publish_partition` validates IDs/ancestor overlap, frames, radii, duplicates,
capacity and assigned height-slot sizes **before** mutation. It reuses the
existing core `SlotCache`, retaining slots across reorder, reclaiming removed
slots, zeroing inactive metadata and invalidating history on dirty/reused slots.
Only dirty/new slots regenerate geometry; pipelines and buffers stay alive.
Sparse interior holes and an empty partition cannot retain old indirect draws.
Source generations identify unchanged bytes; direct `update_pages` invalidates
that shortcut. `update_pages` and `update_view` now take mutable renderer access
because they maintain publication/camera state. Initial construction still
requires at least one patch; subsequent partitions may be empty.

Headless `--prestream-seconds N` (default 3) controls excluded initial streaming;
it is a time budget, **not** a convergence guarantee. `--sweep` advances the
diagnostic orbit by 0.006 rad per frame (including 30 excluded warm-up frames).
Both options require headless mode. Measurement includes selection, mailbox
polling, publication/dirty uploads, camera upload, classify/raster and GPU
completion; p50/p95/p99/max are reported, with CPU streaming timings separately.
Screenshot readback,
CPU/bootstrap/GPU setup, pre-stream and warm-up are excluded. This is neither
GPU-only timing nor game FPS, and asynchronous completion changes capture
residency between runs. Publication/regeneration counts, upload bytes (excluding
the six initial material pages), cache sizes, pending work and capacity refusals
are printed cumulatively since bootstrap (including pre-stream/warm-up), with
measured-window selection/publication counts separately. A moving camera sweep
is not a moving-body/networked-game test.
The window's stream clock survives surface suspension/recreation with its
selection history; a recreated surface must not reset selection debounce time.

Regression coverage adds out-of-order family completion, capacity refusal,
hard height-cache admission, extracted-selector equivalence/debounce, material
refinement area/budget, and hardware slot reorder/reclamation, dirty-only work,
invalid-publication atomicity, sparse/empty draw removal and source-generation
invalidation after direct page edits.

Sequential release captures on Radeon 880M / RADV (1280×720, 10 s pre-stream,
30 warm-up + 1,200 measured frames each):

| Workload | Median | p95 | p99 | Maximum | Final geometry |
|---|---:|---:|---:|---:|---|
| Stationary camera, live material stream | 2.740 ms | 4.907 ms | 8.683 ms | 24.217 ms | 287 tiles, 297,216 drawn triangles |
| Moving camera, live height/material stream | 2.563 ms | 3.717 ms | 9.203 ms | 14.151 ms | 292 tiles, 313,344 drawn triangles |

The sweep published 75 geometry partitions during measurement (132 including
pre-stream/warm-up), with 1,180 regenerated patches total and no capacity
refusals. Its CPU streaming p95/p99/max were 1.077/6.371/11.046 ms; stationary
CPU max was 16.062 ms. Selection/publication hitches remain work to do, not
something the median hides. Material work was still pending at capture end.
The window smoke requested 600 presentations in 5.074 s (118.3/s including
compositor pacing, vsync requested off); display feedback was not measured.
These are not game FPS or a performance comparison against the earlier
64-tile snapshot/Bevy/large_cbt: workload and rendered detail differ.

[Capture metadata](artifacts/owned-streaming-terrain-2026-10-15.json) includes
source/binary identity, timing boundaries, counters and screenshot digests.
[Sweep frame](artifacts/owned-streaming-sweep-2026-10-15.png). Shading and thin
dark edge artifacts at sharp relief remain unaccepted; effects are still absent.

![Live-streamed canonical terrain, not visual acceptance](artifacts/owned-streaming-terrain-2026-10-15.png)

Verification: workspace fmt; all-target/all-feature clippy with Rust warnings
denied; 1,715 workspace lib/bin + 43 integration + 24 separate Bevy-CBT ignored
hardware tests passed (1,782 total). Owned slot/raster regressions run as ordinary
hardware tests. Runtime dependency inspection still finds no Bevy in the native
viewer. Headless stationary/sweep captures and the native window smoke passed.

Next production boundary: move the remaining `WorldTerrain` cache/publication
glue onto the shared lifecycle, port f64 moving-body/camera transforms and the
actual scene/effect passes. Keep the old client runnable; remove each Bevy
consumer only after its production replacement works. GPU conforming bisector
split/merge topology and FastRT acceleration structures remain unimplemented.

### egui shell checkpoint (2026-10-15)

The selected UI integration is **winit + egui**, superseding the GPUI proposal.
egui, egui-wgpu and egui-winit 0.37 are MIT OR Apache-2.0 and already use the
workspace's wgpu 30 line. The HUD embeds the game's existing OFL-1.1 Noto Sans
asset (egui 0.37 has empty default features); Latin/Cyrillic/digit glyph coverage
is regression-tested to prevent a silently text-less/tofu HUD.
Slint is not introduced; neither is a GPUI backend
fork, a renderer downgrade or another logical device. Existing game-specific
application/streaming crates retain GPL-3.0-or-later; reusable engine crates
retain MIT. This decision does not relicense existing code.

`apps/terrain-native` enables its optional `hud` feature by default. The window
requests system/toolkit window decorations (appearance follows compositor policy),
while egui-winit handles widget input, pointer cursor,
clipboard, IME output and native scale-factor conversion. Camera shortcuts are
suppressed when egui consumes the keyboard event; lifecycle/resize/redraw events
still reach the window handler. This is a single-window integration, not a
general multi-viewport desktop framework.
Window/HUD state is released in winit's `exiting` callback, before the consumed
event loop destroys its native display; egui-winit's clipboard must not outlive
the display handle it borrows. Suspension also drops/recreates this state.

The HUD reports residency, pending work, publications, regeneration, material
upload bytes from post-construction updates, capacity refusals and host frame
wall time (including waits, not GPU timing or game FPS). It provides
orbit/distance/reset controls and session-only
manual EV100/VSync edits. Graphics edits update `RequestedGraphics`, resolve
against the existing backend capabilities, then update exposure/presentation.
VSync reconfiguration is deferred until the acquired surface frame is presented.
The source `graphics.toml` is never rewritten.

GPU composition uses the **same device, queue, command encoder and final color
target** as terrain. The HUD pass uses `LoadOp::Load`, alpha blending and no
depth attachment. There is no scene texture import, intermediate scene copy,
GPU-to-CPU readback or second device. egui still builds/tessellates UI on the CPU
and uploads its own mesh/font resources; this is not a claim of zero CPU work
or zero upload. The sRGB surface path uses egui's sRGB-aware output shader.

```sh
cargo run --release -p thessa-terrain-native
cargo run --release -p thessa-terrain-native -- --no-hud --frames 600
cargo run --release -p thessa-terrain-native -- --headless terrain-hud.png --hud --frames 600
cargo run --release -p thessa-terrain-native --no-default-features
```

`--no-hud` does not create egui CPU/GPU state or run a UI pass;
`--no-default-features` also removes the UI dependencies. Headless mode remains
terrain-only by default to preserve its benchmark boundary; explicit `--hud`
includes UI construction/upload/drawing in frame latency and the capture.
Readback is confined to captures/tests and excluded from measured frames.

Regression coverage includes passive-widget camera preservation (unbounded
keyboard orbit must not be clamped by a slider), pointer capture, and an ignored
hardware test for preserving scene pixels outside the HUD across target resize,
1.0/1.5 native scaling and complete HUD recreation. Run that hardware test with
`cargo test -p thessa-terrain-native -- --ignored` on the development GPU.
The same command runs a native-display regression that launches both HUD and
terrain-only windows and requires clean process exit, covering the clipboard
display-lifetime failure found by the Wayland smoke run. CI without a display
leaves that explicitly ignored test disabled.
This is a migration HUD, not production flight/menu/editor UI or visual parity.

Verification: workspace fmt; all-target/all-feature workspace clippy and
no-default-feature viewer clippy with warnings denied; 1,717 workspace lib/bin,
43 integration, 24 existing ignored Bevy-CBT GPU and two new ignored viewer
GPU/native-display tests passed (1,786 successful tests). The runtime dependency
graph contains one wgpu version (30.0.1), no Bevy and no toolkit default font
bundle. All 33 newly resolved Rust packages offer permissive license choices.

Final stationary A/B/B/A observations on Radeon 880M, 1280×720, 600 measured +
30 warm-up frames per run and 10 seconds of pre-stream: terrain-only medians
**2.526 / 2.661 ms**, HUD medians **2.618 / 2.757 ms**. Each ended with 287 tiles,
297,216 drawn triangles and zero measured geometry publications/refusals.
Material jobs remain asynchronous and clocks/scheduling are not locked, so this
is not an isolated constant-overhead estimate or a game-FPS claim. The resized
native Wayland run requested 1,200 presentations in 10.058 seconds (119.3
requests/s), captured the owned window at 1188×813 physical pixels and exited
cleanly. Compositor/resize/capture work is included; displayed-frame feedback is
not measured.

[Raw measurements, counters and limitations](artifacts/owned-egui-hud-2026-10-15.json)
and [native window capture](artifacts/owned-egui-window-2026-10-15.png).

![Shared-device egui migration HUD; terrain visuals remain unaccepted](artifacts/owned-egui-hud-2026-10-15.png)

### Shared material lifecycle continuation (2026-10-15)

The next isolated slice (`feat/terrain-material-lifecycle`, based on the egui
checkpoint) removes duplicated material ownership from the two hosts:

- MIT `rcbt-core::material_source::MaterialPageStore` owns the immutable
  `Arc<MaterialPages>` directory, per-page identities, publication revision,
  ordered source priority and hierarchical budget trimming. Completed batches
  COW the directory once; mip payloads remain shared. Empty batches, unchanged
  priorities, missing removals and satisfied budgets preserve revision/directory.
  Removal and reinsertion cannot reuse a page generation. Revision exhaustion
  is rejected before mutation, not saturated into an aliased dirty identity.
- GPL `terrain-stream::material::MaterialStream` owns nonblocking completion
  polling, refined material demand, regional-ancestor build dependencies,
  deduplication, job admission and source-cache policy. Both `WorldTerrain`
  and the native `TerrainStream` use this implementation. Geometry remains
  independent: no matching height tile is required to accept a material result.
- The existing 512-demand/refinement and level-17 caps remain. The material job
  limit is four, reduced to two for new admissions while at least four height
  jobs are pending; existing jobs are not canceled when pressure changes.
  Hosts retain the 1024-page default source budget. Polling can transiently add
  up to four completed pages; reconcile trims before publication. Budget zero
  starts no jobs. Selection epochs follow the host's accepted/debounced view;
  changing pixel angle also invalidates refinement (including stationary resize).
- Worker disconnection frees the job slot, reports a failure count and permits
  current demand to retry without failing the native frame. Both hosts log it.
- Bevy's `CbtRenderMaterialPages` is now only a resource/publication wrapper.
  A revision check through a **shared** `ResMut` borrow occurs before mutable
  dereference; unchanged frames do not set Bevy's change flag. Render extraction
  shares the directory. The compact microstore still owns its derived decoded
  shadow representation, preserving source generations and encoding only dirty
  pages; it is not moved into the game lifecycle.
- Height-cache eviction no longer pins height entries merely for material jobs
  or deletes material pages sharing the same tile ID. The independent material
  budget/hierarchy decides material eviction; CPU fallback asset removal and
  height publication remain in their existing Bevy bridge.

The producer/store lifetime must span a renderer cache lifetime: source
revisions are local monotonic identities, not globally unique across unrelated
stores. Resetting a source owner requires resetting its renderer consumer too.
No new dependency, unsafe code, GPU device, readback or intermediate scene
target is introduced. The canonical field, material bytes, WGSL, graphics
settings and same-target egui composition are unchanged. This closes the
**material lifecycle slice**, not all `WorldTerrain` ownership or the production
Bevy migration. Remaining work includes height-cache/asset/ECS lifecycle,
moving bodies, full scenes/effects and the production client/UI lifecycle.

Regression coverage includes COW/no-op snapshots, monotonic reinsertion,
bounded hierarchy trimming, revision exhaustion, regional-to-exact scheduling,
job pressure/deduplication, disconnected-worker retry, receiver teardown,
pixel-scale invalidation, geometry-independent residency, Bevy change flags,
compact shadow reuse, and actual GPU dirty/reinsert/fallback raster output.
`cargo bench -p thessa-rcbt-core --bench material_source` measures source-only
snapshot/dirty-batch work at 1024 sources and 512 priorities; it is not a
whole-renderer or game-FPS benchmark.

Final verification: fmt and all-target/all-feature workspace clippy with Rust
warnings denied, no-default-feature viewer clippy, **1,728** workspace lib/bin +
**43** integration + **24** Bevy-CBT ignored GPU + **2** viewer GPU/native-exit
tests passed (**1,797** total). No dependencies or licenses changed; the native
runtime still contains one wgpu version and no Bevy. Local diff/ownership review
confirmed a single material writer per host and a thin Bevy publication boundary.

Radeon 880M stationary A/B/B/A, HUD enabled, 1280×720, 600 measured + 30 warm-up
frames and 10 seconds pre-stream per run: old release medians **2.540 / 2.491 ms**,
new release **2.489 / 2.563 ms**. All runs ended with 287 tiles, 297,216 triangles
and no measured geometry publication or capacity refusal. Material work remains
asynchronous and clocks/scheduling are not locked; this is not an isolated
speedup/overhead estimate. The sweep exercised 50 measured geometry publications
without refusal. The native Wayland window requested 1,200 presentations in
10.232 seconds and exited cleanly (117.3 requests/s, not displayed/game FPS).

The source-only microbenchmark uses 1024 independently allocated synthetic
material pages and 512 priorities. Median clone/drop cost including the priority
Vec was **60 ns**, satisfied-budget check **20 ns** (timer-floor scale), a
four-page COW update **14.277 µs**, and four removals/reinsertions with a retained
snapshot **14.416 µs**. Page construction and all renderer work are excluded;
these measurements are metadata costs, not end-to-end performance claims.

[Raw measurements and validation](artifacts/owned-material-lifecycle-2026-10-15.json),
[stationary capture](artifacts/owned-material-lifecycle-2026-10-15.png), and
[sweep capture](artifacts/owned-material-lifecycle-sweep-2026-10-15.png).
Captures retain the existing unaccepted terrain shading/dark-relief artifacts;
this slice does not claim visual parity or fix them.

### Shared height lifecycle continuation (2026-10-15)

`feat/terrain-height-lifecycle` continues from the material-lifecycle checkpoint.
Both hosts now use the same renderer-free height lifecycle instead of owning
parallel job maps, residency/LRU records and admission/eviction code:

- MIT `rcbt-core::height_source::HeightPageStore` owns a COW directory of
  immutable `Arc<HeightPage>` payloads and local monotonic source revisions.
  Content-equal publication is a no-op; a completed batch copies the directory
  at most once and shares unchanged residual vectors. A changed page or an
  eviction/reinsertion receives a fresh producer revision (not a per-ID version
  reset to one). Revision exhaustion rejects the source batch before mutation.
  Resetting a producer requires resetting its renderer consumer.
- GPL `terrain-stream::height::HeightStream<Resident, Built>` owns height jobs,
  residency, LRU epochs, source publication and bounded admission. Built/resident
  payloads are opaque generic values: the shared code names no Bevy, wgpu, asset,
  entity or mesh types. Native payloads are `()`; the old client's host callback
  converts CPU-built meshes/images into its existing asset handles.
- Inputs are the accepted tile demand, current drawn cover, selection epoch,
  `SelectionView`, existing canonical field and explicit `HeightPolicy`.
  Outputs are completed/failing job counts, dirty tile identities, admission
  pressure and moved retired payloads. Retirements include **replacements** as
  well as LRU evictions; the host releases each returned asset bundle once.
- The current GPU page builder is shared too: unchanged 33x33 canonical field
  samples and 0.5 m absolute quantization budget. CPU fallback mesh/material
  builders remain in the Bevy host; GPU jobs still do not build these assets.
- Policy preserves eight in-flight jobs, 1024 GPU/native cache entries and
  512 CPU-fallback entries. Admission now counts **resident + pending** results
  for both clients; it evicts cold pages before spawning, rather than allowing
  the old client's pending jobs to overrun its cache after completion. The drawn
  cover, demand dependencies and pending jobs are protected. The native six-root
  bootstrap stays pinned; the old client keeps its existing backdrop/bootstrap
  policy. Protected over-budget state is retained/reported, never turned into
  holes to satisfy a smaller budget. Protected resident pages may temporarily
  prevent further refinement until capacity is available. Native counters report
  refusals; the old host logs entry into capacity pressure without per-frame spam.
- Nonblocking polling drains every ready mailbox even if another build failed
  or disconnected. Failed slots are released, logged/reported and retryable;
  the native frame no longer fails merely because one worker disconnected.
- Bevy's `CbtRenderPages` now only publishes/extracts the neutral snapshot.
  The shared `ResMut` revision check precedes mutation, avoiding extraction on
  clean frames. Asset creation/removal stays in `present_cpu_tile` and
  `release_height_assets`; LRU/admission no longer names `Assets` or `Handle`.
  The former poll/upload perf scopes are combined as
  `world.terrain_job_poll_and_asset_upload`; spawn scope includes admission/LRU
  and host retirement. Counter budgets remain cache entries, not byte limits.
- Native publication also notices changed **visible page content** when the
  cover's tile IDs did not change, rather than waiting for another LOD switch.
  Page versions still prevent unchanged geometry from regenerating on the GPU.

The height source is the sole owner of residual vectors. A separate small
TileKey map tracks LRU/opaque host payloads; the native path now pays that extra
metadata/map/Arc overhead to share immutable extraction snapshots, not a second
copy of every height vector. No new dependency, unsafe code, logical GPU device,
scene readback or intermediate render target is introduced. Material residency,
physics, canonical generation, WGSL and shared-target egui composition are
unchanged. The old client remains supported and runnable.

This closes the **height source/cache/job/admission/retirement slice**, not the
entire Bevy migration. `WorldTerrain` still integrates selection/topology and
camera/ECS presentation; CPU fallback asset construction, global client/window
lifecycle, moving bodies, production game assets/effects/UI/networking and
FastRT acceleration structures remain outside this slice.

Regression coverage includes COW/content no-ops, monotonic reinsertion, revision
exhaustion, mixed success/failure batches, bounded admission including pending
results, protected-cover pressure, root pinning, deterministic LRU, one-time
replacement/eviction retirement, receiver teardown, invalid-address rejection
before asset conversion, real Bevy asset retirement, unchanged Bevy change
flags, same-cover height edits and GPU regeneration from source revisions.
The source-only benchmark is
`cargo bench -p thessa-rcbt-core --bench height_source` (1024 33x33 pages).

Final verification: workspace fmt and all-target/all-feature clippy with Rust
warnings denied, no-default-feature viewer clippy, **1,739** workspace lib/bin +
**44** integration + **24** Bevy-CBT ignored GPU + **2** viewer GPU/native-exit
tests passed (**1,809** total). The native runtime still contains no Bevy and
one wgpu version (30.0.1); no dependency/license changed.

Radeon 880M stationary A/B/B/A with HUD, 1280×720, 600 measured + 30 warm-up
frames and 10 seconds pre-stream: old medians **2.701 / 2.560 ms**, new
**2.789 / 2.611 ms**. Geometry matched (287 tiles, 297,216 triangles, no measured
geometry publications/refusals). New medians were slightly higher; asynchronous
material work and unlocked clocks/scheduling prevent isolating migration
overhead or claiming a speedup. Stream CPU medians were 1 µs old / 2–3 µs new.
The sweep exercised 55 measured geometry publications without refusal, retaining
coarse coverage while its final six height jobs were pending. The native window
requested 1,200 presentations in 10.250 seconds and exited cleanly (117.1
requests/s; not displayed-frame feedback or game FPS).

Source benchmark medians: shared snapshot **40 ns**, eight equal completions
**371 ns**, eight dirty completions with a retained snapshot **16.150 µs**, and
eight evictions/reinsertions **17.012 µs**. Fixtures contain 1024 independently
owned 33x33 residual vectors (~2.23 MB). Baking and per-sample fixture cloning
are outside the timed region; dirty samples include directory COW and old
snapshot release. These are source-publication costs, not full lifecycle/GPU
or whole-renderer performance estimates.

[Raw measurements/validation](artifacts/owned-height-lifecycle-2026-10-15.json),
[stationary capture](artifacts/owned-height-lifecycle-2026-10-15.png), and
[sweep capture](artifacts/owned-height-lifecycle-sweep-2026-10-15.png).
Existing terrain shading/blur/dark-relief artifacts remain unaccepted.

### Body/camera frame continuation (2026-10-16)

`feat/body-camera-frames` continues from `5a1c01e` (height lifecycle).

**Ownership and API.** MIT `thessa-render-core` owns only reusable presentation
geometry: `BodyFrame` and `ViewParams`, with glam as its only dependency and
unsafe forbidden. It does not own simulation state, celestial attitudes,
projection policy, wall time, assets, ECS, Bevy, wgpu or a GPU device. The old
`render-wgpu::view::ViewParams` path re-exports the neutral type; the existing
464-byte WGSL uniform ABI remains in the wgpu consumer without layout changes.
No GPL code/content is moved into MIT. The pre-existing flight period selection
is shared in GPL `flight-authority::presentation`; it retains the exact period
preference (explicit rotation period, tidally locked design period, otherwise
80 hours) and x/z/-y axes.

`BodyFrame::new(body_position_m, body_to_world, render_origin_m)` validates finite
positions, subtraction and a unit quaternion (squared-length tolerance 1e-12).
It subtracts the body and render origin in f64 **before** rotation/narrowing.
Point mapping includes translation; direction mapping does not.
`view_in_body` maps an origin-local render camera into the body's coordinates,
preserving projection/exposure. Inputs must use the same presentation sample;
an absolute astronomical camera must not be passed as an origin-local camera.
The wgpu consumer still validates affine/invertible/finite camera/projection
matrices and compatible camera origins before uploading anything.

**Both hosts.** Bevy terrain now uses this frame for CPU tile placement, GPU
body-transform publication, clearance mapping, and camera/forward/stream-origin
selection. Bevy `Transform` is still a local presentation consumer, never an
authoritative astronomical position. Reading the final camera now happens
**after** clearance edits, fixing the previous pilot path's pre-clamp selection
eye. The controller, survey UI, interpolation owner and ECS scheduling remain
in the old host; this slice does not claim to have ported them.

The native viewer keeps its existing body-attached orbit camera. Its render
camera is expressed relative to the reference body's current origin, using a
sample of baked translation and the same uniform terrain spin as flight.
Camera/selection and stellar directions are transformed through the same body
frame; light displacement/distance uses ephemerides sampled at the same time.
The absolute camera is never formed by adding a metre-scale offset to the
astronomical body position. Changed sample lighting is uploaded through the
existing direct-light buffer only; unchanged times do no ephemeris/light work.
A failed sample preserves the prior complete pose/lighting. Body motion alone
does not dirty height pages or regenerate geometry.

**Presentation controls, not a second simulation clock.** Defaults still use
`SimTime::EPOCH`. `--sim-time SECONDS` selects a frozen sample in either host
mode. Headless `--sim-step SECONDS_PER_FRAME` adds a deterministic time offset
per rendered frame, including the 30 excluded warm-up frames. Pre-stream uses
the initial sample. Signed time/steps are allowed for diagnostic rewind; all
sample times must be finite. Streaming debounce still uses host elapsed time;
it is not physics time. No wall-time-driven physics, server warp policy or
authoritative state changes are introduced. The HUD and final headless log
report presentation time; logs also report the body's inertial position.

```sh
# Frozen later pose with the normal window/HUD:
cargo run --release -p thessa-terrain-native -- --sim-time 72000
# Sample baked motion/lighting across the run, keeping the camera body-attached:
cargo run --release -p thessa-terrain-native -- --headless moving-body.png --hud --frames 600 --prestream-seconds 10 --sim-step 120
```

**Numerical boundary.** This does not recover detail already lost in absolute
input coordinates: at 2e12 m a binary64 component has ~0.244 mm ULP. Common
translations at the canonical ~4.82e12 m body position have ~0.977 mm component
ULP; the same origin-attached policy avoids adding camera offsets there. Common
translations with representable offsets cancel exactly in the core regression;
the native body-attached origin avoids absolute offset addition altogether.
Moving-frame tests pin round-trip camera error below 4 nm around a 3.2 Mm body
for normal composed unit rotations. The maximum accepted quaternion length
error can contribute micrometre-scale error at that radius; it is not a global
nanometre guarantee. Final GPU geometry still uses the existing f64 tile-camera
subtraction and hi/lo f32 transport; existing height quantization is unchanged.

**Scope limits.** The attitude is the flight runtime's existing **uniform-spin
approximation**, not a new exact tidal-lock/libration, axial-tilt, pole-precession
or prime-meridian model. Gravity harmonics retain their separate existing frame.
Direct terrain lighting still omits eclipses, atmosphere and shadows. The
sampled body is a frame/light source, not a newly rendered planet object.
Multiple moving scene objects, authoritative interpolation/networking, flight
camera/controller migration, game assets/effects/UI, device recovery and FastRT
acceleration structures remain unfinished. No additional device, scene readback
or intermediate color target is introduced; captures/tests alone read back.
The old survey's local orbit offsets and ECS camera rotation are still f32;
this is extraction of their body/rebase boundary, not replacement of every
camera calculation with f64.

The live-flight continuation below supersedes this checkpoint's local moving-
vehicle limitation; native networking and production controller migration
remain open.

Regression coverage includes a hand-computable quarter-turn, representable
astronomical common translations, 100 moving-origin/rotation camera round trips,
invalid/overflow poses and the tolerated quaternion-roundoff envelope; period
precedence and axis handedness; final post-clearance Bevy camera selection;
time-consistent canonical pose/lights and failed/unchanged sample handling; and
CLI finite-time/headless validation. A real GPU fixture compares identical
non-clear raster output across eight moving-body/origin samples, verifies the
draw count and unchanged vertex buffer, and checks that the same height
partition regenerates zero patches. A native GPU regression isolates pose/light
publication, verifies a changed lit image, and checks unchanged height geometry.
Full-loop sampled motion is additionally exercised by the captures below.

The reusable transform benchmark is
`cargo bench -p thessa-render-core --bench frames`: 1024 point transforms versus
direct quaternion/translation arithmetic, plus one origin-local camera mapping.
It excludes ephemeris/stellar sampling, selection/streaming, GPU transfers and
rendering; compiler barriers and the timer are part of the harness.

Final gates passed: fmt, workspace all-target/all-feature clippy with Rust
warnings denied, no-default-feature viewer clippy, **1,748** workspace lib/bin +
**47** integration + **24** Bevy-CBT hardware + **3** native GPU/window tests
(**1,822** successful tests). The native runtime contains no Bevy and still has
one wgpu version (30.0.1). No external dependency, unsafe code, device or render
target was added; the new internal core crate is MIT and depends only on glam.

Radeon 880M stationary A/B/B/A with HUD, 1280×720, 600 measured + 30 warm-up
frames, 10 seconds pre-stream: old medians **2.577 / 2.547 ms**, new
**2.633 / 2.640 ms**. All ended at 287 tiles / 297,216 triangles with zero
measured geometry publications/refusals. New medians were higher; the HUD gained
a telemetry row, and material scheduling/clocks differ, so these runs neither
isolate mapping overhead nor establish a speedup. Microbenchmark medians were
**2.685 µs** for 1024 shared point transforms (direct arithmetic **3.477 µs**)
and **30 ns** for one camera mapping. Compiler-barrier shapes differ; the short
camera sample is near timer-floor scale, not proof of universally free adapters.

With `--sim-step 120`, the complete native loop sampled body motion through
**75,480 s** while keeping the body-attached 287-tile cover and doing **zero**
measured geometry publications; illumination changed with the sample. The
combined camera sweep/rewind (`--sim-time 72000 --sim-step -60 --sweep`) exercised
**55** measured publications with no refusal and ended at 34,260 s. A later-pose
native Wayland window requested 1,200 presentations in 10.793 seconds and exited
0 (**111.2 requests/s**, not displayed frames or game FPS).

The old Bevy client was also built and launched in surface-survey mode with its
existing unattended screenshot/exit path (`THESSA_AUTOBENCH=1`,
`THESSA_AUTOBENCH_VIEW=surface`, `THESSA_SCREENSHOT_AT_S=6`). It rendered a
COAST frame via `gpu_mesh`, saved a screenshot and exited 0. The embedded server
binary was absent, so it used the existing local-simulation fallback; ALSA/JACK
reported device diagnostics. This smoke does not validate networking/audio,
equal-scene visual quality or compare performance.

[Measurements/validation](artifacts/owned-body-camera-frames-2026-10-16.json),
[epoch capture](artifacts/owned-body-camera-frames-2026-10-16.png),
[moving-body capture](artifacts/owned-body-camera-motion-2026-10-16.png), and
[sweep/rewind capture](artifacts/owned-body-camera-sweep-2026-10-16.png).
The [legacy Bevy smoke](artifacts/owned-body-camera-bevy-smoke-2026-10-16.png)
is a separate scene/window, not a side-by-side parity comparison.
The moving sample is deliberately shown without manufactured ambient light;
its very dark direct-lit terrain is not missing coverage or a claim of complete
night-side rendering. Existing visual artifacts remain unaccepted.

### Shared static mesh/rigid-instance continuation (2026-10-16)

The next slice removes X-15 source ownership from Bevy `WorldAsset` without
replacing the old client. MIT `render-core::mesh` defines triangle vertices,
indices, material/image/texture references and rigid-instance inputs without
ECS, asset-server, GPU or game dependencies. MIT `render-assets` decodes a
bounded **embedded static GLB subset** into these descriptors. Node hierarchy
transforms are flattened once; inverse-transpose normals and mirrored winding
are preserved. Dense accessor/view ranges use checked arithmetic before reader
creation. Input file size, vertices, indices, nodes, image edges and total
decoded RGBA bytes have explicit caller-supplied limits. No URI/file/network
resolution is performed. Sparse accessors, animations/skins, extensions,
non-triangle geometry, attributes other than POSITION/NORMAL/TEXCOORD_0,
emissive and transparent
materials are not part of this subset; unsupported semantic input is rejected.
This is not a general streaming asset manager, hot reload or full glTF support.

The old pilot converts that neutral source to Bevy meshes/images/materials;
base color uses sRGB while normals/metallic-roughness/AO use linear textures.
Its bridge generates tangents and preserves double-sidedness and shared ORM/AO
images. Non-unity normal/AO strengths are explicitly rejected by that bridge
rather than silently lost. Source loading is synchronous once during startup,
not asynchronous Bevy loading. Other assets (including docking demo WorldAsset)
remain unmigrated. The original X-15 calibration and visual basis move unchanged
to GPL `flight-authority::presentation`; they are not copied into MIT libraries.
Regression inspection found the GLB's actual X extent is **17.794085 m**, not
the inherited nominal `16.77 m` source-length constant. The old `15.45/16.77`
scale remains for compatibility, yielding ~16.393 m visual extent; this slice
does not claim a corrected physical mesh calibration.

MIT `render-wgpu::mesh::MeshRenderer` uploads persistent vertex/index/albedo
buffers/textures once. A bounded instance buffer receives complete validated
pose publications, with f64 camera subtraction before f32 narrowing. Invalid
camera/light/instance/capacity updates preserve the prior GPU data and draw
count; empty publication produces no mesh pass. Each primitive is drawn with
one indexed instanced draw. `draw_stats()` and the final native log expose
submitted instances, draw calls, triangles and frame-upload bytes; existing
`drawn_triangles` still counts terrain only. The same device, queue and encoder load the terrain
color **and depth**, draw opaque meshes, then draw HUD. There is no scene
readback, intermediate scene texture or second device. An absent preview creates
no mesh GPU resources and performs no per-frame mesh work. Frame staging reuses
its CPU allocation. Device recovery, asset unloading, culling/LOD, heterogeneous
asset registries and indirect scene submission are future layers.

The native shader is deliberately only **sRGB base-color + Lambert direct/
ambient lighting**, sharing the terrain exposure and body-space stellar sample.
Normal/ORM/AO sources are retained but not uploaded/used by this shader; no
normal mapping, metallic BRDF, shadows, mip-chain generation or material-parity
claim is made. This is diagnostic basic mesh bring-up, not a new configurable
production effect. Any production material/effect extension must enter the
shared requested/resolved graphics quality pipeline before promotion.

`--vehicle-preview` loads the shipped source and samples the **existing**
`FlightAuthority::new` initial position and attitude at epoch, not a decorative
animation or second integrator. This is the constructor's north-pole
compatibility snapshot, **not** the old client's later PostStartup COAST-site
relocation (`initialize_launch_site`). Position/attitude and asset basis are mapped
through the same flight/render convention as the old pilot. The camera orbits
that snapshot at 8–180 m (reset 32 m); normal terrain-only distances are unchanged.
Nonzero presentation time/step is rejected before graphics loading. Moving
instance translation/rotation is exercised by GPU regressions, but live flight,
snapshot interpolation, input/network lifecycle and controller/UI migration
remain unfinished. Do not describe this snapshot as a complete moving-object
production consumer.

The live-flight continuation below adds a separate `--flight` mode and shared
pose/interpolation inputs without changing this frozen preview's contract.

Regressions pin the shipped geometry/image/material inventory, nested nonuniform
and mirrored node transforms, image/sampler/color-space roles, file/resource
budgets, invalid byte ranges/component types/indices and external-URI rejection.
Real GPU coverage checks exact textured pixels, retained existing color/depth,
instance translation/rotation, identical output after an astronomical common
translation, multiple/empty instances and atomic rejection. A native hardware
test draws the actual model over terrain. The exact-pole starter pose exposed
an undefined geographic east; the preview uses a finite orthonormal tangent
there, and close-camera regressions cover all orbit quadrants/distances.

```sh
cargo run --release -p thessa-terrain-native -- --vehicle-preview
cargo run --release -p thessa-terrain-native -- --vehicle-preview --headless x15.png --hud --frames 600 --prestream-seconds 10
cargo bench -p thessa-render-assets --bench decode
cargo bench -p thessa-render-wgpu --bench mesh
```

The X-15 source has two primitives, 86,214 vertices, 78,465 triangles and six
2048² source images (**96 MiB** decoded RGBA). Native uploads only the two
base-color images (**32 MiB**, one mip), plus ~3.53 MiB geometry; Bevy consumes
all six material images. The source CPU snapshot remains resident in the viewer;
source residency/retirement and mip generation are explicitly not optimized yet.
No new unsafe code or external dependency version is introduced. Decoder
dependencies are permissive; reusable libraries remain MIT and game code GPL.
The model remains **CC BY 4.0**, by **cmoreau**; attribution/source/license links
in `assets/models/north_american_x-15_plane.LICENSE.txt` must accompany distribution.

Validation gates: fmt, workspace all-target/all-feature clippy with Rust
warnings denied, no-default-feature viewer clippy, **1,753** workspace lib/bin +
**52** integration + **24** Bevy-CBT hardware + **4** native GPU/window tests
(**1,833** successful tests). The native runtime still contains no Bevy and
uses one wgpu version (30.0.1).

Radeon 880M measurements, 1280×720: terrain-only-with-HUD A/B/B/A against
`6608a48`, 600 measured + 30 warm-up frames and 10 s pre-stream per run, gave
old medians **2.582 / 2.605 ms** and new **2.558 / 2.612 ms**. Every run ended
at 287 tiles / 297,216 terrain triangles with zero measured geometry
publications/refusals. The distributions overlap; asynchronous material work
and unlocked clocks prevent a speedup or universal zero-overhead claim.

The normalized static-mesh microbenchmark (120 measured + 30 warm-up frames,
host update/encode/submit/device-completion latency, no readback) measured
**0.041 ms** for the zero-instance draw floor, **0.157 ms** for one X-15
(78,465 triangles), and **2.712 ms** for 32 instances (2,510,880 triangles).
It excludes asset decoding/upload, terrain, UI and streaming; this zero-lit,
single-mip synthetic workload is not GPU-timestamp timing, production scene
parity or a comparison against Bevy/Solari/FastRT acceleration structures.
Full static CPU source decode measured **140.452 ms median**, 10 samples,
excluding file IO, GPU upload and source destruction.

The complete native initial-flight preview with HUD measured **3.544 ms**
median; the camera sweep measured **3.062 ms**, with 58 measured terrain
publications and no refusals. Both submit one mesh instance, two draws and
78,465 mesh triangles, with **640 bytes** of successful frame uploads. They
have a different terrain cover/camera from the A/B runs, so subtracting those
timings does not isolate mesh cost. The dark polar ground and mostly dark
reverse view reflect the unchanged direct-only illumination: no atmosphere,
sky/ambient model or shadows were manufactured for these screenshots. Geometry
visibility is tested separately with controlled GPU fixtures; these captures
are not final visual acceptance.

The native Wayland preview requested 1,200 presentations in **10.226 s** and
exited 0 (117.3 requests/s, **not** display feedback or game FPS; whole-process
elapsed time was 11.838 s). The debug Bevy client loaded the shared model,
rendered its paused pilot scene, saved a screenshot and exited 0. Its embedded
server binary was absent, so the existing local-simulation fallback ran;
ALSA/JACK diagnostics mean audio/networking were not validated. The COAST pilot
and native constructor snapshot have different poses/cameras/lighting and are
not a side-by-side material/visual parity test. Existing old-client sky/terrain
artifacts are not repaired by this asset slice.

[Measurements/validation](artifacts/owned-mesh-assets-2026-10-16.json),
[terrain-only frame](artifacts/owned-mesh-terrain-2026-10-16.png),
[initial X-15 preview](artifacts/owned-mesh-x15-2026-10-16.png),
[camera-sweep preview](artifacts/owned-mesh-x15-sweep-2026-10-16.png), and
[legacy pilot smoke](artifacts/owned-mesh-bevy-pilot-2026-10-16.png).

### Live flight-presentation continuation (2026-10-16)

The next slice adds an actual moving-vehicle consumer without introducing a
second integrator. GPL `flight-authority::presentation::FlightPose` is a checked
immutable visual sample: `SimTime`, f64 reference-body-relative inertial
position and unit authoritative attitude. Local hosts copy the authority's
existing precise relative coordinate. Network hosts subtract baked body
translation at the snapshot's **own** time, retaining the existing absolute
wire-position precision limit rather than pretending to recover lost bits.
Conversion into the existing pilot visual basis is shared game policy; no
clock, queue, ECS or renderer handle enters this semantic input.

The old pilot publishes both local and interpolated-network poses through this
contract. Failed ephemeris/pose conversion now preserves the **complete** prior
position/attitude/time instead of updating attitude/time while retaining an old
position. This never adopts a presentation sample into authority. Existing
snapshot interpolation arithmetic moves unchanged to GPL
`flight-net::presentation::interpolate_snapshot`: clamped position/velocity/
angular-velocity lerp, normalized shortest-path attitude slerp and interpolated
time, with all other metadata copied from the latest snapshot. The old receive
timestamp/inbox/one-interval delay owner stays in the embedded link. There is
no new interpolation queue, extrapolation, prediction or wire-format change.

Native `--flight` initializes the **same** `FlightAuthority::new` plus
`initialize_world_site` COAST setup used by the old client's PostStartup path.
It shares the canonical immutable `Arc<PlanetField>` already owned by the
terrain stream; no duplicate field generation or second terrain/physics source
is introduced. `advance_with_budget` uses the existing fixed-step equations,
resources, aero, thermal/contact and Navball attitude-hold control. This is a
local diagnostic flight host, not a second solver or a multiplayer authority.
There are no new flight-input controls, controller/UI port or autopilot mode.
`--vehicle-preview` keeps its previous frozen north-pole constructor snapshot.

The completed authority time stages one celestial/body-frame/light sample,
one rigid mesh instance and the tracking orbit-camera center/basis. Body-space
mesh rotation includes the inverse sampled body spin, not only translation.
All values publish together after validation. Renderer/stream selection sees
the same body-local camera and light frame as the mesh. An unchanged completed
time skips pose/ephemeris/light publication; an invalid demand changes neither
authority nor presentation. A physics error stops further demand, clears engine/
manual command/backlog like the old host and retains the last complete visual
sample; it is not an authoritative rollback. Final logs report authority and
presentation time, position, step/backlog and stop status. HUD distinguishes live
flight and a stopped last-valid view; it does not mutate flight commands.

Window mode feeds elapsed demand through the old local client's **0.1 s clamp
and 8 ms cooperative work budget**, at 1× requested time. Suspended surface
recreation resets the elapsed baseline; long drawing gaps do not create an
unbounded catch-up queue. This is not the server's shared warp policy. Headless
mode requests deterministic elapsed time per rendered frame, default **1/60 s**,
including the 30 warm-up frames, with no wall work budget. `--flight-step` is
headless-only, requires `--flight`, and must be finite with `0 < step <= 0.1 s`.
Pre-stream remains frozen at launch; physics only advances in rendered frames.
The authority owns the actual lattice time and remainder, not the frame count
or requested elapsed total. `--sim-time` / `--sim-step` must remain zero in live
flight; `--flight` and `--vehicle-preview` are mutually exclusive. These are
validation controls, not a new configurable graphics effect or warp system.

Local presentation deliberately follows the latest completed local state,
matching the old client's `sync_view`; it does **not** apply network delay or
invent another smoothing scheme. The shared network interpolation is still
consumed by the old embedded path, not a claim of native network transport.
Live remote snapshots, fleets, prediction/reconciliation, camera/controller
migration, player inputs, production UI and the remaining client lifecycle
remain unfinished. The orbit camera, simple base-color/Lambert material,
inherited asset calibration and unported atmosphere/shadows remain unchanged.

```sh
cargo run --release -p thessa-terrain-native -- --flight
cargo run --release -p thessa-terrain-native -- --flight --headless flight.png --hud --frames 600 --prestream-seconds 10
cargo run --release -p thessa-terrain-native -- --flight --flight-step 0.008333333333333333 --headless flight-120.png --frames 1200
cargo bench -p thessa-flight-authority --bench presentation
```

Regressions compare the native canonical live host against direct production
authority advancement, checking exact completed time/relative position/attitude,
nonstatic motion, source-geometry retention, shared sample/light/body/camera
publication and rejection atomicity. Unit tests pin relative-coordinate
preservation, the millimetre-scale absolute-source error envelope at canonical
astronomical positions, sample-time ephemeris subtraction, invalid samples,
interpolation metadata/endpoint/clamp/antipodal-quaternion behavior and the old
pilot's complete-pose failure regression. CLI misuse is rejected before loading
assets/GPU. Hardware tests draw the actual advancing model into the existing
terrain color/depth targets, alongside the previously tested generic moving
rigid-instance path. No new unsafe code, external dependencies, device, scene
readback or intermediate scene target is required.

**Validation and measurements.** On the Radeon 880M, the terrain-only 1280×720
HUD A/B/B/A run (600 measured + 30 warm-up frames, 10 s pre-stream) measured
baseline **2.581 / 2.684 ms** and new **2.646 / 2.636 ms** medians. Every run
ended at 287 tiles / 297,216 terrain triangles with zero measured geometry
publications or capacity refusals. These overlapping measurements include async
material work and unlocked clocks; they establish neither a speedup nor an
isolated adapter tax. The preserved baseline executable is the exact `77ee5d4`
build previously recorded in the static-mesh checkpoint.

The live local-flight and live-plus-camera-sweep captures measured **3.066 /
3.176 ms** medians, reaching **10.5 s** completed authority/presentation time
with identical final relative coordinates and no stopped flight. They include
full local physics demand, streaming, rendering, HUD and GPU completion, **not**
isolated renderer time or game FPS. Camera movement and terrain detail differ
from terrain-only and frozen-preview workloads; do not subtract these numbers
to infer physics/mesh cost or compare them as an equal scene to Bevy. Both runs
submitted one mesh instance / two draws / 78,465 mesh triangles / 640 frame
upload bytes. Their 62 / 83 measured terrain geometry publications had zero
capacity refusals. The sweep ended with 8 height / 3 material jobs pending,
so its final cover was still converging rather than proof of completed residency.

The checked-pose microbenchmark measured **90 ns** median for one pose and
**21.410 µs** for 1,024 poses (1,000 samples each; p95 **101 ns / 45.676 µs**).
This covers validation/axis/visual-basis conversion and timer/barriers only,
excluding physics, ephemeris, assets, streaming, GPU upload/render and waits.
Native Wayland live-flight smoke requested **1,200** presentations in **27.066 s**
(**44.3 requests/s**, not display/game FPS), exited 0, and finished at **12.4 s**
completed simulation with **0.002792447 s** backlog and no flight stop/refusal.
Window elapsed clamping and cooperative scheduling do not promise real-time
simulation; this run does **not** establish 1× realtime throughput. No new
investigation or optimization of that host scheduling/performance is claimed.

Captures and machine-readable methodology/results are recorded in
`artifacts/live-flight-presentation-2026-10-16.json`.

Final gates passed: fmt, workspace all-target/all-feature clippy with warnings
denied, no-default-feature viewer clippy, **1,758** workspace lib/bin + **52**
integration + **24** Bevy-CBT hardware + **4** native GPU + **2** native-window
tests = **1,840 successful tests**. Native-window regressions exercise both
HUD/terrain-only and live-flight HUD/no-HUD teardown; live tests require
positive and equal authority/presentation time with no stopped flight.
Local review covered authority/presentation ownership, sample atomicity,
sample-time precision, body-space rotation, source-Arc reuse, host clock
boundaries, the unchanged network receive policy and GPL/MIT separation.
The debug legacy Bevy pilot also ran **unpaused**, saved a flight screenshot
and exited 0 using the existing local fallback (embedded server executable
absent). Its different camera, game UI, effects and elapsed time are not visual
or equal-workload performance parity; audio and networking were not validated.

![Native live COAST flight at 10.5 s, shared authority pose](artifacts/live-flight-flight-2026-10-16.png)

![Same completed flight time, swept tracking orbit camera](artifacts/live-flight-flight-sweep-2026-10-16.png)

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

The selected shell is winit + egui. winit owns application/window lifecycle,
system decorations and native input. egui-winit bridges widget input/focus,
clipboard, IME and DPI; egui supplies HUD and future editor/debug widgets.
The terrain migration viewer implements the first single-window slice above.
Production flight/menu/editor UI remains unported.

UI state is not the game ECS.

```text
egui state != simulation world
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

Incrementally replace the production platform integration using the validated
migration-viewer path:

```text
Bevy window/input/UI
        ↓
winit + egui + owned renderer
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
> Native Vulkan is the reference GPU backend, wgpu is the portability backend, Rayon is the CPU execution backend, and winit + egui is the selected platform/UI shell.
> The migration is incremental: replace one subsystem at a time and never create work that must be rewritten merely to remove Bevy.
