# 21 — Terrain streaming throughput and representation split

Status: design baseline; invariants normative. Throughput phasing is archival:
GPU-indexed CBT is implemented opt-in (`terrain=gpu_indexed`); the CPU baking
description below applies to the fallback path only.

Status: **design target / performance follow-up**.

The goal of this doc is to increase the travel speed at which realtime terrain manages to load correctly, without degrading continuity and without turning `PlanetField` into a set of pre-baked rasters.

The current vertical slice already demonstrates an important result: fully procedural terrain, including geometry, albedo, roughness, normal maps, mip generation and upload preparation, keeps up with the vehicle up to roughly **3 km/s** on the current machine. For a first implementation this is a good baseline. The problem is not that the cube-sphere or the canonical field are slow per se; the problem is that the client streaming frontier currently does too much work, some of which is not CPU tile-build work at all.

Critical clarification: **render streaming, authoritative surface queries and contact representation are different consumers of one canonical surface**. A dedicated/headless server is not required to have a GPU or a render mesh, but the surface does not disappear without an observer: it is needed for automation, terrain avoidance, landings and contact dynamics.

Main target scheme:

```text
                         PlanetField
                canonical observer-independent field
                              |
             +----------------+----------------+
             |                |                |
             v                v                v
      server/query path   contact path      client renderer
      height/slope/etc.   local CPU patch   visual geometry
      automation/GNC      wheels/legs/body  + material
             |                |                |
             |                |                +--> GPU material/microdetail
             |                |                `--> optional GPU CBT/tessellation
             |                |
             `----------------+--> authoritative physics

No camera is required for the left or middle branches.
```

For the client the goal remains simple:

```text
client streaming frontier = geometry bandwidth problem,
not per-tile texture baking problem
```

## 1. What already works

The current system has the right invariants, they must not be lost for the sake of speed:

- canonical terrain is a pure function of physical direction + wavelength, and a tile is only a cache/address unit;
- the same physical point has the same surface regardless of camera, LOD, sampling order and renderer presence;
- cube-sphere eliminates equirectangular seam and pole pinching;
- a coarse sample is a frequency-prefix of a finer sample, not a different surface;
- global subtraction is performed in `f64`, tile-local geometry is stored in `f32`;
- a parent is retained until replacement children are ready, so visual LOD transition does not create holes;
- coarse horizon cover and fine camera-weighted selection are separated;
- async workers do not block the frame thread;
- cache is bounded;
- authoritative geometry/semantics and the renderer use one canonical physical surface, not independent random fields;
- `FlightAuthority` can already query `PlanetField` without a renderer; the current terrain-impact guard is the first headless consumer of the canonical surface.

These properties matter more than the specific implementation of the current renderer path.

Main architectural rule:

```text
camera visibility may control render work,
but must never define whether physical terrain exists.
```

## 2. Where the client streaming budget is currently spent

A single new render tile currently means much more than a quad mesh.

In simplified form the current worker does:

```text
request TileKey
    |
    +--> build_tile()
    |      + height samples
    |      + normals
    |      + indices / skirts
    |
    +--> build_surface_texture_for_mesh()
    |      + per-texel height prefix
    |      + fine terrain sample
    |      + climate/material evaluation
    |      + residual height for normal map
    |      + grain noise
    |      + albedo
    |      + roughness
    |      + normal
    |
    +--> mesh_from_tile()
    |      + mesh assembly
    |      ` tangents
    |
    +--> surface_image() x3
           ` CPU mip generation / upload-ready images
```

At `L13+` the texture tier is currently 128 px plus apron. Hence a single tile contains roughly 17k material texels, each of which runs several procedural evaluations. In flight streaming this easily becomes substantially more expensive than the mesh geometry itself.

Increasing the worker count can improve throughput on free cores, but it does not change the asymptotics and quickly starts competing with flight/server/client work for CPU and memory bandwidth.

This bottleneck concerns primarily **client visual streaming**. It cannot be fixed by moving the authoritative surface to the GPU: the server and automation still need CPU access to the canonical field.

## 3. Representation bandwidth and wasted resolution

Canonical terrain height currently has physical micro bands down to tens of meters. At the same time near tiles may have mesh vertex spacing and texture texel spacing substantially smaller than this wavelength.

This is not a bug by itself: mesh subdivision is needed for projection, curvature, skirts and future shorter bands. But once geometry resolution becomes finer than the canonical height bandwidth, further increasing subdivisions **adds no new surface shape**.

Spatial-frequency responsibilities need to be split explicitly:

```text
large / medium scale
    authoritative height field
    -> physics queries / contact base / render displacement

medium / near visual scale
    GPU procedural normal / roughness / color modulation

object scale
    deterministic scatter / rocks / debris / vegetation / structures

sub-object scale
    material normal / roughness only
```

A guideline, not a hard contract:

```text
> ~32 m          canonical geometric height
~2..32 m         GPU visual relief / detail normal, optional bounded displacement
~0.2..10 m       deterministic scatter / rocks / local features
< ~1 m           material normal / roughness / albedo microstructure
```

If later collision/gameplay require real geometry below 32 m, the canonical field can be extended with additional bands. But the renderer should not require such an extension just to keep the ground from looking blurry.

Conversely: shader-only displacement below the canonical cutoff must not suddenly become an obstacle for the landing gear. If a feature is physically important, it must exist in the authoritative representation.

## 4. Target split: authoritative surface, contact materialization, render shading

### 4.1 CPU authoritative/query side

`PlanetField` remains a CPU-accessible canonical source of the surface for server/gameplay regardless of the renderer.

Typical consumers:

- height at direction / position;
- surface normal / slope estimate;
- terrain clearance;
- path/trajectory sampling;
- landing-zone evaluation;
- geology/biome/climate semantics where they are gameplay-relevant;
- deterministic placement of large physical surface features.

This is a **query representation**, not a render mesh. In many cases automation can work directly with field samples without materializing a mesh.

Example API direction:

```rust
pub trait SurfaceQuery {
    fn height_m(&self, dir: DVec3, min_wavelength_m: f64) -> f64;
    fn normal(&self, dir: DVec3, min_wavelength_m: f64) -> DVec3;
    fn slope(&self, dir: DVec3, min_wavelength_m: f64) -> f64;
    fn clearance_along(&self, path: &SurfacePathQuery) -> ClearanceResult;
}
```

It is not necessary to introduce exactly this trait now; the semantics matter: query fidelity is defined by the physical task, not by camera LOD.

### 4.2 CPU contact representation

For landing, a single `height_m(point)` is not enough. When a vehicle actually interacts with the surface, the server must be able to obtain a local geometry/contact representation for:

- several landing legs / wheels;
- hull/body contacts;
- local surface normal;
- uneven terrain;
- penetration resolution;
- braking/friction;
- suspension and rolling contacts, with sprung/unsprung bodies in the local
  contact-active vehicle model.

The wheel/strut data and tire-contact acceptance contract is documented in
[`details/05_PROCEDURAL_LANDING_GEAR.md`](details/05_PROCEDURAL_LANDING_GEAR.md).

Target lifecycle:

```text
craft approaches contact envelope
          |
          v
request local authoritative surface patch
          |
          v
PlanetField samples -> CPU contact patch / BVH / height patch
          |
          v
contact solver
          |
          v
craft leaves region -> patch may be evicted
```

This patch is created from **physics need**, not from observer/camera need. On an empty dedicated server a fully automatic landing must work with the same physical surface as with a connected client.

Contact representation does not have to match client render triangulation. What must match is the physical height/feature field within the stated error bound.

### 4.3 Automation without an observer

Automation is a separate surface consumer and must not depend on whether anyone is watching the vehicle.

Examples:

```text
landing planner
    -> sample candidate zones
    -> slope / roughness / clearance queries
    -> choose approach corridor

terrain-following / avoidance
    -> sample look-ahead path
    -> derive clearance envelope

unobserved scripted landing
    -> query canonical field
    -> request contact patch near touchdown
    -> execute full authoritative contact dynamics
```

This is the correct version of "optimization for when nobody is watching":

```text
no observer
    != no terrain
    != no physics

no observer
    -> no render workload
    -> automation/query workload remains if mission logic needs it
    -> contact workload appears only when physical interaction needs it
```

### 4.4 Client render geometry side

CPU client streaming should mostly produce data needed by the raster path and not yet computed more efficiently on the GPU:

- tile anchor;
- positions / displaced surface geometry;
- normals sufficient for base geometry;
- indices and skirts or replacement seam strategy;
- optional compact semantic weights needed by the material;

A new render tile should not by default create three unique CPU-generated texture assets.

### 4.5 GPU visual side

The surface shader receives continuous world/body-fixed coordinates and computes visual detail directly in planet space:

```text
planet/body direction
physical position / radius
base geometric normal
height / slope proxy
semantic climate inputs (if needed)
world seed / material parameters
```

The first things to move to the GPU are:

- regional/variation noise used only for appearance;
- fine color grain;
- small-scale normal perturbation;
- roughness modulation;
- biome/material blending, if its inputs are available without an expensive authoritative query;
- near tiling / triplanar or spherical-coordinate detail layers.

This does not require turning the world into repeated texture wallpaper. Deterministic procedural noise can remain continuous in physical coordinates; only the consumer changes from a CPU image baker to a shader.

## 5. Canonical field vs renderer field

A clear boundary must be kept:

```text
PlanetField authoritative outputs
    - physical height
    - climate / geology / biome semantics where gameplay cares
    - deterministic physical feature placement

SurfaceVisualField
    - cosmetic micro normal
    - cosmetic albedo grain
    - sub-grid roughness
    - shader-only blending detail
```

`SurfaceVisualField` may be derived from the same seed and physical coordinates, but it **must not change collision/flight terrain height**.

This allows the renderer to be much higher-frequency without forcing terrain queries, the contact solver and the server to repeat the shader workload.

If a visual feature must become gameplay-relevant (for example a large boulder or crater), it stops being shader-only and gets a deterministic object/geometry representation on the authoritative side.

A hidden third surface must be avoided:

```text
BAD:
server terrain != automation terrain != visible terrain

GOOD:
one canonical physical field
    + task-specific representations/caches
    + cosmetic visual detail layered on top
```

## 6. GPU/client implementation options

There is no need to pick one technique for the whole range right away. All options in this section are **renderer implementation**, not a required dependency of the dedicated server.

### Option A — procedural WGSL

Port cheap deterministic noise and material functions to WGSL.

Pros:

- no per-tile texture generation/upload;
- continuous world-space coordinates;
- unlimited effective material resolution near camera;
- simple cache story.

Cons:

- shader ALU cost;
- CPU/GPU bitwise identity is not guaranteed;
- complex climate/province calculations should not be duplicated literally on the GPU.

Therefore the GPU path should use only a cosmetic subset or compact precomputed semantic inputs.

### Option B — shared material textures / arrays

Use a small number of reusable detail textures / texture arrays, tiled/triplanar/spherical blended by semantic weights.

Pros:

- very cheap runtime;
- hardware filtering/aniso/mips;
- well suited for rock/soil/snow/ice microstructure.

Cons:

- source assets are needed;
- repeated texture artifacts need to be broken up by rotation/noise blending;
- less procedural uniqueness.

### Option C — hybrid virtual/detail cache

For expensive visual functions, GPU/compute or a background worker may bake reusable pages into a virtual texture/cache, rather than unique textures strictly 1:1 to a geometry tile.

This makes sense later, if pure shader becomes expensive or authored high-detail regions are needed.

The first target should be simpler: **remove per-tile albedo/roughness/normal baking from the critical flight streaming path**.

### Option D — GPU adaptive terrain / CBT-like tessellation

Concurrent Binary Tree / related GPU-driven adaptive triangulation is interesting as a possible late replacement for client-side cube-sphere render mesh streaming.

The architectural boundary is strict:

```text
PlanetField / authoritative CPU surface
          |
          +--> server queries/contact: CPU only
          |
          `--> client visual consumer
                    |
                    `--> GPU CBT / adaptive tessellation
```

CBT does not become the source of truth and is not required on a dedicated server. It only answers the renderer question: **which set of triangles is currently needed to depict the canonical surface with a given screen-space error**.

Potential benefits:

- continuous/adaptive visual triangulation instead of fixed tile mesh density;
- GPU split/merge based on projected error;
- less CPU geometry churn on fast flyover;
- natural path to very fine near-view triangulation without huge numbers of independently built CPU meshes.

But CBT does not automatically solve:

- authoritative contact geometry;
- automation terrain queries;
- procedural height evaluation cost if every vertex still requires expensive field evaluation;
- material detail;
- physical objects/scatter;
- server CPU performance.

Therefore this is **not a Phase 1 optimization**. First the CPU material baking needs to be removed and the remaining geometry bottleneck measured.

## 7. High-speed visual streaming must be predictive, not only reactive

The movement trigger currently quickly realizes that the camera has moved far, but the selection/build pipeline mostly reacts to the current eye/frustum. At km/s speeds the vehicle covers a significant distance within one build latency.

This section concerns client visual representation. Automation/physics prediction has its own query horizons and must not use the camera streaming queue as a source of the surface.

A bounded look-ahead in the direction of camera/vehicle motion is needed.

Target selection inputs:

```rust
pub struct TerrainStreamingView {
    pub eye_body_m: DVec3,
    pub forward_body: DVec3,
    pub velocity_body_mps: DVec3,
    pub angular_velocity_hint: DVec3,
    pub fov_rad: f64,
}
```

The predicted eye is computed from velocity and measured build latency:

```text
lookahead_s = clamp(p95_tile_ready_latency * safety_factor,
                    min_lookahead,
                    max_lookahead)

predicted_eye = eye + velocity * lookahead_s
```

Selection must reserve part of the tile budget for the corridor between `eye` and `predicted_eye`, not just shift the whole frustum forward.

Otherwise a fast craft during a sharp pitch/yaw may have perfect terrain ahead of the trajectory, but hole/coarse ground in the current frame.

Example budget split:

```text
coarse guaranteed cover       fixed
current-frame fine detail     ~50-65%
predictive velocity corridor  ~25-40%
turn / reserve                remainder
```

Exact shares must come from benchmarks, not from this doc.

## 8. Selection generation must not wait for nearly empty workers

The current guard of the form `jobs.len() <= 2` is useful as backpressure, but at high speed it may delay a new selection even when the movement trigger already knows that old jobs are becoming less valuable.

A priority scheduler is needed, where queued work can be reprioritized or dropped before execution starts.

There is no need to cancel an already running CPU job. It is enough to distinguish:

```text
Running jobs     small bounded set; usually finish
Queued requests  mutable priority queue; stale requests may be discarded
Ready cache      reusable if still spatially relevant
```

Priority should account for:

- coverage necessity;
- current frame projected error;
- predicted future projected error;
- distance/time-to-enter view;
- parent availability;
- tile build cost estimate;
- whether tile is already partially cached.

An old request for detail behind the vehicle must not block a coarse/fine tile ahead of it just because it entered the queue 200 ms earlier.

## 9. Different work classes need different priorities

Coarse coverage, geometry refinement and cosmetic detail have different criticality.

Client render classes:

```text
P0  coverage repair / missing visible parent
P1  visible geometry refinement
P2  predicted visual geometry corridor
P3  visible cosmetic/detail preparation
P4  predicted cosmetic detail
P5  render cache warming / optional work
```

After the GPU material split, classes P3/P4 should become almost free for the CPU, which is exactly what frees throughput for P0-P2.

Authoritative/contact jobs must not compete in this queue under the same priorities. Server physics has a separate scheduler/budget; contact preparation for an imminent touchdown is more important than any cosmetic render work.

## 10. Geometry build optimization after the representation split

Only after removing texture baking from the client critical path does it make sense to seriously optimize the render mesh builder.

Candidates:

- batch evaluate height samples for several tiles;
- SIMD-friendly noise sampling where profitable;
- reuse parent samples in children;
- preserve edge samples exactly to avoid duplicate evaluation;
- precompute/reference static index buffers for fixed `TILE_CELLS`;
- avoid per-tile tangent generation if final material uses world/triplanar mapping and tangents are unnecessary;
- reduce mesh attributes to exactly what shader consumes;
- GPU compute mesh generation only if CPU remains bottleneck after simpler fixes;
- later evaluate CBT-like adaptive GPU triangulation if fixed-tile mesh churn remains dominant.

It is especially important to check tangents: if near terrain material moves to world-space/triplanar normals, tangent-space may no longer justify CPU `generate_tangents()` for every tile.

Server-side optimization is considered separately:

- batch/SIMD field queries for automation;
- cached local contact patches;
- reuse samples across nearby wheels/legs;
- bounded refinement from predicted contact time;
- no render-only albedo/normal work.

## 11. Parent/child visual transition

The current finest-ready cover guarantees no holes, but replacement is still discrete. After the throughput work, the perceptual transition can be improved separately:

- geomorph parent -> child;
- short cross-fade/dither for material/detail only;
- shared-edge displacement constraints;
- skirts only as a fallback, not as the primary visual seam mechanism.

But this **must not block the throughput refactor**. Hole-free discrete replacement is better than a beautiful morph that does not manage to stream in time.

Contact representation does not have to repeat this visual transition: physics must see a stable canonical surface, not the dither/morph state of the renderer.

## 12. Instrumentation required before and after refactor

Streaming cannot be judged by maximum vehicle speed alone. Client render metrics need to be recorded:

```text
terrain.requested_tiles
terrain.queued_tiles
terrain.running_jobs
terrain.ready_tiles
terrain.visible_tiles
terrain.stale_requests_dropped
terrain.cache_hit_rate

terrain.tile_geometry_ms p50/p95/p99
terrain.tile_material_ms p50/p95/p99
terrain.tile_upload_ms p50/p95/p99
terrain.request_to_visible_ms p50/p95/p99

terrain.viewer_speed_mps
terrain.lookahead_s
terrain.coverage_age_s
terrain.max_projected_error
terrain.visible_coarse_fallback_count
```

The following metric is especially useful:

```text
time_to_needed = distance_to_future_view / viewer_speed
```

A tile misses its deadline if `request_to_visible > time_to_needed`, even if the build benchmark itself looks fast.

Separate metrics are needed for the headless/server surface:

```text
surface.query_count
surface.query_ms p50/p95/p99
surface.contact_patch_build_ms p50/p95/p99
surface.contact_patch_cache_hits
surface.contact_patch_cache_misses
surface.automation_samples_per_sim_s
surface.contact_refinement_deadline_misses
```

It is important not to mix up `request_to_visible` with `request_to_contact_ready`: they have different consumers and different deadlines.

## 13. Benchmark scenarios

### 13.1 Client visual streaming

Minimal repeatable set:

1. **Hover / survey** — near-zero speed, aggressive near refinement.
2. **Aircraft** — 200-400 m/s at 1-10 km AGL.
3. **Fast atmospheric craft** — 1 km/s.
4. **Current stress point** — 3 km/s.
5. **Target supersonic/hypersonic** — 5 km/s and 8 km/s.
6. **Low-altitude pathological pass** — high speed + low AGL.
7. **Fast turn** — speed is high, camera/vehicle heading changes sharply.
8. **Vertical descent/ascent** — projection footprint changes faster than ground-track distance suggests.

For each:

- no holes;
- bounded cache/jobs;
- visible projected-error distribution;
- p95 request-to-visible latency;
- CPU frame cost;
- total worker utilization;
- GPU material cost after split.

### 13.2 Headless authoritative surface

Separate repeatable set without GPU and without client:

1. automated descent from high altitude;
2. terrain-following trajectory query;
3. landing-zone search over configurable radius;
4. final approach with predicted contact patch prefetch;
5. multi-leg touchdown on sloped/rough surface;
6. rolling/braking contact once wheel dynamics exist;
7. multiple unobserved vehicles approaching different surface regions;
8. high warp far from contact, then transition to full surface/contact fidelity before touchdown.

For each:

- identical canonical surface with and without connected client;
- deterministic query results for fixed seed/state;
- bounded contact patch memory;
- no GPU dependency;
- contact representation ready before physical interaction;
- automation result independent of camera position/view mode.

## 14. Performance targets

The target should not be set as "must be faster than KSP", because world complexity, hardware, visual quality and architecture differ.

### 14.1 Client visual contract

```text
At declared maximum active-flight speed and minimum supported AGL,
terrain rendering must keep a complete visible cover and satisfy the
projected-error target for the central view with bounded latency and memory.
```

First practical target after the GPU material split:

- keep current visual/geometric fidelity;
- at least double sustainable ground speed relative to the current ~3 km/s baseline on the same machine;
- do not increase the tile cache bound just for throughput;
- do not raise the worker count as the sole optimization;
- keep main-thread terrain insertion cost small and bounded.

After that, separately measure 8-12 km/s atmospheric/near-surface stress cases.

### 14.2 Headless/server contract

```text
Surface physics and automation must remain fully functional with no GPU,
no renderer and zero connected observers.
```

Additionally:

- far-from-surface automation uses bounded direct queries, not materialized render terrain;
- contact representation is created in advance from physics/trajectory need;
- landing result does not change due to spectator/client connecting/disconnecting;
- server does not pay for cosmetic material detail;
- fidelity transition is defined by physical error/deadline, not by camera LOD.

## 15. Suggested implementation order

### Phase 1 — measure actual client bottleneck

1. Add separate p50/p95 timings for geometry/material/mips/upload.
2. Record request -> visible latency.
3. Make fixed 3 km/s and 5 km/s benchmark routes.

### Phase 2 — remove CPU material baking from client critical path

4. Make a prototype terrain shader with shared/procedural near detail.
5. Remove per-tile albedo/roughness/normal creation for the prototype path.
6. Check whether tangent generation is still needed after the new mapping.
7. Compare tile throughput and total CPU.

### Phase 3 — predictive client scheduling

8. Add velocity look-ahead corridor.
9. Split queued vs running jobs.
10. Allow discard/reprioritize of stale queued requests.
11. Keep guaranteed coarse cover regardless of prediction.

### Phase 4 — formalize headless surface consumers

12. Extract/fix the observer-independent surface query boundary for automation/flight.
13. Add normal/slope/clearance helpers with explicit wavelength/error request.
14. Design the local contact patch representation and cache lifecycle.
15. Prefetch contact patch by predicted time-to-contact, not by camera distance.
16. Add headless landing/contact benchmarks without GPU.

### Phase 5 — geometry hot path

17. Profile `PlanetField` height-only sampling separately for render and server query workloads.
18. Add sample reuse / batching / SIMD only for measured hotspots.
19. Check parent->child sample reuse on the client and nearby-query reuse on the server.
20. If client mesh churn remains the bottleneck — spike GPU compute/CBT-like adaptive triangulation.

### Phase 6 — visual quality

21. Add GPU micro-normal/material bands below the geometric cutoff.
22. Then scatter/rocks.
23. Only then geomorph/cross-fade, if LOD replacement remains noticeable.

## 16. Non-goals

This refactor must not:

- turn canonical terrain into a fixed global raster;
- make the renderer or GPU-tessellation the source of authoritative height;
- require a GPU on a dedicated server;
- treat absence of an observer as absence of physical terrain;
- tie automation fidelity to camera position/LOD;
- force a headless server to build visual terrain tiles for landing;
- synchronize cosmetic shader noise over the network;
- generate the whole future flight corridor upfront without a bounded budget;
- hide streaming failures with a giant cache;
- increase detail frequency in physics just for visuals;
- require contact triangulation to match render triangulation if both approximate one canonical surface within the given error bound.

## 17. Key invariant

Instead of the old split of "CPU asks physics, GPU asks picture", a slightly more precise contract is needed:

```text
Authoritative physics asks:
    "what is the physical surface here and how to contact it?"

Automation asks:
    "what surface will be along my trajectory / in the landing zone?"

Renderer asks:
    "how to present this same canonical surface on screen right now?"
```

All three questions use one `PlanetField`, but **do not have to use one representation**.

```text
canonical field
    + direct queries          -> automation / clearance / planning
    + local contact patch     -> landing / wheels / body collision
    + visual LOD / CBT / mesh -> renderer
    + cosmetic shader detail  -> pixels only
```

This allows optimizing each task independently:

- nobody watches -> render branch costs zero;
- automation still runs -> query branch remains active;
- touchdown approaches -> contact representation materializes;
- client connects -> visual representation builds without changing physics.

The next step is not to abandon procedurally, but to **move each kind of procedural work where its cost matches its consumer: queries/contact to CPU server side, visual frequency/detail to GPU client side**.

## 18. UMA / unified-memory asset residency follow-up

Apart from the LOD algorithm and the procedural field, the current client path has a lower-level opportunity: to reduce the number of **logical copies of render data**. On UMA this matters especially because CPU and GPU compete for one DRAM pool and one memory bandwidth, but the win is also useful on dGPU as system RAM and memcpy savings.

Three tasks need to be distinguished:

```text
A. lifetime / residency
    do not retain CPU payload after the immutable asset has been prepared by the renderer

B. transient build copies
    do not clone large Vecs between TerrainTile / SurfaceTexture / Mesh / Image

C. true zero-copy / mapped GPU memory
    separate late optimization; do not assume that A or B automatically
    turn the Bevy/wgpu upload path into zero-copy
```

### 18.1 Current code-specific opportunities

Terrain textures are already created as `RenderAssetUsages::RENDER_WORLD`, so after extraction/preparation their CPU-side pixel payload can be discarded. Terrain mesh is meanwhile created as:

```rust
RenderAssetUsages::MAIN_WORLD | RenderAssetUsages::RENDER_WORLD
```

This leaves the vertex/index payload accessible in `Assets<Mesh>` even after the render representation is prepared. For an immutable terrain tile this looks redundant: `CachedTile` already stores `vertices` and `triangles`, and `WorldTerrain` already counts visible counters from cache metadata.

A separate perf path still walks `Assets<Mesh>` for `count_vertices()` / `indices().len()`. This should not be a reason to keep the full CPU mesh resident. These counters can be taken from `WorldTerrain.visible + WorldTerrain.cache`, after which the terrain mesh can become a render-only asset:

```rust
Mesh::new(
    PrimitiveTopology::TriangleList,
    RenderAssetUsages::RENDER_WORLD,
)
```

The target lifecycle then looks like:

```text
worker builds tile
    -> Mesh/Image payload exists on CPU temporarily
    -> Bevy extracts/prepares render asset
    -> CPU payload is dropped
    -> cache keeps Handle + anchor + vertex/triangle/byte metadata
```

This is **not a promise of zero-copy**. Render-world buffer/texture allocation still exists, and the backend may perform staging/upload. The win here is not keeping a second long-lived CPU representation without a consumer.

### 18.2 Avoid worker-local clones before touching renderer internals

The current worker also makes copies even before Bevy asset extraction:

```text
TerrainTile.positions.clone() -> Mesh
TerrainTile.normals.clone()   -> Mesh
TerrainTile.indices.clone()   -> Mesh

SurfaceTexture.albedo.clone()    -> Image
SurfaceTexture.roughness.clone() -> Image
SurfaceTexture.normal.clone()    -> Image
```

After assembly, the original `TerrainTile` / `SurfaceTexture` are no longer the cache representation. So the builder API should be rebuilt around ownership/move:

```text
build_tile() / build_surface_texture()
        |
        v
owned vectors
        |
        +--> move into Mesh
        `--> move into Image
```

For mesh this may mean `mesh_from_tile(tile: TerrainTile, ...)` or destructuring that preserves `anchor`, `vertices`, `triangles` metadata before the move. For texture — pass owned channel vectors to `surface_image()` without `.clone()`.

This reduces transient allocation and memory traffic regardless of GPU architecture. On UMA the benefit is potentially more noticeable precisely because worker CPU traffic and renderer traffic share one memory subsystem.

### 18.3 Memory accounting must describe logical residency, not pretend UMA is discrete VRAM

Current `world.cache_bytes` estimates one mesh payload plus texture bytes. This is a useful logical cache metric, but it does not describe:

- retained CPU mesh payload;
- render-world/GPU allocation;
- temporary worker copies;
- allocator overhead;
- staging/upload buffers;
- physical UMA residency, where there may be no separate independent VRAM pool.

So performance monitoring is better split at least into:

```text
terrain.cache_logical_bytes
terrain.cpu_asset_payload_bytes
terrain.build_transient_bytes_estimate
terrain.upload_bytes_per_s
```

`gpu_mem_bytes`/physical UMA residency should not be synthesized if the backend does not provide a trustworthy number. Logical byte counters still make it possible to verify that the refactor really removed the redundant representation.

### 18.4 Capability-driven UMA fast path — only after profiling

If after the material split, ownership cleanup and render-only assets the measurements show that the bottleneck remains specifically in CPU->GPU upload/copy, then a shared-memory fast path can be investigated separately:

```text
capability detection
    |
    +--> UMA / host-visible device-local memory available
    |       -> mapped/ring-buffer or equivalent upload strategy
    |
    `--> discrete / unsuitable memory type
            -> normal staging/upload path
```

This must be **capability-driven**, not `if vendor == AMD`. Linux/Vulkan, Metal and other wgpu backends remain first-class; a backend-specific shortcut must not leak into `PlanetField`, terrain semantics or server code.

In wgpu/Bevy such a path may require deeper render-asset/custom-buffer integration and explicit synchronization of mapped vs GPU use. So it must not precede the simple measurable changes above.

### 18.5 Suggested low-risk order

Before major renderer rewrites:

1. Switch immutable terrain `Mesh` to `RenderAssetUsages::RENDER_WORLD`.
2. Remove the perf dependency on CPU `Assets<Mesh>` and count visible geometry from `CachedTile` metadata.
3. Remove `.clone()` of large mesh/texture vectors in worker assembly via ownership transfer.
4. Add logical/transient/upload byte metrics.
5. Take identical hover / 3 km/s / 5 km/s captures on UMA and dGPU, if available.
6. Only if upload remains the bottleneck — spike the mapped/shared-memory path.

Acceptance criteria:

- identical visible terrain and canonical physics;
- same visible vertex/triangle counters;
- less retained CPU payload and worker transient traffic;
- RSS/peak memory no worse, expected lower on UMA;
- p95 `request_to_visible` does not regress;
- no vendor lock-in and no GPU requirement for authoritative terrain.
