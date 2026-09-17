# CBT render audit — 2026-09-15

## Scope and reference

Branch: `feat/rcbt-terrain-pipeline`. Fixes were applied on top of the existing
uncommitted renderer work. Simulation, maneuver and graphics-setting changes
already present in the working tree were preserved.

Primary reference: [AnisB/large_cbt](https://github.com/AnisB/large_cbt), inspected
at `7351e6fc603b9b2c2ab4da399b13a9ab0f327398`.
The algorithm is described in [Concurrent Binary Trees for Large-Scale Game
Components](https://arxiv.org/html/2407.02215v1), especially §§2.3 and 3.2–3.3.
Its CBT allocates fixed-size bisector records independently of subdivision
depth. Neighbor propagation maintains a conforming triangulation. This is
fundamentally different from using CPU CBT addresses to schedule square tiles.
The paper's sub-0.2 ms claim concerns triangulation on its measured hardware;
it is not a whole-frame FPS target for the Thessa scene.

## Defects fixed

1. **Wrong GPU vertex stride.** The classifier used
   `ordinal * vertices_per_patch + corner * 2` on an interleaved position/normal
   buffer. Both terms must be multiplied by two. Later leaves were culled or
   assigned triangle density using unrelated positions, sometimes normals.
2. **Incomplete spatial cover.** Any overlapping fine tile could satisfy the
   readiness check for an entire coarse tile. Selection now partitions the
   union into disjoint tiles and retains the siblings along refinement paths.
3. **Replacement and coarsening.** The planning tree previously prevented
   merges underneath requested coarse leaves. A ready presentation snapshot
   now stays independent of later bounded planning commits; it and its pages
   remain resident until the replacement is complete. Selection cannot outrun
   its unfinished replacement.
4. **Invalid and unstable normals.** Forward differences collapsed at the
   last row/column; subtracting neighbouring absolute f32 positions also made
   deep flat tiles acquire false slopes. Analytic cube-sphere derivatives plus
   bounded height differences now avoid both failures. GPU readback checks a
   flat surface's normal against its radial direction to within 1e-5.
5. **Wrong map hemisphere.** The bake stores north at v=0; the shader used the
   opposite latitude sign. Longitude projection now happens after interpolation,
   with wrap-correct gradients and a defined longitude at exact poles.
6. **Unrelated light.** A fixed shader-space light ignored the actual stars and
   camera exposure. The diffuse pass now receives the same three directional
   lights and ambient illuminance as the client scene, in pre-exposed linear HDR.
7. **Avoidable buffer work.** Generation equality, rather than Bevy change flags,
   controls geometry preparation. GPU outputs are reserved directly instead of
   constructing and uploading zero-filled CPU mirrors.
8. **Serial triangle emission.** A workgroup now cooperates on each patch. The
   draw uses one linear vertex stream (`vertex_index / 3`) as in the reference,
   rather than one instance for every triangle.
9. **Misleading telemetry.** Expired GPU-pass samples are excluded from new
   diagnostic batches. With GPU profiling enabled, the triangle counter is read
   asynchronously from the actual indirect stream; the overlay reports GPU
   terrain timing instead of an unconditional “unavailable”.

## Comparison with the primary reference

| Concern | `large_cbt` implementation | Thessa after this fix |
|---|---|---|
| Allocation and topology | Persistent GPU bisector pool, OCBT allocation/reduction | CPU tree and streamed square height pages |
| Refinement continuity | Neighbor-aware split/merge propagation | Complete disjoint tile cover plus skirts; no bisector compatibility chains |
| Geometry work | Active/modified bisector indirection | Full 33×33 grid for each published tile when geometry changes |
| Draw | Single procedural triangle stream | Same one-instance submission shape |
| Shading | Visibility buffer followed by screen-sized material compute | Direct raster, streamed local albedo/roughness array, analytic ocean reflection |
| Precision | Planet/camera calculations in `REAL*_DP`, then camera-relative float | f64 tile anchor subtraction/rotation on CPU, stable local f32 projection on GPU |
| Runtime | DX12 / shader model 6.6 demo | Bevy/wgpu, tested on Linux Vulkan |

Relevant upstream files:

- [mesh update orchestration](https://github.com/AnisB/large_cbt/blob/7351e6fc603b9b2c2ab4da399b13a9ab0f327398/demo/src/mesh/mesh_updater.cpp)
- [update kernels and indirect arguments](https://github.com/AnisB/large_cbt/blob/7351e6fc603b9b2c2ab4da399b13a9ab0f327398/shaders/UpdateMesh.compute)
- [procedural visibility draw](https://github.com/AnisB/large_cbt/blob/7351e6fc603b9b2c2ab4da399b13a9ab0f327398/shaders/Visibility/VisibilityPass.graphics)
- [Earth material pass](https://github.com/AnisB/large_cbt/blob/7351e6fc603b9b2c2ab4da399b13a9ab0f327398/shaders/Earth/MaterialPass.compute)
- [camera-relative deformation](https://github.com/AnisB/large_cbt/blob/7351e6fc603b9b2c2ab4da399b13a9ab0f327398/shaders/Moon/MoonDeformation.compute)

No upstream shader or renderer code was copied into runtime. An isolated CPU
OCBT reduction benchmark would not establish GPU renderer parity, so it is not
used as evidence for whole-frame performance here.

## Validation

```sh
cargo test -p thessa-bevy-rcbt --features render --lib -- --include-ignored
cargo test -p thessa-client --bin thessa-client
cargo build --release -p thessa-client
```

The GPU tests run production WGSL on an actual wgpu adapter. They check signed
height-page interpolation against the CPU sampler, six cube faces at L0/L17,
all border normals, a classifier fixture with only the middle of three leaves
visible, the indirect command and map orientation. The adapter tests cover
publication retention, refinement coverage and bounded coarsening convergence.

Visual/measurement commands (the screenshot variable is optional):

```sh
THESSA_AUTOBENCH=1 THESSA_AUTOBENCH_VIEW=pilot \
THESSA_AUTOBENCH_STATIC=1 THESSA_AUTOBENCH_FULLSCREEN=1 \
THESSA_GPU_PROFILE=1 THESSA_AUTOBENCH_SCREENSHOT=/absolute/path/pilot.png \
target/release/thessa-client
```

Add `THESSA_AUTOBENCH_PAUSED=1` for a stationary scene. For survey scenes use
`THESSA_AUTOBENCH_VIEW=surface` and `THESSA_AUTOBENCH_SITE=0|1|2` (coast,
highlands, volcanic). Captures warm up for six seconds and run for 34 seconds;
a screenshot is requested at 25 seconds. Screenshot readback may affect an
individual frame, so compare captures without it when measuring tails.

## Remaining renderer work

These repairs address the broken visible cover and wasted work; they do not
provide visual or architectural parity with `large_cbt`. The next substantive
change is an actual GPU bisector pool with neighbor propagation and independent
pool capacity/subdivision depth. Its acceptance fixtures should check manifold
coverage and split/merge chains on all cube faces, including capacity exhaustion.
A visibility/material pass remains a separate architectural step. Land still
uses geometric normals and Lambert shading without Bevy PBR shadow parity.
The new ocean is an analytic normal/reflection approximation; it does not
include FFT displacement, foam, underwater rendering or physical waves.


## Additional reference-driven implementation

### Local precision (Earth, Moon and Thessa radii)

`precision.rs` and `precision.wgsl` implement normalized-cube offsets without
subtracting radius-sized f32 vectors. CPU f64 tile anchors are rotated and
translated before narrowing, then the vertex/classifier adds small local
positions. Only 112 bytes per tile change when the camera moves; terrain
geometry does not regenerate for camera motion alone. Body-fixed mapping and
water phase remain separate from render-local positions.

Actual Vulkan compute readbacks cover all six faces at L0 and L17 for radii
1,737,400 m, 3,200,000 m and 6,371,000 m. The L17 fixture projection error is
below 1 mm; the broad L0 fixture tolerance is 2 m. A separate rotated-body,
near-camera fixture is below 0.1 mm. These are projection bounds on the stated
fixtures, not a claim that the sampled/quantized terrain field is millimetric:
its height-page bake budget remains 0.5 m. These tests do not add Earth or Moon
to Thessa's authoritative star system.

### Ocean material

The existing terrain draw now shades water with two filtered analytic wave
normal components, Schlick Fresnel, GGX reflections of the three scene stars,
and a local-up analytic sky approximation. Direct lighting uses the same lux
and exposure as terrain/craft; the sky approximation redistributes a configured
fraction of incident illuminance. Wave phases are bounded in f64 on the CPU
and follow simulation time, including pause. No extra geometry/FFT pass or
wave physics was added. Ice/land are excluded using canonical roughness and
height, rather than an albedo-color heuristic.

The real reference includes FFT wave simulation. Its cost is not represented
as free or included in the paper's triangulation timings. The implementation
here independently follows the reflection/material ideas in its Earth path.

### Local material pages and persistent GPU layers

A separate two-worker material queue refines the visible cover without delaying
height jobs or presentation. Pages contain 128×128 sRGB albedo with linear
roughness in alpha, a border, and eight mip levels generated in linear light.
The GPU array has at most 256 layers, approximately 21.33 MiB including mips.
It allocates only when GPU terrain is enabled. Slot identity follows the tile
ID and source generation, so topology reordering does not reupload textures.
Missing/evicted pages sample the canonical planet map. Pages are removed with
the terrain cache. CPU material bytes are included in terrain cache telemetry.

This removes the coarse planet-map coastline visible in the 50 km coast survey.
Same-level page edges and canonical ocean/land appearance are covered by the
worldgen tests; slot tests cover reorder, eviction, source replacement and
capacity limits. The material uses geometry normals; there is no local normal
map layer yet.

### CPU pool groundwork

`rcbt-core::BisectorPool` provides fixed-capacity leaf slots and generation-checked
handles, transactional exhaustion, split/merge, and migration/reset from the
existing tree. It is **not wired as the renderer's GPU topology engine**. The
bounded CPU benchmark includes the existing BTreeSet tree, packed alternatives,
and the new pool; its timings are not evidence of a frame-rate improvement.
Neighbor-linked GPU bisectors and conforming split/merge propagation remain open.

### Deterministic material comparison

The coast overview used for image comparison additionally sets:

```sh
THESSA_AUTOBENCH_VIEW=surface THESSA_AUTOBENCH_SITE=0 \
THESSA_AUTOBENCH_SURVEY_DISTANCE_M=50000 \
THESSA_AUTOBENCH_SURVEY_PITCH_RAD=-0.9
```

`THESSA_AUTOBENCH_OCEAN=off` disables only the analytic ocean contribution for
same-build cost comparisons. All these overrides are restricted to autobench.


### Benchmark event loop correction

Bevy 0.19.1's default `WinitSettings::game()` uses a roughly 60 Hz low-power
loop when the window lacks focus. This is independent of graphics VSync. The
final autobench path explicitly sets `WinitSettings::continuous()` for both
focus states; normal launches retain the existing behavior. Earlier mixed-focus
whole-frame results are therefore not used to claim an FPS gain. GPU pass
timings remain scoped measurements, not summed estimates of whole-frame GPU time.

## Follow-up: mesh emission, presentation acknowledgement, camera motion

The 2026-09-16 review found four real defects in the optional mesh consumer and
bootstrap handoff. They are addressed as follows:

- A 64-lane mesh workgroup now writes all 81 vertices and 128 primitives with
  strided loops. Only lane zero writes the output counts.
- Indexed compute and mesh emission share `surface_sample.wgsl`, including
  quantized heights, tile-local precision and analytic sphere derivatives.
  Mesh output uses the same CPU f64 tile frames and camera-relative hi/lo
  anchors. The duplicate absolute-position and forward-difference path is gone.
- `CbtGpuPresentation` acknowledges an actual recorded draw after required GPU
  bindings exist. CPU Image residency only enables preparation. Main-world
  backdrop visibility waits for that acknowledgement. Activation epochs and
  material identities prevent reusing acknowledgements across restarts or
  material changes; ordinary complete LOD swaps do not rearm the backdrop.
- Both consumers require the adapter's complete-cover flag and complete height
  pages. The mesh consumer can no longer bypass the presentation fence.

The mesh shader remains an **optional diagnostic consumer** with its original
simple shading; it is not enabled by the game and does not have indexed-path
material/lighting parity. Native mesh-stage drawing has not been visually
validated on this adapter. Naga validates the native shader; a GPU regression
executes its production entry body with compute IO to read back every output
of all sixteen meshlets. It checks counts, triangle coverage, finite normals,
L17 Earth-radius positions against f64 (<1 mm in this fixture), and missing-page
zero counts. This specifically catches the previous uninitialized tails.

Two causes of camera-related detail loss are also addressed:

- Streaming speed now follows the translated craft/survey focus in the body
  frame. Camera orbit and zoom still trigger view selection, but do not inflate
  the velocity LOD bias. The existing high-speed flight budget remains in place.
- A missing child material page reuses the nearest resident ancestor with an
  exact UV subrectangle, including through terrain-cache eviction. It returns
  to the exact child when available. A global-map fallback is still needed when
  neither child nor ancestor fits the bounded material cache.

`THESSA_AUTOBENCH_CAMERA_ORBIT=1` adds a repeatable pilot camera orbit. Combined
with `THESSA_AUTOBENCH_PAUSED=1`, it isolates camera/aircraft disocclusion from
flight. These fixes do not establish that every reported blur at the aircraft
silhouette has been eliminated; there is no temporal AA or motion-blur pass in
this terrain path, and image comparisons must distinguish mip filtering from
streaming fallback.

Migration boundary: these changes repair the existing disposable Bevy bridge.
The shared WGSL geometry and material-ancestor mapping contain no Bevy API.
The acknowledgement resource itself belongs to the bridge, not engine topology.
No Solari, scheduler or new render-graph architecture was introduced.

### Follow-up validation results

`cargo test -p thessa-bevy-rcbt --features mesh-shaders --lib -- --include-ignored`:
29 passed (including six GPU executions). Client tests: 61 passed. Release build
and both 2560×1600 Vulkan runs completed without GPU validation errors.

| Scenario | Frame p50 / p95 / p99, ms | GPU terrain median, ms |
| --- | --- | --- |
| camera-orbit-paused | 15.42 / 17.06 / 18.23 | 2.42 |
| pilot-moving-fixed | 15.39 / 17.63 / 19.47 | 2.79 |

Each capture retained 1,800 frames. A separate CPU trajectory benchmark was
running concurrently, so these whole-frame figures are observations, not an
isolated performance comparison. The paused orbit maintained simulation time
0, streaming speed 0 m/s and detail bias exactly 1.00 throughout. Terrain had
349–351 published patches; the moving-flight run had 382–385. Screenshots show
continuous terrain in the checked frames; residual horizon stepping and distant
material/filtering limitations remain. The reported silhouette-specific blur
has not been proven fully resolved by these captures.

## Follow-up: residency, stable materials and physical orbit captures

The next investigation found three independent sources of tile-shaped changes:
stateless split decisions around LOD thresholds, immediate eviction of inactive
pages, and material classification that depended on the requesting tile's texel
spacing. A distant local page also replaced the globe map abruptly even when
its 32 m material bands were smaller than a screen pixel.

Implemented changes:

- The backend-neutral selector retains previous splits through a 20% coarsening
  band. The GPU triangle classifier has a matching hysteresis policy with
  per-leaf identity checks; changing an ordinal cannot inherit another leaf's
  grid step.
- Camera-demand selection no longer waits for obsolete workers to drain.
  Ready sibling families can advance independently through a complete,
  non-overlapping resident cover, separate from planning-tree convergence.
- CPU height/material residency uses an excess-only LRU budget (1,024 entries
  for GPU terrain, 512 for CPU meshes), pinning presentation, current requests,
  worker results and material ancestors. Material GPU layers retain inactive
  pages until pressure requires eviction; the hardware-clamped upper bound is
  512 layers, about 42.67 MiB including mips.
- Material jobs prioritize the view and can build a shared parent before four
  missing child pages. The existing temporary worker queue allows four material
  jobs when geometry pressure is low, otherwise two.
- GPU material generation evaluates the canonical field at each texel rather
  than interpolating a tile-local macro grid. Material slope uses a fixed 256 m
  scale and spherical tangent offsets. Matching coordinates now produce the
  same bytes across adjacent levels. This costs more CPU per cold material page;
  `cargo bench -p thessa-worldgen-rocky --bench tiles` measures that cost.
- Indexed shading fades local material into the globe map over a continuous
  16–32 m physical pixel footprint. This filters the 32 m detail bands at orbital
  distances without using a flat tile LOD as a colour switch. It is not a full
  virtual-texture implementation: cold nearby pages can still show a global-map
  fallback, and coarse mip borders remain a limitation.
- The GPU cover remains active in pilot orbit; the old 80 km whole-surface
  switch is removed. The existing complete-cover presentation fence still owns
  bootstrap visibility.

Reproducible **physical** orbit capture (run from the repository root):

```sh
THESSA_AUTOBENCH=1 THESSA_AUTOBENCH_STATIC=1 \
THESSA_AUTOBENCH_VIEW=pilot THESSA_AUTOBENCH_ORBIT_ALTITUDE_M=1000000 \
THESSA_AUTOBENCH_SCREENSHOT_DIR=/tmp/thessa-orbit \
target/release/thessa-client --local
```

This sets an initial inertial spacecraft position and circular velocity from
its reference body's ephemeris and gravitational parameter, then advances the
normal authority/physics path at 1x. The camera follows the spacecraft. It is an
orbital initial-condition fixture, not a simulated launch/ascent. At 100 km,
atmospheric drag is still allowed to perturb the initial circle. Embedded mode
rejects this local-only initializer rather than overwriting server authority.
The eight native screenshots have companion `*.state.json` files with sim time,
position, velocity, altitude and pause state. These are sampled when the image
request is queued, so they can precede the captured GPU frame by one frame.

Autobench keeps up to 12,000 frames so the first turn is not lost from the ring.
The paused camera-sweep utility remains available for isolated cache regression,
but its images are not evidence of physical spacecraft travel.
