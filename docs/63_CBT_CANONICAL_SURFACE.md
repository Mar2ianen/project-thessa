# 63 — Canonical height/material surface from persistent bisectors

**Status (2026-10-24): selected by checked-in native launch configurations;
acceptance incomplete.** WORLD13 in [doc 77](77_NATIVE_SURFACE_SHOWCASE.md)
promotes the existing consumer, separates sparse overlapping height residency
from CPU patch partitions, and adds exact fine-material charts and soil-height
pages. Explicit disabled/unsupported paths retain patches; the Rust settings
constructor remains off for existing library/legacy callers. Earlier measured
checkpoints below retain their original dates and scope.
The original verified selection snapshot is `e386ef0`.
[Review 64](64_NATIVE_OPTIMIZATION_REVIEW.md) preserves the rejected intermediate
optimization proposal. The corrective follow-up removes retained terrain origins
and fixed-wave sleep. Geometry, ocean phases, mesh casters and shadow receivers
again share the current f64 camera origin. A full-rank GPU transaction wave
reports outstanding requests and commits; only completed zero/zero feedback may
sleep the same generation. Rejections/exhaustion are not convergence. One bounded
16-byte asynchronous readback fails open, without a per-frame wait; a new view
matrix, position, extent, wave envelope, height/partition/material publication or
ocean geometry sample wakes the work. Encoders must still be submitted in order;
discarded encoders cannot certify quiescence. No geometric-error guarantee or
unchanged-chunk RT optimization is implied.

WORLD13 retires absolute-height/wave-envelope area inflation from the native
classifier. It caused pool exhaustion even on an elevated flat surface. Current
classification combines actual projected area with three edge-midpoint datum
probes through the **same canonical height sampler as emission**. Low/Medium/High
area budgets imply 4/2/1 px sampled deviation targets. A candidate parent is
re-evaluated before a low-error child may merge, preventing split/merge cycles
around exactly represented peaks. Homogeneous frustum rejection precedes w-plane
handling; outward back faces rejected by primary raster do not consume its
viewport refinement budget. No universal error bound follows from three probes.
Scratch emissions include the existing admitted, filtered ocean deformation;
wave geometry contributes actual area without imposing its global amplitude
on dry land. Datum probes do not certify a continuous-wave error bound.
The unused per-page envelope GPU buffer/upload has been removed; canonical
page data, CPU extent diagnostics and existing shadow bounds remain.
The audit regression checks actual GPU quiescence, resize/rotation wakeup and
cached/reference mesh-shadow equality after camera motion. Back-face culling is
restricted to conforming primary geometry; callers may explicitly discard final
scene depth when no downstream consumer reads it. The old performance percentages
remain unaccepted; moving views conservatively re-emit/rebuild as before.
Builds on [`61`](61_CBT_SPHERE_EXPERIMENT.md) (sphere topology) and
[`62`](62_CBT_PARALLEL_TRANSACTIONS.md) (parallel transaction waves) from
baseline `1ff4b3a`. This phase removes the last "pure sphere" qualifier from
the geometry itself: active bisectors are converted into **planet-scale
vertices that sample the canonical height and material pages**, using the
existing residency/publication paths. The native window/headless renderer can
now select this topology through resolved graphics settings; raster, shared
shadows, spectral geometry/optics and the existing `RayScene` consume the same
encoded stream. This does not establish complete Bevy removal or equal-error,
temporal, cross-platform or blind visual acceptance.

**Bevy renderer parity is not an acceptance gate** (see doc 62 §Acceptance).

The shared native primary material now also integrates rough sky/cloud
reflection on both regular and conforming terrain (doc 69); it does not change
topology, canonical heights, convergence or their outstanding acceptance gates.

The 2026-10-31 [production flight replay and scratch optimization](103_CBT_FLIGHT_AND_SCRATCH.md)
verify a frozen 224-frame ground→orbit→ground path and bounded-pool recovery.
Land-only classification scratch skips inactive writes and validates pool
liveness; final publication still clears every inactive slot. Conforming ocean
scratch retains full clearing for its admission/footprint scan. Same-work High
ABBA improves the isolated update/emission interval with identical counters
and endpoint raster. Streaming, dense tessellation error and complete consumer
cost remain separate open gates. `read_topology_records` is an explicit
one-shot diagnostic; ordinary submission still performs no pool readback.

## What is implemented

### Exact surface addresses, independent of allocation slots

`rcbt-core::conforming_surface::SurfaceAddress` maps an integer cube vertex
(depth 1..=58 local bisections) to the canonical cube-face parameterization
used by the rest of the project:

- exact selection of face `0..=5` and exact numerators over
  `FACE_DENOMINATOR = 2^29`, including the closed upper edge;
- shared-edge ownership is deterministic (X, then Y, then Z), so a seam or
  corner vertex has exactly one canonical address regardless of which
  bisector face reaches it first;
- `page_id(TileKey)` is the single implementation of the existing material /
  height quadtree identity (face root `8..13`, then x/y bit pairs). The legacy
  renderer's private `tile_node_id()` now delegates to it instead of keeping a
  second copy of the encoding;
- `tile(level)` clamps the closed upper edge to `(2^level - 1)` exactly like
  the GPU directory lookup, so page selection and UV are computed from the
  same unrounded numerators.

Depth-58 addresses are never rounded to f32 before page ownership is decided;
rounding happens only after the page and its grid are known.

### Heights: borrowed, not re-streamed

`ConformingSurface` does **not** own a height allocator, cache, uploader or
streamer. `TerrainRenderer::height_surface_source()` exposes a cloneable
`HeightSurfaceSource` that holds:

- a *live* handle to the existing `metadata` and `residuals` buffers (same
  GPU memory the legacy patch renderer already uploads into);
- a sparse canonical directory rebuilt from the renderer's existing
  `node_ids` publication, plus a uniform holding the deepest resident level.

The directory is republished inside the existing `publish_partition()`, so
reordering, generation changes, eviction and refill are handled by the
*existing* terrain owner. Nothing new is uploaded, and calling
`height_surface_source()` costs nothing while unused (no buffer, no
publication). The shader walks the resident ancestor chain from the deepest
published level downwards and samples the canonical page with the shared
`page_sample()` decoder. A face with no resident page resolves to an explicit
zero height, never an out-of-bounds slot or a stale evicted page.

### Materials: borrowed residency, no geometry-specific cache

`TerrainRenderer::material_binding()` shares the existing `MaterialArray`
texture/sampler/sparse directory. Bisector refinement never changes layer
assignment; `SlotCache` still decides residency from canonical demand. The
material lookup helpers were extracted from `material_sample.wgsl` into a
shared `page_directory.wgsl` chunk (`PAGE_DIRECTORY_WGSL`) so height and
material lookups hash the same way — `MATERIAL_SAMPLE_WGSL` is now that chunk
plus the original body, and its existing consumers are source-equivalent.

### Precision: f64 camera subtraction reused

`precision_charts(radius, camera)` builds six camera-local
`TileAnchor`/`GpuTileFrame` charts, one per cube face, at a fixed local level
`PRECISION_LEVEL = 12`. Only the chart that contains a vertex is used:

- `anchor_hi_m + anchor_lo_m + cbt_local_offset(...)` is the existing stable
  bend formula from `precision.wgsl`, so the large radius-sized term is added
  once from an f64-subtracted camera-relative anchor;
- the local chart integer origin is subtracted from exact UV numerators
  *before* narrowing to f32, which is what makes depth-58 vertices representable;
- duplicate cube vertices always select the same chart, so a shared seam
  vertex is evaluated identically from both sides.

### Emission kernel

`render-wgpu::conforming_surface::ConformingSurface` runs one compute dispatch
per pool over all slots: virtual roots are zeroed, live leaves decode their
integer cube triangle, look up their canonical height, decode the shared
normal, and write three 48-byte vertex records (position + height, normal +
face, exact UV numerators + resident source slot + level). No patch grid, no
skirts, no CPU mesh build. Layout: 18,875,136 GPU buffer bytes at
131072-capacity (vertex stream + charts + directory), excluding the borrowed
page buffers.

## Verified evidence (Radeon 880M, Vulkan / RADV, Mesa 26.2.4-arch1.1)

### Deep precision and residency

`conforming_surface_borrows_canonical_pages_and_preserves_deep_precision`
runs at `radius = 6,371,000 m` with a single vertex driven to **heap depth
58** (54 local bisections), then compares every emitted vertex against an
independent CPU `TileAnchor::project_body_m()` reference:

| Boundary | Observed max | Declared bound |
|---|---:|---:|
| Whole-planet coordinate error | 0.5937 m | 8 m |
| Vertices within 10 km of camera | 0.001048 m | 0.02 m |
| Shared `HeightPage` decoder disagreement | < 0.005 m | page `max_residual_error_m` |

Additional assertions in the same run:

- every shared seam/corner/deep vertex is **bit identical** across the
  bisectors that reference it (1292 near-camera vertices were compared);
- distinct local deep vertices do **not** collapse to one f32 position;
- face UV, resident source slot and resident level match the independent CPU
  canonical address;
- reorder, generation change, eviction and refill of source pages are all
  republished by the legacy owner and reflected in the emitted geometry, with
  `regenerated == 0` on an unchanged second publication;
- a missing face resolves to the zero-height fallback.

These are **vertex arithmetic** bounds against a reference projection of the
same quantized page data. They do **not** bound chord-to-sphere tessellation
error, analytic-vs-baked terrain disagreement, silhouette error, or astronomical
rotation/frame errors.

### Target-size emission and shared material raster

`canonical_surface_target_cost_and_shared_material_indirect_raster` measures a
98,304-triangle pool in a 131,072-slot capacity with six root height pages
(33² grid):

- emission compute: **p50 1.0337 ms / p95 1.1389 ms**, 40 samples after 10
  warmup dispatches, GPU timestamps only (CPU readbacks, page publication and
  initialization excluded);
- the same run reports absolute coordinate error 1.5638 m with the camera at
  3 radii, within the declared 8 m whole-planet bound;
- an actual **indirect draw** samples the borrowed material residency through
  the shared `material_at_level()` path: publishing an all-red page generation
  makes the red channel dominant, publishing an all-green generation makes
  green dominant, so a geometry-consumer update really reaches the raster;
- a third `update_materials()` call with unchanged bytes reports 0 uploaded
  bytes, and the topology and surface-vertex buffers are byte-identical
  before/after the material-only updates — material residency never rebuilds
  geometry.

Raw samples: `crates/render-wgpu/target/test-tmp/cbt-sphere/surface-cost-ms.csv`;
capture: `.../surface-material.png`. Both are regenerable, ignored build
output.

The measured dispatch emits **every** active slot, including zeroed virtual
roots, with no dirty culling and no view-dependent early-out. It is a topology
size cost, not a production frame cost, and not a game-FPS, Bevy or
`large_cbt` comparison.

## Historical height-bound policy (reference API; superseded for native production)

The APIs/tests in this dated subsection remain available to the isolated sphere
reference experiment. WORLD13's native source/chord probes above replace this
inflation policy in the production consumer; the original monotonicity fixture
does not certify close-view efficiency or convergence.

The classifier no longer measures unit-sphere chords alone.
`rcbt-wgpu::SphereGpu::set_height_bound(height_chord)` publishes a conservative
surface-height bound in unit-sphere chord units (`height_m / body_radius_m`) at
the `Params.surface` uniform (offset 128; `set_view` writes only the first 128
bytes, so the bound survives each per-frame policy update). `pixel_area()` then
inflates a triangle's measured screen area by `(1 + 2h/L)^2`, where `L` is the
triangle's longest chord edge: a raised corner moves on screen by at most that
bound relative to the triangle's own extent. The same inflated value feeds both
the split and merge thresholds, so a larger bound can only refine earlier and
decimate later. Zero (the default) leaves the classifier on plain chords, and it
is a bounded LOD policy input, not a change to emitted geometry.

`TerrainRenderer::height_bound_chord()` derives that bound from canonical data
rather than a manual guess. `publish_partition`/`update_pages` track a per-slot
resident height extent (`min_height_m - max_residual_error_m`,
`max_height_m + max_residual_error_m`) alongside the existing shadow and ocean
bounds; the accessor folds the widest active extent into `max |h| / radius`,
clamped to one chord. Extent is cleared when a slot is evicted, and it is
independent of transient render refinement, so the bound tracks canonical page
residency and nothing else.

Verified on the same Radeon 880M (`height_bound_conservatively_refines_without_changing_the_oracle`):

- **Deterministic per-slot monotonicity.** Two identically seeded pools (uniform
  depth 4, 198 leaves) are classified once, one with bound `0` and one with
  `0.05` chord at camera distance 2.0 radii. `classify` runs before `update`
  mutates the pool, so both `areas` buffers measure the identical triangles:
  all 56 back-face- and frustum-visible triangles grew, **none** shrank
  (`b >= a` for every one of 4096 slots, 0 underflows).
- **Autonomous policy stays oracle-exact under a bound.** A 240-frame sweep with
  bound `0.05` committed 240 transactions with GPU records matching the CPU
  replay at every frame and a valid OCBT allocator mirror afterwards.
- Negative, NaN and infinite bounds are rejected at the API.
- **Bound derives from canonical residency.** `height_bound_chord_follows_resident_page_extent_and_clears_on_eviction`
  publishes a 100 m page and checks the accessor returns exactly
  `max|height ± residual_error| / radius`; a manual update to a −4 km page widens
  it, republishing the same page holds it, and evicting the slot returns it to
  zero. The bound tracks page residency, not render refinement.

This closes the earlier gap where a mountain or a near-camera surface was not
charged for the screen area its height adds. It is a **conservative inflation of
the LOD measure**, not a bound on the actual rendered height error: it uses the
widest page extent as a single body-wide value, so it cannot express per-tile
relief, and nothing yet feeds it into a production mode.

## Diagnostic shadow caster (added after phase three)

### Current production receiver correction (2026-10-19)

The sky/audio combined-flight replay at `c0db7bc` exposed regular triangular
darkening in the opt-in conforming consumer. The initial description as holes
was incorrect: with shadows disabled the same geometry is continuous. An
unbiased canonical depth stream was being sampled at neighbouring PCF texels
against the receiver centre's depth. A trial using the mesh/patch caster bias
alone still failed the new regression and was not retained.

The shared directional receiver now projects its tangent plane into the
orthogonal light-view axes. Each integer PCF tap compares against that plane's
depth at the tap, minus the plane-depth variation over the comparison filter's
one-texel support. The bound derives from the actual atlas texel size and plane
slope, not a new hand-tuned meter offset. The existing requested normal bias,
filter tiers, atlas resolution, cascade bounds and disabled state are unchanged.
Conforming and regular terrain supply the oriented geometric triangle normal
from position derivatives, not a smoothed shading normal. Spectral BRDF normals
remain optical normals; they do not redefine the geometric receiving plane.
The canonical caster stays unbiased, preserving the independent metric-depth
oracle below. No extra textures, passes, publications or geometry are introduced.

`conforming_planetary_receiver_matches_unoccluded_reference` uses a 3,200 km
radius convex datum, a near-ground oblique view and a 512px two-cascade shadow
atlas. An unoccluded reference and the production shadowed stream are compared
over all lit reference pixels, with a three-sRGB8-byte absolute envelope. The
new regression initially failed (23,870/23,870 pixels in its original foreground
ROI); after the correction its expanded full-image domain has 50,362 tested
pixels, zero failures and zero maximum difference on Radeon 880M/Vulkan. An
independent projection of a separate 60m opaque square's shadow centre retains
real occlusion (green channel 85 unoccluded, 10 occluded). This is deterministic
geometric/sampling evidence, not a blind image preference judgment.

The 1280x720 combined native replay no longer has the dense near-field pattern.
Its far-horizon dark silhouettes, live-streaming/LOD convergence, broader
nonplanar/grazing-light error and independent visual acceptance remain outside
this local regression. Finite-resolution filtering still bounds shadow detail;
the tangent-plane correction is not a claim of exact curved-surface transport.
See `docs/artifacts/native-shadow-receiver-2026-10-19.json` for commands,
local gate status and capture/measurement locations.

The final stable-source follow-up passed all fourteen local commands: fmt,
workspace all-feature and native feature-off clippy, workspace unit/integration,
native feature-off unit, Bevy/render/native ignored GPU suites, architecture,
agent-QA, whitespace, release build and a 30-frame headless flight replay.
The final replay's GPU p50 was 3.326 ms surface/shadows, 39.873 ms scene/sky,
and 0.129 ms display. These live-streaming absolute intervals have no matched
whole-frame baseline and are not game FPS or an accepted performance budget.

The shared shadow atlas can now consume the canonical surface. `ShadowAtlas`
owns a `conforming_pipeline`/`conforming_layout` beside the legacy terrain and
mesh caster pipelines, and `encode_conforming_casters(backend, encoder, surface)`
draws the emitted per-bisector vertex stream into every active layer. The
diagnostic vertex shader reads three 48-byte records per allocation slot
(position+height, normal+face, UV+source+level), culls only the all-zero
sentinel the emitter writes for reserved virtual roots/free slots, and applies
`shadow_clip * vec4(position, 1)` directly. Both position and matrix use
camera-relative coordinates; there is no second camera/light-origin
subtraction. This diagnostic surface assumes identity body rotation.

Emission still writes **every** live leaf with no camera culling, so a caster
behind the camera can shadow a visible point. Like `encode_shadows_reference`,
this is an isolated diagnostic consumer: `encode_shadows()` continues to use the
legacy patch caster only, and nothing selects the conforming path at runtime.

Verified on the same Radeon 880M (`conforming_surface_casts_shared_shadow_depth`):

- Six constant 20 m face pages, body radius 1000 m, local depth 8, 3072 live
  triangles, 1 cascade / 512 px, 2 km cover and three distinct sun directions.
- **Independent canonical sphere oracle:** 39,600 rays in the central half-radius
  disks must hit between radii `R` and `sqrt(R²-D²/3)`, where `R=1020 m` and
  `D=178.240861 m` is the longest CPU-oracle triangle edge. For barycentric
  weights, `|Σwᵢvᵢ|²=R²-Σᵢ<ⱼwᵢwⱼDᵢⱼ²`, and `Σᵢ<ⱼwᵢwⱼ≤1/3`, giving the
  conservative inner radius. Central rays avoid silhouette sensitivity.
  Maximum measured depth error is **4.402674 m**, inside the geometry-derived
  **6.014414 m** maximum ray-depth envelope, plus **0.002 m** arithmetic tolerance.
  The diagnostic caster is unbiased so raster slope bias does not masquerade
  as geometric error; legacy mesh/patch caster bias is unchanged.
- Every layer must have >500 valid depth texels; 156,355 overlap legacy depth.
  Legacy comparison is **diagnostic only**, not an acceptance oracle:
  the maximum normalized depth delta is 0.045650303, including legacy bias.
- 9,216 live records and 12,288 allocation slots submitted over three draws.

This supersedes inaccurate explanations in the first shadow report. Real
depth-4 roots 16..27 emit geometry even at local depth zero; only roots 28..31
are virtual. A level-11 tile spans about `2R/2048=0.977 m` in cube coordinates
at this radius, not 15 m. The previously printed `(234,234,277,1301)` was a
concatenated-layer extent, not a per-layer assertion; flat layer-major zipping
does pair corresponding texels. The former normalized `0.02` tolerance had no
independently established metric tessellation envelope and is withdrawn.

## Diagnostic ocean geometry consumer

`ConformingSurface::set_ocean` borrows `OceanField` and the existing material
texture/directory; it adds no spectrum, FFT, streamer, cache or datum backup.
The existing resolved `[ocean]` policy controls this optional consumer:

- Disabled or `geometry_patches=0`/`geometry_distance_m=0`: no consumer buffers,
  pipelines or extra dispatches. Optical-only is the cheaper representation.
- Enabled geometry: two extra compute passes after canonical emission. The
  first reduces near-field triangle count and **twice the longest edge**; the
  second uses that common footprint with shared `ocean_spectral` and canonical
  `ocean_material_weight(height, roughness)`, then fades to datum over the outer
  half of the configured distance. Each legacy 33×33 patch budget unit admits
  2048 bisector triangles. Whole-partition overflow leaves geometry at datum.
- A footprint at least the largest spectral period stays optical-only instead
  of moving coarse vertices by numerical DC residue. The conservative common
  filter includes dry near triangles; local/per-edge filtering is not claimed.

The common footprint is deliberate: incident triangles at different depths
must not choose different displacements of a shared vertex. Canonical face/UV
ownership selects identical radial/material samples for all copies. Page level
is never used as geometric resolution. Height remains in `position_height.w`;
`normal_face.w` is the face, and `address.w` is the height-page level. Missing
height pages do not acquire ocean geometry. Datum height, normal and address
are preserved for classification/optical shading (no doubled wave gradients).
Every encode re-emits datum before deformation, so repeated/paused frames do
not accumulate waves. Caller must publish the same field time, camera and
radius before emission, then run shadow/raster consumers afterwards. There is
an encoded-publication contract below; it does not freeze earlier queue contents.

Hardware test `conforming_ocean_borrows_field_without_cracks_or_accumulation`:

- Independent f64 direct Fourier sums, box mips and periodic bilinear/trilinear
  sampling at 32 displaced vertices: maximum vector error **0.000094677 m**
  under a **0.004 m** absolute arithmetic/filter tolerance. This compares the
  same discretized spectrum, not continuous-wave tessellation fidelity.
- 7,678 duplicate records bit-identical, 5,960 records moved; mixed-depth split
  changes topology and matches the CPU topology oracle, with shared identity
  preserved. Checks time change/rewind, pause/repeat, calm sea, dry material,
  elevated land, height eviction/refill, far camera, coarse real roots,
  budget overflow, disabled/optical-only, and rejected invalid configuration.
- 32 bytes of extra GPU buffers (policy and extent), no CPU readback in encode.
  Timestamp fixture: 98,304 triangles in 131,072 slots, paired same-source
  datum/emission+extent+deformation measurements, 10 warmups + 40 samples.
  Datum p50/p95 **1.081/1.309 ms**, ocean p50/p95 **3.541/4.303 ms** on this
  Radeon 880M/RADV run. Exact timings and checks are recorded in the artifact;
  this is not a
  complete-frame, FPS, equal-error renderer or performance-win claim.

The geometry kernel validates with Naga's empty native-capability set and runs
on RADV/Vulkan here. Metal/WebGPU execution, a production optical/raster
consumer for this surface, Earth-scale wave-phase accuracy, and temporal
continuity across common-footprint changes remain unverified. Existing optical
field publication is reused; no production ocean selection is replaced.

## Encoded publication and existing RayScene integration

`SurfacePublication` contains a monotonically increasing emission revision, the
f64 camera origin used by those vertices and an ocean-configuration revision.
`update_camera` only stages a camera; `encode` publishes it **after** recording
datum emission and optional wave deformation. Reconfiguring ocean invalidates
previous RT consumers; a replacement must wait for emission of that configuration.
Failed configuration leaves the old consumer/publication intact.

`ConformingSurface::ray_source(materials)` borrows the same vertex buffer,
material texture/sampler/directory and optional spectral-field binding. It is
unavailable on devices without enabled ray-query support. Source clones retain
resources and shared publication identity even after producers are dropped;
they are **not immutable snapshots**, page-content pins, or queue-completion
tokens. Callers must order source writes, topology update/publication, emission,
AS build and raster/shadow consumers on the same ordered stream. Encoding and
then discarding an encoder does not publish GPU results. Concurrent mutation
or out-of-order submission of shared publications is outside this contract.

The existing `TerrainRays`/`RayScene` path now accepts bounded allocation chunks
of up to 2048 non-indexed triangles at a 48-byte vertex stride. Canonical page
identity is independent of these chunks. Virtual/free slots emit zero-area
triangles; no live-leaf readback, mesh expansion, duplicate vertices, cache or
second ray scene is introduced. Hit shading decodes the actual surface ABI and
canonical directions; it reuses the shared material/ocean kernels. Reflections
therefore trace the **same emitted/deformed triangles** used by other consumers.

Each new emission conservatively dirties all chunks. A retained publication
reuses its BLASes/TLAS; optical-only field phase and dry material shading can
change without emission or AS work. Whole-pool instance/memory admission uses
the existing resolved RT budget and omits the complete contribution on overflow.
Before first emission or after incompatible configuration, stale geometry is
not admitted. Retained AS adoption also checks field/material/source identity
and does not adopt an under-allocated conforming partition into a larger budget.
Driver-owned AS memory remains outside the existing conservative planning charge.

Hardware oracle `conforming_publications_feed_existing_ray_scene_without_stale_blas`
uses an actual mesh mirror, canonical red/green pages, a 20 m ray interval,
height changes moving the surface out of that interval and exact restoration,
camera rebasing, a committed GPU split, pause/reuse, whole-pool budget failure,
RT disabled, ocean time/rewind, configuration invalidation and retained resource
lifetime. It also checks that optical-only phase changes affect reflection
bytes with **zero** BLAS/TLAS builds. Target-size AS timings and complete gate
results are recorded separately in the RT artifact below.

RT evidence: [`artifacts/conforming-ray-publication-2026-10-18.json`](artifacts/conforming-ray-publication-2026-10-18.json).
The earlier canonical-surface artifact remains the `2851a9f` ocean/shadow-stage
snapshot; its old gap list is not the current implementation status.

The 98,304-live-triangle / 131,072-allocation-slot fixture rebuilds 64 chunk
BLASes: AS-only GPU p50/p95 **5.665 / 5.901 ms** on Radeon 880M / Vulkan / RADV
Mesa 26.2.4-arch1.1 (5 warmups, 20 samples). A retained publication then encodes
zero AS passes. This is a full-build reference/stress result, not a performance
win over cages, procedural water, refit, or the legacy renderer.

Local final gates passed: fmt, workspace/all-target/all-feature clippy and native
no-default-feature clippy with warnings denied; **1,881 lib/bin +91 integration**
tests; **24 Bevy GPU +20 native ignored +26 render-wgpu ignored** tests (the
last total includes nine CBT tests, the new RT oracle and three backend library
tests); architecture guard, five agent-QA tests, JSON validation and diff check.
Remote CI, Metal/WebGPU execution and blind subjective acceptance were not run.

This is the triangle reference backend, not an optimized dynamic-ocean backend.
Every emission rebuilds all chunks, including holes, and allocation chunks are
not guaranteed spatially coherent after topology churn. No low-cost refit,
per-chunk dirty tracking, cage reuse or continuous-wave intersection is claimed.

## AMD tetrahedral cages: representation decision before RT integration

Reviewed the [September 17 AMD demo](https://gpuopen.com/learn/how-tetrahedral-cages-significantly-reduce-bvh-memory-usage/),
[technical blog](https://gpuopen.com/learn/ray-tracing-massive-amounts-animated-geometry/)
and [authors' paper](https://gpuopen.com/download/TetrahedralMeshes_AuthorsVersion.pdf)
(Gruen et al., DOI `10.1145/3820014`, §§3–8 and Tables 1–3).

- The reported ~80 GB vs ~1.7 GB BVH requirement and >300 ms vs ~3.3 ms updates
  belong to AMD's **specific** independently animated plant demo (~500M triangles
  after LOD, primary+shadow rays, 1080p, RX 9070 XT). They are neither a general
  compression ratio nor expected Thessa/Radeon 880M timings. Do not conflate this
  September demo with the paper's separate combined-scene timing table.
- Clip rest geometry into tetrahedra, share static mini-meshes/mini-BLASes, animate
  cage vertices and update instance affine transforms/TLAS instead of dense BLASes.
  The fast variant maps to ordinary triangle BLASes and affine instances; it is
  **not intrinsically DirectX-only**. No DX12 API/dependency is introduced here.
- A coarse cage approximates deformation, not arbitrary topology changes or
  sub-cage waves. Clipping increased triangle count 1.3–2.3× in paper fixtures.
  Adaptive CBT split/merge is not automatically covered by static clipped meshes.
- Crucially, the fast variant does **not guarantee watertightness**: different
  instance-local rays can disagree at shared edges. Small geometric overlaps
  mitigate this in their fixtures but are not a proof suitable for a canonical
  planetary/water surface. Their strict 4D-barycentric/software-traversal variant
  was 19–80× slower in Table 2; it is not the free fast/watertight alternative.

Decision: reserve cages for a later, measured connectivity-preserving vegetation
or repeated-asset animation backend. Do not insert cages into the CBT surface or
replace the canonical ocean spectrum with cage coefficients to finish migration.
Retain the shared-triangle backend as a correctness/fallback reference and keep
the publication contract about emitted state rather than a universal requirement
to rebuild water BLASes for every optical-field update.

Procedural water is a distinct candidate: time-independent conservative bounds
could contain the coefficient-bounded wave envelope, with intersections evaluated
from the **existing** field. Current wgpu/Naga 30 expose AABB geometry and candidate/
generated ray intersections; the current Thessa `rt_trace` is still an opaque
triangle-only committed-hit traversal. API presence is not a tested procedural
backend. A replacement must preserve nearest-hit ordering with opaque land/meshes,
handle multiple roots, tangent/grazing rays, shore masks, shared edges, filtering,
distance fade and missing pages, and bound error against the raster representation.
A single Newton solve or phase-specific AABB is not a completeness guarantee.
Compare full build+traversal cost, memory, invalidations and hit error at equal
quality, including topology/footprint changes and static/paused controls, before
choosing it. The old 33×33 ocean cost (0.120 ms deformation vs 1.412 ms AS at 16
slots) motivates that experiment but does not prove its outcome for adaptive CBT.

## Native production-frame selection

```toml
[terrain_topology]
enabled = true # false: regular patches, no topology resources/work
quality = "medium" # low | medium | high; restart required
```

The shared requested/resolved pipeline publishes explicit budgets:

| Quality | Allocation slots | Transaction waves/frame | Split area (px²) | Merge area (px²) |
|---|---:|---:|---:|---:|
| Low | 32,768 | 1 | 1,024 | 128 |
| Medium | 65,536 | 2 | 256 | 32 |
| High | 131,072 | 4 | 64 | 8 |

Each starts with a one-time local-depth-8 conforming seed and allows heap depth
58. Quality changes representation and bounded convergence work, not canonical
height/material demand or simulation. Unsupported storage/compute limits resolve
to regular patches with a note; the native allocation boundary also checks
negotiated device limits. Registering a plugin or enabling RT does not select
this mode. With `enabled=false`, topology storage bytes and dispatch count are
zero. The legacy Bevy adapter reports this native integration as unsupported.

In conforming mode the legacy vertex/triangle buffers are only tiny layout
placeholders; grid generation/classification/compaction and legacy ocean backup
allocation/dispatches do not run. The existing canonical page/material owner
continues uploads and directory publication. No second residency or geometry
cache is introduced. Ordinary material, sky, HDR/display and mesh integration
remain shared with the native renderer, not a custom diagnostic raster shader.

`update_view_for_extent()` takes the actual render-target extent and stages the
f64 camera origin. The old extent-free `update_view()` rejects conforming mode
rather than invent a resolution. Each bounded wave emits scratch using the
shared canonical sampler and optional admitted spectral deformation, classifies
camera-relative area/source-chord deviation with candidate-parent validation,
then runs the existing parallel plan/reserve/commit/publish pipeline. Scratch
emissions are not consumer epochs; the final emission publishes the geometry
used by all consumers. No absolute planetary f32 projection, CPU grid generation
or per-frame geometry readback is introduced.

The external-area policy visits disjoint strided active-rank subsets per request,
preferring the largest actionable split, else a mergeable diamond. It reuses
the same bounded planner, conflict claims, deterministic ordinal priority and
allocator. Unlike the original cursor-only window, it does not spend all request
slots on distant non-actionable roots while close-view refinement is waiting.
An 80-wave far-to-close Earth-scale replay compares every committed transaction
and neighbor state against the independent CPU topology reference, including
actual refinement and decimation. This is not an immediate-convergence guarantee.

`encode_surface()` then emits the final datum and optional shared spectral
deformation and advances one consumer epoch. Both native frame loops call it
before shared shadows and mesh RT, then rasterize that same stream. The existing
RT synchronization entry point accepts matching conforming resource identity
without running legacy patch synchronization. The shadow cache keys on the
emitted revision as well as light/mesh revisions; off-camera leaves remain
available to casters. Retained epochs permit exact cached shadow reuse, whereas
each new topology emission conservatively invalidates all terrain BLAS chunks
and shadow depth. Dirty-chunk/unchanged-frame optimization is not claimed.

The primary raster uses the shared `CBT_CONFORMING` shader specialization
(default false for existing indexed/mesh/Bevy paths) and the actual 48-byte ABI,
without legacy skirts. WORLD13 primary and shadow draws consume the existing
GPU active indices and indirect arguments, excluding free allocation holes.
`ConformingSurface::new` now borrows the complete `SphereGpu` source so these
indices/arguments share its lifetime; no second compactor or draw cache exists.
The seed publishes valid initial indirect arguments before any topology update.
Diagnostic draw counts include the four reserved virtual roots. With
`ShadowDrawStats.indirect=true`, submitted slots are a capacity upper bound,
not an exact CPU-read count. RT still consumes bounded allocation chunks with
zero-area holes; active-only AS compaction is not implemented. `topology_storage_bytes`, publication revision and
fallback reason are available without per-frame GPU readback. The first native
GPU timing interval is now `render.surface_and_shadows`, explicitly including
classification/transactions/emission/deformation before shadow encoding.

Hardware tests cover Earth-scale primary raster, canonical material-only changes,
retained epochs, rebasing, shared shadow depth, live spectral optics and disabled
fallback. A separate classifier oracle compares 3,072 emitted triangles against
independent f64 projection; the fixture error bound is `8*f32::EPSILON*257²`
px², not a universal geometric screen-error guarantee. Area policy remains a
refinement heuristic until the equal-error/temporal acceptance below is complete.

Production-selection evidence is recorded separately in
[`artifacts/native-conforming-topology-2026-10-18.json`](artifacts/native-conforming-topology-2026-10-18.json).
Earlier ocean/shadow and encoded-RT artifacts retain their dated scope and gap
lists; this document and the status index describe the current implementation.
On the Radeon 880M, four 120-frame native captures (30 warmups, 1280×720,
manual HDR, no HUD) completed, including an X-15 preview whose existing mesh
RayScene contains 64 conforming terrain chunks / 66 total TLAS instances.
Their serial wall p50/p95 were 3.513/3.870 ms (patches), 3.788/4.383 ms (Low),
21.437/24.400 ms (High plus spectral sea, no mesh RT consumer), and
30.110/32.884 ms (High plus spectral sea and X-15 RT). These differ in quality,
content and convergence: **not equal-error comparisons or a performance win**.
The expensive High reference path remains an optimization/acceptance gap; its
surface-and-shadow interval alone measured 17.189 ms p50 in the sea fixture.
The complete local gate run passed fmt, workspace all-target/all-feature and
native no-default-feature clippy, 1,883 unit/bin tests, 91 integration tests,
24 Bevy + 20 native + 28 render-wgpu hardware tests, architecture and five
agent-QA tests. No remote CI or independent subjective review is claimed.

## Known gaps (explicitly not claimed)

1. **Sampled error is not a universal geometric bound.** Three midpoint probes
   can miss off-probe features and are not a proof about all silhouettes, source
   LOD seams or continuous waves. Bounded request waves do not guarantee immediate
   convergence at every new close camera; exhaustion is not convergence.
2. **Dynamic geometry is conservatively rebuilt.** All chunks and shadow depth
   are invalidated by every final emission; optimized ocean intersections,
   spatially coherent RT chunks and unchanged-geometry reuse remain open.
3. **Primary terrain/ocean scene-object reflections remain a wider renderer gap.**
   Mesh reflections trace this surface, and its primary raster uses shared sky
   optics, but primary terrain still does not initiate scene-object RT rays.
4. **Publication is encoded state, not an immutable frame snapshot.** Shared
   epochs/resource retention and RT configuration invalidation now exist;
   source writes, emission and consumers still require the documented ordered
   submission. Concurrent/out-of-order encoders are outside its contract.
5. **No equal-error comparison** against the legacy patch renderer, no blind
   visual review, no cross-platform acceptance, no remote CI claim.

## Reproduction

```sh
export TMPDIR=/home/chechulin/Projects/project-thessa/target/tmp
export CARGO_TARGET_DIR=/home/chechulin/Projects/project-thessa-fastrt/target
cargo test -p thessa-rcbt-core conforming_surface --lib
cargo test -p thessa-render-wgpu --lib surface_shader
cargo test --release -p thessa-render-wgpu --test cbt_sphere -- --ignored --nocapture --test-threads=1
cargo test --release -p thessa-render-wgpu --test terrain_topology -- --ignored --nocapture --test-threads=1
```

The `cbt_sphere` hardware suite is nine tests, including the diagnostic shadow
and ocean consumers. Exact numbers and check logs are
recorded in
[`artifacts/cbt-canonical-surface-2026-10-18.json`](artifacts/cbt-canonical-surface-2026-10-18.json).

Final local ocean/shadow-stage gates: fmt; workspace/all-target/all-feature
clippy and native no-default-feature clippy (`-D warnings`); workspace
lib/bin **1881 passed** and integration **91 passed**; Bevy GPU **24 passed**,
native GPU **20 passed**, and render-wgpu ignored suite **25 passed**, including
the **9** CBT tests (not 25+9); architecture gate and five agent-QA unittests.
Direct exit statuses/argument vectors and logs are retained under
`target/test-tmp/cbt-ocean-checks/`; the artifact records their hashes and
commands. These are local checks, not remote CI or platform/visual acceptance.

## Next

Complete geometric-error/per-tile policy and temporal/equal-error acceptance,
optimize measured dynamic geometry costs, then close the wider renderer/client
migration gaps tracked in doc 39. Do not delete the regular-grid fallback or
claim complete-client Bevy replacement from this topology selection alone.
