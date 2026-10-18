# 63 — Canonical height/material surface from persistent bisectors

**Status (2026-10-18): isolated experimental consumer, verified on hardware.**
Builds on [`61`](61_CBT_SPHERE_EXPERIMENT.md) (sphere topology) and
[`62`](62_CBT_PARALLEL_TRANSACTIONS.md) (parallel transaction waves) from
baseline `1ff4b3a`. This phase removes the last "pure sphere" qualifier from
the geometry itself: active bisectors are converted into **planet-scale
vertices that sample the canonical height and material pages**, using the
existing residency/publication paths. It is still not a production terrain
mode; shadows, ocean, RT and a resolved graphics setting remain open.

**Bevy renderer parity is not an acceptance gate** (see doc 62 §Acceptance).

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

## Known gaps (explicitly not claimed)

1. **Classification ignores height.** `pixel_area()` still measures unit-sphere
   chords; a tall mountain or a near-camera surface is not charged for the
   screen area its height adds. A conservative height/error term is required
   before the LOD decision can be trusted near the ground.
2. **No shadow caster.** The shared shadow atlas still consumes the legacy
   patch-grid vertex stream; the CBT surface is not published to it.
3. **No ocean geometry consumer and no RT/BLAS publication** for bisectors.
4. **No resolved production mode.** Nothing in `graphics.toml` selects this
   path; `TerrainRenderer::encode_scene()` still draws legacy patches. No
   disabled/cheaper fallback level exists, because there is no switch.
5. **No shared revision/lifetime contract yet** between the surface buffer and
   consumers (shadow depth revision, RT build, ocean near-field): callers must
   encode emission after topology publication and before any consumer reads.
6. **No equal-error comparison** against the legacy patch renderer, no blind
   visual review, no cross-platform acceptance, no remote CI claim.

## Reproduction

```sh
export TMPDIR=/home/chechulin/Projects/project-thessa/target/tmp
export CARGO_TARGET_DIR=/home/chechulin/Projects/project-thessa-fastrt/target
cargo test -p thessa-rcbt-core conforming_surface --lib
cargo test -p thessa-render-wgpu --lib surface_shader
cargo test --release -p thessa-render-wgpu --test cbt_sphere -- --ignored --nocapture --test-threads=1
```

The hardware suite is six tests: four topology controls from docs 61/62 plus
the two surface tests described here. Exact numbers and check logs are recorded
in [`artifacts/cbt-canonical-surface-2026-10-18.json`](artifacts/cbt-canonical-surface-2026-10-18.json).

## Next

Per doc 62, the remaining boundary to a real mode is: conservative
height-aware classification error, then shadow/ocean/RT publication with a
geometry revision contract, then a resolved graphics setting with explicit
disabled/cheaper fallbacks and equal-error tests before any legacy grid path
is deleted.
