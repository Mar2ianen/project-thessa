# 115 — Terrain diagnostics and height-residency audit

Status: native diagnostic views implemented; ancestry candidate rejected;
full visual/cost parity remains open. 2026-11-02. The new user audit references
`92d2ac4`; its missing-multiple-scattering observation is superseded by doc 114.
Its water and horizon hypotheses were tested separately rather than assumed to
explain the whole image. The original audit and hashes are retained with
[the artifact](artifacts/terrain-diagnostics-ancestry-2026-11-02.json).

## Explicit native diagnostic policy

`[debug] terrain_view` is a typed requested/resolved setting, exposed in graphics
metadata. It defaults to `none`. At this milestone changing it required
renderer recreation; [doc 120](120_FROZEN_NATIVE_DIAGNOSTICS.md) subsequently
adds runtime switching inside an explicitly configured diagnostic renderer.
Native specializes the existing shared primary terrain shader. No new image,
render pass, texture/storage binding, tracer or material representation is
allocated. Other renderers retain the shared shader's default `none`; the
native-only metadata label must not imply Bevy diagnostic integration.

The existing regular/conforming vertex output carries geometry kind and the
provoking vertex's height-page level. None specializes all diagnostic branches
away. This preserves geometry generation, streaming, caster and ray membership.
Separate processes still have asynchronous residency differences: these views
are **not a frozen whole-scene A/B controller** or a performance benchmark.

| Value | Raw linear terrain RGB | Interpretation |
| --- | --- | --- |
| `none` | Existing beauty output | Production default |
| `water_weight` | Existing water-classifier weight, repeated RGB | Actual roughness/height predicate before normal perturbation, not true water presence |
| `height` | `0.5 + height_m / 512` | Decode as `(R - 0.5) * 512` metres; the shader's height tag, not an independent position oracle. Regular skirts are magenta because their height tag is a sentinel |
| `distance` | Euclidean camera-to-fragment distance / 1024 | Decode as `R * 1024` metres; this is not nonlinear hardware depth |
| `normal` | Perturbed render-space shading normal × 0.5 + 0.5 | Normal before lighting |
| `albedo` | Sampled linear RGB | Includes current detail/globe coverage, no lighting |
| `roughness` | Source roughness, repeated RGB | Before material specular filtering |
| `land_only` | Beauty, with primary terrain water optics suppressed | Keeps geometry, pools as meshes, sky, shadows and exposure; not a material-contract fix |
| `geometry` | Regular surface blue, regular skirt red, conforming surface green, missing-height conforming fallback orange | Geometry **kind**, not a unique primitive/allocation ID |
| `height_page_level` | Provoking vertex source level / 24, repeated RGB | Round `R * 24`; not CBT bisector depth or a guarantee that all three vertices use one page |

Data views return before light, haze and exposure. Their PNGs still pass through
the selected presentation operator and contain ordinary sky/mesh shading.
Read the RGBA16F capture for numeric values; do not decode a tone-mapped PNG as
a mask. MSAA boundary pixels mix terrain and other samples. These are primary
terrain diagnostics, not complete per-object G-buffers. `land_only` retains
normal beauty exposure/haze and does not disable geometric water displacement.

## What the actual FHD masks established

The full-density camera/12° sun/EV11 profile retains the previous diagnostic
100 m/1024/one-cascade shadow tier, four-sample MSAA, RT Off and all plants.
Commands, raw HDR/PNG, frozen source patches and binary hashes are recorded.

- Eight inspected terrain-interior points in foreground and horizon have
  **zero** raw water weight. Their height tags are about 4.75–8.25 m, outside
  the classifier's near-datum band. The current soil stripes therefore cannot
  be attributed to that predicate at these sampled points. Separate puddle/
  mesh optics and other scene regions remain outside this observation.
- The elevated band remains green in the unlit geometry-kind view. It is
  conforming terrain, not the regular-path skirt, and lighting alone cannot
  account for its visible silhouette.
- The band samples coarse **L7 height data**, while nearby terrain uses L17.
  Inspected band distances are roughly 100–132 m; foreground points are around
  6–8 m. These samples come from separate diagnostic runs and are not asserted
  as an identical full-raster correspondence.
- The contract defect itself is independently reproduced on both regular and
  conforming GPU fixtures: at height zero and roughness 115/255, the diagnostic
  reports approximately 45% water weight despite no water-presence input.
  Dry 204/255 and height 2 m controls reject it. This proves the existing
  predicate mixes unrelated material properties; it does **not** prove that
  fixing it will cure the current forest frame.

The five-second-prestream albedo/roughness/normal views and land-only preview
retain the broad foreground patterns. Four inspected foreground source roughness
values span roughly 0.534–0.786; the pattern is visible in perturbed normals as
well. This identifies useful inputs for the next isolated transport test, not
a proof that disabling normals or retuning roughness is the right correction.
The land-only capture is qualitative because geometry/residency is not frozen.

True water presence versus roughness is still an open material/page contract.
No roughness tuning or new mask encoding is hidden in this milestone. Existing
height pages clamp the baked surface to nonnegative height, so replacing the
predicate with a test for negative raster height would not repair that contract.

## Height ancestry experiment: rejected as an accepted horizon fix

Persistent topology requests sparse fine/coarse data; the GPU selects the finest
resident height page and then falls back through ancestors. The visible L17/L7
transition motivated an experiment extending height demand with every ancestor.
It reused the existing bounded job/cache machinery, required no sibling mesh
partition and left the physical field, plants and CBT topology owner unchanged.
A scheduler control exposed the missing parent demand and passed with the
candidate. The raw patch, failing/passing logs and frozen candidate binary are
retained for inspection; **the production policy change and candidate-specific
test were removed from the final change**.

The candidate's five-second-prestream frame had no raised band. Crucially, the
old policy with the same five-second prestream also lost the band. It is
therefore wrong to attribute the disappearance to ancestry. Sparse/transient
data and completion cadence remain implicated; their exact error/publication
mechanism is not isolated. Longer prestream is a diagnostic control, not a fix
for moving-camera streaming.

The old/candidate controls retained 855/891 height pages and 9,714/7,244 active
terrain triangles, with identical primary plant counts/triangles. Each has only
one measured frame, asynchronous residency and serial stage fences. Their
898.242/906.148 ms wall values and 20.124/27.446 ms stream CPU values are recorded
as diagnostic observations, **not a speedup or a proven regression**. No benefit
justifies changing production residency from these measurements. No ancestry
allocation or new mandatory data demand ships with these diagnostics.

A future correction needs fixed-address error against the immutable field and
controlled page arrival/retirement, then transient/moving-camera tests within
the existing memory/job budget. It must distinguish data accuracy from CBT
geometric refinement rather than restoring classic mesh LOD or changing terrain.

## Validation and remaining audit packages

The typed parse/default/resolve test covers all ten values and rejects unknown
ones. 53 graphics and 75 rcbt-core tests pass. The regular/conforming water
instrumentation fixture passes on isolated software Vulkan and RADV; actual native FHD
diagnostic/beauty runs use isolated RADV 880M, no display access, existing per-
stage 20M-triangle admission and unchanged watchdog settings. The first software
attempt caught a WGSL reserved identifier before any hardware run; the renamed
shader passes. Strict Clippy and the Bevy client compile check pass.

Still required: frozen whole-scene capture controls; unique geometry/CBT-depth
and hardware-depth/wireframe output; exact direct/ambient/specular/emission/haze
HDR decomposition; independent stellar contributions; fixed-EV clouds/epoch
matrix; true water presence; bounded source-error/transition tests across more
residency states and paths. Do these before treating SSR/TAA/bloom or new
material tuning as explanations for these defects. Generated artwork remains
valid visual direction; it is not a geometry or physical oracle.
