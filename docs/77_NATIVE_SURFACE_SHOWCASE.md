# 77 — Surface-first visual development

**Status:** in progress, not visual acceptance. Baseline is WORLD05 commit
`54cdfa6`, with the existing canonical WORLD00 shore and prototype plants.
Work is isolated on `feat/native-surface-showcase`; `main` is not changed.

## Target and constraints

Build one convincing, inspectable surface zone before extending its rendering
to the planet. Use WORLD00's surveyed land target at
23.565679258039204° N, 165.13881581863882° E, the unchanged canonical terrain
field and real stellar lighting. No separate showcase terrain, painted shore,
fake atmosphere or camera-specific geometry is introduced.

The standing (2 m eye AGL), close (50 m) and overview (800 m) views share the
numeric target. A moving camera and a surface-to-orbit sequence are additional
acceptance cases, not established by a still image. Plant geometry currently
covers one L14 query window only; this must not be called planet-wide coverage.

## Work order

1. Diagnose the nearly uniform ground and coarse shore in the WORLD05 frames.
   Record actual GPU material residency by level, not just CPU pages or zero
   missing-source counts. Test finite-capacity admission before adding detail.
2. Restore material detail and shore continuity through the existing streamed
   page/ancestor/filtering pipeline. Preserve bounded memory and coarse fallback.
3. Improve substrate/ground-cover and botanical representation where matched
   captures show a remaining gap. Reuse canonical ecology and mesh/material paths.
4. Compose water, terrain, foliage, shadows and sky; validate stability while
   moving rather than tuning an isolated still frame.
5. Expand the validated representation through bounded streaming and quality
   tiers. Retain world identities across transitions; never instantiate a whole
   planet's high-detail mesh/plant population at once.

The unfinished temporal scene integration remains in its separate temporal
worktree. It is not exposed as a working showcase feature here.

## Verification boundary

Same executable settings, source, camera, epoch, resolution and drained-stream
conditions are required for a before/after claim. Preserve PNG, JSON/CSV,
commands, hashes and level counters. Numerical/shader tests establish contracts;
an independent blind review establishes subjective preference. The implementation
owner may inspect frames for defects but cannot self-certify visual acceptance.
Performance requires same-workload measurements on the Radeon 880M/RADV baseline;
neither increased detail nor a different cloud/RT setting is a speedup result.

Starting evidence was WORLD05's
[`native-plant-prototypes-2026-10-21.json`](artifacts/native-plant-prototypes-2026-10-21.json).
Its frames demonstrate prototype geometry, not the requested final visual quality.

## WORLD06a — Saturated material admission

The first matched native captures confirm that the previous global level sort
filled all 256 GPU slots with L0–L9 pages at the standing camera; there were no
L10–L17 entries even after streaming drained. This was not missing CPU material
production. `resident_material_hierarchy` now protects the coarsest resident
support for each path, then emits each path coarse-to-fine in caller priority,
deduplicating shared sources. Native GPU publication, Bevy GPU publication and
CPU source-budget trimming reuse this one function. Texture capacity, page
formats, source versions, material filtering, canonical field and demand
selection are unchanged. Disconnected sources without coarser resident support
remain protected; very small budgets can still defer refinement.

`MaterialUpload.resident_levels[0..32]` counts actual GPU directory entries,
including retained cold slots, after each publication. Native initialization
and streaming retain the latest histogram, exported as
`native_material_resident_levels` in performance metadata and
`material_gpu_resident_levels` in the headless log. Counts are not CPU-ready
pages, material demand, visible-pixel coverage or a general LOD-quality metric.

The repeated standing capture contains 36 L17 GPU pages rather than zero. Ground
variation is still weak; plants are still schematic and the overview shoreline
still exposes coarse/fallback boundaries. This fixes a confirmed admission defect,
not the requested final visual quality. Actual PNG/JSON/CSV/log paths, hashes,
commands and all three residency pairs are recorded in
[`native-material-admission-2026-10-22.json`](artifacts/native-material-admission-2026-10-22.json).
The fixed 60-second prestream and measured frames were serial on the GPU, but
some CPU builds overlapped: timings are diagnostic, not stable performance
acceptance. No blind preference, camera-motion or planet-wide foliage claim is
made for this step.

The existing arena prepared the three unlabeled pairs under
`target/surface-showcase/review/`; no reviewer votes have been collected. Its
private answer key is outside that review directory and is not committed.

Verification on the 880M/RADV: formatting, architecture guard and workspace
Clippy passed; 2,029 workspace lib/bin tests, 103 integration tests and 79 ignored
renderer hardware tests passed. The mandated Bevy ignored-test command also
completed, but selects zero tests on this branch: retained GPU tests have moved
to `render-wgpu/tests/retained_surface.rs` and ran in the renderer suite. The new
saturated-cache GPU oracle checks actual blue fine-page RGB against red coarse
fallback, level counts and zero reuploads for unchanged demand; it passed
separately as well as within workspace integration tests. Tests establish cache
and raster correctness, not subjective quality or a performance improvement.

## WORLD06b — Separate stem and foliage reflectance

Previously the native adapter assigned the same green albedo to a tree's stem,
branch stubs and foliage. `offline::PlantMesh` now has optional `stem` metadata:
the vertex-prefix count and authored sRGB stem reflectance. The producer marks
the existing trunk/branch vertices before emitting the crown; positions, winding,
normals, topology, scatter IDs and world transforms do not change. The lycopsid
stem reflectance is `[0.29, 0.25, 0.17]`, an authored weathered optical prior,
not a measured fossil colour or new structural/material physics.

This is an additive optional field in the existing frozen prototype payload.
Historical bakes without `stem` remain valid single-material prototypes; no
terrain rebake or replacement of existing bundled source data occurs. A marked
triangle must belong entirely to one organ, and stem metadata is range/finite
checked by the existing mesh validation path.

The native adapter publishes a two-texel sRGB palette through the existing
`MeshAsset` base texture and assigns a constant texel-centre UV to each organ.
It does not introduce a plant shader, pass, extra primitive/draw, instance range,
RT object, normal map, or per-frame texture update. Shared raster and secondary
mesh materials read the same texture; alpha remains one, so shadow geometry is
unchanged. Each triangle has identical UVs at all three corners: there is no
cross-organ interpolation or high-frequency texture pattern requiring mipmaps.
Clamp addressing prevents wrapping into the other swatch. Palette quantization
is bounded by half an 8-bit sRGB code value per channel; tests bound the decoded
linear error by 0.0045. Non-stem prototypes retain their exact float albedo.

`[surface_plants].enabled = false` still does no sampler/mesh/palette work. All
existing Low/Medium/High density tiers use the same organ reflectances and the
same fallback behavior. The sole new semantic image is eight bytes for the
lycopsid prototype, independent of instance count; renderer allocation/alignment
is not eight bytes. `native_surface_plants_prototype_texture_bytes` reports the
source image payload separately from geometry and instance buffers. Native
default plants remain off. No throughput, visual preference, temporal stability
or final surface-quality claim follows from this material correction.

Matched silent captures at 2 m / 50 m / 800 m eye AGL are under
`target/surface-showcase/organ-materials/after-{standing2,close50,overview}.png`,
with JSON/CSV/logs beside them. They retain 222 descriptors, three draws and the
same actual GPU material-level histograms as WORLD06a. The bottom 240 rows of
the standing capture are bit-identical: the material correction does not repair
the ground or change its illumination. The new GPU fixture exercises both
palette swatches under linear filtering at full and 0.05 instance scale; it is
an extension of the retained textured-mesh oracle, not a subjective review.
Unlabeled pairs are prepared in the stage's `review/` directory; votes remain
pending. Paths, hashes, bounds and check results are in
[`native-plant-organ-materials-2026-10-22.json`](artifacts/native-plant-organ-materials-2026-10-22.json).

Stage checks passed: formatting, architecture guard, workspace Clippy, 2,030
workspace lib/bin tests, 103 integration tests and 79 ignored renderer hardware
tests. The mandated Bevy ignored command again selects zero tests; migrated
retained tests ran in the renderer suite. The extended GPU palette oracle passed
within integration tests. These checks do not certify aesthetic preference.

### Remaining ground limitation

A standalone probe linked to the actual release worldgen/flight-authority
libraries queried the attached canonical field, not a new showcase generator.
WORLD00's L17 material page has nominal 0.390625 m texels, yet its RGB code ranges
are only `[30,32]`, `[56,59]`, `[34,36]` over the page; standard deviations are
0.545 / 0.914 / 0.647. This is source-material uniformity, not evidence that more
GPU residency or more texture resolution alone will produce convincing ground.
The next optical work must distinguish resolved forest floor from the current
canopy/ground/reed averaged cover while preserving climate, snow, wetland and
canonical height semantics. No ground change is claimed by WORLD06b.

## WORLD06c — Rejected forest-floor prototype

A canopy-gated, footprint-filtered litter/live-ground colour mixture was tested
in the existing material producer. It passed its numerical tests but produced
large blurry patches rather than convincing resolved ground in the standing
preview. The prototype was removed; production ground remains unchanged.
The source patch and negative result are preserved in
[`native-forest-floor-rejected-2026-10-22.json`](artifacts/native-forest-floor-rejected-2026-10-22.json).
This is an author defect rejection, not independent blind preference evidence.

The preview used 10 seconds of prestream and eight frames, so it is not a matched
drained-stream acceptance capture or a performance measurement. A baseline
material-page benchmark was retained, but no before/after performance claim is
made. Next work needs resolved material structure and filtering at standing
scale, not stronger macro noise. No external texture asset was imported.

## WORLD06d — Preserve local material coordinates

The regular-patch raster previously reconstructed material UV from a normalized,
planet-sized `f32` direction. This discards small surface displacements before
the shader even looks up a detailed material page. A new real-raster oracle
exposes the defect: at radius 3,200 km, level 17, off-centre chart coordinates,
the legacy path misses the independently filtered RGB8 reference by 19 code
values on its first sample. The same fixture passes with split coordinates.

The regular path now transports the existing dyadic tile-centre UV separately
from its small grid-local displacement. Displacement is interpolated with the
dominant direction component as a divisor, so projection is rational rather
than a linear approximation to sphere UV. The common page sampler scales the
anchor and displacement separately before forming integer tile and local
coordinates. Albedo, normal maps, page feathering and derivative footprints use
this same address. Same-face neighbour lookup keeps the integer tile address;
cross-face neighbours retain the existing cube reprojection.

This is a precision correction, not a new effect or additional quality tier.
It introduces no texture, allocation, binding, pass, physics state or source
material. Regular raster vertex output gains six 32-bit components (local
numerator/divisor, flat anchor and face); existing shadow/geometry buffers are
unchanged. In the compute emulation of mesh output, WGSL alignment changes the
shared vertex record from 80 to 112 bytes, not 104; the retained emission oracle
now checks the new chart fields as well as complete/tail/skirt geometry. Its
initial stale-stride failure was corrected rather than dropping the test.
The shared sampler's zero-offset constructor preserves callers that
only have direction data. The conforming raster, terrain rays and ocean still
use that legacy direction path: this stage does **not** establish centimetre
addressing for every consumer. At WORLD06d, screen-space page-level selection
still used direction derivatives and the material limit remained L17; the
opt-in continuation below changes these two regular-patch constraints.

The GPU coordinate oracle compares 3,150 split-coordinate cases to independent
`f64` arithmetic: six faces, anchor and requested levels 0/10/17/20/22, outer
and interior tiles, positive/negative displacement, and exact page edges.
Integer page indices must agree exactly; local error must be below 2e-6 page
units for this fixture. The regular-raster oracle sweeps nine subtexel camera
positions on each of six faces, uses a sharp two-colour material boundary,
linear-light filtering and ambient PI, and checks output within two RGB8 levels
of the independent reference with strictly increasing samples. Its nominal
world step is 48.828125/4096 = 0.01192093 m before chart distortion. These are
numerical bounds for these fixtures, not a global seam/temporal envelope or an
independent aesthetic judgment.

Local gates passed: formatting, architecture guard, workspace Clippy, 2,030
workspace lib/bin tests, 103 integration tests and 81 ignored renderer hardware
tests on Radeon 880M/Vulkan/RADV. The required Bevy ignored command selected zero
tests; it is not an additional hardware-test result. The release client built
successfully. Exact commands, shader/test hashes and the intentional legacy-path
failure are recorded in
[`native-material-coordinates-2026-10-22.json`](artifacts/native-material-coordinates-2026-10-22.json).

Silent standing/close/overview captures are under
`target/surface-showcase/material-coordinates/after-{standing2,close50,overview}.png`,
with metadata and timings beside them. A fresh standing baseline replay and all
three candidate captures ran serially after builds finished. All views retain
the previous GPU material-level histograms, 222 plant descriptors and three plant
draws. The same-binary standing replay differs from the earlier WORLD06b capture
at three pixels (maximum channel delta 14), so full-scene pixel diffs are not an
exact deterministic oracle. The isolated raster/reference fixtures above are the
correctness evidence. Three unlabeled pairs are prepared; no reviewer votes or
subjective preference result is claimed.

The fresh standing pair reports median GPU frame times of 104.781 ms before and
97.844 ms after (32 frames each, 60 s prestream). This single pair is diagnostic,
not a counterbalanced repeat study, a speedup or a throughput/no-regression
certificate. Prior close/overview baselines had overlapping CPU checks and are
not timing baselines. The captures still show nearly uniform ground, schematic
plants and coarse shoreline/fallback boundaries: local coordinate accuracy is a
prerequisite, **not** the requested convincing surface zone or planet expansion.

## WORLD06e — Opt-in resolved ground experiment (not visually accepted)

`renderer.terrain_ground_detail = false` is the default. Enabling it requests a
CC0 soil/gravel optical swatch through the existing canonical material producer,
normal plane, mips, residency and raster path. There is no additional GPU pass,
texture-array plane, geometry, collision, population or simulation state.
The resolver disables this experiment when conforming topology is active and
reports the reason: that consumer still lacks precise local material addresses.
Unsupported conforming topology may itself fall back to regular patches, where
the experiment remains available. Secondary rays/ocean are not upgraded.

The asset is Poly Haven **Forest Ground 04**, by Rob Tuytel and Rico Cilliers.
Source PNG MD5s are pinned in `tools/worldgen-rocky/import_ground_detail.py`;
the derived 512-square swatch contains sRGB reflectance, perceptual roughness
and relative height. RGB downsampling and runtime mips operate in linear light.
The 3.15 m repeat and 18 mm relative-height range are explicit authored optical
priors, not measured Thessa geology or calibrated scan dimensions. See
`assets/textures/forest-ground-04.LICENSE.txt`; no new runtime dependency is added.
The body-fixed, seed-phased triplanar projection has no page/camera/worker ID.
It filters the scan before sampling and derives material normals from physical
chart chords; coverage weights apply **after** differentiating the height, so
a habitat boundary cannot become a false cliff. Existing water, wetland, rock,
salt, snow and frost layers retain their ordered ownership.

With the experiment enabled, resolved `surface_quality` Low/Medium/High caps
material refinement at L20/L21/L22 (nominal 4.88/2.44/1.22 cm texels at WORLD00's
radius). Disabled retains L17 and performs no swatch decode/sampling. The same
512-leaf demand, 1024 CPU-page source budget, bounded worker queue and 256 GPU
layers remain in use; higher resolution does not imply more cache capacity or
planet-wide fine residency. Material policy is immutable for a stream lifetime:
changing it requires restart, avoiding mixed old/new optical pages.
The packed asset is 1,382,947 bytes in the executable; its ten decoded mips hold
6,990,500 bytes of float samples, allocated once only when resolved detail is
sampled. Coarse pages use pinned constants without decoding. This CPU source
cost is separate from existing GPU layer storage/upload telemetry; normal
generation still has its independent disabled state.

Regular raster computes symmetric angular chords analytically from the local
cube-chart derivatives, rather than subtracting nearly equal absolute `f32`
directions. Rationalizing the normalization-length difference preserves small
chords while bounding coarse footprints by the sphere's diameter. The shader
ceiling follows the resolved material tier. This
is separate from depth-buffer precision and does not migrate forward-Z.
The GPU footprint oracle compares 432 cases (six chart projections, four chart
locations, nine scales from 1 mm to 12,800 km and two derivative skews) against
independent `f64` central differences. The fixture tolerance is 1 micrometre plus
2 ppm of footprint, not a global image-error or temporal envelope. The existing
actual-raster sharp-boundary test now also exercises L21 and L22: 162 camera
samples across all faces/three levels, within two RGB8 codes of linear-light
bilinear filtering, with strictly increasing subtexel-motion samples.

The first full hardware gate caught two genuine regressions in the initially
linearized footprint: coarse grazing pixels could acquire an unbounded footprint
and incorrectly replace resident materials with the globe fallback. The existing
grazing-GGX and dielectric-furnace tests failed; their tolerances were **not**
changed. Replacing the linearized metric with the exact stable angular chord
restored those tests, and the footprint oracle was expanded through planetary
pixel scales to pin this case. The failed gate log is retained separately from
the subsequent verification.

The first preview blended legacy green cover into resolved floor only below
25 cm texels. It produced an obvious brown detail island around the camera and
was not retained as the production algorithm. All enabled land pages now sample
the same floor material, with the last linear-light mip used as a constant
coarse fallback. Its mean is pinned against the decoded asset within 2e-7 per
channel; those coarse samples need no asset decoding. Finest source resolution
was also extended to L22 rather than magnifying the L17 material.

**Open limitations:** this floor swatch is not a canopy representation hierarchy,
3D gravel, a calibrated BRDF or a general biome asset catalogue. Globe-map
fallback boundaries, scan repetition, normal filtering, close assets, motion,
and independent preference acceptance remain unaccepted. Do not enable it by
default or call the surface complete. The rejected WORLD06c noise experiment
above remains rejected; this is a distinct imported-material experiment.

Final verification passed: formatting, architecture guard, workspace Clippy,
2,036 unit/bin tests, 103 integration tests, 82 ignored renderer hardware tests
on Radeon 880M/Vulkan/RADV, and the release build. The mandated Bevy ignored
command selected zero tests after migration. Final silent standing/close/overview
captures are `target/surface-showcase/ground-detail/v3-on-*.png`, with a matched
same-binary standing disabled control, JSON/CSV and logs alongside them. One
blind pair is prepared with its key separate; zero votes have been collected.

The standing frame has eight L21 GPU material pages but **no L22 residency**.
L22 is a supported ceiling and isolated raster-oracle result, not verified live
coverage. All captures retain 222 plant descriptors and three plant draws. The
final standing off/on GPU medians are 93.315/93.955 ms (one serial diagnostic pair,
not performance acceptance). The separate 32-page paired producer fixture reports
134.994/139.684 ms page medians without/with the swatch; it is not a frame or
streaming-latency benchmark. The ground no longer changes back to green at the
old source-texel activation threshold, but it remains a flat optical proxy,
plants are schematic and the shore is coarse. No convincing-zone or planetary
visual delivery is claimed. Commands, hashes, numerical bounds, failed attempts
and limitations are recorded in
[`native-ground-detail-2026-10-23.json`](artifacts/native-ground-detail-2026-10-23.json).

## WORLD06f — Botanical geometry continuation (not visually accepted)

The same shared `offline::plant_prototypes` producer now emits fern pinnae as
separate leaf geometry and lycopsid crowns on two successive fork levels. The
tree has four terminals with 56 leaves each, smooth cone-side stem normals and
closed terminal ends. No scatter, canonical root, yaw/scale, density policy,
optical material, grouped draw, shadow pass, simulation or collision state is
changed. Historical frozen prototypes are not rebaked. See
[doc 75](75_NATIVE_SURFACE_PLANTS.md#world06f--pinnate-ferns-and-forked-crowns)
for dimensions, bounded payload, geometric/error oracles and remaining limits.

The first forked-crown attempt used 28 leaves per terminal, only 0.58–0.88 m
long. Its narrow blades collapsed into dots at the matched 50 m camera; the
more detailed branching did not compensate for sparse projected leaf area.
That attempt is retained as negative evidence, not shipped as the solution.
The replacement has 1.05–1.45 m leaves, with authored one-sided foliage area
20–30 m² pinned by a geometric regression. These are actual leaves entering the
existing primary/shadow/ray mesh paths, not opaque canopy cards, fake ambient
brightening or camera-facing geometry. This is still an authored alien-tree
proxy, not a fossil reconstruction or validated growth/leaf-transport model.

Total prototype triangles increase from 632 to 4,284; native static vertex/index
payload grows from 68,256 to 462,672 B. Per-prototype source validation is bounded
at 8,192 vertices. The existing default-off plant setting and Low/Medium/High
nested density subsets remain the disabled/cheaper fallbacks. More source and
submitted/shadow/RT triangles are deliberate costs and must not be called a
speedup. No new textures, render passes or material-specific shader are added.

The final 2/50/800 m matched captures retain exactly the same survey/source
metadata, 222 descriptors, three draws and 14,784 frame-upload bytes, with drained
stationary streams. Whole-client GPU medians increase from 99.248 to 111.610 ms,
95.591 to 124.532 ms and 76.509 to 78.096 ms respectively. These are single serial
pairs, not accepted performance; distant dense foliage still needs mesh LOD.
The crown is readable in the candidate, but thin leaves alias, shadows show a
coarse grid, understory is sparse and the ground/shore remain unconvincing. A
64-frame orbit sweep records an endpoint and telemetry only, not a motion review.
Three independent-review pairs are prepared; zero votes, no visual acceptance.

All required local code gates passed, including 2,038 unit/bin tests, 103
integration tests, 82 renderer hardware tests and two focused native plant GPU
diagnostics. Exact commands, source/binary/frame hashes, cost, failed attempts
and omissions:
[`native-botanical-geometry-2026-10-23.json`](artifacts/native-botanical-geometry-2026-10-23.json).

## WORLD06g — Terrain-aware demand and grazing material filtering (candidate)

The opt-in ground experiment now uses the canonical field's local height at the
eye direction (32 m wavelength, cached with material selection) rather than
treating every material page as if it lay on the zero-elevation datum. Refinement,
frustum testing and material admission share that reference. No body radius,
geometry, height source, physics or cache budget changes. It is a local reference,
not exact relief under every page. Disabled detail preserves legacy selection.
Synthetic standing fixtures at elevations -1,000/0/+1,000 m reach L22 within the
same 512-leaf, area-preserving bound; the legacy datum fixture remains L17 at
nonzero elevation. Epoch/pixel-scale caching and disabled datum behavior are
regression tested. Runtime captures record the actual reference and its limits.

The production material sampler previously had no anisotropy, and page selection
used the largest footprint even though its other axis could resolve much more.
The same ground-detail switch and surface-quality policy now resolve 1/4/16x
anisotropy through the adapter's capability probe; unknown/unsupported hardware
falls back to 1x, and conforming/disabled ground detail retain 1x. The raster
chooses page resolution from `max(minor_axis, major_axis / sampler_anisotropy)`;
coverage/globe transitions retain the conservative footprint. This changes
filtering accuracy/bandwidth, not the material source, lighting or simulation.

The GPU footprint oracle now checks 864 cases and four outputs per case: the old
conservative footprint and 1/4/16x bounded elliptical metrics. Independent f64
cube-projection finite differences and Gram eigenvalues supply the reference,
with the existing 1 micrometre + 2 ppm tolerance. The first implementation
underflowed tiny squared cross products at millimetre scales; normalizing the
metric before computing its area fixed that failure without changing tolerances.
A production grazing-raster fixture uses red ancestor pages and a blue fine page
to verify that 16x admits fine material while 1x keeps the coarse representation.
It is an addressing/filter-policy oracle, not a photorealistic texture or temporal
acceptance test.

The full local gates passed: formatting, architecture, workspace Clippy, 2,041
lib/bin tests, 103 integration tests, 83 renderer hardware tests and release
build. The retired Bevy hardware command again selected zero tests. A fresh
matched standing pair measured 116.204/111.124 ms median GPU time before/after;
one serial pair is not stable performance evidence. The candidate reference
height is 5.008 m and eight material pages reach L22, versus a finest resident
L21 before. All static captures end with zero height/material jobs and material
misses; plants remain 222 descriptors, three draws and 14,784 upload bytes.

The foreground still reads as a flat optical proxy. Sharper distant horizontal
bands remain visible, and the canopy, coarse shadows and empty understory are
not convincing. Neither successful GPU oracles nor L22 residency establish
visual acceptance. One blind standing pair is prepared, with separate keys and
zero votes. Candidate close/overview frames are diagnostics, not fresh paired
cost measurements. Reproduction/evidence:
[`native-ground-filtering-2026-10-23.json`](artifacts/native-ground-filtering-2026-10-23.json).

Two single-setting diagnostic captures separate the failures: disabling raster
shadows removes the dark trunk stripes but leaves the horizontal ground bands;
disabling terrain normal maps removes the distant ground bands while leaving
coarse/trunk shadows. This isolates a normal-map response/filtering issue, not a
reason to erase surface relief or disable shadows in the accepted scene. The
feature-off build and fixed L14 plant-root GPU diagnostic also pass.

## WORLD06h — Geometric mesh receivers and near shadow precision (candidate)

The optional shared `shadows.first_cascade_distance_m` gives the near cascade an
explicit radius while preserving cascade count, atlas resolution/memory and far
cover. Native intermediate radii use geometric spacing; Bevy forwards it into
the existing engine cascade builder. Omitted configuration keeps each host's
legacy behavior. Finite requests clamp to 1 m..far cover, non-finite requests
fall back with a diagnostic, and a single cascade still covers the far endpoint.
It is an extension of the shared graphics/atlas pipeline, not a second shadow
system, hidden prerequisite, vendor check or change to simulation/causality.

The canonical-site policy requests four 4096 px cascades and a 64 MiB three-star
atlas budget, which resolves to 1024 px per layer / 48 MiB. Legacy native radii
are 754.460/1897.367/4771.624/12000 m; the old comment saying the first was
`max/40` described the Bevy builder, not this native formula. A 25 m first cover
redistributes those same maps to 25/195.743/1532.619/12000 m: first-map texels
improve from 1.479 m to 4.902 cm, including snapping guards. This trades spatial
precision between cascades, not material spectrum, physical light transport or
quality by increasing GPU memory. No source height, plant geometry or placement
changes accompany it.

Targeted CPU tests cover exact legacy equivalence, count/endpoints, monotonic
geometric spacing, invalid values, requested/resolved round trips, unchanged
resolution/filter/count budgets and disabled behavior. All new projection
fixtures remain within a conservative 1 cm clip-space narrowing envelope even
at 3e12 m origins and 12 km caster reach. The shared production GPU fixture
runs both legacy and explicit cover, retaining per-light occlusion, cache/forced
redraw identity and large-origin precision. These are numerical/resource
oracles; canonical captures and visual/performance acceptance are separate.

The first canonical-site captures, retained as `near-native-v1` and the
`near-*`/`legacy-lowbias-*` frames under `target/surface-showcase/near-shadow`,
showed that redistributing cover did not by itself eliminate trunk stripes.
Reducing the existing metre bias from 1.5 m to 0.02 m was a diagnostic, not a
retained default or a demonstrated global error bound. Those frames are not
the corrected receiver candidate.

Inspection then found the mesh receiver used its smooth/normal-mapped shading
normal to reconstruct the PCF receiver depth plane. Terrain already used the
raster triangle's geometric plane. Mesh raster now reuses that same derivative
construction, including orientation and degenerate fallback; BRDF normals and
caster geometry stay unchanged. The visibility consumer receives geometry,
including bias for optional ray queries, rather than bump-map perturbations.
This is a shared mesh correctness fix, not a plant-specific shader or renderer.

A new production GPU regression uses flat z=-4 m geometry, strongly varying
interpolated normals, zero metre bias and no blocker. Against the exact same
lighting with shadows disabled, the old implementation produced an 18-code
sRGB maximum error; the corrected plane passes the unchanged <=1-code tolerance.
The failing `receiver-before.log` and passing `receiver-after.log` are retained.
The final candidate must still be judged in matched canonical frames; successful
oracles and first-map texel size are not visual acceptance.

The final v2 pipeline passes formatting, architecture, workspace Clippy, 2,043
lib/bin tests, 105 integration tests (including both new actual-GPU fixtures),
83 ignored renderer hardware tests, the focused native plant lifecycle/contact
diagnostics, feature-off check and release build. The retired Bevy ignored
command again selects zero tests. Static streams drain fully; plants remain
222 descriptors, three draws and 14,784 upload bytes. Near standing/close pairs
also match survey/source metadata, material residency histograms, anisotropy
and native map resolution. Equal histograms do not prove equal resident IDs
or bytes; no stronger content determinism is claimed.

Final GPU medians are 106.669/114.963 ms before/near-after standing and
127.386/123.936 ms close; overview candidate is 76.005 ms. These are single serial
pairs, not an accepted budget or speedup. The legacy-cover after-control is
116.543 ms and differs in a few intermediate material residency counts from
before despite drained streams: its whole-frame delta is not shadow-only.
Three blind pairs are prepared with separate keys and zero votes; that legacy
control pair should be treated as invalid for a pure shadow comparison.

Some trunk stripes, grid-like shadows, aliasing thin leaves and the unconvincing
ground/shore remain visible. The exact flat-plane correction is retained, but
neither it nor the 25 m optional cover is final surface/visual acceptance. The
0.02 m experimental bias is not retained; final near captures keep 1.5 m.
Commands, source/binary/frame hashes, negative attempts, numerical bounds and
limits:
[`native-near-shadow-receivers-2026-10-23.json`](artifacts/native-near-shadow-receivers-2026-10-23.json).

Final formatting, workspace Clippy and release rebuild were repeated after an
explanatory legacy-host comment edit; the current release is byte-identical to
the captured v2 executable. Retained source/frame hashes and the parsed
single-setting near-cover delta were independently checked locally.
