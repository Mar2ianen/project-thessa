# 77 — Surface-first visual development

**Status:** in progress, not visual acceptance. Baseline is WORLD05 commit
`54cdfa6`, with the existing canonical WORLD00 shore and prototype plants.
The foundation through `f73eea4` is integrated into main (doc 78); subsequent
visual development continues on `feat/native-surface-showcase`.

## Target and constraints

Build one convincing, inspectable surface zone before extending its rendering
to the planet. Use WORLD00's surveyed land target at
23.565679258039204° N, 165.13881581863882° E, the unchanged canonical terrain
field and real stellar lighting. No separate showcase terrain, painted shore,
fake atmosphere or camera-specific geometry is introduced.

The standing (2 m eye AGL), close (50 m) and overview (800 m) views share the
numeric target. A moving camera and a surface-to-orbit sequence are additional
acceptance cases, not established by a still image. Large-plant geometry covers
one L14 query window; the optional near layer adds at most nine L18 query windows.
Neither is planet-wide coverage.

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

Default-off native temporal history is integrated as a partial T4c capability
(doc 76). It excludes all rigid meshes, including plants; it does not establish
foliage antialiasing or complete temporal/visual acceptance.

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

## WORLD08 — Readable leaf surfaces and understory colonies

This continuation targets two visible defects in the retained `f73eea4` frame:
narrow wire-like crowns and uniformly scattered ground clumps. It changes the
existing shared plant producer and optional understory density, leaving the
canonical terrain, material transport and stellar/sky lighting intact.

Tree terminals now carry 24 broader leaves rather than 56 narrow ones, and fern
fronds carry seven pairs of broader pinnae with real gaps. Reed blades bend and
droop farther instead of reading as upright slabs. One-sided tree foliage area
increases 23.892 → 41.829 m²; fern area increases 0.929 → 1.710 m². All three
prototypes together fall from 4,284 to 2,948 triangles (462,672 → 318,384 native
vertex/index bytes). This is authored optical geometry, not screen-space alpha
dilation, a botanical reconstruction or a guaranteed frame-time improvement.

Understory samples the shared 3D value-noise kernel in body-fixed metres at an
authored eight-metre wavelength, mapping colony occupancy to [0.15, 1.85] times
the existing local ground/reed density. Canonical ecology/water/height/slope still
decide eligibility. The same field is used by every query window and quality
tier; the capacity envelope includes its maximum and gaps reject before the
expensive surface sample. It is a spatial occupancy prior, not new hydrology or
plant growth. Rock/large-plant IDs and transforms remain unchanged; understory
membership can change. Full representation and numerical boundaries are in
[doc 75](75_NATIVE_SURFACE_PLANTS.md) and [doc 70](70_WORLD_SCATTER_IDENTITY.md).

An ImageGen concept informed leaf masses and the alternation of colonies and
open soil under muted overcast light. It is a visual direction reference, not
an engine capture, ground-truth lighting oracle or a shipped asset. The renderer
continues to use actual opaque double-sided leaf triangles and the existing
shared raster/shadow/optional RT paths. No lighting coefficient, extra pass,
alpha-to-coverage, mesh temporal eligibility or launch-default change is added.

The remaining material-normal ground bands, diffuse canopy transport/contact,
subpixel foliage aliasing, streamed roots, near/far continuity and planetary
coverage are still open. Broader silhouettes do not prove temporal stability,
physical transport convergence or subjective preference.

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

## WORLD06i — One-sided shared GGX support (candidate)

The ground-normal investigation used the committed WORLD06h binary and existing
shared settings, not a new renderer path. Disabling specular reflections removes
the horizontal bands even with material normals enabled; reducing surface
quality to Medium changes but does not eliminate them. Disabling clouds leaves
bands visible, rejecting the reflected-cloud hypothesis. The diagnostic images,
settings and complete commands are under
`target/surface-showcase/normal-grazing/isolation-commands.json`. These are
single-setting diagnostic scenes, not accepted defaults or blind preference
comparisons: the final scene must keep its physically caused reflections.

Inspection identified an independently reproducible shared-kernel bug: VNDF
sampling clamped `N.V` to zero even for negative cosines but retained the original
view vector. That samples a visible grazing lobe for a back-facing shading
normal; an exactly opposite view also normalizes a zero stretched vector and
propagates NaNs. The shared kernel now returns zero directional weight strictly
for `N.V < 0`, retaining its existing finite exact-zero/positive grazing limit.
No normal/view rotation, radiance floor, source-field change, surface flattening,
simulation change, new setting, pass or resource is introduced.

The actual-WGSL GPU oracle checks 45 combinations: five negative cosines down to
-1e-7, roughness 0.25/0.5/1 and 4/8/16 samples, with F0 RGB = 0.04/0.5/0.9.
Before correction, 39 cases were nonzero/non-finite; at `N.V=-0.1`, roughness
0.5, 16 samples and uniform radiance 100, the old reflected RGB was
7.440/18.497/28.111, although the one-sided kernel has no outgoing support.
At `N.V=-1` it produced NaNs. The unchanged exact-zero output oracle passes after
correction, and the prior independent solid-angle grazing test also passes with
its original accuracy/convergence bounds. Failing and passing logs are retained
as `hemisphere-before-v2.log`, `hemisphere-after.log` and `grazing-after.log`.

This fixes kernel support, not the complete bump-mapped-surface transport model:
microgeometry visibility, unresolved normal variance and shading-normal energy
compensation remain unimplemented. Canonical frames must still be inspected;
no full visual/temporal or performance acceptance follows from the regression.

The v1 canonical standing capture confirms the kernel fix is not a complete
solution to the horizontal bands: they remain visible. The matched pair retains
the same survey/source metadata, material residency histogram, 222 plant
descriptors, three draws and 14,784 upload bytes, with drained height/material
streams. Its GPU medians are 117.118/107.111 ms before/after; the candidate-only
50 m view is 110.663 ms. These single serial observations are not a speedup or
accepted budget. One blind standing pair is prepared with separate keys and
zero votes. No reflection or normal-map switch is disabled in the candidate.

Full local gates pass: formatting, architecture, workspace Clippy, 2,043 lib/bin
tests, 105 integration tests, 84 renderer hardware tests, feature-off and release
build. The mandated retired Bevy command selects zero tests. After formatting,
the release rebuild matches the captured executable byte-for-byte. This is a
scoped numerical correction, not a visual acceptance milestone. Exact commands,
source/binary/frame hashes, rejected hypotheses, results and remaining gaps:
[`native-ggx-hemisphere-2026-10-23.json`](artifacts/native-ggx-hemisphere-2026-10-23.json).

## WORLD06j — Direct-lobe / outgoing-support diagnosis

This is a diagnostic continuation of `4a536c0`, not a shipped visual correction.
It keeps one independent GPU quadrature oracle and removes all temporary render
changes. The standing scene, canonical geometry/ecology, material sources,
graphics settings and production sampling budgets remain unchanged.

### Rejected quadrature fix

A clear-sky control and a temporary 256-sample terrain build share the same
settings, frozen epoch, survey camera, material residency histogram, 222 plant
descriptors, three draws and drained streams. The visible horizontal bands
remain. Whole-frame mean absolute RGB code differences are only
0.01834/0.01898/0.02938; individual changed pixels are not a substitute for a
subjective comparison. GPU medians are 25.844/167.921 ms for four measured frames
after 30 warmups. This is a short diagnostic, not a stable performance estimate;
there is no reason to retain this cost increase as a visual fix. Its temporary
assembly patch, executable and commands are preserved locally as
`dense256-source.patch`, `dense256-native` and `dense256-commands.json`.

A retained 27-case actual-WGSL probe compares 16 and 4096 samples to independent
uniform incident-solid-angle f64 quadrature of the same bilinear 32x16 field.
The synthetic sky has an oblique horizon, a smooth bright lobe and an absolute
source bound of 100; it is not the canonical scene's actual sky. Small normal
tilts (-0.05/0/0.05), view cosines 0.001/0.02/0.2 and roughness 0.6/0.85/1 are
covered. Measured maximum linear-radiance errors are 0.0317001 for 16 samples,
0.0000301842 for 4096 samples and 0.0000124674 for 256-to-512 reference-grid
convergence. Pinned bounds are respectively 0.04/0.0001/0.00003. These are scoped
absolute source-scale envelopes; a small percentage of source radiance can
still be a significant relative error in a weak reflected term. No general HDR,
sky-grid, cloud quadrature or image-quality guarantee is inferred.

### Isolated direct lobe and rejected partial visibility prototype

A temporary split fragment retains the existing diffuse/energy split but shows
only environment reflection on the left and only direct stellar reflection on
the right. The characteristic bands remain on the direct-only side, not the
environment-only side. A second false-colour fragment encodes signed shading
`N.V`, macro `N.V` and the shading-normal outgoing-support mask; its contours
track the bands. These false-colour/source-isolation frames are diagnostics,
not appearance candidates or valid blind preference pairs.

A CPU probe reused the actual celestial presentation and baked ephemeris at
epoch zero: all three stars are approximately 62.34–62.37 degrees above the
local horizon. Irradiances are 958.119/10.8984/0.013656 W/m². This does not support
a hidden-below-datum-star explanation for this case. The remaining investigation
concerns shading-normal projected area, microvisibility and resolved/unresolved
normal filtering; neither the cloud boundary nor sky sample count explains it.

A temporary direct-only prototype independently evaluated the single-scattering
two-facet construction from Schüssler et al., *Microfacet-based Normal Mapping
for Robust Monte Carlo Path Tracing* (2017), equations 4/8/13/15:
<https://jo.dreggn.org/home/2017_normalmap.pdf>. Both perturbed and compensating
tangent facets used the existing GGX material. The standing clear/cloudy frames
soften some contours but remain inadequate. More importantly, the prototype did
not extend environment reflection and substrate transmission consistently, and
has no accepted geometry/energy oracle. It is **removed**, not exposed through a
local toggle, retained as a production approximation or presented as a complete
normal-mapping model. Its patch and captures are preserved for reproducibility.

All diagnostic executables, six PNG/JSON/CSV/log captures, command manifests,
source patches, celestial probe and paper hash are under
`target/surface-showcase/normal-grazing/`; none of these short diagnostic timings
establishes a performance win. All six captures have matching level histograms,
222 plants, three draws and drained streams, but histograms do not prove identical
resident page IDs/content. Independent visual review remains pending.

Full local gates pass: formatting, architecture, workspace Clippy, 2,043 lib/bin
tests, 105 integration tests, 85 renderer hardware tests, feature-off and release
build. The mandated retired Bevy command selects zero tests. The restored release
is byte-identical to the captured WORLD06i executable: none of the temporary
sampling, split-source, false-colour or two-facet changes remains. Existing README
prototype/visual-acceptance statements remain accurate. Exact commands, hashes,
negative evidence and limitations:
[`native-direct-lobe-diagnosis-2026-10-23.json`](artifacts/native-direct-lobe-diagnosis-2026-10-23.json).

## WORLD06k — Zero-exact signed material normals (candidate)

Following the direct-lobe diagnosis, inspection identified a representation
defect independent of the proposed microvisibility model: the old unsigned RG8
contract cannot store zero under `2*byte/255-1`. Its nominal flat `[128,128]`
normal has XY = `[1/255,1/255]`, approximately 0.318 degrees of artificial tilt.
Even arbitrarily small slopes round to nonzero positive/negative normals. At a
grazing view this can change the one-sided BRDF's outgoing support; it is not
evidence that the source terrain contains this tilt.

The shared page producer and CPU slope mip builder now encode signed normal
components with `round(component*127)`; `Rg8Snorm` hardware filtering decodes
them before interpolation, and the raster sampler no longer applies `*2-1`.
Zero is exactly zero at every mip, and opposite slopes round symmetrically.
The existing RG two-byte payload, directory, residency, eight mips, sampler
anisotropy, +2/+1/+0 normal mip tiers and disabled allocation/work contracts are
unchanged. No new graphics toggle, pass, radiance multiplier, normal rotation,
roughness change, mesh, physical height or simulation change is introduced.

This changes the runtime material-normal byte contract. Normal-bearing decoded
pages must use signed bytes; old unsigned payloads must be regenerated or
re-encoded, not silently mixed. Normal-less pages, bundled albedo/roughness,
legacy RGBA normal exports and Bevy's albedo-only path remain compatible.
The core exposes the shared encoder/decoder so producers and external decoded
page callers do not duplicate a format formula.

The zero-slope GPU oracle compares an absent normal plane against a present
neutral plane with actual regular/conforming terrain, all three surface tiers
and four view cosines (0.001/0.01/0.1/0.7). Conforming geometry settles before
each A/B/A sequence, and exact A/A equality guards against geometry drift. The
old encoding fails all 24 cases, with maximum error 32 linear output codes;
signed encoding passes the same exact full-frame equality requirement. An
initial six-warmup fixture was unstable in the most grazing conforming case;
its 50-code result is invalid, superseded by the stable v2 before/v3 after logs.
These logs are retained under `target/surface-showcase/normal-neutral/`.

A 200,001-point CPU sweep pins zero, odd symmetry and maximum component error
0.5/127 plus f32 decoding roundoff, including the GPU's noncanonical -128 alias.
Existing slope-average, page-border, land-grain/ocean-flat and raster Lambert
tests use the signed contract, with the Lambert fixture additionally exercising
both positive and negative slopes. No component error bound is promoted to a
global grazing BRDF/image error bound. Microvisibility and nonlinear footprint
filtering remain separate unresolved contracts.

Matched standing/close/overview captures are complete against WORLD06j's
byte-identical WORLD06i executable, with unchanged canonical settings, frozen
epoch zero, 1280x720, 60 s prestream, 30 warmups and 32 measured frames. Camera,
resolved settings and plant counts agree; only the recorded one-time plant
preparation duration differs. Material-level histograms and drained streams
agree, but do not establish identical resident page IDs/content.

| Camera | Mean absolute displayed RGB code difference | GPU median before/after, ms |
| --- | --- | --- |
| Standing 2 m | 0.0262 / 0.0255 / 0.0251 | 103.327 / 106.614 |
| Close 50 m | 0.0490 / 0.0465 / 0.0418 | 110.958 / 109.264 |
| Overview 800 m | 0.0398 / 0.0398 / 0.0342 | 65.586 / 65.109 |

Implementer inspection confirms that the characteristic ground bands remain.
The canonical image deltas are small overall, not a bound on local grazing
response (the standing maximum is 17 displayed codes). Coarse/grid-like shadows,
trunk stripes, sparse cover and leaf aliasing remain. These single-order
whole-frame timings on Radeon 880M/RADV are observations, not a stable performance
comparison. The existing normal-map raster/upload diagnostic is rerun in the
hardware suite; its signed input fixture is not bit-identical to the earlier
unsigned one and must not be used to claim a format-conversion speedup.

Full local gates pass: 2,044 lib/bin tests, 105 integration tests, 86 renderer
hardware tests, one signed diagnostic-PNG preview test, formatting, architecture,
all-feature Clippy, native feature-off and release build. The mandated retired
Bevy command selects zero tests and is not hardware coverage. The final release
matches the `normal-native-v2` capture executable byte-for-byte; v1 captures
predate the last Rust API-documentation edits and are superseded. Final gates
include the diagnostic PNG consumer; the other 754 source/config/asset inputs
are unchanged from the initial snapshot. Existing README claims remain accurate.
No blind votes or final visual, temporal, stable-performance or planetary-streaming
acceptance is claimed.
Commands, hashes, numerical envelopes and capture evidence:
[`native-signed-material-normals-2026-10-23.json`](artifacts/native-signed-material-normals-2026-10-23.json).

## WORLD06l — Projected-geometry reference and canonical slope decomposition

This continuation retains **three CPU reference tests, not a renderer change**.
The source decomposition uses a temporary test-only hook in the actual shared
material producer, after material-strength scaling and after imported-height
gradient addition. It samples the 126x126 interior texels of 23 distinct pages
at L17–L22, selected along the final WORLD06k standing camera's 0/10/40/80 m
forward strip. Camera input, frozen erosion and source signature come from the
same canonical setup; duplicate selected pages are omitted.

Procedural grain RMS slopes are 0.095923–0.185048, compared with
0.000586–0.005933 for the weighted imported source (ratio 16.51–170.37).
Mean optical ground weights are 0.70487–0.70560. The legacy and imported sources
are currently additive; this measurement does not establish that one should
be deleted or that the terrain is a flat substrate under a separate cover.
Selected source pages are not actual resident GPU page IDs or per-pixel
footprints; there is no canonical outgoing-support or visibility proof here.
No source amplitude, normal strength, roughness, coverage or transport is changed.

The independent reference in
`crates/render-wgpu/tests/terrain_environment/normal_geometry.rs` uses a periodic
triangular height field, unit x period, slopes +/-s and y extrusion. Facets are
opaque Lambertian with reflectance 0.5. It derives exact visible intervals from
the next geometric peak, then verifies them against 12,800 explicit ray/segment
intersection cases. Projected visible facet area recovers the macro cosine to
an absolute 1e-14 bound. The BRDF integrates joint light/view-visible area and
both local cosines, normalized by both macro projected areas; swapping incoming
and outgoing directions must agree within 1e-12, and the flat limit must recover
0.5/pi within 1e-14. These are geometric/transport tests, not visual preference.

Sixteen incident-direction/slope fixtures integrate the single-bounce BRDF with
64x128 and 128x256 midpoint grids. Fine-grid reflectance is within [0,0.5] plus
1e-12 roundoff; maximum observed coarse/fine discrepancy is 0.000023180 against
a pinned absolute 0.0001 reflected/input-energy envelope. Multiple scattering is
deliberately absent from this reference, so lost inter-facet transport is not
silently reintroduced as a radiance multiplier. The reference is not GGX, the
actual canonical microgeometry, a production model or a normal-filtering bound.

Temporary producer instrumentation and its executable/source patch, the raw
JSON decomposition, an independent Python reference prototype and its results
are preserved under `target/surface-showcase/normal-source/`. Producer hooks are
removed. No new scene capture is needed or claimed: production rendering is
unchanged from WORLD06k. Full local gates pass on 756 unchanged inputs: 2,044
lib/bin, 108 integration and 86 renderer hardware tests, formatting, architecture,
all-feature Clippy, feature-off and release build. The mandated retired Bevy
command selects zero tests. Release SHA256 remains byte-identical to WORLD06k;
existing README claims remain accurate. No new visual, performance, temporal
or planetary-streaming acceptance is claimed. Evidence:
[`native-normal-geometry-reference-2026-10-23.json`](artifacts/native-normal-geometry-reference-2026-10-23.json).

## WORLD07 — Bounded near-eye understory and its measured limits

This stage extends the existing survey plant consumer instead of adding a new
renderer or vegetation simulation. `surface_plants.understory = true` derives
nine same-face L18 query windows from the actual near-eye survey direction and
samples a disjoint L24 identity lattice. The 73 m fine cover uses canonical
habitat, ground height, deterministic kind/scale/yaw and the shared prototype,
material, grouped-draw, opaque and shadow paths. It replaces coarse fern/reed
clumps inside that cover, retains every lycopsid tree and preserves the coarse
far fallback; `false` reproduces the WORLD04 representation exactly. The parent
Low/Medium/High density subset remains the only quality control.

The matched fixed-site standing pair (identical inputs except the boolean,
frozen time, 1280×720, 60 s prestream, 32 frames) retained 222 baseline plants
and 2,020 understory plants. The latter included 1,800 near clumps, three draws,
36,864 candidate visits, 5,881 surface samples and 113.061 ms CPU
preparation in the release capture. Median GPU time rose
105.470 → 147.359 ms; the combined source retained the existing ray-instance
preflight policy (`shared_policy_with_preflighted_instance_budget`). Displayed RGB mean
absolute difference is `[5.115, 4.067, 4.129]` codes, so the layer is visually
present; this is not a performance or aesthetic acceptance. The frame still
reads as sparse bare-ground savanna, not closed forest, and the 73 m boundary to
the coarse population remains visible in principle.

The layer is omitted above 4 m requested eye AGL, keeping the coarse survey at
50 m/overview poses rather than paying for an invisible near patch. Crossing an
L18 boundary regenerates the bounded batch synchronously and costs the measured
CPU interval; there is no background plant streamer, persistence, collision,
ecological succession or plant RT model. Pre-existing dark diagonal foliage
artifacts appear identically in both sides of the pair and are not attributed to
this change. No blind votes or final surface acceptance exist. Evidence:
[`artifacts/native-near-understory-2026-10-23.json`](artifacts/native-near-understory-2026-10-23.json).

## WORLD07a — Nonblocking same-site preparation and reusable GPU sources

The WORLD07 synchronous preparation measurement above remains a historical
baseline. Same-site L18 eye-window transitions now run `surface_plants::prepare`
through the existing `terrain_stream::JobHandle`/Rayon executor. At most one
worker/result is retained; camera requests coalesce and stale output (including
errors) cannot publish against a newer desired window. A lost/current worker or
current preparation error still fails explicitly. No plant-specific scheduler,
independent world sampler, new graphics switch or simulation clock is added.

Frames continue drawing the applied world-fixed descriptors until a matching
result arrives. Pending capture metadata distinguishes applied and requested
windows. Mode/site changes immediately select the compatible coarse/empty
representation; an existing worker may finish bounded work but cannot restore
plants in a map/unsupported view. Matching represented prototype kinds, retained
capacity and RT admission let publication reuse the shared renderer, materials,
buffers and shadow source; changed capacity/layout/admission still reconstructs
it. Instance matrices and shadow/RT invalidation stay in the shared updater.

Initial setup and selected-site changes still sample synchronously. This is a
bounded update-lifetime correction, not a general vegetation streamer, mesh LOD,
GPU-cost fix, ground-band correction or visual acceptance. The wider sequential
replacement and main-promotion obligations are recorded in
[doc 78](78_NATIVE_MAIN_INTEGRATION.md).

The 96-frame frozen-time standing orbit sweep (same site/settings, 60 s
prestream, 30 warmups, 1280×720) has three CPU submissions above 50 ms in each
baseline run, with maxima 158.038 / 236.733 ms. The final candidate has zero,
with maximum 27.685 ms. CPU medians are 4.132 / 5.316 ms baseline and 5.143 ms
candidate: this is removal of a scoped resampling stall, not a normal-frame
throughput gain. GPU medians are 197.513 / 207.352 ms baseline and 204.083 ms
candidate; GPU cost is not fixed. Four background requests were issued over the
whole candidate replay (including warmup), with no pending/stale job at the end.
The last source publication reused the renderer and took 0.008175 ms; this does
not include the normal shared per-frame instance matrix/upload work.

All runs end with the same applied L18 window and 1,994 submitted descriptors;
the hardware round-trip oracle compares descriptors/instances exactly against
synchronous preparation. Final-still RGB mean difference versus the repeated
baseline is below 0.004 displayed codes per channel (maximum 17), not bitwise
frame identity or independent aesthetic acceptance. All measured samples have
epoch zero and zero physics steps. The single final-candidate replay and two
baseline replays are diagnostics, not a universal latency bound. Capacity growth,
layout/admission changes, startup and selected-site preparation can still stall;
the source representation and GPU workload remain prototype quality. Evidence:
[`artifacts/native-async-understory-2026-10-23.json`](artifacts/native-async-understory-2026-10-23.json).

All 12 local command gates pass: formatting, architecture, all-feature Clippy,
2,050 lib/bin tests, 108 integration tests, 46 native ignored hardware/CLI
checks, 86 renderer hardware tests, feature-off, WORLD00 regression and
server/native builds. The mandated retired Bevy command selects zero tests.
All 756 compiled/configuration inputs stayed unchanged. The native-only release
recipe reproduces captured SHA256 `7945e0cd…` exactly. A joint native/server
release build also passes but unifies additional `syn`/`hashbrown` features and
produces a different viewer hash; it is not mislabeled as the capture binary.

## WORLD07b — Stellar ray-line preservation, not shadow removal

The next isolated visibility controls keep WORLD07a's site/camera, settings,
epoch, source and 96-frame sweep. RT-off and shadows-off remove the dark plant
marks; reflections-off does not. That alone is **not** proof of acne: opaque
forks and narrow leaves really cast shadows. A new analytic quad fixture instead
proves a specific defect: shifting a diagonal stellar ray 1 cm along the
receiver normal lets a disjoint 4 mm blade block it. Before the fix, the centre
pixel is black instead of the expected 157 RGB8 codes. Moving the blade onto the
true line must still block it.

The existing shared `surface_visibility` now keeps the original stellar line and
excludes receiver bias along it: `t_min = bias / (N.L)`. A nonpositive cosine or
exclusion beyond `t_max` skips the query. Primary/secondary mesh receivers share
the correction; raster/cloud visibility, reflection-ray offsets, materials,
placement, graphics tiers/defaults, AS lifecycle and simulation are unchanged.

The actual fern/reed/lycopsid producer is also checked against independent f64
triangle rays, two stellar directions, and default 1 cm / diagnostic 0.1 mm
biases. In 720 selected primary interiors per bias, the previous shift yields
38/2 visibility disagreements with the near-zero geometric reference, including
13/0 invented blockers. The corrected line has 13/1 disagreements, all skipped
near blockers and **zero** invented blockers. Shadow-edge hits are excluded
from RGB certification at an explicit 0.03 barycentric margin; this is not an
edge-coverage or universal numerical-error guarantee. The physical reference is
stable between 1e-7/1e-8 m offsets. The default bias is unchanged because these
local fixtures do not establish a smaller far-coordinate error bound.

Final matched still difference is mean absolute displayed RGB
`[0.381, 0.459, 0.197]`, maxima `[101,104,52]`. Twenty scene/source/plant metadata
keys match, and all measured frames have epoch zero and zero simulation steps.
CPU/GPU medians are 4.845/203.463 ms, versus 5.143/204.083 ms for WORLD07a:
these single runs show no claimed throughput improvement or acceptable FPS.
The main visible diagonal branch pattern and sparse/speckled foliage remain;
this is not a wholesale shadow-acne fix or final visual acceptance.

A separate 2560×1440 same-camera capture, downfiltered by averaging decoded
linear RGB in exact 2×2 blocks, is a diagnostic coverage reference, not a shipped
AA setting. It still shows sparse authored crowns. Resolution also changes
terrain/material demand, so its whole-frame difference is not isolated plant-AA
proof. Existing foil widths project below one pixel over much of the cover;
temporal/spatial filtering, hierarchy and canopy representation remain open.
The physical shadow correction must not be replaced with brighter leaves,
shadow disablement or artistic normal damping to conceal those gaps.

A blind pair is prepared under
`target/surface-showcase/foliage-controls/review/`, with its key outside that
directory. No independent vote has been recorded. Commands, settings and binary
hashes, geometric-reference scope, counterfactuals, crops and final comparison:
[`artifacts/native-stellar-ray-line-2026-10-23.json`](artifacts/native-stellar-ray-line-2026-10-23.json).

All 13 local command gates pass: formatting, architecture, workspace all-target/
all-feature Clippy, 2,051 lib/bin and 109 integration tests, 47 native ignored
hardware/CLI checks, 86 renderer hardware tests, native feature-off, WORLD00 and
separate server/viewer release builds. The legacy Bevy command still selects
zero tests. The plant oracle checks 1,392 RGB interiors after 48 shadow-edge
exclusions among 1,440 selected samples. All 757 compiled/configuration inputs
are unchanged; rebuilding the exact native-only recipe reproduces capture
SHA256 `4c89c555a1f847c8cce0eb16f0050318cb382e577e4a4a130472aac9310cd422`.
