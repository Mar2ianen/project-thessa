# 75 — Native surface prototype plants (bounded consumer and prototype geometry)

Current composition continuation (WORLD85, doc 77) uses 1.25 m authored
understory centre spacing. Existing candidates, canonical survivor transforms,
canopy and pebbles remain; no representation-only camera thinning is introduced.
The first standing stage retains 2,617 rather than 6,546 near clumps under the
same 37-range/7 MB limits. Geometry/material history below is retained.

Historical review (WORLD84, doc 77) retains the user's solitary umbrella,
dual-terminal crowns and 21-frond hero fern. Stable form identity now combines
with four organ-level pigment cohorts, shared padded leaf atlases and bounded
asymmetric pinna/rachis geometry. Full/reduced representations retain identical
cohort sequences and canonical instances. The source texture payload is
3,760,128 B; 37 aggregate/plant ranges and the 7 MB vertex/index/AO limit remain.
Current bark factors are form-dependent `[0.36,0.32,0.25]`,
`[0.38,0.34,0.27]`, `[0.34,0.31,0.24]`; living roughness is 0.60,
fern transmission 0.22, tree transmission 0.18/0.20/0.16 and reed 0.18.
The geometric visibility bake is followed by retained authored crown/base
attenuation; this is not computed ground bounce or measured contact visibility.
The shared image atlas adds no pass/draw and the disabled path remains empty.
Historical checkpoints below describe previous geometry/materials, not the
current calibration or reference acceptance. RT stability and independent visual
acceptance remain open; WORLD84 records an original-baseline AMD device loss.

**Status:** bounded native survey consumer and graphics policy are recorded on
`feat/native-surface-plants` (WORLD04). Grounded arching fern/reed/lycopsid
prototypes, per-kind albedo and a second matched capture/blind-review stage are
recorded on `feat/native-plant-prototypes` (WORLD05). This is neither planet-wide
vegetation streaming nor botanical realism, authoritative collision, or final
close-up visual acceptance. The current prototypes remain deliberately authored
meshes; WORLD06f added separate fern pinnae, forked crowns and smooth stem
normals. WORLD09 below replaces broad ribbons with hierarchical pinnate crowns;
neither stage is a botanical reconstruction.

WORLD23 (doc 77), extended by WORLD41, adds bounded near-eye moss shoots, fallen
leaf fragments and optional puddle meshes through this adapter's existing
preparation and publication. Medium/High admit 128/384 moss clumps, 512/2,048
leaf fragments and 8/24 sampled puddle patches; Low keeps optical cover only.
Moss shoots and fallen leaves use folded 5-vertex/4-triangle geometry with
area-weighted normals. Fragment corners sample the shared optical soil source;
dimensions/pigments are authored scenery, not simulated litter mass. The puddle
setting is default-off and parent-gated by ground detail. At most three aggregate
primitive ranges/instances are added, raising total admission to 37 ranges / 7 MB. These
local meshes carry actual geometry in source-reuse checks and append no fake
world scatter identities. Pool closure is sampled, not a hydrology/collision
claim; dynamic-window rebuild costs and exact CBT contact remain open.

WORLD15 ([doc 77](77_NATIVE_SURFACE_SHOWCASE.md)) supersedes the older prototype
counts below: stable identity selects three fern forms, three tree forms (broad
fork, single crown, narrow drooping fork), and one reed form. Six structural
root flares and optical poses follow the shared render-only soil source.
Full/reduced trunks use 16/8 sides and roots 8/4; all forms are checked against
projected-area and sampled surface-distance oracles. The native adapter keeps
shared rigid instancing, material boundaries, texture deduplication, visibility,
shadow and whole-partition RT preflight. At most 34 plant/pebble primitive ranges
and <=6 MB prototype vertex/index/AO payload are admitted. Pebbles now span
4–46 cm, retaining their source IDs and patch density; exact resident-triangle
contact is still not established. Medium/High use directional SH9 diffuse light,
and primary specular filtering consumes the existing normal-map mip mean.
Low and feature-off fallbacks remain explicit. Older WORLD13/14 counts and
4–30 cm dimensions below are historical snapshots, not the current limits.

WORLD22 reviews Gemini's WORLD16–21 calibration (doc 77). Current authored
fern/tree tint factors are [0.44, 0.48, 0.46] / [0.48, 0.52, 0.44], green-leaf
roughness factor 0.50 and diffuse transmission 0.18; stem/dry roughness remains
0.85. The showcase uses 48 m full-detail distance. Fuller crowns retain shared
organ/material boundaries. Spatial understory cohorts bias a bounded scale
fraction, respecting requested min/max and fixed-size policies. Canopy density
and the 20% pioneer floor are authored ecological priors, not solved sunlight.
The three pebble palettes use linear RGB [0.082, 0.076, 0.065],
[0.098, 0.088, 0.075], [0.115, 0.102, 0.085], roughness 0.65/0.70/0.75.
These factors combine with existing textures; they are not measured spectra.
The bark importer now reproduces all six maps byte-for-byte, including the
retained authored gain after intermediate sRGB8 quantization. Local weather
shares the environment publication with terrain/water; the mesh shader no
longer owns unconditional extinction. Earlier dated optical parameters below
are historical, not the current calibration.

WORLD13 extends this existing adapter with a bounded native ground-pebble
consumer, independently of the plant toggle. It reuses the same preparation
queue, source reuse, instancing, RT admission and shared shadow pass. The
pebbles are optical scenery without collision ownership; their existing
`terrain_ground_detail` quality policy and L24 identity extension are described
in docs 70/77. Plant telemetry still counts only plants; shared asset payload,
instance and draw counters include both kinds of surface props.

The surface-showcase continuation in [doc 77](77_NATIVE_SURFACE_SHOWCASE.md)
adds explicit lycopsid stem reflectance through the existing mesh material,
without changing placement. Its later WORLD06f stage changes silhouettes through
the same prototype producer and material path. Its WORLD07 continuation adds an
opt-in bounded near-eye fern/reed layer; it remains experimental, default-off,
and visibly a prototype rather than closed forest. WORLD07a moves same-site
near-window sampling to the existing shared worker executor; setup/site changes
and growth/layout-changing GPU publication remain synchronous. Compatible
completed batches reuse the existing prototype/material/shadow renderer and
instance allocation when they fit; instance matrices still use its normal
shared update path.
Botanical realism and final surface composition remain incomplete.

## WORLD11a — Slender ground-fern laminae (2026-10-24)

After `230d450`, ground pinnae use half-width `0.085 * pinna_span`, longitudinal
camber `0.07 * span` and midrib fold `0.015 * span`. The resulting geometric
length/width is about 6.1:1, rather than 3.6:1. These are authored optical asset
dimensions for the concept's slender fern silhouette, not measured fossil
anatomy. Frond/leaf counts, green/dry boundaries, identities, placement and
material maps remain those of WORLD11. No density, exposure or RGB compensation
is applied to restore the previous larger leaf area.

Full/reduced fern geometry still uses 3,224/1,040 triangles with the same vertex
payload. One-sided foliage area is now 3.196165/3.162293 m²; the full source was
5.408265 m² at WORLD11. The calibrated top-view area ratio prints 1.0000000 and
maximum sampled full-vertex distance to reduced triangles is 0.0098796 m.
Reed/tree geometry is unchanged. The first narrow-leaf experiment retained the
old larger camber and failed the existing face-normal envelope; it was rejected.
The final camber passes the unchanged >0.86 foliage-normal/face alignment bound.

The actual-producer ray fixture now marks near shadow-edge misses on the lit
side as well as hits on the dark side, using the existing 3% barycentric edge
band. Previously a clear f64 ray just outside a blocker could round onto the
edge in f32 and be incorrectly labeled an interior shading failure. Analytic
triangle tests pin that rounding case and reject distant misses, interior hits
and intersections before the exclusion distance. This changes test
classification, not production ray queries or shading, and does not fix or
certify the shadow edges themselves.

The two-RGB8-code interior tolerance is unchanged. Full/reduced fixtures inspect
7,736/2,680 samples, retaining 7,003/2,396 interior samples for the colour check;
a regression requires at least half the samples to remain interior. CPU
finite-exclusion leak counts are 42/0 and 3/0 at 0.01/0.0001 m, with zero
invented blockers. Different geometry and sample sets prevent treating those
counts as a visibility-accuracy improvement. Continuous Hausdorff distance,
all silhouettes, normal-map highlights and temporal stability remain unproven.

The latest scoped checks and actual-camera captures are in doc 77 and
[`artifacts/native-surface-fern-lamina-2026-10-24.json`](artifacts/native-surface-fern-lamina-2026-10-24.json).
WORLD11 below remains the preceding coverage checkpoint.

## WORLD11 — Bounded distant coverage and leaf representation (2026-10-24)

The current consumer queries a deduplicated 3×3 neighborhood of L14 windows
for large plants and, with near-eye understory enabled, 7×7 L18 windows for
canonical L24 clumps. Windows clamp/deduplicate at face edges; cross-face
continuation remains unimplemented. The same immutable scatter population,
32 m canonical ground, ecological/water/slope eligibility and nested density
subsets are retained. Coarse clumps are replaced only inside the near union;
all trees and coarse clumps outside it remain. Every request returns a complete
bounded batch or an error, never a truncated partially filled forest.

Full pinnae retain their eight triangles. The distant alternative uses a
four-corner two-triangle lamina with the same frond/leaf counts, curved midpoint
and projected outline area. Its width factor `(1 + sqrt(2))/2` integrates the
original polygon rather than compensating by a density multiplier. Reeds and
wood remain unchanged. Full/reduced foliage, stems and dry fronds are separate
primitive groups sharing the existing material images and opaque/shadow/RT
paths. Ground ferns now give the three longest, lowest outer fronds an explicit
dry material (authored sRGB 0.34/0.25/0.13, diffuse transmission 0.05),
while ten inner fronds retain green optics. Frond count, attachment geometry,
leaf area and triangle count are unchanged; emission order changes to keep the
material boundary contiguous. At most eleven primitives represent three kinds
and two leaf detail levels. Compatible GPU reuse also compares the representation layout.

The shared resolver defaults Low/Medium/High to 24/36/48 m full-detail distance
per unit instance scale. Optional `surface_plants.detail_distance_m` overrides
this distance (u32 metres); omission preserves tier defaults. The current review
profile explicitly selects 24 m at High. Near-eye contexts add the conservative L18 cell
half-diagonal, accounting for movement inside the current anchor cell; this
avoids coarse leaves at the eye without rebuilding every frame. The anchor
otherwise follows the survey window center. No new scheduler, pass, shader,
alpha card or color/fog concealment is added. Disabled remains zero work.

| Prototype | Full triangles | Reduced triangles | Sampled vertex displacement | Top-view area ratio |
| --- | ---: | ---: | ---: | ---: |
| Fern | 3,224 | 1,040 | 0.016040 m | 0.9999122 |
| Reed | 144 | 144 | 0 m | 1 |
| Tree | 15,360 | 4,992 | 0.039537 m | 1 |

The regression measures every full vertex against the reduced triangle BVH
and pins <0.057 m per unit scale and <2.5% top-view area difference. It does
not certify continuous Hausdorff distance, arbitrary projected silhouettes,
normal-map/specular error or transition preference. Representation changes
remain discrete at preparation boundaries. Canonical identity, transforms,
quality nesting, source ownership and disabled/budget/edge behavior have
separate tests. Actual-producer GPU ray fixtures exercise both representations
against independent f64 triangle visibility and retain the two-RGB8-code
interior-shading tolerance. Full/reduced fixtures inspect 7,780/2,716 samples;
finite exclusion leaks are 43/1 and 8/0 at 0.01/0.0001 m, with zero invented
blockers. The full fixture's change from WORLD10's 45/1 reflects sampling every
seventh triangle after the frond emission reorder, not improved geometric
visibility. These are separate sample sets, not a full/reduced visibility
accuracy ranking. Capture/performance evidence is recorded in doc 77.

Layered, deterministic world-grid management and batched rendering are also
described by [Whatley, GPU Gems 2 chapter 1](https://developer.nvidia.com/gpugems/gpugems2/part-i-geometric-complexity/chapter-1-toward-photorealism-virtual-botany).
This implementation extends Thessa's existing identity sampler and instancing;
it does not copy the chapter's billboard or dissolve implementation.

## WORLD10 — Review of the latest reference iterations (2026-10-24)

This dated checkpoint is superseded by WORLD11 above for coverage, material
grouping, source-sample calibration and final measurements.

This supersedes the dated WORLD09 dimensions and timings below. Gemini's
iterations through `d13364b` include concurrent renderer/geometry work; commit
subjects alone do not describe their complete diffs. The review retains its
organic ground material, explicit senescent tier and larger young-frond arch.
Pixel L1 and channel percentiles against an unmatched concept image are not
visual acceptance, and 450 ms is not an established project frame budget.

The fork now starts at 4.55 m, with asymmetric terminals separated by 3.7 m.
Its lower trunk ends at 4.77 m, so the branch split remains visible beneath the
crowns rather than disappearing inside them. The four frond tiers retain
ascending young, spreading mature, descending outer and hanging dry geometry.
Ground ferns have shorter, differently arched rachises. Pinna width follows the
local rachis tangent; its raised fold follows the actual lamina normal.
Each pinna uses an eight-edge curved outline, eight fan triangles and nine
shared vertices with area-weighted normals. Primary raster, shadows, prototype
visibility and optional RT consume the same indices; there is no alpha mask.

| Current prototype | Triangles | Vertices | One-sided foliage area |
| --- | ---: | ---: | ---: |
| Fern | 3,224 | 4,212 | 5.408 m² |
| Reed | 144 | 432 | 1.175 m² |
| Forked tree | 15,360 | 20,160 | 165.377 m² |
| Total | 18,728 | 24,804 | |

The indexed vertex/index/AO payload is 1,117,680 B, with the existing 32-byte
vertex, four-byte AO and 64-byte instance ABIs. A local cold prototype/bake
measurement was 607.569 ms; the asset and workload differ from WORLD09.
The actual-producer visibility calibration now pins 45/1 finite-exclusion
leaks at 0.01/0.0001 m, with zero invented blockers in the tested samples.
GPU interior shading remains within two RGB8 codes of the independent oracle;
this does not bound every plant, ray, normal-map terminator or shadow edge.

Understory occupancy is now [0, 1.85], with a narrower smooth transition over
noise [-0.2, 0.2]. True gaps alternate with colonies. Its symmetric ensemble
multiplier is 0.925, not 1; exact local counts are not preserved. Existing
water, slope, height and ecological eligibility remain authoritative. The
review profile selects High density, four-sample coverage and
PBR Neutral; TAA remains off. Defaults remain single-sample and default-off
plants/TAA. Five grouped primitives still share three prototype kinds.

The mesh environment fallback now uses the existing GGX response with
roughness-dependent sampling: material Low/Medium/High use 4/8/16 environment
samples and 1/2/4 cloud directions. Without a bound environment, the fallback
uses the existing ambient irradiance. Primary RT misses use that same response. Later-bounce
reflection footprints remain approximate. An all-double-sided pipeline
specialization removes an otherwise unnecessary fragment discard; mixed
materials retain the original backface rejection. On the isolated standing
baseline, this specialization reduced
measured GPU median 385.991 to 226.271 ms; three of 921,600 pixels differ
(maximum RGB8 difference 20), so pixel-exact equivalence is not established on the Radeon 880M/RADV. This is an isolated implementation diagnostic, not a speedup claim for the later, different scene.

Direct diffuse reflection and specular reflection now respect the actual
triangle hemisphere; a smoothed or normal-mapped normal cannot admit incident
light through the back of an opaque sheet. Thin diffuse transmission uses the
opposite geometric hemisphere. Shadow queries orient the exclusion plane
into the incident hemisphere for both primary and secondary hits. The actual
producer fixture exercises these previously leaking smooth-normal samples,
with independent f64 geometry and unchanged two-code RGB tolerance.
The same/opposite-side distinction follows the reflection/transmission
conventions in [PBRT](https://www.pbr-book.org/4ed/Reflection_Models/BSDF_Representation);
no PBRT implementation is copied.

Current measurements, executed checks, capture identities and limitations are
recorded in [doc 77](77_NATIVE_SURFACE_SHOWCASE.md). Sparse distant coverage,
identical archetypes, unresolved thin geometry and incomplete atmospheric
transport still prevent a reference-parity claim.

## WORLD09 — Textured, folded foliage (historical checkpoint, 2026-10-24)

This continues the uncommitted reference work and the Gemini commits through
`2b0ae6e` on `feat/surface-visual-engineering`. It keeps Gemini's tiered crown,
25-window understory, explicit daylight exposure, full atmosphere density,
near-shadow bias and admitted spectral-ocean partition. The two fork terminals
share 48 radial fronds: each has six ascending, nine spreading, six mature and
three dry hanging fronds. Real four-face pinnae replace flat strips at the same
leaf triangle budget. Width changes represent leaf surface area, not raster
coverage dilation. Two short branches add 112 triangles to the Gemini tree.
The ground fern has thirteen rachises with fourteen opposed pinna pairs.
These are authored optical archetypes, not botanical growth or fossil spectra.

| Kind | Triangles | One-sided foliage area |
| --- | ---: | ---: |
| Fern | 1,768 | 8.517 m² |
| Reed | 144 | 1.175 m² |
| Forked tree | 8,448 | 143.405 m² |
| Total | 10,360 | |

The 32,768-vertex validation ceiling bounds the largest prototype at 25,344
vertices and admits historical frozen meshes. Understory scale is 0.55–1.10.
The existing eight-metre world-addressed occupancy field and canonical ecological
eligibility are retained. Gemini expanded the same-face L18 cover from 3×3 to
5×5 windows, with 102,400 candidate/item ceilings. At the fixed standing site
this retains 5,245 unique instances, including 5,032 near clumps, after nine
coarse replacements. This larger cover exceeds the existing 4,096-instance RT
budget, so the whole shared plant RT partition is withheld; raster/shadows and
environment fallback remain. Increasing coverage is not a free performance win.

### Material and geometry ownership

WORLD84 adds optional `foliage_tones`: empty historical arrays select the mature
cohort; present byte arrays must match vertices, use 0–3 and be constant across
each triangle. Native UV conversion addresses the shared padded pigment atlas;
source UVs remain organ-local. This metadata changes neither rigid instances nor
material-group ownership. The earlier checkpoint below describes the previous
unatlased source and organ layout.

`PlantMesh.uvs` and `.ambient_visibility` are optional serde-default arrays;
empty historical arrays remain valid. Present arrays must match vertex count
and contain finite values; visibility must be in [0,1]. A new optional
`senescent` terminal vertex group supplies dry-leaf albedo and transmission.
Triangles cannot cross its validated boundary. The lowest six fronds own this
material explicitly. Generic mesh shading no longer guesses leaf age from local
height or alters unrelated meshes. Historical bakes without this group remain
single-foliage-material assets.

Bark uses one-metre repeating cylindrical UVs. A cropped real pinna texture's
long U axis now follows the leaf length, with a physical midrib fold rather than
an artistic normal rotation. Six bundled CC0 maps provide base colour, OpenGL
normals and packed roughness/metalness, sourced from Poly Haven `bark_willow` and
`fern_02`. See `assets/textures/plants/LICENSE.txt` and `provenance.json` for
creators, source/derived hashes, crop transform and scale. The offline importer
checks pinned input SHA256s. Leaf RGB is explicitly authored in linear space
(gains 2.6/2.5/1.8 before clamping), retaining Gemini's brighter archetype; it is
not a measured optical spectrum. This replaces an undocumented edited PNG whose
old provenance no longer matched. Runtime geometry owns the outline: there is
no alpha masking or alpha-to-coverage.

The producer bakes visibility against its own two-sided triangles with 32
cosine-weighted hemisphere rays per unique position/normal, reusing the existing
Parry 0.30.2 BVH dependency and a 20 µm numerical offset. The first-demand cache
cost 802.535 ms in a local diagnostic; 1,024 cached clones after 32 warmups had
median/p95 0.019186/0.020047 ms. This is asset work, not frame time. Visibility
covers only the prototype, not neighbours/terrain, and is reused on both sides;
there is no universal 32-sample convergence guarantee. Plane/cavity and an
independent square-aperture solid-angle integral constrain the tested cases.

Shared `Primitive.ambient_visibility` uses a separate float vertex stream, leaving
32-byte `Vertex` and 64-byte instance ABIs intact and UVs free for textures.
Absent arrays use one constant value. Raster ambient and secondary-hit ambient
consume the same source; direct illumination is not darkened by this AO.
Stem, green foliage and dry foliage use five grouped draws with shared tree
instance ranges and no duplicated descriptors. Vertex/index/AO payload is
1,243,200 B. Source RGBA textures occupy 3,219,456 B; full mip texels occupy
4,292,592 B, before GPU alignment/driver overhead. RT preflight counts each
primitive's repeated range.

The shared opaque material's [0,1] `diffuse_transmission` partitions total diffuse
scattering albedo A as R=A(1-t), T=At. Ferns now use t=0.34 (WORLD14; previously 0.32), other green foliage 0.45,
dry fronds 0.05 and wood/default materials zero. Raster and secondary RT shading
share front/back cosine terms and physical stellar-ray direction. Occluders
still cast opaque shadows; coloured transmitted shadows and subsurface multiple
scattering are not implemented.

### Filtering and temporal policy

Shared material uploads now build complete mip chains. Base RGB is filtered in
linear light, data/alpha linearly, including exact area weights for NPOT edges.
The normal sampled by shading is normalized; normal-variance/roughness filtering
is not implemented. RT texture fetches still use LOD zero without ray footprints.

The review profile uses single-sample raster and Khronos PBR Neutral display,
retaining TAA off. Four coverage samples remain an explicit optional setting. Renderer defaults remain single-sample and
Reinhard; unsupported/unverified MSAA resolves to single-sample. MSAA uses
hardware linear colour resolve and the nearest covered forward-depth sample
before existing single-sample consumers. It adds 48 B/pixel in HDR and one depth
resolve pass, 44,236,800 B at 1280×720. It is an expensive optional quality mode,
not a performance improvement or complete foliage LOD solution. See doc 59.

Immutable plants may explicitly opt into camera-only motion eligibility; moving
fleet geometry remains masked. Publication epochs reset history. Far background
seeds one fresh sample for bilinear footprints but never reprojects far-depth
history. General sampled-colour/depth TAA composition and moving-camera visual
acceptance remain open (doc 76).

The independent actual-producer ray fixture isolates AO/maps and checks two
stellar directions and 0.01/0.0001 m exclusion biases. Current geometry has
114/11 finite-bias disagreements with the converged near-zero physical reference,
all skipped nearby blockers and zero invented blockers. The GPU agrees within
two RGB8 codes outside the explicit shadow-edge exclusions. Those counts are
asset-specific calibration, not a universal bias guarantee or permission to
hide the remaining visibility error by brightening foliage.

Gemini's hard-coded 25 m fog-free zone, fixed extinction/sky tint, sinusoidal
puddles and ground-normal flattening are removed. Native finite-distance aerial
transport and source-owned wet litter remain work to implement through existing
atmosphere/material systems, not painted substitutes in generic shaders.
Ground bands, sparse distant coverage, aliasing, root/resident-mesh agreement and
GPU cost remain visible issues. No reference match or blind acceptance is claimed.

### Primary research used

- [Tiago Sousa / Crytek, GPU Gems 3 chapter 16](https://developer.nvidia.com/gpugems/gpugems3/part-iii-rendering/chapter-16-vegetation-procedural-animation-and-shading-crysis):
  foliage needs a combination of suitable shape, two-sided lighting and local
  ambient occlusion. This implementation uses actual pinna geometry and a BVH
  visibility bake; it does not copy the chapter's artistic lighting curves.
- [PBRT 4, Diffuse Reflection](https://pbr-book.org/4ed/Reflection_Models/Diffuse_Reflection)
  and [the official diffuse-transmission material contract](https://pbrt.org/fileformat-v4):
  hemisphere cosine weighting and normalized diffuse reflection/transmission
  motivate the passively bounded R/T partition; no PBRT source is copied.
- [Etienne Carrier / Ubisoft, Far Cry 5 world generation](https://blog.playstation.com/?p=199418):
  habitat and terrain should drive placement. Thessa retains its canonical
  climate/slope/water eligibility and world-addressed colonies instead of
  scattering another camera-local random population.

## WORLD08 — Broad leaves and spatial understory colonies (historical stage)

The current shared `offline::plant_prototypes` producer replaces each terminal's
56 narrow leaves with 24 broader, longer arching leaves. Its forked stems and
per-organ reflectances are unchanged. Fern rachises carry seven pairs of broader
pinnae with actual gaps. Reeds keep ten blades and the basal tussock, but their
narrower blades arch farther outward and droop; winding follows the upper side
continuously across the arch. Face normals are still derived from real triangles.
These are authored optical dimensions, not screen-space dilation, alpha masks,
new plant physics or taxonomic reconstructions.

| Kind | Old → new triangles | Old → new one-sided foliage area |
| --- | --- | --- |
| Fern clump | 1,716 → 1,404 | 0.929 → 1.710 m² |
| Reed clump | 144 → 144 | 1.890 → 1.175 m² |
| Lycopsid tree | 2,424 → 1,400 | 23.892 → 41.829 m² |
| Total | 4,284 → 2,948 | |

Native prototype vertex/index payload falls from 462,672 to 318,384 B; the
existing eight-byte organ palette, three grouped draws and 64-byte instance
ABI remain. Maximum horizontal reaches are bounded at 1.35/1.00/3.90 m for
fern/reed/tree, and the existing 8,192-vertex validation ceiling still admits
historical frozen prototypes. Historical bakes retain their stored geometry.
Larger leaf area can increase raster/shadow/RT coverage despite fewer triangles;
this is not a whole-renderer speedup claim.

Only the optional `Understory` population gains an eight-metre, world-addressed
colony-occupancy prior. It reuses `rng::value_noise3` over body-fixed metres,
smoothly modulating local reference density by [0.15, 1.85] before applying the
existing canonical ground/reed coverage and water/height/slope gates. This is
an authored spatial occupancy model, not a new humidity/hydrology field.
Symmetry preserves the ensemble mean, not an exact plant count in each window.
The capacity check includes the 1.85 maximum; gaps reject before the expensive
surface callback. Existing budgets still fail whole queries rather than clipping.
Candidate IDs, jitter, kind/scale/yaw, large plants and rocks retain their
contracts; understory membership may change. Every window and quality tier
samples the same field, preserving exact window unions and nested density.

The existing resolved plant/understory settings and disabled/Low/Medium/High
fallbacks own these changes; no additional pass, setting, shader or per-frame
colony evaluation is added. Disabled plants and disabled understory do not
evaluate the new field. The current opaque double-sided leaves have no alpha-mask path; their
remaining subpixel aliasing is not addressed by alpha-to-coverage. TAA still
rejects plant history. Lighting, terrain material transport, streamed root
agreement, canopy transmission, near/far continuity and final acceptance remain
open (docs 76/77/78).

WORLD08's 11 local command checks passed on 645 unchanged compiled/configuration
inputs: formatting, architecture, workspace Clippy with warnings denied,
2,082 workspace lib/bin tests, 117 integration tests, the actual-producer GPU
visibility oracle, coarse and understory GPU lifecycle/round-trip fixtures,
native feature-off, WORLD00 numeric regression and native-only release build.
The three hardware fixtures ran on Radeon 880M/Vulkan/RADV. They do not replace
the full renderer/native hardware suites or establish remote CI/platform parity.
The visibility fixture now samples 556 interiors per bias: GPU errors and false
blockers are zero; finite receiver exclusion still skips five genuine blockers
at 0.01 m bias and one at 0.0001 m. The previous producer's [13, 1] calibration
was updated to [5, 1] after its geometry changed, without weakening the per-pixel
colour/visibility oracle. Coverage, winding, physical extents, projected leaf
lamina/gaps, colony continuity/seams, exact query unions and nested density
remain tested separately from aesthetic preference.

Final evidence for source `1d6a759`: nine successful frozen-time captures
(three baseline, three geometry/colony, one moving and two same-binary near-cover
controls). Standing retains 2,111 plants, including 1,891 understory clumps,
versus 2,020/1,800 before; canonical large plants remain unchanged. Close and
overview retain 222 plants. All static captures end with zero pending jobs;
the orbit ends with four material jobs, so it is a runtime smoke test, not
streaming-convergence acceptance. Three eligible blind cases are prepared and
unreviewed; original-before versus WORLD08 standing is excluded because the
understory population intentionally changes. The additional explicit near-cover
standing control compares the same WORLD08 population. Exact commands, release,
settings, frame/log hashes, scope costs and limitations:
[`native-leaf-colonies-2026-10-23.json`](artifacts/native-leaf-colonies-2026-10-23.json).

## Requested and resolved contract

`[surface_plants]` in `graphics.toml` has `enabled = false`,
`quality = "high"` and `understory = false` by default. Missing/partial sections
retain these defaults. Quality uses the existing `Quality` enum; invalid values
fail TOML parsing. Unknown keys retain the shared parser's forward-compatible
behavior. `understory` requires the parent consumer to be enabled; it is
restart-required and does not add a new capability or renderer mode.

`RequestedGraphics.surface_plants: SurfacePlantsSettings` enters the existing
`ResolvedGraphicsSettings::from_requested` pipeline. It resolves flat fields:

| Field | Type | Disabled | Low | Medium | High |
| --- | --- | --- | --- | --- | --- |
| `surface_plants_enabled` | `bool` | false | true | true | true |
| `surface_plants_density_scale` | `f64` | 0 | 0.25 | 0.5 | 1 |
| `surface_plants_max_candidates` | `u64` | 0 | 36,864 | 36,864 | 36,864 |
| `surface_plants_max_items` | `u32` | 0 | 36,864 | 36,864 | 36,864 |
| `surface_plants_detail_distance_m` | `f64` | 0 | 24 | 36 | 48 |
| `surface_understory_enabled` | `bool` | false | opt-in | opt-in | opt-in |
| `surface_understory_max_candidates` | `u64` | 0 | 200,704 | 200,704 | 200,704 |
| `surface_understory_max_items` | `u32` | 0 | 200,704 | 200,704 | 200,704 |

Counts follow existing resolved count conventions. The consumer must perform a
checked conversion of `max_items` if its scatter API accepts `usize`. Density
remains `f64` for the scatter API. These fields are also the exact flat capture
metadata keys. Older resolved TOML without these fields deserializes as disabled
with zero density/budgets.

Low/Medium/High/Ultra presets set quality to Low/Medium/High/High, preserving
explicit enabled/disabled intent (as for ocean and topology). Custom leaves
plant policy untouched. Explicit quality overrides remain intact when parsing;
divergence from an applied preset reports custom. All three GUI descriptors
require restart for the retained survey representation.

## Bounded consumer boundary

The current native consumer is **survey-only**: at most nine neighboring
level-14 query windows around the numeric survey target. The level-20 identity
lattice visits at most `9 * 4^(20-14) = 36,864` candidate cells; the worst-case
output fits the matching item ceiling. All tiers traverse the same windows,
not successively larger regions.
Lower density is a nested rendering subset, not a promise of proportional CPU
sampling cost. IDs, habitat, final ground height, kind and scale must not be
derived from quality; reuse the existing scatter identity/eligibility contract
(doc 70) and canonical surface sampling rather than a parallel placement model.

The disabled consumer must return before any surface sampling, jobs, instance
buffers or mesh preparation. Resolved budgets are ceilings, not allocations;
enabling policy alone must not reserve the full item ceiling or create GPU resources.
No new device capability is required by the settings policy. It neither requests
RT nor changes shadow/global render architecture, terrain topology or physics.
The prototype's disabled/Low states are its explicit cheaper fallbacks.

## Native integration

`apps/terrain-native/src/surface_plants.rs` consumes the existing
`scatter_region` kernel with `canonical_plant_surface`, borrowing exactly
`Scene.stream.field()` (the existing source-checked attached `Arc<PlanetField>`).
It does not reload a frozen artifact, introduce a cache/provider, or derive
habitat from resident renderer LOD. The dominant-axis cube chart selects the
single L14 window containing the selected canonical bookmark/numeric target;
moving/zooming the camera within that selection does not regenerate it.
Selecting a different large-plant window synchronously replaces the batch once.
Within the same large-plant window, moving the enabled understory eye window
starts at most one `terrain_stream::JobHandle` on the existing Rayon executor.
Frames retain the applied world-fixed descriptors while it runs. Requests
coalesce to the latest camera window, with no growing queue; stale success/error
results are discarded. Map/unsupported/high-AGL transitions immediately release
or select the coarse representation, and an older worker cannot restore plants.
An already-running worker finishes bounded work even after a mode change; it
does not spawn replacement tasks while the view is withheld. This is not a
second scheduler, a plant residency cache, or planet-wide vegetation streaming.
Initial preparation and selected-site changes remain synchronous. Result
publication reuses the renderer only when represented prototype kinds match,
instances fit the retained capacity and RT-preflight admission is unchanged.
Otherwise it reconstructs the shared renderer with the new actual count; there
is no speculative maximum-budget allocation. Source publication is measured
separately from sampling and normal per-frame instance/camera uploads.

Survey-only `SceneMeshes` selects the plant source instead of a vehicle source.
It uses the same `MeshRenderer`, `update_selected` per-primitive ranges, opaque
pass, shared environment and shadow atlas. Descriptors retain exact world-local
`ScatterId`, kind, direction, signed ground height, scale and yaw alongside their
instances. Groups follow prototype kind order, IDs are sorted within each group,
and each primitive owns one contiguous instance range. Unknown prototype kinds,
scatter budget exhaustion, invalid meshes and renderer errors fail explicitly;
there is no truncation. The selected count, not the 4096 ceiling, determines the
rigid-instance buffer capacity. Zero accepted plants create no prototype source
or GPU mesh renderer. Initial map/vehicle/flight/server/docking views report
`unsupported_*` and perform no plant sampling or prototype preparation. Map
transitions withhold/release the survey representation and prepare again when
returning; this is not an instance cache. Existing vehicle/fleet paths retain
their own source lifetime/capacity rules.

Root position is exactly `(radius_m + ground_height_m) * direction` in body-fixed
f64, with no visual/contact height correction. Local Y is radial up. Zero yaw
uses the right-handed `(East, Up, -North)` basis from the canonical ENU helper;
positive yaw rotates around Up, so a quarter-turn maps local X to North.
The existing renderer subtracts the body-fixed camera in f64 before narrowing.
The producer's existing metre-local `PlantMesh` prototypes are exported directly
from `offline::plant_prototypes`, not regenerated by a new native algorithm.
Their f32 sRGB albedo is decoded continuously with the existing terrain IEC sRGB
convention into linear `Material.base_color`; metallic is zero, perceptual
roughness is 0.92 (the existing rough natural-surface convention), and triangle
sheet fronds/crowns use the existing double-sided material support. There is no
albedo/exposure compensation gain.

Optional generic mesh RT follows the existing `rt_vehicles` policy. Before
constructing the renderer, the consumer sums selected per-primitive range lengths
and compares the actual TLAS demand to `rt_max_instances`. If it cannot fit, the
whole plant source uses the existing raster/shadow/environment fallback with
`SurfaceSettings.ray_tracing = None`, reported as
`withheld_instance_budget_raster_fallback`. No raster descriptors are dropped,
no RT update is attempted after raster publication, and no global/vehicle policy
is changed. Feasible batches still pass through the existing shared device,
memory and terrain admission checks; preflight alone is not a claim of active RT.

### Representation limits

The survey prototypes are authored meshes, not botanical reconstructions or
photogrammetry. The current tree has two asymmetric forks and tiered pinnate
crowns; ground ferns have thirteen arching rachises, including three dry outer
fronds. Reeds retain ten arching tapered blades and a six-blade basal tussock.
Full/reduced geometry, explicit organ materials, baked self visibility and thin
diffuse scattering share the existing double-sided opaque mesh path (WORLD11
above). There is no wind, growth, botanical simulation or collision ownership.
Thin blades can still alias or disappear at a distance, and reduced geometry
does not preserve every view-dependent silhouette or normal-map highlight.
Stems have no terrain-conforming footprint. Canonical 32 m root height need
not equal the interpolated resident triangle, which may use another wavelength
or representation. The consumer applies no root-lowering offset to conceal it.

**Actual root-to-resident-triangle residual is unmeasured in this change.**
Page-height comparisons would not establish that residual; neither a clearance
bound nor a resident-mesh contact guarantee is claimed.

A separate independently reviewed hardware oracle now measures the existing
canonical descriptors against one **fixed generated regular L14 patch**, using
actual GPU vertex readback and f64 radial ray/triangle intersections. All 222
roots hit; signed residuals span -0.068938668 to +0.096391605 m, with nearest-rank
p50 -0.000603619 m and p95 +0.047214586 m. The observed grid-height quantization
error is only 0.000053186 m, not the allowed 0.5 m page budget. This is neither
the actual viewer's streamed resident cover nor a universal contact envelope;
the WORLD04 0.1 m fern prototype gap has been removed by WORLD05, but the
viewer's streamed cover and conforming topology remain unmeasured. Exact
ABI/topology, arithmetic checks, per-kind results and source/log hashes:
[fixed-patch oracle notes](artifacts/plant-ground-contact-oracle-notes.md).

### Diagnostics and cost boundary

Existing headless output and headless/window performance capture metadata expose
`native_surface_plants_*`: applied/disabled/withheld reason; face/level/x/y window;
candidate visits, actual surface callbacks, accepted and retained counts;
setup/worker preparation CPU milliseconds; prototype vertex/index payload bytes;
the selected count × existing 64-byte ABI instance payload bytes; submitted instances, draw calls
and frame upload bytes from the shared renderer. These are submitted workload,
not visibility counts. Preparation timing includes scatter/field sampling,
sorting and prototype/instance conversion, but excludes GPU renderer creation.
Payload/instance bytes are not total GPU allocation: shared uniforms, material,
environment, shadow and optional RT resources remain the existing renderer's
responsibility. The off/withheld counters and preparation time are exactly zero.
`preparation_job_pending`, `preparation_jobs_started` and
`preparation_jobs_discarded` report the bounded job lifecycle;
`publication_cpu_ms` measures the last source adoption/construction separately
from sampling and normal per-frame instance uploads. `renderer_reused` indicates
whether that publication kept the GPU source. The instance payload is not a
retained-capacity measurement: after reuse the buffer can be larger than the
selected count, but never grows without bounded source reconstruction.
Pending same-site metadata retains the actual applied counts/window
and reports `pending_understory_preparation_retaining_applied_window` plus the
requested window, rather than presenting the old descriptors as the new cover.
Diagnostics do not serialize per-instance IDs every frame; exact descriptors are
retained for CPU tests. Draw cost uses the existing render/shadow GPU intervals,
not a new isolated plant timing path.
`native_surface_plants_reduced_instances` counts unique descriptors represented
by reduced geometry, deduplicating repeated organ ranges.
`native_surface_plants_rt_policy` records the preflight/fallback decision; the
existing mesh RT statistics/reason remain the actual shared-backend evidence.

## Verification and integration gates

The settings tests cover defaults, partial sections, all tiers, disable,
serialization/legacy resolution, preset intent/overrides, GUI/capture metadata
and isolation from every unrelated resolved field. These are CPU policy tests,
not evidence of renderer correctness, GPU cost, botanical plausibility or a
performance win.

CPU tests cover disabled/withheld zero work/resources, nonempty WORLD00,
Low/Medium/High exact survivor descriptors, direct canonical height, prototype
validation and range ownership, yaw/poles/seam orientation, bounded error of
planetary f64 roots narrowed only after camera subtraction, explicit budget
errors, empty water representation, and selected-target versus camera/quality
ownership. A 300-instance mixed-kind fixture requires 450 primitive instances
(the fern has separate green/dry groups); it pins the 256-capacity fallback and
exact 450 boundary acceptance without dropping raster descriptors.
The additional ignored hardware test
`scene::fleet_mesh::tests::survey_plants_retain_body_fixed_descriptors_and_submit_grouped_ranges`
checks native mesh construction, submitted range/triangle counts, retention over
camera/time changes, selected-window replacement and map withholding. It was
**not run by this implementation owner** and is not a contact/visual oracle.

The parent ran that initial lifecycle hardware test successfully on Radeon 880M
/ RADV / Vulkan (`target/world04-evidence/parent-plant-mesh-hardware.log`, one
test, 3.63 s). That version covers map entry, not map return, and constructs
without a shadow atlas; it does not alone establish shared shadow-depth cache
invalidation. Enabled/quality controls require restart, so constructor-based
disabled checks must not be described as live-disable transitions.

The strengthened version also passed on the same adapter (one test, 3.31 s;
`parent-plant-atlas-roundtrip-hardware.log`): an actual shared atlas has active
depth passes, identical submissions reuse depth, map entry removes casters and
invalidates depth, map return restores exactly the same descriptors/instances,
and selected-window replacement invalidates once before returning to quiescence.
Its frozen atlas frame isolates cache lifecycle; this is not a pixel-occlusion
or shadow-placement oracle. CPU quality assertions additionally require strict
Low < Medium < High counts, not merely survivor inclusion.

### Isolated CPU evidence

Baseline `49c42dc`, branch `feat/native-surface-plants`, exclusive worktree
`/home/chechulin/Projects/project-thessa-native-plants`. No GPU jobs, commits,
pushes, image judging, whole-workspace gates or authoritative state edits were
performed here. Every Cargo invocation uses:

```bash
TMPDIR=/home/chechulin/Projects/project-thessa/target/tmp \
  CARGO_TARGET_DIR=/home/chechulin/Projects/project-thessa-native-plants/target \
  CARGO_BUILD_JOBS=3 cargo test -p thessa-terrain-native --bin thessa-terrain-native
```

Result: **117 passed, 0 failed, 32 ignored** (hardware/explicit diagnostic tests
remain for the parent). Log: `target/world04-evidence/native-cpu-tests.log`.
The focused `surface_plants::` invocation passed all seven adapter CPU tests;
the selected-target ownership test also passed in the complete native run.
Focused log: `target/world04-evidence/native-focused-cpu-tests.log`.
Scoped `cargo clippy -p thessa-terrain-native --all-targets --all-features -- -D warnings`
and `cargo check -p thessa-terrain-native --no-default-features` also passed with
the same environment (`native-clippy.log`, `native-no-hud-check.log` in that
directory). Changed Rust files passed `rustfmt --edition 2024 --check` and
`git diff --check` passed. No whole-workspace result is implied.
At WORLD00 `(1,L14,6018,11888)` with the unchanged canonical attachment:

| Quality | Candidate visits | Surface callbacks | Accepted/retained | Prototype vertex/index bytes | Instance buffer bytes |
| --- | --- | --- | --- | --- | --- |
| Low | 4096 | 65 | 52 | 15984 | 3328 |
| Medium | 4096 | 134 | 98 | 15984 | 6272 |
| High | 4096 | 298 | 222 | 15984 | 14208 |

Those WORLD04 rows predate the WORLD05 geometry pass (68,256 B). WORLD06f raises
the current prototype payload to 462,672 B at every quality; the retained counts,
rigid-instance bytes and per-frame upload bytes are unchanged (see below).

An initial unoptimized diagnostic observed preparation at approximately
1.77/3.05/6.41 ms respectively. These single CPU test observations are not a
stable startup/frame budget, isolated benchmark, speedup comparison or GPU
cost measurement. The retained telemetry measures each real preparation; GPU
creation cost is outside that timer. The entire disabled/withheld plant work
and allocation counters/time are pinned to zero, not timed as a benchmark.

The integration owner updates current README/status/feature indexes, executes
combined hardware gates and paired captures using the same executable/settings,
and records world signature/epoch/target and cost alongside the captures.
No GPU test or visual acceptance is self-certified by this CPU implementation.

## Parent captures and independent blind review

The parent captured eight actual silent 1280x720 frames: the previous executable's
disabled 50 m control; the new executable with plants off/High at WORLD00 overview
(distance 1800 m, eye AGL 800 m), close-up (80/50 m) and standing (80/2 m); and
Low at 80/50 m. All use frozen epoch/step 0, 60 s prestream, 30 warmup and 32
measured frames, with unchanged clouds, cloud shadows, atlas, materials and
reflection settings. Final survey camera metadata matches exactly for every pair.
All final height/material jobs and material misses are zero. Visible/wanted/height
cache counts match (372/372/496, 368/368/490, 371/371/494); resident-ID snapshots
were not captured. Enabling the source also admits the already-requested shared
mesh/terrain RT scene: total RT instances become 594/590/593, not just 222 plants.

High retains/submits 222 plants, three draws, 8,240 triangles and 14,784 bytes of
reported per-frame upload. Prototype payload is 15,984 bytes and the rigid-instance
buffer is 14,208 bytes. These small payloads are **not** total GPU memory: the
close-up's shared RT scene uses 33,533,808 bytes of its existing 32 MiB budget.
Low retains 52 plants with 3,328 instance-buffer bytes and 3,904 reported upload
bytes. Off reports zero plant samples, candidates, prototypes, instances,
preparation time and upload, with no survey mesh/RT scene. The previous/new
disabled close-up differs at eight pixels (maximum channel difference 20/255);
do not claim byte-identical output. Final camera and world counters are equal.

An independent reviewer saw only scrubbed, randomly shuffled A/B images; the
answer key stayed separate until all four votes were recorded. Results:

- Overview: **tie**, high confidence; tiny silhouettes do not materially improve
  the flat/blurred terrain.
- 50 m and 2 m: **High preferred to disabled**, medium confidence, for spatial
  depth/composition. Faceted umbrella crowns, straight/cropped trunks, dark canopy
  undersides, sparse ground detail and blocky shadows remain visible defects.
- Matched 50 m density variants: **High preferred to Low**, medium confidence,
  for that composition; not an ecological or object-count correctness claim.

This is one reviewer and one shuffle (all High frames happened to be A), not a
statistical aesthetic ranking. Preference does not certify root contact, shadow
placement, temporal/LOD stability, botanical realism or complete surface
acceptance. The geometry remains an explicitly schematic prototype.

### Observed cost, not performance acceptance

| View | Off GPU median ms | High GPU median ms |
| --- | --- | --- |
| Overview | 76.301 | 76.699 |
| 50 m | 82.274 | 96.187 |
| 2 m | 79.025 | 93.200 |

Low at 50 m measured 90.365 ms. These are whole-client GPU timestamps, not
isolated foliage cost or game FPS. The close-up increases must not be hidden:
geometry, shared RT membership and shading are added. Serial single-process
captures with one order, uncontrolled thermals/machine state and no
counterbalanced repeats establish neither a stable regression magnitude nor a
speedup or accepted frame budget. High startup preparation measured
8.106/8.122/5.405 ms; Low 1.324 ms. These include CPU descriptor/prototype
preparation but exclude GPU renderer/AS creation. No allocation/RSS/VRAM
high-water or whole-platform acceptance is claimed.

Exact binary/image/configuration/source hashes, commands, votes and limitations
are recorded in
[`artifacts/native-surface-plants-2026-10-21.json`](artifacts/native-surface-plants-2026-10-21.json).

## WORLD05 prototype geometry

The geometry pass edits only the `offline::plant_prototypes` producers and the
consumer's `cfg(test)` expectations. Placement, world-local IDs, canonical root
height, scale, yaw, fixed L14 window, quality tiers, material/shadow/RT path,
grouped ranges and the no-collision boundary are unchanged. No vertical offset or
root lowering was introduced; fern fronds simply begin at local Y = 0.

| Kind | Triangles (old → new) | Vertices | Max local Y m | Reach m |
| --- | --- | --- | --- | --- |
| Fern clump | 64 → 156 | 468 | 0.665 | 1.14 |
| Reed clump | 60 → 144 | 432 | 2.00 | 0.53 |
| Lycopsid | 24 → 332 | 996 | 9.25 | 2.21 |
| **All three** | **148 → 632** | **1,896** | | |

Prototype payload grows from 15,984 to 68,256 bytes (+52,272 B, 4.27×); the
selected-instance buffer stays 14,208 B and reported per-frame upload stays
14,784 B because only instances, not prototype meshes, are uploaded per frame.
The prototypes use distinct sRGB albedos now — fern `[0.18, 0.38, 0.10]`, reed
`[0.33, 0.35, 0.11]`, lycopsid `[0.17, 0.28, 0.10]` — still decoded by the existing
linear material path. Ferns use 13 arching fronds with a serrated width profile;
reeds use ten tapered near-vertical blades plus a six-blade tussock; the lycopsid
uses a ten-sided tapered trunk with a stepped base, four stub forks and three
whorls of broad arching fronds. Flat per-triangle normals are kept; winding hints
force foliage normals upward/outward. Tests pin extents, root height, minimum
triangle area, winding/normal bounds, triangle ceilings, payload bytes and
determinism. They also assert that arching crown triangles may tilt inward near
the trunk but never have an upper face pointing downward.

### WORLD05 matched captures

Three matched old/new pairs use the same frozen scene and settings as WORLD04:
overview at 1800/800 m, close at 80/50 m and standing at 80/2 m, 1280x720,
epoch/step 0, 60 s prestream, 30 warmup frames and 32 measured frames. Old
executable `b5f33c40…` has the WORLD04 prototypes; new executable `6fe61390…`
has WORLD05 geometry; both run the same committed settings. Camera arguments,
world target and time are identical. Final height/material jobs and material
misses are zero. Cover counts are 496/579/372 (overview), 490/580/368 (close) and
494/581/371 (standing). Total shared RT instances remain 594/590/593; enabling
plants admits the already-requested mesh/terrain RT scene, and the close-up RT
scene is 33,538,848 B. Per-frame instance upload for High remains 14,784 B.

The old executable was re-captured twice at the same close/standing cameras: the
two runs differ at only 5 pixels (maximum channel difference 35/255), while
old-vs-new differs at 177,362 pixels (close, max 134/255), 114,027 pixels
(standing, max 143/255) and 3,797 pixels (overview, max 101/255). The pairs are
therefore materially different, not renderer noise. Scene lighting/cloud state
still carries small nondeterministic variation, so this is a visual comparison,
not a pixel-identity or contact test.

The independent reviewer saw only shuffled A/B images and froze three votes
before the key was read. Decoded result: **arching prototypes preferred over the
previous umbrellas at 50 m (medium confidence) and 2 m (high confidence);
overview tie**. Preference points were 2.5 vs 0.5. The reviewer described the new
50 m form as recognizable frond crowns with tapered trunks and readable ferns,
and the old form as faceted cone-on-pole placeholders. At 2 m the new root flare
and drooping crown read as a coherent plant silhouette. At overview distance
both collapse to tiny silhouettes. Remaining new-geometry defects: thin-frond
aliasing/dark speckle, dark crown undersides, a visibly faceted ten-sided trunk
at 2 m and near-vertical fork stubs. The old geometry's hard facets, laminar cone
overlaps and blob shadows are not retained. This is one reviewer and one shuffle,
not statistical or ecological acceptance.

### WORLD05 observed cost

| View | Old High GPU median ms | New High GPU median ms |
| --- | --- | --- |
| Overview | 77.456 | 80.435 |
| 50 m | 104.016 | 94.837 |
| 2 m | 94.563 | 97.564 |

New Low at 50 m measured 83.927 ms and off 82.152 ms. These are whole-client GPU
timestamp intervals, not isolated foliage cost or game FPS. Runs are serial,
single-order and not counterbalanced; view, capture order and thermal state are
not separated (the old executable's own 50 m and 2 m medians differ by 9.5 ms).
The lower new 50 m median therefore is **not** claimed as a speedup, and the
higher overview and 2 m medians are not a stable regression magnitude. Geometry cost is visible
in prototype payload/triangle counts; CPU preparation measured 5.55/8.49 ms
(new) versus 5.36/8.11 ms (old) at close/standing in these single captures.
No allocation/RSS/VRAM high-water, frame-budget or platform acceptance is claimed.

Exact hashes, commands, camera arguments, votes and limitations are recorded in
[`artifacts/native-plant-prototypes-2026-10-21.json`](artifacts/native-plant-prototypes-2026-10-21.json).

## WORLD06f — Pinnate ferns and forked crowns

This continuation changes only the shared `offline::plant_prototypes` source,
its bounded validation/tests and a CPU benchmark. The native adapter still
forwards positions, normals and indices unchanged; a regression assertion now
checks the normals explicitly. Historical frozen bakes are not rewritten and
retain their stored prototypes. There is no new setting, material/texture,
render pass or camera-specific asset: the existing requested/resolved
`surface_plants` disabled/Low/Medium/High density policy owns this representation.
Disabled and withheld contexts still return before any prototype production.

Ferns have 13 arching rachises, each with nine opposed pairs of narrow pinnae.
The spaces between pinnae are actual geometric gaps, not painted black or alpha
discard, and enter existing raster, shadows and optional RT identically. The
lycopsid proxy branches twice into four terminals, each with 56 individually
arching leaves on three short internodes. These dimensions are authored optical
geometry, not a plant growth model or a taxonomic reconstruction. Stem branches
intersect rather than forming a watertight boolean union; terminal caps close
their visible ends. No physics/collision/biomass state is implied.

The main stem uses 16 sides, forks 12 and terminal shoots 8. Analytic cone-side
normals smooth only the circumference; ring slope changes keep their creases
and terminal caps keep separate planar normals. The existing shared mesh/RT
normal path consumes them, not a foliage-only lighting multiplier.

| Kind | Previous → new triangles | New split vertices | Native vertex/index payload B |
| --- | --- | --- | --- |
| Fern clump | 156 → 1,716 | 5,148 | 185,328 |
| Reed clump | 144 → 144 | 432 | 15,552 |
| Lycopsid | 332 → 2,424 | 7,272 | 261,792 |
| **All three** | **632 → 4,284** | **12,852** | **462,672** |

The source validator's per-prototype ceiling rises from 4,096 to 8,192 vertices
to accommodate real pinnae. A regression rejects over-budget data. This is not
a larger scatter window or instance ceiling. Producer position/normal/index
payload grows from 53,088 to 359,856 B; native payload includes the existing UV
fields. Materials, the 8-byte stem/foliage palette, grouped draw count and
64-byte rigid-instance ABI do not change. More submitted triangles, shared
shadow work and RT geometry are a real cost, not a performance win.

Tests retain root/extents/determinism, organ ownership, nondegenerate triangles
and upward/outward winding. The tree's authored one-sided foliage area is pinned
between 20 and 30 m²; its maximum horizontal reach is bounded at 3.4 m (the old
2.6 m bound no longer describes its larger crown). The area floor is 1e-6 m² for new pinnae/twig caps
and the unchanged 1e-4 m² for reeds. Leaf normals must still match their winding;
stem normals must agree with outward face normals (dot > 0.90). A cone fixture
pins analytic normals within 2e-7 Euclidean vector error and chord inset below
8 mm for its 0.4 m base/16 sides. Rigidly rotated/translated copies agree within
1 micrometre in position and 3e-6 in unit-normal vector. These are optical fixture
envelopes, not contact or canopy-lighting guarantees. An independent projected
triangle test hits a fern pinna and rachis but misses the adjacent air gap.

The initial 28-leaf terminal experiment had only 3,388 total prototype triangles.
Its 0.58–0.88 m leaves disappeared into dots at 50 m: more triangle detail alone
did not provide a readable crown. Those exploratory frames remain under
`target/surface-showcase/botanical-geometry/after-{standing2,close50}.png`; this
version was not retained. The replacement uses 1.05–1.45 m leaves with more real
surface area, not an alpha dilation or lighting multiplier. The serial
`scatter --prototypes` benchmark measures 1,024 complete three-kind builds after
32 warmups; it is not scatter, upload, RT build, frame time or stable performance
acceptance. No claim of botanical/visual acceptance follows from tests or counts.

The final producer benchmark measured 0.019447/0.168944 ms median and
0.021350/0.187249 ms p95 before/after. Final serial full-scene GPU medians were
99.248/111.610 ms standing, 95.591/124.532 ms close and 76.509/78.096 ms overview.
These increases are not performance acceptance; the more detailed source needs
distance-aware representation work. Low close retained 52 descriptors and
measured 86.656 ms. Static High retains 222 descriptors, three draws and 14,784
frame-upload bytes, with no pending height/material jobs or material misses.
The 64-frame orbit sweep retained the same population but ended with four height
jobs: one endpoint and telemetry do not establish motion/temporal acceptance.

Formatting, architecture, workspace Clippy, 2,038 lib/bin tests, 103 integration
tests, 82 renderer hardware tests, the focused native plant lifecycle and fixed
L14 contact diagnostics, feature-off check and release build passed locally.
The mandated retired Bevy command selected zero tests. Three blind pairs are
prepared with separate keys and zero votes. Commands, hashes, negative attempts
and limits are in
[`native-botanical-geometry-2026-10-23.json`](artifacts/native-botanical-geometry-2026-10-23.json).

### WORLD05 combined local verification

Fourteen local command gates passed on committed `HEAD 132b709`: formatting,
whole-workspace all-target/all-feature clippy with `-D warnings`, architecture
check, 2,027 workspace lib/bin tests (45 ignored), 102 integration tests (85
ignored and executed by the hardware suites), feature-off native check, the two
focused plant GPU lifecycle/root oracles, the WORLD00 release regression, 79
renderer hardware tests, server build, 45 native hardware/CLI tests, the legacy
Bevy command and a release build. The 755 tracked test-input manifests were
byte-identical before and after the run (`input-changes.json` is empty). The
release rebuild reproduced the captured executable byte-for-byte
(`6fe61390192971187f82d98c29ecfb77842f363eadce2f5d036f9a764d8ddf9c`), so the
captures correspond to the committed producer rather than only to a working tree.

The legacy `cargo test -p thessa-bevy-rcbt --features render --lib -- --ignored`
gate exits successfully but currently matches **zero** tests: that crate has no
`#[ignore]` tests after the Bevy retirement. It must not be cited as GPU
coverage; the actual hardware coverage above comes from `thessa-render-wgpu` and
`thessa-terrain-native`. These are local results, not remote CI. The blind review
and cost rows above remain subjective/single-order evidence and are not replaced
by the gate count.

### WORLD04 combined local verification

Fourteen local command gates passed on committed `HEAD 71f43ed`: formatting,
whole-workspace all-target/all-feature clippy with `-D warnings`, architecture
check, 2,024 workspace lib/bin tests (45 ignored), 102 integration tests (85
ignored and executed by the hardware suites), feature-off native check, the two
focused plant GPU lifecycle/root oracles, the WORLD00 release regression, 79
renderer hardware tests, server build, 45 native hardware/CLI tests, the legacy
Bevy command and a release build. The 755 tracked test-input manifests were
byte-identical before and after the run (`input-changes.json` is empty). The
release rebuild reproduced the captured executable byte-for-byte
(`b5f33c40f647f12e33c7aecb18115d72f6f05f583311b13a625814690660aed2`), so the
captures correspond to the committed producer rather than only to a working tree.

The legacy `cargo test -p thessa-bevy-rcbt --features render --lib -- --ignored`
gate exits successfully but currently matches **zero** tests: that crate has no
`#[ignore]` tests after the Bevy retirement. It must not be cited as GPU
coverage; the actual hardware coverage above comes from `thessa-render-wgpu` and
`thessa-terrain-native`. These are local results, not remote CI.

## WORLD07 — Bounded near-eye understory

`surface_plants.understory = true` adds a second, opt-in descriptor population
without replacing the WORLD04 consumer. The large-plant survey remains the fixed
L14 window around the selected canonical target. The near layer uses the actual
survey eye direction and nine same-face level-18 query windows (a 3×3 window
around the containing cell); each L18 window owns 4,096 level-24 candidate cells,
so the resolved candidate ceiling is 36,864. Face-edge windows are clamped and
deduplicated. The eye direction is the renderer's body-fixed survey offset, not a
new camera-height or terrain query; radial ground/AGL clamping does not change it.

The L24 lattice is a separate placement contract: understory IDs use the existing
fixed64 node address plus population tag 1, and `ScatterId::cell` disambiguates
them from level-20 plants by node depth. Existing rock/plant IDs are unchanged.
Habitat is the canonical `canonical_plant_surface` ground/reeds coverage, with the
same water, minimum-height and slope rejection as the parent plants. Fern versus
reed choice is deterministic from the world seed and ID. `understory_spacing_m`
defaults to an authored 0.8 m reference centre spacing and clump scales to
0.35–0.9; these are plant-density/size priors, not a meter-scale canopy
simulation. Requested density is compared with the L24 cell capacity and fails
explicitly on saturation rather than clipping.

The layer reuses the existing prototype producer, per-kind material, grouped
draw ranges, opaque pass, environment and shadow atlas. Inside the nine-window
fine cover it replaces coarse fern/reed clumps; it retains every lycopsid tree
and the coarse fern/reed fallback outside that cover. Thus default-off preserves
every WORLD04 descriptor and ID exactly, while enabling the layer does not
duplicate a coarse and a fine clump at the same place. Descriptor order is
restored by sorted ID; prototype draw grouping is unchanged.

Both candidate and item budgets are cumulative and fail the whole preparation
before returning a partial representation. The near layer uses the parent
Low/Medium/High density subset (`.25/.5/1`) and does not introduce another
quality setting or hidden toggle. A camera move into a different L18 window,
or an explicit eye AGL above 4 m (which selects the coarse-only survey), changes
the bounded representation synchronously, retaining the existing "prepare once
per selected window" lifecycle; this is not background streaming. An enabled
understory with no eligible eye pose is a valid coarse-only fallback, not an
error, and never retains a stale near layer.
No collision, persistence, planetary vegetation LOD, biome simulation or plant
RT representation is added. The existing `rt_vehicles` preflight and raster
fallback still apply to the combined planted source.

Linux hardware coverage is the ignored
`near_understory_reuses_the_shared_plant_and_shadow_lifecycle` fixture, which
checks grouped draw/shadow reuse, exact descriptor and instance restoration after
a window round trip, and map-view release. CPU oracles pin disjoint IDs, nested
density, exact parent/child window unions, ground/reed habitat, explicit spacing
validation, budget errors and parent-gated settings isolation. These establish
implementation contracts, not ecological realism or visual acceptance.

### WORLD07 matched standing result

The final candidate executable
`3dc8490b7297ce9619374413a20cbcf55ab277dbf9d150858297f90f3031e969`
was compared with identical `graphics.toml` inputs except
`surface_plants.understory` at the fixed reference site, standing eye AGL 2 m,
80 m survey distance, frozen simulation time/step, 60 s prestream, 30 warmups and
32 measured 1280×720 frames. Baseline retained 222 plants (three draws,
14,208 instance bytes). Understory retained 2,020: 1,800 near clumps plus 220
coarse survivors, after replacing two coarse clumps; it submitted three draws,
129,280 instance bytes, 36,864 candidate visits and 5,881 surface samples.
Preparation was 5.768 ms baseline and 113.061 ms understory on this CPU.
Median GPU frame time was 105.470 ms baseline and 147.359 ms understory; the
combined source kept the existing ray-scene preflight policy
(`shared_policy_with_preflighted_instance_budget`). These are single local
diagnostics, not stable-performance acceptance.

Displayed RGB difference versus baseline has mean absolute
`[5.115, 4.067, 4.129]` codes (max `[137,116,117]`), so the layer is visibly
present rather than a sub-code no-op. The eye-level frame shows plausible
knee-to-waist fern/reed clumps, but the site remains a sparse, bare-ground
savanna; there is no grass mat, canopy closure, botanical realism or final
aesthetic acceptance. The 73 m fine cover and the coarse far population still
meet at a representational boundary. Repeated synchronous regeneration costs
113 ms whenever the eye crosses an L18 window, so this is an experimental
default-off layer, not a shipped streaming solution.

At 50 m eye AGL the layer is intentionally omitted and the coarse survey is
retained; the standing captures remain the only visual A/B evidence. The
pre-existing dark diagonal foliage/occlusion artifacts visible in both sides of
the matched pair are not introduced by this layer. Evidence:
[`artifacts/native-near-understory-2026-10-23.json`](artifacts/native-near-understory-2026-10-23.json).

### WORLD07a follow-up

Same-site resampling is no longer a foreground wait: the current integration
contract above uses one shared worker, stale-result rejection and compatible GPU
source reuse. The matched 96-frame orbit diagnostic has no CPU submission above
50 ms (maximum 27.685 ms), versus three in each baseline (158.038 / 236.733 ms
maxima). Last compatible source adoption was 0.008175 ms, excluding the normal
shared instance upload; total GPU median remains about 204 ms. The queue oracle,
direct synchronous-descriptor comparison, exact round trip, shadow invalidation,
map release/stale rejection and successful source reuse run on the real adapter.
This closes the tested foreground sampling wait, not all stutter, GPU cost,
near/far transitions or visual acceptance. Full scope and immutable inputs:
[`artifacts/native-async-understory-2026-10-23.json`](artifacts/native-async-understory-2026-10-23.json).

### WORLD07b — Keep stellar visibility on the physical ray

Counterfactual captures with RT, shadows or reflections disabled isolate the
dark plant marks to stellar scene visibility, not the reflection lobe. This
does not make every dark branch mark an artifact: opaque leaves and forks
really occlude each other. A separate analytic receiver/blade fixture exposes
a narrower defect: offsetting the origin along the receiver normal can move
the stellar ray sideways into a disjoint 4 mm blade. The original renderer
returns black where the physical ray must be clear.

The shared primary/secondary `surface_visibility` hook now traces from the
receiver on the unchanged stellar line and excludes its normal-distance bias
with `t_min = ray_bias_m / (N.L)`. Nonpositive `N.L` retains the existing
no-query path; exclusion beyond the configured maximum distance also avoids
the query. A blade on the true ray still blocks it. Raster atlas/cloud visibility,
reflection origins, quality tiers, defaults and AS lifetime are unchanged.
No artistic attenuation, altered plant geometry or extra pass is introduced.

The independent f64 oracle uses the actual fern/reed/lycopsid triangle producer,
uniform grey material, two stellar directions and biases 0.01/0.0001 m. It
compares interior primary pixels to analytic direct shading and triangle
visibility; shadow hits within 0.03 barycentric distance of an edge are counted
separately, not certified for raster/intersection edge precision. The physical
comparison converges between 1e-7/1e-8 m reference offsets. At the default bias,
the previous origin shift created 13 false blockers in 720 selected samples;
the new ray has none. Thirteen genuine blockers are still skipped by finite
receiver exclusion, versus one at 0.0001 m. The default is **not** silently
lowered: neither fixture establishes a universal far-coordinate bias bound.
This fixes a causal shadow defect, not foliage coverage/AA, all close-contact
visibility, ground bands, canopy closure or final visual acceptance.

## WORLD13 near-ground pebbles (2026-10-24)

With resolved ground detail, Medium/High surface quality admits three variants
of a 32-triangle/18-vertex closed pebble through this adapter, including CBT.
The source adds 3,096 vertex/index/AO bytes, three opaque rough mineral materials,
different proportions and no images. Nominal instance diameters range from 4 to 30 cm; the base extends
below its canonical ground anchor. Area-weighted geometric normals and the
shared shadow atlas supply shape/lighting; no painted contact AO is added.
The existing all-primitive RT preflight includes pebble instances and retains
whole-partition raster fallback on exhaustion. Plant identity/geometry is
unchanged; at most three additional primitive ranges are published (14 total
with all previous full/reduced plant organs). Variant identity participates in
compatible-source reuse. Placement uses the terrain's 32 m-filtered base plus
world-fixed, 12 mm-footprint render relief (doc 77); exact resident-triangle
contact is not claimed. An independently seeded 8 m patch field breaks uniform
pebble density without copying understory colonies.

The actual eye's L18 window plus eight same-face neighbours bounds the query.
It is prepared only for explicit eye AGL at most 4 m in supported survey views.
Higher/unspecified eyes use the existing optical ground fallback. Medium keeps
half of the High immutable population; Low has no pebble jobs/meshes. Both
candidate and item limits are 36,864 per prepared union, not allocation targets.
The existing shared job coalesces moving windows and discards stale results.
This is not planetary streaming; boundary transitions remain discrete and
same-face. Default-off skips sampling/publication/resources. Ground-only views
can create the shared mesh consumer without enabling plants.

`native_surface_ground_pebbles`, `native_surface_ground_candidates`,
`native_surface_ground_samples` and `native_surface_ground_window` identify the
prepared ground representation. Resolved density/candidate/item limits are
exported alongside them. The legacy `native_surface_plants_retained` remains a
plant-only count; shared mesh bytes/submitted instances/draws include pebbles.
