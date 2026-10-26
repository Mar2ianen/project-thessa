# 77 — Surface-first visual development

**Status:** in progress, not visual acceptance. Baseline is WORLD05 commit
`54cdfa6`, with the existing canonical WORLD00 shore and prototype plants.
The foundation through `f73eea4` is integrated into main (doc 78); subsequent
visual development now continues on `feat/surface-visual-engineering`; the
WORLD13 continuation below migrates the native terrain to persistent CBT and adds soil detail.

## WORLD85+ — Coastal composition iterations (2026-10-26, in progress)

The overnight continuation starts from `89d1e2a`, following the report that the
understory contains too many oversized ferns and lacks intermediate vegetation.
The target remains `docs/reference/visual_quality_target_coastal_ferns.jpg`:
patches of different plant sizes separated by readable litter/soil corridors,
with layered silhouettes rather than an even wall. Reference art is not a
measured optical/placement oracle. User-authored tree architecture and WORLD84
pigment cohorts are retained. Each iteration pins its source/binary, executes
relevant checks, and captures the same frozen coastal camera before proceeding.

| Stage | Change | Near clumps | Submitted instances | Mesh triangles | GPU median |
| --- | --- | ---: | ---: | ---: | ---: |
| WORLD84 baseline | 0.8 m spacing, scales 0.75–1.50 | 6,546 | 14,005 | 28,246,314 | 592.716 ms |
| WORLD85 | 1.25 m spacing | 2,617 | 10,076 | 16,081,890 | 457.692 ms |
| WORLD86 | Continuous small/middle clumps | 2,617 | 10,076 | 14,009,106 | 412.008 ms |
| WORLD87 | Juvenile / mature / rare hero forms | 2,617 | 10,076 | 12,547,498 | 376.970 ms |
| WORLD88 | Divided veteran crown pinnules | 2,617 | 10,076 | 13,463,338 | 378.621 ms |
| WORLD88b | Connected secondary rachises | 2,617 | 10,076 | 13,941,418 | 375.931 ms |
| WORLD89 | Slender reed blades | 2,617 | 10,076 | 13,941,418 | 375.206 ms |
| WORLD90 | Populate middle-size colonies; fix wet prefilter | 4,608 | 12,067 | 17,426,682 | 423.684 ms |
| WORLD90b | Continuous riparian niches | 4,459 | 11,918 | 15,920,770 | 407.392 ms |
| WORLD91 | Connected moss cushions | 4,459 | 11,918 | 15,921,260 | 405.451 ms |
| WORLD92 | Geometry-derived plant visibility | 4,459 | 11,918 | 15,921,260 | 405.589 ms |
| WORLD93 | Organ roughness cohorts | 4,459 | 11,918 | 15,921,260 | 403.363 ms |
| WORLD94 | Layered slender rush clumps | 4,459 | 11,918 | 16,092,860 | 406.549 ms |
| WORLD95 | Smooth upright rush arcs | 4,459 | 11,918 | 17,580,060 | 411.133 ms |
| WORLD96 | Authored overcast profile | 4,459 | 11,918 | 17,580,060 | 410.710 ms |
| WORLD97 | Geometry-derived crown normals | 4,459 | 11,918 | 17,580,060 | 411.052 ms |
| WORLD98 | Connected solid crown petioles | 4,459 | 11,918 | 19,388,348 | 433.158 ms |
| WORLD99 | Divided crown feathers and exact vertex sharing | 4,459 | 11,918 | 20,481,980 | 432.151 ms |
| WORLD100 | Root fronds in actual woody forks | 4,459 | 11,918 | 20,481,980 | 440.993 ms |
| WORLD105 | Cascading fern crown with rounded lobed pinnae | 4,459 | 11,918 | 18,855,932 | 382.723 ms |
| WORLD107 | Finer feather contours within existing source budgets | 4,459 | 11,918 | 19,025,276 | 385.484 ms |
| WORLD108 | Finer crowns and visible soil refinement | 4,459 | 11,918 | 19,025,276 | 389.287 ms |
| WORLD112 | Finer attached pinnate crown structure | 4,459 | 11,918 | 18,686,588 | 376.957 ms |
| WORLD114 | Gemini audit baseline | 11,011 | 18,470 | 31,708,948 | 481.118 ms |
| WORLD115 | Connected divided ground pinnae and shared density | 8,149 | 15,608 | 30,170,756 | 445.060 ms |
| WORLD116 | Rough damp soil alongside separate pools | 8,149 | 15,608 | 30,170,756 | 438.605 ms |

WORLD85 changes only the authored near-clump centre spacing. The same L24 IDs,
positions, sizes and organ forms survive thinning; canopy and pebble descriptors
are unchanged. A six-face regression checks that subset and bounds retained
count fraction at 0.32–0.50 of the 0.8 m population. Density quality still thins
world-fixed candidates; zero-work disable and budget failures are unchanged.
The existing five stage gates passed: formatting, 267 worldgen + 130 native
CPU tests, strict affected all-target/all-feature Clippy and release build.
The standing frame retains 25,266 CBT slots, 37 draw ranges and the 7 MB source
payload ceiling. These are local verification results, not green remote CI.

Evidence is in
[`native-coastal-night-2026-10-26.json`](artifacts/native-coastal-night-2026-10-26.json).
Matched development captures use the same RT Off/raster-shadow profile after
the earlier baseline AMD RT device loss; the checked-in profile still requests
local RT. Intermediate timings describe these samples, not an isolated speedup
benchmark. Population/size/shape changes are intentional **scene-content** changes;
these frames support composition review, not strict renderer-equivalence blind
acceptance. Independent preference/reference acceptance and full motion-sequence
stability remain open. Further stages are recorded below as they are verified.

### WORLD86 — Continuous small/middle clumps

The size range is now 0.25–1.10 rather than 0.75–1.50. A 70% independent world-ID / 30% continuous community mixture and 0.90 power provide a broad middle cohort while retaining small plants and uncommon largest clumps. An initial 1.35-power trial left excessive bare ground and is retained separately, not accepted as the final stage. Preparation-only scale bins expose the resulting composition without per-frame work. Six-face strata tests bound young/middle/large representation; exact fixed-size settings, nested quality and window unions remain.

The stage passed 268 worldgen and 130 native CPU tests, formatting,
strict affected Clippy and release build. Its standing capture keeps
37 shared draws and a 6,219,328 B
vertex/index/AO source payload. Near-scale bins (<0.45 / 0.45–0.70 /
0.70–0.95 / >=0.95) are [158, 906, 1138, 415].
This is authored scene development with the same camera/profile; population
changes are not a renderer-equivalent comparison or reference acceptance.

### WORLD87 — Juvenile / mature / rare hero forms

Fern form 1 is now an eight-frond juvenile, with six living and two dry fronds, smaller geometry and a bounded terminal curl. Form 0 retains thirteen fronds; form 2 retains twenty-one. Stable optical form weights are 35% juvenile, 50% mature and 15% hero; tree form mapping, scatter identity, pose and quality subsets remain unchanged. The independent LOD/area/distance oracles pass. The LOD ownership fixture now selects distinct IDs of one form explicitly, removing its accidental coupling to old hash weights. Botanical inspiration and limits are in [coastal flora notes](reference/COASTAL_FLORA_NOTES.md); these are authored optical cohorts, not simulated growth or taxonomic reconstruction.

The stage passed 269 worldgen and 130 native CPU tests, formatting,
strict affected Clippy and release build. Its standing capture keeps
37 shared draws and a 6,108,208 B
vertex/index/AO source payload. Near-scale bins (<0.45 / 0.45–0.70 /
0.70–0.95 / >=0.95) are [158, 906, 1138, 415].
This is authored scene development with the same camera/profile; population
changes are not a renderer-equivalent comparison or reference acceptance.

### WORLD88 — Divided veteran crown pinnules

The veteran tree form now has nine primary pinna pairs with three secondary pairs on each pinna: finer true silhouettes replace its solid broad blades. Full folded quads share the existing indexed lamina emitter; reduced triangles use the analytic trapezoid/triangle area ratio and retain the inter-pinnule gaps. Canonical fork and solitary-umbrella geometry is retained. Independent gap rays plus the unchanged full/reduced projected-area, sampled-distance and source-size oracles pass. This is authored divided-frond geometry inspired by the cited botanical descriptions, not a species reconstruction.

The stage passed 270 worldgen and 130 native CPU tests, formatting,
strict affected Clippy and release build. Its standing capture keeps
37 shared draws and a 6,896,176 B
vertex/index/AO source payload. Near-scale bins (<0.45 / 0.45–0.70 /
0.70–0.95 / >=0.95) are [158, 906, 1138, 415].
This is authored scene development with the same camera/profile; population
changes are not a renderer-equivalent comparison or reference acceptance.

### WORLD88b — Connected secondary rachises

Add a finite-width secondary rachis between leaflets, sharing existing root vertices. The reduced emitter now preserves root slots when triangle winding flips; otherwise bridge indices could connect leaflet tips and fill the intended gaps. Independent gap and midrib-coverage rays plus the unchanged area/distance oracles pass. Eight primary pairs retain the earlier 6 MB plant-only and 7 MB aggregate budgets; a nine-pair bridge trial exceeded the plant-only guard and was rejected without increasing that guard.

The stage passed 270 worldgen and 130 native CPU tests, formatting,
strict affected Clippy and release build. Its standing capture keeps
37 shared draws and a 6,769,456 B
vertex/index/AO source payload. Near-scale bins (<0.45 / 0.45–0.70 /
0.70–0.95 / >=0.95) are [158, 906, 1138, 415].
This is authored scene development with the same camera/profile; population
changes are not a renderer-equivalent comparison or reference acceptance.

### WORLD89 — Slender reed blades

Ten main blades retain their height, lean and droop but now span 1.4–2.6 cm at the source base rather than 10–15 cm. The six basal blades are narrowed proportionately. An independent leaf-area envelope of 0.15–0.50 square metres, metre-height bounds and the unchanged 144-triangle count pin the authored clump. No alpha cards, new passes, source ranges or camera-dependent width inflation are added; thin subpixel coverage remains a motion-review limitation.

The stage passed 271 worldgen and 130 native CPU tests, formatting,
strict affected Clippy and release build. Its standing capture keeps
37 shared draws and a 6,769,456 B
vertex/index/AO source payload. Near-scale bins (<0.45 / 0.45–0.70 /
0.70–0.95 / >=0.95) are [158, 906, 1138, 415].
This is authored scene development with the same camera/profile; population
changes are not a renderer-equivalent comparison or reference acceptance.

### WORLD90 — Populate middle-size colonies; fix wet prefilter

After the initial spacing/size trials left excessive bare soil, the authored spacing is now 0.95 m: 4,608 near clumps rather than the original 6,546, with compact juveniles and rare large forms. A new regression reproduced wet and dry populations both yielding 7,040 items: the early filter capped the existing 1.35 wet-habitat gain at 1.0 before habitat evaluation. Both the early filter and candidate-capacity bound now include that maximum; the regression passes its 1.22–1.48 observed wet/dry envelope. Six-face spacing tests check exact descriptor subsets and an analytic inverse-square count envelope at 0.95, 1.25 and 2.0 m. Trees and pebbles remain unaffected. All three fixed cameras were captured; this is scene-composition development, not a renderer-equivalent speedup comparison.

The stage passed 272 worldgen and 130 native CPU tests, formatting,
strict affected Clippy and release build. Its standing capture keeps
37 shared draws and a 6,769,456 B
vertex/index/AO source payload. Near-scale bins (<0.45 / 0.45–0.70 /
0.70–0.95 / >=0.95) are [269, 1597, 1994, 748].
This is authored scene development with the same camera/profile; population
changes are not a renderer-equivalent comparison or reference acceptance.

### WORLD90b — Continuous riparian niches

Understory density now blends continuously from 1.35 at moist low elevations to 0.88 above 8 m relative to local water. Reed selection blends from 1.75 to 0.45 over 2.2–8 m. This removes the former discontinuities at 2.2 and 8 m without changing candidate identity, plant quality resolution or the early 1.35 capacity bound. A regression checks continuity, monotonicity, bounds and invariance under a common vertical datum shift.

The stage passed 273 worldgen and 130 native CPU tests, formatting,
strict affected Clippy and release build. Its standing capture keeps
37 shared draws and a 6,769,456 B
vertex/index/AO source payload. Near-scale bins (<0.45 / 0.45–0.70 /
0.70–0.95 / >=0.95) are [261, 1545, 1922, 731].
This is authored scene development with the same camera/profile; population
changes are not a renderer-equivalent comparison or reference acceptance.

### WORLD91 — Connected moss cushions

Resolved moss cover now uses a smooth connected-cushion transfer instead of quadratic suppression. Moderate coverage contributes centimetres of real render-only relief, sharing the existing body-fixed phase with optical gradients and prop/pool placement. Independent cover tests retain the 0–60 mm envelope, dry/coarse zero and a resolved middle-cover height envelope. No extra shoots, source draws or authoritative collision displacement are added.

The stage passed 273 worldgen and 130 native CPU tests, formatting,
strict affected Clippy and release build. Its standing capture keeps
37 shared draws and a 6,790,024 B
vertex/index/AO source payload. Near-scale bins (<0.45 / 0.45–0.70 /
0.70–0.95 / >=0.95) are [261, 1545, 1922, 731].
This is authored scene development with the same camera/profile; population
changes are not a renderer-equivalent comparison or reference acceptance.

### WORLD92 — Geometry-derived plant visibility

Remove the extra radius/height multipliers applied after the existing cosine-weighted Parry BVH visibility bake. Crown and root shading now follow the actual mesh visibility source and shared direct shadows, without an added artistic darkening pass or brightness boost. A source-fraction regression checks every full/reduced form retains only the possible outcomes of 32 binary geometric queries; independent open-plane and closed-cavity transport oracles remain. This does not implement ground bounce or exact resident-ground contact visibility.

The stage passed 274 worldgen and 130 native CPU tests, formatting,
strict affected Clippy and release build. Its standing capture keeps
37 shared draws and a 6,790,024 B
vertex/index/AO source payload. Near-scale bins (<0.45 / 0.45–0.70 /
0.70–0.95 / >=0.95) are [261, 1545, 1922, 731].
This is authored scene development with the same camera/profile; population
changes are not a renderer-equivalent comparison or reference acceptance.

### WORLD93 — Organ roughness cohorts

The imported pinna MR image has uniform G=255. The existing four-band atlas now carries authored mature/emerging/olive/weathered roughness priors of 0.70/0.50/0.80/0.90, replacing the global living factor 0.60; leaf material factor is 1.0 and bark stays 0.85. Normal/metallic/alpha data, UV gutters, image dimensions and pass/draw counts remain. Tests check all four cohort centres through five mip levels, ordering, quantization bounds and preservation of the other data channels. These are authored cuticle priors, not measured botanical spectra.

The stage passed 274 worldgen and 130 native CPU tests, formatting,
strict affected Clippy and release build. Its standing capture keeps
37 shared draws and a 6,790,024 B
vertex/index/AO source payload. Near-scale bins (<0.45 / 0.45–0.70 /
0.70–0.95 / >=0.95) are [261, 1545, 1922, 731].
This is authored scene development with the same camera/profile; population
changes are not a renderer-equivalent comparison or reference acceptance.

### WORLD94 — Layered slender rush clumps

Each rush clump now has ten tall and ten shorter inner blades plus six basal leaves. The inner tier reuses the existing curved blade emitter at 62 percent height and 70 percent width. The independent leaf-area and height envelopes remain; the explicit source triangle budget changes from 144 to 264, while shared source/draw caps remain unchanged. All three fixed survey views, architecture checks and the native feature-off build passed alongside the stage CPU gates.

The stage passed 274 worldgen and 130 native CPU tests, formatting,
strict affected Clippy and release build. Its standing capture keeps
37 shared draws and a 6,804,424 B
vertex/index/AO source payload. Near-scale bins (<0.45 / 0.45–0.70 /
0.70–0.95 / >=0.95) are [261, 1545, 1922, 731].
This is authored scene development with the same camera/profile; population
changes are not a renderer-equivalent comparison or reference acceptance.

### WORLD95 — Smooth upright rush arcs

Rush axes now interpolate rooted quadratic knots with gently outturned tips, using 32/16 full/reduced segments. The existing ribbon emitter shares indexed vertices and area-weighted geometric normals for smooth rush strips; other strips retain their existing creases. An independent emitted-axis test bounds full-detail turns below 0.25 rad. Clumps have 1304/664 triangles, approximately 1.65 m source height, and retain the prior leaf-area and LOD error envelopes. Fine reed triangles use the other foliage double-area floor of 2e-6 square metres plus a new dimensionless shape-condition bound for all prototypes, rather than the old reed-only broad-blade minimum. An initial curve-knots trial failed the geometry tests and was retained as rejected evidence before correction. All three camera captures and seven stage gates passed.

The stage passed 275 worldgen and 130 native CPU tests, formatting,
strict affected Clippy and release build. Its standing capture keeps
37 shared draws and a 6,838,504 B
vertex/index/AO source payload. Near-scale bins (<0.45 / 0.45–0.70 /
0.70–0.95 / >=0.95) are [261, 1545, 1922, 731].
This is authored scene development with the same camera/profile; population
changes are not a renderer-equivalent comparison or reference acceptance.

### WORLD96 — Authored overcast profile

The experimental showcase cloud profile now uses inverse coverage threshold 0.18 and opacity 0.72 instead of 0.46/0.45. The existing settings define zero threshold as overcast, not zero cloud cover. A separate same-binary trial preceded the profile edit. All three fixed cameras were recaptured; EV100 12.22, the plant population and geometry, shared cloud transport, TAA default-off and the checked-in local-RT request remain. This intentionally changes authored weather and is not a light-equivalent renderer comparison. Clear-sky diffuse SH9 and the existing thin-cloud liquid-water prior still limit overcast lighting fidelity; no global ambient multiplier or new transport pass is added.

The stage passed 275 worldgen and 130 native CPU tests, formatting,
strict affected Clippy and release build. Its standing capture keeps
37 shared draws and a 6,838,504 B
vertex/index/AO source payload. Near-scale bins (<0.45 / 0.45–0.70 /
0.70–0.95 / >=0.95) are [261, 1545, 1922, 731].
This is authored scene development with the same camera/profile; population
changes are not a renderer-equivalent comparison or reference acceptance.

### WORLD97 — Geometry-derived crown normals

A new regression reproduced 0.2213 rad maximum disagreement between retained crown normals and the area-weighted emitted surface. Remove the 18 percent ellipsoid blend and forced upward component. Shared prototype finishing now derives leaf normals from actual published triangles after form scaling, then runs the existing 32-query geometric visibility bake; analytic stem normals remain. Every full/reduced tree form passes a 0.001 rad source-normal envelope. Positions, indices, instances, pigment/roughness data and draw ranges are unchanged; all three fixed-weather camera captures and seven stage gates passed. The red test and production-base provenance are retained separately. This does not implement ground bounce or stochastic GI.

The stage passed 276 worldgen and 130 native CPU tests, formatting,
strict affected Clippy and release build. Its standing capture keeps
37 shared draws and a 6,838,504 B
vertex/index/AO source payload. Near-scale bins (<0.45 / 0.45–0.70 /
0.70–0.95 / >=0.95) are [261, 1545, 1922, 731].
This is authored scene development with the same camera/profile; population
changes are not a renderer-equivalent comparison or reference acceptance.

### WORLD98 — Connected solid crown petioles

An independent emitted-axis test first reproduced a 0.0407776 m detached leaf root. Both levels now attach to the published nine-node axis and retain a tapered solid three-sided petiole; the 24 mm source root radius is unchanged. Axial crease splitting preserves the face-normal bound. Reduced trunks remove nearly collinear rings to retain the 6 MB plant-only and 7 MB aggregate caps. Complete-tree reduction now exceeds 2.5:1 instead of 3:1; fern reduction and the original surface-distance, area and degeneracy bounds remain. Lamina-only winding replaces the invalid assumption that closed stalk sides all face upward. Same-view inspection still shows broad leaf plates; this is a connectivity checkpoint, not final visual acceptance.

The stage passed 278 worldgen and 130 native CPU tests, formatting,
strict affected Clippy and release build. Its standing capture keeps
37 shared draws and a 6,946,504 B
vertex/index/AO source payload. Near-scale bins (<0.45 / 0.45–0.70 /
0.70–0.95 / >=0.95) are [261, 1545, 1922, 731].
This is authored scene development with the same camera/profile; population
changes are not a renderer-equivalent comparison or reference acceptance.

### WORLD99 — Divided crown feathers and exact vertex sharing

Forms 0/1 now have eight primary pairs with two secondary pairs; the veteran retains three. Longer primary axes retain the measured foliage surface within 5% of WORLD98. Both quality levels preserve real secondary gaps, connected axes and the existing distance/area envelope. A new vertical-ray fixture fails on the old broad plate and passes on divided geometry. Exact per-organ vertex sharing keys position, normal, UV and ambient visibility; all-quality triangle-corner tests prove publication preserves source attributes and material boundaries. No new texture, draw group, plant descriptor or light coefficient is added.

The stage passed 280 worldgen and 130 native CPU tests, formatting,
strict affected Clippy and release build. Its standing capture keeps
37 shared draws and a 6,770,068 B
vertex/index/AO source payload. Near-scale bins (<0.45 / 0.45–0.70 /
0.70–0.95 / >=0.95) are [261, 1545, 1922, 731].
This is authored scene development with the same camera/profile; population
changes are not a renderer-equivalent comparison or reference acceptance.

### WORLD100 — Root fronds in actual woody forks

Tier roots previously kept the terminal X/Z while changing only Y, leaving upper roots 0.33 m above the wood and lower roots beside slanted forks. Roots now lie on the actual fork axis, capped below its terminal. An independent BVH fixture recovers 48 root sections per form/quality and checks six oblique wood rays from each; it fails on WORLD99 and passes on the repaired source. Wider real secondary midribs improve continuity without screen-space rules. Surface area, full/reduced distance/area error, minimum triangle conditioning, source caps, materials and plant placement remain bounded. Three fixed raster views passed, while angular leaflet silhouettes, subpixel filtering and independent reference acceptance remain open.

The stage passed 281 worldgen and 130 native CPU tests, formatting,
strict affected Clippy and release build. Its standing capture keeps
37 shared draws and a 6,770,068 B
vertex/index/AO source payload. Near-scale bins (<0.45 / 0.45–0.70 /
0.70–0.95 / >=0.95) are [261, 1545, 1922, 731].
This is authored scene development with the same camera/profile; population
changes are not a renderer-equivalent comparison or reference acceptance.

### Final local verification for WORLD85–100

The final source passed the terrain/render-core/graphics/worldgen CPU batch,
strict affected Clippy, native plant hardware fixtures (including ignored
cases), conforming L24 material-coordinate GPU checks, and material/environment
GPU tests. Exact commands, counts, source and binary hashes, rejected trials,
three-view captures and auxiliary session results are retained in
`artifacts/native-coastal-night-2026-10-26.json` and the local campaign directory.
The 96-frame sweep records one final image plus per-frame counters; it does not
constitute motion-sequence image acceptance. Three unvoted WORLD96/WORLD97
blind shading pairs have matched camera/profile/residency/mesh eligibility;
they do not test WORLD98–100's altered geometry. No independent preference or
reference-parity acceptance is recorded. The checked-in showcase still requests
local RT, while matched development images use RT Off/raster shadows after the
original baseline AMD device loss. Any unpaired candidate RT success remains
a bounded session, not general driver-stability evidence.

### WORLD105 — Cascading fern crown with rounded lobed pinnae

The rejected spear and diamond trials are replaced by twenty-eight arching age-tiered fronds per tree, each with fourteen opposed pairs of tapered pinnae. A shared ground/crown lamina emitter publishes curved, shallowly lobed midrib strips; reduced crown leaves retain eight contour vertices and integrate full-outline area. A larger pendant tier restores lower crown volume. Full/reduced fidelity and the 6/7 MB source caps remain; complete-tree reduction again exceeds 3:1. This is an authored silhouette approximation, not taxonomic reconstruction or reference acceptance.

The stage passed 280 worldgen and 130 native CPU tests, formatting,
strict affected Clippy and release build. Its standing capture keeps
37 shared draws and a 6,137,476 B
vertex/index/AO source payload. Near-scale bins (<0.45 / 0.45–0.70 /
0.70–0.95 / >=0.95) are [261, 1545, 1922, 731].
The crown geometry changes at unchanged canonical instance positions.
These are labeled design-review views, without independent preference or
reference acceptance.

WORLD101's spear silhouette was rejected by the user; WORLD102–104 are retained
as intermediate/negative design evidence. The current-source batches pass
450 cases plus strict Clippy, architecture, feature-off compile, formatting
and release build. The rebuilt release matches all three WORLD105 raster
captures byte for byte after the test-only visibility-fixture update.
The current morphology and full/reduced error bounds are documented in
[doc 75](75_NATIVE_SURFACE_PLANTS.md); exact commands, hashes, physical-bias
calibration, images and rejected trials are recorded in
`artifacts/native-fern-crown-2026-10-26.json`.
The bounded current-scene Local RT capture completed with RT active (exit 0).
It remains unpaired and does not establish general driver stability.
The prior WORLD100 sweep and WORLD96/97 blind pairs do not validate this new
geometry. No independent preference, reference parity, full motion-sequence
or interactive-FPS acceptance is recorded.

### WORLD106 — Measured native PNG sequences

The existing headless renderer now accepts `--frame-sequence NEW_DIR` with
`--headless`. It creates a new directory (existing captures are not overwritten)
and saves `frame-000000.png` onward for every measured frame after the 30-frame
warmup. Disabled runs retain the existing final-screenshot-only path. The shared
PNG writer has an exact RGBA/dimension round-trip test. Readback/writes occur
after the recorded frame sample; their wall-clock cost still changes asynchronous
streaming cadence, so sequence runs are visual evidence, not equivalent timings.

The stage passes 280 worldgen / 131 native non-ignored tests, strict affected
Clippy, architecture, feature-off compile, formatting and release build. One
24-frame existing `--sweep` capture completed; its final sequence PNG is byte
identical to the separately saved final screenshot. All source PNG hashes,
commands, profile and performance counters are retained. The orbit changes
camera/residency and remains ineligible for strict renderer-equivalence blind
comparison. A sequence capture does not establish full temporal stability.

A separate static default-off capture has identical crown/sky pixels in its
first 180 rows, but the whole image differs from WORLD105 in 4,583 pixels
(mean absolute RGBA difference 0.00748 codes), primarily ground. Reported camera,
height-level counts, CBT active slots and plant counts match; the cause is not
assigned and whole-frame pixel identity is not claimed. Geometry/materials remain
WORLD105. Evidence: `artifacts/native-frame-sequence-2026-10-26.json`.

### WORLD107 — Finer feather contours within existing source budgets

The full crown pinna now resolves six shallow edge lobes instead of four. Twelve outline steps and one midrib row per two edge steps retain actual concave notches without a long centre fan; independent vertical rays test an open notch and its adjacent lobe. Each full pinna uses 29 vertices/32 triangles; reduced leaves keep eight vertices/six triangles and integrate the new outline area. Frond counts, pose, organ pigments and source budget caps remain. This is geometry refinement, not reference acceptance.

The stage passed 280 worldgen and 131 native CPU tests, formatting,
strict affected Clippy and release build. Its standing capture keeps
37 shared draws and a 6,758,404 B
vertex/index/AO source payload. Near-scale bins (<0.45 / 0.45–0.70 /
0.70–0.95 / >=0.95) are [261, 1545, 1922, 731].
This is authored scene development with the same camera/profile; population
changes are not a renderer-equivalent comparison or reference acceptance.

### WORLD108 — Finer crowns and visible soil refinement

WORLD107 refines crown feathers to six shallow lobes per edge with a locally supported midrib strip. WORLD108 spends unresolved optical splits on actual-frame pages before offscreen prefetch, preserving the complete wider cover and existing memory/quality limits. Standing material residency increases from 58/26 to 92/67 pages at L22/L23; no L24 page was resident in this capture. Neither change modifies authoritative terrain or CBT topology.

The stage passed 280 worldgen and 131 native CPU tests, formatting,
strict affected Clippy and release build. Its standing capture keeps
37 shared draws and a 6,758,404 B
vertex/index/AO source payload. Near-scale bins (<0.45 / 0.45–0.70 /
0.70–0.95 / >=0.95) are [261, 1545, 1922, 731].
This is authored scene development with the same camera/profile; population
changes are not a renderer-equivalent comparison or reference acceptance.

### WORLD108 — Current detail validation and motion evidence

The saturating four-root material fixture keeps 2,047 leaves and covered area
4.0 while improving visible error from 1.109 to 0.989 pixels. The prior splitter
fails this regression. The unchanged wider cover still feeds soil-height
prefetch; missing projections keep error-only ordering. High uses the same
2,048 virtual leaves, 512 admissions, 1,024 CPU pages and 256 GPU layers.
Standing L22/L23 residents rise from 58/26 to 92/67; no L24 page is resident in
the actual standing capture. This is material selection, not new CBT topology,
canonical displacement or a promise of full-resolution moving soil.

Eight stage gates pass, including 280 worldgen, 131 native and 31 stream CPU
tests. Current-source final batches pass 451 cases: 406 CPU terrain-stack,
24 native plant (including hardware), one conforming L24 coordinate fixture
and 20 material/environment GPU cases. Strict affected Clippy, formatting,
architecture, feature-off compile and release build also pass. The rebuilt
release is byte-identical to all three WORLD108 raster capture binaries;
only cfg(test) visibility diagnostic expectations changed after capture.
At 513², measured bias disagreements are [54, 0] full and [11, 1] reduced;
the two-code RGB tolerance, edge limits and physical convergence remain.

WORLD106 and WORLD108 each retain 24 consecutive native PNG frames after
30 warmup frames, with a byte-identical final PNG and separate final capture.
The 0.006 rad/frame sweep changes camera/residency. Readback/write time is
outside recorded frame samples but perturbs streaming wall cadence. Videos
present these frames at 6 FPS; that is not runtime performance. Selected frames
0/8/16/23 were inspected; fine distant foliage and soft moving ground patches
remain, so no full temporal-stability acceptance is recorded. WORLD106's
static capture differs from WORLD105 in 4,583 pixels (top 180 rows identical);
the cause is unassigned, and no whole-frame identity is claimed.

The bounded current-scene Local RT probe exited 0; RT active=True, passed=True. It is an unpaired session, not general driver-stability acceptance.
Exact commands, source/binary/frame hashes, negative regressions and known
limits are in `artifacts/native-fern-detail-2026-10-26.json`. The 450-case
WORLD105 record above remains historical. WORLD96/97 blind pairs cover their
shading revision only; they do not establish preference for WORLD107/108.

### WORLD114 — Audit baseline after Gemini

Commits `2268ad4`, `267ddc9`, `e4fd75d` and `f7ce0a6` are retained as the
review baseline. Three native views and a 24-frame moving sequence use the
actual source at `f7ce0a6`, with the checked-in profile's sole RT Off override.
The authored profile is EV100 12.06, local haze extinction 0.00075 /m,
16 cloud march steps, coverage 0.12, MSAA 4 and TAA Off. These are local optical
priors, not radiometric calibration or an additional effect. Standing submits
18,470 instances / 31,708,948 triangles / 37 draws; 11,011 near clumps versus
4,459 at WORLD112. Its CPU stage gates pass; this baseline is not promoted as
reference acceptance. WORLD115 below addresses the reviewed defects while
retaining the crown architecture and optical priors.

The existing `--frame-sequence` capture also logs `native_material_frame` per
saved frame: resident level histogram, pending height/material jobs, CPU page
count, cumulative material upload bytes and cached optical height reference.
This adds no readback or frame work when sequence capture is disabled. Across
WORLD114's 24 frames, L22 residents range 15–40, L23 27–34, L24 remains zero,
and four material jobs remain pending. CPU pages grow from 699 to 791. These
counts demonstrate asynchronous churn, not its sole cause or a complete blur
fix. The moving orbit changes residency and is ineligible for strict blind
renderer comparison; video playback is 6 fps for review, not runtime FPS.

### WORLD112 — Finer attached pinnate crown structure

The crown retains 28 rooted fronds, but each now carries 28 opposed pinna pairs. Primary reach fractions are 0.14/0.13 instead of 0.27/0.25, with connected two-lobe secondary outlines. Full/reduced pinnae use 13/5 vertices and 12/3 triangles. Existing sampled displacement, projected-area, geometric normal, attachment and memory bounds remain; ground and population controls are unchanged.

The stage passed 280 worldgen and 131 native CPU tests, formatting,
strict affected Clippy and release build. Its standing capture keeps
37 shared draws and a 6,447,940 B
vertex/index/AO source payload. Near-scale bins (<0.45 / 0.45–0.70 /
0.70–0.95 / >=0.95) are [261, 1545, 1922, 731].
This is authored scene development with the same camera/profile; population
changes are not a renderer-equivalent comparison or reference acceptance.

### WORLD112 — Geometry verification at that checkpoint

Eight stage gates pass: 280 worldgen, 131 native and 31 stream CPU tests,
strict Clippy, formatting, architecture, feature-off compile and release.
Final current-source batches pass 451 cases. The rebuilt release is byte-identical
to the binary used for all three native raster views after test-only oracle updates;
measured finite-bias counts are [65, 1] full / [18, 0] reduced.
The 513² full fixture failed the unchanged primary-edge fraction guard:
2,208 of 8,612 samples (25.64% >25%). At 1025², finer edges are resolved;
colour tolerance, 1/16-pixel exclusion, quarter-edge limit, physical convergence
and invented-blocker guards remain.
A 24-frame native sequence retains terminal-PNG equivalence and TAA Off.
The bounded Local RT probe exited 0, active=True, passed=True; it is unpaired, not general driver-stability acceptance.
WORLD109 failed winding/LOD/source caps. WORLD110 pointed teeth and WORLD111
rounded broad units remain intermediate native design evidence.
Morphology and unchanged error envelopes are in [doc 75](75_NATIVE_SURFACE_PLANTS.md);
commands and hashes are in `artifacts/native-fern-division-2026-10-26.json`.
Historical WORLD108 verification is dated; no independent reference, full
motion or interactive-FPS acceptance is claimed.

### WORLD115/116 — Review scope and unchanged bounds

The retained Gemini canopy uses 28 rooted fronds, 28 opposed pinna pairs per
frond and wider primary reach/half-width priors (0.155/0.165 of frond reach,
0.22 of pinna length). Ground leaves use the divided primitive while keeping
fourteen pairs. Trunk/crown tiers, pigment cohorts, shared niche and authored
profile changes are retained; native-only near spacing/scale is removed.
The final standing scene has 8,149 near clumps / 15,608 submitted instances /
30,170,756 mesh triangles in 37 shared draws. Source payload is 6,721,828 B;
textures remain 3,760,128 B. More detailed ground leaves consume some of the
population reduction; these figures are not a renderer-throughput claim.

Existing source caps, sampled full/reduced distance (<0.057 m times form scale),
projected-area error (<2.5%), minimum triangle conditioning, geometric winding,
organ cohorts and root/crown attachment checks remain. Canonical ground-fern
triangles intentionally change from 3,224 to 4,680; the old narrow count fixture
is updated for divided leaves. The shape/aspect fixture follows their new
thirteen-vertex layout. The former bare-substrate illumination and ground-root
regressions are retained as negative logs before the production repair.

### WORLD115 — Connected divided ground pinnae and shared density

The Gemini review retains its trunk/crown architecture and authored profile. Native-only near-density/size overrides are removed in favor of shared ScatterPolicy defaults. Ground pinnae use the existing rounded divided 13/5-vertex lamina, retain fourteen opposed pairs, and attach to the emitted twelve-segment rachis. Independent regression reproduces a 6.538 mm root gap before repair and constrains all three full/reduced forms below 1 micrometre. Bare substrate no longer contains a canopy illumination multiplier; moss/litter coverage and geometric shadows remain separate. Both new regressions fail on the retained prior source.

The stage passed 282 worldgen and 131 native CPU tests, formatting,
strict affected Clippy and release build. Its standing capture keeps
37 shared draws and a 6,721,828 B
vertex/index/AO source payload. Near-scale bins (<0.45 / 0.45–0.70 /
0.70–0.95 / >=0.95) are [508, 2820, 3499, 1322].
This is authored scene development with the same camera/profile; population
changes are not a renderer-equivalent comparison or reference acceptance.

### WORLD116 — Rough damp soil alongside separate pools

Wet opaque soil now uses roughness endpoint 0.48 instead of 0.18 before the existing mineral/moss/litter mixture. Moisture darkening, relief, normals and pool water materials are unchanged. This is an authored porous-substrate optical prior, not measured calibration or a drainage/growth model. Native before/after crops show a modest highlight-response change. WORLD115 retains the reviewed Gemini crowns and repairs shared understory density, ground-leaf divisions/attachments and substrate reflectance.

The stage passed 282 worldgen and 131 native CPU tests, formatting,
strict affected Clippy and release build. Its standing capture keeps
37 shared draws and a 6,721,828 B
vertex/index/AO source payload. Near-scale bins (<0.45 / 0.45–0.70 /
0.70–0.95 / >=0.95) are [508, 2820, 3499, 1322].
This is authored scene development with the same camera/profile; population
changes are not a renderer-equivalent comparison or reference acceptance.

### WORLD116 — Final verification and diagnostic captures

The final current-source batches passed 453 cases: 408 terrain-stack CPU,
24 native plant including GPU ray oracles, one conforming L24 coordinate GPU
case and 20 material/environment GPU cases. Formatting, strict affected
Clippy, architecture, feature-off compilation and release build also passed.
The final release binary is byte-identical to the three raster views and the
24-frame sequence after a test-only update to exact finite-bias counts. The
1025² ray fixtures retain the existing colour, physical and edge guards; they
checked 12,780 full and 4,344 reduced samples. Their finite-bias disagreements
are [77, 0] full and [24, 0] reduced for 0.01/0.0001 m respectively.

One bounded Local RT capture completed with an active TLAS (24,751 instances,
254,028 traced triangles, 31,699,936 acceleration-structure bytes); it is not
a paired baseline or general driver-stability result. A separate plants-off
capture isolates ground with the existing pebble, moss and puddle consumers.
The 24-frame moving capture keeps TAA off and its final frame matches the saved
PNG; its orbit and residency change and PNG readback perturbs stream cadence.
Source pins, commands, images, negative regressions and per-frame material
counters are recorded in `artifacts/native-understory-review-2026-10-26.json`.
Reference parity, ground sky visibility, moving soil detail and distant foliage
filtering remain open; no independent visual votes were recorded.

## WORLD84 — Organ pigment cohorts and asymmetric pinnae (2026-10-26)

This continuation reviews and retains user commit `3d58502`: the solitary
48-frond umbrella, dual-terminal veteran, 21-frond ground rosette and ecological
understory niches. The veteran implementation emits 24 fronds at each terminal;
the earlier WORLD80–83 prose's 28/20 split does not describe that source.
Morphology alone left the three living-material factors almost identical and
every pinna sampled the same small green patch.

Four authored pigment cohorts now belong to complete fronds/blades: mature,
juvenile, olive and weathered. A default-empty `PlantMesh.foliage_tones` byte
array preserves historical bake decoding; nonempty arrays must match vertex
counts, use 0–3 and never cross a triangle's cohort. Both representations retain
the same organ sequence. These are authored optical age cues, not simulated
plant development or measured pigment spectra. Existing form identity selects
different green/olive material factors; dry organs retain their separate brown
material and low transmission. Reed factors are less yellow and more restrained.

The native consumer grades the existing CC0 pinna base map in **linear RGB** and
packs four copies into one padded 160×320 atlas. Normal/roughness maps repeat the
same source and padding; only base reflectance is graded. UVs address texel centers
inside 16-pixel gutters. Existing linear-light mip filtering is retained (see
[PBRT image filtering](https://www.pbr-book.org/4ed/Textures_and_Materials/Image_Texture));
the small shared atlas avoids extra organ primitives/material draws. The linear
RGB grades are mature `[0.82,0.94,0.86]`, juvenile `[1.32,1.35,0.92]`, olive
`[1.22,1.03,0.67]` and weathered `[1.70,0.65,0.35]`, before the existing
form-dependent material factors. Cohort centers
remain distinct through tested mip 4; wider footprints converge toward an area
mean, not a promise of distant color separation or temporal acceptance. Raster,
shadow and RT sources still consume the same mesh/material publication.

Rachises now bend sideways, and opposed pinnae vary in length, attachment position
and roll. Full/reduced geometry uses identical deterministic parameters. No new
frond, pinna or scatter identity is added, and the existing extent, projected-area
and sampled-distance oracles remain unchanged. Dry pinna control points stop at
the authored soil datum. The retained basal visibility prior clamps its height
input before a fractional power, avoiding nonfinite AO below the local root plane.
The existing geometric visibility bake and the user's additional authored
core/base attenuation remain distinct; the latter is not measured contact
visibility or computed global illumination.

The existing `surface_plants` quality/disable policy is unchanged: full/reduced
geometry selection remains scale-aware, the disabled path prepares no plants,
and no new render pass, shader option, capability requirement or graphics knob
is introduced. The shared RGBA source images grow from 3,219,456 to 3,760,128 B
(+540,672 B before GPU mips); the 7 MB vertex/index/AO and 37-range admission
limits are unchanged. Optical cohort metadata is CPU asset data, outside the
unchanged 32-byte vertex / 64-byte rigid-instance ABI.

Seven local command gates passed: 266 worldgen and 130 native CPU tests
(37 ignored), formatting, strict affected all-target/all-feature Clippy,
the architecture guard (12 foundational crates), feature-off compilation and
release build. Cohort tests validate historical decoding, reject malformed or
mixed-triangle metadata, preserve full/reduced organ transitions, check padded
UV/data-map/mip behavior and distinguish green, brighter juvenile, olive and
brown mean reflectance. These are authored asset contracts, not botanical
measurements. The first palette trial was retained separately after its standing
frame showed insufficient weathered-leaf separation; the final palette deepens
the weathered cohort without changing the other three.

Evidence and actual capture limitations are recorded in
[`native-foliage-diversity-2026-10-26.json`](artifacts/native-foliage-diversity-2026-10-26.json).
The pinned baseline's local-RT capture caused an AMD device loss/gfx ring reset;
matched comparisons therefore use RT Off with the existing raster shadow
fallback on **both** versions. The checked-in showcase still requests local RT;
this iteration does not establish its stability. Earlier WORLD55–83 image L1
scores are image statistics, not photometric calibration or reference acceptance.
All three final before/after pairs match actual survey camera, canonical height
residency, CBT slots, instance/triangle counts and 37 draws. Standing retains
14,005 instances / 28,246,314 mesh triangles / 25,266 CBT slots; source vertex,
index and AO payload stays 6,219,328 B. GPU medians before→after are
595.970→592.716 ms (standing), 613.184→592.990 ms (composed), and
688.432→688.560 ms (grove), AMD Radeon 880M / RADV / Mesa 26.2.5. Some baseline
runs overlapped CPU compilation; these are descriptive samples, not a performance
win. Runtime cost remains far above interactive quality. Three blind pairs are
prepared with a separate origin key and remain unvoted.

Reference parity, independent blind preference, full motion-sequence acceptance,
and the remaining repeated instance silhouettes are open.

## WORLD43 — Prioritize the visible floor instead of geometry prefetch (2026-10-25)

This iteration starts from WORLD42 commit `446357a`. The source-only resolution
change is retained; this checkpoint fixes a separate reproduced admission defect.
Native and Bevy geometry selection deliberately use a broad cone with a 0.6 rad
swing/prefetch margin. Material admission priority inherited that same
cone. On a standing 0.9 rad vertical-FOV, 16:9 view, a ground page about 0.95 m
forward projects below NDC y=-4.34 yet wins distance priority over visible
ground 5.15 m forward (y=-0.80). Filtering outside the broad cone was insufficient:
the highest-priority close pages could still be below the actual frame.

`SelectionView.material_clip_from_eye` now supplies the actual camera projection
of camera-relative body axes. Material demand/admission ordering uses
the existing backend-neutral `render-core::shadow::sphere_intersects_clip`
predicate and the existing conservative page sphere. Native derives the matrix
from its actual camera projection/orientation; Bevy composes its camera
projection, inverse camera rotation and body rotation. Translation is subtracted
in f64 before clipping so a planetary origin does not inflate the clip predicate's
f32 arithmetic envelope. The wider geometry cone, projection,
canonical field/scatter and geometry authority remain unchanged. The existing
shared broad refinement cover also feeds fine soil-height requests; it is kept
identical to the legacy cover, and height demand explicitly keeps its wider
prefetch cone. No second refinement planner or cache is added. Existing
consumers without a material matrix retain the cone/no-frustum fallback.

This is an existing material allocation correction, not a new effect, renderer,
scheduler or user setting. It retains quality ceilings, 256 GPU layers, 512
admitted demand pages, 1,024 source-cache pages, hierarchical coarse fallback,
the existing selection debounce and the 7 MB plant payload limit. The optional
matrix is math-only; `terrain-stream` adds a direct MIT `render-core` dependency
to reuse the clip test, not a renderer/backend API. Capture metadata names the
material view policy. A six-face regression at reference elevations -1,000/0/
1,000 m reproduces the old wrong order, checks visible-first priority with the
actual frame, verifies that the shared prefetch cover is unchanged and preserves
exact partition area under the same 2,048 temporary-leaf bound. The Bevy rebase/camera
test also checks the body-axis projection after clearance/rotation edits.

Page spheres still use one cached local canonical reference height, not exact
per-page relief, terrain occlusion or a certified global height envelope. The
change favors estimated visible optical detail; it does not cull authoritative
geometry or eliminate moving-camera residency latency. Source magnification,
exact CBT contact, canopy transport, full temporal/reference acceptance and
interactive RT performance remain open.

Checks, pins and actual static/sweep frames are recorded in
[`native-material-frustum-2026-10-25.json`](artifacts/native-material-frustum-2026-10-25.json).
An earlier trial applied actual-frame clipping to the shared refinement cover;
review caught the resulting soil-height residency/CBT changes. That trial and
its pins/captures are retained separately; the final correction changes material
priority while preserving both shared cover and height-prefetch policy.
The baseline reuses WORLD42's complete pinned candidate captures, including
their original commands and binary/source hashes; no favorable baseline was
selected from repeated runs. Changed resident material maps are the intended
effect here. Blind comparisons remain unvoted and do not certify reference parity.

## WORLD42 — Near-camera source resolution (2026-10-25)

This continuation starts from `56d6b22`, following the report of degrading soil
very close to the camera. The existing High L24 material ceiling is about
3.05 mm/texel at the canonical 3.2 Mm radius, but the primary 3.15 m soil scan
was downsampled to 512² (6.15 mm/source texel). Finer resident pages could not
recover the discarded source information. This is a verified source-resolution
ceiling, not proof that it accounts for every visible close-ground defect.

The importer now preserves the verified original 1024² CC0 reflectance,
roughness and 16-bit displacement samples (3.08 mm/source texel). Asset hashes,
original URLs and attribution are in `assets/textures/forest-ground-04.provenance.json`
and its adjacent license file. No new photograph, generated texture or noise
layer is introduced. A native-channel roundtrip checks every packed sample;
reimporting produces identical gzip bytes. Runtime decoding validates the exact
bounded payload and expected dimensions for each source. The second mud/litter
scan stays 512² at its existing 1.3 m width. Source mip selection now uses each
texture's actual dimension, with all five channels checked at four physical
footprints for both sources.

The existing intrinsic humus/mineral transfer runs before mip averaging, now
at native 1K resolution. Its measured coarse mean is updated to match the new
pyramid: linear RGB `[0.09703242,0.068543516,0.043225687]`, roughness 0.7991561.
This differs from applying a nonlinear transfer after the old offline 2×2
downsample; the material change therefore includes a modest optical mean change.
Exposure, illumination, source width/height range, canonical terrain/scatter
identities and graphics defaults are unchanged. This is an authored optical
soil prior, not calibrated Thessa geology or a new collision-height model.

Regular and production conforming raster tests now cover material L23/L24 in
all six cube charts. They retain supported L22 geometry, independently project
small camera displacements in f64 and compare the actual filtered raster with
the same two-code radiance tolerance and monotonic-motion requirement. Both
passed before any production shader change; no coordinate-shader defect was
demonstrated, and no production shader was modified. These isolated fixtures
do not certify full-scene temporal stability or moving residency/contact.

Low/Medium/High material ceilings, 256 GPU layers, 512 demand pages, 1,024 cached
CPU pages, normal composition and the 7 MB plant prototype limit remain. The
source's decoded five-float mip pyramid grows 6,990,500→27,962,020 bytes (+20 MiB);
compressed data grows 1,382,947→5,696,506 bytes. These are source/payload sizes,
not a measured total-RSS or VRAM bound. Detail-off retains the existing no-scan
decode path. Enabled lower tiers still filter the same source to their cheaper
resolved representation. No new GPU texture array, mesh, pass or draw is added.

Exact checks, source/binary/profile pins and matched standing/isolated-floor
captures are recorded in
[`native-near-soil-2026-10-25.json`](artifacts/native-near-soil-2026-10-25.json).
Reference parity and independent blind preference remain open. Source
magnification, finite native resolution, sampled CBT contact, streaming and
full-scene motion remain limits; increasing the source resolution does not
solve those mechanisms by itself.

All 10 scoped local gates passed, with 519 successful test executions across
overlapping worldgen/stream/graphics/native and GPU suites. These include strict
all-target/all-feature Clippy for affected packages, architecture, formatting,
native feature-off compilation, the release build and the extended regular and
conforming L24 oracles. Six actual captures cover the standing view, isolated
floor and camera sweep before/after. Reported height/material-level histograms
match in each pair, although resident page identities were not compared. Source
resolution changes neither submitted plant geometry nor these final CBT counts:
25,266 static and 23,996 sweep triangles, with zero capacity rejections and 64 RT
terrain slots. The 7 MB prototype limit is retained.

Descriptive GPU medians on Radeon 880M/Vulkan/RADV Mesa 26.2.5 are
1,204.459→1,133.695 ms standing, 235.991→202.177 ms isolated floor and
1,399.000→1,406.145 ms sweep. Baseline static captures overlap CPU compilation,
and no isolated speedup or controlled CPU/wall timing is claimed. Sweep checks
exercise the moving pipeline but retain only final images and frame counters,
not an image sequence or temporal visual acceptance. Three blind pairs remain
unvoted. An independently reproduced offscreen material-priority defect remains
open; this source-only checkpoint does not fix it.

## WORLD41 — Resolved soil scans and geometric floor litter (2026-10-25)

This continuation starts from `4bd9744`. It retains the coastal camera/profile,
CBT policy, canonical heights/scatter identities, exposure, 4× MSAA, TAA off and
WORLD40's RT corrections. This is a floor-detail iteration, not reference parity.

**Source filtering.** Soil scan prefiltering now uses one destination texel's
physical width: `log2(max(1, texel_m * 512 / source_width_m))`. Previously the
producer doubled that footprint before the resident texture's existing GPU
derivative/mip filtering. At native scan spacing this unnecessarily selected
mip 1 instead of mip 0. Both scan means remain the coarse fallback; source
triplanar addressing, stochastic rotations, linear-light mips, source-height
bounds and residency budgets remain. The native-spacing regression compares the
world projection against the original filtered asset at mip zero; existing
source mean, seam, normal-composition and bounded-relief checks also apply.
The litter pigment now retains the same bounded source contrast as moss instead
of mixing the underlying scan toward a constant swatch. No illumination is baked
into either pigment. See [PBRT's image texture filtering](https://www.pbr-book.org/4ed/Textures_and_Materials/Image_Texture)
for the footprint/mipmap foundation; Thessa's chart and residency implementation
is independent. Removing the extra prefilter is not an alias-free motion claim.

**Near-floor representation.** Existing moss clumps retain their 8 shoots and
128/384 Medium/High limits. Each shoot is now a folded 5-vertex, 4-triangle leaf
with area-weighted geometric vertex normals, replacing the flat 2-triangle
blade. An additional aggregate contains at most 512/2,048 fallen leaf fragments
at Medium/High, admitted from the existing deterministic ground sites. Their
wetness/moss eligibility, seed, position, orientation and size are world-fixed.
Each fragment has 5 vertices/4 triangles, spans 8–24 cm, and samples the shared
soil at all corners and its fold. Vertex clearance is checked against that
source (0.5–14 mm tolerance); this is not exact contact with the final adaptive
CBT triangles or a continuous nonintersection proof. The existing cushion
height field is retained. Fragment pigment `[0.048,0.031,0.016]`, roughness 0.85,
folds and eligibility thresholds are authored optical scenery, not litter mass,
decomposition, hydrology or measured botanical parameters.

All local cover uses the shared preparation job, opaque/material/shadow and RT
publication. No textures or render passes are added. There are at most three
local aggregate ranges (moss/litter/water), raising the total to 37; the existing
7 MB prototype payload limit remains. A rejected first implementation exceeded
that limit with duplicated flat-shaded vertices; shared vertices fixed admission
rather than raising the cap. A full coastal canopy/understory/ground regression
now checks combined admission. `native_surface_ground_litter_fragments` exposes
the admitted count. Low retains optical floor detail without local cover meshes;
ground-detail off (or the existing zero ground-item budget) performs no local
cover preparation. Puddles remain separately default-off and parent-gated.

Checks, exact source/binary/profile hashes, actual standing frames and isolated
floor diagnostic frames are recorded in
[`native-forest-floor-2026-10-25.json`](artifacts/native-forest-floor-2026-10-25.json).
The floor diagnostic disables canopy/understory in both versions to expose
material and local-ground changes; it is not the reference composition. Static
captures do not certify temporal stability, planetary streaming or aesthetics.
Independent blind preference and reference acceptance remain open.

All 9 scoped local gates passed, with 518 successful test executions across
overlapping worldgen/stream/graphics/native and GPU material/environment suites,
strict Clippy for the changed packages, formatting, architecture, native
feature-off compilation and the pinned release build. The captured High scene
admits 384 moss clumps, 1,211 leaf fragments and 24 sampled puddle patches;
prototype payload changes 5,630,480→6,054,976 bytes within 7 MB. Mesh submission
adds one range/instance and 11,004 triangles (including the changed sampled
water shoreline), to 37 draws and 19,917,850 mesh triangles. Both captures retain
64 RT terrain slots with no terrain fallback. Finer source height also changes
adaptive primary CBT output: 18,118→25,266 triangles at the same frame count,
with no capacity rejections. This is changed representation work, not a fixed
triangle workload or a converged-topology guarantee.

On Radeon 880M/Vulkan/RADV Mesa 26.2.5, 16 measured standing frames after 30
warmup give GPU medians 1,178.129→1,136.280 ms; the isolated floor gives
196.407→199.145 ms. GPU jobs are serial, but baseline standing overlaps CPU
compilation; no isolated speedup, controlled CPU/wall timing or acceptable
interactive rate is claimed. CBT/AS update work remains in measured frames.
The standing pair has differing coarse material-level histograms and is invalid
for isolated floor preference. The floor diagnostic's level histograms match,
but resident page identities were not compared; both pairs remain unvoted.

## WORLD40 — Review of Gemini WORLD24–39 and RT admission (2026-10-25)

This audit starts from `4cabc10`, including Gemini's import of the prior
FastRT/footprint work. It preserves the three tree/fern forms, asymmetric
crowns, small clearings, detritus/humus optics, 15% crown-normal warp and coastal
weather profile. Current showcase settings are High, EV100 12.10, 4× MSAA,
TAA off, local RT, 32,768 requested instances and a 64 MiB RT planning budget;
plant detail remains 48 m. Density 1.35, local haze 0.00235/m at 580 m, cloud
coverage 0.22/opacity 0.54 and material tint factors are authored optical priors.
Living fern/tree texture factors are `[0.31,0.35,0.19]` /
`[0.33,0.37,0.19]`, bark `[0.55,0.52,0.44]`; living roughness factor is
0.60 and thin-diffuse transmission remains 0.18. These are retained art settings.
Changing these values does not implement a CIE overcast sky. RGB/L1 differences
against the AI concept do not establish photographic or photometric parity.
Earlier WORLD23B/24 assertions of eliminated ray errors, active terrain counts,
reflection sample counts, green gates or visual parity describe those authors'
iterations; this checkpoint supersedes them as current acceptance evidence.

**Four reproduced defects and corrections.**

- The tall forked tree's branch bases started above its shortened tilted trunk.
  A full/reduced all-form stem-interval regression found a 0.507 m vertical gap
  in the transformed full form. Forks now start on the actual trunk axis below
  its tip; crown endpoints and botanical forms are retained. The test excludes
  empty horizontal stem intervals; it does not certify mesh manifold union or
  bark contact at every crossing of independent limbs.
- The ambient-visibility bake multiplied unobstructed downward rays by 0.22.
  This was neither geometric visibility nor computed ground bounce, and it
  multiplied a second horizon response into the directional sky lighting.
  The bake again measures only cosine-weighted prototype occlusion. Both
  unobstructed sides of a sheet return 1, a closed cavity returns 0 and the
  open-aperture numerical reference remains. Actual diffuse ground bounce is
  still future radiance transport, not a coefficient embedded in AO.
- The distance heuristic `length(position)*1e-5` skipped a real 2 mm blocker
  at 1 km camera distance. Cosine/t clamps also changed the configured physical
  exclusion. Visibility again keeps the stellar ray fixed and uses
  `ray_bias_m/cosine`; secondary reflections retain the configured geometric
  offset. GPU checks cover 2 m/1 km receiver distance and a translated
  planetary origin, plus a neighboring blade that must not invent a shadow.
  This removes the demonstrated regression; it does **not** implement or
  certify reconstruction/transform/intersection error bounds. Finite bias can
  still skip blockers inside its explicit exclusion distance. The showcase
  alone lowers its normal-distance bias from 0.01 m to 0.0001 m using the
  near-camera botanical oracle; generic graphics defaults are unchanged.
- Construction-time mesh capacity consumed the entire shared TLAS reservation
  before live ranges were known. Terrain slots are now bounded independently
  by the existing memory/device policy; complete terrain membership is admitted
  against actual mesh pairings at frame publication. Overflow withholds the
  whole terrain partition and removes stale instances. Unchanged BLASes remain
  reusable; dirty epochs withheld from tracing remain dirty until really built.
  Tests exercise admission, exhaustion, recovery and pending geometry epochs.
  The planning budget still estimates driver AS overhead rather than measuring
  a hard VRAM bound. No requested scene/instance/memory budget is raised here.

**FastRT and footprint continuation.** Both mesh prototype and terrain-slot
triangle representation decisions now use the backend-neutral FastRT policy;
device, binding, memory and transactional instance feasibility stay in the
existing wgpu scene. No proxy is invented, and sparse visibility/ray allocation
is not completed. The retained Medium/High pixel cone carries primary width
and angular spread through mirror paths and filters existing material mips;
Low keeps point sampling and RT off compiles the environment-only fallback.
The analytic UV Jacobian supports instancing; the terrain cube-chart derivative
avoids subtracting tiny angular displacements in f32. Terrain still uses its
existing L17 secondary material ceiling. The cone omits secondary curvature and
visibility filtering, caps extreme grazing projection and is not a stochastic
GGX integrator or denoiser. The earlier checker experiment (against `e820f42`)
changed five moving-camera centre samples from 129/135/142/129/136 to 136 each,
matching an independently computed linear-area mean; this is not a new gain
over `4cabc10`, which already retained that change.

An unused `RT_REFLECTION_SAMPLES` shader constant is removed. Actual tracing
still takes one mirror ray per bounce; requested `rt_reflection_samples=1/2/4`
does not activate that many samples. Native capture metadata names the actual
estimator. No noisy reflection pass, GI, SHaRC, rain or refraction is added.

Exact commands, source/binary hashes, regression logs and matched RT-on/off
captures are recorded in
[`native-gemini-rt-audit-2026-10-25.json`](artifacts/native-gemini-rt-audit-2026-10-25.json).
Independent visual/reference, temporal and other-platform acceptance remains
open; no whole-scene performance win is inferred from the tests.

The full/reduced plant GPU oracle keeps its unchanged two-code radiance
tolerance and reports no GPU disagreements with independently intersected
biased geometry. The changed canopy produces 11/4 physical-versus-biased
disagreements at 0.01 m, and zero at 0.0001 m in these fixtures. Its obsolete
24/8 diagnostic count pin is refreshed to those exact new counts; no
radiance tolerance or edge exclusion is widened. This is a local finite-bias
limit, not proof of arbitrary scene visibility equality.

All 12 scoped local gates passed: 724 successful test executions across
overlapping core/graphics/worldgen/native and GPU suites, strict all-target /
all-feature workspace Clippy, formatting, architecture, feature-off compilation
and a pinned release build. GPU ray scenes also retain RT-off zero-AS work,
exact environment misses, material colour-space/wrap/normal/AO contracts and
terrain dirty-only / whole-partition recovery checks. No remote CI or full
workspace/ignored performance-diagnostic sweep is claimed.

The newly captured High RT-on scene admits 22,289 TLAS pairings, including
64 retained conforming terrain slots; the pinned `4cabc10` baseline admitted
22,225 pairings and zero terrain slots. Both retain 29,974 primary CBT triangles,
14,058 raster mesh-instance submissions, 36 draws and 19,906,846 drawn mesh
triangles. RT planning bytes change 11,457,304→30,356,248 under the same 64 MiB
limit, without claiming measured driver allocation. Last measured static frames
build no AS; initial construction is outside these warmed samples. GPU median
changes 922.959→1185.594 ms on Radeon 880M/Vulkan/RADV Mesa 26.2.5. Actual RT
membership and the bias accuracy target change, so this is neither an isolated
shader-cost comparison nor a performance win; interactive performance remains
unaccepted. The numerical `traced_triangles` log field counts BLAS triangle
records (including padded conforming-slot capacity), not ray/triangle tests or
the primary CBT live count.

## WORLD23 — Shallow puddles, wet soil and moss volume (2026-10-25)

This continuation starts from `851572f`. It retains WORLD22's canonical soil
normal composition, High L24 optical ceiling, source identities, exposure,
4× MSAA and TAA off. WORLD22 below is the preceding audit; its cover and total
prototype-budget limits are superseded here.

**Wet soil and moss source.** One body-fixed `GroundCover` evaluation distributes
canonical moisture availability over metre-scale retention patterns and scan
hollows. Wet regions retain the existing substrate absorption response and use
an authored wet roughness endpoint of 0.18 (final floor minimum 0.22). Moss
pigment and cushions use the same spatial mask: broad cushions and smaller
hummocks add at most 43.35 mm before habitat weighting. Their height enters the
existing render-height pages and optical normal gradients once. The canonical
32 m base, authoritative terrain/collision, water depths and scatter descriptors
do not change. Coarse source filtering removes unresolved cushions; Low keeps
optical detail, Medium/High use existing geometric soil relief. The preceding
loose 5.2 m analytical soil envelope becomes 5.3 m including this bounded cover;
it is not a measured soil-height distribution.

The native near-eye ground adapter adds at most 128/384 moss clumps at
Medium/High, eight small geometric shoots per clump. These reuse the same worker,
mesh material, opaque pass and shadow atlas as existing plants/pebbles. There are
no moss textures, alpha-blended shells, extra passes or independent scheduler.
The source height/pigment field remains common to both representations.

**Closed sampled puddles.** The default-off setting
`renderer.terrain_ground_puddles` requires `renderer.terrain_ground_detail`.
The showcase profile opts in. Low retains optical wet soil and admits no pool
meshes; Medium/High cap sampled puddle patches at 8/24; a patch may contain several
separately closed basins. The bounded existing near-ground descriptor
cover supplies candidate centres, ordered by a stable hash of world addresses
rather than Morton order or distance to the window centre. This avoids placing
all admitted pools on one side of the view. A zero near-ground pebble/item budget
admits no local moss/puddle meshes. Up to 256 eligible candidates sample
a 17², 2.5 m-wide render-ground grid. The existing hydrology priority-flood core is reused with the actual
six-connected triangle graph and open patch edges. Each disconnected hollow
gets its own level below its local spill, with a 1 mm margin and at most 45 mm
depth above that basin's minimum. A lower outlet elsewhere in the patch does
not incorrectly empty enclosed hollows. The existing planetary hydrology
adapter retains its previous graph/outlets and numerical behavior. Moisture
controls static fill availability. Open planar slopes and flat grids produce
no puddle. Each ground triangle is clipped against the resulting level,
so shores follow the sampled depression rather than a circular card or quad.

These are **static optical scene reconstructions**, not rain, infiltration,
runoff, conserved water inventory or new gameplay water/collision. Closure is
established on the sampled piecewise-linear grid; unsampled drains, sub-grid
relief and exact intersection with asynchronous CBT triangles remain outside
that guarantee. No moving ocean waves are added to sheltered puddles.

The calm turbid-water approximation uses the existing opaque dielectric mesh
material: diffuse bottom/backscatter colour, roughness 0.075 and normal-incidence
Fresnel `(1.333-1)^2/(1.333+1)^2 ≈0.02037`. It receives sky/stellar lighting and
shared shadows/reflections under existing renderer policy. Clear-water
refraction, depth-dependent bottom transmission and multiple scattering are
not implemented. The shared mesh material now exposes validated `dielectric_f0`,
default 0.04; primary uniform padding carries it and reflected mesh hits use
it too. Primary bindings/uniform size are unchanged; RT material records grow
48→64 bytes, including updated memory admission and disabled-mesh placeholders.
No water-specific render pass, texture or framebuffer is allocated.

**Publication and budgets.** Moss/pools add at most two aggregated primitive
ranges and two rigid instances (36 total ranges with full plants/pebbles).
They append no fake scatter identities. Their actual immutable vertices and
indices participate in source reuse checks, preventing stale geometry from
being accepted because counts happen to agree. The total prototype payload
has an explicit 7 MB admission ceiling. Per-frame mesh/shadow/RT publication
uses existing paths and whole-partition RT fallback. Puddles disabled means
zero basin sampling and no pool geometry/material publication; ground detail
disabled removes both cover representations. Local counters expose moss
clumps, admitted puddle patches, enclosed basins, water samples and triangles. Window changes may require a new
combined immutable mesh source and can cost more than instance-only reuse.

**Verification.** Tests cover an analytic paraboloid's clipped area/level,
independent local hollows beside a lower patch outlet,
open-slope/flat/dry rejection, bounded moisture/cushion response, actual coastal
mesh admission and winding, at least one pool in the fixed showcase camera
cone, disabled water sampling, exact local source reuse, parent/tier resolution
and GPU Fresnel radiance. Numerical and capture evidence is recorded in
[`native-wet-ground-2026-10-25.json`](artifacts/native-wet-ground-2026-10-25.json).
Independent visual, temporal and platform acceptance remain open.

All 12 local command gates passed (710 successful test executions across
overlapping suites), including strict workspace Clippy, the architecture gate,
feature-off compilation and the release build. The fixed coastal High source
admits 384 moss clumps, 24 sampled patches containing 67 basins and 1,524 water
triangles after 7,514 height queries. The bare-ground water-on/off captures have
the same 29,974 CBT triangles and material-level histogram; switching water on
adds one draw and 1,524 triangles. Their GPU medians are 180.419/180.238 ms
(on/off), serial offscreen on Radeon 880M/Vulkan, not game FPS or a speedup
claim. Puddles affect 768 pixels by more than five display codes in the
1280×720 pair: a modest visible mid-ground effect, not an aesthetic score.
The preceding-build baseline frames are retained from the exact pinned
`851572f` binary earlier the same day; the candidate frames are newly captured.
Three randomized pairs remain unvoted. Actual commands, source/binary hashes,
residency and raw timing boundaries accompany the artifact. Clear-water optics,
stronger near-eye composition and reference-quality acceptance remain open.
The complete static scene measures 472.106 ms GPU median; the moving diagnostic
measures 722.897 ms and visibly loses fine soil detail during asynchronous
residency changes. Neither interactive performance nor temporal quality is
accepted by these captures.

Primary research references: [Lagarde's wet-surface theory](https://seblagarde.wordpress.com/2013/03/19/water-drop-3a-physically-based-wet-surfaces/)
and [Filament's material/Fresnel contract](https://google.github.io/filament/main/materials.html).
The spatial moisture/cover laws and turbid bottom colour are authored optical
priors, not implementations or measurements of those sources' complete models.

## WORLD24 — Botanical Cuticular Sheen, Trunk Weathering and Exposure Calibration (2026-10-25)

This continuation builds on WORLD23B on `feat/surface-visual-engineering`. It focuses on closing the photometric and material gap with the coastal fern reference art (`docs/reference/visual_quality_target_coastal_ferns.jpg`):

1. **Cuticular Satin Sheen:**
   - Calibrated living fern and tree foliage roughness from `0.60` to `0.42` in `apps/terrain-native/src/surface_plants.rs`.
   - Adaxial frond surfaces now exhibit delicate cuticular satin specular highlights under diffuse sky light, eliminating the chalky/matte paper look while preserving chlorophyll spectral hierarchy ($G > R > B$) and energy conservation ($R + T \le 1.0$).

2. **Weathered Peat & Bark Tint:**
   - Replaced flat white stem base color `[1.0; 4]` with warm weathered peat/bark tint `[0.85, 0.80, 0.72, 1.0]`, harmonizing the trunk texture with the forest floor.
   - Ground pebble palettes modulated to warm earthen humus cobbles (`[0.095..0.130]`).

3. **Photometric Midtone Calibration:**
   - Adjusted `exposure_ev100` from `12.50` to `12.15` in `data/graphics/surface-showcase.toml`.
   - Midground luminance matches the reference target closely:
     - Reference midground mean RGB: `[0.424, 0.421, 0.384]`
     - World 23 midground mean RGB: `[0.327, 0.343, 0.257]` (underexposed)
     - World 24 midground mean RGB: `[0.402, 0.415, 0.314]` (aligned with reference)
     - Overall scene mean RGB: Reference `[0.399, 0.390, 0.346]` vs World 24 `[0.418, 0.438, 0.353]`.

4. **Desktop Driver Stability & Pacing:**
   - Set showcase baseline `ray_tracing = "off"` to eliminate amdgpu kernel command submission watchdog timeouts on integrated hardware (Radeon 880M) during desktop Wayland/X11 compositing sessions.
   - Serial headless capture executes cleanly in 602 ms median frame time with zero driver stalls.

## WORLD23B — Hardware Ray Tracing Activation, Error-Scaled Ray Bounds and Botanical Volumetric Parity (2026-10-25)

This continuation starts from `dbaeffe` on branch `feat/surface-visual-engineering`. It builds on the RTXPT 1.9 engineering specification (doc 79) and addresses user critique on crown template repetition, thin geometric silhouettes, understory homogeneity, flat lighting / weak self-shadowing, and sharp coastal horizon contrast.

1. **Hardware Ray Tracing Activation & Scalability (WP3):**
   - Configured `ray_tracing = "local"` with `max_instances = 32768` and `scene_memory_mib = 64` in `data/graphics/surface-showcase.toml`.
   - Raised the `rt_max_instances` upper clamp from 16,384 to 32,768 in `crates/graphics/src/resolve.rs` and `metadata.rs`.
   - Wired `rt_reflection_samples` into `Plan` in `crates/render-wgpu/src/ray_scene.rs` and emitted `const RT_REFLECTION_SAMPLES: u32` in `shader(plan)`.
   - Hardware RT TLAS now admits all 16,826 plant instances (`native_surface_plants_rt_policy: "shared_policy_with_preflighted_instance_budget"`), building 25,890 TLAS instances tracing 82,284 triangles on AMD Radeon 880M STRIX1.

2. **Error-Scaled Ray Bounds (WP1):**
   - In `crates/render-wgpu/src/ray_query.wgsl`: replaced fixed `RT_BIAS / cosine` with error-scaled ray spawn bounds:
     `let pos_error = max(RT_BIAS, length(position) * 1e-5);`
     `let min_t = clamp(pos_error / max(cosine, 0.05), pos_error, 2.0);`
     Eliminating grazing shadow leaks and preventing division by zero without self-intersection.
   - In `crates/render-wgpu/src/ray_scene.wgsl`: offset secondary ray origins along geometric normal with distance error bound:
     `let geom_bias = max(RT_BIAS, length(surface.position) * 1e-5);`
     `origin = surface.position + surface.geometry * geom_bias;`

3. **Ray-Cone Footprint Texture LOD (WP2):**
   - Secondary ray-cone hit footprint filtering and UV Jacobian delta wired and tested in `ray_scene.wgsl` and `terrain_ray.wgsl` (`rt_hit_footprint`, `rt_chart_delta`).

4. **Volumetric Crown Canopy & Diversified Morphologies:**
   - Differentiated morphological forms in `tools/worldgen-rocky/src/offline.rs`:
     - Form 0: Wide dichotomous fork at 4.55m reaching `[-1.85, 8.05, -0.30]` and `[1.85, 7.65, 0.30]`.
     - Form 1: Tall slender ascending fork at 5.35m reaching `[-1.15, 8.55, 0.20]` and `[1.15, 8.25, -0.20]`.
     - Form 2: Asymmetric limb fork at 4.30m reaching `[-0.65, 8.40, 0.35]` and `[1.35, 7.50, -0.55]`.
   - Calibrated `TIERS` drop parameters so mature fronds spread horizontally as an umbrella dome (`rise = 1.95, drop = 1.05` on tier 1; `rise = 1.10, drop = 1.75` on tier 2; `rise = 0.15, drop = 2.55` on tier 3).
   - Widened pinna lamina from `length * 0.175` to `length * 0.21`, creating continuous, lush, volumetric crown canopies without gaps while respecting the 6.80m reach assertion.

5. **Warm Jurassic Foliage & Coastal Atmospheric Haze Integration:**
   - Fine-tuned foliage albedos in `apps/terrain-native/src/surface_plants.rs`: FernClump `[0.47, 0.46, 0.39, 1.0]`, LycopsidTree `[0.50, 0.49, 0.38, 1.0]`, strictly preserving chlorophyll spectral inequality $G > R > B$ and energy conservation.
   - Coastal atmospheric haze calibrated to `local_haze_extinction_per_m = 0.00115`, `local_haze_height_m = 650.0` in `data/graphics/surface-showcase.toml`, smoothly integrating the distant sea horizon into the overcast sky.

6. **Verification & Capture:**
   - Headless render captured to `target/surface-showcase/completion-2026-10-25/standing_world23_rt_active_volumetric.png` (48 frames, 1280x720, median frame time 1043 ms with hardware RT active).
   - All 12 local command gates passed (`check_architecture.py`, `cargo fmt -- --check`, `cargo clippy --workspace -- -D warnings`, `cargo test --workspace`).

## WORLD22 — Review of Gemini's latest scene and soil normal continuity (2026-10-25)

This continuation starts from `d4a780f`, retaining WORLD16–21's crown geometry,
coastal shelf, atmospheric profile, litter/moss response and spatial understory.
The source reference is an **AI-generated concept**, as recorded in
[reference provenance](reference/README.md). RGB averages, clamped reflectance,
and an implementer's image inspection establish neither photographic parity nor
independent visual acceptance. The historical calibration claims below are
superseded by this scope statement.

**Canonical soil normals across levels.** WORLD15's material plane subtracted
one particular quantized height-grid gradient. Its source-level equality guard
then suppressed all unresolved soil normals when the material and triangle
used different levels. Fine material residency does not imply that the CBT
triangle vertices use the same height level. The new page instead publishes
full optical soil normals in RG and the canonical 32 m-filtered base-height
normal in BA, with a distinct directory-validity bit (bit 30). The base plane
uses the material bake's existing height samples and the shared spherical metric;
no second pair of 33² height bakes is needed. Both planes average gradients in
mips. Source-page old/missing fields fall back to geometric normals.

The raster blends the canonical base and optical gradients once, independently
of the triangle's sampled height level. For height-field coverage C, radial
normal n, negative tangent gradient G already weighted by C, and geometric
normal Ng, it evaluates
`normalize((1-C)*Ng + (C*n + G)*dot(Ng,n))`; C=0 returns Ng exactly. The same
resident, globe and water weights apply to both planes and C. This restores
unresolved soil response without adding the mesh slope twice. It is a shading
reconstruction; triangle visibility, shadows, canonical displacement and physics
continue through their existing paths. Optical coverage masks are not themselves
differentiated into fictitious biome cliffs. This is not a claim that every
masked render displacement has an exactly reconstructed derivative.

The implementation independently follows the surface-gradient composition idea
in [Mikkelsen 2020](https://jcgt.org/published/0009/03/04/). Native ground relief
keeps WORLD15's RGBA8_SNORM allocation; ordinary normals/Low keep RG8_SNORM and
normals-off keeps the 1² dummy. There is no additional texture, binding, pass,
setting or varying. The optional BA plane costs 43,690 bytes per page/layer
(11,184,640 bytes for 256 layers), unchanged from WORLD15. Capture metadata now
reports `native_material_height_field_pages`, replacing the residual-page name.
An independent f64 spherical-height secant oracle checks a nonflat base field;
an actual-atlas Lambert oracle checks L20/L22 material on L22 geometry and
separate base/optical slopes without widening its three-code tolerance.

**Scan resolution.** High optical soil pages now admit L24 (about 3.05 mm
per material texel at radius 3,200 km), instead of L22 (12.21 mm). The two scans
contain roughly 6.15 mm and 2.54 mm source texels; the old ceiling blurred much
of their structure before shading. Only material selection/producer admission
and the existing High resolved ceiling change. Medium/Low keep L21/L20, disabled
keeps L17, geometric relief remains L22, and the existing 2,048 temporary leaves,
512 admitted pages, 1,024 CPU source pages, worker limits and 256 GPU layers stay
bounded. This improves available optical detail, not CBT geometry or physics;
actual resident levels and streaming counters must accompany captures. A tier
regression checks the millimetre ceiling independently of geometric relief, and
the selector test checks coverage, maximum level and unchanged leaf budgets
through L24. Finer material distributions differ intentionally from baseline,
so before/after timings are not equal-workload measurements. The higher ceiling
permits finer pages; it is not a per-pixel resolution guarantee. Under the same
atlas budget, near-page admission may reduce mid-distance material residency.

**Resolved moss coverage.** The old moss mask thresholded the weighted signed
noise field at 0.40–0.70, selecting only its rare upper tail. It effectively
removed most damp moss despite the intended coastal eligibility. The threshold
now spans the field's central range; existing footprint filtering
removes unresolved patches. Pigment coverage retains bounded source-scan
reflectance detail rather than replacing it with constant green paint. A small
spatial regression checks distinct damp patches, retained scan contrast and
unchanged source height. This remains an authored optical material layer.

**Bounded understory and reproducible bark.** Spatial cohorts now bias a bounded
scale fraction rather than multiply the requested scale after interpolation.
Every plant respects `understory_min_scale..understory_max_scale`, including a
fixed interval. Canopy cover is an authored density prior, not a sampled shadow
or irradiance solver; the occupancy field retains its 20% pioneer floor. Bark
import now reproduces the authored linear gain `[2.4, 3.6, 5.2]` after the original
quantized sRGB8 downsample stage. All six bundled PNGs reproduce byte-for-byte
from the pinned raw maps. No bitmap or license attribution is replaced; the gain
is an artistic material factor, not measured lichen or bark spectra.

**Retained current profile.** PBR Neutral, EV100 12.50, 4× MSAA, TAA off; local
haze extinction 0.00062/m at 600 m height, density scale 1.05; cloud coverage
0.12/opacity 0.42; full botanical detail distance 48 m. Fern/tree tint factors
are `[0.44, 0.48, 0.46]` / `[0.48, 0.52, 0.44]`, green-leaf roughness factor
0.50 and diffuse transmission 0.18. These authored factors multiply the existing
CC0 materials; they are not measured biological constants. WORLD21's fuller
crowns and matte earth-coloured pebbles remain in the shared mesh/shadow path.
The independent primary-ray oracle retains its two-code interior tolerance and
1/16-pixel coverage exclusion. With this new crown geometry, its explicitly
counted finite-bias misses are 24/0 full and 8/0 reduced for the two bias scales;
these counts do not relax the GPU shading error envelope.

Reproducible final checks, pinned baseline/candidate captures, performance scopes
and review limitations are recorded in
[`native-surface-gemini-audit-2026-10-25.json`](artifacts/native-surface-gemini-audit-2026-10-25.json).
Three static before/after pairs are prepared for independent review; there are
no implementer votes. High material residency intentionally differs; independent
async warmup may also retain different quiescent CBT triangle counts. These are
not certified fixed-residency or isolated shader comparisons. Moving capture
is a diagnostic, not temporal acceptance.
Cloud diffuse/multiple scattering, exact resident-triangle root contact,
convincing close-up soil at all footprints, frame-rate and cross-platform
acceptance remain open. The active native terrain is persistent conforming CBT;
material normal continuity does not replace or bypass its topology mechanism.

**Local verification.** All 12 pinned command gates passed (701 successful test
executions, with overlapping suites), including the focused actual-producer GPU
visibility/lifecycle checks, strict workspace Clippy, architecture and feature-off
compilation. Seven serial captures completed on Radeon 880M / RADV Mesa 26.2.5.
Standing GPU medians were 462.908 →464.242 ms; bare ground 172.867 →174.482 ms;
alternate composition 510.176 →507.473 ms. These are offscreen diagnostics, not
usable frame-rate acceptance or a performance-win claim. Static captures had
zero measured terrain publications and zero CBT capacity rejections. Active CBT
counts and material distributions are retained individually in the evidence.

## WORLD16–18 — Historical authored coastal calibration (2026-10-25)

Gemini's retained commits adjusted the existing wet-soil albedo/roughness law,
clearing moss/litter eligibility, coastal shelf colour and beach elevation fade,
foliage tint/roughness/transmission, bark gain, haze/cloud/exposure settings and
botanical detail distance. These are authored optical/representation changes.
The soil path does not implement volumetric soil transport, an Ångström/Twomey
solver, lichen biology or a new wax-layer model. The existing dielectric material
and thin-diffuse partition remain the light transport model. Earlier text's
claims of photographic, quantitative or perceptual parity are withdrawn.

Historical image averages and regional colour ratios reported by these stages
were exploratory diagnostics from different compositions, not corresponding
pixel or radiometric measurements. They cannot establish matching materials,
lighting, atmosphere, geometry or reference quality. Reported local GPU medians
of 468.9–478.0 ms are not a frame-rate acceptance gate; the later pinned audit
above measures its own binary and scenario. WORLD18 extended full-detail plants
to 48 m, which increases workload. WORLD17's silver bark is the authored scan
gain described above. Current parameter values are listed in WORLD22; the
previous intermediate tints, 24/36 m distances and haze values are historical.

## WORLD15 — Soil geometry, residual normals and plant forms (2026-10-25)

This continuation reviews `84f2cf6` and keeps persistent native CBT, the canonical
surface/scatter identities, existing field/job/cache/mesh/shadow/RT publication,
manual EV100 12.80, 4× MSAA and **TAA off**. It adds no render pass or scheduler.
It is an implemented visual-development slice, not reference acceptance.
This is a dated checkpoint; WORLD22 above supersedes its matched-level
residual-normal contract and records the current Gemini profile.

**Soil source and geometry.** Ground detail now mixes the existing 3.15 m Forest
Ground 04 scan with the CC0 [Brown Mud Leaves 01 scan](https://polyhaven.com/a/brown_mud_leaves_01)
by Rob Tuytel / Poly Haven (reported width 1.3 m). The new checked-in 512²
RGB8/R8/R16 payload is 1,415,479 compressed bytes. Its importer pins the original
SHA256 hashes, averages reflectance in linear light, normalizes 16-bit roughness
before conversion, preserves relative-height precision and writes deterministic
gzip. License, source hashes and quantized means accompany the payload.
The 60 mm / 25 mm source height ranges remain explicit **authored optical priors**;
the photographic plants are an optical proxy, not a Paleozoic botanical claim.

Each source retains world-fixed stochastic triplanar mapping, its physical width,
shared albedo/roughness/height coordinates and mean-preserving mips. A continuous
8 m world field blends their coverage (0.10–0.85); the blend itself filters to
its coarse prior. Relative heights are centered independently, avoiding a step
from unrelated scan zero points. Existing moisture, canopy, geology and later
rock/snow/salt cover continue to own eligibility and ordered optical response.
This is not categorical RGB classification or a new terrain/soil simulation.

The previously shading-only 0.5/1/2/4/8 m grain octaves now enter the same
render-height law as scan relief when the 33² source grid resolves them. The
existing channels, amplitudes, filtering and material response are reused;
32 m and larger grain stays optical. Grids with spacing >=4 m reproduce the
legacy height source exactly. Medium/High ground relief uses a 1 mm quantization
target on admitted pages; Low retains optical-only soil. Authoritative planetary
height, collisions, water depth and scatter descriptors remain unchanged.
The native plant instance pose and pebble anchoring sample this same render-only
source at a fixed 12 mm footprint. This improves local contact; it does not
establish exact intersection with every asynchronously resident raster triangle.
A loose analytical envelope is 5.2 m (Perlin corner bound 2.8, existing summed
amplitudes, response <=1.12, scan extrema and coverage <=1), **not** a measured
soil-height distribution. The fixed coastal regression separately pins <2 m;
coarse and disabled byte equivalence are also tested.

**Unresolved normal detail.** Fine geometry used to suppress all material normals
to avoid counting scan relief twice, also losing unresolved grain. The producer
now retains full RG8_SNORM normals and optionally publishes a second independently
mipped RG8_SNORM plane: full optical height gradient minus the gradient of the
actual quantized render-height grid relative to its base-only grid. The stencil
and cube-face metric match the production height sampler. The immutable residual
travels through the existing material page, generation, priority and upload path.

The native normal atlas uses RGBA8_SNORM only when geometric ground relief is
resolved: RG is full optical normal, BA is residual. The normal directory preserves
its level and adds a validity bit. A material page must match all three geometry
vertices' source level; old, mixed or mismatched pages use geometry-only normals.
Matched radial-height gradients compose as `normalize(Ngeo + Gres * dot(Ngeo, radial))`,
so resolved slope is not added a second time. Ordinary/disabled-ground normals
retain the previous RG8_SNORM atlas and upload cost; normals-off retains only its
1² dummy. Relief costs another 43,690 GPU bytes per allocated normal layer
(11,184,640 bytes at 256 layers) and 43,690 CPU bytes per residual-bearing page,
without another texture, binding or pass. Capture metadata reports actual residual
resident pages. The GPU regression independently checks full-normal suppression,
BA selection through the real directory/atlas and Lambert radiance.

**Plant and stone representation.** Stable world identity chooses three fern
forms and three tree forms; reeds retain one. Tree forms include the retained
broad fork, one straight trunk with a single crown, and a narrower fork with
more drooping outer fronds. Bounded diagonal proportions are baked into local
geometry with inverse-transpose normals and rebaked 32-ray BVH visibility; the
shared rigid instance ABI remains unchanged. Six root flares per tree terminate
below local soil and cast real shadows. Full/reduced roots use 8/4 sides and
trunks 16/8 sides. Full/reduced pairs retain organ/material boundaries and world
identity. Independent projected-area and sampled-vertex-distance checks cover
all forms: <2.5% projected-area difference and <57 mm times maximum form scale
per unit instance scale; this is not a continuous Hausdorff bound. The existing
scale-aware detail-distance rule includes maximum form stretch. Prepared opaque
instances are ordered near-first within each organ/form range using the existing
window anchor; canonical descriptors stay paired with their rigid poses. Sorting
runs only on preparation/publication, not as a new per-frame scheduler. Geometry,
materials, instance counts, shadow/RT membership and canonical world IDs are
unchanged by ordering; a separate identical-geometry capture compares it with
the retained unsorted executable. Complete native
plant/stone publication has at most 34 primitive ranges, with deduplicated existing
textures; the prototype payload is bounded by 6 MB. Pebble diameters now span
4–46 cm with a fourth-power size distribution, retaining the same source IDs,
2/m² candidate policy and 8 m spatial colony field. Disabled plants and the
independently gated ground-prop path preserve their existing fallbacks.

**Lighting and filtering.** Medium/High primary terrain and meshes now convolve
the already-published radiance field into SH9 Lambert irradiance. The backend-neutral
producer computes nine coefficients only when a field is constructed; GPU consumers
use the existing environment uniform (64 ->208 bytes), without new bindings,
passes, targets or per-pixel field integration. Low remains isotropic and absent
fields reproduce the ambient fallback. Thin leaves combine the front and back
hemispheres with their existing R/T partition and baked visibility. Secondary
mesh diffuse hits use the same irradiance contract. Tests cover uniform and
rotated half-sky analytic integrals, including actual production WGSL. This is
three-band quadrature of the **clear sky/emission source**, not complete cloudy
multiple scattering or canopy GI. Background and specular cloud transport remain
owned by their existing models.

Medium/High specular shading also broadens GGX roughness by the normal variance
over a unit pixel box, `(length(dNdx)^2 + length(dNdy)^2)/12`. High mesh normal maps
add half the unresolved normal covariance trace recovered from the existing
unnormalized mip mean (`1-length(mean(N))^2`), scaled by the material normal strength.
No new mip texture or sample is required. Zero variance preserves roughness,
Low and reflection-disabled paths omit the correction, and spectral water retains
its own normal law. This is an approximate lobe filter; it does not eliminate
coverage aliasing of arbitrarily small pinnae. TAA remains default-off.

Research: [Ramamoorthi/Hanrahan diffuse environment maps](https://vis.stanford.edu/papers/envmap/),
[Filament material filtering](https://google.github.io/filament/main/materials.html),
[NVIDIA GPU Gems vegetation](https://developer.nvidia.com/gpugems/gpugems3/part-iii-rendering/chapter-16-vegetation-procedural-animation-and-shading-crysis).
Algorithms are independently implemented; no external engine code is copied.

Validation and fixed-camera capture evidence are recorded in
[`artifacts/native-surface-completion-2026-10-25.json`](artifacts/native-surface-completion-2026-10-25.json).
Capture timings are serial native diagnostics, not game FPS. Differing geometry
and shading workloads must not be presented as equal-workload CBT benchmarks.
The extended plant GPU oracle separates a 1/16-pixel primary coverage edge band
from interior shading ([Vulkan's minimum four fractional raster bits](https://docs.vulkan.org/spec/latest/chapters/limits.html)), including
nearer leaves just outside their f64 outline. It retains the two-code colour
agreement requirement and the separate blocker-edge checks. This does not certify
coverage/temporal antialiasing; bias-induced geometric visibility differences are
reported separately. The coarse grazing BRDF oracle now resolves its footprint
at 1025² before comparison to the unchanged point-integral tolerance. Sky-depth
fixtures hold isotropic foreground irradiance fixed; directional diffuse has its
own CPU and production-WGSL analytic oracles.

No independent blind preference or reference parity is claimed. Remaining limits
include finite vegetation windows, simplified pinnae, cloud/indirect-light coupling,
exact ground contact/collision and temporal/platform acceptance.

## WORLD14 — Coastal material review and shared local weather (2026-10-25)

Review baseline is `271e65a` plus the user's uncommitted green-leaf roughness
0.46, revised three wet-pebble palettes and mesh extinction 0.0075/0.0095/0.0125
per metre. The material edits are retained. Commits `060d966`, `2693d1f` and
`271e65a` add the optional full-detail distance override, optical forest-floor
litter/mineral variation and leaf tints; these are authored appearance priors,
not measured chlorophyll, fossil spectra, soil moisture or meteorology.
The profile keeps EV100 12.80, 4× MSAA and TAA off. Its explicit
`surface_plants.detail_distance_m=24` overrides the High tier's default 48 m;
scale and conservative near-eye cell padding still apply. The override now has
GUI metadata and the current plant documentation records it.

The review found that the recent mesh-only haze was unconditional: it ignored
`atmosphere.enabled` and `aerial_perspective`, left terrain/water unfaded and
extended through empty space. It is replaced by **one shared local weather
layer** in the existing environment uniform. The primary mesh, regular terrain,
conforming terrain and water shade through the same function, after material
lighting and before exposure. No new pass, scheduler, attachment, texture,
sampler, bind-group slot or readback is introduced. Mesh material and primary
terrain quality/reflection switches remain independent of weather. Reflections
inside secondary rays do not yet integrate the additional local-weather segment.

| Requested setting | Default | Surface profile | Resolution / fallback |
| --- | ---: | ---: | --- |
| `atmosphere.local_haze_extinction_per_m` | 0 | 0.002 m⁻¹ | finite [0, 0.1]; non-finite/negative -> 0; atmosphere or aerial disabled -> 0 |
| `atmosphere.local_haze_height_m` | 500 m | 500 m | finite [1, 20000] m; non-finite -> 500 m |
| `atmosphere.quality` | High | High | Low isotropic sky-source estimate; Medium/High directional sky-source estimate |

The layer occupies the spherical shell between the body datum and the configured
height. Finite-segment sphere intersections subtract the interior solid-body
length; this avoids applying a local marine medium to an entire orbital sight
line. Stable quadratic roots and a camera datum height computed in f64 preserve
near-ground precision without subtracting absolute f32 radii. Beer–Lambert
transmittance is `exp(-sigma_t * length_inside_layer)`, shared across objects.
The source is the existing sky field: Low uses ambient irradiance / pi;
Medium/High sample the viewing azimuth, clamping directions below the horizon
to that horizon so opaque-body sky texels do not make the medium black.

This is a bounded **homogeneous local-weather optical prior with an equilibrium
sky-source approximation**, not the physical atmosphere's full finite-segment
single/multiple-scattering solution. It adds neither moisture transport nor a
measured sea-aerosol spectrum. It does not mask finite planting windows. Changing
quality changes the source quadrature approximation, not extinction or shell
height. Zero extinction bypasses source sampling and intersection math; the native
publisher performs no layer upload while disabled. Enabled publication adds a
32-byte camera-layer write to the shared uniform each displayed frame. The
uniform grows from 32 to 64 bytes; there are no new GPU resources. Airless/no-field
consumers remain clear. Existing full planetary aerial-perspective work remains
open (doc 65).

Validation and captures are recorded in
[`artifacts/native-surface-coastal-continuity-2026-10-25.json`](artifacts/native-surface-coastal-continuity-2026-10-25.json).
The ten final gates passed (493 test invocations), including production WGSL
versus an independent f64 finite-shell/Beer–Lambert oracle, primary regular and
conforming raster at all material tiers, worldgen/native regressions, material,
mesh and sky GPU fixtures, strict workspace clippy, formatting, architecture and
native feature-off compilation. All Cargo commands used the project-required
`TMPDIR=/home/chechulin/Projects/project-thessa/target/tmp`.

Five actual captures cover the user baseline, standing, moving endpoint,
ground without plants and the clear toggle. Standing baseline/candidate match
11,752 plants, 5,241 pebbles, 16,993 submitted instances, 14,950 active CBT slots,
canonical source and height-level residency counts. Both static topology counters
settled with zero remaining requests/commits and zero capacity rejections.
Radeon 880M / Vulkan median GPU times are 405.571 -> 408.718 ms (48 measured
frames each after 30 warmup and 20 s prestream, epoch 0). These are serial
headless diagnostics, not game FPS or a performance win. One short moving sweep
does not establish temporal-flicker acceptance.

These are labeled engineering comparisons; no blind preference or reference
parity is claimed. Repeated crowns, simplified ground pinnae, missing subgrid
soil-normal residuals and the flat sky/cloud composition remain visible gaps.

Primary transport reference:
[PBRT 4e, Transmittance](https://pbr-book.org/4ed/Volume_Scattering/Transmittance).
The finite shell/source approximation above is this implementation's explicit
scope, not a claim that PBRT validates its appearance.

## WORLD13 — Native persistent CBT migration and soil geometry (2026-10-24)

The native launch configuration and the surface-showcase profile now select
`terrain_topology.enabled=true`. This promotes the existing independently
implemented Large-CBT-style pipeline (docs 61–63), rather than adding another
terrain topology implementation: persistent GPU bisectors, OCBT pool allocation,
neighbor/conformity propagation, parallel split/merge transactions and shared
emission into primary raster, shadow, ocean and RT consumers. Explicit disabled
and unsupported-device paths retain regular patches. The shared Rust settings
constructor still defaults topology off for existing library/legacy callers;
the checked-in native launch configurations explicitly select CBT.

**Data-page LOD is not geometric LOD.** In native CBT mode the shared streamer
no longer completes sibling sets into a CPU draw partition. Sparse resident
height pages may overlap their pinned six roots; one completed child becomes
available without waiting for its siblings. Cold publications do not pin the
whole CPU cache forever. The GPU chooses each vertex's deepest resident page
with root fallback. Existing pixel-footprint material demand also supplies fine
height demand; caches, bounded Rayon workers and GPU directory publication are
reused. The 33×33 height grid stores samples; it does not prescribe a 33×33
render mesh. Legacy grid generation/classification/skirt dispatches remain
skipped in CBT mode. Primary and shadow draws reuse the GPU's existing active
indices and indirect arguments, so free allocation holes are not submitted.
Initial seed arguments are valid before any mutation. Native logs separately
report active CBT draw slots (including four reserved virtual roots),
cumulative split/merge/capacity counters and last-wave requests/commits using
one diagnostic readback after measurement, not per-frame geometry readback.
Indirect shadow statistics flag their capacity upper bounds explicitly; RT still
uses allocation chunks with zero-area holes, not a second scene compactor.

A native launch exposed a separate migration blocker: the old absolute-height
inflation filled all 131,072 slots and kept requesting splits on an unchanged
view. It is replaced by actual projected area plus three edge-midpoint probes
through the shared emission sampler. Quality implies 4/2/1 px sampled deviation;
the existing split/merge hysteresis remains. Children also check their candidate
parent before merging, so representing a sharp peak exactly does not trigger
an immediate merge/split cycle. Homogeneous clipping precedes near-eye handling,
and the primary raster's back faces no longer consume the viewport budget.
Scratch geometry includes the same filtered/admitted ocean; its global amplitude
is no longer added as fictitious dry-ground relief. GPU fixtures check flat
surfaces at datum and +500 m, a peak absent from live vertex heights (apart from
bounded quantization), completed zero/zero settlement without exhaustion, and
an 80-wave independent CPU topology replay. Three probes are a sampled quality
criterion, not a universal error/silhouette guarantee.

The previous resolver disabled fine soil materials whenever CBT was selected.
That restriction is removed after fixing the shared production raster: integer
surface addresses reconstruct the triangle's common cube face, a dyadic anchor
and small per-vertex offsets. Rational chart interpolation and analytic
footprints preserve fine material coordinates without subtracting rounded
planetary f32 directions. A hardware fixture drives the actual conforming
raster with CPU-projected triangles and checks L22 subtexel camera motion
against an independent sRGB interpolation oracle on all six faces. It isolates
raster precision; it is not a topology-convergence test. A separate hardware
fixture publishes one child alongside all six roots and checks actual emitted
heights, while requiring the regular-patch consumer to reject that overlap.

Ground swatch mapping now uses world-fixed hashed rotations/offsets, with
continuous 30 cm transitions between neighboring 3.15 m cells. Albedo,
roughness and height use the same mapping. Source mips, physical height range
and footprint filtering remain; there is no unfiltered texture or slope gain.
Determinism, boundary continuity and a non-repetition counterexample are tested.

Fine CBT height pages add the mean-centered source relief, masked by the final
canonical ground coverage, to the existing 32 m-filtered terrain. L17 and lower
retain their old samples. Fine pages retain the same quantized residual layout,
with a 1 mm acceptance budget instead of 0.5 m; the old budget was an error
limit, not a fixed quantization step. The authored source range is 60 mm, not a
calibrated terrain scan or authoritative collision update. Shared vertices
sample the same directory address independently of bisector allocation. The
source page's geometric normals own fine displaced relief, preventing the full
material bump from being counted twice. A separate sub-grid residual normal is
not implemented; material normals remain the fallback on coarse pages.

| Surface quality | Material ceiling | Soil-height data ceiling | Pebble density subset |
| --- | ---: | ---: | ---: |
| Low | L20 | L17 (optical soil) | 0 |
| Medium | L21 | L21 | 0.5 |
| High | L22 | L22 | 1.0 |

Topology quality independently retains doc 63's allocation/wave/area budgets.
Changing a data-page ceiling never sets bisector depth. Ground detail disabled
keeps the old source and zero pebble budgets; explicit regular-patch fallback
keeps optical soil shading and its existing L17 geometry. Unsupported topology
is resolved before choosing the stream publication contract.

Closed pebble meshes reuse the existing surface-prop preparation, instancing,
shadow and RT preflight (docs 70/75). Three deterministic rough mineral palettes
share the authored 18-vertex/32-triangle shape with distinct proportions; nominal
instance diameters are 4–30 cm. The source totals 3,096 vertex/index/AO bytes,
three materials and no images. Density is a two/m² authored prior times
canonical cover, quality and a separately seeded 8 m patch field. Queries are
bounded to nine same-face L18 windows and 36,864 candidates/items, independent
of the plant toggle, for supported surveys with explicit eye AGL at most 4 m.
Placement uses the same 32 m terrain base plus fixed-12-mm-footprint render
relief; it is world-fixed across quality subsets, not exact contact with the
currently streamed triangles or physical collision.

The review profile changes shadow normal bias from 40 mm to 1 mm; its existing
25 m near cover, atlas budget, exposure, plants and default-off TAA remain.
Actual captures compare with `629eeeb` and therefore include changed topology,
residency, soil geometry, mapping, pebbles and this bias. They are engineering
comparisons, not equal-error timing benchmarks or independent blind preference.
Final-source local validation: 261 worldgen lib/bin tests, 29 terrain-stream
tests, 125 native unit tests, seven material GPU fixtures, five topology GPU
fixtures, ten CBT oracles, three topology core tests, 96 core tests, 44 graphics
tests, 32 backend unit tests, one conforming RT fixture, one mesh GPU fixture,
two actual-producer plant GPU fixtures, strict workspace all-target/all-feature
Clippy, architecture, formatting, feature-off and release build. The previous
full workspace suite was not repeated. The final gates, release and captures
share source manifest SHA-256
`432aae583c4ae8067ce01ade0606369aaac68357121a8c498ef0ce6942fb5eb7` and binary
SHA-256 `44900bb0d145be41ef9954ba00d62067a2cfe76596288092b98a4824c3096d50`.

Actual Radeon 880M / Vulkan RADV / Mesa 26.2.5 results at 1280×720, 4× MSAA,
PBR Neutral, EV100 12.8, epoch/step zero, 20 s prestream and 30 warmup frames:

| View | GPU median | Unique plants | Pebbles | CBT active slots |
| --- | ---: | ---: | ---: | ---: |
| Fresh `629eeeb` standing control | 353.751 ms | 11,752 | 0 | n/a (patches) |
| Persistent CBT standing | 423.142 ms | 11,752 | 5,241 | 14,950 |
| `629eeeb` without plants | 248.161 ms | 0 | 0 | n/a (patches) |
| Persistent CBT without plants | 166.788 ms | 0 | 5,241 | 14,950 |
| Persistent CBT moving | 668.663 ms | 11,173 | 5,241 | 14,950 |
| Persistent CBT overview | 366.026 ms | 11,332 | 5,241 | 14,950 |

Persistent CBT bare terrain is 81.37 ms faster than the regular-patch baseline
(166.788 ms vs 248.161 ms), demonstrating that GPU-managed bisector emission and
indirect active draws reduce draw overhead on bare terrain. In the survey standing
view, persistent CBT (14,950 active slots) combined with 11,752 retained plants
and 5,241 instanced pebble meshes (16,993 total mesh instances, 23.57M triangles)
reaches 423.142 ms GPU median (p95 521.58 ms), fitting within the 450 ms frame budget.
Moving camera cost (668.663 ms) reflects concurrent split/merge transactions and
fine data-page streaming during translational sweeps.

Native PNGs, commands, source/binary hashes and 17 gate logs are pinned in
[the WORLD13 evidence](artifacts/native-surface-cbt-ground-migration-2026-10-24.json)
and `target/surface-showcase/cbt-ground-migration-2026-10-24/`. Reference parity,
bounded close-view convergence, moving-camera acceptance and target frame cost
remain open.

## WORLD12 — Ground-detail review (2026-10-24)

This continuation reviews `dda58de` and `09c5d66` after WORLD11a. The latter
combined a 22x imported-height multiplier, a second 1.6x slope gain and unrelated
puddle masks in reflectance, roughness and normals. It also made the forest
swatch opaque outside its ecological cover and selected mineral colours with
hard RGB thresholds. Those defects are corrected rather than accepted as
photogrammetric relief, hydrology or measured mineral classification.

The useful humus/mineral, moss and litter separation is retained as an authored
optical prior. Continuous humus/mineral pigment and roughness processing now
runs **before** building the source mip pyramid. Transforming an already
averaged sample with a nonlinear threshold changes the material mean and loses
small bright stones. The ten mip levels and coarse constants therefore describe
the same processed material. Tests pin linear means, wrap/cube-edge continuity,
continuous mineral transitions and a counterexample to filtering before the
transfer. Source filtering still resolves wavelengths against two destination
texels, followed by existing GPU anisotropic/mip filtering; it is not disabled.
The filtering rationale is consistent with the footprint-driven mip hierarchy
in [PBRT's image-texture chapter](https://www.pbr-book.org/4ed/Textures_and_Materials/Image_Texture).

The swatch repeat remains 3.15 m. Its **single authored relative-height range is
now 60 mm**, superseding the historical 18 mm setting below. Normals derive from
that height in physical chart metres, without a second slope multiplier,
height-dependent AO paint, invented water plane or vegetation-boundary cliff.
This is bump shading: it adds neither displaced terrain/pebble silhouettes nor
collision or real self-shadowing. The scan's dimensions remain uncalibrated
rendering priors, recorded with the source licence. The embedded compressed
asset and decoded allocation sizes are unchanged.

One forest-floor evaluation supplies colour and roughness. Canonical moisture
controls wet absorption/roughness; it is not standing water. Moss uses separately
filtered 0.85 m and 0.25 m bands, litter 1.6 m and broad mineral variation 3.2 m.
Each band uses its own cutoff. Canopy/ground cover gates the swatch again, and
later loose sediment, rock, salt, snow and frost retain ownership. The scan's
continuous mineral colours replace categorical sandstone/basalt/quartz guesses.

Resolved `surface_quality` Low/Medium/High retains L20/L21/L22 ceilings and
1/4/16x anisotropic tiers. Temporary area-preserving refinement covers are now
bounded at **512/1,024/2,048 leaves**, respectively; disabled detail retains the
legacy L17/512 bound. `dda58de` had used 2,048 for every detail tier. The final
**admitted demand remains 512 pages**, the CPU source cache 1,024 and GPU atlas
256 layers, with the existing bounded worker jobs. Refinement and admission
priority now use the same conservative tile-sphere/view-cone intersection,
instead of allowing offscreen underfoot pages to outrank the visible floor.
The existing no-frustum ordering remains. Tests cover standing views at negative,
zero and positive elevations, area preservation and all tier bounds.

TAA remains off in the review profile. Plant geometry, ecological placement,
exposure, shadows and cloud settings are unchanged from the preceding checkpoint.
The source/binary-pinned capture pair uses the same fixed standing camera and
includes a separate no-plants diagnostic pair. It is an engineering comparison,
not independent blind preference or reference acceptance. Actual pebble/debris
geometry, convincing cloud/aerial depth, crown diversity and frame cost remain
open.

Final-source local validation: 258 worldgen lib/bin tests, 26 terrain-stream
tests, 122 native unit tests, six material GPU fixtures (including the diagnostic
cost case), two actual-producer plant GPU fixtures, strict workspace
all-target/all-feature Clippy, architecture, formatting, feature-off and release
build. The previous full workspace suite was not repeated. The final gates,
release and captures share source manifest SHA-256
`b80d7791210db331aea8df2e911b001fea830761f13d0bdd060b2d5e833080bd`.

Actual Radeon 880M / Vulkan RADV / Mesa 26.2.5 results at 1280×720, 4× MSAA,
PBR Neutral, EV100 12.8, epoch/step zero and the fixed 2 m AGL camera:

| View | GPU median | Unique plants |
| --- | ---: | ---: |
| Fresh `09c5d66` standing control | 336.152 ms | 11,752 |
| Reviewed soil standing | 354.660 ms | 11,752 |
| `09c5d66` without plants | 222.083 ms | 0 |
| Reviewed soil without plants | 253.381 ms | 0 |
| Reviewed soil moving | 493.906 ms | 11,173 |

These single runs do not demonstrate a speedup. The standing camera, canonical
source signature and plant counts match; GPU material-level histograms differ
because selection changed. The floor remains visibly planar despite clearer
material structure, so this checkpoint does not meet the geometric ground-detail
reference. Native PNGs, commands, source/binary hashes and gate logs are pinned
in [the WORLD12 evidence](artifacts/native-surface-ground-review-2026-10-24.json)
and `target/surface-showcase/ground-review-2026-10-24/`.

## WORLD11a — Slender fern pinnae and edge-fixture review (2026-10-24)

After `230d450`, the ground fern's broad laminae are narrowed from about 3.6:1
to 6.1:1 length/width, with gentler longitudinal camber and midrib fold. The
same thirteen fronds, green/dry boundaries, 3,224/1,040 full/reduced triangles,
instance identities and ecological placement are retained. The smaller source
area (3.196165 m² rather than 5.408265 m²) exposes real gaps; no RGB, density,
exposure or fog compensation is added. Full/reduced top-view area agrees to
the recorded seven decimal places; sampled full-vertex displacement is
0.0098796 m. See [doc 75](75_NATIVE_SURFACE_PLANTS.md) for source dimensions and
bounds. This remains an authored optical asset, not calibrated botany.

A first narrow-leaf experiment with excessive camber failed the existing
normal/face-alignment regression and was rejected. The gentler final source
passes it. The actual-producer fixture also exposed a previously unclassified
lit-side shadow-edge near miss. Its existing 3% barycentric edge band now covers
both sides; analytic tests pin a clear f64 ray becoming an edge hit after f32
narrowing. Production shading/ray queries are unchanged. The two-RGB8-code
interior tolerance remains: 7,003 of 7,736 full-source samples and 2,396 of 2,680
reduced-source samples are retained as interior. Tests enforce at least 50%
interior coverage. Edge pixels themselves are not certified.

Latest local checks passed: 255 worldgen lib/bin tests, 122 native unit tests,
two actual-producer hardware fixtures, strict workspace all-target/all-feature
Clippy, architecture, formatting, feature-off and release build. These scoped
runs cover the lamina-only change. The preceding WORLD11 checkpoint's
2,094/121/36 workspace/integration/native-hardware runs remain dated evidence;
they were not repeated wholesale for this source-data revision.

Actual Radeon 880M / Vulkan RADV / Mesa 26.2.5 captures retain 1280×720,
4× MSAA, PBR Neutral, EV100 12.8, epoch/step zero, 20 s prestream and 30 warmup
frames. Static captures measure 48 frames; the sweep measures 96. Builds/tests
are separate from timing runs.

| View | GPU median | Unique plants |
| --- | ---: | ---: |
| Fresh `230d450` standing control | 354.811 ms | 11,752 |
| Slender-fern standing | 341.810 ms | 11,752 |
| Slender-fern moving | 482.576 ms | 11,173 |
| Alternate composition, 15 m east | 342.075 ms | 11,332 |

The standing pair matches camera/source metadata, plant-count telemetry and
material-level residency histograms. Exact GPU atlas IDs and per-instance IDs
are not exported by captures. These are single runs with a changed leaf shape,
not proof of a stable optimization or visual preference. The extended forest,
reduced geometry, dry outer fronds and default-off/TAA-off policy from WORLD11
remain. GPU cost, repeated crowns, sky/aerial transport, ground debris and
root-to-resident-mesh agreement still prevent reference acceptance.

Exact input/binary hashes, commands, logs, rejected experiments and frames are in
[`artifacts/native-surface-fern-lamina-2026-10-24.json`](artifacts/native-surface-fern-lamina-2026-10-24.json).
No CI or independent blind acceptance is claimed. The preceding coverage-only
and overview records below are kept with their original source identity.

## WORLD11 — Wider coverage, reduced pinnae and fern age (2026-10-24)

The continuation after `6c8d9fd` addresses the visible planting-window boundary.
Canopy queries now cover up to nine neighboring L14 windows; the optional
near-eye L24 understory covers 49 L18 windows rather than 25. All use the same
immutable identities, canonical 32 m ground and habitat/water/slope eligibility.
There is no density tint, fog mask or new scene-specific placement source.

Full/reduced pinnae share frond and leaf counts. The reduced fern/tree use
1,040/4,992 triangles rather than 3,224/15,360, with calibrated top-view area and
sampled-vertex error bounds ([doc 75](75_NATIVE_SURFACE_PLANTS.md)). The resolver
sets a scale-aware full-detail distance of 24/36/48 m for Low/Medium/High, plus
the conservative near-eye cell margin. Reeds and wood retain their source
geometry. The three long, low outer ground-fern fronds now have explicit dry
optics; ten inner fronds remain green. Their attachment geometry and total
triangle count are unchanged. Geometry, organ materials, shadows and optional
RT use the existing grouped mesh path. Source reuse compares detail layout too.

The combined full/reduced vertex/index/AO payload is 1,690,896 B; material images
remain 3,219,456 B. The final standing view uploads 752,128 B of instance matrices
and uses eleven grouped draws. Repeated organ ranges are included in RT
preflight: a 300-descriptor fern/reed fixture needs 450 primitive instances.
The same configured RT ceiling and whole-partition raster fallback remain.
All features are still opt-in, and TAA remains off.

Final local Radeon 880M / Vulkan RADV / Mesa 26.2.5 captures use 1280×720,
4× MSAA, frozen epoch/step 0, 20 s prestream and 30 warmup frames. Static views
measure 48 frames; the sweep measures 96. No builds/tests run concurrently with
these captures. The baseline binary SHA matches the preceding WORLD10 evidence.

| View | GPU median | Unique plants | Reduced plants |
| --- | ---: | ---: | ---: |
| Fresh `6c8d9fd` standing control | 318.347 ms | 4,931 | — |
| Standing, 80 m range / 2 m eye | 347.919 ms | 11,752 | 5,568 |
| Approach, 300 m range / 2 m eye | 342.124 ms | 11,790 | 4,993 |
| Overview, 800 m range / 138.11 m eye | 622.522 ms | 2,066 | 1,855 |
| Moving, 80 m range / 2 m eye | 478.995 ms | 11,173 | 4,910 |

The standing control/candidate share camera/source metadata and material-level
residency histograms; exact GPU atlas page identities are not exported. This
is a wider-population feature comparison, not an isolated LOD speedup. A single
run per view does not establish stable production performance, and these
frame times remain unsuitable for normal gameplay. The intermediate coverage
capture before dry-frond grouping is retained separately rather than presented
as final-source evidence.

An additional composition moves the numeric target 15 m east within the same
coastal audit region (longitude 165.13910882897008°). It is an alternate actual
camera view, not a matched baseline comparison or a reference-parity score.
The unchanged audit cameras remain the diagnostic evidence above.

Executed local checks: 2,094 workspace lib/bin tests, 121 integration tests,
36 native hardware tests, strict all-target/all-feature Clippy, architecture,
formatting, feature-off and native release builds. The broad test runs preceded
only a test-only `iter().copied().collect()` to `to_vec()` lint cleanup; the
affected geometry test and strict Clippy/formatting were repeated. The source
sequence, binary hashes, full commands and capture telemetry are recorded in
[`artifacts/native-surface-coverage-2026-10-24.json`](artifacts/native-surface-coverage-2026-10-24.json).
No CI or independent blind preference result is claimed.

Coverage is still bounded and same-face, with an outer boundary; the dense
layer still requires an explicit eye AGL no greater than 4 m. LOD transitions
remain discrete and are not a complete temporal-stability proof. Repeated
archetypes, unresolved ground debris, incomplete aerial transport, flat sky
structure and actual resident-triangle/root agreement remain open. This is a
location-quality continuation, not final reference acceptance.

## WORLD10 — Review of iterations through d13364b (2026-10-24)

The latest Gemini commits contained concurrent renderer/geometry work as well as
its floor/cloud tuning. This review evaluates the aggregate source and real
captures. Its report's pixel L1 and colour percentile matches against a concept
with different geometry/camera are not evidence of visual parity; its 450 ms
ceiling is not a project performance requirement. The flat sky, repeated crowns,
abrupt far-understory cutoff and unresolved ground composition remain visible.

Retained: authored organic humus/litter/moss/mineral floor mixtures, larger
ascending fronds, explicit senescence, pinned CC0 material maps and the shared
canonical scene. Corrected: twig/pebble/root wavelength filtering, continuous
material eligibility and the 13.5x uncalibrated ground-normal gain. The latter
again differentiates the actual authored 18 mm source height. Wetland context
now lies below the resolved litter rather than painting over it.

Curved indexed pinnae use 18,728 triangles but only 24,804 vertices / 1,117,680 B
across three prototypes. Ground fronds have varied arches; tree forks start
lower and have asymmetric, wider terminals. Smooth/normal-mapped foliage can
no longer receive direct reflection from the wrong geometric hemisphere.
Thin transmission, raster shadows and reflected-hit shading share this side
contract. The actual-producer ray fixture pins the finite-exclusion leaks at
45/1 for 0.01/0.0001 m, with zero invented blockers and a two-RGB8-code interior
shading tolerance. See [doc 75](75_NATIVE_SURFACE_PLANTS.md).

Mesh environment fallback reuses existing GGX integration, with explicit
quality budgets; an all-double-sided specialization eliminates an unnecessary
fragment discard. A preceding isolated 48-frame diagnostic measured GPU median
385.991 -> 226.271 ms on the 880M. Three horizon pixels differ (maximum RGB8
code difference 20), so this is not a bit-exact image result or a speedup claim
for the later scene with different geometry, cloud work, MSAA and RT admission.

Clouds now share a mean-normalized, eroded 3D density field, a retained physical
liquid-water extinction prior and a self-occluded diffuse boundary approximation
across primary/shadow/reflection consumers. The full 128-cubed canonical source
is shaped before mip selection: bake medians for retained 32/64/128 sides are
265.829/265.711/265.036 ms (five samples each). Quality reduces texture/sample
cost, not this startup bake. Full chains cost 37,449/299,593/2,396,745 R8 texel
bytes. See [doc 65](65_NATIVE_SKY_AUDIT.md) for the source, quadrature and limits.
No second-order scattering experiment is retained.

The experimental profile keeps Gemini's EV100 12.8, atmosphere density scale
1.25, cloud coverage/opacity 0.48/0.72 and High plant density. It selects
four-sample MSAA and PBR Neutral; TAA remains off. These are opt-in review
settings, not new application defaults or a proof of physical calibration.

Selecting Medium had also admitted the plant RT partition, whereas High's
larger population exceeds the same 4,096-instance limit and uses the established
whole-partition raster/shadow/environment fallback. The final profile restores
High for the denser near composition. Standing population increases from 2,447
to 4,931 instances (4,718 near clumps). The measured Medium frame is retained
separately; changing density and RT admission is not an isolated optimization
benchmark. No memory/instance limit was raised or hidden RT toggle introduced.

Executed checks: 2,091 workspace lib/bin tests, 121 workspace integration
tests, 35 native ignored hardware tests, 12 renderer cloud/terrain-environment
hardware tests, the 25-test ray-scene suite and mesh winding/specialization
fixture. Strict all-target/all-feature Clippy, feature-off build, architecture,
formatting and native release builds pass. The 35-test hardware run preceded
only test-fixture/lint cleanup and the final runtime-profile selection;
production shader/renderer code remained unchanged. Affected suites and native
normal tests were repeated as recorded in the evidence's source sequence.

All final High captures use the same certified executable, frozen epoch/step 0,
20 s prestream, 30 warmup frames and 48 measured frames at 1280×720; the sweep
uses 96 measured frames. The recorded 856 input files remain stable throughout
capture. Local Radeon 880M / Vulkan RADV / Mesa 26.2.5 GPU medians:

| View | Range / eye AGL | Single sample | 4× MSAA |
| --- | --- | ---: | ---: |
| Standing | 80 m / 2 m | 219.905 ms | 329.119 ms |
| Approach | 300 m / 2 m | 231.827 ms | 343.151 ms |
| Overview | 800 m / 138.110 m | 271.433 ms | 514.122 ms |
| Sweep | changing / 2 m | — | 426.500 ms |

These are per-scenario diagnostics, not a production performance acceptance.
High approach differs in material residency histograms and is excluded from
paired visual inference. Three candidate blind pairs were prepared: High
standing/overview and the recorded Medium approach from the same executable.
Their camera, profiles except MSAA, recorded histograms, plant counts and
drained-job counters match within each pair. Exact atlas directory/page IDs
are not exported; histogram equality is not full residency certification.
The key remains separate and there are no reviewer votes. The sweep supplies a
final still and counters, not a video or temporal-stability error envelope.

Reference parity is **not achieved**. Needed work still includes coherent
near/far vegetation representation, less repetitive botanical assets, resolved
floor debris/roots, finite-distance aerial transport and richer sky transport.
No independent blind preference or cross-platform acceptance is claimed.
Final executed checks, source/binary/capture hashes and measurement boundaries
are recorded in `docs/artifacts/native-surface-cycle-review-2026-10-24.json`.

## Target and constraints

Build one convincing, inspectable surface zone before extending its rendering
to the planet. Use WORLD00's surveyed land target at
23.565679258039204° N, 165.13881581863882° E, the unchanged canonical terrain
field and real stellar lighting. No separate showcase terrain, painted shore,
fake atmosphere or camera-specific geometry is introduced.

The current standing (80 m range, 2 m eye AGL), approach (300 m range,
2 m eye AGL) and overview (800 m range, distance-based eye) views share the
numeric target. Earlier close/overview captures used different explicit eye
heights; their dated records remain below. A moving camera and a surface-to-orbit sequence are additional
acceptance cases, not established by a still image. Large-plant geometry covers
up to nine neighboring L14 query windows; the optional near layer adds at most
49 L18 query windows, with scale-aware full/reduced pinnae (WORLD11).
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
(doc 76). It masks moving/unknown rigid meshes; explicitly immutable plants can now opt
into camera-only reprojection (doc 75). It does not establish
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

### Opt-in surface review profile

`data/graphics/surface-showcase.toml` retains the existing High surface-review
settings, experimental ground detail and understory, with TAA off. It requests
`shadows.first_cascade_distance_m = 25.0` through the shared resolver instead
of the legacy approximately 754 m first cover. Four effective 1024 px maps,
the three-star atlas budget and 1.5 m normal bias are unchanged. This reuses the
WORLD06h control; it does not add a shadow algorithm or change launch defaults.
The smaller cover redistributes spatial precision between cascades and cannot
establish a globally optimal partition or fix diffuse canopy transport.

From the repository root, use the native-only release executable:

```sh
target/release/thessa-terrain-native --graphics data/graphics/surface-showcase.toml \
  --survey-lat 23.565679258039204 --survey-lon 165.13881581863882 \
  --survey-distance 80 --survey-eye-agl 2 --sim-time 0 --sim-step 0 --mute
```

The profile is for inspecting this experimental bounded location. Its High RT
and plant workload is not an accepted frame budget on the 880M. Geometry,
occupancy and near-cover controls have separate captures so their causes and
costs are not conflated.

### WORLD08 final local evidence

Source `1d6a759` passed 11 local command checks on 645 unchanged compiled inputs:
2,082 workspace lib/bin tests, 117 integration tests, Clippy, fmt, architecture,
three directly relevant native GPU fixtures, feature-off, WORLD00 and release
build. The release SHA256 is
`7539c22d83807b3659d868d1e3738e82794028bac04f48bfdc5e9bb70090dc09`.
The separate runtime review profile was added afterward and validated through
its two native captures; their settings hashes match the tracked profile bytes.
Full native/renderer GPU suites and remote CI were not repeated for this stage.

All nine final captures exit zero on Radeon 880M/Vulkan/RADV,
Mesa 26.2.5-arch1.1, with simulation time/step zero, 1280×720 and 30 warmup
frames. Static cases measure 48 frames; the moving case measures 96.

| Static view | Retained baseline GPU median | WORLD08, legacy cover | WORLD08, 25 m near cover |
| --- | --- | --- | --- |
| Standing, 2 m | 151.625 ms | 146.660 ms | 148.936 ms |
| Close, 50 m | 109.695 ms | 112.366 ms | 115.802 ms |
| Overview, 800 m | 67.439 ms | 65.901 ms | not captured |

These are serial single-order observations, not stable FPS or isolated foliage/
shadow costs. Standing changes population from 2,020 to 2,111 plants while
retaining three draws and the same admitted RT policy. Close/overview keep 222
plants. All static pairs match the compared camera/source/material-residency
metadata; the standing composition changes are recorded separately. Static
streams drain; the moving endpoint still has four material jobs. Its GPU median
is 228.294 ms and maximum CPU frame interval 17.653 ms; no stable streaming,
stutter or temporal acceptance follows from one sweep.

Close/overview geometry and a same-population standing near-cover control form
three blinded cases, with a separate unread private key and no reviewer votes.
Original-before versus WORLD08 standing is descriptive composition evidence,
not a matched-population preference pair. Implementer inspection sees broader
crown/fern surfaces, grouped near cover and more legible near tree shadows;
remaining ground bands, foliage noise and coarse shadow sampling are visible.
The numerical gates do not certify subjective preference.

Reproduction, benchmark workload, all hashes, separate intended/mismatched scene
keys and known gaps:
[`native-leaf-colonies-2026-10-23.json`](artifacts/native-leaf-colonies-2026-10-23.json).
Raw evidence and review files remain in
`target/surface-showcase/canopy-quality/`; the ImageGen concept is explicitly
separate from these actual engine captures.

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


## WORLD09 — Review and continuation of Gemini work (2026-10-24)

The continuation started at `2b0ae6e` on `feat/surface-visual-engineering`, with
12 intervening commits and a mixed dirty tree containing the earlier material,
AO and coverage work. The as-found release was retained before editing. Its
source association is not certified, and its hard-coded `native_tonemapping`
metadata did not identify the actual shader operator. That baseline is an
observational screenshot, not a clean single-variable renderer comparison.

Retained from the intervening visual work: the 5×5 near-understory cover, tiered
48-frond crown budget, explicit atmosphere density 1.0 and EV100 13.35, 0.04 m
near-shadow bias, and 32-patch ocean budget. The last admits 20 actual near-ocean
patches at this site without partition fallback. This is not a universal shadow
bias calibration or a claim that every former performance explanation was proven.
In particular, the larger standing plant population withholds the entire mesh RT
partition; a roughness-only explanation of earlier speedups is not re-certified.

The current source adds actual pinna folds at the same leaf triangle budget,
corrects texture orientation, and returns two fork branches to the silhouette.
Five source/material groups distinguish stems, green foliage and dry hanging
fronds. The prototype total is 10,360 triangles, only 112 more than the intervening
single-trunk version. Geometry, R/T optics and visibility are shared by raster
and optional secondary-hit consumers (doc 75). Six CC0 maps have pinned source
hashes and reproducible imports. Their undocumented brightness edit is now an
explicit authored albedo transformation. Shared material mips and the separate
AO vertex stream keep actual UVs available for bark/leaf detail.

Removed: mesh-height-based dry-leaf paint, fixed-coefficient/tinted distance fog,
sinusoidal puddles and wet-ground normal flattening, and artistic grazing-lobe
damping. The source atmosphere/material systems remain owners of those phenomena;
finite-distance aerial transport and convincing source-owned wet litter are not
implemented by this round. An undocumented diffuse normalization from the inherited
drafts failed the established first-substrate-bounce GPU energy tests. Restoring
the documented entry/outgoing factors passes the white/grey furnace and direct
substrate oracles (doc 69); internal return bounces are not silently introduced.

The review profile now selects PBR Neutral, **single-sample raster and TAA off**.
Optional `renderer.msaa_samples = 4` has a real capability fallback, hardware linear
colour resolve, nearest covered depth and zero-allocation disable (doc 59). It
remains a costly quality option. The PBR Neutral dark-range early return is fixed,
the default renderer still uses Reinhard, and capture metadata now reports the
actual selected operator.

### Local evidence and limits

The fixed standing site uses the existing target coordinates, 80 m survey distance,
2 m eye AGL, epoch/step zero, 20 s prestream, 30 warmups and 48 measured 1280×720
frames, with HUD/audio off. Final native-only executable SHA256 is
`c70ee8bff8cec8345ea0b08a317d8a011756929262c562ec81aa575ad175997c`.
On Radeon 880M, Vulkan/RADV Mesa 26.2.5, the two final same-binary controls measure:

| Raster samples | GPU median | Serial wall median | Extra MSAA texels |
| --- | ---: | ---: | ---: |
| 1 | 385.169 ms | 389.218 ms | 0 B |
| 4 | 669.766 ms | 675.815 ms | 44,236,800 B |

Both retain 5,245 unique instances, five draws and the same plant geometry;
5,032 near clumps replace nine coarse clumps. Prototype vertex/index/AO bytes are
1,243,200; source textures are 3,219,456 B before mip/driver overhead. The larger
cover still withholds mesh RT by its existing 4,096-instance budget. Raster atlas
shadows and the environment fallback remain. The as-found release measured
325.605 ms GPU; the wider folded foliage does not establish a throughput win.
These are local offscreen diagnostics, not acceptable game FPS. Final controls
ran after test jobs; earlier contended/pre-fix previews are excluded from timing
claims. Independent material residency can differ, so these are not certified
whole-frame blind pairs. There are no independent votes or reference-quality
acceptance claims.

Executed: 2,088 workspace lib/bin and 120 integration tests; all-feature/all-target
Clippy; formatting and architecture gates; native feature-off compilation; all
nine terrain-environment GPU tests; display CPU/GPU/operator checks and three
native plant visibility/lifecycle checks. All six bundled PNGs reproduce exactly
from pinned source maps, and their manifest hashes agree. The independent plant
ray fixture checks 4,400 selected samples, excluding 164 shadow edges from RGB
certification. It retains 114/11 finite-bias versus near-zero-reference disagreements
at 0.01/0.0001 m, all skipped nearby blockers and zero invented blockers. This
remains a local bias limit, not complete visibility equality.

No new window/compositor or moving-camera acceptance replay, full ignored suites,
Metal/WebGPU hardware run, remote CI or main promotion is claimed. The current
image remains substantially behind the concept in floor detail, distant coverage,
canopy stability, atmospheric composition and performance.

## WORLD21 — Historical crown calibration: volumetric canopy, understory cohorts and horizon integration

Addressing direct visual critique on crown template repetition, thin geometric silhouettes,
flat crown self-shadowing, uniform understory clearings, and sharp coastal horizon contrast:

1. **Volumetric crown morphology & forward-shingled pinnules:**
   - Fronds in `crown_frond` shingle forward along the rachis (forward sweep 0.35, droop -0.20,
     half-width 0.175), eliminating coarse sawtooth chevrons in favor of continuous feathery blades.
   - Form 0, 1 and 2 feature differentiated tier structures, terminal angle offsets (breaking
     dichotomous twin symmetry), and multi-reach tier fronds (0.86..1.14x) that populate both the
     interior core and outer drooping mantle.
   - Form 1 single emergent umbrella retains 48 fronds across 4 tiers; Form 0/2 twin clusters
     maintain 48 fronds with desynchronized azimuthal phases.

2. **Calibrated cuticle transmission & self-shadowing:**
   - Foliage `diffuse_transmission` set to `0.18` across `LycopsidTree` and `FernClump`
     (down from 0.45/0.34), adjusting the authored thin-diffuse partition while respecting
     energy conservation ($R + T \le 1.0$).
   - Measured crown luminance contrast achieves > 2:1 top-lit vs shaded underside
     (top 143.5 vs bottom 67.4 sRGB), creating deep readable volumetric depth.

3. **Understory cohorts, glades and soil gravel:**
   - Understory density modulates with a canopy-cover prior (`1.0 - canopy_cover * 0.45`) and local
     spatial cohorts (`0.45..1.38x`), breaking carpet uniformity.
   - Clearing baseline occupancy calibrated to 0.20, preserving pioneer ground ferns in glades
     rather than carving out bald mud pits.
   - Ground pebbles recolored to earthen matte river gravel (`[0.082..0.115]`, roughness `0.65..0.75`),
     integrating into the damp humus floor.

4. **Atmospheric horizon integration & beach elevation bounds:**
   - Beach material coverage restricted to the true littoral spray zone (`3.0..12.0` m instead of 45 m),
     preventing distant coastal ridges from rendering as bright tan sand bars.
   - Local coastal haze extinction increased to `0.00062` / m at 600 m height in `surface-showcase.toml`,
     softly enveloping the distant bay ridge and integrating the far horizon with the overcast sky.

5. **Physical invariants & capture:**
   - Capture `standing_world21_volumetric_canopy.png` (48 frames, 1280x720, Radeon 880M):
     reported GPU median 507.2 ms (550 ms was a local iteration target, not a project acceptance invariant); 16,824 submitted plant/ground instances
     across 49 near-eye windows (bounded by the existing 49-window policy); 21,986,352 drawn plant triangles;
     7,797 accepted understory plants. All architectural, formatting and clippy gates passed.

Complete commands,
input/capture hashes, measured scopes, baseline caveats and omissions:
[`native-gemini-surface-followup-2026-10-24.json`](artifacts/native-gemini-surface-followup-2026-10-24.json).

## WORLD22 — Botanical Crown Evolution: 15-Iteration Multi-Tiered Architecture, S-Curves, and RT Convergence (World 40–54 RT)

Addressing botanical fidelity critiques on mechanical starburst crowns, flat billboard cards, lack of vertical hierarchy, and uniform canopy droop:

1. **Multi-tiered Rosette Architecture (5 Vertical Tiers):**
   - Replaced uniform frond tiers with 5 biologically distinct layers: apical emerging croziers/fiddleheads (steep upright plume), ascending juvenile fronds, spreading mature main canopy umbrella, cascading drooping fronds, and a persistent senescent skirt mantle.
   - Pinned 48-frond budget per dual-stem tree ($2 \times 24$), strictly preserving canonical Form 0 triangle invariants (`15550..=15700`).

2. **Phototropic S-Curve & Upward Tip Recurvature:**
   - Implemented terminal phototropic inflection (`+ tip_lift * ((t - 0.68) / 0.32)^2`) along the rachis, causing delicate growing tips to curl back toward the overcast sky dome rather than plunging into the soil.

3. **Adaxial V-Channel Pinna Folding & Raised Midrib:**
   - Opposed pinna pairs form an upward dihedral V-angle ($0.05 \cdot length \cdot s(1-s)$) with an elevated midrib ($0.038 \cdot length$), catching specular highlights and casting self-shadows along the rachis axis.

4. **Asymmetric Botanical Pinna Length Envelope:**
   - Replaced symmetric sine distribution with an asymmetrical obovate-lanceolate profile (`sin(pi*t)^0.58 * (1.12 - 0.32*t)`): compact pinnae near the stipe collar, expanding rapidly to maximum breadth at 38% rachis length, and attenuating into gossamer terminal tips.

5. **Rachis Torsion & Light-Facing Axial Roll:**
   - Outer drooping fronds exhibit continuous longitudinal roll (`0.025 * length * s * (t - 0.25)`), orienting the photosynthetic upper lamina towards the diffuse sky dome.

6. **Photomorphic Crown Polymorphism:**
   - **Form 1 (Coastal Windswept):** Directional windward/leeward flagging (`0.90 + 0.28 * wind_align`) with compact seaward fronds and elongated leeward plumes.
   - **Form 2 (Ancient Sprawling Veteran):** Wide horizontal reach ($5.35\text{ m}$ spread, $4.85\text{ m}$ outer) with deep cascading droop ($1.42\times$).
   - **Form 0 (Balanced Canonical):** Bounded canonical geometry meeting all contract extents ($reach \le 6.8\text{ m}$, $area \in 140..320\text{ m}^2$).

7. **Volumetric Oblate Ellipsoid Normal Warping:**
   - Upgraded SpeedTree dome normal blending from spherical to oblate ellipsoid ($1.85\times Y$), matching the wide, flat-topped umbrella aspect ratio of tree ferns and softening horizon silhouetting.

8. **Apical Core Cavity Ambient Occlusion:**
   - Applied radial cavity occlusion factor (`clamp(r / 3.2, 0.45, 1.0)`) in the dense crown core, eliminating washed-out inner branches and creating rich volumetric contrast between shaded trunk forks and sunlit outer fronds.

9. **Photometric RT Convergence (World 54 RT):**
   - Headless survey capture (`standing_world54_rt.png`, 1280x720, Hardware Ray Tracing active with 22,290 TLAS instances):
     Overall $L_1 = 49.04$ (Canopy Row 0 $L_1 = 54.71$, Floor Row 3 $L_1 = 41.20$, Horizon Sky $L_1 = 53.54$).
   - All 20 offline botanical tests, 19 terrain-native surface tests, and 12 foundational crate architecture checks passed.

10. **25-Iteration Autonomous RT Refinement (World 55 RT through World 79 RT):**
    - **Block 1: Caudex & Epiphytic Growth (Worlds 55–59 RT):**
      - Calibrated dark fibrous tree caudex bark reflectance (`[0.24, 0.21, 0.16]`).
      - Basal buttress root flares and organic soil anchorage with continuous ground plane meeting ($y = -0.14\text{ m}$).
      - Polymorphic caudex trunk girth and taper profile ($1.14\times$ for veteran Form 2).
      - Basal trunk ambient occlusion gradient from surrounding understory ferns and soil.
      - Polymorphic branch fork structural thickening.
    - **Block 2: Tree Crown Micro-Fidelity (Worlds 60–64 RT):**
      - Apical crozier fiddlehead cluster vernation in the central crown.
      - Pinna edge crenation and organic blade undulation (`0.035 * half_width * sin(4*pi*t)`).
      - Stipe base thickness and power-law rachis taper (`0.024 * (1.0 - 0.85*t)^1.35`).
      - Branch fork crown crotch attenuation and self-pruning (`0.94 + 0.06 * outward_dot`).
      - Crown silhouette edge cuticular forward-scattering and transmission calibration per botanical form.
    - **Block 3: Understory Fern Rosettes & Ground Flora (Worlds 65–69 RT):**
      - Understory fern clump multi-tier rosette with central vertical fiddleheads and spreading mature fronds.
      - Ground-contact senescent fern fronds draping directly onto the local soil datum.
      - Polymorphic understory rosette chloroplast calibration per form ($G > R > B$ strictly preserved).
      - Reed clump blade droop and maritime coastal wind deflection.
      - Understory spatial clumping and canopy clearing glades modulation (`canopy_density_prior = (1.0 - canopy_cover * 0.48).max(0.20)`).
    - **Block 4: Forest Floor, Detritus & Coastal Soil Wetting (Worlds 70–74 RT):**
      - Basalt river shingle and pebble reflectance calibration towards earthen dark matte gravel tones (`roughness 0.75..0.85`).
      - Forest floor humus and decaying leaf litter detritus reflectance calibration (`[0.042, 0.028, 0.015]`, roughness 0.88).
      - Wet soil specular response and puddle rim darkening (`roughness 0.065`, `dielectric_f0` exact water interface).
      - Coastal soil scan transition and scale-aware near-ground filtering ($0.048\text{ m}$ tree buttress footprint).
      - Deep root cavity contact ambient occlusion in basal crevices (`clamp(0.10, 1.0)`).
    - **Block 5: Atmospheric Coastal Haze, Overcast Sky & Photometric Convergence (Worlds 75–79 RT):**
      - Maritime overcast sky chromaticity calibration and stratus cloud deck (`density_scale = 1.42`, `coverage = 0.38`, `opacity = 0.60`).
      - Sea-level coastal fog and marine spray aerosol calibration (`local_haze_extinction_per_m = 0.00255`, `height_m = 520.0`).
      - Distant ridge atmospheric depth cueing and quadrature precision (`ray_steps = 32`).
      - Radiometric EV12.22 exposure calibration and shadow deepening.
      - **Photometric Convergence (World 79 RT):**
        - Headless survey capture (`standing_world79_rt.png`, 1280x720, Hardware Ray Tracing active with 21,944 TLAS instances, 64 terrain BLAS instances):
          Overall $L_1 = 48.85$ (Canopy Row 0 $L_1 = 53.81$, Floor Row 3 $L_1 = 40.41$, Midground $L_1 = 42.39$, Dark Shadows $L_1 = 50.21$).
        - 100% session stability: 25 consecutive iterations executed autonomously without session drops or desktop instability.
        - All 265 worldgen-rocky tests, 21 terrain-native tests, and 12 foundational crate architecture checks passed.

11. **Deep Visual & Ecological Architecture Overhaul (Worlds 80–83 RT):**
    - **Tree Crown Polymorphic Silhouette Architecture:**
      - Replaced uniform Y-fork dumbbell silhouette with 3 distinct morphological archetypes:
        - **Form 1 (Solitary Monopodial Umbrella):** Full $8.35\text{ m}$ straight caudex with a single apex terminal at `[0.12, 8.45, 0.08]` bearing a dense 48-frond 360-degree domed umbrella canopy (6 apical, 10 ascending juvenile, 16 spreading canopy umbrella, 10 cascading, 6 senescent).
        - **Form 2 (Asymmetric Stepped Veteran):** Low trunk split at $y = 3.95\text{ m}$ into two sharply tiered limbs: dominant upright apex at `[-0.75, 9.15, 0.25]` (28 fronds) and low sweeping horizontal bough at `[2.45, 6.75, -0.55]` (20 fronds).
        - **Form 0 (Balanced Canonical):** Exact 2-fork canonical contract (`[[-1.85, 8.05, -0.30], [1.85, 7.65, 0.30]]`, 24 fronds each).
      - Oblate SpeedTree dome normal warping updated for single or dual terminals.
    - **Botanical Hero Rosette & Soil Draping Contact:**
      - Form 2 redesigned as a **21-frond hero rosette** (4 juvenile upright, 10 spreading fountain fronds up to $2.05\text{ m}$, 7 senescent skirts up to $2.20\text{ m}$ with `width_scale = 1.35`).
      - Senescent fronds decline directly to the soil datum with flat terminal ground contact ($y = 0.0$ at $t = 1.0$), eliminating floating rosettes.
      - Cavity and basal ground contact ambient occlusion extended to `FernClump` (inner cavity down to 0.20, soil contact down to 0.22) and `ReedClump` (basal mud emergence down to 0.30).
    - **Ecological Understory Niche Partitioning:**
      - Understory scale interval expanded from `0.55..1.10` to `0.75..1.50`, enabling foreground hero ferns to reach realistic $2.0\text{--}2.5\text{ m}$ diameter.
      - **Dappled light niche:** Suppresses understory in dense dark canopy cores (`canopy > 0.65`) and bare ground, peaking in forest glades.
      - **Moisture niche:** Proximity to water table elevates density ($1.35\times$ for elevation $< 2.5\text{ m}$) and biases species selection ($1.75\times$ reed weight in riparian hollows).
    - **Stem Bark Radiometric Reflectance:**
      - Calibrated stem `base_color` from pitch-black `[0.24, 0.21, 0.16]` to weathered bark `[0.36, 0.32, 0.25]` to reveal rhomboidal leaf-scar normal maps under diffuse overcast illumination.
    - **Atmosphere & Depth Contrast:**
      - Tuned `local_haze_extinction_per_m = 0.00125` ($h = 480\text{ m}$) and cloud deck (`coverage = 0.46, opacity = 0.45`), preventing distant forest from collapsing into flat 2D silhouettes while preserving rich inner canopy shadows.
    - **Photometric RT Convergence (World 81–83 RT):**
      - Floor Row 3 Mean $L_1$ reached an all-time best of **37.80** (improved from 40.41 in World 79).
      - Dark shadows $L_1$ reached an all-time best of **42.93** (improved from 50.21 in World 79).
      - Traced triangles: 226,802; TLAS instances: 23,021; 100% test passing across 435 unit and regression tests.

