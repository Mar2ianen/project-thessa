# 75 — Native surface prototype plants (bounded consumer and prototype geometry)

**Status:** bounded native survey consumer and graphics policy are recorded on
`feat/native-surface-plants` (WORLD04). Grounded arching fern/reed/lycopsid
prototypes, per-kind albedo and a second matched capture/blind-review stage are
recorded on `feat/native-plant-prototypes` (WORLD05). This is neither planet-wide
vegetation streaming nor botanical realism, authoritative collision, or final
close-up visual acceptance. The current prototypes remain deliberately authored
meshes; WORLD06f added separate fern pinnae, forked crowns and smooth stem
normals. WORLD08 below broadens the leaf surfaces while reducing triangle count;
neither stage is a botanical reconstruction.

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

## WORLD08 — Broad leaves and spatial understory colonies

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
| `surface_plants_max_candidates` | `u64` | 0 | 4096 | 4096 | 4096 |
| `surface_plants_max_items` | `u32` | 0 | 4096 | 4096 | 4096 |
| `surface_understory_enabled` | `bool` | false | opt-in | opt-in | opt-in |
| `surface_understory_max_candidates` | `u64` | 0 | 36,864 | 36,864 | 36,864 |
| `surface_understory_max_items` | `u32` | 0 | 36,864 | 36,864 | 36,864 |

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

The agreed initial native consumer is **survey-only**: one fixed level-14 query
window around the numeric survey target. The level-20 identity lattice visits
`4^(20-14) = 4096` candidate cells; the worst-case output fits the 4096-item
ceiling. All tiers traverse the same window, not successively larger regions.
Lower density is a nested rendering subset, not a promise of proportional CPU
sampling cost. IDs, habitat, final ground height, kind and scale must not be
derived from quality; reuse the existing scatter identity/eligibility contract
(doc 70) and canonical surface sampling rather than a parallel placement model.

The disabled consumer must return before any surface sampling, jobs, instance
buffers or mesh preparation. Resolved budgets are ceilings, not allocations;
enabling policy alone must not reserve 4096 items or create GPU resources.
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
photogrammetry. WORLD05 grounded the arching foliage; WORLD06f replaced the
tree's stub forks/whorls with two ascending forks, four leafy terminals and
smooth cone normals. WORLD08 uses seven broad paired pinnae per fern rachis and
24 broad arching leaves per tree terminal. Fern fronds still
begin at local Y = 0; the adapter applies no vertical offset and does **not** lower
roots to hide geometry. Reeds retain ten arching tapered blades plus a six-blade
basal tussock. All three remain split-vertex indexed triangle lists using the
existing double-sided material; there is no foliage transmission, mesh LOD, wind,
growth, per-kind shader or collision ownership. Thin blades can alias or disappear
at a distance, undersides have no transmitted light, and smooth normals do not
remove the stem's polygonal silhouette. Stems have no terrain-conforming footprint. Canonical 32 m
root height need not equal the current interpolated resident triangle (which may
use a different wavelength/representation).

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
ownership. A 300-instance mixed-kind fixture pins Low-RT-sized 256 capacity
fallback and exact boundary acceptance without dropping raster instances.
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
