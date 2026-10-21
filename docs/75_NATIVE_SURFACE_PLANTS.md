# 75 — Native surface prototype plants (bounded consumer and prototype geometry)

**Status:** bounded native survey consumer and graphics policy are recorded on
`feat/native-surface-plants` (WORLD04). Grounded arching fern/reed/lycopsid
prototypes, per-kind albedo and a second matched capture/blind-review stage are
recorded on `feat/native-plant-prototypes` (WORLD05). This is neither planet-wide
vegetation streaming nor botanical realism, authoritative collision, or final
close-up visual acceptance. The current prototypes remain deliberately schematic
flat-shaded meshes; visible limits are recorded at the end of this document.

## Requested and resolved contract

`[surface_plants]` in `graphics.toml` has `enabled = false` and
`quality = "high"` by default. Missing/partial sections retain these defaults.
Quality uses the existing `Quality` enum; invalid values fail TOML parsing.
Unknown keys retain the shared parser's forward-compatible behavior.

`RequestedGraphics.surface_plants: SurfacePlantsSettings` enters the existing
`ResolvedGraphicsSettings::from_requested` pipeline. It resolves flat fields:

| Field | Type | Disabled | Low | Medium | High |
| --- | --- | --- | --- | --- | --- |
| `surface_plants_enabled` | `bool` | false | true | true | true |
| `surface_plants_density_scale` | `f64` | 0 | 0.25 | 0.5 | 1 |
| `surface_plants_max_candidates` | `u64` | 0 | 4096 | 4096 | 4096 |
| `surface_plants_max_items` | `u32` | 0 | 4096 | 4096 | 4096 |

Counts follow existing resolved count conventions. The consumer must perform a
checked conversion of `max_items` if its scatter API accepts `usize`. Density
remains `f64` for the scatter API. These fields are also the exact flat capture
metadata keys. Older resolved TOML without these fields deserializes as disabled
with zero density/budgets.

Low/Medium/High/Ultra presets set quality to Low/Medium/High/High, preserving
explicit enabled/disabled intent (as for ocean and topology). Custom leaves
plant policy untouched. Explicit quality overrides remain intact when parsing;
divergence from an applied preset reports custom. Both GUI descriptors require
restart for the initial retained survey representation.

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
Selecting a different window synchronously replaces the retained batch once.
Preparation is deliberately bounded synchronous CPU work, measured explicitly;
there are no plant jobs, background scheduler, or per-frame field samples.

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

The survey prototypes are hand-built schematic meshes, not botanical assets or
photogrammetry. The WORLD05 geometry pass replaces the original eight-spoke fern,
five-stem reed and umbrella-crown tree with grounded, arching foliage. Fern
fronds now begin at local Y = 0; the adapter still applies no vertical offset and
does **not** lower roots to hide geometry. Reeds use ten vertical tapered blades
plus a six-blade basal tussock. The lycopsid uses a ten-sided tapered trunk with
four stub forks and 24 arching fronds in three whorls. All three remain unindexed
flat-shaded triangle soups using the existing double-sided material; there is no
foliage transmission, LOD, wind, growth, per-kind shader or collision ownership.
The trunk is visibly faceted at 2 m, crown undersides stay dark, thin fronds can
alias into dark speckle at 50 m, and the fork stubs are near-vertical rather than
spreading branches. Stems have no terrain-conforming footprint. Canonical 32 m
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
synchronous preparation CPU milliseconds; prototype vertex/index payload bytes;
the existing 64-byte ABI instance-buffer bytes; submitted instances, draw calls
and frame upload bytes from the shared renderer. These are submitted workload,
not visibility counts. Preparation timing includes scatter/field sampling,
sorting and prototype/instance conversion, but excludes GPU renderer creation.
Payload/instance bytes are not total GPU allocation: shared uniforms, material,
environment, shadow and optional RT resources remain the existing renderer's
responsibility. The off/withheld counters and preparation time are exactly zero.
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

Those WORLD04 rows predate the WORLD05 geometry pass. Current prototype payload
is 68,256 B at every quality; the retained counts, rigid-instance bytes and
per-frame upload bytes are unchanged (see WORLD05 below).

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
