# 75 — Native surface prototype plants (WORLD04 bounded consumer)

**Status:** bounded native survey consumer and graphics policy implemented;
CPU verification is recorded below. Hardware/capture/independent visual review
are recorded separately as integration-owner evidence. This is neither planet-wide
vegetation streaming nor botanical realism, authoritative collision, or final
close-up visual acceptance.

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

These are the existing simple fern/reed/lycopsid prototypes, not realistic plant
assets. In particular the producer's fern geometry begins at local Y = 0.1 m;
the adapter preserves this and does **not** lower the root to hide it. Stems have
no terrain-conforming footprint and the population has no physics/collision
ownership. Canonical 32 m root height need not equal the current interpolated
resident triangle (which may use a different wavelength/representation).
**Actual root-to-resident-triangle residual is unmeasured in this change.**
Page-height comparisons would not establish that residual; neither a clearance
bound nor a resident-mesh contact guarantee is claimed. Parent captures/review
must consider this limitation without introducing a vertical offset.

A separate independently reviewed hardware oracle now measures the existing
canonical descriptors against one **fixed generated regular L14 patch**, using
actual GPU vertex readback and f64 radial ray/triangle intersections. All 222
roots hit; signed residuals span -0.068938668 to +0.096391605 m, with nearest-rank
p50 -0.000603619 m and p95 +0.047214586 m. The observed grid-height quantization
error is only 0.000053186 m, not the allowed 0.5 m page budget. This is neither
the actual viewer's streamed resident cover nor a universal contact envelope;
the fern geometry gap remains separate. Exact ABI/topology, arithmetic checks,
per-kind results and source/log hashes:
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
