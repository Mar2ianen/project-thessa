# 82 — Receiver-aware shadow submission

Status: implementation on the surface feature branch, 2026-10-28. Physical-GPU
replay remains blocked after WORLD142; software fixtures and command counts do
not establish full-frame performance or reference parity. Baseline: `8b3e393`.
This continues [doc 80](80_HORIZON_RENDER_WORK_REVIEW.md) and the certified
[depth-only plant geometry](81_NATIVE_SHADOW_MESH_LOD.md).

## Receiver selection and fallbacks

`shadows.receiver_culling` is a requested/resolved boolean, default **false**;
the showcase profile requests **true**. Disabled shadows resolve it off. Active
ray tracing resolves it off with a note: off-screen ray-hit positions also sample
the atlas and cannot be bounded by the primary camera. Unsupported RT that has
resolved to raster may use receiver selection. Unknown primary/source bounds,
mismatched mesh/terrain primary views, invalid or ill-conditioned projection and
spectral-ocean displacement retain the
original light volume. Terrain caster coverage itself is unchanged.

For each active light/cascade the backend-neutral core encloses all primary
receivers within that cascade's radial selection, using the actual narrowed
primary projection, including jitter. Necessary perspective depth bounds support
ordinary, reversed and oblique projections; orthographic projections retain their
own depth interval. Eight enclosing frustum corners project into a light-plane
convex hull. Fixed arrays hold its half-spaces; no per-instance heap allocation is
needed. Normal offset, PCF support, source-vertex clip/varying arithmetic and
projection/solve roundoff enlarge the selection. Source vertices can lie far
outside a cascade even when their interpolated fragment is nearby. The terrain
arithmetic envelope conservatively uses the body/camera extent and existing
resident source bounds; uncertified custom anchors fail open. Mesh bounds include
absolute transform intermediates, including cancellation.

Only light XY is narrowed. The original light depth interval stays intact because
receiver-plane PCF at grazing incidence can compare on both sides of its central
depth. An off-camera caster is retained whenever it can overlap the receiver
footprint along the light direction. Source/proxy union spheres are tested before
LOD selection; the geometric error is still measured in the **original** light
projection, never the narrowed selection hull. No source geometry, population,
material, shadow bias, light map or shader changes are introduced.

The shadow-cache key includes the actual primary matrix when receiver selection
is eligible. Spectral-ocean/unknown-bound fallback retains view-independent shadow
reuse; rotating only the primary view does not redraw a full-volume atlas. View-only rotation, FOV and jitter therefore invalidate depth. Failed
view updates retain the old key/data. Unchanged frames retain the existing
zero-pass cache hit. Moving-origin world-space shadow reuse is not implemented.

## Command batching and bounds reuse

Supported devices encode indexed indirect commands with original instance indices.
Commands are grouped by primitive and full/proxy representation; depth-only order
is commutative. Twelve disjoint buffer regions prevent later queue writes from
overwriting another light/cascade before submission. Logical command counts stay
separate from encoded multi-draw API batches. Driver-native draw counts are not
inferred: the backend may implement a batch using multiple native operations.

The existing `renderer.shadow_memory_mib` budget first admits the depth atlas.
`shadow_mesh_command_budget_bytes` resolves only the remaining bytes. Consumers
share that atlas's remaining command reservation; resize/drop returns its previous
reservation. Capability failure, insufficient device/shared storage or the legacy
zero allowance retains direct draws with the same geometry. Buffer capacity grows
geometrically within the available budget. The twelve uploaded byte snapshots are
reused; unchanged commands produce zero command uploads. The unculled diagnostic
oracle remains direct and full geometry. This budget covers owned atlas/command
GPU resources, not driver-retained in-flight resources, CPU scratch/snapshots or
the separately admitted plant source geometry.

Uploaded world spheres are cached when instance matrices/ranges change and reused
by primary visibility and all shadow layers. Invalid data retains full geometry.
The cached light-error bound uses `||LM||F <= ||L||F ||M||2` and the same Gershgorin
transform bound; it may retain more full-detail geometry than the previous exact
linear-map calculation, never admit larger error. No meaningful bounds cache is
allocated when raster shadows are absent. This is not zero-allocation rendering.

`ShadowDrawStats` exposes logical commands, API batches, indirect use and dirty
command upload bytes alongside the existing candidate/submitted triangle and LOD
error counters. `encode_casters_observed` reports each real mesh group and layer
without readback; the ordinary path has a no-op observer. Mesh getters report
owned indirect GPU storage and world-bounds CPU storage separately. The canonical
native probe uses this same path and drops its encoder without dense submission.

## Verification and work counts

The canonical probe is the same fixed 80 m / 2 m coastal scene, 1280×720,
RT off, actual retained native producer and dropped shadow encoder as doc 81.
The current profile enables spectral ocean, so receiver selection conservatively
falls back to the original light volumes. Earlier exploratory rectangle-selection
counts of 155.7M are superseded; they omit this final source-precision fallback.

| Cold mesh work | Doc 81 baseline | Current submission |
| --- | ---: | ---: |
| Retained instances | 17,992 | 17,992 |
| Primary triangles / API batches | 17,111,160 / 37 | 17,111,160 / 37 |
| Shadow triangles | 283,199,642 | 283,223,492 |
| Shadow logical commands | 6,918 | 6,930 |
| Shadow API batches | 6,918 | 453 |
| Command GPU storage | 0 | 7,864,320 B |
| World-bounds CPU storage | no persistent cache | 1,784,312 B |
| Plant prototype GPU payload | 45,493,332 B | 45,493,332 B |

The scalar LOD envelope adds 23,850 full-detail triangles (0.0084%); its maximum
admitted error remains below 0.5 texel. API batch count falls 93.45%, without a
claim about driver-native calls. The atlas and shared command storage stay within
the resolved 64 MiB budget; the unchanged 48 MB source admission is separate.

All 12 final CPU/build checks pass on a pinned source manifest: 519 CPU cases
(133 native, 286 worldgen, 50 graphics, 45 render-core and five bounds/projection),
strict all-target/all-feature Clippy, feature-off and architecture checks,
regression-fixture builds and a release build. Earlier exploratory check sweeps
were superseded as the implementation evolved; they are not the final acceptance
source. Native transport tests run sequentially without changing their deadlines.

Five actual isolated software render regressions pass: full/direct primary output
with cached bounds, zero-error original shadow output, receiver-selected final
colour, mixed full/proxy indirect versus direct depth/colour, and the real
full/coarse six-form plants plus ground receiver. The receiver regression covers
six camera/source cases and three spectral-ocean fallback/cache cases. The command
regression covers all twelve layers, two origins, missing capabilities, insufficient
local/shared budget, upload reuse and reservation release. The dense canonical
probe is a separate command-only sixth case, never a submitted scene render.

The actual plant fixture retains its previous geometric/receiver regression
budgets: maximum colour difference 9/255 on one channel, maximum mean RGB
0.04453/255, largest unmatched depth-coverage component two samples. Those are
scoped fixture results, not universal radiometric or temporal guarantees. Images,
raw depth/colour, per-case counters, commands and hashes accompany the artifact.

The final debug encoder observation is 96.78 ms and frozen native update p50/p95
0.268/0.293 ms. The doc 81 encoder observation was 445.07 ms. These separate
observations are not calibrated same-build A/B timing, complete CPU render cost
or a GPU frame-time measurement.

Source manifest: `8d831d68bbe312c5f6b222370f9640ce1d757bf71f085d451f879530df6a2801`.
Release binary: `a2011bcb296be7cdaf87ebe43f77589588f16361ad4646f5b1ed01a548a197c0`.

Final evidence is recorded in
[the artifact](artifacts/native-shadow-submission-2026-10-28.json).

Physical Radeon timings, full-frame moving-camera distributions, global receiver
radiometric/temporal error and independent blind reference acceptance remain open.
TAA remains default-off. The large retained primary workload and remaining shadow
work still require further representation work; this slice does not fix them by
thinning the world population.

The [shadow geometry hierarchy continuation](83_NATIVE_SHADOW_HIERARCHY.md) keeps the existing depth tier, adds an unresolved-organ tier and a sub-texel reed caster, and records the same-build original-producer comparison separately from full-geometry quality evidence.
