# 80 — Horizon review and native render-work correction

Status: implemented primary slice on the surface branch, followed by the
[shadow representation slice](81_NATIVE_SHADOW_MESH_LOD.md); full frame-time
and target-hardware acceptance remain open. Reviewed 2026-10-27 after the user
rejected further cosmetic iterations as the primary workflow.

## Source and method

Reference: [ludovic111/horizon](https://github.com/ludovic111/horizon/tree/cf62fe270b243780f9aa86c49b456d9dd9078b42),
commit `cf62fe270b243780f9aa86c49b456d9dd9078b42`, MIT. This is a source review;
Horizon was not benchmarked or launched and no Minecraft assets were obtained.
The implementation below independently extends Thessa's existing conservative
bounds and mesh consumer. It copies no Horizon code.

Relevant primary files at that pinned commit:

- [`gfx.rs`](https://github.com/ludovic111/horizon/blob/cf62fe270b243780f9aa86c49b456d9dd9078b42/crates/horizon-client/src/gfx.rs):
  view selection, capability-checked indexed indirect batches and direct fallback.
- [`gfx/cull.rs`](https://github.com/ludovic111/horizon/blob/cf62fe270b243780f9aa86c49b456d9dd9078b42/crates/horizon-client/src/gfx/cull.rs)
  and `cull.wgsl`: conservative two-phase occlusion. Delayed results select a
  phase; same-frame tests preserve newly exposed geometry.
- [`gfx/arena.rs`](https://github.com/ludovic111/horizon/blob/cf62fe270b243780f9aa86c49b456d9dd9078b42/crates/horizon-client/src/gfx/arena.rs),
  `upload.rs` and `lod.rs`: persistent storage, reused staging, bounded publication
  and actual far-mesh bounds.
- [`gfx/occlusion.rs`](https://github.com/ludovic111/horizon/blob/cf62fe270b243780f9aa86c49b456d9dd9078b42/crates/horizon-client/src/gfx/occlusion.rs):
  asynchronous voxel-connectivity search. Its topology assumptions do not describe
  Thessa's continuous planetary terrain or open fern crowns.
- [`docs/benchmark-report.md`](https://github.com/ludovic111/horizon/blob/cf62fe270b243780f9aa86c49b456d9dd9078b42/docs/benchmark-report.md):
  matched conditions and separate CPU, GPU and startup measurements. Its example
  figures and voxel microbenchmarks are not Thessa performance evidence.

## Findings and applicability

| Mechanism | Thessa before this slice | Applied / remaining |
| --- | --- | --- |
| View selection before drawing | Every mesh range submitted in primary; conservative culling only for shadow light volumes | Primary per-primitive/instance frustum selection now reuses the existing numerical envelope |
| Indexed indirect batches | Existing prototype instancing kept API calls low, but naively splitting visible runs produced 3,226 calls | Persistent indirect command buffer batches the runs per primitive; explicit direct fallback |
| Stable publications | Frozen mesh matrices were reconstructed and uploaded on every successful update | Reuse validated bytes and draw plans when instances, origin, clip and ranges are unchanged |
| Representation accuracy | Dense near geometry and one coarse fern tier also feed all shadow cascades | Texel-bounded depth-only plant representation now follows in doc 81; a broader primary foliage hierarchy remains necessary |
| Work budgets | 47/48 MB source admission is not a bound on expanded instance/pass work | Candidate/submitted counters and diagnostic stage admission remain; normal full-scene work is still excessive |
| Asynchronous occlusion | No equivalent whole-forest solution | Evaluate only after representation/shadow work; foliage holes may repay box tests poorly |

A shared vertex arena alone would not remove the dominant work: Thessa already
instances a small number of prototypes. Minecraft section connectivity, face
merging and voxel lighting are not drop-in replacements for CBT terrain or
botanical surfaces. Existing persistent CBT semantics remain unchanged.

## Implemented behavior

Primary visibility uses the source bounds and actual camera-relative f32 matrices
published to the vertex shader, including the existing f32 rounding envelope.
Consecutive visible original indices form runs. Neither source instances nor
shadow/RT ranges are thinned. Jitter uses the actual primary projection; camera,
instance, range and source replacement invalidate the plan. An invalid frame
keeps the prior published plan.

On devices advertising `INDIRECT_FIRST_INSTANCE` and indirect execution, each
primitive batches its visible runs using 20-byte indexed indirect commands.
Buffers grow within device limits, are reused and are written only when commands
change. Other devices retain the direct run loop. No vendor policy or new visual
quality toggle is introduced. Original order and instance indices are retained.

Unchanged world instances at the same camera origin reuse serialized model bytes;
view/lighting uniforms still update. A failed publication cannot contaminate that
cache: reuse comes from the last validated uploaded mirror, not partial scratch.
Shadow revisions still follow world geometry/range changes and required origin
changes; primary visibility is not a shadow or RT eligibility test.

`MeshDrawStats` now separates candidate and submitted instances/triangles,
logical indexed commands and API draw calls, and reports actual latest update
bytes including indirect uploads. The native diagnostics consume those values.
A pre-existing debug assertion equating scatter descriptors with all instances
was corrected: aggregated moss/litter/water add draw instances without inventing
scatter identities.

## Measured work, not a hardware speedup

Frozen canonical coast: latitude `23.565679258039204`, longitude
`165.13881581863882`, eye 2 m AGL, survey distance 80 m, 1280×720, epoch zero.
The audited source is the `053e51d` coastal producer with these renderer changes.
The test constructs the actual native terrain/environment and plant consumer on
isolated Mesa llvmpipe 26.2.5. It never submits a raster or acceleration-structure
pass; its shadow encoder is deliberately dropped after counting commands.

| Metric | Original full-range plan | Selected / batched plan |
| --- | ---: | ---: |
| Retained instance population | 17,992 | 17,992 |
| Unique primary instances used | 17,992 candidates | 2,510 |
| Primary mesh triangles | 112,813,834 | 17,111,160 |
| API draw calls | 37 full ranges; 3,226 naive selected runs | 37 batches / 3,226 indexed commands |
| Frozen mesh update bytes | 1,152,064 old estimate/write pattern | 576 measured bytes |

Primary triangle submission falls 84.83% in this view. This is a work count,
not a claim of 6.59× GPU speedup or a general view-dependent reduction.
The final frozen native update has a debug-build CPU p50 of 0.263 ms across
30 samples after five warmups. Earlier probe CPU timings used a different
shared-environment setup; no before/after CPU speedup is inferred from them.
These numbers are not frame time, GPU timing, or player FPS.

The separate **cold shadow plan** has 12 layers, 1,353,766,008 candidate mesh
triangles and **999,585,598 encoded mesh triangles** after existing light-volume
culling, in 5,518 API calls. CPU encoding took 316.07 ms in the debug probe.
This explains why a primary-only fix is insufficient. An exact cached frozen
shadow frame can encode zero layers; camera/source/light updates can expose the
large cold cost again. The total cold mesh plan remains 1,016,696,758 triangles,
of which 98.32% are shadow work. Primary selection reduces this particular cold
mesh total by only 8.60%; its 84.83% primary reduction must not be presented as a
whole-frame result. The above cold counters must not be advertised as a steady
frozen-frame measurement.

Physical Radeon replay remains blocked after WORLD142's desktop-session loss.
No full-frame GPU timing, safe full-scene hardware acceptance or reference parity
is claimed by this slice.

The cold counters above record the primary-only baseline. The subsequent
[shadow representation slice](81_NATIVE_SHADOW_MESH_LOD.md) reduces its 999.6M
encoded shadow triangles to 283.2M; it still does not establish target-GPU FPS.

## Further architectural work

1. Extend the geometric shadow-footprint work in doc 81 with moving-camera and
   broader receiver/radiometric acceptance. The implemented surface-distance
   certificate is not a global receiver-error or temporal-stability guarantee.
2. The [receiver/submission continuation](82_NATIVE_SHADOW_SUBMISSION.md) implements primary receiver selection and view-key invalidation with full-volume RT/uncertain-bound fallbacks. Further audit of world-space cache keys remains.
   Off-camera casters can shadow visible receivers. Camera-relative matrix
   changes alone must not be confused with physical motion, but reusing depth
   across origins requires an explicit floating-point error/coordinate contract.
3. Add further screen-error foliage representations. The current engine's opaque
   geometry path lacks a complete cutout-material contract across primary,
   raster-shadow and ray-query consumers. Alpha cards cannot be introduced only
   in the visible shader: that would create solid-card shadows/reflections.
4. Investigate persistent render anchors for moving-camera instance data, using
   the existing `ViewUniformBlock::new_relative` contract. Include shadow and RT
   coordinate conversion in that change rather than moving just primary vertices.
5. Profile a fixed-resident scene per stage before adding optional occlusion or
   more visual effects. Keep startup generation, publication, CPU encoding,
   GPU work, moving views and stationary cache hits separate.

Acceptance requires matched actual renders, dense real-producer workloads,
cache/invalidation tests, and measured target-hardware frame distributions once
replay is demonstrably bounded. Arbitrary population thinning, silent quality
changes, serial debug traces or source-byte caps do not establish that result.

## Verification record

The final source passes 135 CPU cases (132 native, three conservative-bounds
cases), strict affected all-target/all-feature Clippy, formatting, feature-off
compilation, architecture checks and a release build. Four actual software
render cases pass: full/selected/indirect/direct primary colour and depth,
full/culled shadow depth and final colour, textured moving/rebased instances
with failed-update recovery, and dielectric Fresnel propagation. The fifth
software case records the real coastal command plan without raster/AS submission.
The original transport-deadline failure under concurrent CPU load passed both
an isolated rerun and the final complete native suite; its timeout is unchanged.

Hashes, commands and counter data are in
`artifacts/native-horizon-render-work-2026-10-27.json`. The source manifest and
raw logs are retained with the reproducible output bundle. Physical-GPU,
whole-frame, moving-camera and blind reference acceptance remain open.

The [shadow geometry hierarchy continuation](83_NATIVE_SHADOW_HIERARCHY.md) keeps the existing depth tier, adds an unresolved-organ tier and a sub-texel reed caster, and records the same-build original-producer comparison separately from full-geometry quality evidence.
