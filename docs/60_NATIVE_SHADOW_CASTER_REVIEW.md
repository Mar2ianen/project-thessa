# 60 — Native shadow-caster optimization review

**Status (2026-10-18): reviewed conservative native raster optimization.**
Baseline: `12e70ad`, branch `feat/render-pipeline-optimizations`. This follows
the dated pass-composition measurements in [doc 58](58_NATIVE_PIPELINE_REVIEW.md);
those old timings are not relabeled as measurements of this implementation.

## 2026-10-27 mesh-caster continuation

After WORLD142 reset the desktop GPU, the mesh caster now uses conservative
per-primitive spheres and the existing homogeneous light-clip test. Bounds
scan source positions once on mesh publication; primary visibility also uses them. Instances retain their
original indices; contiguous accepted runs share one indexed draw. This visibility-only checkpoint changed no source
geometry, material, plant identity, quality tier or GPU buffer.

The test reads the exact cached f32 instance-matrix bytes already used for
shadow publication. The local AABB sphere is transformed using a Gershgorin
upper bound on the narrowed linear transform's squared largest singular value,
plus a 32×f32-epsilon transform envelope. The existing clip-plane envelope uses
the actual f32 shadow matrix promoted to f64. Missing/invalid/unbounded data
fails open. This is light-volume rejection, never camera visibility: an
off-camera caster inside the light volume remains. The unculled reference
entry point disables mesh rejection as well as terrain rejection.

`ShadowDrawStats` adds `mesh_candidate_instances`, `mesh_submitted_instances`,
`mesh_candidate_triangles`, `mesh_submitted_triangles` and `mesh_draw_calls`.
These sum primitive/organ groups over actually encoded active layers, rather
than counting unique world objects. Cache hits retain zero actual work. The
existing terrain fields retain their meanings. Layers have distinct encoder
labels; the native stage trace records these counters before shadow submission.

CPU checks cover visible-run identity, invalid data, off-camera light casters,
and 2,000 scaled/rotated transforms against f32 clip-space corners. The existing
full-caster depth/final-color oracle now also requires mesh rejection and covers
far outside, rotated/scaled and behind-camera mesh instances. It passed on
isolated software Vulkan (Mesa 26.2.5 llvmpipe, CPU device, no `/dev/dri`). This
is not Radeon validation, a measured GPU speedup, or desktop-stability proof.
The original dated terrain review and measurements below remain historical.

## Imported finding and verification gap

The useful finding is unnecessary vertex processing: the old depth encoder
submitted every resident terrain slot to every active light/cascade. In the
synthetic 1,024-slot, three-light/four-cascade fixture this is 75,497,472 vertex
invocations per depth redraw, even for tiles far outside the light-view volume.
Exact cache hits already avoid the depth redraw entirely.

The imported implementation and its reported **76.5% GPU reduction /+330% FPS**
were not accepted as trusted evidence. Its split/unified oracles use the same
caster-selection code on both sides, so equality there cannot detect a caster
incorrectly removed from both images. Its nine-point AABB plus arbitrary
`max(5%, 2 m)` padding also did not establish an error bound for curved geometry,
decoded height interpolation, or spectral ocean displacement. The review does
not claim a demonstrated image failure in those supplied benchmark endpoints.

## Retained implementation

- Keep the existing generated terrain pool, atlas, shaders, draw API, shadow
  settings/quality resolution, and depth cache. No second renderer, new graphics
  effect, extra pass, simulation change or production configuration toggle.
- Precompute a sphere per shadow-enabled resident slot, centered at its
  canonical tile anchor. For cube-face normalization `|raw| >= 1` along the
  center-to-sample segment, the derivative norm is at most 1, so the full curved
  surface is enclosed by `sqrt(2) * radius * half_extent + H`. `H` bounds the
  absolute decoded height using `abs(base) + 32768 * abs(scale)`, not the extrema
  of the original unquantized samples. This deliberately admits false positives.
- Numerical allowance: height decoding/interpolation uses a `32 * f32::EPSILON`
  relative allowance; local reconstruction adds `64 * EPSILON * (radius + H)`;
  camera-relative anchor addition adds `64 * EPSILON * max(abs(relative_anchor))`.
  The homogeneous six-plane test uses the **actual f32 GPU clip matrix** promoted
  to f64, with a further `16 * EPSILON` absolute-dot-product/support allowance.
  These are float-roundoff envelopes, not physical fudge factors or tunable
  quality thresholds. Dense CPU surface samples and actual GPU-generated f32
  vertices pin the envelope on this adapter; this is not cross-platform runtime
  certification or a formal proof of every compiler's floating-point behavior.
- Unknown/custom frames, missing/invalid bounds and nonfinite tests fail open.
  Extreme radius/height scales outside the admitted regular f32 range also fail
  open. The test is against the finite **light** volume, never camera visibility:
  off-camera geometry must still cast onto visible receivers.
- **Ocean-enabled terrain deliberately submits the full resident caster set.**
  Its independently evolved/deformed vertices need their own integrated bound
  lifecycle before enabling this optimization there. At this dated checkpoint mesh casters were unchanged; the 2026-10-27
  continuation above extends rejection to meshes.
- Height uploads and dirty partition publications refresh bounds in the existing
  validated success path; retired slots clear bounds. Contiguous accepted slots
  coalesce into draw ranges without allocating a temporary visible-list vector.
- Retained CPU cost is `size_of::<Option<PatchShadowBound>>()` (40 bytes on this
  target) per admitted slot, only when shadows exist. Disabled shadows have zero
  bound-vector capacity and no extra frame work. Atlas dimensions/GPU storage and
  raster quality are unchanged; `COPY_SRC` enables explicit diagnostic readback
  without allocating an ordinary-frame staging resource.

`last_shadow_draws()` exposes candidate slots, submitted slots and draw calls
summed over the last actually encoded active layers, also printed by native
window/headless completion diagnostics. Candidate count includes slot holes;
cache hits/disabled shadows report zero, not potential redraw counts. No global
zero-allocation, zero-copy, or saved-game-FPS claim is made.

## Independent oracles and diagnostic

`encode_shadows_reference()` forces the original full resident draw and clears
the depth cache before/after; it is diagnostic-only. `ShadowAtlas::read_depth()`
explicitly reads all raw Depth32Float layers. Ordinary rendering never calls it.

`shadow_culling_matches_full_casters_not_just_split_unified` compares full/cull
atlas bytes **and** final image bytes, with observable nonempty depth and actual
rejection, moving/large/behind-body origins, three changing light directions,
inactive lights, page replacement/rejection, partition shrink/empty/slot reuse,
cache hits, optical-only ocean fallback and actual displaced ocean time rewind.
Private hardware tests additionally read generated vertices across all six cube
faces, coarse/fine tiles and 1 km/6,000 km radii, and check the disabled allocation
contract. Existing split/unified and HDR tests remain complementary, not the
culling reference oracle.

`full_versus_culled_shadow_cost` runs A/B/A/B then B/A/B/A on one resident scene,
1,024 terrain slots, 64 synthetic mesh instances, 1280×720, RT/ocean off, 12 active
512×512 atlas layers (the resolver clamps the requested 256 size to 512).
Each block retains 300 samples after 30 warmup, separately for static and moving
camera paths. Static samples retain ordinary exact-cache reuse. GPU timestamps
separate depth from remaining scene commands; wall time includes CPU update,
encoding, queue submission and GPU completion. Timing readback, endpoint
atlas/color readback, compilation, streaming and source publication are outside
the measured intervals. Every block endpoint compares atlas/color to the full
reference. This isolates caster selection, not unified-vs-split pass composition,
application loading, UI, HDR cost, audio, fleet simulation or presentation FPS.

Results and executed checks are recorded in
[`artifacts/native-shadow-caster-review-2026-10-18.json`](artifacts/native-shadow-caster-review-2026-10-18.json).
The same-tree diagnostic measured moving full/cull total GPU central values of
**20.982 /4.829 ms** (about **77.0%** lower), with depth **16.762 /0.423 ms** and
update-through-completion wall **21.943 /5.459 ms**. Central values are medians
of four per-mode block medians, not pooled-frame percentiles. Submissions fell
from **12,288 to 84 terrain slots/frame**, while coalesced draws rose from **12
to 30**. Static GPU values were **4.127 /4.122 ms**: the ~0.12% difference does
not establish a static improvement. This independently supports a large
isolated redraw saving, not the imported game-FPS claim.
Final local checks passed: **1,872 workspace lib/bin +90 integration tests**,
workspace/all-feature and native/no-default clippy with warnings denied, fmt,
**24 Bevy-CBT real-adapter tests**, **14 native ignored tests**, both backend
hardware lib oracles plus its CPU diagnostic, all five release unified-scene
checks, and the strengthened full-caster lifecycle oracle rerun. Architecture
guard and five agent-QA tests also passed; broad-suite ignored tests are not
counted as executed merely because the normal workspace run passed.
No blind preference, full-renderer replacement, remote CI or platform-runtime
acceptance is implied. Production ocean culling, tighter height bounds and
mesh-caster culling are deliberately omitted.

The [Horizon work review](80_HORIZON_RENDER_WORK_REVIEW.md) adds primary mesh frustum selection and indirect batching, while preserving complete shadow and RT source ranges. Its real coastal CPU command probe encodes 999,585,598 mesh triangles across twelve cold shadow layers after light-volume rejection; the encoder is discarded without raster submission. This is not steady cached-frame work or hardware timing. Primary visibility must not become caster eligibility. Shadow footprint representation and camera-origin/cache coupling need separate bounded corrections.

## Texel-bounded mesh representation continuation (2026-10-27)

[Doc 81](81_NATIVE_SHADOW_MESH_LOD.md) adds optional certified depth-only mesh LODs and `shadows.mesh_lod_error_texels` (legacy default 0, showcase 0.5). Selection includes actual uploaded instance scale/rotation, orthographic light projection and an f32 arithmetic envelope; unsupported or excessive-error choices retain full geometry. Source bounds expand to enclose replacements. Installation is atomic and invalidates the existing cache. The full-caster oracle disables both culling and LOD; its legacy-zero test now supplies an approximate mesh and still requires exact depth/final-colour equality.

`mesh_lod_instances`, `mesh_lod_triangles_saved` and `mesh_lod_max_error_microtexels` augment the existing actual-layer counters. Identical caster sources share 12-byte position buffers. They join native prototype admission rather than bypassing it. The canonical 12-layer cold plan changes from 999,585,598 to 283,199,642 mesh triangles, with more API runs (5,518 to 6,918). This is a measured command reduction, not GPU timing, driver-stability proof or a universal radiometric error bound. The original dated measurements above remain historical.

The [2026-10-28 shadow submission continuation](82_NATIVE_SHADOW_SUBMISSION.md) adds opt-in primary-receiver selection with full-volume fallbacks, shared-budget indexed indirect shadow batches, dirty command uploads and cached uploaded-world bounds. Its current evidence and limitations are separate from the dated measurements above.

The [shadow geometry hierarchy continuation](83_NATIVE_SHADOW_HIERARCHY.md) keeps the existing depth tier, adds an unresolved-organ tier and a sub-texel reed caster, and records the same-build original-producer comparison separately from full-geometry quality evidence.
