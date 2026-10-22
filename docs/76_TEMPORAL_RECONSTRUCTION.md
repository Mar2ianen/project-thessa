# 76 — Temporal reconstruction foundation

**Status:** audit plus four signal-foundation stages and a CPU resolve contract.
T2a implements deterministic Halton jitter, static-surface reprojection and
depth reconstruction in `render-core`, with a view-block constructor that
preserves the unjittered history projection. T3a adds a standalone GPU
depth-reprojection motion-vector pass with a hardware CPU-oracle test. T3b wires
opt-in motion diagnostics into both native loops, enforces stored opaque scene
depth and masks ocean/all rigid meshes. T2b adds opt-in primary terrain/mesh
jitter with matching depth reconstruction and jitter-free motion. T4a adds the
backend-neutral history-lookup, disocclusion, neighbourhood-clipping and
accumulation-age contract in `render-core`. No GPU colour history, temporal
resolve, presentation change or SDK adapter exists yet. Default rendering remains
the non-temporal reference; the explicit jitter diagnostic visibly shifts samples
without antialiasing them.

Temporal reconstruction is not one post-process. It is a contract between the
authoritative simulation, the frames the renderer actually produces, and the
frames the presenter may generate or repeat. Up scalers and frame-generation
SDKs consume that contract; they must not define it.

## 1. Separation of clocks and frames

| Concept | Owner | Rule |
| --- | --- | --- |
| Simulation time / physics step | `sim-core` (`SimTime`) | Never derived from render or presentation cadence. |
| Rendered frame | native renderer | One completed primary-visibility sample. Advances history exactly once. |
| Presented frame | presenter | May repeat or interpolate; never advances simulation or history. |
| Generated frame (FG) | future presenter/SDK | Presentation-only. Must not age temporal history or trigger alarms/autopilot. |
| View epoch | camera owner | Changes on teleport, cut, projection/extent change or exposure reset. |
| Geometry/material/lighting epoch | renderer | Changes when history is no longer trustworthy. |

History age is counted in **rendered frames**, not presented frames, wall time or
simulation ticks. NVIDIA NRD documents the same requirement for its denoisers:
frame generation must not increase temporal inertia by ageing history with
generated frames. The same rule applies to TAA/TAAU and to any future SDK
adapter.

## 2. Verified current state

These points were checked in `feat/world-surface-runtime-foundation` before this
design was written.

1. **Jitter is a placeholder.** `crates/render-wgpu/src/view.rs` already carries
   `unjittered_clip_from_world`, but both matrices are built from the same
   projection and a test asserts equality. No temporal pass consumes jitter.
2. **Depth may be discarded.** `TerrainRenderer::encode_scene_ops(..., store_depth)`
   selects `StoreOp::Discard` unless a downstream pass asks for storage
   (`encode_scene` hard-codes `true`). A temporal pass cannot assume that the
   current depth attachment still contains the frame.
3. **No scene-integrated motion-vector producer existed at the audit.**
   `ViewParams` and `BodyFrame` provide the correct f64 camera/body transforms,
   but the scene loop has no per-pixel velocity output, no previous-frame
   transform set and no previous displaced-surface state. Matching `TileKey` is
   not triangle identity across CBT split/merge. Since then, T3a added a
   standalone static-depth reprojection pass; T3b now calls it in the scene
   loop with explicit ocean/all-mesh invalid masks, not object motion.
4. **Presentation belongs to the app.** `apps/terrain-native/src/window.rs`
   submits and presents directly. `crates/render-wgpu/src/display.rs` owns only
   the HDR-to-SDR transform. Frame generation needs an explicit presentation
   boundary, not another compute shader.
5. **HDR is an RGBA16Float offscreen target.** With HDR enabled the scene renders
   into `DisplayTransform`'s target, then a fullscreen display pass writes the
   swapchain. This is a natural place to insert a resolve, but it also sets the
   memory budget.

## 3. SDK boundary

Support for Vulkan in a technique and support for Vulkan in the current SDK are
different claims.

| Technology | Current boundary relevant to Thessa |
| --- | --- |
| FidelityFX SDK 1.1.4 FSR 3.1 | Vulkan path includes super resolution and frame interpolation. |
| FSR SDK 2.3.0 / ML components | Current branch does not expose a Vulkan path; do not design a universal “FSR 4 Vulkan” dependency. |
| XeSS 3 Super Resolution | Vulkan supported. |
| XeSS Frame Generation | Official path requires Windows/D3D12 and XeLL; third-party-GPU implementation is limited to one generated frame. |
| NVIDIA NRD | History age must use rendered FPS, not post-FG FPS. |
| FSR radiance cache | Research backlog only; not a universal Vulkan requirement. |

Policy:

- `render-core` and simulation contain no FSR/XeSS/DLSS/NRD types or handles.
- Adapters consume a backend-neutral temporal signal contract and may be absent.
- Vulkan remains the primary native target. A D3D12/Windows-only feature must be
  an explicit capability/quality fallback, never a hidden default or a reason to
  change the render architecture.
- SDK availability is not capability proof; probe the device/loader and degrade
  before allocating temporal state.
- Frame generation is downstream of a real presentation boundary. It is not a
  prerequisite for super resolution and must not be simulated by repeating
  simulation ticks.

## 4. Frame contract

A future `TemporalFrame` should carry at least:

```text
render_frame_index: u64
present_frame_index: u64
simulation_time: SimTime
extent: [u32; 2]
jitter_px: [f32; 2]              // sub-pixel, current rendered frame
jitter_index: u32                // sequence position, not frame identity
view_epoch: u64
geometry_epoch: u64
material_epoch: u64
lighting_epoch: u64
unjittered_clip_from_world: Mat4
previous_unjittered_clip_from_world: Mat4
depth_valid: bool
motion_valid: bool
```

The jittered and unjittered projections are distinct values. Raster uses the
jittered projection. Motion is measured between unjittered UVs: remove the
current jitter from the sample UV before applying the unjittered inverse, then
apply the previous jitter when sampling a jittered history texture. Applying an
unjittered inverse directly to a jittered pixel's centre reconstructs the wrong
ray. Shadow, ray-tracing and other secondary passes use the
unjittered view unless their own contract says otherwise.

History is invalidated on:

- extent/format or projection change (including FOV and jitter feasibility);
- camera cut/teleport, map or survey-window replacement;
- exposure reset when the resolve operates in pre-exposure space;
- geometry topology or height publication that makes depth/motion inconsistent;
- material/lighting changes that change the current color distribution;
- surface suspension, swapchain reconfiguration and device loss.

A resize or UI-only change must not silently blend unrelated history.

## 5. Motion-vector contract

Motion vectors describe the screen-space displacement of the **same surface
point** from the previous rendered frame to the current one. They are not a
difference of independent depth buffers.

For static planetary/CBT terrain:

1. Reconstruct the camera-relative current world position from current depth and
   the matching raster inverse view-projection. If using an unjittered inverse,
   remove the current pixel jitter from sample UV first. View matrices/origin
   deltas are derived in f64 before the GPU's f32 calculation.
2. Project that world position with the previous unjittered view-projection.
3. Convert both positions to unjittered UV and store
   `previous_unjittered_uv - (raster_uv - current_jitter_px / extent)`.
   To sample a previous jittered texture, a future resolve adds the previous
   jitter to that unjittered source UV; velocity itself never includes jitter.
4. Reject/repair samples whose current and previous depths are inconsistent
   beyond the projected error budget.

This does not require triangle identity and therefore remains valid across CBT
split/merge as long as the represented surface did not change. A changed page,
height bound or topology still invalidates the affected region rather than
trusting the reprojection.

Ocean geometry is displaced per frame, so camera-only reprojection is **not**
sufficient. A correct ocean vector needs the previous wave phase/displacement for
the same canonical surface sample, or a previous-depth-guided correction. Until
that exists, ocean pixels must be marked motion-invalid and reconstructed with a
cheaper history or current-frame fallback.

Rigid meshes need their previous model/body transform, not only the previous
camera. Skinned or deforming meshes need a previous vertex/pose source. If that
source is absent, those pixels are motion-invalid; a camera-only vector must not
be presented as full object motion.

Sky/background pixels have no finite surface motion; they must be tagged so
history clamping does not treat them as moving terrain.

## 6. Jitter and depth prerequisites

- Use a deterministic sub-pixel sequence (Halton 2,3 is the default), amplitude
  at most 0.5 pixel in each axis, and apply it in clip space as a translation
  proportional to `clip.w`.
- Preserve both projections. Do not reconstruct the unjittered matrix by
  subtracting an assumed offset from an arbitrary matrix.
- The resolve samples the current scene through the jittered projection and
  reconstructs through its matching inverse, or removes jitter from sample UV
  before applying the unjittered inverse. T2b now implements the matching-inverse
  path for the opt-in primary jitter diagnostic.
- When temporal reconstruction is active, scene depth must be stored. The
  renderer API must make that a required input, not an implicit assumption about
  an attachment that another pass may discard.
- Depth precision is part of the contract: the current Depth32Float target is
  sufficient to test reprojection, but the error envelope must be measured at
  planetary camera distances rather than assumed from a unit-scale scene.

## 7. Memory and quality tiers

At 2560×1600:

| Resource | Format | Bytes |
| --- | --- | --- |
| One colour history | RGBA16Float | 32,768,000 (31.25 MiB) |
| Two colour histories | RGBA16Float | 65,536,000 (62.5 MiB) |
| Velocity | RGBA16Float | 32,768,000 (31.25 MiB) |
| Depth | Depth32Float | 16,384,000 (15.625 MiB) |
| **Total, two histories + velocity + depth** | | **114,688,000 (109.375 MiB)** |

This excludes scene targets, shadow/RT resources, material pages, UI and driver
overhead. On UMA that is real system memory, not free VRAM. Quality tiers must
choose history count/resolution and enable/disable motion or upscaling with an
explicit fallback; `enabled = false` must allocate no history, velocity or depth
for temporal use.

The T3a pass uses RGBA16F, not RG16F, because core WebGPU does not guarantee
RG16F as a storage-writable format; alpha carries vector validity. An
adapter-specific RG16F path would need explicit feature probing and would halve
only the velocity line.

The first native resolve should be native-resolution TAA, not TAAU: one colour
history, no second upscale history. TAAU/SR adapters can add their own histories
only after the base contract is verified.

## 8. Presentation boundary and frame generation

Frame generation belongs after a real presentation queue exists. The current
`window.rs` submit/present path must be extracted behind a backend-neutral
presenter contract before an FG adapter is attempted. Required properties:

- acquired/presented image ownership is explicit and survives reconfigure;
- UI is composited in the correct space before or after upscaling by policy;
- generated frames are tagged and never advance `SimTime`, alarms, autopilot,
  input edges or history age;
- camera/input sampling for generated frames is defined (reprojection or
  interpolation), not an accidental mix of two simulation times;
- failure/disable paths present the real frame with no generated intermediate.

## 9. Staged implementation

| Stage | Deliverable | Exit criterion |
| --- | --- | --- |
| T1 | This audit/contract | Reviewed, no code claims. |
| T2a | Jitter/reprojection math + preserved unjittered view block | Implemented; deterministic CPU tests; zero jitter byte-identical. |
| T2b | Renderer jitter wiring + required depth storage | Implemented: opt-in primary terrain/mesh jitter, matching depth inverse, jitter-free motion, unjittered secondary policy and submitted-frame sequence. No resolve yet. |
| T3a | Static terrain/CBT camera motion-vector pass | Implemented; GPU output matches the CPU reprojection oracle on a depth fixture. |
| T3b | Scene integration for T3a | Implemented: real stored opaque depth, conservative ocean/all-mesh invalid masks, submitted-camera tracking and GPU scene oracle. |
| T4a | Backend-neutral resolve contract | Implemented: jitter-aware history lookup, explicit depth-disocclusion test, YCoCg neighbourhood clipping, age/weight policy and fallbacks; CPU tests only. |
| T4 | Native-resolution TAA with history validation | Static and panning survey captures; no history reuse after cancel/resize; ghosting bounded against a no-TAA reference. |
| T5 | Dynamic mesh and ocean motion (or explicit invalid masks) | Ships/ocean do not ghost; invalid pixels fall back. |
| T6 | TAAU upscale and two-history quality tiers | Matched native/upscaled captures; memory budget recorded. |
| T7 | SDK SR adapters (FSR 3.1 Vulkan first, XeSS SR where available) | Same signals and fallbacks; no SDK types in core. |
| T8 | Presentation boundary + FG (Windows/D3D12-only paths optional) | Real frames unaffected by generated frames; UI/time policy verified. |

No stage may use an SDK to paper over missing jitter, depth, motion or history
invalidation. A native TAA reference is required before comparing adapters.

### T2a delivered

- `crates/render-core/src/temporal.rs` defines `JitterOffset` (Halton 2,3,
  bounded to ±0.5 px), clip-space jitter application, `project_uv_depth`,
  `world_from_depth`, `motion_uv` with the `previous_uv - current_uv`
  convention, and the composed `reproject_static_surface` path.
- `crates/render-wgpu/src/view.rs` adds `ViewUniformBlock::new_jittered`, which
  builds both projections from the caller's matrices. `clip_from_world`,
  `clip_from_view`, `world_from_clip` and `view_from_clip` are jittered;
  `unjittered_clip_from_world` remains the history source.
- Ten `render-core` temporal tests and nine `render-wgpu` view tests pass,
  including depth round-trip, composed static reprojection, behind-camera
  rejection, camera-translation sign, w/depth preservation under jitter, and
  byte identity for zero jitter.
- At the T2a checkpoint nothing called `new_jittered`. T2b below adds the
  opt-in caller without claiming visible antialiasing or aesthetic improvement.

### T3a delivered

- `crates/render-wgpu/src/motion.rs` and `motion_vectors.wgsl` implement a
  fullscreen compute pass that reconstructs a camera-relative world position
  from stored depth, applies the current-to-previous camera offset, projects it
  with the previous unjittered matrix and writes
  `previous_uv - current_uv` into `RGBA16Float`. Alpha `1` is valid; `0`
  marks sky, behind-camera or otherwise invalid samples instead of pretending a
  zero vector is valid.
- The pass takes no history and keeps no per-frame state. The caller supplies
  the current unjittered inverse, the previous unjittered matrix and a small
  `f64`-derived camera offset; absolute planetary translations never enter the
  `f32` uniform.
- `crates/render-wgpu/tests/motion_vectors.rs` renders a depth-only gradient,
  runs the pass and compares sampled vectors with an independent CPU
  `world_from_depth`/projection oracle; it also checks the invalid-alpha path.
  The unit test pins the 160-byte uniform layout and validation.
- At the T3a checkpoint the scene loop did not call the pass. T3b below adds
  integration and explicit invalid masks; no history/TAA consumer exists.

### T3b delivered

- `[temporal].motion_vectors = false` is the default. This diagnostic switch
  resolves through `RequestedGraphics -> ResolvedGraphicsSettings`; HDR-off or
  explicitly unsupported HDR falls back to off with a note. No automatic HDR
  enable, primary jitter, colour history or visible antialiasing is implied.
  Native-resolution RGBA16F signals have no reduced-quality variant yet: off
  is the zero-work fallback, rather than silently changing vector resolution.
- Native window and headless loops encode opaque terrain/mesh/clear background,
  motion, plume/cloud/star/aurora overlays, display and HUD, in that order.
  Resolved motion diagnostics force scene-depth `Store` even if the caller
  passes `store_depth = false`. `encode_scene_ops` remains a compatibility
  wrapper with the original opaque-then-plume order when no diagnostic is used.
- Native HDR alpha becomes a static-geometry validity mask **only when enabled**.
  Water with any optical wave contribution is invalid. The entire spectral
  displacement height-admission band is conservatively invalid, including
  rough/dry fragments: vertex-level displacement material sampling can differ
  from the fragment's filtered material. Every rigid mesh,
  including the early Low-material shader path, is conservatively invalid
  until previous object transforms exist. Sky/far depth is invalid separately.
  Shared Bevy shaders and disabled native alpha/RGB are unchanged.
- `SceneMotion` allocates only the motion texture (8 bytes/pixel), compute
  pipeline and 160-byte uniform. At 2560×1600 the texture is 31.25 MiB, excluding
  existing HDR/depth, driver/pipeline overhead and transient bind groups. Depth
  already exists; this mode adds its store bandwidth, not a duplicate depth
  allocation. No per-frame host readback, simulation work or display pass is
  added. The app still owns submit/present.
- Encoding does not age the camera source. Only a submitted real frame calls
  `commit`; the first frame, explicit reset, resize/reconfigure and changed
  projection/exposure invalidate vectors. Rejected extents preserve resources.
  Camera-cut, body/source/material/light epochs and disocclusion rejection must
  still be wired before a history consumer is enabled.
- The buffer describes **opaque** motion, not the composited translucent image.
  A future resolve/SDK consumer must either run at that boundary or add proper
  translucent/reactive masks; the diagnostic is not yet an SDK-ready full frame.
- At the T3b checkpoint, the GPU scene oracle exercised the real regular/conforming terrain pipelines,
  rough fallback and loaded water materials, Low/High mesh materials, translation
  plus rotation, reset and resize, plus
  stored scene depth with the caller deliberately requesting discard. Source
  radius is 3,200 km, target is 64×64, camera altitude above the source page is
  2 m / 50 m / 400 km. Compared with f64 source views and actual Depth32Float
  samples, final observed worst component errors were 0.000122 px near the surface
  and 0.001446 px at 400 km (test envelope 0.02 px). These are fixture bounds,
  not a universal planetary depth-precision claim. All 136 mesh pixels and all
  4,096 water pixels were invalid. On/off runs preserve HDR RGB bits exactly
  across all six frames, including live CBT/ocean updates and projection/exposure
  changes. No subjective preference or visible improvement is claimed.
- Capture layout is `native-topology-v7-motion`: `render.motion_vectors` is
  separate from opaque raster and `render.scene_overlays`. Actual mode and
  texture bytes are recorded, not inferred from the requested switch.

### T3b verification and measured cost

Evidence: [native-temporal-signals-2026-10-21.json](artifacts/native-temporal-signals-2026-10-21.json).
The same release binary was run off/on/on/off at 1280×720, WORLD00's fixed
land target, 50 m radial eye clearance, epoch/step zero, HUD/audio off,
30 warmup + 60 measured frames per block. Hardware: Radeon 880M, Vulkan/RADV,
Mesa 26.2.5-arch1.1. The mode allocated 7,372,800 texture bytes when enabled
and zero when disabled. The enabled motion-scope medians were 0.366 and
0.197 ms (p95 0.411 and 0.434 ms); disabled empty timestamp intervals were
about 0.020 ms. These timings exclude the extra depth-store bandwidth, which
belongs to the opaque raster scope. They are not isolated shader throughput.
Whole-frame GPU medians ranged from 89.91 to 105.53 ms; asynchronous streaming,
scheduling and thermal noise prevent a stable whole-frame cost/speedup claim.
Each capture reported 368 visible patches; counts alone do not prove identical
resident data. Independent processes produced different PNG hashes, so those
screenshots are not a pixel-parity oracle. The fixed-resident GPU regression
above, rather than these screenshots, establishes unchanged RGB for its cases.
No blind subjective comparison was performed for this non-visible diagnostic
stage; neither aesthetic improvement nor full visual acceptance is claimed.

Both real native-window modes (HUD/no HUD) presented eight frames with motion
enabled and exited cleanly, in addition to the native CLI/hardware suite.
The required Bevy `--ignored` command ran but selected zero tests in this
checkout (there are no ignored tests in that crate); the ordinary render-feature
suite was run separately. Renderer hardware tests ran on the actual adapter,
not an adapter-less fallback. The first native suite failed three server cases
because its sibling server executable was missing; building `thessa-server`
and repeating the suite resolved that prerequisite failure.

### T2b delivered: primary jitter and depth/motion consistency

- `[temporal].jitter = false` is a separate diagnostic, **not TAA**. It requires
  resolved motion diagnostics (and therefore HDR). Unmet prerequisites resolve
  to zero jitter with a note. Enabling it prints a warning: the displayed image
  deliberately trembles because no reconstruction pass exists yet. It is not a
  production quality tier or a recommendation to enable temporal antialiasing.
- The existing `SceneMotion` owns the deterministic Halton sample index. The
  apps query it after acquiring a real frame and use the same sample for terrain,
  meshes and depth-based motion. Only `commit` after queue submission increments
  the index; queries, encoding, presentation and skipped/failed frames do not.
  Reset/resize restarts the sequence and clears the previous camera. Projection
  or exposure changes still invalidate motion independently of sample index.
- Primary terrain gets a separate 464-byte view uniform only when jitter is
  resolved; off allocates no additional uniform. Classification and clear-sky
  background retain the original unjittered buffer. Conforming topology reads
  `unjittered_clip_from_world` explicitly, even if given a jittered view block:
  jitter does not change its policy/generation or wake publication by itself.
  Shadows use their own projection; RT geometry/model/origin state is unchanged.
  Plume, sky layers and HUD retain their existing unjittered cameras.
- After the ordinary mesh/instance update, `update_primary_jitter` replaces only
  its raster view uniform. This costs an extra 464-byte upload per nonzero-jitter
  mesh consumer, but no new instance/shadow/RT publication; zero jitter is a
  no-op after the ordinary update. Existing callers remain unchanged. This
  ordering is required: an ordinary mesh update overwrites the sampled view.
- Motion reconstructs depth with the current **jittered** inverse and compares
  against the previous **unjittered** camera, subtracting current jitter from
  current UV. A stationary camera must therefore report zero motion despite
  changing primary samples. The two jitter coordinates occupy unused padding
  words in the existing 160-byte motion uniform; no new texture, pass or
  history allocation is added. `MotionParams.current_jitter` is validated
  against the existing ±0.5-pixel bound, not silently clamped.
- CPU coverage reconstructs known points with the matching inverse and rejects
  the incorrect convention. The real GPU oracle now tests seven frames, with
  both moving and stationary cameras, regular/conforming terrain, two water
  material cases, Low/High meshes, resets, resize and projection/exposure
  changes. The zero-jitter diagnostic still preserves HDR RGB bits. The jitter
  test requires primary RGB to change, rather than merely uploading an unused
  offset. Separate GPU tests require exact clear-sky image identity and stable
  conforming view-policy/generation across eight jitter samples.
- This remains an opaque signal test, **not** a ghosting/history acceptance
  result. In particular, unjittered translucent overlays currently depth-clip
  against sampled primary depth: the small edge mismatch is not a validated
  resolved-depth/reactive-mask solution. T4 must define that composition boundary,
  retain previous jitter for history lookup, validate disocclusion and epoch
  resets, and prove actual TAA quality before any SDK/FG integration.

### T4a delivered: backend-neutral resolve contract

`thessa_render_core::temporal_resolve` is the CPU reference for the future GPU
pass. It deliberately owns no texture, render graph or presentation state.

- `history_uv` accepts the current raster UV, jitter-free motion, current and
  previous jitter, extent and a history-grid convention. It produces the
  unjittered previous UV, the colour lookup UV and the previous-depth lookup UV.
  The depth address always includes previous jitter because previous depth is
  raster data. The preferred `ResolvedUnjittered` colour convention samples the
  previous accumulated image at the unjittered UV and therefore does **not**
  add previous jitter to colour. `JitteredScene` remains available for a raw
  previous raster image and adds previous jitter to both colour and depth.
- Both jitters are validated through the existing ±0.5-pixel bound before any
  lookup. A half-open image domain rejects UV `1.0` rather than clamping and
  reading the opposite edge. Out-of-bounds, non-finite, zero-age, invalid-motion
  and invalid-epoch candidates fall back to the current colour and reset age.
- `depth_matches` compares the previous depth sample with the depth obtained by
  projecting the current reconstructed surface point through the previous
  camera. The tolerance is an explicit caller policy derived from the depth
  format and reprojection error budget; the core does not invent a universal
  tolerance. This is disocclusion validation, not a claim that a single scalar
  depth is sufficient for every translucent-overlay composition.
- `neighborhood_bounds` computes YCoCg mean and standard deviation over the
  current 3×3 neighbourhood. `clip_history` clamps history to that box before
  blending. A zero-variance neighbourhood therefore rejects a differing history
  sample instead of hiding a ghost behind a long age.
- `history_weight` uses `1 / (age + 1)` with an explicit responsiveness floor
  (default 0.05) and a capped age. Rejected history starts again at age one.
  The policy is deterministic and frame-count based; it never derives from wall
  time, presented frames or simulation ticks.
- The CPU tests cover both history-grid conventions, stationary cameras with
  changing jitter, malformed jitter/UV inputs, depth disocclusion, all fallback
  reasons, clipping of an outlier, zero-variance rejection and weight/age
  convergence.

Local verification for T4a: 2,036 workspace lib/bin tests, 102 integration tests,
workspace Clippy with warnings denied, formatting and the architecture guard all
passed. These are CPU-policy tests; no GPU history or visual claim is implied.

T4a does **not** allocate history, store previous depth, modify display output
or claim antialiasing. T4b must implement the GPU counterpart against this
contract, and native integration must still solve the overlay/overlay-depth
boundary recorded below.

T2b evidence: [native-primary-jitter-2026-10-21.json](artifacts/native-primary-jitter-2026-10-21.json).
Final local gates passed: 2,029 workspace lib/bin tests, 102 integration tests,
79 renderer hardware tests and 43 native hardware/CLI tests, plus fmt, Clippy,
architecture, no-HUD compilation and the Bevy render-feature suite. The actual
GPU scene's worst motion component error against its f64 depth oracle was
0.001445 px moving and 0.000012 px stationary at 64×64 (0.02 px test envelope).
This is not a universal depth-precision or ghosting bound.

The same release binary was captured jitter-off/on/on/off at WORLD00's fixed
land target, 50 m radial eye clearance, 1280×720, epoch/step zero, HUD/audio off,
30 warmup + 60 measured frames per block. Metadata reported 90 submitted samples
per block and 368 visible patches. On Radeon 880M/Vulkan/RADV Mesa
26.2.5-arch1.1, motion-scope medians were 0.169/0.176/0.176/0.193 ms; whole GPU
medians ranged from 90.40 to 99.31 ms. No stable speedup/regression claim follows
from four blocks under asynchronous streaming and scheduling/thermal noise.
An additional muted sweep and eight-frame native HUD/no-HUD windows all exited
successfully with jitter enabled. These are runtime smoke captures, not TAA or
independent subjective visual acceptance.

#### T4 integration findings (not implemented)

- `DisplayTransform` retains a bind group for its own HDR view. A resolve cannot
  merely allocate a different texture: either the display must explicitly accept
  that source or the resolved result must be published back to the existing HDR
  composition target. History must be captured **before** plume/cloud/star/aurora
  overlays; the post-overlay display target is not a reusable opaque history.
- `plume_common.wgsl::unproject` and `sky_layers.wgsl::cloud_fragment` interpret
  scene depth with their own camera projection. Publishing unjittered resolved
  colour without also specifying compatible depth/occlusion sampling leaves a
  subpixel composition mismatch. This requires a measured sampled/resolved-depth
  policy, not an assumption that scalar depth is independent of the sample ray.
- `Scene::tick` already returns `geometry_changed` and `materials_changed`.
  Reuse those publication events for history invalidation rather than adding a
  second terrain observer. Add the missing cut/map/body/light/exposure epochs
  at the existing scene boundary; per-frame CBT identity alone is not history
  validity and must not force unrelated simulation work.

## 10. Regression and benchmark protocol

Every stage records:

- scene, target, epoch/step, camera path, extent, settings and executable hash;
- rendered-frame counts separate from presented/generated counts;
- whether depth and motion were valid per frame;
- history resets and their cause;
- native/no-temporal reference and matched temporal image;
- ghosting/disocclusion metric on scripted camera cuts, pans and CBT updates;
- GPU medians and p95 for primary visibility, resolve and presentation;
- peak temporal allocation, not just the steady-state history bytes;
- disabled-path allocation/work exactly zero;
- hardware/adapter/driver and whether the path is native Vulkan, Metal, WebGPU or
  an SDK fallback.

Single-order captures are descriptive, not stable performance acceptance. A
temporal improvement must be evaluated on image stability and ghosting, not only
on a brighter or sharper still frame.

## 11. Open questions

- Does the survey/TAA path need a fixed camera, or is camera-only reprojection
  acceptable for all static terrain while ocean/ships are invalid?
- What is the measured depth reprojection error at 2 m AGL and at planetary
  camera distances?
- Should exposure be applied before or after history, and how is a manual
  exposure change invalidated?
- Which CBT publication events are spatially bounded enough to preserve local
  history rather than resetting the whole frame?
- Which presentable UI elements must bypass upscaling and FG?
- Is a Vulkan FSR 3.1 adapter worth shipping before a native TAA reference, or
  only after the reference passes the regression matrix?

## 12. Non-goals

This document does not implement or claim:

- DLSS/FSR/XeSS/NRD integration;
- frame generation;
- motion vectors for dynamic meshes or displaced ocean;
- a new render graph or presentation architecture;
- replacement of the canonical terrain/CBT or plant systems;
- stable performance or visual acceptance.
