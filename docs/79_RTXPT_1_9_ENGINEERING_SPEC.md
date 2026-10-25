# Project Thessa — RT/FastRT: source audit RTXPT 1.9 and engineering specification

**Document:** `THESSA-RT-RTXPT-019`  
**Version:** 1.0 · **Date:** 2026-10-25  
**Status:** Proposed technical specification / implementation backlog; **not** evidence of implementation or benchmark results.  
**Language:** Rust 2024 + WGSL; renderer `wgpu 30`; primary target Vulkan/Arch, capability-gated multi-backend operation.  
**Scope:** `crates/render-wgpu`, `crates/render-core`, `crates/graphics`, `crates/fastrt`, `crates/fastrt-wgpu`, `crates/rcbt-core`, `crates/terrain-stream`, `apps/terrain-native` and their tests.  
**Pinned revisions:**
- **Thessa:** [`Mar2ianen/project-thessa@e820f423f2078939c4c3a857abedcf764d3c5ddc`](https://github.com/Mar2ianen/project-thessa/tree/e820f423f2078939c4c3a857abedcf764d3c5ddc), identified as the head of `feat/surface-visual-engineering` at the time of audit.
- **RTXPT:** [`NVIDIA-RTX/RTXPT@303b408ab81790a5051abd7573830b7a4112eb8e`](https://github.com/NVIDIA-RTX/RTXPT/tree/303b408ab81790a5051abd7573830b7a4112eb8e), release tag `v.1.9.0`, published 2026-10-23.

> **Evidence policy:** `[Txx]` = Thessa's own code/docs, `[Nxx]` = NVIDIA code/release/docs, `[Rxx]` = external API/research. Statements marked **observed** are verified in the linked revision; **inferred** means an engineering implication rather than benchmark proof; **proposed** means new requirement. This static audit did **not** compile the code, run hardware tests, benchmark FPS, or compare rendered images. Never present acceptance targets as existing measurements.

---

## 0. Executive decision

**Adopt an incremental, hybrid raster/RT upgrade path; do not port RTXPT's source or replace the planet renderer with its unified forward PT.** The strongest reference is **RTXPT 1.9 Intro Sample**: its nine-stage progression from deferred raster through RT AO/GI/reflections to indirect PT, SHaRC and ReSTIR DI is intentionally built for engines with an existing ray-tracing integration [N02–N04]. Use it as a conceptual progression, not copied code.

**Earliest engineering value:** (1) baseline instrumentation and invariant locks; (2) RT-specific ray-origin precision and secondary-hit texture LOD; (3) careful shader/material/RT-scene planning cleanup; (4) separable noisy signal + reconstruction; (5) *optional* stochastic GGX and one-bounce diffuse GI against reference images. **SHaRC/NEE-AT/large-scale light reservoirs/GPU-authored TLAS** are conditional, later work, not prerequisites for a viable Thessa renderer.

**Non-goals of this program:** copying NVIDIA shaders or SDK modules into MIT code; promising RTXPT's “+20%” in Thessa; supporting two million lights without a demonstrated scene requirement; mandatory DLSS/RTX-only reconstruction; requiring DXR-specific SER/OMM; replacing native sky/cloud/ocean/terrain ownership; making Bevy re-enter renderer crates.

### Decision gates

| Gate | Question | Evidence required | Default if not passed |
|---|---|---|---|
| G0 | Can we reproduce an RT-on/off baseline? | Hardware/capability report, captured settings and GPU timings, reference renders | Freeze visual feature changes |
| G1 | Does ray-origin offset fail in meaningful test cases? | Reproducible self-hit/leak A/B captures under planet-scale transforms | Keep current configurable bias; do not claim defect |
| G2 | Is LOD0 visibly/quantitatively harming secondary hits? | Texture-frequency sequence + ray-hit oracle + fixed exposure | Keep fast LOD0 fallback |
| G3 | Do stochastic reflections improve scene quality per cost? | Reference error and stability on moving cameras + p50/p95 GPU timings | Keep mirror/environment path |
| G4 | Can a native denoiser reconstruct the selected signal? | Denoiser error and motion/disocclusion tests without DLSS | Reject sparse sampling / GI in realtime |
| G5 | Is indirect lighting actually missing in representative scenes? | Controlled Cornell-like + coastal/night scenes and error budget | Postpone GI/cache |
| G6 | Is CPU instance/TLAS preparation limiting? | Timestamped CPU profiles, TLAS build/updates and backend capability audit | No GPU TLAS rewrite |

---

## 1. RTXPT 1.9: what the release actually does

**Verified** in release and code [N01, N03, N05–N09]:

1. **~20%** gain is NVIDIA's comparison of **RTXPT 1.9 against RTXPT 1.8.1** at comparable or better quality; not a general speedup of Vulkan ray tracing. Changed presets, bounce caps, subsampling and DLSS path contribute; it is not an isolated algorithmic A/B.
2. **Sub-1spp**: selected 75%, 50%, 25% of pixels; contribution weighting `1/p` on traced paths; default per-block Sobol `(0,2)` with Owen scrambling; also block-jitter, stratified and stochastic-compacted patterns. Only 1 spp path; **not** used with ReSTIR DI/GI. This targets the main PT noisy signal, not the raster framebuffer.
3. **Main pipeline:** vertex/skinning + BLAS/TLAS → early light processing → V-buffer/denoising-layer prepass → late local light sampling → ~1 SHaRC training ray per 4×4 render pixels → main path tracing → denoiser guides → DLSS Ray Reconstruction → postprocessing. Separate CPU/DX12 async-pipeline improvements do **not** carry over automatically to wgpu/Vulkan.
4. **SHaRC:** trains a spatially hashed radiance cache with path-state propagation; main tracing may terminate into a valid cached radiance result. It is deliberately **biased**, temporally lagged, and has cache-specific artifacts. Its main gain is signal quality per ray for *multi-bounce indirect lighting*.
5. **NEE-AT:** global/local adaptive, feedback-guided light importance selection; renderer supports up to `2*1024*1024` light records, though its own config advises aggregation/LOD to remain below **~500k** where practical. Emissive geometry is grouped into LODs. This is not the number of simultaneously shaded directional lights in Thessa.
6. **TLAS:** RTXPT generates TLAS instance descriptors on GPU each frame and optionally culls; default culling off. CPU optimizations also include scene render caches, parallel work and sparse uploads. Dynamic skinned BLASes are refitted in RTXPT; *wgpu 30 has a different AS API* and cannot be assumed to support the same pathway.
7. **Ray/denoiser precision:** corrected motion-guide handling, jitter/exposure bugs, per-hit error-scaled ray origin offsets; texture LOD via ray cones, specialized specular motion, denoising layers. These are separable engineering lessons for Thessa.
8. **Vulkan feature parity is incomplete:** Vulkan SER, OMM and new async compute prefix are not available like DX12; NVIDIA documents AMD/Vulkan issues. RTXPT is **Windows-oriented**; Vulkan support does not imply a native Linux reference executable.
9. **Two implementations in one repo:** main unified forward PT and separate **Intro Sample** (deferred/hybrid, nine named steps). Do not conflate their material quality, shader paths, denoisers or performance characteristics.

**Important subtlety:** the Intro Sample uses simplified secondary-hit materials and fixed texture LOD for clarity/throughput [N02], while the **advanced** RTXPT implements ray-cone material LOD [N03]. When comparing Thessa's secondary-hit LOD0, do not cite Intro Sample as if it already solved this.

---

## 2. Thessa: source inventory, real behavior and gaps

### 2.1 Renderer/RT ownership

- **Observed:** `crates/fastrt/src/lib.rs` implements a **pure geometry classification policy**: `RasterOnly`, `RtFull`, `RtProxy`, `RtConditional`, `Ignore`, with an explicit triangle budget, no BLAS/TLAS/dispatch [T01]. `fastrt-wgpu` supplies GPU batched slab ray–AABB intersections compared against an f64 CPU oracle [T02]. Neither is Thessa's full-scene production path tracer.
- **Observed:** hardware RT is in `render-wgpu/src/ray_scene.rs`, which creates per-primitive `wgpu::Blas`, one `wgpu::Tlas`, camera-relative instances and terrain BLAS slots; its `encode()` schedules an AS build and tracks basic build counts [T03]. `ray_query.wgsl` owns the inline ray query and visibility; `ray_scene.wgsl` owns reflected-hit lighting [T04–T05].
- **Observed:** `MeshRenderer::build_surface` decides admission with `ray_scene::plan_with_terrain` then specializes a WGSL shader string; the RT path is invoked by an ordinary mesh render pass. **It is not a generic primary-ray GI pass for the entire planetary terrain** [T06–T08].
- **Observed:** primary mesh visibility is rasterized; `rt_reflection` performs bounded reflected-ray iteration. It starts with a mirror direction (`reflect`), shades hit points with shared materials/environment and up to **three** scene directional light slots, traces visibility, and continues through another mirror direction. `RT_ROUGHNESS_CUTOFF=0.6`; beyond cutoff the environment approximation replaces tracing; max bounces resolve to **1–4** [T05, T09]. **This is not a stochastic BSDF path tracer.**
- **Observed:** current secondary mesh texture sampling uses `textureSampleLevel(..., 0.0)`; resident terrain hit path explicitly declares a **zero material footprint** and defers ray-cone filtering [T03, T10]. No primary surface derivatives exist for secondary rays, making RT LOD a real correctness/quality issue to evaluate.
- **Observed:** the shared terrain source exposes **1089 vertices / 2048 triangles per generated patch**, GPU-resident geometry and dirty-only BLAS maintenance. Stable slot identity, geometry epochs and material addressing avoid a duplicated terrain CPU mesh. Near-field ocean secondary hits incorporate shared wave/material information where supported. Do not discard this pipeline in pursuit of generic PT [T03, T07–T08, T11].
- **Observed:** `RayScene::update` builds CPU-side transforms; `prepare()` compares camera-relative matrices and marks TLAS dirty on change; camera motion can therefore force TLAS rebuild even when BLAS is unchanged [T03, T11]. **Observed API constraint:** wgpu TLAS instances are host-managed; the upstream ray tracing API is experimental and its documented update/refit support is limited [R01].
- **Observed:** scene admission estimates bytes and enforces `rt_scene_memory_bytes` and TLAS/BLAS/device limits; resolved scene budget is clamped to **1–128 MiB**, instance maximum to **16,384** [T03, T09]. **This is a policy reservation, not measured driver AS allocation** [T08]. RTXPT's **262,144 resident geometries** counts a different resource; no valid apples-to-apples ratio follows.
- **Observed:** material samples at RT hits are wired by generating shader declarations and `switch` cases **per retained texture binding**. The planner rejects texture/sampler binding overflow and leaves an explicit fallback instead of sampling arbitrary content [T03, T06]. This bounds functionality and can cause shader-specialization churn when material layout changes.

### 2.2 Raster shadows, lighting and temporal state

- **Observed:** `ShadowAtlas` renders raster depth, including generated terrain and opaque mesh; ray visibility multiplies atlas/cloud visibility with RT occlusion when active. Disabling one shadow provider must never brighten an occluded ray [T04, T12].
- **Observed:** Thessa has environment radiance and low-order SH diffuse support, an atmosphere/cloud integration and a spectral ocean path [T05, T13]. **Not equivalent** to multi-bounce diffuse GI or an adaptive emissive-light sampler.
- **Observed:** `temporal_resolve.wgsl` is a TAA-style compute resolve with history, motion/reprojection, previous depth, YCoCg variance clipping, validity masks and max history age **64**; motion-invalid surfaces do not seed reusable history [T14–T15]. It is **not** a specialized stochastic indirect-radiance denoiser.
- **Observed:** `ResolvedGraphicsSettings` has a `rt_reflection_samples` field (quality maps to 1/2/4), but **no read of that field was found in the inspected production paths** `ray_scene.rs`, `ray_scene.wgsl`, `mesh.rs`. Search the complete repository before declaring it dead; do not advertise 4-sample RT based solely on settings metadata [T03, T05–T06, T09].
- **Observed:** the GPU frame cost test `fixed_source_ray_query_frame_cost` is marked `#[ignore]`, while `ray_scene.rs` contains extensive conditional GPU oracles for reflection, materials, geometry membership, dirty BLAS, shadows and exact RT-off environment behavior [T16]. Passing existing unit tests does **not** establish regression-free p95 GPU frame time.
- **Observed:** Bevy retirement remains incomplete, but the native renderer has a defined owner; must not bring Bevy/Solari back into engine code [T17].

### 2.3 Comparative matrix

| Dimension | Thessa@pinned SHA | RTXPT@1.9 | Consequence |
|---|---|---|---|
| Primary visibility | Raster scene and terrain | Main sample: raygen + V-buffer; Intro: G-buffer | Keep Thessa raster ownership |
| Acceleration structures | wgpu BLAS/TLAS, dirty resident terrain | Native DXR/Vulkan AS, GPU TLAS instance generation | Optimize only after profile + API gate |
| Secondary surface hit | Shared mesh/terrain material, fixed LOD0 | Advanced: ray-cone LOD, material permutations | Add measurable ray-footprint path |
| Secondary reflection direction | `reflect()`, roughness cutoff | BSDF/VNDF stochastic path | Optional new quality mode, not replacement by default |
| Diffuse indirect | SH/environment approximation | NEE + multi-bounce + SHaRC | Introduce a separate GI signal if required |
| Visibility | Raster cascades + optional RT rays | RT visibility + OMM, denoising layers | Preserve existing raster/RT shadow interlock |
| Temporal | TAA history+depth+YCoCg | Neural DLSS-RR/NRD & guides | Build owned radiance resolve before sparse sampling |
| Quality oracle | GPU image oracles, ignored timing probe | Accumulating reference + image regression tools | Formalize reproducible reference/capture harness |
| License | MIT in renderer crates | NVIDIA RTX SDK source-available | No direct code import without independent legal clearance |

---

## 3. Invariants — MUST NOT regress

These are requirements across all work packages. A task is incomplete when it meets a visual target but breaks any invariant.

1. **Physical authority:** graphics must not mutate simulation state, ephemerides, ocean authority, terrain height, server flight state or material residency epochs. Only consume published snapshots.
2. **Coordinate precision:** authoritative transforms in `f64` SI; subtract camera origin in `f64` *before* GPU narrowing; ray distances/biases have stated units. Surface precision must not vary with absolute planetary position at fixed camera-relative separation.
3. **Zero work when disabled:** RT off ⇒ no BLAS/TLAS build, RT buffers, ray dispatch, shader RT workload; raster image remains equivalent to its historical reference within an explicitly stated tolerance (bit-exact in existing fixtures).
4. **Explicit capability negotiation:** no blind experimental feature enable, no silent claims of RT on unsupported adapters. Preserve `Off/Local/Full/Auto`, surface flags, meaningful fallback reasons, and include reason in telemetry. Unsupported hardware should use the raster path, not panic.
5. **Scene membership consistency:** only one authoritative geometry/terrain ownership; no duplicate streamer, BVH registry or untracked GPU geometry copy. Dirty BLAS/instance updates follow published epochs, not wall time.
6. **No stale occluders:** failed admission, retired patch or empty TLAS cannot keep casting shadows/reflecting. A rejected update must preserve the last valid published state until explicit recovery.
7. **Material/color:** sRGB base textures, linear metal/roughness/AO; correct texture wrap, normal orientation, dielectric F0, roughness, transmission and ocean optical conventions; one shader-side material contract.
8. **Light-energy accounting:** deterministic directionals/environment must not be re-added on a secondary bounce or after a cache lookup; throughput, visibility, estimator PDF and exposure must have non-overlapping ownership.
9. **History validity:** clear/invalidate on resize, camera cut, geometry epoch, light/material changes when necessary, dynamic occluder visibility, origin rebasing, wrong motion, depth discontinuity. Motion-invalid cannot create stale histories.
10. **Memory boundedness:** no unbounded shader variants, buffer growth, per-frame CPU readback, runaway sample queues, or logging of fabricated GPU timings. A failed budget check reports both requested and resolved work.
11. **Licensing:** Thessa MIT code stays independently implemented and distributable; third-party SDK restrictions are isolated to optional integration boundaries, not transitive into engine crates.
12. **Reproducibility:** each acceptance capture records pinned commit, adapter, driver/API, feature set, resolution, settings, scene/camera/time/weather, history warmup, timings and deterministic/random seeds.

---

## 4. Work packages and acceptance criteria

### WP0 — Reproducible instrumentation and controlled oracle (P0, dependency for all)

**Owner:** `render-wgpu` instrumentation + `apps/terrain-native` capture harness.  
**Why:** existing cost test is ignored; comparing RTXPT “20%” or attributing regressions without hardware-matched measurements is invalid [T16, N01].

**Implement:**
- Add a stable `RtFrameCapture` schema (`serde`/JSON or equivalent) with GPU identity, requested/resolved capabilities, `wgpu` backend, scene+material/terrain generation IDs, geometry instance counts, dirty BLAS count, TLAS build count, AS planner bytes, peak observed allocs *only if obtainable*, surface quality, RT bounces, ray count/samples, shader variant key, enabled passes.
- Timestamp, when supported: mesh raster, shadow atlas, terrain raster, AS build, reflective traversal/shading, temporal resolve, postprocess; retain **unavailable** on adapters without timestamp queries (no CPU time labeled as GPU time).
- Add CPU spans for `RayScene::plan/update/prepare`, shader compilation, binding changes, terrain publication and frame submission; emit p50/p95/p99, driver info and warmup policy.
- Pin at least 6 deterministic scenes: pure raster/RT-off, mirror+off-screen target, metal/rough patterned material, high-frequency textured terrain secondary hit, patch churn with moving camera, large static planet/orbit and low-altitude coast + day/night lighting.
- Provide an offline high-quality reference path (may initially be a slow opt-in CPU/GPU independent integrator, not RTPT code). Fixed seeds and accumulation counts; explicitly mark any clamping/finite-bounce/reference bias.
- Promote a hardware cost probe out of `#[ignore]` only under a labeled optional GPU test suite. CPU CI must not fail because no suitable RT adapter exists. Keep emulator/software adapter coverage for logic/tests.

**Acceptance:** scripts capture identical scene state twice with RT off/on on one device; reports contain raw values, no inferred GPU timings; run-to-run variance and warmup strategy visible; compare image pairs with fixed exposure and quantitatively report error, not merely screenshots. RT-off fixture remains exact. No performance claim until G0 passes.

### WP1 — Robust ray origin and hit error bounds (P1, after WP0)

**Owner:** `ray_query.wgsl`, `ray_scene.wgsl`, terrain hit path and neutral numeric oracle.  
**Observed code:** fixed `RT_BIAS` offset for reflection, `RT_BIAS/cosine` shadow-ray t-min with an explicit avoidance of sideways offset [T04–T05]. NVIDIA 1.9 adds a hit-position error scale [N01]; an independent, publicly described robust offset approach estimates object reconstruction + transform + traversal error and offsets along **geometric** normal [R02].

**Implement:**
- Introduce an engine-owned `RaySpawnBounds` contract: barycentric hit point, triangle edges, instance transform/error bounds, camera-relative point, geometric normal, outgoing side, explicit min/max travel.
- Independently implement and test a *conservative* error-bound offset; do not copy RTXPT source. Guard non-finite/degenerate triangles, extreme grazing angles, large triangles, tiny creases and interior/exterior transmission separately. Preserve non-RT/fallback branch.
- Validate shadow connection rays for self-hit and detached-light cases; keep geometric normals distinct from shading normals and ocean-wave normals.
- Collect an oracle set of adjacent triangles, concave corners, 0.01 m gaps, orbital-scale camera positions, object transforms with scale, near-horizon grazing direction, dynamic terrain anchors and mirror chains. Mark high-risk images for visual inspection.

**Acceptance:** no new self-hits, leaks, misses through thin valid geometry or brightness changes in RT-off controls; precision acceptance via scene-specific tolerance recorded in fixtures. Performance and ray count measured; if cost exceeds agreed budget without visual benefit, retain bias mode as fallback. **Do not invent a universal error-free bound for all drivers.**

### WP2 — Ray-cone / footprint-aware secondary material LOD (P1, after WP0; can parallel WP1)

**Owner:** `ray_scene.wgsl`, `terrain_ray.wgsl`, `ray_scene::shader`, shared material kernel.  
**Observed:** fixed LOD0 and zero terrain footprint [T03, T10]; RTXPT Advanced uses ray-cone hit LOD [N03], and independent paper(s) specify approaches and public-domain examples [R03–R04].

**Implement:**
- Define `RayFootprint` (world-space cone width/spread and/or hit distances), trace/reflection update policy, and conversion to texture footprint. Primary raster may seed footprint from pixel angular size and surface derivatives; secondary rays must **not** call fragment `dpdx/dpdy` or silently assume LOD0.
- Compute separate LOD for imported mesh textures and virtual/world-addressed terrain material directory. Clamp to available resident mip range; parent/edge fallback uses existing residency rules. Include anisotropy decision; no blurry material page when a finer valid level exists.
- Avoid increasing storage-binder pressure without capacity negotiation. Keep `RT_LOD_MODE=legacy` for A/B and platform safety; default flip requires evidence.

**Acceptance:** mip reference tests over checkerboard/sinusoid textures, varying roughness/angle/scale and distances. Compare temporal frequency stability and sharpness against super-sampled reference; check no regressions in texture wrap, colorspace, normal maps, ocean roughness, invalid UVs or missing residency. Time texture sampling on reflective pixels and collect per-mip hit histogram. **No precommitted FPS gain.**

### WP3 — RT admission, FastRT policy convergence and binding scalability (P1/P2)

**Owner:** `fastrt` policy, `ray_scene::plan_with_terrain`, graphics resolution + stats; no new generic scene tree.  
**Observed:** pure `RtClass` exists separately from real planner [T01, T03]; AS reservations are estimates, and shader generation scales with distinct texture bindings [T03, T08].

**Implement:**
- Document whether `RtClass` is intended as live policy or a dormant stage-one prototype; if live, add *an adapter* from `RtClass` decisions into existing `RayScene::Plan` using the current terrain/mesh owners. Do not instantiate a second TLAS or duplicate render mesh source.
- Preserve immutable primitive IDs + live scene epochs; distinguish visual culling from RT occlusion participation. `RasterOnly` objects should **not** secretly remain reflection/shadow occluders; `RtProxy` must have an explicitly validated, separately measurable representation; `RtConditional` must resolve deterministically per frame.
- Report estimated bytes versus driver actual memory **when API supports actual**. For unavailable driver AS sizes, report `estimated/unknown`, never label as VRAM measurement. Define hard host allocation limits and bounded shader variant cache.
- Evaluate bindless/texture arrays/material indirection only after counting the actual failure frequency of generated bindings. Use capability-specific implementation; never assume DX12 `ResourceDescriptorHeap` maps to wgpu WGSL.
- Audit `rt_reflection_samples` end-to-end (definition, every read, planned dispatch/samples, telemetry), either wire to a verified workload or deprecate/rename metadata, documenting migration.

**Acceptance:** no duplicate geometry copy for terrain, no new work under RT-off, rejected full partition leaves no stale occluders, source mutation/reordering/dirty epochs handled, budget exceeds return explainable reason, no invalid shader generated at binding cap. Tests cover at least 0/1/max/over-limit instances and textures; RT stats reflect actual encoded work.

### WP4 — Separate stochastic reflection signal and portable denoiser (P2/P3, after WP0–WP2)

**Owner:** new isolated reflection effect and neutral `render-core` policy; `render-wgpu` backend.  
**Observed:** current pixel reflection is a deterministic perfect mirror mixed with environment, not GGX VNDF random sampling [T05]; historical TAA is not a robust noisy-signal denoiser [T14]. RTXPT Intro Step 3 shows a useful incremental alternative [N02], with sampling literature [R05].

**Implement:**
- Keep current deterministic reflection path as `legacy_mirror` for fallback and low-latency devices.
- Add explicitly opt-in `stochastic_ggx`: GGX **visible-normal distribution** direction sampling with validated PDFs, BRDF/pdf throughput and directional incidence; handle metallic/dielectric F0, roughness, edge conditions, backfaces, thin-surface/transmission policy, multiple importance sampling (if combining light/environment strategies) and terminated/invalid paths.
- Return **separate linear HDR radiance** from a secondary-hit pass; do not blindly add Fresnel/environment/primary AO twice. Preserve shared material model and correct secondary-hit LOD.
- Separate guide state: normal, roughness, depth, primary motion, ray/hit distance, specular motion or explicit invalid marker, geometry/material/light epoch. Denoiser must handle disocclusions, thin reflectors and changed camera origin. Design HDR preprocessing/exposure so variance/reprojection are not dependent on display tone mapping.
- Implement or integrate a **portable, optional** variance-guided temporal/spatial resolve (e.g. independent SVGF-inspired path), preserving the existing TAA as the final-image resolve. Gate all advanced paths by device feature and memory budget.

**Acceptance:** reference scenes show reduced specular-lobe error relative to deterministic mirror on rough materials, with fixed seeds and sample counts; no persistent ghosting on moving objects or changing illumination; no noise explosion with GI off; no double counting; no measurable extra work in `legacy_mirror` or `RT_OFF`. Record p50/p95 GPU cost and memory at 720p/1600p and one native resolution. If reconstruction fails, keep mode opt-in rather than shipping a visibly unstable 25% sampling preset.

### WP5 — Minimal unbiased-ish GI/reference estimator, then controlled sparse sampling (P3)

**Owner:** separate `rt_gi` pass (not forced into terrain shader) and reference integrator.  
**Source route:** RTXPT Intro Step 2 introduces a cosine-weighted diffuse secondary ray, Step 4 adds sampled direct lighting, Step 6 combines diffuse/specular as a stochastic indirect pass [N02].

**Implement:**
- Start with **one cosine-weighted diffuse ray per selected primary surface** and shading at a hit using existing direct-light/environment model. At each hit evaluate `BRDF × incoming radiance × cos / PDF`; track emitter visibility, material emission and term ownership. Avoid dependence on the 3-slot primary directional representation when extending light types; add backend-neutral light record if/when needed.
- First scope: selected opaque mesh and near-field terrain receiver paths with valid normals, materials and motion. Night lights/emissive secondary-hit semantics are *not* assumed complete; require explicit integration with the existing night-light material authority [T17].
- Add brute-force reference mode with fixed scene/camera, >=2 quality levels of sample accumulation (numbers must be recorded), error maps, conservation/reciprocity sanity tests. Do **not** call the finite-bounce, firefly-clamped Intro reference unbiased ground truth [N02].
- Then evaluate sample selection (full, 1/2, 1/4, 3/4) in a **separate noisy radiance pass**. Use stratified/Sobol-like reusable independent implementation, explicit sample-selection probability and `1/p` contribution, correct cleared/invalid signal on unsampled pixels. Preserve unbiased *estimator* expectation before clamping/denoising; separate denoiser bias in reports.

**Acceptance:** diffuse color bleeding and off-screen indirect occlusion appear only where predicted by reference geometry; path throughput is finite and nonnegative, no double counting; environment/no-hit handling matches controls, normal incidence and thin-surface tests pass. Variance/MSE vs a recorded reference improves with accumulated samples; inspect moving-camera histories, invalid motion and disocclusion. Speedups are A/B results, not guaranteed ratios.

### WP6 — Radiance cache evaluation, NEE-AT and ReSTIR only on measured need (P4, after WP5)

**Owner:** separate spatial cache/lighting modules, independently implemented; no SDK text moved into MIT crates.  
**Source route:** SHaRC train/consume paths in RTXPT Advanced [N03, N07], bias/lag documented in its Design [N03]; Intro Steps 5, 7, 8 demonstrate progressive use [N02].

**Implement only if:** GI denoiser still loses physically important secondary signal at representative cost, or measured scene has enough analytic/emissive lights to justify resampling.

- Evaluate sparse *world-space* radiance cache with entry identity keyed to **planet coordinates + clip/LOD + geometry/material/light epochs**, not bare camera-relative float position. Allow varying world-space cell size by scale and separate local/planetary domains; bound collision rate/memory, and define origin shift semantics.
- Implement stable read/write generation and invalidation, adaptive age, light-change convergence tests and cache disable control. Treat cache as an intentionally biased estimator; quantify lag after a lighting toggle. Never let a cache hit duplicate the hit's emission or direct term.
- Start direct-light sampling with a simple alias/CDF or RIS for **the actual number and power distribution of scene lights**; only consider ReSTIR DI with explicit reservoir identity, motion validity and disocclusion tests. NEE-AT is an advanced alternative, not a prerequisite for one to three stellar directional sources.

**Acceptance:** cache-on materially improves reference error per GPU millisecond in at least two representative scenes without intolerable temporal lag, leaking across planets or false brightening. Report collision rate, cache hits, false rejects, invalidations, bytes, build/query/update time, and age distribution. If negative, keep experimental, **do not turn on by default**.

### WP7 — GPU-authored TLAS / AS management and large-instance work (P4/P5, strict API gate)

**Owner:** `render-wgpu` hardware-backend capability review and profiler.  
**Observed:** Thessa owns host `Vec<wgpu::TlasInstance>`, camera-relative instance lists and calls `build_acceleration_structures` [T03]. RTXPT GPU instance descriptors and optional culling are native-engine decisions [N01]. Upstream wgpu ray-tracing feature is experimental; its documented `Tlas` stores host instances and update/refit limitations remain [R01].

**Implement:**
1. Benchmark CPU `update/prepare`, staging writes, TLAS build GPU time separately for static camera, moving camera, many moving meshes, terrain slot churn and empty TLAS, at 100 / 1k / practical max instances.
2. First optimize *within public wgpu API*: stable sort/layout, dirty key and revision tracking, persistent staging reuse, avoid rebuilding BLAS unnecessarily, compact/cull only if occlusion semantics preserved. GPU frustum culling must not remove **off-screen shadow/reflection blockers**.
3. Only if G6 proves bottleneck and a supported API can express GPU-produced instance descriptors, prepare an RFC for backend-specific Vulkan/native escape hatch. Require lifecycle/synchronization/feature fallback contract and cross-backend equal-image oracles. Do not attempt unsupported `wgpu` calls or claim refit/compaction exists.

**Acceptance:** measured reduction of CPU frame time or AS time on representative workload, not just moving instruction count from Rust to a compute shader; all platform fallbacks preserved. Driver AS memory and work scheduling verified where measurable.

---

## 5. Execution order, dependencies and merge gates

```text
           [WP0 instrumentation/reference]
                    |
             +------+-------+
             |              |
        [WP1 ray bias] [WP2 RT texture LOD]
             |              |
             +------+-------+---- [WP3 planning/policy/bindings]
                    |
         [WP4 stochastic reflection + denoiser]
                    |
             [WP5 minimal GI]
                    |
        [WP6 SHaRC / NEE only if useful]

[WP7 GPU-authored TLAS] <-- independent profile/API gate after WP0, no dependency for GI
```

**Per-PR rule:** one substantive rendering change + its invariants/tests + capture delta. Do not combine new GI, a denoiser, scene indexing, TLAS backend change and atmospheric redesign into one PR. Every PR must include rollback flag/fallback, source comparison and a minimal independently reproducing GPU fixture.

### Required output for every task

1. Change list with source paths, API/ABI/schema changes and pin SHA.
2. Unit/integration/GPU test result matrix: passed/failed/skipped by adapter and reason.
3. `before.json` / `after.json` benchmark artifacts, raw timings + **same resolution and quality**.
4. Image diff including HDR radiance where relevant; static and animated sequence, fixed exposure and seeds.
5. Non-regression checklist for all §3 invariants and feature-off paths.
6. Remaining risks, unsupported GPUs/backends and explicit negative results.

### Bench matrix / comparability contract

| Scene/case | Resolution | Compare | Primary metric |
|---|---|---|---|
| RT-off control | 1280×720 + native | unchanged branch vs patch | bit-exact/accepted pixel reference + pass count 0 |
| Mirror off-screen object | 1280×720 | RT off/on; legacy/new | error vs reference, ray time |
| Rough material and checkerboard | 1280×720, 2560×1600 | LOD0/LOD, specular modes | temporal error, aliasing, ms |
| Moving coastal camera | native + orbital distance | bias/LOD/history changes | self-hit/leak count, flicker, p95 |
| Terrain slot churn | 1280×720 | baseline/planner change | BLAS rebuilt, TLAS time, memory |
| Day/night and changing light | 1280×720 | GI/cache on/off | dynamic convergence / ghosting |
| Dense instances | 100/1000/available max | host TLAS optimization | CPU prepare and GPU AS timing |
| No ray-query adapter | supported raster size | RT requested/off/auto | capability explanation, stable fallback |

**Do not over-specify global millisecond/FPS acceptance thresholds before WP0.** Choose and record scene/device-specific thresholds from measured baseline; validate reference error and temporal stability *in addition* to p50/p95 time. In reporting, distinguish quality-regressed “faster preset” from same-quality optimization.

---

## 6. Explicit test contracts and edge cases

- **Geometry/precision:** `f64` position roundtrip, 0-volume AABB permitted, invalid infinities rejected, overlapping/sliver triangles, co-planar/micro-gap surfaces, local and orbital coordinates, nested mirrors, material shading normal vs geometric normal, reused terrain slot with changed epoch.
- **Membership/revisions:** add/remove/deform mesh, move camera only, dirty one of N patches, material-only update, canceled scene publication, insufficient instance budget, empty TLAS, BLAS resource borrowing after immutable mesh replacement.
- **Sampling math:** normal/PDF finite and positive where used; BSDF/PDF conservation fixture; cosine-weighted hemisphere test; sample probability p∈(0,1], `1/p` compensation, expectation vs dense reference before denoising, no infinitely weighted samples when p→0.
- **Temporal:** history invalidate on cut/resize/material/light/geometry epoch; moving mirror reflecting moving target; ocean normal changes; high-velocity camera; bilateral filter refusing cross-edge history; null pixels must not keep stale radiance.
- **Lighting:** three directions active, all off, single sun, shadow atlas absent/present, cloud transit, off-camera blocker, emissive material on secondary hit, day/night toggles; cache disabled must match non-cache accounting.
- **Platforms:** ray-query Vulkan supported; adapter with RT no timestamp; no ray-query; software adapter; low storage-bindings; feature upgrade error; empty scene.
- **Regression:** scene with zero reflection geometry, fully rough material, no RT memory, max instance count, texture binding cap, RT disabled. Must not generate an invalid shader, allocate a BLAS, or silently substitute another material.

**Minimum correctness evidence**: CPU oracle at every numerically sensitive boundary, GPU oracle where supported, and comparison to fixed reference images. A GPU test that explicitly skips on an unsupported adapter is **not** a passing RT test and must be reported as *skipped*.

---

## 7. Open risks / architectural choices requiring an RFC

1. **Who owns stochastic primary visibility?** For now the existing raster renderer. If a general GI pass needs G-buffer/motion materials, define a neutral render target/guide contract, do not silently teach every terrain shader to dispatch rays.
2. **One TLAS / multiple effects.** `RayScene` is already a shared terrain/mesh AS; preserve that. GPU culling for camera visibility conflicts with off-screen occlusion. Define separate *visibility and caster participation masks* if needed.
3. **Ray cones for curved/deforming water.** Conventional triangular footprint may not capture spectral-normal frequency; introduce separate material bandwidth floor or measured roughness filtering rather than falsifying the geometric cone.
4. **Multi-scale cache keys.** Global `f32` voxel/hash coordinates leak precision; distinguish local camera space from stable body-fixed/planet IDs. Floating-origin changes must not make a valid location appear unrelated or vice versa.
5. **Denoiser licenses/performance.** DLSS-RR/NRD are not safe default dependencies for portable MIT, and ordinary TAA is insufficient as a substitute. Decide *separate noise guidance* before claiming sub-1spp support.
6. **Backend-dependent AS.** wgpu `Tlas` has a host-side API and experimental semantics. Do not make a Vulkan-native AS fork before profiling and a documented GPU-backend boundary; stay within `render-wgpu` and the existing `wgpu-exp` consent module.
7. **Memory observability.** Scene planner's 128-MiB reservation is not a guaranteed driver ceiling. Where allocation size is opaque, report conservative estimate and adapter budget separately, never a fabricated resident count.
8. **Copyright/derivative risk.** RTXPT's SDK license grants *restricted* rights to samples and binary distribution, not an MIT/Apache grant. Direct HLSL-to-WGSL transliteration of SDK code risks forming a derivative and should not enter MIT Thessa. Prefer papers, algorithms, own equations/tests and license-compatible references. No legal assurance is given by this engineering note [N09].

---

## 8. Evidence ledger — primary sources pinned to revisions

### Thessa (source inspected directly)

- **[T01]** [`crates/fastrt/src/lib.rs`](https://github.com/Mar2ianen/project-thessa/blob/e820f423f2078939c4c3a857abedcf764d3c5ddc/crates/fastrt/src/lib.rs) — policy-only `RtClass`, full/proxy/conditional classification; explicitly no AS or dispatch.
- **[T02]** [`crates/fastrt-wgpu/src/lib.rs`](https://github.com/Mar2ianen/project-thessa/blob/e820f423f2078939c4c3a857abedcf764d3c5ddc/crates/fastrt-wgpu/src/lib.rs) — batched ray/AABB slab compute, f64 CPU oracle and storage/dispatch limits.
- **[T03]** [`crates/render-wgpu/src/ray_scene.rs`](https://github.com/Mar2ianen/project-thessa/blob/e820f423f2078939c4c3a857abedcf764d3c5ddc/crates/render-wgpu/src/ray_scene.rs) — `plan`, `plan_with_terrain`, `shader`, `RayScene::update/prepare/encode`, instances, hit resources, per-primitive BLAS, TLAS, policy budgets.
- **[T04]** [`crates/render-wgpu/src/ray_query.wgsl`](https://github.com/Mar2ianen/project-thessa/blob/e820f423f2078939c4c3a857abedcf764d3c5ddc/crates/render-wgpu/src/ray_query.wgsl) — actual WGSL inline ray query and RT/raster/cloud visibility interlock.
- **[T05]** [`crates/render-wgpu/src/ray_scene.wgsl`](https://github.com/Mar2ianen/project-thessa/blob/e820f423f2078939c4c3a857abedcf764d3c5ddc/crates/render-wgpu/src/ray_scene.wgsl) — reflection direction, roughness cutoff, direct terms, multiple bounces, secondary hit shader.
- **[T06]** [`crates/render-wgpu/src/mesh.rs`](https://github.com/Mar2ianen/project-thessa/blob/e820f423f2078939c4c3a857abedcf764d3c5ddc/crates/render-wgpu/src/mesh.rs) — render-pass composition, scene ownership, RT encode, shader specialization and render target usage.
- **[T07]** [`crates/render-wgpu/src/terrain_ray.rs`](https://github.com/Mar2ianen/project-thessa/blob/e820f423f2078939c4c3a857abedcf764d3c5ddc/crates/render-wgpu/src/terrain_ray.rs) — resident terrain BLAS and dirty slot publication.
- **[T08]** [`docs/39_BEVY_EXIT_AND_ENGINE_MIGRATION.md`](https://github.com/Mar2ianen/project-thessa/blob/e820f423f2078939c4c3a857abedcf764d3c5ddc/docs/39_BEVY_EXIT_AND_ENGINE_MIGRATION.md) — explicit native renderer, terrain ownership, geometry/memory planning notes and requirements not to copy external implementation.
- **[T09]** [`crates/graphics/src/resolve.rs`](https://github.com/Mar2ianen/project-thessa/blob/e820f423f2078939c4c3a857abedcf764d3c5ddc/crates/graphics/src/resolve.rs) — capability gates, scene memory 1–128 MiB, max instances 16,384, bounces 1–4, `rt_reflection_samples` settings.
- **[T10]** [`crates/render-wgpu/src/terrain_ray.wgsl`](https://github.com/Mar2ianen/project-thessa/blob/e820f423f2078939c4c3a857abedcf764d3c5ddc/crates/render-wgpu/src/terrain_ray.wgsl) — shared material filtering, secondary hit footprint 0, explicit ray-cone TODO.
- **[T11]** [`crates/render-wgpu/tests/ray_scene.rs`](https://github.com/Mar2ianen/project-thessa/blob/e820f423f2078939c4c3a857abedcf764d3c5ddc/crates/render-wgpu/tests/ray_scene.rs) — RT geometry/sea/light/material oracles, dirty BLAS, fallback, the ignored full-frame GPU cost test.
- **[T12]** [`crates/render-wgpu/src/shadows.rs`](https://github.com/Mar2ianen/project-thessa/blob/e820f423f2078939c4c3a857abedcf764d3c5ddc/crates/render-wgpu/src/shadows.rs) — layered raster shadow atlas.
- **[T13]** [`crates/render-wgpu/src/environment.rs`](https://github.com/Mar2ianen/project-thessa/blob/e820f423f2078939c4c3a857abedcf764d3c5ddc/crates/render-wgpu/src/environment.rs) — environment radiance binding and SH diffuse inputs.
- **[T14]** [`crates/render-wgpu/src/temporal_resolve.wgsl`](https://github.com/Mar2ianen/project-thessa/blob/e820f423f2078939c4c3a857abedcf764d3c5ddc/crates/render-wgpu/src/temporal_resolve.wgsl) — GPU temporal resolve and disocclusion.
- **[T15]** [`crates/render-core/src/temporal_resolve.rs`](https://github.com/Mar2ianen/project-thessa/blob/e820f423f2078939c4c3a857abedcf764d3c5ddc/crates/render-core/src/temporal_resolve.rs) — backend-neutral CPU resolve oracle, age/clipping rules.
- **[T16]** [`tests/ray_scene.rs#fixed_source_ray_query_frame_cost`](https://github.com/Mar2ianen/project-thessa/blob/e820f423f2078939c4c3a857abedcf764d3c5ddc/crates/render-wgpu/tests/ray_scene.rs#L1882-L1906) — `#[ignore]` 1280×720 frame cost diagnostic.
- **[T17]** [`docs/66_BEVY_RETIREMENT.md`](https://github.com/Mar2ianen/project-thessa/blob/e820f423f2078939c4c3a857abedcf764d3c5ddc/docs/66_BEVY_RETIREMENT.md) — incomplete Bevy retirement, native scene/material/night lighting boundaries.
- **[T18]** [`crates/render-wgpu/src/mesh.wgsl`](https://github.com/Mar2ianen/project-thessa/blob/e820f423f2078939c4c3a857abedcf764d3c5ddc/crates/render-wgpu/src/mesh.wgsl) — raster material and reflection hook/lighting sum.

### NVIDIA RTXPT v1.9.0

- **[N01]** [RTXPT v.1.9.0 release notes](https://github.com/NVIDIA-RTX/RTXPT/releases/tag/v.1.9.0) — 23 October 2026, performance methodology caveat, sparse patterns, GPU TLAS, shader model requirements, direct limits and breaking changes.
- **[N02]** [`Docs/intro-sample.md`](https://github.com/NVIDIA-RTX/RTXPT/blob/303b408ab81790a5051abd7573830b7a4112eb8e/Docs/intro-sample.md) and [individual steps](https://github.com/NVIDIA-RTX/RTXPT/tree/303b408ab81790a5051abd7573830b7a4112eb8e/Docs/intro) — hybrid/deferred nine-step progression, material simplifications, guides, separate direct/indirect signals.
- **[N03]** [`Docs/Design.md`](https://github.com/NVIDIA-RTX/RTXPT/blob/303b408ab81790a5051abd7573830b7a4112eb8e/Docs/Design.md) — production pass graph, ray cones, SHaRC, path space decomposition, V-buffer, DLSS-RR/async and explicit cache bias.
- **[N04]** [`README.md`](https://github.com/NVIDIA-RTX/RTXPT/blob/303b408ab81790a5051abd7573830b7a4112eb8e/README.md) — platform/Vulkan requirements, known AMD caveats, Intro vs Advanced Sample distinction.
- **[N05]** [`SubSampling.h`](https://github.com/NVIDIA-RTX/RTXPT/blob/303b408ab81790a5051abd7573830b7a4112eb8e/Rtxpt/Shaders/Misc/SubSampling.h) — 1/p weighted sparse pixel selection, four pattern implementations.
- **[N06]** [`NEEATConfig.h`](https://github.com/NVIDIA-RTX/RTXPT/blob/303b408ab81790a5051abd7573830b7a4112eb8e/Rtxpt/Shaders/Libraries/NEE-AT/NEEATConfig.h) — actual 2,097,152-record limit and recommended practical aggregation under ~500k.
- **[N07]** [`PathTracerSHaRC.hlsli`](https://github.com/NVIDIA-RTX/RTXPT/blob/303b408ab81790a5051abd7573830b7a4112eb8e/Rtxpt/Shaders/PathTracer/PathTracerSHaRC.hlsli) — consume/train separation, path storage, deposit/backpropagation/cache query.
- **[N08]** [`PathTracerHandleHit.hlsli`](https://github.com/NVIDIA-RTX/RTXPT/blob/303b408ab81790a5051abd7573830b7a4112eb8e/Rtxpt/Shaders/PathTracer/PathTracerHandleHit.hlsli), [`PathTracerSample.hlsl`](https://github.com/NVIDIA-RTX/RTXPT/blob/303b408ab81790a5051abd7573830b7a4112eb8e/Rtxpt/Shaders/PathTracerSample.hlsl) — core BSDF scatter and ray query/path continuation.
- **[N09]** [`LICENSE.txt`](https://github.com/NVIDIA-RTX/RTXPT/blob/303b408ab81790a5051abd7573830b7a4112eb8e/LICENSE.txt) — NVIDIA RTX SDK terms; **source-available ≠ OSI open source**.

### External specs / independently usable academic references

- **[R01]** [wgpu official ray-tracing API spec](https://github.com/gfx-rs/wgpu/blob/v30.0.0/docs/api-specs/ray_tracing.md) — experimental ray query/Tlas/Blas, CPU-owned TlasInstance and limitations; **pinned to upstream wgpu v30.0.0**; recheck the actual Cargo.lock and backend feature negotiation before changing the API.
- **[R02]** [NVIDIA technical article: Solving Self-Intersection Artifacts in DXR](https://developer.nvidia.com/blog/solving-self-intersection-artifacts-in-directx-raytracing/) — bounds-based geometric-normal offsets and camera-relative precision; public explanatory article, not a license to transplant RTXPT code.
- **[R03]** [Texture LOD Strategies for Real-Time Ray Tracing (Ray Tracing Gems)](https://research.nvidia.com/publication/2019-03_texture-level-detail-strategies-real-time-ray-tracing) — differential vs cone analysis.
- **[R04]** [Improved Shader and Texture LOD Using Ray Cones (JCGT 2021)](https://jcgt.org/published/0010/01/01/) — paper and separately described public-domain implementation.
- **[R05]** [Sampling the GGX Distribution of Visible Normals (Heitz, JCGT)](https://jcgt.org/published/0007/04/01/) — VNDF importance sampling and PDF model.
- **[R06]** [SVGF: Spatiotemporal Variance-Guided Filtering](https://research.nvidia.com/labs/rtr/publication/schied2017spatiotemporal/) — concepts for independent temporal/variance-guided reconstruction; verify actual code license before incorporating implementation.

---

## 9. Definition of done for the entire program

A successful outcome is **not** parity with RTXPT, a copied NVIDIA shader, a specified FPS uplift or a checkbox saying “SHaRC on”. Completion requires:

- An up-to-date source-/capability-true architecture document and stage-controlled `graphics.toml` settings (every option does something measurable, or is documented unsupported).
- A reproducible hardware+reference A/B suite, preserved raster-off behavior and zero-work invariants, and independent numeric GPU/CPU oracles.
- Evidence-backed improvements in correct ray origins and texture footprints wherever actual image defect justified the change.
- Optional higher-quality indirect/specular pathways with their **own** cost/error model, guide buffers, invalidation and rollback controls.
- No regressions of simulation authority, planetary precision, streamed terrain/sea/cloud materials, project licensing or engine ownership.
- A final report with per-device quality/performance tradeoffs, negative results and explicitly deferred work.

**Program completion does not require WP6–WP7 if G5/G6 do not justify them.** The correct engineering result may be a smaller, demonstrably better hybrid renderer.
