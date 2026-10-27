# 58 — Native pipeline optimization review

**Status (2026-10-18): implemented reviewed native pipeline slice, based on `1e86e72`.**
The imported working-tree changes removed per-tile temporary serialization and
introduced a shared terrain/opaque-mesh/sky pass. Their original performance and
parity claims were not treated as trusted evidence. No old measurements are
retroactively relabeled as results of the reviewed implementation.

## Findings in the supplied evidence

- The local JSON files contain **60 measured frames**, 30 warmup frames and
  **1 s pre-stream**, with asynchronous work still pending. Build commit is
  `unknown`. Endpoint tile counts alone do not prove identical per-frame work,
  asset publication, GPU clocks or competing host load.
- Their scenarios explicitly say `vehicle_preview=false live_flight=false
  server_flight=false hud=false`; the main path has no mesh renderer. They cannot
  demonstrate savings from merging terrain and mesh passes, because that second
  mesh pass was absent. A CPU serialization improvement or timing difference in
  these captures is not evidence of a unified-mesh-pass benefit.
- Static `baseline-frame.png` vs `unified-static-frame.png`: **mean absolute
  channel difference 0.000597602**, **max 82/255**, **151 changed pixels** of
  921,600 (403 changed channels). Sweep: **mean 0.188044162**, **max 37/255**,
  **79,214 changed pixels** (213,265 channels). Values include RGBA channels.
  Small average error is not byte identity or "100% visual correspondence".
  These files do not establish that Early-Z caused their differences; asynchronous
  publication was not isolated. A blind visual preference verdict was not run.
- `fixed_source_shadow_frame_cost` still invokes `terrain.encode` followed by
  `mesh.encode`. Its exact-image assertions compare cached and forced **shadow
  depth**, not split vs unified rendering. Its pass is not a unified-path test.
- `frame_bytes` still allocates an output vector; the supplied shadow patch
  creates matrix and frame vectors. Removing inner per-tile `Vec`s is worthwhile,
  but neither **zero-copy GPU upload** nor **zero allocations in the entire hot
  path** follows. Queue/driver/UI/streaming allocations were not instrumented.
- Adjacent marks 2/3 after the combined pass made the old `mesh_and_rt` scope an
  empty interval while `terrain_and_sky` silently absorbed AS/mesh work. That
  changed the meaning of named telemetry and required a fix before acceptance.

The reported reciprocal-median FPS and speedup percentages remain **unverified
whole-application claims**, not accepted product-performance results.

## Reviewed implementation

- `MeshRenderer::encode_rt` performs encoder-level AS work before the pass;
  empty draw sets skip it just as the legacy complete mesh encoder does.
  `draw_into_pass` reuses the same mesh pipeline/materials/ranges. The compatible
  format/single-sample `Depth32Float` and AS-preparation requirements are explicit.
- `TerrainRenderer::encode_scene` retains classification and draws terrain,
  opaque meshes, then depth-rejected sky in one pass. The old `encode` and
  complete mesh encoder remain available as the deterministic reference path;
  there is no graphics toggle, scene copy, alternate renderer or new backend.
  Both window and headless production callers prepare AS before this pass.
- Profiling now brackets **shadows /AS builds /classification+unified raster
  /UI** with five meaningful marks. The new names are `render.shadows`,
  `render.rt_as_build`, `render.scene_classify_and_raster`, and `render.ui`.
  Metadata identifies `gpu_scope_layout=unified-scene-v2`. Old captures remain
  readable, but their differently scoped names are not comparable stage timings.
- Nearby CPU simplifications preserve the existing shader ABI: shadow matrices
  use a fixed 12-entry stack array, and an 800-byte stack uniform block is encoded
  **only when changed**. Layer writes borrow that block's exact matrix slices.
  An explicit initial `None` preserves first-publication behavior even with no
  active suns. This is not a claim that wgpu's queue upload allocates nothing.
- Terrain uses one capacity-bounded reusable CPU byte buffer, initialized at
  renderer construction. Publications and camera-origin changes refill it only
  after validation. Projection, attitude and exposure changes still update the
  view uniform, but do not re-encode or upload unchanged tile-relative anchors.
  Partition publication already uploads new anchors at the current camera.
  Invalid staging never publishes partial frame bytes. The additional retained
  CPU budget is **112 bytes × admitted terrain slot capacity**.
  Each record is packed as one fixed 112-byte stack block before one append;
  the supplied four-byte-at-a-time append loop was slower than the old inner-Vec
  implementation in the first isolated diagnostic and was not retained.

## Verification and measurement boundaries

`tests/unified_scene.rs` compares the actual split and unified APIs on the same
resident scene, with sky, shadow cache/forced redraw, visible/empty opaque meshes,
changing transforms, resize, RT off/on and spectral ocean off/on. The fixture is
synthetic and deterministic, not a canonical beauty or physical-fleet replay.

Its release cost diagnostic runs **A/B/A/B then B/A/B/A** on the **same 1,024 resident
patches /64 synthetic mesh instances**, 1280×720, **600 samples +30 warmup per
block**, for static and identical moving-camera paths. The reversed second
sequence exposes ordering/clock drift. It reports hardware GPU
command p50/p95/p99 and CPU-update-through-GPU-completion wall time. Readback,
streaming, pipeline compilation and source publication are outside samples. RT is
off for this cost comparison. It isolates pass composition on the reviewed tree,
not a full original-vs-new executable benchmark or a game-FPS gain.

The release CPU diagnostic rotates legacy inner-Vec /supplied scalar fresh-Vec
/packed fresh-Vec /packed reused-staging encoders over the same 1,024 anchors,
retaining 200 samples after 20
warmup. An exact-byte oracle covers large origins, capacity reuse and recovery
after invalid staging. This measures CPU encoding only, excluding uploads and
the unchanged-origin skip. Allocation counts are not measured by this diagnostic.

Actual results and commands are recorded in
[`artifacts/native-pipeline-review-2026-10-18.json`](artifacts/native-pipeline-review-2026-10-18.json).
The final 9,600-frame counterbalanced diagnostic measured split/unified GPU
**4.134 /4.111 ms** static and **20.014 /19.978 ms** moving, using the median of
each mode's four block medians. The ~0.55% /0.18% central differences are smaller
than some block/tail drift; they do not establish a stable application speedup.
Final CPU encoding medians were **28.072 /55.303 /23.244 /23.174 μs** for legacy
inner-Vec /supplied scalar /packed fresh /packed staged. Packing improves this
isolated encoder; staging reuse alone does not explain a large frame-time win.
Exact-byte encoding and split/unified images passed, alongside **1,867 workspace
lib/bin +90 integration tests**, workspace/no-default clippy, **24 Bevy-CBT GPU**,
**13 native ignored**, the origin-cache and new-scope perf-capture hardware tests.
No remote-green-CI, blind aesthetic, cross-platform runtime or complete native
renderer/client replacement claim is made. The independent native flight-deck
work is described in [doc 57](57_NATIVE_FLIGHT_DECK.md).

The subsequent [Horizon renderer review](80_HORIZON_RENDER_WORK_REVIEW.md) extends the common mesh pass with conservative primary visibility, capability-checked indexed indirect batches and dirty-only instance/command publication. `MeshDrawStats` distinguishes candidate instances/triangles, actual primary instances/triangles, logical commands, API batches and latest actual update bytes. Shadow/RT source ownership and original indices are unchanged. Exact software colour/depth comparisons cover the original full-range pass and direct fallback, including large origins, jitter and invalidation; this does not establish target-hardware frame time.

The next [shadow representation slice](81_NATIVE_SHADOW_MESH_LOD.md) selects certified depth-only plant geometry by actual light texels. It preserves the primary/RT source and changes geometry cost rather than reducing API calls. The canonical cold shadow count falls from 999.6M to 283.2M with the 0.5-texel showcase setting; no full-frame GPU timing or reference acceptance follows.
