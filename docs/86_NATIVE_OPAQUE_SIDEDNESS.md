# 86 — Native opaque sidedness and immutable demand reuse

Status: continuation of [depth-stream validation](85_NATIVE_DEPTH_VERTEX_STREAMS.md),
2026-10-28. Baseline feature commit `58dffc0`. Physical GPU diagnostics remain
explicitly authorized by the user. The earlier reset incident is retained;
no driver or watchdog settings are changed.

## Investigated topology behaviour

The 30-warmup-frame trace in doc 85 was too short to establish a failure to
converge. A separate fixed-source coastal GPU replay reached zero requests and
commits with publication revision 59 and 53,008 live slots. The original native
60-frame replay also finished with zero requests/commits and no exhaustion.
The topology generation/completion sleep gate, publication invalidation and
geometry policy are retained. Removing that invalidation would expose stale
terrain and was not implemented.

## Opaque sidedness

Previously a mixed mesh asset used one fragment program containing a dynamic
back-face `discard`, including its double-sided foliage. The earlier specialization
removed it only when every material in the entire asset was double-sided.
Single-sided ground details therefore kept the discard-capable program active
for unrelated leaves.

The renderer now uses fixed-function back-face culling for single-sided
materials and no face culling for double-sided materials. Both variants
specialize out the fragment discard and share an explicit pipeline layout.
An asset containing only one kind needs one pipeline; a mixed asset needs two.
The fragment lighting, texture filters, geometry, sample count, depth comparison,
index order, instance order and source admission are unchanged. No extra draw,
prepass, texture or per-frame geometry upload is introduced. Shadows keep their
existing caster rules.

The explicit layout covers the existing frame/material/shadow ABI. The ray plan
owns its group-3 schema, including shadow-only AS bindings, reflected-hit tables,
textures and optional terrain/ocean bindings. RT source participation, AS
admission and ray-sidedness policy are unchanged. The old dynamic-discard path
is available only as the private same-build regression control, not a new user
setting.

Complete colour/depth comparisons cover all-single, all-double and mixed
materials, both windings, mapped normals/colour/AO, 1x/4x MSAA, shadows on/off
and large origins. CPU Vulkan and physical Radeon comparisons require exact
bytes, not a relaxed image tolerance.

## Immutable terrain selection demand

The existing history sampled the full geometry demand again every 0.75 seconds,
even for an identical stationary camera and immutable terrain source. On the
initial native capture, streaming occupied about 68 ms per measured frame.

`SelectionHistory::update` now accepts the shared immutable `Arc<PlanetField>`
and retains one memo entry. Its key includes source identity, complete selection
view (including frustum/material matrix), selection policy, exact-cover mode,
detail bias and the previous fine selection used for hysteresis. Keeping an Arc
pins the source rather than caching a reusable address. Source replacement
invalidates reuse. Empty demand still forces bootstrap.

A matching input reuses the existing demand without advancing its publication
epoch. The existing motion/debounce sampling clock still advances at the same
admitted cadence, so later motion is not averaged over the whole idle interval.
Mailbox polling, failed-job retries, material refinement, source publication,
LRU protection and camera-motion selection remain in the existing runtime.
Only the repeated deterministic selection is omitted. The cache is bounded to
one input and its previous-fine vector; no terrain pages or fields are copied.

A no-memo reference is evaluated across repeated stationary inputs, eye/forward,
frustum, projection, budget, origin/speed, source and exact-cover changes.
Reused demand must equal that freshly computed reference. Native and legacy
hosts already own Arc sources and use the same API.

## Measurement boundary

The initial physical 1280x720, 4x-MSAA, High/HDR captures use RT off, frozen
simulation time, the fixed coastal survey, 30 warmup frames and 30 measured
frames. They use GPU timestamp intervals and ordinary frame submissions, not
the serial `GpuTrace` fences. Initial medians were approximately 790 ms wall,
554 ms opaque raster, 89 ms surface/shadows and 61 ms overlays.

The sidedness-only exploratory run measured approximately 550 ms wall and
349 ms opaque raster. Live streaming can produce different material residency
at different wall cadences; those observations alone do not certify a matched
whole-frame speedup. Exact sidedness regression evidence is separate from these
exploratory native captures. Final checks, source pins and accepted measurements
are recorded with the [machine-readable artifact](artifacts/native-opaque-sidedness-2026-10-28.json).

Both final long captures render 120 total frames (30 warmup + 90 recorded).
For the last 30, the original build measured approximately 684.58 ms wall,
584.27 ms opaque raster and 34.83 ms streaming median; the final build measured
447.65, 382.25 and 2.31 ms respectively. Both ended with zero topology
requests/commits and the final build encoded zero cached shadow passes. Material
residency histograms differ, so these remain scoped native observations rather
than an isolated causal whole-frame percentage.

Final verification passed 18 build/architecture checks and 557 CPU tests,
nine isolated software scenarios and twelve physical Radeon scenarios. The
latter include six small actual ray-query cases covering shadow-only AS,
reflected texture/normal/AO, terrain membership, empty TLAS, shared directional
environment and ocean deformation. No new GPU fault/reset was detected.
Complete coastal depth images and the four authored-plant depth/colour images
also match the preceding depth-stream checkpoint byte for byte.

Non-document source-manifest SHA-256:
`587c9d5e7bcad54175ae596c9e3f6f5eaeb9c6c86ea0ed02ec475455a045a8cd`.
Release product SHA-256:
`0bd867dd5a780e8ebd4d1ae2b687ad9a89060cf662b6344404c7bddd0900da5a`.

This does not establish game FPS, complete RT stability or blind reference
parity. Moving views still incur legitimate refinement and shadow work; primary
material/cloud shading remains a measured bottleneck.
