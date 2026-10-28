# 81 — Native shadow mesh representation

Status: implemented and CPU/software verified on the surface branch, 2026-10-27.
Target-GPU/full-frame and reference acceptance remain open.
This continues the [Horizon work audit](80_HORIZON_RENDER_WORK_REVIEW.md).
The measured baseline is the clean surface-branch tree at `685d093`; separate
user/Gemini palette, droop and soil edits are preserved outside this change.

## Contract

`shadows.mesh_lod_error_texels` controls optional depth-only mesh geometry through
requested/resolved graphics settings, metadata and the native consumer. The
legacy default is **0**, which selects full geometry. The surface showcase opts
into **0.5 texel**. Suggested accuracy levels are 0.25 (strict), 0.5 (balanced)
and 1 (cheaper); the resolver clamps finite requests to 0..=2 and resolves
non-finite values or disabled raster shadows to zero. This is a geometric
accuracy budget, not a general bound on receiver radiance or temporal flicker.

The backend-neutral `ShadowMeshLod` supplies positions, indices, original
primitive identity and a producer-certified two-sided continuous surface-distance
bound in local metres. Primary and ray-query sources retain their original
geometry. Original instance indices and all plant identities survive.

The shared botanical producer retains every frond and primary pinna. A depth-only
pinna has two attached triangular divisions per side; their measured areas and
moments come from the existing full leaf generator. There are no canopy boxes,
alpha cards or population thinning. Shadow geometry skips the optical AO bake.
Full and existing coarse primary representations keep their previous generators.

Certificates compare each material organ in both surface directions using the
existing Parry triangle BVH. Distance at a triangle centroid plus its enclosing
radius is an upper bound by the 1-Lipschitz property of distance to a closed set.
Longest-edge subdivision tightens that bound; depth/work limits retain the
current upper bound rather than publishing a sampled estimate. Identical
triangles need no queries. Empty or invalid data supplies no alternate LOD.
Certificates and generated forms are cached for the immutable producer.

For each light/cascade/instance the renderer projects that bound through the
actual uploaded f32 rigid matrix and orthographic light matrix. A conservative
2D linear-map norm and a separate f32 arithmetic envelope must fit the resolved
texel budget. Missing geometry, invalid/non-affine matrices or excessive error
select full geometry. Culling encloses the replacement surface using the source
sphere expanded by the certified error. Consecutive equal visibility/LOD choices
retain instanced runs. The unculled full-caster oracle also disables LOD.

LOD installation validates the complete replacement before publication; failure
keeps the previous geometry and shadow revision. Success invalidates the shadow
cache. The disabled native path does not generate proxies or allocate their
buffers/pipeline. Depth vertices contain only positions (12 bytes). Identical
full/coarse caster geometry shares GPU buffers; actual unique bytes join the
existing **48 MB** aggregate prototype admission limit. If optional caster buffers
would exceed the remaining budget, they are withheld and full geometry survives;
`native_surface_plants_shadow_lod_budget_withheld` reports that fallback. Small
coarse juvenile forms already at minimum tessellation also keep their existing
geometry. The payload counter covers renderer geometry buffers, not all CPU
caches, textures, acceleration structures or driver memory.

## Work counts

The fixed coastal probe uses 23.565679258039204° / 165.13881581863882°, an 80 m
survey, 2 m eye height, 1280×720, epoch zero, the checked-in showcase profile and
RT off. It initializes the actual terrain/environment/native mesh consumer and
encodes real shadow commands into a **dropped encoder**. It submits no dense
raster or acceleration-structure pass. Physical Radeon replay stays blocked.

| Cold mesh work | Baseline | Depth-only LOD |
| --- | ---: | ---: |
| Retained instances | 17,992 | 17,992 |
| Primary triangles | 17,111,160 | 17,111,160 |
| Primary API batches | 37 | 37 |
| Active shadow layers | 12 | 12 |
| Shadow candidate triangles | 1,353,766,008 | 1,353,766,008 |
| Shadow encoded triangles | 999,585,598 | 283,199,642 |
| Shadow API calls | 5,518 | 6,918 |
| Additional unique caster bytes | 0 | 991,680 |

This first probe reduces encoded shadow triangles by 71.67%. LOD switches create
more runs; reduced geometry is the mechanism, not a draw-call reduction. The
maximum admitted error is 0.499859 texel. Expanded conservative bounds retain a
few additional boundary casters, so the replacement saving counter is not exactly
the difference between the two independently culled totals.

The final debug cold encoder measured 445.07 ms; the earlier baseline probe
recorded 316.07 ms. These individual runs are not a calibrated CPU latency
comparison and do not establish a speedup. Frozen update p50/p95 are
0.256/0.361 ms; exact shadow-cache hits still encode zero passes. No GPU time or FPS is inferred from
triangle counts. Even the reduced cold plan remains large, so complete frame-time,
physical-GPU stability, moving-camera and blind reference acceptance remain open.

## Evidence and remaining acceptance

All 12 final CPU/build checks pass, including 514 CPU cases (133 native,
286 worldgen, 48 graphics, 43 render-core and four bounds/projection), strict
all-target/all-feature Clippy, architecture and feature-off checks, and a release
build. A preliminary parallel native suite hit the existing embedded-server
snapshot deadline; the final complete single-thread suite passes without changing
the timeout or server code.

Four isolated software cases pass: exact primary colour/depth versus original
full ranges, exact zero-error full/culled shadows with a supplied approximate
caster, actual full/coarse six-form plant shadows plus a ground receiver, and
the canonical command-only coastal probe. The ground receiver is drawn separately
from the plant-only depth oracle so it cannot hide missing foliage coverage.
Original source/primary output remains exact after proxy installation. Invalid
replacement batches retain the previous geometry; source changes invalidate the
shadow cache and the next unchanged frame retains the zero-pass cache hit.

Across four real-plant cases the maximum admitted error is 0.473 texel. There
are 107–148 changed coverage samples; 5–14 have no opposite-map sample within one
texel. Their largest connected component has two samples. The measured final
colour, including the ground receiver, differs by at most **9/255** on a channel
and at most **0.04453/255 mean absolute RGB**. The fixture regression budgets are
<=2 connected unmatched samples, <=0.1% unmatched and <=1% changed occupied
coverage, <=16/255 maximum channel and <=0.1/255 mean RGB. These are scoped
regression budgets, not global receiver/radiometric acceptance. The raw
full/LOD depth and colour, difference panels and commands are retained in the
local evidence bundle.

The final canonical source payload is **45,493,332 bytes**, including 991,680
unique caster bytes, within unchanged 48 MB admission. The primary/RT geometry,
canonical populations, terrain CBT policy and TAA default remain unchanged.
A source-byte limit still does not bound expanded multi-pass work.

Hashes, per-case metrics, commands, source/driver pins and explicit limitations
are recorded in [the evidence artifact](artifacts/native-shadow-mesh-lod-2026-10-27.json).
Source manifest: `3d71fecfb23688355b253d55e172c2d9ce6473e32cc4c4b2242439984b1706c3`.
Release binary: `1cd5e8499e550a2a8ec3b220ba1662eeabd1f6e8ab645a4ff56695982d847737`.
Physical-GPU, whole-frame, moving-camera and blind reference acceptance remain open.

The [2026-10-28 shadow submission continuation](82_NATIVE_SHADOW_SUBMISSION.md) adds opt-in primary-receiver selection with full-volume fallbacks, shared-budget indexed indirect shadow batches, dirty command uploads and cached uploaded-world bounds. Its current evidence and limitations are separate from the dated measurements above.

The [shadow geometry hierarchy continuation](83_NATIVE_SHADOW_HIERARCHY.md) keeps the existing depth tier, adds an unresolved-organ tier and a sub-texel reed caster, and records the same-build original-producer comparison separately from full-geometry quality evidence.

The [empty shadow sample continuation](84_NATIVE_SHADOW_RASTER_SAMPLES.md) adds capability-gated rejection of mesh bounds that contain no centered shadow sample, with actual proxy enclosure, exact same-LOD depth/colour oracles and ordinary caster fallback when raster precision is unknown. It leaves the geometry hierarchy and ocean receiver fallback intact.

The [depth vertex-stream continuation](85_NATIVE_DEPTH_VERTEX_STREAMS.md) preserves geometry and shader arithmetic while splitting shadow-enabled mesh sources into 12-byte positions and 20-byte shading attributes. Explicitly authorized physical-GPU checks include complete coastal depth equivalence and counterbalanced release measurements; rejected shader/box candidates are kept separate from acceptance.
