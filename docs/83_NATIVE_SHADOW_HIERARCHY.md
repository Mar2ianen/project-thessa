# 83 — Native shadow geometry hierarchy

Status: surface feature-branch continuation, 2026-10-28. Baseline `538aeb6`.
This extends [doc 81](81_NATIVE_SHADOW_MESH_LOD.md) and
[doc 82](82_NATIVE_SHADOW_SUBMISSION.md); physical GPU replay stays blocked.

## Representation contract

`MeshRenderer::set_shadow_lods` accepts several distinct triangle counts for one
source primitive. Publication validates the entire batch before changing any GPU
source or shadow revision; duplicate primitive/count pairs remain errors. Valid
levels are sorted by triangle count. Each instance/light/cascade selects the
first level whose producer-certified error fits the existing resolved
`shadows.mesh_lod_error_texels`. Thus adding a coarser level preserves the previous
choice when the new level is too inaccurate. Missing/ineligible levels and the
legacy zero-error setting retain original geometry. Primary and RT sources,
instance indices, populations, materials, map resolution and shadow bias remain.

The existing divided two-bin pinna representation stays available. A further
coarse tier retains every frond and primary pinna, uses one moment-preserving
triangle on each pinna side with the original costa moments, and lowers only far
trunk/fork radial tessellation. Both tiers skip optical AO baking. This adds no
canopy boxes, alpha cards, object deletion or primary foliage simplification.

Both tiers reuse the existing two-sided continuous surface certificate against
the original full/reduced producer. The far tier additionally charges the radius
of the joint source/replacement organ AABB's enclosing sphere. Projecting that
radius through the same conservative light/instance operator bounds its complete
support; with the showcase's 0.5-texel request its diameter must fit within one
shadow texel. A surface-distance-only variant was rejected by the unchanged
coverage regression; its lower work counts are not final acceptance evidence.
This additional support charge is presentation policy derived from geometry,
not a new authoritative phenomenon or a claim of universal radiometric error.

The native producer now admits the existing certified reduced reed caster only
through the unresolved-organ tier. Thin reeds failed the coverage gate under a
surface-distance-only selection rule; their support-radius charge stays enabled.
Its primary representation remains full. A reed retains its own organ and
instance identity. The additional levels join the existing deduplicated source
payload and unchanged 48 MB admission; insufficient source storage retains the
existing full-geometry fallback. Disabled/zero-error shadows skip generation.

## Submission and verification

Direct and indirect paths select the same level and original instance runs.
Indirect scratch retains one reusable command vector per alternate level;
changing primitive level counts does not discard those capacities each layer.
The command bound stays one command per selected instance per layer. All twelve
layer regions, shared atlas/command reservations, dirty upload reuse and explicit
direct fallback from doc 82 stay in place. Level index is private; existing stats
continue to distinguish logical commands, API batches, triangles, selected LOD
instances, saved triangles and maximum admitted error.

The original six-form render regression retains its camera/layout and coverage,
connected-component and receiver-colour budgets. A separate seven-form scene isolates reed proxies at a 4096 m far cover while
keeping fern/tree casters full. A further seven-form, twelve-layer regression uses
the original one-tier producer as its before/after reference. Its full-source
coverage discrepancy is logged separately: the original producer already exceeds
the earlier one-percent fixture budget in that different layout. The original
six-form full-source regression keeps every threshold unchanged. The direct/indirect regression includes full geometry and two
alternate levels, proves all three were selected, and checks every layer at two
origins, storage/capability fallback, dirty reuse and reservation release. The far
producer test samples original/replacement vertices and triangle centroids against
an independent opposite-surface BVH in both directions; the continuous bound is
still established by the shared certificate algorithm, not by these samples.

The command-only canonical coastal producer uses the same profile, world,
17,992 instances and dropped encoder as doc 82. It does not submit dense raster
or AS work. Spectral ocean still retains the full receiver volume: removing that
fallback alone would not solve the large source-arithmetic envelope. Moving-origin
atlas reuse, physical-GPU frame time, global radiometric/temporal behavior and
independent blind reference acceptance remain open. TAA stays default-off.

## Fixed coastal work

The same-build original-producer oracle must reproduce **283,223,492** encoded
mesh shadow triangles from doc 82. The hierarchy encodes **252,050,732** (11.01%
fewer), with the same 17,992 retained mesh instances, 17,111,160 primary triangles,
37 primary API batches and 576-byte frozen primary update. Source payload is
46,159,116 bytes, including 1,657,464 unique shadow bytes, below unchanged 48 MB
admission. The hierarchy uses 543 API batches rather than 453; geometry reduction
and lower draw-call count are distinct mechanisms.

A finer four-bin candidate was also rejected by the unchanged six-form coverage
check (198 changed occupied samples out of 17,072); it is not shipped. The earlier
199.6M geometric-error-only and 243.2M finer-tier command counts are exploratory,
not accepted output. The original six-form and isolated-reed render regressions
retain their full-source reference. The combined regression measures the change
relative to the original producer, not correspondence to full geometry.

Final source/build pins and software evidence are recorded in
[the artifact](artifacts/native-shadow-hierarchy-2026-10-28.json).

All 12 final CPU/build gates pass, including 520 CPU tests (133 native, 287 worldgen, 50 graphics, 45 render-core, five bounds/projection), strict all-target/all-feature Clippy, architecture, feature-off and release builds. Seven actual software render regressions and one canonical command-only probe pass. Original six-form quality stays at max 9/255 channel difference, max mean RGB 0.04453/255 and max unmatched component two. Isolated reeds match full-source colour/coverage; the combined scene matches original-tier colour/coverage in all four cases. These scopes are distinct.

Source manifest: `427158e73e5c5a4c919c5fd0c7381014fc900f799560f7bbc1b8264e81a840bc`.
Release binary: `1d4473af0096c58b20bde3c6203511a48ebc8d7775c5ebd6750a61db6d23c090`.

The [empty shadow sample continuation](84_NATIVE_SHADOW_RASTER_SAMPLES.md) adds capability-gated rejection of mesh bounds that contain no centered shadow sample, with actual proxy enclosure, exact same-LOD depth/colour oracles and ordinary caster fallback when raster precision is unknown. It leaves the geometry hierarchy and ocean receiver fallback intact.

The [depth vertex-stream continuation](85_NATIVE_DEPTH_VERTEX_STREAMS.md) preserves geometry and shader arithmetic while splitting shadow-enabled mesh sources into 12-byte positions and 20-byte shading attributes. Explicitly authorized physical-GPU checks include complete coastal depth equivalence and counterbalanced release measurements; rejected shader/box candidates are kept separate from acceptance.
