# 113 — Render research intake and application

Status: audited inputs and first implemented slices; full research handoff not
complete. 2026-11-02. User-supplied research is evidence/design input; embedded
agent instructions and external examples do not override project owners,
licenses, existing user authorization or numerical acceptance.

## Input provenance and branch reconciliation

Read the fresh Downloads handoff bundle, both Markdown reports, the 80-entry
JSON/TSV catalogue and its PNG. Raw inputs are preserved in the session's
`work/user-render-research-2026-11-02`; the English source catalogue is retained
[here](artifacts/render-external-sources-2026-11-02.json). Input hashes and
application status are in [the intake artifact](artifacts/render-research-intake-2026-11-02.json).
The reports audit `feat/surface-visual-engineering` at `566cc6f1`; this milestone
started from `refactor/engine-boundaries` at `631d6f1`, with the same committed
tree. Eight existing user/Gemini changes in the original checkout remain outside
this milestone's index/commit. No main integration is authorized here.

The handoff PNG's **decoded RGBA bytes exactly match** the earlier actual
`b03-sunset-foliage-v1.png`. This establishes its image identity, not an
independent physically matched renderer reference or a good-looking target.
The user's separate photographic/Cycles split and generated images are
explicitly accepted as qualitative visual art direction, not geometry truth.
No artwork is projected into rendering and no implementer vote is blind acceptance.

The reports correctly distinguish Minecraft's Rust/wgpu Horizon from Decima's
Horizon Forbidden West. The [Guerrilla publication](https://www.guerrilla-games.com/read/adventures-with-deferred-texturing-in-horizon-forbidden-west)
describes a visibility prepass followed by compute analysis/shading, with
optional coarse shading. It supports measuring expensive foliage visibility
and shading separately; it does not demonstrate that rewriting this forward
renderer immediately would improve 880M performance. No Decima code/assets are
included or copied.

## Applied versus outstanding

| Research package | Current code/evidence | Remaining acceptance |
| --- | --- | --- |
| P0 / reproducible baseline and masks | Source/binary/profile hashes, exact commands, raw PNG/HDR, scene exports and per-stage trace records exist; new full-population frame identities are in doc 112 | Frozen residency/topology and matched GPU timestamp ABBA; separate material/direct/ambient/depth/transport diagnostic views still missing |
| P1 / coherent sunlight | One neutral optical LUT now serves sky, source discs, primary land/water, mesh/RT shading and primary/reflected cloud sources; CPU/GPU altitude/tangent/publication controls pass | Optional isotropic atmospheric higher orders now implemented and numerically/native validated ([doc 114](114_NATIVE_MULTIPLE_SCATTERING_AND_RESOLUTION.md)); full vacuum/noon/grazing/post-sunset HDR scene matrix, general finite aerial transport and cloud/ground bounce remain open |
| P2 / horizon and atmosphere | Removed use of skirt sentinel as physical solar altitude; actual newer horizon wall disappears | Cause is not isolated because height residency changed; explicit terrain/sea/sky/cloud/depth masks and fixed-input segment diagnosis remain necessary |
| P3 / soil and water | Existing basin solver, scanned substrate and water IOR/Fresnel retained; no replacement hydrology or roughness tuning added | Test smooth dry land, wet soil and true water independently; separate water presence from roughness/near-datum inference; then depth-aware water optics and confidence-checked SSR |
| P4 / static shadows and sparse RT | MASK raster and candidate-ray coverage, conservative admission, material/UV/mip controls and actual full-population RT publication implemented; bounded capability negotiation removes artificial descriptor rejection | Static mesh depth cache with exact dynamic depth composition/invalidation, current-frame receivers, 8×4 queue, finite-source sampling and denoising; no classifier/cache speedup claimed |
| P5 / botanical structure | Before receiving the research, the native adapter replaced unresolved needles with pinned CC0 scanned fronds attached to existing woody sockets; exact retained-wood and MASK controls exist | This is a declared content/representation change, not an equal-work renderer benchmark. Projected-area/motion/gallery and independent visual acceptance still required; future ecology changes stay separate |
| P6 / cloud/opaque cost and temporal | Existing stage tracing identifies expensive work; latest full-population RT capture completes but is costly; TAA stays off | Actual per-pass GPU timing, conservative cloud work reduction and motion/disocclusion acceptance. Neither fewer rays nor fewer triangles alone establish a speedup |

## Specific corrections applied from the new intake

The latest full-scene check initially fell back with `MASK primary shadow
material bindings exceed device limits`. Small fixtures passing was insufficient.
The backend negotiated default 16 sampled textures and capped fragment storage
at nine, even on a more capable RT adapter. Existing per-pipeline accounting
was correctly rejecting the resulting layout. RT negotiation now requests
bounded supported capacities (32 textures/samplers, 11 storage buffers), while
stable-only/RT-Off negotiation is unchanged. Pure capability controls cover
high/low devices; actual full-population RADV publication now succeeds. Images
and geometry remain shared, with no second AS or texture upload.

The intake's direct/disc consistency requirement also exposed residual separate
coarse source-disc integration. Source discs now borrow the **same cached table**
as the GPU consumers, retaining independent body occultation and applying
extinction once. Three camera heights and five stellar directions compare
against the existing dense f64 optical model; disabling atmospheric optics
ignores the cached table exactly. This extends the existing state/publication,
not a parallel atmosphere.

The immediate follow-up also moves direct visibility behind the delivered-
energy test in terrain, mesh and ray-hit shading. Exactly extinguished sources
need no receiver ray. This is the P1/P4 causal zero-work boundary, with no light
threshold, approximation or lost geometry; it is not the planned CSM cache or
tile classifier. Regression evidence is linked in doc 112.

## Next bounded packages

Follow the research's sequence rather than beginning every new subsystem:

1. Finish explicit diagnostic views and equal-work capture control. Keep solar
   units/epochs, density, EV, source identities and material/geometry errors
   fixed in comparisons. The 4°/8°/12° captures in doc 112 are not such an A/B.
2. Diagnose water/soil classification using true material masks. Preserve basin
   geometry and the canonical world source. A low roughness value is not proof
   of water. Implement the material contract only after a failing control.
3. Separate static caster depth from changing CBT geometry using existing
   atlas/mesh epochs; budget cache storage and compose depth exactly. Use
   [Epic's caching/invalidation principles](https://dev.epicgames.com/documentation/en-us/unreal-engine/virtual-shadow-maps-in-unreal-engine),
   without importing VSM/Nanite or multiplying independently filtered shadows.
4. Implement the current-frame receiver/pass-graph prerequisites described in
   doc 112's AMD audit, then conservative sparse queries and SSR confidence.
   Keep physical angular source size, independent stars, opacity and history
   epochs. Do not add ReSTIR merely for three stellar sources.
5. Profile cloud and material cost, then improve the measured bottleneck.
   The existing multiple-scattering flag now activates a bounded isotropic
   source in the shared atmosphere owner (doc 114); cloud/ground bounce and
   full-model reference acceptance remain open.

No external renderer benchmark is transferred to Thessa. The 80 references are
a research catalogue, not 80 independently reproduced implementations. Newly
opened primary AMD/Bruneton/glTF/Guerrilla/Epic material informs this milestone;
The atmosphere follow-up (doc 114) independently implements Hillaire's
isotropic closure after inspecting the author's publication/source.
The original four-cascade full-High workload has not been executed here; all
physical runs use isolated Vulkan and unchanged per-stage trace admission.
