# 112 — Native filtered foliage, MASK rays and shared solar transport

Status: implemented representation/transport corrections, full coastal visual
and performance parity incomplete; 2026-11-02. This supersedes the MASK/direct
sunlight limitations in the dated docs 94/110/111 without rewriting their
historical evidence. TAA and primary RT shadow correction remain default-off.
The later [doc 114](114_NATIVE_MULTIPLE_SCATTERING_AND_RESOLUTION.md) supersedes
this checkpoint's single-scattering-only limitation and records that the
artificial horizon band still occurs with changed residency.

## Actual plants, rather than unresolved opaque needles

The native survey adapter now reuses the pinned B02 `fern_02` core-cutout GLB
from Poly Haven (CC0). The asset, license and conversion record are in
`assets/vegetation/`. Three scanned forms provide ground rosettes and large
crowns; crown roots attach to the **existing worldgen woody sockets**. The
analytic warp transports normals through its inverse transpose. UVs and the
source alpha silhouette remain intact. Ground roots retain the shared soil
pose; instance IDs, world-addressed colony occupancy and admission limits do
not change. These are authored botanical derivatives, not calibrated fossil
species or a procedural growth simulation.

Worldgen exposes full/reduced **woody scaffolds** so native preparation no
longer builds and AO-bakes unused dense procedural laminae. The original
procedural prototypes remain available for their existing consumers/oracles.
A regression compares retained wood positions, normals and indices exactly.
Ground/reed lamina geometry no longer switches at the distance threshold; only
woody tessellation does. Imported alpha mips carry subpixel detail. The scanned
source has no baked AO channel; the mesh compactor preserves that valid case.

The material uses the source base/roughness/normal maps, slight form pigments
and authored diffuse transmission. The original Blender translucency/mix/hue
graph is **not** reproduced. No photograph or generated reference artwork is
projected into the scene. Existing experimental source/material budgets still
apply; increasing scene counts is not the quality strategy.

## Consistent raster and ray coverage

MASK meshes now cast texture-aware depth into the existing CSM atlas, both
in direct and indirect caster paths. Fragment rejection uses the actual UV,
base-alpha factor and material cutoff. Legacy position-only shadow LODs cannot
represent their holes, so such LOD publication is rejected for MASK sources.
Opaque casters retain their existing path; BLEND shadows remain unsupported.

With resolved MSAA > 1 and motion/history disabled, ordinary 0.5-cutoff,
unit-alpha foliage uses alpha-to-coverage. Single-sample/history rendering and
nonstandard factor/cutoff retain hard MASK semantics. This uses the existing
MSAA policy, not a hidden effect flag. It does not implement temporal AA.

The native RayScene now admits MASK geometry. Its BLAS is nonopaque; a query
iterates candidates, interpolates UVs, samples shared raster textures, applies
factor/cutoff, and confirms accepted intersections. Rejected foliage continues
to farther geometry. Medium/High reflection rays transport their existing
pixel cone into the candidate UV Jacobian, using shared mips. Low and current
stellar shadow queries use point-footprint visibility. Finite-source shadow
sampling and filtered temporal shadow coverage are still open.

The same scene/tables feed optional primary land/water correction. A lazy
compatible primary pipeline borrows the scene schema; its cache identity
includes the parameter-binding layout. No second BLAS/TLAS, geometry copy or
texture upload is introduced for primary receivers. MASK shadow-only scenes
budget hit attributes/tables/pairings; ordinary opaque shadow-only scenes keep
their existing 16-byte vertices and table-free path. Storage, texture, instance
and memory limits retain whole-partition fallback. RT device negotiation now
requests at most 32 supported texture/sampler slots and 11 fragment storage
buffers: the previous default 16 texture slots rejected the full native scene
although the adapter could support it. Stable/RT-Off limits remain unchanged;
this requests capacity without allocating another image or AS. RT Off allocates
no scene.
**BLEND ray transmission remains unsupported**, with an explicit reason; MASK
acceptance does not establish glass/refraction or transparent-emitter GI.

A separate lighting bug flipped the geometric normal before tangent mapping,
leaving backface tangent components wrong. Raster and RT now map in the authored
front frame and reverse the complete shading normal before lighting, following
[glTF double-sided lighting](https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html#double-sided).

## One atmospheric extinction model

`thessa-atmosphere::TransmittanceLut` tabulates the existing f64 optical
integrator, using radial/chord-distance coordinates concentrated at the horizon
([Bruneton reference](https://ebruneton.github.io/precomputed_atmospheric_scattering/atmosphere/functions.glsl.html)).
The exact ground tangent no longer incorrectly produces an empty segment and
unit transmission. Body occlusion, ground/space cases and emission-independent
cache matching are explicit. No new simulated atmosphere or fitted light color
is introduced.

Native quality selects 128×32/64, 192×48/96 or 256×64/128 table/integration
budgets. The table is cached with the existing optical state. Sky evaluation
uses the **same view integral**, replacing repeated coarse solar marches with
table samples. At this checkpoint it remained single scattering. The later
[doc 114](114_NATIVE_MULTIPLE_SCATTERING_AND_RESOLUTION.md) implements the
existing multiple-scattering flag through an isotropic source; cloud/ground
bounce remains a tracked gap.

The shared environment storage appends the table, with a 256-byte common
metadata ABI. Camera altitude is subtracted in f64 before upload. Four reads
per contributing source replace per-fragment optical integration. Primary
land/water, meshes, RT hits, source discs and primary/reflected cloud sources consume that
publication. Growth/release rebinds existing consumers; disabled transport
returns the source exactly. A newly uploaded table stays disabled until its
camera/domain metadata is valid. Common storage accounting includes the table;
the cloud volume's 272-byte solar placeholder shares its existing cloud
resources and does not duplicate the table.

Terrain skirts carry a **sentinel** `height_m`, not a physical altitude. Solar
transport now derives their actual position/altitude; treating that sentinel as
space would incorrectly bypass local extinction. The wall disappears in the
new full capture, but residency also changed (314 versus 739 regenerated
pages), so this does **not** isolate its visual cause in conforming geometry.

`--scene b03-sunset` chooses the first descending four-degree primary source
from baked ephemerides and the existing spin. It changes presentation time,
not simulation ownership; explicit `--sim-time` overrides it. B03 itself keeps
its existing defaults. Verified epochs at the canonical B03 site are:

| Elevation | Presentation time, seconds |
| --- | ---: |
| 4° | 81639.80870498807 |
| 8° | 78107.3354013362 |
| 12° | 74566.63966240163 |

## Executed controls

Portable evidence and actual frame identities are recorded in
[the artifact](artifacts/native-foliage-sunlight-2026-11-02.json). Raw commands,
frozen binaries, rejected attempts, images/HDR and logs are retained in the
session's `outputs/sunset-reference-2026-11-02`. Earlier rejected specimens and
the orange EV9 capture are not relabeled as accepted reference matches.

- CPU solar lookup: 72 ground-to-space cases versus the existing 512-step
  integrator; maximum channel transmission error 0.000349.
- GPU solar lookup: 63 cases through six publication states on RADV 880M;
  maximum error below 0.001, with growth, disable, restore and unchanged sky.
- Cached sky: 24 low/high-sun view cases, same 64 view steps versus 512 solar
  steps; maximum absolute radiance error per unit stellar irradiance 0.0000553.
- MASK CSM: five factor/cutoff cases over three layers match real cut geometry
  and direct/indirect depth exactly. MSAA alpha `[0,64,128,192,255]` resolves to
  sRGB `[0,137,188,225,255]` at 4×, versus hard single-sample MASK.
- Backface normal-map red/green: before, symmetric front/back lighting yielded
  `[96,139,61]` / `[21,35,10]`; after, both are `[96,139,61]`.
- Ray MASK: thresholds/endpoints, UV holes against independent cut geometry,
  farther opaque hits, large origins, minified moving checker and budget
  fallback. Thirty regular scene RT controls pass on RADV.
- Primary MASK: ground and spectral water, six alpha/factor/cutoff controls
  each, independent red/blue stars and moving/empty/restored geometry. Red-star
  occlusion leaves blue/ambient light unchanged; transparent holes reproduce
  the exact raster fallback. Opaque primary controls remain green.

No implementer preference is a blind-review result. Full-frame timings with
serial GPU trace fences are diagnostics, not game FPS or equal-work speedups.

## AMD Hybrid Shadows: integration boundary

Reviewed primary material:
[AMD overview](https://gpuopen.com/fidelityfx-hybrid-shadows/),
[presentation](https://gpuopen.com/download/GPUOpen_Hybrid_Shadows.pdf),
[Classifier manual](https://gpuopen.com/manuals/fidelityfx_sdk/techniques/classifier/),
[current sample](https://gpuopen.com/manuals/fidelityfx_sdk/samples/hybrid-shadows/),
and [2021 source at 0cf18422a73eda8b4a23870091372a7717387f3e](https://github.com/GPUOpen-Effects/Hybrid-Shadows/tree/0cf18422a73eda8b4a23870091372a7717387f3e).
The source is a read-only algorithmic reference. Its DX12/Windows host was not
ported or run; current SDK Vulkan support does not establish an executed Linux
integration. No external SDK/code was imported into the runtime.

The useful architecture classifies visible receivers into 8×4 tiles, compacts
nonempty tiles and carries a 32-bit lane mask plus the union ray interval.
CSM blocker search initializes known visibility and shortens uncertain rays;
finite-sun blue-noise rays and resolve/denoising follow. Reflection
classification is a different 8×8 feedback technique, not implemented SSR.
The inspected 2021 alpha helper uses LOD0 and fixed 0.5; Thessa instead needs
its material factor/cutoff and reflection footprint. The current Classifier
manual requests fp16; an independent WGSL port may pack tile coordinates in
u32 while retaining f32 SI intervals, without importing that shader ABI.

**The classifier/queue/denoiser is not implemented.** Native has depth, CSM,
shared scene TLAS and history epochs, but lacks a complete visible-receiver
normal/material publication before lighting. Its forward pass consumes
visibility during shading. A post-opaque classifier cannot correct that same
frame by multiplying final HDR: this would also darken ambient, emission and
other stars. Implement a matching receiver prepass plus lighting consumption,
or retain separate per-star direct contributions for a later resolve. Preserve
MASK holes, MSAA sample ownership and the existing material geometry.

Start with zero uncertain-tile omission, explicit overflow fallback and actual
source angular radii. Thin transmitting leaves need both hemispheres; an opaque
normal rejection cannot be copied blindly. A bright PCF sample is not proof
that a thin caster was represented. Camera/body/origin, geometry and light
changes invalidate history through existing epochs. TAA remains independent.

Calculated payload at 1280×720: 28,800 tiles, 115,200 B per u32 visibility field
and 460,800 B per worst-case 16-byte queue, per source. Three sources total
1,728,000 B **excluding** receiver buffers, histories and other targets. This
is neither a complete effect budget nor a measured whole-frame speedup.

## Remaining integrated-scene work

All user-reported gaps remain in scope; one solved leaf defect does not close
the scene. Acceptance requires fixed-input captures and affordable moving views.

| Gap | Current slice | Next acceptance |
| --- | --- | --- |
| Orange monotony / flat light | Atmospheric sunlight and common sky/cloud extinction, physical sunset epoch | Optional isotropic higher-order source and cooler ambient now implemented (doc 114); cloud/ground bounce, richer structured sky, coherent exposure and full visual acceptance remain |
| Ground / close microrelief | Existing scanned soil, canonical CBT height detail, roughness/normal channels retained | Distinguish soil, litter, moss and water at near/middle distances; remove repeating/mushy transfer and verify motion/filter continuity against declared scale |
| Wetness / puddles | Existing basin fill, bounded pools, wet soil and spectral response retained | Readable water depth/basin boundaries, Fresnel and confidence-checked screen reflections; current primary land/water scene-object reflections and refraction remain open |
| Instancing / medium tiers | Three scanned fern forms and existing world-addressed colonies, stable full lamina | Moisture/topographic cohorts, age/pose variation, intermediate cover and clearings without increasing fern counts indiscriminately |
| Horizon / composition | Actual skirt solar-position correction | Verify horizon geometry/residency separately; declare a review camera with grouped masses/leading lines and a focal point rather than the central trunk cutting the frame |
| Air depth | Existing spherical local haze and shared sky radiance | Height/distance separation, higher-order/cloud diffuse coupling and ground-to-orbit continuity; current haze is still an optical prior |
| Contact / thin shadows | MASK CSM and MASK RT, optional primary correction | Current-frame receiver buffers, conservative 8×4 sparse correction, finite-source sampling/denoising and moving thin-leaf controls |
| Frame cost | Much smaller foliage representation; per-layer trace admission | Matched ordinary GPU p50/p95, static mesh depth caching separate from CBT updates, bounded cloud/reflection work; no full-scene speedup claimed from unequal captures |
| Remaining mesh modes | MASK raster/shadow/ray coverage now implemented | BLEND ray/shadow transport, refraction and transparent-emitter lighting still explicit gaps; do not silently make them opaque |

The full canopy/floor density remains the appearance target. Bounded Low runs
are only diagnostics. New buffers/caches require resolved settings, capability
checks, declared memory/work budgets, a free disabled path and the same scene
owners. Compare dense rays, CSM and hybrid at identical residency/camera/light,
then prepare independent blind pairs; unmatched reference art is qualitative.

## Full-scene checkpoint and research intake

Actual full-density captures at 4°/8°/12° retain 11,320 accepted plants,
9,254 near understory descriptors, 19,652,948 candidate triangles and 2,768,212
primary submitted mesh triangles. Laminae are more coherent, but ground/light/
composition remain far from the photo target. None is accepted as parity.
The three times use different EVs; streamed geometry also differs. They are
appearance diagnostics, **not** matched performance or blind A/B evidence.

After bounded RT binding negotiation, a 640×360 full-population run admits
19,464 TLAS instances (including 64 selected terrain instances), 179,236
resident RT triangles and 25,874,720 planning bytes. Primary RT shadow allocation
and actual use are true. It completes with the same 20M **per-stage** trace
admission, recording individual shadow-layer fences. This is not a duration
guarantee, a raise of the admission limit or execution of the original four-
cascade/12 km High profile. Its three trace-fenced frames have median 635.260 ms;
that is plainly not affordable parity and cannot be compared with unequal
resolution/residency captures as a speedup. Primary correction stays default-off.

[Doc 113](113_RENDER_RESEARCH_INTAKE.md) reconciles the user's newly downloaded
handoff and 80-source catalogue against this newer implementation. Its attached
PNG decodes to exactly the earlier native foliage-v1 pixels; it is a diagnostic
baseline, not the original golden photographic target.

The reproducible raster-only profile is
`data/graphics/b03-sunset-diagnostic.toml`, semantically identical to the frozen
4°/EV9 diagnostic. Select `--scene b03-sunset` and
`--graphics data/graphics/b03-sunset-diagnostic.toml` with the existing isolated headless
capture flow and explicit per-stage trace admission. It preserves High plant
population while declaring one 100 m/1024 shadow cascade; this is a cheaper
shadow tier, **not** default High quality or an equal-quality performance
baseline. No launch defaults are changed by adding the profile.

Canonical export still stores the **unattenuated directional input** in its
legacy `sun.illuminance_rgb_lux`; stock Cycles B03 does not yet reproduce the
new local solar table, sky/cloud transport or three independent source slots.
Do not treat that existing adapter as a matched numerical sunlight oracle.
The independent CPU/GPU transport controls above are the current numerical
reference; external renderer export/transport parity remains open.

## Zero delivered-energy continuation

After the first publication, direct-light loops in primary terrain, meshes and
ray hits now test **delivered** stellar RGB after solar transport, rather than
only the top-of-atmosphere input. If the planet has extinguished a source
exactly, it cannot contribute direct radiance and needs no shadow lookup/query
or direct BRDF evaluation. No epsilon, low-light cutoff, material change,
density reduction or new resource is introduced. Ambient, emission and other
sources retain their existing paths. With solar transport disabled, delivered
RGB equals the input and the previous behavior is retained.

[The follow-up artifact](artifacts/zero-energy-shadow-gate-2026-11-02.json)
records 75 core controls plus rerun RADV library/solar, 30 scene RT, primary
land/water, MASK and surface-lighting suites. This avoids unnecessary receiver
queries; it does not cull the CSM caster layers or implement AMD tile dispatch.
The full scene was not re-timed to manufacture a speed ratio from changing
residency; an equal-work performance measurement remains open.
