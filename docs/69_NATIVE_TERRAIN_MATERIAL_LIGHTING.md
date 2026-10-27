# 69 — Native terrain material lighting

**Status: implemented primary-raster material normals, rough sky reflection and finite grazing GGX energy coupling; terrain visual acceptance incomplete.**

WORLD115 (doc 77) audits the subsequent Gemini forest-floor changes. Canopy
coverage still selects authored moss/litter cover, but no longer multiplies
bare substrate reflectance by an illumination factor. This removes the baked
28% canopy darkening; shared geometric direct shadows remain the lighting
consumer. A dry, cover-filtered fixture fails on the retained Gemini source
and preserves identical substrate albedo, roughness and height after repair.
This does not implement ground sky visibility or indirect illumination. Darker
litter/pebble optical priors from the review are retained as authored materials.

WORLD116 changes the authored wet-soil roughness endpoint from 0.18 to 0.48
before the existing fine-mineral, moss and litter composition. Moisture still
darkens intrinsic substrate reflectance; source heights, normals and canonical
moisture remain. This models an optically rough damp substrate without assuming
a continuous smooth water film from moisture alone. Bounded sampled puddle
geometry retains its separate water material and dielectric response. The 0.48
endpoint is a scene-development prior, not a measured soil calibration or a
claim of resolved porous-water transport. Native before/after crops show a
modest highlight-response change; moving blur and ground sky visibility remain
open. No new pass, resource, setting or quality policy is introduced.

WORLD117 (doc 77) changes only allocation of the bounded native moss-shoot
mesh. A stable world hash is weighted by distance to the active L18 cell centre,
spending more of the existing geometry near the cell while preserving sampled
neighboring-cell shoots. The optical moss pigment, physical height and shared
habitat fields do not change; neither do puddle/litter ordering or budgets.
This is a view-area representation policy, not moss growth. The ordering is
constant within a cell but can change across cell boundaries. Additional CPU
staging holds a copy of the pebble candidates and a cached sort key for each;
there is no extra pass, draw or persistent GPU allocation.

WORLD119 adds a distinct 1024² CC0 Forest Leaves 02 scan for moss-covered
forest litter, rather than deriving moss pigment detail from the gravel scan.
WORLD119 originally graded source RGB by [0.152, 0.325, 0.492]. The incoming
WORLD135 calibration uses [0.180, 0.350, 0.220]; roughness is still blended by
existing cover. Both grades are authored optical priors. The 3 m source width and 20 mm displacement range are authored rendering
priors. Its filtered displacement is mean-centred and multiplied by half the
moss coverage; the resulting cushion remains clamped to 0–60 mm. The same
world-fixed sample feeds render-only relief, prop/pool placement and material
normal generation. Existing triplanar stochastic mapping and source mip
filtering keep phase independent of pages, camera, LOD and worker order.

The packed asset adds 6,140,860 B to the binary and approximately 26.67 MiB
of CPU float mips once fine moss is actually sampled, plus a transient decode
buffer. Disabled ground detail and coarse/no-moss samples do not decode it.
Existing material pages, GPU atlas sizes, passes, draw ranges, quality gates,
canonical collision heights and CBT topology are unchanged. The source scan
contains moss, leaves and twigs; it is an authored optical proxy, not calibrated
Thessa biology or measured reflectance. [Source/license](../assets/textures/forest-moss-litter-02.LICENSE.txt),
source hashes and the reproducible offline importer are checked in. Current
source tests pin source mean, fine detail and world continuity; the prior
substrate-contrast fixture is adjusted because moss now uses its own scan.
The incoming WORLD135 review also increases vegetated-floor coverage through
shared ground/reed habitat weights, wet darkening to [0.65, 0.65, 0.58], and
changes authored pebble/litter/pool optical priors. Water roughness 0.02 applies
to explicit pools; damp opaque soil retains its 0.48 endpoint. These changes
are source-reviewed and captured separately in WORLD136; they do not implement
runoff, indirect illumination or calibrated wet optics. Realistic ground
illumination, moving residency and reference acceptance remain open.

WORLD91 (doc 77) gives resolved moss cover a connected cushion profile:
`smoothstep(0.20, 0.80, moss) * hummock_height`, with the existing 35–60 mm
hummock envelope. Moderate cover now contributes centimetres of geometric
height instead of being suppressed quadratically. The same body-fixed field
feeds render-only relief, optical gradients and prop/pool placement; canonical
terrain/collision heights and the existing quality/footprint cutoffs remain.
This is authored optical morphology, not a moss growth or drainage simulation.

WORLD108 (doc 77) also uses actual-frame visibility to order unresolved material
splits inside the complete wider prefetch cover. Visible pages above the error
target are split before offscreen pages; solved visible pages cannot stop the
remaining prefetch refinement. Missing projection matrices keep the former
error-only order. The 2,048-leaf High cover, 512 admissions, 1,024 CPU pages,
256 GPU layers and resolved quality ceilings are unchanged. A saturating
four-root fixture improves visible error from 1.109 to 0.989 pixels with
2,047 leaves and identical covered area. The standing capture has more L22/L23
residents, but no resident L24 pages; this does not guarantee native close-up
resolution, remove all motion blur, or change canonical terrain/CBT topology.

WORLD43 (doc 77) separates optical material visibility from the geometry cone's
0.6 rad prefetch margin. Shared material admission priority now uses the actual
camera-relative projection and the existing conservative sphere/clip predicate;
native and Bevy both provide it. Missing matrices keep the prior cone fallback.
GPU/CPU page budgets, quality caps, the shared broad refinement cover and soil-height
prefetch remain. Spheres still
use the cached local height reference; this is not exact per-page height or
occlusion. Regression and capture evidence are in doc 77 WORLD43.

WORLD42 (doc 77) preserves the primary soil scan's original 1024² samples instead
of the previous 512² downsample: native 3.08 mm source spacing now matches High
L24 pages more closely. Each source uses its own dimension for mip selection;
the mud/litter scan stays 512². Native-resolution intrinsic transfer precedes
linear-light mip filtering, with updated matching coarse means. GPU page budgets,
quality ceilings and shading/geometry contracts remain; the CPU float source
pyramid costs an additional 20 MiB. Extended regular/conforming GPU oracles pass
through L24 without a production shader change. Exact source provenance,
checks, captures and remaining contact/temporal limits are in doc 77 WORLD42.

WORLD22 ([doc 77](77_NATIVE_SURFACE_SHOWCASE.md)) replaces WORLD15's
matched-level residual with canonical base-height and full optical gradient
composition. Native geometric ground relief keeps RGBA8_SNORM (full optical
normal RG, base-height normal BA, bit-30 validity). Material and triangle source
levels may differ; absent/old fields retain geometry. Ordinary normals retain
RG8_SNORM and normals-off a 1² dummy. Existing page/cache/priority and quality
policies remain. Captures expose `native_material_height_field_pages`.
Medium/High also evaluate SH9 diffuse irradiance from the existing sky field
and filter specular roughness by raster normal variance; Low keeps the original
isotropic/unfiltered fallback. See doc 77 for memory costs, source-height priors,
independent CPU/GPU oracles and known optical reconstruction limits.
Earlier dated implementation sections describe the representation at that time.

WORLD41 removes an extra factor of two from the soil scans' physical source
prefilter footprint; native scan spacing retains mip zero before the existing
resident-page derivative filtering. Both scans keep linear-light mip averages,
world addressing, coarse means, normal composition and unchanged height bounds.
Litter pigment retains bounded scan contrast. This changes optical/soil detail,
not canonical collision heights or the CBT algorithm; motion/alias acceptance
remains open. Exact settings, checks and captures are in doc 77 WORLD41.

WORLD23 extends the same soil field with spatial wetness and bounded moss
cushion height, shared by render displacement and optical normals. Native puddle
meshes use the shared material's validated `dielectric_f0` (0.04 default, about
0.02037 for the water prior); reflected hits consume the same property. Existing
primary uniform padding is reused; RT material records are now 64 bytes.
See doc 77 for sampled closure, Low fallback, limits and static-water scope.

WORLD40 audits the subsequent Gemini scene. Secondary mesh/terrain material
lookups retain Medium/High pixel-cone mip filtering (Low point fallback, existing
RT terrain L17 ceiling), not stochastic GGX tracing. A single mirror ray is
traced per bounce regardless of the requested 1/2/4 sample policy value. Shared
TLAS terrain admission now uses live mesh pairings and preserves whole-partition
fallback. Configured ray bias replaces the demonstrated distance-heuristic
regression; complete error-bounded ray spawning remains open (docs 77/79).

The [surface-showcase continuation](77_NATIVE_SURFACE_SHOWCASE.md) preserves
material coordinates as a dyadic anchor plus small offsets through raster
interpolation and page lookup. WORLD13 extends this precision contract to
persistent conforming bisectors, reconstructing a common triangle face from
exact integer addresses. The default material limit remains L17; optional
ground detail permits bounded L20/L21/L24 material refinement on both topology
paths (WORLD22); geometric relief retains its independent L22 maximum.

With detail enabled, `surface_quality` Low/Medium/High resolves 1/4/16x material
anisotropy against the adapter capability. Unknown/unsupported capabilities use
1x with a note; disabled detail retains 1x. Both topology consumers choose source
resolution from the minor footprint axis bounded by that ratio, retain the
major axis for conservative coverage/globe transitions, and keep mip filtering.
`terrain_material_anisotropy` is resolved capture metadata, not a hidden toggle.

The detail stream uses one cached canonical 32 m-wavelength ground elevation at
the eye direction per material selection epoch/pixel-scale invalidation, for
both refinement distance/frustum and admission priority. This is a local optical
reference, not exact per-tile relief, geometry displacement or collision state.
Default-off selection retains the legacy datum. WORLD12 in doc 77 uses
512/1,024/2,048 temporary refinement leaves for Low/Medium/High detail, then admits
at most 512 pages from the wider prefetch cone, ordered by actual-frame
visibility and error. WORLD108 applies that visible-first ordering to unresolved
refinement candidates as well. The 1,024-page
CPU source, bounded worker jobs and 256 GPU layers are unchanged. Captures expose
`native_material_reference_height_m` and its source/limitation string.

## Ground optical material foundation (WORLD12; extended in doc 77)

The optional scan is processed into continuous humus/mineral reflectance and
roughness before mip averaging, preserving the same processed-material mean
across levels. One shared forest-floor evaluation adds moisture, moss and litter
optics. Canonical ecological cover and later substrate/cover layers retain
ownership. The source height is a single authored 60 mm relative range at a
3.15 m repeat; physical chart differences produce the normal without secondary
slope gain. These are uncalibrated optical priors, not pebble geometry, ambient
visibility, water simulation or authoritative displacement. This supersedes the
18 mm and colour-layer descriptions in dated checkpoints below. See doc 77 for
review findings, scoped validation and source-pinned actual captures.
WORLD13 adds continuous stochastic mapping, mean-centered source relief in
fine CBT height pages and bounded native pebble meshes under the same resolved
effect (docs 63/70/75/77). These are render geometry, not authoritative collision.
WORLD22 reconstructs base-height plus full optical normals where the optional
canonical plane is resident, avoiding double slope and matched-level suppression.
The two CC0 scan sources and current coverage/relief contract are in doc 77.

## Near shadow precision

`shadows.first_cascade_distance_m` optionally sets the first shadow cover radius
without enlarging the atlas, adding cascades or shortening the far cover. For
two or more cascades, native radii interpolate geometrically from this radius
to `shadows.max_distance_m`; one cascade retains the far radius. The shared
resolver clamps finite requests to 1 m..far cover, and a non-finite request
falls back with a note. Omission preserves each host's legacy distribution;
native legacy first radius is `far * 40^(1/count - 1)`, not `far / 40`.
The legacy Bevy adapter forwards the optional first radius into its existing
cascade builder. No renderer-specific control bypass is introduced.

The 64 MiB three-star/four-cascade native policy resolves the requested 4096 px
atlas to 1024 px per layer (48 MiB). At 12 km far cover its legacy first radius
is 754.460 m, giving 1.479 m per texel including the snapping guard. An explicit
25 m first radius gives 25/195.743/1532.619/12000 m radii and 4.902 cm first-map
texels with the same allocation/pass budget. This is a precision redistribution,
not soft-shadow physics or evidence that thin foliage is now resolved.
`normal_bias_m` remains a separate existing control; changing cover alone does
not prove contact fidelity or remove material-normal grazing artifacts.

The requested/resolved optional first radius round-trips; captures report
`shadow_first_cascade_distance_m` (`legacy` when omitted) and
`shadow_max_distance_m`. The disabled path remains atlas-free. CPU tests pin
count/endpoints, legacy equivalence, monotonic spacing, invalid inputs and a
conservative <1 cm f64-to-f32 clip-space narrowing envelope at a 3e12 m origin.
The production shared-shadow GPU fixture also runs with explicit near cover,
retaining independent light occlusion, exact cache/forced-redraw equality and
large-origin behavior. Visual and moving-camera acceptance remain separate.

Opaque mesh receivers now use the same geometric receiver-plane rule as terrain:
screen derivatives of the interpolated position determine the triangle normal,
oriented consistently with the visible surface. Interpolated and normal-mapped
normals remain inputs to lighting, not to PCF depth gradients or geometric ray
visibility bias. A degenerate derivative retains the existing normal fallback.
This avoids interpreting a smooth cone side or a bump map as a different shadow
receiver plane; it does not flatten the BRDF, alter caster geometry or remove
real leaf/branch shadows. Without a visibility consumer the shader optimizer can
discard the unused plane calculation; no new pass, allocation or setting is
introduced for this correctness fix.

The GPU regression uses a single flat, unoccluded quad with strongly varying
smooth normals and zero metre bias. Enabling shadows previously changed pixels
by up to 18 sRGB codes; the geometric-plane implementation passes the same
<=1-code bound against shadows disabled. This is a known-flat-plane lighting
oracle, not subjective visual acceptance. The normal-map grazing bands on the
ground remain a separate issue.

## Source and consumer

Regular and conforming native terrain now receive the existing shared sky field
through their rough dielectric BRDF, not only through a stellar highlight and
diffuse ambient. The material remains sRGB albedo plus linear roughness from
the existing page residency/mip/ancestor path, with an optional tangent-space
normal plane used by the primary raster. Geometry, authoritative heights,
contacts and simulation are unchanged. No new lighting-field producer, render target,
render pass or ray-tracing requirement is introduced; the material page gains a
second texture plane and the primary raster declares its two bindings.

`render-wgpu::environment.wgsl::env_specular_response` integrates incoming radiance
against the same GGX distribution, Schlick-Smith masking and Schlick Fresnel used
by `rcbt-core`'s direct dielectric kernel (F0 = 0.04). It importance-samples
visible GGX normals using Heitz, *Sampling the GGX Distribution of Visible
Normals*, JCGT 7(4), 2018. Deterministic Hammersley quadrature uses no
frame-random noise or history buffer; it does not certify motion stability.
The visible-normal PDF uses exact GGX G1; the weight
corrects it to the existing Schlick-Smith BRDF. Below-surface samples contribute
zero and remain in the denominator; renormalizing survivors would create energy.

Every accepted direction uses shared sky radiance plus the existing
reflected-cloud extinction/in-scatter law at the surface origin, with a bounded
reflection-specific quadrature budget (below).
It does not add the directly evaluated stellar disc a second time. The legacy
shader hook returns zero reflected radiance, preserving the runnable Bevy path
without native bindings. Native material transmission uses the same
directional-hemispherical reflectance on both incoming and outgoing paths,
independent of sky colour or cloud publication/transport budgets. It models
one Lambert-substrate encounter, omitting internal return bounces; this is not
a complete refracting rough-interface or multiple-scattering solver. Legacy
Bevy retains its existing macro-normal Schlick outgoing split and unattenuated
incoming diffuse, without native environment bindings.

The shared native GGX VNDF kernel has one-sided outgoing support: a negative
`dot(shading_normal, eye)` returns zero weight before constructing the stretched
view. At exactly zero the existing finite analytic grazing limit is retained;
there is no positive radiance floor, epsilon hemisphere expansion or normal
rotation toward the viewer. Previously clamping a negative cosine to zero while
retaining the original view vector incorrectly produced reflected energy behind
the shading hemisphere, and an exactly opposite view normalized a zero vector
and propagated NaNs. The correction applies to the existing shared material
kernel, not a terrain-specific coefficient or extra effect switch.

A 45-case actual-WGSL GPU regression covers five negative view cosines, three
roughnesses and 4/8/16 samples, with RGB dielectric/metallic F0 values. Radiance
and directional reflectance must be exactly zero; the existing converged f64
solid-angle reference still pins the finite zero/positive grazing limit. This
does not solve normal-map microvisibility, unresolved slope variance, shading-
normal energy compensation or all grazing visual artifacts.

WORLD06j isolates the remaining reference-scene ground bands to the direct
stellar GGX lobe, not insufficient clear-sky quadrature: a temporary 256-sample
clear-sky capture retains the bands, while an environment-only half-frame does
not. A false-colour outgoing-support capture reproduces their contours where
the bump normal crosses the view hemisphere. The temporary diagnostics are not
renderer settings or retained production paths. The shared direct kernel still
uses the shading-normal projected area; a complete macro/micro visibility and
filtering treatment is not implemented. A direct-only two-facet prototype was
removed: it softened the contours but did not close the visual defect and did
not consistently extend sky transport or the substrate energy split.

The retained anisotropic-environment hardware oracle uses 27 combinations of
three small normal tilts, view cosines 0.001/0.02/0.2 and roughness 0.6/0.85/1.
A synthetic oblique 32x16 field is bounded by radiance 100. Uniform incident
solid-angle f64 quadrature samples the same bilinear grid, independently of the
GPU's VNDF sampling/PDF. The 256x512 and 512x1024 reference grids must converge
within 0.00003; 4096 actual-WGSL samples must agree within 0.0001; production's
16 samples have a fixture-specific absolute envelope of 0.04. These are linear
radiance bounds, not relative error near zero, arbitrary-HDR accuracy, sky-grid
convergence or surface visual acceptance. Production quality budgets remain
4/8/16; test-only dense sampling does not introduce a new effect tier.

## Tangent-space material normals

The native material page carries an optional 128×128 `RG8_SNORM` normal plane
with the same eight mip levels as albedo/roughness. Each component is
`round(clamp(component,-1,1)*127)` stored as an i8 byte; positive z is
reconstructed in the shader. Producer and CPU mip generation share
`encode_material_normal_component` / `decode_material_normal_component`.
The GPU texture performs signed decoding before bilinear/trilinear/anisotropic
filtering; shader sampling must **not** apply an additional `*2-1` transform.

WORLD06k replaces the earlier unsigned representation: RG8_UNORM cannot encode
zero under `2*byte/255-1`. Its nearest pair of codes yields +/-1/255 per component;
the old supposedly flat `[128,128]` plane tilted the normal by approximately
0.318 degrees. Tiny positive/negative slopes were forced onto opposite nonzero
quantization levels. SNORM preserves zero exactly and rounds symmetrically, with
component error <=0.5/127 (+f32 decoding roundoff). The -128 byte is a valid GPU
alias of -1; the canonical encoder emits -127 instead. The component bound is
not a global normal-angle, slope or grazing radiance error bound.

This is a material-normal byte-contract change, not a different terrain/source
height law. `GpuMaterialPage.normal`, `CbtMaterialPage` normal constructors and
decoded normal chains now all require signed bytes. Old unsigned normal payloads
must be regenerated/re-encoded; there is no reliable byte-level auto-detection.
Normal-less decoded pages, existing albedo/roughness bakes, legacy RGBA normal
exports and the Bevy albedo-only path remain unchanged. Native normal pages are
runtime-produced; no checked-in unsigned normal-plane asset is migrated.
The `dump_material_pages` example decodes signed bytes into conventional
unsigned RGB normal-preview PNGs; the preview format is not the runtime format.

The 24-case neutral-plane GPU oracle compares actual regular/conforming raster
frames with an A/B/A geometry-stability control and requires exact equality.
The old representation failed every case, up to 32 linear codes; signed normals
pass. Canonical standing/close/overview frames still retain the ground bands.
This is an encoding correction, not a microvisibility/filtering solution or
visual acceptance. Reproduction and measurements:
[`native-signed-material-normals-2026-10-23.json`](artifacts/native-signed-material-normals-2026-10-23.json).

### Independent projected-geometry boundary (WORLD06l)

`tests/terrain_environment/normal_geometry.rs` supplies a **CPU reference**, not
a selected renderer transport model. Its unit-period triangular height field
has dimensionless slopes +/-s, is extruded along y, and carries opaque Lambert
facets with reflectance 0.5. Exact first-peak visibility intervals are checked
against 12,800 explicit ray/segment intersections. Integrating visible facet
area times the local view cosine must recover the macro projected area. The
BRDF integrates **joint** light/view visibility and divides by both macro
projected areas; reciprocity, the flat Lambert limit and single-bounce passive
energy are then checked separately. Four slopes (0/0.02/0.2/1), five view cosines
(0.001/0.01/0.1/0.5/1) and five azimuths are exercised. Sixteen directional-energy
fixtures compare 64x128 and 128x256 midpoint quadrature with an absolute 0.0001
reflected/input-energy convergence envelope, not a relative weak-signal bound.

A temporary test-only hook in the actual canonical page producer also measured
23 selected L17–L22 pages along the standing camera's 0/10/40/80 m forward strip.
Procedural grain RMS slopes are 0.095923–0.185048, versus 0.000586–0.005933 for
the weighted imported-height contribution: ratios 16.51–170.37. Ground coverage
means are 0.70487–0.70560. These are raw per-page source contributions, not an
inventory of resident page IDs, pixel footprints, mesh normals or actual BRDF
outgoing support. The instrumentation was removed; no source amplitude, material
mixing, visibility multiplier or lighting law changed.

The reference establishes neither a physically complete normal-mapping model
nor agreement of current production GGX/sky/substrate transport with this
triangle-wave surface. GGX facets, multiple scattering, general geometry,
normal-filtering error and canonical-image acceptance remain open. In
particular, the large source ratio does **not** justify attenuating the legacy
relief solely to hide the bands. Evidence:
[`native-normal-geometry-reference-2026-10-23.json`](artifacts/native-normal-geometry-reference-2026-10-23.json).

Generation and residency:

- `build_gpu_material_page` evaluates `surface_normal_grain_height` on land
  and the one-texel cardinal halo required by its derivative stencil. Zeroing
  the source at a water-side stencil point would create a false shore cliff.
  Ocean output normals stay flat and all-ocean pages evaluate no grain; the
  water branch owns its wave normal. The halo is shading data, not seabed relief.
- The normal height spectrum uses Perlin gradient noise at 8 / 4 / 2 / 1 / 0.5 m
  with equal slope contributions where the page texel resolves them, plus the
  broader 32 / 128 / 512 m bands. The existing `band_weight` filter smoothly
  attenuates bands from four to two texels per wavelength; finer bands are
  discarded. The single 8 m value-noise octave used by the CPU residual path
  read as a cellular quilt when promoted to a normal source; the gradient-noise
  variant fills the resolved spectrum instead. This is a representation change,
  not a new terrain law.
- The existing filtered material response now supplies a bounded normal-strength
  and roughness adjustment from exposed geology, loose-cover/deposition,
  fixed-scale slope/curvature, moisture, ecological cover, snow and frost. The
  per-geology values are small authored rendering priors (profile coefficients
  0.92–1.08 before moisture/cover attenuation; roughness offsets at most 0.012),
  not measured lithology parameters. The strength scales the solved tangent
  gradient, not a second height signal, so a categorical geology boundary does
  not become a synthetic relief step. No noise bands, phase, authoritative
  height, collision, water classification or vegetation geometry are added.
  Water remains flat and snow/vegetation attenuate exposed-substrate detail.
  Geology-linked roughness offsets are weighted by uncovered substrate, so
  opaque vegetation/snow/frost do not inherit buried-rock roughness priors.
- Slopes use the inverse local cube-sphere chart metric: the two physical
  direction chords and height differences define a 2×2 solve in the face's
  orthonormal tangent frame. Dividing both differences by one nominal texel
  width is wrong away from face centre, where the chart is skewed and stretched.
  The independent planar-gradient test recovers `[0.3, -0.2]` to `1e-12` on
  all six faces at centre/corners. The one-texel page apron
  makes the inner texel 1/126 derivatives use exactly the same field samples as
  the neighbouring same-level page, so normals meet at shared tile edges.
- Normal mips decode XY, reconstruct positive z, average the actual height
  gradients `normal.xy / normal.z`, and reconstruct a unit normal. Averaging
  unit-normal XY is not slope averaging (the initial prototype did that).
  A `z >= 0.01` numerical floor bounds extreme/invalid input in both CPU and
  shader decoding; it is not a guarantee for arbitrary grazing normal fields.
  Within a mip, hardware bilinear/trilinear filtering interpolates decoded
  signed unit-normal XY before z/slope reconstruction, not exact gradients.
  Quantization, this approximation and conservative nominal texel-width band
  filtering remain representation errors; the analytic plane test is not their
  general bound.
- `render-wgpu::material::MaterialArray` uploads normals to a separate
  `Rg8Snorm` texture array and publishes a second sparse directory containing
  only pages with a normal plane. `MaterialUpload.normal_pages` and `.bytes`
  account for the extra writes. Pages without normals are absent rather than
  sharing a stale slot.
- The native page with normals is 131,070 bytes of mip payload versus 87,380
  bytes for albedo/roughness alone. Base payload for 512 pages rises from
  33,554,432 to 50,331,648 bytes before mip expansion. At 1024 resident slots,
  the additional logical normal-array payload is 42.67 MiB (driver alignment
  can add overhead). The page benchmark also includes a paired 64-page
  close-land workload at levels 14–17 that actually resolves the fine bands.

Shading and consumer boundary:

- `crates/rcbt-core/src/shaders/material_normal.wgsl` is included only in
  `CBT_RASTER_WGSL`. Its `material_at_level_raster` repeats the shared page
  hierarchy, coverage feathering and derivative footprint, but samples the
  normal plane in the same pass as albedo. This avoids a second residency
  traversal and keeps the shared `material_sample.wgsl` binding-free of normals
  for RT terrain, ocean geometry, conforming surfaces and tests.
- The decoded slope is transformed through the cube-face tangent basis and the
  body-to-render rotation, projected onto the geometric tangent plane without
  independently renormalizing its axes, then applied to direct diffuse, land Fresnel,
  direct sun reflection and the shared sky/cloud reflection. The perturbed
  normal does not move the shadow receiver plane, water classification, height
  or night-light mask: those remain geometric/geographic, and the city-light
  mask is emissive rather than a BRDF response.
- Residency and coarse/fine blending operate on premultiplied gradients.
  Missing normal coverage contributes flat; it must not be divided out and
  amplified to full strength. The normal influence shares the globe-map fade.
  The existing roughness/height water classifier suppresses material normals
  before direct diffuse as well as reflections, so a filtered land neighbour
  cannot perturb open-water lighting.
- The Bevy consumer binds its existing material array view at binding 19 and a
  one-entry zero normal directory at binding 20 to satisfy the shared layout.
  Its compile-time normal switch is false, so it performs no per-fragment
  normal-directory lookup or texture sample and allocates no second texture.
  Bevy pages remain albedo-only.

The actual primary-raster oracle (`material_normal_plane_perturbs_actual_raster_diffuse_and_clears_on_removal`)
renders a flat page and a constant +X-tilted page under a +X sun on the real
terrain pipeline, checks the independent unsaturated Lambert reference within
2 RGB8 levels, then removes the
normal plane and requires byte equality with the flat frame. This proves
upload, directory publication, sampling, tangent transform and stale-entry
clearing. The same fixture checks byte-identical flat output with the resolved
normal switch disabled or on open water, and bounds the barely-covered corner
of a lone fine normal page within 5 RGB8 levels of flat (no coverage
amplification). It is not a general normal-map error bound. Same-face horizontal
and vertical shared-edge tests use level-16 land with nonzero resolved relief;
general nonlinear cross-face/LOD seam envelopes and temporal sweeps remain open.
Disabled native residency accepts incomplete optional normal mip chains from
decoded pages because it neither uploads nor samples them; the enabled raw
texture-array path still rejects incomplete chains before residency changes.

Early development captures at sites 0/1/2, 500 m and 5 km, showed added grain
but also regular wind-ripple-like patterns on some slopes. Those captures used
the pre-correction chart/normal blending and are not evidence for the final
algorithm or accepted natural ground cover. Image gradient energy alone is not
a quality metric: aliasing can increase it as easily as meaningful detail.

### Final normal-plane measurements

The corrected implementation was validated on Radeon 880M / Vulkan / RADV,
Mesa 26.2.5. Evidence, exact commands, log/capture hashes, limitations and the
compiled-input manifest hash are recorded in
[`native-terrain-normals-2026-10-20.json`](artifacts/native-terrain-normals-2026-10-20.json).
All 13 local gates passed on 614 byte-identical compiled/configuration inputs:
1979 workspace lib/bin tests, 98 integration tests, 84 native feature-off tests,
73 renderer ignored hardware tests and 43 native ignored tests. The required
Bevy ignored-test command compiled successfully but matched zero tests after
the earlier optics migration; it is not eight more executed GPU tests.

| Diagnostic | Normal disable | Normal enable |
| --- | ---: | ---: |
| 64 paired close-land pages, p50 / p95 ms | 120.829 / 124.704 | 124.223 / 128.376 |
| 64 paired pages, aggregate seconds | 7.794 | 7.981 |
| 128 dirty-page upload bytes | 11,184,640 | 16,776,960 |
| CPU validation/residency/enqueue p50 / p95 ms | 3.792 / 3.962 | 7.964 / 8.378 |
| Upload plus queue submit/GPU drain p50 / p95 ms | 6.906 / 7.388 | 13.187 / 14.157 |

The final broad 512-page generation run measured p50 / p95 107.112 / 129.765 ms,
55.027 s total. Generation does not include mip building, upload or rendering.
The paired 64-page workload uses close land at levels 14–17, counterbalances
on/off order, and measures the **final** filtered Perlin source, unlike the
superseded value-noise prototype. Neither is a streaming-latency measurement.

The separate 1280×720 diffuse-only regular close-land fixture measured
classify/raster medians 1.047 ms disabled, 1.105 ms Low, 1.104 ms Medium and
1.102 ms High (32 timestamp samples each in forward/reverse order).
That is approximately 0.06 ms added normal lookup/shading in this fixture;
the tiny tier differences are not an established performance ranking.
The upload fixture has ten measured dirty snapshots per mode after warmup,
counterbalanced, with no raster. Drained wall time is not isolated DMA time.

Fourteen silent 1280×720 frozen-epoch native captures cover all three sites at
500 m / 5 km, site 2 at 200 m, three normal-disable controls, two conforming
controls and Low/Medium detail at site 2/500 m. Six High regular on-frame GPU
medians span 76.619–106.621 ms. At site 2/500 m, on/off medians were
89.973 / 88.712 ms; at site 1/500 m, 106.621 / 109.851 ms. These are one-run
whole-client measurements, not isolated effect cost or a converged speed claim.
Quality captures also alter reflection budgets. All final material-missing/job
counters are zero; on/off visible/cache counts match, but the complete streaming
snapshot is not identity-pinned. All fourteen have zero channel-sum-below-45
pixels, with minima 116–196; that statistic does not prove absence of geometry
holes or visual acceptance.

The earlier captures show a shared generic grain carrier with only a
bounded material-state response, rather than convincing stratum, fracture,
scree or weathering relief; smooth macro slopes, pale snow/green fields,
grazing directional stretching and blue-grey ridge bands. The
conforming controls still show faceted silhouettes. No plant/rock scatter
geometry or independent blind preference review is claimed. These are
outstanding defects, not a completed natural-terrain or Satisfactory-quality
milestone. The audit below removes the erroneous grazing energy/floor behavior;
it does not supply missing geological or vegetation geometry.

## Material-response follow-up — 2026-10-20

The existing normal carrier now receives a conservative, per-page-texel
strength response from the already-computed exposed-rock/loose-cover masks,
geology, fixed-scale slope/curvature and erosion/deposition evidence, moisture,
ecological cover, snow and frost. Exposed-rock and loose-soil coefficients are
bounded to 0.92–1.08; geology roughness offsets are bounded to ±0.012. These are
authored rendering priors, not calibrated rock mechanics or mineral data. The
response scales the solved chart-space gradient rather than introducing a new
height signal; it adds no frequencies or phase changes and cannot affect
authoritative terrain, contacts, hydrology, water classification or vegetation
geometry. Water has zero normal-detail strength; vegetation/snow attenuate the
substrate response.

The separate fallback fixes are also pinned: Bevy's shared raster compiles out
normal-directory lookup/sampling, and native residency does not reject an
incomplete optional normal mip chain when that feature is disabled. Enabled
normal residency still requires a complete chain and validates the full update
before mutation.

Release/bench measurements on Radeon 880M / Vulkan / RADV Mesa 26.2.5:

| Workload | Current measurement | Boundary |
| --- | ---: | --- |
| 512 serial 128×128 pages, six faces, levels 10–14 | p50 105.488 ms; p95 126.792 ms; total 54.133 s | Material generation only, not loading/frame time |
| 64 paired close-land pages, levels 14–17, normals disabled | p50 118.946 ms; p95 121.271 ms | Page-build CPU time |
| 64 paired close-land pages, levels 14–17, normals enabled | p50 122.114 ms; p95 133.684 ms | Page-build CPU time; no performance win claimed |

Eight silent 1280×720 real native captures cover normals on/off at sites 0/1/2
at 500 m and site 2 at 200 m. Final visible/cache counts match within each pair
and material-missing is zero, but whole streaming snapshots are not identity-
pinned. Measured 32-frame GPU medians vary from 80.921–106.251 ms across the
captures; these are whole-client observations, not an isolated effect cost or
stable on/off ranking. Four matched pairs are prepared for blind review in
`target/test-tmp/native-material-response-final-2026-10-20/blind-review`; no visual
acceptance is claimed. Gate, capture-hash and benchmark details are recorded in
[`native-terrain-material-response-2026-10-20.json`](artifacts/native-terrain-material-response-2026-10-20.json).

## Settings, fallback and accounting

- `[renderer].terrain_normal_maps = false`: native streaming omits normal-height
  grain evaluation, normal-detail response evaluation and normal mip generation;
  upload skips normal bytes; the renderer
  uses only a 1×1 dummy normal texture and the shader's normal branch is a false
  compile-time constant. No full normal array or normal sampling remains.
  The setting is restart-required. Albedo/roughness and physical heights stay
  identical. The default is enabled.
- `surface_quality = low / medium / high` applies +2 / +1 / +0 normal mip bias
  through the resolver. Lower tiers filter more strongly and improve texture
  cache locality, rather than changing physical relief or normal strength.
  Metadata records `terrain_normal_maps` and `terrain_normal_mip_bias`.
- `[renderer].specular_reflections = false`: the native hook is compiled to
  zero, with no quadrature or land-only sky bindings. Other sky/cloud/ocean
  consumers retain their independent settings.
- `[renderer].surface_quality = "low" | "medium" | "high"`: 4 / 8 / 16
   clear-sky samples per land fragment, resolved by
   `ResolvedGraphicsSettings::terrain_environment_samples()`.
  The same budgets evaluate material reflectance for the view and each active,
  above-horizon star. Nonzero aggregate ambient adds four cosine-weighted
  midpoint directions, each using that material budget. This incoming ambient
  approximation assumes isotropic illumination because the existing irradiance
  does not retain angular information. It does not introduce another sky field.
- When clouds are published, use 1 / 2 / 4 directions and 1 / 2 / 4 cloud
  view steps, capped by the cloud producer's steps, and at most one sun-depth
  step (zero if the producer disables sun-depth integration). These budgets
  are resolved by `terrain_environment_cloud_budget()`. Density, phase,
  extinction, scattering and self-shadow laws are unchanged; only numerical
  sample counts change. Each hollow-shell segment holds the physical source at
  its midpoint and solves Beer-Lambert analytically after density quadrature;
  it does not run the full nested primary march per GGX direction. Primary
  clouds and existing mesh/ocean reflection budgets remain unchanged. Material
  transmission retains its clear-quality budget even with clouds, so publishing
  an optically empty cloud deck cannot change the material's direct response.
- No published atmosphere/aurora sky field: zero added land reflection, rather
  than an invented ambient sky. Native direct/diffuse still use the two-sided
  material transmission model (and its quadrature cost), unlike the former
  macro-normal Schlick approximation. The no-field, nonblack direct-lit case is
  tested against an independent reference and included in the GPU cost fixture.
- No new device capability or native backend is required beyond the current
  terrain path. Ocean classification and specular disable behavior are unchanged.

`TerrainRenderer::land_environment_samples()` reports the applied budget;
native capture metadata records `native_terrain_environment_samples`,
`native_terrain_environment_cloud_budget` and the consumer's scope. Cost belongs
to `render.scene_classify_and_raster`. The sky-reflection slice adds no persistent
large allocation; the optional normal array is accounted above. A rejected first implementation nested the
full primary cloud march under every sky sample: measured 1280×720 COAST GPU
median was about 1877 ms, versus 38 ms with specular disabled. These are a
diagnostic of unacceptable added cost, not an optimization win or an isolated
environment-only comparison. Reflection-specific budgets replace that path;
sample counts are still **not** a frame-time guarantee. The optional diagnostic
uses actual GPU timestamps and reports unavailability when timestamp features
cannot be negotiated.

## Grazing audit: finite visibility ratio and coupled material energy

The horizon floor introduced by `5088319` is removed. Its historical evidence
remains in `artifacts/native-terrain-horizon-guard-2026-10-20.json`, but the
interpretation that an empty quadrature justified adding a sky floor was wrong.
Being bounded by incident sky radiance is not a BRDF error bound: the expanded
solid-angle oracle reproduced RGB8 255 instead of 140 at roughness 0.6 and
`N.V = 0.05` before this correction.

Two errors compounded at grazing:

1. The PDF correction computed `Gv_Schlick / Gv_exact` by separately evaluating
   two terms proportional to `N.V`, then clamped their denominators. This loses
   the finite ratio at `N.V = 0`. With `alpha = max(roughness², 0.0025)` and
   `k = (roughness + 1)² / 8`, algebraically cancel `N.V` first:

   `ratio = (nv + sqrt(alpha² + (1 - alpha²) * nv²)) / (2 * (nv * (1 - k) + k))`.

   The exact grazing limit is `alpha / (2k)`, not zero. Rejected directions
   still contribute zero and remain in the denominator. There is no sky floor,
    angular brightness gain, new source or extra reflection sample/pass.
2. The diffuse term used macro-normal Schlick Fresnel, approaching one at
   grazing regardless of roughness, while the reflected term used the rough
   GGX integral. This removed far more diffuse energy than the specular lobe
    could reflect and produced dark ridge dashes. With `R` denoting that same
    law's directional reflectance, native diffuse now uses the reciprocal
    first-substrate-bounce BRDF
    `albedo * (1-R(view)) * (1-R(light)) / pi`. Aggregate ambient uses the
    cosine-weighted average incoming `R`, with the isotropy limitation above.
    Unreflected energy is treated as substrate throughput; a complete rough
    refraction/transmission solver and internal return bounces are omitted.
    For exact `R`, hemispherically integrated outgoing energy for white
    substrate is `R(light) + (1-R(light))*(1-average_R) <= 1`.

An intermediate outgoing-only complement passed an exact-white-furnace test
but failed independent review: its nonreciprocal direct response generated
approximately 2.5% excess energy at roughness 1 and incident cosine 0.01. It
was rejected before committing. The revised omitted-bounce model intentionally
has a white-furnace deficit, not exact equality. Its material quadrature is
also independent of the cheaper reflected-cloud transport budget.

The actual shared-WGSL compute probe at roughness 0.5 and uniform radiance 100
returns 6.03526 at `N.V=0` and `1e-7`, versus an independent solid-angle result
5.95017; at 0.05 it returns 6.05356 versus 5.92255. The absolute tolerance is
0.25 radiance in this fixture, with 512/1024-grid reference convergence below
0.01 radiance. Exact and near-grazing outputs differ by less than 0.0001.
The raster oracle now covers `N.V = 0.7/0.2/0.05/0.01`, regular/conforming,
with the existing 8/255 fixture envelope. White/grey furnace-deficit and
no-sky direct-lit tests cover all quality tiers and both topologies, within
1 RGB8 level at High and 2 at cheaper tiers. Test exposure is chosen to make
missing incoming transmission observable, rather than hidden below tolerance.
An additional actual-WGSL probe integrates collimated-source outgoing energy
on an independent 128x256 solid-angle grid at roughness 0.45/0.6/1, incident
cosine 0.01/0.2/1 and all three budgets. The absolute incident-energy envelope
is 1.005; observed values are below 0.986. Specular reciprocity error stays
below 0.0001; the two diffuse factors are symmetric by construction. The
cosine-averaged material response is compared with independent BRDF integration
within 0.01 absolute reflectance. These are scoped tests, not a mathematical
certificate of finite-quadrature passivity for every direction, an integration
convergence proof, or guarantees for narrow HDR peaks/arbitrary bump maps.

The audit also fixes roughness priors leaking through opaque cover and masked
water-side normal derivatives. Both new CPU regressions were observed failing
before their fixes. Review captures and verification are recorded in
[`native-terrain-material-audit-2026-10-20.json`](artifacts/native-terrain-material-audit-2026-10-20.json).
One fresh model reviewer saw unlabeled pairs of an intermediate outgoing-only
version and preferred it in all four cases after votes were frozen and scored.
That preference did not prevent rejection on physical grounds. It is not a
review of the final two-sided closure or general visual acceptance; the
reviewer still noted speckles, coarse silhouettes, stretched detail and blurred
foreground patches. Final capture evidence is distinguished in the artifact.
A second fresh model reviewer assessed the final two-sided captures without
origins or previous reviews and also preferred the correction in all four
cases after votes were frozen and scored (no ties/invalid cases). It still
reported flat/blurry materials, weak atmospheric depth, angular contours and
an artificial foreground boundary. These are two small checks of different
versions, not eight independent votes on the final implementation or a
general visual-quality milestone.

## Numerical and visual acceptance boundary

`render-wgpu/tests/terrain_environment.rs` checks actual primary raster output:

- all three quality levels, with regular and conforming geometry, black
  albedo and independently integrated uniform radiance;
- normal-incidence tolerances of 7 / 4 / 2 RGB8 levels at the pinned
  radiance/exposure, and an 8/255 High grazing envelope at N.V = 0.7/0.2/0.05/0.01;
- directional red/blue fields: rough material broadens the reflection rather
  than substituting an unfiltered mirror;
- same-size field writes reach the terrain without material reupload;
- field removal and the specular switch eliminate the added term;
- reflected-cloud extinction and in-scatter against an independently integrated
  smooth spherical deck with an isotropic blue source and red incident sky.

The existing sky-background depth test explicitly disables material specular
response: a newly published sky is now supposed to change foreground material
lighting, which is not a background depth overwrite. Its original exact
foreground/background and field-removal assertions remain in place.

The grazing oracle directly integrates D*G*F over solid angle; it does not copy
the shader's sampling/PDF code. Its larger tolerance records finite quadrature
error (the initial 5/255 trial was insufficient), not a new material coefficient.
These are fixture-specific absolute bounds, **not** a general error guarantee
for narrow HDR peaks, arbitrary orientations or grazing material-normal detail.
The cloud-deck fixture is not a global transport-error bound for heterogeneous
weather, narrow stellar phase peaks or grazing cloud paths; four directions and
four density steps with one source per segment are an explicit cheaper fallback,
not converged cloud transport. Spatial variation of source/self-shadow inside a
segment remains a known approximation, not claimed equivalent to the full march.
The cost diagnostic counterbalances disabled / Low / Medium / High and reverse
order on the same 1280×720 sphere/sky/material, with 32 measured samples per
configuration. It measures whole classify/raster plus uncovered sky, without
clouds, shadows or direct light, not whole-client performance.

The preceding sky-reflection stage's 2026-10-20 stable-source hardware run on Radeon 880M / Vulkan / RADV Mesa
26.2.4 measured medians of 0.346 ms (specular disabled), 0.867 ms (Low),
1.218 ms (Medium) and 1.639 ms (High) in that isolated fixture. These are costs,
not a speedup claim or a production-frame budget. Uniform-field High central
RGB8 values were 101 / 77 / 31 for roughness 64 / 153 / 255, equal to the
rounded independent reference. Maximum tested grazing deviation was 7/255.
The reflected-cloud fixture produced `[60, 0, 15]` versus the independent
`[63, 0, 15]`, on both regular and conforming terrain.

The normal plane supplies resolved primary-raster grain, but close-up
geometry/erosion convergence, natural ground-cover/vegetation, finite aerial
perspective and primary scene-object reflections remain separate work. The
captures still show smooth macro forms and regular normal-grain ripples; sky
reflection and a normal plane alone cannot make the planet look like
Satisfactory, and no visual acceptance follows from these frames.
Earlier notes described dark silhouette pixels first as background gaps, then
as an empty grazing environment integral that required a sky floor. Both
diagnoses were incomplete: the audit above identifies the unstable visibility
ratio and mismatched diffuse/specular energy partition. The floor is removed.
A corrected site-1 shadow control (`[atmosphere].enabled = true`,
`[shadows].enabled = false`, identical published ambient
`[1112.58, 2051.11, 4132.93]`) changed only 3.21 % of pixels by more than
4/255, with whole-frame medians 89.230 ms (shadows) versus 89.397 ms (no
shadows); this isolates shadowing from the reflection term but is one site,
not a shadow-quality acceptance.
No blind subjective preference, cross-platform acceptance or whole-scene visual
parity follows from these numerical tests or the captured survey frames.

## Algorithm references and next detail work (2026-10-20)

The Minecraft references are algorithm/design comparisons, not dependencies,
copied shaders, assets, or proof of visual parity. Sources were read directly;
video/search-result descriptions were not treated as implementation evidence.

| Reference | Useful mechanism | Thessa decision and boundary |
| --- | --- | --- |
| [TerraForged ErosionFilter](https://github.com/TerraForged/TerraForged/blob/6dd607ebfd41a3b7274e090b1f2cf565d53d1d44/src/main/java/com/terraforged/mod/worldgen/noise/erosion/ErosionFilter.java) and [RiverCarver](https://github.com/TerraForged/TerraForged/blob/6dd607ebfd41a3b7274e090b1f2cf565d53d1d44/src/main/java/com/terraforged/mod/worldgen/noise/continent/river/RiverCarver.java) (MIT) | Seeded droplets, sediment/deposition, and separate bed/bank/valley profiles rather than indiscriminate high-frequency displacement. | Reuse Thessa's existing frozen runoff/deposition and local refinement (docs 50–52), not a second runtime erosion solver. Later rock/soil masks should come from those fields; the mod's dimensionless rates are not SI parameters to transplant. |
| [Tectonic folded erosion density](https://github.com/Apollounknowndev/tectonic/blob/34241bdb35acda67b5367d49f354c66c05e098e2/src/common/main/resources/resourcepacks/tectonic/data/tectonic/worldgen/density_function/noise/continent/erosion_folded.json) | Composable density/noise graphs for art-directed landform families. The inspected `erosion_folded` is an absolute-value noise operation, not a physical erosion solver. | Useful for procedural composition, not evidence of sediment conservation. Do not replace canonical landforms, geology or physical drainage with a function merely named erosion. No code copied; no permissive license was established in this review. |
| [NoCubes technical README](https://github.com/Cadiboo/NoCubes/blob/38904f71b818a1f4770c5253b9a1dd522201babc/README.md) (LGPL-3.0 repository) | Surface Nets for voxel surfaces, with render and collision consumers of the same surface. Chunk meshes rebuild on change. | Retain shared height/CBT geometry and dirty publication; introducing a second voxel world would not fix material detail. Surface Nets may be relevant to future volumetric caves, not the current heightfield. No code or dependency added. |
| [Complementary generated normals](https://github.com/ComplementaryDevelopment/ComplementaryReimagined/blob/c09950df0650b27dac260c1b0f69afc11387aa6f/shaders/lib/materials/materialMethods/generatedNormals.glsl) and [license](https://github.com/ComplementaryDevelopment/ComplementaryReimagined/blob/c09950df0650b27dac260c1b0f69afc11387aa6f/License.txt) | Footprint-aware finite differences avoid taking unresolved offsets from the texture atlas. | Reuse our physical footprint filtering, but do not infer rock relief from albedo brightness: snow/soil colour is not height. Restrictive custom license: no source copied. |
| [LabPBR material standard](https://shaderlabs.org/wiki/LabPBR_Material_Standard) | Separate normal, relief, roughness/reflectance and wetness information instead of treating colour as geometry. | Conceptual reference only, not a LabPBR compatibility claim. Keep existing SI/native material conventions; do not import its DirectX-oriented tangent convention or texture-relative displacement depth as engine semantics. No textures or wiki text copied. |
| [Mikkelsen, Surface Gradient–Based Bump Mapping Framework, JCGT 9(3), 2020](https://jcgt.org/published/0009/03/04/) | Linear composition of height gradients, followed by one normal resolve. | Applied here to mip construction, residency mixing, and tangent-plane projection. Derivation implemented independently; this is the mathematical reference, not a port of a Minecraft shader. |
| [Practical Real-Time Hex-Tiling, JCGT 11(3), 2022](https://jcgt.org/published/0011/03/05/) | Randomized hex-domain texture reuse, structure-preserving blend weights and normal handling. | Candidate for future lawful material exemplars to avoid obvious repeats without a planet-sized texture. Not implemented: current world-addressed procedural pages do not tile an exemplar, so adding three texture reads now would solve the wrong problem. |
| [OpenSimplex2](https://github.com/KdotJPG/OpenSimplex2) (CC0; some marked legacy files use Unlicense) | Lattice/gradient-set symmetry and orientation are explicit noise-quality considerations; 3D uses a rotated BCC lattice. | Candidate against the current gradient-noise baseline if directional artifacts persist. Do not replace the world generator or add a noise dependency merely on the name; first compare isotropy, filtered derivatives and page throughput. Not imported in this change. |
| [Fast Biome Blending, Without Squareness](https://noiseposti.ng/posts/2021-03-13-Fast-Biome-Blending-Without-Squareness.html), KdotJPG, 2021 | Jittered triangular sample points plus normalized compact-support sparse convolution; query a padded chunk once, but never seed the world by that chunk. Skip work when all contributors agree. | Candidate for continuous material/ecotone weights, not a replacement for canonical climate or categorical geology. Keep weights summing to one and source samples world-addressed; a spherical implementation must explicitly handle chart joins. Reuse existing region/scatter descriptors rather than add a second point registry. Not implemented or copied; the article's Java benchmark is not Thessa performance evidence. |

Next substantive appearance work should share the existing geology,
`surface_appearance_filtered` exposure/deposition logic, and
`ecology::vegetation_cover` ground/canopy/reed fields. Distinct substrate relief
and coherent rock/soil boundaries are preferable to another planet-wide noise
gain. Geometry/scatter for plants and boulders must then preserve those source
identities across density/LOD tiers, participate in shared shadows, and avoid
duplicating the existing deterministic scatter descriptors. These are follow-up
tasks, not implemented vegetation, new collision relief, hex-tiling, or POM.

The subsequent [coastal eligibility correction](73_COASTAL_SURFACE_MATERIALS.md)
reuses those sources to stop height-only sand/wet paint from masking inland
lowlands and exposed cliffs. It changes optical coverage, not substrate relief
or vegetation geometry; the maritime source is explicitly regional/coarse.

The subsequent [world scatter foundation](70_WORLD_SCATTER_IDENTITY.md) replaces
the former tile-seeded/density-dependent kernel with fixed world-local candidate
addresses and hash-threshold density subsets. Its existing offline consumer
uses the final frozen global DEM; full native runtime terrain placement and
plant/rock rendering remain separate integration work.

### WORLD09 shared thin-diffuse mesh material (historical layout; WORLD23 above supersedes RT size)

Opaque mesh materials now carry a [0,1] diffuse-transmission partition, default
zero. Total scattering albedo is split as R=A(1-t), T=At; both primary raster
and secondary RT hits use the corresponding front/back cosine. This property
is a material law through the existing surface quality and shadow/RT resolver,
not a plant-only lighting pass. Low retains the phenomenon with its established
cheaper diffuse BRDF and omitted detail maps; Medium/High retain Fresnel energy
partition. The raster material uniform is now 64 B; RT reuses its reserved
factor word and retains 48 B per material. GLB loading retains t=0; importing
external transmission extensions is not implemented. Occluders still cast
opaque shadows, and ambient uses the existing isotropic/AO approximation.
See doc 75 for the optical assets and explicit physical/visual limits.


WORLD09 also separates optional per-vertex ambient visibility from material UVs,
retaining the 32 B vertex ABI and interpolating the additional float stream in
primary and secondary-hit shading. Six CC0 plant maps use shared colour-space
correct mip uploads; absent visibility remains one. Full mip chains include
NPOT edge area, but RT still uses LOD-zero samples without ray footprints and
normal-map variance filtering remains incomplete. See doc 75 for provenance,
source-owned dry-frond material boundaries and numerical limits. Procedural
height-only puddle/darkening/normal paint from the intermediate Gemini branch is
removed: substrate wetness must come from the existing optical material source,
not a new hydrology claim in the general terrain fragment shader.


The 2026-10-24 review also restores the documented first-substrate-bounce law:
direct entry uses `1 - E_l`, aggregate ambient entry uses `1 - E_avg`, and
outgoing transmission uses `1 - E_v`. The intervening uncommitted normalization
`(1 - E_l)/(1 - E_avg)` with unit ambient entry silently changed this model and
failed the existing white/grey furnace and direct-without-sky oracles. It is not
retained as an undocumented internal-bounce compensation. All nine ignored
terrain-environment GPU tests pass after restoration. The common direct GGX
kernel now cancels its vanishing N.V factors algebraically and preserves the
finite grazing limit, without the intermediate artistic `smoothstep` damping.
This does not establish a complete multilayer/multiple-scattering BRDF or a
universal ground-band fix.


### WORLD10 organic floor review (2026-10-24)

Optional ground detail retains Gemini's authored humus, litter, moss, twig,
mineral and root colour/roughness priors. These are optical material mixtures,
not instantiated debris or measured peat spectra. The 0.08 humus multiplier is
a relative gain on the source reflectance; it does not imply 8% reflectance.
Wetland potential is applied beneath the resolved floor rather than covering
its texture again. Rock, saline, snow, frost and water ownership remain.

The review removes the uncalibrated 13.5x normal-gradient amplification:
normal derivatives again follow the source's documented 18 mm relative-height
range in physical metres. Source mips conservatively filter wavelengths shorter
than two destination texels. Twig (0.45 m), pebble (0.18 m) and root (0.22 m)
fields now use their own wavelength cutoff, including the pebble roughness
term; the former 1.1/0.35 m substitutes allowed undersampled detail. Eligibility
transitions are continuous rather than hard comparisons of filtered height.
These changes address physical scale and aliasing; they do not create resolved
rocks, roots, puddles, erosion or a dynamic wetness solver.

### WORLD11a foliage edge calibration (2026-10-24)

The narrower ground-fern source in [doc 75](75_NATIVE_SURFACE_PLANTS.md) exposed
a lit-side near-miss that the actual-producer fixture had incorrectly counted
as interior. Its existing 3% barycentric shadow-edge band now covers misses
outside the blocker as well as hits inside it. Analytic tests demonstrate a
clear f64 ray becoming an edge hit after f32 narrowing. Interior colour tolerance
remains two RGB8 codes, with explicit retained-interior counts and a minimum
half-sample coverage requirement. Production shading and ray exclusion are
unchanged; finite-precision shadow edges remain outside that interior bound.
