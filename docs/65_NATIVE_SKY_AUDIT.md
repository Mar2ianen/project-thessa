# 65 — Native sky migration: pre-implementation audit

**Audit date: 2026-10-18. Status: implementation/acceptance incomplete.**
Scope: sky, background stars, clouds and aurora. The Bevy client remains runnable;
its implementation is evidence of existing mechanisms, not a visual oracle.


## WORLD14 local-weather continuation (2026-10-25)

Doc 77 records a configurable homogeneous spherical local-haze layer shared by
primary terrain/water/meshes through the existing environment publication.
Low uses an isotropic source estimate; Medium/High a directional sky-source
estimate. Clear/default-off and parent-toggle fallbacks are explicit. This
replaces the later unconditional mesh-only fog experiment; it does not complete
the full physical finite-segment atmosphere migration described in this audit.


## WORLD10 shared cloud-density review (2026-10-24)

Native volumetric clouds now upload a body-fixed, periodic 16.384 km R8 density
field through the existing resolved cloud policy. Four continuous periodic
noise octaves are threshold-eroded and squared before filtering; all quality levels retain mips of
one canonical 128-cubed source. The common volume mean normalizes density;
weather alpha modulates column amount and the sine vertical profile remains.
Mean normalization is volumetric before mip quantization; it does not preserve
every individual local column.
This is an authored optical field, not a simulated weather state. Animation
uses simulation-time rotation, shared with the weather phase.

Resolved volumetric Low/Medium/High retain 32/64/128-cubed texture chains
(37,449 / 299,593 / 2,396,745 bytes). Native Low normally resolves to the cheaper
non-volume path; disabled/non-volume modes do not bake or allocate this field.
A one-byte placeholder keeps the bind interface valid. The 176-byte cloud frame
ABI is unchanged; its spare viewport component carries inverse density mean.
Existing weather allocations are separate. Three-dimensional density is sampled
by primary, surface-shadow, terrain-shadow and reflected consumers, with
explicit mip selection from their actual integration-step footprint.

The retained extinction is 0.0006 per metre at unit density. It is derived from
authored liquid mass 0.004 g/m³, 10 µm effective radius, water density 1000 kg/m³
and the geometric-optics relation beta = 3 LWC / (2 rho r). This is a thin-liquid
prior, not a measurement of this planet. The helper rejects non-finite and
invalid inputs. Increasing extinction tenfold was visually rejected; it is
not retained. See the liquid-water optical-depth relation in the
[ECMWF IFS documentation, section 3.7.4](https://www.ecmwf.int/sites/default/files/elibrary/2021/81269-ifs-documentation-cy47r3-part-ii-data-assimilation_1.pdf).

The cloud's diffuse source now integrates extinction along an outward
hemisphere quadrature. Isotropic scattering of uniform external sky over a
2*pi hemisphere contributes one half of that radiance before self-occlusion;
the old unattenuated full-sphere ambient overlit interiors. Existing resolved
sun budgets bound this work (0/2/4 directions and taps). Zero taps retains the
unoccluded half-hemisphere closure. Direct stars retain the existing phase and
Beer-Lambert transport. This is single scattering with an isotropic diffuse
boundary approximation, not a multiple-scattering solution or a complete
angular sky convolution. No ad hoc RGB cloud tint is added.

Constant-density primary/reflected transport and four depth-dependent diffuse
visibility probes have independent numerical oracles. A native ignored GPU
fixture additionally reads the uploaded nonconstant density field and compares
16 rays across four consumers against a fine f64 column integral. See doc 77
for the actual executed results and tolerances. The tested 16 vertical columns
across four consumers have maximum absolute transmittance error 0.001449
against 8,192-step f64 Simpson integration (24 GPU midpoints, limit 0.003);
this is not an all-ray error envelope. Density-field baking has a
separate benchmark; it is startup work, not frame time.

The visual approach follows field/transport separation described in Guerrilla's
[Nubis](https://www.guerrilla-games.com/read/nubis-realtime-volumetric-cloudscapes-in-a-nutshell)
and [Nubis Evolved](https://www.guerrilla-games.com/read/nubis-evolved), with an
independent implementation. Their production-cloud quality is not claimed.
A tested second-order experiment was rejected for quadrature artefacts and
cost; no second-order source or enlarged frame ABI is retained. Flat sky,
finite-distance aerial transport, coupling diffuse surface light to cloud
occlusion, full atmospheric/cloud multiple scattering and cross-platform
visual acceptance remain open. `multiple_scattering` in the review profile is
an existing atmosphere request, not evidence those native features are complete.


## Inventory and reuse decisions

| Effect | Existing source | Native state before this follow-up | Reuse boundary |
|---|---|---|---|
| Scattering, absorption, airglow | `crates/atmosphere/src/{optics,scattering,lights}.rs`; Bevy `apps/client/src/atmosphere.rs` | physical multi-star single-scattering CPU grid in `apps/terrain-native/src/environment.rs`; common background/ocean/RT-miss sampler in `render-wgpu` | extend that field/publication, never introduce a second sky-gradient shader |
| Background stars | `apps/client/src/main.rs::spawn_starfield/update_starfield` | absent | legacy implementation is 500 seeded emissive spheres, camera-following, hidden in survey mode; not a physical catalogue or reusable star renderer |
| Clouds | `crates/atmosphere/src/clouds.rs`; `apps/client/src/beauty.rs`; `assets/shaders/cloud_volume.wesl` | no native cloud consumer | reuse canonical coverage bake, slab intersection/clipping, profiles, phase and transport math; Bevy materials/imports are adapters, not domain state |
| Aurora | `crates/atmosphere/src/{optics,emission}.rs`; `apps/client/src/beauty.rs::bake_aurora` | no native emission consumer | use shared analytic emission and body-fixed magnetic axis; do not copy the GPL beauty bake into an MIT crate |

The existing native environment grid is only 8x4 / 16x8 / 32x16. That is an
indirect-light fallback, not enough angular bandwidth for stellar discs, star
points, cloud detail or auroral curtains. Increasing sample count alone does
not repair spatial undersampling. A separate high-frequency representation may
consume the same semantic source, not invent a different phenomenon.

## Findings before implementation

- **Shared aurora has an altitude/radius bug.** `aurora_emission` accepts a
  planet-centric point but subtracts `altitude_min_m` directly from its radius;
  there is no body-radius input. Its oval test only checks `> 0`, accepting the
  tiny `sin(PI)` roundoff from an out-of-band point. Physical altitude limits
  must be explicit and tests must require real emission and exact zeros outside.
- The Bevy aurora bake uses latitude offsets for colour and a different temporal
  rate from the shared model. Its texture is not the canonical altitude model.
  The shared model's curtain longitude is currently about world Y, not about an
  arbitrary magnetic pole; tilted poles need an explicit magnetic basis.
- Bevy volume-cloud layers are placed at exaggerated radius fractions
  (`1.008..1.014`, `1.018..1.022`), but ground shadow parallax uses **3000 m**.
  This is a known display proxy, not one consistent physical cloud field.
  Native view/shadow/reflection consumers must not repeat that discrepancy.
- Native environment publication ignores cloud and aurora settings. Its refresh
  key only includes camera/source changes, not simulation-time animation.
  Simply appending animated emission without invalidation would freeze it.
- `atmosphere.sky_dome` is resolved but the current native `share_environment`
  enables the background whenever a radiance field is bound. Visibility of the
  sky draw must be distinct from having a field for indirect illumination.
- Native source samples provide physical directions/irradiance/angular radii,
  but do not yet model eclipse occluders. Do not claim eclipses merely because
  `CelestialLight.visibility` exists or an option is requested.
- Background stars have no separate requested/resolved production setting yet.
  Do not hide the new effect behind an environment variable or plugin presence.

## Required acceptance (not implied by this audit)

1. Independent disabled states and explicit cheaper quality/fallback levels in
   requested/resolved graphics settings; disabled means no substantial work or
   retained effect allocations. Report requested vs actually applied modes.
2. One f64 body/inertial frame contract, phase from simulation time, stable
   rebase/warp and magnetic orientation. Stars must not acquire camera parallax.
3. Same cloud/emission state for background, shadows and reflection consumers;
   correct ordering/occlusion from ground, inside clouds, orbit and behind limb.
4. Numeric oracles for Beer-Lambert columns, sample convergence, point/disc flux,
   shell/below-datum rejection, aurora altitude and phase, day/night visibility,
   and disabled resource/work counters. Reuse existing core tests.
5. Deterministic captures with fixed residency, camera, time, exposure and seeds;
   GPU timing/storage budgets for each applied tier; temporal camera/warp replay.
   Independent blind preference review is distinct from physical correctness.

No claim of complete sky/cloud/star/aurora migration or Bevy removal follows
from this inventory. Metal/WebGPU and independent visual acceptance remain open.

## First corrective implementation after the audit

Later surface acceptance exposed false cloud receiver self-occlusion on the
datum sphere. [Doc 74](74_COASTAL_CLOUD_RECEIVERS.md) records the primary-terrain
boundary correction, failed-before planetary-scale regression and remaining
strict secondary/volume semantics; it does not change this audit's original
snapshot or imply complete sky/surface acceptance.

The shared `aurora_emission` API now takes the explicit datum radius separately
from its planet-centric point. Its altitude window returns exact zeros at and
outside the faces; tilted magnetic poles use a magnetic-axis longitude basis.
`aurora_radiance` integrates both visible pieces of the shell, clips them against
the opaque planet and reuses finite-segment atmospheric transport. Colour
amplitudes are volumetric emission (radiance per metre), not surface radiance.
The vacuum-column oracle compares 64 midpoint samples with independent 4096-step
Simpson integration within **0.02%** for its specified fixture. This is not a
general quadrature or angular-interpolation error guarantee.

Native environment publication now consumes this model through the existing
field and common background/ocean/RT-miss sampling bindings. It uses the existing
`upper_atmosphere.aurora`, quality/intensity/animate settings. Low uses 8 emission
samples per visible shell segment and a 64x32 composed grid; High uses 32 and
128x64. Base scattering keeps its original budget and is interpolated onto the
composed grid instead of being re-integrated at emission resolution. No second
sky gradient or separately phased reflection field is introduced.

Emission uses the legacy aligned-dipole **presentation proxy**, converted to
native body +Y: oval latitude 67 degrees, width 3 degrees, 90–250 km altitude,
activity 0.8 and bottom/top RGB scaled by `1e-6 /m` and requested intensity.
These are illustrative emission parameters, not measured Thessa magnetospheric
state or an MHD simulation. Per-body magnetic/solar-wind inputs remain open.
Time is the sampled simulation time, not accumulated wall time. Animated fields
refresh after a 0.5 simulation-second input change subject to the existing frame
cadence; reverse warp also wakes them, while paused/static fields are retained.
That cadence is a bounded visual policy, not a temporal error certificate.

Scattering, airglow and aurora are independently disabled. Disable all three to
release the environment grid and restore isotropic fallback. `sky_dome=false`
now suppresses the actual background consumer without suppressing the shared
indirect field. Tests exercise these independent states and real emission rather
than roundoff. Diagnostics expose applied emission steps, actual grid dimensions,
storage and evaluation latency.

**Still not closed after this first emission slice:** native background stars and clouds (implemented by the follow-up below); high-frequency auroral
curtains, depth-aware emission composition in front of orbit-view terrain,
per-body magnetic input, emissive RT-light proxies (`aurora_lighting`), cloud
shadow/volume transport, temporal/equal-error budgets and independent visual
acceptance. The current environment is a camera-local directional approximation;
secondary-origin parallax/transport is not solved by sharing the field. Do not
label this first emission consumer as complete volumetric sky migration.

Local verification and target-size cost/error evidence are recorded in
[`native-corrective-sky-2026-10-18.json`](artifacts/native-corrective-sky-2026-10-18.json).
In the polar emission-only CPU fixture, Low/High publication p50 was **3.916 /
73.650 ms**. High latency is an explicit performance gap, not an accepted frame
budget. The deterministic native GPU fixture writes disabled/Low/High/disabled
captures and verifies visible emission and exact fallback restoration; this is
not independent aesthetic acceptance.

## Native stars and shared cloud follow-up

`apps/terrain-native/src/sky_layers.rs` is now called by both window and headless
frames, before the shared display transform/HUD. It consumes resolved settings,
not plugin presence. `[atmosphere].stars` independently enables a fixed synthetic
500-star catalogue and up to three actual source discs. The synthetic catalogue
is illustrative, not measured astronomy; actual source direction, angular radius,
colour and irradiance come from the existing celestial sample. Background
directions rotate from inertial to body space with the same f64 frame as the
camera and lighting; they have no camera parallax. Solid-body occultation is
independent of scattering and resident terrain coverage. Low/Medium/High retain
the catalogue/flux and use 8/16/32 shared atmospheric-transmission samples.
Point flux is distributed over a fixed pixel PSF with solid-angle normalization;
resolved discs use source radiance, not a screen-size brightness multiplier.
The source-disc limb currently uses centre-ray atmospheric/body visibility,
not partial-disc eclipse integration. Discs and the distant catalogue share
the `stars` switch; authoritative stellar lighting does not depend on it.

### Seasonal terrain/source regressions (2026-10-20)

`sky_layers::tests::thessa_half_year_keeps_each_star_occlusion_independent_at_both_distance_extrema`
and the ignored hardware
`thessa_seasonal_b_light_reaches_native_terrain_when_a_is_behind_the_body`
use the unmodified `data/system.toml` baked ephemerides. They search the
seasonal Thessa–BC-barycentre distance extrema around epoch zero, including
Thessa's motion around Nereid. A half-year here is half Nereid's year around A,
not half Thessa's 80-hour moon orbit or six Earth calendar months.

Two distinct requested cases are retained: at the farther seasonal extremum,
the star-facing hemisphere sees distant B; roughly half a host year later,
an A-night receiver has A completely behind Thessa but B/C above the horizon.
A control at far-season midnight hides both A and B: maximum distance does not
put the stars on opposite sides of the sky. In this design ephemeris the two
extrema are about 138.34 Earth days apart, distances about 44.456/42.888 AU,
with A–B angular separations about 0.117/178.348 degrees. These are scoped
seasonal extrema over the tested host-year window, not a global extremum of
the 169-year outer binary orbit.

Body transmission is tested without atmosphere and with 8/16/32 transmission
samples through a fixed N2/O2 optical fixture. In the A-hidden case, the
single-scattering sky with all stars equals the B/C-only sky and remains
nonzero; no global A-visibility multiplier may extinguish the other stars.
Actual regular/conforming native terrain is drawn at both epochs. B-only
central linear RGB agrees with independent Lambert evaluation to **2/255**
per channel using the existing fallback albedo and actual inverse-square
stellar irradiance. Manual exposure is identical across both seasons and all
source-subset controls. At A-night, A-only terrain is exactly dark, all-star
terrain equals B/C-only terrain, and reversing source slots preserves it.

These tests intentionally isolate direct diffuse terrain lighting: no clouds,
ambient, specular, city-mask emission or auto exposure. They do not certify
multi-star city gating, Nereid/other-body eclipses, partial stellar discs,
production atmosphere calibration, full-frame visual preference or a rendered
stellar-disc pixel oracle. See
`artifacts/native-terrain-seasons-2026-10-20.json` for executed gates.

Clouds reuse the canonical `bake_clouds` coverage/opacity source and vertical
sine profile, Beer-Lambert law and dual-HG phase contract. Native decks are
3–6 km and, when requested, 9–10 km above the datum; they are explicit
presentation/weather proxies, not yet authoritative meteorology. Every consumer
uses these same radii, extinction, density texture and simulation-time phase.
`render-wgpu::clouds::CloudBinding` publishes one frame/weather binding;
`clouds.wgsl` is the **single** native transport implementation used by:

- the fullscreen primary pass clipped against stored scene depth;
- direct-light visibility for terrain, ocean, opaque meshes and RT hits;
- reflected environment misses, ocean reflections, and finite RT hit segments.

Opaque depth remains authoritative for foreground occlusion. Both hollow-shell
segments and both decks are sorted along the ray, with solid-body termination;
inside-deck and orbital rays do not use the old exaggerated shell radius or
3000-m-only shadow parallax. `cast_shadows=false` skips the direct receiver
queries, without removing cloud view/reflection extinction or self-shadowing.
Each deck's differential phase is independently wrapped in f64 before upload;
pause, reverse warp and a low-deck period crossing do not jump the high deck.
Disabled clouds allocate no weather bake or cloud transport work;
shared receiver layouts retain only constant-size zero-state placeholders. The
primary volume pipeline can still be required by independently enabled aurora.
Disabled stars allocate no catalogue/buffer/pipeline and run no transmission
queries. Disable stars, clouds and aurora and `SkyLayers` has zero effect
storage/work.

Low/non-volumetric cloud fallback uses one midpoint column and no self-shadow
taps. Medium/High use the common resolved view-step budget (currently capped
at 24) and 2/4 self-shadow taps per shell piece. Direct receiver shadows use at
least 8 column samples. Quality changes representation, not deck geometry or
stellar flux; a one-column fallback is not an equal-error certificate. Native
`stochastic` requests report deterministic march fallback in startup diagnostics
and actual capture metadata. No cloud AS or separate generic ray pipeline is
introduced. Clouds consume current-camera-relative origins for secondary rays;
the underlying clear-sky/auroral directional grid still has camera-local
parallax/transport limitations.

Numerical tests execute the real WGSL: point-source integrated flux at 129x65
and 257x129 is within **3%** of the input flux; a uniform-weather cloud column
matches the shared f64 oracle within **0.005 linear display units** (including
8-bit readback). Shared primary/shadow/reflected queries from below, inside,
orbit and the limb match independent f64 shell integration within **0.003
absolute transmittance/radiance units** for the ambient-only fixture. These
are fixture bounds, not guarantees for arbitrary weather, phase or thin details.
GPU tests verify foreground depth, no star parallax, disabled allocations and
portable WGSL validation. Deterministic example captures live under
`apps/terrain-native/target/test-tmp/sky-layers/native-{stars,clouds}.png`.
The ignored target-size cost test measures 1280x720 RGBA16Float GPU intervals
separately from CPU publication and host submit/wait time. It is not game FPS.

Aurora publication now advances optical depth along each shell instead of
re-integrating camera-to-sample transport at every midpoint. Its shared finite
atmospheric segment is explicitly clipped to the optical shell; vacuum tails
do not attenuate light. Vacuum and nearly uniform absorbing-column tests compare
against independent 4096-step Simpson integration within **0.02%** at 64 samples.
The original timing artifact above remains the historical pre-optimization
baseline; new timing is recorded separately, never substituted into that record.

**Remaining acceptance after the cloud follow-up:** depth-aware aurora over foreground orbital terrain (closed by the primary-volume follow-up below),
high-frequency curtains, per-body magnetic/weather state, partial stellar discs
at the limb/eclipses, atmosphere/cloud multiple scattering and cloud diffuse
irradiance coupling, shadow/temporal/equal-error performance tuning, filtered
rough reflection lobes, Metal/WebGPU execution and independent blind visual
review. Native stars/clouds are no longer absent; complete volumetric sky/client
migration is still not claimed.

## Depth-aware primary aurora

Both native frame paths now integrate the shared analytic emission up to the
actual opaque depth, including orbital foreground terrain and objects inside
the emitting shell. `thessa-atmosphere::AURORA_WGSL` mirrors the CPU altitude,
magnetic basis, oval and curtain model; `aurora_phase` supplies common f64
absolute-time activity/phases. The pole has an explicit zero-longitude limit,
avoiding backend-dependent `atan2(0,0)`. The finite CPU query
`aurora_radiance_to` is the independent primary-segment oracle.

The environment producer retains a separate **clear primary background** from
the same scattering/airglow evaluation, while diffuse irradiance and reflected
misses retain the total clear-plus-aurora field. This is a split publication of
one model, not a second sky gradient. The primary volume pass adds emission at
each sample's distance with atmospheric and cloud extinction up to that sample;
it cannot double-add the environment's aurora. A cloud behind an orbital emitter
does not erase it, while a foreground cloud attenuates it. Background suppression
(`sky_dome`) and volumetric emission (`upper_atmosphere.aurora`) are independent.
No atmosphere provider means no invented aurora volume. Disable clouds and
aurora to remove the volume pass/pipeline; disable stars too for zero layer work.

GPU fixtures cover an opaque hit before/inside/after the auroral shell, plus
ground/orbital cloud-emitter ordering. The finite f64 reference agrees within
**0.006 linear display units** including RGBA8/sRGB readback. Conventional
forward-Z depth remains quantized at orbital distances; the oracle uses the
representable depth and actual narrowed projection coefficients. The primary
consumer inverts those coefficients directly, avoiding additional cancellation
from homogeneous inverse-matrix reconstruction. A fixed 33x33 orbital fixture
is saved as `aurora-orbit-depth.png` alongside the other sky-layer captures.
Portable WGSL validation and the target-size diagnostic include the combined
cloud/aurora path and both aurora tiers.

For disjoint altitude bands the cloud column before an auroral shell interval
is constant on that interval. The primary consumer integrates that prefix once
per interval; overlapping bands retain per-emitter transport. This removes
repeated identical density queries without changing geometry, optical law,
sample counts or the shared weather state. The same depth/order oracles exercise
the optimized path. The 1280x720 combined-High diagnostic decreased from about
32.81 to 4.21 ms GPU p50 in the local before/after runs; individual effect rows
use different camera orientations and must not be added as a common frame cost.
These are isolated sky-layer intervals, not game FPS or an accepted total budget.

**Current gaps:** per-body magnetic/weather inputs, richer emission structure,
secondary-origin clear-sky/auroral field parallax, partial discs/eclipses,
atmosphere/cloud multiple scattering and cloud diffuse irradiance coupling,
filtered rough reflection lobes, temporal/equal-error performance tuning,
Metal/WebGPU execution and independent blind visual review. Primary depth-aware
aurora is no longer an absent consumer; full client/platform migration remains
outside this verified effect slice.

The follow-up measurements, numerical bounds and stable-source verification
record are in
[`native-sky-audio-2026-10-19.json`](artifacts/native-sky-audio-2026-10-19.json).
Its verification status distinguishes pending runs from completed evidence.
