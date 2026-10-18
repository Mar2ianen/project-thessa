# 65 — Native sky migration: pre-implementation audit

**Audit date: 2026-10-18. Status: implementation/acceptance incomplete.**
Scope: sky, background stars, clouds and aurora. The Bevy client remains runnable;
its implementation is evidence of existing mechanisms, not a visual oracle.

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

**Still not closed:** native background stars and clouds; high-frequency auroral
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
