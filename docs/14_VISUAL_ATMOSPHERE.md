# Visual Atmosphere Architecture

Status: design document / implementation target.

Scope: rocky planets and rocky moons first. Gas giants may reuse parts of the same optical model later, but are not a requirement for the first implementation.

The visual-atmosphere subsystem is deliberately separate from climate simulation, cloud weather, aerodynamics, and terrain generation. It consumes physical body/atmosphere data plus current celestial-light geometry and produces rendering inputs.

The core rule is:

> one atmosphere definition, multiple rendering backends.

The game must not have a raster atmosphere and an unrelated ray-traced atmosphere with different physical parameters.

---

## 1. Goals

The visual atmosphere should provide, at minimum:

- physically motivated sky colour;
- horizon haze / aerial perspective;
- limb scattering visible from orbit;
- sunrise/sunset colour shift;
- smooth day/night terminator;
- eclipse darkening;
- multiple stellar light sources with different spectra/colours;
- support for future ray-aware lighting paths;
- optional upper-atmosphere emission: airglow and aurora;
- deterministic behaviour under arbitrary simulation time warp.

The visual result should remain coherent from the surface to orbit without switching to a completely unrelated sky model.

The target is not line-by-line atmospheric spectroscopy or a full radiative-transfer research code. The target is a stable game approximation whose inputs have physical meaning.

---

## 2. Non-goals

The first implementation must not require:

- Navier-Stokes weather simulation;
- a general circulation model;
- per-molecule scattering;
- full spectral path tracing;
- cloud microphysics;
- a coupled climate solver;
- simulation ticks for cloud advection during time warp;
- a separate physical atmosphere state for each graphics preset.

Atmospheric visuals may be approximate while keeping causal relationships correct.

---

## 3. Data ownership

Canonical physical data belongs to celestial-body / atmosphere configuration, not to the renderer.

Examples:

```text
body radius
surface pressure
composition
reference temperature profile / scale parameters
rotation/orientation
host body
stellar sources
```

Visual-only tuning belongs to rendering configuration:

```text
scattering quality
ray-integration steps
LUT resolution
cloud quality
noise/detail quality
aurora rendering quality
ray-tracing participation
```

Changing graphics quality must never change flight physics or climate state.

For Thessa, the current design reference is approximately:

```text
surface pressure: 1.20 bar
N2: 73.5-73.7%
O2: 25.0%
Ar: 1.0-1.2%
CO2: 0.3%
H2O: variable
O3: trace photochemical stratospheric species
reference surface temperature: ~278 K
low gravity: ~0.5 g
```

These numbers are design inputs, not hardcoded renderer constants.

---

## 4. Optical model

The atmosphere should be represented conceptually as an optical medium around a body:

```rust
struct AtmosphereOptics {
    inner_radius_m: f64,
    outer_radius_m: f64,
    rayleigh: ScatteringLayer,
    mie: ScatteringLayer,
    absorption: Vec<AbsorptionLayer>,
    emission: UpperAtmosphereEmission,
}
```

Exact Rust types may differ.

The important operations are conceptually:

```rust
fn transmittance(segment: RaySegment, wavelength_or_rgb: ...) -> ...;
fn sky_radiance(view_ray: Ray, lights: &[CelestialLight], time: SimTime) -> ...;
fn aerial_perspective(segment: RaySegment, lights: &[CelestialLight], time: SimTime) -> ...;
fn upper_atmosphere_emission(view_ray: Ray, time: SimTime) -> ...;
```

These APIs may be implemented through LUTs, analytic approximations, ray marching, or a hybrid. They should not imply brute-force path tracing.

---

## 5. Scattering

The first physically meaningful decomposition should support:

### Rayleigh scattering

Dominant for clear-sky colour and strong wavelength dependence.

Use for:

- blue/cyan daytime sky on Earth-like atmospheres;
- red/orange sunsets;
- distant terrain haze;
- thin orbital limb glow.

### Mie-like scattering

Approximate aerosols/haze with a configurable phase function.

Use for:

- bright horizon haze;
- forward scattering near a star;
- dusty or humid-looking atmospheres;
- world-specific visual identity.

### Absorption

Support one or more broad absorption terms.

For Earth-like rocky worlds this may include an ozone-like absorber. Exact spectroscopy is not required; a compact RGB or low-band approximation is acceptable.

The atmosphere implementation should allow different rocky worlds to obtain distinct sky colours by changing composition/scattering/absorption parameters rather than swapping an arbitrary gradient texture.

---

## 6. Density with altitude

The visual model needs an altitude-dependent density profile. A simple exponential profile is acceptable initially:

```text
rho(h) = rho0 * exp(-h / H)
```

Multiple scale heights may be used for different optical constituents.

The renderer should not assume Earth's ~8 km scale height. Thessa has lower gravity and therefore a substantially more vertically extended atmosphere.

The implementation must be numerically stable for:

- camera below atmosphere;
- camera inside atmosphere;
- camera near the outer optical boundary;
- camera far outside the atmosphere.

---

## 7. Surface-to-orbit continuity

The same atmosphere must produce all of these views:

```text
surface sky
low-altitude haze
high-altitude darkening sky
limb from near orbit
planetary atmospheric ring from far orbit
```

Avoid hard visual mode switches based only on altitude.

A distant/ScaledSpace-like planet representation may use baked approximations for performance, but it should be generated from or tuned to the same optical parameters.

---

## 8. Celestial illumination

Atmospheric rendering must consume a list of celestial light sources rather than assume one Sun.

Conceptual input:

```rust
struct CelestialLight {
    direction: DVec3,
    irradiance_w_m2: f64,
    effective_temperature_k: f64,
    angular_radius_rad: f64,
    visibility: f32,
}
```

`visibility` includes eclipses/occlusion.

For Thessa this matters immediately:

- Asterion A is the main warm K-star illumination source;
- Asterion B is much weaker in total irradiance but hotter/bluer;
- Asterion C is thermally minor but may still be visually present;
- Nereid regularly eclipses Asterion A.

The visual atmosphere should therefore support multi-source twilight and eclipse states from the start.

---

## 9. Eclipses

Eclipses are a geometric illumination effect, not a climate tick.

Given light-source direction/angular size and occluder direction/angular size, derive a continuous source visibility factor:

```text
1.0 = unobscured
0.0 = fully eclipsed
0..1 = partial eclipse / penumbra
```

This factor should affect:

- direct terrain/vehicle lighting;
- atmospheric single-scattering contribution from that star;
- cloud lighting later;
- sky brightness;
- upper-atmosphere emission visibility only indirectly through exposure/contrast.

Do not implement an eclipse by only darkening the surface while leaving a noon-bright sky.

---

## 10. Clouds are a separate subsystem

Clouds are not part of the first atmospheric-scattering implementation.

Future cloud rendering may use the atmosphere's lighting/transmittance queries, but cloud state should remain separate.

The intended cloud model is warp-safe and deterministic:

```text
cloud field = f(body position, altitude, absolute simulation time, climate parameters)
```

not:

```text
advance weather simulation N ticks
```

This allows large time-warp jumps without replaying weather history.

Initial cloud implementation may use a small number of spherical procedural layers rather than full weather dynamics.

For Thessa, likely visual layers include:

- lower/mid-level cloud deck;
- deep convective anvils;
- high thin ice/cirrus-like cloud layer.

Clouds must not be baked permanently into terrain albedo.

---

## 11. Airglow and aurora

Upper-atmosphere emission is separate from scattering.

Conceptually:

```text
Atmosphere
├── scattering
│   ├── Rayleigh
│   ├── Mie-like haze
│   └── absorption
└── emission
    ├── airglow
    └── aurora
```

### Airglow

A weak emissive upper-atmosphere component, mostly important from the night side and limb.

### Aurora

Rocky worlds may optionally define a magnetic field / auroral configuration.

Thessa is a particularly good candidate because it orbits inside the environment of a gas giant. Nereid's magnetosphere can provide a persistent charged-particle environment, while Thessa's own magnetic field can organize precipitation into auroral ovals.

The first aurora implementation does not need magnetohydrodynamics.

Use a deterministic analytic field driven by:

```text
magnetic pole / dipole orientation
oval width
altitude range
activity amplitude
Nereid rotational/orbital phase
absolute simulation time
procedural curtain noise
```

A useful first altitude range is roughly 90-250 km, configurable per body.

Aurora colour may use several broad emissive components, for example green/red oxygen-like emission plus blue-violet nitrogen-like edges. Exact line spectroscopy is not required.

The magnetic axis should be allowed to differ from the spin axis.

---

## 12. Warp-safe time dependence

Every animated atmospheric visual must be evaluable directly from absolute simulation time.

Good:

```text
phase = omega * sim_time
cloud_offset = velocity * sim_time
aurora_activity = analytic_periodic_terms(sim_time) + stateless_noise(sim_time)
```

Bad:

```text
for each physics tick:
    move clouds
    evolve aurora
```

After jumping from T=3 days to T=300 years, the renderer should be able to evaluate the new visual state immediately.

Visual animation does not need bit-identical replay across GPUs, but high-level state and phases should remain deterministic.

---

## 13. Ray-aware architecture

The subsystem should be designed for both raster and ray-aware consumers.

Do not define separate physical atmospheres for each backend.

Preferred conceptual architecture:

```text
AtmosphereOptics
      |
      +--> raster/LUT backend
      |
      +--> ray-aware backend
      |
      +--> future Solari integration
```

A ray-aware path may query atmospheric transmittance along selected segments without ray tracing individual particles.

Examples:

- camera ray through atmosphere;
- shadow/light ray toward a star;
- reflection ray sampling sky radiance;
- atmospheric attenuation between a local light and a receiver;
- plume/aurora emissive proxy interaction later.

The atmosphere itself may still use LUTs and low-step integration internally.

---

## 14. Bevy / Solari integration direction

Bevy is the rendering framework. Domain/physics code must not depend on a particular graphics API.

The initial raster implementation may use Bevy atmosphere/PBR facilities where practical, plus custom shaders/LUTs for missing features.

Solari is an experimental optional high-end path, not the authoritative atmosphere model.

Important design rule:

```text
physical atmosphere parameters
        -> common optical queries
        -> raster backend OR Solari/ray-aware backend
```

If a high-end backend cannot represent an effect directly, a proxy representation is acceptable.

Examples:

- visual aurora: transparent/emissive volume or shell shader;
- ray-traced aurora lighting: optional sparse emissive proxy geometry;
- visual engine plume: custom volume/mesh shader;
- ray-traced plume lighting: separate emissive proxy driven by the same plume state.

The proxy must never become the physical source of truth.

---

## 15. Graphics settings contract

Graphics settings should be represented by TOML first. The future GUI is a typed editor/view of that TOML, not a second independent settings system.

Suggested structure:

```toml
version = 1
preset = "high"

[renderer]
backend = "auto"            # auto | raster | hybrid | raytraced
ray_tracing = "auto"        # off | local | full | auto
resolution_scale = 1.0
hdr = true

[atmosphere]
enabled = true
quality = "high"
aerial_perspective = true
multiple_scattering = true
eclipses = true
multi_star = true
limb_scattering = true
ray_steps = 24

[clouds]
enabled = true
quality = "high"
volumetric = true
ray_steps = 32
shadow_steps = 8
cast_shadows = true
temporal_reprojection = true

[upper_atmosphere]
airglow = true
aurora = true
aurora_quality = "high"
aurora_lighting = true

[raytracing]
enabled = "auto"
terrain = true
vehicles = true
landmarks = true
atmosphere = true
clouds = false
plume_lighting = true
aurora_lighting = true
max_distance_m = 500000.0

[debug]
show_atmosphere_bounds = false
show_rt_proxies = false
show_cloud_bounds = false
```

The schema is illustrative; implementation may refine names.

Presets are only bulk writes/default expansions over the same settings. They are not hidden alternative state.

If a user changes one preset-derived value manually, the UI may display `custom`, but the underlying values remain explicit.

---

## 16. Settings GUI semantics

The future GUI should parse/edit the actual graphics configuration.

It should know metadata for each setting, such as:

```text
path
human-readable label
type
range / enum values
step
storage unit
display unit
advanced/developer visibility
restart requirement
description / tooltip
```

Example:

```text
path: atmosphere.ray_steps
kind: integer
range: 4..128
step: 4
label: Atmosphere ray steps
advanced: true
restart_required: false
```

The GUI should not hide the real configuration model behind opaque `Low/Medium/High` booleans.

Presets remain useful as starting points, but every resolved setting may be inspected/edited.

When editing TOML, prefer preserving comments, unknown future keys, and ordering where practical (for example via a round-trip editing approach rather than rewriting the entire file blindly).

---

## 17. Requested vs resolved graphics settings

Separate user intent from runtime capability.

Conceptually:

```text
graphics.toml
    -> requested GraphicsSettings
    -> preset expansion
    -> hardware/backend capability resolution
    -> ResolvedGraphicsSettings
    -> renderer
```

Example:

```text
requested: ray-aware atmosphere
GPU/backend: no supported RT path
resolved: raster/LUT atmosphere
```

Unsupported high-end features should degrade cleanly rather than fail game startup.

Physics and world state are unaffected by this resolution.

---

## 18. Ray-tracing quality modes

Avoid a single `rt = true/false` switch.

A useful first model is:

### Off

Raster paths only.

### Local

Use ray-traced/high-end lighting where it has the highest local visual value:

- vehicles;
- nearby terrain/landmarks;
- selected reflections/shadows;
- engine plume emissive proxies;
- aurora lighting proxies;
- atmospheric ray-aware queries where supported.

### Full

Enable all supported Solari/ray-aware features within configured budgets.

### Auto

Choose a resolved mode based on backend/GPU support and preset.

This is especially useful for integrated GPUs: they may run selected local RT effects without paying for a full-scene path.

---

## 19. Quality should control budgets, not physics

Examples:

```text
Atmosphere Low:
- smaller LUTs / fewer integration steps
- simpler multiple scattering approximation

Atmosphere High:
- higher LUT resolution
- more integration steps
- better temporal/spatial filtering

Cloud Low:
- 2D/spherical shell approximation

Cloud High:
- volumetric ray march + shadows

Aurora Low:
- emissive textured shell

Aurora High:
- volumetric procedural curtains

Aurora Ultra:
- higher volume quality + optional RT lighting proxy
```

All qualities use the same body, atmosphere, climate, magnetic and celestial-light state.

---

## 20. Suggested runtime separation

A future implementation may use modules roughly like:

```text
visual_environment/
├── atmosphere.rs
├── atmosphere_lut.rs
├── celestial_lighting.rs
├── eclipse.rs
├── clouds.rs
├── upper_atmosphere.rs
├── aurora.rs
├── graphics_settings.rs
├── graphics_capabilities.rs
└── debug.rs
```

This is not a mandated file layout; it documents responsibility boundaries.

Do not put climate/worldgen logic into these modules.

---

## 21. First implementation milestone

The first rocky-atmosphere visual slice is complete when Thessa can show all of the following with no clouds required:

1. surface daytime sky with altitude-dependent scattering;
2. horizon haze / aerial perspective;
3. continuous transition from surface to orbit;
4. orbital atmospheric limb;
5. coloured sunrise/sunset;
6. Asterion-A eclipse by Nereid affecting both surface illumination and sky scattering;
7. secondary-star contribution accepted by the lighting interface;
8. basic airglow or aurora shell can be enabled independently;
9. visual state evaluates directly from current simulation time and geometry;
10. graphics settings can configure the subsystem through TOML.

Clouds, volumetric aurora curtains, and Solari-specific high-end integration may follow incrementally.

---

## 22. Design principle

Prefer this:

```text
physical body/atmosphere
        +
celestial geometry
        +
absolute simulation time
        |
        v
shared visual-environment state
        |
        +--> raster atmosphere
        +--> clouds
        +--> upper-atmosphere emission
        +--> ray-aware/Solari consumers
```

over this:

```text
sky shader with arbitrary colours
cloud simulation tied to frame ticks
separate RT atmosphere parameters
special-case eclipse brightness multiplier
```

The atmosphere should be visually rich, warp-safe, multi-star aware, and ready for hybrid ray tracing without turning Project Thessa into a climate or radiative-transfer simulator.