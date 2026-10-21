# Engine Plume Rendering Architecture

Status: partially implemented. `thessa-plume-core` provides the backend-neutral
analytic profile/optics/medium model. Both clients have a Low-quality impostor
and Medium/High volume consumer. The native consumer reads the actual axial
stations and scene depth; only the legacy client has a field-derived nozzle light.
Wiring live compiled engine/nozzle state into the client, adaptive residual
rendering, and ray-traced lighting remain future work. The rest of this document
records the architecture and acceptance targets for those follow-ups.

The original CPU-baked cone from commit `33e58cd25abf82c17ffc2467edc0ebed23274dd2`
(`feat/atmospheric-beauty`) remains as the Low-quality fallback, not the default
representation for every quality level.

## 1. Problem statement

The original engine-plume implementation was a CPU-baked `32x256` RGBA texture
mapped onto a Bevy cone, with fixed Mach-diamond bands, throttle-driven cone
scale, flicker, emissive color, and a point light. That path remains as the
Low-quality fallback. Medium/High now use the analytic plume profile and a
camera-facing volume ribbon whose shader marches view rays through the plume
cross-section.

The shipped volume path still reads provisional rocket/nozzle values and an
X-15 nozzle offset from the client adapter; it is not yet driven by compiled
vehicle engine data. Crosswind, condensation, secondary combustion, ground
impingement, and ray-traced lighting remain future work.

The long-term architecture must stay cheap in raster mode and remain useful if
Thessa gains its own ray-query/ray-tracing lighting path.

The core rule is:

> Rocket exhaust is a compact world-space participating-medium field. Raster meshes, particles, lights, and future ray tracing are consumers of that field, never the source of its shape.

## 2. Remaining design goals

The fuller plume system should:

- derive shape and optical behavior from engine/nozzle state and the local environment rather than fixed visual dimensions;
- keep the mean supersonic plume analytically cheap;
- spend adaptive storage only on residual structure that the analytic field cannot represent well;
- reuse RCBT semantics for adaptive spatial refinement without coupling plume code to terrain;
- support a portable wgpu raster/compute path;
- leave a clean path to wgpu ray queries and an optional native Vulkan RT backend using the same field representation;
- provide explicit quality tiers through the graphics settings pipeline;
- degrade cleanly to a very cheap impostor;
- cost effectively zero when disabled;
- keep authoritative propulsion and vehicle state independent of Bevy, wgpu, Vulkan, and renderer-only state.

## 3. Non-goals

The first implementation is not a CFD solver. It does not integrate Navier-Stokes in real time and does not attempt combustion chemistry at research-simulation fidelity.

Reduced models are acceptable where they preserve the causal inputs that matter visually. A dimensionless shock-cell approximation calibrated against references is preferable to a magic fixed number of diamonds. A compact optical material table is preferable to a hard-coded orange color.

The plume renderer is not authoritative propulsion physics. Thrust, mass flow, thermal loads, and vehicle dynamics remain owned by simulation systems. The renderer consumes their state.

## 4. Semantic boundary

The backend-neutral plume core is implemented in `crates/plume-core`. It
provides `PlumeSource`, `PlumeEnvironment`, analytic axial profiles, optical
materials, and CPU medium/ray integration. Native source extraction and the
GPL Low optical bake live in `apps/terrain-native/src/plume.rs`; the MIT
GPU transport/integrator lives in `crates/render-wgpu/src/plume.rs` and its
WGSL files. The retained Bevy adapter is `apps/client/src/plume.rs`, with
`assets/shaders/plume_volume.wesl`. This remains a thinner arrangement than the fuller optional
backend split proposed below:

```text
crates/
  plume-core/
      source.rs
      profile.rs
      medium.rs
      cbt_volume.rs
      optics.rs

  plume-wgpu/
      buffers.rs
      prepare.rs
      raster.rs
      shaders/

  plume-vulkan/          # optional, later
      native.rs
      rt.rs

  bevy-plume/
      extract.rs
      node.rs
      plugin.rs
```

`thessa-plume-core` exposes no Bevy, wgpu, or Vulkan types. The proposed
separate `plume-wgpu` / `bevy-plume` crates and optional Vulkan backend have
not been split out; the native GPU consumer belongs to `render-wgpu`.

`apps/client/src/beauty.rs` currently owns gas-giant bands, cloud decks, and
aurora; the plume renderer is already split into `apps/client/src/plume.rs`.
Further effect-local extraction can happen as those renderers grow.

## 5. Source data

The client currently supplies `EnginePlumeInput { throttle, engine_active,
sim_time_s }` and constructs provisional nozzle/environment data in the
renderer adapter. This is sufficient for the shipped visualization, but is too
small to connect actual propulsion hardware. Replace that adapter input with a
compiled nozzle/engine description plus dynamic state and a local environment
sample once client extraction from compiled vehicle/engine state is implemented.

A representative semantic input is:

```rust
pub struct PlumeSource {
    pub nozzle_to_vehicle: RigidTransform,
    pub exit_radius_m: f64,
    pub mass_flow_kg_s: f64,
    pub exhaust_velocity_mps: f64,
    pub exit_pressure_pa: f64,
    pub exit_temperature_k: f64,
    pub exit_mach: f64,
    pub throttle: f64,
    pub exhaust: ExhaustFamily,
}

pub struct PlumeEnvironment {
    pub pressure_pa: f64,
    pub density_kg_m3: f64,
    pub temperature_k: f64,
    pub oxygen_fraction: f64,
    pub flow_velocity_local_mps: [f64; 3],
}
```

The maneuver planner does not need to grow this entire contract. Its current thrust/exhaust-velocity model can remain compact. Nozzle geometry, exhaust optical material, and render-facing state belong in compiled vehicle/engine data and the render extraction path.

No renderer code should locate a vehicle by a hard-coded entity name or assume a fixed nozzle transform. A vehicle may expose zero, one, or many plume sources.

## 6. Analytic mean plume

The mean plume should remain analytic. The renderer should not store a dense volume for the predictable core.

At minimum the mean field is a function of axial distance `z` and radial distance `r`:

```text
R(z)                  mean radius
rho(z, r)             density
T(z, r)               temperature
sigma_t(z, r)         extinction
sigma_s(z, r)         scattering
L_e(z, r)             emission
shock(z, r)           compression/expansion modulation
```

The first-order control parameter is the nozzle-to-ambient pressure ratio:

```text
Pi = p_exit / p_ambient
```

Together with exit Mach and nozzle diameter, it controls whether the plume is overexpanded, near matched, or underexpanded, and therefore the mean envelope and shock-cell strength/spacing.

Do not encode `plume_length`, `plume_width`, or `diamond_count` as primary physical inputs. Quality settings may cap representation distance or sample count, but they must not redefine the physical state.

A practical implementation may precompute a small axial profile when throttle/environment changes materially:

```text
32-64 axial samples:
    radius
    centerline density
    temperature
    extinction
    emission
    shock compression
```

The GPU then performs cheap interpolation plus an analytic radial profile. This gives most of the shape for roughly a kilobyte of profile data per distinct active nozzle state.

## 7. RCBT adaptive residual volume

Do not represent the complete plume as voxels. Represent only the residual between the analytic mean field and higher-frequency structure:

```text
final medium = analytic mean field + adaptive RCBT residual
```

Residual structure includes:

- turbulent mixing-layer deformation;
- crosswind bending and breakup;
- condensation fronts;
- secondary combustion in oxygen-bearing atmospheres;
- plume/surface interaction;
- wake transition regions where a pure analytic core is no longer sufficient.

### 7.1 Use RCBT as a binary spatial tree

Do not force the plume into an octree API. Keep the existing binary-node semantics and define a plume adapter that reconstructs a 3D region from a node path.

A good split policy is longest-axis or error-driven binary subdivision in plume-local space. Long rocket plumes are strongly anisotropic, so binary refinement wastes less space than allocating eight octree children at every refinement step.

Example:

```text
root plume bound
    -> split axial Z
    -> split axial Z
    -> split radial X/Y where required
    -> continue according to error
```

The core RCBT node remains a deterministic binary address. Terrain-specific face/quadtree interpretation stays in the terrain adapter; plume-specific 3D interpretation stays in the plume adapter.

### 7.2 Leaf payload

Start with small residual bricks, preferably `4x4x4` samples. A leaf may have no brick at all when the analytic model is sufficient.

A representative leaf record:

```rust
#[repr(C)]
pub struct MediumLeaf {
    pub brick_index: u32,      // NONE means analytic-only
    pub max_extinction: f16,
    pub max_emission: f16,
    pub residual_error: f16,
    pub flags: u16,
}
```

A compact brick may contain residual density, temperature, composition/condensate, and emission. The exact packing is a measured backend decision.

### 7.3 Hierarchical aggregates

Internal nodes should expose conservative aggregates such as:

```text
max_extinction
max_emission
max_residual_error
```

These values allow both raster and ray paths to skip unimportant subtrees. For a ray segment through a node, a backend may skip the subtree when conservative optical/emission bounds are below the configured error threshold.

The error metric should combine projected optical-depth error, emission variation, density/composition variation, and view/ray importance. Terrain `2:1` balancing is not required for a participating medium because there are no geometric cracks to seal.

### 7.4 Topology cadence

Separate topology updates from payload updates:

```text
RCBT topology      slow / hysteretic
brick payload      fast / GPU-updated
```

Do not split/merge the tree every frame because turbulence evolves. Keep a hysteretic topology and update residual contents inside the current leaves. Rebuild topology only when the error envelope leaves its thresholds.

## 8. Portable raster path

The main wgpu raster backend should use a proxy volume only to generate coverage. The proxy is not the plume geometry and must never become the semantic source.

```text
camera ray
  -> intersect coarse plume bound
  -> traverse analytic field + RCBT residual
  -> integrate participating medium
  -> HDR premultiplied output
```

A short Beer-Lambert integration is sufficient for the first implementation. The pass runs only on pixels covered by the plume bound; it is not a fullscreen raymarch.

The shipped tiers use representation budgets, not different physical models:

- Low: baked cone impostor;
- Medium: field-volume integration, 8 view-ray samples per covered pixel;
- High: the same field-volume integration, 14 samples per covered pixel;
- future tiers: adaptive residual budget and ray-traced lighting/transmittance.

Native coverage is a conservative projected finite-cylinder box, with full-screen
coverage only for near-plane/inside crossings. The legacy consumer uses a
cylindrical billboard. The current Medium/High path does not yet traverse RCBT residuals or use
temporal reuse/jitter. Those remain architecture targets.

Projected size should additionally select cheaper LODs. A sub-pixel plume should collapse to an emissive sprite rather than running a volume pass.

## 9. Shock cells / Mach diamonds

The current fixed five-band sinusoid must remain a fallback artifact only.

The production path should derive shock-cell modulation from exit diameter, exit Mach, and pressure mismatch. It does not need a full CFD solution, but the cell spacing and compression should change with nozzle/environment state.

The shock model should modulate radius, density, temperature, and emission together so that diamonds emerge from the volume rather than from a 2D decal.

The graphics `mach_diamonds` toggle controls only visible shock-cell
modulation; it never changes simulated engine or plume physics. Normal quality
settings change representation cost/fidelity, not physical source state.

## 10. Turbulence and advection

Do not model plume instability as whole-plume brightness flicker. The current
two-sine whole-cone animation is confined to Low. Native Medium/High use
zero-mean advected azimuthal shell modes; the legacy path uses noise. Neither has an
adaptive residual field.

The analytic supersonic core should be comparatively stable. Higher-frequency motion belongs primarily to the mixing layer and residual field. A small deterministic 3D noise field may be advected approximately with exhaust/mixing flow:

```text
p_sample = p_local - flow_direction * advection_speed * time
```

Use a small number of octaves and let the RCBT residual/error policy decide where that structure is worth storing/evaluating.

## 11. Optical material model

Do not hard-code one orange plume palette. Introduce an exhaust optical material description with at least enough information to distinguish:

- hot gas emission;
- soot fraction;
- condensable fraction;
- scattering/extinction behavior;
- possible secondary combustion in oxygen-bearing atmosphere;
- electric/ion beam appearance.

The renderer may use RGB approximations; a spectral renderer is not required. The important property is that color/emission follows exhaust material and temperature rather than throttle alone.

This lets hydrolox, kerolox, solid motors, nuclear thermal exhaust, cold gas, steam, and electric propulsion share the same architecture without sharing the same look.

## 12. Wake, condensation, and surface interaction

Do not extend the hot-core volume indefinitely to represent kilometer-scale exhaust clouds.

Use separate consumers/scales:

```text
hot analytic/RCBT plume
        -> downstream mass/energy injection
        -> coarse aerosol / condensate wake
```

The wake can use GPU splats or a coarse froxel volume at half/quarter resolution. It may advect with atmosphere, expand, cool, condense, and fade on longer timescales.

Ground/surface impingement is also a separate interaction:

```text
PlumeField
  -> surface intersection / footprint
  -> heat + pressure/deposition request
  -> dust / vapor / debris visual injection
```

This keeps the bright supersonic jet, atmospheric cloud, and lifted ground material physically and visually distinct.

## 13. Lighting before full ray tracing

The legacy renderer has one point light at the nozzle for all quality tiers;
its intensity is scaled from the plume profile's integrated radiant power. The
photometric mapping remains provisional. Additional downstream proxies and
ray-traced volume lighting are future work.
The native renderer does **not** apply `engine_plume.light` yet and reports
`nozzle_light=unavailable`. Its existing stellar/RT lighting needs a proper
field-radiance consumer; no second light subsystem is introduced to fake parity.

The long-term lighting architecture may integrate or approximate radiative
power along the plume and emit a small set of `RadianceProxy` records:

```text
Current:  one nozzle point light driven by the field radiant integral
Future:   downstream proxies and/or RT volume lighting
```

This gives a continuous migration path without making a point light part of the semantic representation.

## 14. Future ray-query / native Vulkan path

The field representation must be reusable by future rays.

Do not build a BLAS containing every adaptive leaf. For native Vulkan RT, expose one or a small number of coarse procedural AABBs per plume to the TLAS. Once a ray enters that bound, perform the same RCBT medium traversal used by the portable path.

```text
TLAS
  vehicle triangle BLAS
  static/landmark geometry
  plume procedural AABB
        -> plume-local RCBT traversal
```

The portable backend may use a compute/raster traversal over the same node/brick buffers. A wgpu ray-query backend may use hardware geometry queries for opaque scene hits while still evaluating plume media through the shared field representation.

The target abstraction is therefore not `VkAccelerationStructure` or `wgpu::Tlas`; it is a backend-neutral medium query/integration contract with backend-specific acceleration.

## 15. Graphics settings and quality policy

All plume rendering is controlled through `crates/graphics` and `graphics.toml`. Renderer code must consume resolved settings rather than reading ad-hoc environment variables or hard-coded local quality switches.

Suggested shape:

```toml
[engine_plume]
enabled = true
quality = "medium"

# Optional advanced budgets, normally derived from quality preset:
max_residual_bricks = 2048
max_steps = 8
wake_quality = "medium"
lighting = "proxy"
```

Physical state such as pressure ratio, shock formation, nozzle expansion, condensation eligibility, or plume composition must not be controlled by graphics quality. Quality controls numerical/visual approximation budgets only.

`enabled = false` must remove plume rendering work and side effects. Merely registering the plugin must not switch the global renderer, allocate large persistent resources, add hidden prepasses, or otherwise impose a meaningful frame cost.

## 16. Migration and integration status

Original staged plan (completed items and remaining work):

1. **Done:** retain the cone impostor as the Low fallback.
2. **Partial:** `thessa-plume-core` owns semantic nozzle/environment/profile
   types, while the client adapter still constructs provisional source data.
3. **Partial:** native extraction uses explicit starter identity, not an entity
   name lookup. Remove the legacy lookup and both provisional nozzle offsets;
   extract live plume sources from compiled vehicle/engine state.
4. **Done:** implement the analytic pressure-aware mean plume and axial profile.
5. **Done:** add the Medium/High view-ray volume pass and keep the cone fallback.
6. **Partial:** Medium/High shock modulation uses pressure/Mach-derived
   spacing and amplitude; the Low impostor retains fixed texture bands.
7. Add RCBT binary 3D topology with analytic-only leaves first.
8. Add sparse `4x4x4` residual bricks and conservative node aggregates.
9. Move turbulence/mixing-layer detail into advected residuals; remove whole-plume sine flicker from normal/high paths.
10. **Partial:** derive the existing nozzle point-light intensity from the field
    radiant integral; downstream lighting proxies remain future work.
11. Add condensation/aerosol wake and surface impingement as separate downstream systems.
12. When ray lighting exists, add wgpu ray-query and/or native Vulkan consumers of the same `PlumeField`; do not fork plume semantics.
13. **Done:** Medium/High select the field-volume path; the cone impostor is
    retained only for Low.

## 17. Tests and benchmarks

Current `thessa-plume-core` tests pin deterministic bounded profiles, pressure-
ratio regime changes, shock-spacing response, family-specific optics, invalid
source rejection, zero-throttle behavior, ray integration/transmittance, and
field-derived radiant power. The profile benchmark is run with
`cargo bench -p thessa-plume-core --bench profile`.

Residual/rendering work still needs tests that pin semantics rather than
screenshots alone:

- client shader/CPU-oracle parity for the same field input;
- RCBT residual reconstruction within an explicit error bound against a dense/reference sample set;
- conservative residual-node aggregate bounds and topology hysteresis;
- portable raster and future RT backend agreement on transmittance/emission;
- quality tiers preserve the same physical input and differ only in representation error/cost;
- disabled plugin path has no meaningful per-frame GPU work;
- performance by projected plume size, active engine count, sample budget,
  residual brick count, and wake count.

Visual captures remain useful for regression, but numeric field/transmittance comparisons should be the primary correctness oracle.

### Native consumer verification (2026-10-20)

The GPU tests in `crates/render-wgpu/tests/plume.rs` cover the actual publication:

- 48-station atmospheric/upper-atmosphere/vacuum profiles at 5%, 30% and full
  throttle, 8/14 steps, compared with `integrate_ray(..., 256)`. Shared radiometric
  normalization avoids SDR clipping hiding vacuum errors. The absolute linear-RGB
  envelope is **0.02** for these undeformed, untapered central-chord fixtures,
  not a universal bound on animated/oblique/tail views.
- A constant-radius slice pins RGB and alpha within **0.003**, including RGBA8
  quantization. Exact axial, camera-inside, foreground-occluded and partially
  depth-clipped rays all produce the appropriate finite integral.
- Camera rotation with an unchanged field, 1e12 m translation, immutable profile
  upload reuse, Low texture upload, HDR pipeline creation, integrated scene depth
  storage and disabled resource release have hardware regressions.
- Native producer tests pin pressure-cache response, engine/disable clearing,
  source-identical quality switches, physical nozzle metres independent of mesh
  scale, camera rebasing and the retained Low bake. Actual silent CLI captures
  exercise all tiers and verify identical final authoritative flight telemetry.

### Native work/budget boundary

One explicit starter source is supported, using the provisional 0.55 m /
480 kg/s / 3550 m/s / 120 kPa / 1850 K / Mach 3.5 methalox deck. This is not
calibrated X-15 propulsion or generalized compiled-nozzle extraction. Local
authority and a fresh explicit-starter server replica provide throttle, engine,
time and reference-atmosphere samples. Missing/stale/failed state, arbitrary
retained geometry and map ownership cannot manufacture a plume.

The producer keeps 48 stations and immutable GPU payloads, rebuilding in 2%
throttle / logarithmic approximately 2% pressure bands. The logarithmic pressure
key replaces the retained adapter's saturating `p / (.02*p+50)` key. Rendering
never feeds back into flight or creates a simulation clock.

Low uses the retained 32x256 sRGB bake on a single-wound, 32-sector unit cone;
the game supplies its physical fallback scale. Medium/High interpolate station
radius, extinction, emission and shock directly. A tangent-warped fixed sample
budget resolves thin slices of the long vacuum bound; integration always uses
each bin's physical path length. There is no hard radial skirt clip, averaged
extinction, near-camera opacity fudge or parallel-ray rejection. The integration
bound is 1.6 times the largest published radius. Artistic shell deformation and
last-30%-of-length dissolve remain representation details, not semantic state;
a general full-view error envelope is still future work.

Active frames add one color-load pass, sampling existing stored scene depth
with no copy and no depth writes. Premultiplied emission composites over opaque
terrain/mesh and sky before display mapping. Opaque depth truncates rays even
at hits inside a plume. Disabled settings clear the producer and release the
GPU consumer; absent sources never allocate it. An inactive retained consumer
submits no pass or uploads. There are no timestamp queries outside opt-in
measurement. Resource bounds per allocated consumer are two pipelines,
160-byte camera + 80-byte draw buffers, a 2048-byte station buffer, a 32 KiB
impostor and unit-cone vertices. Counters report submitted draws, march budget
and frame/profile/texture upload bytes, not visible-pixel counts or GPU times.
The headless diagnostic prints them as `native_plume`.
The initial checkpoint used `native-topology-v6-plume`, with plume inside
`render.scene_classify_and_raster`. Current `native-topology-v7-motion`
captures isolate plume plus translucent sky in `render.scene_overlays`, after
the optional opaque-only motion diagnostic (doc 76). Sum opaque raster and
scene overlays before comparing the old combined scope.
Both capture adapters explicitly record native nozzle lighting as unapplied /
unavailable, independently of the shared requested/resolved light toggle.

The CPU extraction diagnostic compares one-source cached frames with forced
48-station rebuilds. A separate 1280x720 one-source GPU fixture uses real command
timestamps (4 warmups, 32 samples, 8/14 steps). Exact commands/results belong in
`artifacts/native-plume-2026-10-20.json`; no whole-client speedup, blind visual
approval, Metal/WebGPU execution or complete Bevy retirement is implied.

## 18. Performance targets

Do not optimize by deleting causal inputs. Optimize representation and work selection.

Important counters should include:

- active plume sources;
- visible plume bounds;
- covered pixels;
- medium evaluations;
- RCBT nodes visited/skipped;
- resident residual bricks;
- topology splits/merges;
- wake particles/froxels;
- proxy lights;
- CPU prepare time;
- GPU plume pass time.

The key scaling target is proportionality to visible optical complexity, not to the volume of the plume bounding box. Empty/analytic regions should be skipped cheaply, and a plume that is off-screen or disabled should cost essentially nothing.

## 19. Architecture invariant

The final architecture should remain:

```text
engine/nozzle state + environment
              -> PlumeSource
              -> analytic mean field
                 + adaptive RCBT residual
              -> PlumeField
                 -> wgpu raster/compute
                 -> cheap lighting proxies
                 -> future wgpu ray queries
                 -> optional native Vulkan RT
```

If a future renderer backend requires changing the physical meaning of `PlumeSource` or duplicating plume-shape logic in a renderer-specific shader, the boundary is wrong.


## 20. External comparison guardrail: finite core is not finite exhaust

The 2026-09-20 KSA audit in
44_KSA_TECHNICAL_COMPARISON_2026_09_20.md found a useful failure mode to
explicitly avoid. KSA's reduced-order plume model is physically informed
(pressure-ratio behavior, Prandtl-Meyer expansion, volumetric rendering, later
trail diffusion), but its bright near-field representation still has an
explicit finite length and a gas-visibility length clamp. In motion this can
read as a geometric end to the exhaust.

For Thessa, finite renderer support must never imply finite physical exhaust.
The hot-core representation may terminate only after its remaining
optical-depth/emission contribution is below a declared error bound **and** its
mass/energy/species contribution has been transferred into the downstream wake
model.

The same audit also reinforces the lighting rule in §13: visible plume radiance
and scene illumination should share a causal source. Medium/high quality should
derive radiance proxies from the field instead of tuning an independent nozzle
light.
