# Engine Plume Rendering Architecture

Status: design target for replacing the current `beauty.rs` plume smoke test.

Reference implementation being replaced: commit `33e58cd25abf82c17ffc2467edc0ebed23274dd2` (`feat/atmospheric-beauty`).

## 1. Problem statement

The current engine plume is intentionally cheap and useful as a bring-up path, but it must not become the long-term representation. It is a CPU-baked `32x256` RGBA texture mapped onto one Bevy cone, with fixed sinusoidal Mach-diamond bands, throttle-driven cone scale, two-sine flicker, a hand-tuned emissive color, and one point light. The active craft is located by a concrete entity name and the nozzle offset is hard-coded for the X-15 preview.

That implementation is a valid `Low`/fallback effect. It is not a suitable semantic model for multiple engine types, altitude-dependent expansion, crosswind, condensation, secondary combustion, ground impingement, volumetric lighting, or future ray-traced lighting.

The replacement must be cheap in raster mode today and remain useful when Thessa gains its own ray-query / ray-tracing lighting path later.

The core rule is:

> Rocket exhaust is a compact world-space participating-medium field. Raster meshes, particles, lights, and future ray tracing are consumers of that field, never the source of its shape.

## 2. Goals

The plume system must:

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

Introduce a backend-neutral plume core. Suggested layering:

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

`plume-core` must not expose Bevy, wgpu, or Vulkan types. `bevy-plume` is an integration adapter, not the semantic API.

The existing `apps/client/src/beauty.rs` should eventually contain only plugin composition for gas giants, clouds, aurora, and plume. The four effects should not remain one monolithic renderer module.

## 5. Source data

The current `EnginePlumeInput { throttle, engine_active, sim_time_s }` is too small for the long-term renderer. Replace it with a compiled nozzle/engine description plus dynamic state and a local environment sample.

A representative semantic input is:

```rust
pub struct PlumeSource {
    pub nozzle_to_vehicle: RigidTransform,

    pub exit_radius_m: f32,
    pub mass_flow_kg_s: f32,
    pub exhaust_velocity_mps: f32,

    pub exit_pressure_pa: f32,
    pub exit_temperature_k: f32,
    pub exit_mach: f32,

    pub throttle: f32,
    pub exhaust: ExhaustMaterialId,
}

pub struct PlumeEnvironment {
    pub pressure_pa: f32,
    pub density_kg_m3: f32,
    pub temperature_k: f32,
    pub oxygen_fraction: f32,
    pub flow_velocity_local_mps: [f32; 3],
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

Suggested quality budgets are representation budgets, not different physical models:

- Low: current cone/impostor or analytic 1-2 evaluation approximation;
- Medium: analytic volume, roughly 4-8 medium evaluations per covered pixel;
- High: analytic volume + RCBT residual, roughly 8-12 evaluations with temporal reuse/jitter where useful;
- future Ultra/RT: larger residual budget and ray-traced lighting/transmittance.

Projected size should additionally select cheaper LODs. A sub-pixel plume should collapse to an emissive sprite rather than running a volume pass.

## 9. Shock cells / Mach diamonds

The current fixed five-band sinusoid must remain a fallback artifact only.

The production path should derive shock-cell modulation from exit diameter, exit Mach, and pressure mismatch. It does not need a full CFD solution, but the cell spacing and compression should change with nozzle/environment state.

The shock model should modulate radius, density, temperature, and emission together so that diamonds emerge from the volume rather than from a 2D decal.

The user-facing `mach_diamonds` toggle should not define whether the physics produces shock cells. If kept at all, it is a debug/diagnostic override. Normal quality settings change the fidelity of their representation, not their existence.

## 10. Turbulence and advection

Do not model plume instability as whole-plume brightness flicker. The current two-sine flicker is acceptable for the fallback path only.

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

The existing single nozzle point light is a valid fallback, but lighting proxies should be derived from the plume field rather than from `1800 * throttle`.

Integrate or approximate radiative power along the plume and emit a tiny set of `RadianceProxy` records:

```text
Low:      one nozzle light
Medium:   nozzle + one downstream proxy
High:     a few importance-weighted proxies
RT:       actual volumetric emission/transmittance, proxies optional
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

## 16. Migration plan from `beauty.rs`

Implement in vertical slices so every stage remains usable:

1. Keep the existing cone effect as `PlumeBackend::Impostor` / Low fallback.
2. Move engine/nozzle/render inputs out of `beauty.rs` into `plume-core` semantic types.
3. Remove hard-coded X-15 entity lookup and nozzle position; extract real plume sources from vehicle data.
4. Implement an analytic pressure-aware mean plume and axial profile.
5. Add a custom wgpu volume pass over a coarse plume bound; keep the cone for fallback/parity.
6. Replace fixed Mach-diamond texture bands with the reduced pressure/Mach shock model.
7. Add RCBT binary 3D topology with analytic-only leaves first.
8. Add sparse `4x4x4` residual bricks and conservative node aggregates.
9. Move turbulence/mixing-layer detail into advected residuals; remove whole-plume sine flicker from normal/high paths.
10. Add field-derived lighting proxies.
11. Add condensation/aerosol wake and surface impingement as separate downstream systems.
12. When ray lighting exists, add wgpu ray-query and/or native Vulkan consumers of the same `PlumeField`; do not fork plume semantics.
13. Delete the normal/high `StandardMaterial` cone path after visual/performance parity is demonstrated. Keep the impostor only as the lowest fallback if it remains useful.

## 17. Tests and benchmarks

The new system needs tests that pin semantics rather than screenshots alone:

- deterministic source/profile generation for fixed engine/environment input;
- increasing ambient pressure changes the expansion regime in the expected direction;
- changing exit Mach/diameter changes shock-cell spacing in the expected direction;
- disabled engine/throttle produces zero visible/lighting contribution;
- RCBT residual reconstruction stays within an explicit error bound against a dense/reference sample set;
- node aggregate bounds are conservative;
- topology hysteresis prevents frame-to-frame split/merge thrash;
- portable raster and future RT backend agree on transmittance/emission within a declared tolerance;
- quality tiers preserve the same physical input and differ only in representation error/cost;
- disabled plugin path has no meaningful per-frame GPU work;
- benchmark cost versus projected plume size, active engine count, residual brick count, and wake count.

Visual captures remain useful for regression, but numeric field/transmittance comparisons should be the primary correctness oracle.

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
