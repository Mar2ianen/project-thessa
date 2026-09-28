# 44 — KSA technical comparison notes (2026-09-20)

Status: dated external-reference audit. This document records observations from the
current Kitten Space Agency public pre-alpha and extracts design consequences for
Project Thessa. It is not a parity target and must not turn KSA implementation
details into Thessa architecture requirements.

## 1. Why this comparison is useful

KSA is a useful control sample because it is solving many of the same visible
spaceflight problems with a very different codebase and product scope. Similar
solutions are evidence that a problem is real; they are not, by themselves, a
reason to copy the solution.

The important split for this audit is:

- **observed/verified KSA behavior**: public build, changelog, or documentation;
- **Thessa consequence**: an independent design choice for this repository;
- **not a target**: production breadth, art/content volume, or UI identity.

## 2. Vehicle editor: still a partial surface

The current KSA vehicle editor is visibly much less complete than the eventual
design and its own public feature list still marks the editor/modular-part
system as partial. Recent builds continue to add basic editor behavior such as
connection typing, rotation snapping, local/assembly gizmo frames, and per-stage
performance readouts.

### Thessa consequence

This does not justify building the editor early. It reinforces the current
dependency order in 05_ROADMAP.md: authoritative vehicle semantics, structural
contracts, propulsion data, and control authority should exist before the editor
becomes a large product surface.

The comparison also shows that a sparse editor is compatible with a surprisingly
deep flight/render prototype. Do not use editor completeness as a proxy for
simulation maturity.

## 3. Solar-system scale: real-scale testbed, not the final KSA target

The current KSA public pre-alpha uses a full-size Solar System and therefore
exposes real interplanetary distances and long transfer times directly. KSA's
FAQ also states that this is a development/test environment: its planned
fictional system is intended to be much smaller, roughly between KSP scale and
2-2.5x KSP scale.

### Thessa consequence

Do not shrink the Asterion system merely because another game eventually plans
to. Thessa already treats astronomical scale as a first-class numerical/render
problem:

- SI/f64 authority;
- render-local anchoring;
- large-system ephemerides;
- multiple propagation regimes.

The useful lesson is that full astronomical scale is practical if time
acceleration, render-local coordinates, and object LOD are designed for it.

## 4. Time warp: one user-facing simulation speed, multiple internal regimes

KSA exposes one user-facing simulation-speed ladder rather than presenting
separate KSP-style "physics warp" and "rails warp" modes. The documented preset
range currently runs from 0.1x through 7,776,000x; arbitrary values are also
available through the simspeed command.

The implementation is nevertheless not one uniform integrator. Public
changelogs refer explicitly to physics bubbles, sleeping/floating on-rails
states, high-quality on-rails positions, and special high-warp timestep/collision
handling. Manual engine/RCS control is restricted above 30x, while already
running engines may continue through higher simulation speeds.

### Thessa consequence

Keep the same conceptual separation already emerging in Thessa:

~~~text
one player-visible time scale
        |
        +-- active/high-fidelity integration
        +-- bounded reduced propagation
        +-- on-rails / cached propagation
        +-- sleeping/background objects
~~~

The propagation regime is an implementation detail selected by error bounds and
gameplay requirements, not a separate fiction exposed to the player.

## 5. Planetary rings: rendering can be rich without simulating every rock

KSA rings use a hybrid representation: analytic/2D scattering at distance,
volumetric effects near the camera, and local ring meshes/instances with LOD and
density control. Changelogs show explicit work on ring-mesh VRAM, local
volumetric thickness, self-shadowing, phase functions, and chunk motion.

Collision with ring material is **not** part of the current public feature set;
the public planned-features page lists collisions with ring rocks only as
semi-confirmed.

### Thessa consequence

"Every ring particle is a rigid body" is the wrong cost model. If ring
interaction becomes gameplay-relevant, use a hierarchy such as:

~~~text
large-scale ring field
    density / optical depth / size distribution / velocity distribution
        |
        +-- renderer: analytic + volumetric + procedural local instances
        +-- collision query: swept volume through the statistical field
        +-- local realization: deterministic nearby rocks only when required
        +-- explicit large moonlets/fragments: normal physical bodies
~~~

This makes ring collisions computationally plausible. The gating question is
gameplay value, not whether raw hardware can update billions of colliders.

Candidate gameplay justifications include mining, navigation hazards,
high-velocity damage, occultation/sensor effects, and collection of ring
material. Without one of those loops, collision-capable rings are scope with
little return.

## 6. Unresolved craft must remain visible: mesh -> glint/sprite -> invisible

KSA has a dedicated distant-vehicle path. Public changelogs describe distant
sprites and a DistantGlint model whose strength can depend on vehicle pose and
an automatically generated irregularity curve. That curve is currently derived
from the vehicle physics bounding box sampled over rotation. Atmospheric
transmittance can attenuate the glint.

The June tuning used an end falloff around 20,000 km, with the editor allowing
values up to 50,000 km. Those numbers are KSA tuning constants, not values to
copy.

### Thessa consequence

Thessa needs an explicit radiometric far-object LOD:

~~~text
resolved mesh
    -> unresolved reflected/emissive glint sprite
    -> below visibility threshold
~~~

The transition must not be driven only by geometric angular diameter. A craft
that is much smaller than one pixel can still be visible because reflected or
emitted radiance survives pixel integration.

Candidate inputs:

- projected solid angle / bounding extent;
- Sun-star irradiance at the vehicle;
- material reflectance/specular response;
- vehicle attitude;
- luminous/emissive sources;
- camera exposure and tone mapping;
- atmosphere transmittance and occultation.

Use a conservative brightness bound for culling, then a cheaper pose-dependent
approximation for the actual sprite. Do not render a sub-pixel full mesh simply
to obtain the glint.

## 7. Navball: Apollo-inspired is a style choice, not a flight-state contract

KSA explicitly describes its navball as based on the Apollo attitude indicator.
Its markers still expose the familiar prograde/retrograde, normal/anti-normal,
and radial directions in the active reference frame.

### Thessa consequence

Keep the semantic layer already specified in 10_PILOT_INTERFACE.md and allow
multiple visual/instrument presentations over it.

A future UI may expose selectable presentations such as:

- compact KSP-like ball;
- Apollo/ADI-like instrument;
- aircraft PFD-style attitude presentation;
- minimal vector/tape mode;
- IVA-specific instrument skin.

All of them must consume the same authoritative reference-frame vectors and
attitude state. Switching style must not switch physics semantics.

## 8. Engine plumes: strong reduced-order physics, but the near-field core is finite

KSA's plume work is technically interesting. Public changelogs describe:

- physically computed plume parameters;
- Prandtl-Meyer expansion calculations;
- pressure-ratio-dependent density/angle behavior;
- volumetric exhaust rendering;
- plume shadows;
- separate core and mixing-layer work;
- explicit expansion/diffusion/dissolution for downstream trail sections.

The important limitation visible in the current build is also present in the
implementation notes: the bright volumetric core has an explicit finite support.
KSA added parameters for maximum core/plume length and later a clamp of total
plume length based on gas visibility. This is better than a fixed arbitrary
meter value, but it can still read visually as "the exhaust stops here" when the
cutoff dominates the fade.

The current long-trail system partly solves a different scale: later plume
sections expand, diffuse, dissolve with time, and can drift with pseudo-wind.

### Thessa consequence

Preserve the split already designed in 38_ENGINE_PLUME_RENDERING.md:

~~~text
near-field hot jet / shock structure
        -> mass + energy + species injection
        -> downstream wake / condensate / aerosol field
~~~

There must be **no physical hard end** to exhaust. Numerical/render support may
be finite, but termination should be based on an optical/error criterion:

~~~text
remaining optical depth < epsilon
AND remaining emitted radiance < epsilon
AND downstream wake injection is conserved
    => stop evaluating the hot-core representation
~~~

This distinction matters especially in vacuum. The gas continues ballistically
after it stops being visually useful as a bright volumetric core.

### 8.1 Plume lighting

In the inspected KSA scene the plume itself is visually sophisticated, while
the illumination it contributes to the vehicle/environment is comparatively
weak. This is a visual observation, not a claim about the internal light model.

For Thessa, avoid treating plume lighting as one decorative nozzle point light.
Derive a small set of radiance proxies from the integrated emitting field at
medium quality, and let a future RT path consume the actual emissive medium.

A useful invariant is:

> If the visible plume becomes brighter/larger because the physical field
> changed, its approximate scene-lighting contribution should change for the
> same reason.

## 9. Eclipses are baseline system behavior

KSA visibly supports eclipses and its ring/planet rendering samples celestial
shadows. Thessa already has eclipse geometry in the Asterion/Nereid design and
shared atmosphere lighting inputs.

### Thessa consequence

No architecture change is needed. Keep eclipses as geometry/lighting state,
not scripted events. Continue to reuse the same occultation result for direct
stellar lighting, atmosphere optics, glints, solar power, and eventually
thermal/climate systems where fidelity justifies it.

## 10. Orbital planes and spin axes: Thessa currently has a split authority

KSA's body-rotation configuration distinguishes rotational definition frames
and supports tilt plus azimuth/alignment. Its public changelog explicitly notes
Perifocal and Ecliptic rotation definition frames and a fix to the order in
which axial alignment and tilt are applied.

Thessa already stores orbital inclination and axial_tilt_deg, and the tilt is
used both visually and by J2/C22 gravity. However, the current implementation
has a semantic split:

- BakedEphemeris::body_state() currently returns orientation = DQuat::IDENTITY;
- the client reconstructs visual spin separately in visual_rotation();
- gravity.rs independently reconstructs a harmonic body-fixed frame;
- axial_tilt_rad is currently documented as a tilt relative to the engine
  reference plane rather than unambiguously as obliquity relative to the
  body's own orbital plane.

This is adequate for the current low-inclination design bodies but is not a
general body-orientation model.

### Thessa consequence

Make body orientation authoritative and frame-explicit before rings, richer
surface coordinates, or strongly tilted bodies depend on it.

Suggested semantic inputs:

~~~text
spin_definition_frame = orbital | inertial
obliquity_rad
spin_axis_azimuth_rad
sidereal_period_s
prime_meridian_at_epoch_rad
tidal_lock mode / lock target
~~~

The baker should resolve those into one canonical orientation contract consumed
by every subsystem:

~~~text
body_orientation_at(id, SimTime)
    -> orientation_inertial
    -> angular_velocity_inertial
    -> spin_axis_inertial
~~~

Renderer, C22/J2, terrain coordinates, rings, atmospheres, surface velocities,
and future sensors must not each reconstruct a slightly different body frame.

## 11. What this comparison does and does not imply

The current Thessa prototype has converged unusually quickly on many of the same
architectural problem boundaries visible in KSA: render-local astronomical
coordinates, reduced propagation regimes, volumetric/field-first plume work,
eclipses, orbital tooling, collision separation, and far-object visibility.

That is **prototype-architecture convergence**, not total production parity.
KSA has a mature proprietary rendering/asset/tool pipeline, substantial content,
editor/debug tooling, and years of accumulated edge-case work. Conversely,
Thessa intentionally targets some different/harder simulation choices, notably
summed multi-body vehicle gravity instead of KSA's patched-conic parent-body
gravity.

The useful conclusion is narrower: none of the items above requires a magical
engine feature unavailable to Thessa. The remaining gap is mostly implementation
depth, content/tooling, validation, and polish rather than discovery of an
unknown architecture.

## 12. Action list extracted from the audit

### High priority

1. Unify body spin/orientation into one authoritative frame-explicit function;
   stop reconstructing it independently in render and harmonic gravity.
2. Add a far-object reflected/emissive glint LOD between resolved meshes and
   culling.
3. In the production plume path, make hot-core termination an
   optical/error-bound decision and conserve downstream wake injection.

### Medium priority

4. Derive plume scene-light proxies from integrated radiance rather than only a
   nozzle-local decorative light.
5. Keep navball semantics fixed but make visual instrument presentation
   selectable/data-driven.
6. Reuse eclipse/occultation state across lighting, glints, solar power, and
   later thermal systems.

### Deferred until gameplay proves value

7. Ring-medium collisions using a statistical field + deterministic local
   realization; never one persistent rigid body per ring particle.
8. Rich vehicle-editor product work beyond the minimum needed to exercise
   procedural vehicle semantics.

## 13. External verification references

- KSA FAQ / scale / physics model:
  https://kittenspaceagency.wiki.gg/wiki/Frequently_Asked_Questions
- KSA time warp:
  https://kittenspaceagency.wiki.gg/wiki/Time_warp
- KSA 2026.6.8.4680, physics bubbles/on-rails/high-warp fixes:
  https://kittenspaceagency.wiki.gg/wiki/Version_2026.6.8.4680
- KSA navball:
  https://kittenspaceagency.wiki.gg/wiki/Navball
- KSA rotation frames/tilt/azimuth changes:
  https://kittenspaceagency.wiki.gg/wiki/Version_2025.12.2.2991
- KSA planetary-ring implementation:
  https://kittenspaceagency.wiki.gg/wiki/Version_2026.1.3.3335
  https://kittenspaceagency.wiki.gg/wiki/Version_2026.3.3.3759
- KSA ring-collision status:
  https://kittenspaceagency.wiki.gg/wiki/Planned_features
- KSA glint work:
  https://kittenspaceagency.wiki.gg/wiki/Version_2026.6.6.4601
  https://kittenspaceagency.wiki.gg/wiki/Version_2026.6.7.4631
- KSA plume physical model / length clamp:
  https://kittenspaceagency.wiki.gg/wiki/Version_2026.4.5.3999
- KSA plume trails:
  https://kittenspaceagency.wiki.gg/wiki/Version_2026.7.6.4939/JSON
