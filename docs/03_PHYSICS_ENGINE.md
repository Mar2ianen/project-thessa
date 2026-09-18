# 03 — Physics engine

## Status

**Implemented numerical prototype.** This document is the current contract for
the paths in `crates/sim-core`, `crates/flight-control`, and
`crates/flight-authority`. Structural fracture, thermal networks, and full
vehicle-system compilation are explicitly future work.

## 3.1. Ownership and units

`thessa-sim-core` is MIT, renderer-independent, and free of Bevy/Tokio/network
dependencies. Authoritative spatial state is `f64` SI data. `SimTime` is the
only simulation-time type; wall-clock values belong to orchestration and
telemetry.

The core exposes pure data and numerical functions. The authority/runtime layer
owns command interpretation, control policy, bake queues, contacts, and
server-facing state.

## 3.2. State and frames

The current state model distinguishes:

- baked celestial body states;
- system-barycentric and body-centered inertial frames;
- body-fixed and local tangent frames;
- vehicle body state (position, velocity, orientation, angular velocity,
  mass/inertia);
- render-local coordinates, which are client-only.

`ReferenceFrame` and `StateVector` preserve frame labels. A render transform or
HUD telemetry value is never authoritative physics state.

## 3.3. Celestial ephemerides

`SystemConfig::bake` reads `data/system.toml` and constructs deterministic
analytic segments for the design system. `BakedEphemeris` returns body states
at a `SimTime` without integrating planet/moon mutual dynamics at runtime.
`thessa-system-baker` serializes a reproducible JSON descriptor.

This is a design-target ephemeris, not a high-fidelity planetary ephemeris.
Periods, phases, and body values are data inputs. Missing phase values are
deterministically resolved by the baker. A future offline pipeline may fit
versioned Chebyshev/Hermite tracks to a more authoritative source.

## 3.4. Gravity

For a test particle:

```text
dx/dt = v
dv/dt = sum_i mu_i * (body_i(t) - x) / |body_i(t) - x|^3
```

All relevant physical sources contribute together. SOI switching and patched
conics are not runtime physics. Synthetic barycenter nodes are coordinate
anchors and do not duplicate their children’s gravitational contribution.

The core provides scalar, ordered batch, Rayon, SIMD-assisted, and hierarchy/
cohort paths. The order of returned targets is preserved and deterministic.

### Gravity hierarchy and cohort patches

`gravity_patch.rs` provides:

- `CohortConfig` with explicit spatial/temporal/error boundaries;
- `GravityPatch` with affine far-field coefficients and exact-near terms;
- `CohortEvaluator` for many target states;
- `affine_segment_bound` for a posted absolute propagation bound;
- deterministic split/fallback behavior when a patch cannot satisfy its bound.

The affine approximation is:


```text
g(x) ≈ g0 + J (x - x0)
```

It is a bounded model reduction, not an invisible physics coefficient. Exact
near terms remain explicit. A caller must fall back to exact evaluation when
the patch is near a body, the budget is exhausted, the source window changes,
or the error bound is not met.

## 3.5. Integrators and coast paths

Implemented paths include:

- adaptive Dormand–Prince 5(4) for general test-particle propagation;
- velocity-Verlet for fixed-step conservative coast checks;
- deterministic bounded substeps for rigid-body flight;
- sampled Verlet/on-rails caches with Hermite position/velocity interpolation;
- piecewise analytic affine propagation via `AffinePropagator`.

`propagate_piecewise` compiles a patch around the current trajectory point,
propagates an analytic frozen-field segment, and rebuilds or stops when the
posted bound expires. It is not permitted to continue silently outside the
bound. Active thrust, atmosphere, contacts, assisted control, and other state-
dependent effects use stepped flight dynamics instead.

The on-rails cache has explicit horizon, sample, impact, atmosphere, obstacle,
and wake semantics. It can be extended or trimmed only after validating the
state and ephemeris identity.

## 3.6. Vehicle and rigid-body dynamics

The current `VehicleDefinition` supports serializable geometry, mass/inertia,
aero panels, control surfaces, and starter propulsion/control channels. The
rigid-body path integrates:

- position and velocity;
- quaternion orientation and angular velocity;
- gravity;
- thrust/external force and moment;
- atmosphere and panel aero;
- control-surface/actuator response;
- contact and terrain boundary checks through the authority adapter.

The current runtime treats one connected vehicle as one rigid body. A complete
structural graph that splits into multiple bodies on failure is not implemented.

## 3.7. Aerodynamics

The panel model evaluates local flow for every panel:

```text
v_local = v_vehicle - v_wind + omega × r_panel
q       = 0.5 * rho * |v_local|²
Re      = rho * |v_local| * chord / dynamic_viscosity
```

Forces are summed in body coordinates and moments use `r × F` around the
center of mass. The implementation includes local AoA/sideslip, finite
planform effects, induced drag, compressibility, smooth stall, transonic and
supersonic corrections, thickness/wave drag, swept-surface normal Mach,
control-surface effectiveness, static `Cm`, dynamic damping, and optional
coefficient tables.

This is a bounded reduced-order model. It is not CFD and does not claim exact
shock positions, separation bubbles, chemistry, boundary-layer transition, or
aeroelasticity. Offline reference solvers can generate validation tables but
are not runtime dependencies. See [`11_AERODYNAMICS.md`](11_AERODYNAMICS.md).

## 3.8. Atmosphere

`AtmosphereConfig` provides deterministic temperature, pressure, density,
viscosity, and speed of sound over its configured layers. It also provides a
rotating-atmosphere velocity boundary and a declared vacuum top.

The current default is ISA-like and is not a final planetary composition
model. A future body-specific model can provide gas composition, `R`, `gamma`,
sea-level state, weather, and altitude-dependent winds without changing the
force/evaluation boundary.

## 3.9. Control, guidance, and actuators

The flight-control crate keeps these layers separate:

```text
pilot / graph input
        ↓
guidance intent
        ↓
aircraft / spacecraft / direct control law
        ↓
flight policy
        ↓
physical control demand
        ↓
allocator
        ↓
actuator dynamics
        ↓
sim-core vehicle state
```

`GuidanceIntent` can represent manual axes, angular rate, attitude, velocity
direction, flight path, or a trajectory plan. A `ControlDemand` contains body
force, body moment, and propulsion demand. Aircraft and spacecraft laws may be
selected or blended by flight condition; neither law directly rotates the
craft. The allocator reports saturation/residuals and actuator dynamics limit
the realized response.

RCS, control surfaces, and propulsion are physical effectors. Policy may limit
or reshape a demand, but it cannot bypass the actuator path.

## 3.10. Contacts and terrain

The authoritative contact backend is `thessa-collision` using
`rapier3d-f64` 0.35 behind `CollisionWorld` (see
[`docs/40_RAPIER_COLLISION_INTEGRATION.md`](40_RAPIER_COLLISION_INTEGRATION.md)).

The runtime supports:

- static colliders: cuboids (pads, test floors, coarse terrain
  proxies) and localized triangle meshes streamed from worldgen;
- kinematic bodies: position-based terrain patches whose pose at
  tick `n+1` is prescribed from the canonical ephemeris and
  body-rotation model — Rapier derives the surface velocity that
  enters contacts, so a landed body rides a moving/rotating body;
- dynamic bodies: rigid-body contact objects with full CCD,
  sleep, and zero-density colliders (mass/inertia come from
  sim-core authority);
- fixed joints: docking/seamless staging connections with
  contacts between joined bodies disabled;
- contact activation hysteresis: a body enters contact-active
  mode at a conservative distance and leaves only past a larger
  threshold; uncertain evidence keeps the body active;
- contact load evidence: `ContactSummary` per pair with normal,
  penetration, and approach speed in the inertial frame, capped at
  64 pairs — loads only, never damage verdicts.

The 120 Hz contact phase follows §7 order: force sampling,
regime classification, Rapier step with external wrenches,
authoritative state readback. Rapier gravity is zero; gravity is
sampled separately by sim-core. Rapier runs on the Rayon pool
when the `parallel` feature is enabled; no dedicated Rapier pool
is created.

Still future work:

- terrain streaming beyond a single-vehicle producer
  (fleet layer with multiple resident patches);
- structural failure mapping onto collision body rebuild
  (requires a structural graph in sim-core);
- wheels, debris bodies, and fluid-surface interactions.

## 3.11. Thermal, structural, and fluid systems

These are architectural requirements, not shipped simulation features yet:

- thermal nodes connected by conductance/radiation edges;
- aerodynamic, engine, and solar heating in the same thermal graph;
- temperature-dependent material strength;
- structural topology changes that update mass, inertia, aero, and thermal
  connectivity;
- fluid/electrical graphs and resource flow.

Do not document them as active runtime behavior until they have state types,
solver paths, known-case tests, regression coverage, and telemetry.

## 3.12. Validation contract

Current tests cover:

- circular and eccentric Kepler cases;
- velocity-Verlet energy bounds and moving-source behavior;
- restricted three-body/Lagrange reference vectors;
- deterministic ordered scalar/Rayon/SIMD batches;
- gravity patch and affine propagation envelopes;
- atmosphere layers, rotating flow, zero flow, and finite derived values;
- aero signs, dynamic pressure, `omega × r`, stall, transonic/supersonic
  branches, coefficient interpolation, and batch equivalence;
- rigid-body forces, attitude, actuator response, and contact stopping;
- on-rails cache reuse, invalidation, impact, wake, extension, and trim.

Reference comparisons live in isolated workspaces:

- `validation/nyx-compare` for orbital propagation diagnostics;
- `validation/aero-compare` for JSBSim/RocketPy and related workflows.

Run the core checks with:

```bash
cargo test -p thessa-sim-core
cargo test -p thessa-flight-authority
cargo bench -p thessa-sim-core --bench gravity
cargo bench -p thessa-sim-core --bench affine_prop
```

## 3.13. Non-goals of the current slice

The current physics slice does not provide final planetary ephemerides, J2/Jn
harmonics, CFD, full aeroelasticity, structural fracture, thermal propagation,
factory/logistics simulation, or production networking. Each of those needs an
explicit state contract, error/validation plan, and benchmark before it should
be called implemented.
