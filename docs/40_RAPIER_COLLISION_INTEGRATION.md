# Rapier collision integration

Status: implementation baseline, 2026-09-17.

This document fixes the ownership and frame rules for adding Rapier to the
Project Thessa authoritative simulation. It is intentionally stricter than a
normal game-engine integration: Thessa already owns celestial gravity,
aerodynamics, propulsion, rigid-body state, warp/rails, and the authoritative
120 Hz time lattice. Rapier is introduced as a local contact/constraint solver,
not as a second world simulation.

## 1. Decision

Use `rapier3d-f64` 0.35.x behind the MIT `thessa-collision` crate.

- `thessa-sim-core` owns authoritative physical data and backend-neutral
  collision geometry.
- `thessa-collision` owns transient Rapier pipeline state and the conversion
  boundary.
- game/server crates decide when a body enters or leaves the contact-active
  regime.
- Rapier types, handles, snapshots, and serialization never cross the
  `thessa-collision` public API.
- Rapier global gravity is always zero. Thessa supplies the actual multi-body
  gravitational load as a per-body force.
- dynamic collider density is zero. The mass, centre of mass, and inertia used
  by the solver come from Thessa's authoritative `RigidBodyProperties`.

References for the pinned API:

- <https://docs.rs/rapier3d-f64/0.35.3/rapier3d_f64/>
- <https://rapier.rs/docs/user_guides/rust/getting_started/>

Rapier 0.35 uses `BroadPhaseBvh`; its SIMD contact/constraint processing is on
independently of the `parallel` feature. The `parallel` feature adds Rayon
parallelism to the physics step.

## 2. Why this is not `Rapier owns all flight physics`

The existing flight model already has semantics Rapier does not and should not
replace:

- deterministic baked celestial ephemerides;
- sum-of-fields gravity without SOI switching;
- atmosphere and local panel aerodynamics;
- engine/RCS/control actuator loads;
- the existing long-horizon integrators and vacuum rails path;
- explicit simulation time and the 120 Hz authoritative lattice.

Running Thessa's rigid-body integrator and then running Rapier for the same
body would integrate translation/rotation twice and is invalid. The ownership
switch is therefore per body and per fixed tick:

```text
non-contact-active body
    Thessa force sampling -> existing Thessa rigid-body integrator / rails

contact-active body
    Thessa force sampling -> ExternalWrench -> Rapier dynamics + contacts
                                          -> RigidBodyState readback
```

There is exactly one pose/velocity integrator for a body in any tick.

## 3. State and force mapping

`RigidBodyState` remains the authoritative representation:

- `position_inertial_m`: f64 inertial position;
- `velocity_inertial_mps`: f64 inertial velocity;
- `orientation_body_to_inertial`: body -> inertial quaternion;
- `angular_velocity_body_rps`: angular velocity in body axes.

`RigidBodyProperties` remains authoritative for mass and the full body-frame
inertia tensor.

For a contact-active body the flight force stage is still evaluated by Thessa.
The Rapier wrench is

```text
F_inertial = FlightForces.total_force_inertial_n
           + mass * gravity_acceleration_inertial_mps2

T_inertial = orientation_body_to_inertial
           * FlightForces.total_moment_body_nm
```

Rapier's pipeline gravity is `Vector::ZERO`, and each Rapier body also has
`gravity_scale(0.0)` as a defensive invariant. This prevents accidental
addition of a uniform Earth-like gravity field to Thessa's N-body field.

`FlightForces.total_force_inertial_n` must not be changed to include gravity
just for collision support; gravity stays a separately sampled field in
`sim-core`.

## 4. Local precision frame

Astronomical inertial coordinates must not be handed directly to a local
contact solver. `CollisionWorld` therefore has a Galilean local frame:

```text
CollisionFrame {
    origin_inertial_m,
    origin_velocity_inertial_mps,
    orientation_local_to_inertial,
}
```

The frame orientation is constant while a world is active. Its origin may have
constant velocity. This keeps the local frame inertial: no Coriolis,
centrifugal, or Euler acceleration terms are silently introduced.

Do **not** make the collision frame rotate with the planet merely to make
terrain stationary. A rotating frame requires fictitious forces and would
create a second dynamics model. Production terrain on a rotating body is
instead moving/kinematic geometry whose pose/velocity is derived from the
body ephemeris and body rotation model.

Recentring is allowed, but it must be an explicit rebase preserving every
body's inertial pose and velocity. It is not a physics step.

## 5. Collision geometry ownership

`thessa-sim-core::collision` contains solver-independent physical shapes:

- `CollisionGeometry`;
- `CollisionPart` with a body-local f64 pose;
- `CollisionShape::{Sphere, Cuboid, Capsule}`;
- `CollisionMaterial { friction, restitution }`.

These are asset/compiler data, not Rapier data. The initial primitive set is
small on purpose. A connected craft should normally compile into a modest
compound of convex primitives or convex parts. A dynamic triangle mesh is not
the canonical vehicle representation.

Terrain is the opposite case: world generation owns the surface and streams a
localized triangle/height representation into the collision backend only near
contact-active bodies. The MVP exposes a static trimesh seam for tests and
non-rotating geometry; the production planetary path must add a kinematic
terrain/body seam before it is wired to live surface flight.

No render mesh, Bevy `Mesh`, RCBT GPU page, or Rapier `ColliderHandle` is a
canonical collision asset.

## 6. Contact activation policy

Do not keep the whole star system in Rapier. Contacts are expensive only where
contacts can actually occur, matching `AGENTS.md`.

The runtime should maintain a contact candidate envelope per dynamic cluster.
A body enters contact-active mode when conservative evidence says it can reach
collision geometry within the activation horizon. Inputs can include:

- terrain/obstacle distance certificate from `PlanetField`;
- vehicle bounding radius and speed;
- relative vehicle-vehicle broad-phase distance;
- predicted path over a bounded horizon;
- docking/joint intent.

The activation boundary must have hysteresis. Enter earlier than exit so a
body does not alternate integrators near one threshold.

A safe first implementation is deliberately conservative: if terrain distance
or a vehicle pair is uncertain, stay contact-active. Optimisation comes after
known-case tests and benchmarks.

Any transition into contact-active mode invalidates the current rails coast.
Any structural topology change, staging event, docking/undocking event, or
collision-geometry rebuild also invalidates/rebuilds the corresponding backend
body.

## 7. Fixed-tick phase order

For the current single-authority runtime the intended 120 Hz order is:

1. drain due simulation-time events and inputs;
2. apply guidance/control/actuator state for this tick;
3. sample ephemerides and rotating-body state;
4. classify flight/contact regime and ensure required terrain/vehicle collision
   geometry is resident;
5. evaluate gravity, atmosphere, aero, propulsion, RCS and other external
   physical loads;
6. partition bodies into non-contact and contact-active sets;
7. advance non-contact bodies with the existing Thessa path (or rails where
   eligible);
8. advance each independent contact island/world with Rapier using the sampled
   external wrenches;
9. read Rapier pose/velocity back into `RigidBodyState`;
10. process contact-derived topology/failure events at an explicit boundary;
11. publish authoritative snapshots/telemetry.

The exact fleet scheduler may batch steps differently, but this dependency
order must remain visible. In particular, force sampling cannot read a pose
that has already been advanced by a different solver in the same tick.

## 8. Rayon ownership

`sim-core` and the server already use Rayon/data parallelism. Do not create one
Rapier pool per collision world.

With the `parallel` feature Rapier 0.35 runs parallel work on the Rayon pool
active on the calling thread when no dedicated Rapier pool is configured. The
scheduler may therefore do

```text
sim_pool.install(|| collision_world.step(dt, wrenches))
```

when that is the chosen execution stage.

Do not place a parallel Rapier step inside an already fine-grained
`par_iter()` over individual bodies. Collision constraints couple bodies in an
island and nested oversubscription would be counterproductive. Preferred
parallelism is:

- parallel force/field/aero evaluation over independent bodies;
- barrier;
- parallel independent collision worlds/islands only when coarse enough;
- Rapier internally parallelizes each sufficiently large world;
- barrier/readback.

For tiny scenes the Rayon overhead can exceed the gain. The crate therefore
keeps `parallel` as a feature so benchmarks can compare the same workload with
and without Rapier's internal parallel path.

## 9. CCD and sleeping

Rapier 0.35 automatically performs CCD against fixed geometry for sufficiently
fast dynamic bodies. `RigidBodyBuilder::ccd_enabled(true)` upgrades to full
CCD against kinematic/dynamic bodies as well. The MVP defaults it on because
spacecraft/debris relative speeds make tunnelling unacceptable; benchmarks may
later select it per collision class.

Sleeping is allowed by default. `CollisionWorld::step` clears user force and
torque every tick before applying the current wrench, so stale thrust cannot
survive an omitted command. It only asks Rapier to wake a body when the wrench
materially changes; contacts/kinematic motion remain able to wake it through
the solver.

Do not add artificial damping to make landed bodies settle. Use physical
friction/restitution, correct geometry, solver parameters, and sleep thresholds.

## 10. Determinism and networking

The authoritative server owns the result. Clients do not send solved contact
poses.

`enhanced-determinism` is exposed as a crate feature for experiments and
replay validation, but the network contract must not require byte-identical
Rapier internal state across machines. Authoritative snapshots contain Thessa
state. Reconnecting/reloading reconstructs the local collision backend from
that state and collision assets.

Before client prediction uses the same solver, add a replay test that feeds an
identical input/contact sequence through supported targets and measures the
state divergence envelope.

## 11. Current MVP in this branch

Implemented:

- workspace `thessa-collision` crate using `rapier3d-f64` 0.35.3;
- backend-neutral f64/SI collision primitives in `thessa-sim-core`;
- explicit inertial/local-frame conversion;
- explicit authoritative mass and full inertia mapping;
- zero-density attached colliders;
- explicit external wrench conversion from the existing flight model;
- zero Rapier gravity;
- compound dynamic primitive support;
- static cuboid and localized static terrain-trimesh insertion;
- full-CCD/sleep policy hooks;
- Rapier step + authoritative `RigidBodyState` readback;
- regression test for astronomical-origin state round-trip;
- known-case test for a gravity-loaded sphere settling on a floor at 120 Hz.

Not implemented yet:

- `FlightAuthority` regime switch to the collision path;
- kinematic rotating planetary terrain;
- collision geometry emitted by `vehicle-baker`/vehicle assets;
- contact event -> structural failure/damage mapping;
- joints/docking API;
- terrain collision streaming and eviction;
- multi-vehicle contact activation broad phase;
- benchmark harness and numerical comparison against the old free-flight
  integrator in the no-contact limit;
- collision debug rendering/telemetry.

The absence of the `FlightAuthority` switch is deliberate: wiring static
terrain into live Thessa surface flight would be physically wrong for a rotating
body. Add the kinematic terrain seam first, then make Rapier the sole rigid-body
integrator for contact-active ticks.

## 12. Next implementation slice

The smallest production-shaped continuation is:

1. make vehicle compilation emit a `CollisionGeometry` asset (do not invent
   X-15 dimensions in runtime code merely to exercise Rapier);
2. add a kinematic terrain/body representation to `thessa-collision` whose
   pose at tick `n+1` comes from the canonical ephemeris/body-rotation model;
3. add `ContactRuntime` to `flight-authority` holding the transient
   `CollisionWorld`, stable Thessa body mapping, activation hysteresis and
   terrain patch set;
4. split the current flight step into `evaluate external loads` and `integrate`
   so contact-active mode reuses exactly the same force evaluation without
   calling `integrate_rigid_body_step_soa`;
5. compare free Rapier motion to the existing custom integrator for a vacuum
   force/torque fixture before enabling contacts;
6. add floor/landing and fast-impact regression fixtures;
7. benchmark 1, 8, 64, 256 and 1024 active dynamic bodies, with both crate
   `parallel` settings, before choosing scheduler granularity.

That sequence preserves one physical model, one authoritative tick order, and
one owner of CPU scheduling while still letting Rapier do what it is good at:
contact detection, constraints, CCD, sleeping and rigid-body contact dynamics.
