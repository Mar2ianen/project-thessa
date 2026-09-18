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

Implemented (update 2026-09-17, second slice):

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
- known-case test for a gravity-loaded sphere settling on a floor at 120 Hz;
- position-based **kinematic terrain seam** (`insert_kinematic_cuboid`,
  `insert_kinematic_trimesh`, `set_next_kinematic_pose`): the caller
  prescribes the ephemeris/body-rotation-derived pose at tick `n+1` and
  Rapier derives the surface velocity that enters contacts — a landed body
  rides a rising platform in the regression test;
- body/patch **removal** (`remove_dynamic_body`, `remove_static_collider`,
  `remove_kinematic_body`) so topology changes and terrain streaming evict
  stale backend state instead of leaking it;
- `ContactRuntime` + `ContactActivation` in `thessa-flight-authority`:
  transient `CollisionWorld` ownership, stable Thessa body mapping with
  rebuild on geometry change, hysteresis activation (uncertain evidence
  stays contact-active), rising-edge reporting for rails invalidation, and
  a wrench seam (`evaluate_wrench`) that reuses exactly the flight step's
  force evaluation without calling the free-flight integrator;
- compiled `CollisionGeometry` assets: `x15_contact_geometry()` (fuselage
  capsule + wing/tail cuboids at the flown aero stations) ships in every
  `X15StarterProfile`, and `data/vehicles/example_aircraft.toml` carries
  four primitives at its panel stations;
- free-flight parity fixture: a vacuum force/torque load tracks the
  authoritative symplectic-Euler translation inside a 0.10 m / 0.05 m/s
  envelope over 1 s at 120 Hz;
- fast-impact CCD fixture (60 m/s sphere, no tunnelling) and a kinematic
  carry fixture;
- `CollisionDebugSnapshot` telemetry (per-body inertial pose/velocity,
  sleep state, active/touching pair counts, JSON-serializable) plus the
  `contact_debug` probe that lands the X-15 compound on a runway
  (settles belly-down at the 0.65 m fuselage keel, sleeps, one touching
  pair);
- `contacts` bench sweeps 1/8/64/256/1024 active bodies and reports
  throughput plus touching/sleeping counts for both `parallel` settings.

Measured 2026-09-17 (settled sleeping piles, 120 Hz, release):

| bodies | parallel body-steps/s | serial body-steps/s |
| -----: | --------------------: | ------------------: |
|      1 |             1 347 322 |           2 276 081 |
|      8 |             3 285 922 |           4 210 441 |
|     64 |             6 293 599 |           6 775 096 |
|    256 |             5 625 426 |           6 512 705 |
|   1024 |             4 676 810 |           4 116 704 |

Serial wins on settled scenes: Rayon overhead exceeds the gain once bodies
sleep, exactly the §8 caveat. Keep `parallel` switchable and re-measure on
awake/constraint-heavy scenes before choosing scheduler granularity.

Not implemented yet (update 2026-09-17):

- terrain streaming beyond the single-vehicle producer: the 120 Hz loop
  re-poses one kinematic patch from worldgen every tick and evicts on
  regime exit; a fleet layer with multiple resident patches is future
  work (`attach/evict` carry it);
- structural failure mapping: no structural graph exists in sim-core yet,
  so there is nothing to map onto. The seam is ready — see §13;
- per-part wireframe gizmos (craft-anchored patch boxes + body markers +
  normal arrows are in §13 fourth-slice items below and are implemented
  in `apps/client/src/contact_gizmos.rs`).

#### Fourth slice — completed in this branch

The following were listed as future work in earlier drafts and are now
implemented in this branch:

- fixed-joint docking in the backend (`attach_fixed_joint` with local
  port frames, contacts between the joined bodies off) plus
  `ContactRuntime` partners (`sync_partner`, `dock_partner`, `undock`,
  `step_many`) so an upper stage or visitor shares the scene; undock is
  impulse-free and removing a body drops its joints;
- solver-free `ContactBroadPhase` for fleets: conservative
  surface-to-surface distance over a bounded horizon, per-pair
  hysteresis, uncertain evidence stays active;
- same-binary replay gate (identical wrench tape, identical snapshot
  JSON) backing the `enhanced-determinism` story — verified by
  `identical_input_sequences_replay_identically`;
- client contact gizmos (`ContactGizmoPlugin`): craft-anchored patch
  boxes, body markers, and contact normal arrows from the authoritative
  snapshot, drawn only while contact-active;
- `CollisionDebugSnapshot` carries patch boxes (live pose +
  extents, trimesh as AABB) and capped contact summaries.

## 13. Contact load evidence and the damage boundary

`CollisionWorld::contact_summaries` reduces every touching pair to
`(parties, normal, penetration, approach speed)` in the inertial frame,
capped at 64 pairs; `step` records them and `drain_contact_events`
hands them out once per tick. `ContactRuntime` and `FlightAuthority`
delegate the drain. Contact *points* are deliberately omitted: the
pair-local point frame is solver-internal, while normal, penetration,
and approach speed are exact.

The backend reports loads, never damage verdicts. When a structural
graph lands in sim-core, its failure pass consumes the drained summaries
at the explicit topology/failure boundary of the tick order (§7 step 10)
and rebuilds the affected backend bodies — the same rebuild path staging
already uses. No exploding/despawning on damage: AGENTS.md §6 stays in
force.

Wired since the third slice: `FlightAuthority::enable_contact_mode`
arms the switch with an explicit hysteresis boundary. Each 120 Hz `step`
polls terrain evidence (field surface clearance, else datum clearance),
invalidates rails on the rising edge, and integrates contact-active ticks
through `ContactRuntime::step` with the same sampled loads as the powered
step (`powered_step_input` is shared; only the integrator differs). Rails
coast is guarded from both directions, and the free-flight terrain-stop
and datum clamp are bypassed while contact-active (solver bounds and the
inside-planet guard stay). A 25 m kinematic patch is re-posed every tick
from the next-tick ephemeris. Verified by a live landing: belly-down X-15
from 2 m settles at the 0.65 m keel with a touching pair, and free-fall
ticks never touch rails.

Completed fourth-slice items (docking, fleet screen, replay, gizmos) —
see §11 "Fourth slice — completed in this branch":

- fixed-joint docking in the backend (`attach_fixed_joint` with local
  port frames, contacts between the joined bodies off) plus
  `ContactRuntime` partners (`sync_partner`, `dock_partner`, `undock`,
  `step_many`) so an upper stage or visitor shares the scene; undock is
  impulse-free and removing a body drops its joints;
- solver-free `ContactBroadPhase` for fleets: conservative
  surface-to-surface distance over a bounded horizon, per-pair
  hysteresis, uncertain evidence stays active;
- same-binary replay gate (identical wrench tape, identical snapshot
  JSON) backing the `enhanced-determinism` story;
- client contact gizmos (`ContactGizmoPlugin`): craft-anchored patch
  boxes, body markers, and contact normal arrows from the authoritative
  snapshot, drawn only while contact-active;
- `CollisionDebugSnapshot` now carries patch boxes (live pose +
  extents, trimesh as AABB) and the capped contact summaries above.

Two integration traps found while wiring (both covered by regression
tests, both worth re-checking on Rapier upgrades):

- Rapier's `IntegrationParameters` defaults
  `normalized_max_linear_velocity` to 400 m/s. Thessa co-moves at orbital
  velocities (50 km/s here), so the clamp silently rewrote authoritative
  state and tripped the flight solver bounds. The backend sets it to
  `f64::MAX`: no solver-imposed speed limit, tunnelling stays on CCD.
- Kinematic prescriptions are converted with the pre-step frame origin
  but take effect in the post-step one, so a translating collision origin
  stales every prescription by `origin_velocity * dt` (one tick of orbital
  motion: the patch trailed the craft by 424 m with zero contacts).
  Contact mode therefore anchors a fixed-origin inertial frame; f64 needs
  no follow-frame. The constant-velocity origin option stays valid for
  future use but must never carry prescriptions.

## 12. Completed implementation slice

All items from the §11 MVP and §13 fourth slice are now implemented in this branch:

1. ✅ vehicle compilation emits a `CollisionGeometry` asset — `x15_contact_geometry()` in `thessa-sim-core`; `data/vehicles/example_aircraft.toml` carries four primitives at its panel stations;
2. ✅ kinematic terrain/body representation — `insert_kinematic_cuboid`, `insert_kinematic_trimesh`, `set_next_kinematic_pose` in `thessa-collision`;
3. ✅ `ContactRuntime` in `thessa-flight-authority` — transient `CollisionWorld`, stable Thessa body mapping, activation hysteresis, terrain patch set;
4. ✅ flight step split into `evaluate external loads` and `integrate` — `ContactRuntime::evaluate_wrench` reuses the flight step's force evaluation;
5. ✅ free Rapier motion compared to custom integrator — `free_rapier_motion_matches_symplectic_euler_envelope` regression test;
6. ✅ floor/landing and fast-impact regression fixtures — `rapier_resolves_gravity_driven_ground_contact`, `fast_body_does_not_tunnel_through_floor`, `kinematic_terrain_carries_a_landed_body`;
7. ✅ benchmark — `contacts` bench sweeps 1/8/64/256/1024 active bodies with both `parallel` settings.

The production-shaped continuation is now:

