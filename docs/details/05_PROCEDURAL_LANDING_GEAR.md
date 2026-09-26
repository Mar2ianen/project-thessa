# Procedural landing gear, rover wheels, and tires

Status: partial model/runtime slice. sim-core component laws, vehicle-baker
TOML input and mass/COM baking, sprung/unsprung wheel partitioning, articulated
Rapier wheel bodies/joints, fold-out landing supports, reusable and crushable
shock laws, terrain footpad loads, and powered `FlightAuthority` integration
are implemented. Wheel rolling/braking, airless traction, absorber energy and
permanent-crush regressions pass. Dynamic-body gear contacts, granular soil
response, full hinged-leg inertia/reaction coupling, and broader fleet
validation remain future work.

This specification covers aircraft landing gear and surface-rover running gear
with one reusable parametric model. Aircraft gear may retract and brake for
landing; rover gear may use multiple driven wheel stations and compliant tires
for low-speed traversal. Rocket/lander assets may use splayed fold-out supports
inspired by Falcon-style reusable hardware or Apollo-style impact attenuation.
A lunar rover may use an airless wheel without pretending that the vacuum is
an atmosphere.

## 1. Ownership and scope

The intended boundary is:

```text
vehicle-baker / design asset
    wheel chassis, fold-out supports and component parameters
        ↓ compile
thessa-sim-core
    SI-valued chassis, tire, strut, brake, drive and landing-leg data
    mass/inertia contributions and deterministic shock/actuator/tire laws
        ↓ force, state and constraint commands
thessa-collision / Rapier
    terrain queries, reduced wheel/footpad loads and contact wrenches
        ↓ authoritative state and contact/load telemetry
    flight-authority
     wheel/absorber state, gear command, braking/motor commands and load telemetry
```

`sim-core` must not depend on Rapier. `thessa-collision` remains the only layer
that owns Rapier types and handles. Rapier supplies the authoritative local
constraint/contact integration for the contact-active assembly; Thessa supplies
gravity and authored engine, brake, spring/damper and motor loads. There is one
pose integrator per body per fixed tick, as specified in
[`docs/40_RAPIER_COLLISION_INTEGRATION.md`](../40_RAPIER_COLLISION_INTEGRATION.md).

The first runtime slice is a bounded reduced-order tire model. It is not CFD,
tire finite-element analysis, or a structural fatigue solver. Its parameters
must be calibrated against measured/reference tire load-deflection data and
carry an absolute force/deflection error envelope. Higher-fidelity curves can
replace the reduced law without changing the vehicle authoring boundary.

## 2. Terminology and required assembly data

A **wheel chassis** is one mounted gear assembly: a strut or carrier, a
parametric set of wheels, and optional braking and electric drive equipment.
`wheel_count` is the number of wheel stations owned by that assembly. Multiple
assemblies on the same vehicle form the complete landing gear or rover running
gear.

Each chassis has:

- a stable name, body-frame attachment point and local mounting orientation;
- `length_m`, the longitudinal span available for wheel stations;
- an explicit wheel layout and positive integer `wheel_count`;
- wheel radius, width, mass and spin-axis inertia;
- a tire construction and its measured/equivalent physical properties;
- strut length, usable stroke, spring rate, damping, preload and load limit;
- a selected brake-actuator rating, replicated per braked wheel;
- an optional electric drive motor, reduction and driven-wheel count;
- steering and retraction are separate mechanisms and are outside this first
  wheel-chassis parameter type. Fold-out legs are authored independently as
  `LandingLegSpec` entries, up to 16 per vehicle.

`length_m` and strut length are distinct quantities. Chassis `length_m` controls
wheel placement; `strut.extended_length_m` is the distance from the vehicle
attachment to the wheel axle at full extension. Suspension stroke is measured
along the strut and is not hidden inside either length.

### 2.1 Wheel layouts

The compiler supports layouts with explicit geometric meaning:

- `Inline`: tandem wheels lie on the assembly centerline and are distributed
  evenly over `length_m` (typical multi-wheel aircraft bogie).
- `AxlePairs`: wheels form left/right pairs at each axle station. `wheel_count`
  must be even; axle stations are evenly distributed over `length_m`, and
  `track_width_m` sets the lateral separation (typical rover or main gear).

For `Inline` with `N > 1`, station `i` is
`x_i = -length_m/2 + i*length_m/(N-1)`. For `N = 1`, it is centered at zero.
For `AxlePairs`, the same formula applies to `N/2` axle stations and each gets
`y = ±track_width_m/2`. These placements are deterministic, symmetric, and
independent of render meshes.

## 3. Tire constructions

Every wheel has a geometric tire envelope and a construction. The first
reduced-order contact law uses radial compression and rate:

```text
p_radial        = max(radius - signed_distance_to_surface, 0)
articulated: δ_tire = p_radial at the live wheel-body pose
δ_tire_dot > 0 while the tire carcass is being compressed
N_tire          = clamp(k_radial * δ_tire + c_radial * δ_tire_dot, 0, max_load)
```

The normal load is unilateral: the tire pushes on the ground but cannot pull
the vehicle toward it. `n` is the terrain normal and `s` is the unit strut
axis; `c_strut` is its axial compression. The articulated runtime measures
strut compression and rate from the live wheel-body pose/velocity, evaluates
the strut's spring/damper load separately, and evaluates tire compression from
the geometric radial overlap at that live pose. The strut applies equal-and-
opposite forces to the sprung and unsprung bodies. The legacy reduced query API
instead treats mount-to-terrain travel as a series compliance and splits it
between tire and strut for callers that do not assemble wheel bodies.
Longitudinal and lateral tire forces are reduced-order
slip-velocity responses, projected together onto the Coulomb friction circle:

```text
F_x = -C_x * longitudinal_contact_slip_velocity
F_y = -C_y * lateral_contact_slip_velocity
sqrt(F_x² + F_y²) <= μ_contact * N_tire
```

`μ_contact` is the resolved tire/terrain material pair; the first slice uses
Rapier's default arithmetic-mean friction combine rule. Brake and motor hub
torques are actuator inputs; they are not added again as direct body forces.
Their corresponding wheel-ground force remains limited by `μ_contact * N_tire`.
No constant `braking_bonus` or `traction_multiplier` may bypass that limit.

**Contact ownership is exclusive.** The articulated wheel collider is a
sensor: it is queryable but cannot create solid solver impulses. Rapier ray
queries provide contact geometry against fixed or kinematic terrain; the tire
law applies contact force to the unsprung wheel body and the strut law applies
its separate reaction to both assembly bodies. Ordinary hull/terrain contacts
remain Rapier solid contacts. Wheel forces against dynamic bodies are excluded
until the terrain/body receives the corresponding equal-and-opposite impulse.

### 3.1 Pneumatic tire

A pneumatic tire authors positive absolute inflation pressure, reference
temperature, radial stiffness and damping measured at that pressure, maximum
deflection, maximum load, and carcass/tread material. Inflation pressure is
not ambient pressure: the contact load uses the pressure differential to the
local atmosphere. The first slice treats pressure and reference temperature as
steady parameters; leak, puncture, pressure-controller, gas heating and
pressure-dependent stiffness are explicit later state additions.

Validation checks positive finite pressure and physical dimensions. The tire
remains usable in vacuum only if its sealed internal pressure and temperature
limits are valid; absence of outside gas does not create tire pressure.

### 3.2 Airless tire

An airless tire has no inflation-pressure field. It instead authors a structural
construction (for example spoke or compliant lattice), material/density data,
radial stiffness and damping, allowable load/deflection, and thermal
operating range. The reduced law uses the measured effective stiffness of that
geometry; the construction identity remains available to mass, durability,
render and future thermal/failure systems.

This path is intended to represent lunar and vacuum rover wheels. It must work
with no atmosphere and derive traction from wheel-ground contact, wheel load,
tire construction and terrain material. It may not silently substitute an
inflatable tire, atmospheric drag, or a game traction constant.

## 4. Struts, brakes and drive

### 4.1 Strut

The strut is an articulated sprung/unsprung connection. Parameters include
extended length, stroke, spring rate (`N/m`), damping (`N·s/m`), preload,
minimum and maximum force, and the physical direction in vehicle coordinates.
The axial force is a bounded spring/damper response to measured suspension
compression and velocity. Compression stops are constraints; they are not
large artificial spring coefficients. Rebound, saturation and energy
dissipation are observable in runtime state/telemetry.

The parameter model records wheel/hub/brake masses and spin inertia separately.
The vehicle bake adds the complete installed gear contribution once to vehicle
mass, center of mass and full inertia. At contact-mode entry, the runtime
partitions tire/brake masses into unsprung wheel bodies and retains structural,
strut and drive masses in the sprung body. It subtracts their mass and inertia
from the total vehicle properties, shifts the sprung collision geometry to its
own COM, and reads the total vehicle COM back from the live body states.
Rapier never infers authoritative mass from collider density.

### 4.2 Brake actuator

The brake actuator is selected per chassis and specifies maximum wheel-hub
braking torque (`N·m`), response rate/time constant and installed mass. Its
rating is replicated per braked wheel unless an asset explicitly selects a
shared axle mechanism. The maximum no-slip tire-ground braking force is bounded
by

```text
F_brake <= min(T_brake / r_wheel, μ_contact * N_tire)
```

where `μ_contact` is the tire/terrain friction pair and `N_tire` is the tire
model's unilateral radial reaction. Brake torque acts on wheel spin and is not
clipped at the grip limit: when torque exceeds `μNr`, wheel lock/slip can occur.
At zero normal load there is no friction force even when the brake is commanded.
Regenerative motor braking is reported separately and cannot exceed the
motor's torque/power envelope; resulting tire force still follows the same
slip/friction-circle law.

### 4.3 Optional electric drive

An assembly may omit drive entirely or carry an electric traction motor. The
traction motor reuses the existing `ElectricMotorSpec` torque/power/efficiency
envelope, with a final-drive ratio, drivetrain efficiency and
`driven_wheel_count <= wheel_count`. Motor torque is distributed over driven
wheels by an explicit differential/hub-drive topology. The first simple
topology is equal torque per driven wheel; limited-slip, torque-vectoring and
independent hub motors are later options, not invisible behavior.

The motor output obeys `P = τ ω`, the rated-power and peak-torque limits. The
wheel-ground force follows contact-patch slip and is friction-circle limited;
motor torque itself is not silently clipped, so a wheel can spin when drive
torque exceeds its contact grip. Electrical bus/battery resource accounting is
a separate vehicle system and must eventually constrain motor electrical
power.

### 4.4 Retractable aircraft wheel chassis

`WheelChassisSpec.retraction` is optional. When present, the authored chassis
mount pose and wheel stations are the fully deployed configuration. The
retraction record supplies a body-frame hinge pivot and unit axis, stowed and
deployed angles, initial position, deployment rate, and maximum actuator
torque. When omitted, the wheels remain fixed at their authored mount pose.
`gear_down` commands both retractable wheel chassis and fold-out legs.

The persistent deployment fraction drives the station and axle geometry. In
contact mode, wheel masses and axle inertias follow the hinge, the assembly COM
is recomputed, and the wheel suspension joint frames track the current strut
and axle axes. Terrain, suspension, and wheel-weight moments about the hinge
load the same torque-limited actuator law used by the support legs. Wheel brake
and drive states remain attached to their station indices throughout the fold.
Free-flight actuator motion advances the same state; free-flight inertia and
hinge angular-momentum reaction remain reduced-model approximations.

### 4.5 Fold-out lander supports and shock absorbers

Each `LandingLegSpec` describes a body-frame hinge, stowed/deployed angles, leg
axis and length, installed leg/footpad mass, pad radius/material, and a powered
fold actuator. `gear_down` is the deployment target: `0` is stowed and `1` is
deployed. The no-load angular rate falls linearly with opposing hinge load,
`rate = rated_rate * (1 - resisting_torque / stall_torque)`; a load at the
stall-torque rating holds the current fraction. Loads come from the queried
foot/terrain force moment arm, so the deployment limit is a torque and geometry
interaction rather than a free animation.

Two absorbers use one contact interface:

- `reusable` is a spring/hydraulic-damper unit with recoverable stroke,
  preload, bottom-out stiffness and a maximum axial load. It returns to zero
  compression after contact unloads;
- `crushable` is a one-shot cellular/honeycomb cartridge. Its elastic portion
  rises to a force plateau, then permanent crush advances monotonically and
  remains in `LandingLegState`. After the cartridge reaches its crush capacity,
  remaining travel engages the bottom-out stiffness. Cumulative plastic and
  damping energy are exposed as telemetry; there is no automatic reset.

For compression `δ`, compression rate `δ_dot`, yield displacement
`δ_y = F_plateau/k`, and permanent crush `δ_p`, the crushable cartridge updates
`δ_p_next = max(δ_p, min(δ - δ_y, δ_p_max))` when compressed beyond yield. Its
recoverable spring load is `k * min(max(δ - δ_p_next, 0), δ_y)`; bottom-out and
damper loads are then added and clamped to the authored force rating. Plastic
energy is `F_plateau * (δ_p_next - δ_p)` and damper energy is
`c * δ_dot² * dt`, both non-negative.

The contact query casts along the deployed leg axis, uses the footpad radius to
resolve sphere/terrain overlap, and requires axis/normal alignment of at least
0.1. That conditioning boundary bounds terrain-normal load amplification to
10 times the absorber's axial load. The terrain normal reaction is
`N = F_axial / alignment`; tangential slip response is capped by the arithmetic
mean of pad and terrain friction times `N`. The resulting single wrench is
applied to the vehicle body at the geometric foot contact point. Feet are not
solid Rapier colliders, so the support load is not duplicated by a second
impulse. Fixed and kinematic terrain are supported; dynamic-body foot reactions
and granular sinkage/shear are not.

Leg and footpad masses/inertia enter the sprung vehicle mass bake and common
COM recenter. In this slice their inertia is baked at the fully deployed pose;
fold motion is a torque-limited persistent kinematic coordinate, not a separate
Rapier rigid body. Mass redistribution and the equal/opposite hinge reaction
during fold motion are explicit fidelity work still to do. The footpad force
and absorber energy laws are exact for their stated reduced model; for planar
terrain the ray/sphere overlap is analytic, while the 0.1 alignment gate bounds
load amplification. The collision regression pins compression and normal-load
values against this closed form to floating-point tolerance. Curved-ground
error is controlled by the terrain query's local tangent approximation and has
no global envelope for arbitrary unbounded curvature.

## 5. Rapier wheel queries and fixed-step order

The articulated Rapier runtime creates one sensor-only dynamic wheel body per
station and attaches it to the sprung body with a joint that permits strut
translation and wheel spin. Fold-out supports remain sprung in this reduced
slice and contribute terrain-query contact wrenches at their feet.
FlightAuthority retains wheel-spin/brake, wheel-chassis deployment, and
per-leg deployment/crush state, advances their actuators, applies optional
motor/brake torques as equal-and-opposite couples, and publishes
contact/drive/gear/shock telemetry.
Thessa gravity is distributed by body mass; external vehicle loads are shifted
from total COM to the sprung-body COM before the one Rapier step. Dynamic-body
wheel/foot contacts are not yet included.

The per-tick order is:

1. synchronize the sprung body, unsprung wheel bodies, constraints and terrain;
2. query each wheel against the preceding Rapier broad phase and evaluate
   measured strut/tire loads plus longitudinal/lateral slip forces;
3. advance brake state, evaluate motor torque, and distribute gravity and
   equal-and-opposite wheel actuator/suspension loads plus landing-foot
   absorber/friction loads;
4. step the contact-active Rapier scene once and reconstruct the total vehicle
   COM state;
5. retain wheel spin/brake and leg deployment/crush state and publish wheel
   load/slip/drive and landing shock/actuator telemetry.

Rapier global gravity remains zero. Thessa supplies gravity and other external
loads. No body or wheel is advanced once by a custom rigid-body integrator and
again by Rapier during the same tick. Deployment, contact activation and rails
invalidation follow the existing `ContactRuntime` policy.

## 6. Vehicle-baker TOML shape

`wheel_chassis` is an optional array in the vehicle-baker input. Quaternion
components are authored as `[x, y, z, w]`; inertia matrices are authored by
rows. The following is accepted input:

```toml
[[wheel_chassis]]
name = "left-rover-bogie"
mount_position_body_m = [-0.2, -0.9, -0.35]
mount_orientation_body_xyzw = [0.0, 0.0, 0.0, 1.0]
length_m = 1.8
layout = "inline"
wheel_count = 3
structural_mass_kg = 18.0
structural_inertia_local_kg_m2 = [
  [0.8, 0.0, 0.0],
  [0.0, 1.2, 0.0],
  [0.0, 0.0, 0.9],
]

[wheel_chassis.tire]
construction = { kind = "airless", structure = { spoked = { spoke_count = 24 } }, structure_density_kg_m3 = 4400.0, minimum_temperature_k = 80.0, maximum_temperature_k = 500.0 }
radius_m = 0.32
width_m = 0.18
mass_kg = 3.4
spin_inertia_kg_m2 = 0.11
radial_stiffness_n_m = 42_000.0
radial_damping_n_s_m = 1_100.0
longitudinal_slip_stiffness_n_per_mps = 3_000.0
lateral_slip_stiffness_n_per_mps = 2_400.0
maximum_deflection_m = 0.075
maximum_load_n = 2_400.0
surface_friction = 0.85

[wheel_chassis.strut]
extended_length_m = 0.42
stroke_m = 0.16
spring_rate_n_m = 31_000.0
damping_n_s_m = 2_800.0
preload_n = 80.0
minimum_force_n = 0.0
maximum_force_n = 9_000.0
mass_per_wheel_kg = 0.8

[wheel_chassis.brake]
maximum_torque_nm = 95.0
response_time_s = 0.12
mass_per_wheel_kg = 0.4

[wheel_chassis.retraction]
pivot_position_body_m = [-0.2, -0.9, 0.1]
hinge_axis_body = [0.0, 1.0, 0.0]
stowed_angle_rad = -1.5707963267948966
deployed_angle_rad = 0.0
initially_deployed = true
deployment_rate_rad_s = 0.8
actuator_max_torque_nm = 12_000.0
```

An `axle_pairs` layout uses the externally tagged TOML value
`{ axle_pairs = { track_width_m = 0.55 } }`. An optional `[wheel_chassis.drive]`
table accepts the motor envelope, stall copper loss, rotor inertia, final-drive
ratio, drivetrain efficiency and driven-wheel count. Omitted `wheel_chassis`
tables keep existing vehicle files valid. The baker includes wheel-chassis
mass and inertia before its one assembly COM shift, then shifts/recompiles the
mount frames and optional retraction pivots with the rest of the vehicle.

The model compiler rejects invalid dimensions, wheel counts/layouts,
pressure/construction mismatches, impossible actuator ratings, bad drive
ratios, non-finite values, and a drive count exceeding the wheel count.

Fold-out supports are authored in the same file. Axes and mount points are
body-frame vectors; `gear_down` moves each leg between its stowed and deployed
angles. Reusable hardware:

```toml
[[landing_legs]]
name = "forward-leg"
mount_position_body_m = [1.2, -0.8, -0.5]
hinge_axis_body = [0.0, 1.0, 0.0]
stowed_leg_axis_body = [0.0, 0.0, 1.0]
stowed_angle_rad = 0.0
deployed_angle_rad = 2.5
initially_deployed = false
deployment_rate_rad_s = 0.6
actuator_max_torque_nm = 18000.0
leg_length_m = 2.5
leg_mass_kg = 24.0
footpad_radius_m = 0.25
footpad_mass_kg = 4.0
footpad_friction = 0.75
footpad_slip_stiffness_n_per_mps = 6000.0
shock_absorber = { kind = "reusable", stroke_m = 0.28, spring_rate_n_m = 65000.0, damping_n_s_m = 8500.0, preload_n = 0.0, bottom_out_stiffness_n_m = 350000.0, maximum_force_n = 180000.0 }
```

For a one-shot Apollo-style crush cartridge, replace the shock value with:

```toml
shock_absorber = { kind = "crushable", elastic_stiffness_n_m = 120000.0, damping_n_s_m = 7000.0, plateau_force_n = 30000.0, maximum_crush_m = 0.35, bottom_out_stiffness_n_m = 500000.0, maximum_force_n = 220000.0 }
```

`data/vehicles/example_body.toml` contains a four-leg splayed reusable lander
configuration. The baker adds its leg masses before the common COM shift, then
shifts and recompiles each hinge frame with the rest of the vehicle.

## 7. Acceptance tests

`thessa-sim-core` known-case and regression tests:

1. **Geometry:** a three-wheel inline chassis of length `2 m` compiles stations
   at `[-1, 0, +1] m` from its mount; a single wheel is centered; two axle pairs
   are mirrored about the assembly centerline and mount.
2. **Tire force:** with `k=200,000 N/m`, `c=10,000 N·s/m`, compression
   `0.02 m`, and compression rate `0.1 m/s`, normal load is `5,000 N` within
   floating-point tolerance. Sufficient rebound never creates tensile load;
   stroke/load clamps are exact.
3. **Construction validation:** pneumatic tire rejects zero/negative/non-finite
   absolute pressure; airless tire has no pressure requirement and compiles in
   vacuum. Both reject non-positive dimensions and stiffness.
4. **Brake/traction:** with `r=0.25 m`, `T_max=500 N·m`, `μ=0.8`, and
   `N=1,000 N`, brake force is capped at `800 N`; at `N=4,000 N` it is capped
   at `2,000 N`; with `N=0` it is zero.
5. **Tire grip:** longitudinal/lateral forces oppose contact-patch slip and
   their combined magnitude never exceeds `μN`; zero normal load gives zero
   tangent force.
6. **Motor envelope:** torque follows the reused motor peak-torque/constant-
   power curve, gear ratio and efficiency; a four-wheel drive divides the
   requested axle torque without creating energy; contact forces obey `μN`,
   while excess hub torque produces wheel slip rather than implicit traction
   control; speed/power caps are pinned.
7. **Thermodynamic independence:** airless-wheel normal force and rolling
   contact are finite and unchanged by atmosphere being absent; no hidden
   vehicle gravity or tire-pressure fallback is used.

`thessa-collision`/authority integration tests:

8. **Static load (implemented reduced slice):** a one-wheel vehicle settles on
     a flat floor using the Rapier ray-query/tire wrench path; normal reaction
     balances Thessa-supplied gravity within an absolute `0.5%` of vehicle
     weight, with finite unsaturated penetration.
9. **Moving terrain and rolling sign (implemented):** a translating kinematic
    patch contributes its surface velocity to contact-patch slip, and the
    authored positive wheel-spin rate cancels the matching forward hub speed.
10. **Rolling/braking (implemented articulated slice):** a driven wheel
     accelerates a sprung/unsprung vehicle on a plane; the brake actuator reduces
     spin, and tire forces obey the `μN` friction circle.
11. **Airless/regolith proxy (implemented bounded slice):** an airless wheel
     drives against a low-friction terrain material with no atmospheric force
     path; tire/terrain friction is resolved and remains inside the Coulomb
     limit. Granular sinkage is not represented.
12. **Suspension energy:** a known compression/rebound sequence matches the
     spring stored-energy change and dissipates non-negative damper energy; hard
     stroke stops do not inject energy.
13. **Determinism and ownership:** repeated fixed-step command/contact tapes
      yield bounded replay error; there is one Rapier integration per contact
      tick, no duplicate Thessa rigid-body integration, and no hidden gravity.
14. **Reusable vs one-shot absorber:** reusable travel returns after rebound;
       crushable travel and absorbed energy persist monotonically, reach the
       bottom-out law at capacity, and never generate a tensile load.
15. **Fold actuator:** deployment/retraction moves at rated no-load speed,
      slows with resisting hinge torque, and stalls at rated torque. Gear
      transitions prevent rails batching until the requested configuration is
      reached.
16. **Footpad load:** a flat-plane three-leg lander reports analytic
      sphere-pad compression, friction-limited tangent force, permanent crush
      and finite vehicle wrench through contact-active authority ticks.

The `thessa-collision` release benchmark measures warm-broad-phase wheel
queries alone, articulated wheel assemblies (sensor wheel bodies, slider/spin
joints, gravity, tire/strut wrench evaluation and one Rapier step), and landing
support queries with shock/friction state and applied loads. Wheel query-only
throughput was 3.67–4.50 million wheel-steps/s across serial/parallel runs.
The latest articulated-wheel results measured:

| wheels | serial wheel-steps/s | parallel wheel-steps/s |
| -----: | -------------------: | ---------------------: |
|      1 |              311,532 |                280,688 |
|      4 |              483,563 |                 68,488 |
|     16 |              552,992 |               106,021 |
|     64 |              580,811 |               275,509 |

Landing-leg throughput includes the queried contact, absorber state update,
footpad friction, wrench application, and one Rapier body step. Every support
remained loaded through the timed run:

| legs | serial leg-steps/s | parallel leg-steps/s | minimum loaded legs |
| ---: | -----------------: | -------------------: | ------------------: |
|    3 |          1,362,770 |            1,194,404 |                   3 |
|    4 |          1,715,194 |            1,324,728 |                   4 |
|    8 |          2,383,322 |            2,217,326 |                   8 |
|   16 |          3,200,146 |            2,997,158 |                  16 |

The `thessa-flight-authority` retractable-wheel benchmark exercises the full
contact-runtime path: current COM split, wheel-body resync, suspension-joint
frame updates, contact queries, gear actuation and one Rapier step. It switches
the gear target halfway through each timed run; the wheels begin clear of
terrain so these numbers isolate retraction overhead rather than loaded tire
forces. One host-local release run measured:

| wheels | timed steps | wheel-steps/s |
| -----: | ----------: | ------------: |
|      1 |       8,000 |        39,420 |
|      4 |       8,000 |        55,846 |
|     16 |       2,000 |        68,753 |
|     64 |         500 |       105,348 |

These are host-local throughput samples, not portable targets; the contact
benchmark also sweeps 1/8/64/256/1024 rigid bodies. Follow-up measurements must
cover representative aircraft and rover fleets, active-contact counts, terrain
streaming, and FlightAuthority dispatch. Known-case force equations are checked
to floating-point tolerance; contact-level error is reported as an absolute
fraction of vehicle weight and wheel load, never as a relative error near zero.

## 8. Deferred fidelity

- tire pressure dynamics, punctures, leakage, wear and carcass temperature;
- nonlinear load/slip curves from measured tire or regolith data;
- granular soil sinkage, bulldozing and wheel-soil shear beyond the calibrated
  Coulomb surface-contact model;
- tire structural fatigue and full electrical bus/battery integration;
- wheel contact with dynamic bodies and the corresponding equal-and-opposite
  terrain/body impulses;
- free-flight rotational inertia changes during wheel retraction, steering,
  anti-skid and active suspension;
- explicit fold-hinge rigid bodies, structural hinge loads and the
  equal-and-opposite angular-momentum reaction of moving gear;
- full visual wheel/tread/spoke CAD and deforming contact patches.

These limits are explicit model boundaries. Lower-detail tiers may simplify
geometry or update frequency only where their force/load error envelope remains
within the stated decision-relevant tolerance.
