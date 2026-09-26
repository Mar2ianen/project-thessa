# Reaction-wheel attitude control

Status: implemented gameplay model. Vehicle assets may install multiple
body-axis wheel banks; `sim-core` allocates their rated torque, vehicle baking
adds bank mass and inertia, and `FlightAuthority` routes residual attitude
demand through wheels before RCS.

## 1. Gameplay contract

Reaction wheels are dependable attitude-control boxes. They do not accumulate a
finite rotor-speed or angular-momentum state, so a long turn does not make them
lose authority. Each bank has a finite, authored torque rating per body axis;
the ratings combine across banks. This intentionally follows the convenient
KSP-style control feel instead of modeling wheel desaturation.

This is still an actuator model, not direct orientation editing. The allocator
emits a body moment, and the rigid-body integrator turns that moment into the
vehicle's angular response using the baked inertia tensor. There is no
wheel-generated torque when the requested moment is zero.

The current slice does not model electrical draw, rotor-speed telemetry,
thermal load, or momentum dumping. Reaction wheels and RCS have independent
enable switches; RCS may supply the part of the requested moment that exceeds
the installed wheel ratings.

## 2. State, inputs, and output

The backend-neutral bank data is:

```text
max_torque_body_nm     non-negative X/Y/Z actuator ratings
mass_kg                installed dry mass
position_body_m        bank center in the authored vehicle frame
inertia_body_kg_m2     bank inertia about its own center
```

For requested residual moment `M_req`, the combined wheel output is computed
independently on each body axis `j`:

```text
M_max,j = Σ bank.max_torque_body_nm[j]
M_wheel,j = clamp(M_req,j, -M_max,j, +M_max,j)
M_RCS = allocate_rcs(M_req - M_wheel)
```

The wheels run before RCS because they need no propellant in this gameplay
slice. If the wheels are disabled, their output is zero and RCS receives the
full residual. A disabled wheel bank does not change its mass or vehicle
inertia.

The runtime applies the actuator reaction moment through its ordinary attitude
integration path. `reaction_wheel_telemetry()` reports the moment actually
delivered by the wheel banks for the current control evaluation; the existing
`actuator_saturated` flag reports an unserved remainder after both wheel and
RCS allocation.

## 3. Asset format

`reaction_wheels` is optional and defaults to empty, so existing vehicle assets
remain valid. Ratings are in N·m; inertia is authored as matrix rows. The
vehicle baker combines module mass and inertia into the final COM frame.

```toml
[[reaction_wheels]]
name = "service-module-wheel-box"
max_torque_body_nm = [250.0, 250.0, 180.0]
mass_kg = 28.0
position_body_m = [-0.4, 0.0, 0.0]
inertia_body_kg_m2 = [
  [8.0, 0.0, 0.0],
  [0.0, 8.0, 0.0],
  [0.0, 0.0, 8.0],
]
```

See [`example_spacecraft.toml`](../../data/vehicles/example_spacecraft.toml).

## 4. Controls and verification

- The pilot HUD button and `Y` key toggle reaction wheels; `R` continues to
  toggle RCS independently.
- Below the authored torque ratings, the wheel allocator returns the request
  without numerical scaling; the combined wheel/RCS regression's absolute
  residual was `5.2e-9 N·m` for a `500 N·m` demand.
- Above a wheel rating, the wheel output clamps to that rating. With RCS
  enabled, RCS can fill the residual; with RCS disabled, the residual is
  reported as actuator saturation.
- A repeated 10,000-step allocation regression confirms that wheel authority
  does not decay with sustained use. No spin-speed behavior is represented by
  this intentionally reduced state model.
- The fleet benchmark runs 256 vehicles with 1, 4, 16, and 64 banks. On the
  development host it measured 32.5, 43.5, 43.2, and 44.5 million bank-steps/s
  respectively (0.03, 0.09, 0.37, and 1.44 μs per vehicle-step). Run it with
  `cargo bench -p thessa-sim-core --bench reaction_wheels`.

The allocator has no iterative solver or per-tick state allocation. Its work
is linear in the number of installed banks; the 64-bank case is a stress case,
while ordinary assets should use only the banks needed to author their torque
ratings.
