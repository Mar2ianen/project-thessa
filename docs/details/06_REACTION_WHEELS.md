# Reaction-wheel attitude control

Status: implemented gameplay model. Vehicle assets may install multiple
body-axis wheel banks; `sim-core` allocates their rated torque, vehicle baking
adds bank mass and inertia, and `FlightAuthority` routes residual attitude
demand through wheels before RCS.

## 1. Gameplay contract

Reaction wheels are dependable attitude-control boxes with an optional rotor
momentum reservoir. Each bank has a finite, authored torque rating per body
axis; the ratings combine across banks. A bank without an authored
`momentum_capacity_nms` keeps the convenient KSP-style feel: sustained
rotation never saturates. A bank *with* a capacity integrates stored momentum
`H += delivered_torque * dt` per axis and clamps its share so `|H|` stays
inside the reservoir; the unserved remainder flows to RCS through the usual
residual path, exactly like motor-rating saturation.

This is still an actuator model, not direct orientation editing. The allocator
emits a body moment, and the rigid-body integrator turns that moment into the
vehicle's angular response using the baked inertia tensor. There is no
wheel-generated torque when the requested moment is zero.

The current slice does not model thermal load. Reaction wheels and RCS have
independent enable switches; RCS may supply the part of the requested moment
that exceeds the installed wheel ratings.

## 1.3 Rotor speeds and motor heat

An authored per-axis `rotor_inertia_kg_m2` turns stored momentum into rotor
speed telemetry (`ω = H / I`, reported per bank alongside the saturation
fraction; speeds never feed back into allocation). Motor heat follows the
same energy ledger: `reaction_wheel_step_heat_w` returns bus draw minus the
rotor kinetic-energy rate (`KE = Σ H²/2I`), clamped at zero — this model has
no regenerative braking, so negative dissipation would mean unphysical energy
creation. Banks without inertia or tracking count the full draw as heat.
Wheel losses route into thermal nodes through `kind = "reaction-wheel"`
heat sources, sharing the caller-supplied `NamedWasteHeat` inventory with
generator routes.

## 1.2 Momentum saturation and desaturation

Saturation is resolved in two passes so net vehicle torque always stays as
commanded. The first pass serves the pilot/autopilot request against the
reservoir clamp; when no bank hits the wall it stands. When a bank does,
the runtime re-serves with an unload-biased target — body torque opposing
the stored momentum, with the total authority split across capped banks —
so the reservoir drains through the wheels while RCS carries the external
compensation (the residual). Unsaturated commands keep full priority; only
the saturating excess is rebalanced. With RCS disabled the residual goes
unserved and the drain still proceeds. Over-capacity stored momentum (only
reachable via deserialized state or a capacity-lowering asset edit) fails
closed at allocation instead of emitting unbounded torque. Stored momentum
is reported per bank via `reaction_wheel_momentum_telemetry()`.

## 1.1 Electrical load coupling

Each bank authors its motor draw as `idle_power_w` (quiescent draw while the
bank is enabled) plus `torque_power_w_per_nm` times the L1 magnitude of the
delivered torque. Both coefficients are per-bank actuator data, defaulting to
zero so older assets keep free wheels.

The flight runtime books each bank's demand on the shared bus under the
same-name consumer convention (the pattern mounted electric thrusters use):
a bank without a same-named consumer stays unmetered. Author the consumer
with `Utility` priority and a rating covering idle plus peak torque draw.
The granted share scales the next tick's wheel request (explicit Euler, like
solar tracking); a starved bus yields authority to RCS, which receives the
unserved remainder. Verified by the shedding regression
(`starved_wheel_bus_sheds_wheels_and_reports_unserved_load`): a 10 W metered
idle with no generation sheds the wheel fraction to 0 and reports 10 W
unserved.

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
idle_power_w = 12.0
torque_power_w_per_nm = 0.4
inertia_body_kg_m2 = [
  [8.0, 0.0, 0.0],
  [0.0, 8.0, 0.0],
  [0.0, 0.0, 8.0],
]
```

A same-named bus consumer meters the draw (see §1.1):

```toml
[[electrical_power.consumers]]
name = "service-module-wheel-box"
rated_power_w = 200.0
priority = "utility"
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
  does not decay with sustained use. Banks without a momentum capacity keep
  the legacy behavior; banks with one fill, saturate, and drain per the
  reservoir tests.
- Rotor-speed telemetry pins `ω = H / I` per axis (4 N·m·s-class momentum
  over 0.5 inertia reads 8/−4/0 rad/s); saturation fraction tracks fill;
  motor heat closes draw-minus-energy-rate (100 W draw against a 72 W
  spin-up rejects 28 W, starved draw clamps at zero).
- Bus coupling regressions pin the demand math (rating-share split plus idle)
  and the starvation path: a metered 10 W idle with no generation sheds the
  wheel grant to zero, reports 10 W unserved, and silences the next allocation.
- The fleet benchmark runs 256 vehicles with 1, 4, 16, and 64 banks. On the
  development host it measured 32.5, 43.5, 43.2, and 44.5 million bank-steps/s
  respectively (0.03, 0.09, 0.37, and 1.44 μs per vehicle-step). Run it with
  `cargo bench -p thessa-sim-core --bench reaction_wheels`.

The allocator has no iterative solver or per-tick state allocation. Its work
is linear in the number of installed banks; the 64-bank case is a stress case,
while ordinary assets should use only the banks needed to author their torque
ratings.
