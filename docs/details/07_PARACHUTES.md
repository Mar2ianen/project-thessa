# KSP-style deployable parachutes

Status: implemented aerodynamic gameplay model. A vehicle asset can install
multiple named canopy packs; the player arms them together, while each pack's
pressure trigger and opening envelope control its own deployment. The model is
authoritative, deterministic, and independent of rendering.

## 1. Gameplay behavior

The pilot's `P` command arms installed packs for automatic deployment. An armed
pack waits until the vehicle is descending, local static pressure reaches its
authored trigger, and dynamic pressure is below its opening limit. This keeps
pre-armed packs stowed through launch and ascent. It then enters a reefed stage
and inflates over a configured simulation-time interval. The reefed canopy
already produces drag. A fully inflated canopy continues to produce drag until
the player cuts it or its structural load limit is exceeded.

The phases are:

```text
Stowed --arm--> Armed --descent + pressure + safe q--> Reefed --> Deployed
                  |                            |            |
                  +--disarm--> Stowed          +--cut------+--> Cut
                                               +--overload--> Failed
```

Cut and failed canopies are terminal for the current vehicle instance. An
armed pack can be disarmed before extraction; once extraction has started,
clearing the command cuts the canopy. Separate packs can use different pressure
triggers, so a drogue can open before the main canopy without a separate special
case in the physics.

## 2. State, inputs, and physical loads

Each pack stores its phase and reefing elapsed time. The deployment command,
atmosphere sample, local flow, and fixed physics timestep are the inputs. The
output is a deployment fraction, dynamic pressure, body-frame force, and
body-frame moment for telemetry and integration.

At the canopy mount `r`, local air-relative velocity includes rotational flow:

```text
v_local = v_air,COM + ω × r
q_local = 0.5 ρ |v_local|²
F_drag = -normalize(v_local) q_local Cd A f_deploy
M_body = r × F_drag
```

`A` is the fully inflated projected area, `Cd` is the authored canopy drag
coefficient, and `f_deploy` is the reefed-to-full area fraction. The force is
composed into the normal flight wrench and reaches the same rigid-body or
contact-active integrator as all other external loads. The canopy does not edit
orientation directly. Its packed mass, mount, and inertia are included in the
vehicle bake and final COM shift.

The static-pressure trigger makes altitude behavior adapt to each body's
atmosphere instead of embedding a planet-specific altitude. Dynamic pressure
gates the start of extraction; `max_canopy_load_n` limits the resulting
aerodynamic force after extraction. An overloaded canopy tears away and stops
contributing force. This first model treats a canopy as a drag-area actuator; it
does not yet simulate line elasticity, canopy shape deformation, inflation
shock, or packing/reuse.

## 3. Asset format

`parachutes` is optional and defaults to empty. Each pack requires its projected
area, drag coefficient, reefed area fraction, inflation time, pressure trigger,
opening dynamic-pressure limit, structural load limit, packed mass, mount, and
pack inertia. Values use SI units. The example includes an early-opening
drogue and a later-opening main canopy:

```toml
[[parachutes]]
name = "main"
reference_area_m2 = 32.0
drag_coefficient = 1.5
reefed_area_fraction = 0.12
inflation_time_s = 3.0
deploy_pressure_pa = 9000.0
max_deploy_dynamic_pressure_pa = 1800.0
max_canopy_load_n = 180000.0
pack_mass_kg = 24.0
position_body_m = [-2.0, 0.0, 0.0]
inertia_body_kg_m2 = [
  [4.0, 0.0, 0.0],
  [0.0, 4.0, 0.0],
  [0.0, 0.0, 4.0],
]
```

See [`example_parachute_vehicle.toml`](../../data/vehicles/example_parachute_vehicle.toml).

## 4. Controls and verification

- `P` and the HUD `CHUTE` control arm/disarm all installed packs; disarming an
  already extracting/open canopy cuts it.
- `ParachuteCommand::{Arm, Disarm, Cut}` and
  `FlightAuthority::command_parachute(name, command)` provide a typed,
  per-pack API addressed by the authored parachute name. The same operation is
  available through `VehiclePartCommand::Parachute` on the flight input API. A
  future staging or action-group dispatcher can route commands through it;
  dispatch itself is not part of this feature slice.
- HUD orbit telemetry reports armed/open/failed pack counts, total canopy drag,
  and maximum sampled canopy dynamic pressure. Authoritative snapshots carry
  each canopy's phase, deployment fraction, pressure, force, and moment.
- Regression tests pin the analytic drag equation, mount moment, pressure and
  dynamic-pressure gates, reefed-to-full timing, disarm/cut/failure behavior,
  pack mass/inertia baking, and force composition in `FlightAuthority`.
- The known-flow test uses `ρ = 0.2 kg/m³`, `v = 100 m/s`, `Cd = 1.5`,
  `A = 20 m²`, and `f = 0.55`: `q = 1000 Pa` and `|F| = 16,500 N`; the
  analytic regression tolerance is `1e-9 N`.
- The benchmark evaluates 256 vehicles with 1, 2, 4, and 8 packs for 10,000
  fixed steps. On the development host it measured 23.8, 25.5, 25.1, and 25.7
  million canopy-steps/s (0.04, 0.08, 0.16, and 0.31 μs per vehicle-step).
  Run with `cargo bench -p thessa-sim-core --bench parachutes`; this measures
  the state/load kernel, excluding atmosphere and integration.

The update is linear in installed pack count and reuses per-vehicle state/load
buffers. Packs in the terminal stowed, cut, or failed phases still report local
pressure telemetry but emit no force. Armed packs prevent vacuum on-rails jumps
so a cached translation step cannot skip their atmospheric deployment trigger.
