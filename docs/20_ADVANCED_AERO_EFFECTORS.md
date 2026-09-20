# 20 — Advanced aerodynamic effectors: flaps, spoilers, grid fins, body flaps

Status: design target — flaps/spoilers/hinged-panels/grid-fins, neutral bounds,
`AeroEffectorModel`, and the high-speed effects plan (§§17–20: sonic boom,
buffet, vapor/cone/contrail visuals, plasma blackout, vortex lift, ground
effect, icing hooks) not implemented; incidence-only control is the runtime.

Status: **design target**.

This doc describes the next realtime-aero layer on top of the existing `PanelAeroModel`: high-lift devices, spoilers/speedbrakes, grid fins, and large Starship-class hinged body flaps. The goal is to extend the already working O(panels) solver without turning the runtime into CFD and without introducing separate `Aircraft`, `Rocket`, or `Starship` classes.

## 1. What already exists

The current `thessa-sim-core` already has the right foundation:

- `AeroPanel` with geometry, sweep, aspect ratio, `thickness_to_chord_ratio`;
- separate `center_of_pressure_body_m`;
- local flow via `omega × r`;
- attached/post-stall blending;
- control deflection as a change in effective AoA;
- static `Cm` and dynamic damping derivatives;
- SoA/SIMD path;
- optional Tier B coefficient tables;
- `ControlSurfaceDefinition`, which links one physical control channel to one or more panels.

`ControlSurfaceDefinition` already admits elevator, split elevons, rudder, flaps, and procedural surfaces in comments. But the current physical control-surface model is effectively just one:

```text
alpha_eff = alpha_aero + effectiveness * control_gain * deflection
```

This is enough for the elevator/rudder/aileron first slice, but not enough for devices that change camber/area, deliberately induce separation, or physically rotate a large part of the projected body area.

## 2. Main architectural split

Two things need to be separated:

```text
ControlSurfaceDefinition
    = physical actuator/channel, limits, linked panels

AeroEffectorModel
    = how actuator state change modifies aero coefficients / geometry
```

That is, `flap` should not become a new kind of vehicle, and `spoiler` should not become a special branch in `FlightAuthority`.

Example target model:

```rust
pub enum AeroEffectorModel {
    Incidence(IncidenceEffector),
    Flap(FlapEffector),
    Spoiler(SpoilerEffector),
    HingedPanel(HingedPanelEffector),
    GridFin(GridFinEffector),
    Table(TableEffectorId),
}
```

`Incidence` preserves the current semantics for elevator/rudder/aileron. The other models add their own local modifiers, after which the shared solver still returns force/moment via the same `AeroResult`.

## 3. Fix the command domain first

Currently `ControlSurfaceDefinition::validate` requires:

```text
minimum_deflection_rad < 0
maximum_deflection_rad > 0
```

and `apply_control_inputs` accepts a normalized command in `[-1, 1]` relative to zero.

This does not express a conventional spoiler or landing flap with a physical range of roughly `0 .. +delta_max`. It should not be worked around with a fictitious negative range.

An explicit neutral + arbitrary physical limits are needed:

```rust
pub struct ControlSurfaceDefinition {
    pub name: String,
    pub panel_indices: Vec<usize>,
    pub minimum_deflection_rad: f64,
    pub neutral_deflection_rad: f64,
    pub maximum_deflection_rad: f64,
    pub aero_effect: AeroEffectorModel,
}
```

Invariant:

```text
min <= neutral <= max
```

The normalized actuator command may stay `[-1, 1]`, but mapping is performed relative to `neutral`:

```text
command < 0: neutral -> min
command > 0: neutral -> max
```

For a pure flap/spoiler, the asset sets `min == neutral == 0`, so the negative half of the command either clamps to neutral, or the policy/allocator directly sets one-sided bounds. The cleaner long-term option is for the allocator to work directly in physical deflection bounds, with normalized pilot input terminating higher up the stack.

## 4. Runtime state must not mutate static geometry unnecessarily

Currently `apply_control_inputs` writes `control_deflection_rad` directly into `AeroPanel`. For the next layer it is better to distinguish:

```rust
AeroPanelDefinition   // static asset geometry
AeroPanelControlState // current actuator-derived state
```

For example:

```rust
pub struct AeroPanelControlState {
    pub deflection_rad: f64,
    pub forced_separation: f64,
    pub camber_delta: f64,
    pub area_scale: f64,
}
```

Not all fields must exist literally in this form. The principle matters: asset geometry does not become storage for transient aerodynamic effects.

The SoA path should receive compact arrays of already prepared control state, so that new effectors do not destroy the SIMD layout.

## 5. Flaps / high-lift devices

A conventional trailing-edge flap changes more than just the effective AoA. It changes:

- camber;
- zero-lift angle;
- lift curve / `CLmax`;
- drag;
- pitching moment;
- stall behavior;
- for Fowler-like devices — effective wing area/chord.

FAA/NASA reference material directly describes lift growth through increased camber and, for extendable devices, area; drag also grows noticeably at large deflection. Flap deployment changes the pitching moment, and the resulting pitch response depends on the specific aircraft geometry and downwash on the tail.

For Tier A, a reduced-order modifier is needed, for example:

```text
alpha0'      = alpha0      + d_alpha0(delta)
lift_slope'  = lift_slope  * k_lift_slope(delta)
CLmax'       = CLmax       + d_CLmax(delta)
CD0'         = CD0         + d_CD0(delta)
Cm'          = Cm          + d_Cm(delta)
stall_angle' = stall_angle + d_stall(delta)
area'        = area        * k_area(delta)       // optional Fowler-like
```

Not all dependencies must be analytic. A piecewise cubic / small lookup curve over normalized deployment is cheap and understandable.

### 5.1 Center of pressure

Physically, a flap changes the pressure distribution and thus the center of pressure. The runtime already has two independent ways to represent the pitching effect:

1. geometric force arm via `center_of_pressure_body_m × F`;
2. aerodynamic pitching coefficient `Cm`.

For a conventional flap, Tier A should preferably keep the static/geometric CoP and encode the resultant-force migration via `d_Cm(delta, alpha)` — this is more stable and does not create moving geometry in the hot loop.

**Must not** move the CoP by the equivalent amount and add the same `d_Cm` at the same time: that is double counting.

A Tier B table can directly specify `CL/CD/Cm` for each flap setting; then no analytic modifier is needed.

### 5.2 Flaperon / elevon

Multiple logical channels should not be allowed to mutate one panel in arbitrary order. A physical surface should be a single actuator, with symmetric bias + differential demand mixed down to the actuator target.

Example:

```text
left_flaperon  = flap_bias + roll_command
right_flaperon = flap_bias - roll_command
```

The allocator then clamps the physical deflection and reports residual/saturation. This fits well with the unified allocator from `docs/18-control-guidance-autopilot.md`.

## 6. Spoilers / speedbrakes

A spoiler is semantically different from an elevator. It rises into the flow and **spoils attached flow** over part of the surface: lift decreases, drag increases. With asymmetric deployment this creates roll/yaw authority; with symmetric deployment — speedbrake/lift dump; after touchdown, lift dump increases the normal load on the wheels and thus the available friction braking.

Therefore a spoiler cannot be modeled only as `alpha_eff += k*delta`.

A good Tier A proxy uses forced separation:

```text
s_nat     = natural_separation(alpha, Mach, ...)
s_spoiler = spoiler_separation(deployment, local_flow)

s_total = 1 - (1 - s_nat) * (1 - s_spoiler)
```

After that, the already existing attached/separated blend automatically reduces lift and moves the moment to the separated branch. Additionally, spoiler-specific form drag is needed:

```text
CD += d_CD_spoiler(deployment, alpha, Mach)
Cm += d_Cm_spoiler(...)
```

`AeroPanel::exposure` should not be used for this. `exposure` semantically refers to occlusion/wake and means what fraction of the panel sees the incoming flow at all. A spoiler does not hide the panel from the air — it changes the flow regime.

### 6.1 Ground spoilers

Ground-spoiler logic lives above aerodynamics:

```text
weight_on_wheels && deployment_command -> spoiler actuator target
```

The solver itself only produces less lift and more drag. If the ground/contact model computes the wheel-braking limit via the normal reaction,

```text
F_brake_max = mu * N
N ~= weight - aerodynamic_lift
```

then the ground-spoiler effect arises naturally without a gamey `+30% brakes` modifier.

## 7. Hinged body flaps / Starship-like surfaces

A Starship-like body flap is not a conventional high-lift flap. It is a large movable surface that noticeably changes the projected geometry and local pressure forces of the whole vehicle.

SpaceX describes the controlled belly-first descent of Starship as independent movement of **two forward and two aft flaps**. On V3, the aft-flap actuation system was separately reworked. These surfaces control aerodynamic moment and energy/attitude during entry/descent, rather than serving as a "flap for increasing `CLmax` on landing".

For such a surface, the current `alpha_eff` is insufficient. A generic `HingedPanel` model is needed that physically rotates the local panel axes around the hinge axis:

```text
R = rotation(hinge_axis, deflection)
chord_axis' = R * chord_axis
lift_axis'  = R * lift_axis
```

The flow sample position can be left near the geometry zone / hinge-derived point, with the force application point set by a separate CoP. For a large surface, the vehicle compiler can split the flap into several zones.

At large deflection angles it naturally becomes an almost plate-like body/control surface; the existing post-stall flat-plate branch is especially useful here.

### 7.1 Body flap authority

Authority must not have a special `if atmosphere`:

```text
q = 0.5 * rho * v^2
F_aero ~ q * S * C
```

In vacuum `rho -> 0`, and body flaps lose authority on their own. The unified allocator sees zero/small derivatives and shifts moment demand to RCS/TVC/reaction wheels.

This also enables a smooth mixed-control transition on entry without a hard switch for "now Starship is controlled by control surfaces".

### 7.2 Large-deflection caveat

For large body flaps, body/flap interference and hypersonic flow can differ strongly from an isolated flat plate. Therefore:

- Tier A `HingedPanel` — gameplay/realtime reduced-order model;
- Tier B table — preferred for the specific Starship-like vehicle;
- Tier C CFD/wind-tunnel/reference data — source of tables and validation, not a runtime dependency.

## 8. Grid fins

A grid fin is not just a small solid fin. The lattice creates complex internal flow; its coefficients depend strongly on Mach, incidence, deflection, and interaction with the body. NASA wind-tunnel work on Orion LAV specifically collected force/moment data for individual grid fins from subsonic through transonic to supersonic (`M=0.5…2.5`), which shows well: it is a natural candidate for a table-driven model.

Target architecture:

```rust
pub struct GridFinEffector {
    pub reference_area_m2: f64,
    pub hinge_axis_body: DVec3,
    pub body_interference_factor: f64,
    pub coefficient_source: GridFinCoefficientSource,
}

pub enum GridFinCoefficientSource {
    AnalyticProxy(GridFinProxy),
    Table(AeroCoefficientTableId),
}
```

A Tier A proxy can produce local normal-force / axial-drag coefficients as a function of:

```text
Mach
local alpha/beta
fin deflection
Reynolds (optional first slice)
```

but there is no need to pretend that a conventional thin-airfoil formula is accurate for a grid lattice in transonic/supersonic flow.

### 8.1 Super Heavy reference

The current Starship V3 / Super Heavy V3 reference must not be hardcoded as "four control surfaces": in May 2026 SpaceX described the transition **from four grid fins to three**, each roughly 50% larger and substantially stronger. They were also moved lower and re-clocked.

Thessa still must not have `SuperHeavyGridFinCount = 3`. A vehicle asset defines any number of independent effectors; V3 is simply a good validation/demo case for an asymmetric three-surface configuration.

## 9. Control allocation

After adding nonlinear effectors, the allocator must not know their names. It works with the local effectiveness matrix / Jacobian:

```text
J_i = d[wrench] / d[actuator_i]
```

For a conventional elevator the derivative is almost symmetric around neutral. For a spoiler with neutral at the lower bound the derivative is naturally one-sided. For a grid fin/body flap the derivative can depend strongly on the current Mach/AoA/q.

Recommended runtime flow:

```text
current state + local atmosphere
        |
        v
sample effector effectiveness around current actuator state
        |
        v
bounded allocator
        |
        v
actuator targets
        |
        v
rate/load-limited actuator dynamics
        |
        v
AeroPanelControlState
        |
        v
PanelAeroModel
```

There is no need to numerically perturb every actuator on every tick. For analytic effectors the derivative can be obtained cheaply; table effectors can store/interpolate derivatives. Numeric finite difference remains a fallback/debug reference.

## 10. Actuator dynamics and aerodynamic load

The next useful realism layer after the geometric effect:

- slew-rate limit;
- asymmetric extension/retraction rate;
- position limits;
- actuator failure/jam;
- hinge-moment / aerodynamic-load limit;
- thermal inhibit/damage;
- power/hydraulic/electric availability.

At high dynamic pressure a surface may have sufficient aerodynamic authority, but the actuator may be unable to reach the requested angle quickly or at all. This should appear as actuator saturation/residual wrench, not as an artificial reduction of `CL`.

The first slice may keep a constant slew rate; the load-dependent limit can be added later.

## 11. Interaction with stall/separation model

Effectors should use the shared separation state, but in different ways:

```text
Incidence:
    changes alpha_eff; authority fades after separation

Flap:
    changes attached coefficients / stall envelope;
    after separation the effect also fades/blends

Spoiler:
    itself adds forced separation

HingedPanel:
    changes geometry axes; then goes through the normal attached/separated solver

GridFin:
    usually coefficient table / dedicated proxy, with its own nonlinear response
```

This preserves a single physical place where attached vs separated flow is decided, and does not smear stall branches across `FlightAuthority`.

## 12. Center of pressure and moments

Common invariant for all new effectors:

```text
M_total = r_CoP × F + M_aero_local + M_dynamic
```

- `r_CoP` is used for the real geometry arm;
- `M_aero_local` — profile/pressure-distribution moment;
- deflection-dependent pressure migration is usually encoded via `dCm`;
- explicit moving CoP is acceptable if it comes from a geometry/table model and **is not duplicated** in `dCm`.

For a large body flap, the separate surface CoP is especially important: a large force far from the CG is the main source of control moment.

## 13. Occlusion / wake

The future BVH/wake solver and the current `exposure` are well suited for:

- body shadowing control surface;
- plume/flow occlusion;
- grid fin in the body wake;
- flap behind another geometry element.

But effectors must not write directly to `exposure` to emulate their own coefficient changes. First external-flow visibility is computed, then effector physics.

## 14. Performance

The requirement stays the same: realtime cost O(number of panels/effectors), with no runtime CFD.

The hot path should reduce to:

- compact per-lane control-state arrays;
- polynomial/piecewise modifiers;
- small table interpolation;
- minimum branches;
- scalar/SIMD parity.

Grid fins and body flaps are not a reason to move the whole craft to an expensive solver. If a specific vehicle requires accuracy — bake coefficients into Tier B.

## 15. Validation plan

### 15.1 Flaps

- neutral state bit-equivalent to the current panel solver;
- deployment increases lift at a low-AoA takeoff-like point;
- large deployment increases drag;
- `dCm` has asset-defined sign and does not double-count CoP;
- Fowler-like `area_scale` changes force proportionally to area at identical coefficients.

### 15.2 Spoilers

- deployment decreases lift and increases drag;
- left-only spoiler creates a roll moment of the correct sign;
- symmetric deployment creates no roll on a symmetric craft;
- with already-separated flow the additional effect is bounded;
- `exposure` stays independent.

### 15.3 Hinged body flaps

- `rho = 0` -> zero aerodynamic authority regardless of deflection;
- symmetric forward/aft commands give the expected pitch sign;
- differential left/right commands give roll/yaw sign;
- authority scales roughly with `q` within one coefficient regime;
- large-deflection geometry stays finite and does not produce NaN near 90°.

### 15.4 Grid fins

- proxy/table continuity across Mach cells;
- deflection = 0 in symmetric flow creates no spurious side moment;
- sign symmetry for `+delta/-delta` where reference data is symmetric;
- optional validation against the public NASA Orion LAV grid-fin dataset / published coefficients;
- body-interference multiplier bounded and explicit.

### 15.5 SIMD / determinism

For each new effector, the following are mandatory:

- scalar vs AVX2/AVX-512 coefficient parity;
- deterministic reduction order;
- serialize/deserialize roundtrip vehicle assets;
- no effect on unrelated panels at neutral;
- benchmark 1 / 16 / 256 / 1024 vehicles.

## 16. Suggested implementation order

### Slice A — control-surface data cleanup

- neutral deflection + unidirectional bounds;
- static definition vs runtime control state;
- preserve current incidence behavior bit-for-bit at neutral/current X-15 settings.

### Slice B — flaps + spoilers

- `FlapEffector` coefficient modifiers;
- `SpoilerEffector` forced separation + form drag;
- allocator support for one-sided actuators;
- add flap/spoiler example vehicle and tests.

### Slice C — generic hinged panels

- hinge axis and rotated local axes;
- large-deflection tests;
- 4-surface Starship-like body-flap demo vehicle;
- mixed RCS/aero allocation across decreasing/increasing `q`.

### Slice D — grid fins

- grid-fin table/proxy ABI;
- 3-fin Super Heavy V3-like demo geometry;
- public NASA grid-fin reference validation;
- transonic table interpolation and body-interference tuning.

### Slice E — actuator load/failures

- hinge moment proxy;
- load-dependent rate/position limits;
- jam/failure states;
- alerting hooks (`ACTUATOR`, `CONTROL AUTHORITY`, configuration warnings).

## 17. High-speed regime effects (transonic → hypersonic)

> New section in English per `AGENTS.md §0` (no new non-English documentation).

The M2 reduced-order branches already cover stall/transonic/supersonic
coefficients for forces. This section plans everything *around* the force
solver at high speed: acoustic footprint, unsteady buffet, condensation
visuals, plasma blackout, vortex lift, and ground effect. The governing rule
is the same as for effectors: every effect is computed from geometry, state,
and field — never a magic multiplier — and every visual-only effect is
explicitly marked as force-neutral so it cannot leak into flight dynamics.

### 17.1 Where the current solver stops

`PanelAeroModel` returns quasi-steady forces up to supersonic Mach. It does
not produce: ground acoustic footprint, unsteady buffet loads, condensation
or trail visuals, ionization/comm effects, nonlinear vortex lift, or
height-dependent induced drag. All items below consume data the sim already
has (Mach, `q`, alpha, altitude, attitude) plus small, explicit additions.

### 17.2 Sonic boom carpet (Tier A analytic proxy)

Physics sketch: a supersonic vehicle trails a Mach cone (half-angle
`μ = asin(1/M)`); the ground intersection is the boom carpet, roughly
`half_width ≈ altitude · cot(μ)` wide, swept along the ground track. The
N-wave overpressure scales with weight, length, altitude, and Mach. Rather
than a full Whitham F-function propagation, Tier A uses a calibrated scaling
law anchored at public reference points (Concorde-class ~2 psf cruise,
subsonic cutoff below which refraction turns the carpet around before it
reaches the ground):

```text
inputs:  Mach, altitude, weight, length, ambient pressure, ground track
output:  carpet polygon (map), peak Δp per ground cell, cutoff flag
```

- Cutoff is gameplay-relevant physics, not a hack: below cutoff Mach (a
  function of the temperature profile the atmosphere model already owns) the
  boom never reaches the ground — high-supersonic corridors vs low boom
  approaches become a real routing decision.
- Gameplay hooks, all derived from the footprint (never touching the flight
  model): window-rattle events above ~1 psf, damage claims above a tuned
  threshold, populated-area noise budget for career/contracts, ATC-style
  supersonic corridors on the map.
- Explicit non-goals: no CFD propagation, no focusing caustics in Tier A
  (flagged for Tier B via ray-tracing tables if ever needed), no effect on
  the generating vehicle's aerodynamics.

### 17.3 Transonic buffet (bounded unsteady load)

Shock-induced separation makes lift fluctuate near the buffet boundary.
Model: a deterministic (seeded-RNG, replay-safe) unsteady increment on
normal force plus control-effectiveness jitter, both strictly bounded:

```text
dCL_buffet = buffet_gain(Mach, alpha) · pseudo_noise(t, seed)
|dCL_buffet| <= buffet_envelope(Mach, alpha)   // hard cap, never diverges
```

The quasi-steady gate is an analytic smoothstep (shock-band Mach ×
high-alpha stall proximity, `buffet_gain` in `sim-core`), not a table —
tables stay Tier B for airframes with measured buffet boundaries. The
unsteady part is a stateless splitmix hash over `(seed, tick, lane)`.
Panels opt in via `AeroConfig::buffet_response` (default 0 = bitwise
legacy output); nonzero response routes SoA lanes onto the reference
scalar path so parity holds by construction.

- Onset boundary from a small table (Mach × alpha) calibrated against
  swept-wing buffet-onset references; outside the boundary gain is exactly 0.
- Telemetry: vibration level feeds the pilot HUD and the alerting hooks
  (`docs/19`), structural fatigue accumulates only through the existing
  thermal/structural graphs once M2 lands them — no parallel damage model.
- Validation: onset boundary shape, zero effect outside, bounded spectrum,
  determinism across replays and worker counts.

### 17.4 Vapor cone (force-neutral visual)

Transonic condensation cloud (Prandtl–Glauert singularity visualization):
rendered when local Mach ∈ [0.95, 1.05] over lifting surfaces AND humidity
allows it. **Adds zero force by design** — it is a visualization of the
pressure field the solver already computed, gated by an aloft-humidity
profile (currently a gap: surface `moisture01` exists in worldgen, the
aloft profile belongs to the `02A` TBD cells).

### 17.5 Contrails (force-neutral visual + signature)

Appleman criterion (cold + humid enough) evaluated per engine/wingTrail
emitter from the atmosphere temperature/humidity profile. Gameplay value is
signature, not physics: a visible trail is a detectable trail (traffic,
screenshots, future stealth considerations). Zero force coupling.

### 17.6 Plasma sheath and radio blackout (gameplay timer from physics)

Entry heating (M2 thermal forbidden-zone work) plus ionization proxy yields
an electron-density estimate along the trajectory; above threshold the
link budget is zero:

```text
heating proxy (velocity, density, nose radius) -> ne estimate
ne > ne_critical(link frequency) -> COMM BLACKOUT window
```

- Gameplay: autopilot/scripts must be able to fly blind through the window
  (ties into `docs/07` waits and `docs/19` alerting); ground stations show
  loss-of-signal honestly instead of freezing telemetry.
- Visual: entry glow intensity from the same heating proxy (shared source,
  no separate magic glow number).
- Validation: blackout entry/exit altitudes vs Shuttle-class reference
  corridors, order-of-magnitude only — Tier A is a window predictor, not a
  plasma solver.

### 17.7 Vortex lift for low-aspect/delta wings

Attached-flow panels underpredict delta lift at high alpha. Add the Polhamus
suction-analogy term, driven purely by geometry the asset already has
(aspect ratio, sweep, area):

```text
CL = Kp·sinα·cos²α + Kv·sin²α·cosα
```

- `Kp` from the existing attached solver (no double count: the potential
  part is the panel lift it already computes); `Kv` from aspect-ratio
  correlation, bounded and documented.
- Validation: delta-wing reference polars (e.g. 60–75° sweep datasets),
  continuity with the attached branch at low alpha, stall blend unchanged.

### 17.8 Ground effect (height-dependent induced drag)

Within roughly one wingspan of the surface, induced drag drops (McCormick /
Raymer-type factor over `h/b`, wingspan `b` from geometry). Affects flare
and float distance on landing — and must vanish with altitude by
construction (`factor → 1` for `h/b → ∞`, exact equality above cutoff, not
asymptotic tail that pollutes cruise).

### 17.9 Icing hooks (listed future, not sliced)

Performance-degradation envelope (CL down, CD up, stall angle in) driven by
visible-moisture + sub-zero exposure time, with anti-ice bleed-air gameplay
hooks. Requires the aloft-moisture profile from §18 first; no slice assigned
until M2 thermal exists.

## 18. Data the sim has vs gaps

| Effect | Already present | Gap to close |
|---|---|---|
| Boom carpet | Mach, altitude, weight, length, track, temperature profile | calibrated overpressure anchors (2 reference points to start) |
| Buffet | Mach, alpha, q, seeded RNG harness | onset-boundary table (small, literature) |
| Vapor cone | local Mach field | aloft-humidity profile (`02A` TBD) |
| Contrails | temperature profile, emitters | aloft humidity (same gap) |
| Plasma blackout | velocity, density, nose radius (M2 heating) | link-frequency thresholds per station |
| Vortex lift | aspect ratio, sweep, area | Kv correlation constants + reference polars |
| Ground effect | height AGL, wingspan | nothing (pure geometry + state) |
| Icing | temperature, exposure time | aloft moisture (same gap), M2 thermal |

The single highest-leverage data gap is the **aloft humidity profile** — it
unblocks vapor, contrails, and icing at once, and `02A` already lists TBD
cells for it.

## 19. Validation plan (high-speed effects)

- Boom: carpet width = `h·cot(asin(1/M))` exactly; cutoff respected (zero
  footprint below cutoff Mach); overpressure monotonic in weight/altitude,
  anchored within 2x of Concorde/Shuttle reference psf bands.
- Buffet: exactly zero outside the onset table; bounded spectrum;
  bit-identical across replays/worker counts (seeded).
- Vapor/contrails: force parity — identical trajectories with visuals
  on/off (bitwise, enforced by test).
- Blackout: entry/exit window exists on a Shuttle-like profile, absent on a
  low-speed descent; scripts survive it in replay fixtures.
- Vortex lift: matches reference delta polars within posted envelope;
  low-alpha continuity with attached solver.
- Ground effect: factor exactly 1 above cutoff; flare distance increases vs
  no-effect baseline on the same approach.
- Perf (per AGENTS.md §10): per-effect scope counters; boom carpet update
  O(track points), buffet O(panels) with the existing SIMD lanes, visuals
  behind the graphics quality tiers (`crates/graphics`).

## 20. Implementation order (continued)

### Slice F — boom carpet + buffet + condensation visual

- boom carpet polygon + Δp proxy + cutoff + map overlay + noise-budget hooks;
- buffet gain/envelope tables + HUD vibration + replay determinism tests;
- vapor-cone visual gated by Mach band (humidity gate stubbed to
  always-false until the `02A` profile lands — visible code path, no fake data).

### Slice G — plasma blackout + vortex lift + ground effect

- heating-proxy → blackout windows + entry glow from one source;
- Polhamus term behind aspect-ratio gating + reference-polar tests;
- ground-effect factor with exact high-altitude cutoff + flare tests.

### Slice H — icing + weather coupling (after M2 thermal)

- visible-moisture exposure accumulator + degradation envelope;
- anti-ice gameplay hooks; coupling point for any future weather model.

## References

High-speed effects (§§17–20):

- NASA Glenn, sonic boom basics (Mach cone, carpet, overpressure factors): https://www.grc.nasa.gov/www/k-12/airplane/sonic.html
- NASA, Seebass-George sonic-boom minimization and Carlson simplified boom prediction (N-wave scaling, cutoff Mach): https://ntrs.nasa.gov/citations/19690023553
- FAA, Noise levels for U.S. certificated and foreign aircraft (psf reference bands): https://www.faa.gov/regulations_policies/policy_guidance/noise/
- Appleman contrail forecasting (temperature–humidity criterion): https://www.weather.gov/
- Polhamus suction analogy for vortex lift on delta wings: https://ntrs.nasa.gov/citations/19660010884
- McCormick / Raymer ground-effect induced-drag factor vs height-to-span ratio.

Effector references (existing):

- Existing aero design: `docs/11_AERODYNAMICS.md`
- Unified control/allocator design: `docs/18-control-guidance-autopilot.md`
- Current panel solver: `crates/sim-core/src/aero.rs`
- Current vehicle/control-surface model: `crates/sim-core/src/vehicle.rs`
- NASA Glenn, spoilers: https://www.grc.nasa.gov/WWW/k-12/VirtualAero/BottleRocket/airplane/spoil.html
- NASA Glenn, flaps/slats: https://www.grc.nasa.gov/www/k-12/airplane/aflap.html
- FAA Airplane Flying Handbook, flap pitching behavior: https://www.faa.gov/sites/faa.gov/files/regulations_policies/handbooks_manuals/aviation/airplane_handbook/10_afh_ch9.pdf
- NASA NTRS, *Grid Fin Stabilization of the Orion Launch Abort Vehicle*: https://ntrs.nasa.gov/citations/20110013520
- NASA NTRS, *Simulation of Grid-Fin Control Surfaces*: https://ntrs.nasa.gov/citations/20110008384
- SpaceX, Starbase overview (two forward + two aft Starship flaps): https://www.spacex.com/vehicles/starship/assets/media/Starbase%20Overview.pdf
- SpaceX, May 2026 Starship V3 update (three Super Heavy grid fins; V3 flap actuation changes): https://www.spacex.com/updates/
