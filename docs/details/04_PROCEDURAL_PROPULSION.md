# Procedural propulsion systems

Status: design baseline with a shipped backend (`thessa-sim-core::propulsion`
+ `feed`): liquid chemical rockets + solid motors, isentropic nozzle core
with mixture sensitivity, cycle/feed bounds, geometry-derived mass, spool
runtime, altitude analyzer, vehicle mounts, tanks and feed lines, RCS and
nuclear thermal models, multi-chamber systems, air-breathing jets
(turbojet/turbofan/ramjet), and the ESTOC combined-cycle engine. Star/finocyl
grain burnback, tank depletion wiring, the flight-loop allocator, shaft-power
propulsion, scramjets, and the editor UI are still TBD (see section 18).

## 1. Design goal

Project Thessa should treat engines as procedural systems assembled from physical components and flow paths rather than as a catalogue of fixed engine parts.

The central rule is:

> An engine is a compiled network of energy-conversion, fluid-flow, and thrust-producing components. Named engine classes are common topologies and presets, not separate simulation laws.

The primary interaction reference is Juno: New Origins, especially its procedural rocket and jet-engine editing. Thessa should preserve the same accessibility while allowing deeper control over cycle topology, materials, working fluids, dimensions, feed systems, and component geometry.

The editor must support two depths:

- a simple mode where a player chooses a familiar engine family/preset and edits a small number of meaningful parameters;
- an advanced mode where the same engine is exposed as a component/flow network.

Both modes must produce the same underlying representation.

## 2. Common propulsion graph

A propulsion system is represented as connected components carrying one or more conserved flows.

Possible flow/resource kinds include:

- liquid or gaseous propellants;
- atmospheric working fluid/reactants;
- combustion products;
- plasma;
- shaft power;
- electrical power;
- thermal power;
- coolant flow.

Components consume, transform, split, merge, or accelerate these flows.

Conceptually:

```text
sources / environment
        |
        v
feed / compression / power conversion
        |
        v
energy addition
        |
        v
expansion / momentum transfer
        |
        v
thrust + exhaust + waste heat
```

The exact internal graph representation is TBD, but it should remain explicit enough that different engine families reuse the same component concepts where physically appropriate.

## 3. Hangar compilation

Procedural engine authoring data should not remain as a free-form editor graph in the hot path.

Leaving the hangar or loading a finalized vehicle compiles each propulsion system into validated runtime data.

Conceptually:

```text
Procedural propulsion graph
    |
    +-- validate topology
    +-- resolve materials and dimensions
    +-- solve/design-point component relationships
    +-- derive maps/limits where needed
    +-- derive mass, inertia, plumbing demand and thermal interfaces
    |
    v
Compiled propulsion system
    +-- runtime flow graph / reduced solver data
    +-- thrust and exhaust model
    +-- thermal loads
    +-- electrical/shaft-power interfaces
    +-- resource connections
    +-- structural/mount loads
    +-- render geometry / nozzle geometry
```

Runtime may still solve dynamic component state, spool speed, temperatures, pressures, ignition state, mode switching, and transient flow. The compile step removes editor-only geometry and performs expensive static derivations.

## 4. Liquid chemical rocket engines

Liquid rocket engines are built from a common set of subsystems rather than separate hard-coded engine types.

A representative path is:

```text
propellant tanks
      |
      v
feed system
      |
      v
injector / mixing
      |
      v
combustion chamber
      |
      v
throat
      |
      v
nozzle
```

### 4.1 Feed and cycle topology

Supported feed/cycle families should include at least:

- pressure-fed;
- electric-pump-fed, including Rutherford-like architectures;
- gas-generator / open-cycle turbopump;
- expander cycle;
- staged combustion;
- fuel-rich staged combustion;
- oxidizer-rich staged combustion;
- full-flow staged combustion;
- additional physically justified cycles later without changing the overall model.

These should be implemented as common components connected differently. A Rutherford-like engine is therefore an electric-pump topology, not a special simulation class.

Relevant procedural inputs may include:

- pump/compressor dimensions and pressure ratio;
- turbine dimensions and inlet temperature;
- shaft speed and power limits;
- electric motor power and efficiency;
- preburner mixture and pressure;
- chamber pressure;
- injector family/area;
- combustion-chamber dimensions;
- mixture ratio;
- throat size;
- nozzle geometry;
- cooling topology;
- component materials.

### 4.2 Propellant pairs

The engine definition selects working propellants/reactants rather than being permanently tied to a single fuel pair.

Examples include, but are not limited to:

- LOX/RP-1;
- LOX/CH4;
- LOX/H2;
- storable hypergolic pairs;
- monopropellants where applicable;
- future fictional/advanced propellants when the setting justifies them.

The model should derive performance from fluid/combustion properties rather than use a fuel-name lookup that directly returns thrust and specific impulse.

### 4.3 Materials and cooling

Materials are physical constraints, not cosmetic tier labels.

Material properties may affect:

- density and component mass;
- allowable temperature;
- yield/ultimate strength;
- creep/lifetime limits;
- thermal conductivity;
- compatibility with propellants/oxidizers;
- manufacturable wall thickness or pressure limit.

Cooling modes may include regenerative, film, ablative, radiative, or later specialized systems.

A high chamber pressure or temperature is valid only if the selected geometry, cooling, material, and feed system can support it.

## 5. Propellant lines and manifolds

Propellant/feed plumbing should be part of the propulsion design rather than an invisible unlimited connection.

The first useful model does not need full CFD inside every pipe, but lines should have enough physical state to create real design trade-offs.

Potential parameters include:

- diameter;
- length;
- material;
- pressure rating;
- insulation;
- fluid;
- fittings/manifold losses;
- allowed mass flow.

From these the compiled system may derive pressure drop, mass, heat leak, flow limits, and pump inlet/outlet requirements.

This creates a natural constraint: a very large engine cannot be fed through arbitrarily small plumbing without consequence.

## 6. Procedural nozzles

Nozzles are procedural geometry, not a fixed `sea level` / `vacuum` selector.

Relevant parameters include:

- throat radius/area;
- expansion ratio or exit radius;
- nozzle length;
- contour/profile;
- wall thickness;
- material;
- cooling method.

Useful preset families may include:

- conical;
- bell;
- shortened bell;
- aerospike or plug-nozzle families later where justified.

The compiler/runtime derives pressure adaptation, over/under-expansion behavior, mass, thermal load, and performance from the actual nozzle and chamber state.

## 7. Nuclear thermal and externally heated rockets

Nuclear thermal propulsion should reuse the same feed/nozzle concepts wherever possible.

Conceptually:

```text
propellant tank
      |
      v
feed system
      |
      v
reactor / heat source
      |
      v
heated working fluid
      |
      v
nozzle
```

The principal difference from a chemical rocket is the source of working-fluid enthalpy.

Reactor/heater parameters may include:

- reactor/core family;
- fissile/fuel material;
- core geometry and dimensions;
- channel geometry;
- thermal power;
- maximum fuel/core temperature;
- structural/moderator materials;
- working-fluid compatibility;
- startup/shutdown/transient limits.

Different propellants such as hydrogen, methane, ammonia, water, or other fluids should naturally trade storage density, molecular mass, temperature limits, thrust, and exhaust velocity.

The same architecture should leave room for other externally heated rocket concepts without creating an unrelated propulsion subsystem.

## 8. Gas-turbine and shaft-producing engines

Turbojet, turbofan, turboprop, and propfan/open-rotor engines should share a common gas-path and shaft-power model.

A representative topology is:

```text
intake
  |
  v
fan / compressor stages
  |
  v
combustor
  |
  v
turbine stages
  |\
  | \---- shaft / gearbox ----> fan or propeller
  v
jet nozzle
```

The topology and power split determine the familiar engine class.

Examples:

- most useful energy exits through the core nozzle -> turbojet-like;
- large bypass/fan flow -> turbofan-like;
- most shaft power drives a propeller -> turboprop-like;
- large open rotor/fan -> propfan/open-rotor-like.

Potential procedural parameters include:

- intake area/geometry class;
- compressor/fan diameter;
- stage count;
- pressure ratio;
- shaft arrangement;
- spool count;
- turbine inlet temperature;
- turbine/compressor materials;
- bypass ratio;
- gearbox ratio;
- combustor dimensions;
- afterburner/reheat section;
- core and bypass nozzle geometry.

Named engine families are presets over this common representation.

## 9. Piston, electric, and generic shaft-power propulsion

Propellers/fans should be reusable thrust-producing components driven by different sources of shaft power.

### 9.1 Piston engines

A piston engine converts fuel/oxidizer chemistry into shaft power, which then drives a propeller or fan.

Procedural parameters may include displacement, cylinder arrangement/count, compression ratio, boost/supercharging, RPM range, cooling, materials, and fuel.

### 9.2 Electric motors

Electric propulsion for aircraft/rotorcraft is represented as:

```text
electrical source -> motor/controller -> shaft -> propeller/fan
```

The motor should expose physical power, torque/RPM, efficiency, thermal limits, and mass rather than exist as a special `electric propeller` force primitive.

This allows batteries, fuel cells, fission reactors, fusion reactors, generators, or other electrical sources to drive the same propeller/fan component.

## 10. Atmospheric reactants are not assumed to be oxygen

Air-breathing combustion must not assume that an atmosphere always provides oxidizer.

The atmosphere supplies a sampled gas mixture. The propulsion system decides whether useful species from that mixture act as oxidizer, fuel, inert working fluid, or some combination.

Examples:

```text
Earth-like atmosphere:
    atmosphere supplies O2 oxidizer
    vehicle supplies hydrocarbon/H2 fuel

Methane-rich atmosphere:
    atmosphere may supply CH4 fuel
    vehicle supplies oxidizer

Hydrogen-rich atmosphere:
    atmosphere may supply H2 fuel
    vehicle supplies oxidizer
```

A combustor therefore consumes reactants according to chemistry/composition rather than according to a hard-coded `intake air = oxidizer` rule.

This is important for Thessa's non-Earth environments and should apply consistently to turbojets, turbofans, ramjets, combined-cycle engines, and other atmospheric propulsion.

## 11. Ramjets and high-speed air-breathing engines

Ramjets/scramjets should reuse intake, combustor, and nozzle concepts without requiring compressor/turbine machinery.

Conceptually:

```text
intake / inlet compression
          |
          v
       combustor
          |
          v
        nozzle
```

Ram compression replaces compressor work. Future supersonic-combustion support can extend the same family toward scramjet-like operation.

Performance should depend on inlet conditions, flight Mach number, geometry/model assumptions, reactant chemistry, thermal limits, and nozzle state.

## 12. Combined-cycle engines

ESTOC-class combined-cycle propulsion (our implementation of the
switchable air/rocket niche) is represented as multi-mode graphs with
shared hardware and alternate flow paths, not as a hard-coded `air mode
/ rocket mode` engine primitive.

Conceptually:

```text
                  +-- intake -> precooler/compressor --+
fuel -------------+                                    +-> chamber -> nozzle
onboard oxidizer --+----------- rocket path ------------+
```

Valves/mode logic select which path is active. Components such as chamber, nozzle, pumps, heat exchangers, shafts, or compressors may be shared between modes.

This architecture should also permit turbo-rocket, ejector-rocket, and other hybrid cycles where future gameplay/physics justifies them.

## 13. Electric/plasma space propulsion

Low-thrust electric propulsion should be componentized by its actual acceleration/heating mechanism.

Useful families include:

- electrostatic ion thrusters;
- Hall-effect thrusters;
- electromagnetic/MPD-type thrusters;
- electrothermal resistojet/arcjet-type systems;
- future VASIMR-like magnetic/plasma systems where appropriate.

The common resource structure is broadly:

```text
electrical power + propellant
        |
        v
ionization / heating / acceleration
        |
        v
exhaust + thrust + waste heat
```

Procedural inputs may include accelerator/grid dimensions, voltage/current, magnetic field, chamber dimensions, propellant, power electronics, cooling, and materials.

Runtime characteristics should emerge from supplied electrical power, mass flow, accelerator state, efficiency, and thermal limits rather than from a fixed thrust value.

## 14. Fusion propulsion

Fusion propulsion should have at least two high-level families.

### 14.1 Continuous fusion / fusion torch

Conceptually:

```text
fusion reactor
      |
      v
plasma / thermal coupling
      |
      v
magnetic or other nozzle
      |
      v
high-velocity exhaust
```

Relevant parameters may include reactor specific power, fusion gain, plasma temperature, exhaust fraction, magnetic-nozzle efficiency, working fluid/reaction products, field strength, and cooling/radiator capacity.

### 14.2 Pulsed fusion

Conceptually:

```text
pellet / pulse-unit feed
         |
         v
fusion pulse system
         |
         v
magnetic / mechanical impulse coupling
         |
         v
thrust pulses
```

This supports pulsed-fusion concepts without pretending they behave like steady-flow chemical engines.

### 14.3 Epstein-class / advanced torch engines

An Epstein-class engine is treated as an extremely advanced continuous fusion/torch topology rather than a single hard-coded magic part.

Its exceptional thrust and exhaust velocity should emerge from advanced reactor specific power, materials, magnetic confinement/nozzle performance, thermal management, and energy-conversion efficiency.

The game should therefore prevent early high-performance torch engines through missing technology/material capability and unsatisfied physical limits rather than by an arbitrary engine-name lock.

## 15. Presets and accessibility

The base game should ship many convenient presets/topologies so that procedural depth does not become mandatory busywork.

Examples may include:

- pressure-fed storable rocket;
- kerolox gas-generator rocket;
- methalox staged-combustion rocket;
- full-flow methalox rocket;
- Rutherford-like electric-pump rocket;
- nuclear-thermal hydrogen engine;
- turbojet;
- high-bypass turbofan;
- turboprop;
- open rotor/propfan;
- piston propeller engine;
- electric propeller drive;
- ramjet;
- ESTOC combined-cycle engine;
- Hall thruster;
- ion thruster;
- MPD/plasma thruster;
- continuous fusion torch;
- pulsed-fusion engine.

A preset instantiates the same editable components available in advanced mode. It must not be an opaque separate physics implementation.

## 16. Cross-system coupling

Procedural propulsion should connect naturally to the rest of Thessa rather than report thrust in isolation.

Important outputs/requirements include:

- propellant/resource mass flow;
- electrical power draw/generation;
- shaft power and torque;
- heat generation and cooling demand;
- radiator requirements for advanced engines;
- atmospheric intake demand;
- exhaust plume state;
- engine mass/inertia;
- mount forces and moments;
- vibration/pulsed loads where relevant;
- plumbing and service connections;
- startup/shutdown time;
- throttle response;
- failure/damage evidence.

This lets propulsion participate in the same structural, thermal, resource, electrical, and vehicle-design systems as other procedural parts.

## 17. Scope boundary

The first implementation does not need to solve every real engine detail.

It does need to establish stable boundaries so higher fidelity can be added without replacing the authoring model.

Initial non-goals may include:

- full transient CFD through every turbomachinery passage;
- detailed injector combustion instability;
- blade-by-blade compressor/turbine simulation;
- microscopic reactor/neutronics transport;
- unrestricted pipe routing CAD;
- exact manufacturing simulation;
- full plasma kinetic simulation.

The target is an engineering game model: component topology and geometry should create meaningful, physically interpretable trade-offs while remaining computationally tractable and editable by normal players.

## 18. Backend implementation (v1)

Shipped in `crates/sim-core/src/propulsion.rs` (MIT engine crate, no Bevy/Tokio/wgpu).

### 18.1 What is modeled

- Isentropic frozen-flow nozzle core: c* from chamber thermo, exit Mach
  from expansion ratio (Newton + bisection fallback), thrust coefficient
  with the ambient pressure term exact, mass flow from throat area.
- Propellant pairs (LOX/RP-1, LOX/methane, LOX/hydrogen, NTO/MMH,
  APCP solid) carrying gamma, chamber temperature, gas constant, bulk
  density, and characteristic length. Performance is derived from these
  properties; the c* calibration test pins each pair within 4% of its
  published anchor.
- Feed cycles as engineering bounds, not multipliers: chamber-pressure
  caps per topology, a gas-generator bypass modeled as a second
  isentropic duct at duct temperature/expansion, electric-pump power from
  flow times pressure rise with a documented specific-power mass.
- Materials as density/yield/temperature properties sizing thin-wall
  chamber and nozzle mass; cooling modes gate pressure (radiative) or
  burn duration (ablative) instead of derating silently.
- Conical/bell nozzles with the divergence factor from wall geometry;
  the bell recovers half the residual divergence loss (thrust envelope
  +/-1% pinned by test).
- Solid BATES grains: equilibrium pressure from the Saint-Robert law in
  closed form over the web, progressive-trace signature pinned by test,
  n >= 1 refused (no stable equilibrium exists).
- Runtime spool state (first-order lag, ignition shots, solid burn
  clock), altitude analyzer over any `AtmosphereConfig` (the Juno
  Performance Analyzer backend: ~180 ns/point, a 21-row curve in under
  4 us), and engine mounts in `VehicleDefinition` with point-mass bake
  aggregation plus uniform-command thrust queries.
- Plume handoff: `EnginePlumeState` maps field-for-field into
  `plume-core` `PlumeSource` through the single `engine_plume_source`
  choke point, with no new cross-crate dependency.

### 18.2 Tails closed after v1

- Mixture-ratio sensitivity: per-pair (ratio, chamber temp, gamma, gas
  constant) tables with piecewise-linear interpolation, hard refusal
  outside the modeled range, reference point reproduced exactly. Tables
  are representative CEA trends; refine with project CEA runs.
- Aerospike contour (linear): near-axial divergence, altitude
  compensation down to base drag on the plug base, separation flag
  never trips by design. Sea-level thrust holds within 3% of vacuum.
- Shaped solid thrust: per-segment port radii (stepped channel) solved
  on a coupled time-stepped trace; boost-sustain signature and
  integrated-vs-geometric propellant agreement pinned by test.
- Thermal interface data: chamber stagnation power, exhaust kinetic
  power (ordering pinned), nozzle wall area; the graph hookup waits for
  a runtime thermal graph to exist.
- Solid depletion queries: remaining grain vs burn clock, vehicle mass
  with grain burned off (inertia held, documented).
- Tanks and feed lines: thin-wall vessels with weld/fixture allowance,
  Darcy-Weisbach + minor-loss drops with a velocity gate (refusal, not
  derating), baker `[[tanks]]` with mass aggregation and a
  pressure-fed feed-pressure cross-check.
- Gimbal authority for the allocator: per-command force/moment pairs
  about the transverse axes from lever arms (thrust-times-arm pinned).
- Editor-facing analyzer CLI: `vehicle-baker --analyze` prints the
  Performance Analyzer table (JSON under `--analyze-json`).

### 18.3 Reaction control and nuclear thermal (v3)

- RCS propellants: monopropellant hydrazine (catalytic chamber through
  the shared pressure-fed liquid path, fixed full thrust) and cold-gas
  nitrogen/helium (chamberless compile; runtime thrust tracks inlet
  pressure exactly through choked flow).
- Pulse physics: triangular valve rise with propellant booked over the
  full open time, so short pulses lose effective Isp causally; minimum
  impulse bit, hydrazine Isp band (210-235 s), and N2 Isp band (65-85 s)
  pinned by test. Mounted `RcsCluster` delivers force/moment impulses
  and PWM-average wrenches (opposed-pair pure couple pinned).
- NTR: power-limited compile (mdot from reactor power balance, chamber
  pressure from choked flow, expander cap refusal), hot-fluid properties
  for H2/CH4/NH3/H2O, frozen-flow dissociation efficiency on
  `kinetic_efficiency` (NERVA-pinned), reactor mass from specific power,
  NERVA-class golden test (750-950 s, 150-350 kN, 8-20 t), startup tau
  wired into spool, decay-heat cooldown tail. Compiled output reuses
  `CompiledLiquid`, so spool/throttle/plume/analyzer paths just work.
- Baker `kind = "nuclear"` plus monopropellant RCS assets; the
  pressure-fed feed cross-check covers RCS tanks. `VehicleDefinition`
  gains a per-mount force/moment `wrench_body_n` for clusters.

### 18.2 Validation

- Merlin-1D-class golden test: 845 kN / 914 kN and 282 s / 311 s within
  5%, dry mass inside the published band with margin.
- Sonic-throat and area-Mach round-trip special cases; vacuum/sea-level
  ordering; throttle linearity pin; Summerfield separation flag;
  cycle/cooling gate refusal tests; NaN-closed validation throughout.
- Bench `crates/sim-core/benches/propulsion.rs`: hangar compile
  ~20 us (liquid) / ~12 us (solid), analyzer and solid-replay sweeps.

### 18.4 Multi-chamber systems (v4)

- `propulsion::system`: one shared feed (single turbopump set, single
  GG duct on total bypass flow, common tanks) driving 1-16 chamber/nozzle
  assemblies at their own stations — RD-170-style clustering.
- Native compile (not N single compiles): shared hardware books once
  from total flow; chambers carry walls, nozzles, injectors, heads, and
  gimbals. A single-chamber system reproduces the standalone liquid
  compile bit-for-bit (pinned); totals scale linearly, so clustering buys
  runtime authority (differential throttle, per-chamber gimbals, one
  plume source per nozzle), not mass magic.
- Runtime: per-chamber throttles with the shared duct following total
  flow, per-nozzle plume states, force/moment wrench with the GG duct
  distributed proportionally (documented rule, shared with the vehicle
  total), per-chamber gimbal authority, independent spool states under
  the same first-order law, and a system altitude analyzer for the
  editor. Gimbal actuators size by chamber thrust (the standalone liquid
  path was corrected to match; Merlin band unaffected).
- Vehicle integration: `systems` mounts with per-chamber bake mass
  (shared hardware at the chamber-mass centroid), uniform-throttle total
  thrust, per-system differential wrench; baker `[[systems]]` with
  nested `[[systems.chambers]]` plus the pressure-fed feed check.

### 18.5 Air-breathing jets (v5)

- `propulsion::air`: turbojet, turbofan, and ramjet from one Brayton
  core. Juno-style sliders (intake area/recovery, compression and bypass
  ratios, turbine temperature, fuel, afterburner, nozzle) plus the cycle
  internals Juno hides: polytropic efficiencies, turbine cooling bleed
  with rotor-bypass work split and mixing loss, customer bleed,
  part-power TIT/pressure/flow schedules, and oxygen gating for
  non-Earth atmospheres (Juno 1.4 scales jets with O2 the same way).
- Turbine cycles run convergent nozzles; ramjets run fixed
  convergent-divergent geometry adapted at the design point (Mach 2 sea
  level) with a Summerfield separation check and a separated fallback to
  convergent-at-throat behavior.
- Fixed-geometry matching: the nozzle sets swallowed flow — demand
  beyond choked capacity rescales the whole engine consistently instead
  of booking fuel for unswallowed air (this exact inconsistency was
  caught by the energy pin during development).
- Validation: Olympus-593-class anchor bands (thrust, Isp, mass order,
  design flow) with documented input uncertainty and no fitted
  multipliers; ramjet static-zero and Mach-rise pins; vacuum/anoxic
  flameout; fan-vs-jet efficiency ordering; reheat tradeoff; full first-
  law energy pins (useful + exhaust KE vs fuel + inlet KE); hypersonic
  drive-limit flameout; size-scaling and refusal tests; Mach × altitude
  analyzer grid (the Juno Mach-table contract, computed from the cycle).

### 18.6 ESTOC combined-cycle engine (v5)

- `propulsion::estoc`: air-breathing turbojet path plus closed-cycle
  rocket path sharing intake ducting, chamber, and nozzle hardware under
  our own name (the switchable air/rocket gameplay niche, no borrowed
  trademarks). Strict mode discipline: manual wins, vacuum always
  rockets, Mach band with hysteresis, dead air path (stalled drive,
  anoxic air) falls back to rocket — never blended.
- Shared convergent nozzle caps rocket expansion (documented): rocket
  mode buys thrust where air fails (vacuum Isp band 200-350 s), not
  orbital efficiency. Rocket chamber from LOX-pair thermo at reference
  mixture, OF-split oxidizer bookkeeping, pump-feed cap, throat-clearance
  validation, reinforcement + feed mass only (nozzle books once).
- Mode transitions smooth thrust first-order over the transition tau
  (threaded prev/mode state; fresh-start convention for the editor);
  per-nozzle plume states reuse the jet handoff; baker `[[jets]]` kinds
  `jet`/`estoc` with analyzer Mach grids and JSON rows.

### 18.7 Still deferred

Star/finocyl grain geometry (needs numerical perimeter burnback, not a
tweak of the port solver), tank depletion wiring into the flight loop
(the queries exist; the loop still flies baked mass), per-engine
allocation in the flight loop (authority pairs exist; the allocator
still sees one lever), shaft-power propulsion (turboprop/piston/electric
fans), scramjets, and the editor UI itself (the CLI/JSON analyzer
is its backend contract).
