# Procedural propulsion systems

Status: design baseline with a shipped backend (`thessa-sim-core::propulsion`
+ `feed`): liquid chemical rockets + solid motors, isentropic nozzle core
with mixture sensitivity, cycle/feed bounds, geometry-derived mass, spool
runtime, altitude analyzer, vehicle mounts, tanks and feed lines, RCS and
nuclear thermal models, multi-chamber systems, air-breathing jets
(turbojet/turbofan/ramjet) with the single-spool shaft/starter runtime
(starter topologies, light-off/self-sustain, relight, generator load),
composition-aware atmosphere queries (section 10), and
the ESTOC combined-cycle engine, plus piston/electric propeller drives and a
stateful, heat-budgeted turboprop takeoff path on a reusable ideal actuator
disk (section 9). Star/finocyl grain burnback, tank depletion wiring, the
flight-loop allocator, transient piston/electric source and prop-shaft
state, finite-blade propeller maps, an independent free-power-turbine spool,
high-fidelity scramjet shock-train/finite-rate chemistry, and the editor UI
are still TBD (see section 18).

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

### 8.1 Spools, gearboxes, starters, and relight

Gas-turbine startup is part of the authored shaft topology, not a free state
change. Multi-spool engines may have independent LP/IP/HP shafts, optional
gearboxes, clutches, and motor-generators. Starter torque acts on a selected
shaft (normally a core/HP spool); a geared fan or LP spool does not imply that
the core is mechanically locked to it.

The runtime state must therefore track enough shaft state to answer:

```text
shaft angular speed / normalized spool speed
compressor/fan aerodynamic torque
turbine torque
starter torque and power
gear/clutch coupling
bearing/accessory losses
minimum light-off speed
minimum self-sustaining speed
ignition state
```

Starter hardware is an authoring choice with real mass/resource consequences.
It is not synonymous with an electrical generator: start torque and generated
electrical power are separate optional shaft accessories, though one reversible
machine may implement both. At minimum support these starter topologies:

```text
none / windmill-only
    no onboard starter hardware;
    saves starter/generator mass and startup power infrastructure;
    cannot self-start at rest;
    airborne relight succeeds only when inlet-driven shaft torque reaches
    light-off speed and combustion can accelerate the core to self-sustain

electric starter-generator
    electrical bus -> motor/generator -> selected spool;
    supports zero-airspeed start;
    after light-off the same machine may generate power

pneumatic / air-turbine starter
    APU, ground cart, or cross-bleed air -> starter turbine -> selected spool;
    trades electrical demand for ducting/valves and an external or onboard
    compressed-air source

rocket / gas-generator bootstrap
    onboard propellant drives a starter turbine or shared rocket machinery;
    especially natural for combined-cycle engines;
    consumes propellant but can start independently of ambient airspeed
```

A starterless aircraft is therefore a valid deliberate design. Wheel motors may
accelerate the vehicle until ram/windmill torque can relight the core; the
required speed is not a constant vehicle stat. It emerges from intake state,
air density, shaft inertia, compressor map/drag, gearbox topology, and the
chosen light-off/self-sustain thresholds.

If wheel propulsion and the engine share an electrical bus, wheel-motor energy
can start an engine at zero airspeed only when a starter-generator/cross-drive
path actually connects that bus to the required core spool. Merely moving the
aircraft on powered wheels does not mechanically spin an uncoupled compressor.

Electrical generation is independently optional. A shafted turbine engine may
carry no generator, a dedicated generator, or a reversible starter-generator.
Authoring/runtime must expose at least:

```text
generator fitted / absent
attached spool
maximum electrical power
maximum shaft torque draw
efficiency map or bounded efficiency
cut-in spool speed
thermal limit
bus connection
motor capability (if reversible)
generator mass
```

Generator load must appear in the shaft work balance. Drawing electrical power
reduces available turbine margin; at low spool speed the generator may be
offline, power-limited, or able to motor the shaft only if it is explicitly a
starter-generator. Conversely, an engine with no generator must not create
electrical bus power just because it is running.

A ramjet has no compressor/turbine shaft, so `starter = none` is its normal
topology and its static thrust remains zero. If a ramjet installation needs
electrical power, it must obtain it from the vehicle bus or from a separately
modelled source (battery, fuel cell, RAT/air-turbine generator, auxiliary
turbogenerator, etc.). A rocket-ejector bootstrap is a separate combined-cycle
path, not a hidden ramjet starter.

The steady-state engine solver must not manufacture starter power. At zero
shaft speed, compressor suction is zero unless an explicit starter, cross-drive,
rocket ejector, or other modeled source creates flow/torque. A separate
steady-state performance analyzer may continue to evaluate already-running
static thrust, but it must label that assumption explicitly.

Status note (2026-09-24): the single-spool runtime described above has landed
in `propulsion::shaft` (section 18.8) — starter topologies with real
stored-energy draw, light-off/self-sustain hysteresis, windmill relight,
spool-scaled compressor suction, and generator load in the shaft work
balance. A power-level downstream power-turbine load now shares this balance,
and its enthalpy extraction is documented in section 18.10. Still deferred
from this section: independent multi-spool/gearbox coupling, torque-level
starter/generator authoring, pneumatic and rocket-bootstrap resource
pipelines (stored-energy topologies currently book one energy reservoir),
and generator thermal limits/bus connection.

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

Status note (2026-09-24): the backend now has a reusable annular ideal
actuator disk with piston-Otto, electric-motor, and gas-turbine power-takeoff
sources, including vehicle mounts, mass aggregation, analyzer rows, and
electrical/fuel/thermal/shaft telemetry (section 18.10). The disk is explicitly
an incompressible momentum-theory bound: blade-element pitch/stall/profile
maps are not implied. Piston/electric sources are steady operating points;
the turboprop shares the existing normalized gas-generator shaft state and
uses a power-level takeoff command. Transient piston/electric startup,
propeller-shaft/free-turbine inertia, and finite-blade maps remain open.

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

The atmosphere may also be useful when it supplies no chemical reactant at all.
A rocket/ejector or air-augmented-rocket topology may inject onboard fuel and
onboard oxidizer, then entrain atmospheric gas as additional working mass. The
ambient gas is heated/mixed by the primary rocket flow and can improve
propulsive efficiency or thrust in the regime where ingesting it is worth the
intake/duct drag. This is a distinct RBCC/ejector topology, not permission for a
normal turbojet combustor to run in an anoxic atmosphere for free.

Therefore a vehicle on an anoxic world has three different cases:

```text
ordinary turbojet/ramjet:
    no usable atmospheric oxidizer -> flameout

air-augmented rocket / ejector mode:
    onboard fuel + onboard oxidizer + ingested atmospheric working mass

pure rocket:
    onboard fuel + onboard oxidizer, intake closed/irrelevant
```

Propulsion must obtain species availability from the authoritative atmosphere
sample. The current scalar `oxygen_fraction` interface is transitional; it
must be replaced by composition-aware queries with explicit molar-vs-mass
fraction semantics.

This is important for Thessa's non-Earth environments and should apply consistently to turbojets, turbofans, ramjets, combined-cycle engines, and other atmospheric propulsion.

Status note (2026-09-23): shipped — see section 18.9.
`AtmosphereSample` now carries an `AtmosphereComposition` (normalized
mole fractions per catalog gas with derived mass-fraction queries, an
explicit molar-vs-mass basis), and `flight_condition` /
`analyze_airbreathing` read species from the sample instead of a
caller-supplied scalar — the transitional `oxygen_fraction` interface
and the `0.232`/`0.274` constants are gone. Turbojets, turbofans,
ramjets, scramjets, and ESTOC all gate through the same query, so anoxic air
flameouts and scarce oxidizer derates through the explicit species
budget. Still future from this section: the air-augmented
rocket/ejector path that spends onboard reactants while entraining
inert atmosphere as working mass (section 12), and propulsion burning
atmospheric CH4/H2 as fuel — the species queries exist; no such cycle
ships yet.

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

Ram compression replaces compressor work. The passive scramjet branch adds
supersonic combustor scheduling to the same intake/nozzle family; its current
shock-train and chemistry limits are recorded in section 18.11.

Performance should depend on inlet conditions, flight Mach number, geometry/model assumptions, reactant chemistry, thermal limits, and nozzle state.

Status note (2026-09-24): the airbreathing cycle family now includes a
scramjet operating branch. It shares the ram-compression, oxygen/thermal
combustion, fixed-geometry C-D nozzle, vehicle-mount, and analyzer paths, but
has no compressor/turbine shaft. Combustion is gated strictly above Mach 1;
the baker analyzer sweeps through Mach 8 and marks sonic/subsonic rows. The
current design point is Mach 6 at 20 km in the default atmosphere. Shock-train
geometry, finite-rate chemistry, and high-enthalpy dissociation remain outside
this engineering model.

## 12. Combined-cycle engines

ESTOC-class combined-cycle propulsion (our implementation of the
switchable air/rocket niche) is represented as multi-mode graphs with
shared hardware and alternate flow paths, not as a hard-coded `air mode
/ rocket mode` engine primitive.

Conceptually:

```text
                                +-- compressor ----------+
atmosphere -> intake -> precooler                         |
                                +-- bypass / ejector -----+-> chamber/mixer -> nozzle
                                      ^                   ^
                                      |                   |
bulk fuel ----------------------------+-------------------+
boost/coolant fuel (optional) --------+
onboard oxidizer ----------------------------------------+
```

Valves/mode logic select which paths are active. Components such as chamber,
nozzle, pumps, heat exchangers, shafts, or compressors may be shared between
modes.

The Thessa reference ESTOC direction is a dense bulk fuel (especially methane)
plus optional hydrogen used where its cryogenic heat sink is valuable. Hydrogen
is not required to be the entire fuel load: a precooler may consume H2 only at
high inlet heat load, then send the warmed H2 to the combustor instead of
discarding it. Closed cycle uses onboard LOX. This keeps the physical reason for
hydrogen without forcing the vehicle to devote SABRE-like tank volume to pure
LH2.

Automatic mode choice should ultimately be driven by the solved operating
envelope (intake recovery, compressor-inlet temperature, precooler heat flux,
shaft/work balance, useful atmospheric reactants, and net thrust), not by Mach
number alone. A Mach hysteresis band remains a useful controller policy/fallback,
not the primary law of nature.

On worlds whose atmosphere does not contain usable oxidizer, the normal
air-combustion ESTOC path must not work. A separately modelled air-augmented
rocket/ejector path may still ingest the atmosphere as working mass while
burning onboard fuel + onboard oxidizer.

This architecture should also permit turbo-rocket, ejector-rocket, RBCC, and
other hybrid cycles where future gameplay/physics justifies them.

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

- `propulsion::air`: turbojet, turbofan, ramjet, and scramjet share one
  airbreathing authoring/runtime boundary. Turbine cycles use Juno-style
  sliders (intake area/recovery, compression and bypass ratios, turbine
  temperature, fuel, afterburner, nozzle) plus the cycle
  internals Juno hides: polytropic efficiencies, turbine cooling bleed
  with rotor-bypass work split and mixing loss, customer bleed,
  part-power TIT/pressure/flow schedules, and oxygen gating for
  non-Earth atmospheres (Juno 1.4 scales jets with O2 the same way). Passive
  ramjet/scramjet cycles have no turbomachinery shaft; the scramjet additionally
  gates combustion on a supersonic combustor inlet.
- Turbine cycles run convergent nozzles; ramjets and scramjets use fixed
  convergent-divergent geometry adapted at Mach 2 / sea level and Mach 6 /
  20 km respectively, with a Summerfield separation check and a separated
  fallback to convergent-at-throat behavior (documented).
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

Closed v5 debt (audited 2026-09-21):

- Reheat gating compares against the solved design turbine-exit state,
  not TIT; `reheat_active` reads the nozzle-scaled AB flow.
- Afterburner oxygen is an explicit species budget (combustor inflow
  minus core burn plus rejoining cooling-bleed O2; customer bleed
  excluded on one basis throughout).
- Drive/work failure and intake starvation are distinct flags (vacuum
  starves with a healthy drive).
- `CompiledJet::spool_tau_s()` returns the air-path spool in both
  variants; ESTOC transition lag lives behind `transition_tau_s()`.
- Analyzer rows label the steady-running suction assumption
  (`suction_assisted`); a first-order jet spool helper bridges to the
  future shaft-state machine.
- Baker `--oxygen` overrides the analyzer O2 mass fraction (Earth 0.232
  default; Thessa ~0.274); the scalar stays an explicit adapter until
  the composition-aware atmosphere API lands.

Closed 2026-09-23 (jet shaft/starter runtime, section 18.8):

- The jet runtime now carries real shaft state: `JetMount` advances
  `JetShaftState` through `advance_jet_shaft` (starter topologies,
  light-off/self-sustain hysteresis, generator load) and evaluates the
  air path at the resulting spool speed with `lit` as the ignition
  gate, so startup, shutdown, windmill relight, and part-spool
  compressor work are actually modeled.
- The `INTAKE_DESIGN_CAPTURE_MACH` suction floor scales with actual
  spool speed: a stopped starterless engine at V=0 draws no free
  intake flow, while the steady analyzer keeps labeling its
  already-running assumption (`suction_assisted`).
- Entries 3-7 of the former debt list (spool/transition accessor
  split, drive-vs-starvation flags, reheat gating basis,
  `reheat_active` scaling, and oxygen species basis) were already
  closed by the 2026-09-21 audit above; the stale duplicates are
  removed here.

### 18.6 ESTOC combined-cycle engine (v5 shipped; v6 obligations)

Shipped v5:

- `propulsion::estoc`: air-breathing turbojet path plus closed-cycle
  rocket path sharing intake ducting, chamber, and nozzle hardware under
  our own name. Shared convergent nozzle caps rocket expansion; rocket
  chamber reuses LOX-pair thermo, OF bookkeeping, pump-feed cap,
  throat-clearance validation, and books shared hardware once.
- Automatic selection currently uses a Mach hysteresis band plus dead-air
  fallback; manual mode can override it. Per-nozzle plume states reuse the
  jet handoff; baker `[[jets]]` kinds `jet`/`estoc` expose analyzer
  Mach grids and JSON rows.

Required v6 physical model:

- Add an explicit precooler/heat-exchanger component with heat-flow,
  effectiveness, wall-temperature, coolant state, and compressor-inlet
  temperature limits. High-Mach airbreathing capability must emerge from
  this thermal budget rather than a renamed ordinary turbojet.
- Split fuel roles: dense `bulk_fuel` (CH4 is the Thessa reference) and
  optional `boost/coolant_fuel` (H2 reference). The H2 stream may be
  scheduled from required heat sink, warmed in the precooler, and then
  burned; it must have independent tank/flow bookkeeping. A pure-H2 ESTOC
  remains expressible, but is not the only topology.
- Select automatic air/rocket transition from solved envelope limits
  (precooler saturation, compressor inlet temperature/work, intake
  recovery, atmospheric reactant availability, and useful net thrust).
  Mach thresholds remain controller hysteresis/policy only.
- Add an optional air-augmented-rocket/ejector path. On an atmosphere with
  no usable oxidizer, the ordinary air-combustion path flames out; an
  ejector path may deliberately spend onboard fuel + oxidizer while using
  ingested gas as extra reaction mass.
- CLOSED 2026-09-23 (section 18.9): the free-standing oxygen scalar is
  gone — `AtmosphereSample` carries `AtmosphereComposition` with an
  explicit molar basis and derived mass fractions (Thessa 25% molar O2
  = ~27.4% by mass, Earth ~23.1%), `flight_condition` and the analyzer
  read the sample, and vehicle-baker takes `--composition` (default
  Thessa air `N2/O2/AR/CO2`) instead of a hard-coded `0.232`.
- ADOPTED 2026-09-23 for the single-spool runtime (section 18.8): ESTOC is
  authorable with an electric starter-generator, pneumatic start,
  rocket/gas-generator bootstrap, or deliberately no starter at all. A
  starterless ESTOC relights in flight once inlet-driven windmilling
  reaches light-off speed and cannot start at rest — spool-scaled
  suction means a stopped compressor draws nothing. Still deferred from
  this item: torque-level (instead of power-level) starter authoring,
  the pneumatic/rocket stored-energy pipelines (all topologies
  currently book one energy reservoir), and multi-spool attachment.
- PARTIAL 2026-09-23 (section 18.8): generator fit, rated power, cut-in
  spool speed, bounded efficiency, mass, and the electrical load now
  participate in the runtime shaft work balance — the request is capped
  at rated power, scaled by efficiency onto the shaft, below cut-in the
  generator is offline, and an overdraw bogs the spool down instead of
  being padded. Still missing: thermal limit, bus connection, and an
  efficiency map instead of the bounded scalar.
- Multi-spool/gearbox authoring must keep mode-transition dynamics separate
  from shaft dynamics. LP/IP/HP spool inertia and coupling, starter attachment,
  and optional geared fan reduction are independent of the ESTOC
  air/rocket-valve transition.

Known v5 correctness debt:

- Transition smoothing (first-tick-only application, thrust-only smoothing
  with flow/Isp jumping) and the "vacuum always rockets" contract
  contradiction were closed before 2026-09-23: smoothing now evolves the
  full flow/thermodynamic snapshot with Isp recomputed from smoothed
  flows, and manual `Air` in vacuum is the documented contract (honored,
  clean flameout).
- v5 uses one `JetFuel` for both air and rocket paths and has no
  precooler state, so it cannot yet represent the intended CH4 + H2
  tripropellant/thermal architecture.

### 18.7 Still deferred

Star/finocyl grain geometry (needs numerical perimeter burnback, not a
tweak of the port solver), tank depletion wiring into the flight loop
(the queries exist; the loop still flies baked mass), per-engine
allocation in the flight loop (authority pairs exist; the allocator
still sees one lever), transient piston/electric source startup and
propeller-shaft inertia, finite-blade propeller pitch/stall/profile maps, and
an independent free-power-turbine/prop-rotor inertia model (the current
turboprop extracts power through the shared normalized gas-generator shaft),
high-fidelity scramjet inlet/shock-train geometry and finite-rate chemistry,
and the editor UI itself (the CLI/JSON analyzer is its backend contract).

### 18.8 Jet shaft/starter runtime (section 8.1, single spool)

Shipped 2026-09-23: `propulsion::shaft` turns the section 8.1 contract
into the runtime for one normalized compressor spool, closing the v5
"no spool state" and suction-floor debt above.

Formal description:

- State (`JetShaftState`): `spool_n ∈ [0, 1]` (normalized compressor
  speed), `lit` (self-sustaining combustion), and `starter_charge_j`
  (remaining stored starter energy; constant for `StarterKind::None`).
- Inputs (`ShaftCommand` + flight condition + physics `dt_s`):
  throttle demand, starter engagement, and requested electrical
  generator load. `dt_s` is always physics seconds.
- Outputs (`ShaftTelemetry`): the next state plus starter
  active/shaft power/draw/charge, generator electrical and shaft-side
  draw, demand, capacity, friction, and net shaft power — the debug
  channel for the effect.
- Balance per step: capacity exists only while `lit` (reheat fuel is
  downstream of the turbine and never drives the shaft). Core-turbine
  capacity is booked as
  `FRAC × TURBINE_SHAFT_HEAT_FRACTION × fuel × η_comb × LHV`. A loaded
  downstream power turbine credits only its commanded extracted power to
  shaft capacity, then the matching propeller load is booked on demand
  (section 18.10).
  compressor/fan demand and bearing friction
  (`SHAFT_FRICTION_FRACTION × P_ref × n³`) always cost rotation. The
  starter adds shaft power at its topology efficiency (electric 0.85,
  pneumatic 0.70, rocket bootstrap 0.40), capped by
  `charge × η / dt` so a spent battery/air bottle really dies. The
  generator comes online above its cut-in spool, caps at rated power,
  and draws `load / efficiency` from the shaft — never padded, so an
  overdraw bogs the spool down. Integration:
  `Δn = net × dt / (P_ref × spool_tau_s)`, clamped to `[0, 1]`.
- Light-off hysteresis: throttle > 0 commands ignition, ≤ 0 commands
  shutdown; the core lights at `n ≥ light_off_n` and flames out below
  `self_sustain_n` (or on zero fuel, zero air, vacuum, or anoxic air).
  Engaging a starter with `StarterKind::None`, ramjet shaft commands,
  and non-finite/out-of-range inputs are explicit refusals (NaN fails
  closed).
- Calibration: `P_ref` (design turbine shaft reference power) and `FRAC`
  come from the compile-time design run so full-throttle sea-level
  static is an exact steady equilibrium — the anchor that keeps every
  runtime number comparable with v5. Compile refuses when
  `FRAC × TURBINE_SHAFT_HEAT_FRACTION + power_turbine_heat_fraction > 1`
  (the combined shaft booking would exceed combustor heat release) or a
  design point with no shaft-usable heat; gas-side drive feasibility is a
  separate refusal.
- Suction and schedules ride the actual spool: the
  `INTAKE_DESIGN_CAPTURE_MACH` floor scales with `spool_n` (zero at
  rest — no free intake flow without a shaft source), head with
  `1 + (ratio − 1) × n²`, and corrected-flow demand with
  `0.35 + 0.65 × n`. The steady analyzer
  (`CompiledAirbreather::operating_point`) bisects net shaft power
  over `[light_off_n, 1]` and reports the solved equilibrium; where
  nothing sustains (vacuum, anoxia, hypersonic drive limit) it reports
  `spool_n = 0.0` while evaluating the failure flags at full spool as
  a labeled already-running attempt. Ramjets skip the solve and
  report the `1.0` placeholder.
- Threading: `EstocCommand` became `JetCommand` and now also carries
  `shaft`, `starter_engaged`, and `generator_load_w`.
  `JetMount::estoc_point` advances the shaft first, then evaluates the
  air path at the resulting spool with `lit` gating ignition;
  `JetCommand::with_state` threads mode, transition snapshot, and
  shaft state into the next tick.

Known special cases and regression coverage (13 tests in
`propulsion::shaft` plus the mount-level crank test): cold start with
a fitted starter, starterless no-start at rest and refused engagement,
windmill relight at speed, light-off/self-sustain hysteresis, starter
charge depleting to a dead crank, generator load/cut-in/rating,
suction scaling with spool, steady-solve vs transient equilibrium
agreement, exact design-point equilibrium at full spool, ramjet and
authoring refusals, vacuum/anoxic never lighting, and the net-power
balance helper.

Numerical error: the steady solve performs 40 halvings, placing the
equilibrium within `(1 − light_off_n) / 2^40 ≈ 1e-12` spool — orders
below thrust-band resolution. The runtime step is first-order Euler in
`dt_s` (same scheme and caller-owned cadence as the existing spool
law).

Benchmark: `benches/propulsion.rs` reports steady-solve cost
(µs/solve) and cold-crank cost (ns/step to light-off).

### 18.9 Composition-aware atmosphere (section 10)

Shipped 2026-09-23: species availability is an authoritative atmosphere
property instead of a transitional caller scalar.

- Formal description: `GasKind` is the explicit catalog (N2, O2, Ar,
  CO2, SO2, H2, He, CH4, NH3, H2O — unknown design gases still fail at
  parse instead of silently becoming Earth air).
  `AtmosphereComposition` stores normalized **mole** fractions per
  catalog gas and derives mass fractions on query through
  `GasKind::molar_mass_kg_mol` (the single conversion source);
  constructors (`parse`, `from_mole_fractions`, `from_mass_fractions`,
  presets `thessa_air`/`earth_air`/`anoxic`) validate and normalize
  empty/NaN/negative/duplicate input. `AtmosphereConfig` owns the
  composition (design-string constructors fill it; scalar constructors
  keep the documented Earth-like default) and stamps it into every
  `AtmosphereSample`; `flight_condition` and `analyze_airbreathing` take
  no species argument at all.
- Inputs/outputs: config + altitude → sample carrying species; the core
  and afterburner query `mass_fraction(Oxygen)` for their explicit
  species budgets; `oxygen_limited` telemetry reports gating.
- Known special cases: anoxic composition (pure CO2) gives zero
  oxidizer with real inlet properties → clean flameout; scarce
  mixtures derate TIT through the species budget; Thessa (~0.274 by
  mass) and Earth (~0.231 by mass) both clear the combustor's ~7%
  requirement, so no design anchor moved.
- Regression: 4 new atmosphere tests (basis conversion + unit sums,
  validation refusals, anoxic-no-oxidizer, config→sample stamping)
  plus the re-run vacuum/anoxic flameout, scarce-O2 gating,
  drive-limit, and ESTOC mode anchors.
- Numerical error: basis conversion is direct f64 arithmetic (~1e-16
  relative); Earth's mass fraction lands at 0.2314 versus the retired
  0.232 scalar (0.3% relative), felt only under oxygen gating — which
  no sea-level anchor enters.
- Benchmark (`benches/propulsion.rs`): ~1.2 ns per mass-fraction query;
  the air analyzer sweep (55 rows of altitude × Mach) runs at ~4.5 µs
  per row including the steady spool solve, with the per-row species
  stamp inside that cost.

### 18.10 Steady shaft-power propeller drives (section 9)

Shipped 2026-09-24: a reusable ideal actuator disk can be driven by a
four-stroke piston engine, a continuous-duty electric motor, or a gas-turbine
power takeoff, and mounted on a vehicle with force/moment, dry-mass, baker,
and analyzer paths.

- State and inputs: these are steady operating-point components, not a
  transient rotor state machine. `PropellerDriveCommand` supplies normalized
  throttle and source RPM. `PropellerDriveSpec` combines a geometric annular
  disk, gearbox efficiency, a source (`PistonEngineSpec` or
  `ElectricMotorSpec`), and source-to-propeller reduction ratio.
- Propulsor law: the incompressible ideal actuator disk solves
  `P = 2 ρ A v_i (V + v_i)^2` and reports
  `T = 2 ρ A v_i (V + v_i)`. The solution is bracketed by 0 and
  `cbrt(P/(2ρA))`, then bisected 80 times. Propulsive efficiency is
  `T V / P`; it is exactly zero at static conditions. At zero density,
  thrust and absorbed power are zero with `density_limited` telemetry.
  Blade pitch/stall, profile drag, swirl, and compressibility are outside
  this ideal bound and are not hidden in fitted thrust multipliers.
- Piston source: bore, stroke, and cylinder count determine swept volume.
  The air-standard four-stroke Otto cycle uses mixture `R` and `γ`, authored
  compression/boost/volumetric/combustion efficiencies, and fuel LHV plus
  stoichiometry. Fuel is capped by sampled oxygen mass fraction. Friction
  MEP and isentropic supercharger work are deducted before brake power;
  wall-heat fraction plus friction are checked against cooling capacity,
  with a bisection throttle cap and `cooling_limited` evidence. Installed
  engine dry mass is an explicit hardware property.
- Electric source: the motor has a constant-torque region up to base speed
  (`P_rated/τ_peak`) and a constant-power region to maximum RPM. Electrical
  draw is `P_mech/η`; waste heat is `P_mech(1/η − 1)`. Cooling caps
  mechanical output and reports `thermal_limited`; installed mass and bus
  draw are explicit outputs. Standstill winding losses and battery state of
  charge are not represented by this steady component.
- Turboprop source: `TurbopropDriveSpec` compiles an airbreather with a
  positive `ShaftSpec::power_turbine_heat_fraction`, power turbine mass,
  rated full-spool RPM, reduction ratio, and the same propeller component.
  The compile-time energy check requires
  `core_FRAC × 0.5 + power_turbine_heat_fraction ≤ 1`. At runtime available
  PTO is the lesser of the authored fuel-heat budget and the downstream gas
  enthalpy/pressure bound. For hot-stream flow `ṁ`, burned-gas `cp`, turbine
  inlet-to-power-stage temperature `T₃`, and turbine efficiency `ηₜ`,
  `P_PTO,max = ṁ cp ηₜ min(max(T₃ − T_ambient, 0), T₃ ηₜ(1−ε))`, where
  `ε=10⁻¹²` keeps the pressure ratio positive. A requested load `P_PTO`
  removes `ΔT=P_PTO/(ṁ cp ηₜ)` and updates total pressure by
  `p_out/p_in=[1−ΔT/(T₃ηₜ)]^(γ/(γ−1))` before core-nozzle matching.
  `advance_jet_shaft_loaded` books that same extracted power as output and
  propeller draw in the common-shaft balance, so unused maximum capacity is
  not silently credited and requested load above either limit is refused.
  Propeller RPM follows normalized shaft speed times rated RPM divided by
  reduction ratio. This is a power-level,
  single-spool turboprop approximation, not a distinct free-power-turbine
  spool or torque/inertia model.
- Vehicle integration: `PropellerDriveMount` validates the unit thrust axis;
  `VehicleDefinition::propeller_drives_wrench_body_n` sums thrust and
  mount-station cross products and returns per-drive telemetry. The baker
  accepts `[[propeller_drives]]`, aggregates source/rotor/gearbox dry mass at
  the mount, recenters with the other vehicle masses, and exposes altitude ×
  true-airspeed analyzer rows (`--source-rpm`).
- Turboprop vehicle integration: `TurbopropMount` advances the shaft state,
  sums the loaded core-nozzle and propeller thrust at one mount station, and
  returns the next command state. The baker accepts `[[turboprops]]`, includes
  gas path, power turbine, propeller, and reduction gear mass in COM/inertia,
  and the analyzer reports combined thrust over altitude × true-airspeed;
  `--power-takeoff-fraction` requests that share of each row's available PTO.
- Known-case/regression tests: static ideal-disk thrust matches
  `(2ρAP²)^(1/3)` within `2e-14` relative; forward-flight efficiency matches
  `V/(V+v_i)`; Otto efficiency matches `1 − r^(1−γ)`; anoxic air gives zero
  piston fuel/shaft power; low cooling capacity and motor losses report
  their thermal caps; the electric gear path pins power conservation and
  vehicle force/moment/mass aggregation. Turboprop regressions pin PTO heat
  and enthalpy refusals, core exhaust-temperature/thrust reduction, shaft-load
  telemetry, mount wrench/mass, and vacuum analyzer output.
- Numerical error: 80 root halvings bound the induced-velocity interval by
  `cbrt(P/(2ρA))/2^80`; the static closed-form regression is within
  `2e-14` relative in f64. The cooling cap uses 56 throttle halvings; its
  regression leaves heat rejection no more than `1e-7 W` above capacity.
  Turbine takeoff uses closed-form temperature/pressure relations; the
  enthalpy limiter prevents `T_out < T_ambient`, and `ε` bounds pressure-ratio
  evaluation away from zero.
- Benchmark (`benches/propulsion.rs`, 11 altitudes × 4 airspeeds, 44 rows):
  electric drive analyzer 486 ns/row, piston drive analyzer 286 ns/row, and
  turboprop analyzer 1,031 ns/row in the recorded release run. These include
  atmosphere sampling, source evaluation, power-turbine extraction where
  applicable, and the actuator-disk solve.
- Remaining section 9 work: transient piston/electric source startup and
  propeller-shaft state, finite-blade pitch/stall/profile maps, and an
  independent free-power-turbine/propeller inertia model. These remain
  explicit section 18.7 debts rather than being inferred from the ideal disk.

### 18.11 Scramjet branch (section 11)

Shipped 2026-09-24: `AirCycle::Scramjet` reuses the passive airbreather cycle,
species accounting, fixed C-D nozzle, vehicle mount, and altitude/Mach analyzer.

- Formal boundary: `AirbreathingSpec` with cycle `scramjet`, compressor ratio
  exactly 1, zero bypass, no afterburner, and an inert `ShaftSpec`. There is no
  compressor, turbine, starter, generator, or power takeoff. Attempting to add
  shaft hardware or drive the shaft is refused. The legacy input field
  `turbine_inlet_temp_k` is the scheduled combustor-exit total temperature for
  ramjet and scramjet passive cycles.
- Inputs and outputs: atmosphere sample/composition, Mach/true airspeed,
  intake area/recovery class, fuel/combustor temperature, thrust/flows,
  fixed-nozzle exit state, and explicit `scramjet_limited` and
  `combustion_thermal_limited` telemetry. Ram total conditions use
  `Tt = Tamb(1+0.2M²)` and the authored intake pressure
  recovery; capture demand is anchored at the compile point. Fuel is zero at
  `M ≤ 1` and above the sonic boundary is capped by available atmospheric O₂
  and the combustor temperature target. No static suction flow is created.
  Above the authored combustor total-temperature target the fuel schedule
  closes and reports `combustion_thermal_limited`, not turbine-drive failure.
- Design/nozzle: design sizing is fixed at Mach 6 / 20 km in the default
  atmosphere. The shared fixed-geometry C-D nozzle is adapted there and uses
  the existing area/Mach inversion and separation telemetry. No compressor or
  turbine mass is booked; intake, combustor, nozzle, and common hardware mass
  still follow the airbreather geometry/flow fits.
- Vehicle/analyzer: `JetMount` takes the passive path without advancing a
  nonexistent shaft and rejects starter/generator commands. Analyzer rows
  propagate both limit flags; vehicle-baker uses Mach 0/1/2/4/6/8 for
  scramjets (`M` flags the sonic boundary and `T` flags over-temperature
  inlet rows) and its JSON output carries the same telemetry.
- Known special cases/regressions: at Mach 1 the engine reports limited,
  unlit, zero fuel and zero thrust; at Mach 6 / 20 km hydrogen combusts, the
  fixed nozzle exits supersonically, and thrust is positive. First-law
  telemetry check bounds useful power plus exhaust kinetic power by chemical
  fuel power plus inlet kinetic power. Anoxic Mach 6 air is oxygen-limited;
  at Mach 8 / 20 km the total-temperature schedule is closed and the
  thermal-limit flag is set without setting `drive_limited`. Starter/
  afterburner combinations are refused; a mounted scramjet produces thrust
  and refuses shaft accessory commands.
- Numerical error: total inlet conditions and the sonic gate are closed-form;
  nozzle area/Mach inversion uses the already-tested bisection-backed solver
  (see section 18.5). The Mach boundary is exact at `M=1`; this branch adds no
  new iterative solver.
- Benchmark (`benches/propulsion.rs`, 11 altitudes × 6 Mach values): the
  release analyzer measured 520.9 ns/row on this machine, including
  low-speed boundary rows, the active supersonic range, and declared-vacuum
  samples.
- Remaining scramjet fidelity belongs to the explicit section 18.7 debt:
  inlet/shock-train geometry and finite-rate chemistry/dissociation, not an
  untracked extension of the current ram-compression bound.
