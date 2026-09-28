# Procedural propulsion systems

Status: design baseline with a shipped backend (`thessa-sim-core::propulsion`
+ `feed`): liquid chemical rockets + solid motors, isentropic nozzle core
with mixture sensitivity, cycle/feed bounds, geometry-derived mass, spool
runtime, altitude analyzer, vehicle mounts, tanks and feed lines, RCS and
nuclear thermal models, multi-chamber systems, air-breathing jets
(turbojet/turbofan/ramjet/scramjet) with a single-spool default and an explicit
two-spool LP/HP turbofan shaft train, starter topologies,
light-off/self-sustain, relight, and generator load,
composition-aware atmosphere queries (section 10), and the ESTOC combined-cycle
engine, plus piston/electric propeller drives and a
stateful, heat-budgeted turboprop takeoff path on a reusable ideal actuator
disk (section 9), steady electric spacecraft thrusters (section 13), and
continuous/pulsed fusion propulsion (section 14).
Fixed-step resource coupling now covers liquid/solid rockets, air-breathing
jets and APUs, electric and shaft-power drives, RCS, fuel cells, and continuous
and pulsed fusion. Compatible reachable tanks share one allocation and commit
with moving-mass/inertia updates; APUs and fuel cells also join the vehicle
electrical bus, and APU pneumatic bleed can crank fitted jet starters. Transient
piston/electric source and prop-shaft state, finite-blade propeller maps, an
independent free-power-turbine spool, high-fidelity scramjet shock-train and
finite-rate chemistry, fusion confinement and transient thermal fidelity, and
the editor UI remain future work (see section 18).

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

Historical status snapshot (2026-09-24; superseded below): the single-spool
runtime had landed, while multi-spool/gearbox coupling, torque-rated hardware,
tank-backed pneumatic/rocket-bootstrap consumables, generator thermal limits,
and ordinary jet generator bus export were still open. The implemented state
is recorded in the 2026-09-28 update and section 18.8 below.

Update (2026-09-28): tank-backed pneumatic/rocket-bootstrap draws,
torque-rated starter/generator hardware, generator efficiency maps and local
thermal limiting are now implemented, and jet-mounted generator output joins
APU generation on the shared bus. A two-spool LP/HP turbofan with independently
integrated rotors, a geared fan, torque-rated accessories on either spool, and
a coupled steady-state solver is implemented below. Three-spool/independent
free-power-turbine topologies, clutching, and generator thermal-node integration
remain open.

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

Automatic mode choice uses the solved operating envelope (intake recovery,
compressor-inlet temperature, precooler heat flux, shaft/work balance,
atmospheric reactants, and net thrust). A Mach hysteresis band only delays a
return from rocket mode while the air path remains viable; it is not the
primary transition law.

On worlds whose atmosphere does not contain usable oxidizer, the normal
air-combustion ESTOC path must not work. A separately modelled air-augmented
rocket/ejector path may still ingest the atmosphere as working mass while
burning onboard fuel + onboard oxidizer. The ESTOC v6 ejector uses free-stream
capture area and motive-jet kinetic energy; zero-speed aspiration remains a
higher-fidelity geometry debt.

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

Status note (2026-09-24): the sim-core now compiles gridded-ion, Hall,
magnetoplasmadynamic (MPD), resistojet, and arcjet designs, mounts them on a
vehicle, and books power-processor, active hardware, and radiator mass. Each
steady operating point is bounded by its electrical-power, feed-flow, current,
and radiative heat-rejection limits.

The vehicle-wide shared electrical bus is documented in
[`09_ELECTRICAL_POWER.md`](09_ELECTRICAL_POWER.md). In the authoritative runtime,
electric thrusters, electric propeller drives, and fusion drivers/chargers use
same-name bus consumers; their actual bus allocations bound their operating
points. Direct operating-point commands remain available to standalone model
callers.

- Gridded-ion/Hall exhaust velocity follows singly charged particle energy,
  `ve = sqrt(2 e V / mi)`. Ion current follows particle throughput, and feed
  flow is capped by rated power, accelerator current, propellant utilization,
  and radiator duty. The Hall annular channel's field coil is included in the
  structure mass estimate.
- MPD thrust uses the reduced self-field Maecker relation,
  `T = μ0 I² ln(ra/rc)/(4π)`, bounded by arc voltage/current, bus power, feed
  flow, and jet kinetic energy. This is an engineering relation, not an
  electrode/plasma simulation.
- Resistojet/arcjet designs use constant-γ gas heat capacity, heater efficiency,
  maximum exhaust temperature, and nozzle efficiency:
  `ve = sqrt(2 ηn cp (Texhaust - Tinlet))`. Electrical input covers gas
  enthalpy; the nozzle's residual exhaust enthalpy leaves with the propellant.
- All families report thrust, effective Isp, consumed flow, electrical draw,
  jet kinetic power, residual exhaust-internal power, waste heat, radiator
  capacity, current, and active limiting flags. The energy telemetry closes
  `Pelec = Pjet + Pexhaust-internal + Qwaste`; radiator heat is only local
  conversion loss, not residual exhaust enthalpy. Radiator capacity is
  `εσA(Trad⁴ - Tbackground⁴)`.
- This is a steady propulsion operating-point model: plasma kinetics,
  multi-charge states, electrode erosion, plume interaction, Hall field
  topology, and engine-internal transients are not modeled. Mounted vehicle
  runtime separately handles reachable propellant depletion and fixed-step bus
  dispatch/storage.

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

Status note (2026-09-24): sim-core now compiles a reduced continuous torch
and a distinct event-driven pulsed engine, each with vehicle mounts, mass
aggregation, baker authoring, heat/power telemetry, and thrust/moment
integration. These models establish the component/resource boundary; they do
not claim a fusion-plasma or confinement simulation.

`FusionReaction` carries reaction energy and charged-product energy share for
D-T (17.6 MeV, 3.5 MeV charged), equal-branch D-D (3.65 MeV average, 2.425 MeV
average charged), D-He3 (18.3 MeV charged), and p-B11 (8.68 MeV charged).
Specific energy is `Q × N_A / molar_mass_of_reactants`; the compiled design
uses that value to derive fusion fuel consumption, not a reaction-name thrust
lookup.

For a continuous torch, `fusion_gain = fusion_power / driver_power`. The
reaction power is limited by driver availability, rated fusion power, and the
radiator heat budget. With charged fraction `fc`, plasma coupling `ηc`, and
magnetic-nozzle efficiency `ηn`,

```text
Pjet       = Pfusion × fc × ηc × ηn
Pexhaust   = Pfusion × fc × ηc × (1 − ηn)
Qlocal     = Pdriver + Pfusion × (1 − fc) + Pfusion × fc × (1 − ηc)
Pfusion    = fuel_flow × reaction_specific_energy
thrust     = sqrt(2 × Pjet × (fuel_flow + working_flow))
```

The ideal mixed exhaust carries reaction products plus commanded working
fluid. Radiator capacity is `εσA(Trad⁴ − Tbackground⁴)`, and runtime clips
fusion power when `Qlocal` would exceed it. Reactor dry mass follows rated
fusion power / reactor specific power; nozzle walls, a field coil sized from
`B = μ0 n I` and conductor current density, and radiators add geometry/material
mass at the mount. The model does not solve confinement, plasma temperature,
reaction-rate kinetics, ash separation, neutron shielding, or a magnetic-field
map.

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

`PulsedFusionSpec` instead authors fuel and working-fluid mass per shot,
fusion gain, driver charge power, finite buffer capacity/specific energy,
pulse duration/frequency, coupling/nozzle efficiency, and pulse-chamber/coil/
radiator geometry. State is `(pulse_phase, stored_driver_energy, cumulative
shots)`. Each physics-step advance charges the finite buffer, fires only when
both cadence and driver-energy conditions are met, and returns an impulse
total for the step. The cadence lower bound is the maximum of authored period,
pulse duration, and `waste_heat_per_pulse / radiator_capacity`; the disarmed
state may recharge without firing or advancing the firing phase.

```text
Ishot = sqrt(2 × Ejet_per_shot × (fuel_mass + working_mass))
Tstep = sum(Ishot) / dt
Ebus + Efusion = ΔEbuffer + Ejet + Eexhaust-internal + Qwaste
```

The radiator cadence is an average-duty limit: each shot reports its heat
energy and step-average heat rate, while this reduced model has no transient
thermal-mass node or shot-temperature solver. The thermal graph must consume
the event heat energy when that integration is added. Driver storage is
lossless here; charging efficiency, pulse-unit depletion, shock coupling,
mechanical pusher plates, radiation damage, and fragmentation are fidelity
debts rather than hidden multipliers.

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

Shipped in `crates/propulsion/src/propulsion/` (MIT engine crate, no
Bevy/Tokio/wgpu).

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
  aggregation, per-mount operating-point allocation, and mass-flow queries.
- Flight-authority liquid-engine and multi-chamber-system draws use each
  operating point's kg/s and mixture ratio, reserve compatible reachable
  mixed/component tanks through assembly resource connectivity, and update
  tank inventory, vehicle mass, inertia, and body-frame COM each fixed tick.
  Solid motors use their compiled burn curve and remaining-grain query. Named
  tanks support explicit, mass-conserving transfers over an open resource path;
  no authored pipe geometry or detailed pipeline solver is required.
  `flight-authority/benches/resource_allocation.rs` measures the allocator on a
  32-engine/64-tank 120-Hz vehicle workload; the current optimized local run
  measured 4.35–4.56 us per allocation.
- The fixed-step authority also plans mounted APU, jet/ESTOC, RCS, fuel-cell,
  electric-thruster, fusion, piston/electric propeller-drive, and turboprop
  demands against that allocator. Each limited command is reduced and its
  operating point re-evaluated before forces or source output are accepted.
  Fuel-cell hydrogen/oxygen and external propulsion demands share the tank
  transaction; committed tank and grain draws update mass, inertia, and COM.
  APU generation and dispatched fuel-cell output enter the same ideal bus.
  Electric thrusters and bus-powered fusion/electric propeller loads use
  same-name bus consumers; absent consumers supply no electrical power.
  Named `resource_feed_ports` routes send a generic consumer to an assembly
  engine-feed endpoint without an authored pipe network or line-flow solver.
- A pneumatic starter request is derived from the fitted starter's rated
  inlet power. Lit APUs share that bleed request as a physical shaft load, and
  only their delivered bleed is passed to jet/turboprop starter operating
  points. External bleed does not consume onboard starter reserve; any
  remaining starter demand may use the fitted reserve.
- Plume handoff: `EnginePlumeState` maps field-for-field into
  `plume-core` `PlumeSource` through the single `engine_plume_source`
  choke point, with no new cross-crate dependency.

#### Runtime resource contract

- State is `VehicleResourceState`: one remaining-mass value per installed tank,
  plus one burn clock and ignition bit per solid motor. The fixed-step runtime
  also carries electrical-bus, starter/shaft, and pulsed-fusion operating state.
- Inputs include mounted rocket/chamber commands, ambient pressure, fixed-step
  duration, installed-consumer operating-point demands, and tank-to-feed-port
  reachability through the current assembly topology. Outputs include allocated
  wrenches and actual commands, per-tank draw, consumer scales/flows,
  fuel-limited status, updated subsystem state, and the moving-mass frame shift.
- A liquid/system operating point supplies each requested kg/s. For a resource
  group, requested mass is `sum(mass_flow_i * dt)`. Compatible mixed inventory
  is drawn first; split bipropellant inventory is constrained by the smaller
  available oxidizer/fuel amount at each engine's mixture ratio. A shared
  availability scale reduces each engine's requested throttle proportionally;
  engines that would fall below their minimum stable throttle are shut off and
  the remaining demand is reallocated. Pure working-fluid engines match the
  compiled fluid identity rather than the plume-label propellant pair. Generic
  installed consumers receive one common availability scale across all their
  reactants, and explicit feed ports restrict allocation to topologically
  reachable tanks.
- Commit conserves total mass: tank and grain mass changes update vehicle mass,
  first moment, and inertia about the new center of mass, then shift all baked
  body-frame geometry and the inertial state to that frame. The commit returns
  the frame shift; tank transfers conserve mass and enforce source inventory,
  destination capacity, resource identity, and open assembly connectivity.
  Fixed-step propellant inventory, electrical/APU/jet/fusion/turboprop state,
  parachute state, and free-flight gear actuators are committed only after the
  integrated endpoint passes the solver guard; rejected ticks do not consume
  propellant or advance those states.
- Regression cases pin liquid demand/tank-draw closure within `1e-10 kg` for
  0.1-second steps, summed thrust within `1e-8 N`, vehicle mass change within
  `1e-9 kg` (solid grain: `1e-8 kg`), and transfer conservation within
  `1e-12 kg`. These are floating-point regression tolerances, not uncertainty
  estimates for the underlying engine constitutive models. The assembly test
  verifies that a closed hatch blocks feed.

### 18.2 Tails closed after v1

- Mixture-ratio sensitivity: per-pair (ratio, chamber temp, gamma, gas
  constant) tables with piecewise-linear interpolation, hard refusal
  outside the modeled range, reference point reproduced exactly. Tables
  are representative CEA trends; refine with project CEA runs.
- Aerospike contour (linear): near-axial divergence, altitude
  compensation down to base drag on the plug base, separation flag
  never trips by design. Sea-level thrust holds within 3% of vacuum.
- Shaped solid thrust: per-segment stepped circular ports plus star and
  finocyl profiles solve port burnback on a coupled time-stepped trace;
  boost/sustain and geometry-error envelopes are pinned by tests.
- Thermal interface data: chamber stagnation power, exhaust kinetic
  power (ordering pinned), nozzle wall area; the graph hookup waits for
  a runtime thermal graph to exist.
- Solid depletion queries: remaining grain vs burn clock; the flight loop
  now applies grain mass and point-mass inertia changes along that clock.
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
  pressure exactly through choked flow). The mounted fixed-step path currently
  evaluates cold-gas mounts at their authored rated inlet pressure; tank
  blowdown and line-pressure coupling remain future work.
- Pulse physics: triangular valve rise with propellant booked over the
  full open time, so short pulses lose effective Isp causally; minimum
  impulse bit, hydrazine Isp band (210-235 s), and N2 Isp band (65-85 s)
  pinned by test. Mounted `RcsCluster` delivers force/moment impulses
  and PWM-average wrenches (opposed-pair pure couple pinned).
- Mounted `RcsMount`s also join the authoritative flight allocator: control
  requests become per-nozzle pulse duties, compatible reachable tank inventory
  limits the delivered wrench, and pulse mass commits through the vehicle
  moving-mass/inertia path.
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

### 18.6 ESTOC combined-cycle engine (v6 shipped, reduced model)

Shipped v5 foundation:

- `propulsion::estoc`: air-breathing turbojet path plus closed-cycle
  rocket path sharing intake ducting, chamber, and nozzle hardware under
  our own name. Shared convergent nozzle caps rocket expansion; rocket
  chamber reuses LOX-pair thermo, OF bookkeeping, pump-feed cap,
  throat-clearance validation, and books shared hardware once.
- Per-nozzle plume states reuse the jet handoff; baker `[[jets]]` kinds
  `jet`/`estoc` expose air-path Mach grids, and ESTOCs additionally emit
  steady `EstocAltitudePoint` rows with selected mode, solved spool fraction,
  fuel split, oxidizer/air flow, precooler duty, and saturation to text/JSON
  analyzers.

- `EstocPrecoolerSpec` authors rated heat flow, effectiveness, pressure
  recovery, compressor-inlet temperature limit, finite wall mass/heat
  capacity and temperature bounds, coolant inlet/outlet temperatures,
  coolant specific heat, and maximum coolant flow. Runtime threads wall
  temperature and coolant outlet temperature through `EstocTransient`.
- Exchanger heat rate is bounded by effectiveness and the minimum hot/cold
  capacity rate, rated duty, the compressor-inlet temperature limit, coolant
  flow/enthalpy, and remaining wall thermal capacity. Wall energy advances as
  `ΔTwall = Qwall × dt / (mwall cpwall)`. Compressor work uses the cooled
  total temperature and exchanger pressure recovery, and that same conditioned
  shaft balance drives spool integration. Unmet cooling leaves
  `precooler_saturated` visible in the operating point. The steady analyzer
  does not credit finite wall storage; only continuous coolant/boost-fuel
  capacity is available at equilibrium.
- Fuel roles are independent: `bulk_fuel` supplies the conventional air and
  rocket paths (defaulting to legacy `air.fuel`), and optional
  `boost_coolant_fuel` passes through the precooler and is burned. Hydrogen is
  the reference coolant/boost fuel; methane remains a dense bulk-fuel option.
  `AirOperatingPoint` and `EstocPoint` report total, bulk, and boost flows
  separately. Warmed coolant sensible heat returns to the combustor energy
  balance; boost chemical energy displaces bulk-fuel energy at the scheduled
  turbine-inlet target, with combined oxygen demand applied to both streams.
  Oxygen-limited operation scales boost-coolant flow and its returned heat
  together, preserving the coolant outlet-temperature limit.
  The compile-time rocket path uses the bulk fuel's LOX pair.
- Automatic mode evaluates the actual cooled cycle: it requires usable
  oxygen, delivered air, a lit/non-drive-limited core, compressor temperature
  within its authored limit, and positive net thrust. It otherwise selects
  rocket, or selects the ejector when oxygen is absent and captured flow is
  nonzero. `switch_mach_hi` is an upper policy bound while the air path is
  viable; `switch_mach_lo` supplies rocket-to-air return hysteresis.
  The steady analyzer solves the shaft equilibrium with the conditioned cycle
  and exchanger pressure recovery; if the shaft cannot sustain itself, it
  reports the selected fallback mode and a zero spool fraction. Finite wall
  storage is not credited in that equilibrium.
- Optional `EstocEjectorSpec` sizes inlet area, mixing length, shroud density
  and thickness, and mixing efficiency. Capture is `mdot_air = rho A V∞`; motive
  kinetic power is mixed over rocket exhaust plus captured air, and thrust
  closes mixed-stream momentum against inlet momentum while retaining the
  rocket nozzle pressure term. Capture uses composition-aware ambient density.
  Ejector dry mass follows shroud geometry. A
  stopped craft with no captured flow falls back to the closed-cycle rocket.
- Baker `[jets.precooler]`, `[jets.ejector]`, `bulk_fuel`, and
  `boost_coolant_fuel` fields compile through normal mass/COM baking.
  Regressions pin heat/flow limits, wall-state advancement, separate CH4/H2
  bookkeeping, O2-gated ejector selection, ejector thrust against rocket-only
  operation, zero-capture fallback, invalid authoring, and TOML mass baking.
- Numerical closure: the heat-partition solve bisects its feasible heat-flow
  bracket 48 times (at the 20 MW reference duty, the final bracket is below
  `7.2e-8 W`); the regression energy balance closes within `1e-8` relative.
  Ejector momentum uses the captured-flow and motive-jet kinetic-energy
  equations directly; its explicit `mixing_efficiency` bounds the unresolved
  mixing loss rather than fitting thrust. The release benchmark measured
  `241.5 ns/row` over six Mach points in Earth air and six in nitrogen-only
  atmosphere on this machine.

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
  suction means a stopped compressor draws nothing. Pneumatic and
  rocket-bootstrap starters may now draw an explicitly selected stored
  vehicle resource using authored recoverable specific energy. Starter
  draw participates in the same reachable-tank allocation and moving-mass
  commit as engine flow. Optional torque-rated starters use the authored
  rotor inertia and design speed; the existing power/normalized-spool law
  remains the default for single-spool engines. The two-spool geared-turbofan
  extension is implemented in section 18.8 below; multi-spool ESTOC transitions
  remain unsupported.
- PARTIAL 2026-09-23 (section 18.8): generator fit, rated power, cut-in
  spool speed, mass, and requested electrical load participate in the
  runtime shaft work balance. Efficiency may be a scalar or a piecewise
  linear normalized-speed map. Optional winding/casing heat capacity and
  conductance limit output against the maximum temperature; optional torque
  ratings additionally cap output at low shaft speed. APU and ordinary
  jet-mounted generator output are dispatched onto the shared vehicle bus.
  The thermal state is currently local to `JetShaftState`, not part of the
  vehicle thermal-node graph.
- A two-spool LP/HP geared turbofan now keeps mode transitions separate from
  shaft dynamics, with a selected-spool starter and generator. General LP/IP/HP
  shaft networks, clutches, and multi-spool ESTOC air/rocket-valve transitions
  remain future work.

Known v5 correctness debt:

- Transition smoothing (first-tick-only application, thrust-only smoothing
  with flow/Isp jumping) and the "vacuum always rockets" contract
  contradiction were closed before 2026-09-23: smoothing now evolves the
  full flow/thermodynamic snapshot with Isp recomputed from smoothed
  flows, and manual `Air` in vacuum is the documented contract (honored,
  clean flameout).
- CLOSED 2026-09-24: separate bulk and boost/coolant fuels, finite precooler
  state, envelope-driven transitions, and an anoxic ejector are implemented.
  The model has no local heat-rejection edge from the precooler wall, no
  two-phase H2 property table, and no steady coolant recirculation (the
  streamed coolant is the boost fuel). These thermal/material refinements
  are explicit fidelity debts, not hidden mode coefficients.

### 18.7 Still deferred

Transient piston/electric source startup and propeller-shaft inertia,
finite-blade propeller pitch/stall/profile maps, and an independent
free-power-turbine/prop-rotor inertia model (the current turboprop extracts
power through the shared normalized gas-generator shaft), high-fidelity
scramjet inlet/shock-train geometry and finite-rate chemistry, local casing
failure after shaped-grain web breakthrough, and the editor UI itself (the
CLI/JSON analyzer is its backend contract) remain future work.

### 18.8 Jet shaft/starter runtime (section 8.1)

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
  `charge × η / dt` so a spent battery/air bottle really dies. Tank-backed
  pneumatic/rocket starters derive their available reservoir from reachable
  tank inventory and charge it in the shared fixed-step resource transaction
  at the authored `specific_energy_j_kg`. If a starter torque rating is
  present, the rotor integrates `I dω/dt = τ_engine + τ_starter − τ_load`,
  with starter work bounded by the same stored-energy budget; otherwise the
  normalized power-law integration above remains active. APU-supplied
  pneumatic input can supplement the onboard reserve up to rated starter
  power; the supplying APU books bleed as shaft load, and accepted external
  power is reported separately from reserve draw. The
  generator comes online above its cut-in spool, caps at rated power,
  and draws `load / efficiency` from the shaft — never padded, so an
  overdraw bogs the spool down. Efficiency may be interpolated from a
  normalized-speed map; optional local heat capacity/conductance limits the
  electrical request to keep its implicit temperature step under the maximum.
  Torque-rated generator hardware additionally caps low-speed electrical
  output by `τ_max ω η`. With power-level hardware, integration is
  `Δn = net × dt / (P_ref × spool_tau_s)`, clamped to `[0, 1]`; explicit
  rotor-inertia hardware integrates angular speed from net torque instead.
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

Tank-backed starter TOML extends the existing `[jets.shaft.starter]` block:

```toml
[jets.shaft]
design_speed_rad_s = 1200.0 # required with torque-rated starter/generator hardware
rotor_inertia_kg_m2 = 0.8

[jets.shaft.starter]
kind = "pneumatic" # or "rocket-bootstrap"
power_w = 200000.0
resource = "nitrogen" # pneumatic: nitrogen/helium; bootstrap: authored stored propellant
specific_energy_j_kg = 500000.0
mass_kg = 12.0
maximum_shaft_torque_nm = 800.0 # optional torque-level actuator rating
```

With `resource` present, `charge_j` must be zero. Runtime energy availability
is derived from compatible tanks reachable through the selected feed route;
the specific energy is an explicit hardware property, not inferred from the
resource name. A torque-rated accessory requires both shaft design speed and
rotor inertia. `maximum_shaft_torque_nm` remains optional on starter and
generator independently, preserving the power-based shaft integrator where no
torque-rated hardware is installed.

Generator maps and thermal limits use the same `[jets.shaft.generator]`
authoring block. `efficiency_map` is an ordered array of `{ spool_n,
efficiency }` points. `thermal` contains `heat_capacity_j_k`,
`conductance_w_k`, `initial_temperature_k`, and `maximum_temperature_k`.
The runtime interpolates conversion efficiency, computes waste heat, and
limits requested electrical output so the implicit local thermal step stays
within the authored maximum temperature.

```toml
[jets.shaft.generator]
fitted = true
power_w = 50000.0
efficiency = 0.85 # fallback scalar; the map overrides it when present
efficiency_map = [
  { spool_n = 0.4, efficiency = 0.72 },
  { spool_n = 1.0, efficiency = 0.91 },
]
maximum_shaft_torque_nm = 120.0 # optional; enables torque-rated shaft behavior
cut_in_spool_n = 0.45
mass_kg = 25.0

[jets.shaft.generator.thermal]
heat_capacity_j_k = 80000.0
conductance_w_k = 35.0
initial_temperature_k = 293.15
maximum_temperature_k = 430.0
```

Known special cases and regression coverage in `propulsion::shaft` plus the
mount-level crank test: cold start with
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
(µs/solve), cold-crank cost (ns/step to light-off), and the two-spool
transient runtime (ns/step).

#### Two-spool geared turbofan extension

The 2026-09-28 runtime adds one explicit two-rotor topology for turbofans:

- `JetShaftState.spool_n` is normalized HP compressor speed and
  `low_pressure_spool_n` is normalized LP/fan speed. Turbine work is split by
  `high_pressure_turbine_power_fraction`; the HP rotor pays core-compressor
  work and the LP rotor pays fan work plus gearbox loss.
- The fan speed ratio is `fan angular speed / LP rotor angular speed`. The
  fan's inertia is reflected to the LP shaft as
  `J_LP + J_fan × ratio²`. Gear efficiency increases LP shaft demand by
  `1 / efficiency`; fan pressure rise follows LP speed, core compression
  follows HP speed, and intake capture uses the bypass-weighted LP/HP speed.
- Both rotors integrate their own torque balance with their authored design
  speed and inertia. A starter and generator require torque ratings in this
  topology and may select `attached_spool = "low_pressure"` or
  `"high_pressure"`; starter energy, generator efficiency, local generator
  temperature, and torque limits are evaluated at the selected rotor speed.
- The authored turbine split must balance the compiled sea-level static design
  point on both rotors. For authoring, calibrate the HP share against
  `P_HP / (P_HP + P_fan / gear_efficiency)` from the corresponding single-spool
  design balance; compile rejects a split that does not close each rotor's
  work budget. The steady operating-point solver alternates bounded bisections
  for the coupled HP and LP equilibria, each with 36 halvings and a 32-sweep
  cap; the tested off-design root closes both power balances within
  `1e-5 × capacity`.
- The single-spool runtime remains the default and is unchanged when
  `multi_spool` is omitted. This topology currently supports only a geared
  turbofan with one LP fan rotor and one HP core rotor. Three-spool engines,
  clutches, multiple accessory machines per shaft, free-power-turbine shafts,
  and multi-spool ESTOC mode transitions are not implemented.

Example authoring (the turbine split is engine-specific and must close the
design work balance):

```toml
[jets.shaft.multi_spool]
low_pressure_design_speed_rad_s = 500.0
low_pressure_rotor_inertia_kg_m2 = 4.0
fan_rotor_inertia_kg_m2 = 2.0
high_pressure_design_speed_rad_s = 1000.0
high_pressure_rotor_inertia_kg_m2 = 2.0
high_pressure_turbine_power_fraction = 0.724245263613
fan_gear_speed_ratio = 0.5
fan_gear_efficiency = 0.95

[jets.shaft.starter]
kind = "electric"
power_w = 12000.0
charge_j = 2000000.0
maximum_shaft_torque_nm = 60.0
attached_spool = "low_pressure"
mass_kg = 2.0
```

Regression coverage checks independent rotor acceleration/loading, LP-mounted
starter and generator response, gear-reflected inertia, fan-flow response,
off-design dual-rotor equilibrium, mount state threading, and asset
serialization round-trip. `vehicle-baker` compiles the authoring example; the
`propulsion` bench reports the multi-spool transient step and coupled
off-design steady-solve costs on the same workload as the single-spool shaft.
The latest release run measured `339 ns/step` and `21.29 µs/solve` for the
two-spool turbofan on this machine; benchmark results are hardware-dependent.

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

### 18.12 Star and finocyl solid-grain burnback

Shipped 2026-09-24: `SolidGrainGeometry::{Star, Finocyl}` compile noncircular
grain ports into the existing pressure-coupled APCP burn trace. Circular BATES
and stepped-channel ports retain their analytic-radius path.

- Formal geometry: star ports are alternating root/tip regular polygons;
  finocyl ports are a central core joined to evenly spaced radial fin slots
  with a short polygonal tip arc. Existing `core_radius_m` is the star root or
  finocyl center radius; stepped `segment_core_radii_m` overrides it per
  segment. Lobe/fin tips must lie inside the cylindrical grain and outside
  every configured root.
- Burnback: a 128×128 cell-center signed-distance field is built inside the
  cylindrical grain section. Port area is the count of cells with distance
  `≤ web`; the perimeter is a four-direction Crofton crossing estimate.
  These are reduced to 400 linearly interpolated web stations per segment.
  Authoring refuses lobe depth, fin width, or outer web thinner than two
  grid cells at the selected grain radius, keeping profiles inside the
  validated resolution envelope.
  Each trace step evaluates per-segment perimeter × segment length plus the
  existing optional burning end faces, then solves the shared Saint-Robert
  equilibrium `Pc=[Ab·a·ρ·c*/At]^(1/(1−n))` and regresses each segment by
  `a·Pcⁿ·dt`. This is a sampled geometric burn-front model, not a pressure or
  thrust curve fit.
- Telemetry: every `BurnPoint` records total burning surface area, total open
  port area and perimeter, web, chamber pressure, mass flow and thrust.
  Compiled-solid state carries the selected grain profile. Baker TOML accepts
  `grain_geometry = { kind = "star", tip_count = 6, tip_radius_m = 0.30 }`
  or `grain_geometry = { kind = "finocyl", fin_count = 8,
  fin_tip_radius_m = 0.32, fin_width_rad = 0.24 }`; omission remains the
  legacy circular profile.
- Regressions: initial raster port area is within 2.5% and Crofton perimeter
  within 8% of exact polygon area/perimeter; trace-integrated propellant mass
  is within 7% of the exact cross-section volume. Tests also pin monotonic
  open area, zero terminal burn surface, impulse/Isp consistency, geometry
  validation, and TOML baking for both families.
- Numerical error: geometry tolerances above are envelope checks against
  closed-form polygon geometry at the production grid. Time integration
  continues to use the existing burn-trace step rule; pressure is recomputed
  from the sampled perimeter rather than interpolating thrust directly.
- Benchmark (`benches/propulsion.rs`): release compile measured 4.35 ms for
  a four-segment star and 4.73 ms for a four-segment finocyl, versus 126 µs
  for analytic four-segment BATES. The distance-field path is hangar compile
  work; runtime replays the precompiled burn curve.
- Limitation: the current grain model completes the sampled burn front at
  full cross-section depletion; local case exposure/rupture before then is
  not yet represented as a structural failure event (section 18.7 debt).

### 18.13 Electric spacecraft thrusters (section 13)

Shipped 2026-09-24: `propulsion::electric` compiles five steady electric
thruster designs and installs them through `ElectricThrusterMount`.

- Formal boundary: `ElectricThrusterSpec` carries species, hardware topology,
  rated bus power/feed flow, power-processor specific power, active structure
  material/thickness, radiator area/emissivity/temperature/areal mass, plasma
  ionization efficiency, and propellant inlet temperature. The baker accepts
  `[[electric_thrusters]]` with a tagged `design` table (`gridded-ion`,
  `hall-effect`, `magnetoplasmadynamic`, `resistojet`, or `arcjet`). The
  installed dry mass is PPU rating / PPU specific power + geometry-derived
  active hardware + radiator mass; vehicle mass, inertia, final COM, and
  station wrench all include the mount.
- Species: xenon, krypton, argon, iodine, nitrogen, hydrogen, ammonia, and
  water carry molar mass, constant-γ gas heat capacity, and first-ionization
  reference energy. Ion accelerators assume singly charged species and a
  constant propellant-utilization fraction. Their particle speed is
  `sqrt(2 e V / mi)`; beam current is particle throughput × elementary charge.
  Electrical ionization/acceleration efficiencies divide the respective
  energy requirements, and the feed is capped by bus power, current, rated
  flow, and radiator rejection.
- Hall hardware is an annular channel. The reduced performance law is the
  same electrostatic particle-energy relation as the gridded accelerator;
  channel geometry and magnetic field size the channel plus copper field-coil
  mass (`N·I = BL/μ0`, conductor section from authored current density), while
  the authored discharge-current rating bounds the beam. Coil excitation
  power, electron transport, and field topology are expressly outside this
  version.
- MPD performance uses the self-field Maecker relation
  `T = μ0 I² ln(ra/rc)/(4π)`. Arc-voltage, current rating, bus power,
  ionization energy, flow, and the jet-power efficiency jointly bound the
  selected discharge current. Geometry-based electrode volume contributes
  device mass.
- Resistojet/arcjet use `cp = γR/(γ−1)` and gas enthalpy rise
  `Δh = cp(Tmax−Tinlet)`: `ve = sqrt(2 ηnozzle Δh)` and required input per
  mass is `Δh/ηheater`. Arcjet current/voltage add an electrical cap. The
  residual enthalpy is exhaust-internal power, not radiator heat.
- Thermal and energy accounting: radiator capacity is
  `εσA(Trad⁴−Tbackground⁴)`. Every returned point carries thrust, effective
  Isp, feed flow, input power, jet kinetic power, exhaust-internal power,
  waste heat, radiator capacity, discharge current, and power/flow/current/
  thermal flags. Regression pins the first-law closure
  `Pelec = Pjet + Pexhaust-internal + Qwaste` and the command/rating guards.
- Known-case anchors: a 1 kV xenon ion beam has the closed-form particle exit
  speed from singly charged xenon mass; its effective vehicle Isp includes
  neutral propellant left by the authored utilization. MPD force equals the
  Maecker current-squared relation; electrothermal speed matches the
  constant-γ enthalpy/nozzle expression. A deliberately undersized Hall
  radiator caps mass flow exactly at its radiative heat envelope. TOML
  regression bakes an ion thruster, aggregates dry mass, recenters its mount,
  and verifies its off-origin force/moment.
- Numerical error: accelerator speed, MPD force law, gas enthalpy, and
  black-body radiator capacity are closed-form in this reduced model; runtime
  adds no numerical iteration except a bounded 56-step bisection for MPD
  radiator clipping. Species property, utilization, efficiency, and constant-γ
  errors are engineering-model inputs/limits, not hidden thrust calibration.
- Benchmark (`benches/propulsion.rs`): an 11-power × 4-flow sweep measured
  21.0 ns/row gridded-ion, 20.7 ns/row Hall, 30.3 ns/row MPD, 38.1 ns/row
  resistojet, and 22.2 ns/row arcjet on this machine.
- Fidelity debt: pulsed-power supplies, charge-state distributions, plume
  divergence, electrode/grid erosion, transient bus storage, Hall electron
  transport, and VASIMR-class RF/helicon coupling remain outside this steady
  backend. Flight-authority resource coupling draws the compiled species from
  reachable tanks and supplies only the same-name consumer's delivered bus
  power to the operating point.

### 18.14 Continuous and pulsed fusion propulsion (section 14)

Shipped 2026-09-24: `propulsion::fusion` provides separate continuous-torch
and event-driven pulsed-fusion contracts; both are installable in a vehicle
and authorable through `vehicle-baker`.

- Continuous state is a steady design (`FusionTorchSpec` / compiled torch),
  with command `(available driver power, requested working-fluid flow)` and
  point telemetry for fusion/driver power, reaction-fuel and working-fluid
  flow, total exhaust flow, thrust/Isp, jet kinetic power, exhaust-internal
  power, radiator waste heat, radiator capacity, and active limits. Reaction
  energy, charged-product share, plasma coupling, nozzle conversion, fusion
  gain, and the radiator determine the outputs. D-T, D-D, D-He3, and p-B11
  carry documented Q-values and reactant molar masses.
- Torch energy closure is
  `Pfusion + Pdriver = Pjet + Pexhaust-internal + Qlocal`. Neutron-carried
  energy, driver input, and uncoupled charged energy load local heat; magnetic
  nozzle conversion losses remain as exhaust internal energy. Thrust is the
  ideal momentum relation over the sum of reacting fuel products and supplied
  working fluid. The radiator clips fusion power against local heat, and
  geometry/specific-power inputs size reactor, walls, field coil, and radiator.
- Pulsed state is `(pulse_phase_s, stored_driver_energy_j, cumulative_shots)`;
  command carries charge-bus power and arm state. `advance(state, command,
  dt)` returns the next state plus pulse count, step impulse/average thrust,
  fuel and working-fluid flow, fusion/driver energy, buffer delta, jet and
  exhaust energy, waste heat, and power/thermal-limit flags. The buffer has
  finite energy and charge power; an unarmed drive can recharge without
  firing. Cadence is bounded by the authored maximum rate, pulse duration,
  available driver energy, and average radiative heat duty.
- Pulsed energy closes stepwise:
  `Ebus + Efusion = ΔEbuffer + Ejet + Eexhaust-internal + Qwaste`. Per-shot
  impulse is `sqrt(2 Ejet mexhaust)` and vehicle force uses the impulse divided
  by physics `dt`; mounted moments use the mount lever arm. Heat is reported
  as event energy and step-average watts. The cadence guarantees average
  `Qpulse / pulse_interval <= radiator_capacity`; no pulse thermal-mass state
  or shot-temperature transient is claimed.
- Baker tables are `[[fusion_torches]]` and
  `[[pulsed_fusion_systems]]`. Their compiled dry mass includes reactor/pulse
  hardware, driver or buffer, geometry-derived chamber/nozzle and copper
  field coils, and radiator. Final assembly COM and inertia include both
  mount families. Regressions pin D-T Q/mass energy, charged share, torch and
  pulse first-law closure, radiator clipping, pulse cadence/charge boundaries,
  step-partition invariance, invalid-state refusal, mounted wrenches, TOML
  bake, and final recentering.
- Numerical error: reaction energy and continuous thrust are closed-form;
  pulse events are advanced to exact cadence/charge event boundaries in f64
  and energy bookkeeping closes to floating-point tolerance. No fusion-rate,
  confinement, charged-particle transport, or thermal transient solver is
  approximated by a fitted thrust coefficient.
- Benchmark (`benches/propulsion.rs`): release 11-power × 4-flow steady-torch
  grid measured 23.1 ns/row; the 11-charge-power × 4-step-size pulse advance
  grid measured 34.4 ns/row. Each row advances compiled runtime state/physics,
  not TOML parsing or design compilation.
- Fidelity debt: fusion reaction-rate/confinement and burn dynamics, ash
  management, neutron shielding, pulse-buffer losses, transient thermal
  storage and thermal-graph coupling, pulse-unit depletion, shock/mechanical
  coupling, magnetic-field topology, and technology/material unlock policy
  remain future work. These limits are distinct from the shipped steady and
  event-driven engine interfaces.

### 18.15 Fixed-step resource and electrical integration

The authoritative flight runtime evaluates installed resource consumers once
per fixed step, iterating constrained command scales until the operating points
and shared tank assignment agree. The returned tank plan is committed after
the rigid-body step; consumed mass updates the vehicle frame through the
existing mass/inertia path.

- State includes `VehicleResourceState`, electrical-bus state, APU and jet
  shaft state, turboprop shaft state, and pulsed-fusion buffer state. Commands
  cover mounted engines, APU/jet starters, RCS duty allocation, electric
  thruster feed, fusion working flow/charge, and shaft-drive output.
- One tank transaction covers rocket/chamber flow, mounted jet/APU and
  propeller/turboprop fuel, RCS pulses, electric-thruster species, fusion
  reactants/working fluid, and fuel-cell hydrogen/oxygen. Each installed
  consumer's named port or generic `resource_feed_ports` route limits it to
  compatible tanks reachable through the current assembly links. Consumers
  have unique names and share one availability scale across multiple
  reactants; overlapping ports cannot overdraw shared tanks.
- Resource limitation feeds back into throttle, flow, RCS duty, or pulse
  arming, then re-evaluates the affected operating point before its wrench or
  electrical output is accepted. Fuel cells share the transaction with other
  consumers; constrained cell output is redispatched on the bus.
- APU generator output is evaluated from its gas-turbine shaft and joins the
  bus as actual auxiliary generation. Fuel-cell output is dispatched against
  bus loads and tank inventory. Electric thrusters, fusion drivers/chargers,
  and electric propeller drives request their operating-point load through a
  same-name bus consumer; missing loads provide zero power. APU pneumatic
  bleed is a shaft load and supplies compatible jet/turboprop starters from
  lit installed APUs.
- Baker/runtime coverage includes generic named feed routes, a combined
  APU/fuel-cell/RCS example, shared-reactant allocation, fixed-step mounted
  RCS gas draw and mass updates, fuel-cell dispatch alongside an installed
  electric thruster, and an APU-to-pneumatic-jet-start regression. Detailed
  pipeline pressure/flow, starter duct geometry, transient piston/electric
  source startup, multiple accessory machines per spool, and a general
  multi-spool starter network remain outside this slice.
