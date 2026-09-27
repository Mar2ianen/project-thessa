# Vehicle electrical power system

Status: implemented shared-bus model and vehicle-baker authoring slice.
Vehicle assets can install batteries, ultracapacitor (ionistor) banks,
fission reactors, fixed/foldable solar cell arrays with optional single-axis
sun tracking, and rated power consumers. Consumers share one ideal vessel
bus; there is no authored wire routing or per-part electrical graph.

## 1. Contract

The electrical system is deterministic vehicle state. A step receives simulation
duration, stellar source inputs (irradiance plus occluder geometry),
requested consumer loads, reactor operating fractions, optional solar-array
deployment targets, and sun-tracking commands. It returns updated battery and
ultracapacitor energy, reactor fuel, deployment/tracking state, and per-part
source/load telemetry.

Parts join the bus by being declared on the same vehicle. The bus is an
engineering-game abstraction with no cable geometry, voltage drop, current
limits, circuit breakers, or cross-vehicle power transfer. Source capacity and
part ratings are still explicit, so a consumer can be shed when the vessel
cannot supply its requested load.

## 2. Sources and storage

### 2.1 Solar cells

An array is a rectangular grid of identical cells. Its area is derived from the
integer X/Y cell counts and cell dimensions. For each supplied stellar source
(typically the three Asterion suns, but any count works), the array uses
post-occlusion irradiance and projected incidence:

```text
A = count_x × count_y × cell_size_x × cell_size_y
w_i = irradiance_i × visibility_i × Π eclipse(light_i, occluder_j)
Psolar = deployment_fraction × A × cell_efficiency
         × Σ w_i × max(0, normal(θ) · direction_to_star_i)
```

`SolarFluxSource::from_luminosity` derives irradiance with the inverse-square
law; `from_luminosity_with_occluders` additionally derives the stellar
angular radius from star radius and range. Each `SolarOccluder` carries a
body-frame direction (from an authoritative ephemeris/attitude transform) and
an angular radius (from body size and range, e.g. a planet or another
vehicle). The combined dimming multiplies the caller's `visibility` by every
geometric disc-overlap factor, using the same circle-circle lens formula as
the lighting pipeline (independently implemented in vehicle-core so power
stays free of visual-crate dependencies). Total eclipse yields exactly 0.0;
a small craft transiting the disc blocks `(ro/rl)^2`; clear geometry yields
1.0. Supplying occluders without a stellar angular radius fails the step
closed instead of silently passing full sun: a point source cannot produce a
penumbra.

Fixed arrays keep their reference orientation. Single-axis arrays rotate the
whole cell sheet about an authored body-frame axis (alpha-joint topology; the
axis is assumed to pass through the array center, so position is unchanged).
Per step the drive resolves a target angle — a manual angle wins, otherwise
the instantaneous optimum over all supplied sources when auto-tracking is
enabled, otherwise it holds — then slews toward it within `slew_rate × dt`,
drawing `actuator_power × duty` as a utility bus load. The optimum maximizes
the weighted projected sum above, so a tracking wing follows the strongest
unoccluded sun and ignores eclipsed ones; it holds position in darkness or
when the sun lies along the slew axis (no gain, no wasted power). Power uses
the beginning-of-step angle (explicit Euler), like fold deployment. Mass
baking uses the reference orientation; runtime tracking does not yet
recompute vehicle inertia.

Cell and support areal densities derive installed mass. The baker includes
array mass, its sheet inertia, position, and the vehicle-frame center-of-mass
shift. The current slice does not animate panel geometry, model
own-vehicle self-shadowing, degradation, or thermal derating.

### 2.2 Batteries and ultracapacitors

Battery authoring specifies usable energy capacity, initial charge fraction,
charge/discharge power ratings and efficiencies, specific energy, dimensions,
and position. Installed mass is `capacity / specific_energy`; cuboid inertia is
derived from the specified dimensions.

Ultracapacitor (ionistor/supercapacitor) banks use the same energy/power-bound
bus model as a separate authoring type, so high-power/low-energy pulse buffers
are explicit. Charge/discharge headroom pools across batteries and
ultracapacitors in proportion to instantaneous limits — the ideal bus assigns
no chemistry priority. Voltage dynamics, leakage/self-discharge, and cycle
ageing are future work.

The runtime stores energy in joules. Charge and discharge are bounded by both
their power rating and remaining energy/capacity. Bus-side charging adds
`P × efficiency × dt`; bus-side discharge removes `P × dt / efficiency`.

### 2.3 Fission reactors

Reactor output is bounded by authored thermal rating, electric conversion
efficiency, radiator capacity, requested power fraction, and remaining fissile
inventory. The heat-rejection limit is:

```text
Pelectric,max = min(Pthermal,rated × η,
                    Pradiator × η / (1 - η))
Qwaste = Pelectric × (1 - η) / η
fuel_used = Pelectric × dt / (η × fuel_specific_energy)
```

The reactor package and initial fuel inventory contribute installed mass. The
runtime tracks fuel consumed and remaining; runtime vehicle mass/inertia are not
yet recomputed as reactor fuel is consumed.

## 3. Load allocation

Each `PowerConsumerSpec` provides a unique name, rated power, and priority.
Per-step commands request a non-negative load no greater than its rating.
Priorities shed in this order: life support, flight control, propulsion, and
utility. Loads at the same priority share a shortfall in proportion to their
requested powers.

Available solar and reactor generation serve loads before storage discharges.
Unused solar generation charges storage first; unused requested reactor
capacity can charge any remaining storage capacity. Charge and discharge are
bounded by per-store limits, and excess generated solar power is reported as
spill. Unsupplied requested power is reported per consumer and as a total.
Fold and sun-tracking actuators join the bus as utility loads with their
authored power draws.

An electric thruster can use a same-named consumer. After the bus step,
`VehicleDefinition::electric_thruster_commands_from_bus` converts that
consumer's delivered power into the thruster's available-power input; the
thruster's existing physical power/flow/thermal limits remain authoritative.
Other parts can use the generic consumer interface; automatic load extraction
from landing-gear motors, reaction wheels, cabin equipment, and other actuators
is future integration work.

## 4. Vehicle TOML

The optional `[electrical_power]` table contains `[[electrical_power.batteries]]`,
`[[electrical_power.ultracapacitors]]`, `[[electrical_power.solar_arrays]]`,
`[[electrical_power.reactors]]`, and `[[electrical_power.consumers]]`. A
single-axis tracking drive is an optional `[electrical_power.solar_arrays.tracking]`
sub-table with rotation axis, angle limits, slew rate, actuator power, and
initial angle; omitting it means fixed. The complete parameterized example is
[`data/vehicles/example_powered_spacecraft.toml`](../../data/vehicles/example_powered_spacecraft.toml).
It includes both a fixed array and a foldable single-axis-tracking array with
cell-grid sizes, an energy store, an ultracapacitor pulse buffer, a fission
source, and prioritized loads.

The baker validates unique names, positive dimensions/ratings, efficiencies,
unit panel axes, tracking-axis geometry, and load configuration. It adds power hardware mass/inertia
before the final vehicle center-of-mass bake and shifts all installed
component positions into the final body frame. Older vehicle assets without
the table remain valid with an empty power system.

## 5. Verification and boundary

Regression tests cover inverse-square flux, incidence and eclipse visibility,
geometric occluders (total/annular/stacked, fail-closed without a stellar
disc), cell-area scaling, mass/inertia derivation, priority shedding, battery
and ultracapacitor energy/efficiency bounds, reactor heat/fuel balance,
fold-actuator power limits, single-axis tracking toward the strongest of three
suns, eclipse hold, and powered-thruster bus allocation. The tracking search
(72-sample scan plus local refinement) recovers >= 99.5% of a 3600-step
brute-force optimum on a three-sun fixture. The 64-vessel runtime benchmark is
`cargo bench -p thessa-sim-core --bench electrical_power` (~240k vessel steps/s
with tracking, occlusion, and storage pooling).

The bus is a powered-load allocation model, not a complete electrical network.
Voltage/current dynamics, ultracapacitor leakage, converters, short circuits,
solar thermal behavior, fuel-mass center-of-mass drift, own-vehicle
self-shadowing, multi-axis gimbals, automatic subsystem demand generation, and
power exchange across docked assemblies remain future work.
