# Vehicle electrical power system

Status: implemented shared-bus model, propulsion source/load coupling, and
vehicle-baker authoring slice.
Vehicle assets can install batteries, ultracapacitor (ionistor) banks,
fission reactors, fixed/foldable solar cell arrays with optional single-axis
sun tracking, fuel cells, and rated power consumers. Mounted APU generators
and tank-fed fuel cells supply the same ideal vessel bus; there is no authored
wire routing or per-part electrical graph.

## 1. Contract

The electrical system is deterministic vehicle state. A step receives simulation
duration, stellar source inputs (irradiance plus occluder geometry),
requested consumer loads, reactor operating fractions, optional solar-array
deployment targets, and sun-tracking commands. The flight runtime also derives
loads from installed electric thrusters, fusion drivers/chargers, and electric
propeller drives by their same-name consumer. It returns updated battery and
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
w_i = irradiance_i × visibility_i
     × (1 - area(light_disc_i ∩ union(occluder_discs_ij))
            / area(light_disc_i))
Psolar = deployment_fraction × A × cell_efficiency
         × Σ w_i × max(0, normal(θ) · direction_to_star_i)
```

`SolarFluxSource::from_luminosity` derives irradiance with the inverse-square
law; `from_luminosity_with_occluders` additionally derives the stellar
angular radius from star radius and range. Each `SolarOccluder` carries a
body-frame direction (from an authoritative ephemeris/attitude transform) and
an angular radius (from body size and range, e.g. a planet, another vehicle,
or the vessel's own hull via `VehicleDefinition::own_body_occluder`, which
ray-casts the baked collision geometry on the CPU). The combined dimming
multiplies the caller's `visibility` by the fraction of the stellar disc not
covered by the **union** of the occluder discs. Circle intersections partition
the exposed boundary arcs, whose area is integrated once; coincident and
partial shadows therefore are not counted multiple times. A single-disc case
reduces to the standard circle-circle lens area. This computation uses the
same planar apparent-disc model as the lighting pipeline, independently
implemented in vehicle-core so power stays free of visual-crate dependencies.
Total eclipse yields exactly 0.0; a small craft centered on the disc blocks
`(ro/rl)^2`; clear geometry yields 1.0. Supplying occluders without a stellar
angular radius fails the step closed instead of silently passing full sun: a
point source cannot produce a
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
no chemistry priority. Stored energy additionally decays as
`E *= exp(-dt / tau)` with the authored `self_discharge_time_s` (default
infinite, i.e. legacy lossless); the step loss is reported as
`self_discharge_loss_j`. Voltage dynamics and cycle
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
runtime tracks fuel consumed and remaining; `live_mass_properties(state)`
recomputes vessel mass/COM/inertia from the remaining inventory so fuel burn
drifts the bake without a second bookkeeping path.

## 3. Load allocation

Each `PowerConsumerSpec` provides a unique name, rated power, and priority.
Per-step commands request a non-negative load no greater than its rating.
Priorities shed in this order: life support, flight control, propulsion, and
utility. Loads at the same priority share a shortfall in proportion to their
requested powers.

Available solar, auxiliary-generator, reactor, and fuel-cell generation serve
loads before storage discharges (in that source order).
Unused generation charges storage in the same source order: solar surplus
first, then auxiliary-generator surplus, then reactor surplus, then fuel-cell
surplus. Each stage is bounded by remaining per-store charge limits. Excess
generated solar and auxiliary power that cannot charge or serve load is
reported as spill; fuel-cell and reactor output only burns fuel to serve load
plus accepted charge, so unburned capacity is simply not burned. Unsupplied
requested power is reported per consumer and as a total.
Fold and sun-tracking actuators join the bus as utility loads with their
authored power draws.

Electric thrusters, continuous/pulsed fusion power electronics, and electric
propeller drives use same-named consumers. The runtime derives each request
from the commanded operating point, takes only delivered bus power, and then
limits thrust/output accordingly. Electric-thruster feed, fusion reactants and
working fluids, and fuel-cell reactants are planned in the shared tank
transaction; tank scarcity triggers a new operating-point evaluation and bus
redispatch. An unconfigured consumer supplies no electric propulsion power.
Other parts can use the generic consumer interface; automatic load extraction
from landing-gear motors, cabin equipment, and other actuators
is future integration work. Reaction-wheel banks already derive their demand
automatically (see [`06_REACTION_WHEELS.md`](06_REACTION_WHEELS.md)).

### 3.1 Docked bus exchange

Two vessels on a shared docked interface exchange power through
`DockedBusTie { max_transfer_power_w }` — the cable rating, the only
authored number. The transfer resolves from last-step telemetry on both
sides (explicit Euler, like every cross-tick coupling):
`min(exporter spill, importer unserved, rating)`, booked into the
importer's command as auxiliary input where it joins the normal source
order. The exporter must have actually spilled the power, so the tie can
neither invent energy nor double-serve a load; phantom inputs fail closed.
In the regression a 400 W spill against a 1000 W shortfall through a 250 W
tie clears 250 W of unserved load and reports it as auxiliary-to-load.

## 4. Vehicle TOML

The optional `[electrical_power]` table contains `[[electrical_power.batteries]]`,
`[[electrical_power.ultracapacitors]]`, `[[electrical_power.solar_arrays]]`,
`[[electrical_power.reactors]]`, `[[electrical_power.fuel_cells]]`, and
`[[electrical_power.consumers]]`. A fuel cell may specify `feed_port_name` to
restrict hydrogen/LOX reachability; an installed APU mount may specify
`feed_port_name` for its jet-fuel reachability; generic consumers can be routed
through top-level `[[resource_feed_ports]]`. APUs are authored separately
through top-level `[[auxiliary_power_units]]` and their actual generator output
joins the same bus only through the manual chain
`plan_auxiliary_power_units` → `auxiliary_generation_power_w` →
`advance_electrical_power_with_demands` plus `commit_resource_flows`; the bus
never invents APU power on its own, and a positive auxiliary input with no
installed APU or fitted jet generator fails closed. A single-axis tracking drive is an optional
`[electrical_power.solar_arrays.tracking]`
sub-table with rotation axis, angle limits, slew rate, actuator power, and
initial angle; omitting it means fixed. A nested
`[electrical_power.solar_arrays.tracking.secondary]` block with the same
fields promotes the drive to a two-axis gimbal: the secondary axis rides in
the primary-rotated frame, both joints slew within their limits, and their
motor draws share one utility load slot. The complete parameterized example is
[`data/vehicles/example_powered_spacecraft.toml`](../../data/vehicles/example_powered_spacecraft.toml).
It includes both a fixed array and a foldable two-axis-tracking array with
cell-grid sizes, an energy store, an ultracapacitor pulse buffer, a fission
source, and prioritized loads.
[`data/vehicles/example_apu_fuel_cell.toml`](../../data/vehicles/example_apu_fuel_cell.toml)
shows tank-fed LH₂/LOX fuel-cell power alongside an installed APU and cold-gas
RCS.

The baker validates unique names, positive dimensions/ratings, efficiencies,
unit panel axes, tracking-axis geometry, and load configuration. It adds power
hardware mass/inertia before the final vehicle center-of-mass bake and shifts all installed
component positions into the final body frame. Older vehicle assets without
the table remain valid with an empty power system.

## 5. Verification and boundary

Regression tests cover inverse-square flux, incidence and eclipse visibility,
geometric occluders (total/annular plus coincident, partial, disjoint, and
near-tangent unions; fail-closed without a stellar disc), cell-area scaling,
mass/inertia derivation, priority shedding, battery
and ultracapacitor energy/efficiency bounds, reactor heat/fuel balance,
fold-actuator power limits, fuel-cell stoichiometry/inventory, auxiliary APU
generation, ultracapacitor self-discharge decay, reactor live-mass drift, single-axis tracking toward the strongest of three suns, eclipse
hold, two-axis gimbal outperformance on an off-axis sun with both joints
slewing and both motors booked, manual beta-target override, degenerate
gimbal authoring rejected, and powered-thruster bus allocation. The tracking search
(72-sample scan plus local refinement) recovers >= 99.5% of a 3600-step
brute-force optimum on a three-sun fixture. The 64-vessel runtime benchmark is
`cargo bench -p thessa-sim-core --bench electrical_power` (~187k vessel steps/s
on this run, with tracking, two overlapping occluders, and storage pooling).

The bus is a powered-load allocation model, not a complete electrical network.
Voltage/current dynamics, converters, short circuits,
solar thermal behavior, own-vehicle
self-shadowing, three-axis gimbals, automatic load extraction from nonpropulsive
actuators past reaction wheels, and power exchange across docked assemblies
remain future work.
