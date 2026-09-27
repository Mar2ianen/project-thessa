# Vehicle thermal system

Status: implemented lumped-node model and vehicle-baker authoring slice.
Vehicle assets can install thermal nodes, conduction links, and area
radiators. Nodes absorb sunlight through the same occluded stellar inputs as
the power bus, pick up Sutton-Graves stagnation aero heating, accept explicit
internal loads (engine/reactor waste heat wired by the caller), and reject
heat to the cold background directly or through radiators. Shield-specific
thermal protection and ablation are not coupled yet, and there is no automatic
damage: overheating is reported, never auto-exploded.

## 1. Contract

The thermal system is deterministic vehicle state. A step receives simulation
duration, stellar source inputs (shared with the power bus, occlusion
included), an optional local airflow, and per-node internal loads. It returns
updated node temperatures plus per-node and vessel-total heat-flow telemetry.

The model sits between KSP and a full finite-element solve: real lumped
physics (capacity, conduction, T⁴ radiation, incidence, stagnation heating)
on a coarse authored graph, without per-panel CFD, temperature-dependent
material strength, or phase change.

## 2. Nodes, links, and radiators

### 2.1 Nodes

A node is one isothermal lump: a structure bay, tank wall, avionics box, or
reactor block. Authoring specifies mass, specific heat, initial and maximum
temperature, emissivity, solar absorptivity, radiating area, solar-exposed
area with a body-frame normal, aero area with a nose radius, and position.
Heat capacity is `mass × cp`; mass joins the vehicle COM bake as a point
mass (documented simplification for small thermal hardware).

Per step, each node integrates:

```text
C·dT/dt = Qsolar + Qaero + Qinternal + Qconducted − Qradiated − Qradiator
Qsolar  = α · Aexp · Σ w_i · max(0, n · s_i)
Qaero   = k · sqrt(ρ/Rn) · v³ · Aaero         (Sutton-Graves stagnation)
Qradiated = ε · σ · Arad · (T⁴ − Tbg⁴),  Tbg = 2.725 K
Qconducted = Σ G·(T_neighbor − T)            (energy-conserving by construction)
```

`w_i` is the post-occlusion irradiance from the shared stellar inputs, so a
panel in eclipse heats like it is in eclipse. Occluders may come from planets
and other vehicles, or from the vessel's own hull:
`VehicleDefinition::own_body_occluder` derives a blocker disc from the baked
collision geometry with a CPU ray query (no GPU), so arrays, nodes, and
radiators share one shadow. `Qinternal` is an explicit
per-node load: the caller wires reactor waste-heat telemetry (or any engine
heat) into it by node name. Negative internal loads model active cooling.

### 2.2 Links and radiators

A conduction link joins two named nodes with a symmetric conductance (W/K).
Self-links, unknown nodes, and duplicate pairs fail closed at validation.

A radiator is an area device tied to one node: it rejects
`εσA(T_node⁴ − T_bg⁴)` from that node and absorbs sunlight on the same area
with its own absorptivity and normal. Fixed radiators stay exposed; foldable
radiators scale effective area by their deployed fraction, slewing toward
targets at the authored rate. The deployment motor draws bus power booked by
the caller: each step reports the requested motor power and moves only by the
granted share (`radiator_power_fraction`, same pattern as electric-thruster
available power).

### 2.3 Integration

Explicit Euler with internal stability substepping. The step is split from a
conservative linearized bound (radiation slope at `max(current, rated)`
temperature plus all link conductances); beyond 4096 substeps the step is
rejected so the caller shortens `dt` instead of integrating garbage. Long
soaks converge to the closed-form radiator equilibrium
`P = εσA(T⁴ − T_bg⁴)`; the energy balance (stored change vs net flows) closes
to integration precision and is pinned by regression.

## 3. Vehicle TOML

The optional `[thermal]` table contains `[[thermal.nodes]]`,
`[[thermal.links]]`, and `[[thermal.radiators]]` (fixed by default;
`deployment = "foldable"` plus rate/actuator/initial-fraction fields for a
deploying wing), plus an optional
`convective_k` (Sutton-Graves constant for the operating atmosphere, Earth
air by default). The powered-spacecraft example
[`data/vehicles/example_powered_spacecraft.toml`](../../data/vehicles/example_powered_spacecraft.toml)
carries a service-module node, a reactor-block node, their link, and a
radiator wing.

The baker validates unique names, positive masses/capacities, temperature
ordering, unit normals, known link/radiator endpoints, and positive
conductance. Node and radiator masses join the final vehicle center-of-mass
bake with body-frame recentering. Older assets without the table remain valid
with an empty thermal system.

## 4. Verification and boundary

Regression tests cover two-node conduction convergence to the
capacity-weighted mean, flat-plate solar analytic energy, closed-form
radiator equilibrium, foldable-radiator rate and bus-power limits, the
Sutton-Graves number (494.1 kW/m² at
ρ = 0.01 kg/m³, v = 3000 m/s, Rn = 1 m), own-hull ray occlusion (box blocks,
strut grazes past, inside means full sky) end to end through the power bus,
energy-balance closure, overheat margins, and fail-closed specs/commands/
states. The 64-vessel runtime benchmark is
`cargo bench -p thessa-sim-core --bench thermal` (~1.3M vessel
steps/s for a 4-node/3-link/2-radiator graph with one deploying wing).

Not modeled: shield-specific thermal protection and ablation (see
`details/11_HEAT_SHIELDS.md`; shield aerodynamics are implemented through the
shared force pipeline), temperature-dependent material strength, phase change,
convective cooling to airflow,
own-vehicle self-shadowing beyond the collision-part ray query (silhouette
penumbra of near misses), multi-axis radiator gimbals, and automatic
engine-to-node heat wiring (the caller maps waste-heat telemetry onto node
loads explicitly).
