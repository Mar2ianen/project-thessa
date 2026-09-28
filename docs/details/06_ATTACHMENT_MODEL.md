# Part attachment and assembly model

Status: authored attach nodes, pose solving, transformed rigid-body
geometry/mass, validated topology, and runtime crew/air/feed connectivity
are implemented. Named non-tree resource edges and pressure-limited
`FeedLine` routing are implemented. Contact-scene docking now advances the
persisted D1 protocol, installs/removes a fixed joint, and reports solver
joint force/moment. A breakup reconstruction primitive now rebuilds component
mass, inertia, COM position, and COM velocity from complete caller-supplied
per-body mass records. The baker and server do not yet retain/migrate complete
per-part subsystem ownership into separate authoritative vehicle clusters.
Reference stays KSP: parts mate through explicit nodes into one craft tree.

## 1. Goal

Separate parts assemble into one craft through authored interfaces:

- stack and hatch nodes on bodies (KSP-style, explicit records — never
  inferred from proximity);
- one validated assembly tree (no cycles, no forests, diameter match);
- geometric mating transforms that carry aero panels, collision parts,
  tank mounts, cabin seats/exits, ports, controls, and hull mass/inertia
  into the craft frame;
- passable cabins: crew and air domains shared through open hatches;
- resource reachability: which tanks can feed installed consumers through
  named engine-feed ports.

Still outside this slice: internal assembly-joint load resolution,
material-rated structural failure and physical cluster splitting,
authoritative multi-vehicle ownership, finite-rate cabin flow, and airlock
parts. Resource edges currently carry reachability plus optional feed-line
pressure loss; they do not model distributed line pressure, line storage,
transients, pumps, or flow sharing through a branched pipe network.

## 2. Nodes (hangar authoring)

`AttachNode { name, site, kind, diameter_m? }` lives on
`ProceduralBody` next to ports and heat shields:

- `site`: `aft-end` / `forward-end` / explicit loft-surface
  `station { x_m, clock_rad }` (validated inside the station range;
  `clock_rad = 0` is +Y and positive angles turn toward +Z).
- `kind`: `stack` (structure + resources, crew never passes, always
  open) vs `hatch` (adds crew/air when open; carries fuel only when
  open, like KSP with crossfeed disabled).
- `diameter_m`: explicit docking standards, else the local section
  diameter. End nodes mate at the end-section centre; station nodes sit
  on the superellipse outline and use its local surface normal. Node
  names are unique per body.

## 3. Links and tree validation

`AssemblyLink { name, parent_body, parent_node, child_body, child_node,
hatch_open = true }` (TOML endpoints are `body.node`). `compile_assembly`
fails closed on duplicate/unknown bodies or links, unknown nodes, a node
used twice, self-links, non-positive interface diameters, diameter
mismatch beyond 5% relative (fit an adapter part instead — future),
hatch links below the 0.5 m crew-passage minimum, cycles, and forests.
One body with no links compiles alone.

A hatch node may mate any node kind; a mixed link is passable exactly
when its hatch side is open. Pure stack links have no crew passage and
always pass resources.

The root keeps its authored translation. Every child is solved in the
tree from its parent node: node points coincide and their outward normals
oppose. A half-turn in the mating frame fixes the default axial roll.
The child `origin_body_m` is a preview/authoring placement only once it is
attached; the link pose is authoritative. Without assembly links, legacy
per-body origins are preserved.

## 4. Domains

Volumes are non-tank regions; tanks are never crew volumes:

- **Crew domains** (`crew_groups`): union over open hatch links, plus the
  documented open-interior rule — volumes in one body share passage
  unless a future bulkhead part says otherwise. `crew_can_pass` is the
  direct runtime query. Suits and pressure remain cabin-policy checks.
- **Air domains** (`air_groups`): open links between pressurized volumes
  only. A dry region can be crew-passable but does not join a pressure
  domain.
- **Resource reachability** (`feed_paths`): tank regions to `engine-mount`
  ports through resource-open structural links and explicit non-structural
  crossfeed edges, as qualified `body.region` →
  `body.port` pairs (`Bipropellant` regions expose `body.region-ox` and
  `body.region-fuel`). A closed hatch blocks flow through that structural
  link; explicitly authored crossfeed edges remain independent. The
  fixed-step allocator draws from the reachable inventory according to each
  installed consumer's operating-point flow. Rocket mixture ratio, APU/jet
  fuel, RCS propellant, electric working fluid, fusion reactants, and fuel-cell
  hydrogen/oxygen all use compatible stored-resource identities. APUs and fuel
  cells may author their endpoint directly; `resource_feed_ports` maps other
  consumer names to an engine-feed endpoint. Named compatible tanks may also
  be manually transferred across an open resource path.

## 5. Runtime connectivity (`thessa-vehicle-core::assembly`)

`VehicleDefinition.assembly` retains named bodies, volume addresses,
resource endpoints, and named mutable `AssemblyLinkState`s. Runtime
queries expose topological `crew_can_pass`, pressure-aware
`crew_can_pass_safely`, `cabins_share_air`, named crew/air domains, and
qualified feed paths. Pressure-aware passage requires non-vacuum air in all
regions along an unsuited or hose-fed crew member's route; self-contained
suits also permit dry/vacuum passage. This is a compartment access query,
not character movement. Structural stack joints never pass crew but always
pass resources. Index-based helpers are geometry-free and validate endpoint
ranges. Fixed-step resource planning honors current link/edge state and named
feed endpoints for routed consumers; shared inventory is reserved across
overlapping ports before one mass/inertia commit. An optional `FeedLine` on a
structural link or non-tree resource edge applies the existing propulsion
pressure-drop law. For a routed fluid, `FeedResourceProperties` supplies
density, dynamic viscosity, regulated source pressure, and required consumer
inlet pressure. The allocator evaluates the least-drop reachable path, enforces
the line's pressure rating and hard velocity gate, and bisects a shared
consumer flow to the largest pressure-feasible value before committing tank
draw. Omitting fluid properties retains the ideal legacy route; a route through
authored lines must provide them. This is a static path calculation, not a
branched hydraulic network solver. Opening an exterior assembly hatch into a
dry region refuses while
its connected pressure domain contains air unless the caller explicitly
asserts that all exposed occupants are suited. That operation vents the
affected domain and updates vehicle mass properties and the body-frame COM
atomically. Opening after a separate `vent_cabin` operation is also allowed.
An authored hatch that starts open to a dry body vents the connected pressure
domain during baking, before mass and inertia aggregation; a runtime assembly
with positive cabin air still exposed to a dry body fails validation.
When an open hatch joins pressure volumes, `VehicleDefinition`
resolves the ideal-gas equilibrium as an instantaneous state transition:
total air, oxygen, and sensible thermal energy are conserved, and the
resulting inventory is distributed by chamber volume at common pressure,
temperature, and composition (constant dry-air heat capacity and gas
constant). Closing the hatch preserves each chamber's current state.
The baker resolves initially open pressure domains before final COM/inertia
aggregation, venting any domain open to a dry body and equalizing domains that
remain pressurized. Vehicle-level vent/repress operations apply to the entire
connected pressure domain. Later runtime hatch and cabin-pressure changes
update cabin inventories and vehicle mass/inertia plus every stored body-frame
point, including propulsion, landing-gear, reaction-wheel, and parachute
mounts. Finite-rate orifice flow remains future work.

The flow feasibility search uses 32 monotone bisection steps, bounding its
final relative flow interval by `2^-32` (less than `2.4e-10` of requested
flow), before f64 rounding; the regression pins the resulting inlet-pressure
root within `0.1 Pa` for its known line fixture. The line constitutive
approximation remains the existing Darcy-Weisbach law: laminar `64/Re`,
smooth-turbulent Blasius, and documented entrance/exit and bend coefficients;
rough-pipe and two-phase uncertainty are model limits rather than solver
error. Regression coverage checks the supported flow against the authored
minimum inlet pressure and the velocity gate. The `resource_routing` bench
measured `436.89 us` per plan for
one 64-body, 32-tank, 16-consumer vehicle with 63 authored line segments on the
current development run (`cargo bench -p thessa-sim-core --bench
resource_routing`; local hardware dependent).

## 6. Baker wiring

Optional `[[assembly.links]]` and `[[assembly.resource_edges]]` on the vehicle
asset. Present links are resolved and validated through `compile_assembly`; transforms are applied
before aero, mass, and collision aggregation and the root/part poses are
reported. Runtime connectivity is serialized onto the baked
`VehicleDefinition`; absent links skip silently for legacy assets. Baker
test bakes `data/vehicles/example_assembly.toml` (stage + capsule, sealed
hatch, independent fuel umbilical), checks attach-frame position/normal
residuals below `1e-12 m`, and verifies that the umbilical preserves feed
reachability without opening crew or cabin-air paths.

Pose solving is a tree traversal, O(parts + links); runtime connectivity
is rebuilt from the compact volume/link graph when queried. Attach-frame
coincidence is algebraic and tested to a `1e-12 m` floating-point
residual for axial and radial joints. Baker reports each final rigid
transform for hangar diagnostics. The cabin regression compares final
pressure against the ideal-gas equilibrium and pins air/O2 residuals
below `1e-12 kg` and the mass-weighted temperature residual below
`1e-10 kg K` for a two-chamber mixed-temperature case. The `assembly_air`
benchmark processes 64 connected chambers, including runtime mass-property
bookkeeping and coordinate-shift handling, in `56.58 us` per transition on
the current development run (`cargo bench -p thessa-sim-core --bench
assembly_air`; local hardware dependent).

## 7. Docking, joint loads, and split planning

`ContactRuntime` can sync a partner into the shared Rapier scene, validate
live local-port kinematics against `DockingSession`, begin soft capture, align
and hard-dock with a fixed joint, advance pressure equalization in physics
time, report the latest joint solver impulse as average force/moment, and
remove the joint while preserving both solved body states. The D1 contact test
exercises that sequence under an applied load. `CollisionWorld` reports
constraint loads only; it does not assign structural damage or infer ratings.
This partner-scene path is not yet wired into the server: `apps/server::Sim`
still owns one `FlightAuthority`, and the current client protocol has no
vehicle-targeted docking, undocking, or separation commands. The server therefore
does not yet transfer vehicle ownership or persist a dock graph.

`VehicleAssembly::body_components_after_link_failure` computes deterministic
body-index components after removing a named structural edge, and
`split_after_link_failure` returns independently validated assembly graphs
with local indices and no resource edge crossing the physical split. These
operations deliberately ignore cross-cluster umbilicals when determining
structure. `reconstruct_clusters_after_link_failure` additionally takes one
complete `AssemblyBodyMassProperties` record per authored body, aggregates
centroidal inertia with the parallel-axis theorem, and recenters each
`RigidBodyState`. Released clusters inherit the original orientation and
angular velocity; their COM velocities include the physical `omega x r`
offset. The caller supplies the source vehicle's current mass properties;
total mass, source COM, and inertia must close against the body records before
a split is accepted. Regression coverage checks conservation and released COM
states for a rotating three-body stack. The mass records must include all
body-owned hardware and payload: incomplete records fail closed.

The existing `assembly_air` benchmark now also measures a 64-body chain split
and mass/kinematic reconstruction. A 2,000-reconstruction release run on an
AMD Ryzen 7 8745H measured `29.78 us` per reconstruction. This measures the
current topology and mass-state primitive only; it excludes part-definition
migration and authoritative fleet insertion.

This is the physical rigid-body reconstruction contract, not yet a complete
runtime vehicle reconstruction. The baked `VehicleDefinition` does not yet
retain enough per-part mass, inertia, aero, collision, thermal, actuator, and
inventory ownership to instantiate these results as independent authorities.

## 8. Next slices

- Internal assembly-joint load paths, per-link strength ratings, baked
  per-part ownership, and complete aero/collision/subsystem migration when
  constructing independent vehicle clusters.
- Server-level docking/separation ownership and persistence across independent
  vehicle authorities.
- Branched feed-network pressure/flow solving, rate limits, and drain-order
  policy beyond the current least-drop path calculation.
- Finite-rate hatch/orifice flow; airlock parts (cycled volume instead of
  whole-cabin venting); adapter parts for diameter transitions.
