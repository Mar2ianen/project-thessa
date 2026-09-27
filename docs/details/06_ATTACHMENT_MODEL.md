# Part attachment and assembly model

Status: authored attach nodes, pose solving, transformed rigid-body
geometry/mass, validated topology, and runtime crew/air/feed connectivity
are implemented. The assembled craft is currently one rigid body; joint
loads, structural breakup, and runtime docking/separation remain later
slices. Reference stays KSP: parts mate through explicit nodes into one
craft tree.

## 1. Goal

Separate parts assemble into one craft through authored interfaces:

- stack and hatch nodes on bodies (KSP-style, explicit records — never
  inferred from proximity);
- one validated assembly tree (no cycles, no forests, diameter match);
- geometric mating transforms that carry aero panels, collision parts,
  tank mounts, cabin seats/exits, ports, controls, and hull mass/inertia
  into the craft frame;
- passable cabins: crew and air domains shared through open hatches;
- fuel reachability: which tanks can feed which engine ports.

Non-goals in this slice: per-joint loads, structural failure and cluster
splitting, runtime docking/undocking, finite-rate cabin flow, fuel *flow*
simulation (reachability only), airlock parts, struts and fuel lines
(they require explicit non-tree graph edges).

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
- **Fuel reachability** (`feed_paths`): tank regions to `engine-mount`
  ports through resource-open links, as qualified `body.region` →
  `body.port` pairs. Closed hatches block fuel like sealed KSP docks.

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
ranges. Opening an exterior assembly hatch into a dry region refuses while
its connected pressure domain contains air unless the caller explicitly
asserts that all exposed occupants are suited. That operation vents the
affected domain and updates vehicle mass properties and the body-frame COM
atomically. Opening after a separate `vent_cabin` operation is also allowed.
When an open hatch joins pressure volumes, `VehicleDefinition`
resolves the ideal-gas equilibrium as an instantaneous state transition:
total air, oxygen, and sensible thermal energy are conserved, and the
resulting inventory is distributed by chamber volume at common pressure,
temperature, and composition (constant dry-air heat capacity and gas
constant). Closing the hatch preserves each chamber's current state.
The baker resolves initially open domains before final COM/inertia
aggregation. Later runtime hatch changes update cabin inventories and vehicle
mass/inertia plus the body-frame COM for gas redistribution and venting.
Finite-rate orifice flow remains future work.

## 6. Baker wiring

Optional `[[assembly.links]]` on the vehicle asset. Present links are
resolved and validated through `compile_assembly`; transforms are applied
before aero, mass, and collision aggregation and the root/part poses are
reported. Runtime connectivity is serialized onto the baked
`VehicleDefinition`; absent links skip silently for legacy assets. Baker
test bakes `data/vehicles/example_assembly.toml` (stage + capsule, open
hatch), checks attach-frame position/normal residuals below `1e-12 m`,
and toggles the hatch to verify crew passage, pressure equalization, and
cross-part feed reachability.

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

## 7. Next slices

- Joint load paths, structural failure/splitting, dock/undock events,
  and per-link load limits.
- Fuel flow simulation over `feed_paths` (rates, drain order).
- Finite-rate hatch/orifice flow; airlock parts (cycled volume instead of
  whole-cabin venting).
- Adapter parts for diameter transitions; struts/fuel lines as explicit
  non-tree edges.
