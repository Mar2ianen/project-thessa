# Part attachment and assembly model

Status: first slice implemented (nodes, validated topology, crew/air
domains, fuel reachability, runtime recompute). Merged assembly physics
(transforms, joint loads) is a later slice. Reference stays KSP: parts
mate through explicit nodes into one craft tree.

## 1. Goal

Separate parts assemble into one craft through authored interfaces:

- stack and hatch nodes on bodies (KSP-style, explicit records — never
  inferred from proximity);
- one validated assembly tree (no cycles, no forests, diameter match);
- passable cabins: crew and air domains shared through open hatches;
- fuel reachability: which tanks can feed which engine ports.

Non-goals in this slice: merged multi-body physics (transforms beyond
the existing axis-aligned mounts, joint loads, undocking at runtime),
fuel *flow* simulation (reachability only), airlock parts, struts and
fuel lines (they would break the tree rule — reserved).

## 2. Nodes (hangar authoring)

`AttachNode { name, site, kind, diameter_m? }` lives on
`ProceduralBody` next to ports and heat shields:

- `site`: `aft-end` / `forward-end` / explicit `station { x_m }`
  (validated inside the station range).
- `kind`: `stack` (structure + resources, crew never passes, always
  open) vs `hatch` (adds crew/air when open; carries fuel only when
  open, like KSP with crossfeed disabled).
- `diameter_m`: explicit docking standards, else the local section
  diameter. Node names are unique per body.

## 3. Links and tree validation

`AssemblyLink { name, parent_body, parent_node, child_body, child_node,
hatch_open = true }` (TOML endpoints are `body.node`). `compile_assembly`
fails closed on: unknown bodies/nodes, a node used twice, self-links,
stack diameter mismatch beyond 5% relative (fit an adapter part
instead — future), hatch links below the 0.5 m crew-passage minimum,
cycles, and forests. One body with no links compiles alone.

A hatch node may mate any node kind; stack nodes have no door, so a
mixed link is passable exactly when its hatch side is open. Stack links
force `hatch_open` open.

## 4. Domains

Volumes are non-tank regions; tanks are never crew volumes:

- **Crew domains** (`crew_groups`): union over open links, plus the
  documented open-interior rule — volumes in one body share air and
  passage unless a future bulkhead part says otherwise. Suits and
  pressure are runtime checks (`sim-core::cabin`), not compile facts.
- **Air domains** (`air_groups`): open links between pressurized
  volumes only. A dry cabin shares crew passage but never air.
- **Fuel reachability** (`feed_paths`): tank regions to `engine-mount`
  ports through resource-open links, as qualified `body.region` →
  `body.port` pairs. Closed hatches block fuel like sealed KSP docks.

## 5. Runtime recompute (`sim-core::assembly`)

Index-based mirrors with no geometry: `crew_groups`, `air_groups`
(plus a pressurized mask), and `feed_reachable` (tank/port body
indices) from `AssemblyLinkState { a, b, hatch, open }`. Sealing or
opening a hatch recomputes all three domains through one path; link
endpoints are validated against the node count. Hangar truth and
runtime truth share semantics and mirrored regression tests.

## 6. Baker wiring

Optional `[[assembly.links]]` on the vehicle asset. Present links are
resolved, validated through `compile_assembly`, and reported (root,
crew/air domain sizes, feed paths); absent links skip silently
(legacy assets). Baker test bakes `data/vehicles/example_assembly.toml`
(stage + capsule, open hatch) and asserts root, shared crew domain,
and the tank→engine feed path.

## 7. Next slices (not started)

- Merged assembly physics: orientation mounts, joint load paths,
  dock/undock events, per-link load limits.
- Fuel flow simulation over `feed_paths` (rates, drain order).
- Airlock part (cycled volume instead of whole-cabin venting).
- Adapter parts for diameter transitions; struts/fuel lines as
  explicit non-tree edges.
- Vehicle-level link states (hatch toggles persist on the definition).
