# Detail design notes

Status: mixed: design baselines and implemented vehicle-system models. Each
document marks its own implementation boundary; see [`docs/00_STATUS.md`](../00_STATUS.md)
for the current project-wide implementation index.

This directory contains focused engineering/gameplay specifications for concrete vehicle, station, infrastructure, and interaction details that are too narrow for the top-level architecture documents.

These notes define design intent, implementation contracts, and cross-system
invariants. Exact balance values and unimplemented behavior remain marked TBD
in the relevant document.

## Index

- [Docking ports](01_DOCKING_PORTS.md) — standard androgynous pressurized docking interfaces and compact utility attachment points.
- [Procedural aerodynamic surfaces](02_PROCEDURAL_AERO_SURFACES.md) — spline-authored wings/fins, mechanization, nested control surfaces, and hangar compilation into solver-ready aerodynamic panels.
- [Procedural fuselages and body modules](03_PROCEDURAL_FUSELAGES.md) — revolve/loft body authoring, derived structure/interior volume, semantic modules and presets, with a constrained path toward future cutouts.
- [Procedural propulsion systems](04_PROCEDURAL_PROPULSION.md) — component/flow-graph propulsion covering chemical rockets, nuclear thermal, gas turbines, atmospheric-reactant engines, electric/plasma propulsion, combined cycles, and fusion systems.
- [Cabin editor](05_CABIN_EDITOR.md) — parametric airliner and fighter cabins; seat blocks/decks, per-place suit overrides, static exits, venting/EVA rules, and control authority compile through the vehicle baker.
- [Part attachment and assembly](06_ATTACHMENT_MODEL.md) — KSP-style attach nodes, geometric part transforms, rigid-body aggregation, live crew/air domains, and fuel reachability; joint failure/separation later.
- [Procedural landing gear and rover wheels](05_PROCEDURAL_LANDING_GEAR.md) — parameterized wheel chassis, pneumatic/airless tires, suspension struts, fold-out lander legs with reusable/crushable shocks, gear deployment, brake actuators, optional electric drives, and Rapier contact integration.
- [Reaction-wheel attitude control](06_REACTION_WHEELS.md) — KSP-style body-moment authority with authored per-axis torque ratings, vehicle mass baking, RCS residual allocation, and pilot controls.
- [KSP-style deployable parachutes](07_PARACHUTES.md) — automatic pressure-triggered extraction, dynamic-pressure opening limits, reefed inflation, canopy overload failure, and physically applied drag at vehicle mounts.
- [Vehicle part commands](08_VEHICLE_PART_COMMANDS.md) — shared typed subsystem commands for pilot inputs and future stage/action-group dispatch.
- [Vehicle electrical power](09_ELECTRICAL_POWER.md) — parameterized batteries, fission reactors, static/foldable cell arrays, and prioritized loads on one wire-free vessel bus.
- [Vehicle thermal system](10_THERMAL_SYSTEM.md) — lumped thermal nodes with conduction links and fixed/foldable area radiators; solar, Sutton-Graves aero, and wired internal heating; own-hull ray occlusion; overheat reporting without auto-damage.
- [Heat shields and thermal protection](11_HEAT_SHIELDS.md) — Newtonian shield-disc aerodynamics through the common drag/lift contract and wing hex tile-layer toggle implemented; shield thermal protection/ablation and spline-bounded tile screens remain design.
