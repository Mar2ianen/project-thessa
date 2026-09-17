# Detail design notes

This directory contains focused engineering/gameplay specifications for concrete vehicle, station, infrastructure, and interaction details that are too narrow for the top-level architecture documents.

These notes define stable design intent and cross-system invariants. Exact balance values and implementation details may remain TBD until the relevant simulation subsystem is implemented.

## Index

- [Docking ports](01_DOCKING_PORTS.md) — standard androgynous pressurized docking interfaces and compact utility attachment points.
- [Procedural aerodynamic surfaces](02_PROCEDURAL_AERO_SURFACES.md) — spline-authored wings/fins, mechanization, nested control surfaces, and hangar compilation into solver-ready aerodynamic panels.
- [Procedural fuselages and body modules](03_PROCEDURAL_FUSELAGES.md) — revolve/loft body authoring, derived structure/interior volume, semantic modules and presets, with a constrained path toward future cutouts.
- [Procedural propulsion systems](04_PROCEDURAL_PROPULSION.md) — component/flow-graph propulsion covering chemical rockets, nuclear thermal, gas turbines, atmospheric-reactant engines, electric/plasma propulsion, combined cycles, and fusion systems.
