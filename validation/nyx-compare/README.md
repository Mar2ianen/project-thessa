# Nyx/ANISE comparison harness

This is a reference-only validation tool. It is intentionally a separate
Cargo workspace because `nyx-space` is AGPL-3.0-or-later and must not become a
dependency of the MIT `thessa-sim-core` runtime crate or of the GPL game
applications.

The harness creates the same initial Cartesian state in both implementations:

- reference: ANISE two-body Kepler propagation from the Nyx 2.5.3 ecosystem;
- candidate: `thessa-sim-core` `f64` adaptive Dormand–Prince 5(4) propagation
  through a fixed central point-mass field.

It reports absolute position and velocity differences for several eccentric,
inclined LEO/GEO cases and multiple coast durations. The reference is an
independent two-body oracle, not a validation of Thessa's future baked
multi-body ephemeris fit.

The same executable also checks the harder current slice: all five circular
restricted three-body Lagrange points, five-period L4/L5 co-rotation, an
eight-burn impulsive maneuver schedule, and the actual 24-body design system
with an eight-burn Thessa vehicle replay. The design-system `halo` result is
reported as a diagnostic because its current Borea orbit is eccentric and
inclined while the halo segment is still circular and coplanar.

The command exits non-zero if the initial state conversion or the candidate
drift exceeds the duration-scaled validation gate. ANISE's `Orbit::at_epoch`
result is printed as a diagnostic only; the current inclined-LEO case exposes
a large discrepancy in that path, while Nyx's numerical propagator agrees with
the candidate.

Run from the repository root:

```bash
/usr/bin/cargo run --manifest-path validation/nyx-compare/Cargo.toml --release
```

Network access is needed once to download the isolated AGPL reference
dependencies. The root workspace commands do not build this harness.
