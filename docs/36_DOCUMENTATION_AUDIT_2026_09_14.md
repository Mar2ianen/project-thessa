# Documentation and implementation audit — 2026-09-14

## Scope

This audit compares the repository documentation with the current checkout,
not with an aspirational release plan. The source snapshot at the start of the
audit was `e1454ac` on `feat/control-guidance-autopilot`; the working tree now
also contains the CBT integration described in
[`docs/37_CBT_INTEGRATION_STATUS_2026_09_14.md`](37_CBT_INTEGRATION_STATUS_2026_09_14.md).
`origin/main` was fetched and
is at `e329bf2`; the additional `laptop` remote could not be reached from the
current network, so its remote-tracking refs were not refreshed.

The working tree already contained user changes before this audit:
`Cargo.toml`, `Cargo.lock`, and the untracked `crates/maneuver/` crate. They
were preserved. The maneuver crate is included in the workspace and passes its
tests, but remains uncommitted user work.

## Verification performed

The following command completed successfully on this checkout:

```text
cargo test --workspace
```

All executed tests passed. The command still reports one compiler warning in
the uncommitted maneuver crate (`unused variable: momentum` in
`crates/maneuver/src/search.rs`). This audit does not alter code outside the
documentation scope.

Strict `cargo fmt --all -- --check` and
`cargo clippy --workspace --all-targets --all-features -- -D warnings` were
also attempted. They currently fail only inside that same uncommitted crate:
formatting differences in `lambert.rs`, `ops.rs`, and `search.rs`, plus four
Clippy findings (unused variable, negated partial comparison, too many function
arguments, and a manual range containment check). Existing code outside the
crate was not changed to hide those findings.

## Current implementation map

### Implemented crates and applications

| Area | Current source of truth | State |
| --- | --- | --- |
| Numerical core | `crates/sim-core` | Implemented prototype |
| SIMD helpers | `crates/simd` | Implemented optional kernels |
| Atmosphere optics | `crates/atmosphere` | Implemented shared visual model |
| Graphics settings | `crates/graphics` | Implemented renderer-independent resolver |
| Performance capture | `crates/perf` | Implemented capture model and tests |
| Flight control | `crates/flight-control` | Implemented guidance, laws, policy, allocation, actuators |
| Flight authority | `crates/flight-authority` | Implemented authoritative flight/runtime adapter |
| Autopilot graph | `crates/autopilot` | Implemented typed graph and event-driven waits |
| JavaScript blocks | `crates/autopilot-js` | Implemented sandboxed QuickJS bridge |
| CBT core | `crates/rcbt-core` | Implemented backend-neutral topology/page contracts |
| CBT Bevy adapter | `crates/bevy-rcbt` | Implemented universal frame scheduler; client-integrated |
| CBT wgpu adapter | `crates/rcbt-wgpu` | Implemented optional portable dispatch adapter |
| Wire protocol | `crates/protocol`, `crates/flight-net` | Implemented framed, validated input/snapshot path |
| Maneuver planning | `crates/maneuver` | Implemented prototype; uncommitted in this checkout |
| Server | `apps/server` | Implemented headless stdio/TCP authoritative shell |
| Client | `apps/client` | Implemented Bevy map/pilot/terrain/atmosphere slice |
| System baking | `tools/system-baker` | Implemented TOML to baked JSON |
| Vehicle baking | `tools/vehicle-baker` | Implemented vehicle asset compilation |
| Rocky world generation | `tools/worldgen-rocky` | Implemented offline field and texture pipeline |

### Implemented physics paths

- Baked analytic celestial ephemerides provide body states; runtime does not
  integrate planet/moon mutual dynamics.
- Ship gravity is full multi-body point-mass gravity over relevant physical
  sources; synthetic barycenter nodes do not double-count their children.
- Adaptive Dormand–Prince, velocity-Verlet, deterministic rigid-body stepping,
  sampled coast tables, and on-rails wakes are implemented.
- Gravity cohort patches compile an affine far field with exact-near terms,
  enforce explicit error budgets, and fail open to the exact path.
- `AffinePropagator` and `propagate_piecewise` use analytic frozen-patch
  segments only while the posted bound holds; they rebuild or fall back when
  it does not.
- Atmosphere, local `omega × r` flow, panel aero, stall/transonic/supersonic
  corrections, dynamic damping, and control-surface actuator response are
  implemented. This remains a reduced-order model, not CFD.
- Structural fracture, thermal graph coupling, fluid/electrical networks,
  factory production, and persistent logistics are not implemented in the
  current runtime.

## Documentation status policy

The repository now uses these labels:

- **Implemented prototype** — behavior exists in the current workspace and has
  tests or a runnable path.
- **Partial** — the document describes a shipped slice plus explicit missing
  pieces.
- **Design / future work** — an intended architecture or gameplay direction,
  not a runtime guarantee.
- **Reference / validation** — an external comparison or provenance record,
  not a runtime dependency.

The large world/gameplay, performance, atmosphere, and roadmap documents are
design references unless their status says otherwise. The current code and
the implementation documents are authoritative when a design document differs.

## Corrections made in this documentation pass

- Replaced mixed Russian/English prose in repository documentation with English.
- Rewrote the README around the current crate/application graph and marked
  factory, structural, thermal, persistence, and production multiplayer work
  as future or partial.
- Updated the decision index to include the accepted licensing, renderer,
  autopilot, and aerodynamic-boundary ADRs.
- Reclassified gravity cohorts and analytic affine propagation from proposal
  language to implemented prototype language, while keeping their error bounds
  and exact fallback requirements explicit.
- Removed stale descriptions of a purely hypothetical autopilot: the current
  graph, QuickJS, guidance, plan, wait, and server execution paths are now
  documented as implemented prototypes.
- Updated the 2026-09-12 optimization audit to point at the current source
  snapshot instead of an older commit and preserved its benchmark caveats.
- Added the CBT implementation map, universal Bevy adapter, cube-sphere
  address bridge, pure Rust/libcbt backend choice, optional wgpu adapter, and
  explicit GPU replacement gates.
- Kept world lore, release vision, structural/thermal systems, factory systems,
  and large-scale multiplayer clearly labelled as future design.

## Maintenance rule

When a crate boundary or public type changes, update the implementation map and
the nearest current implementation document in the same change. Do not turn a
proposal into an implementation claim without a source path, a test, and a
known limitation. Do not delete historical measurements; add a dated result
and state which commit/configuration produced it.
