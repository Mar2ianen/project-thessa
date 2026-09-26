# 06 — Open questions

Status: living index. Items close by landing the decision in the relevant
implementation document or ADR and striking them here (see `docs/00_STATUS.md`
for the implemented/future map).

These items are intentionally not locked in the current prototype. Accepted
decisions belong in the relevant implementation document or ADR.

## World and lore

- final project and body names;
- final lore hook for the Thessa colony;
- native biosphere and habitability details;
- how much formation history the player needs to know.

## Celestial data

- canonical epoch and orbital phases;
- resonant offsets and libration amplitudes for the Nereid chain;
- Nix tidal `Q/k2` and lifetime;
- final Cinder orbit;
- Nereid obliquity and ring tilt; frame-explicit spin-axis azimuth/definition and one canonical body-orientation contract shared by render, J2/C22, terrain and rings;
- body-specific atmosphere profiles and composition;
- spin periods and prime meridians for triaxial bodies; Jn beyond degree 2;
- weather and climate fields;
- a fully consistent Halo co-orbital solution.

The current `data/system.toml` target for Thessa (`R=3200 km`, approximately
`0.500 g`, `p0=1.20 bar`) is not an open value, but it is still not canonical.

## Gameplay

- player inventory and cargo abstraction;
- exact research/unlock rules;
- maintenance intensity and failure consequences;
- player death and vehicle-loss rules;
- whether contracts/economy exist;
- how much manual flight is required before route certification;
- minimum standard library for MechJeb-like guidance blocks (minimum shipped:
  `Ascent`, `LandAt`, `ExecuteManeuver`, `Rendezvous`-approach as parameterized
  native subgraphs — `docs/07_AUTOPILOT.md`; completion of the rest still open);
- graph ownership semantics across staging and docking (validator already
  rejects ambiguous controller ownership; physical separation, topology
  mutation, and child-vehicle ownership transfer still open — `docs/07 §7.7`);
- whether route automation requires a successful manual/reference flight.

## Vehicle editor

- cross-section parameterization and material thickness UX;
- depth of engine-cycle design;
- procedural wheel/gear editor UX (the physical wheel-chassis parameter and
  Rapier boundary are specified in
  [`details/05_PROCEDURAL_LANDING_GEAR.md`](details/05_PROCEDURAL_LANDING_GEAR.md));
- structural/thermal graph visualization;
- shared design asset and instance storage;
- design validation and control-authority reports.

## Physics

The Tier-A aero baseline and gravity cohort/affine paths are now implemented
prototypes rather than open architecture choices. Remaining physics questions:

- wake/occlusion geometry and exposure compilation;
- whole-vehicle versus per-zone coefficient tables;
- table axes for beta, control deflection, Reynolds, and dynamic derivatives;
- structural solver and fracture order;
- reduced-order aeroelasticity;
- slosh fidelity;
- atmospheric heating and ablative heat shield model;
- CPU ray-sampling budget;
- deterministic tolerance policy across AVX builds;
- spin periods and prime meridians for triaxial bodies (hyperbolic/parabolic osculating
  coverage has landed in `sim-core`; J2/C22 evaluation landed too, higher-degree Jn still open).

## Runtime

Decided: Rapier is the local contact/constraint solver (zero global gravity,
Thessa-sampled wrenches, authoritative readback — `docs/40_RAPIER_COLLISION_INTEGRATION.md`).
Avian versus Parry-only is no longer an open choice. Remaining runtime questions:

- Lightyear or another production replication layer after a spike;
- save database and migration format;
- web client scope;
- prediction model for remote craft;
- production transport choice (UDP/QUIC/WebTransport/etc. — stdio/TCP headless
  transport is the current spike, not the production decision);
- Windows packaging policy for the internal wgpu backend.

## Performance targets

Benchmark-backed targets are still needed for:

- 1080p Low/Medium frame rate;
- active atmospheric craft at x1;
- warp with 1k/5k/10k vehicles;
- factory object count;
- thermal/structural nodes per design;
- acceptable memory on 24 GiB systems.

## Licensing and distribution

The engine MIT/game GPL-3.0-or-later split is accepted. Remaining questions
include asset/music licenses, generic versus game-specific protocol boundaries,
LGPL dependencies inside MIT crates, and whether AGPL tooling remains isolated
validation only.
