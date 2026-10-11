# Licensing policy

Status: accepted baseline, 2026-09-07.

## Split

Project Thessa is a mixed-license repository.

### MIT — reusable engine/tooling

Intended for:

- `sim-core` and future `sim-*` crates;
- generic vehicle/physics libraries;
- generic ephemeris/integrator tooling;
- reusable protocol primitives if they remain game-agnostic;
- reusable developer tools.

Goal: simulation technology is independently reusable/auditable without inheriting the game's copyleft.

### GPL-3.0-or-later — game code

Intended for:

- native/web game client applications;
- authoritative game server application;
- game-specific rules/progression/content code;
- game UI tied to Project Thessa gameplay.

GPL game code may link/use MIT engine crates. MIT engine crates must not depend on GPL game crates.

### Assets

Art, music, fonts, third-party datasets and trademarks have their own licenses. GPL code licensing does not automatically choose an asset license. Asset policy remains TBD.

## Dependencies

Permissive dependencies are preferred in MIT engine crates. LGPL/AGPL/GPL dependencies require an ADR that checks the exact linking/distribution consequences. A GPL game does not make it acceptable to accidentally change the reusable engine's effective licensing.

Current examples:

- `nyx-space` (AGPL): validation/reference by default, not an engine runtime dependency;
- `avian_fdm` (LGPL): reference/spike pending explicit dependency decision.
- `large_cbt` (AnisB, NO license upstream): vendored OCBT sources under
  `third_party/large_cbt` are quarantined as a dev-time benchmark oracle
  only (see `third_party/large_cbt.REVISION` for provenance, the recorded
  file fingerprints, and the compliance posture). The compiled objects link
  exclusively into dev-dependencies of `thessa-rcbt-core` and
  `thessa-rcbt-wgpu` (benches and hardware tests); no runtime library,
  binary, or asset links them, and Cargo never propagates a dev-dependency
  downstream. No large_cbt code is copied into the project. If upstream
  publishes a license, adopt it there immediately; if upstream objects,
  remove the vendor directory and the ffi binding the same day.

The current `validation/nyx-compare` package is an isolated AGPL
reference-only Cargo workspace. It is intentionally excluded from the root
workspace and is never linked by `thessa-sim-core` or the game applications.

## SPDX

Each Cargo package declares its own license. Do not put a single `workspace.package.license` over the mixed repository.
