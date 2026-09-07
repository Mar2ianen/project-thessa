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

The current `validation/nyx-compare` package is an isolated AGPL
reference-only Cargo workspace. It is intentionally excluded from the root
workspace and is never linked by `thessa-sim-core` or the game applications.

## SPDX

Each Cargo package declares its own license. Do not put a single `workspace.package.license` over the mixed repository.
