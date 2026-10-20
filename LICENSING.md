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

`crates/render-core` is MIT: it contains only backend-neutral f64 rigid-frame
geometry, camera inputs and static triangle/material/rigid-instance descriptors
(glam math, no game content, spin model or policy).
`crates/render-assets` is MIT: a bounded embedded static-GLB decoder using
permissively licensed `gltf` 1.4 (`MIT OR Apache-2.0`) and `image` 0.25
(`MIT OR Apache-2.0`), already present in the dependency graph. The production
decoder library embeds no game asset. Its tests/benchmarks reference the existing
CC BY 4.0 X-15; the GPL native application embeds that model for its optional
diagnostic preview. Binary distribution must retain cmoreau's attribution and
the source/license links in `assets/models/north_american_x-15_plane.LICENSE.txt`.
The existing flight terrain spin/axis convention remains GPL in
`flight-authority::presentation`; the GPL native viewer samples it alongside
baked ephemerides and combines it with the reusable presentation geometry.
Checked `flight-authority::presentation::FlightPose` and the old embedded
client's extracted `flight-net::presentation` snapshot interpolation remain
GPL game code. No game mapping or interpolation policy is moved into MIT
render libraries. Hosts still own receive clocks, inboxes and physics demand.

Goal: simulation technology is independently reusable/auditable without inheriting the game's copyleft.

### GPL-3.0-or-later — game code

Intended for:

- native/web game client applications;
- authoritative game server application;
- game-specific rules/progression/content code;
- game UI tied to Project Thessa gameplay.

`crates/terrain-stream` is GPL-3.0-or-later: it contains the game client's
terrain selection/streaming policy, extracted without relicensing it. The MIT
`rcbt-core` and `render-wgpu` crates do not depend on it; the GPL client/viewer
combine that policy with the reusable renderer.

GPL game code may link/use MIT engine crates. MIT engine crates must not depend on GPL game crates.

The native plume port follows that boundary: its provisional vehicle/nozzle
deck and retained optical texture bake stay GPL in `apps/terrain-native`.
MIT `render-wgpu` consumes immutable axial samples and a caller-provided texture;
its medium integration follows MIT `plume-core` and its modal shell deformation
is independently implemented, not a relicensed legacy game shader.

The retained docking fixture is ported GPL-to-GPL into `apps/terrain-native`;
its D1 source/proxy calibration remains game-owned. The explicit static-rest-pose
option in MIT `render-assets` contains only generic GLB decoding policy and no
copied game fixture or asset. Animation sampling/skin support is not implied.

The city-mask optical bake is directly ported GPL-to-GPL into native game code.
MIT `render-wgpu` only uploads caller-supplied R8 bytes to the existing shared
terrain-shader consumer; no GPL settlement envelope or speckle bake is moved
into reusable engine crates.

### Assets

Art, music, fonts, third-party datasets and trademarks have their own licenses. GPL code licensing does not automatically choose an asset license. Asset policy remains TBD.

## Dependencies

Permissive dependencies are preferred in MIT engine crates. LGPL/AGPL/GPL dependencies require an ADR that checks the exact linking/distribution consequences. A GPL game does not make it acceptable to accidentally change the reusable engine's effective licensing.

Current examples:

- `egui`, `egui-wgpu`, `egui-winit` 0.37 (`MIT OR Apache-2.0`): optional
  default-enabled HUD dependencies of `apps/terrain-native`; no Slint or GPUI
  dependency/backend fork is introduced. The application remains GPL game code;
  the reusable renderer remains MIT and has no UI toolkit dependency.
  All newly resolved toolkit dependencies offer permissive terms; in particular,
  `self_cell` is used under its Apache-2.0 alternative, not GPL-2.0-only.
  HUD text reuses `assets/fonts/NotoSans-Regular.ttf` under OFL-1.1; its existing
  attribution and license in `assets/fonts/` must accompany binary distribution.
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
