# Bevy visual slice

Status: implemented prototype (CPU fallback default; GPU-indexed CBT plus
baked beauty/plume opt-in via `graphics.toml`).

## Status

**Implemented prototype.** The client is a Bevy 0.19 application that reads the
same design system as the baker and uses `BakedEphemeris` for body positions and
periods. It is not yet a networked production client, but the default local
path already consumes snapshots from an embedded authoritative server process.

## Current features

- dark 3D hierarchical system map with deterministic star field;
- physical design bodies and barycentric coordinate anchors;
- checked-in textures and procedural PBR material fallbacks;
- body labels, parent hierarchy, orbit lines, and target selection;
- full-system overview plus local Asterion A, Nereid, Orthea, Vesper, and B/C
  binary modes;
- explicit map scale and readable visual radius scaling;
- smooth simulation-time display and map camera navigation;
- atmosphere optics shared with the `thessa-atmosphere` crate;
- pilot scene with X-15 visual asset, navball/PFD, flight HUD, terrain, water,
  performance overlay, and flight tracing;
- streamed rocky terrain tiles with parent retention during refinement (CPU
  fallback path; `terrain=gpu_indexed` selects the opt-in CBT raster with
  material pages instead);
- raster water reflection baseline and optional graphics-setting resolution;
- baked beauty shells (cloud decks, gas-giant bands, aurora) and field-first
  engine plume, all gated by `graphics.toml`;
- render-local anchoring so large barycentric coordinates do not jitter.

## Coordinate contract

Authoritative positions remain `f64` SI values from the simulation/server. The
client converts them to a render-local frame near the camera or selected body.
`bevy::Transform` is visual output only. Visual radius exaggeration is allowed
for map readability and blends toward true scale near the camera.

The client interpolates visual poses between snapshots. HUD telemetry and
control decisions retain the authoritative metadata and are not recomputed
from the interpolated transform.

## Launch

```bash
cargo run -p thessa-client
cargo run -p thessa-client -- --local
```

The regular path starts a local server process and communicates over framed
stdio. `--local` is a legacy diagnostic path that steps the authority in the
client process.

## Known limits

- the visible system uses design-target analytic ephemerides;
- full online multiplayer, authentication, persistent saves, and production
  interest management are not implemented;
- the authoritative terrain contact boundary is spherical/sampled even though
  the client can render richer generated terrain;
- atmosphere, water, clouds, and RT effects are visual reduced-order systems
  (clouds/aurora/gas-giants/plume ship as cheap CPU-baked shells plus a
  camera-facing volume ribbon; volumetric clouds and full RT remain future);
- rendering backends for the current client must continue to go through
  Bevy/wgpu abstractions (see `docs/39` for the longer-term migration plan);
- a WASM/WebGPU client target is future work.

## Verification

Client tests cover render-frame mapping, body selection, map modes, visual
radius blending, camera anchoring, atmosphere material data, terrain streaming,
water cubemaps, pilot frames, navball projection, and X-15 axis/orientation
contracts.

```bash
cargo test -p thessa-client
cargo test --workspace
```
