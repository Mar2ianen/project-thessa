# 48 — Thessa reconstruction checkpoint

Status (2026-10-02): partial implementation on the local
`feat/thessa-landscape-reconstruction` branch in the isolated
`/tmp/opencode/thessa-render-audit` worktree. Not promoted into main or pushed.
Unrelated main-tree thermal/resource work is excluded.

## Implemented boundary

The existing SpecRecipe -> Manifest -> PlanetField pipeline now preserves
validated ocean bounds, climate strengths, a 278 K area-mean static worldgen
proxy and geothermal province counts. Province site coordinate pairs and
longitude normalization are repaired. This is not a climate solver or a change
to flight atmosphere physics. Nominal geothermal flux is still 0.146 W/m2;
full body-file heating propagation remains incomplete.

The dry plateau complex contains colocated uplift and a salt basin. Authored
crater depths replace enormous simple-crater aspect-ratio depths. Live erosion
and sediment infill remain incomplete; impacts are geological history, not
mandatory fresh-crater landmarks.

Mountain arcs have curved spines and connected lateral ridges. Rift systems
have connected tributaries without multiplying maximum depth at intersections.
Regional wetland support reuses depression depth, terrain slope, aridity and
spherical catchment area with ocean outlets. Periodic bilinear sampling and
pole-cap averaging avoid nearest-cell wetland rectangles. Ocean/highlands are
excluded; supported plain sites become sedimentary Wetland. This is regional
suitability, not resolved ponds, river channels, floodplains or delta geometry.

Global export uses canonical 32 m material classification, physical spectral
appearance filtering and canonical roughness. Material slope reuses macro
samples. Earlier thessa-v4 maps predate the arc/rift/wetland delta and are not
promoted as current assets. Renderer mesh/indexed integration is retained as
separate uncommitted work; no rendering acceptance is implied.

## Evidence

156 worldgen library tests, 5 CLI tests, 75 client tests and 10 real hardware
RCBT tests passed on the Radeon 880M. Scoped worldgen all-target/all-feature
clippy with warnings denied and formatting passed. Not whole-workspace push
gates. The equal-area audit uses 16,384 samples at 32 m and 49x49 spherical
landmark surveys; observed extrema are not certified bounds.

Single-run release CPU field observations: ocean 60.0281%, temperature mean
278.0137 K, geothermal mean 0.146161 W/m2, mountain arc neighbourhood peak
5687.972 m, glaciated coast neighbourhood peak 7848.270 m, regional wetland
potential mean 0.00700246 and 36 Wetland samples. These are not renderer
benchmarks. Artifacts/checkpoint are in the main checkout's
`target/planet-refactor-audit/` because `/tmp` reached its quota.

The earlier planet-v4 coast screenshot is invalid: all three material maps
failed to load from the shared binary asset root. Doc 47's matched rectangular
material defect and streaming-latency failure remain open. No KSA/KSP2 parity
or superiority claim.

## Next contract

Rivers, ecological cover and civilization must reuse the same terrain,
hydrology and climate. The approximately 150 million population target and
Carboniferous-inspired wet-lowland vegetation direction are specified in
`tools/worldgen-rocky/README.md`. Settlement placement follows water, gentle
terrain, climate, resources, ports and transport; demographic/traffic simulation
is out of scope. Close vegetation, city geometry, night emission, connected
infrastructure, erosion/infill, cratons/plateau coverage and surface/orbit
visual acceptance remain incomplete.
