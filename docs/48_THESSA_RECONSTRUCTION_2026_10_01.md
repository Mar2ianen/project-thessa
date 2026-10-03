# 48 — Thessa reconstruction checkpoint

This archive preserves both dated working-copy snapshots. Their uncommitted/
unpromoted status describes those dates, not the current publication. Current
runtime and remaining acceptance work are tracked in docs 49–53.

## 2026-10-02 snapshot

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
## 2026-10-01 snapshot

Status: partial reconstruction in the isolated `/tmp/opencode/thessa-render-audit`
worktree. These changes are not yet promoted to the main source tree. No commit
or push. Main-tree thermal/resource work is deliberately excluded.

## Design boundary

Read together: doc 13 in full, atlas 02 (especially Thessa's captured-world
history), 01 celestial system, 02A atmosphere and 02D biosphere, plus both
worldgen TOMLs. Thessa is an old, cool, oceanic, geologically active captured
world, not Earth with shuffled continents or a fresh-crater showcase.

Current user clarification supersedes doc 13's old fresh-crater readability
target: impacts survive as eroded, buried or flooded geology, not mandatory
navigation landmarks. Wetlands, floodplains, deltas, sedimentary plains and
other relief families must be completed, not removed. Do not relocate drowned
craters to land simply to make them prominent. Keep `data/system.toml`
authoritative; the proposed stellar changes in doc 13 remain unapplied.

## Staged implementation

- Validated climate strengths and a 278 K area-mean static worldgen proxy now
  cross SpecRecipe -> Manifest -> PlanetParams. This is not a physical climate
  solver or a change to flight atmosphere thermodynamics.
- Authored geothermal province count ranges reach deterministic placement.
  Preferred latitude/longitude pairs stay paired; longitude normalization no
  longer moves provinces to the opposite hemisphere. Nominal flux remains
  0.146 W/m2; the full body-file heating contract is still incomplete.
- The dry plateau complex now contains colocated uplift and a salt basin.
  Large-crater depths use explicit 1–4 km recipe bounds rather than scaling
  simple-crater depth to enormous diameters. Live erosion remains incomplete.
- Mountain ranges now have a curved spine and connected tapered lateral
  ridges. Rift systems have connected side valleys; tributary intersections
  do not multiply the authored depth. Both use existing spherical placement,
  seeded footprint warp and finite feathering, not another terrain generator.
- Regional wetland support derives from existing priority-flood depression
  depth, terrain slope, aridity and downhill catchment area. Catchments use
  spherical SI cell areas and terminate at ocean; the shared accumulation
  routine still serves the original count-based API. Bilinear periodic context
  sampling avoids introducing nearest-cell rectangular wetland patches.
- Ocean/highland samples reject wetland support. Supported lowland plain sites
  become sedimentary Wetland; local material slopes suppress saturated-ground
  colour on cliffs. This is regional suitability, not resolved pond geometry,
  a local flood solver, complete river networks or finished delta terrain.
- Global export classifies materials at the same 32 m source as local pages,
  filters unresolved appearance bands by physical texel size and exports
  canonical roughness. Material slope reuses its already computed macro term.

## Measurements and artifacts

The new `canonical_audit` example samples 16,384 equal-area directions at a
32 m field cutoff and measures 49x49 spherical neighbourhoods around every
landmark, recording shape, seed, location and sampled extrema. These extrema
are observations, not certified global bounds.

Latest single-run **release CPU field audit**, not a renderer benchmark:

| Quantity | Observation |
| --- | ---: |
| Ocean area | 60.0281% |
| Mean surface proxy temperature | 278.0137 K |
| Mean geothermal flux | 0.146161 W/m2 |
| Mountain arc length / sampled neighbourhood peak | 1,532.865 km / 5,687.972 m |
| Rift length / authored maximum incision | 1,211.388 km / 3,773.867 m |
| Glaciated coast sampled neighbourhood peak | 7,848.270 m |
| Mean regional wetland potential | 0.00700246 |
| Wetland biome samples | 36 / 16,384 |

Wetland potential is a fractional suitability mean, not flooded area. The
480x270 image is an unlit equirectangular material preview, not a gameplay
capture or proof that all landmarks are readable from orbit.

Artifacts: `target/planet-refactor-audit/` in the main checkout, including
`arc-rift-release.json`, `wetland-release.json`, corresponding 480x270 previews,
test logs and checkpoint archives. `/tmp` reached its quota; the new audit
outputs use disk storage instead. Earlier `/tmp/opencode/planet-*.json` and
`thessa-v4` maps were inspected, not regenerated.

The existing `/tmp/opencode/planet-v4-indexed-coast.png` is INVALID for visual
acceptance: its log reports all three v4 material maps missing. A shared target
binary resolved assets in the main checkout, not the worktree. Do not interpret
its blank view as the new planet or as a repaired renderer.

## Validation and remaining work

156 worldgen library tests and 5 CLI tests pass in the isolated worktree.
Scoped all-target/all-feature clippy with warnings denied and formatting pass.
10 ignored RCBT tests execute on the Radeon 880M, including real hardware paths.
These are scoped checks, not whole-workspace push gates or visual acceptance.

Next: coherent continental/craton and plateau coverage, inherited impact erosion
with sediment transport/infill through the existing bake/field system, geology
linked basalt provinces, glacier/coast placement, local wetland and drainage
refinement, floodplains/deltas and surface-to-orbit agreement. Regional context
must not become the final metre-scale hydrology truth. Existing v4 maps predate
the arc/rift/wetland delta and must not be presented as matching current source.
Rectangular material transitions and loading latency remain unresolved; doc 47's
matched debug failure is still valid. No KSA/KSP2 parity or superiority claim.
