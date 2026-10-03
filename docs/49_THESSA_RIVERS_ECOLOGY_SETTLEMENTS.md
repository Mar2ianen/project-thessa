# 49 — Thessa hydrology and ecological biomes

Status: earlier partial checkpoint, superseded where noted by
`50_THESSA_MOISTURE_EROSION_BRIDGE.md`. Uncommitted implementation in the isolated disk worktree
`work/thessa-planet-rebuild`. The main checkout is not updated. This checkpoint
continues doc 48 and the later river/ecology/settlement slice; it is not visual
acceptance of the planet or renderer.

## Implemented contracts at this earlier checkpoint

- `SpecRecipe -> Manifest -> PlanetParams` carries downhill river routing,
  physical catchment threshold, lake and closed-basin salt-flat switches.
- Water-grid classification and canonical `RiverReach` descriptors use the
  same strictly-downhill receivers and spherical catchment areas. Hashed
  coordinate-based river seeds are removed. Flat cells and closed pits remain
  terminal; lake-throughflow routing is not implemented.
- Priority flood retains exact spill elevations, without per-cell epsilon
  slopes or millimetre-quantized heap ordering. Connected submerged land cells
  at the same spill level form a `LakeBasin`; longitude is periodic. Basins
  with maximum depth greater than 60 m include their shallow margins. Area and
  capacity use spherical SI cell areas. Basin-mean aridity selects fresh-water
  suitability versus salt-flat suitability, not a water balance simulation.
- `PlanetField::lake_basins()` and `world_layers.json` expose these regional
  descriptors. Capacity is a geometric capacity at spill level, not an actual
  water inventory. Pole rows retain the inherited outlet convention.
- Primary ecological biomes use temperature, moisture, elevation, saturation
  and continentality: forests, grassland, steppe, dry/cold barren ground,
  maritime plains, alpine meadow/barren ground, snow and wetlands. Geology
  stays separate: an inherited impact floor can support a forest or wetland.
  Active volcanic/saline sites retain their identity. This is a static worldgen
  proxy, not plant physiology, seasonal weather or a climate solver.
- Ocean-distance drivers now use periodic bilinear sampling rather than
  nearest-cell steps; pole endpoints share one mean. Only continuous drivers
  are interpolated, not discrete biome/geology IDs. This removes a source of
  rectangular climate transitions, not the diagnosed renderer material bug.
- Existing ecological coverage/scatter and inhabited-region placement are
  preserved. Forty-eight separated suitable regions conserve the authored
  150 million population. Buildings, roads and vegetation rendering are absent.

## Remaining acceptance work at this earlier checkpoint

Regional drainage still comes from a 2-degree grid, approximately 112 km
meridional spacing. Do not inflate these edges into finished river channels or
claim their interpolated arcs are continuously downhill in the analytic field.
Lake descriptors are not rendered water surfaces or resolved shorelines. Local
channel carving, lake shoreline refinement, floodplain/delta sediment transport,
and wetland ponds remain necessary. Live-field erosion/infill is still missing;
ecological classification does not destroy crater relief. Native mesh integration
is preserved. Rectangular material-page transitions and loading latency remain
unaccepted; no KSA/KSP2 parity or superiority claim is established.

## Artifact and worktree provenance

The old `/tmp/opencode/thessa-render-audit` files remain untouched after copying
to a newly registered detached worktree on disk. Its old `.git` pointer shares
metadata with another worktree and must not be used for future index operations.
Disk storage avoids the `/tmp` quota; compiler scratch uses the approved disk
`target/tmp`. Tests use crate-local `target/test-tmp`.

Earlier maps and reports were inspected, not regenerated. New measurements live
under `target/planet-biomes-audit` in this worktree. They are CPU field audits,
not renderer benchmarks or visual proof. Existing `thessa-v4` images are stale
relative to this implementation.

## Earlier validation and observations (2026-10-02; historical snapshot)

The release CPU field audit sampled 16,384 equal-area directions at a 32 m
cutoff without regenerating a texture. Observed ocean area is 60.0281%, mean
proxy temperature 278.0137 K, 67 river reaches with 22 ocean outlets, and 205
regional fresh-water-suitable lake basins. Ecological samples include 645 cool
forest, 438 temperate forest, 1,182 grassland, 504 steppe and 50 wetland sites.
These are sample counts, not certified geographic bounds or rendered coverage.

The audit contains no salt-suitable lake basin and no dry-desert biome samples:
dry-interior/salt-basin coverage remains a real design gap, not a completed
feature. Field construction took 73.48 ms in this single release run. Material
page builds took 55.55–66.30 ms for four separately sampled levels. These are
CPU audit observations, not loading latency, frame time, cold-cache results,
statistical benchmarks or evidence of a renderer speedup.

168 worldgen library tests and 5 CLI tests pass. Scoped all-target/all-feature
clippy with warnings denied and workspace formatting pass. Ten ignored RCBT
tests execute successfully on the Radeon 880M, including actual GPU paths.
Logs live alongside the audit. Full-workspace push gates have not been run;
no commit or push is requested. The existing native material/mesh integration
is not changed by this hydrology/biome delta.

The client-bin test build is blocked by an unrelated source inconsistency:
`crates/flight-authority/src/runtime/advance.rs:547` initializes
`ReactionWheelAllocation` without its `momentum_saturated` field (E0063).
The physics source is intentionally not edited. This blocks a new client-frame
validation, even though the isolated worldgen and real GPU tests pass.

## Current continuation

Doc 50 records the newer ocean-supply/orographic moisture, routed annual basin
balance and explicit frozen-erosion adapter. This addresses the missing dry
biomes and the previously disconnected erosion result without claiming local
water geometry or visual acceptance. The default client/authority launch path
still does not load the new offline artifact, and the unrelated physics build
blocker remains untouched. Measurements above are kept as historical observations.
