# 1 — Spatial plant communities

Status: placement implementation criteria closed after tests, independent technical
review and capture inspection; overall scene/reference parity remains open.

## Research

[Unreal procedural foliage](https://dev.epicgames.com/documentation/en-us/unreal-engine/open-world-tools-property-reference-in-unreal-engine) separates seed dispersal, growth and shade competition. Local UE 5.8.4 `ProceduralFoliageTile.cpp`, `GetSeedOffset`, `AgeSeeds`, `SpreadSeeds` and shade-only passes were inspected. [Deussen et al. 1998](https://doi.org/10.1145/280814.280898) is the ecosystem research target; the PDF fetch failed, so its detailed algorithm is not claimed as reviewed. OpenEngine `GrassClumpModel.h` explicitly preserves even placement while grouping blade attributes: useful for tufts, insufficient for stand-scale gaps.

## Applicability review

Existing Thessa scatter has immutable spherical IDs, regional ecological eligibility, an 8 m understory patch field and nested quality thinning. Adult plants bypassed the local patch field. A full seed simulation copied from tile-local Unreal would require boundary neighborhoods and stable ordering; it is not a drop-in replacement. Use the existing canonical surface callback and retain the generic scatter density/area contract. Bare soil must retain pebbles; moss can persist in glades.

## Implementation

`scatter::vegetation_community_occupancy` is a smooth, bounded 24 m world-fixed habitat prior. `community_plant_surface` multiplies the three existing ecological covers; native canopy and understory use that adapter. Original canonical substrate, stone sampling, water and slope remain unchanged. Grass consumes the same occupancy with its existing moisture gate. Maximum density and admission ceilings do not increase. Existing IDs/positions are retained for surviving objects; membership changes intentionally. No neighbor shading or biological succession is claimed.

## Verification and remaining work

Tests check true zero-occupancy gaps, fully occupied stands, centimetre-scale continuity and cube-edge continuity. A canonical-field test verifies only covers change, retaining ground/slope/water. Existing query partition, area-density, identity and quality-subset oracles still run unchanged. Fixed-camera GPU evidence is indexed by doc 132. Review the actual gap sizes before changing abundance; add neighbor-dependent eligibility only through this owner with query-boundary tests.

The first 64 m candidate was rejected as too sparse near this camera. The retained
24 m prior makes fully empty cores rarer, with transition thresholds -0.65/-0.10
on the existing value-noise field. Its current specimen still has lower total
plant abundance; this is not proof of equal-cost or superior forest quality.

## Sequential completion — neighbor-aware communities

Implementation acceptance for this item requires: correlated stands/glades;
existing climate/water/slope eligibility; actual neighboring tree root/crown
influence; quality-stable source neighborhoods; cube-face continuity; bounded
source/candidate costs; technical review and an actual scene capture. Full
biological growth/competition is not required by this rendering task.

The adapter now builds a body-fixed 32 m spatial index from the full-density
community-filtered tree population, before visual quality thinning. Canonical
coarse tree IDs are thinned through one stable hash on both coarse and near
contexts; Low/Medium membership intentionally changes from the former direct
density threshold. High membership remains. The full source shares the existing
9×4096 candidate and output ceilings; lower quality may perform more canonical
CPU sampling than previously. No new per-frame GPU work or render pass occurs.

Crown centers/radii reuse the actual woody sockets and the same dimensions as
the pinned foliage renderer. Stem-base exclusion uses the scaffold's conservative
near-root radial extent. These are ecological proximity/clearance proxies,
not a sky-visibility, photon transport or exact collision solver. Ferns tolerate
shelter; reeds and short grass favor gaps; small plants are rejected where their
footprint overlaps the conservative root exclusion. No extra shading AO is used.
The query halo expands from geometric reach (large scaled trees may exceed a
single 32 m cell); it is not hard-coded to 27 cells.

Window queries now reproject beyond cube edges instead of clamping to one face.
Tests cover all six faces/corners, root exclusion, smooth crown proximity,
different fern/reed response, input-order invariance and canonical neighborhood
counts across quality. Existing coarse/near identity and quality-subset tests
remain. The standard FHD capture retains 20 frozen diagnostic views. Source and
review evidence: `docs/artifacts/forest-neighbor-communities-2026-11-04.json`.
