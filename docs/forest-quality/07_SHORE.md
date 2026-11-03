# 7 — Shoreline integration

Status: reference/applicability reviewed; shoreline integration open.

## Research

[Unreal Single Layer Water](https://dev.epicgames.com/documentation/en-us/unreal-engine/single-layer-water-shading-model-in-unreal-engine) uses depth/color input, scattering/absorption and tiled reflection composition. Local `SingleLayerWaterShading.ush` was inspected for scene-depth and water-to-scene transmittance ownership. The mechanism separates water transport from decorative shoreline foam.

## Applicability review

Thessa already has planetary water levels, hydrology/wetland ecological covers, retained wet soil, `surface_water::fill_closed_basin`, local clipped puddles and a spectral ocean. Closed puddles do not constitute a resolved coastline. Wetland niches use relative elevation; wave foam must depend on wave/breaking state, not a white distance band. Unreal's custom pass cannot simply be pasted into the existing portable forward renderer.

## Implementation decision

Preserve water and terrain ownership while auditing a shared shore-distance/depth input for substrate and vegetation. No foam, invented waves, new RT pass or bright shoreline strip is added in this iteration. Follow the existing graphics resolver/fallback contracts for future reflection integration.

## Verification needed

Pin wet/dry shore transects and compare water depth, substrate moisture, plant eligibility and beauty. Validate shoreline alignment with terrain/ocean rather than camera proximity. SSR and confidence-driven fallback remain separately tracked; this item is not closed.

The executed 10 m crown view exposes a regular bright stipple/zigzag on the
water near the shore. This is recorded as an unresolved water sampling/aliasing
artifact, not accepted foam or a physically justified wave pattern.
