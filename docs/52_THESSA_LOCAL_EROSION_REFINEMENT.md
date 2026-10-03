# 52 — Local erosion refinement of the preserved Thessa parent

Status: partial, isolated uncommitted continuation, 2026-10-03.
No commit, push or main-checkout promotion. Doc 51 is the preceding runtime/
material checkpoint, not evidence of visually accepted terrain.

## Diagnosis and scope

The actual highland bookmark is 38° N, 171° W, on the existing
`dry-plateau-0`: its source contribution is 4926 m. The v7 launch field at that
point is 3395 m, about 268 K, with moisture 0.140 and regional displacement
-1204 m. It is not a missing plateau or a moist alpine snowfield. Independent
fine height bands survived the 0.5°/28 km erosion bake, and temperature-driven
snow covered an arid plateau almost as readily as a wet mountain.

This continuation refines existing geometry, rather than adding another random
landmark layer or darkening the rendered image. The completed global parent,
its 736 reaches, 252 basins, ecological material/vegetation snapshots and earlier
native cache remain untouched in `thessa-erosion-v2.surface.json.gz`.

## Frozen hierarchy and shared consumers

`refine-world` loads that parent, checks the live source identity and every
parent DEM vertex against the attached field, and runs the existing SI runoff
erosion/transport pipeline on explicitly configured finer regions. No global
erosion generation is repeated. Schema 4 extends the existing `FrozenSurface`
format and its bounded gzip loader/atomic no-overwrite publisher; schemas 2/3
retain their existing read semantics. The new artifact is
`data/worldgen/thessa-erosion-v3.surface.json.gz`.

Each region stores its physical recipe, regular unwrapped latitude/longitude
coordinates, pre-refinement source heights, final-minus-source displacement,
source cutoff, exported sediment and volume residual. Validation checks finite
coordinates/arrays, declared spacing, zero edge displacement, sediment budget,
unique ids and disjoint domains. Dateline-crossing coordinates are valid;
nonpolar limits and per-region/total budgets are explicit. All regions belong
to this parent artifact; unrelated parent fields fail refinement.

`PlanetField` consumes the same immutable hierarchy in direct height, shared
prefix height, full sample and prefix sample paths. Native GPU/CPU meshes,
material classification, launch/contact queries and global map export share
those paths; no renderer-only displacement is introduced. The shared launch
bundle uses v3. This checkpoint used v9 fallback assets; doc 53 records the
subsequent material revision and active v10 assets.

The parent DEM's stored material/ecology/vegetation/native-tile snapshots remain
historical coarse data. `FrozenSurface::height_m` samples that global DEM, not a
complete fine analytic field; live consumers require the attached `PlanetField`.
Global river/lake descriptors are still regional, not fine channel/water meshes.

## Physical model and boundaries

`data/worldgen/thessa-regional-erosion.toml` declares three 512²-cell domains at
nominal 256 m spacing (513² vertices, about 131 km wide), centred on the original
highland, coast and volcanic bookmarks. The source is band-limited at twice the
cell spacing (512 m), rather than aliasing metre-scale noise into a coarse DEM.
Actual east/west spacing and cell areas vary with latitude and use spherical
geometry. Each region evolves for 128 x 10,000 years with the existing stream
power erodibility and 2 km sediment settling length.

Regional `HeightGrid` explicitly disables longitude wrapping. Its shared
drainage receiver selects maximum downhill slope among axial/diagonal neighbours
at their physical distances. Global periodic grids retain their previous
four-neighbour receiver/tie ordering. Priority-flood and basin traversal respect
regional side outlets; opposite sides of a local grid are not neighbours.

Precipitation/runoff comes from the existing climate proxy. Incision, transported
load, exponential settling and terminal basin infill are the existing
`erode_runoff` mechanism, not decorative channels. Outer edge vertices retain
the parent surface and are open sediment outlets. Exported volume is tracked,
not deposited as an artificial edge wall or silently lost in the budget:

```text
sum(cell_area * displacement) + exported_sediment = numerical residual
```

The outer domain is context for the central survey region. Missing incoming
catchments from outside that domain and deposition beyond its open outlets
remain unresolved: this is not a globally coupled erosion solve. Displacement
is continuous to zero at the edge, but a global C1-normal match and an apron
convergence error envelope are not established.

Incised regions retain unresolved source relief. Deposits instead reduce the
magnitude of fine source relief by the available deposition depth, towards the
resolved reference, rather than adding fresh noise on top of sediment. If the
source fine residual is `r` and deposited depth is `d`, the remaining residual
is `sign(r) * max(abs(r)-d, 0)` and the resolved surface rises by `d`. Exposed
peaks initially receive less cover and hollows receive more; zero/thin deposits
cannot erase a whole depression. Symmetric +/- residual pairs preserve the
specified mean deposition exactly; arbitrary sub-cell geometry is unresolved.
At the source cutoff, sampled vertices reproduce stored source + displacement.
The sediment budget is verified at that resolved grid scale; sub-cell sediment
inventory is not reconstructed. At 32 m, the retained source bands below 512 m
bound the fine-height/depositional-envelope difference (about 29.13 m for the
current knobs); this is not a metre-scale volume or contact-error guarantee.
`TerrainSample::local_deposition_m` exposes positive fine-region displacement
separately from the often-negative coarse regional displacement, so material
smoothness/cover can follow local deposits rather than ignoring them.

Coast/highland survey scoring uses `regional_parent_sample` to avoid selecting an unrelated
unrefined neighbour merely because fine erosion changed the score. Actual final
land height still rejects submerged choices and controls spawn/contact. This
parent sample is geological/debug metadata, not a collision-height API. Doc 53
corrects the geothermal canyon's misleading volcanic bookmark; the preserved
third refinement does not move with that camera-selection correction.

## Snow supply, not a contrast adjustment

Existing cold suitability is unchanged. Visual cover now also uses a static
one-year availability proxy: precipitation x moisture² x solid fraction,
converted from water equivalent to settled snow at 300 kg/m³, divided by the
source's 32 m roughness amplitude. Thin cover therefore leaves an arid plateau's
substrate exposed; concave ground still favours accumulation and steep ground
favours exposed rock. `SurfaceAppearance::snow` retains suitability and
`snow_cover` reports supply/geometry-limited cover separately.

This is not a dynamic seasonal snowpack, wind redistribution, sublimation,
glacial evolution or a measured annual inventory. No climate temperature,
precipitation field, biome or exposure setting is changed to make the image
darker. The existing material quality/filtering pipeline consumes this state;
no new plugin, environment-controlled effect or graphics prerequisite is added.

## Measured offline result

One release refinement/publication run processed three 513² grids in 57.783 s
(compilation excluded, publication included); it reused the preserved parent.
These are offline construction observations, not frames or a loading speedup.

| Domain | Displacement range | Exported sediment | Volume residual |
| --- | --- | --- | --- |
| Dry highland plateau | -805.1 .. +623.9 m | 1.3941e11 m³ | -3.05e-5 m³ |
| Humid coastal lowland | -193.7 .. +137.9 m | 2.3759e10 m³ | +3.81e-6 m³ |
| Geothermal canyon (historical volcanic bookmark) | -1038.5 .. +661.2 m | 1.9142e11 m³ | -1.22e-4 m³ |

Tests cover open-edge volume accounting, diagonal drainage, preserved parent
DEM, exact source-cutoff vertices, direct/prefix/sample agreement, burial of
fine source relief, dateline sampling, invalid coordinates/edges/overlaps and
precipitation-limited snow without relabelling climate suitability. Initial
worldgen validation passed 186 library and 5 CLI tests; subsequent validation
and real frame review are recorded in the audit directory. No visual acceptance
or whole-system rendering superiority claim follows from these tests.

## Reproduction and remaining work

```bash
cargo run --release -p thessa-worldgen-rocky -- refine-world \
  --recipe data/worldgen/worldgen_recipe.toml \
  --frozen data/worldgen/thessa-erosion-v2.surface.json.gz \
  --regions data/worldgen/thessa-regional-erosion.toml \
  --out target/local-refinement.surface.json.gz
cargo run --release -p thessa-flight-authority --example terrain_survey_audit
```

Native shadow casting/receiving, resolved lake surfaces/shorelines and wetland
ponds, fine drainage channels and floodplain/delta integration, sediment export
coupling, landscape/apron convergence, glacial history and rendered vegetation/
infrastructure remain incomplete. Structural arcs/rifts/plateaus are reused,
but whole-world middle-scale diversity is not established by three regions.
Matched final release frames must verify basin/saddle readability, loading,
coast/highland/volcanic differences and surface-to-orbit transitions before
claiming visual completion.
