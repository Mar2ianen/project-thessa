# thessa-worldgen-rocky (dev tool, MIT)

Sim-first generator for **rocky planets only**. No clouds (separate system).

## Architecture: the FIELD is canonical, PNGs are consumers

```text
celestial body (data/system.toml)
        + world recipe / spec
                |
                v
      global planetary field
   /       |        |        \
height   biome    geology  climate
   |
spectral terrain bands (stable physical wavelengths)
   |
authored landmark overrides
   |
frozen global erosion + finer regional runoff displacement/infill
   |
   v
sample(direction, min_wavelength_m)
```

**GLOBAL RASTER != COMPLETE TERRAIN.** PNG/PGM previews (480x270,
1920x1080) are orbit-preview/debug caches. Close surface detail always comes
from the canonical field's physical-scale source and frozen erosion hierarchy,
not enlarged preview pixels. Frozen DEM spacing bounds the resolved erosion;
analytic detail below it is not a substitute for resolved channel geometry.

Native adaptive terrain and configured contact consumers sample the
same field; tiles/chunks are cache units and never change the terrain.

The offline compiler applies runoff-driven, volume-conserving erosion to the
canonical source. `PlanetField::with_frozen_erosion` explicitly consumes its
final-minus-source displacement, checks the source contract, and preserves fine
incised source detail, native height/prefix paths and the source datum. Fine
regional deposits apply finite-depth infill rather than fresh detail on top.
Loading does
not run erosion. The shared client/server launch setup now bundles and checks
`data/worldgen/thessa-erosion-v3.surface.json.gz`; failure is explicit, with no
silent analytic fallback. Runtime material consumers use the attached field,
not the historical artifact's regional material/tile caches.
It derives regional hydrology and material drivers; local river/wetland geometry
remains incomplete. GPT Image 2.5 maps are MACRO
style/region hints, never authoritative physics.

## Layers (`prompts/` hold color legends)

1. `height` — OPTIONAL macro hint, piecewise datum mapping
2. `albedo` — unlit base color hints
3. `biomes` — broad region hints (bake assigns taxonomy sites)
4. `roughness` — hint; final derived from biome/geology rules
5. `normal` — always DERIVED from final height, never painted
6. `hydrology` — hint; final routed downhill from terrain
7. `minerals` — hint; final placed by geology causality

## Resolution policy

Resolution-agnostic: gores are normalized fractions with overlap, all scales
in metres (macro 100-2000 km, meso 5-200 km, micro cm-km). Regenerate GPT
maps at any valid 2.5 size later without changing the manifest schema.

## Inhabited-world readability

Implementation checkpoint: climate contract, curved mountain arcs, branching
rifts, colocated plateau/basin uplift and regional drainage-driven wetlands are
implemented. The uncommitted disk worktree additionally derives ecological
biomes, catchment-threshold river reaches, connected lake/salt basin descriptors,
ecological scatter and inhabited regions conserving 150 million population.
Continuous ocean-distance drivers are interpolated, not nearest-cell sampled.
Local water geometry, vegetation rendering and infrastructure remain incomplete.
Ocean-supplied moisture decays with travel distance and cumulative ascent;
lake salt suitability compares routed annual liquid inflow with temperature-
dependent potential evaporation. Both are static proxies, not weather or actual
lake water inventory. Frozen schema 4 adds bounded, disjoint finer erosion
regions, with open-edge sediment accounting. Schema 3 records the pre-erosion reference; schema 2
remains readable but cannot be attached as an erosion delta without that reference.
See `docs/52_THESSA_LOCAL_EROSION_REFINEMENT.md` for local erosion/snow supply and
`docs/53_THESSA_SURFACE_MATERIAL_REVIEW.md` for rejected frames and current
geological/ecological material changes. None establishes full visual acceptance.

Some rocky worlds are inhabited rather than pristine terrain. Civilization is
a **derived world layer and visual/navigation context**, not a city-building
simulation.

For the inhabited moon target, use a total population on the order of
**150 million**. That is large enough that the moon must not read as empty from
orbit or during atmospheric/low-altitude flight, even though individual cities
are not simulated at parcel/building-management fidelity.

The intended footprint is globally sparse but globally present:

- settlements and a limited number of major urban regions follow water,
  terrain, resources, ports and transport access rather than uniform random
  placement;
- long-distance infrastructure connects population centres: roads/rail or
  equivalent surface corridors, power/utility routes, ports, landing sites and
  industrial/resource nodes;
- night-side emissive patterns and large-scale developed regions provide
  orbital readability without requiring every building to exist as gameplay
  state;
- progressively closer LODs may resolve coarse developed-region masks into
  road graphs, blocks, landmark structures and deterministic local scatter;
- the world generator owns the stable causal placement fields, while the
  renderer/runtime chooses how much of that infrastructure to materialize for
  the current scale.

This layer should answer "why is infrastructure here?" from existing world
state (hydrology, slope, biome/climate, geology/minerals and authored
landmarks). It should not turn `worldgen-rocky` into a demographic or traffic
simulator. Detailed city growth, zoning, economics and per-building agents are
explicitly out of scope for the near-term terrain/worldgen stack.

Vegetation follows the same rule: biome/climate/hydrology produce ecological
coverage fields, and close-range renderers materialize deterministic local
instances. A Carboniferous-inspired vegetation set is a desired inhabited-moon
art direction, with dense wet lowlands/floodplains and sparser vegetation where
climate, elevation or substrate suppress it.

## Usage

Offline surface compilation (never overwrites a completed artifact):

```bash
cargo run --release -p thessa-worldgen-rocky -- bake-world \
  --recipe data/worldgen/worldgen_recipe.toml --out target/thessa.surface.json.gz
cargo run --release -p thessa-worldgen-rocky --example canonical_audit -- \
  --frozen target/thessa.surface.json.gz
```

The second command audits existing data without regenerating maps or running
erosion. The frozen global DEM is regional (0.5 degrees, about 28 km), not a
metre-scale shoreline/channel mesh. Plant prototypes and instances are baked
renderer-neutral data, not an integrated vegetation renderer.
The bundled v3 artifact reuses the global v2 parent and adds three 256 m grids;
this still does not establish metre-scale water surfaces or a globally coupled
sediment budget. To refine an existing parent without repeating its bake:

```bash
cargo run --release -p thessa-worldgen-rocky -- refine-world \
  --recipe data/worldgen/worldgen_recipe.toml \
  --frozen data/worldgen/thessa-erosion-v2.surface.json.gz \
  --regions data/worldgen/thessa-regional-erosion.toml \
  --out target/local-refinement.surface.json.gz
```

Export matching globe fallback maps from existing data (no erosion rerun):

```bash
cargo run --release -p thessa-worldgen-rocky -- export-client-maps \
  --recipe data/worldgen/worldgen_recipe.toml \
  --frozen data/worldgen/thessa-erosion-v3.surface.json.gz \
  --out target/frozen-client-maps --width 2048 --height 1024
cargo bench -p thessa-worldgen-rocky --bench material_pages
```

`TerrainSample::erosion_displacement_m` combines frozen global/local displacement;
`local_deposition_m` keeps positive local displacement separately. Materials
distinguish `SurfaceAppearance::snow` (cold suitability) and `snow_cover`
(one-year precipitation/roughness-limited cover proxy, not a seasonal inventory)
and `frost_cover` (isolated optical frost weight). Geological stone/soil and
ground/canopy/reeds use distinct optical proxies with physical-texel filtering;
these are not rendered vegetation or measured spectral material textures.
`PlanetField::regional_parent_sample` is for stable survey scoring/debugging,
never contact/launch height.
`PlanetField::surface_geometry(direction, scale_m)` returns a central-difference
rise/run and tangent height Laplacian in 1/m (positive in hollows), including
landmarks and frozen erosion. Material consumers request a fixed 256 m scale;
`curvature_per_m` stays zero in samples without a requested geometry stencil.
This is a representation descriptor, not a local sediment or snow inventory.

```bash
cargo run -p thessa-worldgen-rocky -- check --manifest data/worldgen/thessa_demo.toml
cargo run -p thessa-worldgen-rocky -- sample --manifest data/worldgen/thessa_demo.toml --lat-deg 5 --lon-deg -40 --detail-scale-m 250
cargo run -p thessa-worldgen-rocky -- preview --manifest data/worldgen/thessa_demo.toml --step-deg 2 --out /tmp/pv
cargo run -p thessa-worldgen-rocky -- bake --manifest data/worldgen/thessa_demo.toml --step-deg 2 --out /tmp/report.json
```

Demo recipes: `data/worldgen/example_rocky.toml` (minimal),
`data/worldgen/thessa_demo.toml` (tectonics + landmarks),
`data/worldgen/moon.toml` (real Moon dimensions).
