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
   v
sample(direction, min_wavelength_m)
```

**GLOBAL RASTER != COMPLETE TERRAIN.** PNG/PGM previews (480x270,
1920x1080) are orbit-preview/debug caches. Close surface detail always comes
from sampling the field at shorter wavelengths, never from upscaling raster.
Raster resolution is mostly irrelevant to surface detail.

Future consumers (adaptive cube-sphere renderer, collision mesher) sample the
same field; tiles/chunks are cache units and never change the terrain.

Pipeline: authored macrostructure (tectonic boundaries + landmark features)
runs through deterministic erosion, then derives consistent geology/biome,
hydrology, roughness, minerals and normals. GPT Image 2.5 maps are MACRO
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

```bash
cargo run -p thessa-worldgen-rocky -- check --manifest data/worldgen/thessa_demo.toml
cargo run -p thessa-worldgen-rocky -- sample --manifest data/worldgen/thessa_demo.toml --lat-deg 5 --lon-deg -40 --detail-scale-m 250
cargo run -p thessa-worldgen-rocky -- preview --manifest data/worldgen/thessa_demo.toml --step-deg 2 --out /tmp/pv
cargo run -p thessa-worldgen-rocky -- bake --manifest data/worldgen/thessa_demo.toml --step-deg 2 --out /tmp/report.json
```

Demo recipes: `data/worldgen/example_rocky.toml` (minimal),
`data/worldgen/thessa_demo.toml` (tectonics + landmarks),
`data/worldgen/moon.toml` (real Moon dimensions).
