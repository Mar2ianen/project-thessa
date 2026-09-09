# thessa-worldgen-rocky (dev tool, MIT)

Sim-first generator for **rocky planets only**. No clouds (separate system).

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
