# thessa-worldgen-rocky (dev tool, MIT)

Generator base maps for **rocky planets only**. No clouds (separate system).

Pipeline: GPT Image 2.5 (Flare drafts, Sunburst final) paints 7 gore layers per
orange slice + caps, then the tool validates the manifest and adds
deterministic procedural micro-detail at runtime.

## Layers (see `prompts/` for color legends)

1. `height` — grayscale datum mapping
2. `albedo` — unlit base color
3. `biomes` — flat hex mask (normative)
4. `roughness` — 0 smooth … 255 rough
5. `normal` — tangent-space RGB
6. `hydrology` — R ocean / G lakes+rivers / B ice
7. `minerals` — R iron / G rare / B volatiles

## Resolution policy

Resolution-agnostic by design: gores are normalized fractions with overlap, and
detail uses physical wavelengths (meters), not pixels. Regenerate GPT maps at
any valid 2.5 size later (16px grid, ratio ≤ 3:1, edge ≤ 3840, 655k–8.29M px;
above 2560x1440 experimental) without changing the manifest schema.

## Usage

```bash
cargo run -p thessa-worldgen-rocky -- check --manifest data/worldgen/example_rocky.toml
cargo run -p thessa-worldgen-rocky -- list-prompts
cargo run -p thessa-worldgen-rocky -- sample --manifest data/worldgen/example_rocky.toml --lat-deg 12 --lon-deg -40 --base-height-m 800 --detail-scale-m 250
```
