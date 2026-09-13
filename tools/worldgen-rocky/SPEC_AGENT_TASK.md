# Agent Task — Extend `thessa-worldgen-rocky` for Thessa v0.2

Use `THESSA_DESIGN.md`, `thessa_v02.toml` and `worldgen_recipe.toml` as the design brief.

## Goal

Turn the current rocky-world tool from a mostly image/prompt scaffold into a deterministic
macro/meso world generator capable of producing a recognizable Thessa.

Keep scope practical. The runtime may still consume a simple global FHD-ish map for now.
Do not build virtual texturing, voxel planets, streaming terrain or a new renderer.

## Required work

1. Parse the extended world recipe TOML.
2. Add explicit deterministic large-feature generators:
   - impact basin / crater;
   - mountain arc / ridge chain;
   - plateau;
   - rift / canyon system;
   - volcanic province;
   - shield volcano / caldera;
   - archipelago;
   - glacier region / ice cap;
   - sedimentary / salt basin.
3. Use spherical/geographic coordinates or an equivalent seam-safe representation internally.
4. Generate macro height first.
5. Derive slope/curvature/drainage helpers from height.
6. Generate geology from feature provenance, not independent random noise.
7. Generate hydrology from terrain:
   - ocean by datum;
   - depressions can become lakes;
   - rivers must not intentionally flow uphill;
   - closed dry basins can produce salt flats.
8. Generate biome from climate-driver masks + terrain + geology.
9. Derive normal from final height.
10. Roughness/minerals/albedo may use biome+geology+feature provenance.
11. Keep optional GPT-generated source maps as hints/art direction, not unquestioned truth.
12. Add preview export for at least:
    - height;
    - biome;
    - geology;
    - hydrology;
    - geothermal activity;
    - eclipse exposure / continentality diagnostic masks.

## Height encoding fix

The old source-map convention was contradictory when it claimed both:
- `0 = -8000 m`
- `128 = 0 m`
- `255 = +12000 m`
- and "linear mapping".

Use an explicit datum-centered piecewise mapping if byte grayscale is still required:

```text
0..128   -> height_min_m .. 0
128..255 -> 0 .. height_max_m
```

After import, work in physical metres.

Add endpoint and round-trip tests.

## Planetary readability

A preview downscaled to 480×270 must retain:
- at least 5 clearly distinct major regions;
- the giant impact basin;
- major mountain arc;
- major canyon/rift system;
- large volcanic province;
- polar/glacial geometry.

Do not create uniformly noisy terrain.

## Determinism

Same config + same seed must reproduce exactly the same generated scalar/discrete fields
within the chosen deterministic representation.

Do not use thread scheduling or hash iteration order as part of generated output.

## Local detail

Do not put metre-scale boulders into the global map.

You may add deterministic scatter descriptors / local-terrain recipes based on:
- biome;
- geology;
- slope;
- curvature;
- elevation;
- feature provenance.

The goal is to let later runtime terrain reconstruct:
- rocks;
- talus;
- outcrops;
- lava blocks;
- dune ripples;
- small gullies;
- local craters.

## Geothermal zoning

Implement an explicit geothermal activity field.

The Thessa recipe should create:
- a few major active provinces;
- several secondary fields;
- many weak/local fields.

Activity should correlate with volcanic/rift geology and be usable later for:
- hot springs;
- fumaroles;
- sulfur fields;
- hydrothermal deposits;
- local snow suppression.

Do not apply the global mean tidal flux as uniform visible heating.

## Biomes

Support the richer biome taxonomy described in `THESSA_DESIGN.md`.
Keep `biome`, `geology` and `feature tags` separate.

Discrete IDs must never be bilinearly blurred.

## Tests

At minimum test:
- config validation;
- height encoding;
- deterministic generation;
- different seeds differ;
- no NaN/Inf;
- seam continuity for continuous fields;
- valid biome/geology IDs;
- river routing does not intentionally climb;
- derived normals follow final height;
- feature placement obeys physical size constraints;
- macro features remain recognizable at preview resolution.

Run:

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets --all-features -- -D warnings
cargo test --workspace
```

Do not weaken unrelated existing tests.

## Non-goals

- full climate GCM;
- plate-tectonic simulation over geological time;
- CFD;
- voxel planets;
- runtime renderer replacement;
- final resource balancing;
- exact biochemical ecosystem simulation.

Fake causes are acceptable when they produce coherent consequences.
Pure featureless fractal noise is not.
