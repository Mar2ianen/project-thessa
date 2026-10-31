# B03 canonical coastal scene evidence

This directory records the first **diagnostic** export of the
`B03-canonical-coast-v1` scene and its Blender Cycles comparison. It does not
establish renderer parity: the Cycles ground is an approximate independent
material reconstruction, not the native CBT terrain material.

## Committed files

- `scene.json` — camera, sun, ambient and scene statistics.
- `scene.LICENSE.txt` — scene licence notice.
- `cycles_metrics.json` — 32 spp procedural-ground diagnostic.
- `artifact_hashes.json` — hashes of the large generated inputs.
- `display.png` — native 1280×720 display output.

`scene.glb`, `linear.rgba16f`, `golden_radiance.exr` and
`golden_reference.jpg` are deliberately not committed. Their hashes and sizes
are recorded in `artifact_hashes.json`; regenerate the first two with the
native command below.

## Reproduce the native export

Run from the repository root and write the generated files outside the
checkout (for example under `$XDG_CACHE_HOME`):

```sh
cargo run -p thessa-terrain-native --release -- \
  --graphics data/graphics/surface-showcase.toml \
  --survey-lat 23.565679258039204 \
  --survey-lon 165.13881581863882 \
  --survey-distance 80 \
  --survey-eye-agl 2.5 \
  --width 1280 --height 720 --frames 60 \
  --headless /path/outside/repo/b03/display.png \
  --linear-hdr /path/outside/repo/b03/linear.rgba16f \
  --export-canonical-coast /path/outside/repo/b03
```

The export path receives `scene.glb`, `scene.json`, `scene.LICENSE.txt`,
`display.png` and (when HDR is available) `linear.rgba16f`. The committed
`scene.json` and `display.png` are copies of the observed run; the GLB is the
hash-pinned input for the Cycles adapter (not committed).

## Reproduce the Cycles diagnostic

```sh
blender --background --factory-startup --disable-autoexec \
  --python-exit-code 1 \
  --python tools/agent-qa/cycles_b03.py -- \
  data/reference-scenes/b03-canonical-coast \
  --output /path/outside/repo/b03-cycles \
  --samples 32 --ground procedural
```

`--ground reference-radiance` projects native terrain radiance as emission and
is a sanity check only, not independent parity evidence. The committed
`cycles_metrics.json` uses the default `procedural` ground and the
`tools/agent-qa/cycles_b03.py` at `067e98d` (SHA-256
`d5e9d9b7a3433e94398caee2f3f4af1f6cc3522dfa74ed0dc01d89d51d4e7afd`).

## Limits

- The native export was produced before this directory had a clean capture
  manifest. It is a dated diagnostic, not a source-clean benchmark.
- Terrain UVs are synthesized (`x*0.05`, `z*0.05`); the Cycles ground uses
  object-space procedural/PBR approximation rather than the native material.
- Plant transmission/translucency and the atmosphere are approximated.
- The 32 spp comparison is noisy; no pass/fail threshold is claimed.
