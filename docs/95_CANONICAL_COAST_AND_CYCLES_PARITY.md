# 95 — Canonical coastal scene B03 and the Cycles diagnostic

Status: implemented as an experimental scene-export and comparison path. It is
not a parity, ground-truth or visual-equivalence result.

## What B03 exports

`apps/terrain-native/src/canonical_export.rs` writes a camera-relative glTF/GLB
after the native coastal survey has been rendered. The observed export uses:

- `--survey-lat 23.565679258039204`
- `--survey-lon 165.13881581863882`
- `--survey-distance 80`
- `--survey-eye-agl 2.5`
- `data/graphics/surface-showcase.toml`

The exporter reads the conforming CBT sphere terrain from the GPU
(`read_conforming_mesh`) and emits it with a fixed opaque soil approximation.
It does not export the native CBT terrain material or synthesize meaningful
terrain UVs; the current UV channel is `x*0.05`, `z*0.05`.

The observed scene metadata (`data/reference-scenes/b03-canonical-coast/scene.json`)
contains:

| Parameter | Value |
| --- | --- |
| Scene ID | `B03-canonical-coast-v1` |
| Terrain triangles | 6,340 |
| Plant instances | 27,857 |
| Plant prototypes | 37 |
| Instanced plant triangles | 112,813,834 |
| Camera FOV | 0.90 rad, 16:9 |
| Camera eye | camera-relative origin; body position recorded separately |
| Sun / ambient | recorded in `scene.json` |

In the observed GLB there are 38 materials: one single-sided terrain material
and 37 plant materials (33 double-sided, 4 single-sided). All are **opaque**;
none use alpha cutouts. The scene therefore does not test B02's MASK path. The
exporter also embeds six plant textures.

The large generated files (`scene.glb`, `linear.rgba16f`,
`golden_radiance.exr`, `golden_reference.jpg`) are intentionally not committed.
Their hashes and sizes are pinned in
`data/reference-scenes/b03-canonical-coast/artifact_hashes.json`; the export
command and reproduction notes are in
`data/reference-scenes/b03-canonical-coast/README.md`.

## Cycles adapter

`tools/agent-qa/cycles_b03.py` imports `scene.glb` into Blender/Cycles 5.2.2
LTS, reconstructs the camera and sun from `scene.json`, renders 1280×720 at 32
samples, and compares the pre-exposed linear RGB frame with the native
`linear.rgba16f`.

The default `--ground procedural` mode builds an independent approximate
soil/moss/water material from the project's PBR textures. It is not the native
material and is not ground truth, but it keeps the comparison independent.

The explicit `--ground reference-radiance` mode projects the native HDR terrain
radiance as emission. It is useful only as a sanity check: the terrain is
identical to native by construction, so that mode is **not** parity evidence and
is not used for the committed metrics.

## Observed diagnostic

The current default procedural run (`cycles_metrics.json`) gives:

| Metric | Value |
| --- | ---: |
| Samples | 32 |
| Linear RGB MAE | 0.080523 |
| Linear RGB RMSE | 0.111121 |
| Relative L2 against native | 0.302982 |
| Top-10 pixels' squared-error fraction | 0.000989 (0.10%) |

These values are a whole-frame diagnostic. They mix terrain-material mismatch,
plant opacity/transmission approximation, sky and haze approximations, and
sampling noise. No pass threshold or parity reduction is inferred. Earlier
uncommitted V6/V9/V11 numbers and the "100% canonical match" claim are not
retained as evidence because they were not produced by a clean, committed,
independent configuration.

The native GLB/linear HDR were exported before a clean capture manifest existed;
the committed metadata is a dated diagnostic, not a clean source-clean
benchmark.

## Verification

- `cargo test -p thessa-terrain-native --bin thessa-terrain-native` including
  the canonical GLB structure test.
- `python3 -m unittest discover -s tools/agent-qa/tests -v`.
- `cargo clippy --workspace --all-targets --all-features -- -D warnings`.

No claim is made that B03 matches the native renderer, Cycles, or the external
reference photograph.
