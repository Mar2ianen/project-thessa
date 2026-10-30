# 94 — Native cutout pipeline, reference scene B02, and the Cycles B02 diagnostic

Status: implemented and captured on the AMD Radeon 880M. The Cycles result is a
same-scene diagnostic, not a parity or visual-equivalence claim.

## Native glTF MASK cutouts

### Core data model

`crates/render-core/src/mesh.rs` adds `AlphaMode::{Opaque, Mask, Blend}` and
`Material::alpha_cutoff`. `MeshAsset::validate` requires a finite, non-negative
cutoff. The cutoff is ignored for `Opaque` and `Blend`; opaque materials do not
have an enforced 0.5 value.

`crates/render-assets/src/lib.rs` adds `decode_glb_cutouts`, which accepts
`Opaque` and `Mask` and rejects `Blend` (the consumer has no transparent
sorting). The ordinary `decode_glb` and rest-pose paths remain opaque-only.

### Raster coverage

`crates/render-wgpu/src/mesh.rs` compiles two primary shader variants. The
masked variant evaluates texture and normal derivatives before coverage, then
uses the same expression as the shader:

```wgsl
if (base_sample.a * material.color.a < material.optical.z) { discard; }
```

`material.optical.z` carries the cutoff. Opaque primitives keep the no-discard
pipeline, preserving early-Z. Masked and opaque primitives are not re-sorted
into batches: the draw loop selects the appropriate pipeline per primitive.

Cutout coverage is not silently approximated in unsupported consumers. The
mesh renderer rejects a simultaneous shadow atlas, and `ray_scene` reports
`MASK/BLEND ray coverage is unsupported` and falls back to raster cutouts.
`Blend` remains unsupported by the native mesh consumer.

`crates/render-wgpu/tests/mesh_cutouts.rs` is a GPU regression covering holes,
depth, cutoff values `0`, `0.5`, `1.5`, opaque alpha semantics, shadow
rejection, and RT rejection.

## Reference scene B02

`crates/reference-scenes/src/lib.rs` defines `B02-vegetation-v1` using the pinned
Poly Haven Fern 02 core-cutout asset. The description is derived from the
decoded asset (triangles, object count, used-material alpha mode/cutoff and
sidedness), so a custom `--reference-asset` cannot silently keep stale counts.

| Parameter | Value |
| --- | --- |
| Asset ID | `vegetation-fern-02-core-cutout-v1` |
| Asset SHA-256 | `10025cc15e9067b430e5be4b2628a06ac0c7f8acc49d5b2f61dddeda08a48b26` |
| Mesh objects / triangles | 4 / 6,232 |
| Used material | `MASK`, cutoff `0.5`, double-sided |
| Camera eye / target | `[0.46, 1.6, 2.8]` / `[0.46, 0.18, 0.44]` |
| Vertical FOV / near / far | 50° / 0.05 m / 50 m |
| Direction to light | normalized `[-0.5, 1.0, 0.75]` |
| Direct / ambient illuminance | `[10000, 10000, 10000]` lux / `[0, 0, 0]` |

The native CLI accepts `--reference-scene b02` and an optional
`--reference-asset <PATH>`. Without an override it resolves the verified cache
entry under `$XDG_CACHE_HOME/thessa/reference-assets/...`.

## Cycles diagnostic

`tools/agent-qa/cycles_b02.py` imports the same `scene.glb`, verifies the
capture hashes and scene identity, uses Cycles 5.2.2 LTS CPU with seed 0, 64
samples, BOX filtering, 16 transparent bounces, direct-light bounces and no
denoising, applies the native exposure and Reinhard mapping, and compares
pre-exposed linear RGB.

The observed dirty-source diagnostic on 640×480 is:

```json
{
  "rgb_components": 921600,
  "foreground_union_pixels": 16480,
  "pre_exposed_linear_rgb_mae": 0.0006014475,
  "pre_exposed_linear_rgb_rmse": 0.0036630174,
  "relative_l2_against_thessa": 0.40966978,
  "top_10_pixels_fraction_of_squared_error": 0.01268881
}
```

A small whole-frame MAE does not establish parity: the relative L2 value is
0.41, and the remaining error is spread across the frame rather than
concentrated in a few pixels (the top ten contribute 1.27%). The result mixes
cutout silhouette/filtering, MSAA-versus-path-sampling and BRDF differences.
No pass threshold is claimed.

The comparison record is
[`docs/artifacts/b02-cycles-comparison-2026-10-29.json`](artifacts/b02-cycles-comparison-2026-10-29.json).
The committed JSON records the capture source as dirty; a clean recapture must
replace that provenance before the artifact is treated as a clean benchmark.

## Verification

- `cargo test -p thessa-render-wgpu --test mesh_cutouts` (GPU, Radeon 880M).
- `cargo test --workspace --lib --bins`.
- `cargo clippy --workspace --all-targets --all-features -- -D warnings`.
- `python3 -m unittest discover -s tools/agent-qa/tests -v` (34 tests).
- `python3 tools/agent-qa/check_architecture.py` (44 packages, 20 guarded foundations).

The native B02 capture and Cycles run are diagnostic evidence only.
