# 128 — Partial ground-normal coverage correction

Status: reproduced GPU regression and minimal raster composition fix,
2026-11-03. This fixes a conditional normal-amplitude error, not the complete
near-soil appearance or reference parity.

## Source mechanism and scope

[Mikkelsen's Surface Gradient–Based Bump Mapping Framework (JCGT 2020)](https://jcgt.org/published/0009/03/04/)
describes accumulating weighted surface gradients before resolving a normal.
The actual Unity Graphics implementation was inspected at
[`0efc28ea742b1966b9dd9cc4489e58b986a13faf`](https://github.com/Unity-Technologies/Graphics/tree/0efc28ea742b1966b9dd9cc4489e58b986a13faf):
`NormalSurfaceGradient.hlsl` resolves the accumulated gradient, and HDRP's
`LitDataIndividualLayer.hlsl` weights its detail contribution once. No Unity
code or assets were copied. Source hashes accompany doc 127's reference record.

Thessa already has scan-normal residuals and canonical base-height planes
(doc 124). Replacing that machinery would duplicate an existing owner. The
error is in `rcbt-core/src/shaders.rs` at the relief composition call site:

- First, the optical gradient perturbs the geometric normal to form a fallback.
- With partial canonical height-field coverage C, the reconstruction preserves
  `(1-C)` of that already perturbed fallback.
- Previously it also added the entire optical gradient again. On a flat base,
  the resulting optical slope is `(2-C)` times the requested slope.
- The correction adds only the C share of optical gradient, plus the already
  coverage-weighted canonical base gradient. The fallback supplies the rest.

Zero coverage returns the exact existing optical fallback. Full coverage uses
canonical base plus optical detail as before. Relief-disabled rendering and
geometry used for visibility are unchanged. This retains the existing tilted-
surface blending approximation; it is not a new exact general-purpose normal
blend. No new pass, binding, texture, source page, history or graphics setting
is added. Existing ground-detail/normal settings and fallbacks own the path.

## Independent raster regression

`partial_height_field_coverage_does_not_amplify_optical_slope` renders the
actual terrain pipeline at 65×65. Both fine and ancestor material pages contain
identical constant optical normals. Only the fine page contains a canonical
flat base plane. Camera positions exercise the fine page's residency feather.
The test decodes the actual ReliefWeight and Normal diagnostic outputs from
the sRGB target, rather than assuming fractional coverage was exercised.

| Actual canonical coverage | Requested normal x | Corrected raster x |
| ---: | ---: | ---: |
| 0.155926 | 0.370079 | 0.373371 |
| 0.496933 | 0.370079 | 0.373371 |
| 0.846873 | 0.370079 | 0.373371 |
| 1.000000 | 0.370079 | 0.373371 |

At the first position the old shader produces x=0.598205 and fails. The fixed
shader passes all four positions within 0.025, allowing 8-bit diagnostic
quantization. At least two positions must actually have partial coverage.
The existing `height_field_normals_are_independent_of_geometry_and_material_levels`
GPU test also passes, retaining no-base fallback, canonical base response and
optical/base composition at different material/geometry levels. Both run on
Radeon 880M/RADV with only the render node exposed and no display connection.

Commands:

```sh
cargo test -p thessa-render-wgpu --test material partial_height_field_coverage -- --ignored --nocapture --test-threads=1
cargo test -p thessa-render-wgpu --test material height_field_normals_are_independent -- --ignored --nocapture --test-threads=1
cargo test -p thessa-rcbt-core --lib
cargo clippy -p thessa-render-wgpu --test material -- -D warnings
cargo build --release -p thessa-terrain-native
python3 tools/agent-qa/check_architecture.py
```

The GPU executables were run inside the existing headless render-node-only
bubblewrap envelope. All 76 core unit tests, strict Clippy, release build and
architecture check passed. Raw before/after test logs are preserved under the
external `ground-detail-repair-2026-11-03/` output directory. The old/fixed test
is a controlled technical A/B with identical synthetic input; no blind
subjective preference is asserted.

## Soil-height audit limits

The canonical optical height is not only a 60 mm photo-scan displacement.
`lod::ground_relief_height_m` combines mean-centered scan height, moss cushion
height and the existing filtered 0.5–8 m procedural grain, then applies material
coverage. The doc 127 fixture's approximately 208 mm optical delta therefore
must not be called 208 mm of scan displacement or page quantization error.
Geometry and material pages also use different source footprints. Coverage-
weighted gradients deliberately omit derivatives of the coverage mask; the
existing documented blending approximation is not newly proven to cause the
foreground blur.

Actual selected pages, source bandwidth, shaded-normal stability in motion,
root agreement with the rendered CBT chord, substrate material quality and
full-scene performance remain open. The isolated regression establishes a
real defect even if a particular beauty frame predominantly has full coverage.

## Full-scene diagnostic control

The corrected release ran B03 at 1920×1080 with unchanged High settings, fixed
camera/time and 20-second readiness: `optical-gradient-v5-fhd-20s`. All 20
frozen diagnostic views preserve geometry, depth and beauty round-trip.
The mesh population and source payload remain unchanged from moss v4. These
serial stage-fenced diagnostics have `performance_equivalent=false`; their
durations do not establish production FPS or an acceleration.

Root reviewed the actual beauty frame: the floor remains visibly muddy and
repetitive. In ROI x=400..1499, y=800..1079, the geometry diagnostic selects
209,624 pure terrain interior pixels, excluding meshes/MSAA boundaries. All
have height coverage 1; none are partially covered. Therefore this fix does
not explain the foreground defect in that frozen frame. The controlled GPU
regression establishes the transition bug independently. Different process
page publications prevent claiming an exact whole-frame pixel A/B here.

Evidence and hashes: [partial-ground-normal-2026-11-03.json](artifacts/partial-ground-normal-2026-11-03.json).
