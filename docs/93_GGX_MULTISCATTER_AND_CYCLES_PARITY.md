# 93 — GGX multiscatter energy compensation and the B01 Cycles diagnostic

> Historical Gemini branch record (`3e9e30c`), imported 2026-11-02. Claims below
> describe that branch, not current production acceptance. See
> [doc 110](110_NATIVE_SCENE_AND_NOZZLE_INTEGRATION.md) for the integration audit.
> The roughness-only GGX fit was not retained. B03 target-photo projection is
> an art study and is excluded from independent/blind reference evidence.

Status: implemented and measured as a diagnostic. This is not a visual or
physical parity claim.

## Motivation

The first B01 Material Lab comparison ([doc 92](92_EXTERNAL_RENDERER_COMPARISON.md))
exposed two different problems:

1. **Smooth-peak clipping.** The raster path evaluates the BRDF once at the
   pixel centre, while Cycles integrates sub-pixel samples. A generic epsilon
   on the GGX denominator could clip the physical peak and lose energy.
2. **Missing multiple scattering.** Single-scatter Smith-GGX drops paths that
   bounce between microfacets. Rough conductors can therefore be darker than a
   path-traced reference.

## Implementation

The shared direct-sun response is `surface_sun_reflection` in
`crates/rcbt-core/src/shaders/ocean.wgsl`. It is included by the mesh, terrain,
environment and ray-scene shaders; the change is not in `mesh.wgsl`.

The smooth-peak path uses the cancellation-safe form

$$
D_{\mathrm{GGX}} = \frac{\alpha^2}{\pi\left(\lVert N\times H\rVert^2 + (N\cdot H)^2\alpha^2\right)^2},
$$

with the existing positive alpha floor. This avoids a generic epsilon on the
squared denominator.

The compact multiple-scattering approximation is the three.js DFG fit used in
Fdez-Agüera-style compensation, not a full Kulla-Conty or Cycles `MULTI_GGX`
implementation:

$$
E_{ss}(r) = \operatorname{clamp}(1.0 - 0.55r, 0, 1), \qquad
E_{ms} = \max(1-E_{ss}, 0),
$$

$$
F_{avg} = F_0 + \frac{1-F_0}{21}, \qquad
F_{ms} = \frac{F F_{avg}}{\max(1 - E_{ms}F_{avg}, 10^{-6})}, \qquad
F_{total} = F + F_{ms}E_{ms}.
$$

`F_total` replaces the single-scatter Fresnel factor. The implementation is
analytic and adds no lookup texture or runtime allocation. The `1/21`
hemisphere average and the split-sum fit are approximations; no spectral
energy-conservation claim follows.

## B01 diagnostic

The capture and comparison are 640×480, AMD Radeon 880M (`RADV STRIX1`, Mesa
26.2.5-arch1.1), against Blender 5.2.2 LTS (`d13f752e3b9c`, 64 samples CPU,
BOX filter). Metrics are over the whole pre-exposed linear RGB frame:

| Stage | Source | Linear RGB MAE | Linear RGB RMSE | Relative L2 against native | Top-10 px error |
| --- | --- | ---: | ---: | ---: | ---: |
| Initial diagnostic | `dee0eda`, doc 92 | 0.00271568 | 0.36535218 | 2.080346 | 99.9% |
| Smooth peak + multiscatter | `ccde98f`, clean capture `e3ab865` | 0.00213465 | 0.31302077 | 1.436160 | 99.9% |
| Diagnostic reduction | — | −21.4% | −14.3% | −30.9% | — |

`ccde98f` contains both the denominator change and the multiscatter term.
The clean B01 capture and Cycles run were rerun at `e3ab865`; the full
record is [`docs/artifacts/b01-cycles-comparison-2026-10-29.json`](artifacts/b01-cycles-comparison-2026-10-29.json).
`c332c5b` is a smooth-peak commit on another branch and is **not** an ancestor
of this branch; its intermediate row is therefore not claimed here.

The top-10 pixels still contribute 99.9% of the squared error. The remaining
difference is dominated by sub-pixel highlight sampling and geometry/AA, not by
a proven BRDF equivalence. Relative L2 uses the native frame as the
denominator; it is not a physical radiance error.

## Verification

- `cargo fmt --all -- --check`
- `cargo clippy --workspace --all-targets --all-features -- -D warnings`
- `cargo test --workspace --lib --bins`
- `python3 -m unittest discover -s tools/agent-qa/tests -v`
- `python3 tools/agent-qa/check_architecture.py`

No performance or visual-parity claim is made.
