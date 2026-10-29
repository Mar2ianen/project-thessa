# 93 — Preserve smooth direct-GGX highlights

Status: shared shader correction implemented and verified on software Vulkan and Radeon 880M.

The first external [B01 Cycles diagnostic](92_EXTERNAL_RENDERER_COMPARISON.md)
identified narrow-highlight and rough-metal differences. Investigation found
a concrete numerical error in the shared direct specular kernel, independent
of Cycles' material model: applying the generic `1e-6` epsilon to the squared
GGX denominator truncated smooth normal-distribution peaks.

## Correction and ownership

For alpha `a` and half-vector cosine `c`, GGX has
`D = a² / (pi * (1 - c² + a²*c²)²)`. At normal incidence its peak is
`1/(pi*a²)`. Under the existing minimum alpha 0.0025, that peak is about
50,929.58. The old denominator floor instead produced 6.25. A unit-illuminance,
normal-incidence dielectric with F0=0.04 therefore returned **0.0625 instead
of 509.2958** from the direct reflection kernel.

The repaired implementation uses the equivalent positive sum
`|N x H|² + a²*(N.H)²`, avoiding cancellation near alignment. The existing
positive alpha bound makes a denominator epsilon unnecessary. `N.H` is clamped
to its physical [0,1] interval, and `D` is evaluated as `(a/denominator)²/pi`.
The mathematical basis, including the cross-product identity, is described in
[Filament's NDF discussion](https://google.github.io/filament/Filament.html).
This implementation is independent project code using that identity.

`rcbt-core/src/shaders/ocean.wgsl::surface_sun_reflection` remains the sole owner.
Existing native mesh, terrain, ocean, secondary ray-hit and Bevy shader consumers
reuse it. This is an accuracy correction to the existing specular response,
not a new effect or configuration mode. The configured material, quality,
reflection-disable, roughness-bound and raster-footprint policies still apply.
No extra pass, texture, lookup table, persistent allocation or dependency is
introduced. Corrected direct arithmetic does have a cost; no speedup is claimed.

## Independent regression

`environment::tests::smooth_direct_ggx_preserves_the_analytic_peak_and_tail`
evaluates eight roughnesses (0, .02, .05, .1, .15, .2, .5, 1), seven half-vector
angles (0 through .5 radians) and F0 channels .04, .5 and 1. The f64 oracle uses
`sin²(theta) + a²*cos²(theta)` with the existing Schlick-Smith visibility;
there is no denominator floor or reference image fitted into the calculation.

The test failed on the old shader with the scalar result above. After repair,
all 56 cases / 168 RGB comparisons pass with maximum relative error
`3.2042433e-7` on both llvmpipe and Radeon 880M/RADV Vulkan. The regression's
acceptance envelope is `2e-4`, allowing ordinary f32 backend rounding.

Existing finite Fresnel, reciprocal substrate-energy and footprint-filtering
checks pass. Actual primary PBR texture/disable behavior and retained ocean
reflection also pass on Radeon. The ordinary libraries pass 75 rcbt-core tests
and 40 render-wgpu tests (21 diagnostics remain ignored by default). Strict
all-target Clippy for both crates, formatting and 28 Python quality tests pass.

## Actual B01 result

A new native 640×480 capture used 30 warmup and 60 measured serial frames on
Radeon 880M, RADV/Mesa 26.2.5-arch1.1, with the existing direct-light profile.
Geometry and scene-definition hashes, actual camera/projection/exposure/light
metadata and requested settings match the previous capture. Candidate/submitted
work remains 22,082 triangles and 11 logical/API draws. All HDR components are
finite; the maximum pre-exposed RGB rises from 85.0625 to 124.

Against the **same saved Cycles float32 HDR**, with the same light-unit/BSDF/AA
limits recorded in doc 92:

| Whole-frame pre-exposed RGB diagnostic | Before | After |
| --- | ---: | ---: |
| MAE | 0.00271568 | 0.00264613 |
| RMSE | 0.36535218 | 0.31984315 |
| Relative L2, fixed Cycles denominator | 0.75492887 | 0.66089335 |

RMSE falls about 12.46%. This is a scoped numerical image result, not general
renderer parity or an aesthetic verdict. The narrowest highlights still differ
because pixel reconstruction, roughness filtering and material models differ.
The roughest metallic sphere remains visibly different. A local Blender import
probe confirms the reference Principled materials use `MULTI_GGX`; Thessa's
single-scattering model needs a separate energy/white-furnace investigation.

Raw serial dev medians are GPU 0.25625 → 0.27701 ms, mesh raster
0.20112 → 0.21272 ms, CPU through submit 0.11527 → 0.06763 ms. These captures
were taken at different times, without an interleaved controlled cost experiment.
They record the small lab's observed cost and do not establish an isolated
arithmetic regression, a speedup or a coastal frame-time budget.

Source/binary/shader hashes, actual captures, diagnostic metrics and test
outcomes are pinned in [the evidence record](artifacts/stable-direct-ggx-2026-10-29.json).

## Milestone record

- **Source commit:** `b0c8b5a` (integrated development tree `dee0eda`).
- **Resulting commit:** the commit introducing this document.
- **Architectural change:** correction in the existing shared WGSL owner;
  consumers inherit it without a new API, runtime dependency or settings path.
- **Changed files:** shared shader, existing GPU regression module, dated
  evidence and synchronized reference/status documents.
- **Tests executed:** failing old-shader reproducer; ordinary 75/40 library
  tests; five explicit Vulkan library diagnostics including the new oracle on
  CPU and Radeon; two existing physical primary/ocean integration tests; actual
  native Radeon B01 capture; strict Clippy/format and 28 Python tests.
- **Tests not executed:** full coastal replay, moving-camera/temporal or blind
  review, Metal/WebGPU hardware, Unreal/Godot captures, full-workspace tests.
- **Numerical evidence:** analytic error and fixed-reference HDR metrics above.
- **Visual evidence:** actual same-data B01 PNG/HDR capture, no subjective vote.
- **GPU/CPU measurements:** scoped dev medians above; no performance win claim.
- **Imported assets/licenses:** none; generated B01 geometry remains MIT.
- **Known limitations:** native evidence explicitly records a dirty development
  source fingerprint; the saved renderer/test patch exactly reconstructs its
  recorded diff hash. This repairs the direct NDF, not multiscattering, pixel
  integration, vegetation cutouts or the full coastal composition.
- **Next smallest useful change:** isolate and validate rough-metal energy
  transport, then complete explicit B02 cutout support and fixed density tiers.
