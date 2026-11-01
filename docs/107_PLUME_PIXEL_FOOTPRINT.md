# 107 — Finite pixel footprints for native plumes

Status: implemented in native Medium/High; verified 2026-11-01. This slice
concerns sensor sampling of the existing optical field, not a change to gas dynamics or a claim
of calibrated combustion or visual parity.

## Why the previous thin-jet comparison disagreed

The doc 106 thin-full control used a single ray through each native pixel
centre. Stock Cycles used its default Blackman–Harris filter of width 1.5.
Fifteen bright core pixels account for 98.84% of the squared RGB discrepancy.
A separate f64 integration of the immutable f32 optical publication reproduces
the native point samples. Integrating over each pixel instead converges toward
stock Cycles with an explicitly selected unit-width box filter. Merely changing
the reference filter does not fix the native point-sampling aliasing.

The diagnostic keeps the original Cycles scene, exposure and field intact.
The old Blackman–Harris results remain historical evidence. The new comparisons
state both filters explicitly. The [Blender film documentation](https://docs.blender.org/manual/en/4.0/render/cycles/render_settings/film.html)
describes reconstruction filters; the actual installed 5.2.2 settings were read
from the saved scene rather than inferred from documentation defaults.

## Representation and quality

The existing resolved plume quality controls the finite-pixel budget:

- Off has no plume work or resources; Low keeps its existing textured impostor.
- Medium permits a 4 by 4 box grid, with its existing 8 base march steps.
- High permits an 8 by 8 box grid, with its existing 14 base march steps.

At each pixel, the projected local Gaussian radius and incident compression
sheet thickness select a dyadic grid. The smooth Gaussian skirt is capped at
2 by 2; only unresolved compression sheets receive the finer quality budget.
Resolved features keep one ray. The minimum sheet thickness is the existing 0.08 coherent-core radius;
a pixel-sized margin includes nearby unresolved core coverage. Straight and
curved consumers use the same grid rule. Beyond the local four-radius Gaussian
filtering envelope plus the pixel span and maximum modal shell displacement,
the ray keeps point transport. This does not clip density, emission or ray
support. It prevents a narrow distant segment inside a broad conservative
capsule from triggering fine filtering across empty screen coverage. Curved
demand uses the closest finite centre segment, independently of world/camera orientation.

Each subray intersects the existing support and integrates its actual physical
chord, including the opaque-depth endpoint. Misses contribute zero; a missed
subray must not discard the whole pixel. The shader averages transported HDR
radiance and opacity. Averaging density or emission before exponentiation
would solve a different transport problem. Coverage expands by one pixel to
include footprints overlapping the support. The grid occupies the former
radial-reference padding: Draw remains 112 bytes and Frame remains 160 bytes.
`pixel_ray_limit` reports a maximum, not measured rays or GPU time.

The adaptation is a local feature estimate, not an error-certified adaptive
integrator. Features smaller than the capped grid can still alias. All subrays
share the single-sample scene depth of their pixel;
opaque silhouette coverage requires multisample depth or a separate coverage
solution. There is no temporal accumulation, camera jitter or change to the
TAA default-off policy. Dynamics, stations, compression, RGB, exposure, source
units and simulation are unchanged.

## Validation

The offline f64 fixture uses 8192 uniform chord bins and independent cylinder/cap intersection; it reuses only
the neutral optical sampler. Its nested pixel grids test spatial convergence
separately from ray-integration convergence. On the three brightest thin-full
pixels, 16 and 32 grids agree closely; 2 by 2 is insufficient.

The final 126-pixel thin-full ROI differs from an independent 8 by 8 f64 box
integral by 0.1308% relative RGB L2 and 0.00006267 maximum absolute opacity.
On the three brightest pixels, 16 and 32 grids converge further; the native
8-grid budget is approximate, not the dense oracle itself.

Stock Cycles 5.2.2 LTS uses 2048 samples, CPU/two threads, no denoising or
adaptive sampling, BOX with explicit width 1, the existing unbiased volume
integrator and identical scene/camera/field. Final HDR is compared to the
retained stock renders only after complete scene JSON equality checks; their
original captures/reports and candidate source identities remain intact.
The separate final recomparison records hashes and does not relabel the
original Cycles source as the final renderer. Relative L2 below uses Cycles
RGB in the denominator, unlike the historical helper's Thessa denominator.

| Control | Point vs BOX RGB L2 | Final vs BOX RGB L2 |
| --- | ---: | ---: |
| compiled/sea-full | 2.404% | 0.301% |
| compiled/sea-half | 2.532% | 0.514% |
| compiled/matched-full | 1.132% | 1.132% |
| compiled/matched-half | 1.147% | 1.147% |
| compiled/thin-full | 58.243% | 1.716% |
| compiled/thin-half | 37.445% | 0.542% |
| compiled/matched-axial | 2.095% | 2.095% |
| compiled/matched-inside | 0.544% | 0.544% |
| compiled/matched-oblique | 0.486% | 0.486% |
| motion/sea-full | 2.503% | 0.382% |
| motion/thin-full | 55.599% | 1.561% |
| motion/matched-inside | 0.797% | 0.797% |

The compiled matched-full/half/axial/inside/oblique and curved matched-inside
controls remain byte-identical to point sampling. The 32-frame 640 by 360
native motion preview holds all fields, poses, camera and exposure equal to
doc 106 after removing only the new filter metadata. Three blind before/after
pairs have separate answer keys and **zero independent votes**.

## Cost and rejected variants

Release RADV/880M timestamps measure one 1280 by 720 plume pass, 20 warmups
followed by 80 recorded samples per case, two repetitions per variant. CSV
frame numbers 0..79 already exclude warmups. Old-point/final/final/old-point
is ABBA; a new-binary point-budget pair separately checks the refactor's base
cost. Stations are byte-identical across variants; repeated HDR is identical
within each variant. The new pixel integral intentionally differs from point
HDR. No clocks, watchdogs, drivers or compositor configuration were changed.
No Blender or Cargo job ran during these measurements. Baseline variation
remains visible; this is not a whole-frame FPS or statistically powered claim.

| High control | Old point p50, two runs (ms) | Final p50, two runs (ms) |
| --- | ---: | ---: |
| straight, whole atmospheric | 0.519, 0.520 | 0.539, 0.534 |
| straight, whole vacuum | 0.642, 0.653 | 0.682, 0.699 |
| straight, near atmospheric | 1.076, 1.078 | 1.104, 1.185 |
| straight, near vacuum | 1.054, 1.074 | 1.122, 1.177 |
| curved, whole atmospheric | 1.744, 1.522 | 1.683, 1.643 |
| curved, whole vacuum | 1.365, 1.272 | 1.516, 1.517 |
| curved, near atmospheric | 7.382, 6.879 | 7.261, 7.215 |
| curved, near vacuum | 7.420, 6.650 | 7.081, 7.093 |

The far curved vacuum control costs about +0.20 ms / +15% over its same-run
point baseline. Near curved controls are within the measured baseline spread;
no optimization win is claimed. The initial fine grid across all unresolved
Gaussian skirts regressed that far control to 6.44–6.48 ms and was rejected.
A local-envelope-only diagnostic still cost about 3.83 ms under concurrent
CPU building and was not used as the canonical timing result. The retained
2-grid smooth-skirt cap reserves the expensive grid for compression sheets.
The benchmark's 8-step rows also use cap 8 as stress controls; they are not
measurements of production Medium's cap 4. Raw JSON includes all ten cases
and p50/p95/p99, not just the table above.

## Checks, reproduction and limits

Passed: 45 unchanged plume-core tests, 40 renderer unit tests, 9 Python adapter
tests, 5 native producer tests; nine GPU integration controls and two binding/
station controls on each of RADV and explicitly selected llvmpipe Vulkan.
The final native CLI checks all four tiers, identical flight state, silent
audio and 0/0/16/64 maximum spatial-ray counters. Medium/High positive blue
emission is separately checked in a fixed nozzle ROI. Low only darkens pixels
at daylight exposure: a pre-existing non-photometric fallback defect, now F09.
Whole CLI image differences can contain terrain-residency changes and are not
blind visual pairs. Strict Clippy for three crates/all targets, format, diff
and the 44-package/20-foundation architecture gate pass.

The first software-selection attempt accidentally selected physical OpenGL
and hit the existing depth-texture-load GLSL restriction. Its failure log is
retained; the final llvmpipe runs disable EGL discovery and identify Vulkan.
A first source-bound Cycles batch ended with SIGTERM before its report; it is
kept as partial data and excluded. The fresh completed batch is authoritative.
No broad workspace test-suite completion is claimed: an over-broad release
build was stopped, then verification targeted the affected owners.

Use new output directories outside the checkout. The adapter exposes
`capture --suite compiled|motion --pixel-grid 8 --profile release --allow-dirty`
and `cycles CAPTURE --samples 2048 --pixel-filter BOX`; default invocations
retain historical point controls and Blackman–Harris reference behavior.
The ignored `frozen_plume_pixel_integral` fixture accepts `THESSA_PLUME_INPUT`
and `THESSA_BENCH_OUTPUT` for independent dense point/box quadrature. Exact
isolated commands, logs, source snapshots, binary/scene hashes, raw HDR, cost
CSVs and private blind keys are under the machine-local evidence root:

```text
/home/chechulin/Documents/Codex/2026-10-23/new-chat-3/outputs/plume-footprint-2026-11-01
```

[The evidence record](artifacts/plume-pixel-footprint-2026-11-01.json) names
final `capped-*` artifacts and keeps rejected variants distinct.
[The labelled diagnostic](artifacts/plume-pixel-footprint-2026-11-01.png)
compares actual HDR-derived images, not a generated artistic reference.

No new optical calibration, real-firing image parity, independent blind votes,
whole-frame performance acceptance, surface illumination, volumetric NEE or
Mach-reflection solver is established here. Those remain in doc 98.
