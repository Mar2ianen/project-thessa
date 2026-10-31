# 102 — Compiled-engine volume gallery and reference camera origin

Status: nine actual native/Cycles transport comparisons now use a compiled
methalox engine's operating outputs. The reference near clipping mismatch is
fixed. F02 remains partial; live F03 nozzle extraction, physical optical
calibration, continuous-camera and blind visual acceptance remain open.

## Same published field, actual external renderer

`plume_reference.py capture --suite compiled` runs the ignored
`frozen_compiled_plume_reference` GPU fixture. A generated gas-generator bell
engine has 9.7 MPa chamber pressure, 0.08 m throat radius, expansion ratio 18
and 0.9 m nozzle length. Its complete authored specification and actual
`EngineOperatingPoint` are recorded in each scene. This is a procedural test
engine, not a measured real-world firing or the live player's engine.

The six side views combine effective throttle 1/0.5 with ambient pressure
101,325 Pa, matched exit pressure, or 1,000 Pa. The three other views use the
matched/full case: along the axis, inside the medium, and oblique. Sea-level
cases are overexpanded, matched cases have the weakest cells and thin-air
cases are underexpanded. Full and half actual flows are approximately 107.35
and 53.67 kg/s. Near-nozzle density is checked against those actual flows;
`engine_plume_source` receives throttle **one**, avoiding double scaling.
Its documentation now distinguishes already-allocated points from provisional
full-design decks. No runtime formula is changed.

Both engines consume the same frozen 48 f32 stations, radial Gaussian mixture,
shock multiplier, bounded cylinder, camera/FOV, exposure one and black world.
Native publishes 14- and 16-step HDR at 129×129. Erosion, tail dissolve, flicker,
scattering, opaque objects and temporal history are absent. Each case frames
its full field; the pressure/throttle rows are not fixed-distance image pairs.

Cycles uses stock Volume Coefficients and two packed, linear, Non-Color float
textures to represent radius/extinction/shock and emission along the axis.
They contain only the captured coefficients, with no color conversion or
fitted gain. Normalized image coordinates interpolate the uniform stations;
their ideal spacing differs from rounded native positions by less than 2e−7
of field length. Nonuniform profiles are rejected. Saved `.blend` files retain
the packed textures and node graph. This is a stock material description,
not a replacement Cycles integrator or runtime asset.

## Correct the reference ray start

The initial inside-medium comparison had RGB RMSE 0.0049093 at 2048 Cycles
samples, with little improvement over 256 samples. Independent f64 dense
uniform integration agreed much more closely with native. Repeating that
integration after omitting the first 0.1 m reproduced the Cycles result.

The cause was the adapter camera's 0.1 m near clip. Native's volume shader
starts at the optical camera (`t0=0`), independently of its opaque projection
near plane. Cycles' [perspective camera implementation](https://github.com/blender/blender/blob/main/intern/cycles/kernel/camera/camera.h)
offsets each ray origin by the near-plane distance. The adapter now uses
Cycles' minimum 1e−6 m clip start and records/checks the actual value. It does
not change the optical camera matrix, FOV, coefficients, exposure or native
renderer. Inside RMSE falls to 0.0004084. This is a corrected comparison
contract, **not** a renderer improvement or physical calibration.

## Actual measurements (2026-10-31)

Stock Blender/Cycles 5.2.2 LTS `d13f752e3b9c`, CPU/two threads, seed zero,
256/2048 samples, unbiased volume mode, step rate 0.1, maximum 4096 steps,
denoising/adaptive sampling off. Native executes on Radeon 880M/RADV and
software Vulkan/llvmpipe. RGB is compared; opaque Cycles film alpha is not
native transmittance. Shared Reinhard/sRGB display is applied after readback.

| Case | Native 14 versus Cycles 256, RGB RMSE | Versus Cycles 2048 |
| --- | ---: | ---: |
| Sea/full | 0.00070772 | 0.00031613 |
| Sea/half | 0.00054005 | 0.00021010 |
| Matched/full | 0.00061246 | 0.00028862 |
| Matched/half | 0.00040606 | 0.00017666 |
| Thin/full | 0.00236235 | 0.00170492 |
| Thin/half | 0.00116770 | 0.00067262 |
| Matched/axial | 0.00076480 | 0.00045641 |
| Matched/inside | 0.00107450 | 0.00040836 |
| Matched/oblique | 0.00111141 | 0.00038993 |

These whole-frame errors include stochastic transport, pixel reconstruction,
native quadrature and target quantization. Thin-field near-nozzle detail can
be subpixel. Sixteen native steps do not consistently reduce this combined
error, so no sampling-budget or shader change is shipped based on it.

An exploratory independent NumPy integration of the frozen coefficients uses
512/1024 **uniform** bins, without the GPU's warped bins. Successive RGB RMSE
is below 6.6e−8 for the inside, thin/full and axial controls. Native 14 versus
the denser result gives about 0.000172/0.000237/0.000537 respectively. This is
fixture-specific convergence evidence; no arbitrary animated-ray bound is
inferred. Retained scripts/results and hashes are in the evidence record.

Hardware/software differences for all nine frames fit
`0.002*abs(hardware) + 2e−6`; maximum absolute RGBA error is 0.00024414.
The four original analytic native HDR controls remain byte-identical to doc
100. Before the separate near-clip correction, the generalized Cycles adapter
also reproduced all four old raw Cycles controls byte-for-byte.

39 Python tests, 35 plume-core tests, strict render-wgpu/plume-core all-target
Clippy, format/diff and the 44-package/20-foundation architecture gate pass.
The two new dependencies are **test-only** existing workspace crates/libraries;
production dependencies and shaders, physics, quality settings and TAA defaults
are unchanged. Reference render durations are not engine performance results.

Evidence: [JSON record](artifacts/compiled-plume-gallery-2026-10-31.json).
Raw captures, commands and saved scenes remain in the external
`engine-boundaries-2026-10-29/compiled-plume*` directories. The gallery is
labelled technical evidence, not a blind preference pack. Continuous motion,
origin shifts, depth occlusion, more exhaust families, live accepted mount
publication, scattering/lighting and calibrated appearance remain open in
[doc 98](98_GRAPHICS_PARITY_ROADMAP.md).
