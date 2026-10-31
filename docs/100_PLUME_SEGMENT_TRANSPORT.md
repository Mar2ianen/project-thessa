# 100 — Finite-cell plume transport and actual Cycles volume controls

Status: finite-cell emission integration corrected in the CPU reference,
native WGSL and retained Bevy WESL consumer. Four source-bound volume controls
were rendered in native and stock Cycles. Engine calibration, live nozzle
extraction, general-volume convergence and aesthetic acceptance remain open.

## Correct the emission integral

The old marcher added `incoming_T * j * ds * exp(-sigma*ds)`: it attenuated
all emission from a cell as if it originated at the far boundary. This
underestimated a glowing absorbing medium and made brightness depend strongly
on the representation's step count. The CPU oracle had the same defect, so
CPU/GPU agreement alone did not expose it.

For constant coefficients inside a cell, the
[equation of transfer](https://pbr-book.org/4ed/Light_Transport_II_Volume_Rendering/The_Equation_of_Transfer)
gives:

```text
T_cell = exp(-sigma * ds)
L_cell = j * (1 - T_cell) / sigma       when sigma > 0
L_cell = j * ds                        when sigma = 0
L += incoming_T * L_cell
T *= T_cell
```

The CPU uses `expm1` to retain thin-cell precision. WGSL/WESL use the
dimensionless series `1 - tau/2 + tau²/6` below optical depth 0.01, otherwise
the exponential quotient. The omitted relative series term is at most about
4.2e−8 before f32 rounding. This is a numerical accuracy branch, not a density
cutoff or an optical-material coefficient. It retains emissive zero-extinction
media without division by zero.

Source profiles, geometry, sampling positions/counts, transmittance, exposure
and quality settings are unchanged. Low's retained texture bake is unchanged.
No authoritative propulsion, mass, force or thermal state is modified; no new
production allocation, setting or dependency is introduced.

## Independent checks

The new CPU regression uses the conservation identity `L = S*(1-T)` whenever
`j(r) = S*sigma(r)`. This holds independently of radial quadrature. The old code
fails it: one test returned 0.8000297 instead of 0.9132762 at one step. The
corrected implementation satisfies the 2e−12 absolute test envelope across
four extinction scales and 1/2/8/14/64/256 steps.

An actual HDR GPU test looks down an undeformed constant axial field. Its
central ray is a homogeneous emitting/absorbing chord with a known closed-form
solution. Sixty-four cases cover optical depths 0, 1e−8, 1e−4, 0.01, 0.1, 1, 10
and 100; 1/8/14/16 steps; and full/partially depth-clipped chords. The old shader
fails this independent test. The corrected shader passes on Radeon and software
Vulkan within `0.002*abs(expected) + 2e−6`, including half-float storage and depth
quantization. Existing slab, 18 physical-profile chords, camera-inside/axial
rays, pre-exposure and disabled/cache lifecycle oracles also pass. These are
stated fixture envelopes, not a universal animated-plume error bound.

## Same field in an external engine

`tools/agent-qa/plume_reference.py capture` builds and runs the ignored native
`frozen_plume_volume_reference` GPU fixture. It verifies source/binary identities
before and after execution and retains raw RGBA16F, field/camera JSON and common
display PNGs. The controls are 129×129, 14 steps, gain 1, identity camera,
0.9 rad vertical FOV, no lights, erosion, tail dissolve or temporal history.
They are synthetic transport controls, **not** calibrated rocket-engine data.

Each field has two identical-coefficient axial endpoints, length 4 m,
radius 1 m, radial integration bound 1.6 m, unit shock and emission
`[0.3,0.15,0.07]` per metre. Three look down the axis from 5 m before the nozzle,
with central optical depths 0/1/10. The fourth looks from the side at 10 m,
with extinction 0.15/m. The radial weight is the existing two-Gaussian mixture.

For these constant axial fields, a separate f64 mathematical oracle integrates
the Gaussian mixture along finite-cylinder chords using `erf`. It does not
call either marcher. It evaluates pixel centres without a reconstruction
filter. This separates native integration/precision error from reference-engine
sampling noise.

The Cycles adapter declares the **same coefficients and bounded field** using
stock [Volume Coefficients](https://docs.blender.org/manual/en/latest/render/shader_nodes/shader/volume_coefficients.html)
and ordinary math nodes, inside an unscaled 12-triangle carrier box. Density
support masks enforce the same cylinder/caps. Scatter coefficients are zero.
It uses the captured camera, verifies actual FOV and unscaled carrier state,
and records the actual camera matrix;
no custom Cycles integrator, fitted exposure or recoloring is used. Saved
`.blend` files retain the complete node graph. The carrier is an integration
boundary, not a replacement surface asset or a glTF material conversion.

Actual Blender/Cycles **5.2.2 LTS**, build `d13f752e3b9c`, CPU, seed 0,
256 and 2048 samples, adaptive sampling/denoising off, unbiased volume mode,
volume step rate 0.1 and maximum 4096 steps completed. Raw EXR/float32 radiance
is retained; shared Reinhard/sRGB display follows readback. RGB is compared;
Cycles' opaque-film alpha is not native transmittance.

## Before/after at the same 14 steps

The old native shader from source base `3519ea2` was compiled with the same
current fixture and captured separately, then the corrected shader was restored.
Field/camera JSON is byte-identical before and after. All four packed native
alpha images are byte-identical. Immutable old/new binaries and source snapshots
remain outside Git.

| Control | Old RGB RMSE versus exact pixel centres | Corrected RMSE | Error reduction |
| --- | ---: | ---: | ---: |
| Axial, zero extinction | 0.0000786097 | 0.0000786097 | unchanged |
| Axial, central optical depth 1 | 0.0023880660 | 0.0000618890 | 38.6× |
| Axial, central optical depth 10 | 0.0032204929 | 0.0000201026 | 160.2× |
| Side view | 0.0006114133 | 0.0000697079 | 8.8× |

These are whole-frame linear RGB errors, including background and target
quantization. They are not aesthetic scores or whole-scene quality multipliers.
Maximum corrected alpha error against the exact controls is below 0.00091;
the correction itself leaves the stored alpha unchanged.

| Control | Corrected native versus Cycles, 256-sample RMSE | 2048-sample RMSE |
| --- | ---: | ---: |
| Axial, zero extinction | 0.00366921 | 0.00123359 |
| Axial, depth 1 | 0.00127750 | 0.00045613 |
| Axial, depth 10 | 0.00020706 | 0.00007614 |
| Side view | 0.00230168 | 0.00083078 |

The approximately threefold decrease after eightfold sampling is consistent
with Monte Carlo variance. This does not prove that every remaining difference
is noise: native point sampling/half-float precision and Cycles pixel
reconstruction differ. No brightness/contrast parameter was adjusted between
these runs.

## Reproduction and cost

Outputs must be new and outside the checkout. Use `--allow-dirty` only to record
an explicit development fingerprint. Run hardware jobs without desktop access;
the executed commands used isolated `bwrap`, private configuration and only a
render node. The normal software check disables EGL vendor discovery inside
that process to select the pinned Vulkan ICD; no driver files are modified.

```sh
python3 tools/agent-qa/plume_reference.py capture \
  --output /absolute/new-native-volume --profile dev
blender --background --factory-startup --disable-autoexec --python-exit-code 1 \
  --threads 2 --python tools/agent-qa/plume_reference.py -- cycles \
  /absolute/new-native-volume --output /absolute/new-cycles-volume --samples 2048
```

The existing `published_profile_lookup_cost` fixture was reused for release
ABBA on ten 1280×720 cases, keeping station bytes, field geometry and 8/14 steps
identical. GPU intervals cover only the plume pass. Main whole-field medians
are within about 1% in this run; a near-nozzle vacuum case costs about 6.6% more.
Some close-up samples have substantial run-to-run variation. Full CSV/p50/p95/p99
and binary hashes are retained; **no speedup or whole-frame budget is claimed**.
Cycles render duration is not an engine performance comparison.

## Milestone record (2026-10-31)

[Evidence](artifacts/plume-segment-transport-2026-10-31.json) binds before/after
captures, actual Cycles settings/camera, analytic metrics, source/binary hashes,
release ABBA and logs. Labelled technical comparison frames remain outside Git;
they are not blind preference packs.

- Source base `3519ea2` / integrated `d69dc3f`; no dependency change.
- 35 plume-core tests, 38 Python tests, native station and six plume GPU oracles
  on Radeon/RADV and llvmpipe Vulkan, ordinary render-wgpu unit checks, six
  retained client plume tests, strict relevant all-target Clippy, format/diff and
  44-package/20-foundation architecture gate pass.
- Bevy Rust/tests and its shader-prologue contract were checked; its material was
  not rendered on GPU. Metal/WebGPU, full canonical-scene replay, arbitrary
  heterogeneous/animated field bounds, scattering/GI, engine calibration and
  independent preference acceptance were not performed.
- New controls/node graphs are generated project data; no Epic, Blender or PBRT
  implementation code was copied into runtime.

Continue F02 with compiled-engine and continuous-camera galleries, and F03 with
accepted per-nozzle runtime observations rather than inferring them from a
requested throttle. The complete backlog remains in [doc 98](98_GRAPHICS_PARITY_ROADMAP.md).
