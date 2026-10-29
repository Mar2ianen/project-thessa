# 99 — Plume pre-exposure and measured lookup audit

Status: shared native pre-exposure fixed; five lookup alternatives measured and
rejected. No lookup performance win or aesthetic parity is claimed.

## Correct scene composition

Native terrain/meshes write pre-exposed scene radiance, but the Low plume and
volume plume ignored the camera's exposure. A frozen field therefore kept its
brightness while the surrounding scene changed with exposure. This also broke
the linear-HDR composition contract before tone mapping.

The existing 160-byte plume frame uniform now carries the shared view exposure
in previously unused padding. Both representations multiply outgoing RGB by
that gain once, after optical/texture evaluation and before alpha-over. Alpha
and physical transmittance are unchanged. Camera-only exposure updates do not
rebuild the cached profile or texture. There is no new setting, allocation,
sampling budget, simulation input or optical calibration coefficient.

The actual RGBA16F GPU oracle exercises Low and Volume at gains 1, 0.25, 0 and
4. It checks finite nonnegative HDR, proportional RGB within half-float
quantization, exact unchanged alpha, signal above one, and zero cached-field
upload on exposure-only updates. At gain 1, all ten frozen field images remain
byte-identical to the original baseline. Radiance templates themselves remain
provisional: this correction is not validation of fuel chemistry or absolute
photometry.

## Lookup experiment: retain the existing algorithm

The original marcher linearly searches its station publication per sample.
Five alternatives were implemented in isolated source snapshots: lower-bound
binary search, exponential bracketing, uniform-spacing prediction with exact
coordinate correction, a short linear prefix with binary tail, and a carried
ray interval with correction in both directions. None is shipped: each sped up
the distant/whole-field fixtures but regressed at least one near-nozzle view.
The shader retains the original linear lookup and interpolation arithmetic.

Each retained release binary ran ABBA on Radeon 880M/RADV, without an
intentional concurrent build/GPU job. Ten 1280×720 RGBA16F cases cover sea-level
and vacuum 48-station fields, 8/14 steps, whole-field and near-nozzle views,
plus 2/64-station publication limits. Every case warms 20 frames and records
80 GPU/CPU/wall samples. GPU intervals cover the plume pass, excluding clear,
readback and timestamp mapping. Static-field CPU encoding is not profile-build
or complete-frame cost. Driver/clocks were not modified; ABBA does not eliminate
unmanaged power/desktop noise.

| Rejected alternative | Best GPU median change | Worst GPU median change |
| --- | ---: | ---: |
| Binary search | −66.9% | +62.5% |
| Exponential bracket | −64.7% | +8.1% |
| Corrected spacing prediction | −78.2% | +16.3% |
| Linear prefix / binary tail | −61.5% | +15.6% |
| Carried ray interval | −73.9% | +15.1% |

Changes compare the mean of the two candidate medians with the mean of the two
baseline medians per case; negative is cheaper. Consult exact values in the
evidence, rather than treating rounded extrema as a general speed estimate.
All 200 release HDR captures agree exactly by case. No geometry, physical
source, station publication or sample budget was reduced to obtain a win.

## Contracts and verification

The publication boundary now explicitly rejects descending axial coordinates;
finite nondecreasing coordinates, including repeats, remain supported. A GPU
compute oracle calls the **production** WGSL sampling function for 1,077
queries across 2/3/48/64 stations with uniform, quadratic and repeated spacing.
It checks eight returned components against an independent CPU linear walk at
exact stations, immediately adjacent floats and out-of-range positions. Around
zero it uses ±1e−6 m; WGSL may flush the adjacent subnormal ±1e−45 m.

- Source base `939609a` / integrated `8d197af`.
- Radeon/RADV and pinned llvmpipe **Vulkan** pass the station oracle and all five
  ordinary plume GPU oracles. Existing 18 physical-profile chord checks retain
  maximum absolute RGB error 0.003799 on Radeon, under their 0.02 envelope.
- 40 ordinary render-wgpu library tests pass, with 22 hardware diagnostics
  ignored separately. Strict all-target Clippy and format/diff checks pass.
- Initial default software selection used GL; its existing GLSL translation
  rejects depth `textureLoad`. The software verification process disables EGL
  vendor discovery so the pinned Vulkan ICD is selected. Both failed/successful
  commands are retained; no production selector or driver configuration changes.
- No Metal/WebGPU hardware, full planetary trajectory, external volume render,
  blind preference, or full-scene frame-time acceptance was performed.

[Evidence](artifacts/plume-exposure-lookup-2026-10-30.json) records all rejected
ABBA samples/statistics, source patches, immutable binary and HDR identities,
shipping source hashes and limits. Binaries/CSV/raw frames stay outside Git.
The opt-in fixture is reproducible with:

```sh
THESSA_BENCH_OUTPUT=/absolute/new-output cargo test --release \
  -p thessa-render-wgpu --test plume published_profile_lookup_cost \
  -- --ignored --nocapture --test-threads 1
```

The current [roadmap](98_GRAPHICS_PARITY_ROADMAP.md) keeps compiled-engine
extraction, transport convergence/reference volumes and CBT whole-system
acceptance ahead of further speculative lookup rewrites.
