# 87 — Native cloud cost and finite Fresnel endpoints

Status: measured continuation, 2026-10-29; baseline feature commit `a1b8e43`.
The retained production change corrects the Schlick cosine domain. The average
reflectance table, cloud loop-invariant hoisting and cloud zero-support branches
were investigated and rejected. No cloud quality reduction is shipped.

## Correctness change

A normalized floating-point dot product may exceed one by one ULP. WGSL `pow`
is undefined for a negative base, including exponent five. At roughness
`0.50862426`, view cosine `sqrt(0.125)` and sample 3 of 4, Radeon returned
`V.H = 1.0000001`: the old environment Fresnel weight was NaN. A subsequent
reflectance clamp concealed the NaN as missing reflected energy rather than
repairing its cause. The affected averaged response changed by 0.011800097.

The shared `surface_schlick_fresnel` helper clamps the physical cosine to [0,1]
before evaluating the existing fifth-power formula. The environment VNDF,
direct stellar BRDF, primary mesh and secondary ray-hit Fresnel now use it.
Cosines in the physical interval retain the previous arithmetic and material
law. There is no reflectance floor, energy multiplier, exposure or palette change.

The GPU regression evaluates 513 neighbouring roughness values around the
reproducer and five independent endpoint/interior cosines, with F0 channels
0, 0.04 and 1. It requires finite weights, the independently bounded central
physical limit and zero-F0 behaviour without a floor. Existing solid-angle,
grazing, reciprocal substrate and filtering oracles remain separate checks.

## Rejected optimizations

An experimental 2,049-knot average-GGX-response table used 32,784 additional
bytes in the existing environment storage buffer. After the Fresnel repair,
maximum absolute response errors on 65,537 roughness values were 2.96e-6,
1.60e-6 and 9.58e-7 for 4, 8 and 16 samples. An actual GPU microbenchmark
improved from 0.01174 to 0.00323 ms. However, a full native replay still took
446.08 ms per frame (last-30 median), against the previous 447.65 ms.
Live residency also differed. This did not justify the new representation;
the table, allocation, metadata use and CPU integration were removed.

A second experiment hoisted cloud density/phase invariants and skipped exact
zero-support work. Twenty-four same-data cases covering column/full march,
two coverage settings, two times and three altitudes matched the frozen shader
byte for byte on CPU Vulkan and Radeon. Nevertheless, the clouded microprobe
showed column transport slightly slower and full march only about 1% faster.
The full native replay measured 455.65 ms, including 69.61 ms in overlays.
An ablation retaining only the zero branches matched all 24 physical-GPU cases
(including an explicit zero-opacity control), but
was slower in both probes: column 0.02234 -> 0.02316 ms and full march
1.15648 -> 1.25258 ms. Production cloud code was restored exactly. Frozen
sources, controls, source/binary pins and raw timings survive in the evidence.
These results do not imply that zero-support shortcuts are universally slow.

## Cost attribution and next boundary

Ordinary headless Vulkan captures use 1280x720, 4x MSAA, High, HDR/PbrNeutral,
TAA off, RT off, the fixed coastal survey camera, time zero, three seconds of
prestream and 30 warmup frames. There are no per-stage serial profiling fences.
Separate diagnostic captures disabled existing effects to locate cost:

| Diagnostic | Opaque raster median | Overlay median |
| --- | ---: | ---: |
| No plants | 115.68 ms | 86.57 ms |
| No specular reflections | 172.75 ms | 60.48 ms |
| No clouds | 90.93 ms | 0.26 ms |

These short captures still include topology/material convergence; subtraction
is not an isolated causal speedup or whole-game FPS claim. Their settings are
counterfactual diagnostics, not accepted quality candidates. They identify
repeated cloud transport in surface lighting and reflected columns as a major
remaining cost. Reusing a bounded cloud-light representation merits a separate
calibrated prototype; it is not implemented by this change.

The research direction follows representation reuse in Hillaire's
[production atmosphere method](https://sebh.github.io/publications/egsr2020.pdf)
and [sky/cloud presentation](https://blog.selfshadow.com/publications/s2020-shading-course/hillaire/s2020_pbs_hillaire_slides.pdf).
This is a design inference; those papers do not validate this scene's cloud
quadrature, a local cache error bound or a specific performance prediction.

## Evidence and limits

The retained source passed 11 build/CPU/architecture gates (271 ordinary tests),
six isolated CPU-Vulkan regressions and seven physical-GPU compute/raster/RT
regressions. The final ordinary native capture completed without a new kernel
GPU fault; its last-30 medians were 449.90 ms frame, 382.69 ms opaque raster
and 61.13 ms overlays. This correctness fix is not claimed as a speedup.

Machine-readable commands, source pins, outcomes and raw evidence locations
are recorded in `artifacts/native-cloud-cost-fresnel-2026-10-29.json`. Physical
checks use the authorized Radeon 880M, RADV/Mesa 26.2.5, Vulkan, a render-node-only
headless process and kernel-event inspection. The historical desktop reset is
retained. No driver, watchdog or kernel settings are changed.

The coastal reference still has fuller fern crowns, stronger habitat composition
and more convincing substrate integration. A numerical Fresnel repair and
performance diagnostics do not establish visual parity or a blind preference
win. No main merge is included; existing user/Gemini work remains unstaged.
