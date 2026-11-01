# 108 — Native Low consumes the shared optical field

Status: native implementation verified 2026-11-01. This closes the native
brightness defect F09, not optical calibration or general visual parity.

## Diagnosis and implementation

Doc 107 found that native Low's 32x256 unit-range sRGB texture became almost
black after daylight pre-exposure. Its authored alpha still darkened the scene.
The old CLI test counted any image difference and could mistake this for a
visible emitter; unrelated terrain residency also affected that counter.

Native Low now uses the existing volume consumer at six base march steps,
one pixel ray and up to nine centre points. Medium remains 8 base steps / up
to 4 by 4 pixels / 9 centres; High remains 14 / up to 8 by 8 / 17 centres.
The existing compression-aware refinement can raise any tier to 64 bins per
merged interval. These are budgets, not fixed total work or GPU timings.

All native tiers share the same cached 48-station mean, spatial compression,
source-derived photometric emission, absorption, pressure expansion and 3D
motion. There is no Low display-only brightness multiplier, texture bake,
cone-scale flicker or independent artistic diamond train. Source/profile
handles survive quality and exposure changes. The producer no longer allocates
the 32 KiB CPU texture; the backend's existing generic cone API/resources and
the legacy Bevy Low implementation remain compatible. No new graphics setting,
capability, shader, render pass, global mode, simulation path or force is added.
Off still clears the source/consumer and imposes no meaningful plume work.
The provisional native nozzle deck is unchanged; this is not live hardware
extraction or a propulsion-model revision. TAA stays default-off.

## Accuracy and exposure

A four-step candidate failed a normalized chord control at 5 kPa / throttle
0.05: the blue error was 0.07778 against a 0.25 peak reference, approximately
31%. It was rejected. Six steps pass the expanded 180 controls on both RADV
and llvmpipe: five ambient pressures, three throttles, four axial locations,
and 6/8/14 budgets, against converged independent 4096/8192-bin CPU transport.
Maximum absolute normalized RGB error across those budgets is 0.019260.
The Low regression limit is 0.025; Medium/High retain 0.02. This covers the
specified central transverse chords, not every view or animated field.

An actual-producer fixture renders all native tiers at 101325, 1000 and 0 Pa,
EV13 and EV5: 18 HDR frames over black. Source, profile, geometry, camera and
pose are equal within each pressure group. EV changes scale RGB by 256 while
opacity remains byte-identical. The largest deviation from exact scaled
half-float RGB is 0.00001520, bounded by `(256+1)*2^-24`: one binary16
subnormal quantum per conversion. The night-exposure HDR peaks remain above
one without clipping. These are exposure controls, not different physical
night/day lighting or calibrated test firings.

| Pressure (Pa) | Low EV13 peak | Low EV5 peak | Low vs High RGB L2 | Medium vs High RGB L2 |
| ---: | ---: | ---: | ---: | ---: |
| 101325 | 0.7974 | 204.125 | 4.63% | 1.48% |
| 1000 | 2.5312 | 648.000 | 18.43% | 0.49% |
| 0 | 0.9199 | 235.500 | 3.52% | 1.80% |

The actual silent native CLI at EV13 keeps authoritative flight state identical
for Off/Low/Medium/High. A fixed nozzle ROI now requires **positive blue emission**
(over 15 code values), rather than any signed difference. The frozen old Low
binary produces zero such pixels; new Low/Medium/High produce 305/308/308.
The old and new command use the same config/assets/camera and replay inputs.
Full opaque backgrounds can still differ through streaming; these CLI images
are functional evidence, not blind preference or a whole-frame A/B metric.

Low point samples retain aliasing that Medium/High finite-pixel filtering
addresses. Existing stock Cycles 5.2.2 BOX/2048 results are reused only after
all physical scene parameters match, excluding the explicit representation
fields `steps` and `pixel_grid`. No source identity or original report is
rewritten. New Low six-step captures include a same-point 16-step control:

| Frozen compiled control | Low point vs Cycles BOX RGB L2 | Six vs sixteen point steps |
| --- | ---: | ---: |
| compiled/sea-full | 4.33% | 3.68% |
| compiled/sea-half | 2.71% | 1.11% |
| compiled/matched-full | 1.48% | 1.15% |
| compiled/matched-half | 1.50% | 1.17% |
| compiled/thin-full | 58.09% | 0.35% |
| compiled/thin-half | 37.39% | 1.09% |
| compiled/matched-axial | 10.53% | 9.45% |
| compiled/matched-inside | 2.63% | 2.20% |
| compiled/matched-oblique | 1.60% | 1.51% |
| motion/sea-full | 4.64% | 4.07% |
| motion/thin-full | 55.88% | 0.76% |
| motion/matched-inside | 2.01% | 2.03% |

The large thin-jet differences mostly expose point-versus-pixel filtering;
the axial control also exposes coarse longitudinal integration. Low is not
accepted as High-equivalent. This table does not weaken the separate 180-chord
limits or turn them into a general error envelope.

## Cost and retained behavior

One 1280x720 plume pass on Radeon 880M/RADV, release fixture, 20 warmups and
80 measured timestamps per case: Medium/Low/Low/Medium is ABBA. Two High runs
provide context. All ten cases, p50/p95/p99, commands and binary hashes are in
the evidence JSON. No Blender or Cargo job ran during these measurements;
the later old-binary CLI run started after the cost report was finalized.

Low/Medium use the same nine-point path in curved controls; High refines it
to seventeen. Station bytes are identical across all tiers and repeated HDR
is identical within each tier. Low/Medium integration/pixel budgets differ
intentionally. The 2/64-station cases are stress controls, not the native
producer's normal 48-station setting.

| Control | Low p50, two runs (ms) | Medium p50, two runs (ms) |
| --- | ---: | ---: |
| straight, whole atmospheric | 0.247, 0.235 | 0.338, 0.334 |
| straight, whole vacuum | 0.323, 0.334 | 0.438, 0.433 |
| straight, near atmospheric | 0.522, 0.527 | 0.726, 0.760 |
| straight, near vacuum | 0.525, 0.536 | 0.736, 0.736 |
| curved, whole atmospheric | 0.570, 0.559 | 0.856, 0.798 |
| curved, whole vacuum | 0.517, 0.507 | 0.760, 0.768 |
| curved, near atmospheric | 2.198, 2.177 | 2.892, 2.991 |
| curved, near vacuum | 2.275, 2.379 | 3.074, 2.963 |

Low's pass is about 23–33% cheaper than Medium in these controls. This is not
an equal-quality comparison, an improvement over the old cheap cone, or a
whole-frame FPS claim. The representation is more faithful and still has
an explicit cost/accuracy tradeoff. No frequency/driver/watchdog changes were
made. Near High remains expensive and F08 is not closed.

A fresh 32-frame 640x360 High motion preview has exactly the same full scene
JSON and byte-identical HDR as doc 107. Medium's existing eight-step route is
unchanged. Three matched Low/High blind pairs over black have separate keys
and zero independent votes; the labelled diagnostic is not a blind judgment.

## Verification and reproduction

Passed: 45 core, 51 graphics, 40 renderer and four native producer unit tests;
nine GPU integration controls on each RADV and llvmpipe Vulkan, including the
180-chord expansion; four-tier actual CLI, 18-frame exposure fixture, 32-frame
High preservation; nine adapter Python tests; strict Clippy for four crates/all
targets, format/diff, and the 44-package/20-foundation architecture gate.
Only affected owners were checked; no full workspace test-suite pass is claimed.

The existing reference harness now accepts `capture --low --suite compiled`
or `--suite motion`, requiring point pixels and publishing actual six-step
metadata. Historical default/High controls retain their settings. The ignored
native `plume_quality_exposure_reference` fixture writes HDR/scene records to
a new `THESSA_BENCH_OUTPUT` directory. Exact isolated commands, logs, old/new
binaries, fields, HDR, cost CSVs, source snapshots and blind keys are under:

```text
/home/chechulin/Documents/Codex/2026-10-23/new-chat-3/outputs/plume-low-field-2026-11-01
```

[Evidence record](artifacts/native-low-plume-field-2026-11-01.json) and
[labelled comparison](artifacts/native-low-plume-field-2026-11-01.png).
No mass flow, thrust, clock, mount, RGB ramp, photometric convention or optical
coefficient is changed. Native surface lighting from the volume, accepted live
nozzle state/history, calibrated/nonlinear gas optics, general moving-camera
acceptance, legacy Low radiometry and photo parity remain open in doc 98.
