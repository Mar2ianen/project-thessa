# 96 — Preserve transparent order while batching instance runs

Status: implemented and measured on Radeon 880M; exact HDR regressions passed.

The initial BLEND consumer submitted one API draw per visible primitive
instance. Even when the sorted order was a consecutive instance range of the
same primitive, it rebound the same resources and emitted individual draws.
This created avoidable command overhead for transparent instanced populations.

The consumer now scans the final sorted sequence and joins only adjacent
ascending instance IDs of the same renderer and primitive. It submits that
range through ordinary wgpu instanced indexed drawing. Material, sidedness,
instance data, geometry and primitive order are unchanged. Lower-numbered
instances precede higher-numbered instances in the
[Vulkan primitive-order contract](https://docs.vulkan.org/spec/latest/chapters/drawing.html#drawing-primitive-order).
There is no shader, indirect buffer, extra sort or per-frame allocation for this
scan. Nonconsecutive/descending IDs and primitive/consumer boundaries split runs.

The original per-instance queue remains intact. Cross-consumer composition
sorts that original queue before coalescing, so another renderer at an
intermediate depth can split a locally contiguous run. `draw_stats` reports
standalone consumer command/API counts; `draw_scene_into_pass` now returns the
actual merged transparent API count. It excludes opaque draws. The existing
center-sort limitations for intersecting surfaces remain unchanged.

## Fixed-work measurement

The release fixture uses one two-triangle primitive and **8,192 visible
instances / 16,384 triangles**, 1280×720 RGBA16Float, High, alpha=0.5,
emission `[0.1,0.5,0.9]`, exposure=0.5 and no incident illumination. There is no
RT, shadow atlas or temporal history. Each run warms 30 frames then measures
120 serial frames, using the existing GPU interval/readback mechanisms.
CPU time includes update through queue submission; GPU time covers the mesh
pass. Completion wait, timestamp mapping and image readback stay separate.

| ABBA run | API draws | CPU submit median, ms | GPU mesh median, ms | Serial wall median, ms |
| --- | ---: | ---: | ---: | ---: |
| Baseline A1 | 8192 | 2.3213 | 1.3806 | 3.9804 |
| Candidate B1 | 1 | 0.1485 | 0.1674 | 0.4883 |
| Candidate B2 | 1 | 0.1188 | 0.1513 | 0.4280 |
| Baseline A2 | 8192 | 2.2768 | 1.3630 | 3.9586 |

All four HDR readbacks are byte-identical, SHA-256
`a3a56e83c70ec78fc39016ca57b115fc257f45b13f49fb3c823fb336d76aa3d4`.
Immutable baseline/candidate binaries were retained before running ABBA. No
shader compilation job ran intentionally alongside these measurements; the
Unreal SM6 cold compilation started afterwards. Driver and clocks were not
modified. This demonstrates a command-overhead improvement on this workload,
**not** an equivalent coastal/B02 or cross-engine speedup. Fragment-heavy,
interleaved or descending populations can gain less or retain individual draws.

```sh
THESSA_BENCH_OUTPUT=/absolute/new-output cargo test --release \
  -p thessa-render-wgpu --test transparency fixed_transparent_instance_submission_cost \
  -- --ignored --nocapture --test-threads 1
```

## Verification and milestone record

[Evidence](artifacts/transparent-instance-runs-2026-10-29.json) pins binaries,
source patches, CSV/HDR hashes, captures, platform and scope. Local command JSON,
source patches, both binaries and per-frame CSVs remain outside Git.

- **Source:** `14cedd8` / integrated `f8b00ae`; resulting commit introduces this doc.
- **Architecture/dependencies:** private iterator over the existing queue and
  existing indexed instance ranges; no dependency, production setting or shader change.
- **Checks:** Radeon and llvmpipe MASK plus three ordinary transparency fixtures;
  strict wgpu all-target Clippy; format/diff checks. The new textured-alpha GPU
  oracle compares an instanced draw against separate consumers using
  noncommuting colors, reversed instance order, a culled ID gap, and a third
  consumer inserted between depths. It also verifies the merged API count.
  Native B01/B05 PNG, HDR, GLB and JSON remain byte-identical to their previous
  captures. ABBA uses the same scene and original instance indices.
- **Not run:** Metal/WebGPU hardware, full workspace/coastal replay, vegetation
  density tiers or blind preference acceptance. No new physical model is introduced.
- **Assets/licenses:** generated test geometry only; MIT; no imported shader code.
- **Remaining work:** finish an actual Unreal HDR comparison, build native B02
  density tiers and measure their bottlenecks separately.
