# 85 — Native depth vertex streams and physical GPU validation

Status: feature-branch continuation, 2026-10-28. Baseline `c23498b`.
Extends [the shadow sample selection checkpoint](84_NATIVE_SHADOW_RASTER_SAMPLES.md).
Physical GPU diagnostics were explicitly authorized by the user in this session;
the previous reset incident remains recorded. The runs use only the render node,
headless Vulkan, explicit admission, serial fences and kernel-event inspection.
These namespaces do not contain a device-wide reset. No driver/watchdog setting
was changed.

## Transport contract

A mesh created with the shared raster shadow atlas stores positions in a
12-byte vertex stream and normal/UV attributes in a separate 20-byte stream.
Primary rendering reads both streams plus the existing optional AO stream.
Depth rendering reads only positions. Source values, vertex/index order,
materials, triangle counts, instance indices, transforms and shader arithmetic
are unchanged. Total vertex payload stays 32 bytes per vertex; data is split,
not duplicated or quantized. RT retains its existing independent source.
The public CPU `Vertex` model and 64-byte rigid-instance transport remain.

Without raster shadows, the existing interleaved 32-byte source and AO binding
remain. No split attribute allocation or extra vertex binding is added to that
path. The split primary path uses three vertex-buffer slots, within the existing
default device negotiation. No new backend feature, pass, dispatch, texture or
per-frame source upload is introduced. Source and shadow memory admission remain.

Full and alternate depth sources now use the same 12-byte input layout and
share one depth pipeline. The optional alternate-pipeline handle remains the
existing resolved geometry-selection gate and aliases that pipeline. Missing,
invalid or ineligible depth levels still select original full geometry. TAA
remains default-off.

The independent [Arm vertex-efficiency guide](https://learn.arm.com/learning-paths/mobile-graphics-and-gaming/optimizing-vertex-efficiency/vme-learning-path/)
explains separating positions from attributes that a depth pass does not consume.
That is an architecture idea, not a prediction of Radeon performance. The
[NVIDIA pipeline analysis guide](https://developer.nvidia.com/gpugems/gpugems/part-v-performance-and-practicalities/chapter-28-graphics-pipeline-performance)
likewise recommends changing vertex-format size to identify transfer cost.
The implementation is independent; hardware results below measure this change.

## Diagnostic reuse and image contract

`ShadowAtlas::encode_layer_for_diagnostics` encodes one selected layer through
the production caster implementation. Its sample-culling flag selects the
existing production or no-sample oracle. A private query groups the existing
selection flags and optional layer; ordinary callers still encode every active
layer. `read_layer_depth` bounds-checks the index and copies only that layer.
Full-atlas readback remains available. The direct/indirect regression verifies
that twelve individual submissions/readbacks reproduce the complete atlas,
colour and draw statistics at two origins, including fallback and dirty reuse.

The physical coastal diagnostic reuses the fixed retained producer and matrices.
It first discards its whole-plan encoders. It then submits layers separately with
30-million mesh-triangle admission per submission using the existing durable
`GpuTrace`, completes a fence after each stage, and reads actual GPU timestamps.
A long interval stops subsequent replay. All eight blocks per layer compare the
complete layer depth against their reference. This measures serial shadow stages,
not overlapped whole-frame throughput. Instrumented wall times include durable
trace writes and must not be interpreted as calibrated frame time.

The actual plant fixture also has an explicit physical-GPU entry point. Its
existing coverage/colour limits, full-source versus original-tier references,
failed-update atomicity, source identities and cache assertions remain. Physical
profiling uses counterbalanced blocks with timestamps and separate completion
wall times. CPU-only entries still require the isolated CPU adapter.

## Accepted and rejected candidates

The accepted change is source transport and depth-pipeline reuse. The canonical
plan remains **208,101,356** mesh shadow triangles, 17,992 retained mesh instances,
17,111,160 primary triangles and the previous source/command admission. The
0.5-texel geometry policy and original full-source fallback remain.

A reassociated matrix/vector shader was rejected before performance acceptance:
it changed two coverage samples, 187 colour bytes and common-sample depth by
0.000020563602 in a bounded physical fixture. Production WGSL was not changed.

An affine-box extension to sample selection was also rejected. It reduced the
plan to 205,517,378 triangles, but did not establish a repeatable GPU advantage
worth the additional CPU selection work. The sphere implementation from doc 84
is retained. Its source snapshot, raw metrics and rejected shader counterfactual
are archived separately from final acceptance. No image tolerance was relaxed.

## Measured physical result

AMD Radeon 880M Graphics (RADV STRIX1), Mesa 26.2.5, Vulkan. Four release-test
processes ran interleaved/split/split/interleaved. Within each process, each of
twelve layers used ABAB/BABA selection blocks; vertex-format comparison uses
only the same sphere-selected **208,101,356-triangle** plan. Central values are
medians of the eight block medians per layer across the two processes per format,
then summed across layers. They are not pooled-frame percentiles or game FPS.

| Matched mesh source | Sum of layer GPU central values |
| --- | ---: |
| Interleaved 32-byte full-source stride | 102.782 ms |
| Split 12-byte position stride | 67.907 ms |

This is **33.93% lower** for those serial shadow intervals. The three first-cascade
layers fall from approximately 13.2 ms each to 5.4 ms each; distant layers already
using 12-byte depth proxies change much less. Layout and common-pipeline reuse
are measured together; these results do not isolate their individual effects.

Every complete coastal layer depth image is byte-identical across all four
processes. Saved full/proxy plant depth and colour images are also byte-identical
across the vertex formats. The first matched full plant timing changes from
about 2.77 to 1.53 ms, a bounded fixture result rather than coastal/frame FPS.
Kernel inspection recorded no new GPU fault/reset for these accepted runs.

## Final implementation verification

The final source passed 16 checks: formatting, strict all-target/all-feature
Clippy, 524 CPU tests across native/graphics/render-core/worldgen/bounds,
feature-off and legacy-client checks, architecture validation, release product,
and render-oracle/physical-test builds. Eight isolated software-Vulkan cases
passed: seven rendering regressions and one dense command-plan probe that drops
its encoders without submitting the coastal geometry.

Five final physical replays passed on the Radeon 880M: direct/indirect atlas
and individual-layer readbacks, authored plants/receiver, dense coastal layers,
primary transform/rebase/failed-update atomicity, and no-shadow 4x MSAA coverage.
The final twelve coastal layer images match every frozen A/B/B/A image byte for
byte. The four saved plant depth/colour files also match both frozen formats.
No new GPU fault/reset was detected in the per-run kernel records.

The [machine-readable evidence](artifacts/native-depth-vertex-streams-2026-10-28.json)
records commands, driver, source/binary hashes, exact image comparisons, raw
block medians and rejected candidates. Final non-document source-manifest SHA-256:
`66e9cf8fc6f782244307c30766b7b3986d92241a6801953312eb578a354d12db`.
Release product SHA-256:
`2f94e5f997e34afdcce9e31a853f9841c1a6633eb594b6b8b1d245f1667ca965`.

Physical native headless renders also completed at 320x180 and 1280x720, with the
checked profile except explicit Vulkan and RT off. It rendered 30 warmup frames
and one measured frame using existing serial stage tracing, 4x MSAA and TAA off.
The screenshots and traces are evidence of native integration, not FPS acceptance.
The static-camera trace still encodes all twelve shadow layers on frame 30.
`ConformingSurface::encode` advances its publication revision on every actual
encode, and the terrain shadow cache includes that revision. Topology already
has a generation/completion sleep gate; that gate did not eliminate the repeated
work in this capture. The next investigation is which update or transaction
keeps it active. Deleting the surface invalidation would permit stale terrain
depth and is not a valid fix. This transport change leaves that gate intact.

Whole-frame moving-camera time, RT stability, independent reference preference,
and full replacement acceptance remain open.

The [sidedness/demand investigation](86_NATIVE_OPAQUE_SIDEDNESS.md) extends this checkpoint. A longer fixed-source coastal replay confirmed topology convergence after the earlier 30-warmup window. Publication invalidation remains; the next changes target opaque fragment execution and repeated identical CPU selection.
