# Native GPU stage trace and mesh-caster continuation — 2026-10-27

Implemented after the confirmed WORLD142 GPU reset/session loss. Source
`5d71b8e8f3859cef1768e5261b6b1446c20e91081e0ec89cf20c4432e6f005ca`, release binary
`2c7c01e6a8825e57f6b27b50ce1ca07a91202077330fefeea51948407b7dfb83`, base commit `397adc6e312f7542112122a8a5d8fa567d5b5c7f`.
The forest candidate remains uncommitted and not hardware validated.

## Changes

- `--gpu-trace NEW.toml` records process ID, adapter, settings, host milestones,
  durable submission intentions and queue-fenced stage completion. It covers
  warmup and final readback, rejects existing/aliased output files and conflicts
  with performance capture. Normal rendering/performance capture has no new
  serial trace fences or trace admission checks.
- Trace-only per-stage mesh admission defaults to 1,000,000 triangles with a
  positive explicit override. Oversized primary work exits before frame-render
  submission. Encoded shadow work is checked again before submission. Source
  geometry and scene quality are never silently changed by admission.
- Shadow mesh instances are conservatively rejected only outside a light
  volume. Bounds reuse the actual uploaded f32 transforms and shared clip tests;
  off-camera shadow casters remain. Original indices and contiguous ranges are
  preserved without an extra instance buffer. Candidate/submitted mesh work is
  reported per actually encoded shadow redraw. The full reference bypasses
  both terrain and mesh rejection.

## Executed validation

- 178 CPU cases: 43 render-core, 3 caster numerical/range cases,
  132 native cases. Formatting, strict affected Clippy, feature-off compile,
  architecture, debug/release builds and compile-only hardware oracle passed.
- The extended full-versus-culled depth-atlas and final-color oracle passed
  with nonempty depth and actual mesh rejection on software Vulkan. Its renderer
  source is identical to the final tree; only native trace/PID/path validation
  changed after that test executable was built.
- Seven actual CLI cases passed on software Vulkan: a 31-frame small X-15 scene
  including 30 warmup frames, over-budget rejection before any trace render
  submission, missing-headless rejection, perf conflict, zero-limit rejection,
  direct output collision and symlink output collision. The positive trace has
  693 events and 310 queue-completed stage records, covering 10 stages.
  Negative cases were checked for the intended error, not just a nonzero exit.

## Software isolation

Mesa's 2.33 MiB `vulkan-swrast` 26.2.5 package was extracted into the workspace,
verified against the local package database SHA-256 and selected by a private
absolute ICD manifest. Nothing was installed into the system. Each actual
renderer test ran in Bubblewrap user/PID/network namespaces with a fresh `/dev`
without `/dev/dri`, no selected display socket, and a read-only host filesystem
(except its explicit diagnostic output directory). The native launcher asserts
these exclusions immediately before exec. `vulkaninfo` and the actual stage
trace both identify the sole adapter as llvmpipe, deviceType CPU.

[Khronos documents the explicit-driver override](https://github.com/KhronosGroup/Vulkan-Loader/blob/main/docs/LoaderDriverInterface.md).
The physical Radeon device was not used. Its watchdog/recovery, system driver
and compositor were not reconfigured. The existing physical-GPU incident
marker was not removed or bypassed for hardware work. Software timings are
not Radeon speed or desktop-stability evidence.

The small preview uses the retained North American X-15 Plane asset by cmoreau,
CC BY4.0; see `X15-SOURCE-LICENSE.txt` in the local evidence bundle.

## Remaining limits

- The software CLI scene is a96x64 X-15 preview with RT, clouds, persistent CBT, motion and temporal resolve disabled; completion records include empty stages, not acceptance of those features.
- Radeon hardware, desktop stability and the original full coastal workload were not rerun. The incident replay marker remains in place.
- A mesh-triangle admission threshold does not bound shader/compute duration and cannot guarantee protection from driver/GPU hangs.
- Serial fences and fsync change scheduling and may mask a race. Trace timings are not performance measurements.
- Primary mesh submission still contains the same geometry and density; this change adds light-volume shadow rejection, not a primary raster workload reduction.
- Software full/cull equality is scoped to the extended synthetic oracle, not a visual/reference-parity acceptance for the forest.
- The original failing draw/shader instruction remains unknown; no new RADV wave/command hang dump was captured.

Full commands, hashes and counters are in
[`native-gpu-trace-caster-2026-10-27.json`](native-gpu-trace-caster-2026-10-27.json).
Local evidence bundle: `/home/chechulin/Documents/Codex/2026-10-23/new-chat-3/outputs/gpu-trace-review-2026-10-27`. User-facing flag and numeric
contracts are in docs 15/60; incident cause and raw workload are kept separately
in `native-gpu-reset-2026-10-27.md`.
