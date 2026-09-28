# CBT integration status — 2026-09-15

Status: integrated opt-in as of 2026-09-15 (default `cpu`, `gpu_indexed` via
`graphics.toml`). CPU fallback, indexed raster, and material pages shipped;
owner visual acceptance and CPU-vs-GPU numeric comparison remain gates;
persistent GPU topology stays future.

The CBT branch is now integrated into the workspace and the interactive client
at the scheduling/topology boundary.

## Delivered

- backend-neutral `thessa-rcbt-core` and packed CPU tree;
- pure Rust and optional upstream `libcbt` topology implementations, with the
  foreign path also serving conformance/performance comparisons;
- bound upstream `large_cbt` OCBT 128K/256K/512K/1M layouts for direct
  packed-buffer comparison;
- exact per-level bit-packed `CompactTree` rank storage, retaining the
  dirty-path update contract while cutting the depth-20 CPU mirror footprint
  by roughly 11x;
- universal `thessa-bevy-rcbt` plugin with bounded per-frame commits;
- optional Bevy render-world extraction of exact depth-37-safe CBT leaf
  records and quantized height pages into GPU storage/metadata buffers;
- generation-gated WGSL page sampling and 33x33 position/normal vertex
  expansion;
- portable GPU clip classification, screen-space grid steps, active-triangle
  compaction, canonical albedo sampling, and bounded LOD skirts in one
  indirect draw;
- an isolated native wgpu mesh-shader experiment kept outside the normal
  client build;
- explicit cube-sphere `TileKey` ↔ CBT Morton address mapping;
- live client submission of camera view and terrain split/merge candidates;
- compact `HeightPage` baking API in worldgen;
- optional portable `thessa-rcbt-wgpu` adapter;
- tests for topology, page error/serialization, plugin scheduling, and address
  round trips.

## Deliberate remaining work

The visible client mesh is still the legacy CPU fallback by default. The
render-world bridge transfers the committed topology and finished client pages
into GPU buffers, produces exact page-sampled vertices, and now has an opt-in
Bevy `Core3d` raster consumer with one procedural indirect draw, GPU-selected
active triangles, bounded LOD skirts, correct reverse-Z depth ordering, and no
per-leaf render-pass loop. The remaining gate is owner acceptance of the
visual regression scene plus a numeric CPU-vs-GPU geometry comparison at the
same camera views. Until that gate passes, the fallback remains the default
and authoritative surface queries stay on the canonical field.

The indexed raster path is selected explicitly with
`[renderer] terrain = "gpu_indexed"` in `graphics.toml`. The default remains
`"cpu"`; both keep the closed backdrop underneath missing pages. The normal
client does not request experimental mesh-shader features.

The client deliberately does not use the dense `CompactTree` buffer directly:
its cube-sphere address contract reaches depth 37, while the dense layout is
capped at depth 20. The extracted record stream preserves the full `u64` node
address as two `u32` words, so this boundary introduces no precision loss.

The topology backend policy is already two-track: pure Rust is the default
path, and `thessa-rcbt-ffi` provides a selectable runtime adapter over the
verified upstream `libcbt` implementation. The current Bevy path stays on
pure Rust because its client configuration reaches depth 37, beyond the
foreign backend's explicit 24-level bound; this is a deployment constraint,
not a benchmark-only classification.

The performance track also studied the upstream `large_cbt` demo. Its useful
lesson is not a DirectX renderer dependency: it keeps the CBT update state and
active triangle stream on the GPU, submits one procedural indirect draw, and
shades a visibility buffer in a screen-sized material pass. Our portable path
now adopts the single-draw, screen-space classification, active-triangle, and
LOD-sealing shape, while retaining CPU topology and the exact CPU field as
fallback authority. Persistent GPU topology updates, GPU page residency, and
the visibility/material pass remain future work; no DirectX API is part of the
project boundary.

The best local release capture reached 93.9 FPS p50 at 2560×1600 with
`gpu_indexed`, bloom disabled, VSync disabled, and a static pilot view. A
later repeat under the same requested mode measured 73.9 FPS p50, so the 90 FPS
target is not yet a stable result. These are renderer workload measurements,
not universal hardware claims; repeatability and owner visual acceptance remain
open gates. The typed default remains CPU terrain; this working tree explicitly selects
`gpu_indexed` with VSync disabled for the integration tests.


## 2026-09-15 render audit

See [the source-level comparison and regression results](38_CBT_RENDER_AUDIT_2026_09_15.md).
The current adapter now retains complete presentation snapshots, fills the
unrefined siblings of mixed-LOD selections, coarsens the planning tree, uses
correct position strides and finite border normals, and maps north-first
albedo with scene lighting. The indirect stream uses one instance as in the
reference. GPU-produced buffers no longer have uploaded zero-filled CPU
mirrors. The bridge is still a tiled renderer with CPU-owned topology; these
fixes do not constitute the paper's GPU bisector-pool implementation.
