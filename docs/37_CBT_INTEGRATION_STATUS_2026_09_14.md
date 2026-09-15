# CBT integration status — 2026-09-14

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
  expansion with one procedural indirect draw and one instance per leaf;
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
Bevy `Core3d` raster consumer with one procedural indirect draw, correct
reverse-Z depth ordering, and no per-leaf render-pass loop. The remaining gate
is visible-cover draw selection, material parity, and a visual/numeric
comparison against the CPU path at the same camera views. Until those checks
pass, the fallback remains the default and authoritative surface queries stay
on the canonical field.

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
now adopts the single-draw submission shape, while retaining CPU topology and
the exact CPU field as fallback authority. The next step is portable GPU
screen-space classification and active-triangle compaction; no DirectX API is
part of the project boundary.
