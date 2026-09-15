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
  expansion with one indexed indirect draw command per leaf;
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
into GPU buffers, produces exact page-sampled vertices and an indexed indirect
draw-list, and now has an opt-in Bevy `Core3d` raster consumer with correct
reverse-Z depth ordering. The remaining gate is visible-cover draw selection,
material parity, and a visual/numeric comparison against the CPU path at the
same camera views. Until those checks pass, the fallback remains the default
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

The performance track now has a third reference: `large_cbt`'s OCBT dense
bitfield/rank layout. The first local baseline shows the current dirty-path
Rust implementation ahead on sparse commits, while using more memory. The
next optimization is to retain that incremental CPU path and port the OCBT
buffer contract plus GPU allocation/propagation stages behind portable
wgpu/WGSL, rather than importing the upstream DirectX renderer. The first
byte-compatible mirror benchmark is 12.36M incremental OCBT operations/s
versus 0.201M operations/s for upstream full reduction at the same 1M-bit
capacity and footprint; it remains a local workload baseline. The variable-
depth `CompactTree` now measures 786,456 bytes versus 8,650,752 bytes for the
current scalar packed tree, at 15.4M versus 46.5M CPU operations/s. It remains
an opt-in format until GPU-side packed updates recover the scalar-path latency.

The license status of the vendored `large_cbt` source is unresolved; provenance
and redistribution require a separate review.
