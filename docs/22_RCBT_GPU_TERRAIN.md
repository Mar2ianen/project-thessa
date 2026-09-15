# 22 — RCBT GPU terrain

Status: logical core, game scheduling, exact render-world leaf transport,
generation-gated GPU page transport, and opt-in Bevy `Core3d` indexed and
hardware mesh-shader consumers are integrated, 2026-09-15. The legacy CPU
mesh remains the default until visible-cover draw selection and material parity
are validated.

## Boundary

```text
PlanetField / baked surface
    = authoritative physical surface

thessa-rcbt-core
    = backend-neutral tree, page, budget, and capability contracts

thessa-bevy-rcbt
    = Bevy frame input/output, bounded topology commit, render-world leaf/page
      storage, GPU page expansion, indexed indirect draw-list generation, and
      opt-in `Core3d` indexed or hardware mesh-shader passes

thessa-rcbt-wgpu
    = optional portable wgpu adapter

apps/client terrain adapter
    = cube-face mapping, camera policy, materials, and fallback mesh path
```

`thessa-rcbt-core` has no Bevy, Tokio, or graphics dependency. `CbtBackend`
describes logical buffers, kernels, barriers, and dispatches using associated
types; it does not expose a `wgpu::Device` in the core API. The wgpu adapter
is therefore replaceable and is not required by the server.

## Current implementation

The workspace contains:

- `Tree`, `PackedTree`, deterministic split/merge planning, leaf snapshots,
  two-to-one balance support, and stable topology encoding;
- `HeightPage`, a compact quantized page with an absolute reconstruction-error
  contract and binary serialization;
- two selectable CPU topology implementations: the default pure Rust
  `Tree`/`PackedTree` path and the optional safe `libcbt` FFI path, with the
  latter also used for conformance and performance comparisons;
- `thessa-rcbt-large-ffi`, binding the upstream OCBT 128K/256K/512K/1M packed
  tree-plus-bitfield layouts for direct CPU/GPU-buffer comparison;
- `thessa-rcbt-core::compact::CompactTree`, an exact per-level bit-packed
  rank representation for the variable-depth CBT, with no approximate counts;
- `thessa-rcbt-wgpu::ocbt::OcbtPoolMirror`, a byte-compatible compact OCBT
  mirror whose mutations update only the packed ancestor path and return the
  dirty words for a sparse upload;
- a universal `CbtPlugin` with `CbtFrameInput`, `CbtRenderState`, and
  `CbtFrameOutput`;
- an optional `thessa-bevy-rcbt/render` bridge that extracts exact
  `[node_id_lo, node_id_hi, depth, ordinal]` records and matching
  quantized `HeightPage` payloads into Bevy render-world storage buffers;
- a generation-gated WGSL geometry pass that samples the packed signed-16
  residual pages, expands each available leaf to a 33x33 cube-sphere vertex
  grid with normals, and writes one indexed indirect command per leaf;
- an opt-in native wgpu mesh-shader pass that emits the same 33x33 patch as
  sixteen 8x8 meshlets directly from the quantized pages, without allocating
  the intermediate vertex/index buffers; the pass is capability- and limit-
  gated and keeps the indexed path as fallback;
- a portable `wgpu` adapter with WGSL validation and capability reporting;
- client terrain integration that submits the live body-frame view and
  cube-sphere split/merge candidates.

The current visible mesh remains the CPU tile path by default. The indexed GPU
bridge does height-page sampling, produces position/normal vertices plus a
standard `DrawIndexedIndirect` list, and can consume those buffers in a
reverse-Z Bevy `Core3d` pass. The hardware mesh path samples the same page
representation directly and emits sixteen meshlets per leaf; missing pages
become zero-output mesh workgroups. Both paths are explicit experimental
switches because visible-cover draw selection and material parity still need
validation. The indexed compute pass runs only when topology, page payloads,
or surface radius changes; camera-origin changes upload only the affine
body-to-render-local matrix.

For launch-time visual smoke tests:

- `THESSA_CBT_GPU_RASTER=1` enables the indexed indirect path;
- `THESSA_CBT_GPU_MESH=1` requests wgpu's experimental native mesh-shader
  feature and enables the direct mesh path.

Both modes hide CPU tile entities and keep the closed backdrop as a
low-resolution fallback. The mesh mode must be used only on an adapter that
exposes the requested wgpu feature; wgpu feature requests happen before device
creation, so an unsupported explicit request may fail startup. Without either
variable the normal cross-platform CPU path is unchanged.

The client reaches binary CBT depth 37 (`3 + 2 * tile_level`). A dense
`CompactTree` is capped at depth 20, so the client transport intentionally
uses exact depth-agnostic leaf records rather than forcing the game into a
smaller tree. This is the no-slowdown path: scalar dirty-path topology remains
the CPU hot path, and packed layouts are used where they reduce GPU bandwidth.

## CPU backend choice

Pure Rust is the default because it is portable, memory-safe at the language
boundary, and supports the full logical depth contract. `thessa-rcbt-ffi`
exposes the upstream `libcbt` implementation as an independently maintained
alternative. Its serial entry points are valid for an explicitly selected
runtime topology backend; its OpenMP entry points are optional and fall back
to serial compilation when the toolchain cannot provide OpenMP.

`thessa-rcbt-large-ffi` is the second external reference. It is not used as a
drop-in sparse topology API: the upstream OCBT contract is a dense bitfield
plus packed rank/sum buffers, with reduction semantics intended for GPU
parallelism. Its raw buffers are therefore bound directly for comparison and
for the later wgpu memory-pool port. The license status of the vendored
`large_cbt` snapshot is unresolved; redistribution requires a separate
provenance review.

The current universal Bevy plugin selects the pure Rust implementation by
default. The FFI backend is intentionally exposed as a separate runtime
adapter rather than hidden behind a mandatory native dependency; a future
client profile can select it where its single-threaded handle and depth cap
fit the target workload.

The foreign backend has an explicit supported depth range of 5..=24 because
the upstream heap footprint grows exponentially. It is therefore a useful
fast implementation and a proven cross-check, but it does not replace the
deeper pure Rust path for every terrain configuration. The two implementations
share observable split/merge semantics and are compared through the same
operation sequences.

The first release-build baseline on the development machine is recorded by
`cargo bench -p thessa-rcbt-core --bench cbt_vs_large`: the dirty-path
`PackedTree` processed 43.7M operations/s, versus 242.9K bit updates/s plus
full OCBT-1M reduction and 16.4K sparse split/merge operations/s plus full
`libcbt` reduction. This is a workload baseline, not a universal hardware
claim; the benchmark also reports the footprint tradeoff (8.65 MiB for the
current Rust packed tree versus 152 KiB for OCBT-1M).

The same workload now includes the exact precision-reduced `CompactTree`. The
latest local run measured 46.5M operations/s for `PackedTree` and 15.4M for
`CompactTree`, while reducing the depth-20 footprint from 8,650,752 bytes to
786,456 bytes. It is therefore not yet the CPU default; its purpose is to
provide the compact rank layout that makes the GPU bandwidth/atomic design
viable. No geometry or authoritative simulation precision is reduced here.

The wgpu crate also has a focused follow-up workload,
`cargo bench -p thessa-rcbt-wgpu --bench ocbt_vs_reduce`, which compares the
byte-compatible incremental OCBT mirror with the upstream `set_bit` plus full
`reduce()` contract at the same 1M-bit capacity. This is the relevant gate for
the future memory-pool/indirect-draw path: GPU propagation must preserve the
dirty-path property instead of merely moving the full reduction to a shader.
The first release run processed 12.36M incremental operations/s versus 0.201M
operations/s for upstream full reduction, with identical 155,900-byte
footprints; this is a local workload result, not a hardware-independent
claim.

## Required GPU replacement gates

Before making GPU CBT the default renderer, the implementation must provide:

1. visible-cover draw selection and a launch-time fallback around the existing
   Bevy raster consumer; **consumer done, parity gate open**
2. page upload and vertex generation from `HeightPage`; **done**
3. cube-face seam and skirt handling equivalent to the current path;
4. an authoritative CPU readback/query path that does not require GPU RT;
5. absolute geometry error bounds against `PlanetField`;
6. target-batch benchmarks and a visual regression scene;
7. a fallback when adapter capabilities or measured workload do not justify
   the GPU path.

No vendor check or DirectX-specific API is part of this design. Backend choice
is capability-driven and remains behind wgpu/native rendering boundaries.
