# 22 — RCBT GPU terrain

Status: logical core, game scheduling, exact render-world leaf transport,
generation-gated GPU page transport, GPU screen-space active-triangle
classification, portable skirts, and opt-in Bevy `Core3d` procedural indirect
consumer are integrated, 2026-09-15. The legacy CPU mesh remains the default
until the GPU visual regression is accepted by the project owner.

## Boundary

```text
PlanetField / baked surface
    = authoritative physical surface

thessa-rcbt-core
    = backend-neutral tree, page, budget, and capability contracts

thessa-bevy-rcbt
    = Bevy frame input/output, bounded topology commit, render-world leaf/page
      storage, GPU page expansion, indexed indirect draw-list generation, and
      an opt-in portable `Core3d` indexed pass

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
  residual pages and expands each available leaf to a 33x33 cube-sphere vertex
  grid with normals;
- a portable GPU classifier that rejects leaves outside the clip volume,
  selects a screen-space grid step (1/2/4/8), compacts surface triangles plus
  bounded edge skirts into a persistent active-triangle buffer, and writes one
  indirect draw command;
- a procedural raster consumer that executes that one indirect draw. Its
  fragment stage samples the canonical body-fixed albedo map, and the draw
  path has no per-leaf render-pass loop;
- an isolated optional native wgpu mesh-shader experiment in the Bevy bridge
  crate;
  it is not compiled into the normal client and is not part of the portable
  game path;
- a portable `wgpu` adapter with WGSL validation and capability reporting;
- client terrain integration that submits the live body-frame view and
  cube-sphere split/merge candidates.

The current visible mesh remains the CPU tile path by default. It is an
adaptive fallback rather than one fixed grid: `[renderer] terrain_mesh_cells`
is the middle-field baseline, cover levels use a coarse 8..16-cell grid, and
L13+ tiles use up to twice the baseline (capped at 64 cells). This moves
geometry budget from the horizon into the ground that can affect the pilot
view; texture resolution remains controlled independently by the terrain
texture LOD policy. Close tiles also receive deterministic filtered material
grain and normal detail; that layer is visual-only and does not alter the
authoritative height field or collision queries.

The GPU bridge does height-page sampling and produces position/normal vertices
for the available leaves. Its reverse-Z Bevy `Core3d` consumer uses one
`DrawIndirect` command with one instance and three vertices per GPU-compacted
triangle. Camera-origin changes upload only the affine body-to-render-local
matrix; classification also checks the actual render matrix for roll and
viewport changes. The closed sphere is used for initial bootstrap; after the
first complete cover, the renderer retains that snapshot until every page of
its replacement is ready. Coarse/fine selection is partitioned without losing
unrefined siblings. See [the render audit](38_CBT_RENDER_AUDIT_2026_09_15.md). The
current topology and page residency are still CPU-owned, and every leaf still
gets a 33x33 working page, so this is a portable active-triangle consumer,
not yet a full persistent GPU CBT implementation.

## Reference demo study

The upstream `large_cbt` demo is a useful architecture reference, but it is
not a portable renderer to copy: its README explicitly targets DX12 and Shader
Model 6.6. The important performance properties are representation-level:

- CBT topology, classification, split/merge, balancing, allocation, and
  propagation stay in persistent GPU buffers and are driven by indirect
  dispatches;
- the visible mesh is a compact active-bisector/triangle stream, rendered by a
  single procedural indirect draw rather than by one patch/entity/material per
  leaf;
- a visibility buffer is shaded in a screen-sized material pass, so planet
  material work does not multiply with terrain patch count;
- planet-space calculations retain double-precision camera/planet coordinates
  and convert to camera-relative floats only at the raster boundary.

Our current path shares the single-draw and active-triangle submission shape,
but it still has CPU-owned topology, one fixed 33x33 page expansion per leaf,
and local material pages with scene lighting, camera exposure and analytic
ocean reflection. It does not claim parity with the reference demo's persistent GPU topology, visibility buffer, or material pass.
The next portable milestones are persistent dirty-path GPU topology updates,
GPU-managed geometry residency and a screen-sized visibility/material pass. Numeric
GPU readback regression now checks page interpolation, all six faces through
L17, finite edge normals, classification and the map projection; its tested
local L17 projection tolerance is one millimetre on Earth/Moon/Thessa fixtures;
this is separate from the 0.5 m terrain-page sampling budget. The CPU topology
and authoritative field stay as fallback and query authority throughout.

For launch-time visual smoke tests:

- set `[renderer] terrain = "gpu_indexed"` in `graphics.toml` to enable the
  indexed indirect path;
- leave it at `"cpu"` for the default legacy tile path. No environment switch
  requests hardware mesh features in the normal client.

For the CPU path, `[renderer] terrain_mesh_cells` controls the middle-field
tile grid density (8..64; the checked-in high preset uses 24). The actual
per-tile density is selected from the CBT/L0..L20 tile level: coarse cover is
cheaper, L12 keeps the configured baseline, and L13+ receives the near-detail
multiplier. The indexed CBT path keeps its fixed 33x33 page contract and
ignores this CPU-only density setting.

The indexed mode hides CPU tile entities and keeps the closed backdrop as a
low-resolution fallback while pages stream. Its material uses the global
equirectangular albedo with the project's east-positive longitude convention;
LOD transitions receive bounded radial skirts. The optional mesh-shader
experiment remains crate-local and requires an explicitly built experimental
feature.

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
   Bevy raster consumer; **GPU classifier, active triangles, skirts and
   fallback done; owner visual gate open**
2. page upload and vertex generation from `HeightPage`; **done**
3. cube-face seam and skirt handling equivalent to the current path;
4. an authoritative CPU readback/query path that does not require GPU RT;
5. absolute geometry error bounds against `PlanetField`;
6. target-batch benchmarks and a visual regression scene;
7. a fallback when adapter capabilities or measured workload do not justify
   the GPU path.

No vendor check or DirectX-specific API is part of this design. Backend choice
is capability-driven and remains behind wgpu/native rendering boundaries.

The follow-up also adds f64 tile anchors, bounded 256-layer material streaming,
and a filtered ocean material; see `38_CBT_RENDER_AUDIT_2026_09_15.md` for
implementation limits and reproduction commands.
