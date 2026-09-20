# 22 — `rcbt`: GPU-driven adaptive terrain and baked surface hierarchy

Status: architecture baseline (§§1–18) normative; §19 baseline and §20
follow-ups archival as of 2026-09-15 — indexed raster, page provider,
extraction, and indirect draw shipped opt-in; persistent GPU topology and
neighbor propagation remain future.

Status: **implementation baseline / performance target**.

In the workspace the first packages are already added: `thessa-rcbt-core` (pure Rust
logical tree and backend contract), `thessa-rcbt-ref` (portable differential
oracle), `thessa-rcbt-wgpu` (WGSL/wgpu dispatch prototype) and
`thessa-bevy-rcbt` (thin client resource/plugin adapter). This is not yet a replacement
for the current client terrain renderer: the CPU tile path remains the fallback until parity.

This doc records the next major terrain step after the current very fast CPU tile builder. The current cube-sphere path is useful as a baseline and fallback: it proved that procedural geometry can be churned on the CPU at a rate on the order of kilometers per second and scaled well across cores. But the fixed tile grid remains too coarse a refinement unit: when only a few additional triangles are locally needed, a whole tile is built, and the CPU budget ends up bounded by the total amount of geometry that has to be produced at all.

The goal of the next stage is to **stop generating unneeded topology on the CPU**.

```text
canonical physical surface
        |
        +--> server/query/contact representation
        |
        `--> client height/page cache
                 |
                 v
              rcbt
         GPU adaptive topology
                 |
                 v
       render triangles / indirect draw
```

`rcbt` is the working name for a reusable Rust implementation of Concurrent Binary Tree / LEB-style adaptive triangulation. It is not a Bevy-specific subsystem and does not become the authoritative terrain.

---

## 1. Core architectural contract

Three things are kept separate:

```text
PlanetField / baked canonical surface
    = physical ground truth

rcbt
    = topology / adaptive visual representation

Bevy / wgpu / Vulkan
    = integration/backend
```

Consequences:

- the dedicated server does not depend on `rcbt`, Bevy, wgpu, or the GPU;
- the renderer can replace Bevy without rewriting the CBT algorithm;
- wgpu can be replaced with a direct Vulkan backend for the CBT path without changing the public algorithm API;
- CBT tree state does not define the physical surface;
- render triangulation and contact triangulation may differ as long as both fit within the declared physical error bound.

`wgpu` is a good default portable backend, but not an architectural dependency of `rcbt-core`.

---

## 2. Why CBT is closer to Nanite in role here, but not in construction

Both systems solve GPU-driven LOD/geometry selection, but the source representation differs:

```text
Nanite-like path
high-poly authored geometry
    -> offline clusters/hierarchy
    -> runtime GPU cluster selection

CBT terrain path
coarse continuous domain
    -> runtime binary split/merge topology
    -> displacement/height field
```

For Thessa the second model is the one that matters: the surface is global, continuous, and procedural/baked-hybrid, so it is cheaper to maintain topology as a compact adaptive tree than to continuously create and destroy CPU mesh tiles.

---

## 3. Planned crate split

Target split, not a requirement to create all crates immediately:

```text
rcbt-core
    pure Rust logical tree / addressing / layouts / scheduling contracts
    no Bevy, no wgpu, no Vulkan types

rcbt-ref
    reference/oracle wrapper over upstream libcbt
    test + benchmark dependency, not production runtime dependency

rcbt-wgpu
    portable GPU backend

rcbt-vulkan
    optional native Vulkan backend for lower overhead / missing features

bevy-rcbt
    Bevy RenderApp/extraction/camera integration only

Thessa terrain layer
    cube-sphere roots, body coordinates, height/page provider, material semantics
```

If the crate count gets in the way on an early prototype, it may physically live temporarily as fewer packages. The boundary is still considered part of the API design.

---

## 4. Backend API must abstract CBT work, not repaint wgpu

The following pseudo-abstraction is forbidden:

```rust
trait Backend {
    fn device(&self) -> &wgpu::Device;
    fn encoder(&mut self) -> &mut wgpu::CommandEncoder;
}
```

It merely leaks wgpu to the outside.

`rcbt-core` must operate on its own concepts:

```rust
pub trait CbtBackend {
    type Buffer;
    type Pipeline;
    type Commands;

    fn capabilities(&self) -> CbtCapabilities;
    fn create_buffer(&self, desc: BufferDesc) -> Result<Self::Buffer, BackendError>;
    fn create_pipeline(&self, kernel: CbtKernel) -> Result<Self::Pipeline, BackendError>;
    fn begin_commands(&self) -> Self::Commands;
    fn dispatch(
        &self,
        commands: &mut Self::Commands,
        pipeline: &Self::Pipeline,
        bindings: &CbtBindings<Self::Buffer>,
        groups: [u32; 3],
    );
    fn barrier(&self, commands: &mut Self::Commands, barrier: CbtBarrier);
    fn submit(&self, commands: Self::Commands) -> Result<(), BackendError>;
}
```

The exact trait shape is determined by prototypes/benchmarks. Meaning matters more than signatures: the public model describes CBT operations/resources/capabilities, not the types of a specific graphics API.

The capability model is likewise its own:

```rust
pub struct CbtCapabilities {
    pub subgroup_size: Option<u32>,
    pub subgroup_ballot: bool,
    pub storage_u64: bool,
    pub atomic_u64: bool,
    pub indirect_draw: bool,
    pub indirect_count: bool,
    pub persistent_mapping: bool,
    pub device_address: bool,
    pub cooperative_matrix: bool,
}
```

Do not write `if vendor == AMD/NVIDIA`. Fast paths are selected by capabilities and measured workload.

---

## 5. Shader/kernel boundary

`rcbt-core` must not treat WGSL as part of the semantic API.

Example logical kernels:

```rust
pub enum CbtKernel {
    Classify,
    Bisect,
    Simplify,
    Reduce,
    CompactLeaves,
    BuildDrawList,
    GenerateVertices,
}
```

The default portable implementation may be WGSL + Naga/wgpu. A native Vulkan backend may use SPIR-V-specialized kernels if that is measurably faster or unlocks the needed subgroup/device-address capabilities.

A shader fork is allowed only given:

- identical observable tree semantics;
- differential/regression tests;
- a benchmarked win;
- capability gate.

---

## 6. Upstream `libcbt` is an oracle and a benchmark target

Reference project: `jdupuy/libcbt`.

A thin FFI/reference adapter is needed, but **do not build the production path over C FFI**.

The reference is used for:

```text
same initial tree
same split/merge workload
        |
        +--> upstream libcbt
        `--> rcbt
                 |
                 v
compare logical state / encode-decode / leaf set / counts
```

Where the internal heap layout intentionally differs, observable topology is compared, not the byte-for-byte representation.

The FFI wrapper must be small and isolated. Bindgen does not have to become a build dependency of the whole workspace; for a small stable C surface, manual `extern "C"` declarations are acceptable.

The main goal of performance work is not "don't lose to C". **If a new representation can be faster, compatibility with the original's internal layout is not a goal.**

---

## 7. Performance philosophy: beat the reference, not port it

`rcbt` is not considered successful merely because the Rust implementation is within a few percent of the reference.

Target mindset:

```text
same mathematics / same observable tree semantics
                     |
                     v
      different representation and batching
                     |
                     v
 materially better latency / throughput / scaling
```

Main directions:

- packed bitsets / bitplanes instead of pointer-heavy node representation;
- dense `u64`/`u128` word processing;
- branchless classification/decode where measured useful;
- `popcnt`, `leading_zeros`, `trailing_zeros` and other native bit ops;
- SoA/AoSoA for metadata hot paths;
- batched split/merge decisions;
- thread-local mutation queues + bounded commit instead of fine-grained shared mutation;
- vectorized classification where it beats scalar bit tricks;
- no allocation in steady-state frame update;
- layout chosen for actual terrain update, not for a pretty object API.

A single `split(node)` benchmark is insufficient. The main workload is a moving-camera frame update, where most leaves stay stable, a small percentage splits/merges, then a compact active/draw list is built.

---

## 8. Rust-specific low-level rules: layout, padding, false sharing

Rust does not guarantee the layout of ordinary structs the way low-level code sometimes wants. For hot shared state, explicit "dirty" work will be necessary, but locally and measurably.

Rules:

- use `#[repr(C)]` on FFI/GPU ABI boundaries, not as a blanket performance annotation;
- `#[repr(align(N))]` / cache-padded wrappers are allowed for per-thread counters, queue heads, and frequently-written state if perf counters show false sharing;
- do not pad every struct "just in case" — extra footprint can hurt cache residency more than false sharing;
- do not treat the physical cache-line size as an eternal universal API constant; a platform-specific implementation may choose a reasonable alignment;
- separate hot read-only arrays from hot mutable counters;
- keep per-worker state separate, commit in large batches;
- GPU ABI structs have separate explicit layout tests.

Example implementation detail, not a public contract:

```rust
#[repr(align(64))]
struct Padded<T>(T);
```

Such a wrapper makes sense for a measured x86-64 false-sharing case, but must not leak into the logical tree API.

Benchmarks must include single-thread and scaling: padding that "sped up 16 threads" but slowed down 1 thread and bloated the working set must be judged by the real frame workload.

---

## 9. Baked canonical height hierarchy

The server is not required to recompute the expensive procedural field for every terrain query. The next surface format should bake the canonical low/mid spatial frequencies more aggressively, keeping a procedural residual only where it is cheaper than storage.

No global fixed raster down to 32 m is needed. A hierarchical cube-sphere/page representation is needed:

```text
planet
  face
    coarse page
      child pages ...
```

Each page may store:

```text
base_height
quantized residual samples
residual_scale
min_height
max_height
max_residual_error
max_slope_bound
semantic summary / optional material weights
```

Practical compact format candidate:

```text
per-page base: f32/f64
samples: i16 residuals
scale/bias: f32
```

Exact quantization is chosen by error budget, not up front.

`PlanetField::height_m(dir, min_wavelength_m)` keeps its semantic contract, but the implementation may choose:

```text
coarse/far query
    -> baked coarse page only

medium query
    -> baked finer page / residual

contact query
    -> finest baked page available
       + bounded short-wave analytic residual if required
```

In this way the source of truth becomes a hybrid: deterministic bake + deterministic residual, rather than mandatory recomputation of all macro/meso bands on every runtime query.

---

## 10. Why page bounds matter to the server

Conservative bounds need to be baked together with height.

This enables hierarchical rejection:

```text
trajectory segment
    if clearance > page.max_height + error + margin
        reject whole subtree
    else
        refine/query children
```

Landing search likewise first works from coarse slope/error bounds and only then refines candidate zones.

This is especially important for multiplayer/server scale: the best query is the one that never had to be made at thousands of points.

---

## 11. First CBT integration does not require porting all `PlanetField` math to GPU

The first version may be hybrid:

```text
CPU/offline:
canonical baked height pages
        |
        v
client GPU page cache
        |
        v
rcbt split/merge
        |
        v
sample/displace generated vertices
```

This already removes CPU topology churn.

Further renderer-only work can then be moved over as profiling dictates:

```text
baked low/mid frequency canonical height
+ GPU procedural meso/detail where safe
+ GPU cosmetic microdetail
```

The authoritative server result remains defined by the baked canonical representation + declared deterministic residual, not by renderer shader state.

---

## 12. Cooperative/matrix units: not for the tree, maybe for page decoding

The CBT hot path itself is a poor candidate for tensor/matrix hardware. Its workload is mostly:

- packed bits;
- ballot/popcount;
- scans/reductions;
- compaction;
- address/neighbor decode;
- split/merge decisions.

For this, subgroup/bit operations are more natural than matrix multiply units.

Do not turn a prefix sum or bit-tree update into GEMM just to tick the "tensor cores used" box.

But cooperative-matrix hardware may be useful **next to** CBT if baked height pages are later stored in a highly compressed representation.

Potential optional path:

```text
compressed height/residual page
        |
        v
small block / neural decoder
        | cooperative matrix when available
        v
height block in GPU cache
        |
        v
rcbt
```

This only makes sense if profiling shows that page bandwidth/storage matters more than decoder ALU cost and the compression ratio genuinely pays for the complexity.

`cooperative_matrix` is an optional capability. The core CBT correctness/performance contract does not depend on it.

---

## 13. GPU update pipeline target

Initial target pipeline:

```text
camera + error parameters
        |
        v
classify active leaves
        |
        v
batched split / merge
        |
        v
reduce / compact active topology
        |
        v
build draw/dispatch args
        |
        v
sample height / generate vertex attributes
        |
        v
indirect render
```

Optimization must consider the whole frame pipeline. If two kernels can be safely fused to remove a global memory pass/barrier, that is potentially more important than speeding up an individual kernel by 5%.

A native Vulkan backend is especially interesting for experiments with:

- explicit barriers/synchronization;
- subgroup features;
- buffer device address;
- persistent mapped/shared-memory paths;
- indirect count / generated draw data;
- vendor-neutral extensions, gated by capabilities.

Having such a backend does not cancel the portable wgpu path.

---

## 14. Benchmarks

### 14.1 CPU/reference micro + workload benches

Compare `libcbt` and `rcbt-core`:

```text
create/reset
encode/decode batches
stable traversal
random split workload
random merge workload
split/merge oscillation
moving-camera-like sparse mutation
full refinement stress
compact leaf-list construction
```

Sizes must reach the real number of candidate leaves, not toy trees.

Metrics:

```text
ns / update
M leaves / s
cycles / leaf
instructions / leaf
branch misses
L1/L2/LLC misses
bytes touched / leaf
allocations / update
peak working set
1 -> 2 -> 4 -> 8 -> 16 thread scaling
```

Linux benchmark path: custom harness/Criterion where appropriate + `perf stat`/`perf record`.

### 14.2 Differential correctness

After an identical sequence of operations, compare:

- leaf count;
- active logical leaf set;
- encode/decode roundtrip;
- parent/child relationships;
- neighbor/topology constraints;
- serialized logical snapshot where defined.

Fuzz/property tests must generate long split/merge sequences and compare against the reference oracle.

### 14.3 GPU benches

Measure separately:

```text
classify
update
reduce
compact
indirect-build
vertex generation
full CBT frame
```

And separately, an end-to-end terrain scenario:

```text
hover
200-400 m/s aircraft
1 km/s
3 km/s old CPU baseline point
5/8/12 km/s stress
low-AGL fast pass
fast turn
```

The main KPI is not raw triangle count, but frame cost + projected error + absence of holes + bounded memory.

---

## 15. Integration into Bevy

Bevy integration must be a thin adapter:

```text
Bevy camera / extraction
        |
        v
bevy-rcbt adapter
        |
        v
rcbt runtime/backend
```

`bevy-rcbt` is responsible for:

- extraction camera/view inputs;
- render-world resource lifetime;
- scheduling render/compute passes;
- integration with depth/material/shadows;
- debug visualization/metrics.

It does **not** own tree semantics, backend abstraction, or canonical terrain.

If Bevy later gains a suitable native GPU-driven terrain primitive, the adapter may become thinner or disappear. If Thessa moves away from Bevy, `rcbt-core` and the backend crates remain.

---

## 16. UMA/mobile considerations

UMA matters not only for desktop APUs, but also for mobile SoCs. Hence it is desirable to:

- minimize extra CPU copies and staging;
- avoid holding duplicated long-lived CPU/GPU payloads with no consumer;
- treat bytes touched and upload/copy bandwidth as KPIs on par with compute time;
- select shared-memory fast paths in a capability-driven way;
- not assume that host-visible memory is automatically fast for CPU reads;
- not make the mobile path a separate physics/terrain semantics.

CBT by itself reduces topology churn, and baked compressed pages can further reduce pressure on shared memory bandwidth.

---

## 17. Acceptance criteria for the first serious `rcbt` milestone

A feature is not considered successful merely because a picture appeared.

All of the following are required at once:

- differential correctness against `libcbt` reference for supported semantics;
- no Bevy/wgpu types in `rcbt-core` public API;
- portable wgpu backend works at least on Linux/Vulkan;
- native Vulkan prototype can use the same logical runtime API;
- current CPU tile renderer remains the fallback until parity is confirmed;
- terrain visual error bounded and measurable;
- no holes/cracks beyond declared fallback policy;
- steady-state update allocation-free or effectively allocation-free;
- real moving-camera workload materially faster than the reference CPU topology path;
- full terrain path removes CPU geometry generation as the dominant bottleneck;
- dedicated/headless server behavior does not depend on the presence of `rcbt`.

Performance target intentionally aggressive: **reference implementation — baseline to beat, not a speed ceiling to imitate**.

---

## 18. Implementation order

1. Pin the reference `libcbt` revision/license and build a tiny `rcbt-ref` oracle.
2. Build the differential test harness before aggressive optimization.
3. Implement a minimal pure-Rust logical tree.
4. Collect single-thread/reference counters and find the real hot representation.
5. Move to a packed/batched layout; only then add parallel update.
6. Add false-sharing/scaling probes and cache padding only where counters show the need.
7. Implement the portable GPU prototype (`rcbt-wgpu`) with a synthetic height field.
8. Connect Bevy via a thin adapter, not via types in core.
9. Connect the baked height page provider and compare against the current CPU tile renderer on identical camera routes.
10. Implement the native Vulkan backend only after a concrete measurable reason appears, not for the sake of Vulkan itself.
11. After that, explore compressed page formats.
12. Cooperative-matrix decoder — only a separate spike if memory/page bandwidth remains the bottleneck.

Do not optimize everything at once: the reference oracle and repeatable workload must exist before architectural tricks.

---

## 19. Current implementation baseline

The current code slice provides:

- heap-addressed `Node` values with the `libcbt` root/child/parent semantics;
- split and parent-merge operations with explicit errors;
- left-to-right leaf decode/encode symmetry;
- atomic update batches and deterministic bounded frame planning;
- terrain-adapter-supplied neighbor traversal with a bounded 2:1 balance pass;
- stable topology serialization with validation and round-trip tests;
- independent observable-topology differential tests against the reference
  model;
- backend-neutral `CbtBackend`, capability, binding, barrier, dispatch, and
  metrics contracts;
- a portable wgpu adapter with validated buffer ranges and compile-checked WGSL
  kernel entry points;
- a moving-camera-like sparse mutation benchmark (`cargo bench -p
  thessa-rcbt-core --bench tree`);
- a head-to-head bench against the real vendored C library
  (`cargo bench -p thessa-rcbt-core --bench cbt_vs_libcbt`): identical
  split/merge sequences replayed on both sides with leaf-set parity checks.
  The frames workload is a five-column table (same 2000x32 ops, same final
  topology): native-direct ~10 ms, native real plan+commit ~100 ms, libcbt
  public `cbt_Update` path ~2.9 s, libcbt sparse (batch FFI + reduce-only)
  ~210 ms, libcbt sparse through OpenMP/16t ~1.3-3.1 s on small live sets
  (thread overhead dominates there). In other words: the scary baseline for
  sparse commits is native-plan+commit vs libcbt-sparse at roughly x2, not
  x300; the x300 number compares raw local mutations against a full
  decode+reduce per frame, which is a different commit model, not a
  different CPU. Full refinement holds at ~x4-6 native, full decode at
  ~x9-14. OpenMP scaling of the C reduce path on a 65k-leaf refine, five
  repeats per level (median + spread, never a single sample): 1t 29.8+-0.4,
  2t 15.8+-1.0, 4t 8.7+-0.6, 8t 6.3+-4.0, 16t 606+-672 ms. Scaling is clean
  to 8 threads (~x4.7, and 8t already matches native bulk-refine pace);
  16t is confirmed bimodal, not a one-off: oversubscribed GOMP pool on this
  box, outside our code, needs an isolated-box retest before any claim.
  Rerun every table on your own machine before quoting it;
- a GPU sparse-commit implementation (`rcbt-wgpu` heap layout + `apply_ops`
  / `decode_all` WGSL kernels, `u32` atomics only, no extensions) measured on
  real hardware (`cargo bench -p thessa-rcbt-wgpu --bench gpu_cbt`, AMD 780M
  via Vulkan): mixed split/merge parity asserted against the CPU oracle,
  then refine columns cpu-native / libcbt-public / libcbt-sparse / gpu /
  gpu+readback / gpu-uma on IDENTICAL batches (d12-d18, all parities OK).
  Current tune: libcbt-sparse beats the GPU at 4-16k (0.1/0.6 ms vs ~2 ms,
  launch overhead dominates there) and roughly ties at 65k (2.7-2.8 ms);
  at 262k the GPU leads (8 vs 13 ms) while cpu-native trails both (34 ms).
  UMA verdict, measured not claimed: phase breakdown shows host transfers
  as the biggest GPU slice, but the single-submit `gpu-uma` path (one
  encoder, one final decode, reusable staging, integrated-GPU gate, never a
  vendor check) lands within noise of the per-batch pipelined path on every
  depth — batching does not win here because per-batch submits overlap CPU
  prep with GPU execution. Per the repo rule (no win, no API change) the UMA
  path stays bench-only; portable wgpu also forbids the true zero-copy
  (MAP_READ cannot combine with STORAGE, mapped_at_creation is
  MAP_WRITE|COPY_SRC-only), so on unified memory the remaining copy is
  already a plain memcpy with nothing left to take;
- a terrain-level `legacy-cpu vs rcbt-pages` comparison on shared selections
  (`cargo bench -p thessa-worldgen-rocky --bench compare`);
- a shipped-asset anchor (`cargo bench -p thessa-worldgen-rocky --bench
  assets`): the live recipe field reproduces `assets/worlds/thessa-v3`
  albedo at RMSE 0.0013 (linear, stride 16), legacy tiles match shipped
  pixels at RMSE 0.0001-0.005 with millimetre-exact heights, and RCBT pages
  on the identical footprints build in ~0.2 ms vs 18-67 ms at 198 bytes vs
  54-206 KB with millimetre-exact heights. Pages are geometry-only by
  design, so they carry no albedo column;
- an upstream study note (from the vendored `libcbt` source, not folklore):
  its reduction is a SWAR-vectorized bitfield prepass over the deepest six
  levels plus full per-level sums upward — always O(heap size), never
  O(dirty set). Decode walks root-to-leaf per leaf with bitfield extracts.
  Split/merge themselves are single bit writes. That is exactly the cost
  our sparse commit sidesteps, and the measured gaps match the asymptotics;
- a packed single-threaded tree (`rcbt-core::packed::PackedTree`, depth cap
  20 from the 8 MiB dense-sums footprint) with identical observable
  semantics to `Tree`, differentially tested against it on refine + sparse
  + invalid-op sequences. It closes the loop the BTreeSet tree left open:
  refine d12-d18 at x22-x59 over `libcbt` (0.0-2.6 ms vs 1.2-150 ms),
  decode-all at x26, sparse k-sweep at x7-x1950 with parity on every point.
  The upstream baseline is beaten on CPU at every measured workload; no
  SIMD was needed — profiling showed pointer chasing and allocation, not
  vectorizable ALU, and per the repo rule (§10) no intrinsics were forced.
  The cap is explicit: deeper trees stay on `Tree`, the crossover bench
  routes by measured cost;
- a dirty-op sweep on a fixed 262k-leaf tree
  (`cargo bench -p thessa-rcbt-wgpu --bench crossover`): T_cpu(k),
  T_sparse(k), T_gpu(k) for k = 4..16384 with leaf-set parity on every
  point, plus the computed k_crossover lines the backend uses to place a
  commit. Current tune on 780M: k_crossover(gpu-full < cpu-native) = 4096,
  k_crossover(gpu-full < libcbt-sparse) = 16 (first crossing; the k=64
  point is noisy, so the backend must use the curve with hysteresis, not a
  single threshold);
- dynamic wave scenarios (`cargo bench -p thessa-rcbt-core --bench dynamic`
  for CPU, dynamic section of `crossover` for GPU): an analytic 3-wave
  heightfield plus an optional sweeping camera drive error-based split/merge
  every frame (120 frames, ~120 ops/frame pure waves, ~320 with camera,
  leaf counts oscillating 3.7k/6.3k). Same op streams on all sides, parity
  everywhere. Current tune: waves — packed 0.13 ms vs native 1.7 (x13) vs
  sparse 12.5 (x96) vs public 179 (x1370); waves+camera — packed 0.35 vs
  native 5.0 (x14) vs sparse 13.1 (x37) vs public 366 (x1000+). GPU on the
  same streams: ~16-27 ms with per-frame readback, ~15-16 ms commit-only —
  i.e. the kernels are idle most of the time and per-frame host round-trips
  (submit+poll+buffer/bindgroup alloc, ~0.15 ms x 120) dominate. That gap is
  the quantitative case for follow-up 2, not a GPU-speed verdict;
- a workgroup ancestor-combining apply kernel (`apply_ops_combined`,
  `cutoff` uniform, shared-memory `acc[1024]`, unconditional barriers so
  op-less threads still participate). Dense 32k-split batch: baseline
  ~0.93 ms vs ~0.86/0.79/0.78/0.70-0.80 ms at cutoff 0/6/8/10 across runs
  (roughly -10..-25%, run variance is real) — confirming that upper-node
  atomic contention is the dense-batch bottleneck. Sparse k=4096:
  combining is neutral (1.15 vs 1.04-1.09), confirming the overhead only
  pays when ancestors are actually shared. Cutoff stays a measured knob,
  not a constant.

This baseline intentionally does not claim the M2.5 exit criteria. Baked page
provider, LEB/cube-sphere neighbor balancing, indirect terrain draws, Bevy
extraction, and visual error captures remain the
next integration layers. No server or authoritative `PlanetField` code may
depend on them.

---

## 20. Open follow-ups with measured status

1. **Batch resolve phase (required before production GPU commits).**
   Status: empirically motivated, not implemented. The crossover bench
   initially fed ancestor/descendant-overlapping batches to concurrent GPU
   threads and corrupted topology exactly as predicted (sequential CPU
   replayed the same batches fine). Current benches guarantee disjoint
   parents by construction; shared upper ancestors are safe (commutative
   adds only). A production classifier must either guarantee the same or
   run a resolve phase first.
2. **Persistent compact leaf/draw list.** `decode_all` walks root-to-leaf
   per leaf every frame. Once `k_crossover` routing exists, the natural
   next step is maintaining the compact list incrementally and skipping
   full decode on frames the renderer does not need it. Measured motivation:
   on wave dynamics the GPU commit itself is ~15 ms per 120 frames while
   per-frame readback pushes it to ~17-27 ms — and the CPU packed path does
   the same 120 frames in 0.13-0.35 ms total, so every host round-trip must
   go before GPU dynamics can compete at small k.
3. **Native Vulkan UMA residency.** The portable-wgpu UMA experiment is
   closed (within noise, stays bench-only). Untested and still worthwhile:
   native Vulkan with `HOST_VISIBLE | DEVICE_LOCAL` backing, persistent
   mapped/shared memory, explicit CPU/GPU ownership and sync — motivated
   by single residency (memory/power on APU/mobile), not by FPS alone.
