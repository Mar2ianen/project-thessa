# 22 — `rcbt`: GPU-driven adaptive terrain and baked surface hierarchy

Статус: **implementation baseline / performance target**.

В workspace уже добавлены первые packages: `thessa-rcbt-core` (pure Rust
logical tree и backend contract), `thessa-rcbt-ref` (portable differential
oracle), `thessa-rcbt-wgpu` (WGSL/wgpu dispatch prototype) и
`thessa-bevy-rcbt` (thin client resource/plugin adapter). Это ещё не замена
текущего client terrain renderer: CPU tile path остаётся fallback до parity.

Эта дока фиксирует следующий major terrain step после текущего очень быстрого CPU tile builder. Текущий cube-sphere path полезен как baseline и fallback: он доказал, что procedural geometry можно молотить на CPU со скоростью порядка километров в секунду и хорошо масштабировать по cores. Но fixed tile grid остаётся слишком грубой единицей refinement: при локальной потребности в нескольких дополнительных triangles строится целый tile, а CPU budget в итоге упирается в суммарное количество геометрии, которую вообще приходится производить.

Цель следующего этапа — **перестать генерировать ненужную topology на CPU**.

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

`rcbt` — рабочее имя reusable Rust implementation Concurrent Binary Tree / LEB-style adaptive triangulation. Он не является Bevy-specific subsystem и не становится authoritative terrain.

---

## 1. Главный архитектурный контракт

Три вещи не смешиваются:

```text
PlanetField / baked canonical surface
    = физическая истина

rcbt
    = topology / adaptive visual representation

Bevy / wgpu / Vulkan
    = integration/backend
```

Следствия:

- dedicated server не зависит от `rcbt`, Bevy, wgpu или GPU;
- renderer может заменить Bevy, не переписывая CBT algorithm;
- wgpu может быть заменён direct Vulkan backend для CBT path без изменения public algorithm API;
- CBT tree state не определяет физическую поверхность;
- render triangulation и contact triangulation могут отличаться, если обе укладываются в declared physical error bound.

`wgpu` — хороший default portable backend, но не архитектурная зависимость `rcbt-core`.

---

## 2. Почему CBT здесь ближе к Nanite по роли, но не по устройству

Обе системы решают GPU-driven LOD/geometry selection, но исходная representation разная:

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

Для Thessa важна именно вторая модель: поверхность глобальная, непрерывная и процедурная/baked-hybrid, поэтому topology выгоднее поддерживать как compact adaptive tree, а не непрерывно создавать и уничтожать CPU mesh tiles.

---

## 3. Planned crate split

Target split, не требование немедленно создать все crates:

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

Если crate count на раннем prototype мешает работе, физически это может временно жить меньшим числом packages. Boundary всё равно считается частью API design.

---

## 4. Backend API must abstract CBT work, not repaint wgpu

Запрещён псевдо-abstraction вида:

```rust
trait Backend {
    fn device(&self) -> &wgpu::Device;
    fn encoder(&mut self) -> &mut wgpu::CommandEncoder;
}
```

Он только протаскивает wgpu наружу.

`rcbt-core` должен оперировать собственными concepts:

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

Точная форма trait определяется prototype/benchmarks. Смысл важнее сигнатур: public model описывает CBT operations/resources/capabilities, а не типы конкретного graphics API.

Capability model также свой:

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

Нельзя писать `if vendor == AMD/NVIDIA`. Fast paths выбираются по capabilities и измеренному workload.

---

## 5. Shader/kernel boundary

`rcbt-core` не должен считать WGSL частью semantic API.

Пример logical kernels:

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

Default portable implementation может быть WGSL + Naga/wgpu. Native Vulkan backend может использовать SPIR-V-specialized kernels, если это измеримо быстрее или открывает нужные subgroup/device-address возможности.

Шейдерный fork допустим только при наличии:

- одинаковых observable tree semantics;
- differential/regression tests;
- benchmark выигрыша;
- capability gate.

---

## 6. Upstream `libcbt` is an oracle and a benchmark target

Reference project: `jdupuy/libcbt`.

Нужно сделать тонкий FFI/reference adapter, но **не строить production path через C FFI**.

Reference используется для:

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

Там, где internal heap layout intentionally отличается, сравнивается observable topology, а не byte-for-byte representation.

FFI wrapper должен быть маленьким и изолированным. Bindgen не обязан становиться build dependency всего workspace; для небольшого стабильного C surface ручные `extern "C"` declarations приемлемы.

Главная цель performance work — не «не проиграть C». **Если новая representation может быть быстрее, compatibility с внутренним layout оригинала не является целью.**

---

## 7. Performance philosophy: beat the reference, not port it

`rcbt` не считается успешным только потому, что Rust implementation находится в пределах нескольких процентов от reference.

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

Основные направления:

- packed bitsets / bitplanes вместо pointer-heavy node representation;
- dense `u64`/`u128` word processing;
- branchless classification/decode where measured useful;
- `popcnt`, `leading_zeros`, `trailing_zeros` и другие native bit ops;
- SoA/AoSoA для metadata hot paths;
- batched split/merge decisions;
- thread-local mutation queues + bounded commit instead of fine-grained shared mutation;
- vectorized classification where it beats scalar bit tricks;
- no allocation in steady-state frame update;
- layout chosen for actual terrain update, not for a pretty object API.

Одиночный `split(node)` benchmark недостаточен. Главный workload — moving-camera frame update, где большинство leaves остаётся stable, небольшой процент split/merge, затем строится compact active/draw list.

---

## 8. Rust-specific low-level rules: layout, padding, false sharing

Rust не гарантирует layout обычных structs так, как иногда хочется low-level code. Для hot shared state придётся явно заниматься «грязью», но локально и измеримо.

Правила:

- `#[repr(C)]` использовать на FFI/GPU ABI boundaries, а не как blanket performance annotation;
- `#[repr(align(N))]` / cache-padded wrappers допустимы для per-thread counters, queue heads и frequently-written state, если perf counters показывают false sharing;
- не паддить каждый struct «на всякий случай» — лишний footprint может ухудшить cache residency сильнее, чем false sharing;
- physical cache-line size не считать вечной универсальной константой API; platform-specific implementation может выбрать разумный alignment;
- hot read-only arrays отделять от hot mutable counters;
- per-worker state держать раздельно, commit делать крупными batches;
- GPU ABI structs имеют отдельные explicit layout tests.

Пример implementation detail, не public contract:

```rust
#[repr(align(64))]
struct Padded<T>(T);
```

Такой wrapper имеет смысл для измеренного x86-64 false-sharing case, но не должен протечь в logical tree API.

Benchmark обязан включать single-thread и scaling: padding, которое «ускорило 16 threads», но замедлило 1 thread и раздуло working set, должно оцениваться по реальному frame workload.

---

## 9. Baked canonical height hierarchy

Server не обязан заново вычислять дорогой procedural field для каждого terrain query. Следующий surface format должен сильнее bake'ить canonical low/mid spatial frequencies, сохраняя procedural residual только там, где это выгоднее хранения.

Не нужен глобальный fixed raster до 32 m. Нужна hierarchical cube-sphere/page representation:

```text
planet
  face
    coarse page
      child pages ...
```

Каждая page может хранить:

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

Практичный compact format-кандидат:

```text
per-page base: f32/f64
samples: i16 residuals
scale/bias: f32
```

Точная quantization выбирается по error budget, не заранее.

`PlanetField::height_m(dir, min_wavelength_m)` сохраняет semantic contract, но implementation может выбрать:

```text
coarse/far query
    -> baked coarse page only

medium query
    -> baked finer page / residual

contact query
    -> finest baked page available
       + bounded short-wave analytic residual if required
```

Таким образом source of truth становится hybrid: deterministic bake + deterministic residual, а не обязательное повторное вычисление всех macro/meso bands на каждом runtime query.

---

## 10. Why page bounds matter to the server

Вместе с height нужно bake'ить conservative bounds.

Это позволяет делать hierarchical rejection:

```text
trajectory segment
    if clearance > page.max_height + error + margin
        reject whole subtree
    else
        refine/query children
```

Landing search аналогично сначала работает по coarse slope/error bounds и только потом уточняет потенциальные зоны.

Это особенно важно для multiplayer/server scale: лучший query — тот, который не пришлось делать в тысячах точек.

---

## 11. First CBT integration does not require porting all `PlanetField` math to GPU

Первая версия может быть hybrid:

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

Это уже убирает CPU topology churn.

Далее можно переносить renderer-only work по мере профилирования:

```text
baked low/mid frequency canonical height
+ GPU procedural meso/detail where safe
+ GPU cosmetic microdetail
```

Authoritative server result при этом остаётся определён baked canonical representation + declared deterministic residual, а не renderer shader state.

---

## 12. Cooperative/matrix units: not for the tree, maybe for page decoding

Сам CBT hot path — плохой кандидат для tensor/matrix hardware. Его workload в основном:

- packed bits;
- ballot/popcount;
- scans/reductions;
- compaction;
- address/neighbor decode;
- split/merge decisions.

Для этого subgroup/bit operations естественнее matrix multiply units.

Не надо превращать prefix sum или bit tree update в GEMM только ради галочки «tensor cores used».

Но cooperative-matrix hardware может быть полезно **рядом** с CBT, если baked height pages позже будут храниться в сильно compressed representation.

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

Это имеет смысл только если profiling показывает, что page bandwidth/storage важнее decoder ALU cost и compression ratio действительно окупает complexity.

`cooperative_matrix` — optional capability. Core CBT correctness/performance contract не зависит от него.

---

## 13. GPU update pipeline target

Исходный target pipeline:

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

Оптимизация должна рассматривать весь frame pipeline. Если два kernels можно безопасно слить и убрать global memory pass/barrier — это потенциально важнее, чем ускорить отдельный kernel на 5%.

Native Vulkan backend особенно интересен для экспериментов с:

- explicit barriers/synchronization;
- subgroup features;
- buffer device address;
- persistent mapped/shared-memory paths;
- indirect count / generated draw data;
- vendor-neutral extensions, gated by capabilities.

Наличие такого backend не отменяет portable wgpu path.

---

## 14. Benchmarks

### 14.1 CPU/reference micro + workload benches

Сравнивать `libcbt` и `rcbt-core`:

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

Размеры должны доходить до реального числа candidate leaves, а не toy trees.

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

После одинаковой sequence операций сравнивать:

- leaf count;
- active logical leaf set;
- encode/decode roundtrip;
- parent/child relationships;
- neighbor/topology constraints;
- serialized logical snapshot where defined.

Fuzz/property tests должны генерировать длинные split/merge sequences и сравнивать с reference oracle.

### 14.3 GPU benches

Отдельно мерить:

```text
classify
update
reduce
compact
indirect-build
vertex generation
full CBT frame
```

И отдельно end-to-end terrain scenario:

```text
hover
200-400 m/s aircraft
1 km/s
3 km/s old CPU baseline point
5/8/12 km/s stress
low-AGL fast pass
fast turn
```

Главный KPI — не raw triangle count, а frame cost + projected error + absence of holes + bounded memory.

---

## 15. Integration into Bevy

Bevy integration должна быть thin adapter:

```text
Bevy camera / extraction
        |
        v
bevy-rcbt adapter
        |
        v
rcbt runtime/backend
```

`bevy-rcbt` отвечает за:

- extraction camera/view inputs;
- render-world resource lifetime;
- scheduling render/compute passes;
- integration with depth/material/shadows;
- debug visualization/metrics.

Он **не** владеет tree semantics, backend abstraction или canonical terrain.

Если Bevy позже получает подходящий native GPU-driven terrain primitive, adapter может стать тоньше или исчезнуть. Если Thessa уходит с Bevy, `rcbt-core` и backend crates остаются.

---

## 16. UMA/mobile considerations

UMA актуальна не только для desktop APU, но и для mobile SoC. Поэтому желательно:

- минимизировать лишние CPU copies и staging;
- не держать дублированные long-lived CPU/GPU payloads без consumer;
- считать bytes touched и upload/copy bandwidth такими же KPI, как compute time;
- capability-driven выбирать shared-memory fast paths;
- не предполагать, что host-visible memory автоматически быстра для CPU reads;
- не делать mobile path отдельной физикой/terrain semantics.

CBT сам по себе снижает churn topology, а baked compressed pages могут дополнительно уменьшить pressure на shared memory bandwidth.

---

## 17. Acceptance criteria for the first serious `rcbt` milestone

Фича не считается успешной просто потому, что картинка появилась.

Нужно одновременно:

- differential correctness against `libcbt` reference for supported semantics;
- no Bevy/wgpu types in `rcbt-core` public API;
- portable wgpu backend работает хотя бы на Linux/Vulkan;
- native Vulkan prototype может использовать тот же logical runtime API;
- current CPU tile renderer остаётся fallback до подтверждения parity;
- terrain visual error bounded and measurable;
- no holes/cracks beyond declared fallback policy;
- steady-state update allocation-free or effectively allocation-free;
- real moving-camera workload materially быстрее reference CPU topology path;
- full terrain path снимает CPU geometry generation as dominant bottleneck;
- dedicated/headless server behavior не зависит от наличия `rcbt`.

Performance target intentionally aggressive: **reference implementation — baseline to beat, not a speed ceiling to imitate**.

---

## 18. Implementation order

1. Зафиксировать reference `libcbt` revision/license and build tiny `rcbt-ref` oracle.
2. Сделать differential test harness до агрессивной оптимизации.
3. Реализовать минимальный pure-Rust logical tree.
4. Снять single-thread/reference counters и найти real hot representation.
5. Перейти на packed/batched layout; только затем добавлять parallel update.
6. Добавить false-sharing/scaling probes и cache padding только там, где counters показывают необходимость.
7. Реализовать portable GPU prototype (`rcbt-wgpu`) с synthetic height field.
8. Подключить Bevy через thin adapter, не через types в core.
9. Подключить baked height page provider и сравнить с current CPU tile renderer на одинаковых camera routes.
10. Реализовать native Vulkan backend только после появления конкретной measurable причины, не ради самого факта Vulkan.
11. После этого исследовать compressed page formats.
12. Cooperative-matrix decoder — только отдельный spike, если memory/page bandwidth остаётся bottleneck.

Не оптимизировать всё одновременно: reference oracle и repeatable workload должны существовать раньше архитектурных трюков.

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
