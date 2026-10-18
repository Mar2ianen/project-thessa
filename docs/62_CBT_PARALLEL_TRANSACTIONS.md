# 62 — Parallel conforming CBT transactions

**Status (2026-10-18): measured, isolated GPU topology experiment.** Builds on
the committed/pushed sphere baseline `3b7bd17` ([phase one](61_CBT_SPHERE_EXPERIMENT.md)).
The new wave path changes multiple disjoint compatibility chains/diamonds in
parallel, with exact serial/CPU oracles and a 128K-capacity resident-pool fixture.
It is still a pure sphere, not production height/material/shadow/ocean/RT terrain.

## Acceptance boundary

**Bevy renderer parity is not a migration gate.** The new renderer is judged
against canonical surface data, conforming topology, explicit numerical/quality
budgets, correct native consumers and measured work. Preserving a runnable
legacy client is a regression constraint, not a requirement to copy Bevy's
appearance, internal layout, LOD policy or renderer.

The two controls below are **not Bevy**:

- an independent CPU reference checks geometric/topological correctness;
- serial execution of exactly the same accepted GPU plans checks parallel
  mutation byte-for-byte and measures the cost of parallelizing that stage.

Neither control establishes whole-system `large_cbt` replacement or game FPS.
No upstream HLSL/C++ is imported. The independent implementation continues to
use the published refinement/decimation ideas documented in phase one.

## Transaction wave

`SphereGpu::encode_batch()` runs these ordered compute stages:

1. Classify projected area for the active pool in parallel (view policy only).
2. Clear per-record claims and initialize the bounded diagnostic request window.
3. Plan up to **256 candidate requests** from a rotating active-index window.
   Each split caches its entire compatibility chain and preflights depth and
   generation headroom. Each merge requires a complete four-leaf diamond.
4. Claim **whole affected records**, including outside neighbors whose link
   fields will change, using `atomicMin(request ordinal)`. Locking only the
   subdivided leaves would still allow concurrent writes to a shared external
   neighbor, particularly across cube-face boundaries.
5. Validate all claims in parallel. A request is eligible only if it owns every
   claim. Priority is deterministic for this snapshot, not arrival-order racing.
6. Run a short **serial admission prefix over at most 256 requests**, not a
   serial pool traversal or topology mutation. Assign disjoint free-rank ranges
   only to whole chains that fit the declared logical capacity.
7. Decode every admitted chain's free slots in parallel **before any topology
   changes**. Reject generation-exhausted allocations without touching the pool.
   Merges' newly freed slots are not reused until the next wave.
8. Commit disjoint chains/diamonds in parallel. No inter-workgroup spinning,
   global shader barrier, 64-bit atomic, full rank reduction or CPU topology
   upload/readback is required.
9. Publish counters/cursor/indirect arguments, then decode the active index list
   in parallel for consumers. Wgpu pass boundaries provide stage ordering.

Conflict losers retry through later rotating windows; the admitted set is
conservative, not maximal. There is no claim of optimal scheduling, bounded
view-error convergence, or full-pool fairness under arbitrary moving cameras.
The fixed 256-request window is an explicit **experimental resource budget**,
not an unconfigurable production quality policy. Production integration must
resolve update/error budgets through graphics settings.

### Existing mechanisms retained

- The CPU reference still uses `BisectorPool` rather than another allocator.
  Initial uniform seeds reuse `Tree` and the same pool's materialization path.
- Tree-to-pool import now passes already-known child handles into recursion,
  avoiding a linear slot search at every split. The partition validator uses
  exact sorted max-depth intervals instead of repeated recursive slot scans.
  Corruption tests pin rejection of gaps, overlaps and wrong cached counts.
- Uniform sphere initialization pairs exact integer directed edges by lookup;
  the geometric validator retains those edge indices instead of rescanning the
  pool per edge. This permits an actual 98K-triangle CPU oracle fixture without
  changing topology semantics. Initialization is outside GPU cost samples.
- The GPU retains the existing OCBT-17 packed-rank/bitfield ABI. Only its WGSL
  access changes: occupied bits use atomic OR/AND, packed ancestor fields use
  atomic deltas. A field's count remains within its subtree's slot count, which
  fits its packed width. Adding/removing each distinct slot once therefore
  cannot carry/borrow into neighboring packed fields. Shared packed words and
  roots are checked against `OcbtPoolMirror` after concurrent updates.
- Shared split/merge primitives serve both the one-request control and parallel
  waves. `encode_batch(..., false)` plans/reserves the **same** wave but commits
  its requests in one invocation. It is not the previous one-request algorithm.

### Lifetime and transport

The batch API exposes explicit `plan → commit → publish` boundaries for scoped
timestamps; callers must complete them in order on a pool without interleaving
another update. Ordinary callers use `encode_batch()` for the complete wave.
Manual single-address requests still use the original `encode()` diagnostic;
batch mode accepts `set_view()` or the diagnostic `set_sweep()` policy.

The scratch buffer has four header words followed by 256 records of 200 u32s:
kind, source slot, chain length, required free slots, assigned free ordinal,
status, low/high logical address; then 64 chain slots and 128 predecoded free
slots. Final statuses are 0=idle, 1=committed, 2=capacity rejected, 3=depth limit,
4=ineligible, 5=generation headroom, 7=conflict. Transient planning/admission
statuses must not remain after publication. The header reports next cursor,
window size, committed requests and reserved-slot high-water count.

The adapter uses eight storage buffers plus one uniform binding, within portable
WebGPU storage-buffer limits; all atomics are u32. At 128K capacity, resident
buffer allocations total **5,991,868 bytes**, excluding driver/pipeline/staging
memory, CPU oracle state and any terrain data. This is not memory parity with
the paper's complete implementation. The experiment constructor retains both
controls; the earlier 2K fixture now allocates 314,812 buffer bytes rather than
phase one's 101,804 because it includes plans and claims.

No production caller constructs this experimental pipeline. No new environment
switch, implicit plugin activation or production graphics mode is introduced;
ordinary client costs/settings remain unchanged.

## Correctness evidence

The 360-wave small fixture starts with a uniform conforming sphere, fills the
pool, decimates back to the twelve roots, then applies projected-area decisions
and a camera teleport. Every wave:

- compares parallel and serialized GPU plans, records, generations, bitfields,
  packed ranks, active indices and counters byte-for-byte;
- replays each committed **GPU-chosen** address/operation into the independent
  CPU reference, comparing all logical neighbors;
- checks exact reversed geometric edges, full per-root coverage and Euler
  characteristic two on the CPU;
- requires the complete old-record footprint of each capacity-rejected chain
  to remain unchanged, even when unrelated transactions commit in the same wave.

The first verified run committed up to **61 transactions in one wave**, exercised
737 forced chains, 33,663 conflicts, 5,578 capacity rejections and 1,976 merges,
with 1,589 commits after teleport. These are fixture counters, not throughput
guarantees. The unit-sphere GPU position probe retains the absolute `1e-6`
per-coordinate decoder bound and bit-identical shared vertices; the observed
maximum error in the parallel fixture was about `5.12e-8`. This does not bound
chord-to-sphere tessellation error or planetary metre/cm precision.
The final batch topology also drives actual indirect raster: the nonempty
256×256 output must match the same-topology serialized control RGBA bytes
exactly. This is a deterministic consumer test, not blind aesthetic acceptance.

The large fixture starts at **98,304 active triangles** in a **131,072-slot**
pool, with four reserved virtual roots and maximum total heap depth 18. The
same-workload controls must publish identical bytes and commit identical
request counts. Final exact geometric edge/coverage/Euler and OCBT oracles run
on the raw GPU snapshot, not merely on a rendered image.

## Measured boundary

Hardware: Radeon 880M, Vulkan / RADV, Mesa 26.2.4-arch1.1. The large fixture
alternates diagnostic split/merge sweeps every 16 waves. Projected-area
classification's dispatch is present but its work is **inactive** under that
policy. This measures topology machinery, not height sampling or an autonomous
camera-classification budget.

Both controls start from identical independent GPU pools. Paired execution order
is counterbalanced **ABBA**. After 30 warmup waves, 100 individually timestamped
samples per control cover planning, commit, and publication/indexation. CPU
readbacks, byte comparisons, initialization and compilation are outside the GPU
timestamps. Percentiles use sorted sample indices 50/95; totals are sums of
consecutive intervals per sample, not sums of their medians.

The measured run committed **2,219 identical requests per control** during
the measured window, at most 29 per wave in this target-sized fixture, ending at
98,936 triangles:

| GPU scope, p50 | Serialized commit control | Parallel commit |
|---|---:|---:|
| Planning/admission/allocation | 0.1339 ms | 0.1343 ms |
| Commit identical accepted transactions | 0.1672 ms | 0.0111 ms |
| Publication + active-index decode | 0.1228 ms | 0.1239 ms |
| Whole topology wave | 0.4292 ms | 0.2718 ms |

These are isolated same-plan controls, **not** a comparison against Bevy, the
original one-request loop, `large_cbt`, a full renderer, or game FPS. One paired
campaign does not establish universal performance or a platform acceptance gate.
Exact final rerun values and executed gates are recorded in
[`artifacts/cbt-parallel-transactions-2026-10-18.json`](artifacts/cbt-parallel-transactions-2026-10-18.json).
Raw samples regenerate at
`crates/render-wgpu/target/test-tmp/cbt-sphere/batch-cost.csv`.
The recorded raw CSV hash is in the artifact; sample counts, identical commit
totals and total/commit p50/p95 were independently recomputed from that file.

## Reproduction

```sh
export TMPDIR=/home/chechulin/Projects/project-thessa/target/tmp
export CARGO_TARGET_DIR=/home/chechulin/Projects/project-thessa-fastrt/target
cargo test --release -p thessa-rcbt-core --lib
cargo test -p thessa-rcbt-wgpu sphere::tests
cargo test --release -p thessa-render-wgpu --test cbt_sphere -- --ignored --nocapture --test-threads=1
```

The first two sphere hardware tests retain the phase-one single-request control;
the additional tests exercise parallel waves and the target-capacity fixture.
The CBT fixtures start no audio-bearing app. Separate native regression windows
use the existing silent ALSA/Pulse/PipeWire test environment.

Local checks passed: workspace/all-target/all-feature clippy with warnings
denied, native no-default-feature clippy, fmt, **1,878 lib/bin +90 integration**
tests, **24 Bevy GPU +20 native ignored +3 backend ignored +4 CBT GPU** tests,
the strengthened indirect-raster rerun, portable WGSL validation, architecture
guard and five agent-QA tests. Ordinary workspace commands retain their ignored
suites; the list names hardware suites explicitly rerun, not every benchmark.
The Bevy suite is regression safety only, not a visual parity gate. No remote
CI, blind subjective preference or cross-platform runtime acceptance is claimed.

## Next native migration boundary

The next work is canonical height/material sampling and conservative geometric
error summaries, reusing existing residency rather than reintroducing geometry
patch LOD. It also needs local/f64-subtracted planetary precision, then the native
raster/shadow/ocean/RT publication contract. A resolved experimental native mode
needs disabled/cheaper fallback settings and measured numerical/quality budgets.
Concrete reuse points are `HeightPageStore`'s immutable revisioned snapshots,
the existing quantized `HeightPage` decoder/publication lifecycle in
`render-wgpu::terrain`, and `TileAnchor`'s stable local projection. The experimental
cube frames on faces 0..3 are rotated relative to `tile_anchor::face_frame`;
surface-page lookup needs an explicit canonical face-UV mapping, not direct
reuse of the experiment's local triangle coordinates as page UVs. Page keys,
height/material revisions and canonical seam ownership must remain independent
of transient bisector slots and render refinement.
Neither Bevy visual parity nor copying its internals is required. Until those
native consumers exist, deleting the working regular-grid path would remove
functionality rather than complete the migration.
