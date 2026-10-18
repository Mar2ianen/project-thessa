# 61 — Persistent conforming CBT sphere experiment

**Status (2026-10-18): isolated CPU/GPU correctness prototype, not a production
terrain migration or `large_cbt` replacement.** Baseline `ff6a31f` (the reviewed
shadow optimization was committed and pushed before this experiment).

## Research boundary

Primary source: Anis Benyoub and Jonathan Dupuy, *Concurrent Binary Trees for
Large-Scale Game Components*, HPG 2024, [arXiv:2407.02215](https://arxiv.org/abs/2407.02215),
especially §2's compatible refinement and conservative diamond decimation and
§3's separation of allocation from subdivision addresses. The upstream README
at `AnisB/large_cbt` revision `7351e6fc603b9b2c2ab4da399b13a9ab0f327398`
identifies that paper and its DX12/SM6.6 demo. The inspected Git tree contains no
license file; no upstream HLSL/C++ is imported into runtime by this change.
Existing quarantined CPU oracle sources remain unchanged.

“Remove classical LOD” means replacing fixed render-patch grids/skirts and their
discrete step selection, **not eliminating adaptive geometric LOD**. The current
production `classify_grid_span()` still selects 1/2/4/8 and emits regular-grid
triangles. Deleting it before height/material residency, precision, shadows and
RT can consume the new topology would break the runnable client. It therefore
remains the production/control path. This first experiment proves the distinct
topology representation on a pure sphere before touching those consumers.
Bevy visual/implementation parity is **not an acceptance gate** for this
migration. Acceptance is defined by the canonical surface, conforming topology,
explicit numerical/quality budgets and working native consumers; the legacy
client remains runnable rather than prescribing the new renderer's internals.

## Implemented contract

- `rcbt-core::conforming::SphereTopology` is an independent, deliberately slow
  transactional CPU reference, reusing `BisectorPool` rather than introducing a
  second CPU allocator. Twelve oriented cube-face triangles have exactly paired
  directed edges, including face boundaries. Four reserved virtual roots allow
  the existing single binary-domain pool contract to remain intact.
- Heap IDs carry the geometric address; slot indices carry allocation identity.
  Roots occupy depth 4 (IDs 16..27); IDs 28..31 are reserved. Up to 54 further
  bisections fit the existing total-depth-58 `Node` contract. Pool capacity is
  independent of this depth; a 1,024-slot test reaches 64-bit IDs beyond u32.
  This is **not** a claim of depth 64 or centimeter-accurate planetary rendering.
- A slot contains 32 bytes: low/high u32 heap ID, depth, generation, three edge
  neighbors and padding. Edges are `a→b`, `b→c`, `c→a` with the last as refinement
  edge. Children are `[b, midpoint(a,c), a]` /`[c, midpoint(a,c), b]`. This is an
  explicit cube-triangle variant, not the paper's arbitrary-halfedge initializer.
- Split walks the compatibility chain to a mutual refinement-edge pair,
  preflights depth/generation and reserves **all** additional slots before
  changing a live edge. The chain then commits from coarse to fine. Exhaustion
  rejects the whole request, with no partial topology or occupancy mutation.
- Merge accepts only a complete four-leaf sibling diamond and restores both
  parents together. Cross-face neighbor links are changed locally; no skirts,
  fixed 33×33 vertex pages, grid-step selection or CPU-generated triangle list
  exists in this experimental draw.
- `rcbt-wgpu::sphere::SphereGpu` keeps the pool, OCBT allocation/rank buffers,
  active indices and indirect arguments resident. The new portable WGSL allocator
  consumes the **existing `OcbtPoolMirror` ABI**, with incremental packed ancestor
  updates and occupied/free rank decode. It does not reimplement a separate CPU
  allocation hierarchy or tie geometric depth to the rank-tree depth.
- First GPU implementation is **single-writer topology mutation**: parallel
  projected-area classification → one bounded compatibility/diamond transaction
  → parallel OCBT active-index decode → indirect sphere raster. There is at most
  one requested transaction per update; forced chain work can touch several
  diamonds. It is not the paper's concurrent multi-request propagation pipeline.
- CPU publishes only a camera/policy or a diagnostic logical request. Initial
  root upload is one-time. Ordinary experiment updates have no CPU topology
  readback/upload, full rank reduction, pool recreation or full topology rebuild.
  Explicit oracle readbacks are outside cost samples.

## Geometry, quality and lifetime boundary

The surface is the **unit sphere only**. Integer cube coordinates at scale 2²⁸
decode the bisection path exactly before f32 normalization; shared integer
vertices are identical across cube faces. The CPU oracle checks reversed edges,
complete root area, allocator partition and Euler characteristic 2. No-T-junction
coverage is a topology property, not an inference from a nice-looking screenshot.
An explicit GPU vertex probe compares the actual shared decoder against f64
normalized integer vertices with an absolute per-coordinate bound of `1e-6` on
the unit sphere, and requires **bit-identical** decoded positions for shared
vertices. This bounds decoder arithmetic, not the chord-to-sphere tessellation
error and not an astronomical-coordinate precision budget.

The prototype policy uses projected triangle pixel area, split/merge hysteresis
(`split_area > 4 * merge_area`), a declared maximum address depth and a bounded
pool. The fixture uses 400 /25 px² at 512×512. It does **not** guarantee a physical
or screen-space height-error envelope. Its centroid-facing visibility heuristic
is only an experiment policy, not a conservative production terrain visibility
test. Near-plane/silhouette quality, view-frustum prioritization, and camera-inside
behavior need a production contract before integration.

This is **test-only rendering**, deliberately not a selectable `graphics.toml`
production mode. No production caller constructs the new resources, and normal
viewer/Bevy settings and disabled costs are unchanged. Do not silently activate
this by plugin registration or an environment variable. A future native mode
must use `RequestedGraphics → resolution → ResolvedGraphicsSettings`, explicit
quality/error budgets and the existing portable fallback; it must not replace
the live planet with an untextured unit sphere.

The OCBT transport remains 128K-capable (depth 17); logical slot capacity can be
smaller, with allocation restricted to that prefix. The fixture uses 2,048
slots. Reserved roots are not visible triangles; four degenerate primitive
positions remain in the indirect command to avoid special compaction semantics.
Resource byte counts exclude driver, pipeline, staging and CPU-reference memory.

## Verification and reproduction

```sh
export TMPDIR=/home/chechulin/Projects/project-thessa/target/tmp
export CARGO_TARGET_DIR=/home/chechulin/Projects/project-thessa-fastrt/target
cargo test --release -p thessa-rcbt-core conforming -- --nocapture
cargo test -p thessa-rcbt-wgpu sphere::tests -- --nocapture
cargo test --release -p thessa-render-wgpu --test cbt_sphere -- --ignored --nocapture --test-threads=1
```

The hardware tests run on this machine, not skipped as adapter-less. The manual
GPU oracle checks semantic address/neighbor equality with the CPU reference after
every operation, allocation/rank bytes against the existing OCBT mirror,
exhaustion/recovery, invalid requests, maximum depth, slot reuse and high-word ID
carry. CPU randomized tests also check geometric conformity and complete merging
back to twelve roots. The autonomous GPU camera/teleport fixture replays each
**GPU-chosen** operation into the CPU reference; CPU does not supply its topology.
The generation-headroom fixture injects an exhausted generation, verifies
transactional rejection including the allocator bytes, and restores the fixture
to demonstrate recovery. Production generation reset/rebase is not implemented.

The test writes a wire capture to
`crates/render-wgpu/target/test-tmp/cbt-sphere/sphere.png`. The CBT fixture starts
no audio-bearing application; separate native regression windows use the silent
ALSA/Pulse/PipeWire environment recorded in the artifact. Timing is scoped GPU classification+update+index and
indirect raster, with compilation/readbacks outside samples. Absolute prototype
cost is not a speedup against the old renderer or the paper: there is no
same-surface-error, same-height/material/shadow/RT workload comparison yet.
The measured window is **not a converged static frame**: the log reports actual
split/merge/exhaustion counts for those samples. Warmup is 30 frames, followed by
100 individually timestamped GPU samples; CPU oracle synchronization/replay is
outside the timestamp brackets. Percentiles use sorted samples at indices 50/95.

Final isolated run: Radeon 880M, Vulkan / RADV, Mesa 26.2.4-arch1.1,
2,048 slots, 101,804 GPU buffer bytes, 512×512, final 342 visible triangles:

| GPU scope | p50 (ms) | p95 (ms) |
|---|---:|---:|
| Classification + one topology transaction + active-index decode | 0.1360 | 0.1615 |
| Indirect wire raster | 0.0284 | 0.0311 |

The 100-sample window committed **0 split requests /82 merges**, with no capacity
rejections; it continues decimation after the camera teleport. This is neither
steady-state convergence nor 128K-active-bisector throughput. The earlier serial
classification exploration measured 1.267 ms p50 for the update/index scope;
that guided parallel classification but was not a counterbalanced A/B campaign.

Executed checks, numerical scope and measurements are recorded in
[`artifacts/cbt-sphere-experiment-2026-10-18.json`](artifacts/cbt-sphere-experiment-2026-10-18.json).
Local gates passed: fmt; workspace/all-target/all-feature clippy with warnings
denied; native no-default-feature clippy; **1,876 workspace lib/bin +90
integration** tests; **24 Bevy GPU +20 native ignored +3 backend ignored +2 new
CBT GPU** tests; final portable shader validation, architecture guard and five
agent-QA tests. The ordinary workspace run retains its default ignored suites;
this list names which hardware suites were explicitly rerun, not every benchmark.
No blind aesthetic acceptance, remote CI or cross-platform runtime acceptance
is implied by deterministic topology and capture tests.

## Next migration gates (not implemented)

1. Parallel multi-request planning/reservation/conflict resolution with the same
   transactional exhaustion semantics; benchmark at the intended 128K pool size.
2. Canonical surface sampling through the existing height/material cache and
   explicit geometric-error summaries. Data pages/mips are not render-patch LOD.
3. Reuse f64 body/camera subtraction and local precision transport before
   claiming deep planetary geometry; f32 unit-sphere raster is not that solution.
4. Shared raster/shadow/RT/ocean geometry publication and stable revision/lifetime
   contracts; no accidental loss of off-camera casters or stale BLAS geometry.
5. Resolved experimental native mode, fallback, deterministic replay, numerical
   envelope, visual checks and same-error end-to-end measurements. Only then
   consider removing the legacy regular-grid renderer.
