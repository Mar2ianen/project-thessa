# 64 — Review of the proposed native pipeline optimizations

Historical rejected-proposal snapshot. The corrective implementation is tracked
in [doc 63](63_CBT_CANONICAL_SURFACE.md); retain the measurements and failures
below as the original audit, not as current test outcomes.

**Status (2026-10-18): not accepted; GPU counterexamples reproduced.**
This dated review concerns the local Pass 1/2/3 proposal and supplied captures.
The verified, pushed selection baseline is `e386ef0`; the later optimization
edits remain uncommitted. This is not performance acceptance.
Machine-readable evidence: [review artifact](artifacts/native-optimization-review-2026-10-18.json).

## Blocking correctness findings

1. **LOD wake conditions are incomplete.**
   `terrain_topology.rs::update_view_anchored` does not key rotation/projection.
   Extent/bound changes upload policy but do not reset `convergence_budget` or
   dirty geometry. After `encode` returns at zero budget, they cannot refine.
   The hardware reproducer settled at epoch **9**, resized **257 → 4096**, then
   rotated **180 degrees** at the same position: both new epochs remained **9**.
   A fixed wave count is a work budget, **not a convergence test**; no GPU
   remaining-work certificate is checked by this proposed stop condition.

2. **Shadow depth is reused after actual caster coordinates change.**
   `mesh.rs::update_ranges` copies changed camera-relative staging into
   `shadow_instances`, but advances `shadow_revision` only for changed world
   `MeshInstance` values/ranges. `terrain.rs::update_shadows` uses the retained
   terrain anchor, while mesh casters use current camera-relative matrices.
   Moving the camera **1 m** in a fixed world scene (below the proposed 2 m
   reanchor cutoff) caused **zero cached shadow passes**, although a forced
   reference draw changed **510 depth texels**. Casters and receivers must share
   a declared origin; changing only the revision key is not a complete fix.

3. **Ocean phase is inconsistent with the retained terrain origin.**
   `update_ocean_profiled` publishes residues for camera `C`, while retained
   vertices are relative to anchor `A`. `ocean_spectral` therefore uses
   `(P-A)+mod(C, period)`, not `(P-A)+mod(A, period)`: an unwanted `C-A` phase
   offset at world point `P`. Near-field taper also assumes the eye is at zero.
   RT hit placement separately converts to the current camera origin, so primary
   and secondary hits need not sample the same phase. This is a code/coordinate
   finding; a displaced-origin GPU phase oracle was **not** run here.

The existing short production-consumer GPU test still passes; it does not cover
post-budget viewport/rotation changes or cached fixed casters after camera motion.
The new audit reproducer fails its required-invariants assertion, exit **101**.

## Supplied timing evidence

| Capture | Wall p50 (ms) | p95 (ms) | p99 (ms) |
|---|---:|---:|---:|
| `final-preview-2560-sweep` | 18.6575 | 40.1829 | 43.3651 |
| `pass3-2560-sweep` | 14.5559 | 34.0728 | 38.2609 |
| `final-preview-720p-sweep` | 6.6662 | 30.9490 | 33.5694 |
| `pass3-720p-sweep` | 5.0788 | 26.1757 | 31.6491 |
| `final-preview-2560-static` | 14.4821 | 15.8654 | 17.4999 |
| `pass3-2560-static` | 10.2936 | 11.1419 | 16.7440 |
| `final-preview-720p-static` | 4.2171 | 5.0616 | 5.7194 |
| `pass3-720p-static` | 3.3641 | 3.8947 | 4.2101 |

These medians are reproducible, but they are not controlled revision comparisons:

- The files contain 120 **already-measured** frames after 30 warmups. Taking
  `frames[30:]` discards another 30 measured frames, not warmups.
- Paired prestream duration differs (**1 vs 3 s**), with live async residency.
  All eight build commits are **`unknown`**, without source/binary hashes.
- Baseline `1e86e72` does not contain `terrain_topology.rs`; the capture settings
  include persistent conforming topology. The claimed commit attribution is not
  established by those files.
- The reported raster **13.7019 → 10.4597 ms** is exactly **frame 10**, not the
  median. Full-capture raster p50 is **15.8486 → 11.9362 ms**.
- Surface/shadow cost exceeds 1 ms in **40/39 of 120** frames of the 2560 sweep
  pair and **39/38** of the 720p pair. A 0.002 ms median does not eliminate that
  cost. Reciprocal median serial wall time is not sustained game FPS.

## Plausible improvements, unproven attribution

- `new_relative` has the right mathematical translation relation, not guaranteed
  bit identity after different f32 rounding. All consumers must agree on origin.
- Back-face culling is a candidate for outward positive-radius triangles, not
  proof of 50% less fragment work. Deep topology/horizon cases need oracles.
- Explicit depth discard is suitable if no later consumer reads depth; the
  preserving default API remains. It is a backend hint, not evidence of exactly
  16.38 MB less external traffic. Culling/discard were not measured separately.
- `render-core::shadow::directional_clip` already performs real f64 xy snapping.
  The proposal did not introduce that algorithm. The xy cell alone does not key
  all depth/origin/caster changes.

The 2560 sweep image difference is reproducibly **0.09938397/255** mean, maximum
**26**, but **817,234 pixels** have a changed RGB component. At 720p it is
**0.14761068/255**, maximum **25**, with **234,381** changed pixels. A small global
mean neither localizes error to the horizon nor excludes holes, temporal faults,
wrong shadows or phase drift. The supplied amplified-diff snippet multiplies
`uint8` before clipping and can wrap; widen first. This is not blind acceptance.

## Reproduction and boundaries

Hardware: Radeon 880M / Vulkan / RADV Mesa `26.2.4-arch1.1`. The audit tree includes
the local optimization edits plus in-progress page-height-envelope binding. The
reproducer uses flat zero-height pages, independently of those envelope changes.
This is not a byte-identical, hashed original Gemini build; none was supplied.

Snapshots/hashes, exact commands, exit statuses and logs are in
`target/test-tmp/optimization-review/`. Reproducer:
`crates/render-wgpu/tests/optimization_review.rs`. RT is disabled to isolate LOD
and the shared raster atlas; this is not a traversal/performance test.

```sh
TMPDIR=/home/chechulin/Projects/project-thessa/target/tmp \
CARGO_TARGET_DIR=/home/chechulin/Projects/project-thessa-fastrt/target \
cargo test --release -p thessa-render-wgpu --test optimization_review \
  -- --ignored --nocapture --test-threads=1
```

Do not push the optimization proposal as verified before correcting invalidation,
origin propagation, actual convergence detection and matched-work measurements.
Doc 63's full green gate record belongs to **`e386ef0`**, not this working tree.
