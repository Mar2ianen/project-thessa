# 77 — Surface-first visual development

**Status:** in progress, not visual acceptance. Baseline is WORLD05 commit
`54cdfa6`, with the existing canonical WORLD00 shore and prototype plants.
Work is isolated on `feat/native-surface-showcase`; `main` is not changed.

## Target and constraints

Build one convincing, inspectable surface zone before extending its rendering
to the planet. Use WORLD00's surveyed land target at
23.565679258039204° N, 165.13881581863882° E, the unchanged canonical terrain
field and real stellar lighting. No separate showcase terrain, painted shore,
fake atmosphere or camera-specific geometry is introduced.

The standing (2 m eye AGL), close (50 m) and overview (800 m) views share the
numeric target. A moving camera and a surface-to-orbit sequence are additional
acceptance cases, not established by a still image. Plant geometry currently
covers one L14 query window only; this must not be called planet-wide coverage.

## Work order

1. Diagnose the nearly uniform ground and coarse shore in the WORLD05 frames.
   Record actual GPU material residency by level, not just CPU pages or zero
   missing-source counts. Test finite-capacity admission before adding detail.
2. Restore material detail and shore continuity through the existing streamed
   page/ancestor/filtering pipeline. Preserve bounded memory and coarse fallback.
3. Improve substrate/ground-cover and botanical representation where matched
   captures show a remaining gap. Reuse canonical ecology and mesh/material paths.
4. Compose water, terrain, foliage, shadows and sky; validate stability while
   moving rather than tuning an isolated still frame.
5. Expand the validated representation through bounded streaming and quality
   tiers. Retain world identities across transitions; never instantiate a whole
   planet's high-detail mesh/plant population at once.

The unfinished temporal scene integration remains in its separate temporal
worktree. It is not exposed as a working showcase feature here.

## Verification boundary

Same executable settings, source, camera, epoch, resolution and drained-stream
conditions are required for a before/after claim. Preserve PNG, JSON/CSV,
commands, hashes and level counters. Numerical/shader tests establish contracts;
an independent blind review establishes subjective preference. The implementation
owner may inspect frames for defects but cannot self-certify visual acceptance.
Performance requires same-workload measurements on the Radeon 880M/RADV baseline;
neither increased detail nor a different cloud/RT setting is a speedup result.

Current evidence remains WORLD05's
[`native-plant-prototypes-2026-10-21.json`](artifacts/native-plant-prototypes-2026-10-21.json).
Its frames demonstrate prototype geometry, not the requested final visual quality.

## WORLD06a — Saturated material admission

The first matched native captures confirm that the previous global level sort
filled all 256 GPU slots with L0–L9 pages at the standing camera; there were no
L10–L17 entries even after streaming drained. This was not missing CPU material
production. `resident_material_hierarchy` now protects the coarsest resident
support for each path, then emits each path coarse-to-fine in caller priority,
deduplicating shared sources. Native GPU publication, Bevy GPU publication and
CPU source-budget trimming reuse this one function. Texture capacity, page
formats, source versions, material filtering, canonical field and demand
selection are unchanged. Disconnected sources without coarser resident support
remain protected; very small budgets can still defer refinement.

`MaterialUpload.resident_levels[0..32]` counts actual GPU directory entries,
including retained cold slots, after each publication. Native initialization
and streaming retain the latest histogram, exported as
`native_material_resident_levels` in performance metadata and
`material_gpu_resident_levels` in the headless log. Counts are not CPU-ready
pages, material demand, visible-pixel coverage or a general LOD-quality metric.

The repeated standing capture contains 36 L17 GPU pages rather than zero. Ground
variation is still weak; plants are still schematic and the overview shoreline
still exposes coarse/fallback boundaries. This fixes a confirmed admission defect,
not the requested final visual quality. Actual PNG/JSON/CSV/log paths, hashes,
commands and all three residency pairs are recorded in
[`native-material-admission-2026-10-22.json`](artifacts/native-material-admission-2026-10-22.json).
The fixed 60-second prestream and measured frames were serial on the GPU, but
some CPU builds overlapped: timings are diagnostic, not stable performance
acceptance. No blind preference, camera-motion or planet-wide foliage claim is
made for this step.

The existing arena prepared the three unlabeled pairs under
`target/surface-showcase/review/`; no reviewer votes have been collected. Its
private answer key is outside that review directory and is not committed.

Verification on the 880M/RADV: formatting, architecture guard and workspace
Clippy passed; 2,029 workspace lib/bin tests, 103 integration tests and 79 ignored
renderer hardware tests passed. The mandated Bevy ignored-test command also
completed, but selects zero tests on this branch: retained GPU tests have moved
to `render-wgpu/tests/retained_surface.rs` and ran in the renderer suite. The new
saturated-cache GPU oracle checks actual blue fine-page RGB against red coarse
fallback, level counts and zero reuploads for unchanged demand; it passed
separately as well as within workspace integration tests. Tests establish cache
and raster correctness, not subjective quality or a performance improvement.
