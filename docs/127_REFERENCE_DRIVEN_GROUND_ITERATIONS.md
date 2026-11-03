# 127 — Reference-driven ground iterations and CBT planet source audit

Status: source audit and reviewer/worker iteration loop established 2026-11-03.
The new external engine was inspected as source; its build, visual quality and
performance have not been validated here. No reference-parity claim is made.

## Roles and iteration record

The user requested GPT-6 Luna as implementation worker with an independent
reviewer owning research, acceptance tests and integration. The current runtime
has no sidebar-thread creation tool; Luna runs as a subagent in a separate Git
worktree. The worker cannot approve its own visual output or relax the
reviewer-owned acceptance cases.

Every candidate records one concrete research/engine mechanism, inspected
source revision and paths, the matching existing Thessa owner, reproduced
failure, changed behavior, actual checks/capture and the review decision.
Technical correctness and subjective preference remain separate. An increase
in vegetation counts or a decrease in source bytes is not an acceptance result.
Before integrating a milestone, README, feature docs and the status index must
agree with code. The blind arena in doc 55 remains the required mechanism for
an actual blind-preference claim.

## Verified primary references

| Reference | Inspected source and mechanism | Thessa consequence |
| --- | --- | --- |
| [Large CBT, Benyoub/Dupuy, HPG 2024](https://ggx-research.github.io/publication/2024/07/27/publication-cbt.html), [upstream](https://github.com/AnisB/large_cbt) | Persistent bisector pool, neighbor conformity, allocation/release, indexation and indirect rendering. Existing audit pins upstream `7351e6fc603b9b2c2ab4da399b13a9ab0f327398` (doc 98). | Remains the main terrain architecture reference. Topology, source sampling, material residency and full frame cost require separate evidence. |
| [OpenEngine](https://github.com/Game-Crafters-Guild/OpenEngine/tree/bdf5c2710fe37c1c5ddbac7787477daa87524af3) | Actual `CBTTerrain`, `CBTTerrainECS` and `TerrainGrass` source at `bdf5c2710fe37c1c5ddbac7787477daa87524af3`; repository MIT license. | A verified open game-engine implementation with spherical CBT terrain, rather than only a terrain demo. It may be the project the user remembered; identity is not confirmed. |
| Unreal 5.8.4 installed source | `LandscapeGrass.cpp:2388–2440,2588–2665`: triangle-barycentric root height and triangle normal, followed by instance alignment. | Fix placement against the actual surface representation. Current canonical-sampler roots are a narrower first step. |
| [AMD procedural grass](https://gpuopen.com/learn/mesh_shaders/mesh_shaders-procedural_grass_rendering/) | Quadratic centerlines, blade patches, distance reduction and width compensation. | Preserve pointed silhouettes and projected coverage when changing geometry; CPU reuse alone does not solve foliage appearance. |
| [AMD Hybrid Shadows](https://github.com/GPUOpen-Effects/Hybrid-Shadows/tree/0cf18422a73eda8b4a23870091372a7717387f3e) | MIT sample pinned at `0cf18422a73eda8b4a23870091372a7717387f3e`: `Classify.hlsl`, `ShadowRaytrace.hlsl`, tile masks and reduced ray intervals. | Existing FastRT scalar skip policy (doc 111) is not this classifier. Full receivers, queues, conservative confidence and cache invalidation remain implementation work. |

The downloaded research bundle and its source catalogue are already indexed
by doc 113. Its shadow references are AMD Hybrid Shadows plus Unreal's static/
dynamic cache invalidation, not merely a generic extra RT pass.

OpenEngine's CBT shader comments explicitly describe porting upstream
`large_cbt/update_utilities.hlsl`. The upstream licensing issue recorded in doc
98 therefore still applies to copying that algorithm implementation through a
second repository. The current audit compares mechanisms; Thessa's production
Rust/WGSL implementation remains independently derived from research/contracts.
Unreal is inspected locally as engine source; no Epic implementation is copied
into the repository.

A second candidate, [World42's author's CBT description](https://forum.babylonjs.com/t/wip-real-scale-planet/56910/5),
links a WebGPU GPU-driven planetary CBT branch. Git access currently reports
repository not found. It is not treated as inspected source or a shipped
reference here. Tempest's own documentation says public source distributions
are unavailable; its feature description cannot substitute for source review.

## OpenEngine mechanisms checked against Thessa

- `CBTTerrain/Shaders/CBT/cbt_kernels.comp` implements classify/split/allocation,
  propagation, simplification and indexation; `CBTLayout.h` distinguishes physical
  slots from bisector heap IDs. Compare with Thessa's existing conforming pool
  and doc 103 before introducing another topology mechanism.
- `cbt_planet_page.glsl` reconstructs a fragment-local coordinate from a CPU
  split anchor and local offset, then samples paged height and gradients.
  Thessa already has camera-local precision charts, split material addresses
  and analytic chart derivatives (docs 77/115/119); the existence of a similar
  external technique is not evidence that replacing ours fixes blurry soil.
- `TerrainGrass/.../grass_surface_pages.glsl` selects the same surface pages,
  unified fallback and footprint semantics as terrain shading. Placement writes
  page-derived root height/normal; the grass vertex stage consumes them. This
  directly exposes Thessa's remaining CPU-source versus resident-page gap.
- `GrassClumpModel.h` derives correlated height, facing and color from a second
  world-anchored jittered lattice. It preserves placement coverage while making
  groups read as plants. Thessa already has world-addressed ecology/cohorts;
  compare the represented attributes rather than add a parallel scatter system.
- `terrain_grass_vertex_modifier.glsl` separates cantilever spine, droop, a
  broad-to-point width profile, view-dependent fold and bounded projected-width
  compensation. V3 uses the broad-to-point profile and a wider independently
  implemented quadratic fan after pointed/rooted v2 left little stiff fans.
- `CBTScreenTarget.h` intentionally scales target pixels relative to 1080 rows
  to keep geometry demand independent of resolution. This differs from a true
  fixed-pixel error target; it must not silently replace Thessa's quality policy.

## Remaining ground acceptance work

1. Retain the shared tip/UV correction and independently sampled crown contact
   from doc 126. Restore broad, natural grass coverage without reducing density
   or changing light/exposure to hide defects. V2 is rejected; v3 is retained
   only as a broader-leaf improvement with the whole-scene target still open.
2. Measure roots against the selected GPU height page and rendered CBT chord,
   including parent fallback, publication changes and camera motion. A test
   comparing `ground_position` against itself cannot prove that contract.
3. Replace the broad untextured moss facets with convincing small-scale
   structure/material coverage. Doubling smooth domes was explicitly rejected.
4. Audit close-soil source bandwidth, material footprint/filtering and geometric
   relief as separate causes. Roughness 0.96 did not repair the visible failure;
   more vegetation is useful composition but does not close material quality.
5. Continue static/dynamic shadow caching and conservative selective correction
   from the exact AMD/Epic mechanisms with dense unchanged caster populations.
   No classifier or caching speedup is claimed by this ground iteration.

The fixed B03 target and externally rendered Cycles frames remain visual
references. An AI artwork guides lighting/material/composition, not numerical
geometry identity. Full scene, moving camera, production timing and blind
preference acceptance remain outstanding.

## Reproducible page-source diagnostic

The diagnostic source is preserved in
[`ground-page-parity-probe-2026-11-03.rs`](artifacts/ground-page-parity-probe-2026-11-03.rs).
It bakes 33×33 pages under the production 0.5 m / 1 mm encoding-error rule,
then bilinearly samples the fixture direction using `HeightPage::sample`. Root
independently reran the probe against the canonical project field. At the
fixture, the CPU optical height is 5.215545383 m; synthetic L7/L17/L18/L22 page
deltas are respectively +7.200775417 m, −57.650772 mm, −32.603947 mm and
+0.365051 mm. They are source-footprint/interpolation evidence, not a claim
that those pages were resident in the actual frame or that quantization alone
causes the difference. The probe lives outside production and adds no runtime
path. Exact input source hashes for inspected engines are in
[`ground-reference-sources-2026-11-03.json`](artifacts/ground-reference-sources-2026-11-03.json).
