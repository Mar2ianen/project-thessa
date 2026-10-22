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

## WORLD06b — Separate stem and foliage reflectance

Previously the native adapter assigned the same green albedo to a tree's stem,
branch stubs and foliage. `offline::PlantMesh` now has optional `stem` metadata:
the vertex-prefix count and authored sRGB stem reflectance. The producer marks
the existing trunk/branch vertices before emitting the crown; positions, winding,
normals, topology, scatter IDs and world transforms do not change. The lycopsid
stem reflectance is `[0.29, 0.25, 0.17]`, an authored weathered optical prior,
not a measured fossil colour or new structural/material physics.

This is an additive optional field in the existing frozen prototype payload.
Historical bakes without `stem` remain valid single-material prototypes; no
terrain rebake or replacement of existing bundled source data occurs. A marked
triangle must belong entirely to one organ, and stem metadata is range/finite
checked by the existing mesh validation path.

The native adapter publishes a two-texel sRGB palette through the existing
`MeshAsset` base texture and assigns a constant texel-centre UV to each organ.
It does not introduce a plant shader, pass, extra primitive/draw, instance range,
RT object, normal map, or per-frame texture update. Shared raster and secondary
mesh materials read the same texture; alpha remains one, so shadow geometry is
unchanged. Each triangle has identical UVs at all three corners: there is no
cross-organ interpolation or high-frequency texture pattern requiring mipmaps.
Clamp addressing prevents wrapping into the other swatch. Palette quantization
is bounded by half an 8-bit sRGB code value per channel; tests bound the decoded
linear error by 0.0045. Non-stem prototypes retain their exact float albedo.

`[surface_plants].enabled = false` still does no sampler/mesh/palette work. All
existing Low/Medium/High density tiers use the same organ reflectances and the
same fallback behavior. The sole new semantic image is eight bytes for the
lycopsid prototype, independent of instance count; renderer allocation/alignment
is not eight bytes. `native_surface_plants_prototype_texture_bytes` reports the
source image payload separately from geometry and instance buffers. Native
default plants remain off. No throughput, visual preference, temporal stability
or final surface-quality claim follows from this material correction.

Matched silent captures at 2 m / 50 m / 800 m eye AGL are under
`target/surface-showcase/organ-materials/after-{standing2,close50,overview}.png`,
with JSON/CSV/logs beside them. They retain 222 descriptors, three draws and the
same actual GPU material-level histograms as WORLD06a. The bottom 240 rows of
the standing capture are bit-identical: the material correction does not repair
the ground or change its illumination. The new GPU fixture exercises both
palette swatches under linear filtering at full and 0.05 instance scale; it is
an extension of the retained textured-mesh oracle, not a subjective review.
Unlabeled pairs are prepared in the stage's `review/` directory; votes remain
pending. Paths, hashes, bounds and check results are in
[`native-plant-organ-materials-2026-10-22.json`](artifacts/native-plant-organ-materials-2026-10-22.json).

Stage checks passed: formatting, architecture guard, workspace Clippy, 2,030
workspace lib/bin tests, 103 integration tests and 79 ignored renderer hardware
tests. The mandated Bevy ignored command again selects zero tests; migrated
retained tests ran in the renderer suite. The extended GPU palette oracle passed
within integration tests. These checks do not certify aesthetic preference.

### Remaining ground limitation

A standalone probe linked to the actual release worldgen/flight-authority
libraries queried the attached canonical field, not a new showcase generator.
WORLD00's L17 material page has nominal 0.390625 m texels, yet its RGB code ranges
are only `[30,32]`, `[56,59]`, `[34,36]` over the page; standard deviations are
0.545 / 0.914 / 0.647. This is source-material uniformity, not evidence that more
GPU residency or more texture resolution alone will produce convincing ground.
The next optical work must distinguish resolved forest floor from the current
canopy/ground/reed averaged cover while preserving climate, snow, wetland and
canonical height semantics. No ground change is claimed by WORLD06b.
