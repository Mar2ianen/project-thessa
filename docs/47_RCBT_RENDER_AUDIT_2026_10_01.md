# 47 — RCBT render audit (2026-10-01)

Status: source repairs, hardware tests and native mesh-stage draw verified;
material-LOD visual acceptance FAILED in the matched coast comparison below.
This is not a claim of whole-renderer parity with `large_cbt`, KSA or KSP2.

## Findings and repairs

- The former diagnostic mesh consumer bypassed the triangle classifier and
  emitted every 33×33 surface at full density without LOD skirts. It now
  consumes the same compact surface/skirt triangle stream as indexed draw.
- A fixed green diffuse material ignored texture residency, ocean reflection,
  scene stars and exposure. Both consumers now evaluate the same vertex helper
  and fragment shader, including ancestor material-page mapping and filtering.
- Separate draw systems could suppress indexed draw even when mesh limits
  rejected submission. One shared draw system now selects a supported consumer;
  incompatible dispatch limits retain indexed draw and the presentation fence.
- Mesh emission uses 32 primitives / 96 vertices per workgroup. The existing
  classifier writes raster arguments at byte 0 and mesh indirect arguments at
  byte 16. Dispatch fans out at 256 groups per row, with partial/empty guards.
- Live coast captures exposed large rectangular material-resolution changes.
  The material source had inherited the geometry leaf's resolution, even when
  the visible colour field required finer sampling. Material demand now refines
  the existing view cover separately according to projected texel footprint,
  without changing height topology. Fragment sampling resolves body-fixed cube
  addresses through a sparse resident-page directory and interpolates adjacent
  requested material levels. It can use finer resident material under a coarse
  mesh leaf. This replaces geometry-ordinal material selection, not the
  canonical field, and preserves sharp nearby detail rather than blurring the
  entire image to hide a seam.
- Static survey captures now ignore interactive camera input; earlier runs could
  be zoomed mid-capture, invalidating same-scene comparisons.

## Settings and fallback

`graphics.toml` exposes `renderer.terrain`:

| Request | Consumer |
| --- | --- |
| `cpu` | Existing streamed CPU mesh path; no mesh-stage work |
| `gpu_indexed` | Portable compute/classification plus indirect raster draw |
| `gpu_auto` | Mesh draw when the enabled device feature and limits suffice; indexed otherwise |
| `gpu_mesh` | Explicit preference for mesh draw; unsupported devices fall back with a resolver note |

The checked-in configuration selects `gpu_auto`; library/schema defaults stay
CPU for conservative callers. Mesh features are negotiated, not unconditionally
required from unsupported devices. The client resolves terrain again after
device initialization, before terrain setup; performance metadata records the
resolved consumer. Oversized covers can still fall back per draw.

These are representation/backend choices, not physics or different material
quality. The existing 512-layer GPU material bound is retained; independent
material demand is capped at 512 pages and CPU residency at 1,024 pages, keeping
the nearest resident source and one ancestor for current demands. The sparse
directory occupies at most 32 KiB at 512 layers. Mesh shaders
do not replace CPU topology planning or streaming with a persistent GPU pool.
Shared GPU counters remain `terrain_triangles`, `terrain_geometry`,
`terrain_classifier` and `terrain` draw timing.

## Acceptance still required

- Hardware GPU tests, native mesh-stage execution, same-scene mesh/indexed
  screenshots and frame/pass timings on the Radeon 880M.
- Surface/coast/highland and orbital captures; camera motion must not expose
  cracks, stale output, tile-shaped colour switches or material loss.
- KSA/KSP2 references must be real gameplay screenshots with provenance; their
  copyrighted images are comparison inputs, not redistributable game assets.

The normative architecture in doc 22 still requires persistent GPU bisector
allocation, neighbor/conformity propagation, bounded exhaustion and whole-system
same-error measurements. Current tile selection, skirts and mesh emission do
not satisfy that complete topology target. Terrain still lacks Bevy PBR shadow
parity, local normal-map detail, volumetric clouds and physical ocean waves.
These gaps preclude a verified “better than KSA/KSP2” claim at this stage.

## Validation so far

An isolated checkout of `16741ab` with renderer changes excludes unrelated,
in-progress thermal/resource changes in the main working tree. On the Radeon
880M / RADV Mesa 26.2.3, 51 RCBT tests (including hardware GPU tests), 75 client
tests and 18 graphics tests pass. Native mesh coast capture completed without
wgpu validation errors and reported `terrain=gpu_mesh` with actual terrain GPU
timing. A first world-addressed material capture removes the prominent
rectangular resolution boundary in the inspected coast frame; full close-up,
motion and same-resolution backend comparisons remain necessary.

Cold/missing material sources still fall back to resident ancestors or the
canonical globe map. Coarse mip borders, source arrival transitions, height
geomorph and conforming topology are not proven solved by this material repair.

## Matched coast delta — 18:48 / 18:50 UTC

The original `174049` mesh and `174252` indexed runs are NOT a valid A/B
benchmark: resolutions differ (1272×1430 versus 2560×1600), the final view
differs (surface versus pilot), and the inspected survey distances differ.
Their timing difference must not be attributed to mesh shaders.

New captures use the same latest shader sources, Radeon 880M, Vulkan/RADV,
2560×1600 fullscreen, VSync off, manual EV100=13, RGBA material storage,
paused local authority, static surface view, COAST site 0, survey distance
50,000 m and default survey pitch. Survey distance is a camera offset, not
a measured altitude. Only `renderer.terrain` changes between runs.

| Debug aggregate | Indexed `184818` | Mesh `185048` |
| --- | ---: | ---: |
| Samples in matched 30 s capture window | 2,211 | 2,123 |
| Frame wall p50 / p95 / p99, ms | 12.227 / 22.775 / 34.903 | 12.911 / 23.829 / 35.290 |
| `render.terrain` GPU median / p95, ms | 2.764 / 3.369 | 3.100 / 3.702 |
| Classified triangles, min / median / max | 294,912 / 294,912 / 294,912 | 294,912 / 294,912 / 294,912 |
| Visible patches, min / median / max | 330 / 330 / 330 | 330 / 330 / 330 |
| Material missing, min / median / p95 / max | 17 / 100 / 230 / 251 | 22 / 109 / 236 / 258 |
| Material jobs, min / median / p95 / max | 0 / 4 / 4 / 4 | 4 / 4 / 4 / 4 |

Quantiles use linear interpolation between sorted samples. Each window begins
at its first captured frame after the existing six-second warmup and ends
30 seconds later. These are aggregate DEBUG measurements with cold per-process
material residency still streaming, not a release benchmark or a steady-state
backend speed claim. GPU timing covers terrain draw, not the complete frame.

Both inspected images retain conspicuous straight-edged material-resolution
changes in the distant coast. Arrival timing differs between runs; pixel
identity is not established. This confirms that the defect is reproducible in
the shared material pipeline, rather than exclusive to mesh emission. The
world-addressed repair and later globe-footprint/face-derivative changes do NOT
yet satisfy visual acceptance. Native draw succeeds without GPU validation
errors; that does not imply acceptable image quality.

`terrain_material_missing` counts visible geometry leaves without an EXACT CPU
material page. It does not measure missing fragment sources, GPU directory
residency, ancestor coverage or independently refined material-demand backlog.
In particular, `missing > 0` with `jobs = 0` is not sufficient evidence of a
wedged scheduler. Long-running four-job occupancy is observed, but page-build
latency and time to usable detail have not been isolated yet.

Artifacts (outside game assets): `/tmp/opencode/rcbt-matched-indexed-coast.png`
and `rcbt-matched-mesh-coast.png`, their matching `.log` files, and
`/tmp/opencode/thessa-render-audit/perf-20261001-{184818,185048}.json`.
The isolated worktree now contains the latest renderer/client source and
retains its deliberate comparison configuration; the main config stays
`gpu_auto`. No unrelated physics/resource files were changed by this delta.

The next stage is canonical planet/recipe conformance and representative
terrain fixtures, alongside shared material-streaming correctness, not further
claims of renderer superiority. Low-altitude/highland, pilot/surface, orbit,
motion, release benchmarks and material-border acceptance remain open.

The latest source sync passes 51 RCBT tests including actual hardware tests,
75 client tests, 18 graphics tests and the RCBT integration test. The first
recipe-conformance repair passes 141 worldgen library tests. Isolated-worktree
`cargo fmt --all -- --check` and scoped all-target/all-feature clippy with
warnings denied pass. Main-tree unrelated thermal/resource work is excluded;
these are not whole-workspace validation or push gates.

A renderer-only source archive, tracked patch and SHA-256 manifest are saved
at `/tmp/opencode/rcbt-checkpoint-20261001/`. The shared status file is excluded
from that archive because its other edits belong to concurrent physics work.
There is no commit or push. The planet refactor's source gaps and staged
acceptance plan are recorded in doc 13; its first repair preserves current
planet appearance rather than claiming the material defect is fixed.

## Real-image references

Retrieved 2026-10-01, stored outside game assets for inspection only:

- KSA: [Sunset on Mars, Gale crater](https://kittenspaceagency.wiki.gg/wiki/File:Sunset_on_Mars.png),
  uploaded by KiwiShark on 2026-08-31. The file page explicitly marks the
  screenshot as copyrighted by the game studio/licensors. The visible comparison
  targets are readable overlapping terrain silhouettes, ground microdetail,
  local rock silhouettes and contact shadows. Mars sunset is not a colour or
  atmosphere-density target for Thessa's different physical environment.
- KSP2: [official Steam screenshot gallery](https://store.steampowered.com/app/954850/Kerbal_Space_Program_2/),
  obtained from `api/appdetails?appids=954850&filters=screenshots`, rather than a
  CGI announcement trailer. Gallery IDs 1, 6, 7 and 12 are useful surface,
  orbital-limb, rover and barren-terrain samples. Some have an early-access
  watermark; exact build/quality settings are unknown. They establish visual
  references, not a matched performance benchmark.

Inspection of these images shows that geometry continuity alone is insufficient:
surface detail, grounded contact shadows, atmosphere/limb integration and object
readability also need acceptance fixtures. This change restores the GPU
consumers' shared material contract; it does not manufacture those missing
systems or silently change physical fields to mimic another planet's palette.
