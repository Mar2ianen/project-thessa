# 126 — Patchy ground cover and wet-soil response

Status: implemented native cover geometry and canonical wet-soil response;
reference visual acceptance remains open. The floor still exposes repetitive,
muddy substrate. The latest reviewed numerical slice fixes blade tips and
canonical-surface root contact; it does not establish rendered-page agreement.

## Current implementation

- High admits at most 4,250 moss cushions and 21,200 grass tufts. Medium caps
  these at 1,200 and 1,600. Low and disabled ground detail publish no cover.
- Moss uses 24 short curved shoots per colony in three olive palettes. Each
  root is sampled independently against optical soil after its jittered
  golden-angle offset. A continuous local heading field correlates neighboring
  shoots. Lengths are 26–60 mm with pointed tips; broad smooth dome faces are
  removed. The occupancy mask follows dampness and shallow depressions. This
  is a fine-shoot approximation, not validated botanical moss realism.
- Grass uses fourteen curved, pointed blades per crown and four shape variants
  per palette, twelve reusable prototypes in total. High admits up to four
  independently rooted tufts around each selected site. Their height is sampled
  after the world-fixed offset. A frame from neighboring surface samples aligns
  the tuft to local terrain, and deterministic yaw/scale vary its appearance.
  Grass keeps a broad body through 40% of its curve before tapering to a point;
  a wider sector-jittered fan restores the footprint lost by the shared crown.
  Moisture gates occupancy; moss no longer excludes grass from its patches.
- Blades carry longitudinal UVs from root zero to tip one. Root/tip fans avoid
  blunt caps and zero-area tip triangles for all supported segment counts 1–4.
  Vertex compaction includes UVs, preserving distinct texture coordinates.
- Cover uses existing mesh instances, materials, shadows and publication. There
  is no added pass or image. More reusable instances reduce source mesh bytes
  but can increase culling/submission work; source bytes do not prove speedup.
- Wet porous soil retains roughness 0.88 with existing mineral/litter variation.
  Explicit basin water uses 0.008. High admits 24 pools, Medium 4. Cover/pools
  remain optical scenery without wind, collision, growth or hydrology.

## Implementation references

AMD's [procedural grass research implementation](https://gpuopen.com/learn/mesh_shaders/mesh_shaders-procedural_grass_rendering/)
provides quadratic blade geometry, patches and coverage compensation. Thessa
keeps CPU preparation and the existing portable renderer rather than adopting
its mesh shader backend.

Unreal 5.8.4's actual `Engine/Source/Runtime/Landscape/Private/LandscapeGrass.cpp`
was inspected at lines 2388–2440 and 2588–2665. It obtains placement height from
triangle barycentrics and derives the corresponding triangle normal before
aligning each instance. Thessa independently samples its canonical optical
surface and builds a finite-difference frame. The distinction matters: this is
not yet placement against the currently resident rendered triangle. The public
[FGrassVariety declaration](https://dev.epicgames.com/documentation/en-us/unreal-engine/API/Runtime/Landscape/FGrassVariety)
identifies the related alignment, jitter and variation controls.

[Doc 127](127_REFERENCE_DRIVEN_GROUND_ITERATIONS.md) records the inspected
OpenEngine CBT/grass source, the AMD shadow implementation and the review loop.

## Numerical checks

Reviewer-owned `wet_ground_acceptance.rs` checks pointed centerline tips,
nondegenerate triangles, finite unit normals and root/tip UV semantics for
segment counts 1–4. The near-eye fixture inspects actual transformed grass
roots against the canonical optical sampler, permitting at most 6 mm float and
20 mm burial. It initially reproduced approximately 108 mm float and 122 mm
burial from the broad, rigid multi-tuft prototype. Independently positioned
crowns pass. Existing cover tests also retain full-canopy admission, bounded
counts, source payload, geometry reuse and Low/Off behavior; all five focused
checks passed for v3. V4 adds a moss fixture bounding individual triangle area
to 8 cm² and checking 15–120 mm of occupied height; all six focused checks pass.
This rejects large bare facets without proving convincing moss appearance.

This root fixture is a canonical-sampler consistency check. It does not read
GPU height residency, quantized page interpolation or the CBT triangle chord.
That remaining contract is explicitly tracked in doc 127.

## B03 capture and rejected candidates

All captures below use the fixed B03 sunset camera, 1920×1080, the same graphics
config, simulation time 74566.63966240163 and 20 seconds of prestream on Radeon
880M/RADV. Paths are relative to the task's external outputs directory,
`ground-detail-repair-2026-11-03/`.

| Capture | Moss / grass | Mesh prototype bytes | Outcome |
| --- | ---: | ---: | --- |
| `ground-cover-moss-dense-fhd-20s` | 4,000 / 4,251 | 45,821,076 | Previous compact cover; muddy bare floor remains |
| `ground-cover-grass-moss-balanced-fhd-20s` | 4,250 / 5,300 | 46,630,852 | Distributed unique geometry; density remains source-budget limited |
| `ground-grass-instanced-patches-fhd-20s` | 4,250 / 15,900 | 22,334,580 | Reusable three-tuft patches; broad rigid roots were later found incorrect |
| `ground-grass-moss-varied-fhd-20s` | 4,250 / 15,900 | 22,458,564 | Twelve shape/palette groups; coverage still weak |
| `ground-cover-moss-pair-fhd-20s` | 8,500 / 15,900 | 43,368,564 | Rejected: prominent smooth green polygonal pads |
| `luna-pointed-grounded-v2-fhd-20s` | 4,250 / 21,200 | 22,348,356 | Numerical correction passes; visual coverage rejected as insufficient |
| `luna-broad-cantilever-v3-fhd-20s` | 4,250 / 21,200 | 22,348,356 | Retained as a bounded shape improvement; reference target remains unaccepted |
| `luna-moss-shoots-v4-fhd-20s` | 4,250 / 21,200 | 25,918,356 | Retained: smooth moss facets removed; dense natural coverage remains open |

The last capture uses 16,343,040 bytes of source images separately from mesh
payload, 2,508,416 instance-buffer bytes, 36 mesh draw calls and 5,361 submitted
instances in v2 (5,394 in v3 and v4). All 20 frozen diagnostic views retain identical published geometry,
depth and beauty round-trip within that run. Native log, exact command and dirty
source/binary fingerprints are preserved beside the frame. An initial attempt
with both diagnostic sequence and perf-capture was rejected by the CLI before
rendering and is recorded separately.

These are serial stage-fenced diagnostic captures with
`performance_equivalent=false`; their frame durations are not game FPS.
Independent runs also publish different material-page sets, so they do not
establish exact pixel A/B identity. The root review is openly labeled rather
than presented as blind subjective acceptance. Ground texturing, moss shape,
coverage, temporal stability and affordable full-scene parity remain open.

A roughness-only 0.88→0.96 experiment barely changed the visible soil and was
reverted. It does not support raising roughness again as the cause-level fix.

The v3 root review observed broader, visibly curved leaf bodies and improved
local coverage at unchanged tuft/moss counts. Source byte counts remain the
same; visibility bounds admit a slightly different set of instances. The five
focused tests, strict native all-target Clippy, release build and architecture
gate passed in the root checkout. Rooted crowns, substrate quality and moss
appearance still do not satisfy the reference scene as a whole.

V4 was reviewed against [Lengyel et al. (2001), real-time fur](https://hhoppe.com/proj/fur/)
and the explicit strand/flow-field representation in
[iMoss](https://github.com/ledhieu/imoss/tree/8e3409a77bb370738ce6738fef0c6e892b6cdd1c).
No shell pass, iMoss code, assets, fake lighting or supersampling were imported.
iMoss is a stylized Marimo algae study under CC-BY-NC 4.0, used only for
algorithm/visual inspection. Its strand volume suggests a representation, not
a terrestrial moss botanical oracle. Root review observes that the smooth
green facets are gone, but the small shoots still read as sparse fine clutter.
V4 passed six focused tests, strict all-target native Clippy and a release
build. The 20-view FHD capture preserves frozen geometry/depth/beauty identity.
The additional source geometry is a measured cost, not a performance gain.
