# 137 — Ground source fidelity: native litter and independent roughness

Status: bounded source-fidelity/bandwidth improvement verified, 2026-11-04;
baseline `d26efd5`. Broad close-soil smear and forest reference parity remain open.

## Research and applicability review

[Filament's full PBR sample](https://github.com/google/filament/blob/e079be78521a8da7542ebedfb86d279deaa98132/samples/sample_full_pbr.cpp#L260)
reads base color and roughness from separate source maps (lines 260 and 278).
[Unreal's PBR material inputs](https://dev.epicgames.com/documentation/en-us/unreal-engine/physically-based-materials-in-unreal-engine)
likewise expose distinct material inputs. Thessa already imports matched roughness
maps, but its soil pigment transfer multiplied roughness by a luminance-derived
`1 - 0.35 * mineral` factor. That overrides the supplied surface channel without
a separate material classification. This step preserves the scan rather than
replacing it with a new constant roughness. No engine code is copied.

A second source bottleneck remained after doc 135: the pinned CC0
[Brown Mud Leaves 01](https://polyhaven.com/a/brown_mud_leaves_01) source is 1K,
but the importer reduced all four planes to 512. The existing fine-footprint
soil/litter mixture assigns litter a weight from 0.1 to 0.85. At its unchanged
1.3 m repeat, this layer has a 2.539 mm source pitch, coarser than High's 1.526 mm
L25 page. Native 1K restores a 1.270 mm source pitch. Soil/moss source dimensions
and the authored height ranges/physical repeats are unchanged.

The sampler was investigated first. The independent L24 periodic magnification
oracle now exercises both isotropic and supported 16x anisotropic filtering:
216 channel/phase comparisons at two physical footprints and two cube faces
stay within one RGB8 code of CPU bilinear sampling. Unsupported adapters retain
the isotropic control and explicitly skip the anisotropic case. No production
sampler bypass, negative mip bias or sharpening is justified by this result.
The [Vulkan sampling specification](https://vulkan.lunarg.com/doc/view/1.4.350.0/linux/antora/spec/latest/chapters/textures.html)
allows implementation-dependent anisotropic schemes; this measured result is
scoped to the local RADV adapter and these magnification footprints.

## Implementation

- Soil pigment grading retains its existing RGB response but no longer changes
  the independent source roughness. The exact soil coarse mean becomes
  0.90076196, replacing the modified 0.7873502. Existing wetness, moss and litter
  response still compose afterward; a selected composed page changes only
  0.802328 to 0.803387 in mean roughness. This is not a global matte override.
- The existing FG01/GN01 importers retain native 1024-square litter RGB, roughness,
  R16 height and OpenGL-normal lattices with the same pinned source checksums.
  Runtime linear-light/gradient mips still own filtering. Source means and asset
  provenance/license descriptions match the new payloads.
- The existing ground-page probe admits L25, matching the current High ceiling.
  No shader, binding, pass, GPU texture/cache budget, page format, topology,
  graphics setting, exposure, source scale or scatter population changes.

The existing ground-detail/normal quality settings and disabled lazy-decode
behavior remain. Large CBT owns terrain topology; source lattice dimensions
are independent of geometric subdivision.

## Verification and separate visual decision

The new source regression fails on the baseline: one roughness texel becomes
0.7229264 instead of its source 0.88235295. After the correction, every soil
mip-0 roughness sample exactly matches the decoded source channel; changing RGB
brightness cannot modify a supplied roughness. Existing strict source-mean,
mip, continuity, cover, normal-residual and adjacent-page tests remain intact.
All 306 worldgen tests pass (1 ignored), along with strict all-target/all-feature
worldgen Clippy, render-material Clippy, formatting, native build and architecture.
The final equivalent-f32 literal normalization retains the exact encoded values;
the strict mean guard was rerun afterward.

The exact fixed L24 roughness-only page keeps RGB, optical normals, base normals
and water byte-identical. In a separate 512/native-1K L25 pair, both pages use
face 1, x=12325312, y=24347772, at 1.526 mm pitch. Native albedo and normal
images retain smaller leaves/stems and finer slope structure. Water and canonical
base-normal planes remain byte-identical. The L24 pair is a minification control;
its average colors change only slightly. An independent decode verifies all
four packed native planes against the pinned source PNGs, including normalized
R16-to-R8 roughness and R16 height preservation.

Root inspected every emitted page/control image and both FHD captures' complete
21-view diagnostic sets, native beauties and the final native albedo defect.
Independent Luna review inspected the same source decision and native views:
retain the bounded channel/bandwidth correction; no new blocking artifact found.
The full scene still has broadly smeared/striated soil, weak material integration
and unresolved close foliage/contact limitations. These changes do not establish
soil acceptance. The separate timed scene captures have different residency and
are not an exact whole-frame A/B or blind preference test. Each diagnostic run
preserves its own frozen geometry, depth and beauty round trip. TAA is off.

The [evidence manifest](artifacts/ground-source-fidelity-2026-11-04.json) records
source/log/image identities, exact page controls, primary references and review.

## Cost and remaining work

The litter 5-channel f32 source pyramid adds 20 MiB and its 2-channel normal
residual pyramid adds 8 MiB: **28 MiB additional retained CPU scalar data**,
excluding allocator overhead and temporary decoding. Packed companions grow
from 2,151,183 to 8,771,365 bytes (+6,620,182 bytes). GPU allocations and sampling
counts are unchanged. Debug captures use serial GPU fences and explicitly have
`performance_equivalent=false`; no release timing or speedup is claimed.

This closes the retained source-plane override and avoidable 512-source ceiling.
It does not close foreground blur, source physical calibration, temporal/aliasing
acceptance or full-scene quality. Further work must isolate the remaining
large-scale scan/mask/relief appearance instead of repeating a flat roughness
experiment or assuming an unmeasured sampler blur.
