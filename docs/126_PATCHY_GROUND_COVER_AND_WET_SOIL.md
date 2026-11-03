# 126 — Patchy ground cover and wet-soil response

Status: implemented in native near-ground preparation and the canonical
worldgen material response. The B03 sunset still has visible bare and muddy
substrate; this is a bounded visual improvement, not reference parity.

## Changes

- High admits at most 3,400 moss colonies and 4,800 grass tufts in the existing
  near-eye ground window. Medium caps these at 1,200 and 1,600. Low and disabled
  ground detail publish no cover geometry.
- Moss colonies use a 10-sided footprint, three height-profile rings, short
  curved shoots, and three dark olive material variants. Ground samples follow
  the same scanned-soil relief source used by pebbles and litter.
- Grass tufts use 14 curved, tapered blades with deterministic per-world
  variation in direction, length, width, and bend. Moss and grass are emitted
  into the current aggregate mesh/material ranges; no per-plant draw or new
  rendering pass is added.
- Wet porous soil uses a roughness prior of 0.88, modulated by fine mineral
  variation and existing moss/litter material response. Explicit basin water
  remains separate and uses roughness 0.008. High admits at most 24 pools;
  Medium admits 4.
- The cover and pool populations are static optical scenery. This does not add
  wind, collision, growth, soil displacement, or hydrology.

## External renderer references

AMD's [procedural grass rendering article](https://gpuopen.com/learn/mesh_shaders/mesh_shaders-procedural_grass_rendering/)
describes quadratic Bézier blade silhouettes, patch formation, and reducing
far geometry while compensating apparent coverage. The implementation adopts
the bent, tapered blade construction and patch grouping, but remains on the
existing CPU-built native mesh path; it does not port the AMD mesh shader.

Epic's [Unreal material input documentation](https://dev.epicgames.com/documentation/en-us/unreal-engine/material-inputs-in-unreal-engine)
defines roughness as the spread of reflected light, from mirror-like at zero to
diffuse/matte at one, and notes that roughness can vary spatially. This informed
the separation between porous wet soil (rough) and discrete basin water
(smooth), rather than applying one polished response to the whole wet floor.

## Capture evidence

The fixed B03 sunset camera was rendered at 1920×1080 with 20 seconds of
prestream on Radeon 880M/RADV. The density iteration is preserved at
`outputs/ground-detail-repair-2026-11-03/ground-cover-dense-fhd-20s/`; the same
camera/config and earlier shape/color iterations are adjacent. It reports
3,400 moss colonies, 4,295 grass tufts, 24 pools, 4,569 ms
surface-plant/ground preparation, 43,135,668 bytes of mesh prototype payload,
and 16,343,040 bytes of prototype textures. Mesh payload remains 4.86 MB below
the existing 48,000,000-byte admission limit; texture bytes are reported
separately. These individual captures do not establish steady-state FPS or a
general performance result.

The final image has more low ground cover and rounded, smaller moss groups than
the prior iteration. Some muddy ground remains exposed and still has visible
banding/scan character; the grass cap is not saturated because the damp-habitat
mask rejects some candidates. The material-page residency issue at the closest
footprint and full visual parity remain open. No change was made to the CBT
terrain topology or material-page ownership.
