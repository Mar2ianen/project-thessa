# 126 — Patchy ground cover and wet-soil response

Status: implemented in native near-ground preparation and the canonical
worldgen material response. The B03 sunset still has visible bare and muddy
substrate; this is a bounded visual improvement, not reference parity.

## Changes

- High admits at most 4,000 moss colonies and 4,800 grass tufts in the existing
  near-eye ground window. Medium caps these at 1,200 and 1,600. Low and disabled
  ground detail publish no cover geometry.
- Moss colonies use a 10-sided footprint, three height-profile rings, short
  curved shoots, and three olive-green material variants. Ground samples follow
  the same scanned-soil relief source used by pebbles and litter. Their
  continuous pigment mask admits more values from the moss field and biases
  coverage toward centimetre-scale hollows at equal dampness. It retains scan
  contrast and leaves exposed substrate between colonies; High increases the
  bounded population from 3,400 to 4,000 without changing per-colony geometry.
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

Epic's [Landscape Materials](https://dev.epicgames.com/documentation/unreal-engine/landscape-materials-in-unreal-engine)
and [Open World Tools Tips](https://dev.epicgames.com/documentation/unreal-engine/open-world-tools-tips)
show material-layer weights and foliage masks sharing placement semantics,
with height blending used for overlays such as soil beneath grass. Thessa's
existing moisture/canopy mask now also follows local depressions for moss; it
remains a continuous worldgen field, not a port of Unreal's layer system.

## Capture evidence

The fixed B03 sunset camera was rendered at 1920×1080 with 20 seconds of
prestream on Radeon 880M/RADV. Earlier shape/color iterations are preserved at
`outputs/ground-detail-repair-2026-11-03/ground-cover-dense-fhd-20s/` and
`outputs/ground-detail-repair-2026-11-03/ground-cover-green-moss-fhd-20s/`;
the latest mask/population iteration is
`outputs/ground-detail-repair-2026-11-03/ground-cover-moss-dense-fhd-20s/`.
All use the same camera and graphics config. The latest capture reports 4,000
moss colonies, 4,251 grass tufts, 24 pools, 4,946 ms surface-plant/ground
preparation, 45,821,076 bytes of mesh prototype payload, and 16,343,040 bytes
of prototype textures. Mesh payload is 2.18 MB below the existing
48,000,000-byte admission limit; texture bytes are reported separately. The
capture uses serial stage fences with `performance_equivalent=false`, so its
frame timing is diagnostic and not a performance result.

Compared with the matched 3,400-colony frame, the latest image adds 600 small
moss colonies and shifts more cover into shallow depressions. Muddy ground and
scan banding remain visible, and the increased mesh payload narrows the existing
admission headroom. The closest material-page residency issue and full visual
parity remain open. No change was made to CBT terrain topology or material-page
ownership.
