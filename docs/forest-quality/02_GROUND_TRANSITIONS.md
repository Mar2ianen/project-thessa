# 2 — Soil-to-plant transitions

Status: grass transition implemented; soil/moss/roots integration partial.

## Research

OpenEngine `terrain_grass_surface.glsl`, `GrassBladeCanopyRoot` and `GrassBladeCanopy` at revision `bdf5c2710fe37c1c5ddbac7787477daa87524af3` distinguish authored organ pigmentation from an optional terrain-color handover. [AMD procedural grass](https://gpuopen.com/learn/mesh_shaders/mesh_shaders-procedural_grass_rendering/) supplies the curved-blade/patch reference. Existing CC0 moss organs are reviewed separately in doc 131.

## Applicability review

Thessa already roots grass and moss on the canonical optical soil and has litter fragments, senescent fronds, tree bases, wetness and closed puddles. Another ground scatter or painted ambient-occlusion multiplier would duplicate those owners. A pigment gradient alone is not ground-color sampling or contact shadowing.

## Implementation

Four existing grass variants now differ in length, width, bend and radial/fan habit. Juvenile leaves vary within each tuft. A shared 16×64 sRGB texture provides root/body/tip pigment variation through existing UVs, preserving root=0 diagnostics. Fourteen blades, three segments, existing population caps and twelve grass groups remain. This adds 4096 decoded texture bytes. Existing moss and litter are retained.

## Verification and remaining work

Root-clearance, pointed-tip, nondegenerate triangle and shared source-budget tests run. FHD preview is unblinded author inspection. Ground-color handover, dead-frond accumulation around actual parents and geometric moss transition remain open; prioritize a 0.3 m soil view before adding populations. Root clearance against the CPU optical field does not prove clearance against every partially resident rendered triangle.

The executed 10 m view also exposes the sparse/limited near-ground cover beyond
the immediate camera footprint. Far-cover representation and ground transition
remain necessary even after local tuft morphology improves.
