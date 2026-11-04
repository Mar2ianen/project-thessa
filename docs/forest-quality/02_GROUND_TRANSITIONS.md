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

## Sequential implementation — parent litter and moss emergence

Reference review: Unreal PCG `PCGCopyPoints.cpp` explicitly combines source/target
transforms and parent/child seeds; `PCGProjectionElement.cpp` projects the derived
spatial data onto the target surface before point realization. The inspected
UE 5.8.4 files and [PCG node reference](https://dev.epicgames.com/documentation/en-us/unreal-engine/procedural-content-generation-framework-node-reference-in-unreal-engine)
provide the parent-linked/projected-detail mechanism. No UE code is copied.
Thessa extends its existing canopy neighborhood and canonical soil projector.

Each canonical tree and represented mature ground fern supplies three stable
candidate dead fronds. Child positions, headings and lengths inherit the parent
pose/crown dimensions and a seeded organ ID. Stable child priority mixes parents
before the Medium/High 8/32-frond cap; water rejection precedes admission.
The earlier procedural 29-piece fishbone was rejected and removed. The current
candidate extracts three connected organs from the pinned CC0
[Poly Haven fern_02](https://polyhaven.com/a/fern_02) scan. Conversion provenance,
source hashes, exact UVs, source normal/roughness maps and the explicit offline
2K diffuse/alpha inputs are recorded beside the GLB. Brown pigment is authored,
not a measured dead-tissue material. RGBA16 preserves source alpha samples in
the converted asset; the runtime decodes to RGBA8. The 2K base image requires
16 MiB decoded versus 4 MiB for the earlier 1K candidate. This is static optical
litter, not a growth/decay or wind simulation.

The new organs have their own cutout draw group and counter, separate from the
existing 512/2048 little litter fragments. Whole organs are admitted within a
2/8 MB Medium/High geometry sub-budget and the unchanged 48 MB total source
mesh guard. No partly admitted frond is published. The small untextured litter
material now explicitly sets metallic=0: its inherited glTF-default metallic=1
was incorrect for organic tissue. The global glTF default is unchanged; see
[the specification](https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html).

Geometric moss emergence now consumes the same canonical `cover.moss` as the
terrain texture. Colony footprint and shoot height shrink smoothly toward the
textured fringe and grow into higher-coverage patches. Sites, 24 shoots per
colony, source cutouts and the 1200/4250 colony caps remain unchanged. Existing
wetness and soil relief continue to own substrate variation; no dampness decal,
extra AO or new water solver is added.

The 10 m view exposed an unrelated but important transition defect: the entire
near cover was disabled above 4 m eye clearance. The existing survey envelope
now retains it through 32 m with unchanged 7×7 near-plant / 3×3 ground windows
and population/source budgets. Scene-routing tests cover 0.3,2,10,32,32.01,50 m.
Higher views still use coarse cover: this bounded policy is not a continuous
planet-wide ecological/grass LOD or a moving-camera transition acceptance.

Tests verify parent/order-stable litter sites, nondegenerate geometry, preserved
UV endpoints, bounded refinement and clearance relative to retained source curl.
They also reject collapse of all shading normals to radial-up. The v12/v13
wide/soil/crown captures published zero dead fronds and cannot validate their
geometry. The integration fixture now explicitly requires nonzero fronds.

The v21 GPU geometry control exercised three forms at 0.4, 2.1 and 6 m; all nine
retained their isolated visible pixels on the analytic reference receiver. This
is not resident-CBT attachment or aesthetic acceptance: the reference grid and
CPU optical field do not reproduce every actual resident terrain triangle.
The old gray fixture used Low quality, without normal-map/specular appearance.
A separate High PBR oblique capture now accompanies the unchanged Low occlusion
control. Visual inspection still rejects edge-on/comb-like forms and black
patches. Source curl is no longer clipped to 15 mm, source normals are transformed
by the local ground deformation instead of being replaced with radial-up, and
intermediate refinement normals are interpolated. These corrections do not by
themselves close the appearance review.

The close-soil surface texture itself is still unresolved and belongs to item 6;
this step must not be reported as fixing the foreground material.

## User visual rejection, 2026-11-04

The user explicitly rejected the v18 macro: comb-like orange leaf elements,
broken-looking rachis, rigid plates and an unconvincing bend. The 90% visibility
guard measures a narrow intersection case and does not establish a convincing
fallen frond. This candidate is not visually accepted and item 2 remains open.
Subsequent work must correct the organ silhouette, physical scale and dry
material, then assess it in the scene against botanical/source references.

### Source-plane correction after v21

Independent asset review found that the second PCA axis followed arch/sheet-normal
variance rather than transverse blade width (57.8° and 65.3° misalignment in two
forms). Scaling that direction as curl squeezed actual width out of the soil
plane. The converter now projects the longitudinal axis into the area-weighted
authored-normal tangent plane and obtains width by their cross product. The
native source regression requires the area-weighted normal to align with local
+Y, so a blade placed on XZ cannot silently become edge-on again. Conversion
metadata records this basis and inverse-transpose normal transformation.

### Normal-map isolation and v24 candidate

The v23b actual-GPU High control changes only the normal-map binding: black
patches disappear with geometric shading normals. The source map had retained
full relief after the organ was compressed to 35% height. The converted glTF now
sets `normalTexture.scale=0.35`, the same compression, as a flattened-sheet
microrelief approximation; this is not an exact deformation rebake. V24 retains
the normal map and removes the conspicuous black patches in the inspected form-0
oblique macro. This is a narrow material correction, not a complete botanical or
scene-quality acceptance. The decoded-material regression pins this scale.

Current checks: native package tests 153 passed / 50 ignored in its primary test
binary (integration targets also pass); the all-form/all-length v22 GPU Low
geometry control passed all nine cases. V24 High material control and full native
FHD integration are recorded separately. Architecture and all-target/all-feature
Clippy were rerun after the final normal-scale asset update and passed. No blind preference, resident-CBT contact
or production performance result is inferred from these debug runs.

The organ deformation preserves source triangle winding. The previous
one-corner shading-normal reorientation was removed: a smooth corner normal is
not an authoritative carrier orientation. Root recomputation on the corrected
source found two corner/face sign disagreements, not a widespread source winding
failure; the normal-map A/B remains the evidence for the visible black patches.

The v24 full FHD debug capture completed with 20 matching frozen diagnostics,
32 published fronds and zero frond sub-budget rejections. Mesh payload is
33,846,912 B and decoded texture payload is 45,707,264 B. The whole scene still
misses the reference in soil, distant cover and light. This capture predates the
small source-winding cleanup and is not an exact final-state A/B. Full provenance
and limitations are in the [candidate evidence](../artifacts/forest-ground-transitions-candidate-2026-11-04.json).

Final v25 source-winding controls reran all nine form/length cases successfully,
with separate High mapped/geometric-normal views. Native tests (153 passed,
50 ignored in the primary binary, integration targets passing) and all-target /
all-feature Clippy passed after that cleanup. No further geometry/material change
was made after these checks. Visual scene acceptance remains open.

## Complete visual review and resting rachis, 2026-11-04

Every artifact of an iteration now requires inspection (AGENTS.md / doc 55),
including the complete variant set and diagnostic buffers; numerical tests do
not substitute for shape/material/contact review. Root inspected all 36 v25
organ controls, 20 frozen v24 scene views, five v26 shadow/contact controls and
15 v27 controls. Defects were also opened at native resolution.

Reference/applicability review: Unreal's [Spline Mesh component](https://dev.epicgames.com/documentation/en-us/unreal-engine/blueprint-spline-mesh-component-property-reference-in-unreal-engine)
separates a mesh's forward axis and cross-section from its longitudinal curve.
This supplies the authored deformation reference, not a dead-leaf physics model.
The source scan already provides pinna folds. Thessa needs to remove its living
canopy rachis arch before resting it on the floor, rather than flattening the
entire organ or implementing another simulation. No UE code is copied.

The High fixture now uses the existing shared raster alpha-cutout caster /
receiver pass and a separate grazing camera. The Low coverage control remains
unchanged. V26 reveals the large unsupported source arch. V27 subtracts a fitted
cubic longitudinal y(z) from the source before its existing 0.35 curl compression.
Normals receive the exact inverse-transpose shear/scale, preserving texture UVs,
source topology and transverse folds. The source geometry regression rejects
both a flat card and return to the living arch. This is a static optical resting
pose, not a simulated flexible body.

V27's normalized height span is 0.053–0.061 AFTER the 0.35 compression. At 2.1 m
that means up to 11.2–12.8 cm of remaining pinna curl; it must not be reported as
4 cm by applying compression twice. Native parent-derived organs are often
shorter. All three new low views lose the long floating arch, with portions
occluded by intervening soil as expected. Uniform dry pigment, grazing dark marks
and reference-scene quality remain open. Luna independently inspected all 15
frames and confirmed the normal transform; its earlier erroneous winding count
was withdrawn after a NumPy broadcast correction. Root verified the actual curl
scale separately. See the [complete review manifest](../artifacts/forest-ground-transitions-visual-review-2026-11-04.json).

## Pigment control and native close-ground check

The pinned Poly Haven source supplies spatial pigment/damage variation as well
as normal/roughness maps. Converting its RGB to one luminance removed all
chromatic variation. The Blender/Cycles [linear-color reference](https://developer.blender.org/docs/features/cycles/units/)
provides the color-processing boundary; this is authored scan recoloring, not a
measured senescence model or copied Blender algorithm. V28 retained all
median-white-balanced source chromaticity and was rejected after inspection of
all 15 controls: it creates conspicuous pink/purple patches. V29 retains only a
10% source chromatic residual around the authored brown target. All 15 controls
were inspected; it stays brown and retains modest local variation. Geometry,
normal/ORM maps and alpha remain unchanged by this pigment ablation.

The v27 native FHD integration and all 20 frozen diagnostics were inspected.
The v29 actual 0.3 m native view was inspected at full resolution. It exposes
large black patches on LIVE leaves and foreground soil mush; those cannot be
claimed repaired by the fallen-organ changes. Item 2 and overall scene visual
acceptance remain open. The next foliage diagnosis must isolate the shared
normal/material response on actual living source geometry.

Final current package checks: 153 primary tests pass / 50 ignored; integration
targets pass; all-target/all-feature Clippy, architecture and diff checks pass.
The v29 three-form material controls pass geometry visibility, with all 15
rendered images reviewed. No production performance or blind preference claim.
