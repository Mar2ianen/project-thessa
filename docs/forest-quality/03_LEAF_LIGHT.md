# 3 — Light within foliage

Status: source and applicability reviewed; no new transport implementation in this iteration.

## Research

Actual UE 5.8.4 `Engine/Shaders/Private/ShadingModels.ush`, `TwoSidedBxDF`, uses separate backlighting transmission, wrapped diffuse and a directional scattering lobe. This proprietary source is inspected as a reference; no code is copied. It is not equivalent to uniformly increasing ambient brightness.

## Applicability review

Thessa `mesh.wgsl` already separates reflected/transmitted sunlight, queries visibility, attenuates solar irradiance through atmosphere, and mixes front/back sky irradiance via material transmission. Backface normal maps are corrected (doc 112). Any adjustment must preserve the shared light transport and reflection/transmission energy split. Existing ambient visibility must not become an extra artistic AO layer.

## Implementation decision

Keep the current transport pending a measured backlit specimen. Cattail dry spikes explicitly have zero diffuse transmission and no inappropriate leaf-vein normal map; living leaf transport remains. No new leaf glow or exposure coefficient is introduced.

## Verification needed

Freeze geometry/time/exposure and compare direct, diffuse sky and beauty for leaf front/back, sun behind/in front and transmission off/on. Validate normals and optical source values before changing the scattering model. Current Lambertian transmission lacks Unreal's directional thin-leaf lobe; reference parity remains open.
