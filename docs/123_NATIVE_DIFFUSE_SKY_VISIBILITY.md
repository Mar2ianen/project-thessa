# 123 — Current-frame diffuse sky receivers and contact visibility

Status: opt-in native HDR contact visibility implemented. Full GI/bounce,
off-screen transport, complete material G-buffers and reference parity remain
open. This extends the existing terrain/mesh/MSAA owners without a deferred
renderer replacement or a second geometry prepass.

## Explicit policy and frame state

```toml
[renderer]
ambient_visibility = true
ambient_visibility_quality = "high"
ambient_visibility_radius_m = 2.0
```

The shared requested/resolved pipeline gates the effect on native HDR receiver
support (two RGBA16Float attachments). Defaults remain Off; unsupported hosts,
including the Bevy integration, report an unoccluded-sky fallback. TAA and other
effects are not enabled as prerequisites. Off allocates no receiver or new
pass. Radius is a finite approximation support in metres, clamped to 0.05–20;
nonfinite requests resolve to 2 m. Low/Medium/High use 8/16/32 depth probes:
4 directions × 2 distances, 8 × 2, and 8 × 4 respectively. Quality changes the
quadrature, not source energy or geometry.

Opaque/MASK terrain and mesh shading writes its beauty plus one RGBA16Float
receiver in the same pass. RGB is the **actual pre-exposed reflected diffuse
sky contribution**, with existing material AO, Fresnel/metallic partition and
camera-path haze transmittance. Alpha carries the existing baked/material
visibility, independently of beauty's temporal/coverage alpha. Geometry and
normals for contact visibility are reconstructed from that frame's stored
Depth32Float, using the same camera-relative matrices and primary jitter.

The finite cosine-weighted depth search produces a scalar visibility. Unknown,
off-screen or mixed-depth MSAA samples remain unoccluded. Occlusion has a
quadratic finite-distance falloff. The 2 mm/numerical precision origin bias
avoids self-intersection; it is unrelated to a star or pigment. The existing
baked visibility and screen estimate combine as their minimum, a conservative
upper bound when blocker correlation is unknown, rather than multiplying the
same prototype's occlusion twice. This can under-occlude disjoint blockers;
it does not reconstruct their joint directional visibility.

One subtractive HDR fullscreen pass removes only the lost reflected diffuse
sky contribution. Direct sunlight, specular reflection, emission, air
in-scattering and the opposite-hemisphere leaf transmission are excluded.
Terrain's separate spectral water path is excluded until it exports its own
matching component. Sky and BLEND composition follow this correction; existing
plume/cloud/display stages keep their ownership. No framebuffer colour copy
or stochastic temporal history is required.

## Cost and public integration

The receiver costs 8 bytes/pixel at single sample, or 40 bytes/pixel with the
retained 4× attachment plus resolve: 16,588,944 / 82,944,144 bytes at FHD,
including its 144-byte view/policy uniform. It adds one MRT write and one
fullscreen pass, and separates existing background/BLEND composition. No
extra terrain/plant geometry draw is introduced. Low quality reduces probes;
MSAA/resolution remain their existing explicit user choices. These allocation
and probe counts are not a measured whole-frame speedup.

`TerrainRenderer::encode_opaque_scene_with_ambient_ops` and
`MeshRenderer::encode_opaque_with_ambient` require the optional receiver when
constructed with the effect enabled. Then resolve depth, encode the visibility
correction and compose background/transparency using their retained single
beauty target. The native headless, window/resize and reference-scene hosts
perform the complete sequence. Existing single-target APIs remain the Off
path. Invalid requested/resolved extents/modes fail before allocation.

## Executed controls and limits

A production-renderer box/floor fixture compares a 65,536-direction independent
finite-range ray/box integral against all probe tiers and 1×/4× MSAA. At the
selected contact it reports approximately 0.193–0.196 occlusion against
0.211–0.221 reference, within a pinned 0.06 absolute envelope. This is a scoped
fixture error, not a universal bound for single-layer screen depth. Tests also
check unchanged HDR with export/correction Off, Off/On/Off identity, unchanged
depth, direct/emissive/transmission exclusion, baked-visibility overlap and
resource-free disable. The actual native B05 transparency/emission host also runs Off/On at 640×480,
three measured frames; its zero-sky receiver leaves HDR byte-identical. This
checks host composition, not transparent-object occlusion or glass transport.
Low terrain has a field-free ambient fallback and must
retain depth even when no sky or motion pass otherwise needs it.

Frozen native diagnostics now prepend Off/On/Off beauty controls when this
receiver exists, retain exact HDR receiver bytes across those controls, and
then replay the existing 17 terrain views. Optional GPU timestamps bracket the
visibility pass only; unsupported devices record null. On the bounded FHD B03
32-probe/4× run one enabled interval is about 3.8 ms. The disabled *QA control*
still executes a pass and costs about 1.7–1.8 ms; production Off does not allocate
or execute it. These isolated, fenced intervals exclude MRT/depth/composition
cost and are not a whole-frame speedup or accepted production budget. The receiver remains a partial
component output; the old irradiance diagnostic views are still not an exact
full radiance decomposition. Commands, identities, captures, failed attempts
and executed results are in [the artifact](artifacts/diffuse-sky-visibility-2026-11-03.json).

References: [AMD CACAO](https://gpuopen.com/fidelityfx-cacao/),
[XeGTAO](https://github.com/GameTechDev/XeGTAO). These motivate depth-based local
visibility and its geometry/thickness limitations. This small portable WGSL
implementation is independent; it does not claim to port CACAO/XeGTAO or match
their filtering/convergence. Hidden/thin geometry, depth-edge ambiguity, angular
quadrature, isotropic visibility weighting, moving-camera stability and affordable
full-scene reference acceptance remain open. There is no indirect bounce here.
