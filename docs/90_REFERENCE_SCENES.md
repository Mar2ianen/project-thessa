# 90 — Deterministic reference-scene data

Status: B01 data/GLB library implemented; native benchmark adapter and B02 staged.
Capture infrastructure is operational ([doc 89](89_REFERENCE_CAPTURE.md)).

## Ownership

`thessa-reference-scenes` is MIT and depends only on neutral `render-core`,
`glam` and existing `serde_json`. No worldgen, sim-core, celestial, GPU, Bevy or
window/runtime handles. Its dev dependency on `render-assets` checks the actual
import contract. This is the 44th workspace package; the dated 43-package audit
in [doc 88](88_ENGINE_BOUNDARIES_AND_REFERENCE_SCENES.md) remains the R1 baseline.
Architecture guards include this new neutral foundation.

## B01 material lab v1

`MaterialLab` constructs one frozen dataset and its portable geometry together.
There are 10 generated unit spheres plus a plane, 12,254 vertices and 22,082
triangles. Sphere tessellation: 48 slices × 24 stacks; duplicate seam vertices
share exact positions/normals, and degenerate pole triangles are omitted.
All geometry is baked into right-handed Y-up metre coordinates; no simulation
clock, origin rebasing or asynchronous residency influences construction.

- Metallic rows: 0 and 1. Roughness columns: 0.05, 0.2, 0.4, 0.7 and 1.
- Sphere base color: linear RGBA `[0.8, 0.4, 0.12, 1]`.
- Plane: linear 18% grey, metallic 0, roughness 0.8; dielectric F0 0.04.
- Fixed camera: eye `[0, 10, 18]` m, target `[0, 0.7, 0]` m; 50° vertical FOV,
  near/far 0.1/100 m. Projection aspect is the caller's capture extent.
- One white directional light: normalized `[-0.5, 1, 0.75]`, RGB illuminance
  `[10000, 10000, 10000]` lux. No environment/ambient illumination, black background.
- Fixed epoch 0 seconds. Exposure remains caller-owned through resolved graphics.

## Portable export

```sh
cargo run --offline -p thessa-reference-scenes --example export_b01 -- /tmp/b01-new
```

The destination must be new. Outputs: `scene.glb`, explicit `scene.json`
camera/light/unit/generator metadata and `scene.LICENSE.txt`. No third-party
asset, texture/HDRI, conversion tool or large download is needed. Generated
mathematical geometry is distributed under MIT with its notice.

The private encoder handles only this known opaque untextured dataset, using
[glTF 2.0](https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html): interleaved
position/normal/UV, uint32 triangle indices and explicit metallic-roughness
materials. It is not an arbitrary asset converter. No material extensions,
alpha modes, animation, skinning or externally imported content are dropped.
JSON object ordering is canonicalized; same-build exports are repeatable.
Platform transcendental rounding is not assumed byte-identical: captures must
retain the actual exported geometry hash rather than infer it from the name.

## Validation and acceptance boundary

Two CPU tests check finite valid geometry, all 22,082 outward nonzero faces,
unit normals, material-grid values, invalid camera inputs, repeatable GLB and
round-trip through the real bounded GLB decoder. Position/UV/index/material
bindings remain exact; existing decoder unit-normal rounding is bounded by
2e-7. Its reverse root traversal and appended unused default material are
accounted for without changing decoder behavior.
Strict all-target Clippy, formatting and the real 44-package architecture gate
pass. The export example executed and produced actual GLB/JSON/license files.
No native B01 render, GPU timing, texture-linearization/normal-map variant,
HDRI comparison or reference-renderer equivalence is claimed by this data milestone.
The first native adapter must consume the shared dataset through existing
MeshRenderer/display/perf capture, without constructing a canonical game Scene.
B02 asset ingestion follows only after that capture is verified.

## B01 data milestone record

- **Source commit:** `271e56a` (capture infrastructure; clean capture repeated at this commit).
- **Resulting commit:** the commit introducing this document.
- **Architectural change:** small neutral scene-data library and GPU-free GLB
  export in the existing workspace; native integration remains separate.
- **Dependencies:** existing render-core/glam/serde_json; dev render-assets.
  No new external package/version; no game/backend dependency.
- **Changed files:** new crate/example/license, workspace manifest/lock,
  guard classification, licensing/docs/status indexes.
- **Tests executed:** 2 CPU geometry/round-trip tests; strict all-target Clippy;
  actual export example; architecture gate (44 packages, 20 foundations),
  Python quality tests, formatting/diff checks.
- **Tests not executed:** native B01 rendering, hardware timing or external adapters.
- **Numerical evidence:** geometry/material/normal bounds above; no physics or BRDF changes.
- **Visual evidence / GPU/CPU measurements:** none for B01 yet.
- **Imported assets/licenses:** none; generated primitives MIT, notice included.
- **Known limitations:** this initial variant is untextured and direct-light only.
- **Next smallest useful change:** native B01 capture adapter using existing renderer/perf.
