# 90 — Deterministic reference-scene data

Status: B01/B05 data and native/Cycles captures implemented; B02 asset ingestion implemented, native B02 pending.
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
The original data milestone did not claim native rendering/timing. The native
continuation below now captures B01; texture/normal-map and HDRI variants,
external-renderer equivalence and native B02 remain separate work.
The pinned B02 external-cache producer and declared conversion losses are
recorded in [doc 91](91_B02_ASSET_PROVENANCE.md).
The first external same-GLB Cycles diagnostic and its explicit comparison limits
are now recorded in [doc 92](92_EXTERNAL_RENDERER_COMPARISON.md); renderer parity
remains unaccepted.
[Doc 93](93_STABLE_DIRECT_GGX.md) records the first resulting production shader
correction: stable direct-GGX peaks, an analytic regression and a new native
same-data capture against the fixed external HDR reference.

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

## Native B01 capture

```sh
python3 tools/agent-qa/capture.py data/reference-scenes/b01-capture.toml \
  --output /tmp/b01-capture-new --profile dev
```

The dedicated `b01-graphics.toml` fixes High surface quality, manual EV100 12,
HDR/Reinhard, one coverage sample and enabled specular response. RT, shadows,
temporal reconstruction, automatic exposure and bloom are off. The baseline
validator rejects hidden workload changes and disabled specular response.
In the existing shader, `specular_reflections=false` also disables direct
stellar GGX, so that setting is unsuitable for a metallic-roughness lab.

`--reference-scene b01` dispatches before loading the canonical game Scene.
Only capture/configuration/resolution/frame flags are accepted. Headless PNG
and perf paths are required; scene sidecars must be new, and image/perf/CSV/HDR/
sidecar paths must be distinct. The app still links its normal game dependencies,
but this execution path constructs no planet, ephemerides, streaming or flight
runtime. The library remains neutral and GPU-free.

The native adapter decodes the **same exported GLB** using the existing importer,
then uses existing MeshRenderer, display transform, timestamp intervals and
PerfCollector. Camera/light/platform formatting and GPU interval conversion
are shared with canonical captures. Mesh pass/clear/display timings have their
own `reference-mesh-v1` layout; these are not equivalent to canonical terrain
scope names. Unsupported effects are explicitly listed in the record.

The tool binds `scene.json` identity to the manifest and actual native scenario,
and records `reference_scene_manifest_sha256` plus `reference_geometry_sha256`.
The existing `scene_manifest_sha256` remains the requested run-manifest hash.
GLB, scene JSON and MIT notice are included alongside images/perf outputs.
Both requested and resolved graphics remain in the existing metadata pipeline.

### B01 native milestone record (2026-10-29)

- **Source commit:** `27984f4` (B01 library; integrated development tree `b4f8365`).
- **Resulting commit:** the commit introducing this section.
- **Architectural change:** independent reference execution path over existing
  renderer/perf consumers; shared capture formatting takes neutral frame/light
  inputs instead of requiring game Scene. No renderer or physics rewrite.
- **Affected dependencies:** native app → reference-scenes and existing
  serde_json; no new external package/version.
- **Changed files:** native reference adapter, CLI/perf helpers, app manifest/lock,
  B01 TOML profiles, capture identity/tests and synchronized docs.
- **Tests executed:** 135 native tests passed, 43 hardware/diagnostic tests ignored
  by default; explicit baseline/alias guards; strict native all-target Clippy;
  feature-off compile; 21 Python quality tests; real graph/format/diff gates.
  Two actual 640×480, 30-warmup/60-measured-frame B01 captures on Radeon 880M
  passed with byte-identical PNG, RGBA16F, GLB and scene JSON. Existing canonical
  128×96 software-Vulkan capture also passed after perf-helper extraction.
- **Tests not executed:** moving-camera or temporal quality acceptance, HDRI/
  texture/normal-map variants, external engine adapters or blind preference.
- **Numerical evidence:** every HDR component finite (RGB range 0..85.0625);
  generated GLB hash equals the independent CPU export. Geometry SHA-256:
  `d5777b784049330a639b7cb81b96a953cfe12e582f1d781f4f027577283feaf1`;
  scene definition SHA-256:
  `cc209032c99ebb4987f49085556754552c8ec262aa5245459722cfdf2b27b08f`.
- **Visual evidence:** actual material-grid captures, static exact repeat only;
  no independent BRDF/renderer equivalence or subjective acceptance inferred.
- **GPU/CPU measurements:** initial full-PBR dev capture: median three-interval
  GPU sum 0.3503 ms; CPU through submit 0.1140 ms, 60 samples. Radeon 880M
  (RADV STRIX1), Vulkan/radv Mesa 26.2.5-arch1.1. Fixed workload: 22,082 candidate
  and submitted triangles, 11 logical/API draws. This is a small direct-light
  lab, **not** a coastal renderer speedup or release frame-time budget.
- **Imported assets/licenses:** none; generated geometry MIT, notice exported.
- **Known limitations:** measurements above use an explicitly fingerprinted
  dirty dev build; no general driver stability claim. The first diagnostic
  incorrectly disabled specular response and is retained as a rejected
  diffuse-only configuration, not accepted B01 PBR evidence.
- **Next smallest useful change:** individually audit/fetch bounded Fern02/Moss01
  payloads for B02; preserve alpha and material semantics.

## B05 transparency and emission

[Doc 95](95_TRANSPARENT_EMISSIVE_SURFACES.md) defines the additional controlled
BLEND/emission scene and its native/Cycles evidence. It uses the same generated
GLB/JSON, capture provenance and existing renderer consumers. The native profile
is `data/reference-scenes/b05-capture.toml`; the external adapter accepts both
B01 and B05. B05 is an alpha-coverage diagnostic, with no glass refraction or GI.

An additional official Khronos EmissiveStrengthTest original-byte cache and
opt-in GPU oracle are recorded in [doc 95](95_TRANSPARENT_EMISSIVE_SURFACES.md#original-khronos-asset-continuation).
Its individual license is CC-BY-4.0 (AGI/Ed Mackey), not CC0. It preserves all
90 original triangles; this is not yet a native B06 capture scene.
