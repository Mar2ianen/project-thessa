# 95 — Transparent and emissive surfaces

Status: explicit glTF BLEND and HDR emission implemented; B05 native/Cycles
captures executed. Refraction, transparent shadows/RT coverage and GI remain open.

## Material and composition contract

This extends the existing neutral material, bounded GLB importer and mesh
consumer. It does not introduce a second renderer. The references are
[glTF alpha coverage](https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html#alpha-coverage),
[KHR_materials_emissive_strength](https://github.com/KhronosGroup/glTF/blob/main/extensions/2.0/Khronos/KHR_materials_emissive_strength/README.md)
and [Godot's standard transparency modes and limits](https://docs.godotengine.org/en/4.7/tutorials/3d/standard_material_3d.html#transparency).
Actual comparison uses the stock Blender glTF importer and Cycles, not a
reimplementation of Thessa's shader inside the reference renderer.

`decode_glb_surfaces` explicitly accepts OPAQUE/MASK/BLEND, emissive factors and
emissive strength. Strict `decode_glb`, rest-pose and cutout APIs retain their
previous subsets. Unsupported extensions, emissive textures and transmission
fail rather than being silently dropped. Linear emission is factor × strength,
finite and nonnegative, before exposure; values above one remain HDR. Its
glTF RGB luminance convention is cd/m². Absolute spectral calibration is unverified.

BLEND coverage is base-color texture alpha × factor alpha. The shader outputs
premultiplied **all outgoing radiance**, including emission; the fixed-function
operation is `source + destination × (1-alpha)`. Alpha zero therefore contributes
no light under this glTF coverage model. It is not an additive-particle material.
Opaque alpha remains ignored; MASK keeps its early coverage rejection.

Opaque/MASK draws precede BLEND. The existing visibility and frame cache prepare
a stable back-to-front queue of visible primitive instances. BLEND depth-tests
against opaque geometry without writing depth. Single- and double-sided
variants retain authored sidedness. Multiple consumers can use
`MeshRenderer::draw_scene_into_pass` to merge their queues; separate opaque and
transparent entry points let the terrain host compose its sky between them.
Shortcut: center sorting cannot resolve intersecting/self-overlapping surfaces;
upgrade only with an independently validated transparency representation.

Low/Medium/High retain the same coverage/emission semantics and existing lighting
quality differences. Assets without BLEND create no blended pipeline or queue
entries. Per visible transparent primitive instance there is one sorted draw,
reported by existing work counters. The primary material record grows from 64
to 80 bytes. RT retains its 64-byte record by using previously unused optical
words for emission. Opaque ray hits add emission independently of incident light,
reflectance and AO, before continuing the existing transport chain.

BLEND temporal history and MASK/BLEND shadow-atlas construction explicitly fail.
Nonopaque ray planning reports an unavailable-coverage reason and uses the
existing raster fallback. No transparent ray rectangles, refraction, volume
absorption, area-light GI or emissive-texture support is claimed. Low-level
hosts omitting resolved settings must keep temporal history disabled themselves.

## B05 and actual external comparison

`B05-transparency-emission-v1` exports three generated MIT panels: an opaque
background, a far red emitter (alpha 0.25) and a near green emitter (alpha 0.5).
Primitive order is deliberately unsorted. The dataset has 12 vertices, six
triangles, a fixed camera at `[0,0,6]`, 50° vertical FOV, black environment and
zero incident light. Unexposed overlap RGB is `[0.5375,1.06875,0.125]`.

```sh
python3 tools/agent-qa/capture.py data/reference-scenes/b05-capture.toml \
  --output /absolute/new-native-b05 --profile dev
blender --background --factory-startup --disable-autoexec --python-exit-code 1 \
  --threads 2 --python tools/agent-qa/cycles_b01.py -- \
  /absolute/new-native-b05 --output /absolute/new-cycles-b05 --samples 1024
```

Use `--allow-dirty` only for an explicitly recorded development capture. The
native profile fixes HDR, EV100=1, Reinhard, High, specular on, MSAA=1 and
shadows/RT/TAA/bloom/auto exposure off. Cycles imports the exact GLB and captured
camera, with eight transparent bounces, one surface bounce, seed zero, CPU,
no denoising/adaptive sampling and the common display conversion. This is
emission-only; the B01 numeric sun-unit mapping does not affect B05.

Actual 640×480 Radeon 880M/RADV and Blender 5.2.2 LTS captures passed.
For interior ROI x=[295,345), y=[180,300), Cycles mean differs from analytic RGB
by at most 0.00157%. Native differs by at most 1.67 target half-float ULPs after
intermediate blend quantization. Whole-frame HDR MAE=0.0004569164 and
RMSE=0.0067536433 include stochastic alpha and silhouette reconstruction;
these are diagnostics, not physical/visual parity acceptance.

Native PNG/HDR/GLB/JSON repeat exactly. Existing opaque B01 PNG/HDR/GLB/JSON are
byte-identical to the previous `ef528c0` milestone. CPU offline and GPU raster
durations are not a cross-engine performance comparison.

## Verification and milestone record

[Evidence](artifacts/transparent-emission-2026-10-29.json) retains source/tool,
scene and artifact hashes, actual metadata, ROI values, work/timing scope and
check logs. The captured dirty patch and untracked source files are retained
outside Git and match the recorded source fingerprint.

- **Source:** `ef528c0`, integrated development tree `efa34df`; resulting commit
  is the commit introducing this document.
- **Architecture/dependencies:** existing core/assets/wgpu/native/reference
  consumers; enable emissive-strength support in the existing glTF dependency.
  No new package, scene framework, device or renderer.
- **Checks executed:** 48 render-core, three reference-scene and 40 render-wgpu
  unit tests; seven GLB tests; two native reference validators; 28 Python tests;
  strict affected all-target Clippy; architecture gate (44 packages/20 foundations).
  Radeon: MASK fixture, two BLEND/HDR fixtures and 28 ordinary RT fixtures.
  llvmpipe: MASK and both BLEND/HDR fixtures. GPU cases cover input order,
  global consumer order, moved/culled/removed instances, static cache reuse,
  HDR above one, temporal rejection and opaque emission in reflections.
- **Not executed:** full workspace/coastal replay, Metal/WebGPU hardware, blind
  votes, native B02 density tiers or transparent temporal/shadow/RT acceptance.
  Existing expensive hardware diagnostics remain ignored.
- **Assets/licenses:** generated B05 geometry is MIT; no third-party scene or
  shader code is introduced. Blender stays an external development executable.
- **Performance:** six-triangle diagnostic only; no coastal speedup claim.
  Next work is bounded vegetation tiers and independently verified transmission.
