# 97 — Actual Unreal B05 linear-HDR comparison

Status: implemented and executed on Radeon 880M/RADV with the user's Unreal
5.8.4 Installed Build. Numerical diagnostic only; visual preference, temporal
quality and cross-engine performance acceptance remain open.

## Dataset and capture contract

The adapter imports the **original** B05 GLB from a verified native capture
using stock Interchange: three panels, 12 vertices, six triangles, no lights.
It verifies geometry bounds/counts and derives the Unreal camera from the
captured native matrices, converting metres `(x,y,z)` to centimetres
`100*(x,z,y)`. The actual read-back camera, material parameters, console
variables, engine version and input/output/source hashes accompany the image.
The frozen GLB SHA-256 is
`d21ab7b6501c832fcc471be13d7d09586f2dc22932d7e9b89a9c103f985d5f2f`.

`SceneCapture2D/SCS_SCENE_COLOR_HDR` renders 640×480 RGBA16F; raw readback uses
`normalize=False`. AA, pre-exposure, automatic exposure, bloom, motion blur,
GI, reflections, separate translucency and RT are off. Three capture warmups
precede readback. RGB is compared after applying the native exposure gain
0.41666666 externally. Both images use the shared Reinhard/sRGB conversion;
Unreal's tonemapper is excluded. Alpha is **not** compared: Unreal returns
inverse opacity, whereas native capture alpha has a different contract.

## Import mapping and control experiment

In this Installed Build's tested texture-free stock import path, CPU material
parameters contained the correct `EmissiveStrength` values 2 and 4, but the
rendered output followed the normalized emissive factor. Repeating with only
`update_material_instance` left the same pixels/metrics. This is a finding for
this path, not a claim that Unreal universally lacks extension support.

The adapter therefore reads `emissiveFactor * emissiveStrength` from the original
GLB and puts that literal HDR product into the existing stock `EmissiveFactor`
input, with `EmissiveStrength=1`. It verifies the parameter readback and waits
for material shader completion via the public statistics API. Stock PBR graph,
alpha, geometry and original GLB remain intact. This explicitly declared
adapter parameter mapping is not a fitted correction from native pixels.

| Whole-frame pre-exposed RGB diagnostic | Stock import / update-only control | Literal source mapping |
| --- | ---: | ---: |
| MAE | 0.02671988635 | 0.00001634569037 |
| RMSE | 0.07826475849 | 0.00003961954350 |
| Relative L2 against native | 0.54656854 | 0.00027668642 |

The mapped Unreal overlap is `[0.537109375,1.068359375,0.12493896484375]`
before exposure, versus analytic `[0.5375,1.06875,0.125]`. Native applies
exposure before half-float blending; external exposure follows Unreal's
half-float blending, so small quantization differences are expected. The final
CLI run and an independently invoked mapped run have byte-identical raw HDR,
pre-exposed HDR, display PNG and GLB. This does not establish physical plume,
glass, GI or coastal-scene parity.

## Reproduction

Use a fresh copy of `data/reference-scenes/unreal-b05` **outside** the checkout
as the writable project. Obtain native B05 with [doc 95](95_TRANSPARENT_EMISSIVE_SURFACES.md).
The copied project enables stock Python/Interchange/editor scripting, Vulkan
SM6 and RT off. It isolates Python and disables pip installation. No source
engine build, editor window, custom shader or third-party Python module is
needed. Cold stock shader compilation is substantial; retain writable DDC.

The tested inner command, with absolute paths without spaces, is:

```sh
SDL_VIDEODRIVER=dummy SDL_VIDEO_DRIVER=dummy \
  /absolute/UE/Engine/Binaries/Linux/UnrealEditor-Cmd \
  /absolute/project/B05.uproject -unattended -nosplash -nosound \
  -vulkan -RenderOffScreen -AllowCommandletRendering \
  -DDC=InstalledNoZenLocalFallback -LocalDataCachePath=/absolute/DDC \
  -UserDir=/absolute/user-output -run=pythonscript \
  -script="/absolute/checkout/tools/agent-qa/unreal_b05.py /absolute/native-b05 --output /absolute/new-capture"
```

Actual execution wrapped this in `bwrap`: read-only filesystem except project,
DDC and output, private temporary configuration, no display or network, and
only `/dev/dri/renderD128` exposed. Both SDL variable spellings matter with the
host's inherited environment. Standard asynchronous shader compilation must
remain enabled. When using `subprocess`, pass `-script=... ...` as **one argv
element without literal surrounding quotes**; the shell command above removes
its syntactic quotes. Paths with spaces were not exercised.

## Milestone record (2026-10-30)

[Evidence](artifacts/unreal-b05-comparison-2026-10-30.json) retains complete
comparison metadata, stock/publication controls, repeat hashes, command identity,
installation provenance and explicit acceptance limits. Images, logs, immutable
adapter/config snapshots and the SDK stay outside Git.

- Source base `c25bd04` / integrated `00924ed`; native c25 milestone capture reused.
- New development adapter/project template only; no Rust, production renderer,
  simulation, package dependency or Epic source copied into the repository.
- 33 Python tests pass, including camera, bounds and literal emission mapping.
  The copied repository template and actual CLI completed with exit 0.
- No engine frame-time, Metal/WebGPU, Godot render, temporal trajectory, blind
  preference or whole-scene acceptance was measured. Bootstrap/cold-compile
  timings and readback waits are not rendering performance measurements.

The project-specific implementation sequence is tracked in
[doc 98](98_GRAPHICS_PARITY_ROADMAP.md).
