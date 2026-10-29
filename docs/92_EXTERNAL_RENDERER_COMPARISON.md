# 92 — External renderer setup and first B01 Cycles diagnostic

Status: Godot downloaded/verified; B01/B05 Cycles comparisons executed; Unreal Installed Build unpacked, probe status recorded below.

The initial R1–R4 refactor deferred external adapters. After those milestones
and B01/native capture, the user explicitly requested external-engine setup and
comparison. This continuation adds one development-only Cycles adapter over
the existing portable B01 export. No production renderer or Rust dependencies
change. B02 MASK rendering and vegetation tiers remain pending.

## Tool setup (2026-10-29)

| Tool | Verified state | Source |
| --- | --- | --- |
| Blender/Cycles 5.2.2 LTS | Existing `/usr/bin/blender`, build `d13f752e3b9c`; actual CPU render passed | Local distribution build; binary hash recorded per comparison |
| Godot 4.7.2 stable | Official Linux x86_64 ZIP downloaded, size/SHA-256 verified; extracted binary reports `4.7.2.stable.official.ed1daf0bf` | [Official download](https://godotengine.org/download/linux/), [pinned release](https://github.com/godotengine/godot/releases/tag/4.7.2-stable) |
| Unreal Engine | User browser ZIP download complete; extraction completed after size/path/disk checks; probe status below | [Epic Linux download](https://www.unrealengine.com/en-US/linux), [Installed Build instructions](https://dev.epicgames.com/documentation/unreal-engine/linux-development-quickstart-for-unreal-engine) |

Godot ZIP: `Godot_v4.7.2-stable_linux.x86_64.zip`, 77,860,424 bytes,
SHA-256 `cadd3204e728a35d3f13adb7fd0d7902636b79f6b95c40c265eb73b6c35329e4`.
The extracted binary SHA-256 is
`8d106cbe6144c2dc7e881d61d2429c1a8a76e6b22ef48bd5e48dcf934953f71e`.
The local external tool cache is `work/reference-engines/godot-4.7.2` beside
the checkout; download/installation provenance stays there. No .NET package,
export templates or source build are required for the intended native scene
capture. Godot rendering itself has not yet been validated.

Unreal needs a precompiled Linux **Installed Build** with editor/import support,
not a source checkout. Epic requires an authenticated account to download it;
the user completed that browser download. The agent did not read browser cookies,
accept account terms or use unofficial mirrors. Archive size and unpacked size
must be checked against remaining disk space before extraction. No Unreal
capture or adapter is claimed here; the later headless bootstrap probe is
recorded below.

## Cycles B01 adapter

First obtain a verified native B01 capture using [doc 90](90_REFERENCE_SCENES.md).
Then use the same exported dataset:

```sh
blender --background --factory-startup --disable-autoexec --python-exit-code 1 \
  --threads 2 --python tools/agent-qa/cycles_b01.py -- \
  /absolute/native-b01-capture --output /absolute/new-cycles-output --samples 64
```

`cycles_b01.py` verifies the native capture's source label, artifact hashes,
scene/geometry identities, actual light metadata and frozen HDR/Reinhard
baseline before importing its exact `scene.glb`. Unexpected material-lab
identity, settings, modified inputs and non-finite HDR fail. Output directories
must be new and outside the checkout/native capture. Inputs and adapter source
fingerprint are checked again after rendering.

The stock Blender glTF importer must produce 11 mesh objects and 22,082
triangles. Its Y-up → Z-up mapping is applied to the actual captured camera
matrix and sun direction. The camera's vertical projection comes from captured
projection metadata, with unchanged extent, clipping and metre scale. There
is no background illumination, shadowing, denoising, adaptive sampling or
secondary-surface transport. The diagnostic uses CPU Cycles, seed 0, a
one-surface-bounce limit and BOX pixel reconstruction of width 1.

Outputs retain scene-linear float32 `radiance.exr`, pre-exposed top-down
`linear.rgba32f`, generated `display.png`, original `thessa.png`, the source
GLB/definition/license, and `comparison.json` with hashes and settings.
Display conversion explicitly applies the native captured exposure gain,
per-channel Reinhard and sRGB once. Blender's AgX/Standard view is not baked
into the comparison. The native image remains unchanged.

## Comparison limits and first observations

This is a **same-scene diagnostic**, not accepted Equal Work or Equal Quality:

- The light mapping is operational and explicit: one Thessa RGB illuminance
  input unit becomes one Cycles normalized SUN energy unit. This aligns numeric
  irradiance inputs; it does **not** establish spectral, luminous-efficacy or
  absolute photometric calibration. See [Cycles units and colors](https://developer.blender.org/docs/features/cycles/units/).
- Cycles imports glTF through its Principled BSDF. Thessa uses its existing
  direct GGX/Lambert/Fresnel shader and derivative roughness filtering. These
  models have not been proved equivalent; the adapter does not replace Cycles'
  BSDF with a copied Thessa shader.
- Cycles path samples and BOX reconstruction differ from native raster MSAA=1.
  The outputs also differ in float32/float16 storage. Narrow specular peaks and
  silhouette pixels require separate analysis rather than a single score.
- CPU offline Cycles duration and native GPU raster timestamps are not a
  performance comparison. Shadows, GI, environment, vegetation, temporal motion
  and subjective preference are outside this baseline.

The first actual 640×480, 64-sample CPU render has matching framing and the
expected geometry. Whole-frame pre-exposed RGB MAE is approximately 0.002716;
relative L2 is approximately 2.0803. Over 99.9% of squared error is concentrated
in the ten highest-error pixels around narrow highlights, so that L2 value
does not summarize the entire material grid. The roughest metallic sphere also
has a visible reflectance difference requiring a BSDF-specific investigation.
These observations establish a useful external diagnostic, **not parity**.
The subsequent [direct-GGX correction](93_STABLE_DIRECT_GGX.md) fixes one
numerical cause and records a new same-data native capture. The initial numbers
above remain the dated baseline; rough-metal/AA/model differences remain open.

The [portable evidence record](artifacts/b01-cycles-comparison-2026-10-29.json)
contains actual source/tool/artifact identities, settings, limits and metrics.
Use `arena.py` only after a reviewer accepts the declared comparison conditions;
no blind votes or physical acceptance are inferred from these metrics.

## Milestone record

- **Source commit:** `5320b93` (integrated development tree `41ebf48`).
- **Resulting commit:** the commit introducing this document.
- **Architectural change:** one Blender-side development adapter consuming the
  existing portable capture; no game scene, engine dependency or shader rewrite.
- **Changed files:** adapter/tests, dated evidence, this document and living indexes.
- **Tests executed:** 28 Python quality tests; actual Cycles 640×480/64-sample
  CPU capture in an isolated process without display/GPU/network; matching PNG
  and float32 HDR across repeated seed-0 renders; input/hash,
  geometry-count and finite-output checks; PNG/HDR convention tests; Godot
  archive verification and headless version probe. Syntax/diff checks passed.
- **Tests not executed:** Rust rebuild (no Rust changed), Godot/Unreal rendering,
  shadows/GI/temporal comparisons, independent spectral calibration, blind votes.
- **Numerical / visual evidence:** real same-GLB images and float32/float16 HDR
  comparison above; physical/visual parity remains unaccepted.
- **GPU/CPU measurements:** CPU render duration recorded as provenance only;
  no cross-engine performance claim.
- **Imported assets/licenses:** generated B01 geometry retains its MIT notice.
  Blender/Godot are external development executables, not engine dependencies.
- **Known limitations:** numeric light-unit mapping, different BSDF/AA/precision,
  initial diagnostic source fingerprint explicitly records dirty development files.
- **Next smallest useful change:** investigate isolated material differences,
  complete explicit B02 cutouts/tiers, and add a Godot capture over the same data.

## B05 continuation (2026-10-29)

[Doc 95](95_TRANSPARENT_EMISSIVE_SURFACES.md) extends the existing adapter to
B05 without replacing the stock glTF importer or Cycles materials. This
emission-only scene isolates alpha composition and uses eight transparent
bounces, 1,024 fixed samples and the same native exposure/display convention.
Its successful actual captures do not imply refraction, GI or temporal parity.

The user's Unreal ZIP download completed: 39,819,212,076 archive bytes,
299,959 entries and 76,967,950,798 declared unpacked bytes. Embedded
`Engine/Build/Build.version` identifies 5.8.4, changelist 58,902,052. Paths and
disk reserve were checked before extraction into the external tool
cache; unzip completed successfully. ZIP SHA-256 is
`cdff1e0c02010eb4fcf9c38d293cfd0f1afda09272256c146d73db52f570f607`.
Extraction provenance is `work/reference-engines/unreal-5.8.4/installation.json`
beside the checkout. No Unreal render is claimed. An isolated NullRHI bootstrap probe first failed
because its default DDC paths were read-only. Repeating with
`-DDC-ForceMemoryCache` and a temporary configuration mount initialized the
Installed Build and reached the deliberately missing
`__ThessaBootstrapProbe__` commandlet, then shut down with expected exit 1.
This verifies bootstrap/commandlet lookup, not a render or a successful real
commandlet. Display, GPU devices and network were unavailable inside the probe;
user configuration was not changed. Commands/logs are retained in the external
evidence directory under `ue-nullrhi-probe` and `ue-nullrhi-memory-probe`.
