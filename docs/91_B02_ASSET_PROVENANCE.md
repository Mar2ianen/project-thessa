# 91 — B02 vegetation asset provenance

Status: pinned ingestion and explicit raster cutout capability implemented; native B02 scene pending.

## Selected source data

[Fern 02](https://polyhaven.com/a/fern_02) and
[Moss 01](https://polyhaven.com/a/moss_01) are individually selected under
[Poly Haven's asset CC0 policy](https://polyhaven.com/license).
`data/reference-scenes/b02-assets.json` pins the original 1K Blender files and
four maps per asset by URL, size and SHA-256. The ten selected files total about
3.6 MB. No collection or high-resolution variant is downloaded.

The exact CC0 notice is committed in `data/reference-scenes/licenses/CC0-1.0.txt`,
with its SHA-256 and SPDX upstream commit/blob recorded in the manifest. The
Poly Haven website/API implementation is not an asset license; none of that
implementation is copied into the runtime. Live data requests identify this
project with a distinct User-Agent.

| Dataset | Selected geometry | Triangles | Derived GLB bytes |
| --- | --- | ---: | ---: |
| Fern 02 | Four full evaluated source clumps | 6,232 | 4,047,388 |
| Moss 01 | Evaluated `moss_01_geometry_nodes` root | 245,438 | 13,961,464 |

The published 1K Moss glTF descriptor contains only 214 template triangles.
It is deliberately **not** used as the fine-geometry workload. Source prototype
and LOD-library objects are excluded from the evaluated root export. There is
no decimation. The public glTF descriptors' audit hashes are retained in the
manifest, separately from the selected `.blend` sources.

## Reproduce or verify the external cache

```sh
python3 tools/agent-qa/reference_assets.py --cache /absolute/external/cache
python3 -m unittest discover -s tools/agent-qa/tests -v
```

The default cache is `$XDG_CACHE_HOME/thessa/reference-assets` (or
`~/.cache/thessa/reference-assets`). Caching inside the source checkout is
rejected. The tool verifies existing derived GLBs without requiring Blender or
network access. Missing artifacts require Blender **5.2.2 LTS**, build
`d13f752e3b9c`, and the pinned source files. Each missing source is fetched
within a 16 MiB per-file budget; existing mismatched cache files fail rather
than being overwritten. Licenses and the repository converter are checked
before ingestion. No third-party Python scripts are fetched or executed.

Blender runs in background factory mode with automatic source scripts disabled,
two threads and a nonzero Python-error exit code. It executes only the audited
repository converter. The result must match the pinned GLB SHA-256, triangle
count, source hash, converter hash, Blender version and build. A failed result
is not installed. `conversion.json`, `conversion.log` and `command.json` remain
beside each external cached GLB. Large binary assets do not enter Git history.

## Explicit derived material variant: `core-cutout-v1`

This dataset is an independently labeled core metallic-roughness/cutout variant,
**not an equivalent export of the original Blender shader graphs**:

- Evaluated geometry, authored object world transforms, UV layers and vertex
  normals use the stock glTF exporter. Blender Z-up becomes right-handed Y-up;
  source unit scale is checked at one metre. No authored tangent accessor is
  exported. The existing native derivative tangent frame has not been verified
  against Blender/MikkTSpace. Independent numerical geometry/normal round-trip
  validation remains pending.
- Scanned base color, roughness, normal and alpha maps are used directly.
  Complex source translucency, Mix/Add and hue graphs are omitted explicitly.
  The derived baseline uses default glTF dielectric IOR 1.5 and specular
  response; source IOR/specular settings are retained in conversion reports.
  The simple Moss substrate retains its base color, metallic and roughness
  constants. Source normal-map strength is retained (Moss: 2).
- Source dithered/continuous alpha behavior becomes `MASK` at the stored cutoff
  0.5, with source sidedness retained. This is a benchmark coverage definition,
  not Blender dithered-render equality. Only exact identity-default IOR/specular
  extension declarations are removed. Unexpected extensions fail conversion.
- All maps retain 1024×1024 dimensions. Embedded base RGB is byte-identical to
  decoded source JPEG RGB. Source 16-bit alpha and EXR data become 8-bit PNG;
  EXR values outside [0, 1] are clipped. Source normal maxima are 1.0263671875
  (Fern) and 1.0234375 (Moss); no angular normal-error bound is claimed.

The committed [audit evidence](artifacts/b02-asset-audit-2026-10-29.json) records
portable source/derived hashes, evaluated frames and object counts, material
deltas and texture checks. Maximum normalized alpha error is 0.002030 for Fern
and 0.001969 for Moss. At cutoff 0.5, Fern atlas coverage is unchanged; Moss
changes by one atlas texel out of 1,048,576. These are texture-conversion checks,
not rendered silhouette or temporal-quality acceptance.

The strict default importer still rejects MASK. The later
[explicit cutout API and raster pipeline](94_MESH_CUTOUT_COVERAGE.md) now retain
coverage without changing opaque B01 output. Shadow/RT coverage, density tiers,
culling/overdraw, native B02 captures and external vegetation comparisons remain
unimplemented here.

## Asset-ingestion milestone record (2026-10-29)

- **Source commit:** `148dce7` (integrated development tree `6440053`).
- **Resulting commit:** the commit introducing this document.
- **Architectural change:** development-only, pinned cache producer; no runtime
  dependency or renderer change, no additional Cargo package/version.
- **Changed files:** asset/license manifests, trusted converter, standard-library
  fetch/verify tool and tests, portable audit evidence and synchronized indexes.
- **Tests executed:** 25 Python quality tests; fresh conversion from pinned raw
  inputs in a second cache, with no GPU/display/network access, matching both
  derived GLB hashes; cached verification; syntax/diff checks. Earlier selected
  downloads were verified against published sizes/checksums. RGB/alpha checks
  used local Pillow against the actual embedded image bytes.
- **Tests not executed:** Rust tests/build (no Rust changed), native B02 GPU
  rendering, shadow/RT/temporal behavior, external adapters or subjective votes.
- **Numerical evidence:** triangle totals, exact derived repeat hashes and texture
  errors above; no independent BRDF or full shader-equivalence acceptance.
- **Visual evidence / GPU/CPU measurements:** none for B02; conversion is CPU-only
  and is not a renderer benchmark. Existing B01 repeat evidence remains separate.
- **Imported assets/licenses:** selected Poly Haven Fern 02 and Moss 01, CC0-1.0;
  authors and license/source evidence retained per asset. Blender is an external
  development tool; no Blender implementation is copied into the engine.
- **Known limitations:** pinned exporter build required for conversion; declared
  shader simplification, data quantization/clipping and unverified tangent frame.
- **Next smallest useful change:** explicit MASK capability with opaque regression
  checks, then fixed-camera/light B02 scene data and bounded density tiers.
