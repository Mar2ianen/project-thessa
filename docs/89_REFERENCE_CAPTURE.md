# 89 — Native capture provenance

Status: R5 capture infrastructure and native B01/B05 implemented
([doc 90](90_REFERENCE_SCENES.md)); B02 staged.
No benchmark assets were imported for this milestone.

## Existing paths, extended

`tools/agent-qa/capture.py` builds and launches `thessa-terrain-native`. It reuses
native offscreen targets/PNG, the requested → capability → resolved graphics
pipeline, `thessa-perf` JSON/CSV and the blind arena PNG validator. It does not
introduce a second renderer, app framework, performance model or image judge.

The native app accepts `--linear-hdr PATH` alongside `--headless PATH.png`.
When native HDR is supported/enabled, it writes tightly packed RGBA16Float
bytes from the pre-tonemap target after all measured frames. These are
**pre-exposed linear scene values**, not absolute radiance; exposure gain is
recorded. Four 16-bit channels per pixel, top-to-bottom rows, no row padding;
the tested little-endian Vulkan host preserves IEEE binary16 values. No EXR
encoder, rescaling or extra tone mapping is applied. HDR-disabled/unsupported
runs explicitly report unavailable and do not fabricate an HDR file.

`render-wgpu::readback::read_color` is the shared diagnostic byte readback for
RGBA8 and RGBA16Float. The existing PNG path uses it too. It validates image
format, COPY_SRC usage, single-sample 2D shape, aligned row arithmetic and buffer
budget. Capture/readback is outside measured frames. The normal window path
allocates no readback staging buffer; HDR attachments gain COPY_SRC usage only.

## Manifest and invocation

A small TOML capture manifest describes the requested run:

```toml
schema_version = 1
id = "existing-native-probe"
width = 128
height = 96
frames = 2
native_args = ["--prestream-seconds", "0", "--no-hud", "--graphics", "graphics.toml"]
```

This example uses the existing canonical producer, not a new reference scene.
Use a bounded explicit graphics configuration for diagnostic runs; the checked-in
production graphics configuration is not a fixed-work reference benchmark.

```sh
python3 tools/agent-qa/capture.py /path/to/capture.toml \
  --output /tmp/thessa-capture-new --profile dev
# Use --profile release (the default) for future performance measurements.
```

Output must be a new directory outside the source checkout. The tool owns
headless/perf/HDR paths, resolution and frame-count arguments. Frames are
bounded to the existing 1800-frame perf ring; dimensions to 4096 per axis.
Native still performs exactly 30 warmup frames.

Clean source is required by default. `--allow-dirty` is an explicit diagnostic
option: parent commit, tracked binary-diff hash and untracked-file hashes are
recorded, and the compiled label gains `-dirty`. Such a label is not a claim
that the parent commit alone reproduces the binary. The tool verifies source,
binary, manifest and declared inputs did not change during the run.

Tracked configuration/source inputs are identified by the source commit (or
explicit dirty fingerprint). External inputs belong in `[[inputs]]` with an
exact `path` and lowercase `sha256`. Relative paths resolve against the source
checkout, not the manifest directory. The tool verifies input hashes before
building and after capture. It does not fetch anything or run conversion scripts.

Each `[[assets]]` entry additionally needs: id, source, license,
upstream_revision_or_version, source_sha256, converted_sha256,
conversion_tool_version, asset_format, unit_scale, coordinate_convention,
texture_color_spaces, geometry_lod and a local license_notice path. The consumed
file hash must match converted_sha256; the license notice is hashed too.
These fields record a prior **individual license/conversion audit**; their
presence does not automatically establish licensing truth. No placeholders.
Large assets remain in a reproducible external cache, outside ordinary Git.

## Outputs and identity

- `scene.toml`: exact capture manifest; its SHA-256 is the run-manifest identity.
- `display.png`: actual native display output; `review.png` retains its pixels
  and rendering chunks while removing origin metadata through the existing arena.
- `linear.rgba16f`: pre-exposed linear HDR, when available.
- `perf.json` / `perf.csv`: existing native CPU/GPU/pass samples.
- `commands.json`, `build.jsonl`, `build.log`, `native.log`: actual build/run evidence.
- `capture.json`: source identity, binary hash, profile, verified inputs,
  actual native metadata and hashes of the output artifacts (excluding itself).

The capture metadata adds final camera pose/projection/origin, exposure gain,
simulation time, ambient and directional RGB illuminance/directions, projection
convention, HDR capability and warmup. Existing metadata retains requested and
resolved graphics, sky/environment configuration and backend/GPU/driver.
GPU timestamps remain explicitly unavailable when unsupported. CPU submission,
wall completion and GPU pass scopes retain the existing measurement boundaries;
serialization and image readback remain outside samples. Dev/CPU-Vulkan samples
are diagnostics, not target-GPU performance results.

The manifest hash identifies the declared run. It does not yet prove immutable
canonical resident geometry, asynchronous streaming state or equivalent content
in another renderer. The capture probe uses the existing canonical scene and
is **not** B03 Frozen Coast. B01/B02 will supply deterministic scene definitions
and content hashes; native B01 now emits actual GLB/scene sidecars with separate
`reference_geometry_sha256` and `reference_scene_manifest_sha256`, bound to the
requested and actual scene IDs. B02, B03/B04 and external adapters remain later work.

## Comparison

Pass matched `review.png` files into `tools/agent-qa/arena.py`. Keep capture
records and answer keys outside the blind directory. Camera, content,
exposure/color handling, settings and workload must agree before interpreting
preferences or timings. The tool never assigns its own visual acceptance.
Physical correctness, subjective appearance, performance and temporal stability
remain four separate outcomes, initialized as unevaluated in the record.

## Verification (2026-10-29)

- 20 Python quality tests: architecture, arena, checksums, reserved output/workload
  flags, incomplete asset provenance, source label, camera and warmup metadata.
- Strict Clippy for render-wgpu/native all targets; architecture gate and formatting.
- Explicit isolated software-Vulkan readback test: 7×3 RGBA8 and RGBA16Float
  images retain exact values (including HDR 2.0) and remove row padding.
- Actual capture-tool smoke: existing canonical producer, 128×96, 30 warmup +
  2 measured frames, HDR on, no clouds/shadows/plants/RT, dev profile. Both PNG
  and 98,304-byte linear HDR saved with CPU/GPU scopes and full provenance.
  Backend: llvmpipe / Mesa 26.2.5-arch1.1 / LLVM 23.1.3, isolated from /dev/dri
  and display. The explicitly dirty source fingerprint is retained in the
  capture; no physical-GPU performance, B01/B02 or visual acceptance claimed.

A subsequent clean-source smoke repeated at `271e56a` with `dirty=false`, no
untracked source files and the exact commit in native perf metadata. This
confirms the compiled source-label path beyond the earlier dirty diagnostic.

B01 native capture now has a tested fixed-work path and two exact static Radeon
880M repeats. See [doc 90](90_REFERENCE_SCENES.md) for profiles, supported flags,
static mesh work counters and physical-GPU evidence scope. Reference sidecars
are checked for matching IDs and hashed separately from the requested run
manifest. Ordinary canonical captures retain their previous timing scope layout.

The same capture contract now supports [B05 transparent/emissive panels](95_TRANSPARENT_EMISSIVE_SURFACES.md),
including exact static repeats, shared Cycles inputs and separate scene/geometry
hashes. No new capture protocol or game-scene loader is introduced.
