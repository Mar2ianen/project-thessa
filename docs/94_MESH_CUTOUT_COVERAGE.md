# 94 — Explicit mesh cutout coverage

Status: opaque/MASK raster materials implemented; alpha blending and emission are the next milestone.

The neutral `Material` now records glTF `AlphaMode` and a finite nonnegative
cutoff. Opaque alpha is ignored; MASK survives when sampled base alpha times
the material alpha is at least the cutoff. Cutoff zero retains even zero-alpha
texels; values above one reject the surface. This follows the
[glTF coverage contract](https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html#alpha-coverage).
The `Blend` semantic is represented explicitly but the current mesh consumer
rejects it until a transparent pass exists.

`decode_glb_cutouts` is an explicit importer entry point. Ordinary `decode_glb`
and rest-pose decoding remain strict opaque/non-emissive subsets. BLEND,
animations and material extensions are still rejected by this milestone.
Source UVs, alpha factors, sidedness and cutoff are retained.

The existing mesh consumer creates a separate masked shader/pipeline only when
the asset contains MASK. It shares the existing explicit layouts and buffers,
uses the unused material optical Z word for cutoff, and selects fixed-function
sidedness per primitive. The opaque shader gains no coverage-discard branch.
Its color texture sample now retains alpha for the masked variant.

Masked texture samples and normal derivatives are evaluated before coverage
rejection; rejected texels skip direct/environment lighting and write neither
color nor depth. Normal-footprint derivatives are reused after rejection rather
than evaluated in divergent control flow. This is the standard alpha-scissor
representation used for foliage in [Godot](https://docs.godotengine.org/en/4.7/tutorials/3d/standard_material_3d.html#transparency),
not partial surface transparency or physical transmission.

Unsupported MASK shadow construction fails explicitly. Ray planning reports an
unavailable-coverage reason and keeps the existing raster fallback; direct
`RayScene` construction rejects non-opaque sources. It cannot silently trace
solid leaf rectangles. Shadow coverage, alpha-to-coverage, coverage-preserving
mips and temporal acceptance remain future work.

## Verification

- 48 render-core tests, six actual GLB decoder tests and 40 ordinary render-wgpu
  tests pass; 21 GPU diagnostics remain ignored by default.
- The new actual GPU fixture runs 12 combinations of quality, sidedness,
  factor and cutoff on both llvmpipe and Radeon 880M/RADV Vulkan. A blue opaque
  background is drawn after foreground cutouts, checking holes do not block its
  depth. It also checks opaque alpha=0 remains opaque, equality at cutoff,
  cutoff zero/above one, both masked sidedness pipelines, shadow rejection and
  explicit RT fallback.
- Strict all-target Clippy for render-core/assets/wgpu passes. Native app build
  and its real 640×480, 30-warmup/60-measured-frame Radeon B01 capture pass.
  PNG, pre-exposed HDR, GLB and scene JSON are byte-identical to the pre-cutout
  `c332c5b` renderer capture. There is no B02 performance claim yet.

[Evidence](artifacts/mesh-cutouts-2026-10-29.json) pins the checks, binary and
capture hashes. Large assets remain in the external cache from
[doc 91](91_B02_ASSET_PROVENANCE.md); a native B02 scene is still pending.

## Milestone record

- **Source commit:** `c332c5b` (integrated development tree `bd331b9`).
- **Resulting commit:** the commit introducing this document.
- **Architectural change / dependencies:** neutral material coverage semantics,
  opt-in existing decoder and existing raster variants; no new dependency.
- **Changed files:** core material/validation, decoder/tests, mesh/ray consumer
  guards, GPU coverage fixture and synchronized documentation/evidence.
- **Tests executed:** the CPU/import/GPU/build/capture/Clippy checks above.
- **Tests not executed:** full coastal replay, Metal/WebGPU hardware, temporal
  or blind acceptance, full workspace tests, native B02 density captures.
- **Numerical/visual evidence:** explicit alpha/depth fixture and exact opaque
  B01 repeat; no preference or cross-engine parity inferred.
- **GPU/CPU measurements:** no speedup claim; rejected MASK texels structurally
  skip lighting, but workload scaling needs the bounded vegetation scene.
- **Imported assets/licenses:** none in this change; existing CC0 cache is retained.
- **Known limitations / next change:** implement the requested transparent and
  emissive materials using standard compositing, then build bounded B02 tiers.
