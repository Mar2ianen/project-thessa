# 136 — Opaque terrain direct-sun hemisphere

Status: closed correctness slice, 2026-11-04; baseline `a123410`.
Close-ground texture quality and forest reference parity remain open.

## Research and applicability review

[pbrt-v3 BSDF reflection classification](https://github.com/mmp/pbrt-v3/blob/13d871faae88233b327d04cda24022b8bb0093ee/src/core/reflection.cpp#L613)
uses the geometric normal to distinguish reflection from transmission even when
shading uses another normal. Thessa already follows this distinction in
`mesh_diffuse_cosines` and secondary RT surface lighting. The primary terrain
fragment instead accepted direct diffuse and land GGX sunlight solely through
its mapped normal. A normal tilted toward the source could therefore illuminate
an opaque triangle whose actual geometry faced away from it. This change reuses
Thessa's existing contract; no external source code was copied.

The receiver is an opaque exterior terrain triangle. Its existing derivative
`receiver_normal`, oriented with the smooth normal, owns the geometric hemisphere.
The material normal still owns the BRDF shape. The rule is not a complete
shading-normal energy correction, nor a replacement for terrain subdivision.

## Implementation

The shared terrain fragment gates direct land diffuse and direct land GGX with
`dot(receiver_normal, light_direction) > 0`. An independent accumulator preserves
the previous liquid endpoint; coverage blends land and water without transmitting
light into the opaque fraction. Diagnostic 13 follows the same coverage split.
Sky/environment illumination, source atmospheric transport, water wave/spectral
reflection inputs and existing shadow visibility remain unchanged.

There are no new settings, effects, passes, bindings, allocations, assets or
terrain topology changes. Existing normal-map/specular quality settings still
apply. Cost is bounded fragment ALU and one additional RGB accumulator; release
performance has not been measured. TAA remains off.

## Verification and visual review

The physical-GPU regression
`mapped_ground_cannot_reflect_sunlight_through_its_geometric_backside` exercises
flat and sloped roots, regular and persistent conforming topology, Low/Medium/High,
Lambert and coupled GGX, black-albedo specular isolation, and sky off/on. Clouds,
atmosphere, shadows and ocean are explicitly disabled in this isolated fixture.

- Original shader: all 72 backside comparisons fail. The minimal Lambert case
  produces RGB `[82,82,82]` instead of the unlit `[0,0,0]`.
- Fixed shader: all 72 exactly match the sky-only control, including a sloped
  receiver with the source above the planetary horizon. All 36 front-light
  positive controls remain active.
- Red/green liquid controls: 24 pure-water full frames are byte-identical;
  all 48 mixed/full-water center samples are identical. A separate partial-water
  Lambert oracle scales the normal gradient before normalization and agrees
  within one RGB8 code while the opaque fraction rejects backside light.
- Core unit tests: 89 passed, 1 ignored. Relevant terrain-environment suite:
  16 passed; only the diagnostic GPU-cost benchmark excluded. The final partial-
  water oracle was added afterward and its targeted physical-GPU run passed.
- Existing ray-scene compatibility: 32 passed, 4 ignored (special hardware/cost
  diagnostics). Strict Clippy, native build and the architecture gate pass.

Root visually inspected every emitted control image through complete review
sheets, plus the native FHD B03 beauty and all 21 diagnostic views. Independent
Luna review examined the shared land/water equations, fixture and native views;
no blocking regression was found for this slice. Synthetic coarse roots expose
the expected faceted geometric terminator. The actual 0.3 m B03 capture has no
obvious new visual artifact, but still shows broad smeared/striated ground.
Separate timed scene captures have different residency and are not exact whole-
frame A/B or a blind preference comparison. Debug GPU fences are explicitly
`performance_equivalent=false` and cannot support a frame-time claim.

Source, binary, log and complete image hashes are recorded in
[the evidence manifest](artifacts/opaque-terrain-sun-hemisphere-2026-11-04.json).
Rejected fixture assumptions (backlit pure water and linear interpolation of
coverage endpoints with different normals) were corrected before acceptance;
the production contract was not weakened to satisfy them.

## Remaining work

This closes backside direct-sun leakage only. Close-soil blur/material scale,
finite-path atmosphere, directional leaf transport, general shading-normal
correction, coarse-triangle terminators, coastline integration and full-scene
visual/performance acceptance remain separate open work.
