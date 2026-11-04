# 120 — Frozen native terrain diagnostics

Status: foundational task 1 implemented and verified in one process. Independent
process restoration, transport decomposition and reference parity remain open.

## Published-frame contract

`thessa-terrain-native --terrain-diagnostic-sequence NEW_DIR` captures 18 terrain
views after normal warmup/rendering. With the optional doc 123 ambient receiver,
three beauty Off/On/Off controls precede these views (21 total); the actual HDR
receiver must remain byte-identical across controls. It freezes the already-published CBT
vertices, topology records/state, draw arguments, height/material residency,
meshes, camera, ephemeris time, exposure, sky/ocean/light resources and shadow
atlas. No stream polling, simulation tick, topology update, mesh preparation or
resource publication occurs between views. Pending CPU workers may finish but
cannot publish into this immutable scene. This is an in-process replay; the
manifest alone cannot restore GPU resources in another process.

The sequence requires a static headless survey, conforming CBT, HDR, RT Off,
temporal effects Off and no HUD/performance capture. Unsupported combinations
fail before rendering; the output directory must be new. Normal execution with
`debug.terrain_view = "none"` retains constant shader specialization and zero
new diagnostic storage. An explicitly configured diagnostic renderer allocates
one 16-byte uniform and supports resolved view changes without recreating the
renderer. The sequence seeds that opt-in allocation when the input requests
None; its saved requested/resolved configuration records this QA seed.
The diagnostic flag itself introduces no production receiver image or pass.
The independent opt-in ambient visibility effect has its own receiver/pass
contract in doc 123. Diagnostic runs request optional GPU timestamps for that
pass; missing timestamp support is recorded explicitly as unavailable.

Each view writes PNG and raw pre-exposed RGBA16Float. One raw resolved
Depth32Float image, raw vertex/topology buffers, requested/resolved graphics,
compiled world recipe/system and a manifest record source generations,
residency levels, epochs, matrices, light sources, population and draw state.
Completion requires byte-identical depth across all views, byte-identical
beauty PNG/HDR at the start/end, and unchanged vertices/topology/draw/epochs.
The manifest starts incomplete and is updated after each successful view;
a failed or partial sequence never receives a completion claim.

Geometric/material debug values remain unexposed. Direct/Ambient are source
irradiance divided by PI with exposure; Specular is the pre-mix land reflection.
They are **not additive final-radiance components**. Meshes and sky remain in
beauty. This does not implement a general per-object G-buffer.

## Executed acceptance

The bounded dynamic-view GPU regression passes on isolated software Vulkan and
Radeon 880M/RADV. It checks diagnostic allocation policy, rejected None-to-debug
activation, nontrivial view changes, depth preservation and round-trip identity.
CLI rejection tests and strict render-wgpu/native Clippy pass; the architecture
guard passes for 44 packages/20 foundations.

Actual B03 ran at 1920×1080, 4× MSAA, full High plant demand, 12° sun, EV11,
5 seconds prestream and one 1024²/100 m local shadow cascade. All 17 views
completed; depth, beauty round trip and published geometry checks passed.
The frozen primary mesh plan has 17,992 candidates, 19,660,054 candidate
triangles and 2,775,318 submitted triangles. Height residency contains 875
pages; pending jobs were [5,4] and were deliberately not published during replay.
Exact identities, commands, manifest and hashes are in
[the artifact](artifacts/frozen-native-diagnostics-2026-11-03.json).

The physical run uses the retained isolated render-node runner and serial
per-stage fences with a 20M mesh-triangle admission limit. Its wall times are
**not production FPS or GPU timestamp measurements**. There is no quality or
speedup claim: this milestone makes subsequent defect comparisons causal.
Doc 121 subsequently implements independent water presence. Source-error
bounds, bounce and complete receiver/component outputs remain separate tasks
from doc 119; bounded current-frame sky visibility is subsequently implemented
in doc 123.

Doc 134 adds `resolved_material_lod` (code 16) beside the unchanged requested
`material_lod` (code 11). This extends the current frozen sequence by one view;
the historical 17/20-view captures above retain their original counts.
The new view reports albedo/roughness material-page ancestry and resident page
coverage, not normal-directory residency, within-page mip level or final
water/globe contribution. See [doc 134](134_RESOLVED_MATERIAL_PAGE_LOD.md).
