# 134 — Resolved material-page LOD and close-soil diagnosis

Status: diagnostic implemented; actual native close-ground capture inspected,
2026-11-04; baseline `443c4f4`. This is instrumentation, not a soil-quality fix.

## Reference research

The inspected MIT OpenEngine source is pinned to
[`bdf5c2710fe37c1c5ddbac7787477daa87524af3`](https://github.com/Game-Crafters-Guild/OpenEngine/tree/bdf5c2710fe37c1c5ddbac7787477daa87524af3).
`Engine/Modules/CBTTerrain/Shaders/CBT/cbt_page.glsl:68–78` carries the level that
actually answers a page query, while lines 246–277 and 471–519 resolve resident
ancestors and their weights. `cbt_planet_page.glsl` consumes that source for
planet gradients. Thessa already has an ancestor walk; no fallback or topology
algorithm is copied. Thessa's higher level numbers mean finer pages, whereas
OpenEngine's source uses a different level direction. Large CBT remains the
terrain topology reference; this step measures optical page residency separately.

## Applicability review

The actual 0.3 m frame has badly smeared ground. Logs showing some L24 pages
cannot prove that the visible receiver samples them. The old `material_lod`
view shows requested footprint detail, not the resident ancestor actually used.
That distinction must be measurable before changing source filtering, page
budgets, geometry or authored roughness. Prior partial-height normal correction
(doc 128) does not explain this full-coverage foreground failure.

## Implementation

`debug.terrain_view = "resolved_material_lod"` uses the existing requested/resolved
settings and runtime diagnostic uniform. Old numeric values remain unchanged;
the new selector is 16. During the existing material ancestor walk, accumulate
`actual_level * material_sample_weight`. Combine coarse/fine sums with the same
fractional requested-LOD blend as the actual material, then divide by their
combined resident coverage.

Unexposed linear channels are R = weighted page level / resolved maximum level,
G = material-page coverage, B = 0. With no material pages the output is magenta.
The current frozen native sequence includes this view (18 views, or 21 with the
ambient-visibility controls) and writes both PNG and raw RGBA16Float.

This reports the albedo/roughness directory. Separate normal-page residency,
hardware within-page mips and final globe/water mixing are not measured. Use
near-ground albedo/normal/footprint/relief controls to interpret it. It introduces
no pass, image, binding or allocation; normal beauty keeps constant view None,
allowing the unused scalar sum to be eliminated. Runtime-enabled diagnostic
shaders retain the scalar accumulation as debug work.

## Verification

Root integrated only the reviewed diagnostic hunks from Luna's separate worktree,
preserving newer root normal-coverage tests and unrelated comments. CPU compilation
alone missed two fixture setup errors caught by the physical run: the height-page
ID helper intentionally stops at L22, and ground-detail-disabled clamps material
LOD to L17. The corrected fixture uses explicit L23/L24 descendant IDs and the
existing High ground-detail setting; production selection is unchanged.

The physical-GPU test passes parent-only L23 answering an L24 request, both levels
resident, a genuine fractional request (23.574 matching resolved 23.574), and the
explicit missing-page sentinel. Existing tolerances are not relaxed. Eight material GPU regressions pass, including cube-chart subtexel motion,
anisotropic admission, material upload/fallback, independent height normals and
the unchanged partial-coverage slope guard. CPU graphics tests pass 55 cases,
core tests 89 (1 ignored), native tests 153 (51 ignored); strict Clippy and
architecture pass.

## Actual scene evidence and next decision

The FHD B03 camera at 0.3 m completed all 21 frozen views. Depth, beauty round
trip and published geometry are identical. The complete sheet and native-resolution
albedo/normal defects were inspected. Both albedo and normal remain visibly smeared.
In the bottom-third ROI (656,599 covered terrain pixels), requested page LOD has
median 24 and resolved LOD has median 23.9883 (RGBA16Float rounding of 24/24).
Only 0.1846% have a requested/resolved gap above half a level, and 0.1255% above
one level. Height coverage is 0.9995–1.0. Thus missing fine material ancestors
are **not the dominant albedo-blur cause in this captured frame**. This conclusion
does not establish normal-directory residency or other viewpoints.

The median measured geometric pixel footprint is 1.600 mm (10th percentile
0.734 mm), while the nominal L24 material page spacing is 3.052 mm. The native
soil scan is 1024 samples across an authored 3.15 m repeat (3.076 mm); moss is
1024 across 3 m (2.930 mm). This identifies a real sampling ceiling, not yet
proof that simply increasing source/page resolution fixes all visible streaks.
Audit source bandwidth and within-page mips independently before implementing
another soil change. [The evidence record](artifacts/resolved-material-lod-2026-11-04.json)
contains image/log hashes, raw-channel ROI statistics and the explicit limitations.
