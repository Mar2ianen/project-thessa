# 6 — Physical scale and ground-normal energy

Status: source/page bandwidth increased with integer L25 addresses; foreground appearance open.

## Research

[Mikkelsen 2020](https://jcgt.org/published/0009/03/04/) and actual Unity Graphics surface-gradient implementation at `0efc28ea742b1966b9dd9cc4489e58b986a13faf` are the composition reference already inspected in docs 127–128. Source gradients must be combined once and filtered by physical sampling footprint. More high-frequency noise is not an energy or scale correction.

## Applicability review

Thessa combines scan height, scan-normal residuals, procedural metric-scale mesorelief and moss cushion displacement. Partial height-field coverage previously duplicated optical slope; doc 128 fixes that exact bug. The muddy foreground in the preceding inspected ROI had full coverage, so that bug cannot explain it. A prior roughness-only experiment was rejected. Meter-scale procedural relief and high-contrast masks still require separate inspection from millimetre-scale normal detail.

## Implementation decision

Allow explicit survey eye clearance down to 0.3 m; retain defaults and normal survey behavior. Near plane follows the existing 5% eye-clearance rule (15 mm at 0.3 m). Boundary tests include 0.3/2/10/100 m and reject 0.299 m. Do not modify roughness or introduce uncalibrated sharpening before inspecting normal/albedo/roughness at physical scale.

## Verification needed

Capture a 0.3 m soil specimen with fixed source/exposure. Measure normal-gradient energy over physical frequency bands and inspect scan/mesorelief/moss separately. The present vegetation work does not repair close soil or prove temporal stability. Full terrain subdivision remains Large CBT-based; no return to classical tile LOD is introduced.

The executed 0.3 m view still shows blurred foreground soil and intersecting close
fronds. Its log reports material pages through level 24 and four pending material
jobs at capture. Presence of level-24 pages does not prove local footprint coverage;
inspect the actual receiver debug values rather than raising a global LOD limit.

The subsequent [resolved-page audit](../134_RESOLVED_MATERIAL_PAGE_LOD.md)
inspects all 21 native views at 0.3 m. It rejects missing fine albedo/roughness
ancestors as the dominant blur explanation in this frame: only 0.1846% of covered
bottom-third pixels have a requested/resolved gap above 0.5. Albedo and normal
still look smeared; the approximately 3 mm source/page spacing is coarser than
many foreground pixels. Source bandwidth, within-page mip filtering and separate
normal residency remain separate tests, not an accepted quality repair.

The [bandwidth candidate](../135_CLOSE_GROUND_BANDWIDTH.md) carries matching 2K
soil/moss sources and High L25 optical pages (roughly 1.5 mm samples), retaining
Low/Medium and fixed GPU budgets. It fixes reproduced f32 carry/cube-neighbor
failures. Full 21-view source-only and L25 scene reviews see incremental finer
variation, while the broad smeared soil remains. CPU source cache rises 224 MiB;
release timing, broader view residency and soil/reference acceptance remain open.

[Direct-sun hemisphere correction](../136_OPAQUE_TERRAIN_SUN_HEMISPHERE.md) closes the reproduced
opaque-ground backside diffuse/GGX leak. Sky and water controls are preserved;
complete native visual review found no obvious new regression. This is a light
contract repair, not closure of item 6 soil blur or reference acceptance.

[Doc 137](../137_GROUND_SOURCE_FIDELITY.md) retains the independent soil roughness source
and restores the litter scan to native 1K across color, roughness, height and
normal planes. Soil/moss stay 2K; GPU page/cache budgets are unchanged. The
extra CPU source pyramids cost 28 MiB. Matched L25 pages show finer source
features, while broad foreground smear and reference acceptance remain open.

[Registered scan composition](../138_REGISTERED_GROUND_MATERIAL.md) supersedes the earlier
brightness-derived humus/mineral grade and duplicate flat litter coat. Scanned
soil/litter now retain their albedo/roughness detail beneath the existing wetness
and moss layers; normal/base/water page controls remain identical. Independent
review retains this material slice, with broad close-ground smear and full-scene
reference acceptance still open. No persistent memory or GPU work is added.
