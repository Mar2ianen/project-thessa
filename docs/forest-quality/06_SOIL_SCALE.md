# 6 — Physical scale and ground-normal energy

Status: reference and existing implementation reviewed; foreground appearance open.

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
