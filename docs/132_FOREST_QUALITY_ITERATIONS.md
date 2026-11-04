# 132 — Forest quality: separate research, review and implementation

Status: active seven-item campaign from the supplied forest critique, 2026-11-04.
Each linked item separates reference research, applicability review, implementation
and verification. A researched item is not an implemented or visually accepted item.

| Item | Record | Current implementation status |
| --- | --- | --- |
| 1 | [Spatial communities](forest-quality/01_COMMUNITIES.md) | Closed placement slice: communities, canonical crown/root niches, cross-face halos |
| 2 | [Ground transitions](forest-quality/02_GROUND_TRANSITIONS.md) | Parent-linked projected litter, cover-driven moss emergence and 32 m near-cover envelope in verification |
| 3 | [Leaf transport](forest-quality/03_LEAF_LIGHT.md) | Shared mesh GGX black-patch correction verified; directional scattering lobe and scene acceptance open |
| 4 | [Tree architecture](forest-quality/04_TREE_ARCHITECTURE.md) | Single/forked morphotypes now weighted 65/25/10; new architecture open |
| 5 | [Forest depth](forest-quality/05_FOREST_DEPTH.md) | Existing spherical haze audited; finite-path integration open |
| 6 | [Soil detail scale](forest-quality/06_SOIL_SCALE.md) | 0.3 m resolved-page audit rejects dominant albedo ancestor fallback; source/mip detail open |
| 7 | [Shoreline](forest-quality/07_SHORE.md) | Water/depth/ecology owners audited; coherent shoreline open |

## Cattail step carried into the campaign

The separately requested tall-grass replacement reuses `ReedClump`: three round
culms terminate in brown volumetric spikes, alongside the retained curved strap
leaves. [USGS Typha morphology](https://nas.er.usgs.gov/queries/FactSheet.aspx?speciesID=3020)
is the organ reference. This is an authored cattail-like optical proxy, not a
species-accurate or palaeobotanical claim. Existing terminal dry-organ material,
LOD generation, shadow/ray geometry and graphics quality tiers own the result.
The six-sided culms preserve the normal/face agreement guard; brown spike cones
are excluded from an upward-only *leaf* normal guard, without relaxing face tests.

Full clump triangles increase from 1304 to 1520. The dry material adds one source
group (36 → 37 in the coastal fixture); grouped RT instance demand counts the
additional organ even though the world descriptor is shared. Admission and
fallback tests verify this extra demand rather than truncating the raster source.

## Validation protocol

Use five diagnostic views: 0.3 m soil, 2 m interior, 10 m crowns, 100 m forest and
low-sun backlight. Keep scenario/time/exposure fixed within each material/light
experiment. A placement experiment intentionally changes membership; surviving
IDs/positions and the underlying substrate remain fixed. Reuse existing albedo,
normal, roughness, direct and diffuse-sky diagnostics. Independent runs using a
timed readiness threshold may have different page publication; they are not exact
whole-frame A/B controls. Frozen diagnostic sequences verify per-run geometry.

Captures use a render-node-only isolated process, one GPU job at a time. Debug
builds with serial stage fences are not production frame-time evidence. Author
inspection is unblinded and cannot close independent aesthetic acceptance.

Evidence and actual completed check counts are recorded in
[the campaign artifact](artifacts/forest-quality-2026-11-04.json).

## Executed iteration review

The 64 m occupancy candidate was rejected after author inspection: it removed
most nearby canopy and reduced the grass population to 6,780. The 24 m follow-up
retains 7,610 plant descriptors and 9,628 grass tufts in this particular near-eye
window, with 4,250 moss colonies and 6,692 pebbles unchanged. It is a correlated
placement step, not a proven ecosystem or aesthetic-parity result. Its source
mesh/image payloads are 33,543,212 / 28,930,048 bytes.

Luna separately reviewed the actual source diff and callers, reporting no
concrete correctness/contract defects; that review did not run tests or GPU work.
Root ran 305 worldgen library tests and 145 native binary tests (1/48 ignored),
Clippy for both crates with all targets/features and warnings denied, formatting
and the architecture gate. No production timing claim follows.

The new 0.3 m view confirms unresolved blurry, high-contrast soil and close
leaf intersection/coverage limitations. It does not certify soil quality. The
additional soil/interior/crown/100 m-range camera captures are single beauty
views; the standard sunset capture owns the 20 frozen diagnostic buffers. A
separately oriented backlit specimen and matched source-residency before/after
controls are still required for the complete five-camera rubric.

Item 1's placement criteria are now closed: canonical neighbor influence,
quality/source stability and cross-face halos pass tests and independent review.
Luna's v10/v11 inspection found no tall-tree density loss or new boundary artifact;
the 1,786 rejected clumps account for the intended lower understory density.
This does not close overall forest realism or certify a blind preference. See
[neighbor evidence](artifacts/forest-neighbor-communities-2026-11-04.json).

Item 2 remains open after the user's explicit dead-frond macro rejection. The
current scanned-organ candidate corrects clipped curl and collapsed shading
normals and adds a separate High PBR appearance view. Passing soil-visibility
controls does not accept its comb-like silhouette or establish resident-CBT
contact; see the item-specific review record.

The next item-2 pass requires complete visual inspection of every emitted image.
Real shared shadows and grazing cameras exposed the inherited living rachis arch;
the offline resting-organ deformation removes that arch while preserving pinna
folds. Complete variant/scene review and its remaining material/contact defects
are recorded in the item-specific visual manifest. Item 2 remains open.
