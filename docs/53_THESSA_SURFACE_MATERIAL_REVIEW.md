# 53 — Rejected surface frames and substrate/material continuation

Status: partial reconstruction, 2026-10-03, with native lighting follow-up below; prepared for publication at the
owner's request. Visual acceptance remains rejected/incomplete.

The user explicitly rejected the three v8 survey frames: the highlands were
only mediocre, the coastal green surface unacceptable, and the geothermal
ochre view especially poor. The erosion/snow changes in doc 52 are retained,
but neither those tests nor those images establish visual acceptance.

## What the rejected images actually sampled

`terrain_survey_audit` inspects the shared client/server field, not a parallel
preview model. At the original bookmarks, v9 reported:

| Bookmark | Substrate | Ecological biome | Height | Moisture |
| --- | --- | --- | --- | --- |
| Coast, 23° N / 165° E | Regolith | TemperateForest | 102 m | 0.911 |
| Highlands, 38° N / 171° W | ContinentalCrust | ColdDesert | 3263 m | 0.146 |
| Former `VOLCANIC`, 16° S / 159° E | Sedimentary | CanyonProvince | 1164 m | 0.317 |

The last bookmark was selected by geothermal flux (7.86 W/m²) and target
elevation, not volcanic geometry. A hot sedimentary canyon must not be
misrepresented as basalt to match its UI label. Its existing frozen refinement
is retained; it remains a real geothermal/canyon terrain region.

## Implemented material changes

The shared `surface_appearance_filtered` now uses geological substrate
reflectance for exposed stone and weathered loose soil separately. Basaltic,
felsic, continental, sedimentary, impact, evaporite, till, regolith and
hydrothermal substrates no longer collapse into a single dry ochre or grey
rock mixture. Elevation alone no longer overwrites every highland with grey.
These palettes are approximate optical/art-direction constants, not measured
spectral mineral composition or a weathering chemistry model.

Existing ecology's ground, canopy and reed weights select distinct optical
cover. A moist forest is no longer the same pure green mixture as grass and
reeds. Wetness darkens loose substrate; exposure and local deposition still
select rock versus cover through the existing geometry/displacement signals.
This is a filtered optical proxy, not canopy geometry, crown shadows or a
vegetation renderer. Changing paint cannot complete the missing forest scene.

Intermediate 384/192/96 m material variation supplements the existing fine
grain, with the same physical-texel band filtering. Signals are deterministic
in body space, independent of tile ordinals, and disappear when a texel cannot
resolve them. They modulate selected optical materials, not terrain geometry,
hydrology or authoritative contacts. They do not stand in for resolved strata,
scree, transported sediment layers or scanned material textures.

Land roughness now varies with ecological cover, exposure, wetness and frost.
The GPU fragment consumer reuses the existing bounded dielectric GGX kernel
and linear-lux lighting convention, with dielectric F0=0.04, instead of
ignoring roughness on all dry terrain. Ocean classification remains separate
from ocean-effect enablement: disabling the ocean effect does not turn the
ocean into shiny land. Up to three existing directional lights contribute;
zero-intensity lights skip the kernel. There is no new pass, large allocation,
plugin requirement or separate effect/settings pipeline. These are corrections
to the native terrain material consumer, using its existing quality/storage
resolution and disabled terrain behavior, not a new independently toggled effect.

The native page format remains sRGB RGB plus linear roughness alpha; shared
residency, ancestor blending, mip filtering and portable/microstore upload paths
are unchanged. Active fallback maps are `assets/worlds/thessa-v10`, exported
from the same v3 attached field/material evaluator. v8/v9 assets/captures remain
historical diagnostics rather than being overwritten.

`SurfaceAppearance` retains `snow` (climate suitability) and `snow_cover`
(finite supply proxy), and adds `frost_cover` for isolated frost verification.
Frost tests no longer assume every substrate has the former constant roughness
or falsely attribute moisture-dependent snow/soil changes entirely to frost.

## Survey correction and comparison boundary

The third survey now prefers a daylight above-water authored shield volcano or
volcanic province whose actual classified substrate is basaltic/hydrothermal.
Its shared client/server selection remains deterministic. Coast/highlands keep
parent-based scoring and final terrain spawn/contact queries. The fallback scan
exists for recipes without eligible volcanic candidates; the canonical recipe
must pass a substrate regression test.

Changing this bookmark does not alter the canonical terrain field, and does not
move/rebuild the frozen canyon refinement. The new volcano is not thereby
locally erosion-refined. Its images are a different-camera diagnostic, not a
matched before/after improvement of the rejected canyon. Highland/coast frames
can be compared at their original camera settings.

## Evidence and unresolved problems

### Native lighting follow-up — 2026-10-20

Native regular/conforming terrain now integrate the shared sky/cloud radiance
through the existing rough dielectric GGX material, with resolved 4/8/16
clear-sky directions, cheaper reflected-cloud columns and compile-out specular
disable. See [doc 69](69_NATIVE_TERRAIN_MATERIAL_LIGHTING.md)
for the numerical bounds, lifecycle and scoped GPU cost diagnostic. Native
terrain cast/receive shadows also now use the shared atlas (docs 60/63), rather
than remaining absent as in the historical reconstruction below. Optional RG8
primary-raster material normals now follow the native page hierarchy with
metric-correct gradients, filtered mip tiers and an explicit zero-work disable.
Their shared world-addressed grain carrier now has bounded substrate/exposure,
cover, moisture and snow/frost response plus small geology-linked roughness
priors. These are authored appearance priors, not calibrated lithology or
resolved strata/weathering relief.
The follow-up audit in doc 69 removes the artificial grazing sky floor,
evaluates the finite GGX visibility limit, and derives both incoming and
outgoing substrate throughput from that rough-material integral. It also excludes buried-rock roughness
priors under opaque cover and preserves unmasked coastal normal stencils.
Fresh survey captures no longer show the broad blue-grey grazing bands or
black ridge dashes seen in the preceding slice. One fresh model reviewer,
shown only unlabeled pairs, preferred an intermediate outgoing-only version in
all four cases after votes were frozen and scored. That version was still
rejected for directional energy creation and replaced by a two-sided,
first-substrate-bounce closure. The reviewer noted speckles, blurred/abrupt
material transitions and stretched detail; the earlier preference does not
establish final-patch visual acceptance or physical validation.
A fresh blind reviewer separately preferred the final two-sided version in all
four matched cases, while still identifying blurred/stretched materials,
angular contours, speckling and an artificial foreground boundary. This remains
one final-version model review, not broad acceptance or physical validation.
Natural material detail, vegetation, close-up geometry/transport continuity and independent
whole-frame visual acceptance remain incomplete. This does not accept the
rejected frames or change canonical terrain.

### Original reconstruction boundary

Tests pin geological contrast at identical physical state, bounded material
roughness, filtering of unresolved intermediate variation, climate suitability
versus snow supply, direct frost cover, and the shared dielectric kernel's
roughness-dependent specular lobe and back-facing-light rejection on hardware.
Successful tests alone cannot establish plausible materials or visual quality.
Runtime captures and final validation are saved under
`target/planet-biomes-audit/`; their reviewed outcomes are added below.

Still missing: calibrated lithology-specific relief such as strata, fractures,
scree and weathering, resolved vegetation/canopy geometry, spatially continuous
geological transitions and transported material inventories, proper fine
channels/shore surfaces and demonstrated close-up/surface-to-orbit acceptance.
Native primary terrain already has shared-atlas cast/receive shadows and a
material-normal shading path; neither alone repairs uniform shapes or flat
forest coverage.
The accepted standard is the actual final frame, not a noisy texture swatch,
an albedo histogram or a CPU-only material test.

## Publication checkpoint — 2026-10-03

At the owner's request, the reconstruction and earlier main-checkout renderer
changes are combined on the current main history. Existing vehicle/resource/
thermal/control changes remain byte-identical to the previously published
`c6673a6` main for their source, assets and canonical subsystem documentation.
Working-copy backups were saved before integration. Historical maps v4–v9,
frozen global v2 and refined v3, and the earlier audits remain preserved; v10 is
the active map set. Publishing this implementation does not accept the rejected
frames or assert that the material revision has passed final visual review.

Pre-push local gates on the combined tree passed:

- workspace formatting;
- workspace all-target/all-feature Clippy with warnings denied;
- workspace library/binary tests: 1569 passed, 14 existing diagnostics/hardware
  tests ignored in that default invocation;
- workspace integration tests: 4 passed;
- the explicit native terrain hardware run on the Radeon 880M: all 12 passed,
  including the shared dielectric roughness/reflection regression.

Logs are retained under `target/planet-biomes-audit/publication-gates-fixed/`.
Initial merge validation failed because two identical ocean-bound tests had
been carried by both working copies; the duplicates were removed without losing
either assertion set. The new stable-toolchain single-element-loop lint in
server asset lookup was fixed without changing lookup behavior. Earlier failed
logs remain under `publication-gates/` and the substrate audit filenames.
Local perf outputs and per-crate test scratch are ignored, not deleted; the
unrelated main-checkout scratch file `x` is excluded from publication.
