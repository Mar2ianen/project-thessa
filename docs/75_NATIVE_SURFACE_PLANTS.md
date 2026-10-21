# 75 — Native surface prototype plants (policy draft)

**Status:** prepared graphics policy only, awaiting the native consumer in the
same vertical slice. This configuration change must not ship alone as a claimed
plant renderer. It does not establish vegetation realism, collision ownership,
streaming, or visual acceptance. README/status integration belongs to the native
consumer integration change; existing incomplete-vegetation claims remain valid.

## Requested and resolved contract

`[surface_plants]` in `graphics.toml` has `enabled = false` and
`quality = "high"` by default. Missing/partial sections retain these defaults.
Quality uses the existing `Quality` enum; invalid values fail TOML parsing.
Unknown keys retain the shared parser's forward-compatible behavior.

`RequestedGraphics.surface_plants: SurfacePlantsSettings` enters the existing
`ResolvedGraphicsSettings::from_requested` pipeline. It resolves flat fields:

| Field | Type | Disabled | Low | Medium | High |
| --- | --- | --- | --- | --- | --- |
| `surface_plants_enabled` | `bool` | false | true | true | true |
| `surface_plants_density_scale` | `f64` | 0 | 0.25 | 0.5 | 1 |
| `surface_plants_max_candidates` | `u64` | 0 | 4096 | 4096 | 4096 |
| `surface_plants_max_items` | `u32` | 0 | 4096 | 4096 | 4096 |

Counts follow existing resolved count conventions. The consumer must perform a
checked conversion of `max_items` if its scatter API accepts `usize`. Density
remains `f64` for the scatter API. These fields are also the exact flat capture
metadata keys. Older resolved TOML without these fields deserializes as disabled
with zero density/budgets.

Low/Medium/High/Ultra presets set quality to Low/Medium/High/High, preserving
explicit enabled/disabled intent (as for ocean and topology). Custom leaves
plant policy untouched. Explicit quality overrides remain intact when parsing;
divergence from an applied preset reports custom. Both GUI descriptors require
restart for the initial retained survey representation.

## Bounded consumer boundary

The agreed initial native consumer is **survey-only**: one fixed level-14 query
window around the numeric survey target. The level-20 identity lattice visits
`4^(20-14) = 4096` candidate cells; the worst-case output fits the 4096-item
ceiling. All tiers traverse the same window, not successively larger regions.
Lower density is a nested rendering subset, not a promise of proportional CPU
sampling cost. IDs, habitat, final ground height, kind and scale must not be
derived from quality; reuse the existing scatter identity/eligibility contract
(doc 70) and canonical surface sampling rather than a parallel placement model.

The disabled consumer must return before any surface sampling, jobs, instance
buffers or mesh preparation. Resolved budgets are ceilings, not allocations;
enabling policy alone must not reserve 4096 items or create GPU resources.
No new device capability is required by the settings policy. It neither requests
RT nor changes shadow/global render architecture, terrain topology or physics.
The prototype's disabled/Low states are its explicit cheaper fallbacks.

## Verification and integration still required

The settings tests cover defaults, partial sections, all tiers, disable,
serialization/legacy resolution, preset intent/overrides, GUI/capture metadata
and isolation from every unrelated resolved field. These are CPU policy tests,
not evidence of renderer correctness, GPU cost, botanical plausibility or a
performance win.

The integration owner must wire the native consumer before shipping, update
current status/feature indexes, and record actual survey target/world revision,
candidate visits, sampler calls, accepted/retained/drawn counts, CPU preparation
cost and GPU bytes/draw cost. Disabled should report zero plant work. Native
renderer/oracle tests, same-scenario cost measurements and independent visual
review remain integration gates, not completed by this draft.
