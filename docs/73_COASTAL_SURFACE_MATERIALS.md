# 73 — Coastal surface material eligibility

Status: implemented appearance-only correction; close-surface visual acceptance
and precise shoreline/sediment transport remain open. Baseline: `3d8698f`.

## Reproduced defect

The previous land material replaced every surface below 3 m with the same
yellow sand, fading the replacement by 45 m. A second height-only multiplier
darkened all ground between 0 and 3 m. Neither operation required a coast,
gentle ground or exposed sediment. Inland lowlands acquired a false beach;
basaltic and felsic low cliffs had exactly identical albedo.

The new regressions first failed against the baseline. Their original output
is retained locally in `target/test-tmp/world-coastal-material-before.log`.

## Existing sources and bounded appearance policy

The correction reuses `TerrainSample` climate, geology, salinity, slope,
ecological cover and geomorphology. It adds no height, noise, erosion,
hydrology or vegetation subsystem, and does not change authoritative state.

- Recover the regional ocean-distance proxy from continentality using the same
  `CONTINENTALITY_DISTANCE_M` normalization as the climate producer.
- Maritime eligibility is one within one meridional ocean-context spacing and
  fades smoothly to zero at two spacings. For the canonical 2-degree context,
  these distances are approximately 111.7 and 223.4 km. This is deliberately a
  coarse regional exclusion, **not** a local coastline-distance query.
- Retain the existing 3–45 m altitude fade only within eligible ground. Gentle
  slopes fade from 0.08 to 0.35 rise/run; exposed substrate suppresses sediment
  replacement continuously. Apply the coastal color to the bare substrate
  **before** the existing vegetation, wetland, rock, salt, snow and frost
  layers, so their partial opacity remains consistent with roughness/normals.
- Evaporite rejects the layer; salinity fades it out by 0.25. This preserves
  the existing playa/crust material instead of overwriting it with sand.
- Fine material uses the existing weathered-soil palette of the local geology,
  rather than imposing the same yellow on every coast. The existing filtered,
  world-addressed cover texture and dampening remain in use.
- The provisional 0–3 m, at-most-18% wet-rim darkening is multiplied by the same
  eligibility. It no longer darkens inland lowlands outside regional maritime
  support or exposed cliffs merely because they cross the ocean datum. Isolated
  near-datum inland ground inside that coarse support can still receive it.

These are bounded authored optical coverage priors, not a sediment inventory,
beach formation solver, measured capillary height or physical ecology model.
No quality tier changes the geographical predicate. The predicate has no tile,
camera, request-order or material-texel input. Existing physical-footprint
filtering still governs material texture bands; continuous eligibility does
not imply that every finite-resolution material mip is pixel-identical.

## Verification and limits

Tests cover inland datum invariance, exposed cliff geology separation, eligible
bare coast, partial vegetation/wetland substrate attenuation, full wetland and
exposure rejection, salt and slope gates, and continuity at the regional and
altitude fade endpoints. Independent review found an intermediate implementation
still repainted fractional cover; that implementation was rejected and a
failing regression added before moving the layer below the cover stack. The complete
worldgen suite also covers parent/child material edges and canonical land/water
publication. Numerical assertions are fixture-specific, not a physical coast
accuracy envelope: the coarse context has no metre-scale shoreline bound.

The submerged appearance branch, roughness model, optional normal strength,
canonical heights, frozen erosion and water classification are unchanged in
source. Freshly exported fallback maps consume the same correction as live pages.
Their manifest records `coastal_material = regional_maritime_lithology_substrate_v1`.
Rebaking also refreshes earlier material changes not yet present in the retained
maps: a roughness PNG change alone is not evidence that this coastal correction
changed the roughness algorithm. The normal map and regional world-layer
descriptors remain byte-identical in this export.
Performance checks concern page construction, not FPS or whole-client tails.

The 2048×1024 export was also rebuilt from exact baseline source `3d8698f`
in a separate target directory. That baseline-generated roughness is
byte-identical to the new roughness; 289,512 pixels of the old retained
roughness map were stale before this correction. Baseline-generated albedo
also differed from the retained old map at 52,319 pixels. Comparing fresh
baseline and fresh corrected albedo isolates this change to 45,848 pixels,
with mean absolute channel difference 0.77334/255 and maximum 120/255.
These are texture-difference diagnostics, not accuracy or preference scores.

Local verification passed workspace formatting, all-target/all-feature Clippy,
the architecture guard, 1,988 workspace lib/bin and 98 integration tests, plus
8 terrain-environment ignored hardware tests and a separate 8-test terrain
hardware target. The required legacy Bevy ignored command matched zero tests;
the real hardware coverage was in the renderer targets. The existing serial
512-page workload observed 65.267 s total, p50 117.241 ms and p95 203.359 ms;
baseline observations were collected under different concurrent load and do
not support a speedup claim. Exact logs, fallback comparison and limitations:
[`artifacts/coastal-surface-materials-2026-10-21.json`](artifacts/coastal-surface-materials-2026-10-21.json).

The [numeric WORLD00 survey](72_WORLD_SURFACE_BASELINE.md) subsequently located
an actual 2×2 km coastal site. Its entire 4,225-height grid, crossing and search
results remain exactly unchanged by this optical correction. At the fully
vegetated land target, albedo changes from the old sand
`[0.55516034, 0.53878385, 0.40395406]` to
`[0.12242220, 0.22971278, 0.13902363]`, while roughness remains 0.94 and all
reported physical/ecological inputs remain unchanged.

The [scatter identity foundation](70_WORLD_SCATTER_IDENTITY.md) is now integrated
for renderer-neutral descriptors and the existing offline consumer.
Remaining work: fixed 50 m/2 m AGL capture acceptance, finer runoff-derived relief,
native runtime scatter placement,
and production vegetation/rock consumers. Neither this correction nor a
preferred survey image establishes natural surface geometry or visual parity.
