# 50 — Thessa moisture and frozen erosion bridge

Status: 2026-10-02 checkpoint, superseded by the runtime continuation in doc 51;
uncommitted in `work/thessa-planet-rebuild`.
Docs 48–49 retain earlier audit snapshots. No main-checkout promotion, commit,
push, client-frame acceptance or KSA/KSP2 superiority claim is established.

## Shared causal contracts

`SpecRecipe -> Manifest -> PlanetParams/PlanetField` remains canonical.
`data/system.toml` remains the physical-body authority. The stellar proposal in
doc 13 is not applied.

- Ocean supply decays exponentially with physical travel distance and cumulative
  positive ascent. A shortest-attenuation-path spherical grid can go around
  mountains; descending into a basin does not restore condensed water. Continuous
  moisture is sampled periodically, without the previous random additive floor.
  `moisture_transport_m`, `rainout_height_m` and the previously ignored
  `high_cloud_wet_region_bias` reach the shared climate contract. This is an
  isotropic static proxy, not prevailing-wind/weather or precipitation physics.
- Closed basin geometry still comes from priority flood. Annual liquid inflow
  follows the existing strictly-downhill receiver/accumulation graph. Local
  supply and boundary inflow are counted once per basin. Salt suitability compares
  this inflow with potential evaporation at spill extent; persistent freezing
  excludes a liquid evaporative playa. The recipe specifies SI reference annual
  precipitation and evaporation; saturation-vapour-pressure scaling uses
  Clausius-Clapeyron. These are proxies, not actual water/salt inventories or
  an equilibrium shoreline solver. Flat/pit throughflow remains unresolved.
- Elevation no longer returns before authored geology is considered. An authored
  salt-basin shape alone supplies sedimentary basin geology, not evaporites.
  Dry ecology is considered before generic highland relief IDs. The dry-plateau
  complex now specifies a 2–3 km basin incision; climate responds to its geometry.
- The existing runoff erosion kernel transports cubic metres and deposits on
  the same spherical grid, including closed floors and ocean cells. The canonical
  offline recipe now represents 32 x 100,000 years. This is a landscape-evolution
  proxy, not a calibrated reconstruction of Thessa's geological age.

## Frozen surface and canonical sampling

The existing `bake-world` compiler emits schema 3 with the pre-erosion height
reference and a deterministic serialized source signature. Schema 2 remains
readable; it cannot be attached as an erosion delta without a source reference.
Completed bakes are never overwritten.

`PlanetField::with_frozen_erosion` checks the source, applies final-minus-source
displacement, and refreshes climate/wetland context and inhabited regions. The
source datum and fine analytic band amplitudes are preserved. Native mesh height,
contact queries, shared height prefixes and material samples use the same field.
The adapter does not execute erosion. Frozen drainage descriptors retain their
original resolution rather than being silently rerouted at 2 degrees; exports
report that resolution. Change the source-signature contract version whenever
the underlying analytic source semantics change.

At this checkpoint the normal launch setup still constructed the analytic source.
Doc 51 now records activation of the same bundled source-checked artifact on both
sides. Neither preserved fine analytic detail nor this adapter constitutes
detailed eroded river/lake geometry.

Frozen materials/ecology are recomputed from final height, moisture, wetland
support and saline suitability. Baked shore segments and plant prototypes/
instances are renderer-neutral data, not integrated water/vegetation rendering.

## Validation (2026-10-02)

Artifacts and logs are under `target/planet-biomes-audit/`. Earlier maps and
`offline-v1.surface.json.gz` were preserved, not regenerated. One new schema-3
bake, `erosion-bridge-v2.surface.json.gz`, contains a 720 x 361 regional DEM,
736 river reaches, 252 basins (one salt-suitable), 15,086 sea shore segments and
7,301 plant instances. Grid spacing is 0.5 degrees, about 28 km meridionally;
none of these counts proves metre-scale channel/shore quality.

The attached-field release CPU audit (`erosion-bridge-matched-release.json`)
samples 16,384 equal-area directions at 32 m cutoff. Observations: ocean area
53.8208%, mean proxy temperature 278.1161 K, 401 cold-desert, 286 stony-desert,
38 saline and 260 wetland samples. The population remains 150 million. The
sampled source-to-final displacement ranges from -4,316 to +3,473 m. The
whole-grid signed sediment residual is -8 m3, about 8.3e-16 of half the
area-weighted absolute displacement volume. This tests numerical conservation,
not geological accuracy or mass density assumptions.

One release compiler run took 5.563 s. One CPU audit observed 102.69 ms source
construction, 119.92 ms artifact read/validation and 193.34 ms attachment/context
refresh. These are separate single-run observations, not cold-cache loading,
frame timing, repeated benchmarks, or a renderer speedup. The earlier
`erosion-bridge-release.json` used coarser rerouted descriptors; the matched
audit above supersedes it without regenerating the completed bake.

178 worldgen library and 5 CLI tests pass; scoped all-target/all-feature clippy
passes with warnings denied. Ten ignored RCBT tests ran successfully on the
Radeon 880M, including real GPU paths. Regressions cover moisture attenuation,
seam/pole behavior, basin inflow counting and frozen playa rejection, rim erosion
and floor infill, source mismatch/double-application rejection, schema roundtrip,
population conservation and shared height/prefix consistency within 1e-8 m at
tested points. Full-workspace push gates are not claimed.

## Remaining acceptance work

The authored dry complex's eroded center is now a near-freezing steppe sample,
not a forced salt ID; the global bake retains one salt-suitable basin. Its exact
geographic suitability and erosion calibration remain to be evaluated. Do not
tune it by painting arbitrary salt/desert discs.

The correction grid is regional and preserves source fine detail; it cannot
prove destruction of every small crater. Local runoff channels, lake water
levels/shorelines, floodplains, deltas, wetland ponds, glacial evolution and
vegetation/infrastructure rendering remain incomplete. Native integration is
preserved, not visually accepted. Rectangular material-page transitions and
loading latency are still open in this snapshot. Doc 51 resolves the reported
`ReactionWheelAllocation::momentum_saturated` build blocker as shared-target
cache contamination, without changing physics code, and records native frames.
