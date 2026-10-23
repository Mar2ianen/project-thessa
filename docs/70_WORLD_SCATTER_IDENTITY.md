# 70 — World-addressed rock and plant scatter

**Status:** WORLD-01 foundation, WORLD-03 CPU canonical plant-surface adapter,
renderer-neutral descriptors and the existing offline plant consumer. WORLD04
adds a bounded native survey prototype consumer with shared graphics quality
policy ([doc 75](75_NATIVE_SURFACE_PLANTS.md)). No production foliage/rock
streaming, instance cache, collision ownership or visual acceptance is claimed.
The historical snapshots in docs 49–52 remain historical.

## Contract and consumer boundary

`tools/worldgen-rocky/src/scatter.rs::scatter_region` replaces the tile-seeded
placement kernel; there is no retained parallel legacy placement API. Repository
consumer inspection initially found only `offline.rs::bake_plants`, now using
this API. WORLD04 additionally uses the canonical callback from the bounded
native survey adapter in `apps/terrain-native/src/surface_plants.rs`.
Rock descriptors share that kernel but have no production rendering consumer.
Authoritative native terrain/appearance sources are unchanged. The WORLD-03 adapter
below supplies the existing kernel from an attached `PlanetField`; it does not
add a production instance-streaming consumer or replace the offline callback.

Inputs are world seed, datum radius in metres, an existing `lod::TileKey` query
window, rock/plant population, explicit policy and a pure local final-surface
callback. Valid windows use cube faces 0–5 and levels 0–27. The callback must not
depend on window, camera, density, invocation count or worker/request order.
It is called only for candidates passing conservative density rejection.

Output positions are body-fixed **unit directions plus signed ground elevation
in metres above the same spherical datum**, not tile-local metres. Physical
position is `(radius_m + ground_height_m) * direction`. Scale is dimensionless;
yaw is radians about radial up, relative to `sphere::enu_basis(direction)`.
Slope is dimensionless **rise/run**, not an angle or a normalized [0,1] value.
Water is an optional local datum elevation; ground at or below it is rejected.
Density is instances per **datum-sphere km²**, not per relief-surface km².
There is no authoritative collision/obstacle consumer in this slice. A future
renderer may thin visible instances without changing the world's collision
population; graphics quality must not create/remove physical obstacles.

Plant density describes clumps/crowns, not blades. The existing ecological
coverage proxy and prototype meshes are preserved, not promoted to plant
physiology or a botanical model. Rock kinds retain the existing biome/geology/
talus rules; the misleading old `slope01` name is removed.

## Version-1 fixed64 identity and ownership

Rocks and plants have one candidate per **fixed level-20 cube cell**. The
optional near-eye understory is a separate, explicitly versioned population with
one candidate per **fixed level-24 cube cell**. Both levels are world placement
contracts, never derived from requested density or render LOD. A candidate's
`ScatterId(u64)` packs

```text
(lod::cbt_node_for_tile(canonical_cell).id() << 1) | population_bit
population_bit: 0 = rocks, 1 = plants or understory
```

The shared tag is disambiguated by the validated node depth: level 20 is
plants, level 24 is understory. Their node-ID ranges cannot alias, so existing
rock/plant identities are unchanged. `ScatterId::cell` accepts exactly those two
depth/tag combinations and decodes the same existing address mechanism. This
extension is part of the placement contract; it is not a renderer LOD. IDs are
**world-local**: persist the owning world/body identity and placement-contract
version with them;
the seed alone is not a globally unique world identifier. Seed changes move
candidates but do not make the local address space globally unique. Changing
the fixed lattice/hash/channel contract requires an explicit placement version
and migration, not an unnoticed quality-setting change.

Existing stateless `rng::hash01(world_seed, channel, id, 0)` supplies independent
placement, acceptance, kind, scale and yaw channels. The ID, direction, kind,
scale and yaw of a surviving candidate do not change when only density scale or
the query partition changes. Terrain/ecological edits may intentionally alter
ground elevation, eligibility or kind; no persistence of obsolete ecology is
promised. Density nesting assumes unchanged local surface inputs.

Jitter is the midpoint of a 24-bit grid inside its canonical cell. Even the
existing hash's floating-point endpoint rounding cannot place it on an edge.
Every point is strictly inside one cube face and one half-open cell. Thus cube
seams/corners and the exact poles (cell edges) have no duplicate candidates;
there is no pole-specific longitude lattice or seam alias to reconcile. No
candidate is deliberately placed exactly on a face edge or pole. Queries finer
than the population's candidate level filter the same dyadic position through
level 27. Directions are
always evaluated from the canonical cell, never reconstructed from the query.

Successful batches are strictly ID-sorted. A parent batch is exactly the sorted
union of its four children, including complete descriptor values, for identical
sampler/policy. Overlapping query results must be unioned by ID. Offline baking
first removes duplicates/descendants covered by requested ancestors and sorts
the final descriptors, so overlap does not waste its union budget.

## Density, numerical and stochastic error

For canonical grid width `N = 2^20`, cube coordinates `(u,v)` and datum radius
`R`, the local physical Jacobian per candidate cell is

```text
A_km2(u,v) = (2R/N)^2 / (10^6 * (1 + u² + v²)^(3/2))
p_keep = local_density_per_km2 * density_scale * A_km2(u,v)
```

Acceptance compares a separate immutable hash against this probability.
Ignoring pseudo-random/hash and subcell quantization error, the expected count
is the spherical integral of local density, including local masks. This avoids
treating `TileKey::span_m().powi(2)` as spherical tile area: corner/centre cell
areas differ by up to `3^(3/2)`. Counts are stratified Bernoulli samples, **not
rounded targets, Poisson-disc placement or exact density quotas**. Under the
independent-uniform hash model, count variance is at most the expected count;
a useful conservative count scale is `sqrt(expected_count)`. This is a model of
pseudo-random sampling, not a proof of cryptographic/statistical independence.

At R = 3,200,000 m, the maximum cell Jacobian is 37.252903 m² and the supported
global level-20 density ceiling is **26,843.5456 instances/km²**. The
conservative population envelopes are `640 * density_scale` for rocks,
`3000 * density_scale` for plants and, at the reference spacing, up to
`1e6 / understory_spacing_m² * density_scale` for understory. The level-24
understory lattice has a much larger cell capacity; its requested density is
still checked against that lattice rather than assumed safe. Requests above
capacity return `DensityCapacityExceeded`, even if the eventual local mask would be empty;
there is no clipped probability or silent density saturation. In particular,
full-cover plants support scale up to approximately 8.94785 on this body. Larger
bodies have a lower ceiling; this finite address population is an explicit
foundation limit, not unbounded density support.

Relative to continuous within-cell jitter, quantization moves each cube-chart
coordinate by at most `2^-44`. Normalized cube projection is 1-Lipschitz in
these coordinates, so the angular displacement is at most approximately
`sqrt(2) * 2^-44` radians, or **0.258 micrometres** at this radius. The smooth
Jacobian's corresponding relative perturbation is below `1.3e-13`. These bounds
do not bound discontinuous habitat/water classifications arbitrarily close to
a boundary. The regression asserts unit-vector squared-norm error below
`1e-15`; platform-independent last-bit equality of libm operations is not
claimed. Same-build repeated/partitioned queries are exact descriptor matches.
Ground heights are the callback's f64 values, without storage quantization.

The 16-seed density regression uses an independent analytic spherical rectangle
area integral and bounds mean count error by `6*sqrt(expected/16)+1` instances:

| Window (face, L, x, y) | Area km² | Expected at 3000/km² | Observed mean | Envelope |
| --- | ---: | ---: | ---: | ---: |
| (0,12,2048,2048) | 2.441405668 | 7324.217 | 7316.500 | 129.372 |
| (0,12,0,0) | 0.470078340 | 1410.235 | 1408.625 | 57.330 |
| (2,12,4095,2048) | 0.863483585 | 2590.451 | 2588.438 | 77.345 |

This pins the tested fixture's absolute density error, not arbitrary world-wide
count accuracy or visual uniformity. There is no hard minimum separation across
canonical cells and no blue-noise claim.

## Budgets and offline final-surface scope

`ScatterPolicy` defaults to 16,777,216 candidate visits, 1,024,000 output items,
density scale 1, minimum plant elevation 2 m, maximum plant slope 0.25 and
`understory_spacing_m = 0.8` and understory clump scales 0.35–0.9. Candidate
work is exactly preflighted as
`4^(candidate_level-query_level)` for coarser/equal windows, or one overlapping
cell for finer windows; the latter may own no candidate.
`CandidateBudgetExceeded` is returned before sampling/allocation. Output-budget
exhaustion returns `ItemBudgetExceeded` and **no partial batch**. Invalid inputs
or sampled surface values also return errors. Zero density performs zero visits,
surface callbacks and instance allocations. Invalid requests still fail validation.
Large descriptor and accumulated offline-instance buffers use fallible,
geometric growth bounded by the item budget. Reservation failure returns
`AllocationFailed`; a capacity-overflow regression exercises this without large
allocations. The conversion temporarily retains a batch and accumulated output;
this is bounded storage, not zero-copy. This is not a process-wide no-OOM
guarantee for callbacks, serializers or unrelated library allocations.
Nonzero queries cost O(visited cells + output sorting), including at low density;
fixed-lattice enumeration is intentionally not claimed to be a runtime-optimal
LOD/streaming scheduler. There is no planet-global instance list or bake.

`OfflineRecipe.scatter` exposes the same policy in existing recipe serialization
and spends these budgets across the entire requested nonoverlapping tile union.
The existing 256-region/level-10–20/native-page constraints remain. The former
4,000-item per-tile truncation is gone; excessive work/output now errors.

For each sampled candidate, the existing offline consumer uses final
`FrozenSurface::height_m`, interpolated **final-bake ecological cover**, ocean
datum and non-saline lake descriptors, and a centred ±32 m final-height stencil.
The centred slope has O(32²) truncation on a smooth height field (component error
≤ `max_abs_third_derivative * 32² / 6`); no smoothness/error bound is asserted
across frozen DEM triangle edges. A known angular ramp pins slope 0.5 within
1e-6 in the equatorial fixture. Final-height sampling is checked exactly against
the stored instance height; constant-height interpolation can round by one ulp.

**Important limitation:** this offline surface is the frozen global triangulated
DEM, not the attached `PlanetField`'s preserved fine analytic bands or regional
erosion hierarchy (doc 52). Ecological interpolation is a stored final-bake proxy,
not a fresh high-resolution ecology solve. Lake membership retains the existing
nearest hydrology-cell descriptor boundary, not resolved shore/channel geometry;
rivers/wetland ponds have no resolved water mask here. The API can consume the
coherent attached-field sampler below without changing candidate IDs. The
offline callback deliberately retains its historical frozen-DEM behavior; it
does not evaluate unused analytic biome/geology metadata or override final cover.

New `PlantInstance.id` values are retained in the existing frozen wire format;
the optional additive field leaves historical schema-2/3/4 bakes readable with
`None`. Historical instance IDs are not fabricated from array indices. Addressed
instances validate their population/cell and strict unique ordering. Existing
bundled bakes are not regenerated or silently relabelled.

Independent review found that exact `total <= 1` validation rejected valid
interpolated cover. The stored producer tolerance is shared at `1e-12`; scatter
allows an additional eight f64 ulps at unity for interpolation arithmetic and
normalizes only that bounded overshoot before density/type selection. Negative,
nonfinite or materially overfull cover still errors. Mixed-cover and upper-bound
offline fixtures pin parent/child equality and nested subsets after normalization.
The complete bundled historical v3 artifact is loaded/validated without inventing
IDs, in addition to the historical individual-instance round trip.

After those review corrections, 216 library and 5 CLI tests and scoped
all-target/all-feature Clippy passed. `review-fixes-all-tests.log`,
`review-fixes-clippy.log`, and `review-fixes-bench.csv` retain the final evidence.
The final cheap-callback L10 plant diagnostic visited 1,048,576 candidates and
returned 117,401 items: median 33.336 ms, sample p95 34.108 ms over 15 iterations.
It ran under different concurrent load from the earlier 55.4 ms observation;
this is not a measured speedup, runtime canonical-field cost, or renderer result.

## WORLD-03: canonical plant-surface callback

`scatter::canonical_plant_surface(&PlanetField, direction)` is a narrow plant-only
adapter for `scatter_region`. It borrows the caller's field: production callers
already obtain the canonical `Arc<PlanetField>` from `TerrainStream::field()` or
`flight-authority::launch_site::canonical_world_field()`. There is no new loader,
provider, cache, streamer, or renderer. The adapter samples whichever field it
is given; attachment/source validation remains the existing field owner's job.
Both new public samplers (`canonical_plant_surface` and `water_height_m`) require
a **finite body-fixed unit direction**. The scatter kernel guarantees this
precondition; these are not fallible validation APIs for arbitrary vectors.

```rust,ignore
let field = terrain_stream.field(); // Existing source-checked canonical Arc.
let batch = scatter_region(
    field.params.seed,
    field.params.radius_m,
    region,
    ScatterPopulation::Plants,
    policy,
    |dir| canonical_plant_surface(field, dir),
)?;
```

The fixed contracts are intentionally distinct and never depend on query LOD,
camera, density, worker order, or mesh resolution:

| Input | Source and physical scale |
| --- | --- |
| Signed ground elevation, SI metres | `sample_surface(dir, PLANT_SURFACE_WAVELENGTH_M)` at **32 m** minimum wavelength; exactly the direct `height_m(dir, 32)` value, without height corrections or storage quantization |
| Placement slope, rise/run | `surface_geometry(dir, 32).0`, centred **±16 m** final-height stencil, including attached global/regional erosion and retained analytic relief |
| Ecological cover | `ecology::vegetation_cover` using that final 32 m semantic sample and **256 m** geometry slope, shared directly with the existing `lod::MATERIAL_SLOPE_WAVELENGTH_M` source of truth (±128 m stencil) |
| Water elevation, SI metres | `PlanetField::water_height_m(dir)`: ocean datum **0 m**, or the larger non-saline lake descriptor elevation at the nearest hydrology cell |

The 32 m constant is a placement contract, deliberately independent of texture
quality. `sample_surface` alone leaves `slope_hint = 0`; that value and the older
approximate `sample` slope hint are not placement geometry. Using the finer
placement slope for ecology would also change the material's existing cover
contract. The adapter evaluates no new `rocky01` noise: the existing
`ScatterSurface::plants` constructor supplies unused rock placeholders. It is
not a rock-surface adapter. The fixed-address kernel, IDs, policy, and historical
plant identifiers/wire compatibility are unchanged.

### Physical and numerical limits

The geometry API differentiates height against datum-sphere arc metres. With
half-stencil distance `d`, a smooth directional height function's central
derivative truncation is bounded by `max_abs_third_derivative * d² / 6` per
component (`d = 16 m` for placement; `128 m` for material). The slope magnitude
error is at most the norm of the component errors, before floating-point height
evaluation error. At nonzero elevation this is the existing datum-sphere metric,
not a new relief-surface arc-length correction. No universal third-derivative
bound is established across noise knots, DEM triangle edges, clamps, or regional
blend boundaries; there is **no global absolute physical slope-error guarantee**.
The WORLD00 landmark regression uses the existing 1e−5 m height tolerance and
1e−9 rise/run tolerance against its rounded documented slope; these are numerical
regression tolerances, not terrain/model accuracy bounds.

A 32 m cutoff preserves finer analytic relief relative to the frozen DEM, but
does not establish 32 m resolved erosion: bundled regional erosion is nominally
256 m. Nor does a sampled plant acquire a new collision height contract; its
stored ground is exactly the chosen canonical 32 m surface. Finer surface bands
may differ and are not corrected or bounded by this checkpoint.

The water accessor and offline wrapper share the **extracted, unchanged**
nearest-cell basin lookup: round latitude/longitude to a hydrology cell, wrap
longitude, clamp the pole row, ignore saline basins, and take the maximum above
the ocean datum. Dimensions come from the hydrology grid, not the 2° climate
context. The bundled attachment uses 0.5° cells (approximately 27.93 km meridional
spacing at R = 3,200,000 m; nearest-cell latitude support ±13.96 km), not metre-scale
lake geometry. Water elevation 0 on positive land is a comparison threshold,
not a wet/dry declaration. This is not a shoreline-position error envelope,
resolved river/wetland-pond mask, water-volume solve, or continuous coastal
distance field. Unsupported local water geometry remains a known gap.

Frozen validation accepts unsorted basin cells, while the retained lookup uses
binary search. `with_frozen_erosion` therefore sorts **only the field-owned cloned
cell vectors**, once at attachment. The accepted frozen input, validator,
extracted helper, and historical offline wrapper/output behavior are unchanged.
An accepted reversed-cell bundled fixture reproduced a silent miss at cell
`[17, 342]` (0 m instead of 413.0495928638749 m); the regression now matches the
sorted control at every cell in that lake and checks that the input stays reversed.

### CPU evidence and timing

Baseline: `20c3098d558bfbe9b232a0163813e9cdc52b31f0`, branch
`feat/canonical-plant-sampling`, exclusive worktree
`/home/chechulin/Projects/project-thessa-plant-sampling`. No frozen erosion
artifact or dated WORLD00 audit was modified. The focused tests reconstruct the
same bundled, source-checked attachment as the existing WORLD00 example and
launch-site loader, without a game-crate dependency or erosion rebake.

`scatter_runtime_tests.rs` verifies the existing WORLD00 land/ocean special
cases; separate placement/material slopes and cover consistency over 81 nearby
points; attached 0.5° lake membership/saline exclusion; non-vacuous rejection of
otherwise eligible plants in bundled fresh basin 241; and exact parent/reversed
children/repeat/density-subset descriptors and per-instance direct ground height
in `(face=1, L14, x=6018, y=11888)`. A shared-helper regression pins dry/ocean
fallback, salt exclusion, maximum level, nearest-cell membership, poles, and the
longitude seam. Existing offline scatter tests retain their original fixtures
and exercise the unchanged wrapper behavior.

Every Cargo command uses this worktree's **own** target:

```bash
TMPDIR=/home/chechulin/Projects/project-thessa/target/tmp \
  CARGO_TARGET_DIR="$PWD/target" \
  cargo test -p thessa-worldgen-rocky --lib --bins
TMPDIR=/home/chechulin/Projects/project-thessa/target/tmp \
  CARGO_TARGET_DIR="$PWD/target" \
  cargo clippy -p thessa-worldgen-rocky --all-targets --all-features -- -D warnings
```

Actually run: **229 library + 5 CLI tests passed**; the timing diagnostic is
ignored in normal tests and was subsequently run explicitly in release mode.
Scoped all-target/all-feature Clippy, changed-file
`rustfmt --edition 2024 --check`, `git diff --check`, the architecture metadata
gate (12 guarded crates), and the unmodified WORLD00 example regression
(`cargo test -p thessa-worldgen-rocky --example surface_patch_audit`, 1 passed)
also passed. Logs are in this worktree's `target/world03-evidence/`:
`worldgen-tests.log`, `clippy.log`, `architecture.log`, and
`world00-regression.log`. These are CPU/source gates, not whole-workspace CI.
`unsorted-lake-before-fix.log` records the failing accepted-input regression;
`unsorted-lake-after-fix.log` records its pass after field-only normalization.

An explicitly ignored CPU timing diagnostic reuses the same actual attached
field, sampling its WORLD00 parent window `(1, L12, 1504, 2972)` at density scales
0, 0.25, and 1. Nonzero cases visit 65,536 canonical cells. Attachment loading,
parent/child equality, and exact stored-height checks are outside timing; each
of 15 timed queries includes candidate enumeration, actual field/ecology/water
sampling, allocation, sorting, and drop. Output columns are scenario, density
scale, visited candidates, surface samples, items, median ms, nearest-rank p95
ms. **Do not run while the parent's native GPU gates are active.** Afterwards:

```bash
TMPDIR=/home/chechulin/Projects/project-thessa/target/tmp \
  CARGO_TARGET_DIR="$PWD/target" \
  cargo test --release -p thessa-worldgen-rocky --lib \
  scatter::runtime_tests::canonical_plant_batch_cost -- --exact --ignored --nocapture
```

After the parent's native GPU gates finished, the release diagnostic passed
on Linux x86-64, AMD Ryzen AI 9 H 365, rustc 1.99.1:

| Density scale | Candidates | Surface samples | Items | Median ms | Sample p95 ms |
| --- | --- | --- | --- | --- | --- |
| 0 | 0 | 0 | 0 | 0.000040 | 0.000100 |
| 0.25 | 65,536 | 1,303 | 910 | 32.435672 | 32.782649 |
| 1 | 65,536 | 5,166 | 3,752 | 81.865647 | 122.619487 |

Exact command/build output, counters, and timings are in
`target/world03-evidence/canonical-plants-cost.log`; hardware/compiler metadata
are in `cpu.txt` and `rustc.txt` there. This agent launched no other test/build job
alongside the timed run, but process affinity, thermals, and unrelated
machine activity were not controlled. The near-zero row is timer-scale overhead,
not a stable nanosecond performance budget. This is an actual canonical CPU
sampling cost diagnostic, **not a speedup comparison**, frame-budget acceptance,
renderer/visual/collision result, or whole-workspace CI. The earlier cheap-callback
timings used different surfaces/scenarios and are not a same-workload baseline.
README/status integration was synchronized by the parent; final combined
workspace/hardware checks are recorded separately from these isolated CPU gates.

The parent repeated the same command on integrated `67ff8b4` source before its
serial GPU gates: median/sample-p95 milliseconds were 0.000021/0.000060
(disabled), 21.467861/21.681038 (quarter density) and 80.976240/81.628916 (full
density), with identical candidate/sample/item counts. This repeat establishes
neither a stable speedup nor a tail bound; machine state was uncontrolled.
Both runs are recorded separately in the
[integration artifact](artifacts/canonical-plant-sampling-2026-10-21.json).

On the combined cloud-fix/CPU-sampler branch all **13 local command gates**
passed with **753 unchanged source/configuration/asset inputs**: workspace fmt,
all-target/all-feature Clippy, architecture, seven focused canonical runtime
regressions, explicit release timing, 2,011 lib/bin tests, 98 integration tests,
79 ignored renderer hardware tests, 43 ignored native hardware/CLI tests,
WORLD00 numeric regression, server build and native release build. The required
legacy Bevy ignored command matched zero tests; hardware coverage came from the
renderer/native targets. This is local evidence, not remote CI or visual
acceptance. Exact commands, durations, log/source hashes and omissions are in
the same artifact.

## Evidence and reproduction

Baseline: `3d8698f`, branch `feat/world-scatter-identity`, exclusive disk worktree
`/home/chechulin/Projects/project-thessa-world-scatter`. Evidence is in that
worktree's **`target/world-scatter-evidence/`**, not a shared target or `/tmp`.

- `baseline-regressions.log`: two pre-fix failures reproduced against the old
  kernel: density changes relocated survivors; a 12,000 request returned 4,000.
- `worldgen-tests.log`: 213 library and 5 CLI tests passed.
- `consumer-scatter-tests.log`: 13 tests passed with density diagnostics, local
  masks, cross-LOD/seam/pole tests and the real offline consumer regressions.
- `clippy.log`, `format.log`, `architecture.log`: scoped/static gate outputs.
- `scatter-bench.csv`, `release-build.log`, `cpu.txt`: CPU-only cost diagnostic.

Every Cargo command uses
`TMPDIR=/home/chechulin/Projects/project-thessa/target/tmp` and
`CARGO_TARGET_DIR=/home/chechulin/Projects/project-thessa-world-scatter/target`.
Run `cargo test -p thessa-worldgen-rocky --lib --bins` and
`cargo clippy -p thessa-worldgen-rocky --all-targets --all-features -- -D warnings`
with those environment assignments. No full-workspace/push gate or GPU test is
claimed for this CPU-only scope.

The integration owner wired the benchmark in the existing crate manifest.
Run `cargo bench -p thessa-worldgen-rocky --bench scatter` with the same
environment. The first diagnostic was compiled manually before this wiring;
the final `review-fixes-bench.csv` uses the actual Cargo benchmark target.

On the AMD Ryzen AI 9 H 365, 15-query samples including allocation/sorting/drop
observed full-cover plants at L12 centre: 65,536 visits, 7,335 items, median
3.124 ms / nearest-rank p95 4.434 ms; L10 centre: 1,048,576 visits, 117,401 items,
55.419 / 63.052 ms. The L10 25%-density subset had 29,396 items and median
28.412 ms. This cheap deterministic callback does **not** measure actual offline
field/ecology sampling, runtime foliage rendering, frame budgets or an old/new
speedup. Concurrent machine activity was not controlled. Parent/children equality
is asserted outside each timed scenario. GPU/backend and visual gates are not
applicable to this descriptor-only implementation.
