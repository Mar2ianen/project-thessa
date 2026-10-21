# 70 — World-addressed rock and plant scatter

**Status:** WORLD-01 foundation, renderer-neutral descriptors and the existing
offline plant consumer. No production runtime foliage/rock renderer, streaming
instance cache, collision ownership, graphics quality integration or visual
acceptance is claimed. The historical snapshots in docs 49–52 remain historical.

## Contract and consumer boundary

`tools/worldgen-rocky/src/scatter.rs::scatter_region` replaces the tile-seeded
placement kernel; there is no retained parallel legacy placement API. Repository
consumer inspection found only `offline.rs::bake_plants`, now using this API.
Rock descriptors share that kernel but have no production rendering consumer.
Existing native terrain/appearance code is unchanged.

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

Each population has one candidate per **fixed level-20 cube cell**. This level
is a world placement contract, never derived from requested density or render
LOD. A candidate's `ScatterId(u64)` packs

```text
(lod::cbt_node_for_tile(canonical_cell).id() << 1) | population_bit
population_bit: 0 = rocks, 1 = plants
```

This reuses the existing cube face/Morton/CBT address mapping, is collision-free
within a world, and is not a probabilistic hash of a render tile. `ScatterId::cell`
validates/decodes the same existing address mechanism. IDs are **world-local**:
persist the owning world/body identity and placement-contract version with them;
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
than level 20 filter the same dyadic position through level 27. Directions are
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
global density ceiling is **26,843.5456 instances/km²**. The conservative
population envelopes are `640 * density_scale` for rocks and
`3000 * density_scale` for plants. Requests above capacity return
`DensityCapacityExceeded`, even if the eventual local mask would be empty;
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
density scale 1, minimum plant elevation 2 m and maximum plant slope 0.25.
Candidate work is exactly preflighted as `4^(20-query_level)` for level ≤20,
or one overlapping cell for finer windows; the latter may own no candidate.
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
rivers/wetland ponds have no resolved water mask here. The API can consume a
future coherent final runtime sampler without changing candidate IDs, but this
change does not claim that integration. The offline callback does not evaluate
unused analytic biome/geology metadata or use it to override final cover.

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
