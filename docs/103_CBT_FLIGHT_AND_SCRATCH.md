# 103 — Production CBT flight replay and inactive scratch traffic

Status: a fixed-source production ground→orbit→ground replay and bounded-pool
recovery control pass. Repeated inactive-slot clearing is removed from
land-only classification scratch. Final published geometry remains fully
cleared. P01 is partial: dense equivalent-error, streaming, complete consumer
cost, astronomical frames and visual acceptance remain open.

## Verify the production path

`production_ground_orbit_ground_preserves_closed_canonical_surface` uses the
actual `TerrainRenderer` classifier, transaction waves, canonical emission and
indirect raster. It does not replace them with the sphere-prototype draw.
The topology budget matches High: 131,072 slots, four waves, 64/8 px² split/
merge thresholds. The body radius is 3,200 km, viewport 129×129. Six frozen
17×17 root height pages describe smooth relief, quantized with a 1 mm budget.
They remain byte-identical throughout the replay and both A/B variants.

The 224 frames contain 64 initial settling frames, a 96-frame continuous
camera path from 2 m to approximately 6,387 km altitude and back with lateral
motion, then 64 return settling frames. The camera remains body-fixed/f64;
these are camera-local origin changes, not an astronomical frame replay.

The new `TerrainRenderer::read_topology_records` is an explicit diagnostic
snapshot of the existing GPU pool. It allocates/maps only when requested;
ordinary submission remains asynchronous and performs no pool readback. The
existing independent geometric oracle is shared with `cbt_sphere` tests.

For **every** replay frame the test checks reversed integer edges and exact
neighbor links, full root coverage and Euler characteristic two. No open edge,
T-junction or missing root is permitted. Draw membership/count matches the
pool, including the four reserved degenerate virtual-root slots. Sixteen
geometry snapshots additionally check the shared canonical page sampler,
bit-identical duplicate vertices and zeroed inactive published slots, including
the fields consumed by shadow/RT paths.

On Radeon 880M/RADV, maximum vertex-coordinate error against the f64 projection
of the **same decoded pages** is 0.001082 m within 10 km of the camera, and
0.672823 m across the whole body-relative stream. Guards remain 0.02/8 m.
This is arithmetic/representation accuracy, not a tessellation, analytic-height
or close-up material error bound. Software Vulkan also passes the same guards, with maxima 0.001305/1.095006 m.
Both settled ground endpoints contain 1,846
geometric triangles, have no pending requests/commits and reproduce the same
RGBA8 raster byte-for-byte. Epochs advance from 39 to 165 during flight; four
additional unchanged frames retain the settled publication.

A separate control uses the **same height pages**, a 4,096-slot pool and
513×513 viewport. After 96 close frames it has rejected allocations but still
reports pending requests, while its mesh remains closed. After 96 far frames
it reaches completed zero requests/commits; a subsequent close view wakes it
again. Cumulative rejection counts remain diagnostic history, not a convergence
test. Closedness is checked at every overload/recovery/wakeup frame as well.
This deliberately insufficient budget is not an equal-error speed comparison.

## Reduce work without changing the surface

Each transaction wave emits temporary geometry for classification. Previously
it also rewrote all free/virtual slots to zero, although the land classifier
needs only live bisectors. At High capacity, most slots in this fixture are
free. Repeating their 144-byte clears across four waves wastes bandwidth.

The scratch entry point now emits live geometry without clearing inactive
slots. The classifier checks pool liveness before reading scratch vertices,
so a freed slot cannot retain a stale request or participate in parent
hysteresis. The final publication uses the complete emitter and still zeros
every inactive slot for raster, shadows and the full-capacity RT stream.
Live vertices, canonical sampling, scores, topology policy, waves and request
ordering are unchanged.

Ocean admission scans scratch vertices to derive a shared geometry footprint
and budget. When conforming ocean geometry is present, scratch retains the
original complete clear/emission path. This is an internal representation
optimization, with no new graphics toggle, source/height change or reduction
of physically caused wave geometry. An explicit shared bind-group layout
supports both compute entry points; automatic exclusive layouts cannot be
shared. There is one additional compute pipeline, no additional GPU buffer.
Topology-disabled paths still allocate no conforming surface resources.

## Same-work release ABBA (2026-10-31)

Immutable old/new binaries use identical test code and inputs. The baseline
retains the original full emitter/classifier. No compiler runs during timed
replays, and no clock/driver setting changes. GPU intervals separate topology/
classification/height emission from terrain raster; pool readbacks, CPU
oracles, endpoint readbacks, initialization and pipeline creation are outside
the intervals. Per-frame CSV is retained. The continuous-flight phase follows
64 initial frames and supplies 96 samples per run. Reported tail percentiles
use the lower empirical quantile, sorted index `floor((n-1)*q)`.

| High-budget run | Topology + emission p50, ms | p95, ms | p99, ms | Raster p50, ms |
| --- | ---: | ---: | ---: | ---: |
| A, original | 6.490 | 6.959 | 7.171 | 0.0302 |
| B, optimized | 3.270 | 3.942 | 4.209 | 0.0308 |
| B, optimized | 2.937 | 3.428 | 3.716 | 0.0301 |
| A, original | 6.410 | 6.888 | 6.982 | 0.0300 |

This fixture reduces topology/emission p50 by approximately 49–55%. The
smaller 16K-capacity control also improves: 1.85–1.90 ms to 1.48–1.49 ms.
The exact frozen pages, **all per-frame physical topology/counter/epoch rows**,
coordinate-error summaries, both endpoint rasters and overload/recovery
records match byte-for-byte across all ABBA runs. No equal-quality claim is
inferred merely from lower triangle counts; the counts are unchanged.

High's tracked GPU buffer storage remains 24,867,116 bytes in both variants;
driver memory for the additional pipeline is not included. These are scoped
GPU results for a sparse, frozen six-page surface. They do not establish
full-client FPS, a coastal/ocean speedup, streaming cost or performance parity
with [Large CBT](https://arxiv.org/html/2407.02215v1). Its allocation/triangulation
ideas guide the independent Rust/WGSL implementation; upstream unlicensed
renderer code is not copied.

## Evidence and remaining acceptance

[Machine-readable evidence](artifacts/cbt-flight-scratch-2026-10-31.json) binds
source/binary identities, fields, commands, raw CSV, actual checks and limits.
External directories `cbt-flight` and `cbt-flight-high` retain the snapshots,
replays and ABBA binaries. Existing canonical precision/page lifecycle,
conforming ocean, shared shadows and RT publication regressions are included
in the final verification; software Vulkan repeats the production flight
control. Exact commands/results are in the record, rather than inferred CI.

The existing RT regression had a stale construction-capacity assumption: its
current selection has one mesh pairing, so a budget of three correctly admits
both terrain chunks. Original and optimized emitters both failed the old
assertion. The test now pins the three actual live instances, then uses a
budget of two to test whole-partition fallback. Runtime RT admission is unchanged.

Still open: dense canonical tessellation/error comparisons (three midpoint
probes are not a universal bound), dynamic multi-level residency, flight over
production soil, astronomical/body-frame transformations, shadow/ocean/RT
whole-system p50/p95/p99, cross-engine rendering and independent visual votes.
This closes a fixed-source continuity/recovery slice of P01 and a measured
land-only update optimization, not the complete graphics goal.
