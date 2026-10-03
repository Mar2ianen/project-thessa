# 51 — Thessa runtime deployment and material geomorphology

Status: 2026-10-03 runtime/material checkpoint; superseded by doc 52 for the
frozen hierarchy/snow-cover model and by doc 53 for current materials/assets.
Isolated and uncommitted.
No main-checkout promotion, commit, push or KSA/KSP2 superiority claim.
Docs 48–50 retain earlier audit results; this document supersedes their runtime
deployment and build-blocker claims.

## Shared launch and artifacts

Client, local authority and headless server use `canonical_world_field` and the
same bundled `data/worldgen/thessa-erosion-v2.surface.json.gz`. These bytes were
copied from the completed doc-50 bake, not regenerated. Attachment validates the
source, seed and radius and fails explicitly on a mismatch. A process-wide
`OnceLock<Result<Arc<PlanetField>, String>>` shares the immutable field across
consumers. Filesystem cwd and external renderer settings cannot select another
authority surface.

Source identity compares parsed JSON values exactly, not object member ordering
or whitespace. Client dependency feature unification enables serde_json's
`preserve_order`, whereas the offline CLI normally sorts object keys. Numerical
parameters and array order remain exact; changed parameters still fail.
The new `FrozenSurface::read_gzip` stream loader retains the filesystem loader's
512 MiB decoded limit and full validation.

`export-client-maps --frozen <artifact>` exports maps from that same attached
field without rerunning erosion. Active assets are `assets/worlds/thessa-v7`;
earlier v4/v5/v6 assets remain untouched. The binary prefers packaged sibling
assets, then this build's source assets using an absolute path: direct launches
must not lose all textures merely because Cargo's runtime env is absent.
The frozen artifact's historical regional material/native-tile caches are not
used as current close-range material pages.

## Material representation and loading

- World-addressed GPU pages fade to shared resident ancestors beside missing
  neighbours, including diagonals and cube-face edges. Complete equal-resolution
  neighbourhoods keep detail all the way to their edges. Uncovered weight uses
  the matching globe map; no black/error material is blended in.
- All resident ancestors are explicit GPU demand, ordered coarse-first under
  slot exhaustion. CPU eviction uses the same hierarchy; shared fallback pages
  cannot merely be left in opportunistic cold slots.
- Cold material demand starts four levels above a requested close page (up to
  256 descendants share one build), then advances two levels at a time to the
  exact target. Height jobs never wait for material jobs. This is loading order,
  not a quality reduction or a change to terrain.
- Appearance noise is filtered at the physical texel spacing before page
  generation. Mips cannot repair grain that already aliased into the base page.
  Fixed-scale classification is preserved; filtered colours at different page
  levels are intentionally allowed to differ. Fully resolved shared samples
  and same-resolution page borders remain regression-tested.
- `terrain_material_missing` counts exact-page deficits in the effective
  material priority after merging view candidates with the live cover and
  applying the 512-page demand cap. It does not count excluded candidates or
  geometry leaves whose colour footprint can be different. Zero describes CPU
  page availability for scheduled demand, not proof of fully uploaded GPU slots
  or a satisfied visual error target.

## Geometry-correlated material contract

Material consumers use the existing field and appearance pipeline, not a new
relief generator. `surface_geometry(direction, scale_m)` samples the actual
canonical height, including landmarks and frozen displacement, on two tangent
axes. At distance `d = scale/2` it returns:

```text
gx = (h(+east) - h(-east)) / (2d)
gy = (h(+north) - h(-north)) / (2d)
slope = sqrt(gx² + gy²)
curvature = sum(h(neighbour) - h(center)) / d²    [1/m]
```

Material geometry scale is fixed at 256 m, independent of page/mesh LOD.
Positive curvature identifies hollows, negative curvature identifies crests.
`TerrainSample::erosion_displacement_m` retains the actual frozen regional
final-minus-source displacement. `curvature_per_m` is zero in ordinary samples
that have not requested the geometry stencil.

Steep/convex eroded ground favours exposed rock; gentle concave/depositional
ground favours loose cover and smoother small appearance bands. Snow suitability
still comes from climate. Visual cover can accumulate in hollows and thin over
exposed ground, with only a weak curvature-only exposure bias; the first v6
prototype stripped crests into dark contour bands and was rejected. No warm
region is reclassified as snow and no salt disc is
painted. These are appearance proxies, not snow/sediment inventories, a transport
solver, or proof of local scree deposition. Central finite differences have
second-order truncation on smooth height fields; the 256 m descriptor is not a
sub-metre curvature reconstruction, especially across the regional DEM's
piecewise-linear correction boundaries.

## Validation and visual limits

The former missing `momentum_saturated` error came from a shared Cargo target
containing main-checkout physics metadata. This worktree's actual structure has
no such field. An independent `target/isolated-build` compiles the real worktree
without touching physics. The initial runtime slice passed 179 worldgen library,
5 CLI, 77 client, 77 authority and 41 RCBT CPU tests. Twelve ignored RCBT hardware
tests passed on Radeon 880M, including residency-edge weights and native mesh
emission. Final v7 validation passed 182 worldgen library, 5 CLI, 77 client,
53 server, 77 authority and 41 RCBT CPU tests. All twelve ignored RCBT hardware
tests ran again and passed on this machine; the unrelated authority wall-clock
diagnostic remains ignored. Workspace formatting and scoped all-target/
all-feature clippy with warnings denied passed. Release client and server builds
passed. These are focused checks, not a claim that all full-workspace push gates
ran. Logs are `geomorphology-final-*.log` under the audit directory.

Native debug images exist under `target/planet-biomes-audit`. The valid initial
highland frame (`frozen-highlands-assets-debug/07-39s.png`) was explicitly rejected:
too uniform rounded middle relief, evenly distributed grain, overly white
materials and weak light/shadow structure. `frozen-highlands-debug` without Cargo's
asset env is invalid for visual/performance acceptance because its assets failed
to load; it diagnosed the standalone asset-path bug. The earlier `survey` run
selected map view, not a surface acceptance view. None of these is accepted as
proof that Thessa is finished.

The GPU indexed/mesh terrain currently receives stellar Lambert lighting and
ocean reflection but does not participate in Bevy's raster shadow caster/receiver
pipeline. Raising global shadow quality does not give this custom draw terrain
self-shadowing. Separate same-camera CPU/PBR shadow-on/off fixtures are under
`target/planet-biomes-audit/shadow-ab`; renderer changes must not be presented as
a pure shadow-only comparison. Timings must distinguish debug vs release,
serial material build benchmarks vs frames, and fresh runtime residency vs OS
filesystem caches. No measured speedup or strict cold-filesystem-cache result
is established by the loading-order change alone.

### Release diagnostic observations (v6 prototype, not accepted art)

Three serial fresh-runtime captures used the same paused highland camera,
25 km survey distance, 1270 x 1588 pixels, manual EV13, no cloud animation,
and local authority. Source assets resolved correctly without Cargo's runtime
environment. The CPU shadow-on/off fixtures differ only in shadow enablement;
both use a 12 km cascade cover, so conclusions do not extend to distant terrain
or other sun angles. Their 39 s images have RGB MAE 0.0081/255, maximum channel
difference 2/255 and 1.90% differing pixels. At this camera, enabling shadows
does not explain or fix the uniform landscape.
The ground-only crop `(0, 560)..(1270, 1540)` excludes sky, buttons and the
bottom caption: MAE remains 0.0118/255, max 2/255, with 2.73% differing pixels.
The tiny difference is therefore not solely dilution by the sky area.

| Renderer | Aggregate frame p50 / p95 | Terrain GPU scope p50 |
| --- | --- | --- |
| GPU indexed | 8.19 / 9.40 ms | 1.94 ms |
| CPU/PBR, shadows off | 5.65 / 7.09 ms | not separately reported |
| CPU/PBR, shadows on | 10.48 / 12.68 ms | not separately reported |

These are one-run aggregate release observations, not A/B renderer parity or
a speedup claim: CPU/GPU representation and material normal handling differ.
The v6 curvature exposure produced artificial dark contours and was rejected;
the v7 continuation limits curvature-only exposure while retaining slope-driven
outcrops and accumulation-dependent texture filtering. Artifacts and paired
state/capture metadata are under `target/planet-biomes-audit/shadow-ab`, including
`release-v6-report.json`. Earlier debug timings must not be pooled with them.

The new `material_pages` benchmark measures 512 serial 128² pages on six faces,
levels 10..14, with attached erosion and 256 m geometry descriptors. One v6
bench-profile run observed 60.550 s total, page p50 106.15 ms, p95 193.89 ms,
max 386.69 ms and 32 MiB of base-page payload. This is a CPU construction
workload, not a frame/loading benchmark or a claim that cold page generation is
free. The curvature cap changes appearance, not the five-point stencil cost.
Further caching/prebaking and same-workload measurements remain necessary for
loading acceptance.

### V7 frame review and telemetry correction

The matching GPU release frame at `shadow-ab/gpu-v7/shots/07-39s.png` has weaker
dark crest contours than v6, but still shows repeated rounded basin forms,
very pale cover and soft, weakly differentiated surface detail. It is not a
visually accepted reconstruction. Basin/saddle geometry is unchanged by these
material edits. The aggregate frame p50/p95 was 8.07/9.16 ms and terrain GPU
scope p50 was 1.84 ms in this single run; no speedup claim follows. Matching
metadata and observations are in `shadow-ab/release-v7-report.json`.

That historical v7 run ended with no material jobs but 43 reported missing
pages. Code inspection found that the counter counted pre-cap view candidates
while scheduling used a different merged, capped priority. The subsequent
counter correction uses that same effective priority, with a regression for
excluded/nonresident candidates and warm unrelated pages. It changes neither
the bounded demand nor page loading; old reports retain their original values
and cannot establish a queue stall or final material completion.
The initial follow-up test referenced a non-exported renderer constant and failed
to compile. Using the existing public worldgen page-size constant fixes the
test without changing the release path. All 78 client binary tests, client
all-target/all-feature clippy and workspace fmt then passed; the corrected
counter's release client build also passed. Both failed and passing logs remain
under `geomorphology-telemetry-*.log`, with passing reruns named
`geomorphology-telemetry-fixed-*.log`. No new visual or loading acceptance is
claimed from this telemetry-only correction.

## Remaining acceptance work

The user's middle-scale criterion is explicit: rare long structural breaks,
plateaus and pronounced connected valleys; differing ridge/slope/basin-floor
detail; geometry-correlated exposures, scree and pale accumulation; preserve the
connected basin/saddle regions that already suggest a geological history.
Existing arcs/rifts/plateaus must be reused/refined, not duplicated as random
cosmetic landmarks. The current material change does not alter middle-scale
height geometry and cannot complete local erosion by changing colour.
The canonical field still combines independent meso/micro bands and additional
4/8 km mountain detail (`field.rs::detail_parts_m`, `terrain.rs`); geometry-aware
appearance filtering does not spatially redistribute those height bands.

Local routed channels, equilibrium lake surfaces and shores, floodplains/deltas,
wetland ponds, glacial evolution, vegetation/infrastructure rendering, GPU shadow
parity and full surface-to-orbit visual/loading acceptance remain incomplete.
