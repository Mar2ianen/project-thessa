# 72 — WORLD00 canonical coastal surface baseline

Status: CPU-only numeric audit, 2026-10-21. This is a reproducible acceptance
**site**, not visual acceptance, a replacement world generator, or a universal
terrain accuracy gate. Baseline source: `3d8698f918d151fb2e94290d0b0af4611231471b`.

## Source and scenario contract

`tools/worldgen-rocky/examples/surface_patch_audit.rs` builds the exact same
recipe/manifest/field and source-checked bundled frozen attachment as
`flight-authority::launch_site::canonical_world_field`, without a dependency on
flight-authority. It samples the attached `PlanetField`, **not** the frozen
global DEM in isolation. No regional or global erosion rebake is performed.
The source seed is **7** and datum radius is **3,200,000 m**.

External `sha256sum` of the unmodified source artifacts:

| Artifact under `data/worldgen/` | SHA-256 |
| --- | --- |
| `worldgen_recipe.toml` | `e4687f25559fbddb81b58536ec26397f1e0f1ec45a27a9d46b2d1e4f396f4048` |
| `thessa-erosion-v3.surface.json.gz` | `04ce20649b1d8810ea917bf705d763631beebdedca01b3fcd3fc337665463332` |
| `thessa-regional-erosion.toml` | `685d080d90f67274e6950e79d95b47e5b360b70f3026cf51bd5b4d636fe3c937` |

The fixed scenario is `WORLD00-coastal-2km-v1`. Starting at the old coastal
bookmark (23° N, 165° E), search every fourth vertex of the bundled
`humid-coastal-lowland` region: 129 × 129 vertices, nominal 1024 m spacing,
131,072 m nominal width. Evaluate **final height with a 32 m minimum wavelength**
at each vertex, detect opposite signs on axial neighbor edges, and bisect each
edge. Bisection retains submerged/nonnegative endpoint signs and requires both
a ≤1 mm final bracket and a ≤1 micrometre midpoint height residual. These are
root-location tolerances, not terrain/model error bounds.

Select the bisected crossing nearest the old bookmark; stable row-major,
east-edge-before-north-edge traversal breaks equal-distance ties. This is the
nearest **detected grid-edge** crossing, not a continuous nearest-coastline
certificate. Narrow islands/inlets between grid samples can be missed. If the
region has no crossing, the example fails rather than inventing an ocean
distance or reusing a lowland score as a shoreline. The final patch must contain
both submerged and positive-height samples and a nearby land camera target.

The patch is exactly 2000 × 2000 **ENU coordinate metres**, centered on the
selected crossing via existing `sphere::enu_to_dir`, with 65 × 65 vertices and
31.25 m spacing (64 cells). Projection onto the sphere makes physical arc
distances infinitesimally different from tangent-coordinate distances. Heights
and slopes use a 32 m field cutoff; slopes reuse `surface_geometry(dir, 32)`
(central ±16 m stencil of final height), not the approximate `sample` slope
hint. Quantiles are ordered sampled values at integer ranks, not an interpolated
statistical estimator. Submerged fraction is a vertex fraction, not integrated
water area. The JSON includes the complete height grid and geographic corners.

The land target is the nearest row-major patch vertex with height ≥2 m and ENU
distance 128–1000 m from the crossing. This is camera framing, not a flat pad,
vehicle spawn, collision-clearance, hydrology, or safe landing certificate.
Climate/geology come from the final 32 m sample. Optical cover uses the existing
256 m material geometry stencil, reported separately from the 32 m slope.

## Measured WORLD00 site

On Linux x86-64, AMD Ryzen AI 9 H 365, rustc 1.99.1, generic release build:

| Quantity | Value |
| --- | --- |
| Old bookmark final height | +102.021085122 m; **not a coastline** |
| Search final-height range | −621.360380 .. +384.566323 m |
| Opposite-sign grid edges bisected | 954 |
| Selected crossing latitude, east longitude | **23.567917375616958°, 165.13942625682847°** |
| Distance from old bookmark to selected crossing | 32,514.995916 m; not certified continuous ocean distance |
| Crossing midpoint height residual | −0.000000621883391 m |
| Final bracket width / bisections | 0.000244140582 m / 22 |
| Patch height min / max | −10.742522751 / +17.731800420 m |
| Patch height p10 / p50 / p90 / p99 | −1.461894490 / 4.821779309 / 9.034812331 / 14.877760423 m |
| Submerged vertices / all vertices | 566 / 4225 = 0.1339644970414201 |
| All-vertex slope min / p10 / p50 / p90 / p99 / max, rise/run | 0.000069529 / 0.003732745 / 0.020823390 / 0.077477204 / 0.119388919 / 0.197472167 |
| Land-only slope min / p10 / p50 / p90 / p99 / max, rise/run | 0.000069529 / 0.003447588 / 0.017291656 / 0.072298880 / 0.117171379 / 0.197472167 |

The original neighboring grid points really bracket zero:

| Endpoint latitude | East longitude | Final 32 m-cutoff height |
| --- | --- | --- |
| 23.55003948332559° | 165.13942625682847° | +12.187490396448244 m |
| 23.568374132769776° | 165.13942625682847° | −2.0759892266078985 m |

The final retained sign bracket is:

| Endpoint latitude | East longitude | Final height |
| --- | --- | --- |
| 23.567917377802615° | 165.13942625682847° | −0.000013482709880108246 m |
| 23.567917373431296° | 165.13942625682847° | +0.000012238988494672753 m |

**Land camera target for the native CLI integration:** latitude
**23.565679258039204°**, east longitude **165.13881581863882°**, canonical height
**4.557292563542587 m**. The target is ENU `(−31.25, −125, 0)` m from the crossing,
128.847050731 m great-circle distance. Zero ENU up here is a direction-mapping
input, not terrain height or camera altitude. Set camera clearance separately.

At that land target: `TemperateForest` biome, `Regolith` geology, 283.665083412 K,
moisture 0.961782617, wetland suitability 0.123430042, geothermal flux
6.953827692e−6 W/m², total frozen displacement +123.924726585 m, fine-region
positive deposition proxy 8.091750418 m. Final 32 m slope is 0.002384743 rise/run;
the 256 m material slope is 0.016568885. Existing optical state is vegetation
1.0, snow suitability/cover 0, frost 0, roughness 0.94, unlit sRGB albedo
`[0.555160344, 0.538783848, 0.403954059]`. These are existing material outputs,
not proof that tree geometry is rendered or that shoreline cover is correct.

## Reproduction and artifacts

Run from the worktree root; all scratch stays under crate `target/test-tmp`.
Every Cargo invocation on this machine must set the indicated `TMPDIR`.

```bash
mkdir -p tools/worldgen-rocky/target/test-tmp/world00
TMPDIR=/home/chechulin/Projects/project-thessa/target/tmp \
  CARGO_TARGET_DIR="$PWD/target" \
  cargo run --release -p thessa-worldgen-rocky --example surface_patch_audit \
  > tools/worldgen-rocky/target/test-tmp/world00/surface-patch-audit.json
sha256sum data/worldgen/worldgen_recipe.toml \
  data/worldgen/thessa-erosion-v3.surface.json.gz \
  data/worldgen/thessa-regional-erosion.toml
TMPDIR=/home/chechulin/Projects/project-thessa/target/tmp \
  CARGO_TARGET_DIR="$PWD/target" \
  cargo bench -p thessa-worldgen-rocky --bench material_pages \
  > tools/worldgen-rocky/target/test-tmp/world00/material-pages-baseline.jsonl
TMPDIR=/home/chechulin/Projects/project-thessa/target/tmp \
  CARGO_TARGET_DIR="$PWD/target" \
  cargo test --release -p thessa-worldgen-rocky --example surface_patch_audit
```

Two independent example process runs produced byte-identical JSON (including
the entire 4225-height grid), SHA-256
`cd4d3246ab340ac560fea5e31fcd9c6e408b2110dbd837acfae5109733416029`.
`surface-patch-audit.json` and `surface-patch-audit-repeat.json` are retained in
`tools/worldgen-rocky/target/test-tmp/world00/` in the baseline worktree. The
example's numeric regression pins this source/site: 954 crossing edges, crossing
coordinates within 1e−8 degrees, bracket signs and tolerances, exact submerged
vertex count, height extrema/land height within 1e−5 m, and land target/class.
Those tolerances accommodate last-ulp platform differences; they are not an
error envelope for the physical model. Same-machine byte identity is not a
cross-platform bit-identity promise.

The unmodified `material_pages` benchmark is a serial CPU page-generation cost
baseline: 512 broad pages plus 64 close-land paired normal-off/normal-on pages
with counterbalanced order. It is **not** a coastal scene, frame-time, renderer,
streaming, or loading benchmark. Reuse the same executable/workload and
counterbalance before/after process order under comparable machine load for
future appearance comparisons; one concurrently loaded run proves no speedup.

One measured broad baseline processed 512 pages in **91.910892647 s**: page
p50 **166.340511 ms**, p95 **325.375060 ms**, maximum **517.728195 ms**;
50,331,648 payload bytes (33,554,432 albedo + 16,777,216 normal). Measurements
were collected while other work could compile/run on the machine, with no
exclusive CPU scheduling or thermal/load control. They are a recorded comparison
input, not a performance budget or improvement claim.

The same completed run's 64 paired close-land pages measured:

| Normal maps | Total sampling time | Page p50 | Page p95 | Page maximum |
| --- | --- | --- | --- | --- |
| Off | 9.070859018 s | 133.902535 ms | 186.299964 ms | 223.239322 ms |
| On | 9.548782764 s | 139.218182 ms | 220.165850 ms | 245.242159 ms |

All three emitted records are retained in
`tools/worldgen-rocky/target/test-tmp/world00/material-pages-baseline.jsonl`.

The built baseline executable is
`/home/chechulin/Projects/project-thessa-world-baseline/target/release/deps/material_pages-eadc2a788c66387e`.
The initial Cargo benchmark attempt exhausted its 120 s timeout just after
compilation/startup, before emitting measurements; the executable was then run
directly without a timeout. Build/startup log:
`tools/worldgen-rocky/target/test-tmp/world00/material-pages-baseline-build.log`.
To repeat without compilation, execute that exact path from the worktree root
and redirect stdout to a **new** JSONL artifact (retain the baseline); the
benchmark itself counterbalances normal-off/on order internally.

Verification actually run: the release example twice (`cmp` identical), the
release example regression (**1 passed**, 0 ignored; 0.97 s test execution),
`cargo clippy --release -p thessa-worldgen-rocky --example surface_patch_audit --
-D warnings`, `rustfmt --edition 2024 --check` on the example, and
`git diff --check`. Cargo used the explicit machine `TMPDIR` and this worktree's
own `target/`. Test/build logs are alongside the JSON artifacts. Whole-workspace
tests, CI, architecture metadata gate, and GPU tests were not run for this
example/document-only slice.

## Scope and limitations

- No physics, geometry, recipe, frozen artifact, renderer, material, or settings
  behavior is changed. Existing README and status claims about partial worldgen
  and incomplete visual acceptance remain accurate. This isolated worktree owns
  only the example and this document; living-index integration is parent-owned.
- The existing regional erosion is nominally 256 m; 32 m samples retain finer
  analytic relief and do not establish 32 m resolved erosion or sediment
  inventory. Coastline means the height-zero contour at the stated cutoff,
  not simulated tides, wave runup, or resolved coastal water geometry.
- No continuous ocean-distance field, drainage/channel reconstruction, coastal
  geology correction, global nearest-shore proof, universal geometric error
  envelope, or patch convergence study is provided.
- No GPU work, screenshots, blind preference review, surface-to-orbit visual
  acceptance, or whole-workspace CI claim is part of WORLD00.
