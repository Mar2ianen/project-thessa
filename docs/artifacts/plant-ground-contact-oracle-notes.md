# Fixed regular GPU patch: canonical plant-root contact diagnostic

Status: diagnostic implemented and parent hardware measurement completed. This is
not foliage rendering, viewer resident-cover acceptance, or a root-accuracy
guarantee. Baseline: `66278ace92d89b01faec089b6feee553ae4f039c`, isolated branch
`test/native-plant-ground-contact` in `project-thessa-plant-ground`.

## Scope and source

`apps/terrain-native/tests/plant_ground_contact.rs` uses the existing public
`flight_authority::canonical_world_field()` source-checked bundled canonical
field. It enumerates the full-density canonical plant descriptors in exactly
`(face=1, L14, x=6018, y=11888)`: 4,096 candidate visits, without output
truncation. Every returned descriptor must retain exactly the direct
`field.height_m(direction, 32)` ground elevation. No descriptor offset, snap,
new placement mechanism, or world-source fallback is introduced.

It bakes one page with `lod::bake_height_page(field, window, 33, 0.5)`, the same
producer and arguments as `terrain_stream::height::bake_page`. At this L14
window the source wavelength is `max(tile_span/32, 32) = 32 m`; source samples
are clamped to the datum as in the existing native height path. Plant placement
slope is the existing 32 m geometry contract (centred +/-16 m stencil), ecology
uses the existing 256 m material geometry, and the default plant policy remains
minimum elevation 2 m / maximum slope 0.25 rise/run / density scale 1.

The actual `TerrainRenderer` geometry compute consumes the compact page. The
test updates a body-fixed camera, encodes and submits the regular
classification/raster pass, then reads draw state and **generated GPU vertex
storage** through `read_vertices`. A draw-count assertion requires full step-1
resolution: 2,048 surface triangles plus 256 skirt triangles. This fixture
therefore cannot silently measure a decimated grid while assuming step 1.

## Independent reconstruction and intersection

The inspected ABI and topology are in `crates/rcbt-core/src/shaders.rs`:

- `build_geometry` writes a row-major 33x33 grid; each vertex is eight f32s:
  local XYZ / 1, normal XYZ / decoded height.
- `precision.wgsl::cbt_local_offset` already expresses its result in
  **body-fixed Cartesian axes**, using `TileAnchor`'s cube-face derivative
  vectors and radial normal. These are not ENU components. Independent f64
  reconstruction is `anchor.anchor_body_m + f64(local_xyz)`, with **no extra
  basis rotation**. `tile_frame.wgsl::cbt_render_position` confirms that the
  renderer adds the anchor to this offset.
- `classify_active` emits `(a,b,c)` and `(b,d,c)` for each step-1 cell, with
  `a=y*33+x`, `b=a+1`, `c=a+33`, `d=c+1`. The diagonal is **b-c**, not a-d.
  The independent oracle reconstructs these triangle indices explicitly.

Indices are source-traced at the baseline, not read back from the private
active-triangle buffer; GPU readback supplies the actual vertex positions and
draw count. No public API change is needed for this fixed regular-patch oracle.

For each descriptor, an independent f64 radial ray from the body centre is
intersected with every reconstructed surface-triangle plane; cross-product
barycentrics reject points outside each triangle. Every root must find a hit.
Duplicate edge hits are harmless; the nearest positive hit is selected.
Skirts are not ground and are excluded. There is no CPU page interpolation at
plant positions and no reuse of the renderer's ray-query intersection code.

The canonical root is `(R + descriptor.ground_height_m) * direction`; reported
signed metres are its radial projection minus the ray's terrain radius.
Negative means below the generated terrain; positive means above it. Counts
use the exact numerical sign (and separately report exact zero), not an
undocumented visual/contact dead band. Global and nonempty per-kind summaries
report count, minimum, nearest-rank p50/p95, maximum, maximum absolute error,
and below/above/zero counts; all three plant-kind counts are printed even if
zero. Adapter/driver/backend identity, seed/radius, page base/residual scale,
stored quantization error and source range, sampling scales, candidate/sample
counts, submitted triangles, and vertex-frame/source-height errors are printed.

## Numerical checks and limitations

CPU tests cover exact planar/oblique radial hits, both winding orders,
parallel/backwards/outside/degenerate misses, a boundary hit, signed clearance,
the nonplanar b-c diagonal (deliberately distinct from bilinear and a-d
interpolation), a planet-radius translated tilted plane, and body-XYZ anchor
reconstruction on all six cube faces. A separate CPU fixture test checks actual
nonempty canonical descriptors and exact uncorrected ground heights.

The hardware test checks all vertex words for finiteness, homogeneous W,
normal length, and frame reconstruction against independently normalized cube
directions at each vertex's returned height. Its conservative local f32
arithmetic sanity allowance is
`128 * f32::EPSILON * (R*anchor.half_extent + abs(height) + 1)` metres. It also
checks returned vertex heights against actual source grid samples with the
0.5 m page budget plus a small f32 decode allowance. Neither check establishes
a root-contact envelope. Barycentric inclusion allows only arithmetic
roundoff, `64 * f64::EPSILON * max(ray_t,1) / max(edge_lengths)`, accounting for
planet-sized ray-point subtraction; this is dimensionless, not a plant-height
correction or contact threshold. Unit-direction squared-norm error is checked
below `1e-15`; projecting the canonical root contributes radius-scaled f64
roundoff, not a physical height adjustment.

No tight or broad contact bound is asserted before measurement. The page
quantization budget bounds grid samples, **not** between-vertex relief,
triangle chords, arbitrary windows, or actual viewer resident-mesh agreement.
A fixture-specific envelope, if later added, needs measured local relief and
source/representation error evidence. No universal root-accuracy claim follows.

The fern prototype's intrinsic minimum local Y of `0.1 * scale` is a separate
prototype-geometry limitation: its visible mesh may start above its descriptor
root. This diagnostic does not render prototypes, compensate this gap, or
change placement. Water/dynamic oceans, animation, actual viewer resident
cover/LOD transitions, persistent conforming topology, foliage streaming,
material displacement, collision ownership, visual preference and complete
surface acceptance are deliberately outside scope.

## Reproduction and execution record

Use this worktree's **own** target; do not share build outputs. Every Cargo
invocation sets the approved scratch directory. No GPU execution is authorized
for this isolated implementation until the parent approves it.

Compile without executing any test:

```bash
TMPDIR=/home/chechulin/Projects/project-thessa/target/tmp \
  CARGO_TARGET_DIR="$PWD/target" \
  cargo test -p thessa-terrain-native --test plant_ground_contact --no-run
```

CPU-only helper and descriptor tests (hardware test remains ignored):

```bash
TMPDIR=/home/chechulin/Projects/project-thessa/target/tmp \
  CARGO_TARGET_DIR="$PWD/target" \
  cargo test -p thessa-terrain-native --test plant_ground_contact -- --nocapture
```

**Parent-only, after approval**, run the single hardware diagnostic:

```bash
TMPDIR=/home/chechulin/Projects/project-thessa/target/tmp \
  CARGO_TARGET_DIR="$PWD/target" \
  cargo test -p thessa-terrain-native --test plant_ground_contact \
    fixed_l14_gpu_patch_plant_root_contact -- --exact --ignored --nocapture
```

Additional static gate:

```bash
TMPDIR=/home/chechulin/Projects/project-thessa/target/tmp \
  CARGO_TARGET_DIR="$PWD/target" \
  cargo clippy -p thessa-terrain-native --test plant_ground_contact -- -D warnings
```

The hardware test is marked `#[ignore = "requires a wgpu adapter"]` and fails
explicitly rather than silently passing without an adapter. Hardware execution
and its limited result are recorded below. No viewer-contact acceptance, visual
acceptance, remote CI or benchmark win is claimed.

Actually executed on 2026-10-21:

- Compile-only command above: passed after removing a reference to the
  crate-private `lod::MATERIAL_SLOPE_WAVELENGTH_M`. The actual canonical adapter
  still owns that sampling decision. The output's 256 m material-scale label
  is baseline source-checked metadata only, not a new sampler or public getter.
- CPU-only command above: **4 passed, 0 failed, 1 ignored** (2.43 s test time).
  The ignored item was exactly the GPU diagnostic.
- Changed-file `rustfmt --edition 2024 --check
  apps/terrain-native/tests/plant_ground_contact.rs`: passed.
- `python3 tools/agent-qa/check_architecture.py`: passed (12 guarded foundation
  crates).
- Scoped Clippy command above: passed with `-D warnings` (43.39 s).

Only the new test and these notes are owned by this isolated change. Existing
README/doc-70/status claims about the CPU sampler and outstanding production
resident-mesh agreement remain accurate; no production behavior or acceptance
status is changed here.

## Parent hardware result — 2026-10-21

The exact hardware command above passed on AMD Radeon 880M Graphics
(RADV STRIX1), Vulkan, Mesa 26.2.5-arch1.1. Log in the isolated worktree:
`target/parent-ground-hardware.log`. All **222** canonical roots intersected the
actual generated regular patch, with no placement offsets. The readback
confirmed step 1: 2,048 interior plus 256 skirt triangles; skirts were excluded
from the ground oracle.

Hardware log SHA256:
`7ab755ba51e97b031238a8a706f58036625e373cebe417fc21eeb4e0b6a3f24d`.
Test source SHA256:
`e36624de6dbf9bae18b5692c37c70c85fd12b0561c6b5bd39b49b2930e56e04f`.

| Population | Count | Min signed m | Nearest-rank p50 m | Nearest-rank p95 m | Max signed m |
| --- | --- | --- | --- | --- | --- |
| All | 222 | -0.068938668 | -0.000603619 | 0.047214586 | 0.096391605 |
| Fern | 62 | -0.053092270 | -0.000715423 | 0.050159831 | 0.096391605 |
| Reed | 12 | -0.030268495 | -0.000929817 | 0.027635578 | 0.027635578 |
| Lycopsid | 148 | -0.068938668 | -0.000573151 | 0.034903295 | 0.074226679 |

Negative means the descriptor root is below the generated terrain; positive is
above. There were 155 negative and 67 positive values, none exactly zero.
Maximum absolute residual was **0.096391605 m**. This is a measured fixed-fixture
result, not an asserted universal bound. Vertex-frame reconstruction error was
at most 0.000036354 m; GPU decoded grid height versus source error was at most
0.000053186 m. The page stored residual scale was 0.00010644426 m, stored maximum
error 0.00005327431 m and range 0–5.3496675 m. Thus the allowed 0.5 m page budget
was not the observed grid quantization error; it must not be quoted as measured
root error or as a between-vertex guarantee.

This still does **not** measure the actual viewer's streamed resident cover,
conforming topology or fern geometry gap. Those remain separate limitations.
