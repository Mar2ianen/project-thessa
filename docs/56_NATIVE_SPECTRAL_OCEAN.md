# 56 — Native spectral ocean

**Status (2026-10-17): implemented native render-only slice.** Independent Rust /
portable WGSL implementation; no upstream code, dependency, asset or license
change. The legacy Bevy client remains runnable and retains its analytic ocean.
This is not an authoritative fluid model, complete ocean optics, or complete
Bevy/`large_cbt` renderer replacement.

## Contract and ownership

- `render-core::ocean::{SeaState,Spectrum,Mode}` owns backend-neutral SI inputs,
  deterministic coefficients, dispersion, phase reduction and a direct Fourier
  reference evaluator. It imports no Bevy/wgpu handles or wall clock.
- `render-wgpu::ocean::OceanField` owns persistent coefficients/work buffers, the
  portable GPU inverse FFT and a shared height/gradient/slope-moment array.
  `TerrainRenderer` owns its bounded optional near-slot backup/deformation and
  integrates into the existing geometry, classifier, raster, shadow and mesh-ray
  publication flow. No separate mesh, topology scheduler or ray scene is added.
- Native window/headless hosts call `update_ocean` **after** height, material and
  view updates, **before** atlas encoding and mesh `sync_rt_terrain`. Geometry
  generation, field publication, deformation and AS queries are queue ordered.
- Inputs: local surface gravity `mu/radius²` from the same baked body as terrain,
  uniform reference water depth, wind speed/planar angle, significant height,
  seed, simulation sample seconds, and the existing clamped sea-datum material.
  Native flight uses the authority's presentation sample, not a second timer.
- Outputs: periodic linear height in metres, analytic dimensionless slopes,
  unresolved slope variance, optional displaced near-grid vertices, optical
  normals/roughness/Fresnel and counters. No collision, aero, buoyancy, terrain
  source heights or authoritative simulation state is modified.

## Spectrum and planet mapping

The independent reference is Jerry Tessendorf, *Simulating Ocean Water* (2001);
the wind-shaped base is the Phillips spectrum. Three disjoint radial frequency
bands have fixed physical periods **2048, 256 and 32 m**. A seeded Gaussian
realization is normalized to the requested ensemble significant height
`Hs = 4 sigma`; it is not an exact per-frame extremum constraint. Wind changes
the spectrum shape; Hs is an independent prescribed sea-state input, not a
predicted result of fetch/weather integration. Short-wave damping uses the
water capillary length (`sigma=0.074 N/m`, `rho=1025 kg/m³`).

Each mode evolves with finite-depth gravity-wave dispersion:

```text
omega² = g k tanh(k depth)
h(k,t) = h0(k) exp(i omega t) + conjugate(h0(-k)) exp(-i omega t)
```

DC and Nyquist lines are zero; paired modes maintain Hermitian symmetry. The
two-pass radix-2 inverse FFT evaluates height and both analytic derivatives with
shared twiddles. Coefficients use a spatial-sum convention; GPU uploads scale by
N² and each FFT pass divides by N. A CPU f64 direct sum is the independent GPU
oracle, **not** a second production spatial evaluator.

Three independent planar realizations (YZ/XZ/XY) blend smoothly with normalized
`normal_component^4` weights. This avoids cube-face/pole UV switches; the same
field is sampled on both sides of every cube edge. Gradients include derivatives
of the blend weights and are projected onto the sphere tangent. Camera-relative
points combine with CPU f64 camera residues modulo each period; no planet-sized
f32 position subtraction determines phase. Rebased f64 mode phases plus at most
60 seconds of GPU delta avoid accumulating an unbounded f32 timer. Tests cover
negative/reversed time and samples through 1e9 seconds; large time jumps/rebases
upload coefficients and their spike cost is not separately benchmarked. This is not an
unbounded astronomical phase-precision guarantee.

This construction is a homogeneous render sea, not globally advected climate:
the configured planar wind angle is used in each plane, and depth is uniform,
not terrain-derived bathymetry. Dispersion is linear: no choppy horizontal
displacement, breaking, foam, wakes, shoaling/refraction or shore interaction.

## Filtering, geometry and optical consumers

2026-10-18 diagnostic follow-up: the persistent bisector surface can also
borrow this field/material residency through `ConformingSurface::set_ocean`.
It re-emits datum, reduces a common near-field footprint, and deforms shared
vertices identically under the existing resolved ocean settings; disabled,
optical-only, coarse and whole-partition-overflow fallbacks are explicit.
This adapter is not selected by production raster/RT; the generated-patch
consumer below is unchanged. Numerical oracles, measured scoped costs and
remaining temporal/platform gaps are in [doc 63](63_CBT_CANONICAL_SURFACE.md).

The shared `rgba16float` array stores height, two slopes and a scaled second
slope moment (a coefficient-bounded scale, up to ×1024, avoids small-moment
subnormal loss without overflowing half-float at steep inputs). Slopes outside
the portable range fail explicitly instead of publishing infinities. Mips average the
moments, not normalized normals. Sampling combines filtered slopes with
`max(E[s²]-E[s]²,0)` plus omitted-frequency slope variance. The optical GGX width
uses `alpha² = base_roughness^4 + variance`, with water IOR 1.333. This is a
bounded-cost scalar/isotropic lobe approximation; it does **not** establish an
anisotropic slope distribution, measured energy-conserving BRDF convergence,
or a convolved environment lookup. Triplanar omitted variance is a conservative
planar estimate, not an exact tangent-space covariance transport.

Near deformation references the **existing generated 33×33 terrain pool**. A
bounded backup holds only admitted slots; each update restores the datum before
recapturing/applying displacement. A regenerated/reused slot's revision makes
its old backup ineligible. Heights above the existing sea-datum band and rough
land/ice materials do not deform. Material classification uses the existing
world-addressed directory/ancestor/edge kernel and material alpha roughness,
not a new ocean mask or CPU-expanded mesh. Material-directory/texture changes
conservatively invalidate the near set; even an unrelated far-page upload can
currently re-deform/rebuild it. Localizing this dependency is future work.

Candidate bounds reuse the existing nine-point tile AABB plus a normalized-cube
Hessian envelope; selected slots must intersect the configured camera sphere.
Displacement fades smoothly from half the radius to its edge. Cells sample the
field with a two-cell footprint; modes below geometric resolution remain optical
detail. Cells whose footprint spans the longest full period have zero-mean DC
height and need no deformation. Admitted near slots retain the **full grid** in
classification to avoid decimating already-displaced vertices. Culling expands
only those slots by the coefficient-derived absolute height envelope (twice
the envelope between sample points). Unselected terrain retains ordinary LOD
and culling.

Raster, the shared raster atlas and optional terrain BLASes consume these same
deformed vertices. Only selected slots rebuild BLASes on changed samples;
camera movement may also update the distance fade/admission, and thus rebuild
near BLASes. A paused, unchanged field/material/camera performs no FFT,
deformation, AS or atlas redraw. This explicitly supersedes the **camera-only
never rebuilds terrain BLASes** statement for opt-in, camera-faded ocean slots.
Unrelated terrain remains persistent and dirty-only as before.

Ocean raster reflection samples the existing published single-scattering sky
grid (or explicit isotropic ambient when absent), not a fabricated second sky.
Mesh RT water hits sample the same field for water normals, roughness and F0;
intersection geometry is the shared deformed grid. Ray-hit field sampling still
uses explicit zero footprint: no ray cones or rough environment convolution.
Primary ocean/terrain receivers still use atlas shadows and environment
reflection; **they do not yet reflect nearby meshes by ray query**. Outside the
near sphere or on admission failure, RT/raster intersections retain the datum;
optical waves continue. That is an explicit geometric approximation, not an
exact displaced whole-planet ray surface.

The coefficient sum supplies an absolute height envelope, including frequencies
omitted at cheaper quality. Geometry interpolation/fading cannot exceed it;
GPU tests pin the fixture bound. The Hs=2 m / seed=17 fixture's conservative
envelope is 96.616082 m, **not** its observed wave height. This is not a gameplay
error budget and must not be used to authorize collision/buoyancy approximations.

## Requested/resolved settings and budgets

All controls are `[ocean]` in `graphics.toml` / `RequestedGraphics`, resolved
through `ResolvedGraphicsSettings`; no environment/plugin-presence toggle.

| Control | Meaning |
|---|---|
| `enabled=false` (default) | No field, FFT/deformation pipelines, ocean bindings, backup, copy-destination usage or ocean dispatch. Legacy path stays intact. |
| `quality=low/medium/high` | N=32/64/128 for all nine band/plane layers; same 128² reference sea state, omitted slope variance retained optically. Presets change quality, not physical inputs or enable. |
| `wind_speed_m_s`, `wind_direction_rad`, `significant_height_m`, `depth_m`, `seed` | Prescribed render sea state; edits currently require renderer recreation. |
| `geometry_patches=16` | Hard backup/admission limit 0..64. Zero is optical-only. |
| `geometry_distance_m=300` | 0..20000 m; zero is optical-only. A 1 km /64-slot diagnostic is deliberately more expensive than the default. |

The active near partition is admitted **whole** or not at all. Overflow restores
previous displaced slots and reports optical-only fallback; subsequent smaller
partitions recover. There is no arbitrary truncated or stale blocker subset.
Ordinary portable storage-buffer/filtered RGBA16/array-texture capabilities are
used; no mesh shader, native Vulkan API or RT requirement for raster ocean.
Unsupported explicit physical inputs/device resource limits fail clearly.
Mesh RT retains its existing capability/binding/whole-terrain-budget fallbacks;
reflection hit bindings additionally account for the shared ocean texture/sampler.

Field GPU data allocations: **835,672 /3,342,424 /13,369,432 bytes** for Low /
Medium /High. Backup/list/params cost `slots*(1089*32+4)+16`: 557,648 bytes at
16 slots; 2,230,544 at 64. Counts exclude CPU coefficient storage, driver
pipeline/query/AS allocations and small disabled-environment placeholders.
These fixed quality/capacity bounds are separate from the RT scene's existing
planning budget; borrowed vertices are not allocated twice for RT.

`OceanStats` / native completion logs report N, evaluations, last FFT dispatches,
field/near data bytes, admitted slots, absolute envelope and fallback reason.
Profiling remains opt-in. Four-mark field, two-mark deformation and three-mark
AS/mesh diagnostics report actual hardware timestamps when available; absent
support stays unavailable. Standard native frame GPU scopes begin **after** the
field/deformation submissions, so their GPU span excludes this work; host frame
completion includes it. Use the separate diagnostics for attribution.

## Verification and performance

Core tests cover conjugate symmetry, zero/calm state, periodicity, truncation,
finite-depth/shallow/deep special cases and local-gravity scaling. GPU tests
compare height/slopes with direct sums at N=16/32/64/128 across time/rebases,
using absolute+relative half-float tolerances `0.002 + 0.002*abs(reference)`;
moment mips retain their stored mean within 0.5%+1e-7. Geometry tests cover the
height envelope, no accumulation, deterministic pause/rewind, material/height
refresh, whole-partition overflow/recovery, full-grid culling of an interior
crest and cube-edge/camera-rebase agreement within **2 mm** on the fixture.
The seam fixture uses equal-resolution neighboring tiles. Different-level edges
can sample different geometric footprints and retain the existing raster skirts;
watertight displaced mixed-LOD ray geometry is not established by these tests.
Mesh-ray oracles confirm that changed shared waves alter ray results, paused
waves need zero AS builds, and rewinding reproduces output bytes.

Retained release diagnostics (Radeon 880M, RADV Vulkan) separately measure
evolution, FFT, mips, near deformation, AS rebuilds and mesh/RT draw. The final
bounded-moment run's sums of stage medians were approximately
**0.067 /0.222 /0.636 ms** at Low/Medium/High (not measured total medians).
Sixteen near slots cost **0.120 ms** deformation and **1.412 ms** AS builds,
versus one slot's **0.019 /0.435 ms**. These are scoped
same-implementation costs, not renderer FPS, GPU throughput limits or a
performance win over Bevy/`large_cbt`. Unlocked clocks, first-run concurrent
compilation and serial diagnostic readback limit comparisons.

Commands, final repeats, native controls and actual check results are in
[`artifacts/native-spectral-ocean-2026-10-17.json`](artifacts/native-spectral-ocean-2026-10-17.json).
Native captures are integration/scoped-cost evidence, **not** blind aesthetic
acceptance or cross-platform runtime validation.
