# 106 — Spatial compression cells and reference-driven plume revisions

Status: a reduced spatial shock field is implemented in `plume-core` and native
Medium/High. It fixes axial-table aliasing and separates stable compression
structure from the moving mixing skirt. This is not calibrated combustion,
a nonlinear Mach-reflection solver, native volume lighting, or accepted visual
parity. Evidence is in [the numerical record](artifacts/plume-spatial-shocks-2026-11-01.json).

## Reference observations and handoff audit

The user's architectural handoff was checked against the working branch, not
assumed to describe its newer source. Exact homogeneous-segment transport
already exists (doc 100), as do the curved centerline, transported nozzle frame
and capsule support (doc 105). The five previous station-indexing experiments
regressed a near-nozzle case (doc 99). Its proposed uniform-coordinate lookup
is also inapplicable to the new quadratic publication without another measured
implementation and a generic-publication fallback.

Actual primary visual references inspected:

- [Skyroot/Dhawan-III test, Pawan Kumar Chandana](https://www.linkedin.com/posts/pawan-kumar-chandana-a90b6514_that-electric-blue-flame-those-perfect-mach-activity-7433450505064022016-aanZ):
  the manufacturer's post explicitly identifies methalox. The primary image
  shows blue/violet repeated bright regions, transparent gaps and a diffuse
  outer stream. It does not supply operating pressures, dimensions, exposure,
  spectral response or a calibrated radiance target.
- [Relativity Aeon R testing](https://www.relativityspace.com/aeon-r-progress):
  March/July 2023 TCA photographs and the February 2024 test video at 10 s.
  A blue near-nozzle region, bright compressed features and a separate broad
  orange discharge are visible. Steam, structures and clipped highlights
  cannot be treated as the main jet's cross-section or its absolute emission.
  Gas-generator-only entries were excluded as main-bell references.
- [NASA normal-shock equations](https://www.grc.nasa.gov/www/k-12/airplane/normal.html)
  provide the perfect-gas jump relations; the
  [Mach-angle reference](https://www.grc.nasa.gov/www/k-12/airplane/machang.html)
  constrains interpretation of characteristic cones. Neither validates the
  optical shape closure below as a CFD result.

The October KSA source/assembly audit in doc 104 was revisited: its shock-cell
phase and spatial LUTs are separate from its smooth gas profile, and density
and emissive structures have different roles. No proprietary GLSL, optical
LUT, texture or XML template was imported into the MIT engine. The open KSP
mod mentioned in the handoff is a future reference, not copied implementation.

## Keep spatial frequencies out of the smooth table

Previously, a cosine was sampled into 48 stations uniformly spanning up to
4000 m. A cell only a few metres long could be missed before ray integration
started. Increasing march steps could not recover that information.

Generated `AxialProfile` stations now contain only the smooth mean; their
legacy `shock` is one. `AxialProfile::cells` carries cell spacing, coherent-core
radius, compression and phase. Historical arbitrary optical publications
retain their axial/J0 interpretation. CPU, straight and curved GPU consumers
evaluate the independent spatial field at actual emitted distance and radius.
`plume_shock.wgsl` is shared by both native volume entries.

Station positions are `z_i = length * (i/(N-1))^2`, with exact endpoints.
This spends the existing 48-station budget near the steep nozzle gradient.
Interpolation remains linear in metres between the published stations.
The stock Cycles adapter uses the tagged inverse coordinate and then the
physical interpolation fraction; it does not interpolate in sqrt(z).

## Reduced compression and optics

For the pressure-restoring jump `p_jump = max(Pe/Pa, Pa/Pe)`, solve the normal
shock pressure relation for its normal Mach component, capped by exit Mach:

```text
Mn^2 = min(Me^2, 1 + (p_jump - 1)*(gamma + 1)/(2*gamma))
C = (gamma + 1)*Mn^2 / ((gamma - 1)*Mn^2 + 2)
```

`gamma` is a validated physical `PlumeSource` input, not chosen by an RGB
preset. The compiled control forwards the compiled engine value through
`engine_plume_source`; the provisional native/legacy deck explicitly uses 1.2.
The independent CPU test recovers pressure and downstream enthalpy from mass,
momentum and energy conservation. This verifies the one-dimensional jump,
not conservation or shock topology throughout the spatial closure.

The existing Tam-based cell length remains a first approximation. A periodic
round double-cone compressed region, a finite-width incident surface and
a Gaussian axial core form the spatial optical closure. Overexpanded and
underexpanded sources have different phase; compression vanishes continuously
at matched pressure. Mixing erases it over three cell lengths, and the skirt
outside 1.2 coherent radii is unmodulated. True vacuum has no atmospheric train
of cells. There is no explicit Mach disk, triple point, flow-separation model,
thermochemical species solve or prediction of nonlinear topology transitions.

Density/extinction and emission no longer receive the same multiplier:
the latter uses a provisional collisional density-squared proxy. Mean emission
also dilutes with density squared as the cross-section expands. The developed
mixing slope turns on smoothly over the coherence length instead of applying
in full at the exit; this avoids prematurely extinguishing the potential core.
These are reduced closures, not independently calibrated gas mixing or
temperature/species emission. The methalox RGB ramp now stays blue/violet
downstream, qualitatively following the Dhawan-III image; it is not a spectrum.

The stable Gaussian core and spatial compression use undeformed radial
distance. Advected azimuthal motion deforms only the mixing skirt. This removes
the previous noisy deformation of the near-nozzle compression structure while
retaining nozzle-frame roll and motion.

## Units before lighting

The handoff correctly identified that `mdot * velocity` is momentum flux in N,
not power in W. `jet_kinetic_power_w` now reports `0.5 * mdot * throttle * Ve^2`;
the supplied operating exhaust speed is not throttled a second time. Tests
pin the velocity-squared and single-flow scaling. Enthalpy and chemical power
remain separate, absent inputs.

The full native CLI exposed a separate unit defect: a unit-scale emission
field disappears under the same daylight photometric exposure that correctly
renders surfaces. The revised native mean field therefore uses a declared
photometric brightness reference, not a per-view gain:

```text
Bv(T) = 683 * integral_380nm^780nm(B_lambda(T) * Ybar(lambda) d_lambda)
B_lambda(T) = 2*h*c^2 / (lambda^5 * expm1(h*c/(lambda*k*T)))
j_exit_RGB = rho_exit * kappa_t * Bv(T_exit) * authored_hue_and_gain
```

`Bv` is in cd/m2 and `j` in RGB cd/m3 before exposure. The original provisional
mass-extinction coefficient `kappa_t = 0.5 + soot` is interpreted in m2/kg;
it is not a measured gas absorption/scattering spectrum. Planck constants are
SI; the 1931 Y approximation comes from
[Wyman, Sloan and Shirley, Eq. 4/Table 1](https://cwyman.org/papers/jcgt13_xyzApprox.pdf).
The [CIE Planck-law definition](https://cie.co.at/eilvterm/17-24-005) and
[NIST SI photometric convention](https://www.nist.gov/pml/special-publication-330/sp-330-section-2)
provide the unit reference. Eighty 5 nm midpoint bins are evaluated only when
the source profile rebuilds. Comparison with an independent 0.1 nm integration
of the same fit bounds quadrature error below 2e-4 over 800–6000 K.

This is a graybody **brightness envelope with authored chromaticity and a
collisional spatial proxy**, not an assertion that clean blue combustion is
an LTE blackbody. Non-LTE species, mass optical coefficients, thermal balance
and absolute firing calibration remain missing. Surface and volume now share
the same photometric/exposure convention; that does not make the optical
closure a validated combustion model. Native previews use the same fixed
EV100=13 policy as the daylight client, not an exposure fitted to each case.

The legacy Bevy `emission_gain` remains normalized momentum flux, its duplicate
throttle multiplier removed. It is not radiated W. `radiant_power` remains a
historical unmodulated cross-section proxy, not the integral/CDF of this
heterogeneous field and not a calibrated watt quantity. It must not become
the PDF or energy source of native RT lighting without a proper field integral.

## Quadrature and work boundaries

The normal 8/14-step budget is retained outside the coherent core. Only rays
whose conservative closest approach intersects the 1.2-radius compression
support receive a radial demand `ceil(base_steps * compression)`, with an axial
cell-resolution demand capped at 64 per merged interval. For midpoint
quadrature, error scales with amplitude/N^2, while the optical contrast scales
with compression squared; scaling N with compression avoids paying the
strong-shock budget for weak cells. This is verified against the same dense
oracles, not asserted as a general error theorem. Sampling is concentrated at core width. Near-axial
straight rays concentrate near the nozzle instead of distributing fourteen
samples over the entire far tail. Exactly matched flow, disabled diamonds and
no-cell publications skip this extra work. Extinction and emission use the
same physical segment length and exact homogeneous-segment transport.

`march_steps` is the requested base per interval; `march_step_limit` is the
per-interval upper bound, not a measured number of executed samples. The CLI
reports both. Native Medium has eight swept segments, High sixteen, so the
conservative maxima are 512/1024 samples per ray if every segment produced a
disconnected interval. Disabled rendering allocates/submits nothing meaningful.
TAA remains default-off. Draw uniforms increase from 96 to 112 bytes; profile
and motion buffers retain their owners and immutable publication rules.

An offline CPU oracle can request up to 16,384 bins. Expanded tests cover 120
pressure/throttle/near-nozzle/chord/quality combinations, check convergence
between 4096 and 8192 bins, then hold the existing 0.02 absolute linear-RGB
envelope. The old 256-bin reference itself undersampled narrow incident regions
inside a very wide bound; its failing results are retained in raw evidence.
Eight curved views with depth variants also compare against independent dense
world-space integration. Native and stock Cycles tests measure transport,
not subjective preference or combustion accuracy.

## Measured validation and cost

The final field has 45 passing core tests, 40 render-library tests, eight
Python capture-schema tests, five native producer tests, nine GPU integration
tests on each of RADV and llvmpipe, and two library GPU oracles per backend.
The 120-case converged chord control reaches 0.011602 absolute linear-RGB
error against its 0.02 envelope; spatial curved probes reach 0.007419 against
0.012. These bounds apply to the declared fixtures, not all animated views.
Strict all-target Clippy for the three affected core/native crates and the
44-package architecture gate pass. Full native CLI Off/Low/Medium/High keeps
identical asserted flight state, silent audio, and visible enabled tiers.
The legacy Bevy application was not compile/run validated in this milestone.

An intermediate field disappeared under daylight exposure; the failing full
CLI test is retained. The photometric reference fixes that failure. Final CLI
images use a fresh external scratch mount: reusing an intermediate scratch
would invalidate its earlier PNG provenance. The final orbit uses actual
native Cache/PlumeRenderer, thirty-two prescribed frames at EV100=13. It is a
kinematic diagnostic, not a firing or an accepted flight-history replay.

Release ABBA on Radeon 880M, ten 1280x720 views, twenty warmup/eighty measured
frames per case, compares fixed strong-shock refinement with contrast-aware
refinement. All profile bytes match across variants; HDR repeats within each
variant are exact. Cross-variant HDR differs by the bounded quadrature change.
The source field, camera, time and optical normalization do not change.

| Mean/paired plume-pass cost | Fixed refinement | Contrast-aware refinement |
| --- | --- | --- |
| Near High curved case 5, paired p50 | 17.265 / 17.326 ms | 9.496 / 8.836 ms |
| Near High straight case 5, paired p50 | 2.917 / 2.576 ms | 1.523 / 1.531 ms |
| Vacuum straight case 2, mean p50 | 0.616 ms | 0.722 ms |
| Vacuum curved case 3, mean p50 | 1.817 ms | 1.891 ms |

The near curved case improves 47% on mean p50, but this is not a universal
speedup: the largest recorded straight regression is 0.106 ms (17.2%), curved
0.074 ms (4.1%). The full table and conditions are in the evidence JSON.
An unrelated Blender render was allowed to finish before timing; no external
Blender was detected during these ABBA runs. No driver/clock/global settings
were changed. Whole-client FPS and KSA performance are unmeasured. A separate
20,000-iteration **debug** producer probe reports 2.575 us cached extraction and
56.225 us rebuild; those are not release CPU-frame timings.

Nine straight and three curved fields were rendered by stock Cycles 5.2.2,
2048 samples, CPU, stock Volume Coefficients. The final quadrature was compared
against those retained EXRs only after asserting identical complete optical
scene/camera descriptions. The original Cycles source identities remain
intact; the final native capture has its own identity. Relative RGB L2 spans
0.54–48.99% for the straight set and 0.80–48.03% for the curved set; the worst
thin-jet discrepancy is unresolved. No fitted gain or physical-parity pass
threshold is used to hide this result.

Three blind pairs compare fixed versus contrast-aware sampling with identical
fields, poses and exposure. Keys are separate, and there are **zero independent
votes**. This is not a blind comparison against real footage or KSA. The
[diagnostic native views](artifacts/plume-spatial-shocks-2026-11-01.png) are
labeled implementation evidence, not a self-awarded reference match.

## Remaining acceptance

The reference record separates original stock Cycles captures from any later
native recomparison with byte-identical optical scene descriptions. Small
bright thin-jet features still expose substantial relative image differences;
pixel reconstruction, integration and model uncertainty must be investigated
separately. A low whole-image RMSE over black background is not visual parity.

Next work follows the handoff: actual accepted nozzle hardware/history (F03),
regular-cell versus Mach-reflection topology and calibrated optics (F04), then
an integral/CDF/PDF of the same emissive medium for FastRT visibility and
volume transmittance (F05). Compare diffuse and metal receivers with an opaque
occluder. Do not substitute unrelated point lights. Reference foundations are
[Practical Illumination from Flames](https://jcgt.org/published/0002/02/10/) and
[Higher Order Ray Marching](https://giga.cps.unizar.es/~amunoz/projects/CGF2014_higherorder/);
they are research inputs, not implemented methods claimed by this patch.
Separate plume temporal history, screen-footprint filtering, overlaps,
multi-nozzle transport and residual CBT remain open. Existing motion,
capsule support and exact segment transport must be reused.
