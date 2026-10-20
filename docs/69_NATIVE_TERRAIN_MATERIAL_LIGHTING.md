# 69 — Native terrain material lighting

**Status: implemented primary sky-reflection slice and grazing horizon guard; terrain visual acceptance incomplete.**

## Source and consumer

Regular and conforming native terrain now receive the existing shared sky field
through their rough dielectric BRDF, not only through a stellar highlight and
diffuse ambient. The material remains sRGB albedo plus linear roughness from
the existing page residency/mip/ancestor path. Geometry, authoritative heights,
contacts and simulation are unchanged. No new field producer, streamer, texture,
render target, render pass or ray-tracing requirement is introduced.

`render-wgpu::environment.wgsl::env_specular_from` integrates incoming radiance
against the same GGX distribution, Schlick-Smith masking and Schlick Fresnel used
by `rcbt-core`'s direct dielectric kernel (F0 = 0.04). It importance-samples
visible GGX normals using Heitz, *Sampling the GGX Distribution of Visible
Normals*, JCGT 7(4), 2018. Deterministic Hammersley quadrature uses no
frame-random noise or history buffer; it does not certify motion stability.
The visible-normal PDF uses exact GGX G1; the weight
corrects it to the existing Schlick-Smith BRDF. Below-surface samples contribute
zero and remain in the denominator; renormalizing survivors would create energy.

Every accepted direction uses shared sky radiance plus the existing
reflected-cloud extinction/in-scatter law at the surface origin, with a bounded
reflection-specific quadrature budget (below).
It does not add the directly evaluated stellar disc a second time. The legacy
shader hook returns zero, preserving the runnable Bevy path without requiring
native bindings.

## Settings, fallback and accounting

- `[renderer].specular_reflections = false`: the native hook is compiled to
  zero, with no quadrature or land-only sky bindings. Other sky/cloud/ocean
  consumers retain their independent settings.
- `[renderer].surface_quality = "low" | "medium" | "high"`: 4 / 8 / 16
  clear-sky samples per land fragment, resolved by
  `ResolvedGraphicsSettings::terrain_environment_samples()`.
- When clouds are published, use 1 / 2 / 4 directions and 1 / 2 / 4 cloud
  view steps, capped by the cloud producer's steps, and at most one sun-depth
  step (zero if the producer disables sun-depth integration). These budgets
  are resolved by `terrain_environment_cloud_budget()`. Density, phase,
  extinction, scattering and self-shadow laws are unchanged; only numerical
  sample counts change. Each hollow-shell segment holds the physical source at
  its midpoint and solves Beer-Lambert analytically after density quadrature;
  it does not run the full nested primary march per GGX direction. Primary
  clouds and existing mesh/ocean reflection budgets remain unchanged.
- No published atmosphere/aurora sky field: zero added land reflection, rather
  than an invented ambient sky. Existing direct/diffuse terrain behavior remains.
- No new device capability or native backend is required beyond the current
  terrain path. Ocean classification and specular disable behavior are unchanged.

`TerrainRenderer::land_environment_samples()` reports the applied budget;
native capture metadata records `native_terrain_environment_samples`,
`native_terrain_environment_cloud_budget` and the consumer's scope. Cost belongs
to `render.scene_classify_and_raster`. There is no new persistent large
allocation. A rejected first implementation nested the
full primary cloud march under every sky sample: measured 1280×720 COAST GPU
median was about 1877 ms, versus 38 ms with specular disabled. These are a
diagnostic of unacceptable added cost, not an optimization win or an isolated
environment-only comparison. Reflection-specific budgets replace that path;
sample counts are still **not** a frame-time guarantee. The optional diagnostic
uses actual GPU timestamps and reports unavailability when timestamp features
cannot be negotiated.

## Grazing horizon guard

The visible-normal estimator deliberately keeps rejected below-surface samples
in its denominator. At a silhouette every sample can be rejected; Fresnel then
approaches one and removes the diffuse term as well, so the fragment's final
radiance is exactly zero. On the real terrain this produced near-black dashes
along ridge crests (and larger pure-black patches on the surveyed volcanic
site), not geometry holes.

`env_specular_from` now returns a continuous sky floor in the extreme grazing
band:

- `horizon = 1 - smoothstep(0, 0.2, N.V)`;
- `floor = env_base_radiance(normal, ambient) * F * horizon`;
- the result is `max(integrated, floor)`.

The floor uses only the already-published environment field and the existing
Schlick Fresnel. It adds no field, texture, pass or material coefficient, it
does not renormalize surviving directions, and it is zero at `N.V >= 0.2`.
Above that band the rejected-sample denominator is unchanged. The guard is a
bounded numerical horizon term, not a second BRDF; its physical reading is the
unresolved grazing sky reflection that the finite quadrature loses, and it is
bounded by the sky radiance times Fresnel.

The targeted diagnostic used a constant-red fragment output to prove the black
pixels were shaded fragments, then found zero final radiance with non-zero
albedo and illuminance. A GPU compute probe calls the actual shared WGSL
function with a uniform 100-radiance field at `N.V = 0`, `1e-7` and `0.05`.
The pre-guard shader returns exactly zero at `N.V = 0`, so the probe is a real
regression rather than a source-shape check. With the guard the three values
are `100`, `99.99994` and `66.051` radiance, finite and bounded by the uniform
field. A full-terrain probe at `N.V = 0.01`, `0.001` and `0.0001` also stays
non-black on regular and conforming geometry. The existing independent
`N.V = 0.2` and `0.7` grazing oracle still passes, so the guard does not alter
the calibrated band. On the surveyed real frames, all sampled near-black
pixels disappeared; a specular-off control shows that the remaining thin dark
ridge rims are reflected sky at grazing incidence rather than holes. Those
rims are still an aliasing/visual-quality issue, not an accepted result.

## Numerical and visual acceptance boundary

`render-wgpu/tests/terrain_environment.rs` checks actual primary raster output:

- all three quality levels, with regular and conforming geometry, black
  albedo and independently integrated uniform radiance;
- normal-incidence tolerances of 7 / 4 / 2 RGB8 levels at the pinned
  radiance/exposure, and an 8/255 High grazing envelope at N.V = 0.7 and 0.2;
- directional red/blue fields: rough material broadens the reflection rather
  than substituting an unfiltered mirror;
- same-size field writes reach the terrain without material reupload;
- field removal and the specular switch eliminate the added term;
- reflected-cloud extinction and in-scatter against an independently integrated
  smooth spherical deck with an isotropic blue source and red incident sky.

The existing sky-background depth test explicitly disables material specular
response: a newly published sky is now supposed to change foreground material
lighting, which is not a background depth overwrite. Its original exact
foreground/background and field-removal assertions remain in place.

The grazing oracle directly integrates D*G*F over solid angle; it does not copy
the shader's sampling/PDF code. Its larger tolerance records finite quadrature
error (the initial 5/255 trial was insufficient), not a new material coefficient.
These are fixture-specific absolute bounds, **not** a general error guarantee
for narrow HDR peaks, arbitrary orientations or grazing material-normal detail.
The cloud-deck fixture is not a global transport-error bound for heterogeneous
weather, narrow stellar phase peaks or grazing cloud paths; four directions and
four density steps with one source per segment are an explicit cheaper fallback,
not converged cloud transport. Spatial variation of source/self-shadow inside a
segment remains a known approximation, not claimed equivalent to the full march.
The cost diagnostic counterbalances disabled / Low / Medium / High and reverse
order on the same 1280×720 sphere/sky/material, with 32 measured samples per
configuration. It measures whole classify/raster plus uncovered sky, without
clouds, shadows or direct light, not whole-client performance.

The 2026-10-20 stable-source hardware run on Radeon 880M / Vulkan / RADV Mesa
26.2.4 measured medians of 0.346 ms (specular disabled), 0.867 ms (Low),
1.218 ms (Medium) and 1.639 ms (High) in that isolated fixture. These are costs,
not a speedup claim or a production-frame budget. Uniform-field High central
RGB8 values were 101 / 77 / 31 for roughness 64 / 153 / 255, equal to the
rounded independent reference. Maximum tested grazing deviation was 7/255.
The reflected-cloud fixture produced `[60, 0, 15]` versus the independent
`[63, 0, 15]`, on both regular and conforming terrain.

Missing material normals, close-up geometry/erosion convergence, vegetation,
finite aerial perspective and primary scene-object reflections remain separate
work. Sky reflection alone cannot make the planet look like Satisfactory.
An earlier note in this document described the dark silhouette pixels as
constant-colour background gaps. That interpretation was wrong: the pixels
were fully shaded terrain fragments whose final radiance was zero because the
grazing environment integral was empty. The horizon guard above removes them.
A corrected site-1 shadow control (`[atmosphere].enabled = true`,
`[shadows].enabled = false`, identical published ambient
`[1112.58, 2051.11, 4132.93]`) changed only 3.21 % of pixels by more than
4/255, with whole-frame medians 89.230 ms (shadows) versus 89.397 ms (no
shadows); this isolates shadowing from the reflection term but is one site,
not a shadow-quality acceptance.
No blind subjective preference, cross-platform acceptance or whole-scene visual
parity follows from these numerical tests or the captured survey frames.
