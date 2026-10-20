# 69 — Native terrain material lighting

**Status: implemented primary-raster material normals, primary sky-reflection slice and grazing horizon guard; terrain visual acceptance incomplete.**

## Source and consumer

Regular and conforming native terrain now receive the existing shared sky field
through their rough dielectric BRDF, not only through a stellar highlight and
diffuse ambient. The material remains sRGB albedo plus linear roughness from
the existing page residency/mip/ancestor path, with an optional tangent-space
normal plane used by the primary raster. Geometry, authoritative heights,
contacts and simulation are unchanged. No new lighting-field producer, render target,
render pass or ray-tracing requirement is introduced; the material page gains a
second texture plane and the primary raster declares its two bindings.

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

## Tangent-space material normals

The native material page now carries an optional 128×128 linear RG8 normal
plane with the same eight mip levels as albedo/roughness. The bytes are
`x * 0.5 + 0.5`, `y * 0.5 + 0.5`; positive z is reconstructed in the shader.
Pages built without a normal plane resolve to the flat geometric normal, so
older decoded pages and the Bevy albedo-only path remain valid.

Generation and residency:

- `build_gpu_material_page` evaluates `surface_normal_grain_height` only where
  the canonical surface sample is land. Ocean pages stay flat: the water branch
  owns its wave normal, and inventing seabed relief would add cost without
  affecting the water BRDF.
- The normal height spectrum uses Perlin gradient noise at 8 / 4 / 2 / 1 / 0.5 m
  with equal slope contributions where the page texel resolves them, plus the
  broader 32 / 128 / 512 m bands. The existing `band_weight` filter smoothly
  attenuates bands from four to two texels per wavelength; finer bands are
  discarded. The single 8 m value-noise octave used by the CPU residual path
  read as a cellular quilt when promoted to a normal source; the gradient-noise
  variant fills the resolved spectrum instead. This is a representation change,
  not a new terrain law.
- Slopes use the inverse local cube-sphere chart metric: the two physical
  direction chords and height differences define a 2×2 solve in the face's
  orthonormal tangent frame. Dividing both differences by one nominal texel
  width is wrong away from face centre, where the chart is skewed and stretched.
  The independent planar-gradient test recovers `[0.3, -0.2]` to `1e-12` on
  all six faces at centre/corners. The one-texel page apron
  makes the inner texel 1/126 derivatives use exactly the same field samples as
  the neighbouring same-level page, so normals meet at shared tile edges.
- Normal mips decode XY, reconstruct positive z, average the actual height
  gradients `normal.xy / normal.z`, and reconstruct a unit normal. Averaging
  unit-normal XY is not slope averaging (the initial prototype did that).
  A `z >= 0.01` numerical floor bounds extreme/invalid input in both CPU and
  shader decoding; it is not a guarantee for arbitrary grazing normal fields.
  Within a mip, hardware bilinear/trilinear filtering still interpolates RG8
  unit-normal XY before decoding, not exact gradients. Quantization, this
  approximation and conservative nominal texel-width band filtering remain
  representation errors; the analytic plane test is not their general bound.
- `render-wgpu::material::MaterialArray` uploads normals to a separate
  `Rg8Unorm` texture array and publishes a second sparse directory containing
  only pages with a normal plane. `MaterialUpload.normal_pages` and `.bytes`
  account for the extra writes. Pages without normals are absent rather than
  sharing a stale slot.
- The native page with normals is 131,070 bytes of mip payload versus 87,380
  bytes for albedo/roughness alone. Base payload for 512 pages rises from
  33,554,432 to 50,331,648 bytes before mip expansion. At 1024 resident slots,
  the additional logical normal-array payload is 42.67 MiB (driver alignment
  can add overhead). The page benchmark also includes a paired 64-page
  close-land workload at levels 14–17 that actually resolves the fine bands.

Shading and consumer boundary:

- `crates/rcbt-core/src/shaders/material_normal.wgsl` is included only in
  `CBT_RASTER_WGSL`. Its `material_at_level_raster` repeats the shared page
  hierarchy, coverage feathering and derivative footprint, but samples the
  normal plane in the same pass as albedo. This avoids a second residency
  traversal and keeps the shared `material_sample.wgsl` binding-free of normals
  for RT terrain, ocean geometry, conforming surfaces and tests.
- The decoded slope is transformed through the cube-face tangent basis and the
  body-to-render rotation, projected onto the geometric tangent plane without
  independently renormalizing its axes, then applied to direct diffuse, land Fresnel,
  direct sun reflection and the shared sky/cloud reflection. The perturbed
  normal does not move the shadow receiver plane, water classification, height
  or night-light mask: those remain geometric/geographic, and the city-light
  mask is emissive rather than a BRDF response.
- Residency and coarse/fine blending operate on premultiplied gradients.
  Missing normal coverage contributes flat; it must not be divided out and
  amplified to full strength. The normal influence shares the globe-map fade.
  The existing roughness/height water classifier suppresses material normals
  before direct diffuse as well as reflections, so a filtered land neighbour
  cannot perturb open-water lighting.
- The Bevy consumer binds its existing material array view at binding 19 and a
  one-entry zero normal directory at binding 20. Bevy pages remain albedo-only;
  the empty directory resolves to the flat normal without a second texture
  allocation.

The actual primary-raster oracle (`material_normal_plane_perturbs_actual_raster_diffuse_and_clears_on_removal`)
renders a flat page and a constant +X-tilted page under a +X sun on the real
terrain pipeline, checks the independent unsaturated Lambert reference within
2 RGB8 levels, then removes the
normal plane and requires byte equality with the flat frame. This proves
upload, directory publication, sampling, tangent transform and stale-entry
clearing. The same fixture checks byte-identical flat output with the resolved
normal switch disabled or on open water, and bounds the barely-covered corner
of a lone fine normal page within 5 RGB8 levels of flat (no coverage
amplification). It is not a general normal-map error bound. Same-face horizontal
and vertical shared-edge tests use level-16 land with nonzero resolved relief;
general nonlinear cross-face/LOD seam envelopes and temporal sweeps remain open.

Early development captures at sites 0/1/2, 500 m and 5 km, showed added grain
but also regular wind-ripple-like patterns on some slopes. Those captures used
the pre-correction chart/normal blending and are not evidence for the final
algorithm or accepted natural ground cover. Image gradient energy alone is not
a quality metric: aliasing can increase it as easily as meaningful detail.

### Final normal-plane measurements

The corrected implementation was validated on Radeon 880M / Vulkan / RADV,
Mesa 26.2.5. Evidence, exact commands, log/capture hashes, limitations and the
compiled-input manifest hash are recorded in
[`native-terrain-normals-2026-10-20.json`](artifacts/native-terrain-normals-2026-10-20.json).
All 13 local gates passed on 614 byte-identical compiled/configuration inputs:
1979 workspace lib/bin tests, 98 integration tests, 84 native feature-off tests,
73 renderer ignored hardware tests and 43 native ignored tests. The required
Bevy ignored-test command compiled successfully but matched zero tests after
the earlier optics migration; it is not eight more executed GPU tests.

| Diagnostic | Normal disable | Normal enable |
| --- | ---: | ---: |
| 64 paired close-land pages, p50 / p95 ms | 120.829 / 124.704 | 124.223 / 128.376 |
| 64 paired pages, aggregate seconds | 7.794 | 7.981 |
| 128 dirty-page upload bytes | 11,184,640 | 16,776,960 |
| CPU validation/residency/enqueue p50 / p95 ms | 3.792 / 3.962 | 7.964 / 8.378 |
| Upload plus queue submit/GPU drain p50 / p95 ms | 6.906 / 7.388 | 13.187 / 14.157 |

The final broad 512-page generation run measured p50 / p95 107.112 / 129.765 ms,
55.027 s total. Generation does not include mip building, upload or rendering.
The paired 64-page workload uses close land at levels 14–17, counterbalances
on/off order, and measures the **final** filtered Perlin source, unlike the
superseded value-noise prototype. Neither is a streaming-latency measurement.

The separate 1280×720 diffuse-only regular close-land fixture measured
classify/raster medians 1.047 ms disabled, 1.105 ms Low, 1.104 ms Medium and
1.102 ms High (32 timestamp samples each in forward/reverse order).
That is approximately 0.06 ms added normal lookup/shading in this fixture;
the tiny tier differences are not an established performance ranking.
The upload fixture has ten measured dirty snapshots per mode after warmup,
counterbalanced, with no raster. Drained wall time is not isolated DMA time.

Fourteen silent 1280×720 frozen-epoch native captures cover all three sites at
500 m / 5 km, site 2 at 200 m, three normal-disable controls, two conforming
controls and Low/Medium detail at site 2/500 m. Six High regular on-frame GPU
medians span 76.619–106.621 ms. At site 2/500 m, on/off medians were
89.973 / 88.712 ms; at site 1/500 m, 106.621 / 109.851 ms. These are one-run
whole-client measurements, not isolated effect cost or a converged speed claim.
Quality captures also alter reflection budgets. All final material-missing/job
counters are zero; on/off visible/cache counts match, but the complete streaming
snapshot is not identity-pinned. All fourteen have zero channel-sum-below-45
pixels, with minima 116–196; that statistic does not prove absence of geometry
holes or visual acceptance.

Implementer inspection still finds generic grain rather than convincing
substrate-specific ground, smooth macro slopes, pale snow/green fields,
grazing directional stretching and blue-grey ridge bands. The conforming
controls still show faceted silhouettes. No plant/rock scatter geometry or
independent blind preference review is claimed. These are outstanding defects,
not a completed natural-terrain or Satisfactory-quality milestone.

## Settings, fallback and accounting

- `[renderer].terrain_normal_maps = false`: native streaming omits grain
  evaluation and normal mip generation; upload skips normal bytes; the renderer
  uses only a 1×1 dummy normal texture and the shader's normal branch is a false
  compile-time constant. No full normal array or normal sampling remains.
  The setting is restart-required. Albedo/roughness and physical heights stay
  identical. The default is enabled.
- `surface_quality = low / medium / high` applies +2 / +1 / +0 normal mip bias
  through the resolver. Lower tiers filter more strongly and improve texture
  cache locality, rather than changing physical relief or normal strength.
  Metadata records `terrain_normal_maps` and `terrain_normal_mip_bias`.
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
to `render.scene_classify_and_raster`. The sky-reflection slice adds no persistent
large allocation; the optional normal array is accounted above. A rejected first implementation nested the
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

The preceding sky-reflection stage's 2026-10-20 stable-source hardware run on Radeon 880M / Vulkan / RADV Mesa
26.2.4 measured medians of 0.346 ms (specular disabled), 0.867 ms (Low),
1.218 ms (Medium) and 1.639 ms (High) in that isolated fixture. These are costs,
not a speedup claim or a production-frame budget. Uniform-field High central
RGB8 values were 101 / 77 / 31 for roughness 64 / 153 / 255, equal to the
rounded independent reference. Maximum tested grazing deviation was 7/255.
The reflected-cloud fixture produced `[60, 0, 15]` versus the independent
`[63, 0, 15]`, on both regular and conforming terrain.

The normal plane supplies resolved primary-raster grain, but close-up
geometry/erosion convergence, natural ground-cover/vegetation, finite aerial
perspective and primary scene-object reflections remain separate work. The
captures still show smooth macro forms and regular normal-grain ripples; sky
reflection and a normal plane alone cannot make the planet look like
Satisfactory, and no visual acceptance follows from these frames.
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

## Algorithm references and next detail work (2026-10-20)

The Minecraft references are algorithm/design comparisons, not dependencies,
copied shaders, assets, or proof of visual parity. Sources were read directly;
video/search-result descriptions were not treated as implementation evidence.

| Reference | Useful mechanism | Thessa decision and boundary |
| --- | --- | --- |
| [TerraForged ErosionFilter](https://github.com/TerraForged/TerraForged/blob/6dd607ebfd41a3b7274e090b1f2cf565d53d1d44/src/main/java/com/terraforged/mod/worldgen/noise/erosion/ErosionFilter.java) and [RiverCarver](https://github.com/TerraForged/TerraForged/blob/6dd607ebfd41a3b7274e090b1f2cf565d53d1d44/src/main/java/com/terraforged/mod/worldgen/noise/continent/river/RiverCarver.java) (MIT) | Seeded droplets, sediment/deposition, and separate bed/bank/valley profiles rather than indiscriminate high-frequency displacement. | Reuse Thessa's existing frozen runoff/deposition and local refinement (docs 50–52), not a second runtime erosion solver. Later rock/soil masks should come from those fields; the mod's dimensionless rates are not SI parameters to transplant. |
| [Tectonic folded erosion density](https://github.com/Apollounknowndev/tectonic/blob/34241bdb35acda67b5367d49f354c66c05e098e2/src/common/main/resources/resourcepacks/tectonic/data/tectonic/worldgen/density_function/noise/continent/erosion_folded.json) | Composable density/noise graphs for art-directed landform families. The inspected `erosion_folded` is an absolute-value noise operation, not a physical erosion solver. | Useful for procedural composition, not evidence of sediment conservation. Do not replace canonical landforms, geology or physical drainage with a function merely named erosion. No code copied; no permissive license was established in this review. |
| [NoCubes technical README](https://github.com/Cadiboo/NoCubes/blob/38904f71b818a1f4770c5253b9a1dd522201babc/README.md) (LGPL-3.0 repository) | Surface Nets for voxel surfaces, with render and collision consumers of the same surface. Chunk meshes rebuild on change. | Retain shared height/CBT geometry and dirty publication; introducing a second voxel world would not fix material detail. Surface Nets may be relevant to future volumetric caves, not the current heightfield. No code or dependency added. |
| [Complementary generated normals](https://github.com/ComplementaryDevelopment/ComplementaryReimagined/blob/c09950df0650b27dac260c1b0f69afc11387aa6f/shaders/lib/materials/materialMethods/generatedNormals.glsl) and [license](https://github.com/ComplementaryDevelopment/ComplementaryReimagined/blob/c09950df0650b27dac260c1b0f69afc11387aa6f/License.txt) | Footprint-aware finite differences avoid taking unresolved offsets from the texture atlas. | Reuse our physical footprint filtering, but do not infer rock relief from albedo brightness: snow/soil colour is not height. Restrictive custom license: no source copied. |
| [LabPBR material standard](https://shaderlabs.org/wiki/LabPBR_Material_Standard) | Separate normal, relief, roughness/reflectance and wetness information instead of treating colour as geometry. | Conceptual reference only, not a LabPBR compatibility claim. Keep existing SI/native material conventions; do not import its DirectX-oriented tangent convention or texture-relative displacement depth as engine semantics. No textures or wiki text copied. |
| [Mikkelsen, Surface Gradient–Based Bump Mapping Framework, JCGT 9(3), 2020](https://jcgt.org/published/0009/03/04/) | Linear composition of height gradients, followed by one normal resolve. | Applied here to mip construction, residency mixing, and tangent-plane projection. Derivation implemented independently; this is the mathematical reference, not a port of a Minecraft shader. |
| [Practical Real-Time Hex-Tiling, JCGT 11(3), 2022](https://jcgt.org/published/0011/03/05/) | Randomized hex-domain texture reuse, structure-preserving blend weights and normal handling. | Candidate for future lawful material exemplars to avoid obvious repeats without a planet-sized texture. Not implemented: current world-addressed procedural pages do not tile an exemplar, so adding three texture reads now would solve the wrong problem. |
| [OpenSimplex2](https://github.com/KdotJPG/OpenSimplex2) (CC0; some marked legacy files use Unlicense) | Lattice/gradient-set symmetry and orientation are explicit noise-quality considerations; 3D uses a rotated BCC lattice. | Candidate against the current gradient-noise baseline if directional artifacts persist. Do not replace the world generator or add a noise dependency merely on the name; first compare isotropy, filtered derivatives and page throughput. Not imported in this change. |
| [Fast Biome Blending, Without Squareness](https://noiseposti.ng/posts/2021-03-13-Fast-Biome-Blending-Without-Squareness.html), KdotJPG, 2021 | Jittered triangular sample points plus normalized compact-support sparse convolution; query a padded chunk once, but never seed the world by that chunk. Skip work when all contributors agree. | Candidate for continuous material/ecotone weights, not a replacement for canonical climate or categorical geology. Keep weights summing to one and source samples world-addressed; a spherical implementation must explicitly handle chart joins. Reuse existing region/scatter descriptors rather than add a second point registry. Not implemented or copied; the article's Java benchmark is not Thessa performance evidence. |

Next substantive appearance work should share the existing geology,
`surface_appearance_filtered` exposure/deposition logic, and
`ecology::vegetation_cover` ground/canopy/reed fields. Distinct substrate relief
and coherent rock/soil boundaries are preferable to another planet-wide noise
gain. Geometry/scatter for plants and boulders must then preserve those source
identities across density/LOD tiers, participate in shared shadows, and avoid
duplicating the existing deterministic scatter descriptors. These are follow-up
tasks, not implemented vegetation, new collision relief, hex-tiling, or POM.

In particular, the current `scatter::scatter_distribution` is tile-seeded and
changes its grid with requested density. Identical inputs are deterministic,
but a different tile level/density is **not** a stable subset of the same
world instances. Native plant/rock rendering must first extend/reuse that
contract (or the existing frozen regional descriptors) with persistent
world-space candidate identity and hash-threshold density subsets; it must
not simply rerun the tile kernel at each visual LOD and call the result stable.
