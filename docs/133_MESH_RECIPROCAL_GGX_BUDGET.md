# 133 — Mesh GGX energy and black foliage patches

Status: shared mesh correction verified on the physical GPU, 2026-11-04;
baseline `efeab0a`. The shared mesh material response is corrected. This is not forest-scene
parity, a new leaf-scattering lobe, a general reciprocal shading-normal solution,
or production performance acceptance.

## Reference research and applicability review

[Filament's material reference](https://google.github.io/filament/main/filament.html)
separates directional GGX response/DFG from an individual Fresnel evaluation.
Thessa already has the corresponding directional-response quadrature in
`environment.wgsl`. More importantly, the accepted terrain contract in
[doc 69](69_NATIVE_TERRAIN_MATERIAL_LIGHTING.md#grazing-audit-finite-visibility-ratio-and-coupled-material-energy)
requires incoming AND outgoing first-substrate transmission. No external code,
new illumination source, palette/exposure gain, or new subsystem is needed.

The actual living-fern control uses three native forms and normal strengths
1, 0.35 and 0. All nine baseline images were inspected: disabling the normal map
does not remove the common black patches. A physical-GPU white-furnace regression
then reproduces complete extinction: Medium, roughness 0.2, shading `N.V=-0.2`
returns RGB8 `[0,0,0]` under nonzero uniform white ambient.

The mesh removed diffuse with `Schlick(N.V)` while adding rough specular with a
different hemispherical GGX response. In particular, a shading normal behind the
view gives no supported specular lobe, yet the former diffuse debit reaches one.
The result is a black patch unrelated to the source albedo/alpha contour.

An outgoing-only complement initially removed the black patches and passed an
exact-white test. It was rejected after the terrain-contract audit: exact unity
in that test does not establish reciprocity or bounded directional energy. Its
v2/v3 images are intermediate diagnostics, not accepted final evidence.

## Implementation

Medium/High, with reflections enabled, use the existing vector GGX directional
reflectance `R` for both interfaces. Each direct source receives
`albedo * (1-R(view)) * (1-R(light)) / pi`, in addition to the existing metallic,
reflection/transmission partition, cosine and visibility terms. Back-side leaf
illumination uses the normal oriented toward that source for the incoming
interface. No extra ambient-occlusion multiplier is introduced.

Aggregate diffuse sky uses the existing cosine-weighted average incoming response:
`albedo * (1-R(view)) * (1-average_R)`. This retains the terrain path's isotropy
approximation for its aggregate incoming energy. The existing SH9 sky illumination
and per-source atmosphere/cloud/shadow transport remain their own owners.

The environment/ray hook returns reflected radiance AND its existing unit-source
response, so primary outgoing reflectance reuses the already evaluated samples.
Medium/High ray chains use the same response for their first weight, secondary
incoming/outgoing diffuse factors and specular continuation. Primary misses retain
exact raster fallback. The single mirror-direction approximation, roughness cutoff,
finite bounce count and AS policy remain; this is not a stochastic GGX path tracer.

Low retains its previous cheap model: full Lambert when reflections are disabled,
F0 diffuse debit and F0 ray weights when enabled. Medium/High reflections-off keep
the cheap F0 outgoing debit and no response quadrature. Pure metals skip incoming
substrate integration because their diffuse contribution is exactly zero.

## Cost and limits

No new texture, storage buffer, GPU pass, material/vertex ABI, or runtime setting
is introduced. Primary outgoing response adds no new VNDF samples. A contributing
Medium/High diffuse surface adds `S` material weights per active source and `4S`
for nonzero aggregate ambient, using the existing quality sample count. The maximum
is 7S extra weights (56/112 for Medium/High); Low and reflections-off add none.
Secondary RT hits have corresponding bounded material work per hit. This cost is
meaningful; no speed improvement is claimed and a release timing remains required.

The model omits internal substrate-return bounces and existing GGX multiscattering.
Its white-dielectric furnace therefore has the intentional deficit
`R(view)+(1-R(view))*(1-average_R)`, rather than exact unity. The direct substrate
factor is reciprocal on its valid shading hemisphere; geometric/shading-normal
mismatch and general anisotropic-sky transport are not solved by this correction.

## Verification

The raster suite includes the furnace deficit against independent f64 solid-angle
integration (two qualities, three roughnesses, four view cosines and three diffuse
transmission values), colored/metallic response, explicit Low/reflections-off
fallbacks, and a direct-light high-F0 front/back leaf oracle. That direct oracle
requires both interface factors, so an outgoing-only implementation cannot pass
by matching an isotropic sky.

The pre-existing terrain Simpson/solid-angle reference is shared by test targets;
its terrain resolution is preserved. RT radiance expectations are derived from
that same physical law and the independent smooth-mirror limit of the existing
Schlick-Smith visibility. The original two-code tolerance is retained. It is not
valid to keep the former unit-throughput mirror/Schlick-only expected values
when testing the changed response.

Physical-GPU checks on the isolated RADV render node passed: 9 raster lighting
cases, 32 ray-scene cases, 43 renderer unit cases and the existing 27-case
substrate energy/reciprocity probe. The probe's maximum integrated outgoing energy
was 0.985663 and maximum specular reciprocity error was 0.000069006, within its
unchanged bounds. The ignored ambient-visibility integration also passed all
12 combinations of reflections off/on, MSAA 1/4 and 8/16/32 probes, preserving
its direct/emission/depth/round-trip assertions. A high-F0 secondary-hit regression also caught and corrected accidental incoming
substrate attenuation of the specular fallback source (162 vs 165 RGB codes
before correction, unchanged two-code tolerance). Native package tests passed
153 cases (51 ignored). Strict all-target/all-feature Clippy and the architecture
gate passed. These are local checks, not a claim of green remote CI.

All nine final living-fern specimen images were inspected by the author and an
independent Luna reviewer. Against the identical baseline silhouette, near-black
pixels (all RGB codes below 8) in the source-normal variants fell from
491/1328/1523 to zero for forms 0/1/2. The reduced/geometric-normal controls also
lost their black patches. This is an unblinded causal bug comparison, not a blind
preference experiment. The actual FHD 0.3 m native scene was inspected at full
resolution: the prominent foreground leaf patches are removed, while soil remains
visibly smeared. Timed streaming makes this scene capture unsuitable for exact
whole-frame A/B claims. The wide FHD capture and all 20 frozen diagnostic views were also inspected;
depth, published geometry and beauty round trip remain identical within that run.
The evidence inventory records source/binary/image hashes
and the rejected outgoing-only intermediates.

See [the machine-readable record](artifacts/mesh-reciprocal-ggx-2026-11-04.json).
Release GPU timing, general mapped-normal reciprocity, directional leaf scattering
and forest/reference acceptance remain open.
