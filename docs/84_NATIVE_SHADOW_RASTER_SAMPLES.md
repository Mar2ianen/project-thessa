# 84 — Native empty shadow sample selection

Status: feature-branch continuation, 2026-10-28. Baseline `b9b178a`.
Extends [the shadow geometry hierarchy](83_NATIVE_SHADOW_HIERARCHY.md).
Physical desktop GPU replay remains blocked after the recorded reset incident.

## Contract

`shadows.raster_sample_culling` defaults to false; the surface showcase requests
true. The graphics resolver requires a capability guaranteeing centered 1x
triangle samples and a framebuffer subpixel precision floor. The native wgpu
probe supplies four fractional bits only for the Vulkan API contract. Unknown
backends, the legacy Bevy host, insufficient precision and disabled shadows use
the existing caster path. Captures expose the resolved optional per-axis vertex
snapping guard (`shadow_raster_sample_guard_texels`); fallback notes state when
precision is unknown. The optimization also applies with RT active because it
preserves the entire shadow atlas, including off-screen receivers.

The mesh depth pipelines use filled triangles, one sample and ordinary
(non-conservative) rasterization. Each primitive's original bounding sphere is
expanded to enclose every actual replacement vertex, independently of the
producer's approximation certificate. The expansion is computed once when LODs
are installed; cached uploaded-world bounds already enclose narrowed instance
transforms, nonuniform scale/shear and their f32 arithmetic. The maximum expansion
across all supplied levels encloses every possible selection, including full
geometry. Missing or invalid bounds retain the full caster.

The backend-neutral `render_core::shadow::RasterFootprint` projects this sphere
into an enclosing circle using a Gershgorin upper bound on the XY operator's
largest singular value. It adds per-axis f32 clip/viewport arithmetic and raster
vertex snapping envelopes. A caster can be omitted only if the closest shadow
sample lies strictly outside the circle plus those envelopes. Equality retains
the caster. The expanded circle is convex, so it encloses the filled triangles,
including clipped triangles. A cheap radius test retains large footprints before
performing sample-grid arithmetic. Invalid/nonfinite inputs fail open. This is
independent of receiver-volume selection, so the existing spectral-ocean full
receiver-volume fallback remains.

No shader, triangle source, population, material, bias, resolution, light, primary
or RT representation changes. There is no GPU allocation, pass or readback added.
Existing direct/indirect submission, twelve disjoint command regions, shared
memory admission, dirty upload reuse and cache invalidation remain. Disabled
sample selection skips footprint construction and extra per-instance math.
`mesh_sample_rejected_instances` counts primitive/instance/layer uses, including
uses already outside the clip volume; it is not a unique population count or a
standalone measure of triangles saved.

`TerrainRenderer::encode_shadows_without_sample_culling` is a diagnostic oracle:
it keeps the same source levels, clip and receiver selection and disables only
sample-grid rejection. It invalidates the depth cache before and after use.
The existing full-source oracle still disables LOD and all culling. These oracle
scopes are distinct; production callers use the ordinary cached path.

## Verification boundary

Core tests cover empty grid cells, touching samples, atlas edges, invalid inputs
and 28,800 enclosed-sample cases across light directions, map sizes, cascade
radii and astronomical origins. A backend test verifies proxy-vertex enclosure
under actual narrowed nonuniform/sheared transforms.

The direct/indirect regression compares every depth byte in twelve layers and
final colour against the no-sample oracle at two origins. It must witness sample
rejections and full geometry plus both alternate levels, and retains its dirty
reuse, capability/budget fallback and reservation release assertions. Actual
plant, isolated reed and combined original-tier fixtures additionally compare
same-LOD sample selection against no-sample depth/colour exactly; their previous
quality thresholds and distinct full-source/before-after references remain.
The LOD-selection witness is checked in the no-sample oracle, because exact
sample rejection can legitimately remove every eligible proxy draw. An initial
harness assertion demanding a surviving proxy draw failed in the isolated-reed
case after its exact depth/colour comparison passed; that attempt is retained in
the evidence separately. No image tolerance or production algorithm was changed
for the harness correction.
The full-source zero-error and primary-receiver integration cases also request
sample selection and retain their original exact comparisons.

The canonical coastal probe encodes and drops dense draw commands. It never
submits dense raster or acceleration-structure work. Same-build oracles retain
the preceding hierarchy count of 252,050,732 triangles and the original producer
count of 283,223,492. Actual hardware GPU frame time, temporal stability across
moving cameras and independent reference acceptance remain unmeasured. TAA
stays default-off.

## Primary sources

The [Vulkan rasterization specification](https://docs.vulkan.org/spec/latest/chapters/primsrast.html)
defines a centered sample for single-sample rasterization and ordinary filled
triangle coverage. The [Vulkan required limits](https://docs.vulkan.org/spec/latest/chapters/limits.html)
require at least four framebuffer subpixel precision bits. The implementation
uses a conservative one-grid-step (1/16 pixel) vertex snapping allowance rather
than assuming exact floating-point coverage. API contract selection is confined
to the backend probe; the graphics and mathematical contracts contain no Vulkan
handles or dependencies. No third-party source was copied.

## Final fixed-scene evidence

The final same-build coastal plan contains **208,101,356** mesh shadow triangles,
43,949,376 fewer than the 252,050,732 hierarchy oracle (**17.44%**). The original
one-tier producer oracle reproduces 283,223,492. Source payload stays 46,159,116
bytes below 48 MB; unique shadow geometry is 1,657,464 bytes. The 17,992 retained
mesh instances, 17,111,160 primary triangles, 37 primary API batches, 576-byte
frozen primary update, 1,784,312-byte world-bound cache and 7,864,320-byte command
allocation stay unchanged. The rejection counter records 130,587 primitive/
instance/layer uses, not that many unique plants.

Indirect logical commands increase from 16,755 to 26,925 and dirty command bytes
from 335,100 to 538,500 as holes split runs. API batches change from 543 to 540.
The command limit remains one command per instance/layer under the existing
reservation. Geometry reduction therefore does not certify a frame-time gain.
Cold shadow work remains large, including approximately 59.38M triangles across
the three first cascades.

All **13** final gates pass, including strict all-target/all-feature Clippy,
release, feature-off, architecture and legacy Bevy-client compilation. **524**
CPU tests pass: 133 native, 287 worldgen, 51 graphics, 47 render-core and six
bounds/projection cases. Seven actual isolated software render regressions and
one dropped-encoder coastal probe pass. Same-LOD sample/no-sample depth and
colour are exact in all plant cases. The isolated reeds reject 20–33 uses/case;
the combined fixture rejects 32–57. The original six-form grouped fixture has
zero sample rejections and retains its unchanged quality result (max 9/255 RGB,
max mean 0.04453/255, max unmatched component two). These are distinct witnesses.

The [artifact](artifacts/native-shadow-raster-samples-2026-10-28.json) records
commands, hashes, driver/containment, full per-case results, counts and omissions.
Source manifest: `de80d052c217096f05875a0216089fd899ed1cfe93689949d08251bd2c99496f`.
Release binary: `2d5a875012cb0f233c96a9d0842cbd3c651c03a8b7bca9b6d97661e18884379f`.
