# 68 — Native inhabited-land night-light consumer

**Status: implemented primary-terrain optical slice. Updated: 2026-10-20.**

This is a direct GPL-to-GPL port of the retained city-mask producer into
`apps/terrain-native/src/night_lights.rs`, not a settlement/electricity model.
The existing MIT `rcbt-core` terrain fragment shader already owns primary
surface emission; native `render-wgpu::TerrainRenderer` now binds the supplied
R8 mask instead of its black placeholder. No new pass, light, ray scene,
force/state path or simulation clock is introduced.

## Inputs and publication

The game reads the existing immutable world's `inhabited_regions`: population,
suitability and body-fixed centre directions. It uses exactly the retained
population-weighted smooth settlement envelope, 0.012 rad base radius plus
0.05 rad population-share reach, deterministic speckle hashes and a land gate
from the same world's `sample_surface(dir, 64000 m).height_m > 5 m`. These are
the old optical-proxy parameters, not calibrated city dimensions or watts.

The output is a static equirectangular R8 mask in the terrain globe convention:
east-positive longitude `atan2(-z,x)`, north at v=0. Native uses repeat-U /
clamp-V sampling, including filtering across the zero-meridian seam. The world
is immutable for a Scene session; the CPU mask is cached by extent and reused
when the GPU renderer is recreated. Quality changes rebake; disable/zero gain
release the source cache at renderer creation. No per-frame bake or upload is
performed, nor does the presentation mutate settlement or flight state.

`RequestedGraphics -> capability resolution -> ResolvedGraphicsSettings` owns
the enable/gain/tier budget. `night_light_mask_size()` is the shared resolved
budget, consumed by both producer and renderer:

| State/tier | R8 extent | CPU mask bytes | GPU texel bytes | Startup upload |
|---|---:|---:|---:|---:|
| disabled or intensity=0 | none | 0 | 0 | 0 |
| Low | 128×64 | 8192 | 8192 | 8192 |
| Medium | 256×128 | 32768 | 32768 | 32768 |
| High | 512×256 | 131072 | 131072 | 131072 |

Texture allocation overhead is not included in texel-byte counters. The
disabled path reuses the existing one-texel black binding needed by the shared
shader; it creates no effect texture, bake, upload, prepass or architecture
switch. An allocated but unpublished mask stays dark. Full length is validated
before any texture/uniform publication; malformed updates leave the previous
mask active. Normal settings edits require restart, as declared in metadata.

## Primary render contract and limitations

Regular and conforming terrain both consume the same shared fragment shader.
Its retained authored warm emission is `(1,0.72,0.42) * mask * night * gain * 500`
in pre-exposure linear RGB. `night` retains the old first-light-slot smooth
terminator gate (-0.15..0.12 dot product). This is not multi-source total
illumination, eclipse-aware lighting, automatic electrical activity or a
calibrated radiance law. Slot ordering belongs to the existing celestial
lighting producer. The mask follows body-fixed geometry as the light and body
rotate; quality changes resolution, not settlement existence or physics.

There is no separate mesh-city geometry and no city emission in secondary RT
hits, ocean/mesh reflections or indirect-light transport. These are explicit
limitations, not hidden enabled effects. Native day-side near-ground captures
may show no visible city contribution by design. End-to-end CLI tests check
source/upload/physics boundaries; the numerical GPU fixture independently
isolates night-side emission. Wider whole-frame temporal/visual acceptance is
not implied by either test.

## Counters, budget and verification

`TerrainRenderer::night_lights_stats()` exposes extent, cumulative uploaded
bytes and allocated texel bytes. The actual headless host prints them as
`native_night_lights`; startup logs record source-region count, nonzero texels
and elapsed bake time. Capture metadata records resolved extent and the
primary-only optical-source boundary. No GPU scope-layout change is needed:
the existing scene classify/raster interval includes the single additional
night-side texture sample; there is no additional pass or query in normal use.

Tests cover deterministic/speckled/empty/ocean masks, population/distance and
UV mapping, all tiers and zero-work disabled settings. An independently
compiled retained-versus-native 18-metro all-land producer fixture matched
every byte at every tier; its FNV64 results are pinned in native tests.
Those hashes establish retained-producer parity for that fixture, not a new
physical reference. The production-world CLI test uses real source data and
verifies one startup bake/upload with identical final authoritative telemetry
for off/zero/Low/Medium/High modes, with silent audio.

GPU tests use actual hardware: constant-mask night-side linear RGB is checked
within 2/255 per channel on regular and conforming terrain at all three tiers;
daylight central samples match a dark mask; rejected publications preserve the
central sample and counters. Conforming silhouettes continue independent LOD
refinement, so the test does not assert false whole-frame pixel identity.
Longitude orientation, wrapped seam filtering, direct-light updates without
reupload, HDR pipeline admission and disabled allocation are also covered.

The CPU diagnostic times one startup bake of 18 all-land metros at each tier,
excluding terrain sampling. The GPU diagnostic compares actual timestamped
classify/raster intervals for the same isolated regular sphere at 1280×720,
four warmups plus 32 samples, disabled versus High constant mask. It includes
fixed raster cost and is not mask-only cost, a whole-client benchmark or a
performance-win claim. Production CPU sampling cost is reported separately.
Executed gates, hashes, timings and known omissions:
`artifacts/native-night-lights-2026-10-20.json`.
