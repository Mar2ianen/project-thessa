# 59 — Native manual HDR scene and SDR display

**Status (2026-10-18): implemented native manual-exposure slice; acceptance is
bounded by the checks and measurements recorded below.**

The previous native path applied exposure in terrain/mesh/sky and stored the
result directly in an 8-bit sRGB attachment. Values above one were clipped
before any display operator could preserve highlight differences. This slice
adds a float intermediate and one global display operator, not another lighting,
environment, reflection, graphics-policy or camera subsystem.

## Contract and settings

The existing flow remains `RequestedGraphics -> active-backend capabilities ->
ResolvedGraphicsSettings -> renderer`:

- `[renderer].hdr = true`: when RGBA16Float supports both render attachment and
  texture binding, draw the shared scene into that format and apply per-channel
  **Reinhard `x / (1+x)`** to the SDR output. This is the implemented higher
  fidelity representation; the per-channel global curve is not a complete
  photographic/scene-referred color appearance model or an AgX/ACES match.
- `hdr = false`: lower-cost direct LDR with the original output format and
  pipeline, **no intermediate target/bind group/pipeline/post pass**, no tone map,
  no extra per-frame work except an inactive branch. Bright highlights clip as
  before. No physical lighting/state is removed; the representation is cheaper.
- A known unsupported float target resolves to this direct fallback with a
  diagnostic note. Capabilities come from adapter format usages, not OS/vendor.
  Pre-init/legacy callers without a native probe retain their requested HDR
  policy; native callers always use the active adapter before constructing it.
- `[renderer].exposure_ev100` retains `E = 1 / (1.2 * 2^EV100)` and is applied
  **once** by the existing scene shaders. Resolution clamps finite input to
  `[-16, 32]`; NaN/infinity use EV13. The native HUD's daylight slider remains
  `1..20`; TOML supports the wider validated range. `render-core::display`
  contains the backend-neutral math, not graphics handles or policy.
- HDR is a restart-required setting. `[renderer].auto_exposure` and `bloom`
  continue to describe shared/legacy requests but are **not applied** by this
  native manual slice. The checked-in config already requests manual exposure.
  Native logs/capture metadata say `native_auto_exposure_applied=false` and
  `native_bloom_applied=false`, even when requested true. Metering, adaptation
  timing/cut/reset semantics and bloom remain future work, not hidden toggles.

## Rendering and lifecycle

`render-wgpu::display::DisplayTransform` selects the existing scene format, owns
only the optional float color view/pipeline/binding, and borrows the caller's
SDR output. It shares the existing device/queue and existing scene depth target.
The pass order is **shadow depth -> AS preparation -> classification/shared
terrain+opaque-mesh+sky raster -> display transform -> egui/FastNavball**.
No shader applies exposure twice. Mesh reflections, ocean and sky still consume
the same semantic lighting/environment state and quality settings.

The source is **pre-exposed linear RGB**, not a gamma-encoded image or an
unexposed radiance buffer. `textureLoad` samples exact physical pixels with no
sampler, mip chain, filtering or intermediate scale. sRGB attachments encode
once in hardware; supported non-sRGB RGBA8/BGRA8 SDR attachments use the same
piecewise sRGB transfer in the shader. Native window negotiation continues to
require sRGB. This slice does not negotiate a 10-bit/float HDR-monitor colorspace.

Negative/invalid source components display black; positive overflows are
contained to the half-float maximum before mapping. Finite pre-exposed highlights
up to **65,504** retain the half-float representation; brighter values saturate,
and subnormal/near-zero storage has finite precision. This is not arbitrary
dynamic range or an absolute physical-radiance calibration. For normal finite
half-floats, storage relative rounding is at most about **2^-11**; the global
curve derivative is `1/(1+x)^2`. The hardware pattern oracle pins the combined
storage/operator/sRGB/8-bit result within **one output code value per channel**
at black, transfer toe, middle gray, unity, 4×/16× highlights, colored channels
and the finite storage limit. It does not prove all possible shader inputs or a
perceptual error envelope for a production scene.

HUD and GPU navball remain display-referred, native-resolution consumers of the
final SDR target, not the scene float target. Their palette/exposure never
depends on world brightness. An egui overlay oracle compares opaque UI colors
exactly with HDR off/on over black and pins UI white at 255, not Reinhard gray.

Resize changes only the float view/binding; the pipeline, world geometry,
shadow resources and navball remain retained. Same-size changes do nothing.
Invalid/zero extents leave the previous display resources intact; the window
already skips drawing while minimized. Window recreation rebuilds display
resources on its new backend as it does the other render resources. Disabling
requires restart/recreation and releases them; it cannot silently switch the
global render architecture just because a plugin was registered.

## Budgets, counters and verification

HDR adds **8 bytes × physical width × physical height** of persistent color
storage (7.03 MiB at 1280×720, 15.82 MiB at 1920×1080, 28.13 MiB at 2560×1440),
plus small pipeline/binding overhead. It reuses depth rather than allocating a
second depth buffer. There is one additional fullscreen read/write pass, no
per-frame uniform upload, CPU image, readback, adaptation buffer or target churn.
The initial diagnostic budget is **<0.5 ms GPU at 1920×1080** on the development
880M for this pass alone, not a cross-platform guarantee or a runtime controller.

`native_hdr`, `native_hdr_target_bytes`, `native_scene_format` and
`native_tonemapping` are exported next to the unapplied-feature flags.
`gpu_scope_layout=native-display-v3` brackets **shadows /AS /scene raster
/display transform /UI** with six marks. The disabled display interval contains
only timestamp/scheduling overhead, not a meaningful shader cost. The changed
scene format/fidelity means old v2 frame timings are not matched-quality A/B.

Tests cover resolver fallback/invalid EV, CPU exposure/curve special cases,
actual GPU pixel/transfer/highlight oracles, odd/non-square sizes, shrink/grow,
invalid resize, paused determinism and exact disabled/direct identity. The
existing split/unified scene fixture also runs in HDR with RT/ocean off/on and
manual exposure change/rewind; exact pixels pin pass composition, not a full
physical atmosphere/ocean approximation. The release cost diagnostic measures
the display pass alone at three native resolutions, after 30 warmup frames,
200 samples each; its CPU wait includes submit/GPU completion and is not CPU
shader time. It excludes world raster/UI/setup/readback and does not claim FPS.

On the development Radeon 880M /RADV Vulkan, the final standalone pass measured
**0.151 /0.362 /0.622 ms GPU median** at 720p /1080p /1440p; 1080p p99 was
**0.470 ms**. These are added display-quality costs, not a renderer speedup.
Final gates passed **1,870 workspace lib/bin +90 integration tests**,
workspace and native no-default clippy, fmt, **24 Bevy-CBT GPU +14 native ignored**
checks, the dual-HDR-state perf-capture and origin-cache regressions, and release
display/HDR-scene oracles. Four silent native flight captures reached identical
authority time/body position. In the spacecraft captures, **54,689 opaque sphere
interior pixels** were exactly unchanged between HDR/LDR and EV13/EV12, while the
world changed. This excludes the antialiased rim and translucent panel, and is
integration evidence rather than blind aesthetic acceptance.

Actual commands, checks and performance/capture results are recorded in
[`artifacts/native-hdr-display-2026-10-18.json`](artifacts/native-hdr-display-2026-10-18.json).
No blind subjective preference, legacy tonemapper match, cross-platform runtime
acceptance, production renderer replacement, auto exposure or bloom is claimed.
