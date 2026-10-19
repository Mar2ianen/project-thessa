# 57 — Native flight deck and FastNavball

**Status (2026-10-19): implemented native instrument/readout slice, not full pilot/UI parity.**
The native HUD now has separate aircraft and spacecraft layouts and an app-owned,
native-pixel GPU attitude sphere. The legacy Bevy HUD remains runnable and retains
its shared CPU128 instrument. No external instrument code, KSA/Apollo artwork,
font, dependency or unsafe code was imported. The spacecraft layout is an original
FDAI-inspired black/white display, not an Apollo gimbal/error-needle simulation.

## Ownership and frame contract

- GPL `flight-authority::navball::InstrumentFrame` supplies a pure f64 orthonormal
  surface frame and hemisphere projection. Existing `navball_pixels` is unchanged.
- Native `Scene::flight_data` supplies local up, the body's rotation pole and
  surface-relative velocity in vehicle coordinates, sampled from the same shared
  kinematics/authority state as speed and altitude. Body axes are nose **+X**,
  screen-right **−Y**, screen-up **+Z**. No camera orientation drives the instrument.
- Both hosts now consume `flight-authority::instruments::sample`: a read-only
  newest-authority SI/f64 sample in the dominant-pull display-body frame. The
  solver reference, summed gravity, state and control laws do not change. Native
  AGL is added only for matching home terrain and uses the exact inertial sample
  at its own epoch, not cached/interpolated render coordinates. Acoustics still
  samples the source's reference atmosphere, never the instrument display body.
- Projected north is the rotation pole projected onto the tangent plane; east is
  `north × up`. A pole/vertical-nose degeneracy explicitly makes heading unavailable.
  A deterministic tangent keeps the disk finite; it is not a fabricated compass
  measurement. Zero/invalid velocity omits prograde/retrograde cues. Rear-hemisphere
  markers are omitted rather than reflected onto the visible disk.
- Native server instruments retain the existing **newest acknowledged replica**
  boundary, while the world uses its interpolated presentation pose. They are not
  claimed to be time-identical. A stopped-flight label marks frozen data.
- `apps/terrain-native/src/fastnavball.{rs,wgsl}` owns GPU resources in the existing
  egui renderer's callback resources. GPL game code does not migrate into MIT
  renderer crates. There is no second device, scene readback, intermediate scene
  image, per-attitude texture, or second generic UI framework.

## Representation alternatives

Three alternatives were exercised, with deliberately different coverage:

1. **Existing CPU128 baseline:** generates a 128² RGBA attitude image, historically
   rebuilt at most 30 Hz after local-up travel. It does not represent heading and
   uploads 65,536 bytes on a rebuild. It stays in the legacy client, not as a hidden
   production native fallback.
2. **Analytic GPU (selected by `auto`):** one six-vertex quad in the existing UI
   pass. Each physical output pixel projects the sphere, transforms into its live
   frame and evaluates derivative-filtered graticules, horizon and palette. No
   instrument texture allocation. A changed attitude/palette uploads **48 bytes**;
   an unchanged frame uploads nothing. No wall/simulation refresh timer gates it.
3. **Baked GPU (explicit option):** identical sphere/frame/projection, sampling a
   one-time **R8 static graticule**, with area-reduced mips and wrapped longitude.
   Labels/markers remain live vectors/glyphs, not baked attitude pixels. The static
   angular coverage and the analytic derivative line widths are not pixel-identical;
   this is a cost/representation experiment, not a quality-matched speed claim.

Auto chooses analytic because it avoids atlas preparation/storage and baking did
not show a stable advantage at both measured extents. The baked variant remains
selectable for controlled comparisons. Neither path rebuilds a 128 px image and
stretches it to a larger HUD. The callback's viewport is the actual physical size
after DPI scaling; ordinary egui clipping applies.

Bezel, roll ticks, heading/pitch labels, fixed craft reticle and surface prograde/
retrograde/radial-up markers use the existing egui vector/font path. Rotation about
local up therefore changes heading even when the old local-up-only raster would
remain unchanged. There are no target markers, orbital-normal markers, inertial
reference selector, rendezvous errors or maneuver nodes without their semantic
inputs. The disk is a surface-referenced attitude indicator, not a complete
navigation/autopilot system.

## Layouts and controls

- **Aircraft:** blue/brown sphere, selected speed/altitude tapes,
  datum/AGL/vertical-speed readouts, throttle, engine/SAS and RCS/gear controls.
   Missing speed/altitude omits the tape; no invented IAS or radar sensor.
- **Spacecraft / FDAI:** black/white sphere, scalar vertical/horizontal surface
  velocity decomposition, altitude, explicit pitch/roll, throttle and RCS/reaction
  wheel switches. Undefined vertical-nose Euler roll is displayed as unavailable.
  `horizontal² = max(surface_speed² − vertical_speed², 0)`; no orbital quantity is
  inferred from that number. `SIM` is the current simulation sample since epoch,
  not a separately defined mission/launch timer.
- Layout buttons change **display only**. They never select a spacecraft control
  law, change authoritative physics, spawn a rocket or reinterpret stick axes.
- Native session-only selectors choose **Surface / Air / Orbital / Target**
  speed and velocity cues, plus **Datum / AGL** altitude. Attitude remains local
  in every case: selecting Orbital changes the velocity marker, not the ball's
  attitude frame. Target has no supplied semantic input and is unavailable.
  Vertical/horizontal spacecraft decomposition remains explicitly surface-
  referenced regardless of the selected scalar speed. Preferences persist in
  the HUD view until recreation, not graphics.toml or server commands.
- Away from the launch body the legacy body-relative fallback is retained and
  labelled: Surface has no installed foreign rotation provider. The launch
  body's air/rotation/terrain cannot be applied to the display body. AGL is
  unavailable there; the local grid uses the baked system's +Z polar axis.
- Mach, dynamic pressure, angle of attack/sideslip, thrust, central-body TWR and
  non-gravitational g-load have native readouts. Expand **Air / orbit / subsystem
  details** for pressure/density, instantaneous two-body Ap/Pe, actual wheel
  torque and canopy phase/load summary. A sub-datum Pe is a diagnostic, not a
  solved impact or JIT curve. No new target, sensor, force or atmosphere model.
- Air/force fields are local-authority observations only. Remote snapshots do
  not transmit them or sufficient resources for authoritative reconstruction;
  display-only `last_forces` on a replica is deliberately ignored. Remote wheel
  torque and checked canopy loads come from the newest accepted snapshot. A
  malformed/mismatched canopy vector becomes unavailable, never the previous
  replica-local load. Both hosts reuse the authority's existing canopy validator.
- The existing detailed migration/debug/control window is collapsed by default
  during flight, and remains available. Survey/fleet controls remain intact.
- `--hud`/`--no-hud` retain their host meaning. `[instruments] enabled=false`
  disables only the flight deck, not debugging/flight commands. No-data/disabled
  paths remove its callback resource and do no instrument GPU work.

Requested/resolved policy lives in `crates/graphics`, including the shared
`resolve_instruments` helper used by the full resolver and native HUD. All modes
require only baseline portable fragment/uniform support; baking additionally
uses filtered R8 textures within the baseline 2048 dimension limit.

```toml
[instruments]
enabled = true
quality = "high"
layout = "aircraft"           # aircraft | rocket
navball_renderer = "auto"     # auto | analytic | baked
```

| Quality | Logical diameter | Optional static atlas | R8 mip data bytes |
|---|---:|---:|---:|
| Low | 192 points | 512×256 | 174,763 |
| Medium | 256 points | 1024×512 | 699,051 |
| High | 320 points | 2048×1024 | 2,796,203 |

The disk uses 88% of the logical widget diameter, leaving a bezel/tick margin.
Window/DPI scale and available area determine physical raster size; these are not
offscreen-resolution caps. Quality/presets do not change frame math or telemetry.
The local diagnostic budget is **0.1 ms GPU clear+quad at 512²** on the 880M;
`navball_cost_diagnostic` reports hardware p50/p95 and host preparation/submission
cost rather than enforcing a noisy timing assertion. The full application's
`render.ui` GPU scope and resolved layout/diameter/atlas metadata are recorded by
the existing perf-capture path. These are not whole-frame or cross-device budgets.
The normal desktop deck is tested from 800×600 through 1280×900 at 1×/1.5× DPI;
very small/mobile layouts and accessibility review remain future work.

## Evidence and verification

On Radeon 880M/RADV, the final sequential release diagnostic measured the old
**CPU128 generation** at 0.391 ms median (no upload/draw included). Analytic
**GPU clear+quad** medians were 0.0089 /0.0293 ms at 256²/512²; baked was
0.0091 /0.0419 ms. These are different
CPU/GPU scopes and different quality from CPU128, not an end-to-end speedup ratio.
The baked High atlas costs ~2.67 MiB plus about 37–40 ms host setup in this run;
pipeline compilation/caches and clocks are uncontrolled.

Silent sequential native flight captures with 120 measured frames (30 warmup,
10 seconds frozen pre-stream) measured **whole egui UI command intervals** of
~0.0389 /0.0369 /0.0382 ms for aircraft analytic /spacecraft analytic /spacecraft
baked, versus ~0.0169 ms with the flight deck disabled. Those intervals include
other HUD work; host frame/stream/authority variance prevents a stable full-frame
speedup claim. All captures end at the same 2.5 s authority time and body position.
Parallel renderer batching changes in this working tree are not isolated by
these captures and are not attributed to FastNavball. Their subsequent review
and profiling-boundary correction are documented separately in [doc 58](58_NATIVE_PIPELINE_REVIEW.md).

Tests cover frame axes/heading, pole/vertical/invalid fallbacks, hidden/stopped
velocity markers, static atlas mip completeness/wrap, live GPU response to all
three attitude axes, deterministic pause/rewind, both palettes and 256²/512²
output. A branch-cut fixture pins antimeridian AA/mip continuity within four
encoded channel values for a ±1e-6 rad heading change. Hardware HUD tests exercise
the actual egui callback at 1×/1.5× DPI,
resource recreation, exact preservation outside the overlay, disabled release
and passive command preservation. Captures are inspected integration evidence,
**not blind subjective acceptance**. No WASM/Metal runtime acceptance is claimed.
Final stacked gates passed **1,867 workspace lib/bin +90 integration tests**,
workspace/no-default clippy, **24 Bevy-CBT GPU** and **13 native ignored** checks,
plus the new-scope perf-capture, unified-scene and origin-cache hardware checks.

Those counts and timings describe the original 2026-10-18 instrument stage.
The 2026-10-19 shared-readout follow-up additionally tests two own epochs over
Thessa/Nereid with 0.001 m altitude / 0.001 m/s velocity and 0.02 m circular-apsis
bounds, no physics/reference mutation, replica provenance and checked canopy
withdrawal. Native regressions cover display-body/terrain/audio separation,
observed subsystem adoption, session-only selectors and unavailable formatting.
The real-adapter deck fixture exercises Surface/Air/Orbital/Target and Datum/AGL
at 800×600 and 1280×900 / 1× and 1.5× DPI, passive flight-command preservation,
outside-overlay pixels and disabled/help resource release. A release 64-craft
read-model diagnostic records local/replica CPU cost without stepping physics.
Follow-up commands/results: `artifacts/native-flight-readouts-2026-10-19.json`.
All 16 final local follow-up gates passed on 598 unchanged compiled inputs:
1,928 workspace lib/bin, 95 integration, 56 feature-off, 24 Bevy GPU, 31 renderer
and 31 native ignored tests, plus QA, build and muted window/headless smoke.
The release 64-craft read-model cost was 18.519 µs/craft local and 16.331 µs/craft
replica (single-run diagnostic, not a paired win or game FPS).

Reproduction, raw measures and actual check results:
[`artifacts/native-flight-deck-2026-10-18.json`](artifacts/native-flight-deck-2026-10-18.json).

## Adjacent review fix: transactional ray publication

The reported `RayScene::update` issue was real: clearing the live transform and
primitive arrays before all validation could corrupt subsequent `encode()` even
when callers recovered. It now stages into reusable candidate arrays, validates
camera/ranges/capacity/transforms, and swaps only on complete success. Rejections
preserve published/pending transforms, pairing, camera, counts and dirty state.
The extra CPU staging capacity is bounded by the scene's admitted instance count;
it is reused rather than allocated on every frame. A real-adapter regression
covers NaN, late capacity failure, out-of-bounds/reversed ranges and count/camera
errors, both with an already built scene and a valid update pending encoding.
Recovery reuses the same acceleration structures and unchanged instances rebuild
nothing. No GPU scene/resource reallocation is introduced by this fix.
