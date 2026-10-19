# 67 — Native system map and shared navigation

**Status: implemented native map slice; full migration incomplete. 2026-10-19.**

Baseline: `05ceb1b`. The old Bevy map remains available while its remaining
maneuver, display-body and appearance obligations are replaced. This slice is
navigation presentation, not a second celestial or vehicle simulation.

## State and ownership

`flight-authority::navigation` owns the shared six-scope policy, body cycling,
adaptive Kepler sampling, bounded future-rails indexing/projection and an f64
orthographic map camera. Both hosts consume the extracted helpers. Canonical
ephemerides stay in SI/inertial axes; only the legacy host converts its output
to scaled Bevy coordinates. The old adaptive sampler could overrun its vertex
budget across base intervals; the shared sampler budgets *additional* splits
globally and always retains the complete closed curve.

Native `map::Source` lazily caches the 32-body catalogue and parent-relative
curves on first open. A closed map performs no map ephemeris sampling, toolkit
work or GPU draw; reopening retains the small immutable CPU catalogue (not
another terrain/material cache). Open-frame snapshots evaluate the existing
baked ephemeris at the newest authority's SimTime when flying and at scene
presentation SimTime when surveying. Camera subtraction and line clipping
happen in f64 before conversion to toolkit pixel coordinates. Body radius is
physical; the three-pixel outline minimum is an explicit selection marker,
not enlarged simulation geometry. Labels prioritize selected/hovered bodies
and skip collisions; the selector always lists every physical in-scope body.

PRIMARY is the newest authoritative state with its labelled SimTime; all map
celestial bodies use that **same** epoch. The underlying interpolated 3D scene
may lag, but mixing its older planets with a newer inertial craft would invent
apparent altitude from their common orbital motion. Before the first valid
remote snapshot no craft marker exists. Local available rails use `FlightAuthority::rails.path()`
and project every future sample relative to the reference body's ephemeris at
that sample's own time. This is a reference-body-relative future curve anchored
at the reference body's current position, not a barycentric future track or a
two-body prediction. Impact circles are forecast display facts, not solved
contacts. The polyline is bounded to 1,025 points including current craft.
No renderer-triggered bake or second integrator is added. The lean wire carries
no rails path: native remote mode explicitly reports no forecast rather than
silently integrating a different one locally.

## Controls and rendering

- `M` toggles map/scene (pilot when flying, otherwise survey/preview) while HUD
  is enabled; `--map` starts open. Headless
  capture needs both `--map --hud`; feature-off or disabled HUD is rejected
  before world loading. The HUD also has an open-map button and window close.
- Six scope buttons select system/Asterion A/Nereid/Orthea/Vesper/B-C views.
  The selector, Previous/Next or marker click select a body; Frame body and
  Follow body track it. Fit scope includes the scope extent and viewport aspect.
  Frame PRIMARY follows its observed position; panning releases follow.
- Right drag orbits, middle drag pans and wheel zooms over the map canvas.
  These gestures never edit the pilot/survey cameras or steer the craft.
- Manual stick, cursor steering, throttle slew and held SAS inversion are
  released/suppressed in map mode. Engine/throttle/base SAS intent remains.
  Clock/relaunch keys remain available when no focused widget captures them;
  opening the map neither pauses nor relaunches the authority. M and HUD
  transitions clear held host demands, preventing stale-key replay on return.
- The existing egui renderer loads the existing display-referred color target
  after the display transform. No second device, renderer, global render mode,
  audio device or simulation clock is introduced. The terrain/flight scene
  keeps updating behind the map; this is not a map-only performance mode.
  The flight navball callback resource is released while map UI owns the HUD.
- Performance capture records `native_map_open`; map UI costs stay in the
  existing host/UI scopes and must not be compared as a hidden no-HUD workload.
  Headless egui animation time is scripted at 60 presentation frames/s, including
  warmup, so machine speed cannot leave a captured window half faded. This is
  toolkit animation only, not a change to physics/sky/audio SimTime or flight dt.

## Verification boundary

Deterministic tests pin physical-body scope counts (30/17/9/3/3/13), metre-scale
projection at a 1e13 m origin, unit camera gestures and f64 line clipping,
cached path reuse and zero closed-map samples. All baked Kepler curves are
checked against 2,048 dense exact ephemeris samples per orbit: maximum sampled
point-to-segment error is below the **absolute** envelope `0.001 * a` metres,
and curves close within `1e-12 * a`, with at most 513 vertices. These are tested
catalogue bounds, not a universal pixel-error guarantee for arbitrary orbits
or deep zoom. The independent moving-reference rails fixture verifies own-time
subtraction within 1 mm, future-only decimation, last-sample/impact retention,
clipping and expired/malformed rejection.

Toolkit tests exercise every scope at system and one-metre zoom, finite bounded
mesh vertices and actual selector/PRIMARY-framing clicks. Host tests cover map
input ownership, unchanged flight cameras, continued local stepping and remote
own-time/no-local-prediction behavior. Shared-device GPU tests cover target-load
preservation, DPI, resize and recreation with the map open. A real headless CLI
fixture verifies map canvas pixels, silent capture, unchanged fixed dt and
continued authoritative time advancement (one encoded LSB tolerance for egui
display dithering). The target-size ignored benchmark
records catalogue cost/bytes, open snapshot and toolkit/tessellation costs and
closed-map dispatch, not game FPS or an old-client performance win.

All fourteen final local gates passed on 596 unchanged source inputs, including
1,915 workspace lib/bin tests, 95 integration tests, 24 Bevy GPU tests, 31 native
and 31 render-wgpu ignored hardware/diagnostic tests. Actual commands, hardware,
cost measurements and the retained failed/during-edit diagnostics are recorded
in `artifacts/native-system-map-2026-10-19.json`. No independent
visual/usability review, OS-level key/pointer injection, Metal/WebGPU execution,
remote CI or complete Bevy retirement is implied.

## Remaining obligations

The old map's renderer-triggered conditional gravity-only forecast request
during powered/aerodynamic flight is not ported: native draws only a currently
available shared bake. Reintegrate that request through the existing BakeQueue
host lifecycle, not another prediction system. Retained orbit/apsis/wake readouts,
instrument reference/display-body choices and richer celestial/ring appearance
also remain. The legacy pilot's one-shot server `ExecuteManeuver` fixture and
coastal flyby intent adapter still need native entry points; an interactive
maneuver editor is broader future work, not an implemented old-map feature.
Remote forecast transport, guidance panels, secondary-fleet map markers and
generalized trajectory error/pixel adaptation remain open. Tiny/mobile layouts and dense-system
navigation require usability acceptance. Do not remove the old map or its
remaining tests merely because the native host can now display its ephemerides.
