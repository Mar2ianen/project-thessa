# 66 — Bevy retirement: source inventory and acceptance ledger

**Status: active, incomplete. Updated: 2026-10-19.**

The requested outcome is removal of Bevy and reintegration or replacement of
code tied to its host, not merely an optional native terrain benchmark. Keep
the legacy client runnable until its retained functions have usable native
consumers. Bevy appearance is not an acceptance oracle. Dated measurements and
earlier partial-migration records in doc 39 retain their historical scope.

## Dependency boundary

Direct Cargo coupling is confined to `apps/client` and `crates/bevy-rcbt`, plus
the root workspace dependency. Foundation simulation, streaming, acoustics,
plume semantics, assets and native renderer crates do not require Bevy. This
small dependency count does **not** imply a small amount of remaining work.

## Source-to-consumer inventory

The table is a source inspection ledger, not a completed visual acceptance
matrix. Existing neutral implementations must be reused; no second simulation,
streamer, allocator, propagation resolver or generic RT scene is needed.

| Legacy source | Existing neutral/native owner | Remaining retirement obligation |
|---|---|---|
| `main.rs` lifecycle, scheduling, assets, window/input | `terrain-native/{main,window}.rs`, `render-assets`, winit/egui | promote a complete native client entry point; preserve packaged asset discovery and useful CLI scenarios |
| `terrain.rs`, `bevy-rcbt` jobs, caches, render extraction | `terrain-stream`, `rcbt-core`, `rcbt-wgpu`, `render-wgpu`, native `scene.rs` | retain conforming topology/shared surface epochs and independent height/material residency; remove the bridge only after its retained tests have neutral/native coverage |
| `pilot.rs`, `embedded.rs` flight/server input and cameras | `flight-authority`, `flight-net`, native `scene.rs`, `window.rs` | pause/warp/relaunch and focus release now have native consumers; reconcile remaining flight deck commands and reference/display-body choices; do not fabricate authoritative state in UI |
| `pilot/{hud,navball,fastnavball,...}.rs` | native `hud.rs`, `fastnavball.rs`, shared instrument settings | audit every retained readout/control; native flight deck exists but not all old widgets/commands have replacements |
| `navigation.rs`, `map_ui.rs`, `orbits.rs`, celestial visuals in `main.rs` | `sim-core` baked ephemerides, shared `flight-authority::navigation`/`ThreadBakeQueue`, native `map.rs`/`celestial.rs` | doc 67 adds six-scope map, body/PRIMARY framing, cached Kepler curves, a physics-JIT cache consumer with shared execution guards, dominant-body apsis/escape/wake readouts; powered/live JIT futures, instrument display-body choices and appearance/usability acceptance remain |
| `atmosphere.rs`, sky/cloud/aurora portions of `beauty.rs` | `atmosphere`, native `environment.rs`/`sky_layers.rs`, shared `render-wgpu` environment/clouds | doc 65 records implemented stars, cloud transport and depth-aware aurora; finite aerial perspective and richer transport/acceptance remain |
| `water.rs` and GPU terrain water shading | `atmosphere::ocean`, shared native spectral ocean/material/RT consumers | preserve one spectral surface across consumers; dynamic ocean RT reuse and wider transport/temporal acceptance remain |
| `plume.rs`, `assets/shaders/plume_volume.wesl` | `plume-core` source/profile/medium/optics | native resolved Low/Medium/High field consumer and diagnostics; keep provisional nozzle inputs explicit and never make mesh shape authoritative |
| `night_lights.rs`, gas-giant portions of `beauty.rs` | worldgen inhabited regions, material/assets layers | extract existing GPL optical bakes to game-owned code or independently implement a neutral model; native material consumers still missing |
| `audio.rs` | `audio-core`, `audio-synth`, native `audio.rs` | native engine/device adapter, exterior/cabin routing and RCS/GPWS/boom fixtures now have consumers; docking event integration remains; captures/tests stay silent and missing audio hardware does not prevent rendering |
| `contact_gizmos.rs`, `docking_demo.rs` | `collision`, shared docking state machine, native `contact_preview.rs` | preserve actual solved contact/docking inspection and deterministic replay; a static mesh preview is not a solved docking replacement |
| Bevy bloom/exposure/AA, Solari | native `DisplayTransform`, `RayScene`, FastRT | manual HDR/display and scene-object rays exist; remaining settings must be implemented or explicitly resolved to documented fallbacks, not falsely marked applied |
| `perf.rs`, Bevy BRP integration | `perf`, native capture schema, agent-QA tools | preserve measured CPU/GPU attribution and useful capture/control operations; never relabel CPU submit as GPU time |

## Working order and deletion gate

The 2026-10-19 combined-frame replay exposed regular triangular dark artifacts
in the opt-in conforming terrain consumer. Further isolation identified shadow
self-comparison, **not holes in geometry**: disabling shadows removes the grid.
The receiver-plane follow-up in doc 63 fixes the near-field pattern and preserves
real occlusion. Numeric sky gates did not catch this complete-frame defect.
The original rejected captures remain evidence; the follow-up is not independent
visual acceptance of the whole client or a certification of the distant horizon.

### Retained flight input audit

The old `pilot_input` also owns functions not covered by a native WASDQE test:

- **Implemented follow-up:** relaunch after a stopped flight (`Reset` command
  remotely, shared launch-site reset locally), preserving the simulation clock;
- **Implemented follow-up:** pause and warp requests, with acknowledged server
  pause/effective warp distinct from client intent, no second client clock in
  server mode, explicit bindings F8 / brackets / Backspace and CLI/HUD controls;
- **Implemented follow-up:** map/pilot switching (M / HUD / `--map`) and exclusive
  camera-versus-flight input capture; the existing authority continues with
  neutral manual axes and retained throttle/base SAS, not a second map clock;
- **Implemented follow-up:** Mouse Aim and held temporary SAS inversion share
  the shipped neutral input helpers; focus loss and suspension release held
  axes/gestures, cursor steering and the SAS override and send neutral input;
- **Implemented follow-up:** F1 controls help, F2 flight HUD visibility and F3
  telemetry/fleet panels. Help exclusively owns scene/manual input without
  pausing or cutting thrust; hidden instruments release their GPU callback.
  Map remains usable independently of flight-HUD visibility. Guidance-specific
  manual-takeover behavior still belongs to the remaining adapter below;
- opt-in coastal flyby guidance and server-side maneuver execution fixtures.

Source inspection distinguishes retained functionality from future UI: the
legacy map had orbit/apsis/wake readouts and asked the existing BakeQueue for a
conditional coast forecast even during powered flight. The latter is deliberately
removed under the user's actual-JIT-path requirement, not preserved as the main
map trajectory. Its `bound_prediction_arc`
helper is test-only, not another production forecast to port. The pilot's
`maneuver_kick_mps` sends a one-shot typed `ExecuteManeuver`; there is no shipped
interactive maneuver-node editor to preserve. Do not inflate that fixture into
an editor-completion claim or make the whole future editor a host-deletion gate.

The map follow-up now extracts the server's existing `ThreadBakeQueue` into
flight-authority (the server retains a compatibility import) and uses it for
native local flight. At most one worker runs; reset discards its result without
oversubscribing CPU work, and shutdown joins it. The existing `run_bake`/checked
adoption/coast-reuse pipeline remains the only bake path. Maps only read the
physics-produced execution cache: opening/reading/pausing a map never starts a
bake or advances physics. Shared execution eligibility, bounded interval and
geometry certification prevent drawing a hypothetical coast in powered/aero/
controlled/contact/stale states, or drawing past the current guard boundary.
Vacuum-coast physics owns its bake requests. Remote replicas do
not install/request a worker or synthesize an executable future. Native projection and the extracted
orbital summary follow the dominant-body display hint without changing the
solver reference or summed gravity. Next-wake countdowns are local-only;
transmitted wake notices remain visible remotely. The current actual-cache
consumer covers certified coast batches only. Generalized powered/live JIT
execution futures (doc 07 §7.10) and their remote publication are unfinished;
unavailable is explicit, never a disguised gravity-only prediction. See doc 67.

The remaining instrument audit is concrete: `pilot::live_flight_ui_state`
derives dominant-body-relative orbital/surface readouts, apsides, air data,
load/propulsion and parachute summaries; `craft_orbit_summary` adds scheduler
wakes/notices. Native `FlightData` currently supplies reference-body speed,
datum/AGL and navball inputs, not all of those fields. Missing remote force/air
observations must remain unavailable rather than be inferred from a lean
snapshot as if acknowledged. F1/F2/F3 visibility now has native consumers;
remaining mode/readout actions still need reconciliation. The visibility state
is session-owned UI policy, never authoritative state or an acknowledgement.

The visibility adapter keeps telemetry collection and server observation running
when F3 hides their panels; it is not a profiler/transport toggle. F2 suppresses
flight HUD meshes and releases the navball callback without changing graphics
requests, engine/throttle/base SAS, map ownership or scene rendering. F1 restores
visible help even from hidden UI. Help temporarily replaces the other HUD/map
widgets, so neither toolkit map gestures nor native camera/manual commands can
leak through it. Opening/closing help releases held demands; an ordinary F2/F3
visibility edit does not discard a held flight key. These flags persist during
normal resizing but are currently reset with a recreated window HUD, not saved
to graphics.toml. No OS-level key injection or independent usability acceptance
is claimed. Evidence is in `artifacts/native-hud-visibility-2026-10-19.json`.

Unmarked items remain replacement obligations, not features certified by the
existing native flight deck. Native arrow keys still control the camera;
pause/warp use separate bindings and cannot silently double-handle them.
Shared guidance, rails and command validation already exist; porting a UI must
not introduce a parallel controller, trajectory bake or server authority.

Concrete reuse points for that follow-up:

- `FlightAuthority::reset_to_launch_site` already clears the stopped state,
  resets controls/scheduler/bake and rebuilds the canonical launch pose without
  rewinding time. Native recovery should call it locally or send existing wire
  `Command::Reset` remotely, not reconstruct an authority from an initial seed.
- `FlightAuthority::advance_with_budget` already accepts simulation-time debt,
  advances fixed physics/rails, bounds CPU work and sheds unserved whole-tick
  debt. Host pause/warp intent must scale admitted elapsed time, not solver dt,
  and must report actual time advanced separately from requested warp.
- `flight-net::presentation::input_for_server` deliberately takes caller-owned
  ordered commands and separate throttle/engine intent. Native now sends the
  requested pause/warp votes through it instead of fixed unpaused x1 demands.
- `Snapshot::{paused,effective_warp}` are checked authoritative observations.
  Preserve them in the native observer; UI vote state is not acknowledgement.
- The old `SimulationClock` is a Bevy resource containing just presentation
  seconds/tick and multiplier/pause intent. It is not another physics clock
  worth reproducing alongside `FlightAuthority::flight_time_s`.

The native clock adapter is intent and observation only. `--flight-step` remains
a headless scripted admission interval; local window elapsed time is clamped to
0.1s before warp scaling and existing solver work limits. Paused/x0 frames do not
advance physics, sky, body spin or simulation-time audio events. Native capture
records local fixed steps/rails/debt and actual frame-wall effective warp;
server mode retains the reported server rate without inventing local work.
Checked authoritative physics-stop snapshots keep the live link available for
Reset; malformed packets, silence/deadlines and transport failure still close
it. The bounded event queue reserves the two time votes and refuses excess
Reset requests transactionally rather than expanding without a limit.
The real stdio-server regression now acknowledges pause, relaunch while paused,
resume and x4 demand; local tests cover invalid warp, stopped-flight recovery,
work limiting, input capture and focus release. No new force/control solver,
network payload, dependency, GPU pass or audio device is introduced. See
`docs/artifacts/native-flight-clock-2026-10-19.json` for reproducible checks.
All fourteen final local gates passed on 594 unchanged compiled source files,
including the real-server exchange, GPU CLI pause/warp counters and a muted
30-presentation native Wayland paused-flight/HUD smoke. This closes those
specific host adapters, not the remaining map, guidance or visual obligations.

The Mouse Aim continuation uses the existing pilot's 6% NDC deadband, normalized
forward direction and 0.85 pitch/yaw request, with keyboard takeover and a unit
vector clamp before precision. The previous native diagonal keyboard path did
not clamp the combined vector; it now matches the old pilot's demand policy.
Camera gestures, pointer-owned widgets, keyboard capture, missing/unfocused
cursor and pause/x0 cannot leak steering. Hold F inverts the retained SAS base
demand until release; the lean Snapshot does **not** acknowledge SAS. Neither
conditioner rotates authoritative state; local/server control laws still allocate
actual actuators. The old pilot consumes the same extracted math, not a duplicate
native law. See `artifacts/native-mouse-input-2026-10-19.json` for validation.
Its fourteen final commands passed on unchanged compiled sources, including
workspace/feature-off tests, hardware suites, real-server exchange, headless
clock counters and a muted Wayland flight/HUD smoke. OS-level pointer injection
and independent visual/usability acceptance were not performed.

### Plume adapter audit before native reuse

`plume-core::AxialProfile` already owns radius, emission, extinction and shock
stations; `sample_medium` and `integrate_ray` are the numerical references.
The legacy `assets/shaders/plume_volume.wesl` is **not** a drop-in semantic
implementation: it contains Bevy imports/material bindings, fitted mean
extinction rather than the complete station profile, a hard radial skirt cut,
tail dissolve, turbulence deformation and camera-inside fade. It also discards
near-axial rays when the cylinder quadratic is nearly parallel. These are
representation changes requiring an error envelope, not facts to inherit just
because they were visible in the old client. A native consumer should sample
the existing published axial profile, handle parallel/inside/depth-clipped rays,
and test its integration against the CPU oracle before adding residual detail.
The provisional rocket deck and X-15 nozzle offset remain game-owned inputs,
not calibrated compiled engine state. Field-derived lighting must enter the
shared lighting/RT consumer; a second generic light or ray-scene subsystem is
not a replacement strategy.

1. Land the verified shared sky/cloud/aurora publication and its evidence.
2. Reintegrate remaining small host adapters (audio and game-owned optical
   producers), then native field/render consumers and finite transport.
3. Replace map/navigation, remaining flight deck/event fixtures and runtime
   controls using their existing authoritative owners.
4. Promote the native application, move retained bridge-only tests to the owning
   subsystem, update packaging/CI/tools/docs, and delete the obsolete host and
   adapter. Do not claim completion by dropping behavior or disabling tests.

For each implementation stage record actual commands, baseline, scenario,
hardware/backend where relevant, numeric/reference bounds, counters and measured
costs. Run workspace, feature-off and hardware gates before pushing. Keep quality
fallbacks and disabled resource/work contracts explicit. Visual fixtures are
deterministic evidence, not independent aesthetic approval.

Final removal requires a Bevy-free **full native client dependency graph** and
workspace manifests/lockfile, plus usable native entry points for retained game
functions above. Search remaining source, shaders, CI, packaging and tools for
active Bevy-only contracts. Historical documentation may mention Bevy; active
instructions and status must identify the actual client. Platform/visual or
transport gaps cannot be concealed by a clean Cargo graph.
