# Project Thessa

> Factory, logistics, and aerospace engineering in one physical world.

[![CI](https://github.com/Mar2ianen/project-thessa/actions/workflows/ci.yml/badge.svg)](https://github.com/Mar2ianen/project-thessa/actions/workflows/ci.yml)
![Rust](https://img.shields.io/badge/Rust-1.95%2B-000000?logo=rust&logoColor=white)
![Engine license](https://img.shields.io/badge/engine-MIT-blue)
![Game license](https://img.shields.io/badge/game-GPL--3.0--or--later-blue)

Project Thessa is a cross-platform Rust prototype for a factory and aerospace
sandbox. The intended game loop connects production, surface logistics, vehicle
design, atmospheric flight, orbital mechanics, and interplanetary logistics.

The repository is currently a **pre-alpha engineering prototype**. Internal APIs,
data formats, and design values may change before `0.1.0`; no compatibility
promise is made for them.

## Current implementation

The checked-in vertical slice currently contains:

- `thessa-sim-core`: compatibility facade for deterministic baked ephemerides,
  full multi-body test-particle gravity, atmosphere and panel aerodynamics,
  rigid-body flight, on-rails coast caches, cohort gravity patches, piecewise
  analytic affine propagation, cabin pressure and crew authority, part
  assembly connectivity/topology splitting and rigid-body cluster mass/COM
  reconstruction from complete per-body inputs, with non-tree crossfeed and
  pressure-limited feed lines, installed reaction-wheel banks, deployable
  parachutes, fixed-step tank-limited propulsion and moving-mass updates,
  mounted RCS, tank-backed pneumatic/rocket-bootstrap starters, torque-rated
  starter/generator hardware, generator efficiency/thermal limits, and
  independent LP/HP geared-turbofan dynamics plus ordinary jet/APU generation
  on a parameterized wire-free vehicle power bus with batteries,
  ultracapacitors, fission reactors and fuel cells, sun-tracking cell-grid
  solar arrays with overlap-aware geometric occlusion, prioritized consumers,
  and a lumped thermal-node network with conduction links and area radiators. The
  implementation is separated into `thessa-aero-core`,
  `thessa-celestial`, `thessa-trajectory`, `thessa-propulsion`, and
  `thessa-vehicle-core`; `thessa-aero-core` also provides an experimental
  packed residual table path with coefficient-to-force error bounds, while
  canonical f64 tables and analytic aerodynamics remain available;
- `thessa-aero-surfaces`, `thessa-fuselage`, and `thessa-vehicle-baker`:
  procedural lifting-surface/body compilers, interior-volume and mass
  properties, cabin layouts and presets, static exits, controls, and assembled
  vehicle asset compilation including electrical power, thermal, shield-aero,
  and wing tile-layer hardware;
- `thessa-plume-core`: backend-neutral engine-exhaust profiles, optics, and
  participating-medium evaluation, consumed by both plume renderers;
- `thessa-flight-control` and `thessa-flight-authority`: typed guidance,
  aircraft/spacecraft/direct control laws, policy limits, physical allocation,
  actuator dynamics, the authoritative flight stepper, and contact-scene D1
  docking/separation primitives with solver joint-load telemetry;
- `thessa-autopilot` and `thessa-autopilot-js`: validated typed graph IR,
  sequence/parallel/wait execution, simulation-time scheduling, sandboxed
  QuickJS blocks, and typed trajectory-plan execution;
- `thessa-maneuver`: two-body planning helpers (circularization, Hohmann,
  Lambert, plane change, velocity matching) plus candidate search. Planning
  approximations are revalidated through the exact field before execution;
- `thessa-flight-net` and `thessa-protocol`: versioned framed input/snapshot
  transport (wire v9) with strict validation, ordered typed part commands,
  per-vehicle input/guidance targeting, and fleet Separate/Dock/Undock
  commands plus fleet snapshots;
- `apps/server`: headless authoritative simulation over stdio or TCP, with a
  server-owned vehicle fleet (separation spawning, dock sessions with fixed
  joints, live resource-state preservation across separation, fleet snapshots);
- `apps/client`: Bevy map, pilot HUD, atmospheric and field-first plume
  rendering, terrain streaming, a temporary audio adapter, and an embedded
  authoritative-server path;
- `thessa-audio-core` and `thessa-audio-synth`: backend-neutral propagation
  semantics and procedural engine DSP, consumed by both the legacy Bevy and
  native rodio adapters; headless captures/tests do not open audio devices;
- `thessa-worldgen-rocky`: deterministic rocky-world fields, geology, climate,
  landmarks, LOD, obstacle reports, and client texture export. Recipe conformance
  is partial; authored ocean bounds now reach canonical datum calibration.
  World-addressed rock/plant descriptors preserve identities across query LOD
  and density, with the existing offline plant bake consumer
  ([doc 70](docs/70_WORLD_SCATTER_IDENTITY.md)).
  A plant-only callback now borrows the canonical attached field for 32 m ground
  and placement slope, material-consistent ecological cover and coarse lake/ocean
  rejection. Disabled-by-default `[surface_plants]` enables a bounded native
  survey-only prototype consumer through the existing opaque mesh/shadow path
  ([doc 75](docs/75_NATIVE_SURFACE_PLANTS.md)); the prototypes now use grounded
  folded pinnate fronds, CC0 bark/leaf material maps, geometry-baked ambient
  visibility and thin diffuse leaf scattering. WORLD10 adds curved indexed pinnae,
  a lower asymmetric fork and roughness-filtered mesh environment response. Immutable plants can
  participate in opt-in TAA; moving meshes and water remain masked. Geometry
  and GPU cost increase; reference matching and visual acceptance remain open. An optional default-off near-eye
  understory adds canonical fern/reed clumps on a separate fixed L24 identity
  lattice inside 49 L18 windows, with world-addressed eight-metre colony
  occupancy and canonical ecological eligibility, reusing the same quality
  tiers, prototypes,
  opaque mesh and shadow paths. WORLD11 extends large plants to nine neighboring
  L14 windows and selects scale-aware reduced pinnae beyond the full-detail
  range, plus separate dry outer ground-fern fronds without more triangles;
  plant identities and ecological eligibility remain unchanged. WORLD11a narrows
  ground pinnae to a roughly 6:1 lamina aspect with gentler camber, using the same
  triangle budget. Same-site near-window changes use one shared
  worker with stale-result rejection; this is not a vegetation streamer. Shared
  stellar shadow rays now preserve their physical line instead of offsetting
  sideways into narrow leaves; finite receiver-bias and subpixel-coverage gaps
  remain. Production vegetation streaming, fully modeled botanical assets and
  physical obstacle ownership remain unimplemented.
  An opt-in [surface review profile](data/graphics/surface-showcase.toml) enables
  the experimental location, PBR Neutral display and 25 m near-shadow cover (TAA off);
  the review profile now selects 4× MSAA at substantial GPU cost;
  [doc 77](docs/77_NATIVE_SURFACE_SHOWCASE.md) gives the fixed-site launch command
  and remaining quality/performance limits. WORLD12 reviews the latest soil
  experiment: continuous mineral/humus processing before mip averaging, shared
  moisture/pigment roughness response, one explicit 60 mm optical height range,
  and view-cone-aware material admission within the existing cache budgets.
  WORLD12 used authored optical priors; later render-only relief does not alter hydrology.
  WORLD13 promotes persistent GPU CBT in both native launch configurations,
  separates sparse height-page residency from geometric split/merge, fixes
  elevation-driven exhaustion with source/chord probes and parent merge validation,
  and uses existing GPU active indices/indirect draws for primary terrain and shadows;
  preserves exact fine material coordinates on bisectors, and adds stochastic soil mapping,
  render-only height relief and bounded near-eye pebble meshes. Low keeps optical
  soil; Medium/High refine the same source and thin one world-addressed population.
  WORLD14 retains the reviewed soil/foliage material priors and optional 24 m plant
  detail override; an opt-in spherical local-haze layer now shares one environment
  publication across primary terrain, water and meshes, with explicit clear and
  cheaper isotropic fallbacks. Full physical aerial transport remains open.
  WORLD15 adds a second CC0 mud/litter scan, promotes the existing sub-8 m grain
  into resolved render-height pages, preserves unresolved normals without double
  bumping, introduces grounded single/forked tree forms and larger pebble variation,
  and adds SH9 directional diffuse lighting plus specular footprint filtering.
  Low/disabled policies retain their cheaper representations; TAA remains off.
  WORLD22 reviews Gemini's fuller crowns and coastal profile, restores soil
  normal detail across material/geometry levels using canonical base gradients,
  retains millimetre scan detail with bounded High L24 material pages, restores
  spatial moss patches and scan contrast, bounds understory cohorts
  to requested sizes, and makes bark import reproducible.
  WORLD22 retained 48 m plant detail, EV100 12.50, 4× MSAA and TAA off.
  WORLD23 adds wet-soil variation, source-height moss cushions and bounded moss
  shoots, plus default-off sampled-basin puddles through the shared mesh/shadow
  path. The showcase opts in; these are static optical scenery, not new hydrology.
  WORLD40 audits Gemini's later crowns, clearings and coastal weather (EV100
  12.10, 4× MSAA, TAA off, local RT), connects both mesh/terrain representation
  decisions to FastRT, preserves filtered secondary-hit texture mips and fixes
  floating forks, horizon-dependent AO, distance-dependent shadow leaks and
  premature shared-TLAS exhaustion. Ground bounce and stochastic GI remain open.
  WORLD41 removes redundant soil-scan prefilter blur, retains scan contrast
  through litter pigment and adds bounded folded moss/ground-leaf geometry in
  the shared mesh/shadow path, within the existing 7 MB prototype limit.
  WORLD42 restores native 1K soil scan data for near-camera detail, filters each
  source at its actual resolution and extends regular/conforming material-motion
  oracles through L24; GPU page budgets remain, CPU source mips add 20 MiB.
  WORLD43 fixes close pages below the frame outranking visible soil: native/Bevy
  material priority uses the actual camera projection while the shared refinement
  cover and soil-height prefetch remain, under the same page/cache budgets.
  WORLD84 retains the latest user crown/rosette forms, adds organ-level
  green/juvenile/olive/weathered pigment cohorts and asymmetric pinnae through
  shared padded leaf textures, without extra draws or a changed mesh budget.
  Matched captures use raster shadows after an original-baseline AMD RT device loss.
  Coastal continuation: reviewed incoming soil/understory edits, scanned moss and connected shoots, and compound ground/crown fronds with attached secondary pinnules and preserved geometric error guards. WORLD139 single-blade contours were rejected; WORLD142 passed CPU checks but its first raster capture reset the GPU and desktop session; physical GPU replay remains blocked. Isolated software-Vulkan specimens now inspect actual native meshes/materials/shadows: ground pinnules are broader, dry organs subdued, and reduced crowns retain divided edges. The scene profile uses 24 cloud steps. Reference and temporal acceptance remain open.
  Reference acceptance, exact contact, collision and planetary prop streaming remain open.
  The remaining planet refactor is tracked in doc 13; a reproducible canonical
  coastal 2×2 km CPU audit site and its limitations are recorded in
  [doc 72](docs/72_WORLD_SURFACE_BASELINE.md), not visual acceptance;
- `thessa-rcbt-core`, `thessa-rcbt-ffi`, `thessa-rcbt-large-ffi`,
  `thessa-bevy-rcbt`, and `thessa-rcbt-wgpu`:
  backend-neutral adaptive terrain topology, a fast pure Rust implementation,
  optional verified upstream `libcbt` and `large_cbt` OCBT implementations,
  universal Bevy frame scheduling, compact height pages, and a portable GPU
  adapter. An experimental microscaled material-page codec and LOD/mip path is
  available for render-world A/B; it currently pre-decodes to RGBA and raw
  storage remains the default. The configured client uses GPU-auto terrain
  (optional mesh shaders with indexed GPU fallback); CPU terrain remains
  selectable. Both GPU draws share LOD, skirts, scene lighting and
  world-addressed material sampling independent of geometry LOD. Docs 47–53
  track historical failures and current work; visual acceptance is incomplete.
  Native launch configurations now select the persistent conforming GPU bisector
  pool and shared raster/shadow/ocean/RT consumers (docs 63/77); regular patches
  remain the explicit/unsupported fallback, with legacy Bevy integration retained;
- `thessa-render-wgpu`: owned headless/surface-compatible backend and CBT compute → indirect raster
  passes, camera-relative uniforms, dirty quantized-page updates and bounded
  material-page residency and bounded reusable geometry slots without
  Bevy. `cargo run --release -p thessa-render-wgpu --example terrain_frame -- terrain.png`
  produces a synthetic terrain screenshot and offscreen latency benchmark;
  it does **not** replace the production client's complete scene/UI/input yet
   (doc 39, implementation checkpoint);
  Opt-in `[temporal].motion_vectors` diagnostics now use stored opaque scene
  depth with explicit ocean/mesh invalid masks and submitted-camera tracking
  ([doc 76](docs/76_TEMPORAL_RECONSTRUCTION.md)). Separate `[temporal].jitter`
  diagnostics sample primary terrain/meshes while preserving secondary views
  and jitter-free velocity. Default-off `[temporal].taa` now connects the shared
  resolve to native scene history before overlays, with explicit quality and
  HDR/capability fallback. All rigid meshes (including static plants) and ocean
  reject history and cannot seed filtered history; composition, a measured
  ghosting envelope and visual acceptance remain open. The
  [T4c evidence](docs/artifacts/native-temporal-history-2026-10-23.json) records
  13 local gates, nine captures and three prepared, unreviewed blind pairs;
  orbit is excluded from preference because material residency differs.
  No upscaling or frame generation is implemented.
- `apps/terrain-native`: Bevy-free window/headless migration viewer using the
  same frozen canonical field, HIGHLANDS bookmark, height/material builders and
  stellar ephemerides as the game. Run `cargo run --release -p thessa-terrain-native`
   (egui HUD, Left/Right orbit, Up/Down zoom, Esc exit when UI does not capture input), or add
   `-- --headless terrain.png --frames 120` for a screenshot/frame benchmark.
  Numeric surface targets (`--survey-lat`/`--survey-lon`) and exact radial eye
  clearance (`--survey-eye-agl 50` or `2`) support reproducible close-ups;
  [doc 71](docs/71_NATIVE_SURFACE_ACCEPTANCE.md) describes their source,
  framing and acceptance limits.
  Live async height/material streaming shares the extracted client selection,
  dependency/priority policies, bounded Rayon executor and height/material
  job/cache lifecycles in GPL `thessa-terrain-stream`. Source snapshots/revisions
  live in MIT `rcbt-core`; the old client publishes their shared directories
  through thin Bevy resources. Height admission counts pending results against
  cache capacity and returns retired payloads to the host's asset bridge.
  Height eviction does not remove material pages.
  Native compact-material requests explicitly resolve to raw RGBA residency
  with a diagnostic; the codec and retained shader/GPU regressions now run
  without Bevy, but native compact residency is not claimed (doc 41).
  Backend-neutral f64 body/camera frames now live in MIT `thessa-render-core`
  and are consumed by both clients. The native viewer defaults to the frozen
  epoch; `--sim-time 72000` samples a different celestial pose, and headless
  `--sim-step 120` samples motion per rendered frame (including warm-up), not
  authoritative simulation warp. See doc 39 for the spin-model boundary.
  `--vehicle-preview` displays the shared X-15 GLB at the initial flight-state
  pose through the owned pilot camera (quaternion orbit, 8–180 m zoom, pan,
  optional chase framing). The old pilot now converts the same neutral
  mesh/material source instead of loading this model through Bevy `WorldAsset`.
   Native terrain and meshes share scene color/depth; HUD loads the final SDR
   target after the optional HDR display transform;
   its configurable opaque path now supports shared GGX stellar/environment
   reflection, normal/metallic-roughness/AO maps and a memory-bounded three-star
    cascade shadow atlas shared with terrain. Scene meshes can use negotiated
    ray-query BLAS/TLAS reflections; `[atmosphere]` enables a quality-tiered
    shared single-scattering sky field for mesh reflections/RT misses and
    derived diffuse ambient (see doc 39's environment-radiance checkpoint).
    Regular/conforming terrain also integrate that sky through rough GGX, with
    resolved 4/8/16 clear-sky directions, cheaper reflected-cloud columns and
    compile-out specular disable
    ([doc 69](docs/69_NATIVE_TERRAIN_MATERIAL_LIGHTING.md)). Optional primary-raster
     material normals share page residency, with resolved disable and mip-filter
     quality tiers. Bounded residency now protects coarse coverage then admits
     nearby refinement paths instead of starving all fine pages behind distant
     coarse demand ([surface-first work](docs/77_NATIVE_SURFACE_SHOWCASE.md));
     natural substrate detail, vegetation and final close-up
     terrain acceptance remain incomplete.
      The bounded survey plants distinguish stems and foliage through the shared
      mesh material; pinnate fern geometry and forked crowns use smooth stem
      normals without extra draws (docs 75/77). Visual acceptance remains open.
     Regular-patch material sampling preserves local chart precision; the tested
      forest-floor colour-noise prototype was rejected, not shipped (doc 77).
      A distinct default-off CC0 soil/gravel experiment can refine regular-patch
       materials to L20/L21/L24 with capability-resolved 1/4/16x anisotropy by
       quality and a terrain-aware local distance reference; cross-LOD appearance and visual
      acceptance remain open (doc 77).
    RT hits now interpolate UVs/smooth normals and reuse raster base/MR/normal/AO
    textures; bounded stellar shadow rays supplement the shared atlas on meshes
     and reflected hits. The background now samples that same sky field at far
     depth in the terrain pass, with no second integration, upload or render
      target. Mesh queries can also trace the resident generated terrain grid
      through persistent dirty-only BLASes, sharing raster material pages.
      Over-budget partitions fall back whole, never silently drop blockers.
       Opt-in `[ocean]` now adds a shared multiband spectral field, filtered
       water optics and bounded near-grid displacement shared by raster/shadows/RT
       ([doc 56](docs/56_NATIVE_SPECTRAL_OCEAN.md)); overflow is optical-only.
         Source discs/stars now have native consumers (doc 65); full physical aerial perspective,
        auto exposure/bloom and primary terrain/ocean mesh-query
        reflection consumers remain missing: **not** complete PBR/Bevy parity.
       Seasonal regressions use actual Thessa/Nereid/BC ephemerides: distant
       visible B, and A hidden by Thessa while B/C still light native terrain.
      `[engine_plume]` now selects native Low impostor / Medium 8-step / High
      14-step actual-profile volume rendering with opaque-depth clipping and
      resource-free disable. The starter nozzle deck remains provisional;
      native nozzle lighting is explicitly unavailable ([doc 38](docs/38_ENGINE_PLUME_RENDERING.md)).
      `--docking-demo` now runs the retained two-body Rapier/DockingSession
      experiment with solved poses, D1 CAD rest surfaces, native inspection and
      cabin-only capture cue. `--docking-time` / `--docking-step` provide silent
      headless replay; this isolated fixture is not orbital/fleet authority
      ([docking contract](docs/details/01_DOCKING_PORTS.md#12-native-retained-fixture-port-2026-10-20)).
      `[night_lights]` now binds the retained inhabited-land optical mask to
      native regular/conforming terrain, with cached Low/Medium/High startup
      bakes and no mask work when disabled or gain=0. Primary-only authored
      emission, not simulated electrical activity or secondary-ray lighting
      ([doc 68](docs/68_NATIVE_NIGHT_LIGHTS.md)).
     The shared shadow atlas reuses depth only for unchanged light matrices,
     terrain and mesh casters; motion/source changes invalidate it, with no
      quality or update-rate reduction. Optional
      `shadows.first_cascade_distance_m` redistributes near precision without
      increasing atlas memory or shortening far cover; omission keeps legacy
      spacing. Mesh and terrain shadow receivers use the triangle plane rather
      than smooth/material shading normals
      ([doc 69](docs/69_NATIVE_TERRAIN_MATERIAL_LIGHTING.md)).
     Headless `--frame-sequence NEW_DIR` saves every measured frame as PNG,
     excluding warmup, alongside the final `--headless` screenshot. It requires
     a new directory, preserving prior captures. Readback/writes stay outside
     frame timing samples but perturb streaming wall cadence; use it for visual
     motion review, not a performance-equivalent replay.
     Headless `--gpu-trace NEW.toml` adds crash-durable stage intent/completion
     records and explicit mesh-work admission, separate from performance capture
     ([doc 15](docs/15_PERFORMANCE_MONITORING.md)). Mesh shadows conservatively
     reject instances outside each light volume; Radeon stability remains open.
     Headless `--perf-capture target/native-perf.json` exports the existing
     `thessa-perf` JSON/CSV format, including hardware GPU command intervals
     where supported (explicitly unavailable otherwise). Normal runs create no
     profiler queries/readback buffers. Windowed `F4` now opens the shared
     collector as a passive overlay, `Shift+F4` writes a JSON+CSV capture and
     `F5` cycles the profiling level; scopes enable only while the overlay is
     visible or a capture runs, and the windowed overlay never presents CPU
     submit as GPU time (GPU intervals stay the headless path). See docs 15/39
     for measurement boundaries.
     Live flight-input
  intents (stick/throttle/engine/precision/control-law/SAS) are owned in
   local `--flight` and `--server-flight`; remaining production game UI is missing.
  Preview requires epoch time and no
  time stepping. Reusable source descriptors/decoding live in MIT `render-core`
  / `render-assets`; game asset calibration remains GPL. X-15 by cmoreau is
  CC BY 4.0 (see its asset license).
  `--flight` instead starts the existing local `FlightAuthority` at the canonical
  COAST launch and tracks its live pose with the owned pilot camera (Navball
  attitude hold by default, no network/game-UI parity). The compatibility
  starter arms full legacy throttle with the engine on, exactly like the old
  client inherits. Owned GPL `flight-authority::input` shares stick mapping,
  throttle slew/presets, engine toggle and precision factors with the old
  pilot: `--throttle 0.5` presets the intent deterministically, `--engine-on`
  arms the engine, window `WASDQE` flies the stick, `Shift`/`Ctrl` slew the
  throttle, `X`/`Z` cut/fill, `Space` toggles the engine, `C` cycles the
  control law, `T` toggles SAS and `CapsLock` scales precision; the HUD mirrors
  the same panel plus installed-system toggles (`R` RCS, `Y` wheels, `G` gear,
   `P` chutes) applied as the explicit part commands the old pilot sends.
   Live flight also accepts `--paused` and `--warp 0..131072`; window F8 pauses,
   `[`/`]` halve/double warp and Backspace relaunches at the canonical site
   without rewinding simulation time. HUD requests and server acknowledgement
   are separate; a disconnected server is never replaced by local physics.
   Losing window focus releases held axes/gestures without cutting the engine.
   Mouse Aim now consumes the cursor through the same 6% deadband as the old
   pilot; keyboard stick takes precedence and diagonal demand is unit-limited
   before precision scaling. Orbit/pan and UI gestures never also steer.
    Hold `F` to invert the retained SAS demand temporarily; release restores it.
    With HUD enabled, `M` switches between the scene and native system map (`--map` starts
    there; headless captures also require `--hud`). Six scopes use baked
    ephemerides, f64 framing and cached Kepler geometry; select/frame/follow a
    body or frame PRIMARY, right-drag orbit, middle-drag pan and wheel zoom.
    Flight continues with neutral manual axes and retained throttle/SAS.
    The map reads the physics JIT's existing execution cache through the same
    eligibility/interval/geometry guards as the solver, bounded to the current
    executable coast interval. It never requests a hypothetical zero-thrust/
    zero-aero forecast. Powered/live JIT trajectories are not implemented yet
    and are shown as unavailable, not replaced by a coast curve. The map also
    includes dominant-body osculating Ap/Pe/escape and local wake countdowns;
    remote snapshots carry no JIT curve or
     next-wake queue, only observed wake notices. Maneuver interaction remains unported.
    See [doc 67](docs/67_NATIVE_SYSTEM_MAP.md).
    `F1` opens controls help, `F2` hides/shows the flight HUD (not the map),
    and `F3` toggles telemetry/fleet panels. Help suppresses manual/camera input
    without pausing or cutting thrust; F1 restores visible help from a hidden HUD.
    Hiding instruments releases their navball callback resource, not the scene.
    `F7` (or `--flyby`, or the HUD checkbox) engages opt-in coastal flyby
    guidance on the live craft: a direct port of the retained pilot's typed
    fixture that captures the launch datum/shore heading and writes only normal
    pilot demands (stick/throttle/SAS/engine/chase, Navball control law) —
    never pose, forces or authoritative state. It requires fresh unpaused x1
    `--flight`/`--server-flight` outside the map; any manual stick/throttle key,
    map/help/focus loss, pause/warp/relaunch, control-mode/SAS/engine change,
    stale flight or link loss cancels it without auto-resume. In server mode it
    stays an intent over acknowledged pose/kinematics: no local physics step and
    no fabricated air/force observations. It is not obstacle avoidance or a
    general autopilot; see [doc 57](docs/57_NATIVE_FLIGHT_DECK.md).
    `--maneuver-kick MPS` (requires `--server-flight`, 1<dv<=2000 m/s) queues
    one typed `ExecuteManeuver` prograde node at authority time + 20 s over the
    normal wire after the first acknowledged moving snapshot, at most once per
    session: the server executes the plan, local mode has no executor and is
    rejected, and a rejected plan surfaces in the server wake notice.
   Both flight decks now consume shared dominant-body SI/f64 instrument data.
   Native session-only selectors choose Surface/Air/Orbital/Target speed cues and
   Datum/AGL altitude without changing attitude reference or flight controls.
   Air/loads, instantaneous Ap/Pe, wheel torque and canopy details have native
   readouts; missing remote air/force observations remain unavailable. AGL uses
   only matching home terrain; away from home, Surface explicitly falls back to
   body-relative velocity without applying the launch world's rotation.
   Native `[instruments]` selects aircraft or
   FDAI-inspired spacecraft decks with a native-pixel GPU navball, full surface
   heading/pitch/roll and live vector cues; no 128 px/30 Hz CPU image rebuild.
   An optional static-grid bake was measured against the default analytic path
   ([doc 57](docs/57_NATIVE_FLIGHT_DECK.md)). Legacy Bevy retains CPU128.
   High-level assist/autopilot panels are not ported yet. Native terrain,
   opaque meshes and sky share one pass, with AS builds timed separately;
   [doc 58](docs/58_NATIVE_PIPELINE_REVIEW.md) records the optimization review,
    fixed-resident comparisons and limits of the supplied performance claims.
    Native non-ocean terrain also rejects conservative out-of-atlas casters;
    [doc 60](docs/60_NATIVE_SHADOW_CASTER_REVIEW.md) records full-caster depth/image
    oracles and the isolated performance boundary. Ocean retains full coverage.
    A separate [CBT sphere experiment](docs/61_CBT_SPHERE_EXPERIMENT.md) tests
    persistent conforming GPU bisectors without patch grids/skirts.
    Its [parallel transaction follow-up](docs/62_CBT_PARALLEL_TRANSACTIONS.md)
    adds conflict-filtered GPU split/merge waves and a measured 128K-slot fixture;
    Bevy renderer parity is not a migration gate.
    The [canonical surface consumer](docs/63_CBT_CANONICAL_SURFACE.md) then
    emits planet-scale bisector vertices through heap depth 58 using the existing
    height/material residency (no second streamer), with shared-vertex and
    depth-precision oracles plus a shared-material indirect draw. Current native
     refinement uses actual projected area, shared-source midpoint deviation and
     candidate-parent checks, rather than absolute elevation as relief. Native `[terrain_topology] enabled=true` now selects
     it for the real window/headless frame, with Low/Medium/High budgets and a
     resource-free disabled/regular-patch fallback. The shared shadow atlas uses
     those same vertices and their emitted revision; the ocean consumer borrows
     the spectral field and resolved ocean policy, with
    shared-vertex filtering, non-accumulating emission and optical-only fallback.
    Shared publication epochs now feed those vertices into the existing native
    RayScene with canonical material/water hits and whole-pool budget fallback;
    optical-only phase changes do not require AS builds.
     Its LOD classifier projects emitted camera-relative positions, not absolute
     planetary f32 coordinates, and reuses bounded deterministic CBT transactions.
     Per-tile error/convergence, temporal/equal-error acceptance and wider native
     client/effect migration remain incomplete; this is not full Bevy removal.
     The [rejected optimization audit](docs/64_NATIVE_OPTIMIZATION_REVIEW.md)
     now has corrective GPU quiescence/origin/cache regressions (doc 63), not
     accepted speedup claims. [Sky migration audit](docs/65_NATIVE_SKY_AUDIT.md)
      inventories existing sky, star, cloud and aurora producers before porting.
      Its native follow-up now draws flux-normalized distant stars/source discs
      and depth-clipped clouds. Shared cloud weather/transport also feeds direct
      terrain/mesh/ocean visibility and reflected rays; auroral emission feeds the
      common sky environment and a depth-clipped primary volume with ordered
      cloud extinction, including orbital foreground terrain. Atmosphere/cloud
      multiple scattering and wider visual/platform acceptance remain open.
      Primary terrain uses a [datum-aware cloud receiver](docs/74_COASTAL_CLOUD_RECEIVERS.md)
      to avoid false planetary self-shadowing on chord-meshed water; generic
      volume/mesh interior rejection remains strict.
      Native windows also reuse the shared procedural engine/acoustic routing;
      `--audio-cabin` selects the diagnostic cabin and `--mute` skips audio
      device creation. Headless captures are always silent.
   Native `[renderer].hdr=true` now retains pre-exposed linear scene light in
   RGBA16Float, maps it with Reinhard to SDR, then draws the HUD unchanged.
   `hdr=false` or an unsupported target uses the direct LDR fallback without an
   extra target/pass. Native auto exposure and bloom remain unapplied; captures
   state this explicitly ([doc 59](docs/59_NATIVE_HDR_DISPLAY.md)).
  `--chase` selects the
  chase framing that follows craft attitude; window keys `V`/`Backquote` toggle
   chase/reset, arrows orbit/zoom, right/middle-drag orbit/pan, wheel zooms.
   Terrain-only survey now uses the shared GPL `flight-authority::survey`
   quaternion camera in both clients: right-drag orbits, Shift-right-drag
   turns free look, wheel zooms (80 m–2,000 km), Backquote resets to 5 km.
   `--site 0|1|2`, HUD buttons or window `F6` select COAST/HIGHLANDS/VOLCANIC
   (default HIGHLANDS), resetting distance/free look while preserving orbit.
   `--survey-distance 80..2000000` applies a deterministic framing distance in
   metres after that reset for headless close-up captures; it is survey-only.
   This changes the old native 30 km framing; pre-slice terrain timings are
   not equal-workload comparisons. The celestial map camera is not ported.
  Window elapsed demand uses the
  old local client's 0.1 s clamp/8 ms work budget. Headless `--flight-step 0.016666666666666666`
  requests elapsed simulation time per frame, including warm-up; pre-stream is
  frozen. Body spin, direct lights, mesh and camera publish at the authority's
  actual completed time. Checked GPL `FlightPose` inputs are shared with the old
   pilot; its existing network interpolation arithmetic now lives in `flight-net`.
   The native child-process stdio link also lives in GPL `flight-net::embedded`;
   the old client retains only its Bevy resource adapter. A headless protocol
   smoke is available with `cargo run -p thessa-flight-net --example embedded_probe -- PATH_TO_THESSA_SERVER`
    after building `thessa-server`. Native `--server-flight` now adopts PRIMARY
    snapshots through that same link instead of stepping locally; `--server PATH`
    selects a server executable (default: sibling `thessa-server`). It is exclusive
    with local `--flight` and preview; throttle/chase/HUD controls work in either
    flight mode. Scene/body/light time uses receive-time interpolation, while
    instruments use the newest authoritative sample and throttle/engine intent
    remains separate from acknowledgement. Ordered part commands are forwarded;
    subsystem checkboxes show demands, not confirmed server state. Snapshot silence
    (`--server-timeout 5`, also the handshake deadline), pipe failure or rejected
    input freezes the last complete scene and kills/reaps the owned server, never
    falling back to local physics. Before the first snapshot the pose is static
    and instruments are omitted. Presets are queued before GPU startup/pre-stream.
    Server headless runs use wall time (including
    setup/pre-stream), not `--flight-step`. The shared reader also receives complete
    fleet membership (including explicit empty packets); a native read-only
    inspector shows stable IDs, ticks, each vehicle's own sample time and
    reference-body distance. Missing/stale packets are not treated as despawn.
    At most 63 secondaries fit alongside PRIMARY; invalid membership/poses stop
     the session. A revision-matched, on-change catalogue supplies bounded display
     names and exact authoritative contact parts (64 parts per secondary); the
     first row opens an auto-fit body-frame wireframe. Missing geometry and
     budget omission are explicit; no partial hull or X-15 copy is substituted.
      Names are labels, not model IDs. Wire v10 carries retained body-shell
     appearance with explicit absent/over-budget states (8 parts, 1,024 vertices,
     4,096 indices per secondary). Native draws available shells with a neutral
     finish through the existing opaque mesh pass, only when a secondary's
     samples bracket PRIMARY's presentation epoch; no extrapolation masks clock
      debt. Stale/revision-mismatched observations withhold world draws.
      COM-only translations reuse GPU sources and send only lightweight
      contact/pose descriptors: immutable meshes have a separately revisioned,
      bounded active-set publication. True mesh-set changes send the complete
      active payload set; this is not arbitrary incremental asset streaming.
      PRIMARY also uses its authoritative retained geometry, including after a
      split. Only the explicit standard-authority identity permits the bundled
      X-15; unknown, absent, over-budget or revision-mismatched PRIMARY appearance
      is not replaced with it. PRIMARY appearance is independent of the fleet
      quality toggle. Authored materials, full designs, prediction/reconciliation
      and reconnect remain unimplemented. v7/v8/v9 peers fail version checking.
     `VehicleDefinition` also supports optional renderer-neutral retained meshes
     with explicit assembly-body ownership. Splits partition/recenter their poses
     without copying mesh buffers. The baker now retains procedural body shells
     using the existing fuselage tessellator; wing appearance and authored
     materials remain separate work. `[body_render]` controls offline sampling
     (default enabled, axial 4/radial 32); disabling it changes no physics.
   Its input mailbox counts all commands (including state votes and fleet/
   autopilot events) against the protocol's 64-command packet limit; observed
   pipe/encode failure or overflow closes the link and refuses further input.
   This is not a heartbeat or a server acknowledgement for each queued input.
  Headless `--prestream-seconds 10 --sweep` exercises
   changing demand rather than a fixed resident snapshot.
   The default `hud` feature adds MIT/Apache-2.0 egui 0.37 on the renderer's
   existing wgpu 30 device/queue and color target: no scene texture copy,
   readback or second device. Window decorations are requested through winit;
   their appearance follows desktop/compositor policy.
   HUD controls edit camera, manual exposure and VSync; graphics changes are
   session-only and pass through the shared settings resolver. `--no-hud`
   omits UI allocation/work; `--no-default-features` excludes egui entirely.
   Headless captures omit HUD unless `--hud` is explicitly supplied.
    Manual exposure/vsync and retained fleet mesh enable/quality are consumed
    from `graphics.toml` (`--graphics PATH`). Fleet Low/Medium/High culls below
    estimated 8/2/0.5 pixels; disabled adds no fleet GPU meshes or track work.
     Missing full design/material replication, effects/production game UI/external networking and
  coarse terrain mean this is **not** a replacement client or game-FPS result;
- isolated validation harnesses for orbital and aerodynamic reference checks.

Factory gameplay, general three-spool/clutched and free-power-turbine dynamics,
combined control of jointed stacks, per-vehicle graphs/scripts/plans, client
fleet rendering, vehicle despawn policy, physical structural fracture beyond
rated-joint assessment, joint-to-link load mapping with authoritative
failure/split commit, branched fluid networks, full electrical networks, shield thermal
protection/ablation, save persistence, and production multiplayer remain future
work. Vehicle thermal nodes and the
ideal shared power bus are implemented slices; they are not full thermal,
structural, or electrical-network coupling.

## Physical and architectural invariants

- There is one gravity model: physical sources contribute simultaneously;
  sphere-of-influence boundaries are UI or optimization hints, never physics.
- The server is authoritative. Clients send commands and consume snapshots;
  they do not send arbitrary world state.
- `sim-core` is independent of Bevy, Tokio, networking, and renderer APIs.
- Authoritative spatial state uses `f64` SI values and `SimTime` rather than
  wall-clock time.
- Guidance produces physical demands. Control laws, policies, allocators, and
  actuators realize those demands; no hidden yaw/stability/drag multipliers or
  direct craft rotation are used to fake control authority.
- CPU geometry/BVH queries are the canonical server path. Hardware ray tracing
  may accelerate client work but is never the only authoritative path.
- Engine and reusable tooling crates are MIT; game-specific crates and apps are
  GPL-3.0-or-later. See [`LICENSING.md`](LICENSING.md).

## Repository layout

```text
apps/client/             GPL Bevy client
apps/server/             GPL authoritative server shell
crates/sim-core/         MIT compatibility facade for simulation-domain crates
crates/aero-core/        MIT atmosphere and aerodynamic models
crates/celestial/        MIT ephemerides, gravity fields, and simulation time
crates/trajectory/       MIT propagation and coast-cache algorithms
crates/propulsion/       MIT propulsion models and feed systems
crates/vehicle-core/     MIT rigid-body, vehicle, and mechanism models
crates/simd/             MIT optional numeric kernels
crates/fuselage/         MIT procedural body compiler
crates/plume-core/       MIT backend-neutral plume field
crates/audio-core/       MIT backend-neutral acoustic semantics
crates/audio-synth/      MIT procedural audio DSP
crates/flight-*          GPL flight authority, control, and transport
crates/autopilot*        GPL graph and JavaScript automation
crates/maneuver/         MIT trajectory planning primitives
crates/protocol/         MIT shared wire framing
crates/graphics/         MIT renderer-independent graphics settings
crates/atmosphere/       MIT shared visual atmosphere optics
crates/perf/             MIT performance capture model
crates/rcbt-*            MIT adaptive terrain topology and backend adapters
tools/                   MIT offline bakers and world generation
data/                    system, vehicle, aero, material, and worldgen inputs
docs/                    current architecture, design, validation, and ADRs
validation/              isolated external-reference workspaces
logs/                    intentionally preserved diagnostic captures
```

## Build and test

Rust `1.95+` is required by the workspace.

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets --all-features -- -D warnings
cargo test --workspace
cargo test -p thessa-sim-core --release
```

Build the checked-in system and vehicle descriptors:

```bash
cargo run -p thessa-system-baker -- \
  --input data/system.toml \
  --output data/system.baked.json

cargo run -p thessa-vehicle-baker -- \
  --input data/vehicles/example_aircraft.toml \
  --output /tmp/example_aircraft.baked.json
```

Run the client:

```bash
cargo run -p thessa-client
```

The client normally starts an embedded authoritative server process. Use
`--local` only when you specifically need the legacy in-process stepping path.
The headless server also supports stdio and TCP transports; run
`cargo run -p thessa-server -- --help` for the current CLI.

## Pilot controls

| Key | Action |
| --- | --- |
| `M` | Map / Pilot |
| `W/S` | pitch down / up |
| `A/D` | yaw left / right |
| `Q/E` | roll left / right |
| `Caps Lock` | precision control, 25% input |
| `Shift/Ctrl` | throttle up / down |
| `Z` / `X` | full throttle / cutoff |
| `Space` | engine on / off |
| `T` / hold `F` | toggle / invert SAS |
| `R` / `Y` | RCS / reaction wheels |
| `G` / `P` | landing gear / parachutes |
| `V` | free / follow camera |
| `` ` `` | reset camera |
| `Escape` / `F8` / `Pause` | pause |
| `F1` | controls and HUD legend |
| `F2` | hide / show UI |
| `F3` | extra telemetry |
| `F7` | coastal flyby guidance (manual input takes over) |
| RMB / MMB / wheel | orbit / pan / zoom |

The pilot interface contract is documented in
[`docs/10_PILOT_INTERFACE.md`](docs/10_PILOT_INTERFACE.md).

## Validation and benchmarks

Orbital reference harness:

```bash
cargo run --manifest-path validation/nyx-compare/Cargo.toml --release
```

Aerodynamic comparison without external packages:

```bash
cargo run --manifest-path validation/aero-compare/Cargo.toml --release
```

Optional local JSBSim/RocketPy comparison:

```bash
python -m venv .venv-aero
.venv-aero/bin/pip install jsbsim rocketpy

THESSA_AERO_PYTHON=.venv-aero/bin/python \
  cargo run --manifest-path validation/aero-compare/Cargo.toml --release -- \
  --require-external
```

Selected benchmarks:

```bash
cargo bench -p thessa-sim-core --bench gravity
cargo bench -p thessa-sim-core --bench affine_prop
cargo bench -p thessa-sim-core --bench aero
cargo bench -p thessa-sim-core --bench flight
cargo bench -p thessa-maneuver --bench porkchop
```

The CI workflow runs formatting, Clippy, cross-platform builds/tests, release
simulation checks, benchmark compilation, and isolated reference validation.

## Documentation map

Start here:

1. [`docs/00_STATUS.md`](docs/00_STATUS.md) — living implementation/future-work
   index; start here for the current status of each major document.
2. [`docs/03_PHYSICS_ENGINE.md`](docs/03_PHYSICS_ENGINE.md) — physics contracts
   and implemented numerical paths.
3. [`docs/04_RUNTIME_ARCHITECTURE.md`](docs/04_RUNTIME_ARCHITECTURE.md) — current
   process, crate, threading, and transport boundaries.
4. [`docs/07_AUTOPILOT.md`](docs/07_AUTOPILOT.md) — current graph and scripting
   layer plus future standard-library work.
5. [`docs/08_NUMERICAL_VERTICAL_SLICE.md`](docs/08_NUMERICAL_VERTICAL_SLICE.md)
   — numerical validation and known limits.
6. [`docs/10_PILOT_INTERFACE.md`](docs/10_PILOT_INTERFACE.md) — pilot HUD and
   control contract.
7. [`docs/11_AERODYNAMICS.md`](docs/11_AERODYNAMICS.md) — aero model and
   reference matrix.
8. [`docs/details/README.md`](docs/details/README.md) — vehicle subsystem and
   authoring specifications, including implemented surface/body compilers,
   landing gear, reaction wheels, and parachutes.
9. [`docs/21_TERRAIN_STREAMING_THROUGHPUT.md`](docs/21_TERRAIN_STREAMING_THROUGHPUT.md)
   — current terrain streaming and CBT boundary.
10. [`docs/22_RCBT_GPU_TERRAIN.md`](docs/22_RCBT_GPU_TERRAIN.md) — CBT GPU
    boundary and replacement gates.
11. [`docs/42_EPHEMERIS_RESIDUAL_STORAGE.md`](docs/42_EPHEMERIS_RESIDUAL_STORAGE.md)
    — bounded residual compression for ephemeris/gravity caches.
12. [`docs/43_AERO_COEFFICIENT_RESIDUAL_STORAGE.md`](docs/43_AERO_COEFFICIENT_RESIDUAL_STORAGE.md)
    — bounded residual storage for large aerodynamic coefficient fields.
13. [`docs/05_ROADMAP.md`](docs/05_ROADMAP.md) — dependency-ordered future work.
14. [`CHANGELOG.md`](CHANGELOG.md) — notable changes.

ADRs live in [`docs/adr/`](docs/adr/). Design-only documents are labelled as
vision, proposal, or future work; implemented behavior is described in the
current implementation documents and source-level API comments.

## Rocky world generator

Thessa reconstruction includes recipe-controlled climate, curved mountain arcs,
branching rifts and regional drainage-driven wetlands. The runtime continuation
adds ecological biomes, catchment-driven river reaches, connected lake
basin descriptors and inhabited regions. Ocean-supply/orographic moisture,
routed annual lake budgets, and an explicit frozen-erosion adapter now share the
canonical field path (doc 50). Client and server now bundle the same source-checked
erosion artifact; default launch/contact consumers share an immutable field.
The preserved global bake now has three 256 m runoff-erosion refinements (doc 52)
shared by height/material/contact consumers. Depositional regions reduce fine
source relief; snow cover is limited by the existing precipitation proxy.
The rejected survey frames are tracked in doc 53; geological substrates and
ecological optical cover are now distinct, and native land shading consumes
roughness. These changes do not establish visual acceptance.
Material pages use fixed-scale slope/curvature and displacement, with
residency-edge filtering and physical-texel appearance filtering (docs 51–52).
Local water geometry,
vegetation rendering and infrastructure remain incomplete. See
[doc 50](docs/50_THESSA_MOISTURE_EROSION_BRIDGE.md),
[doc 51](docs/51_THESSA_RUNTIME_GEOMORPHOLOGY.md),
[doc 52](docs/52_THESSA_LOCAL_EROSION_REFINEMENT.md),
[doc 53](docs/53_THESSA_SURFACE_MATERIAL_REVIEW.md),
[doc 49](docs/49_THESSA_RIVERS_ECOLOGY_SETTLEMENTS.md) and the worldgen README;
source validation is not visual acceptance.

`thessa-worldgen-rocky` is part of the workspace. It generates deterministic
rocky-world fields and exports client textures. Launch-site and configured
terrain-contact queries use the shared canonical height field; detailed water
contacts and surface transport remain future work.

Thessa's reconstruction follows the captured-world design, retaining wetlands
and other lowland relief rather than making fresh craters the main landmarks.
Climate, arc/rift and regional hydrology changes are staged in an isolated
worktree; [doc 48](docs/48_THESSA_RECONSTRUCTION_2026_10_01.md) records the exact
source boundary, measurements and open acceptance gates. They are not yet the
main-tree planet or evidence of improved gameplay rendering.

```bash
cargo run -p thessa-worldgen-rocky -- \
  check --manifest data/worldgen/thessa_demo.toml

cargo run --release -p thessa-worldgen-rocky -- \
  export-client-texture \
  --recipe data/worldgen/worldgen_recipe.toml \
  --body-file data/worldgen/thessa_v02.toml \
  --out /tmp/thessa-preview.png --width 2048
```

Flight instrument SVG sources and their 4x PNG exports are in
`assets/ui/flight/`; F1 includes their illustrated legend.

## Contributing

Read [`AGENTS.md`](AGENTS.md) and [`CONTRIBUTING.md`](CONTRIBUTING.md) before
changing simulation code. Physics changes should include a known-case test,
an error envelope against a decision-relevant scale, and a benchmark when the
change affects a hot path.

## Agent quality infrastructure

The agent development guide and blind visual A/B workflow are documented in
[agent quality control](docs/55_AGENT_QUALITY_CONTROL.md). Their executable
architecture check and visual-arena tools live under
[`tools/agent-qa/`](tools/agent-qa/README.md). The current arena works on
supplied PNG frames. Opt-in [native capture provenance](docs/89_REFERENCE_CAPTURE.md)
adds actual PNG/linear HDR/perf evidence; automated model judging remains planned.

Native render-work continuation: [Horizon source review and work counters](docs/80_HORIZON_RENDER_WORK_REVIEW.md) records conservative primary mesh selection, persistent indexed indirect batches and dirty-only instance/command uploads. The fixed coastal primary plan drops from 112.8M to 17.1M triangles at 37 API batches. [Texel-bounded depth-only plant geometry](docs/81_NATIVE_SHADOW_MESH_LOD.md) reduces the subsequent cold shadow plan from 999.6M to 283.2M triangles within the existing source-byte budget. These are scoped work counts, not target-GPU frame-time acceptance.

The [shadow submission continuation](docs/82_NATIVE_SHADOW_SUBMISSION.md) adds receiver-aware caster selection, shared-budget indirect batches and cached world bounds. At that checkpoint the ocean profile retained full-volume shadow selection; 6,930 logical shadow commands used 453 API batches. Physical-GPU/full-frame and reference acceptance remain open.

The [shadow geometry hierarchy](docs/83_NATIVE_SHADOW_HIERARCHY.md) reduces the same-build coastal mesh shadow plan from 283.22M to 252.05M triangles under the unchanged 0.5-texel request, with the original near tier retained and an unresolved-organ far tier. This remains a work-count checkpoint; physical-GPU frame time and reference acceptance are open.

[Empty shadow sample selection](docs/84_NATIVE_SHADOW_RASTER_SAMPLES.md) reduces the same coastal plan further to 208.10M mesh shadow triangles (17.44% below the hierarchy checkpoint). It retains source geometry and rejects only conservative bounds containing no shadow sample; unknown raster precision keeps the ordinary caster path. Indirect command fragmentation increases, and target-GPU frame-time acceptance remains open.

[Depth vertex streams and physical GPU validation](docs/85_NATIVE_DEPTH_VERTEX_STREAMS.md) retain the 208.10M-triangle shadow plan while separating positions from material attributes. Matched Radeon 880M release diagnostics lower the sum of serial layer GPU central values from 102.78 to 67.91 ms (-33.93%), with byte-identical complete layer depths. This is scoped shadow-stage evidence; whole-frame moving-camera and reference acceptance remain open.

[Opaque sidedness and stable selection demand](docs/86_NATIVE_OPAQUE_SIDEDNESS.md) removes mixed-material fragment discard through shared-layout culling variants and reuses deterministic stationary demand from an immutable source. The topology investigation confirmed normal convergence; its generation/publication rules remain. Exact image regressions and ordinary native GPU profiles have separate evidence scopes.

[Cloud cost and finite Fresnel endpoints](docs/87_NATIVE_CLOUD_COST_AND_FRESNEL.md) fixes normalized-dot overflow in the shared Schlick formula across environment, stellar, mesh and ray-hit shading. Measured average-response-table and cloud-loop candidates were rejected; cloud transport remains the identified performance target, with no quality downgrade or reference-parity claim.

[Engine-boundary refactoring](docs/88_ENGINE_BOUNDARIES_AND_REFERENCE_SCENES.md)
starts with a 43-package ownership audit and resolved dependency gate in the
existing workspace. R1–R4 are implemented: direct collision type ownership,
shared terrain mesh ABI requirements, and one app-owned mesh-frame publication
path for native window/headless hosts. R5 capture provenance is implemented;
[B01 material-lab data](docs/90_REFERENCE_SCENES.md) now exports generated
geometry and explicit camera/light metadata as GLB/JSON without GPU or game
state. Its native capture now uses the existing mesh/display/perf consumers,
with verified static 880M repeats and explicit scene/geometry hashes. B02
asset ingestion remains staged; no coastal speedup or reference parity claimed.
