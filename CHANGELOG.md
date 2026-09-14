# Changelog

Notable Project Thessa changes are recorded here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions follow
Semantic Versioning where it is meaningful for a pre-alpha prototype. Internal
APIs, data formats, and save files are not stable before `0.1.0`.

## [Unreleased]

### Added

- A shared baked free-flight trajectory for the map and flight view, an
  ephemeris table, and a simulation-time event scheduler; baking runs in a
  compute worker.
- Coast accuracy checks over a 200,000 s horizon and baking/256-cache
  benchmarks exposed through the performance monitor.
- The rocky world-generation branch in the workspace, including a shared 16K
  Thessa v2 texture for map and flight views.
- A recorded 581 s spin-up regression and free-rotation energy/angular-momentum
  checks.
- Bevy 0.19 pilot/PFD slice with `SURFACE`, `AIR`, `ORBITAL`, and `TARGET`
  speed frames, datum/AGL altitude modes, and a dynamic navball.
- A client-local X-15 flight-test adapter using the shared `sim-core` 6-DoF
  integrator, multi-body gravity, and atmosphere/aero pipeline.
- KSP-like Mouse Aim, Navball/SAS, Rate Control, and Direct/Raw control modes.
- Low-overhead CSV flight tracing for reproducing bad flight states.
- MIT aerodynamic runtime with an aggregated panel model, local `omega x r`
  flow, compressibility, smooth stall, transonic/supersonic corrections,
  control surfaces, dynamic damping, and optional coefficient tables.
- Deterministic atmosphere provider with `T/p/rho`, speed of sound, and
  viscosity.
- Generic serializable `VehicleDefinition` and the TOML-to-JSON
  `vehicle-baker` path.
- Isolated aero validation harnesses for JSBSim, RocketPy, VSPAERO, AVL, SU2,
  and OpenRocket reference workflows.
- Sampled rigid-body duration API for recalculating gravity, altitude, wind,
  and control inputs at deterministic substeps.
- `Reset` in flight wire protocol v4, restarting at the canonical site while
  preserving simulation time.
- Declared terrain obstacle heights and geometric track certification for
  unattended craft, future landing guidance, and impact prediction.
- Shared `canonical_launch_setup` helper so server and client surveys derive
  the same launch sites from one recipe.
- Raster water reflections using a procedural HDR sky cubemap and per-tile
  `EnvironmentMapLight`; SSR/RT reflections and waves remain future work.
- Unattended frame measurement through `THESSA_AUTOBENCH=1`, with warm-up,
  x1/x8/x64/x256 warp levels, JSON/CSV output, and the same path as Shift+F4.
- Cohort gravity patches and bounded affine propagation in `sim-core`, with
  exact-near terms, error envelopes, deterministic fallback, and fleet/planner
  benchmarks.
- `thessa-maneuver`: typed maneuver plans, Hohmann/circularization/Lambert/
  plane-change helpers, candidate search, and finite-burn execution commands.
- Typed autopilot graph execution, simulation-time waits, sandboxed QuickJS
  blocks, guidance targets, landing/impact site declarations, and server-side
  graph/plan execution.
- Backend-neutral RCBT topology/page contracts, packed CPU/reference
  implementations, a universal Bevy frame plugin, cube-sphere Morton mapping,
  and an optional portable wgpu adapter. The client runs CBT scheduling beside
  the existing CPU terrain renderer while GPU geometry parity is validated.

### Changed

- Verlet baking reuses the ephemeris snapshot and accepted endpoint
  acceleration; old rails points are trimmed in accumulated blocks.
- The performance monitor and CSV separate on-rails time and include it in
  effective warp.
- The client normally flies through an embedded authoritative server process;
  `--local` preserves the legacy in-frame stepping path.
- Sun-shadow cascades cover kilometre-scale distances; visible low-sun shadows
  still depend on time of day and warp.
- Terrain and atmosphere rendering use interpolated render poses instead of
  snapshot cadence, while telemetry remains authoritative.
- Pilot near-plane and local-relief budgets are dynamic; streamed terrain
  retains old coverage until replacement tiles are ready.
- The default visual `density_scale` changed from `0.55` to `0.3`; simulation
  pressure was not changed.
- Safe SIMD wrappers, zero-gravity bodies, cache extension rollback, and SAS
  handling in coast batches were hardened.
- Angular stepping uses an implicit midpoint/Cayley rotation so free rotation
  does not gain energy from explicit Euler integration.
- The pilot HUD now uses a compact KSP-like bottom dock, 21 vector icons, and
  F1/F2/F3 visibility levels.
- X-15 rendering uses the shared starter asset, physical FBW allocation, and a
  fixed 120 Hz loop. Trace names include surface commands and SAS targets.
- X-15 asset axes, control mappings, telemetry reference frames, finite-planform
  aero corrections, and supersonic wave-drag terms were aligned with physics.
- The Thessa design target is `R=3200 km`, `g≈0.500 g`, and `p0=1.20 bar`.
- CI is split into cross-platform, quality, release-simulation, and reference
  validation jobs.

### Fixed

- Runtime RT/raster switching on supported GPUs keeps the deferred prepass.
- Warp is limited by the CPU budget without changing the physics step; the
  effective speed is visible in the performance monitor.
- Trajectory caches invalidate on mass/orbit changes; Hermite velocity matches
  the position derivative; table boundaries and non-gravitating contacts are
  checked.
- The map draws only the future coast path with a bounded vertex count.
- Streamed surface tiles appear incrementally and LOD transitions preserve
  coverage until the replacement is ready.
- Pilot render-origin jitter, horizontal-tail lift signs, control directions,
  SAS handoff, RCS saturation behavior, target/FPV placeholders, AGL display,
  map/metre coordinate mixing, and non-finite flight states were corrected.
- Atmosphere rotation now converts `omega` to craft coordinates before
  evaluating `omega x r`.

### Validation

- RocketPy apples-to-apples fin-set: `CL_alpha` error `0.000002%` and center
  of pressure error `0.000%` at `M=0.95`.
- X-15-like 5 s / 100 Hz proxy against JSBSim: Mach error `0.246%`, altitude
  error `0.905%`; AoA remains a proxy gap and is not used for global tuning.
- Nyx/ANISE remains isolated from the runtime dependency graph.

## [0.0.1] - 2026-09-07

### Added

- The first numerical vertical slice: baked deterministic ephemerides,
  multi-body test-particle gravity, adaptive Dormand–Prince 5(4), and
  velocity-Verlet.
- `system-baker`, a reproducible system descriptor, and the initial design
  system.
- Bevy hierarchical celestial map.
- MIT engine/GPL game licensing boundary and ADR baseline.
- Nyx/ANISE numerical validation and Lagrange-point reference vectors.

> No Git tag/release exists for `0.0.1`; this changelog does not pretend that a
> release link exists.
