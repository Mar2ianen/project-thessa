# Changelog

Notable Project Thessa changes are recorded here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions follow
Semantic Versioning where it is meaningful for a pre-alpha prototype. Internal
APIs, data formats, and save files are not stable before `0.1.0`.

## [Unreleased]

### Fixed

- False effective cloud telemetry: harmonized `active_cloud_representation()` to respect `THESSA_CLOUD_BACKEND` environment overrides across `thessa-graphics`, `perf.rs`, and `beauty.rs`.
- Disabled feature resource allocation in `setup_beauty`: multi-megabyte texture bakes, mesh displacements, and entity spawning are bypassed when clouds, gas giants, or aurora are disabled, using 1x1 dummy images (saving 10+ MiB RAM and startup latency); volume meshes and materials are only allocated when `clouds_volumetric` is true; marked `clouds.enabled`, `clouds.volumetric`, `upper_atmosphere.aurora`, and `gas_giant.enabled` with `restart_required: true`.
- Nyquist bandlimiting across the cloud volume pipeline in `cloud_volume.wgsl`: octaves fade to their verified empirical/analytical expectation/mean (0.933 for `billow_cubic`, confirmed by 50,000-sample test) to strictly conserve field energy across varying view step chords without coarse LOD density loss.
- Space-skipping peek coordinate sync: removed out-of-sync `deck_uv(p)` peek without wind shear in favor of single-source evaluation in `slab_density`, eliminating redundant texture fetches and edge culling artifacts.
- Exact extinction parity: removed arbitrary `0.95` multiplier on extinction in Backend A, restoring exact Beer-Lambert parity with Backend B and analytical integrals.
- Raymarch tower bounds: expanded analytical ray-sphere ceiling to `cloud.shape.z` ($1.5\times$ slab thickness) preventing clipping of cloud towers up to $1.35\times$ thickness.
- Convergence and parity verification: replaced clamped integration test with unclamped Simpson 1/3 solver and exact GPU boundary-reset trapezoidal model in `thessa-atmosphere`, plus genuine hardware compute readback testing on AMD Radeon 880M in `thessa-bevy-rcbt` directly slicing production WGSL and evaluating against production Rust functions in `thessa_atmosphere::clouds`.
- Documented physical ($h \approx 3\text{ km}$) vs presentation ($h \in [25.6, 44.8]\text{ km}$) cloud altitude mapping and scene-depth limitation in `docs/14_VISUAL_ATMOSPHERE.md`.

### Added

- Low-deck volumetric cloud march (`[clouds] volumetric`, Medium/High; Low
  keeps the shell fallback, `enabled = false` stays free): per-pixel
  Beer-Lambert march through a planet-radii slab over a domain-warped
  EVE-style weather bake plus fully GPU-side per-pixel weather (cloud
  family, deck-height levels, wind shear), Schneider (Horizon Zero Dawn) shaping and lighting —
  coverage remap, type-dependent height gradient, five-octave ridged
  billow-detail erosion with footprint fade,
  Beer-Powder sun through-scatter, dual-lobe phase with forward
  silver lining, height-graded ambient (dark base, bright top) fed by the
  shared eclipse visibility. Decks read as puffy volume from orbit, inside
  the slab, and from the surface instead of painted alpha. Backend-neutral
  slab math + CPU oracle in `thessa-atmosphere::clouds` (interval/planet-clip/
  shaping/lighting tests, error envelope, `cloud_slab` bench); raster
  consumer in `assets/shaders/cloud_volume.wgsl` (dual integrators selectable via
  `[clouds] integrator`: Backend A real-time stratified ray marching, Backend B
  stochastic volumetric scattering approximation; max 24 view steps from
  `ray_steps`, discrete per-step weather-bake evaluation, transmittance early-out,
  step-aware Nyquist bandlimited noise octaves, turret-scale sun taps). Cirrus
  stays a streaky shell; cloud shadows sample the deck bake with physical
  cumulus deck parallax (~3 km).
- Physical cloud-deck winds (was: 12 km/s hurricane drift): decks co-rotate
  with the planet plus zonal breezes (~15 m/s low deck, ~40 m/s cirrus jet)
  about the planet pole axis, with an f64-folded phase that never jitters
  under warp; shader detail advects at matching speed with a seamless
  pre-wrapped UV phase. Far-hemisphere volume fragments are culled in-shader
  (they smeared a white sheet past the limb where terrain wrote no depth).
  The CPU per-tile cloud-shadow path no longer runs under GPU raster (its
  constants printed tile-edged quads over the smooth per-pixel field).
- Low-deck cloud shadows on terrain (`clouds.cast_shadows`, on for High/Ultra
  presets): the GPU surface samples the deck bake the shells show, offset by
  sun parallax for the physical deck height with soft alpha shaping over the
  direct sun only (skylight/ambient stay); CPU-streamed tiles apply the same
  deck as a per-tile factor. Cirrus casts no shadow.
- Night-side city lights (`[night_lights]`, bake resolution by quality):
  warm emissive clusters baked once at startup from the world's inhabited
  regions, gated by the terminator in the GPU terrain shader; the CPU
  fallback path has no emissive slot and stays dark.
- Cloud-deck relief (tangent-space normals from the coverage field),
  three-shell aurora curtains, and stars placed for the manual exposure.
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
- Deployable parachutes, reaction-wheel banks, and typed installed-part
  commands over flight wire protocol v5, including named gear and canopy
  controls for future stage/action-group dispatch.
- Procedural lofted fuselages with derived interior volume, tank/body-strip
  compilation, aero controls, conservative contact geometry, and a combined
  lifting-body/wing fixture.
- Procedural lifting-surface compilation for authored planforms, bend, sections,
  controls, fold-state geometry, structural sizing, and contact parts; the
  vehicle baker merges `[[procedural_surfaces]]` into vehicle assets with
  rebased panel/control/fold references. Runtime wing-fold actuation and render
  mesh generation remain future work.
- Experimental microscaled material-page storage: adaptive codec and residency
  phases A–E, GPU LOD/mip/aniso sampling, and a render-world storage A/B. The
  compact path currently pre-decodes to RGBA; raw storage remains the default,
  and crack-free height geometry/canonical baked-format adoption remain open.
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
- Atmospheric beauty effects (all behind graphics settings): procedural
  gas-giant bands (CPU-baked), 1–2 cloud shell decks for ocean worlds,
  Low-quality engine-plume cone fallback, and aurora shell over the analytic
  oval. New `graphics.toml` sections
  `[gas_giant]`, `[engine_plume]`, extended `[clouds]` and `[upper_atmosphere]`
  (aurora intensity/animation). The plume reads `EnginePlumeInput`; live
  compiled engine/nozzle state is not yet wired into the client renderer.
- New `plume-core` crate: backend-neutral `PlumeSource`/`PlumeEnvironment`
  contract, analytic axial mean profile (pressure-ratio regime, shock-cell
  spacing), participating-medium CPU oracle (`sample_medium`, `integrate_ray`,
  `radiant_power`), exhaust optical material table with a shared CPU/GPU hue
  ramp, and a plume-local RCBT adapter. 25 semantic tests and a `profile`
  bench.
- Field-first plume renderer (Medium/High): a camera-facing ribbon carries
  coverage while `assets/shaders/plume_volume.wgsl` marches view rays through
  the round cross-section and Beer-Lambert-integrates the `plume-core` mean
  field. Cone impostor kept for Low; pilot-view gating kept. The point-light
  proxy intensity is derived from the field's integrated radiant power.
  field. Cone impostor kept for Low; pilot-view gating kept.
- Coastal beauty flyby (F7 in pilot view, `THESSA_FLYBY=1` for unattended):  the live X-15 flies a low pass on honest stick/throttle intents through the
  normal input wire (no teleports, no survey scenes), holding datum altitude
  and 200 m/s with a bank-hold leg along the shoreline (heading searched from
  field geometry, turned with SAS-captured banks) and a hard safety climb
  below 300 m. Surface kinematics subtract the reference body's ephemeris
  velocity (using raw inertial speed opens the governor at orbital velocity
  and stalls the wing — measured terrain impact, pinned by a 30 s closed-loop
  test). Any stick/throttle key, M, or an error disengages instantly;
  engaging switches to chase camera. Tuned for 1x warp; high warp may
  porpoise (it recovers).
- Fixed chase camera looking at the sky: `-craft_orientation * Vec3::X`
  negates the quaternion (unary minus binds tighter), and -q is the same
  rotation as q, so the sign vanished and the basis became a reflection that
  `from_mat3` turned into garbage. Parenthesized vector negation; camera
  framing against spawn and level attitudes is now pinned by tests.

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
- Thessa shoreline beauty pass (visual only, heights/classification untouched):
  surf-foam waterline, turquoise shallow shelf, wet-sand rim; the far
  `assets/worlds/thessa-v3` globe is rebaked with the same appearance model as
  tiles/GPU pages so orbit and surface agree (ocean glossy for sun glint,
  foam/ice matte). Clouds bake white cores with cool gray-blue skirts and run
  two decks (low + cirrus); `aerial_perspective` is on and honestly wired
  (off collapses the aerial LUT instead of a dead toggle).
- Pilot-view sky actually has clouds now: beauty followers read computed
  visibility (the PFD planet only inherits it, so the old explicit check never
  matched and shells stayed hidden), fall back to the live terrain streamer
  placement when the entity match is hidden, and fade whole-sphere decks out
  until well above deck altitude (near/inside they are giant soap blobs — a
  full-screen veil measured at 60 km). The 60 km column stays nearly clear
  per physics (pinned by a 60 km oracle regression test on the scattering
  model); the merged renderer follows the upstream aerial-LUT range for
  horizon coverage — re-measure the mid-altitude veil against it if haze
  looks wrong.
- Cloud decks bake billow cells plus cool-skirt shading instead of smooth
  blobs; aurora bakes field-aligned curtain rays over the intact oval.
  Shell pole UVs collapse to one column (a full texture row smeared over
  each pole triangle read as giant white wedges from inside).
- LOD-faded surface grain/micro (`surface_appearance_filtered`): coarse texels no
  longer alias sub-texel octaves into random per-tile means (the far-view
  quilt); mesh vertices and LOD-independent GPU pages keep full detail.
- Cloud/aurora shell pole fix: Bevy's UV sphere spans a full texture row over
  each pole triangle, which reads as giant white wedges from inside the shell
  (measured on a 30 km maneuver frame). Pole UVs collapse to one column.
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
- Volumetric plume no longer draws a milky sheet wider than the jet body:
  medium weight is hard-clipped outside the true barrel radius, turbulence
  runs azimuthally (fixed axis basis) at low frequencies instead of
  axisymmetric rings, shock cells are softened, march steps 8/14
  (Medium/High). The tail dissolves toward `z = len` instead of a flat cut,
  `atan2` at the axis is guarded against NaN on strict drivers.

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
