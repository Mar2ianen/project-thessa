# 00 — Documentation status: implemented vs future

Status: living index, worldgen rows reviewed 2026-10-03. Each row states what is merged
reality and what is still design. Per-file `Status:` headers agree with this
table; open decisions live in `docs/06_OPEN_QUESTIONS.md`. Regenerate
`data/system.baked.json` via `thessa-system-baker` after any `data/system.toml`
edit.

The numerical implementation is split across `thessa-aero-core`,
`thessa-celestial`, `thessa-trajectory`, `thessa-propulsion`, and
`thessa-vehicle-core`; `thessa-sim-core` preserves the established aggregate
API for existing consumers.

Legend: ✅ implemented · 🟡 partial (shipped slice + open remainder) ·
🔵 design baseline/target (no code yet) · 🕰️ historical record (dated audit,
do not update in place).

## Simulation and physics

| Doc | Status | Implemented | Future |
| --- | --- | --- | --- |
| `01_CELESTIAL_SYSTEM.md` | ✅ reference | baked ephemerides, summed gravity, 32-body working set | stability/Halo/co-orbital validation checklist |
| `03_PHYSICS_ENGINE.md` | 🟡 prototype | DP5-FSAL + DP8-DOP853, variational STM, EphemerisFrame, monopole/quadrupole gravity tree, single-tick cohort patches with Hessian bounds, thrust/RTN arcs, envelope-barrier + active-set allocator, Rapier contact backend, gated J2/C22 harmonics, lumped vehicle thermal network and ideal electrical bus | time-span patches, thermal/structural coupling, fluid/resource graphs, CFD, fracture |
| `08_NUMERICAL_VERTICAL_SLICE.md` | 🟡 prototype | test-particle contract, baked hierarchy, on-rails/Verlet, harnesses, gated J2/C22 | fitted segments, higher-degree harmonics, joint multi-leg shooting, structural coupling and shield thermal protection |
| `11_AERODYNAMICS.md` | ✅ runtime model | panel SoA/SIMD + tables + post-actuator aero-result reuse (2.60× in 64-panel microbenchmark) + fuselage Munk strips/body controls + composition-aware atmosphere samples + upper-band/vacuum reductions | per-body vertical atmosphere profiles, weather/winds, full wake/occlusion, high-enthalpy chemistry and viscous hypersonics, arbitrary table axes |
| `details/02_PROCEDURAL_AERO_SURFACES.md` | 🟡 compiler/runtime slice | procedural planform/bend/section compiler, tolerance/error-budget panelization, baked five-channel control mixing with arbitrary surface counts, control ownership, compiled fold-state records, structural sizing, hex tile-layer toggle, contact parts, vehicle-baker integration | render mesh/editor UX, runtime wing-fold actuation, flap/airbrake input wiring, broader structure/failure coupling, solver-integrated airfoil polars, authored catalogs and balance |
| `details/03_PROCEDURAL_FUSELAGES.md` | 🟡 implementation slice | loft compiler, interior volumes, pressure-shell/tank regions, aero body strips and controls, cabin seat/monument geometry and mass, mass/inertia, conservative contacts, renderer-neutral mesh | editor UX, cutouts, section roll, fluid redistribution and structural failure |
| `details/05_CABIN_EDITOR.md` | 🟡 implemented authoring/runtime slice | seat blocks and per-place suit overrides, class presets, monuments, paired static exits, decks and 747/Concorde/fighter layouts, TOML baker, runtime cabin inventories and pilot authority, connected-domain vent/repress, COM recentering across body-frame mounts | evacuation, moving/cutout doors, metabolic O2, finite vent/repress flow, consumables and editor UX |
| `details/06_ATTACHMENT_MODEL.md` | 🟡 implemented assembly/resource slice | authored attach nodes, pose solving, transformed geometry/mass, validated part trees, runtime crew/air domains, fuel reachability, named non-tree crossfeed edges and path-specific pressure-limited feed-line routing with per-consumer fluid properties, shared fixed-step tank allocation/commit and transfer with rocket mounts on global allocation, contact-scene D1 joint telemetry, topology splitting with subtree-entry roots, per-link strength ratings with reporting-only solver-load assessment, baked per-body ownership with definition-split migration and live tank/solid inventory preservation (bench: 12.85 μs), dynamic per-body mass records for tank, solid-motor, and cabin-air changes, mass/COM/inertia reconstruction from complete per-body properties (64-body benchmark: 29.78 μs), authored D1 docking ports, and server-level fleet ownership with separation spawning, bus-preserving root clusters, dock sessions, vacuum-gated joints, and fleet snapshots | explicit per-part bodies for hand-authored hardware, drive/gear/chute/fold/disc migration, independent power/thermal network partition, internal-joint load-path resolution, joint-to-link load mapping with authoritative failure/split commit, jointed-stack control, per-vehicle graphs/scripts/plans, client fleet render, despawn, branched hydraulic networks and finite-rate cabin flow |
| `details/04_PROCEDURAL_PROPULSION.md` | 🟡 resource-aware runtime slice | compiled chemical/solid/nuclear-thermal, airbreathing/ESTOC/APU, electric, shaft-power, RCS and continuous/pulsed fusion models; fixed-step mounted-consumer allocation and moving-mass/inertia updates; tank-backed pneumatic/rocket-bootstrap starters, optional torque-rated starter/generator hardware, generator efficiency maps/local thermal limits, APU and jet generator export to the bus, independent LP/HP turbofan rotor dynamics with geared fan and coupled steady solve | general three-spool/clutched and free-power-turbine topologies, multi-spool ESTOC transitions, generator thermal-node integration, transient piston/electric source fidelity, higher-fidelity propulsion models and editor UI |
| `20_ADVANCED_AERO_EFFECTORS.md` | 🟡 partial foundation | incidence controls and hinged fuselage body strips with load-limited actuators | general flap/spoiler/hinged-panel/grid-fin models, neutral bounds, `AeroEffectorModel`, high-speed plan (§§17–20: boom/buffet/plasma/vortex/ground-effect) |
| `23_GRAVITY_FIELD_COHORTS.md` | 🟡 partial | monopole tree, quadrupole rung, single-tick patches, Hessian spatial bound | time-span patches, cohort keys, planner-patch reuse, GPU |
| `24_ANALYTIC_AFFINE_PROPAGATION.md` | ✅ prototype | far-only analytic STM on single-tick cohorts | atmosphere/thrust/contact integration, global proof |
| `40_RAPIER_COLLISION_INTEGRATION.md` | ✅ baseline | local contact solver, zero-gravity Rapier, readback, regime switch, stale contact telemetry clearing, articulated wheel bodies/joints, sensor-only tire queries, split mass properties, powered wheel/strut/brake/drive stepping | dynamic-body wheel contacts, richer terrain contact boundary, full PBR parity |
| `details/05_PROCEDURAL_LANDING_GEAR.md` | 🟡 partial | wheel/tire/strut/brake/drive laws, mass/COM bake, sprung/unsprung split, articulated Rapier wheel bodies/joints, retractable chassis, fold-out legs, powered authority state/telemetry | dynamic-body wheel contacts, granular soil response, steering/anti-skid, electrical bus limits, representative fleet benchmarks |
| `details/06_REACTION_WHEELS.md` | ✅ implemented model | named banks, per-axis torque allocation, automatic bus-load coupling (idle + marginal torque draw, same-name utility consumer, starvation yields to RCS), rotor-momentum reservoirs with saturation and RCS desaturation, rotor-speed and saturation telemetry, motor-heat ledger routable to thermal nodes, mass/inertia bake, RCS residual routing, telemetry and pilot controls | thermal state |
| `details/07_PARACHUTES.md` | ✅ implemented model | named packs, pressure/q deployment gates, reefing, viscous-elastic suspension lines with shock smoothing, canopy breathing deformation, aerodynamic loads/failure, cut/failed repack servicing with authored timer and clean restow, mass/inertia bake and telemetry | inflation shock beyond line compliance |
| `details/08_VEHICLE_PART_COMMANDS.md` | ✅ implemented API | ordered group/named commands on current wire v10 (per-vehicle targeting introduced in v7), including per-engine throttle and propellant transfer plus parachute repack, fleet Separate/Dock/Undock, and per-vehicle input/guidance targeting with per-vehicle control mode, guidance, and maneuver/burn executors; pilot local prediction and server application | stage-definition resolution, action-group routing, and per-vehicle graphs/scripts/plans |
| `details/09_ELECTRICAL_POWER.md` | 🟡 implemented vehicle slice | ideal shared bus, prioritized consumers, battery/ultracapacitor energy/efficiency limits with pooled headroom, ultracapacitor self-discharge, fission and fuel-cell sources, reactor live-mass drift, APU and jet generation, solar arrays/occlusion/single- and two-axis tracking, mounted electric-propulsion load coupling, automatic reaction-wheel load coupling, docked-bus exchange across a shared interface, vehicle-baker TOML and 64-vessel benchmark | automatic load coupling for other nonpropulsive actuators, electrical circuits/current dynamics, cycle ageing, self-shadowing, three-axis gimbals |
| `details/10_THERMAL_SYSTEM.md` | 🟡 implemented vehicle slice | lumped thermal nodes with conduction links and fixed/foldable area radiators, single-axis radiator pointing gimbals with edge-on auto-tracking, shared occluded solar inputs with own-hull ray occlusion, Sutton-Graves aero heating, wired internal loads, authored reactor/fuel-cell/generator waste-heat routing, stability-substepped integration, overheat reporting, mass/COM baking, vehicle-baker TOML and 64-vessel benchmark | shield-specific thermal-protection/ablation coupling (design baseline in 11), temperature-dependent strength, phase change, convective cooling, silhouette penumbra, multi-axis radiator gimbals, automatic engine-to-node heat wiring past the authored routes |
| `details/11_HEAT_SHIELDS.md` | 🟡 partial slice | Newtonian shield-disc aerodynamics through the common drag/lift force contract, baker/vehicle wiring, wing hex tile-layer toggle with mass and lumped thermal nodes | ablative recession/burn-through, shield-specific thermal protection, spline-bounded tile screens, tile detachment, jettison, subsonic disc blend |

## World and lore

| Doc | Status | Implemented | Future |
| --- | --- | --- | --- |
| `02_WORLD_ATLAS.md` | 🟡 partial | §§1–4, Mora midpoints | §5.1/§7 point at 02B; §8 rules ongoing |
| `02A_ATMOSPHERE_MODEL.md` | 🟡 partial | baked mixtures, runtime composition/species queries, A-system/Janus/Mora rows | per-altitude composition/temperature profiles, TBD cells, Khepri local field, biome coupling |
| `02B_BC_SUBSYSTEM.md` | ✅ reference | working ranges = baked midpoints | formation narrative, canon lock, stability proof |
| `02C_FAR_COMPANION.md` | 🔵 design target | — (deliberately unbaked) | encounter geometry, cloud phase-space, travel benchmarks |
| `02D_BIOSPHERE_CHIRALITY.md` | 🔵 future work | — | food-refinery progression, biosafety, narrative |
| `03_INTERSTELLAR_SCOPE.md` | 🔵 design baseline | — | cloud model, phase-space, travel benchmarks, RSS egg |
| `13_THESSA_V02_DESIGN.md` | 🟡 partial reconstruction | bulk values, climate, curved arcs, branching rifts, wetlands; ecological biomes, dry regions, basin balance, bundled global/local frozen erosion and material revision (docs 49–53) | local water geometry, global local-erosion coupling, vegetation rendering, infrastructure, stellar proposal |
| `48_THESSA_RECONSTRUCTION_2026_10_01.md` | 🕰️ reconstruction checkpoint | preserved 2026-10-01/02 climate/relief/wetland audits | continuation in docs 49–53 |
| `49_THESSA_RIVERS_ECOLOGY_SETTLEMENTS.md` | 🕰️ earlier isolated checkpoint | ecological biomes, continuous ocean-distance drivers, regional river/lake descriptors, ecological cover/scatter, 150-million inhabited-region descriptors | continuation in doc 50; historical audit retained |
| `50_THESSA_MOISTURE_EROSION_BRIDGE.md` | 🕰️ isolated erosion checkpoint | causal moisture attenuation, routed annual lake balance, offline runoff/infill, source-checked frozen delta | runtime continuation in doc 51 |
| `51_THESSA_RUNTIME_GEOMORPHOLOGY.md` | 🕰️ isolated runtime/material checkpoint | bundled client/server erosion field, semantic source matching, matching globe maps, residency-edge filtering, coarse-first material loading, fixed-scale geometry/displacement appearance | continuation in doc 52 |
| `52_THESSA_LOCAL_EROSION_REFINEMENT.md` | 🟡 partial refinement checkpoint | preserved global parent plus three 256 m runoff-erosion regions, open sediment export/budget, shared final height/prefix/sample/contact paths, finite-depth fine infill and precipitation-limited snow | resolved water channels/shores/deltas, global sediment coupling, landscape/apron convergence, GPU shadow parity, vegetation/infrastructure rendering, loading and surface-to-orbit acceptance |
| `53_THESSA_SURFACE_MATERIAL_REVIEW.md` | 🟡 rejected frames; material continuation | geological rock/soil reflectance, distinct ground/canopy/reed optical proxies, physically filtered intermediate material patterns, native land roughness shading, corrected volcanic bookmark, bilinear salt-spill shorelines (no nearest-cell rectangles), coherent snow-drift thresholds, textured beaches/bare ground, rebaked v10 albedo/roughness, CPU survey-frame diagnostics | material-normal detail, cast shadows, canopy geometry, geological/transport continuity, final visual acceptance |
| `02_GAMEPLAY.md` | 🔵 baseline | §2.5 and §2.7–2.8 describe current flight/automation slices, including vehicle part controls | surface transport, editor, economy, contracts, logistics blocks, certification |

## Client, render, terrain

| Doc | Status | Implemented | Future |
| --- | --- | --- | --- |
| `04_RUNTIME_ARCHITECTURE.md` | 🟡 prototype | process split, 120 Hz authority, warp/rails, single-pass ingress dispatch, Arc-shared outbound snapshots, f64→render-local | editor/graphs, persistence, WASM, non-promises |
| `09_BEVY_VISUAL_SLICE.md` | 🟡 prototype | map/pilot/CPU terrain/water/atmosphere + configured GPU-auto CBT, optional shared-material mesh draw with indexed fallback + beauty/plume | multiplayer/auth/persistence, WASM |
| `10_PILOT_INTERFACE.md` | ✅ prototype | HUD/navball/camera, RCS/reaction-wheel/gear/parachute controls, typed part commands, telemetry/snapshot boundary | — |
| `14_VISUAL_ATMOSPHERE.md` | 🟡 partial | raster/LUT + shell clouds + low-deck cloud volume march + gas-giant bands + aurora + field-first plume volume | weather coupling, full Solari |
| `19_ALERTING_AND_FLIGHT_PHASES.md` | 🔵 design target | background only (regime/mode inputs) | `FlightPhase`, alerts, arbitration, Slices A–D |
| `21_TERRAIN_STREAMING_THROUGHPUT.md` | 🔵 baseline | invariants normative | scheduler, geomorph, UMA fast path |
| `22_RCBT_GPU_TERRAIN.md` | 🟡 baseline | §§1–18 normative, Arc-shared extraction snapshots, batched COW page streaming, stable height slots, dirty-ordinal geometry dispatch | GPU bisector pool, native Vulkan, compressed pages |
| `47_RCBT_RENDER_AUDIT_2026_10_01.md` | 🕰️ draw/failed-visual checkpoint | native mesh draw with shared classifier/material, capability fallback, world-addressed material LOD, hardware regressions, matched debug coast metrics | later residency/material work in docs 51–53; final loading/visual acceptance and persistent conforming GPU topology remain incomplete |
| `37_CBT_INTEGRATION_STATUS_2026_09_14.md` | ✅ opt-in | fallback + indexed raster + material pages | visual acceptance, numeric comparison, persistent topology |
| `38_CBT_RENDER_AUDIT_2026_09_15.md` | ✅ dated audit | defects fixed + follow-ups; later native spectral-ocean slice in doc 56 | historical gaps included bisector pool, shadow parity, FFT ocean and virtual texture; current scope in docs 39/56 |
| `38_ENGINE_PLUME_RENDERING.md` | 🟡 implemented renderer slice | backend-neutral plume core, Low impostor, Medium/High field-integrated volume ribbon and field-derived light | live compiled engine/nozzle state in client, adaptive residual integration, ray-traced lighting and higher-quality tiers |
| `39_BEVY_EXIT_AND_ENGINE_MIGRATION.md` | 🟡 isolated implementation branches | FastRT classification + bounded slab compute; owned CBT compute/indirect raster, backend-neutral f64 body/camera frames, dirty height/material publication and bounded geometry slots; Bevy-free canonical window/headless viewer with coherent ephemeris/spin/stellar lighting, shared async terrain streaming and COW sources, winit + shared-device egui HUD; shared GLB sources and explicit compatibility X-15 identity; shared GPL flight pose/cameras/input/instruments; canonical COAST local authority or explicit PRIMARY server flight via shared embedded transport, checked adoption/input construction, receive-time interpolation, separate intent/acknowledgement and failure freeze; bounded own-time fleet observer/contact inspector; body-owned retained render meshes, procedural body baking and wire-v10 descriptors with separate immutable mesh epochs; authoritative retained PRIMARY/split appearance, COM-stable GPU publication and rotating-body-frame draws; neutral fleet shells use the existing opaque pass at a bracketed PRIMARY epoch, with resolved quality/disabled policy and 64-total-vehicle admission; textured mesh ray-query reflections/shadows plus resident generated-terrain membership with dirty-only per-slot BLASes, shared material pages and whole-partition budget fallback; shared sky environment/background, GPU capture and exact shadow-depth reuse; bounded spectral near-ocean geometry/optics (doc 56) | production client replacement, remaining WorldTerrain topology/map-camera/ECS integration and CPU fallback assets, complete render designs/materials and general incremental residency, prediction/reconciliation, external TCP/reconnect and fuller resources, PBR material parity, production UI/assists/dominant-body display and remaining assets/effects, complete platform integration, exact displaced far-ocean/atmosphere/cloud RT membership, primary terrain/ocean mesh-query reflections and FastRT ray-budget/visibility policy |
| `54_BEVY_0_20_MIGRATION_PLAN.md` | 🔵 study plan | 0.19.1→0.20.0 migrated on branch (deps, extraction, WESL, buttons, wgpu-30) + 8 regression tests, gates green | merge/defer decision |
| `41_MICROSCALED_SURFACE_STORAGE.md` | 🟡 experimental | codec A–E, allocator/residency, GPU LOD/mip/aniso path, game + render-world A/B measured; compact path pre-decodes to RGBA and raw remains default | packed material-shader sampling, crack-free geometry, baked-format adoption, production height pages |
| `43_AERO_COEFFICIENT_RESIDUAL_STORAGE.md` | 🟡 experimental | scalar adaptive residual codec with force/moment error bounds, safe table/query validation, non-negative decoded drag, and direct panel sampling; 257×257 storage is 1.77× smaller; scalar sampling was 1.58× slower in the dated baseline and 1.77–2.10× slower across three later one-pass reruns; canonical tables remain the default | fused SIMD decode, channel-specific budgets, real VLM/CFD fields, production acceptance |
| `45_CAD_RCBT_GEOMETRY.md` | 🔵 design baseline | D1 STEP/BRep fixture audited; existing RCBT + mesh-shader terrain path reused architecturally | CAD importer, normalized BRep runtime, crack-free adaptive face meshing, editor integration |
| `46_AUDIO_AND_ACOUSTIC_PROPAGATION.md` | 🟡 first slice | `audio-core` path semantics, exact-vacuum gating, structural/direct paths, Mach-cone arrival; `audio-synth` live procedural engine DSP; temporary Bevy RCS/GPWS/docking/demo adapter | calibrated physical engine telemetry/spectra, procedural one-shot mechanisms, structural attenuation graph, real IVA listener, occlusion/reverb/HRTF, post-Bevy backend |
| `56_NATIVE_SPECTRAL_OCEAN.md` | 🟡 native render-only slice | independent multiband GPU FFT, simulation-time finite-depth/local-gravity waves, slope-moment filtering, bounded shared raster/atlas/mesh-RT near geometry, explicit quality/optical-only fallback, GPU oracles and scoped costs | nonlinear waves, foam/wakes, bathymetry/weather, authoritative fluid coupling, far displaced intersections, primary ocean mesh-query reflections, ray cones/environment convolution, blind visual/platform acceptance |
| `57_NATIVE_FLIGHT_DECK.md` | 🟡 native instrument slice | native-pixel GPU FastNavball, full surface heading/pitch/roll, vector cues, aircraft /FDAI-inspired spacecraft decks, explicit disabled/quality/analytic/baked policy, scoped representation comparison; transactional ray-update recovery fix | inertial/orbit/target/maneuver frames, full FDAI rate/error needles, assist/autopilot panels, small/mobile/accessibility and blind aesthetic/platform acceptance |
| `58_NATIVE_PIPELINE_REVIEW.md` | 🟡 reviewed native pipeline slice | shared terrain/opaque-mesh/sky pass, explicit AS-before-raster API, corrected timestamp scopes, reusable terrain staging/unchanged-origin skip and fixed shadow serialization; fixed-resident split/unified oracle and A/B diagnostic | supplied whole-application speedup/parity/allocation claims not accepted; no blind/platform acceptance or complete renderer replacement |
| `59_NATIVE_HDR_DISPLAY.md` | 🟡 manual native HDR slice | capability-resolved pre-exposed RGBA16Float scene, Reinhard to SDR/sRGB before egui/FastNavball, resource-free direct LDR fallback, retained resize, explicit display-transform GPU scope and applied-vs-requested telemetry | auto exposure/metering/adaptation, bloom, AgX/ACES/legacy tonemapper match, HDR monitor output, temporal AA/upscaling, blind/platform acceptance |
| `60_NATIVE_SHADOW_CASTER_REVIEW.md` | 🟡 reviewed native raster optimization | conservative per-layer terrain caster rejection, validated height/partition bound lifecycle, allocation-free draw-range coalescing, full-caster depth/image oracle and same-tree counterbalanced cost diagnostic | ocean retains full caster coverage; tighter bounds, mesh culling, representative production-frame and platform acceptance remain open; supplied game-FPS claim not accepted |
| `61_CBT_SPHERE_EXPERIMENT.md` | 🟡 phase-one topology snapshot | CPU reference reusing BisectorPool, persistent GPU OCBT-managed bisectors, conforming compatibility-chain split/diamond merge across cube faces, transactional exhaustion, pair-u32 addresses and indirect sphere-only diagnostic | single-writer limitation advanced by doc 62; production still uses regular render patches; surface/precision/consumer integration remains open |
| `62_CBT_PARALLEL_TRANSACTIONS.md` | 🟡 isolated parallel topology experiment | bounded GPU request windows, whole-footprint deterministic conflict claims, whole-chain reservation, parallel split/merge and atomic packed-rank maintenance; exact serial byte/CPU geometry oracles; measured 128K-capacity/98K-triangle fixture | canonical height/material/error/precision and native raster/shadow/ocean/RT publication remain unimplemented; Bevy parity is not an acceptance gate; no full large_cbt renderer replacement claim |

The doc 39 branch also shares the owned GPL surface-survey camera between
clients (quaternion orbit/free look/zoom/reset and native bookmark selection).
Doc 59 supersedes the older native no-HDR checkpoint: terrain, mesh/RT and sky
can now retain linear highlights in a shared float target before SDR mapping.
Automatic exposure and bloom remain unported; this is not HDR display output.
Native survey framing is now 5 km by default rather than 30 km; prior terrain
timings are not an equal-workload comparison. The celestial map camera remains
in the unported remainder above.
The owned native embedded-server transport now lives in GPL `flight-net`
(stdio handshake, receive timestamps/inbox, input mailbox and child cleanup);
the old client keeps only a Bevy resource wrapper. Native viewer snapshot
adoption and read-only fleet observations are implemented on the migration
branch, including revision-matched authoritative contact geometry and an
inspector wireframe. Bounded retained body shells now render with a neutral
finish in the existing opaque pass when their samples cover PRIMARY's render
epoch; common COM translations reuse GPU sources and interpolation histories
in their publication frame. Shape/relative-pose changes still rebuild the combined
source. Wire-v10 pose/contact descriptors reuse a bounded immutable mesh epoch,
so COM changes transmit no mesh payload. Actual active mesh-set changes still
publish a complete payload epoch, not arbitrary incremental asset deltas.
Missing/stale geometry and out-of-bracket clocks withhold draws rather
than substitute a model or extrapolate. Full authored appearance/materials
and prediction/reconciliation remain unported. PRIMARY now consumes its own
retained geometry and split ownership. Only explicit standard-authority provenance
authorizes the bundled X-15; absent/omitted/unknown/mismatched PRIMARY appearance
draws nothing. PRIMARY is independent of the fleet-quality setting. Snapshot
interpolation does not blend different PRIMARY COM revisions (fleet-only changes
retain interpolation), and both mesh paths use the current rotating body frame.
General incremental GPU residency remains absent.
Geometry receiver rejection is now transactional for both packet kinds:
failed reconstruction cannot poison descriptor/mesh revisions or the delivered
marker; corrected same-revision retries are covered by state-retention tests.
Native opaque lighting now consumes resolved material/reflection settings and
the existing shadow settings: shared GGX with linear metallic/roughness, normal
and AO maps, plus a three-source cascaded terrain/mesh shadow atlas (48 MiB at
the default 64 MiB budget). Disabled shadows allocate no atlas or shadow
pipelines. The atlas now retains depth for exactly unchanged light matrices,
terrain pages/membership/camera origin, and immutable mesh source + instance /
primitive-range bytes. Changes invalidate it; there is no quality, cadence or
geometry reduction. Forced-redraw GPU oracles compare byte-identical frames;
static-only cost evidence is in doc 39's shadow-depth-cache checkpoint.
Native scene-object reflections now come from a Thessa-owned
ray-query path (`renderer.ray_tracing` + `[raytracing]` budgets): one BLAS per
primitive over consolidated copies, TLAS instances from the existing instance
ranges, and a named reflection hook whose raster fallback is exactly the
environment term (white-mirror misses are byte-identical; dielectric/AO/roughness
misses agree within 1/255). Explicit RT requests
fail fast on a device without negotiated ray query; RT off allocates and builds
no acceleration structures. See doc 39's ray-query and textured-hit/shadow
checkpoints for oracles, telemetry and limits.
Mesh queries now also trace the resident generated terrain grid (including its
clamped sea datum), with persistent per-slot BLASes, dirty-only rebuilds and
shared raster material sampling. Whole-partition budget rejection is explicit;
no partial occluder truncation. Native window/headless mesh consumers synchronize
membership after publication/replacement. Opt-in `[ocean]` now adds the shared
multiband spectral field, filtered slope moments and bounded near-grid waves
consumed by raster, the atlas and mesh RT; whole near-partition overflow is
optical-only. See doc 56 for physical inputs, quality, bounds and scoped costs.
This is not full replacement: primary terrain/ocean mesh-query reflection receivers, atmosphere/cloud
traversal, auto exposure/bloom, mesh mip chains and complete material parity remain
absent. See doc 39's resident-terrain checkpoint for costs and ocean direction.
Hit shading now interpolates UVs/smooth normals, samples the raster material's
base colour (sRGB) and High-tier MR/normal/AO (linear), and uses the shared GGX
direct-light and tangent-frame functions. Hit roughness now controls specular
continuation as well as the stellar BRDF. Native mesh and reflected-hit stellar
visibility adds opaque mesh/resident-terrain RT shadow rays to the shared raster
atlas; primary terrain still receives atlas visibility. Shadow rays obey the resolved
shadow enable and RT distance/scene budgets, with atlas-only fallback if RT
is unavailable. Specular-off no longer implies no AS work when RT shadows
are enabled: disabling both consumers does. The original
ray-query checkpoint captures are not reflection evidence: RT on/off differences
were within the measured same-config noise. That explanation was corrected by measurement
after the slice landed — the trace gate already uses the primary *textured*
roughness (0.333 median on the canonical asset, below the 0.6 cutoff, so the
trace is eligible). Mostly dielectric primary texels and the then-black
environment were consistent with the tiny delta; changed pixels were not
individually attributed. The native viewer now consumes resolved atmosphere
enable/quality/steps/density/multi-star/airglow settings to evaluate the shared physical
single-scattering sky into a direction grid: sky background, mesh raster reflections and RT
misses sample it; terrain/mesh diffuse ambient comes from its local-radial
hemisphere integral. Unchanged fields upload once, not every frame; disabling
releases grid storage and the background consumer. The background shares the
scene's GPU publication with mesh/RT consumers (one upload per revision), draws
at far depth in the existing terrain pass and does not overwrite foreground
depth. Spectral ocean reflection now samples that same directional field.
No sun disks or aerial perspective yet. Doc 59 adds manual pre-exposed HDR
storage and Reinhard to SDR; it does not implement auto exposure or bloom.
See doc 39's environment-radiance and sky/perf checkpoints for error/cost
evidence and limitations. Shadows only
see retained scene geometry;
off-cover terrain residency and production shadow-quality acceptance are pending.
The material `SlotCache` now uses an ID index and one deterministic free/LRU
candidate pass, checked against the original policy over mixed replays.
Its 64/256/1024/4096-page warm/cold/churn benchmark is a CPU residency
microbenchmark, not a renderer-FPS or GPU-upload win; see doc 22. Headless native
`--perf-capture PATH.json` now writes `thessa-perf` JSON/CSV with host-frame /
streaming scopes and opt-in hardware GPU command intervals for shadows,
AS builds, shared scene classification/raster, HDR display transform and UI
(`native-display-v3`). Unsupported timestamps are explicitly unavailable;
normal devices allocate no profiler. This is serial offscreen instrumentation,
not asynchronous window profiling, isolated sky/AS/trace timings or complete
simulation/counter coverage. StagingBelt A/B and submission batching remain open.
The shared link now caps complete pending command packets at the protocol's
64-command limit (all variants and state votes), and worker pipe/encode
failures close the input side visibly. Stalled-peer heartbeat/reconnect and
external native networking remain future work. Wire v10 separates bounded
on-change descriptors and active mesh payloads; PRIMARY and fleet snapshots name
the matching descriptor revision. v7/v8/v9 peers are rejected. This is
not a full render-design/model registry or complete resource replica.
Vehicle definitions now include optional MIT `RenderGeometry`:
f64 rigid part poses, indexed local meshes shared by Arc, and validated body
ownership. Existing cluster reconstruction partitions/remaps the parts and moves
their origins with the physical COM; mass, contact and forces do not consume
these meshes. The baker retains procedural body shells using the existing
fuselage tessellator (configurable `[body_render]`); source assembly transforms
and final COM shifts are preserved. Surfaces, authored appearance materials and
full vehicle appearance remain later slices. Native fleet meshes consume
resolved `renderer.fleet_meshes` / `fleet_quality` settings with explicit disabled
and Low/Medium/High angular-size culling levels. The legacy host still ignores
fleet observations (but is rebuilt for v10).

## Autopilot and roadmap

| Doc | Status | Implemented | Future |
| --- | --- | --- | --- |
| `05_ROADMAP.md` | 🟡 order | M0–M1/M3–M4/M6 slices; stdlib minimum; M5 search prototype | staging, editor, depots, canon ephemeris, UX |
| `07_AUTOPILOT.md` | 🟡 slice | IR, wait-bounded cycles, per-wait generations, 4 blocks, B-plane + variational, L0–L2 replays | full stdlib/editor, staging topology, L3–L4, powered rails |
| `18-control-guidance-autopilot.md` | 🟡 baseline | authority/laws/allocator/graph/QuickJS/waits/server | allocator generalization, wire replacement, Phases 4–5 |
| `12_X15_ASSET_PROVENANCE.md` | 📦 provenance | record current | — |
| `15_PERFORMANCE_MONITORING.md` | ✅ active | capture/scopes/overlay/counters, native headless GPU intervals | window/Bevy GPU timestamps, finer GPU scopes, backend memory, dev levels |

## Process and reference

| Doc | Status | Notes |
| --- | --- | --- |
| `06_OPEN_QUESTIONS.md` | 🟡 living index | Rapier decided; stdlib/ownership narrowed; J2/C22 landed; rest open |
| `35_BLAZE_AUDIT_2026_09_12.md` | 🕰️ record | scoped to base `45e0b79`; see 36 for current status |
| `36_DOCUMENTATION_AUDIT_2026_09_14.md` | 🕰️ record | snapshot `e1454ac`; superseded by this file for post-merge state |
| `44_KSA_TECHNICAL_COMPARISON_2026_09_20.md` | 🕰️ external audit | KSA comparison: warp, rings, glints, plume, instruments, body orientation |
| `details/01_DOCKING_PORTS.md` | 🟡 D1 contact-scene slice | persisted D1 docking protocol, fixed-joint install/removal, pressure-equalization gate, solver joint-load readback, authored port inventory, and server fleet ownership with dock sessions, vacuum-gated joints, and fleet snapshots | calibrated capture dynamics and class-dependent ratings, jointed-stack control, client fleet render, and complete part-level structural integration |
| `REFERENCES.md` | 📚 reference | UX-only refs; Nyx/ANISE isolation; no copied code |
| `adr/0007–0011` | ✅ accepted | licensing, render boundary, autopilot graphs, aero boundary, RCBT boundary |
