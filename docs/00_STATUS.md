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
| `53_THESSA_SURFACE_MATERIAL_REVIEW.md` | 🟡 rejected frames; material continuation | geological rock/soil reflectance, distinct ground/canopy/reed optical proxies, physically filtered intermediate material patterns, native primary material normals with bounded material-state response/roughness plus star/sky shading and shared shadow atlas (docs 60/63/69), corrected volcanic bookmark, bilinear salt-spill shorelines (no nearest-cell rectangles), coherent snow-drift thresholds, textured beaches/bare ground, rebaked v10 albedo/roughness, CPU survey-frame diagnostics | calibrated substrate-specific relief/strata, shadow error/convergence acceptance, canopy geometry, geological/transport continuity, final visual acceptance |
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
| `38_ENGINE_PLUME_RENDERING.md` | 🟡 implemented renderer slice | backend-neutral plume core; native resolved Low impostor and Medium/High actual-station, scene-depth-clipped consumers with GPU oracles/counters; legacy field-derived nozzle light | native nozzle-light consumer, live compiled nozzle extraction, general animated/full-view error envelope, adaptive residuals and ray-traced lighting |
| `39_BEVY_EXIT_AND_ENGINE_MIGRATION.md` | 🟡 native migration slice integrated into main | FastRT classification + bounded slab compute; owned CBT compute/indirect raster, backend-neutral f64 body/camera frames, dirty height/material publication and bounded geometry slots; Bevy-free canonical window/headless viewer with coherent ephemeris/spin/stellar lighting, shared async terrain streaming and COW sources, winit + shared-device egui HUD; shared GLB sources and explicit compatibility X-15 identity; shared GPL flight pose/cameras/input/instruments with dominant-body readouts (doc 57); canonical COAST local authority or explicit PRIMARY server flight via shared embedded transport, checked adoption/input construction, receive-time interpolation, separate intent/acknowledgement and failure freeze; bounded own-time fleet observer/contact inspector; body-owned retained render meshes, procedural body baking and wire-v10 descriptors with separate immutable mesh epochs; authoritative retained PRIMARY/split appearance, COM-stable GPU publication and rotating-body-frame draws; neutral fleet shells use the existing opaque pass at a bracketed PRIMARY epoch, with resolved quality/disabled policy and 64-total-vehicle admission; textured mesh ray-query reflections/shadows plus resident generated-terrain membership with dirty-only per-slot BLASes, FastRT representation classification, pixel-cone texture mips, shared material pages and live-pairing whole-partition TLAS budget fallback; shared sky environment/background, GPU capture and exact shadow-depth reuse; bounded spectral near-ocean geometry/optics (doc 56); native system map/navigation (doc 67) | production client replacement, remaining client/game integration and CPU fallback assets, complete render designs/materials and general incremental residency, prediction/reconciliation, external TCP/reconnect and fuller resources, PBR material parity, production UI/assists/maneuvers and remaining assets/effects, complete platform integration, exact displaced far-ocean/atmosphere/cloud RT membership, primary terrain/ocean mesh-query reflections and FastRT ray-budget/visibility policy |
| `54_BEVY_0_20_MIGRATION_PLAN.md` | 🔵 study plan | 0.19.1→0.20.0 migrated on branch (deps, extraction, WESL, buttons, wgpu-30) + 8 regression tests, gates green | merge/defer decision |
| `41_MICROSCALED_SURFACE_STORAGE.md` | 🟡 experimental | codec A–E, allocator/residency, GPU LOD/mip/aniso path, game + render-world A/B measured; retained four-plane material codec and 25 hardware shader/material tests no longer need Bevy; native explicitly resolves compact requests to raw; legacy compact path pre-decodes to RGBA and raw remains default | native compact residency, packed material-shader sampling, crack-free geometry, baked-format adoption, production height pages |
| `43_AERO_COEFFICIENT_RESIDUAL_STORAGE.md` | 🟡 experimental | scalar adaptive residual codec with force/moment error bounds, safe table/query validation, non-negative decoded drag, and direct panel sampling; 257×257 storage is 1.77× smaller; scalar sampling was 1.58× slower in the dated baseline and 1.77–2.10× slower across three later one-pass reruns; canonical tables remain the default | fused SIMD decode, channel-specific budgets, real VLM/CFD fields, production acceptance |
| `45_CAD_RCBT_GEOMETRY.md` | 🔵 design baseline | D1 STEP/BRep fixture audited; existing RCBT + mesh-shader terrain path reused architecturally | CAD importer, normalized BRep runtime, crack-free adaptive face meshing, editor integration |
| `46_AUDIO_AND_ACOUSTIC_PROPAGATION.md` | 🟡 first slice | `audio-core` path semantics, exact-vacuum gating, structural/direct paths, Mach-cone arrival; `audio-synth` live procedural engine DSP; native RCS/GPWS/boom/docking cues and optional device adapter | calibrated engine telemetry/spectra and impact audio, procedural one-shot mechanisms, structural attenuation graph, real IVA listener, occlusion/reverb/HRTF |
| `56_NATIVE_SPECTRAL_OCEAN.md` | 🟡 native render-only slice | independent multiband GPU FFT, simulation-time finite-depth/local-gravity waves, slope-moment filtering, bounded shared raster/atlas/mesh-RT near geometry, explicit quality/optical-only fallback, GPU oracles and scoped costs | nonlinear waves, foam/wakes, bathymetry/weather, authoritative fluid coupling, far displaced intersections, primary ocean mesh-query reflections, ray cones/environment convolution, blind visual/platform acceptance |
| `57_NATIVE_FLIGHT_DECK.md` | 🟡 native instrument/readout slice | native-pixel FastNavball, local heading/pitch/roll, dominant-body shared SI/f64 readouts, session-only speed/cue and Datum/AGL selectors, air/load/apsis/wheel/canopy details with explicit replica provenance, aircraft/FDAI layouts and disabled/quality policy; transactional ray-update recovery | nonlocal attitude frames, target/maneuver inputs, full FDAI rate/error needles, assists, small/mobile/accessibility and blind aesthetic/platform acceptance |
| `58_NATIVE_PIPELINE_REVIEW.md` | 🟡 reviewed native pipeline slice | shared terrain/opaque-mesh/sky pass, explicit AS-before-raster API, corrected timestamp scopes, reusable terrain staging/unchanged-origin skip and fixed shadow serialization; fixed-resident split/unified oracle and A/B diagnostic | supplied whole-application speedup/parity/allocation claims not accepted; no blind/platform acceptance or complete renderer replacement |
| `59_NATIVE_HDR_DISPLAY.md` | 🟡 manual native HDR slice | capability-resolved pre-exposed RGBA16Float scene, Reinhard to SDR/sRGB before egui/FastNavball, resource-free direct LDR fallback, retained resize, explicit display-transform GPU scope and applied-vs-requested telemetry | auto exposure/metering/adaptation, bloom, AgX/ACES/legacy tonemapper match, HDR monitor output, temporal AA/upscaling, blind/platform acceptance |
| `60_NATIVE_SHADOW_CASTER_REVIEW.md` | 🟡 reviewed native raster optimization | conservative per-layer terrain caster rejection, validated height/partition bound lifecycle, allocation-free draw-range coalescing, full-caster depth/image oracle and same-tree counterbalanced cost diagnostic; 2026-10-27 mesh light-volume rejection and expanded-work counters with CPU/software-Vulkan checks | ocean retains full caster coverage; representative production-frame and hardware/platform acceptance of the mesh extension remain open; supplied game-FPS claim not accepted |
| `61_CBT_SPHERE_EXPERIMENT.md` | 🟡 phase-one topology snapshot | CPU reference reusing BisectorPool, persistent GPU OCBT-managed bisectors, conforming compatibility-chain split/diamond merge across cube faces, transactional exhaustion, pair-u32 addresses and indirect sphere-only diagnostic | single-writer limitation advanced by doc 62; canonical surface/native production selection advanced by doc 63; regular patches remain explicit/unsupported fallback |
| `62_CBT_PARALLEL_TRANSACTIONS.md` | 🟡 persistent topology pipeline | bounded GPU request windows, whole-footprint deterministic conflict claims, whole-chain reservation, parallel split/merge and atomic packed-rank maintenance; exact serial byte/CPU geometry oracles; measured 128K-capacity/98K-triangle fixture; external-area actionable request selection reuses the same planner | doc 63 adds canonical sampling, emitted precision, relief-bound classification and production selection; no full large_cbt renderer replacement or equal-error/temporal acceptance claim |
| `63_CBT_CANONICAL_SURFACE.md` | 🟡 configured native production-frame consumer | configured native persistent CBT with GPU-active indirect raster/shadow draws, sparse overlapping height data, exact fine-material charts, canonical addresses through heap depth 58, borrowed height/material residency and f64 precision charts; seam/residency/metric-shadow/Fourier oracles; shared-epoch RayScene hits and whole-pool fallback; resolved disabled/Low/Medium/High native topology mode now selects shared primary raster, atlas, ocean and RT geometry with emitted-revision shadow caching; actual camera-relative area/shared-source midpoint error, parent merge validation, primary visibility and actionable parallel requests with independent f64/CPU replay oracles | per-tile relief/error and convergence, optimized dynamic-ocean RT/dirty-chunk reuse, primary terrain scene-object RT reflections and equal-error/temporal/platform acceptance; regular patches remain explicit/unsupported fallback; full-client Bevy removal remains open |
| `64_NATIVE_OPTIMIZATION_REVIEW.md` | 🟡 historical rejected proposal; corrective implementation in doc 63 | supplied medians/tails recomputed; GPU failures retained as audit evidence; new regression verifies zero-request quiescence, LOD wake and mesh shadow/reference equality | original performance claims remain unaccepted; matched-work measurements and wider acceptance stay open |
| `65_NATIVE_SKY_AUDIT.md` | 🟡 audited native sky consumers | shared aurora altitude/axis and incremental optical transport; native Low/High emission field with simulation-time refresh; flux-normalized background stars/source discs; depth-clipped clouds with shared weather/geometry/phase and terrain/mesh/ocean shadows plus reflected-ray transport; primary aurora with finite-depth clipping and ordered cloud extinction; separate clear primary/total indirect publications; independent disable and numerical GPU oracles; actual-ephemeris far-visible B and A-hidden/B-visible regular/conforming terrain regressions | secondary-origin clear-sky/auroral parallax, partial stellar discs/eclipses, magnetic/weather data, richer curtains, cloud diffuse coupling/multiple scattering, RT-light proxies and temporal/performance/visual/platform acceptance |
| `67_NATIVE_SYSTEM_MAP.md` | 🟡 native map/navigation slice | six baked scopes, f64 framing/orbit/pan/zoom, body/PRIMARY selection/follow, cached Kepler curves, physics-JIT coast-cache consumer sharing execution guards and bounded interval, dominant-body Ap/Pe/escape/local-wake readouts, exclusive input ownership, shared-device egui and headless capture | powered/live executable JIT futures (never substituted with conditional coasts), remote JIT/next-wake transport, richer celestial appearance and independent usability/platform acceptance |
| `68_NATIVE_NIGHT_LIGHTS.md` | 🟡 native primary-terrain optical slice | direct retained inhabited-land mask producer, resolved Low/Medium/High budgets, cached startup-only bake/upload, regular/conforming shared primary emission and zero-work disable; CPU parity and hardware numerical/source tests | calibrated radiance/activity, multi-source illumination/eclipses, secondary-ray emission and whole-frame visual/temporal acceptance |
| `69_NATIVE_TERRAIN_MATERIAL_LIGHTING.md` | 🟡 primary terrain normal/sky-reflection slice | optional zero-exact RG8_SNORM material normals through shared native residency, WORLD22 canonical base-height/full-optical normal composition across levels and bounded High L24 optical pages and quality-gated RGBA atlas, SH9 directional diffuse and GGX footprint filtering, metric-correct cube-sphere gradients and slope blending, bounded substrate/exposure/cover response and small geology-linked roughness priors, resolved disable/+2/+1/+0 mip tiers and raster Lambert oracle; shared sky/cloud rough GGX with resolved 4/8/16 clear directions or 1/2/4 cloud columns; regular/conforming reflection oracles and scoped GPU diagnostic; WORLD108 visible-first unresolved optical splits retain the complete wider prefetch cover and unchanged budgets; WORLD115/116 separates intrinsic substrate reflectance from canopy illumination and retains rough damp soil alongside separate water pools | calibrated natural substrate relief/strata, vegetation, general HDR/cloud quadrature error/convergence, primary scene-object reflections and close-up/visual/platform acceptance |
| `70_WORLD_SCATTER_IDENTITY.md` | 🟡 world scatter and canonical CPU sampling | fixed world-local IDs, cross-LOD/query unions, density subsets and explicit budgets; existing offline plant consumer plus a borrowed attached-field plant callback; bounded native survey plant consumer in doc 75 | resident-mesh root agreement, local water geometry, instance streaming, production foliage/rocks and collision ownership |
| `71_NATIVE_SURFACE_ACCEPTANCE.md` | 🟡 reproducible surface camera | fixed numeric targets, exact radial 50 m/2 m eye clearance, source/camera metadata and unchanged default survey/launch ownership | resident-mesh clearance/convergence, complete 2 km visual coverage and blind visual acceptance |
| `72_WORLD_SURFACE_BASELINE.md` | ✅ scoped numeric audit site | actual canonical zero crossing, reproducible 2×2 km ENU patch, frozen-source identity, land target and numeric regression | universal terrain accuracy or visual/vegetation acceptance not claimed |
| `73_COASTAL_SURFACE_MATERIALS.md` | 🟡 coastal optical eligibility correction | existing coarse maritime context, slope/exposure/cover/salinity gate the low-altitude sediment and wet rim; weathered local geology replaces universal yellow sand; failing-before inland/cliff regressions | precise shoreline/sediment availability, fine relief, actual close-surface and independent visual acceptance |
| `74_COASTAL_CLOUD_RECEIVERS.md` | 🟡 primary terrain receiver correction | datum-support planet predicate avoids false interior rejection on chord-meshed/reconstructed water; existing cloud integration/disabled policy and generic strict volume/mesh helpers retained | secondary terrain receivers, near-limb/reconstruction/convergence bounds, wave self-occlusion and wider surface acceptance |
| `75_NATIVE_SURFACE_PLANTS.md` | 🟡 bounded native survey prototype | opt-in density tiers, up to nine L14 canonical plant windows plus optional 49-window L18/L24 understory and scale-aware full/reduced pinnae, one shared background job for same-site near changes with stale-result rejection and compatible GPU-source reuse, WORLD11a slender pinnae, scale-aware geometry and dry outer ground fronds, lower asymmetric forks, roughness-filtered mesh environment, CC0 bark/leaf maps, explicit dry-frond materials, geometry-baked AO and thin diffuse scattering, world-addressed understory colonies with bounded cohort scales, WORLD21 fuller crown forms and reproducible authored bark import, shared opaque/shadow path with physical stellar ray-line preservation and bounded actual-producer visibility oracles, RT preflight/fallback, zero-work disable and authored fern/reed/forked-crown geometry; isolated CPU-Vulkan specimens with actual native shadows/materials, broader 6/8/12-pair ground organs and moment-preserving divided coarse wings (2–12 divisions/side), unchanged numerical guards and 47/48 MB source caps; physical GPU replay still blocked | actual resident-mesh root agreement, finite-bias/edge visibility, botanical realism, GPU cost and wider stable performance acceptance, planetary instance streaming and collision ownership |
| `77_NATIVE_SURFACE_SHOWCASE.md` | 🟡 surface-first work in progress | actual GPU material-level telemetry; protected coarse support plus caller-priority refinement replaces detail-starving global level admission; separate stem/foliage reflectance; regular-patch split-coordinate material sampling and local derivative footprint; default-off CC0 ground-swatch experiment with L20/L21/L24 material caps (WORLD22), WORLD12 continuous pre-mip mineral/humus transfer, explicit 60 mm optical relief, shared moisture/pigment roughness and view-cone material priority; WORLD14 reviewed authored soil/foliage material priors, documented full-detail distance override and shared configurable spherical local weather across primary ground/water/meshes; WORLD15 dual CC0 ground scans, existing 0.5–8 m grain promoted into render geometry, WORLD22 canonical base-height/full-optical normal composition across levels and bounded High L24 optical pages, WORLD23 variable wetness, render-height moss cushions/shoots and default-off sampled-basin puddles with shared dielectric materials, bounded understory cohort sizes and reproducible bark import, rooted single/forked plant forms, 4–46 cm pebbles, SH9 diffuse and footprint roughness filtering; WORLD40 audits Gemini WORLD24–39 asymmetric crowns, clearings, detritus and coastal profile (EV12.10, local RT, 32k requested instance/64 MiB planning limits), fixes floating fork attachment and horizon-dependent AO, restores configured ray bias after a reproduced distance-heuristic leak, shares FastRT representation policy across mesh/terrain and admits whole terrain partitions against live TLAS pairings, retains Medium/High pixel-cone hit mips and reports the actual single-mirror-ray estimator; no computed ground bounce, stochastic GI or reference parity; WORLD41 removes redundant scan prefilter blur, preserves litter scan contrast and adds folded moss/ground-leaf geometry with bounded Medium/High counts, 37 aggregate/plant ranges and the unchanged 7 MB prototype budget; WORLD42 restores the native 1K soil source (3.08 mm samples), dimension-aware source mips, matching nonlinear-transfer means and regular/conforming L24 material-motion checks, with unchanged GPU page budgets and +20 MiB CPU source mips; WORLD43 fixes below-frame pages starving visible soil through actual camera-relative material clipping in native/Bevy, preserving geometry prefetch and page/cache limits; WORLD13 configured persistent CBT with sampled source/chord error and parent merge validation, sparse height residency independent of geometric LOD, exact bisector material charts, stochastic mapping, render-only soil-height pages and bounded L24 pebble meshes with shared jobs/instancing/shadows and Low optical fallback; rejected colour-noise prototype recorded, not shipped; WORLD55–79 executes 25 autonomous iterations across 5 blocks: Block 1 tree caudex bark, basal buttress root flares, polymorphic trunk girth, basal AO and branch fork thickening; Block 2 apical crozier fiddleheads, pinna edge crenation, power-law rachis taper, branch fork crotch attenuation and cuticle edge transmission; Block 3 understory multi-tier rosettes, ground-contact senescent drape, polymorphic chloroplast reflectance, reed droop/wind tilt and canopy clearing glades; Block 4 basalt shingle reflectance, detritus/humus scattering, wet soil specular/puddle rim darkening, scale-aware near-ground filtering and deep root cavity contact AO; Block 5 maritime overcast sky chromaticity, sea-level coastal fog/marine spray aerosol, distant ridge depth cueing, authored EV12.22 exposure and uncontrolled image metrics at that dated checkpoint; no calibrated radiometric or general driver-stability acceptance; WORLD84 retains user crown/rosette geometry and adds deterministic organ pigment cohorts, padded shared foliage atlases (+540,672 B source images), asymmetric pinnae and LOD-stable cohort metadata under unchanged 37-range/7 MB limits; matched raster comparison after a baseline RT device loss, no reference or RT-stability acceptance; WORLD85+ coastal composition: incoming WORLD121–135 soil, density, pigment and optics edits are reviewed and captured in WORLD136; WORLD139 single-blade geometry was rejected. WORLD142 reconstructs compound fronds with attached pinnules and retains the numeric geometry guards, but its first raster capture caused a confirmed GPU reset and Niri session loss. A CPU-only reconstruction measures 148.57 million submitted mesh triangles per pass; automatic desktop GPU runs are blocked. Aggregate source admission is 48 MB and prototype validation 262,144 vertices; these limits do not bound instanced GPU work. The scene uses 24 cloud march steps. Subsequent isolated CPU-Vulkan specimens improve ground organ width and replace flat coarse wings with attached divisions while retaining numeric and source-byte limits; this is not physical-GPU or reference acceptance. Moss retains the independently scanned forest substrate and connected shoots. WORLD120 raster evidence passed 455 batch cases, but its bounded LocalRT session failed with device loss; no general RT stability, reference or temporal acceptance. | canopy/floor representation hierarchy, convincing ground/foliage/water composition, moving-camera and blind acceptance, bounded planet-wide vegetation streaming |
| `78_NATIVE_MAIN_INTEGRATION.md` | 🟡 default-off native capability integrated into main | fast-forward promotion of tested implementation plus evidence as `99a1462`; 13 local gates, nine hashed captures and three unreviewed blind pairs; defaults and old host retained | unresolved visual/transport/streaming/temporal/host obligations and full replacement |
| `79_RTXPT_1_9_ENGINEERING_SPEC.md` | 🟡 technical specification & engineering progression | source audit of Thessa RT/FastRT vs NVIDIA RTXPT 1.9; incremental hybrid raster/RT progression; 12 mandatory non-regression invariants; 8 work packages (WP0..WP7) with decision gates G0..G6; WP1 (error-scaled ray bounds), WP2 (ray-cone footprint LOD), and WP3 (RT admission scalability / reflection samples) implemented and verified on branch | WP4..WP7 conditional progression, benchmarks, empirical validation |
| `76_TEMPORAL_RECONSTRUCTION.md` | 🟡 temporal foundation + opt-in native history lifecycle (T4c partial) | default-off TAA in headless/window loops, shared CPU/GPU running-mean correction, pre-overlay ping-pong colour and previous depth, submitted-only commits, publication/view-mode/exposure/projection/resize resets, explicit quality and capability fallback; separate motion/resolve GPU scopes | composed sampled/resolved-depth policy, complete arbitrary-cut epochs, metric disocclusion envelope, object/deformation motion and broader static eligibility, moving-scene ghosting/visual acceptance, TAAU and SDK/FG adapters; not production TAA acceptance |

The [WORLD00 coastal baseline](72_WORLD_SURFACE_BASELINE.md) now identifies an
actual canonical zero crossing and a 2×2 km sampled land/water neighborhood,
with a fixed nearby land target and numeric regression. It is a CPU audit site,
not close-surface visual acceptance, finer erosion or production vegetation.

The native survey supports fixed numeric targets and explicit 50 m/2 m radial
eye clearance with final-camera/source capture metadata; default survey
placement and launch authority are unchanged. See
[doc 71](71_NATIVE_SURFACE_ACCEPTANCE.md); close-up captures are not themselves
terrain convergence or visual acceptance.

The integrated 2026-10-21 surface foundation passed 14 local command gates on
750 unchanged source/configuration/asset inputs: 2,003 workspace lib/bin tests,
98 integration tests, the WORLD00 numeric example regression, 88 native
feature-off tests, 43 native ignored hardware/CLI checks, and both 8-test
terrain/terrain-environment hardware targets. Six real silent frames have
paired exact camera metadata and drained final height jobs with zero missing
material pages. Three blind pairs are prepared but unreviewed after the agent
usage limit; water stippling, flat cover, absent vegetation geometry and broader
surface acceptance remain open. These are local checks, not remote CI or a
speedup/parity claim. See
[`artifacts/world-surface-foundation-2026-10-21.json`](artifacts/world-surface-foundation-2026-10-21.json).

The subsequent [primary cloud receiver correction](74_COASTAL_CLOUD_RECEIVERS.md)
isolates the dominant water stippling to false planetary interior rejection,
not relief/material noise. It restores datum-supported surface visibility while
keeping cloud integration and strict generic volume/mesh semantics. Three new
blind pairs favor the correction; the older material-only review preferred the
old sandy overview with two ties. Neither result establishes natural ground
detail, vegetation, shadow convergence or complete surface acceptance. The
first corrected timing observations were higher; no stable cost or speedup
claim is made.

WORLD-02 passed 12 local gates on 752 unchanged inputs: 2,003 workspace lib/bin
tests, 98 integration tests, 79 ignored renderer hardware tests and 43 ignored
native hardware/CLI tests, plus targeted cloud regression, format, all-feature
Clippy, architecture, WORLD00 numeric regression and release build. The rebuilt
binary matches the captured correction. Exact scope and evidence:
[`artifacts/coastal-cloud-receivers-2026-10-21.json`](artifacts/coastal-cloud-receivers-2026-10-21.json).

The [world scatter foundation](70_WORLD_SCATTER_IDENTITY.md) replaces tile/density
seeding with fixed world-local IDs, exact partition unions and nested density
subsets, explicit resource/density errors, and the existing offline plant consumer.
Its bounded native survey-only prototype consumer is tracked separately in
doc 75; production foliage and full runtime-terrain placement remain incomplete.

WORLD-03 adds a plant-only canonical surface callback for the existing scatter
kernel, reusing the attached field's final height, metric geometry, ecology and
coarse hydrology descriptors. This closes the CPU sampler handoff, not native
plant publication, contact/obstacle ownership or finer shoreline geometry. The
existing offline frozen-DEM consumer deliberately keeps its historical behavior;
no erosion artifact, world height or historical plant ID is rewritten. See
[doc 70](70_WORLD_SCATTER_IDENTITY.md) for contracts, evidence and limitations.

WORLD04 connects those canonical descriptors to the existing native opaque mesh,
shadow atlas and environment path for one selected L14 survey window. The shared
`[surface_plants]` settings default off; quality thins nested render subsets,
not authoritative obstacles. This is not planet-wide streaming, realistic
botanical assets, collision ownership or final surface acceptance. See
[doc 75](75_NATIVE_SURFACE_PLANTS.md).

WORLD05 replaces the schematic prototypes with grounded arching fern/reeds and
tapered lycopsid trunks with drooping frond whorls, plus distinct per-kind
albedo. Placement IDs, heights, scales, window, density tiers and the no-collision
boundary are unchanged. A matched old/new capture and independent blind review
preferred the new geometry at 50 m and 2 m and tied at overview distance; thin
fronds still alias, crown undersides remain dark, and the trunk is faceted. No
stable performance or botanical-realism acceptance is claimed. Evidence:
[`artifacts/native-plant-prototypes-2026-10-21.json`](artifacts/native-plant-prototypes-2026-10-21.json).

The 2026-10-21 WORLD04 capture stage passed 14 local command gates on 755
unchanged test inputs, with the captured executable reproduced byte-for-byte by
the committed source, plus one independent blind review. The legacy Bevy
`--ignored` command matched zero tests and is not counted as hardware coverage;
actual GPU suites are the renderer and native runs. Evidence:
[`artifacts/native-surface-plants-2026-10-21.json`](artifacts/native-surface-plants-2026-10-21.json).
No remote CI or stable performance acceptance is claimed.

WORLD06f refines the same shared plant source with actual pinna gaps and forked
leafy terminals, smooth stem normals and bounded 8,192-vertex validation. Native
prototype payload is now 462,672 B, not the WORLD05 68,256 B; no scatter ID,
canonical root, material, draw grouping, density tier or physics is changed.
The corresponding numerical/adapter checks are not botanical, temporal or
independent visual acceptance. Current details are in doc 75 and doc 77.

WORLD06g is a candidate continuation of the default-off ground experiment:
local canonical elevation now informs material demand distance, and surface
quality resolves 1/4/16x anisotropic filtering with an explicit isotropic
capability fallback. Regular-patch material LOD uses the bounded elliptical
footprint; geometry, physics, source material and cache budgets are unchanged.
Synthetic CPU/GPU oracles and the full local gates pass (2,041 lib/bin, 103
integration and 83 renderer hardware tests). The standing capture now retains
eight L22 material pages rather than stopping at L21; its local reference is
5.008 m above datum. One fresh standing pair measured 116.204/111.124 ms GPU
median, not an accepted speedup. This does not establish convincing ground, seam-free
anisotropic page filtering, moving-camera acceptance or planetary streaming.
See [doc 69](69_NATIVE_TERRAIN_MATERIAL_LIGHTING.md) and
[doc 77](77_NATIVE_SURFACE_SHOWCASE.md); commands, hashes and limitations:
[`artifacts/native-ground-filtering-2026-10-23.json`](artifacts/native-ground-filtering-2026-10-23.json).

WORLD06h is a candidate shared shadow correction and precision redistribution:
meshes now use the true raster triangle receiver plane rather than smooth/bump
normals, matching terrain; optional first cascade radius keeps far cover,
count/allocation budget and legacy spacing when omitted. The default capture
budget otherwise spends 1024 texels across a 754 m first radius. The new GPU
flat-plane oracle reproduces an 18-code false shadow before correction and passes
the unchanged <=1-code bound afterward. Targeted CPU and shared-shadow GPU
oracles and full local gates pass (2,043 lib/bin, 105 integration, 83 ignored
renderer hardware tests, focused plant GPU diagnostics, feature-off and release
build). Canonical standing/close GPU medians are 106.669/114.963 ms and
127.386/123.936 ms before/near-after: not stable performance evidence.
Thin foliage, some trunk artifacts and grid-like shadows remain; material-normal
grazing bands are separate and unresolved. No independent visual acceptance.
See [doc 69](69_NATIVE_TERRAIN_MATERIAL_LIGHTING.md) and
[doc 77](77_NATIVE_SURFACE_SHOWCASE.md); exact evidence:
[`artifacts/native-near-shadow-receivers-2026-10-23.json`](artifacts/native-near-shadow-receivers-2026-10-23.json).

WORLD06i is a candidate shared-GGX hemisphere correction: negative shading-normal
view cosines no longer masquerade as a visible grazing lobe or normalize a zero
stretched view. A new 45-case hardware oracle rejects 39 old nonzero/non-finite
responses and passes after correction; the existing exact/positive grazing
oracle also passes without changed tolerances. Full local gates pass (2,043
lib/bin, 105 integration and 84 renderer hardware tests, feature-off and release
rebuild matching the captured executable). Canonical frames confirm the visible
ground bands remain; no normal-map visual or stable performance acceptance.
Evidence:
[`artifacts/native-ggx-hemisphere-2026-10-23.json`](artifacts/native-ggx-hemisphere-2026-10-23.json).

WORLD06j adds an anisotropic-environment GPU/f64 quadrature oracle and narrows the
remaining ground-band diagnosis to the direct stellar lobe and bump-normal view
support. A temporary 256-sample capture did not remove the bands; source-isolation,
false-colour support and direct-only two-facet prototype captures are preserved.
All temporary render changes were removed: the incomplete visibility prototype
is not a production feature. Full local gates pass (2,043 lib/bin, 105 integration,
85 renderer hardware tests, feature-off, Clippy and release build); the release
remains byte-identical to WORLD06i. No new visual or performance acceptance is
claimed. Evidence:
[`artifacts/native-direct-lobe-diagnosis-2026-10-23.json`](artifacts/native-direct-lobe-diagnosis-2026-10-23.json).

WORLD06k is a candidate zero-exact material-normal representation correction:
the shared runtime producer, CPU slope mips, GPU texture and raster sampler now
use RG8_SNORM rather than unsigned RG8. A neutral plane matches no plane exactly
in all 24 regular/conforming, quality and view fixtures; the former encoding
failed every case, with up to 32 linear output codes of error. Source component
quantization is bounded by 0.5/127 plus f32 decode roundoff. Memory, settings,
quality budgets, canonical geometry and material colour/roughness are unchanged.
Matched standing/close/overview captures are complete; the characteristic bands
remain, with mean displayed-code changes below 0.05 per RGB channel. Full local
gates pass: 2,044 lib/bin, 105 integration, 86 renderer hardware tests, the signed
diagnostic-PNG preview test, formatting, Clippy, architecture, feature-off and
release rebuild matching the final captured executable. No ground-band,
performance or aesthetic acceptance follows from this numerical regression
(see docs 69/77).
Evidence:
[`artifacts/native-signed-material-normals-2026-10-23.json`](artifacts/native-signed-material-normals-2026-10-23.json).

WORLD06l adds three CPU projected-geometry reference tests: exact facet
visibility versus 12,800 ray/segment cases, macro projected-area coverage,
reciprocity/flat-Lambert limits and passive single-bounce energy with a pinned
quadrature-convergence envelope. A temporary actual-producer probe finds legacy
grain RMS slopes 16.51–170.37 times the weighted imported contribution on 23
selected canonical pages; this is not actual raster-footprint/visibility proof.
All instrumentation is removed and production transport is unchanged. Full local
gates pass (2,044 lib/bin, 108 integration and 86 renderer hardware tests, Clippy,
architecture, feature-off and release identity); no normal-transport correction
or visual acceptance is claimed. See docs 69/77 and
[`artifacts/native-normal-geometry-reference-2026-10-23.json`](artifacts/native-normal-geometry-reference-2026-10-23.json).

WORLD07 adds a default-off, parent-gated near-eye fern/reed layer to the bounded
survey plant consumer: nine L18 windows sampled on a disjoint L24 lattice,
canonical habitat/ground and the shared prototype/mesh/shadow/environment paths.
Inside the 73 m cover it replaces coarse ground clumps while retaining trees and
the far fallback; `understory=false` preserves WORLD04 exactly. The matched
standing pair retained 222 → 2,020 plants and measured 105.470 → 147.359 ms
median GPU time and 113 ms synchronous preparation. The frame remains sparse bare-ground savanna, not closed forest; no
performance, streaming or aesthetic acceptance is claimed. See docs 70/75/77 and
[`artifacts/native-near-understory-2026-10-23.json`](artifacts/native-near-understory-2026-10-23.json).

WORLD07a moves same-site understory preparation to one shared worker, coalesces
camera demand, rejects stale results and reuses compatible GPU plant sources.
The 96-frame orbit diagnostic removes the three >50 ms CPU submissions seen in
each baseline (158/237 ms maxima); candidate maximum is 27.7 ms. GPU medians
remain about 198–207 ms, and startup/site or capacity/layout-changing publication
can still stall. Exact descriptor/shadow/map lifecycle tests accompany the
capture; no visual, stable-FPS or planetary-streaming acceptance follows. Latest
main was incorporated into the topic branch at this checkpoint; the later
T4c promotion and remaining replacement gates are recorded in doc 78. Evidence:
[`artifacts/native-async-understory-2026-10-23.json`](artifacts/native-async-understory-2026-10-23.json).

WORLD07b fixes a specific stellar-visibility error: normal-offset shadow origins
could move the light ray into a disjoint narrow blade. The shared mesh hook now
preserves the physical line and excludes only a configured receiver interval.
An analytic quad regression and actual-producer f64 visibility oracle accompany
matched frozen-time captures. In 720 selected plant interiors, default-bias
invented blockers drop from 13 to zero; 13 genuine nearby blockers are still
skipped by finite bias. Sparse/subpixel foliage, the major diagonal branch
pattern, ground bands, wider transport and visual acceptance remain open. See
docs 75/77/78 and
[`artifacts/native-stellar-ray-line-2026-10-23.json`](artifacts/native-stellar-ray-line-2026-10-23.json).

On 2026-10-23, remote main was fast-forwarded from `d2bb846` to `99a1462`
(implementation `95a3888` plus evidence/docs); both main and the surface topic
were confirmed at that candidate. Launch defaults and the old host are retained.
The subsequent promotion record changes documentation only.

T4c (`95a3888`) integrates default-off native opaque history with masked-source
rejection. All 13 local command gates passed on 645 unchanged inputs, including
2,080 lib/bin tests, 117 integration tests, 47 native hardware/CLI checks and
87 renderer hardware checks. All nine final captures succeeded with simulation
time/step zero. Three matched static blind pairs are prepared but unreviewed;
orbit is excluded because material residency differs and resets in 111 of 126
submitted frames. The retained pre-temporal baseline is not bit-exact (six
pixels differ) or an isolated T4c reference. Composition, metric disocclusion,
object/ocean motion, streaming accumulation and independent visual acceptance
remain open. See docs 76/78 and
[`artifacts/native-temporal-history-2026-10-23.json`](artifacts/native-temporal-history-2026-10-23.json).

WORLD11a narrows ground pinnae to approximately 6:1 length/width and reduces
camber/fold without increasing triangle count. Geometric gaps replace the
previous broad-leaf appearance; placement and material maps remain canonical.
The actual-producer test now classifies lit-side shadow-edge near misses with
the existing edge band, retaining the two-code interior shading tolerance and
explicit interior-coverage counts. Latest scoped checks pass: 255 worldgen,
122 native unit and two hardware fixtures, plus strict workspace Clippy,
architecture/formatting, feature-off and release build. Matched standing control
and candidate retain 11,752 plants; GPU medians are 354.811/341.810 ms in one run
each, not an established speedup. Final reference acceptance remains open.
See [doc 77](77_NATIVE_SURFACE_SHOWCASE.md) and
[`artifacts/native-surface-fern-lamina-2026-10-24.json`](artifacts/native-surface-fern-lamina-2026-10-24.json).

The preceding WORLD11 checkpoint extended the bounded visual consumer to nine L14 canopy windows and
49 near-eye L18 windows, with scale-aware full/reduced pinnae and three dry
outer ground-fern fronds without extra triangles. The standing view now retains
11,752 plants (5,568 reduced), measured at 347.919 ms GPU median versus a fresh
4,931-plant / 318.347 ms control; this is wider coverage, not an isolated speedup.
The shared instance/shadow/RT policies, canonical ground and default-off state
remain. Local checks pass: 2,094 workspace lib/bin, 121 integration and 36 native
hardware tests, plus scoped lint/architecture/feature-off/release gates, with the
test-only lint-cleanup sequence recorded in the evidence. Bounded coverage,
discrete LOD, repetition, sky/ground composition and production frame cost
still prevent reference acceptance. See [doc 77](77_NATIVE_SURFACE_SHOWCASE.md)
and [`artifacts/native-surface-coverage-2026-10-24.json`](artifacts/native-surface-coverage-2026-10-24.json).

The preceding WORLD10 checkpoint reviewed Gemini iterations through `d13364b`, retaining its organic floor
and young-frond architecture while removing 13.5x ground-normal exaggeration
and correcting per-band material filtering. Curved indexed pinnae, a visible
lower asymmetric fork, geometric-hemisphere lighting and roughness-filtered
mesh environment response share primary/shadow/secondary consumers. Native
clouds share a filtered, mean-normalized 3D field and self-occluded diffuse
boundary; this remains a single-scattering approximation. The experimental
profile uses High plant density, PBR Neutral, four-sample MSAA and TAA off.
Renderer defaults are unchanged. Pixel L1 against the concept is not visual
acceptance. Current tests, captures, timings and remaining gaps are recorded in
[doc 77](77_NATIVE_SURFACE_SHOWCASE.md); neither reference parity nor production
performance is established. At that checkpoint, local checks passed (2,091 workspace lib/bin,
121 integration, 35 native hardware and 12 cloud/environment hardware tests,
plus scoped renderer/lint/feature-off gates). Final High standing retains
4,931 instances and measures 219.905/329.119 ms single-sample/4× MSAA on the
880M; this is still unsuitable for production frame rates. Exact provenance,
three candidate blind pairs and rejected residency comparisons are in
[`artifacts/native-surface-cycle-review-2026-10-24.json`](artifacts/native-surface-cycle-review-2026-10-24.json).

The historical WORLD09 checkpoint followed up the Gemini visual branch through `2b0ae6e`. It retains
the larger 25-window cover and tiered crowns while correcting leaf UV orientation,
adding physical midrib folds without more leaf triangles, and separating dry
fronds in source-owned materials. Forked trunks restore the reference silhouette.
The three prototypes total 10,360 triangles; five grouped draws share instance
ranges and 1,243,200 B of vertex/index/AO data. Six CC0 PBR maps have pinned,
reproducible provenance and mip filtering. Shared thin-diffuse R/T shading and
optional static-plant motion eligibility reach raster and secondary-hit consumers.
The review profile requests PBR Neutral with TAA off and single-sample raster;
optional 4× MSAA is available, while renderer defaults remain single-sample/Reinhard. Fog and procedural puddle paint added in the intervening
visual branch are removed; finite-distance atmospheric transport and wet-litter
source integration remain incomplete. Docs 59/69/70/75/76/77 describe the actual
scope, verification and expensive quality fallback. The ImageGen reference remains
substantially ahead of the engine; there is no independent blind acceptance.
The 2026-10-24 continuation passes 2,088 workspace lib/bin, 120 integration
and nine terrain-environment GPU tests plus the scoped checks recorded in
[`artifacts/native-gemini-surface-followup-2026-10-24.json`](artifacts/native-gemini-surface-followup-2026-10-24.json).
Single-sample/4× GPU medians are 385.169/669.766 ms on the local 880M;
performance remains unacceptable for a production scene.

WORLD08 continues surface quality on the surface topic after main's `f73eea4`
checkpoint: broader fern/crown leaf surfaces and arched reeds use 2,948 rather
than 4,284 prototype triangles (318,384 rather than 462,672 native bytes).
An eight-metre world-addressed occupancy prior groups the optional understory
within canonical ecological eligibility; it changes understory membership while
preserving IDs, exact window unions and nested quality. Large plants/rocks,
terrain/material lighting, settings/defaults and shared render paths are retained.
The ImageGen reference is a concept, not an engine or acceptance result.
Remaining transport, subpixel aliasing, root/near-far, streaming and visual
acceptance obligations are recorded in docs 70/75/77.
Eleven local checks pass on 645 unchanged compiled inputs (2,082 lib/bin,
117 integration and three focused native GPU fixtures). Nine final captures
exit zero, including two separately validated controls for the opt-in
`data/graphics/surface-showcase.toml` profile's existing 25 m near-shadow cover.
Three blind pairs are prepared without votes. Standing population changes;
the moving endpoint still has four material jobs. Stable performance, full
hardware-suite/platform and visual acceptance remain open. Evidence:
[`artifacts/native-leaf-colonies-2026-10-23.json`](artifacts/native-leaf-colonies-2026-10-23.json).

The 2026-10-21 WORLD05 prototype-geometry stage passed the same 14 local command
gates on committed `HEAD 132b709` with 755 unchanged inputs: 2,027 workspace
lib/bin and 102 integration tests, the focused plant hardware oracles, 79
renderer and 45 native hardware/CLI tests, and a release rebuild matching the
captured binary byte-for-byte (`6fe61390…`). The independent blind review and
single-order cost observations remain scoped as stated in doc 75. The legacy
Bevy `--ignored` command again matched zero tests and is not hardware coverage.
Evidence:
[`artifacts/native-plant-prototypes-2026-10-21.json`](artifacts/native-plant-prototypes-2026-10-21.json).
No remote CI or stable performance acceptance is claimed.

The 2026-10-20 native plume stage passed all 27 local command gates, including
workspace/feature-off and actual GPU suites. Of 607 tracked test inputs, 606
remained byte-identical; the only edit during the run was configuration comments,
with parsed TOML values independently verified unchanged. Exact evidence and
scoped timing/oracle bounds: `artifacts/native-plume-2026-10-20.json`.

The 2026-10-20 retained native docking replay and shared
joint-load correction passed 29 local command gates on 609 unchanged inputs
(1957 workspace lib/bin and 97 integration tests, plus actual GPU suites).
Exact scoped evidence: `artifacts/native-docking-2026-10-20.json`.

The native night-light follow-up passed 31 local command gates on 612 unchanged
inputs: 1962 workspace lib/bin and 97 integration tests, feature-off checks and
actual GPU suites. The retained bake matches all bytes in its recorded fixture;
GPU timing and primary-only numerical bounds are scoped in
`artifacts/native-night-lights-2026-10-20.json`.

The subsequent terrain/source regression and material-boundary stage passed
33 local command gates on 612 unchanged compiled/configuration inputs: 1967
workspace lib/bin tests, 97 integration tests and the actual hardware suites.
The old Bevy ignored-GPU gate now matches zero tests: its 24 tests and the
formerly feature-gated mesh compute surrogate run as 25 Bevy-free tests in
`render-wgpu`; this is coverage relocation, not skipped hardware testing.
Seasonal far-visible B and A-hidden/B-visible terrain checks passed for regular
and conforming geometry. Evidence:
`artifacts/native-terrain-seasons-2026-10-20.json`.

The native rough sky-reflection stage (doc 69) passed 13 local command gates on
613 unchanged compiled/configuration inputs: 1968 workspace lib/bin tests,
97 integration tests, 84 native feature-off tests, 70 renderer ignored hardware
tests and 43 native ignored tests. This includes the five new terrain
reflection/oracle/cost tests; it is not whole-frame visual acceptance or remote
CI. Scoped evidence: `artifacts/native-terrain-environment-2026-10-20.json`.

The native terrain grazing-horizon guard (doc 69, **superseded by the audit
below**) followed on 613 unchanged
compiled/configuration inputs with 13 local command gates: 1968 workspace
lib/bin tests, 97 integration tests, 84 native feature-off tests, 72 renderer
ignored hardware tests and 43 native ignored tests. A new GPU compute probe
calls the actual shared grazing function at N.V=0, 1e-7 and 0.05; it fails
against the pre-guard shader at N.V=0 and returns bounded non-zero radiance
with the guard. A full-terrain probe covers regular and conforming geometry at
N.V=0.01/0.001/0.0001. All nine
real captures (three sites at 500 m and 5 km, two conforming controls and one
specular-off control) have no pixel with channel sum below 45, with a minimum
of 183 and a specular-off minimum of 278. This corrects the earlier
background-gap reading: the black fragments were zero-radiance grazing
reflections, not geometry holes. Thin dark ridge rims remain reflected sky with
specular enabled; visual acceptance is still incomplete. Evidence:
`artifacts/native-terrain-horizon-guard-2026-10-20.json`.

The material-normal continuation adds a native optional RG8 page plane, correct
cube-sphere chart metrics, filtered gradient mip/residency blending, and a
resolved restart-required disable with no normal page generation/upload or full
array allocation. The primary raster has an independent unsaturated Lambert
oracle; secondary rays and Bevy remain albedo-only/flat-normal consumers.
Doc 69 records the Minecraft/procedural-renderer reference review and licensing
boundaries. The shared grain now has a bounded material-state response, but is
not calibrated natural substrate relief, vegetation geometry or whole-frame
visual acceptance. The final normal-plane stage passed all 13 local
gates on 614 unchanged compiled/configuration inputs; its 14 silent captures,
paired generation workload, raster/upload cost and known appearance defects are
recorded in `artifacts/native-terrain-normals-2026-10-20.json` (not remote CI or
blind acceptance).

The material-response follow-up on `e567274` adds bounded existing-material
response to the normal gradients and roughness, fixes the albedo-only Bevy
lookup and disabled optional-mip validation paths, and keeps physical terrain
unchanged. Workspace formatting, all-target/all-feature Clippy, 1981
workspace lib/bin tests, all workspace integration tests, the architecture
guard, the enabled/disabled Bevy shader parse test and actual 880M renderer
diagnostics passed. The 64-page close-land benchmark measured p50 118.946 ms
disabled / 122.114 ms enabled; eight real on/off captures show no stable
whole-client timing conclusion. Four matched pairs are staged for blind review,
not yet reviewed. Exact checks, current page-build measurements, capture hashes
and limitations are in
`artifacts/native-terrain-material-response-2026-10-20.json`; visual acceptance
remains open.

The subsequent audit of `cc037f1` reproduced two material bugs (buried geology
roughness leaking through opaque cover and zero-masked water-side normal
stencils) and two grazing-lighting errors. It replaces the sky-floor workaround
with the finite Schlick/GGX visibility ratio and uses that same rough-material
directional reflectance for reciprocal incoming/outgoing diffuse throughput.
An intermediate outgoing-only complement was rejected for directional energy
creation despite passing a white furnace. Cloud transport budgets no longer
alter material quadrature. Expanded independent grazing, collimated-energy,
no-field direct lighting and furnace-deficit fixtures cover the revised model.
Current evidence and explicit visual gaps are
in `artifacts/native-terrain-material-audit-2026-10-20.json`; see doc 69 for
the corrected derivation. The historical horizon-guard artifact is a record of
the rejected workaround, not current behavior or a BRDF-accuracy guarantee.
Local verification covers 1983 workspace lib/bin tests, 98 integration tests,
76 distinct renderer hardware tests (one long target rerun after command
timeout), all-feature Clippy, Bevy shader parsing and the feature-off native
path. The final native ignored suite also passed all 43 tests after building its
debug-server prerequisite.
Six final silent captures have no missing material pages; one fresh
blind reviewer preferred the final correction in all four matched cases while
still flagging blur/stretch, angular contours and weak atmospheric depth.
No performance win, global visual acceptance or full client parity is claimed.

The remaining source-to-consumer retirement obligations are tracked in
[doc 66](66_BEVY_RETIREMENT.md). Remaining host adapters and game functions must
be replaced before removing `apps/client` and `crates/bevy-rcbt`; a clean native
terrain dependency graph alone is not full-client replacement.

The 2026-10-19 sky/audio stage passed its twelve local stable-source gates.
Its combined-flight capture nevertheless exposed triangular darkening in opt-in
conforming terrain (also with ocean disabled, absent on regular patches).
Follow-up isolation identified self-shadow sampling, not missing geometry.
Doc 63 records the shared receiver-plane correction and real-occlusion oracle;
doc 66 and the dated sky/audio artifact retain the original rejected evidence.
No whole-frame visual acceptance follows from the passing numeric effect tests.

The doc 39 branch also shares the owned GPL surface-survey camera between
clients (quaternion orbit/free look/zoom/reset and native bookmark selection).
Doc 63 adds persistent canonical topology to the actual native window/headless
frame; WORLD13/doc 77 now selects it in both checked-in native launch profiles
and publishes sparse overlapping height pages independently of triangulation. Shared shadows/ocean/RT use its emitted epoch rather than
regular-grid membership; this closes consumer selection, not the wider client
replacement, geometric-error/convergence or primary-reflection acceptance gates.
Doc 64 retains the rejected optimization snapshot. The corrective follow-up in
doc 63 restores current-camera coordinates, uses completed GPU zero-request/
zero-commit feedback instead of a wave-count guess, and fixes mesh-caster and
shadow-matrix invalidations. Its regressions now pass; original speedup claims
are not accepted. Doc 65 audits sky/star/cloud/aurora reuse before implementation.
Doc 59 supersedes the older native no-HDR checkpoint: terrain, mesh/RT and sky
can now retain linear highlights in a shared float target before SDR mapping.
Automatic exposure and bloom remain unported; this is not HDR display output.
Native survey framing is now 5 km by default rather than 30 km; prior terrain
timings are not an equal-workload comparison. Doc 67 supersedes the earlier
unported map-camera boundary: the native system map now has a shared neutral
camera/navigation core. Maneuver interaction remains; the shared instrument
follow-up below supplies dominant-body and speed/altitude display behavior.
The owned native embedded-server transport now lives in GPL `flight-net`
and now carries native pause/warp votes and relaunch commands. CLI/HUD/key
controls reuse the existing authority: no solver-dt change, second simulation
clock or local fallback on server loss. Requested and acknowledged state stay
separate; focus loss releases held inputs. See doc 66's retained-flight audit.
Native Mouse Aim now consumes the shared cursor/deadband conditioner with manual
stick takeover, unit-clamped diagonal demand and exclusive camera/UI ownership.
Held F temporarily inverts retained SAS intent and releases safely on focus loss;
the lean wire snapshot does not acknowledge SAS. This is input integration, not
a new control law or completion of map/help/guidance migration.
The native continuation also restores F1 help, F2 flight-HUD visibility and F3
telemetry/fleet visibility. Help captures manual/camera input without pausing or
cutting thrust; hidden navball callbacks are released, and map UI remains usable.
These are session-only UI adapters. Opt-in coastal flyby guidance is now a
direct native port (F7/`--flyby`/HUD): demands-only pilot input over the
acknowledged authority — with takeover, focus-loss, pause/warp and link-loss
cancellation and no auto-resume — and in server mode intents only, never local
steps or fabricated air/force observations. The retained client's negated
bank-hold sign is fixed in both decks. The one-shot typed server
`ExecuteManeuver` fixture is likewise ported (see below); remaining
guidance/target semantic inputs stay open.
The map now reads the physics optimizer's existing JIT cache, through shared
execution eligibility/interval/geometry guards; it neither requests a bake nor
substitutes a hypothetical zero-force coast during powered/live flight. Only
the currently executable bounded coast interval is drawn; generalized powered
JIT futures remain unimplemented (doc 07 §7.10). Native local physics reuses the
server's extracted one-worker ThreadBakeQueue. Dominant-body Ap/Pe/escape and
local-wake readouts have native consumers; remote mode shows transmitted notices,
never a local future/scheduler as authoritative.

Both flight decks now consume `flight-authority::instruments`: dominant-body
kinematics, local air/load and instantaneous apsis diagnostics, plus wheel/canopy
observations. Native session-only speed/altitude selectors do not change local
attitude, solver reference or controls. Home terrain/rotation/air never migrate
to a foreign display body; acoustics keeps the source's reference atmosphere.
Lean replicas withhold absent air/thrust/TWR/g-load even when display-only force
reconstructions exist. Checked transmitted canopy/wheel loads remain observable.
Target/guidance semantic inputs and remaining flight/event fixtures stay open.
The one-shot typed server `ExecuteManeuver` fixture is ported as native
`--maneuver-kick MPS` (server-flight only, one prograde node on the normal
wire, one session maximum); local mode has no executor and stays rejected.
The transport retains its stdio handshake, receive timestamps/inbox, input
mailbox and child cleanup;
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
simulation/counter coverage. The windowed F4 overlay/capture/profile adapter
over the same collector is native now: always-kept wall/CPU frame records
with scopes only while visible/capturing, GPU time reported unavailable in
the window (the blocking interval readback would stall the live present
loop). StagingBelt A/B and submission batching remain open.
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
| `15_PERFORMANCE_MONITORING.md` | ✅ active | capture/scopes/overlay/counters, native headless GPU intervals, opt-in crash-durable serial stage traces with mesh-work admission | Radeon validation of stage tracing, window/Bevy GPU timestamps, finer GPU scopes, backend memory, dev levels |

## Process and reference

| Doc | Status | Notes |
| --- | --- | --- |
| `06_OPEN_QUESTIONS.md` | 🟡 living index | Rapier decided; stdlib/ownership narrowed; J2/C22 landed; rest open |
| `35_BLAZE_AUDIT_2026_09_12.md` | 🕰️ record | scoped to base `45e0b79`; see 36 for current status |
| `36_DOCUMENTATION_AUDIT_2026_09_14.md` | 🕰️ record | snapshot `e1454ac`; superseded by this file for post-merge state |
| `44_KSA_TECHNICAL_COMPARISON_2026_09_20.md` | 🕰️ external audit | KSA comparison: warp, rings, glints, plume, instruments, body orientation |
| `details/01_DOCKING_PORTS.md` | 🟡 D1 contact-scene slice | persisted protocol, fixed-joint install/removal, pressure gate, corrected latest-substep joint-load readback, authored ports, server fleet ownership and snapshots; native solved two-craft replay, D1 rest surfaces, inspector and structural cue | calibrated capture/class ratings, petal actuation, jointed-stack control, interactive ownership/commands, live-flight patch/normal overlays and complete structural integration |
| `REFERENCES.md` | 📚 reference | UX-only refs; Nyx/ANISE isolation; no copied code |
| `adr/0007–0011` | ✅ accepted | licensing, render boundary, autopilot graphs, aero boundary, RCBT boundary |

Native primary-only render-work checkpoint (2026-10-27): [80 — Horizon review](80_HORIZON_RENDER_WORK_REVIEW.md) is a partial architectural correction: conservative primary frustum selection, persistent indexed indirect batches with direct fallback, actual candidate/submitted counters and reuse of unchanged instance/command data. Canonical identities and full shadow/RT sources remain. At that checkpoint, receiver/texel-aware shadow representation, moving-camera work bounds, complete cutout-material semantics and target-hardware frame-time acceptance remained open; doc 81 below implements the geometric shadow representation slice.

Native shadow-representation continuation (2026-10-27): [81 — Native shadow mesh LOD](81_NATIVE_SHADOW_MESH_LOD.md) adds producer-certified depth-only plant geometry selected per light/cascade/instance by projected error. The legacy default is full geometry; the showcase requests 0.5 texel. The fixed coastal cold shadow plan falls from 999.6M to 283.2M encoded mesh triangles with 991,680 additional unique caster bytes under unchanged 48 MB admission. Primary/RT sources and world identities remain. Geometric error is not a global radiometric or temporal guarantee; the remaining cold work is still large and physical-GPU replay stays blocked.

Native shadow-submission continuation (2026-10-28): [82 — Receiver-aware shadow submission](82_NATIVE_SHADOW_SUBMISSION.md) adds opt-in primary-receiver light-plane hulls with source-arithmetic/filter support and RT/unknown-bound fallback, view-key invalidation, shared-budget indirect shadow batches, dirty uploads and cached world spheres. Moving-origin shadow reuse, physical-GPU/full-frame timing and broader radiometric/temporal and independent blind reference acceptance remain open. Hardware replay remains blocked.

Doc 82 final verification: 519 CPU tests and 12 build/architecture gates, five actual isolated software-Vulkan render regressions and one dropped-encoder native probe. The ocean-profile plan uses 453 API batches for 6,930 logical shadow commands and 283,223,492 mesh shadow triangles; primary work and world identities remain. These counts do not establish target-GPU frame time.

Native shadow hierarchy (2026-10-28): [83 — Native shadow geometry hierarchy](83_NATIVE_SHADOW_HIERARCHY.md) adds multiple certified levels per primitive, an unresolved-organ coarse tier and reed caster, sorted coarsest-fitting selection, per-level indirect runs and a same-build original producer oracle. Coastal shadow work falls 283.22M → 252.05M with unchanged primary geometry and identities, 46,159,116 source bytes under 48 MB admission, and explicit full-source versus before/after evidence scopes. Physical-GPU/frame-time, general radiometric/temporal and blind reference acceptance remain open.
