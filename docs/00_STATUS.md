# 00 — Documentation status: implemented vs future

Status: living index, reviewed 2026-09-28. Each row states what is merged
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
| `details/06_ATTACHMENT_MODEL.md` | 🟡 implemented assembly/resource slice | authored attach nodes, pose solving, transformed geometry/mass, validated part trees, runtime crew/air domains, fuel reachability, named non-tree crossfeed edges and pressure-limited feed-line routing, shared fixed-step tank allocation/commit and transfer, contact-scene D1 joint telemetry, topology splitting, per-link strength ratings with solver-load failure assessment, baked per-body ownership with definition-split migration (bench: 12.85 μs), mass/COM/inertia reconstruction from complete per-body properties (64-body benchmark: 29.78 μs), authored D1 docking ports, and server-level fleet ownership with separation spawning, dock sessions, vacuum-gated joints, and fleet snapshots | explicit per-part bodies for hand-authored hardware, drive/gear/chute/fold/disc migration, power/thermal network partition, internal-joint load-path resolution, jointed-stack control, secondary commanding, client fleet render, despawn, branched hydraulic networks and finite-rate cabin flow |
| `details/04_PROCEDURAL_PROPULSION.md` | 🟡 resource-aware runtime slice | compiled chemical/solid/nuclear-thermal, airbreathing/ESTOC/APU, electric, shaft-power, RCS and continuous/pulsed fusion models; fixed-step mounted-consumer allocation and moving-mass/inertia updates; tank-backed pneumatic/rocket-bootstrap starters, optional torque-rated starter/generator hardware, generator efficiency maps/local thermal limits, APU and jet generator export to the bus, independent LP/HP turbofan rotor dynamics with geared fan and coupled steady solve | general three-spool/clutched and free-power-turbine topologies, multi-spool ESTOC transitions, generator thermal-node integration, transient piston/electric source fidelity, higher-fidelity propulsion models and editor UI |
| `20_ADVANCED_AERO_EFFECTORS.md` | 🟡 partial foundation | incidence controls and hinged fuselage body strips with load-limited actuators | general flap/spoiler/hinged-panel/grid-fin models, neutral bounds, `AeroEffectorModel`, high-speed plan (§§17–20: boom/buffet/plasma/vortex/ground-effect) |
| `23_GRAVITY_FIELD_COHORTS.md` | 🟡 partial | monopole tree, quadrupole rung, single-tick patches, Hessian spatial bound | time-span patches, cohort keys, planner-patch reuse, GPU |
| `24_ANALYTIC_AFFINE_PROPAGATION.md` | ✅ prototype | far-only analytic STM on single-tick cohorts | atmosphere/thrust/contact integration, global proof |
| `40_RAPIER_COLLISION_INTEGRATION.md` | ✅ baseline | local contact solver, zero-gravity Rapier, readback, regime switch, stale contact telemetry clearing, articulated wheel bodies/joints, sensor-only tire queries, split mass properties, powered wheel/strut/brake/drive stepping | dynamic-body wheel contacts, richer terrain contact boundary, full PBR parity |
| `details/05_PROCEDURAL_LANDING_GEAR.md` | 🟡 partial | wheel/tire/strut/brake/drive laws, mass/COM bake, sprung/unsprung split, articulated Rapier wheel bodies/joints, retractable chassis, fold-out legs, powered authority state/telemetry | dynamic-body wheel contacts, granular soil response, steering/anti-skid, electrical bus limits, representative fleet benchmarks |
| `details/06_REACTION_WHEELS.md` | ✅ implemented model | named banks, per-axis torque allocation, mass/inertia bake, RCS residual routing, telemetry and pilot controls | automatic electrical-load coupling, rotor momentum, thermal state and desaturation |
| `details/07_PARACHUTES.md` | ✅ implemented model | named packs, pressure/q deployment gates, reefing, aerodynamic loads/failure, mass/inertia bake and telemetry | line elasticity, canopy deformation, inflation shock and packing/reuse |
| `details/08_VEHICLE_PART_COMMANDS.md` | ✅ implemented API | ordered group/named commands on wire v6, including per-engine throttle and propellant transfer plus fleet Separate/Dock/Undock; pilot local prediction and server application | stage-definition resolution and action-group routing |
| `details/09_ELECTRICAL_POWER.md` | 🟡 implemented vehicle slice | ideal shared bus, prioritized consumers, battery/ultracapacitor energy/efficiency limits with pooled headroom, fission and fuel-cell sources, APU and jet generation, solar arrays/occlusion/tracking, mounted electric-propulsion load coupling, vehicle-baker TOML and 64-vessel benchmark | automatic load coupling for nonpropulsive actuators, electrical circuits/current dynamics, ultracapacitor leakage, self-shadowing, multi-axis gimbals, reactor fuel-mass drift, docked-bus exchange |
| `details/10_THERMAL_SYSTEM.md` | 🟡 implemented vehicle slice | lumped thermal nodes with conduction links and fixed/foldable area radiators, shared occluded solar inputs with own-hull ray occlusion, Sutton-Graves aero heating, wired internal loads, stability-substepped integration, overheat reporting, mass/COM baking, vehicle-baker TOML and 64-vessel benchmark | shield-specific thermal-protection/ablation coupling (design baseline in 11), temperature-dependent strength, phase change, convective cooling, silhouette penumbra, multi-axis gimbals, automatic waste-heat wiring |
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
| `13_THESSA_V02_DESIGN.md` | 🟡 partial | bulk values, landmark/biome recipes | §2 stellar proposal, climate/clouds/tidal targets |
| `02_GAMEPLAY.md` | 🔵 baseline | §2.5 and §2.7–2.8 describe current flight/automation slices, including vehicle part controls | surface transport, editor, economy, contracts, logistics blocks, certification |

## Client, render, terrain

| Doc | Status | Implemented | Future |
| --- | --- | --- | --- |
| `04_RUNTIME_ARCHITECTURE.md` | 🟡 prototype | process split, 120 Hz authority, warp/rails, single-pass ingress dispatch, Arc-shared outbound snapshots, f64→render-local | editor/graphs, persistence, WASM, non-promises |
| `09_BEVY_VISUAL_SLICE.md` | 🟡 prototype | map/pilot/terrain-CPU/water-cubemap/atmosphere + opt-in GPU CBT + beauty/plume | multiplayer/auth/persistence, WASM |
| `10_PILOT_INTERFACE.md` | ✅ prototype | HUD/navball/camera, RCS/reaction-wheel/gear/parachute controls, typed part commands, telemetry/snapshot boundary | — |
| `14_VISUAL_ATMOSPHERE.md` | 🟡 partial | raster/LUT + shell clouds + gas-giant bands + aurora + field-first plume volume | volumetric clouds, weather coupling, full Solari |
| `19_ALERTING_AND_FLIGHT_PHASES.md` | 🔵 design target | background only (regime/mode inputs) | `FlightPhase`, alerts, arbitration, Slices A–D |
| `21_TERRAIN_STREAMING_THROUGHPUT.md` | 🔵 baseline | invariants normative | scheduler, geomorph, UMA fast path |
| `22_RCBT_GPU_TERRAIN.md` | 🟡 baseline | §§1–18 normative, Arc-shared extraction snapshots, batched COW page streaming, stable height slots, dirty-ordinal geometry dispatch | GPU bisector pool, native Vulkan, compressed pages |
| `37_CBT_INTEGRATION_STATUS_2026_09_14.md` | ✅ opt-in | fallback + indexed raster + material pages | visual acceptance, numeric comparison, persistent topology |
| `38_CBT_RENDER_AUDIT_2026_09_15.md` | ✅ audit | defects fixed + follow-ups | bisector pool, shadow parity, FFT ocean, virtual texture |
| `38_ENGINE_PLUME_RENDERING.md` | 🟡 implemented renderer slice | backend-neutral plume core, Low impostor, Medium/High field-integrated volume ribbon and field-derived light | live compiled engine/nozzle state in client, adaptive residual integration, ray-traced lighting and higher-quality tiers |
| `39_BEVY_EXIT_AND_ENGINE_MIGRATION.md` | 🔵 migration plan | policy/freeze rules | §§4–20 migration phases |
| `41_MICROSCALED_SURFACE_STORAGE.md` | 🟡 experimental | codec A–E, allocator/residency, GPU LOD/mip/aniso path, game + render-world A/B measured; compact path pre-decodes to RGBA and raw remains default | packed material-shader sampling, crack-free geometry, baked-format adoption, production height pages |
| `45_CAD_RCBT_GEOMETRY.md` | 🔵 design baseline | D1 STEP/BRep fixture audited; existing RCBT + mesh-shader terrain path reused architecturally | CAD importer, normalized BRep runtime, crack-free adaptive face meshing, editor integration |
| `46_AUDIO_AND_ACOUSTIC_PROPAGATION.md` | 🟡 first slice | `audio-core` path semantics, exact-vacuum gating, structural/direct paths, Mach-cone arrival; `audio-synth` live procedural engine DSP; temporary Bevy RCS/GPWS/docking/demo adapter | calibrated physical engine telemetry/spectra, procedural one-shot mechanisms, structural attenuation graph, real IVA listener, occlusion/reverb/HRTF, post-Bevy backend |

## Autopilot and roadmap

| Doc | Status | Implemented | Future |
| --- | --- | --- | --- |
| `05_ROADMAP.md` | 🟡 order | M0–M1/M3–M4/M6 slices; stdlib minimum; M5 search prototype | staging, editor, depots, canon ephemeris, UX |
| `07_AUTOPILOT.md` | 🟡 slice | IR, wait-bounded cycles, per-wait generations, 4 blocks, B-plane + variational, L0–L2 replays | full stdlib/editor, staging topology, L3–L4, powered rails |
| `18-control-guidance-autopilot.md` | 🟡 baseline | authority/laws/allocator/graph/QuickJS/waits/server | allocator generalization, wire replacement, Phases 4–5 |
| `12_X15_ASSET_PROVENANCE.md` | 📦 provenance | record current | — |
| `15_PERFORMANCE_MONITORING.md` | ✅ active | capture/scopes/overlay/counters | GPU timestamps, backend memory, dev levels |

## Process and reference

| Doc | Status | Notes |
| --- | --- | --- |
| `06_OPEN_QUESTIONS.md` | 🟡 living index | Rapier decided; stdlib/ownership narrowed; J2/C22 landed; rest open |
| `35_BLAZE_AUDIT_2026_09_12.md` | 🕰️ record | scoped to base `45e0b79`; see 36 for current status |
| `36_DOCUMENTATION_AUDIT_2026_09_14.md` | 🕰️ record | snapshot `e1454ac`; superseded by this file for post-merge state |
| `44_KSA_TECHNICAL_COMPARISON_2026_09_20.md` | 🕰️ external audit | KSA comparison: warp, rings, glints, plume, instruments, body orientation |
| `details/01_DOCKING_PORTS.md` | 🔵 design baseline | interface and gameplay contract | runtime implementation |
| `REFERENCES.md` | 📚 reference | UX-only refs; Nyx/ANISE isolation; no copied code |
| `adr/0007–0011` | ✅ accepted | licensing, render boundary, autopilot graphs, aero boundary, RCBT boundary |
