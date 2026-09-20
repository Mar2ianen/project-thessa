# 00 — Documentation status: implemented vs future

Status: living index. Each row states what is merged reality and what is still
design. Per-file `Status:` headers agree with this table; open decisions live
in `docs/06_OPEN_QUESTIONS.md`. Regenerate `data/system.baked.json` via
`thessa-system-baker` after any `data/system.toml` edit.

Legend: ✅ implemented · 🟡 partial (shipped slice + open remainder) ·
🔵 design baseline/target (no code yet) · 🕰️ historical record (dated audit,
do not update in place).

## Simulation and physics

| Doc | Status | Implemented | Future |
| --- | --- | --- | --- |
| `01_CELESTIAL_SYSTEM.md` | ✅ reference | baked ephemerides, summed gravity, 32-body working set | stability/Halo/co-orbital validation checklist |
| `03_PHYSICS_ENGINE.md` | 🟡 prototype | DP5-FSAL + DP8-DOP853, variational STM, EphemerisFrame, monopole tree + single-tick cohorts, thrust/RTN arcs, envelope-barrier + active-set allocator, Rapier contact backend, gated J2/C22 harmonics | quadrupole, time-span patches, thermal/structural/fluid graphs, CFD, fracture |
| `08_NUMERICAL_VERTICAL_SLICE.md` | 🟡 prototype | test-particle contract, baked hierarchy, on-rails/Verlet, harnesses, gated J2/C22 | fitted segments, higher-degree harmonics, joint multi-leg shooting, structural/thermal |
| `11_AERODYNAMICS.md` | ✅ runtime model | panel SoA/SIMD + tables + upper-band/vacuum reductions | body gas/weather/winds, full wake, hypersonics, arbitrary axes |
| `20_ADVANCED_AERO_EFFECTORS.md` | 🔵 design target | background only (incidence-only control) | flaps/spoilers/hinged-panels/grid-fins, neutral bounds, `AeroEffectorModel`, high-speed plan (§§17–20: boom/buffet/plasma/vortex/ground-effect) |
| `23_GRAVITY_FIELD_COHORTS.md` | 🟡 partial | monopole tree, single-tick patches, Hessian spatial bound | time-span patches, quadrupole, cohort keys, planner-patch reuse, GPU |
| `24_ANALYTIC_AFFINE_PROPAGATION.md` | ✅ prototype | far-only analytic STM on single-tick cohorts | atmosphere/thrust/contact integration, global proof |
| `40_RAPIER_COLLISION_INTEGRATION.md` | ✅ baseline | local contact solver, zero-gravity Rapier, readback, regime switch | wheels, rich-terrain contact boundary, full PBR parity |

## World and lore

| Doc | Status | Implemented | Future |
| --- | --- | --- | --- |
| `02_WORLD_ATLAS.md` | 🟡 partial | §§1–4, Mora midpoints | §5.1/§7 point at 02B; §8 rules ongoing |
| `02A_ATMOSPHERE_MODEL.md` | 🟡 partial | data rule + A-system/Janus/Mora rows | TBD cells, Khepri local field, biome coupling |
| `02B_BC_SUBSYSTEM.md` | ✅ reference | working ranges = baked midpoints | formation narrative, canon lock, stability proof |
| `02C_FAR_COMPANION.md` | 🔵 design target | — (deliberately unbaked) | encounter geometry, cloud phase-space, travel benchmarks |
| `02D_BIOSPHERE_CHIRALITY.md` | 🔵 future work | — | food-refinery progression, biosafety, narrative |
| `03_INTERSTELLAR_SCOPE.md` | 🔵 design baseline | — | cloud model, phase-space, travel benchmarks, RSS egg |
| `13_THESSA_V02_DESIGN.md` | 🟡 partial | bulk values, landmark/biome recipes | §2 stellar proposal, climate/clouds/tidal targets |
| `02_GAMEPLAY.md` | 🔵 baseline | §2.5 chain; §2.7–2.8 shipped subsets noted | editor, economy, contracts, logistics blocks, certification |

## Client, render, terrain

| Doc | Status | Implemented | Future |
| --- | --- | --- | --- |
| `04_RUNTIME_ARCHITECTURE.md` | 🟡 prototype | process split, 120 Hz authority, warp/rails, snapshots, f64→render-local | editor/graphs, persistence, WASM, non-promises |
| `09_BEVY_VISUAL_SLICE.md` | 🟡 prototype | map/pilot/terrain-CPU/water-cubemap/atmosphere + opt-in GPU CBT + beauty/plume | multiplayer/auth/persistence, WASM |
| `10_PILOT_INTERFACE.md` | ✅ prototype | HUD/navball/camera/commands/telemetry/snapshot boundary | — |
| `14_VISUAL_ATMOSPHERE.md` | 🟡 partial | raster/LUT + shell clouds + gas-giant bands + aurora + field-first plume | volumetrics, weather coupling, full Solari |
| `19_ALERTING_AND_FLIGHT_PHASES.md` | 🔵 design target | background only (regime/mode inputs) | `FlightPhase`, alerts, arbitration, Slices A–D |
| `21_TERRAIN_STREAMING_THROUGHPUT.md` | 🔵 baseline | invariants normative | scheduler, geomorph, UMA fast path |
| `22_RCBT_GPU_TERRAIN.md` | 🔵 baseline | §§1–18 normative | GPU bisector pool, incremental lists, native Vulkan, compressed pages |
| `37_CBT_INTEGRATION_STATUS_2026_09_14.md` | ✅ opt-in | fallback + indexed raster + material pages | visual acceptance, numeric comparison, persistent topology |
| `38_CBT_RENDER_AUDIT_2026_09_15.md` | ✅ audit | defects fixed + follow-ups | bisector pool, shadow parity, FFT ocean, virtual texture |
| `38_ENGINE_PLUME_RENDERING.md` | 🔵 design target | — (replaces `beauty.rs` smoke test) | residual bricks, RT lighting, Ultra |
| `39_BEVY_EXIT_AND_ENGINE_MIGRATION.md` | 🔵 migration plan | policy/freeze rules | §§4–20 migration phases |
| `41_MICROSCALED_SURFACE_STORAGE.md` | 🔵 design target | — | codec, error metrics, material pages first |

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
| `details/01–04_*.md` | 🔵 design baselines | docking/aero/fuselage/propulsion families; values TBD |
| `REFERENCES.md` | 📚 reference | UX-only refs; Nyx/ANISE isolation; no copied code |
| `adr/0007–0011` | ✅ accepted | licensing, render boundary, autopilot graphs, aero boundary, RCBT boundary |
