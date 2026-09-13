# 05 — Roadmap: от формул до игры

Это **dependency order**, не календарный план.

## M0 — numerical kernel

Цель: доказать, что математика и representations работают без Bevy gameplay.

- `SimTime`, frames, f64 state;
- baked two-body ephemeris prototype;
- point-mass multi-body gravity;
- adaptive orbit integrator;
- J2;
- tests: ellipse, Lagrange, nodal precession;
- benchmark 1k/10k test particles;
- offline Nereid resonant-system validation tool.

**Exit:** headless binary умеет стабильно прогнать craft вокруг Nereid и показать реальные perturbations/Lagrange behavior.

## M1 — rocket physics lab

- Bevy/wgpu cross-platform visualization shell (Linux-first dev; no DX-specific code);
- procedural cylinder/tank sections;
- chemical engine parameterization;
- 6-DoF rigid cluster;
- staging;
- atmosphere profile;
- basic drag/aero zones;
- FBW for TVC/RCS;
- first MechJeb-like composable blocks: HoldAttitude -> Ascent -> Stage -> booster recovery;
- simple launch/landing;
- telemetry/gizmos.

**Exit:** можно собрать двухступенчатую ракету, выйти с Thessa-like test body на орбиту и посадить booster physically.

## M2 — serious aero / spaceplane / belly-flop

- wing/control-surface geometry;
- local panel forces;
- Mach/post-stall model;
- hinge torque/actuator limits;
- aero occlusion;
- lifting body support;
- structural graph baseline;
- thermal graph + entry heating;
- fracture into multiple clusters.
- water interaction baseline: splashdown drag, buoyancy/floatation for
  debris and ditched craft, wave state as a visual-only field first
  (rendering lands earlier, see M3; no hydro CFD — response curves only).

**Exit:** один solver способен разумно воспроизводить conventional aircraft, spaceplane и Starship-like belly-flop/flip/landing без vehicle-class hacks.

## M3 — Thessa vertical slice

- terrain + floating origin;
- water rendering baseline: sky-cubemap specular + SSR on smooth pixels
  (raster), ray-traced specular under Solari; animated wave normals later;
- clouds first slice: thin high ice decks (design doc 13 §5), clear regions
  stay readable from orbit — billboards before volumes, never an opaque ball;
- player movement;
- resource nodes;
- building placement;
- power;
- miner/smelter/assembler/storage;
- belts/pipes;
- first vehicle depot;
- save/load.

**Exit:** игра уже является маленьким first-person factory builder даже без других moons.

## M4 — surface logistics + automation

- trucks;
- recorded/planned routes;
- trains;
- physical station load/unload;
- event-driven typed graph/VM;
- MechJeb-like high-level guidance standard library;
- sequence/condition/wait/retry/parallel/reusable subgraphs;
- alarms;
- basic time warp in singleplayer/server.

### M4a — autopilot track: stabilization rework first (prerequisite)

Current stabilization grew per-mode and per-callsite (SAS attitude law,
RCS arbitration, trim Newton, surface rate limiting, client input mapping,
SAS-target capture rules) — every new autopilot block would reinvent it.
Before the graph library: unify.

- Inventory: `allocate_controls`, trim solver, control modes, SAS/RCS
  toggles and capture, client axis mapping, saturation flags.
- One attitude-command architecture for all consumers (manual, SAS hold,
  graph blocks): attitude/rate laws -> trim service -> FBW allocator ->
  actuators. No per-mode Newton copies; trim stays a shared service with
  its convergence envelope, not an inlined loop.
- Explicit SAS/RCS arbitration and saturation semantics (who yields to
  whom, what latches, what the HUD reports).
- Manual-override and target-capture rules as data, not scattered
  conditionals (see docs/07 §7.2 layering: scripts emit guidance
  targets, never forces).
- Then graph blocks on top per AGENTS §9 combinators (sequence,
  condition/switch, wait/event, loop/retry/fallback, fork/join,
  parameterized subgraphs, abort paths) and docs/07 vocabulary.
- First consumers: planned landing site via `declare_obstacles`
  (touchdown validation before commit) and ascent/landing guidance;
  `stage/separate` multi-`VehicleId` handoff rides the same command bus.

**Exit (M4a):** manual flight, SAS hold and a scripted ascent fly the same
allocator with identical trim behavior; a landing script validates its
site against declared obstacles and aborts on violation.

**Exit:** фабрика может работать автономно и пережить несколько игровых суток warp.

## M5 — Nereid system gameplay

- canonical baked ephemeris v1;
- system map;
- transfer planner;
- orbital depots;
- reusable route automation;
- Pyra/Pelagos/Auron/Borea/Nix content;
- resource differentiation;
- gravity-assist route planning;
- eclipses/planetshine/multi-star solar.

**Exit:** реальная multi-moon industrial network.

## M6 — multiplayer

- authoritative server;
- commands/snapshots;
- prediction/interpolation;
- interest management;
- shared warp consensus;
- persistent server saves;
- Lightyear spike/decision;
- web dashboard/spectator prototype;
- Linux + Windows + macOS native packaging smoke tests;
- WASM/WebGPU build smoke test.

**Exit:** несколько игроков могут строить, летать и warp'ить один causal world.

## M7 — nuclear age

- fission power;
- nuclear thermal;
- nuclear electric/ion;
- radiator depth;
- cryogenics/boil-off;
- maintenance/reliability;
- Orthea + moons;
- Vesper + moons.

**Exit:** logistics topology меняется из-за новых propulsion Pareto frontiers.

## M8 — fusion industrialization

- D separation;
- Li/T breeding chain;
- pulsed fusion prototype engines;
- D-He3;
- Nereid atmospheric skimmers;
- isotope separation;
- high-power thermal/radiator systems;
- torch-class late game.

**Exit:** быстрый interplanetary fleet существует, но только после реальной supply chain.

## M9 — BC endgame

- inter-component flight at outer-star scale;
- Janus/Mora content;
- binary-star lighting/eclipses up close;
- circumbinary trajectory planning;
- long-haul automation.

**Exit:** внешний binary перестаёт быть sky object и становится industrial destination.

---

## Не делать раньше времени

- full weather CFD;
- full FEM;
- planet formation simulator;
- procedural galaxy;
- FTL;
- photorealistic renderer;
- сотни raw resource types;
- economy/market simulator;
- complicated NPC civilization.

Сначала доказать главный loop и массовую физическую логистику.
