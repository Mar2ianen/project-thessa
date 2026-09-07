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

**Exit:** один solver способен разумно воспроизводить conventional aircraft, spaceplane и Starship-like belly-flop/flip/landing без vehicle-class hacks.

## M3 — Thessa vertical slice

- terrain + floating origin;
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
