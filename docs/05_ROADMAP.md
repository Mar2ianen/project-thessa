# 05 — Roadmap: от формул до игры

Это **dependency order**, не календарный план.

Несколько архитектурных boundaries являются cross-cutting и не откладываются до соответствующего gameplay milestone:

- server-authoritative semantic model существует с ранних vertical slices; M6 означает multiplayer/network hardening, а не первое появление authority;
- canonical surface/query path существует независимо от renderer;
- Bevy/wgpu — текущая client integration, но reusable GPU algorithms не должны принимать Bevy/wgpu types как свой domain API;
- terrain renderer может меняться независимо от `PlanetField`, contact representation и headless server.

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

- Bevy client visualization shell; reusable numerical/render subsystems keep their own engine-independent boundaries;
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

## M2.5 — canonical surface + adaptive terrain architecture

Это отдельный dependency milestone перед большим surface gameplay, потому что terrain больше не является одной фичей renderer'а.

### Authoritative surface

- сохранить observer-independent `PlanetField`/surface-query contract;
- сильнее bake'ить low/mid-frequency canonical height в hierarchical cube-sphere pages;
- хранить per-page conservative `min/max height`, error/slope bounds и compact quantized residuals;
- оставить bounded procedural short-wave residual только там, где он дешевле хранения;
- headless query/landing/contact path не зависит от render mesh/GPU;
- local contact patches материализуются по physics need, а не camera LOD.

### Client adaptive geometry

- current CPU tile builder остаётся measured baseline/fallback;
- `rcbt` prototype: pure Rust logical CBT/LEB layer + differential oracle against upstream `libcbt`;
- performance goal — materially beat reference workload, а не просто сделать порт без regression;
- packed/batched tree representation, false-sharing/scaling measurements, cache padding only where measured;
- portable `rcbt-wgpu` backend;
- thin `bevy-rcbt` integration;
- optional native Vulkan backend behind the same semantic backend API when profiling gives a concrete reason;
- no Bevy/wgpu/Vulkan types in `rcbt-core` public API;
- GPU split/merge + compact/indirect draw replaces CPU topology churn when parity is proven;
- cooperative/matrix hardware рассматривается только как optional compressed-height-page decoder, не как обязательный CBT primitive.

Подробности: `docs/21_TERRAIN_STREAMING_THROUGHPUT.md` и `docs/22_RCBT_GPU_TERRAIN.md`.

**Exit:** одинаковая canonical surface доступна headless server и client; server surface queries используют baked hierarchy/error bounds; client умеет рендерить ту же поверхность через measured GPU adaptive topology path без зависимости core algorithm от Bevy/wgpu.

## M3 — Thessa vertical slice

- terrain + floating origin поверх M2.5 representation split;
- player movement;
- authoritative local surface/contact integration для player/vehicles;
- resource nodes;
- building placement;
- power;
- miner/smelter/assembler/storage;
- belts/pipes;
- first vehicle depot;
- save/load.

**Exit:** игра уже является маленьким first-person factory builder даже без других moons, а renderer terrain representation можно заменить без изменения canonical surface/save/server semantics.

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

## M6 — multiplayer hardening

Server authority к этому моменту уже существует. Здесь добавляется именно multi-user networking/productization:

- commands/snapshots over network;
- prediction/interpolation/rollback policy;
- interest management;
- shared warp consensus;
- persistent server saves;
- multiple unobserved surface vehicles using the same canonical baked/query representation;
- Lightyear spike/decision;
- web dashboard/spectator prototype;
- Linux + Windows + macOS native packaging smoke tests;
- WASM/WebGPU build smoke test.

**Exit:** несколько игроков могут строить, летать и warp'ить один causal world; подключение/отключение spectator/client не меняет physical terrain or landing result.

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
- complicated NPC civilization;
- native Vulkan backend только ради самого факта Vulkan, без measured limitation wgpu path;
- neural/cooperative-matrix terrain decoder до доказанного page bandwidth bottleneck.

Сначала доказать главный loop и массовую физическую логистику.
