# 02 — Базовые механики и полный функциональный контур

## 2.1. Жанровое ядро

**First-person factory builder + физический aerospace sandbox.**

Не цель: Factorio с космическими таймерами. Не цель: KSP с меню колоний. Игрок физически находится внутри инфраструктуры, строит её, проектирует транспорт и затем автоматизирует реальные маршруты.

Основной цикл:

```text
найти ресурс
→ добыть
→ переработать
→ построить производство
→ упереться в географию/throughput
→ спроектировать транспорт
→ автоматизировать маршрут
→ получить новый материал/технологию
→ масштабировать сеть
```

Космос — следующий transport tier после belts/trucks/trains/aircraft, а не отдельная сцена.

---

## 2.2. Старт

Игрок начинает на **Thessa** с минимальным deployed habitat / cargo module и ручным/полуручным инструментом.

Нет:

- готового космодрома;
- бесплатной фабрики;
- магического VAB, создающего аппарат за деньги;
- science points за флаги.

Первые часы:

```text
manual extraction
→ power
→ miner
→ smelting
→ belts/pipes
→ assemblers
→ storage
→ ground vehicle
→ larger factory
→ aerospace materials
→ first flight
```

Первая ракета должна быть следствием уже существующей промышленности.

---

## 2.3. Производство

### Building classes

Минимальный полный набор:

- miners / pumps / atmospheric intakes;
- smelters / furnaces;
- chemical processors;
- separators / isotope separators;
- assemblers;
- heavy fabrication halls;
- tanks / silos / warehouses;
- conveyors;
- pipes;
- power generation / storage / distribution;
- vehicle depots / stations;
- rail;
- launch/landing pads;
- orbital docks / depots;
- research / test infrastructure.

### Items

Не симулировать каждый болт rigid body. Conveyor inventory может быть deterministic stream/slots, визуально соответствующий движению предметов.

Физичность обязательна там, где она создаёт игровой смысл:

- trucks действительно едут;
- trains действительно движутся по путям;
- aircraft действительно летают;
- rockets/boosters действительно проходят ascent/entry/landing;
- cargo имеет массу и влияет на аппарат.

### Throughput model

У каждой цепочки есть минимум:

- production rate;
- storage capacity;
- transport payload;
- route latency;
- cadence/window;
- failure/reliability;
- energy cost.

Большая latency не является штрафом сама по себе. После заполнения pipeline throughput определяется cadence/payload. Это делает десятки часов межлунного перелёта нормальным factory gameplay.

---

## 2.4. Ресурсная модель

Около 10–12 primary feedstock families, см. `data/resources.toml`.

Принцип:

- не усложнять сырьё ради chemistry simulator;
- география должна заставлять строить логистику;
- refined products и components могут быть значительно разнообразнее raw resources.

Пример progression pressure:

```text
Thessa gives Fe/Cu/stone/quartz/water/carbon
→ sulfur удобнее Pyra
→ aluminium + Ni/Co + fissile scale требует Auron
→ D/Li chemistry выгодна Pelagos
→ N2/CH4/ice scale выгодна Borea
→ industrial He3 требует Nereid atmosphere
```

---

## 2.5. Параметрический vehicle editor

Редактор не делит аппарат на классы `rocket`, `plane`, `truck`, `spaceplane`.

Игрок получает геометрию и системы:

### Geometry

- fuselage/tank sections с настраиваемыми cross-sections;
- wings/control surfaces;
- fairings/cargo bays;
- structural beams/frames;
- landing gear / wheels;
- hinges/actuators;
- radiators;
- intakes;
- engine mounts.

### Engines

Двигатель задаётся не только prefab name, а технологическим классом + параметрами.

Chemical example:

- propellant pair;
- chamber pressure class;
- mixture ratio;
- throat/scale;
- expansion ratio;
- throttle range;
- restart capability;
- gimbal;
- cooling class;
- efficiency class;
- engine TWR class.

Игра выводит thrust, Isp, mass, heat, flow, manufacturing requirements.

### Design compilation

После редактирования `VehicleDesign` компилируется в shared immutable data. 300 одинаковых грузовых craft используют один design asset, а instances хранят только state/damage/cargo/fuel.

---

## 2.6. Управление и fly-by-wire

Default UX похож на хороший fly-by-wire:

Игрок задаёт:

- pitch;
- yaw;
- roll;
- throttle;
- translation/RCS commands.

Control allocator распределяет command по доступным physical actuators:

- aerodynamic surfaces;
- thrust vectoring;
- differential thrust;
- RCS;
- reaction systems, если они физически установлены.

Рули сами выбирают знак/микс, но не получают magic authority.

Advanced modes:

- direct actuator mode;
- custom mixer;
- rate command;
- attitude command;
- AoA/g-limited law;
- custom script controller.

Editor должен уметь оценивать control authority в выбранных design points (скорость, Mach, altitude, mass state), а не только рисовать Center of Lift.

---

## 2.7. Surface transport

### Trucks

Базовая модель автоматизации:

1. игрок или planner задаёт/проезжает route;
2. сохраняются route corridor / waypoints / station rules;
3. autopilot физически управляет throttle/steering/brakes;
4. машина остаётся collision-capable world object;
5. route failure возникает из реальной геометрии/проходимости, а не только из таймера.

Можно эволюционировать от recorded route к path planner, но physical execution сохраняется.

### Trains

- physical consist;
- schedule;
- stations/load rules;
- signalling/intersections;
- high-throughput surface backbone.

### Aircraft

Промежуточный логистический tier там, где атмосфера и география это оправдывают. Те же vehicle/aero systems, что и у космопланов.

---

## 2.8. Космическая логистика

Корабль — такой же автоматизируемый transport entity.

Маршрут хранит не магический `A -> B time`, а:

- departure condition/window;
- guidance program;
- staging rules;
- target encounter;
- refuel/load/unload rules;
- return/reuse sequence.

### Reuse

Многоступенчатая многоразовость обязательна как поддерживаемый сценарий:

- booster separation;
- boostback;
- entry;
- aerodynamic control;
- landing burn;
- upper-stage/spaceplane return;
- pad turnaround;
- maintenance/refuel.

Автоматизированный booster физически садится. Он не исчезает после separation и не появляется в inventory.

### Gravity assists

Planner должен уметь находить/показывать:

- direct Hohmann-like transfers;
- faster high-energy transfers;
- resonant gravity-assist tours;
- low-energy routes near Lagrange regions.

Для factory network это создаёт выбор `latency ↔ propellant ↔ fleet size`.

---

## 2.9. Warp

Warp общий для всего server simulation.

### Multiplayer rule

- один global simulation time;
- warp request — shared/consensual;
- любой игрок может потребовать `x1`;
- политика голосования/host rights настраиваемая, но нет отдельных временных линий.

### Simulation

Warp не означает «заморозить фабрику и переставить корабль вперёд». За simulated time продолжают работать:

- production;
- storage;
- logistics;
- orbital propagation;
- automation;
- thermal/energy systems в требуемой fidelity.

Фактический максимальный warp ограничивается compute budget и текущим набором active high-rate events.

---

## 2.10. Alarms и event scheduler

Будильники — системная механика, а не только UI.

Примеры:

- `T - 5 min to atmospheric entry`;
- `next Auron window`;
- `cargo >= 20 t`;
- `tank temperature > limit`;
- `eclipse starts`;
- `maneuver node in 30 s`;
- `BC eclipse`.

Alarm может:

- уведомить игроков;
- попросить/снизить warp;
- разбудить script coroutine;
- включить automation branch.

---

## 2.11. Visual scripting / autopilot

UX reference — MechJeb: игрок должен иметь готовые понятные операции уровня `Ascent Guidance`, `Maneuver Planner`, `Landing Guidance`, `Rendezvous`, `Docking`, SmartASS-like attitude guidance. Разница: в Thessa это **компонуемые typed blocks**, а не отдельные несвязанные окна/режимы.

Полная модель описана в `07_AUTOPILOT.md`.

### High-level guidance blocks

- `Ascent(target_orbit, constraints)`;
- `PlanTransfer(target, objective)`;
- `ExecuteManeuver(node/plan)`;
- `Rendezvous(target)`;
- `Dock(port)`;
- `LandAt(pad/coordinates)`;
- `RecoverBooster(site)`;
- `HoldAttitude / HoldAoA / HoldRate`;
- `WarpTo(event)` / `SetAlarm`;
- `Load / Unload / Refuel`;
- `WaitForWindow`.

High-level block использует штатные planner/guidance/FBW системы и остаётся физически ограничен доступными actuators, thrust, propellant, thermal/structural limits.

### Composition

Блоки соединяются в graph:

```text
WaitForWindow(Auron)
→ Load(cargo)
→ Ascent(orbit)
→ Stage
   ├─ booster: RecoverBooster(home_pad)
   └─ upper:   PlanTransfer(Auron)
              → ExecuteManeuver
              → Rendezvous(depot)
              → Dock
              → Unload
```

Нужны first-class:

- sequence;
- `if/switch`;
- `wait until`;
- retry/fallback;
- loop;
- **parallel/fork/join**;
- reusable parameterized subgraphs/functions;
- typed vehicle/resource/target handles;
- explicit success/failure/abort outputs.

Parallel обязателен: separation физически создаёт независимые аппараты, и booster recovery должен идти одновременно с upper-stage mission.

### Low-level blocks

- read position/velocity/omega;
- read local flow/AoA/Mach/q;
- read temperatures/stresses;
- read orbit/frame/target state;
- command actuator;
- engine throttle/gimbal;
- RCS/translation;
- vector/math/PID/control primitives.

Игрок может собрать собственный guidance controller, но это не требуется для типового reusable launch.

### Runtime

UI block-based, Scratch/Snap-like по доступности; runtime — event-driven VM/IR. `WAIT UNTIL altitude < X` по возможности компилируется в wake condition, а не poll каждого graph на каждом physics tick.

## 2.12. Power

Источники:

- combustion/chemical early;
- solar;
- geothermal;
- fission;
- later fusion.

Solar model учитывает реальные источники света:

- A direct;
- reflected Nereid/rings;
- B;
- C;
- eclipses/occlusion;
- panel orientation;
- позже — спектральную эффективность.

Это позволяет night backup от внешней двойной и planetshine без специальных бонусов.

---

## 2.13. Propulsion progression

Progression — не `engine Mk1 -> Mk2`, а новые Pareto frontiers.

### Tier 0/1 — chemical

- solid/hybrid optional early;
- LOX/CH4;
- LOX/H2;
- atmospheric jets/turbines where useful.

Chemical остаётся endgame-relevant для high-TWR surface operations.

### Tier 2 — nuclear thermal

- reactor + working fluid;
- high thrust, better Isp than chemical;
- transfer stages/tugs;
- reactor/thermal shielding matter.

### Tier 3 — nuclear electric / ion

- reactor + electric thruster + radiator;
- low thrust, huge propellant efficiency;
- идеален для bulk freight, где latency приемлема.

### Tier 4 — D-T fusion

- deuterium from water/hydrogen;
- tritium bred from lithium-bearing feedstock;
- first prototypes are expensive;
- industrialization requires reactor/pellet/magnet/radiator logistics.

### Tier 5 — D-He3 / advanced fusion

- He3 mainly from Nereid atmospheric mining;
- atmosphere skimmers/separators become major industrial project;
- faster interplanetary logistics.

### Tier 6 — torch-class fusion

- near-Expanse gameplay envelope without copying named fiction technology;
- sustained acceleration changes optimal routing from impulse/coast to accelerate/flip/decelerate;
- very expensive energy/material/thermal infrastructure;
- first units can exist long before mass production.

Technology can be **prototyped** before it is **industrialized**. Один дорогой инженерный craft возможен; массовый fleet требует supply chain.

---

## 2.14. Research / progression

Не давать абстрактные science points за посещение места.

Progression sources:

- build/test milestone;
- material availability;
- prototype success;
- industrial throughput milestone;
- research facility requiring real inputs/energy;
- sample/data delivery where appropriate.

Технология считается реально освоенной, когда экономика способна её производить, а не когда UI поставил галочку.

---

## 2.15. Failure, maintenance, reliability

Не делать игру maintenance-clicker, но реальные systems должны иметь последствия:

- thermal overstress;
- structural fatigue/damage;
- actuator jam;
- engine degradation/failure;
- landing damage;
- propellant leak/boil-off;
- radiator failure.

Automation должна уметь вывести craft из расписания в service depot. Reliability — параметр fleet throughput, не только cinematic explosion chance.

---

## 2.16. Multiplayer

Target: cooperative persistent worlds.

- authoritative dedicated server;
- host-client possible;
- native client full control/prediction;
- optional WASM client: spectator/map/editor/light gameplay depending performance;
- interest management by celestial/local region;
- remote world не требует full-frequency snapshots.

Игрок на Thessa не должен получать 20 Hz transforms для каждого lander на Borea.

---

## 2.17. Графика

Цель — читаемая, красивая, но не GPU-first.

Design target:

- clean PBR;
- good atmosphere/scattering;
- clouds with scalable quality;
- terrain LOD;
- strong instancing/batching;
- shadows/SSAO/emissive;
- optional expensive effects;
- raytraced rendering never mandatory.

Принцип: новая графическая фича, которая поднимает minimum GPU выше 780M-класса, должна быть выключаемой.

---

## 2.18. Полный функционал release-vision

Release-vision считается достигнутым, когда в одном persistent world возможно:

1. начать с локальной базы на Thessa;
2. построить многолинейную фабрику;
3. автоматизировать trucks/trains;
4. спроектировать самолёт/ракету/spaceplane произвольной параметрической формы;
5. физически выйти на орбиту;
6. автоматизировать reusable launch loop;
7. построить orbital depot;
8. открыть регулярную межлунную логистику с реальными окнами;
9. использовать gravity assists/Lagrange infrastructure;
10. индустриализировать несколько moons;
11. построить nuclear/fusion supply chain;
12. добывать gas giant atmosphere;
13. поддерживать fleet из множества одновременно физических craft;
14. делать всё это в server-authoritative multiplayer с shared warp;
15. выйти к Orthea/Vesper;
16. в late game добраться до BC circumbinary Janus system.
