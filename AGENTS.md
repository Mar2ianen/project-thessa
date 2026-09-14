# AGENTS.md

Правила для code/design agents, работающих с Project Thessa.

## 1. Главный инвариант

**Не подменять физическую причинность игровыми коэффициентами, если эффект можно получить из геометрии, материала, поля или исполнительного органа.**

Примеры:

- запрещено: `yaw_power`, `magic_drag`, `stability_bonus`, прямой поворот craft;
- допустимо: площадь руля, положение шарнира, момент привода, локальный поток, сила, плечо, FBW allocator;
- запрещено: телепорт груза между космопортами;
- допустимо: физический транспорт + расписание + буферы + warp.

## 2. Границы модулей

### `sim-core`

- authoritative simulation state;
- **не зависит от Bevy, Tokio, Lightyear, Avian**;
- допускаются math/serialization/data-parallel crates после review;
- все пространственные состояния authoritative simulation — `f64`;
- canonical units — SI;
- physics time — `SimTime`, не `Instant`/`SystemTime`.


## 2.1. Cross-platform invariant

- никакого DirectX/Vulkan/Metal/wgpu-specific API в domain/simulation/gameplay code;
- shaders и render data не проектируются вокруг DX-only semantics;
- current native client integration — Bevy + wgpu, но reusable GPU algorithm core **не обязан и не должен** экспортировать Bevy/wgpu types;
- для reusable GPU subsystem preferred layering: backend-agnostic core -> portable wgpu backend -> optional measured native backend -> Bevy adapter;
- допустимые backend families: Vulkan, Metal, WebGPU и backend, который wgpu выбирает на поддерживаемой платформе;
- direct Vulkan backend допустим внутри isolated renderer/GPU subsystem после профилирования; он не меняет simulation/gameplay API;
- использование D3D12 самим wgpu на Windows не делает DirectX частью архитектурного API; прямые DX12/DXR calls требуют отдельного ADR и по умолчанию запрещены;
- capability checks предпочтительнее vendor checks (`if AMD/NVIDIA/...`);
- Linux является first-class dev/runtime target, не портом после Windows;
- WASM/WebGPU компилируемость portable client/render path должна регулярно проверяться CI после появления web target.

### Bevy client

Bevy отвечает за текущую client integration:

- rendering integration / default wgpu backend;
- input;
- UI;
- assets;
- client ECS;
- debug gizmos/tooling;
- visual interpolation/extraction.

Bevy не владеет semantic API reusable renderer algorithms. Если subsystem можно использовать вне Bevy (`rcbt`, compression/streaming kernels и т.п.), его core types/traits не должны принимать `Entity`, `RenderWorld`, `wgpu::Device` и подобные integration handles как domain API.

`bevy::Transform` никогда не является authoritative космической координатой.

### Server

- Tokio: networking, async I/O, persistence orchestration, admin/metrics;
- Rayon/custom fixed worker pool: CPU-heavy simulation batches;
- CPU-heavy solver не запускается как обычный `tokio::spawn` future;
- simulation tick имеет один authoritative порядок фаз.

## 3. Небесная механика

- звёзды, планеты, луны и canonical minor bodies следуют baked deterministic ephemerides;
- runtime не интегрирует их взаимную динамику;
- корабли, станции, обломки и временные объекты получают сумму гравитационных ускорений от релевантных тел;
- никаких SOI-switch как части физики;
- SOI может существовать только как UI/optimization hint;
- gravity harmonics вычисляются в body-fixed frame;
- Lagrange points не хардкодятся: они являются следствием полей и эфемерид.

## 4. Аппараты

`VehicleDesign` компилируется как минимум в:

- render representation;
- collision representation;
- aerodynamic zones/panels;
- structural graph;
- thermal graph;
- mass/inertia model;
- actuator graph;
- fluid/electrical connectivity.

Не хранить сотни fixed parts только ради того, чтобы потом агрегировать их обратно.

### Rigid-body policy

Один connected structural cluster может интегрироваться как один rigid body, пока деформации допускают это приближение. Подвижные шарниры/створки могут иметь явные DOF. При разрушении structural graph разбивается на connected components, каждая становится отдельным cluster/body.

## 5. Аэродинамика и управление

- силы считаются локально по aero zones/panels;
- локальная скорость учитывает translational velocity, atmospheric motion и `omega x r`;
- control surfaces имеют hinge geometry, limits, rate и actuator torque;
- actuator может не достигнуть command deflection под аэродинамической нагрузкой;
- FBW выдаёт команды исполнительным органам, но не добавляет момент в craft напрямую;
- low-level/manual control обязан оставаться возможным.

## 6. Температура и разрушения

- температура не сводится к одной цифре на весь craft;
- thermal nodes связаны conductance/radiation edges;
- aerodynamic/engine/solar heating входит в тот же thermal graph;
- прочность материалов может зависеть от температуры;
- damage не должен автоматически означать explosion/despawn;
- structural failure меняет topology, mass, inertia, aero и thermal graph.

## 7. Ray queries

Physics ray tracing означает **геометрические ray/BVH queries**, а не обязательный DXR/Vulkan RT.

Canonical server path должен работать без GPU. Hardware RT/compute может ускорять клиентские/локальные расчёты, но не должен быть единственным способом получить authoritative результат.

## 8. Мультиплеер и warp

- server authoritative;
- клиент отправляет input/commands, не произвольный world state;
- controlling client может предсказывать свой craft;
- остальные interpolate snapshots;
- warp — единый server time scale;
- изменение warp — consensual/shared policy;
- alarm/autopilot event scheduler работает в simulation time.

## 9. Скрипты автопилота

UX reference: MechJeb-подобные готовые операции (`Ascent Guidance`, `Maneuver Planner`, `Landing Guidance`, `Rendezvous`, `Docking`, attitude helpers), но в Thessa они **не являются изолированными режимами**. Любой high-level action — блок с typed inputs/outputs, который можно вкладывать в reusable graph/subprogram.

Обязательные combinators:

- sequence;
- condition / switch;
- wait/event;
- loop/retry/fallback;
- parallel/fork/join;
- reusable parameterized subgraph;
- explicit abort/failure path.

`stage/separate` может породить несколько `VehicleId`; graph обязан уметь передать разные ветки управления booster/upper stage.

Предпочтение event-driven VM:

- `WAIT UNTIL` компилируется в wake condition;
- спящие программы не poll'ятся каждый physics tick;
- high-level блоки (`point prograde`, `land at pad`, `target orbit`) используют штатные guidance/control systems;
- low-level sensors/actuators доступны для продвинутых программ;
- один и тот же script layer используется для логистики и flight automation.

## 10. Производительность

Оптимизировать representation, а не физические законы. Но там, где эффект доказуемо пренебрежим (§13), — редуцировать модель, а не интегрировать шум.

Для low-level CPU/GPU code performance является частью architecture, но unsafe/layout/platform tricks допускаются только с benchmark и понятным fallback. «Idiomatic» не является целью, если измеримо мешает throughput; «грязный» layout trick не является целью, если counters не показывают проблему.

## 10.1. Политика редукции модели (bounded model reduction)

Главный инвариант (§1) запрещает подменять причинность коэффициентами **там, где эффект влияет на решения** (траектория, управление, разрушения). Где влияние ограничено доказуемой огибающей ошибки — редукция разрешена и предпочтительна, особенно для множества аппаратов. Каждое упрощение обязано иметь:

- **явную границу** в конфиге (плотность/высота/режим), а не магическое число в коде;
- **калибровку** reference-параметров измерением против полной модели;
- **абсолютную огибающую ошибки** против decision-relevant масштаба (вес, тяга, допуски батча), а не относительную (относительная врёт на малых величинах);
- **regression test**, пинящий огибающую;
- **benchmark** выигрыша.

Примеры действующей редукции:

- `vacuum_cutoff_density_kg_m3`: ниже порога среда — declared vacuum (точно 0.0), включаются exact-vacuum fast paths и rails-батчи;
- верхние слои (`cutoff < ρ < COAST`): эталонно-площадное сопротивление вместо панельного цикла, нулевой аэромомент (RCS доминирует на порядки);
- батчи летают только в declared vacuum; длина прыжка капирована (`MAX_COAST_BATCH_JUMP_S`) под responsiveness/wake latency;
- проверка близости в батчах — точная по эфемеридам (5 сэмплов craft–body), а не worst-case margin по цепной скорости: margin считал планету враждебной (подлёт на полной орбитальной скорости) и душил все околопланетные батчи; точная проверка плюс revalidation текущего состояния в допуске 5 м перед каждым прыжком даёт ту же безопасность без ложных отказов.

Предпочтения:

- SoA/AoSoA для массовых численных kernels;
- batch gravity/aero/atmosphere/thermal evaluation;
- AVX2 native baseline, optional AVX-512 build;
- adaptive simulation rate/step для разных режимов;
- expensive contacts включаются там, где реально есть контакты;
- profiler/benchmarks обязательны до ручных intrinsics;
- packed bitsets/bitplanes и word-wise operations допустимы для topology/state machines, если это улучшает measured workload;
- false sharing проверяется отдельно: per-thread hot mutable state можно cache-pad/align, но padding не добавляется blanket-правилом;
- `#[repr(C)]` используется для FFI/GPU ABI/layout contracts, а не как универсальная performance annotation;
- `#[repr(align(N))]`/padded wrapper требует benchmark на target workload и оценки working-set cost;
- hot read-only state и frequently-written counters/queue heads по возможности разделяются;
- native GPU fast path обязан сравниваться с portable backend на одинаковой telemetry.

## 10.2. Reusable GPU algorithms

Для `rcbt` и будущих reusable GPU subsystems:

- core semantic API не зависит от Bevy/wgpu/Vulkan;
- backend trait описывает operations/capabilities, а не thin wrappers над `Device`/`CommandEncoder`;
- wgpu/WGSL — portable default;
- native Vulkan/SPIR-V path допустим как optional backend при измеримой причине;
- subgroup/bit operations предпочтительнее искусственного использования matrix units для bit-tree work;
- cooperative/matrix hardware может использоваться optional decoder'ом compressed data только после отдельного benchmark;
- upstream/reference implementation можно bind'ить как oracle/benchmark baseline, но production path не обязан сохранять его internal layout;
- performance target может быть агрессивнее reference: observable semantics важнее внутренней compatibility;
- см. `docs/22_RCBT_GPU_TERRAIN.md`.

## 11. Лицензии

До решения по лицензии проекта:

- не добавлять AGPL dependencies в runtime;
- LGPL Rust dependencies требуют отдельного осознанного решения;
- не копировать код из `nyx-space` / `avian_fdm` в проект;
- их документацию/алгоритмические идеи можно использовать как reference с самостоятельной реализацией и первичными источниками.

## 12. Definition of done для physics feature

Фича не считается готовой без:

1. формального описания state/inputs/outputs;
2. теста известного частного случая;
3. regression case;
4. численной оценки ошибки;
5. benchmark хотя бы на target-size batch;
6. debug visualization/telemetry, если эффект невозможно проверить глазами иначе.


## 12. Licensing boundary

- engine/reusable crates: SPDX `MIT`;
- game apps/game-specific crates: SPDX `GPL-3.0-or-later`;
- не переносить GPL-only game code в MIT engine crates;
- permissive dependencies предпочтительны для engine; copyleft dependency в engine требует ADR с анализом redistribution/linking boundary;
- generated code/asset licenses сохраняются явно;
- см. `LICENSING.md`.
