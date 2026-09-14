# 04 — Runtime architecture


## 4.0. Cross-platform contract

Cross-platform — исходное ограничение, не post-release port.

- simulation, protocol, persistence и gameplay code не знают DirectX/Vulkan/Metal/wgpu;
- текущий native client shell — Bevy + wgpu, но это **integration boundary**, не обязательный API reusable GPU algorithms;
- reusable GPU subsystems (`rcbt` и аналогичные) определяют собственный semantic backend trait и не экспортируют Bevy/wgpu types из core;
- Linux/Vulkan, macOS/Metal, browser/WebGPU — first-class architecture targets;
- wgpu остаётся default portable backend; direct Vulkan backend допустим за render/backend adapter boundary, если profiling показывает measurable overhead или отсутствующую capability;
- Windows по умолчанию поддерживается через backend wgpu без DX-specific game code; wgpu может внутренне выбрать D3D12, но никакие DirectX types/calls не проходят за render adapter boundary;
- physics ray/BVH path не зависит от DXR/Vulkan RT;
- portable shader/kernel semantics предпочтительнее platform forks; WGSL — хороший default source, но measured native SPIR-V specialization допустима внутри native Vulkan backend;
- platform-specific optimization допускается только за feature/capability boundary и после профилирования.

Если позже принимается политика `Vulkan-only on Windows` или отдельный native Vulkan client path, это packaging/render-backend ADR, а не изменение simulation API.

---

## 4.1. High-level split

```text
                     ┌──────────────────────────────┐
                     │      native / web client     │
                     │ Bevy UI/ECS + render adapter │
                     │ wgpu default / native GPU opt│
                     └──────────────┬───────────────┘
                                    │ commands/snapshots
                                    ▼
┌────────────────────────────────────────────────────────────┐
│                     authoritative server                    │
│                                                            │
│ Tokio: network / persistence / admin / async I/O           │
│                   │                                        │
│                   ▼                                        │
│            simulation coordinator                          │
│                   │                                        │
│                   ▼                                        │
│      fixed CPU pool / Rayon / custom batches               │
│ gravity | aero | thermal | structure | factory | guidance  │
└────────────────────────────────────────────────────────────┘
```

Single-player может запускать server in-process или рядом отдельным process; semantic model остаётся server-authoritative.

Renderer implementation может меняться без изменения authoritative server/domain APIs.

---

## 4.2. Workspace boundary

Планируемая декомпозиция после prototype:

```text
crates/
  sim-core/          IDs, time, state, schedule, shared primitives
  sim-ephemeris/     baked celestial states / frames
  sim-orbit/         gravity, integrators, trajectory tools
  sim-flight/        6-DoF, atmosphere, aero, propulsion
  sim-structure/     structural graph, fracture
  sim-thermal/       thermal graph
  sim-factory/       production/logistics state
  sim-script/        event-driven VM
  vehicle-design/    parametric design + compiler
  protocol/          network protocol / snapshots / commands
  persistence/       save format / migrations
  graphics/          reusable render settings/metadata/adapters

  # terrain/GPU adaptive geometry, когда prototype boundary стабилизируется:
  rcbt-core/         backend-agnostic CBT logic/layout contracts
  rcbt-ref/          reference/oracle binding for tests/benchmarks only
  rcbt-wgpu/         portable GPU backend
  rcbt-vulkan/       optional native Vulkan backend
  bevy-rcbt/         thin Bevy integration only

apps/
  client/            native Bevy client shell
  server/            headless authoritative server
  web-client/        optional WASM packaging/features

tools/
  system-baker/      offline celestial integration/fitting
  physics-lab/       standalone test/debug scene
  benchmarks/        batch kernels
```

В v0.1 scaffold создан только минимальный subset, чтобы не делать архитектуру фиктивным количеством crates раньше кода. Имена выше — boundary target, а не требование немедленно дробить workspace.

---

## 4.3. Threading

### Server

Предпочтительная модель:

```text
Tokio runtime threads
  ├ network receive/send
  ├ persistence
  ├ HTTP/admin/metrics
  └ async orchestration

Simulation coordinator (dedicated thread or tightly controlled task)
  └ Rayon/custom pool
       ├ gravity batches
       ├ aero batches
       ├ thermal batches
       ├ structural batches
       └ planning batches
```

Не спамить `spawn_blocking` на каждую aero job. Tokio blocking pool оптимизирован под bounded blocking tasks и имеет большой default limit; для постоянного CPU-heavy workload нужен отдельный bounded pool.

### Client

Bevy schedules client state and current render integration. Background design compilation/trajectory previews могут использовать Bevy `AsyncComputeTaskPool`, но authoritative numerical core и reusable GPU algorithm cores от него не зависят.

GPU-driven subsystems должны по возможности получать compact frame inputs (`camera`, `error target`, resource handles), а не Bevy ECS types как persistent domain state.

---

## 4.4. Server tick

Simulation coordinator имеет явные фазы. Пример:

```text
1. collect commands for tick/time interval
2. advance celestial ephemeris handles
3. wake scheduled scripts/events
4. update factory/logistics discrete events
5. run active vehicle dynamics batches
6. contacts / fracture / topology changes
7. commit state transitions
8. generate snapshots/events
9. advance SimTime
```

Реальная последовательность может меняться после solver prototype, но должна быть детерминированной и документированной.

---

## 4.5. Warp scheduling

`SimTime` не привязан к wall clock.

Server выбирает chunk simulated time с учётом:

- requested shared warp;
- ближайшего alarm/event boundary;
- active atmospheric/contact vehicles;
- integrator error estimates;
- compute budget.

Важный принцип: **не обязательно глобально резать весь мир до 50 Hz**, если только один craft делает landing. Системы работают с собственными schedules, а coordinator синхронизирует causal boundaries.

---

## 4.6. Networking

### Authority

Client sends:

- pilot inputs;
- build commands;
- script edits;
- route/schedule commands;
- inventory/logistics intents where allowed;
- warp request/vote.

Client не сообщает серверу authoritative `position = ...` для управляемого craft.

### Replication tiers

Пример interest policy:

```text
local active bubble         high-rate transforms/state
same surface region         lower-rate + interpolation
same celestial body         coarse logistics/telemetry
other moon/planet           events + summaries
far BC/A-system             only subscribed telemetry/events
```

### Prediction

Controlling client может запускать тот же local flight model и rollback/correction. Remote craft обычно snapshot interpolation.

Lightyear 0.29 — сильный prototype candidate: Bevy 0.19 compatibility, prediction, interpolation, interest management, WebTransport/WASM.

---

## 4.7. Native / WASM

### Native client

Full feature set:

- local prediction;
- full 3D scene;
- editor;
- high-quality telemetry/debug;
- local server option;
- optional native GPU backend for isolated reusable subsystems when justified by profiling.

### WASM/Web client

Не обязан исполнять весь `sim-core`.

Target scopes по нарастающей:

1. system map / factory dashboard / alarms;
2. spectator client;
3. vehicle editor;
4. light gameplay;
5. full client only if performance/security/threads allow.

Bevy 0.19 официально демонстрирует browser examples через WASM + WebGPU; WebGL2 fallback остаётся полезным compatibility path. Web target не должен диктовать ограничения native simulation.

Portable wgpu/WebGPU path остаётся обязательным fallback для reusable renderer subsystems, даже если native Vulkan backend становится быстрее на desktop Linux.

---

## 4.8. Rendering coordinates

Authoritative:

```text
f64 inertial/body-centered coordinates
```

Renderer:

```text
render_pos_f32 = (sim_pos_f64 - render_origin_f64).as_f32()
```

Для surface scenes render origin следует за игроком/камерой. Для orbital/system map используются отдельные scale/frame representations.

Bevy hierarchy не должна зеркалить реальную celestial hierarchy буквально, если это мешает precision/culling.

---

## 4.9. Persistence

Save format должен хранить:

- content/system ephemeris version;
- simulation time;
- factories/buildings;
- vehicle designs (deduplicated by stable hash/ID);
- vehicle instance states;
- scripts;
- inventories;
- routes;
- alarms;
- damage/thermal state;
- RNG seeds/weather state;
- server rules.

Не хранить canonical positions планет как mutable save data: они восстанавливаются из `ephemeris_version + SimTime`.

---

## 4.10. Vehicle design deduplication

```text
DesignId(hash)
  ├ geometry
  ├ aero tables/model
  ├ structural graph template
  ├ thermal graph template
  ├ collision representation
  └ render assets

VehicleInstance
  ├ DesignId
  ├ rigid cluster states
  ├ fuel/cargo
  ├ damage
  ├ temperatures
  └ autopilot state
```

Это критично для fleet scale и network bandwidth.

---

## 4.11. Protocol design

Не сериализовать Bevy `Entity` IDs как network identity.

Использовать stable domain IDs:

- `VehicleId`;
- `DesignId`;
- `BuildingId`;
- `BodyId`;
- `RouteId`;
- `ProgramId`.

Protocol crate не зависит от renderer/client-only types.

---

## 4.12. Build targets

Proposal:

```text
client-native-avx2
client-native-avx512
server-avx2
server-avx512
client-wasm-webgpu
client-wasm-webgl2 (optional compatibility)
```

Optional measured renderer experiments may add a native Vulkan feature/build, but это не отдельная simulation/gameplay target family.

Bootstrap launcher на x86_64 может CPUID-select AVX2/AVX-512 binary. Никакой причины заставлять consumer Intel поддерживать отсутствующий AVX-512; это отдельный optimized target.

---

## 4.13. Dependency policy

### Accepted direction

- mixed repository licensing: engine MIT, game GPL-3.0-or-later;
- Rust edition 2024;
- Bevy 0.19.x client shell;
- wgpu through Bevy as default portable render backend;
- Tokio current 1.x server async;
- Rayon current 1.x compute;
- Serde for content/protocol prototypes;
- math crate compatible with Bevy adapter (initially glam 0.32 line).

### Candidate / spike first

- Lightyear;
- Parry f64;
- Avian f64 local contacts;
- native Vulkan backend for isolated reusable GPU subsystems;
- `rcbt` adaptive terrain stack (see `docs/22_RCBT_GPU_TERRAIN.md`).

### Reference / license-sensitive

- avian_fdm (LGPL-3.0-or-later);
- nyx-space (AGPLv3).

Новые foundational dependencies принимаются ADR после prototype measurement.

---

## 4.14. Observability

Server должен уметь отдавать breakdown:

```text
sim_time
requested/effective warp
active vehicles by regime
step counts
solver reject counts
physics ms by subsystem
factory event count
script wakeups
snapshot bytes/sec
interest sets
```

В debug client нужны force/aero/thermal/structural overlays. Без этого физический sandbox невозможно нормально отлаживать.

Reusable GPU subsystems дополнительно должны отдавать backend-neutral metrics: update/dispatch time, active elements, bytes touched/uploaded, working-set size и backend/capability selection. Сравнение wgpu/native backend без одинаковой telemetry не считается benchmark.


## 4.15. License/package boundary

Workspace package defaults не задают одну лицензию всему repository. Каждый package объявляет SPDX license сам.

```text
MIT:
  sim-*
  vehicle-design
  protocol (если остаётся generic)
  reusable tools/libraries
  reusable GPU algorithm crates / adapters where possible

GPL-3.0-or-later:
  client game app
  authoritative game server app
  game-specific content/rules crates
```

Game code может зависеть от MIT engine. Обратная зависимость запрещена. Это сохраняет engine пригодным для переиспользования вне GPL game.

---

## 4.16. Reusable GPU subsystem ownership

Низкоуровневый renderer algorithm не должен принадлежать Bevy только потому, что первый consumer — Bevy client.

Target layering:

```text
algorithm/core
    owns logical state, layouts, scheduling contracts, capability model
         |
         +--> portable backend (wgpu/WGSL)
         `--> optional native backend (Vulkan/SPIR-V)
                    |
                    v
             engine adapter (Bevy)
                    |
                    v
               game/client
```

Правила:

- core public API не возвращает `wgpu::Device`, `wgpu::Buffer`, Bevy `Entity`, `RenderWorld` и platform handles;
- backend trait описывает semantic operations/resources, а не является thin rename конкретного API;
- native backend не должен протекать в domain/gameplay code;
- engine adapter может быть удалён/заменён без переписывания logical algorithm;
- capability checks предпочтительнее vendor checks;
- native fast path обязан иметь portable fallback и repeatable benchmark;
- shader/kernel specialization допустима при одинаковых observable semantics.

Первый concrete subsystem с этим контрактом — `rcbt` для adaptive terrain (`docs/22_RCBT_GPU_TERRAIN.md`).
