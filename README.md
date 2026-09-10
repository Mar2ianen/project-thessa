# Project Thessa

> Factory, logistics and aerospace engineering tied together by one physical world.

[![CI](https://github.com/Mar2ianen/project-thessa/actions/workflows/ci.yml/badge.svg)](https://github.com/Mar2ianen/project-thessa/actions/workflows/ci.yml)
![Rust](https://img.shields.io/badge/Rust-1.95%2B-000000?logo=rust&logoColor=white)
![Engine license](https://img.shields.io/badge/engine-MIT-blue)
![Game license](https://img.shields.io/badge/game-GPL--3.0--or--later-blue)

**Project Thessa** — кроссплатформенный factory / aerospace sandbox на Rust.
Игрок начинает с промышленной инфраструктуры на спутнике газового гиганта,
строит локальную логистику, проектирует транспорт, выходит на орбиту и
постепенно связывает производством всю систему.

Главная идея: **орбитальная механика — часть логистики, а не отдельная
мини-игра**. Масса, объём, время перелёта, топливо, launch cadence, transfer
windows, atmosphere и управление аппаратом должны образовывать одну систему.

Проект находится в **pre-alpha / engineering prototype**. Форматы данных,
internal API и design numbers до `0.1.0` могут меняться без compatibility
promise.

---

## Что отличает Thessa

- **Одна физика вместо SOI-магии.** Корабли одновременно чувствуют гравитацию
  всех движущихся источников; sphere of influence остаётся UI/optimization
  concept, а не законом мира.
- **Factory + aerospace.** Интересный throughput находится не в бесконечном
  апгрейде conveyor tier, а в mass/volume, loading, launch vehicles, времени
  перелёта, окнах и инфраструктуре.
- **Aircraft и spaceplanes — first-class.** Атмосфера, AoA, Mach, q,
  control surfaces и 6-DoF dynamics находятся в том же simulation kernel,
  что и orbital flight.
- **Automation без отдельной “магической” физики.** Будущий autopilot,
  planner и scripting должны управлять теми же actuators и trajectory model,
  которыми пользуется игрок.
- **Linux-first, но не Linux-only.** Gameplay/simulation API не знает о
  DirectX; client построен на Bevy/wgpu. CI собирает и тестирует workspace на
  Linux, macOS и Windows.
- **Reference solvers — только validation.** Nyx/ANISE, JSBSim, RocketPy,
  AVL, VSPAERO, SU2 и OpenRocket не становятся runtime dependencies.

---

## Текущий vertical slice

| Подсистема | Что уже есть |
| --- | --- |
| Celestial runtime | deterministic baked ephemerides, multi-body test-particle gravity, Lagrange/reference validation |
| Integrators | adaptive Dormand–Prince 5(4), velocity-Verlet, deterministic rigid-body stepping |
| Aerodynamics | O(panels) analytic model, local flow, stall, transonic/supersonic corrections, dynamic damping, coefficient tables |
| Atmosphere | deterministic `T/p/rho`, viscosity, speed of sound, rotating-atmosphere boundary |
| Vehicle runtime | serializable geometry, mass/inertia, control surfaces, `vehicle-baker` |
| Pilot mode | Bevy PFD/navball, live X-15 test adapter, throttle/SAS/RCS/manual controls, flight tracing |
| Validation | Nyx/ANISE gravity/orbit harness, RocketPy apples-to-apples fin checks, JSBSim aircraft proxy checks |

### Последние numerical checks

| Сценарий | Результат |
| --- | ---: |
| L1–L3 circular restricted three-body residual | `8.8e-16–1.2e-18` |
| L4/L5 drift за 5 периодов | `2.8–3.0 mm`, около `9e-7 m/s` |
| 8 burns против Nyx DP7/8 | `0.52 m`, `5.49e-4 m/s` |
| Design system | 24 тела, 22 gravity sources |
| RocketPy fin-set `CL_alpha`, `M=0.95` | error `0.000002%` |
| RocketPy fin-set center of pressure | error `0.000%` |
| X-15-like 5 s proxy vs JSBSim | Mach `0.246%`, altitude `0.905%` |

X-15 comparison намеренно остаётся **proxy validation**, а не заявлением о
reference-grade X-15 fidelity: bundled JSBSim aircraft содержит полный корпус,
хвост, trim/control logic и табличные коэффициенты, которых нет у компактного
owned geometry proxy.

---

## Архитектура

```text
                      data/system.toml
                             │
                             ▼
                     thessa-system-baker
                             │
                             ▼
                   data/system.baked.json
                             │
                             ▼
┌────────────────────── thessa-sim-core ──────────────────────┐
│                                                             │
│  baked ephemerides ──► multi-body gravity                   │
│          │                       │                           │
│          └──────────────┬────────┘                           │
│                         ▼                                    │
│                  vehicle dynamics                            │
│             ┌───────────┼───────────┐                        │
│             ▼           ▼           ▼                        │
│        atmosphere      aero      propulsion                  │
│             └───────────┴───────────┘                        │
│                         │                                    │
│                         ▼                                    │
│                 authoritative f64/SI                         │
└─────────────────────────┬───────────────────────────────────┘
                          │
               ┌──────────┴──────────┐
               ▼                     ▼
         apps/client             apps/server
         Bevy / wgpu          authoritative shell
```

### Жёсткие границы

- `crates/sim-core` не зависит от Bevy/Tokio и хранит authoritative state в
  `f64` / SI.
- `Transform` и UI telemetry не являются источником physics state.
- Runtime не вызывает CFD/astrodynamics reference solvers.
- Engine/tooling crates — MIT; game-specific applications — GPL-3.0-or-later.
- GPU-specific interfaces не протекают в gameplay/domain API.

Подробнее: [`docs/03_PHYSICS_ENGINE.md`](docs/03_PHYSICS_ENGINE.md),
[`docs/04_RUNTIME_ARCHITECTURE.md`](docs/04_RUNTIME_ARCHITECTURE.md) и
[`LICENSING.md`](LICENSING.md).

---

## Быстрый старт

Требуется Rust `1.95+`.

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets --all-features -- -D warnings
cargo test --workspace
cargo test -p thessa-sim-core --release
```

Запуск baker-ов:

```bash
cargo run -p thessa-system-baker -- \
  --input data/system.toml \
  --output data/system.baked.json

cargo run -p thessa-vehicle-baker -- \
  --input data/vehicles/example_aircraft.toml \
  --output /tmp/example_aircraft.baked.json
```

Запуск клиента:

```bash
cargo run -p thessa-client
```

### Pilot controls

| Клавиша | Действие |
| --- | --- |
| `M` | Map ↔ Pilot |
| `W/S` | нос вниз / вверх |
| `A/D` | yaw влево / вправо |
| `Q/E` | roll влево / вправо |
| `Caps Lock` | точное управление (25% команды) |
| `Shift/Ctrl` | throttle up/down |
| `Z` / `X` | full throttle / cutoff |
| `Space` | engine on/off |
| `T` / hold `F` | переключить SAS / временно инвертировать SAS |
| `R` / `G` | RCS / gear state |
| `V` | свободная / следящая камера |
| `` ` `` | сброс камеры |
| `Escape` / `F8` / `Pause` | пауза |
| `F1` | все подсказки и схема интерфейса |
| `F2` | скрыть / показать интерфейс |
| `F3` | дополнительная телеметрия |
| RMB / MMB / wheel | свободное вращение / pan / zoom |

Режим управления выбирается кнопкой справа сверху. Клик по скорости на навболе
переключает speed frame; клик по высотомеру — datum/AGL. SAS, RCS, шасси,
двигатель, камера, карта и пауза имеют кнопки; зелёный цвет означает включённое
состояние. Дополнительные данные скрыты по умолчанию.

Pilot/PFD contract: [`docs/10_PILOT_INTERFACE.md`](docs/10_PILOT_INTERFACE.md).

---

## Validation и benchmarks

Gravity/orbit reference harness:

```bash
cargo run --manifest-path validation/nyx-compare/Cargo.toml --release
```

Aero comparison harness без обязательных external packages:

```bash
cargo run --manifest-path validation/aero-compare/Cargo.toml --release
```

Для локального JSBSim/RocketPy comparison:

```bash
python -m venv .venv-aero
.venv-aero/bin/pip install jsbsim rocketpy

THESSA_AERO_PYTHON=.venv-aero/bin/python \
  cargo run --manifest-path validation/aero-compare/Cargo.toml --release -- \
  --require-external
```

Benchmarks:

```bash
cargo bench -p thessa-sim-core --bench gravity
cargo bench -p thessa-sim-core --bench aero
cargo bench -p thessa-sim-core --bench flight
```

CI выполняет formatting/Clippy, cross-platform build/tests, release sim-core
checks, benchmark compilation и isolated reference validation.

---

## Структура репозитория

```text
apps/client/             GPL Bevy client
apps/server/             GPL authoritative server shell
crates/sim-core/         MIT numerical/physics kernel
crates/protocol/         MIT shared protocol boundary
tools/system-baker/      MIT system TOML → baked JSON
tools/vehicle-baker/     MIT vehicle TOML → baked JSON
data/                    system, resources and vehicle design data
docs/                    design, physics, runtime, pilot and aero docs
validation/nyx-compare/  isolated astrodynamics reference workspace
validation/aero-compare/ isolated aero reference workspace
logs/flight-traces/      intentionally preserved diagnostic captures
```

### Документы, с которых стоит начать

1. [`docs/01_CELESTIAL_SYSTEM.md`](docs/01_CELESTIAL_SYSTEM.md) — система и стартовый мир.
2. [`docs/03_PHYSICS_ENGINE.md`](docs/03_PHYSICS_ENGINE.md) — physics contracts.
3. [`docs/04_RUNTIME_ARCHITECTURE.md`](docs/04_RUNTIME_ARCHITECTURE.md) — runtime boundaries.
4. [`docs/05_ROADMAP.md`](docs/05_ROADMAP.md) — dependency-ordered roadmap.
5. [`docs/07_AUTOPILOT.md`](docs/07_AUTOPILOT.md) — composable automation/autopilot direction.
6. [`docs/08_NUMERICAL_VERTICAL_SLICE.md`](docs/08_NUMERICAL_VERTICAL_SLICE.md) — current numerical slice.
7. [`docs/10_PILOT_INTERFACE.md`](docs/10_PILOT_INTERFACE.md) — PFD/navball/control contract.
8. [`docs/11_AERODYNAMICS.md`](docs/11_AERODYNAMICS.md) — aero fidelity tiers and validation.
9. [`CHANGELOG.md`](CHANGELOG.md) — notable changes.

---

## Roadmap

```text
M0  numerical kernel
      ↓
M1  controllable vehicle / propulsion / contact
      ↓
M2  aero + spaceplanes + thermal/structural systems
      ↓
M3  factory + local logistics
      ↓
M4  orbital/intermoon logistics + automation
      ↓
M5  server-authoritative multiplayer and fleet operations
```

Приоритет — не количество контента, а доказательство одного цельного loop:

```text
factory → logistics → vehicle design → flight → orbital logistics → factory
```

---

## Лицензирование

- reusable engine/tooling — [MIT](LICENSES/MIT.txt);
- game applications и game-specific code —
  [GPL-3.0-or-later](LICENSES/GPL-3.0-or-later.txt);
- assets, музыка, fonts и external validation data лицензируются отдельно.

Перед добавлением новой dependency см. [`LICENSING.md`](LICENSING.md).
AGPL/LGPL code не должен случайно протекать в MIT runtime boundary.

## Участие

Перед изменениями simulation code прочитай [`AGENTS.md`](AGENTS.md) и
[`CONTRIBUTING.md`](CONTRIBUTING.md). Для physics changes желательно добавлять
не только unit test, но и инвариант/reference vector, который объясняет,
**какую физическую ошибку этот тест не даёт вернуть**.

### Rocky world generator

The `dev/worldgen-rocky-tool` branch is integrated into this checkout.
`thessa-worldgen-rocky` builds with the workspace, and its 16384×8192 Thessa v2
texture is shared by the orbital map and flight view. Fetch LFS assets after
cloning (`git lfs pull`). The field generator remains an offline tool; the
current runtime surface mesh/contact boundary is still spherical.

```bash
cargo run -p thessa-worldgen-rocky -- check --manifest data/worldgen/thessa_demo.toml
cargo run --release -p thessa-worldgen-rocky -- export-client-texture --recipe data/worldgen/worldgen_recipe.toml --body-file data/worldgen/thessa_v02.toml --out /tmp/thessa-preview.png --width 2048
```

Flight instruments form a compact bottom dock. The SVG icon sources and their
4x PNG exports live in `assets/ui/flight/`; F1 includes their illustrated legend.
