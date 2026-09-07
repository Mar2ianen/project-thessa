# Project Thessa

> Фабрика, логистика и космическая инженерия, связанные настоящей физикой.

**Project Thessa** — кроссплатформенный factory / aerospace sandbox. Игрок
сначала строит промышленность на спутнике газового гиганта, затем создаёт
транспортную сеть, выходит на орбиту и постепенно связывает экономику всей
системы. Орбитальная механика здесь — это throughput, latency и стоимость
логистики, а не отдельная мини-игра.

Статус: **pre-prototype / design baseline v0.1**. Числа и названия небесных
тел пока являются design target, а не окончательным canon.

## Что уже работает

Первый numerical vertical slice реализован на Rust:

- `sim-core` хранит authoritative state в `f64` и SI units;
- небесные тела используют детерминированные baked/on-rails Kepler ephemerides;
- корабль получает сумму гравитации от всех движущихся point-mass тел одновременно;
- есть adaptive Dormand–Prince 5(4) и fixed-step velocity-Verlet;
- `system-baker` читает `data/system.toml` и создаёт воспроизводимый JSON descriptor;
- импульсные `delta-v` можно задавать упорядоченным расписанием между coast-дугами;
- root runtime не зависит от Bevy, Tokio, DirectX или Nyx;
- Nyx/ANISE подключены только в отдельном reference-only validation harness.
- `apps/client` имеет Bevy 0.19 hierarchical system map: он создаёт все
  физические тела из того же `SystemConfig::bake()`/`BakedEphemeris`, а не из
  собственных orbital constants, и переключает локальные map scopes.

Последний сложный прогон:

| Сценарий | Результат |
|---|---:|
| L1–L3 в circular restricted three-body vector | residual `8.8e-16–1.2e-18` |
| L4/L5 за 5 периодов | `2.8–3.0 mm`, около `9e-7 m/s` |
| 8 орбитальных burns против Nyx DP7/8 | `0.52 m`, `5.49e-4 m/s` |
| Design system | 24 тела, 22 gravity sources |
| Thessa vehicle replay | 172800 s, 1221 accepted steps, 0 rejected |

`halo` в текущем design descriptor намеренно помечен как diagnostic
approximation: Borea имеет ненулевые eccentricity/inclination, а точный
elliptic co-orbital segment ещё не реализован.

## Архитектурные границы

```text
data/system.toml
        │
        ▼
system-baker ──► data/system.baked.json ──► sim-core
                                             │
                               ┌─────────────┴─────────────┐
                               ▼                           ▼
                       baked body states          test-particle gravity
                               │                           │
                               └─────────────┬─────────────┘
                                             ▼
                                  adaptive / Verlet solver
```

- **MIT engine crates**: reusable numerical kernel, data formats и tooling.
- **GPL game crates**: client, server и game-specific code.
- **Cross-platform**: simulation/gameplay API не знает о DirectX; client
  rendering boundary реализуется через Bevy/wgpu.
- **Physics first**: SOI switching не является источником физики; Lagrange
  regions должны следовать из полей и эфемерид.

## Быстрый старт

Из корня репозитория:

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets --all-features -- -D warnings
cargo test --workspace
cargo test -p thessa-sim-core --release
cargo bench -p thessa-sim-core --bench gravity
cargo run -p thessa-system-baker -- \
  --input data/system.toml \
  --output data/system.baked.json
cargo run -p thessa-client
```

Клиент открывает Bevy-карту всей design-системы, строит орбиты через
`sim-core::BakedEphemeris` и позволяет переходить к локальным картам Nereid,
Orthea, Vesper и B/C binary; серверный authoritative snapshot будет подключён
следующим шагом. Подробности, шкалы и управление:
[`docs/09_BEVY_VISUAL_SLICE.md`](docs/09_BEVY_VISUAL_SLICE.md).

Reference harness запускается отдельно, потому что его AGPL-зависимости не
должны попадать в runtime graph:

```bash
cargo run --manifest-path validation/nyx-compare/Cargo.toml --release
```

При первом запуске harness потребуется скачать его изолированные зависимости.

## Структура

```text
crates/sim-core/       MIT numerical kernel
crates/protocol/       MIT shared protocol types
apps/server/            GPL server shell
apps/client/            GPL client shell
tools/system-baker/     MIT TOML → baked JSON tool
validation/nyx-compare/ отдельный reference-only workspace
data/                   system and resource design targets
docs/                   architecture, physics, roadmap and ADRs
```

Документы для чтения:

1. [`docs/01_CELESTIAL_SYSTEM.md`](docs/01_CELESTIAL_SYSTEM.md) — небесная система и стартовый мир.
2. [`docs/03_PHYSICS_ENGINE.md`](docs/03_PHYSICS_ENGINE.md) — physics model и границы solver-а.
3. [`docs/04_RUNTIME_ARCHITECTURE.md`](docs/04_RUNTIME_ARCHITECTURE.md) — client/server и cross-platform boundary.
4. [`docs/05_ROADMAP.md`](docs/05_ROADMAP.md) — dependency-ordered roadmap.
5. [`docs/07_AUTOPILOT.md`](docs/07_AUTOPILOT.md) — composable autopilot graphs.
6. [`docs/08_NUMERICAL_VERTICAL_SLICE.md`](docs/08_NUMERICAL_VERTICAL_SLICE.md) — текущие формулы, тесты и точность.
7. [`LICENSING.md`](LICENSING.md) — граница MIT engine / GPL game.

## Roadmap

- **M0 — numerical kernel:** текущий срез, J2, гармоники, fitted ephemerides.
- **M1 — rocket physics lab:** 6-DoF vehicle, thrust, actuators, staging и
  telemetry.
- **M2 — aero / spaceplane:** panel forces, control surfaces, thermal и
  structural graphs.
- **M3+ — factory и logistics:** ресурсы, производство, транспорт, automation
  и server-authoritative multiplayer.

Сначала доказываем физический kernel и главный loop
`factory → logistics → aerospace → factory`, затем расширяем content.

## Лицензия

- reusable engine/tooling crates — [MIT](LICENSES/MIT.txt);
- game applications и game-specific code — [GPL-3.0-or-later](LICENSES/GPL-3.0-or-later.txt);
- assets, музыка и шрифты лицензируются отдельно.

Подробности и правила для новых зависимостей: [`LICENSING.md`](LICENSING.md).

## Участие

Проект пока на стадии прототипа. Перед изменениями simulation code прочитайте
[`AGENTS.md`](AGENTS.md) и [`CONTRIBUTING.md`](CONTRIBUTING.md). Не добавляйте
AGPL/LGPL-зависимости в MIT runtime без отдельного ADR.
