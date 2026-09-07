# 08 — Numerical vertical slice

Статус: **implemented prototype**, 2026-09-07. Это не canonical celestial ephemeris: `data/system.toml` всё ещё имеет статус design target.

## Что реализовано

- `thessa-sim-core` остаётся MIT и не знает о Bevy, Tokio или DirectX.
- `SimTime` — явное время симуляции в секундах; authoritative vectors — `glam::DVec3`/`DQuat` (`f64`).
- `ReferenceFrame` и `StateVector` маркируют system-barycentric, body-centered/body-fixed, local tangent, vehicle и render-local frames.
- `KeplerOrbit` решает elliptic Kepler equation и возвращает положение/скорость в произвольный момент без wall-clock состояния.
- `BakedEphemeris` строит иерархическую deterministic on-rails картину: система A–BC, внутренний B–C binary, планеты, луны и minor bodies.
- `GravityField` суммирует point-mass gravity от всех физических тел одновременно. Синтетические `system_barycenter`/`bc_barycenter` не дублируют вклад их компонентов.
- `accelerations` использует Rayon для batch-evaluation с сохранением порядка входных состояний и фиксированным порядком источников.
- `propagate_adaptive` — embedded Dormand–Prince 5(4) с раздельными tolerance для метров и метров/секунду.
- `propagate_velocity_verlet` — fixed-step coast solver с bounded energy error в static conservative field; для движущихся baked sources он только второго порядка, не строго symplectic.
- `thessa-system-baker` читает TOML, валидирует граф host/parent, печатает epoch sample и по `--output` пишет JSON формата 1.

## Формальный контракт

Для test particle `x, v` и `SimTime t`:

```text
dx/dt = v
dv/dt = sum_i mu_i * (body_i(t).position - x) / |body_i(t).position - x|^3
```

`body_i(t)` вычисляется из baked analytic segments. SOI switching отсутствует. Вклад ship-to-ship gravity, J2/Jn harmonics, collisions, aero, thrust и thermal coupling намеренно не включены в этот первый срез.

`SystemConfig::bake` использует design periods только как diagnostic metadata. Mean motion выводится из `mu` и relative semi-major axis; для component-orbits бинарных систем используется общий relative mean motion. Отсутствующие phase angles v0.1 детерминированно принимаются равными нулю.

## Проверки

`sim-core` содержит regression tests для:

1. круговой двухтельной орбиты за аналитический период;
2. эксцентричной двухтельной орбиты за аналитический период;
3. bounded specific-energy error для 50 периодов velocity-Verlet;
4. restricted three-body L4 residual;
5. обмена heliocentric energy в поле движущегося secondary;
6. exact replay и order-preserving parallel batch gravity;
7. явной маркировки reference frame;
8. детерминированного ordered schedule из импульсных delta-v.

`system-baker` отдельно парсит реальный `data/system.toml`, получает 24 тела и 22 физических gravity sources и проверяет epoch state. Generated descriptor лежит в `data/system.baked.json`; он является воспроизводимым build output, не окончательным game canon.

### Внешний reference cross-check

Добавлен изолированный `validation/nyx-compare` workspace. Он использует
`nyx-space 2.5.3` с `ANISE 0.10.6`, но не является частью root workspace и не
попадает в runtime dependency graph. Для одинакового начального Cartesian
state vector сравниваются:

- Nyx Dormand–Prince 7/8 `Propagator` с `OrbitalDynamics::two_body()`;
- наш `sim-core` Dormand–Prince 5(4) в SI `f64`;
- отдельный analytic diagnostic через ANISE `Orbit::at_epoch` и собственный
  `KeplerOrbit` oracle.

Команда:

```bash
/usr/bin/cargo run --manifest-path validation/nyx-compare/Cargo.toml --release
```

Последний прогон системным Arch Rust 1.98.1 дал расхождение candidate против
Nyx numerical propagator от `1.4e-3 m` до `1.69 m` и от `9.7e-8 m/s` до
`1.29e-3 m/s` на 6 h–7 d LEO/GEO coast cases. Входные state vectors совпали
с собственным `KeplerOrbit` oracle лучше `1.1e-8 m` и `1.1e-12 m/s`.

В том же прогоне ANISE `Orbit::at_epoch` дал около `1.6e7 m` ошибки только на
наклонном LEO кейсе, тогда как Nyx numerical propagator и собственный oracle
сошлись. Этот путь оставлен диагностическим и не используется как pass/fail
gate до отдельного разбора версии ANISE.

### Сложные сценарии

В тот же reference-only harness добавлены отдельные проверки:

- все пять точек Lagrange в circular restricted three-body test-векторе;
- пять периодов движения вокруг L4 и L5 в полном inertial gravity field;
- восемь импульсных манёвров с coast-дугами между ними, сравнённых с Nyx
  DP7/8;
- реальный `data/system.toml`: 24 тела, 22 gravity sources, конечность
  состояний на 30 суток и 8-burn replay аппарата вокруг Thessa.

Результат последнего прогона: L1–L3 residual `8.8e-16–1.2e-18`, L4/L5 после
пяти периодов — `2.8–3.0e-3 m`, а 8-burn Earth-like sequence против Nyx —
`5.18e-1 m` по позиции и `5.49e-4 m/s` по скорости.

Для design-системы `halo` теперь получает правильный бинарный `mu` пары
Nereid–Borea. Но текущая конфигурация Borea имеет `e=0.005` и наклонение
`0.20°`, тогда как `halo` пока задан круговым и coplanar; поэтому pair-only
L4 residual остаётся `1.99e-2` и печатается как
`diagnostic-approximation`. Это ограничение текущего Kepler-segment baker, а
не pass/fail gate точного Lagrange equilibrium.

Команды для актуального toolchain:

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets --all-features -- -D warnings
cargo test --workspace
cargo test -p thessa-sim-core --release
cargo bench -p thessa-sim-core --bench gravity
cargo run -p thessa-system-baker -- --input data/system.toml --output data/system.baked.json
```

На машине, где stable Rust младше объявленного `rust-version = 1.95`, к отдельным командам временно добавляется `--ignore-rust-version`; это обход проверки версии Cargo, а не изменение численной семантики. Full workspace также зависит от Bevy 0.19.1, который требует Rust 1.95.

## Точность и ограничения

- Аналитический baker даёт machine-repeatable evaluation одной и той же сборки, но не обещает долгосрочную физическую устойчивость design-конфигурации.
- Adaptive solver контролирует локальную embedded error estimate; это не глобальная гарантия ошибки и не замена long-horizon validation.
- Verlet тестируется на conservative central field; при движущихся источниках поле time-dependent, поэтому сохраняемая величина не обязана быть постоянной.
- Floating-point replay стабилен при одинаковом бинарнике, source order и Rayon result order. Cross-ISA bit identity пока не обещается.
- Прямой `nyx-space` dependency не добавлялся: независимые analytic/reference cases сохраняют MIT runtime boundary. Внешний high-fidelity cross-check остаётся отдельной validation harness задачей.

## Следующие шаги

1. Добавить J2 и общий spherical-harmonics API с body-fixed frame и nodal-precession tests.
2. Заменить design Kepler segments на versioned fitted Chebyshev/Hermite segments после offline n-body relaxation.
3. Добавить hyperbolic/parabolic segment support и event/encounter step policy.
4. Разобрать расхождение ANISE `Orbit::at_epoch` на наклонных орбитах и
   зафиксировать версию/набор reference vectors для CI.
5. Добавить dedicated elliptic Lagrange/coorbital segment с общей фазой и
   eccentricity, затем заменить diagnostic approximation для `halo` точной
   моделью.
6. Подключить vehicle state, thrust/actuator interfaces и server-side batch
   scheduling, не перенося game/GPL code в MIT crate.
