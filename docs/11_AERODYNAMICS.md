# 11 — Аэродинамический движок

Статус: **первый realtime vertical slice**, 2026-09-08.

Цель — не встроить CFD в каждый игровой кадр, а разделить задачу на три
fidelity tiers:

| Tier | Runtime | Назначение |
| --- | --- | --- |
| A | `PanelAeroModel`, O(panels), CPU | realtime forces/moments для обычного аппарата |
| B | `AeroCoefficientTable`, bilinear lookup | импорт поляр из VLM/CFD и дорогих офлайн-прогонов |
| C | JSBSim, AVL, VSPAERO, SU2, OpenRocket/RocketPy | независимая валидация, генерация таблиц, не runtime |

## Контракт

MIT crate `thessa-sim-core` получает:

```text
AeroState {
    velocity_body_mps,
    angular_velocity_body_rps,
}
AeroEnvironment {
    density_kg_m3,
    speed_of_sound_mps,
    dynamic_viscosity_pa_s,
    wind_velocity_body_mps,
}
AeroGeometry { panels: Vec<AeroPanel> }
```

Произвольный aircraft не запекается в один solver-specific класс. Его asset
описывается `VehicleDefinition`: `AeroGeometry`, `RigidBodyProperties` и
список `ControlSurfaceDefinition`. Asset можно сериализовать через serde,
собрать из editor/procedural данных и передать в тот же runtime API, что и
reference aircraft. Полярная таблица — опциональный asset конкретного
vehicle, а не часть `thessa-sim-core` и не обязательное условие для полёта.
Минимальный TOML-to-JSON pipeline уже есть в `thessa-vehicle-baker`, пример
лежит в `data/vehicles/example_aircraft.toml`; он не ограничивает число
панелей, каналов или форму аппарата.

Для aircraft flight points environment может быть получен из
`AtmosphereConfig::sample(altitude_m)` без ручного хардкода скорости звука.
Default provider использует ISA-подобные слои 0–47 км и возвращает
температуру, давление, плотность, вязкость и `a = sqrt(gamma*R*T)`.
`AtmosphereConfig::aero_environment` преобразует этот sample в тот же
`AeroEnvironment`, поэтому на высоте меняются одновременно dynamic pressure,
Reynolds и Mach. Это ещё не planetary composition model: для Thessa-worlds
нужны отдельные `R`, `gamma`, sea-level state и gravity per body.

Оси аппарата: `+X` вперёд, `+Y` вправо, `+Z` вверх. Для панели локальная
скорость строго следует контракту physics engine:

```text
v_local = v_vehicle - v_wind + omega × r_panel
q       = 0.5 * rho * |v_local|²
Re      = rho * |v_local| * chord / dynamic_viscosity
```

Силы складываются в body frame, момент считается как `r × F` относительно
центра масс. `AeroResult` возвращает aggregate force/moment без обязательной
allocation; `evaluate_detailed` дополнительно отдаёт разбор по панелям для
debug overlay, telemetry и настройки модели.

Знак `alpha` единый для runtime и coefficient tables: `alpha > 0` означает,
что нос аппарата выше набегающего потока (в body frame поток имеет компоненту
`-lift_axis`). Поэтому симметричная несущая поверхность с положительным
`lift_coefficient_sign` создаёт положительную подъёмную силу по своему
`lift_axis`; инвертированный стабилизатор задаётся отдельным знаком, а не
переворачивает общую конвенцию.

У панели отдельно задаются фактический `span_m`, эффективный aspect ratio,
sweep, `thickness_to_chord_ratio` и point приложения силы
(`center_of_pressure_body_m`). Для прямоугольной
панели `AeroPanel::new` заполняет эти поля из `area/chord`; для конических,
трапециевидных и body-mounted поверхностей используется
`AeroPanel::with_planform`. Для стабилизаторов есть отдельный
`lift_coefficient_sign`, поэтому инвертированная подъёмная сила не ломает
знак `AoA`. Поэтому размах не восстанавливается ошибочно из площади и root
chord.

## Что делает Tier A

Аналитическая модель намеренно компактная, но не статическая «drag-only»
заглушка:

- локальный `AoA` (положительный нос вверх относительно потока) и sideslip по
  каждой панели;
- `omega × r`, wind и различная скорость потока на удалённых панелях;
- линейный lift slope в subsonic области с Prandtl–Glauert-like ограничением;
- finite-planform lift slope через Diederich correlation с явным
  body-interference multiplier;
- непрерывное смешивание около `M=0.8…1.2`;
- supersonic lift slope с finite near-sonic cap;
- induced drag `k * CL²`;
- transonic wave-drag rise и linearized supersonic wave drag
  `C_D,wave = 4·alpha²/sqrt(M²−1)`;
- supersonic thickness wave drag для профилей с заданным `t/c`, включая drag
  при нулевом lift;
- swept-surface correction через Mach component normal to the leading edge;
- continuous bounded post-stall curve с `tanh` saturation;
- control-surface effectiveness через изменение эффективного `AoA`;
- static local pitching moment `Cm`, независимый от `r × F` центра давления;
- динамические derivatives по безразмерным `p/q/r` для roll/pitch/yaw damping;
- panel exposure `[0, 1]` как вход для будущего BVH occlusion/wake solver;
- optional coefficient table, clamped за пределами экспортированного домена.

В analytic baseline используется `2π/rad` для 2-D flat-plate slope и
`β=0.6` как transonic floor. Это сохраняет конечный slope около `M=1` и
совместимо с Barrowman/RocketPy convention; supersonic branch продолжает
тонкокрылый `4/sqrt(M²-1)` trend с плавным переходом после `M=1`.

Для swept surfaces при `M>1` shock-forming branch использует
`M_n = M*cos(sweep)`. Если `M_n <= 1`, поверхность остаётся на конечной
transonic branch даже при сверхзвуковом полном freestream Mach. Это позволяет
одной analytic model обслуживать и прямое крыло, и swept/delta-like aircraft;
точные zero-lift body wave drag и nonlinear shock interactions всё равно
должны приходить из Tier B поляр.

Это инженерная reduced-order модель. Она не обещает точные shock position,
separation bubbles, hypersonic chemistry, boundary-layer transition или
aeroelastic coupling. Для таких эффектов используется Tier C, после чего
коэффициенты можно запечь в Tier B.

## Почему не hardware RT в authoritative solver

Ray tracing хорошо подходит для visibility/occlusion, plume impingement,
thermal view factors и локальных сенсоров. Он не должен быть обязательным
источником сил: GPU backend, precision и порядок редукции различаются между
машинами, а dedicated server может быть без GPU. Поэтому canonical path —
CPU `f64`; Rayon распараллеливает независимые vehicles и сохраняет порядок
результатов. Один небольшой craft не дробится на множество задач: overhead
параллельности может быть дороже panel loop.

Рекомендуемый budget после подключения vehicle integrator:

- 50–100 Hz aero sample для active atmospheric vehicle;
- adaptive substeps при rapid attitude change, max-Q, stall и landing;
- 8–64 aggregated panels для обычного craft;
- 128+ панелей только для крупных/специальных аппаратов или offline bake;
- 1 / 16 / 256 / 1024 vehicles — обязательные benchmark sizes.

Добавлен `benches/flight.rs`: 256 независимых vehicles × 16 panels, один
6-DoF шаг `10 ms`, Rayon сохраняет порядок результатов. На текущей машине
последний release-прогон занял `84.484 µs` на весь batch/frame (`42.242 ms`
за 500 итераций). Это ориентир одной конфигурации, а не гарантированный FPS:
следом нужны 1/16/1024 размеры, table path и server-side contention.

## Tier B: импорт поляр

`AeroCoefficientTable` хранит монотонные сетки Mach × AoA и samples
`(CL, CD, CY, Cm)`. В runtime выполняется bilinear interpolation, а вход за
границей clamped к краю таблицы. Это безопасный default для внешних VLM/CFD
данных: solver не вызывается из игрового кадра и не влияет на license boundary.

`AeroCoefficientTable::from_csv` принимает обычный CSV с колонками
`mach,alpha_deg,cl,cd,cy,cm`; это позволяет офлайн-конвертеру снять поляр с
JSBSim/VSPAERO/SU2, проверить provenance и положить результат рядом с
vehicle asset. Валидационный harness уже выполняет такой цикл для X-15:
четыре точки JSBSim импортируются в table path, а повторный runtime расчёт
даёт `CL/CD/Cm` с нулевым измеренным расхождением в точке `M=2, AoA=5°`.
Это не означает, что любая произвольная геометрия автоматически получает
такую точность: таблица действительна только для своей geometry/reference
area, Reynolds/atmosphere и диапазона управления.

Следующая версия таблиц должна добавить независимые измерения `beta`,
control deflection, Reynolds и dynamic derivatives (`Cl/Cm/Cn` по `p/q/r`).
До этого control effectiveness и side-force slope являются explicit
reduced-order parameters, а не скрытой магией.

## Reference matrix

Сравнивать нужно только одинаковые reference area/length, body axes,
atmosphere, Mach/Reynolds, mass properties и sign conventions.

| Reference | Что сравниваем | Где применяется |
| --- | --- | --- |
| JSBSim (C++, LGPL) | nonlinear 6-DoF, configurable coefficient functions/tables, forces/moments, atmosphere | aircraft/rocket trajectory and attitude response |
| AVL (MIT, source distribution) | thin lifting surfaces, low-angle `CL/CD/Cm`, stability derivatives, trim/eigenmodes | subsonic aircraft baseline; не stall/hypersonic oracle |
| OpenVSP/VSPAERO (NASA OS agreement) | VLM/panel geometry, polars, stability derivatives, supersonic delta-wing cases | fast offline aircraft/spaceplane table generation |
| SU2 (open-source CFD) | compressible Euler/RANS forces, moments, `Cp`, transonic/supersonic flow | expensive single-point/sweep validation |
| OpenRocket (GPL) | model rocket 6-DoF trajectory, Barrowman surfaces, drag curves, staging | rocket integration and trajectory cross-check |
| RocketPy (MIT) | rocket 3/6-DoF, Barrowman/custom coefficient curves, atmosphere and flight logs | rocket coefficient/table cross-check |
| NASA CRM | common aircraft geometry and published validation data | same geometry through VSPAERO/SU2 and imported table |
| NASA Space Shuttle Aerodynamic Data Book | Orbiter six-component operational aero data across entry/landing regimes | shuttle-like lifting-body table and range sanity |

Источники и license notes находятся в `docs/REFERENCES.md`. В репозиторий не
копируются сторонние model files или coefficient dumps без отдельной проверки
лицензии. `data/aero/reference_cases.toml` содержит только provenance и
параметры собственных proxy-vectors.

## Validation vectors

`validation/aero-compare` выполняет воспроизводимый local run для:

1. low-angle finite wing proxy, subsonic;
2. axial rocket-like proxy at `M=0.95`;
3. lifting-body/shuttle-like proxy at `M≈5.3` and `AoA=20°`.

Команда:

```bash
env RUSTC=/usr/bin/rustc PATH=/usr/bin:/bin \
  /usr/bin/cargo run --manifest-path validation/aero-compare/Cargo.toml --release
```

Harness пробует найти локальные `jsbsim`, `vspaero`, `avl`, `openrocket` и
сообщает `external_engines=none`, если они не установлены. При доступном
Python-окружении он уже запускает адаптеры JSBSim для bundled `737`, `X15` и
`Shuttle`, а также RocketPy для Barrowman fin-set. В CSV печатаются
нормализованные `CL`, `CD`, `Cm`, ошибки в абсолютных единицах и процентах.

JSBSim rows сейчас являются proxy-gap измерением: его полноценные bundled
aircraft не совпадают с компактной геометрией Thessa. RocketPy fin-set имеет
одинаковые размеры с четырьмя panel primitives и потому является первым
apples-to-apples coefficient check. Он прогоняется по Mach sweep от `0.3` до
`5.0`; `--require-external` превращает отсутствие внешнего solver/package в
failure.

Последний прогон после геометрической поправки:

| Сравнение | Thessa | Reference | Ошибка |
| --- | ---: | ---: | ---: |
| RocketPy fin-set `CL_alpha` at `M=0.95` | `2.832937757` | `2.832937809` | `0.000002%` |
| RocketPy fin-set `CP` | `1.278571 m` | `1.278571 m` | `0.000%` |

Эти две строки являются apples-to-apples проверкой: совпадают root/tip chord,
span, sweep, fin count, body radius и reference area. JSBSim proxy rows
остаются диагностикой границы reduced-order модели, а не target для глобальной
подгонки.

В Mach sweep для этого fin-set ошибка `CL_alpha` остаётся практически нулевой
на `M=0.3…0.95`, составляет `2.25%` при `M=1.1`, `1.45%` при `M=1.2`, затем
возрастает до `10.52%` при `M=2` и `20.54%` при `M=5`. Причина последнего —
RocketPy продолжает Barrowman/Prandtl-коррекцию, а Thessa учитывает нормальный
Mach swept surface и отдельный тонкокрылый supersonic law. Для аппарата, где
нужны именно reference-grade полёры, следующим слоем должны быть импортированные
Mach/AoA tables, а не очередная глобальная подгонка analytic branch.

Для X-15-like aircraft proxy добавлен JSBSim coefficient sweep по
`M=0.95, 1.1, 1.2, 1.5, 2, 3, 5` при `alpha=5°`. На `M=5` ошибка `CD`
составила `8.49%`, но `CL` — `52.17%`: это честно показывает границу
одно-панельной модели, потому что bundled X-15 учитывает корпус, хвост и
табличные коэффициенты, которых у proxy нет. Такой результат не используется
для глобальной подстройки runtime.

В harness добавлен и короткий связный 6-DoF прогон X-15-like proxy: старт на
24.384 км, `M=2`, `AoA=5°`, 5 секунд, 100 Гц с ограничением внутренних шагов
`10 ms`. Наш proxy получил `M=1.9700`, `h=24506.4 m`, `AoA=4.7433°`; bundled
JSBSim X15 — `M=1.9749`, `h=24286.7 m`, `AoA=0.5905°`. Mach расхождение
составило `0.246%`, высоты — `0.905%`, но AoA — `703%`: это ожидаемый
proxy-gap из-за отсутствия полного корпуса, хвоста, trim/управляющих каналов и
JSBSim coefficient tables в нашей owned geometry. Этот прогон является
диагностикой интегратора, а не утверждением reference-grade X-15 fidelity.

## Acceptance criteria

- zero flow/vacuum → zero aero force and moment;
- drag opposes local relative velocity, positive AoA produces signed lift;
- force scales with `q`, including `omega × r` contribution;
- rotating atmosphere contributes `omega_body × position_body` to air velocity;
- no NaN or discontinuity-induced blow-up at `M≈1` and `M>1`;
- bounded duration integration is deterministic under smaller max steps;
- panel forces sum deterministically and batch output order is stable;
- every imported table records source, geometry, reference area/length,
  atmosphere, Mach/Reynolds grid and sign convention;
- external comparison reports numerical error separately for `CL`, `CD`,
  moments and integrated trajectory — one scalar score is insufficient.

## Известные ограничения и следующий срез

Сейчас есть первый full-state 6-DoF rigid vehicle integrator с semi-implicit
translation, implicit-midpoint angular dynamics, парным Cayley quaternion
update, atmosphere rotation coupling и optional dynamic p/q/r damping.
Свободное вращение сохраняет энергию и инерциальный угловой момент;
регрессия записанного spin-up проверяется через production flight path. Но ещё отсутствуют изменяемые mass/CoM в одном шаге,
automatic editor-to-panel meshing, BVH exposure solver, real control-surface
hinge torque, aeroelasticity, ground effect, propwash, hypersonic real-gas
chemistry и automatic external CSV provenance adapters. Это сознательная
граница reduced-order runtime; CFD не добавляется в игровой кадр.

Следующий порядок работ: загрузчик `VehicleDefinition` из asset-файлов →
изменяемые mass/CoM updates → control-surface hinge/actuator coupling →
metadata-bearing generated coefficient tables → BVH exposure/wake →
CRM/shuttle/rocket apples-to-apples tables → offline hypersonic real-gas/
thermal coupling.
