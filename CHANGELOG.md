# Changelog

Все заметные изменения Project Thessa фиксируются здесь.

Формат следует [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), а версии
следуют Semantic Versioning настолько, насколько это применимо к pre-alpha
прототипу. До `0.1.0` совместимость внутренних API, форматов данных и save-файлов
не гарантируется.

## [Unreleased]

### Added

- Общая запечённая траектория свободного полёта и карты, таблица эфемерид,
  планировщик событий по времени симуляции; запекание вынесено в compute worker.
- Проверки точности coast на горизонте 200000 с и benchmark запекания / 256 кешей;
  длительность запекания видна в performance monitor.

- Интегрирована ветка rocky worldgen: генератор в workspace и общая
  текстура Thessa v2 16K для карты и полёта.
- Регрессия записанного spin-up на 581 с и проверка сохранения энергии
  и инерциального углового момента при быстром свободном вращении.

- Bevy 0.19 pilot/PFD vertical slice с отдельными `SURFACE`, `AIR`, `ORBITAL`
  и `TARGET` speed frames, datum/AGL altitude modes и динамическим navball.
- Client-local X-15 flight-test adapter, подключённый к общему `sim-core`
  6-DoF integrator, multi-body gravity и atmosphere/aero pipeline.
- KSP-like control modes: Mouse Aim, Navball/SAS, Rate Control и Direct/Raw.
- Low-overhead CSV flight tracing для воспроизведения плохих flight states.
- MIT aerodynamic runtime: aggregated panel model, local `omega × r` flow,
  compressibility, smooth stall, transonic/supersonic corrections, control
  surfaces, dynamic damping и optional coefficient tables.
- Deterministic atmosphere provider с `T/p/rho`, speed of sound и viscosity.
- Generic serializable `VehicleDefinition` и `vehicle-baker` TOML → JSON path.
- Isolated aero validation harness для JSBSim/RocketPy/VSPAERO/AVL/SU2/
  OpenRocket reference workflows.
- Sampled rigid-body duration API, позволяющий world adapter пересчитывать
  gravity/altitude/wind/control inputs на каждом deterministic substep.

### Changed

- Угловой шаг заменён на implicit midpoint с Cayley-поворотом: свободное
  вращение больше не получает численную прибавку энергии от explicit Euler.

- Pilot HUD приближен к KSP: плотный нижний блок приборов вокруг navball,
  21 векторная иконка и явные цветовые состояния кнопок; подсказки только в F1, extra telemetry по F3.
- M открывает карту, V переключает камеру, backquote сбрасывает её,
  F2 скрывает UI; камера свободно проходит полюса.
- X-15 использует общий starter asset, физический FBW allocator и фиксированные
  120 Hz; trace names уникальны и включают команды рулей/SAS target.

- X-15 imported GLB axes приведены к физическим vehicle axes во всех attitudes.
- KSP pitch/yaw/roll command mapping и SAS target response стабилизированы.
- Pilot altitude/speed telemetry теперь считается относительно reference body,
  а не из barycentric velocity/position напрямую.
- Aero analytic baseline использует finite-planform correction, swept normal
  Mach и отдельный supersonic wave-drag term.
- Thessa design target обновлён до `R=3200 km`, `g≈0.500 g`, `p0=1.20 bar`.
- CI разделён на cross-platform, quality, release-simulation и reference
  validation jobs; workspace проверяется на Linux, macOS и Windows.

### Fixed

- Shift+F12 переключает RT / raster без перезапуска на поддерживаемых GPU;
  deferred prepass сохраняется, поэтому геометрия не исчезает при выключении RT.
- Варп ограничен бюджетом CPU на кадр, без изменения физического шага;
  фактическая скорость показана в performance monitor. sim-core оптимизирован
  и в development-сборке клиента.

- Кеш траектории инвалидируется при изменении масс и орбит; Hermite-скорость
  согласована с производной положения. Проверяются границы таблицы эфемерид
  и столкновения с физическими телами без гравитационного влияния.
- Карта рисует только будущую часть coast, с ограниченным числом вершин.
- Готовые участки поверхности появляются постепенно, без ожидания всей очереди;
  смена LOD сохраняет старое покрытие до готовности замены.
- Бюджет локального рельефа распределяется по нормированной экранной ошибке;
  normal maps согласованы с текущей сеткой 32×32, лимит тайлов не превышается.

- Дрожание геометрии вдали от старта: pilot render origin теперь у аппарата.
- Ошибочный отрицательный lift slope горизонтального хвоста и направления
  рулей; W/S теперь нос вниз/вверх, A/D влево/вправо.
- SAS не возвращает к старому курсу при ручном развороте; ограниченные RCS
  тормозят до цели без качаний от насыщения регулятора в вакууме.
- Убраны ложные target/FPV placeholders, фиктивный AGL и обнуление отрицательного PE.

- Исправлены ошибки знака AoA/control channels, приводившие к неверной реакции
  X-15 на ручной pitch/yaw/roll.
- Убрано смешивание map-scale и metre-scale координат в pilot preview.
- Добавлены guards против non-finite/unbounded pilot flight states.
- Flight trace пишет достаточный набор state/control/force channels для поиска
  shaking и controller/aero regressions.
- Вращение атмосферы больше не смешивает inertial/reference-body axes с
  повернутыми vehicle axes: `omega` переводится в craft frame перед `omega × r`.

### Validation

- RocketPy apples-to-apples fin-set: `CL_alpha` error `0.000002%` и CP error
  `0.000%` при `M=0.95`.
- X-15-like 5 s / 100 Hz proxy против JSBSim: Mach error `0.246%`, altitude
  error `0.905%`; AoA остаётся proxy-gap и не используется для глобального
  тюнинга analytic model.
- Nyx/ANISE reference harness остаётся изолированным от runtime dependency
  graph.

## [0.0.1] - 2026-09-07

### Added

- Первый numerical vertical slice: baked deterministic ephemerides, multi-body
  test-particle gravity, adaptive Dormand–Prince 5(4) и velocity-Verlet.
- `system-baker`, reproducible system descriptor и первоначальная design system.
- Bevy hierarchical celestial map.
- MIT engine / GPL game licensing boundary и ADR/documentation baseline.
- Numerical validation против Nyx/ANISE и Lagrange-point reference vectors.

> Git tag/release для `0.0.1` пока не создан; changelog не притворяется, что
> существует release link, которого ещё нет.
