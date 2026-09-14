# 23 — Baked gravity hierarchy and target-cohort field cache

Статус: **architecture / performance design target**.

Эта дока фиксирует следующий major gravity optimization layer поверх уже существующих baked ephemerides, adaptive integration и SIMD-oriented ephemeris tables.

Главная идея: runtime не должен каждый раз отвечать на вопрос

> «какова сумма гравитации всех тел в этой точке прямо сейчас?»

с нуля для каждого аппарата и каждого RK stage, если много объектов находятся в одной области пространства и времени, а дальние источники образуют гладкое поле.

Вместо этого нужны две независимые и совместимые формы sharing:

```text
source-side sharing
    baked astronomical hierarchy / multipoles

           +

target-side sharing
    local field patch per spatial-temporal cohort
```

Первая уменьшает число источников, которые нужно раскрывать. Вторая позволяет сотням близких targets переиспользовать уже скомпилированное локальное поле.

Это **не SOI**, не patched conics и не «грузовые корабли получают дешёвую физику». Все физические источники продолжают существовать; approximation разрешается только при bounded error.

---

## 1. Current baseline and missing sharing

Текущий `GravityField::acceleration(position, time)` проходит по всем gravity sources, получает `body_state`, затем для каждого источника считает distance, reciprocal distance cubed и суммирует point-mass acceleration.

`EphemerisTable` уже решает важную часть стоимости:

- sample'ит source motion один раз на horizon;
- хранит component-major position/velocity arrays;
- интерполирует source states вместо повторных Kepler solves;
- имеет SIMD-friendly layout;
- snapshot'ит все body centers на выбранный epoch.

Но даже после этого каждый target отдельно повторяет source accumulation.

То есть текущий conceptual hot path остаётся близким к:

```text
for target in targets:
    for source in gravity_sources:
        evaluate source center
        dx = source - target
        r2 = dot(dx, dx)
        target.g += mu * dx / r^3
```

Adaptive tick integration уменьшает количество accepted time steps, но каждое оставшееся RK force evaluation всё равно платит за source set.

Experimental `gravity_interpolation` уже исследует temporal force reuse. Новый дизайн не заменяет эту работу, а делает approximation source-aware и target-shareable.

---

## 2. Design goals

Основные цели:

- materially снизить gravity cost для fleet-scale simulation;
- сделать `100–300` близких freighters намного дешевле `100–300 × one craft`;
- сохранить exact/authoritative fallback;
- использовать одну и ту же bounded approximation semantics в simulation, trajectory planning и autopilot candidate search;
- сначала получить сильный CPU-only path;
- оставить GPU/other accelerators как optional backend later;
- не привязывать correctness к классу аппарата, vendor'у CPU/GPU или observer state.

Target gameplay workload:

```text
100–300 freighters
    wait for similar launch windows
    depart from same depot/moon
    fly in a relatively compact group
    share destination and much of the trajectory geometry
```

Это почти идеальный target-side sharing case.

---

## 3. Source hierarchy is part of celestial representation

В Thessa astronomical hierarchy известна заранее и стабильна по topology. Поэтому generic Barnes–Hut tree, перестраиваемый каждый tick, не нужен.

Пример logical tree:

```text
Asterion system root
├─ A branch
│  ├─ Asterion A
│  ├─ Khepri
│  ├─ Nereid system
│  │  ├─ Nereid
│  │  └─ moons ...
│  ├─ Orthea system
│  └─ Vesper system
│
└─ BC branch
   ├─ Asterion B
   ├─ Asterion C
   └─ Janus system
```

Inner node — не fake body. Это aggregate representation физически существующих children.

Candidate node data:

```rust
pub struct GravitySourceNode {
    pub mu: f64,
    pub child_range: Option<ChildRange>,

    // baked/time-varying tracks
    pub barycenter_track: TrackId,
    pub radius_bound_track: TrackId,
    pub quadrupole_track: Option<TrackId>,

    // conservative metadata for opening/error tests
    pub max_internal_radius_m: f64,
    pub max_quadrupole_norm: f64,
}
```

Точная storage форма определяется baker/runtime representation.

---

## 4. Bake the hierarchy because the source trajectories are already baked

Поскольку canonical celestial trajectories уже deterministic/baked, source hierarchy тоже можно подготовить offline.

Для каждого internal node можно заранее получить как функции `SimTime`:

```text
mu_total
barycenter(t)
internal radius bound(t)
quadrupole(t)
optional higher conservative bounds
```

Для пары B+C это означает, что runtime не обязан сначала вычислить B(t), C(t), а потом каждый раз собирать aggregate заново.

Можно хранить эти tracks через тот же класс representations, который применяется к ephemerides:

- Hermite segments;
- Chebyshev segments;
- uniform sampled tracks where cheaper;
- bounded interpolation error metadata.

Source-tree topology должна быть versioned вместе с system ephemeris/content version.

---

## 5. Far-field representation

Для достаточно далёкого source node первый approximation — monopole at barycenter:

\[
\mathbf g(\mathbf x)=\mu\frac{\mathbf r}{|\mathbf r|^3}.
\]

Если monopole error budget уже недостаточен, следующий natural correction — quadrupole.

Barycentric expansion особенно удобна, потому что dipole term относительно barycenter исчезает.

Conceptual fidelity ladder:

```text
far
    monopole

closer
    monopole + quadrupole

nearer
    descend to children

very near
    exact body/source terms
```

Opening decision не должен быть hardcoded как `s/r < theta` only. Предпочтительно использовать conservative acceleration-error estimate:

```text
if estimated_node_error <= allocated_gravity_error_budget:
    accept aggregate node
else:
    descend
```

Классический geometric ratio может быть cheap prefilter, но final policy должна быть tied to physical/numerical error budget.

---

## 6. Target cohorts: share a local field, not one acceleration vector

Два близких аппарата не имеют строго одинаковую acceleration. Поэтому нельзя просто вычислить `g(center)` и отдать всем.

Но дальнее поле локально гладкое. Для target cohort вокруг anchor `x0`:

\[
\mathbf g(\mathbf x)
\approx
\mathbf g_0
+J(\mathbf x-\mathbf x_0)
\]

где `J` — gravity gradient / tidal tensor.

Для point-mass source:

\[
J = \mu\left(\frac{3\mathbf r\mathbf r^T}{r^5}-\frac{I}{r^3}\right).
\]

При необходимости следующий уровень:

\[
\mathbf g(\mathbf x)
\approx
\mathbf g_0
+J\Delta\mathbf x
+\frac12H[\Delta\mathbf x,\Delta\mathbf x].
\]

То есть одна expensive field compilation может обслуживать много targets через несколько FMA на объект.

Candidate runtime cache:

```rust
pub struct GravityPatch {
    pub center: DVec3,
    pub radius_m: f64,
    pub start: SimTime,
    pub end: SimTime,

    pub g0: DVec3,
    pub jacobian: DMat3,
    pub hessian: Option<GravityHessian>,

    // sources that cannot be safely absorbed into the shared approximation
    pub exact_sources: SmallVec<[BodyId; 4]>,

    // already accepted aggregate source nodes; avoids retraversal while valid
    pub accepted_nodes: Vec<GravityNodeId>,

    pub accel_error_bound: f64,
}
```

Exact type/layout is provisional.

---

## 7. Split common far field from individual near corrections

Most useful fleet representation is likely:

\[
\mathbf g_i=
\mathbf g_{far,patch}(\mathbf x_i,t)
+\sum_{j\in near}\mathbf g_j(\mathbf x_i,t).
\]

Example near Nereid:

```text
shared patch:
    Asterion A
    BC aggregate
    far planets/moon groups

per-craft exact terms:
    Nereid
    current nearby moon(s)
```

This keeps local encounter fidelity without forcing every far source through every target's exact loop.

If the convoy approaches another body, the source tree naturally opens and the exact-source list changes.

---

## 8. Cohort construction

A cohort is not «all cargo ships».

Class/role may define a requested error budget or scheduling policy, but spatial/temporal validity defines actual sharing.

Useful cohort dimensions:

```text
spatial region
simulation time span / integration lattice span
requested gravity error budget
integration regime / step schedule
```

Possible key direction:

```rust
pub struct GravityCohortKey {
    pub time_span: TickSpan,
    pub spatial_cell: GravityCellId,
    pub accuracy_class: GravityAccuracyClass,
}
```

But implementation should avoid locking into a grid if bounding spheres/clusters work better.

A convoy can start as one cohort:

```text
compute center + radius
compile patch
```

If error bound fails because the group stretches or enters a high-gradient region:

```text
cohort
  -> split spatially
  -> rebuild/refresh child patches
```

If neighboring cohorts later converge and share compatible time/error ranges, merge is allowed but not required for first implementation.

---

## 9. Error-driven validity, not vehicle-class cheats

A patch is valid while both spatial and temporal remainder bounds stay within budget.

Conceptually:

```text
spatial error
    higher-order field terms over cohort radius

+

temporal error
    source motion / multipole evolution over patch interval

<= allocated gravity error budget
```

The system may have named accuracy classes for ergonomics, but they are requests, not guarantees of approximation acceptance.

Example:

```text
active landing
    tiny budget

normal free flight
    medium budget

on-rails / long coast
    larger budget

planner broad search
    intentionally loose first pass
```

An on-rails freighter passing close to Nereid must automatically refine/split/open sources if its requested bound cannot be satisfied.

---

## 10. Interaction with adaptive timestep

Temporal integration and field representation are separate adaptive axes:

```text
time adaptation
    how often target state is advanced

source adaptation
    how deeply source hierarchy is opened

target adaptation
    how large a target cohort can share one local field patch

cache adaptation
    how long that patch remains valid
```

Large integration steps do not automatically permit sloppy gravity, but the integrator can explicitly allocate part of its local error budget to field approximation.

A useful future contract is something like:

```text
integrator local error budget
    -> reserve state-integration component
    -> reserve field-approximation component
```

The gravity evaluator then returns both acceleration and a conservative approximation bound.

This avoids duplicated hidden tolerances.

---

## 11. CPU-only first implementation

First serious version is CPU-only.

Reasons:

- authoritative server is already CPU-first;
- target batch is initially `~100–300`, not millions;
- shared patch collapses expensive math into a tiny polynomial evaluation;
- CPU SIMD avoids device synchronization/data-residency complexity;
- profiling should prove that GPU is still needed after representation improvement.

Preferred target layout for cohort evaluation:

```text
positions SoA:
    x[]
    y[]
    z[]

shared:
    center
    g0
    J
    optional H

output SoA:
    ax[]
    ay[]
    az[]
```

This maps cleanly to AVX2/AVX-512 lanes.

Steady-state target:

- no allocation in hot evaluation loop;
- no per-target source-tree traversal while patch is valid;
- no per-target ephemeris interpolation for accepted far nodes;
- exact-near source loop remains short and SIMD/batch friendly where useful.

Cache padding/align tricks only after measured false sharing, consistent with project-wide performance policy.

### Source-count padding and narrow tails (open question)

Текущий каскад (`table.rs::accel_with`): 8-wide AVX-512 (`gravity_chunk`),
затем 4-wide AVX2 (`gravity_quad`), затем scalar. При 22 источниках хвост —
2 тела всегда скалярно (8+8+4+2). Открытые вопросы:

1. Падинг массива источников фиктивными негравитирующими телами до кратного
   8/4, чтобы каждый тик шёл полными лейнами без скалярного хвоста? Перед
   решением нужно закрыть:
   - ядра возвращают `None` на нулевой дистанции / NaN в ЛЮБОМ лейне, включая
     негравитирующие, а скалярный путь, наоборот, скипает их до проверки
     дистанции. Пад должен доказать, что фиктивный лейн никогда не триггерит
     `None` для произвольного таргета — иначе пад превращается в принудительный
     fallback на скаляр, то есть в пессимизацию;
   - нулевой вклад побитово: `total + 0.0` меняет знак `-0.0`, а горизонтальная
     сумма внутри ядра идёт в другом порядке, чем скалярное накопление. Либо
     доказать отсутствие влияния, либо явно зафиксировать, что SIMD batch-путь
     не обязан быть побитово идентичен скаляру (и чем расхождение покрыто);
   - кто владеет падингом: бейкер (версионированная раскладка, ноль копий в
     рантайме) vs рантайм (копия+пад на снапшот — цена копии против цены хвоста).
2. Более узкие инструкции для маленьких хвостов вместо скаляра: 2-wide
   128-бит (SSE2 `_mm_*_pd`, есть везде на x86-64) закрыл бы ровно хвост
   22 = 8+8+4+2. Альтернатива — AVX-512 masking (k-маски) одним частично
   активным опом вместо каскада 8→4→scalar. Цена — рост dispatch-поверхности
   и ещё один путь для валидации.
3. Сначала измерить долю хвоста: при 22 источниках скалярно идут не более 2 из
   22 (~9% источников при другой цене за источник). Решение — только по замеру
   хвоста против цены копии/пада и сложности масок.

   Измерение (Zen 5, native, kernel-level, 22 источника, один eval): каскад
   8+8+4+2scalar — 66 нс; padded-to-24 3x8 — 53 нс; pure scalar — 224 нс.
   Хвост (2 скалярных лейна) ≈ 13–19 нс, ~20% времени ядра. Предварительный
   вывод: падинг экономит ~20% времени ядра накопления, но exact-near split
   (§8) снимает вопрос иначе — горячим циклом становятся 1–4 точных источника,
   которые 4-wide ядро покрывает без хвоста вообще. Падинг/маски отложить до
   замера нового горячего цикла после реализации §8.

   Update (второй проход): frame/cohort kernels стали строго serial по
   измерению (диспетчеризация дороже математики до ~10k таргетов), так что
   вопрос падинга/масок жив только для table-snapshot пути (`accel_with`),
   где каскад 8→4→scalar остаётся. Там решение — по-прежнему замер хвоста
   против цены копии/пада, отдельно от флотового тика.

---

## 12. Autopilot and trajectory planner reuse

The same representation can accelerate trajectory search even more aggressively than authoritative flight.

Typical planner workload:

```text
same start epoch
same start region
same celestial configuration
hundreds/thousands of candidate burns
```

Instead of compiling gravity independently for every candidate:

```text
build GravityPatch(time interval, region, error budget)

candidate 1 ┐
candidate 2 ├─ reuse patch
candidate 3 ┤
...         │
candidate N ┘
```

Recommended staged search:

```text
phase 1
    huge candidate set
    loose bounded field approximation

phase 2
    survivors
    tighter patch / smaller cohorts

phase 3
    final handful
    authoritative/exact revalidation
```

This is acceptable because pruning approximation does not define final physical truth. Chosen trajectories are revalidated against the authoritative gravity path before execution/commit.

---

## 13. Fleet-level planning sharing

For convoys following essentially one route, planning itself can be shared:

```text
FleetPlan
    reference trajectory
    departure/launch window
    common source hierarchy decisions
    common GravityPatch sequence
    shared predicted events

VehicleFollower
    local offset
    collision/deconfliction
    actuator/propellant state
    correction burn
```

This does not force formation flight. It only avoids solving the same long-horizon orbital problem hundreds of times when differences are small.

Individual craft remain authoritative physical objects.

---

## 14. Backend evolution

Architecture should allow multiple evaluator backends later, but backend proliferation is explicitly deferred.

Initial:

```text
CpuExact
CpuSimdPatch
```

Possible later measured paths:

```text
GpuPatchBatch
GpuTrajectoryBatch
```

A future GPU backend becomes interesting if:

- target count is large enough;
- positions/state already reside on GPU for multiple propagation steps;
- transfer/synchronization cost is amortized;
- entire candidate-trajectory batches can remain device-side.

NPU/tensor/matrix hardware is not a baseline requirement. Tiny `3x3` affine evaluation is a poor match. If future quadratic/high-order batched representations become GEMM-shaped at very large N, they can be benchmarked separately.

Backend choice must remain capability/workload driven, not vendor driven.

---

## 15. Baked source hierarchy plus runtime target hierarchy

The useful asymmetry is:

```text
sources
    small count
    known astronomical topology
    deterministic baked trajectories
    fixed/versioned source tree

vs

targets
    potentially thousands
    dynamic positions
    dynamic grouping
    runtime cohort split/merge
```

So Thessa does not need a fully generic N-body FMM implementation.

A specialized architecture can be significantly simpler and faster:

```text
baked source tree
    + runtime source opening
    + runtime target cohorts
    + local polynomial patches
```

This is FMM-like in spirit without paying for generality that the game does not need.

---

## 16. Benchmark plan

Microbenchmarks alone are insufficient.

### 16.1 Primitive costs

Measure:

```text
exact point-mass source contribution
source-tree traversal
monopole aggregate
quadrupole aggregate
patch compile
patch affine eval scalar
patch affine eval AVX2
patch affine eval AVX-512 where available
quadratic eval if implemented
```

### 16.2 Fleet workloads

At minimum:

```text
1 target
16 targets
64 targets
128 targets
300 targets
1000 targets
```

Scenarios:

1. compact convoy in deep space;
2. convoy near Nereid with 1–3 exact-near sources;
3. convoy stretches until cohort split;
4. group crosses from A-side far-field BC aggregate toward Janus/BC and tree opens;
5. mixed active + rails targets in same region;
6. multiple independent convoys in different regions;
7. long warp/coast with large integration steps;
8. planner batch with 1k/10k+ candidates.

Metrics:

```text
ns / target force evaluation
ns / patch compile
source terms evaluated / target
source-tree nodes visited / patch
patch reuse count
cohort split count
exact-near terms / target
field approximation max accel error
trajectory position/velocity error
end-to-end simulated seconds / wall second
1/2/4/8/16 thread scaling
L1/L2/LLC misses
bytes touched / target
allocations
```

Main KPI is end-to-end fleet propagation throughput at pinned physical error, not maximum isolated FMA throughput.

---

## 17. Correctness and validation

Every approximation path must be checked against exact source accumulation.

Required tests:

- source aggregate monopole/quadrupole vs explicit children across representative geometry;
- baked aggregate tracks vs runtime aggregate recomputation;
- patch field vs exact field over random points inside validity volume;
- temporal validity over full patch interval;
- cohort splitting before declared error bound is exceeded;
- trajectory propagation exact vs patch-based over reference scenarios;
- final planner candidate revalidation;
- deterministic replay independent of Rayon worker count.

Error should be measured in absolute decision-relevant units:

```text
m/s² acceleration error
m position divergence
m/s velocity divergence
```

not only relative percentage near zero.

---

## 18. Non-goals

This work must not:

- introduce SOI switching as physical law;
- remove gravity sources because they are inconvenient;
- make `cargo` or `debris` an excuse for unbounded fake physics;
- require GPU on authoritative server;
- rebuild a generic Barnes–Hut source tree every tick when topology is known;
- bake a gigantic 4D `g(x,y,z,t)` grid for the whole system;
- hide approximation error inside unexplained magic tolerances;
- couple correctness to current vehicle class names;
- force all targets in one region to use one timestep or one integration regime;
- skip final exact revalidation for planner/autopilot outputs when execution correctness matters.

---

## 19. Suggested implementation order

1. Add an exact benchmark that reports source-accumulation cost separately from ephemeris lookup/interpolation.
2. Introduce logical baked source-tree metadata without changing `GravityField` results.
3. Implement monopole aggregate evaluation and differential tests against explicit children.
4. Add conservative opening/error criterion.
5. Add optional quadrupole representation only if measurements justify it.
6. Implement single `GravityPatch` affine field around a fixed anchor and validate over radius/time bounds.
7. SIMD batch-evaluate one patch over SoA target arrays.
8. Split far shared patch from short exact-near source list.
9. Add runtime target cohort construction + split on failed bound.
10. Reuse patch/tree decisions across repeated integration evaluations.
11. Integrate with rails/fleet workloads and benchmark `100–300` freighters.
12. Reuse the same patch compiler in autopilot/trajectory candidate search.
13. Only after CPU path is measured, decide whether a GPU batch backend has a real end-to-end win.

---

## 20. Acceptance criteria

First serious milestone is accepted when:

- exact path remains available and unchanged in physical definition;
- baked source hierarchy is deterministic/versioned;
- source aggregates have conservative measured error bounds;
- a compact fleet can share one field patch without per-target source-tree traversal;
- patch invalidation/splitting happens before its error contract is violated;
- near-body encounters automatically refine rather than using class-based hacks;
- authoritative server remains CPU-only capable;
- `100–300`-target fleet benchmark shows a material end-to-end speedup over independent exact accumulation at matched trajectory error;
- autopilot/planner can reuse the same bounded representation and exact-revalidate finalists;
- performance counters show where the remaining cost lives before adding GPU/NPU/tensor complexity.

Ключевой принцип:

```text
Do not solve the same slowly-varying gravity problem hundreds of times.

Compile the field once where possible,
prove its validity envelope,
and spend exact work only where the error budget demands it.
```

---

## 21. Первые измерения (шаги 1–9, Zen 5, native, 22 источника)

Команды: `cargo bench-native -p thessa-sim-core --bench flock`,
`--bench fleet_prop [size] [threads]`. Флот — 40 000 тиков × 0.5 с
(20 000 сим-секунд), бюджет когорты 1e-9 м/с². Точность — абсолютная
дивергенция траекторий против exact-накопления за весь прогон.

Финальный KPI, 1 поток, deep-space cruise:

```text
x128:  direct 60.0 s | framed 0.94 s | cohort 0.31 s (192x vs direct, 3.0x vs framed) | 2.8 см
x300:  direct 110 s  | framed 1.52 s | cohort 0.38 s (293x vs direct, 4.0x vs framed) | 6.0 см
x1000: direct 468 s  | framed 6.06 s | cohort 0.99 s (475x vs direct, 6.1x vs framed) | 6.3 см
```

Скейлинг x300 cohort deep-space: 1T 0.38 с | 2T 0.56 с | 4T 0.81 с |
8T 1.46 с | 20T 2.21 с. Больше потоков — строго медленнее: батч
latency-bound, диспетчеризация Rayon (~десятки мкс на регион) доминирует
над математикой (патч — 4–14 нс/таргет). Вывод для продакшена: тики флота
таких размеров — одним потоком (или few threads), Rayon — на более coarse
уровень; замерять, а не предполагать.

Сценарий low-orbit shear (кеплеров сдвиг разносит группу до 586 км):
когорты сплитятся (21 672 сплита, 1.5 когорты/тик, 3.1 exact/22),
фолбэков 0, дивергенция 11 см / 47 мкм/с — деградация graceful, физика
там, где её требует ошибка, а не класс аппарата.

Попутные гипотезы: hoist (mu,index) из per-target цикла — выигрыша нет
(в шуме, оставлен как фундамент); SIMD-хвост 22 источников —
каскад 66 нс / паддед-24 53 нс / скаляр 224 нс (см. вопрос в §11);
par_iter+Result-collect на 1000 лейнов — 229 мкс против 14 мкс
sequential (диагноз гранулярности, лечится чанками 1/worker +
infallible kernel + finite-scan).

---

## 22. Добивка: шаги 5, 10, 12, 13 и acceptance §20

### Шаг 5 — квадруполь: не justified (измерено)

Opening pressure монополя на реальной системе (22 источника, 24 узла):
deep-space convoy при бюджете 1e-9 — 13/22 exact на таргет (18 узлов
визитов), при 1e-12 — 20/22. То есть монополь открывает больше половины
дерева даже в дальнем поле. Но потребителя, которому это мешает, нет:
per-target оценка флота идёт патчами (0–3 exact/таргет при тех же
бюджетах), compile патча — exact и дешёвый (5 мкс). Квадруполь окупился
бы только для tree-opened compile или систем >100 источников.
Вердикт: отложен, условие возврата — давление измерением, а не мнением.

### Шаг 10 — reuse window: работает, с честной границей применимости

`CohortEvaluator` (scratch без аллокаций + окно одного патча): spatial
remainder при новом радиусе + temporal staleness (Липшиц `2/|y|^3` на
сдвиг источников), всё bound-driven, без магических порогов. Доказано
пилой на бинарной системе: reuse 5 тиков, линейный рост bound до
бюджета, rebuild, повтор — каждый тик сверен с exact.
Граница: на system.toml при тиках 0.5 с и бюджете 1e-9 быстрые луны
сдвигают дальнее поле на ~1e-8/тик — окно корректно перекомпилирует
каждый тик (reuses 0%). Reuse платит при свободных бюджетах, медленных
эпохах и одноэпохальных кандидатах планировщика (temporal = 0).

### Шаг 12 — планировщик: форма нагрузки доказана, интегрировать нечего

Maneuver planner отсутствует (ManeuverNode reserved, flight loop его не
исполняет) — втыкать некуда. Вместо этого бенч `planner_batch`
(1k/10k кандидатов, одна эпоха, один регион): общий патч против exact
batch — x1000: 12x (52 нс/кандидат), x10000: 1.5x (диспетчеризация
амортизирована, exact тоже быстр); max error везде внутри posted bound.
API (`compile_patch`, `GravityPatch`) готово к будущему планировщику;
финалисты валидируются exact-путём по контракту fail-open.

### Шаг 13 — GPU: не нужен (измерено, условие возврата зафиксировано)

Тик флота x1000 — 25 мкс single-thread CPU all-in. Только PCIe
round-trip (позиции+состояния туда, ускорения обратно + sync) съест
больше всего CPU-бюджета. GPU-бэкенд оправдан при (а) N ≥ ~50k, где
трансфер амортизируется, или (б) device-resident траекториях на много
шагов — оба условия из §14, ни одно не выполнено. CPU-only остаётся
каноническим путём.

### Acceptance §20 — построчно

- exact path доступен и не менялся: `acceleration()` нетронут, побитовые
  регрессии (`frame_batch...`, финалы 40k-прогона) зелёные;
- иерархия детерминирована и версионирована: топология выводится из
  baked-контента (его `format_version`), отдельного версионирования не
  требует;
- агрегаты имеют консервативные измеренные границы: дифтесты монополя
  (measured ≤ posted, скейлинг bound/field), Hessian-константа прибита
  численно;
- компактный флот делит один патч без per-target обхода: deep-space
  40k тиков — cohorts/tick 1.0, splits 0, exact 0/22;
- инвалидация/сплит до нарушения контракта: сплит-тесты, пила reuse,
  shear-прогон (21 672 сплита, фолбэков 0);
- у тела — refine без классовых хаков: exact-near по дистанции/вкладу
  (`classify_routes_near_host_to_exact`);
- сервер остаётся CPU-only capable: все KPI выше — один поток CPU;
- флот 100–300 — материальный выигрыш при согласованной ошибке:
  293x vs direct / 4.0x vs framed на x300, дивергенция 6 см за 20 000 с;
- планировщик переиспользует то же представление: бенч §12 + контракт
  exact-ревалидации;
- счётчики показывают остаток цены: exact/target, splits, cohorts,
  reuses, posted bound — в каждом прогоне fleet_prop/planner_batch.

---

## 23. Второй проход: ревью-фиксы, serial kernels, настоящие низкие орбиты

Код-ревью пачки `eb4d03c/096a06b` (читался именно код, не summary) дал
семь пунктов — все приняты:

- P1/benchmark: `6.9e6 m` — это ~3700 км над Thessa (R = 3200 км), не
  низкая орбита. Якоря заменены на `radius_m + alt`, трио 100/300/1000 км.
- P1/perf: inner Rayon удалён из `accelerations_from_frame` и
  `eval_patch_batch` — kernels строго serial; parallelism только уровнем
  выше (fleets/cohorts/jobs). Замер подтверждает: framed/cohort
  thread-invariant (x300: 792/190 мс на 1T против 790/183 мс на 20T),
  дивергенция побитово та же.
- P2/telemetry: `exact_terms` считался до проверки бюджета (отвергнутый
  parent засчитывал невыполненную работу) — перенесён в accepted branch.
- P2/perf cliff: median-tie degenerate check удалён — tie-break по index
  и так детерминированно делит пополам; `[0,0,0,1]` больше не валится в
  full exact.
- P2/contract: tree тратит remaining budget (ascending обход) — posted
  bound всегда ≤ allocated; multi-accept тест падает на старом коде,
  проходит на новом (проверено revert-прогоном).
- P3: leaf support radius = 0 (point-mass; физический радиус — дело
  collision/surface, не monopole error).
- P3/validation: split-тест требует measured ≤ posted (не 10x budget) +
  deterministic property-test (LCG, 3 системы × 16 геометрий).
- Perf-мясо: inplace indices + `split_at_mut`, indexed compile без копий
  групп; serial kernels пишут сразу в слоты (без temp+scatter).

Финальный KPI, 40k тиков × 0.5 с, бюджет 1e-9 (обе колонки честно):

```text
case            direct 1T   direct 20T  framed   cohort   vs best-direct  vs framed  div
deep x128       23.7 s      —           0.40 s   0.159 s  ~25x (*)        2.5x       2.8 см
deep x300       55.8 s      9.24 s      0.79 s   0.190 s  50x             4.2x       6.0 см
deep x1000      184.8 s     —           2.41 s   0.407 s  ~75x (*)        5.9x       6.3 см
low100km x300   55.5 s      9.25 s      0.79 s   0.302 s  30x             2.6x       2.3 м
low300km x300   55.5 s      9.36 s      0.81 s   0.299 s  31x             2.7x       0.98 м
low1000km x300  55.6 s      8.97 s      0.78 s   0.309 s  28x             2.5x       0.54 м
```

(*) direct на 20T измерен только для x300 (~6x vs 1T); для x128/x1000
оценка делением. Первая колонка — выигрыш против старого алгоритма,
вторая — цена именно новой математики относительно нормального exact.

Низкие орбиты: exact-near pressure 3.1–3.4/22 (против 0 в deep space),
splits тысячи, reuse 26–34%, группа разносится до ~1800 км — и всё равно
2.5–2.7x vs framed при дивергенции 0.5–2.3 м за 20 000 с.

Планировщик serial-vs-serial: x1000 — 8.4x (7 нс/кандидат), x10000 —
7.5–8x. Single-tick флок: cohort 15–22 нс/таргет deep/low при 1–2 exact.

Вывод про parallelism обновлён и усилен: kernels serial
by construction (измерение применено в реализации, а не только в доке);
масштабирование — coarse задачами. Точка crossover для возврата
внутреннего Rayon не найдена до 10k таргетов — single-thread cohort тик
x1000 занимает 10 мкс.
