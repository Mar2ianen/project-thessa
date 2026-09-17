# 20 — Advanced aerodynamic effectors: flaps, spoilers, grid fins, body flaps

Статус: **design target**.

Эта дока описывает следующий слой realtime-aero поверх существующего `PanelAeroModel`: high-lift devices, spoilers/speedbrakes, grid fins и большие hinged body flaps уровня Starship. Цель — расширить уже работающий O(panels) solver, не превращая runtime в CFD и не вводя отдельные классы `Aircraft`, `Rocket` или `Starship`.

## 1. Что уже есть

Текущий `thessa-sim-core` уже имеет правильную базу:

- `AeroPanel` с геометрией, sweep, aspect ratio, `thickness_to_chord_ratio`;
- отдельный `center_of_pressure_body_m`;
- локальный flow через `omega × r`;
- attached/post-stall blending;
- control deflection как изменение effective AoA;
- static `Cm` и dynamic damping derivatives;
- SoA/SIMD path;
- optional Tier B coefficient tables;
- `ControlSurfaceDefinition`, который связывает один physical control channel с одной или несколькими панелями.

`ControlSurfaceDefinition` уже комментарием допускает elevator, split elevons, rudder, flaps и procedural surfaces. Но текущая физическая модель control surface фактически одна:

```text
alpha_eff = alpha_aero + effectiveness * control_gain * deflection
```

Этого достаточно для elevator/rudder/aileron first slice, но недостаточно для устройств, которые меняют camber/area, намеренно вызывают separation или физически поворачивают большую часть projected body area.

## 2. Главный architectural split

Нужно разделить две вещи:

```text
ControlSurfaceDefinition
    = physical actuator/channel, limits, linked panels

AeroEffectorModel
    = как изменение actuator state меняет aero coefficients / geometry
```

То есть `flap` не должен становиться новым видом vehicle, а `spoiler` — специальной веткой в `FlightAuthority`.

Пример target model:

```rust
pub enum AeroEffectorModel {
    Incidence(IncidenceEffector),
    Flap(FlapEffector),
    Spoiler(SpoilerEffector),
    HingedPanel(HingedPanelEffector),
    GridFin(GridFinEffector),
    Table(TableEffectorId),
}
```

`Incidence` сохраняет нынешнюю semantics для elevator/rudder/aileron. Остальные модели добавляют свои локальные modifiers, после чего общий solver всё равно возвращает force/moment тем же `AeroResult`.

## 3. Сначала исправить command domain

Сейчас `ControlSurfaceDefinition::validate` требует:

```text
minimum_deflection_rad < 0
maximum_deflection_rad > 0
```

а `apply_control_inputs` принимает normalized command в `[-1, 1]` относительно нуля.

Это не выражает обычный spoiler или landing flap с физическим диапазоном примерно `0 .. +delta_max`. Не стоит обходить это фиктивным отрицательным диапазоном.

Нужен explicit neutral + arbitrary physical limits:

```rust
pub struct ControlSurfaceDefinition {
    pub name: String,
    pub panel_indices: Vec<usize>,
    pub minimum_deflection_rad: f64,
    pub neutral_deflection_rad: f64,
    pub maximum_deflection_rad: f64,
    pub aero_effect: AeroEffectorModel,
}
```

Инвариант:

```text
min <= neutral <= max
```

Normalized actuator command может остаться `[-1, 1]`, но mapping выполняется относительно `neutral`:

```text
command < 0: neutral -> min
command > 0: neutral -> max
```

Для чистого flap/spoiler asset задаёт `min == neutral == 0`, поэтому отрицательная половина команды либо clamp-ится в neutral, либо policy/allocator вообще задаёт односторонние bounds. Более чистый долгосрочный вариант — allocator работает сразу в physical deflection bounds и normalized pilot input заканчивается выше по stack.

## 4. Runtime state не должен мутировать static geometry без необходимости

Сейчас `apply_control_inputs` пишет `control_deflection_rad` прямо в `AeroPanel`. Для следующего слоя лучше различать:

```rust
AeroPanelDefinition   // static asset geometry
AeroPanelControlState // current actuator-derived state
```

Например:

```rust
pub struct AeroPanelControlState {
    pub deflection_rad: f64,
    pub forced_separation: f64,
    pub camber_delta: f64,
    pub area_scale: f64,
}
```

Не все поля обязаны существовать буквально в таком виде. Важен принцип: asset geometry не становится storage для transient aerodynamic effects.

SoA path должен получать compact arrays уже подготовленного control state, чтобы новые effectors не уничтожили SIMD layout.

## 5. Flaps / high-lift devices

Обычный trailing-edge flap меняет не только effective AoA. Он меняет:

- camber;
- zero-lift angle;
- lift curve / `CLmax`;
- drag;
- pitching moment;
- stall behavior;
- для Fowler-like devices — effective wing area/chord.

FAA/NASA reference material прямо описывает рост lift через увеличение camber и, для выдвижных механизаций, площади; drag при больших deflection также заметно растёт. Flap deployment меняет pitching moment, причём итоговый pitch response зависит от конкретной геометрии самолёта и downwash на хвост.

Для Tier A нужен reduced-order modifier, например:

```text
alpha0'      = alpha0      + d_alpha0(delta)
lift_slope'  = lift_slope  * k_lift_slope(delta)
CLmax'       = CLmax       + d_CLmax(delta)
CD0'         = CD0         + d_CD0(delta)
Cm'          = Cm          + d_Cm(delta)
stall_angle' = stall_angle + d_stall(delta)
area'        = area        * k_area(delta)       // optional Fowler-like
```

Не все зависимости должны быть аналитическими. Piecewise cubic / small lookup curve по normalized deployment дешёвая и понятная.

### 5.1 Center of pressure

Физически flap меняет pressure distribution и тем самым center of pressure. В runtime уже есть два независимых способа представить pitching effect:

1. geometric force arm через `center_of_pressure_body_m × F`;
2. aerodynamic pitching coefficient `Cm`.

Для обычного flap Tier A предпочтительно держать static/geometric CoP и кодировать migration результирующей силы через `d_Cm(delta, alpha)` — это устойчивее и не создаёт moving geometry в hot loop.

**Нельзя одновременно** двигать CoP на эквивалентную величину и добавлять тот же `d_Cm`: это double counting.

Tier B table может напрямую задавать `CL/CD/Cm` для каждого flap setting; тогда analytic modifier не нужен.

### 5.2 Flaperon / elevon

Не нужно разрешать нескольким logical channels мутировать одну панель в произвольном порядке. Physical surface должна быть одним actuator, а symmetric bias + differential demand смешиваются до actuator target.

Пример:

```text
left_flaperon  = flap_bias + roll_command
right_flaperon = flap_bias - roll_command
```

Дальше allocator clamps physical deflection и сообщает residual/saturation. Это хорошо ложится на unified allocator из `docs/18-control-guidance-autopilot.md`.

## 6. Spoilers / speedbrakes

Spoiler по смыслу отличается от elevator. Он поднимается в поток и **портит attached flow** на части поверхности: lift уменьшается, drag увеличивается. При асимметричном deployment это создаёт roll/yaw authority; при симметричном — speedbrake/lift dump; после touchdown lift dump увеличивает normal load на колёса и тем самым доступное friction braking.

Поэтому spoiler нельзя моделировать только как `alpha_eff += k*delta`.

Хороший Tier A proxy использует forced separation:

```text
s_nat     = natural_separation(alpha, Mach, ...)
s_spoiler = spoiler_separation(deployment, local_flow)

s_total = 1 - (1 - s_nat) * (1 - s_spoiler)
```

После этого уже существующий attached/separated blend автоматически уменьшает lift и переводит moment к separated branch. Дополнительно нужен spoiler-specific form drag:

```text
CD += d_CD_spoiler(deployment, alpha, Mach)
Cm += d_Cm_spoiler(...)
```

`AeroPanel::exposure` для этого использовать не надо. `exposure` семантически относится к occlusion/wake и означает, какая доля панели вообще видит incoming flow. Spoiler не скрывает панель от воздуха — он меняет режим обтекания.

### 6.1 Ground spoilers

Ground-spoiler logic находится выше aerodynamics:

```text
weight_on_wheels && deployment_command -> spoiler actuator target
```

Сам solver только создаёт меньший lift и больший drag. Если ground/contact model считает предел колёсного торможения через normal reaction,

```text
F_brake_max = mu * N
N ~= weight - aerodynamic_lift
```

то эффект ground spoilers возникает естественно без игрового `+30% brakes` modifier.

## 7. Hinged body flaps / Starship-like surfaces

Starship-like body flap — это не conventional high-lift flap. Это большая подвижная поверхность, которая заметно меняет projected geometry и local pressure forces всего аппарата.

SpaceX описывает controlled belly-first descent Starship как независимое движение **двух forward и двух aft flaps**. На V3 отдельно переработана actuation system aft flaps. Эти поверхности управляют аэродинамическим моментом и energy/attitude during entry/descent, а не служат «закрылком для увеличения `CLmax` при посадке».

Для такой поверхности текущего `alpha_eff` недостаточно. Нужен generic `HingedPanel` model, который физически поворачивает локальные panel axes вокруг hinge axis:

```text
R = rotation(hinge_axis, deflection)
chord_axis' = R * chord_axis
lift_axis'  = R * lift_axis
```

Flow sample position можно оставить около geometry zone / hinge-derived point, а force application point задавать отдельным CoP. Для большой поверхности vehicle compiler может разбить flap на несколько zones.

При больших углах deflection она естественно становится почти plate-like body/control surface; existing post-stall flat-plate branch здесь особенно полезна.

### 7.1 Body flap authority

Authority не должна иметь специальный `if atmosphere`:

```text
q = 0.5 * rho * v^2
F_aero ~ q * S * C
```

В вакууме `rho -> 0`, и body flaps сами теряют authority. Unified allocator видит нулевые/малые derivatives и переносит moment demand на RCS/TVC/reaction wheels.

Это также позволяет плавный mixed-control transition на entry без жёсткого переключателя «теперь Starship управляется рулями».

### 7.2 Large-deflection caveat

Для больших body flaps body/flap interference и hypersonic flow могут сильно отличаться от isolated flat plate. Поэтому:

- Tier A `HingedPanel` — gameplay/realtime reduced-order model;
- Tier B table — preferred для конкретного Starship-like vehicle;
- Tier C CFD/wind-tunnel/reference data — источник таблиц и validation, не runtime dependency.

## 8. Grid fins

Grid fin — не просто маленький solid fin. Решётка создаёт сложное внутреннее течение; её coefficients сильно зависят от Mach, incidence, deflection и взаимодействия с корпусом. NASA wind-tunnel work по Orion LAV специально собирало force/moment data отдельных grid fins от subsonic через transonic до supersonic (`M=0.5…2.5`), что хорошо показывает: это естественный кандидат на table-driven модель.

Target architecture:

```rust
pub struct GridFinEffector {
    pub reference_area_m2: f64,
    pub hinge_axis_body: DVec3,
    pub body_interference_factor: f64,
    pub coefficient_source: GridFinCoefficientSource,
}

pub enum GridFinCoefficientSource {
    AnalyticProxy(GridFinProxy),
    Table(AeroCoefficientTableId),
}
```

Tier A proxy может выдавать local normal-force / axial-drag coefficients как функцию:

```text
Mach
local alpha/beta
fin deflection
Reynolds (optional first slice)
```

но не нужно притворяться, что обычный thin-airfoil formula точна для grid lattice в transonic/supersonic flow.

### 8.1 Super Heavy reference

Актуальный Starship V3 / Super Heavy V3 reference нельзя хардкодить как «четыре руля»: SpaceX в мае 2026 описала переход **с четырёх grid fins на три**, каждая примерно на 50% больше и существенно прочнее. Они также перенесены ниже и re-clocked.

Thessa всё равно не должна иметь `SuperHeavyGridFinCount = 3`. Vehicle asset задаёт любое число independent effectors; V3 просто хороший validation/demo case для асимметричной трёхповерхностной конфигурации.

## 9. Control allocation

После добавления нелинейных effectors allocator не должен знать их названия. Он работает с локальной effectiveness matrix / Jacobian:

```text
J_i = d[wrench] / d[actuator_i]
```

Для обычного elevator derivative почти симметричен около neutral. Для spoiler с neutral на lower bound derivative естественно one-sided. Для grid fin/body flap derivative может сильно зависеть от текущего Mach/AoA/q.

Рекомендуемый runtime flow:

```text
current state + local atmosphere
        |
        v
sample effector effectiveness around current actuator state
        |
        v
bounded allocator
        |
        v
actuator targets
        |
        v
rate/load-limited actuator dynamics
        |
        v
AeroPanelControlState
        |
        v
PanelAeroModel
```

Не обязательно численно perturb-ить каждый actuator на каждом tick. Для analytic effectors derivative можно получить дёшево; table effectors могут хранить/interpolate derivatives. Numeric finite difference остаётся fallback/debug reference.

## 10. Actuator dynamics and aerodynamic load

Следующий полезный realism layer после geometric effect:

- slew-rate limit;
- asymmetric extension/retraction rate;
- position limits;
- actuator failure/jam;
- hinge-moment / aerodynamic-load limit;
- thermal inhibit/damage;
- power/hydraulic/electric availability.

При большом dynamic pressure поверхность может иметь достаточную aerodynamic authority, но actuator может не суметь быстро или вообще физически дойти до requested angle. Это должно проявляться как actuator saturation/residual wrench, а не как искусственное уменьшение `CL`.

Первый slice может оставить constant slew rate; load-dependent limit добавить позже.

## 11. Interaction with stall/separation model

Effectors должны использовать общий separation state, но по-разному:

```text
Incidence:
    меняет alpha_eff; authority fades after separation

Flap:
    меняет attached coefficients / stall envelope;
    после separation эффект также fades/blends

Spoiler:
    сам добавляет forced separation

HingedPanel:
    меняет geometry axes; затем проходит обычный attached/separated solver

GridFin:
    обычно coefficient table / dedicated proxy, со своим nonlinear response
```

Это сохраняет одно физическое место, где решается attached vs separated flow, и не размазывает stall branches по `FlightAuthority`.

## 12. Center of pressure and moments

Общий инвариант для всех новых effectors:

```text
M_total = r_CoP × F + M_aero_local + M_dynamic
```

- `r_CoP` используется для реального geometry arm;
- `M_aero_local` — профильный/pressure-distribution moment;
- deflection-dependent pressure migration обычно кодируется через `dCm`;
- explicit moving CoP допустим, если он приходит из geometry/table model и **не дублируется** в `dCm`.

Для large body flap отдельный CoP поверхности особенно важен: большая сила далеко от CG и есть основной источник control moment.

## 13. Occlusion / wake

Будущий BVH/wake solver и текущий `exposure` хорошо подходят для:

- body shadowing control surface;
- plume/flow occlusion;
- grid fin в следе корпуса;
- flap behind another geometry element.

Но effectors не должны напрямую писать в `exposure` для имитации своих собственных coefficient changes. Сначала считается external-flow visibility, затем effector physics.

## 14. Performance

Требование остаётся тем же: realtime cost O(number of panels/effectors), без runtime CFD.

Hot path должен сводиться к:

- compact per-lane control-state arrays;
- polynomial/piecewise modifiers;
- small table interpolation;
- минимум branches;
- scalar/SIMD parity.

Grid fins и body flaps не являются причиной переводить весь craft на дорогой solver. Если конкретный vehicle требует точности — bake coefficients в Tier B.

## 15. Validation plan

### 15.1 Flaps

- neutral state bit-equivalent нынешнему panel solver;
- deployment увеличивает lift в low-AoA takeoff-like point;
- большой deployment увеличивает drag;
- `dCm` имеет asset-defined sign и не double-counts CoP;
- Fowler-like `area_scale` меняет force пропорционально площади при одинаковых coefficients.

### 15.2 Spoilers

- deployment уменьшает lift и увеличивает drag;
- left-only spoiler создаёт roll moment правильного знака;
- symmetric deployment не создаёт roll на symmetric craft;
- при already-separated flow дополнительный effect bounded;
- `exposure` остаётся независимым.

### 15.3 Hinged body flaps

- `rho = 0` -> zero aerodynamic authority независимо от deflection;
- symmetric forward/aft commands дают ожидаемый pitch sign;
- differential left/right commands дают roll/yaw sign;
- authority масштабируется примерно с `q` в одном coefficient regime;
- large-deflection geometry остаётся finite и не создаёт NaN около 90°.

### 15.4 Grid fins

- proxy/table continuity across Mach cells;
- deflection = 0 на symmetric flow не создаёт spurious side moment;
- sign symmetry для `+delta/-delta` там, где reference data симметрична;
- optional validation против public NASA Orion LAV grid-fin dataset / published coefficients;
- body-interference multiplier bounded and explicit.

### 15.5 SIMD / determinism

Для каждого нового effector обязательны:

- scalar vs AVX2/AVX-512 coefficient parity;
- deterministic reduction order;
- serialize/deserialize roundtrip vehicle assets;
- no effect on unrelated panels at neutral;
- benchmark 1 / 16 / 256 / 1024 vehicles.

## 16. Suggested implementation order

### Slice A — control-surface data cleanup

- neutral deflection + unidirectional bounds;
- static definition vs runtime control state;
- preserve current incidence behavior bit-for-bit at neutral/current X-15 settings.

### Slice B — flaps + spoilers

- `FlapEffector` coefficient modifiers;
- `SpoilerEffector` forced separation + form drag;
- allocator support for one-sided actuators;
- add flap/spoiler example vehicle and tests.

### Slice C — generic hinged panels

- hinge axis and rotated local axes;
- large-deflection tests;
- 4-surface Starship-like body-flap demo vehicle;
- mixed RCS/aero allocation across decreasing/increasing `q`.

### Slice D — grid fins

- grid-fin table/proxy ABI;
- 3-fin Super Heavy V3-like demo geometry;
- public NASA grid-fin reference validation;
- transonic table interpolation and body-interference tuning.

### Slice E — actuator load/failures

- hinge moment proxy;
- load-dependent rate/position limits;
- jam/failure states;
- alerting hooks (`ACTUATOR`, `CONTROL AUTHORITY`, configuration warnings).

## References

- Existing aero design: `docs/11_AERODYNAMICS.md`
- Unified control/allocator design: `docs/18-control-guidance-autopilot.md`
- Current panel solver: `crates/sim-core/src/aero.rs`
- Current vehicle/control-surface model: `crates/sim-core/src/vehicle.rs`
- NASA Glenn, spoilers: https://www.grc.nasa.gov/WWW/k-12/VirtualAero/BottleRocket/airplane/spoil.html
- NASA Glenn, flaps/slats: https://www.grc.nasa.gov/www/k-12/airplane/aflap.html
- FAA Airplane Flying Handbook, flap pitching behavior: https://www.faa.gov/sites/faa.gov/files/regulations_policies/handbooks_manuals/aviation/airplane_handbook/10_afh_ch9.pdf
- NASA NTRS, *Grid Fin Stabilization of the Orion Launch Abort Vehicle*: https://ntrs.nasa.gov/citations/20110013520
- NASA NTRS, *Simulation of Grid-Fin Control Surfaces*: https://ntrs.nasa.gov/citations/20110008384
- SpaceX, Starbase overview (two forward + two aft Starship flaps): https://www.spacex.com/vehicles/starship/assets/media/Starbase%20Overview.pdf
- SpaceX, May 2026 Starship V3 update (three Super Heavy grid fins; V3 flap actuation changes): https://www.spacex.com/updates/
