# 20 — Advanced aerodynamic effectors: flaps, spoilers, grid fins, body flaps

Status: design target — flaps/spoilers/hinged-panels/grid-fins, neutral bounds,
`AeroEffectorModel`, and the high-speed effects plan (§§17–20: sonic boom,
buffet, vapor/cone/contrail visuals, plasma blackout, vortex lift, ground
effect, icing hooks) not implemented; incidence-only control is the runtime.

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

## 17. High-speed regime effects (transonic → hypersonic)

> New section in English per `AGENTS.md §0` (no new non-English documentation).

The M2 reduced-order branches already cover stall/transonic/supersonic
coefficients for forces. This section plans everything *around* the force
solver at high speed: acoustic footprint, unsteady buffet, condensation
visuals, plasma blackout, vortex lift, and ground effect. The governing rule
is the same as for effectors: every effect is computed from geometry, state,
and field — never a magic multiplier — and every visual-only effect is
explicitly marked as force-neutral so it cannot leak into flight dynamics.

### 17.1 Where the current solver stops

`PanelAeroModel` returns quasi-steady forces up to supersonic Mach. It does
not produce: ground acoustic footprint, unsteady buffet loads, condensation
or trail visuals, ionization/comm effects, nonlinear vortex lift, or
height-dependent induced drag. All items below consume data the sim already
has (Mach, `q`, alpha, altitude, attitude) plus small, explicit additions.

### 17.2 Sonic boom carpet (Tier A analytic proxy)

Physics sketch: a supersonic vehicle trails a Mach cone (half-angle
`μ = asin(1/M)`); the ground intersection is the boom carpet, roughly
`half_width ≈ altitude · cot(μ)` wide, swept along the ground track. The
N-wave overpressure scales with weight, length, altitude, and Mach. Rather
than a full Whitham F-function propagation, Tier A uses a calibrated scaling
law anchored at public reference points (Concorde-class ~2 psf cruise,
subsonic cutoff below which refraction turns the carpet around before it
reaches the ground):

```text
inputs:  Mach, altitude, weight, length, ambient pressure, ground track
output:  carpet polygon (map), peak Δp per ground cell, cutoff flag
```

- Cutoff is gameplay-relevant physics, not a hack: below cutoff Mach (a
  function of the temperature profile the atmosphere model already owns) the
  boom never reaches the ground — high-supersonic corridors vs low boom
  approaches become a real routing decision.
- Gameplay hooks, all derived from the footprint (never touching the flight
  model): window-rattle events above ~1 psf, damage claims above a tuned
  threshold, populated-area noise budget for career/contracts, ATC-style
  supersonic corridors on the map.
- Explicit non-goals: no CFD propagation, no focusing caustics in Tier A
  (flagged for Tier B via ray-tracing tables if ever needed), no effect on
  the generating vehicle's aerodynamics.

### 17.3 Transonic buffet (bounded unsteady load)

Shock-induced separation makes lift fluctuate near the buffet boundary.
Model: a deterministic (seeded-RNG, replay-safe) unsteady increment on
normal force plus control-effectiveness jitter, both strictly bounded:

```text
dCL_buffet = buffet_gain(Mach, alpha) · pseudo_noise(t, seed)
|dCL_buffet| <= buffet_envelope(Mach, alpha)   // hard cap, never diverges
```

- Onset boundary from a small table (Mach × alpha) calibrated against
  swept-wing buffet-onset references; outside the boundary gain is exactly 0.
- Telemetry: vibration level feeds the pilot HUD and the alerting hooks
  (`docs/19`), structural fatigue accumulates only through the existing
  thermal/structural graphs once M2 lands them — no parallel damage model.
- Validation: onset boundary shape, zero effect outside, bounded spectrum,
  determinism across replays and worker counts.

### 17.4 Vapor cone (force-neutral visual)

Transonic condensation cloud (Prandtl–Glauert singularity visualization):
rendered when local Mach ∈ [0.95, 1.05] over lifting surfaces AND humidity
allows it. **Adds zero force by design** — it is a visualization of the
pressure field the solver already computed, gated by an aloft-humidity
profile (currently a gap: surface `moisture01` exists in worldgen, the
aloft profile belongs to the `02A` TBD cells).

### 17.5 Contrails (force-neutral visual + signature)

Appleman criterion (cold + humid enough) evaluated per engine/wingTrail
emitter from the atmosphere temperature/humidity profile. Gameplay value is
signature, not physics: a visible trail is a detectable trail (traffic,
screenshots, future stealth considerations). Zero force coupling.

### 17.6 Plasma sheath and radio blackout (gameplay timer from physics)

Entry heating (M2 thermal forbidden-zone work) plus ionization proxy yields
an electron-density estimate along the trajectory; above threshold the
link budget is zero:

```text
heating proxy (velocity, density, nose radius) -> ne estimate
ne > ne_critical(link frequency) -> COMM BLACKOUT window
```

- Gameplay: autopilot/scripts must be able to fly blind through the window
  (ties into `docs/07` waits and `docs/19` alerting); ground stations show
  loss-of-signal honestly instead of freezing telemetry.
- Visual: entry glow intensity from the same heating proxy (shared source,
  no separate magic glow number).
- Validation: blackout entry/exit altitudes vs Shuttle-class reference
  corridors, order-of-magnitude only — Tier A is a window predictor, not a
  plasma solver.

### 17.7 Vortex lift for low-aspect/delta wings

Attached-flow panels underpredict delta lift at high alpha. Add the Polhamus
suction-analogy term, driven purely by geometry the asset already has
(aspect ratio, sweep, area):

```text
CL = Kp·sinα·cos²α + Kv·sin²α·cosα
```

- `Kp` from the existing attached solver (no double count: the potential
  part is the panel lift it already computes); `Kv` from aspect-ratio
  correlation, bounded and documented.
- Validation: delta-wing reference polars (e.g. 60–75° sweep datasets),
  continuity with the attached branch at low alpha, stall blend unchanged.

### 17.8 Ground effect (height-dependent induced drag)

Within roughly one wingspan of the surface, induced drag drops (McCormick /
Raymer-type factor over `h/b`, wingspan `b` from geometry). Affects flare
and float distance on landing — and must vanish with altitude by
construction (`factor → 1` for `h/b → ∞`, exact equality above cutoff, not
asymptotic tail that pollutes cruise).

### 17.9 Icing hooks (listed future, not sliced)

Performance-degradation envelope (CL down, CD up, stall angle in) driven by
visible-moisture + sub-zero exposure time, with anti-ice bleed-air gameplay
hooks. Requires the aloft-moisture profile from §18 first; no slice assigned
until M2 thermal exists.

## 18. Data the sim has vs gaps

| Effect | Already present | Gap to close |
|---|---|---|
| Boom carpet | Mach, altitude, weight, length, track, temperature profile | calibrated overpressure anchors (2 reference points to start) |
| Buffet | Mach, alpha, q, seeded RNG harness | onset-boundary table (small, literature) |
| Vapor cone | local Mach field | aloft-humidity profile (`02A` TBD) |
| Contrails | temperature profile, emitters | aloft humidity (same gap) |
| Plasma blackout | velocity, density, nose radius (M2 heating) | link-frequency thresholds per station |
| Vortex lift | aspect ratio, sweep, area | Kv correlation constants + reference polars |
| Ground effect | height AGL, wingspan | nothing (pure geometry + state) |
| Icing | temperature, exposure time | aloft moisture (same gap), M2 thermal |

The single highest-leverage data gap is the **aloft humidity profile** — it
unblocks vapor, contrails, and icing at once, and `02A` already lists TBD
cells for it.

## 19. Validation plan (high-speed effects)

- Boom: carpet width = `h·cot(asin(1/M))` exactly; cutoff respected (zero
  footprint below cutoff Mach); overpressure monotonic in weight/altitude,
  anchored within 2x of Concorde/Shuttle reference psf bands.
- Buffet: exactly zero outside the onset table; bounded spectrum;
  bit-identical across replays/worker counts (seeded).
- Vapor/contrails: force parity — identical trajectories with visuals
  on/off (bitwise, enforced by test).
- Blackout: entry/exit window exists on a Shuttle-like profile, absent on a
  low-speed descent; scripts survive it in replay fixtures.
- Vortex lift: matches reference delta polars within posted envelope;
  low-alpha continuity with attached solver.
- Ground effect: factor exactly 1 above cutoff; flare distance increases vs
  no-effect baseline on the same approach.
- Perf (per AGENTS.md §10): per-effect scope counters; boom carpet update
  O(track points), buffet O(panels) with the existing SIMD lanes, visuals
  behind the graphics quality tiers (`crates/graphics`).

## 20. Implementation order (continued)

### Slice F — boom carpet + buffet + condensation visual

- boom carpet polygon + Δp proxy + cutoff + map overlay + noise-budget hooks;
- buffet gain/envelope tables + HUD vibration + replay determinism tests;
- vapor-cone visual gated by Mach band (humidity gate stubbed to
  always-false until the `02A` profile lands — visible code path, no fake data).

### Slice G — plasma blackout + vortex lift + ground effect

- heating-proxy → blackout windows + entry glow from one source;
- Polhamus term behind aspect-ratio gating + reference-polar tests;
- ground-effect factor with exact high-altitude cutoff + flare tests.

### Slice H — icing + weather coupling (after M2 thermal)

- visible-moisture exposure accumulator + degradation envelope;
- anti-ice gameplay hooks; coupling point for any future weather model.

## References

High-speed effects (§§17–20):

- NASA Glenn, sonic boom basics (Mach cone, carpet, overpressure factors): https://www.grc.nasa.gov/www/k-12/airplane/sonic.html
- NASA, Seebass-George sonic-boom minimization and Carlson simplified boom prediction (N-wave scaling, cutoff Mach): https://ntrs.nasa.gov/citations/19690023553
- FAA, Noise levels for U.S. certificated and foreign aircraft (psf reference bands): https://www.faa.gov/regulations_policies/policy_guidance/noise/
- Appleman contrail forecasting (temperature–humidity criterion): https://www.weather.gov/
- Polhamus suction analogy for vortex lift on delta wings: https://ntrs.nasa.gov/citations/19660010884
- McCormick / Raymer ground-effect induced-drag factor vs height-to-span ratio.

Effector references (existing):

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
