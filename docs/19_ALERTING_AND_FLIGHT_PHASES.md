# 19 — Flight-aware alerting and aural warning system

Статус: **design target**.

Эта дока задаёт архитектуру предупреждений для пилотируемых аппаратов Thessa: от stall/overspeed/terrain до конфигурационных, двигательных, тепловых и орбитальных предупреждений. Главный принцип: alerting не является набором `if` в HUD и не должен угадывать контекст на клиенте. Условия, inhibition, debounce, latch и priority вычисляются из авторитетного состояния полёта; клиент только визуализирует и воспроизводит звук.

Система намеренно не копирует конкретный Boeing/Airbus/Honeywell product. Авиационные стандарты используются как reference для семантики warning/caution/advisory и минимизации nuisance alerts, а тексты, тоны и voice pack являются собственными asset-данными проекта.

## 1. Что уже есть

`FlightAuthority` уже хранит почти все низкоуровневые входы, на которых должен строиться первый alerting slice:

- `FlightRegime::{Aero, Coast}`;
- altitude / local-air kinematics;
- Mach, AoA и dynamic pressure через `last_forces`;
- throttle и engine state;
- `gear_down`;
- control input и фактический `surface_input`;
- `actuator_saturated`;
- SAS/RCS state;
- authoritative `flight_error`;
- ephemeris, gravity, terrain field и world tick.

`FlightRegime` сейчас означает **solver regime**, а не этап миссии: `Aero` выбирается при достаточно большой плотности, `Coast` — ниже `COAST_DENSITY_KG_M3`. Это полезный вход alerting, но его нельзя перегружать значениями вроде `Approach` или `Landing`.

Текущий `ControlMode` также не является flight phase: `MouseAim/Navball/Rate/Direct` описывают способ управления. После control/GNC refactor input mapping, guidance, control law, policy и allocator должны оставаться отдельными слоями.

## 2. Инварианты

1. **Physics first.** Warning следует из физического состояния, envelope, capability и mission context; он не меняет состояние аппарата.
2. **Server-authoritative state.** Для сетевой игры активный набор flight alerts одинаков у всех клиентов данного аппарата.
3. **Sparse transitions.** По сети передаются переходы / текущий active set, а не аудиособытие на каждом physics tick.
4. **No audio logic in physics.** Solver выдаёт значения и события. Alert manager решает, что активно. Client audio решает, чем это проиграть.
5. **No nuisance spam.** Для каждого условия есть hysteresis/debounce/rearm semantics.
6. **Priority is explicit.** `PULL UP` не должен ждать, пока закончит говорить advisory про конфигурацию.
7. **Capability-gated.** Нет cabin-altitude warning у беспилотной болванки без pressurized cabin; нет `TOO LOW GEAR` у аппарата без шасси.
8. **Data-driven vocabulary.** Набор alerts зависит от avionics package / vehicle definition, а не от `VehicleKind::Airliner`.
9. **Deterministic.** Runtime TTS не участвует в authoritative path. Voice lines и tones заранее сгенерированы/поставляются как assets.

## 3. Solver regime ≠ operational flight phase

Для alert inhibition нужен ещё один ортогональный слой — operational phase. Он не должен менять физику и не должен быть единственным источником истины.

```rust
pub enum FlightPhase {
    Surface,
    Takeoff,
    AtmosphericFlight,
    Approach,
    Landing,
    GoAround,
    Ascent,
    Entry,
    PoweredDescent,
    Coast,
    Rendezvous,
    Docking,
}
```

Это **не обязательный финальный enum**. Важна архитектура: phase выводится из kinematics + guidance/mission intent + vehicle capability и используется только как контекст. Для гибридного spaceplane возможны, например:

```text
FlightRegime::Aero + FlightPhase::Entry
FlightRegime::Aero + FlightPhase::Approach
FlightRegime::Coast + FlightPhase::Rendezvous
```

Некоторые признаки лучше вообще не прятать в enum, а держать ортогональными:

```rust
pub struct AlertFlightContext {
    pub solver_regime: FlightRegime,
    pub phase: FlightPhase,
    pub weight_on_wheels: bool,
    pub radar_altitude_m: Option<f64>,
    pub terrain_clearance_m: Option<f64>,
    pub vertical_speed_mps: f64,
    pub mach: f64,
    pub dynamic_pressure_pa: f64,
    pub aoa_rad: f64,
    pub load_factor_body: glam::DVec3,
    pub gear_fraction: Option<f64>,
    pub high_lift_fraction: Option<f64>,
    pub actuator_saturated: bool,
}
```

`FlightPhase` может быть explicit guidance state, автоматическим classifier или смесью обоих: intent определяет, что аппарат пытается делать, kinematics проверяет, что он действительно находится в соответствующем режиме.

## 4. Alert data model

Минимальный runtime объект:

```rust
pub enum AlertSeverity {
    Warning,
    Caution,
    Advisory,
}

pub enum AuralKind {
    None,
    Tone(AuralId),
    Voice(AuralId),
    ToneThenVoice { tone: AuralId, voice: AuralId },
}

pub struct AlertDefinition {
    pub id: AlertId,
    pub severity: AlertSeverity,
    pub priority: u16,
    pub aural: AuralKind,
    pub repeat: RepeatPolicy,
    pub debounce_s: f64,
    pub clear_debounce_s: f64,
    pub latch: LatchPolicy,
    pub group: AlertGroup,
}
```

Condition и inhibition лучше не хранить как closures в asset. Первый implementation может иметь native Rust evaluators по stable `AlertId`; позже data assets могут задавать пороги/варианты текста.

Состояние одного alert:

```text
Inactive
  -> Pending         condition true, debounce not elapsed
  -> Active          annunciation + optional aural
  -> Latched         source cleared, acknowledgement/reset still required
  -> Inactive        clear/rearm complete
```

Для некоторых alerts `Latched` не нужен. Для critical warning acknowledgement не обязано глушить повторяющийся aural, пока hazardous condition остаётся активным.

## 5. Priority и audio arbitration

Одновременно могут существовать десятки активных сообщений, но cockpit не должен превращаться в аудио-DDoS.

Рекомендуемая модель:

```text
active conditions
      |
      v
severity / priority ordering
      |
      +-> visual annunciations: все релевантные
      |
      `-> aural arbiter: один foreground stream + tones with explicit policy
```

Правила первой версии:

- более высокий priority может preempt низкий voice line;
- один и тот же alert не стартует заново каждый tick;
- изменение параметра внутри активного condition не считается новым alert;
- command warnings (`PULL UP`, collision/impact, catastrophic thermal/structural limit) preempt configuration advisories;
- resolved warning не доигрывается, если его смысл уже ложен;
- repeated voice использует minimum repeat interval;
- aural silence/acknowledge является avionics action, не изменением physics condition.

FAA AC 25.1322-1 используется только как reference на общую идею различимых warning/caution/advisory и своевременного, не перегружающего экипаж alerting.

## 6. Первый inventory

Это не означает, что все пункты обязаны существовать у каждого craft.

| Alert | Базовое условие | Context / inhibition |
| --- | --- | --- |
| `STALL` | AoA / stall margin вышел за envelope при достаточном `q` | только `FlightRegime::Aero`; hysteresis |
| `OVERSPEED` | Mach/IAS/equivalent-speed envelope exceeded | `Aero`; порог задаёт vehicle envelope |
| `HIGH_AOA` | приближение к AoA limit | `Aero`; advisory/caution до stall |
| `HIGH_G` | load factor / structural load margin | capability/envelope-specific |
| `ACTUATOR_SATURATED` | allocator/runtime не может реализовать demand | suppress transient spikes; current `actuator_saturated` seed |
| `TERRAIN` | predictive terrain clearance недостаточен | не на `Surface`; требует terrain/radar source |
| `PULL_UP` | imminent terrain/impact trajectory | highest flight-path priority |
| `SINK_RATE` | excessive descent near terrain | approach/landing/low-altitude context |
| `DONT_SINK` | потеря высоты после takeoff/go-around | takeoff/go-around context |
| `TOO_LOW_GEAR` | низко + landing intent + gear not deployed | craft with retractable gear only |
| `TOO_LOW_FLAPS` | низко + landing intent + high-lift config insufficient | craft with high-lift devices only |
| `GEAR_OVERSPEED` | gear extended above deployment envelope | retractable gear only |
| `CONFIG_TAKEOFF` | takeoff demand with invalid configuration | Surface/Takeoff only |
| `CONFIG_LANDING` | landing intent with invalid configuration | Approach/Landing only |
| `ENGINE_OUT` | commanded/required engine unavailable | engine group policy decides severity |
| `ENGINE_FIRE` | fire detector/event | if vehicle has modeled fire system |
| `PROP_LOW` | propellant below mission-configured reserve | propulsion capability only |
| `CABIN_ALTITUDE` | pressurized occupied volume unsafe | crewed/pressurized capability only |
| `THERMAL_LIMIT` | heat flux / skin / component temperature margin | Entry/high-heating context |
| `MAX_Q` | dynamic pressure exceeds vehicle limit | atmospheric ascent/entry |
| `STRUCTURAL_LOAD` | force/moment/stress proxy exceeds limit | any regime where load model valid |
| `COLLISION` | predicted conjunction / near-field collision | rendezvous/docking/general traffic |
| `DOCKING_CLOSURE` | closure rate / alignment unsafe | Docking only |

`GLIDESLOPE` и похожие navigation-specific warnings должны появляться только при наличии соответствующего guidance/navigation source. Нельзя делать их просто из altitude + vertical speed.

## 7. Terrain / GPWS-like warnings

Не надо реализовывать GPWS как копию конкретной сертифицированной коробки. Нужен функциональный набор проверок поверх собственного terrain model.

Базовые входы:

```text
terrain clearance
vertical speed
horizontal velocity
predicted path over short horizon
gear/high-lift configuration
flight phase / landing intent
```

Первый deterministic predictor может брать несколько будущих samples вдоль ballistic/local-velocity projection. Позже его можно заменить trajectory-aware envelope, не меняя alert ABI.

Важный инвариант: terrain system предупреждает о **риске**, а не о номинальной высоте. Низкий полёт в landing configuration не должен постоянно орать `PULL UP`; быстро закрывающийся рельеф должен.

FAA TAWS guidance используется как reference на command-style terrain alerting и на необходимость раннего предупреждения с минимизацией unwanted alerts, но thresholds Thessa являются собственными игровыми/vehicle data.

## 8. Aural assets

Aural system имеет стабильные semantic IDs:

```text
warning.master
warning.terrain
warning.pull_up
warning.stall
warning.overspeed
warning.fire
caution.master
callout.minimums
callout.altitude.500
...
```

Voice и nonspeech assets отделены от кода:

```text
assets/audio/alerts/<pack>/manifest.toml
assets/audio/alerts/<pack>/voice/*.ogg
assets/audio/alerts/<pack>/tones/*.ogg
```

Рекомендуемая asset policy:

- speech генерируется offline TTS или записывается специально для проекта;
- простые siren/chime/whoop assets синтезируются собственным generator tool;
- runtime не зависит от облачного TTS;
- manifest хранит provenance, generator version и лицензию каждого файла;
- код может оставаться GPL/MIT split по текущей схеме, sound pack лицензируется отдельно;
- чужой CC0 reference можно использовать для анализа, но если нужен единый NC asset pack, финальный waveform лучше генерировать самостоятельно.

Локализация — другой voice pack с теми же `AuralId`. Authoritative alert ID от языка не зависит.

## 9. Network boundary

Authoritative server вычисляет condition state и transitions. Client получает:

```rust
pub struct AlertSnapshot {
    pub active: Vec<ActiveAlert>,
    pub generation: u64,
}

pub enum AlertEvent {
    Activated { id: AlertId, instance: u64 },
    Escalated { id: AlertId, instance: u64 },
    Cleared { id: AlertId, instance: u64 },
}
```

Не обязательно слать event на каждый snapshot. Нужны:

- sparse events для немедленного audio response;
- active set в periodic/full snapshot для reconnect/resync;
- monotonic `instance/generation`, чтобы duplicate packet не переиграл warning.

Чисто локальные UI alerts (`controller disconnected`, `audio device lost`, `network jitter`) находятся в другом namespace и никогда не маскируются под flight warning.

## 10. Scheduler и стоимость

Большинство условий дешёвые и могут проверяться на fixed flight tick. Дорогие predictive alerts не обязаны работать на 120 Hz.

Пример budget:

```text
120 Hz: stall, overspeed, loads, actuator saturation
20-30 Hz: gear/configuration, propulsion/thermal summaries
5-10 Hz: short-horizon terrain prediction
1-5 Hz: long-horizon conjunction / mission advisories
```

Alert manager должен получать уже вычисленные telemetry values, а не повторно вызывать аэродинамику, terrain solver или ephemeris без причины.

Для known-time events можно использовать существующий `EventScheduler`; alerting не должен создавать JS polling loop.

## 11. Tests

Минимальный suite:

1. condition crossing вызывает ровно один `Activated`;
2. threshold jitter не создаёт spam благодаря hysteresis/debounce;
3. high-priority warning preempts low-priority voice;
4. alert после clear/rearm может активироваться повторно;
5. `TOO_LOW_GEAR` невозможен без landing/low-altitude context;
6. `STALL` невозможен в exact vacuum независимо от orientation;
7. capability-gated alert отсутствует на craft без соответствующей системы;
8. snapshot resync не переигрывает уже активный aural;
9. scalar/server replay даёт одинаковую sequence of `AlertEvent`;
10. time-warp не пропускает critical transition: interval evaluation либо certified guard, либо boundary substep.

## 12. Порядок реализации

### Slice A — framework

- `AlertId`, severity, state machine, debounce/latch/rearm;
- alert manager внутри authoritative flight layer;
- protocol active-set + sparse transitions;
- client annunciator + audio arbiter;
- generated test tone/voice pack.

### Slice B — существующая telemetry

- stall/high-AoA;
- overspeed;
- max-Q/high-G;
- actuator saturation;
- engine/prop state;
- gear/configuration.

### Slice C — terrain / landing

- terrain clearance;
- predictive `TERRAIN` / `PULL UP`;
- sink-rate / don't-sink;
- landing configuration warnings;
- radio/radar-altitude callouts.

### Slice D — spaceflight

- thermal/entry envelope;
- structural load envelope;
- conjunction/collision;
- rendezvous/docking closure;
- mission-specific avionics packages.

## References

- FAA AC 25.1322-1, *Flightcrew Alerting*: https://www.faa.gov/documentLibrary/media/Advisory_Circular/AC_25.1322-1.pdf
- FAA AC 23-18, TAWS/GPWS installation guidance: https://www.faa.gov/documentlibrary/media/advisory_circular/ac_23-18.pdf
- Existing control/GNC design: `docs/18-control-guidance-autopilot.md`
- Authoritative mode definitions: `crates/flight-authority/src/mode.rs`
- Authoritative runtime telemetry/state: `crates/flight-authority/src/runtime.rs`
