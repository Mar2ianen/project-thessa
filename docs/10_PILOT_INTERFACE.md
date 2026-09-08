# 10 — Pilot interface / Primary Flight Display

## 10.1. Goal

Режим пилотирования должен сохранить сильную сторону KSP: один компактный
инструмент позволяет одновременно понимать attitude, направление движения и
орбитальную геометрию. Но интерфейс не должен смешивать разные reference
frames и разные физические величины в один переключатель.

KSP-like navball используется как UX reference, не как буквальная копия.
Базовая идея Thessa:

```text
world/camera view
        │
        ├── flight director / markers
        │
        └── primary flight display
             ├ attitude indicator
             ├ heading/track
             ├ speed tape
             ├ altitude tape
             ├ vertical speed
             ├ flight/air data
             ├ propulsion/control state
             └ orbit/target summary
```

Главный инвариант: HUD показывает **derived telemetry**. Он не является
источником физики и не хранит authoritative vehicle state.

---

## 10.2. KSP reference и намеренные отличия

Полезные идеи KSP/navball:

- attitude sphere остаётся читаемой независимо от направления камеры;
- prograde/retrograde и maneuver marker видны прямо на attitude instrument;
- surface/orbit/target velocity modes дают один понятный ручной flight workflow;
- normal/antinormal/radial markers позволяют летать без отдельного avionics UI.

Что в Thessa лучше разделить:

1. **Attitude reference** и **velocity reference** независимы.
2. `surface`, `air`, `orbit`, `target` — разные velocity vectors, а не один
   магически меняющий смысл `speed`.
3. datum altitude и terrain clearance/AGL показываются отдельно.
4. heading (куда смотрит craft) и track/flight-path direction (куда он реально
   движется) не смешиваются.
5. aero telemetry (`AoA`, sideslip, Mach, q) появляется из atmosphere/aero
   model и не притворяется orbital navigation.
6. control/structural/thermal warnings отражают физические limits: actuator
   saturation, q/load margin, heating, terrain clearance, propellant state.

---

## 10.3. Reference frames

### Attitude reference

Пользователь может выбирать presentation frame независимо от speed mode:

- `LOCAL` — local tangent/horizon относительно body-fixed frame;
- `ORBIT` — orbital frame (`prograde`, `normal`, `radial` basis);
- `TARGET` — frame, связанный с target/relative geometry;
- `INERTIAL` — inertial reference для специальных операций.

Default около поверхности: `LOCAL`.

Default в устойчивом orbital flight: пользовательский последний выбор;
`ORBIT` предлагается, но не форсится.

### Velocity reference

Одновременно могут существовать:

- `SURFACE`: velocity относительно rotating body surface;
- `AIR`: velocity относительно локальной атмосферы/wind field;
- `ORBIT`: body-centered inertial velocity относительно reference body;
- `TARGET`: relative velocity к выбранному target.

UI выбирает одну величину как primary speed tape, но остальные могут оставаться
markers/readouts. Поэтому переключение speed mode не обязано менять attitude
sphere semantics.

---

## 10.4. Attitude indicator

### Baseline

Центральный instrument должен показывать:

- horizon / local vertical для `LOCAL`;
- vehicle forward/up axes;
- pitch ladder;
- roll scale;
- heading/azimuth;
- selected attitude reference;
- flight director cue / commanded attitude, если активен guidance/FBW.

### Vector markers

При наличии данных поверх indicator можно показывать:

- surface prograde / retrograde;
- air-relative flight-path vector / reverse;
- orbital prograde / retrograde;
- normal / antinormal;
- radial out / radial in;
- target / anti-target;
- relative-velocity marker;
- maneuver/guidance target.

Marker имеет явный reference/type. Один и тот же значок не должен менять
физический смысл молча.

### Low-speed behavior

Направление velocity плохо определено при очень малой скорости. Такой marker
не прыгает случайно: он гаснет/становится invalid ниже физически осмысленного
порога, который зависит от источника telemetry.

---

## 10.5. Altimeter

Одного `ALT` недостаточно.

### Datum altitude

`ALT DATUM`:

```text
vehicle reference point
minus reference ellipsoid/sphere datum
```

Для первого planetary implementation допустим spherical datum
`|r| - body.radius_m`. После terrain/shape model datum может стать ellipsoid /
reference geoid-like surface без изменения UI semantics.

Это аналог orbital/navigation altitude. Он **не гарантирует clearance над
рельефом**.

### Radar / terrain altitude

`AGL`:

- расстояние до реальной terrain/collision surface по local-down/radar model;
- может быть недоступно без terrain data/sensor solution;
- near-ground является primary landing altitude;
- позже sensor/raycast model может учитывать inclination, buildings и выбранную
  radar origin point.

Не заменять datum altitude одной кнопкой. Оба значения полезны одновременно.

### Atmospheric altitude

Если физическая atmosphere/sensor model это оправдывает, отдельно возможны:

- pressure altitude;
- density altitude;
- atmospheric layer/pressure readout.

Они не являются геометрической высотой.

### Vertical speed

`V/S` — projection relative velocity на local vertical. Знак и reference frame
фиксируются в telemetry contract.

При landing полезны дополнительно:

- terrain closure rate;
- predicted time-to-ground/impact при валидном решении;
- landing-site-relative altitude.

---

## 10.6. Speed / air data

Primary speed tape выбирается пользователем или flight phase preset.

Минимальные readouts:

- surface-relative speed;
- orbital/body-centered inertial speed;
- true airspeed, если атмосфера определена;
- Mach;
- target closing speed;
- vertical speed;
- optional ground track speed.

Для atmosphere flight рядом должны быть:

- dynamic pressure `q`;
- angle of attack;
- sideslip;
- optionally static pressure / density / speed of sound in debug/advanced view.

---

## 10.7. Orbit strip

При валидной osculating orbit solution компактно показывать:

- apoapsis datum altitude;
- periapsis datum altitude;
- time to apoapsis/periapsis;
- inclination;
- eccentricity where useful;
- next maneuver/event time;
- target encounter/relative data when selected.

Это summary того же trajectory/planner model, который используется map mode.
Pilot HUD не реализует вторую орбитальную математику.

Для escape/hyperbolic/non-osculating cases fields должны иметь нормальную
валидность, а не печатать фиктивный `AP`/`PE`.

---

## 10.8. Propulsion / control strip

Минимум:

- throttle command;
- actual thrust / available thrust later;
- local TWR when reference gravity meaningful;
- g-load / proper acceleration;
- active FBW/guidance mode;
- RCS state;
- actuator saturation warning;
- propellant summary;
- stage/vehicle topology status.

Не показывать `control authority = 80%` как arbitrary stat. Warning должен
происходить из реального allocator/actuator state.

---

## 10.9. Warning philosophy

Warnings derived from physical state:

- `ACT SAT` — allocator упёрся в actuator constraints;
- `THERM` — thermal margin violated/approaching limit;
- `STRUCT` — structural load margin;
- `Q` / aero load warning — dynamic pressure/load condition;
- `TERRAIN` — closure/clearance logic;
- `PROP` — feed/propellant limitation;
- `GUIDANCE` — commanded state currently unreachable/failed.

Color is secondary. Text/icon/shape must carry semantics so UI stays usable
without relying only on color perception.

## 10.10. Current vertical slice

The client currently exposes the PFD with the KSP-inspired default layout:

- primary speed and altitude tapes with moving display markers;
- a world-space preview scene with a compact X-15-like flight-test silhouette,
  three-engine plume, cockpit glass, and a lowered textured
  planet horizon so the PFD reads as a flight view rather than a system map;
- surface/air/orbital/target speed frames and datum/AGL altitude toggle;
- circular attitude instrument with pitch ladder, pitch labels, a visual
  heading-cardinal marker, roll scale, prograde/retrograde/target cues, a
  separated flight-path marker, and the mouse aim reticle;
- propulsion, target/orbit, control-mode, and flight-status cards;
- F6/P toggles map and pilot views, M/Shift+M changes control mode,
  V changes speed frame, B changes altitude frame, and the Pilot camera uses
  RMB look, MMB pan, wheel zoom and Home reset.

The current pilot has a client-local X-15 flight-test adapter. Its state is
advanced by the shared `sim-core` 6-DoF rigid-body integrator using the X-15
panel asset, Thessa gravity field and Thessa atmosphere. Fuel is intentionally
infinite in this vertical slice; engine staging, throttle, SAS/RCS and control
surface commands are live. Terrain collision, landing gear forces and network
authority are still outside this slice.

---

## 10.11. Data flow

Target architecture:

```text
authoritative vehicle state (f64/SI)
        │
        ├── body/reference-frame solution
        ├── atmosphere/aero solution
        ├── terrain/radar solution
        ├── orbit/target solution
        ├── propulsion/control solution
        └── thermal/structural solution
                    │
                    ▼
           PilotTelemetrySnapshot
                    │
                    ▼
          Bevy pilot HUD / PFD
```

The client read model must never be fed back into physics as state.

Networked client later receives an equivalent telemetry/snapshot contract from
server state plus local prediction. The current GPL client-local struct is a
prototype boundary; stable wire types move to `protocol` only after vehicle
state IDs and replication semantics are fixed.

---

## 10.12. Current implementation scaffold

`apps/client/src/pilot.rs` contains the first functional PFD shell:

- `FlightUiState` — client-side derived read model in SI, with position,
  attitude, independent speed/altitude frames, aero, orbit, target,
  propulsion and warning fields;
- `PilotHudState` — view/control state and Mouse Aim command hand-off;
- `PilotHudPlugin` — Bevy UI and input boundary;
- `F6` or `P` — switches between map/debug and Pilot view;
- `M` / `Shift+M` — cycles control modes forward/backward;
- `V` — cycles the primary speed reference (`surface`, `air`, `orbital`,
  `target`); `B` — toggles datum/AGL;
- `W/S` pitch, `A/D` yaw, `Q/E` roll;
- `Shift/Ctrl` throttle, `X` cutoff, `Z` full throttle, `Space` stage/engine;
- `T` SAS, `R` RCS, `G` landing gear;
- the current X-15 state is live telemetry; unavailable values still render as
  `--` rather than being replaced by fabricated flight data.

Stage 1 now renders one compact navball, speed and altitude panels, vehicle and
flight-context panels, a status block, and a mouse reticle. Map HUD and orbit
gizmos are hidden only while Pilot is active and return on `F6`. The navball is
an attitude-display shell with a shaded sphere, pitch ladder and labels, roll
scale, a live cardinal heading marker, and cue stubs; numeric attitude values
are not repeated in the altitude or propulsion cards, and it does not duplicate
a second large artificial horizon.

The remaining data-driven layers can later replace the readout contents without
changing the UI hierarchy:

```text
PilotHudRoot
├── SpeedTape
├── AttitudeIndicator
│    ├ horizon/pitch ladder
│    ├ roll/heading scale
│    ├ vector markers
│    └ flight-director cue
├── AltitudeTape
├── VerticalSpeed
├── AirDataStrip
├── OrbitStrip
├── PropulsionControlStrip
└── WarningStack
```

---

## 10.13. Implementation order

### P0 — contract / skeleton

- telemetry names and SI semantics;
- textual preview;
- datum vs AGL distinction;
- independent attitude/velocity references.

### P1 — first controllable rigid craft

Completed for the X-15 flight-test adapter; the next vehicle should use the
same asset boundary rather than a new pilot-only physics path.

- local tangent frame;
- attitude quaternion -> indicator;
- heading/pitch/roll;
- surface/orbital speed;
- spherical datum altitude;
- vertical speed;
- throttle/g-load;
- map <-> pilot view transition.

### P2 — terrain/atmosphere

Atmosphere, air-relative velocity and Mach/q/AoA are now connected for the
X-15 slice; terrain/radar and contact dynamics remain pending.

- AGL ray/terrain solution;
- air-relative velocity;
- Mach/q/AoA/beta;
- terrain warning;
- flight-path vector.

### P3 — orbital/guidance

- osculating orbit strip;
- target-relative markers;
- maneuver marker;
- autopilot/flight-director target cue;
- SmartASS-like manual attitude targets backed by physical FBW.

### P4 — serious vehicle physics

- actuator saturation;
- actual thrust/feed limits;
- thermal/structural margins;
- multi-cluster/staging status;
- landing guidance cues.

---

## 10.14. Non-goals for the first HUD

Do not block M1 on:

- simulated cockpit switches;
- photorealistic avionics screens;
- sensor noise/INS drift;
- certification-style aircraft symbology completeness;
- arbitrary skin/theme framework;
- per-manufacturer cockpit UI.

First prove that one consistent telemetry contract supports rocket, aircraft,
spaceplane and weird player-built craft without vehicle-class-specific hacks.
