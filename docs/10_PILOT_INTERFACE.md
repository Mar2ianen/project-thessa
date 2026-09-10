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

The default layout follows the KSP reference: a 256 px navball at bottom center,
selected speed in its top cap and heading below. A compact dock places thrust
and utility icons on its left, altitude/vertical speed and SAS/RCS/gear/mode on
its right. Detailed air/orbit panels expand directly above the side instruments.
The mode picker opens beside its button. All actions have original vector icons.

All keyboard hints live in F1. F2 hides the HUD. Detailed air/orbit telemetry
is hidden until F3 or its chart button is selected. State buttons use shape,
labels and green illumination; warnings appear only when applicable.
Prograde/retrograde cues are geometric symbols using the selected velocity
reference and the same spherical projection as the navball texture. Invalid,
slow or rear-hemisphere vectors are hidden. There is no fabricated target cue.

The client-local X-15 uses sim-core at 120 Hz. The FBW allocator drives bounded,
slew-limited panel deflections and finite RCS force couples; controller moments
are requests, never forces added directly to the vehicle. SAS yields to manual
rate input, captures the achieved attitude, then brakes and holds after release.
There is no horizon/prograde restoring assist in vacuum. RCS authority comes
from three opposed prototype jet pairs (400 N per jet, 1.4/5 m arms), not an
X-15 fidelity claim. Flight tests cover fifteen minutes and full orbital turns.

All authoritative positions remain f64. For rendering, the aircraft is the
floating origin: its meshes stay at zero and the planet center is converted to
f32 only after subtraction in f64. The camera rotates by quaternion through
both poles without angle clamps, with optional craft-relative chase mode.

Fuel remains unlimited, gear has a state but no contact-force model, and the
spherical anti-burial boundary remains a prototype. AGL is unavailable without
terrain data. Negative periapsis altitude is displayed honestly. g-load uses
standard 9.80665 m/s²; TWR uses local gravity.

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

## 10.12. Implementation and controls

- `apps/client/src/pilot.rs`: input, f64 telemetry, preview scene, camera.
- `apps/client/src/pilot/control.rs`: fixed flight cadence, surface allocation,
  finite RCS couples, regression tests through the production path.
- `apps/client/src/pilot/hud.rs`: instruments, geometric cues, button actions.
- `crates/sim-core/src/vehicle.rs`: shared starter geometry; the client does not
  keep a second X-15 panel definition.

Controls match the README table: W/S nose down/up, A/D yaw left/right, Q/E roll;
M map, V camera, backquote reset, Caps Lock precision, F1 help, F2 UI,
F3 telemetry, Escape/F8 pause, T SAS, held F temporarily inverts SAS, R RCS,
G gear, Shift/Ctrl throttle, X/Z zero/full throttle, Space engine.
Speed/altitude frames and control mode are selected using instrument buttons.
Keys for features not implemented (quicksave, IVA, brakes, action sets) are not
reassigned to unrelated actions.

The old mouse option remains a rate-steering aid with a neutral dead zone,
labelled `MOUSE STEERING`; it does not claim a world-direction autopilot.
Mouse steering is suppressed over buttons, while operating the camera, while
help is open, on focus loss and during pause.

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

### 10.13 Compact flight dock and integrated worldgen

The bottom dock groups thrust and utility icons to the left of the navball,
altitude/vertical speed and SAS/RCS/gear/mode to the right. Optional telemetry
opens above those wings; F1 contains the illustrated icon/key legend. The
editable icon sources use a 24-unit SVG grid and 4x PNG exports.

The merged rocky generator's 16K texture is shared by map and flight materials.
The deterministic field remains an offline generator; runtime terrain mesh LOD
and collision sampling are not implied by loading its albedo texture.

### 10.14 Fast rotation regression

The 2026-09-09 capture stopped at 581.375 s after direct-control manoeuvres.
Explicit Euler added energy to the gyroscopic term even with zero external
moment. `sim-core` now solves

`I (omega1 - omega0) / dt = torque - omega_mid × (I omega_mid)`

with `omega_mid = (omega0 + omega1)/2`, then applies the paired normalized
Cayley quaternion `(dt*omega_mid/2, 1)`. State and units remain unchanged;
external aero/actuator moments are sampled once per bounded step. Newton's
three-variable solve has a fixed iteration limit and reports non-convergence.
No damping force, attitude clamp or modified inertia is introduced.

Torque-free energy and inertial angular momentum errors stay below 7e-13
over 60 seconds at the captured fast-spin scale. Replaying the recorded
commands reaches 641.375 s without the former stop (peak 10.796 rad/s).
The flight benchmark includes both cruise and fast asymmetric rotation.
