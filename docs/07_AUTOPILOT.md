# 07 — Autopilot, guidance graphs and automation

## 7.1. Goal

Сохранить сильную UX-идею MechJeb: типовая сложная операция доступна как понятный high-level autopilot action. При этом убрать ограничение «каждый autopilot — отдельное окно/режим»: actions должны свободно комбинироваться в один graph и работать тем же механизмом, что industrial logistics automation.

Reference vocabulary: MechJeb Ascent Guidance, Maneuver Planner/Node Executor, Landing Guidance, Rendezvous/Docking Guidance, SmartASS-like attitude targeting. Это UX/reference vocabulary, не code dependency.

## 7.2. Layering

```text
visual graph
  ↓ compile/validate
typed automation IR
  ↓
event-driven VM / scheduler
  ↓
planner + guidance laws
  ↓
FBW/control allocator
  ↓
physical actuators
  ↓
physics
```

Script никогда не добавляет магическую силу/момент. Даже `LandAt` в итоге выдаёт guidance targets, а control allocator управляет реальными engines/surfaces/RCS.

## 7.3. Standard-library blocks

### Attitude / control

- `Point(direction/frame)`
- `HoldAttitude`
- `HoldRate`
- `HoldAoA`
- `HoldG`
- `SetThrottle`
- `Translate`

### Orbital planning / execution

- `TargetOrbit`
- `Circularize`
- `ChangePlane`
- `PlanTransfer`
- `ExecuteManeuver`
- `MatchVelocity`
- `Rendezvous`
- `Dock`

### Flight phases

- `Ascent`
- `Stage/Separate`
- `Boostback`
- `AtmosphericEntry`
- `LandAt`
- `RecoverBooster`

### Logistics

- `WaitForWindow`
- `WaitForCargo`
- `Load` / `Unload`
- `Refuel`
- `DepartRoute`
- `SetAlarm`
- `WarpRequest`

## 7.4. Composition primitives

Graph language requires:

- `Sequence`;
- `If/Switch`;
- `WaitUntil/Event`;
- `Loop`;
- `Retry`;
- `Fallback`;
- `Parallel/Fork`;
- `Join`;
- parameterized reusable `Subgraph`;
- explicit error/abort path.

Blocks have typed ports and contracts. Examples: `VehicleId`, `Target`, `OrbitGoal`, `PadId`, `CargoFilter`, `ManeuverPlan`, `Window`.

## 7.5. Staging and parallel vehicles

Staging changes world topology. A block such as:

```text
Stage
```

may return:

```text
{ parent_or_upper: VehicleId, detached: [VehicleId...] }
```

The graph can immediately fork:

```text
Ascent
→ Stage
  ├ booster -> RecoverBooster(PadA)
  └ upper   -> TargetOrbit(120 km)
              -> WaitForWindow(Pelagos)
              -> PlanTransfer
              -> ExecuteManeuver
```

This is required for physical reusable launch cadence: booster recovery is not a background inventory operation.

## 7.6. Event-driven execution

Do not poll every graph every physics tick. `WaitUntil` should compile to the cheapest valid wake source:

- exact `SimTime`;
- scheduled orbital event;
- threshold watcher with known next-check policy;
- cargo/inventory event;
- contact/staging/docking event;
- guidance completion/failure event.

High-rate controller loops are separate guidance/control systems activated only while needed.

## 7.7. User-facing complexity

Three levels share one runtime:

1. **preset:** choose `Launch to orbit` / `Land at pad`;
2. **graph:** compose standard blocks;
3. **low-level:** math/sensors/controllers/direct actuator commands.

A normal player should automate a reusable booster without writing PID math. An advanced player must be able to replace high-level blocks with their own subgraphs/controllers.

## 7.8. Validation

Before run, graph compiler checks where possible:

- type compatibility;
- missing target/vehicle handles;
- impossible obvious resource requirements;
- branch ownership conflicts (two controllers commanding the same actuator set);
- cycles without wait/yield where relevant;
- unavailable technology/sensors.

Runtime failures remain possible because physics is real: insufficient thrust, actuator saturation, thermal damage, missed window, collision, etc.
