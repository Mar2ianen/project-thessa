# 02 — Gameplay vision and current boundary

## Status

**Design / future work.** This document describes the intended factory,
logistics, and aerospace loop. The current repository implements the physics,
pilot, automation, world-generation, and server slices that support the loop;
it does not yet implement the complete factory game.

## 2.1. Core loop

The intended loop is:

```text
survey world → extract resources → refine and assemble → design vehicle
→ fly/transport cargo → build better infrastructure → extend logistics
```

Orbital mechanics is part of the logistics cost. Mass, volume, loading time,
propellant, launch cadence, transfer windows, atmosphere, and vehicle control
should remain connected rather than become independent minigames.

## 2.2. Starting situation

The player starts with a small industrial foothold on Thessa, a moon of Nereid.
The early game should teach resource extraction, power, storage, a local route,
and a first physical launch. The exact narrative hook, inventory model, and
player progression are still open questions.

## 2.3. Production model

The planned building families are:

- extraction and survey;
- power generation and storage;
- processing, smelting, chemistry, and assembly;
- tanks, depots, warehouses, and cargo handling;
- vehicle design, testing, and maintenance;
- communication, navigation, and flight operations.

Production is intended to be constrained by real throughput: mass, volume,
energy, heat rejection, loading/unloading, transportation distance, and
available vehicles. Conveyor tiers or abstract teleportation must not hide
physical transport where transport is a decision-relevant part of the game.

No production, inventory, power, or persistence runtime exists yet. Current
resource names and world data are configuration inputs for future systems.

## 2.4. Vehicle design

The intended editor is parametric rather than a list of hundreds of fixed
parts. A design can include:

- fuselage, tanks, wings, control surfaces, fairings, bays, frames, gear,
  hinges, radiators, intakes, and engine mounts;
- propulsion parameters such as propellant pair, chamber/expansion class,
  throttle/restart range, gimbal, cooling, and efficiency;
- actuator, fluid, electrical, structural, and thermal connectivity.

The implemented `VehicleDefinition` and `vehicle-baker` cover the early shared
asset boundary: geometry, mass/inertia, aero panels, control surfaces, and
starter propulsion/control data. The editor, full compilation pipeline,
damage topology, and reusable design database are future work.

## 2.5. Flight and fly-by-wire

The player provides pitch, yaw, roll, throttle, and translation demands. The
current control stack resolves them through guidance, control law, flight
policy, allocation, and actuator dynamics. Aerodynamic surfaces, RCS,
propulsion, and future effectors must produce the result physically.

Available current control concepts include manual axes, attitude/rate
guidance, aircraft/spacecraft laws, envelope limits, RCS, control surfaces,
and typed trajectory-plan guidance. High-level guidance does not directly add
force or moment to the craft.

## 2.6. Surface transport

Future trucks, trains, and aircraft should remain physical world objects:

1. a player or planner creates a route/corridor;
2. the vehicle follows it by commanding throttle, steering, brakes, or flight;
3. stations physically load and unload cargo;
4. route failures come from geometry, terrain, capability, or scheduling;
5. route automation uses the same event-driven graph layer as flight.

The current world generator and obstacle certification provide groundwork for
this direction, but no production route scheduler is shipped.

## 2.7. Space logistics

Spacecraft remain transport entities. The intended system includes launch
windows, depots, reusable stages, transfer planning, gravity assists,
refuelling, and physical finite burns. The current maneuver crate implements
planning helpers and typed plans; two-body calculations are search
approximations and must be revalidated through the exact multi-body field.

Warp changes simulation time globally under server policy. It may batch a
craft only when explicit atmosphere, contact, obstacle, control, and error
conditions allow it.

## 2.8. Automation

The current automation layer is a typed graph IR with sequence, parallel,
wait/event, validation, failure, and server-owned continuations. QuickJS can
produce typed graph values inside a restricted host.

The intended standard library contains reusable blocks such as `Ascent`,
`TargetOrbit`, `Circularize`, `PlanTransfer`, `ExecuteManeuver`, `LandAt`,
`Rendezvous`, `Dock`, `Load`, `Unload`, `Refuel`, and `SetAlarm`. Several
guidance/plan primitives exist; the complete flight and logistics standard
library does not.

Staging must be able to create multiple `VehicleId` branches. A detached
booster remains a physical vehicle that can be routed to recovery, while the
upper stage continues its own graph branch.

## 2.9. Power and propulsion progression

The design direction is:

1. chemical propulsion and conventional power;
2. nuclear thermal propulsion and fission power;
3. nuclear electric/ion propulsion;
4. advanced fusion and torch-class late game.

These are gameplay progression goals, not implemented engine catalogs. Any
future propulsion feature must feed mass, thrust, propellant, heat, power, and
actuator contracts rather than grant class-based performance multipliers.

## 2.10. Failure and maintenance

The intended failure model includes wear, thermal damage, propellant/energy
shortage, actuator saturation, structural damage, and service/maintenance.
Damage should alter physical capability and topology rather than automatically
explode or despawn an object.

Structural and thermal graphs are not in the current runtime, so this section
remains a design constraint for future implementation.

## 2.11. Multiplayer direction

The current server shell already owns simulation time, client commands,
autopilot execution, snapshots, and consensual warp votes over stdio/TCP.
Production multiplayer still needs authentication, persistence, interest
management, prediction/interpolation policy, fleet replication, and deployment
packaging.

## 2.12. Release vision

The smallest meaningful release should demonstrate one causal loop:

```text
factory → local logistics → vehicle design → physical flight
→ orbital/intermoon logistics → factory
```

Content count is less important than proving that the same state, units, time,
and physical actuator path survive the whole loop.
