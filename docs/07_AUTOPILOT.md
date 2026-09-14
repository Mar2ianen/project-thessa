# 07 — Autopilot, guidance graphs, and automation

## Status

**Implemented prototype.** The current graph IR, server runner, QuickJS bridge,
typed guidance, wait scheduler, and maneuver-plan execution are in
`crates/autopilot`, `crates/autopilot-js`, `crates/flight-control`,
`crates/flight-net`, `crates/maneuver`, and `apps/server`. The complete game
standard library and logistics automation remain future work.

## 7.1. Authority pipeline

```text
pilot input / native graph / QuickJS block
                    ↓
              typed graph IR
                    ↓
          server-owned scheduler
                    ↓
        planner and guidance intent
                    ↓
             control law/policy
                    ↓
             physical allocator
                    ↓
                actuators
                    ↓
                 physics
```

An autopilot block emits typed intent or a physical demand request. It does not
add a hidden force, moment, velocity change, or direct craft rotation.

## 7.2. Current graph model

`AutopilotGraph` has typed nodes and ports. The validator checks port types,
required inputs, controller ownership conflicts, bounded names, and cycles that
do not cross a wait boundary. The runner supports stable sequence order,
parallel branches, join behavior, event/time waits, and explicit failure or
abort outcomes.

The current wire layer can submit validated graph IR to the authoritative
server. The server executes it against the same guidance/control path as manual
input and wakes waits from `SimTime` or named domain events.

## 7.3. Typed guidance

`GuidanceIntent` currently represents:

- manual pilot axes;
- angular-rate targets;
- attitude targets;
- velocity-direction targets in inertial, surface, flight-path, or target
  frames;
- flight-path targets;
- trajectory-plan references.

The flight-control layer resolves guidance through aircraft, spacecraft, or
direct control laws, applies flight policy, allocates force/moment/propulsion
to real effectors, and applies actuator limits.

## 7.4. Current standard-library surface

The current server/QuickJS bridge includes typed constructors for direction and
target-frame guidance, translation guidance, flight-path targets, maneuver
plans, physical burn commands, landing sites, impact sites, simulation-time
sleep, named events, and plan guards. Pure plans cannot silently contain live
event waits; the scheduler owns those waits.

The following names remain the intended UX vocabulary and are not all shipped
as complete blocks yet:

- attitude: `Point`, `HoldAttitude`, `HoldRate`, `HoldAoA`, `HoldG`, `Translate`;
- orbital: `TargetOrbit`, `Circularize`, `ChangePlane`, `PlanTransfer`,
  `ExecuteManeuver`, `MatchVelocity`, `Rendezvous`, `Dock`;
- flight: `Ascent`, `Stage/Separate`, `Boostback`, `AtmosphericEntry`,
  `LandAt`, `RecoverBooster`;
- logistics: `WaitForWindow`, `WaitForCargo`, `Load`, `Unload`, `Refuel`,
  `DepartRoute`, `SetAlarm`, `WarpRequest`.

## 7.5. Maneuver planning boundary

`thessa-maneuver` is intentionally below the graph VM and above low-level
control. It provides typed `ManeuverPlan` values and helpers for circularization,
Hohmann transfers, Lambert rendezvous, plane changes, velocity matching, and
candidate search.

The planner uses a central two-body model for cheap operations/search. It is not
authoritative. Candidate plans are revalidated through the exact multi-body
field, and execution produces finite-time guidance/propulsion demands through
the authority rather than teleporting velocity.

## 7.6. Event-driven waits

Sleeping work is parked in the scheduler rather than polled each physics tick.
Wake sources include:

- an exact `SimTime` deadline;
- a named domain event;
- a composite wait condition;
- plan guard or guidance completion/failure;
- future cargo, staging, docking, or contact events.

The scheduler remembers events needed by composite waits and preserves event
order. A future graph compiler must reject unbounded busy loops and provide an
explicit abort path.

## 7.7. Staging and ownership

Staging is a topology-changing operation. The intended graph result is a set of
new `VehicleId` branches, each with an explicit controller owner. A booster
recovery branch and an upper-stage transfer branch must be independently
schedulable. The current graph validator already prevents ambiguous controller
ownership; complete physical separation and multi-body vehicle topology are
future work.

## 7.8. JavaScript boundary

QuickJS is a high-level producer of typed values. The host exposes deterministic
constructors and denies ambient capabilities. It receives no mutable authority,
rigid-body handle, actuator reference, filesystem, network, or wall-clock
capability.

Source size, execution time, continuation, and wait limits are enforced. The
server owns the simulation-time scheduler; QuickJS continuations are parked and
resumed by the host.

## 7.9. User-facing levels

One runtime should support:

1. presets such as `Launch to orbit` or `Land at pad`;
2. visual graphs built from standard blocks;
3. advanced typed graphs and low-level sensors/actuators.

The implementation is complete enough for the graph/plan vertical slice, not
for the final editor UX or logistics library.

## 7.10. Validation

Current tests cover graph type checking, ownership, wait boundaries, sequence/
parallel execution, event memory, scheduler wake/repark, script limits,
guidance parsing, plan validation, stale-plan aborts, wire round trips, and
server execution through the authority.

```bash
cargo test -p thessa-autopilot
cargo test -p thessa-autopilot-js
cargo test -p thessa-server
```
