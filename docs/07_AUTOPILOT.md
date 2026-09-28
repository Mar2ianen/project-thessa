# 07 — Autopilot, guidance graphs, and automation

Status: implemented vertical slice (typed IR, runner with wait-bounded-cycle
support, per-wait event generations, 4-block stdlib minimum, staged + B-plane
+ variational correction, L0–L2 replay fixtures in CI); L3–L4 replays, full
stdlib/editor, staging topology, and powered rails still future.

## Status

**Implemented vertical slice; not yet a finished subsystem.** The current tree has a
working typed graph IR and validator, deterministic native graph runner,
server-owned event/time waits, QuickJS sandbox and async continuation bridge,
typed guidance/control integration, trajectory-plan execution, maneuver search,
and authoritative server/wire integration.

The implemented pieces live primarily in `crates/autopilot`,
`crates/autopilot-js`, `crates/flight-control`, `crates/flight-authority`,
`crates/flight-net`, `crates/maneuver`, and `apps/server`.

The major remaining work is no longer proving that the architecture works. It is
production hardening and breadth: graph-loop runtime semantics, event-generation
semantics, a complete standard library/editor, physical staging and topology
changes, logistics automation, opportunistic powered-trajectory baking, and a
real-mission qualification suite.

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

The same authority path is used by manual flight, native graph blocks, QuickJS,
and maneuver-plan execution. This is intentional: automation does not get a
second physics API.

## 7.2. Current graph model

`AutopilotGraph` has typed nodes and ports. The validator currently checks:

- duplicate nodes and ports;
- required inputs and multiple drivers;
- port direction and type compatibility;
- bounded node/port/event names;
- controller ownership conflicts by `ActuatorGroup`;
- cycles that do not cross a wait-capable node.

`GraphRunner` executes ready nodes in deterministic `NodeId` order, keeps
independent branches runnable around parked waits, supports joins, stores typed
outputs, and exposes explicit complete/wait/fail/abort states.

The wire layer can submit validated graph IR to the authoritative server. The
server executes it against the same guidance/control path as manual input and
wakes waits from `SimTime` or named domain events.

### Wait-bounded cycles (shipped)

The validator deliberately allows a cycle when the strongly connected path
contains a wait/yield boundary, and the runner executes it in a single pass:
a cycle-closing edge into a wait-capable node is cut (`loopback_edges`),
readiness wiring ignores cut edges, and each iteration boundary is a parked
wait (`GraphRunner`, `lib.rs`). Cyclic graphs with wait boundaries are a
finished runtime feature with regression coverage.

## 7.3. Typed guidance and control

`GuidanceIntent` currently represents:

- manual pilot axes;
- angular-rate targets;
- attitude targets;
- velocity-direction targets in body, inertial, surface, orbit, and target
  frames;
- flight-path targets;
- trajectory-plan references.

The flight-control layer resolves guidance through aircraft, spacecraft, or
direct control laws, applies flight policy, allocates force/moment/propulsion
to real effectors, and applies actuator limits/dynamics.

The authority also supports typed propulsion targets with physical actuator
response rather than treating an autopilot throttle request as an instantaneous
force mutation.

## 7.4. Current standard-library surface

The current server/QuickJS bridge includes typed constructors for direction and
target-frame guidance, translation guidance, flight-path targets, maneuver
plans, physical burn commands, landing sites, impact sites, simulation-time
sleep, named events, and plan guards. QuickJS can return typed guidance, plans,
landing/impact sites, waits, and diagnostics.

Pure plans cannot silently contain live domain-event waits; the scheduler owns
those waits.

The following names remain the intended UX vocabulary and are not all shipped
as complete blocks yet:

- attitude: `Point`, `HoldAttitude`, `HoldRate`, `HoldAoA`, `HoldG`, `Translate`;
- orbital: `TargetOrbit`, `Circularize`, `ChangePlane`, `PlanTransfer`,
  `ExecuteManeuver`, `MatchVelocity`, `Rendezvous`, `Dock`;
- flight: `Ascent`, `Stage/Separate`, `Boostback`, `AtmosphericEntry`,
  `LandAt`, `RecoverBooster`;
- logistics: `WaitForWindow`, `WaitForCargo`, `Load`, `Unload`, `Refuel`,
  `DepartRoute`, `SetAlarm`, `WarpRequest`.

Shipped as parameterized native subgraphs in `thessa-autopilot` (each with
its event contract, pure guidance math, and runner coverage): `Ascent`
(`ascent`), `LandAt` (`landing`), `ExecuteManeuver` (`execute`), and
`Rendezvous` approach (`rendezvous`). `Rendezvous` ends at a guarded hold:
`Dock` stays unshipped until docking ports exist as vehicle hardware (no
capture mechanism, no docked topology may be promised before that).

## 7.5. Maneuver planning boundary

`thessa-maneuver` is intentionally below the graph VM and above low-level
control. It provides typed `ManeuverPlan` values and helpers for
circularization, Hohmann-class transfers, Lambert rendezvous, plane changes,
velocity matching, and candidate search.

The current porkchop path is staged rather than pretending one model is truth:

```text
broad Lambert grid
        ↓
local refinement / patched-conic energy pricing
        ↓
parking-orbit anomaly phasing
        ↓
full N-body propagation + differential correction
        ↓
corrected executable ManeuverPlan with measured miss
```

Broad search results are represented as `BroadRoute` and cannot be fed directly
to the executor. The cheap two-body/patched-conic stages scout candidate windows
and energies; only the corrected full-N-body result is an executable plan.
Correction uses variational-STM Newton Jacobians (no finite differences) with
a TCM prefix cache, plus 2D B-plane minimum-norm targeting per encounter
(`correct_shooting`, `correct_bplane_shooting`; legacy B-plane plus exact 3D
polish in chain legs).

The current search also handles the central-body/depot case explicitly instead
of starting a Lambert arc from the body's point-mass center, and uses the
central body state at the correct departure and arrival epochs when building
relative endpoints.

This separation is important for future interplanetary and interstellar
planners: pruning approximations may be aggressive, but physical truth remains
with authoritative propagation/certification.

## 7.6. Event-driven waits

Sleeping work is parked in the scheduler rather than polled each physics tick.
Wake sources include:

- an exact `SimTime` deadline;
- a named domain event;
- `Any` / `All` composite wait conditions;
- plan guard or guidance completion/failure;
- future cargo, staging, docking, or contact events.

`WaitSet` and `GraphRunner` remember events needed by a parked composite wait,
so an `All(time, event)` can observe the event first and later wake exactly at
the time deadline. QuickJS continuations are parked behind the same native wait
machinery.

### Per-wait event generations (shipped)

`TrajectoryPlanRunner` consumes event generations per wait: each wait observes
one generation of its event name, retired on wait completion. A second wait
for the same name therefore cannot complete from an earlier occurrence. Guards
can express "wait for `stage` generation > N", which also gives
baked/speculative execution a clean invalidation token.

## 7.7. Staging and ownership

Staging is a topology-changing operation. The intended graph result is a set of
new `VehicleId` branches, each with an explicit controller owner. A booster
recovery branch and an upper-stage transfer branch must be independently
schedulable.

The graph validator already prevents ambiguous controller ownership. Complete
physical separation, topology mutation, child-vehicle ownership transfer, and
multi-vehicle continuation are still future work.

## 7.8. JavaScript boundary

QuickJS is a high-level producer of typed values. The host exposes deterministic
constructors and denies ambient capabilities. It receives no mutable authority,
rigid-body handle, actuator reference, filesystem, network, or wall-clock
capability.

The current defaults bound source size, VM memory and stack, and synchronous
execution time. Async `sim.sleep` / event waits park a QuickJS promise in the
Rust scheduler; they do not sleep a worker or poll JS at the physics cadence.

The runtime owns continuation wakeup and can cancel pending tasks. The sandbox
therefore remains useful for large fleets: thousands of dormant scripts do not
imply thousands of active loops.

## 7.9. Trajectory plans, bakeability, and deoptimization

`TrajectoryPlan` currently contains declarative `Coast`, `Burn`, `Guidance`, and
`Wait` segments and advertises one of three bakeability classes:

```text
Pure     deterministic; intended to be fully bakeable
Guarded  bakeable while declared assumptions remain valid
Live     requires live execution
```

`TrajectoryPlanRunner` executes `Pure`/`Guarded` plans in `Baked` mode and can
`deoptimize(...)` to `Live` without losing its segment cursor. Existing reasons
include guard invalidation, manual override, and live interrupt.

That is already the control-flow boundary needed for certified future
execution: a baked future is an optimization, not authority. When an assumption
fails, execution falls back to the live control path at the same declarative
plan position.

## 7.10. Opportunistic powered-trajectory baking

The next major scaling feature should generalize "rails" from "unpowered vacuum
coast" to "a future state evolution that has already been computed and
certified".

When server CPU is idle and a deterministic or guarded maneuver is scheduled in
the future, a background worker may execute the expensive finite-burn physics
ahead of simulation time and cache a compact trajectory representation:

```text
future plan segment
      ↓
background exact integration while CPU is idle
      ↓
state / mass / attitude curve + error bounds + dependency certificate
      ↓
segment reaches authoritative SimTime
      ↓
validate certificate
  ┌───┴────────────┐
valid            invalid
  ↓                 ↓
serve baked        discard/deopt
trajectory         and execute live
```

This is deliberately opportunistic. Wasted precomputation is acceptable if it
uses otherwise-idle CPU; incorrect authority is not.

A powered baked segment should depend on at least the initial vehicle state,
vehicle/topology revision, mass/propellant state, relevant body/ephemeris
revision, maneuver-plan revision, actuator/engine availability, and any guard
event generations. Topology-changing or externally visible actions such as
staging, docking, cargo transfer, or vehicle creation remain commit boundaries
unless they are represented as deferred effects and revalidated at their
simulation time.

The important performance result is temporal load shifting. A server can use
quiet wall-clock periods to integrate tomorrow's burns, then serve hundreds of
simultaneous scheduled maneuvers as baked trajectories instead of forcing every
craft back onto per-tick live integration at the same instant.

The existing `BakeQueue`, coast rails, plan bakeability, guard/deoptimization
model, and exact maneuver propagation provide the pieces; powered rails should
reuse them rather than create a second autopilot runtime.

## 7.11. User-facing levels

One runtime should support:

1. presets such as `Launch to orbit` or `Land at pad`;
2. visual graphs built from standard blocks;
3. advanced typed graphs and low-level sensors/actuators.

The implementation is complete enough for the graph/plan vertical slice, not
for the final editor UX or logistics library.

## 7.12. Correctness record (both known bugs fixed)

Two runtime bugs were identified and fixed, with regression coverage:

1. **wait-containing cycles deadlocked at first execution** — fixed by cutting
   cycle-closing edges into wait-capable nodes (`loopback_edges`; single-pass
   execution, §7.2).
2. **trajectory-plan event memory too broad** — fixed by per-wait event
   generations (`seen_events` retired on wait completion, §7.6; multi-generation
   burnout test in `execute`).

No further open loop/event-model bugs are known; expanding the runtime no
longer waits on this section.

## 7.13. Current validation

Current tests cover graph type checking, ownership, wait boundaries, stable
sequence/parallel execution, event memory, scheduler wake/repark, QuickJS
limits and continuations, guidance parsing, plan validation/deoptimization,
wire round trips, and server execution through the authority.

The core local test entry points remain:

```bash
cargo test -p thessa-autopilot
cargo test -p thessa-autopilot-js
cargo test -p thessa-maneuver
cargo test -p thessa-server
```

Unit tests are necessary but not sufficient for the final maneuver/autopilot
stack. L0–L2 mission replays now run in CI (`mission_replays.rs`,
`mission_replays.toml`; millisecond-class); L3–L4 remain the slower
qualification target.

## 7.14. Real-mission replay qualification suite

The final autopilot/maneuver qualification should include Solar-System scenarios
that are close to real missions rather than only synthetic Hohmann/Lambert unit
cases. The goal is not to reproduce every historical trajectory-correction
maneuver or DSN navigation estimate exactly. The goal is to reproduce the
mission architecture with real ephemerides and realistic event ordering closely
enough that the planner must solve the same class of problem.

Each replay should pin mission milestones such as encounter body, encounter
order, epoch/window tolerance, closest-approach or target-orbit class, arrival
`v_inf` / C3 where meaningful, total correction budget, final capture/escape
state, and accumulated miss. The same scenario should be runnable through
broad search, exact correction, declarative plan execution, live authority, and
powered-rails replay; those paths must agree within declared tolerances.

### Shipped fixtures (L0–L2)

Data-driven fixtures in `data/mission_replays.toml`, run by
`crates/maneuver/tests/mission_replays.rs` (must stay millisecond-class for
CI): Apollo Earth–Luna, Earth–Mars/Venus/Mars-return/Jupiter/Mercury/Saturn
Hohmann-class directs, Mariner-10 Earth–Venus–Mercury tour, Voyager-1
Earth–Jupiter–Saturn chain, Voyager-2 Grand Tour chain. Fixtures use
design-relative windows with ~10% headroom caps and declare bodies, windows,
and miss budgets — no hard-coded mission scripts.

The 12-mission table below stays the target set: Pioneer 10/11, New Horizons,
Galileo, Cassini, MESSENGER, BepiColombo, Juno, Solar Orbiter, and Lucy have
no fixtures yet, and L3–L4 (deterministic DSMs, finite burns, powered-rails
equivalence) remain future.

### Core replay set (target)

| Mission | Reference sequence | What it qualifies |
| --- | --- | --- |
| Pioneer 10 | launch -> Jupiter flyby -> solar escape | minimal outer-planet flyby, high-energy escape, long coast |
| Pioneer 11 | launch -> Jupiter -> Saturn -> solar escape | sequential gravity assists and retargeting after the first encounter |
| Voyager 1 | launch -> Jupiter -> Saturn -> escape | high-energy two-assist transfer with long interplanetary coasts |
| Voyager 2 | launch -> Jupiter -> Saturn -> Uranus -> Neptune -> escape | flagship Grand Tour: four planetary encounters whose errors compound across twelve years |
| New Horizons | launch -> Jupiter gravity assist -> Pluto -> Arrokoth | very high launch energy, one major assist, multi-year coast, distant small-body targeting |
| Galileo | launch -> Venus -> Earth -> Earth -> Jupiter orbit insertion | repeated assists at the same body, resonant return geometry, final capture |
| Cassini-Huygens | launch -> Venus -> Venus -> Earth -> Jupiter -> Saturn orbit insertion | VVEJGA chain, mixed inner/outer Solar-System assists, final giant-planet capture |
| MESSENGER | launch -> Earth -> Venus -> Venus -> Mercury -> Mercury -> Mercury -> Mercury orbit insertion | repeated energy-removing assists into a deep solar gravity well, then capture |
| BepiColombo | launch -> Earth -> Venus x2 -> Mercury x6 -> Mercury capture | nine planetary flybys plus solar-electric low-thrust cruise; strongest combined assist/low-thrust qualification |
| Juno | launch -> deep-space maneuvers -> Earth gravity assist -> Jupiter orbit insertion | planned deep-space burns, Earth assist, long coast, large capture burn |
| Solar Orbiter | launch -> Venus/Earth assists -> repeated Venus assists | repeated gravity assists used primarily to reshape perihelion and increase heliocentric inclination |
| Lucy | launch -> repeated Earth assists -> main-belt/Trojan encounters -> Earth return -> opposite Trojan swarm | resonant Earth returns, deterministic DSMs, multiple small-body targets, long-lived multi-encounter plan |

Voyager 2 should be the canonical "does the whole gravity-assist planner actually
work?" acceptance case. BepiColombo should be the canonical hybrid
low-thrust + repeated-flyby case. Galileo/Cassini/MESSENGER are especially useful
because they force repeated encounters with Earth/Venus/Mercury instead of
letting the planner succeed with one lucky slingshot.

### Fidelity levels

A useful progression is:

```text
L0  event order only
L1  historical encounter windows and bodies
L2  realistic flyby altitude / v_inf / C3 / capture class
L3  approximate deterministic deep-space maneuvers and finite burns
L4  historical ephemerides + powered-rails bake/replay equivalence
```

L0-L2 are cheap CI scenarios (shipped, §Shipped fixtures). L3-L4 can run as slower qualification or
benchmark jobs. Exact historical reconstruction may use published SPICE kernels
or equivalent source ephemerides later; the autopilot architecture should not
depend on hard-coded mission scripts.

### Reference sources for scenario construction

Use primary mission sources when turning these scenarios into fixtures:

- NASA Voyager mission and Voyager 2 timeline:
  <https://voyager.gsfc.nasa.gov/mission.html> and
  <https://science.nasa.gov/mission/voyager/voyager-2/>
- NASA Pioneer 10 and Pioneer 11:
  <https://science.nasa.gov/mission/pioneer-10/> and
  <https://science.nasa.gov/mission/pioneer-11/>
- NASA New Horizons:
  <https://science.nasa.gov/mission/new-horizons/>
- NASA Galileo:
  <https://science.nasa.gov/mission/galileo/>
- NASA Cassini trajectory:
  <https://science.nasa.gov/resource/interplanetary-trajectory/>
- NASA MESSENGER:
  <https://science.nasa.gov/mission/messenger/>
- ESA BepiColombo journey/factsheet:
  <https://www.esa.int/Science_Exploration/Space_Science/BepiColombo/BepiColombo_factsheet>
- NASA Juno:
  <https://www.jpl.nasa.gov/missions/juno/>
- ESA Solar Orbiter flybys:
  <https://www.esa.int/Science_Exploration/Space_Science/Solar_Orbiter/Solar_Orbiter_perihelia_and_flybys>
- NASA Lucy mission planning/timeline:
  <https://science.nasa.gov/mission/lucy/>

The replay suite should remain data-driven. A mission fixture declares bodies,
epoch windows, burns/encounters and tolerances; it must not be a one-off code
path named `voyager2_special_case`.