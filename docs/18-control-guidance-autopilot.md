# Unified control, guidance, and autopilot architecture

Status: partially implemented design record.

The original refactor is now represented by `thessa-flight-control`,
`thessa-flight-authority`, `thessa-autopilot`, `thessa-autopilot-js`,
`thessa-flight-net`, and `thessa-maneuver`. Guidance intents, aircraft/
spacecraft/direct laws, policy, allocation, actuator dynamics, typed graph
execution, simulation-time waits, QuickJS limits, and plan execution are
implemented prototypes. This document retains the detailed design rationale;
the current behavior and missing pieces are summarized in
[`07_AUTOPILOT.md`](07_AUTOPILOT.md) and
[`04_RUNTIME_ARCHITECTURE.md`](04_RUNTIME_ARCHITECTURE.md).

This document defines the control refactor that collapses the current SAS/assist split into one guidance-and-control stack, separates aircraft and spacecraft control laws, and adds a MechJeb-like programmable automation layer backed by QuickJS.

The goal is to keep the authoritative physics and low-level control path in Rust while making high-level vehicle automation composable, scriptable, deterministic, and cheap enough to scale to large fleets.

## 1. Design goals

The flight stack should have one explicit direction of authority:

```text
pilot input / autopilot graph
          |
          v
       guidance
   "what do I want?"
          |
          v
      control law
 "what wrench is needed?"
          |
          v
   flight policy / FBW
    "what is allowed?"
          |
          v
    control allocator
   "what can produce it?"
          |
          v
   actuator dynamics
          |
          v
       sim-core
```

The architecture should:

- remove SAS as a special parallel subsystem;
- separate input mapping, guidance, control law, policy, allocation, and physical actuators;
- support both aircraft and spacecraft control on the same vehicle;
- make differential thrust, TVC, RCS, reaction wheels, control surfaces, and later effectors first-class allocator inputs;
- keep low-level loops deterministic and native Rust;
- expose a typed, sandboxed high-level automation API to JavaScript;
- represent automation visually as a typed dataflow graph inspired by MechJeb/Scratch and Unix pipelines;
- park automation at `await` boundaries in the Rust scheduler rather than polling scripts every physics tick;
- preserve the server-authoritative model and remain compatible with trajectory certification / baking.

## 2. Original problem statement

The current `ControlMode` combines concepts from different layers:

```rust
pub enum ControlMode {
    MouseAim,
    Navball,
    Rate,
    Direct,
}
```

`MouseAim` is an input scheme, `Navball` mostly selects an attitude target, `Rate` is a control-law choice, and `Direct` bypasses assistance. At the same time, `sas_enabled` independently enables attitude hold for selected modes.

The current runtime then performs several logically distinct jobs in one path:

1. interpret pilot axes;
2. maintain / update an SAS attitude target;
3. derive desired angular rates;
4. derive a desired body moment from inertia and angular-rate error;
5. trim aerodynamic control surfaces;
6. send residual moment demand to RCS;
7. track actuator saturation.

This behavior is useful and should be preserved, but the abstraction boundary should be changed.

## 3. Core data model

### 3.1 Pilot input is not a control law

Physical devices and UI produce a normalized pilot command. They do not directly select actuator behavior.

```rust
pub struct PilotAxes {
    pub pitch: f64,
    pub yaw: f64,
    pub roll: f64,
    pub translation: glam::DVec3,
    pub propulsion: f64,
}
```

Input schemes such as mouse steering, navball steering, keyboard axes, HOTAS, or gamepad are client/UI concerns. Their result is a guidance input, not actuator state.

### 3.2 Guidance intent

Guidance expresses what the vehicle should do without describing how the vehicle achieves it.

```rust
pub enum GuidanceIntent {
    ManualAxes(PilotAxes),

    AngularRate {
        rate_body_rps: glam::DVec3,
    },

    Attitude {
        target_body_to_inertial: glam::DQuat,
        roll_policy: RollPolicy,
    },

    VelocityDirection {
        direction: DirectionTarget,
        roll_policy: RollPolicy,
    },

    FlightPath {
        target: FlightPathTarget,
    },

    Trajectory {
        plan: TrajectoryPlanId,
    },
}
```

Examples that were traditionally SAS modes become guidance producers:

- attitude hold;
- prograde / retrograde;
- normal / anti-normal;
- radial in / out;
- surface prograde;
- target / anti-target;
- heading / pitch / roll hold;
- ascent guidance;
- landing guidance;
- docking guidance;
- trajectory-plan following.

SAS therefore stops existing as a special boolean subsystem. "SAS" in the UI may remain as a familiar label for a set of simple guidance modules, but internally it is just guidance.

### 3.3 Control demand

Control laws convert guidance into desired physical action.

```rust
pub struct ControlDemand {
    pub force_body_n: glam::DVec3,
    pub moment_body_nm: glam::DVec3,
    pub propulsion: PropulsionDemand,
}
```

This is the main interface between control logic and actuator allocation.

## 4. Aircraft and spacecraft control laws

Control laws are separated by control semantics rather than by hard-coded vehicle class. A spaceplane may use both.

```rust
pub enum FlightControlLaw {
    Aircraft(AircraftControlLaw),
    Spacecraft(SpacecraftControlLaw),
    Direct(DirectControlLaw),
}
```

### 4.1 Aircraft control

Aircraft control interprets pilot input in aerodynamic terms. Exact modes can evolve, but the intended semantics are approximately:

```text
pitch -> pitch-rate / normal-load / AoA demand
roll  -> roll-rate demand
yaw   -> yaw-rate / sideslip demand
```

Possible protections and assists include:

- angle-of-attack limiting;
- positive and negative g limiting;
- overspeed protection;
- sideslip suppression;
- coordinated-turn assistance;
- automatic trim;
- stability augmentation.

The current aerodynamic surface solver and effectiveness-matrix logic should move behind this interface rather than remain embedded in the flight runtime.

### 4.2 Spacecraft control

Spacecraft control primarily converts attitude or angular-rate targets into desired body torque, then optionally translation targets into desired body force.

The existing attitude controller already provides the seed behavior:

```text
attitude target
      -> desired angular rate
      -> desired angular acceleration
      -> desired body moment
```

The spacecraft controller must not care whether the requested moment will be produced by RCS, TVC, reaction wheels, differential thrust, aerodynamic surfaces, or a combination.

### 4.3 Switching / blending

A vehicle can switch or blend control laws based on configuration and flight condition.

Examples:

```text
high dynamic pressure -> aircraft law
low dynamic pressure  -> spacecraft law
```

The transition may be automatic, pilot-selected, or vehicle-specific. This is not a `VehicleKind::Plane` / `VehicleKind::Rocket` distinction.

## 5. Flight policy and avionics

Physical capability and avionics permission are separate concepts.

A cockpit, flight computer, or avionics package may supply policy such as:

```rust
pub struct FlightPolicy {
    pub max_aoa_rad: Option<f64>,
    pub max_positive_g: Option<f64>,
    pub max_negative_g: Option<f64>,
    pub reverse_airborne_allowed: bool,
    pub reverse_in_atmosphere_allowed: bool,
    pub augmentation_allowed: bool,
}
```

This makes cockpits and avionics meaningful gameplay components without changing the physical vehicle underneath them.

Policy may constrain or reshape a `ControlDemand`, but it must not mutate rigid-body state directly.

## 6. Propulsion demand

Propulsion input is a vehicle-level demand, not necessarily one engine's throttle.

The intended command range can expose meaningful values below zero and above nominal full thrust:

```text
-20% ... 0% ... 100% ... 120%
 reverse        normal      augmentation
```

`100%` means normal full propulsion demand. `120%` means augmented propulsion and does not imply exactly 1.2x thrust. Depending on the vehicle it may mean:

- afterburner;
- auxiliary chemical engines in addition to ion / nuclear propulsion;
- additional booster engines;
- RCS or auxiliary-thrust augmentation.

Negative demand may be fulfilled by reverse thrusters, retro engines, or another reverse-capable system. Flight policy may inhibit it in inappropriate regimes.

Input mapping remains separate. A KSP-like binding may use `Z` for immediate 100% demand and deliberate double-tap `ZZ` for immediate augmentation.

## 7. Universal control allocator

The control allocator receives a desired wrench and propulsion demand and maps them onto available effectors.

Potential effectors include:

- aerodynamic control surfaces;
- RCS thrusters;
- thrust-vector control;
- differential engine thrust;
- reaction wheels;
- airbrakes / spoilers;
- dedicated reverse thrusters;
- later, slow trim effectors such as movable ballast or fuel transfer.

Every effector exposes its contribution and limits in a common form. For thrust-producing effectors, torque naturally follows from `tau = r x F`.

The allocator should own:

- effectiveness evaluation;
- constraints and saturation;
- prioritization / weighting;
- actuator-group availability;
- residual error reporting;
- allocation across mixed effectors.

The controller should never contain branches such as "if RCS is enabled, add RCS moment". It requests a wrench; the allocator decides how to realize it.

The existing behavior — solve aerodynamic surfaces, evaluate the achieved aerodynamic moment, then assign the residual to RCS and report saturation — should be preserved as the first implementation of the generalized allocator.

## 8. Actuator dynamics

Actuator dynamics remain below allocation.

Examples:

- control-surface slew rate;
- engine spool / ignition delay;
- gimbal slew;
- valve / RCS response;
- reaction-wheel torque and momentum limits.

The allocator outputs actuator targets. The actuator layer advances the physical actuator state at the world tick.

## 9. High-level autopilot graph

High-level automation is a typed directed graph inspired by MechJeb/Scratch visually and Unix pipelines semantically.

A block is conceptually a small process:

```text
+-------------------------+
| ascent-planner.js       |
|                         |
| stdin  <----            |
| stdout ---->            |
| stderr ---->            |
|                         |
| status: 0               |
+-------------------------+
```

The Unix analogy is intentional:

- `stdin` is the primary typed input bus;
- `stdout` is the primary typed result bus;
- `stderr` is a separate diagnostic / fault stream;
- blocks may be connected into pipelines;
- graph helpers may visually resemble `pipe`, `tee`, merge, filter, sink, redirect, and `2>&1`.

This is not a byte-stream ABI. The buses are typed.

### 9.1 Typed ports

`stdin` / `stdout` may contain named typed subports instead of forcing everything through one tuple.

Example:

```text
stdin
 |- target: Body
 |- state: VehicleState
 `- limits: FlightEnvelope

stdout
 |- attitude: AttitudeTarget
 `- propulsion: PropulsionTarget
```

Useful graph-level types include:

- `VehicleState`;
- `Body`;
- `OrbitTarget`;
- `TrajectoryPlan`;
- `AttitudeTarget`;
- `AngularRateTarget`;
- `FlightPathTarget`;
- `PositionTarget`;
- `PropulsionTarget`;
- `DockingTarget`;
- `FlightEnvelope`;
- diagnostics and events.

Invalid connections should be rejected by the editor / graph compiler.

### 9.2 Diagnostics / stderr

`stderr` is typed internally even if it renders as console-like text.

```rust
pub enum Diagnostic {
    Warning { code: String, message: String },
    ConstraintViolation { name: String, value: f64, limit: f64 },
    NoSolution,
    Saturated { actuator_group: String },
    InvalidState,
}
```

Blocks also expose an execution status:

```rust
pub enum BlockStatus {
    Ok,
    Waiting,
    Failed,
    Degraded,
}
```

Numeric Unix-like exit codes may be exposed as UI / scripting sugar, but Rust should use typed status values.

## 10. JavaScript execution with QuickJS

High-level graph blocks may be implemented in JavaScript using `rquickjs` / QuickJS.

JavaScript is for high-level logic such as:

- ascent planning;
- landing planning;
- rendezvous planning;
- transfer planning;
- station keeping;
- docking sequencing;
- fuel / center-of-gravity management;
- mission sequencing;
- trajectory-plan construction.

JavaScript is not used for:

- 120 Hz servo loops;
- attitude-rate control;
- aircraft FBW inner loops;
- actuator allocation;
- actuator dynamics;
- authoritative physics.

Those remain Rust.

### 10.1 Host API rule

Scripts never receive mutable access to `FlightAuthority`, rigid-body state, or physical actuators.

Bad:

```js
craft.state.velocity.x = 1000;
craft.rcs[3].thrust = 1;
```

Good:

```js
return Guidance.attitude(target);
```

or:

```js
return Plan.thrustArc({
    duration: 84300,
    throttle: 0.72,
    direction: prograde,
});
```

Rust remains the only authority that mutates simulation state.

### 10.2 Determinism and sandboxing

The JS environment should expose simulation-domain APIs only.

No direct:

- filesystem;
- sockets;
- HTTP;
- wall-clock time;
- process execution;
- uncontrolled randomness.

If randomness is needed, expose a deterministic RNG derived from simulation state / world seed.

QuickJS runtime limits should include memory, stack, and an interrupt handler so runaway scripts can be terminated.

A single global VM for the entire server is undesirable. Prefer VM/runtime sharding across workers, with each craft or graph pinned to one shard while runnable.

## 11. `async` / `await` semantics

`await` is a boundary between JavaScript automation and the authoritative Rust scheduler.

A script reaching a simulation wait should become parked rather than polled every physics tick.

Conceptually:

```text
JS coroutine
    |
    | await sim event
    v
Rust creates WaitCondition
    |
    +-> EventScheduler / subsystem guard
    |
    `-> JS execution is parked

... simulation advances with no JS work ...

condition fires
    |
    v
Rust resolves pending promise
    |
    v
QuickJS job queue runs until the next await / completion
```

`await` must never mean `thread::sleep()` and should not imply JS-side per-tick polling.

### 11.1 Predictable waits

If the wake time is analytically or deterministically known, Rust schedules it directly.

Examples:

```js
await sim.sleep(300);
await flight.untilBurnEnd();
await flight.untilScheduledLaunch();
await trajectory.segmentEnd();
```

These become one scheduler event at a known `WorldTick`.

### 11.2 Event / guard waits

State-dependent waits register a cheap native event or guard in the owning subsystem.

Examples:

```js
await flight.untilAltitude(100_000);
await flight.untilApoapsis();
await flight.stageSeparated();
await flight.engineReady();
await docking.capture();
```

They do not run JS every tick.

### 11.3 Wait sets

Composite waiting should be supported:

```js
await any(
    flight.event("docked"),
    flight.event("impact"),
    sim.timeout(600),
);
```

Rust owns a `WaitSet`; the first completed condition resolves the promise and unregisters the remaining guards.

This makes large fleets cheap: thousands of scripts may exist while only a small runnable subset consumes CPU.

The model is deliberately similar to a Unix process sleeping in the kernel and being woken by an event.

## 12. Automation and trajectory baking

High-level automation should prefer producing declarative plans over continuously issuing low-level commands.

Example:

```text
Target Orbit
    |
    v
Ascent Planner
    | TrajectoryPlan
    v
Plan Validator / Follower
    |
    +-> AttitudeTarget
    `-> PropulsionTarget
```

A planner may emit long finite-burn arcs, coasts, attitude changes, low-thrust segments, and event guards.

Graph nodes / plans may advertise bakeability:

```rust
pub enum Bakeability {
    Pure,
    Guarded,
    Live,
}
```

- `Pure`: deterministic and fully bakeable;
- `Guarded`: bakeable while declared guards remain valid;
- `Live`: requires active execution.

This integrates with the existing rails / certification direction. Predictable automation becomes a compiled future rather than a permanently running controller.

Known non-certifiable events end the baked future ahead of the event, with the current design target of a 300-second live warm-up window before an inevitable live interrupt.

A deterministic burn is not itself an interrupt; it can be part of the baked plan.

## 13. Server authority and networking

The authoritative server owns:

- guidance state relevant to simulation;
- selected control law;
- flight policy;
- allocator state;
- actuator state;
- JS graph execution / waits;
- trajectory-plan validation and baking.

Clients send pilot intent and graph-edit / command operations, never authoritative actuator state or world state.

The legacy wire representation still carries implementation details such as
`ControlMode`, `sas_target_xyzw`, and `sas_enabled` for compatibility. Typed
guidance and autopilot messages now exist alongside it; a later cleanup may
remove the legacy fields after migration coverage is complete.

The client may still locally render guidance targets, modes, diagnostics, and graph state, but simulation decisions remain server-side.

## 14. Proposed crate / module boundaries

A possible split is:

```text
thessa-sim-core
  rigid-body physics, aero, gravity, actuator-facing primitives

thessa-flight-control
  guidance types
  aircraft controller
  spacecraft controller
  flight policy
  control allocator
  actuator command types

thessa-flight-authority
  world-tick orchestration
  scheduler integration
  controller/allocator execution
  rails / certified-flight integration

thessa-autopilot
  typed graph IR
  graph validation / compilation
  wait conditions
  plan types

thessa-autopilot-js
  rquickjs host bindings
  VM shards / sandbox
  promise <-> Rust wait bridge

client
  physical input mapping
  graph editor
  HUD / diagnostics
```

The exact crate split is not normative; the dependency direction is.

`sim-core` must not depend on QuickJS or UI. JavaScript must not own physics state.

## 15. Remaining work and historical refactor plan

Phases 1–7 below are implemented to prototype depth. They remain useful as a
checklist for missing edge cases and production hardening; phase 8 and the
complete graph/editor/library integration are future work.

The refactor should be behavior-preserving before new features are added.

### Phase 1: extract existing control behavior

Move the current attitude/rate controller, aerodynamic trim logic, residual RCS allocation, surface slew, and saturation reporting out of the monolithic flight runtime into explicit native control modules.

Keep regression tests pinning current behavior.

### Phase 2: split concepts

Replace the current mixed `ControlMode` model with separate concepts:

- input scheme;
- guidance intent;
- control law;
- flight policy;
- actuator allocation.

Remove `sas_enabled` as a simulation-level special case. Preserve familiar UI affordances through guidance modules if desired.

### Phase 3: aircraft / spacecraft controllers

Create explicit aircraft and spacecraft control laws while preserving the existing X-15 behavior through adapters and tests.

### Phase 4: generalized allocator

Generalize the current aerodynamic-surface + RCS path into the common allocator. Add interfaces for TVC and differential thrust even if the first implementation only has surfaces and RCS available.

### Phase 5: network protocol update

Introduce a new protocol version that transmits pilot / guidance intent rather than SAS implementation details. Do not keep `sas_target_xyzw` and `sas_enabled` as permanent wire concepts.

### Phase 6: graph IR

Add the typed autopilot graph independently of JavaScript. Native Rust blocks should be enough to validate graph semantics, execution, diagnostics, and waiting.

### Phase 7: QuickJS blocks

Add `thessa-autopilot-js` with strict host bindings, runtime limits, deterministic APIs, and Rust-backed `await`.

### Phase 8: trajectory-plan compilation

Allow high-level graph modules to emit plans that can be validated, certified, baked, and deoptimized back to live control when guards fail.

## 16. Non-goals

This design intentionally does not require:

- exposing raw physics mutation to scripts;
- running JavaScript at the physics tick rate;
- simulating avionics as electrical circuits;
- hard-coding every vehicle as aircraft or spacecraft;
- preserving SAS as a parallel magic subsystem;
- tying an autopilot graph to the renderer;
- requiring polling for waits;
- requiring a separate thread per sleeping script;
- making every MechJeb-like feature part of the native binary.

## 17. Design summary

The final model is:

```text
pilot / JS graph
      |
      v
   guidance
      |
      v
 aircraft | spacecraft controller
      |
      v
 flight policy
      |
      v
 universal allocator
      |
      v
 physical actuators
      |
      v
   sim-core
```

High-level automation behaves like a typed Unix pipeline. JavaScript blocks consume `stdin`, produce `stdout`, report diagnostics on `stderr`, and sleep at `await` points in the Rust scheduler. Rust owns the world, the low-level control loops, the allocator, and all physical state.

This keeps manual flight, FBW, SAS-like assists, MechJeb-like automation, large-fleet scheduling, and certified trajectories inside one coherent control architecture rather than growing as separate special cases.
