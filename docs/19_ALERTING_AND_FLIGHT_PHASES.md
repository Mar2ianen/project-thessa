# 19 — Flight-aware alerting and aural warning system

Status: design target — `FlightRegime`/`ControlMode` context accurately
described; `FlightPhase`, alert/event protocol, manager, and Slices A–D
not implemented.

This document defines the warning architecture for Thessa crewed vehicles: from stall/overspeed/terrain to configuration, propulsion, thermal, and orbital warnings. The core principle: alerting is not a set of `if`s in the HUD and must not guess context on the client. Conditions, inhibition, debounce, latch, and priority are computed from the authoritative flight state; the client only visualizes and plays sound.

The system deliberately does not copy any specific Boeing/Airbus/Honeywell product. Aviation standards are used as a reference for warning/caution/advisory semantics and for minimizing nuisance alerts, while the texts, tones, and voice pack are the project's own asset data.

## 1. What already exists

`FlightAuthority` already stores almost all low-level inputs on which the first alerting slice should be built:

- `FlightRegime::{Aero, Coast}`;
- altitude / local-air kinematics;
- Mach, AoA, and dynamic pressure via `last_forces`;
- throttle and engine state;
- `gear_down`;
- control input and actual `surface_input`;
- `actuator_saturated`;
- SAS/RCS state;
- authoritative `flight_error`;
- ephemeris, gravity, terrain field, and world tick.

`FlightRegime` currently means a **solver regime**, not a mission stage: `Aero` is selected when density is sufficiently high, `Coast` — below `COAST_DENSITY_KG_M3`. This is a useful alerting input, but it must not be overloaded with values like `Approach` or `Landing`.

The current `ControlMode` is also not a flight phase: `MouseAim/Navball/Rate/Direct` describe the control method. After the control/GNC refactor, input mapping, guidance, control law, policy, and allocator must remain separate layers.

## 2. Invariants

1. **Physics first.** A warning follows from the physical state, envelope, capability, and mission context; it does not change the vehicle state.
2. **Server-authoritative state.** In networked play, the active flight-alert set is identical across all clients of a given vehicle.
3. **Sparse transitions.** Transitions / the current active set are transmitted over the network, not an audio event on every physics tick.
4. **No audio logic in physics.** The solver produces values and events. The alert manager decides what is active. Client audio decides how to play it.
5. **No nuisance spam.** Every condition has hysteresis/debounce/rearm semantics.
6. **Priority is explicit.** `PULL UP` must not wait while a configuration advisory finishes speaking.
7. **Capability-gated.** No cabin-altitude warning on an uncrewed shell without a pressurized cabin; no `TOO LOW GEAR` on a vehicle without landing gear.
8. **Data-driven vocabulary.** The alert set depends on the avionics package / vehicle definition, not on `VehicleKind::Airliner`.
9. **Deterministic.** Runtime TTS does not participate in the authoritative path. Voice lines and tones are pre-generated/supplied as assets.

## 3. Solver regime ≠ operational flight phase

Alert inhibition needs one more orthogonal layer — the operational phase. It must not change physics and must not be the single source of truth.

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

This is **not a mandatory final enum**. What matters is the architecture: phase is derived from kinematics + guidance/mission intent + vehicle capability and is used only as context. For a hybrid spaceplane, for example, the following are possible:

```text
FlightRegime::Aero + FlightPhase::Entry
FlightRegime::Aero + FlightPhase::Approach
FlightRegime::Coast + FlightPhase::Rendezvous
```

Some attributes are better kept orthogonal rather than hidden inside the enum at all:

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

`FlightPhase` may be an explicit guidance state, an automatic classifier, or a mix of both: intent determines what the vehicle is trying to do, kinematics verifies that it is actually in the corresponding regime.

## 4. Alert data model

Minimal runtime object:

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

Conditions and inhibitions are better not stored as closures in assets. The first implementation may have native Rust evaluators keyed by stable `AlertId`; later, data assets may define thresholds/text variants.

State of a single alert:

```text
Inactive
  -> Pending         condition true, debounce not elapsed
  -> Active          annunciation + optional aural
  -> Latched         source cleared, acknowledgement/reset still required
  -> Inactive        clear/rearm complete
```

For some alerts, `Latched` is not needed. For a critical warning, acknowledgement is not required to mute the repeating aural while the hazardous condition remains active.

## 5. Priority and audio arbitration

Dozens of messages may be active simultaneously, but the cockpit must not turn into an audio DDoS.

Recommended model:

```text
active conditions
      |
      v
severity / priority ordering
      |
      +-> visual annunciations: all relevant
      |
      `-> aural arbiter: single foreground stream + tones with explicit policy
```

First-version rules:

- a higher priority may preempt a lower voice line;
- the same alert does not restart on every tick;
- a parameter change inside an active condition does not count as a new alert;
- command warnings (`PULL UP`, collision/impact, catastrophic thermal/structural limit) preempt configuration advisories;
- a resolved warning is not played out if its meaning is already false;
- repeated voice uses a minimum repeat interval;
- aural silence/acknowledge is an avionics action, not a change of the physics condition.

FAA AC 25.1322-1 is used only as a reference for the general idea of distinguishable warning/caution/advisory levels and timely alerting that does not overload the crew.

## 6. Initial inventory

This does not mean that every item must exist on every craft.

| Alert | Base condition | Context / inhibition |
| --- | --- | --- |
| `STALL` | AoA / stall margin outside the envelope at sufficient `q` | `FlightRegime::Aero` only; hysteresis |
| `OVERSPEED` | Mach/IAS/equivalent-speed envelope exceeded | `Aero`; threshold defined by the vehicle envelope |
| `HIGH_AOA` | approaching the AoA limit | `Aero`; advisory/caution before stall |
| `HIGH_G` | load factor / structural load margin | capability/envelope-specific |
| `ACTUATOR_SATURATED` | allocator/runtime cannot realize the demand | suppress transient spikes; current `actuator_saturated` seed |
| `TERRAIN` | predictive terrain clearance insufficient | not on `Surface`; requires a terrain/radar source |
| `PULL_UP` | imminent terrain/impact trajectory | highest flight-path priority |
| `SINK_RATE` | excessive descent near terrain | approach/landing/low-altitude context |
| `DONT_SINK` | altitude loss after takeoff/go-around | takeoff/go-around context |
| `TOO_LOW_GEAR` | low + landing intent + gear not deployed | craft with retractable gear only |
| `TOO_LOW_FLAPS` | low + landing intent + high-lift config insufficient | craft with high-lift devices only |
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

`GLIDESLOPE` and similar navigation-specific warnings must appear only when the corresponding guidance/navigation source is present. They must not be derived from altitude + vertical speed alone.

## 7. Terrain / GPWS-like warnings

GPWS should not be implemented as a copy of a specific certified box. A functional set of checks over the project's own terrain model is needed.

Base inputs:

```text
terrain clearance
vertical speed
horizontal velocity
predicted path over short horizon
gear/high-lift configuration
flight phase / landing intent
```

The first deterministic predictor may take several future samples along a ballistic/local-velocity projection. Later it can be replaced with a trajectory-aware envelope without changing the alert ABI.

Important invariant: the terrain system warns about **risk**, not about nominal altitude. Low flight in landing configuration must not constantly shout `PULL UP`; fast-closing terrain must.

FAA TAWS guidance is used as a reference for command-style terrain alerting and for the need for early warning with minimized unwanted alerts, but Thessa thresholds are the project's own game/vehicle data.

## 8. Aural assets

The aural system has stable semantic IDs:

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

Voice and nonspeech assets are separated from code:

```text
assets/audio/alerts/<pack>/manifest.toml
assets/audio/alerts/<pack>/voice/*.ogg
assets/audio/alerts/<pack>/tones/*.ogg
```

Recommended asset policy:

- speech is generated via offline TTS or recorded specifically for the project;
- simple siren/chime/whoop assets are synthesized with the project's own generator tool;
- runtime does not depend on cloud TTS;
- the manifest stores provenance, generator version, and license of each file;
- code may keep the GPL/MIT split under the current scheme; the sound pack is licensed separately;
- third-party CC0 references may be used for analysis, but if a unified NC asset pack is needed, the final waveform is better generated independently.

Localization is a different voice pack with the same `AuralId`s. The authoritative alert ID does not depend on language.

## 9. Network boundary

The authoritative server computes condition state and transitions. The client receives:

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

An event does not need to be sent on every snapshot. What is needed:

- sparse events for immediate audio response;
- active set in a periodic/full snapshot for reconnect/resync;
- monotonic `instance/generation` so that a duplicate packet does not replay a warning.

Purely local UI alerts (`controller disconnected`, `audio device lost`, `network jitter`) live in a different namespace and never masquerade as flight warnings.

## 10. Scheduler and cost

Most conditions are cheap and can be checked on the fixed flight tick. Expensive predictive alerts do not need to run at 120 Hz.

Example budget:

```text
120 Hz: stall, overspeed, loads, actuator saturation
20-30 Hz: gear/configuration, propulsion/thermal summaries
5-10 Hz: short-horizon terrain prediction
1-5 Hz: long-horizon conjunction / mission advisories
```

The alert manager must receive already-computed telemetry values rather than re-invoking aerodynamics, the terrain solver, or ephemeris without reason.

For known-time events, the existing `EventScheduler` can be used; alerting must not create a JS polling loop.

## 11. Tests

Minimal suite:

1. condition crossing triggers exactly one `Activated`;
2. threshold jitter does not create spam thanks to hysteresis/debounce;
3. high-priority warning preempts low-priority voice;
4. an alert can activate again after clear/rearm;
5. `TOO_LOW_GEAR` is impossible without landing/low-altitude context;
6. `STALL` is impossible in exact vacuum regardless of orientation;
7. a capability-gated alert is absent on a craft without the corresponding system;
8. snapshot resync does not replay an already-active aural;
9. scalar/server replay yields an identical sequence of `AlertEvent`;
10. time-warp does not skip a critical transition: interval evaluation, either a certified guard or a boundary substep.

## 12. Implementation order

### Slice A — framework

- `AlertId`, severity, state machine, debounce/latch/rearm;
- alert manager inside the authoritative flight layer;
- protocol active-set + sparse transitions;
- client annunciator + audio arbiter;
- generated test tone/voice pack.

### Slice B — existing telemetry

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
