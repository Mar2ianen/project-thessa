# Audio architecture and acoustic propagation

Status: first semantic slice implemented; backend calibration and full source integration remain.

This document defines the semantic boundary for sound before the current Bevy
client host is replaced. Audio presentation must not become coupled to Bevy,
the render backend, or a particular mixer library.

The central rule is:

> A physical event produces acoustic and vibration sources. What a listener
> hears is derived from the available propagation paths and the listener
> context. The simulation does not emit a pre-mixed `play("foo.wav")`
> command as authoritative state.

This keeps physical causality intact without forcing every presentation mode to
be literally silent whenever no real acoustic path exists.

## 1. Goals

The audio architecture must support:

- engines, pumps, valves, RCS, mechanisms, impacts, docking and structural
  vibration;
- atmospheric propagation, including delayed sonic-boom arrival;
- cabin and suit audio;
- structure-borne sound in vacuum;
- GPWS, alarms, avionics and other explicitly generated speaker audio;
- radio and communications paths;
- exterior cameras in atmosphere and vacuum;
- multiplayer without server-side audio mixing;
- replacement of the current Bevy client host without changing gameplay or
  simulation APIs;
- optional cinematic presentation that is clearly separate from the physical
  acoustic model.

The first implementation should stay small. It needs a correct semantic seam,
not a full room-acoustics solver.

## 2. Non-goals

The initial system does not require:

- full wave-equation acoustics;
- CFD-derived broadband noise;
- per-triangle acoustic simulation;
- server-side HRTF, reverberation or final mixing;
- graphics ray tracing as a mandatory dependency;
- a sound-emitter component attached to every visual entity;
- authoritative storage of sample names or backend voice handles.

## 3. Layering

The intended dependency direction is:

```text
authoritative simulation / gameplay
        |
        | physical events and source state
        v
backend-neutral audio semantics
        |
        | listener-specific propagation resolution
        v
client audio presentation
        |
        | resolved voices / buses / filters
        v
audio backend
```

The current backend may be provided by the Bevy client. A later native client
may use another library. Neither choice is part of the semantic API.

The first reusable slice now follows this layering:

```text
crates/audio-core          source + listener + propagation semantics
apps/client/audio          client policy, buses, assets, mixing decisions
current backend            Bevy audio or another mixer
```

`crates/audio-core` is now the backend-neutral layer. The current Bevy adapter
lives in `apps/client/src/audio.rs` and is explicitly disposable.

## 4. Sound is not the event

Gameplay code should describe what happened, not which sample to play.

Examples of semantic sources include:

```rust
enum SoundEvent {
    ContactImpact { /* impulse / material evidence */ },
    LatchEngaged { /* mechanism identity */ },
    ValveTransition { /* valve, pressure delta */ },
    AlarmTriggered { /* alert identity */ },
    SonicBoomArrival { /* derived atmospheric event */ },
}
```

Long-lived sources are better represented as persistent source state than as a
high-rate stream of events:

```rust
struct AcousticSourceState {
    source_id: SourceId,
    body_id: BodyId,
    local_position_m: DVec3,
    acoustic_power_w: f64,
    vibration_power_w: f64,
    spectrum: SpectrumDescriptor,
    directivity: DirectivityDescriptor,
}
```

The concrete API may differ, but the distinction is important:

- one-shot events describe discontinuities;
- persistent sources describe continuing machinery or flow;
- the renderer/mixer chooses samples, synthesis layers, loop points and
  crossfades.

An engine therefore does not own an audio clip. It exposes physical state that
can drive several presentation layers.

## 5. Propagation paths

A listener may receive the same source through more than one physical path.

### 5.1 Atmospheric path

Sound may propagate through a gas when a continuous atmospheric path exists
between source and listener.

The model may depend on:

- local density and pressure;
- temperature and therefore sound speed;
- source/listener relative velocity;
- distance;
- occlusion;
- atmospheric absorption;
- source directivity.

Vacuum means no atmospheric acoustic path. It does not imply that the listener
must hear nothing through every other path.

### 5.2 Cabin-air path

A listener inside a pressurized volume may hear sources coupled into that
volume through:

- machinery physically located inside it;
- loudspeakers;
- structure-to-air transmission;
- leaks or open pressure connections.

Cabin audio is not derived from the exterior camera mix.

### 5.3 Structure-borne path

Mechanical energy can reach a listener through connected structure even in
vacuum.

This path is important for:

- engines mounted to the same vehicle;
- RCS firings;
- docking impacts;
- latch motion;
- pumps and rotating machinery;
- wheel or landing-gear loads;
- actuator motion;
- structural failure.

The first implementation does not need a finite-element acoustic solve.
A reduced structural propagation model may operate on the structural graph if
its parameters retain physical meaning and are calibrated against the more
detailed mechanical model.

A source should therefore be able to expose both airborne acoustic power and
structure-borne vibration independently.

### 5.4 Suit path

An EVA listener may hear:

- suit pumps and fans through suit structure and suit atmosphere;
- impacts or hand/boot contact coupled through the suit;
- radio;
- their own equipment.

An unrelated external source across vacuum has no airborne path to the suit.

### 5.5 Radio / electrical path

Radio, intercom, GPWS, warning tones and synthesized avionics audio are not
environmental sound propagation.

They are explicit signal paths into a speaker/headset endpoint and continue to
work in vacuum when their electrical/data path is available.

## 6. Listener context

The listener is more than a world-space point.

A client listener context may include:

```rust
struct ListenerContext {
    pose: ListenerPose,
    pressure_volume: Option<PressureVolumeId>,
    structural_cluster: Option<StructuralClusterId>,
    suit: Option<SuitId>,
    radio_endpoints: SmallVec<[RadioEndpointId; 4]>,
    presentation_mode: AudioPresentationMode,
}
```

This makes camera and crew modes explicit.

Typical cases:

| Listener | Atmospheric | Cabin | Structure | Radio |
| --- | --- | --- | --- | --- |
| exterior camera in air | yes | no | no | optional |
| exterior camera in vacuum | no | no | no | optional |
| seated crew in cabin | maybe via hull | yes | yes | yes |
| free crew inside module | maybe via hull | yes | yes | yes |
| EVA crew in vacuum | no | suit only | contact-dependent | yes |

The exact listener model must not be encoded as a Bevy camera component.

## 7. Engines

Engine sound should be layered from the same physical state that drives the
engine model.

Useful source channels include:

- exhaust/plume airborne noise;
- chamber and injector broadband noise;
- turbopump or electric-pump machinery;
- feed-system and valve transients;
- structure-borne vibration;
- startup, shutdown and mode-transition events.

The audible result depends on the propagation path.

Examples:

- an exterior atmospheric listener may hear plume and machinery noise;
- an exterior vacuum camera receives no airborne exhaust sound;
- crew inside the same vehicle may hear strong structure-borne engine energy
  plus cabin-transmitted machinery noise;
- a remote docked module may receive attenuated structure-borne vibration
  through the docking/structural graph.

Combined-cycle engines may change their acoustic source model with their
physical engine state, such as air-breathing, transition and closed-cycle
operation. Audio follows engine state; audio must not invent a separate
authoritative engine mode.

## 8. RCS and mechanisms

RCS is a useful sanity check for the propagation model.

An RCS pulse may be:

- loud outside in atmosphere;
- silent to a detached exterior camera in vacuum;
- audible as a short structure-borne impulse to crew in the vehicle;
- different again for a suit or vehicle in direct mechanical contact.

The same principle applies to docking petals, latches, landing gear, valves,
pumps and other mechanisms.

## 9. Docking and impacts

Docking must not collapse to one canned "dock" sound.

Physical evidence may include:

- first contact impulse;
- repeated soft-capture contacts;
- damper motion;
- latch engagement;
- hard-dock load transfer;
- seal or mechanism motion;
- structural ringing.

Two vehicles touching in vacuum can transmit vibration through the new
mechanical connection after contact. An external camera floating nearby still
does not receive an atmospheric sound path.

Contact material and impulse information should come from the existing
collision/contact boundary rather than be guessed from the visual asset.

## 10. GPWS, alarms and avionics

GPWS, warning tones, callouts and avionics are generated audio endpoints, not
environmental sound emitters.

They should be routed through a cockpit/suit/intercom bus and gated by the
relevant electrical, avionics and data state when those systems become
physical.

Their existence therefore does not depend on atmosphere.

Presentation code may spatialize a cockpit speaker if desired, but the
authoritative fact is that a warning endpoint emitted a signal.

## 11. Sonic boom

Sonic boom must not be implemented as a sample that plays when a vehicle's
Mach number crosses 1 or when the vehicle passes the camera.

The listener hears the disturbance when the pressure wave reaches the listener.

The implementation should therefore separate:

```text
supersonic source trajectory
        |
        v
wave / Mach-cone propagation
        |
        v
listener intersection at a later time
        |
        v
boom event for presentation
```

The first model may be geometric and atmosphere-local rather than a full
nonlinear acoustic solver, but it should preserve:

- finite propagation speed;
- delayed arrival;
- dependence on the source trajectory;
- no boom in vacuum;
- listener-local pressure-wave arrival rather than source-local triggering.

The existing ray/BVH infrastructure may accelerate geometric queries, but the
canonical semantic model must not require GPU or graphics ray tracing.

## 12. Occlusion and geometric queries

Acoustic occlusion may use backend-neutral geometric ray/BVH queries.

This follows the project ray-query invariant:

- the authoritative or reference path must work on CPU;
- graphics RT hardware may accelerate local/client queries;
- Vulkan RT, DXR, Metal RT or FastRT must not leak into the domain API;
- lower quality modes may reduce query cadence or geometric detail without
  changing whether an actual propagation medium exists.

For large interiors, portal/volume connectivity may be more useful than
shooting many rays through every frame.

## 13. Delay and Doppler

Propagation delay is part of the physical model where it is perceptually or
mechanically relevant.

At minimum:

- sonic boom requires delayed arrival;
- distant atmospheric events should be capable of delayed playback;
- radio delay may be represented separately from acoustic delay when long
  distances matter;
- structure-borne delay may initially be reduced if the resulting error is
  bounded and inaudible at vehicle scale.

Doppler should be derived from source/listener motion and propagation medium,
not from a gameplay coefficient.

The mixer may use a reduced presentation approximation, but source and listener
kinematics remain the semantic inputs.

## 14. Physical and cinematic presentation

The physics model should report that an exterior vacuum listener has no
airborne acoustic path.

The presentation layer may nevertheless offer an explicit cinematic mode for
players who prefer exterior sound in space.

This is allowed because it is a presentation policy, not a change to
authoritative physics.

Recommended policy states:

```rust
enum AudioPresentationMode {
    Physical,
    Cinematic,
}
```

`Physical` follows available propagation paths.

`Cinematic` may add non-authoritative exterior cues, but those cues:

- never affect AI, sensors, damage or gameplay state;
- are not replicated as physical evidence;
- are visibly/configurably presentation behavior;
- must not replace the physical mix used for cabin, suit or diagnostics.

This avoids sacrificing usability merely to preserve an aesthetic rule while
keeping the simulation model honest.

## 15. Multiplayer boundary

The server is authoritative for physical source state and gameplay-relevant
events.

The server should not mix audio for every listener.

A normal flow is:

```text
server:
    engine/contact/mechanism state
        |
        v
replicated semantic event/state

client:
    listener context
    + local geometry / pressure volumes / structural connectivity
        |
        v
propagation resolution
        |
        v
mixing / samples / HRTF / reverberation
```

Client audio may be predicted for locally controlled low-latency actions when
the corresponding gameplay action is already predicted.

A rejected predicted action must be reconciled just like other presentation
state.

## 16. Backend boundary and Bevy migration

No gameplay system should retain a Bevy audio handle as authoritative state.

The client-facing backend should consume a compact resolved command set such
as:

```rust
trait AudioBackend {
    fn start_voice(&mut self, voice: ResolvedVoice) -> VoiceHandle;
    fn update_voice(&mut self, handle: VoiceHandle, voice: ResolvedVoice);
    fn stop_voice(&mut self, handle: VoiceHandle, fade_s: f32);
    fn submit_one_shot(&mut self, voice: ResolvedVoice);
}
```

The exact trait is not fixed here. The invariant is that backend handles stop
at the client presentation boundary.

This permits:

```text
today:   audio semantics -> Bevy-backed mixer
later:   audio semantics -> native mixer
```

without changing engine, docking, GPWS or atmospheric code.

## 17. State machines

Many audio sources are naturally derived from existing simulation state
machines.

Examples:

- engine startup -> running -> shutdown;
- air-breathing -> transition -> closed-cycle;
- docking free -> soft capture -> hard dock;
- landing gear extension -> contact -> compression;
- valve closed -> opening -> open -> closing;
- alarm inactive -> active -> acknowledged.

Audio should subscribe to those physical transitions instead of maintaining a
second competing state machine when the simulation already owns the truth.

A presentation-only state machine may still manage sample envelopes,
crossfades and voice lifetime.

## 17.1 Procedural-first presentation

Mechanical and flow-driven sounds should be procedural by default when their
physical state already exposes the parameters needed to synthesize them.

Good procedural candidates include:

- engines and turbomachinery;
- plume / jet broadband noise;
- RCS pulses;
- pumps, valves and actuators;
- docking/contact transients and structural ringing;
- landing gear and wheel/ground interaction;
- warning tones and other non-speech avionics cues.

Recorded assets are primarily for content whose information is linguistic or
otherwise authored rather than generated by the physical state. Spoken cockpit
warnings/callouts are the clearest example.

The semantic layer still exposes physical state and events rather than a
specific synthesis algorithm. The current Bevy `Pitch` tones are only a
diagnostic presentation fixture; they are not the target sound model.

## 18. Performance policy

The system should be event-driven where possible.

Prefer:

- persistent voices for continuous machinery;
- parameter updates only when source state changes materially;
- bounded-rate propagation/occlusion queries;
- listener-centric culling;
- source importance and audibility budgets;
- cached pressure-volume and structural connectivity;
- lower update cadence for distant or masked sources.

Avoid:

- one ray cast per source per audio sample;
- rebuilding a full acoustic graph every frame;
- network replication of final gain/pan/filter values;
- per-tick one-shot events for a continuous engine.

Audio quality settings may change:

- HRTF quality;
- convolution/reverb quality;
- propagation query cadence;
- occlusion detail;
- number of simultaneous voices;
- structural-path approximation detail.

They must not change authoritative physics.

## 19. First slice

The first implementation proves the boundary with a small set of sources and
keeps uncalibrated presentation tones out of the semantic crate:

1. continuous engine presentation is gated by resolved airborne versus
   structure-borne path availability;
2. RCS produces a short cabin structure-borne cue on a real control edge;
3. the live D1 docking fixture publishes a capture-impact cue;
4. the existing low-altitude/descending warning feeds a cockpit-only GPWS
   prototype tone;
5. exterior atmospheric and exact-vacuum listeners use the same
   `resolve_airborne_path` primitive;
6. a debug cabin listener (`--audio-cabin`) exercises structural
   propagation before IVA exists;
7. `sonic_boom_arrival` solves the Mach-cone tangency and delayed arrival for
   a straight supersonic segment;
8. `--audio-demo` audibly exercises the delayed-boom path without committing
   placeholder sound assets.

Acceptance cases should include:

- exterior vacuum camera hears no physical engine exhaust;
- cabin listener still hears same-vehicle engine vibration in vacuum;
- GPWS remains audible in vacuum when its cockpit audio path is powered;
- docking contact is audible through connected structure but not through nearby
  vacuum;
- sonic boom arrives after the visible vehicle has passed when geometry
  requires it;
- switching audio backend does not change simulation state.

## 20. Open questions

The following are intentionally not fixed yet:

- the concrete audio backend after the Bevy client host is removed;
- calibration and spectral models for procedural engine, RCS, mechanism and impact synthesis;
- HRTF library and spatializer;
- pressure-volume / room representation for large stations;
- structural attenuation model and calibration source;
- how much interior reverberation is geometry-derived;
- radio simulation fidelity and latency model;
- whether cinematic exterior-space audio is enabled by default;
- exact quality tiers and voice budgets.

Those decisions should be made behind the semantic boundary above rather than
by leaking backend-specific audio types into simulation code.
