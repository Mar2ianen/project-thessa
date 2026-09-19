# 10 — Pilot interface contract

## Status

**Implemented client prototype.** The pilot HUD, navball, camera, command
mapping, telemetry frames, and server snapshot path are implemented in
`apps/client`. The visual contract is deliberately separate from authoritative
simulation state. Contact debug visualization lives in
`apps/client/src/contact_gizmos.rs`.

## 10.1. Modes and controls

| Input | Current action |
| --- | --- |
| `M` | map/pilot mode |
| `W/S` | pitch down/up |
| `A/D` | yaw left/right |
| `Q/E` | roll left/right |
| `Caps Lock` | precision input at 25% |
| `Shift/Ctrl` | throttle up/down |
| `Z` / `X` | full throttle/cutoff |
| `Space` | engine toggle |
| `T` | SAS toggle |
| hold `F` | temporary SAS inversion |
| `R` / `G` | RCS / gear toggle |
| `V` | free/follow camera |
| backquote | reset camera |
| `Escape` / `F8` / `Pause` | pause |
| `F1` | control and icon legend |
| `F2` | hide/show UI |
| `F3` | extra telemetry |
| RMB / MMB / wheel | orbit/pan/zoom |

The UI also exposes buttons for SAS, RCS, gear, engine, camera, map, and pause.
Green indicates an enabled state. Extra data is hidden by default.

## 10.2. Control path

```text
keyboard/HUD
    ↓
PilotAxes / validated command
    ↓
server or embedded authority
    ↓
GuidanceIntent
    ↓
flight control law and policy
    ↓
allocator and actuator dynamics
    ↓
authoritative vehicle state
```

The client never writes an authoritative transform. Manual axes, SAS/attitude
hold, rate guidance, direct mode, RCS, throttle, gear, and reset are commands
or typed intents. The authority returns snapshots and telemetry.

## 10.3. Speed frames

The HUD supports independent speed-reference selection:

- `SURFACE`: velocity relative to the selected body’s rotating surface;
- `AIR`: air-relative velocity;
- `ORBITAL`: velocity relative to the selected body’s inertial frame;
- `TARGET`: target-relative display when a target is available.

The navball speed label cycles these frames when clicked. Speed frame is a
display/control reference and does not rewrite the physics state.

## 10.4. Altitude frames

Altitude can be displayed as:

- datum altitude from the selected/reference body;
- AGL/terrain-relative altitude where an authoritative terrain sample exists.

Negative values are preserved when physically meaningful. Missing preview data
is not formatted as authoritative physics.

## 10.5. Navball and vector cues

The navball is a dynamic sphere projection, not a second attitude state. It
uses the selected reference frame and hides cues that are unavailable. Vector
markers include prograde/retrograde and relevant surface/orbital/target cues.
The rendered craft orientation and navball share the same physical axes.

## 10.6. Camera and map

The map and pilot scenes use anchored render-local coordinates. Camera follow,
selection, orbiting, panning, zoom, and frame-selected requests may move the
render origin without changing authoritative focus or craft state.

The camera can pass through both poles; no singularity-prone latitude clamp is
used. Far map views use readability scale; close views blend body meshes toward
physical size.

## 10.7. X-15 adapter

The current visual flight test uses the imported North American X-15 mesh with
the shared starter physical vehicle profile. The asset conversion maps its
GLB axes to physical craft axes (`+X` forward, `+Y` right, `+Z` up), then applies
the physics orientation to the render mesh.

Regression tests cover initial, pitch, roll, and combined attitudes including
the mesh’s child rotation. The visual asset is not a claim of reference-grade
X-15 aerodynamics; comparison against JSBSim is a proxy validation case.

## 10.8. Snapshot/interpolation boundary

The client receives framed snapshots. It may interpolate visual pose and terrain
presentation between arrivals. It must preserve authoritative metadata such as
simulation time, reference body, altitude mode, control status, and effective
warp. The server remains the owner of commands, time, and physics.

## 10.9. Instrument assets

Flight icon SVG sources and 4x PNG exports are in `assets/ui/flight/`. The F1
legend is the human-readable inventory of those controls. Asset provenance is
recorded in the adjacent README files.

## 10.10. Verification

```bash
cargo test -p thessa-client
cargo test -p thessa-server
```

The tests cover control mappings, frames, navball projection, orientation,
render anchoring, camera behavior, missing telemetry, and reset behavior.
