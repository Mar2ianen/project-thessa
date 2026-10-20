# Docking ports

Status: design baseline with a D1 contact-scene integration slice. The persisted
docking protocol, fixed-joint installation/removal, pressure-equalization gate,
and solver joint-load readback are implemented; physical capture dynamics,
class-dependent load ratings, and server-level ownership remain future work.

## 1. Design goals

Project Thessa uses a family of androgynous docking interfaces rather than male/female port pairs.

The central design rule is that **docking compatibility and structural capacity are separate properties**:

- all standard pressurized docking ports share the same hermetic inner interface;
- different port classes may dock with each other;
- larger classes provide a stiffer and stronger structural connection when compatible outer capture structure can engage;
- a separate compact utility attachment interface exists for machinery and station hardware and is intentionally incompatible with the pressurized docking family.

The system should make mixed fleets practical without making port size meaningless.

## 2. Standard pressurized port family

The initial family has three nominal classes:

| Class | Nominal size | Pressurized core | Intended role |
| --- | ---: | ---: | --- |
| D1 | 1 m | common 1 m core | spacecraft, small modules, general-purpose docking |
| D2 | 2 m | common 1 m core | larger modules, cargo vehicles, stronger station connections |
| D3 | 3 m or larger | common 1 m core | very large modules, major station structures, high-load connections |

`1 m`, `2 m`, and `3 m` are nominal interface classes, not yet final external envelopes. The exact clear passage diameter, seal diameter, latch circle, keep-out volume, and external dimensions remain to be specified.

### 2.1 Common hermetic core

Every D1/D2/D3 port contains the same approximately 1 m pressurized mating core.

This common core provides the compatibility floor:

- D1 ↔ D1: compatible;
- D1 ↔ D2: compatible;
- D1 ↔ D3: compatible;
- D2 ↔ D2: compatible;
- D2 ↔ D3: compatible;
- D3 ↔ D3: compatible.

The common core is responsible for the pressure seal and the minimum hard-dock connection. A successful mixed-class dock therefore does not require a separate adapter merely to create a pressure-tight passage.

## 3. Structural compatibility

Larger docking classes add structural capture area outside the common pressure core. Their purpose is not to create a different airtight interface, but to carry greater axial, shear, bending, and torsional loads with lower compliance.

The effective joint strength is determined by the structure that both mating ports can actually engage. Consequently:

- a large port does not magically make a D1 partner a high-load connection;
- equal or closely matched larger classes can exploit more of their outer structural hardware;
- mixed-class docking remains physically valid but may have lower stiffness and lower allowable loads than a same-class connection.

The exact rule for partial outer-ring engagement is intentionally left open until the port geometry is modeled. The implementation must not encode port class as a cosmetic size tag; it must produce a real difference in joint stiffness/load limits.

A useful high-level invariant is:

> Every standard port can create the common hermetic dock; additional shared structure raises the mechanical rating of the joint.

## 4. Androgynous operation

Standard docking ports are mechanically androgynous. There is no permanent male/female pairing in the vehicle definition.

A docking attempt may still assign temporary control roles such as `active` and `passive` for guidance, capture sequencing, or actuator ownership, but these are runtime roles rather than incompatible hardware types.

This avoids adapter proliferation and lets stations, spacecraft, tugs, and modules reuse the same interface family.

## 5. Docking sequence

The gameplay/physics model should distinguish at least the following states:

1. **Free** — no physical capture.
2. **Soft capture** — compliant capture hardware has caught the other port and is damping residual relative motion.
3. **Aligned** — port axes and clocking are within hard-capture tolerance.
4. **Hard dock** — the common structural/pressure interface is locked.
5. **Outer structure engaged** — class-dependent structural hardware has latched where geometry permits.
6. **Pressure equalized** — hatches may be opened if both connected volumes are pressurizable and operational.

These stages should remain distinct in the simulation. A joint may be mechanically captured without yet being pressure-tight, and a pressure-tight mixed-class connection may still have a lower structural rating than a full same-class dock.

## 6. Simulation boundaries

Docking touches several independent graphs and they should not be collapsed into one boolean `docked` flag.

### 6.1 Mechanical graph

The collision/rigid-body backend owns the physical joint while the vehicles remain separate dynamic bodies. It provides relative pose constraints and physical load evidence.

Port design data should eventually define at least:

- capture envelope;
- alignment tolerance;
- permitted relative linear/angular velocity at capture;
- translational stiffness/damping;
- rotational stiffness/damping;
- hard-dock axial/shear force limits;
- bending/torsional moment limits;
- break/failure behavior.

Exact values are TBD.

### 6.2 Pressure graph

A successful hard dock through the standard family may connect two pressurized volumes through the common hermetic core. Pressure equalization and hatch state are separate from the existence of the mechanical joint.

### 6.3 Resource/data graph

Ports may also expose standardized power, data, and fluid connections. These are logically separate from pressure and structure so damaged or intentionally isolated services do not require breaking the physical dock.

## 7. Compact utility attachment interface

Thessa also has a separate compact attachment standard for technical hardware.

This interface is deliberately **not** part of D1/D2/D3 and is not cross-compatible with standard pressurized docking ports.

Its physical concept is a very small attachment point rather than a human-passable docking collar.

Primary uses include:

- station maneuvering-engine/RCS modules;
- robotic manipulators and their end effectors;
- removable sensors and instruments;
- maintenance hardware;
- external service modules;
- construction fixtures;
- other small unpressurized station/vehicle equipment.

The utility interface has no pressurized passage. It may carry mechanical load plus optional power, data, and propellant/fluid services depending on the attached hardware.

It should be substantially more compact than D1, making it practical to place many attachment points around a vehicle or station without implying crew-transfer capability.

## 8. Gameplay consequences

The port family should create useful engineering choices rather than arbitrary compatibility restrictions:

- D1 remains sufficient for crew transfer and ordinary docking;
- larger ports are worth their mass, volume, and placement cost because they produce much stiffer high-load connections;
- emergency or improvised mixed-class docking remains possible through the universal core;
- large station trusses and massive modules benefit from D2/D3 rather than merely using many D1 ports;
- utility hardware does not consume full docking-port real estate;
- manipulators and service equipment can use dense networks of compact attachment points.

The player should be able to infer most of this from geometry: the common inner core communicates compatibility, while the larger outer structure communicates load capacity.

## 9. Open design questions

The following values and mechanics should be decided later with the structural and vehicle-construction systems:

- whether D3 remains exactly 3 m or becomes a broader heavy-port class above 3 m;
- exact clear passage diameter of the common core;
- exact outer-ring geometry for D2/D3 mixed docking;
- whether mixed D2 ↔ D3 engages only the common core or an intermediate structural ring;
- soft-capture mechanism and allowable capture velocities;
- structural load/stiffness values for each engagement configuration;
- standard power/data/fluid pinout and transfer limits;
- whether utility attachments have one universal service interface or typed variants;
- failure modes: latch loss, seal loss, partial outer-ring failure, complete joint failure.

Until those values exist, code should depend on explicit port/interface capabilities rather than assuming that nominal diameter alone determines every behavior.

## 10. Runtime integration slice

The first executable D1 slice is implemented at the CAD/runtime boundary:

- `thessa-sim-core::DockingSession` owns the persisted protocol state and
  validates compatible D1 port frames, capture speed, alignment tolerances and
  pressure equalization progression;
- `thessa-collision::CollisionWorld::attach_fixed_joint` is the Rapier hard
  docking constraint, with contacts between the joined craft disabled so the
  explicit structural constraint owns the interface;
- `thessa-flight-authority::ContactRuntime` syncs a partner into the same
  contact scene, checks live port-frame kinematics for soft capture/alignment,
  installs the fixed joint after alignment, advances pressure equalization in
  simulation time, and removes the joint on separation while retaining both
  solved states;
- `CollisionWorld::joint_loads(dt_s)` exposes the latest solver-substep reaction
  force and moment magnitudes, not a full-frame impulse sum or peak. Rapier
  0.35 writes the final small-step impulse, so its duration is
  `dt_s / num_solver_iterations`; the wrapper currently configures no per-body
  extra iterations or multiple CCD time subdivisions. Dividing by the full
  frame incorrectly under-reported these loads by four at default settings.
  Angular locked-DOF impulses are additionally mapped through Rapier's public
  quaternion-error Jacobian before being interpreted as a moment; at aligned
  frames that Jacobian is half a rotation matrix, not unit angular axes.
  Readback covers locked-DOF reactions, not motor/limit impulse aggregation or
  a high-speed/large-misalignment peak-load estimate.
  A two-equal-body regression pins F/2 and torque/2 across 1/2/4/8 small steps
  at 60/120 Hz within 1e-6 N/Nm. This is solver telemetry for a future strength
  policy, not an automatic failure threshold;
- `CollisionWorld::attach_revolute_joint` is the mechanism seam for the six
  provisional soft-capture petals in the D1 asset;
- the current `SoftCapture` transition is explicitly a logical MVP gate; the
  petals/revolute bridge is available for mechanism integration, but latch
  damping and capture stiffness are not yet calibrated physical behavior;
- solver gravity remains zero and craft mass/inertia remain authoritative
  `RigidBodyProperties`, consistent with the Rapier integration contract;
- the collision regression fixture creates two D1 craft, completes the full
  docking state sequence, transfers an applied load through the dock, and
  verifies a clean undock.
- the flight-authority contact regression runs the persisted session through
  capture, align, hard dock, pressure equalization, applied load readback, and
  separation; internal joints between parts of one baked vehicle still do not
  have separate Rapier bodies or structural ratings.

The fixture is an integration proof, not a final structural calibration. Exact
petal stiffness, seal loads, latch ratings and failure thresholds remain open
design values until CAD feature recognition and contact calibration are done.


## 11. Reference CAD assembly

A D1 v2.2 FreeCAD/STEP assembly now exists as the first concrete geometry
reference for this family. The supplied STEP fixture is approximately
1252 x 1223 x 332 mm and contains 139 solids, 1744 faces and 4229 unique edges.

The model already represents the soft-capture mechanism geometrically rather
than as a decorative ring: guide petals, damper barrels/rods, fixed and moving
clevises, rails/rollers, hard-latch hooks, pressure sealing hardware and service
hardware are separate modeled components.

The assembly is intentionally a reference fixture, not yet a locked production
D1 envelope. Exact dimensions and load ratings in this document remain TBD
until the mechanical model is validated.

For rendering/asset architecture, this fixture is the first golden candidate for
the CAD -> normalized BRep -> adaptive RCBT -> transient mesh pipeline described
in 'docs/45_CAD_RCBT_GEOMETRY.md'. Docking physics must consume explicit
mechanical semantics from the design; it must not depend on the current
view-dependent render tessellation.

## 12. Native retained-fixture port (2026-10-20)

The old `apps/client/src/docking_demo.rs` now has a direct GPL native consumer
in `apps/terrain-native/src/docking.rs`. It calls the same `CollisionWorld` and
`DockingSession`, not a new solver, controller, or animation-driven pose path.

```sh
cargo run --release -p thessa-terrain-native -- --docking-demo --mute
cargo run --release -p thessa-terrain-native -- --docking-demo --headless target/docking.png --docking-time 5.5 --docking-step 0 --hud --mute
```

The retained fixture is an **isolated zero-gravity vacuum experiment**, not a
spacecraft orbit or live fleet authority: two 300 kg bodies with diagonal
400 kg m² inertia, starting at x=±1.5 m and vx=∓0.02 m/s; port stations are
x=±1.4 m, with the same central body/eight ring-cuboid collision proxies.
`SoftCapture` starts as a logical gate. The measured port-frame gap must be
within 0.025 m and `DockingSession::align` must accept actual solved kinematics
before the fixed joint is installed. Outer engagement and the 0.5 s pressure
gate use the existing protocol. Once equalized, a real 350 N wrench on A acts
for 240 ticks (2 s), then the joint is removed and the final solved poses freeze,
matching the retained fixture's completion behavior. There is no undock kick.

The native driver admits at most 0.05 s per presentation and accumulates exact
1/120 s solver steps; arbitrary frame cadence never changes solver dt. Headless
`--docking-time 0..30` seeks before warmup, while `--docking-step 0..0.05`
(default 1/60) controls admission including the 30 warmup frames; zero freezes
the requested checkpoint. Seek quantizes downward to a tick and clears old
audio events. Flight/server/map/site/preview/time-control combinations fail
before loading the world; the window map switch is inert in this fixture.

The existing opaque mesh path renders the two actual solved rigid instances
with per-primitive instance ranges, retained colored body/port proxies and D1
CAD surfaces. `decode_glb_rest_pose` explicitly flattens the authored node
transforms without sampling animation clips. Ordinary `decode_glb` still
rejects clips, and both reject skins/extensions/external resources and enforce
the same budgets. This preserves the old unanimated CAD inspection, not a
working petal-actuator model. The source exporter maps STEP +Z into glTF +Y;
native maps that actual mating normal to ±X, fixing the old adapter's +Z
assumption. A geometry regression pins the thin axis to the mating corridor.

The display anchor is one million metres above the current survey bookmark,
solely to reuse native camera, celestial lighting, terrain and mesh consumers;
it is never fed into the isolated solver as a physical trajectory. No local
`FlightAuthority`, JIT forecast, flight engine plume, or remote replica is made.
HUD/telemetry report state, gap, load time, actual velocities, joint count,
latest-substep joint force and solved proxy inspection. Camera/UI reads cannot
alter solved state. In cabin mode only, the once-consumed capture event emits
the retained authored 110 Hz / 180 ms structural cue; exterior vacuum, mute,
headless tests and seeking remain silent. This is not calibrated impact audio.

Tests pin cadence-independent replay, single capture, full transition order,
load transfer, ≤0.025 m port error, total momentum within 1e-5 kg m/s of the
700 N s applied impulse, and the 175 N partner reaction within 0.001 N during
steady loading. CLI captures cover approach, engagement, load and clean undock
through the real native host; these are deterministic physical-state evidence,
not independent aesthetic approval. The one-fixture CPU diagnostic runs nine
1000-requested-tick trials, freezing completed fixtures; it is not a pure
per-tick solver or whole-client benchmark. Executed gates and exact evidence:
`../artifacts/native-docking-2026-10-20.json`.

General live-flight contact patch/normal scene overlays, interactive docking
commands, calibrated petal capture, server ownership UI and jointed-stack
control remain separate obligations. The native solved-fixture inspector does
not claim to replace every contact/debug view in the legacy host.
