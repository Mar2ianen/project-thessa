# Docking ports

Status: design baseline; exact load ratings, capture geometry, and balance values are TBD.

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


## 10. Reference CAD assembly

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
