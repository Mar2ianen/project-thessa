# Procedural fuselages and body modules

Status: design baseline. Exact editor UX, structural shell model, internal packing rules, cutout implementation, and balancing values are TBD.

## 1. Design goal

Project Thessa should treat fuselages and body modules as procedural solids with gameplay semantics, not as a catalogue of fixed meshes.

The core rule is:

> Procedural body geometry defines shape. Module semantics define purpose. Presets define convenience.

A tank, service module, passenger cabin, avionics bay, cargo module, or pressure vessel may therefore share the same underlying geometric representation while differing in structural, resource, interior, and system behavior.

The editor should sit between KSP-style fixed Lego parts and a full CAD system: free enough to create useful custom spacecraft geometry, but deliberately constrained enough to remain predictable, fast, and understandable.

## 2. Limited procedural solid modeler

The initial body editor should expose two primary construction operations rather than general-purpose CAD.

### 2.1 Revolve

A two-dimensional spline profile is revolved around the local longitudinal axis.

This naturally represents:

- cylindrical and spherical tanks;
- cones and nose sections;
- ogive-like fairings;
- pressure vessels;
- engine bells or other axisymmetric service geometry where appropriate.

The profile is authoring geometry only. It is compiled into runtime representations before simulation.

### 2.2 Loft / longitudinal extrusion

A body is described by a sequence of cross-section stations along the local longitudinal axis.

Conceptually:

```text
x = 0.0 m   section A
x = 2.0 m   section B
x = 7.0 m   section C
x = 8.5 m   section D
```

Each station owns a two-dimensional spline section. The solid interpolates between stations.

This is intentionally more capable than a constant-section linear extrusion while remaining much simpler than unrestricted CAD.

Per-station data may include:

- section spline shape;
- scale;
- lateral/vertical offset;
- rotation about the longitudinal axis;
- optional local wall/structural parameters.

This lets the same primitive produce cylinders, tapers, circular-to-oval transitions, rounded rectangular sections, asymmetrical bodies, mild curvature, twist, lifting-body-like forms, and other useful spacecraft or aircraft fuselages.

The exact mathematical interpolation between stations is TBD, but it must remain deterministic and reject self-intersecting/invalid solids cleanly.

## 3. Editor modes

Simple users should not need to manipulate every spline point.

The editor may expose convenient high-level parameters such as:

- length;
- diameter or width/height;
- nose taper;
- tail taper;
- ovalness;
- section count;
- station offsets.

These controls are only views over the same underlying spline/station representation.

An advanced mode may expose the actual section splines and station transforms directly.

The goal is one representation with two editing depths, not separate "simple" and "advanced" part types.

## 4. Geometry and semantics are separate layers

A procedural body should not encode its gameplay purpose in its mesh-generation primitive.

The conceptual layers are:

```text
Body geometry
    -> structural model
    -> usable/internal volume
    -> module semantics
    -> ports and service interfaces
```

### 4.1 Geometry

Defines external and internal shape through revolve or loft/station data.

### 4.2 Structure

Defines shell/spar/frame behavior, structural mass contribution, stiffness, and eventual failure limits.

The first implementation may use simplified structural parameters while preserving the separation from module semantics.

### 4.3 Interior volume

The compiler derives the usable enclosed volume from the procedural solid, pressure-shell rules, wall thickness, and excluded regions.

This volume is a gameplay resource in its own right rather than a cosmetic number.

### 4.4 Module semantics

A module describes what a portion of the available volume does.

Possible roles include:

- propellant tank;
- oxidizer/fuel tank pair or other resource storage;
- passenger/crew cabin;
- cargo bay;
- avionics bay;
- battery/electrical bay;
- life-support equipment;
- generic service module;
- engine/machinery bay;
- unpressurized equipment volume;
- pressure vessel.

The same geometric body may contain several semantic regions.

### 4.5 Ports and service interfaces

Docking ports, utility attachment points, fluid connections, power/data interfaces, engine mounts, and similar external interfaces attach to the body independently of the geometry primitive used to create it.

## 5. Internal longitudinal allocation

A procedural body may divide its usable interior into longitudinal or otherwise derived regions.

For example:

```text
0-2 m    avionics
2-6 m    propellant
6-9 m    passenger cabin
9-10 m   service equipment
```

The exact editor representation is TBD, but internal allocations should be derived from real available geometry rather than from a fixed nominal part capacity.

Consequences of shape should therefore matter naturally:

- a narrow nose has less useful cabin or tank volume;
- a wider center section can carry more propellant or cargo;
- structural walls and cutouts reduce usable volume;
- different shapes produce different mass and inertia distributions.

## 6. Presets are convenience, not separate physics classes

The base game should ship common ready-to-use body/module presets so that procedural freedom does not become an editor tax.

Initial examples may include:

- round tank;
- elliptical tank;
- conformal/non-circular tank;
- passenger/crew module;
- service module;
- cargo module;
- avionics bay;
- generic pressure vessel;
- unpressurized structural/service section.

These presets should instantiate the same procedural geometry + semantic systems available to advanced users.

A stock round tank is therefore not a separate simulation primitive. It is a convenient preconfigured procedural body with tank semantics.

## 7. Hangar compilation

As with procedural aerodynamic surfaces, authoring geometry should not remain in the runtime hot path.

Leaving the hangar, loading a vehicle for simulation, or otherwise finalizing construction should compile the body into solver/runtime data.

Conceptually:

```text
ProceduralBody
    |
    +-- validate splines/stations
    +-- build watertight solid
    +-- derive external and internal volumes
    +-- apply structural shell / exclusions
    +-- resolve module allocations
    |
    v
Compiled body data
    +-- render mesh
    +-- collision geometry / decomposition
    +-- aerodynamic/body representation
    +-- structural representation
    +-- mass/inertia contribution
    +-- usable interior volumes
    +-- resource capacities
    +-- attachment/port surfaces
```

The runtime should consume compiled numerical and structural data, not editor spline objects.

## 8. Derived physical properties

Where practical, body properties should be derived from geometry rather than independently entered as arbitrary values.

Potential compiler outputs include:

- enclosed volume;
- wetted/external area;
- frontal/reference area;
- center of volume;
- shell/structure mass estimate;
- center of mass contribution;
- inertia tensor contribution;
- pressure-hull volume;
- aerodynamic body zones;
- collision primitives or convex decomposition;
- mount/attachment surface data.

Exact fidelity may vary by subsystem, but all consumers should derive from one canonical authoring shape.

## 9. Future subtractive operations

The first implementation does not require boolean modeling, but the representation should leave room for a deliberately small subtractive layer later.

Useful cutout operations include:

- windows;
- hatches and doors;
- docking-port recesses;
- landing-gear or equipment bays;
- air intakes;
- engine/service openings;
- cargo-bay openings.

A future cutout may be represented approximately as:

```text
Cutout {
    shape: Circle | RoundedRect | Spline2D,
    projection: Radial | NormalToSurface,
    depth: Through | Partial,
}
```

This is intentionally not a promise of unrestricted constructive solid geometry.

The editor should avoid becoming a general CAD feature tree with arbitrary booleans, constraints, fillets, and hundreds of dependent operations.

## 10. Scope boundary

The body editor should remain constrained around longitudinal solids and a small number of predictable modifiers.

The first implementation should not attempt:

- unrestricted free-form 3D sculpting;
- general CAD sketches/constraints;
- arbitrary boolean feature trees;
- arbitrary sweep paths;
- production-grade manufacturing geometry;
- final structural finite-element analysis.

A future sweep/path operation may be considered only if real gameplay needs justify the added complexity.

The longitudinal station model is valuable specifically because it gives players broad creative freedom while remaining easy to validate, mesh, collide, simulate, serialize, and compile.

## 11. Architectural invariant

The intended vehicle-authoring model can be summarized as:

> Wings are procedural planforms compiled into aerodynamic zones. Fuselages are procedural longitudinal solids compiled into geometry, structure, mass, aerodynamics, collision, and interior representations. Gameplay modules then assign purpose to that compiled volume.

This separation should allow future systems such as structural damage, pressure simulation, resource plumbing, passenger habitation, cargo handling, and cutouts to evolve without replacing the core vehicle geometry representation.
