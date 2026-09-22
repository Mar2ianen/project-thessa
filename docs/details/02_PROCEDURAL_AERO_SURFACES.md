# Procedural aerodynamic surfaces

Status: design baseline. Exact editor UX, panelization thresholds, structural limits, airfoil data, and balancing values are TBD.

## 1. Design goal

Project Thessa uses procedural aerodynamic surfaces rather than a catalogue of fixed wing parts.

The central rule is:

> The editor stores an aerodynamic surface as authoring geometry; leaving the hangar compiles that geometry into solver-ready aerodynamic panels, render geometry, collision geometry, mechanism data, and structural data.

Runtime flight code must not depend on editor splines or render meshes. The existing `AeroPanel` representation remains the solver-facing force primitive.

Primary interaction references are Juno: New Origins, Kerbal Space Program 2 procedural wings, and SimplePlanes 2. They are UX and authoring references rather than physics specifications.

## 2. One procedural surface model

There are no separate fixed-shape parts for rectangular, swept, delta, ogival, tail, fin, winglet, or folding-wing surfaces. A single procedural surface authoring model should be able to represent all of them.

Simple users should be able to obtain conventional forms from a small parameter set. Advanced users should be able to edit the underlying planform and bend geometry directly.

The core representation intentionally separates:

- the flat 2D planform;
- the mapping of that planform into 3D;
- local section/profile data;
- mechanical regions and joints.

This makes "draw the wing, then bend or mechanize it" the normal workflow rather than forcing users to assemble a curved wing from many unrelated parts.

## 3. Flat planform authoring

The preferred base representation is a flat local planform defined by two spanwise spline boundaries:

- leading edge `x_le(s)`;
- trailing edge `x_te(s)`;
- normalized span coordinate `s` from root to tip.

For every valid station:

```text
chord(s) = x_te(s) - x_le(s)
```

A point inside the undeformed surface may be parameterized by span coordinate `s` and normalized chord coordinate `u`:

```text
P_flat(s, u) = lerp(leading_edge(s), trailing_edge(s), u)
u in [0, 1]
```

This keeps the object recognizably wing-like while allowing nearly arbitrary useful planforms.

A simple trapezoidal wing is therefore only a low-complexity spline. More advanced users may create strongly curved leading/trailing edges, ogival or compound-delta planforms, and other smooth shapes without switching to another part type.

The editor may expose convenience controls such as span, root chord, tip chord, sweep, taper, or tip offset, but these are only views over the same underlying spline representation.

## 4. Out-of-plane bend curve

After the flat planform is authored, the surface may be bent out of its original plane by a third spanwise curve.

The first implementation should expose this as a simple bend/elevation function such as:

```text
z_bend(s)
```

or an equivalent spanwise reference path.

The flat planform remains the canonical parameter domain. The bend curve maps spanwise stations into 3D and defines the local span frame used to place each section.

Conceptually:

```text
flat planform
    |
    +-- x_le(s)
    +-- x_te(s)
    |
    v
spanwise bend / reference curve
    |
    +-- z_bend(s)
    +-- local section frame
    |
    v
3D aerodynamic surface
```

The local tangent of the bend/reference curve should participate in construction of the section frame. Section twist remains an independent degree of freedom rather than being implicitly baked into the bend.

This should support, using one authoring surface:

- ordinary dihedral and anhedral;
- gull and inverted-gull wings;
- smoothly canted tips;
- blended winglets;
- near-vertical tip surfaces;
- Pathfinder-like wings whose outer portion rises continuously out of the main wing plane.

The first implementation does not need unrestricted free-form 3D sculpture. A spanwise bend curve over the flat planform is deliberately more constrained and easier to validate, compile, and edit.

The compiler must distinguish projected planform quantities from actual 3D surface quantities. Bending a surface changes projected span/area and local aerodynamic frames without changing the material surface area merely because of orientation.

## 5. Spanwise section data

Planform and bend geometry still do not fully define the surface. Spanwise section data may additionally describe:

- local airfoil/profile;
- thickness or thickness-to-chord ratio;
- incidence;
- geometric twist;
- optional local roll/cant adjustment when not fully described by the bend frame;
- optional camber/profile parameters;
- structural thickness or spar envelope when structural simulation consumes it.

The first implementation does not need to expose every parameter to the user. The format should nevertheless leave a clean extension point for per-section aerodynamic profiles rather than assuming one global airfoil forever.

## 6. Mechanization is independent from geometry

Part variants should primarily describe mechanical behavior, not restrict available planform shapes.

The same procedural geometry may be used as:

- a fixed aerodynamic surface;
- a fully moving surface / all-moving tail;
- a surface carrying one or more control regions;
- a surface containing one or more fold joints.

A fully moving surface rotates the complete aerodynamic surface about its configured hinge/actuation axis. It is not a special aerodynamic primitive.

A fold joint rotates a child/outboard region relative to the parent surface and is not the same mechanism as a control surface.

## 7. Embedded control surfaces

Control surfaces are regions drawn on top of the parent procedural surface rather than separate fixed-shape wing parts.

The preferred editor workflow follows the SimplePlanes-style interaction model:

1. create the parent aerodynamic surface;
2. draw/select a control region directly on that surface;
3. choose a control/mechanism preset;
4. optionally override the preset's kinematics, limits, actuator, and control mixing.

A control region should be describable by:

- a spanwise interval or spanwise boundary spline;
- chordwise leading/trailing boundaries;
- a hinge line or other kinematic reference;
- deflection/translation limits;
- actuator assignment;
- one or more control-channel inputs.

Control-surface boundaries may themselves be curved and should follow the procedural parent geometry rather than assuming rectangular cuts.

### 7.1 User-facing control presets

User-facing types such as the following should primarily be presets over generic region geometry, kinematics, and control mixing:

- aileron;
- elevator;
- rudder;
- elevon;
- flaperon;
- flap;
- slat;
- spoiler;
- airbrake;
- trim/servo/anti-servo tab.

For example, an elevon is not a unique aerodynamic primitive. It is a region whose command is a mix of pitch and roll inputs. A flaperon similarly mixes flap deployment with roll command.

Conceptually:

```text
elevon_command   = pitch * k_pitch + roll * k_roll
flaperon_command = flap  * k_flap  + roll * k_roll
```

Presets may choose sensible signs, gains, symmetric/asymmetric pairing, limits, and default actuator behavior, but advanced users should be able to edit those mappings.

Slats and other devices whose motion is not a pure hinge rotation may use a different kinematic preset while retaining the same region-on-parent authoring model.

### 7.2 Nested surfaces and trim tabs

The representation should permit a hinged region to contain a smaller hinged region.

This naturally supports:

- trim tabs;
- servo tabs;
- anti-servo tabs;
- similar secondary control devices.

A trim tab is therefore not a hard-coded special case in the aerodynamic solver. It is a small nested control region with its own command/trim behavior.

## 8. Folding surfaces

Folding is a first-class mechanical property layered on top of the same procedural surface geometry.

A fold joint divides the parent surface into an inboard/parent region and an outboard/child region. The child region receives a rigid transform around the fold hinge while retaining its authored planform, bend curve, section data, controls, and nested mechanisms.

A fold-joint definition should be able to describe:

- fold boundary / span station;
- hinge line and axis;
- deployed angle;
- one or more stowed/folded angles;
- deployment rate;
- hard stops;
- actuator;
- lock state and lock limits;
- allowed operating states or flight-envelope constraints;
- optional automatic deployment/folding rules.

Typical use cases include:

- Boeing 777X-style folding wingtips for ground/gate compatibility;
- Dream Chaser-style wing folding for launch inside a payload fairing;
- carrier/storage folding;
- variable-geometry concepts where in-flight folding is intentionally allowed.

Ground/launch-only folding and in-flight variable geometry use the same basic joint representation but may have very different structural limits and control rules.

Folding must not destroy the underlying aerodynamic definition. The material/3D surface area of a rigidly folded child region is unchanged by the fold transform, while projected area, projected span, collision envelope, local force directions, and aerodynamic interactions may change.

## 9. Hangar compilation

Leaving the hangar, loading a vehicle asset for simulation, or otherwise finalizing construction runs a deterministic vehicle compilation step.

Conceptually:

```text
ProceduralSurface
    |
    +-- validate planform splines
    +-- validate bend/reference curve
    +-- validate section geometry
    +-- validate control regions and fold topology
    +-- sample planform, bend, and section functions
    +-- split at important geometric and mechanism boundaries
    +-- derive local aerodynamic properties and transforms
    |
    v
Compiled surface data
    +-- render mesh
    +-- collision geometry
    +-- structural representation
    +-- AeroPanel[]
    +-- ControlSurfaceDefinition[] / equivalent control mapping
    +-- FoldJointDefinition[] / equivalent mechanism data
```

The authoring representation is not evaluated in the flight hot path.

A single editor surface may therefore compile into multiple connected numerical/structural regions while still remaining one object from the user's perspective.

## 10. Aerodynamic panel generation

The compiler converts the procedural surface into a relatively small set of aerodynamic zones.

An aerodynamic panel is a force-integration zone, not a render triangle. Render tessellation and aerodynamic panelization are intentionally independent.

For each generated zone the compiler should derive the solver inputs already represented by `AeroPanel`, including where applicable:

- actual 3D area;
- projected planform area where the model needs it;
- representative chord;
- representative span;
- planform aspect ratio;
- mean aerodynamic sweep;
- thickness-to-chord ratio;
- force sample position;
- geometry-derived center of pressure;
- local chord, span, normal, and lift axes;
- control deflection ownership;
- fold-joint ownership / transform chain;
- future profile/polar reference.

### 10.1 Adaptive panelization

Panel density should follow aerodynamic/geometric complexity rather than a fixed grid size.

A new panel boundary should be introduced when needed because of a meaningful change in, for example:

- local chord;
- sweep;
- bend/dihedral/cant;
- twist/incidence;
- section profile or thickness;
- curvature of the planform;
- control-surface ownership;
- fold-joint ownership;
- other compiler error estimates.

Simple rectangular or trapezoidal surfaces may compile to very few aerodynamic zones. Complex curved/bent planforms may require many more without affecting render-mesh density.

Exact subdivision/error thresholds are TBD and should be chosen from accuracy/performance measurements rather than a fixed design-time panel count.

### 10.2 Mechanization boundaries are hard splits

Every control-region boundary or fold boundary that changes independent motion must force a panelization boundary.

No compiled panel should straddle two regions that can receive different deflections or rigid transforms. This lets runtime control continue to operate on precompiled panel groups rather than querying procedural geometry.

## 11. Runtime representation

The current solver-neutral runtime model is the intended target of compilation:

- `AeroPanel` remains the local aerodynamic force primitive;
- control channels address one or more compiled panels;
- fold joints transform precompiled panel groups;
- the runtime evaluates compiled panels and mechanism state, not editor splines and not mesh triangles.

This preserves an important architectural boundary:

> Users edit physical geometry; the vehicle compiler derives the numerical model; the runtime consumes only compiled numerical data.

## 12. Compiler validation and reference-aircraft tests

Procedural-wing compilation should have both analytic geometry tests and golden reconstructions of real vehicles.

The goal of the real-vehicle suite is not to claim that the current aerodynamic solver exactly reproduces measured aircraft performance. It is to verify that the authoring model can represent real planforms/mechanisms and that compilation preserves their known geometry and kinematics.

### 12.1 Analytic compiler tests

Simple synthetic fixtures should cover geometry for which the expected result is known independently:

- rectangular wing;
- trapezoidal wing;
- swept trapezoid;
- constant-dihedral wing;
- smoothly bent wing with an analytic bend function;
- one hinged control region;
- nested trim tab;
- one fold joint at a known station.

Tests should independently integrate or derive expected values rather than comparing the compiler to another call into the same implementation.

At minimum they should verify:

- total actual surface area;
- projected planform area;
- span and projected span;
- mean/representative chord quantities;
- panel centroid and aggregate centroid;
- local frame orientation;
- symmetry;
- continuity across non-mechanical subdivision boundaries;
- exact ownership at control/fold hard boundaries;
- preservation of child-region material area through rigid folding.

Panelized approximations should converge toward the analytic reference as compiler tolerance is tightened.

### 12.2 Real-vehicle golden reconstructions

Maintain small authoring fixtures reconstructed from public manufacturer/NASA geometry or sufficiently good public drawings.

Initial reference set:

- **Boeing 777X** — validates a conventional swept wing plus folding wingtip. Public Boeing data gives a 71.8 m extended wingspan and 64.8 m ground/folded wingspan. The test should compile both mechanism states and compare the resulting external envelope.
- **Dream Chaser / Tenacity** — validates a compact lifting-body spaceplane whose wings fold into the launch configuration. NASA/Sierra Space publicly describe the wings as folding for launch inside a 5 m payload fairing; public NASA material also provides a roughly 7 m deployed wingspan for Dream Chaser reference geometry. The fixture should verify deployed geometry, folded transform topology, and launch-envelope fit for the selected documented configuration.
- **Space Shuttle Orbiter** — validates a large highly swept delta-like wing, elevon regions, and compilation of a shuttle-class planform. NASA publishes an Orbiter wingspan of 78 ft / about 23.8 m; additional fixture dimensions should be tied to the exact public drawing/source used.
- **Concorde** — validates a strongly curved/ogival delta planform that requires more than a simple trapezoid and exercises adaptive subdivision of curved leading/trailing edges.

A Pathfinder-like fixture may additionally be kept as a **fictional visual regression** for a single surface with smoothly rising/canted tips and embedded controls. It is useful for feature coverage but must not be treated as real-world validation ground truth.

Reference sources should be recorded alongside each fixture so that a test failure can be distinguished from a changed reconstruction or source assumption.

Useful authoritative starting points include:

- Boeing 777X technical specifications and folding-wingtip material;
- NASA/Sierra Space Dream Chaser launch-configuration material;
- NASA Space Shuttle Orbiter reference dimensions;
- museum/manufacturer/archival drawings for Concorde geometry.

### 12.3 What the real-vehicle tests compare

Each fixture should emit a compact deterministic `CompiledSurfaceSummary` or equivalent golden record containing geometry-only outputs such as:

- deployed and folded/stowed bounding boxes;
- total and projected span;
- total actual surface area;
- projected planform area;
- root/tip/reference chords where meaningful;
- mean aerodynamic chord derived from geometry;
- selected sweep angles;
- bend/dihedral/cant at reference stations;
- control-region area and hinge positions;
- fold hinge positions and child transforms;
- aerodynamic-panel count;
- sum of panel areas;
- panel/control/fold ownership graph.

The tests should compare those outputs against both:

1. an independent integration of the authored geometry;
2. public reference values available for the real vehicle.

Reference tolerances must be stored per fixture and reflect source quality. A manufacturer dimension can use a tight tolerance; a value reconstructed from a perspective-corrected drawing or photograph needs a wider declared tolerance.

The suite should not silently normalize a poor reconstruction until it passes. Any deliberate discrepancy from the public reference should be documented in the fixture.

### 12.4 Mechanism-state invariants

Real and synthetic fold/control tests should additionally assert state invariants:

- rigid folding does not change child material area;
- folding changes the external envelope exactly through the mechanism transform;
- no `AeroPanel` crosses an independently moving hinge/fold boundary;
- control deflection preserves region topology and ownership;
- deploying a fold to its authored flight angle reconstructs the same compiled flight geometry as direct compilation in that state;
- mirrored left/right surfaces remain mirrored after equivalent mechanism commands.

These tests protect the compiler architecture independently of later aerodynamic-model changes.

## 13. Future aerodynamic profile data

The existing aerodynamic model still contains vehicle/global coefficients that will eventually be too coarse for surfaces with different section profiles.

Procedural-surface authoring should therefore avoid baking in the assumption that all compiled panels share one permanent global airfoil model.

A future extension may attach an `AeroProfileId`, polar reference, or equivalent per-panel/per-section aerodynamic description. This is an extension point, not a requirement for the first procedural-wing implementation.

## 14. Non-goals for the first implementation

The initial system does not need to solve:

- arbitrary CFD from render geometry;
- unrestricted free-form 3D surface sculpture;
- final aeroelastic deformation;
- final structural failure/load limits;
- every real-world high-lift linkage;
- exact flight-performance reproduction for every reference aircraft;
- final per-airfoil polar format.

It should instead establish a stable authoring-to-compiled-data pipeline that can grow into those features without replacing the vehicle representation.
