# Procedural aerodynamic surfaces

Status: design baseline. Exact editor UX, panelization thresholds, structural limits, airfoil data, and balancing values are TBD.

## 1. Design goal

Project Thessa uses procedural aerodynamic surfaces rather than a catalogue of fixed wing parts.

The central rule is:

> The editor stores an aerodynamic surface as authoring geometry; leaving the hangar compiles that geometry into solver-ready aerodynamic panels, render geometry, collision geometry, and structural data.

Runtime flight code must not depend on editor splines or render meshes. The existing `AeroPanel` representation remains the solver-facing force primitive.

Primary interaction references are Juno: New Origins, Kerbal Space Program 2 procedural wings, and SimplePlanes 2. They are UX and authoring references rather than physics specifications.

## 2. One procedural surface model

There are no separate fixed-shape parts for rectangular, swept, delta, ogival, tail, or fin surfaces. A single procedural surface model can represent all of them.

Simple users should be able to obtain conventional forms from a small parameter set. Advanced users should be able to edit the underlying planform directly.

The preferred authoring representation is a flat local planform defined by two spanwise spline boundaries:

- leading edge `x_le(s)`;
- trailing edge `x_te(s)`;
- normalized span coordinate `s` from root to tip.

For every valid station:

```text
chord(s) = x_te(s) - x_le(s)
```

This keeps the object recognizably wing-like while allowing nearly arbitrary useful planforms.

A simple trapezoidal wing is therefore only a low-complexity spline. More advanced users may create strongly curved leading/trailing edges, ogival or compound-delta planforms, and other smooth shapes without switching to another part type.

The editor may expose convenience controls such as span, root chord, tip chord, sweep, taper, or tip offset, but these are only views over the same underlying spline representation.

## 3. Spanwise section data

Planform alone does not fully define the surface. Spanwise section data may additionally describe:

- local airfoil/profile;
- thickness or thickness-to-chord ratio;
- incidence;
- geometric twist;
- optional camber/profile parameters;
- structural thickness or spar envelope when structural simulation consumes it.

The first implementation does not need to expose every parameter to the user. The format should nevertheless leave a clean extension point for per-section aerodynamic profiles rather than assuming one global airfoil forever.

## 4. Mechanization is independent from planform

Part variants should primarily describe mechanical behavior, not restrict available planform shapes.

The same procedural geometry may be used as:

- a fixed aerodynamic surface;
- a fully moving surface / all-moving tail;
- a surface carrying one or more hinged control regions.

A fully moving surface rotates the complete aerodynamic surface about its configured hinge/actuation axis. It is not a special aerodynamic primitive.

## 5. Embedded control surfaces

Control surfaces are regions defined on top of the parent procedural surface rather than separate fixed-shape wing parts.

A control region should be describable by:

- a spanwise interval or spanwise boundary spline;
- chordwise leading/trailing boundaries;
- a hinge line;
- deflection limits;
- actuator/control-channel assignment.

This supports conventional ailerons, elevators, rudders, elevons, flaps, spoilers, airbrakes, and unusual custom surfaces with the same representation.

Control-surface boundaries may themselves be curved and should follow the procedural parent geometry rather than assuming rectangular cuts.

### 5.1 Nested surfaces and trim tabs

The representation should permit a hinged region to contain a smaller hinged region.

This naturally supports:

- trim tabs;
- servo tabs;
- anti-servo tabs;
- similar secondary control devices.

A trim tab is therefore not a hard-coded special case in the aerodynamic solver. It is a small nested control region with its own command/trim behavior.

## 6. Hangar compilation

Leaving the hangar, loading a vehicle asset for simulation, or otherwise finalizing construction runs a deterministic vehicle compilation step.

Conceptually:

```text
ProceduralSurface
    |
    +-- validate spline/section geometry
    +-- sample planform and section functions
    +-- split at important geometric and mechanism boundaries
    +-- derive local aerodynamic properties
    |
    v
Compiled surface data
    +-- render mesh
    +-- collision geometry
    +-- structural representation
    +-- AeroPanel[]
    +-- ControlSurfaceDefinition[] / equivalent control mapping
```

The authoring representation is not evaluated in the flight hot path.

## 7. Aerodynamic panel generation

The compiler converts the procedural surface into a relatively small set of aerodynamic zones.

An aerodynamic panel is a force-integration zone, not a render triangle. Render tessellation and aerodynamic panelization are intentionally independent.

For each generated zone the compiler should derive the solver inputs already represented by `AeroPanel`, including where applicable:

- area;
- representative chord;
- representative span;
- planform aspect ratio;
- mean aerodynamic sweep;
- thickness-to-chord ratio;
- force sample position;
- geometry-derived center of pressure;
- local chord and lift axes;
- control deflection ownership;
- future profile/polar reference.

### 7.1 Adaptive panelization

Panel density should follow aerodynamic/geometric complexity rather than a fixed grid size.

A new panel boundary should be introduced when needed because of a meaningful change in, for example:

- local chord;
- sweep;
- twist/incidence;
- section profile or thickness;
- curvature of the planform;
- control-surface ownership;
- other compiler error estimates.

Simple rectangular or trapezoidal surfaces may compile to very few aerodynamic zones. Complex curved planforms may require many more without affecting render-mesh density.

Exact subdivision/error thresholds are TBD and should be chosen from accuracy/performance measurements rather than a fixed design-time panel count.

### 7.2 Mechanization boundaries are hard splits

Every hinge/control-region boundary that changes aerodynamic actuation must force a panelization boundary.

No compiled panel should straddle two regions that can receive different deflections. This lets runtime control continue to operate on precompiled panel groups rather than querying procedural geometry.

## 8. Runtime representation

The current solver-neutral runtime model is the intended target of compilation:

- `AeroPanel` remains the local aerodynamic force primitive;
- control channels address one or more compiled panels;
- the runtime evaluates panels, not splines and not mesh triangles.

This preserves an important architectural boundary:

> Users edit physical geometry; the vehicle compiler derives the numerical model; the runtime consumes only compiled numerical data.

## 9. Future aerodynamic profile data

The existing aerodynamic model still contains vehicle/global coefficients that will eventually be too coarse for surfaces with different section profiles.

Procedural-surface authoring should therefore avoid baking in the assumption that all compiled panels share one permanent global airfoil model.

A future extension may attach an `AeroProfileId`, polar reference, or equivalent per-panel/per-section aerodynamic description. This is an extension point, not a requirement for the first procedural-wing implementation.

## 10. Non-goals for the first implementation

The initial system does not need to solve:

- arbitrary CFD from render geometry;
- unrestricted free-form 3D surface sculpture;
- final aeroelastic deformation;
- final structural failure/load limits;
- every real-world high-lift device;
- final per-airfoil polar format.

It should instead establish a stable authoring-to-compiled-data pipeline that can grow into those features without replacing the vehicle representation.