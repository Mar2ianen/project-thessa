# Thessa D1 docking-port MVP asset

This bundle is the first local CAD fixture for the D1 docking-port family. It
is derived from the user-supplied `thessa_d1_v2_2_free.step` file and follows
the CAD/RCBT boundary in `docs/45_CAD_RCBT_GEOMETRY.md`:

- STEP/FreeCAD remain offline authoring and import tooling;
- the FreeCAD documents retain the source BRep assembly and a semantic group
  for the provisional six-part soft-capture mechanism;
- OBJ/glTF are derived presentation products, not the authoritative mechanical
  source;
- `physics.toml` is the Rapier-facing provisional contact/mechanism contract;
- `materials.json` keeps optical material IDs independent of tessellation and
  carries the corresponding Rapier contact coefficients.

## Current MVP

The six visible, radially repeated guide/petal source solids at indices
`17,20,25,28,33,36` are exposed as `SoftCapturePetal_01..06`. Each has a
provisional tangent hinge at radius `0.550 m`, `z = -0.068 m`, and a 25 degree
open pose. The GLB contains a 60-frame open/close animation so the moving parts
are immediately inspectable in a game asset viewer.

The mapping is explicitly provisional until CAD feature recognition confirms
which source solids are guide petals, damper components, rails, and latches.
All final capture tolerances, stiffness/damping values, and load ratings remain
TBD in `docs/details/01_DOCKING_PORTS.md`.

## Rapier boundary

`physics.toml` uses SI metres and the existing Thessa convention:

- Rapier gravity is zero;
- CCD is enabled for contact-active bodies;
- collider density does not define mass or inertia;
- the eight ring proxy cuboids leave a central passage open;
- body mass and inertia must come from Thessa `RigidBodyProperties`;
- the six petal revolute joints map to the public
  `thessa-collision::CollisionWorld::attach_revolute_joint` seam;
- a completed hard dock maps to
  `CollisionWorld::attach_fixed_joint`, while the authoritative docking state
  and pressure progression live in `thessa-sim-core::DockingSession`.

The runtime regression fixture creates two D1 craft from the same proxy
contract, drives them through capture/alignment/hard-dock/pressure
equalization, applies a force to one craft, and then undocks them without an
artificial velocity kick.

Do not promote the provisional proxy or coefficients to final gameplay values
without a structural/contact calibration pass.

## Rebuild

With FreeCAD 1.1.x and Blender 5.x installed:

```text
FreeCADCmd -c "exec(open('tools/cad-import/build_d1_docking_port.py').read())"
blender -b --python tools/cad-import/export_d1_gltf.py
```

The source STEP is intentionally not checked into the repository yet. The
generated `manifest.json` records its SHA-256 and provenance placeholder.
