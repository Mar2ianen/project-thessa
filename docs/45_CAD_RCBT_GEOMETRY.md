# 45 — Parametric CAD assets and adaptive RCBT meshing

Status: design baseline. The repository already contains the reusable RCBT
topology/GPU machinery and an experimental hardware mesh-shader consumer for
terrain; the CAD importer, normalized BRep asset format, CAD-specific adaptive
surface evaluator, and CAD runtime/editor integration described here are not yet
implemented.

## 1. Decision

Project Thessa should treat most hard-surface vehicle and station geometry as
**parametric/CAD geometry first, triangle meshes second**.

The canonical asset is not a fixed tessellation. It is a compact geometric
description containing topology, exact or bounded-error surfaces, material and
mechanical semantics. Triangle geometry is a derived presentation cache produced
for the current camera, renderer capability, and error budget.

The core invariant is:

> A triangle is a temporary representation of a surface, not the source of
> truth for a mechanical asset.

This extends the same representation-on-demand policy already used by planetary
terrain:

~~~text
planet field / CAD surface / other continuous source
                    |
                    v
              adaptive topology
                    |
                    v
          bounded-error local samples
                    |
                    v
        GPU-generated raster geometry
~~~

RCBT is the intended adaptive-topology mechanism because the project already
uses it for view-dependent planetary detail, stable logical identities,
incremental refinement/coarsening, and GPU-driven presentation.

## 2. Scope

This policy is intended for geometry that is naturally described by dimensions,
profiles, sweeps, revolutions, booleans, holes, fillets, and mechanical
relationships:

- docking ports and attachment hardware;
- tanks and pressure vessels;
- rocket chambers and nozzles;
- feed pipes, manifolds, ducts and structural tubes;
- fuselage and fairing sections;
- wings, control surfaces and aerodynamic bodies where the authoring model is
  parametric;
- landing gear, wheel hubs, struts and actuators;
- radiators and solar-array structure;
- station trusses, pressure modules and machine housings;
- robotics and other hard-surface mechanisms.

A conventional imported triangle mesh remains valid where it is the correct
source material, for example the current X-15 visual reference. Characters,
organic art, scanned geometry, vegetation and one-off sculpted objects are not
forced through a CAD representation merely for architectural uniformity.

## 3. STEP is an interchange/reference format, not necessarily the gameplay authoring model

STEP is a useful input because it preserves BRep topology and exact analytic or
spline surfaces much better than a game mesh format. It is also a useful
round-trip/export format for FreeCAD and other CAD tooling.

However, raw STEP generally does not preserve a complete editable feature tree,
constraints, spreadsheet parameters, or application-specific procedural intent.
Therefore distinguish three layers:

~~~text
editable design / feature graph
        |
        | regenerate / import
        v
normalized BRep asset
        |
        | adaptive presentation
        v
RCBT -> raster geometry
~~~

For imported third-party or manually authored CAD, STEP may be the highest
available source of truth.

For native Thessa procedural parts, the preferred editable source is a
game-owned design schema/feature graph. That graph may emit STEP for tooling and
must emit the same normalized BRep/runtime surface representation used by
imported CAD.

Do not put a general STEP parser in the per-frame render hot path.

## 4. Initial reference fixture: D1 docking-port assembly

The initial model used to validate this design is the supplied
'thessa_d1_v2_2_free.step' D1 docking-port assembly. It is not currently checked
into the repository; this section records the measured geometry so future
importers can reproduce the same audit.

Measured with an OpenCascade STEP/BRep reader:

| Property | Value |
| --- | ---: |
| STEP file size | ~5.9 MB |
| Bounding box | ~1252 x 1223 x 332 mm |
| Solids | 139 |
| Faces | 1744 |
| Unique edges | 4229 |

Face surface classes:

| Surface | Faces |
| --- | ---: |
| Plane | 1201 |
| Cylinder | 504 |
| Sphere | 19 |
| B-spline | 18 |
| Cone | 2 |

Unique edge curve classes:

| Curve | Edges |
| --- | ---: |
| Line | 2936 |
| Circle | 1226 |
| B-spline | 29 |
| Ellipse | 19 |
| Other | 19 |

The important result is not the exact count. Approximately 99% of the faces are
analytic planes/cylinders/spheres/cones. The docking-port model is therefore a
good example of why storing a permanently tessellated mesh is wasteful: most of
the visible geometry can be evaluated directly from very compact analytic
descriptions, while the small B-spline tail uses the generic evaluator.

The assembly also contains real soft-capture mechanics: soft-capture ring,
guide petals, damper barrels and rods, clevises, rails/rollers, hard-latch hooks,
pressure sealing hardware, service panels and docking-aid hardware. The visual
asset can therefore share semantic identities with mechanical simulation
without inferring mechanism meaning from triangles.

This model should become the first golden fixture for the CAD pipeline once its
asset provenance/license is recorded and it is deliberately added to the
repository.

## 5. Normalized CAD/BRep runtime asset

The renderer must not depend directly on FreeCAD document objects or on one CAD
kernel's private handles. Import/tooling converts source CAD into a
backend-neutral normalized representation.

A representative logical shape is:

~~~text
CadAsset
  solids[]
  shells[]
  faces[]
    surface_id
    trim_loops[]
    material_id
    semantic_id?
  edges[]
    curve_id
    adjacent_faces[]
  vertices[]

Surface
  Plane
  Cylinder
  Cone
  Sphere
  Torus
  SurfaceOfRevolution
  SurfaceOfExtrusion
  Bezier
  BSpline
  ...

Curve
  Line
  Circle
  Ellipse
  Bezier
  BSpline
  ...
~~~

Exact serialized structs are intentionally left open until the importer spike.
The observable requirements are more important than the initial storage layout:

- stable face/edge identities;
- shared edge topology preserved exactly;
- explicit trim loops;
- explicit local transforms;
- analytic primitives kept analytic instead of immediately converted to
  splines;
- bounded-error spline representation;
- material and optional gameplay/mechanical semantic identifiers;
- deterministic output for identical input.

The normalized asset belongs to a reusable engine/tooling layer and must not
contain Bevy entities, Bevy meshes, wgpu handles or Vulkan objects.

## 6. Adaptive surface representation

Each BRep face is a parametric domain. Conceptually:

~~~text
(face_id, u, v) -> position
                -> geometric normal
                -> optional derivatives / curvature
~~~

RCBT controls how finely that domain is represented for the current view.

A coarse leaf may emit only the minimum triangles required to cover the trimmed
domain. As projected error grows, the leaf is split and the same exact surface
is evaluated at additional parameter locations.

For common analytic surfaces the evaluators are cheap:

~~~text
Plane(u, v)
Cylinder(phi, z)
Cone(phi, z)
Sphere(theta, phi)
~~~

B-spline/NURBS surfaces use a generic evaluator but remain a minority in the
initial D1 fixture.

The topology must not assume that all faces are rectangular untrimmed patches.
Trim boundaries are first-class input to coverage and error evaluation.

## 7. Error metric

Refinement is driven by an explicit screen-space/geometric error budget, not by
a named LOD level.

A CAD leaf's refinement score should include at least:

~~~text
projected surface deviation
+ curvature / normal deviation
+ silhouette importance
+ trim-boundary approximation error
+ optional material/displacement requirement
~~~

The exact scalar combination requires measurement. The renderer may use cheap
conservative bounds before evaluating the full metric.

Important cases:

- a large planar face can remain extremely coarse even close to the camera;
- a curved silhouette may require refinement while an equally close flat
  interior does not;
- a small fillet may become relevant only during inspection;
- sub-pixel geometry must not be refined merely because its physical dimensions
  are small;
- material microstructure should not force geometric subdivision when normal,
  displacement or BRDF representation is cheaper.

The user-facing quality setting changes allowed presentation error and resource
budgets. It does not change the source geometry.

## 8. Crack-free shared edges

Independent face tessellation is not allowed to create cracks along BRep seams.

A shared topological edge owns the boundary discretization used by all adjacent
faces. Both faces evaluate their boundary vertices from the same edge
subdivision state rather than inventing two approximately equal copies.

Conceptually:

~~~text
          shared BRep edge
         /               \
        v                 v
     face A             face B
  interior RCBT      interior RCBT
~~~

The face interiors may refine independently. The boundary samples must remain
identical.

The same rule applies to:

- periodic seams on cylinders/revolved surfaces;
- trimmed B-spline boundaries;
- transitions between analytic and spline faces;
- CAD boolean seams;
- hard edges where position is continuous but normal is deliberately
  discontinuous.

A crack-free edge invariant is a correctness requirement and needs numeric
tests, not only screenshots.

## 9. GPU presentation paths

The existing terrain renderer already demonstrates both required classes of
presentation path:

1. portable compute/indexed/indirect raster;
2. experimental hardware mesh shaders.

'crates/bevy-rcbt/src/render.rs' already contains a feature-gated mesh-shader
consumer for RCBT terrain. A 33x33 terrain patch is emitted as sixteen 8x8
meshlets. The current game client does not enable that feature and keeps the
indexed GPU path active.

CAD must reuse the same architectural split rather than invent a renderer tied
to mesh shaders.

Preferred flow:

~~~text
normalized CAD surfaces
        |
        v
RCBT leaf topology
        |
        +--> portable path:
        |      compute/evaluate dirty leaves
        |      compact visible triangles
        |      indirect draw
        |
        +--> mesh-shader path:
               evaluate selected leaves/meshlets
               emit transient vertices/primitives
~~~

Both consumers see the same semantic CAD asset and the same adaptive topology.

A hardware capability changes how triangles are emitted, not what the asset
means.

## 10. Incremental generation and caching

The terrain path already has the correct performance philosophy: changing or
streaming one page does not regenerate the entire visible planet. CAD should
keep the same rule.

Refining one local feature should update only affected leaves/boundary data.
Unchanged geometry remains resident.

Useful caches include:

- normalized surface descriptors;
- trim acceleration structures;
- shared-edge subdivision state;
- current RCBT topology;
- evaluated leaf samples;
- GPU-visible face/edge descriptor pages;
- optional triangulation fallback pages for unsupported surface classes.

Cache identity must follow stable CAD topology identities rather than transient
triangle indices.

## 11. Precision and coordinate hierarchy

CAD inspection creates a large dynamic range: the player may look at the same
part from kilometers away and then move a camera millimeters from a latch.

Do not evaluate microscopic detail by adding f32 offsets directly to
astronomical/world positions.

Use a coordinate hierarchy such as:

~~~text
authoritative world position          f64
        |
        v
vehicle / station local frame         f64
        |
        v
part / solid local frame              f64 or stable high precision
        |
        v
face / patch render-local frame       f32-friendly
        |
        v
GPU surface evaluation
~~~

Only the final small local domain needs to fit ordinary GPU f32 precision.

The same asset must not start visibly jittering merely because it is mounted on
a vehicle far from the system origin.

## 12. Materials are independent from tessellation

A material must not be painted in triangle UV space if those triangles are
ephemeral.

The material sampler consumes stable surface information, for example:

~~~text
MaterialSample
  part_local_position
  geometric_normal
  face_id
  face_parametric_uv
  material_frame
  curvature / edge distance where useful
  semantic region id
~~~

Use coordinate systems according to the phenomenon:

- part-local 3D coordinates for dirt, soot, oxidation and effects that must
  cross BRep seams continuously;
- face/material coordinates for machining marks, directional brushed metal or
  intentionally aligned patterns;
- explicit decals/feature-local coordinates for labels and markings.

The existing material-page/microstore work is reusable here. CAD does not need
a second unrelated material system. Procedural material fields can generate
albedo/roughness/normal/displacement pages or be evaluated directly according
to measured cost.

Changing tessellation density must not move the material pattern.

## 13. Multi-scale surface detail

The exact CAD surface should not be forced to carry every visible spatial
frequency.

A useful representation split is:

~~~text
meters .. millimeters
    exact/parametric CAD + adaptive RCBT tessellation

millimeters .. tens of micrometers
    bounded procedural displacement / normal detail

smaller optical structure
    roughness / anisotropy / BRDF parameters
~~~

A machined cylinder can therefore remain a mathematically exact cylinder while
its turning grooves and scratches appear only when they become optically
relevant.

This avoids exploding topology to represent details that belong in the material
model.

## 14. Editing and compilation

A native procedural part should expose physically meaningful parameters rather
than mesh-edit operations.

Examples:

~~~text
docking port:
  seal diameter
  capture-ring dimensions
  latch count / placement
  damper stroke and attachment geometry
  shell/ring thickness
  service connector layout

rocket nozzle:
  throat radius
  expansion ratio
  contour parameters
  length
  wall thickness
  cooling geometry
~~~

Changing those parameters regenerates the normalized geometric representation
and all dependent compiled products.

The hangar/editor compile boundary may produce:

~~~text
editable CadDesign / feature graph
          |
          +--> normalized BRep
          +--> render surface descriptors
          +--> collision representation
          +--> structural graph / section properties
          +--> mass and inertia
          +--> actuator/joint anchors
          +--> thermal/material interfaces
          +--> service/fluid/electrical attachment geometry
~~~

Do not infer these semantic properties back from the rendered triangles.

For imported STEP without native feature history, editing may initially be
limited to transforms, material/semantic assignment and a bounded set of
recognized geometric operations. Full semantic editing requires a native
feature/design graph or preserved authoring metadata.

## 15. Collision and physics representation

The adaptive render tessellation is not authoritative collision geometry.

Collision and mechanical systems are separate consumers of the same design:

- simple analytic primitives where sufficient;
- convex/compound proxies;
- dedicated contact surfaces;
- local high-detail geometry where contact behavior truly requires it;
- explicit joint/actuator anchors for mechanisms.

For the D1 port, soft capture should be driven by the modeled capture ring,
dampers, guides and joint constraints, not by collisions against whatever
triangle density the camera happens to request.

Render refinement therefore cannot change authoritative mass, collision,
stiffness or docking behavior.

## 16. Streaming and visibility

A large station may eventually contain many CAD parts with enormous potential
surface complexity. Cost must scale with visible error, not with the maximum
tessellation of every source model.

Required behavior:

- off-screen/inactive CAD geometry performs effectively no per-frame surface
  work;
- distant parts collapse to coarse analytic leaves or an object-level impostor;
- visible parts refine only where projected error requires it;
- occluded parts may stop refining and may coarsen;
- inspection camera proximity can spend much more geometry budget on a tiny
  local region without globally refining the vehicle;
- stable leaf identities should permit temporal reuse and avoid rebuild
  thrashing.

Object-level culling happens before expensive per-face refinement.

## 17. Backend and tooling boundary

CAD import is tooling/asset-pipeline work. Renderer/runtime surface evaluation
is engine work.

Suggested layering:

~~~text
tools/cad-import/
    STEP / FreeCAD / other importer
    normalization
    validation
    optional feature recognition

crates/cad-core/
    backend-neutral normalized surfaces/topology
    deterministic serialization
    CPU evaluators/reference tests

crates/cad-rcbt/
    face/edge adaptive policy
    error bounds
    RCBT mapping

renderer backends/
    wgpu portable consumer
    native Vulkan consumer later

bridges/bevy/
    temporary integration only
~~~

Names are illustrative; the boundary is normative.

Choice of CAD kernel/import library is still open and must receive an explicit
license, determinism and performance review before it becomes a repository
dependency.

## 18. Golden tests

The D1 docking port should exercise the first serious importer/render tests.

### Import/topology

- deterministic solid/face/edge counts for the pinned fixture;
- stable surface/curve class inventory;
- shared-edge adjacency is preserved;
- trim loops remain valid;
- bounding box is stable within a declared tolerance;
- serialize -> deserialize keeps observable geometry.

### Surface evaluation

For every supported surface class:

- known analytic sample vectors;
- position error against the CAD-kernel oracle;
- normal error;
- boundary/periodic seam agreement.

### Tessellation

- maximum geometric deviation below the declared leaf error;
- screen-space error obeys the requested pixel budget;
- adjacent faces share boundary positions exactly within the chosen numeric
  contract;
- refinement/coarsening does not produce holes or overlaps;
- camera movement does not cause unbounded topology thrash.

### Materials

- material coordinates remain stable across refinement;
- part-local continuous effects cross CAD seams without visible discontinuity;
- face-local machining patterns preserve intended orientation.

## 19. Benchmarks

At minimum benchmark the D1 fixture across a deterministic camera sweep:

~~~text
far station view
    -> approach
    -> docking distance
    -> close inspection
    -> extreme local inspection
~~~

Record:

- active solids/faces;
- RCBT leaves;
- leaf split/merge count;
- emitted vertices/primitives;
- CPU selection/update time;
- surface-evaluation time;
- GPU geometry/raster time;
- resident geometry/material bytes;
- shared-edge cache size;
- maximum measured geometric/screen-space error.

Compare at least:

1. adaptive CAD/RCBT;
2. a conventional static high-detail tessellation;
3. a small manually prepared fixed-LOD chain if available.

The target is not "CAD is always faster". The target is that cost follows the
visible detail actually required while preserving a source that remains
editable and scale-independent.

## 20. Migration path

Implement in slices:

1. Pin the D1 fixture and provenance after the asset is ready to enter the repo.
2. Build a command-line STEP audit/import tool.
3. Define the normalized surface/topology serialization.
4. Support planes, cylinders, cones and spheres first; they cover ~99% of D1
   faces.
5. Add B-spline surface/curve evaluation and trimmed-face coverage.
6. Add shared-edge ownership and crack-free adaptive subdivision.
7. Render a single imported face through the existing portable RCBT GPU path.
8. Render the full D1 assembly with view-dependent refinement.
9. Add the mesh-shader consumer using the existing RCBT mesh-shader capability
   as the architectural reference, not as a mandatory backend.
10. Attach existing procedural material/microstore sampling.
11. Connect stable semantic subparts to docking/collision/actuator data.
12. Add native procedural CadDesign parts and in-game editing.
13. Expand the same path to engines, tanks, fuselages, gear and station
    structures.

## 21. Architecture invariant

The intended final relationship is:

~~~text
editable physical design
        |
        v
normalized exact/bounded-error geometry
        |
        +--> simulation/compiler products
        |
        +--> RCBT adaptive surface topology
                 |
                 +--> portable compute/indirect raster
                 +--> mesh shaders where supported
                 +--> RT proxy/full geometry where useful
~~~

If implementing a new renderer requires changing the physical CAD asset, or if
editing a dimension requires manually rebuilding a family of fixed LOD meshes,
the boundary is wrong.
