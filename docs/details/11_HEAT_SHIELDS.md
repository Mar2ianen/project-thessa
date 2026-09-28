# Heat shields and thermal protection

Status: 🟡 partial slice. Shield aerodynamics (§1.3) and the wing tile-layer
toggle (§2.4) are implemented with hangar, runtime, regression, and baker
coverage. Ablation/recession (§1.2) and spline-bounded tile screens (§2.1–2.3)
remain design: no code yet. Implementation must keep following this document
and satisfy the physics-feature definition of done before any remaining row
is called implemented.

Two thermal-protection families share one thermal coupling, but author and
behave differently:

- round ablative shields (capsule forebody discs with an ablator that
  recesses while carrying heat away);
- Starship-like tile screens (reusable hexagonal tiles paving a
  spline-bounded region of a body surface).

Heat shields were explicitly out of scope until now
(`details/10_THERMAL_SYSTEM.md`); this document lifts that boundary for
design, not for runtime behavior.

## 1. Round ablative shields

### 1.1 Authoring

Extends the existing `BodyHeatShield` hangar record (end section supplies
diameter; thickness and shell material are authored). New fields:

```text
heat_of_ablation_j_kg   effective heat absorbed per recessed kilogram
ablation_onset_temp_k   surface temperature where recession starts
char_emissivity         re-radiation emissivity of the charred surface
backing_node            vehicle thermal node receiving conducted heat
```

Diameter continues to derive from the end section; curvature (cap radius)
derives from the end-section shape, never from a free coefficient.

### 1.2 Runtime model

The shield front face is a lumped surface over its disc area:

```text
Qaero = k·sqrt(ρ/Rn)·v³·Adisc     (shared Sutton-Graves input, Rn = cap radius)
Qrerad = εchar·σ·Adisc·(Ts⁴ − Tbg⁴)
Qcond  = Gback·(Ts − Tbacking)    (into the named thermal node)
```

Below onset temperature the surface equilibrates like any hot structure.
Above it, the residual `Qaero − Qrerad − Qcond` recesses ablator mass at
`ṁ = Qnet / h_abl`, decreasing remaining thickness. At zero thickness the
shield raises a burn-through flag and passes full aero heat to the backing
node. No automatic explosion or despawn: the flag joins the existing
overheat reporting.

Consumed ablator mass is tracked in telemetry; live vehicle-mass updates
from recession are future work (same boundary as reactor-fuel mass today).
Jettisoning a spent shield is a staging-topology event and stays future.

### 1.3 Shield aerodynamics (implemented)

Every baked shield disc flies inside the shared aero summation — the same
`AeroGeometry`, the same flow solution, the same `AeroResult` as the wing
and body panels — never through a side channel or a separate
density/velocity input. `AeroGeometry.blunt_discs` carries flat circular
faces; the common model evaluates them with Newtonian impact theory, so a
capsule at angle of attack trims with genuine incidence lift and no
authored lift coefficient. Every local zone follows the shared
`AeroForceBreakdown` contract: drag is parallel and opposite to local flow;
lift is the entire perpendicular force. The shield's geometry-derived
Newtonian resultant is decomposed into those components, then summed with the
panel contributions into `AeroResult.force_body_n`:

```text
q_i = 1/2·ρ·|v_i|²,  c = max(0, n·v̂_i),  v_i includes ω×r like panels
F = −2·q_i·A·c²·n,  M = r×F,  A = π·(d/2)²
```

`HeatShieldMount` retains name, position, normal (from the body end through
the assembly transform), and diameter on the vehicle; the baker maps mounts
into geometry discs. Shield mass stays in the fuselage hull aggregate and
is never re-added. Detailed mode records one `AeroDiscLoad` per disc beside
the per-panel loads, exposing the same force decomposition. The SIMD lane
path uses the same shared summation after the panel kernels. Vacuum/`skip_aero`
silences discs exactly like panels. The panel-only upper-atmosphere reduction
does not apply to shielded geometry; shields retain full shared aero there
until a shield-inclusive drag/lift error envelope is calibrated. Newtonian
impact is hypersonic-oriented; subsonic disc accuracy (blended regime) is
future work.

## 2. Hexagonal tile screens

### 2.1 Authoring

A screen is authored on one named procedural body:

```text
body                  body supplying the loft surface
loop                  closed spline in (x, θ) surface coordinates
tile_size_m           hexagon flat-to-flat width
tile_thickness_m
tile_gap_m            expansion gap between neighbors
tile_material         density, cp, emissivity, absorptivity, max temp
backing_nodes         vehicle thermal nodes receiving tile conduction
```

The `(x, θ)` address space is the loft primitive itself (axial station plus
section angle, cf. `section::outline_point`); no parallel UV system is
introduced.

### 2.2 Fill algorithm (hangar compile, deterministic)

1. Unwrap the body surface into the `(s, θ)` plane, where `s` is axial arc
   length; map the spline loop onto it.
2. Lay a flat-top axial-coordinate hex lattice with pitch
   `tile_size_m + tile_gap_m`.
3. Keep cells whose interior coverage is ≥ 50% (winding test on sampled
   cell corners); drop the rest and report coverage %.
4. Project kept centers back to 3D for position, surface normal, per-tile
   area, and mass.

Guards (fail closed with the offending station/segment named):

- loop must be closed, non-self-intersecting, and inside body bounds;
- tile flat width must be small against local curvature
  (`tile_size_m < 0.2 · R_curv`, Starship-scale rule of thumb made explicit);
- coverage below 90% of the loop area warns (narrow slivers the lattice
  cannot represent).

### 2.3 Runtime model

Tiles are not individual thermal nodes — hundreds of nodes per vehicle is
explicitly rejected. Per body region the tile layer aggregates into one
lumped tile node (mass = Σ kept tiles, area = Σ tile faces) conductively
linked to the backing structure node(s); aero heating applies to tile area
with tile emissivity/absorptivity. Telemetry reports the hottest region and
its margin; tile-granular temperature is debug-viz only (tessellated grid
colored by region temperature).

Per-tile detachment (missing-tile exposure of structure) is modeled as a
future attached/missing set, not in the first slice: all compiled tiles are
assumed attached.

### 2.4 Wing tile-layer toggle (implemented)

Wings and tails do not author tile objects. `ProceduralSurface.tile_layer`
is one toggle: tile size/thickness/gap, density, specific heat, emissivity,
absorptivity, max temperature, and nose radius. The surface compiler paves
both wetted sides from the mounted panels — count floored per side on a
flat-to-flat pitch lattice; face area and mass are summed over the estimated
tile count, so expansion gaps contribute neither tile mass nor tile thermal
area. A layer that cannot fit at least one tile per side fails compilation.
Area-weighted centroid and mean lift reference supply position and normal.
The baker adds the tile mass to the thermal bake plus one lumped tile
node per surface (`{surface}.tiles`: both faces radiate, one face takes
sun/aero through the mean normal; backside solar is future work). The
covered wing keeps flying on its own panels; tiles are conformal coating
with no separate aerodynamics.

## 3. Shared thermal coupling

Both families enter the existing `ThermalSystem` (`details/10`): shields as
a surface balance over a backing node, tile layers as aggregated nodes with
links. Aero heat uses the same `ThermalFlowCondition`; solar uses the shared
occluded stellar inputs. Overheat/burn-through flags join telemetry; nothing
auto-damages.

## 4. Baker and TOML sketch

- Body `heat_shields` gain the §1.1 ablation fields (geometry/mass path
  unchanged).
- New `[[thermal_tile_screens]]` vehicle table (or body-attached equivalent
  if the fuselage authoring owns the loop; the baker owns the decision at
  implementation time) carrying §2.1 fields; the baker runs the §2.2 fill,
  then emits thermal nodes/links plus mass/inertia like any hardware.
- Legacy assets without either table stay valid with bare structure.

## 5. Verification plan (§12)

- Known cases: stagnation recession rate against the §1.2 analytic balance;
  hex fill on a straight cylinder section reproduces the exact lattice count;
  closed energy balance per step.
- Regression: burn-through flag timing on a reference entry pulse; tile
  coverage % on a reference body.
- Error envelope: coverage shortfall vs loop area; recession vs analytic
  within a stated absolute band (W-scale, not relative near zero).
- Benchmark: tile fill on a representative loft; runtime step with a
  representative region count (target: keep the 64-vessel thermal budget).
- Telemetry: remaining ablator thickness, burn-through flag, hottest tile
  region and margin, tile debug tessellation.

## 6. Non-goals

Pyrolysis gas chemistry and blowing corrections, in-depth char conduction
profiles, aerodynamic shape change from recession, tile-by-tile CFD,
transpiration cooling, shield jettison dynamics, and temperature-dependent
strength (still future per `03_PHYSICS_ENGINE.md` §3.11).
