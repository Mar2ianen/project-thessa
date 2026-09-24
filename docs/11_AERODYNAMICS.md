# 11 — Aerodynamics and validation

## Status

**Implemented reduced-order runtime model.** The model is in
`crates/sim-core/src/aero.rs`, with atmosphere coupling, vehicle baking,
control surfaces, and validation harnesses. It is not CFD and does not claim
exact shock/separation/aeroelastic behavior.

## 11.1. Runtime boundary

The server-side canonical path is CPU `f64` and works without a GPU. Rayon and
SIMD may accelerate ordered batches, but hardware ray tracing is not required
for authoritative forces. Reference solvers are isolated offline tools.

```text
VehicleDefinition
       ↓
local aero panels / coefficient tables
       ↓
AtmosphereConfig + local flow
       ↓
aggregate body force and moment
       ↓
flight authority / rigid-body integrator
```

## 11.2. Local flow contract

For each panel:

```text
v_local = v_vehicle - v_wind + omega × r_panel
q       = 0.5 * rho * |v_local|²
Re      = rho * |v_local| * chord / dynamic_viscosity
```

The panel’s local AoA and sideslip use body axes and its own lift/drag axes.
Forces are summed in body frame. Moment is `r × F` around the center of mass;
static local pitching moment is kept separate from the center-of-pressure
moment.

The sign convention is consistent across runtime and tables: positive alpha
means the nose is above the incoming flow in body coordinates. An inverted
stabilizer uses an explicit lift sign rather than changing the global AoA
convention.

## 11.3. Implemented Tier A model

The analytic panel model includes:

- local AoA and sideslip;
- `omega × r` and rotating-atmosphere flow;
- finite planform and effective aspect ratio;
- subsonic lift slope with bounded compressibility;
- transonic blending near `M≈0.8–1.2`;
- finite near-sonic supersonic lift branch;
- induced drag;
- transonic wave-drag rise;
- linearized supersonic wave drag;
- thickness wave drag at zero lift;
- swept-surface normal Mach;
- bounded continuous post-stall curve;
- control-surface effectiveness;
- static `Cm` and dynamic roll/pitch/yaw damping;
- optional panel exposure input and imported coefficient tables.

The coefficient-table path uses monotonic Mach/AoA grids and bilinear
interpolation, clamping outside the exported domain. The table’s reference
area, length, Reynolds/atmosphere range, body axes, and control deflection
range remain part of its provenance.

## 11.4. Atmosphere coupling

The default atmosphere is a deterministic ISA-like layered provider returning
temperature, pressure, density, viscosity, and speed of sound. The configured
body atmosphere feeds both dynamic pressure and Mach/Reynolds values. It also
supports atmospheric rotation and a declared vacuum cutoff.

This is not a final planetary composition or weather model. Body-specific gas
constants, winds, weather, and heating correlations remain future inputs.

## 11.5. Reduction and upper atmosphere

Upper-band reference-area drag and zero aero moment are allowed only under the
explicit configured density/regime boundary where the error envelope is
decision-irrelevant and RCS dominates. The declared vacuum path is exact zero
air load. Each reduction requires a calibration, absolute envelope regression,
and benchmark.

## 11.6. Control surfaces and actuators

Control surfaces have geometry, hinge axis/limits, response rate, and actuator
torque/authority. A command is not an instantaneous deflection: aerodynamic
load may leave the surface short of its requested position. The flight-control
allocator receives a desired force/moment and reports saturation/residuals.

This boundary keeps direct/manual control possible and prevents FBW from
injecting an artificial craft moment.

## 11.7. Reference matrix

Compare only matching reference area/length, body axes, atmosphere, Mach,
Reynolds, mass properties, and sign conventions.

| Reference | Use |
| --- | --- |
| JSBSim | nonlinear aircraft/rocket 6-DoF and coefficient functions/tables |
| AVL | low-angle thin lifting-surface and stability baselines |
| OpenVSP/VSPAERO | VLM/panel geometry and offline coefficient tables |
| SU2 | expensive compressible/transonic/supersonic spot checks |
| OpenRocket | model-rocket trajectory and staging cross-check |
| RocketPy | rocket coefficient/table and atmosphere cross-check |
| NASA CRM | common aircraft geometry validation |
| NASA Shuttle Aerodynamic Data Book | lifting-body operational range sanity |

Reference code and data must remain outside the MIT runtime boundary unless a
license review says otherwise. See [`REFERENCES.md`](REFERENCES.md).

## 11.8. Current validation vectors

`validation/aero-compare` covers:

1. a low-angle finite-wing proxy;
2. an axial rocket-like proxy near `M=0.95`;
3. a lifting-body/shuttle-like proxy near `M≈5.3`, `AoA=20°`;
4. optional JSBSim/RocketPy comparisons when local packages are installed.

## 11.8.1. Delta wings and vortex lift

The Concorde ogive fixture (`aero-surfaces`, aspect ratio 1.813, effective
leading-edge sweep near 60 deg) carries a calibrated Polhamus vortex factor
(`concorde::VORTEX_LIFT_FACTOR = 3.0`, NASA TN D-3767 `Kv` for ~60 deg
sharp-edge sweep). Anchors: AVL 3.36 VLM on the same 7-station planform gives
a 1.98/rad attached slope against the solver's 2.04/rad (+3.0%); with the
vortex term the solver reproduces the `Kp = 1.98 / Kv = 3.0` polar within
~2.5% at 5–15 deg AoA. The term is second-order at small angles, so the
linear slope is untouched. Pinned by
`concorde_delta_vortex_lift_matches_polhamus_band`. Vortex-induced drag is
not modeled yet (induced drag follows the attached branch only), so no
cruise L/D claim is made for deltas.

## 11.8.2. Body aerodynamics (fuselage strips)

Fuselage bodies compile to two strip panels per axial zone (pitch plus
yaw plane) with Munk/slender-body interference: each zone carries the
potential-flow normal force of its signed section-area gradient
(`2 * |dA|`, with lift direction following the gradient sign) through a
geometry-derived interference value. A pointed forebody recovers the
`2 * S_base / S_ref` slope with zero per-vehicle tuning, a constant
barrel correctly carries almost none, and a boat-tail retains its
opposite-sign contribution. Centers of pressure follow the first moment
of area change, giving real CP travel (and the genuine nose-forward
destabilization that makes rocket fins necessary). Cross-sections and
volume centroids are integrated along the actual interpolated loft.
Axial blunt/base/wave drag is reported as bookkeeping
(wetted area, base area, fineness) for Tier B table calibration, not
wired to runtime `Cd0`.

Body strips set `side_force_scale = 0`: their lateral answer already
arrives through the lift path of the orthogonal strips, and the shared
sideslip convention would otherwise double-count pitch-plane crossflow
as sideslip on yaw-normal panels. The scale defaults to `1.0`, so every
legacy panel is bitwise identical.

Procedural bodies may assign normalized control channels to axial ranges
of their pitch or yaw strip panels. Range edges split the body zone schedule,
so a command changes only the selected generated panels; the vehicle baker
rebases those indices into the same runtime control list used by wings.
Each compiled region has a geometry-derived hinge at its leading axial edge
and section centroid. The runtime applies the deflection as a rigid rotation
of selected panel positions, centers of pressure, and axes; it does not add a
direct vehicle moment or also apply the legacy coefficient deflection.

An authored body-control actuator may specify its no-load angular rate and
stall torque. Detailed panel forces and moments determine the signed
aerodynamic hinge torque, and a linear torque-speed envelope reduces actuator
rate against opposing load. At rated stall torque the commanded actuator
stalls; aerodynamic control effectiveness itself remains untouched. Missing
actuator data preserves the shared command-response path (including the
flight-authority normalized command slew) for older assets. In vacuum the
actuator advances at its rated no-load rate without an aerodynamic solve.
Hinge motion currently affects aero-panel geometry only; moved-surface
mass/inertia and contact geometry, actuator mass, moving render meshes,
electrical/hydraulic power, thermal inhibition, and structural hinge failure
remain outside this slice.

## 11.8.3. Lifting bodies

Lifting-body fuselages (Dream Chaser class) compile through the same
strip pipeline with two additions. First, asymmetric sections shift
outline centroids toward the fuller half, so volumes, mass, and contact
geometry stay camber-honest. Second, strip chord axes tilt with the
local centerline slope while lift axes re-orthogonalize in the
constructor: a drooped nose therefore carries genuine nose-down camber
physics (negative lift at zero alpha, pinned by regression) with no new
coefficient. Chine vortex lift itself stays a per-vehicle config factor
calibrated the Concorde way (imported polars or tunnel/CFD), never an
invented constant: the Dream Chaser fixture pins the Munk-class slope
(within 0.3 of `2 * S_base / S_ref`, about 1.7 per maximum frontal area
for this notional geometry) and the camber shift, not a vortex number.

The repository also contains a bundled X-15-like proxy comparison. Its Mach
and altitude errors are useful regression measurements, not a statement that
the compact proxy reproduces the full X-15 model.

## 11.9. Performance and limits

The target batch sizes are 1, 16, 256, and 1024 vehicles, with 8–64 aggregated
panels for ordinary craft. One small craft should not be split into many tasks
when parallel overhead dominates.

The current model does not provide exact shock location, separation bubbles,
hypersonic chemistry, boundary-layer transition, aeroelastic coupling,
complete wake/occlusion, or arbitrary Reynolds/control-table axes. Those need
offline reference evidence and a new documented contract.

Run:

```bash
cargo test -p thessa-sim-core
cargo bench -p thessa-sim-core --bench aero
cargo bench -p thessa-sim-core --bench aero_panels
cargo run --manifest-path validation/aero-compare/Cargo.toml --release
```
