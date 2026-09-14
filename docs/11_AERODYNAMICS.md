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
