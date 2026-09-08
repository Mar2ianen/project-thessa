# Thessa aerodynamic comparison harness

This is an isolated validation workspace. It depends on the MIT
`thessa-sim-core` crate, but it is not part of the root runtime workspace and
does not add JSBSim, SU2, OpenVSP, AVL, OpenRocket or RocketPy to the game
dependency graph.

Run the owned regression vectors with:

```bash
env RUSTC=/usr/bin/rustc PATH=/usr/bin:/bin \
  THESSA_AERO_PYTHON=.venv-aero/bin/python \
  /usr/bin/cargo run --manifest-path validation/aero-compare/Cargo.toml --release
```

The recommended persistent local environment is `.venv-aero` at the repository
root. It is ignored by Git and can be recreated with:

```bash
python -m venv .venv-aero
.venv-aero/bin/pip install jsbsim rocketpy
```

The executable probes optional local installations of JSBSim, VSPAERO, AVL,
OpenRocket, SU2 and the Python packages JSBSim/RocketPy. It does not pretend
that a missing executable or package is a comparison result.
`--require-external` turns the absence of all optional tools into a failure,
which is useful on a validation workstation or CI image that provides the
external solvers.

Every run also executes owned X-15 wing-proxy boundary vectors at
`M=0.8, 1.0, 2.0, 6.0` and a bounded performance smoke. The smoke fails on a
non-finite aero/trajectory/reference value or when the evaluation loop exceeds
`THESSA_AERO_MAX_SMOKE_MS` (default 2000 ms); set that variable explicitly for
slower CI hardware rather than disabling the check.

When JSBSim and RocketPy are available, the harness also runs real adapters:

- JSBSim bundled `737`, `X15` and `Shuttle` models are sampled at the same
  Mach/AoA points as the Thessa proxy cases. It reports normalized `CL`, `CD`
  and `Cm` plus absolute and percentage errors.
- A short 5-second, 100-Hz X-15-like rigid-body run is compared with the
  bundled JSBSim X15 by final Mach, altitude and AoA. This is deliberately
  printed as a proxy-gap diagnostic: the owned model has simplified wing/tail
  geometry and no JSBSim trim/control tables.
- The X-15 path additionally runs a supersonic sweep at
  `M=0.95, 1.1, 1.2, 1.5, 2.0, 3.0, 5.0` for a finite-wing proxy using the
  bundled model's wing area, span and chord. The missing fuselage/tail and
  JSBSim lookup tables are reported as proxy gap, not hidden in a pass score.
- RocketPy's Barrowman trapezoidal-fin calculation is sampled at
  `M=0.3, 0.8, 0.95, 1.1, 1.2, 2.0, 5.0` and compared with the same four-fin
  panel geometry's centered `CL_alpha` and center-of-pressure estimate.

The JSBSim rows are explicitly proxy-gap measurements: the bundled aircraft
geometry is not identical to the compact Thessa proxy geometry. They quantify
the current reduced-order model gap and are not a certification result. The
RocketPy fin row uses matching fin dimensions and is the first apples-to-apples
coefficient check.

The three first vectors are intentionally compact proxies, not copied model
files:

- a low-angle finite lifting surface for subsonic polar checks;
- a transonic rocket-like axial case for drag-rise continuity;
- a hypersonic shuttle-like lifting-body proxy for finite-force and moment
  checks.

Before accepting a coefficient table for gameplay, run the same geometry and
reference conditions through the named external solver and record the source,
mesh, turbulence/viscosity settings, reference area and coordinate convention
alongside the exported table. The runtime must consume the table, not link the
external solver.
