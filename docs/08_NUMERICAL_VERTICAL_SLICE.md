# 08 — Numerical vertical slice

## Status

**Implemented prototype, 2026-09-14.** `data/system.toml` remains a design
target rather than canonical celestial data.

## 8.1. Implemented paths

- `thessa-sim-core` is MIT and independent of Bevy, Tokio, and DirectX;
- `SimTime`, frame-labelled `f64` vectors/quaternions, and SI units are used in
  authoritative state;
- `KeplerOrbit` evaluates deterministic elliptic segments without wall-clock
  state;
- `BakedEphemeris` builds the A–BC, binary, planet, moon, and minor-body design
  hierarchy;
- `GravityField` sums physical point-mass sources simultaneously;
- ordered batch evaluation preserves input order (direct path Rayon-parallel,
  `EphemerisFrame` path deliberately serial);
- adaptive Dormand–Prince 5(4) with FSAL plus 8(5,3), variational sensitivity,
  velocity-Verlet, sampled coast, single-tick gravity cohorts, monopole source
  tree, `EphemerisFrame`, thrust arcs, and piecewise affine propagation
  are available;
- atmosphere, panel aero, rigid-body flight, actuator response, and contacts
  are connected through the authority layer;
- `thessa-system-baker` validates TOML and writes format-1 JSON output.

## 8.2. Formal test-particle contract

For position `x`, velocity `v`, and simulation time `t`:

```text
dx/dt = v
dv/dt = sum_i mu_i * (body_i(t).position - x) / |body_i(t).position - x|^3
```

The body state comes from baked analytic segments. There is no SOI switch.
Ship-to-ship gravity, J2/Jn harmonics, collisions, thrust, and thermal
coupling are separate extensions rather than hidden terms in this contract.

The atmosphere/flight slice adds `PanelAeroModel`, `AtmosphereConfig`,
`evaluate_flight_forces`, and deterministic rigid-body duration integration.

## 8.3. Verification coverage

`sim-core` tests cover:

1. circular and eccentric two-body orbits;
2. bounded energy error for velocity-Verlet;
3. restricted three-body/Lagrange residuals;
4. moving-secondary energy exchange;
5. ordered exact replay and parallel batches;
6. frame labels and deterministic impulse schedules;
7. cohort patch bounds, splits, exact-near terms, and fallback;
8. affine state-transition propagation and piecewise revalidation;
9. atmosphere, rotating air, aero signs, dynamic damping, stall, and
   transonic/supersonic finite behavior;
10. rigid-body, actuator, contact, and on-rails cache behavior.

Run:

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets --all-features -- -D warnings
cargo test --workspace
cargo test -p thessa-sim-core --release
cargo bench -p thessa-sim-core --bench gravity
cargo bench -p thessa-sim-core --bench affine_prop
```

## 8.4. External reference harness

`validation/nyx-compare` is a separate workspace using Nyx/ANISE only for
diagnostics. It compares common Cartesian states against our independent
Dormand–Prince and Kepler paths. `validation/aero-compare` performs analogous
reference workflows for aero proxy vectors and optional local JSBSim/RocketPy.

Reference results are evidence for selected vectors, not a global correctness
claim. ANISE paths that do not match the current reference vector remain
diagnostic until their epoch/frame/version assumptions are isolated.

## 8.5. Current limitations

- analytic design ephemerides are deterministic but not a long-term physical
  canon;
- embedded local error control is not a global error guarantee;
- velocity-Verlet is second-order in a time-dependent moving-source field;
- identical binary/source order provides replay stability, not cross-ISA bit
  identity;
- J2/Jn, full terrain contact, thermal,
  structural, and factory systems are not complete. Hyperbolic/parabolic
  osculating-element readout has landed (`sim-core` ephemeris); fitted
  hyperbolic/parabolic baker segments are still open.

## 8.6. Next numerical work

1. versioned fitted ephemeris segments and hyperbolic/parabolic support;
2. body-fixed harmonics and precession reference vectors;
3. tighter planner/authority integration for finite burns (partially landed:
   thrust arcs + variational search with TCM cache; joint multi-leg shooting
   still open);
4. wider fleet benchmarks and error envelopes;
5. structural/thermal state contracts after the flight kernel boundary is
   stable.
