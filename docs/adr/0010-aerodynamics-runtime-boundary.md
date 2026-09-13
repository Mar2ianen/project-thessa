# ADR 0010: reduced-order realtime aero with offline reference solvers

Status: accepted for the first atmospheric physics slice

## Context

The game needs believable aircraft, rockets and lifting bodies at realtime
cost, including transonic and supersonic regimes. Full CFD, VLM and external
6-DoF engines are valuable references, but they are too expensive or have
licenses that must not enter the MIT reusable runtime crate.

## Decision

1. `thessa-sim-core` owns an independent `PanelAeroModel` with SI `f64`, local
   panel velocities, force/moment accumulation, smooth stall handling and
   transonic/supersonic reduced-order corrections.
2. `AeroCoefficientTable` is the import boundary for offline VLM/CFD results.
   Runtime interpolation is deterministic and clamped.
3. JSBSim, AVL, OpenVSP/VSPAERO, SU2, OpenRocket and RocketPy are validation or
   table-generation tools only. They are not root runtime dependencies.
4. Rayon batches independent vehicle cases. Hardware RT is optional for future
   visibility/occlusion and sensor workloads; it is not required by the
   authoritative CPU/server path.
5. Third-party geometry/coefficient data is not vendored until its individual
   license and provenance are recorded.

## Consequences

The first slice is fast, portable and suitable for dedicated servers, but it
does not replace CFD or a full flight-dynamics model. Accuracy is expressed by
regime-specific test vectors and imported tables, not by claiming that an
analytic panel law is universal. Higher fidelity can be added behind the same
force/moment contract.
