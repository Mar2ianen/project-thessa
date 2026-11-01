# 109 — Accepted rocket nozzle outlets

Status: implemented read-only F03 producer slice. Accepted liquid/solid/system
observations now carry physical outlet geometry and the already-allocated plume
state. Native/remote consumption, owner/revision binding and multi-nozzle
transport remain open. This change does not alter an image.

## Use the existing physical source

[Doc 101](101_ACCEPTED_PROPULSION_OBSERVATIONS.md) retained the exact operating
points used by resource allocation. A renderer still could not locate every
exit: solid bakes discarded authored nozzle length, and system chamber bakes
discarded both length and contour. Recovering length from asset scale or
fitting it to a desired plume would introduce another source of geometry.

Fresh solid and chamber compilation now retains those authored values.
`EngineMount::nozzle_outlet` and `CompiledChamber::nozzle_outlet` return the
same backend-neutral `NozzleOutlet` contract: exit position in current
COM-relative body metres, exhaust unit axis in body axes and exit radius in
metres. For a circular nozzle with throat station **p**, thrust axis **a** and
authored throat-to-exit length **L**, exit position is **p − L a** and exhaust
axis is **−a**. No asset transform, camera or requested throttle participates.
Current physical mount axes are used; unimplemented gimbal motion is not
invented by this readout.

Nonfinite/invalid geometry is unavailable. Aerospikes remain unavailable because
their effective area/plug data does not define a circular outlet disk. Existing
valid force calculations are not replaced by this geometric readout.

`VehiclePropulsionObservation.nozzles` contains engines in definition order,
then each system's chambers. `RocketNozzleId` identifies either an engine slot
or a system/chamber pair. Each `RocketNozzleObservation` contains optional outlet,
physical gamma, optional `EnginePlumeState` and active interval fraction. It
uses existing `plume_state`/`plume_states` conversions of the exact allocated
points; no second operating-point solve is performed. The original scalar
observations remain available.

Inactive solids retain their identity/known outlet but have no state and zero
active fraction. Liquid/system shutdown retains actual zero flow. A partially
burned solid retains its active-interval midpoint state and fraction, rather
than an averaged or repeatedly throttled flow. The accepted end `SimTime` stamps
the sampled interval, not a promise that fuel is still burning at its endpoint.

This continues the source boundary motivated by the
[KSA audit](104_KSA_AUDIT_AND_RADIAL_SHOCK.md) and
[NASA thrust equation](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/rocket-thrust/):
use actual nozzle geometry and accepted mass flow/exhaust state before optical
presentation. These references do not validate the appearance of this readout.

## Acceptance and moving body frames

The tentative outlet is measured in the precommit body frame. After endpoint
acceptance, FlightAuthority translates it by the same COM shift applied to the
vehicle before publishing the observation. Rejection preserves the entire last
accepted observation. The shared resource-geometry shift also translates a
retained outlet during a manual, same-time fuel transfer. It changes no flow,
temperature, axis, gamma, active fraction or sample time.

Tests check postburn outlet equality against the current physical mount within
`1e-12 m`, require a nonzero COM shift and verify that manual transfer preserves
world outlet position within `1e-3 m` and point velocity within `1e-8 m/s`.
The world tolerance accounts for the fixture's astronomical f64 coordinates;
the tighter body-frame check detects a misplaced small COM correction.
Rotated/translated mount tests use `1e-12 m`/unit-axis absolute tolerances.

## Existing formats and missing geometry

The vehicle baker writes named-field JSON. New compiled solid length and chamber
length/contour fields are `Option` with `serde(default)`: older JSON remains
loadable, with outlet `None`. Tests remove exactly those keys and verify that
legacy versus fresh operating points are identical. No rebake or guessed
length is imposed on existing assets. Present invalid solid length is rejected
by the compiled engine validator; absence remains supported.

Old observation JSON missing `nozzles` decodes to unknown (`None`), while fresh
`Some(empty)` explicitly describes no rocket nozzles. Serde defaults do **not**
promise compatibility with earlier positional binary layouts. Workspace
inspection found no insertion of these types into the current flight-net wire;
protocol v10 is unchanged. Future wire publication must update its explicit
version and compatibility tests.

## Validation and measured cost

[Evidence JSON](artifacts/accepted-nozzle-outlets-2026-11-01.json) records source
and frozen binary hashes, all ABBA batches, check commands and limits. Library
tests passed: propulsion 128, vehicle-core 159, authority 115 and sim-core 152;
three existing tests remain ignored. Strict all-target Clippy passed for those
four crates. Flight-net, server and vehicle-baker all-target compile checks,
the 44-package/20-foundation architecture gate and its 13 tests also passed.

The unchanged release `resource_allocation` fixture uses 32 liquid engines,
64 tanks, a fixed inventory and a 1/120 s interval. Baseline is the retained
doc 101 binary, verified against the current HEAD's numerical/allocator source
and benchmark hash. Both binaries use the bench profile; newer lockfile changes
only add renderer development dependencies. ABBA runs use 64 warmups and five
20,000-allocation batches per process, with every batch included.

On Ryzen AI 9 H 365, median batch means were 3.4941 us baseline and 3.90795 us
with outlets: **+0.41385 us / +11.84%**. Force, moment, tank draws, solid clocks/
ignition, allocated throttles, total flow and limited status match bit-for-bit.
This is added observation cost, not a speedup. Initial baseline batches show
process/cold noise. Batch means are not frame-latency percentiles. Solid and
system paths have regression coverage but are not the timed workload.

## Remaining F03 work

Bind definition-local IDs to owning vehicle and geometry/frame revision;
publish the accepted interval through the versioned remote contract without
relabeling interpolated poses as new physical samples. Consume the actual
atmosphere composition and flow, preserve unavailable legacy/scalar sources,
and implement correct multiple-nozzle optical transport. Partial-tick solid
emission needs interval/history handling. The native provisional engine deck
is unchanged. No new GPU capture, blind preference result, real-engine
calibration or whole-frame performance claim is made here. TAA stays off.
