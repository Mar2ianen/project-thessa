# 101 — Accepted rocket operating observations

Status: the existing installed liquid/solid/multi-chamber allocation now retains
its actual operating points. FlightAuthority publishes the observation only
after accepting the physical tick. Live per-nozzle renderer extraction (F03),
mount/frame publication and remote transport remain open.

## Preserve the physical source

The native plume currently uses a fixed provisional engine deck. A requested
throttle cannot describe fuel starvation, different chamber commands, or a
solid motor that continues burning after its command is released. NASA's
[rocket thrust equation](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/rocket-thrust/)
identifies mass flow, exhaust velocity, exit area and the exit/ambient pressure
difference as physical inputs. Retain the already-computed outputs rather than
running a second propulsion solve in the renderer.

`VehiclePropulsionAllocation.observation` contains a
`VehiclePropulsionObservation`: ambient pressure in Pa, physical interval in
seconds, definition-ordered optional `EngineOperatingSample` slots and
definition-ordered `SystemOperatingPoint` values. Each sample holds the exact
`EngineOperatingPoint` used for force and its active interval fraction. Liquid
off/starved slots retain actual zero-flow points; solids have no sample before
ignition or after burnout. Presence or an active fraction of one does not
imply a positive flow.

For a solid ending inside the tick, the observation retains the existing
active-interval midpoint point and duration fraction. It does not replace the
separately integrated solid propellant ledger with midpoint quadrature.
`CompiledPropulsionSystem::operating_point_and_wrench` returns the same chamber
points used by the unchanged force/moment loop; the existing `wrench_body_n`
API delegates to it. No extra solve, force, resource draw or clock is added.

`FlightAuthority.last_propulsion_observation` is optional and stamped with the
accepted interval's **end SimTime**. It moves the tentative observation after
the endpoint guard and resource/COM commit. Rejected ticks preserve the last
accepted observation. The legacy scalar-thrust path publishes no observation.
The vehicle-core types are also re-exported by sim-core. Existing baked vehicle
and flight-net formats are unchanged; these serializable new types are not yet
inserted into either format.

## Checks and cost (2026-10-31)

Tests cover exact liquid points, resource-limited versus requested throttle,
starved zero flow, all independently commanded system chambers, system
shutdown, solid ignition/command-off/partial-tick burnout, accepted publication
and rejection before/after an accepted observation. Relevant propulsion,
vehicle-core and authority library tests pass; strict relevant all-target
Clippy, native compile and the architecture gate are recorded in the evidence.

The existing release `resource_allocation` benchmark uses the same 32 engines,
64 tanks, 1/120 s interval and fixed initial inventory in both variants.
Retained binaries run ABBA, with 64 warmup allocations and five 20,000-allocation
batches per process. Source and binary hashes are retained. Force, moment,
tank draws, solid clocks/ignition, allocated throttles, total flow and
fuel-limited status match **bit-for-bit** for this fixture.

On the Ryzen AI 9 H 365, median batch means are approximately 3.306 us before
and 3.441 us after: +0.135 us / +4.08%. This is the cost of retaining the
observations, not a speedup. First batches show cold/process noise; every batch
is included in the reported median. These are allocation batch means, not
per-frame latency percentiles or a whole-client budget. The system/solid
branches have regression coverage but are not the timed benchmark workload.

Evidence: [machine-readable record](artifacts/accepted-propulsion-observations-2026-10-31.json).
Raw logs/binaries remain under the external `engine-boundaries-2026-10-29/`
`nozzle-observation` output directory.

## Remaining F03 boundary

This change alone does not alter the image. No GPU capture is needed to claim
an unchanged renderer; the actual Cycles volume controls remain in
[doc 100](100_PLUME_SEGMENT_TRANSPORT.md). Their transport agreement does not
calibrate a real engine. Before native consumption, bind observations to
definition/frame revisions and actual nozzle outlets across COM changes and
separation, publish them through the checked remote contract, handle multiple
nozzles, and query the actual atmosphere composition/flow. Multi-chamber
compiled geometry currently omits authored nozzle length; do not invent its
exit position. Airbreathing/APU/electric/fusion observations are outside this
rocket-only slice. The compatibility X-15's scalar thrust has no compiled
nozzle, so missing physical telemetry must remain explicit.

Already-throttled `EnginePlumeState` flow must not be multiplied by the pilot's
requested throttle again. Solid active fractions are interval observations,
not permission to change the authoritative burn clock. No renderer may feed
these observations back into force, tank allocation or time.
