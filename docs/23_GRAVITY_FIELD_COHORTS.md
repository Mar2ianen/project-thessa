# 23 — Baked gravity hierarchy and target-cohort field cache

Status: partially implemented — monopole `GravitySourceTree`, single-tick
affine `GravityPatch` / `CohortEvaluator` with Hessian spatial bound landed;
time-span patches, quadrupole, cohort keys, planner-patch reuse, and GPU
backends remain design targets. Struct sketches below are the target API and
do not all compile against the current single-tick types.

Status: **architecture / performance design target**.

This doc defines the next major gravity optimization layer on top of the already existing baked ephemerides, adaptive integration and SIMD-oriented ephemeris tables.

The main idea: the runtime should not answer the question

> "what is the total gravity of all bodies at this point right now?"

from scratch for every vehicle and every RK stage when many objects are in one region of space and time, and distant sources form a smooth field.

Instead, two independent and compatible forms of sharing are needed:

```text
source-side sharing
    baked astronomical hierarchy / multipoles

           +

target-side sharing
    local field patch per spatial-temporal cohort
```

The first reduces the number of sources that need to be opened. The second lets hundreds of nearby targets reuse an already compiled local field.

This is **not SOI**, not patched conics, and not "cargo ships get cheap physics". All physical sources continue to exist; approximation is allowed only under bounded error.

---

## 1. Current baseline and missing sharing

The current `GravityField::acceleration(position, time)` walks over all gravity sources, obtains `body_state`, then for each source computes distance, reciprocal distance cubed and sums the point-mass acceleration.

`EphemerisTable` already solves an important part of the cost:

- samples source motion once per horizon;
- stores component-major position/velocity arrays;
- interpolates source states instead of repeated Kepler solves;
- has a SIMD-friendly layout;
- snapshots all body centers at the chosen epoch.

But even after that, each target separately repeats source accumulation.

That is, the current conceptual hot path remains close to:

```text
for target in targets:
    for source in gravity_sources:
        evaluate source center
        dx = source - target
        r2 = dot(dx, dx)
        target.g += mu * dx / r^3
```

Adaptive tick integration reduces the number of accepted time steps, but each remaining RK force evaluation still pays for the source set.

Experimental `gravity_interpolation` already explores temporal force reuse. The new design does not replace that work; it makes approximation source-aware and target-shareable.

---

## 2. Design goals

Main goals:

- materially reduce gravity cost for fleet-scale simulation;
- make `100–300` nearby freighters much cheaper than `100–300 × one craft`;
- keep an exact/authoritative fallback;
- use the same bounded approximation semantics in simulation, trajectory planning and autopilot candidate search;
- get a strong CPU-only path first;
- leave GPU/other accelerators as an optional backend for later;
- do not tie correctness to vehicle class, CPU/GPU vendor, or observer state.

Target gameplay workload:

```text
100–300 freighters
    wait for similar launch windows
    depart from same depot/moon
    fly in a relatively compact group
    share destination and much of the trajectory geometry
```

This is an almost ideal target-side sharing case.

---

## 3. Source hierarchy is part of celestial representation

In Thessa the astronomical hierarchy is known in advance and stable in topology. Therefore a generic Barnes–Hut tree rebuilt every tick is not needed.

Example logical tree:

```text
Asterion system root
├─ A branch
│  ├─ Asterion A
│  ├─ Khepri
│  ├─ Nereid system
│  │  ├─ Nereid
│  │  └─ moons ...
│  ├─ Orthea system
│  └─ Vesper system
│
└─ BC branch
   ├─ Asterion B
   ├─ Asterion C
   └─ Janus system
```

An inner node is not a fake body. It is an aggregate representation of physically existing children.

Candidate node data:

```rust
pub struct GravitySourceNode {
    pub mu: f64,
    pub child_range: Option<ChildRange>,

    // baked/time-varying tracks
    pub barycenter_track: TrackId,
    pub radius_bound_track: TrackId,
    pub quadrupole_track: Option<TrackId>,

    // conservative metadata for opening/error tests
    pub max_internal_radius_m: f64,
    pub max_quadrupole_norm: f64,
}
```

The exact storage form is determined by the baker/runtime representation.

---

## 4. Bake the hierarchy because the source trajectories are already baked

Since canonical celestial trajectories are already deterministic/baked, the source hierarchy can also be prepared offline.

For each internal node the following can be obtained in advance as functions of `SimTime`:

```text
mu_total
barycenter(t)
internal radius bound(t)
quadrupole(t)
optional higher conservative bounds
```

For the B+C pair this means the runtime is not obliged to first compute B(t), C(t) and then reassemble the aggregate from scratch every time.

These tracks can be stored through the same class of representations that is applied to ephemerides:

- Hermite segments;
- Chebyshev segments;
- uniform sampled tracks where cheaper;
- bounded interpolation error metadata.

Source-tree topology must be versioned together with the system ephemeris/content version.

---

## 5. Far-field representation

For a sufficiently distant source node the first approximation is a monopole at the barycenter:

\[
\mathbf g(\mathbf x)=\mu\frac{\mathbf r}{|\mathbf r|^3}.
\]

If the monopole error budget is already insufficient, the next natural correction is the quadrupole.

The barycentric expansion is especially convenient because the dipole term relative to the barycenter vanishes.

Conceptual fidelity ladder:

```text
far
    monopole

closer
    monopole + quadrupole

nearer
    descend to children

very near
    exact body/source terms
```

The opening decision must not be hardcoded as `s/r < theta` only. It is preferable to use a conservative acceleration-error estimate:

```text
if estimated_node_error <= allocated_gravity_error_budget:
    accept aggregate node
else:
    descend
```

The classic geometric ratio can be a cheap prefilter, but the final policy must be tied to the physical/numerical error budget.

---

## 6. Target cohorts: share a local field, not one acceleration vector

Two nearby vehicles do not have strictly identical acceleration. Therefore one cannot simply compute `g(center)` and give it to everyone.

But the far field is locally smooth. For a target cohort around anchor `x0`:

\[
\mathbf g(\mathbf x)
\approx
\mathbf g_0
+J(\mathbf x-\mathbf x_0)
\]

where `J` is the gravity gradient / tidal tensor.

For a point-mass source:

\[
J = \mu\left(\frac{3\mathbf r\mathbf r^T}{r^5}-\frac{I}{r^3}\right).
\]

If needed, the next level:

\[
\mathbf g(\mathbf x)
\approx
\mathbf g_0
+J\Delta\mathbf x
+\frac12H[\Delta\mathbf x,\Delta\mathbf x].
\]

That is, one expensive field compilation can serve many targets with a few FMAs per object.

Candidate runtime cache:

```rust
pub struct GravityPatch {
    pub center: DVec3,
    pub radius_m: f64,
    pub start: SimTime,
    pub end: SimTime,

    pub g0: DVec3,
    pub jacobian: DMat3,
    pub hessian: Option<GravityHessian>,

    // sources that cannot be safely absorbed into the shared approximation
    pub exact_sources: SmallVec<[BodyId; 4]>,

    // already accepted aggregate source nodes; avoids retraversal while valid
    pub accepted_nodes: Vec<GravityNodeId>,

    pub accel_error_bound: f64,
}
```

Exact type/layout is provisional.

---

## 7. Split common far field from individual near corrections

Most useful fleet representation is likely:

\[
\mathbf g_i=
\mathbf g_{far,patch}(\mathbf x_i,t)
+\sum_{j\in near}\mathbf g_j(\mathbf x_i,t).
\]

Example near Nereid:

```text
shared patch:
    Asterion A
    BC aggregate
    far planets/moon groups

per-craft exact terms:
    Nereid
    current nearby moon(s)
```

This keeps local encounter fidelity without forcing every far source through every target's exact loop.

If the convoy approaches another body, the source tree naturally opens and the exact-source list changes.

---

## 8. Cohort construction

A cohort is not "all cargo ships".

Class/role may define a requested error budget or scheduling policy, but spatial/temporal validity defines actual sharing.

Useful cohort dimensions:

```text
spatial region
simulation time span / integration lattice span
requested gravity error budget
integration regime / step schedule
```

Possible key direction:

```rust
pub struct GravityCohortKey {
    pub time_span: TickSpan,
    pub spatial_cell: GravityCellId,
    pub accuracy_class: GravityAccuracyClass,
}
```

But implementation should avoid locking into a grid if bounding spheres/clusters work better.

A convoy can start as one cohort:

```text
compute center + radius
compile patch
```

If error bound fails because the group stretches or enters a high-gradient region:

```text
cohort
  -> split spatially
  -> rebuild/refresh child patches
```

If neighboring cohorts later converge and share compatible time/error ranges, merge is allowed but not required for first implementation.

---

## 9. Error-driven validity, not vehicle-class cheats

A patch is valid while both spatial and temporal remainder bounds stay within budget.

Conceptually:

```text
spatial error
    higher-order field terms over cohort radius

+

temporal error
    source motion / multipole evolution over patch interval

<= allocated gravity error budget
```

The system may have named accuracy classes for ergonomics, but they are requests, not guarantees of approximation acceptance.

Example:

```text
active landing
    tiny budget

normal free flight
    medium budget

on-rails / long coast
    larger budget

planner broad search
    intentionally loose first pass
```

An on-rails freighter passing close to Nereid must automatically refine/split/open sources if its requested bound cannot be satisfied.

---

## 10. Interaction with adaptive timestep

Temporal integration and field representation are separate adaptive axes:

```text
time adaptation
    how often target state is advanced

source adaptation
    how deeply source hierarchy is opened

target adaptation
    how large a target cohort can share one local field patch

cache adaptation
    how long that patch remains valid
```

Large integration steps do not automatically permit sloppy gravity, but the integrator can explicitly allocate part of its local error budget to field approximation.

A useful future contract is something like:

```text
integrator local error budget
    -> reserve state-integration component
    -> reserve field-approximation component
```

The gravity evaluator then returns both acceleration and a conservative approximation bound.

This avoids duplicated hidden tolerances.

---

## 11. CPU-only first implementation

First serious version is CPU-only.

Reasons:

- authoritative server is already CPU-first;
- target batch is initially `~100–300`, not millions;
- shared patch collapses expensive math into a tiny polynomial evaluation;
- CPU SIMD avoids device synchronization/data-residency complexity;
- profiling should prove that GPU is still needed after representation improvement.

Preferred target layout for cohort evaluation:

```text
positions SoA:
    x[]
    y[]
    z[]

shared:
    center
    g0
    J
    optional H

output SoA:
    ax[]
    ay[]
    az[]
```

This maps cleanly to AVX2/AVX-512 lanes.

Steady-state target:

- no allocation in hot evaluation loop;
- no per-target source-tree traversal while patch is valid;
- no per-target ephemeris interpolation for accepted far nodes;
- exact-near source loop remains short and SIMD/batch friendly where useful.

Cache padding/align tricks only after measured false sharing, consistent with project-wide performance policy.

---

## 12. Autopilot and trajectory planner reuse

The same representation can accelerate trajectory search even more aggressively than authoritative flight.

Typical planner workload:

```text
same start epoch
same start region
same celestial configuration
hundreds/thousands of candidate burns
```

Instead of compiling gravity independently for every candidate:

```text
build GravityPatch(time interval, region, error budget)

candidate 1 ┐
candidate 2 ├─ reuse patch
candidate 3 ┤
...         │
candidate N ┘
```

Recommended staged search:

```text
phase 1
    huge candidate set
    loose bounded field approximation

phase 2
    survivors
    tighter patch / smaller cohorts

phase 3
    final handful
    authoritative/exact revalidation
```

This is acceptable because pruning approximation does not define final physical truth. Chosen trajectories are revalidated against the authoritative gravity path before execution/commit.

---

## 13. Fleet-level planning sharing

For convoys following essentially one route, planning itself can be shared:

```text
FleetPlan
    reference trajectory
    departure/launch window
    common source hierarchy decisions
    common GravityPatch sequence
    shared predicted events

VehicleFollower
    local offset
    collision/deconfliction
    actuator/propellant state
    correction burn
```

This does not force formation flight. It only avoids solving the same long-horizon orbital problem hundreds of times when differences are small.

Individual craft remain authoritative physical objects.

---

## 14. Backend evolution

Architecture should allow multiple evaluator backends later, but backend proliferation is explicitly deferred.

Initial:

```text
CpuExact
CpuSimdPatch
```

Possible later measured paths:

```text
GpuPatchBatch
GpuTrajectoryBatch
```

A future GPU backend becomes interesting if:

- target count is large enough;
- positions/state already reside on GPU for multiple propagation steps;
- transfer/synchronization cost is amortized;
- entire candidate-trajectory batches can remain device-side.

NPU/tensor/matrix hardware is not a baseline requirement. Tiny `3x3` affine evaluation is a poor match. If future quadratic/high-order batched representations become GEMM-shaped at very large N, they can be benchmarked separately.

Backend choice must remain capability/workload driven, not vendor driven.

---

## 15. Baked source hierarchy plus runtime target hierarchy

The useful asymmetry is:

```text
sources
    small count
    known astronomical topology
    deterministic baked trajectories
    fixed/versioned source tree

vs

targets
    potentially thousands
    dynamic positions
    dynamic grouping
    runtime cohort split/merge
```

So Thessa does not need a fully generic N-body FMM implementation.

A specialized architecture can be significantly simpler and faster:

```text
baked source tree
    + runtime source opening
    + runtime target cohorts
    + local polynomial patches
```

This is FMM-like in spirit without paying for generality that the game does not need.

---

## 16. Benchmark plan

Microbenchmarks alone are insufficient.

### 16.1 Primitive costs

Measure:

```text
exact point-mass source contribution
source-tree traversal
monopole aggregate
quadrupole aggregate
patch compile
patch affine eval scalar
patch affine eval AVX2
patch affine eval AVX-512 where available
quadratic eval if implemented
```

### 16.2 Fleet workloads

At minimum:

```text
1 target
16 targets
64 targets
128 targets
300 targets
1000 targets
```

Scenarios:

1. compact convoy in deep space;
2. convoy near Nereid with 1–3 exact-near sources;
3. convoy stretches until cohort split;
4. group crosses from A-side far-field BC aggregate toward Janus/BC and tree opens;
5. mixed active + rails targets in same region;
6. multiple independent convoys in different regions;
7. long warp/coast with large integration steps;
8. planner batch with 1k/10k+ candidates.

Metrics:

```text
ns / target force evaluation
ns / patch compile
source terms evaluated / target
source-tree nodes visited / patch
patch reuse count
cohort split count
exact-near terms / target
field approximation max accel error
trajectory position/velocity error
end-to-end simulated seconds / wall second
1/2/4/8/16 thread scaling
L1/L2/LLC misses
bytes touched / target
allocations
```

Main KPI is end-to-end fleet propagation throughput at pinned physical error, not maximum isolated FMA throughput.

---

## 17. Correctness and validation

Every approximation path must be checked against exact source accumulation.

Required tests:

- source aggregate monopole/quadrupole vs explicit children across representative geometry;
- baked aggregate tracks vs runtime aggregate recomputation;
- patch field vs exact field over random points inside validity volume;
- temporal validity over full patch interval;
- cohort splitting before declared error bound is exceeded;
- trajectory propagation exact vs patch-based over reference scenarios;
- final planner candidate revalidation;
- deterministic replay independent of Rayon worker count.

Error should be measured in absolute decision-relevant units:

```text
m/s² acceleration error
m position divergence
m/s velocity divergence
```

not only relative percentage near zero.

---

## 18. Non-goals

This work must not:

- introduce SOI switching as physical law;
- remove gravity sources because they are inconvenient;
- make `cargo` or `debris` an excuse for unbounded fake physics;
- require GPU on authoritative server;
- rebuild a generic Barnes–Hut source tree every tick when topology is known;
- bake a gigantic 4D `g(x,y,z,t)` grid for the whole system;
- hide approximation error inside unexplained magic tolerances;
- couple correctness to current vehicle class names;
- force all targets in one region to use one timestep or one integration regime;
- skip final exact revalidation for planner/autopilot outputs when execution correctness matters.

---

## 19. Suggested implementation order

1. Add an exact benchmark that reports source-accumulation cost separately from ephemeris lookup/interpolation.
2. Introduce logical baked source-tree metadata without changing `GravityField` results.
3. Implement monopole aggregate evaluation and differential tests against explicit children.
4. Add conservative opening/error criterion.
5. Add optional quadrupole representation only if measurements justify it.
6. Implement single `GravityPatch` affine field around a fixed anchor and validate over radius/time bounds.
7. SIMD batch-evaluate one patch over SoA target arrays.
8. Split far shared patch from short exact-near source list.
9. Add runtime target cohort construction + split on failed bound.
10. Reuse patch/tree decisions across repeated integration evaluations.
11. Integrate with rails/fleet workloads and benchmark `100–300` freighters.
12. Reuse the same patch compiler in autopilot/trajectory candidate search.
13. Only after CPU path is measured, decide whether a GPU batch backend has a real end-to-end win.

---

## 20. Acceptance criteria

First serious milestone is accepted when:

- exact path remains available and unchanged in physical definition;
- baked source hierarchy is deterministic/versioned;
- source aggregates have conservative measured error bounds;
- a compact fleet can share one field patch without per-target source-tree traversal;
- patch invalidation/splitting happens before its error contract is violated;
- near-body encounters automatically refine rather than using class-based hacks;
- authoritative server remains CPU-only capable;
- `100–300`-target fleet benchmark shows a material end-to-end speedup over independent exact accumulation at matched trajectory error;
- autopilot/planner can reuse the same bounded representation and exact-revalidate finalists;
- performance counters show where the remaining cost lives before adding GPU/NPU/tensor complexity.

Key principle:

```text
Do not solve the same slowly-varying gravity problem hundreds of times.

Compile the field once where possible,
prove its validity envelope,
and spend exact work only where the error budget demands it.
```
