# 42 — Bounded ephemeris residual storage for gravity evaluation

Status: **design / prototype target**.

This document proposes a physics-side analogue of
[`docs/41_MICROSCALED_SURFACE_STORAGE.md`](41_MICROSCALED_SURFACE_STORAGE.md).

The shared principle is simple:

> A globally wide numeric range often has a much smaller local residual range.
> Store a local reference once, encode the residual cheaply, and carry an
> explicit error bound with the representation.

The target here is sampled ephemeris / gravity-cache state used by fast
propagation, fleet simulation, maneuver search, and long-horizon on-rails work.

This does **not** change the gravity model. Canonical celestial state remains
f64, all gravity sources still contribute, and the exact path remains available.
A compressed representation is usable only when its declared error fits the
same bounded-approximation policy already used by the gravity hierarchy and
cohort field cache.

---

## 1. Current baseline

`EphemerisTable` already removes repeated Kepler solves by sampling source
motion over a horizon and storing component-major arrays:

~~~text
pos_x / pos_y / pos_z
vel_x / vel_y / vel_z
~~~

Runtime source state is reconstructed with cubic Hermite interpolation.

The layout is SIMD-friendly. Current x86 execution uses:

~~~text
AVX-512: 8 f64 lanes
AVX2:    4 f64 lanes
scalar:  tail / fallback
~~~

Every stored sample still pays six full f64 values per source.

For the current 40k-tick fleet benchmark, with one node every eight steps and
roughly 30 sources, the raw state payload is only about:

~~~text
5000 nodes * 30 sources * 6 components * 8 bytes ~= 7.2 MiB
~~~

That fits in a large desktop L3 cache, so this proposal is **not expected to
materially improve that benchmark**.

The intended workloads are larger:

- long warp horizons;
- multiple simultaneously resident prediction horizons;
- maneuver-search workers holding several source tables;
- very long route planning;
- persistent server caches;
- UMA machines where CPU and GPU share memory bandwidth.

---

## 2. Apply locality in time, not across unrelated bodies

Surface microscaling groups spatially nearby texels because their local value
range is small.

For ephemerides, the useful locality is primarily **temporal**.

Do not assume that several unrelated bodies at one epoch share a useful numeric
range merely because a SIMD kernel evaluates them together. Their barycentric
positions may differ enormously.

Instead, define a logical residual block over one body's short track segment:

~~~text
body track
  |
  +-- segment t0..tN
      |
      +-- predictor / anchor state
      +-- residual scale metadata
      +-- packed position residuals
      +-- packed velocity residuals
~~~

The physical layout may still be transposed into component-major or AoSoA form
so the hot path decodes 4 or 8 bodies at once.

Logical codec locality and physical SIMD layout are separate concerns.

---

## 3. Candidate representation

The first prototype should compare at least:

1. constant anchor;
2. linear state predictor;
3. low-order polynomial / Hermite-compatible predictor.

A simple linear predictor is:

[
\hat{\mathbf p}(t)=\mathbf p_0+\mathbf v_0\Delta t
]

[
\hat{\mathbf v}(t)=\mathbf v_0
]

with stored residuals:

[
\Delta\mathbf p=\mathbf p-\hat{\mathbf p}
]

[
\Delta\mathbf v=\mathbf v-\hat{\mathbf v}.
]

Candidate precision ladder:

~~~text
Residual8
Residual16
Residual32 / compact-float candidate, only if measured useful
Raw64 exact fallback
~~~

Codec choice must be deterministic.

The prototype should compare shared vector scales against per-component scales.
Per-component metadata costs more but may avoid wasting precision when one axis
has a much smaller residual range than the others.

---

## 4. Canonical state remains f64

The project invariant remains unchanged:

- authoritative spatial state is f64;
- baked canonical ephemerides remain authoritative;
- the gravity law is unchanged;
- compressed tracks are cache / acceleration representations;
- loss of precision is allowed only under an explicit numerical envelope.

The first implementation must therefore behave like a derived cache:

~~~text
canonical baked ephemeris
          |
          v
  sampled f64 source states
          |
          v
 residual table encoding
          |
          v
 bounded decode
          |
          v
 gravity evaluator
~~~

If a representation cannot satisfy the current gravity error budget, runtime
falls back to a tighter codec rung or the exact source path.

Do not make a lossy residual table the only copy of canonical source state in
the first implementation.

---

## 5. Hermite interpolation gives a clean decode-error bound

The existing table reconstructs positions with cubic Hermite interpolation.

For normalized segment coordinate (s \in [0,1]):

[
\mathbf p(s)
 = h_{00}(s)\mathbf p_0
 + h_{10}(s)h\mathbf v_0
 + h_{01}(s)\mathbf p_1
 + h_{11}(s)h\mathbf v_1.
]

Suppose decoded endpoint positions have norm error at most
(epsilon_p), and decoded endpoint velocities have norm error at most
(epsilon_v).

Then:

[
\|\delta\mathbf p(s)\|
\le
(|h_{00}|+|h_{01}|)\epsilon_p
+
h(|h_{10}|+|h_{11}|)\epsilon_v.
]

On (s \in [0,1]):

[
|h_{00}|+|h_{01}|=1
]

and

[
|h_{10}|+|h_{11}|=s(1-s)\le\frac14.
]

Therefore a convenient segment-wide bound is:

[
\boxed{
\epsilon_{interp}
\le
\epsilon_p + \frac{h}{4}\epsilon_v
}
]

where (h) is the node spacing in seconds.

A future predictor/codec may publish a tighter bound, but never a weaker
guarantee than the actual decoded path.

---

## 6. Convert source-position error into gravity error

For one point-mass source:

[
\mathbf a(\mathbf r)=
\mu\frac{\mathbf r}{|\mathbf r|^3}.
]

The spectral norm of the Jacobian is:

[
\|J\|_2 = \frac{2\mu}{|\mathbf r|^3}.
]

This is the same derivative structure already used by the gravity hierarchy's
conservative monopole opening bound.

If the decoded source position is within (epsilon) metres of the canonical
position, and source-target distance is (d > epsilon), the mean-value bound
gives:

[
\boxed{
\|\delta\mathbf a\|
\le
\frac{2\mu\epsilon}{(d-\epsilon)^3}
}
]

where (epsilon) may be the segment-wide (epsilon_{interp}).

The evaluator can therefore make a target-specific decision:

~~~text
decode source / table block
compute source-position error bound
convert it to acceleration-error bound at this target

if bound fits remaining gravity budget:
    accept compressed source state
else:
    use tighter representation or exact source state
~~~

No gameplay-specific class or SOI-like heuristic is required.

---

## 7. Compose the existing approximation budgets

The gravity hierarchy already returns a conservative acceleration bound for
accepted aggregate nodes. Cohort patches also have a declared approximation
budget.

Residual ephemeris storage introduces another bounded source of error.

The invariant is therefore:

[
E_{codec}
+
E_{aggregate}
+
E_{cohort}
\le
E_{gravity\_budget}.
]

The implementation does not have to reserve fixed percentages for each source.
It may spend one shared remaining budget deterministically as approximations are
accepted, like the current tree traversal already does.

If an aggregate node's barycenter is itself decoded from a residual track, its
decode error must be included rather than silently treating that center as exact.

---

## 8. Close approaches naturally fall back

The acceleration error caused by source-position error grows approximately as
(d^{-3}).

Therefore the same compressed block may be excellent in deep space and
unacceptable near a source. That is desirable.

~~~text
far from source
    Residual8 often fits

closer
    Residual16 or tighter representation

near well / precision-critical
    Raw64 or exact source evaluation
~~~

This is the same policy as the source hierarchy: cheaper representations are
accepted by an error bound, not by a hard spatial boundary.

---

## 9. Preserve the current SIMD hot path

Compression is useful only if saved memory traffic exceeds decode cost.

The logical temporal block must not force the gravity loop back into scalar
per-body object decoding.

A preferred physical layout keeps residual samples component-major:

~~~text
segment
  node
    pos_x residuals for bodies...
    pos_y residuals for bodies...
    pos_z residuals for bodies...
    vel_x residuals for bodies...
    vel_y residuals for bodies...
    vel_z residuals for bodies...
~~~

with predictor/scale metadata in parallel arrays.

The hot x86 path can then conceptually become:

~~~text
load 8x or 4x packed residuals
widen to integer/f64 lanes
decode = predictor + scale * residual
feed decoded centers directly into gravity kernel
~~~

Candidate targets:

- AVX2 4-wide baseline;
- AVX-512 8-wide optional fast path;
- scalar portable fallback.

Benchmark fused decode+gravity, not decode in isolation.

A representation that compresses 4x but makes force evaluation slower is not a
win for the main simulation path.

---

## 10. Metadata must not erase the compression win

Very short residual blocks reduce local range but increase metadata cost.
Very long blocks amortize metadata but require wider residuals.

Benchmark at least:

~~~text
segment lengths:
    4 / 8 / 16 / 32 nodes

precision:
    i8 / i16 / raw64

scale:
    per-vector
    per-component

predictor:
    anchor
    linear
    low-order polynomial
~~~

Report:

- encoded bytes;
- bytes per source-state sample;
- compression ratio vs six f64 values;
- maximum position decode error;
- maximum velocity decode error;
- Hermite segment error bound;
- measured source-state / trajectory error;
- decode throughput;
- fused decode + gravity throughput.

---

## 11. Residency interaction

Unlike material pages, ephemeris tables are mostly immutable after construction,
so dirty-block upload logic is not the primary benefit.

The representation is still useful for residency:

~~~text
hot current horizon
    decoded / tightly packed RAM

warm planning horizons
    residual-compressed RAM

cold long-range tables
    mmap / file-backed storage
~~~

A future planner cache may reuse LRU/slab ideas from the surface microstore, but
the first experiment should isolate numeric compression from cache policy.

---

## 12. Scope and non-goals

Apply first to:

- `EphemerisTable` source position/velocity samples;
- long-horizon on-rails source caches;
- maneuver/planner source tables;
- optional precomputed aggregate-node tracks.

Do not initially apply to:

- mu values;
- accumulated target acceleration vectors;
- authoritative vehicle state;
- integration state;
- canonical save data;
- short-lived `GravityNodeFrame` values without profiling evidence.

Do not quantize the final gravity sum merely to make the representation look
uniform.

---

## 13. Prototype plan

### Phase A — CPU reference codec

Implement a backend-neutral reference representation, provisionally
`EphemerisResidualTable`.

Requirements:

- deterministic encode/decode;
- Raw64 reference mode;
- Residual16;
- Residual8;
- explicit per-block position and velocity error bounds;
- scalar correctness oracle.

### Phase B — Hermite proof and gravity bound

Regression tests must prove:

- decoded endpoint errors stay within metadata;
- interpolated position error stays within the declared Hermite bound;
- measured acceleration error stays below the converted gravity bound;
- exact fallback happens before the configured budget is exceeded.

### Phase C — SIMD decode

Implement:

- AVX2 decode path;
- optional AVX-512 decode path;
- scalar fallback;
- differential tests against scalar decode;
- fused decode + gravity benchmark.

### Phase D — real workloads

Measure:

- current 40k-tick fleet benchmark;
- a deliberately cache-cold long horizon;
- many simultaneously resident planning horizons;
- maneuver search;
- low-orbit and deep-space cases.

The current small fleet table is expected to show little or no gain. That is not
a failure if larger residency workloads show a clear bandwidth/cache win.

### Phase E — optional hierarchy tracks

Only after the source-table path is proven, test the same codec for baked
aggregate-node barycenter/radius/quadrupole tracks.

Any additional error must join the existing hierarchy error accounting.

---

## 14. Acceptance gates

A production compressed gravity-table path requires all of:

1. deterministic decoding;
2. explicit finite position/velocity error metadata;
3. a conservative Hermite interpolation bound;
4. conversion of source-state error into acceleration error;
5. total approximation error within the configured gravity budget;
6. exact/raw fallback;
7. differential tests against canonical f64 ephemerides;
8. a measured workload where end-to-end throughput or useful residency improves;
9. no regression of the AVX2 baseline path;
10. an AVX-512 fast path only where separately measured useful.

---

## 15. Design summary

The reusable idea from microscaled surface storage is not a particular 2/4/6-bit
format. It is a hierarchy of references:

~~~text
canonical f64 ephemeris
        |
        v
track / segment predictor
        |
        v
local residual scale
        |
        v
small packed residual
~~~

For gravity, representation error can then be translated into the same physical
unit already used by runtime approximation policy:

~~~text
source-state error [m]
        |
        v
gravity Jacobian bound
        |
        v
acceleration error [m/s^2]
        |
        v
shared gravity error budget
~~~

That keeps the optimization representation-driven rather than physics-changing:
Thessa still evaluates the same gravitational field, but cached source samples
do not need full global precision when a local bounded delta is sufficient.


---

## 16. Related bounded-residual domains

This gravity-specific design is one instance of a broader project pattern.

The aerodynamic analogue is documented separately in
[`docs/43_AERO_COEFFICIENT_RESIDUAL_STORAGE.md`](43_AERO_COEFFICIENT_RESIDUAL_STORAGE.md):
large Mach/alpha/beta/Re/control coefficient fields can use local predictors,
adaptive packed residuals, and explicit conversion from coefficient-space error
to force/moment error.

Keep the domains separate in code. Reuse the representation principles and
error-budget discipline, not a renderer- or gravity-specific container type.
