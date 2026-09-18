# 43 — Bounded residual storage for aerodynamic coefficient fields

Status: **design / prototype target**.

Implementation status (branch `feat/aero-residual-prototype`):

- scalar CPU reference `AeroResidualTable` implemented beside the canonical
  `AeroCoefficientTable`;
- 4x4 Mach/alpha tiles use a bilinear f64 corner predictor with independent
  per-coefficient residual scales;
- cheapest-first `Residual4 -> Residual6 -> Residual8 -> Residual16 -> Raw64`
  selection is measured against explicit per-coefficient max-absolute-error
  budgets;
- payload is contiguous across tiles; edge-partial tiles and canonical
  clamping/bilinear sampling semantics are covered;
- `sample_with_error` returns a local interpolation-safe coefficient envelope
  from the four contributing tile bounds; `AeroCoefficientError::physical_bound`
  converts it into conservative force and moment envelopes using
  q/S/c/moment-arm inputs;
- unit tests cover adaptive selection, interpolation-space error, odd extents,
  zero-budget fallback, and physical error conversion;
- `aero_residual` benchmark reports storage density plus scalar
  decode/interpolation overhead on a 257x257 synthetic stall/transonic field.

Still open: integration into `PanelAeroModel`, physical-budget-driven codec
selection, AVX2/AVX-512 fused decode, real VLM/CFD fixtures, and
higher-dimensional coefficient fields.

This document applies the same local-reference / bounded-residual principle used
by surface microstorage and ephemeris residual storage to aerodynamic coefficient
tables.

Related designs:

- [`docs/41_MICROSCALED_SURFACE_STORAGE.md`](41_MICROSCALED_SURFACE_STORAGE.md)
  — renderer-side microscaled pages;
- [`docs/42_EPHEMERIS_RESIDUAL_STORAGE.md`](42_EPHEMERIS_RESIDUAL_STORAGE.md)
  — bounded residual storage for gravity/ephemeris caches;
- [`docs/11_AERODYNAMICS.md`](11_AERODYNAMICS.md)
  — aerodynamic model and validation.

The reusable principle is:

> Keep the physical model and its explicit error envelope, but avoid paying full
> global numeric range for every locally smooth sample.

This proposal targets imported/offline aerodynamic data, especially future
multi-dimensional VLM/CFD coefficient fields. It does **not** replace the current
analytic panel model or its SIMD kernels.

---

## 1. Current baseline

`AeroCoefficientTable` currently stores a regular bilinear table over:

~~~text
Mach x alpha
~~~

with one `AeroCoefficients` value per grid point:

~~~text
CL
CD
CY
Cm
~~~

All four values are currently f64, so one grid point costs 32 bytes before
container overhead.

The current two-dimensional tables are small enough that compression is unlikely
to matter. The motivating workload is a future coefficient surface such as:

~~~text
Mach
x alpha
x beta
x Reynolds
x control deflection
x configuration / flap / gear state
~~~

A dense field grows multiplicatively with every axis. At that point memory
residency and cache bandwidth become first-class runtime costs.

The current analytic path remains the preferred path where it is both adequate
and cheaper. Residual storage is for large sampled coefficient fields where the
table is the model.

---

## 2. Locality lives in aerodynamic parameter space

Coefficient fields are often globally nonlinear but locally smooth.

Examples:

- attached-flow CL varies smoothly with alpha over a local interval;
- CD changes gradually through much of the subsonic envelope;
- Cm surfaces often have locally simple slopes;
- beta/control sweeps are locally correlated;
- sharp behavior is concentrated near stall, transonic transitions, separated
  flow, control saturation, or configuration boundaries.

Therefore a table tile can store:

~~~text
local predictor
+ local residual scale
+ packed residuals
~~~

instead of storing every coefficient at full global precision.

A logical tile might initially be 4x4 over Mach x alpha. Future higher-dimensional
tables may tile the two fastest-varying axes while treating the remaining axes as
slice selectors.

Do not assume one fixed tile shape is optimal.

---

## 3. Predictor-first representation

A plain block minimum/range codec is a useful baseline, but aerodynamic tables
have stronger structure than arbitrary scalar textures.

The preferred prototype should compare:

1. constant/base predictor;
2. separable linear predictor;
3. bilinear plane over a Mach-alpha tile;
4. optional higher-order local predictor only if benchmarks justify it.

For one coefficient C over a 2D tile:

[
\hat C(M,\alpha)
=
a
+
b\,\Delta M
+
c\,\Delta\alpha
+
d\,\Delta M\,\Delta\alpha.
]

The stored value is then:

[
\Delta C
=
C-\hat C.
]

Smooth regions should leave a very small residual range.

Candidate precision ladder:

~~~text
Residual4 / Residual6
Residual8
Residual16
Raw32 or Raw64 fallback
~~~

The exact widths are not fixed by this document. The encoder should choose the
cheapest representation that satisfies an explicit physical error budget.

A separate scale per coefficient is preferred initially:

~~~text
scale_CL
scale_CD
scale_CY
scale_Cm
~~~

because the dynamic ranges and physical sensitivities differ substantially.

---

## 4. Preserve interpolation semantics

The current `AeroCoefficientTable::sample` performs bilinear interpolation in
Mach and alpha.

A compressed representation must not silently change table semantics.

Two valid implementation strategies are:

### Decode-grid-point strategy

Decode the four grid samples participating in the existing bilinear interpolation,
then run the same interpolation logic.

Advantages:

- exact preservation of current interpolation structure;
- simple differential testing;
- easy fallback to raw samples.

### Decode-local-model strategy

Store a local predictor and residual form that evaluates the interpolated value
directly.

Advantages:

- potentially fewer memory loads;
- predictor can absorb most local variation.

Costs:

- harder proof that the result matches the declared table interpolation/error
  envelope;
- more complicated boundary handling between tiles.

The first prototype should prefer decode-grid-point semantics and use the local
predictor only as a storage transform. Direct local-model evaluation is a later
optimization.

---

## 5. Coefficient error converts directly into physical force error

For a panel with dynamic pressure q and area S:

[
L=qS C_L,
qquad
D=qS C_D,
qquad
Y=qS C_Y.
]

If the codec guarantees:

[
|\delta C_L|\le\epsilon_L,
qquad
|\delta C_D|\le\epsilon_D,
qquad
|\delta C_Y|\le\epsilon_Y,
]

then the corresponding force-component bounds are:

[
|\delta L|\le qS\epsilon_L,
]

[
|\delta D|\le qS\epsilon_D,
]

[
|\delta Y|\le qS\epsilon_Y.
]

A simple conservative total-force bound is therefore:

[
\boxed{
\|\delta\mathbf F\|
\le
qS(\epsilon_L+\epsilon_D+\epsilon_Y)
}
]

although a tighter norm bound may be used if the basis transformation is
formally accounted for.

This is the important architectural property: codec error can be expressed in a
decision-relevant physical unit rather than an arbitrary "coefficient bits"
metric.

---

## 6. Moment error has two contributions

The direct aerodynamic pitching-moment term is:

[
M_c=qSc C_m
]

for reference chord c.

Thus:

[
|\delta M_c|
\le
qSc\epsilon_m.
]

The panel force is also applied at a center-of-pressure / moment arm r, so force
error produces an additional torque error:

[
\delta\boldsymbol\tau_r
=
\mathbf r\times\delta\mathbf F.
]

Therefore:

[
\|\delta\boldsymbol\tau_r\|
\le
\|\mathbf r\|\,\|\delta\mathbf F\|.
]

A conservative panel torque envelope is:

[
\boxed{
\|\delta\boldsymbol\tau\|
\le
qSc\epsilon_m
+
\|\mathbf r\|\,\|\delta\mathbf F\|
}
]

with the exact reference point matching the production force-assembly path.

This lets the encoder/runtime judge a compressed tile against both force and
control-relevant moment budgets.

---

## 7. Budget selection should be physical, not bit-centric

Do not choose a codec solely because:

~~~text
max coefficient error <= 0.001
~~~

Instead define the accepted representation through physical envelopes.

For a declared operating envelope of q, S, c, and moment arm, a tile may be
accepted if it guarantees, for example:

~~~text
force error <= configured panel/fleet tolerance
moment error <= configured control tolerance
~~~

The implementation may still store coefficient-space error metadata because it
is compact and reusable, but the acceptance policy should convert it into force
and moment bounds.

For a table shared by many vehicles or geometries, keep codec metadata
geometry-independent and perform the q/S/c/r conversion at runtime or in the
vehicle compiler.

For a vehicle-specific compiled table, stronger precomputed physical bounds may
be possible.

---

## 8. Adaptive precision is especially useful near difficult regimes

Aerodynamic fields are not uniformly smooth.

Expected easy regions:

- low-alpha attached flow;
- broad subsonic ranges away from stall;
- regular beta sweeps;
- locally linear control response.

Expected difficult regions:

- stall onset and post-stall transition;
- transonic drag rise;
- supersonic branch transitions;
- separated-flow changes;
- control saturation or nonlinear hinge effects;
- abrupt configuration changes.

The codec ladder should naturally react:

~~~text
smooth tile
    Residual4 / Residual6

ordinary tile
    Residual8

sharp tile
    Residual16

pathological / discontinuous tile
    Raw32 / Raw64
~~~

No smoothing is permitted merely to improve compression.

If a physical discontinuity is present in the source data, the representation
must preserve it within the declared error envelope.

---

## 9. Do not compress the current analytic SIMD path for its own sake

The existing analytic coefficient path already has dedicated AVX2/AVX-512
kernels and avoids table traffic.

That path should remain untouched unless profiling finds an actual bottleneck.

This proposal is for:

- imported VLM/CFD coefficient grids;
- future higher-dimensional lookup models;
- precomputed control/configuration surfaces;
- expensive offline models whose runtime form is table-driven.

Do not add encode/decode overhead to an analytic formula merely because residual
storage is available elsewhere in the engine.

---

## 10. SIMD-friendly physical layout

The storage transform should not destroy the current SoA/SIMD strategy.

A future table layout may keep each coefficient in independent streams:

~~~text
CL residuals...
CD residuals...
CY residuals...
Cm residuals...
~~~

with tile predictor/scale metadata in adjacent arrays.

The hot path can then:

~~~text
load packed residual lanes
widen
FMA(predictor/scale, residual)
interpolate
assemble panel force/moment
~~~

Candidate x86 targets:

- AVX2 baseline;
- AVX-512 optional fast path;
- scalar portable fallback.

As with the gravity residual proposal, benchmark **fused decode + interpolation +
force assembly**, not codec decode in isolation.

A 4x smaller table that makes the actual aero loop slower is not a successful
runtime optimization.

---

## 11. Interaction with batch aerodynamics

For fleet-scale evaluation, many panels may query the same or nearby coefficient
tiles.

Possible wins include:

- more coefficient data resident in L2/L3;
- fewer cache misses across many vehicles;
- cheaper prefetch of adjacent Mach/alpha tiles;
- sharing decoded/predictor metadata across panels using the same airfoil/model;
- better effective density for large multi-axis tables.

The cache key should describe the aerodynamic model/table and tile coordinates,
not a vehicle entity.

A decoded-tile cache may be useful only after the packed direct-sample path is
benchmarked. Avoid re-expanding the entire table by default.

---

## 12. Validation requirements

The compressed table must be validated at three levels.

### Coefficient-space

For every sampled grid point:

- max absolute error per coefficient;
- RMS error per coefficient;
- worst tile / regime;
- exact/raw fallback coverage.

### Interpolation-space

Sample inside cells, not only at grid points:

- compare compressed bilinear result against canonical f64 table interpolation;
- explicitly hit tile boundaries;
- sweep stall/transonic/supersonic transition regions;
- verify no new discontinuity is introduced by tile boundaries.

### Physical-space

Compare full panel outputs:

- force vector error in newtons;
- moment error in N*m;
- representative q/S/c/r envelopes;
- trajectory/control regressions where the table affects decisions.

The physical regression is the merge gate.

---

## 13. Prototype plan

### Phase A — scalar storage transform

Implement a reference `AeroResidualTable` beside the current
`AeroCoefficientTable`.

Start with:

- 4x4 Mach-alpha tiles;
- bilinear predictor;
- Residual8;
- Residual16;
- Raw64;
- deterministic encoding;
- exact current table as the oracle.

### Phase B — adaptive codec ladder

Add:

- lower-bit candidate(s), initially R4/R6;
- cheapest-first selection under declared coefficient error bounds;
- tile statistics and byte accounting;
- special coverage for stall/transonic tiles.

### Phase C — physical error budgeting

For each tile/codec rung:

- convert coefficient error to force/moment bounds;
- reject representations that exceed the configured physical envelope;
- prove runtime interpolation does not exceed the stored/developed bound.

### Phase D — SIMD decode

Implement:

- AVX2 baseline;
- optional AVX-512 path;
- scalar fallback;
- differential tests across all tiers.

### Phase E — realistic large table

The optimization is not justified by the current tiny Mach-alpha table.

Generate or import a deliberately realistic multi-axis workload and measure:

- total bytes;
- bytes per grid point;
- cache miss rate;
- batch aero throughput;
- fused decode/interpolate/force cost;
- physical error envelope.

Only then decide whether the representation belongs in the default runtime.

---

## 14. Acceptance gates

A production compressed aerodynamic table path requires all of:

1. deterministic encoding/decoding;
2. no change to analytic aero semantics;
3. preserved table interpolation semantics or a formally bounded replacement;
4. explicit per-coefficient error metadata;
5. conversion to force and moment error bounds;
6. adaptive fallback in difficult regimes;
7. no artificial smoothing of stall/transonic/discontinuous source data;
8. scalar oracle and differential tests;
9. AVX2 baseline maintained;
10. end-to-end throughput or residency improvement on a realistically large
    coefficient field.

---

## 15. Shared project pattern

Surface data, ephemeris caches, and aerodynamic coefficient fields are different
domains, but the same representation rule applies:

~~~text
wide global value range
        |
        v
local reference / predictor
        |
        v
small residual range
        |
        v
adaptive packed representation
        |
        v
explicit error bound in domain-relevant units
        |
        v
raw / exact fallback when the bound does not fit
~~~

For aerodynamics the final unit is especially convenient:

~~~text
coefficient error
        |
        v
q * S / q * S * c
        |
        v
force [N] / moment [N*m]
        |
        v
simulation / control error budget
~~~

The point is not to "use FP4 everywhere". The useful idea is bounded local
residual representation: spend bits where the physics needs them and stop
repeatedly storing global precision where a local predictor already explains
most of the value.
