# 24 — Analytic propagation inside an affine gravity patch

## Status

Design note / experiment proposal for Project Thessa. Implemented and
verified — see §18. Implementation: `crates/sim-core/src/affine_propagator.rs`
(`AffinePropagator`, `affine_segment_bound`), tests in `src/tests.rs`,
benchmarks `benches/affine_prop.rs`.

The starting point is the local affine gravity approximation

\[
\mathbf g(\mathbf x)
\approx
\mathbf g_0 + J(\mathbf x-\mathbf x_0),
\]

where:

- \(\mathbf x_0\) is the patch anchor;
- \(\mathbf g_0 = \mathbf g(\mathbf x_0)\);
- \(J = \partial \mathbf g / \partial \mathbf x\) is the local gravity-gradient / tidal tensor;
- the patch is valid only while its existing spatial/temporal error bound remains inside the allocated gravity budget.

The important observation is that **once \(g_0\) and \(J\) are frozen for a patch, the equations of motion are linear with constant coefficients**. There is therefore no requirement to advance the vehicle through that patch with RK/Verlet-style microsteps.

The patch can be propagated analytically.

---

## 1. From affine gravity to a linear ODE

The translational equation is

\[
\ddot{\mathbf x}
=
\mathbf g_0 + J(\mathbf x-\mathbf x_0).
\]

Define

\[
\mathbf c = \mathbf g_0 - J\mathbf x_0.
\]

Then

\[
\ddot{\mathbf x}=J\mathbf x+\mathbf c.
\]

This is a linear second-order ODE with constant coefficients.

A generic first-order state-space form is

\[
\mathbf y =
\begin{bmatrix}
\mathbf x \\
\mathbf v
\end{bmatrix},
\]

\[
\dot{\mathbf y}
=
A\mathbf y+\mathbf b,
\]

with

\[
A=
\begin{bmatrix}
0&I\\
J&0
\end{bmatrix},
\qquad
\mathbf b=
\begin{bmatrix}
0\\
\mathbf c
\end{bmatrix}.
\]

The exact solution over one frozen patch interval \(\Delta t\) is

\[
\mathbf y(t+\Delta t)
=
e^{A\Delta t}\mathbf y(t)
+
\int_0^{\Delta t}
e^{A(\Delta t-\tau)}\mathbf b\,d\tau.
\]

So even in the most generic form, propagation is a matrix exponential plus a small affine term.

---

## 2. Better form: propagate deviations around a reference trajectory

The constant term disappears if we propagate relative to a reference solution.

Let \(\mathbf x_r(t)\) be a reference trajectory satisfying

\[
\ddot{\mathbf x}_r
=
\mathbf g(\mathbf x_r).
\]

For nearby trajectories define

\[
\delta\mathbf x
=
\mathbf x-\mathbf x_r,
\]

\[
\delta\mathbf v
=
\mathbf v-\mathbf v_r.
\]

Linearizing around the reference trajectory gives

\[
\delta\ddot{\mathbf x}
=
J\,\delta\mathbf x.
\]

Now define

\[
\delta\mathbf y=
\begin{bmatrix}
\delta\mathbf x\\
\delta\mathbf v
\end{bmatrix}.
\]

Then

\[
\delta\dot{\mathbf y}
=
A\,\delta\mathbf y,
\]

with

\[
A=
\begin{bmatrix}
0&I\\
J&0
\end{bmatrix}.
\]

Therefore

\[
\delta\mathbf y(t+\Delta t)
=
\Phi(\Delta t)\,\delta\mathbf y(t),
\]

where

\[
\Phi(\Delta t)
=
e^{A\Delta t}
\]

is the **state transition matrix** (STM).

This is especially useful for maneuver planning: thousands of candidate trajectories can share the same \(\Phi\) while they remain inside the same validity region.

---

## 3. The gravity-gradient tensor is symmetric

For a point-mass gravitational field, the tidal tensor is symmetric.

For one source:

\[
J
=
\mu
\left(
\frac{3\mathbf r\mathbf r^T}{r^5}
-
\frac{I}{r^3}
\right).
\]

A sum of symmetric tensors is also symmetric, so the affine far-field tensor remains symmetric.

Therefore

\[
J=Q\Lambda Q^T,
\]

where:

- \(Q\) is orthogonal;
- \(\Lambda=\operatorname{diag}(\lambda_1,\lambda_2,\lambda_3)\).

Transform the relative coordinates:

\[
\mathbf q=Q^T\delta\mathbf x,
\qquad
\mathbf u=Q^T\delta\mathbf v.
\]

Then the vector ODE becomes three independent scalar ODEs:

\[
\ddot q_i=\lambda_i q_i.
\]

This removes the need for a generic \(6\times6\) matrix exponential on the hot path.

---

## 4. Closed-form solution per eigenmode

For each eigenvalue \(\lambda\):

### 4.1 \(\lambda < 0\): oscillatory mode

Let

\[
\omega=\sqrt{-\lambda}.
\]

Then

\[
q(t+\Delta t)
=
q(t)\cos(\omega\Delta t)
+
\frac{u(t)}{\omega}\sin(\omega\Delta t),
\]

\[
u(t+\Delta t)
=
-\omega q(t)\sin(\omega\Delta t)
+
u(t)\cos(\omega\Delta t).
\]

### 4.2 \(\lambda > 0\): hyperbolic mode

Let

\[
\sigma=\sqrt{\lambda}.
\]

Then

\[
q(t+\Delta t)
=
q(t)\cosh(\sigma\Delta t)
+
\frac{u(t)}{\sigma}\sinh(\sigma\Delta t),
\]

\[
u(t+\Delta t)
=
\sigma q(t)\sinh(\sigma\Delta t)
+
u(t)\cosh(\sigma\Delta t).
\]

### 4.3 \(\lambda \approx 0\): free-drift mode

Use the limiting form

\[
q(t+\Delta t)=q(t)+u(t)\Delta t,
\]

\[
u(t+\Delta t)=u(t).
\]

In implementation this branch needs a numerical threshold so that division by a tiny \(\sqrt{|\lambda|}\) does not destroy precision.

---

## 5. Patch propagator

A compiled affine patch can therefore contain more than just

```text
g0
J
exact-near sources
error bound
```

It can also contain a precompiled local propagator:

```text
AffineGravityPatch
├── center
├── g0
├── J
├── exact-near sources
├── error bound
└── propagator
    ├── Q
    ├── lambda[3]
    └── optionally cached coefficients for common dt values
```

For a fixed \(\Delta t\), cache

```text
mode[i]:
    C_i
    S_i
    derivative coefficients
```

and one candidate propagation becomes approximately:

```text
delta_x -> Q^T delta_x
delta_v -> Q^T delta_v

3 independent 2x2 mode updates

q -> Q q
u -> Q u
```

This is only a small number of dot products, multiplies and adds per candidate.

---

## 6. Piecewise analytic propagation

A single affine patch is not valid over arbitrary distance or time.

The intended algorithm is therefore **piecewise analytic**:

```text
current state / candidate cloud
        ↓
compile affine gravity patch
        ↓
build local STM / eigenmode propagator
        ↓
propagate analytically to the largest allowed dt
        ↓
check spatial + temporal validity
        ↓
still valid?
   ├─ yes → continue with same patch
   └─ no  → rebuild patch / split cohort / exact fallback
```

Over several segments:

\[
\Phi_{\mathrm{total}}
=
\Phi_n
\Phi_{n-1}
\cdots
\Phi_1.
\]

This is the same general idea as an adaptive integrator, except the inner segment is solved analytically instead of with many force evaluations.

---

## 7. Maneuver planner application

This is particularly attractive for maneuver nodes.

A broad-search planner can evaluate many nearby candidate burns:

```text
one candidate cloud
        ↓
one ephemeris frame / local field description
        ↓
one affine patch
        ↓
one Φ(dt)
        ↓
thousands of candidate state vectors
        ↓
matrix/vector propagation
```

A staged planner can then use progressively tighter models:

```text
10 000 candidates
    ↓
loose gravity budget
coarse piecewise-analytic propagation

1 000 survivors
    ↓
tighter patch budget

100 survivors
    ↓
smaller segments / more rebuilds

5 finalists
    ↓
exact gravity + exact numerical integration
```

This keeps the authoritative exact path while making the search stage cheap.

---

## 8. Where this is likely to work well

### Short lunar transfers

Very promising.

Reasons:

- relatively short time of flight;
- candidate trajectories remain spatially coherent for a useful period;
- source geometry changes less over each local segment;
- the candidate cloud can share the same field representation;
- near a moon the patch can simply refine/rebuild.

### Local rendezvous / formation / docking

Extremely promising.

This is close to the classic domain of linearized relative-motion equations.

### Station keeping and Lagrange-region operations

Also promising, especially when a local linearization remains valid over useful intervals.

### Interplanetary transfers

Still useful, but less magical.

For a Mars-like transfer:

- \(J(t)\) changes substantially over the full horizon;
- source positions move significantly;
- candidate trajectories diverge over long times;
- the local affine validity interval can become short.

The method becomes a sequence of many local STMs rather than one giant analytic hop.

It may still be much cheaper than full exact integration of every broad-search candidate.

---

## 9. Relationship to known orbital mechanics

This idea is not new in celestial mechanics.

It belongs to the family of:

- variational equations;
- linearized relative orbital dynamics;
- state transition matrices;
- gravity-gradient / tidal-tensor propagation.

Famous special cases include:

- Hill equations;
- Clohessy-Wiltshire equations for relative motion near a circular orbit;
- Tschauner-Hempel equations for elliptic reference orbits.

The useful part for Thessa is not inventing the mathematics itself.

The useful part is that the existing **error-bounded affine gravity patch representation naturally exposes exactly the quantities needed to use this machinery adaptively in a general multi-body simulation**.

---

## 10. Important limitation: exact-near sources

The current patch form is conceptually

\[
\mathbf g(\mathbf x)
=
\mathbf g_{\mathrm{affine}}(\mathbf x)
+
\sum_{j\in\mathrm{near}}\mathbf g_j(\mathbf x).
\]

The affine far field is analytically solvable.

The exact-near point-mass terms are not, in general, compatible with the same constant-coefficient STM.

Therefore there are several regimes.

### A. No exact-near sources

Best case.

The entire patch is analytically propagatable.

### B. One dominant near source

Potentially use a different analytic base solution:

```text
exact Kepler around near body
+
linearized perturbation from far field
```

This is likely worth investigating separately.

### C. Several exact-near sources

Use the analytic affine propagator as a predictor / coarse planner path, then:

- shorten the segment;
- numerically integrate only the near correction;
- or fall back to the existing exact integrator.

The error-budget system should decide this, not a hardcoded object class.

---

## 11. Possible hybrid: analytic far field + numerical near correction

Split

\[
\ddot{\mathbf x}
=
J\mathbf x+\mathbf c
+
\mathbf f_{\mathrm{near}}(\mathbf x,t).
\]

The linear part has an exact propagator.

Then use variation of constants:

\[
\mathbf y(t+\Delta t)
=
\Phi(\Delta t)\mathbf y(t)
+
\int_0^{\Delta t}
\Phi(\Delta t-\tau)
\mathbf f_{\mathrm{near}}(\mathbf y(\tau),\tau)
\,d\tau.
\]

This suggests exponential-integrator-style schemes:

- solve the affine far field exactly;
- numerically sample only the nonlinear near correction.

If the far field dominates total source count, this may reduce the number and cost of force evaluations dramatically.

This is a separate experiment and should be benchmarked against the current integrator rather than assumed to win.

---

## 12. Numerical details

### Eigen decomposition

Because \(J\) is symmetric, use a symmetric \(3\times3\) eigensolver.

Requirements:

- deterministic ordering of eigenmodes;
- stable behavior near repeated eigenvalues;
- no arbitrary sign dependence in externally visible semantics;
- benchmark the eigensolve cost against patch compilation.

The eigensolve happens once per patch, not per candidate.

### Near-zero eigenvalues

Use series expansions or the free-drift limit when

\[
|\lambda|\Delta t^2
\]

is very small.

For example:

\[
\frac{\sin(\omega t)}{\omega}
\rightarrow t
\]

as \(\omega\to0\).

Likewise:

\[
\frac{\sinh(\sigma t)}{\sigma}
\rightarrow t.
\]

Avoid unstable division by tiny \(\omega\) or \(\sigma\).

### Hyperbolic overflow

Large positive \(\lambda\Delta t^2\) can make `cosh`/`sinh` explode.

That is also a strong indication that the patch should not be trusted over such a long interval.

The validity controller should generally cut the segment before numeric overflow becomes relevant.

---

## 13. Error accounting

The analytic solution is exact only for the frozen affine field.

The total propagation error still contains:

```text
spatial truncation of gravity field
+
temporal change of g0/J/source geometry
+
exact-near approximation, if any
+
reference-trajectory linearization error
+
floating-point error
```

The existing gravity error budget should remain the governing mechanism.

A useful future contract is:

```text
segment accepted iff

field spatial remainder
+ field temporal remainder
+ linearization/propagator remainder
<= allocated gravity propagation budget
```

The key requirement is that the analytic propagator must not bypass the existing fail-open semantics.

---

## 14. Suggested implementation structure

```rust
struct AffinePropagator {
    // Symmetric eigenbasis of J.
    basis: DMat3,
    lambda: DVec3,
}

struct StepCoefficients {
    // Per eigenmode coefficients for a specific dt.
    // Exact representation is implementation-dependent.
    pos_from_pos: DVec3,
    pos_from_vel: DVec3,
    vel_from_pos: DVec3,
    vel_from_vel: DVec3,
}
```

Conceptual API:

```rust
impl AffinePropagator {
    fn compile(jacobian: DMat3) -> Result<Self, PropagatorError>;

    fn coefficients(&self, dt: f64) -> StepCoefficients;

    fn propagate_relative(
        &self,
        coeffs: &StepCoefficients,
        dx: DVec3,
        dv: DVec3,
    ) -> (DVec3, DVec3);
}
```

For candidate batches:

```rust
for candidate in candidates {
    let (dx1, dv1) =
        propagator.propagate_relative(&coeffs, candidate.dx, candidate.dv);

    candidate.dx = dx1;
    candidate.dv = dv1;
}
```

The hot path should be allocation-free.

---

## 15. Benchmarks to run

### Benchmark A: pure affine oracle

Construct a frozen affine field and compare:

```text
analytic STM
vs
Velocity Verlet
vs
RK4 / current high-accuracy path
```

Measure:

- wall time;
- state error;
- evaluations per trajectory;
- throughput for 1 / 100 / 1k / 10k candidates.

The STM result should be treated as the exact oracle for the frozen linear field.

### Benchmark B: real gravity patch, no exact-near sources

Use a real deep-space patch.

Compare over increasing segment lengths:

```text
analytic frozen patch
vs
current cohort numerical integration
vs
exact gravity integration
```

Measure divergence and posted bounds.

### Benchmark C: lunar transfer

Generate a candidate cloud for a transfer to a moon.

Compare:

```text
full exact planner
piecewise affine STM planner
staged STM -> exact finalists
```

Metrics:

- candidates/s;
- number of patch rebuilds;
- average segment duration;
- final position/velocity error;
- maneuver ranking agreement;
- final exact revalidation failure rate.

### Benchmark D: long interplanetary transfer

Same experiment on a Mars-like horizon.

This establishes where the method stops being worthwhile.

---

## 16. Acceptance criteria

The experiment is worth keeping if it demonstrates all of the following:

1. Frozen affine propagation matches the analytic oracle to floating-point tolerance.
2. Real patch propagation never exceeds its posted propagation bound.
3. Candidate throughput materially exceeds numerical stepping.
4. The improvement survives end-to-end planner integration.
5. Finalists can always be revalidated by the exact authoritative path.
6. No object-class or mission-specific physics shortcuts are introduced.
7. CPU-only execution remains first-class.

---

## 17. Likely outcome

The most interesting possibility is that the affine patch is not merely a faster gravity evaluator.

It may be a **compiled local dynamics model**.

Instead of:

```text
for every candidate
    for every integration step
        evaluate gravity
        integrate
```

the planner may become:

```text
compile local dynamics once

for every candidate
    apply local propagator

when the error bound expires
    compile the next local dynamics segment
```

If this works, the next major gravity optimization is not:

> make `g0 + J*dx` cheaper

but:

> stop evaluating `g0 + J*dx` at every integration step.

That is the experiment.

---

## 18. Verification status (implemented)

Implementation: `crates/sim-core/src/affine_propagator.rs`
(`AffinePropagator`, `affine_segment_bound`), tests in `src/tests.rs`,
benchmarks `benches/affine_prop.rs`. Deviations from the §14 sketch are
deliberate and noted below.

Math cross-checks (proved by hand, pinned by tests):

- the tidal tensor is symmetric (matches `tidal_tensor` assembly) and
  traceless (`|tr| <= 1e-9` on a real compiled patch) — so vacuum
  dynamics is always a saddle, never pure oscillation; a hyperbolic
  direction must exist (asserted on a real patch: λmax > 0, λmin < 0);
- the absolute form `x'' = Jx + c` subsumes the relative STM form
  (`c = 0`), so the implementation exposes `propagate()` rather than
  only `propagate_relative()` — one code path for both;
- no `J^-1` anywhere (singular in general): per-mode particular
  solutions instead (`-c/λ`, resp. `c t²/2` in drift mode).

Accuracy (§16 items 1–3, 5–7):

1. Oracle: STM vs converged RK4 (h = 0.01 s) on a frozen mixed-sign
   field, three trajectories — agreement 1e-9 relative. ✅
2. Real far-only deep-space patch, analytic segments vs exact RK4 with
   per-stage frames (60/600/3600 s): divergence 0.099 м / 99 м / 21 км
   vs posted `affine_segment_bound` 0.59 м / 622 м / 183 км — margin
   x6–x9, stable across two segment decades. The posted bound is a
   field bound (m/s²); the test converts with `dt²/2` (valid here:
   σ·dt << 1 throughout). ✅
3. Throughput (`affine_prop` bench): 7 нс/кандидат vs Verlet500
   3.5 мкс (x500) and RK4-100 1.8 мкс (x260) at x1000; one analytic
   eval 40–150 нс vs an exact-RK4 segment 0.8–56 мс. ✅
4. Planner integration: no planner exists yet (same caveat as doc 23
   §12) — interface ready, finalists revalidatable by the untouched
   exact path. ⏳
5. No class shortcuts (pure eigenmode math), CPU-only. ✅

Boundary behavior is fail-open, not silent: expiry returns INFINITY as
the rebuild signal (asserted at a 10-hour excursion); patches with
exact-near sources refuse analytic propagation
(`AnalyticNeedsFarField`) instead of dropping point-mass terms (regimes
B/C remain future work, §10–§11); the raw single-target evaluator is
crate-private so a frozen patch cannot be paired with foreign-epoch
states through public API — only `CohortEvaluator` enforces the
envelope per tick.

Eigensolver contract (§12): fixed 12 Jacobi sweeps, descending sort,
sign canonicalization — same input gives bitwise the same basis
(asserted); asymmetric input is rejected, never symmetrized silently;
Taylor branch at `|λ|dt² < 1e-8` pinned against an independent series;
`σdt > 50` returns `IntervalTooLong`.

Formulation trap, caught by the convergence test (do not regress):
`propagate()` takes anchor-relative displacement and returns a
displacement, so its constant is `g0` — never `g0 - J*anchor` (that
constant belongs to the origin-anchored form and silently accelerates
in the wrong direction: exactly `2g0` off for central geometry,
quadratic in time). The API doc states the pairing explicitly, and the
budget-convergence test (tighter budget must land closer) fails on any
mixing of the two forms.

Two verification scars worth keeping: the first "honest" reference
(symplectic Euler 0.5 s) carried ~1e-3 m of its own step error and
looked like a bound violation until replaced by per-stage-frame RK4;
and the Taylor test reference was first-order in velocity while the
branch is second-order — both times the test oracle was weaker than
the code under test, both times fixed on the oracle side.
