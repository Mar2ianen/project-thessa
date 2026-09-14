# 24 — Analytic propagation inside an affine gravity patch

## Status

**Implemented and tested prototype.** The implementation is in
`crates/sim-core/src/affine_propagator.rs`; patch compilation and bounds are in
`gravity_patch.rs`; tests are in `src/tests.rs`; the A/B benchmark is
`benches/affine_prop.rs`.

## 24.1. Local affine field

For a patch anchored at `x0`:

```text
g(x) ≈ g0 + J (x - x0)
```

`g0` is the field at the anchor and `J` is the local gravity-gradient tensor.
The translational equation becomes:

```text
x'' = J x + c
c   = g0 - J x0
```

This is a linear second-order ODE with constant coefficients while the patch is
frozen.

## 24.2. State-transition form

For a six-dimensional state `y = [x, v]`:

```text
y' = A y + b

A = [ 0 I ]
    [ J 0 ]
```

The homogeneous part is propagated by `exp(A dt)`. Because a gravity-gradient
tensor is symmetric for point-mass sources, the implementation diagonalizes
`J` and propagates independent scalar modes instead of using a generic hot-path
6×6 matrix exponential.

For an eigenvalue `lambda`, the mode uses trigonometric functions when
`lambda < 0`, hyperbolic functions when `lambda > 0`, and the polynomial limit
when `lambda = 0`. The code uses stable small-argument series branches and
deterministic eigenvector ordering.

## 24.3. Anchor-relative formulation

The runtime stores/propagates patch-relative coordinates where possible. This
avoids large absolute terms in the affine constant and makes the validity
bound depend on the target excursion from the anchor.

The formulation is mathematically equivalent to the absolute affine equation;
the implementation includes regression tests for both the constant term and
the anchor-relative call sites.

## 24.4. Piecewise driver

`propagate_piecewise` follows this policy:

1. compile a cohort patch at the current trajectory point;
2. refuse analytic mode if input, budget, or source conditions are invalid;
3. propagate one analytic segment;
4. compute the posted field/remainder bound;
5. accept the endpoint only while the bound is inside the configured budget;
6. rebuild the patch or return an explicit fallback reason otherwise.

The driver never treats a failed certificate as a success. Exact stepped
propagation remains the authority near bodies, with exact-near terms, under
thrust/contacts/atmosphere, or whenever the bound is exhausted.

## 24.5. Verification

The tests compare the analytic frozen-field solution against an independent
RK4 oracle, check deterministic compilation, eigen identities, zero/small
eigenvalue limits, finite outputs, and posted bound envelopes. The piecewise
driver is also tested near bodies, with an exact-near patch, with zero budget,
and across extension boundaries.

```bash
cargo test -p thessa-sim-core
cargo bench -p thessa-sim-core --bench affine_prop
```

The benchmark compares analytic STM propagation with repeated Verlet/RK4
evaluation for frozen and real deep-space patches. It is a CPU microbenchmark,
not an FPS claim.

## 24.6. Scope and limits

- coefficients are frozen only for the certified segment;
- moving source evolution is represented by the patch bound and triggers a
  rebuild when necessary;
- the analytic path does not replace atmosphere, thrust, contact, actuator,
  or thermal integration;
- a successful bound is an absolute numerical envelope for the configured
  state/field, not a global proof for arbitrary trajectories;
- the field remains the same gravity model; only its local representation and
  propagation method change.
