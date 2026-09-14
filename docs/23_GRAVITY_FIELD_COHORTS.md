# 23 — Baked gravity hierarchy and target-cohort field cache

## Status

**Implemented prototype.** The design in this document is now represented by
`crates/sim-core/src/gravity_patch.rs`, its public `Cohort*` types, the affine
propagator, tests, and fleet/planner benchmarks. The implementation remains a
bounded optimization with an exact fallback; it is not an SOI or a separate
physics mode for cargo craft.

## 23.1. Problem and baseline

The exact path evaluates all relevant physical gravity sources for every target
and force evaluation. Baked ephemerides already share source motion, but many
nearby targets can still repeat the same far-field work.

The current optimization has two independent forms of sharing:

```text
source hierarchy / reusable source state
                 +
target cohort / local affine field
```

Every physical source remains present. Approximation is admitted only when its
absolute error bound fits the configured decision-relevant budget.

## 23.2. Source hierarchy

The baked system has stable parent topology. `GravityPatch` can compile a
source-side hierarchy/tree and open or combine nodes according to distance and
error policy. An aggregate node represents physical children; it is not a fake
body and must not double-count their gravity.

Far-field representation follows this ladder:

```text
far       monopole / aggregate
closer    higher aggregate detail where permitted
nearer    descend to children
very near exact body/source terms
```

The final decision is tied to an acceleration/error budget rather than a lone
hard-coded geometric `s/r` threshold. The current implementation uses explicit
config and conservative bound tests.

## 23.3. Cohort patch

For a cohort around anchor `x0`, the local field is:

```text
g(x) ≈ g0 + J (x - x0)
```

where `g0` is the exact/compiled field at the anchor and `J` is the gravity
gradient. Near terms remain exact; only the accepted far field is expanded.
The `GravityPatch` stores the anchor, field coefficients, source/window
identity, and conservative bound metadata.

`CohortEvaluator` evaluates many target positions in the same patch while
preserving target order. It reports coverage, splits, exact-near work, and
error-related telemetry. Scratch-backed evaluation avoids unnecessary hot-path
allocation.

## 23.4. Explicit policy

`CohortConfig` defines the model boundary. It controls values such as:

- allowed absolute acceleration error;
- cohort radius and split behavior;
- spatial/temporal reuse window;
- near-source opening/expansion policy;
- recursion and work guards.

If a field is too close, the source window changes, the patch cannot satisfy
the budget, or an input is invalid, the evaluator returns an error or exact
fallback decision. It never silently returns an unbounded approximation.

## 23.5. Analytic propagation

Inside a frozen far-only affine patch, translational dynamics have constant
coefficients:

```text
g(x) = g0 + J (x - x0)
x''   = J x + (g0 - J x0)
```

The symmetric gravity-gradient tensor is diagonalized and propagated with a
state-transition matrix. `propagate_piecewise` rebuilds the patch when the
posted spatial/temporal bound expires. If exact-near terms are present or the
bound cannot be certified, it stops and lets the exact integrator continue.

This optimization changes representation and stepping cost, not the physical
field contract.

## 23.6. Validation and benchmarks

Tests cover:

- scalar versus patch evaluation;
- exact-near terms and source hierarchy openings;
- random geometry error envelopes;
- deterministic ordered and parallel batches;
- cohort splitting before a bound is violated;
- field-window reuse and invalidation;
- affine propagation against an independent RK4 frozen-field oracle;
- piecewise fallback near bodies and with zero budget.

Selected benchmarks:

```bash
cargo bench -p thessa-sim-core --bench flock
cargo bench -p thessa-sim-core --bench fleet_prop
cargo bench -p thessa-sim-core --bench fleet_validate
cargo bench -p thessa-sim-core --bench planner_batch
cargo bench -p thessa-sim-core --bench affine_prop
```

Fleet numbers are workload-dependent. Any future performance claim must include
target count, source count, patch configuration, exact-path comparison, and an
absolute error envelope.

## 23.7. Non-goals

This layer does not implement SOI switching, patched-conic authority, GPU-only
correctness, ship-to-ship gravity shortcuts, or a craft-class-specific physics
mode. GPU/terrain work may reuse the representation later, but the CPU path
remains canonical.
