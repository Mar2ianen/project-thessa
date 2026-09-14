//! Analytic propagation inside a frozen affine gravity field
//! (docs/23_GRAVITY_FIELD_COHORTS.md companion design note
//! `thessa_affine_gravity_analytic_propagation.md`).
//!
//! Once `g0`/`J` are frozen, motion is linear with constant coefficients:
//! `x'' = Jx + c` with `c = g0 - Jx0`. The tidal tensor of point-mass
//! gravity is symmetric (hence diagonalizable by an orthogonal eigenbasis)
//! and traceless (so a vacuum patch always has a hyperbolic direction —
//! never pure oscillation). Each eigenmode integrates in closed form
//! (oscillatory / hyperbolic / drift), which removes the generic 6x6 matrix
//! exponential from the hot path: one candidate propagation is a few dot
//! products plus three independent 2x2 mode updates.
//!
//! The absolute form subsumes the relative (STM) form: propagating
//! deviations `(dx, dv)` is the same code path with `c = 0`. No matrix
//! inverse appears anywhere (J is singular in general), and near-zero
//! eigenvalues take a Taylor branch keyed on `|lambda| dt^2`.

use glam::{DMat3, DVec3};

use crate::{
    BakedEphemeris, BodyId, CohortConfig, EphemerisError, EphemerisFrame, PatchError, SimTime,
    gravity_patch::{affine_segment_bound, compile_patch_at},
};

/// Switch to the Taylor branch when `|lambda| dt^2` is this small: the
/// direct sin/sinh formulas lose nothing yet, but the Taylor remainder
/// (~s^2/24) is already far below floating-point noise, and the branch
/// structurally excludes any 0/0 edge.
const TAYLOR_S_THRESHOLD: f64 = 1.0e-8;
/// Refuse coefficients past this `sigma dt`: cosh/sinh would overflow, and
/// — more importantly — no patch should be trusted over dozens of
/// e-foldings. The validity controller must cut the segment first; this is
/// the backstop, and it fails open.
const MAX_SIGMA_DT: f64 = 50.0;
/// Fixed Jacobi sweep count: 3x3 converges in far fewer, and a fixed count
/// keeps the eigensolve exactly reproducible for identical inputs.
const JACOBI_SWEEPS: usize = 12;

/// Compiled local propagator: orthonormal eigenbasis of `J` (columns) plus
/// eigenvalues. Compile once per patch, propagate many candidates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AffinePropagator {
    basis: DMat3,
    lambda: DVec3,
}

/// Per-`dt` scalar coefficients for one eigenmode:
/// `q1 = c_pos*q0 + c_vel*u0 + c_cst*c_q`,
/// `u1 = v_pos*q0 + v_vel*u0 + v_cst*c_q`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ModeCoefficients {
    pub c_pos: f64,
    pub c_vel: f64,
    pub c_cst: f64,
    pub v_pos: f64,
    pub v_vel: f64,
    pub v_cst: f64,
}

/// Step coefficients for all three modes at one `dt`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StepCoefficients {
    pub modes: [ModeCoefficients; 3],
}

impl AffinePropagator {
    /// Compile from a tidal tensor. Requires finite, symmetric `J`
    /// (asymmetric input is rejected, never symmetrized silently).
    pub fn compile(jacobian: DMat3) -> Result<Self, PropagatorError> {
        if !jacobian.is_finite() {
            return Err(PropagatorError::NonFiniteJacobian);
        }
        let skew = jacobian - jacobian.transpose();
        let scale = max_abs_entry(jacobian).max(1.0e-300);
        if max_abs_entry(skew) > 1.0e-12 * scale {
            return Err(PropagatorError::AsymmetricJacobian);
        }
        let (mut basis, mut lambda) = jacobi_eigen(jacobian);
        sort_modes(&mut basis, &mut lambda);
        canonicalize_signs(&mut basis);
        if !basis.is_finite() || !lambda.is_finite() {
            return Err(PropagatorError::NonFiniteJacobian);
        }
        Ok(Self { basis, lambda })
    }

    pub fn eigenvalues(&self) -> DVec3 {
        self.lambda
    }

    pub fn basis(&self) -> DMat3 {
        self.basis
    }

    /// Coefficients for one frozen interval. Fails open past the overflow
    /// backstop or on non-finite steps.
    pub fn coefficients(&self, dt: f64) -> Result<StepCoefficients, PropagatorError> {
        if !dt.is_finite() || dt < 0.0 {
            return Err(PropagatorError::NonFiniteStep);
        }
        let lambda = [self.lambda.x, self.lambda.y, self.lambda.z];
        Ok(StepCoefficients {
            modes: [
                mode_coefficients(lambda[0], dt)?,
                mode_coefficients(lambda[1], dt)?,
                mode_coefficients(lambda[2], dt)?,
            ],
        })
    }

    /// Propagate anchor-relative `(displacement, velocity)` under
    /// `x'' = Jx + c` over the interval baked into `coeffs`, returning the
    /// new displacement (caller adds the anchor back exactly once).
    ///
    /// Formulations that must not be mixed: with anchor-relative state
    /// `q = Q^T (x - anchor)` the constant is `Q^T g0` — i.e. callers pass
    /// `constant = g0`, never `g0 - J*anchor` (that constant belongs to the
    /// origin-anchored form `q = Q^T x`, which reintroduces the 1e9-scale
    /// fp floor this API exists to avoid).
    ///
    /// Displacement in/out is load-bearing precision design, not just API
    /// shape: transforming absolute 1e9-scale positions through the basis
    /// every segment floors accuracy at ~1e-6 m per segment, which defeats
    /// budget tightening (measured: tighter budgets landed FARTHER). With
    /// anchor-relative state the floor drops to excursion scale (~1e-10 m
    /// here) and convergence is monotone in the budget again. Relative
    /// (STM) propagation is the same call with `c = DVec3::ZERO`.
    pub fn propagate(
        &self,
        coeffs: &StepCoefficients,
        displacement: DVec3,
        velocity: DVec3,
        constant: DVec3,
    ) -> (DVec3, DVec3) {
        let inverse = self.basis.transpose();
        let q0 = inverse * displacement;
        let u0 = inverse * velocity;
        let cq = inverse * constant;
        let q0 = [q0.x, q0.y, q0.z];
        let u0 = [u0.x, u0.y, u0.z];
        let cq = [cq.x, cq.y, cq.z];
        // Manual unroll keeps lane order explicit and deterministic.
        let mut q_arr = [0.0; 3];
        let mut u_arr = [0.0; 3];
        for i in 0..3 {
            let mode = &coeffs.modes[i];
            q_arr[i] = mode.c_pos * q0[i] + mode.c_vel * u0[i] + mode.c_cst * cq[i];
            u_arr[i] = mode.v_pos * q0[i] + mode.v_vel * u0[i] + mode.v_cst * cq[i];
        }
        let q1 = DVec3::from_array(q_arr);
        let u1 = DVec3::from_array(u_arr);
        (self.basis * q1, self.basis * u1)
    }
}

fn mode_coefficients(lambda: f64, dt: f64) -> Result<ModeCoefficients, PropagatorError> {
    let s = lambda * dt * dt;
    if !s.is_finite() {
        return Err(PropagatorError::NonFiniteStep);
    }
    // Taylor branch: second order in s is exact to ~s^2/24 <= 1e-18 here.
    if s.abs() < TAYLOR_S_THRESHOLD {
        let t2 = dt * dt;
        let t3 = t2 * dt;
        return Ok(ModeCoefficients {
            c_pos: 1.0 + s / 2.0,
            c_vel: dt + lambda * t3 / 6.0,
            c_cst: t2 / 2.0 + lambda * t2 * t2 / 24.0,
            v_pos: lambda * dt + lambda * lambda * t3 / 6.0,
            v_vel: 1.0 + s / 2.0,
            v_cst: dt + lambda * t3 / 6.0,
        });
    }
    if lambda < 0.0 {
        let omega = (-lambda).sqrt();
        let phase = omega * dt;
        // phase^2 = |s| >= threshold: no 0/0 edge by construction.
        Ok(ModeCoefficients {
            c_pos: phase.cos(),
            c_vel: phase.sin() / omega,
            c_cst: -(1.0 - phase.cos()) / lambda,
            v_pos: -omega * phase.sin(),
            v_vel: phase.cos(),
            v_cst: phase.sin() / omega,
        })
    } else {
        let sigma = lambda.sqrt();
        let argument = sigma * dt;
        if argument > MAX_SIGMA_DT {
            return Err(PropagatorError::IntervalTooLong);
        }
        Ok(ModeCoefficients {
            c_pos: argument.cosh(),
            c_vel: argument.sinh() / sigma,
            c_cst: -(1.0 - argument.cosh()) / lambda,
            v_pos: sigma * argument.sinh(),
            v_vel: argument.cosh(),
            v_cst: argument.sinh() / sigma,
        })
    }
}

fn get(matrix: &DMat3, row: usize, col: usize) -> f64 {
    matrix.col(col)[row]
}

/// Largest absolute entry of a 3x3 (glam has no `max_element` on mat3).
fn max_abs_entry(matrix: DMat3) -> f64 {
    matrix
        .col(0)
        .abs()
        .max(matrix.col(1).abs())
        .max(matrix.col(2).abs())
        .max_element()
}

fn set(matrix: &mut DMat3, row: usize, col: usize, value: f64) {
    matrix.col_mut(col)[row] = value;
}

/// Cyclic Jacobi eigensolve for symmetric 3x3: fixed sweep count, fixed
/// pair order — bit-reproducible for identical inputs.
fn jacobi_eigen(mut a: DMat3) -> (DMat3, DVec3) {
    let mut q = DMat3::IDENTITY;
    for _ in 0..JACOBI_SWEEPS {
        for (p, r) in [(0, 1), (0, 2), (1, 2)] {
            let apq = get(&a, p, r);
            if apq == 0.0 {
                continue;
            }
            let app = get(&a, p, p);
            let aqq = get(&a, r, r);
            let theta = (aqq - app) / (2.0 * apq);
            let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            for k in 0..3 {
                let akp = get(&a, k, p);
                let akq = get(&a, k, r);
                set(&mut a, k, p, c * akp - s * akq);
                set(&mut a, k, r, s * akp + c * akq);
            }
            for k in 0..3 {
                let apk = get(&a, p, k);
                let aqk = get(&a, r, k);
                set(&mut a, p, k, c * apk - s * aqk);
                set(&mut a, r, k, s * apk + c * aqk);
            }
            for k in 0..3 {
                let qkp = get(&q, k, p);
                let qkq = get(&q, k, r);
                set(&mut q, k, p, c * qkp - s * qkq);
                set(&mut q, k, r, s * qkp + c * qkq);
            }
        }
    }
    (q, DVec3::new(get(&a, 0, 0), get(&a, 1, 1), get(&a, 2, 2)))
}

/// Sort modes descending by eigenvalue (selection order fixed); signs
/// canonicalized so the largest-abs component of each eigenvector is
/// positive — same input, same basis, no platform-dependent flips.
fn sort_modes(basis: &mut DMat3, lambda: &mut DVec3) {
    let mut order = [0, 1, 2];
    order.sort_by(|a, b| lambda[*b].total_cmp(&lambda[*a]));
    let sorted_basis = DMat3::from_cols(
        basis.col(order[0]),
        basis.col(order[1]),
        basis.col(order[2]),
    );
    let sorted_lambda = DVec3::new(lambda[order[0]], lambda[order[1]], lambda[order[2]]);
    *basis = sorted_basis;
    *lambda = sorted_lambda;
}

fn canonicalize_signs(basis: &mut DMat3) {
    for col in 0..3 {
        let vector = basis.col(col);
        let components = [vector.x, vector.y, vector.z];
        let mut pivot = 0;
        for i in 1..3 {
            if components[i].abs() > components[pivot].abs() {
                pivot = i;
            }
        }
        if components[pivot] < 0.0 {
            *basis.col_mut(col) = -vector;
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PropagatorError {
    NonFiniteJacobian,
    AsymmetricJacobian,
    NonFiniteStep,
    IntervalTooLong,
}

impl std::fmt::Display for PropagatorError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NonFiniteJacobian => write!(formatter, "non-finite tidal tensor"),
            Self::AsymmetricJacobian => write!(formatter, "tidal tensor is not symmetric"),
            Self::IntervalTooLong => write!(
                formatter,
                "hyperbolic interval past the overflow backstop; cut the segment"
            ),
            Self::NonFiniteStep => write!(formatter, "non-finite propagation step"),
        }
    }
}

impl std::error::Error for PropagatorError {}

/// Floor for the compile ball: prevents degenerate zero-radius acceptance
/// (whose bound is trivially zero) and the grind it would cause.
const MIN_BALL_RADIUS_M: f64 = 1_000.0;
/// Analytic segments below one physics tick buy nothing over exact
/// stepping: hand the remainder to the exact integrator instead.
const MIN_SEGMENT_DT_S: f64 = 0.5;
/// Sanity cap; the temporal bound normally binds far earlier.
const MAX_SEGMENT_DT_S: f64 = 3_600.0;

/// One accepted analytic segment endpoint.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AnalyticStep {
    pub time_s: f64,
    pub position: DVec3,
    pub velocity: DVec3,
    pub dt_s: f64,
    /// Posted position bound for this segment (m).
    pub bound_m: f64,
}

/// Why piecewise propagation stopped early (fail-open: the caller continues
/// with exact integration from the last accepted step).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AnalyticFallback {
    /// An exact-near source entered the segment ball (near-body regime:
    /// analytic far-field does not apply).
    ExactNear { body: BodyId },
    /// Adaptive shrink reached the physics-tick floor.
    DtFloor,
}

/// Piecewise report: `steps[0]` is the initial state (dt 0, bound 0).
#[derive(Debug, Clone, PartialEq)]
pub struct PiecewiseReport {
    pub steps: Vec<AnalyticStep>,
    pub fallback: Option<AnalyticFallback>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum AnalyticError {
    InvalidInput,
    Patch(PatchError),
    Propagator(PropagatorError),
    Ephemeris(EphemerisError),
}

impl std::fmt::Display for AnalyticError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidInput => write!(formatter, "invalid analytic propagation input"),
            Self::Patch(error) => write!(formatter, "patch error: {error}"),
            Self::Propagator(error) => write!(formatter, "propagator error: {error}"),
            Self::Ephemeris(error) => write!(formatter, "ephemeris error: {error}"),
        }
    }
}

impl std::error::Error for AnalyticError {}

impl From<PatchError> for AnalyticError {
    fn from(error: PatchError) -> Self {
        Self::Patch(error)
    }
}

impl From<PropagatorError> for AnalyticError {
    fn from(error: PropagatorError) -> Self {
        Self::Propagator(error)
    }
}

impl From<EphemerisError> for AnalyticError {
    fn from(error: EphemerisError) -> Self {
        Self::Ephemeris(error)
    }
}

enum Attempt {
    Accept(AnalyticStep),
    Shrink,
    Fallback(AnalyticFallback),
}

/// Piecewise-analytic propagation (design note §6): compile a patch at the
/// trajectory point, propagate analytically while the posted bound holds,
/// rebuild on expiry — the adaptive-integrator shape with analytic inner
/// segments. Regime A only (far-only patches); near-body encounters and
/// sub-tick segments fail open via `fallback`, never silently.
///
/// `budget_mps2` is the same field-error currency as the cohort system;
/// the reported per-segment `bound_m` converts it with `dt^2/2` (valid:
/// sigma * dt << 1 on every accepted segment — a segment that violated it
/// would blow the field bound first... see `affine_segment_bound`).
#[allow(clippy::too_many_arguments)]
pub fn propagate_piecewise(
    ephemeris: &BakedEphemeris,
    frame: &mut EphemerisFrame,
    initial_position: DVec3,
    initial_velocity: DVec3,
    start: SimTime,
    config: CohortConfig,
    horizon_s: f64,
    dt_initial_s: f64,
) -> Result<PiecewiseReport, AnalyticError> {
    config.validate().map_err(PatchError::Gravity)?;
    if !initial_position.is_finite()
        || !initial_velocity.is_finite()
        || !start.0.is_finite()
        || !horizon_s.is_finite()
        || horizon_s <= 0.0
        || !dt_initial_s.is_finite()
    {
        return Err(AnalyticError::InvalidInput);
    }
    let mut steps = vec![AnalyticStep {
        time_s: start.0,
        position: initial_position,
        velocity: initial_velocity,
        dt_s: 0.0,
        bound_m: 0.0,
    }];
    let mut time_s = start.0;
    let mut position = initial_position;
    let mut velocity = initial_velocity;
    let mut dt = dt_initial_s.clamp(MIN_SEGMENT_DT_S, MAX_SEGMENT_DT_S);
    if !dt.is_finite() || dt <= 0.0 {
        return Err(AnalyticError::InvalidInput);
    }
    while time_s < start.0 + horizon_s {
        let remaining = start.0 + horizon_s - time_s;
        let mut dt_try = dt.min(remaining);
        loop {
            match attempt_segment(ephemeris, frame, position, velocity, time_s, config, dt_try)? {
                Attempt::Accept(step) => {
                    time_s = step.time_s;
                    position = step.position;
                    velocity = step.velocity;
                    steps.push(step);
                    dt = (dt_try * 1.5).min(MAX_SEGMENT_DT_S);
                    break;
                }
                Attempt::Shrink => {
                    if dt_try <= MIN_SEGMENT_DT_S {
                        return Ok(PiecewiseReport {
                            steps,
                            fallback: Some(AnalyticFallback::DtFloor),
                        });
                    }
                    dt_try /= 2.0;
                }
                Attempt::Fallback(reason) => {
                    return Ok(PiecewiseReport {
                        steps,
                        fallback: Some(reason),
                    });
                }
            }
        }
    }
    Ok(PiecewiseReport {
        steps,
        fallback: None,
    })
}

fn attempt_segment(
    ephemeris: &BakedEphemeris,
    frame: &mut EphemerisFrame,
    position: DVec3,
    velocity: DVec3,
    time_s: f64,
    config: CohortConfig,
    dt: f64,
) -> Result<Attempt, AnalyticError> {
    if !dt.is_finite() || dt <= 0.0 || !position.is_finite() || !velocity.is_finite() {
        return Err(AnalyticError::InvalidInput);
    }
    let states = frame
        .evaluate(ephemeris, SimTime(time_s))
        .map_err(AnalyticError::Ephemeris)?
        .to_vec();
    let speed = velocity.length();
    if !speed.is_finite() {
        return Err(AnalyticError::InvalidInput);
    }
    // Predicted ball covers the linear drift plus a floor: exact-near
    // classification then sees the real segment ball, and a post-hoc
    // excursion check keeps the acceptance honest.
    let radius_m = (speed * dt * 1.25 + MIN_BALL_RADIUS_M).max(MIN_BALL_RADIUS_M);
    if !radius_m.is_finite() {
        return Err(AnalyticError::InvalidInput);
    }
    let patch = compile_patch_at(ephemeris, &states, position, radius_m, config)?;
    if !patch.exact.is_empty() {
        // Genuinely near (the ball reaches a source) → hand to exact
        // integration at once. Otherwise the ball is merely too big for
        // the budget (single-source overflow) → shrink it; that converges
        // because contributions scale with r^2.
        let mut nearest = BodyId(0);
        let mut nearest_distance = f64::INFINITY;
        for source in ephemeris.gravity_sources() {
            let state = states
                .get(source.id.index())
                .ok_or(PatchError::UnknownSource(source.id))?;
            let distance = (state.position_inertial - position).length();
            if distance < nearest_distance {
                nearest_distance = distance;
                nearest = source.id;
            }
        }
        if !nearest_distance.is_finite() {
            return Err(AnalyticError::InvalidInput);
        }
        if nearest_distance <= radius_m {
            return Ok(Attempt::Fallback(AnalyticFallback::ExactNear {
                body: nearest,
            }));
        }
        return Ok(Attempt::Shrink);
    }
    let propagator = AffinePropagator::compile(patch.jacobian)?;
    let coeffs = match propagator.coefficients(dt) {
        Ok(coeffs) => coeffs,
        Err(PropagatorError::IntervalTooLong) => return Ok(Attempt::Shrink),
        Err(other) => return Err(AnalyticError::Propagator(other)),
    };
    let constant = patch.g0;
    let (delta_position, end_velocity) =
        propagator.propagate(&coeffs, DVec3::ZERO, velocity, constant);
    let end_position = position + delta_position;
    if !end_position.is_finite() || !end_velocity.is_finite() {
        return Ok(Attempt::Shrink);
    }
    // Excursion over sub-samples, including the endpoint.
    let mut excursion_m = 0.0_f64;
    for quarter in 1..=4 {
        let sub = propagator.coefficients(dt * quarter as f64 / 4.0)?;
        let (delta, _) = propagator.propagate(&sub, DVec3::ZERO, velocity, constant);
        excursion_m = excursion_m.max(delta.length());
    }
    if excursion_m.is_nan() || excursion_m > radius_m {
        return Ok(Attempt::Shrink);
    }
    let field_bound = affine_segment_bound(ephemeris, &states, &patch, excursion_m, dt)?;
    if !field_bound.is_finite() {
        return Ok(Attempt::Shrink);
    }
    if field_bound > config.error_budget_mps2 {
        return Ok(Attempt::Shrink);
    }
    Ok(Attempt::Accept(AnalyticStep {
        time_s: time_s + dt,
        position: end_position,
        velocity: end_velocity,
        dt_s: dt,
        bound_m: field_bound * dt * dt / 2.0,
    }))
}
