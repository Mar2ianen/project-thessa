//! Local gravity patches shared by target cohorts (docs/23_GRAVITY_FIELD_COHORTS.md
//! sections 6-9, implementation steps 6-9).
//!
//! Distant sources form a smooth field, so one expensive compilation per
//! cohort — exact acceleration `g0` plus the tidal tensor `J` at an anchor —
//! serves every nearby target through a few FMA:
//!
//! ```text
//! g(x) ~= g0 + J * (x - x0) + sum over exact-near sources.
//! ```
//!
//! Error analysis (point-mass Hessian, Frobenius norm derived in
//! [`HESSIAN_FROBENIUS_NORM`]): the affine remainder over a ball of radius
//! `r` around the anchor is bounded by
//!
//! ```text
//! |R| <= HESSIAN_REMAINDER * mu * r^2 / (D - r)^4 ,  D > r,
//! ```
//!
//! summed over absorbed far sources. Near sources (inside the ball, or
//! closer than `near_open_factor * r`) stay exact per target, so local
//! encounter fidelity never depends on the approximation. A cohort whose
//! total bound exceeds the budget splits (median cut along the longest
//! axis); cohorts of two or fewer targets evaluate exactly.
//!
//! v1 compiles `g0`/`J` exactly once per tick (single-tick validity).
//! Multi-tick reuse with a motion bound, tree-opened compilation, and the
//! quadrupole ladder are documented in the architecture doc as follow-ups,
//! not part of this layer.

use glam::{DMat3, DVec3};
use rayon::prelude::*;

use crate::{BakedEphemeris, BodyId, BodyState, GravityError};

/// Exact Frobenius norm of the point-mass Hessian (third-derivative tensor
/// of `mu*y/|y|^3`), in units of `mu/r^4`: `3*sqrt(10) ~= 9.4868`.
/// Derivation: with `n = y/r`,
/// `T_ijk = (mu/r^4)[3(d_ij n_k + d_ik n_j + d_jk n_i) - 15 n_i n_j n_k]`,
/// and `||T||_F^2 = (mu/r^4)^2 (9*15 - 90*3 + 225*1) = 90 (mu/r^4)^2`.
/// Pinned numerically by `hessian_norm_matches_closed_form` in tests.
pub const HESSIAN_FROBENIUS_NORM: f64 = 9.486_832_980_505_138;
/// Affine remainder prefactor: `HESSIAN_FROBENIUS_NORM / 2`.
pub const HESSIAN_REMAINDER: f64 = 4.743_416_490_252_569;

/// Cohort policy. Every simplification has its explicit boundary here, not
/// as a magic number in code (AGENTS.md section 10.1).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CohortConfig {
    /// Allocated gravity error budget per target (m/s^2, absolute).
    pub error_budget_mps2: f64,
    /// Sources closer than `near_open_factor * cohort_radius` stay exact.
    pub near_open_factor: f64,
    /// Cohort split recursion guard.
    pub max_depth: u32,
}

impl Default for CohortConfig {
    fn default() -> Self {
        Self {
            error_budget_mps2: 1.0e-9,
            near_open_factor: 10.0,
            max_depth: 16,
        }
    }
}

impl CohortConfig {
    fn validate(self) -> Result<(), GravityError> {
        if !self.error_budget_mps2.is_finite()
            || self.error_budget_mps2 < 0.0
            || !self.near_open_factor.is_finite()
            || self.near_open_factor <= 1.0
            || self.max_depth > 64
        {
            return Err(GravityError::NonFinite { body_id: BodyId(0) });
        }
        Ok(())
    }
}

/// One compiled local field: shared far field plus the exact-near list.
#[derive(Debug, Clone)]
pub struct GravityPatch {
    /// Anchor: affine expansion point.
    pub center: DVec3,
    /// Validity ball radius around the anchor (single tick).
    pub radius_m: f64,
    /// Exact acceleration at the anchor from absorbed far sources only.
    pub g0: DVec3,
    /// Exact tidal tensor at the anchor from absorbed far sources only.
    pub jacobian: DMat3,
    /// Near sources evaluated exactly per target: `(body, mu)`.
    pub exact: Vec<(BodyId, f64)>,
    /// Achieved total error bound over the ball (m/s^2).
    pub error_bound_mps2: f64,
}

/// Tidal tensor of one point-mass term: `mu*(3*r*r^T/D^5 - I/D^3)`.
fn tidal_tensor(offset: DVec3, mu: f64, distance: f64) -> DMat3 {
    let outer = DMat3::from_cols(offset * offset.x, offset * offset.y, offset * offset.z);
    (outer * (3.0 / distance.powi(5)) - DMat3::IDENTITY * (1.0 / distance.powi(3))) * mu
}

/// One far source carried through compilation and reuse checks.
#[derive(Debug, Clone, Copy)]
struct FarSource {
    body: BodyId,
    mu: f64,
    offset: DVec3,
    distance: f64,
}

/// Split sources into exact-near and absorbed-far for one anchor ball.
/// Pure function of (states, anchor, radius, config): shared by fresh
/// compilation and reuse validation, so both agree on membership.
#[derive(Debug)]
struct SourceClasses {
    exact: Vec<(BodyId, f64)>,
    far: Vec<FarSource>,
}

fn classify_sources(
    ephemeris: &BakedEphemeris,
    states: &[BodyState],
    anchor: DVec3,
    radius_m: f64,
    config: CohortConfig,
) -> Result<SourceClasses, PatchError> {
    let mut exact = Vec::new();
    let mut far = Vec::new();
    for source in ephemeris.gravity_sources() {
        let state = states
            .get(source.id.index())
            .ok_or(PatchError::UnknownSource(source.id))?;
        let offset = state.position_inertial - anchor;
        let distance_squared = offset.length_squared();
        if !distance_squared.is_finite() {
            return Err(PatchError::Gravity(GravityError::NonFinite {
                body_id: source.id,
            }));
        }
        if distance_squared == 0.0 {
            return Err(PatchError::Gravity(GravityError::Singularity {
                body_id: source.id,
            }));
        }
        let distance = distance_squared.sqrt();
        // Mandatory exact: the validity ball reaches the source, so no
        // positive clearance exists for a remainder bound.
        if distance <= radius_m || distance < config.near_open_factor * radius_m {
            exact.push((source.id, source.mu));
            continue;
        }
        let clearance = distance - radius_m;
        let contribution = HESSIAN_REMAINDER * source.mu * radius_m.powi(2) / clearance.powi(4);
        // A lone source that already busts the whole budget goes exact
        // instead of forcing the cohort to split down to metre balls:
        // splitting is for spread-out groups in a smooth field, not for a
        // dominant near host (doc 23 section 7).
        if contribution > config.error_budget_mps2 {
            exact.push((source.id, source.mu));
            continue;
        }
        far.push(FarSource {
            body: source.id,
            mu: source.mu,
            offset,
            distance,
        });
    }
    exact.sort_by_key(|(body, _)| body.0);
    Ok(SourceClasses { exact, far })
}

/// Compile one patch over `positions` (non-empty): bounding ball anchor,
/// exact-near classification, exact `g0`/`J` over far sources, total bound.
/// Fails open (caller falls back) only on non-finite input, never silently.
pub fn compile_patch(
    ephemeris: &BakedEphemeris,
    states: &[BodyState],
    positions: &[DVec3],
    config: CohortConfig,
) -> Result<GravityPatch, PatchError> {
    config.validate().map_err(PatchError::Gravity)?;
    let anchor = positions
        .iter()
        .fold(DVec3::ZERO, |sum, position| sum + *position)
        / positions.len() as f64;
    if !anchor.is_finite() {
        return Err(PatchError::NonFiniteInput);
    }
    let mut radius_m = 0.0_f64;
    for position in positions {
        if !position.is_finite() {
            return Err(PatchError::NonFiniteInput);
        }
        radius_m = radius_m.max((*position - anchor).length());
    }
    if !radius_m.is_finite() {
        return Err(PatchError::NonFiniteInput);
    }
    let classes = classify_sources(ephemeris, states, anchor, radius_m, config)?;
    let mut g0 = DVec3::ZERO;
    let mut jacobian = DMat3::ZERO;
    let mut error_bound_mps2 = 0.0;
    for source in &classes.far {
        let inverse = source.distance.recip();
        if !inverse.is_finite() {
            return Err(PatchError::Gravity(GravityError::NonFinite {
                body_id: source.body,
            }));
        }
        g0 += source.offset * (source.mu * inverse.powi(3));
        jacobian += tidal_tensor(source.offset, source.mu, source.distance);
        let clearance = source.distance - radius_m;
        error_bound_mps2 += HESSIAN_REMAINDER * source.mu * radius_m.powi(2) / clearance.powi(4);
    }
    if !g0.is_finite()
        || !jacobian.is_finite()
        || !error_bound_mps2.is_finite()
        || error_bound_mps2 < 0.0
    {
        return Err(PatchError::NonFiniteInput);
    }
    Ok(GravityPatch {
        center: anchor,
        radius_m,
        g0,
        jacobian,
        exact: classes.exact,
        error_bound_mps2,
    })
}

/// Evaluate one target through a patch: shared affine far field plus exact
/// near terms. Deterministic: summation order is patch order, independent
/// of worker count.
pub fn evaluate_patch(
    patch: &GravityPatch,
    states: &[BodyState],
    position: DVec3,
) -> Result<DVec3, GravityError> {
    let mut total = patch.g0 + patch.jacobian * (position - patch.center);
    for (body, mu) in &patch.exact {
        let state = states
            .get(body.index())
            .ok_or(crate::EphemerisError::UnknownBody(*body))?;
        let offset = state.position_inertial - position;
        let distance_squared = offset.length_squared();
        if !distance_squared.is_finite() {
            return Err(GravityError::NonFinite { body_id: *body });
        }
        if distance_squared == 0.0 {
            return Err(GravityError::Singularity { body_id: *body });
        }
        let inverse_distance = distance_squared.sqrt().recip();
        total += offset * (*mu * inverse_distance.powi(3));
    }
    if total.is_finite() {
        Ok(total)
    } else {
        Err(GravityError::NonFinite {
            body_id: patch
                .exact
                .first()
                .map(|(body, _)| *body)
                .unwrap_or(BodyId(0)),
        })
    }
}

/// Infallible twin of [`evaluate_patch`]: identical operations in identical
/// order, minus the branches. Singular lanes evaluate to NaN and propagate,
/// so callers finite-scan outputs once and rerun only failed lanes
/// fallibly. Bit-identical output wherever the checked path succeeds.
pub fn evaluate_patch_unchecked(
    patch: &GravityPatch,
    states: &[BodyState],
    position: DVec3,
) -> DVec3 {
    let mut total = patch.g0 + patch.jacobian * (position - patch.center);
    for (body, mu) in &patch.exact {
        let offset = states[body.index()].position_inertial - position;
        let distance_squared = offset.length_squared();
        let inverse_distance = distance_squared.sqrt().recip();
        total += offset * (*mu * inverse_distance.powi(3));
    }
    total
}

/// Evaluate one patch over Structure-of-Arrays target slices (doc 23 section
/// 11 target layout): shared anchor/`g0`/`J` broadcast, SoA outputs. The
/// exact-near terms accumulate per lane afterwards through
/// [`evaluate_patch`]-equivalent math; this entry covers the shared affine
/// part, which is the vectorizable bulk.
pub fn evaluate_patch_soa(
    patch: &GravityPatch,
    xs: &[f64],
    ys: &[f64],
    zs: &[f64],
    out_ax: &mut [f64],
    out_ay: &mut [f64],
    out_az: &mut [f64],
) {
    debug_assert_eq!(xs.len(), ys.len());
    debug_assert_eq!(xs.len(), zs.len());
    debug_assert_eq!(xs.len(), out_ax.len());
    debug_assert_eq!(xs.len(), out_ay.len());
    debug_assert_eq!(xs.len(), out_az.len());
    let j = patch.jacobian;
    for lane in 0..xs.len() {
        let dx = xs[lane] - patch.center.x;
        let dy = ys[lane] - patch.center.y;
        let dz = zs[lane] - patch.center.z;
        out_ax[lane] = patch.g0.x + j.x_axis.x * dx + j.y_axis.x * dy + j.z_axis.x * dz;
        out_ay[lane] = patch.g0.y + j.x_axis.y * dx + j.y_axis.y * dy + j.z_axis.y * dz;
        out_az[lane] = patch.g0.z + j.x_axis.z * dx + j.y_axis.z * dy + j.z_axis.z * dz;
    }
}

/// Cohort evaluation report: shared-patch accelerations plus the achieved
/// bound and splitting telemetry.
#[derive(Debug)]
pub struct CohortReport {
    pub accelerations: Vec<DVec3>,
    pub error_bound_mps2: f64,
    pub cohort_count: usize,
    pub split_count: usize,
    /// Total exact-near evaluations (`sum(cohort_targets * exact.len)`):
    /// the per-target exact work the patch did not absorb.
    pub exact_terms: u64,
    /// Largest cohort ball radius (m): grows when the group disperses.
    pub max_radius_m: f64,
}

/// Evaluate all targets through runtime cohorts: build one ball, compile its
/// patch, split while the posted bound exceeds the budget. Order-preserving
/// (input order out), worker-count independent.
pub fn evaluate_cohorts(
    ephemeris: &BakedEphemeris,
    states: &[BodyState],
    positions: &[DVec3],
    config: CohortConfig,
) -> Result<CohortReport, PatchError> {
    config.validate().map_err(PatchError::Gravity)?;
    let mut out: Vec<Option<DVec3>> = vec![None; positions.len()];
    let mut indices: Vec<usize> = (0..positions.len()).collect();
    let mut report = CohortReport {
        accelerations: Vec::new(),
        error_bound_mps2: 0.0,
        cohort_count: 0,
        split_count: 0,
        exact_terms: 0,
        max_radius_m: 0.0,
    };
    split_cohort(
        ephemeris,
        states,
        positions,
        &mut indices,
        &mut out,
        &mut report,
        config,
        0,
    )?;
    report.accelerations = out
        .into_iter()
        .map(|slot| slot.expect("every target evaluated"))
        .collect();
    Ok(report)
}

#[allow(clippy::too_many_arguments)]
fn split_cohort(
    ephemeris: &BakedEphemeris,
    states: &[BodyState],
    positions: &[DVec3],
    indices: &mut [usize],
    out: &mut [Option<DVec3>],
    report: &mut CohortReport,
    config: CohortConfig,
    depth: u32,
) -> Result<(), PatchError> {
    if indices.is_empty() {
        return Ok(());
    }
    // Tiny cohorts evaluate exactly: no anchor, no remainder, zero error.
    if indices.len() <= 2 || depth >= config.max_depth {
        for index in indices.iter() {
            out[*index] = Some(evaluate_exact_all(ephemeris, states, positions[*index])?);
        }
        report.cohort_count += 1;
        report.exact_terms += (indices.len() * ephemeris.gravity_sources().count()) as u64;
        return Ok(());
    }
    let group: Vec<DVec3> = indices.iter().map(|index| positions[*index]).collect();
    let patch = compile_patch(ephemeris, states, &group, config)?;
    report.max_radius_m = report.max_radius_m.max(patch.radius_m);
    report.exact_terms += (group.len() * patch.exact.len()) as u64;
    if patch.error_bound_mps2 <= config.error_budget_mps2 {
        let mut accelerations = vec![DVec3::ZERO; group.len()];
        eval_patch_batch(&patch, states, &group, &mut accelerations)?;
        for (slot, acceleration) in indices.iter().zip(accelerations) {
            out[*slot] = Some(acceleration);
        }
        report.cohort_count += 1;
        report.error_bound_mps2 = report.error_bound_mps2.max(patch.error_bound_mps2);
        return Ok(());
    }
    // Bound violated: median split along the longest bounding-box axis.
    let (mut min, mut max) = (group[0], group[0]);
    for position in &group[1..] {
        min = min.min(*position);
        max = max.max(*position);
    }
    let extent = max - min;
    let axis = if extent.x >= extent.y && extent.x >= extent.z {
        0
    } else if extent.y >= extent.z {
        1
    } else {
        2
    };
    let mut ordered = indices.to_owned();
    let coordinate = |index: usize| match axis {
        0 => positions[index].x,
        1 => positions[index].y,
        _ => positions[index].z,
    };
    ordered.sort_by(|a, b| coordinate(*a).total_cmp(&coordinate(*b)).then(a.cmp(b)));
    // Degenerate split (all equal on the axis): evaluate exactly rather
    // than recursing forever on a zero-radius ball with a blown bound.
    let half = ordered.len() / 2;
    if half == 0 || coordinate(ordered[half - 1]) == coordinate(ordered[half]) {
        for index in indices.iter() {
            out[*index] = Some(evaluate_exact_all(ephemeris, states, positions[*index])?);
        }
        report.cohort_count += 1;
        report.exact_terms += (indices.len() * ephemeris.gravity_sources().count()) as u64;
        return Ok(());
    }
    report.split_count += 1;
    let mut right = ordered.split_off(half);
    let mut left = ordered;
    split_cohort(
        ephemeris,
        states,
        positions,
        &mut left,
        out,
        report,
        config,
        depth + 1,
    )?;
    split_cohort(
        ephemeris,
        states,
        positions,
        &mut right,
        out,
        report,
        config,
        depth + 1,
    )?;
    Ok(())
}

/// One chunked batch through a patch into an aligned output slice: one
/// chunk per worker (same granularity argument as the framed batch) with an
/// infallible kernel; finite-scan once, rerun only failed lanes fallibly
/// for their exact error identity.
fn eval_patch_batch(
    patch: &GravityPatch,
    states: &[BodyState],
    group: &[DVec3],
    out: &mut [DVec3],
) -> Result<(), PatchError> {
    debug_assert_eq!(group.len(), out.len());
    let workers = rayon::current_num_threads().max(1);
    let chunk = group.len().div_ceil(workers).max(1);
    out.par_chunks_mut(chunk)
        .zip(group.par_chunks(chunk))
        .for_each(|(out_block, pos_block)| {
            for (slot, position) in out_block.iter_mut().zip(pos_block.iter()) {
                *slot = evaluate_patch_unchecked(patch, states, *position);
            }
        });
    for (position, acceleration) in group.iter().zip(out.iter_mut()) {
        if !acceleration.is_finite() {
            *acceleration = evaluate_patch(patch, states, *position)?;
        }
    }
    Ok(())
}

/// Exact all-source accumulation for tiny/degenerate cohorts: same checks
/// and summation order as [`GravityField`](crate::GravityField).
fn evaluate_exact_all(
    ephemeris: &BakedEphemeris,
    states: &[BodyState],
    position: DVec3,
) -> Result<DVec3, PatchError> {
    let mut total = DVec3::ZERO;
    for source in ephemeris.gravity_sources() {
        let state = states
            .get(source.id.index())
            .ok_or(PatchError::UnknownSource(source.id))?;
        let offset = state.position_inertial - position;
        let distance_squared = offset.length_squared();
        if !distance_squared.is_finite() {
            return Err(PatchError::Gravity(GravityError::NonFinite {
                body_id: source.id,
            }));
        }
        if distance_squared == 0.0 {
            return Err(PatchError::Gravity(GravityError::Singularity {
                body_id: source.id,
            }));
        }
        let inverse_distance = distance_squared.sqrt().recip();
        total += offset * (source.mu * inverse_distance.powi(3));
    }
    if total.is_finite() {
        Ok(total)
    } else {
        Err(PatchError::Gravity(GravityError::NonFinite {
            body_id: BodyId(0),
        }))
    }
}

/// Patch/cohort failure: invalid policy, unknown source in a short states
/// slice, or the wrapped exact-path gravity error. Fails open by contract:
///
/// callers fall back to the exact [`GravityField`](crate::GravityField) path.
#[derive(Debug, Clone, PartialEq)]
pub enum PatchError {
    Gravity(GravityError),
    UnknownSource(BodyId),
    NonFiniteInput,
}

impl std::fmt::Display for PatchError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Gravity(error) => write!(formatter, "patch gravity error: {error}"),
            Self::UnknownSource(body) => write!(formatter, "unknown patch source {body:?}"),
            Self::NonFiniteInput => write!(formatter, "non-finite patch input"),
        }
    }
}

impl std::error::Error for PatchError {}

impl From<GravityError> for PatchError {
    fn from(error: GravityError) -> Self {
        Self::Gravity(error)
    }
}

/// Single-cohort reuse window: a compiled patch plus the source positions
/// at compile time (per body slot), so later ticks can bound field
/// staleness from source motion without recompiling.
#[derive(Debug, Clone)]
struct ReuseWindow {
    patch: GravityPatch,
    old_positions: Vec<DVec3>,
    config: CohortConfig,
}

/// Check whether a window still covers `positions` under `config` at the
/// current `states`. All bound-driven, no magic thresholds:
/// membership must agree exactly, then spatial remainder (at the new ball
/// radius) plus temporal staleness (Lipschitz `2/|y|^3` on `y/|y|^3` times
/// per-source displacement) must fit the budget. Returns the new radius
/// and the achieved bound when reusable.
fn reuse_bound(
    window: &ReuseWindow,
    ephemeris: &BakedEphemeris,
    states: &[BodyState],
    positions: &[DVec3],
    config: CohortConfig,
) -> Result<Option<(f64, f64)>, PatchError> {
    let anchor = window.patch.center;
    let mut radius_new = 0.0_f64;
    for position in positions {
        if !position.is_finite() {
            return Err(PatchError::NonFiniteInput);
        }
        radius_new = radius_new.max((*position - anchor).length());
    }
    if !radius_new.is_finite() {
        return Err(PatchError::NonFiniteInput);
    }
    let classes = classify_sources(ephemeris, states, anchor, radius_new, config)?;
    if classes.exact != window.patch.exact {
        return Ok(None);
    }
    let mut temporal = 0.0;
    for source in &classes.far {
        let old = window
            .old_positions
            .get(source.body.index())
            .copied()
            .ok_or(PatchError::UnknownSource(source.body))?;
        let displacement = ((source.offset + anchor) - old).length();
        if !displacement.is_finite() {
            return Err(PatchError::NonFiniteInput);
        }
        let clearance = source.distance - displacement;
        if clearance <= 0.0 {
            return Ok(None);
        }
        temporal += source.mu * 2.0 * displacement / clearance.powi(3);
    }
    let mut spatial = 0.0;
    for source in &classes.far {
        // Far membership guarantees positive clearance (classify routes
        // `distance <= radius` to exact), but guard the division anyway.
        let clearance = source.distance - radius_new;
        if clearance <= 0.0 {
            return Ok(None);
        }
        spatial += HESSIAN_REMAINDER * source.mu * radius_new.powi(2) / clearance.powi(4);
    }
    let total = spatial + temporal;
    if !total.is_finite() {
        return Err(PatchError::NonFiniteInput);
    }
    if total <= config.error_budget_mps2 {
        Ok(Some((radius_new, total)))
    } else {
        Ok(None)
    }
}

/// Cohort evaluation borrowing the evaluator's scratch: same telemetry as
/// [`CohortReport`] but zero-copy on the hot path.
#[derive(Debug)]
pub struct CohortEval<'a> {
    pub accelerations: &'a [DVec3],
    pub error_bound_mps2: f64,
    pub cohort_count: usize,
    pub split_count: usize,
    pub exact_terms: u64,
    pub max_radius_m: f64,
    pub reused_window: bool,
}

/// Stateful hot-loop evaluator (doc 23 section 10, step 10): owns reusable
/// scratch so steady-state ticks allocate nothing, and carries a
/// single-cohort reuse window across ticks while the spatial + temporal
/// remainder bounds hold. Split/multi-cohort ticks rebuild through the
/// stateless path and drop the window.
#[derive(Debug, Default)]
pub struct CohortEvaluator {
    out: Vec<DVec3>,
    window: Option<ReuseWindow>,
    /// Ticks served from the reuse window (telemetry).
    pub reuses: u64,
    /// Ticks that compiled fresh or split (telemetry).
    pub rebuilds: u64,
}

impl CohortEvaluator {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn evaluate(
        &mut self,
        ephemeris: &BakedEphemeris,
        states: &[BodyState],
        positions: &[DVec3],
        config: CohortConfig,
    ) -> Result<CohortEval<'_>, PatchError> {
        config.validate().map_err(PatchError::Gravity)?;
        if positions.is_empty() {
            self.out.clear();
            return Ok(CohortEval {
                accelerations: &[],
                error_bound_mps2: 0.0,
                cohort_count: 0,
                split_count: 0,
                exact_terms: 0,
                max_radius_m: 0.0,
                reused_window: false,
            });
        }
        // Fast path: previous single-cohort window still covers the group.
        if let Some(window) = &self.window
            && window.config == config
            && let Some((radius_new, achieved)) =
                reuse_bound(window, ephemeris, states, positions, config)?
        {
            let exact_len = window.patch.exact.len();
            let patch = &window.patch;
            self.out.clear();
            self.out.resize(positions.len(), DVec3::ZERO);
            eval_patch_batch(patch, states, positions, &mut self.out)?;
            self.reuses += 1;
            return Ok(CohortEval {
                accelerations: &self.out,
                error_bound_mps2: achieved,
                cohort_count: 1,
                split_count: 0,
                exact_terms: (positions.len() * exact_len) as u64,
                max_radius_m: radius_new,
                reused_window: true,
            });
        }
        // Fresh single-cohort attempt: one compile, no split machinery.
        let patch = compile_patch(ephemeris, states, positions, config)?;
        if patch.error_bound_mps2 <= config.error_budget_mps2 {
            self.out.clear();
            self.out.resize(positions.len(), DVec3::ZERO);
            eval_patch_batch(&patch, states, positions, &mut self.out)?;
            let mut old_positions = vec![DVec3::ZERO; ephemeris.bodies.len()];
            for source in ephemeris.gravity_sources() {
                old_positions[source.id.index()] = states
                    .get(source.id.index())
                    .ok_or(PatchError::UnknownSource(source.id))?
                    .position_inertial;
            }
            let radius_m = patch.radius_m;
            let bound = patch.error_bound_mps2;
            let exact_len = patch.exact.len();
            self.window = Some(ReuseWindow {
                patch,
                old_positions,
                config,
            });
            self.rebuilds += 1;
            return Ok(CohortEval {
                accelerations: &self.out,
                error_bound_mps2: bound,
                cohort_count: 1,
                split_count: 0,
                exact_terms: (positions.len() * exact_len) as u64,
                max_radius_m: radius_m,
                reused_window: false,
            });
        }
        // Spread-out group: stateless split path, no window.
        self.window = None;
        self.rebuilds += 1;
        let report = evaluate_cohorts(ephemeris, states, positions, config)?;
        self.out.clear();
        self.out.extend_from_slice(&report.accelerations);
        Ok(CohortEval {
            accelerations: &self.out,
            error_bound_mps2: report.error_bound_mps2,
            cohort_count: report.cohort_count,
            split_count: report.split_count,
            exact_terms: report.exact_terms,
            max_radius_m: report.max_radius_m,
            reused_window: false,
        })
    }
}
