use std::{error::Error, fmt};

use glam::DVec3;
use rayon::prelude::*;

use crate::{BakedEphemeris, BodyId, BodyState, SimTime};

/// Runtime gravity view. The ephemeris remains the sole source of moving-body
/// positions; no SOI switching is performed.
pub struct GravityField<'a> {
    ephemeris: &'a BakedEphemeris,
    source_ids: Vec<BodyId>,
    harmonics: Vec<BodyHarmonics>,
}

/// Relative truncation for degree-2 harmonics: while
/// `(R/r)² · max(|J2|, |C22|)` sits below this, the correction is skipped.
/// Bounded-reduction boundary (§10.1): far-field cost is exactly zero (one
/// multiply per source on the already-known `r²`), near-field keeps the full
/// closed form. Strict-interior points (`r < R`) also stay monopole-only —
/// the external expansion is not valid inside the reference sphere, and that
/// path is bitwise identical to the old loop.
pub const HARMONICS_TRUNCATION: f64 = 1e-9;

/// Precomputed per-source harmonic data. Built once in `from_ephemeris`;
/// the hot loop only reads it. Bodies with `j2 == c22 == 0` are flagged
/// inactive and cost exactly one branch in the loop.
#[derive(Debug, Clone, Copy)]
pub struct BodyHarmonics {
    pub active: bool,
    pub j2: f64,
    pub c22: f64,
    pub ref_radius_sq: f64,
    /// Inertial pole unit vector (body-fixed +Z), from `axial_tilt_rad`
    /// about the engine X axis. Fixed over the bake horizon: pole
    /// precession is not modeled (documented).
    pub pole: DVec3,
    /// Spin rate for the C22 longitude, if the TOML gives a period.
    pub spin_rad_s: Option<f64>,
    pub prime_meridian_rad: f64,
    /// Tidally locked C22 bodies keep their long axis toward the host:
    /// orientation is rebuilt from the host state at eval time.
    pub locked_parent: Option<BodyId>,
}

impl BodyHarmonics {
    /// Derive evaluation params from a baked body. Pole from axial tilt,
    /// spin from the rotation period, locked orientation from the parent.
    pub fn new(body: &crate::BakedBody) -> Self {
        let tilt = body.axial_tilt_rad;
        let pole = DVec3::new(0.0, -tilt.sin(), tilt.cos());
        Self {
            active: body.j2 != 0.0 || body.c22 != 0.0,
            j2: body.j2,
            c22: body.c22,
            ref_radius_sq: body.radius_m * body.radius_m,
            pole,
            spin_rad_s: body
                .rotation_period_s
                .filter(|period| *period > 0.0)
                .map(|period| std::f64::consts::TAU / period),
            prime_meridian_rad: body.prime_meridian_rad,
            locked_parent: (body.tidal_lock && body.c22 != 0.0)
                .then_some(body.parent)
                .flatten(),
        }
    }
}

/// Body-fixed frame axes `(x, y, z)` in inertial coordinates. `x` is the
/// long axis (C22 longitude origin), `z` the pole. Right-handed throughout.
fn harmonic_frame(
    params: &BodyHarmonics,
    time_s: f64,
    locked_host_dir: Option<DVec3>,
) -> (DVec3, DVec3, DVec3) {
    let z = params.pole;
    if let Some(host_dir) = locked_host_dir {
        // Long axis toward the host; re-orthogonalize against the pole.
        let x = (host_dir - z * host_dir.dot(z)).normalize_or_zero();
        let x = if x == DVec3::ZERO {
            orthogonal_to(z)
        } else {
            x
        };
        let y = z.cross(x);
        return (y.cross(z), y, z);
    }
    let x0 = orthogonal_to(z);
    let y0 = z.cross(x0);
    match params.spin_rad_s {
        Some(spin) => {
            let phase = params.prime_meridian_rad + spin * time_s;
            let (sin, cos) = phase.sin_cos();
            (x0 * cos + y0 * sin, y0 * cos - x0 * sin, z)
        }
        // No spin data: triaxial bodies sit in a static frame (documented
        // assumption — currently only Cinder-class shards, which have no
        // measured period yet).
        None => {
            let (sin, cos) = params.prime_meridian_rad.sin_cos();
            (x0 * cos + y0 * sin, y0 * cos - x0 * sin, z)
        }
    }
}

fn orthogonal_to(axis: DVec3) -> DVec3 {
    let reference = if axis.z.abs() < 0.9 {
        DVec3::Z
    } else {
        DVec3::X
    };
    (reference - axis * reference.dot(axis)).normalize_or_zero()
}

/// Truncation gate shared by the acceleration and potential lanes: false
/// means monopole-only (inactive coefficients, interior points, or a
/// far-field contribution below [`HARMONICS_TRUNCATION`]).
fn harmonic_gate(params: &BodyHarmonics, distance_squared: f64) -> bool {
    let magnitude = params.j2.abs().max(params.c22.abs());
    magnitude != 0.0
        && distance_squared >= params.ref_radius_sq
        && (params.ref_radius_sq / distance_squared) * magnitude >= HARMONICS_TRUNCATION
}
/// Degree-2 potential terms (unnormalized J2/C22) at a body-fixed point,
/// for gradient cross-checks and energy bookkeeping. Sign convention is the
/// codebase one (`U` carries the negative monopole, `a = −∇U`) — NOT the
/// geodesy convention: both terms are negated relative to e.g. Vallado.
/// Matches [`harmonic_correction`] term by term.
pub fn harmonic_potential_terms(
    mu: f64,
    ref_radius_m: f64,
    j2: f64,
    c22: f64,
    bf: DVec3,
) -> (f64, f64) {
    let r_sq = bf.length_squared();
    let r = r_sq.sqrt();
    let ref_sq = ref_radius_m * ref_radius_m;
    let u_j2 = mu * j2 * ref_sq * (3.0 * bf.z * bf.z - r_sq) / (2.0 * r_sq * r_sq * r);
    let u_c22 = -3.0 * mu * c22 * ref_sq * (bf.x * bf.x - bf.y * bf.y) / (r_sq * r_sq * r);
    (u_j2, u_c22)
}

/// Degree-2 acceleration correction in the inertial frame.
///
/// `offset` points from the evaluation point toward the body (the monopole
/// loop's convention). Closed form for unnormalized J2/C22 with the
/// body-fixed long axis on x (derivation in the commit notes; equatorial,
/// polar and long/short-axis special cases pinned by unit tests):
///
/// ```text
/// w = μR²/r⁵,  v = 3μC22R²/r⁷
/// ax = 1.5·w·J2·x·(5z²−r²)/r² + v·x·(2r²−5(x²−y²))
/// ay = 1.5·w·J2·y·(5z²−r²)/r² − v·y·(2r²+5(x²−y²))
/// az = −1.5·w·J2·z·(3r²−5z²)/r² − 5·v·z·(x²−y²)
/// ```
///
/// Returns `None` when the point is outside the truncation gate
/// ([`HARMONICS_TRUNCATION`]) or inside the reference sphere — both cases
/// stay monopole-only, exactly like the old loop.
#[allow(clippy::too_many_arguments)]
pub fn harmonic_correction(
    mu: f64,
    params: &BodyHarmonics,
    offset_inertial: DVec3,
    distance_squared: f64,
    time_s: f64,
    locked_host_dir: Option<DVec3>,
) -> Option<DVec3> {
    if !harmonic_gate(params, distance_squared) {
        return None;
    }
    let (axis_x, axis_y, axis_z) = harmonic_frame(params, time_s, locked_host_dir);
    // The closed form takes field-point coordinates p = probe − body, while
    // the monopole loop hands us offset = body − probe. All correction terms
    // are odd in position, so evaluating at the offset would flip the sign
    // (the even potential is unaffected — which is why only the closed-form
    // tests catch this). Negate once, here.
    let x = -offset_inertial.dot(axis_x);
    let y = -offset_inertial.dot(axis_y);
    let z = -offset_inertial.dot(axis_z);
    let r_sq = distance_squared;
    let r = r_sq.sqrt();
    let ref_sq = params.ref_radius_sq;
    let w = mu * ref_sq / (r_sq * r_sq * r);
    let v = 3.0 * mu * params.c22 * ref_sq / (r_sq * r_sq * r_sq * r);
    let j2_common = 1.5 * w * params.j2 / r_sq;
    let x_sq_minus_y_sq = x * x - y * y;
    let ax = j2_common * x * (5.0 * z * z - r_sq) + v * x * (2.0 * r_sq - 5.0 * x_sq_minus_y_sq);
    let ay = j2_common * y * (5.0 * z * z - r_sq) - v * y * (2.0 * r_sq + 5.0 * x_sq_minus_y_sq);
    let az = -j2_common * z * (3.0 * r_sq - 5.0 * z * z) - 5.0 * v * z * x_sq_minus_y_sq;
    let correction = DVec3::new(ax, ay, az);
    if correction.is_finite() {
        Some(axis_x * ax + axis_y * ay + axis_z * az)
    } else {
        None
    }
}

impl<'a> GravityField<'a> {
    pub fn from_ephemeris(ephemeris: &'a BakedEphemeris) -> Self {
        let mut sources: Vec<_> = ephemeris.gravity_sources().collect();
        sources.sort_by_key(|body| body.id);
        let source_ids = sources.iter().map(|body| body.id).collect();
        let harmonics = sources
            .iter()
            .map(|body| BodyHarmonics::new(body))
            .collect();
        Self {
            ephemeris,
            source_ids,
            harmonics,
        }
    }

    pub fn source_count(&self) -> usize {
        self.source_ids.len()
    }

    /// Body states at `time` in a caller-owned frame, for the dynamical
    /// step cap. The RK scratch frame must not serve as the cap cache:
    /// after a rejected step it holds stage timestamps past the retry
    /// point, and the cap would read future body positions. A dedicated
    /// frame evaluated at the current step time keeps the cap honest, at
    /// one extra ephemeris evaluation per capped step.
    pub fn cap_states<'frame>(
        &self,
        frame: &'frame mut crate::EphemerisFrame,
        time: SimTime,
    ) -> Result<&'frame [crate::BodyState], GravityError> {
        Ok(frame.evaluate(self.ephemeris, time)?)
    }

    /// Ephemeris state for LVLH steering frames: thrust arcs reference a
    /// central body, and the field owns the ephemeris borrow. Transparent
    /// passthrough (same errors as direct ephemeris reads).
    pub fn body_state(&self, id: BodyId, time: SimTime) -> Result<BodyState, GravityError> {
        Ok(self.ephemeris.body_state(id, time)?)
    }

    pub fn acceleration(&self, position: DVec3, time: SimTime) -> Result<DVec3, GravityError> {
        let mut total = DVec3::ZERO;
        for (index, body_id) in self.source_ids.iter().enumerate() {
            let body = self.ephemeris.body(*body_id)?;
            let state = self.ephemeris.body_state(*body_id, time)?;
            let offset = state.position_inertial - position;
            let distance_squared = offset.length_squared();
            if !distance_squared.is_finite() {
                return Err(GravityError::NonFinite { body_id: *body_id });
            }
            if distance_squared == 0.0 {
                return Err(GravityError::Singularity { body_id: *body_id });
            }
            let inverse_distance = distance_squared.sqrt().recip();
            total += offset * (body.mu * inverse_distance.powi(3));
            total += self.harmonic_lane(
                index,
                body.mu,
                offset,
                state.position_inertial,
                distance_squared,
                time.0,
                None,
            )?;
            if !total.is_finite() {
                return Err(GravityError::NonFinite { body_id: *body_id });
            }
        }
        Ok(total)
    }

    /// Shared harmonic lane for both accumulation paths. Identical inputs
    /// and op order keep `acceleration` and `acceleration_from_states`
    /// bitwise identical. `body_pos` is the inertial body center;
    /// `host_pos_override` carries a precomputed host center for the frame
    /// path (the direct path reads it from the ephemeris instead, so one
    /// evaluation never pays two Kepler solves for the same host).
    /// Wide signature is deliberate: one call site per path, no struct
    /// allocation in the hot loop.
    #[allow(clippy::too_many_arguments)]
    fn harmonic_lane(
        &self,
        index: usize,
        body_mu: f64,
        offset: DVec3,
        body_pos: DVec3,
        distance_squared: f64,
        time_s: f64,
        host_pos_override: Option<DVec3>,
    ) -> Result<DVec3, GravityError> {
        let params = &self.harmonics[index];
        if !params.active {
            return Ok(DVec3::ZERO);
        }
        let locked_host_dir = match params.locked_parent {
            None => None,
            Some(_) => {
                let host_pos = match host_pos_override {
                    Some(cached) => cached,
                    None => {
                        let parent = params.locked_parent.expect("locked lane has a parent");
                        self.ephemeris
                            .body_state(parent, SimTime(time_s))
                            .map(|state| state.position_inertial)?
                    }
                };
                Some(host_pos - body_pos)
            }
        };
        // Without a host center a locked C22 frame cannot be built; stay
        // monopole-only rather than fabricating orientation.
        if params.locked_parent.is_some() && locked_host_dir.is_none() {
            return Ok(DVec3::ZERO);
        }
        Ok(harmonic_correction(
            body_mu,
            params,
            offset,
            distance_squared,
            time_s,
            locked_host_dir,
        )
        .unwrap_or(DVec3::ZERO))
    }

    /// Frame-evaluating twin of [`GravityField::acceleration`]: the frame is
    /// evaluated once at `time`, then accumulation runs from the slice via
    /// [`GravityField::acceleration_from_states`]. Bitwise identical to
    /// [`GravityField::acceleration`] for the same timestamp; the win is
    /// that a Dormand–Prince step needs seven RHS evaluations at seven
    /// nearby timestamps, and without the frame each one re-walks every
    /// shared parent chain (58 Kepler solves per call vs 23 per frame on
    /// the 24-body design system). Keep one frame per stepping context.
    pub fn acceleration_with_frame(
        &self,
        position: DVec3,
        time: SimTime,
        frame: &mut crate::EphemerisFrame,
    ) -> Result<DVec3, GravityError> {
        let states = frame.evaluate(self.ephemeris, time)?;
        self.acceleration_from_states(position, states, time)
    }

    /// Acceleration plus gravity gradient from one frame evaluation, for
    /// variational propagation: the ephemeris slice serves both
    /// accumulations, so the augmented RHS costs a single Kepler set per
    /// stage like the plain one. Same errors and summation discipline as
    /// the separate calls.
    pub fn gravity_with_gradient(
        &self,
        position: DVec3,
        time: SimTime,
        frame: &mut crate::EphemerisFrame,
    ) -> Result<(DVec3, glam::DMat3), GravityError> {
        let states = frame.evaluate(self.ephemeris, time)?;
        Ok((
            self.acceleration_from_states(position, states, time)?,
            self.gravity_gradient_from_states(position, states)?,
        ))
    }

    /// Gravity from a precomputed [`EphemerisFrame`] slice instead of fresh
    /// per-body lookups. Same source order, same checks, same summation —
    /// bitwise identical to [`GravityField::acceleration`] for the same
    /// timestamp (harmonics included: the shared lane runs here too, with
    /// the host center read from the slice). A short slice reports the
    /// missing body as unknown rather than panicking on indexing.
    pub fn acceleration_from_states(
        &self,
        position: DVec3,
        states: &[crate::BodyState],
        time: SimTime,
    ) -> Result<DVec3, GravityError> {
        let mut total = DVec3::ZERO;
        for (index, body_id) in self.source_ids.iter().enumerate() {
            let body = self.ephemeris.body(*body_id)?;
            let state = states
                .get(body_id.index())
                .ok_or(crate::EphemerisError::UnknownBody(*body_id))?;
            let offset = state.position_inertial - position;
            let distance_squared = offset.length_squared();
            if !distance_squared.is_finite() {
                return Err(GravityError::NonFinite { body_id: *body_id });
            }
            if distance_squared == 0.0 {
                return Err(GravityError::Singularity { body_id: *body_id });
            }
            let inverse_distance = distance_squared.sqrt().recip();
            total += offset * (body.mu * inverse_distance.powi(3));
            let host_pos = body
                .parent
                .and_then(|parent| states.get(parent.index()))
                .map(|host| host.position_inertial);
            total += self.harmonic_lane(
                index,
                body.mu,
                offset,
                state.position_inertial,
                distance_squared,
                time.0,
                host_pos,
            )?;
            if !total.is_finite() {
                return Err(GravityError::NonFinite { body_id: *body_id });
            }
        }
        Ok(total)
    }

    pub fn potential(&self, position: DVec3, time: SimTime) -> Result<f64, GravityError> {
        let mut total = 0.0;
        for (index, body_id) in self.source_ids.iter().enumerate() {
            let body = self.ephemeris.body(*body_id)?;
            let state = self.ephemeris.body_state(*body_id, time)?;
            let offset = state.position_inertial - position;
            let distance = offset.length();
            if !distance.is_finite() {
                return Err(GravityError::NonFinite { body_id: *body_id });
            }
            if distance == 0.0 {
                return Err(GravityError::Singularity { body_id: *body_id });
            }
            total -= body.mu / distance;
            let params = &self.harmonics[index];
            if harmonic_gate(params, distance * distance) {
                let locked_host_dir = params
                    .locked_parent
                    .and_then(|parent| self.ephemeris.body_state(parent, time).ok())
                    .map(|host| host.position_inertial - state.position_inertial);
                let (axis_x, axis_y, axis_z) = harmonic_frame(params, time.0, locked_host_dir);
                if params.locked_parent.is_some() && locked_host_dir.is_none() {
                    continue;
                }
                let bf = DVec3::new(offset.dot(axis_x), offset.dot(axis_y), offset.dot(axis_z));
                let (u_j2, u_c22) = harmonic_potential_terms(
                    body.mu,
                    params.ref_radius_sq.sqrt(),
                    params.j2,
                    params.c22,
                    bf,
                );
                total += u_j2 + u_c22;
            }
        }
        Ok(total)
    }

    /// Ordered output is retained even though each independent state is
    /// evaluated in parallel. Per-state source accumulation order is stable,
    /// making replay behaviour independent of the worker count.
    pub fn accelerations(
        &self,
        positions: &[DVec3],
        time: SimTime,
    ) -> Result<Vec<DVec3>, GravityError> {
        positions
            .par_iter()
            .map(|position| self.acceleration(*position, time))
            .collect()
    }

    /// Batch twin of [`GravityField::accelerations`] over one precomputed
    /// [`EphemerisFrame`](crate::EphemerisFrame): the caller evaluates the
    /// frame once per tick, then every target accumulates from the same
    /// slice instead of re-walking parent chains per target per source.
    /// Source `(mu, state index)` pairs resolve once per call, not once per
    /// target: N×S ephemeris lookups become S. Same order, same checks, same
    /// summation — bitwise identical to [`GravityField::accelerations`] for
    /// the same timestamp.
    ///
    /// Deliberately serial: fleet-tick batches are latency-bound, and
    /// measurements show Rayon dispatch dominating the math below ~4k
    /// targets (x300 cohort: 1T 0.38 s vs 20T 2.21 s over 40k ticks).
    /// Parallelism belongs one level up — across independent
    /// fleets/cohorts/jobs — never inside this kernel.
    pub fn accelerations_from_frame(
        &self,
        positions: &[DVec3],
        states: &[crate::BodyState],
    ) -> Result<Vec<DVec3>, GravityError> {
        let mut out = Vec::new();
        self.accelerations_from_frame_into(positions, states, &mut out)?;
        Ok(out)
    }

    /// Scratch-writing twin of [`GravityField::accelerations_from_frame`]
    /// for hot loops: `out` is reused across ticks (resized only when the
    /// target count changes), so steady-state ticks allocate nothing.
    pub fn accelerations_from_frame_into(
        &self,
        positions: &[DVec3],
        states: &[crate::BodyState],
        out: &mut Vec<DVec3>,
    ) -> Result<(), GravityError> {
        let sources = self.resolved_frame_sources(states.len())?;
        if out.len() != positions.len() {
            out.resize(positions.len(), DVec3::ZERO);
        }
        for (position, slot) in positions.iter().zip(out.iter_mut()) {
            *slot = accumulate_frame(*position, states, &sources)?;
        }
        Ok(())
    }

    /// `(mu, id)` pairs in accumulation order for the integrator's
    /// dynamical step cap. A body that fails lookup reports `mu = 0` and
    /// is skipped by the cap (same effect as a non-contributing source).
    pub(crate) fn cap_sources(&self) -> Vec<(f64, BodyId)> {
        self.source_ids
            .iter()
            .map(|id| {
                (
                    self.ephemeris.body(*id).map(|body| body.mu).unwrap_or(0.0),
                    *id,
                )
            })
            .collect()
    }

    /// Gravity gradient `G = ∂a/∂r` from a precomputed frame slice, for
    /// variational (state-transition-matrix) propagation:
    ///
    /// ```text
    /// G = Σ μ (3·d·dᵀ/|d|⁵ − I/|d|³),  d = R_body − r
    /// ```
    ///
    /// Same source order, checks and summation discipline as
    /// [`GravityField::acceleration_from_states`]: singular or non-finite
    /// lanes report the same errors instead of NaNs. Sensitivity dynamics
    /// (`Ṡr = Sv`, `Ṡv = G·Sr`) integrated alongside the trajectory give
    /// the exact arrival Jacobian for differential correction — one
    /// augmented propagation per Newton iteration instead of nominal plus
    /// three finite-difference perturbations.
    pub fn gravity_gradient_from_states(
        &self,
        position: DVec3,
        states: &[crate::BodyState],
    ) -> Result<glam::DMat3, GravityError> {
        let mut total = glam::DMat3::ZERO;
        for body_id in &self.source_ids {
            let body = self.ephemeris.body(*body_id)?;
            let state = states
                .get(body_id.index())
                .ok_or(crate::EphemerisError::UnknownBody(*body_id))?;
            let offset = state.position_inertial - position;
            let distance_squared = offset.length_squared();
            if !distance_squared.is_finite() {
                return Err(GravityError::NonFinite { body_id: *body_id });
            }
            if distance_squared == 0.0 {
                return Err(GravityError::Singularity { body_id: *body_id });
            }
            let inv = distance_squared.sqrt().recip();
            let inv3 = inv * inv * inv;
            let outer_scale = 3.0 * body.mu * inv3 * inv * inv;
            let trace_scale = body.mu * inv3;
            // μ·(3·d·dᵀ/|d|⁵ − I/|d|³), columns of the outer product.
            total += glam::DMat3::from_cols(
                offset * (outer_scale * offset.x),
                offset * (outer_scale * offset.y),
                offset * (outer_scale * offset.z),
            ) - glam::DMat3::IDENTITY * trace_scale;
            if !total.is_finite() {
                return Err(GravityError::NonFinite { body_id: *body_id });
            }
        }
        Ok(total)
    }

    /// Resolve `(mu, state index)` for every source in accumulation order.
    /// Reports the same unknown-body error as the per-target path for a
    /// short states slice.
    fn resolved_frame_sources(
        &self,
        states_len: usize,
    ) -> Result<Vec<(f64, BodyId)>, GravityError> {
        let mut sources = Vec::with_capacity(self.source_ids.len());
        for body_id in &self.source_ids {
            if body_id.index() >= states_len {
                return Err(crate::EphemerisError::UnknownBody(*body_id).into());
            }
            sources.push((self.ephemeris.body(*body_id)?.mu, *body_id));
        }
        Ok(sources)
    }
}

/// Accumulate point-mass terms from pre-resolved `(mu, state index)` pairs.
/// Same per-source order, checks and summation as
/// [`GravityField::acceleration`]; the caller guarantees every index is in
/// bounds, so a short slice is reported before the batch starts.
fn accumulate_frame(
    position: DVec3,
    states: &[crate::BodyState],
    sources: &[(f64, BodyId)],
) -> Result<DVec3, GravityError> {
    let mut total = DVec3::ZERO;
    for (mu, body_id) in sources {
        let state = &states[body_id.index()];
        let offset = state.position_inertial - position;
        let distance_squared = offset.length_squared();
        if !distance_squared.is_finite() {
            return Err(GravityError::NonFinite { body_id: *body_id });
        }
        if distance_squared == 0.0 {
            return Err(GravityError::Singularity { body_id: *body_id });
        }
        let inverse_distance = distance_squared.sqrt().recip();
        total += offset * (*mu * inverse_distance.powi(3));
        if !total.is_finite() {
            return Err(GravityError::NonFinite { body_id: *body_id });
        }
    }
    Ok(total)
}

#[derive(Debug, Clone, PartialEq)]
pub enum GravityError {
    Ephemeris(crate::EphemerisError),
    Singularity { body_id: BodyId },
    NonFinite { body_id: BodyId },
}

impl fmt::Display for GravityError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Ephemeris(error) => error.fmt(formatter),
            Self::Singularity { body_id } => {
                write!(formatter, "gravity singularity at {body_id:?}")
            }
            Self::NonFinite { body_id } => {
                write!(
                    formatter,
                    "non-finite gravity contribution from {body_id:?}"
                )
            }
        }
    }
}

impl Error for GravityError {}

impl From<crate::EphemerisError> for GravityError {
    fn from(error: crate::EphemerisError) -> Self {
        Self::Ephemeris(error)
    }
}
