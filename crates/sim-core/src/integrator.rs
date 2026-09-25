use std::{error::Error, fmt};

use glam::DVec3;

use crate::{
    BakedEphemeris, BodyId, BodyState, EphemerisFrame, GravityError, GravityField, SimTime,
};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TestParticleState {
    pub position: DVec3,
    pub velocity: DVec3,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct IntegratorStats {
    pub accepted_steps: u64,
    pub rejected_steps: u64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PropagationResult {
    pub state: TestParticleState,
    pub end_time: SimTime,
    pub stats: IntegratorStats,
}

/// An instantaneous inertial delta-v applied at a time relative to the
/// propagation start. Burns are deliberately state changes, not thrust
/// integrations; finite-burn propulsion belongs to a later vehicle model.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImpulsiveBurn {
    pub time_s: f64,
    pub delta_v_mps: DVec3,
}

/// Configuration for Dormand–Prince 5(4). Tolerances are separated by unit so
/// position and velocity are not accidentally compared on the same scale.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AdaptiveIntegratorConfig {
    pub initial_step_s: f64,
    pub min_step_s: f64,
    pub max_step_s: f64,
    pub absolute_position_tolerance_m: f64,
    pub absolute_velocity_tolerance_mps: f64,
    pub relative_tolerance: f64,
    pub max_steps: u64,
    /// Optional physics-aware step ceiling: each step is additionally
    /// capped at `eta` times the local dynamical time `sqrt(d^3/mu)` to
    /// the nearest gravity source. `None` keeps the legacy behavior
    /// (constant `max_step_s` only). The cap is a safety rail, not a
    /// driver: on smooth cruise the error controller picks smaller steps
    /// on its own, while near a well the cap shrinks below any constant
    /// ceiling before the controller even notices. States used for the
    /// timescale lag the step start by at most one RK stage (the frame
    /// already evaluated); with a conservative `eta` that staleness is
    /// negligible next to the margin, and near wells the accuracy
    /// controller dominates anyway.
    pub dynamical_eta: Option<f64>,
}

impl Default for AdaptiveIntegratorConfig {
    fn default() -> Self {
        Self {
            initial_step_s: 30.0,
            min_step_s: 1.0e-6,
            max_step_s: 3_600.0,
            absolute_position_tolerance_m: 1.0e-3,
            absolute_velocity_tolerance_mps: 1.0e-6,
            relative_tolerance: 1.0e-10,
            max_steps: 1_000_000,
            dynamical_eta: None,
        }
    }
}

/// Fixed-step velocity-Verlet. It has bounded energy error for static
/// conservative fields and is cheap for long coast segments. Moving baked
/// sources make the field explicitly time-dependent, so in that case it is
/// second-order accurate but not strictly symplectic.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VerletConfig {
    pub step_s: f64,
    pub max_steps: u64,
}

impl Default for VerletConfig {
    fn default() -> Self {
        Self {
            step_s: 10.0,
            max_steps: 10_000_000,
        }
    }
}

/// How a sampled prediction path ended. Impact/Completed are display facts
/// about the integrated test particle, not events in the authoritative sim.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SampledPathEnd {
    Completed,
    Impact(BodyId),
}

#[derive(Debug, Clone, PartialEq)]
pub struct SampledPath {
    /// Inertial positions including the initial state, one per accepted step.
    pub positions: Vec<DVec3>,
    /// Inertial velocities parallel to `positions`. The fractional impact
    /// sample reuses the step's start velocity (display-grade; the impact
    /// epoch itself is exact).
    pub velocities: Vec<DVec3>,
    /// Inertial accelerations parallel to `positions`, from the same field
    /// evaluations that advanced each step (initial node included). Velocity
    /// sampling interpolates these, never differentiates positions: at
    /// ~1e12 m barycentric coordinates f64 rounding is ~1e-3 m, and the
    /// position-Hermite derivative amplifies it by ~1/h into 0.01-0.1 m/s
    /// of pure noise. The impact fractional sample reuses the step-start
    /// acceleration (same display-grade caveat as its velocity).
    pub accelerations: Vec<DVec3>,
    /// Exact sample epochs, including a fractional final impact step.
    pub times: Vec<SimTime>,
    pub end_time: SimTime,
    pub end: SampledPathEnd,
    pub stats: IntegratorStats,
}

/// Fixed-step velocity-Verlet recording every sample, for map prediction
/// lines. The field is the full summed multi-body gravity (no SOI switch),
/// so the line stays honest where two-body osculating elements would lie
/// (strong third-body pull, near-parabolic energy). Stops early when the
/// particle enters any of `impact_bodies`, so the line never dives through
/// a planet. Impact is the earliest segment/sphere entry, with a fractional
/// final epoch. This is a display approximation, not contact dynamics. Step count is `VerletConfig::max_steps`; callers size the step
/// from the osculating period (bound) or a fixed horizon (escape).
pub fn propagate_sampled_verlet(
    ephemeris: &BakedEphemeris,
    initial: TestParticleState,
    start_time: SimTime,
    config: VerletConfig,
    impact_bodies: &[BodyId],
) -> Result<SampledPath, IntegratorError> {
    if !initial.position.is_finite()
        || !initial.velocity.is_finite()
        || !start_time.0.is_finite()
        || !(start_time.0 + config.step_s * config.max_steps as f64).is_finite()
    {
        return Err(IntegratorError::InvalidConfig(
            "non-finite sampled trajectory input".into(),
        ));
    }
    if !config.step_s.is_finite() || config.step_s <= 0.0 {
        return Err(IntegratorError::InvalidConfig(
            "Verlet step must be positive".into(),
        ));
    }
    // Fail fast on unknown impact bodies; per-step state lookups below are
    // then infallible for validated ephemerides.
    for body in impact_bodies {
        ephemeris
            .body(*body)
            .map_err(|error| IntegratorError::InvalidConfig(error.to_string()))?;
    }
    let field = GravityField::from_ephemeris(ephemeris);
    let mut positions = Vec::with_capacity(config.max_steps.min(4096) as usize + 1);
    positions.push(initial.position);
    let mut velocities = Vec::with_capacity(config.max_steps.min(4096) as usize + 1);
    velocities.push(initial.velocity);
    let mut accelerations = Vec::with_capacity(config.max_steps.min(4096) as usize + 1);
    accelerations.push(field.acceleration(initial.position, start_time)?);
    let mut times = vec![start_time];
    if let Some(body) = impact_at(ephemeris, impact_bodies, initial.position, start_time) {
        return Ok(SampledPath {
            positions,
            velocities,
            accelerations,
            times,
            end_time: start_time,
            end: SampledPathEnd::Impact(body),
            stats: IntegratorStats::default(),
        });
    }
    let mut state = initial;
    let mut time = start_time;
    let mut stats = IntegratorStats::default();
    let mut end = SampledPathEnd::Completed;
    // The initial acceleration is already evaluated above (and stored in
    // `accelerations`). Reuse it as the first step's `acceleration_0`:
    // otherwise the N-body field is evaluated twice per step (2N instead
    // of N+1). Mirrors `run_table_loop`, which carries `acceleration_1`
    // into the next step.
    let mut acceleration_0 = *accelerations.last().expect("initial acceleration pushed");
    while stats.accepted_steps < config.max_steps {
        let h = config.step_s;
        let next_position = state.position + state.velocity * h + acceleration_0 * (0.5 * h * h);
        let next_time = time.offset(h);
        // Test moving-body relative segments before sampling gravity at an
        // endpoint that may be inside a source. Return the earliest entry,
        // independent of the caller's body order, rather than the far side.
        if let Some((body, fraction)) = impact_segment(
            ephemeris,
            impact_bodies,
            state.position,
            next_position,
            time,
            next_time,
        ) {
            time = time.offset(h * fraction);
            positions.push(state.position.lerp(next_position, fraction));
            velocities.push(state.velocity);
            accelerations.push(acceleration_0);
            times.push(time);
            stats.accepted_steps += 1;
            end = SampledPathEnd::Impact(body);
            break;
        }
        let acceleration_1 = field.acceleration(next_position, next_time)?;
        state = TestParticleState {
            position: next_position,
            velocity: state.velocity + (acceleration_0 + acceleration_1) * (0.5 * h),
        };
        time = next_time;
        stats.accepted_steps += 1;
        positions.push(state.position);
        velocities.push(state.velocity);
        accelerations.push(acceleration_1);
        times.push(time);
        // The accepted endpoint is exactly the next step's start state;
        // carry its acceleration forward so N steps cost N+1 evaluations.
        acceleration_0 = acceleration_1;
    }
    Ok(SampledPath {
        positions,
        velocities,
        accelerations,
        times,
        end_time: time,
        end,
        stats,
    })
}

/// Fast variant of [`propagate_sampled_verlet`] for long coast bakes: source
/// bodies are sampled onto a Hermite table every `table_every_steps` steps
/// (see [`EphemerisTable`](crate::EphemerisTable)) instead of Kepler-solved
/// per step. Same impact semantics; a step whose table acceleration fails
/// falls back to the exact field for that step, so singularities behave like
/// the exact path. Prefer this for multi-day rails bakes; keep the exact
/// variant for short display lines and tests.
pub fn propagate_sampled_verlet_fast(
    ephemeris: &BakedEphemeris,
    initial: TestParticleState,
    start_time: SimTime,
    config: VerletConfig,
    impact_bodies: &[BodyId],
    table_every_steps: u64,
) -> Result<SampledPath, IntegratorError> {
    if !initial.position.is_finite()
        || !initial.velocity.is_finite()
        || !start_time.0.is_finite()
        || !(start_time.0 + config.step_s * config.max_steps as f64).is_finite()
    {
        return Err(IntegratorError::InvalidConfig(
            "non-finite sampled trajectory input".into(),
        ));
    }
    if !config.step_s.is_finite() || config.step_s <= 0.0 {
        return Err(IntegratorError::InvalidConfig(
            "Verlet step must be positive".into(),
        ));
    }
    for body in impact_bodies {
        ephemeris
            .body(*body)
            .map_err(|error| IntegratorError::InvalidConfig(error.to_string()))?;
    }
    let table_every = table_every_steps.max(1);
    let horizon_s = config.step_s * config.max_steps as f64;
    let sources: Vec<BodyId> = {
        let mut ids: Vec<_> = ephemeris.gravity_sources().map(|body| body.id).collect();
        ids.extend_from_slice(impact_bodies);
        ids.sort();
        ids.dedup();
        ids
    };
    let table = crate::EphemerisTable::build(
        ephemeris,
        &sources,
        start_time,
        start_time.offset(horizon_s),
        config.step_s * table_every as f64,
    )
    .map_err(|error| IntegratorError::InvalidConfig(error.to_string()))?;
    let field = GravityField::from_ephemeris(ephemeris);
    let mut positions = Vec::with_capacity(config.max_steps.min(4096) as usize + 1);
    positions.push(initial.position);
    let mut velocities = Vec::with_capacity(config.max_steps.min(4096) as usize + 1);
    velocities.push(initial.velocity);
    let mut accelerations = Vec::with_capacity(config.max_steps.min(4096) as usize + 1);
    let mut times = vec![start_time];
    // Bake-start containment is checked exactly (one lookup, no table error
    // possible); per-step segments below use the table consistently.
    if let Some(body) = impact_at(ephemeris, impact_bodies, initial.position, start_time) {
        // Single-sample path: still carry the start acceleration so the
        // vectors stay parallel (sample_at answers its own epoch). A total
        // field failure here (exact-center singularity) keeps the historic
        // Ok with a zero placeholder rather than invalidating the bake.
        accelerations.push(
            table
                .acceleration_at(initial.position, start_time)
                .or_else(|| field.acceleration(initial.position, start_time).ok())
                .unwrap_or(DVec3::ZERO),
        );
        return Ok(SampledPath {
            positions,
            velocities,
            accelerations,
            times,
            end_time: start_time,
            end: SampledPathEnd::Impact(body),
            stats: IntegratorStats::default(),
        });
    }
    let mut stats = IntegratorStats::default();
    let mut end = SampledPathEnd::Completed;
    let time = run_table_loop(
        &table,
        &field,
        impact_bodies,
        initial,
        start_time,
        config.step_s,
        config.max_steps,
        &mut positions,
        &mut velocities,
        &mut accelerations,
        &mut times,
        &mut stats,
        &mut end,
    )?;
    Ok(SampledPath {
        positions,
        velocities,
        accelerations,
        times,
        end_time: time,
        end,
        stats,
    })
}

/// Table-backed Verlet loop with per-endpoint snapshots: one Hermite eval
/// per track per new endpoint serves gravity and the impact segment test.
/// Accepted endpoints and their acceleration carry into the next step;
/// N full steps need N+1 snapshots and gravity evaluations, not 2N.
/// A step whose snapshot accel fails (singularity/non-finite) falls back to
/// the exact field for that eval, so behavior matches the exact path.
#[allow(clippy::too_many_arguments)]
fn run_table_loop(
    table: &crate::EphemerisTable,
    field: &GravityField,
    impact_bodies: &[BodyId],
    initial: TestParticleState,
    start_time: SimTime,
    step_s: f64,
    max_steps: u64,
    positions: &mut Vec<DVec3>,
    velocities: &mut Vec<DVec3>,
    accelerations: &mut Vec<DVec3>,
    times: &mut Vec<SimTime>,
    stats: &mut IntegratorStats,
    end: &mut SampledPathEnd,
) -> Result<SimTime, IntegratorError> {
    use crate::TableSnapshot;
    let mut state = initial;
    let mut time = start_time;
    let mut start_snap = TableSnapshot::default();
    let mut end_snap = TableSnapshot::default();
    if stats.accepted_steps >= max_steps {
        return Ok(time);
    }
    table.snapshot(time, &mut start_snap);
    let mut acceleration_0 = match table.accel_from(&start_snap, state.position) {
        Some(acceleration) => acceleration,
        None => field.acceleration(state.position, time)?,
    };
    // Fresh bakes carry only the initial sample; resume calls arrive with
    // the resume sample (and its acceleration) already stored.
    if accelerations.len() < positions.len() {
        accelerations.push(acceleration_0);
    }
    while stats.accepted_steps < max_steps {
        let h = step_s;
        let next_position = state.position + state.velocity * h + acceleration_0 * (0.5 * h * h);
        let next_time = time.offset(h);
        table.snapshot(next_time, &mut end_snap);
        if let Some((body, fraction)) = table.impact_from(
            &start_snap,
            &end_snap,
            state.position,
            next_position,
            impact_bodies,
        ) {
            time = time.offset(h * fraction);
            positions.push(state.position.lerp(next_position, fraction));
            velocities.push(state.velocity);
            accelerations.push(acceleration_0);
            times.push(time);
            stats.accepted_steps += 1;
            *end = SampledPathEnd::Impact(body);
            break;
        }
        let acceleration_1 = match table.accel_from(&end_snap, next_position) {
            Some(acceleration) => acceleration,
            None => field.acceleration(next_position, next_time)?,
        };
        state = TestParticleState {
            position: next_position,
            velocity: state.velocity + (acceleration_0 + acceleration_1) * (0.5 * h),
        };
        time = next_time;
        // The accepted endpoint is exactly the next step's initial state.
        // Swap owned buffers; neither the centers nor gravity need recomputing.
        std::mem::swap(&mut start_snap, &mut end_snap);
        acceleration_0 = acceleration_1;
        stats.accepted_steps += 1;
        positions.push(state.position);
        velocities.push(state.velocity);
        accelerations.push(acceleration_1);
        times.push(time);
    }
    Ok(time)
}

/// Extend an existing [`SampledPath`] forward by up to `extra_steps` fixed
/// steps, appending samples in place. The resume state is the path's own end
/// (no re-integration of covered ground); a short table spans only the
/// extension horizon, so each call costs proportionally to the chunk, not
/// the whole path. Returns `Ok(true)` when samples were appended,
/// `Ok(false)` when the path already ended in impact or reached
/// `total_max_steps`. Step size must match the baked path (derived from its
/// first two samples); impact bodies are re-validated like a fresh bake.
pub fn propagate_sampled_extend(
    ephemeris: &BakedEphemeris,
    path: &mut SampledPath,
    config_step_s: f64,
    total_max_steps: u64,
    extra_steps: u64,
    impact_bodies: &[BodyId],
    table_every_steps: u64,
) -> Result<bool, IntegratorError> {
    if !matches!(path.end, SampledPathEnd::Completed) {
        return Ok(false);
    }
    if path.positions.len() < 2
        || path.times.len() != path.positions.len()
        || path.velocities.len() != path.positions.len()
        || path.accelerations.len() != path.positions.len()
        || path.times.last() != Some(&path.end_time)
        || !path.end_time.0.is_finite()
        || !path.positions.last().is_some_and(|p| p.is_finite())
        || !path.velocities.last().is_some_and(|v| v.is_finite())
        || !path.accelerations.last().is_some_and(|a| a.is_finite())
    {
        return Err(IntegratorError::InvalidConfig(
            "cannot extend a degenerate path".into(),
        ));
    }
    let baked_step = path.times[1].seconds() - path.times[0].seconds();
    if !baked_step.is_finite()
        || baked_step <= 0.0
        || !config_step_s.is_finite()
        || config_step_s <= 0.0
        || (baked_step - config_step_s).abs() > 1e-9 * config_step_s.max(1e-9)
    {
        return Err(IntegratorError::InvalidConfig(
            "extension step must match the baked path step".into(),
        ));
    }
    for body in impact_bodies {
        ephemeris
            .body(*body)
            .map_err(|error| IntegratorError::InvalidConfig(error.to_string()))?;
    }
    let done = path.stats.accepted_steps;
    if done >= total_max_steps {
        return Ok(false);
    }
    let take = extra_steps.min(total_max_steps - done);
    if take == 0 {
        return Ok(false);
    }
    let resume = TestParticleState {
        position: *path.positions.last().expect("checked non-empty"),
        velocity: *path.velocities.last().expect("checked non-empty"),
    };
    let resume_time = path.end_time;
    let sources: Vec<BodyId> = {
        let mut ids: Vec<_> = ephemeris.gravity_sources().map(|body| body.id).collect();
        ids.extend_from_slice(impact_bodies);
        ids.sort();
        ids.dedup();
        ids
    };
    let table = crate::EphemerisTable::build(
        ephemeris,
        &sources,
        resume_time,
        resume_time.offset(config_step_s * take as f64),
        config_step_s * table_every_steps.max(1) as f64,
    )
    .map_err(|error| IntegratorError::InvalidConfig(error.to_string()))?;
    let field = GravityField::from_ephemeris(ephemeris);
    // Exact containment at the resume point (one lookup); segments below use
    // the short table consistently.
    if let Some(body) = impact_at(ephemeris, impact_bodies, resume.position, resume_time) {
        path.end = SampledPathEnd::Impact(body);
        return Ok(false);
    }
    // accepted_steps already counts baked steps; bound the shared loop to
    // the table span (done + take), never the total budget — past the short
    // table the interpolant would clamp to its endpoint state and feed the
    // loop wrong gravity.
    let mut stats = path.stats;
    let mut end = SampledPathEnd::Completed;
    let original_len = path.positions.len();
    let result = run_table_loop(
        &table,
        &field,
        impact_bodies,
        resume,
        resume_time,
        config_step_s,
        done + take,
        &mut path.positions,
        &mut path.velocities,
        &mut path.accelerations,
        &mut path.times,
        &mut stats,
        &mut end,
    );
    let end_time = match result {
        Ok(time) => time,
        Err(error) => {
            // Keep the stored samples and metadata consistent if an exact
            // fallback fails partway through this extension.
            path.positions.truncate(original_len);
            path.velocities.truncate(original_len);
            path.accelerations.truncate(original_len);
            path.times.truncate(original_len);
            return Err(error);
        }
    };
    path.stats = stats;
    path.end_time = end_time;
    path.end = end;
    Ok(true)
}

/// Timescale-following variable-step sampling for far display prediction
/// (interstellar escapes, year horizons). Fixed coarse steps cannot resolve
/// a fast periapsis bend: the asymptote error compounds forever (measured
/// 7.4e10 m over a year at 4 h uniform steps). Instead each step spans a
/// fraction `eta` of the local dynamical time `sqrt(d^3/mu)` to the nearest
/// source, clamped to `[h_min, h_max]` — dense at periapsis, daily strides
/// in deep cruise, ~500 samples per year. Same Verlet update per step and
/// same segment impact semantics; sample times are non-uniform (the shared
/// Hermite sampler already handles that). Display-grade only: the flight
/// loop never rides this, it rides fixed-step rails.
#[allow(clippy::too_many_arguments)]
pub fn propagate_sampled_verlet_scaled(
    ephemeris: &BakedEphemeris,
    initial: TestParticleState,
    start_time: SimTime,
    h_min: f64,
    h_max: f64,
    eta: f64,
    max_samples: u64,
    impact_bodies: &[BodyId],
) -> Result<SampledPath, IntegratorError> {
    if !initial.position.is_finite()
        || !initial.velocity.is_finite()
        || !start_time.0.is_finite()
        || !h_min.is_finite()
        || !h_max.is_finite()
        || !eta.is_finite()
        || h_min <= 0.0
        || h_max < h_min
        || eta <= 0.0
    {
        return Err(IntegratorError::InvalidConfig(
            "non-finite scaled trajectory input".into(),
        ));
    }
    for body in impact_bodies {
        ephemeris
            .body(*body)
            .map_err(|error| IntegratorError::InvalidConfig(error.to_string()))?;
    }
    let sources: Vec<(BodyId, f64)> = {
        let mut ids: Vec<_> = ephemeris.gravity_sources().map(|body| body.id).collect();
        ids.extend_from_slice(impact_bodies);
        ids.sort();
        ids.dedup();
        ids.into_iter()
            .map(|id| {
                ephemeris
                    .body(id)
                    .map(|body| (id, body.mu))
                    .map_err(|error| IntegratorError::InvalidConfig(error.to_string()))
            })
            .collect::<Result<Vec<_>, _>>()?
    };
    let field = GravityField::from_ephemeris(ephemeris);
    let mut positions = vec![initial.position];
    let mut velocities = vec![initial.velocity];
    // Display-only path, but keep vectors parallel like every other bake.
    let mut accelerations = vec![field.acceleration(initial.position, start_time)?];
    let mut times = vec![start_time];
    if let Some(body) = impact_at(ephemeris, impact_bodies, initial.position, start_time) {
        return Ok(SampledPath {
            positions,
            velocities,
            accelerations,
            times,
            end_time: start_time,
            end: SampledPathEnd::Impact(body),
            stats: IntegratorStats::default(),
        });
    }
    // Local dynamical time from exact body states (a handful of Kepler
    // solves per step, not per source per substep).
    let timescale = |position: DVec3, time: SimTime| -> f64 {
        let mut best = f64::INFINITY;
        for (id, mu) in &sources {
            if *mu <= 0.0 {
                continue;
            }
            let Ok(state) = ephemeris.body_state(*id, time) else {
                continue;
            };
            let d = (state.position_inertial - position).length();
            if d > 0.0 && d.is_finite() {
                best = best.min((d * d * d / mu).sqrt());
            }
        }
        if best.is_finite() {
            (eta * best).clamp(h_min, h_max)
        } else {
            h_max
        }
    };
    let mut state = initial;
    let mut time = start_time;
    let mut stats = IntegratorStats::default();
    let mut end = SampledPathEnd::Completed;
    while positions.len() as u64 <= max_samples {
        let h = timescale(state.position, time);
        let acceleration_0 = field.acceleration(state.position, time)?;
        let next_position = state.position + state.velocity * h + acceleration_0 * (0.5 * h * h);
        let next_time = time.offset(h);
        if let Some((body, fraction)) = impact_segment(
            ephemeris,
            impact_bodies,
            state.position,
            next_position,
            time,
            next_time,
        ) {
            time = time.offset(h * fraction);
            positions.push(state.position.lerp(next_position, fraction));
            velocities.push(state.velocity);
            accelerations.push(acceleration_0);
            times.push(time);
            stats.accepted_steps += 1;
            end = SampledPathEnd::Impact(body);
            break;
        }
        let acceleration_1 = field.acceleration(next_position, next_time)?;
        state = TestParticleState {
            position: next_position,
            velocity: state.velocity + (acceleration_0 + acceleration_1) * (0.5 * h),
        };
        time = next_time;
        stats.accepted_steps += 1;
        positions.push(state.position);
        velocities.push(state.velocity);
        accelerations.push(acceleration_1);
        times.push(time);
    }
    Ok(SampledPath {
        positions,
        velocities,
        accelerations,
        times,
        end_time: time,
        end,
        stats,
    })
}

/// First impact body whose physical radius contains the point, if any.
/// Bodies that fail lookup are skipped: the caller validates the list once
/// up front, so this only fires for structurally invalid ephemerides.
pub(crate) fn impact_at(
    ephemeris: &BakedEphemeris,
    impact_bodies: &[BodyId],
    position: DVec3,
    time: SimTime,
) -> Option<BodyId> {
    for body_id in impact_bodies {
        let Ok(body) = ephemeris.body(*body_id) else {
            continue;
        };
        if body.radius_m <= 0.0 {
            continue;
        }
        let Ok(state) = ephemeris.body_state(*body_id, time) else {
            continue;
        };
        if (position - state.position_inertial).length() < body.radius_m {
            return Some(*body_id);
        }
    }
    None
}

/// Earliest sphere entry along a step in each body's moving frame.
/// Linear relative motion is a display approximation within this one step;
/// this is not a continuous collision solver for authoritative flight.
pub(crate) fn impact_segment(
    ephemeris: &BakedEphemeris,
    impact_bodies: &[BodyId],
    from: DVec3,
    to: DVec3,
    start_time: SimTime,
    end_time: SimTime,
) -> Option<(BodyId, f64)> {
    let mut first: Option<(BodyId, f64)> = None;
    for body_id in impact_bodies {
        let Ok(body) = ephemeris.body(*body_id) else {
            continue;
        };
        if body.radius_m <= 0.0 {
            continue;
        }
        let Ok(start) = ephemeris.body_state(*body_id, start_time) else {
            continue;
        };
        let Ok(end) = ephemeris.body_state(*body_id, end_time) else {
            continue;
        };
        let relative = from - start.position_inertial;
        let delta = (to - end.position_inertial) - relative;
        let a = delta.length_squared();
        if a <= 0.0 {
            continue;
        }
        let b = relative.dot(delta);
        let c = relative.length_squared() - body.radius_m.powi(2);
        let discriminant = b * b - a * c;
        if discriminant < 0.0 {
            continue;
        }
        // Stable quadratic root for approaching spheres, without cancellation.
        let denominator = -b + discriminant.sqrt();
        let fraction = if c <= 0.0 { 0.0 } else { c / denominator };
        if (0.0..=1.0).contains(&fraction) && first.is_none_or(|(_, previous)| fraction < previous)
        {
            first = Some((*body_id, fraction));
        }
    }
    first
}

pub fn propagate_velocity_verlet(
    field: &GravityField<'_>,
    initial: TestParticleState,
    start_time: SimTime,
    duration_s: f64,
    config: VerletConfig,
) -> Result<PropagationResult, IntegratorError> {
    validate_duration(duration_s)?;
    if !config.step_s.is_finite() || config.step_s <= 0.0 {
        return Err(IntegratorError::InvalidConfig(
            "Verlet step must be positive".into(),
        ));
    }
    let mut state = initial;
    let mut time = start_time;
    let mut remaining = duration_s;
    let mut stats = IntegratorStats::default();
    while remaining > 0.0 {
        if stats.accepted_steps >= config.max_steps {
            return Err(IntegratorError::MaxSteps);
        }
        let h = config.step_s.min(remaining);
        let acceleration_0 = field.acceleration(state.position, time)?;
        let next_position = state.position + state.velocity * h + acceleration_0 * (0.5 * h * h);
        let next_time = time.offset(h);
        let acceleration_1 = field.acceleration(next_position, next_time)?;
        state = TestParticleState {
            position: next_position,
            velocity: state.velocity + (acceleration_0 + acceleration_1) * (0.5 * h),
        };
        time = next_time;
        remaining -= h;
        stats.accepted_steps += 1;
    }
    Ok(PropagationResult {
        state,
        end_time: time,
        stats,
    })
}

pub fn propagate_adaptive(
    field: &GravityField<'_>,
    initial: TestParticleState,
    start_time: SimTime,
    duration_s: f64,
    config: AdaptiveIntegratorConfig,
) -> Result<PropagationResult, IntegratorError> {
    validate_duration(duration_s)?;
    validate_adaptive_config(config)?;
    let mut state = initial;
    let mut time = start_time;
    let mut remaining = duration_s;
    let mut step_s = config.initial_step_s.min(config.max_step_s);
    let mut stats = IntegratorStats::default();
    // One frame per propagation: each of the seven RK stages evaluates the
    // ephemeris once per timestamp instead of re-walking shared parent
    // chains per source (see `acceleration_with_frame`). Buffers grow to
    // the body count on first use, then no per-step allocation.
    let mut frame = EphemerisFrame::new();
    // Dedicated cap cache: the RK scratch frame holds rejected stage
    // timestamps past the retry point (see GravityField::cap_states).
    let mut cap_frame = EphemerisFrame::new();
    // Source mus resolved once, in field order, for the dynamical cap.
    let cap_mus: Option<Vec<(f64, BodyId)>> = config.dynamical_eta.map(|_| field.cap_sources());
    // FSAL cache: Dormand–Prince 5(4) evaluates its last stage at the
    // accepted endpoint, which is exactly the next step's first stage
    // (First Same As Last) — one RHS evaluation saved per accepted step.
    // A rejected step leaves (state, time) untouched, so its first stage
    // stays valid for the retried smaller step and is kept as well.
    // Reuse is bitwise-exact (same values, skipped recomputation), hence
    // trajectories do not change, only the evaluation count does.
    let mut cached_k1: Option<Derivative> = None;

    while remaining > 0.0 {
        if stats.accepted_steps + stats.rejected_steps >= config.max_steps {
            return Err(IntegratorError::MaxSteps);
        }
        let mut h = step_s.min(remaining);
        if let (Some(eta), Some(sources)) = (config.dynamical_eta, cap_mus.as_ref()) {
            let states = field.cap_states(&mut cap_frame, time)?;
            h = h.min(dynamical_cap(
                state.position,
                states,
                sources,
                eta,
                config.max_step_s,
            ));
        }
        if h < config.min_step_s && remaining > config.min_step_s {
            return Err(IntegratorError::StepUnderflow { step_s: h });
        }
        let (candidate, error_state, used_k1, k7) =
            dormand_prince_step(field, state, time, h, cached_k1.take(), &mut frame)?;
        let error = normalized_error(error_state, candidate, config);
        if error <= 1.0 || h <= config.min_step_s {
            if error > 1.0 {
                return Err(IntegratorError::StepUnderflow { step_s: h });
            }
            state = candidate;
            time = time.offset(h);
            remaining -= h;
            stats.accepted_steps += 1;
            step_s = next_step(h, error, config.max_step_s);
            cached_k1 = Some(k7);
        } else {
            stats.rejected_steps += 1;
            step_s = (h * (0.9 * error.powf(-0.2)).clamp(0.1, 0.5)).max(config.min_step_s);
            cached_k1 = Some(used_k1);
        }
    }
    Ok(PropagationResult {
        state,
        end_time: time,
        stats,
    })
}

/// Propagate an ordered sequence of instantaneous inertial burns.
///
/// `ImpulsiveBurn::time_s` is relative to `start_time`; equal timestamps are
/// allowed for staged impulses. Each coast arc uses the same adaptive solver
/// and the returned statistics are the sum over all arcs.
pub fn propagate_adaptive_with_burns(
    field: &GravityField<'_>,
    initial: TestParticleState,
    start_time: SimTime,
    duration_s: f64,
    burns: &[ImpulsiveBurn],
    config: AdaptiveIntegratorConfig,
) -> Result<PropagationResult, IntegratorError> {
    validate_duration(duration_s)?;
    validate_adaptive_config(config)?;
    validate_burn_schedule(duration_s, burns)?;

    let mut state = initial;
    let mut elapsed_s = 0.0;
    let mut stats = IntegratorStats::default();
    for burn in burns {
        let coast = propagate_adaptive(
            field,
            state,
            start_time.offset(elapsed_s),
            burn.time_s - elapsed_s,
            config,
        )?;
        state = coast.state;
        stats.accepted_steps += coast.stats.accepted_steps;
        stats.rejected_steps += coast.stats.rejected_steps;
        state.velocity += burn.delta_v_mps;
        elapsed_s = burn.time_s;
    }

    let coast = propagate_adaptive(
        field,
        state,
        start_time.offset(elapsed_s),
        duration_s - elapsed_s,
        config,
    )?;
    stats.accepted_steps += coast.stats.accepted_steps;
    stats.rejected_steps += coast.stats.rejected_steps;
    Ok(PropagationResult {
        state: coast.state,
        // The semantic endpoint is the requested start + duration. The
        // adaptive arc may accumulate a few ulps while landing on each
        // segment endpoint, so do not leak that bookkeeping round-off.
        end_time: start_time.offset(duration_s),
        stats,
    })
}

/// Sensitivity of the trajectory to its initial velocity (the 3-DOF
/// control of a midcourse burn): columns of `position`/`velocity` are
/// ∂(r, v)/∂v0_j. Start from [`VelocitySensitivity::identity`] at the burn
/// point (`Sr = 0`, `Sv = I`); the arrival `position` matrix is then the
/// miss Jacobian for differential correction — one augmented propagation
/// per Newton iteration instead of nominal plus three finite-difference
/// perturbations.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VelocitySensitivity {
    pub position: [DVec3; 3],
    pub velocity: [DVec3; 3],
}

impl VelocitySensitivity {
    pub fn identity() -> Self {
        Self {
            position: [DVec3::ZERO; 3],
            velocity: [DVec3::X, DVec3::Y, DVec3::Z],
        }
    }
}

/// Augmented trajectory plus sensitivity at the end of
/// [`propagate_adaptive_sensitivity`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SensitivityPropagation {
    pub state: TestParticleState,
    pub sensitivity: VelocitySensitivity,
    pub end_time: SimTime,
    pub stats: IntegratorStats,
}

#[derive(Debug, Clone, Copy)]
struct AugmentedDerivative {
    state: Derivative,
    sens_position: [DVec3; 3],
    sens_velocity: [DVec3; 3],
}

fn augmented_derivative(
    field: &GravityField<'_>,
    state: TestParticleState,
    sens: VelocitySensitivity,
    time: SimTime,
    frame: &mut EphemerisFrame,
) -> Result<AugmentedDerivative, IntegratorError> {
    let (acceleration, gradient) = field.gravity_with_gradient(state.position, time, frame)?;
    let mut sens_velocity = [DVec3::ZERO; 3];
    for (slot, column) in sens_velocity.iter_mut().zip(sens.position.iter()) {
        *slot = gradient * *column;
    }
    Ok(AugmentedDerivative {
        state: Derivative {
            position: state.velocity,
            velocity: acceleration,
        },
        sens_position: sens.velocity,
        sens_velocity,
    })
}

fn add_scaled_sens(
    sens: VelocitySensitivity,
    derivative: AugmentedDerivative,
    scale: f64,
) -> VelocitySensitivity {
    let mut out = sens;
    for i in 0..3 {
        out.position[i] += derivative.sens_position[i] * scale;
        out.velocity[i] += derivative.sens_velocity[i] * scale;
    }
    out
}

fn combine_augmented(
    state: TestParticleState,
    sens: VelocitySensitivity,
    h: f64,
    terms: &[(f64, AugmentedDerivative)],
) -> (TestParticleState, VelocitySensitivity) {
    let mut next_state = state;
    let mut next_sens = sens;
    for (coefficient, derivative) in terms {
        next_state = add_scaled(next_state, derivative.state, h * *coefficient);
        next_sens = add_scaled_sens(next_sens, *derivative, h * *coefficient);
    }
    (next_state, next_sens)
}

/// Variational twin of [`propagate_adaptive`]: the trajectory and its
/// initial-velocity sensitivity ride the same Dormand–Prince steps (same
/// step sequence logic, same FSAL reuse, same dynamical cap). The error
/// controller watches the trajectory alone — sensitivity obeys the linear
/// variational equation driven by that trajectory, so it needs no
/// separate tolerance. Returns the end state plus the end sensitivity.
pub fn propagate_adaptive_sensitivity(
    field: &GravityField<'_>,
    initial: TestParticleState,
    sensitivity: VelocitySensitivity,
    start_time: SimTime,
    duration_s: f64,
    config: AdaptiveIntegratorConfig,
) -> Result<SensitivityPropagation, IntegratorError> {
    validate_duration(duration_s)?;
    validate_adaptive_config(config)?;
    let mut state = initial;
    let mut sens = sensitivity;
    let mut time = start_time;
    let mut remaining = duration_s;
    let mut step_s = config.initial_step_s.min(config.max_step_s);
    let mut stats = IntegratorStats::default();
    let mut frame = EphemerisFrame::new();
    // Dedicated cap cache: the RK scratch frame holds rejected stage
    // timestamps past the retry point (see GravityField::cap_states).
    let mut cap_frame = EphemerisFrame::new();
    let cap_mus: Option<Vec<(f64, BodyId)>> = config.dynamical_eta.map(|_| field.cap_sources());
    let mut cached_k1: Option<AugmentedDerivative> = None;

    while remaining > 0.0 {
        if stats.accepted_steps + stats.rejected_steps >= config.max_steps {
            return Err(IntegratorError::MaxSteps);
        }
        let mut h = step_s.min(remaining);
        if let (Some(eta), Some(sources)) = (config.dynamical_eta, cap_mus.as_ref()) {
            let states = field.cap_states(&mut cap_frame, time)?;
            h = h.min(dynamical_cap(
                state.position,
                states,
                sources,
                eta,
                config.max_step_s,
            ));
        }
        if h < config.min_step_s && remaining > config.min_step_s {
            return Err(IntegratorError::StepUnderflow { step_s: h });
        }
        let (candidate, candidate_sens, error_state, used_k1, k7) =
            augmented_dp_step(field, state, sens, time, h, cached_k1.take(), &mut frame)?;
        let error = normalized_error(error_state, candidate, config);
        if error <= 1.0 || h <= config.min_step_s {
            if error > 1.0 {
                return Err(IntegratorError::StepUnderflow { step_s: h });
            }
            state = candidate;
            sens = candidate_sens;
            time = time.offset(h);
            remaining -= h;
            stats.accepted_steps += 1;
            step_s = next_step(h, error, config.max_step_s);
            cached_k1 = Some(k7);
        } else {
            stats.rejected_steps += 1;
            step_s = (h * (0.9 * error.powf(-0.2)).clamp(0.1, 0.5)).max(config.min_step_s);
            cached_k1 = Some(used_k1);
        }
    }
    Ok(SensitivityPropagation {
        state,
        sensitivity: sens,
        end_time: time,
        stats,
    })
}

#[allow(clippy::too_many_arguments)]
fn augmented_stage(
    field: &GravityField<'_>,
    state: TestParticleState,
    sens: VelocitySensitivity,
    h: f64,
    time: SimTime,
    terms: &[(f64, AugmentedDerivative)],
    frame: &mut EphemerisFrame,
) -> Result<AugmentedDerivative, IntegratorError> {
    let (next_state, next_sens) = combine_augmented(state, sens, h, terms);
    augmented_derivative(field, next_state, next_sens, time, frame)
}

#[allow(clippy::too_many_arguments)]
fn augmented_dp_step(
    field: &GravityField<'_>,
    state: TestParticleState,
    sens: VelocitySensitivity,
    time: SimTime,
    h: f64,
    k1: Option<AugmentedDerivative>,
    frame: &mut EphemerisFrame,
) -> Result<
    (
        TestParticleState,
        VelocitySensitivity,
        TestParticleState,
        AugmentedDerivative,
        AugmentedDerivative,
    ),
    IntegratorError,
> {
    // FSAL: reuse the previous accepted step's last stage when the caller
    // hands it in; otherwise evaluate. The returned `used_k1` lets the
    // caller keep it across a rejected retry (same state and time).
    let k1 = match k1 {
        Some(k1) => k1,
        None => augmented_derivative(field, state, sens, time, frame)?,
    };
    let k2 = augmented_stage(
        field,
        state,
        sens,
        h,
        time.offset(h * 1.0 / 5.0),
        &[(1.0 / 5.0, k1)],
        frame,
    )?;
    let k3 = augmented_stage(
        field,
        state,
        sens,
        h,
        time.offset(h * 3.0 / 10.0),
        &[(3.0 / 40.0, k1), (9.0 / 40.0, k2)],
        frame,
    )?;
    let k4 = augmented_stage(
        field,
        state,
        sens,
        h,
        time.offset(h * 4.0 / 5.0),
        &[(44.0 / 45.0, k1), (-56.0 / 15.0, k2), (32.0 / 9.0, k3)],
        frame,
    )?;
    let k5 = augmented_stage(
        field,
        state,
        sens,
        h,
        time.offset(h * 8.0 / 9.0),
        &[
            (19372.0 / 6561.0, k1),
            (-25360.0 / 2187.0, k2),
            (64448.0 / 6561.0, k3),
            (-212.0 / 729.0, k4),
        ],
        frame,
    )?;
    let k6 = augmented_stage(
        field,
        state,
        sens,
        h,
        time.offset(h),
        &[
            (9017.0 / 3168.0, k1),
            (-355.0 / 33.0, k2),
            (46732.0 / 5247.0, k3),
            (49.0 / 176.0, k4),
            (-5103.0 / 18656.0, k5),
        ],
        frame,
    )?;
    let k7 = augmented_stage(
        field,
        state,
        sens,
        h,
        time.offset(h),
        &[
            (35.0 / 384.0, k1),
            (500.0 / 1113.0, k3),
            (125.0 / 192.0, k4),
            (-2187.0 / 6784.0, k5),
            (11.0 / 84.0, k6),
        ],
        frame,
    )?;
    let (fifth, fifth_sens) = combine_augmented(
        state,
        sens,
        h,
        &[
            (35.0 / 384.0, k1),
            (500.0 / 1113.0, k3),
            (125.0 / 192.0, k4),
            (-2187.0 / 6784.0, k5),
            (11.0 / 84.0, k6),
        ],
    );
    // Fourth-order companion for the error estimate: trajectory only.
    // Sensitivity obeys the linear variational equation driven by that
    // trajectory, so it inherits the step sequence without its own
    // tolerance — no sensitivity error is estimated.
    let fourth = combine(
        state,
        h,
        &[
            (5179.0 / 57600.0, k1.state),
            (7571.0 / 16695.0, k3.state),
            (393.0 / 640.0, k4.state),
            (-92097.0 / 339200.0, k5.state),
            (187.0 / 2100.0, k6.state),
            (1.0 / 40.0, k7.state),
        ],
    );
    let error = TestParticleState {
        position: fifth.position - fourth.position,
        velocity: fifth.velocity - fourth.velocity,
    };
    Ok((fifth, fifth_sens, error, k1, k7))
}

#[derive(Debug, Clone, Copy)]
struct Derivative {
    position: DVec3,
    velocity: DVec3,
}

fn derivative(
    field: &GravityField<'_>,
    state: TestParticleState,
    time: SimTime,
    frame: &mut EphemerisFrame,
) -> Result<Derivative, IntegratorError> {
    Ok(Derivative {
        position: state.velocity,
        velocity: field.acceleration_with_frame(state.position, time, frame)?,
    })
}

fn add_scaled(state: TestParticleState, derivative: Derivative, scale: f64) -> TestParticleState {
    TestParticleState {
        position: state.position + derivative.position * scale,
        velocity: state.velocity + derivative.velocity * scale,
    }
}

fn combine(state: TestParticleState, h: f64, terms: &[(f64, Derivative)]) -> TestParticleState {
    let mut result = state;
    for (coefficient, derivative) in terms {
        result = add_scaled(result, *derivative, h * *coefficient);
    }
    result
}

fn dormand_prince_step(
    field: &GravityField<'_>,
    state: TestParticleState,
    time: SimTime,
    h: f64,
    k1: Option<Derivative>,
    frame: &mut EphemerisFrame,
) -> Result<(TestParticleState, TestParticleState, Derivative, Derivative), IntegratorError> {
    // FSAL: reuse the previous accepted step's last stage when the caller
    // hands it in; otherwise evaluate. The returned `used_k1` lets the
    // caller keep it across a rejected retry (same state and time).
    let k1 = match k1 {
        Some(k1) => k1,
        None => derivative(field, state, time, frame)?,
    };
    let k2 = derivative(
        field,
        combine(state, h, &[(1.0 / 5.0, k1)]),
        time.offset(h * 1.0 / 5.0),
        frame,
    )?;
    let k3 = derivative(
        field,
        combine(state, h, &[(3.0 / 40.0, k1), (9.0 / 40.0, k2)]),
        time.offset(h * 3.0 / 10.0),
        frame,
    )?;
    let k4 = derivative(
        field,
        combine(
            state,
            h,
            &[(44.0 / 45.0, k1), (-56.0 / 15.0, k2), (32.0 / 9.0, k3)],
        ),
        time.offset(h * 4.0 / 5.0),
        frame,
    )?;
    let k5 = derivative(
        field,
        combine(
            state,
            h,
            &[
                (19372.0 / 6561.0, k1),
                (-25360.0 / 2187.0, k2),
                (64448.0 / 6561.0, k3),
                (-212.0 / 729.0, k4),
            ],
        ),
        time.offset(h * 8.0 / 9.0),
        frame,
    )?;
    let k6 = derivative(
        field,
        combine(
            state,
            h,
            &[
                (9017.0 / 3168.0, k1),
                (-355.0 / 33.0, k2),
                (46732.0 / 5247.0, k3),
                (49.0 / 176.0, k4),
                (-5103.0 / 18656.0, k5),
            ],
        ),
        time.offset(h),
        frame,
    )?;
    let k7 = derivative(
        field,
        combine(
            state,
            h,
            &[
                (35.0 / 384.0, k1),
                (500.0 / 1113.0, k3),
                (125.0 / 192.0, k4),
                (-2187.0 / 6784.0, k5),
                (11.0 / 84.0, k6),
            ],
        ),
        time.offset(h),
        frame,
    )?;
    let fifth = combine(
        state,
        h,
        &[
            (35.0 / 384.0, k1),
            (500.0 / 1113.0, k3),
            (125.0 / 192.0, k4),
            (-2187.0 / 6784.0, k5),
            (11.0 / 84.0, k6),
        ],
    );
    let fourth = combine(
        state,
        h,
        &[
            (5179.0 / 57600.0, k1),
            (7571.0 / 16695.0, k3),
            (393.0 / 640.0, k4),
            (-92097.0 / 339200.0, k5),
            (187.0 / 2100.0, k6),
            (1.0 / 40.0, k7),
        ],
    );
    Ok((
        fifth,
        TestParticleState {
            position: fifth.position - fourth.position,
            velocity: fifth.velocity - fourth.velocity,
        },
        k1,
        k7,
    ))
}

fn normalized_error(
    error: TestParticleState,
    candidate: TestParticleState,
    config: AdaptiveIntegratorConfig,
) -> f64 {
    let position_scale = config.absolute_position_tolerance_m
        + config.relative_tolerance * candidate.position.abs().max_element().max(1.0);
    let velocity_scale = config.absolute_velocity_tolerance_mps
        + config.relative_tolerance * candidate.velocity.abs().max_element().max(1.0);
    (error.position.abs().max_element() / position_scale)
        .max(error.velocity.abs().max_element() / velocity_scale)
}

fn next_step(step_s: f64, error: f64, max_step_s: f64) -> f64 {
    next_step_order(step_s, error, max_step_s, 5.0)
}

/// Dormand–Prince 8(5,3) tableau (Hairer & Wanner; numeric values as in
/// SciPy's BSD-licensed `dop853_coefficients` — the tableau is a
/// mathematical fact, the stepping code around it is ours). 12 stages
/// with FSAL: stage 12 evaluated at the 8th-order endpoint doubles as
/// the next step's first stage. `E5` estimates the 8th-vs-5th difference
/// for step control; stiffness detection and dense output are omitted
/// (endpoint-only propagation). Rows list `(stage, coefficient)` pairs.
const DOP853_C: [f64; 12] = [
    0.0,
    0.05260015195876773,
    0.0789002279381516,
    0.1183503419072274,
    0.2816496580927726,
    0.3333333333333333,
    0.25,
    0.3076923076923077,
    0.6512820512820513,
    0.6,
    0.8571428571428571,
    1.0,
];

const DOP853_A1: [(usize, f64); 1] = [(0, 0.05260015195876773)];
const DOP853_A2: [(usize, f64); 2] = [(0, 0.0197250569845379), (1, 0.0591751709536137)];
const DOP853_A3: [(usize, f64); 2] = [(0, 0.02958758547680685), (2, 0.08876275643042054)];
const DOP853_A4: [(usize, f64); 3] = [
    (0, 0.2413651341592667),
    (2, -0.8845494793282861),
    (3, 0.924834003261792),
];
const DOP853_A5: [(usize, f64); 3] = [
    (0, 0.037037037037037035),
    (3, 0.17082860872947386),
    (4, 0.12546768756682242),
];
const DOP853_A6: [(usize, f64); 4] = [
    (0, 3.7109375e-2),
    (3, 0.17025221101954405),
    (4, 0.06021653898045596),
    (5, -1.7578125e-2),
];
const DOP853_A7: [(usize, f64); 5] = [
    (0, 0.03709200011850479),
    (3, 0.17038392571223998),
    (4, 0.10726203044637328),
    (5, -0.015319437748624402),
    (6, 0.008273789163814023),
];
const DOP853_A8: [(usize, f64); 6] = [
    (0, 0.6241109587160757),
    (3, -3.3608926294469414),
    (4, -0.868219346841726),
    (5, 27.59209969944671),
    (6, 20.154067550477894),
    (7, -43.48988418106996),
];
const DOP853_A9: [(usize, f64); 7] = [
    (0, 0.47766253643826434),
    (3, -2.4881146199716677),
    (4, -0.590290826836843),
    (5, 21.230051448181193),
    (6, 15.279233632882423),
    (7, -33.28821096898486),
    (8, -0.020331201708508627),
];
const DOP853_A10: [(usize, f64); 8] = [
    (0, -0.9371424300859873),
    (3, 5.186372428844064),
    (4, 1.0914373489967295),
    (5, -8.149787010746927),
    (6, -18.52006565999696),
    (7, 22.739487099350505),
    (8, 2.4936055526796523),
    (9, -3.0467644718982196),
];
const DOP853_A11: [(usize, f64); 9] = [
    (0, 2.273310147516538),
    (3, -10.53449546673725),
    (4, -2.0008720582248625),
    (5, -17.9589318631188),
    (6, 27.94888452941996),
    (7, -2.8589982771350235),
    (8, -8.87285693353063),
    (9, 12.360567175794303),
    (10, 0.6433927460157636),
];
/// 8th-order endpoint combination (also the FSAL stage location).
const DOP853_B: [(usize, f64); 8] = [
    (0, 0.054293734116568765),
    (5, 4.450312892752409),
    (6, 1.8915178993145003),
    (7, -5.801203960010585),
    (8, 0.3111643669578199),
    (9, -0.1521609496625161),
    (10, 0.20136540080403034),
    (11, 0.04471061572777259),
];
/// 8th-vs-5th error estimator weights over stages 0..11.
const DOP853_E5: [f64; 12] = [
    0.01312004499419488,
    0.0,
    0.0,
    0.0,
    0.0,
    -1.2251564463762044,
    -0.4957589496572502,
    1.6643771824549864,
    -0.35032884874997366,
    0.3341791187130175,
    0.08192320648511571,
    -0.022355307863886294,
];

/// Combine previously computed stages by an indexed coefficient row.
fn combine_indexed(
    state: TestParticleState,
    h: f64,
    stages: &[Derivative; 12],
    row: &[(usize, f64)],
) -> TestParticleState {
    let mut result = state;
    for (index, coefficient) in row {
        result = add_scaled(result, stages[*index], h * *coefficient);
    }
    result
}

fn dop853_step(
    field: &GravityField<'_>,
    state: TestParticleState,
    time: SimTime,
    h: f64,
    k1: Option<Derivative>,
    frame: &mut EphemerisFrame,
) -> Result<(TestParticleState, TestParticleState, Derivative, Derivative), IntegratorError> {
    let mut k = [Derivative {
        position: DVec3::ZERO,
        velocity: DVec3::ZERO,
    }; 12];
    k[0] = match k1 {
        Some(k1) => k1,
        None => derivative(field, state, time, frame)?,
    };
    const ROWS: [&[(usize, f64)]; 11] = [
        &DOP853_A1,
        &DOP853_A2,
        &DOP853_A3,
        &DOP853_A4,
        &DOP853_A5,
        &DOP853_A6,
        &DOP853_A7,
        &DOP853_A8,
        &DOP853_A9,
        &DOP853_A10,
        &DOP853_A11,
    ];
    for (i, row) in ROWS.iter().enumerate() {
        let stage = i + 1;
        let at = combine_indexed(state, h, &k, row);
        k[stage] = derivative(field, at, time.offset(h * DOP853_C[stage]), frame)?;
    }
    let eighth = combine_indexed(state, h, &k, &DOP853_B);
    // FSAL stage at the endpoint; doubles as the next step's k1.
    let k_fsal = derivative(field, eighth, time.offset(h), frame)?;
    // The E5 weights combine stage *derivatives*; the error estimate is a
    // state difference, so the combination needs the step factor `h` (the
    // DP5 path gets this via `combine`). Without it the estimator is off by
    // ~h (units m/s instead of m) and the controller flies blind.
    let mut error_position = DVec3::ZERO;
    let mut error_velocity = DVec3::ZERO;
    for (stage, weight) in DOP853_E5.iter().enumerate() {
        error_position += k[stage].position * (*weight * h);
        error_velocity += k[stage].velocity * (*weight * h);
    }
    Ok((
        eighth,
        TestParticleState {
            position: error_position,
            velocity: error_velocity,
        },
        k[0],
        k_fsal,
    ))
}

/// 8th-order twin of [`propagate_adaptive`]: same configuration type,
/// same validation, same FSAL reuse, same dynamical cap, same statistics.
/// Fewer, larger steps on smooth arcs at ~11 RHS evaluations per accepted
/// step (vs 6 for DP5 with FSAL) — the trade pays off where the error
/// controller, not the ceiling, sets the pace.
pub fn propagate_adaptive_dop853(
    field: &GravityField<'_>,
    initial: TestParticleState,
    start_time: SimTime,
    duration_s: f64,
    config: AdaptiveIntegratorConfig,
) -> Result<PropagationResult, IntegratorError> {
    validate_duration(duration_s)?;
    validate_adaptive_config(config)?;
    let mut state = initial;
    let mut time = start_time;
    let mut remaining = duration_s;
    let mut step_s = config.initial_step_s.min(config.max_step_s);
    let mut stats = IntegratorStats::default();
    let mut frame = EphemerisFrame::new();
    // Dedicated cap cache: the RK scratch frame holds rejected stage
    // timestamps past the retry point (see GravityField::cap_states).
    let mut cap_frame = EphemerisFrame::new();
    let cap_mus: Option<Vec<(f64, BodyId)>> = config.dynamical_eta.map(|_| field.cap_sources());
    let mut cached_k1: Option<Derivative> = None;

    while remaining > 0.0 {
        if stats.accepted_steps + stats.rejected_steps >= config.max_steps {
            return Err(IntegratorError::MaxSteps);
        }
        let mut h = step_s.min(remaining);
        if let (Some(eta), Some(sources)) = (config.dynamical_eta, cap_mus.as_ref()) {
            let states = field.cap_states(&mut cap_frame, time)?;
            h = h.min(dynamical_cap(
                state.position,
                states,
                sources,
                eta,
                config.max_step_s,
            ));
        }
        if h < config.min_step_s && remaining > config.min_step_s {
            return Err(IntegratorError::StepUnderflow { step_s: h });
        }
        let (candidate, error_state, used_k1, k_fsal) =
            dop853_step(field, state, time, h, cached_k1.take(), &mut frame)?;
        let error = normalized_error(error_state, candidate, config);
        if error <= 1.0 || h <= config.min_step_s {
            if error > 1.0 {
                return Err(IntegratorError::StepUnderflow { step_s: h });
            }
            state = candidate;
            time = time.offset(h);
            remaining -= h;
            stats.accepted_steps += 1;
            step_s = next_step_order(h, error, config.max_step_s, 8.0);
            cached_k1 = Some(k_fsal);
        } else {
            stats.rejected_steps += 1;
            step_s = (h * (0.9 * error.powf(-1.0 / 8.0)).clamp(0.1, 0.5)).max(config.min_step_s);
            cached_k1 = Some(used_k1);
        }
    }
    Ok(PropagationResult {
        state,
        end_time: time,
        stats,
    })
}

/// Error-controller growth factor for a method of the given order: the
/// local error scales as `h^(order+1)`, so the step rescales by
/// `error^(-1/order)` (0.9 safety, clamped like the DP5 path).
fn next_step_order(step_s: f64, error: f64, max_step_s: f64, order: f64) -> f64 {
    let factor = if error == 0.0 {
        5.0
    } else {
        (0.9 * error.powf(-1.0 / order)).clamp(0.2, 5.0)
    };
    (step_s * factor).min(max_step_s)
}

/// Physics-aware step ceiling: `eta` times the local dynamical time
/// `sqrt(d^3/mu)` to the nearest gravity source. An empty `states` slice
/// (frame not yet evaluated on the first step) disables the cap by
/// returning `max_step_s` — the accuracy controller owns the first step.
/// Sources with non-positive `mu` or missing states are skipped; when
/// nothing contributes, the constant ceiling applies.
fn dynamical_cap(
    position: DVec3,
    states: &[BodyState],
    sources: &[(f64, BodyId)],
    eta: f64,
    max_step_s: f64,
) -> f64 {
    let mut tau = f64::INFINITY;
    for (mu, id) in sources {
        if *mu <= 0.0 {
            continue;
        }
        let Some(state) = states.get(id.index()) else {
            continue;
        };
        let d = (state.position_inertial - position).length();
        if d > 0.0 && d.is_finite() {
            tau = tau.min((d * d * d / mu).sqrt());
        }
    }
    if tau.is_finite() {
        (eta * tau).min(max_step_s)
    } else {
        max_step_s
    }
}

fn validate_duration(duration_s: f64) -> Result<(), IntegratorError> {
    if !duration_s.is_finite() || duration_s < 0.0 {
        Err(IntegratorError::InvalidConfig(
            "duration must be finite and non-negative".into(),
        ))
    } else {
        Ok(())
    }
}

fn validate_adaptive_config(config: AdaptiveIntegratorConfig) -> Result<(), IntegratorError> {
    if !config.initial_step_s.is_finite()
        || !config.min_step_s.is_finite()
        || !config.max_step_s.is_finite()
        || config.initial_step_s <= 0.0
        || config.min_step_s <= 0.0
        || config.max_step_s < config.min_step_s
        || config.absolute_position_tolerance_m <= 0.0
        || config.absolute_velocity_tolerance_mps <= 0.0
        || config.relative_tolerance <= 0.0
        || config.max_steps == 0
        || config
            .dynamical_eta
            .is_some_and(|eta| !eta.is_finite() || eta <= 0.0)
    {
        return Err(IntegratorError::InvalidConfig(
            "adaptive integrator configuration is invalid".into(),
        ));
    }
    Ok(())
}

fn validate_burn_schedule(duration_s: f64, burns: &[ImpulsiveBurn]) -> Result<(), IntegratorError> {
    let mut previous_time_s = 0.0;
    for (index, burn) in burns.iter().enumerate() {
        if !burn.time_s.is_finite()
            || burn.time_s < 0.0
            || burn.time_s > duration_s
            || !burn.delta_v_mps.is_finite()
            || (index > 0 && burn.time_s < previous_time_s)
        {
            return Err(IntegratorError::InvalidConfig(format!(
                "burn {index} is outside the ordered propagation interval"
            )));
        }
        previous_time_s = burn.time_s;
    }
    Ok(())
}

/// Thrust direction for a finite-burn arc.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ThrustDirection {
    /// Fixed unit vector, inertial frame.
    Inertial(DVec3),
    /// Along the instantaneous inertial velocity.
    Prograde,
    /// Against the instantaneous inertial velocity.
    Retrograde,
    /// LVLH components (radial outward, in-track, orbit-normal) in the
    /// spacecraft-centered RTN frame around `central`, normalized at use.
    /// R = r̂ (outward), C = ĥ (orbit normal), T = C×R (in-track, equals
    /// prograde on circular orbits). Needs non-degenerate orbit geometry
    /// (nonzero radius, non-radial flight) — honest errors otherwise.
    Rtn {
        central: BodyId,
        radial: f64,
        transverse: f64,
        normal: f64,
    },
}

/// Spacecraft-centered RTN basis (right-handed R/T/C) from
/// central-relative position/velocity. Single definition shared by the
/// propagation RHS and the maneuver executor — frames must agree exactly.
/// `None` on degenerate geometry (at the center, or radial flight with no
/// orbit plane).
pub fn rtn_basis(
    position_rel_central_m: DVec3,
    velocity_rel_central_mps: DVec3,
) -> Option<(DVec3, DVec3, DVec3)> {
    if !position_rel_central_m.is_finite() || !velocity_rel_central_mps.is_finite() {
        return None;
    }
    if position_rel_central_m.length_squared() <= 0.0 {
        return None;
    }
    let radial = position_rel_central_m.normalize();
    let momentum = position_rel_central_m.cross(velocity_rel_central_mps);
    if momentum.length_squared() <= 0.0 {
        return None;
    }
    let cross_track = momentum.normalize();
    let transverse = cross_track.cross(radial);
    Some((radial, transverse, cross_track))
}

/// One finite-thrust arc; times relative to propagation start. Thrust and
/// mass flow are full-throttle ratings scaled by `throttle_01`. Mass
/// depletes in closed form inside the arc (`m(t) = m0 - mdot*t` — no extra
/// ODE state); the adaptive stepper sees only the smooth time-dependent
/// acceleration. Arcs must be ordered and non-overlapping; everything
/// between arcs coasts ballistically on the proven path.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThrustArc {
    pub start_s: f64,
    pub duration_s: f64,
    pub direction: ThrustDirection,
    pub throttle_01: f64,
    pub thrust_n: f64,
    pub mass_flow_kgs: f64,
}

/// Result of thrust propagation: end state plus remaining mass.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThrustPropagationResult {
    pub state: TestParticleState,
    pub end_time: SimTime,
    pub final_mass_kg: f64,
    pub stats: IntegratorStats,
}

fn validate_thrust_schedule(
    duration_s: f64,
    arcs: &[ThrustArc],
    initial_mass_kg: f64,
) -> Result<(), IntegratorError> {
    if !initial_mass_kg.is_finite() || initial_mass_kg <= 0.0 {
        return Err(IntegratorError::InvalidConfig(
            "initial mass must be finite and positive".into(),
        ));
    }
    let mut previous_end_s = 0.0;
    for (index, arc) in arcs.iter().enumerate() {
        if !arc.start_s.is_finite()
            || !arc.duration_s.is_finite()
            || !arc.throttle_01.is_finite()
            || !arc.thrust_n.is_finite()
            || !arc.mass_flow_kgs.is_finite()
            || arc.start_s < 0.0
            || arc.duration_s < 0.0
            || arc.start_s + arc.duration_s > duration_s
            || arc.start_s < previous_end_s
            || arc.throttle_01 < 0.0
            || arc.throttle_01 > 1.0
            || arc.thrust_n < 0.0
            || arc.mass_flow_kgs < 0.0
        {
            return Err(IntegratorError::InvalidConfig(format!(
                "thrust arc {index} is outside the ordered propagation interval"
            )));
        }
        if let ThrustDirection::Inertial(direction) = arc.direction {
            if !direction.is_finite() {
                return Err(IntegratorError::InvalidConfig(format!(
                    "thrust arc {index} has non-finite inertial direction"
                )));
            }
            if arc.throttle_01 > 0.0 && arc.thrust_n > 0.0 && direction.length_squared() <= 0.0 {
                return Err(IntegratorError::InvalidConfig(format!(
                    "thrust arc {index} has zero direction with a live engine"
                )));
            }
        }
        previous_end_s = arc.start_s + arc.duration_s;
    }
    Ok(())
}

/// Thrust acceleration vector (inertial) at the current state and mass.
/// Dead arcs (zero throttle or zero thrust) coast exactly, without
/// touching the direction (so parked zero directions are legal). LVLH
/// steering resolves its central body through the field (exact at every
/// stage — validation-time cost, never per-tick flight cost).
fn thrust_vector(
    field: &GravityField<'_>,
    state: TestParticleState,
    time: SimTime,
    direction: ThrustDirection,
    throttle_01: f64,
    thrust_n: f64,
    mass_kg: f64,
) -> Result<DVec3, IntegratorError> {
    let newtons = throttle_01 * thrust_n;
    if newtons <= 0.0 {
        return Ok(DVec3::ZERO);
    }
    let unit = match direction {
        ThrustDirection::Inertial(fixed) => fixed.normalize(),
        ThrustDirection::Prograde => {
            if state.velocity.length_squared() <= 0.0 {
                return Err(IntegratorError::InvalidConfig(
                    "prograde steering needs nonzero velocity".into(),
                ));
            }
            state.velocity.normalize()
        }
        ThrustDirection::Retrograde => {
            if state.velocity.length_squared() <= 0.0 {
                return Err(IntegratorError::InvalidConfig(
                    "retrograde steering needs nonzero velocity".into(),
                ));
            }
            -state.velocity.normalize()
        }
        ThrustDirection::Rtn {
            central,
            radial,
            transverse,
            normal,
        } => {
            let center = field
                .body_state(central, time)
                .map_err(|error| IntegratorError::InvalidConfig(format!("rtn central: {error}")))?;
            let position_rel = state.position - center.position_inertial;
            let velocity_rel = state.velocity - center.velocity_inertial;
            let (basis_r, basis_t, basis_c) = rtn_basis(position_rel, velocity_rel).ok_or(
                IntegratorError::InvalidConfig("rtn steering needs orbit geometry".into()),
            )?;
            let blended = basis_r * radial + basis_t * transverse + basis_c * normal;
            if !blended.is_finite() || blended.length_squared() <= 0.0 {
                return Err(IntegratorError::InvalidConfig(
                    "rtn steering needs nonzero components".into(),
                ));
            }
            blended.normalize()
        }
    };
    Ok(unit * (newtons / mass_kg))
}

fn thrust_derivative(
    field: &GravityField<'_>,
    state: TestParticleState,
    time: SimTime,
    mass_kg: f64,
    arc: &ThrustArc,
) -> Result<Derivative, IntegratorError> {
    let mut acceleration = field.acceleration(state.position, time)?;
    acceleration += thrust_vector(
        field,
        state,
        time,
        arc.direction,
        arc.throttle_01,
        arc.thrust_n,
        mass_kg,
    )?;
    Ok(Derivative {
        position: state.velocity,
        velocity: acceleration,
    })
}

/// Evaluation context for one thrust-arc step: closed-form mass needs the
/// arc start mass and the propagation-relative arc start time.
struct ThrustStepCtx<'a> {
    arc: &'a ThrustArc,
    mass_at_arc_start_kg: f64,
    arc_start_elapsed_s: f64,
    step_start_elapsed_s: f64,
}

impl ThrustStepCtx<'_> {
    fn mass_at(&self, elapsed_s: f64) -> f64 {
        self.mass_at_arc_start_kg
            - self.arc.mass_flow_kgs * self.arc.throttle_01 * (elapsed_s - self.arc_start_elapsed_s)
    }
}

#[allow(clippy::too_many_arguments)]
fn dormand_prince_thrust_step(
    field: &GravityField<'_>,
    state: TestParticleState,
    time: SimTime,
    h: f64,
    ctx: &ThrustStepCtx<'_>,
) -> Result<(TestParticleState, TestParticleState), IntegratorError> {
    // Same DP5 tableau as the ballistic core; only the derivative carries
    // thrust (time-dependent through closed-form mass, state-dependent
    // through velocity-aligned steering). The ballistic path is untouched.
    let thrust_at = |state: TestParticleState, time: SimTime, elapsed_s: f64| {
        thrust_derivative(field, state, time, ctx.mass_at(elapsed_s), ctx.arc)
    };
    let k1 = thrust_at(state, time, ctx.step_start_elapsed_s)?;
    let k2 = thrust_at(
        combine(state, h, &[(1.0 / 5.0, k1)]),
        time.offset(h * 1.0 / 5.0),
        ctx.step_start_elapsed_s + h * 1.0 / 5.0,
    )?;
    let k3 = thrust_at(
        combine(state, h, &[(3.0 / 40.0, k1), (9.0 / 40.0, k2)]),
        time.offset(h * 3.0 / 10.0),
        ctx.step_start_elapsed_s + h * 3.0 / 10.0,
    )?;
    let k4 = thrust_at(
        combine(
            state,
            h,
            &[(44.0 / 45.0, k1), (-56.0 / 15.0, k2), (32.0 / 9.0, k3)],
        ),
        time.offset(h * 4.0 / 5.0),
        ctx.step_start_elapsed_s + h * 4.0 / 5.0,
    )?;
    let k5 = thrust_at(
        combine(
            state,
            h,
            &[
                (19372.0 / 6561.0, k1),
                (-25360.0 / 2187.0, k2),
                (64448.0 / 6561.0, k3),
                (-212.0 / 729.0, k4),
            ],
        ),
        time.offset(h * 8.0 / 9.0),
        ctx.step_start_elapsed_s + h * 8.0 / 9.0,
    )?;
    let k6 = thrust_at(
        combine(
            state,
            h,
            &[
                (9017.0 / 3168.0, k1),
                (-355.0 / 33.0, k2),
                (46732.0 / 5247.0, k3),
                (49.0 / 176.0, k4),
                (-5103.0 / 18656.0, k5),
            ],
        ),
        time.offset(h),
        ctx.step_start_elapsed_s + h,
    )?;
    let k7 = thrust_at(
        combine(
            state,
            h,
            &[
                (35.0 / 384.0, k1),
                (500.0 / 1113.0, k3),
                (125.0 / 192.0, k4),
                (-2187.0 / 6784.0, k5),
                (11.0 / 84.0, k6),
            ],
        ),
        time.offset(h),
        ctx.step_start_elapsed_s + h,
    )?;
    let fifth = combine(
        state,
        h,
        &[
            (35.0 / 384.0, k1),
            (500.0 / 1113.0, k3),
            (125.0 / 192.0, k4),
            (-2187.0 / 6784.0, k5),
            (11.0 / 84.0, k6),
        ],
    );
    let fourth = combine(
        state,
        h,
        &[
            (5179.0 / 57600.0, k1),
            (7571.0 / 16695.0, k3),
            (393.0 / 640.0, k4),
            (-92097.0 / 339200.0, k5),
            (187.0 / 2100.0, k6),
            (1.0 / 40.0, k7),
        ],
    );
    Ok((
        fifth,
        TestParticleState {
            position: fifth.position - fourth.position,
            velocity: fifth.velocity - fourth.velocity,
        },
    ))
}

/// Integrate one thrust arc; returns end state and end mass.
fn propagate_thrust_arc(
    field: &GravityField<'_>,
    initial: TestParticleState,
    mass_at_arc_start_kg: f64,
    start_time: SimTime,
    arc_start_elapsed_s: f64,
    arc: &ThrustArc,
    config: AdaptiveIntegratorConfig,
) -> Result<(TestParticleState, f64, IntegratorStats), IntegratorError> {
    let mut state = initial;
    let mut elapsed_s = arc_start_elapsed_s;
    let mut remaining = arc.duration_s;
    let mut step_s = config.initial_step_s.min(config.max_step_s);
    let mut stats = IntegratorStats::default();
    while remaining > 0.0 {
        if stats.accepted_steps + stats.rejected_steps >= config.max_steps {
            return Err(IntegratorError::MaxSteps);
        }
        let h = step_s.min(remaining);
        if h < config.min_step_s && remaining > config.min_step_s {
            return Err(IntegratorError::StepUnderflow { step_s: h });
        }
        let ctx = ThrustStepCtx {
            arc,
            mass_at_arc_start_kg,
            arc_start_elapsed_s,
            step_start_elapsed_s: elapsed_s,
        };
        // Absolute epoch of this step: `start_time` is the propagation
        // origin and `elapsed_s` is propagation-relative (the caller passes
        // both). Subtracting the arc start here would shift the whole arc
        // back by `arc.start_s` and sample gravity/RTN at the wrong epoch.
        let time = start_time.offset(elapsed_s);
        let (candidate, error_state) = dormand_prince_thrust_step(field, state, time, h, &ctx)?;
        let error = normalized_error(error_state, candidate, config);
        if error <= 1.0 || h <= config.min_step_s {
            if error > 1.0 {
                return Err(IntegratorError::StepUnderflow { step_s: h });
            }
            state = candidate;
            elapsed_s += h;
            remaining -= h;
            stats.accepted_steps += 1;
            step_s = next_step(h, error, config.max_step_s);
        } else {
            stats.rejected_steps += 1;
            step_s = (h * (0.9 * error.powf(-0.2)).clamp(0.1, 0.5)).max(config.min_step_s);
        }
    }
    let mass_end_kg = mass_at_arc_start_kg - arc.mass_flow_kgs * arc.throttle_01 * arc.duration_s;
    Ok((state, mass_end_kg, stats))
}

/// Propagate ballistic coasts and finite-thrust arcs over one horizon.
///
/// Arcs split the horizon exactly like impulsive burns do; coasts reuse
/// the proven ballistic stepper. Propellant is checked BEFORE each arc
/// (an arc that would empty the tanks is a planning error, not a silent
/// coast). Returns the end state and remaining mass.
pub fn propagate_adaptive_with_thrust(
    field: &GravityField<'_>,
    initial: TestParticleState,
    initial_mass_kg: f64,
    start_time: SimTime,
    duration_s: f64,
    arcs: &[ThrustArc],
    config: AdaptiveIntegratorConfig,
) -> Result<ThrustPropagationResult, IntegratorError> {
    validate_duration(duration_s)?;
    validate_adaptive_config(config)?;
    validate_thrust_schedule(duration_s, arcs, initial_mass_kg)?;

    let mut state = initial;
    let mut mass_kg = initial_mass_kg;
    let mut elapsed_s = 0.0;
    let mut stats = IntegratorStats::default();
    for arc in arcs {
        if arc.start_s > elapsed_s {
            let coast = propagate_adaptive(
                field,
                state,
                start_time.offset(elapsed_s),
                arc.start_s - elapsed_s,
                config,
            )?;
            state = coast.state;
            stats.accepted_steps += coast.stats.accepted_steps;
            stats.rejected_steps += coast.stats.rejected_steps;
            elapsed_s = arc.start_s;
        }
        if arc.duration_s > 0.0 {
            let consumption = arc.mass_flow_kgs * arc.throttle_01 * arc.duration_s;
            if mass_kg - consumption <= 0.0 {
                return Err(IntegratorError::InvalidConfig(
                    "thrust arc depletes propellant".into(),
                ));
            }
            let (end, mass_end, arc_stats) =
                propagate_thrust_arc(field, state, mass_kg, start_time, elapsed_s, arc, config)?;
            state = end;
            mass_kg = mass_end;
            stats.accepted_steps += arc_stats.accepted_steps;
            stats.rejected_steps += arc_stats.rejected_steps;
            elapsed_s += arc.duration_s;
        }
    }
    if duration_s > elapsed_s {
        let coast = propagate_adaptive(
            field,
            state,
            start_time.offset(elapsed_s),
            duration_s - elapsed_s,
            config,
        )?;
        state = coast.state;
        stats.accepted_steps += coast.stats.accepted_steps;
        stats.rejected_steps += coast.stats.rejected_steps;
    }
    Ok(ThrustPropagationResult {
        state,
        end_time: start_time.offset(duration_s),
        final_mass_kg: mass_kg,
        stats,
    })
}

#[derive(Debug, Clone, PartialEq)]
pub enum IntegratorError {
    Gravity(GravityError),
    InvalidConfig(String),
    MaxSteps,
    StepUnderflow { step_s: f64 },
}

impl fmt::Display for IntegratorError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Gravity(error) => error.fmt(formatter),
            Self::InvalidConfig(message) => {
                write!(formatter, "invalid integrator config: {message}")
            }
            Self::MaxSteps => write!(formatter, "integrator exceeded max_steps"),
            Self::StepUnderflow { step_s } => {
                write!(formatter, "integrator step underflow at {step_s} s")
            }
        }
    }
}

impl Error for IntegratorError {}

impl From<GravityError> for IntegratorError {
    fn from(error: GravityError) -> Self {
        Self::Gravity(error)
    }
}

#[cfg(test)]
mod cap_tests {
    use super::*;
    use crate::{BakedBody, KeplerOrbit};

    fn binary_ephemeris() -> BakedEphemeris {
        let (mu_primary, mu_secondary, separation): (f64, f64, f64) = (1.0e12, 2.0e11, 2.0e7);
        let total_mu = mu_primary + mu_secondary;
        let mean_motion = (total_mu / separation.powi(3)).sqrt();
        let primary = KeplerOrbit::new(
            total_mu,
            separation * mu_secondary / total_mu,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        .and_then(|orbit| orbit.with_mean_motion(mean_motion))
        .expect("valid primary orbit");
        let secondary = KeplerOrbit::new(
            total_mu,
            separation * mu_primary / total_mu,
            0.0,
            0.0,
            0.0,
            0.0,
            std::f64::consts::PI,
        )
        .and_then(|orbit| orbit.with_mean_motion(mean_motion))
        .expect("valid secondary orbit");
        BakedEphemeris::new(
            "TEST_CAP_BINARY",
            vec![
                BakedBody::synthetic_barycenter(BodyId(0), "barycenter", total_mu, None, None),
                BakedBody::orbital(BodyId(1), "primary", mu_primary, 0.0, BodyId(0), primary),
                BakedBody::orbital(
                    BodyId(2),
                    "secondary",
                    mu_secondary,
                    0.0,
                    BodyId(0),
                    secondary,
                ),
            ],
        )
        .expect("valid binary ephemeris")
    }

    fn hand_cap(
        ephemeris: &BakedEphemeris,
        sources: &[(f64, BodyId)],
        position: DVec3,
        time: SimTime,
        eta: f64,
        max_step_s: f64,
    ) -> f64 {
        let mut tau = f64::INFINITY;
        for (mu, id) in sources {
            if *mu <= 0.0 {
                continue;
            }
            let state = ephemeris.body_state(*id, time).expect("body state");
            let d = (state.position_inertial - position).length();
            if d > 0.0 && d.is_finite() {
                tau = tau.min((d * d * d / mu).sqrt());
            }
        }
        (eta * tau).min(max_step_s)
    }

    #[test]
    fn cap_reads_current_positions_not_rejected_stage_positions() {
        // A rejected RK step leaves the scratch frame holding stage
        // timestamps past the retry point. The cap must not read body
        // positions from it: with a moving source, stale positions give a
        // different step cap, so the test pins the fresh value exactly.
        let ephemeris = binary_ephemeris();
        let field = GravityField::from_ephemeris(&ephemeris);
        let sources = field.cap_sources();
        let now = SimTime::EPOCH;
        let future = SimTime(10_000.0);
        let probe = DVec3::new(4.0e7, 0.0, 0.0);
        let (eta, max_step_s) = (0.02, 1.0e6);
        // Mimic rejected stages: scratch now holds FUTURE positions.
        let mut scratch = EphemerisFrame::new();
        field
            .acceleration_with_frame(probe, future, &mut scratch)
            .expect("stage evaluation succeeds");
        // The source actually moved, so staleness is observable.
        let stale = dynamical_cap(probe, scratch.states(), &sources, eta, max_step_s);
        // The cap path evaluates its own frame at the current time.
        let mut cap_frame = EphemerisFrame::new();
        let states = field.cap_states(&mut cap_frame, now).expect("cap reads");
        let got = dynamical_cap(probe, states, &sources, eta, max_step_s);
        let expected = hand_cap(&ephemeris, &sources, probe, now, eta, max_step_s);
        assert!(
            (got - expected).abs() <= 1.0e-9 * expected.max(1.0),
            "cap must match current-time positions, got={got} expected={expected}"
        );
        assert!(
            (stale - got).abs() > 1.0e-6 * got.max(1.0),
            "test needs a geometry where staleness matters, stale={stale} got={got}"
        );
    }
}
