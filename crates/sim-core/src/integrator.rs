use std::{error::Error, fmt};

use glam::DVec3;

use crate::{BakedEphemeris, BodyId, GravityError, GravityField, SimTime};

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
    let mut times = vec![start_time];
    if let Some(body) = impact_at(ephemeris, impact_bodies, initial.position, start_time) {
        return Ok(SampledPath {
            positions,
            velocities,
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
    while stats.accepted_steps < config.max_steps {
        let h = config.step_s;
        let acceleration_0 = field.acceleration(state.position, time)?;
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
        times.push(time);
    }
    Ok(SampledPath {
        positions,
        velocities,
        times,
        end_time: time,
        end,
        stats,
    })
}

/// Shared fixed-step velocity-Verlet sampling loop. `accel` serves gravity
/// (exact field or table-backed), `impact` tests moving-body segments; both
/// are plain closures so exact, fast and resume paths share one loop with no
/// virtual dispatch. Appends to `target`, which may already hold samples
/// (resume/extend), and reports the final end state through `end`.
#[allow(clippy::too_many_arguments)]
fn run_sampled_loop(
    initial: TestParticleState,
    start_time: SimTime,
    step_s: f64,
    max_steps: u64,
    accel: impl Fn(DVec3, SimTime) -> Result<DVec3, IntegratorError>,
    impact: impl Fn(DVec3, DVec3, SimTime, SimTime) -> Option<(BodyId, f64)>,
    positions: &mut Vec<DVec3>,
    velocities: &mut Vec<DVec3>,
    times: &mut Vec<SimTime>,
    stats: &mut IntegratorStats,
    end: &mut SampledPathEnd,
) -> Result<SimTime, IntegratorError> {
    let mut state = initial;
    let mut time = start_time;
    while stats.accepted_steps < max_steps {
        let h = step_s;
        let acceleration_0 = accel(state.position, time)?;
        let next_position = state.position + state.velocity * h + acceleration_0 * (0.5 * h * h);
        let next_time = time.offset(h);
        if let Some((body, fraction)) = impact(state.position, next_position, time, next_time) {
            time = time.offset(h * fraction);
            positions.push(state.position.lerp(next_position, fraction));
            velocities.push(state.velocity);
            times.push(time);
            stats.accepted_steps += 1;
            *end = SampledPathEnd::Impact(body);
            break;
        }
        let acceleration_1 = accel(next_position, next_time)?;
        state = TestParticleState {
            position: next_position,
            velocity: state.velocity + (acceleration_0 + acceleration_1) * (0.5 * h),
        };
        time = next_time;
        stats.accepted_steps += 1;
        positions.push(state.position);
        velocities.push(state.velocity);
        times.push(time);
    }
    Ok(time)
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
    let mut times = vec![start_time];
    // Bake-start containment is checked exactly (one lookup, no table error
    // possible); per-step segments below use the table consistently.
    if let Some(body) = impact_at(ephemeris, impact_bodies, initial.position, start_time) {
        return Ok(SampledPath {
            positions,
            velocities,
            times,
            end_time: start_time,
            end: SampledPathEnd::Impact(body),
            stats: IntegratorStats::default(),
        });
    }
    let mut stats = IntegratorStats::default();
    let mut end = SampledPathEnd::Completed;
    let accel = |position: DVec3, time: SimTime| -> Result<DVec3, IntegratorError> {
        match table.acceleration_at(position, time) {
            Some(acceleration) => Ok(acceleration),
            None => Ok(field.acceleration(position, time)?),
        }
    };
    let impact = |from: DVec3, to: DVec3, start: SimTime, end: SimTime| {
        table.impact_segment(impact_bodies, from, to, start, end)
    };
    let time = run_sampled_loop(
        initial,
        start_time,
        config.step_s,
        config.max_steps,
        accel,
        impact,
        &mut positions,
        &mut velocities,
        &mut times,
        &mut stats,
        &mut end,
    )?;
    Ok(SampledPath {
        positions,
        velocities,
        times,
        end_time: time,
        end,
        stats,
    })
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
    if path.positions.len() < 2 || path.times.len() != path.positions.len() {
        return Err(IntegratorError::InvalidConfig(
            "cannot extend a degenerate path".into(),
        ));
    }
    let baked_step = path.times[1].seconds() - path.times[0].seconds();
    if !config_step_s.is_finite()
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
    let accel = |position: DVec3, time: SimTime| -> Result<DVec3, IntegratorError> {
        match table.acceleration_at(position, time) {
            Some(acceleration) => Ok(acceleration),
            None => Ok(field.acceleration(position, time)?),
        }
    };
    let impact = |from: DVec3, to: DVec3, start: SimTime, end: SimTime| {
        table.impact_segment(impact_bodies, from, to, start, end)
    };
    // accepted_steps already counts baked steps; bound the shared loop to
    // the table span (done + take), never the total budget — past the short
    // table the interpolant would clamp to its endpoint state and feed the
    // loop wrong gravity.
    let mut stats = path.stats;
    let mut end = SampledPathEnd::Completed;
    let end_time = run_sampled_loop(
        resume,
        resume_time,
        config_step_s,
        done + take,
        accel,
        impact,
        &mut path.positions,
        &mut path.velocities,
        &mut path.times,
        &mut stats,
        &mut end,
    )?;
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
    let mut times = vec![start_time];
    if let Some(body) = impact_at(ephemeris, impact_bodies, initial.position, start_time) {
        return Ok(SampledPath {
            positions,
            velocities,
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
        times.push(time);
    }
    Ok(SampledPath {
        positions,
        velocities,
        times,
        end_time: time,
        end,
        stats,
    })
}

/// First impact body whose physical radius contains the point, if any.
/// Bodies that fail lookup are skipped: the caller validates the list once
/// up front, so this only fires for structurally invalid ephemerides.
fn impact_at(
    ephemeris: &BakedEphemeris,
    impact_bodies: &[BodyId],
    position: DVec3,
    time: SimTime,
) -> Option<BodyId> {
    for body_id in impact_bodies {
        let body = ephemeris.body(*body_id).ok()?;
        if body.radius_m <= 0.0 {
            continue;
        }
        let state = ephemeris.body_state(*body_id, time).ok()?;
        if (position - state.position_inertial).length() < body.radius_m {
            return Some(*body_id);
        }
    }
    None
}

/// Earliest sphere entry along a step in each body's moving frame.
/// Linear relative motion is a display approximation within this one step;
/// this is not a continuous collision solver for authoritative flight.
fn impact_segment(
    ephemeris: &BakedEphemeris,
    impact_bodies: &[BodyId],
    from: DVec3,
    to: DVec3,
    start_time: SimTime,
    end_time: SimTime,
) -> Option<(BodyId, f64)> {
    let mut first: Option<(BodyId, f64)> = None;
    for body_id in impact_bodies {
        let body = ephemeris.body(*body_id).ok()?;
        if body.radius_m <= 0.0 {
            continue;
        }
        let start = ephemeris.body_state(*body_id, start_time).ok()?;
        let end = ephemeris.body_state(*body_id, end_time).ok()?;
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

    while remaining > 0.0 {
        if stats.accepted_steps + stats.rejected_steps >= config.max_steps {
            return Err(IntegratorError::MaxSteps);
        }
        let h = step_s.min(remaining);
        if h < config.min_step_s && remaining > config.min_step_s {
            return Err(IntegratorError::StepUnderflow { step_s: h });
        }
        let (candidate, error_state) = dormand_prince_step(field, state, time, h)?;
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
        } else {
            stats.rejected_steps += 1;
            step_s = (h * (0.9 * error.powf(-0.2)).clamp(0.1, 0.5)).max(config.min_step_s);
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

#[derive(Debug, Clone, Copy)]
struct Derivative {
    position: DVec3,
    velocity: DVec3,
}

fn derivative(
    field: &GravityField<'_>,
    state: TestParticleState,
    time: SimTime,
) -> Result<Derivative, IntegratorError> {
    Ok(Derivative {
        position: state.velocity,
        velocity: field.acceleration(state.position, time)?,
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
) -> Result<(TestParticleState, TestParticleState), IntegratorError> {
    let k1 = derivative(field, state, time)?;
    let k2 = derivative(
        field,
        combine(state, h, &[(1.0 / 5.0, k1)]),
        time.offset(h * 1.0 / 5.0),
    )?;
    let k3 = derivative(
        field,
        combine(state, h, &[(3.0 / 40.0, k1), (9.0 / 40.0, k2)]),
        time.offset(h * 3.0 / 10.0),
    )?;
    let k4 = derivative(
        field,
        combine(
            state,
            h,
            &[(44.0 / 45.0, k1), (-56.0 / 15.0, k2), (32.0 / 9.0, k3)],
        ),
        time.offset(h * 4.0 / 5.0),
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
    let factor = if error == 0.0 {
        5.0
    } else {
        (0.9 * error.powf(-0.2)).clamp(0.2, 5.0)
    };
    (step_s * factor).min(max_step_s)
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
