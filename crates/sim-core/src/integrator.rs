use std::{error::Error, fmt};

use glam::DVec3;

use crate::{GravityError, GravityField, SimTime};

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
