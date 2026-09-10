use std::{error::Error, fmt};

use glam::{DMat3, DQuat, DVec3};
use serde::{Deserialize, Serialize};

use crate::{
    AeroEnvironment, AeroError, AeroGeometry, AeroModel, AeroResult, AeroState, AtmosphereConfig,
    AtmosphereError,
};

const MIN_STEP_S: f64 = 1.0e-9;
const MIN_MASS_KG: f64 = 1.0e-9;
const MIN_INERTIA: f64 = 1.0e-12;

/// Position/orientation state of one rigid vehicle.
///
/// `orientation` maps body-frame vectors into the inertial frame. Angular
/// velocity is stored in body axes, so a vehicle can use the same state with a
/// fixed body geometry and an atmosphere-relative aero sample.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct RigidBodyState {
    pub position_inertial_m: DVec3,
    pub velocity_inertial_mps: DVec3,
    pub orientation_body_to_inertial: DQuat,
    pub angular_velocity_body_rps: DVec3,
}

impl RigidBodyState {
    pub fn new(
        position_inertial_m: DVec3,
        velocity_inertial_mps: DVec3,
        orientation_body_to_inertial: DQuat,
        angular_velocity_body_rps: DVec3,
    ) -> Result<Self, FlightError> {
        let state = Self {
            position_inertial_m,
            velocity_inertial_mps,
            orientation_body_to_inertial,
            angular_velocity_body_rps,
        };
        state.validate()?;
        Ok(state)
    }

    pub fn stationary(position_inertial_m: DVec3) -> Self {
        Self {
            position_inertial_m,
            velocity_inertial_mps: DVec3::ZERO,
            orientation_body_to_inertial: DQuat::IDENTITY,
            angular_velocity_body_rps: DVec3::ZERO,
        }
    }

    fn validate(self) -> Result<(), FlightError> {
        if !self.position_inertial_m.is_finite()
            || !self.velocity_inertial_mps.is_finite()
            || !self.orientation_body_to_inertial.is_finite()
            || !self.angular_velocity_body_rps.is_finite()
        {
            return Err(FlightError::InvalidState(
                "rigid-body state contains a non-finite value".into(),
            ));
        }
        let orientation_error = (self.orientation_body_to_inertial.length_squared() - 1.0).abs();
        if orientation_error > 1.0e-6 {
            return Err(FlightError::InvalidState(
                "body orientation must be a unit quaternion".into(),
            ));
        }
        Ok(())
    }
}

/// Constant mass properties for the first rigid-body slice.
///
/// Fuel burn and moving-mass updates will replace this value between steps;
/// the integrator does not cache mass or inertia, so those updates remain
/// deterministic and do not require a new vehicle type.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct RigidBodyProperties {
    pub mass_kg: f64,
    pub inertia_body_kg_m2: DMat3,
}

impl RigidBodyProperties {
    pub fn new(mass_kg: f64, inertia_body_kg_m2: DMat3) -> Result<Self, FlightError> {
        let properties = Self {
            mass_kg,
            inertia_body_kg_m2,
        };
        properties.validate()?;
        Ok(properties)
    }

    fn validate(self) -> Result<(), FlightError> {
        if !self.mass_kg.is_finite()
            || self.mass_kg <= MIN_MASS_KG
            || !self.inertia_body_kg_m2.is_finite()
        {
            return Err(FlightError::InvalidProperties(
                "mass and inertia must be finite and positive".into(),
            ));
        }
        let determinant = self.inertia_body_kg_m2.determinant();
        let matrix = self.inertia_body_kg_m2;
        let entries = [
            matrix.x_axis.x,
            matrix.x_axis.y,
            matrix.x_axis.z,
            matrix.y_axis.x,
            matrix.y_axis.y,
            matrix.y_axis.z,
            matrix.z_axis.x,
            matrix.z_axis.y,
            matrix.z_axis.z,
        ];
        let scale = entries
            .iter()
            .map(|entry| entry.abs())
            .fold(1.0_f64, f64::max);
        let symmetric_tolerance = 1.0e-10 * scale;
        let symmetric = (matrix.x_axis.y - matrix.y_axis.x).abs() <= symmetric_tolerance
            && (matrix.x_axis.z - matrix.z_axis.x).abs() <= symmetric_tolerance
            && (matrix.y_axis.z - matrix.z_axis.y).abs() <= symmetric_tolerance;
        let leading_minor_2 = matrix.x_axis.x * matrix.y_axis.y - matrix.y_axis.x.powi(2);
        if !symmetric
            || !determinant.is_finite()
            || determinant <= MIN_INERTIA
            || matrix.x_axis.x <= MIN_INERTIA
            || leading_minor_2 <= MIN_INERTIA
        {
            return Err(FlightError::InvalidProperties(
                "inertia tensor must be symmetric positive-definite".into(),
            ));
        }
        Ok(())
    }
}

/// External forces and gravity sampled for one flight step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlightStepInput {
    pub altitude_m: f64,
    pub gravity_acceleration_inertial_mps2: DVec3,
    /// Vehicle position relative to the rotating atmosphere/body origin,
    /// expressed in vehicle body axes.
    pub position_body_m: DVec3,
    /// Translational/local wind velocity already expressed in vehicle axes.
    /// Rigid atmosphere rotation is added by `evaluate_flight_forces` after
    /// transforming the configured body angular velocity into the same frame.
    pub wind_velocity_body_mps: DVec3,
    pub extra_force_body_n: DVec3,
    pub extra_moment_body_nm: DVec3,
}

impl FlightStepInput {
    pub const fn new(altitude_m: f64, gravity_acceleration_inertial_mps2: DVec3) -> Self {
        Self {
            altitude_m,
            gravity_acceleration_inertial_mps2,
            position_body_m: DVec3::ZERO,
            wind_velocity_body_mps: DVec3::ZERO,
            extra_force_body_n: DVec3::ZERO,
            extra_moment_body_nm: DVec3::ZERO,
        }
    }

    fn validate(self) -> Result<(), FlightError> {
        if !self.altitude_m.is_finite()
            || !self.gravity_acceleration_inertial_mps2.is_finite()
            || !self.position_body_m.is_finite()
            || !self.wind_velocity_body_mps.is_finite()
            || !self.extra_force_body_n.is_finite()
            || !self.extra_moment_body_nm.is_finite()
        {
            return Err(FlightError::InvalidInput(
                "flight step input contains a non-finite value".into(),
            ));
        }
        if self.altitude_m < 0.0 {
            return Err(FlightError::InvalidInput(
                "flight altitude must be non-negative".into(),
            ));
        }
        Ok(())
    }
}

/// Forces and accelerations sampled before a rigid-body state update.
#[derive(Debug, Clone, PartialEq)]
pub struct FlightForces {
    pub environment: AeroEnvironment,
    pub aero: AeroResult,
    pub total_force_body_n: DVec3,
    pub total_moment_body_nm: DVec3,
    pub total_force_inertial_n: DVec3,
    pub acceleration_inertial_mps2: DVec3,
    pub angular_acceleration_body_rps2: DVec3,
}

/// Evaluate atmosphere, local aero, gravity and rigid-body rotational terms
/// without mutating the vehicle state.
pub fn evaluate_flight_forces<M: AeroModel>(
    model: &M,
    geometry: &AeroGeometry,
    atmosphere: AtmosphereConfig,
    state: RigidBodyState,
    properties: RigidBodyProperties,
    input: FlightStepInput,
) -> Result<FlightForces, FlightError> {
    state.validate()?;
    properties.validate()?;
    input.validate()?;
    let rotating_air_velocity_body_mps = atmosphere
        .rotating_air_velocity_body_mps(input.position_body_m, state.orientation_body_to_inertial)
        .map_err(FlightError::Atmosphere)?;
    let environment = atmosphere
        .aero_environment(
            input.altitude_m,
            input.wind_velocity_body_mps + rotating_air_velocity_body_mps,
        )
        .map_err(FlightError::Atmosphere)?;
    let velocity_body_mps =
        state.orientation_body_to_inertial.inverse() * state.velocity_inertial_mps;
    let aero_state = AeroState::new(velocity_body_mps, state.angular_velocity_body_rps);
    let aero = model
        .evaluate_state(aero_state, environment, geometry)
        .map_err(FlightError::Aero)?;
    let total_force_body_n = aero.force_body_n + input.extra_force_body_n;
    let total_moment_body_nm = aero.moment_body_nm + input.extra_moment_body_nm;
    let total_force_inertial_n = state.orientation_body_to_inertial * total_force_body_n;
    let acceleration_inertial_mps2 =
        input.gravity_acceleration_inertial_mps2 + total_force_inertial_n / properties.mass_kg;
    let angular_momentum_body = properties.inertia_body_kg_m2 * state.angular_velocity_body_rps;
    let angular_acceleration_body_rps2 = properties.inertia_body_kg_m2.inverse()
        * (total_moment_body_nm - state.angular_velocity_body_rps.cross(angular_momentum_body));
    if !total_force_body_n.is_finite()
        || !total_moment_body_nm.is_finite()
        || !total_force_inertial_n.is_finite()
        || !acceleration_inertial_mps2.is_finite()
        || !angular_acceleration_body_rps2.is_finite()
    {
        return Err(FlightError::NonFiniteResult);
    }
    Ok(FlightForces {
        environment,
        aero,
        total_force_body_n,
        total_moment_body_nm,
        total_force_inertial_n,
        acceleration_inertial_mps2,
        angular_acceleration_body_rps2,
    })
}

// Implicit midpoint for Euler's rigid-body equation. Its quadratic invariants
// (rotational energy and |I omega|) are conserved with zero external moment.
// Newton solves only three angular unknowns; aerodynamic loads stay sampled
// once at the beginning of the caller's bounded physics step.
fn integrate_rotation(
    orientation: DQuat,
    omega: DVec3,
    inertia: DMat3,
    moment: DVec3,
    step_s: f64,
) -> Result<(DQuat, DVec3), FlightError> {
    let inverse_inertia = inertia.inverse();
    let mut next = omega;
    for _ in 0..12 {
        let midpoint = (omega + next) * 0.5;
        let momentum = inertia * midpoint;
        let residual =
            next - omega - step_s * (inverse_inertia * (moment - midpoint.cross(momentum)));
        if residual.length() <= 1.0e-13 * (1.0 + next.length()) {
            // Cayley rotation paired with midpoint transports the body angular
            // momentum into the same inertial vector for torque-free motion.
            let half = midpoint * (step_s * 0.5);
            let delta = DQuat::from_xyzw(half.x, half.y, half.z, 1.0).normalize();
            return Ok(((orientation * delta).normalize(), next));
        }
        let column = |axis: DVec3| {
            axis + step_s
                * 0.5
                * (inverse_inertia * (axis.cross(momentum) + midpoint.cross(inertia * axis)))
        };
        let jacobian = DMat3::from_cols(column(DVec3::X), column(DVec3::Y), column(DVec3::Z));
        next -= jacobian.inverse() * residual;
        if !next.is_finite() {
            return Err(FlightError::NonFiniteResult);
        }
    }
    Err(FlightError::InvalidInput(
        "angular midpoint solve did not converge; reduce the step".into(),
    ))
}

/// Advance a rigid vehicle by one deterministic semi-implicit step.
///
/// Translation uses symplectic-Euler ordering (`v` then `x`). Rotation solves
/// Euler's equation by implicit midpoint, paired with a normalized Cayley
/// quaternion increment. This preserves energy and inertial angular momentum
/// for torque-free motion instead of numerically accelerating a spinning body.
/// External loads are sampled at the beginning of the step. Callers should
/// still substep rapidly changing aerodynamic loads and control saturation.
pub fn integrate_rigid_body_step<M: AeroModel>(
    model: &M,
    geometry: &AeroGeometry,
    atmosphere: AtmosphereConfig,
    state: RigidBodyState,
    properties: RigidBodyProperties,
    input: FlightStepInput,
    step_s: f64,
) -> Result<(RigidBodyState, FlightForces), FlightError> {
    if !step_s.is_finite() || step_s < MIN_STEP_S {
        return Err(FlightError::InvalidStep);
    }
    let forces = evaluate_flight_forces(model, geometry, atmosphere, state, properties, input)?;
    let velocity_inertial_mps =
        state.velocity_inertial_mps + forces.acceleration_inertial_mps2 * step_s;
    let position_inertial_m = state.position_inertial_m + velocity_inertial_mps * step_s;
    let (orientation_body_to_inertial, angular_velocity_body_rps) = integrate_rotation(
        state.orientation_body_to_inertial,
        state.angular_velocity_body_rps,
        properties.inertia_body_kg_m2,
        forces.total_moment_body_nm,
        step_s,
    )?;
    let next_state = RigidBodyState::new(
        position_inertial_m,
        velocity_inertial_mps,
        orientation_body_to_inertial,
        angular_velocity_body_rps,
    )?;
    Ok((next_state, forces))
}

/// Advance a vehicle for a bounded duration using deterministic equal-sized
/// substeps with one already-sampled input. This compatibility path is useful
/// when gravity, atmosphere and external controls are effectively constant
/// over the duration. State/time-dependent world inputs should use
/// [`integrate_rigid_body_duration_sampled`] so every substep sees a coherent
/// environment sample.
#[allow(clippy::too_many_arguments)]
pub fn integrate_rigid_body_duration<M: AeroModel>(
    model: &M,
    geometry: &AeroGeometry,
    atmosphere: AtmosphereConfig,
    state: RigidBodyState,
    properties: RigidBodyProperties,
    input: FlightStepInput,
    duration_s: f64,
    max_step_s: f64,
) -> Result<(RigidBodyState, FlightForces), FlightError> {
    integrate_rigid_body_duration_sampled(
        model,
        geometry,
        atmosphere,
        state,
        properties,
        duration_s,
        max_step_s,
        |_, _| Ok(input),
    )
}

/// Advance a vehicle for a bounded duration while re-sampling the complete
/// [`FlightStepInput`] before every deterministic substep.
///
/// The callback receives the current rigid-body state and elapsed time from
/// the start of this duration. World adapters can therefore update altitude,
/// multi-body gravity, reference-body translation/rotation, weather and
/// controller moments without freezing those values at render-frame start.
#[allow(clippy::too_many_arguments)]
pub fn integrate_rigid_body_duration_sampled<M, F>(
    model: &M,
    geometry: &AeroGeometry,
    atmosphere: AtmosphereConfig,
    mut state: RigidBodyState,
    properties: RigidBodyProperties,
    duration_s: f64,
    max_step_s: f64,
    mut sample_input: F,
) -> Result<(RigidBodyState, FlightForces), FlightError>
where
    M: AeroModel,
    F: FnMut(RigidBodyState, f64) -> Result<FlightStepInput, FlightError>,
{
    if !duration_s.is_finite()
        || duration_s < 0.0
        || !max_step_s.is_finite()
        || max_step_s < MIN_STEP_S
    {
        return Err(FlightError::InvalidStep);
    }
    if duration_s == 0.0 {
        let input = sample_input(state, 0.0)?;
        return Ok((
            state,
            evaluate_flight_forces(model, geometry, atmosphere, state, properties, input)?,
        ));
    }
    let step_count = (duration_s / max_step_s).ceil();
    if !step_count.is_finite() || step_count > 1_000_000.0 {
        return Err(FlightError::InvalidStep);
    }
    let step_s = duration_s / step_count;
    let mut elapsed_s = 0.0;
    let mut forces = None;
    for _ in 0..step_count as u64 {
        let input = sample_input(state, elapsed_s)?;
        let (next_state, next_forces) = integrate_rigid_body_step(
            model, geometry, atmosphere, state, properties, input, step_s,
        )?;
        state = next_state;
        forces = Some(next_forces);
        elapsed_s += step_s;
    }
    Ok((
        state,
        forces.expect("positive duration has at least one step"),
    ))
}

#[derive(Debug, Clone, PartialEq)]
pub enum FlightError {
    InvalidState(String),
    InvalidProperties(String),
    InvalidInput(String),
    InvalidStep,
    NonFiniteResult,
    Atmosphere(AtmosphereError),
    Aero(AeroError),
}

impl fmt::Display for FlightError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidState(message) => write!(formatter, "invalid rigid-body state: {message}"),
            Self::InvalidProperties(message) => {
                write!(formatter, "invalid rigid-body properties: {message}")
            }
            Self::InvalidInput(message) => write!(formatter, "invalid flight input: {message}"),
            Self::InvalidStep => write!(
                formatter,
                "flight integration step must be finite and positive"
            ),
            Self::NonFiniteResult => {
                write!(formatter, "flight force or acceleration is non-finite")
            }
            Self::Atmosphere(error) => write!(formatter, "atmosphere error: {error}"),
            Self::Aero(error) => write!(formatter, "aerodynamic error: {error}"),
        }
    }
}

impl Error for FlightError {}

impl From<AtmosphereError> for FlightError {
    fn from(error: AtmosphereError) -> Self {
        Self::Atmosphere(error)
    }
}

impl From<AeroError> for FlightError {
    fn from(error: AeroError) -> Self {
        Self::Aero(error)
    }
}
