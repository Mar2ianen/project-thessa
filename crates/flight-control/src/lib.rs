//! Typed native guidance and control contracts.
//!
//! This crate contains the domain boundary described by the unified control
//! architecture.  It has no runtime, renderer, network, or JavaScript
//! dependency: clients can produce intent, while the authoritative flight
//! runtime turns that intent into a physical demand and allocates it to
//! effectors.

use std::{error::Error, fmt};

use glam::{DMat3, DQuat, DVec3};
use serde::{Deserialize, Serialize};

/// Physical input mapping is a UI concern, not a control law.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum InputScheme {
    MouseSteering,
    Navball,
    Keyboard,
    Hotas,
    Gamepad,
}

/// Normalized pilot input before guidance interprets it.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PilotAxes {
    pub pitch: f64,
    pub yaw: f64,
    pub roll: f64,
    pub translation: DVec3,
    /// Propulsion demand in the documented `-0.2 .. 1.2` range. Values below
    /// zero request reverse capability; values above one request augmentation.
    pub propulsion: f64,
}

impl Default for PilotAxes {
    fn default() -> Self {
        Self {
            pitch: 0.0,
            yaw: 0.0,
            roll: 0.0,
            translation: DVec3::ZERO,
            propulsion: 0.0,
        }
    }
}

impl PilotAxes {
    pub fn validate(self) -> Result<(), ControlError> {
        if !self.pitch.is_finite()
            || !self.yaw.is_finite()
            || !self.roll.is_finite()
            || !self.translation.is_finite()
            || !self.propulsion.is_finite()
        {
            return Err(ControlError::NonFinite("pilot axes"));
        }
        if !(-0.2..=1.2).contains(&self.propulsion) {
            return Err(ControlError::OutOfRange {
                name: "propulsion",
                value: self.propulsion,
                min: -0.2,
                max: 1.2,
            });
        }
        Ok(())
    }

    pub fn clamped(self) -> Self {
        Self {
            pitch: self.pitch.clamp(-1.0, 1.0),
            yaw: self.yaw.clamp(-1.0, 1.0),
            roll: self.roll.clamp(-1.0, 1.0),
            translation: self.translation.clamp(DVec3::splat(-1.0), DVec3::ONE),
            propulsion: self.propulsion.clamp(-0.2, 1.2),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RollPolicy {
    Free,
    Hold,
    Fixed,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DirectionFrame {
    Body,
    Surface,
    Orbit,
    Inertial,
    Target,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct DirectionTarget {
    pub direction: DVec3,
    pub frame: DirectionFrame,
    /// Stable ephemeris body id used when `frame` is `Target`. Other frames
    /// keep this unset so wire payloads remain compact and unambiguous.
    #[serde(default)]
    pub target_body: Option<u32>,
}

impl DirectionTarget {
    pub fn new(direction: DVec3, frame: DirectionFrame) -> Result<Self, ControlError> {
        if !direction.is_finite() || direction.length_squared() <= 1.0e-12 {
            return Err(ControlError::InvalidDirection);
        }
        Ok(Self {
            direction: direction.normalize(),
            frame,
            target_body: None,
        })
    }

    pub fn for_target(direction: DVec3, target_body: u32) -> Result<Self, ControlError> {
        let mut target = Self::new(direction, DirectionFrame::Target)?;
        target.target_body = Some(target_body);
        Ok(target)
    }

    /// Trust-boundary check: direction shape plus the frame/body invariant
    /// (`Target` carries a body, every other frame carries none). Note
    /// `new` alone cannot enforce this — it always leaves the body unset,
    /// so even `Target` built through `new` fails here until resolved
    /// through `for_target` or deserialized with the body attached.
    pub fn validate(&self) -> Result<(), ControlError> {
        if !self.direction.is_finite() || self.direction.length_squared() <= 1.0e-12 {
            return Err(ControlError::InvalidDirection);
        }
        match self.frame {
            DirectionFrame::Target if self.target_body.is_some() => Ok(()),
            DirectionFrame::Target => Err(ControlError::InvalidTargetBody),
            _ if self.target_body.is_none() => Ok(()),
            _ => Err(ControlError::InvalidTargetBody),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct FlightPathTarget {
    pub direction: DirectionTarget,
    pub roll_policy: RollPolicy,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct TrajectoryPlanId(pub u64);

/// Guidance says what the vehicle should do; it does not select actuators.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum GuidanceIntent {
    ManualAxes(PilotAxes),
    AngularRate {
        rate_body_rps: DVec3,
    },
    Attitude {
        target_body_to_inertial: DQuat,
        roll_policy: RollPolicy,
    },
    VelocityDirection {
        direction: DirectionTarget,
        roll_policy: RollPolicy,
    },
    FlightPath {
        target: FlightPathTarget,
    },
    Trajectory {
        plan: TrajectoryPlanId,
    },
}

impl GuidanceIntent {
    pub fn validate(&self) -> Result<(), ControlError> {
        match self {
            Self::ManualAxes(axes) => axes.validate(),
            Self::AngularRate { rate_body_rps } if !rate_body_rps.is_finite() => {
                Err(ControlError::NonFinite("angular-rate guidance"))
            }
            Self::Attitude {
                target_body_to_inertial,
                ..
            } if !target_body_to_inertial.is_finite()
                || (target_body_to_inertial.length_squared() - 1.0).abs() > 1.0e-5 =>
            {
                Err(ControlError::InvalidAttitude)
            }
            Self::VelocityDirection { direction, .. } => direction.validate(),
            Self::FlightPath { target } => target.direction.validate(),
            _ => Ok(()),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PropulsionDemand {
    /// Normalized demand: reverse below zero, nominal thrust at one,
    /// augmentation above one. The propulsion system decides how to realize
    /// the semantic range.
    pub normalized: f64,
}

impl PropulsionDemand {
    pub fn new(normalized: f64) -> Result<Self, ControlError> {
        if !normalized.is_finite() {
            return Err(ControlError::NonFinite("propulsion demand"));
        }
        if !(-0.2..=1.2).contains(&normalized) {
            return Err(ControlError::OutOfRange {
                name: "propulsion demand",
                value: normalized,
                min: -0.2,
                max: 1.2,
            });
        }
        Ok(Self { normalized })
    }
}

/// Main hand-off from guidance/control laws to the allocator.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ControlDemand {
    pub force_body_n: DVec3,
    pub moment_body_nm: DVec3,
    pub propulsion: PropulsionDemand,
}

/// Absolute wrench envelope: generous for any single X-15-class plant
/// (authoritative thrust is ~2.5e5 N), but blocks absurd 1e308 N wire
/// values from reaching the integrator as Inf/NaN state.
pub const MAX_CONTROL_FORCE_N: f64 = 50.0e6;
pub const MAX_CONTROL_MOMENT_NM: f64 = 50.0e6;

impl ControlDemand {
    pub fn zero() -> Self {
        Self {
            force_body_n: DVec3::ZERO,
            moment_body_nm: DVec3::ZERO,
            propulsion: PropulsionDemand { normalized: 0.0 },
        }
    }

    pub fn validate(self) -> Result<(), ControlError> {
        // Finite-only by design: the allocator saturates large-but-finite
        // requests (e.g. RCS saturation probes), so magnitude envelopes
        // live at the trust boundary (`validate_envelope`), not here.
        if !self.force_body_n.is_finite() || !self.moment_body_nm.is_finite() {
            return Err(ControlError::NonFinite("control demand"));
        }
        PropulsionDemand::new(self.propulsion.normalized).map(|_| ())
    }

    /// Wire/authority envelope for demands arriving from clients. Internal
    /// control laws bypass this and saturate through the allocator instead.
    pub fn validate_envelope(self) -> Result<(), ControlError> {
        self.validate()?;
        if self.force_body_n.length() > MAX_CONTROL_FORCE_N {
            return Err(ControlError::OutOfRange {
                name: "control force",
                value: self.force_body_n.length(),
                min: 0.0,
                max: MAX_CONTROL_FORCE_N,
            });
        }
        if self.moment_body_nm.length() > MAX_CONTROL_MOMENT_NM {
            return Err(ControlError::OutOfRange {
                name: "control moment",
                value: self.moment_body_nm.length(),
                min: 0.0,
                max: MAX_CONTROL_MOMENT_NM,
            });
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct AircraftControlLaw {
    pub max_aoa_rad: Option<f64>,
    pub max_positive_g: Option<f64>,
    pub max_negative_g: Option<f64>,
    pub coordinated_turn_assist: bool,
    pub max_translation_force_n: f64,
}

impl AircraftControlLaw {
    /// Convert aircraft-oriented guidance into a pure physical demand. The
    /// protection limits only reshape the requested pitch response; actuator
    /// selection remains the allocator's responsibility.
    pub fn control_demand(
        self,
        state: AircraftState,
        intent: &GuidanceIntent,
        propulsion: PropulsionDemand,
    ) -> Result<ControlDemand, ControlError> {
        state.validate()?;
        intent.validate()?;
        validate_optional_limit(self.max_aoa_rad)?;
        validate_optional_limit(self.max_positive_g)?;
        validate_optional_limit(self.max_negative_g)?;
        if !self.max_translation_force_n.is_finite() || self.max_translation_force_n < 0.0 {
            return Err(ControlError::InvalidController);
        }
        let (desired_rate, force_body_n) = match intent {
            GuidanceIntent::ManualAxes(axes) => (
                DVec3::new(axes.roll, -axes.pitch, -axes.yaw) * 0.16,
                axes.translation * self.max_translation_force_n,
            ),
            GuidanceIntent::AngularRate { rate_body_rps } => (*rate_body_rps, DVec3::ZERO),
            GuidanceIntent::Attitude {
                target_body_to_inertial,
                ..
            } => {
                let mut error = state.attitude.orientation_body_to_inertial.inverse()
                    * *target_body_to_inertial;
                if error.w < 0.0 {
                    error = -error;
                }
                (error.to_scaled_axis() * 1.6, DVec3::ZERO)
            }
            _ => return Err(ControlError::UnsupportedIntent),
        };
        if !desired_rate.is_finite() {
            return Err(ControlError::NonFinite("desired aircraft rate"));
        }
        let omega = state.attitude.angular_velocity_body_rps;
        let moment = state.attitude.inertia_body_kg_m2 * ((desired_rate - omega) / 0.35)
            + omega.cross(state.attitude.inertia_body_kg_m2 * omega);
        let demand = ControlDemand {
            force_body_n,
            moment_body_nm: moment,
            propulsion,
        };
        FlightPolicy {
            max_aoa_rad: self.max_aoa_rad,
            max_positive_g: self.max_positive_g,
            max_negative_g: self.max_negative_g,
            ..FlightPolicy::default()
        }
        .constrain_demand_with_context(
            demand,
            true,
            true,
            FlightPolicyContext {
                angle_of_attack_rad: if state.air_velocity_body_mps.length() > 1.0e-6 {
                    (-state.air_velocity_body_mps.z).atan2(state.air_velocity_body_mps.x)
                } else {
                    0.0
                },
                load_factor_g: state.load_factor_g,
            },
        )
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SpacecraftControlLaw {
    pub attitude_response_s: f64,
    pub translation_response_s: f64,
    pub max_rate_rps: Option<f64>,
    pub max_translation_force_n: f64,
}

impl Default for SpacecraftControlLaw {
    fn default() -> Self {
        Self {
            attitude_response_s: 0.35,
            translation_response_s: 0.5,
            max_rate_rps: None,
            max_translation_force_n: 800.0,
        }
    }
}

/// Rigid-body state consumed by a native attitude controller. It is a small
/// read-only view rather than a handle to authoritative simulation state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AttitudeState {
    pub orientation_body_to_inertial: DQuat,
    pub angular_velocity_body_rps: DVec3,
    pub inertia_body_kg_m2: DMat3,
}

/// Read-only flight condition supplied to the aircraft control law.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AircraftState {
    pub attitude: AttitudeState,
    pub air_velocity_body_mps: DVec3,
    pub load_factor_g: f64,
}

impl AircraftState {
    pub fn validate(self) -> Result<(), ControlError> {
        self.attitude.validate()?;
        if !self.air_velocity_body_mps.is_finite() || !self.load_factor_g.is_finite() {
            return Err(ControlError::NonFinite("aircraft state"));
        }
        Ok(())
    }
}

impl AttitudeState {
    pub fn validate(self) -> Result<(), ControlError> {
        if !self.orientation_body_to_inertial.is_finite()
            || !self.angular_velocity_body_rps.is_finite()
            || !self.inertia_body_kg_m2.is_finite()
        {
            return Err(ControlError::NonFinite("attitude state"));
        }
        if (self.orientation_body_to_inertial.length_squared() - 1.0).abs() > 1.0e-5 {
            return Err(ControlError::InvalidAttitude);
        }
        Ok(())
    }
}

fn validate_optional_limit(limit: Option<f64>) -> Result<(), ControlError> {
    if limit.is_some_and(|value| !value.is_finite() || value <= 0.0) {
        return Err(ControlError::InvalidController);
    }
    Ok(())
}

impl SpacecraftControlLaw {
    /// Convert an attitude/rate intent to a requested body moment. The law
    /// does not know or care which effector will realize that moment.
    pub fn control_demand(
        self,
        state: AttitudeState,
        intent: &GuidanceIntent,
        propulsion: PropulsionDemand,
    ) -> Result<ControlDemand, ControlError> {
        state.validate()?;
        intent.validate()?;
        if !self.attitude_response_s.is_finite()
            || self.attitude_response_s <= 0.0
            || !self.translation_response_s.is_finite()
            || self.translation_response_s <= 0.0
            || !self.max_translation_force_n.is_finite()
            || self.max_translation_force_n < 0.0
        {
            return Err(ControlError::InvalidController);
        }
        let (desired_rate, force_body_n) = match intent {
            GuidanceIntent::ManualAxes(axes) => (
                DVec3::new(axes.roll, -axes.pitch, -axes.yaw) * 0.16,
                axes.translation * self.max_translation_force_n,
            ),
            GuidanceIntent::AngularRate { rate_body_rps } => (*rate_body_rps, DVec3::ZERO),
            GuidanceIntent::Attitude {
                target_body_to_inertial,
                ..
            } => {
                let mut error =
                    state.orientation_body_to_inertial.inverse() * *target_body_to_inertial;
                if error.w < 0.0 {
                    error = -error;
                }
                let mut rate = error.to_scaled_axis() * 1.6;
                if let Some(max_rate) = self.max_rate_rps {
                    if !max_rate.is_finite() || max_rate <= 0.0 {
                        return Err(ControlError::InvalidController);
                    }
                    rate = rate.clamp_length_max(max_rate);
                }
                (rate, DVec3::ZERO)
            }
            _ => return Err(ControlError::UnsupportedIntent),
        };
        if !desired_rate.is_finite() {
            return Err(ControlError::NonFinite("desired angular rate"));
        }
        let omega = state.angular_velocity_body_rps;
        let moment = state.inertia_body_kg_m2 * ((desired_rate - omega) / self.attitude_response_s)
            + omega.cross(state.inertia_body_kg_m2 * omega);
        Ok(ControlDemand {
            force_body_n,
            moment_body_nm: moment,
            propulsion,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct DirectControlLaw {
    pub force_scale_n: f64,
    pub moment_scale_nm: f64,
}

impl Default for DirectControlLaw {
    fn default() -> Self {
        Self {
            force_scale_n: 800.0,
            moment_scale_nm: 400.0,
        }
    }
}

impl DirectControlLaw {
    fn control_demand(
        self,
        intent: &GuidanceIntent,
        propulsion: PropulsionDemand,
    ) -> Result<ControlDemand, ControlError> {
        intent.validate()?;
        PropulsionDemand::new(propulsion.normalized)?;
        if !self.force_scale_n.is_finite()
            || self.force_scale_n < 0.0
            || !self.moment_scale_nm.is_finite()
            || self.moment_scale_nm < 0.0
        {
            return Err(ControlError::InvalidController);
        }
        let GuidanceIntent::ManualAxes(axes) = intent else {
            return Err(ControlError::UnsupportedIntent);
        };
        Ok(ControlDemand {
            force_body_n: axes.translation * self.force_scale_n,
            moment_body_nm: DVec3::new(axes.roll, -axes.pitch, -axes.yaw) * self.moment_scale_nm,
            propulsion,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum FlightControlLaw {
    Aircraft(AircraftControlLaw),
    Spacecraft(SpacecraftControlLaw),
    Direct(DirectControlLaw),
}

/// Read-only state selected by a control-law dispatcher. Pairing state and
/// law explicitly prevents a plane controller from silently consuming a
/// spacecraft-only state (or vice versa).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ControlLawState {
    Aircraft(AircraftState),
    Spacecraft(AttitudeState),
}

impl FlightControlLaw {
    pub fn control_demand(
        self,
        state: ControlLawState,
        intent: &GuidanceIntent,
        propulsion: PropulsionDemand,
    ) -> Result<ControlDemand, ControlError> {
        match (self, state) {
            (Self::Aircraft(law), ControlLawState::Aircraft(state)) => {
                law.control_demand(state, intent, propulsion)
            }
            (Self::Spacecraft(law), ControlLawState::Spacecraft(state)) => {
                law.control_demand(state, intent, propulsion)
            }
            (Self::Direct(law), _) => law.control_demand(intent, propulsion),
            (Self::Aircraft(_), ControlLawState::Spacecraft(_)) => {
                Err(ControlError::MismatchedControlState)
            }
            (Self::Spacecraft(_), ControlLawState::Aircraft(_)) => {
                Err(ControlError::MismatchedControlState)
            }
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct FlightPolicy {
    pub max_aoa_rad: Option<f64>,
    pub max_positive_g: Option<f64>,
    pub max_negative_g: Option<f64>,
    pub reverse_airborne_allowed: bool,
    pub reverse_in_atmosphere_allowed: bool,
    pub augmentation_allowed: bool,
}

/// Current flight condition used by policy protections that cannot be
/// inferred from a demand alone.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlightPolicyContext {
    pub angle_of_attack_rad: f64,
    pub load_factor_g: f64,
}

/// Outer fraction of an envelope limit over which a pushing-outward
/// demand ramps to zero instead of switching off discontinuously.
const ENVELOPE_RAMP_FRACTION: f64 = 0.10;

/// Smooth barrier scale for one envelope axis: full demand inside the
/// ramp band, linear ramp across its outer fraction, zero at and past
/// the limit. Callers guarantee finite `value_abs` and positive finite
/// `limit` (see `validate_optional_limit`).
fn envelope_scale(value_abs: f64, limit: f64) -> f64 {
    let band_start = limit * (1.0 - ENVELOPE_RAMP_FRACTION);
    if value_abs >= limit {
        0.0
    } else if value_abs <= band_start {
        1.0
    } else {
        (limit - value_abs) / (limit - band_start)
    }
}

impl FlightPolicy {
    /// Apply policy at the guidance/control boundary.  The policy owns
    /// permission checks while the returned demand remains a pure value: no
    /// rigid-body, controller, or actuator state is mutated here.
    pub fn constrain_demand(
        self,
        mut demand: ControlDemand,
        airborne: bool,
        in_atmosphere: bool,
    ) -> ControlDemand {
        demand.propulsion = self.constrain_propulsion(demand.propulsion, airborne, in_atmosphere);
        demand
    }

    /// Apply propulsion permissions and aircraft envelope protections to a
    /// complete demand. At an active AoA or load-factor limit, outward
    /// pitch torque ramps down smoothly instead of switching off, while
    /// the allocator remains free to realize roll/yaw and any
    /// still-permitted translation.
    ///
    /// A hard zero at the boundary chatters (limit crossed -> torque 0 ->
    /// recover slightly -> torque back -> crossed again, every tick). The
    /// ramp is a continuous function of the state, so no chatter is
    /// possible; hysteresis would need last-tick engagement plumbed
    /// through the authority, while smoothness gives the same stability
    /// statelessly — and this function stays a pure value mapping.
    pub fn constrain_demand_with_context(
        self,
        mut demand: ControlDemand,
        airborne: bool,
        in_atmosphere: bool,
        context: FlightPolicyContext,
    ) -> Result<ControlDemand, ControlError> {
        if !context.angle_of_attack_rad.is_finite() || !context.load_factor_g.is_finite() {
            return Err(ControlError::NonFinite("flight policy context"));
        }
        validate_optional_limit(self.max_aoa_rad)?;
        validate_optional_limit(self.max_positive_g)?;
        validate_optional_limit(self.max_negative_g)?;
        demand = self.constrain_demand(demand, airborne, in_atmosphere);
        let aoa_scale = match self.max_aoa_rad {
            // With +X forward and +Z up, +Y is nose-down; outward means
            // pushing away from zero AoA on the current side.
            Some(limit)
                if (context.angle_of_attack_rad > 0.0 && demand.moment_body_nm.y < 0.0)
                    || (context.angle_of_attack_rad < 0.0 && demand.moment_body_nm.y > 0.0) =>
            {
                envelope_scale(context.angle_of_attack_rad.abs(), limit)
            }
            _ => 1.0,
        };
        // Positive load factor is reduced by nose-down torque; negative
        // load factor is reduced by nose-up torque. Keep the recovery
        // direction available at either envelope edge instead of touching
        // the whole pitch channel.
        let g_scale = match (
            self.max_positive_g,
            self.max_negative_g,
            demand.moment_body_nm.y,
        ) {
            (Some(limit), _, moment) if moment < 0.0 => {
                envelope_scale(context.load_factor_g.max(0.0), limit)
            }
            (_, Some(limit), moment) if moment > 0.0 => {
                envelope_scale((-context.load_factor_g).max(0.0), limit)
            }
            _ => 1.0,
        };
        demand.moment_body_nm.y *= aoa_scale.min(g_scale);
        Ok(demand)
    }

    /// Apply permission limits without touching rigid-body or actuator state.
    pub fn constrain_propulsion(
        self,
        demand: PropulsionDemand,
        airborne: bool,
        in_atmosphere: bool,
    ) -> PropulsionDemand {
        let mut normalized = demand.normalized;
        if normalized < 0.0
            && (!self.reverse_airborne_allowed
                || (in_atmosphere && !self.reverse_in_atmosphere_allowed))
        {
            normalized = 0.0;
        }
        if normalized > 1.0 && !self.augmentation_allowed {
            normalized = 1.0;
        }
        if !airborne && normalized < 0.0 && !self.reverse_airborne_allowed {
            normalized = 0.0;
        }
        PropulsionDemand { normalized }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub enum ActuatorGroup {
    AerodynamicSurfaces,
    Rcs,
    ThrustVector,
    DifferentialThrust,
    ReactionWheels,
    ReverseThrusters,
}

/// Scalar actuator dynamics applied after allocation. A target is first
/// clipped to the physical command interval, then approached with a
/// first-order response and an optional slew-rate limit.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ActuatorDynamics {
    pub response_s: f64,
    pub max_rate_per_s: Option<f64>,
    pub min_command: f64,
    pub max_command: f64,
}

impl ActuatorDynamics {
    pub fn validate(self) -> Result<(), ControlError> {
        if !self.response_s.is_finite()
            || self.response_s <= 0.0
            || self
                .max_rate_per_s
                .is_some_and(|rate| !rate.is_finite() || rate <= 0.0)
            || !self.min_command.is_finite()
            || !self.max_command.is_finite()
            || self.min_command > self.max_command
        {
            return Err(ControlError::InvalidController);
        }
        Ok(())
    }

    pub fn advance(self, current: f64, target: f64, dt_s: f64) -> Result<f64, ControlError> {
        self.validate()?;
        if !current.is_finite() || !target.is_finite() || !dt_s.is_finite() || dt_s < 0.0 {
            return Err(ControlError::NonFinite("actuator state"));
        }
        let current = current.clamp(self.min_command, self.max_command);
        let target = target.clamp(self.min_command, self.max_command);
        let alpha = 1.0 - (-dt_s / self.response_s).exp();
        let mut next = current + (target - current) * alpha;
        if let Some(max_rate_per_s) = self.max_rate_per_s {
            next = current + (next - current).clamp(-max_rate_per_s * dt_s, max_rate_per_s * dt_s);
        }
        Ok(next.clamp(self.min_command, self.max_command))
    }
}

/// One generalized allocator effector contribution. The command scalar is
/// bounded to `[0, 1]`; signed effectors expose opposing contributions as
/// separate entries or use a signed `force_per_command` where appropriate.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct EffectorContribution {
    pub group: ActuatorGroup,
    pub force_per_command_n: DVec3,
    pub moment_per_command_nm: DVec3,
    pub max_command: f64,
    pub weight: f64,
}

impl EffectorContribution {
    pub fn validate(self) -> Result<(), ControlError> {
        if !self.force_per_command_n.is_finite()
            || !self.moment_per_command_nm.is_finite()
            || !self.max_command.is_finite()
            || self.max_command <= 0.0
            || !self.weight.is_finite()
            || self.weight <= 0.0
        {
            return Err(ControlError::InvalidEffector);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct AllocationResult {
    pub commands: Vec<f64>,
    pub achieved_force_body_n: DVec3,
    pub achieved_moment_body_nm: DVec3,
    pub residual_force_body_n: DVec3,
    pub residual_moment_body_nm: DVec3,
    pub saturated: bool,
}

/// Deterministic bounded least-squares allocator. It solves the coupled
/// 6-DOF problem `min ||A u - b||² + Σ (u_i / weight_i)² * reg` subject to
/// `0 <= u_i <= max_command` via an active-set loop, so the result does not
/// depend on effector declaration order. `weight` is a preference (larger =
/// cheaper) that only breaks ties in redundant directions; on a determined
/// axis the demand is met exactly up to the bound. Force (N) and moment (Nm)
/// rows share one norm, matching the previous greedy metric; callers that
/// mix them should keep their scales comparable (the RCS plant does).
/// The contract is intentionally generic; control laws are unchanged.
pub fn allocate_wrench(
    demand: ControlDemand,
    effectors: &[EffectorContribution],
) -> Result<AllocationResult, ControlError> {
    demand.validate()?;
    for effector in effectors {
        effector.validate()?;
    }
    let count = effectors.len();
    let mut columns = Vec::with_capacity(count);
    for effector in effectors {
        columns.push([
            effector.force_per_command_n.x,
            effector.force_per_command_n.y,
            effector.force_per_command_n.z,
            effector.moment_per_command_nm.x,
            effector.moment_per_command_nm.y,
            effector.moment_per_command_nm.z,
        ]);
    }
    let target = [
        demand.force_body_n.x,
        demand.force_body_n.y,
        demand.force_body_n.z,
        demand.moment_body_nm.x,
        demand.moment_body_nm.y,
        demand.moment_body_nm.z,
    ];
    if count == 0 {
        let saturated = target.iter().map(|v| v * v).sum::<f64>() > 1.0e-12;
        return Ok(AllocationResult {
            commands: Vec::new(),
            achieved_force_body_n: DVec3::ZERO,
            achieved_moment_body_nm: DVec3::ZERO,
            residual_force_body_n: demand.force_body_n,
            residual_moment_body_nm: demand.moment_body_nm,
            saturated,
        });
    }
    // Regularization is a fallback for rank-deficient free sets only
    // (opposed RCS pairs, duplicate columns): it splits redundant
    // authority by weight instead of declaration order. Determined axes
    // solve exactly with no penalty, so decoupled plants stay bit-exact.
    let max_col_norm_sq = columns
        .iter()
        .map(|col| col.iter().map(|v| v * v).sum::<f64>())
        .fold(0.0_f64, f64::max);
    let reg_scale = 1.0e-12 * max_col_norm_sq.max(1.0);
    let mut commands = vec![0.0; count];
    let mut free = vec![true; count];
    // Active-set loop: solve the free subsystem, fix the worst bound
    // violator, repeat. At most `count` fixes, so this always terminates.
    let mut solved_free: Vec<(usize, f64)> = Vec::new();
    for _ in 0..=count {
        let free_indices: Vec<usize> = (0..count).filter(|&i| free[i]).collect();
        if free_indices.is_empty() {
            solved_free.clear();
            break;
        }
        // Residual demand after fixed contributions (index-order sum for
        // determinism).
        let mut residual_target = target;
        for i in 0..count {
            if !free[i] && commands[i] != 0.0 {
                for row in 0..6 {
                    residual_target[row] -= columns[i][row] * commands[i];
                }
            }
        }
        let dim = free_indices.len();
        let mut matrix = vec![vec![0.0; dim]; dim];
        let mut rhs = vec![0.0; dim];
        for (a, &col_a) in free_indices.iter().enumerate() {
            let mut dot_b = 0.0;
            for row in 0..6 {
                dot_b += columns[col_a][row] * residual_target[row];
            }
            rhs[a] = dot_b;
            for (b, &col_b) in free_indices.iter().enumerate() {
                let dot = columns[col_a]
                    .iter()
                    .zip(columns[col_b].iter())
                    .map(|(a, b)| a * b)
                    .sum();
                matrix[a][b] = dot;
            }
        }
        // Exact solve first; only a (near-)singular free set retries with
        // the weight-scaled diagonal.
        let mut matrix_exact = matrix.clone();
        let mut rhs_exact = rhs.clone();
        let solution = match solve_dense_system(&mut matrix_exact, &mut rhs_exact) {
            Some(exact) => exact,
            None => {
                for (a, &col_a) in free_indices.iter().enumerate() {
                    let weight = effectors[col_a].weight;
                    matrix[a][a] += reg_scale / (weight * weight);
                }
                match solve_dense_system(&mut matrix, &mut rhs) {
                    Some(regularized) => regularized,
                    None => {
                        // Singular despite regularization: hold the free set
                        // at zero and report the residual, never NaN.
                        for &i in &free_indices {
                            commands[i] = 0.0;
                        }
                        solved_free.clear();
                        break;
                    }
                }
            }
        };
        // Worst violator first keeps the path deterministic; ties resolve
        // to the lowest declaration index.
        let mut worst: Option<(usize, f64, f64)> = None;
        for (a, &col) in free_indices.iter().enumerate() {
            let value = solution[a];
            let bound = if value < 0.0 {
                Some(0.0)
            } else if value > effectors[col].max_command {
                Some(effectors[col].max_command)
            } else {
                None
            };
            if let Some(clamped) = bound {
                let distance = (value - clamped).abs();
                let replace = match worst {
                    None => true,
                    Some((_, best_distance, _)) => distance > best_distance,
                };
                if replace {
                    worst = Some((col, distance, clamped));
                }
            }
        }
        if let Some((col, _, clamped)) = worst {
            free[col] = false;
            commands[col] = clamped;
            solved_free.clear();
        } else {
            solved_free = free_indices
                .iter()
                .enumerate()
                .map(|(a, &col)| (col, solution[a]))
                .collect();
            break;
        }
    }
    for (col, value) in solved_free {
        commands[col] = value.clamp(0.0, effectors[col].max_command);
    }
    let mut achieved_force = DVec3::ZERO;
    let mut achieved_moment = DVec3::ZERO;
    for (index, effector) in effectors.iter().enumerate() {
        achieved_force += effector.force_per_command_n * commands[index];
        achieved_moment += effector.moment_per_command_nm * commands[index];
    }
    let residual_force = demand.force_body_n - achieved_force;
    let residual_moment = demand.moment_body_nm - achieved_moment;
    Ok(AllocationResult {
        commands,
        achieved_force_body_n: achieved_force,
        achieved_moment_body_nm: achieved_moment,
        residual_force_body_n: residual_force,
        residual_moment_body_nm: residual_moment,
        saturated: residual_force.length_squared() + residual_moment.length_squared() > 1.0e-12,
    })
}

/// Small dense solver with partial pivoting for the allocator's free
/// subsystem. Returns `None` on (near-)singularity instead of NaN. Pivot
/// ties keep the lowest row so repeated calls stay deterministic.
fn solve_dense_system(matrix: &mut [Vec<f64>], rhs: &mut [f64]) -> Option<Vec<f64>> {
    let dim = rhs.len();
    if matrix.len() != dim || matrix.iter().any(|row| row.len() != dim) {
        return None;
    }
    for pivot in 0..dim {
        let mut best_row = pivot;
        let mut best_mag = matrix[pivot][pivot].abs();
        for (row, candidate) in matrix.iter().enumerate().skip(pivot + 1) {
            let mag = candidate[pivot].abs();
            if mag > best_mag {
                best_mag = mag;
                best_row = row;
            }
        }
        if !best_mag.is_finite() || best_mag <= 1.0e-24 {
            return None;
        }
        if best_row != pivot {
            matrix.swap(pivot, best_row);
            rhs.swap(pivot, best_row);
        }
        let diagonal = matrix[pivot][pivot];
        for row in (pivot + 1)..dim {
            let factor = matrix[row][pivot] / diagonal;
            if factor != 0.0 {
                let (before, rest) = matrix.split_at_mut(row);
                let (target, source) = (&mut rest[0], &before[pivot]);
                for (col, slot) in target.iter_mut().enumerate().skip(pivot) {
                    *slot -= factor * source[col];
                }
                rhs[row] -= factor * rhs[pivot];
            }
        }
    }
    let mut solution = vec![0.0; dim];
    for row in (0..dim).rev() {
        let mut sum = rhs[row];
        for col in (row + 1)..dim {
            sum -= matrix[row][col] * solution[col];
        }
        let diagonal = matrix[row][row];
        if !diagonal.is_finite() || diagonal.abs() <= 1.0e-24 {
            return None;
        }
        let value = sum / diagonal;
        if !value.is_finite() {
            return None;
        }
        solution[row] = value;
    }
    Some(solution)
}

#[derive(Debug, Clone, PartialEq)]
pub enum ControlError {
    NonFinite(&'static str),
    OutOfRange {
        name: &'static str,
        value: f64,
        min: f64,
        max: f64,
    },
    InvalidDirection,
    InvalidAttitude,
    InvalidTargetBody,
    InvalidEffector,
    InvalidController,
    MismatchedControlState,
    UnsupportedIntent,
}

impl fmt::Display for ControlError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonFinite(name) => write!(formatter, "{name} contains a non-finite value"),
            Self::OutOfRange {
                name,
                value,
                min,
                max,
            } => {
                write!(formatter, "{name}={value} is outside [{min}, {max}]")
            }
            Self::InvalidDirection => write!(formatter, "direction must be finite and non-zero"),
            Self::InvalidAttitude => write!(formatter, "attitude target must be a unit quaternion"),
            Self::InvalidTargetBody => {
                write!(formatter, "direction target frame and target body mismatch")
            }
            Self::InvalidEffector => write!(formatter, "effector contribution is invalid"),
            Self::InvalidController => write!(formatter, "control-law parameters are invalid"),
            Self::MismatchedControlState => {
                write!(formatter, "control law and supplied state are incompatible")
            }
            Self::UnsupportedIntent => write!(
                formatter,
                "control law does not support this guidance intent"
            ),
        }
    }
}

impl Error for ControlError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pilot_axes_validate_and_clamp_semantic_propulsion_range() {
        let axes = PilotAxes {
            pitch: 2.0,
            propulsion: -0.2,
            ..PilotAxes::default()
        };
        assert!(axes.validate().is_ok());
        assert_eq!(axes.clamped().pitch, 1.0);
    }

    #[test]
    fn guidance_rejects_non_unit_attitude() {
        let error = GuidanceIntent::Attitude {
            target_body_to_inertial: DQuat::from_xyzw(0.0, 0.0, 0.0, 2.0),
            roll_policy: RollPolicy::Hold,
        }
        .validate()
        .expect_err("non-unit target must fail");
        assert_eq!(error, ControlError::InvalidAttitude);
    }

    #[test]
    fn target_frame_without_body_fails_validation() {
        // The wire trust boundary: a deserialized Target frame with no
        // body must fail here, not later at runtime resolution.
        let bare = DirectionTarget {
            direction: DVec3::X,
            frame: DirectionFrame::Target,
            target_body: None,
        };
        assert_eq!(bare.validate(), Err(ControlError::InvalidTargetBody));
        let intent = GuidanceIntent::VelocityDirection {
            direction: bare,
            roll_policy: RollPolicy::Hold,
        };
        assert_eq!(intent.validate(), Err(ControlError::InvalidTargetBody));
        let path = GuidanceIntent::FlightPath {
            target: FlightPathTarget {
                direction: bare,
                roll_policy: RollPolicy::Hold,
            },
        };
        assert_eq!(path.validate(), Err(ControlError::InvalidTargetBody));
    }

    #[test]
    fn target_frame_with_body_and_plain_frames_validate() {
        let resolved = DirectionTarget::for_target(DVec3::X, 7).expect("resolved target builds");
        assert!(resolved.validate().is_ok());
        assert!(
            GuidanceIntent::VelocityDirection {
                direction: resolved,
                roll_policy: RollPolicy::Hold,
            }
            .validate()
            .is_ok()
        );
        // A stray body on a non-target frame is equally ambiguous.
        let mut stray =
            DirectionTarget::new(DVec3::X, DirectionFrame::Inertial).expect("inertial builds");
        stray.target_body = Some(7);
        assert_eq!(stray.validate(), Err(ControlError::InvalidTargetBody));
        // Serde round trip preserves the invariant both ways.
        let json = serde_json::to_string(&resolved).expect("serializes");
        let back: DirectionTarget = serde_json::from_str(&json).expect("deserializes");
        assert!(back.validate().is_ok());
    }

    #[test]
    fn control_demand_envelope_rejects_absurd_wire_magnitudes() {
        let huge_force = ControlDemand {
            force_body_n: DVec3::new(1.0e308, 0.0, 0.0),
            moment_body_nm: DVec3::ZERO,
            propulsion: PropulsionDemand::new(0.0).unwrap(),
        };
        // Finite-only validation passes (allocator saturates); the trust
        // boundary envelope rejects.
        assert!(huge_force.validate().is_ok());
        assert!(huge_force.validate_envelope().is_err());
        let huge_moment = ControlDemand {
            force_body_n: DVec3::ZERO,
            moment_body_nm: DVec3::new(0.0, 1.0e308, 0.0),
            propulsion: PropulsionDemand::new(0.0).unwrap(),
        };
        assert!(huge_moment.validate().is_ok());
        assert!(huge_moment.validate_envelope().is_err());
        // Sane X-15-class wrench still passes both.
        let sane = ControlDemand {
            force_body_n: DVec3::new(1.0e5, 0.0, 0.0),
            moment_body_nm: DVec3::new(0.0, 1.0e4, 0.0),
            propulsion: PropulsionDemand::new(1.0).unwrap(),
        };
        sane.validate().expect("realistic wrench must pass");
        sane.validate_envelope()
            .expect("realistic wrench must pass envelope");
    }

    #[test]
    fn policy_does_not_allow_reverse_by_default() {
        let policy = FlightPolicy::default();
        assert_eq!(
            policy
                .constrain_propulsion(PropulsionDemand::new(-0.2).unwrap(), true, true)
                .normalized,
            0.0
        );
    }

    #[test]
    fn policy_constrains_a_complete_demand_without_mutating_the_wrench() {
        let demand = ControlDemand {
            force_body_n: DVec3::new(1.0, 2.0, 3.0),
            moment_body_nm: DVec3::new(4.0, 5.0, 6.0),
            propulsion: PropulsionDemand::new(-0.2).unwrap(),
        };
        let constrained = FlightPolicy::default().constrain_demand(demand, true, true);
        assert_eq!(constrained.force_body_n, demand.force_body_n);
        assert_eq!(constrained.moment_body_nm, demand.moment_body_nm);
        assert_eq!(constrained.propulsion.normalized, 0.0);

        let augmentation = FlightPolicy {
            augmentation_allowed: true,
            ..FlightPolicy::default()
        }
        .constrain_demand(
            ControlDemand {
                propulsion: PropulsionDemand::new(1.2).unwrap(),
                ..ControlDemand::zero()
            },
            true,
            false,
        );
        assert_eq!(augmentation.propulsion.normalized, 1.2);
    }

    #[test]
    fn allocator_reports_residual_when_one_effector_is_insufficient() {
        let result = allocate_wrench(
            ControlDemand {
                moment_body_nm: DVec3::X * 10.0,
                ..ControlDemand::zero()
            },
            &[EffectorContribution {
                group: ActuatorGroup::Rcs,
                force_per_command_n: DVec3::ZERO,
                moment_per_command_nm: DVec3::X,
                max_command: 2.0,
                weight: 1.0,
            }],
        )
        .unwrap();
        assert_eq!(result.commands, vec![2.0]);
        assert!(result.saturated);
        assert_eq!(result.achieved_moment_body_nm, DVec3::X * 2.0);
    }

    #[test]
    fn allocator_is_order_invariant_on_coupled_demand() {
        // Two diagonal effectors couple X/Y force: a greedy allocator in
        // declaration order starves the second axis, the joint solve does
        // not. Permuting the inputs must permute the commands and keep the
        // achieved wrench identical.
        let demand = ControlDemand {
            force_body_n: DVec3::new(10.0, 10.0, 0.0),
            ..ControlDemand::zero()
        };
        let first = EffectorContribution {
            group: ActuatorGroup::Rcs,
            force_per_command_n: DVec3::new(1.0, 0.9, 0.0),
            moment_per_command_nm: DVec3::ZERO,
            max_command: 10.0,
            weight: 1.0,
        };
        let second = EffectorContribution {
            group: ActuatorGroup::Rcs,
            force_per_command_n: DVec3::new(0.9, 1.0, 0.0),
            moment_per_command_nm: DVec3::ZERO,
            max_command: 10.0,
            weight: 1.0,
        };
        let forward = allocate_wrench(demand, &[first, second]).unwrap();
        let reversed = allocate_wrench(demand, &[second, first]).unwrap();
        assert!(!forward.saturated);
        assert!(!reversed.saturated);
        assert!((forward.achieved_force_body_n - demand.force_body_n).length() < 1.0e-6);
        assert!((reversed.achieved_force_body_n - demand.force_body_n).length() < 1.0e-6);
        assert!((forward.commands[0] - reversed.commands[1]).abs() < 1.0e-9);
        assert!((forward.commands[1] - reversed.commands[0]).abs() < 1.0e-9);
    }

    #[test]
    fn allocator_uses_only_the_correct_side_of_an_opposed_pair() {
        // Starter-vehicle pattern: +X and -X jets as separate entries.
        // A +800 N request must not fire the opposing jet.
        let demand = ControlDemand {
            force_body_n: DVec3::X * 400.0,
            ..ControlDemand::zero()
        };
        let pair = [
            EffectorContribution {
                group: ActuatorGroup::Rcs,
                force_per_command_n: DVec3::X * 800.0,
                moment_per_command_nm: DVec3::ZERO,
                max_command: 1.0,
                weight: 1.0,
            },
            EffectorContribution {
                group: ActuatorGroup::Rcs,
                force_per_command_n: DVec3::X * -800.0,
                moment_per_command_nm: DVec3::ZERO,
                max_command: 1.0,
                weight: 1.0,
            },
        ];
        let result = allocate_wrench(demand, &pair).unwrap();
        assert!(!result.saturated);
        assert!((result.commands[0] - 0.5).abs() < 1.0e-9);
        assert_eq!(result.commands[1], 0.0);
        assert!((result.achieved_force_body_n - demand.force_body_n).length() < 1.0e-6);
    }

    #[test]
    fn allocator_prefers_higher_weight_when_redundant() {
        // Identical columns: residual is zero either way, so the cheaper
        // (higher-weight) effector must take the load.
        let demand = ControlDemand {
            force_body_n: DVec3::X * 100.0,
            ..ControlDemand::zero()
        };
        let cheap = EffectorContribution {
            group: ActuatorGroup::Rcs,
            force_per_command_n: DVec3::X * 100.0,
            moment_per_command_nm: DVec3::ZERO,
            max_command: 2.0,
            weight: 4.0,
        };
        let pricey = EffectorContribution {
            group: ActuatorGroup::Rcs,
            force_per_command_n: DVec3::X * 100.0,
            moment_per_command_nm: DVec3::ZERO,
            max_command: 2.0,
            weight: 1.0,
        };
        let result = allocate_wrench(demand, &[cheap, pricey]).unwrap();
        assert!(!result.saturated);
        assert!(result.commands[0] > result.commands[1]);
        assert!((result.achieved_force_body_n - demand.force_body_n).length() < 1.0e-6);
    }

    #[test]
    fn spacecraft_controller_outputs_moment_without_selecting_an_actuator() {
        let law = SpacecraftControlLaw::default();
        let demand = law
            .control_demand(
                AttitudeState {
                    orientation_body_to_inertial: DQuat::IDENTITY,
                    angular_velocity_body_rps: DVec3::ZERO,
                    inertia_body_kg_m2: DMat3::from_diagonal(DVec3::splat(2.0)),
                },
                &GuidanceIntent::AngularRate {
                    rate_body_rps: DVec3::X,
                },
                PropulsionDemand::new(0.0).unwrap(),
            )
            .unwrap();
        assert_eq!(demand.force_body_n, DVec3::ZERO);
        assert_eq!(demand.moment_body_nm, DVec3::X * (2.0 / 0.35));
    }

    #[test]
    fn spacecraft_controller_maps_manual_translation_to_body_force() {
        let demand = SpacecraftControlLaw::default()
            .control_demand(
                AttitudeState {
                    orientation_body_to_inertial: DQuat::IDENTITY,
                    angular_velocity_body_rps: DVec3::ZERO,
                    inertia_body_kg_m2: DMat3::from_diagonal(DVec3::splat(2.0)),
                },
                &GuidanceIntent::ManualAxes(PilotAxes {
                    translation: DVec3::new(0.5, -1.0, 0.25),
                    ..PilotAxes::default()
                }),
                PropulsionDemand::new(0.0).unwrap(),
            )
            .unwrap();
        assert_eq!(demand.force_body_n, DVec3::new(400.0, -800.0, 200.0));
    }

    #[test]
    fn unified_dispatcher_realizes_direct_axes_as_a_physical_demand() {
        let demand = FlightControlLaw::Direct(DirectControlLaw::default())
            .control_demand(
                ControlLawState::Spacecraft(AttitudeState {
                    orientation_body_to_inertial: DQuat::IDENTITY,
                    angular_velocity_body_rps: DVec3::ZERO,
                    inertia_body_kg_m2: DMat3::from_diagonal(DVec3::splat(2.0)),
                }),
                &GuidanceIntent::ManualAxes(PilotAxes {
                    pitch: 0.5,
                    translation: DVec3::new(0.25, 0.0, -0.5),
                    propulsion: 0.75,
                    ..PilotAxes::default()
                }),
                PropulsionDemand::new(0.75).unwrap(),
            )
            .unwrap();
        assert_eq!(demand.force_body_n, DVec3::new(200.0, 0.0, -400.0));
        assert_eq!(demand.moment_body_nm, DVec3::new(0.0, -200.0, 0.0));
    }

    #[test]
    fn unified_dispatcher_rejects_a_mismatched_aircraft_state() {
        let error = FlightControlLaw::Aircraft(AircraftControlLaw::default())
            .control_demand(
                ControlLawState::Spacecraft(AttitudeState {
                    orientation_body_to_inertial: DQuat::IDENTITY,
                    angular_velocity_body_rps: DVec3::ZERO,
                    inertia_body_kg_m2: DMat3::from_diagonal(DVec3::splat(2.0)),
                }),
                &GuidanceIntent::ManualAxes(PilotAxes::default()),
                PropulsionDemand::new(0.0).unwrap(),
            )
            .expect_err("aircraft law must not consume spacecraft state");
        assert_eq!(error, ControlError::MismatchedControlState);
    }

    #[test]
    fn aircraft_controller_applies_aoa_and_load_factor_protection() {
        let state = AircraftState {
            attitude: AttitudeState {
                orientation_body_to_inertial: DQuat::IDENTITY,
                angular_velocity_body_rps: DVec3::ZERO,
                inertia_body_kg_m2: DMat3::from_diagonal(DVec3::splat(2.0)),
            },
            air_velocity_body_mps: DVec3::new(100.0, 0.0, -100.0),
            load_factor_g: 3.0,
        };
        let law = AircraftControlLaw {
            max_aoa_rad: Some(0.1),
            max_positive_g: Some(2.0),
            ..AircraftControlLaw::default()
        };
        let demand = law
            .control_demand(
                state,
                &GuidanceIntent::ManualAxes(PilotAxes {
                    pitch: 1.0,
                    translation: DVec3::X,
                    ..PilotAxes::default()
                }),
                PropulsionDemand::new(0.0).unwrap(),
            )
            .unwrap();
        assert_eq!(demand.moment_body_nm.y, 0.0);
        assert_eq!(demand.force_body_n, DVec3::ZERO);
    }

    #[test]
    fn envelope_protection_preserves_pitch_recovery_direction_symmetrically() {
        let policy = FlightPolicy {
            max_aoa_rad: Some(0.1),
            max_positive_g: Some(2.0),
            max_negative_g: Some(2.0),
            ..FlightPolicy::default()
        };
        let demand = |pitch_moment: f64, aoa: f64, load_factor_g: f64| {
            policy
                .constrain_demand_with_context(
                    ControlDemand {
                        moment_body_nm: DVec3::new(1.0, pitch_moment, 3.0),
                        ..ControlDemand::zero()
                    },
                    true,
                    true,
                    FlightPolicyContext {
                        angle_of_attack_rad: aoa,
                        load_factor_g,
                    },
                )
                .unwrap()
                .moment_body_nm
        };

        // Positive AoA is reduced by +Y (nose-down), while -Y would deepen
        // it and is clipped. The negative-AoA case is the mirror image.
        assert_eq!(demand(2.0, 0.2, 0.0).y, 2.0);
        assert_eq!(demand(-2.0, 0.2, 0.0).y, 0.0);
        assert_eq!(demand(-2.0, -0.2, 0.0).y, -2.0);
        assert_eq!(demand(2.0, -0.2, 0.0).y, 0.0);

        // The g-limit has the same recovery sign convention and must not
        // remove the allowed direction when the opposite edge is exceeded.
        assert_eq!(demand(2.0, 0.0, 3.0).y, 2.0);
        assert_eq!(demand(-2.0, 0.0, 3.0).y, 0.0);
        assert_eq!(demand(-2.0, 0.0, -3.0).y, -2.0);
        assert_eq!(demand(2.0, 0.0, -3.0).y, 0.0);
    }

    #[test]
    fn envelope_ramp_is_continuous_and_hits_both_endpoints() {
        // Outer 10% of each limit ramps outward torque to zero instead of
        // switching: with limit 0.1 the band is [0.09, 0.1].
        let policy = FlightPolicy {
            max_aoa_rad: Some(0.1),
            max_positive_g: Some(2.0),
            max_negative_g: Some(2.0),
            ..FlightPolicy::default()
        };
        let pitch_at_aoa = |moment: f64, aoa: f64| {
            policy
                .constrain_demand_with_context(
                    ControlDemand {
                        moment_body_nm: DVec3::new(0.0, moment, 0.0),
                        ..ControlDemand::zero()
                    },
                    true,
                    true,
                    FlightPolicyContext {
                        angle_of_attack_rad: aoa,
                        load_factor_g: 0.0,
                    },
                )
                .unwrap()
                .moment_body_nm
                .y
        };
        // Deep inside: untouched; at/past the limit: zero (old endpoints).
        assert_eq!(pitch_at_aoa(-2.0, 0.05), -2.0);
        assert_eq!(pitch_at_aoa(-2.0, 0.09), -2.0);
        assert_eq!(pitch_at_aoa(-2.0, 0.1), 0.0);
        assert_eq!(pitch_at_aoa(-2.0, 0.2), 0.0);
        // Mid-band: half torque; recovery direction never touched.
        assert!((pitch_at_aoa(-2.0, 0.095) + 1.0).abs() < 1.0e-9);
        assert_eq!(pitch_at_aoa(2.0, 0.095), 2.0);
        // Monotone magnitude across the band: no discrete jump means no
        // boundary chatter by construction.
        let mut previous = pitch_at_aoa(-2.0, 0.085).abs();
        for step in 1..=30 {
            let aoa = 0.085 + 0.001 * step as f64;
            let current = pitch_at_aoa(-2.0, aoa).abs();
            assert!(
                current <= previous + 1.0e-9,
                "outward torque magnitude must not grow toward the limit"
            );
            previous = current;
        }
    }

    #[test]
    fn actuator_dynamics_respects_response_rate_and_command_bounds() {
        let dynamics = ActuatorDynamics {
            response_s: 0.5,
            max_rate_per_s: Some(0.4),
            min_command: -1.0,
            max_command: 1.0,
        };
        let first = dynamics.advance(0.0, 1.0, 0.5).unwrap();
        assert!((first - 0.2).abs() < 1.0e-12);
        let settled = dynamics.advance(first, 4.0, 20.0).unwrap();
        assert_eq!(settled, 1.0);
        assert_eq!(dynamics.advance(0.0, -4.0, 20.0).unwrap(), -1.0);
    }
}
