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
    /// complete demand. At an active AoA or load-factor limit, pitch torque
    /// is removed while the allocator remains free to realize roll/yaw and
    /// any still-permitted translation.
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
        let pitch_pushes_aoa_outward = self.max_aoa_rad.is_some_and(|limit| {
            context.angle_of_attack_rad.abs() >= limit
                && ((context.angle_of_attack_rad > 0.0 && demand.moment_body_nm.y < 0.0)
                    || (context.angle_of_attack_rad < 0.0 && demand.moment_body_nm.y > 0.0))
        });
        // With +X forward and +Z up, +Y is nose-down. Positive load factor
        // is reduced by nose-down torque; negative load factor is reduced by
        // nose-up torque. Keep the recovery direction available at either
        // envelope edge instead of zeroing the whole pitch channel.
        let pitch_pushes_g_outward = self
            .max_positive_g
            .is_some_and(|limit| context.load_factor_g >= limit && demand.moment_body_nm.y < 0.0)
            || self.max_negative_g.is_some_and(|limit| {
                context.load_factor_g <= -limit && demand.moment_body_nm.y > 0.0
            });
        if pitch_pushes_aoa_outward || pitch_pushes_g_outward {
            demand.moment_body_nm.y = 0.0;
        }
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

/// Deterministic first allocator implementation. It greedily projects the
/// remaining wrench onto each available effector in declaration order. The
/// contract is intentionally generic; the runtime may replace this policy
/// with a constrained least-squares solver without changing control laws.
pub fn allocate_wrench(
    demand: ControlDemand,
    effectors: &[EffectorContribution],
) -> Result<AllocationResult, ControlError> {
    demand.validate()?;
    for effector in effectors {
        effector.validate()?;
    }
    let mut commands = vec![0.0; effectors.len()];
    let mut achieved_force = DVec3::ZERO;
    let mut achieved_moment = DVec3::ZERO;
    for (index, effector) in effectors.iter().enumerate() {
        let remaining_force = demand.force_body_n - achieved_force;
        let remaining_moment = demand.moment_body_nm - achieved_moment;
        let force_scale = effector.force_per_command_n.length_squared();
        let moment_scale = effector.moment_per_command_nm.length_squared();
        let denominator = force_scale + moment_scale;
        if denominator <= 1.0e-24 {
            continue;
        }
        let projection = (remaining_force.dot(effector.force_per_command_n)
            + remaining_moment.dot(effector.moment_per_command_nm))
            / denominator;
        let command = (projection * effector.weight).clamp(0.0, effector.max_command);
        commands[index] = command;
        achieved_force += effector.force_per_command_n * command;
        achieved_moment += effector.moment_per_command_nm * command;
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
            Self::InvalidTargetBody => write!(
                formatter,
                "direction target frame and target body mismatch"
            ),
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
        let resolved =
            DirectionTarget::for_target(DVec3::X, 7).expect("resolved target builds");
        assert!(resolved.validate().is_ok());
        assert!(GuidanceIntent::VelocityDirection {
            direction: resolved,
            roll_policy: RollPolicy::Hold,
        }
        .validate()
        .is_ok());
        // A stray body on a non-target frame is equally ambiguous.
        let mut stray = DirectionTarget::new(DVec3::X, DirectionFrame::Inertial)
            .expect("inertial builds");
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
