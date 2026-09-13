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
}

impl DirectionTarget {
    pub fn new(direction: DVec3, frame: DirectionFrame) -> Result<Self, ControlError> {
        if !direction.is_finite() || direction.length_squared() <= 1.0e-12 {
            return Err(ControlError::InvalidDirection);
        }
        Ok(Self {
            direction: direction.normalize(),
            frame,
        })
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
            Self::VelocityDirection { direction, .. } => {
                DirectionTarget::new(direction.direction, direction.frame).map(|_| ())
            }
            Self::FlightPath { target } => {
                DirectionTarget::new(target.direction.direction, target.direction.frame).map(|_| ())
            }
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

impl ControlDemand {
    pub fn zero() -> Self {
        Self {
            force_body_n: DVec3::ZERO,
            moment_body_nm: DVec3::ZERO,
            propulsion: PropulsionDemand { normalized: 0.0 },
        }
    }

    pub fn validate(self) -> Result<(), ControlError> {
        if !self.force_body_n.is_finite() || !self.moment_body_nm.is_finite() {
            return Err(ControlError::NonFinite("control demand"));
        }
        PropulsionDemand::new(self.propulsion.normalized).map(|_| ())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct AircraftControlLaw {
    pub max_aoa_rad: Option<f64>,
    pub max_positive_g: Option<f64>,
    pub max_negative_g: Option<f64>,
    pub coordinated_turn_assist: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SpacecraftControlLaw {
    pub attitude_response_s: f64,
    pub translation_response_s: f64,
    pub max_rate_rps: Option<f64>,
}

impl Default for SpacecraftControlLaw {
    fn default() -> Self {
        Self {
            attitude_response_s: 0.35,
            translation_response_s: 0.5,
            max_rate_rps: None,
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
        if !self.attitude_response_s.is_finite() || self.attitude_response_s <= 0.0 {
            return Err(ControlError::InvalidController);
        }
        let desired_rate = match intent {
            GuidanceIntent::ManualAxes(axes) => {
                DVec3::new(axes.roll, -axes.pitch, -axes.yaw) * 0.16
            }
            GuidanceIntent::AngularRate { rate_body_rps } => *rate_body_rps,
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
                rate
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
            force_body_n: DVec3::ZERO,
            moment_body_nm: moment,
            propulsion,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct DirectControlLaw;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum FlightControlLaw {
    Aircraft(AircraftControlLaw),
    Spacecraft(SpacecraftControlLaw),
    Direct(DirectControlLaw),
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
    InvalidEffector,
    InvalidController,
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
            Self::InvalidEffector => write!(formatter, "effector contribution is invalid"),
            Self::InvalidController => write!(formatter, "control-law parameters are invalid"),
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
}
