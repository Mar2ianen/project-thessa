//! Authoritative docking-port compatibility and state progression.
//!
//! This module intentionally does not own a contact solver.  It describes the
//! physical event that the flight layer asks the collision backend to enforce:
//! Rapier supplies the hard mechanical constraint, while this state machine
//! owns the docking protocol and its persistence-friendly state.

use std::{error::Error, fmt};

use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};

use crate::flight::RigidBodyState;

const QUATERNION_TOLERANCE: f64 = 1.0e-6;
const MIN_EQUALIZATION_S: f64 = 1.0e-6;
const DEFAULT_ALIGNMENT_ANGULAR_VELOCITY_LIMIT_RPS: f64 = 0.05;

fn default_alignment_angular_velocity_limit_rps() -> f64 {
    DEFAULT_ALIGNMENT_ANGULAR_VELOCITY_LIMIT_RPS
}

/// Common pressurized docking family used by the D1 prototype.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DockingPortClass {
    D1Pressurized,
}

impl DockingPortClass {
    pub const fn family(self) -> &'static str {
        match self {
            Self::D1Pressurized => "standard_pressurized",
        }
    }
}

/// Persisted progression of one docking pair.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DockingPortState {
    Free,
    SoftCapture,
    Aligned,
    HardDock,
    OuterStructureEngaged,
    PressureEqualized,
}

impl DockingPortState {
    pub const fn mechanically_connected(self) -> bool {
        matches!(
            self,
            Self::HardDock | Self::OuterStructureEngaged | Self::PressureEqualized
        )
    }
}

/// A port's stable identity and body-local docking frame.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DockingPortSpec {
    pub id: String,
    pub class: DockingPortClass,
    pub local_position_m: DVec3,
    pub local_orientation: DQuat,
    /// Maximum closing speed accepted by the soft-capture latches.
    pub capture_speed_limit_mps: f64,
    /// Maximum residual port-frame offset accepted by alignment.
    pub alignment_position_tolerance_m: f64,
    /// Maximum residual angular error accepted by alignment.
    pub alignment_angle_tolerance_rad: f64,
    /// Maximum relative angular velocity accepted before the hard-dock
    /// constraint is installed.
    #[serde(default = "default_alignment_angular_velocity_limit_rps")]
    pub alignment_angular_velocity_limit_rps: f64,
}

impl DockingPortSpec {
    pub fn d1(
        id: impl Into<String>,
        local_position_m: DVec3,
        local_orientation: DQuat,
    ) -> Result<Self, DockingError> {
        Self::new(
            id,
            DockingPortClass::D1Pressurized,
            local_position_m,
            local_orientation,
            0.05,
            0.025,
            2.0_f64.to_radians(),
        )
    }

    #[allow(clippy::too_many_arguments)]
    pub fn new(
        id: impl Into<String>,
        class: DockingPortClass,
        local_position_m: DVec3,
        local_orientation: DQuat,
        capture_speed_limit_mps: f64,
        alignment_position_tolerance_m: f64,
        alignment_angle_tolerance_rad: f64,
    ) -> Result<Self, DockingError> {
        let spec = Self {
            id: id.into(),
            class,
            local_position_m,
            local_orientation,
            capture_speed_limit_mps,
            alignment_position_tolerance_m,
            alignment_angle_tolerance_rad,
            alignment_angular_velocity_limit_rps: DEFAULT_ALIGNMENT_ANGULAR_VELOCITY_LIMIT_RPS,
        };
        spec.validate()?;
        Ok(spec)
    }

    pub fn validate(&self) -> Result<(), DockingError> {
        if self.id.trim().is_empty() {
            return Err(DockingError::InvalidPort(
                "port id must not be empty".into(),
            ));
        }
        if !self.local_position_m.is_finite() || !self.local_orientation.is_finite() {
            return Err(DockingError::InvalidPort(
                "port frame contains a non-finite value".into(),
            ));
        }
        if (self.local_orientation.length_squared() - 1.0).abs() > QUATERNION_TOLERANCE {
            return Err(DockingError::InvalidPort(
                "port orientation must be a unit quaternion".into(),
            ));
        }
        if !self.capture_speed_limit_mps.is_finite() || self.capture_speed_limit_mps <= 0.0 {
            return Err(DockingError::InvalidPort(
                "capture speed limit must be finite and positive".into(),
            ));
        }
        if !self.alignment_position_tolerance_m.is_finite()
            || self.alignment_position_tolerance_m <= 0.0
            || !self.alignment_angle_tolerance_rad.is_finite()
            || self.alignment_angle_tolerance_rad <= 0.0
            || !self.alignment_angular_velocity_limit_rps.is_finite()
            || self.alignment_angular_velocity_limit_rps <= 0.0
        {
            return Err(DockingError::InvalidPort(
                "alignment tolerances must be finite and positive".into(),
            ));
        }
        Ok(())
    }
}

/// Relative port-frame evidence supplied by the flight/pose layer.
///
/// Callers must construct this with [`DockingKinematics::between`]. That
/// factory accounts for each port's body-local frame, the port lever arm, and
/// body angular velocity before a docking transition can be accepted.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DockingKinematics {
    relative_position_m: DVec3,
    relative_orientation: DQuat,
    relative_velocity_mps: DVec3,
    relative_angular_velocity_rps: DVec3,
}

impl DockingKinematics {
    pub fn between(
        state_a: RigidBodyState,
        port_a: &DockingPortSpec,
        state_b: RigidBodyState,
        port_b: &DockingPortSpec,
    ) -> Result<Self, DockingError> {
        port_a.validate()?;
        port_b.validate()?;
        validate_state(state_a, "body A")?;
        validate_state(state_b, "body B")?;

        let port_a_position = state_a.position_inertial_m
            + state_a.orientation_body_to_inertial * port_a.local_position_m;
        let port_b_position = state_b.position_inertial_m
            + state_b.orientation_body_to_inertial * port_b.local_position_m;
        let port_a_orientation =
            (state_a.orientation_body_to_inertial * port_a.local_orientation).normalize();
        let port_b_orientation =
            (state_b.orientation_body_to_inertial * port_b.local_orientation).normalize();
        let omega_a_world =
            state_a.orientation_body_to_inertial * state_a.angular_velocity_body_rps;
        let omega_b_world =
            state_b.orientation_body_to_inertial * state_b.angular_velocity_body_rps;
        let port_a_velocity = state_a.velocity_inertial_mps
            + omega_a_world.cross(port_a_position - state_a.position_inertial_m);
        let port_b_velocity = state_b.velocity_inertial_mps
            + omega_b_world.cross(port_b_position - state_b.position_inertial_m);
        let kinematics = Self {
            relative_position_m: port_b_position - port_a_position,
            relative_orientation: (port_a_orientation.inverse() * port_b_orientation).normalize(),
            relative_velocity_mps: port_b_velocity - port_a_velocity,
            relative_angular_velocity_rps: omega_b_world - omega_a_world,
        };
        if !kinematics.relative_position_m.is_finite()
            || !kinematics.relative_orientation.is_finite()
            || !kinematics.relative_velocity_mps.is_finite()
            || !kinematics.relative_angular_velocity_rps.is_finite()
        {
            return Err(DockingError::InvalidKinematics(
                "relative docking kinematics contain a non-finite value".into(),
            ));
        }
        if (kinematics.relative_orientation.length_squared() - 1.0).abs() > QUATERNION_TOLERANCE {
            return Err(DockingError::InvalidKinematics(
                "relative docking orientation must be a unit quaternion".into(),
            ));
        }
        Ok(kinematics)
    }

    pub const fn relative_position_m(self) -> DVec3 {
        self.relative_position_m
    }

    pub const fn relative_orientation(self) -> DQuat {
        self.relative_orientation
    }

    pub const fn relative_velocity_mps(self) -> DVec3 {
        self.relative_velocity_mps
    }

    pub const fn relative_angular_velocity_rps(self) -> DVec3 {
        self.relative_angular_velocity_rps
    }

    fn angle_error_rad(self) -> f64 {
        let w = self.relative_orientation.w.abs().clamp(-1.0, 1.0);
        2.0 * w.acos()
    }
}

/// One compatible port pair and its deterministic docking protocol state.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DockingSession {
    pub port_a: DockingPortSpec,
    pub port_b: DockingPortSpec,
    pub state: DockingPortState,
    pub pressure_equalization_elapsed_s: f64,
    pub pressure_equalization_required_s: f64,
}

impl DockingSession {
    pub fn new(
        port_a: DockingPortSpec,
        port_b: DockingPortSpec,
        pressure_equalization_required_s: f64,
    ) -> Result<Self, DockingError> {
        port_a.validate()?;
        port_b.validate()?;
        if port_a.id == port_b.id {
            return Err(DockingError::IncompatiblePorts(
                "a docking session needs two distinct port ids".into(),
            ));
        }
        if port_a.class != port_b.class {
            return Err(DockingError::IncompatiblePorts(format!(
                "port classes do not match: {:?} vs {:?}",
                port_a.class, port_b.class
            )));
        }
        if !pressure_equalization_required_s.is_finite()
            || pressure_equalization_required_s < MIN_EQUALIZATION_S
        {
            return Err(DockingError::InvalidPressureDuration(
                pressure_equalization_required_s,
            ));
        }
        Ok(Self {
            port_a,
            port_b,
            state: DockingPortState::Free,
            pressure_equalization_elapsed_s: 0.0,
            pressure_equalization_required_s,
        })
    }

    pub fn begin_soft_capture(&mut self, relative_speed_mps: f64) -> Result<(), DockingError> {
        self.require_state(DockingPortState::Free)?;
        if !relative_speed_mps.is_finite() || relative_speed_mps < 0.0 {
            return Err(DockingError::InvalidKinematics(
                "relative speed must be finite and non-negative".into(),
            ));
        }
        let limit = self
            .port_a
            .capture_speed_limit_mps
            .min(self.port_b.capture_speed_limit_mps);
        if relative_speed_mps > limit {
            return Err(DockingError::CaptureSpeedExceeded {
                speed_mps: relative_speed_mps,
                limit_mps: limit,
            });
        }
        self.state = DockingPortState::SoftCapture;
        Ok(())
    }

    pub fn align(&mut self, kinematics: DockingKinematics) -> Result<(), DockingError> {
        self.require_state(DockingPortState::SoftCapture)?;
        let position_tolerance = self
            .port_a
            .alignment_position_tolerance_m
            .min(self.port_b.alignment_position_tolerance_m);
        let angle_tolerance = self
            .port_a
            .alignment_angle_tolerance_rad
            .min(self.port_b.alignment_angle_tolerance_rad);
        let speed_limit = self
            .port_a
            .capture_speed_limit_mps
            .min(self.port_b.capture_speed_limit_mps);
        let angular_velocity_limit = self
            .port_a
            .alignment_angular_velocity_limit_rps
            .min(self.port_b.alignment_angular_velocity_limit_rps);
        let position_error = kinematics.relative_position_m.length();
        let angle_error = kinematics.angle_error_rad();
        let speed = kinematics.relative_velocity_mps.length();
        let angular_speed = kinematics.relative_angular_velocity_rps.length();
        if position_error > position_tolerance
            || angle_error > angle_tolerance
            || speed > speed_limit
            || angular_speed > angular_velocity_limit
        {
            return Err(DockingError::AlignmentOutOfTolerance {
                position_error_m: position_error,
                angle_error_rad: angle_error,
                relative_speed_mps: speed,
                relative_angular_velocity_rps: angular_speed,
            });
        }
        self.state = DockingPortState::Aligned;
        Ok(())
    }

    pub fn hard_dock(&mut self) -> Result<(), DockingError> {
        self.require_state(DockingPortState::Aligned)?;
        self.state = DockingPortState::HardDock;
        Ok(())
    }

    pub fn engage_outer_structure(&mut self) -> Result<(), DockingError> {
        self.require_state(DockingPortState::HardDock)?;
        self.state = DockingPortState::OuterStructureEngaged;
        Ok(())
    }

    pub fn advance_pressure_equalization(&mut self, step_s: f64) -> Result<(), DockingError> {
        if self.state != DockingPortState::OuterStructureEngaged {
            return Err(DockingError::InvalidTransition {
                from: self.state,
                expected: DockingPortState::OuterStructureEngaged,
            });
        }
        if !step_s.is_finite() || step_s <= 0.0 {
            return Err(DockingError::InvalidPressureDuration(step_s));
        }
        self.pressure_equalization_elapsed_s = (self.pressure_equalization_elapsed_s + step_s)
            .min(self.pressure_equalization_required_s);
        if self.pressure_equalization_elapsed_s >= self.pressure_equalization_required_s {
            self.state = DockingPortState::PressureEqualized;
        }
        Ok(())
    }

    pub fn undock(&mut self) -> Result<(), DockingError> {
        if !self.state.mechanically_connected() {
            return Err(DockingError::InvalidTransition {
                from: self.state,
                expected: DockingPortState::HardDock,
            });
        }
        self.state = DockingPortState::Free;
        self.pressure_equalization_elapsed_s = 0.0;
        Ok(())
    }

    fn require_state(&self, expected: DockingPortState) -> Result<(), DockingError> {
        if self.state != expected {
            return Err(DockingError::InvalidTransition {
                from: self.state,
                expected,
            });
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum DockingError {
    InvalidPort(String),
    InvalidKinematics(String),
    IncompatiblePorts(String),
    InvalidPressureDuration(f64),
    CaptureSpeedExceeded {
        speed_mps: f64,
        limit_mps: f64,
    },
    AlignmentOutOfTolerance {
        position_error_m: f64,
        angle_error_rad: f64,
        relative_speed_mps: f64,
        relative_angular_velocity_rps: f64,
    },
    InvalidTransition {
        from: DockingPortState,
        expected: DockingPortState,
    },
}

impl fmt::Display for DockingError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidPort(message) => write!(formatter, "invalid docking port: {message}"),
            Self::InvalidKinematics(message) => {
                write!(formatter, "invalid docking kinematics: {message}")
            }
            Self::IncompatiblePorts(message) => {
                write!(formatter, "incompatible docking ports: {message}")
            }
            Self::InvalidPressureDuration(value) => {
                write!(formatter, "invalid pressure equalization duration: {value}")
            }
            Self::CaptureSpeedExceeded {
                speed_mps,
                limit_mps,
            } => write!(
                formatter,
                "soft capture speed {speed_mps} m/s exceeds limit {limit_mps} m/s"
            ),
            Self::AlignmentOutOfTolerance {
                position_error_m,
                angle_error_rad,
                relative_speed_mps,
                relative_angular_velocity_rps,
            } => write!(
                formatter,
                "alignment out of tolerance: position {position_error_m} m, angle {angle_error_rad} rad, speed {relative_speed_mps} m/s, angular speed {relative_angular_velocity_rps} rad/s"
            ),
            Self::InvalidTransition { from, expected } => {
                write!(
                    formatter,
                    "invalid docking transition from {from:?}; expected {expected:?}"
                )
            }
        }
    }
}

impl Error for DockingError {}

fn validate_state(state: RigidBodyState, label: &str) -> Result<(), DockingError> {
    if !state.position_inertial_m.is_finite()
        || !state.velocity_inertial_mps.is_finite()
        || !state.orientation_body_to_inertial.is_finite()
        || !state.angular_velocity_body_rps.is_finite()
    {
        return Err(DockingError::InvalidKinematics(format!(
            "{label} state contains a non-finite value"
        )));
    }
    if (state.orientation_body_to_inertial.length_squared() - 1.0).abs() > QUATERNION_TOLERANCE {
        return Err(DockingError::InvalidKinematics(format!(
            "{label} orientation must be a unit quaternion"
        )));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn d1_session_reaches_pressure_equalized_and_can_undock() {
        let a = DockingPortSpec::d1("craft-a-port", DVec3::X, DQuat::IDENTITY).unwrap();
        let b = DockingPortSpec::d1("craft-b-port", DVec3::NEG_X, DQuat::IDENTITY).unwrap();
        let mut session = DockingSession::new(a, b, 2.0).unwrap();
        session.begin_soft_capture(0.02).unwrap();
        let state_a = RigidBodyState::stationary(DVec3::ZERO);
        let state_b = RigidBodyState::new(
            DVec3::new(2.005, 0.0, 0.0),
            DVec3::new(0.01, 0.0, 0.0),
            DQuat::IDENTITY,
            DVec3::ZERO,
        )
        .unwrap();
        let port_a = session.port_a.clone();
        let port_b = session.port_b.clone();
        session
            .align(DockingKinematics::between(state_a, &port_a, state_b, &port_b).unwrap())
            .unwrap();
        session.hard_dock().unwrap();
        session.engage_outer_structure().unwrap();
        session.advance_pressure_equalization(1.0).unwrap();
        assert_eq!(session.state, DockingPortState::OuterStructureEngaged);
        session.advance_pressure_equalization(1.0).unwrap();
        assert_eq!(session.state, DockingPortState::PressureEqualized);
        session.undock().unwrap();
        assert_eq!(session.state, DockingPortState::Free);
    }

    #[test]
    fn kinematics_use_port_frames_and_lever_arm_velocity() {
        let quarter_turn = std::f64::consts::FRAC_PI_2;
        let port_a = DockingPortSpec::d1(
            "craft-a-port",
            DVec3::X,
            DQuat::from_rotation_z(-quarter_turn),
        )
        .unwrap();
        let port_b = DockingPortSpec::d1(
            "craft-b-port",
            DVec3::NEG_X,
            DQuat::from_rotation_z(quarter_turn),
        )
        .unwrap();
        let state_a = RigidBodyState::new(
            DVec3::ZERO,
            DVec3::ZERO,
            DQuat::from_rotation_z(quarter_turn),
            DVec3::Z,
        )
        .unwrap();
        let state_b = RigidBodyState::new(
            DVec3::ZERO,
            DVec3::ZERO,
            DQuat::from_rotation_z(-quarter_turn),
            DVec3::ZERO,
        )
        .unwrap();

        let kinematics = DockingKinematics::between(state_a, &port_a, state_b, &port_b).unwrap();
        assert!(kinematics.relative_position_m().length() < 1.0e-12);
        assert!(
            kinematics
                .relative_orientation()
                .angle_between(DQuat::IDENTITY)
                < 1.0e-12
        );
        assert!((kinematics.relative_velocity_mps() - DVec3::X).length() < 1.0e-12);
        assert!((kinematics.relative_angular_velocity_rps() + DVec3::Z).length() < 1.0e-12);
    }
}
