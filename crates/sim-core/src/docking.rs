//! Authoritative docking-port compatibility and state progression.
//!
//! This module intentionally does not own a contact solver.  It describes the
//! physical event that the flight layer asks the collision backend to enforce:
//! Rapier supplies the hard mechanical constraint, while this state machine
//! owns the docking protocol and its persistence-friendly state.

use std::{error::Error, fmt};

use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};

const QUATERNION_TOLERANCE: f64 = 1.0e-6;
const MIN_EQUALIZATION_S: f64 = 1.0e-6;

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
        {
            return Err(DockingError::InvalidPort(
                "alignment tolerances must be finite and positive".into(),
            ));
        }
        Ok(())
    }
}

/// Relative port-frame evidence supplied by the flight/pose layer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DockingKinematics {
    pub relative_position_m: DVec3,
    pub relative_orientation: DQuat,
    pub relative_velocity_mps: DVec3,
}

impl DockingKinematics {
    pub fn new(
        relative_position_m: DVec3,
        relative_orientation: DQuat,
        relative_velocity_mps: DVec3,
    ) -> Result<Self, DockingError> {
        let kinematics = Self {
            relative_position_m,
            relative_orientation,
            relative_velocity_mps,
        };
        if !kinematics.relative_position_m.is_finite()
            || !kinematics.relative_orientation.is_finite()
            || !kinematics.relative_velocity_mps.is_finite()
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
        let position_error = kinematics.relative_position_m.length();
        let angle_error = kinematics.angle_error_rad();
        let speed = kinematics.relative_velocity_mps.length();
        if position_error > position_tolerance
            || angle_error > angle_tolerance
            || speed > speed_limit
        {
            return Err(DockingError::AlignmentOutOfTolerance {
                position_error_m: position_error,
                angle_error_rad: angle_error,
                relative_speed_mps: speed,
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
            } => write!(
                formatter,
                "alignment out of tolerance: position {position_error_m} m, angle {angle_error_rad} rad, speed {relative_speed_mps} m/s"
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn d1_session_reaches_pressure_equalized_and_can_undock() {
        let a = DockingPortSpec::d1("craft-a-port", DVec3::X, DQuat::IDENTITY).unwrap();
        let b = DockingPortSpec::d1("craft-b-port", DVec3::NEG_X, DQuat::IDENTITY).unwrap();
        let mut session = DockingSession::new(a, b, 2.0).unwrap();
        session.begin_soft_capture(0.02).unwrap();
        session
            .align(
                DockingKinematics::new(
                    DVec3::new(0.005, 0.0, 0.0),
                    DQuat::IDENTITY,
                    DVec3::new(0.01, 0.0, 0.0),
                )
                .unwrap(),
            )
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
}
