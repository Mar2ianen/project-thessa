//! Pilot input modes and solver regimes (moved verbatim from the client).

use glam::DQuat;
use serde::{Deserialize, Serialize};
use thessa_flight_control::{GuidanceIntent, PilotAxes, RollPolicy};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ControlMode {
    MouseAim,
    Navball,
    Rate,
    Direct,
}

impl ControlMode {
    pub const ALL: [Self; 4] = [Self::MouseAim, Self::Navball, Self::Rate, Self::Direct];

    pub fn next(self) -> Self {
        let index = Self::ALL.iter().position(|mode| *mode == self).unwrap_or(0);
        Self::ALL[(index + 1) % Self::ALL.len()]
    }

    pub fn previous(self) -> Self {
        let index = Self::ALL.iter().position(|mode| *mode == self).unwrap_or(0);
        Self::ALL[(index + Self::ALL.len() - 1) % Self::ALL.len()]
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::MouseAim => "MOUSE STEERING",
            Self::Navball => "ATTITUDE HOLD",
            Self::Rate => "RATE CONTROL",
            Self::Direct => "DIRECT / RAW",
        }
    }

    pub fn description(self) -> &'static str {
        match self {
            Self::MouseAim => "Cursor commands pitch/yaw rate; center to hold.",
            Self::Navball => "direct attitude target control",
            Self::Rate => "command angular rates",
            Self::Direct => "raw actuator input",
        }
    }

    /// Compatibility adapter for the legacy UI/wire enum. New callers should
    /// produce [`GuidanceIntent`] directly; this keeps old clients on the same
    /// authority direction while the protocol migrates away from SAS fields.
    pub fn into_guidance_intent(
        self,
        axes: PilotAxes,
        sas_target_orientation: DQuat,
        sas_enabled: bool,
    ) -> GuidanceIntent {
        match self {
            Self::MouseAim | Self::Navball if sas_enabled => GuidanceIntent::Attitude {
                target_body_to_inertial: sas_target_orientation,
                roll_policy: RollPolicy::Hold,
            },
            Self::Rate => GuidanceIntent::AngularRate {
                rate_body_rps: glam::DVec3::new(axes.pitch, axes.yaw, axes.roll),
            },
            _ => GuidanceIntent::ManualAxes(axes),
        }
    }
}

/// Solver regime of the flown vehicle: dense-air 6-DoF flight vs vacuum coast.
///
/// Coast is a solver optimization, never a physics switch: translation stays
/// full multi-body gravity plus thrust, rotation stays torque-free plus RCS.
/// Below the density threshold aero moments are orders of magnitude under RCS
/// authority, so the trim solver freezes the surfaces instead of chasing a
/// near-singular effectiveness matrix. The threshold is a density, not an
/// altitude, so it follows any atmosphere the config provides.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum FlightRegime {
    #[default]
    Aero,
    Coast,
}

impl FlightRegime {
    pub fn label(self) -> &'static str {
        match self {
            Self::Aero => "AERO",
            Self::Coast => "COAST",
        }
    }
}
