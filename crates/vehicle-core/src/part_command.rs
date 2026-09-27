//! Discrete commands for installed vehicle subsystems.
//!
//! The flight server and future stage/action-group dispatch share this
//! command vocabulary. Group commands configure a subsystem as a whole, while
//! individually addressable components use their authored names. This module
//! does not decide when stages or action groups emit commands.

use serde::{Deserialize, Serialize};

use crate::ParachuteCommand;

/// Idempotent control commands for currently modeled installed actuators.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VehiclePartCommand {
    SetRcsEnabled {
        enabled: bool,
    },
    SetReactionWheelsEnabled {
        enabled: bool,
    },
    SetReactionWheelBankEnabled {
        name: String,
        enabled: bool,
    },
    SetLandingGearDeployed {
        deployed: bool,
    },
    SetWheelChassisDeployed {
        name: String,
        deployed: bool,
    },
    SetLandingLegDeployed {
        name: String,
        deployed: bool,
    },
    SetParachutesArmed {
        armed: bool,
    },
    /// Address one authored parachute by its unique component name.
    Parachute {
        name: String,
        command: ParachuteCommand,
    },
}
