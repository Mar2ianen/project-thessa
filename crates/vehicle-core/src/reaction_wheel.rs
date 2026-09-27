//! Backend-neutral internal reaction-wheel actuators.
//!
//! Wheel banks are game-oriented moment actuators: their authored torque
//! ratings are available continuously, without rotor-momentum saturation.
//! Contact, renderer and power-system backends are intentionally outside this
//! module.

use std::{error::Error, fmt};

use glam::{DMat3, DVec3};
use serde::{Deserialize, Serialize};

use crate::RigidBodyProperties;

/// Authored three-axis reaction-wheel assembly, with wheel axes aligned to the
/// vehicle body axes. Multiple assemblies are allowed and their torque
/// authority is combined per axis.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ReactionWheelBankSpec {
    pub name: String,
    /// Installed torque rating about body X/Y/Z, in N·m.
    pub max_torque_body_nm: DVec3,
    /// Installed assembly mass, included in vehicle COM and inertia baking.
    pub mass_kg: f64,
    /// Assembly center of mass in the vehicle authoring frame.
    pub position_body_m: DVec3,
    /// Inertia tensor about the assembly center of mass, in body axes.
    pub inertia_body_kg_m2: DMat3,
}

impl ReactionWheelBankSpec {
    pub fn validate(&self) -> Result<(), ReactionWheelError> {
        if self.name.trim().is_empty()
            || !self.max_torque_body_nm.is_finite()
            || !self.mass_kg.is_finite()
            || self.mass_kg <= 0.0
            || !self.position_body_m.is_finite()
            || self.max_torque_body_nm.min_element() < 0.0
        {
            return Err(ReactionWheelError::InvalidConfiguration(format!(
                "'{}' has a missing name or non-finite/negative rating, mass, or position",
                self.name
            )));
        }
        RigidBodyProperties::new(self.mass_kg, self.inertia_body_kg_m2).map_err(|error| {
            ReactionWheelError::InvalidConfiguration(format!(
                "'{}' has invalid mass properties: {error}",
                self.name
            ))
        })?;
        Ok(())
    }
}

/// One reaction-wheel allocation result for a physics step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ReactionWheelAllocation {
    pub delivered_torque_body_nm: DVec3,
    pub saturated: bool,
}

/// Allocate requested body moment to aligned reaction-wheel banks. A bank's
/// rating is a per-axis actuator capability, not a consumable momentum
/// reservoir; as in KSP's gameplay model, sustained rotation does not saturate.
pub fn allocate_reaction_wheels(
    specs: &[ReactionWheelBankSpec],
    requested_torque_body_nm: DVec3,
) -> Result<ReactionWheelAllocation, ReactionWheelError> {
    allocate_reaction_wheels_masked(specs, None, requested_torque_body_nm)
}

/// Allocate to only the enabled reaction-wheel banks. The mask is parallel to
/// `specs`; disabled banks remain installed but provide no torque authority.
pub fn allocate_reaction_wheels_with_enabled_banks(
    specs: &[ReactionWheelBankSpec],
    enabled_banks: &[bool],
    requested_torque_body_nm: DVec3,
) -> Result<ReactionWheelAllocation, ReactionWheelError> {
    allocate_reaction_wheels_masked(specs, Some(enabled_banks), requested_torque_body_nm)
}

fn allocate_reaction_wheels_masked(
    specs: &[ReactionWheelBankSpec],
    enabled_banks: Option<&[bool]>,
    requested_torque_body_nm: DVec3,
) -> Result<ReactionWheelAllocation, ReactionWheelError> {
    if !requested_torque_body_nm.is_finite() {
        return Err(ReactionWheelError::InvalidState(
            "requested torque must be finite".into(),
        ));
    }
    if let Some(enabled) = enabled_banks
        && enabled.len() != specs.len()
    {
        return Err(ReactionWheelError::InvalidState(
            "reaction-wheel enable mask must match the installed banks".into(),
        ));
    }
    for spec in specs {
        spec.validate()?;
    }
    let total_rating = specs
        .iter()
        .enumerate()
        .filter(|(index, _)| enabled_banks.is_none_or(|enabled| enabled[*index]))
        .map(|(_, spec)| spec.max_torque_body_nm)
        .sum::<DVec3>();
    let delivered = DVec3::new(
        requested_torque_body_nm
            .x
            .clamp(-total_rating.x, total_rating.x),
        requested_torque_body_nm
            .y
            .clamp(-total_rating.y, total_rating.y),
        requested_torque_body_nm
            .z
            .clamp(-total_rating.z, total_rating.z),
    );
    let residual = requested_torque_body_nm - delivered;
    Ok(ReactionWheelAllocation {
        delivered_torque_body_nm: delivered,
        saturated: residual.length_squared() > 1.0e-12,
    })
}

#[derive(Debug, Clone, PartialEq)]
pub enum ReactionWheelError {
    InvalidConfiguration(String),
    InvalidState(String),
}

impl fmt::Display for ReactionWheelError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidConfiguration(message) => {
                write!(formatter, "invalid reaction-wheel configuration: {message}")
            }
            Self::InvalidState(message) => {
                write!(formatter, "invalid reaction-wheel state: {message}")
            }
        }
    }
}

impl Error for ReactionWheelError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn bank(name: &str, torque: DVec3) -> ReactionWheelBankSpec {
        ReactionWheelBankSpec {
            name: name.into(),
            max_torque_body_nm: torque,
            mass_kg: 10.0,
            position_body_m: DVec3::ZERO,
            inertia_body_kg_m2: DMat3::IDENTITY,
        }
    }

    #[test]
    fn rated_torque_is_available_indefinitely_without_momentum_saturation() {
        let specs = [bank("main", DVec3::splat(20.0))];
        for _ in 0..10_000 {
            let result = allocate_reaction_wheels(&specs, DVec3::X * 15.0).unwrap();
            assert_eq!(result.delivered_torque_body_nm, DVec3::X * 15.0);
            assert!(!result.saturated);
        }
    }

    #[test]
    fn multiple_banks_combine_torque_ratings_per_axis() {
        let specs = [
            bank("small", DVec3::X * 10.0),
            bank("large", DVec3::X * 30.0),
        ];
        let result = allocate_reaction_wheels(&specs, DVec3::X * 20.0).unwrap();
        assert_eq!(result.delivered_torque_body_nm, DVec3::X * 20.0);
        assert!(!result.saturated);
    }

    #[test]
    fn disabled_bank_does_not_contribute_torque_rating() {
        let specs = [
            bank("offline", DVec3::X * 10.0),
            bank("online", DVec3::X * 30.0),
        ];
        let result =
            allocate_reaction_wheels_with_enabled_banks(&specs, &[false, true], DVec3::X * 20.0)
                .unwrap();
        assert_eq!(result.delivered_torque_body_nm, DVec3::X * 20.0);
        assert!(!result.saturated);

        let saturated =
            allocate_reaction_wheels_with_enabled_banks(&specs, &[false, true], DVec3::X * 50.0)
                .unwrap();
        assert_eq!(saturated.delivered_torque_body_nm, DVec3::X * 30.0);
        assert!(saturated.saturated);
    }

    #[test]
    fn requested_moment_saturates_only_at_the_configured_motor_rating() {
        let specs = [bank("main", DVec3::new(8.0, 12.0, 20.0))];
        let result = allocate_reaction_wheels(&specs, DVec3::new(100.0, -100.0, 10.0)).unwrap();
        assert_eq!(
            result.delivered_torque_body_nm,
            DVec3::new(8.0, -12.0, 10.0)
        );
        assert!(result.saturated);
    }
}
