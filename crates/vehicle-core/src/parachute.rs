//! Backend-neutral deployable parachutes for atmospheric vehicles.
//!
//! The canopy produces ordinary local aerodynamic drag at its mounting point.
//! Deployment is a simulation-time state machine with a pressure trigger,
//! dynamic-pressure opening envelope, reefed inflation, and a structural load
//! limit. Rendering and packed-canopy geometry are consumers of this state.

use std::{error::Error, fmt};

use glam::{DMat3, DVec3};
use serde::{Deserialize, Serialize};

use crate::{AtmosphereSample, RigidBodyProperties};

pub const MAX_PARACHUTES: usize = 64;

/// One authored parachute pack and its deployed canopy.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ParachuteSpec {
    pub name: String,
    /// Projected canopy area when fully inflated, in m².
    pub reference_area_m2: f64,
    /// Dimensionless canopy drag coefficient for the authored reference area.
    pub drag_coefficient: f64,
    /// Fraction of full drag area available during the reefed stage, in (0, 1].
    pub reefed_area_fraction: f64,
    /// Simulation time from reefed opening to full inflation, in seconds.
    pub inflation_time_s: f64,
    /// Automatic opening trigger on descent into this static pressure (Pa).
    pub deploy_pressure_pa: f64,
    /// Opening is held while local dynamic pressure exceeds this safe limit.
    pub max_deploy_dynamic_pressure_pa: f64,
    /// Maximum aerodynamic canopy load before the parachute tears away (N).
    pub max_canopy_load_n: f64,
    /// Installed packed assembly mass (kg).
    pub pack_mass_kg: f64,
    /// Mount center in the authored vehicle frame (m).
    pub position_body_m: DVec3,
    /// Pack inertia tensor about its own center, in body axes (kg·m²).
    pub inertia_body_kg_m2: DMat3,
}

impl ParachuteSpec {
    pub fn validate(&self) -> Result<(), ParachuteError> {
        if self.name.trim().is_empty() {
            return Err(ParachuteError::InvalidConfiguration(
                "name must not be empty".into(),
            ));
        }
        for (value, name) in [
            (self.reference_area_m2, "reference area"),
            (self.drag_coefficient, "drag coefficient"),
            (self.inflation_time_s, "inflation time"),
            (self.deploy_pressure_pa, "deployment pressure"),
            (
                self.max_deploy_dynamic_pressure_pa,
                "maximum deployment dynamic pressure",
            ),
            (self.max_canopy_load_n, "maximum canopy load"),
            (self.pack_mass_kg, "pack mass"),
        ] {
            if !value.is_finite() || value <= 0.0 {
                return Err(ParachuteError::InvalidConfiguration(format!(
                    "{name} must be finite and positive"
                )));
            }
        }
        if !self.reefed_area_fraction.is_finite()
            || !(0.0..=1.0).contains(&self.reefed_area_fraction)
            || self.reefed_area_fraction == 0.0
        {
            return Err(ParachuteError::InvalidConfiguration(
                "reefed area fraction must be finite and in (0, 1]".into(),
            ));
        }
        if !self.position_body_m.is_finite() {
            return Err(ParachuteError::InvalidConfiguration(
                "mount position must be finite".into(),
            ));
        }
        RigidBodyProperties::new(self.pack_mass_kg, self.inertia_body_kg_m2)
            .map_err(|error| ParachuteError::InvalidConfiguration(error.to_string()))?;
        Ok(())
    }

    /// Advance the opening state and evaluate the resulting body-frame load.
    /// `radial_velocity_mps` is vehicle radial velocity relative to the body;
    /// extraction is descent-only. `local_air_velocity_body_mps` is vehicle
    /// velocity relative to local air at the center of mass; the mount's
    /// `ω × r` contribution is added here.
    pub fn advance(
        &self,
        mut state: ParachuteState,
        environment: ParachuteEnvironment,
    ) -> Result<ParachuteLoad, ParachuteError> {
        self.validate()?;
        let ParachuteEnvironment {
            atmosphere,
            radial_velocity_mps,
            center_of_mass_air_velocity_body_mps,
            angular_velocity_body_rps,
            dt_s,
        } = environment;
        if !atmosphere.pressure_pa.is_finite()
            || atmosphere.pressure_pa < 0.0
            || !atmosphere.density_kg_m3.is_finite()
            || atmosphere.density_kg_m3 < 0.0
            || !radial_velocity_mps.is_finite()
            || !center_of_mass_air_velocity_body_mps.is_finite()
            || !angular_velocity_body_rps.is_finite()
            || !dt_s.is_finite()
            || dt_s <= 0.0
            || !state.inflation_elapsed_s.is_finite()
            || state.inflation_elapsed_s < 0.0
        {
            return Err(ParachuteError::InvalidState(
                "environment and state must be finite; pressure and density must be non-negative and timestep positive".into(),
            ));
        }

        let mount_air_velocity_body_mps = center_of_mass_air_velocity_body_mps
            + angular_velocity_body_rps.cross(self.position_body_m);
        let speed_mps = mount_air_velocity_body_mps.length();
        if !speed_mps.is_finite() {
            return Err(ParachuteError::NonFiniteLoad);
        }
        let dynamic_pressure_pa = 0.5 * atmosphere.density_kg_m3 * speed_mps * speed_mps;
        if !dynamic_pressure_pa.is_finite() {
            return Err(ParachuteError::NonFiniteLoad);
        }

        if state.phase == ParachutePhase::Armed
            && radial_velocity_mps < 0.0
            && atmosphere.pressure_pa >= self.deploy_pressure_pa
            && dynamic_pressure_pa <= self.max_deploy_dynamic_pressure_pa
        {
            state.phase = ParachutePhase::Reefed;
            state.inflation_elapsed_s = 0.0;
        } else if state.phase == ParachutePhase::Reefed {
            state.inflation_elapsed_s =
                (state.inflation_elapsed_s + dt_s).min(self.inflation_time_s);
            if state.inflation_elapsed_s >= self.inflation_time_s {
                state.phase = ParachutePhase::Deployed;
            }
        }

        let deployment_fraction = match state.phase {
            ParachutePhase::Reefed => {
                self.reefed_area_fraction
                    + (1.0 - self.reefed_area_fraction)
                        * (state.inflation_elapsed_s / self.inflation_time_s).clamp(0.0, 1.0)
            }
            ParachutePhase::Deployed => 1.0,
            _ => 0.0,
        };
        let mut force_body_n = if speed_mps > 0.0 && deployment_fraction > 0.0 {
            -mount_air_velocity_body_mps / speed_mps
                * (dynamic_pressure_pa
                    * self.drag_coefficient
                    * self.reference_area_m2
                    * deployment_fraction)
        } else {
            DVec3::ZERO
        };
        let mut moment_body_nm = self.position_body_m.cross(force_body_n);
        if !force_body_n.is_finite() || !moment_body_nm.is_finite() {
            return Err(ParachuteError::NonFiniteLoad);
        }

        if deployment_fraction > 0.0 && force_body_n.length() > self.max_canopy_load_n {
            state.phase = ParachutePhase::Failed;
            state.inflation_elapsed_s = 0.0;
            force_body_n = DVec3::ZERO;
            moment_body_nm = DVec3::ZERO;
        }

        Ok(ParachuteLoad {
            state,
            deployment_fraction: if state.phase == ParachutePhase::Failed {
                0.0
            } else {
                deployment_fraction
            },
            dynamic_pressure_pa,
            force_body_n,
            moment_body_nm,
        })
    }

    /// Apply a discrete command to this installed canopy. A stage or future
    /// action-group dispatcher can address a pack by its authored name and
    /// route the command here without owning parachute state transitions.
    pub fn apply_command(&self, state: &mut ParachuteState, command: ParachuteCommand) {
        match (state.phase, command) {
            (ParachutePhase::Stowed, ParachuteCommand::Arm) => {
                state.phase = ParachutePhase::Armed;
                state.inflation_elapsed_s = 0.0;
            }
            (ParachutePhase::Armed, ParachuteCommand::Disarm) => {
                state.phase = ParachutePhase::Stowed;
                state.inflation_elapsed_s = 0.0;
            }
            (
                ParachutePhase::Armed | ParachutePhase::Reefed | ParachutePhase::Deployed,
                ParachuteCommand::Cut,
            )
            | (ParachutePhase::Reefed | ParachutePhase::Deployed, ParachuteCommand::Disarm) => {
                state.phase = ParachutePhase::Cut;
                state.inflation_elapsed_s = 0.0;
            }
            _ => {}
        }
    }
}

/// Discrete per-canopy controls suitable for direct pilot input today and
/// stage/action-group routing later.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ParachuteCommand {
    /// Start waiting for the canopy's automatic descent/pressure/q trigger.
    Arm,
    /// Cancel before extraction; an extracting or deployed canopy is cut.
    Disarm,
    /// Irreversibly sever an armed or deployed canopy.
    Cut,
}

/// One fixed-step parachute environment sample, already expressed in vehicle
/// body axes where applicable.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ParachuteEnvironment {
    pub atmosphere: AtmosphereSample,
    /// Craft velocity projected onto local radial up; negative means descent.
    pub radial_velocity_mps: f64,
    pub center_of_mass_air_velocity_body_mps: DVec3,
    pub angular_velocity_body_rps: DVec3,
    pub dt_s: f64,
}

/// KSP-style packed-to-open state. The player arms a pack; its authored
/// pressure trigger and dynamic-pressure envelope determine when extraction
/// starts. Open canopies can be cut, and overload failure is terminal.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ParachutePhase {
    #[default]
    Stowed,
    Armed,
    Reefed,
    Deployed,
    Cut,
    Failed,
}

/// Authoritative opening state for one installed parachute.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ParachuteState {
    pub phase: ParachutePhase,
    pub inflation_elapsed_s: f64,
}

impl Default for ParachuteState {
    fn default() -> Self {
        Self {
            phase: ParachutePhase::Stowed,
            inflation_elapsed_s: 0.0,
        }
    }
}

/// Per-canopy physical load and state for the latest physics step.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct ParachuteLoad {
    pub state: ParachuteState,
    pub deployment_fraction: f64,
    pub dynamic_pressure_pa: f64,
    pub force_body_n: DVec3,
    pub moment_body_nm: DVec3,
}

#[derive(Debug, Clone, PartialEq)]
pub enum ParachuteError {
    InvalidConfiguration(String),
    InvalidState(String),
    NonFiniteLoad,
}

impl fmt::Display for ParachuteError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidConfiguration(message) => {
                write!(formatter, "invalid parachute configuration: {message}")
            }
            Self::InvalidState(message) => write!(formatter, "invalid parachute state: {message}"),
            Self::NonFiniteLoad => formatter.write_str("parachute load is non-finite"),
        }
    }
}

impl Error for ParachuteError {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::AtmosphereComposition;

    fn spec() -> ParachuteSpec {
        ParachuteSpec {
            name: "main-canopy".into(),
            reference_area_m2: 20.0,
            drag_coefficient: 1.5,
            reefed_area_fraction: 0.1,
            inflation_time_s: 2.0,
            deploy_pressure_pa: 10_000.0,
            max_deploy_dynamic_pressure_pa: 2_000.0,
            max_canopy_load_n: 100_000.0,
            pack_mass_kg: 12.0,
            position_body_m: DVec3::new(-2.0, 0.0, 0.0),
            inertia_body_kg_m2: DMat3::IDENTITY,
        }
    }

    fn atmosphere(pressure_pa: f64, density_kg_m3: f64) -> AtmosphereSample {
        AtmosphereSample {
            altitude_m: 0.0,
            temperature_k: 280.0,
            pressure_pa,
            density_kg_m3,
            speed_of_sound_mps: 330.0,
            dynamic_viscosity_pa_s: 1.8e-5,
            composition: AtmosphereComposition::parse("N2").unwrap(),
        }
    }

    fn environment(
        pressure_pa: f64,
        density_kg_m3: f64,
        radial_velocity_mps: f64,
        center_of_mass_air_velocity_body_mps: DVec3,
        dt_s: f64,
    ) -> ParachuteEnvironment {
        ParachuteEnvironment {
            atmosphere: atmosphere(pressure_pa, density_kg_m3),
            radial_velocity_mps,
            center_of_mass_air_velocity_body_mps,
            angular_velocity_body_rps: DVec3::ZERO,
            dt_s,
        }
    }

    fn armed_state(spec: &ParachuteSpec) -> ParachuteState {
        let mut state = ParachuteState::default();
        spec.apply_command(&mut state, ParachuteCommand::Arm);
        state
    }

    #[test]
    fn deployment_waits_for_pressure_trigger_and_safe_dynamic_pressure() {
        let spec = spec();
        let armed = spec
            .advance(
                armed_state(&spec),
                environment(20_000.0, 1.0, 5.0, DVec3::ZERO, 0.02),
            )
            .unwrap();
        assert_eq!(armed.state.phase, ParachutePhase::Armed);
        assert_eq!(armed.force_body_n, DVec3::ZERO);

        let still_armed = spec
            .advance(
                armed.state,
                environment(20_000.0, 1.0, -5.0, DVec3::X * 100.0, 0.02),
            )
            .unwrap();
        assert_eq!(still_armed.state.phase, ParachutePhase::Armed);

        let reefed = spec
            .advance(
                still_armed.state,
                environment(20_000.0, 1.0, -5.0, DVec3::X * 50.0, 0.02),
            )
            .unwrap();
        assert_eq!(reefed.state.phase, ParachutePhase::Reefed);
        assert_eq!(reefed.deployment_fraction, 0.1);
        assert!((reefed.dynamic_pressure_pa - 1_250.0).abs() < 1.0e-12);
        assert!((reefed.force_body_n.length() - 3_750.0).abs() < 1.0e-9);
        assert!(reefed.force_body_n.x < 0.0);
    }

    #[test]
    fn reefed_canopy_inflates_over_sim_time_and_produces_mount_moment() {
        let spec = spec();
        let reefed = ParachuteState {
            phase: ParachutePhase::Reefed,
            inflation_elapsed_s: 0.0,
        };
        let middle = spec
            .advance(
                reefed,
                environment(20_000.0, 0.2, -10.0, DVec3::X * 100.0, 1.0),
            )
            .unwrap();
        assert_eq!(middle.state.phase, ParachutePhase::Reefed);
        assert_eq!(middle.deployment_fraction, 0.55);
        assert!((middle.force_body_n.length() - 16_500.0).abs() < 1.0e-9);
        assert!(middle.force_body_n.x < 0.0);
        assert_eq!(middle.moment_body_nm, DVec3::ZERO);

        let deployed = spec
            .advance(
                middle.state,
                environment(20_000.0, 0.2, -10.0, DVec3::X * 100.0, 1.0),
            )
            .unwrap();
        assert_eq!(deployed.state.phase, ParachutePhase::Deployed);
        assert_eq!(deployed.deployment_fraction, 1.0);

        let offset_spec = ParachuteSpec {
            position_body_m: DVec3::Y * 2.0,
            ..spec
        };
        let moment = offset_spec
            .advance(
                deployed.state,
                environment(20_000.0, 0.2, -10.0, DVec3::X * 100.0, 0.02),
            )
            .unwrap();
        assert!(moment.moment_body_nm.z > 0.0);
    }

    #[test]
    fn canopy_mount_flow_includes_omega_cross_r() {
        let spec = spec();
        let deployed = ParachuteState {
            phase: ParachutePhase::Deployed,
            inflation_elapsed_s: 0.0,
        };
        let load = spec
            .advance(
                deployed,
                ParachuteEnvironment {
                    atmosphere: atmosphere(20_000.0, 1.0),
                    radial_velocity_mps: -1.0,
                    center_of_mass_air_velocity_body_mps: DVec3::ZERO,
                    angular_velocity_body_rps: DVec3::Y,
                    dt_s: 0.02,
                },
            )
            .unwrap();
        // ω × r = +2 Z m/s at the aft mount, so local drag points toward -Z.
        assert!((load.dynamic_pressure_pa - 2.0).abs() < 1.0e-12);
        assert!(load.force_body_n.z < 0.0);
    }

    #[test]
    fn canopy_can_be_disarmed_cut_or_torn_by_excess_load() {
        let spec = spec();
        let armed = spec
            .advance(
                armed_state(&spec),
                environment(0.0, 0.0, -1.0, DVec3::ZERO, 0.02),
            )
            .unwrap();
        assert_eq!(armed.state.phase, ParachutePhase::Armed);
        let mut disarming = armed.state;
        spec.apply_command(&mut disarming, ParachuteCommand::Disarm);
        let stowed = spec
            .advance(disarming, environment(0.0, 0.0, -1.0, DVec3::ZERO, 0.02))
            .unwrap();
        assert_eq!(stowed.state.phase, ParachutePhase::Stowed);

        let deployed = ParachuteState {
            phase: ParachutePhase::Deployed,
            inflation_elapsed_s: 0.0,
        };
        let mut cutting = deployed;
        spec.apply_command(&mut cutting, ParachuteCommand::Cut);
        let cut = spec
            .advance(
                cutting,
                environment(20_000.0, 0.2, -1.0, DVec3::X * 20.0, 0.02),
            )
            .unwrap();
        assert_eq!(cut.state.phase, ParachutePhase::Cut);
        assert_eq!(cut.force_body_n, DVec3::ZERO);

        let mut weak = spec;
        weak.max_canopy_load_n = 1.0;
        let failed = weak
            .advance(
                deployed,
                environment(20_000.0, 0.2, -1.0, DVec3::X * 100.0, 0.02),
            )
            .unwrap();
        assert_eq!(failed.state.phase, ParachutePhase::Failed);
        assert_eq!(failed.force_body_n, DVec3::ZERO);
    }
}
