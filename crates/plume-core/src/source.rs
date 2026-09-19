//! Plume source data: compiled nozzle/engine description, dynamic state,
//! and a local environment sample (`docs/38` section 5).
//!
//! Construction of these values is engine-sim / compiled-vehicle-data work.
//! This crate only defines the contract and pure functions over it. The
//! maneuver planner keeps its compact thrust/exhaust-velocity model; nozzle
//! geometry, exhaust optical material, and render-facing state travel here.

use std::fmt;

/// Rigid nozzle-to-vehicle transform: metre translation + unit quaternion
/// (x, y, z, w). Plain data, no math-crate dependency.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RigidTransform {
    /// Nozzle exit centre in vehicle-local metres.
    pub translation_m: [f64; 3],
    /// Unit quaternion (x, y, z, w), nozzle-local +Z down the exhaust axis.
    pub rotation_xyzw: [f64; 4],
}

impl RigidTransform {
    /// Identity transform (exhaust along vehicle-local +Z).
    pub const IDENTITY: Self = Self {
        translation_m: [0.0; 3],
        rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
    };

    /// Rotation component normalized; translation untouched.
    pub fn normalized_rotation(self) -> Self {
        let [x, y, z, w] = self.rotation_xyzw;
        let n = (x * x + y * y + z * z + w * w).sqrt().max(1e-12);
        Self {
            translation_m: self.translation_m,
            rotation_xyzw: [x / n, y / n, z / n, w / n],
        }
    }
}

/// Which exhaust family this engine belongs to. Selects an
/// [`crate::optics::OpticalMaterial`] (render hues), never physics.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum ExhaustFamily {
    Hydrolox,
    Kerolox,
    #[default]
    Methalox,
    Hypergolic,
    Solid,
    NuclearThermal,
    ColdGas,
    Ion,
}

/// Semantic plume input for one nozzle (doc section 5).
///
/// All fields are SI. `throttle` is 0..=1; `throttle <= 0` means the engine
/// contributes zero visible/lighting output (pinned by test).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlumeSource {
    pub nozzle_to_vehicle: RigidTransform,
    /// Nozzle exit radius (m, > 0).
    pub exit_radius_m: f64,
    /// Mass flow at full throttle (kg/s, >= 0).
    pub mass_flow_kg_s: f64,
    /// Exhaust velocity at full throttle (m/s, >= 0).
    pub exhaust_velocity_mps: f64,
    /// Static pressure at the nozzle exit plane (Pa, >= 0).
    pub exit_pressure_pa: f64,
    /// Static temperature at the nozzle exit plane (K, >= 0).
    pub exit_temperature_k: f64,
    /// Exit Mach number (> 1 for a choked supersonic nozzle).
    pub exit_mach: f64,
    /// 0..=1, 0 is off.
    pub throttle: f64,
    pub exhaust: ExhaustFamily,
}

/// Local environment sample at the nozzle (doc section 5).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlumeEnvironment {
    /// Ambient static pressure (Pa, >= 0; vacuum is exactly 0).
    pub pressure_pa: f64,
    /// Ambient density (kg/m^3, >= 0).
    pub density_kg_m3: f64,
    /// Ambient temperature (K, >= 0).
    pub temperature_k: f64,
    /// Oxygen volume fraction 0..=1 (secondary-combustion eligibility).
    pub oxygen_fraction: f64,
    /// Relative airflow in nozzle-local axes (m/s).
    pub flow_velocity_local_mps: [f64; 3],
}

/// First-order control parameter: nozzle-to-ambient pressure ratio
/// `Pi = p_exit / p_ambient`. Vacuum (`p_ambient <= 0`) is positive infinity:
/// every sea-level nozzle is extremely underexpanded in vacuum.
pub fn pressure_ratio(source: &PlumeSource, env: &PlumeEnvironment) -> f64 {
    if env.pressure_pa <= 0.0 {
        return f64::INFINITY;
    }
    if source.exit_pressure_pa <= 0.0 {
        return 0.0;
    }
    source.exit_pressure_pa / env.pressure_pa
}

/// Validation for profile building. Returns a static reason when the source
/// cannot produce a meaningful field (uninited data, not a physics regime).
// Validity guards must reject NaN: `!(x > 0.0)` catches NaN while the lint's
// suggested `x <= 0.0` would accept it. The negated form is deliberate.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn validation_error(source: &PlumeSource, env: &PlumeEnvironment) -> Option<&'static str> {
    if !(source.exit_radius_m > 0.0) {
        return Some("exit_radius_m must be > 0");
    }
    if !(source.exit_mach > 1.0) {
        return Some("exit_mach must be > 1 for a supersonic nozzle");
    }
    if source.throttle < 0.0 || source.throttle > 1.0 || !source.throttle.is_finite() {
        return Some("throttle must be finite in 0..=1");
    }
    if source.mass_flow_kg_s < 0.0
        || source.exhaust_velocity_mps < 0.0
        || source.exit_pressure_pa < 0.0
        || source.exit_temperature_k < 0.0
    {
        return Some("negative engine state");
    }
    if env.pressure_pa < 0.0 || env.density_kg_m3 < 0.0 || env.temperature_k < 0.0 {
        return Some("negative environment state");
    }
    if !(0.0..=1.0).contains(&env.oxygen_fraction) {
        return Some("oxygen_fraction must be in 0..=1");
    }
    None
}

impl fmt::Display for ExhaustFamily {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let name = match self {
            Self::Hydrolox => "hydrolox",
            Self::Kerolox => "kerolox",
            Self::Methalox => "methalox",
            Self::Hypergolic => "hypergolic",
            Self::Solid => "solid",
            Self::NuclearThermal => "nuclear-thermal",
            Self::ColdGas => "cold-gas",
            Self::Ion => "ion",
        };
        write!(f, "{name}")
    }
}

#[cfg(test)]
pub mod tests {
    use super::*;

    pub fn sample_source() -> PlumeSource {
        PlumeSource {
            nozzle_to_vehicle: RigidTransform::IDENTITY,
            exit_radius_m: 0.65,
            mass_flow_kg_s: 520.0,
            exhaust_velocity_mps: 3560.0,
            exit_pressure_pa: 68_000.0,
            exit_temperature_k: 1900.0,
            exit_mach: 3.4,
            throttle: 1.0,
            exhaust: ExhaustFamily::Methalox,
        }
    }

    pub fn sample_env_sea_level() -> PlumeEnvironment {
        PlumeEnvironment {
            pressure_pa: 101_325.0,
            density_kg_m3: 1.225,
            temperature_k: 288.15,
            oxygen_fraction: 0.21,
            flow_velocity_local_mps: [0.0; 3],
        }
    }

    #[test]
    fn vacuum_is_infinite_pressure_ratio() {
        let source = sample_source();
        let vacuum = PlumeEnvironment {
            pressure_pa: 0.0,
            ..sample_env_sea_level()
        };
        assert!(pressure_ratio(&source, &vacuum).is_infinite());
    }

    #[test]
    fn sample_inputs_validate_clean() {
        assert_eq!(
            validation_error(&sample_source(), &sample_env_sea_level()),
            None
        );
    }

    #[test]
    fn bad_inputs_name_a_reason() {
        let mut source = sample_source();
        source.exit_mach = 0.8;
        assert!(validation_error(&source, &sample_env_sea_level()).is_some());
        let mut source = sample_source();
        source.throttle = 2.0;
        assert!(validation_error(&source, &sample_env_sea_level()).is_some());
    }
}
