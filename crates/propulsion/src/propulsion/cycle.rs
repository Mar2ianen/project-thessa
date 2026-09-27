//! Liquid feed/power cycle topologies: pressure-rise capability, bypass
//! bookkeeping, and mass/power consequences — never thrust multipliers.

use serde::{Deserialize, Serialize};

/// Liquid feed/power cycle topology. Each variant gates attainable chamber
/// pressure and carries its own flow bookkeeping; none multiplies thrust.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum EngineCycle {
    PressureFed,
    ElectricPump,
    GasGenerator,
    StagedCombustion,
    FullFlowStaged,
}

/// Engineering bounds + flow bookkeeping for a cycle (documented typical
/// demonstrated ranges, generous rather than balance-tuned).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CycleLimits {
    /// Maximum chamber pressure the feed system can sustain (Pa).
    pub max_chamber_pressure_pa: f64,
    /// Gas-generator bypass as a fraction of chamber flow (0 = closed).
    pub gg_bypass_fraction: f64,
    /// Gas-generator duct temperature (K, 0 when no GG duct).
    pub gg_temperature_k: f64,
    /// Deep-throttle combustion-stability floor (throttle fraction).
    pub min_throttle: f64,
    /// Spool/valve first-order time constant (s).
    pub spool_tau_s: f64,
}

impl EngineCycle {
    /// Cycle bounds.
    pub fn limits(self) -> CycleLimits {
        match self {
            Self::PressureFed => CycleLimits {
                max_chamber_pressure_pa: 3.0e6,
                gg_bypass_fraction: 0.0,
                gg_temperature_k: 0.0,
                min_throttle: 0.30,
                spool_tau_s: 0.4,
            },
            Self::ElectricPump => CycleLimits {
                max_chamber_pressure_pa: 12.0e6,
                gg_bypass_fraction: 0.0,
                gg_temperature_k: 0.0,
                min_throttle: 0.35,
                spool_tau_s: 0.3,
            },
            Self::GasGenerator => CycleLimits {
                max_chamber_pressure_pa: 21.0e6,
                gg_bypass_fraction: 0.030,
                gg_temperature_k: 1050.0,
                min_throttle: 0.40,
                spool_tau_s: 0.6,
            },
            Self::StagedCombustion => CycleLimits {
                max_chamber_pressure_pa: 30.0e6,
                gg_bypass_fraction: 0.0,
                gg_temperature_k: 0.0,
                min_throttle: 0.50,
                spool_tau_s: 1.0,
            },
            Self::FullFlowStaged => CycleLimits {
                max_chamber_pressure_pa: 35.0e6,
                gg_bypass_fraction: 0.0,
                gg_temperature_k: 0.0,
                min_throttle: 0.50,
                spool_tau_s: 1.2,
            },
        }
    }
}
