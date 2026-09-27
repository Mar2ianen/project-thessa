//! Chamber/nozzle structural materials and cooling topologies: physical
//! properties that size walls and gate pressure/duration — never tier labels.

use serde::{Deserialize, Serialize};

use super::{PropulsionError, require_positive};

/// Chamber/nozzle structural material: density and strength size the walls,
/// temperature gates the cooling mode. Tier labels are forbidden; only
/// physical properties travel here.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ChamberMaterial {
    pub density_kg_m3: f64,
    /// Yield strength for thin-wall sizing (Pa).
    pub yield_strength_pa: f64,
    /// Maximum service wall temperature (K).
    pub max_wall_temp_k: f64,
}

impl ChamberMaterial {
    /// Regeneratively cooled copper-alloy liner class (high conductivity,
    /// fuel-cooled; Mach-diamond-era workhorse assumption).
    pub fn regen_alloy() -> Self {
        Self {
            density_kg_m3: 8900.0,
            yield_strength_pa: 300.0e6,
            max_wall_temp_k: 900.0,
        }
    }

    /// Nickel-superalloy class for high-pressure chambers.
    pub fn nickel_superalloy() -> Self {
        Self {
            density_kg_m3: 8190.0,
            yield_strength_pa: 1000.0e6,
            max_wall_temp_k: 1350.0,
        }
    }

    /// Radiative niobium-alloy class (low strength, high temperature).
    pub fn radiative_niobium() -> Self {
        Self {
            density_kg_m3: 8570.0,
            yield_strength_pa: 300.0e6,
            max_wall_temp_k: 1750.0,
        }
    }

    /// Ablative silica/phenolic class (sacrificial liner, duration-limited).
    pub fn ablative() -> Self {
        Self {
            density_kg_m3: 1800.0,
            yield_strength_pa: 100.0e6,
            max_wall_temp_k: 1800.0,
        }
    }

    pub(crate) fn validate(self) -> Result<(), PropulsionError> {
        require_positive(self.density_kg_m3, "material density")?;
        require_positive(self.yield_strength_pa, "material yield strength")?;
        require_positive(self.max_wall_temp_k, "material max wall temperature")?;
        Ok(())
    }
}

/// Chamber cooling topology: gates attainable pressure/duration physically.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum CoolingMode {
    /// Fuel-cooled jacket: full cycle pressure allowed.
    Regenerative,
    /// Wall radiates: heat-flux bound caps chamber pressure at 8 MPa.
    Radiative,
    /// Sacrificial liner: single burns capped at 300 s (throat erosion
    /// beyond that is not modeled, so the compiler refuses instead).
    Ablative,
}

impl CoolingMode {
    /// Maximum cumulative single-burn duration in seconds (`None` =
    /// unlimited by cooling).
    pub fn max_single_burn_s(self) -> Option<f64> {
        match self {
            Self::Regenerative | Self::Radiative => None,
            Self::Ablative => Some(300.0),
        }
    }

    /// Radiative heat-flux pressure cap (Pa, `None` = no cooling cap).
    pub fn pressure_cap_pa(self) -> Option<f64> {
        match self {
            Self::Regenerative | Self::Ablative => None,
            Self::Radiative => Some(8.0e6),
        }
    }

    /// Relative nozzle wall thickness factor (documented: cooled walls run
    /// thin, radiative needs section, ablative carries a liner).
    pub(crate) fn nozzle_wall_factor(self) -> f64 {
        match self {
            Self::Regenerative => 0.5,
            Self::Radiative => 1.2,
            Self::Ablative => 2.0,
        }
    }
}
