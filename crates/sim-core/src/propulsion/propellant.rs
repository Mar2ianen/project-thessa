//! Chemical propellant pairs: reference chamber thermo plus mixture-ratio
//! tables. Performance is derived from these properties, never looked up
//! by fuel name.

use serde::{Deserialize, Serialize};

use super::PropulsionError;

/// Chemical propellant pairs. Each carries reference chamber thermo at its
/// documented design mixture ratio; off-reference ratios interpolate the
/// mixture tables below.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum Propellant {
    /// LOX/RP-1, reference oxidizer-to-fuel ratio ~2.7.
    LoxRp1,
    /// LOX/methane, reference ratio ~3.5.
    LoxMethane,
    /// LOX/hydrogen, reference ratio ~6.0.
    LoxHydrogen,
    /// NTO/MMH storable hypergolic, reference ratio ~1.65.
    NtoMmh,
    /// Ammonium-perchlorate composite solid propellant.
    SolidApcp,
}

/// Reference chamber thermo for a propellant pair (Sutton-typical values;
/// the `characteristic_velocity` calibration test pins each against
/// published c* within 4%).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PropellantThermo {
    /// Ratio of specific heats of the chamber products.
    pub gamma: f64,
    /// Chamber (adiabatic flame) temperature at the reference ratio (K).
    pub chamber_temp_k: f64,
    /// Specific gas constant of the products (J/kg/K).
    pub gas_constant_j_kg_k: f64,
    /// Bulk/storable density for feed-power bookkeeping (kg/m^3).
    pub bulk_density_kg_m3: f64,
    /// Characteristic chamber length Vc/At (m).
    pub characteristic_length_m: f64,
    /// Published characteristic velocity used as the calibration anchor.
    pub reference_c_star_mps: f64,
}

impl Propellant {
    /// Reference thermo for the pair.
    pub fn thermo(self) -> PropellantThermo {
        match self {
            Self::LoxRp1 => PropellantThermo {
                gamma: 1.24,
                chamber_temp_k: 3670.0,
                gas_constant_j_kg_k: 378.0,
                bulk_density_kg_m3: 1030.0,
                characteristic_length_m: 1.1,
                reference_c_star_mps: 1770.0,
            },
            Self::LoxMethane => PropellantThermo {
                gamma: 1.22,
                chamber_temp_k: 3680.0,
                gas_constant_j_kg_k: 405.0,
                bulk_density_kg_m3: 830.0,
                characteristic_length_m: 1.0,
                reference_c_star_mps: 1830.0,
            },
            Self::LoxHydrogen => PropellantThermo {
                gamma: 1.22,
                chamber_temp_k: 3560.0,
                gas_constant_j_kg_k: 616.0,
                bulk_density_kg_m3: 360.0,
                characteristic_length_m: 0.8,
                reference_c_star_mps: 2300.0,
            },
            Self::NtoMmh => PropellantThermo {
                gamma: 1.25,
                chamber_temp_k: 3400.0,
                gas_constant_j_kg_k: 380.0,
                bulk_density_kg_m3: 1190.0,
                characteristic_length_m: 0.9,
                reference_c_star_mps: 1700.0,
            },
            Self::SolidApcp => PropellantThermo {
                gamma: 1.20,
                chamber_temp_k: 3400.0,
                gas_constant_j_kg_k: 300.0,
                bulk_density_kg_m3: 1770.0,
                characteristic_length_m: 0.0,
                reference_c_star_mps: 1520.0,
            },
        }
    }

    /// True for the solid grain path (grain geometry instead of feed).
    pub fn is_solid(self) -> bool {
        matches!(self, Self::SolidApcp)
    }

    /// Reference (design) oxidizer-to-fuel ratio for the pair.
    pub fn reference_mixture_ratio(self) -> Option<f64> {
        match self {
            Self::LoxRp1 => Some(2.7),
            Self::LoxMethane => Some(3.5),
            Self::LoxHydrogen => Some(6.0),
            Self::NtoMmh => Some(1.65),
            Self::SolidApcp => None,
        }
    }

    /// Mixture table: (oxidizer-to-fuel ratio, chamber temp K, gamma, gas
    /// constant J/kg/K). Representative CEA-trend values bracketing the
    /// reference point; refine with project CEA runs. The middle row always
    /// reproduces [`Propellant::thermo`] exactly (pinned by test).
    fn mixture_table(self) -> Option<&'static [(f64, f64, f64, f64)]> {
        match self {
            Self::LoxRp1 => Some(&[
                (2.0, 3450.0, 1.25, 360.0),
                (2.7, 3670.0, 1.24, 378.0),
                (3.4, 3520.0, 1.23, 390.0),
            ]),
            Self::LoxMethane => Some(&[
                (2.8, 3500.0, 1.23, 430.0),
                (3.5, 3680.0, 1.22, 405.0),
                (4.2, 3600.0, 1.21, 385.0),
            ]),
            Self::LoxHydrogen => Some(&[
                (4.5, 3300.0, 1.24, 700.0),
                (6.0, 3560.0, 1.22, 616.0),
                (7.5, 3650.0, 1.20, 540.0),
            ]),
            Self::NtoMmh => Some(&[
                (1.30, 3200.0, 1.26, 400.0),
                (1.65, 3400.0, 1.25, 380.0),
                (2.00, 3450.0, 1.24, 365.0),
            ]),
            Self::SolidApcp => None,
        }
    }

    /// Chamber thermo at an oxidizer-to-fuel ratio: piecewise-linear
    /// interpolation inside the mixture table, hard refusal outside it.
    /// `None` selects the reference ratio.
    pub fn thermo_at_mixture(
        self,
        mixture_ratio: Option<f64>,
    ) -> Result<PropellantThermo, PropulsionError> {
        let reference = self.thermo();
        let Some(table) = self.mixture_table() else {
            if mixture_ratio.is_some() {
                return Err(PropulsionError::InvalidSpec(
                    "solid grain chemistry is fixed; no mixture knob".into(),
                ));
            }
            return Ok(reference);
        };
        let Some(ratio) = mixture_ratio else {
            return Ok(reference);
        };
        if !ratio.is_finite() {
            return Err(PropulsionError::InvalidSpec(
                "mixture ratio must be finite".into(),
            ));
        }
        if ratio < table.first().expect("non-empty table").0
            || ratio > table.last().expect("non-empty table").0
        {
            return Err(PropulsionError::InvalidSpec(format!(
                "mixture ratio {ratio} outside the modeled range [{}, {}]",
                table.first().expect("non-empty table").0,
                table.last().expect("non-empty table").0
            )));
        }
        for window in table.windows(2) {
            let (low, high) = (window[0], window[1]);
            if ratio <= high.0 {
                let span = (high.0 - low.0).max(1e-12);
                let fraction = (ratio - low.0) / span;
                return Ok(PropellantThermo {
                    gamma: low.2 + (high.2 - low.2) * fraction,
                    chamber_temp_k: low.1 + (high.1 - low.1) * fraction,
                    gas_constant_j_kg_k: low.3 + (high.3 - low.3) * fraction,
                    ..reference
                });
            }
        }
        Ok(reference)
    }
}

#[cfg(test)]
mod tests {
    use super::super::nozzle::characteristic_velocity;
    use super::*;

    #[test]
    fn c_star_matches_published_values() {
        // Calibration of the (gamma, Tc, R) triples against Sutton-typical
        // published c*: proves the tabulated thermo is self-consistent.
        for propellant in [
            Propellant::LoxRp1,
            Propellant::LoxMethane,
            Propellant::LoxHydrogen,
            Propellant::NtoMmh,
            Propellant::SolidApcp,
        ] {
            let thermo = propellant.thermo();
            let c_star = characteristic_velocity(&thermo);
            let error = (c_star - thermo.reference_c_star_mps).abs() / thermo.reference_c_star_mps;
            assert!(
                error < 0.04,
                "{propellant:?}: c* {c_star:.0} vs published {:.0}",
                thermo.reference_c_star_mps
            );
        }
    }

    #[test]
    fn mixture_reference_thermo_is_exact() {
        // The middle table row is the reference point exactly.
        for propellant in [
            Propellant::LoxRp1,
            Propellant::LoxMethane,
            Propellant::LoxHydrogen,
            Propellant::NtoMmh,
        ] {
            let reference = propellant.reference_mixture_ratio().expect("ref ratio");
            let at_ref = propellant
                .thermo_at_mixture(Some(reference))
                .expect("reference ratio compiles");
            assert_eq!(at_ref, propellant.thermo());
        }
        // Outside the table and on solids: refusal, not extrapolation.
        assert!(
            Propellant::LoxHydrogen
                .thermo_at_mixture(Some(9.0))
                .is_err()
        );
        assert!(Propellant::SolidApcp.thermo_at_mixture(Some(1.0)).is_err());
    }
}
