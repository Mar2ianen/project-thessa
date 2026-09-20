//! Propellant tanks and feed lines (`docs/details/04` section 5).
//!
//! Tanks are thin-wall pressure vessels sized from volume, pressure, and
//! material — the same wall math as chambers, not a mass lookup. Feed lines
//! carry Darcy-Weisbach pressure drop plus standard minor-loss coefficients
//! for bends and entrance/exit. No CFD, no hidden margins: every fit is
//! named and the velocity gate refuses erosive flow instead of derating it.

// Validity guards use `!(x > 0.0)` so NaN fails closed (repo convention).
#![allow(clippy::neg_cmp_op_on_partial_ord)]

use serde::{Deserialize, Serialize};

use crate::propulsion::{ChamberMaterial, PropulsionError};

/// Tank shell shape.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum TankShape {
    Sphere { diameter_m: f64 },
    Cylinder { diameter_m: f64, length_m: f64 },
}

impl TankShape {
    /// Internal volume (m^3).
    pub fn volume_m3(self) -> Result<f64, PropulsionError> {
        match self {
            Self::Sphere { diameter_m } => {
                if !(diameter_m > 0.0) {
                    return Err(PropulsionError::InvalidSpec(
                        "tank diameter must be finite and > 0".into(),
                    ));
                }
                Ok(std::f64::consts::PI / 6.0 * diameter_m.powi(3))
            }
            Self::Cylinder {
                diameter_m,
                length_m,
            } => {
                if !(diameter_m > 0.0) || !(length_m > 0.0) {
                    return Err(PropulsionError::InvalidSpec(
                        "tank dimensions must be finite and > 0".into(),
                    ));
                }
                Ok(std::f64::consts::PI / 4.0 * diameter_m * diameter_m * length_m)
            }
        }
    }

    /// Wetted shell area (m^2, caps included for cylinders).
    fn area_m2(self) -> Result<f64, PropulsionError> {
        match self {
            Self::Sphere { diameter_m } => {
                self.volume_m3()?;
                Ok(std::f64::consts::PI * diameter_m * diameter_m)
            }
            Self::Cylinder {
                diameter_m,
                length_m,
            } => {
                self.volume_m3()?;
                Ok(std::f64::consts::PI * diameter_m * length_m
                    + 2.0 * std::f64::consts::PI / 4.0 * diameter_m * diameter_m)
            }
        }
    }
}

/// Tank authoring: shape, service pressure, shell material.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct TankSpec {
    pub shape: TankShape,
    /// Maximum service pressure (Pa).
    pub pressure_pa: f64,
    pub material: ChamberMaterial,
}

/// Compiled tank: volume, dry mass, pressure rating.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CompiledTank {
    pub volume_m3: f64,
    pub dry_mass_kg: f64,
    pub max_pressure_pa: f64,
    /// Propellant mass at full fill for a bulk density (kg).
    pub full_propellant_kg: f64,
}

impl TankSpec {
    /// Validate authoring values (NaN fails closed).
    pub fn validate(&self) -> Result<(), PropulsionError> {
        self.shape.volume_m3()?;
        if !(self.pressure_pa > 0.0) {
            return Err(PropulsionError::InvalidSpec(
                "tank pressure must be finite and > 0".into(),
            ));
        }
        self.material.validate()?;
        Ok(())
    }

    /// Hangar compile for a propellant bulk density.
    pub fn compile(&self, bulk_density_kg_m3: f64) -> Result<CompiledTank, PropulsionError> {
        self.validate()?;
        if !(bulk_density_kg_m3 > 0.0) {
            return Err(PropulsionError::InvalidSpec(
                "propellant bulk density must be finite and > 0".into(),
            ));
        }
        let volume_m3 = self.shape.volume_m3()?;
        let area_m2 = self.shape.area_m2()?;
        let diameter_m = match self.shape {
            TankShape::Sphere { diameter_m } => diameter_m,
            TankShape::Cylinder { diameter_m, .. } => diameter_m,
        };
        // Thin-wall sizing with the shared safety factor, plus a documented
        // 15% allowance for welds, ports, and mounting fixtures.
        let wall_m = self.pressure_pa * diameter_m / (2.0 * self.material.yield_strength_pa) * 1.5;
        let dry_mass_kg = area_m2 * wall_m * self.material.density_kg_m3 * 1.15;
        Ok(CompiledTank {
            volume_m3,
            dry_mass_kg,
            max_pressure_pa: self.pressure_pa,
            full_propellant_kg: volume_m3 * bulk_density_kg_m3,
        })
    }
}

/// One tank installed on a vehicle: compiled data plus its mount station.
/// Dry mass aggregates as a point mass at bake time; propellant fill is a
/// runtime concern (tanks bake full; depletion wiring is future work).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct TankMount {
    pub tank: CompiledTank,
    /// Mount station in vehicle body metres.
    pub position_body_m: [f64; 3],
}

impl TankMount {
    /// Validate mount data (NaN fails closed).
    pub fn validate(&self) -> Result<(), PropulsionError> {
        if self.position_body_m.iter().any(|v| !v.is_finite()) {
            return Err(PropulsionError::InvalidSpec(
                "tank position must be finite".into(),
            ));
        }
        if !(self.tank.volume_m3 > 0.0) || !(self.tank.dry_mass_kg >= 0.0) {
            return Err(PropulsionError::InvalidSpec(
                "tank data must be positive".into(),
            ));
        }
        Ok(())
    }
}
/// Feed line authoring: diameter, length, bend count, rating.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct FeedLine {
    /// Inner diameter (m).
    pub diameter_m: f64,
    /// Developed length (m).
    pub length_m: f64,
    /// Number of smooth bends (0.3 velocity heads each, documented).
    pub bends: u32,
    /// Rated pressure for wall sizing (Pa).
    pub rated_pressure_pa: f64,
    pub material: ChamberMaterial,
}

/// Velocity gate: flow faster than this is refused (erosion/cavitation
/// heuristic, documented — not a derating curve).
pub const FEED_MAX_VELOCITY_MPS: f64 = 15.0;
/// Entrance + exit minor-loss coefficient (velocity heads, documented).
const MINOR_ENTRANCE_EXIT_K: f64 = 1.5;
/// One smooth bend minor-loss coefficient (documented).
const MINOR_BEND_K: f64 = 0.3;

impl FeedLine {
    /// Validate authoring values (NaN fails closed).
    pub fn validate(&self) -> Result<(), PropulsionError> {
        if !(self.diameter_m > 0.0) {
            return Err(PropulsionError::InvalidSpec(
                "line diameter must be finite and > 0".into(),
            ));
        }
        if !(self.length_m >= 0.0) {
            return Err(PropulsionError::InvalidSpec(
                "line length must be finite and >= 0".into(),
            ));
        }
        if !(self.rated_pressure_pa > 0.0) {
            return Err(PropulsionError::InvalidSpec(
                "line rating must be finite and > 0".into(),
            ));
        }
        self.material.validate()?;
        Ok(())
    }

    /// Mean flow velocity (m/s) for a mass flow and fluid density.
    pub fn flow_velocity_mps(
        &self,
        mass_flow_kg_s: f64,
        density_kg_m3: f64,
    ) -> Result<f64, PropulsionError> {
        self.validate()?;
        if !(mass_flow_kg_s >= 0.0) || !(density_kg_m3 > 0.0) {
            return Err(PropulsionError::InvalidSpec(
                "flow and density must be finite, density > 0".into(),
            ));
        }
        let area_m2 = std::f64::consts::PI / 4.0 * self.diameter_m * self.diameter_m;
        Ok(mass_flow_kg_s / (density_kg_m3 * area_m2))
    }

    /// Darcy-Weisbach pressure drop (Pa): laminar 64/Re below Re 2300,
    /// Blasius smooth-turbulent above (documented; rough-pipe and
    /// two-phase regimes are refused by the velocity gate, not modeled).
    pub fn pressure_drop_pa(
        &self,
        mass_flow_kg_s: f64,
        density_kg_m3: f64,
        viscosity_pa_s: f64,
    ) -> Result<f64, PropulsionError> {
        if !(viscosity_pa_s > 0.0) {
            return Err(PropulsionError::InvalidSpec(
                "viscosity must be finite and > 0".into(),
            ));
        }
        let velocity_mps = self.flow_velocity_mps(mass_flow_kg_s, density_kg_m3)?;
        if velocity_mps == 0.0 {
            return Ok(0.0);
        }
        if velocity_mps > FEED_MAX_VELOCITY_MPS {
            return Err(PropulsionError::UnsupportedCombination(format!(
                "line velocity {velocity_mps:.1} m/s exceeds the {FEED_MAX_VELOCITY_MPS:.0} m/s gate: resize the line"
            )));
        }
        let reynolds = density_kg_m3 * velocity_mps * self.diameter_m / viscosity_pa_s;
        let friction = if reynolds < 2300.0 {
            64.0 / reynolds
        } else {
            0.316 / reynolds.powf(0.25)
        };
        let minor_k = MINOR_ENTRANCE_EXIT_K + MINOR_BEND_K * self.bends as f64;
        Ok((friction * self.length_m / self.diameter_m + minor_k)
            * 0.5
            * density_kg_m3
            * velocity_mps
            * velocity_mps)
    }

    /// Dry line mass: thin-wall tube at rating plus 10% fittings.
    pub fn dry_mass_kg(&self) -> Result<f64, PropulsionError> {
        self.validate()?;
        let wall_m = self.rated_pressure_pa * self.diameter_m
            / (2.0 * self.material.yield_strength_pa)
            * 1.5;
        Ok(std::f64::consts::PI
            * self.diameter_m
            * self.length_m.max(0.01)
            * wall_m
            * self.material.density_kg_m3
            * 1.1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sphere_tank_volume_and_mass_band() {
        // D = 1.24 m sphere holds ~1 m^3; aluminum-class wall at 0.5 MPa
        // must land in the tens-of-kg band (order-of-magnitude pin).
        let spec = TankSpec {
            shape: TankShape::Sphere { diameter_m: 1.24 },
            pressure_pa: 500_000.0,
            material: ChamberMaterial {
                density_kg_m3: 2700.0,
                yield_strength_pa: 300.0e6,
                max_wall_temp_k: 400.0,
            },
        };
        let tank = spec.compile(1000.0).expect("tank compiles");
        assert!((tank.volume_m3 - 1.0).abs() < 0.01);
        assert!((tank.full_propellant_kg - 1000.0).abs() < 10.0);
        assert!(
            tank.dry_mass_kg > 5.0 && tank.dry_mass_kg < 60.0,
            "dry mass {} kg outside the band",
            tank.dry_mass_kg
        );
    }

    #[test]
    fn laminar_drop_matches_hagen_poiseuille() {
        // Special case: Re < 2300 must reproduce Hagen-Poiseuille
        // dP = 32 mu L v / D^2 exactly (same math, independent path).
        let line = FeedLine {
            diameter_m: 0.05,
            length_m: 3.0,
            bends: 0,
            rated_pressure_pa: 3.0e6,
            material: ChamberMaterial::regen_alloy(),
        };
        let density = 1000.0;
        let viscosity = 1.0e-3;
        let flow = 0.05;
        let drop = line
            .pressure_drop_pa(flow, density, viscosity)
            .expect("drop");
        let velocity = flow / (density * std::f64::consts::PI / 4.0 * 0.05 * 0.05);
        let reynolds = density * velocity * 0.05 / viscosity;
        assert!(reynolds < 2300.0, "test must stay laminar");
        // Hagen-Poiseuille without minor losses is not directly comparable
        // (entrance/exit K rides along), so subtract the minor term here.
        let minor = MINOR_ENTRANCE_EXIT_K * 0.5 * density * velocity * velocity;
        let expected = 32.0 * viscosity * 3.0 * velocity / (0.05 * 0.05);
        assert!(
            ((drop - minor) - expected).abs() / expected < 1e-9,
            "laminar friction must equal Hagen-Poiseuille"
        );
    }

    #[test]
    fn drop_grows_with_flow_and_gates_velocity() {
        let line = FeedLine {
            diameter_m: 0.05,
            length_m: 3.0,
            bends: 2,
            rated_pressure_pa: 3.0e6,
            material: ChamberMaterial::regen_alloy(),
        };
        let low = line
            .pressure_drop_pa(0.5, 1000.0, 1.0e-3)
            .expect("low flow");
        let high = line
            .pressure_drop_pa(1.0, 1000.0, 1.0e-3)
            .expect("high flow");
        assert!(high > low * 2.0, "turbulent drop grows faster than linear");
        // ~40 kg/s through a 50 mm line is ~20 m/s: refused, not derated.
        assert!(line.pressure_drop_pa(40.0, 1000.0, 1.0e-3).is_err());
        assert!(line.dry_mass_kg().expect("mass") > 0.0);
    }

    #[test]
    fn nan_inputs_fail_closed() {
        let spec = TankSpec {
            shape: TankShape::Sphere {
                diameter_m: f64::NAN,
            },
            pressure_pa: 500_000.0,
            material: ChamberMaterial::regen_alloy(),
        };
        assert!(spec.compile(1000.0).is_err());
        let line = FeedLine {
            diameter_m: 0.05,
            length_m: 3.0,
            bends: 0,
            rated_pressure_pa: 3.0e6,
            material: ChamberMaterial::regen_alloy(),
        };
        assert!(line.pressure_drop_pa(f64::NAN, 1000.0, 1.0e-3).is_err());
    }
}
