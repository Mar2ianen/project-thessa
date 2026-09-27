//! Propellant tanks and feed lines (`docs/details/04` section 5).
//!
//! Tanks are thin-wall pressure vessels sized from volume, pressure, and
//! material — the same wall math as chambers, not a mass lookup. Feed lines
//! carry Darcy-Weisbach pressure drop plus standard minor-loss coefficients
//! for bends and entrance/exit. No CFD, no hidden margins: every fit is
//! named and the velocity gate refuses erosive flow instead of derating it.

// Validity guards use `!(x > 0.0)` so NaN fails closed (repo convention).
#![allow(clippy::neg_cmp_op_on_partial_ord)]

use glam::{DMat3, DVec3};
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
                if !diameter_m.is_finite() || diameter_m <= 0.0 {
                    return Err(PropulsionError::InvalidSpec(
                        "tank diameter must be finite and > 0".into(),
                    ));
                }
                let volume = std::f64::consts::PI / 6.0 * diameter_m.powi(3);
                if !volume.is_finite() || volume <= 0.0 {
                    return Err(PropulsionError::InvalidSpec(
                        "tank volume must remain finite and > 0".into(),
                    ));
                }
                Ok(volume)
            }
            Self::Cylinder {
                diameter_m,
                length_m,
            } => {
                if !diameter_m.is_finite()
                    || diameter_m <= 0.0
                    || !length_m.is_finite()
                    || length_m <= 0.0
                {
                    return Err(PropulsionError::InvalidSpec(
                        "tank dimensions must be finite and > 0".into(),
                    ));
                }
                let volume = std::f64::consts::PI / 4.0 * diameter_m * diameter_m * length_m;
                if !volume.is_finite() || volume <= 0.0 {
                    return Err(PropulsionError::InvalidSpec(
                        "tank volume must remain finite and > 0".into(),
                    ));
                }
                Ok(volume)
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

    /// Intrinsic inertia of the pressure shell plus its authored initial-fill
    /// propellant about the tank centroid. Cylinder shell mass is divided
    /// between the cylindrical wall and two flat end plates by their area;
    /// this matches the surface-area mass sizing above. Fluid is modeled as
    /// a uniform solid ellipsoid/cylinder until depletion-state fluid motion
    /// supplies a changing fill distribution.
    pub fn intrinsic_inertia_body_kg_m2(
        self,
        dry_mass_kg: f64,
        propellant_mass_kg: f64,
    ) -> Result<DMat3, PropulsionError> {
        if !dry_mass_kg.is_finite()
            || dry_mass_kg < 0.0
            || !propellant_mass_kg.is_finite()
            || propellant_mass_kg < 0.0
        {
            return Err(PropulsionError::InvalidSpec(
                "tank masses must be finite and non-negative".into(),
            ));
        }
        let inertia = match self {
            Self::Sphere { diameter_m } => {
                self.volume_m3()?;
                let radius2 = 0.25 * diameter_m * diameter_m;
                let shell = (2.0 / 3.0) * dry_mass_kg * radius2;
                let fluid = (2.0 / 5.0) * propellant_mass_kg * radius2;
                DMat3::from_diagonal(DVec3::splat(shell + fluid))
            }
            Self::Cylinder {
                diameter_m,
                length_m,
            } => {
                self.volume_m3()?;
                let radius = 0.5 * diameter_m;
                let side_area = 2.0 * std::f64::consts::PI * radius * length_m;
                let cap_area = 2.0 * std::f64::consts::PI * radius * radius;
                let total_area = side_area + cap_area;
                let side_mass = dry_mass_kg * side_area / total_area;
                let cap_mass = dry_mass_kg - side_mass;
                let shell_ix = side_mass * radius * radius + cap_mass * radius * radius / 2.0;
                let shell_transverse = side_mass
                    * (radius * radius / 2.0 + length_m * length_m / 12.0)
                    + cap_mass * (radius * radius / 4.0 + length_m * length_m / 4.0);
                let fluid_ix = 0.5 * propellant_mass_kg * radius * radius;
                let fluid_transverse =
                    propellant_mass_kg * (3.0 * radius * radius + length_m * length_m) / 12.0;
                DMat3::from_diagonal(DVec3::new(
                    shell_ix + fluid_ix,
                    shell_transverse + fluid_transverse,
                    shell_transverse + fluid_transverse,
                ))
            }
        };
        if !inertia.is_finite() {
            return Err(PropulsionError::InvalidSpec(
                "tank inertia must remain finite".into(),
            ));
        }
        Ok(inertia)
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

/// Compiled tank: capacity, dry mass, pressure rating and full-fill capacity mass.
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
        if !self.pressure_pa.is_finite() || self.pressure_pa <= 0.0 {
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
        if !bulk_density_kg_m3.is_finite() || bulk_density_kg_m3 <= 0.0 {
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
        // Thin-wall sizing with shape-correct membrane stress and a shared
        // 1.5 factor of safety, plus a documented 15% allowance for welds,
        // ports, and mounting fixtures. A sphere carries half the hoop
        // stress of a cylinder at the same diameter and pressure.
        let stress_denominator = match self.shape {
            TankShape::Sphere { .. } => 4.0,
            TankShape::Cylinder { .. } => 2.0,
        };
        let wall_m = self.pressure_pa * diameter_m
            / (stress_denominator * self.material.yield_strength_pa)
            * 1.5;
        let dry_mass_kg = area_m2 * wall_m * self.material.density_kg_m3 * 1.15;
        let full_propellant_kg = volume_m3 * bulk_density_kg_m3;
        if !wall_m.is_finite() || !dry_mass_kg.is_finite() || !full_propellant_kg.is_finite() {
            return Err(PropulsionError::InvalidSpec(
                "tank compile overflowed a physical property".into(),
            ));
        }
        Ok(CompiledTank {
            volume_m3,
            dry_mass_kg,
            max_pressure_pa: self.pressure_pa,
            full_propellant_kg,
        })
    }
}

/// One tank installed on a vehicle: compiled data plus its mount station.
/// Tank mass contributes its intrinsic initial-fill inertia plus the parallel
/// axis term at bake time. `initial_propellant_kg` is optional for backward
/// compatibility; older baked mounts are interpreted as full.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct TankMount {
    pub tank: CompiledTank,
    /// Mount station in vehicle body metres.
    pub position_body_m: [f64; 3],
    /// Intrinsic dry-shell plus initial-load propellant tensor about this
    /// mount's centroid (kg m^2). Missing on legacy baked assets.
    #[serde(default = "zero_tank_inertia")]
    pub intrinsic_inertia_body_kg_m2: DMat3,
    /// Initial loaded propellant mass (kg). `None` means full capacity for
    /// pre-field serialized mounts.
    #[serde(default)]
    pub initial_propellant_kg: Option<f64>,
}

fn zero_tank_inertia() -> DMat3 {
    DMat3::ZERO
}

impl TankMount {
    pub fn loaded_propellant_kg(&self) -> f64 {
        self.initial_propellant_kg
            .unwrap_or(self.tank.full_propellant_kg)
    }

    /// Validate mount data (NaN fails closed).
    pub fn validate(&self) -> Result<(), PropulsionError> {
        if self.position_body_m.iter().any(|v| !v.is_finite()) {
            return Err(PropulsionError::InvalidSpec(
                "tank position must be finite".into(),
            ));
        }
        if !self.tank.volume_m3.is_finite()
            || self.tank.volume_m3 <= 0.0
            || !self.tank.dry_mass_kg.is_finite()
            || self.tank.dry_mass_kg < 0.0
            || !self.tank.max_pressure_pa.is_finite()
            || self.tank.max_pressure_pa <= 0.0
            || !self.tank.full_propellant_kg.is_finite()
            || self.tank.full_propellant_kg < 0.0
        {
            return Err(PropulsionError::InvalidSpec(
                "tank data must be finite and have positive capacity".into(),
            ));
        }
        if let Some(initial) = self.initial_propellant_kg
            && (!initial.is_finite() || initial < 0.0 || initial > self.tank.full_propellant_kg)
        {
            return Err(PropulsionError::InvalidSpec(
                "initial propellant mass must be finite and within tank capacity".into(),
            ));
        }
        let inertia = self.intrinsic_inertia_body_kg_m2;
        if !inertia.is_finite()
            || inertia.x_axis.x < 0.0
            || inertia.y_axis.y < 0.0
            || inertia.z_axis.z < 0.0
            || (inertia.x_axis.y - inertia.y_axis.x).abs() > 1e-9
            || (inertia.x_axis.z - inertia.z_axis.x).abs() > 1e-9
            || (inertia.y_axis.z - inertia.z_axis.y).abs() > 1e-9
        {
            return Err(PropulsionError::InvalidSpec(
                "tank intrinsic inertia must be finite, symmetric and non-negative".into(),
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
    fn tank_intrinsic_inertia_matches_shell_and_solid_special_cases() {
        let sphere = TankShape::Sphere { diameter_m: 2.0 }
            .intrinsic_inertia_body_kg_m2(3.0, 10.0)
            .unwrap();
        // Thin spherical shell: 2/3 m r^2; solid sphere: 2/5 m r^2.
        assert!((sphere.x_axis.x - 6.0).abs() < 1e-12);
        assert!((sphere.y_axis.y - 6.0).abs() < 1e-12);
        assert!((sphere.z_axis.z - 6.0).abs() < 1e-12);

        let cylinder = TankShape::Cylinder {
            diameter_m: 2.0,
            length_m: 4.0,
        }
        .intrinsic_inertia_body_kg_m2(3.0, 10.0)
        .unwrap();
        // The shell's 3 kg are area-partitioned between side (2.4 kg) and
        // caps (0.6 kg); the 10 kg liquid uses a solid-cylinder tensor.
        assert!((cylinder.x_axis.x - 7.7).abs() < 1e-12);
        assert!((cylinder.y_axis.y - 22.783_333_333_333_33).abs() < 1e-12);
        assert!((cylinder.z_axis.z - cylinder.y_axis.y).abs() < 1e-12);
    }

    #[test]
    fn spherical_and_cylindrical_pressure_shells_use_shape_correct_stress() {
        let material = ChamberMaterial {
            density_kg_m3: 1000.0,
            yield_strength_pa: 1.0e9,
            max_wall_temp_k: 1000.0,
        };
        let sphere = TankSpec {
            shape: TankShape::Sphere { diameter_m: 2.0 },
            pressure_pa: 1.0e6,
            material,
        }
        .compile(1.0)
        .unwrap();
        let cylinder = TankSpec {
            shape: TankShape::Cylinder {
                diameter_m: 2.0,
                length_m: 4.0,
            },
            pressure_pa: 1.0e6,
            material,
        }
        .compile(1.0)
        .unwrap();
        let sphere_wall_m = 1.0e6 * 2.0 / (4.0 * 1.0e9) * 1.5;
        let cylinder_wall_m = 1.0e6 * 2.0 / (2.0 * 1.0e9) * 1.5;
        let expected_sphere_mass = std::f64::consts::PI * 4.0 * sphere_wall_m * 1000.0 * 1.15;
        let expected_cylinder_mass = 10.0 * std::f64::consts::PI * cylinder_wall_m * 1000.0 * 1.15;
        assert!((sphere.dry_mass_kg - expected_sphere_mass).abs() < 1e-12);
        assert!((cylinder.dry_mass_kg - expected_cylinder_mass).abs() < 1e-12);
    }

    #[test]
    fn tank_geometry_and_compile_reject_non_finite_or_overflowed_values() {
        assert!(
            TankShape::Sphere {
                diameter_m: f64::INFINITY
            }
            .volume_m3()
            .is_err()
        );
        assert!(
            TankShape::Cylinder {
                diameter_m: 1.0e200,
                length_m: 1.0,
            }
            .volume_m3()
            .is_err()
        );

        let shape = TankShape::Sphere { diameter_m: 1.0 };
        let material = ChamberMaterial::nickel_superalloy();
        assert!(
            TankSpec {
                shape,
                pressure_pa: f64::INFINITY,
                material,
            }
            .compile(1.0)
            .is_err()
        );
        assert!(
            TankSpec {
                shape: TankShape::Sphere { diameter_m: 2.0 },
                pressure_pa: 1.0e6,
                material,
            }
            .compile(f64::MAX)
            .is_err()
        );
    }

    #[test]
    fn tank_mount_tracks_initial_load_separately_from_capacity() {
        let shape = TankShape::Sphere { diameter_m: 1.0 };
        let tank = TankSpec {
            shape,
            pressure_pa: 1.0e6,
            material: ChamberMaterial::nickel_superalloy(),
        }
        .compile(800.0)
        .unwrap();
        let legacy_full = TankMount {
            tank,
            position_body_m: [0.0; 3],
            intrinsic_inertia_body_kg_m2: DMat3::ZERO,
            initial_propellant_kg: None,
        };
        assert_eq!(legacy_full.loaded_propellant_kg(), tank.full_propellant_kg);

        let partial = TankMount {
            initial_propellant_kg: Some(0.25 * tank.full_propellant_kg),
            ..legacy_full
        };
        assert!(partial.validate().is_ok());
        assert_eq!(
            partial.loaded_propellant_kg(),
            0.25 * tank.full_propellant_kg
        );
        let overfilled = TankMount {
            initial_propellant_kg: Some(1.01 * tank.full_propellant_kg),
            ..partial
        };
        assert!(overfilled.validate().is_err());
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
