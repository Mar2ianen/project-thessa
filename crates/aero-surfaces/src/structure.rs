//! Structural mass and fuel-volume estimation from material and thickness.
//!
//! The compiler already knows every zone's area, span, chord, and
//! thickness. With an authored [`StructuralLayout`] (real manufacturing
//! parameters: skin gauge, spar gauges, material densities) it additionally
//! derives primary-structure mass from first principles, plus two explicit
//! layout fractions with textbook-anchored defaults (spar caps, secondary
//! structure) and a wing-box fuel volume. No statistical weight formulas,
//! no hidden coefficients: every number is geometry times density or a
//! declared layout choice the tests vary.
//!
//! Scope limits, documented: spar caps are a declared multiple of web
//! mass (loads-sized caps need the structural-graph slice); ribs,
//! fasteners, and systems ride the secondary fraction; control-surface
//! gaps are not cut from the skin (conservative); inertia is a per-zone
//! point-mass assembly about the body origin, O(zone^2)-accurate.
//! Tank mounts are NOT fabricated here: `TankShape` has no wing-box
//! variant, so the compiler reports volume plus centroid and the baker
//! aggregates mass while a fake cylinder would lie about packaging.

use glam::{DMat3, DVec3};
use serde::{Deserialize, Serialize};

use crate::SurfaceError;

/// Structural solid with handbook density. Presets are typical values,
/// not mill certs: 7075-T6 / 2024-T3 sheet, quasi-isotropic CFRP
/// laminate, Ti-6Al-4V, structural steel.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SolidMaterial {
    /// Material name for hangar display and golden records.
    pub name: String,
    /// Mass density in kg/m^3.
    pub density_kg_m3: f64,
}

impl SolidMaterial {
    /// Aluminum 7075-T6, 2810 kg/m^3.
    pub fn aluminum_7075() -> Self {
        Self {
            name: "Al-7075-T6".into(),
            density_kg_m3: 2810.0,
        }
    }

    /// Aluminum 2024-T3, 2780 kg/m^3.
    pub fn aluminum_2024() -> Self {
        Self {
            name: "Al-2024-T3".into(),
            density_kg_m3: 2780.0,
        }
    }

    /// Quasi-isotropic carbon laminate, 1600 kg/m^3 typical.
    pub fn carbon_fiber() -> Self {
        Self {
            name: "CFRP-quasi-iso".into(),
            density_kg_m3: 1600.0,
        }
    }

    /// Titanium Ti-6Al-4V, 4430 kg/m^3.
    pub fn titanium() -> Self {
        Self {
            name: "Ti-6Al-4V".into(),
            density_kg_m3: 4430.0,
        }
    }

    fn validate(&self) -> Result<(), SurfaceError> {
        if self.name.trim().is_empty() {
            return Err(SurfaceError::InvalidSurface(
                "structural material needs a name".into(),
            ));
        }
        if !self.density_kg_m3.is_finite() || self.density_kg_m3 <= 0.0 {
            return Err(SurfaceError::InvalidSurface(format!(
                "material '{}' density must be positive and finite",
                self.name
            )));
        }
        Ok(())
    }
}

/// Authored structural layout: manufacturing gauges plus declared
/// layout fractions. Everything the mass integral multiplies is either
/// compiled geometry, a gauge in metres, or one of the two documented
/// fractions below.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StructuralLayout {
    /// Skin material (both sides).
    pub skin_material: SolidMaterial,
    /// Skin gauge per side in mm (manufacturing parameter).
    pub skin_gauge_mm: f64,
    /// Spar web material (front and rear webs).
    pub spar_material: SolidMaterial,
    /// Spar web height as a fraction of local section thickness.
    pub spar_depth_fraction: f64,
    /// Spar shear-web gauge in mm.
    pub spar_web_gauge_mm: f64,
    /// Spar caps as a multiple of web mass (loads-sized caps need the
    /// structural-graph slice; textbook wing breakdowns put caps at
    /// roughly 1.5-2x web mass, default 1.5).
    pub spar_cap_fraction: f64,
    /// Ribs, fasteners, sealant, and miscellany as a fraction of primary
    /// (skin plus spar) mass. Textbook breakdowns land near 0.15-0.25;
    /// default 0.2.
    pub secondary_fraction: f64,
    /// Usable fuel-box chord fractions `(front, rear)`, e.g. the
    /// 15-65 percent box between the spars.
    pub fuel_box_chord: (f64, f64),
    /// Fill efficiency: baffles, ribs, and unusable corners displace
    /// roughly 10-20 percent; default 0.85.
    pub fuel_fill_efficiency: f64,
}

impl StructuralLayout {
    /// Light metal baseline: 2 mm 7075 skin, 3 mm spar webs at 60 percent
    /// depth, 15-65 percent fuel box.
    pub fn metal_baseline() -> Self {
        Self {
            skin_material: SolidMaterial::aluminum_7075(),
            skin_gauge_mm: 2.0,
            spar_material: SolidMaterial::aluminum_7075(),
            spar_depth_fraction: 0.6,
            spar_web_gauge_mm: 3.0,
            spar_cap_fraction: 1.5,
            secondary_fraction: 0.2,
            fuel_box_chord: (0.15, 0.65),
            fuel_fill_efficiency: 0.85,
        }
    }

    /// Check the layout contract: positive finite gauges, fractions in
    /// range, ordered fuel box inside [0, 1].
    pub fn validate(&self) -> Result<(), SurfaceError> {
        self.skin_material.validate()?;
        self.spar_material.validate()?;
        for (label, value) in [
            ("skin gauge", self.skin_gauge_mm),
            ("spar web gauge", self.spar_web_gauge_mm),
        ] {
            if !value.is_finite() || value <= 0.0 {
                return Err(SurfaceError::InvalidSurface(format!(
                    "structural {label} must be positive and finite (mm)"
                )));
            }
        }
        for (label, value, bound) in [
            ("spar depth fraction", self.spar_depth_fraction, 1.0),
            ("spar cap fraction", self.spar_cap_fraction, 5.0),
            ("secondary fraction", self.secondary_fraction, 1.0),
            ("fuel fill efficiency", self.fuel_fill_efficiency, 1.0),
        ] {
            if !value.is_finite() || value < 0.0 || value > bound {
                return Err(SurfaceError::InvalidSurface(format!(
                    "structural {label} must be in [0, {bound}]"
                )));
            }
        }
        let (box_front, box_rear) = self.fuel_box_chord;
        if !box_front.is_finite()
            || !box_rear.is_finite()
            || !(0.0..=1.0).contains(&box_front)
            || !(0.0..=1.0).contains(&box_rear)
            || box_front >= box_rear
        {
            return Err(SurfaceError::InvalidSurface(
                "fuel box needs ordered chord fractions inside [0, 1]".into(),
            ));
        }
        Ok(())
    }
}

/// Compiled structural output for one surface: mass breakdown, center of
/// mass, body-origin inertia, and usable fuel volume with its centroid.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CompiledStructure {
    /// Total structural mass in kg (primary plus secondary).
    pub mass_kg: f64,
    /// Skin mass in kg.
    pub skin_mass_kg: f64,
    /// Spar mass in kg (webs plus caps).
    pub spar_mass_kg: f64,
    /// Mass-weighted center of mass in body metres.
    pub center_of_mass_body_m: DVec3,
    /// Inertia tensor about the body origin in kg m^2 (per-zone
    /// point-mass assembly at panel centers of pressure).
    pub inertia_body_kg_m2: DMat3,
    /// Usable fuel volume in m^3 (wing box times fill efficiency).
    pub fuel_volume_m3: f64,
    /// Volume-weighted fuel centroid in body metres (tank placement).
    pub fuel_centroid_body_m: DVec3,
}

impl Default for CompiledStructure {
    fn default() -> Self {
        Self {
            mass_kg: 0.0,
            skin_mass_kg: 0.0,
            spar_mass_kg: 0.0,
            center_of_mass_body_m: DVec3::ZERO,
            inertia_body_kg_m2: DMat3::ZERO,
            fuel_volume_m3: 0.0,
            fuel_centroid_body_m: DVec3::ZERO,
        }
    }
}

impl CompiledStructure {
    /// Mirror across the body `x/z` plane: scalars stay, centers flip
    /// `y`, inertia off-diagonals conjugate by `diag(1,-1,1)`.
    pub(crate) fn mirrored(&self) -> Self {
        let mirror_point = |point: DVec3| DVec3::new(point.x, -point.y, point.z);
        let mut inertia = self.inertia_body_kg_m2;
        inertia.y_axis.x = -inertia.y_axis.x;
        inertia.x_axis.y = -inertia.x_axis.y;
        inertia.y_axis.z = -inertia.y_axis.z;
        inertia.z_axis.y = -inertia.z_axis.y;
        Self {
            mass_kg: self.mass_kg,
            skin_mass_kg: self.skin_mass_kg,
            spar_mass_kg: self.spar_mass_kg,
            center_of_mass_body_m: mirror_point(self.center_of_mass_body_m),
            inertia_body_kg_m2: inertia,
            fuel_volume_m3: self.fuel_volume_m3,
            fuel_centroid_body_m: mirror_point(self.fuel_centroid_body_m),
        }
    }
}
