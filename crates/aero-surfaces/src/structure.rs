//! Structural mass and fuel-volume estimation from material and thickness.
//!
//! The compiler already knows every zone's area, span, chord, and
//! thickness. With an authored [`StructuralLayout`] (real manufacturing
//! parameters: gauges, spacings, material data, and the sizing load
//! case) it derives structure from first principles: skin and spar webs
//! as geometry times density, ribs as plates of the section shape at an
//! authored spacing, spar caps sized from the bending moment under the
//! authored limit lift (Schrenk distribution integrated spanwise, cap
//! area from allowable stress), and fuel volume as the wing box minus
//! explicit rib displacement and sump.
//!
//! Deliberately absent (cut as observation-fitted physics): statistical
//! weight formulas, cap/secondary mass fractions, fill-efficiency
//! tuning, stall brackets. What remains unmodeled is documented, not
//! hidden: fasteners, sealant, systems, and loads-sized details beyond
//! caps (joints, cutouts) are not in the total; the mass is a
//! conservative primary-structure estimate, exact in its own terms.
//! Tank mounts are NOT fabricated: `TankShape` has no wing-box variant,
//! so the compiler reports volume plus centroid and the baker aggregates
//! mass while a fake cylinder would lie about packaging.

use glam::{DMat3, DVec3};
use serde::{Deserialize, Serialize};

use crate::{Naca4, SurfaceError};

/// Structural solid with handbook data. Strength values are typical
/// yield-ish analysis allowables (7075-T6 Fty 503 MPa, 2024-T3 Fty 345
/// MPa, Ti-6Al-4V ~880 MPa, structural steel ~800 MPa; CFRP 400 MPa is a
/// deliberately conservative generic laminate value, layup-dependent in
/// reality). No safety factors: analysis, not certification.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SolidMaterial {
    /// Material name for hangar display and golden records.
    pub name: String,
    /// Mass density in kg/m^3.
    pub density_kg_m3: f64,
    /// Allowable tensile stress in MPa for cap sizing.
    pub allowable_stress_mpa: f64,
}

impl SolidMaterial {
    /// Aluminum 7075-T6: 2810 kg/m^3, 503 MPa.
    pub fn aluminum_7075() -> Self {
        Self {
            name: "Al-7075-T6".into(),
            density_kg_m3: 2810.0,
            allowable_stress_mpa: 503.0,
        }
    }

    /// Aluminum 2024-T3: 2780 kg/m^3, 345 MPa.
    pub fn aluminum_2024() -> Self {
        Self {
            name: "Al-2024-T3".into(),
            density_kg_m3: 2780.0,
            allowable_stress_mpa: 345.0,
        }
    }

    /// Quasi-isotropic carbon laminate: 1600 kg/m^3, conservative 400 MPa.
    pub fn carbon_fiber() -> Self {
        Self {
            name: "CFRP-quasi-iso".into(),
            density_kg_m3: 1600.0,
            allowable_stress_mpa: 400.0,
        }
    }

    /// Titanium Ti-6Al-4V: 4430 kg/m^3, 880 MPa.
    pub fn titanium() -> Self {
        Self {
            name: "Ti-6Al-4V".into(),
            density_kg_m3: 4430.0,
            allowable_stress_mpa: 880.0,
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
        if !self.allowable_stress_mpa.is_finite() || self.allowable_stress_mpa <= 0.0 {
            return Err(SurfaceError::InvalidSurface(format!(
                "material '{}' allowable must be positive and finite (MPa)",
                self.name
            )));
        }
        Ok(())
    }
}

/// Authored structural layout: manufacturing gauges, spacings, material
/// data, and the sizing load case. Everything the mass integral
/// multiplies is compiled geometry, a gauge, a spacing, handbook data,
/// or the authored limit load (the same handoff real loads groups give
/// to stress: a limit-load scalar, not a fitted curve).
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
    /// Rib pitch in metres (manufacturing parameter).
    pub rib_spacing_m: f64,
    /// Rib plate gauge in mm (solid-plate ribs: a conservative upper
    /// bound, lightening holes not modeled).
    pub rib_gauge_mm: f64,
    /// Limit lift the surface must carry in newtons (sizing load case,
    /// e.g. 2.5 g times the carried weight share). Caps are sized from
    /// its Schrenk-distributed bending moment.
    pub design_limit_lift_n: f64,
    /// Usable fuel-box chord fractions `(front, rear)`, e.g. the
    /// 15-65 percent box between the spars.
    pub fuel_box_chord: (f64, f64),
    /// Unusable sump/trapped fraction of the box (operational
    /// requirement, typically a few percent).
    pub fuel_sump_fraction: f64,
}

impl StructuralLayout {
    /// Light metal baseline for a given sizing load: 2 mm 7075 skin,
    /// 3 mm spar webs at 60 percent depth, 0.5 m rib pitch with 1.0 mm
    /// ribs, 15-65 percent fuel box, 3 percent sump.
    pub fn metal_baseline(design_limit_lift_n: f64) -> Self {
        Self {
            skin_material: SolidMaterial::aluminum_7075(),
            skin_gauge_mm: 2.0,
            spar_material: SolidMaterial::aluminum_7075(),
            spar_depth_fraction: 0.6,
            spar_web_gauge_mm: 3.0,
            rib_spacing_m: 0.5,
            rib_gauge_mm: 1.0,
            design_limit_lift_n,
            fuel_box_chord: (0.15, 0.65),
            fuel_sump_fraction: 0.03,
        }
    }

    /// Check the layout contract: positive finite gauges/spacings/load,
    /// fractions in range, ordered fuel box inside [0, 1].
    pub fn validate(&self) -> Result<(), SurfaceError> {
        self.skin_material.validate()?;
        self.spar_material.validate()?;
        for (label, value) in [
            ("skin gauge", self.skin_gauge_mm),
            ("spar web gauge", self.spar_web_gauge_mm),
            ("rib spacing", self.rib_spacing_m),
            ("rib gauge", self.rib_gauge_mm),
            ("design limit lift", self.design_limit_lift_n),
        ] {
            if !value.is_finite() || value <= 0.0 {
                return Err(SurfaceError::InvalidSurface(format!(
                    "structural {label} must be positive and finite"
                )));
            }
        }
        if !self.spar_depth_fraction.is_finite() || !(0.0..=1.0).contains(&self.spar_depth_fraction)
        {
            return Err(SurfaceError::InvalidSurface(
                "spar depth fraction must be in [0, 1]".into(),
            ));
        }
        if !self.fuel_sump_fraction.is_finite() || !(0.0..=1.0).contains(&self.fuel_sump_fraction) {
            return Err(SurfaceError::InvalidSurface(
                "fuel sump fraction must be in [0, 1]".into(),
            ));
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
    /// Total structural mass in kg (skin plus spar plus ribs).
    pub mass_kg: f64,
    /// Skin mass in kg.
    pub skin_mass_kg: f64,
    /// Spar web mass in kg.
    pub spar_web_mass_kg: f64,
    /// Spar cap mass in kg (bending-sized under the limit lift).
    pub spar_cap_mass_kg: f64,
    /// Rib mass in kg (solid plates at the authored pitch).
    pub rib_mass_kg: f64,
    /// Mass-weighted center of mass in body metres.
    pub center_of_mass_body_m: DVec3,
    /// Inertia tensor about the body origin in kg m^2 (per-zone
    /// point-mass assembly at panel centers of pressure).
    pub inertia_body_kg_m2: DMat3,
    /// Usable fuel volume in m^3 (wing box minus rib displacement and
    /// sump, never negative).
    pub fuel_volume_m3: f64,
    /// Volume-weighted fuel centroid in body metres (tank placement).
    pub fuel_centroid_body_m: DVec3,
}

impl Default for CompiledStructure {
    fn default() -> Self {
        Self {
            mass_kg: 0.0,
            skin_mass_kg: 0.0,
            spar_web_mass_kg: 0.0,
            spar_cap_mass_kg: 0.0,
            rib_mass_kg: 0.0,
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
            spar_web_mass_kg: self.spar_web_mass_kg,
            spar_cap_mass_kg: self.spar_cap_mass_kg,
            rib_mass_kg: self.rib_mass_kg,
            center_of_mass_body_m: mirror_point(self.center_of_mass_body_m),
            inertia_body_kg_m2: inertia,
            fuel_volume_m3: self.fuel_volume_m3,
            fuel_centroid_body_m: mirror_point(self.fuel_centroid_body_m),
        }
    }
}

/// Schrenk spanwise load share at normalized station `eta in [0, 1]`
/// (root to tip): the average of the trapezoid (chord over mean chord)
/// and the elliptical distribution. Analytic, no fitting.
pub(crate) fn schrenk_share(chord_here: f64, mean_chord: f64, eta: f64) -> f64 {
    let trapezoid = chord_here / mean_chord.max(1e-12);
    let elliptic = (4.0 / std::f64::consts::PI) * (1.0 - eta * eta).max(0.0).sqrt();
    0.5 * (trapezoid + elliptic)
}

/// Spar-cap cross-section area in m^2 for a bending moment: two caps
/// (upper/lower) share the couple over the spar depth arm,
/// `A = M / (allowable * depth)`.
pub(crate) fn cap_area_m2(moment_nm: f64, allowable_pa: f64, depth_m: f64) -> f64 {
    if allowable_pa <= 0.0 || depth_m <= 0.0 {
        return 0.0;
    }
    moment_nm.max(0.0) / (allowable_pa * depth_m)
}

/// Section plate area coefficient is [`Naca4::area_coefficient`]: rib
/// plates scale with chord squared times the computed family constant.
pub(crate) fn rib_area_coefficient() -> f64 {
    Naca4::area_coefficient()
}

/// Partial plate coefficient over a chord-fraction interval, for rib
/// displacement inside the fuel box only.
pub(crate) fn rib_area_coefficient_range(u0: f64, u1: f64) -> f64 {
    Naca4::area_coefficient_range(u0, u1)
}
