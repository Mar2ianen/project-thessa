//! Hull structural output and thin-shell mass/inertia math.
//!
//! The compiler integrates skin/cap triangle mass moments and polygonal
//! frame-line moments directly from the loft. The public closed-form helpers
//! below cover idealized circular/solid sections; they are not substituted
//! for the compiled loft's geometric moments. Fuel is a liquid inner solid
//! in the volume model.

use glam::{DMat3, DVec3};
use serde::{Deserialize, Serialize};

/// Compiled hull structure in body-local metres (mount shifts it later).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CompiledHull {
    /// Skin plus frames (kg). Tank pressure shells are separate: they
    /// travel with [`thessa_sim_core::TankMount`] dry mass at assembly.
    pub mass_kg: f64,
    /// Skin-only mass (kg) for hangar display.
    pub skin_mass_kg: f64,
    /// Frames-only mass (kg) for hangar display.
    pub frame_mass_kg: f64,
    /// Center of mass in body-local metres.
    pub center_of_mass_body_m: DVec3,
    /// Inertia about the body-local origin (kg m^2).
    pub inertia_body_kg_m2: DMat3,
    /// Outer-mold lateral area (m^2).
    pub wetted_area_m2: f64,
}

/// Thin-shell inertia for a constant elliptical tube with uniformly
/// distributed perimeter mass, integrated numerically over the outline.
/// For circular sections this reduces to `Ix=m r^2` and transverse
/// `m(r^2/2 + L^2/12)`.
pub fn tube_inertia(mass_kg: f64, semi_a_m: f64, semi_b_m: f64, length_m: f64) -> DMat3 {
    const SAMPLES: usize = 4096;
    let angular_step = std::f64::consts::TAU / SAMPLES as f64;
    let mut previous = (semi_a_m, 0.0);
    let mut weighted_y2 = 0.0;
    let mut weighted_z2 = 0.0;
    let mut perimeter = 0.0;
    for index in 1..=SAMPLES {
        let angle = index as f64 * angular_step;
        let current = (semi_a_m * angle.cos(), semi_b_m * angle.sin());
        let ds = (current.0 - previous.0).hypot(current.1 - previous.1);
        weighted_y2 += 0.5 * (previous.0.powi(2) + current.0.powi(2)) * ds;
        weighted_z2 += 0.5 * (previous.1.powi(2) + current.1.powi(2)) * ds;
        perimeter += ds;
        previous = current;
    }
    let mean_y2 = weighted_y2 / perimeter;
    let mean_z2 = weighted_z2 / perimeter;
    DMat3::from_diagonal(DVec3::new(
        mass_kg * (mean_y2 + mean_z2),
        mass_kg * (mean_z2 + length_m.powi(2) / 12.0),
        mass_kg * (mean_y2 + length_m.powi(2) / 12.0),
    ))
}

/// Solid elliptical-cylinder inertia about its own centroid (fuel liquid).
pub fn solid_inertia(mass_kg: f64, semi_a_m: f64, semi_b_m: f64, length_m: f64) -> DMat3 {
    DMat3::from_diagonal(DVec3::new(
        mass_kg * (semi_a_m.powi(2) + semi_b_m.powi(2)) / 4.0,
        mass_kg * (3.0 * semi_b_m.powi(2) + length_m.powi(2)) / 12.0,
        mass_kg * (3.0 * semi_a_m.powi(2) + length_m.powi(2)) / 12.0,
    ))
}

/// Thin circular-wire ring in the `y/z` plane about its own centroid.
pub fn ring_inertia(mass_kg: f64, radius_m: f64) -> DMat3 {
    DMat3::from_diagonal(DVec3::new(
        mass_kg * radius_m.powi(2),
        0.5 * mass_kg * radius_m.powi(2),
        0.5 * mass_kg * radius_m.powi(2),
    ))
}

/// Point-mass inertia about the origin: `m(|r|^2 I - r r^T)`.
pub fn point_inertia(mass_kg: f64, center: DVec3) -> DMat3 {
    let outer = DMat3::from_cols(center * center.x, center * center.y, center * center.z);
    (DMat3::from_diagonal(DVec3::splat(center.length_squared())) - outer) * mass_kg
}
