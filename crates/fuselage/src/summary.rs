//! Compiled-body telemetry for goldens and debug views.
//!
//! Every number here is derived from the authored stations by the same
//! code path as the solver inputs, so a golden failure reads as either a
//! compiler regression or a changed documented assumption.

use glam::DVec3;
use serde::{Deserialize, Serialize};

/// Geometry-only telemetry record for one compiled body.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct CompiledBodySummary {
    /// Body name from authoring.
    pub name: String,
    /// Overall length in metres.
    pub length_m: f64,
    /// Maximum diameter (twice the largest semi-axis) in metres.
    pub max_diameter_m: f64,
    /// Maximum cross-section area in m^2 (body coefficient reference).
    pub frontal_area_m2: f64,
    /// Tail-base area in m^2 (base-drag bookkeeping).
    pub base_area_m2: f64,
    /// Outer-mold lateral plus disc area in m^2 (skin-friction bookkeeping).
    pub wetted_area_m2: f64,
    /// Enclosed outer-mold volume in m^3.
    pub enclosed_volume_m3: f64,
    /// Summed adaptive-Simpson absolute-error estimate for the enclosed
    /// outer-mold volume (m^3).
    #[serde(default)]
    pub volume_error_m3: f64,
    /// Enclosed center of volume in vehicle metres.
    pub center_of_volume_m: DVec3,
    /// Hull shell plus frames plus manifest mass in kg.
    pub dry_mass_kg: f64,
    /// Summed tank-region inner volume in m^3.
    pub tank_capacity_m3: f64,
    /// Axial aero zone count.
    pub zone_count: usize,
    /// Solver panel count (two per zone).
    pub panel_count: usize,
    /// Estimated absolute area error for lateral skin and end-cap polygons
    /// (axial/angular Richardson refinement, m^2; not a rigorous bound).
    #[serde(default, alias = "perimeter_error_m2")]
    pub lateral_area_error_m2: f64,
}

impl CompiledBodySummary {
    #[allow(clippy::too_many_arguments)]
    pub fn build(
        name: &str,
        length_m: f64,
        max_diameter_m: f64,
        frontal_area_m2: f64,
        base_area_m2: f64,
        wetted_area_m2: f64,
        enclosed_volume_m3: f64,
        volume_error_m3: f64,
        center_of_volume_m: DVec3,
        dry_mass_kg: f64,
        tank_capacity_m3: f64,
        zone_count: usize,
        panel_count: usize,
        lateral_area_error_m2: f64,
    ) -> Self {
        Self {
            name: name.into(),
            length_m,
            max_diameter_m,
            frontal_area_m2,
            base_area_m2,
            wetted_area_m2,
            enclosed_volume_m3,
            volume_error_m3,
            center_of_volume_m,
            dry_mass_kg,
            tank_capacity_m3,
            zone_count,
            panel_count,
            lateral_area_error_m2,
        }
    }
}
