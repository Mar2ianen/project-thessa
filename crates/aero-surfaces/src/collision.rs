//! Contact geometry compiled from aerodynamic zones.
//!
//! The solver never sees this module: contact needs volumes where the
//! solver needs zones. Each compiled panel becomes an oriented cuboid
//! (chordwise x, spanwise y, normal z in part-local axes) with the
//! panel's thickness, floored at 1 mm so idealized zero-gauge sections
//! still present a contact volume. Ownership merges panels of one
//! mechanism region into a single body-axis box each, because a contact
//! solver wants a handful of rigid volumes per independently moving
//! part, not hundreds of zones. Merged boxes cover the union bounding
//! box of their panels exactly; orientation inside a bent region is
//! conservatively absorbed by the axis-aligned union.

use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};
use thessa_sim_core::{AeroPanel, CollisionMaterial, CollisionPart};

use crate::{CompiledSurface, SurfaceError};

/// How contact geometry compiles from zones.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CollisionOptions {
    /// Merge panels sharing fold/control ownership into one body-axis
    /// box per mechanism region. `false` keeps one oriented cuboid per
    /// zone (debugging and fine contact work, heavy for a solver).
    pub merge_regions: bool,
    /// Contact friction for the compiled skin.
    pub friction: f64,
    /// Contact restitution for the compiled skin.
    pub restitution: f64,
}

impl Default for CollisionOptions {
    fn default() -> Self {
        Self {
            merge_regions: true,
            friction: 0.4,
            restitution: 0.05,
        }
    }
}

impl CollisionOptions {
    fn validate(&self) -> Result<(), SurfaceError> {
        if !self.friction.is_finite() || self.friction < 0.0 {
            return Err(SurfaceError::InvalidCollision(
                "contact friction must be finite and non-negative".into(),
            ));
        }
        if !self.restitution.is_finite() || !(0.0..=1.0).contains(&self.restitution) {
            return Err(SurfaceError::InvalidCollision(
                "contact restitution must be in [0, 1]".into(),
            ));
        }
        Ok(())
    }
}

impl CompiledSurface {
    /// Contact parts for the compiled panels: one oriented cuboid per
    /// zone, or one body-axis box per mechanism region when merging.
    /// Positions are body metres in the compiled mechanism state; fold
    /// state changes need a recompile (hangar-side, like everything
    /// else here).
    pub fn collision_parts(
        &self,
        options: &CollisionOptions,
    ) -> Result<Vec<CollisionPart>, SurfaceError> {
        options.validate()?;
        let material = CollisionMaterial::new(options.friction, options.restitution)
            .map_err(|error| SurfaceError::InvalidCollision(error.to_string()))?;
        if options.merge_regions {
            self.merged_region_boxes(material)
        } else {
            self.zone_boxes(material)
        }
    }

    /// One oriented cuboid per zone: part-local x chordwise, y
    /// spanwise, z along the panel normal.
    fn zone_boxes(&self, material: CollisionMaterial) -> Result<Vec<CollisionPart>, SurfaceError> {
        self.panels
            .iter()
            .map(|panel| {
                // Solver side axis keeps the basis right-handed:
                // chord x side = lift.
                let span_dir = panel.lift_axis_body.cross(panel.chord_axis_body);
                let orientation =
                    basis_to_quat(panel.chord_axis_body, span_dir, panel.lift_axis_body);
                let half_extents = DVec3::new(
                    0.5 * panel.chord_m,
                    0.5 * panel.span_m,
                    zone_half_thickness(panel),
                );
                CollisionPart::new(
                    panel.center_of_pressure_body_m,
                    orientation,
                    thessa_sim_core::CollisionShape::Cuboid {
                        half_extents_m: half_extents,
                    },
                    material,
                )
                .map_err(|error| SurfaceError::InvalidCollision(error.to_string()))
            })
            .collect()
    }

    /// One body-axis box per (fold, control) ownership region: the exact
    /// union bounding box of its panels' corners.
    fn merged_region_boxes(
        &self,
        material: CollisionMaterial,
    ) -> Result<Vec<CollisionPart>, SurfaceError> {
        use std::collections::BTreeMap;

        let mut groups: BTreeMap<(Option<usize>, Option<usize>), (DVec3, DVec3)> = BTreeMap::new();
        for (panel, tag) in self.panels.iter().zip(self.tags.iter()) {
            let span_dir = panel.lift_axis_body.cross(panel.chord_axis_body);
            let axes = [
                (panel.chord_axis_body, 0.5 * panel.chord_m),
                (span_dir, 0.5 * panel.span_m),
                (panel.lift_axis_body, zone_half_thickness(panel)),
            ];
            let mut corner_min = DVec3::splat(f64::INFINITY);
            let mut corner_max = DVec3::splat(f64::NEG_INFINITY);
            for signs in 0..8 {
                let mut corner = panel.center_of_pressure_body_m;
                for (axis_index, (axis, half)) in axes.iter().enumerate() {
                    let sign = if signs & (1 << axis_index) == 0 {
                        -1.0
                    } else {
                        1.0
                    };
                    corner += *axis * (sign * half);
                }
                corner_min = corner_min.min(corner);
                corner_max = corner_max.max(corner);
            }
            let entry = groups
                .entry((tag.fold, tag.control))
                .or_insert((DVec3::splat(f64::INFINITY), DVec3::splat(f64::NEG_INFINITY)));
            entry.0 = entry.0.min(corner_min);
            entry.1 = entry.1.max(corner_max);
        }
        groups
            .values()
            .map(|(min, max)| {
                CollisionPart::new(
                    0.5 * (*min + *max),
                    DQuat::IDENTITY,
                    thessa_sim_core::CollisionShape::Cuboid {
                        half_extents_m: 0.5 * (*max - *min),
                    },
                    material,
                )
                .map_err(|error| SurfaceError::InvalidCollision(error.to_string()))
            })
            .collect()
    }
}

/// Panel thickness as a contact half-depth, floored at 1 mm: idealized
/// sections may author zero gauge, but a contact solver needs a volume.
fn zone_half_thickness(panel: &AeroPanel) -> f64 {
    (0.5 * panel.thickness_to_chord_ratio * panel.chord_m).max(1e-3)
}

/// Rotation taking part-local (x, y, z) onto the given orthonormal
/// frame columns.
fn basis_to_quat(x: DVec3, y: DVec3, z: DVec3) -> DQuat {
    DQuat::from_mat3(&glam::DMat3::from_cols(x, y, z))
}
