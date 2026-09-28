//! Contact geometry for compiled fuselages.
//!
//! One solver-neutral axis-aligned bounding cuboid per authored segment.
//! This keeps the axial envelope exact and bounds both endpoint sections
//! plus their linearly interpolated offsets without capsule end-cap gaps.

use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};
use thessa_sim_core::{CollisionMaterial, CollisionPart, CollisionShape};

use crate::{FuselageError, ProceduralBody};

/// Contact-generation options.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BodyCollisionOptions {
    /// One part per authored body segment. `false` is reserved for a
    /// future merged-hull mode and is rejected in this slice.
    pub per_segment: bool,
    /// Contact friction for the hull skin.
    pub friction: f64,
    /// Contact restitution for the hull skin.
    pub restitution: f64,
}

impl Default for BodyCollisionOptions {
    fn default() -> Self {
        Self {
            per_segment: true,
            friction: 0.7,
            restitution: 0.0,
        }
    }
}

impl BodyCollisionOptions {
    pub fn validate(&self) -> Result<(), FuselageError> {
        if !self.per_segment {
            return Err(FuselageError::InvalidOptions(
                "merged-hull contact is not implemented in this slice".into(),
            ));
        }
        if !self.friction.is_finite() || self.friction < 0.0 {
            return Err(FuselageError::InvalidOptions(
                "contact friction must be finite and non-negative".into(),
            ));
        }
        if !self.restitution.is_finite() || !(0.0..=1.0).contains(&self.restitution) {
            return Err(FuselageError::InvalidOptions(
                "contact restitution must be finite and in [0, 1]".into(),
            ));
        }
        Ok(())
    }
}

/// Contact parts for a compiled body: one part per authored segment in
/// the mounted vehicle frame.
pub fn body_collision_parts(
    body: &ProceduralBody,
    options: &BodyCollisionOptions,
) -> Result<Vec<CollisionPart>, FuselageError> {
    body.validate()?;
    options.validate()?;
    let material = CollisionMaterial::new(options.friction, options.restitution)
        .map_err(|error| FuselageError::InvalidCollision(error.to_string()))?;
    let mut parts = Vec::new();
    for pair in body.stations.windows(2) {
        let (s0, s1) = (pair[0], pair[1]);
        let length = s1.x_m - s0.x_m;
        let center_x = 0.5 * (s0.x_m + s1.x_m);
        let min_y = (s0.offset_y_m - s0.half_width_m).min(s1.offset_y_m - s1.half_width_m);
        let max_y = (s0.offset_y_m + s0.half_width_m).max(s1.offset_y_m + s1.half_width_m);
        let min_z = (s0.offset_z_m - s0.bottom_height_m).min(s1.offset_z_m - s1.bottom_height_m);
        let max_z = (s0.offset_z_m + s0.top_height_m).max(s1.offset_z_m + s1.top_height_m);
        let center =
            DVec3::new(center_x, 0.5 * (min_y + max_y), 0.5 * (min_z + max_z)) + body.origin_body_m;
        let half_extents = DVec3::new(length / 2.0, 0.5 * (max_y - min_y), 0.5 * (max_z - min_z));
        let shape = CollisionShape::Cuboid {
            half_extents_m: half_extents,
        };
        parts.push(
            CollisionPart::new(center, DQuat::IDENTITY, shape, material)
                .map_err(|error| FuselageError::InvalidCollision(error.to_string()))?,
        );
    }
    if parts.is_empty() {
        return Err(FuselageError::InvalidCollision(
            "body produced no contact parts".into(),
        ));
    }
    Ok(parts)
}
