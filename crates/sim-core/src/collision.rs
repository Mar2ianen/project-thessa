//! Backend-neutral collision geometry owned by the authoritative simulation.
//!
//! These types deliberately describe physical collision shapes without
//! exposing Rapier (or any other solver) handles/types. A vehicle asset can be
//! compiled once into this representation and consumed by the authoritative
//! collision backend, debug tooling, or a future replacement backend.

use std::{error::Error, fmt};

use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};

const MIN_DIMENSION_M: f64 = 1.0e-6;
const QUATERNION_TOLERANCE: f64 = 1.0e-6;

/// Contact material parameters expressed directly in physical solver terms.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CollisionMaterial {
    /// Coulomb friction coefficient. Values above one are valid for sufficiently
    /// adhesive/high-friction material pairs, so only non-negativity is enforced.
    pub friction: f64,
    /// Normal coefficient of restitution in `[0, 1]`.
    pub restitution: f64,
}

impl Default for CollisionMaterial {
    fn default() -> Self {
        Self {
            friction: 0.7,
            restitution: 0.0,
        }
    }
}

impl CollisionMaterial {
    pub fn new(friction: f64, restitution: f64) -> Result<Self, CollisionError> {
        let material = Self {
            friction,
            restitution,
        };
        material.validate()?;
        Ok(material)
    }

    pub fn validate(self) -> Result<(), CollisionError> {
        if !self.friction.is_finite() || self.friction < 0.0 {
            return Err(CollisionError::InvalidMaterial(
                "friction must be finite and non-negative".into(),
            ));
        }
        if !self.restitution.is_finite() || !(0.0..=1.0).contains(&self.restitution) {
            return Err(CollisionError::InvalidMaterial(
                "restitution must be finite and in [0, 1]".into(),
            ));
        }
        Ok(())
    }
}

/// Principal axis of a capsule's cylindrical segment.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum CollisionAxis {
    X,
    Y,
    Z,
}

/// Convex primitive used for dynamic vehicle collision geometry.
///
/// Non-convex dynamic geometry should be compiled into several convex parts
/// rather than represented by a triangle mesh. Terrain meshes belong to the
/// world collision backend, not inside a vehicle asset.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum CollisionShape {
    Sphere {
        radius_m: f64,
    },
    Cuboid {
        half_extents_m: DVec3,
    },
    Capsule {
        axis: CollisionAxis,
        /// Half-length of the line segment between the two spherical caps.
        half_segment_m: f64,
        radius_m: f64,
    },
}

impl CollisionShape {
    pub fn validate(self) -> Result<(), CollisionError> {
        match self {
            Self::Sphere { radius_m } => validate_positive(radius_m, "sphere radius"),
            Self::Cuboid { half_extents_m } => {
                if !half_extents_m.is_finite()
                    || half_extents_m.x < MIN_DIMENSION_M
                    || half_extents_m.y < MIN_DIMENSION_M
                    || half_extents_m.z < MIN_DIMENSION_M
                {
                    return Err(CollisionError::InvalidShape(
                        "cuboid half-extents must be finite and positive".into(),
                    ));
                }
                Ok(())
            }
            Self::Capsule {
                half_segment_m,
                radius_m,
                ..
            } => {
                validate_positive(half_segment_m, "capsule half-segment")?;
                validate_positive(radius_m, "capsule radius")
            }
        }
    }
}

/// One primitive located in vehicle body coordinates.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CollisionPart {
    pub local_position_m: DVec3,
    pub local_orientation: DQuat,
    pub shape: CollisionShape,
    pub material: CollisionMaterial,
}

impl CollisionPart {
    pub fn new(
        local_position_m: DVec3,
        local_orientation: DQuat,
        shape: CollisionShape,
        material: CollisionMaterial,
    ) -> Result<Self, CollisionError> {
        let part = Self {
            local_position_m,
            local_orientation,
            shape,
            material,
        };
        part.validate()?;
        Ok(part)
    }

    pub fn validate(self) -> Result<(), CollisionError> {
        if !self.local_position_m.is_finite() || !self.local_orientation.is_finite() {
            return Err(CollisionError::InvalidPose(
                "collision-part pose contains a non-finite value".into(),
            ));
        }
        let orientation_error = (self.local_orientation.length_squared() - 1.0).abs();
        if orientation_error > QUATERNION_TOLERANCE {
            return Err(CollisionError::InvalidPose(
                "collision-part orientation must be a unit quaternion".into(),
            ));
        }
        self.shape.validate()?;
        self.material.validate()
    }
}

/// Collision representation compiled from one connected rigid vehicle.
///
/// Empty geometry is permitted so old/partial vehicle assets remain loadable;
/// it means the vehicle deliberately has no contact representation yet.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct CollisionGeometry {
    pub parts: Vec<CollisionPart>,
}

impl CollisionGeometry {
    pub fn new(parts: Vec<CollisionPart>) -> Result<Self, CollisionError> {
        let geometry = Self { parts };
        geometry.validate()?;
        Ok(geometry)
    }

    pub fn is_empty(&self) -> bool {
        self.parts.is_empty()
    }

    pub fn validate(&self) -> Result<(), CollisionError> {
        for part in &self.parts {
            part.validate()?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum CollisionError {
    InvalidMaterial(String),
    InvalidShape(String),
    InvalidPose(String),
}

impl fmt::Display for CollisionError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidMaterial(message) => write!(formatter, "invalid collision material: {message}"),
            Self::InvalidShape(message) => write!(formatter, "invalid collision shape: {message}"),
            Self::InvalidPose(message) => write!(formatter, "invalid collision pose: {message}"),
        }
    }
}

impl Error for CollisionError {}

fn validate_positive(value: f64, label: &str) -> Result<(), CollisionError> {
    if !value.is_finite() || value < MIN_DIMENSION_M {
        return Err(CollisionError::InvalidShape(format!(
            "{label} must be finite and at least {MIN_DIMENSION_M} m"
        )));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_non_unit_part_orientation() {
        let result = CollisionPart::new(
            DVec3::ZERO,
            DQuat::from_xyzw(0.0, 0.0, 0.0, 2.0),
            CollisionShape::Sphere { radius_m: 1.0 },
            CollisionMaterial::default(),
        );
        assert!(matches!(result, Err(CollisionError::InvalidPose(_))));
    }

    #[test]
    fn empty_geometry_is_a_valid_migration_state() {
        assert!(CollisionGeometry::default().validate().is_ok());
    }
}
