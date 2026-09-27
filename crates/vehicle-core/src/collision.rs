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

impl CollisionAxis {
    fn vector(self) -> DVec3 {
        match self {
            Self::X => DVec3::X,
            Self::Y => DVec3::Y,
            Self::Z => DVec3::Z,
        }
    }
}

/// Minimum positive ray-hit distance: receivers sitting exactly on a part
/// surface do not self-shadow.
const RAY_HIT_EPS_M: f64 = 1.0e-9;

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

    /// Conservative bounding-sphere radius for occlusion discs.
    pub fn bounding_radius_m(self) -> f64 {
        match self {
            Self::Sphere { radius_m } => radius_m,
            Self::Cuboid { half_extents_m } => half_extents_m.length(),
            Self::Capsule {
                half_segment_m,
                radius_m,
                ..
            } => half_segment_m + radius_m,
        }
    }

    /// Point containment in the part-local frame (shape at origin, canonical
    /// orientation). Boundary counts as inside.
    fn contains_point_local(self, point: DVec3) -> bool {
        match self {
            Self::Sphere { radius_m } => point.length() <= radius_m,
            Self::Cuboid { half_extents_m } => {
                point.x.abs() <= half_extents_m.x
                    && point.y.abs() <= half_extents_m.y
                    && point.z.abs() <= half_extents_m.z
            }
            Self::Capsule {
                axis,
                half_segment_m,
                radius_m,
            } => {
                let along = (point.dot(axis.vector()) / half_segment_m).clamp(-1.0, 1.0);
                (point - axis.vector() * (along * half_segment_m)).length() <= radius_m
            }
        }
    }

    /// Nearest forward ray entry distance in the part-local frame, or `None`.
    fn ray_hit_t(self, origin: DVec3, direction: DVec3) -> Option<f64> {
        if direction.length_squared() <= 0.0 {
            return None;
        }
        match self {
            Self::Sphere { radius_m } => ray_sphere_t(origin, direction, radius_m),
            Self::Cuboid { half_extents_m } => ray_cuboid_t(origin, direction, half_extents_m),
            Self::Capsule {
                axis,
                half_segment_m,
                radius_m,
            } => ray_capsule_t(origin, direction, axis.vector(), half_segment_m, radius_m),
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

    /// Own-body solar occlusion as seen from a receiver point: at most one
    /// disc `(direction, angular_radius_rad)`.
    ///
    /// Pure CPU geometric ray query over the authored contact parts — the
    /// canonical server path, no GPU needed. For every part the receiver
    /// sits outside of, a ray toward the sun is tested against the true
    /// shape; each hit contributes its bounding-sphere disc and the largest
    /// wins (conservative: a small strut in front of a big hull still yields
    /// the hull's full shadow). A receiver inside any part means full-sky
    /// blockage (`π`). Silhouette-edge penumbra of near misses is not
    /// modeled; finite-sun-disc penumbra is handled downstream by the
    /// circle-overlap eclipse factor.
    pub fn own_body_occluder(
        &self,
        receiver_body_m: DVec3,
        sun_direction_body: DVec3,
    ) -> Option<(DVec3, f64)> {
        if !receiver_body_m.is_finite()
            || !sun_direction_body.is_finite()
            || (sun_direction_body.length() - 1.0).abs() > 1.0e-9
        {
            return None;
        }
        let mut widest_rad: f64 = 0.0;
        for part in &self.parts {
            let offset = receiver_body_m - part.local_position_m;
            if part
                .shape
                .contains_point_local(part.local_orientation.conjugate() * offset)
            {
                return Some((sun_direction_body, std::f64::consts::PI));
            }
            let origin_local = part.local_orientation.conjugate() * offset;
            let dir_local = part.local_orientation.conjugate() * sun_direction_body;
            if part.shape.ray_hit_t(origin_local, dir_local).is_some() {
                let distance = offset.length();
                let radius = part.shape.bounding_radius_m();
                if radius > 0.0 && distance > 0.0 {
                    widest_rad = widest_rad.max((radius / distance).min(1.0).asin());
                }
            }
        }
        if widest_rad > 0.0 {
            Some((sun_direction_body, widest_rad))
        } else {
            None
        }
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
            Self::InvalidMaterial(message) => {
                write!(formatter, "invalid collision material: {message}")
            }
            Self::InvalidShape(message) => write!(formatter, "invalid collision shape: {message}"),
            Self::InvalidPose(message) => write!(formatter, "invalid collision pose: {message}"),
        }
    }
}

impl Error for CollisionError {}

/// Closest ray entry distance, or `None` on miss. Origin inside the shape
/// is a miss here (callers test containment separately); hits at or behind
/// the origin are ignored so surface-mounted receivers stay lit.
fn ray_sphere_t(origin: DVec3, direction: DVec3, radius_m: f64) -> Option<f64> {
    let a = direction.length_squared();
    if a <= 0.0 {
        return None;
    }
    let half_b = origin.dot(direction);
    let c = origin.length_squared() - radius_m * radius_m;
    let disc = half_b * half_b - a * c;
    if disc <= 0.0 {
        return None;
    }
    let root = disc.sqrt();
    let t = (-half_b - root) / a;
    if t > RAY_HIT_EPS_M { Some(t) } else { None }
}

fn ray_cuboid_t(origin: DVec3, direction: DVec3, half_extents: DVec3) -> Option<f64> {
    let mut t_enter = 0.0_f64;
    for axis in 0..3 {
        let origin_a = origin[axis];
        let dir_a = direction[axis];
        let half = half_extents[axis];
        if dir_a.abs() < 1.0e-12 {
            if origin_a.abs() > half {
                return None;
            }
        } else {
            let mut t0 = (-half - origin_a) / dir_a;
            let mut t1 = (half - origin_a) / dir_a;
            if t0 > t1 {
                std::mem::swap(&mut t0, &mut t1);
            }
            t_enter = t_enter.max(t0);
            if t_enter > t1 {
                return None;
            }
        }
    }
    if t_enter > RAY_HIT_EPS_M {
        Some(t_enter)
    } else {
        None
    }
}

fn ray_capsule_t(
    origin: DVec3,
    direction: DVec3,
    axis: DVec3,
    half_segment_m: f64,
    radius_m: f64,
) -> Option<f64> {
    let mut best: Option<f64> = None;
    // Cylindrical wall.
    let dir_parallel = axis * direction.dot(axis);
    let dir_perp = direction - dir_parallel;
    if dir_perp.length_squared() > 1.0e-24 {
        let origin_perp = origin - axis * origin.dot(axis);
        let a = dir_perp.length_squared();
        let half_b = origin_perp.dot(dir_perp);
        let c = origin_perp.length_squared() - radius_m * radius_m;
        let disc = half_b * half_b - a * c;
        if disc > 0.0 {
            let root = disc.sqrt();
            for t in [(-half_b - root) / a, (-half_b + root) / a] {
                if t > RAY_HIT_EPS_M {
                    let along = (origin + direction * t).dot(axis);
                    if along.abs() <= half_segment_m && best.is_none_or(|current| t < current) {
                        best = Some(t);
                    }
                }
            }
        }
    }
    // Spherical caps.
    for cap in [axis * half_segment_m, axis * -half_segment_m] {
        if let Some(t) = ray_sphere_t(origin - cap, direction, radius_m)
            && best.is_none_or(|current| t < current)
        {
            best = Some(t);
        }
    }
    best
}

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

    fn hull_box() -> CollisionGeometry {
        CollisionGeometry::new(vec![
            CollisionPart::new(
                DVec3::ZERO,
                DQuat::IDENTITY,
                CollisionShape::Cuboid {
                    half_extents_m: DVec3::new(1.0, 1.0, 2.0),
                },
                CollisionMaterial::default(),
            )
            .expect("valid hull part"),
        ])
        .expect("valid hull")
    }

    #[test]
    fn hull_blocks_sun_behind_it_and_passes_sun_in_front() {
        let hull = hull_box();
        // Panel 5 m above the hull, sun straight up: clear.
        assert!(
            hull.own_body_occluder(DVec3::new(0.0, 0.0, 5.0), DVec3::Z)
                .is_none()
        );
        // Same panel, sun straight down behind the hull: blocked.
        let blocked = hull
            .own_body_occluder(DVec3::new(0.0, 0.0, 5.0), -DVec3::Z)
            .expect("hull must block");
        assert_eq!(blocked.0, -DVec3::Z);
        // Bounding sphere radius sqrt(6) at 5 m: asin(sqrt(6)/5).
        assert!((blocked.1 - (6.0_f64.sqrt() / 5.0).asin()).abs() < 1.0e-12);
        // Side sun missing the hull: clear.
        assert!(
            hull.own_body_occluder(DVec3::new(0.0, 0.0, 5.0), DVec3::X)
                .is_none()
        );
    }

    #[test]
    fn receiver_inside_hull_means_full_sky_blockage() {
        let hull = hull_box();
        let full = hull
            .own_body_occluder(DVec3::new(0.0, 0.0, 0.5), DVec3::Y)
            .expect("inside means blocked");
        assert!((full.1 - std::f64::consts::PI).abs() < 1.0e-12);
    }

    #[test]
    fn strut_ray_hit_uses_true_shape_not_just_spheres() {
        // Thin pole along Z at x = 10; a ray along +X offset 3 m to the
        // side grazes past it (closest approach 3 m >> radius 0.1 and past
        // the 2.1 m bounding sphere): no hit, no shadow.
        let pole = CollisionGeometry::new(vec![
            CollisionPart::new(
                DVec3::new(10.0, 0.0, 0.0),
                DQuat::IDENTITY,
                CollisionShape::Capsule {
                    axis: CollisionAxis::Z,
                    half_segment_m: 2.0,
                    radius_m: 0.1,
                },
                CollisionMaterial::default(),
            )
            .expect("valid pole"),
        ])
        .expect("valid geometry");
        assert!(
            pole.own_body_occluder(DVec3::new(5.0, 3.0, 0.0), DVec3::X)
                .is_none()
        );
        // Ray straight down the pole axis from above hits the cap.
        assert!(
            pole.own_body_occluder(DVec3::new(10.0, 0.0, 5.0), -DVec3::Z)
                .is_some()
        );
    }

    #[test]
    fn bad_occlusion_inputs_claim_no_shadow() {
        let hull = hull_box();
        assert!(
            hull.own_body_occluder(DVec3::new(0.0, 0.0, 5.0), DVec3::ZERO)
                .is_none()
        );
        assert!(
            hull.own_body_occluder(DVec3::new(0.0, 0.0, 5.0), DVec3::new(2.0, 0.0, 0.0))
                .is_none()
        );
        assert!(
            hull.own_body_occluder(DVec3::new(f64::NAN, 0.0, 5.0), DVec3::Z)
                .is_none()
        );
    }
}
