//! Shared instrument math and the legacy CPU128 navball renderer. The Bevy
//! pilot uses CPU pixels; the native HUD consumes the surface instrument frame
//! with its own GPU renderer. No clocks, ECS, images or renderer handles live
//! here; each host owns presentation and any texture upload/rebuild gating.
//!
//! The renderer is total: a degenerate local-up falls back to +Z, so hosts
//! never receive NaNs. Pixel output for a given input is bit-deterministic.
use glam::DVec3;

/// Backend-neutral surface-referenced instrument frame. Vehicle axes are nose
/// +X, screen-right -Y, screen-up +Z. Heading is explicitly unavailable at a
/// pole or when the nose is vertical; a deterministic tangent keeps the ball
/// finite without inventing a compass measurement.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InstrumentFrame {
    pub up: DVec3,
    pub north: DVec3,
    pub east: DVec3,
    pub prograde: Option<DVec3>,
    pub heading_rad: Option<f64>,
}
impl InstrumentFrame {
    pub fn new(up: DVec3, pole: DVec3, velocity: DVec3) -> Self {
        let measured_up = up.try_normalize();
        let up = measured_up.unwrap_or(DVec3::Z);
        let pole = pole.try_normalize().unwrap_or(DVec3::ZERO);
        let tangent = pole - up * pole.dot(up);
        let north = (tangent.length_squared() > 1e-12).then(|| tangent.normalize());
        let compass_valid = measured_up.is_some() && north.is_some();
        let helper = if up.x.abs() < 0.9 { DVec3::X } else { DVec3::Y };
        let north = north.unwrap_or_else(|| (helper - up * helper.dot(up)).normalize());
        let east = north.cross(up).normalize();
        let heading_rad = (compass_valid && up.x.abs() < 1.0 - 1e-12)
            .then(|| east.x.atan2(north.x).rem_euclid(std::f64::consts::TAU));
        Self {
            up,
            north,
            east,
            prograde: velocity.try_normalize(),
            heading_rad,
        }
    }

    /// Orthographic front hemisphere, in disk coordinates (screen Y down).
    /// A hidden vector is omitted, not incorrectly reflected onto the front.
    pub fn project(direction: DVec3) -> Option<[f32; 2]> {
        let direction = direction.try_normalize()?;
        (direction.x >= 0.0).then_some([-direction.y as f32, -direction.z as f32])
    }
}

/// Instrument texture edge in pixels, inherited from the shipped pilot HUD.
pub const NAVBALL_TEXTURE_SIZE: u32 = 128;

/// Minimum squared local-up travel (in unit-body components) before a host
/// should rebuild the texture. Mirrors the shipped 30 Hz/hysteresis policy
/// that avoids rebuilds for sub-pixel attitude changes.
pub const REBUILD_TRAVEL_SQUARED: f64 = 4.0e-6;
/// Maximum instrument refresh rate in hertz, as in the shipped HUD.
pub const MAX_REFRESH_HZ: f64 = 30.0;

pub fn navball_pixels(local_up_body: DVec3) -> Vec<u8> {
    const SIZE: u32 = NAVBALL_TEXTURE_SIZE;
    let center = SIZE as f64 * 0.5;
    let radius = center - 1.0;
    let local_up = local_up_body.try_normalize().unwrap_or(DVec3::Z);
    let mut pixels = Vec::with_capacity((SIZE * SIZE * 4) as usize);

    for y in 0..SIZE {
        for x in 0..SIZE {
            let dx = x as f64 + 0.5 - center;
            let dy = y as f64 + 0.5 - center;
            let distance = dx.hypot(dy);
            if distance > radius {
                pixels.extend_from_slice(&[0, 0, 0, 0]);
                continue;
            }

            let normalized_x = dx / radius;
            let normalized_y = dy / radius;
            // The navball is a view of a unit sphere from the vehicle nose
            // (+X). Screen-right is forward x up = -Y; screen-up is +Z. This
            // projection makes the horizon respond to all three attitude
            // axes, including roll, instead of painting a flat semicircle.
            let sphere_depth = (1.0 - normalized_x * normalized_x - normalized_y * normalized_y)
                .max(0.0)
                .sqrt();
            let sphere_direction =
                DVec3::new(sphere_depth, -normalized_x, -normalized_y).normalize();
            let horizon = sphere_direction.dot(local_up);
            let sky_weight = ((horizon + 0.035) / 0.070).clamp(0.0, 1.0);
            let light_direction = DVec3::new(0.42, -0.38, 0.82).normalize();
            let light = (sphere_direction.dot(light_direction) * 0.5 + 0.5).clamp(0.0, 1.0);
            let edge_shade = 0.70 + 0.30 * light;
            let horizon_glow = (1.0 - horizon.abs() * 15.0).clamp(0.0, 1.0);
            let sky = DVec3::new(0.08, 0.52, 0.72);
            let ground = DVec3::new(0.60, 0.32, 0.095);
            let mut colour = ground.lerp(sky, sky_weight);

            // KSP-style pitch ladder: lines are contours on the sphere in the
            // current local-up frame. They curve naturally near the rim and
            // remain stable when the aircraft rolls or pitches.
            for pitch_deg in [
                -60.0_f64, -45.0, -30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0, 45.0, 60.0,
            ] {
                let pitch_level = pitch_deg.to_radians().sin();
                let distance_to_line = (horizon - pitch_level).abs();
                let line_width = if pitch_deg == 0.0 { 0.018 } else { 0.012 };
                if distance_to_line < line_width {
                    let line_strength = 1.0 - distance_to_line / line_width;
                    colour = colour.lerp(DVec3::splat(0.82), line_strength * 0.78);
                }
            }
            colour = colour.lerp(DVec3::new(0.95, 0.62, 0.16), horizon_glow * 0.12);
            let edge_alpha = ((radius - distance) * 5.0).clamp(0.0, 1.0);
            pixels.extend_from_slice(&[
                (colour.x * edge_shade * 255.0).clamp(0.0, 255.0) as u8,
                (colour.y * edge_shade * 255.0).clamp(0.0, 255.0) as u8,
                (colour.z * edge_shade * 255.0).clamp(0.0, 255.0) as u8,
                (edge_alpha * 255.0).clamp(0.0, 255.0) as u8,
            ]);
        }
    }

    pixels
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn instrument_frame_tracks_heading_all_axes_and_explicit_degeneracies() {
        let level = InstrumentFrame::new(DVec3::Z, DVec3::X, DVec3::X * 200.0);
        assert_eq!(level.north, DVec3::X);
        assert_eq!(level.east, -DVec3::Y);
        assert_eq!(level.heading_rad, Some(0.0));
        assert_eq!(InstrumentFrame::project(DVec3::X), Some([0.0, 0.0]));
        assert_eq!(InstrumentFrame::project(-DVec3::Y), Some([1.0, 0.0]));
        assert!(InstrumentFrame::project(-DVec3::X).is_none());
        assert!(InstrumentFrame::project(DVec3::NAN).is_none());
        let east = InstrumentFrame::new(DVec3::Z, DVec3::Y, DVec3::ZERO);
        assert!((east.heading_rad.unwrap() - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
        assert_eq!(east.prograde, None);
        for axis in [DVec3::X, DVec3::Y, DVec3::Z] {
            for angle in [-3.0, -1.0, 0.0, 0.7, 3.1] {
                let q = glam::DQuat::from_axis_angle(axis, angle);
                let frame = InstrumentFrame::new(q * DVec3::Z, q * DVec3::X, q * DVec3::X);
                assert!(frame.north.distance(q * DVec3::X) < 1e-12);
                assert!(frame.east.distance(q * (-DVec3::Y)) < 1e-12);
                assert!(frame.up.dot(frame.north).abs() < 1e-12);
            }
        }
        for up in [DVec3::X, DVec3::ZERO, DVec3::NAN, DVec3::INFINITY] {
            let frame = InstrumentFrame::new(up, up, DVec3::NAN);
            assert!(frame.up.is_finite() && frame.north.is_finite() && frame.east.is_finite());
            assert!(frame.heading_rad.is_none());
            assert!(frame.prograde.is_none());
        }
        for invalid in [DVec3::ZERO, DVec3::NAN, DVec3::INFINITY] {
            assert!(
                InstrumentFrame::new(invalid, DVec3::X, DVec3::ZERO)
                    .heading_rad
                    .is_none()
            );
            assert!(
                InstrumentFrame::new(DVec3::Z, invalid, DVec3::ZERO)
                    .heading_rad
                    .is_none()
            );
        }
        assert_eq!(
            InstrumentFrame::new(DVec3::Z, DVec3::X * 100.0, DVec3::ZERO).heading_rad,
            Some(0.0)
        );
    }

    #[test]
    fn navball_is_a_dynamic_deterministic_sphere_projection() {
        let level = navball_pixels(DVec3::Z);
        let pitched = navball_pixels(DVec3::X);
        let size = NAVBALL_TEXTURE_SIZE as usize;
        assert_eq!(level.len(), size * size * 4);
        assert_ne!(level, pitched);
        assert_eq!(level, navball_pixels(DVec3::Z));
        // The projected disk keeps transparent corners, so it remains a
        // sphere-shaped instrument when rendered inside the circular frame.
        assert_eq!(&level[0..4], &[0, 0, 0, 0]);
        // Degenerate input falls back instead of producing NaNs.
        for up in [DVec3::ZERO, DVec3::NAN, DVec3::INFINITY] {
            assert_eq!(navball_pixels(up), level);
        }
    }
}
