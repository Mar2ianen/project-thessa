//! Existing navball instrument renderer, shared by the Bevy pilot HUD and the
//! native migration viewer. Pure CPU pixels only: a 128 px RGBA sphere view
//! from the vehicle nose (+X) with a KSP-style pitch ladder in the local-up
//! frame. No clocks, throttling policy, ECS, Bevy images or renderer
//! handles; each host owns texture upload and rebuild gating.
//!
//! The renderer is total: a degenerate local-up falls back to +Z, so hosts
//! never receive NaNs. Pixel output for a given input is bit-deterministic.
use glam::DVec3;

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
