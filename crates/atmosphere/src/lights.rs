//! Celestial illumination: multi-star light list consumed by every backend.
//!
//! The renderer must consume a list of sources rather than assume one Sun
//! (spec section 8): Asterion A (warm K main), B (hotter, bluer, weaker at
//! Thessa), C (thermally minor, visually present).

use glam::DVec3;
use serde::{Deserialize, Serialize};

/// One stellar source in planet-centric inertial frame.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CelestialLight {
    /// Unit vector from the receiver toward the star.
    pub direction_to_star: DVec3,
    /// Broadband irradiance at the receiver (W/m^2).
    pub irradiance_w_m2: f64,
    /// Linear RGB tint derived from effective temperature.
    pub color_rgb: [f64; 3],
    pub angular_radius_rad: f64,
    /// Eclipse/occlusion factor: 1.0 unobscured, 0.0 fully eclipsed.
    pub visibility: f64,
}

impl CelestialLight {
    /// Weighted RGB irradiance used by shaders and CPU queries.
    pub fn weighted_rgb(&self) -> [f64; 3] {
        let k = (self.irradiance_w_m2 * self.visibility.clamp(0.0, 1.0)).max(0.0);
        [
            self.color_rgb[0] * k,
            self.color_rgb[1] * k,
            self.color_rgb[2] * k,
        ]
    }
}

/// Inverse-square irradiance from luminosity and distance.
pub fn irradiance_at_distance(luminosity_w: f64, distance_m: f64) -> f64 {
    if !luminosity_w.is_finite()
        || !distance_m.is_finite()
        || luminosity_w <= 0.0
        || distance_m <= 0.0
    {
        return 0.0;
    }
    luminosity_w / (4.0 * std::f64::consts::PI * distance_m * distance_m)
}

/// Small-angle angular radius of a body, guarded against degenerate input.
pub fn angular_radius(body_radius_m: f64, distance_m: f64) -> f64 {
    if !body_radius_m.is_finite()
        || !distance_m.is_finite()
        || body_radius_m <= 0.0
        || distance_m <= body_radius_m
    {
        return 0.0;
    }
    (body_radius_m / distance_m).asin().max(0.0)
}

/// Approximate linear-RGB blackbody tint for 1000..=15000 K.
///
/// Uses the Tanner Helland polynomial approximation (green/blue channels)
/// with a warm-red ramp below 6600 K. Good enough for star tints; not a
/// spectroscopy reference.
pub fn blackbody_rgb(temperature_k: f64) -> [f64; 3] {
    let t = (temperature_k / 100.0).clamp(10.0, 150.0);
    let r = if t <= 66.0 {
        1.0
    } else {
        (1.292_756_058 * (t - 60.0).powf(-0.133_204_759_2)).clamp(0.0, 1.0)
    };
    let g = if t <= 66.0 {
        (0.390_081_578_8 * t.ln() - 0.631_841_443_7).clamp(0.0, 1.0)
    } else {
        (1.129_890_860_9 * (t - 60.0).powf(-0.075_514_849_2)).clamp(0.0, 1.0)
    };
    let b = if t >= 66.0 {
        1.0
    } else if t <= 19.0 {
        0.0
    } else {
        (0.543_206_789_1 * (t - 10.0).ln() - 1.196_254_089_1).clamp(0.0, 1.0)
    };
    // Normalize so tint carries colour, not brightness (brightness comes
    // from irradiance).
    let max = r.max(g).max(b).max(1e-6);
    [r / max, g / max, b / max]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn asterion_colors_follow_temperature() {
        let warm = blackbody_rgb(5100.0);
        let hot = blackbody_rgb(8700.0);
        assert!(warm[0] >= warm[2], "5100 K must be warm-tinted");
        assert!(hot[2] >= hot[0], "8700 K must be blue-tinted");
        for c in warm.into_iter().chain(hot) {
            assert!((0.0..=1.0).contains(&c));
        }
    }

    #[test]
    fn irradiance_falls_with_distance_squared() {
        let near = irradiance_at_distance(1.0e26, 1.0e11);
        let far = irradiance_at_distance(1.0e26, 2.0e11);
        assert!((near / far - 4.0).abs() < 1e-9);
        assert_eq!(irradiance_at_distance(1.0, 0.0), 0.0);
    }

    #[test]
    fn visibility_weights_irradiance() {
        let light = CelestialLight {
            direction_to_star: DVec3::Z,
            irradiance_w_m2: 1000.0,
            color_rgb: [1.0, 1.0, 1.0],
            angular_radius_rad: 0.01,
            visibility: 0.5,
        };
        assert_eq!(light.weighted_rgb(), [500.0, 500.0, 500.0]);
    }
}
