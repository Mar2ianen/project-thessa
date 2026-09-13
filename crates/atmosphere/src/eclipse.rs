//! Geometric eclipse visibility: continuous source factor, not a brightness hack.
//!
//! Given light direction/angular size and occluder direction/angular size,
//! derive `1.0 = unobscured, 0.0 = fully eclipsed, 0..1 = penumbra`
//! (spec section 9). The factor feeds direct lighting, sky scattering and
//! cloud lighting together — never surface-only darkening.

use glam::DVec3;

/// Fraction of the stellar disk remaining visible behind a circular occluder.
///
/// Inputs are unit directions from the receiver and angular radii in radians.
/// Uses exact circle-circle overlap area; smooth and continuous everywhere,
/// including the penumbra band. Returns a value in `0..=1`.
pub fn eclipse_visibility(
    light_dir: DVec3,
    light_angular_radius_rad: f64,
    occluder_dir: DVec3,
    occluder_angular_radius_rad: f64,
) -> f64 {
    let rl = light_angular_radius_rad.max(0.0);
    let ro = occluder_angular_radius_rad.max(0.0);
    if rl <= 0.0 {
        return 1.0;
    }
    if ro <= 0.0 {
        return 1.0;
    }
    let cos_sep = light_dir
        .normalize_or_zero()
        .dot(occluder_dir.normalize_or_zero())
        .clamp(-1.0, 1.0);
    let d = cos_sep.acos();
    if d >= rl + ro {
        return 1.0;
    }
    if d <= (ro - rl).abs() {
        return if ro >= rl {
            0.0
        } else {
            1.0 - (ro * ro) / (rl * rl)
        };
    }
    // Partial overlap: visible fraction = 1 - overlap_area / light_area.
    let overlap = circle_overlap_area(rl, ro, d);
    (1.0 - overlap / (std::f64::consts::PI * rl * rl)).clamp(0.0, 1.0)
}

/// Area of intersection of two circles with radii `r0`, `r1` whose centers
/// are `d` apart. Standard lens formula, numerically clamped.
fn circle_overlap_area(r0: f64, r1: f64, d: f64) -> f64 {
    let d = d.max(1e-12);
    let arg0 = ((d * d + r0 * r0 - r1 * r1) / (2.0 * d * r0)).clamp(-1.0, 1.0);
    let arg1 = ((d * d + r1 * r1 - r0 * r0) / (2.0 * d * r1)).clamp(-1.0, 1.0);
    let t0 = r0 * r0 * arg0.acos();
    let t1 = r1 * r1 * arg1.acos();
    let t2 = ((-d + r0 + r1).max(0.0)
        * (d + r0 - r1).max(0.0)
        * (d - r0 + r1).max(0.0)
        * (d + r0 + r1).max(0.0))
    .sqrt();
    // Classic lens form: r0²acos + r1²acos − ½√((−d+r0+r1)(d+r0−r1)(d−r0+r1)(d+r0+r1)).
    (t0 + t1 - 0.5 * t2).max(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    #[test]
    fn full_and_empty_eclipse() {
        let full = eclipse_visibility(DVec3::Z, 0.01, DVec3::Z, 0.02);
        assert!((full - 0.0).abs() < 1e-12);
        let none = eclipse_visibility(DVec3::Z, 0.01, DVec3::X, 0.02);
        assert!((none - 1.0).abs() < 1e-12);
    }

    #[test]
    fn half_overlap_is_about_half() {
        // Equal disks whose centers are one radius apart overlap ~39%; the
        // visible fraction is ~0.61. Check the band, not an exact constant.
        let v = eclipse_visibility(DVec3::Z, 0.01, DVec3::new(0.01, 0.0, 1.0).normalize(), 0.01);
        assert!((0.3..=0.9).contains(&v), "half overlap gave {v}");
    }

    #[test]
    fn penumbra_is_continuous() {
        let rl = 0.01;
        let ro = 0.05;
        // Sweep from clear sky into totality: start outside the penumbra.
        let mut prev = 1.0;
        let mut crossings = 0;
        for i in 0..=200 {
            let angle = (rl + ro) - (i as f64 / 200.0) * (rl + ro);
            let dir = DVec3::new(angle.sin(), 0.0, angle.cos());
            let v = eclipse_visibility(DVec3::Z, rl, dir, ro);
            assert!((0.0..=1.0).contains(&v));
            assert!((v - prev).abs() < 0.05, "jump at step {i}: {prev} -> {v}");
            if v < 1.0 {
                crossings += 1;
            }
            prev = v;
        }
        assert!(crossings > 0, "penumbra band must be crossed");
    }

    #[test]
    fn small_occluder_gives_annular_fraction() {
        // Tiny occluder centered on a large disk blocks (ro/rl)^2.
        let v = eclipse_visibility(DVec3::Z, 0.02, DVec3::Z, 0.01);
        assert!((v - 0.75).abs() < 1e-9, "annular transit gave {v}");
        let _ = FRAC_PI_2;
    }
}
