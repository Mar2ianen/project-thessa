//! Camera-relative, texel-stable directional shadow cover. Only presentation;
//! callers supply the same f64 camera/light frame as their scene geometry.
use glam::{DMat4, DVec3};

pub fn cascade_radii(distance_m: f64, count: u32) -> Option<Vec<f64>> {
    if !(1..=4).contains(&count) || !distance_m.is_finite() || distance_m <= 0.0 {
        return None;
    }
    Some(
        (0..count)
            .map(|i| distance_m * 40.0_f64.powf(f64::from(i + 1) / f64::from(count) - 1.0))
            .collect(),
    )
}

/// Project camera-relative points into WebGPU [0,1] depth. The xy cover is
/// snapped in f64; casters up to `caster_reach_m` toward the light are included.
pub fn directional_clip(
    camera_m: DVec3,
    toward_light: DVec3,
    radius_m: f64,
    caster_reach_m: f64,
    size: u32,
) -> Option<DMat4> {
    if !camera_m.is_finite()
        || !toward_light.is_finite()
        || (toward_light.length_squared() - 1.0).abs() > 1e-4
        || size == 0
        || !radius_m.is_finite()
        || radius_m <= 0.0
        || !caster_reach_m.is_finite()
        || caster_reach_m < radius_m
    {
        return None;
    }
    let sun = toward_light.normalize();
    let up = if sun.dot(DVec3::Y).abs() < 0.95 {
        DVec3::Y
    } else {
        DVec3::X
    };
    let right = up.cross(sun).normalize();
    let vertical = sun.cross(right);
    // Leave one texel on either side for snapping and filter support.
    let extent = radius_m * (1.0 + 4.0 / f64::from(size));
    let texel = 2.0 * extent / f64::from(size);
    let residual = |axis: DVec3| {
        let v = camera_m.dot(axis);
        v - (v / texel).round() * texel
    };
    let center = -right * residual(right) - vertical * residual(vertical);
    let depth = caster_reach_m + extent;
    // glam's `directx` name denotes the standard [0,1] projection convention
    // also used by WebGPU/Vulkan; no backend API or DX dependency is involved.
    Some(
        glam::dcamera::rh::proj::directx::orthographic(
            -extent,
            extent,
            -extent,
            extent,
            0.0,
            2.0 * depth,
        ) * glam::dcamera::rh::view::look_at_mat4(center + sun * depth, center, vertical),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn cover_rejects_invalid_inputs_and_bounds_narrowing_at_astronomical_origins() {
        assert!(cascade_radii(f64::NAN, 4).is_none());
        assert!(cascade_radii(10.0, 0).is_none());
        assert_eq!(cascade_radii(12000.0, 4).unwrap().last(), Some(&12000.0));
        let camera = DVec3::new(3e12, -2e12, 1e12);
        let sun = DVec3::new(0.2, 0.7, 0.4).normalize();
        let clip = directional_clip(camera, sun, 300.0, 12000.0, 1024).unwrap();
        for p in [
            DVec3::ZERO,
            DVec3::X * 299.0,
            DVec3::Y * 299.0,
            DVec3::Z * 299.0,
        ] {
            let q = clip.project_point3(p);
            assert!(q.x.abs() < 1.0 && q.y.abs() < 1.0 && (0.0..1.0).contains(&q.z));
            let narrow = clip.as_mat4().project_point3(p.as_vec3()).as_dvec3();
            assert!(
                (q - narrow).length() * 24000.0 < 0.003,
                "<3mm projection-space error over a 24km depth interval"
            );
        }
        assert!(directional_clip(camera, DVec3::ZERO, 1.0, 2.0, 512).is_none());
    }
}
