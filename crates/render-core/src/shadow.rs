//! Camera-relative, texel-stable directional shadow cover. Only presentation;
//! callers supply the same f64 camera/light frame as their scene geometry.
use glam::{DMat4, DVec3};

pub fn cascade_radii(distance_m: f64, count: u32) -> Option<Vec<f64>> {
    cascade_radii_with_first(distance_m, count, None)
}

/// Optional explicit near cover with the same cascade count/far endpoint.
/// Omission preserves the original distribution byte-for-byte.
pub fn cascade_radii_with_first(
    distance_m: f64,
    count: u32,
    first_m: Option<f64>,
) -> Option<Vec<f64>> {
    if !(1..=4).contains(&count) || !distance_m.is_finite() || distance_m <= 0.0 {
        return None;
    }
    if let Some(first) = first_m {
        if !first.is_finite() || first <= 0.0 || first > distance_m {
            return None;
        }
        return Some(
            (0..count)
                .map(|i| {
                    if i + 1 == count {
                        distance_m
                    } else {
                        first * (distance_m / first).powf(f64::from(i) / f64::from(count - 1))
                    }
                })
                .collect(),
        );
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

/// Conservative homogeneous clip-plane test for a camera-relative sphere.
/// Uses the actual narrowed GPU projection (promoted to f64 by the caller).
/// Invalid or numerically unbounded inputs fail open. A small f32 dot-product
/// envelope retains boundary casters; it is not a geometric/quality tolerance.
pub fn sphere_intersects_clip(clip: DMat4, center: DVec3, radius: f64) -> bool {
    if !clip.is_finite() || !center.is_finite() || !radius.is_finite() || radius < 0.0 {
        return true;
    }
    let w = clip.row(3);
    for plane in [
        w + clip.row(0),
        w - clip.row(0),
        w + clip.row(1),
        w - clip.row(1),
        clip.row(2),
        w - clip.row(2),
    ] {
        let n = plane.truncate();
        let support = radius * n.length();
        let distance = n.dot(center) + plane.w;
        let error =
            16.0 * f64::from(f32::EPSILON) * (n.abs().dot(center.abs()) + plane.w.abs() + support);
        if distance + support < -error {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn explicit_near_cover_preserves_count_far_endpoint_and_projection_bounds() {
        for count in 1..=4 {
            let legacy = cascade_radii(12000.0, count).unwrap();
            assert_eq!(
                cascade_radii_with_first(12000.0, count, None).unwrap(),
                legacy
            );
            for first in [1.0, 25.0, 300.0, 12000.0] {
                let radii = cascade_radii_with_first(12000.0, count, Some(first)).unwrap();
                assert_eq!(radii.len(), count as usize);
                assert_eq!(*radii.last().unwrap(), 12000.0);
                if count > 1 {
                    assert_eq!(radii[0], first);
                }
                assert!(radii.windows(2).all(|r| r[0] <= r[1]));
                for radius in radii {
                    let clip = directional_clip(
                        DVec3::new(3e12, -2e12, 1e12),
                        DVec3::new(0.2, 0.7, 0.4).normalize(),
                        radius,
                        12000.0,
                        1024,
                    )
                    .unwrap();
                    for p in [
                        DVec3::ZERO,
                        DVec3::X * radius,
                        DVec3::Y * radius,
                        DVec3::Z * radius,
                    ] {
                        let q = clip.project_point3(p);
                        assert!(q.x.abs() < 1.0 && q.y.abs() < 1.0 && (0.0..1.0).contains(&q.z));
                        let narrow = clip.as_mat4().project_point3(p.as_vec3()).as_dvec3();
                        assert!(
                            (q - narrow).length() * 48000.0 < 0.01,
                            "<1cm conservative clip-space narrowing envelope"
                        );
                    }
                }
            }
        }
        for bad in [f64::NAN, f64::INFINITY, 0.0, -1.0, 12001.0] {
            assert!(cascade_radii_with_first(12000.0, 4, Some(bad)).is_none());
        }
        let radii = cascade_radii_with_first(12000.0, 4, Some(25.0)).unwrap();
        assert!((radii[1] / radii[0] - radii[2] / radii[1]).abs() < 1e-12);
        let old = cascade_radii(12000.0, 4).unwrap()[0];
        assert!(
            old / radii[0] > 30.0,
            "same texel count, over 30x finer first cover"
        );
    }
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

    #[test]
    fn sphere_planes_retain_contacts_off_camera_casters_and_invalid_input() {
        for center in [
            DVec3::ZERO,
            DVec3::X,
            DVec3::Y,
            DVec3::Z,
            DVec3::new(-1.0, -1.0, 0.0),
        ] {
            assert!(sphere_intersects_clip(DMat4::IDENTITY, center, 0.0));
        }
        assert!(!sphere_intersects_clip(
            DMat4::IDENTITY,
            DVec3::new(0.0, 0.0, -0.1),
            0.09
        ));
        assert!(sphere_intersects_clip(
            DMat4::IDENTITY,
            DVec3::new(0.0, 0.0, -0.1),
            0.1
        ));
        assert!(sphere_intersects_clip(DMat4::NAN, DVec3::ZERO, 1.0));
        assert!(sphere_intersects_clip(DMat4::IDENTITY, DVec3::NAN, 1.0));
        for camera in [DVec3::ZERO, DVec3::new(3e12, -2e12, 1e12)] {
            let sun = DVec3::new(0.2, 0.7, 0.4).normalize();
            let clip = directional_clip(camera, sun, 20.0, 200.0, 256)
                .unwrap()
                .as_mat4()
                .as_dmat4();
            // Outside a forward camera frustum is irrelevant: only the finite
            // light-view volume may reject a caster that is behind the viewer.
            assert!(sphere_intersects_clip(clip, sun * 150.0, 2.0));
            assert!(!sphere_intersects_clip(clip, sun * 500.0, 2.0));
            let inverse = clip.inverse();
            for axis in 0..3 {
                for sign in [-1.0, 1.0] {
                    let mut q = DVec3::new(0.0, 0.0, 0.5);
                    q[axis] = if axis == 2 { (sign + 1.0) / 2.0 } else { sign };
                    assert!(sphere_intersects_clip(
                        clip,
                        inverse.project_point3(q),
                        0.001
                    ));
                }
            }
        }
    }
}

/// Conservative footprint for filled affine triangles in a centered, single-sample
/// raster grid. The consumer supplies the actual narrowed shader matrix and a
/// guaranteed per-axis raster vertex snapping envelope. Unknown precision must
/// disable this optimization; invalid geometry always fails open.
#[derive(Debug, Clone, Copy)]
pub struct RasterFootprint {
    rows: [glam::DVec4; 2],
    row_norms: glam::DVec2,
    scale: f64,
    pixels: f64,
    size: u32,
    snap_guard: f64,
}
impl RasterFootprint {
    pub fn new(light: DMat4, size: u32, snap_guard: f64) -> Option<Self> {
        if !light.is_finite()
            || light.row(3) != glam::DVec4::W
            || size == 0
            || !snap_guard.is_finite()
            || !(0.0..0.5).contains(&snap_guard)
        {
            return None;
        }
        let rows = [light.row(0), light.row(1)];
        let x = rows[0].truncate();
        let y = rows[1].truncate();
        // Gershgorin bounds the largest singular value of the XY projection.
        let scale = (x.length_squared().max(y.length_squared()) + x.dot(y).abs()).sqrt()
            * (1.0 + 128.0 * f64::EPSILON);
        Some(Self {
            rows,
            row_norms: glam::DVec2::new(x.length(), y.length()),
            scale,
            pixels: f64::from(size) * 0.5,
            size,
            snap_guard,
        })
    }

    pub fn intersects_sphere(&self, center: DVec3, radius: f64) -> bool {
        if !center.is_finite() || !radius.is_finite() || radius < 0.0 {
            return true;
        }
        let radius_px = radius * self.scale * self.pixels;
        // Every point inside a grid cell is within this radius of a sample.
        // Outside the atlas this is merely a conservative early accept.
        if !radius_px.is_finite() || radius_px >= std::f64::consts::FRAC_1_SQRT_2 {
            return true;
        }
        let mut distance_squared = 0.0;
        for axis in 0..2 {
            let row = self.rows[axis];
            let pixel = (row.truncate().dot(center) + row.w + 1.0) * self.pixels;
            let arithmetic = 32.0
                * f64::from(f32::EPSILON)
                * (row.truncate().abs().dot(center.abs())
                    + row.w.abs()
                    + self.row_norms[axis] * radius
                    + 1.0)
                * self.pixels;
            if !pixel.is_finite() || !arithmetic.is_finite() {
                return true;
            }
            let nearest = (pixel - 0.5).round().clamp(0.0, f64::from(self.size - 1)) + 0.5;
            let distance = (nearest - pixel).abs() - self.snap_guard - arithmetic;
            distance_squared += distance.max(0.0).powi(2);
        }
        distance_squared <= radius_px * radius_px
    }
}

#[cfg(test)]
mod raster_sample_tests {
    use super::*;

    #[test]
    fn footprint_retains_contacts_and_invalid_inputs_but_rejects_empty_cells() {
        let grid = RasterFootprint::new(DMat4::IDENTITY, 64, 1.0 / 16.0).unwrap();
        let point = |x, y| DVec3::new(x / 32.0 - 1.0, y / 32.0 - 1.0, 0.5);
        assert!(grid.intersects_sphere(point(10.5, 20.5), 0.0));
        assert!(!grid.intersects_sphere(point(10.0, 20.0), 0.0));
        assert!(!grid.intersects_sphere(point(10.0, 20.0), 0.1 / 32.0));
        assert!(grid.intersects_sphere(point(10.0, 20.0), 0.7 / 32.0));
        assert!(grid.intersects_sphere(point(10.5 - 1.0 / 16.0, 20.5), 0.0));
        assert!(grid.intersects_sphere(point(0.5, 63.5), 0.0));
        assert!(!grid.intersects_sphere(point(-1.0, 65.0), 0.0));
        assert!(grid.intersects_sphere(DVec3::NAN, 0.0));
        assert!(grid.intersects_sphere(DVec3::ZERO, f64::NAN));
        assert!(grid.intersects_sphere(DVec3::ZERO, -1.0));
        for bad in [DMat4::NAN, DMat4::ZERO, DMat4::from_cols_array(&[1.0; 16])] {
            assert!(RasterFootprint::new(bad, 64, 0.0625).is_none());
        }
        for bad in [f64::NAN, f64::INFINITY, -1.0, 0.5] {
            assert!(RasterFootprint::new(DMat4::IDENTITY, 64, bad).is_none());
        }
        assert!(RasterFootprint::new(DMat4::IDENTITY, 0, 0.0625).is_none());
    }

    #[test]
    fn footprint_never_loses_projected_samples_within_enclosed_spheres() {
        let mut checked = 0;
        for size in [512, 1024, 8192] {
            for radius in [25.0, 12000.0] {
                for origin in [DVec3::ZERO, DVec3::new(3e12, -2e12, 1e12)] {
                    for sun in [DVec3::Z, DVec3::Y, DVec3::new(0.2, 0.7, 0.4).normalize()] {
                        let clip = directional_clip(origin, sun, radius, 12000.0, size)
                            .unwrap()
                            .as_mat4()
                            .as_dmat4();
                        let grid = RasterFootprint::new(clip, size, 0.0625).unwrap();
                        for i in 0..100 {
                            let x = (i * 37 % size) as f64 + 0.5;
                            let y = (i * 59 % size) as f64 + 0.5;
                            let point = clip.inverse().project_point3(DVec3::new(
                                2.0 * x / f64::from(size) - 1.0,
                                2.0 * y / f64::from(size) - 1.0,
                                0.5,
                            ));
                            for r in [0.0, radius / f64::from(size) * 0.1] {
                                for axis in [DVec3::X, DVec3::Y, DVec3::Z, sun] {
                                    assert!(grid.intersects_sphere(point + axis * r, r));
                                    checked += 1;
                                }
                            }
                        }
                    }
                }
            }
        }
        assert_eq!(checked, 28800);
    }
}

/// A primary-receiver footprint extruded through the full light depth range.
/// Fixed-size planes keep per-frame construction allocation-free.
#[derive(Debug, Clone)]
pub struct ReceiverVolume {
    light: DMat4,
    planes: [glam::DVec4; 8],
    count: usize,
}
impl ReceiverVolume {
    pub fn intersects_sphere(&self, center: DVec3, radius: f64) -> bool {
        if !center.is_finite() || !radius.is_finite() || radius < 0.0 {
            return true;
        }
        if !sphere_intersects_clip(self.light, center, radius) {
            return false;
        }
        self.planes[..self.count].iter().all(|plane| {
            let n = plane.truncate();
            let support = radius * n.length();
            let distance = n.dot(center) + plane.w;
            let error = 16.0
                * f64::from(f32::EPSILON)
                * (n.abs().dot(center.abs()) + plane.w.abs() + support);
            !distance.is_finite()
                || !support.is_finite()
                || !error.is_finite()
                || distance + support >= -error
        })
    }
}

/// Tighten only the XY caster-selection volume to all primary receivers within
/// this cascade. Depth stays unchanged: receiver-plane PCF can compare depths
/// on either side of the centre sample at grazing incidence. `None` fails open.
/// Inputs are the actual narrowed shader matrices, promoted by the consumer.
/// `receiver_error_m` bounds primary clip/varying disagreement from source
/// vertex arithmetic, including vertices outside this cascade.
pub fn receiver_caster_volume(
    primary: DMat4,
    light: DMat4,
    radius_m: f64,
    normal_bias_m: f64,
    receiver_error_m: f64,
    map_size: u32,
    filter_radius: u32,
) -> Option<ReceiverVolume> {
    if !primary.is_finite()
        || !light.is_finite()
        || light.row(3) != glam::DVec4::W
        || !radius_m.is_finite()
        || radius_m <= 0.0
        || !normal_bias_m.is_finite()
        || normal_bias_m < 0.0
        || !receiver_error_m.is_finite()
        || receiver_error_m < 0.0
        || map_size == 0
        || filter_radius > 2
    {
        return None;
    }
    let normal_bias_m = f64::from(normal_bias_m as f32);
    if !normal_bias_m.is_finite() {
        return None;
    }
    // Source vertices may be much farther than the interpolated receiver;
    // the consumer bounds their clip/varying arithmetic separately.
    // Include f32 cascade-distance/interpolation arithmetic before finding the
    // receiver domain. Normal offset is applied separately in light XY below.
    let radius = radius_m + 32.0 * f64::from(f32::EPSILON) * (radius_m + 1.0);
    let rows = [
        primary.row(0),
        primary.row(1),
        primary.row(2),
        primary.row(3),
    ];
    let error = rows.map(|r| {
        32.0 * f64::from(f32::EPSILON)
            * (r.truncate().abs().element_sum() * radius + r.w.abs() + 1.0)
            + r.truncate().length() * receiver_error_m
    });
    let w = rows[3];
    let perspective = w.truncate().length_squared() > 0.0;
    let (depth_row, lo, hi) = if perspective {
        let alpha = rows[2].truncate().dot(w.truncate()) / w.truncate().length_squared();
        let beta = rows[2].w - alpha * w.w;
        let residual = (rows[2].truncate() - alpha * w.truncate()).length() * radius;
        let mut lo = -error[3];
        let mut hi = w.w + w.truncate().length() * radius;
        // Necessary depth bounds for z>=0 and w-z>=0; retain reverse-Z and
        // oblique projections by enclosing their residual over the radius.
        for (a, b) in [
            (alpha, -beta - residual - error[2]),
            (1.0 - alpha, beta - residual - error[2] - error[3]),
        ] {
            if a > 0.0 {
                lo = lo.max(b / a);
            } else if a < 0.0 {
                hi = hi.min(b / a);
            } else if b > 0.0 {
                return None;
            }
        }
        (w, lo, hi)
    } else {
        if w.w <= 0.0 {
            return None;
        }
        let z = rows[2];
        (
            z,
            (-error[2]).max(z.w - z.truncate().length() * radius),
            (w.w + error[2]).min(z.w + z.truncate().length() * radius),
        )
    };
    if !lo.is_finite() || !hi.is_finite() || lo > hi {
        return None;
    }
    let a = glam::DMat3::from_cols(rows[0].truncate(), rows[1].truncate(), depth_row.truncate())
        .transpose();
    let inverse = a.inverse();
    if !inverse.is_finite() {
        return None;
    }
    let norm = |m: glam::DMat3| m.to_cols_array().into_iter().map(f64::abs).sum::<f64>();
    let mut points = [glam::DVec2::ZERO; 8];
    let mut point_count = 0;
    let mut numerical_m = 0.0_f64;
    for depth in [lo, hi] {
        for sx in [-1.0, 1.0] {
            for sy in [-1.0, 1.0] {
                let ww = if perspective { depth } else { w.w };
                let rhs = DVec3::new(
                    sx * (ww + error[0] + error[3]) - rows[0].w,
                    sy * (ww + error[1] + error[3]) - rows[1].w,
                    depth - depth_row.w,
                );
                let p = inverse * rhs;
                // A badly conditioned solve with error larger than the shader's own
                // coordinate envelope disables tightening rather than losing receivers.
                let rounding = 256.0
                    * f64::EPSILON
                    * norm(inverse)
                    * (norm(a) * p.abs().max_element() + rhs.abs().max_element() + 1.0);
                if !p.is_finite()
                    || !rounding.is_finite()
                    || rounding > f64::from(f32::EPSILON) * radius.max(1.0)
                {
                    return None;
                }
                numerical_m = numerical_m.max(rounding);
                let q = light * p.extend(1.0);
                let xy = glam::DVec2::new(q.x, q.y);
                points[point_count] = xy;
                point_count += 1;
            }
        }
    }
    points.sort_unstable_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
    let cross = |a: glam::DVec2, b: glam::DVec2, c: glam::DVec2| (b - a).perp_dot(c - a);
    let mut hull = [glam::DVec2::ZERO; 16];
    let mut count = 0;
    for p in &points {
        if count > 0 && hull[count - 1] == *p {
            continue;
        }
        while count >= 2 && cross(hull[count - 2], hull[count - 1], *p) <= 0.0 {
            count -= 1;
        }
        hull[count] = *p;
        count += 1;
    }
    let lower = count;
    for p in points.iter().rev().skip(1) {
        if count > 0 && hull[count - 1] == *p {
            continue;
        }
        while count > lower && cross(hull[count - 2], hull[count - 1], *p) <= 0.0 {
            count -= 1;
        }
        hull[count] = *p;
        count += 1;
    }
    if count < 4 {
        return None;
    }
    count -= 1;
    if count > 8 {
        return None;
    }
    let projection_error = glam::DVec2::from_array(std::array::from_fn(|i| {
        let r = light.row(i);
        32.0 * f64::from(f32::EPSILON)
            * (r.truncate().abs().element_sum() * (radius + normal_bias_m) + r.w.abs() + 1.0)
    }));
    let hull_error = 128.0
        * f64::EPSILON
        * points
            .iter()
            .map(|p| p.abs().max_element())
            .fold(1.0_f64, f64::max);
    let mut planes = [glam::DVec4::ZERO; 8];
    for i in 0..count {
        let delta = hull[(i + 1) % count] - hull[i];
        let n = glam::DVec2::new(-delta.y, delta.x);
        let mut plane = light.row(0) * n.x + light.row(1) * n.y;
        let norm = plane.truncate().length();
        if norm == 0.0 || !norm.is_finite() {
            return None;
        }
        plane.w -= n.dot(hull[i]);
        plane.w += norm * (normal_bias_m + numerical_m)
            + n.abs().element_sum() * 2.0 * f64::from(filter_radius + 1) / f64::from(map_size)
            + n.abs().dot(projection_error)
            + n.length() * hull_error;
        planes[i] = plane / norm;
        if !planes[i].is_finite() {
            return None;
        }
    }
    Some(ReceiverVolume {
        light,
        planes,
        count,
    })
}

#[cfg(test)]
mod receiver_tests {
    use super::*;
    #[test]
    fn receiver_volume_keeps_both_light_directions_filter_and_normal_offset() {
        let mut witnessed = 0;
        for radius in [1.0, 25.0, 100.0, 12000.0] {
            for origin in [DVec3::ZERO, DVec3::new(3e12, -2e12, 1e12)] {
                for direction in [DVec3::Z, DVec3::Y, DVec3::new(0.3, 0.7, 0.2).normalize()] {
                    let light = directional_clip(origin, direction, radius, 12000.0, 1024)
                        .unwrap()
                        .as_mat4()
                        .as_dmat4();
                    let perspective =
                        glam::dcamera::rh::proj::directx::perspective(0.9, 1.7, 0.1, 1e8);
                    let mut reversed = perspective.transpose();
                    reversed.z_axis = reversed.w_axis - reversed.z_axis;
                    let mut oblique = perspective.transpose();
                    oblique.z_axis += oblique.x_axis * 0.03;
                    for projection in [
                        perspective,
                        reversed.transpose(),
                        oblique.transpose(),
                        glam::dcamera::rh::proj::directx::orthographic(
                            -20.0, 20.0, -15.0, 15.0, 0.1, 300.0,
                        ),
                    ] {
                        let primary = (projection * DMat4::from_rotation_y(0.7))
                            .as_mat4()
                            .as_dmat4();
                        let tightened =
                            receiver_caster_volume(primary, light, radius, 0.2, 0.0, 1024, 2)
                                .unwrap();
                        for i in 0..2000 {
                            let t = i as f64;
                            let spread = if i % 2 == 0 { radius.min(20.0) } else { radius };
                            let p = DVec3::new(
                                (t * 1.7).sin() * spread,
                                (t * 0.3).sin() * spread,
                                (t * 0.9).cos() * spread,
                            );
                            let clip = primary * p.extend(1.0);
                            if p.length() > radius
                                || clip.w <= 0.0
                                || clip.x.abs() > clip.w
                                || clip.y.abs() > clip.w
                                || clip.z < 0.0
                                || clip.z > clip.w
                            {
                                continue;
                            }
                            witnessed += 1;
                            let n = DVec3::new((t * 0.2).sin(), (t * 0.4).cos(), (t * 0.6).sin())
                                .normalize();
                            let inverse = light.inverse();
                            for side in [-1.0, 1.0] {
                                let sample = p
                                    + n * 0.2
                                    + inverse.transform_vector3(DVec3::new(
                                        side * 6.0 / 1024.0,
                                        -side * 6.0 / 1024.0,
                                        0.0,
                                    ));
                                for depth in [-10000.0, -100.0, 0.0, 100.0, 10000.0] {
                                    let caster = sample + direction * depth;
                                    if sphere_intersects_clip(light, caster, 0.0) {
                                        assert!(
                                            tightened.intersects_sphere(caster, 0.0),
                                            "lost receiver {p:?} caster {caster:?}"
                                        );
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        assert!(witnessed > 100);
        assert!(
            receiver_caster_volume(DMat4::ZERO, DMat4::IDENTITY, 100.0, 0.0, 0.0, 1024, 2)
                .is_none()
        );
    }
    #[test]
    fn receiver_volume_rejects_lateral_casters_but_keeps_off_camera_occluders() {
        let primary = glam::dcamera::rh::proj::directx::perspective(0.9, 1.0, 0.1, 1e8)
            .as_mat4()
            .as_dmat4();
        let light = directional_clip(DVec3::ZERO, DVec3::Z, 100.0, 12000.0, 1024)
            .unwrap()
            .as_mat4()
            .as_dmat4();
        let tightened = receiver_caster_volume(primary, light, 100.0, 0.001, 0.0, 1024, 2).unwrap();
        assert!(sphere_intersects_clip(
            light,
            DVec3::new(80.0, 0.0, -4.0),
            0.5
        ));
        assert!(!tightened.intersects_sphere(DVec3::new(80.0, 0.0, -4.0), 0.5));
        let uncertain =
            receiver_caster_volume(primary, light, 100.0, 0.001, 100.0, 1024, 2).unwrap();
        assert!(uncertain.intersects_sphere(DVec3::new(80.0, 0.0, -4.0), 0.5));
        for error in [f64::NAN, f64::INFINITY, -1.0] {
            assert!(receiver_caster_volume(primary, light, 100.0, 0.001, error, 1024, 2).is_none());
        }
        assert!(tightened.intersects_sphere(DVec3::new(0.0, 0.0, 10.0), 0.5));
    }
}
