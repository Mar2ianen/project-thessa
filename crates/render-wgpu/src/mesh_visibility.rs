//! Conservative caster rejection using the shared homogeneous sphere test.
use glam::{DVec3, Mat4};
use std::ops::Range;

#[derive(Clone, Copy)]
pub(crate) struct Bounds {
    center: DVec3,
    radius: f64,
}

#[derive(Clone, Copy)]
pub(crate) struct WorldBounds {
    center: DVec3,
    radius: f64,
    stretch: f64,
    magnitude: f64,
    axis_abs_norm: f64,
}
#[derive(Clone, Copy)]
pub(crate) struct LightProjection {
    norm: f64,
    offset: f64,
    pixels: f64,
}
impl LightProjection {
    pub fn new(clip: Mat4, size: u32) -> Option<Self> {
        let clip = clip.as_dmat4();
        if !clip.is_finite() || clip.row(3) != glam::DVec4::W || size == 0 {
            return None;
        }
        Some(Self {
            norm: (clip.row(0).truncate().length_squared()
                + clip.row(1).truncate().length_squared())
            .sqrt(),
            offset: glam::DVec2::new(clip.w_axis.x, clip.w_axis.y).length(),
            pixels: f64::from(size) * 0.5,
        })
    }
}
impl WorldBounds {
    pub fn receiver_error_m(self) -> f64 {
        128.0 * f64::from(f32::EPSILON) * self.magnitude
    }
    fn radius_with_error(self, error: f64) -> f64 {
        self.radius + error * (self.stretch + 32.0 * f64::from(f32::EPSILON) * self.axis_abs_norm)
    }
    pub fn intersects(self, clip: Mat4, error: f64) -> bool {
        thessa_render_core::shadow::sphere_intersects_clip(
            clip.as_dmat4(),
            self.center,
            self.radius_with_error(error),
        )
    }
    pub fn intersects_receivers(
        self,
        volume: &thessa_render_core::shadow::ReceiverVolume,
        error: f64,
    ) -> bool {
        volume.intersects_sphere(self.center, self.radius_with_error(error))
    }
    pub fn projected_error(self, light: LightProjection, error: f64) -> Option<f64> {
        if !error.is_finite() || error < 0.0 {
            return None;
        }
        // ||L M||_F <= ||L||_F ||M||_2. Gershgorin already bounds ||M||_2;
        // Cauchy-Schwarz encloses both shader products without rebuilding LM.
        let projected = (light.norm * error * self.stretch
            + 32.0
                * f64::from(f32::EPSILON)
                * (light.norm * (self.magnitude + error * self.axis_abs_norm) + light.offset))
            * light.pixels;
        projected.is_finite().then_some(projected)
    }
}

impl Bounds {
    pub fn intersects_uploaded(self, clip: Mat4, bytes: &[u8]) -> bool {
        if bytes.len() != 64 {
            return true;
        }
        let words = std::array::from_fn(|j| {
            f32::from_le_bytes(bytes[j * 4..j * 4 + 4].try_into().unwrap())
        });
        self.intersects(clip, Mat4::from_cols_array(&words))
    }
    #[cfg(test)]
    pub fn expanded(self, error_m: f64) -> Self {
        Self {
            radius: self.radius + error_m,
            ..self
        }
    }

    /// Orthographic light error in radial texels, including an f32 arithmetic
    /// envelope for the shader's separate instance and clip multiplications.
    #[cfg(test)]
    pub fn projected_error_texels(
        self,
        clip: Mat4,
        bytes: &[u8],
        error_m: f64,
        size: u32,
    ) -> Option<f64> {
        if bytes.len() != 64 || !error_m.is_finite() || error_m < 0.0 || size == 0 {
            return None;
        }
        let model = Mat4::from_cols_array(&std::array::from_fn(|i| {
            f32::from_le_bytes(bytes[i * 4..i * 4 + 4].try_into().unwrap())
        }))
        .as_dmat4();
        let light = clip.as_dmat4();
        if !model.is_finite()
            || !light.is_finite()
            || model.row(3) != glam::DVec4::W
            || light.row(3) != glam::DVec4::W
        {
            return None;
        }
        let m = light * model;
        let scale =
            (m.row(0).truncate().length_squared() + m.row(1).truncate().length_squared()).sqrt();
        let abs = |m: glam::DMat4| {
            glam::DMat4::from_cols(
                m.x_axis.abs(),
                m.y_axis.abs(),
                m.z_axis.abs(),
                m.w_axis.abs(),
            )
        };
        let magnitude =
            abs(light) * (abs(model) * (self.center.abs() + DVec3::splat(self.radius)).extend(1.0));
        let rounding = 32.0 * f64::from(f32::EPSILON) * magnitude.truncate().truncate().length();
        let error = (scale * error_m + rounding) * f64::from(size) * 0.5;
        error.is_finite().then_some(error)
    }

    pub fn new(positions: impl Iterator<Item = [f32; 3]>) -> Option<Self> {
        let mut min = DVec3::splat(f64::INFINITY);
        let mut max = DVec3::splat(f64::NEG_INFINITY);
        for p in positions {
            let p = DVec3::from_array(p.map(f64::from));
            if !p.is_finite() {
                return None;
            }
            min = min.min(p);
            max = max.max(p);
        }
        let center = (min + max) * 0.5;
        let radius = (max - min).length() * 0.5;
        (center.is_finite() && radius.is_finite()).then_some(Self { center, radius })
    }

    /// `model` is the actual camera-relative f32 matrix uploaded to the shader.
    pub fn intersects(self, clip: Mat4, model: Mat4) -> bool {
        let (center, radius) = self.transformed(model);
        thessa_render_core::shadow::sphere_intersects_clip(clip.as_dmat4(), center, radius)
    }
    pub fn uploaded_world(self, bytes: &[u8]) -> Option<WorldBounds> {
        if bytes.len() != 64 {
            return None;
        }
        let model = Mat4::from_cols_array(&std::array::from_fn(|i| {
            f32::from_le_bytes(bytes[i * 4..i * 4 + 4].try_into().unwrap())
        }));
        if !model.is_finite() || model.row(3) != glam::Vec4::W {
            return None;
        }
        let mut world = self.transform(model);
        let m = model.as_dmat4();
        world.axis_abs_norm =
            (m.x_axis.truncate().abs() + m.y_axis.truncate().abs() + m.z_axis.truncate().abs())
                .length();
        Some(world)
    }
    fn transformed(self, model: Mat4) -> (DVec3, f64) {
        let world = self.transform(model);
        (world.center, world.radius)
    }
    fn transform(self, model: Mat4) -> WorldBounds {
        let m = model.as_dmat4();
        let axes = [
            m.x_axis.truncate(),
            m.y_axis.truncate(),
            m.z_axis.truncate(),
        ];
        // Gershgorin bounds the squared largest singular value, including the
        // tiny non-orthogonality of a narrowed rigid transform.
        let stretch = axes
            .iter()
            .map(|a| axes.iter().map(|b| a.dot(*b).abs()).sum::<f64>())
            .fold(0.0_f64, f64::max)
            .sqrt();
        let center = m.transform_point3(self.center);
        let magnitude = m.w_axis.truncate().abs()
            + axes[0].abs() * (self.center.x.abs() + self.radius)
            + axes[1].abs() * (self.center.y.abs() + self.radius)
            + axes[2].abs() * (self.center.z.abs() + self.radius);
        // Enclose f32 instance multiplication as well as the shared clip-plane
        // test's projection envelope. Invalid bounds deliberately fail open.
        let magnitude = magnitude.length();
        let radius = self.radius * stretch + 32.0 * f64::from(f32::EPSILON) * magnitude;
        WorldBounds {
            center,
            radius,
            stretch,
            magnitude,
            axis_abs_norm: 0.0,
        }
    }
}

/// Retain original instance indices and join consecutive visible instances;
/// no population thinning, source mutation or repacked instance buffer.
pub(crate) fn visible_runs(
    range: Range<u32>,
    mut visible: impl FnMut(u32) -> bool,
) -> impl Iterator<Item = Range<u32>> {
    let mut next = range.start;
    std::iter::from_fn(move || {
        while next < range.end && !visible(next) {
            next += 1;
        }
        let start = next;
        while next < range.end && visible(next) {
            next += 1;
        }
        (start < next).then_some(start..next)
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn caster_clip_preserves_intersections_and_original_indices() {
        let b = Bounds::new([[-0.1; 3], [0.1; 3]].into_iter()).unwrap();
        assert!(b.intersects(
            Mat4::IDENTITY,
            Mat4::from_translation(glam::vec3(0.0, 0.0, 0.5))
        ));
        assert!(b.intersects(
            Mat4::IDENTITY,
            Mat4::from_translation(glam::vec3(1.05, 0.0, 0.5))
        ));
        assert!(!b.intersects(
            Mat4::IDENTITY,
            Mat4::from_translation(glam::vec3(2.0, 0.0, 0.5))
        ));
        assert!(!b.intersects(
            Mat4::IDENTITY,
            Mat4::from_translation(glam::vec3(0.0, 0.0, -1.0))
        ));
        assert_eq!(
            visible_runs(10..18, |i| [11, 12, 15, 17].contains(&i)).collect::<Vec<_>>(),
            [11..13, 15..16, 17..18]
        );
        let mut all = visible_runs(3..7, |_| true);
        assert_eq!(all.next(), Some(3..7));
        assert!(all.next().is_none());
        assert!(visible_runs(3..7, |_| false).next().is_none());
    }

    #[test]
    fn rejection_never_removes_a_corner_inside_the_actual_f32_clip() {
        let positions = std::array::from_fn::<_, 8, _>(|i| {
            [
                if i & 1 == 0 { -2.0 } else { 3.0 },
                if i & 2 == 0 { -0.5 } else { 10.0 },
                if i & 4 == 0 { -0.2 } else { 0.4 },
            ]
        });
        let b = Bounds::new(positions.into_iter()).unwrap();
        for i in 0..2000 {
            let t = i as f32;
            let model = Mat4::from_scale_rotation_translation(
                glam::Vec3::splat(0.01 + (t * 0.71).sin().abs() * 8.0),
                glam::Quat::from_euler(glam::EulerRot::YXZ, t * 0.17, t * 0.37, t * 0.11),
                glam::vec3(
                    (t * 0.73).sin() * 80.0,
                    (t * 0.31).cos() * 80.0,
                    (t * 0.23).sin() * 80.0,
                ),
            );
            for clip in [
                Mat4::IDENTITY,
                glam::camera::rh::proj::directx::perspective(0.9, 1.6, 0.1, 500.0),
                thessa_render_core::shadow::directional_clip(
                    DVec3::splat(3.2e6),
                    DVec3::new(0.2, 0.8, 0.4).normalize(),
                    25.0,
                    12000.0,
                    1024,
                )
                .unwrap()
                .as_mat4(),
            ] {
                let combined = clip * model;
                let any_inside = positions.iter().any(|p| {
                    let p = combined * glam::Vec3::from_array(*p).extend(1.0);
                    p.x.abs() <= p.w && p.y.abs() <= p.w && p.z >= 0.0 && p.z <= p.w
                });
                assert!(
                    !any_inside || b.intersects(clip, model),
                    "lost a real f32 corner at {i}"
                );
            }
        }
        assert!(b.intersects(Mat4::from_cols_array(&[f32::NAN; 16]), Mat4::IDENTITY));
    }

    #[test]
    fn off_camera_casters_are_retained_in_their_light_cover() {
        let b = Bounds::new([[-0.2; 3], [0.2; 3]].into_iter()).unwrap();
        let model = Mat4::from_translation(glam::vec3(4.0, 0.0, -2.0));
        let primary = glam::camera::rh::proj::directx::perspective(0.9, 1.0, 0.1, 500.0);
        let shadow =
            thessa_render_core::shadow::directional_clip(DVec3::ZERO, DVec3::Y, 10.0, 100.0, 1024)
                .unwrap()
                .as_mat4();
        let bytes: Vec<_> = model
            .to_cols_array()
            .into_iter()
            .flat_map(f32::to_le_bytes)
            .collect();
        assert!(!b.intersects_uploaded(primary, &bytes));
        assert!(b.intersects_uploaded(shadow, &bytes));
        assert!(
            b.intersects_uploaded(shadow, &bytes[..60]),
            "missing transform must fail open"
        );
    }
}

#[cfg(test)]
mod shadow_lod_tests {
    use super::*;
    fn bytes(m: Mat4) -> Vec<u8> {
        m.to_cols_array()
            .into_iter()
            .flat_map(f32::to_le_bytes)
            .collect()
    }
    #[test]
    fn projected_error_follows_light_texels_scale_and_fails_closed() {
        let bounds = Bounds::new([[-1.0; 3], [1.0; 3]].into_iter()).unwrap();
        let light = Mat4::from_scale(glam::vec3(0.01, 0.01, 0.001));
        let error = bounds
            .projected_error_texels(light, &bytes(Mat4::IDENTITY), 0.1, 1024)
            .unwrap();
        assert!((0.724..0.725).contains(&error));
        let scaled = bounds
            .projected_error_texels(
                light,
                &bytes(Mat4::from_scale(glam::Vec3::splat(2.0))),
                0.1,
                1024,
            )
            .unwrap();
        assert!((scaled / error - 2.0).abs() < 1e-10);
        assert_eq!(
            bounds
                .projected_error_texels(light, &bytes(Mat4::IDENTITY), 0.1, 2048)
                .unwrap(),
            error * 2.0
        );
        assert!(
            bounds
                .projected_error_texels(
                    glam::camera::rh::proj::directx::perspective(1.0, 1.0, 0.1, 100.0),
                    &bytes(Mat4::IDENTITY),
                    0.1,
                    1024
                )
                .is_none()
        );
        assert!(
            bounds
                .projected_error_texels(light, &[], 0.1, 1024)
                .is_none()
        );
        assert!(
            bounds
                .projected_error_texels(light, &bytes(Mat4::IDENTITY), f64::NAN, 1024)
                .is_none()
        );
    }
}

#[cfg(test)]
mod cached_tests {
    use super::*;
    #[test]
    fn cached_spheres_match_primary_and_bound_original_shadow_error() {
        let bounds = Bounds::new([[-2.0, -1.0, -0.5], [1.0, 4.0, 2.0]].into_iter()).unwrap();
        for i in 0..2000 {
            let t = i as f32 * 0.017;
            let model = Mat4::from_scale_rotation_translation(
                glam::vec3(0.1 + t, 0.2 + t * 0.4, 0.3 + t * 0.7),
                glam::Quat::from_rotation_y(t) * glam::Quat::from_rotation_x(t * 0.3),
                glam::vec3(t.sin() * 100.0, t.cos() * 30.0, -5.0 - t),
            );
            let bytes: Vec<_> = model
                .to_cols_array()
                .into_iter()
                .flat_map(f32::to_le_bytes)
                .collect();
            let world = bounds.uploaded_world(&bytes).unwrap();
            for direction in [glam::DVec3::Z, glam::DVec3::new(0.3, 0.7, 0.2).normalize()] {
                let light = thessa_render_core::shadow::directional_clip(
                    glam::DVec3::ZERO,
                    direction,
                    100.0,
                    12000.0,
                    1024,
                )
                .unwrap()
                .as_mat4();
                assert_eq!(
                    bounds.intersects(light, model),
                    world.intersects(light, 0.0)
                );
                let error = 0.09;
                let cached = world
                    .projected_error(LightProjection::new(light, 1024).unwrap(), error)
                    .unwrap();
                let original = bounds
                    .expanded(error)
                    .projected_error_texels(light, &bytes, error, 1024)
                    .unwrap();
                assert!(
                    cached >= original - 1e-10,
                    "cached {cached} < original {original}"
                );
                if bounds.expanded(error).intersects(light, model) {
                    assert!(world.intersects(light, error));
                }
            }
        }
        assert!(bounds.uploaded_world(&[]).is_none());
        assert!(
            bounds
                .uploaded_world(
                    &Mat4::ZERO
                        .to_cols_array()
                        .into_iter()
                        .flat_map(f32::to_le_bytes)
                        .collect::<Vec<_>>()
                )
                .is_none()
        );
    }
}
