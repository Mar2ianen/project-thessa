//! Conservative caster rejection using the shared homogeneous sphere test.
use glam::{DVec3, Mat4};
use std::ops::Range;

#[derive(Clone, Copy)]
pub(crate) struct Bounds {
    center: DVec3,
    radius: f64,
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
        let radius = self.radius * stretch + 32.0 * f64::from(f32::EPSILON) * magnitude.length();
        thessa_render_core::shadow::sphere_intersects_clip(clip.as_dmat4(), center, radius)
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
