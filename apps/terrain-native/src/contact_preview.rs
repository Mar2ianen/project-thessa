//! Inspector-only contact wireframe, not an external/beauty vehicle model.
use glam::DVec3;
use thessa_sim_core::{CollisionAxis, CollisionGeometry, CollisionShape};

const ARC_SEGMENTS: i32 = 16;

/// Auto-fit before narrowing: even astronomical body-local dimensions must
/// not overflow an f32 UI coordinate. At most 64 parts, 16 circle segments.
fn segments(geometry: &CollisionGeometry) -> Vec<[[f64; 2]; 2]> {
    let bound = geometry.parts.iter().fold(0.0_f64, |bound, part| {
        bound
            .max(part.local_position_m.abs().max_element())
            .max(part.shape.bounding_radius_m())
    });
    if !bound.is_finite() || bound <= 0.0 {
        return Vec::new();
    }
    let mut lines = Vec::new();
    for part in &geometry.parts {
        let project = |point: DVec3| {
            let p = part.local_position_m / bound + part.local_orientation * point;
            [
                (p.x - p.y) / 2.0_f64.sqrt(),
                (p.x + p.y - 2.0 * p.z) / 6.0_f64.sqrt(),
            ]
        };
        let mut line = |a, b| lines.push([project(a), project(b)]);
        let mut arc = |center: DVec3, a: DVec3, b: DVec3, angle: f64| {
            for i in 0..ARC_SEGMENTS {
                let point = |i| {
                    let t = f64::from(i) * angle / f64::from(ARC_SEGMENTS);
                    center + a * t.cos() + b * t.sin()
                };
                line(point(i), point(i + 1));
            }
        };
        match part.shape {
            CollisionShape::Sphere { radius_m } => {
                let radius = radius_m / bound;
                for (a, b) in [
                    (DVec3::X, DVec3::Y),
                    (DVec3::Y, DVec3::Z),
                    (DVec3::Z, DVec3::X),
                ] {
                    arc(DVec3::ZERO, a * radius, b * radius, std::f64::consts::TAU);
                }
            }
            CollisionShape::Cuboid { half_extents_m } => {
                let h = half_extents_m / bound;
                let corner = |index| {
                    DVec3::new(
                        if index & 1 == 0 { -h.x } else { h.x },
                        if index & 2 == 0 { -h.y } else { h.y },
                        if index & 4 == 0 { -h.z } else { h.z },
                    )
                };
                for index in 0..8 {
                    for bit in [1, 2, 4] {
                        if index & bit == 0 {
                            line(corner(index), corner(index | bit));
                        }
                    }
                }
            }
            CollisionShape::Capsule {
                axis,
                half_segment_m,
                radius_m,
            } => {
                let axis = match axis {
                    CollisionAxis::X => DVec3::X,
                    CollisionAxis::Y => DVec3::Y,
                    CollisionAxis::Z => DVec3::Z,
                };
                let radial = axis.any_orthonormal_vector();
                let radius = radius_m / bound;
                let half = half_segment_m / bound;
                for side in [radial, axis.cross(radial)] {
                    for sign in [-1.0, 1.0] {
                        arc(
                            axis * (sign * half),
                            side * radius,
                            axis * (sign * radius),
                            std::f64::consts::PI,
                        );
                    }
                }
                // End the mutable arc borrow before adding the cylinder lines.
                for side in [radial, -radial, axis.cross(radial), -axis.cross(radial)] {
                    line(axis * half + side * radius, -axis * half + side * radius);
                }
            }
        }
    }
    lines
}

pub fn show(ui: &mut egui::Ui, geometry: &CollisionGeometry) {
    let lines = segments(geometry);
    ui.small("Body-frame contact shell · auto-fit isometric · not a beauty mesh");
    if lines.is_empty() {
        ui.label("Geometry cannot be projected at this scale");
        return;
    }
    let mut bound = 0.0_f64;
    for point in lines.iter().flatten() {
        bound = bound.max(point[0].abs()).max(point[1].abs());
    }
    let (rect, _) = ui.allocate_exact_size(egui::vec2(260.0, 140.0), egui::Sense::hover());
    let scale = 60.0 / bound.max(1e-12);
    for [a, b] in lines {
        let position =
            |p: [f64; 2]| rect.center() + egui::vec2((p[0] * scale) as f32, (p[1] * scale) as f32);
        ui.painter().line_segment(
            [position(a), position(b)],
            egui::Stroke::new(1.0, egui::Color32::LIGHT_BLUE),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use thessa_sim_core::{CollisionMaterial, CollisionPart};
    #[test]
    fn every_shape_projects_finitely_and_pose_changes_the_outline() {
        for shape in [
            CollisionShape::Sphere { radius_m: 2.0 },
            CollisionShape::Cuboid {
                half_extents_m: DVec3::new(1.0, 2.0, 3.0),
            },
            CollisionShape::Capsule {
                axis: CollisionAxis::Z,
                half_segment_m: 3.0,
                radius_m: 1.0,
            },
        ] {
            let mut geometry = CollisionGeometry::new(vec![
                CollisionPart::new(
                    DVec3::ZERO,
                    glam::DQuat::IDENTITY,
                    shape,
                    CollisionMaterial::default(),
                )
                .unwrap(),
            ])
            .unwrap();
            let before = segments(&geometry);
            assert!(!before.is_empty());
            let expected = match shape {
                CollisionShape::Sphere { .. } => 3 * ARC_SEGMENTS,
                CollisionShape::Cuboid { .. } => 12,
                CollisionShape::Capsule { .. } => 4 * ARC_SEGMENTS + 4,
            };
            assert_eq!(before.len(), expected as usize);
            assert!(
                before
                    .iter()
                    .flatten()
                    .flatten()
                    .all(|x| x.is_finite() && x.abs() < 4.0)
            );
            geometry.parts[0].local_position_m = DVec3::new(1e200, -1e200, 1e200);
            let after = segments(&geometry);
            assert!(
                after
                    .iter()
                    .flatten()
                    .flatten()
                    .all(|x| x.is_finite() && x.abs() < 4.0)
            );
            assert_ne!(before, after);
        }
        assert!(1.0 - (std::f64::consts::PI / f64::from(ARC_SEGMENTS)).cos() < 0.0193);
        assert!(1.0 - (std::f64::consts::PI / f64::from(2 * ARC_SEGMENTS)).cos() < 0.0049);
    }
}
