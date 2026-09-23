use glam::DVec3;

use crate::{BodyStation, ProceduralBody, tessellate_fuselage};

fn asymmetric_body() -> ProceduralBody {
    let stations = vec![
        BodyStation::new(-2.0, 1.0, 2.0, 0.5, 2.5, 3.5, 0.5, -0.25).unwrap(),
        BodyStation::new(4.0, 2.0, 1.0, 1.5, 3.0, 4.0, -1.0, 0.75).unwrap(),
    ];
    ProceduralBody::new("asymmetric loft", stations, DVec3::new(10.0, 20.0, 30.0)).unwrap()
}

#[test]
fn tessellation_is_finite_nondegenerate_and_outward_wound() {
    let mesh = tessellate_fuselage(&asymmetric_body(), 3, 16).unwrap();

    assert_eq!(mesh.positions.len(), mesh.normals.len());
    assert_eq!(mesh.indices.len() % 3, 0);
    assert!(mesh.positions.iter().all(|position| position.is_finite()));
    assert!(
        mesh.normals
            .iter()
            .all(|normal| { normal.is_finite() && (normal.length() - 1.0).abs() < 1e-12 })
    );

    for triangle in mesh.indices.chunks_exact(3) {
        let [a, b, c] = [
            mesh.positions[triangle[0] as usize],
            mesh.positions[triangle[1] as usize],
            mesh.positions[triangle[2] as usize],
        ];
        let face_normal = (b - a).cross(c - a);
        assert!(face_normal.is_finite());
        assert!(
            face_normal.length_squared() > 0.0,
            "degenerate triangle {triangle:?}"
        );

        let average_normal = mesh.normals[triangle[0] as usize]
            + mesh.normals[triangle[1] as usize]
            + mesh.normals[triangle[2] as usize];
        assert!(
            face_normal.dot(average_normal) > 0.0,
            "triangle winding disagrees with normals: {triangle:?}"
        );
    }
}

#[test]
fn tessellation_preserves_asymmetric_bounding_envelope_and_offsets() {
    let mesh = tessellate_fuselage(&asymmetric_body(), 2, 16).unwrap();
    let min = mesh.positions.iter().copied().reduce(DVec3::min).unwrap();
    let max = mesh.positions.iter().copied().reduce(DVec3::max).unwrap();

    assert_eq!(min, DVec3::new(8.0, 17.0, 29.25));
    assert_eq!(max, DVec3::new(14.0, 21.5, 31.75));
}

#[test]
fn tessellation_is_stable_and_uses_per_interval_axial_samples() {
    let body = asymmetric_body();
    let first = tessellate_fuselage(&body, 3, 16).unwrap();
    let second = tessellate_fuselage(&body, 3, 16).unwrap();

    assert_eq!(first, second);
    let side_ring_count = (body.stations.len() - 1) * 3 + 1;
    assert_eq!(first.positions.len(), side_ring_count * 16 + 2 * (16 + 1));
    assert!(tessellate_fuselage(&body, 0, 16).is_err());
    assert!(tessellate_fuselage(&body, 1, 7).is_err());
}
