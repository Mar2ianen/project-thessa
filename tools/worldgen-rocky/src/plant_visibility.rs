//! Optical asset bake: cosine-weighted visibility, reusing Parry's triangle BVH.

use crate::offline::PlantMesh;
use parry3d_f64::{
    math::Vector,
    query::{Ray, RayCast},
    shape::TriMesh,
};

pub(crate) fn bake(mesh: &PlantMesh, samples: usize) -> Vec<f32> {
    assert!(samples > 0);
    let vertices = mesh
        .positions
        .iter()
        .map(|p| Vector::from_array(p.map(f64::from)))
        .collect();
    let triangles = mesh.indices.as_chunks::<3>().0.to_vec();
    let shape = TriMesh::new(vertices, triangles).expect("validated prototype triangles");
    let mut cache = std::collections::BTreeMap::new();
    mesh.positions
        .iter()
        .zip(&mesh.normals)
        .map(|(p, n)| {
            let key = (p.map(f32::to_bits), n.map(f32::to_bits));
            *cache.entry(key).or_insert_with(|| {
                let normal = Vector::from_array(n.map(f64::from)).normalize();
                let helper = if normal.y.abs() < 0.9 {
                    Vector::Y
                } else {
                    Vector::X
                };
                let tangent = normal.cross(helper).normalize();
                let bitangent = normal.cross(tangent);
                // Offset bounds f32 source-coordinate rounding at this prototype's
                // scale. Rays otherwise include every surface, on both sides.
                let origin = Vector::from_array(p.map(f64::from)) + normal * 0.00002;
                let visible = (0..samples)
                    .filter(|&i| {
                        let u = (i as f64 + 0.5) / samples as f64;
                        let angle = i as f64 * 2.399_963_229_728_653;
                        let direction = tangent * (u.sqrt() * angle.cos())
                            + bitangent * (u.sqrt() * angle.sin())
                            + normal * (1.0 - u).sqrt();
                        shape
                            .cast_local_ray(&Ray::new(origin, direction), f64::MAX, false)
                            .is_none()
                    })
                    .count();
                visible as f32 / samples as f32
            })
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn visibility_matches_open_plane_and_closed_cavity() {
        let mut mesh = PlantMesh {
            kind: crate::scatter::ScatterKind::FernClump,
            positions: vec![
                [0.0; 3],
                [-1.0, 0.0, -1.0],
                [1.0, 0.0, -1.0],
                [1.0, 0.0, 1.0],
                [-1.0, 0.0, 1.0],
            ],
            normals: vec![[0.0, 1.0, 0.0]; 5],
            uvs: vec![],
            foliage_tones: vec![],
            indices: vec![1, 3, 2, 1, 4, 3],
            albedo_srgb: [0.3; 3],
            stem: None,
            senescent: None,
            ambient_visibility: vec![],
        };
        for samples in [16, 32, 128] {
            assert_eq!(bake(&mesh, samples), vec![1.0; 5]);
        }
        // Visibility is geometry-only, independent of the world's horizon or
        // ground pigment. Both sides of an open sheet see an empty hemisphere.
        mesh.normals.fill([0.0, -1.0, 0.0]);
        assert_eq!(bake(&mesh, 128), vec![1.0; 5]);
        mesh.normals.fill([0.0, 1.0, 0.0]);
        mesh.positions.extend([
            [-1.0, 1.0, -1.0],
            [1.0, 1.0, -1.0],
            [1.0, 1.0, 1.0],
            [-1.0, 1.0, 1.0],
        ]);
        mesh.normals.resize(9, [0.0, 1.0, 0.0]);
        mesh.indices.extend([
            5, 6, 7, 5, 7, 8, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7, 6, 3, 4, 8, 3, 8, 7, 4, 1, 5, 4, 5,
            8,
        ]);
        for samples in [16, 32, 128] {
            assert_eq!(bake(&mesh, samples)[0], 0.0);
        }
        // Open the roof of the cavity.
        mesh.indices.drain(6..12);
        // Independent numerical reference for the square aperture (not samples
        // or ray queries): integrate h² / (pi * (h²+x²+z²)²) over [-1,1]².
        let n = 400;
        let step = 2.0 / n as f64;
        let reference: f64 = (0..n)
            .flat_map(|x| {
                (0..n).map(move |z| {
                    let x = -1.0 + (x as f64 + 0.5) * step;
                    let z = -1.0 + (z as f64 + 0.5) * step;
                    step * step / (std::f64::consts::PI * (1.0 + x * x + z * z).powi(2))
                })
            })
            .sum();
        assert!((f64::from(bake(&mesh, 4096)[0]) - reference).abs() < 0.003);
    }
}
