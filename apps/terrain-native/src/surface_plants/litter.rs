// SPDX-License-Identifier: GPL-3.0-or-later
//! Scanned CC0 fern organs, dry pigment and canonical ground projection.
use super::*;

pub(super) fn source() -> Result<&'static MeshAsset, String> {
    static SOURCE: std::sync::OnceLock<Result<MeshAsset, String>> = std::sync::OnceLock::new();
    SOURCE
        .get_or_init(|| {
            thessa_render_assets::decode_glb_cutouts(
                include_bytes!("../../../../assets/vegetation/fern-litter-01.glb"),
                Default::default(),
            )
        })
        .as_ref()
        .map_err(Clone::clone)
}

pub(super) fn material(prepared: &mut Prepared) -> Result<usize, String> {
    let source = source()?;
    let mut material = super::foliage::copy_material(&mut prepared.asset, source, 0);
    material.metallic = 0.0;
    material.roughness = 0.85;
    material.diffuse_transmission = 0.0;
    let index = prepared.asset.materials.len();
    prepared.asset.materials.push(material);
    Ok(index)
}

pub(super) fn append_frond(
    mesh: &mut Primitive,
    field: &PlanetField,
    source: &Primitive,
    center: DVec3,
    along: DVec3,
    length: f64,
    anchor: DVec3,
) {
    let up = center.normalize();
    let across = up.cross(along).normalize();
    let min_curl = source
        .vertices
        .iter()
        .map(|v| f64::from(v.position[1]))
        .fold(f64::INFINITY, f64::min);
    let project = |vertex: Vertex| {
        let point = center
            + along * ((f64::from(vertex.position[2]) - 0.5) * length)
            + across * (f64::from(vertex.position[0]) * length);
        // Preserve the scanned curl instead of clipping the organ into a flat card.
        let curl = (f64::from(vertex.position[1]) - min_curl) * length;
        let ground = super::wet_ground::ground_position;
        let position = ground(field, point) + up * (0.0015 + curl);
        // Inverse-transpose of the ground deformation, including its local slope.
        let step = 0.01;
        let tangent = |direction: DVec3| {
            (ground(field, point + direction * step) - ground(field, point - direction * step))
                / (2.0 * step)
        };
        let deformation = DMat3::from_cols(tangent(across), up, tangent(along));
        let normal = (deformation.inverse().transpose()
            * DVec3::from(vertex.normal.map(f64::from)))
        .normalize();
        Vertex {
            position: (position - anchor).as_vec3().to_array(),
            normal: normal.as_vec3().to_array(),
            uv: vertex.uv,
        }
    };
    for tri in source.indices.as_chunks::<3>().0 {
        conform_triangle(mesh, tri.map(|i| source.vertices[i as usize]), &project, 0);
    }
}

// Bounded organ-only refinement: long carrier faces cannot cut through soil
// between correctly rooted vertices. Six bisections cap source growth at 64x;
// the caller admits whole organs against a separate byte budget.
fn conform_triangle(
    mesh: &mut Primitive,
    raw: [Vertex; 3],
    project: &impl Fn(Vertex) -> Vertex,
    depth: u8,
) {
    let p = raw.map(project);
    if depth < 6 {
        let midpoint = |a: Vertex, b: Vertex| Vertex {
            position: std::array::from_fn(|i| (a.position[i] + b.position[i]) * 0.5),
            uv: std::array::from_fn(|i| (a.uv[i] + b.uv[i]) * 0.5),
            normal: (glam::Vec3::from(a.normal) + glam::Vec3::from(b.normal))
                .normalize()
                .to_array(),
        };
        let edges = [(0, 1, 2), (1, 2, 0), (2, 0, 1)];
        let error = edges
            .iter()
            .map(|&(a, b, _)| {
                let sample = project(midpoint(raw[a], raw[b]));
                let linear = (DVec3::from(p[a].position.map(f64::from))
                    + DVec3::from(p[b].position.map(f64::from)))
                    * 0.5;
                (DVec3::from(sample.position.map(f64::from)) - linear).length()
            })
            .fold(0.0, f64::max);
        if error > 0.0015 {
            let &(a, b, c) = edges
                .iter()
                .max_by(|&&(a, b, _), &&(c, d, _)| {
                    let distance = |x: usize, y: usize| {
                        (DVec3::from(p[x].position.map(f64::from))
                            - DVec3::from(p[y].position.map(f64::from)))
                        .length_squared()
                    };
                    distance(a, b).total_cmp(&distance(c, d))
                })
                .unwrap();
            let middle = midpoint(raw[a], raw[b]);
            conform_triangle(mesh, [raw[a], middle, raw[c]], project, depth + 1);
            conform_triangle(mesh, [middle, raw[b], raw[c]], project, depth + 1);
            return;
        }
    }
    // Keep source winding: the radial-height deformation preserves orientation.
    // A single smooth shading normal must not independently flip carrier faces.
    let first = mesh.vertices.len() as u32;
    mesh.vertices.extend(p);
    mesh.indices.extend([first, first + 1, first + 2]);
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn dead_fronds_keep_botanical_cutouts_and_dielectric_pbr_maps() {
        let asset = source().unwrap();
        assert_eq!(asset.primitives.len(), 3);
        for p in &asset.primitives {
            assert!(p.vertices.len() >= 60 && p.indices.len() >= 150);
            let min = p
                .vertices
                .iter()
                .map(|v| v.position[2])
                .fold(f32::INFINITY, f32::min);
            let max = p
                .vertices
                .iter()
                .map(|v| v.position[2])
                .fold(f32::NEG_INFINITY, f32::max);
            assert!(min >= -0.1 && (max - 1.0).abs() < 1e-4);
            let min_y = p
                .vertices
                .iter()
                .map(|v| v.position[1])
                .fold(f32::INFINITY, f32::min);
            let max_y = p
                .vertices
                .iter()
                .map(|v| v.position[1])
                .fold(f32::NEG_INFINITY, f32::max);
            assert!(
                (0.015..0.075).contains(&(max_y - min_y)),
                "resting frond retains pinna folds without the living rachis arch"
            );
            let sheet_normal = p
                .indices
                .as_chunks::<3>()
                .0
                .iter()
                .map(|tri| {
                    let v = tri.map(|i| p.vertices[i as usize]);
                    let positions = v.map(|v| glam::Vec3::from(v.position));
                    let area = (positions[1] - positions[0])
                        .cross(positions[2] - positions[0])
                        .length();
                    v.into_iter()
                        .map(|v| glam::Vec3::from(v.normal))
                        .sum::<glam::Vec3>()
                        * area
                })
                .sum::<glam::Vec3>()
                .normalize();
            assert!(
                sheet_normal.dot(glam::Vec3::Y) > 0.99,
                "blade face must lie in XZ; PCA arch variance must not become width"
            );
            let m = &asset.materials[p.material];
            assert_eq!(m.metallic, 0.0);
            assert_eq!(
                m.normal_scale, 0.35,
                "microrelief follows offline sheet-height compression"
            );
            assert_eq!(m.alpha_mode, thessa_render_core::mesh::AlphaMode::Mask);
            assert!(m.normal_texture.is_some() && m.metallic_roughness_texture.is_some());
            let image = &asset.images[asset.textures[m.base_texture.unwrap()].image];
            assert!(image.rgba8.as_chunks::<4>().0.iter().any(|p| p[3] == 0));
            assert!(image.rgba8.as_chunks::<4>().0.iter().any(|p| p[3] == 255));
        }
    }
}
