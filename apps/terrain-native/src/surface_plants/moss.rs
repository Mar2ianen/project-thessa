//! Pinned CC0 botanical shoot shapes through the existing cutout/PBR path.
use super::*;
use thessa_render_core::mesh::AlphaMode;

pub(super) fn source() -> Result<&'static MeshAsset, String> {
    static SOURCE: std::sync::OnceLock<Result<MeshAsset, String>> = std::sync::OnceLock::new();
    SOURCE
        .get_or_init(|| {
            thessa_render_assets::decode_glb_cutouts(
                include_bytes!("../../../../assets/vegetation/moss-shoots-01.glb"),
                Default::default(),
            )
        })
        .as_ref()
        .map_err(Clone::clone)
}

pub(super) fn materials(prepared: &mut Prepared) -> Result<[usize; 3], String> {
    let source = source()?;
    Ok([
        [1.0, 1.0, 1.0, 1.0],
        [0.82, 0.92, 0.72, 1.0],
        [0.92, 0.90, 0.75, 1.0],
    ]
    .map(|tint| {
        let mut material = super::foliage::copy_material(&mut prepared.asset, source, 0);
        material.base_color = tint;
        material.roughness = 0.98;
        material.alpha_mode = AlphaMode::Mask;
        material.alpha_cutoff = 0.5;
        let index = prepared.asset.materials.len();
        prepared.asset.materials.push(material);
        index
    }))
}

pub(super) fn append_shoot(
    mesh: &mut Primitive,
    source: &Primitive,
    root: DVec3,
    rotation: DMat3,
    scale: f64,
    anchor: DVec3,
) {
    let first = mesh.vertices.len() as u32;
    mesh.vertices.extend(source.vertices.iter().map(|v| {
        Vertex {
            position: (root + rotation * DVec3::from(v.position.map(f64::from)) * scale - anchor)
                .as_vec3()
                .to_array(),
            normal: (rotation * DVec3::from(v.normal.map(f64::from)))
                .normalize()
                .as_vec3()
                .to_array(),
            uv: v.uv,
        }
    }));
    mesh.indices
        .extend(source.indices.iter().map(|i| first + i));
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn pinned_shoots_keep_botanical_cutouts_and_standard_pbr_maps() {
        let source = source().unwrap();
        source.validate().unwrap();
        assert_eq!(source.primitives.len(), 3);
        let mut counts = source
            .primitives
            .iter()
            .map(|p| p.indices.len() / 3)
            .collect::<Vec<_>>();
        counts.sort_unstable();
        assert_eq!(counts, [6, 6, 8]);
        for primitive in &source.primitives {
            let m = &source.materials[primitive.material];
            assert_eq!(m.alpha_mode, AlphaMode::Mask);
            assert!(m.normal_texture.is_some() && m.metallic_roughness_texture.is_some());
            let image = &source.images[source.textures[m.base_texture.unwrap()].image];
            let pixels = image.rgba8.as_chunks::<4>().0;
            assert!(pixels.iter().any(|p| p[3] == 0));
            assert!(pixels.iter().any(|p| p[3] == 255));
            assert!(
                primitive
                    .vertices
                    .iter()
                    .any(|v| v.uv[0] > 0.01 && v.uv[1] > 0.01)
            );
        }
    }
}
