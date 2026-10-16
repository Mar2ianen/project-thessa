//! Host conversion only. Decoding, scene transforms and material/image source
//! ownership no longer belong to Bevy's WorldAsset/glTF loader for this model.
use super::*;
use bevy::{
    image::{ImageAddressMode, ImageFilterMode, ImageSampler, ImageSamplerDescriptor},
    mesh::{Indices, PrimitiveTopology},
    render::render_resource::Face,
};

type HostPrimitive = (Handle<Mesh>, Handle<StandardMaterial>);

pub(super) fn convert(
    source: &thessa_render_core::mesh::MeshAsset,
    meshes: &mut Assets<Mesh>,
    images: &mut Assets<Image>,
    materials: &mut Assets<StandardMaterial>,
) -> Result<Vec<HostPrimitive>, String> {
    source.validate().map_err(str::to_owned)?;
    // The old StandardMaterial bridge has no general normal/occlusion strength
    // input. Refuse a lossy conversion (the shipped X15 uses unity for both).
    if source
        .materials
        .iter()
        .any(|m| m.normal_scale != 1.0 || m.occlusion_strength != 1.0)
    {
        return Err("Bevy bridge requires unity normal/AO strengths".into());
    }
    let mut textures = std::collections::BTreeMap::new();
    let mut texture = |id: Option<usize>, srgb: bool| {
        id.map(|id| {
            textures
                .entry((id, srgb))
                .or_insert_with(|| {
                    let t = source.textures[id];
                    let data = &source.images[t.image];
                    let mut image = Image::new(
                        Extent3d {
                            width: data.width,
                            height: data.height,
                            depth_or_array_layers: 1,
                        },
                        TextureDimension::D2,
                        data.rgba8.clone(),
                        if srgb {
                            TextureFormat::Rgba8UnormSrgb
                        } else {
                            TextureFormat::Rgba8Unorm
                        },
                        RenderAssetUsages::default(),
                    );
                    let wrap = |w| match w {
                        thessa_render_core::mesh::Wrap::Clamp => ImageAddressMode::ClampToEdge,
                        thessa_render_core::mesh::Wrap::Mirror => ImageAddressMode::MirrorRepeat,
                        _ => ImageAddressMode::Repeat,
                    };
                    image.sampler = ImageSampler::Descriptor(ImageSamplerDescriptor {
                        address_mode_u: wrap(t.wrap[0]),
                        address_mode_v: wrap(t.wrap[1]),
                        mag_filter: if t.linear {
                            ImageFilterMode::Linear
                        } else {
                            ImageFilterMode::Nearest
                        },
                        min_filter: if t.min_linear {
                            ImageFilterMode::Linear
                        } else {
                            ImageFilterMode::Nearest
                        },
                        ..default()
                    });
                    images.add(image)
                })
                .clone()
        })
    };
    let handles: Vec<_> = source
        .materials
        .iter()
        .map(|m| {
            materials.add(StandardMaterial {
                base_color: Color::linear_rgba(
                    m.base_color[0],
                    m.base_color[1],
                    m.base_color[2],
                    m.base_color[3],
                ),
                base_color_texture: texture(m.base_texture, true),
                normal_map_texture: texture(m.normal_texture, false),
                metallic_roughness_texture: texture(m.metallic_roughness_texture, false),
                metallic: m.metallic,
                perceptual_roughness: m.roughness,
                occlusion_texture: texture(m.occlusion_texture, false),
                double_sided: m.double_sided,
                cull_mode: if m.double_sided {
                    None
                } else {
                    Some(Face::Back)
                },
                ..default()
            })
        })
        .collect();
    source
        .primitives
        .iter()
        .map(|p| {
            let mut mesh = Mesh::new(
                PrimitiveTopology::TriangleList,
                RenderAssetUsages::default(),
            );
            mesh.insert_attribute(
                Mesh::ATTRIBUTE_POSITION,
                p.vertices.iter().map(|v| v.position).collect::<Vec<_>>(),
            );
            mesh.insert_attribute(
                Mesh::ATTRIBUTE_NORMAL,
                p.vertices.iter().map(|v| v.normal).collect::<Vec<_>>(),
            );
            mesh.insert_attribute(
                Mesh::ATTRIBUTE_UV_0,
                p.vertices.iter().map(|v| v.uv).collect::<Vec<_>>(),
            );
            mesh.insert_indices(Indices::U32(p.indices.clone()));
            if source.materials[p.material].normal_texture.is_some() {
                mesh.generate_tangents().map_err(|e| e.to_string())?;
            }
            Ok((meshes.add(mesh), handles[p.material].clone()))
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn shipped_source_converts_to_bevy_without_world_asset_loading_or_texture_role_loss() {
        let source = thessa_render_assets::decode_glb(
            include_bytes!("../../../../assets/models/north_american_x-15_plane.glb"),
            Default::default(),
        )
        .unwrap();
        let mut meshes = Assets::default();
        let mut images = Assets::default();
        let mut materials = Assets::default();
        let handles = convert(&source, &mut meshes, &mut images, &mut materials).unwrap();
        assert_eq!(handles.len(), 2);
        assert_eq!(images.len(), 6);
        for (mesh, material) in handles {
            let mesh = meshes.get(&mesh).unwrap();
            assert!(mesh.attribute(Mesh::ATTRIBUTE_TANGENT).is_some());
            let material = materials.get(&material).unwrap();
            assert!(material.double_sided && material.cull_mode.is_none());
            assert!(material.normal_map_texture.is_some());
            assert_eq!(
                material.metallic_roughness_texture,
                material.occlusion_texture
            );
            assert_eq!(
                images
                    .get(material.base_color_texture.as_ref().unwrap())
                    .unwrap()
                    .texture_descriptor
                    .format,
                TextureFormat::Rgba8UnormSrgb
            );
            assert_eq!(
                images
                    .get(material.normal_map_texture.as_ref().unwrap())
                    .unwrap()
                    .texture_descriptor
                    .format,
                TextureFormat::Rgba8Unorm
            );
        }
    }
}
