// SPDX-License-Identifier: GPL-3.0-or-later
//! Reuse the pinned CC0 B02 fronds instead of unresolved opaque pinnule needles.
use super::*;
use thessa_render_core::mesh::AlphaMode;

fn source() -> Result<&'static MeshAsset, String> {
    static SOURCE: std::sync::OnceLock<Result<MeshAsset, String>> = std::sync::OnceLock::new();
    SOURCE
        .get_or_init(|| {
            thessa_render_assets::decode_glb_surfaces(
                include_bytes!("../../../../assets/vegetation/fern-02.glb"),
                Default::default(),
            )
        })
        .as_ref()
        .map_err(Clone::clone)
}

pub(super) fn append(
    prepared: &mut Prepared,
    prototype: &thessa_worldgen_rocky::offline::PlantMesh,
    form: u8,
    reduced: bool,
    range: Range<u32>,
) -> Result<(), String> {
    use thessa_worldgen_rocky::scatter::ScatterKind;
    let source = source()?;
    let variant = usize::from(form).min(2);
    let original = &source.primitives[variant];
    // The pinned asset is a 1m-spaced gallery. Recover its isolated clump root
    // from the bounded footprint, independent of decoder node traversal order.
    let min = original
        .vertices
        .iter()
        .fold(glam::Vec3::splat(f32::INFINITY), |a, v| {
            a.min(glam::Vec3::from_array(v.position))
        });
    let max = original
        .vertices
        .iter()
        .fold(glam::Vec3::splat(f32::NEG_INFINITY), |a, v| {
            a.max(glam::Vec3::from_array(v.position))
        });
    let root = ((min + max) * 0.5).round();
    let radius = original
        .vertices
        .iter()
        .map(|v| (v.position[0] - root.x).hypot(v.position[2] - root.z))
        .fold(0.0_f32, f32::max);
    if !(0.1..1.0).contains(&radius) || max.y - min.y < 0.05 {
        return Err("pinned fern gallery footprint changed".into());
    }
    let tree = prototype.kind == ScatterKind::LycopsidTree;
    let sockets = if tree {
        thessa_worldgen_rocky::offline::plant_crown_attachments(form)
    } else {
        vec![[0.; 3]]
    };
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    for (socket_index, socket) in sockets.into_iter().enumerate() {
        let (target_radius, target_height, target_drop) = if tree {
            match form {
                1 => (4.7_f32, 2.0, 1.8),
                2 => (if socket_index == 0 { 3.3 } else { 4.0 }, 1.8, 1.8),
                _ => (3.6, 1.7, 1.5),
            }
        } else {
            match form {
                1 => (0.65, 0.45, 0.0),
                2 => (2.2, 1.1, 0.0),
                _ => (1.3, 0.75, 0.0),
            }
        };
        let horizontal = target_radius / radius;
        let vertical = target_height / (max.y - min.y);
        let drop = target_drop / (radius * radius);
        let first = vertices.len() as u32;
        for v in &original.vertices {
            let x = v.position[0] - root.x;
            let z = v.position[2] - root.z;
            let y = (v.position[1] - if tree { 0.0 } else { min.y }) * vertical
                - drop * (x * x + z * z);
            let n = glam::Vec3::new(
                v.normal[0] / horizontal + 2.0 * drop * x * v.normal[1] / (horizontal * vertical),
                v.normal[1] / vertical,
                v.normal[2] / horizontal + 2.0 * drop * z * v.normal[1] / (horizontal * vertical),
            )
            .normalize();
            vertices.push(Vertex {
                position: [
                    socket[0] + x * horizontal,
                    socket[1] + y,
                    socket[2] + z * horizontal,
                ],
                normal: n.to_array(),
                uv: v.uv,
            });
        }
        indices.extend(original.indices.iter().map(|i| first + i));
    }
    let mut material = source.materials[original.material].clone();
    let mut remap = |texture: Option<usize>| -> Option<usize> {
        texture.map(|index| {
            let texture = &source.textures[index];
            let image = &source.images[texture.image];
            let image_index = prepared
                .asset
                .images
                .iter()
                .position(|i| {
                    i.width == image.width && i.height == image.height && i.rgba8 == image.rgba8
                })
                .unwrap_or_else(|| {
                    let i = prepared.asset.images.len();
                    prepared.asset.images.push(image.clone());
                    i
                });
            let mut copy = *texture;
            copy.image = image_index;
            prepared
                .asset
                .textures
                .iter()
                .position(|t| t == &copy)
                .unwrap_or_else(|| {
                    let i = prepared.asset.textures.len();
                    prepared.asset.textures.push(copy);
                    i
                })
        })
    };
    material.base_texture = remap(material.base_texture);
    material.metallic_roughness_texture = remap(material.metallic_roughness_texture);
    material.normal_texture = remap(material.normal_texture);
    material.occlusion_texture = remap(material.occlusion_texture);
    material.alpha_mode = AlphaMode::Mask;
    material.diffuse_transmission = 0.18;
    material.base_color = match form {
        1 => [0.85, 1.0, 0.9, 1.0],
        2 => [1.0, 0.94, 0.72, 1.0],
        _ => [0.92, 1.0, 0.86, 1.0],
    };
    let index = prepared.asset.materials.len();
    prepared.asset.materials.push(material);
    prepared.asset.primitives.push(Primitive {
        vertices,
        indices,
        material: index,
        ambient_visibility: vec![],
    });
    prepared.ranges.push(range);
    prepared.reduced_primitives.push(reduced);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn pinned_fronds_keep_alpha_and_attach_to_shared_woody_sockets() {
        use thessa_worldgen_rocky::{
            offline,
            scatter::{ScatterId, ScatterKind},
        };
        for kind in [ScatterKind::FernClump, ScatterKind::LycopsidTree] {
            for form in 0..3 {
                let id = (0..1000)
                    .map(ScatterId)
                    .find(|id| offline::plant_form(*id, kind) == form)
                    .unwrap();
                let p = convert(
                    vec![ScatterItem {
                        id,
                        kind,
                        direction: DVec3::Y.to_array(),
                        ground_height_m: 0.,
                        scale: 1.,
                        yaw_rad: 0.,
                    }],
                    3_200_000.,
                )
                .unwrap();
                p.asset.validate().unwrap();
                let leaves: Vec<_> = p
                    .asset
                    .primitives
                    .iter()
                    .filter(|v| p.asset.materials[v.material].alpha_mode == AlphaMode::Mask)
                    .collect();
                assert_eq!(leaves.len(), 1);
                let leaf = leaves[0];
                assert!(leaf.indices.len() / 3 < 8000);
                let material = &p.asset.materials[leaf.material];
                assert_eq!(material.alpha_cutoff, 0.5);
                let image = &p.asset.images[p.asset.textures[material.base_texture.unwrap()].image];
                assert!(image.rgba8.as_chunks::<4>().0.iter().any(|p| p[3] == 0));
                assert!(image.rgba8.as_chunks::<4>().0.iter().any(|p| p[3] == 255));
                if kind == ScatterKind::LycopsidTree {
                    for socket in offline::plant_crown_attachments(form) {
                        let distance = leaf
                            .vertices
                            .iter()
                            .map(|v| {
                                glam::Vec3::from_array(v.position)
                                    .distance(glam::Vec3::from_array(socket))
                            })
                            .fold(f32::INFINITY, f32::min);
                        assert!(
                            distance < 0.13,
                            "form={form} detached crown socket={socket:?} gap={distance}"
                        );
                    }
                } else {
                    assert!(leaf.vertices.iter().all(|v| v.position[1] >= -0.0001));
                }
                assert!(
                    collect_shadow_lods(&p, false).is_empty(),
                    "retired position-only leaf LODs are inapplicable"
                );
            }
        }
    }
}
