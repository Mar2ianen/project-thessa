//! Frozen canonical coastal scene export for Blender Cycles reference rendering (B03).
use glam::{DVec3, Mat4, Vec3};
use serde_json::json;
use std::{error::Error, fs::File, io::BufWriter, path::Path};
use thessa_rcbt_core::lighting::SurfaceLighting;
use thessa_render_core::{
    ViewParams,
    mesh::{AlphaMode, Image, MeshAsset, Primitive},
};

pub struct CanonicalExportInput<'a> {
    pub terrain: Option<&'a Primitive>,
    pub plants_asset: &'a MeshAsset,
    pub instance_matrices: &'a [Vec<Mat4>],
    pub view: &'a ViewParams,
    pub lighting: &'a SurfaceLighting,
    pub camera_m: DVec3,
    pub fov_y_rad: f64,
    pub near_m: f64,
    pub far_m: f64,
    pub width: u32,
    pub height: u32,
}

fn encode_png(image: &Image) -> Result<Vec<u8>, Box<dyn Error>> {
    let mut png_bytes = Vec::new();
    let mut encoder = png::Encoder::new(&mut png_bytes, image.width, image.height);
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    encoder.write_header()?.write_image_data(&image.rgba8)?;
    Ok(png_bytes)
}

pub fn encode_canonical_glb(input: &CanonicalExportInput<'_>) -> Result<Vec<u8>, Box<dyn Error>> {
    let mut bin = Vec::new();
    let mut views = Vec::new();
    let mut accessors = Vec::new();
    let mut meshes = Vec::new();
    let mut nodes = Vec::new();

    let has_terrain = input.terrain.is_some();
    let mat_offset = if has_terrain { 1 } else { 0 };

    // 1. Terrain Mesh (if present)
    if let Some(terrain) = input.terrain {
        let v_start = bin.len();
        let min_pos: [f32; 3] = std::array::from_fn(|i| {
            terrain
                .vertices
                .iter()
                .map(|v| v.position[i])
                .fold(f32::INFINITY, f32::min)
        });
        let max_pos: [f32; 3] = std::array::from_fn(|i| {
            terrain
                .vertices
                .iter()
                .map(|v| v.position[i])
                .fold(f32::NEG_INFINITY, f32::max)
        });
        for v in &terrain.vertices {
            for n in v.position.into_iter().chain(v.normal).chain(v.uv) {
                bin.extend(n.to_le_bytes());
            }
        }
        let v_view = views.len();
        views.push(json!({
            "buffer": 0,
            "byteOffset": v_start,
            "byteLength": bin.len() - v_start,
            "byteStride": 32,
            "target": 34962
        }));

        let pos_acc = accessors.len();
        accessors.push(json!({
            "bufferView": v_view,
            "byteOffset": 0,
            "componentType": 5126,
            "count": terrain.vertices.len(),
            "type": "VEC3",
            "min": min_pos,
            "max": max_pos
        }));
        let norm_acc = accessors.len();
        accessors.push(json!({
            "bufferView": v_view,
            "byteOffset": 12,
            "componentType": 5126,
            "count": terrain.vertices.len(),
            "type": "VEC3"
        }));
        let uv_acc = accessors.len();
        accessors.push(json!({
            "bufferView": v_view,
            "byteOffset": 24,
            "componentType": 5126,
            "count": terrain.vertices.len(),
            "type": "VEC2"
        }));

        while !bin.len().is_multiple_of(4) {
            bin.push(0);
        }
        let idx_start = bin.len();
        for &idx in &terrain.indices {
            bin.extend(idx.to_le_bytes());
        }
        let idx_view = views.len();
        views.push(json!({
            "buffer": 0,
            "byteOffset": idx_start,
            "byteLength": bin.len() - idx_start,
            "target": 34963
        }));
        let idx_acc = accessors.len();
        accessors.push(json!({
            "bufferView": idx_view,
            "componentType": 5125,
            "count": terrain.indices.len(),
            "type": "SCALAR"
        }));

        meshes.push(json!({
            "name": "terrain_conforming",
            "primitives": [{
                "attributes": {
                    "POSITION": pos_acc,
                    "NORMAL": norm_acc,
                    "TEXCOORD_0": uv_acc
                },
                "indices": idx_acc,
                "material": 0,
                "mode": 4
            }]
        }));

        // Terrain node (mesh 0, identity matrix)
        nodes.push(json!({
            "name": "terrain",
            "mesh": 0
        }));
    }

    // 2. Botanical Prototypes
    let mut plant_mesh_indices = Vec::with_capacity(input.plants_asset.primitives.len());
    for (p_idx, p) in input.plants_asset.primitives.iter().enumerate() {
        while !bin.len().is_multiple_of(4) {
            bin.push(0);
        }
        let v_start = bin.len();
        for v in &p.vertices {
            for n in v.position.into_iter().chain(v.normal).chain(v.uv) {
                bin.extend(n.to_le_bytes());
            }
        }
        let v_view = views.len();
        views.push(json!({
            "buffer": 0,
            "byteOffset": v_start,
            "byteLength": bin.len() - v_start,
            "byteStride": 32,
            "target": 34962
        }));

        let min_pos: [f32; 3] = std::array::from_fn(|i| {
            p.vertices
                .iter()
                .map(|v| v.position[i])
                .fold(f32::INFINITY, f32::min)
        });
        let max_pos: [f32; 3] = std::array::from_fn(|i| {
            p.vertices
                .iter()
                .map(|v| v.position[i])
                .fold(f32::NEG_INFINITY, f32::max)
        });

        let pos_acc = accessors.len();
        accessors.push(json!({
            "bufferView": v_view,
            "byteOffset": 0,
            "componentType": 5126,
            "count": p.vertices.len(),
            "type": "VEC3",
            "min": min_pos,
            "max": max_pos
        }));
        let norm_acc = accessors.len();
        accessors.push(json!({
            "bufferView": v_view,
            "byteOffset": 12,
            "componentType": 5126,
            "count": p.vertices.len(),
            "type": "VEC3"
        }));
        let uv_acc = accessors.len();
        accessors.push(json!({
            "bufferView": v_view,
            "byteOffset": 24,
            "componentType": 5126,
            "count": p.vertices.len(),
            "type": "VEC2"
        }));

        while !bin.len().is_multiple_of(4) {
            bin.push(0);
        }
        let idx_start = bin.len();
        for &idx in &p.indices {
            bin.extend(idx.to_le_bytes());
        }
        let idx_view = views.len();
        views.push(json!({
            "buffer": 0,
            "byteOffset": idx_start,
            "byteLength": bin.len() - idx_start,
            "target": 34963
        }));
        let idx_acc = accessors.len();
        accessors.push(json!({
            "bufferView": idx_view,
            "componentType": 5125,
            "count": p.indices.len(),
            "type": "SCALAR"
        }));

        let mesh_id = meshes.len();
        meshes.push(json!({
            "name": format!("botany_prototype_{p_idx}"),
            "primitives": [{
                "attributes": {
                    "POSITION": pos_acc,
                    "NORMAL": norm_acc,
                    "TEXCOORD_0": uv_acc
                },
                "indices": idx_acc,
                "material": p.material + mat_offset,
                "mode": 4
            }]
        }));
        plant_mesh_indices.push(mesh_id);
    }

    // 3. Plant Instance Nodes
    for (p_idx, matrices) in input.instance_matrices.iter().enumerate() {
        if p_idx >= plant_mesh_indices.len() {
            break;
        }
        let mesh_id = plant_mesh_indices[p_idx];
        for (inst_idx, &mat) in matrices.iter().enumerate() {
            nodes.push(json!({
                "name": format!("plant_p{p_idx}_i{inst_idx}"),
                "mesh": mesh_id,
                "matrix": mat.to_cols_array()
            }));
        }
    }

    // 4. Images
    let mut gltf_images = Vec::with_capacity(input.plants_asset.images.len());
    for img in &input.plants_asset.images {
        while !bin.len().is_multiple_of(4) {
            bin.push(0);
        }
        let png = encode_png(img)?;
        let start = bin.len();
        let len = png.len();
        bin.extend(png);
        let view_idx = views.len();
        views.push(json!({
            "buffer": 0,
            "byteOffset": start,
            "byteLength": len
        }));
        gltf_images.push(json!({
            "bufferView": view_idx,
            "mimeType": "image/png"
        }));
    }

    // 5. Samplers & Textures
    let gltf_samplers = vec![json!({
        "magFilter": 9729,
        "minFilter": 9987,
        "wrapS": 10497,
        "wrapT": 10497
    })];

    let mut gltf_textures = Vec::with_capacity(input.plants_asset.textures.len());
    for tex in &input.plants_asset.textures {
        gltf_textures.push(json!({
            "sampler": 0,
            "source": tex.image
        }));
    }

    // 6. Materials
    let mut gltf_materials = Vec::new();
    if has_terrain {
        gltf_materials.push(json!({
            "name": "terrain_soil",
            "pbrMetallicRoughness": {
                "baseColorFactor": [0.18, 0.14, 0.10, 1.0],
                "metallicFactor": 0.0,
                "roughnessFactor": 0.85
            },
            "doubleSided": false,
            "alphaMode": "OPAQUE"
        }));
    }

    for (m_idx, mat) in input.plants_asset.materials.iter().enumerate() {
        let mut pbr = json!({
            "baseColorFactor": mat.base_color,
            "metallicFactor": mat.metallic,
            "roughnessFactor": mat.roughness
        });
        if let Some(tex) = mat.base_texture {
            pbr["baseColorTexture"] = json!({ "index": tex });
        }
        if let Some(tex) = mat.metallic_roughness_texture {
            pbr["metallicRoughnessTexture"] = json!({ "index": tex });
        }

        let mut mat_json = json!({
            "name": format!("plant_material_{m_idx}"),
            "pbrMetallicRoughness": pbr,
            "doubleSided": mat.double_sided,
            "alphaMode": match mat.alpha_mode {
                AlphaMode::Opaque => "OPAQUE",
                AlphaMode::Mask => "MASK",
                AlphaMode::Blend => "BLEND",
            }
        });

        if mat.alpha_mode == AlphaMode::Mask {
            mat_json["alphaCutoff"] = json!(mat.alpha_cutoff);
        }

        if let Some(tex) = mat.normal_texture {
            mat_json["normalTexture"] = json!({
                "index": tex,
                "scale": mat.normal_scale
            });
        }

        if mat.diffuse_transmission > 0.0 {
            mat_json["extensions"] = json!({
                "KHR_materials_transmission": {
                    "transmissionFactor": mat.diffuse_transmission
                }
            });
        }

        gltf_materials.push(mat_json);
    }

    let scene_nodes: Vec<usize> = (0..nodes.len()).collect();
    let document = json!({
        "asset": {
            "version": "2.0",
            "generator": "Thessa Canonical Coastal Freeze (B03) v1",
            "copyright": "Project Thessa"
        },
        "extensionsUsed": ["KHR_materials_transmission"],
        "scene": 0,
        "scenes": [{ "nodes": scene_nodes }],
        "nodes": nodes,
        "meshes": meshes,
        "materials": gltf_materials,
        "textures": gltf_textures,
        "images": gltf_images,
        "samplers": gltf_samplers,
        "buffers": [{ "byteLength": bin.len() }],
        "bufferViews": views,
        "accessors": accessors
    });

    let mut json_bytes = serde_json::to_vec(&document)?;
    while !json_bytes.len().is_multiple_of(4) {
        json_bytes.push(b' ');
    }
    while !bin.len().is_multiple_of(4) {
        bin.push(0);
    }

    let total_len = 12 + 8 + json_bytes.len() + 8 + bin.len();
    let mut glb = Vec::with_capacity(total_len);
    glb.extend(0x46546c67_u32.to_le_bytes()); // "glTF"
    glb.extend(2_u32.to_le_bytes()); // version 2
    glb.extend((total_len as u32).to_le_bytes());

    // JSON chunk
    glb.extend((json_bytes.len() as u32).to_le_bytes());
    glb.extend(0x4e4f534a_u32.to_le_bytes()); // "JSON"
    glb.extend(json_bytes);

    // BIN chunk
    glb.extend((bin.len() as u32).to_le_bytes());
    glb.extend(0x004e4942_u32.to_le_bytes()); // "BIN\0"
    glb.extend(bin);

    Ok(glb)
}

pub fn export_canonical_scene(
    directory: &Path,
    input: &CanonicalExportInput<'_>,
    display_pixels: Option<&[u8]>,
    linear_hdr_bytes: Option<&[u8]>,
) -> Result<(), Box<dyn Error>> {
    std::fs::create_dir_all(directory)?;

    let glb = encode_canonical_glb(input)?;
    std::fs::write(directory.join("scene.glb"), glb)?;

    let forward = -input.view.world_from_view.z_axis.truncate().normalize();
    let up = input.view.world_from_view.y_axis.truncate().normalize();
    let right = input.view.world_from_view.x_axis.truncate().normalize();
    let sun_dir = Vec3::from_slice(&input.lighting.directions[0][..3]);

    let terrain_triangles = input.terrain.map_or(0, |t| t.indices.len() / 3);
    let total_plant_instances: usize = input.instance_matrices.iter().map(|m| m.len()).sum();
    let total_plant_triangles: usize = input
        .plants_asset
        .primitives
        .iter()
        .zip(input.instance_matrices)
        .map(|(p, insts)| (p.indices.len() / 3) * insts.len())
        .sum();

    let scene_json = json!({
        "schema_version": 1,
        "id": "B03-canonical-coast-v1",
        "name": "Thessa Canonical Coastal World Scene",
        "geometry": "scene.glb",
        "coordinate_system": "glTF 2.0 right-handed Y-up (camera-relative origin)",
        "camera": {
            "eye_m": [0.0, 0.0, 0.0],
            "camera_body_m": [input.camera_m.x, input.camera_m.y, input.camera_m.z],
            "forward": [forward.x, forward.y, forward.z],
            "up": [up.x, up.y, up.z],
            "right": [right.x, right.y, right.z],
            "fov_y_rad": input.fov_y_rad,
            "aspect_ratio": input.width as f64 / input.height as f64,
            "near_m": input.near_m,
            "far_m": input.far_m,
            "exposure": input.view.exposure
        },
        "sun": {
            "direction_to_light": [sun_dir.x, sun_dir.y, sun_dir.z],
            "illuminance_rgb_lux": [
                input.lighting.colors_lux[0][0],
                input.lighting.colors_lux[0][1],
                input.lighting.colors_lux[0][2]
            ]
        },
        "ambient": {
            "ambient_rgb_lux": [
                input.lighting.ambient_lux[0],
                input.lighting.ambient_lux[1],
                input.lighting.ambient_lux[2]
            ]
        },
        "stats": {
            "terrain_triangles": terrain_triangles,
            "plant_instances": total_plant_instances,
            "plant_triangles": total_plant_triangles,
            "plant_prototypes": input.plants_asset.primitives.len()
        }
    });

    std::fs::write(
        directory.join("scene.json"),
        serde_json::to_string_pretty(&scene_json)?,
    )?;

    std::fs::write(
        directory.join("scene.LICENSE.txt"),
        "Canonical Thessa coastal world scene geometry and textures.\nMIT License / CC0 1.0 Universal.\n",
    )?;

    if let Some(pixels) = display_pixels {
        let path = directory.join("display.png");
        let mut png = png::Encoder::new(
            BufWriter::new(File::create(path)?),
            input.width,
            input.height,
        );
        png.set_color(png::ColorType::Rgba);
        png.set_depth(png::BitDepth::Eight);
        png.write_header()?.write_image_data(pixels)?;
    }

    if let Some(bytes) = linear_hdr_bytes {
        std::fs::write(directory.join("linear.rgba16f"), bytes)?;
    }

    eprintln!(
        "canonical_scene_exported directory={} terrain_triangles={} plant_instances={} plant_triangles={}",
        directory.display(),
        terrain_triangles,
        total_plant_instances,
        total_plant_triangles
    );

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::{DMat4, Mat4};
    use thessa_render_core::mesh::{Material, Texture, Vertex, Wrap};

    fn triangle(material: usize) -> Primitive {
        Primitive {
            ambient_visibility: vec![],
            vertices: vec![
                Vertex {
                    position: [0.0, 0.0, 0.0],
                    normal: [0.0, 0.0, 1.0],
                    uv: [0.0, 0.0],
                },
                Vertex {
                    position: [1.0, 0.0, 0.0],
                    normal: [0.0, 0.0, 1.0],
                    uv: [1.0, 0.0],
                },
                Vertex {
                    position: [0.0, 1.0, 0.0],
                    normal: [0.0, 0.0, 1.0],
                    uv: [0.0, 1.0],
                },
            ],
            indices: vec![0, 1, 2],
            material,
        }
    }

    #[test]
    fn canonical_glb_embeds_terrain_plants_textures_and_cutoffs() {
        let terrain = triangle(0);
        let plants = MeshAsset {
            primitives: vec![triangle(0)],
            materials: vec![Material {
                alpha_mode: AlphaMode::Mask,
                alpha_cutoff: 0.3,
                base_texture: Some(0),
                double_sided: true,
                metallic: 0.0,
                roughness: 0.6,
                ..Default::default()
            }],
            textures: vec![Texture {
                image: 0,
                wrap: [Wrap::Repeat; 2],
                linear: false,
                min_linear: true,
            }],
            images: vec![Image {
                width: 1,
                height: 1,
                rgba8: vec![255, 128, 64, 255],
            }],
        };
        let instances = vec![vec![Mat4::IDENTITY]];
        let view = ViewParams {
            world_from_view: DMat4::IDENTITY,
            clip_from_view: DMat4::IDENTITY,
            world_position_m: DVec3::ZERO,
            exposure: 1.0,
        };
        let glb = encode_canonical_glb(&CanonicalExportInput {
            terrain: Some(&terrain),
            plants_asset: &plants,
            instance_matrices: &instances,
            view: &view,
            lighting: &SurfaceLighting::default(),
            camera_m: DVec3::ZERO,
            fov_y_rad: 0.8,
            near_m: 0.1,
            far_m: 100.0,
            width: 16,
            height: 9,
        })
        .unwrap();

        assert_eq!(&glb[..4], b"glTF");
        let word = |offset: usize| {
            u32::from_le_bytes(glb[offset..offset + 4].try_into().unwrap()) as usize
        };
        assert_eq!(word(4), 2);
        assert_eq!(word(8), glb.len());
        let json_len = word(12);
        assert_eq!(word(16), 0x4e4f534a);
        let document: serde_json::Value = serde_json::from_slice(&glb[20..20 + json_len]).unwrap();
        assert_eq!(document["asset"]["version"], "2.0");
        assert_eq!(document["meshes"].as_array().unwrap().len(), 2);
        assert_eq!(document["nodes"].as_array().unwrap().len(), 2);
        assert_eq!(document["materials"].as_array().unwrap().len(), 2);
        assert_eq!(document["materials"][1]["alphaMode"], "MASK");
        assert!((document["materials"][1]["alphaCutoff"].as_f64().unwrap() - 0.3).abs() < 1e-6);
        assert_eq!(document["materials"][1]["doubleSided"], true);
        assert_eq!(document["textures"].as_array().unwrap().len(), 1);
        assert_eq!(document["images"].as_array().unwrap().len(), 1);
        assert_eq!(document["accessors"].as_array().unwrap().len(), 8);
        assert_eq!(document["nodes"][0]["name"], "terrain");
        assert_eq!(document["nodes"][1]["name"], "plant_p0_i0");
        assert_eq!(document["nodes"][1]["matrix"].as_array().unwrap().len(), 16);
        let bin_len = word(20 + json_len);
        assert_eq!(bin_len % 4, 0);
        assert!(bin_len > 0);
    }
}
