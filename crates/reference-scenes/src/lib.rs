//! Frozen reference inputs, independent of Thessa simulation/worldgen/backends.
#![forbid(unsafe_code)]
use glam::DVec3;
use serde_json::{Value, json};
use thessa_render_core::{
    ViewParams,
    mesh::{AlphaMode, Material, MeshAsset, Primitive, Vertex},
};

pub const B01_ID: &str = "B01-material-lab-v1";
pub const B02_ID: &str = "B02-vegetation-v1";
pub const B03_ID: &str = "B03-canonical-coast-v1";
pub const B03_SURFACE_SHOWCASE_LAT_DEG: f64 = 23.565679258039204;
pub const B03_SURFACE_SHOWCASE_LON_DEG: f64 = 165.13881581863882;
pub const B03_SURFACE_SHOWCASE_DISTANCE_M: f64 = 80.0;
pub const B03_SURFACE_SHOWCASE_EYE_AGL_M: f64 = 2.5;
pub const B02_FERN_02_SHA256: &str =
    "10025cc15e9067b430e5be4b2628a06ac0c7f8acc49d5b2f61dddeda08a48b26";
pub const GEOMETRY_LICENSE_NOTICE: &str = include_str!("../LICENSE");
pub const B02_CC0_LICENSE_NOTICE: &str =
    include_str!("../../../data/reference-scenes/licenses/CC0-1.0.txt");
const SLICES: usize = 48;
const STACKS: usize = 24;
const BASE_COLOR: [f32; 4] = [0.8, 0.4, 0.12, 1.0];
const PLANE_COLOR: [f32; 4] = [0.18, 0.18, 0.18, 1.0];
const EYE_M: [f64; 3] = [0.0, 10.0, 18.0];
const TARGET_M: [f64; 3] = [0.0, 0.7, 0.0];
const FOV_Y_RAD: f64 = 50_f64.to_radians();
const NEAR_M: f64 = 0.1;
const FAR_M: f64 = 100.0;
pub const ROUGHNESS: [f32; 5] = [0.05, 0.2, 0.4, 0.7, 1.0];

/// Immutable mathematical material lab. Source geometry, materials and portable
/// GLB are produced together; no asynchronous residency or simulation clock.
pub struct MaterialLab {
    asset: MeshAsset,
    glb: Vec<u8>,
}
impl Default for MaterialLab {
    fn default() -> Self {
        Self::new()
    }
}
impl MaterialLab {
    pub fn new() -> Self {
        let mut materials = Vec::new();
        let mut primitives = Vec::new();
        for (row, metallic) in [0.0, 1.0].into_iter().enumerate() {
            for (column, roughness) in ROUGHNESS.into_iter().enumerate() {
                let material = materials.len();
                materials.push(Material {
                    base_color: BASE_COLOR,
                    metallic,
                    roughness,
                    ..Default::default()
                });
                primitives.push(sphere(
                    [(column as f32 - 2.0) * 2.8, 1.0, (row as f32 - 0.5) * 4.8],
                    material,
                ));
            }
        }
        let material = materials.len();
        materials.push(Material {
            base_color: PLANE_COLOR,
            metallic: 0.0,
            roughness: 0.8,
            ..Default::default()
        });
        primitives.push(Primitive {
            ambient_visibility: vec![],
            material,
            vertices: [
                [-9.0, 0.0, -7.0],
                [9.0, 0.0, -7.0],
                [9.0, 0.0, 7.0],
                [-9.0, 0.0, 7.0],
            ]
            .into_iter()
            .enumerate()
            .map(|(i, position)| Vertex {
                position,
                normal: [0.0, 1.0, 0.0],
                uv: [(i == 1 || i == 2) as u8 as f32, (i >= 2) as u8 as f32],
            })
            .collect(),
            indices: vec![0, 2, 1, 0, 3, 2],
        });
        let asset = MeshAsset {
            primitives,
            materials,
            textures: vec![],
            images: vec![],
        };
        debug_assert!(asset.validate().is_ok());
        let glb = encode_lab_glb(&asset);
        Self { asset, glb }
    }
    pub fn asset(&self) -> &MeshAsset {
        &self.asset
    }
    pub fn glb(&self) -> &[u8] {
        &self.glb
    }
    pub fn direction_to_light(&self) -> [f32; 3] {
        DVec3::new(-0.5, 1.0, 0.75).normalize().as_vec3().to_array()
    }
    pub fn illuminance_rgb_lux(&self) -> [f32; 3] {
        [10_000.0; 3]
    }
    pub fn view(&self, extent: [u32; 2], exposure: f32) -> Option<ViewParams> {
        if extent.contains(&0) || !exposure.is_finite() || exposure <= 0.0 {
            return None;
        }
        let eye = DVec3::from_array(EYE_M);
        Some(ViewParams {
            world_from_view: glam::dcamera::rh::view::look_at_mat4(
                eye,
                DVec3::from_array(TARGET_M),
                DVec3::Y,
            )
            .inverse(),
            clip_from_view: glam::dcamera::rh::proj::directx::perspective(
                FOV_Y_RAD,
                f64::from(extent[0]) / f64::from(extent[1]),
                NEAR_M,
                FAR_M,
            ),
            world_position_m: eye,
            exposure,
        })
    }
    pub fn description(&self) -> Value {
        json!({"schema_version":1,"id":B01_ID,"geometry":"scene.glb","units":"metres","coordinate_system":"right-handed, Y-up",
            "sphere_radius_m":1.0,"sphere_slices":SLICES,"sphere_stacks":STACKS,"metallic_rows":[0.0,1.0],"roughness_columns":ROUGHNESS,
            "base_color_linear_rgba":BASE_COLOR,"plane_base_color_linear_rgba":PLANE_COLOR,"plane_roughness":0.8,"plane_metallic":0.0,"dielectric_f0":0.04,
            "camera_eye_m":EYE_M,"camera_target_m":TARGET_M,"camera_up":[0.0,1.0,0.0],"fov_y_rad":FOV_Y_RAD,"near_m":NEAR_M,"far_m":FAR_M,
            "direction_to_light":self.direction_to_light(),"illuminance_rgb_lux":self.illuminance_rgb_lux(),"ambient_rgb_lux":[0.0,0.0,0.0],
            "simulation_time_s":0.0,"environment":"none","background_linear_rgba":[0.0,0.0,0.0,1.0],
            "generator":"thessa-reference-scenes B01 v1","third_party_assets":[],"geometry_license":"MIT"})
    }
}

const B02_EYE_M: [f64; 3] = [0.46, 1.6, 2.8];
const B02_TARGET_M: [f64; 3] = [0.46, 0.18, 0.44];
const B02_FOV_Y_RAD: f64 = 50_f64.to_radians();
const B02_NEAR_M: f64 = 0.05;
const B02_FAR_M: f64 = 50.0;

/// Frozen B02 vegetation scene parameters using pinned Poly Haven Fern 02 assets.
pub struct VegetationLab;

impl Default for VegetationLab {
    fn default() -> Self {
        Self::new()
    }
}

impl VegetationLab {
    pub fn new() -> Self {
        Self
    }
    pub fn direction_to_light(&self) -> [f32; 3] {
        DVec3::new(-0.5, 1.0, 0.75).normalize().as_vec3().to_array()
    }
    pub fn illuminance_rgb_lux(&self) -> [f32; 3] {
        [10_000.0; 3]
    }
    pub fn view(&self, extent: [u32; 2], exposure: f32) -> Option<ViewParams> {
        if extent.contains(&0) || !exposure.is_finite() || exposure <= 0.0 {
            return None;
        }
        let eye = DVec3::from_array(B02_EYE_M);
        Some(ViewParams {
            world_from_view: glam::dcamera::rh::view::look_at_mat4(
                eye,
                DVec3::from_array(B02_TARGET_M),
                DVec3::Y,
            )
            .inverse(),
            clip_from_view: glam::dcamera::rh::proj::directx::perspective(
                B02_FOV_Y_RAD,
                f64::from(extent[0]) / f64::from(extent[1]),
                B02_NEAR_M,
                B02_FAR_M,
            ),
            world_position_m: eye,
            exposure,
        })
    }
    pub fn description(&self, asset: &MeshAsset) -> Value {
        let triangles = asset
            .primitives
            .iter()
            .map(|p| p.indices.len() / 3)
            .sum::<usize>();
        let used_materials: Vec<&Material> = {
            let mut used = std::collections::BTreeSet::new();
            for primitive in &asset.primitives {
                used.insert(primitive.material);
            }
            used.into_iter()
                .filter_map(|index| asset.materials.get(index))
                .collect()
        };
        let alpha_mode = if used_materials
            .iter()
            .any(|m| m.alpha_mode == AlphaMode::Blend)
        {
            "BLEND"
        } else if used_materials
            .iter()
            .any(|m| m.alpha_mode == AlphaMode::Mask)
        {
            "MASK"
        } else {
            "OPAQUE"
        };
        let cutoffs: Vec<f32> = used_materials
            .iter()
            .filter(|m| m.alpha_mode == AlphaMode::Mask)
            .map(|m| m.alpha_cutoff)
            .collect();
        let alpha_cutoff = if cutoffs.is_empty() || !cutoffs.windows(2).all(|w| w[0] == w[1]) {
            Value::Null
        } else {
            json!(cutoffs[0])
        };
        let double_sided =
            !used_materials.is_empty() && used_materials.iter().all(|m| m.double_sided);
        json!({
            "schema_version": 1,
            "id": B02_ID,
            "geometry": "scene.glb",
            "units": "metres",
            "coordinate_system": "right-handed, Y-up",
            "asset_id": "vegetation-fern-02-core-cutout-v1",
            "asset_source": "https://polyhaven.com/a/fern_02",
            "triangles": triangles,
            "mesh_objects": asset.primitives.len(),
            "alpha_mode": alpha_mode,
            "alpha_cutoff": alpha_cutoff,
            "double_sided": double_sided,
            "camera_eye_m": B02_EYE_M,
            "camera_target_m": B02_TARGET_M,
            "camera_up": [0.0, 1.0, 0.0],
            "fov_y_rad": B02_FOV_Y_RAD,
            "near_m": B02_NEAR_M,
            "far_m": B02_FAR_M,
            "direction_to_light": self.direction_to_light(),
            "illuminance_rgb_lux": self.illuminance_rgb_lux(),
            "ambient_rgb_lux": [0.0, 0.0, 0.0],
            "simulation_time_s": 0.0,
            "environment": "none",
            "background_linear_rgba": [0.0, 0.0, 0.0, 1.0],
            "generator": "thessa-reference-scenes B02 v1",
            "third_party_assets": ["vegetation-fern-02-core-cutout-v1"],
            "geometry_license": "CC0-1.0"
        })
    }
}

fn sphere(center: [f32; 3], material: usize) -> Primitive {
    let mut vertices = Vec::new();
    for y in 0..=STACKS {
        let theta = std::f64::consts::PI * y as f64 / STACKS as f64;
        for x in 0..=SLICES {
            let phi = std::f64::consts::TAU * (x % SLICES) as f64 / SLICES as f64;
            let normal = if y == 0 {
                [0.0, 1.0, 0.0]
            } else if y == STACKS {
                [0.0, -1.0, 0.0]
            } else {
                [
                    (theta.sin() * phi.cos()) as f32,
                    theta.cos() as f32,
                    (theta.sin() * phi.sin()) as f32,
                ]
            };
            vertices.push(Vertex {
                position: std::array::from_fn(|i| center[i] + normal[i]),
                normal,
                uv: [x as f32 / SLICES as f32, y as f32 / STACKS as f32],
            });
        }
    }
    let mut indices = Vec::new();
    for y in 0..STACKS {
        for x in 0..SLICES {
            let a = (y * (SLICES + 1) + x) as u32;
            let b = a + 1;
            let c = a + (SLICES + 1) as u32;
            let d = c + 1;
            if y > 0 {
                indices.extend([a, b, c]);
            }
            if y + 1 < STACKS {
                indices.extend([b, d, c]);
            }
        }
    }
    Primitive {
        vertices,
        indices,
        material,
        ambient_visibility: vec![],
    }
}

// Only the private generated opaque, untextured lab enters this encoder.
// General imported materials must use their verified original asset files.
fn encode_lab_glb(asset: &MeshAsset) -> Vec<u8> {
    let mut bin = Vec::new();
    let mut views = Vec::new();
    let mut accessors = Vec::new();
    let mut meshes = Vec::new();
    let mut nodes = Vec::new();
    for p in &asset.primitives {
        let start = bin.len();
        for v in &p.vertices {
            for n in v.position.into_iter().chain(v.normal).chain(v.uv) {
                bin.extend(n.to_le_bytes());
            }
        }
        let view = views.len();
        views.push(json!({"buffer":0,"byteOffset":start,"byteLength":bin.len()-start,"byteStride":32,"target":34962}));
        let first = accessors.len();
        let min: [f32; 3] = std::array::from_fn(|i| {
            p.vertices
                .iter()
                .map(|v| v.position[i])
                .fold(f32::INFINITY, f32::min)
        });
        let max: [f32; 3] = std::array::from_fn(|i| {
            p.vertices
                .iter()
                .map(|v| v.position[i])
                .fold(f32::NEG_INFINITY, f32::max)
        });
        accessors.push(json!({"bufferView":view,"byteOffset":0,"componentType":5126,"count":p.vertices.len(),"type":"VEC3","min":min,"max":max}));
        accessors.push(json!({"bufferView":view,"byteOffset":12,"componentType":5126,"count":p.vertices.len(),"type":"VEC3"}));
        accessors.push(json!({"bufferView":view,"byteOffset":24,"componentType":5126,"count":p.vertices.len(),"type":"VEC2"}));
        let start = bin.len();
        for i in &p.indices {
            bin.extend(i.to_le_bytes());
        }
        let view = views.len();
        views.push(
            json!({"buffer":0,"byteOffset":start,"byteLength":bin.len()-start,"target":34963}),
        );
        accessors.push(
            json!({"bufferView":view,"componentType":5125,"count":p.indices.len(),"type":"SCALAR"}),
        );
        nodes.push(json!({"mesh":meshes.len()}));
        meshes.push(json!({"primitives":[{"attributes":{"POSITION":first,"NORMAL":first+1,"TEXCOORD_0":first+2},"indices":first+3,"material":p.material,"mode":4}]}));
    }
    let materials:Vec<_>=asset.materials.iter().map(|m|json!({"pbrMetallicRoughness":{"baseColorFactor":m.base_color,"metallicFactor":m.metallic,"roughnessFactor":m.roughness},"alphaMode":"OPAQUE","doubleSided":false})).collect();
    let mut document = json!({"asset":{"version":"2.0","generator":"thessa-reference-scenes B01 v1","copyright":"Project Thessa; generated mathematical geometry; MIT"},"scene":0,"scenes":[{"nodes":(0..nodes.len()).collect::<Vec<_>>()}],"nodes":nodes,"meshes":meshes,"materials":materials,"buffers":[{"byteLength":bin.len()}],"bufferViews":views,"accessors":accessors});
    document.sort_all_objects();
    let mut json = serde_json::to_vec(&document).expect("finite generated glTF");
    while !json.len().is_multiple_of(4) {
        json.push(b' ');
    }
    while !bin.len().is_multiple_of(4) {
        bin.push(0);
    }
    let mut glb = Vec::new();
    for word in [
        0x46546c67,
        2,
        (12 + 8 + json.len() + 8 + bin.len()) as u32,
        json.len() as u32,
        0x4e4f534a,
    ] {
        glb.extend(word.to_le_bytes());
    }
    glb.extend(json);
    glb.extend((bin.len() as u32).to_le_bytes());
    glb.extend(0x004e4942_u32.to_le_bytes());
    glb.extend(bin);
    glb
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn lab_geometry_has_outward_nonzero_faces_and_explicit_material_grid() {
        let lab = MaterialLab::new();
        let asset = lab.asset();
        asset.validate().unwrap();
        assert_eq!(asset.primitives.len(), 11);
        assert_eq!(asset.materials.len(), 11);
        assert_eq!(
            asset
                .primitives
                .iter()
                .map(|p| p.indices.len() / 3)
                .sum::<usize>(),
            22_082
        );
        for (i, p) in asset.primitives.iter().enumerate() {
            if i < 10 {
                assert_eq!(asset.materials[p.material].roughness, ROUGHNESS[i % 5]);
                assert_eq!(asset.materials[p.material].metallic, (i / 5) as f32);
            }
            for v in &p.vertices {
                assert!((glam::Vec3::from_array(v.normal).length() - 1.0).abs() < 2e-7);
            }
            for face in p.indices.as_chunks::<3>().0 {
                let a = glam::Vec3::from_array(p.vertices[face[0] as usize].position);
                let b = glam::Vec3::from_array(p.vertices[face[1] as usize].position);
                let c = glam::Vec3::from_array(p.vertices[face[2] as usize].position);
                assert!(
                    (b - a)
                        .cross(c - a)
                        .dot(glam::Vec3::from_array(p.vertices[face[0] as usize].normal))
                        > 1e-7
                );
            }
        }
        assert!(lab.view([0, 100], 1.0).is_none());
        assert!(lab.view([100, 100], f32::NAN).is_none());
        assert!(
            lab.view([640, 480], 1.0)
                .unwrap()
                .clip_from_view
                .is_finite()
        );
    }
    #[test]
    fn portable_lab_is_repeatable_and_round_trips_through_real_decoder() {
        let lab = MaterialLab::new();
        assert_eq!(lab.glb(), MaterialLab::new().glb());
        let decoded = thessa_render_assets::decode_glb(lab.glb(), Default::default()).unwrap();
        // The existing decoder appends one unreferenced glTF default material.
        assert_eq!(decoded.materials.len(), lab.asset().materials.len() + 1);
        assert_eq!(decoded.primitives.len(), lab.asset().primitives.len());
        // The decoder traverses scene roots in reverse stack order. Compare
        // semantic material bindings, not an incidental flattened draw order.
        for b in &lab.asset().primitives {
            let a = decoded
                .primitives
                .iter()
                .find(|p| p.material == b.material)
                .unwrap();
            assert_eq!(a.vertices.len(), b.vertices.len());
            for (av, bv) in a.vertices.iter().zip(&b.vertices) {
                assert_eq!(av.position, bv.position);
                assert_eq!(av.uv, bv.uv);
                assert!(
                    (glam::Vec3::from_array(av.normal) - glam::Vec3::from_array(bv.normal))
                        .length()
                        < 2e-7,
                    "decoder unit-normal rounding must remain bounded"
                );
            }
            assert_eq!(a.indices, b.indices);
            assert_eq!(a.material, b.material);
        }
        for (a, b) in decoded.materials.iter().zip(&lab.asset().materials) {
            assert_eq!(a.base_color, b.base_color);
            assert_eq!(a.metallic, b.metallic);
            assert_eq!(a.roughness, b.roughness);
        }
    }
    #[test]
    fn b02_vegetation_lab_definition_and_view_are_valid() {
        let lab = VegetationLab::new();
        let asset = MeshAsset {
            primitives: vec![Primitive {
                vertices: vec![
                    Vertex {
                        position: [-1.0, 0.0, 0.0],
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
                material: 0,
                ambient_visibility: vec![],
            }],
            materials: vec![Material {
                alpha_mode: AlphaMode::Mask,
                alpha_cutoff: 0.5,
                double_sided: true,
                ..Default::default()
            }],
            textures: vec![],
            images: vec![],
        };
        let desc = lab.description(&asset);
        assert_eq!(desc["id"], B02_ID);
        assert_eq!(desc["triangles"], 1);
        assert_eq!(desc["mesh_objects"], 1);
        assert_eq!(desc["alpha_mode"], "MASK");
        assert_eq!(desc["alpha_cutoff"], 0.5);
        assert_eq!(desc["double_sided"], true);
        assert!(lab.view([0, 100], 1.0).is_none());
        assert!(lab.view([100, 100], f32::NAN).is_none());
        let view = lab.view([640, 480], 1.0).unwrap();
        assert!(view.clip_from_view.is_finite());
        assert!(view.world_from_view.is_finite());
        assert_eq!(B02_FERN_02_SHA256.len(), 64);
        assert!(!B02_CC0_LICENSE_NOTICE.is_empty());
    }
}
