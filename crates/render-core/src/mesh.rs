//! Static triangle sources and rigid instances; no asset system/backend types.
use glam::{DMat4, DQuat, DVec3};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    pub uv: [f32; 2],
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Wrap {
    Clamp,
    Repeat,
    Mirror,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Texture {
    pub image: usize,
    pub wrap: [Wrap; 2],
    pub linear: bool,
    pub min_linear: bool,
}
#[derive(Debug, Clone)]
pub struct Image {
    pub width: u32,
    pub height: u32,
    pub rgba8: Vec<u8>,
}
impl Image {
    /// Area-filtered mip chain. Base-colour RGB is averaged in linear light;
    /// alpha and data maps are linear. Normal maps normalize after sampling.
    pub fn mip_chain(&self, srgb: bool) -> Option<Vec<Image>> {
        if self.width == 0
            || self.height == 0
            || u64::from(self.width)
                .checked_mul(u64::from(self.height))
                .and_then(|n| n.checked_mul(4))
                != Some(self.rgba8.len() as u64)
        {
            return None;
        }
        let mut levels = vec![self.clone()];
        while levels.last().unwrap().width > 1 || levels.last().unwrap().height > 1 {
            let source = levels.last().unwrap();
            let width = (source.width / 2).max(1);
            let height = (source.height / 2).max(1);
            let mut rgba8 = Vec::with_capacity(width as usize * height as usize * 4);
            for y in 0..height {
                for x in 0..width {
                    let x0 = (f64::from(x) * f64::from(source.width)) / f64::from(width);
                    let x1 = (f64::from(x + 1) * f64::from(source.width)) / f64::from(width);
                    let y0 = (f64::from(y) * f64::from(source.height)) / f64::from(height);
                    let y1 = (f64::from(y + 1) * f64::from(source.height)) / f64::from(height);
                    let mut sum = [0.0; 4];
                    for sy in y0.floor() as u32..y1.ceil() as u32 {
                        for sx in x0.floor() as u32..x1.ceil() as u32 {
                            let weight = (x1.min(f64::from(sx + 1)) - x0.max(f64::from(sx)))
                                * (y1.min(f64::from(sy + 1)) - y0.max(f64::from(sy)));
                            for (c, out) in sum.iter_mut().enumerate() {
                                let value = f64::from(
                                    source.rgba8[((sy * source.width + sx) * 4) as usize + c],
                                ) / 255.0;
                                let value = if srgb && c < 3 {
                                    if value <= 0.04045 {
                                        value / 12.92
                                    } else {
                                        ((value + 0.055) / 1.055).powf(2.4)
                                    }
                                } else {
                                    value
                                };
                                *out += weight * value;
                            }
                        }
                    }
                    for (c, value) in sum.into_iter().enumerate() {
                        let value = value / ((x1 - x0) * (y1 - y0));
                        let value = if srgb && c < 3 {
                            if value <= 0.0031308 {
                                value * 12.92
                            } else {
                                1.055 * value.powf(1.0 / 2.4) - 0.055
                            }
                        } else {
                            value
                        };
                        rgba8.push((value.clamp(0.0, 1.0) * 255.0).round() as u8);
                    }
                }
            }
            levels.push(Image {
                width,
                height,
                rgba8,
            });
        }
        Some(levels)
    }
}

#[derive(Debug, Clone)]
pub struct Material {
    pub base_color: [f32; 4],
    pub base_texture: Option<usize>,
    pub normal_texture: Option<usize>,
    pub normal_scale: f32,
    pub metallic_roughness_texture: Option<usize>,
    pub metallic: f32,
    pub roughness: f32,
    pub occlusion_texture: Option<usize>,
    pub occlusion_strength: f32,
    pub double_sided: bool,
    /// Fraction of diffuse scattering leaving the opposite hemisphere.
    /// Reflectance = albedo * (1-t), transmittance = albedo * t.
    pub diffuse_transmission: f32,
    /// Normal-incidence dielectric Fresnel reflectance; water is about 0.0204.
    pub dielectric_f0: f32,
}
impl Default for Material {
    fn default() -> Self {
        Self {
            base_color: [1.0; 4],
            base_texture: None,
            normal_texture: None,
            normal_scale: 1.0,
            metallic_roughness_texture: None,
            metallic: 1.0,
            roughness: 1.0,
            occlusion_texture: None,
            occlusion_strength: 1.0,
            double_sided: false,
            diffuse_transmission: 0.0,
            dielectric_f0: 0.04,
        }
    }
}
#[derive(Debug, Clone)]
pub struct Primitive {
    /// Optional baked scalar ambient visibility, independent of material UVs.
    /// Empty means one at every vertex; historical/imported assets retain it.
    pub ambient_visibility: Vec<f32>,
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
    pub material: usize,
}
/// Optional depth-only replacement for one primitive. The producer certifies
/// a two-sided continuous surface-distance bound in local metres. Primary and
/// ray-query geometry continue to use the original MeshAsset. Consumers may
/// supply several distinct triangle counts for one primitive and select the
/// smallest representation satisfying the projected error budget.
#[derive(Debug, Clone, PartialEq)]
pub struct ShadowMeshLod {
    pub primitive: usize,
    pub positions: Vec<[f32; 3]>,
    pub indices: Vec<u32>,
    pub max_error_m: f64,
}

impl ShadowMeshLod {
    pub fn same_geometry(&self, other: &Self) -> bool {
        self.positions == other.positions && self.indices == other.indices
    }
    pub fn geometry_bytes(&self) -> usize {
        self.positions.len() * 12 + self.indices.len() * 4
    }
}

#[derive(Debug, Clone)]
pub struct MeshAsset {
    pub primitives: Vec<Primitive>,
    pub materials: Vec<Material>,
    pub textures: Vec<Texture>,
    pub images: Vec<Image>,
}
impl MeshAsset {
    /// Reject malformed data before GPU/host asset allocation. This validates
    /// semantic references and numeric data, not resource policy budgets.
    pub fn validate(&self) -> Result<(), &'static str> {
        if self.primitives.is_empty() {
            return Err("empty mesh asset");
        }
        for image in &self.images {
            if image.width == 0
                || image.height == 0
                || u64::from(image.width)
                    .checked_mul(u64::from(image.height))
                    .and_then(|n| n.checked_mul(4))
                    != Some(image.rgba8.len() as u64)
            {
                return Err("invalid mesh image extent/payload");
            }
        }
        if self.textures.iter().any(|t| t.image >= self.images.len()) {
            return Err("invalid mesh image reference");
        }
        for material in &self.materials {
            if [
                material.base_texture,
                material.normal_texture,
                material.metallic_roughness_texture,
                material.occlusion_texture,
            ]
            .into_iter()
            .flatten()
            .any(|t| t >= self.textures.len())
            {
                return Err("invalid mesh texture reference");
            }
            if material
                .base_color
                .iter()
                .any(|v| !v.is_finite() || !(0.0..=1.0).contains(v))
                || [
                    material.metallic,
                    material.roughness,
                    material.occlusion_strength,
                    material.diffuse_transmission,
                    material.dielectric_f0,
                ]
                .into_iter()
                .any(|v| !v.is_finite() || !(0.0..=1.0).contains(&v))
                || !material.normal_scale.is_finite()
            {
                return Err("invalid mesh material factors");
            }
        }
        for p in &self.primitives {
            if p.material >= self.materials.len()
                || p.vertices.is_empty()
                || p.indices.is_empty()
                || !p.indices.len().is_multiple_of(3)
                || p.indices.iter().any(|&i| i as usize >= p.vertices.len())
                || (!p.ambient_visibility.is_empty()
                    && (p.ambient_visibility.len() != p.vertices.len()
                        || p.ambient_visibility
                            .iter()
                            .any(|v| !v.is_finite() || !(0.0..=1.0).contains(v))))
            {
                return Err("invalid mesh primitive");
            }
            if p.vertices.iter().any(|v| {
                v.position
                    .iter()
                    .chain(&v.normal)
                    .chain(&v.uv)
                    .any(|x| !x.is_finite())
                    || (glam::Vec3::from(v.normal).length_squared() - 1.0).abs() > 1e-3
            }) {
                return Err("invalid mesh vertex");
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn texture_mips_preserve_linear_energy_and_npot_edge_area() {
        let image = Image {
            width: 2,
            height: 1,
            rgba8: vec![0, 0, 0, 0, 255, 255, 255, 255],
        };
        let color = image.mip_chain(true).unwrap();
        assert_eq!(color.len(), 2);
        assert_eq!(color[1].rgba8, vec![188, 188, 188, 128]);
        assert_eq!(image.mip_chain(false).unwrap()[1].rgba8, vec![128; 4]);
        let odd = Image {
            width: 3,
            height: 1,
            rgba8: vec![255, 255, 255, 255, 0, 0, 0, 255, 255, 255, 255, 255],
        };
        assert_eq!(
            odd.mip_chain(false).unwrap()[1].rgba8,
            vec![170, 170, 170, 255]
        );
        assert!(
            Image {
                width: 0,
                height: 1,
                rgba8: vec![]
            }
            .mip_chain(true)
            .is_none()
        );
    }

    #[test]
    fn rigid_instances_subtract_camera_before_narrowing_and_reject_invalid_poses() {
        let origin = DVec3::new(1.5e11, -1e12, 2e12);
        let mut instance = MeshInstance {
            position_m: origin + DVec3::new(0.03125, 2.0, -10.0),
            rotation: DQuat::from_rotation_y(0.37),
            scale: 0.5,
        };
        let matrix = instance.camera_relative_matrix(origin).unwrap();
        assert_eq!(
            matrix.w_axis.truncate(),
            glam::Vec3::new(0.03125, 2.0, -10.0)
        );
        for scale in [0.0, -1.0, f64::INFINITY, f64::NAN, 1e40, 1e-40, 1e30] {
            instance.scale = scale;
            assert!(instance.camera_relative_matrix(origin).is_none());
        }
    }
    #[test]
    fn image_extent_overflow_and_invalid_references_are_errors_not_panics() {
        let mut source = MeshAsset {
            images: vec![Image {
                width: u32::MAX,
                height: u32::MAX,
                rgba8: vec![],
            }],
            textures: vec![],
            materials: vec![],
            primitives: vec![Primitive {
                ambient_visibility: vec![],
                vertices: vec![],
                indices: vec![],
                material: 0,
            }],
        };
        assert!(source.validate().is_err());
        source.images.clear();
        assert!(source.validate().is_err());
    }
}

/// A rigid source instance. Translation and camera share one f64 frame;
/// scale is a positive uniform asset-to-SI conversion, never a force input.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MeshInstance {
    pub position_m: DVec3,
    pub rotation: DQuat,
    pub scale: f64,
}
impl MeshInstance {
    pub fn camera_relative_matrix(&self, camera_m: DVec3) -> Option<glam::Mat4> {
        let frame = crate::BodyFrame::new(self.position_m, self.rotation, camera_m)?;
        // Keep squared shader-normal lengths representable as normal f32.
        let scale = self.scale as f32;
        if !self.scale.is_finite() || scale < f32::MIN_POSITIVE.sqrt() || scale > f32::MAX.sqrt() {
            return None;
        }
        let matrix = frame.render_from_body() * DMat4::from_scale(DVec3::splat(self.scale));
        let narrow = matrix.as_mat4();
        narrow.is_finite().then_some(narrow)
    }
}
