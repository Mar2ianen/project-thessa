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
        }
    }
}
#[derive(Debug, Clone)]
pub struct Primitive {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
    pub material: usize,
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
#[derive(Debug, Clone, Copy)]
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
