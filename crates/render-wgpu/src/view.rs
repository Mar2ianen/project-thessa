//! View uniforms for the own render loop (R3 foundation).
//!
//! Layout is byte-identical to the `ViewUniforms` prefix the portable CBT
//! shaders consume (`thessa_rcbt_core::shaders`, classifier slim mirror and
//! raster block): seven column-major `mat4x4<f32>` (448 bytes), then
//! `world_position: vec3<f32>` plus `exposure: f32` (16 bytes) — 464 bytes
//! total. Absolute camera translation is removed in `f64` before any
//! narrowing. Shader "world" means camera-relative render space, matching
//! `TileAnchor::to_gpu(camera_body_m)`; never upload an absolute body
//! translation through `render_from_body`. The projection
//! formula itself stays with the future camera owner, which supplies
//! `clip_from_view` directly.
//!
//! A non-invertible `world_from_view` yields `None` instead of a poisoned
//! block — the frame must fall back or skip, never upload NaN.

#[cfg(test)]
use glam::DVec3;
use glam::{DMat4, Mat4, Vec3};
/// Compatibility re-export. The semantic camera input has no backend dependency.
pub use thessa_render_core::ViewParams;

/// The 464-byte block the CBT shaders bind. Field order matches the WGSL
/// `ViewUniforms` struct exactly; see module docs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ViewUniformBlock {
    pub clip_from_world: Mat4,
    pub unjittered_clip_from_world: Mat4,
    pub world_from_clip: Mat4,
    pub world_from_view: Mat4,
    pub view_from_world: Mat4,
    pub clip_from_view: Mat4,
    pub view_from_clip: Mat4,
    pub world_position: Vec3,
    pub exposure: f32,
}

impl ViewUniformBlock {
    /// Size of [`Self::to_bytes`] in bytes.
    pub const BYTE_LEN: usize = 464;

    /// Derive every matrix from the inputs. The own engine has no jitter
    /// yet, so `unjittered_clip_from_world` equals `clip_from_world` by
    /// construction (kept as a separate field for shader compatibility).
    pub fn new(params: &ViewParams) -> Option<Self> {
        if params.world_from_view.determinant().abs() <= f64::MIN_POSITIVE
            || !is_finite_mat4(&params.world_from_view)
            || !is_finite_mat4(&params.clip_from_view)
            || !params.world_position_m.is_finite()
            || !params.exposure.is_finite()
        {
            return None;
        }
        // Match the tile anchor contract: shaders operate on offsets from
        // this camera, not planet-sized absolute f32 positions.
        let mut world_from_view = params.world_from_view;
        let origin_error = (world_from_view.w_axis.truncate() - params.world_position_m)
            .abs()
            .max_element();
        let origin_tolerance =
            16.0 * f64::EPSILON * params.world_position_m.abs().max_element().max(1.0);
        let affine_error = (world_from_view.row(3) - glam::DVec4::W)
            .abs()
            .max_element();
        if origin_error > origin_tolerance || affine_error > 16.0 * f64::EPSILON {
            return None;
        }
        // The supplied f64 camera position is authoritative. Do not retain
        // tiny translation residue from an absolute look-at matrix inverse.
        world_from_view.w_axis = glam::DVec4::W;
        let view_from_world = world_from_view.inverse();
        let clip_from_view = params.clip_from_view;
        let clip_from_world = clip_from_view * view_from_world;
        let world_from_clip = clip_from_world.inverse();
        let view_from_clip = clip_from_view.inverse();
        if !is_finite_mat4(&clip_from_world)
            || !is_finite_mat4(&world_from_clip)
            || !is_finite_mat4(&view_from_clip)
        {
            return None;
        }
        Some(Self {
            clip_from_world: narrow(clip_from_world)?,
            unjittered_clip_from_world: narrow(clip_from_world)?,
            world_from_clip: narrow(world_from_clip)?,
            world_from_view: narrow(world_from_view)?,
            view_from_world: narrow(view_from_world)?,
            clip_from_view: narrow(clip_from_view)?,
            view_from_clip: narrow(view_from_clip)?,
            world_position: Vec3::ZERO,
            exposure: params.exposure,
        })
    }

    /// Exact shader-layout bytes: 7 column-major mat4, then xyz + exposure.
    pub fn to_bytes(&self) -> [u8; Self::BYTE_LEN] {
        let mut out = [0u8; Self::BYTE_LEN];
        let mut at = 0;
        for m in [
            self.clip_from_world,
            self.unjittered_clip_from_world,
            self.world_from_clip,
            self.world_from_view,
            self.view_from_world,
            self.clip_from_view,
            self.view_from_clip,
        ] {
            for word in m.to_cols_array() {
                out[at..at + 4].copy_from_slice(&word.to_le_bytes());
                at += 4;
            }
        }
        for word in [
            self.world_position.x,
            self.world_position.y,
            self.world_position.z,
            self.exposure,
        ] {
            out[at..at + 4].copy_from_slice(&word.to_le_bytes());
            at += 4;
        }
        debug_assert_eq!(at, Self::BYTE_LEN);
        out
    }
}

fn is_finite_mat4(m: &DMat4) -> bool {
    m.to_cols_array().iter().all(|x| x.is_finite())
}

fn narrow(m: DMat4) -> Option<Mat4> {
    let result = Mat4::from_cols_array(&m.to_cols_array().map(|x| x as f32));
    result.is_finite().then_some(result)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn translation(x: f64, y: f64, z: f64) -> DMat4 {
        DMat4::from_translation(glam::DVec3::new(x, y, z))
    }

    #[test]
    fn translation_camera_yields_exact_derived_matrices() {
        // Identity projection: every derived matrix is a pure translation
        // with hand-computable elements.
        let params = ViewParams {
            world_from_view: translation(100.0, -50.0, 25.0),
            clip_from_view: DMat4::IDENTITY,
            world_position_m: DVec3::new(100.0, -50.0, 25.0),
            exposure: 1.25,
        };
        let block = ViewUniformBlock::new(&params).expect("invertible view");
        assert_eq!(block.view_from_world, Mat4::IDENTITY);
        assert_eq!(block.world_from_view, Mat4::IDENTITY);
        assert_eq!(block.clip_from_world, block.view_from_world);
        assert_eq!(block.unjittered_clip_from_world, block.clip_from_world);
        assert_eq!(block.world_from_clip, block.world_from_view);
        assert_eq!(block.clip_from_view, Mat4::IDENTITY);
        assert_eq!(block.view_from_clip, Mat4::IDENTITY);
        assert_eq!(block.world_position, Vec3::ZERO);
        assert_eq!(block.exposure, 1.25);
    }

    #[test]
    fn degenerate_view_returns_none_not_nan() {
        let zero_x = DMat4::from_cols(
            glam::DVec4::ZERO,
            glam::DVec4::Y,
            glam::DVec4::Z,
            glam::DVec4::W,
        );
        let params = ViewParams {
            world_from_view: zero_x,
            clip_from_view: DMat4::IDENTITY,
            world_position_m: DVec3::ZERO,
            exposure: 1.0,
        };
        assert!(ViewUniformBlock::new(&params).is_none());
        let nan = ViewParams {
            world_from_view: DMat4::IDENTITY,
            clip_from_view: DMat4::IDENTITY,
            world_position_m: DVec3::new(f64::NAN, 0.0, 0.0),
            exposure: 1.0,
        };
        assert!(ViewUniformBlock::new(&nan).is_none());
    }

    #[test]
    fn byte_layout_matches_shader_offsets() {
        let params = ViewParams {
            world_from_view: DMat4::IDENTITY,
            clip_from_view: translation(-100.0, 50.0, -25.0),
            world_position_m: DVec3::ZERO,
            exposure: 1.25,
        };
        let bytes = ViewUniformBlock::new(&params)
            .expect("invertible")
            .to_bytes();
        assert_eq!(bytes.len(), ViewUniformBlock::BYTE_LEN);
        let word = |at: usize| f32::from_le_bytes(bytes[at..at + 4].try_into().unwrap());
        // clip_from_world is a column-major translation matrix: diagonal 1,
        // translation in the last column (words 12,13,14).
        assert_eq!(word(0), 1.0);
        assert_eq!(word(20), 1.0);
        assert_eq!(word(40), 1.0);
        assert_eq!(word(48), -100.0);
        assert_eq!(word(52), 50.0);
        assert_eq!(word(56), -25.0);
        // world_position at 448, exposure at 460.
        assert_eq!(word(448), 0.0);
        assert_eq!(word(452), 0.0);
        assert_eq!(word(456), 0.0);
        assert_eq!(word(460), 1.25);
    }

    #[test]
    fn rejects_f32_overflow_and_singular_projection() {
        let mut params = ViewParams {
            world_from_view: DMat4::IDENTITY,
            clip_from_view: DMat4::IDENTITY,
            world_position_m: DVec3::ZERO,
            exposure: 1.0,
        };
        params.clip_from_view.x_axis.x = 1e40;
        assert!(ViewUniformBlock::new(&params).is_none());
        params.clip_from_view.x_axis.x = 1e-40; // inverse overflows f32
        assert!(ViewUniformBlock::new(&params).is_none());
        params.clip_from_view.x_axis.x = 0.0;
        assert!(ViewUniformBlock::new(&params).is_none());
        params.clip_from_view = DMat4::IDENTITY;
        params.world_from_view = translation(1e40, 0.0, 0.0);
        assert!(ViewUniformBlock::new(&params).is_none());
    }

    #[test]
    fn rejects_inconsistent_camera_origin_and_non_affine_view() {
        let mut params = ViewParams {
            world_from_view: translation(10.0, 0.0, 0.0),
            clip_from_view: DMat4::IDENTITY,
            world_position_m: DVec3::ZERO,
            exposure: 1.0,
        };
        assert!(ViewUniformBlock::new(&params).is_none());
        params.world_position_m = DVec3::new(10.0, 0.0, 0.0);
        assert!(ViewUniformBlock::new(&params).is_some());
        params.world_from_view.x_axis.w = 0.1;
        assert!(ViewUniformBlock::new(&params).is_none());
    }

    #[test]
    fn astronomical_translation_does_not_reach_uniforms() {
        let rotation = DMat4::from_rotation_y(0.37);
        let make = |origin: DVec3| {
            ViewUniformBlock::new(&ViewParams {
                world_from_view: DMat4::from_translation(origin) * rotation,
                clip_from_view: glam::dcamera::rh::proj::directx::perspective(1.0, 1.5, 0.1, 1e7),
                world_position_m: origin,
                exposure: 1.0,
            })
            .unwrap()
        };
        let near = make(DVec3::ZERO);
        let far = make(DVec3::new(1.5e11, -6.4e6, 1e12));
        assert_eq!(near.to_bytes(), far.to_bytes());
        let point = Vec3::new(0.03125, 0.0, -2.0);
        assert_eq!(
            near.clip_from_world.project_point3(point),
            far.clip_from_world.project_point3(point)
        );
    }
}
