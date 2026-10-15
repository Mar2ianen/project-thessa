//! View uniforms for the own render loop (R3 foundation).
//!
//! Layout is byte-identical to the `ViewUniforms` prefix the portable CBT
//! shaders consume (`bevy-rcbt/src/render.rs`, classifier slim mirror and
//! raster block): seven column-major `mat4x4<f32>` (448 bytes), then
//! `world_position: vec3<f32>` plus `exposure: f32` (16 bytes) — 464 bytes
//! total. All math runs in `f64` and narrows once, mirroring the
//! `clip_from_world` fallback in `draw_cbt_geometry`; the projection
//! formula itself stays with the future camera owner, which supplies
//! `clip_from_view` directly.
//!
//! A non-invertible `world_from_view` yields `None` instead of a poisoned
//! block — the frame must fall back or skip, never upload NaN.

use glam::{DMat4, DVec3, Mat4, Vec3};

/// Camera-side inputs for one view, f64 SI. No Bevy types.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ViewParams {
    /// Camera transform (rotation + translation), metres.
    pub world_from_view: DMat4,
    /// Projection matrix (camera owner's formula).
    pub clip_from_view: DMat4,
    /// Camera world position in metres, narrowed separately.
    pub world_position_m: DVec3,
    /// Photometric exposure multiplier, passed through as-is.
    pub exposure: f32,
}

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
        let world_from_view = params.world_from_view;
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
            clip_from_world: narrow(clip_from_world),
            unjittered_clip_from_world: narrow(clip_from_world),
            world_from_clip: narrow(world_from_clip),
            world_from_view: narrow(world_from_view),
            view_from_world: narrow(view_from_world),
            clip_from_view: narrow(clip_from_view),
            view_from_clip: narrow(view_from_clip),
            world_position: Vec3::new(
                params.world_position_m.x as f32,
                params.world_position_m.y as f32,
                params.world_position_m.z as f32,
            ),
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

fn narrow(m: DMat4) -> Mat4 {
    Mat4::from_cols_array(&m.to_cols_array().map(|x| x as f32))
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
        assert_eq!(
            block.view_from_world,
            Mat4::from_translation([-100.0, 50.0, -25.0].into())
        );
        assert_eq!(
            block.world_from_view,
            Mat4::from_translation([100.0, -50.0, 25.0].into())
        );
        assert_eq!(block.clip_from_world, block.view_from_world);
        assert_eq!(block.unjittered_clip_from_world, block.clip_from_world);
        assert_eq!(block.world_from_clip, block.world_from_view);
        assert_eq!(block.clip_from_view, Mat4::IDENTITY);
        assert_eq!(block.view_from_clip, Mat4::IDENTITY);
        assert_eq!(block.world_position, Vec3::new(100.0, -50.0, 25.0));
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
            world_from_view: translation(100.0, -50.0, 25.0),
            clip_from_view: DMat4::IDENTITY,
            world_position_m: DVec3::new(100.0, -50.0, 25.0),
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
        assert_eq!(word(448), 100.0);
        assert_eq!(word(452), -50.0);
        assert_eq!(word(456), 25.0);
        assert_eq!(word(460), 1.25);
    }
}
