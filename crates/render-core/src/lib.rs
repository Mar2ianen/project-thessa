//! Backend-neutral presentation geometry. No scene policy, physics clock, ECS
//! or graphics handles. Callers supply poses from their existing state owner.
#![forbid(unsafe_code)]

use glam::{DMat4, DQuat, DVec3};
pub mod environment;
pub mod mesh;
pub mod ocean;
pub mod shadow;

/// Camera pose in a caller-selected coordinate frame, in SI metres. Projection
/// and photometric exposure are supplied by the camera/graphics policy owner.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ViewParams {
    /// Affine camera transform. Translation must match `world_position_m`.
    pub world_from_view: DMat4,
    pub clip_from_view: DMat4,
    /// Camera origin in the same frame as `world_from_view`. A renderer must
    /// subtract this in f64 before narrowing to its camera-relative transport.
    pub world_position_m: DVec3,
    pub exposure: f32,
}

/// A rigid body in an origin-rebased presentation frame. Absolute positions
/// are subtracted **before** rotating or narrowing. The orientation maps body
/// directions to world directions; translation never affects directions.
///
/// This is a presentation transform, not authoritative state or a spin model.
/// The caller must sample body, camera and origin at the same presentation time.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodyFrame {
    body_offset_m: DVec3,
    body_to_world: DQuat,
}

impl BodyFrame {
    /// Reject invalid poses rather than silently normalizing a corrupt state.
    /// Unit quaternions tolerate ordinary f64 composition round-off (1e-12).
    pub fn new(
        body_position_m: DVec3,
        body_to_world: DQuat,
        render_origin_m: DVec3,
    ) -> Option<Self> {
        let body_offset_m = body_position_m - render_origin_m;
        if !body_position_m.is_finite()
            || !render_origin_m.is_finite()
            || !body_offset_m.is_finite()
            || !body_to_world.is_finite()
            || (body_to_world.length_squared() - 1.0).abs() > 1e-12
        {
            return None;
        }
        Some(Self {
            body_offset_m,
            body_to_world,
        })
    }

    pub fn rotation(&self) -> DQuat {
        self.body_to_world
    }
    pub fn body_to_render_point(&self, point_body_m: DVec3) -> DVec3 {
        self.body_to_world * point_body_m + self.body_offset_m
    }
    pub fn render_to_body_point(&self, point_render_m: DVec3) -> DVec3 {
        self.body_to_world.conjugate() * (point_render_m - self.body_offset_m)
    }
    pub fn body_to_render_direction(&self, direction: DVec3) -> DVec3 {
        self.body_to_world * direction
    }
    pub fn render_to_body_direction(&self, direction: DVec3) -> DVec3 {
        self.body_to_world.conjugate() * direction
    }
    /// Host-local matrix, not an absolute astronomical f32 transform.
    pub fn render_from_body(&self) -> DMat4 {
        DMat4::from_rotation_translation(self.body_to_world, self.body_offset_m)
    }
    /// Express an origin-local render camera in body coordinates for terrain
    /// selection and camera-relative geometry. Projection/exposure are unchanged.
    /// Do not feed an absolute world camera to an origin-rebased frame.
    pub fn view_in_body(&self, render_view: &ViewParams) -> ViewParams {
        let mut local = render_view.world_from_view;
        local.w_axis -= self.body_offset_m.extend(0.0);
        let world_from_view = DMat4::from_quat(self.body_to_world.conjugate()) * local;
        ViewParams {
            world_from_view,
            world_position_m: self.render_to_body_point(render_view.world_position_m),
            clip_from_view: render_view.clip_from_view,
            exposure: render_view.exposure,
        }
    }
}

#[cfg(test)]
mod tests;
