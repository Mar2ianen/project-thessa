//! Backend-neutral temporal sampling contracts.
//!
//! This module owns only deterministic math: sub-pixel jitter, camera/static
//! surface reprojection and depth reconstruction. It has no GPU, window or
//! simulation dependency and does not advance history. A renderer that samples
//! history must still validate disocclusion, topology/material changes and
//! dynamic-object motion; those are not implied by these helpers.
//!
//! Conventions:
//!
//! - matrices are `f64` and use the renderer's homogeneous NDC convention
//!   (`x,y` in `[-1,1]`, DirectX-style `z` in `[0,1]`);
//! - UVs are normalized, y-down image coordinates;
//! - jitter is measured in output pixels, positive `y` downwards;
//! - world positions and matrices passed together must use the same origin and
//!   units (SI metres). A camera-relative renderer must add the current camera
//!   origin in `f64` before reprojecting against a previous absolute matrix.

use glam::{DMat4, DVec3, DVec4};

/// Largest permitted sub-pixel offset on either axis.
///
/// A larger offset would no longer be a reconstruction jitter; it would shift
/// primary visibility enough to change silhouettes and invalidate the usual
/// history bounds.
pub const MAX_JITTER_PX: f64 = 0.5;

/// A sub-pixel sample offset in output pixels, positive `y` downwards.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JitterOffset {
    pub pixels: [f64; 2],
}

impl JitterOffset {
    pub const ZERO: Self = Self { pixels: [0.0, 0.0] };

    /// Deterministic Halton (2,3) sequence sample for a rendered-frame index.
    ///
    /// The index is one-based so the first rendered frame already receives a
    /// stable, non-degenerate sample. History is counted in rendered frames,
    /// never in presented or frame-generated frames.
    pub fn halton(frame_index: u64) -> Self {
        let index = frame_index.wrapping_add(1);
        Self {
            pixels: [
                radical_inverse(index, 2) - 0.5,
                radical_inverse(index, 3) - 0.5,
            ],
        }
    }

    pub fn is_zero(self) -> bool {
        self.pixels == [0.0, 0.0]
    }

    /// Clip-space NDC offset for an output extent, or `None` for an invalid
    /// offset. A zero offset is valid without a meaningful extent.
    pub fn ndc(self, extent: [u32; 2]) -> Option<[f64; 2]> {
        if !self.pixels.iter().all(|value| value.is_finite())
            || self.pixels.iter().any(|value| value.abs() > MAX_JITTER_PX)
        {
            return None;
        }
        if self.is_zero() {
            return Some([0.0, 0.0]);
        }
        if extent[0] == 0 || extent[1] == 0 {
            return None;
        }
        Some([
            self.pixels[0] * 2.0 / f64::from(extent[0]),
            -self.pixels[1] * 2.0 / f64::from(extent[1]),
        ])
    }

    /// Apply this offset to an unjittered projection.
    ///
    /// The result adds `ndc.x * w` and `ndc.y * w` to clip space, so primary
    /// visibility shifts by a fixed number of pixels at every depth and the
    /// homogeneous `w` (and therefore depth) is unchanged.
    pub fn apply(self, clip_from_view: DMat4, extent: [u32; 2]) -> Option<DMat4> {
        let ndc = self.ndc(extent)?;
        if ndc == [0.0, 0.0] {
            return Some(clip_from_view);
        }
        let shift = DMat4::from_cols(
            DVec4::X,
            DVec4::Y,
            DVec4::Z,
            DVec4::new(ndc[0], ndc[1], 0.0, 1.0),
        );
        Some(shift * clip_from_view)
    }
}

impl JitterOffset {
    /// Explicit constructor for tests, adapters and scripted captures.
    ///
    /// Values outside [`MAX_JITTER_PX`] are rejected later by [`Self::ndc`] /
    /// [`Self::apply`] instead of being silently clamped.
    pub const fn new(pixels: [f64; 2]) -> Self {
        Self { pixels }
    }
}

fn radical_inverse(mut index: u64, base: u64) -> f64 {
    debug_assert!(base >= 2);
    let mut inverse = 1.0 / base as f64;
    let mut result = 0.0;
    while index > 0 {
        result += (index % base) as f64 * inverse;
        index /= base;
        inverse /= base as f64;
    }
    result
}

/// Project an absolute world position into y-down UV and DirectX depth.
///
/// Returns `None` behind the camera or outside the projection's valid depth
/// range. This is the current-frame half of a reprojection; it deliberately
/// does not use triangle identity, so it remains defined across CBT
/// split/merge when the represented surface itself did not change.
pub fn project_uv_depth(
    clip_from_world_unjittered: DMat4,
    world: DVec3,
) -> Option<([f64; 2], f64)> {
    if !world.is_finite() {
        return None;
    }
    let clip = clip_from_world_unjittered * world.extend(1.0);
    if !clip.is_finite() || clip.w <= 0.0 {
        return None;
    }
    let ndc = clip.truncate() / clip.w;
    if !(0.0..=1.0).contains(&ndc.z) {
        return None;
    }
    Some(([ndc.x * 0.5 + 0.5, 0.5 - ndc.y * 0.5], ndc.z))
}

/// Reconstruct an absolute world position from a y-down UV and DirectX depth.
///
/// `world_from_clip_unjittered` must be the inverse of the unjittered
/// projection, not of the jittered primary-visibility matrix.
pub fn world_from_depth(
    world_from_clip_unjittered: DMat4,
    uv: [f64; 2],
    depth: f64,
) -> Option<DVec3> {
    if !uv.iter().all(|value| value.is_finite())
        || !(0.0..=1.0).contains(&uv[0])
        || !(0.0..=1.0).contains(&uv[1])
        || !depth.is_finite()
        || !(0.0..=1.0).contains(&depth)
    {
        return None;
    }
    let ndc = DVec3::new(uv[0] * 2.0 - 1.0, 1.0 - uv[1] * 2.0, depth);
    let homogeneous = world_from_clip_unjittered * ndc.extend(1.0);
    if !homogeneous.is_finite() || homogeneous.w.abs() <= f64::MIN_POSITIVE {
        return None;
    }
    let world = homogeneous.truncate() / homogeneous.w;
    world.is_finite().then_some(world)
}

/// Screen-space displacement from the current frame to the previous one.
///
/// The sign convention is `previous_uv - current_uv`, so history is sampled at
/// `current_uv + motion_uv`. Returns `None` for non-finite input; bounds and
/// depth validation belong to the caller's history policy.
pub fn motion_uv(current_uv: [f64; 2], previous_uv: [f64; 2]) -> Option<[f64; 2]> {
    if !current_uv
        .iter()
        .chain(previous_uv.iter())
        .all(|v| v.is_finite())
    {
        return None;
    }
    Some([
        previous_uv[0] - current_uv[0],
        previous_uv[1] - current_uv[1],
    ])
}

/// One static-surface reprojection result.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Reprojection {
    pub current_uv: [f64; 2],
    pub previous_uv: [f64; 2],
    pub motion_uv: [f64; 2],
}

/// Reproject a static surface sample through current depth into the previous
/// camera.
///
/// Both matrices must be unjittered and expressed in the same absolute `f64`
/// frame. This is the camera-motion path for planetary/CBT terrain and
/// body-fixed meshes; it is **not** valid for a surface whose own world
/// position changed (displaced ocean, moving vehicle) unless the caller supplies
/// an additional object-motion term. The returned UV may lie outside `[0,1]`;
/// the history sampler must clamp or reject it.
pub fn reproject_static_surface(
    current_world_from_clip_unjittered: DMat4,
    previous_clip_from_world_unjittered: DMat4,
    current_uv: [f64; 2],
    current_depth: f64,
) -> Option<Reprojection> {
    let world = world_from_depth(
        current_world_from_clip_unjittered,
        current_uv,
        current_depth,
    )?;
    let (previous_uv, _) = project_uv_depth(previous_clip_from_world_unjittered, world)?;
    let motion_uv = motion_uv(current_uv, previous_uv)?;
    Some(Reprojection {
        current_uv,
        previous_uv,
        motion_uv,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::{DVec2, dcamera};

    fn perspective() -> DMat4 {
        dcamera::rh::proj::directx::perspective(1.0, 1.5, 0.1, 1_000.0)
    }

    fn camera_at(x: f64) -> DMat4 {
        let world_from_view = DMat4::from_translation(DVec3::new(x, 0.0, 10.0));
        perspective() * world_from_view.inverse()
    }

    #[test]
    fn halton_is_deterministic_bounded_and_non_degenerate() {
        let mut distinct = std::collections::BTreeSet::new();
        for frame in 0..64 {
            let jitter = JitterOffset::halton(frame);
            assert!(jitter.pixels.iter().all(|v| v.is_finite()));
            assert!(jitter.pixels.iter().all(|v| v.abs() <= MAX_JITTER_PX));
            assert!(jitter.ndc([1920, 1080]).is_some());
            assert_eq!(jitter, JitterOffset::halton(frame));
            distinct.insert((
                (jitter.pixels[0] * 1e9).round() as i64,
                (jitter.pixels[1] * 1e9).round() as i64,
            ));
        }
        assert!(distinct.len() > 32);
    }

    #[test]
    fn zero_jitter_is_valid_without_extent() {
        assert_eq!(JitterOffset::ZERO.ndc([0, 0]), Some([0.0, 0.0]));
        assert_eq!(
            JitterOffset::ZERO.apply(perspective(), [0, 0]),
            Some(perspective())
        );
    }

    #[test]
    fn oversized_or_non_finite_jitter_is_rejected() {
        for pixels in [
            [0.5000001, 0.0],
            [0.0, -0.5000001],
            [f64::NAN, 0.0],
            [0.0, f64::INFINITY],
        ] {
            assert!(JitterOffset::new(pixels).ndc([1920, 1080]).is_none());
            assert!(
                JitterOffset::new(pixels)
                    .apply(perspective(), [1920, 1080])
                    .is_none()
            );
        }
        assert!(
            JitterOffset::new([0.25, 0.0])
                .apply(perspective(), [0, 1080])
                .is_none()
        );
    }

    #[test]
    fn jitter_moves_ndc_but_preserves_w_and_depth() {
        let point = DVec3::new(0.125, -0.25, -5.0);
        let unjittered = perspective();
        let jitter = JitterOffset::new([0.5, -0.25]);
        let extent = [2000, 1000];
        let jittered = jitter.apply(unjittered, extent).unwrap();
        let a = unjittered * point.extend(1.0);
        let b = jittered * point.extend(1.0);
        assert!((a.w - b.w).abs() < 1e-12);
        assert!((a.z / a.w - b.z / b.w).abs() < 1e-12);
        let ndc_delta = DVec2::new(b.x / b.w - a.x / a.w, b.y / b.w - a.y / a.w);
        let expected = DVec2::new(
            0.5 * 2.0 / extent[0] as f64,
            -(-0.25) * 2.0 / extent[1] as f64,
        );
        assert!((ndc_delta - expected).length() < 1e-12);
    }

    #[test]
    fn identity_reprojection_has_zero_motion() {
        let clip_from_world = camera_at(0.0);
        let world = DVec3::new(0.0, 0.0, 0.0);
        let (uv, _) = project_uv_depth(clip_from_world, world).unwrap();
        let motion = motion_uv(uv, uv).unwrap();
        assert_eq!(motion, [0.0, 0.0]);
    }

    #[test]
    fn camera_translation_moves_static_surface_in_history() {
        let world = DVec3::new(0.0, 0.0, 0.0);
        let (current_uv, _) = project_uv_depth(camera_at(1.0), world).unwrap();
        let (previous_uv, _) = project_uv_depth(camera_at(0.0), world).unwrap();
        assert!(current_uv[0] < 0.5);
        assert!((previous_uv[0] - 0.5).abs() < 1e-12);
        let motion = motion_uv(current_uv, previous_uv).unwrap();
        assert!(motion[0] > 0.0);
        assert!(motion[1].abs() < 1e-12);
    }

    #[test]
    fn behind_camera_and_out_of_depth_are_rejected() {
        let clip_from_world = camera_at(0.0);
        assert!(project_uv_depth(clip_from_world, DVec3::new(0.0, 0.0, 100.0)).is_none());
        let outside = clip_from_world * DVec3::new(0.0, 0.0, -2_000.0).extend(1.0);
        assert!(project_uv_depth(clip_from_world, DVec3::new(0.0, 0.0, -2_000.0)).is_none());
        assert!(outside.w > 0.0);
    }

    #[test]
    fn depth_reconstruction_round_trips() {
        let clip_from_world = camera_at(0.37);
        let world = DVec3::new(0.25, -0.125, -1.5);
        let (uv, depth) = project_uv_depth(clip_from_world, world).unwrap();
        let reconstructed = world_from_depth(clip_from_world.inverse(), uv, depth).unwrap();
        assert!((reconstructed - world).length() < 1e-9);
    }

    #[test]
    fn static_reprojection_composes_depth_and_camera_motion() {
        let current = camera_at(0.0);
        let previous = camera_at(1.0);
        let world = DVec3::new(0.0, 0.0, 0.0);
        let (uv, depth) = project_uv_depth(current, world).unwrap();
        let reprojected = reproject_static_surface(current.inverse(), previous, uv, depth).unwrap();
        assert_eq!(reprojected.current_uv, uv);
        assert!(reprojected.previous_uv[0] < uv[0]);
        assert!(reprojected.motion_uv[0] < 0.0);
        assert!((reprojected.motion_uv[0] - (reprojected.previous_uv[0] - uv[0])).abs() < 1e-15);

        let same = reproject_static_surface(current.inverse(), current, uv, depth).unwrap();
        assert!(same.motion_uv[0].abs() < 1e-12);
        assert!(same.motion_uv[1].abs() < 1e-12);
    }

    #[test]
    fn invalid_reconstruction_inputs_are_rejected() {
        let inverse = camera_at(0.0).inverse();
        assert!(world_from_depth(inverse, [1.1, 0.5], 0.5).is_none());
        assert!(world_from_depth(inverse, [0.5, -0.1], 0.5).is_none());
        assert!(world_from_depth(inverse, [0.5, 0.5], 1.1).is_none());
        assert!(world_from_depth(inverse, [f64::NAN, 0.5], 0.5).is_none());
        assert!(motion_uv([f64::NAN, 0.0], [0.0, 0.0]).is_none());
    }
}
