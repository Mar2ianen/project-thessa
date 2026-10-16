//! Existing flight/terrain coordinate convention, shared with presentation
//! consumers. This is game policy, not a new celestial attitude model.
use glam::{DQuat, DVec3};
use thessa_sim_core::{BakedBody, SimTime};

/// Validated visual sample relative to its caller-selected reference body.
/// No clock, interpolation queue, physics stepping or renderer handles. Local
/// authority consumers retain the precise relative coordinate; wire consumers
/// subtract the ephemeris at the sample's own time before publication.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlightPose {
    time: SimTime,
    relative_position_m: DVec3,
    orientation: DQuat,
}
impl FlightPose {
    pub fn new(time: SimTime, relative_position_m: DVec3, orientation: DQuat) -> Option<Self> {
        if !time.0.is_finite()
            || !relative_position_m.is_finite()
            || !orientation.is_finite()
            || (orientation.length_squared() - 1.0).abs() > 1e-12
        {
            return None;
        }
        Some(Self {
            time,
            relative_position_m,
            orientation,
        })
    }
    pub fn from_authority(authority: &crate::FlightAuthority) -> Option<Self> {
        Self::new(
            SimTime(authority.flight_time_s),
            authority.relative_position_m,
            authority.state.orientation_body_to_inertial,
        )
    }
    pub fn from_inertial(
        ephemeris: &thessa_sim_core::BakedEphemeris,
        body: thessa_sim_core::BodyId,
        time: SimTime,
        state: &thessa_sim_core::RigidBodyState,
    ) -> Option<Self> {
        let reference = ephemeris.body_state(body, time).ok()?;
        Self::new(
            time,
            state.position_inertial_m - reference.position_inertial,
            state.orientation_body_to_inertial,
        )
    }
    pub fn time(self) -> SimTime {
        self.time
    }
    pub fn relative_position_m(self) -> DVec3 {
        self.relative_position_m
    }
    pub fn render_position_m(self) -> DVec3 {
        inertial_to_render_axes(self.relative_position_m)
    }
    pub fn visual_orientation(self) -> glam::Quat {
        craft_visual_orientation(self.orientation)
    }
    /// Uncalibrated f64 attitude for retained body-axis meshes, not GLB axes.
    pub fn orientation_body_to_inertial(self) -> DQuat {
        self.orientation
    }
}

// Inherited pilot calibration, retained for visual compatibility. The decoded
// GLB's measured X extent is 17.794085 m, not this nominal source length.
pub const X15_SOURCE_LENGTH_M: f32 = 16.77;
pub const X15_AUTHORED_LENGTH_M: f32 = 15.45;
/// Authored GLB nose -X / top +Y / span +Z to the existing craft-visual
/// parent nose +Y / top -Z / span +X. Game asset calibration remains GPL.
pub fn x15_asset_to_craft_rotation() -> glam::Quat {
    glam::Quat::from_mat3(&glam::Mat3::from_cols(
        glam::Vec3::NEG_Y,
        glam::Vec3::NEG_Z,
        glam::Vec3::X,
    ))
}

/// Existing visual-parent basis used by the pilot. Only the final local
/// attitude is narrowed; authoritative orientation stays f64 in the runtime.
pub fn craft_visual_orientation(orientation: DQuat) -> glam::Quat {
    let forward = inertial_to_render_axes(orientation * DVec3::X)
        .as_vec3()
        .normalize_or_zero();
    let lateral = inertial_to_render_axes(orientation * DVec3::Y)
        .as_vec3()
        .normalize_or_zero();
    let up = inertial_to_render_axes(orientation * DVec3::Z)
        .as_vec3()
        .normalize_or_zero();
    if forward.length_squared() < 1e-8
        || lateral.length_squared() < 1e-8
        || up.length_squared() < 1e-8
    {
        return glam::Quat::IDENTITY;
    }
    glam::Quat::from_mat3(&glam::Mat3::from_cols(lateral, forward, -up))
}

/// Same period selection used by flight atmosphere/contact/terrain sampling.
/// It is a uniform-spin approximation, not eccentric tidal-lock libration or
/// the gravity harmonics frame (which also models axial tilt/prime meridian).
pub fn terrain_rotation_period_s(body: &BakedBody) -> f64 {
    body.rotation_period_s
        .or_else(|| body.tidal_lock.then_some(body.design_period_s).flatten())
        .unwrap_or(80.0 * 3_600.0)
}

/// Inertial Z-up to terrain/render Y-up, with the original x/z/-y handedness.
pub fn inertial_to_render_axes(vector: DVec3) -> DVec3 {
    DVec3::new(vector.x, vector.z, -vector.y)
}

/// Body-fixed terrain to inertial-aligned render axes at presentation time.
/// Callers must use the same interpolated time for camera, body and lighting.
pub fn terrain_rotation(time: SimTime, period_s: f64) -> DQuat {
    DQuat::from_rotation_y(time.0 * std::f64::consts::TAU / period_s)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn checked_pose_rejects_invalid_samples_and_preserves_relative_precision() {
        let relative = DVec3::new(3_200_000.123456789, 0.03125, -0.5);
        let pose = FlightPose::new(SimTime(12.5), relative, DQuat::from_rotation_y(0.7)).unwrap();
        assert_eq!(pose.relative_position_m(), relative);
        assert_eq!(
            pose.render_position_m(),
            DVec3::new(relative.x, relative.z, -relative.y)
        );
        for (time, p, q) in [
            (f64::NAN, relative, DQuat::IDENTITY),
            (0.0, DVec3::NAN, DQuat::IDENTITY),
            (0.0, relative, DQuat::from_xyzw(0.0, 0.0, 0.0, 2.0)),
        ] {
            assert!(FlightPose::new(SimTime(time), p, q).is_none());
        }
    }
    #[test]
    fn inertial_pose_subtracts_reference_at_its_sample_time_and_rejects_failed_ephemeris() {
        let system: thessa_sim_core::SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).unwrap();
        let ephemeris = system.bake().unwrap();
        let body = ephemeris.body_id("thessa").unwrap();
        let time = SimTime(12_345.0);
        let reference = ephemeris.body_state(body, time).unwrap();
        let relative = DVec3::new(3_200_000.125, 100.0, -200.0);
        let mut state = thessa_sim_core::RigidBodyState::new(
            reference.position_inertial + relative,
            DVec3::ZERO,
            DQuat::from_rotation_x(0.8),
            DVec3::ZERO,
        )
        .unwrap();
        let pose = FlightPose::from_inertial(&ephemeris, body, time, &state).unwrap();
        assert!(
            (pose.relative_position_m() - relative).length() < 0.001,
            "binary64 absolute source cannot promise sub-ULP recovery"
        );
        assert_eq!(pose.time(), time);
        assert_eq!(
            pose.visual_orientation(),
            craft_visual_orientation(state.orientation_body_to_inertial)
        );
        assert!(FlightPose::from_inertial(&ephemeris, body, SimTime(f64::NAN), &state).is_none());
        state.position_inertial_m = DVec3::NAN;
        assert!(FlightPose::from_inertial(&ephemeris, body, time, &state).is_none());
    }
    #[test]
    fn baked_period_and_axes_preserve_authority_convention() {
        let system: thessa_sim_core::SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).unwrap();
        let ephemeris = system.bake().unwrap();
        let id = ephemeris.body_id("thessa").unwrap();
        let mut body = ephemeris
            .bodies
            .iter()
            .find(|b| b.id == id)
            .unwrap()
            .clone();
        assert_eq!(terrain_rotation_period_s(&body), 288_000.0);
        body.rotation_period_s = Some(100.0);
        assert_eq!(terrain_rotation_period_s(&body), 100.0);
        body.rotation_period_s = None;
        body.tidal_lock = false;
        assert_eq!(terrain_rotation_period_s(&body), 288_000.0);
        assert_eq!(
            inertial_to_render_axes(DVec3::new(1.0, 2.0, 3.0)),
            DVec3::new(1.0, 3.0, -2.0)
        );
        assert!((terrain_rotation(SimTime(25.0), 100.0) * DVec3::Z - DVec3::X).length() < 1e-14);
    }
}
