//! Existing flight/terrain coordinate convention, shared with presentation
//! consumers. This is game policy, not a new celestial attitude model.
use glam::{DQuat, DVec3};
use thessa_sim_core::{BakedBody, SimTime};

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
