//! Existing survey/map camera controller, shared by the Bevy terrain client
//! and the native migration viewer. Presentation only: quaternion orbit and
//! free-look state, exponential zoom and the neutral eye offset in the
//! caller's site frame. No clocks, input devices, ECS, site lists or renderer
//! handles; hosts own bookmarks, the minimum-altitude field query and look-at
//! construction.
//!
//! Factors are inherited unchanged from the shipped survey: right-drag orbit
//! (Shift-drag looks instead), wheel zoom and per-site framing. Invalid
//! demands are ignored and degenerate site frames return `None` so hosts
//! keep their last valid view instead of publishing NaNs. Note the shipped
//! placement used a bare `.normalize()` for the site north that yields NaNs
//! at exact polar sites; the shared controller uses a finite polar basis.
use glam::{DVec3, Mat3, Quat, Vec3};

/// Default/limit policy inherited from the shipped survey (metres, f32).
pub const DEFAULT_DISTANCE_M: f32 = 5000.0;
pub const MIN_DISTANCE_M: f32 = 80.0;
pub const MAX_DISTANCE_M: f32 = 2_000_000.0;
/// Orbit elevation: the eye sits `distance * ELEVATION_FRACTION` above the
/// site plane before the orbit quaternion applies.
pub const ELEVATION_FRACTION: f32 = 0.18;
/// Minimum eye clearance above local terrain, inherited from the survey.
pub const MIN_ALTITUDE_MARGIN_M: f64 = 5.0;
/// Right-drag gain inherited from the shipped survey (radians per pixel).
const DRAG_RAD_PER_PX: f32 = 0.004;
/// Wheel zoom gain: `distance *= (-scroll_lines * ZOOM_PER_LINE).exp()`.
const ZOOM_PER_LINE: f32 = 0.1;

/// Survey camera state. The orbit/look quaternions stay unit and the
/// distance inside its bounds by construction.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurveyCamera {
    orbit: Quat,
    look: Quat,
    distance_m: f32,
}

impl SurveyCamera {
    /// Shipped survey default: identity orbit/look at 5 km.
    pub fn survey_default() -> Self {
        Self {
            orbit: Quat::IDENTITY,
            look: Quat::IDENTITY,
            distance_m: DEFAULT_DISTANCE_M,
        }
    }

    pub fn orbit(self) -> Quat {
        self.orbit
    }
    pub fn look(self) -> Quat {
        self.look
    }
    pub fn distance_m(self) -> f32 {
        self.distance_m
    }

    /// Replace the orbit wholesale. Rejects non-finite or non-unit values.
    pub fn set_orbit(&mut self, orbit: Quat) {
        if orbit.is_finite() && (orbit.length_squared() - 1.0).abs() <= 1e-6 {
            self.orbit = orbit;
        }
    }

    pub fn set_look(&mut self, look: Quat) {
        if look.is_finite() && (look.length_squared() - 1.0).abs() <= 1e-6 {
            self.look = look;
        }
    }

    /// Clamped replacement for slider/host restores. Ignores non-finite input.
    pub fn set_distance_m(&mut self, distance_m: f32) {
        if distance_m.is_finite() {
            self.distance_m = distance_m.clamp(MIN_DISTANCE_M, MAX_DISTANCE_M);
        }
    }

    /// Right-drag gesture from the shipped survey. With Shift held the same
    /// drag turns the free-look quaternion instead of the orbit. Ignores
    /// non-finite deltas.
    pub fn drag(&mut self, dx_px: f32, dy_px: f32, look: bool) {
        if !dx_px.is_finite() || !dy_px.is_finite() {
            return;
        }
        let delta = Quat::from_rotation_y(-dx_px * DRAG_RAD_PER_PX)
            * Quat::from_rotation_x(-dy_px * DRAG_RAD_PER_PX);
        if look {
            self.look = (self.look * delta).normalize();
        } else {
            self.orbit = (self.orbit * delta).normalize();
        }
    }

    /// Pure yaw step for keyboard orbit and deterministic headless sweeps.
    /// Ignores non-finite input.
    pub fn orbit_yaw(&mut self, yaw_rad: f32) {
        if !yaw_rad.is_finite() {
            return;
        }
        self.orbit = (self.orbit * Quat::from_rotation_y(yaw_rad)).normalize();
    }

    /// Exponential wheel zoom with the legacy limits. Ignores non-finite or
    /// zero scroll; overflow clamps far like the legacy clamp.
    pub fn zoom(&mut self, scroll_lines: f32) {
        if !scroll_lines.is_finite() || scroll_lines == 0.0 {
            return;
        }
        let scaled = self.distance_m * (-scroll_lines * ZOOM_PER_LINE).exp();
        if scaled.is_finite() {
            self.set_distance_m(scaled);
        } else if scaled == f32::INFINITY {
            self.distance_m = MAX_DISTANCE_M;
        }
    }

    /// Multiplicative zoom for keyboard shortcuts sharing one factor policy.
    /// Ignores non-finite or non-positive factors.
    pub fn zoom_by_factor(&mut self, factor: f32) {
        if !factor.is_finite() || factor <= 0.0 {
            return;
        }
        let scaled = self.distance_m * factor;
        if scaled.is_finite() {
            self.set_distance_m(scaled);
        } else if scaled == f32::INFINITY {
            self.distance_m = MAX_DISTANCE_M;
        }
    }

    /// Per-site framing restore, as on survey site selection: distance and
    /// free look reset while the orbit is preserved.
    pub fn reset_site(&mut self) {
        self.look = Quat::IDENTITY;
        self.distance_m = DEFAULT_DISTANCE_M;
    }

    /// Full default restore.
    pub fn reset(&mut self) {
        *self = Self::survey_default();
    }

    /// Neutral eye offset and site-up reference in the caller's site frame,
    /// using the exact legacy basis. Returns `None` for non-finite or
    /// degenerate site-up so hosts keep their previous view.
    pub fn offsets(&self, site_up: DVec3) -> Option<(Vec3, Vec3)> {
        if !site_up.is_finite() || (site_up.length_squared() - 1.0).abs() > 1e-6 {
            return None;
        }
        let north = (DVec3::Y - site_up * site_up.y)
            .try_normalize()
            .unwrap_or(DVec3::Z);
        let east = north.cross(site_up).try_normalize()?;
        let basis = Mat3::from_cols(east.as_vec3(), site_up.as_vec3(), -north.as_vec3());
        let offset = basis
            * (self.orbit * Vec3::new(0.0, self.distance_m * ELEVATION_FRACTION, self.distance_m));
        if !offset.is_finite() {
            return None;
        }
        Some((offset, site_up.as_vec3()))
    }

    /// Keep the eye at least `minimum_m` from the body center. Returns the
    /// eye unchanged on degenerate input; hosts compute the minimum from
    /// their own field query plus the inherited margin.
    pub fn enforce_minimum_altitude(eye: DVec3, minimum_m: f64) -> DVec3 {
        if !eye.is_finite() || !minimum_m.is_finite() || minimum_m <= 0.0 {
            return eye;
        }
        if eye.length() < minimum_m {
            eye.try_normalize()
                .map(|dir| dir * minimum_m)
                .unwrap_or(eye)
        } else {
            eye
        }
    }
}

impl Default for SurveyCamera {
    fn default() -> Self {
        Self::survey_default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Verbatim copy of the pre-extraction survey placement core (offsets
    /// only; minimum-altitude clamp and look-at stay host-side).
    fn legacy_offsets(orbit_state: Quat, distance: f32, site_up: DVec3) -> (Vec3, Vec3) {
        let north = (DVec3::Y - site_up * site_up.y).normalize();
        let east = north.cross(site_up).normalize();
        let basis = Mat3::from_cols(east.as_vec3(), site_up.as_vec3(), -north.as_vec3());
        let offset = basis * (orbit_state * Vec3::new(0.0, distance * 0.18, distance));
        (offset, site_up.as_vec3())
    }

    #[test]
    fn defaults_match_the_shipped_survey_policy() {
        let camera = SurveyCamera::survey_default();
        assert_eq!(camera.orbit, Quat::IDENTITY);
        assert_eq!(camera.look, Quat::IDENTITY);
        assert_eq!(camera.distance_m, DEFAULT_DISTANCE_M);
        assert_eq!(
            (DEFAULT_DISTANCE_M, MIN_DISTANCE_M, MAX_DISTANCE_M),
            (5000.0, 80.0, 2_000_000.0)
        );
        assert_eq!(ELEVATION_FRACTION, 0.18);
        assert_eq!(MIN_ALTITUDE_MARGIN_M, 5.0);
    }

    #[test]
    fn gestures_match_legacy_factors_clamps_and_resets() {
        let mut camera = SurveyCamera::survey_default();
        camera.drag(25.0, -10.0, false);
        let expected = (Quat::IDENTITY
            * Quat::from_rotation_y(-25.0 * 0.004)
            * Quat::from_rotation_x(10.0 * 0.004))
        .normalize();
        assert!((camera.orbit - expected).length() < 1e-6);
        assert_eq!(camera.look, Quat::IDENTITY);
        camera.drag(25.0, -10.0, true);
        assert!((camera.look - expected).length() < 1e-6);
        camera.orbit_yaw(0.05);
        assert!(
            (camera.orbit - (expected * Quat::from_rotation_y(0.05)).normalize()).length() < 1e-6
        );

        camera.zoom(10.0);
        assert!((camera.distance_m - 5000.0 * (-10.0 * 0.1f32).exp()).abs() < 0.5);
        camera.zoom(-1_000.0);
        assert_eq!(camera.distance_m, MAX_DISTANCE_M);
        camera.zoom(1_000.0);
        assert_eq!(camera.distance_m, MIN_DISTANCE_M);
        camera.zoom_by_factor(1.1);
        assert!((camera.distance_m - MIN_DISTANCE_M * 1.1).abs() < 1e-3);
        camera.set_distance_m(1e12);
        assert_eq!(camera.distance_m, MAX_DISTANCE_M);

        camera.reset_site();
        assert_eq!(camera.look, Quat::IDENTITY);
        assert_eq!(camera.distance_m, DEFAULT_DISTANCE_M);
        assert!(
            (camera.orbit - (expected * Quat::from_rotation_y(0.05)).normalize()).length() < 1e-6
        );
        camera.reset();
        assert_eq!(camera, SurveyCamera::survey_default());
    }

    #[test]
    fn invalid_demands_leave_the_camera_untouched() {
        let mut camera = SurveyCamera::survey_default();
        let before = camera;
        camera.drag(f32::NAN, 0.0, false);
        camera.drag(0.0, f32::INFINITY, true);
        camera.orbit_yaw(f32::NAN);
        camera.zoom(f32::NAN);
        camera.zoom(0.0);
        camera.zoom_by_factor(0.0);
        camera.zoom_by_factor(-1.0);
        camera.zoom_by_factor(f32::INFINITY);
        camera.set_distance_m(f32::NAN);
        camera.set_orbit(Quat::from_xyzw(0.0, 0.0, 0.0, 0.0));
        camera.set_look(Quat::from_xyzw(f32::NAN, 0.0, 0.0, 1.0));
        assert_eq!(camera, before);
        // Infinite zoom-out clamps far like the legacy clamp.
        camera.zoom_by_factor(f32::MAX);
        assert_eq!(camera.distance_m, MAX_DISTANCE_M);
    }

    #[test]
    fn offsets_match_legacy_math_exactly_across_frames_and_states() {
        let ups = [
            DVec3::new(0.2, 0.9, -0.4).normalize(),
            DVec3::new(-0.6, 0.01, 0.8).normalize(),
            DVec3::new(0.0, 0.999999, 0.001).normalize(),
            DVec3::new(0.0, -0.999999, -0.001).normalize(),
        ];
        for (orbit, distance) in [
            (Quat::IDENTITY, 5000.0),
            (Quat::from_rotation_x(0.6), 80.0),
            (
                (Quat::from_rotation_y(1.2) * Quat::from_rotation_x(-0.4)).normalize(),
                2_000_000.0,
            ),
        ] {
            let mut camera = SurveyCamera::survey_default();
            camera.set_orbit(orbit);
            camera.set_distance_m(distance);
            for up in ups {
                let (offset, site_up) = camera.offsets(up).unwrap();
                let (e_offset, e_up) = legacy_offsets(camera.orbit, camera.distance_m, up);
                assert_eq!(offset, e_offset);
                assert_eq!(site_up, e_up);
            }
        }
    }

    #[test]
    fn degenerate_site_frames_fail_closed() {
        let camera = SurveyCamera::survey_default();
        for up in [
            DVec3::ZERO,
            DVec3::new(f64::NAN, 0.0, 0.0),
            DVec3::new(0.0, f64::INFINITY, 0.0),
            DVec3::Y * 2.0,
        ] {
            assert!(camera.offsets(up).is_none());
        }
        // Near-polar sites keep the pole fallback instead of NaNs. Exact
        // poles are excluded from the differential test above because the
        // verbatim legacy `.normalize()` genuinely yields NaNs there.
        for up in [DVec3::Y, DVec3::NEG_Y] {
            let (offset, site_up) = camera.offsets(up).unwrap();
            assert!(offset.is_finite() && site_up.is_finite());
        }
    }

    #[test]
    fn minimum_altitude_clamp_matches_legacy_selection() {
        let eye = DVec3::new(0.0, 3_200_000.0, 0.0);
        assert_eq!(
            SurveyCamera::enforce_minimum_altitude(eye, 3_200_000.0 + 5.0),
            DVec3::new(0.0, 3_200_005.0, 0.0)
        );
        assert_eq!(
            SurveyCamera::enforce_minimum_altitude(eye, 3_000_000.0),
            eye
        );
        assert_eq!(SurveyCamera::enforce_minimum_altitude(eye, f64::NAN), eye);
        assert!(SurveyCamera::enforce_minimum_altitude(DVec3::NAN, 3_200_005.0).is_nan());
        assert_eq!(
            SurveyCamera::enforce_minimum_altitude(DVec3::ZERO, 3_200_005.0),
            DVec3::ZERO
        );
    }
}
