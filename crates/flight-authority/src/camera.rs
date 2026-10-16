//! Existing pilot camera controller, shared by the Bevy pilot and the native
//! migration viewer. Presentation only: quaternion orbit, chase framing, pan
//! and zoom state plus the neutral eye/target/up offsets in the caller's craft
//! frame. No clocks, input devices, ECS or renderer handles; each host owns
//! its gestures, frame placement and look-at construction.
//!
//! The math is the extracted pilot transform: a quaternion orbit with no
//! polar clamp (camera-up survives vertical crossings), camera-axis panning,
//! exponential scroll zoom and an optional chase basis that follows craft
//! attitude. Invalid demands are ignored and degenerate frames return `None`
//! so hosts keep their last valid view instead of publishing NaNs.
use glam::{Mat3, Quat, Vec2, Vec3};

/// Default/limit policy inherited from the shipped pilot (metres, f32).
pub const DEFAULT_DISTANCE_M: f32 = 32.0;
pub const MIN_DISTANCE_M: f32 = 8.0;
pub const MAX_DISTANCE_M: f32 = 180.0;
/// Default orbit tilt pitched slightly down, as in the shipped pilot.
pub const DEFAULT_ORBIT_TILT_RAD: f32 = -0.18;
/// Pan stays within a few metres of the craft; it is a framing offset.
pub const PAN_LIMIT_M: f32 = 4.0;
/// Keyboard/sweep yaw step matching a 10 px drag at the legacy drag gain.
pub const KEYBOARD_YAW_STEP_RAD: f32 = 0.05;
/// Right-drag gain inherited from the shipped pilot (radians per pixel).
const DRAG_RAD_PER_PX: f32 = 0.005;
/// Middle-drag pan gain: metres per pixel at 1 m distance.
const PAN_PER_PX_AT_1_M: f32 = 0.0018;
/// Scroll zoom gain: `distance *= (-scroll_lines * ZOOM_PER_LINE).exp()`.
const ZOOM_PER_LINE: f32 = 0.09;
/// The look target sits slightly below the pan target, as in the pilot.
const LOOK_DOWN_FRACTION: f32 = 0.12;

/// Pilot camera state. The orbit quaternion is always unit length and the
/// distance/pan stay inside their bounds by construction.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlightCamera {
    orbit: Quat,
    chase: bool,
    distance_m: f32,
    pan: Vec2,
}

impl FlightCamera {
    /// Shipped pilot default: slight downward tilt, close chase-ready orbit.
    pub fn pilot_default() -> Self {
        Self {
            orbit: Quat::from_rotation_x(DEFAULT_ORBIT_TILT_RAD),
            chase: false,
            distance_m: DEFAULT_DISTANCE_M,
            pan: Vec2::ZERO,
        }
    }

    pub fn orbit(self) -> Quat {
        self.orbit
    }
    pub fn chase(self) -> bool {
        self.chase
    }
    pub fn distance_m(self) -> f32 {
        self.distance_m
    }
    pub fn pan(self) -> Vec2 {
        self.pan
    }

    /// Replace the orbit wholesale. Rejects non-finite or non-unit values so a
    /// bad caller cannot wedge the camera; the previous orbit is kept.
    pub fn set_orbit(&mut self, orbit: Quat) {
        if orbit.is_finite() && (orbit.length_squared() - 1.0).abs() <= 1e-6 {
            self.orbit = orbit;
        }
    }

    /// Clamped replacement for slider/host restores. Ignores non-finite input.
    pub fn set_distance_m(&mut self, distance_m: f32) {
        if distance_m.is_finite() {
            self.distance_m = distance_m.clamp(MIN_DISTANCE_M, MAX_DISTANCE_M);
        }
    }

    pub fn set_chase(&mut self, chase: bool) {
        self.chase = chase;
    }
    pub fn toggle_chase(&mut self) {
        self.chase = !self.chase;
    }

    /// Right-drag orbit from the shipped pilot. Ignores non-finite deltas.
    pub fn drag_orbit(&mut self, dx_px: f32, dy_px: f32) {
        if !dx_px.is_finite() || !dy_px.is_finite() {
            return;
        }
        self.orbit = (self.orbit
            * Quat::from_rotation_y(-dx_px * DRAG_RAD_PER_PX)
            * Quat::from_rotation_x(-dy_px * DRAG_RAD_PER_PX))
        .normalize();
    }

    /// Pure yaw step for keyboard orbit and deterministic headless sweeps.
    /// Ignores non-finite input.
    pub fn orbit_yaw(&mut self, yaw_rad: f32) {
        if !yaw_rad.is_finite() {
            return;
        }
        self.orbit = (self.orbit * Quat::from_rotation_y(yaw_rad)).normalize();
    }

    /// Middle-drag pan along the camera axes. Ignores non-finite deltas.
    pub fn pan_by(&mut self, dx_px: f32, dy_px: f32) {
        if !dx_px.is_finite() || !dy_px.is_finite() {
            return;
        }
        let scale = self.distance_m * PAN_PER_PX_AT_1_M;
        self.pan += Vec2::new(-dx_px, dy_px) * scale;
        self.pan = self
            .pan
            .clamp(Vec2::splat(-PAN_LIMIT_M), Vec2::splat(PAN_LIMIT_M));
    }

    /// Exponential scroll zoom with the legacy limits. Ignores non-finite or
    /// zero scroll. An overflowing scale clamps to the far limit, matching
    /// the legacy `clamp` on the product.
    pub fn zoom(&mut self, scroll_lines: f32) {
        if !scroll_lines.is_finite() || scroll_lines == 0.0 {
            return;
        }
        self.apply_scaled_distance(self.distance_m * (-scroll_lines * ZOOM_PER_LINE).exp());
    }

    /// Multiplicative zoom for keyboard shortcuts sharing one factor policy.
    /// Ignores non-finite or non-positive factors; overflow clamps far.
    pub fn zoom_by_factor(&mut self, factor: f32) {
        if !factor.is_finite() || factor <= 0.0 {
            return;
        }
        self.apply_scaled_distance(self.distance_m * factor);
    }

    fn apply_scaled_distance(&mut self, scaled: f32) {
        if scaled.is_finite() {
            self.set_distance_m(scaled);
        } else if scaled == f32::INFINITY {
            self.distance_m = MAX_DISTANCE_M;
        }
    }

    /// Restore the default framing. Chase mode is a view selection, not a
    /// framing error, so it is preserved like the shipped reset key.
    pub fn reset(&mut self) {
        let chase = self.chase;
        *self = Self::pilot_default();
        self.chase = chase;
    }

    /// Neutral eye/target/up offsets in the caller's craft frame, using the
    /// exact legacy basis. Returns `None` for non-finite or degenerate
    /// inputs; hosts must keep their previous view in that case.
    pub fn offsets(&self, craft_orientation: Quat, local_up: Vec3) -> Option<(Vec3, Vec3, Vec3)> {
        if !craft_orientation.is_finite()
            || !local_up.is_finite()
            || local_up.length_squared() < 1e-12
        {
            return None;
        }
        let forward = craft_orientation * Vec3::Y;
        let basis = if self.chase {
            // NOTE: parenthesize the negation! `-craft_orientation * v`
            // negates the QUATERNION (unary minus binds tighter), and -q is
            // the same rotation as q — the sign silently vanishes and the
            // basis becomes a reflection with garbage orientation.
            Quat::from_mat3(&Mat3::from_cols(
                -(craft_orientation * Vec3::X),
                -(craft_orientation * Vec3::Z),
                -forward,
            ))
        } else {
            let north = (Vec3::Y - local_up * local_up.y)
                .try_normalize()
                .unwrap_or(Vec3::Z);
            let east = north.cross(local_up).normalize();
            if !east.is_finite() {
                return None;
            }
            Quat::from_mat3(&Mat3::from_cols(east, local_up, -north))
        };
        let orbit = basis * self.orbit;
        if !orbit.is_finite() {
            return None;
        }
        let up = orbit * Vec3::Y;
        let target = orbit * Vec3::new(self.pan.x, self.pan.y, 0.0);
        let eye = target + orbit * Vec3::Z * self.distance_m;
        let look_target = target - up * (self.distance_m * LOOK_DOWN_FRACTION);
        if !eye.is_finite() || !look_target.is_finite() || !up.is_finite() {
            return None;
        }
        Some((eye, look_target, up))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Verbatim copy of the pre-extraction pilot transform core, kept to pin
    /// bit-exact migration of the framing math (offsets only; hosts own the
    /// final look-at in their own frame).
    fn legacy_offsets(
        orbit_state: Quat,
        chase: bool,
        distance: f32,
        pan: Vec2,
        craft_orientation: Quat,
        local_up: Vec3,
    ) -> (Vec3, Vec3, Vec3) {
        let forward = craft_orientation * Vec3::Y;
        let basis = if chase {
            Quat::from_mat3(&Mat3::from_cols(
                -(craft_orientation * Vec3::X),
                -(craft_orientation * Vec3::Z),
                -forward,
            ))
        } else {
            let north = (Vec3::Y - local_up * local_up.y)
                .try_normalize()
                .unwrap_or(Vec3::Z);
            let east = north.cross(local_up).normalize();
            Quat::from_mat3(&Mat3::from_cols(east, local_up, -north))
        };
        let orbit = basis * orbit_state;
        let up = orbit * Vec3::Y;
        let target = orbit * Vec3::new(pan.x, pan.y, 0.0);
        let eye = target + orbit * Vec3::Z * distance;
        (eye, target - up * (distance * 0.12), up)
    }

    #[test]
    fn defaults_match_the_shipped_pilot_policy() {
        let camera = FlightCamera::pilot_default();
        assert!((camera.orbit - Quat::from_rotation_x(DEFAULT_ORBIT_TILT_RAD)).length() < 1e-7);
        assert!(!camera.chase);
        assert_eq!(camera.distance_m, DEFAULT_DISTANCE_M);
        assert_eq!(camera.pan, Vec2::ZERO);
        assert_eq!(
            (DEFAULT_DISTANCE_M, MIN_DISTANCE_M, MAX_DISTANCE_M),
            (32.0, 8.0, 180.0)
        );
    }

    #[test]
    fn gestures_match_legacy_factors_clamps_and_reset() {
        let mut camera = FlightCamera::pilot_default();
        camera.drag_orbit(20.0, -10.0);
        let expected = (Quat::from_rotation_x(DEFAULT_ORBIT_TILT_RAD)
            * Quat::from_rotation_y(-20.0 * 0.005)
            * Quat::from_rotation_x(10.0 * 0.005))
        .normalize();
        assert!((camera.orbit - expected).length() < 1e-6);
        camera.orbit_yaw(0.05);
        assert!(
            (camera.orbit - (expected * Quat::from_rotation_y(0.05)).normalize()).length() < 1e-6
        );

        camera.pan_by(1_000.0, -2_000.0);
        assert_eq!(camera.pan, Vec2::new(-4.0, -4.0));
        camera.pan_by(-10.0, 5.0);
        assert!(
            (camera.pan - Vec2::new(-4.0 + 10.0 * 32.0 * 0.0018, -4.0 + 5.0 * 32.0 * 0.0018))
                .length()
                < 1e-6
        );

        camera.set_distance_m(32.0);
        camera.zoom(10.0);
        assert!((camera.distance_m - 32.0 * (-10.0 * 0.09f32).exp()).abs() < 1e-5);
        camera.zoom(-1_000.0);
        assert_eq!(camera.distance_m, MAX_DISTANCE_M);
        camera.zoom(1_000.0);
        assert_eq!(camera.distance_m, MIN_DISTANCE_M);
        camera.zoom_by_factor(1.1);
        assert!((camera.distance_m - MIN_DISTANCE_M * 1.1).abs() < 1e-5);
        camera.set_distance_m(1e9);
        assert_eq!(camera.distance_m, MAX_DISTANCE_M);

        camera.set_chase(true);
        camera.reset();
        assert!((camera.orbit - Quat::from_rotation_x(DEFAULT_ORBIT_TILT_RAD)).length() < 1e-7);
        assert_eq!(camera.distance_m, DEFAULT_DISTANCE_M);
        assert_eq!(camera.pan, Vec2::ZERO);
        assert!(camera.chase, "reset preserves the chase view selection");
        camera.toggle_chase();
        assert!(!camera.chase);
    }

    #[test]
    fn invalid_demands_leave_the_camera_untouched() {
        let mut camera = FlightCamera::pilot_default();
        let before = camera;
        camera.drag_orbit(f32::NAN, 0.0);
        camera.drag_orbit(0.0, f32::INFINITY);
        camera.orbit_yaw(f32::NAN);
        camera.pan_by(f32::NAN, 0.0);
        camera.zoom(f32::NAN);
        camera.zoom(0.0 - 0.0 + f32::NAN);
        camera.zoom_by_factor(0.0);
        camera.zoom_by_factor(-2.0);
        camera.zoom_by_factor(f32::INFINITY);
        camera.set_distance_m(f32::NAN);
        camera.set_orbit(Quat::from_xyzw(0.0, 0.0, 0.0, 0.0));
        camera.set_orbit(Quat::from_xyzw(f32::NAN, 0.0, 0.0, 1.0));
        assert_eq!(camera, before);
        camera.set_orbit(Quat::from_rotation_z(0.7));
        assert!((camera.orbit - Quat::from_rotation_z(0.7)).length() < 1e-7);
    }

    #[test]
    fn offsets_match_legacy_math_exactly_across_frames_and_states() {
        let orientations = [
            Quat::IDENTITY,
            Quat::from_rotation_x(1.1),
            Quat::from_rotation_y(-0.7),
            Quat::from_rotation_z(2.3),
            Quat::from_axis_angle(Vec3::new(0.3, 0.8, -0.5).normalize(), 1.9),
        ];
        let ups = [
            Vec3::NEG_Z,
            Vec3::Y,
            Vec3::NEG_Y,
            Vec3::new(0.2, 0.9, -0.4).normalize(),
            Vec3::new(-0.6, 0.01, 0.8).normalize(),
        ];
        let mut states = Vec::new();
        for chase in [false, true] {
            for orbit in [
                Quat::from_rotation_x(DEFAULT_ORBIT_TILT_RAD),
                Quat::from_rotation_x(-0.65),
                (Quat::from_rotation_y(0.4) * Quat::from_rotation_x(0.2)).normalize(),
            ] {
                for (distance, pan) in [
                    (32.0, Vec2::ZERO),
                    (8.0, Vec2::new(4.0, -4.0)),
                    (180.0, Vec2::new(1.5, 2.5)),
                ] {
                    let mut camera = FlightCamera::pilot_default();
                    camera.set_orbit(orbit);
                    camera.set_chase(chase);
                    camera.set_distance_m(distance);
                    camera.pan_by(pan.x / (distance * 0.0018), pan.y / (distance * 0.0018));
                    states.push(camera);
                }
            }
        }
        for camera in states {
            for orientation in orientations {
                for up in ups {
                    let (eye, look, upv) = camera.offsets(orientation, up).unwrap();
                    let (e_eye, e_look, e_up) = legacy_offsets(
                        camera.orbit,
                        camera.chase,
                        camera.distance_m,
                        camera.pan,
                        orientation,
                        up,
                    );
                    assert_eq!(eye, e_eye);
                    assert_eq!(look, e_look);
                    assert_eq!(upv, e_up);
                }
            }
        }
    }

    #[test]
    fn chase_offsets_frame_level_flight_with_the_planet_below() {
        // Ported layout pin (no renderer): level flight, nose along render
        // +Y with +Z down, so planet-up is -Z. Same numbers as the pilot.
        let mut camera = FlightCamera::pilot_default();
        camera.set_chase(true);
        let (eye, look, up) = camera.offsets(Quat::IDENTITY, Vec3::NEG_Z).unwrap();
        assert!(eye.is_finite() && look.is_finite() && up.is_finite());
        assert!((eye - Vec3::new(0.0, -31.5, -5.7)).length() < 3.0);
        let forward = (look - eye).normalize();
        let cam_up = up.normalize();
        assert!(
            cam_up.dot(Vec3::NEG_Z) > 0.9,
            "chase camera rolled: up={cam_up:?}"
        );
        let to_planet = (Vec3::new(0.0, 0.0, 3_200_000.0) - eye).normalize();
        assert!(forward.dot(to_planet) > 0.0, "planet behind chase camera");
        assert!(forward.z > 0.15, "chase camera must look down-forward");
    }

    #[test]
    fn degenerate_frames_fail_closed_instead_of_publishing_nans() {
        let camera = FlightCamera::pilot_default();
        for (orientation, up) in [
            (Quat::IDENTITY, Vec3::ZERO),
            (Quat::IDENTITY, Vec3::new(f32::NAN, 0.0, 0.0)),
            (Quat::from_xyzw(f32::NAN, 0.0, 0.0, 1.0), Vec3::NEG_Z),
            (Quat::IDENTITY, Vec3::new(0.0, f32::INFINITY, 0.0)),
        ] {
            assert!(camera.offsets(orientation, up).is_none());
        }
    }

    #[test]
    fn long_sweeps_stay_unit_and_finite() {
        let mut camera = FlightCamera::pilot_default();
        for _ in 0..630 {
            camera.orbit_yaw(0.006);
        }
        assert!((camera.orbit.length_squared() - 1.0).abs() < 1e-6);
        let (eye, look, up) = camera.offsets(Quat::IDENTITY, Vec3::NEG_Z).unwrap();
        assert!(eye.is_finite() && look.is_finite() && up.is_finite());
    }
}
