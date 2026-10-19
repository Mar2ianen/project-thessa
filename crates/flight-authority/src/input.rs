//! Existing pilot flight-input conditioning, shared by the Bevy pilot and the
//! native migration viewer. Pure intent math only: stick-key mapping, throttle
//! slew/presets, Mouse Aim deadband/selection, SAS demand and precision. No input devices,
//! clocks, ECS or authority handles; hosts own keys, timing and application.
//!
//! Conventions are inherited unchanged: stick `x > 0` is stick-back (S key,
//! nose up), `y > 0` yaws right (D key), `z > 0` rolls with E. Throttle slews
//! at 0.55/s while held and clamps to `0..=1`. Invalid demands leave the
//! previous intent untouched (fail closed, never NaN).
use glam::{DVec3, Vec2, Vec3};

/// Throttle slew rate while Shift/Ctrl is held (fraction per second).
pub const THROTTLE_SLEW_PER_S: f64 = 0.55;
/// Stick scaling while precision controls hold.
pub const PRECISION_FACTOR: f64 = 0.25;

/// Held stick keys, named by effect. Hosts translate their own key codes
/// (Bevy `KeyCode`, winit `PhysicalKey`) into this neutral set.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct StickKeys {
    /// S key: stick-back, nose up (`+x`).
    pub nose_up: bool,
    /// W key: stick-forward, nose down (`-x`).
    pub nose_down: bool,
    /// D key: yaw right (`+y`).
    pub yaw_right: bool,
    /// A key: yaw left (`-y`).
    pub yaw_left: bool,
    /// E key: roll (`+z`).
    pub roll_right: bool,
    /// Q key: roll (`-z`).
    pub roll_left: bool,
}

impl StickKeys {
    /// True when any stick axis is held (manual takeover signal).
    pub fn any(self) -> bool {
        self.nose_up
            || self.nose_down
            || self.yaw_right
            || self.yaw_left
            || self.roll_right
            || self.roll_left
    }
}

/// Stick vector from held keys: `(S-W, D-A, E-Q)` as `f64`. Matches the
/// shipped keyboard mapping exactly, including simultaneous opposites
/// cancelling on an axis.
pub fn stick_input(keys: StickKeys) -> DVec3 {
    DVec3::new(
        (keys.nose_up as i8 - keys.nose_down as i8) as f64,
        (keys.yaw_right as i8 - keys.yaw_left as i8) as f64,
        (keys.roll_right as i8 - keys.roll_left as i8) as f64,
    )
}

/// One throttle slew step. Non-finite or negative `dt_s` leaves the setting
/// untouched; the result always stays in `0..=1`.
pub fn slew_throttle(throttle: f64, dt_s: f64, up: bool, down: bool) -> f64 {
    if !throttle.is_finite() || !dt_s.is_finite() || dt_s < 0.0 {
        return throttle;
    }
    (throttle + (up as i8 - down as i8) as f64 * dt_s * THROTTLE_SLEW_PER_S).clamp(0.0, 1.0)
}

/// Precision scaling for manual stick commands.
pub fn apply_precision(input: DVec3, precision: bool) -> DVec3 {
    input * if precision { PRECISION_FACTOR } else { 1.0 }
}

/// Shipped Mouse Aim conditioner: screen y down, 6% NDC deadband and a
/// normalized forward direction. Presentation intent only, never craft pose.
/// Cursor and viewport use the same coordinate units (pixels or UI points).
/// Missing/invalid inputs are neutral, not the last nonzero steering demand.
pub fn mouse_aim_direction(cursor: Option<[f32; 2]>, viewport: [f32; 2]) -> Vec3 {
    let Some(cursor) = cursor else {
        return Vec3::NEG_Z;
    };
    let cursor = Vec2::from_array(cursor);
    let size = Vec2::from_array(viewport);
    if !cursor.is_finite() || !size.is_finite() || size.min_element() <= 0.0 {
        return Vec3::NEG_Z;
    }
    let ndc = Vec2::new(
        (cursor.x / size.x * 2.0 - 1.0).clamp(-1.0, 1.0),
        (1.0 - cursor.y / size.y * 2.0).clamp(-1.0, 1.0),
    );
    let steer = (ndc.abs() - Vec2::splat(0.06)).max(Vec2::ZERO) / 0.94 * ndc.signum();
    Vec3::new(steer.x, steer.y, -1.0).normalize()
}

/// Manual keyboard takes precedence over Mouse Aim, including in assisted
/// modes. Clamp the complete vector before precision, as in the shipped pilot.
pub fn manual_control_input(
    keyboard: DVec3,
    direction: Vec3,
    mouse_aim: bool,
    precision: bool,
) -> DVec3 {
    let axes = if mouse_aim && keyboard.length_squared() < 1e-8 {
        DVec3::new(f64::from(direction.y), f64::from(direction.x), 0.0) * 0.85
    } else {
        keyboard
    };
    if !axes.is_finite() {
        return DVec3::ZERO;
    }
    apply_precision(axes.clamp_length_max(1.0), precision)
}

/// Bundled live input intent for hosts that step the authority directly
/// (the native viewer). The Bevy pilot keeps its split
/// intent-vs-acknowledgement fields and uses the free functions above.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlightInput {
    /// Commanded throttle fraction `0..=1`.
    pub throttle: f64,
    /// Commanded engine state.
    pub engine_active: bool,
    /// Held stick keys.
    pub keys: StickKeys,
    /// Precision scaling engaged.
    pub precision: bool,
    pub mouse_direction: Vec3,
    /// Base SAS demand (the lean server snapshot does not acknowledge SAS).
    pub sas_enabled: bool,
    /// Temporary hold-to-invert. Releasing focus or the key must clear it.
    pub sas_inverted: bool,
}

impl Default for FlightInput {
    fn default() -> Self {
        Self {
            throttle: 0.0,
            engine_active: false,
            keys: StickKeys::default(),
            precision: false,
            mouse_direction: Vec3::NEG_Z,
            sas_enabled: true,
            sas_inverted: false,
        }
    }
}

impl FlightInput {
    /// Checked constructor; rejects non-finite or out-of-range throttle.
    pub fn new(throttle: f64, engine_active: bool) -> Option<Self> {
        if !throttle.is_finite() || !(0.0..=1.0).contains(&throttle) {
            return None;
        }
        Some(Self {
            throttle,
            engine_active,
            ..Default::default()
        })
    }

    /// Slider/host restore with legacy clamping. Ignores non-finite input.
    pub fn set_throttle(&mut self, throttle: f64) {
        if throttle.is_finite() {
            self.throttle = throttle.clamp(0.0, 1.0);
        }
    }

    /// Hold-to-slew the throttle. Invalid steps are ignored.
    pub fn slew(&mut self, dt_s: f64, up: bool, down: bool) {
        self.throttle = slew_throttle(self.throttle, dt_s, up, down);
    }

    pub fn cut(&mut self) {
        self.throttle = 0.0;
    }

    pub fn full(&mut self) {
        self.throttle = 1.0;
    }

    pub fn toggle_engine(&mut self) {
        self.engine_active = !self.engine_active;
    }

    pub fn toggle_precision(&mut self) {
        self.precision = !self.precision;
    }

    /// Raw keyboard-only intermediate, retained for mapping diagnostics.
    /// Hosts use `command_scaled` for the final unit-limited authority demand.
    pub fn stick_scaled(self) -> DVec3 {
        apply_precision(stick_input(self.keys), self.precision)
    }

    pub fn command_scaled(self, mouse_aim: bool) -> DVec3 {
        manual_control_input(
            stick_input(self.keys),
            self.mouse_direction,
            mouse_aim,
            self.precision,
        )
    }

    pub fn effective_sas(self) -> bool {
        self.sas_enabled ^ self.sas_inverted
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mouse_conditioner_matches_the_shipped_deadband_and_axis_convention() {
        for size in [
            Vec2::new(1280.0, 720.0),
            Vec2::new(640.0, 360.0),
            Vec2::new(720.0, 1280.0),
        ] {
            for x in 0..=20 {
                for y in 0..=20 {
                    let cursor = Vec2::new(x as f32 / 20.0, y as f32 / 20.0) * size;
                    // Independent transcription of the old pilot's screen-space
                    // conditioner, before selecting keyboard versus cursor input.
                    let ndc = Vec2::new(
                        (cursor.x / size.x * 2.0 - 1.0).clamp(-1.0, 1.0),
                        (1.0 - cursor.y / size.y * 2.0).clamp(-1.0, 1.0),
                    );
                    let steer =
                        (ndc.abs() - Vec2::splat(0.06)).max(Vec2::ZERO) / 0.94 * ndc.signum();
                    let expected = Vec3::new(steer.x, steer.y, -1.0).normalize();
                    assert_eq!(
                        mouse_aim_direction(Some(cursor.to_array()), size.to_array()),
                        expected
                    );
                }
            }
        }
        let size = [1000.0, 1000.0];
        for cursor in [
            None,
            Some([500.0, 500.0]),
            Some([515.0, 485.0]),
            Some([f32::NAN, 0.0]),
        ] {
            assert_eq!(mouse_aim_direction(cursor, size), Vec3::NEG_Z);
        }
        for size in [[0.0, 10.0], [-1.0, 10.0], [10.0, f32::INFINITY]] {
            assert_eq!(mouse_aim_direction(Some([0.0, 0.0]), size), Vec3::NEG_Z);
        }
        let top = mouse_aim_direction(Some([500.0, 0.0]), size);
        let right = mouse_aim_direction(Some([1000.0, 500.0]), size);
        assert!(
            (manual_control_input(DVec3::ZERO, top, true, false).x - 0.85 / 2.0_f64.sqrt()).abs()
                < 1e-7
        );
        assert!(
            (manual_control_input(DVec3::ZERO, right, true, false).y - 0.85 / 2.0_f64.sqrt()).abs()
                < 1e-7
        );
        assert_eq!(manual_control_input(DVec3::X, right, true, false), DVec3::X);
        assert_eq!(
            manual_control_input(DVec3::ZERO, right, false, false),
            DVec3::ZERO
        );
        assert_eq!(
            manual_control_input(DVec3::ONE, right, true, true),
            DVec3::ONE.normalize() * 0.25
        );
        assert_eq!(
            manual_control_input(DVec3::ZERO, Vec3::NAN, true, false),
            DVec3::ZERO
        );
    }

    #[test]
    fn temporary_sas_is_an_inversion_of_retained_intent_not_an_extra_toggle() {
        let mut input = FlightInput::default();
        assert!(input.effective_sas());
        input.sas_inverted = true;
        assert!(!input.effective_sas());
        input.sas_inverted = true;
        assert!(!input.effective_sas(), "repeats do not flip demand");
        input.sas_enabled = false;
        assert!(
            input.effective_sas(),
            "base demand may change while the override is held"
        );
        input.sas_inverted = false;
        assert!(!input.effective_sas());
    }

    #[test]
    fn stick_mapping_matches_the_shipped_wasdqe_table() {
        let key = |nose_up: bool,
                   nose_down: bool,
                   yaw_right: bool,
                   yaw_left: bool,
                   roll_right: bool,
                   roll_left: bool| StickKeys {
            nose_up,
            nose_down,
            yaw_right,
            yaw_left,
            roll_right,
            roll_left,
        };
        // (keys, expected): S=stick-back nose-up +x, W forward -x, D yaw +y,
        // A -y, E roll +z, Q -z; opposites cancel per axis.
        for (keys, expected, held) in [
            (
                key(false, false, false, false, false, false),
                DVec3::ZERO,
                false,
            ),
            (
                key(false, true, false, false, false, false),
                DVec3::NEG_X,
                true,
            ),
            (key(true, false, false, false, false, false), DVec3::X, true),
            (key(false, false, true, false, false, false), DVec3::Y, true),
            (
                key(false, false, false, true, false, false),
                DVec3::NEG_Y,
                true,
            ),
            (key(false, false, false, false, true, false), DVec3::Z, true),
            (
                key(false, false, false, false, false, true),
                DVec3::NEG_Z,
                true,
            ),
            (key(true, true, true, true, true, true), DVec3::ZERO, true),
            (
                key(true, false, true, false, false, true),
                DVec3::new(1.0, 1.0, -1.0),
                true,
            ),
        ] {
            assert_eq!(stick_input(keys), expected);
            assert_eq!(keys.any(), held);
        }
    }

    #[test]
    fn throttle_slew_presets_and_toggles_hold_the_legacy_policy() {
        assert_eq!((THROTTLE_SLEW_PER_S, PRECISION_FACTOR), (0.55, 0.25));
        assert_eq!(slew_throttle(0.0, 1.0, true, false), 0.55);
        assert_eq!(slew_throttle(0.5, 0.1, false, true), 0.5 - 0.055);
        assert_eq!(slew_throttle(0.9, 1.0, true, false), 1.0);
        assert_eq!(slew_throttle(0.1, 1.0, false, true), 0.0);
        assert_eq!(slew_throttle(0.4, 1.0, true, true), 0.4);
        assert_eq!(slew_throttle(0.4, 0.0, true, false), 0.4);
        // Invalid steps never corrupt the setting.
        for dt in [f64::NAN, f64::INFINITY, -1.0] {
            assert_eq!(slew_throttle(0.4, dt, true, false), 0.4);
        }
        assert!(slew_throttle(f64::NAN, 0.1, true, false).is_nan());
        let mut input = FlightInput::new(0.2, false).unwrap();
        input.slew(2.0, true, false);
        assert_eq!(input.throttle, 1.0);
        input.cut();
        assert_eq!(input.throttle, 0.0);
        input.full();
        assert_eq!(input.throttle, 1.0);
        input.toggle_engine();
        assert!(input.engine_active);
        input.toggle_engine();
        assert!(!input.engine_active);
        input.keys.nose_up = true;
        input.keys.yaw_right = true;
        assert_eq!(input.stick_scaled(), DVec3::new(1.0, 1.0, 0.0));
        input.toggle_precision();
        assert_eq!(input.stick_scaled(), DVec3::new(0.25, 0.25, 0.0));
        assert!(FlightInput::new(f64::NAN, false).is_none());
        assert!(FlightInput::new(1.5, false).is_none());
        assert!(FlightInput::new(-0.1, false).is_none());
    }

    #[test]
    fn long_held_slew_converges_and_stays_bounded() {
        let mut throttle = 0.0;
        for _ in 0..10_000 {
            throttle = slew_throttle(throttle, 1.0 / 60.0, true, false);
        }
        assert_eq!(throttle, 1.0);
        for _ in 0..10_000 {
            throttle = slew_throttle(throttle, 1.0 / 60.0, false, true);
        }
        assert_eq!(throttle, 0.0);
    }
}
