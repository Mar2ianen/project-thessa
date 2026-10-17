//! Display math only; no graphics handles, scene policy or simulation changes.

/// Manual EV100 convention shared by the existing PBR host. Resolution clamps
/// input to [-16, 32]; invalid direct callers receive the default daylight EV13.
pub fn manual_exposure(ev100: f32) -> f32 {
    let ev = if ev100.is_finite() {
        ev100.clamp(-16.0, 32.0)
    } else {
        13.0
    };
    1.0 / (1.2 * 2.0f32.powf(ev))
}

/// Reinhard (2002), per-channel global operator on pre-exposed linear RGB.
/// Negative/NaN radiance is black; values beyond the half-float target's finite
/// range saturate safely before the rational map. sRGB encoding belongs to the
/// output attachment, not this function. This is not an ACES/AgX or Bevy match.
pub fn reinhard(rgb: [f32; 3]) -> [f32; 3] {
    rgb.map(|v| {
        let x = v.clamp(0.0, 65504.0);
        if x.is_nan() { 0.0 } else { x / (1.0 + x) }
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn exposure_convention_stops_and_invalid_inputs() {
        assert!((manual_exposure(13.0) - 1.0 / (1.2 * 8192.0)).abs() < 1e-10);
        assert_eq!(manual_exposure(1.0), manual_exposure(0.0) / 2.0);
        assert_eq!(manual_exposure(f32::NAN), manual_exposure(13.0));
        assert_eq!(manual_exposure(1000.0), manual_exposure(32.0));
        assert_eq!(manual_exposure(-1000.0), manual_exposure(-16.0));
    }
    #[test]
    fn reinhard_special_cases_monotonic_highlights_and_fault_containment() {
        assert_eq!(reinhard([0.0, 1.0, 3.0]), [0.0, 0.5, 0.75]);
        assert_eq!(reinhard([-1.0, f32::NAN, f32::NEG_INFINITY]), [0.0; 3]);
        assert_eq!(reinhard([f32::INFINITY; 3]), reinhard([65504.0; 3]));
        let mut previous = 0.0;
        for i in 0..4096 {
            let x = i as f32 / 32.0;
            let y = reinhard([x; 3])[0];
            assert!(y >= previous && y < 1.0);
            previous = y;
        }
        assert!(reinhard([4.0; 3])[0] < reinhard([16.0; 3])[0]);
    }
}
