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

/// Khronos PBR Neutral tone mapper: preserves saturation of foliage and avoids
/// hue skew or highlight clipping on bright translucent leaves and sky.
// Adapted from KhronosGroup/ToneMapping; Copyright 2024 The Khronos Group, Inc.
// Apache-2.0 (LICENSES/Apache-2.0.txt). Modified for Rust/WGSL native HDR.
pub fn pbr_neutral(rgb: [f32; 3]) -> [f32; 3] {
    let clean = rgb.map(|v| {
        let x = v.clamp(0.0, 65504.0);
        if x.is_nan() { 0.0 } else { x }
    });
    let start_compression = 0.8 - 0.04;
    let desaturation = 0.15;
    let x = clean[0].min(clean[1]).min(clean[2]);
    let offset = if x < 0.08 { x - 6.25 * x * x } else { 0.04 };
    let c = [clean[0] - offset, clean[1] - offset, clean[2] - offset];
    let peak = c[0].max(c[1]).max(c[2]);
    if peak < start_compression {
        return c;
    }
    let d = 1.0 - start_compression;
    let new_peak = 1.0 - d * d / (peak + d - start_compression);
    let scale = new_peak / peak;
    let c_scaled = c.map(|v| v * scale);
    let g = 1.0 - 1.0 / (desaturation * (peak - new_peak) + 1.0);
    [
        c_scaled[0] * (1.0 - g) + new_peak * g,
        c_scaled[1] * (1.0 - g) + new_peak * g,
        c_scaled[2] * (1.0 - g) + new_peak * g,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn neutral_preserves_the_khronos_low_range_offset_and_compression_boundary() {
        for (actual, expected) in pbr_neutral([0.04, 0.06, 0.08])
            .into_iter()
            .zip([0.01, 0.03, 0.05])
        {
            assert!((actual - expected).abs() < 1e-8);
        }
        let below = pbr_neutral([0.79999; 3])[0];
        let above = pbr_neutral([0.80001; 3])[0];
        assert!((above - below).abs() < 0.00003);
        assert_eq!(pbr_neutral([0.0; 3]), [0.0; 3]);
        assert_eq!(pbr_neutral([f32::NAN; 3]), [0.0; 3]);
    }
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
