//! Height encoding: piecewise-linear datum-centered grayscale.
//!
//! Source grayscale `g` in [0,1] maps as:
//!   [0.0, 0.5]  => [height_min_m, 0.0]      (datum at exactly g = 0.5)
//!   [0.5, 1.0]  => [0.0, height_max_m]
//!
//! The old "128 = 0 with linear [-8000, +12000]" claim was self-contradictory
//! for asymmetric ranges, so it is replaced by this documented mapping.
//! Decode to f64 metres at import; every later stage works in metres.

pub const DATUM_GRAY: f64 = 0.5;

/// Decode normalized grayscale to metres.
pub fn decode_height_m(gray01: f64, height_min_m: f64, height_max_m: f64) -> f64 {
    let g = gray01.clamp(0.0, 1.0);
    if g <= DATUM_GRAY {
        height_min_m * (1.0 - g / DATUM_GRAY)
    } else {
        height_max_m * ((g - DATUM_GRAY) / (1.0 - DATUM_GRAY))
    }
}

/// Encode metres back to normalized grayscale (for preview/export round-trip).
pub fn encode_height_01(height_m: f64, height_min_m: f64, height_max_m: f64) -> f64 {
    if height_m <= 0.0 {
        if height_min_m >= 0.0 {
            return 0.0;
        }
        DATUM_GRAY * (1.0 - (height_m / height_min_m).clamp(0.0, 1.0))
    } else {
        if height_max_m <= 0.0 {
            return 1.0;
        }
        DATUM_GRAY + (1.0 - DATUM_GRAY) * (height_m / height_max_m).clamp(0.0, 1.0)
    }
}

/// Max round-trip error in metres for asymmetric ranges at 8-bit quantization.
pub fn roundtrip_error_bound_m(height_min_m: f64, height_max_m: f64) -> f64 {
    let below = height_min_m.abs() / 256.0;
    let above = height_max_m.abs() / 254.0;
    below.max(above) + 1e-9
}

#[cfg(test)]
mod tests {
    use super::*;

    const MIN: f64 = -8000.0;
    const MAX: f64 = 12000.0;

    #[test]
    fn endpoints_decode_exactly() {
        assert_eq!(decode_height_m(0.0, MIN, MAX), MIN);
        assert_eq!(decode_height_m(0.5, MIN, MAX), 0.0);
        assert_eq!(decode_height_m(1.0, MIN, MAX), MAX);
    }

    #[test]
    fn datum_is_continuous_from_both_sides() {
        let below = decode_height_m(0.5 - 1e-12, MIN, MAX);
        let above = decode_height_m(0.5 + 1e-12, MIN, MAX);
        assert!(below < 0.0 && below > -1.0);
        assert!(above > 0.0 && above < 1.0);
    }

    #[test]
    fn asymmetric_quarters_are_not_naively_linear() {
        // Naive linear [-8000,+12000] would put g=0.25 at -3000; piecewise gives -4000.
        assert_eq!(decode_height_m(0.25, MIN, MAX), -4000.0);
        assert_eq!(decode_height_m(0.75, MIN, MAX), 6000.0);
    }

    #[test]
    fn roundtrip_stays_within_quantization_bound() {
        let bound = roundtrip_error_bound_m(MIN, MAX);
        // 8-bit gray steps.
        for step in 0..=255u32 {
            let g = f64::from(step) / 255.0;
            let h = decode_height_m(g, MIN, MAX);
            let g2 = encode_height_01(h, MIN, MAX);
            let h2 = decode_height_m(g2, MIN, MAX);
            assert!(
                (h - h2).abs() <= bound,
                "step {step}: {h} vs {h2} (bound {bound})"
            );
        }
    }

    #[test]
    fn out_of_range_grays_clamp() {
        assert_eq!(decode_height_m(-1.0, MIN, MAX), MIN);
        assert_eq!(decode_height_m(2.0, MIN, MAX), MAX);
    }
}
