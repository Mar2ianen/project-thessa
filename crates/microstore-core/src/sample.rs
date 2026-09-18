//! Bilinear sampling over scalar fields (CPU reference for the GPU
//! sample-time path).
//!
//! Coordinates are normalized UV in `[0, 1]`; out-of-range UVs clamp to
//! the edge texel, matching the shader. All arithmetic is `f32` so the
//! CPU mirror rounds like the WGSL baseline (which has no f64).

use crate::codec::ScalarField;

/// Bilinear sample of one field at normalized UV, returned in code levels
/// (`0..=255` as `f32`, unrounded).
pub fn sample_bilinear(field: &ScalarField, u: f32, v: f32) -> f32 {
    let w = field.width as f32;
    let h = field.height as f32;
    let x = (u.clamp(0.0, 1.0) * (w - 1.0)).clamp(0.0, w - 1.0);
    let y = (v.clamp(0.0, 1.0) * (h - 1.0)).clamp(0.0, h - 1.0);
    let x0 = x.floor();
    let y0 = y.floor();
    let x1 = (x0 + 1.0).min(w - 1.0);
    let y1 = (y0 + 1.0).min(h - 1.0);
    let fx = x - x0;
    let fy = y - y0;
    let at =
        |xx: f32, yy: f32| field.data[(yy as usize) * field.width as usize + xx as usize] as f32;
    let a = at(x0, y0);
    let b = at(x1, y0);
    let c = at(x0, y1);
    let d = at(x1, y1);
    a * (1.0 - fx) * (1.0 - fy) + b * fx * (1.0 - fy) + c * (1.0 - fx) * fy + d * fx * fy
}

/// Bilinear sample rounded to a code level (the GPU kernel output shape).
pub fn sample_rounded(field: &ScalarField, u: f32, v: f32) -> u8 {
    sample_bilinear(field, u, v).round().clamp(0.0, 255.0) as u8
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fixtures;

    #[test]
    fn corners_and_center_sample_exactly() {
        // 2x2 field [0, 64, 128, 255]: corners reproduce, center averages.
        let field = ScalarField::new(2, 2, vec![0, 64, 128, 255]).unwrap();
        assert_eq!(sample_bilinear(&field, 0.0, 0.0), 0.0);
        assert_eq!(sample_bilinear(&field, 1.0, 0.0), 64.0);
        assert_eq!(sample_bilinear(&field, 0.0, 1.0), 128.0);
        assert_eq!(sample_bilinear(&field, 1.0, 1.0), 255.0);
        assert_eq!(
            sample_bilinear(&field, 0.5, 0.5),
            (0.0 + 64.0 + 128.0 + 255.0) / 4.0
        );
    }

    #[test]
    fn out_of_range_uv_clamps_to_edge() {
        let field = fixtures::gradient(16, 16);
        assert_eq!(
            sample_bilinear(&field, -2.0, 0.5),
            sample_bilinear(&field, 0.0, 0.5)
        );
        assert_eq!(
            sample_bilinear(&field, 1.0, 99.0),
            sample_bilinear(&field, 1.0, 1.0)
        );
    }

    #[test]
    fn uniform_samples_constant() {
        let field = fixtures::uniform(24, 24, 77);
        for (u, v) in [(0.0, 0.0), (0.33, 0.71), (1.0, 1.0), (0.5, 0.5)] {
            let s = sample_bilinear(&field, u, v);
            assert!((76.0..=78.0).contains(&s), "{s}");
        }
    }

    #[test]
    fn rounded_output_is_byte() {
        let field = fixtures::noise(32, 32, 9);
        for i in 0..64 {
            let u = (i as f32 * 0.15973) % 1.0;
            let v = (i as f32 * 0.36711) % 1.0;
            let _ = sample_rounded(&field, u, v);
        }
    }
}
