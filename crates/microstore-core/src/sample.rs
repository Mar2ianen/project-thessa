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

/// UV Jacobian per screen pixel: how fast normalized UV changes along
/// screen x/y. A real shader reads these from hardware derivatives; the
/// prototype takes them as inputs so LOD and anisotropy stay testable
/// without a rasterizer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UvJacobian {
    /// d(uv)/dx.
    pub dudx: f32,
    /// d(uv)/dy.
    pub dudy: f32,
    /// Second row is implicit unit for axis probes; full form below.
    pub dvdx: f32,
    /// d(v)/dy.
    pub dvdy: f32,
}

/// Mip level for a footprint (standard max-footprint rule): half the
/// log2 of the larger squared texel footprint, clamped to the chain.
/// `size` is `(width, height)` of level 0 in texels.
pub fn lod_level(jac: &UvJacobian, size: [f32; 2], max_level: u32) -> u32 {
    let px = jac.dudx * size[0];
    let py = jac.dudy * size[0];
    let qx = jac.dvdx * size[1];
    let qy = jac.dvdy * size[1];
    let rho_sq = (px * px + py * py).max(qx * qx + qy * qy);
    if rho_sq <= 1.0 {
        return 0;
    }
    (0.5 * rho_sq.log2()).floor().clamp(0.0, max_level as f32) as u32
}

/// Anisotropy ratio: major over minor footprint axis, at least 1.
/// Drives tap counts (1/2/4/8) in [`sample_aniso`].
pub fn aniso_ratio(jac: &UvJacobian, size: [f32; 2]) -> f32 {
    let major_sq = ((jac.dudx * size[0]).powi(2) + (jac.dudy * size[0]).powi(2))
        .max((jac.dvdx * size[1]).powi(2) + (jac.dvdy * size[1]).powi(2));
    let minor_sq = ((jac.dudx * size[0]).powi(2) + (jac.dudy * size[0]).powi(2))
        .min((jac.dvdx * size[1]).powi(2) + (jac.dvdy * size[1]).powi(2));
    if minor_sq <= 1e-12 {
        return major_sq.sqrt().max(1.0);
    }
    (major_sq / minor_sq).sqrt().max(1.0)
}

/// Anisotropic sample: `taps` bilinear probes spread across one pixel
/// footprint along its major axis, averaged. `taps` rounds down to
/// {1, 2, 4, 8}; 1 tap is plain bilinear. All arithmetic is f32 to mirror
/// the shader.
pub fn sample_aniso(field: &ScalarField, u: f32, v: f32, jac: &UvJacobian, taps: u32) -> f32 {
    let taps = match taps {
        0 | 1 => 1,
        2 => 2,
        3 | 4 => 4,
        _ => 8,
    };
    if taps == 1 {
        return sample_bilinear(field, u, v);
    }
    // Major axis in UV units: the longer of the two Jacobian rows,
    // normalized; taps span one pixel footprint along it.
    let ax = (jac.dudx, jac.dudy);
    let bx = (jac.dvdx, jac.dvdy);
    let a_len_sq = ax.0 * ax.0 + ax.1 * ax.1;
    let b_len_sq = bx.0 * bx.0 + bx.1 * bx.1;
    let (dx, dy) = if a_len_sq >= b_len_sq { ax } else { bx };
    let len = (dx * dx + dy * dy).sqrt().max(1e-12);
    let (dx, dy) = (dx / len, dy / len);
    // Tap centers tile half a pixel footprint to each side along the
    // major axis; `len` converts the unit direction back to UV units.
    let mut sum = 0.0f32;
    for i in 0..taps {
        let t = (i as f32 + 0.5) / taps as f32 - 0.5;
        sum += sample_bilinear(field, u + dx * t * len, v + dy * t * len);
    }
    sum / taps as f32
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

    fn axis_jac(dudx: f32, dvdy: f32) -> UvJacobian {
        UvJacobian {
            dudx,
            dudy: 0.0,
            dvdx: 0.0,
            dvdy,
        }
    }

    #[test]
    fn lod_selects_by_max_footprint() {
        // 64x64 field: one-texel footprints stay at level 0.
        assert_eq!(
            lod_level(&axis_jac(1.0 / 64.0, 1.0 / 64.0), [64.0, 64.0], 7),
            0
        );
        // Four-texel footprints -> level 2.
        assert_eq!(
            lod_level(&axis_jac(4.0 / 64.0, 4.0 / 64.0), [64.0, 64.0], 7),
            2
        );
        // Anisotropic stretch takes the major axis (8 texels -> level 3).
        assert_eq!(
            lod_level(&axis_jac(8.0 / 64.0, 1.0 / 64.0), [64.0, 64.0], 7),
            3
        );
        // Clamp to the chain length.
        assert_eq!(lod_level(&axis_jac(8.0, 8.0), [64.0, 64.0], 3), 3);
        // Sub-texel footprints stay at level 0, never negative.
        assert_eq!(lod_level(&axis_jac(0.001, 0.001), [64.0, 64.0], 7), 0);
    }

    #[test]
    fn aniso_ratio_measures_stretch() {
        assert_eq!(
            aniso_ratio(&axis_jac(1.0 / 64.0, 1.0 / 64.0), [64.0, 64.0]),
            1.0
        );
        assert!((aniso_ratio(&axis_jac(8.0 / 64.0, 2.0 / 64.0), [64.0, 64.0]) - 4.0).abs() < 1e-5);
    }

    #[test]
    fn aniso_tap_counts_quantize() {
        let field = fixtures::gradient(16, 16);
        let jac = axis_jac(2.0 / 16.0, 2.0 / 16.0);
        // 1 tap (and 0) reproduce plain bilinear exactly.
        assert_eq!(
            sample_aniso(&field, 0.3, 0.4, &jac, 1),
            sample_bilinear(&field, 0.3, 0.4)
        );
        assert_eq!(
            sample_aniso(&field, 0.3, 0.4, &jac, 0),
            sample_bilinear(&field, 0.3, 0.4)
        );
    }

    #[test]
    fn aniso_kills_checker_aliasing() {
        // 1-texel checker sampled at a texel center with an 8-texel
        // footprint: bilinear aliases to an extreme, aniso averages out.
        let field = fixtures::checker(16, 16, 1);
        let u = 4.0 / 15.0;
        let v = 8.0 / 15.0;
        let jac = axis_jac(8.0 / 15.0, 8.0 / 15.0);
        let point = sample_bilinear(&field, u, v);
        assert!(point == 0.0 || point == 255.0, "aliased {point}");
        let filtered = sample_aniso(&field, u, v, &jac, 8);
        assert!(
            (100.0..=155.0).contains(&filtered),
            "antialiased {filtered}"
        );
    }
}
