//! 2D slice bench: a vertical cross-section through a synthetic cloud
//! street, marched top-down with 2D sun taps (the 1D column bench cannot see
//! horizontal neighbor occlusion — the main turret-modeling mechanism).
//! Prints an ASCII brightness map plus contrast stats; tune the lighting
//! constants here in seconds instead of screenshot cycles.
//!
//! Constants marked SYNC mirror `assets/shaders/cloud_volume.wesl`.

use thessa_atmosphere::clouds::{ambient_rgb, beer_powder, dual_lobe_phase, type_gradient};

// SYNC with cloud_volume.wesl. Units: per SLAB thickness (the shader's
// 900/radii over a 0.006-radii slab = 5.4 per slab here, since this bench
// marches in slab fractions, not radii).
const EXT: f64 = 6.5;
const SUN_GAIN: f64 = 5.0;
const PHASE_AMBIENT: f64 = 0.4;
const PHASE_GAIN: f64 = 12.0;
const FLOOR: f64 = 0.10;
const DETAIL_AMP: f64 = 1.6;
const AMBIENT_BOTTOM: [f64; 3] = [0.40, 0.46, 0.58];
const AMBIENT_TOP: [f64; 3] = [0.90, 0.96, 1.10];

fn hash2(ix: i64, iy: i64) -> f64 {
    let mut h = (ix as u64)
        .wrapping_mul(0x9E37_79B9_7F4A_7C15)
        .wrapping_add((iy as u64).wrapping_mul(0xBF58_476D_1CE4_E5B9))
        .wrapping_add(0xC10D);
    h ^= h >> 30;
    h = h.wrapping_mul(0xBF58_476D_1CE4_E5B9);
    h ^= h >> 27;
    h = h.wrapping_mul(0x94D0_49BB_1331_11EB);
    h ^= h >> 31;
    (h as f64) / (u64::MAX as f64)
}

fn smootherstep(t: f64) -> f64 {
    let t = t.clamp(0.0, 1.0);
    t * t * t * (t * (t * 6.0 - 15.0) + 10.0)
}

/// Tiling value noise in x (wraps `period`), clamped in y.
fn vnoise(x: f64, y: f64, period: i64) -> f64 {
    let xi = x.floor() as i64;
    let yi = y.floor() as i64;
    let xf = x - xi as f64;
    let yf = y - yi as f64;
    let wrap = |v: i64| v.rem_euclid(period.max(1));
    let a = hash2(wrap(xi), yi);
    let b = hash2(wrap(xi + 1), yi);
    let c = hash2(wrap(xi), yi + 1);
    let d = hash2(wrap(xi + 1), yi + 1);
    let u = smootherstep(xf);
    let v = smootherstep(yf);
    a + (b - a) * u + (c - a) * v + (a - b - c + d) * u * v
}

fn fbm(x: f64, y: f64, base_freq: i64, octaves: usize) -> f64 {
    let (mut sum, mut amp, mut norm) = (0.0, 0.5, 0.0);
    for o in 0..octaves {
        let f = base_freq * (1 << o);
        sum += amp * vnoise(x * f as f64, y * f as f64, f);
        norm += amp;
        amp *= 0.5;
    }
    sum / norm.max(1e-9)
}

/// 2D density over (x 0..1 = 600 km street, h 0..1 = slab).
fn density(x: f64, h: f64, type01: f64) -> f64 {
    // Large-scale coverage envelope: clear gaps and dense systems, or every
    // column marches the same overcast and contrast is dead on arrival.
    let cover = smoothstep_pub(0.25, 0.75, fbm(x * 2.0 + 7.7, 3.3, 2, 3));
    // Puff street: blobs across the domain, sheared with height so turrets
    // lean (vertical density structure from 2D noise).
    let xs = x + h * 0.15;
    let mut base = fbm(xs * 6.0, 0.5 + h * 2.0, 6, 4);
    base *= (0.15 + 0.85 * cover) * (0.65 + 0.45 * smoothstep_pub(0.2, 0.8, type01));
    // Ridged fines carve turrets (same shear: structure, not smear).
    let r1 = fbm((xs + 0.03) * 48.0 + 3.1, h * 24.0, 48, 1);
    let c1 = 1.0 - (r1 * 2.0 - 1.0).abs();
    let r2 = fbm((xs + 0.07) * 96.0 + 9.7, h * 48.0, 96, 1);
    let c2 = 1.0 - (r2 * 2.0 - 1.0).abs();
    let detail = (c1 * c1 * 0.7 + c2 * c2 * 0.3 + 0.35) / 1.0;
    base += (detail - 0.55) * DETAIL_AMP * (0.40 + 0.60 * h);
    let mut d = ((base - FLOOR) / (1.0 - FLOOR)).clamp(0.0, 1.0);
    d *= type_gradient(h, type01);
    d * (0.50 + 0.85 * type01) * 0.9
}

fn smoothstep_pub(e0: f64, e1: f64, x: f64) -> f64 {
    let t = ((x - e0) / (e1 - e0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// March one nadir column at street position `x0`. Sun from the +x side at
/// 45° elevation. Returns (rgb, alpha).
fn march_column(x0: f64, type01: f64) -> ([f64; 3], f64) {
    let steps = 12usize;
    let dt = 1.0 / steps as f64;
    // Aspect: street width 1.0 = 600 km, slab = 20 km → horizontal distances
    // scale by 1/30 in slab units.
    let aspect = 1.0 / 30.0;
    let sun = (
        std::f64::consts::FRAC_1_SQRT_2,
        std::f64::consts::FRAC_1_SQRT_2,
    ); // +x, up
    let cos_vs = -std::f64::consts::FRAC_1_SQRT_2; // nadir view vs sun
    let phase = dual_lobe_phase(cos_vs);
    let mut t = 1.0;
    let mut scat = [0.0; 3];
    for i in 0..steps {
        let h = 1.0 - (i as f64 + 0.5) / steps as f64;
        let d = density(x0, h, type01);
        if d <= 0.0 {
            continue;
        }
        // Two sun taps along the sun ray + half-step self term.
        let tap = |s: f64| {
            let px = x0 + sun.0 * s * aspect;
            let ph = (h + sun.1 * s).clamp(0.0, 1.0);
            density(px.rem_euclid(1.0), ph, type01)
        };
        let span = 0.8;
        let sun_depth = ((tap(span * 0.5) * 0.6 + tap(span) * 0.4) * span + d * dt * 0.5) * EXT;
        let bp = beer_powder(sun_depth);
        let sun_l = bp * (PHASE_AMBIENT + PHASE_GAIN * phase) * SUN_GAIN;
        let amb = ambient_rgb(h, t, AMBIENT_BOTTOM, AMBIENT_TOP);
        let absorb = (-EXT * d * dt).exp();
        for c in 0..3 {
            scat[c] += t * (sun_l + amb[c]) * d * dt * EXT * absorb;
        }
        t *= absorb;
        if t < 0.02 {
            t = 0.0;
            break;
        }
    }
    let alpha = (1.0 - t).clamp(0.0, 1.0);
    let n = alpha.max(1e-3);
    ([scat[0] / n, scat[1] / n, scat[2] / n], alpha)
}

#[test]
fn slice_contrast_turrets_model_crevices() {
    const W: usize = 64;
    eprintln!(
        "probe dens(0.5,0.35)={:.3} dens(0.1,0.5)={:.3} fbm={:.3}",
        density(0.5, 0.35, 0.6),
        density(0.1, 0.5, 0.6),
        fbm(3.0, 1.2, 6, 4)
    );
    let mut row = String::new();
    let mut lumas = Vec::new();
    for i in 0..W {
        let (rgb, alpha) = march_column(i as f64 / W as f64, 0.6);
        let luma = (rgb[0] + rgb[1] + rgb[2]) / 3.0 * alpha.clamp(0.15, 1.0);
        lumas.push(luma);
        row.push(if luma < 0.15 {
            ' '
        } else if luma < 0.4 {
            '.'
        } else if luma < 0.8 {
            'o'
        } else if luma < 1.4 {
            'O'
        } else {
            '#'
        });
    }
    eprintln!("street: {row}");
    let max = lumas.iter().cloned().fold(0.0f64, f64::max);
    let min = lumas.iter().cloned().fold(f64::INFINITY, f64::min);
    let mean = lumas.iter().sum::<f64>() / lumas.len() as f64;
    eprintln!(
        "max={max:.2} min={min:.2} mean={mean:.2} contrast={:.1}",
        max / min.max(1e-3)
    );
    // Internal modeling only: the render adds dark-ocean contrast on top
    // (gaps show ~0.05 ocean, not veil), so the bench gate is 2.8, not 10.
    assert!(max > 1.2, "tops must run hot white, max={max:.2}");
    assert!(
        max / min.max(1e-3) >= 2.8,
        "turrets need modeled contrast, got {:.1}",
        max / min.max(1e-3)
    );
}
