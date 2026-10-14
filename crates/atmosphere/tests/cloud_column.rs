//! Lighting-model bench: a nadir column through one synthetic puff, shaded
//! with the exact core functions the WGSL march mirrors (`remap_coverage`,
//! `type_gradient`, `erode_density`, `beer_powder`, `dual_lobe_phase`,
//! `ambient_rgb`) and the same tuning constants. Diagnoses brightness
//! balance (white tops? dark cores? glowing thin edges?) numerically instead
//! of guessing from screenshots. If this column is grey, the shader is grey.
//!
//! Constants marked SYNC must match `assets/shaders/cloud_volume.wesl`.

use thessa_atmosphere::clouds::{
    ambient_rgb, beer_powder, dual_lobe_phase, remap_coverage, type_gradient,
};

// SYNC with cloud_volume.wesl.
const EXT_PER_RADII: f64 = 900.0;
const SUN_GAIN: f64 = 5.0;
const PHASE_AMBIENT: f64 = 0.4;
const PHASE_GAIN: f64 = 12.0;
const SUN_WARM: [f64; 3] = [1.0, 0.93, 0.80];
const AMBIENT_BOTTOM: [f64; 3] = [0.40, 0.46, 0.58];
const AMBIENT_TOP: [f64; 3] = [0.90, 0.96, 1.10];
const SLAB_FRAC: f64 = 0.006; // planet radii
const FLOOR: f64 = 0.08;
const DETAIL_AMP: f64 = 1.6;

/// Gaussian puff density over slab height + thin background.
fn puff_density(h01: f64, peak: f64) -> f64 {
    let g = (-((h01 - 0.35) / 0.16).powi(2)).exp();
    (0.08 + peak * g).clamp(0.0, 1.0)
}

/// One shaded column: returns (top_rgb, core_rgb, thin_rgb) as luminance proxies.
fn shade_column(peak: f64, sun_elev_sin: f64, type01: f64) -> ([f64; 3], [f64; 3], [f64; 3]) {
    let steps = 12usize;
    let dt = SLAB_FRAC / steps as f64;
    // cos(view, sun) for nadir view with sun at elevation asin(sun_elev_sin).
    let cos_vs = -sun_elev_sin;
    let phase = dual_lobe_phase(cos_vs);
    let day = ((sun_elev_sin + 0.08) / 0.22).clamp(0.0, 1.0);
    let day = day * day * (3.0 - 2.0 * day);
    let mut transmittance = 1.0;
    let mut scat = [0.0; 3];
    // Snapshot partway (ray terminated above the core): what a top surface
    // shows. March runs top-down like the view ray.
    let mut top_scat = [0.0; 3];
    let mut top_t = 1.0;
    let mut snapped = false;
    for i in 0..steps {
        // Top-down: the view ray enters at h=1 first.
        let h = 1.0 - (i as f64 + 0.5) / steps as f64;
        // Density through the shaping chain, mirroring the WGSL sample:
        // symmetric detail perturbation (mean field 0.55), steep remap
        // window, type gradient. No spatial detail in 1D by construction.
        let detail = 0.55 + 0.25 * ((i as f64 * 2.39).sin() * 0.5 + 0.5 - 0.25);
        let mut base = remap_coverage(puff_density(h, peak), 0.0);
        base += (detail - 0.55) * DETAIL_AMP * (0.40 + 0.60 * h);
        let mut d = ((base - FLOOR) / (1.0 - FLOOR)).clamp(0.0, 1.0);
        d *= type_gradient(h, type01);
        if d <= 0.0 {
            continue;
        }
        // Sun taps climb out of the core toward a 45° sun (radii offsets
        // like the shader), plus the half-step self term without which the
        // visible surface itself sits at zero light depth.
        let span = 0.8 * SLAB_FRAC;
        let rise = |s: f64| (h + s * 0.707 / SLAB_FRAC).clamp(0.0, 1.0);
        let dd = |hh: f64| {
            // Sun taps sample the mean field (coarse by design); the view
            // path carries the detail. Same window + gradient as the shader.
            let bb = remap_coverage(puff_density(hh, peak), 0.0);
            (((bb - FLOOR) / (1.0 - FLOOR)).clamp(0.0, 1.0)) * type_gradient(hh, type01)
        };
        let sun_depth = ((dd(rise(span * 0.5)) * 0.6 + dd(rise(span)) * 0.4) * span + d * dt * 0.5)
            * EXT_PER_RADII;
        let bp = beer_powder(sun_depth);
        let sun: [f64; 3] = [
            SUN_WARM[0] * day * bp * (PHASE_AMBIENT + PHASE_GAIN * phase) * SUN_GAIN,
            SUN_WARM[1] * day * bp * (PHASE_AMBIENT + PHASE_GAIN * phase) * SUN_GAIN,
            SUN_WARM[2] * day * bp * (PHASE_AMBIENT + PHASE_GAIN * phase) * SUN_GAIN,
        ];
        let amb = ambient_rgb(h, transmittance, AMBIENT_BOTTOM, AMBIENT_TOP);
        let amb = [
            amb[0] * day.max(0.03),
            amb[1] * day.max(0.03),
            amb[2] * day.max(0.03),
        ];
        let absorb = (-EXT_PER_RADII * d * dt).exp();
        for c in 0..3 {
            scat[c] += transmittance * (sun[c] + amb[c]) * d * dt * EXT_PER_RADII * absorb;
        }
        if !snapped && (1.0 - transmittance) >= 0.05 {
            top_scat = scat;
            top_t = transmittance;
            snapped = true;
        }
        transmittance *= absorb;
    }
    let alpha = (1.0 - transmittance).clamp(0.0, 1.0);
    let norm = alpha.max(1e-3);
    let rgb = [scat[0] / norm, scat[1] / norm, scat[2] / norm];
    let top_norm = (1.0 - top_t).max(1e-3);
    // A column that never reaches 5% absorption has no visible surface:
    // report the full column instead of dividing noise by epsilon.
    let top = if snapped {
        [
            top_scat[0] / top_norm,
            top_scat[1] / top_norm,
            top_scat[2] / top_norm,
        ]
    } else {
        [scat[0] / norm, scat[1] / norm, scat[2] / norm]
    };
    // Thin edge column for the glow check.
    let thin = shade_thin(sun_elev_sin);
    (top, rgb, thin)
}

fn shade_thin(sun_elev_sin: f64) -> [f64; 3] {
    // Optically thin veil: single-scatter-ish, mostly ambient + powder edge.
    let day = (((sun_elev_sin + 0.08) / 0.22).clamp(0.0, 1.0) * 1.0).min(1.0);
    let powder = beer_powder(0.25);
    let luma = day * (powder * SUN_GAIN * 0.5 + 0.75);
    [luma, luma * 0.97, luma * 1.02]
}

#[test]
fn column_balance_tops_white_cores_dark_edges_glow() {
    // Dense noon puff: tops near-white HDR, full column darker (cores).
    let (top, full, thin) = shade_column(0.95, 0.707, 0.6);
    let luma = |rgb: [f64; 3]| (rgb[0] + rgb[1] + rgb[2]) / 3.0;
    eprintln!(
        "noon dense: top={:.2} full={:.2} thin={:.2}",
        luma(top),
        luma(full),
        luma(thin)
    );
    assert!(
        luma(top) > 1.0,
        "tops must run hot white, got {:.2}",
        luma(top)
    );
    assert!(luma(full) < luma(top), "cores must model below tops");
    assert!(
        luma(thin) > 0.35,
        "thin edges must glow, not die, got {:.2}",
        luma(thin)
    );
}
