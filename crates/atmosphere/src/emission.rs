//! Upper-atmosphere emission: deterministic analytic aurora (spec §11).
//!
//! Emission is separate from scattering. The first implementation needs no
//! magnetohydrodynamics: oval geometry + altitude range + activity amplitude
//! + procedural curtains, all evaluable from absolute simulation time.

use glam::DVec3;

use crate::optics::AuroraParams;

/// Auroral emission RGB at a planet-centric point and simulation time.
///
/// Deterministic in `(position, sim_time_s)`: magnetic latitude selects the
/// oval band, altitude selects the green bottom / red-violet top mix,
/// activity and curtain phase come from analytic periodic terms of
/// `sim_time_s`. Returns zeros outside the shell or when disabled.
pub fn aurora_emission(params: &AuroraParams, position_m: DVec3, sim_time_s: f64) -> [f64; 3] {
    let r = position_m.length();
    if r <= 0.0 || !sim_time_s.is_finite() {
        return [0.0; 3];
    }
    let dir = position_m / r;
    let pole = params.magnetic_pole.normalize_or_zero();
    if pole == DVec3::ZERO {
        return [0.0; 3];
    }
    // Magnetic latitude from the dipole axis.
    let mag_lat = (dir.dot(pole)).clamp(-1.0, 1.0).asin().abs();
    let oval = (-((mag_lat - params.oval_latitude_rad).powi(2))
        / (2.0_f64 * params.oval_width_rad.max(1e-3).powi(2)))
    .exp();
    if oval < 1e-3 {
        return [0.0; 3];
    }
    // Altitude band with smooth edges, mapped to colour mix.
    let alt = r - params.altitude_min_m.min(params.altitude_max_m);
    let band = (params.altitude_max_m - params.altitude_min_m).max(1.0);
    let frac = (alt / band).clamp(0.0, 1.0);
    let alt_window = (frac * std::f64::consts::PI).sin().max(0.0);
    if alt_window <= 0.0 {
        return [0.0; 3];
    }
    // Activity: slow diurnal-ish term plus a faster substorm-ish term, both
    // pure functions of absolute simulation time (warp-safe).
    let activity = params.activity.clamp(0.0, 1.0)
        * (0.55_f64
            + 0.30 * (sim_time_s * 2.0 * std::f64::consts::PI / 31_680.0).sin()
            + 0.15 * (sim_time_s * 2.0 * std::f64::consts::PI / 1_370.0 + 1.7).sin())
        .max(0.0);
    // Curtains: a few static longitude octaves sliding slowly with time.
    let lon = dir.z.atan2(dir.x);
    let curtain = 0.6
        + 0.25 * (3.0 * lon + sim_time_s * 0.002).sin()
        + 0.15 * (7.0 * lon - sim_time_s * 0.005).sin();
    let intensity = oval * alt_window * activity * curtain.max(0.0);
    [
        params.color_bottom_rgb[0] * (1.0 - frac) + params.color_top_rgb[0] * frac,
        params.color_bottom_rgb[1] * (1.0 - frac) + params.color_top_rgb[1] * frac,
        params.color_bottom_rgb[2] * (1.0 - frac) + params.color_top_rgb[2] * frac,
    ]
    .map(|c| (c * intensity).max(0.0))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    fn oval_params() -> AuroraParams {
        AuroraParams {
            magnetic_pole: DVec3::Z,
            oval_latitude_rad: 67.0_f64.to_radians(),
            oval_width_rad: 3.0_f64.to_radians(),
            altitude_min_m: 90_000.0,
            altitude_max_m: 250_000.0,
            activity: 0.8,
            color_bottom_rgb: [0.2, 1.0, 0.35],
            color_top_rgb: [0.9, 0.25, 0.3],
        }
    }

    #[test]
    fn oval_emits_and_equator_does_not() {
        let params = oval_params();
        let r = 3_200_000.0 + 150_000.0;
        let lat = 67.0_f64.to_radians();
        let on_oval = DVec3::new(r * lat.cos(), 0.0, r * lat.sin());
        let glow = aurora_emission(&params, on_oval, 1_000_000.0);
        assert!(glow.iter().sum::<f64>() > 0.0, "oval must emit: {glow:?}");
        let equator = DVec3::new(r, 0.0, 0.0);
        assert_eq!(aurora_emission(&params, equator, 1_000_000.0), [0.0; 3]);
    }

    #[test]
    fn emission_is_deterministic_and_finite() {
        let params = oval_params();
        let p = DVec3::new(1.0, 0.5, 2.0).normalize() * 3_350_000.0;
        let a = aurora_emission(&params, p, 50_000_000.0);
        let b = aurora_emission(&params, p, 50_000_000.0);
        assert_eq!(a, b);
        assert!(a.iter().all(|c| c.is_finite() && *c >= 0.0));
        let _ = FRAC_PI_2;
    }
}
