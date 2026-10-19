//! Upper-atmosphere emission: deterministic analytic aurora (spec §11).
//!
//! Emission is separate from scattering. The first implementation needs no
//! magnetohydrodynamics: oval geometry + altitude range + activity amplitude
//! + procedural curtains, all evaluable from absolute simulation time.

use glam::DVec3;

use crate::optics::AuroraParams;

/// Absolute-time activity and longitude phases, shared with GPU adapters.
pub fn aurora_phase(params: &AuroraParams, time_s: f64) -> [f64; 3] {
    let tau = std::f64::consts::TAU;
    let activity = params.activity.clamp(0.0, 1.0)
        * (0.55
            + 0.30 * (time_s * tau / 31_680.0).sin()
            + 0.15 * (time_s * tau / 1_370.0 + 1.7).sin())
        .max(0.0);
    [
        activity,
        (time_s * 0.002).rem_euclid(tau),
        (-time_s * 0.005).rem_euclid(tau),
    ]
}

/// Integrate the shared volumetric emission over both visible portions of its
/// spherical altitude band. Colour inputs are emission per metre per steradian;
/// output is radiance in the same units as the atmosphere. This is an analytic
/// presentation model, not an MHD/particle simulation. Atmospheric attenuation
/// reuses the shared finite-segment transport, and the solid planet occludes.
pub fn aurora_radiance(
    optics: &crate::AtmosphereOptics,
    camera_m: DVec3,
    view_dir: DVec3,
    sim_time_s: f64,
    steps: u32,
) -> [f64; 3] {
    aurora_radiance_to(optics, camera_m, view_dir, 1e30, sim_time_s, steps)
}

/// Finite camera segment, clipped by an actual opaque hit (SI metres).
pub fn aurora_radiance_to(
    optics: &crate::AtmosphereOptics,
    camera_m: DVec3,
    view_dir: DVec3,
    distance_m: f64,
    sim_time_s: f64,
    steps: u32,
) -> [f64; 3] {
    let Some(params) = &optics.emission.aurora else {
        return [0.0; 3];
    };
    let dir = view_dir.normalize_or_zero();
    if !camera_m.is_finite()
        || !sim_time_s.is_finite()
        || dir == DVec3::ZERO
        || !distance_m.is_finite()
        || distance_m <= 0.0
        || !params.altitude_min_m.is_finite()
        || params.altitude_min_m <= 0.0
        || !params.altitude_max_m.is_finite()
        || params.altitude_max_m <= params.altitude_min_m
    {
        return [0.0; 3];
    }
    let outer = optics.inner_radius_m + params.altitude_max_m;
    let inner = optics.inner_radius_m + params.altitude_min_m;
    let Some((a, b)) = crate::clouds::sphere_interval(camera_m, dir, DVec3::ZERO, outer) else {
        return [0.0; 3];
    };
    let mut intervals = [(a.max(0.0), b), (0.0, 0.0)];
    if let Some((c, d)) = crate::clouds::sphere_interval(camera_m, dir, DVec3::ZERO, inner) {
        intervals = [(a.max(0.0), b.min(c)), (a.max(d).max(0.0), b)];
    }
    let dark = crate::CelestialLight {
        direction_to_star: DVec3::Y,
        irradiance_w_m2: 0.0,
        color_rgb: [0.0; 3],
        angular_radius_rad: 0.0,
        visibility: 0.0,
    };
    let mut radiance = [0.0; 3];
    for (a, b) in intervals {
        let Some((a, b)) = crate::clip_to_planet(
            a,
            b.min(distance_m),
            camera_m,
            dir,
            DVec3::ZERO,
            optics.inner_radius_m,
        ) else {
            continue;
        };
        let count = steps.clamp(1, 256);
        let dt = (b - a) / f64::from(count);
        // Integrate the foreground once, then advance the same optical depth
        // along the shell. Re-integrating camera->sample at every point was
        // quadratic in the quality budget and caused long High publications.
        let (mut transmission, _) =
            crate::aerial_perspective(optics, camera_m, camera_m + dir * a, &dark, count);
        for i in 0..count {
            let point = camera_m + dir * (a + (f64::from(i) + 0.5) * dt);
            let emission = aurora_emission(params, optics.inner_radius_m, point, sim_time_s);
            let sigma = if point.length() <= optics.outer_radius_m {
                crate::scattering::extinction_rgb(optics, point)
            } else {
                [0.0; 3]
            };
            for channel in 0..3 {
                let absorb = (-sigma[channel] * dt).exp();
                radiance[channel] += emission[channel] * transmission[channel] * absorb.sqrt() * dt;
                transmission[channel] *= absorb;
            }
        }
    }
    radiance
}

/// Auroral emission RGB at a planet-centric point and simulation time.
///
/// Deterministic in `(position, sim_time_s)`: magnetic latitude selects the
/// oval band, altitude selects the green bottom / red-violet top mix,
/// activity and curtain phase come from analytic periodic terms of
/// `sim_time_s`. Returns zeros outside the shell or when disabled.
pub fn aurora_emission(
    params: &AuroraParams,
    body_radius_m: f64,
    position_m: DVec3,
    sim_time_s: f64,
) -> [f64; 3] {
    let r = position_m.length();
    if !r.is_finite()
        || r <= 0.0
        || !sim_time_s.is_finite()
        || !body_radius_m.is_finite()
        || body_radius_m <= 0.0
        || !params.altitude_min_m.is_finite()
        || !params.altitude_max_m.is_finite()
        || params.altitude_max_m <= params.altitude_min_m
    {
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
    let altitude_m = r - body_radius_m;
    // Exact zeros at both shell boundaries: sin(PI) is positive roundoff,
    // not evidence that a point above the shell emits.
    if altitude_m <= params.altitude_min_m || altitude_m >= params.altitude_max_m {
        return [0.0; 3];
    }
    let band = params.altitude_max_m - params.altitude_min_m;
    let frac = (altitude_m - params.altitude_min_m) / band;
    let alt_window = (frac * std::f64::consts::PI).sin().max(0.0);
    if alt_window <= 0.0 {
        return [0.0; 3];
    }
    // Activity: slow diurnal-ish term plus a faster substorm-ish term, both
    // pure functions of absolute simulation time (warp-safe).
    let [activity, phase3, phase7] = aurora_phase(params, sim_time_s);
    // Curtains: a few static longitude octaves sliding slowly with time.
    // Longitude about the magnetic axis, not an unrelated world-Y axis.
    let helper = if pole.dot(DVec3::Y).abs() < 0.9 {
        DVec3::Y
    } else {
        DVec3::Z
    };
    let right = helper.cross(pole).normalize();
    let forward = pole.cross(right);
    let horizontal = glam::DVec2::new(dir.dot(right), dir.dot(forward));
    let lon = if horizontal.length_squared() <= 1e-12 {
        0.0
    } else {
        horizontal.y.atan2(horizontal.x)
    };
    let curtain = 0.6 + 0.25 * (3.0 * lon + phase3).sin() + 0.15 * (7.0 * lon + phase7).sin();
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
        let glow = aurora_emission(&params, 3_200_000.0, on_oval, 1_000_000.0);
        assert!(
            glow.iter().sum::<f64>() > 0.01,
            "oval must really emit: {glow:?}"
        );
        let equator = DVec3::new(r, 0.0, 0.0);
        assert_eq!(
            aurora_emission(&params, 3_200_000.0, equator, 1_000_000.0),
            [0.0; 3]
        );
    }

    #[test]
    fn emission_is_deterministic_and_finite() {
        let params = oval_params();
        let p = DVec3::new(1.0, 0.5, 2.0).normalize() * 3_350_000.0;
        let a = aurora_emission(&params, 3_200_000.0, p, 50_000_000.0);
        let b = aurora_emission(&params, 3_200_000.0, p, 50_000_000.0);
        assert_eq!(a, b);
        assert!(a.iter().all(|c| c.is_finite() && *c >= 0.0));
    }

    #[test]
    fn altitude_limits_reject_outside_shell_exactly_at_multiple_body_radii() {
        let mut params = oval_params();
        params.oval_latitude_rad = std::f64::consts::FRAC_PI_2;
        // Axis-aligned shell points make the exact mathematical boundaries
        // representable, independent of trig/vector normalization roundoff.
        let direction = DVec3::Z;
        for radius in [1_000_000.0, 3_200_000.0, 6_371_000.0] {
            for altitude in [-1.0, 0.0, 90_000.0, 250_000.0, 250_001.0, 1_000_000.0] {
                assert_eq!(
                    aurora_emission(&params, radius, direction * (radius + altitude), 0.0),
                    [0.0; 3]
                );
            }
            assert!(
                aurora_emission(&params, radius, direction * (radius + 170_000.0), 0.0)
                    .iter()
                    .sum::<f64>()
                    > 0.01
            );
        }
        for radius in [f64::NAN, f64::INFINITY, 0.0, -1.0] {
            assert_eq!(
                aurora_emission(&params, radius, direction * 3_350_000.0, 0.0),
                [0.0; 3]
            );
        }
        assert_eq!(
            aurora_emission(&params, 3_200_000.0, DVec3::NAN, 0.0),
            [0.0; 3]
        );
    }

    #[test]
    fn integrated_emission_converges_and_solid_planet_occludes() {
        let radius = 3_200_000.0;
        let mut optics = crate::nitrogen_oxygen_optics(radius, 0.0, 8_000.0).unwrap();
        // Exact vacuum special case, independently integrate the sine window.
        optics.absorption.clear();
        let mut params = oval_params();
        params.oval_latitude_rad = std::f64::consts::FRAC_PI_2;
        params.color_bottom_rgb = [1e-6; 3];
        params.color_top_rgb = [1e-6; 3];
        optics.emission.aurora = Some(params.clone());
        let camera = DVec3::Z * (radius + 1.0);
        let reference = crate::clouds::integrate_slab_unclamped_simpson(
            |alt| aurora_emission(&params, radius, DVec3::Z * (radius + alt), 0.0)[0],
            params.altitude_min_m,
            params.altitude_max_m,
            4096,
            1.0,
        )
        .1;
        let measured = aurora_radiance(&optics, camera, DVec3::Z, 0.0, 64)[0];
        assert!(
            (measured - reference).abs() < reference * 0.0002,
            "{measured} vs {reference}"
        );
        assert_eq!(
            aurora_radiance(&optics, camera, -DVec3::Z, 0.0, 64),
            [0.0; 3]
        );
        assert_eq!(
            aurora_radiance(&optics, camera, DVec3::ZERO, 0.0, 64),
            [0.0; 3]
        );
        optics.emission.aurora = None;
        assert_eq!(
            aurora_radiance(&optics, camera, DVec3::Z, 0.0, 64),
            [0.0; 3]
        );
    }

    #[test]
    fn incremental_emission_transport_matches_independent_absorbing_column() {
        let radius = 3_200_000.0;
        let mut optics = crate::nitrogen_oxygen_optics(radius, 0.0, 8000.0).unwrap();
        optics.absorption.clear();
        optics.outer_radius_m = radius + 1_000_000.0;
        optics.rayleigh.beta_rgb = [3e-6; 3];
        optics.rayleigh.scale_height_m = 1e15;
        let mut params = oval_params();
        params.oval_latitude_rad = std::f64::consts::FRAC_PI_2;
        params.color_bottom_rgb = [1e-6; 3];
        params.color_top_rgb = [1e-6; 3];
        optics.emission.aurora = Some(params.clone());
        let reference = crate::clouds::integrate_slab_unclamped_simpson(
            |alt| {
                aurora_emission(&params, radius, DVec3::Z * (radius + alt), 0.0)[0]
                    * (-3e-6 * (alt - 1.0)).exp()
            },
            params.altitude_min_m,
            params.altitude_max_m,
            4096,
            1.0,
        )
        .1;
        let measured = aurora_radiance(&optics, DVec3::Z * (radius + 1.0), DVec3::Z, 0.0, 64)[0];
        assert!(
            (measured - reference).abs() < reference * 0.0002,
            "{measured} vs {reference}"
        );
        // Outside the optical shell, transport must remain exactly vacuum,
        // even if the mathematical exponential still has a nonzero tail.
        optics.outer_radius_m = radius + 1.0;
        let vacuum = aurora_radiance(&optics, DVec3::Z * (radius + 2.0), DVec3::Z, 0.0, 64);
        optics.rayleigh.beta_rgb = [0.0; 3];
        assert_eq!(
            vacuum,
            aurora_radiance(&optics, DVec3::Z * (radius + 2.0), DVec3::Z, 0.0, 64)
        );
    }
}
