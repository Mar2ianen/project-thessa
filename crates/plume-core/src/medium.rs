//! Participating-medium sampling over the analytic mean field
//! (`docs/38` sections 6, 8).
//!
//! This is the CPU reference oracle: raster and future ray backends must
//! agree with [`integrate_ray`] within their declared representation error
//! (doc section 17). The GPU mirrors this math; it must never fork it.
//!
//! Radial model (documented reduced form): a hot Gaussian core plus a cooler
//! mixing-layer skirt. Shock modulation comes from the axial profile, so
//! diamonds emerge from the volume rather than from a decal.

use crate::profile::AxialProfile;

/// One medium sample: extinction (1/m) and linear HDR emission RGB.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MediumSample {
    pub extinction_per_m: f64,
    pub emission_rgb: [f64; 3],
}

/// Radial weight 0..~1.2 at axial `z_m`, radial `r_m` against the local mean
/// radius `radius_m`: Gaussian core plus a mixing-layer skirt.
// Validity guards must reject NaN: `!(x > 0.0)` catches NaN while the lint's
// suggested `x <= 0.0` would accept it. The negated form is deliberate.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn radial_weight(r_m: f64, radius_m: f64) -> f64 {
    if !(radius_m > 0.0) || !r_m.is_finite() {
        return 0.0;
    }
    let u = (r_m.abs() / radius_m).min(3.0);
    (-2.2 * u * u).exp() + 0.18 * (-0.7 * u * u).exp()
}

/// First axisymmetric pressure mode of a circular jet, J0(j01*r/R).
/// The core boundary is a pressure node; the mixing skirt is unmodulated.
/// This is a weak-shock representation, not a nonlinear Mach-disk solution.
pub fn shock_multiplier(axial_shock: f64, radius_fraction: f64) -> f64 {
    if !axial_shock.is_finite() || !radius_fraction.is_finite() {
        return 1.0;
    }
    let u = radius_fraction.abs();
    if u >= 1.0 || axial_shock == 1.0 {
        return 1.0;
    }
    // J0(x) = sum((-x*x/4)^k / (k!)^2), through k=6.
    // At x<=j01 the next term bounds absolute truncation below 5.3e-7.
    let q = 1.445_796_490_736_696_2 * u * u;
    let mode = 1.0
        + q * (-1.0
            + q * (0.25
                + q * (-1.0 / 36.0 + q * (1.0 / 576.0 + q * (-1.0 / 14_400.0 + q / 518_400.0)))));
    (1.0 + (axial_shock.max(0.15) - 1.0) * mode.clamp(0.0, 1.0)).max(0.15)
}

/// Medium at axial `z_m` / radial `r_m`: axial interpolation from the
/// profile, then the radial weight. Station emission already carries
/// material hues (shared ramp); only weight and the shared shock factor
/// apply here. Empty profile (engine off) is vacuum with zero emission.
pub fn sample_medium(profile: &AxialProfile, z_m: f64, r_m: f64) -> MediumSample {
    let Some(station) = profile.evaluate(z_m) else {
        return MediumSample {
            extinction_per_m: 0.0,
            emission_rgb: [0.0; 3],
        };
    };
    // Station emission carries hues, decay and heat. The circular pressure
    // mode modulates core emission/extinction together, with a quiet skirt.
    let w = radial_weight(r_m, station.radius_m);
    let shock = shock_multiplier(station.shock, r_m / station.radius_m);
    MediumSample {
        extinction_per_m: (station.extinction_per_m * w * shock).max(0.0),
        emission_rgb: [
            (station.emission_rgb[0] * w * shock).max(0.0),
            (station.emission_rgb[1] * w * shock).max(0.0),
            (station.emission_rgb[2] * w * shock).max(0.0),
        ],
    }
}

/// Total radiant power proxy (linear RGB): emission integrated over the
/// plume volume, `sum(emission * pi * R^2 * dz)`. Lighting proxies derive
/// from this field integral (doc section 13), never from bare throttle.
pub fn radiant_power(profile: &AxialProfile) -> [f64; 3] {
    let mut total = [0.0; 3];
    for window in profile.stations.windows(2) {
        let (a, b) = (window[0], window[1]);
        let dz = (b.z_m - a.z_m).max(0.0);
        let area = std::f64::consts::PI * (0.5 * (a.radius_m + b.radius_m)).powi(2);
        for (channel, slot) in total.iter_mut().enumerate() {
            *slot += 0.5 * (a.emission_rgb[channel] + b.emission_rgb[channel]) * area * dz;
        }
    }
    total
}
/// Beer-Lambert slab integration across a chord at axial `z_m`:
/// march `steps` samples over `[-half_chord_m, +half_chord_m]`, accumulate
/// emission attenuated by running transmittance. Returns
/// `(transmittance, rgb)`. This is the oracle the GPU must match within
/// representation error; `steps` is a budget, not physics (tested).
// Validity guards must reject NaN: `!(x > 0.0)` catches NaN while the lint's
// suggested `x <= 0.0` would accept it. The negated form is deliberate.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn integrate_ray(
    profile: &AxialProfile,
    z_m: f64,
    half_chord_m: f64,
    steps: u32,
) -> (f64, [f64; 3]) {
    let steps = steps.clamp(1, 256) as usize;
    if !(half_chord_m > 0.0) || profile.is_empty() {
        return (1.0, [0.0; 3]);
    }
    let dt = 2.0 * half_chord_m / steps as f64;
    let mut transmittance = 1.0;
    let mut rgb = [0.0; 3];
    for i in 0..steps {
        let r = -half_chord_m + (i as f64 + 0.5) * dt;
        let sample = sample_medium(profile, z_m, r);
        let absorb = (-sample.extinction_per_m * dt).exp();
        // Integrate emission throughout the cell, not at its far boundary.
        let emission_length = if sample.extinction_per_m > 0.0 {
            -(-sample.extinction_per_m * dt).exp_m1() / sample.extinction_per_m
        } else {
            dt
        };
        rgb[0] += transmittance * sample.emission_rgb[0] * emission_length;
        rgb[1] += transmittance * sample.emission_rgb[1] * emission_length;
        rgb[2] += transmittance * sample.emission_rgb[2] * emission_length;
        transmittance *= absorb;
    }
    (transmittance, rgb)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::profile::build_axial_profile;
    use crate::source::tests::{sample_env_sea_level, sample_source};

    #[test]
    fn radial_shock_matches_independent_bessel_series_and_core_boundary() {
        let mut max_error = 0.0_f64;
        for i in 0..=10_000 {
            let u = f64::from(i) / 10_000.0;
            let q = (2.404_825_557_695_773 * u).powi(2) / 4.0;
            let mut term = 1.0;
            let mut exact = 1.0;
            for k in 1..32 {
                term *= -q / f64::from(k * k);
                exact += term;
            }
            let actual = shock_multiplier(2.0, u) - 1.0;
            max_error = max_error.max((actual - exact).abs());
            assert!(
                (actual - exact).abs() < 5.3e-7,
                "u={u} actual={actual} exact={exact}"
            );
        }
        assert!(
            max_error > 1e-7,
            "oracle must exercise truncation, not mirror the polynomial"
        );
        for axial in [0.15, 0.45, 0.75, 1.0, 1.55, 4.0] {
            assert!((shock_multiplier(axial, 0.0) - axial).abs() < 1e-15);
            for u in [1.0, 1.25, 3.0] {
                assert_eq!(shock_multiplier(axial, u), 1.0);
            }
            for u in [0.0, 0.2, 0.5, 0.9] {
                assert_eq!(shock_multiplier(axial, u), shock_multiplier(axial, -u));
                let value = shock_multiplier(axial, u);
                assert!(value >= axial.min(1.0) - 1e-15 && value <= axial.max(1.0) + 1e-15);
                assert_eq!(shock_multiplier(1.0, u), 1.0);
            }
        }
        assert_eq!(shock_multiplier(f64::NAN, 0.5), 1.0);
        assert_eq!(shock_multiplier(1.5, f64::INFINITY), 1.0);
    }

    fn live_profile() -> AxialProfile {
        build_axial_profile(&sample_source(), &sample_env_sea_level(), 48).unwrap()
    }

    #[test]
    fn core_is_brighter_than_edge() {
        let profile = live_profile();
        let z = profile.length_m * 0.15;
        let core = sample_medium(&profile, z, 0.0);
        let edge = sample_medium(&profile, z, profile.evaluate(z).unwrap().radius_m * 1.5);
        let core_sum: f64 = core.emission_rgb.iter().sum();
        let edge_sum: f64 = edge.emission_rgb.iter().sum();
        assert!(core_sum > edge_sum * 3.0, "{core_sum} vs {edge_sum}");
    }

    #[test]
    fn transmittance_falls_with_chord_and_extinction() {
        let profile = live_profile();
        let z = profile.length_m * 0.1;
        let (t_narrow, _) = integrate_ray(&profile, z, 0.2, 16);
        let (t_wide, _) = integrate_ray(&profile, z, 3.0, 16);
        assert!(t_narrow > t_wide);
        assert!((0.0..=1.0).contains(&t_wide));
    }

    #[test]
    fn constant_source_function_conserves_emitted_energy_at_any_step_budget() {
        // E(r) = sigma(r) * S gives L = S*(1-T), independently of the
        // radial quadrature. Attenuating each whole cell's emission at its
        // far boundary violates this identity, especially in thick cells.
        for extinction in [1e-8, 0.1, 1.0, 100.0] {
            let mut profile = live_profile();
            for s in &mut profile.stations {
                s.extinction_per_m = extinction;
                s.emission_rgb = [4.0, 2.0, 1.0].map(|v| v * extinction);
            }
            for steps in [1, 2, 8, 14, 64, 256] {
                let (transmittance, rgb) = integrate_ray(&profile, 2.0, 1.6, steps);
                for (actual, source_function) in rgb.into_iter().zip([4.0, 2.0, 1.0]) {
                    let expected = source_function * (1.0 - transmittance);
                    assert!(
                        (actual - expected).abs() < 2e-12,
                        "sigma={extinction} steps={steps} actual={actual} expected={expected}"
                    );
                }
            }
        }
    }

    #[test]
    fn dead_engine_integrates_to_vacuum() {
        let mut source = sample_source();
        source.throttle = 0.0;
        let profile = build_axial_profile(&source, &sample_env_sea_level(), 32).unwrap();
        let (t, rgb) = integrate_ray(&profile, 2.0, 2.0, 16);
        assert_eq!(t, 1.0);
        assert_eq!(rgb, [0.0; 3]);
    }

    #[test]
    fn step_budget_changes_error_not_physics() {
        let profile = live_profile();
        let z = profile.length_m * 0.2;
        let (_, fine) = integrate_ray(&profile, z, 2.0, 64);
        let (_, coarse) = integrate_ray(&profile, z, 2.0, 4);
        for channel in 0..3 {
            let denom = fine[channel].abs().max(0.02);
            let rel = (fine[channel] - coarse[channel]).abs() / denom;
            assert!(rel < 0.35, "channel {channel} rel {rel}");
        }
    }

    #[test]
    fn radiant_power_tracks_throttle_and_vanishes_when_off() {
        let full = radiant_power(&live_profile());
        assert!(full.iter().all(|c| *c > 0.0));
        let mut source = sample_source();
        source.throttle = 0.4;
        let part =
            radiant_power(&build_axial_profile(&source, &sample_env_sea_level(), 48).unwrap());
        for channel in 0..3 {
            assert!(part[channel] < full[channel]);
        }
        source.throttle = 0.0;
        let off =
            radiant_power(&build_axial_profile(&source, &sample_env_sea_level(), 48).unwrap());
        assert_eq!(off, [0.0; 3]);
    }

    #[test]
    fn sampling_is_deterministic_and_finite() {
        let profile = live_profile();
        let a = sample_medium(&profile, 3.0, 0.4);
        let b = sample_medium(&profile, 3.0, 0.4);
        assert_eq!(a, b);
        assert!(a.extinction_per_m.is_finite());
    }
}

#[cfg(test)]
mod probe_tests {
    use super::*;
    use crate::source::tests::{sample_env_sea_level, sample_source};

    #[test]
    fn low_throttle_field_stays_faint() {
        // Contract the volume renderer relies on: at part throttle the
        // mean field is long but optically thin, so the march must fade it
        // out instead of rendering an opaque balloon (screenshot-proven).
        let mut source = sample_source();
        source.throttle = 0.3;
        let env = sample_env_sea_level();
        let profile = crate::profile::build_axial_profile(&source, &env, 32).unwrap();
        assert!(
            profile.length_m > 5.0,
            "long but thin, len={}",
            profile.length_m
        );
        let r0 = source.exit_radius_m;
        for (z, half) in [
            (0.5, profile.length_m / 2.0),
            (profile.length_m * 0.3, r0 * 2.0),
            (profile.length_m * 0.8, r0 * 3.0),
        ] {
            let (t, _) = integrate_ray(&profile, z, half, 64);
            assert!(
                1.0 - t < 0.15,
                "thin field at 30% throttle, z={z} alpha={}",
                1.0 - t
            );
        }
    }
}
