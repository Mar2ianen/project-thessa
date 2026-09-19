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
pub fn radial_weight(r_m: f64, radius_m: f64) -> f64 {
    if !(radius_m > 0.0) || !r_m.is_finite() {
        return 0.0;
    }
    let u = (r_m.abs() / radius_m).min(3.0);
    (-2.2 * u * u).exp() + 0.18 * (-0.7 * u * u).exp()
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
    // Station emission already carries material hues, axial decay, and heat;
    // here only the radial weight and the shared shock factor apply, so
    // diamonds modulate density/temperature/emission together (doc §9).
    let w = radial_weight(r_m, station.radius_m);
    let shock = station.shock.max(0.15);
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
        for channel in 0..3 {
            total[channel] += 0.5 * (a.emission_rgb[channel] + b.emission_rgb[channel]) * area * dz;
        }
    }
    total
}
/// Beer-Lambert slab integration across a chord at axial `z_m`:
/// march `steps` samples over `[-half_chord_m, +half_chord_m]`, accumulate
/// emission attenuated by running transmittance. Returns
/// `(transmittance, rgb)`. This is the oracle the GPU must match within
/// representation error; `steps` is a budget, not physics (tested).
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
        rgb[0] += transmittance * sample.emission_rgb[0] * dt * absorb;
        rgb[1] += transmittance * sample.emission_rgb[1] * dt * absorb;
        rgb[2] += transmittance * sample.emission_rgb[2] * dt * absorb;
        transmittance *= absorb;
    }
    (transmittance, rgb)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::profile::build_axial_profile;
    use crate::source::tests::{sample_env_sea_level, sample_source};

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
