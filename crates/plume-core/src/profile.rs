//! Analytic mean plume: axial profile from source + environment
//! (`docs/38` section 6).
//!
//! The mean field stays analytic and cheap. A backend precomputes a small
//! axial profile when nozzle state changes materially; the GPU interpolates
//! plus an analytic radial falloff. Residual 3D structure (turbulence,
//! crosswind, condensation) belongs in the later RCBT residual, not here.
//!
//! Reduced-model notes (all provisional, direction-tested, never absolute
//! truth claims):
//!
//! - Expansion regime follows the pressure ratio `Pi = p_exit / p_ambient`
//!   with a 5% deadband so small numerical noise never flaps the regime.
//! - Shock-cell spacing follows the Tam vortex-sheet approximation
//!   `L = pi * D * sqrt(M^2 - 1) / 2.4048`. Mismatch changes shock
//!   AMPLITUDE, not first-order spacing.
//! - Plume length scales with `D * M * sqrt(Pi)` (momentum vs ambient).
//!   The leading constant is provisional and flagged for calibration
//!   against reference firings; tests pin direction, not metres.
//! - Throttle scales mass flow and exhaust velocity linearly from the
//!   full-throttle deck values. Real engine decks are nonlinear; engine-sim
//!   owns the true curve when it lands.

use crate::source::{PlumeEnvironment, PlumeSource, pressure_ratio, validation_error};
use crate::optics::optical_material;

/// Expansion regime from the pressure ratio (5% deadband).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExpansionRegime {
    /// Pi < 0.95: overexpanded, oblique shocks at the lip, weak cells.
    Overexpanded,
    /// 0.95..=1.05: near-matched, weakest shock structure.
    Matched,
    /// Pi > 1.05: underexpanded, strong Mach disks/diamonds, wide fan.
    Underexpanded,
}

/// Regime from source + environment.
pub fn expansion_regime(source: &PlumeSource, env: &PlumeEnvironment) -> ExpansionRegime {
    let pi = pressure_ratio(source, env);
    if pi < 0.95 {
        ExpansionRegime::Overexpanded
    } else if pi <= 1.05 {
        ExpansionRegime::Matched
    } else {
        ExpansionRegime::Underexpanded
    }
}

/// First shock-cell spacing (m) from exit diameter, exit Mach, and ambient
/// mismatch direction. Spacing grows with `D` and `M` (Tam); the pressure
/// ratio feeds amplitude separately via [`shock_amplitude`].
pub fn shock_cell_spacing_m(exit_diameter_m: f64, exit_mach: f64, pi: f64) -> f64 {
    if !(exit_diameter_m > 0.0) || !(exit_mach > 1.0) || !pi.is_finite() || pi <= 0.0 {
        return 0.0;
    }
    // Tam vortex-sheet cell, weakly stretched when strongly underexpanded
    // (documented provisional 8% per doubling, capped): direction-tested.
    let tam = std::f64::consts::PI * exit_diameter_m * (exit_mach * exit_mach - 1.0).sqrt()
        / 2.4048;
    let stretch = (1.0 + 0.08 * (pi.max(1.0).log2())).min(1.4);
    tam * stretch
}

/// Shock modulation amplitude 0..1 from mismatch distance. Matched flow has
/// almost no cells; deep over/underexpansion saturates near 1.
pub fn shock_amplitude(pi: f64) -> f64 {
    if !pi.is_finite() || pi <= 0.0 {
        return 0.0;
    }
    let distance = (pi.ln() / 0.05).abs().min(4.0);
    (distance / 4.0).clamp(0.0, 1.0)
}

/// One axial station of the mean profile (SI, `f64`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxialStation {
    /// Axial distance downstream of the exit plane (m, >= 0).
    pub z_m: f64,
    /// Mean plume radius (m, > 0).
    pub radius_m: f64,
    /// Centerline density (kg/m^3, >= 0).
    pub center_density_kg_m3: f64,
    /// Centerline static temperature (K, >= 0).
    pub center_temp_k: f64,
    /// Extinction coefficient (1/m, >= 0).
    pub extinction_per_m: f64,
    /// Emission RGB, linear, HDR-capable (>= 0 each).
    pub emission_rgb: [f64; 3],
    /// Shock compression multiplier, ~0.6..1.6 (1 = no modulation).
    pub shock: f64,
}

/// Mean axial profile: `2..=64` stations, endpoints exact.
#[derive(Debug, Clone, PartialEq)]
pub struct AxialProfile {
    pub stations: Vec<AxialStation>,
    pub regime: ExpansionRegime,
    /// Visible length (m): last station position.
    pub length_m: f64,
}

impl AxialProfile {
    /// Empty (engine off): zero stations, zero length.
    pub fn empty(regime: ExpansionRegime) -> Self {
        Self {
            stations: Vec::new(),
            regime,
            length_m: 0.0,
        }
    }

    pub fn is_empty(&self) -> bool {
        self.stations.is_empty()
    }

    /// Linear interpolation at axial `z_m` (clamped to the profile span).
    /// Returns `None` for an empty profile.
    pub fn evaluate(&self, z_m: f64) -> Option<AxialStation> {
        let n = self.stations.len();
        if n == 0 {
            return None;
        }
        if n == 1 || z_m <= 0.0 {
            return Some(self.stations[0]);
        }
        let last = self.stations[n - 1];
        if z_m >= last.z_m {
            return Some(last);
        }
        let mut lo = 0usize;
        while lo + 1 < n && self.stations[lo + 1].z_m < z_m {
            lo += 1;
        }
        let a = self.stations[lo];
        let b = self.stations[lo + 1];
        let span = (b.z_m - a.z_m).max(1e-9);
        let t = ((z_m - a.z_m) / span).clamp(0.0, 1.0);
        let lerp = |x: f64, y: f64| x + (y - x) * t;
        Some(AxialStation {
            z_m,
            radius_m: lerp(a.radius_m, b.radius_m),
            center_density_kg_m3: lerp(a.center_density_kg_m3, b.center_density_kg_m3),
            center_temp_k: lerp(a.center_temp_k, b.center_temp_k),
            extinction_per_m: lerp(a.extinction_per_m, b.extinction_per_m),
            emission_rgb: [
                lerp(a.emission_rgb[0], b.emission_rgb[0]),
                lerp(a.emission_rgb[1], b.emission_rgb[1]),
                lerp(a.emission_rgb[2], b.emission_rgb[2]),
            ],
            shock: lerp(a.shock, b.shock),
        })
    }
}

/// Emission scale shared by the CPU builder and GPU uniform fill (single
/// source of truth): material luminosity times throttle-scaled jet power in
/// megawatts, clamped. The GPU must use this, never its own gain formula.
pub fn emission_gain(source: &PlumeSource) -> f64 {
    let material = optical_material(source.exhaust);
    let power = source.mass_flow_kg_s * source.throttle
        * source.exhaust_velocity_mps
        * source.throttle;
    material.luminosity * (power / 1.0e6).clamp(0.0, 40.0)
}
///
/// Returns an empty profile (not an error) when `throttle <= 0`: a shut-down
/// engine contributes zero visible/lighting output.
pub fn build_axial_profile(
    source: &PlumeSource,
    env: &PlumeEnvironment,
    max_samples: u32,
) -> Result<AxialProfile, &'static str> {
    if let Some(reason) = validation_error(source, env) {
        return Err(reason);
    }
    let regime = expansion_regime(source, env);
    if source.throttle <= 0.0 {
        return Ok(AxialProfile::empty(regime));
    }
    let samples = max_samples.clamp(2, 64) as usize;

    let pi = pressure_ratio(source, env);
    let diameter = 2.0 * source.exit_radius_m;
    // Effective deck at throttle (linear provisional, see module notes).
    let mass_flow = source.mass_flow_kg_s * source.throttle;
    let exit_temp = source.exit_temperature_k;
    // Visible length: momentum vs ambient. Constant 2.2 is provisional.
    let length = diameter * source.exit_mach * pi.max(0.05).sqrt() * 2.2;
    let length = length.clamp(diameter * 2.0, 4000.0);

    let spread = 0.12 + 0.10 * (1.0 - 1.0 / pi.max(1.0));
    let cell = shock_cell_spacing_m(diameter, source.exit_mach, pi);
    let amp = shock_amplitude(pi);
    let material = optical_material(source.exhaust);
    // Emission scale: mass flow x throttle x thermal content x material.
    let lum = emission_gain(source);

    let mut stations = Vec::with_capacity(samples);
    for i in 0..samples {
        let z = length * i as f64 / (samples - 1) as f64;
        let zn = z / length;
        let radius = source.exit_radius_m * (1.0 + spread * z / diameter.max(1e-6));
        let decay = 1.0 / (1.0 + 6.0 * zn * zn);
        let density = (mass_flow
            / (source.exhaust_velocity_mps * std::f64::consts::PI * radius * radius).max(1e-9))
            .max(0.0);
        let temp = env.temperature_k + (exit_temp - env.temperature_k).max(0.0) * decay;
        let shock = if cell > 0.0 {
            1.0 + amp * 0.55 * (2.0 * std::f64::consts::PI * z / cell).cos() * (-z / (3.0 * cell.max(length * 0.05))).exp()
        } else {
            1.0
        };
        let heat = (temp / exit_temp.max(1.0)).clamp(0.0, 1.0);
        // Station emission carries hue (shared ramp) and decay, but NOT the
        // shock factor: shock applies once downstream (sampler / shader),
        // never squared by baking it here too.
        let e = lum * (0.15 + 0.85 * heat) * decay.max(0.02);
        let hue = crate::optics::ramp_rgb(material, zn);
        stations.push(AxialStation {
            z_m: z,
            radius_m: radius,
            center_density_kg_m3: density,
            center_temp_k: temp,
            extinction_per_m: (density * (0.5 + material.soot)).max(0.0),
            emission_rgb: [
                (hue[0] * e).max(0.0),
                (hue[1] * e).max(0.0),
                (hue[2] * e).max(0.0),
            ],
            shock,
        });
    }
    // Anchor the nozzle station exactly (radius/temperature boundary).
    if let Some(first) = stations.first_mut() {
        first.z_m = 0.0;
        first.radius_m = source.exit_radius_m;
        first.center_temp_k = exit_temp;
    }
    Ok(AxialProfile {
        stations,
        regime,
        length_m: length,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::source::tests::{sample_env_sea_level, sample_source};

    #[test]
    fn regime_moves_with_pressure_ratio() {
        let source = sample_source();
        let mut env = sample_env_sea_level();
        // Exit 68 kPa: sea level (101 kPa) is overexpanded.
        assert_eq!(expansion_regime(&source, &env), ExpansionRegime::Overexpanded);
        // Matched band around exit pressure.
        env.pressure_pa = 68_000.0;
        assert_eq!(expansion_regime(&source, &env), ExpansionRegime::Matched);
        // High altitude: underexpanded.
        env.pressure_pa = 5_000.0;
        assert_eq!(
            expansion_regime(&source, &env),
            ExpansionRegime::Underexpanded
        );
        // Vacuum: underexpanded.
        env.pressure_pa = 0.0;
        assert_eq!(
            expansion_regime(&source, &env),
            ExpansionRegime::Underexpanded
        );
    }

    #[test]
    fn shock_spacing_grows_with_mach_and_diameter() {
        let base = shock_cell_spacing_m(1.3, 3.4, 1.0);
        assert!(base > 0.0);
        assert!(shock_cell_spacing_m(1.3, 4.5, 1.0) > base);
        assert!(shock_cell_spacing_m(2.6, 3.4, 1.0) > base);
        assert_eq!(shock_cell_spacing_m(0.0, 3.4, 1.0), 0.0);
        assert_eq!(shock_cell_spacing_m(1.3, 0.9, 1.0), 0.0);
    }

    #[test]
    fn shock_amplitude_peaks_away_from_matched() {
        let matched = shock_amplitude(1.0);
        assert!(shock_amplitude(0.3) > matched);
        assert!(shock_amplitude(4.0) > matched);
        assert!((0.0..=1.0).contains(&shock_amplitude(20.0)));
    }

    #[test]
    fn zero_throttle_is_empty() {
        let mut source = sample_source();
        source.throttle = 0.0;
        let profile = build_axial_profile(&source, &sample_env_sea_level(), 48).unwrap();
        assert!(profile.is_empty());
        assert_eq!(profile.length_m, 0.0);
        assert!(profile.evaluate(10.0).is_none());
    }

    #[test]
    fn profile_is_deterministic_and_bounded() {
        let a = build_axial_profile(&sample_source(), &sample_env_sea_level(), 48).unwrap();
        let b = build_axial_profile(&sample_source(), &sample_env_sea_level(), 48).unwrap();
        assert_eq!(a, b);
        assert_eq!(a.stations.len(), 48);
        assert_eq!(a.stations[0].z_m, 0.0);
        for station in &a.stations {
            assert!(station.radius_m > 0.0);
            assert!(station.center_density_kg_m3 >= 0.0);
            assert!(station.center_temp_k >= 0.0);
            assert!(station.extinction_per_m >= 0.0);
            assert!(station.emission_rgb.iter().all(|c| *c >= 0.0 && c.is_finite()));
            assert!(station.shock.is_finite());
        }
    }

    #[test]
    fn budgets_agree_on_shared_stations() {
        // Quality changes representation cost, not physics (doc section 15).
        let fine = build_axial_profile(&sample_source(), &sample_env_sea_level(), 64).unwrap();
        let coarse = build_axial_profile(&sample_source(), &sample_env_sea_level(), 16).unwrap();
        assert_eq!(fine.length_m, coarse.length_m);
        assert_eq!(fine.regime, coarse.regime);
        for z in [0.0, 2.0, 5.0, 12.0, 30.0] {
            let a = fine.evaluate(z).unwrap();
            let b = coarse.evaluate(z).unwrap();
            for channel in 0..3 {
                let denom = a.emission_rgb[channel].abs().max(0.05);
                let rel = (a.emission_rgb[channel] - b.emission_rgb[channel]).abs() / denom;
                assert!(rel < 0.25, "z={z} channel={channel} rel={rel}");
            }
        }
    }

    #[test]
    fn vacuum_lengthens_and_sea_level_shortens() {
        let source = sample_source();
        let sea = build_axial_profile(&source, &sample_env_sea_level(), 32).unwrap();
        let mut vacuum_env = sample_env_sea_level();
        vacuum_env.pressure_pa = 0.0;
        let vacuum = build_axial_profile(&source, &vacuum_env, 32).unwrap();
        assert!(vacuum.length_m > sea.length_m);
    }

    #[test]
    fn invalid_source_is_an_error_not_a_profile() {
        let mut source = sample_source();
        source.exit_radius_m = 0.0;
        assert!(build_axial_profile(&source, &sample_env_sea_level(), 32).is_err());
    }
}
