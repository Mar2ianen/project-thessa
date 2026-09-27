//! High-speed regime effects (Slice F): sonic-boom carpet, transonic buffet,
//! condensation visuals.
//!
//! All items are pure functions of scalars specifically so they stay testable
//! and SIMD/portable without solver surgery:
//! - [`boom_carpet`] turns Mach/altitude/weight/length plus the atmosphere
//!   profile into a ground footprint (Tier A calibrated scaling, not CFD);
//! - [`buffet_gain`] is the quasi-steady transonic buffet gate consumed by
//!   the panel solver behind an explicit opt-in response;
//! - [`buffet_fluctuation`] is the deterministic unsteady part for consumers
//!   that own a tick (replay-safe: splitmix hash, no stored RNG state);
//! - [`vapor_cone_active`] gates the force-neutral condensation visual.
//!
//! NaN policy follows the codebase rule: every boundary rejects NaN
//! (`!(x >= 0)` form, never `x < 0`), so invalid air data fails closed.

use crate::aero::smoothstep;
use std::{error::Error, fmt};

/// High-speed effect input or calibration failure.
#[derive(Debug, Clone, PartialEq)]
pub enum HighSpeedError {
    InvalidInput(String),
}

impl fmt::Display for HighSpeedError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidInput(message) => write!(formatter, "invalid high-speed input: {message}"),
        }
    }
}

impl Error for HighSpeedError {}

/// Sonic-boom ground footprint for one supersonic cruise point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoomCarpet {
    /// False when the boom never reaches the ground: subsonic flight, or
    /// supersonic below the refraction cutoff. All fields below are zero
    /// in that case (no silent partial footprint).
    pub reaches_ground: bool,
    /// Carpet half-width: `altitude * sqrt(M^2 - 1)` (Mach-cone geometry).
    pub half_width_m: f64,
    /// Peak N-wave overpressure from the Tier A scaling law, Pa.
    pub peak_overpressure_pa: f64,
    /// Cutoff Mach from the temperature profile: rays turn before the
    /// ground below this. Isothermal profiles give exactly 1.0 (no
    /// cutoff), which is the correct physics, not a missing model.
    pub cutoff_mach: f64,
}

/// Tier A overpressure calibration: Concorde-class anchor (185 t, 62 m,
/// M = 2.0 at 18 km) must read ~2 psf. Fixed constant, documented here —
/// not tuned per call site.
pub const BOOM_OVERPRESSURE_GAIN: f64 = 1.37;

/// Reference anchor the calibration is pinned to (Concorde-class cruise).
pub const BOOM_ANCHOR_PSF: f64 = 2.0;

/// Compute the boom carpet for a steady supersonic point.
///
/// `weight_n` / `length_m` are the whole-vehicle values (the boom is an
/// aircraft-level effect, never per-panel). `ambient` is sampled at
/// altitude, `ground_pressure_pa` / `ground_sound_mps` at the surface
/// below the track. Subsonic or below-cutoff flight returns a zeroed
/// non-reaching carpet instead of an error — absence of boom is a normal
/// answer, while NaN/negative inputs are rejected.
#[allow(clippy::too_many_arguments)]
pub fn boom_carpet(
    mach: f64,
    altitude_m: f64,
    weight_n: f64,
    length_m: f64,
    ambient_pressure_pa: f64,
    ambient_sound_mps: f64,
    ground_pressure_pa: f64,
    ground_sound_mps: f64,
) -> Result<BoomCarpet, HighSpeedError> {
    let finite_positive = [mach, altitude_m, weight_n, length_m];
    if finite_positive.iter().any(|v| !v.is_finite() || *v <= 0.0) {
        return Err(HighSpeedError::InvalidInput(
            "mach/altitude/weight/length must be finite and positive".into(),
        ));
    }
    // NaN must fail validation: `!(x >= 0)` rejects NaN while `x < 0`
    // would accept it. The negated form is deliberate (see plume-core).
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    for (value, name) in [
        (ambient_pressure_pa, "ambient pressure"),
        (ambient_sound_mps, "ambient sound speed"),
        (ground_pressure_pa, "ground pressure"),
        (ground_sound_mps, "ground sound speed"),
    ] {
        if !(value >= 0.0) {
            return Err(HighSpeedError::InvalidInput(format!(
                "{name} must be finite and non-negative"
            )));
        }
    }
    if ambient_sound_mps == 0.0 || ground_sound_mps == 0.0 {
        return Err(HighSpeedError::InvalidInput(
            "sound speeds must be positive".into(),
        ));
    }
    // Refraction cutoff: rays turn when M * a(h) < a(ground).
    let cutoff_mach = ground_sound_mps / ambient_sound_mps;
    let quiet = BoomCarpet {
        reaches_ground: false,
        half_width_m: 0.0,
        peak_overpressure_pa: 0.0,
        cutoff_mach,
    };
    if mach <= 1.0 || mach < cutoff_mach {
        return Ok(quiet);
    }
    // Carlson-style simplified scaling: sqrt(p_amb * p_ground) sets the
    // acoustic scale, (M^2 - 1)^(-3/8) the Mach decay, and the
    // weight/(pressure * length * altitude) group is dimensionless.
    let mach_factor = (mach * mach - 1.0).powf(-3.0 / 8.0);
    let loading = weight_n / (ground_pressure_pa * length_m * altitude_m);
    if !(mach_factor.is_finite() && loading.is_finite()) {
        return Err(HighSpeedError::InvalidInput(
            "boom scaling overflowed its inputs".into(),
        ));
    }
    let peak = BOOM_OVERPRESSURE_GAIN
        * (ambient_pressure_pa * ground_pressure_pa).sqrt()
        * mach_factor
        * loading.sqrt();
    if !peak.is_finite() {
        return Err(HighSpeedError::InvalidInput(
            "boom overpressure is non-finite".into(),
        ));
    }
    Ok(BoomCarpet {
        reaches_ground: true,
        half_width_m: altitude_m * (mach * mach - 1.0).sqrt(),
        peak_overpressure_pa: peak,
        cutoff_mach,
    })
}

/// Transonic condensation-cloud gate (Prandtl–Glauert visualization).
/// Force-neutral by contract: callers must never feed this into the force
/// solver. `humidity01` is the aloft value; until the atmosphere model
/// carries a humidity profile, callers pass 0.0 and the gate stays shut
/// (visible code path, no fake moisture data).
pub fn vapor_cone_active(mach: f64, humidity01: f64) -> bool {
    mach.is_finite() && (0.95..=1.05).contains(&mach) && humidity01 >= 0.6
}

/// Quasi-steady transonic buffet gate, 0..=1.
///
/// Shock-band Mach factor times high-alpha proximity to the stall angle
/// (both smoothsteps: mul/add only, SIMD-safe). Exactly 0 outside the
/// band, so panels with no buffet response keep bitwise-identical
/// coefficients. The unsteady part lives in [`buffet_fluctuation`].
pub fn buffet_gain(normal_mach: f64, alpha_rad: f64, stall_angle_rad: f64) -> f64 {
    // Same NaN-closed validity style as above: garbage in, exactly 0.0 out.
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    if !(normal_mach.is_finite() && alpha_rad.is_finite()) || !(stall_angle_rad > 0.0) {
        return 0.0;
    }
    smoothstep(0.85, 1.05, normal_mach) * smoothstep(0.6, 0.9, alpha_rad.abs() / stall_angle_rad)
}

/// Deterministic unsteady buffet fluctuation in [-1, 1].
///
/// Stateless splitmix64 hash of `(seed, tick, lane)`: identical across
/// runs, worker counts, and ISAs (wrapping `u64` arithmetic only), so
/// replays stay bit-identical. Consumers scale it by their own envelope,
/// e.g. `fluct * gain * response * 0.05 * max_lift`. Never NaN.
pub fn buffet_fluctuation(seed: u64, tick: u64, lane: u64) -> f64 {
    let mut state = seed
        .wrapping_add(tick.wrapping_mul(0x9E3779B97F4A7C15))
        .wrapping_add(lane.wrapping_mul(0xBF58476D1CE4E5B9));
    state = (state ^ (state >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
    state = (state ^ (state >> 27)).wrapping_mul(0x94D049BB133111EB);
    state ^= state >> 31;
    (state >> 11) as f64 / (u64::MAX >> 11) as f64 * 2.0 - 1.0
}

#[cfg(test)]
mod tests {
    use super::*;

    const CONCORDE_W_N: f64 = 1.81e6;
    const CONCORDE_L_M: f64 = 61.7;

    fn cruise() -> (f64, f64, f64, f64, f64, f64, f64, f64) {
        // (mach, alt, weight, length, p_amb, a_amb, p_gnd, a_gnd): Concorde
        // class at 18 km over a 15 C day.
        (
            2.0,
            18_000.0,
            CONCORDE_W_N,
            CONCORDE_L_M,
            7_500.0,
            295.0,
            100_000.0,
            340.0,
        )
    }

    #[test]
    fn boom_anchor_monotonicity_and_cutoff() {
        let (mach, alt, weight, length, p_amb, a_amb, p_gnd, a_gnd) = cruise();
        let carpet = boom_carpet(mach, alt, weight, length, p_amb, a_amb, p_gnd, a_gnd).unwrap();
        assert!(carpet.reaches_ground);
        // Concorde-class anchor: ~2 psf (96 Pa) within a wide Tier A band.
        assert!(
            (carpet.peak_overpressure_pa - 96.0).abs() < 40.0,
            "anchor drifted: {} Pa",
            carpet.peak_overpressure_pa
        );
        // Carpet geometry is exact Mach-cone math.
        let expected_half = alt * (mach * mach - 1.0).sqrt();
        assert!((carpet.half_width_m - expected_half).abs() < 1e-9);
        // Monotonicity: heavier hits harder, higher/faster hits softer.
        let heavy = boom_carpet(mach, alt, 2.0 * weight, length, p_amb, a_amb, p_gnd, a_gnd)
            .unwrap()
            .peak_overpressure_pa;
        assert!((heavy / carpet.peak_overpressure_pa - 2.0_f64.sqrt()).abs() < 1e-9);
        let high = boom_carpet(mach, 2.0 * alt, weight, length, p_amb, a_amb, p_gnd, a_gnd)
            .unwrap()
            .peak_overpressure_pa;
        assert!(high < carpet.peak_overpressure_pa);
        let fast = boom_carpet(2.5, alt, weight, length, p_amb, a_amb, p_gnd, a_gnd)
            .unwrap()
            .peak_overpressure_pa;
        assert!(fast < carpet.peak_overpressure_pa);
        // Subsonic: quiet carpet, not an error.
        let quiet = boom_carpet(0.9, alt, weight, length, p_amb, a_amb, p_gnd, a_gnd).unwrap();
        assert!(!quiet.reaches_ground);
        assert_eq!(quiet.peak_overpressure_pa, 0.0);
        assert_eq!(quiet.half_width_m, 0.0);
        // Supersonic below cutoff: refraction turns the rays around.
        let cut = boom_carpet(1.05, alt, weight, length, p_amb, a_amb, p_gnd, a_gnd).unwrap();
        assert!(!cut.reaches_ground, "M=1.05 must sit below the cutoff");
        assert!(cut.cutoff_mach > 1.0);
        // Isothermal profile: cutoff exactly 1.0 (no cutoff physics).
        let iso = boom_carpet(1.5, alt, weight, length, p_amb, 300.0, p_gnd, 300.0).unwrap();
        assert!(iso.reaches_ground);
        assert_eq!(iso.cutoff_mach, 1.0);
    }

    #[test]
    fn boom_rejects_non_physical_inputs() {
        let (mach, alt, weight, length, p_amb, a_amb, p_gnd, a_gnd) = cruise();
        // NaN fails closed on every lane (never a silent zero footprint).
        for bad in [
            (f64::NAN, alt, weight, length, p_amb, a_amb, p_gnd, a_gnd),
            (mach, f64::NAN, weight, length, p_amb, a_amb, p_gnd, a_gnd),
            (mach, alt, weight, length, f64::NAN, a_amb, p_gnd, a_gnd),
            (mach, alt, weight, length, p_amb, a_amb, f64::NAN, a_gnd),
        ] {
            let (m, h, w, l, pa, aa, pg, ag) = bad;
            assert!(boom_carpet(m, h, w, l, pa, aa, pg, ag).is_err());
        }
        assert!(boom_carpet(mach, 0.0, weight, length, p_amb, a_amb, p_gnd, a_gnd).is_err());
        assert!(boom_carpet(mach, alt, -1.0, length, p_amb, a_amb, p_gnd, a_gnd).is_err());
        assert!(boom_carpet(mach, alt, weight, length, p_amb, a_amb, p_gnd, 0.0).is_err());
    }

    #[test]
    fn vapor_gate_needs_band_and_moisture() {
        assert!(vapor_cone_active(1.0, 0.8));
        assert!(vapor_cone_active(0.95, 1.0));
        assert!(vapor_cone_active(1.05, 0.6));
        assert!(!vapor_cone_active(0.9, 1.0));
        assert!(!vapor_cone_active(1.1, 1.0));
        assert!(!vapor_cone_active(1.0, 0.5));
        // Humidity stub: dry air never shows vapor, by contract.
        assert!(!vapor_cone_active(1.0, 0.0));
        assert!(!vapor_cone_active(f64::NAN, 1.0));
    }

    #[test]
    fn buffet_gain_is_zero_outside_and_bounded_inside() {
        let stall = 18.0_f64.to_radians();
        // Cruise and low alpha: exactly 0 (legacy bitwise parity).
        assert_eq!(buffet_gain(0.5, 0.1, stall), 0.0);
        assert_eq!(buffet_gain(0.9, 0.05, stall), 0.0);
        assert_eq!(buffet_gain(1.5, 0.01, stall), 0.0);
        // Garbage in, zero out (never NaN into the solver).
        assert_eq!(buffet_gain(f64::NAN, 0.2, stall), 0.0);
        assert_eq!(buffet_gain(0.95, f64::NAN, stall), 0.0);
        assert_eq!(buffet_gain(0.95, 0.2, 0.0), 0.0);
        assert_eq!(buffet_gain(0.95, 0.2, -0.1), 0.0);
        // In the shock band near stall: strictly positive, bounded by 1.
        let gain = buffet_gain(0.95, 0.9 * stall, stall);
        assert!(gain > 0.0 && gain <= 1.0, "gain={gain}");
        let deep = buffet_gain(1.1, stall, stall);
        assert!(
            deep > gain,
            "deeper into the band must gain, {deep} vs {gain}"
        );
    }

    #[test]
    fn buffet_fluctuation_is_deterministic_bounded_and_live() {
        // Same inputs, same output — across calls, lanes differ.
        let a = buffet_fluctuation(7, 1000, 3);
        assert_eq!(a, buffet_fluctuation(7, 1000, 3));
        assert_ne!(a, buffet_fluctuation(7, 1000, 4));
        assert_ne!(a, buffet_fluctuation(7, 1001, 3));
        assert_ne!(a, buffet_fluctuation(8, 1000, 3));
        // Bounded in [-1, 1], never NaN, over a wide sweep.
        let mut min = 1.0_f64;
        let mut max = -1.0_f64;
        for tick in (0..500).step_by(7) {
            for lane in 0..8 {
                let value = buffet_fluctuation(12345, tick, lane);
                assert!(value.is_finite());
                min = min.min(value);
                max = max.max(value);
            }
        }
        // A live fluctuation source must actually use both halves.
        assert!(min < -0.5, "min={min}");
        assert!(max > 0.5, "max={max}");
    }
}
