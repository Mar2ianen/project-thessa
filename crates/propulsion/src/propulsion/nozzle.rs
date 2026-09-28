//! Ideal frozen-flow isentropic nozzle kernel (Sutton) plus nozzle contour
//! families. No tables: the Mach solver iterates deterministically.

use serde::{Deserialize, Serialize};

use super::{NOZZLE_EFFICIENCY, PropellantThermo, PropulsionError};

/// Characteristic velocity c* = sqrt(R*Tc) / Gamma (m/s).
pub fn characteristic_velocity(thermo: &PropellantThermo) -> f64 {
    let gamma = thermo.gamma;
    let gamma_fn = gamma.sqrt() * (2.0 / (gamma + 1.0)).powf((gamma + 1.0) / (2.0 * (gamma - 1.0)));
    (thermo.gas_constant_j_kg_k * thermo.chamber_temp_k).sqrt() / gamma_fn
}

/// Supersonic area ratio A/A* for a Mach number (isentropic).
fn area_ratio_from_mach(gamma: f64, mach: f64) -> f64 {
    let term = (2.0 / (gamma + 1.0)) * (1.0 + (gamma - 1.0) / 2.0 * mach * mach);
    term.powf((gamma + 1.0) / (2.0 * (gamma - 1.0))) / mach
}

/// Supersonic exit Mach for an expansion ratio. Newton iteration on the log
/// residual with a bisection fallback; deterministic, no tables.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn mach_from_area_ratio(gamma: f64, expansion_ratio: f64) -> Result<f64, PropulsionError> {
    if !(gamma > 1.0) || !(expansion_ratio >= 1.0) {
        return Err(PropulsionError::InvalidSpec(
            "gamma must be > 1 and expansion ratio >= 1".into(),
        ));
    }
    if expansion_ratio == 1.0 {
        return Ok(1.0);
    }
    let target = expansion_ratio.ln();
    let mut mach = (2.0 + expansion_ratio.ln()).max(1.5);
    for _ in 0..64 {
        let residual = area_ratio_from_mach(gamma, mach).ln() - target;
        if residual.abs() < 1e-13 {
            return Ok(mach);
        }
        let step = 1e-6 * mach.max(1.0);
        let slope = (area_ratio_from_mach(gamma, mach + step).ln()
            - area_ratio_from_mach(gamma, mach - step).ln())
            / (2.0 * step);
        if !slope.is_finite() || slope == 0.0 {
            break;
        }
        let next = mach - residual / slope;
        if !next.is_finite() || next <= 1.0 {
            break;
        }
        mach = next;
    }
    // Bisection fallback on a bracket that always contains the root for
    // finite expansion ratios.
    let mut low = 1.0_f64;
    let mut high = 2.0_f64;
    while area_ratio_from_mach(gamma, high).ln() < target {
        high *= 2.0;
        if high > 1.0e6 {
            return Err(PropulsionError::InvalidSpec(
                "expansion ratio out of range".into(),
            ));
        }
    }
    for _ in 0..200 {
        let mid = 0.5 * (low + high);
        if area_ratio_from_mach(gamma, mid).ln() < target {
            low = mid;
        } else {
            high = mid;
        }
    }
    Ok(0.5 * (low + high))
}

/// Exit-plane state for an expansion ratio at chamber conditions.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct NozzleExitState {
    pub exit_mach: f64,
    /// Exit-to-chamber pressure ratio.
    pub exit_pressure_ratio: f64,
    /// Exit static temperature (K).
    pub exit_temp_k: f64,
    /// Exhaust velocity at full expansion (m/s).
    pub exhaust_velocity_mps: f64,
}

pub(crate) fn nozzle_exit(
    thermo: &PropellantThermo,
    expansion_ratio: f64,
) -> Result<NozzleExitState, PropulsionError> {
    let exit_mach = mach_from_area_ratio(thermo.gamma, expansion_ratio)?;
    let exit_pressure_ratio = (1.0 + (thermo.gamma - 1.0) / 2.0 * exit_mach * exit_mach)
        .powf(-thermo.gamma / (thermo.gamma - 1.0));
    let exit_temp_k =
        thermo.chamber_temp_k / (1.0 + (thermo.gamma - 1.0) / 2.0 * exit_mach * exit_mach);
    let exhaust_velocity_mps =
        exit_mach * (thermo.gamma * thermo.gas_constant_j_kg_k * exit_temp_k).sqrt();
    Ok(NozzleExitState {
        exit_mach,
        exit_pressure_ratio,
        exit_temp_k,
        exhaust_velocity_mps,
    })
}

/// Ideal thrust coefficient for chamber pressure `chamber_pa`, exit state,
/// expansion ratio, ambient pressure, and geometric divergence factor.
pub fn thrust_coefficient(
    thermo: &PropellantThermo,
    chamber_pa: f64,
    exit: &NozzleExitState,
    expansion_ratio: f64,
    ambient_pa: f64,
    divergence_factor: f64,
) -> f64 {
    let gamma = thermo.gamma;
    let momentum = divergence_factor
        * NOZZLE_EFFICIENCY
        * ((2.0 * gamma * gamma / (gamma - 1.0))
            * (2.0 / (gamma + 1.0)).powf((gamma + 1.0) / (gamma - 1.0))
            * (1.0 - exit.exit_pressure_ratio.powf((gamma - 1.0) / gamma)))
        .sqrt();
    let exit_pa = exit.exit_pressure_ratio * chamber_pa;
    momentum + (exit_pa - ambient_pa) / chamber_pa * expansion_ratio
}

/// Nozzle contour family. Bell/cone derive the divergence factor from wall
/// geometry; the bell recovers half the residual divergence loss
/// (documented engineering approximation, thrust envelope +/-1% pinned
/// by test). The aerospike is altitude-compensating: the free jet boundary
/// adapts to ambient, so the pressure term never goes overexpanded-wide;
/// ambient eats only the documented base area (base drag).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum NozzleContour {
    Conical,
    Bell,
    Aerospike,
}

/// Aerospike base area as a fraction of equivalent exit area (base drag,
/// documented).
pub(crate) const AEROSPIKE_BASE_FRACTION: f64 = 0.05;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sonic_throat_and_mach_solver_round_trip() {
        // Known special cases: eps = 1 chokes exactly; the solver inverts
        // the area-Mach relation to 1e-9 on the Juno bell range.
        let mach = mach_from_area_ratio(1.24, 1.0).expect("sonic throat");
        assert!((mach - 1.0).abs() < 1e-12);
        for (gamma, eps) in [(1.2, 3.0), (1.22, 16.0), (1.24, 35.0), (1.25, 80.0)] {
            let mach = mach_from_area_ratio(gamma, eps).expect("mach solve");
            assert!(mach > 1.0);
            let round_trip = area_ratio_from_mach(gamma, mach);
            assert!(
                (round_trip - eps).abs() / eps < 1e-9,
                "gamma {gamma} eps {eps}: got {round_trip}"
            );
        }
    }
}
