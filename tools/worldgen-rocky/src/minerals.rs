//! Geology-driven mineral deposits.
//!
//! Deposits follow geology, never independent RGB noise:
//! - iron/mafic: basaltic provinces + impact breccia (craters expose depth)
//! - rare/felsic: old highlands + intrusive/vein zones
//! - volatiles: volcanic vents, evaporite playas, polar cold traps
//!
//! Placement is deterministic from (planet seed, geology, cell). Gameplay
//! resource *balancing* stays separate; this only decides causal placement.

use crate::{biomes::Geology, rng};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Deposit {
    IronMafic,
    RareFelsic,
    Volatile,
}

/// Richness 0..1 for each channel at a site.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MineralRichness {
    pub iron: f64,
    pub rare: f64,
    pub volatile: f64,
}

/// Deterministic deposit sampling. `elevation_m`, `slope01` and `dist_to_vent_m`
/// come from terrain; `arid01` from climate.
#[allow(clippy::too_many_arguments)]
pub fn sample_minerals(
    seed: u64,
    geology: Geology,
    elevation_m: f64,
    slope01: f64,
    dist_to_vent_m: f64,
    arid01: f64,
    polar01: f64,
    cell_ix: i64,
    cell_iy: i64,
) -> MineralRichness {
    let iron_prior = match geology {
        Geology::Basaltic => 0.7,
        Geology::ImpactBreccia => 0.6,
        Geology::OceanicCrust => 0.4,
        Geology::Regolith => 0.3,
        _ => 0.1,
    };
    let rare_prior = match geology {
        Geology::FelsicHighland => 0.7,
        Geology::ContinentalCrust => 0.4,
        Geology::ImpactBreccia => 0.35,
        _ => 0.08,
    };
    let volatile_prior = match geology {
        Geology::Evaporite => 0.8,
        Geology::Basaltic if dist_to_vent_m < 50_000.0 => 0.6,
        _ => 0.1,
    };
    // Sparse blobs: thresholded noise keeps deposits compact, not soup.
    let blob = |channel: u32, threshold: f64| -> f64 {
        let n = rng::fbm(seed, channel, cell_ix as f64 / 6.0, cell_iy as f64 / 6.0, 3) * 0.5 + 0.5;
        if n > threshold {
            (n - threshold) / (1.0 - threshold)
        } else {
            0.0
        }
    };
    let slope_gate = (1.0 - slope01 * 0.5).clamp(0.0, 1.0);
    let iron = (iron_prior * blob(301, 0.55) * slope_gate).clamp(0.0, 1.0);
    let rare =
        (rare_prior * blob(302, 0.60) * (0.4 + 0.6 * (elevation_m.max(0.0) / 6000.0).min(1.0)))
            .clamp(0.0, 1.0);
    let mut volatile = volatile_prior * blob(303, 0.55);
    // Cold traps: polar ice keeps volatiles regardless of geology.
    volatile = (volatile + polar01 * 0.5 * blob(304, 0.5)).clamp(0.0, 1.0);
    // Dry playas concentrate evaporites.
    if matches!(geology, Geology::Evaporite) {
        volatile = (volatile + arid01 * 0.3).clamp(0.0, 1.0);
    }
    MineralRichness {
        iron,
        rare,
        volatile,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn basalt_beats_granite_for_iron() {
        let common = (1000.0, 0.2, 100_000.0, 0.3, 0.0, 11, 7);
        let b = sample_minerals(
            9,
            Geology::Basaltic,
            common.0,
            common.1,
            common.2,
            common.3,
            common.4,
            common.5,
            common.6,
        );
        let f = sample_minerals(
            9,
            Geology::Sedimentary,
            common.0,
            common.1,
            common.2,
            common.3,
            common.4,
            common.5,
            common.6,
        );
        // Priors differ; with same noise field basalt must not lose.
        assert!(b.iron >= f.iron);
    }

    #[test]
    fn deterministic_and_bounded() {
        let a = sample_minerals(
            9,
            Geology::ImpactBreccia,
            500.0,
            0.3,
            20_000.0,
            0.5,
            0.0,
            3,
            4,
        );
        let b = sample_minerals(
            9,
            Geology::ImpactBreccia,
            500.0,
            0.3,
            20_000.0,
            0.5,
            0.0,
            3,
            4,
        );
        assert_eq!(a.iron, b.iron);
        for v in [a.iron, a.rare, a.volatile] {
            assert!((0.0..=1.0).contains(&v) && v.is_finite());
        }
    }

    #[test]
    fn cold_trap_adds_volatiles_on_any_geology() {
        let plain = sample_minerals(9, Geology::Regolith, 0.0, 0.1, 1e9, 0.0, 0.0, 3, 4);
        let polar = sample_minerals(9, Geology::Regolith, 0.0, 0.1, 1e9, 0.0, 1.0, 3, 4);
        assert!(polar.volatile >= plain.volatile);
    }
}
