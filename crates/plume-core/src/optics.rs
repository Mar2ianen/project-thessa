//! Exhaust optical materials (`docs/38` section 11).
//!
//! These are RENDER HUES, not physics: compact RGB approximations that let
//! hydrolox, kerolox, methalox, solids, nuclear-thermal, cold gas, and ion
//! beams share one plume architecture without sharing one look. Values are
//! linear-space, HDR-capable (core entries exceed 1 for bloom), loosely
//! calibrated against reference test-firing photography. They must never
//! feed back into thrust, heating, or dynamics.

use crate::source::ExhaustFamily;

/// Optical description of one exhaust family.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OpticalMaterial {
    /// Hot core emission RGB (linear, may exceed 1 for bloom).
    pub core_rgb: [f64; 3],
    /// Mid-plume emission RGB (linear).
    pub mid_rgb: [f64; 3],
    /// Cool edge/smoke RGB (linear).
    pub edge_rgb: [f64; 3],
    /// Overall luminosity gain applied by the profile builder.
    pub luminosity: f64,
    /// Soot fraction 0..=1 (raises extinction, warms the edge).
    pub soot: f64,
    /// Smoke/condensate fraction 0..=1 (downstream sheath strength).
    pub smoke: f64,
}

/// Axial hue ramp shared by the CPU builder and the GPU volume pass
/// (single source of truth for hues; absolute scale comes from the profile):
/// hot core hues in the first metres blending fast to mid, then cool edge
/// hues downstream. `axial_01` is 0 at the nozzle, 1 at the visible tail.
/// The steep core falloff is deliberate: references show a white-blue core
/// confined to the first diameters, not a gradual wash. Constants are
/// mirrored in `assets/shaders/plume_volume.wgsl`.
pub fn ramp_rgb(material: OpticalMaterial, axial_01: f64) -> [f64; 3] {
    let zn = axial_01.clamp(0.0, 1.0);
    let core_bias = (-6.0 * zn).exp();
    let edge_bias = 1.0 - (-2.0 * zn).exp();
    let edge_mix = (edge_bias * 0.45).min(0.6);
    let no_edge = 1.0 - edge_mix;
    [
        (material.core_rgb[0] * core_bias + material.mid_rgb[0] * (1.0 - core_bias)) * no_edge
            + material.edge_rgb[0] * edge_mix,
        (material.core_rgb[1] * core_bias + material.mid_rgb[1] * (1.0 - core_bias)) * no_edge
            + material.edge_rgb[1] * edge_mix,
        (material.core_rgb[2] * core_bias + material.mid_rgb[2] * (1.0 - core_bias)) * no_edge
            + material.edge_rgb[2] * edge_mix,
    ]
}
/// Shared hue divisor: max channel over core/mid/edge (guaranteed > 0).
/// The axial builder and the GPU both divide hues by this ONE divisor, so
/// the ramp stays linear and CPU/GPU agree exactly while every channel
/// stays in 0..=1 (absolute brightness lives in the profile emission
/// scale, never in the hues). Prevents hue x luminosity double-counting.
pub fn hue_divisor(material: OpticalMaterial) -> f64 {
    let peak = material
        .core_rgb
        .into_iter()
        .chain(material.mid_rgb)
        .chain(material.edge_rgb)
        .fold(0.0_f64, f64::max);
    peak.max(0.05)
}
/// Render hues for an exhaust family. Every family is covered; every value
/// is finite and non-negative (pinned by test).
pub fn optical_material(family: ExhaustFamily) -> OpticalMaterial {
    match family {
        ExhaustFamily::Hydrolox => OpticalMaterial {
            // Near-transparent blue Mach diamonds, faint orange afterburn.
            core_rgb: [0.65, 0.85, 2.6],
            mid_rgb: [0.45, 0.62, 1.5],
            edge_rgb: [0.35, 0.30, 0.28],
            luminosity: 0.8,
            soot: 0.0,
            smoke: 0.25,
        },
        ExhaustFamily::Kerolox => OpticalMaterial {
            // Sooty orange fireball core, heavy smoke.
            core_rgb: [3.2, 1.9, 0.9],
            mid_rgb: [2.2, 0.95, 0.30],
            edge_rgb: [0.55, 0.30, 0.16],
            luminosity: 1.3,
            soot: 0.9,
            smoke: 0.9,
        },
        ExhaustFamily::Methalox => OpticalMaterial {
            // Blue-white diamond core (Raptor-like), pink-orange mid-body.
            core_rgb: [1.4, 1.5, 2.8],
            mid_rgb: [2.4, 1.05, 0.55],
            edge_rgb: [0.60, 0.30, 0.18],
            luminosity: 1.2,
            soot: 0.25,
            smoke: 0.55,
        },
        ExhaustFamily::Hypergolic => OpticalMaterial {
            // Bright orange-red, moderately smoky.
            core_rgb: [3.0, 1.5, 0.5],
            mid_rgb: [2.0, 0.70, 0.22],
            edge_rgb: [0.50, 0.24, 0.14],
            luminosity: 1.1,
            soot: 0.4,
            smoke: 0.6,
        },
        ExhaustFamily::Solid => OpticalMaterial {
            // Blinding white-orange core, dense smoke trail.
            core_rgb: [3.6, 2.8, 1.8],
            mid_rgb: [2.6, 1.5, 0.7],
            edge_rgb: [0.70, 0.55, 0.42],
            luminosity: 1.5,
            soot: 0.5,
            smoke: 1.0,
        },
        ExhaustFamily::NuclearThermal => OpticalMaterial {
            // Hydrogen heated without combustion: faint blue-white shimmer.
            core_rgb: [0.5, 0.7, 1.8],
            mid_rgb: [0.30, 0.42, 1.0],
            edge_rgb: [0.16, 0.18, 0.24],
            luminosity: 0.5,
            soot: 0.0,
            smoke: 0.1,
        },
        ExhaustFamily::ColdGas => OpticalMaterial {
            // No combustion: faint white condensation puff only.
            core_rgb: [0.25, 0.27, 0.30],
            mid_rgb: [0.18, 0.19, 0.21],
            edge_rgb: [0.10, 0.11, 0.12],
            luminosity: 0.15,
            soot: 0.0,
            smoke: 0.35,
        },
        ExhaustFamily::Ion => OpticalMaterial {
            // Violet-blue beam, near-zero smoke, long and thin (profile
            // length still comes from the same builder; missions tune it).
            core_rgb: [0.7, 0.5, 3.0],
            mid_rgb: [0.45, 0.35, 1.8],
            edge_rgb: [0.20, 0.16, 0.60],
            luminosity: 0.9,
            soot: 0.0,
            smoke: 0.0,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn all_families() -> [ExhaustFamily; 8] {
        use ExhaustFamily::*;
        [
            Hydrolox,
            Kerolox,
            Methalox,
            Hypergolic,
            Solid,
            NuclearThermal,
            ColdGas,
            Ion,
        ]
    }

    #[test]
    fn every_family_has_finite_non_negative_optics() {
        for family in all_families() {
            let m = optical_material(family);
            for rgb in [m.core_rgb, m.mid_rgb, m.edge_rgb] {
                assert!(
                    rgb.iter().all(|c| c.is_finite() && *c >= 0.0),
                    "{family} has bad RGB: {rgb:?}"
                );
            }
            assert!(m.luminosity.is_finite() && m.luminosity >= 0.0);
            assert!((0.0..=1.0).contains(&m.soot), "{family} soot");
            assert!((0.0..=1.0).contains(&m.smoke), "{family} smoke");
        }
    }

    #[test]
    fn families_are_visually_distinct() {
        // Cores must differ: sharing one look defeats the material table.
        let cores: Vec<[f64; 3]> = all_families()
            .iter()
            .map(|f| optical_material(*f).core_rgb)
            .collect();
        for i in 0..cores.len() {
            for j in (i + 1)..cores.len() {
                let dist_sq: f64 = (0..3)
                    .map(|c| (cores[i][c] - cores[j][c]).powi(2))
                    .sum();
                assert!(dist_sq > 0.05, "families {i} and {j} share a core look");
            }
        }
    }

    #[test]
    fn methalox_core_is_blue_and_mid_is_warm() {
        let m = optical_material(ExhaustFamily::Methalox);
        assert!(m.core_rgb[2] > m.core_rgb[0]);
        assert!(m.mid_rgb[0] > m.mid_rgb[2]);
    }

    #[test]
    fn shared_divisor_bounds_tinted_ramp() {
        // Hues divided by the shared divisor stay in 0..=1: absolute
        // brightness lives only in the profile emission scale.
        for family in all_families() {
            let m = optical_material(family);
            let divisor = hue_divisor(m);
            assert!(divisor > 0.0);
            for t in [0.0, 0.05, 0.25, 0.6, 1.0] {
                for channel in ramp_rgb(m, t) {
                    assert!(channel / divisor <= 1.0 + 1e-9);
                    assert!(channel / divisor >= 0.0);
                }
            }
        }
    }
}
