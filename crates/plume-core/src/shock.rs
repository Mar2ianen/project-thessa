//! Analytic, axisymmetric compression cells, independent of the axial table.
//!
//! Cell length uses the existing circular-jet approximation. Compression is
//! bounded by the perfect-gas normal-shock relation (NASA/NACA 1135). The
//! double-cone geometry and mixing decay are a reduced optical closure, not
//! a characteristic solver or a prediction of Mach-disk location.

use crate::{PlumeEnvironment, PlumeSource, profile::shock_cell_spacing_m, source::pressure_ratio};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShockCells {
    pub spacing_m: f64,
    pub radius_m: f64,
    /// Density jump across the equivalent pressure-restoring shock.
    pub compression: f64,
    /// Overexpanded jets start contracting; underexpanded jets expand first.
    pub phase: f64,
}

impl ShockCells {
    pub const NONE: Self = Self {
        spacing_m: 0.0,
        radius_m: 1.0,
        compression: 1.0,
        phase: 0.0,
    };

    pub fn from_source(source: &PlumeSource, env: &PlumeEnvironment) -> Self {
        let pi = pressure_ratio(source, env);
        let spacing_m = shock_cell_spacing_m(2.0 * source.exit_radius_m, source.exit_mach, pi);
        if spacing_m <= 0.0 || source.throttle <= 0.0 {
            return Self::NONE;
        }
        let gamma = source.gamma;
        let pressure_jump = pi.max(1.0 / pi);
        let normal_mach2 = (1.0 + (pressure_jump - 1.0) * (gamma + 1.0) / (2.0 * gamma))
            .min(source.exit_mach * source.exit_mach);
        let compression = (gamma + 1.0) * normal_mach2 / ((gamma - 1.0) * normal_mach2 + 2.0);
        Self {
            spacing_m,
            radius_m: source.exit_radius_m,
            compression,
            phase: if pi < 1.0 { 0.5 } else { 0.0 },
        }
    }

    /// Relative extinction/density and collisional-emission multipliers.
    /// A round double cone gives the compressed core a spatial diamond
    /// section from every azimuth. Mixing erases cells downstream. The
    /// optical rho^2 proxy is not a combustion spectrum or blackbody law.
    pub fn sample(self, z_m: f64, r_m: f64) -> [f64; 2] {
        if self.spacing_m <= 0.0 || self.compression <= 1.0 {
            return [1.0; 2];
        }
        let x = z_m.max(0.0) / self.spacing_m;
        let u = r_m.abs() / self.radius_m;
        let phase = (x + self.phase).fract();
        let width = 1.0 - (2.0 * phase - 1.0).abs();
        let softness = 0.08 + 0.04 * x;
        let section = width.max(softness);
        let axial = 2.0 * phase - 1.0;
        let body = (-2.0 * ((u / section).powi(2) + axial * axial)).exp();
        let incident = (-0.5 * ((u - axial.abs()) / softness).powi(2)).exp();
        let compression_region = body.max(incident);
        let mask = 1.0 - smoothstep(0.8, 1.2, u);
        let coherence = (-x / 3.0).exp() * (1.0 - (-z_m.max(0.0) / self.radius_m).exp());
        let density = 1.0 / self.compression
            + (self.compression - 1.0 / self.compression) * compression_region;
        let amount = mask * coherence;
        [
            1.0 + amount * (density - 1.0),
            1.0 + amount * (density * density - 1.0),
        ]
    }
}

fn smoothstep(lo: f64, hi: f64, x: f64) -> f64 {
    let t = ((x - lo) / (hi - lo)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::source::tests::{sample_env_sea_level, sample_source};

    #[test]
    fn compression_obeys_mass_momentum_and_energy_jump() {
        let source = sample_source();
        for pi in [0.1, 0.5, 1.0, 2.0, 10.0, 1000.0] {
            let mut env = sample_env_sea_level();
            env.pressure_pa = source.exit_pressure_pa / pi;
            let cells = ShockCells::from_source(&source, &env);
            let gamma = 1.2;
            let m2 = (1.0 + (pi.max(1.0 / pi) - 1.0) * (gamma + 1.0) / (2.0 * gamma))
                .min(source.exit_mach.powi(2));
            // Unit upstream rho/p, speed from Mach. Recover downstream
            // pressure/enthalpy from conservation, independently of closure.
            let v2 = gamma * m2;
            let rho = cells.compression;
            let p = 1.0 + v2 * (1.0 - 1.0 / rho);
            let h0 = gamma / (gamma - 1.0) + v2 / 2.0;
            let h1 = gamma / (gamma - 1.0) * p / rho + v2 / (2.0 * rho * rho);
            assert!((h0 - h1).abs() < 1e-12);
            assert!((1.0..11.0).contains(&rho));
        }
    }

    #[test]
    fn nozzle_matched_vacuum_skirt_and_downstream_limits() {
        let source = sample_source();
        let mut env = sample_env_sea_level();
        env.pressure_pa = source.exit_pressure_pa / 4.0;
        let cells = ShockCells::from_source(&source, &env);
        assert_eq!(cells.sample(0.0, 0.0), [1.0; 2]);
        assert_eq!(
            cells.sample(cells.spacing_m, 2.0 * cells.radius_m),
            [1.0; 2]
        );
        for v in cells.sample(100.0 * cells.spacing_m, 0.0) {
            assert!((v - 1.0).abs() < 1e-12);
        }
        let center = cells.sample(0.5 * cells.spacing_m, 0.4 * cells.radius_m);
        let gap = cells.sample(cells.spacing_m, 0.4 * cells.radius_m);
        assert!(center[1] > gap[1] * 3.0);
        env.pressure_pa = source.exit_pressure_pa;
        assert_eq!(
            ShockCells::from_source(&source, &env).sample(1.0, 0.0),
            [1.0; 2]
        );
        env.pressure_pa = 0.0;
        assert_eq!(ShockCells::from_source(&source, &env), ShockCells::NONE);
    }
}
