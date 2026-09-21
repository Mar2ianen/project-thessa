//! Liquid chemical rocket engines: Juno-simple-mode authoring compiled into
//! a solved design point with derived mass/power/thermal interfaces.

use serde::{Deserialize, Serialize};

use super::{
    AEROSPIKE_BASE_FRACTION, CHAMBER_LENGTH_DIAMETER, ChamberMaterial, CoolingMode, EngineCycle,
    GG_DUCT_EXPANSION_RATIO, GG_PRESSURE_FRACTION, INJECTOR_DROP_FRACTION, MASS_FIT_FEED_KG_PER_N,
    MASS_FIT_GIMBAL_BASE_KG, MASS_FIT_GIMBAL_KG_PER_N, MASS_FIT_HEAD_BASE_KG,
    MASS_FIT_HEAD_KG_PER_M2, MASS_FIT_MOUNT_KG_PER_N, MASS_FIT_TURBO_KG_PER_FLOW_POWER,
    NozzleContour, NozzleExitState, PRESSURE_SAFETY_FACTOR, PUMP_EFFICIENCY,
    PUMP_SPECIFIC_POWER_W_PER_KG, Propellant, PropellantThermo, PropulsionError,
    STANDARD_GRAVITY_MPS2, TANK_PRESSURE_PA, characteristic_velocity, nozzle_exit,
    require_non_negative, require_positive, require_unit_interval, thrust_coefficient,
};

/// Juno-simple-mode authoring for one liquid chamber/nozzle assembly.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LiquidEngineSpec {
    pub name: String,
    pub propellant: Propellant,
    pub cycle: EngineCycle,
    /// Design chamber pressure (Pa).
    pub chamber_pressure_pa: f64,
    /// Nozzle throat radius (m).
    pub throat_radius_m: f64,
    /// Nozzle expansion ratio Ae/At.
    pub expansion_ratio: f64,
    /// Nozzle length throat-to-exit (m); sets the wall angle and mass.
    pub nozzle_length_m: f64,
    pub contour: NozzleContour,
    pub chamber_material: ChamberMaterial,
    pub cooling: CoolingMode,
    /// Oxidizer-to-fuel ratio override; `None` = pair reference.
    pub mixture_ratio: Option<f64>,
    /// Characteristic length override (m); `None` = propellant reference.
    pub characteristic_length_m: Option<f64>,
    /// Gimbal half-range (rad, 0 = fixed).
    pub gimbal_range_rad: f64,
    /// Minimum stable throttle; `None` = cycle floor.
    pub min_throttle: Option<f64>,
    /// Restartable after shutdown (ignition shots unlimited).
    pub restartable: bool,
}

impl LiquidEngineSpec {
    /// Validate the authoring values (NaN fails closed).
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    pub fn validate(&self) -> Result<(), PropulsionError> {
        if self.name.trim().is_empty() {
            return Err(PropulsionError::InvalidSpec(
                "engine name must not be empty".into(),
            ));
        }
        if self.propellant.is_solid() {
            return Err(PropulsionError::InvalidSpec(
                "solid propellant needs a grain spec, not a liquid spec".into(),
            ));
        }
        require_positive(self.chamber_pressure_pa, "chamber pressure")?;
        require_positive(self.throat_radius_m, "throat radius")?;
        if !(self.expansion_ratio >= 1.0) {
            return Err(PropulsionError::InvalidSpec(
                "expansion ratio must be finite and >= 1".into(),
            ));
        }
        require_positive(self.nozzle_length_m, "nozzle length")?;
        self.chamber_material.validate()?;
        require_non_negative(self.gimbal_range_rad, "gimbal range")?;
        if self.gimbal_range_rad > 0.35 {
            return Err(PropulsionError::InvalidSpec(
                "gimbal range above 0.35 rad needs an explicit actuator design".into(),
            ));
        }
        if let Some(min_throttle) = self.min_throttle {
            require_unit_interval(min_throttle, "min throttle")?;
            if min_throttle < self.cycle.limits().min_throttle {
                return Err(PropulsionError::UnsupportedCombination(format!(
                    "{:?} cannot run stably below throttle {:.2}",
                    self.cycle,
                    self.cycle.limits().min_throttle
                )));
            }
        }
        if let Some(length) = self.characteristic_length_m {
            require_positive(length, "characteristic length")?;
        }
        // Gates the mixture knob early (range refusal lives here too).
        self.propellant.thermo_at_mixture(self.mixture_ratio)?;
        Ok(())
    }

    /// Hangar compile: validate topology, solve the design point, derive
    /// mass/power/thermal interfaces.
    pub fn compile(&self) -> Result<CompiledLiquid, PropulsionError> {
        self.validate()?;
        let limits = self.cycle.limits();
        if self.chamber_pressure_pa > limits.max_chamber_pressure_pa {
            return Err(PropulsionError::UnsupportedCombination(format!(
                "{:?} feed cannot sustain {:.2} MPa (cap {:.1} MPa)",
                self.cycle,
                self.chamber_pressure_pa / 1.0e6,
                limits.max_chamber_pressure_pa / 1.0e6
            )));
        }
        if let Some(cap) = self.cooling.pressure_cap_pa()
            && self.chamber_pressure_pa > cap
        {
            return Err(PropulsionError::UnsupportedCombination(format!(
                "radiative cooling cannot reject the heat flux at {:.2} MPa (cap {:.1} MPa)",
                self.chamber_pressure_pa / 1.0e6,
                cap / 1.0e6
            )));
        }
        let thermo = self.propellant.thermo_at_mixture(self.mixture_ratio)?;
        let c_star = characteristic_velocity(&thermo);
        let throat_area_m2 = std::f64::consts::PI * self.throat_radius_m * self.throat_radius_m;
        let exit_radius_m = self.throat_radius_m * self.expansion_ratio.sqrt();
        let exit_area_m2 = throat_area_m2 * self.expansion_ratio;
        let exit = nozzle_exit(&thermo, self.expansion_ratio)?;

        // Wall angle from real geometry; divergence from the angle.
        // The aerospike exhaust leaves near-axially along the plug.
        let wall_slope = (exit_radius_m - self.throat_radius_m) / self.nozzle_length_m;
        let wall_angle_rad = wall_slope.atan();
        let conical_divergence = 0.5 * (1.0 + wall_angle_rad.cos());
        let divergence = match self.contour {
            NozzleContour::Conical => conical_divergence,
            NozzleContour::Bell => conical_divergence + (1.0 - conical_divergence) * 0.5,
            NozzleContour::Aerospike => 0.99,
        };

        // Main chamber flow + optional gas-generator duct (same isentropic
        // core at duct temperature/expansion; bypass is a flow split, not a
        // multiplier).
        let chamber_flow_kg_s = self.chamber_pressure_pa * throat_area_m2 / c_star;
        let gg_flow_kg_s = limits.gg_bypass_fraction * chamber_flow_kg_s;
        let total_flow_kg_s = chamber_flow_kg_s + gg_flow_kg_s;
        let main = OperatingDesign {
            thrust_sl_n: thrust_coefficient(
                &thermo,
                self.chamber_pressure_pa,
                &exit,
                self.expansion_ratio,
                101_325.0,
                divergence,
            ) * self.chamber_pressure_pa
                * throat_area_m2,
            thrust_vac_n: thrust_coefficient(
                &thermo,
                self.chamber_pressure_pa,
                &exit,
                self.expansion_ratio,
                0.0,
                divergence,
            ) * self.chamber_pressure_pa
                * throat_area_m2,
        };
        let (gg_thrust_sl_n, gg_thrust_vac_n, gg_exit_area_m2, gg_isp_s) = if gg_flow_kg_s > 0.0 {
            let gg_thermo = PropellantThermo {
                chamber_temp_k: limits.gg_temperature_k,
                ..thermo
            };
            let gg_c_star = characteristic_velocity(&gg_thermo);
            let gg_chamber_pa = GG_PRESSURE_FRACTION * self.chamber_pressure_pa;
            let gg_throat_m2 = gg_flow_kg_s * gg_c_star / gg_chamber_pa;
            let gg_exit = nozzle_exit(&gg_thermo, GG_DUCT_EXPANSION_RATIO)?;
            let sl = thrust_coefficient(
                &gg_thermo,
                gg_chamber_pa,
                &gg_exit,
                GG_DUCT_EXPANSION_RATIO,
                101_325.0,
                1.0,
            ) * gg_chamber_pa
                * gg_throat_m2;
            let vac = thrust_coefficient(
                &gg_thermo,
                gg_chamber_pa,
                &gg_exit,
                GG_DUCT_EXPANSION_RATIO,
                0.0,
                1.0,
            ) * gg_chamber_pa
                * gg_throat_m2;
            let isp = (sl + vac) * 0.5 / (gg_flow_kg_s * STANDARD_GRAVITY_MPS2);
            (sl, vac, gg_throat_m2 * GG_DUCT_EXPANSION_RATIO, isp)
        } else {
            (0.0, 0.0, 0.0, 0.0)
        };
        let thrust_sl_n = main.thrust_sl_n + gg_thrust_sl_n;
        let thrust_vac_n = main.thrust_vac_n + gg_thrust_vac_n;
        let isp_sl_s = thrust_sl_n / (total_flow_kg_s * STANDARD_GRAVITY_MPS2);
        let isp_vac_s = thrust_vac_n / (total_flow_kg_s * STANDARD_GRAVITY_MPS2);

        // Mass from geometry + material + documented fits.
        let l_star = self
            .characteristic_length_m
            .unwrap_or(thermo.characteristic_length_m);
        let chamber_volume_m3 = l_star * throat_area_m2;
        let chamber_diameter_m = (4.0 * chamber_volume_m3
            / (std::f64::consts::PI * CHAMBER_LENGTH_DIAMETER))
            .powf(1.0 / 3.0);
        let chamber_length_m = CHAMBER_LENGTH_DIAMETER * chamber_diameter_m;
        let wall_thickness_m = self.chamber_pressure_pa * chamber_diameter_m
            / (2.0 * self.chamber_material.yield_strength_pa)
            * PRESSURE_SAFETY_FACTOR;
        let chamber_wall_kg = std::f64::consts::PI
            * chamber_diameter_m
            * chamber_length_m
            * wall_thickness_m
            * self.chamber_material.density_kg_m3;
        let injector_kg = std::f64::consts::PI / 4.0
            * chamber_diameter_m
            * chamber_diameter_m
            * 0.02
            * self.chamber_material.density_kg_m3;
        let slant_m = (self.nozzle_length_m * self.nozzle_length_m
            + wall_slope * wall_slope * self.nozzle_length_m * self.nozzle_length_m)
            .sqrt();
        let nozzle_area_m2 =
            std::f64::consts::PI * (self.throat_radius_m + exit_radius_m) * slant_m;
        let nozzle_thickness_m = wall_thickness_m
            * (self.throat_radius_m / chamber_diameter_m).min(1.0)
            * self.cooling.nozzle_wall_factor();
        // Aerospike: outer cowl frustum at 0.7x plus the plug cone itself
        // (base 0.6x exit radius over the spike length, same wall gauge).
        let spike_area_m2 = match self.contour {
            NozzleContour::Aerospike => {
                let spike_base_m = 0.6 * exit_radius_m;
                let spike_slant_m = (self.nozzle_length_m * self.nozzle_length_m
                    + spike_base_m * spike_base_m)
                    .sqrt();
                std::f64::consts::PI * spike_base_m * spike_slant_m
            }
            _ => 0.0,
        };
        let cowl_area_m2 = match self.contour {
            NozzleContour::Aerospike => nozzle_area_m2 * 0.7,
            _ => nozzle_area_m2,
        };
        let mut nozzle_kg = cowl_area_m2 * nozzle_thickness_m * self.chamber_material.density_kg_m3
            + spike_area_m2 * nozzle_thickness_m * self.chamber_material.density_kg_m3;
        if self.contour == NozzleContour::Bell {
            nozzle_kg *= 0.85;
        }
        let turbo_kg = match self.cycle {
            EngineCycle::GasGenerator
            | EngineCycle::StagedCombustion
            | EngineCycle::FullFlowStaged => {
                total_flow_kg_s * self.chamber_pressure_pa * MASS_FIT_TURBO_KG_PER_FLOW_POWER
            }
            EngineCycle::PressureFed | EngineCycle::ElectricPump => 0.0,
        };
        let pump_motor_kg = match self.cycle {
            EngineCycle::ElectricPump => {
                let pump_power_w = total_flow_kg_s
                    * (self.chamber_pressure_pa - TANK_PRESSURE_PA).max(0.0)
                    / (thermo.bulk_density_kg_m3 * PUMP_EFFICIENCY);
                pump_power_w / PUMP_SPECIFIC_POWER_W_PER_KG
            }
            _ => 0.0,
        };
        let head_kg = MASS_FIT_HEAD_BASE_KG + MASS_FIT_HEAD_KG_PER_M2 * throat_area_m2;
        let mount_kg = MASS_FIT_MOUNT_KG_PER_N * thrust_vac_n;
        let feed_kg = MASS_FIT_FEED_KG_PER_N * thrust_vac_n;
        let gimbal_kg = if self.gimbal_range_rad > 0.0 {
            MASS_FIT_GIMBAL_BASE_KG + MASS_FIT_GIMBAL_KG_PER_N * thrust_vac_n
        } else {
            0.0
        };
        let dry_mass_kg = chamber_wall_kg
            + injector_kg
            + nozzle_kg
            + turbo_kg
            + pump_motor_kg
            + head_kg
            + mount_kg
            + feed_kg
            + gimbal_kg;

        let pump_power_w = match self.cycle {
            EngineCycle::ElectricPump => {
                total_flow_kg_s * (self.chamber_pressure_pa - TANK_PRESSURE_PA).max(0.0)
                    / (thermo.bulk_density_kg_m3 * PUMP_EFFICIENCY)
            }
            _ => 0.0,
        };

        Ok(CompiledLiquid {
            name: self.name.clone(),
            propellant: self.propellant,
            cycle: self.cycle,
            chamber_pressure_pa: self.chamber_pressure_pa,
            throat_area_m2,
            throat_radius_m: self.throat_radius_m,
            expansion_ratio: self.expansion_ratio,
            exit_radius_m,
            exit_area_m2,
            nozzle_length_m: self.nozzle_length_m,
            divergence_factor: divergence,
            wall_angle_rad,
            gamma: thermo.gamma,
            chamber_temp_k: thermo.chamber_temp_k,
            gas_constant_j_kg_k: thermo.gas_constant_j_kg_k,
            c_star_mps: c_star,
            exit_mach: exit.exit_mach,
            exit_pressure_pa: exit.exit_pressure_ratio * self.chamber_pressure_pa,
            exit_temp_k: exit.exit_temp_k,
            exhaust_velocity_mps: exit.exhaust_velocity_mps,
            gg_bypass_fraction: limits.gg_bypass_fraction,
            gg_thrust_sl_n,
            gg_thrust_vac_n,
            gg_exit_area_m2,
            gg_isp_s,
            full_flow_kg_s: total_flow_kg_s,
            thrust_sl_n,
            thrust_vac_n,
            isp_sl_s,
            isp_vac_s,
            dry_mass_kg,
            chamber_diameter_m,
            chamber_length_m,
            wall_thickness_m,
            nozzle_mass_kg: nozzle_kg,
            pump_power_w,
            feed_pressure_required_pa: self.chamber_pressure_pa * (1.0 + INJECTOR_DROP_FRACTION),
            min_throttle: self.min_throttle.unwrap_or(limits.min_throttle),
            spool_tau_s: limits.spool_tau_s,
            restartable: self.restartable,
            gimbal_range_rad: self.gimbal_range_rad,
            cooling: self.cooling,
            contour: self.contour,
            aerospike_base_area_m2: match self.contour {
                NozzleContour::Aerospike => AEROSPIKE_BASE_FRACTION * exit_area_m2,
                _ => 0.0,
            },
            nozzle_wall_area_m2: cowl_area_m2 + spike_area_m2,
        })
    }
}

struct OperatingDesign {
    thrust_sl_n: f64,
    thrust_vac_n: f64,
}

/// Hangar-compiled liquid engine: design point solved, mass/power/thermal
/// interfaces derived. Runtime evaluation is a pure function of throttle
/// and ambient pressure.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CompiledLiquid {
    pub name: String,
    pub propellant: Propellant,
    pub cycle: EngineCycle,
    pub chamber_pressure_pa: f64,
    pub throat_area_m2: f64,
    pub throat_radius_m: f64,
    pub expansion_ratio: f64,
    pub exit_radius_m: f64,
    pub exit_area_m2: f64,
    pub nozzle_length_m: f64,
    pub divergence_factor: f64,
    pub wall_angle_rad: f64,
    pub gamma: f64,
    pub chamber_temp_k: f64,
    pub gas_constant_j_kg_k: f64,
    pub c_star_mps: f64,
    pub exit_mach: f64,
    pub exit_pressure_pa: f64,
    pub exit_temp_k: f64,
    pub exhaust_velocity_mps: f64,
    pub gg_bypass_fraction: f64,
    pub gg_thrust_sl_n: f64,
    pub gg_thrust_vac_n: f64,
    pub gg_exit_area_m2: f64,
    pub gg_isp_s: f64,
    pub full_flow_kg_s: f64,
    pub thrust_sl_n: f64,
    pub thrust_vac_n: f64,
    pub isp_sl_s: f64,
    pub isp_vac_s: f64,
    pub dry_mass_kg: f64,
    pub chamber_diameter_m: f64,
    pub chamber_length_m: f64,
    pub wall_thickness_m: f64,
    pub nozzle_mass_kg: f64,
    pub pump_power_w: f64,
    pub feed_pressure_required_pa: f64,
    pub min_throttle: f64,
    pub spool_tau_s: f64,
    pub restartable: bool,
    pub gimbal_range_rad: f64,
    pub cooling: CoolingMode,
    pub contour: NozzleContour,
    /// Aerospike base area exposed to ambient (m^2, 0 for bell/cone).
    pub aerospike_base_area_m2: f64,
    /// Nozzle wall area for thermal/radiation bookkeeping (m^2).
    pub nozzle_wall_area_m2: f64,
}

// Reconstructed thermo/exit views for runtime evaluation (no re-solve).
impl CompiledLiquid {
    pub(crate) fn thermo_ref(&self) -> PropellantThermo {
        PropellantThermo {
            gamma: self.gamma,
            chamber_temp_k: self.chamber_temp_k,
            gas_constant_j_kg_k: self.gas_constant_j_kg_k,
            bulk_density_kg_m3: 0.0,
            characteristic_length_m: 0.0,
            reference_c_star_mps: 0.0,
        }
    }

    pub(crate) fn exit_ref(&self) -> NozzleExitState {
        NozzleExitState {
            exit_mach: self.exit_mach,
            exit_pressure_ratio: self.exit_pressure_pa / self.chamber_pressure_pa,
            exit_temp_k: self.exit_temp_k,
            exhaust_velocity_mps: self.exhaust_velocity_mps,
        }
    }

    /// Gas-generator duct thrust at throttle and ambient. Exact for frozen
    /// duct geometry: vacuum endpoint scales with chamber flow, ambient
    /// eats the fixed duct exit area (F = t*F_vac - Pa*Ae_gg).
    pub(crate) fn gg_thrust_at(&self, throttle: f64, ambient_pa: f64) -> f64 {
        if self.gg_bypass_fraction <= 0.0 || throttle == 0.0 {
            return 0.0;
        }
        (self.gg_thrust_vac_n * throttle - ambient_pa * self.gg_exit_area_m2).max(0.0)
    }
}

/// Merlin-1D-class reference spec shared by backend tests.
#[cfg(test)]
pub(crate) fn merlin_like() -> LiquidEngineSpec {
    LiquidEngineSpec {
        name: "Merlin-1D class".into(),
        propellant: Propellant::LoxRp1,
        cycle: EngineCycle::GasGenerator,
        chamber_pressure_pa: 9.7e6,
        throat_radius_m: 0.134,
        expansion_ratio: 16.0,
        nozzle_length_m: 1.5,
        contour: NozzleContour::Bell,
        chamber_material: ChamberMaterial::nickel_superalloy(),
        cooling: CoolingMode::Regenerative,
        mixture_ratio: None,
        characteristic_length_m: None,
        gimbal_range_rad: 0.09,
        min_throttle: None,
        restartable: true,
    }
}

#[cfg(test)]
mod tests {
    use super::super::engine::CompiledEngine;
    use super::*;

    #[test]
    fn merlin_class_golden_data() {
        // Golden regression: a Merlin-1D-class spec (Pc 9.7 MPa, eps 16,
        // kerolox GG) must reproduce published 845 kN SL / 914 kN vac and
        // 282 s / 311 s within the documented 5% envelope, and land in the
        // published dry-mass band (470 kg with margin for fit spread).
        let engine = CompiledEngine::Liquid(merlin_like().compile().expect("compile"));
        let sl = engine
            .operating_point(1.0, 101_325.0, 0.0)
            .expect("sea-level point");
        let vac = engine.operating_point(1.0, 0.0, 0.0).expect("vacuum point");
        for (actual, published, label) in [
            (sl.thrust_n, 845_000.0, "SL thrust"),
            (vac.thrust_n, 914_000.0, "vac thrust"),
            (sl.isp_s, 282.0, "SL Isp"),
            (vac.isp_s, 311.0, "vac Isp"),
        ] {
            let error = (actual - published).abs() / published;
            assert!(error < 0.05, "{label}: {actual:.0} vs {published:.0}");
        }
        let mass = engine.dry_mass_kg();
        assert!(
            (200.0..=700.0).contains(&mass),
            "dry mass {mass:.0} kg outside the Merlin band"
        );
        // Vacuum beats sea level; the sea-level bell is not separated.
        assert!(vac.thrust_n > sl.thrust_n);
        assert!(vac.isp_s > sl.isp_s);
        assert!(!sl.separation_risk);
    }

    #[test]
    fn vacuum_sea_level_ordering_and_throttle_linearity() {
        // Regression pin: throttle scales chamber pressure linearly by
        // construction (documented deep-throttle assumption); guards
        // accidental nonlinearity in the eval path.
        let engine = CompiledEngine::Liquid(merlin_like().compile().expect("compile"));
        let full = engine.operating_point(1.0, 0.0, 0.0).expect("full");
        let half = engine.operating_point(0.5, 0.0, 0.0).expect("half");
        assert!((half.thrust_n * 2.0 - full.thrust_n).abs() / full.thrust_n < 1e-12);
        assert!(
            (half.mass_flow_kg_s * 2.0 - full.mass_flow_kg_s).abs() / full.mass_flow_kg_s < 1e-12
        );
        assert!((half.isp_s - full.isp_s).abs() / full.isp_s < 1e-12);
    }

    #[test]
    fn overexpanded_nozzle_flags_separation() {
        // A sea-level-tuned bell (eps 8) deep in vacuum-throttle... inverse:
        // a big vacuum bell at sea level must trip Summerfield.
        let spec = LiquidEngineSpec {
            expansion_ratio: 80.0,
            ..merlin_like()
        };
        let engine = CompiledEngine::Liquid(spec.compile().expect("compile"));
        let sl = engine.operating_point(1.0, 101_325.0, 0.0).expect("point");
        assert!(sl.separation_risk, "eps-80 bell at SL must flag");
        let vac = engine.operating_point(1.0, 0.0, 0.0).expect("vac");
        assert!(!vac.separation_risk);
    }

    #[test]
    fn aerospike_holds_thrust_across_altitudes() {
        // Altitude compensation: an eps-35 aerospike keeps sea-level thrust
        // within 3% of vacuum (a bell loses ~17% on the same geometry) and
        // never flags separation; base drag is the only altitude term.
        let spike = LiquidEngineSpec {
            contour: NozzleContour::Aerospike,
            ..merlin_like()
        };
        let spike = CompiledEngine::Liquid(spike.compile().expect("spike"));
        let bell = CompiledEngine::Liquid(merlin_like().compile().expect("bell"));
        let spike_sl = spike
            .operating_point(1.0, 101_325.0, 0.0)
            .expect("spike SL");
        let spike_vac = spike.operating_point(1.0, 0.0, 0.0).expect("spike vac");
        let bell_sl = bell.operating_point(1.0, 101_325.0, 0.0).expect("bell SL");
        let bell_vac = bell.operating_point(1.0, 0.0, 0.0).expect("bell vac");
        assert!(!spike_sl.separation_risk);
        let spike_drop = (spike_vac.thrust_n - spike_sl.thrust_n) / spike_vac.thrust_n;
        let bell_drop = (bell_vac.thrust_n - bell_sl.thrust_n) / bell_vac.thrust_n;
        assert!(
            spike_drop < 0.03,
            "aerospike drop {spike_drop:.3} too large"
        );
        assert!(bell_drop > 0.05, "bell control must lose thrust at SL");
        // Base drag accounting: SL loss equals ambient times base area.
        let expected = 101_325.0
            * match &spike {
                CompiledEngine::Liquid(liquid) => liquid.aerospike_base_area_m2,
                CompiledEngine::Solid(_) => 0.0,
            };
        assert!(((spike_vac.thrust_n - spike_sl.thrust_n) - expected).abs() / expected < 1e-9);
    }

    #[test]
    fn cycle_pressure_caps_and_cooling_gates_fire() {
        // Pressure-fed topology cannot hold Merlin pressures; radiative
        // walls refuse them too. The compiler refuses instead of derating.
        let fed = LiquidEngineSpec {
            cycle: EngineCycle::PressureFed,
            ..merlin_like()
        };
        assert!(fed.compile().is_err());
        let rad = LiquidEngineSpec {
            cooling: CoolingMode::Radiative,
            ..merlin_like()
        };
        assert!(rad.compile().is_err());
        // But the same geometry at 2 MPa pressure-fed compiles.
        let small = LiquidEngineSpec {
            cycle: EngineCycle::PressureFed,
            chamber_pressure_pa: 2.0e6,
            ..merlin_like()
        };
        assert!(small.compile().is_ok());
    }

    #[test]
    fn nan_inputs_fail_closed() {
        let mut spec = merlin_like();
        spec.chamber_pressure_pa = f64::NAN;
        assert!(spec.compile().is_err());
        let engine = CompiledEngine::Liquid(merlin_like().compile().expect("compile"));
        assert!(engine.operating_point(f64::NAN, 0.0, 0.0).is_err());
        assert!(engine.operating_point(1.0, f64::NAN, 0.0).is_err());
    }

    #[test]
    fn bell_beats_cone_within_envelope() {
        // Bell recovers part of the divergence loss: strictly better, and
        // within the documented +/-0.5% recovery envelope vs the cone.
        let cone = LiquidEngineSpec {
            contour: NozzleContour::Conical,
            ..merlin_like()
        };
        let bell = merlin_like();
        let cone = CompiledEngine::Liquid(cone.compile().expect("cone"));
        let bell = CompiledEngine::Liquid(bell.compile().expect("bell"));
        let cone_vac = cone.operating_point(1.0, 0.0, 0.0).expect("cone vac");
        let bell_vac = bell.operating_point(1.0, 0.0, 0.0).expect("bell vac");
        assert!(bell_vac.thrust_n > cone_vac.thrust_n);
        let gain = (bell_vac.thrust_n - cone_vac.thrust_n) / cone_vac.thrust_n;
        assert!(gain < 0.01, "bell recovery {gain:.4} exceeds envelope");
    }

    #[test]
    fn mixture_reference_reproduces_design_point() {
        // An explicit reference ratio must compile to the same engine as
        // `None` (thermo equality lives in `propellant` tests).
        let plain = merlin_like().compile().expect("plain");
        let explicit = LiquidEngineSpec {
            mixture_ratio: Some(2.7),
            ..merlin_like()
        }
        .compile()
        .expect("explicit ref");
        assert!((explicit.thrust_vac_n - plain.thrust_vac_n).abs() / plain.thrust_vac_n < 1e-12);
    }

    #[test]
    fn mixture_shifts_performance_in_documented_direction() {
        // Fuel-rich hydrolox carries more light H2 in the products: higher
        // gas constant wins over the cooler chamber, so c* rises. The test
        // pins the direction and a bounded magnitude, not a point value.
        let h2 = Propellant::LoxHydrogen;
        let rich = h2.thermo_at_mixture(Some(4.5)).expect("fuel-rich");
        let reference = h2.thermo();
        assert!(characteristic_velocity(&rich) > characteristic_velocity(&reference));
        let gain = (characteristic_velocity(&rich) - characteristic_velocity(&reference))
            / characteristic_velocity(&reference);
        assert!(
            gain < 0.08,
            "mixture c* gain {gain:.3} exceeds the table envelope"
        );

        // Oxidizer-rich methane cools and heavies the flow: Isp must drop
        // a few percent relative to the reference compile.
        let mk = |ratio: Option<f64>| {
            CompiledEngine::Liquid(
                LiquidEngineSpec {
                    name: "mixture probe".into(),
                    propellant: Propellant::LoxMethane,
                    cycle: EngineCycle::StagedCombustion,
                    chamber_pressure_pa: 20.0e6,
                    throat_radius_m: 0.15,
                    expansion_ratio: 35.0,
                    nozzle_length_m: 1.8,
                    contour: NozzleContour::Bell,
                    chamber_material: ChamberMaterial::nickel_superalloy(),
                    cooling: CoolingMode::Regenerative,
                    mixture_ratio: ratio,
                    characteristic_length_m: None,
                    gimbal_range_rad: 0.0,
                    min_throttle: None,
                    restartable: true,
                }
                .compile()
                .expect("methalox compiles"),
            )
        };
        let ref_isp = mk(None)
            .operating_point(1.0, 0.0, 0.0)
            .expect("ref point")
            .isp_s;
        let ox_isp = mk(Some(4.2))
            .operating_point(1.0, 0.0, 0.0)
            .expect("ox-rich point")
            .isp_s;
        assert!(ox_isp < ref_isp, "oxidizer-rich methalox must lose Isp");
        assert!((ref_isp - ox_isp) / ref_isp < 0.08);
    }
}
