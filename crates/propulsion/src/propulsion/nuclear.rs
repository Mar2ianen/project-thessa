//! Nuclear thermal propulsion: a reactor heats a working fluid that expands
//! through a nozzle (docs/details/04 section 7). Feed and nozzle concepts
//! are shared with chemical rockets; the enthalpy source differs.
//!
//! Power-limited compile: mass flow comes from the reactor power balance
//! (mdot = P / cp·ΔT), chamber pressure follows from choked flow, thrust
//! from the shared isentropic core at core temperature. Frozen-flow
//! dissociation losses for hot hydrogen ride on `kinetic_efficiency`
//! (documented, NERVA-pinned) instead of a thrust multiplier. The compiled
//! output is a plain [`CompiledLiquid`] (spool, throttle, plume, and
//! analyzer paths reuse it) plus an [`NtrSupplement`] with reactor data.
//!
//! Plume-family mapping for the game-side choke point: all NTR exhaust
//! reads as nuclear-thermal.

// Validity guards use `!(x > 0.0)` so NaN fails closed (repo convention).
#![allow(clippy::neg_cmp_op_on_partial_ord)]

use serde::{Deserialize, Serialize};

use super::AEROSPIKE_BASE_FRACTION;
use super::{
    ChamberMaterial, CompiledLiquid, CoolingMode, EngineCycle, NozzleContour, Propellant,
    PropulsionError, STANDARD_GRAVITY_MPS2, nozzle_exit, require_non_negative, require_positive,
    thrust_coefficient,
};
use super::{MASS_FIT_FEED_KG_PER_N, MASS_FIT_GIMBAL_BASE_KG, MASS_FIT_GIMBAL_KG_PER_N};
use super::{MASS_FIT_HEAD_BASE_KG, MASS_FIT_HEAD_KG_PER_M2, MASS_FIT_MOUNT_KG_PER_N};
use super::{MASS_FIT_TURBO_KG_PER_FLOW_POWER, PRESSURE_SAFETY_FACTOR};

/// Frozen-flow kinetic/dissociation efficiency for hot hydrogen expansion
/// (documented: H2 dissociation at 2500 K+ is invisible to frozen flow;
/// the NERVA golden test pins the full chain).
pub const NTR_KINETIC_EFFICIENCY: f64 = 0.90;
/// Expander-cycle chamber-pressure cap (Pa, documented).
pub const NTR_MAX_CHAMBER_PRESSURE_PA: f64 = 12.0e6;
/// Cryogenic propellant inlet temperature after regen pickup (K, documented).
pub const NTR_INLET_TEMP_K: f64 = 150.0;
/// Decay-heat cooldown impulse as a fraction of burn impulse (documented).
pub const NTR_COOLDOWN_FRACTION: f64 = 0.04;
/// Default reactor specific mass (kg per MW thermal, NERVA-class fit).
pub const NTR_DEFAULT_SPECIFIC_MASS_KG_PER_MW: f64 = 10.0;
/// Default startup time to full power (s, NERVA-class demonstrated).
pub const NTR_DEFAULT_STARTUP_TAU_S: f64 = 60.0;
/// Default rated continuous burn (s, fuel-lifetime class bound).
pub const NTR_DEFAULT_RATED_BURN_S: f64 = 7200.0;

/// Reactor working fluid: hot-gas properties (documented approximations;
/// chamber temperature comes from the core, not the pair).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum NtrFluid {
    Hydrogen,
    Methane,
    Ammonia,
    Water,
}

impl NtrFluid {
    /// (gamma, gas constant J/kg/K, liquid storage density kg/m^3).
    pub fn properties(self) -> (f64, f64, f64) {
        match self {
            Self::Hydrogen => (1.30, 4157.0, 71.0),
            Self::Methane => (1.28, 560.0, 422.0),
            Self::Ammonia => (1.30, 800.0, 610.0),
            Self::Water => (1.30, 462.0, 1000.0),
        }
    }

    /// Editor propellant for tank sizing (bulk density source).
    pub fn tank_propellant(self) -> Propellant {
        match self {
            Self::Hydrogen => Propellant::LoxHydrogen,
            Self::Methane => Propellant::LoxMethane,
            Self::Ammonia => Propellant::NtoMmh,
            Self::Water => Propellant::ColdGasNitrogen,
        }
    }
}

/// Nuclear thermal engine authoring: reactor plus nozzle assembly.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NuclearThermalSpec {
    pub name: String,
    pub fluid: NtrFluid,
    /// Fuel/core temperature (K).
    pub core_temp_k: f64,
    /// Reactor thermal power (MW).
    pub core_power_mw: f64,
    /// Reactor specific mass override (kg/MW thermal).
    pub reactor_specific_mass_kg_per_mw: Option<f64>,
    pub throat_radius_m: f64,
    pub expansion_ratio: f64,
    pub nozzle_length_m: f64,
    pub contour: NozzleContour,
    pub material: ChamberMaterial,
    pub cooling: CoolingMode,
    pub gimbal_range_rad: f64,
    /// Startup time constant override (s).
    pub startup_tau_s: Option<f64>,
    /// Minimum stable throttle override.
    pub min_throttle: Option<f64>,
}

/// Reactor-side data accompanying the compiled liquid core.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct NtrSupplement {
    pub core_power_mw: f64,
    pub core_temp_k: f64,
    pub reactor_mass_kg: f64,
    pub startup_tau_s: f64,
    pub rated_burn_time_s: f64,
    pub cooldown_fraction: f64,
}

impl NtrSupplement {
    /// Decay-heat tail impulse for a burn delivering `burn_impulse_ns`.
    pub fn cooldown_impulse_ns(&self, burn_impulse_ns: f64) -> f64 {
        if !burn_impulse_ns.is_finite() || burn_impulse_ns <= 0.0 {
            return 0.0;
        }
        self.cooldown_fraction * burn_impulse_ns
    }
}

impl NuclearThermalSpec {
    /// Validate authoring values (NaN fails closed).
    pub fn validate(&self) -> Result<(), PropulsionError> {
        if self.name.trim().is_empty() {
            return Err(PropulsionError::InvalidSpec(
                "engine name must not be empty".into(),
            ));
        }
        require_positive(self.core_temp_k, "core temperature")?;
        if self.core_temp_k > 3200.0 {
            return Err(PropulsionError::InvalidSpec(
                "solid-core fuel above 3200 K needs an explicit fuel design".into(),
            ));
        }
        require_positive(self.core_power_mw, "core power")?;
        require_positive(self.throat_radius_m, "throat radius")?;
        if !(self.expansion_ratio >= 1.0) {
            return Err(PropulsionError::InvalidSpec(
                "expansion ratio must be finite and >= 1".into(),
            ));
        }
        require_positive(self.nozzle_length_m, "nozzle length")?;
        self.material.validate()?;
        require_non_negative(self.gimbal_range_rad, "gimbal range")?;
        if let Some(specific) = self.reactor_specific_mass_kg_per_mw {
            require_positive(specific, "reactor specific mass")?;
        }
        if let Some(tau) = self.startup_tau_s {
            require_positive(tau, "startup tau")?;
        }
        if let Some(min_throttle) = self.min_throttle
            && (!min_throttle.is_finite() || !(0.0..=1.0).contains(&min_throttle))
        {
            return Err(PropulsionError::InvalidSpec(
                "min throttle must be finite in [0, 1]".into(),
            ));
        }
        Ok(())
    }

    /// Hangar compile: power balance first, then the shared nozzle core.
    /// Returns the runtime liquid core plus reactor-side data.
    pub fn compile(&self) -> Result<(CompiledLiquid, NtrSupplement), PropulsionError> {
        self.validate()?;
        let (gamma, gas_r, storage_density) = self.fluid.properties();
        let cp = gamma * gas_r / (gamma - 1.0);
        let power_w = self.core_power_mw * 1.0e6;
        // Reactor power balance (exact, pinned by test): the full thermal
        // power heats the flow from inlet to core temperature.
        let mass_flow_kg_s = power_w / (cp * (self.core_temp_k - NTR_INLET_TEMP_K));
        // Choked-flow consistency sets the chamber pressure for the
        // authored throat (no independent Pc knob on a power-limited core).
        let gamma_fn =
            gamma.sqrt() * (2.0 / (gamma + 1.0)).powf((gamma + 1.0) / (2.0 * (gamma - 1.0)));
        let c_star = (gas_r * self.core_temp_k).sqrt() / gamma_fn;
        let throat_area_m2 = std::f64::consts::PI * self.throat_radius_m * self.throat_radius_m;
        let chamber_pa = mass_flow_kg_s * c_star / throat_area_m2;
        if chamber_pa > NTR_MAX_CHAMBER_PRESSURE_PA {
            return Err(PropulsionError::UnsupportedCombination(format!(
                "expander feed cannot sustain {:.2} MPa (cap {:.1} MPa): open the throat",
                chamber_pa / 1.0e6,
                NTR_MAX_CHAMBER_PRESSURE_PA / 1.0e6
            )));
        }
        let thermo = super::PropellantThermo {
            gamma,
            chamber_temp_k: self.core_temp_k,
            gas_constant_j_kg_k: gas_r,
            bulk_density_kg_m3: storage_density,
            characteristic_length_m: 0.0,
            reference_c_star_mps: 0.0,
        };
        let exit_radius_m = self.throat_radius_m * self.expansion_ratio.sqrt();
        let exit_area_m2 = throat_area_m2 * self.expansion_ratio;
        let exit = nozzle_exit(&thermo, self.expansion_ratio)?;
        let wall_slope = (exit_radius_m - self.throat_radius_m) / self.nozzle_length_m;
        let conical = 0.5 * (1.0 + wall_slope.atan().cos());
        let divergence = match self.contour {
            NozzleContour::Conical => conical,
            NozzleContour::Bell => conical + (1.0 - conical) * 0.5,
            NozzleContour::Aerospike => 0.99,
        };
        let ideal_vac = thrust_coefficient(
            &thermo,
            chamber_pa,
            &exit,
            self.expansion_ratio,
            0.0,
            divergence,
        ) * chamber_pa
            * throat_area_m2;
        let ideal_sl = thrust_coefficient(
            &thermo,
            chamber_pa,
            &exit,
            self.expansion_ratio,
            101_325.0,
            divergence,
        ) * chamber_pa
            * throat_area_m2;
        let thrust_vac_n = ideal_vac * NTR_KINETIC_EFFICIENCY;
        let thrust_sl_n = ideal_sl * NTR_KINETIC_EFFICIENCY;
        let isp_vac_s = thrust_vac_n / (mass_flow_kg_s * STANDARD_GRAVITY_MPS2);
        let isp_sl_s = thrust_sl_n / (mass_flow_kg_s * STANDARD_GRAVITY_MPS2);

        // Mass: reactor (specific-power fit) + nozzle/head/feed/mount
        // hardware on the same wall math as chemical engines.
        let specific = self
            .reactor_specific_mass_kg_per_mw
            .unwrap_or(NTR_DEFAULT_SPECIFIC_MASS_KG_PER_MW);
        let reactor_kg = self.core_power_mw * specific;
        let slant_m = (self.nozzle_length_m * self.nozzle_length_m
            + (exit_radius_m - self.throat_radius_m).powi(2))
        .sqrt();
        let wall_m = chamber_pa * self.throat_radius_m / (2.0 * self.material.yield_strength_pa)
            * PRESSURE_SAFETY_FACTOR
            * 0.5;
        let mut nozzle_kg = std::f64::consts::PI
            * (self.throat_radius_m + exit_radius_m)
            * slant_m
            * wall_m
            * self.material.density_kg_m3;
        if self.contour == NozzleContour::Bell {
            nozzle_kg *= 0.85;
        }
        let turbo_kg = mass_flow_kg_s * chamber_pa * MASS_FIT_TURBO_KG_PER_FLOW_POWER;
        let head_kg = MASS_FIT_HEAD_BASE_KG + MASS_FIT_HEAD_KG_PER_M2 * throat_area_m2;
        let mount_kg = MASS_FIT_MOUNT_KG_PER_N * thrust_vac_n;
        let feed_kg = MASS_FIT_FEED_KG_PER_N * thrust_vac_n;
        let gimbal_kg = if self.gimbal_range_rad > 0.0 {
            MASS_FIT_GIMBAL_BASE_KG + MASS_FIT_GIMBAL_KG_PER_N * thrust_vac_n
        } else {
            0.0
        };
        let dry_mass_kg =
            reactor_kg + nozzle_kg + turbo_kg + head_kg + mount_kg + feed_kg + gimbal_kg;
        let startup_tau_s = self.startup_tau_s.unwrap_or(NTR_DEFAULT_STARTUP_TAU_S);

        let engine = CompiledLiquid {
            name: self.name.clone(),
            // Plume-label approximation for the working fluid (the thermo
            // above is authoritative; this selects render hues downstream).
            propellant: self.fluid.tank_propellant(),
            cycle: EngineCycle::StagedCombustion,
            chamber_pressure_pa: chamber_pa,
            throat_area_m2,
            throat_radius_m: self.throat_radius_m,
            expansion_ratio: self.expansion_ratio,
            exit_radius_m,
            exit_area_m2,
            nozzle_length_m: self.nozzle_length_m,
            divergence_factor: divergence,
            wall_angle_rad: wall_slope.atan(),
            gamma,
            chamber_temp_k: self.core_temp_k,
            gas_constant_j_kg_k: gas_r,
            c_star_mps: c_star,
            exit_mach: exit.exit_mach,
            exit_pressure_pa: exit.exit_pressure_ratio * chamber_pa,
            exit_temp_k: exit.exit_temp_k,
            exhaust_velocity_mps: exit.exhaust_velocity_mps,
            gg_bypass_fraction: 0.0,
            gg_thrust_sl_n: 0.0,
            gg_thrust_vac_n: 0.0,
            gg_exit_area_m2: 0.0,
            gg_isp_s: 0.0,
            full_flow_kg_s: mass_flow_kg_s,
            thrust_sl_n,
            thrust_vac_n,
            isp_sl_s,
            isp_vac_s,
            dry_mass_kg,
            chamber_diameter_m: 2.0 * self.throat_radius_m,
            chamber_length_m: 0.0,
            wall_thickness_m: wall_m,
            nozzle_mass_kg: nozzle_kg,
            pump_power_w: 0.0,
            feed_pressure_required_pa: chamber_pa,
            min_throttle: self.min_throttle.unwrap_or(0.5),
            spool_tau_s: startup_tau_s,
            restartable: true,
            gimbal_range_rad: self.gimbal_range_rad,
            cooling: self.cooling,
            contour: self.contour,
            aerospike_base_area_m2: match self.contour {
                NozzleContour::Aerospike => AEROSPIKE_BASE_FRACTION * exit_area_m2,
                _ => 0.0,
            },
            nozzle_wall_area_m2: std::f64::consts::PI
                * (self.throat_radius_m + exit_radius_m)
                * slant_m,
            kinetic_efficiency: NTR_KINETIC_EFFICIENCY,
        };
        let supplement = NtrSupplement {
            core_power_mw: self.core_power_mw,
            core_temp_k: self.core_temp_k,
            reactor_mass_kg: reactor_kg,
            startup_tau_s,
            rated_burn_time_s: NTR_DEFAULT_RATED_BURN_S,
            cooldown_fraction: NTR_COOLDOWN_FRACTION,
        };
        Ok((engine, supplement))
    }
}

#[cfg(test)]
mod tests {
    use super::super::CompiledEngine;
    use super::*;

    fn nerva_like() -> NuclearThermalSpec {
        NuclearThermalSpec {
            name: "NERVA class".into(),
            fluid: NtrFluid::Hydrogen,
            core_temp_k: 2700.0,
            core_power_mw: 1120.0,
            reactor_specific_mass_kg_per_mw: None,
            throat_radius_m: 0.11,
            expansion_ratio: 100.0,
            nozzle_length_m: 2.0,
            contour: NozzleContour::Bell,
            material: ChamberMaterial::nickel_superalloy(),
            cooling: CoolingMode::Regenerative,
            gimbal_range_rad: 0.05,
            startup_tau_s: None,
            min_throttle: None,
        }
    }

    #[test]
    fn nerva_class_golden_data() {
        // NERVA-class solid core (2700 K, ~1.1 GW): vacuum Isp in the
        // 750-950 s band, thrust in the 150-350 kN band, dry mass
        // reactor-dominated inside 8-20 t.
        let (liquid, supplement) = nerva_like().compile().expect("nerva compiles");
        let engine = CompiledEngine::Liquid(liquid);
        let vac = engine.operating_point(1.0, 0.0, 0.0).expect("vac");
        assert!(
            (750.0..=950.0).contains(&vac.isp_s),
            "NTR Isp {} s outside the band",
            vac.isp_s
        );
        assert!(
            (150_000.0..=350_000.0).contains(&vac.thrust_n),
            "NTR thrust {} N outside the band",
            vac.thrust_n
        );
        let mass = engine.dry_mass_kg();
        assert!(
            (8_000.0..=20_000.0).contains(&mass),
            "NTR dry mass {mass:.0} kg outside the band"
        );
        // Power balance is exact by construction: the full thermal power
        // heats the booked flow from inlet to core temperature.
        let cp = 1.30 * 4157.0 / 0.30;
        let balance = vac.mass_flow_kg_s * cp * (2700.0 - NTR_INLET_TEMP_K);
        assert!((balance - 1.12e9).abs() / 1.12e9 < 1e-9);
        // Startup wiring: the compiled spool constant is the reactor tau,
        // restarts are free (XE-class demonstrated dozens of restarts).
        assert_eq!(supplement.startup_tau_s, NTR_DEFAULT_STARTUP_TAU_S);
        assert_eq!(engine.spool_tau_s(), NTR_DEFAULT_STARTUP_TAU_S);
        let (floor, _) = engine.throttle_range();
        assert!((floor - 0.5).abs() < 1e-12);
        // Cooldown tail is positive and small.
        let burn_impulse = vac.thrust_n * 3600.0;
        let tail = supplement.cooldown_impulse_ns(burn_impulse);
        assert!(tail > 0.0 && tail < 0.1 * burn_impulse);
    }

    #[test]
    fn hydrogen_beats_heavier_fluids() {
        // At fixed power and core temperature, light exhaust wins: H2 Isp
        // must clear ammonia by a wide margin (same physics, no tuning).
        for (fluid, label) in [(NtrFluid::Hydrogen, "H2"), (NtrFluid::Ammonia, "NH3")] {
            let spec = NuclearThermalSpec {
                fluid,
                core_power_mw: 500.0,
                ..nerva_like()
            };
            let (liquid, _) = spec.compile().expect("compiles");
            let isp = CompiledEngine::Liquid(liquid)
                .operating_point(1.0, 0.0, 0.0)
                .expect("point")
                .isp_s;
            if label == "H2" {
                assert!(isp > 700.0, "H2 Isp {isp:.0} too low");
            } else {
                assert!(isp < 500.0, "NH3 Isp {isp:.0} too high");
            }
        }
    }

    #[test]
    fn expander_cap_refuses_undersized_throat() {
        // A pinhole throat drives chamber pressure past the expander cap:
        // the compiler refuses and tells the author to open the throat.
        let spec = NuclearThermalSpec {
            throat_radius_m: 0.02,
            ..nerva_like()
        };
        assert!(spec.compile().is_err());
        // Core temperature above solid-fuel class needs an explicit design.
        let hot = NuclearThermalSpec {
            core_temp_k: 3500.0,
            ..nerva_like()
        };
        assert!(hot.compile().is_err());
    }
}
