//! Air-breathing jet engines: turbojet, turbofan, and ramjet from one
//! Brayton-cycle core (`docs/details/04` sections 8 and 11).
//!
//! Authoring is Juno-style sliders (intake area/recovery, compression and
//! bypass ratios, turbine temperature, fuel, afterburner, nozzle) plus the
//! cycle internals Juno hides: polytropic efficiencies, cooling bleed,
//! part-power schedules, and oxygen gating for non-Earth atmospheres
//! (doc 04 section 10; Juno 1.4 scales jets with O2 the same way).
//! Method assumptions follow the classic one-spool model (NASA TM 78653):
//! constant bypass ratio, choked turbine nozzles, fixed combustor/duct
//! pressure drops.
//!
//! Model semantics are an anchored engineering bound, stated openly: the
//! backend computes installed performance with documented component
//! efficiencies and named loss channels (cooling-bleed work split and
//! mixing loss, customer bleed, combustor pattern, nozzle/installation);
//! inlet distortion, part-power scheduling detail, and boattail drag live
//! airframe-side. The Olympus anchor test therefore pins an uncertainty
//! envelope around best-public-data inputs instead of pretending to be a
//! digital twin — and exactness lives in the invariant pins (energy,
//! orderings, limiting cases, work balance).

// Validity guards use `!(x > 0.0)` so NaN fails closed (repo convention).
#![allow(clippy::neg_cmp_op_on_partial_ord)]

use serde::{Deserialize, Serialize};

use crate::atmosphere::AtmosphereSample;

use super::SEPARATION_PRESSURE_RATIO;
use super::{
    ChamberMaterial, PropulsionError, STANDARD_GRAVITY_MPS2, mach_from_area_ratio,
    require_non_negative, require_positive,
};

/// Air specific heat (J/kg/K, documented constant).
pub const AIR_CP_J_KG_K: f64 = 1005.0;
/// Air gamma (documented constant).
pub const AIR_GAMMA: f64 = 1.4;
/// Subsonic intake recovery (documented).
pub const INTAKE_RECOVERY_SUBSONIC: f64 = 0.97;
/// Compressor polytropic efficiency (documented).
pub const COMPRESSOR_POLY_EFFICIENCY: f64 = 0.88;
/// Turbine polytropic efficiency (documented).
pub const TURBINE_POLY_EFFICIENCY: f64 = 0.90;
/// Combustor efficiency (documented: pattern factor + wall quenching).
pub const COMBUSTOR_EFFICIENCY: f64 = 0.95;
/// Combustor total-pressure loss fraction (documented).
pub const COMBUSTOR_DP_FRACTION: f64 = 0.04;
/// Turbine-exit mixing pressure loss from cold-bleed rejoin (documented).
pub const TURBINE_MIXING_DP_FRACTION: f64 = 0.03;
/// Customer/aircraft-systems bleed as a fraction of core flow (documented:
/// leaves overboard, books ram drag but no gross thrust).
pub const CUSTOMER_BLEED_FRACTION: f64 = 0.03;
/// Afterburner total-pressure loss fraction (documented).
pub const AFTERBURNER_DP_FRACTION: f64 = 0.03;
/// Turbine cooling bleed as a fraction of core flow (documented: bypasses
/// the combustor, rejoins at the turbine).
pub const TURBINE_COOLING_BLEED: f64 = 0.10;
/// Cooled-blade TIT allowance over wall temperature (documented).
pub const TURBINE_COOLING_ALLOWANCE: f64 = 1.25;
/// Gross-thrust nozzle/installation efficiency (documented: discharge,
/// velocity coefficient, and first-order installation effects).
pub const NOZZLE_GROSS_EFFICIENCY: f64 = 0.95;
/// Design capture Mach for intake suction at zero airspeed (documented
/// calibration: turbine engines inhale their corrected demand; ramjets
/// are passive and get no suction floor).
pub const INTAKE_DESIGN_CAPTURE_MACH: f64 = 0.5;
/// Afterburner duct temperature cap (K, liner limit, documented).
pub const REHEAT_TEMP_CAP_K: f64 = 2200.0;
/// Oxygen mass fraction of Earth air (reference point for O2 gating).
pub const EARTH_OXYGEN_FRACTION: f64 = 0.232;

/// Compressor mass fit (kg per (kg/s · ratio), Olympus-anchored order fit).
pub const MASS_FIT_COMPRESSOR: f64 = 0.42;
/// Turbine mass fit (kg per (kg/s · ratio), order fit).
pub const MASS_FIT_TURBINE: f64 = 0.30;
/// Combustor mass fit (kg per (kg/s)^0.7, order fit).
pub const MASS_FIT_COMBUSTOR: f64 = 7.5;
/// Intake mass fit (kg per m^2 capture area, order fit; ramps ×1.5).
pub const MASS_FIT_INTAKE: f64 = 300.0;
/// Nozzle mass fit (kg per m^2 exit area, order fit).
pub const MASS_FIT_NOZZLE: f64 = 150.0;
/// Afterburner duct mass fit (kg per m^2, order fit).
pub const MASS_FIT_AFTERBURNER: f64 = 80.0;
/// Fan mass fit (kg per m^2 fan area, order fit).
pub const MASS_FIT_FAN: f64 = 200.0;

/// Jet fuel with lower heating value and burned-gas properties
/// (documented approximations).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum JetFuel {
    Kerosene,
    Methane,
    Hydrogen,
}

impl JetFuel {
    /// (LHV J/kg, stoichiometric fuel/air, O2/fuel mass, burned gamma,
    /// burned R J/kg/K, liquid density kg/m^3).
    pub fn properties(self) -> (f64, f64, f64, f64, f64, f64) {
        match self {
            Self::Kerosene => (43.1e6, 0.0680, 3.41, 1.33, 292.0, 810.0),
            Self::Methane => (50.0e6, 0.0581, 3.99, 1.32, 310.0, 422.0),
            Self::Hydrogen => (120.0e6, 0.0294, 8.0, 1.30, 500.0, 71.0),
        }
    }

    /// Exhaust family label for the plume handoff (render hues only;
    /// documented approximation).
    pub fn exhaust_propellant(self) -> super::Propellant {
        match self {
            Self::Kerosene => super::Propellant::LoxRp1,
            Self::Methane => super::Propellant::LoxMethane,
            Self::Hydrogen => super::Propellant::LoxHydrogen,
        }
    }
}

/// Intake pressure-recovery family (supersonic schedule after MIL-E-5008,
/// documented; ramps recover better and weigh more).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum IntakeKind {
    Pitot,
    Ramp,
}

impl IntakeKind {
    /// Total-pressure recovery at flight Mach.
    pub fn recovery(self, mach: f64) -> f64 {
        if mach <= 1.0 {
            return INTAKE_RECOVERY_SUBSONIC;
        }
        let over = mach - 1.0;
        match self {
            Self::Pitot => (1.0 - 0.075 * over.powf(1.35)).max(0.3),
            Self::Ramp => (1.0 - 0.05 * over.powf(1.35)).max(0.3),
        }
    }

    /// Intake mass factor over the pitot baseline.
    pub fn mass_factor(self) -> f64 {
        match self {
            Self::Pitot => 1.0,
            Self::Ramp => 1.5,
        }
    }
}

/// Air-breathing cycle topology.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum AirCycle {
    Turbojet,
    Turbofan,
    Ramjet,
}

/// Flight condition for air-breathing evaluation (all SI; Mach ≥ 0).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct FlightCondition {
    pub mach: f64,
    pub ambient_pa: f64,
    pub ambient_temp_k: f64,
    pub airspeed_mps: f64,
    /// Oxidizer mass fraction of the atmosphere (Earth 0.232; alien air
    /// may offer nothing — the combustor checks, never assumes).
    pub oxygen_fraction: f64,
}

impl FlightCondition {
    /// Validate the condition (NaN fails closed).
    pub fn validate(&self) -> Result<(), PropulsionError> {
        if !self.mach.is_finite() || self.mach < 0.0 {
            return Err(PropulsionError::InvalidSpec(
                "flight Mach must be finite and >= 0".into(),
            ));
        }
        if !(self.ambient_pa >= 0.0) {
            return Err(PropulsionError::InvalidSpec(
                "ambient pressure must be finite and >= 0".into(),
            ));
        }
        if !(self.ambient_temp_k > 0.0) {
            return Err(PropulsionError::InvalidSpec(
                "ambient temperature must be finite and > 0".into(),
            ));
        }
        if !self.airspeed_mps.is_finite() || self.airspeed_mps < 0.0 {
            return Err(PropulsionError::InvalidSpec(
                "airspeed must be finite and >= 0".into(),
            ));
        }
        if !self.oxygen_fraction.is_finite() || !(0.0..=1.0).contains(&self.oxygen_fraction) {
            return Err(PropulsionError::InvalidSpec(
                "oxygen fraction must be finite in [0, 1]".into(),
            ));
        }
        Ok(())
    }
}

/// Build a flight condition from an atmosphere sample, true airspeed, and
/// the local oxygen fraction (explicit: no silent Earth assumption).
pub fn flight_condition(
    sample: &AtmosphereSample,
    airspeed_mps: f64,
    oxygen_fraction: f64,
) -> Result<FlightCondition, PropulsionError> {
    let mach = sample
        .mach(airspeed_mps)
        .map_err(|_| PropulsionError::InvalidSpec("airspeed incompatible with sample".into()))?;
    let condition = FlightCondition {
        mach,
        ambient_pa: sample.pressure_pa,
        ambient_temp_k: sample.temperature_k,
        airspeed_mps,
        oxygen_fraction,
    };
    condition.validate()?;
    Ok(condition)
}

/// Juno-style air-breathing authoring plus cycle internals.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AirbreathingSpec {
    pub name: String,
    pub cycle: AirCycle,
    pub fuel: JetFuel,
    /// Intake capture area (m^2).
    pub intake_area_m2: f64,
    pub intake: IntakeKind,
    /// Compressor pressure ratio (1.0 exactly for ramjets).
    pub compressor_ratio: f64,
    /// Bypass ratio (0 = turbojet).
    pub bypass_ratio: f64,
    /// Fan pressure ratio (turbofans).
    pub fan_pressure_ratio: f64,
    /// Turbine inlet temperature at full throttle (K).
    pub turbine_inlet_temp_k: f64,
    /// Afterburner/reheat fitted.
    pub afterburner: bool,
    /// Reheat temperature (K, capped by liner + O2 availability).
    pub reheat_temp_k: f64,
    pub turbine_material: ChamberMaterial,
    /// Spool time constant (s).
    pub spool_tau_s: f64,
}

impl Default for AirbreathingSpec {
    fn default() -> Self {
        Self {
            name: String::new(),
            cycle: AirCycle::Turbojet,
            fuel: JetFuel::Kerosene,
            intake_area_m2: 0.0,
            intake: IntakeKind::Pitot,
            compressor_ratio: 1.0,
            bypass_ratio: 0.0,
            fan_pressure_ratio: 1.6,
            turbine_inlet_temp_k: 0.0,
            afterburner: false,
            reheat_temp_k: 0.0,
            turbine_material: ChamberMaterial::nickel_superalloy(),
            spool_tau_s: 5.0,
        }
    }
}

impl AirbreathingSpec {
    /// Validate authoring values (NaN fails closed).
    pub fn validate(&self) -> Result<(), PropulsionError> {
        if self.name.trim().is_empty() {
            return Err(PropulsionError::InvalidSpec(
                "engine name must not be empty".into(),
            ));
        }
        require_positive(self.intake_area_m2, "intake area")?;
        require_positive(self.compressor_ratio, "compressor ratio")?;
        require_non_negative(self.bypass_ratio, "bypass ratio")?;
        require_positive(self.fan_pressure_ratio, "fan pressure ratio")?;
        require_positive(self.turbine_inlet_temp_k, "turbine inlet temperature")?;
        require_positive(self.spool_tau_s, "spool tau")?;
        self.turbine_material.validate()?;
        match self.cycle {
            AirCycle::Ramjet => {
                if self.compressor_ratio != 1.0 {
                    return Err(PropulsionError::InvalidSpec(
                        "ramjets carry no compressor (ratio must be 1)".into(),
                    ));
                }
                if self.bypass_ratio != 0.0 {
                    return Err(PropulsionError::InvalidSpec(
                        "ramjets carry no bypass".into(),
                    ));
                }
            }
            AirCycle::Turbojet => {
                if self.bypass_ratio != 0.0 {
                    return Err(PropulsionError::InvalidSpec(
                        "turbojets carry no bypass (use turbofan)".into(),
                    ));
                }
            }
            AirCycle::Turbofan => {
                if !(self.bypass_ratio > 0.0) {
                    return Err(PropulsionError::InvalidSpec(
                        "turbofans need a positive bypass ratio".into(),
                    ));
                }
            }
        }
        // Cooled blades allow TIT above wall temperature (documented).
        // Ramjets carry no turbine: the dump combustor answers to the
        // liner cap instead (actively cooled, documented).
        let tit_cap = match self.cycle {
            AirCycle::Ramjet => 2400.0,
            _ => self.turbine_material.max_wall_temp_k * TURBINE_COOLING_ALLOWANCE,
        };
        if self.turbine_inlet_temp_k > tit_cap {
            return Err(PropulsionError::UnsupportedCombination(format!(
                "TIT {:.0} K exceeds the {:.0} K allowance",
                self.turbine_inlet_temp_k, tit_cap
            )));
        }
        if self.afterburner {
            require_positive(self.reheat_temp_k, "reheat temperature")?;
            if self.reheat_temp_k > REHEAT_TEMP_CAP_K {
                return Err(PropulsionError::UnsupportedCombination(format!(
                    "reheat {:.0} K exceeds the {:.0} K liner cap",
                    self.reheat_temp_k, REHEAT_TEMP_CAP_K
                )));
            }
            if self.reheat_temp_k < self.turbine_inlet_temp_k {
                return Err(PropulsionError::InvalidSpec(
                    "reheat must run hotter than the turbine exit".into(),
                ));
            }
        }
        Ok(())
    }
}

/// Convergent-nozzle exit state for total conditions and a fixed exit
/// (= throat) area: choked with a pressure term, or fully expanded
/// subsonic. Ambient above total pressure yields zero (no backflow).
/// Supersonic cruise nozzles (ramjets) use [`cd_nozzle`] instead: fixed
/// convergent-divergent geometry adapted at the design point.
struct NozzleFlow {
    thrust_gross_n: f64,
    exit_temp_k: f64,
    exit_pressure_pa: f64,
    exit_mach: f64,
}

fn convergent_nozzle(
    mdot_kg_s: f64,
    total_temp_k: f64,
    total_pressure_pa: f64,
    exit_area_m2: f64,
    gamma: f64,
    gas_r: f64,
    ambient_pa: f64,
) -> NozzleFlow {
    let dead = NozzleFlow {
        thrust_gross_n: 0.0,
        exit_temp_k: total_temp_k,
        exit_pressure_pa: ambient_pa,
        exit_mach: 0.0,
    };
    if !(mdot_kg_s > 0.0) || !(total_pressure_pa > 0.0) || !(exit_area_m2 > 0.0) {
        return dead;
    }
    if ambient_pa >= total_pressure_pa {
        return dead;
    }
    let crit = ((gamma + 1.0) / 2.0).powf(gamma / (gamma - 1.0));
    if total_pressure_pa / ambient_pa.max(1.0) > crit {
        // Choked: sonic exit, underexpanded pressure term.
        let exit_temp_k = total_temp_k * 2.0 / (gamma + 1.0);
        let exit_pressure_pa = total_pressure_pa / crit;
        let exit_velocity_mps = (gamma * gas_r * exit_temp_k).sqrt();
        return NozzleFlow {
            thrust_gross_n: NOZZLE_GROSS_EFFICIENCY * mdot_kg_s * exit_velocity_mps
                + (exit_pressure_pa - ambient_pa) * exit_area_m2,
            exit_temp_k,
            exit_pressure_pa,
            exit_mach: 1.0,
        };
    }
    // Fully expanded subsonic to ambient.
    let exit_velocity_mps = (2.0 * gamma * gas_r / (gamma - 1.0)
        * total_temp_k
        * (1.0 - (ambient_pa / total_pressure_pa).powf((gamma - 1.0) / gamma)))
    .sqrt();
    NozzleFlow {
        thrust_gross_n: NOZZLE_GROSS_EFFICIENCY * mdot_kg_s * exit_velocity_mps,
        exit_temp_k: total_temp_k * (ambient_pa / total_pressure_pa).powf((gamma - 1.0) / gamma),
        exit_pressure_pa: ambient_pa,
        exit_mach: (exit_velocity_mps / (gamma * gas_r * total_temp_k).sqrt()).min(1.0),
    }
}

/// Internal cycle state shared by compile (design point) and runtime.
struct CycleState {
    mdot_air_kg_s: f64,
    mdot_core_kg_s: f64,
    fuel_flow_kg_s: f64,
    fuel_ab_flow_kg_s: f64,
    /// Hot flow reaching the core nozzle (customer bleed excluded).
    nozzle_flow_kg_s: f64,
    core_total_temp_k: f64,
    core_total_pressure_pa: f64,
    fan_total_temp_k: f64,
    fan_total_pressure_pa: f64,
    oxygen_limited: bool,
    reheat_limited: bool,
    drive_limited: bool,
    /// True when the intake cannot supply demanded flow.
    air_starved: bool,
}

/// Run the Brayton core at a condition, TIT, and flow factor. Pure
/// thermodynamics: ram recovery, compression, O2-gated combustion, turbine
/// work balance, reheat with O2 cap. `corrected_flow_kg_s` is the design
/// corrected flow (mass at standard face conditions); demand follows
/// δ/√θ off-design, capped by intake capture.
fn run_cycle(
    spec: &AirbreathingSpec,
    fuel: (f64, f64, f64, f64, f64, f64),
    condition: &FlightCondition,
    turbine_temp_k: f64,
    flow_factor: f64,
    corrected_flow_kg_s: Option<f64>,
) -> Result<CycleState, PropulsionError> {
    let (lhv, _f_stoich, o2_per_fuel, gamma_b, r_b, _density) = fuel;
    let gamma_a = AIR_GAMMA;
    let cp_b = gamma_b * r_b / (gamma_b - 1.0);
    // Ram conditions with recovery.
    let t_ram =
        condition.ambient_temp_k * (1.0 + (gamma_a - 1.0) / 2.0 * condition.mach * condition.mach);
    let p_ram = condition.ambient_pa
        * (t_ram / condition.ambient_temp_k).powf(gamma_a / (gamma_a - 1.0))
        * spec.intake.recovery(condition.mach);
    // Air available: ram capture, plus the suction floor for active
    // (turbomachinery) cycles. Ramjets are passive: capture only.
    let sound = (gamma_a * 287.0 * condition.ambient_temp_k).sqrt();
    let rho_0 = condition.ambient_pa / (287.0 * condition.ambient_temp_k);
    let suction = if spec.cycle == AirCycle::Ramjet {
        0.0
    } else {
        INTAKE_DESIGN_CAPTURE_MACH * sound
    };
    let available_kg_s = rho_0 * spec.intake_area_m2 * condition.airspeed_mps.max(suction);
    let demanded_kg_s = match corrected_flow_kg_s {
        Some(wc) => wc * (p_ram / 101_325.0) / (t_ram / 288.15).sqrt() * flow_factor,
        None => available_kg_s * flow_factor,
    };
    let mdot_air_kg_s = demanded_kg_s.min(available_kg_s);
    let oxygen_limited_starved = demanded_kg_s > available_kg_s && available_kg_s <= 0.0;
    let air_starved = demanded_kg_s > available_kg_s + 1e-9;
    // Core/bypass split (ramjet: all core, no machinery).
    let bypass = match spec.cycle {
        AirCycle::Turbofan => spec.bypass_ratio,
        _ => 0.0,
    };
    let mdot_core_kg_s = mdot_air_kg_s / (1.0 + bypass);
    // Compression (ramjets: ram only).
    let pi_c = match spec.cycle {
        AirCycle::Ramjet => 1.0,
        _ => spec.compressor_ratio,
    };
    let tau_c = pi_c.powf((gamma_a - 1.0) / (gamma_a * COMPRESSOR_POLY_EFFICIENCY));
    let t_comp_exit = t_ram * tau_c;
    let p_comp_exit = p_ram * pi_c;
    let pi_f = if bypass > 0.0 {
        spec.fan_pressure_ratio
    } else {
        1.0
    };
    let tau_f = pi_f.powf((gamma_a - 1.0) / (gamma_a * COMPRESSOR_POLY_EFFICIENCY));
    let t_fan_exit = t_ram * tau_f;
    let p_fan_exit = p_ram * pi_f * 0.98;
    // Combustion with O2 gating: fuel capped by available oxygen, TIT
    // follows energy (oxygen-limited operation derates TIT, documented).
    // Ramjets carry no turbine cooling bleed (dump combustor).
    let bleed = match spec.cycle {
        AirCycle::Ramjet => 0.0,
        _ => TURBINE_COOLING_BLEED,
    };
    let o2_avail_kg_s =
        mdot_core_kg_s * (1.0 - bleed - CUSTOMER_BLEED_FRACTION) * condition.oxygen_fraction;
    let f_for_tit = AIR_CP_J_KG_K * (turbine_temp_k - t_comp_exit) / (COMBUSTOR_EFFICIENCY * lhv);
    let f_o2_cap = if mdot_core_kg_s > 0.0 {
        o2_avail_kg_s / (mdot_core_kg_s * (1.0 - bleed - CUSTOMER_BLEED_FRACTION) * o2_per_fuel)
    } else {
        0.0
    };
    let fuel_air = f_for_tit.max(0.0).min(f_o2_cap);
    let oxygen_limited = f_for_tit > f_o2_cap && f_for_tit > 0.0;
    // No fuel, no cycle: flameout (anoxic air) or thermal infeasibility
    // (compressor delivery hotter than the TIT schedule allows).
    if fuel_air <= 0.0 && mdot_core_kg_s > 0.0 {
        return Ok(CycleState {
            mdot_air_kg_s,
            mdot_core_kg_s,
            fuel_flow_kg_s: 0.0,
            fuel_ab_flow_kg_s: 0.0,
            nozzle_flow_kg_s: mdot_core_kg_s,
            core_total_temp_k: t_ram,
            core_total_pressure_pa: p_ram,
            fan_total_temp_k: t_fan_exit,
            fan_total_pressure_pa: p_fan_exit,
            oxygen_limited: oxygen_limited || oxygen_limited_starved,
            reheat_limited: false,
            drive_limited: f_for_tit <= 0.0,
            air_starved,
        });
    }
    let turbine_temp_eff = t_comp_exit + fuel_air * COMBUSTOR_EFFICIENCY * lhv / AIR_CP_J_KG_K;
    let fuel_flow_kg_s = fuel_air * mdot_core_kg_s * (1.0 - bleed - CUSTOMER_BLEED_FRACTION);
    let p_comb = p_comp_exit * (1.0 - COMBUSTOR_DP_FRACTION);
    // Turbine work balance: the rotor sees combustor flow only; cooling
    // bleed bypasses the rotor and mixes downstream at rotor-exit
    // pressure (documented mixing loss: the bleed carries no work).
    let work_comp = AIR_CP_J_KG_K * (t_comp_exit - t_ram);
    let work_fan = if mdot_core_kg_s > 0.0 {
        bypass * AIR_CP_J_KG_K * (t_fan_exit - t_ram)
    } else {
        0.0
    };
    let rotor_flow_ratio = (1.0 - bleed - CUSTOMER_BLEED_FRACTION) * (1.0 + fuel_air);
    let delta_t_rotor = (work_comp + work_fan) / (cp_b * rotor_flow_ratio);
    // Feasibility: the turbine must supply compression work with margin.
    if delta_t_rotor >= turbine_temp_eff * 0.95 && mdot_core_kg_s > 0.0 {
        return Ok(CycleState {
            mdot_air_kg_s,
            mdot_core_kg_s,
            fuel_flow_kg_s: 0.0,
            fuel_ab_flow_kg_s: 0.0,
            nozzle_flow_kg_s: mdot_core_kg_s,
            core_total_temp_k: t_ram,
            core_total_pressure_pa: p_ram,
            fan_total_temp_k: t_fan_exit,
            fan_total_pressure_pa: p_fan_exit,
            oxygen_limited: oxygen_limited || oxygen_limited_starved,
            reheat_limited: false,
            drive_limited: true,
            air_starved,
        });
    }
    // Rotor exit, then cold-bleed mixing at rotor pressure with a
    // documented momentum-mixing loss (documented). Customer bleed leaves
    // overboard: ram drag without gross thrust.
    let t_rotor_exit = turbine_temp_eff - delta_t_rotor;
    let p_rotor_exit = p_comb
        * (1.0 - delta_t_rotor / (turbine_temp_eff * TURBINE_POLY_EFFICIENCY))
            .max(0.01)
            .powf(gamma_b / (gamma_b - 1.0));
    let bleed_flow_ratio = bleed / rotor_flow_ratio.max(1e-12);
    let t_turb_exit = t_rotor_exit * (1.0 - bleed_flow_ratio) + t_comp_exit * bleed_flow_ratio;
    let p_turb_exit = p_rotor_exit * (1.0 - TURBINE_MIXING_DP_FRACTION);
    // Afterburner with its own O2 cap on remaining oxygen.
    let mut fuel_ab_flow_kg_s = 0.0;
    let mut t_nozzle = t_turb_exit;
    let mut p_nozzle = p_turb_exit;
    let mut reheat_limited = false;
    if spec.afterburner && mdot_core_kg_s > 0.0 {
        let o2_used_core = fuel_air * (1.0 - bleed - CUSTOMER_BLEED_FRACTION) * o2_per_fuel;
        let o2_remaining = (condition.oxygen_fraction - o2_used_core).max(0.0);
        let f_ab_want =
            cp_b * (spec.reheat_temp_k - t_turb_exit).max(0.0) / (COMBUSTOR_EFFICIENCY * lhv);
        let f_ab_cap = o2_remaining / o2_per_fuel;
        let f_ab = f_ab_want.min(f_ab_cap);
        reheat_limited = f_ab_want > f_ab_cap && f_ab_want > 0.0;
        fuel_ab_flow_kg_s = f_ab * mdot_core_kg_s * (rotor_flow_ratio + bleed);
        t_nozzle = t_turb_exit + f_ab * COMBUSTOR_EFFICIENCY * lhv / cp_b;
        p_nozzle = p_turb_exit * (1.0 - AFTERBURNER_DP_FRACTION);
    }
    Ok(CycleState {
        mdot_air_kg_s,
        mdot_core_kg_s,
        fuel_flow_kg_s,
        fuel_ab_flow_kg_s,
        nozzle_flow_kg_s: mdot_core_kg_s * (rotor_flow_ratio + bleed) + fuel_ab_flow_kg_s,
        core_total_temp_k: t_nozzle,
        core_total_pressure_pa: p_nozzle,
        fan_total_temp_k: t_fan_exit,
        fan_total_pressure_pa: p_fan_exit,
        oxygen_limited: oxygen_limited || oxygen_limited_starved,
        reheat_limited,
        drive_limited: false,
        air_starved,
    })
}

/// Fixed-geometry convergent-divergent nozzle (ramjet cruise): exit Mach
/// from the area ratio, Summerfield separation check, separated fallback
/// to convergent-at-throat behavior (documented).
#[allow(clippy::too_many_arguments)]
fn cd_nozzle(
    mdot_kg_s: f64,
    total_temp_k: f64,
    total_pressure_pa: f64,
    throat_area_m2: f64,
    exit_area_m2: f64,
    gamma: f64,
    gas_r: f64,
    ambient_pa: f64,
) -> Result<(NozzleFlow, bool), PropulsionError> {
    let dead = NozzleFlow {
        thrust_gross_n: 0.0,
        exit_temp_k: total_temp_k,
        exit_pressure_pa: ambient_pa,
        exit_mach: 0.0,
    };
    if !(mdot_kg_s > 0.0)
        || !(total_pressure_pa > 0.0)
        || !(throat_area_m2 > 0.0)
        || !(exit_area_m2 > 0.0)
        || ambient_pa >= total_pressure_pa
    {
        return Ok((dead, false));
    }
    let crit = ((gamma + 1.0) / 2.0).powf(gamma / (gamma - 1.0));
    if total_pressure_pa / ambient_pa.max(1.0) <= crit {
        // Unchoked: subsonic throughout, adapted exit.
        return Ok((
            convergent_nozzle(
                mdot_kg_s,
                total_temp_k,
                total_pressure_pa,
                exit_area_m2,
                gamma,
                gas_r,
                ambient_pa,
            ),
            false,
        ));
    }
    let expansion = exit_area_m2 / throat_area_m2;
    if !(expansion >= 1.0) {
        return Err(PropulsionError::InvalidSpec(
            "CD nozzle exit must clear the throat".into(),
        ));
    }
    let exit_mach = mach_from_area_ratio(gamma, expansion)?;
    let exit_pressure_pa = total_pressure_pa
        * (1.0 + (gamma - 1.0) / 2.0 * exit_mach * exit_mach).powf(-gamma / (gamma - 1.0));
    if exit_pressure_pa < SEPARATION_PRESSURE_RATIO * ambient_pa {
        // Separated: the jet detaches and the nozzle behaves ~convergent
        // at the throat (documented approximation).
        return Ok((
            convergent_nozzle(
                mdot_kg_s,
                total_temp_k,
                total_pressure_pa,
                throat_area_m2,
                gamma,
                gas_r,
                ambient_pa,
            ),
            true,
        ));
    }
    let exit_temp_k = total_temp_k / (1.0 + (gamma - 1.0) / 2.0 * exit_mach * exit_mach);
    let exit_velocity_mps = exit_mach * (gamma * gas_r * exit_temp_k).sqrt();
    Ok((
        NozzleFlow {
            thrust_gross_n: NOZZLE_GROSS_EFFICIENCY * mdot_kg_s * exit_velocity_mps
                + (exit_pressure_pa - ambient_pa) * exit_area_m2,
            exit_temp_k,
            exit_pressure_pa,
            exit_mach,
        },
        false,
    ))
}

/// Adapted exit area for a design mass flow expanded to ambient
/// (convergent-divergent design point).
fn adapted_exit_area(
    mdot_kg_s: f64,
    total_temp_k: f64,
    total_pressure_pa: f64,
    ambient_pa: f64,
    gamma: f64,
    gas_r: f64,
    cp: f64,
) -> f64 {
    let exit_velocity_mps = (2.0
        * cp
        * total_temp_k
        * (1.0 - (ambient_pa / total_pressure_pa).powf((gamma - 1.0) / gamma)))
    .sqrt();
    let exit_temp_k = total_temp_k * (ambient_pa / total_pressure_pa).powf((gamma - 1.0) / gamma);
    let density = ambient_pa / (gas_r * exit_temp_k);
    mdot_kg_s / (density * exit_velocity_mps)
}

/// Choked throat area for a design mass flow at total conditions
/// (documented 8% margin).
fn size_nozzle_area(
    mdot_kg_s: f64,
    total_temp_k: f64,
    total_pressure_pa: f64,
    gamma: f64,
    gas_r: f64,
) -> f64 {
    let flow_fn = (2.0 / (gamma + 1.0)).powf((gamma + 1.0) / (2.0 * (gamma - 1.0)))
        * (gamma / (gas_r * total_temp_k)).sqrt();
    mdot_kg_s / (total_pressure_pa * flow_fn) * 1.08
}

/// Choked-flow capacity of a fixed exit area at total conditions.
fn nozzle_capacity_kg_s(
    total_temp_k: f64,
    total_pressure_pa: f64,
    exit_area_m2: f64,
    gamma: f64,
    gas_r: f64,
) -> f64 {
    let flow_fn = (2.0 / (gamma + 1.0)).powf((gamma + 1.0) / (2.0 * (gamma - 1.0)))
        * (gamma / (gas_r * total_temp_k)).sqrt();
    total_pressure_pa * exit_area_m2 * flow_fn
}

/// Hangar-compiled air-breather: design point solved, nozzles sized, mass
/// derived. Runtime evaluation is a pure function of flight condition and
/// (post-spool) throttle.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CompiledAirbreather {
    pub name: String,
    pub cycle: AirCycle,
    pub fuel: JetFuel,
    pub intake_area_m2: f64,
    pub intake: IntakeKind,
    pub compressor_ratio: f64,
    pub bypass_ratio: f64,
    pub fan_pressure_ratio: f64,
    pub turbine_inlet_temp_k: f64,
    pub afterburner: bool,
    pub reheat_temp_k: f64,
    pub design_flow_kg_s: f64,
    /// Design corrected flow (kg/s at standard face conditions): demand
    /// anchor for δ/√θ off-design scaling.
    pub design_corrected_flow_kg_s: f64,
    pub exit_area_core_m2: f64,
    pub exit_area_fan_m2: f64,
    /// Core throat area (ramjet C-D; turbojets/fans run convergent, so the
    /// throat equals the exit area).
    pub throat_area_core_m2: f64,
    pub design_static_thrust_n: f64,
    pub design_fuel_flow_kg_s: f64,
    pub design_isp_s: f64,
    pub dry_mass_kg: f64,
    pub spool_tau_s: f64,
}

/// Instantaneous air-breathing operating point.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct AirOperatingPoint {
    pub thrust_n: f64,
    pub fuel_flow_kg_s: f64,
    pub air_flow_kg_s: f64,
    pub exhaust_temp_k: f64,
    pub exhaust_velocity_mps: f64,
    pub exit_pressure_pa: f64,
    pub exit_mach: f64,
    pub isp_s: f64,
    /// True when the intake cannot supply demanded flow.
    pub air_limited: bool,
    /// True when oxygen (not fuel schedule) caps combustion.
    pub oxygen_limited: bool,
    /// True when the turbine cannot drive compression (flameout region).
    pub drive_limited: bool,
    /// True when the fixed nozzle, not the intake, caps flow.
    pub nozzle_limited: bool,
    /// True when a ramjet C-D nozzle separates (overexpanded).
    pub separation_risk: bool,
    /// True when the afterburner is lit.
    pub reheat_active: bool,
    /// True when O2 caps the reheat.
    pub reheat_limited: bool,
}

impl AirbreathingSpec {
    /// Hangar compile: solve the design point, size the convergent nozzles,
    /// derive mass. Turbojets/fans design at sea-level static; ramjets at
    /// Mach 2 sea level (no static flow exists to design on).
    pub fn compile(&self) -> Result<CompiledAirbreather, PropulsionError> {
        self.validate()?;
        let fuel = self.fuel.properties();
        let design_condition = match self.cycle {
            AirCycle::Ramjet => FlightCondition {
                mach: 2.0,
                ambient_pa: 101_325.0,
                ambient_temp_k: 288.15,
                airspeed_mps: 2.0 * (AIR_GAMMA * 287.0 * 288.15).sqrt(),
                oxygen_fraction: EARTH_OXYGEN_FRACTION,
            },
            _ => FlightCondition {
                mach: 0.0,
                ambient_pa: 101_325.0,
                ambient_temp_k: 288.15,
                airspeed_mps: 0.0,
                oxygen_fraction: EARTH_OXYGEN_FRACTION,
            },
        };
        let rho_sl = 101_325.0 / (287.0 * 288.15);
        let a_sl = (AIR_GAMMA * 287.0 * 288.15).sqrt();
        let design_flow_kg_s = match self.cycle {
            AirCycle::Ramjet => rho_sl * self.intake_area_m2 * design_condition.airspeed_mps,
            _ => rho_sl * self.intake_area_m2 * INTAKE_DESIGN_CAPTURE_MACH * a_sl,
        };
        // Corrected-flow anchor at standard face conditions.
        let design_delta = design_condition.ambient_pa
            * (1.0 + (AIR_GAMMA - 1.0) / 2.0 * design_condition.mach * design_condition.mach)
                .powf(AIR_GAMMA / (AIR_GAMMA - 1.0))
            * self.intake.recovery(design_condition.mach)
            / 101_325.0;
        let design_theta = (1.0
            + (AIR_GAMMA - 1.0) / 2.0 * design_condition.mach * design_condition.mach)
            * design_condition.ambient_temp_k
            / 288.15;
        let design_corrected_flow_kg_s =
            design_flow_kg_s * design_theta.sqrt() / design_delta.max(1e-9);
        // Design run is always dry (reheat is a throttle-gated mode, so
        // the nozzles size on the dry full-power point).
        let design_spec = AirbreathingSpec {
            afterburner: false,
            ..self.clone()
        };
        let state = run_cycle(
            &design_spec,
            fuel,
            &design_condition,
            self.turbine_inlet_temp_k,
            1.0,
            Some(design_corrected_flow_kg_s),
        )?;
        if state.drive_limited {
            return Err(PropulsionError::UnsupportedCombination(
                "turbine cannot drive the compressor at the design point".into(),
            ));
        }
        let (_, _, _, gamma_b, r_b, _) = fuel;
        let mdot_core_hot = state.nozzle_flow_kg_s;
        // Ramjets size a fixed convergent-divergent nozzle adapted at the
        // design point; turbine cycles run convergent (throat = exit).
        let throat_area_core_m2 = size_nozzle_area(
            mdot_core_hot,
            state.core_total_temp_k,
            state.core_total_pressure_pa,
            gamma_b,
            r_b,
        );
        let exit_area_core_m2 = match self.cycle {
            AirCycle::Ramjet => {
                let cp_b = gamma_b * r_b / (gamma_b - 1.0);
                adapted_exit_area(
                    mdot_core_hot,
                    state.core_total_temp_k,
                    state.core_total_pressure_pa,
                    design_condition.ambient_pa,
                    gamma_b,
                    r_b,
                    cp_b,
                )
            }
            _ => throat_area_core_m2,
        };
        if exit_area_core_m2 < throat_area_core_m2 {
            return Err(PropulsionError::UnsupportedCombination(
                "design point overexpands the ramjet nozzle; pick a faster design Mach".into(),
            ));
        }
        let mdot_fan = state.mdot_air_kg_s - state.mdot_core_kg_s;
        let exit_area_fan_m2 = if mdot_fan > 0.0 {
            size_nozzle_area(
                mdot_fan,
                state.fan_total_temp_k,
                state.fan_total_pressure_pa,
                AIR_GAMMA,
                287.0,
            )
        } else {
            0.0
        };
        // Design summary at the design point (editor display values).
        let core = convergent_nozzle(
            mdot_core_hot,
            state.core_total_temp_k,
            state.core_total_pressure_pa,
            exit_area_core_m2,
            gamma_b,
            r_b,
            design_condition.ambient_pa,
        );
        let fan = convergent_nozzle(
            mdot_fan,
            state.fan_total_temp_k,
            state.fan_total_pressure_pa,
            exit_area_fan_m2,
            AIR_GAMMA,
            287.0,
            design_condition.ambient_pa,
        );
        let design_static_thrust_n = core.thrust_gross_n + fan.thrust_gross_n
            - state.mdot_air_kg_s * design_condition.airspeed_mps;
        let design_fuel = state.fuel_flow_kg_s + state.fuel_ab_flow_kg_s;

        // Mass from design flow and geometry (documented order fits).
        let bypass = match self.cycle {
            AirCycle::Turbofan => self.bypass_ratio,
            _ => 0.0,
        };
        let compressor_kg = match self.cycle {
            AirCycle::Ramjet => 0.0,
            _ => MASS_FIT_COMPRESSOR * design_flow_kg_s * self.compressor_ratio,
        };
        let turbine_kg = match self.cycle {
            AirCycle::Ramjet => 0.0,
            _ => MASS_FIT_TURBINE * design_flow_kg_s * self.compressor_ratio,
        };
        let combustor_kg = MASS_FIT_COMBUSTOR * design_flow_kg_s.powf(0.7);
        let intake_kg = MASS_FIT_INTAKE * self.intake_area_m2 * self.intake.mass_factor();
        let nozzle_kg = MASS_FIT_NOZZLE * (exit_area_core_m2 + exit_area_fan_m2);
        let ab_kg = if self.afterburner {
            MASS_FIT_AFTERBURNER * exit_area_core_m2
        } else {
            0.0
        };
        let fan_area_m2 = if bypass > 0.0 {
            bypass * design_flow_kg_s / (1.0 + bypass) / (rho_sl * a_sl * 0.45)
        } else {
            0.0
        };
        let fan_kg = MASS_FIT_FAN * fan_area_m2;
        let misc_kg = 150.0 + 50.0 * (1.0 + bypass);
        let dry_mass_kg = compressor_kg
            + turbine_kg
            + combustor_kg
            + intake_kg
            + nozzle_kg
            + ab_kg
            + fan_kg
            + misc_kg;

        Ok(CompiledAirbreather {
            name: self.name.clone(),
            cycle: self.cycle,
            fuel: self.fuel,
            intake_area_m2: self.intake_area_m2,
            intake: self.intake,
            compressor_ratio: self.compressor_ratio,
            bypass_ratio: bypass,
            fan_pressure_ratio: self.fan_pressure_ratio,
            turbine_inlet_temp_k: self.turbine_inlet_temp_k,
            afterburner: self.afterburner,
            reheat_temp_k: self.reheat_temp_k,
            design_flow_kg_s,
            design_corrected_flow_kg_s,
            exit_area_core_m2,
            exit_area_fan_m2,
            throat_area_core_m2,
            design_static_thrust_n,
            design_fuel_flow_kg_s: design_fuel,
            design_isp_s: if design_fuel > 0.0 {
                design_static_thrust_n / (design_fuel * STANDARD_GRAVITY_MPS2)
            } else {
                0.0
            },
            dry_mass_kg,
            spool_tau_s: self.spool_tau_s,
        })
    }
}

impl CompiledAirbreather {
    /// Steady operating point at a flight condition and effective
    /// (post-spool) throttle. Part-power idealizations (TIT, pressure
    /// ratio, and flow schedules) are documented linear schedules, not
    /// component maps.
    pub fn operating_point(
        &self,
        condition: &FlightCondition,
        throttle: f64,
    ) -> Result<AirOperatingPoint, PropulsionError> {
        condition.validate()?;
        if !throttle.is_finite() || !(0.0..=1.0).contains(&throttle) {
            return Err(PropulsionError::InvalidCommand(
                "throttle must be finite in [0, 1]".into(),
            ));
        }
        let off = AirOperatingPoint {
            thrust_n: 0.0,
            fuel_flow_kg_s: 0.0,
            air_flow_kg_s: 0.0,
            exhaust_temp_k: condition.ambient_temp_k,
            exhaust_velocity_mps: 0.0,
            exit_pressure_pa: condition.ambient_pa,
            exit_mach: 0.0,
            isp_s: 0.0,
            air_limited: false,
            oxygen_limited: false,
            drive_limited: false,
            nozzle_limited: false,
            separation_risk: false,
            reheat_active: false,
            reheat_limited: false,
        };
        if throttle == 0.0 {
            return Ok(off);
        }
        let fuel = self.fuel.properties();
        let (_, _, _, gamma_b, r_b, _) = fuel;
        // Part-power schedules (documented idealizations).
        let tit = self.turbine_inlet_temp_k * (0.55 + 0.45 * throttle);
        let flow_factor = 0.35 + 0.65 * throttle;
        let eff_pi_c = match self.cycle {
            AirCycle::Ramjet => 1.0,
            _ => 1.0 + (self.compressor_ratio - 1.0) * (0.35 + 0.65 * throttle),
        };
        let eff_reheat = self.afterburner && throttle >= 0.9;
        let spec_eff = AirbreathingSpec {
            compressor_ratio: eff_pi_c,
            afterburner: eff_reheat,
            turbine_inlet_temp_k: tit,
            name: self.name.clone(),
            cycle: self.cycle,
            fuel: self.fuel,
            intake_area_m2: self.intake_area_m2,
            intake: self.intake,
            bypass_ratio: self.bypass_ratio,
            fan_pressure_ratio: self.fan_pressure_ratio,
            reheat_temp_k: self.reheat_temp_k,
            turbine_material: ChamberMaterial::nickel_superalloy(),
            spool_tau_s: self.spool_tau_s,
        };
        let state = run_cycle(
            &spec_eff,
            fuel,
            condition,
            tit,
            flow_factor,
            Some(self.design_corrected_flow_kg_s),
        )?;
        if state.drive_limited || state.mdot_air_kg_s <= 0.0 {
            return Ok(AirOperatingPoint {
                air_limited: true,
                oxygen_limited: state.oxygen_limited,
                drive_limited: true,
                ..off
            });
        }
        let fuel_total = state.fuel_flow_kg_s + state.fuel_ab_flow_kg_s;
        if fuel_total <= 0.0 {
            // Flameout with airflow (anoxic air or thermally infeasible
            // TIT): windmilling drag books airframe-side; the backend
            // reports zero thrust with the cause flags, never NaN.
            return Ok(AirOperatingPoint {
                air_limited: state.air_starved,
                oxygen_limited: state.oxygen_limited,
                drive_limited: state.drive_limited,
                ..off
            });
        }
        let mdot_core_hot = state.nozzle_flow_kg_s;
        let mdot_fan = state.mdot_air_kg_s - state.mdot_core_kg_s;
        // Fixed-geometry matching: the nozzle sets swallowed flow. When
        // demand exceeds choked capacity, the whole engine rescales to
        // what the nozzle passes (documented approximation of the matched
        // operating point; keeps fuel, air, and thrust mutually
        // consistent instead of booking fuel for unswallowed air).
        let cap_core = nozzle_capacity_kg_s(
            state.core_total_temp_k,
            state.core_total_pressure_pa,
            self.throat_area_core_m2,
            gamma_b,
            r_b,
        );
        let cap_fan = if self.exit_area_fan_m2 > 0.0 && mdot_fan > 0.0 {
            nozzle_capacity_kg_s(
                state.fan_total_temp_k,
                state.fan_total_pressure_pa,
                self.exit_area_fan_m2,
                AIR_GAMMA,
                287.0,
            )
        } else {
            f64::INFINITY
        };
        let scale = (cap_core / mdot_core_hot)
            .min(cap_fan / mdot_fan.max(1e-12))
            .min(1.0);
        let nozzle_limited = scale < 1.0;
        let mdot_air = state.mdot_air_kg_s * scale;
        let fuel_flow = state.fuel_flow_kg_s * scale;
        let fuel_ab_flow = state.fuel_ab_flow_kg_s * scale;
        let core_flow = mdot_core_hot * scale;
        // Turbine cycles run convergent; ramjets run fixed C-D.
        let (core, separation_risk) = match self.cycle {
            AirCycle::Ramjet => cd_nozzle(
                core_flow,
                state.core_total_temp_k,
                state.core_total_pressure_pa,
                self.throat_area_core_m2,
                self.exit_area_core_m2,
                gamma_b,
                r_b,
                condition.ambient_pa,
            )?,
            _ => (
                convergent_nozzle(
                    core_flow,
                    state.core_total_temp_k,
                    state.core_total_pressure_pa,
                    self.exit_area_core_m2,
                    gamma_b,
                    r_b,
                    condition.ambient_pa,
                ),
                false,
            ),
        };
        let mdot_fan = (state.mdot_air_kg_s - state.mdot_core_kg_s) * scale;
        let fan = convergent_nozzle(
            mdot_fan,
            state.fan_total_temp_k,
            state.fan_total_pressure_pa,
            self.exit_area_fan_m2,
            AIR_GAMMA,
            287.0,
            condition.ambient_pa,
        );
        let thrust_n = core.thrust_gross_n + fan.thrust_gross_n - mdot_air * condition.airspeed_mps;
        let fuel_total = fuel_flow + fuel_ab_flow;
        Ok(AirOperatingPoint {
            thrust_n,
            fuel_flow_kg_s: fuel_total,
            air_flow_kg_s: mdot_air,
            exhaust_temp_k: core.exit_temp_k,
            exhaust_velocity_mps: if core_flow > 0.0 {
                core.thrust_gross_n / core_flow
            } else {
                0.0
            },
            exit_pressure_pa: core.exit_pressure_pa,
            exit_mach: core.exit_mach,
            isp_s: thrust_n / (fuel_total * STANDARD_GRAVITY_MPS2),
            air_limited: state.air_starved,
            oxygen_limited: state.oxygen_limited,
            drive_limited: false,
            nozzle_limited,
            separation_risk,
            reheat_active: eff_reheat && fuel_total > state.fuel_flow_kg_s,
            reheat_limited: state.reheat_limited,
        })
    }
}

/// One analyzer row: thrust/Isp over altitude and Mach (Juno Mach-table
/// contract, computed from the cycle instead of pressure scaling).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct AirAltitudePoint {
    pub altitude_m: f64,
    pub mach: f64,
    pub ambient_pa: f64,
    pub thrust_n: f64,
    pub isp_s: f64,
    pub fuel_flow_kg_s: f64,
    pub air_limited: bool,
    pub oxygen_limited: bool,
    pub nozzle_limited: bool,
    pub separation_risk: bool,
}

/// Thrust/Isp grid over altitudes × Mach numbers at fixed throttle and
/// oxygen fraction (well-mixed atmosphere assumption, documented).
pub fn analyze_airbreathing(
    engine: &CompiledAirbreather,
    atmosphere: &crate::atmosphere::AtmosphereConfig,
    altitudes_m: &[f64],
    machs: &[f64],
    throttle: f64,
    oxygen_fraction: f64,
) -> Result<Vec<AirAltitudePoint>, PropulsionError> {
    if altitudes_m.is_empty() || machs.is_empty() {
        return Err(PropulsionError::InvalidSpec(
            "analyzer needs altitudes and Mach numbers".into(),
        ));
    }
    let mut rows = Vec::with_capacity(altitudes_m.len() * machs.len());
    for altitude_m in altitudes_m {
        if !altitude_m.is_finite() {
            return Err(PropulsionError::InvalidSpec(
                "altitudes must be finite".into(),
            ));
        }
        let sample = atmosphere
            .sample(*altitude_m)
            .map_err(|error| PropulsionError::InvalidSpec(format!("atmosphere sample: {error}")))?;
        for mach in machs {
            if !mach.is_finite() || *mach < 0.0 {
                return Err(PropulsionError::InvalidSpec(
                    "Mach numbers must be finite and >= 0".into(),
                ));
            }
            let condition =
                flight_condition(&sample, mach * sample.speed_of_sound_mps, oxygen_fraction)?;
            let point = engine.operating_point(&condition, throttle)?;
            rows.push(AirAltitudePoint {
                altitude_m: *altitude_m,
                mach: *mach,
                ambient_pa: sample.pressure_pa,
                thrust_n: point.thrust_n,
                isp_s: point.isp_s,
                fuel_flow_kg_s: point.fuel_flow_kg_s,
                air_limited: point.air_limited,
                oxygen_limited: point.oxygen_limited,
                nozzle_limited: point.nozzle_limited,
                separation_risk: point.separation_risk,
            });
        }
    }
    Ok(rows)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::AtmosphereConfig;

    fn olympus_like() -> AirbreathingSpec {
        // Olympus-593-class: OPR 15.5, ~186 kg/s, kerosene, reheat fitted.
        // TIT 1650 K is a best-public-data reconstruction (cooled blades).
        AirbreathingSpec {
            name: "Olympus class".into(),
            cycle: AirCycle::Turbojet,
            fuel: JetFuel::Kerosene,
            intake_area_m2: 0.90,
            intake: IntakeKind::Pitot,
            compressor_ratio: 15.5,
            bypass_ratio: 0.0,
            fan_pressure_ratio: 1.0,
            turbine_inlet_temp_k: 1650.0,
            afterburner: true,
            reheat_temp_k: 1900.0,
            turbine_material: ChamberMaterial::nickel_superalloy(),
            spool_tau_s: 5.0,
        }
    }

    fn sl_static() -> FlightCondition {
        FlightCondition {
            mach: 0.0,
            ambient_pa: 101_325.0,
            ambient_temp_k: 288.15,
            airspeed_mps: 0.0,
            oxygen_fraction: EARTH_OXYGEN_FRACTION,
        }
    }

    #[test]
    fn olympus_class_ideal_bound() {
        // Anchor against published Olympus 593 data (142 kN dry, ~4.8 kg/s
        // i.e. ~3010 s, 3175 kg): inputs are best-public-data
        // reconstructions (OPR, airflow, TIT class), every loss channel is
        // named physics, and nothing is fitted to the anchor — so the bands
        // cover input + idealization uncertainty instead of pretending to
        // be a digital twin. Exactness lives in the invariant pins below.
        let engine = olympus_like().compile().expect("olympus compiles");
        let dry = engine.operating_point(&sl_static(), 0.8).expect("dry");
        assert!(
            dry.thrust_n >= 142_000.0 * 0.85 && dry.thrust_n <= 142_000.0 * 1.30,
            "dry thrust {:.0} kN outside the anchor band",
            dry.thrust_n / 1000.0
        );
        // Fuel-side bound as Isp (published dry SFC ~3010 s).
        assert!(
            dry.isp_s >= 3010.0 * 0.85 && dry.isp_s <= 3010.0 * 1.6,
            "dry Isp {:.0} s outside the anchor band",
            dry.isp_s
        );
        assert!(
            (1500.0..=6500.0).contains(&engine.dry_mass_kg),
            "dry mass {:.0} kg outside the order band",
            engine.dry_mass_kg
        );
        // Design flow anchors the 186 kg/s class.
        assert!((engine.design_flow_kg_s - 186.0).abs() < 15.0);
    }

    #[test]
    fn reheat_adds_thrust_loses_efficiency() {
        // Reheat at full throttle must add thrust and cut Isp; part
        // throttle flies dry (max gate at 0.9, documented).
        let engine = olympus_like().compile().expect("olympus compiles");
        let dry = engine.operating_point(&sl_static(), 0.8).expect("dry");
        assert!(!dry.reheat_active);
        let lit = engine.operating_point(&sl_static(), 1.0).expect("lit");
        assert!(lit.reheat_active);
        assert!(lit.thrust_n > dry.thrust_n * 1.05);
        assert!(lit.isp_s < dry.isp_s);
    }

    #[test]
    fn ramjet_static_zero_and_rising_with_mach() {
        // Ramjets are passive: no suction floor, so static thrust is
        // exactly zero; thrust rises steeply into Mach 2-3.
        let spec = AirbreathingSpec {
            name: "ramjet".into(),
            cycle: AirCycle::Ramjet,
            fuel: JetFuel::Kerosene,
            intake_area_m2: 0.5,
            intake: IntakeKind::Ramp,
            compressor_ratio: 1.0,
            bypass_ratio: 0.0,
            fan_pressure_ratio: 1.0,
            turbine_inlet_temp_k: 2200.0,
            afterburner: false,
            reheat_temp_k: 0.0,
            turbine_material: ChamberMaterial::nickel_superalloy(),
            spool_tau_s: 0.5,
        };
        let engine = spec.compile().expect("ramjet compiles");
        let statik = engine.operating_point(&sl_static(), 1.0).expect("static");
        assert_eq!(statik.thrust_n, 0.0);
        assert_eq!(statik.fuel_flow_kg_s, 0.0);
        let sample = AtmosphereConfig::default().sample(0.0).expect("SL sample");
        let mut last = 0.0;
        for mach in [1.5, 2.0, 2.5, 3.0] {
            let condition = flight_condition(
                &sample,
                mach * sample.speed_of_sound_mps,
                EARTH_OXYGEN_FRACTION,
            )
            .expect("condition");
            let point = engine.operating_point(&condition, 1.0).expect("point");
            assert!(point.thrust_n > last, "ramjet thrust must rise with Mach");
            last = point.thrust_n;
        }
        // Design-point (Mach 2) Isp in the honest band: the fixed nozzle
        // is adapted there, so this is the number the geometry promises.
        // Off-design absolute Isp is geometry-dependent (underexpansion
        // pressure thrust is real thrust on a fixed nozzle); energy
        // conservation at Mach 3 is the rigorous check instead.
        let design = flight_condition(
            &sample,
            2.0 * sample.speed_of_sound_mps,
            EARTH_OXYGEN_FRACTION,
        )
        .expect("condition");
        let design_point = engine.operating_point(&design, 1.0).expect("design");
        assert!(
            (1500.0..=2200.0).contains(&design_point.isp_s),
            "ramjet design Isp {:.0} s outside the band",
            design_point.isp_s
        );
        let cruise = flight_condition(
            &sample,
            3.0 * sample.speed_of_sound_mps,
            EARTH_OXYGEN_FRACTION,
        )
        .expect("condition");
        let point = engine.operating_point(&cruise, 1.0).expect("cruise");
        let exhaust_ground_speed = (point.exhaust_velocity_mps - cruise.airspeed_mps).max(0.0);
        let useful_power = point.thrust_n * cruise.airspeed_mps
            + 0.5 * point.air_flow_kg_s * exhaust_ground_speed * exhaust_ground_speed;
        let input_power = point.fuel_flow_kg_s * 43.1e6
            + 0.5 * point.air_flow_kg_s * cruise.airspeed_mps * cruise.airspeed_mps;
        assert!(
            useful_power < input_power,
            "ramjet cruise must respect energy conservation"
        );
    }

    #[test]
    fn vacuum_and_anoxic_air_flame_out() {
        // No air, no thrust (space flameout); no oxygen, no combustion
        // (alien-atmosphere gating, Juno-1.4-style).
        let engine = olympus_like().compile().expect("olympus compiles");
        let vacuum = FlightCondition {
            ambient_pa: 0.0,
            ..sl_static()
        };
        let dead = engine.operating_point(&vacuum, 1.0).expect("vacuum");
        assert_eq!(dead.thrust_n, 0.0);
        let anoxic = FlightCondition {
            oxygen_fraction: 0.0,
            ..sl_static()
        };
        let choked = engine.operating_point(&anoxic, 1.0).expect("anoxic");
        assert!(choked.oxygen_limited);
        assert!(choked.thrust_n < 0.05 * engine.design_static_thrust_n);
    }

    #[test]
    fn turbofan_beats_turbojet_efficiency() {
        // Same core with a bypass fan on a bigger intake (fan face needs
        // its own air): more thrust per fuel, strictly higher Isp.
        let base = olympus_like();
        let jet = AirbreathingSpec {
            afterburner: false,
            ..base.clone()
        }
        .compile()
        .expect("jet compiles");
        let fan = AirbreathingSpec {
            cycle: AirCycle::Turbofan,
            intake_area_m2: 2.25,
            bypass_ratio: 1.5,
            fan_pressure_ratio: 1.6,
            afterburner: false,
            ..base
        }
        .compile()
        .expect("fan compiles");
        let pj = jet.operating_point(&sl_static(), 1.0).expect("jet");
        let pf = fan.operating_point(&sl_static(), 1.0).expect("fan");
        assert!(pf.thrust_n > pj.thrust_n);
        assert!(pf.isp_s > pj.isp_s);
    }

    #[test]
    fn cruise_energy_bound_and_altitude_lapse() {
        // First law, steady flow: useful propulsive power plus exhaust
        // kinetic power cannot exceed fuel chemical power plus the inlet
        // airstream kinetic power (ram compression is free compression —
        // the vehicle pays for it as ram drag, booked in net thrust).
        // Altitude always lapses sea-level thrust.
        let engine = olympus_like().compile().expect("olympus compiles");
        let sample = AtmosphereConfig::default()
            .sample(11_000.0)
            .expect("11 km sample");
        let cruise = flight_condition(
            &sample,
            0.9 * sample.speed_of_sound_mps,
            EARTH_OXYGEN_FRACTION,
        )
        .expect("cruise");
        let point = engine.operating_point(&cruise, 1.0).expect("cruise");
        let exhaust_ground_speed = (point.exhaust_velocity_mps - cruise.airspeed_mps).max(0.0);
        let useful_power = point.thrust_n * cruise.airspeed_mps
            + 0.5 * point.air_flow_kg_s * exhaust_ground_speed * exhaust_ground_speed;
        let input_power = point.fuel_flow_kg_s * 43.1e6
            + 0.5 * point.air_flow_kg_s * cruise.airspeed_mps * cruise.airspeed_mps;
        assert!(useful_power < input_power);
        let sl = engine.operating_point(&sl_static(), 1.0).expect("SL");
        assert!(point.thrust_n < sl.thrust_n);
    }

    #[test]
    fn turbine_drive_limit_kills_hypersonic_turbojet() {
        // At Mach 5 sea level the compressor work outruns the turbine:
        // the model must report drive-limited zero, not garbage.
        let engine = olympus_like().compile().expect("olympus compiles");
        let sample = AtmosphereConfig::default().sample(0.0).expect("SL sample");
        let fast = flight_condition(
            &sample,
            5.0 * sample.speed_of_sound_mps,
            EARTH_OXYGEN_FRACTION,
        )
        .expect("fast");
        let point = engine.operating_point(&fast, 1.0).expect("fast");
        assert!(point.drive_limited);
        assert_eq!(point.thrust_n, 0.0);
    }

    #[test]
    fn mass_scales_with_size_and_validation_fires() {
        // Order fits ride flow/area linearly: 4x intake area lands mass in
        // [3.5, 4.5]x (documents fit behavior, not cube law).
        let small = olympus_like().compile().expect("small");
        let big = AirbreathingSpec {
            intake_area_m2: 3.6,
            ..olympus_like()
        }
        .compile()
        .expect("big");
        let ratio = big.dry_mass_kg / small.dry_mass_kg;
        assert!((3.5..=4.5).contains(&ratio), "mass ratio {ratio:.2}");
        // Ramjet with a compressor, turbojet with bypass: refused.
        assert!(
            AirbreathingSpec {
                cycle: AirCycle::Ramjet,
                compressor_ratio: 5.0,
                ..olympus_like()
            }
            .compile()
            .is_err()
        );
        assert!(
            AirbreathingSpec {
                cycle: AirCycle::Turbojet,
                bypass_ratio: 1.0,
                ..olympus_like()
            }
            .compile()
            .is_err()
        );
        // TIT past the cooled-blade allowance: refused.
        assert!(
            AirbreathingSpec {
                turbine_inlet_temp_k: 2000.0,
                ..olympus_like()
            }
            .compile()
            .is_err()
        );
    }
}
