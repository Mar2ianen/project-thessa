//! Air-breathing jet engines: turbojet, turbofan, ramjet, and scramjet from one
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

use crate::atmosphere::{AtmosphereComposition, AtmosphereSample, GasKind};

use super::SEPARATION_PRESSURE_RATIO;
use super::shaft::{SHAFT_FRICTION_FRACTION, ShaftBalance, ShaftSpec, TURBINE_SHAFT_HEAT_FRACTION};
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
pub(super) const COMPRESSOR_POLY_EFFICIENCY: f64 = 0.88;
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
/// Design capture Mach for intake suction at zero vehicle speed
/// (documented calibration: turbine engines inhale their corrected
/// demand). The floor scales with actual spool speed — a stopped
/// compressor draws nothing (section 8.1: no free intake flow without
/// a shaft source) — and ramjet/scramjet cycles are passive, so they get no
/// floor.
pub const INTAKE_DESIGN_CAPTURE_MACH: f64 = 0.5;
/// Afterburner duct temperature cap (K, liner limit, documented).
pub const REHEAT_TEMP_CAP_K: f64 = 2200.0;

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
    Scramjet,
}

impl AirCycle {
    /// Passive inlet/combustor/nozzle cycles with no compressor or turbine
    /// shaft state.
    pub const fn has_shaft(self) -> bool {
        !matches!(self, Self::Ramjet | Self::Scramjet)
    }

    const fn is_passive(self) -> bool {
        !self.has_shaft()
    }
}

/// Flight condition for air-breathing evaluation (all SI; Mach ≥ 0).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct FlightCondition {
    pub mach: f64,
    pub ambient_pa: f64,
    pub ambient_temp_k: f64,
    pub airspeed_mps: f64,
    /// Well-mixed species basis sampled from the atmosphere (section 10):
    /// the combustor queries oxidizer availability from it and checks,
    /// never assumes — alien air may offer nothing.
    pub composition: AtmosphereComposition,
}

impl FlightCondition {
    /// Ambient mass density from the sampled species basis:
    /// ρ = p M̄ / (R_u T) (composition-aware, section 10).
    pub fn density_kg_m3(&self) -> f64 {
        let gas_constant = self.composition.gas_constant_j_kg_k();
        if !gas_constant.is_finite()
            || gas_constant <= 0.0
            || !self.ambient_temp_k.is_finite()
            || self.ambient_temp_k <= 0.0
            || !self.ambient_pa.is_finite()
            || self.ambient_pa < 0.0
        {
            return 0.0;
        }
        self.ambient_pa / (gas_constant * self.ambient_temp_k)
    }

    /// Ambient speed of sound from the sampled species basis:
    /// a = sqrt(γ_mix R_mix T) (composition-aware, section 10).
    pub fn speed_of_sound_mps(&self) -> f64 {
        let gas_constant = self.composition.gas_constant_j_kg_k();
        let gamma = self.composition.mean_heat_capacity_ratio();
        if !gas_constant.is_finite()
            || gas_constant <= 0.0
            || !gamma.is_finite()
            || gamma <= 1.0
            || !self.ambient_temp_k.is_finite()
            || self.ambient_temp_k <= 0.0
        {
            return 0.0;
        }
        (gamma * gas_constant * self.ambient_temp_k).sqrt()
    }

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
        if !self.composition.is_sane() {
            return Err(PropulsionError::InvalidSpec(
                "flight-condition composition must be sane (finite, non-negative, positive total)"
                    .into(),
            ));
        }
        Ok(())
    }
}

/// Build a flight condition from an atmosphere sample and true airspeed.
/// Species availability comes from the sample itself (section 10): the
/// authoritative atmosphere decides, never a caller-supplied scalar.
pub fn flight_condition(
    sample: &AtmosphereSample,
    airspeed_mps: f64,
) -> Result<FlightCondition, PropulsionError> {
    let mach = sample
        .mach(airspeed_mps)
        .map_err(|_| PropulsionError::InvalidSpec("airspeed incompatible with sample".into()))?;
    let condition = FlightCondition {
        mach,
        ambient_pa: sample.pressure_pa,
        ambient_temp_k: sample.temperature_k,
        airspeed_mps,
        composition: sample.composition,
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
    /// Compressor pressure ratio (1.0 exactly for ramjets/scramjets).
    pub compressor_ratio: f64,
    /// Bypass ratio (0 = turbojet).
    pub bypass_ratio: f64,
    /// Fan pressure ratio (turbofans).
    pub fan_pressure_ratio: f64,
    /// Turbine-inlet target or passive combustor-exit total-temperature
    /// target at full throttle (K), depending on `cycle`.
    pub turbine_inlet_temp_k: f64,
    /// Afterburner/reheat fitted.
    pub afterburner: bool,
    /// Reheat temperature (K, capped by liner + O2 availability).
    pub reheat_temp_k: f64,
    pub turbine_material: ChamberMaterial,
    /// Spool time constant (s).
    pub spool_tau_s: f64,
    /// Shaft topology: starter, generator, light-off/self-sustain
    /// thresholds (section 8.1). Inert default = deliberate starterless
    /// windmill-only design.
    #[serde(default)]
    pub shaft: ShaftSpec,
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
            shaft: ShaftSpec::default(),
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
        self.shaft.validate(self.cycle)?;
        match self.cycle {
            AirCycle::Ramjet | AirCycle::Scramjet => {
                if self.compressor_ratio != 1.0 {
                    return Err(PropulsionError::InvalidSpec(
                        "ramjets and scramjets carry no compressor (ratio must be 1)".into(),
                    ));
                }
                if self.bypass_ratio != 0.0 {
                    return Err(PropulsionError::InvalidSpec(
                        "ramjets and scramjets carry no bypass".into(),
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
        // Passive ramjet/scramjet combustors carry no turbine and use the
        // documented dump-combustor liner cap.
        let tit_cap = match self.cycle {
            AirCycle::Ramjet | AirCycle::Scramjet => 2400.0,
            _ => self.turbine_material.max_wall_temp_k * TURBINE_COOLING_ALLOWANCE,
        };
        if self.turbine_inlet_temp_k > tit_cap {
            return Err(PropulsionError::UnsupportedCombination(format!(
                "TIT {:.0} K exceeds the {:.0} K allowance",
                self.turbine_inlet_temp_k, tit_cap
            )));
        }
        if self.afterburner {
            if self.cycle == AirCycle::Scramjet {
                return Err(PropulsionError::UnsupportedCombination(
                    "scramjet combustor does not use a separate afterburner duct".into(),
                ));
            }
            require_positive(self.reheat_temp_k, "reheat temperature")?;
            if self.reheat_temp_k > REHEAT_TEMP_CAP_K {
                return Err(PropulsionError::UnsupportedCombination(format!(
                    "reheat {:.0} K exceeds the {:.0} K liner cap",
                    self.reheat_temp_k, REHEAT_TEMP_CAP_K
                )));
            }
            // The reheat-vs-turbine-exit check needs the solved design
            // state, so it lives in `compile`, not here.
        }
        Ok(())
    }
}

/// Convergent-nozzle exit state for total conditions and a fixed exit
/// (= throat) area: choked with a pressure term, or fully expanded
/// subsonic. Ambient above total pressure yields zero (no backflow).
/// Passive supersonic nozzles (ramjets/scramjets) use [`cd_nozzle`] instead: fixed
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
    boost_fuel_flow_kg_s: f64,
    fuel_ab_flow_kg_s: f64,
    /// Hot flow reaching the core nozzle (customer bleed excluded).
    nozzle_flow_kg_s: f64,
    core_total_temp_k: f64,
    core_total_pressure_pa: f64,
    fan_total_temp_k: f64,
    fan_total_pressure_pa: f64,
    /// Compressor + fan power the shaft must supply at this spool speed
    /// (W): the shaft-side demand of the evaluated state.
    shaft_demand_w: f64,
    combustor_heat_release_w: f64,
    core_gamma: f64,
    core_gas_constant_j_kg_k: f64,
    compressor_inlet_total_temp_k: f64,
    precooler_heat_flow_w: f64,
    precooler_wall_heat_flow_w: f64,
    /// Maximum downstream power-turbine output at this gas state (W), after
    /// the authored fuel-heat budget and the positive exhaust-enthalpy bound.
    power_takeoff_capacity_w: f64,
    oxygen_limited: bool,
    reheat_limited: bool,
    drive_limited: bool,
    combustion_thermal_limited: bool,
    /// True when the intake cannot supply demanded flow.
    air_starved: bool,
    /// Scramjet fuel is inhibited at/below the sonic combustor boundary.
    scramjet_limited: bool,
}

#[derive(Debug, Clone, Copy)]
pub(super) struct AirCycleConditioning {
    pub compressor_inlet_total_temp_k: f64,
    pub compressor_pressure_recovery: f64,
    pub boost_fuel: JetFuel,
    pub boost_fuel_flow_kg_s: f64,
    /// Sensible heat picked up by boost fuel and returned to the combustor.
    pub coolant_heat_flow_w: f64,
    pub wall_heat_flow_w: f64,
}

#[derive(Debug, Clone, Copy)]
struct CycleDriveInput {
    spool_n: f64,
    ignition: bool,
    corrected_flow_kg_s: Option<f64>,
    power_takeoff_w: f64,
    power_takeoff_heat_fraction: f64,
    conditioning: Option<AirCycleConditioning>,
}

/// Run the Brayton core at a condition, TIT, spool speed, and ignition
/// command. Pure thermodynamics: ram recovery, spool-scheduled
/// compression (head ~ spool^2, corrected-flow schedule and suction
/// floor follow the actual compressor speed), O2-gated combustion,
/// turbine work balance, reheat with O2 cap. `spool_n` is the
/// normalized compressor speed in [0, 1] — the part-power schedules
/// ride it, never the throttle command; `ignition` gates the fuel
/// schedule (false = windmilling/shutdown: airflow only).
/// `corrected_flow_kg_s` is the design corrected flow (mass at standard
/// face conditions); demand follows δ/√θ off-design, capped by intake
/// capture. `power_takeoff_w` is work extracted by the downstream power
/// turbine; its heat budget is `power_takeoff_heat_fraction` of combustor
/// heat, and the same work reduces core-nozzle total temperature/pressure.
fn run_cycle(
    spec: &AirbreathingSpec,
    fuel: (f64, f64, f64, f64, f64, f64),
    condition: &FlightCondition,
    turbine_temp_k: f64,
    drive: CycleDriveInput,
) -> Result<CycleState, PropulsionError> {
    let CycleDriveInput {
        spool_n,
        ignition,
        corrected_flow_kg_s,
        power_takeoff_w,
        power_takeoff_heat_fraction,
        conditioning,
    } = drive;
    if !power_takeoff_w.is_finite() || power_takeoff_w < 0.0 {
        return Err(PropulsionError::InvalidCommand(
            "power-turbine load must be finite and >= 0".into(),
        ));
    }
    if !power_takeoff_heat_fraction.is_finite()
        || !(0.0..=1.0).contains(&power_takeoff_heat_fraction)
    {
        return Err(PropulsionError::InvalidSpec(
            "power-turbine heat fraction must be finite in [0, 1]".into(),
        ));
    }
    if power_takeoff_w > 0.0 && (!ignition || power_takeoff_heat_fraction == 0.0) {
        return Err(PropulsionError::InvalidCommand(
            "power-turbine load requires a lit core and nonzero authored takeoff capacity".into(),
        ));
    }
    let (lhv, _f_stoich, o2_per_fuel, gamma_b, r_b, _density) = fuel;
    let gamma_a = AIR_GAMMA;
    let cp_b = gamma_b * r_b / (gamma_b - 1.0);
    let is_passive = spec.cycle.is_passive();
    let scramjet_limited = spec.cycle == AirCycle::Scramjet && condition.mach <= 1.0;
    let spool_n = spool_n.clamp(0.0, 1.0);
    // Ram conditions with recovery.
    let t_ram =
        condition.ambient_temp_k * (1.0 + (gamma_a - 1.0) / 2.0 * condition.mach * condition.mach);
    let p_ram = condition.ambient_pa
        * (t_ram / condition.ambient_temp_k).powf(gamma_a / (gamma_a - 1.0))
        * spec.intake.recovery(condition.mach);
    let compressor_inlet_total_temp_k = conditioning
        .map(|input| input.compressor_inlet_total_temp_k)
        .unwrap_or(t_ram);
    let compressor_pressure_recovery = conditioning
        .map(|input| input.compressor_pressure_recovery)
        .unwrap_or(1.0);
    if !compressor_inlet_total_temp_k.is_finite()
        || compressor_inlet_total_temp_k <= 0.0
        || compressor_inlet_total_temp_k > t_ram + 1e-9
        || !compressor_pressure_recovery.is_finite()
        || !(0.0..=1.0).contains(&compressor_pressure_recovery)
    {
        return Err(PropulsionError::InvalidCommand(
            "pre-cooler outlet temperature and pressure recovery are invalid".into(),
        ));
    }
    // Air available: ram capture, plus the suction floor for active
    // (turbomachinery) cycles scaled by actual spool speed — a stopped
    // compressor inhales nothing. Ramjet/scramjet cycles are passive:
    // capture only.
    let sound = (gamma_a * 287.0 * condition.ambient_temp_k).sqrt();
    let rho_0 = condition.ambient_pa / (287.0 * condition.ambient_temp_k);
    let suction = if is_passive {
        0.0
    } else {
        INTAKE_DESIGN_CAPTURE_MACH * sound * spool_n
    };
    let available_kg_s = rho_0 * spec.intake_area_m2 * condition.airspeed_mps.max(suction);
    // Corrected-flow part-power schedule follows spool speed (passive-cycle
    // capture is spool-independent: no machinery to schedule).
    let flow_factor = if is_passive {
        1.0
    } else {
        0.35 + 0.65 * spool_n
    };
    let demanded_kg_s = match corrected_flow_kg_s {
        Some(wc) => wc * (p_ram / 101_325.0) / (t_ram / 288.15).sqrt() * flow_factor,
        None => available_kg_s * flow_factor,
    };
    let mdot_air_kg_s = demanded_kg_s.min(available_kg_s);
    let oxygen_limited_starved = demanded_kg_s > available_kg_s && available_kg_s <= 0.0;
    let air_starved = demanded_kg_s > available_kg_s + 1e-9;
    // Core/bypass split (passive cycles: all core, no machinery).
    let bypass = match spec.cycle {
        AirCycle::Turbofan => spec.bypass_ratio,
        _ => 0.0,
    };
    let mdot_core_kg_s = mdot_air_kg_s / (1.0 + bypass);
    // Compression (passive cycles: ram only). Turbomachinery pressure ratio
    // follows spool^2: no rotation, no pressure rise.
    let pi_c = match spec.cycle {
        AirCycle::Ramjet | AirCycle::Scramjet => 1.0,
        _ => 1.0 + (spec.compressor_ratio - 1.0) * spool_n * spool_n,
    };
    let tau_c = pi_c.powf((gamma_a - 1.0) / (gamma_a * COMPRESSOR_POLY_EFFICIENCY));
    let t_comp_exit = compressor_inlet_total_temp_k * tau_c;
    let p_comp_exit = p_ram * compressor_pressure_recovery * pi_c;
    let pi_f = if bypass > 0.0 {
        1.0 + (spec.fan_pressure_ratio - 1.0) * spool_n * spool_n
    } else {
        1.0
    };
    let tau_f = pi_f.powf((gamma_a - 1.0) / (gamma_a * COMPRESSOR_POLY_EFFICIENCY));
    let t_fan_exit = t_ram * tau_f;
    let p_fan_exit = p_ram * pi_f * 0.98;
    // Shaft demand: what the compressor and fan take from the shaft at
    // this spool speed (specific works per kg of core flow; the fan
    // term carries the bypass weighting). Zero at zero spool (no head),
    // so a stopped starterless engine books no demand and no suction.
    let work_comp = AIR_CP_J_KG_K * (t_comp_exit - compressor_inlet_total_temp_k);
    let work_fan = if bypass > 0.0 {
        bypass * AIR_CP_J_KG_K * (t_fan_exit - t_ram)
    } else {
        0.0
    };
    let shaft_demand_w = mdot_core_kg_s * (work_comp + work_fan);
    // Combustion with O2 gating: fuel capped by available oxygen, TIT
    // follows energy (oxygen-limited operation derates TIT, documented).
    // Passive cycles carry no turbine cooling bleed (dump combustor).
    let bleed = match spec.cycle {
        AirCycle::Ramjet | AirCycle::Scramjet => 0.0,
        _ => TURBINE_COOLING_BLEED,
    };
    let o2_mass_fraction = condition.composition.mass_fraction(GasKind::Oxygen);
    let o2_avail_kg_s = mdot_core_kg_s * (1.0 - bleed - CUSTOMER_BLEED_FRACTION) * o2_mass_fraction;
    let combustor_air_kg_s = mdot_core_kg_s * (1.0 - bleed - CUSTOMER_BLEED_FRACTION);
    let f_for_tit = AIR_CP_J_KG_K * (turbine_temp_k - t_comp_exit) / (COMBUSTOR_EFFICIENCY * lhv);
    let conditioning_fuel = conditioning.map(|input| input.boost_fuel.properties());
    let boost_flow_requested_kg_s = if ignition && !scramjet_limited {
        conditioning.map_or(0.0, |input| input.boost_fuel_flow_kg_s)
    } else {
        0.0
    };
    let mut coolant_heat_flow_w = if ignition && !scramjet_limited {
        conditioning.map_or(0.0, |input| input.coolant_heat_flow_w)
    } else {
        0.0
    };
    let wall_heat_flow_w = if ignition && !scramjet_limited {
        conditioning.map_or(0.0, |input| input.wall_heat_flow_w)
    } else {
        0.0
    };
    let (
        fuel_air,
        fuel_flow_kg_s,
        boost_fuel_flow_kg_s,
        oxygen_limited,
        turbine_temp_eff,
        combustor_heat_release_w,
        core_gamma,
        core_gas_constant_j_kg_k,
        core_cp,
    ) = if let Some(boost_properties) =
        conditioning_fuel.filter(|_| boost_flow_requested_kg_s > 0.0)
    {
        let boost_lhv = boost_properties.0;
        let boost_o2_per_fuel = boost_properties.2;
        let requested_heat_w = (AIR_CP_J_KG_K * (turbine_temp_k - t_comp_exit) * mdot_core_kg_s
            - coolant_heat_flow_w)
            .max(0.0)
            / COMBUSTOR_EFFICIENCY;
        let bulk_wanted_kg_s =
            (requested_heat_w - boost_flow_requested_kg_s * boost_lhv).max(0.0) / lhv;
        let requested_oxygen_kg_s =
            bulk_wanted_kg_s * o2_per_fuel + boost_flow_requested_kg_s * boost_o2_per_fuel;
        let oxygen_scale = if requested_oxygen_kg_s > 0.0 {
            (o2_avail_kg_s / requested_oxygen_kg_s).clamp(0.0, 1.0)
        } else {
            1.0
        };
        // Boost fuel is both the precooler coolant and combustor fuel. When
        // oxygen availability scales its actual flow, the heat it can carry
        // back to the combustor must scale with that same flow.
        coolant_heat_flow_w *= oxygen_scale;
        let bulk_flow = bulk_wanted_kg_s * oxygen_scale;
        let boost_flow = boost_flow_requested_kg_s * oxygen_scale;
        let total_fuel_flow = bulk_flow + boost_flow;
        let fuel_air = if combustor_air_kg_s > 0.0 {
            total_fuel_flow / combustor_air_kg_s
        } else {
            0.0
        };
        let heat_release =
            COMBUSTOR_EFFICIENCY * (bulk_flow * lhv + boost_flow * boost_lhv) + coolant_heat_flow_w;
        let turbine_temp = t_comp_exit + heat_release / (AIR_CP_J_KG_K * mdot_core_kg_s.max(1e-12));
        let boost_cp = boost_properties.3 * boost_properties.4 / (boost_properties.3 - 1.0);
        let main_cp = cp_b;
        let mixed_cp = if total_fuel_flow > 0.0 {
            (bulk_flow * main_cp + boost_flow * boost_cp) / total_fuel_flow
        } else {
            main_cp
        };
        let mixed_r = if total_fuel_flow > 0.0 {
            (bulk_flow * r_b + boost_flow * boost_properties.4) / total_fuel_flow
        } else {
            r_b
        };
        (
            fuel_air,
            bulk_flow,
            boost_flow,
            requested_oxygen_kg_s > o2_avail_kg_s,
            turbine_temp,
            heat_release,
            mixed_cp / (mixed_cp - mixed_r),
            mixed_r,
            mixed_cp,
        )
    } else {
        let f_o2_cap = if combustor_air_kg_s > 0.0 {
            o2_avail_kg_s / (combustor_air_kg_s * o2_per_fuel)
        } else {
            0.0
        };
        let fuel_air = if ignition && !scramjet_limited {
            f_for_tit.max(0.0).min(f_o2_cap)
        } else {
            0.0
        };
        let fuel_flow = fuel_air * mdot_core_kg_s * (1.0 - bleed - CUSTOMER_BLEED_FRACTION);
        let heat_release = fuel_flow * COMBUSTOR_EFFICIENCY * lhv;
        (
            fuel_air,
            fuel_flow,
            0.0,
            f_for_tit > f_o2_cap && f_for_tit > 0.0,
            t_comp_exit + fuel_air * COMBUSTOR_EFFICIENCY * lhv / AIR_CP_J_KG_K,
            heat_release,
            gamma_b,
            r_b,
            cp_b,
        )
    };
    // No fuel, no cycle: flameout (anoxic air, ignition off, or thermal
    // infeasibility — compressor delivery hotter than the TIT schedule
    // allows).
    if fuel_air <= 0.0 && mdot_core_kg_s > 0.0 {
        if power_takeoff_w > 0.0 {
            return Err(PropulsionError::InvalidCommand(
                "power-turbine load requires positive combustor fuel flow".into(),
            ));
        }
        return Ok(CycleState {
            mdot_air_kg_s,
            mdot_core_kg_s,
            fuel_flow_kg_s: 0.0,
            boost_fuel_flow_kg_s: 0.0,
            fuel_ab_flow_kg_s: 0.0,
            nozzle_flow_kg_s: mdot_core_kg_s,
            core_total_temp_k: t_ram,
            core_total_pressure_pa: p_ram,
            fan_total_temp_k: t_fan_exit,
            fan_total_pressure_pa: p_fan_exit,
            shaft_demand_w,
            combustor_heat_release_w: 0.0,
            core_gamma,
            core_gas_constant_j_kg_k,
            compressor_inlet_total_temp_k,
            precooler_heat_flow_w: coolant_heat_flow_w + wall_heat_flow_w,
            precooler_wall_heat_flow_w: wall_heat_flow_w,
            power_takeoff_capacity_w: 0.0,
            oxygen_limited: oxygen_limited || oxygen_limited_starved,
            reheat_limited: false,
            drive_limited: spec.cycle.has_shaft() && f_for_tit <= 0.0,
            combustion_thermal_limited: f_for_tit <= 0.0,
            air_starved,
            scramjet_limited,
        });
    }
    let p_comb = p_comp_exit * (1.0 - COMBUSTOR_DP_FRACTION);
    // Turbine work balance: the rotor sees combustor flow only; cooling
    // bleed bypasses the rotor and mixes downstream at rotor-exit
    // pressure (documented mixing loss: the bleed carries no work).
    let rotor_flow_ratio = (1.0 - bleed - CUSTOMER_BLEED_FRACTION) * (1.0 + fuel_air);
    let delta_t_rotor = (work_comp + work_fan) / (core_cp * rotor_flow_ratio);
    // Feasibility: the turbine must supply compression work with margin.
    if delta_t_rotor >= turbine_temp_eff * 0.95 && mdot_core_kg_s > 0.0 {
        if power_takeoff_w > 0.0 {
            return Err(PropulsionError::InvalidCommand(
                "power-turbine load unavailable while the core turbine is drive-limited".into(),
            ));
        }
        return Ok(CycleState {
            mdot_air_kg_s,
            mdot_core_kg_s,
            fuel_flow_kg_s: 0.0,
            boost_fuel_flow_kg_s: 0.0,
            fuel_ab_flow_kg_s: 0.0,
            nozzle_flow_kg_s: mdot_core_kg_s,
            core_total_temp_k: t_ram,
            core_total_pressure_pa: p_ram,
            fan_total_temp_k: t_fan_exit,
            fan_total_pressure_pa: p_fan_exit,
            shaft_demand_w,
            combustor_heat_release_w: 0.0,
            core_gamma,
            core_gas_constant_j_kg_k,
            compressor_inlet_total_temp_k,
            precooler_heat_flow_w: coolant_heat_flow_w + wall_heat_flow_w,
            precooler_wall_heat_flow_w: wall_heat_flow_w,
            power_takeoff_capacity_w: 0.0,
            oxygen_limited: oxygen_limited || oxygen_limited_starved,
            reheat_limited: false,
            drive_limited: true,
            combustion_thermal_limited: false,
            air_starved,
            scramjet_limited,
        });
    }
    // Rotor exit, then cold-bleed mixing at rotor pressure with a
    // documented momentum-mixing loss (documented). Customer bleed leaves
    // overboard: ram drag without gross thrust.
    let t_rotor_exit = turbine_temp_eff - delta_t_rotor;
    let p_rotor_exit = p_comb
        * (1.0 - delta_t_rotor / (turbine_temp_eff * TURBINE_POLY_EFFICIENCY))
            .max(0.01)
            .powf(core_gamma / (core_gamma - 1.0));
    let bleed_flow_ratio = bleed / rotor_flow_ratio.max(1e-12);
    let t_turb_exit = t_rotor_exit * (1.0 - bleed_flow_ratio) + t_comp_exit * bleed_flow_ratio;
    let mixing_loss = if bleed > 0.0 {
        TURBINE_MIXING_DP_FRACTION
    } else {
        0.0
    };
    let p_turb_exit = p_rotor_exit * (1.0 - mixing_loss);
    let hot_nozzle_flow_kg_s =
        mdot_core_kg_s * (rotor_flow_ratio + bleed) + fuel_flow_kg_s + boost_fuel_flow_kg_s;
    let configured_takeoff_w = power_takeoff_heat_fraction * combustor_heat_release_w;
    let max_power_turbine_delta_t = (t_turb_exit - condition.ambient_temp_k)
        .max(0.0)
        .min(t_turb_exit * TURBINE_POLY_EFFICIENCY * (1.0 - 1.0e-12));
    let thermal_takeoff_w =
        (hot_nozzle_flow_kg_s * core_cp * TURBINE_POLY_EFFICIENCY * max_power_turbine_delta_t)
            .max(0.0);
    let power_takeoff_capacity_w = configured_takeoff_w.min(thermal_takeoff_w);
    if power_takeoff_w > power_takeoff_capacity_w + 1.0e-9 * power_takeoff_capacity_w.max(1.0) {
        return Err(PropulsionError::InvalidCommand(format!(
            "power-turbine load {:.3} MW exceeds available {:.3} MW",
            power_takeoff_w / 1.0e6,
            power_takeoff_capacity_w / 1.0e6,
        )));
    }
    let mut t_turb_exit = t_turb_exit;
    let mut p_turb_exit = p_turb_exit;
    if power_takeoff_w > 0.0 {
        let delta_t_power_turbine =
            power_takeoff_w / (hot_nozzle_flow_kg_s * core_cp * TURBINE_POLY_EFFICIENCY);
        let pressure_factor = 1.0 - delta_t_power_turbine / (t_turb_exit * TURBINE_POLY_EFFICIENCY);
        if !(pressure_factor > 0.0) || !pressure_factor.is_finite() {
            return Err(PropulsionError::InvalidCommand(
                "power-turbine extraction leaves no physical core-nozzle state".into(),
            ));
        }
        p_turb_exit *= pressure_factor.powf(core_gamma / (core_gamma - 1.0));
        t_turb_exit -= delta_t_power_turbine;
    }
    // Afterburner with an explicit species O2 budget: combustor air
    // arrives with its oxygen minus what the core burned; cooling bleed
    // rejoins carrying its oxygen with it (burnable here); customer bleed
    // left overboard and never returns. All terms are mass flows, one
    // basis throughout.
    let mut fuel_ab_flow_kg_s = 0.0;
    let mut t_nozzle = t_turb_exit;
    let mut p_nozzle = p_turb_exit;
    let mut reheat_limited = false;
    if spec.afterburner && mdot_core_kg_s > 0.0 {
        let boost_o2_per_fuel = conditioning_fuel.map_or(0.0, |properties| properties.2);
        let o2_used_core_kg_s =
            fuel_flow_kg_s * o2_per_fuel + boost_fuel_flow_kg_s * boost_o2_per_fuel;
        let o2_cooling_kg_s = mdot_core_kg_s * bleed * o2_mass_fraction;
        let o2_for_ab_kg_s = (o2_avail_kg_s - o2_used_core_kg_s + o2_cooling_kg_s).max(0.0);
        let ab_stream_kg_s =
            mdot_core_kg_s * (rotor_flow_ratio + bleed) + fuel_flow_kg_s + boost_fuel_flow_kg_s;
        let f_ab_want =
            cp_b * (spec.reheat_temp_k - t_turb_exit).max(0.0) / (COMBUSTOR_EFFICIENCY * lhv);
        let f_ab_cap = if ab_stream_kg_s > 0.0 {
            o2_for_ab_kg_s / (ab_stream_kg_s * o2_per_fuel)
        } else {
            0.0
        };
        let f_ab = f_ab_want.min(f_ab_cap);
        reheat_limited = f_ab_want > f_ab_cap && f_ab_want > 0.0;
        fuel_ab_flow_kg_s = f_ab * ab_stream_kg_s;
        t_nozzle = t_turb_exit + f_ab * COMBUSTOR_EFFICIENCY * lhv / cp_b;
        p_nozzle = p_turb_exit * (1.0 - AFTERBURNER_DP_FRACTION);
    }
    Ok(CycleState {
        mdot_air_kg_s,
        mdot_core_kg_s,
        fuel_flow_kg_s,
        boost_fuel_flow_kg_s,
        fuel_ab_flow_kg_s,
        nozzle_flow_kg_s: mdot_core_kg_s * (rotor_flow_ratio + bleed)
            + fuel_flow_kg_s
            + boost_fuel_flow_kg_s
            + fuel_ab_flow_kg_s,
        core_total_temp_k: t_nozzle,
        core_total_pressure_pa: p_nozzle,
        fan_total_temp_k: t_fan_exit,
        fan_total_pressure_pa: p_fan_exit,
        shaft_demand_w,
        combustor_heat_release_w,
        core_gamma,
        core_gas_constant_j_kg_k,
        compressor_inlet_total_temp_k,
        precooler_heat_flow_w: coolant_heat_flow_w + wall_heat_flow_w,
        precooler_wall_heat_flow_w: wall_heat_flow_w,
        power_takeoff_capacity_w,
        oxygen_limited: oxygen_limited || oxygen_limited_starved,
        reheat_limited,
        drive_limited: false,
        combustion_thermal_limited: false,
        air_starved,
        scramjet_limited,
    })
}

/// Fixed-geometry convergent-divergent nozzle (ramjet/scramjet cruise): exit Mach
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
    /// Core throat area (ramjet/scramjet C-D; turbojets/fans run convergent,
    /// so the throat equals the exit area).
    pub throat_area_core_m2: f64,
    pub design_static_thrust_n: f64,
    pub design_fuel_flow_kg_s: f64,
    pub design_isp_s: f64,
    pub dry_mass_kg: f64,
    pub spool_tau_s: f64,
    /// Shaft topology carried through compile (starter/generator mass is
    /// already booked into `dry_mass_kg`).
    #[serde(default)]
    pub shaft: ShaftSpec,
    /// Design-point shaft normalization `FRAC`: turbine heat fraction
    /// scaled so the full-throttle sea-level design point is an exact
    /// shaft equilibrium (demand + friction = capacity at spool 1.0).
    /// Ramjets/scramjets carry 0.0 (no shaft).
    #[serde(default)]
    pub shaft_turbine_frac: f64,
    /// Reference shaft power `P_ref` (W): design turbine shaft capacity
    /// before normalization. Sizes bearing friction and the normalized
    /// spool dynamics (`dn/dt = net / (P_ref * spool_tau_s)`). Ramjets
    /// carry 0.0 for ramjets/scramjets (no shaft).
    #[serde(default)]
    pub shaft_reference_power_w: f64,
}

/// Instantaneous air-breathing operating point.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct AirOperatingPoint {
    pub thrust_n: f64,
    pub fuel_flow_kg_s: f64,
    /// Bulk-fuel portion of `fuel_flow_kg_s` (including reheat fuel).
    #[serde(default)]
    pub bulk_fuel_flow_kg_s: f64,
    /// Independently metered coolant/boost-fuel portion of total fuel.
    #[serde(default)]
    pub boost_fuel_flow_kg_s: f64,
    pub air_flow_kg_s: f64,
    pub exhaust_temp_k: f64,
    pub exhaust_velocity_mps: f64,
    pub exit_pressure_pa: f64,
    pub exit_mach: f64,
    pub isp_s: f64,
    #[serde(default)]
    pub compressor_inlet_total_temp_k: f64,
    #[serde(default)]
    pub precooler_heat_flow_w: f64,
    #[serde(default)]
    pub precooler_wall_heat_flow_w: f64,
    /// Solved steady spool speed (normalized) this point was evaluated
    /// at — or the sustainable equilibrium the steady solver found.
    /// `0.0` means no sustainable shaft equilibrium exists (the engine
    /// cannot hold its own compressor there); in that case the
    /// thermodynamic channels are evaluated at full spool as an
    /// already-running attempt so failure flags stay reportable
    /// (documented analyzer fallback — runtime shaft truth lives in
    /// `propulsion::shaft::advance_jet_shaft`).
    #[serde(default)]
    pub spool_n: f64,
    /// True when the core is actually burning in this point (fuel flow
    /// positive with air present). A commanded-but-unlit engine — failed
    /// light-off, anoxic air, or an unsustained shaft — reports false
    /// with cause flags instead of pretending to run.
    #[serde(default)]
    pub lit: bool,
    /// True when the intake cannot supply demanded flow.
    pub air_limited: bool,
    /// True when oxygen (not fuel schedule) caps combustion.
    pub oxygen_limited: bool,
    /// True when the turbine cannot drive compression (flameout region).
    pub drive_limited: bool,
    /// True when inlet total temperature already exceeds the combustor target.
    #[serde(default)]
    pub combustion_thermal_limited: bool,
    /// True when a scramjet is below the supersonic-combustor boundary.
    #[serde(default)]
    pub scramjet_limited: bool,
    /// True when the fixed nozzle, not the intake, caps flow.
    pub nozzle_limited: bool,
    /// True when a ramjet C-D nozzle separates (overexpanded).
    pub separation_risk: bool,
    /// True when part of the airflow came from the static suction floor
    /// scaled by actual spool speed: the steady-running assumption is
    /// active (an already-spinning compressor), and the analyzer labels
    /// it per row.
    pub suction_assisted: bool,
    /// True when the afterburner is lit.
    pub reheat_active: bool,
    /// True when O2 caps the reheat.
    pub reheat_limited: bool,
}

impl AirbreathingSpec {
    /// Hangar compile: solve the design point, size the convergent nozzles,
    /// derive mass. Turbojets/fans design at sea-level static; ramjets at
    /// Mach 2 sea level; scramjets at Mach 6 / 20 km. Passive cycles have no
    /// static flow point to design on.
    pub fn compile(&self) -> Result<CompiledAirbreather, PropulsionError> {
        self.validate()?;
        let fuel = self.fuel.properties();
        let design_condition = match self.cycle {
            AirCycle::Ramjet => FlightCondition {
                mach: 2.0,
                ambient_pa: 101_325.0,
                ambient_temp_k: 288.15,
                airspeed_mps: 2.0 * (AIR_GAMMA * 287.0 * 288.15).sqrt(),
                composition: AtmosphereComposition::earth_air(),
            },
            AirCycle::Scramjet => {
                let atmosphere = crate::atmosphere::AtmosphereConfig::default()
                    .with_composition(AtmosphereComposition::earth_air());
                let sample = atmosphere.sample(20_000.0).map_err(|error| {
                    PropulsionError::InvalidSpec(format!("scramjet design atmosphere: {error}"))
                })?;
                flight_condition(&sample, 6.0 * sample.speed_of_sound_mps)?
            }
            _ => FlightCondition {
                mach: 0.0,
                ambient_pa: 101_325.0,
                ambient_temp_k: 288.15,
                airspeed_mps: 0.0,
                composition: AtmosphereComposition::earth_air(),
            },
        };
        let rho_sl = 101_325.0 / (287.0 * 288.15);
        let a_sl = (AIR_GAMMA * 287.0 * 288.15).sqrt();
        let design_flow_kg_s = match self.cycle {
            AirCycle::Ramjet => rho_sl * self.intake_area_m2 * design_condition.airspeed_mps,
            AirCycle::Scramjet => {
                design_condition.ambient_pa / (287.0 * design_condition.ambient_temp_k)
                    * self.intake_area_m2
                    * design_condition.airspeed_mps
            }
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
            CycleDriveInput {
                spool_n: 1.0,
                ignition: true,
                corrected_flow_kg_s: Some(design_corrected_flow_kg_s),
                power_takeoff_w: 0.0,
                power_takeoff_heat_fraction: self.shaft.power_turbine_heat_fraction,
                conditioning: None,
            },
        )?;
        if state.drive_limited {
            return Err(PropulsionError::UnsupportedCombination(
                "turbine cannot drive the compressor at the design point".into(),
            ));
        }
        // Core-shaft calibration at the design point (documented): reference
        // power is the design turbine shaft capacity, and `FRAC`
        // normalizes it so demand + bearing friction exactly balance
        // capacity at spool 1.0 — the full-throttle sea-level design
        // point is an exact steady equilibrium, which anchors every
        // runtime comparison against v5 numbers. `FRAC` is the ratio of
        // shaft work the design actually needs to the nominal 50%-of-
        // heat split, so well-calibrated engines land near 1.0 (a
        // fraction slightly above 1.0 just means the compressor needs a
        // hair more than the nominal split). A configured downstream
        // power-turbine share uses the remaining combustor-heat budget. The
        // refusal enforces energy conservation: core-shaft fraction times
        // the nominal split plus the power-turbine fraction may not exceed
        // all combustor heat release. Gas-side
        // feasibility (turbine temperature margin at the design point)
        // is refused separately above — that check, not this one, is
        // what rejects an engine that cannot drive its compressor.
        let lhv = fuel.0;
        let (shaft_turbine_frac, shaft_reference_power_w) = if !self.cycle.has_shaft() {
            (0.0, 0.0)
        } else {
            let capacity_raw_w =
                TURBINE_SHAFT_HEAT_FRACTION * state.fuel_flow_kg_s * COMBUSTOR_EFFICIENCY * lhv;
            if !(capacity_raw_w > 0.0) {
                return Err(PropulsionError::UnsupportedCombination(
                    "design point releases no shaft-usable heat".into(),
                ));
            }
            let frac =
                (state.shaft_demand_w + SHAFT_FRICTION_FRACTION * capacity_raw_w) / capacity_raw_w;
            let maximum_core_fraction =
                (1.0 - self.shaft.power_turbine_heat_fraction) / TURBINE_SHAFT_HEAT_FRACTION;
            if !(frac <= maximum_core_fraction) {
                return Err(PropulsionError::UnsupportedCombination(
                    "core shaft and power-turbine demand exceed combustor heat release at the design point".into(),
                ));
            }
            (frac, capacity_raw_w)
        };
        // Reheat must clear the solved design turbine-exit temperature
        // (spec-level TIT comparison would reject valid targets between
        // turbine exit and TIT).
        if self.afterburner && self.reheat_temp_k <= state.core_total_temp_k {
            return Err(PropulsionError::UnsupportedCombination(format!(
                "reheat {:.0} K must exceed design turbine-exit {:.0} K",
                self.reheat_temp_k, state.core_total_temp_k
            )));
        }
        let (_, _, _, gamma_b, r_b, _) = fuel;
        let mdot_core_hot = state.nozzle_flow_kg_s;
        // Ramjets/scramjets size a fixed convergent-divergent nozzle adapted
        // at the design point; turbine cycles run convergent (throat = exit).
        let throat_area_core_m2 = size_nozzle_area(
            mdot_core_hot,
            state.core_total_temp_k,
            state.core_total_pressure_pa,
            gamma_b,
            r_b,
        );
        let exit_area_core_m2 = match self.cycle {
            AirCycle::Ramjet | AirCycle::Scramjet => {
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
                "design point overexpands the passive-cycle nozzle; pick a faster design Mach"
                    .into(),
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
            AirCycle::Ramjet | AirCycle::Scramjet => 0.0,
            _ => MASS_FIT_COMPRESSOR * design_flow_kg_s * self.compressor_ratio,
        };
        let turbine_kg = match self.cycle {
            AirCycle::Ramjet | AirCycle::Scramjet => 0.0,
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
        // Starter and generator are authored shaft hardware with real
        // mass (0 for a deliberate starterless/generator-less design).
        let shaft_kg = self.shaft.starter.mass_kg + self.shaft.generator.mass_kg;
        let dry_mass_kg = compressor_kg
            + turbine_kg
            + combustor_kg
            + intake_kg
            + nozzle_kg
            + ab_kg
            + fan_kg
            + shaft_kg
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
            shaft: self.shaft.clone(),
            shaft_turbine_frac,
            shaft_reference_power_w,
        })
    }
}

impl CompiledAirbreather {
    /// Part-power effective spec at a commanded throttle: the TIT
    /// schedule and reheat gate follow the throttle command
    /// (documented); pressure and flow schedules ride spool speed
    /// inside [`run_cycle`], never the throttle, so an unspooled
    /// engine never gets full-pressure air for free.
    fn effective_spec(&self, throttle: f64) -> (AirbreathingSpec, f64, bool) {
        let tit = self.turbine_inlet_temp_k * (0.55 + 0.45 * throttle);
        let eff_reheat = self.afterburner && throttle >= 0.9;
        let spec_eff = AirbreathingSpec {
            afterburner: eff_reheat,
            turbine_inlet_temp_k: tit,
            name: self.name.clone(),
            cycle: self.cycle,
            fuel: self.fuel,
            intake_area_m2: self.intake_area_m2,
            intake: self.intake,
            compressor_ratio: self.compressor_ratio,
            bypass_ratio: self.bypass_ratio,
            fan_pressure_ratio: self.fan_pressure_ratio,
            reheat_temp_k: self.reheat_temp_k,
            turbine_material: ChamberMaterial::nickel_superalloy(),
            spool_tau_s: self.spool_tau_s,
            shaft: self.shaft.clone(),
        };
        (spec_eff, tit, eff_reheat)
    }

    /// Lean shaft power balance at an explicit spool speed: one cycle
    /// evaluation, no nozzle matching — the form the steady spool solver
    /// bisects (identical numbers to the balance returned by
    /// [`Self::operating_point_at_spool`], just cheaper per call).
    ///
    /// Demand and friction cost rotation whether or not the core burns;
    /// capacity exists only when ignition actually runs (reheat fuel is
    /// downstream of the turbine and never drives the shaft).
    fn lean_shaft_balance(
        &self,
        condition: &FlightCondition,
        throttle: f64,
        spool_n: f64,
        ignition: bool,
    ) -> Result<ShaftBalance, PropulsionError> {
        let fuel = self.fuel.properties();
        let (spec_eff, tit, _) = self.effective_spec(throttle);
        let state = run_cycle(
            &spec_eff,
            fuel,
            condition,
            tit,
            CycleDriveInput {
                spool_n,
                ignition,
                corrected_flow_kg_s: Some(self.design_corrected_flow_kg_s),
                power_takeoff_w: 0.0,
                power_takeoff_heat_fraction: self.shaft.power_turbine_heat_fraction,
                conditioning: None,
            },
        )?;
        Ok(self.balance_from_state(&state, spool_n, ignition))
    }

    /// Balance view of an already-evaluated cycle state (shared by the
    /// operating-point path and [`Self::shaft_balance`]).
    fn balance_from_state(&self, state: &CycleState, spool_n: f64, ignition: bool) -> ShaftBalance {
        ShaftBalance {
            demand_w: state.shaft_demand_w,
            capacity_w: if ignition {
                self.shaft_turbine_frac
                    * TURBINE_SHAFT_HEAT_FRACTION
                    * state.combustor_heat_release_w
            } else {
                0.0
            },
            power_takeoff_capacity_w: if ignition {
                state.power_takeoff_capacity_w
            } else {
                0.0
            },
            friction_w: SHAFT_FRICTION_FRACTION * self.shaft_reference_power_w * spool_n.powi(3),
        }
    }

    /// Evaluate the air path at an EXPLICIT spool speed (transient
    /// state from the shaft machine, or a steady-solved value) instead
    /// of solving for equilibrium. `ignition` gates the fuel schedule:
    /// false = windmilling/shutdown (airflow only, zero fuel), true =
    /// the fuel schedule runs — light-off viability is still judged by
    /// the caller (`propulsion::shaft` owns the hysteresis), while O2
    /// and thermal feasibility stay gated inside the cycle. Returns the
    /// operating point plus the shaft power balance at this spool
    /// speed.
    pub fn operating_point_at_spool(
        &self,
        condition: &FlightCondition,
        throttle: f64,
        spool_n: f64,
        ignition: bool,
    ) -> Result<(AirOperatingPoint, ShaftBalance), PropulsionError> {
        self.operating_point_at_spool_loaded(condition, throttle, spool_n, ignition, 0.0)
    }

    /// [`Self::operating_point_at_spool`] with mechanical power extracted by
    /// an authored downstream power turbine. The requested load is checked
    /// against both the configured combustor-heat budget and the available
    /// gas enthalpy; extracted work lowers core-nozzle total temperature and
    /// pressure.
    pub fn operating_point_at_spool_loaded(
        &self,
        condition: &FlightCondition,
        throttle: f64,
        spool_n: f64,
        ignition: bool,
        power_takeoff_w: f64,
    ) -> Result<(AirOperatingPoint, ShaftBalance), PropulsionError> {
        self.operating_point_at_spool_internal(
            condition,
            throttle,
            spool_n,
            ignition,
            power_takeoff_w,
            None,
        )
    }

    pub(super) fn operating_point_at_spool_conditioned(
        &self,
        condition: &FlightCondition,
        throttle: f64,
        spool_n: f64,
        ignition: bool,
        conditioning: AirCycleConditioning,
    ) -> Result<(AirOperatingPoint, ShaftBalance), PropulsionError> {
        self.operating_point_at_spool_internal(
            condition,
            throttle,
            spool_n,
            ignition,
            0.0,
            Some(conditioning),
        )
    }

    fn operating_point_at_spool_internal(
        &self,
        condition: &FlightCondition,
        throttle: f64,
        spool_n: f64,
        ignition: bool,
        power_takeoff_w: f64,
        conditioning: Option<AirCycleConditioning>,
    ) -> Result<(AirOperatingPoint, ShaftBalance), PropulsionError> {
        condition.validate()?;
        if !throttle.is_finite() || !(0.0..=1.0).contains(&throttle) {
            return Err(PropulsionError::InvalidCommand(
                "throttle must be finite in [0, 1]".into(),
            ));
        }
        if !spool_n.is_finite() || !(0.0..=1.0).contains(&spool_n) {
            return Err(PropulsionError::InvalidCommand(
                "spool must be finite in [0, 1]".into(),
            ));
        }
        let off = AirOperatingPoint {
            thrust_n: 0.0,
            fuel_flow_kg_s: 0.0,
            bulk_fuel_flow_kg_s: 0.0,
            boost_fuel_flow_kg_s: 0.0,
            air_flow_kg_s: 0.0,
            exhaust_temp_k: condition.ambient_temp_k,
            exhaust_velocity_mps: 0.0,
            exit_pressure_pa: condition.ambient_pa,
            exit_mach: 0.0,
            isp_s: 0.0,
            compressor_inlet_total_temp_k: condition.ambient_temp_k,
            precooler_heat_flow_w: 0.0,
            precooler_wall_heat_flow_w: 0.0,
            spool_n,
            lit: false,
            air_limited: false,
            oxygen_limited: false,
            drive_limited: false,
            combustion_thermal_limited: false,
            scramjet_limited: false,
            nozzle_limited: false,
            separation_risk: false,
            suction_assisted: false,
            reheat_active: false,
            reheat_limited: false,
        };
        let fuel = self.fuel.properties();
        let (spec_eff, tit, eff_reheat) = self.effective_spec(throttle);
        let mut state = run_cycle(
            &spec_eff,
            fuel,
            condition,
            tit,
            CycleDriveInput {
                spool_n,
                ignition,
                corrected_flow_kg_s: Some(self.design_corrected_flow_kg_s),
                power_takeoff_w,
                power_takeoff_heat_fraction: self.shaft.power_turbine_heat_fraction,
                conditioning,
            },
        )?;
        // Oxygen gating can reduce the boost-fuel flow after the ESTOC
        // exchanger has scheduled its cooling. Re-evaluate the cycle with
        // only the coolant heat and mass flow that the combustor actually
        // accepts, so the compressor does not retain unpowered precooling.
        if let Some(mut current) = conditioning
            .filter(|input| input.coolant_heat_flow_w > 0.0 && input.boost_fuel_flow_kg_s > 0.0)
        {
            let ram_temp_k = condition.ambient_temp_k
                * (1.0 + (AIR_GAMMA - 1.0) * 0.5 * condition.mach * condition.mach);
            for _ in 0..16 {
                let actual_coolant_heat_w =
                    (state.precooler_heat_flow_w - state.precooler_wall_heat_flow_w).max(0.0);
                let air_capacity_w_k = state.mdot_air_kg_s * AIR_CP_J_KG_K;
                if air_capacity_w_k <= 0.0 {
                    break;
                }
                let actual_inlet_temp_k = (ram_temp_k
                    - (actual_coolant_heat_w + state.precooler_wall_heat_flow_w)
                        / air_capacity_w_k)
                    .max(1.0);
                if (current.compressor_inlet_total_temp_k - actual_inlet_temp_k).abs()
                    <= 1e-6 * actual_inlet_temp_k.max(1.0)
                {
                    break;
                }
                current.compressor_inlet_total_temp_k =
                    0.5 * (current.compressor_inlet_total_temp_k + actual_inlet_temp_k);
                state = run_cycle(
                    &spec_eff,
                    fuel,
                    condition,
                    tit,
                    CycleDriveInput {
                        spool_n,
                        ignition,
                        corrected_flow_kg_s: Some(self.design_corrected_flow_kg_s),
                        power_takeoff_w,
                        power_takeoff_heat_fraction: self.shaft.power_turbine_heat_fraction,
                        conditioning: Some(current),
                    },
                )?;
            }
        }
        let balance = self.balance_from_state(&state, spool_n, ignition);
        // Static-suction label: air above ram-only capture at low speed
        // runs on the steady-running assumption, scaled by the actual
        // spool speed this point was evaluated at.
        let sound_speed_mps = (AIR_GAMMA * 287.0 * condition.ambient_temp_k).sqrt();
        let suction_floor_mps = if self.cycle.is_passive() {
            0.0
        } else {
            INTAKE_DESIGN_CAPTURE_MACH * sound_speed_mps * spool_n
        };
        let ram_only_kg_s = condition.ambient_pa / (287.0 * condition.ambient_temp_k)
            * self.intake_area_m2
            * condition.airspeed_mps;
        let suction_assisted = |mdot_air: f64| {
            suction_floor_mps > 0.0
                && condition.airspeed_mps < suction_floor_mps
                && mdot_air > ram_only_kg_s + 1e-9
        };
        if state.drive_limited || state.mdot_air_kg_s <= 0.0 {
            // Drive/work failure and intake starvation stay distinct
            // flags: vacuum starves with a healthy drive.
            return Ok((
                AirOperatingPoint {
                    air_limited: state.mdot_air_kg_s <= 0.0 || state.air_starved,
                    oxygen_limited: state.oxygen_limited,
                    drive_limited: state.drive_limited,
                    combustion_thermal_limited: state.combustion_thermal_limited,
                    scramjet_limited: state.scramjet_limited,
                    suction_assisted: suction_assisted(state.mdot_air_kg_s),
                    compressor_inlet_total_temp_k: state.compressor_inlet_total_temp_k,
                    precooler_heat_flow_w: state.precooler_heat_flow_w,
                    precooler_wall_heat_flow_w: state.precooler_wall_heat_flow_w,
                    ..off
                },
                balance,
            ));
        }
        let fuel_bulk = state.fuel_flow_kg_s + state.fuel_ab_flow_kg_s;
        let fuel_total = fuel_bulk + state.boost_fuel_flow_kg_s;
        if fuel_total <= 0.0 {
            // Flameout with airflow (anoxic air, ignition off, or a
            // thermally infeasible TIT): the airflow is real and stays
            // reported (windmilling drags on it), zero thrust with the
            // cause flags, never NaN.
            return Ok((
                AirOperatingPoint {
                    air_flow_kg_s: state.mdot_air_kg_s,
                    air_limited: state.air_starved,
                    oxygen_limited: state.oxygen_limited,
                    drive_limited: state.drive_limited,
                    combustion_thermal_limited: state.combustion_thermal_limited,
                    scramjet_limited: state.scramjet_limited,
                    suction_assisted: suction_assisted(state.mdot_air_kg_s),
                    compressor_inlet_total_temp_k: state.compressor_inlet_total_temp_k,
                    precooler_heat_flow_w: state.precooler_heat_flow_w,
                    precooler_wall_heat_flow_w: state.precooler_wall_heat_flow_w,
                    ..off
                },
                balance,
            ));
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
            state.core_gamma,
            state.core_gas_constant_j_kg_k,
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
        let boost_fuel_flow = state.boost_fuel_flow_kg_s * scale;
        let fuel_ab_flow = state.fuel_ab_flow_kg_s * scale;
        let core_flow = mdot_core_hot * scale;
        // Turbine cycles run convergent; ramjets/scramjets run fixed C-D.
        let (core, separation_risk) = match self.cycle {
            AirCycle::Ramjet | AirCycle::Scramjet => cd_nozzle(
                core_flow,
                state.core_total_temp_k,
                state.core_total_pressure_pa,
                self.throat_area_core_m2,
                self.exit_area_core_m2,
                state.core_gamma,
                state.core_gas_constant_j_kg_k,
                condition.ambient_pa,
            )?,
            _ => (
                convergent_nozzle(
                    core_flow,
                    state.core_total_temp_k,
                    state.core_total_pressure_pa,
                    self.exit_area_core_m2,
                    state.core_gamma,
                    state.core_gas_constant_j_kg_k,
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
        let fuel_bulk = fuel_flow + fuel_ab_flow;
        let fuel_total = fuel_bulk + boost_fuel_flow;
        Ok((
            AirOperatingPoint {
                thrust_n,
                fuel_flow_kg_s: fuel_total,
                bulk_fuel_flow_kg_s: fuel_bulk,
                boost_fuel_flow_kg_s: boost_fuel_flow,
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
                compressor_inlet_total_temp_k: state.compressor_inlet_total_temp_k,
                precooler_heat_flow_w: state.precooler_heat_flow_w * scale,
                precooler_wall_heat_flow_w: state.precooler_wall_heat_flow_w * scale,
                spool_n,
                lit: fuel_total > 0.0,
                air_limited: state.air_starved,
                oxygen_limited: state.oxygen_limited,
                drive_limited: false,
                combustion_thermal_limited: state.combustion_thermal_limited,
                scramjet_limited: state.scramjet_limited,
                nozzle_limited,
                separation_risk,
                suction_assisted: suction_assisted(mdot_air),
                // Scaled AB flow directly: under nozzle limiting the unscaled
                // core comparison would misreport a lit afterburner as off.
                reheat_active: eff_reheat && fuel_ab_flow > 0.0,
                reheat_limited: state.reheat_limited,
            },
            balance,
        ))
    }

    /// Steady spool equilibrium at a commanded throttle: bisect net
    /// shaft power (capacity − demand − friction with ignition
    /// commanded and the generator unloaded — the editor/analyzer
    /// convention) over `[light_off_n, 1]`. Returns `0.0` when the core
    /// cannot sustain itself even at light-off speed (vacuum, anoxic
    /// air, too-thin air, drive-limited heat: no sustained rotation
    /// exists) and `1.0` when net power is still positive at redline
    /// (the spool is governed there). Numerical error: 40 halvings put
    /// `n*` within `(1 − light_off_n)/2^40 ≈ 1e-12` spool — orders
    /// below thrust-band resolution (documented).
    fn solve_steady_spool(
        &self,
        condition: &FlightCondition,
        throttle: f64,
    ) -> Result<f64, PropulsionError> {
        let net = |spool_n: f64| -> Result<f64, PropulsionError> {
            Ok(self
                .lean_shaft_balance(condition, throttle, spool_n, true)?
                .net_w())
        };
        let mut lo = self.shaft.light_off_n;
        if !(net(lo)? > 0.0) {
            return Ok(0.0);
        }
        let hi_net = net(1.0)?;
        if !(hi_net < 0.0) {
            return Ok(1.0);
        }
        let mut hi = 1.0;
        for _ in 0..40 {
            let mid = 0.5 * (lo + hi);
            if net(mid)? > 0.0 {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        Ok(0.5 * (lo + hi))
    }

    /// Steady operating point at a flight condition and effective
    /// (post-spool) throttle: solve the sustainable shaft equilibrium
    /// first, then evaluate the cycle there (part-power TIT and reheat
    /// follow the throttle; pressure/flow follow the solved spool).
    ///
    /// Ramjets and scramjets have no shaft: they evaluate directly at the
    /// full-flow schedule and report `spool_n = 1.0` as the no-derating
    /// placeholder. When no sustainable equilibrium exists but throttle
    /// commands run, the point is evaluated at full spool as an
    /// already-running attempt so failure flags stay reportable while
    /// `spool_n` reports the solved (stopped) shaft — the documented
    /// section 8.1 analyzer fallback; runtime shaft truth lives in
    /// `propulsion::shaft::advance_jet_shaft`.
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
        if throttle == 0.0 {
            return Ok(AirOperatingPoint {
                thrust_n: 0.0,
                fuel_flow_kg_s: 0.0,
                bulk_fuel_flow_kg_s: 0.0,
                boost_fuel_flow_kg_s: 0.0,
                air_flow_kg_s: 0.0,
                exhaust_temp_k: condition.ambient_temp_k,
                exhaust_velocity_mps: 0.0,
                exit_pressure_pa: condition.ambient_pa,
                exit_mach: 0.0,
                isp_s: 0.0,
                compressor_inlet_total_temp_k: condition.ambient_temp_k,
                precooler_heat_flow_w: 0.0,
                precooler_wall_heat_flow_w: 0.0,
                spool_n: 0.0,
                lit: false,
                air_limited: false,
                oxygen_limited: false,
                drive_limited: false,
                combustion_thermal_limited: false,
                scramjet_limited: self.cycle == AirCycle::Scramjet && condition.mach <= 1.0,
                nozzle_limited: false,
                separation_risk: false,
                suction_assisted: false,
                reheat_active: false,
                reheat_limited: false,
            });
        }
        let solved = if !self.cycle.has_shaft() {
            1.0
        } else {
            self.solve_steady_spool(condition, throttle)?
        };
        let eval_spool = if solved > 0.0 { solved } else { 1.0 };
        let (mut point, _) =
            self.operating_point_at_spool(condition, throttle, eval_spool, true)?;
        point.spool_n = solved;
        Ok(point)
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
    #[serde(default)]
    pub drive_limited: bool,
    pub combustion_thermal_limited: bool,
    pub scramjet_limited: bool,
    pub nozzle_limited: bool,
    pub separation_risk: bool,
    /// True wherever the steady-running suction assumption is active
    /// (explicit label; spool-scaled at the row's steady spool speed).
    pub suction_assisted: bool,
    /// Steady spool speed solved for this row (0.0 where the row is not
    /// self-sustaining: vacuum, anoxia, or hypersonic drive limit).
    pub spool_n: f64,
}

/// Thrust/Isp grid over altitudes × Mach numbers at fixed throttle;
/// species ride the atmosphere config's well-mixed composition (the
/// authoritative basis, section 10 — no caller scalar).
pub fn analyze_airbreathing(
    engine: &CompiledAirbreather,
    atmosphere: &crate::atmosphere::AtmosphereConfig,
    altitudes_m: &[f64],
    machs: &[f64],
    throttle: f64,
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
            let condition = flight_condition(&sample, mach * sample.speed_of_sound_mps)?;
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
                drive_limited: point.drive_limited,
                combustion_thermal_limited: point.combustion_thermal_limited,
                scramjet_limited: point.scramjet_limited,
                nozzle_limited: point.nozzle_limited,
                separation_risk: point.separation_risk,
                suction_assisted: point.suction_assisted,
                spool_n: point.spool_n,
            });
        }
    }
    Ok(rows)
}

/// First-order throttle lag for jet spools: effective throttle approaches
/// the target with time constant `tau_s`. Command-side scheduling only —
/// the physical spool balance (starter torque, light-off/self-sustain
/// hysteresis, generator load) lives in `propulsion::shaft`; this helper
/// remains for callers that want a plain lag filter on the throttle
/// command itself. Pure function, physics seconds.
pub fn advance_jet_spool(
    current_effective: f64,
    target: f64,
    dt_s: f64,
    tau_s: f64,
) -> Result<f64, PropulsionError> {
    if !current_effective.is_finite() || !target.is_finite() || !(0.0..=1.0).contains(&target) {
        return Err(PropulsionError::InvalidCommand(
            "spool throttle must be finite, target in [0, 1]".into(),
        ));
    }
    if !dt_s.is_finite() || dt_s < 0.0 || !tau_s.is_finite() || tau_s <= 0.0 {
        return Err(PropulsionError::InvalidCommand(
            "spool dt/tau must be finite, dt >= 0, tau > 0".into(),
        ));
    }
    let current = current_effective.clamp(0.0, 1.0);
    let alpha = 1.0 - (-dt_s / tau_s).exp();
    Ok((current + (target - current) * alpha).clamp(0.0, 1.0))
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
            shaft: ShaftSpec::default(),
        }
    }

    fn sl_static() -> FlightCondition {
        FlightCondition {
            mach: 0.0,
            ambient_pa: 101_325.0,
            ambient_temp_k: 288.15,
            airspeed_mps: 0.0,
            composition: AtmosphereComposition::earth_air(),
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
            shaft: ShaftSpec::default(),
        };
        let engine = spec.compile().expect("ramjet compiles");
        let statik = engine.operating_point(&sl_static(), 1.0).expect("static");
        assert_eq!(statik.thrust_n, 0.0);
        assert_eq!(statik.fuel_flow_kg_s, 0.0);
        let sample = AtmosphereConfig::default().sample(0.0).expect("SL sample");
        let mut last = 0.0;
        for mach in [1.5, 2.0, 2.5, 3.0] {
            let condition =
                flight_condition(&sample, mach * sample.speed_of_sound_mps).expect("condition");
            let point = engine.operating_point(&condition, 1.0).expect("point");
            assert!(point.thrust_n > last, "ramjet thrust must rise with Mach");
            last = point.thrust_n;
        }
        // Design-point (Mach 2) Isp in the honest band: the fixed nozzle
        // is adapted there, so this is the number the geometry promises.
        // Off-design absolute Isp is geometry-dependent (underexpansion
        // pressure thrust is real thrust on a fixed nozzle); energy
        // conservation at Mach 3 is the rigorous check instead.
        let design = flight_condition(&sample, 2.0 * sample.speed_of_sound_mps).expect("condition");
        let design_point = engine.operating_point(&design, 1.0).expect("design");
        assert!(
            (1500.0..=2200.0).contains(&design_point.isp_s),
            "ramjet design Isp {:.0} s outside the band",
            design_point.isp_s
        );
        let cruise = flight_condition(&sample, 3.0 * sample.speed_of_sound_mps).expect("condition");
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
    fn scramjet_requires_supersonic_combustion_and_reuses_the_passive_nozzle_path() {
        let spec = AirbreathingSpec {
            name: "scramjet-test".into(),
            cycle: AirCycle::Scramjet,
            fuel: JetFuel::Hydrogen,
            intake_area_m2: 0.5,
            intake: IntakeKind::Ramp,
            compressor_ratio: 1.0,
            bypass_ratio: 0.0,
            fan_pressure_ratio: 1.0,
            turbine_inlet_temp_k: 2_300.0,
            afterburner: false,
            reheat_temp_k: 0.0,
            turbine_material: ChamberMaterial::nickel_superalloy(),
            spool_tau_s: 5.0,
            shaft: ShaftSpec::default(),
        };
        let engine = spec.compile().expect("scramjet compiles");
        assert_eq!(engine.shaft_reference_power_w, 0.0);
        assert_eq!(engine.shaft_turbine_frac, 0.0);
        assert!(engine.design_flow_kg_s > 0.0);
        assert!(engine.dry_mass_kg > 0.0);

        let sample = AtmosphereConfig::default()
            .sample(20_000.0)
            .expect("20 km sample");
        let sonic = flight_condition(&sample, sample.speed_of_sound_mps).expect("sonic condition");
        let limited = engine.operating_point(&sonic, 1.0).expect("sonic boundary");
        assert!(limited.scramjet_limited);
        assert!(!limited.lit);
        assert_eq!(limited.fuel_flow_kg_s, 0.0);
        assert_eq!(limited.thrust_n, 0.0);
        assert!(
            engine
                .operating_point(&sonic, 0.0)
                .expect("off sonic point")
                .scramjet_limited
        );

        let supersonic =
            flight_condition(&sample, 6.0 * sample.speed_of_sound_mps).expect("Mach 6");
        let point = engine
            .operating_point(&supersonic, 1.0)
            .expect("supersonic operating point");
        assert!(!point.scramjet_limited);
        assert!(point.lit);
        assert!(point.fuel_flow_kg_s > 0.0);
        assert!(point.thrust_n > 0.0);
        assert!(point.exit_mach > 1.0);

        let anoxic = FlightCondition {
            composition: AtmosphereComposition::anoxic(),
            ..supersonic
        };
        let oxygen_starved = engine
            .operating_point(&anoxic, 1.0)
            .expect("anoxic scramjet point");
        assert!(oxygen_starved.oxygen_limited);
        assert!(!oxygen_starved.scramjet_limited);
        assert!(!oxygen_starved.lit);
        assert_eq!(oxygen_starved.fuel_flow_kg_s, 0.0);
        assert_eq!(oxygen_starved.thrust_n, 0.0);

        let hypersonic =
            flight_condition(&sample, 8.0 * sample.speed_of_sound_mps).expect("Mach 8");
        let too_hot = engine
            .operating_point(&hypersonic, 1.0)
            .expect("high-enthalpy inlet");
        assert!(!too_hot.scramjet_limited);
        assert!(!too_hot.drive_limited, "passive scramjets have no shaft");
        assert!(too_hot.combustion_thermal_limited);
        assert!(!too_hot.lit);
        assert_eq!(too_hot.fuel_flow_kg_s, 0.0);

        let exhaust_ground_speed = (point.exhaust_velocity_mps - supersonic.airspeed_mps).max(0.0);
        let useful_power = point.thrust_n * supersonic.airspeed_mps
            + 0.5 * point.air_flow_kg_s * exhaust_ground_speed * exhaust_ground_speed;
        let fuel_lhv = JetFuel::Hydrogen.properties().0;
        let input_power = point.fuel_flow_kg_s * fuel_lhv
            + 0.5 * point.air_flow_kg_s * supersonic.airspeed_mps.powi(2);
        assert!(useful_power < input_power, "scramjet first-law bound");

        assert!(
            AirbreathingSpec {
                afterburner: true,
                reheat_temp_k: 2_300.0,
                ..spec.clone()
            }
            .compile()
            .is_err()
        );
        assert!(
            AirbreathingSpec {
                shaft: ShaftSpec {
                    starter: super::super::StarterSpec {
                        kind: super::super::StarterKind::Electric,
                        power_w: 1_000.0,
                        charge_j: 10_000.0,
                        mass_kg: 1.0,
                    },
                    ..ShaftSpec::default()
                },
                ..spec
            }
            .compile()
            .is_err()
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
            composition: AtmosphereComposition::anoxic(),
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
        let cruise = flight_condition(&sample, 0.9 * sample.speed_of_sound_mps).expect("cruise");
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
        let fast = flight_condition(&sample, 5.0 * sample.speed_of_sound_mps).expect("fast");
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

    #[test]
    fn suction_label_and_oxygen_composition() {
        // Static sea-level rows run on the suction floor (labeled); fast
        // rows do not. Thessa's 25%-molar (~27.4%-mass) O2 behaves exactly
        // like Earth air here (no false gating: both clear the ~7% the
        // combustor actually needs), while 5% O2 derates with the flag.
        let engine = olympus_like().compile().expect("olympus compiles");
        let sample = AtmosphereConfig::default().sample(0.0).expect("SL sample");
        let rows = analyze_airbreathing(
            &engine,
            &AtmosphereConfig::default(),
            &[0.0],
            &[0.0, 2.0],
            1.0,
        )
        .expect("analyze");
        assert!(rows[0].suction_assisted);
        assert!(!rows[1].suction_assisted);
        let thessa_config =
            AtmosphereConfig::default().with_composition(AtmosphereComposition::thessa_air());
        let rich = flight_condition(&thessa_config.sample(0.0).expect("thessa sample"), 0.0)
            .expect("thessa air");
        let earth = flight_condition(&sample, 0.0).expect("earth air");
        let p_rich = engine.operating_point(&rich, 0.8).expect("rich");
        let p_earth = engine.operating_point(&earth, 0.8).expect("earth");
        assert!(!p_rich.oxygen_limited);
        assert!((p_rich.thrust_n - p_earth.thrust_n).abs() / p_earth.thrust_n < 1e-12);
        let scarce_config = AtmosphereConfig::default().with_composition(
            AtmosphereComposition::from_mass_fractions(&[
                (GasKind::Nitrogen, 0.95),
                (GasKind::Oxygen, 0.05),
            ])
            .expect("scarce mix"),
        );
        let scarce = flight_condition(&scarce_config.sample(0.0).expect("scarce sample"), 0.0)
            .expect("scarce");
        let p_scarce = engine.operating_point(&scarce, 0.8).expect("scarce");
        assert!(p_scarce.oxygen_limited);
        assert!(p_scarce.thrust_n < p_earth.thrust_n);
    }

    #[test]
    fn jet_spool_lag_is_first_order() {
        // v5 bridge until the shaft-state machine lands: effective
        // throttle approaches the target first-order; the flight loop
        // owns the state, this owns the law.
        let tau = 5.0;
        let advanced = advance_jet_spool(0.0, 1.0, 1.0e6, tau).expect("spool");
        assert!((advanced - 1.0).abs() < 1e-9);
        let step = advance_jet_spool(0.0, 1.0, tau, tau).expect("step");
        assert!((step - (1.0 - (-1.0f64).exp())).abs() < 1e-12);
        let held = advance_jet_spool(0.4, 0.4, 10.0, tau).expect("held");
        assert!((held - 0.4).abs() < 1e-12);
        assert!(advance_jet_spool(0.0, 2.0, 1.0, tau).is_err());
        assert!(advance_jet_spool(f64::NAN, 1.0, 1.0, tau).is_err());
    }
}
