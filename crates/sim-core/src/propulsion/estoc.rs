//! ESTOC combined-cycle engines: an air-breathing turbojet path plus a
//! closed-cycle rocket path sharing intake ducting, chamber, and nozzle
//! hardware — one engine, two flow paths, valve-selected.
//!
//! This fills the gameplay niche of switchable air/rocket engines under our
//! own name. Mode discipline is strict: only one flow path is active;
//! automatic selection uses the solved air-path envelope with a Mach
//! policy band, while manual mode can select either path. The shared
//! convergent nozzle caps rocket expansion (documented): rocket mode buys
//! thrust where air fails, not orbital efficiency.
//!
//! The rocket path reuses the LOX pair thermo at its reference mixture
//! ratio, the isentropic kernel, and the Summerfield flag. Air path is a
//! [`CompiledAirbreather`]; oxidizer is LOX from vehicle tanks.

// Validity guards use `!(x > 0.0)` so NaN fails closed (repo convention).
#![allow(clippy::neg_cmp_op_on_partial_ord)]

use serde::{Deserialize, Serialize};

use super::air::{AIR_CP_J_KG_K, AirCycleConditioning, COMBUSTOR_EFFICIENCY};
use super::shaft::{JetShaftState, ShaftBalance};
use super::{
    AirOperatingPoint, AirbreathingSpec, CompiledAirbreather, FlightCondition, JetFuel,
    MASS_FIT_FEED_KG_PER_N, NozzleExitState, Propellant, PropellantThermo, PropulsionError,
    STANDARD_GRAVITY_MPS2, characteristic_velocity, mach_from_area_ratio, require_non_negative,
    require_positive, require_unit_interval, thrust_coefficient,
};

/// Rocket pump-feed pressure cap (Pa, documented).
pub const ESTOC_MAX_ROCKET_PC_PA: f64 = 35.0e6;
/// Default switch Mach band (documented design choice, not a law).
pub const ESTOC_DEFAULT_SWITCH_MACH_HI: f64 = 3.5;
pub const ESTOC_DEFAULT_SWITCH_MACH_LO: f64 = 3.2;
/// Default mode-transition thrust lag (s, documented valve/flow transient).
pub const ESTOC_DEFAULT_TRANSITION_TAU_S: f64 = 3.0;
/// Shared-structure reinforcement as a fraction of air-path dry mass
/// (documented: the ducting carries both pressure regimes).
pub const ESTOC_REINFORCEMENT_FRACTION: f64 = 0.05;

/// ESTOC operating mode (valve-selected, never blended).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum EstocMode {
    Air,
    Rocket,
    Ejector,
}

/// One regenerative air precooler: exchanger effectiveness and duty, a
/// finite-temperature wall store, coolant thermodynamics, and compressor
/// inlet limits are all authorable hardware properties.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct EstocPrecoolerSpec {
    pub maximum_heat_flow_w: f64,
    pub effectiveness: f64,
    pub maximum_compressor_inlet_temp_k: f64,
    pub pressure_recovery: f64,
    pub wall_mass_kg: f64,
    pub wall_specific_heat_j_kg_k: f64,
    pub wall_initial_temp_k: f64,
    pub wall_max_temp_k: f64,
    pub coolant_inlet_temp_k: f64,
    pub coolant_max_outlet_temp_k: f64,
    pub coolant_specific_heat_j_kg_k: f64,
    pub maximum_coolant_flow_kg_s: f64,
}

impl EstocPrecoolerSpec {
    fn validate(&self) -> Result<(), PropulsionError> {
        require_positive(self.maximum_heat_flow_w, "pre-cooler maximum heat flow")?;
        require_unit_interval(self.effectiveness, "pre-cooler effectiveness")?;
        require_positive(
            self.maximum_compressor_inlet_temp_k,
            "pre-cooler compressor-inlet temperature limit",
        )?;
        require_positive(self.pressure_recovery, "pre-cooler pressure recovery")?;
        if self.pressure_recovery > 1.0 {
            return Err(PropulsionError::InvalidSpec(
                "pre-cooler pressure recovery must be <= 1".into(),
            ));
        }
        require_positive(self.wall_mass_kg, "pre-cooler wall mass")?;
        require_positive(
            self.wall_specific_heat_j_kg_k,
            "pre-cooler wall specific heat",
        )?;
        require_positive(
            self.wall_initial_temp_k,
            "pre-cooler initial wall temperature",
        )?;
        require_positive(self.wall_max_temp_k, "pre-cooler maximum wall temperature")?;
        if self.wall_initial_temp_k > self.wall_max_temp_k {
            return Err(PropulsionError::InvalidSpec(
                "pre-cooler wall starts above its temperature limit".into(),
            ));
        }
        require_positive(
            self.coolant_inlet_temp_k,
            "pre-cooler coolant inlet temperature",
        )?;
        require_positive(
            self.coolant_max_outlet_temp_k,
            "pre-cooler coolant outlet temperature limit",
        )?;
        if self.coolant_max_outlet_temp_k <= self.coolant_inlet_temp_k {
            return Err(PropulsionError::InvalidSpec(
                "pre-cooler coolant outlet limit must exceed its inlet temperature".into(),
            ));
        }
        require_positive(
            self.coolant_specific_heat_j_kg_k,
            "pre-cooler coolant specific heat",
        )?;
        require_non_negative(
            self.maximum_coolant_flow_kg_s,
            "pre-cooler maximum coolant flow",
        )?;
        let coolant_heat_capacity_per_kg_j_kg = (self.coolant_max_outlet_temp_k
            - self.coolant_inlet_temp_k)
            * self.coolant_specific_heat_j_kg_k;
        if !(self.wall_mass_kg * self.wall_specific_heat_j_kg_k).is_finite()
            || !(self.maximum_coolant_flow_kg_s * self.coolant_specific_heat_j_kg_k).is_finite()
            || !coolant_heat_capacity_per_kg_j_kg.is_finite()
        {
            return Err(PropulsionError::InvalidSpec(
                "pre-cooler derived thermal capacities must be finite".into(),
            ));
        }
        Ok(())
    }
}

/// Geometry and measured mixing efficiency for an air-augmented rocket
/// ejector. Captured air is bounded by the inlet area and free-stream mass
/// flux; the mixed jet speed follows the rocket kinetic-energy budget.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct EstocEjectorSpec {
    pub capture_area_m2: f64,
    pub mixing_length_m: f64,
    pub mixing_efficiency: f64,
    pub structure_density_kg_m3: f64,
    pub wall_thickness_m: f64,
}

impl EstocEjectorSpec {
    fn validate(&self) -> Result<(), PropulsionError> {
        require_positive(self.capture_area_m2, "ejector capture area")?;
        require_positive(self.mixing_length_m, "ejector mixing length")?;
        require_unit_interval(self.mixing_efficiency, "ejector mixing efficiency")?;
        require_positive(self.structure_density_kg_m3, "ejector structure density")?;
        require_positive(self.wall_thickness_m, "ejector wall thickness")?;
        Ok(())
    }

    fn dry_mass_kg(self) -> f64 {
        let radius_m = (self.capture_area_m2 / std::f64::consts::PI).sqrt();
        2.0 * std::f64::consts::PI
            * radius_m
            * self.mixing_length_m
            * self.wall_thickness_m
            * self.structure_density_kg_m3
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
struct CompiledEjector {
    spec: EstocEjectorSpec,
    dry_mass_kg: f64,
}

impl JetFuel {
    /// LOX pair selecting the rocket oxidizer/fuel reference ratio.
    pub fn lox_pair(self) -> Propellant {
        match self {
            Self::Kerosene => Propellant::LoxRp1,
            Self::Methane => Propellant::LoxMethane,
            Self::Hydrogen => Propellant::LoxHydrogen,
        }
    }
}

/// ESTOC authoring: shared air path plus rocket chamber data.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EstocSpec {
    pub name: String,
    pub air: AirbreathingSpec,
    /// Dense primary propellant for both open and closed paths. `None`
    /// preserves the legacy `air.fuel` choice.
    #[serde(default)]
    pub bulk_fuel: Option<JetFuel>,
    /// Optional fuel metered through the precooler and burned downstream.
    #[serde(default)]
    pub boost_coolant_fuel: Option<JetFuel>,
    /// Optional regenerative inlet precooler.
    #[serde(default)]
    pub precooler: Option<EstocPrecoolerSpec>,
    /// Optional anoxic-atmosphere ejector path.
    #[serde(default)]
    pub ejector: Option<EstocEjectorSpec>,
    /// Rocket chamber pressure (Pa).
    pub rocket_chamber_pressure_pa: f64,
    /// Rocket throat radius (m); expansion follows from the SHARED nozzle
    /// exit (must clear it, validated).
    pub rocket_throat_radius_m: f64,
    /// Oxidizer/fuel ratio override; `None` = LOX-pair reference.
    pub oxidizer_fuel_ratio: Option<f64>,
    /// Switch band overrides (Mach); `None` = defaults.
    pub switch_mach_hi: Option<f64>,
    pub switch_mach_lo: Option<f64>,
    /// Transition lag override (s).
    pub transition_tau_s: Option<f64>,
}

/// Hangar-compiled ESTOC: shared air core plus rocket chamber data.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CompiledEstoc {
    pub name: String,
    pub air: CompiledAirbreather,
    pub bulk_fuel: JetFuel,
    pub boost_coolant_fuel: Option<JetFuel>,
    pub precooler: Option<EstocPrecoolerSpec>,
    ejector: Option<CompiledEjector>,
    pub rocket_chamber_pressure_pa: f64,
    pub rocket_throat_area_m2: f64,
    pub rocket_expansion_ratio: f64,
    pub oxidizer_fuel_ratio: f64,
    pub rocket_gamma: f64,
    pub rocket_chamber_temp_k: f64,
    pub rocket_gas_constant: f64,
    pub rocket_c_star_mps: f64,
    pub rocket_exit_mach: f64,
    pub rocket_exit_temp_k: f64,
    pub rocket_exhaust_velocity_mps: f64,
    pub rocket_thrust_sl_n: f64,
    pub rocket_thrust_vac_n: f64,
    pub rocket_isp_sl_s: f64,
    pub rocket_isp_vac_s: f64,
    pub rocket_flow_kg_s: f64,
    pub rocket_extra_mass_kg: f64,
    pub dry_mass_kg: f64,
    pub feed_pressure_required_pa: f64,
    pub switch_mach_hi: f64,
    pub switch_mach_lo: f64,
    pub transition_tau_s: f64,
    pub min_throttle: f64,
}

/// Transition state: the full smoothed flow/thermodynamic snapshot the
/// runtime threads across ticks (`prev`) so mode changes evolve every
/// channel — thrust, fuel, oxidizer, air, exhaust state — instead of
/// snapping one scalar while the rest jump.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct EstocTransient {
    pub thrust_n: f64,
    pub fuel_flow_kg_s: f64,
    #[serde(default)]
    pub bulk_fuel_flow_kg_s: f64,
    #[serde(default)]
    pub boost_fuel_flow_kg_s: f64,
    pub oxidizer_flow_kg_s: f64,
    pub air_flow_kg_s: f64,
    pub exhaust_temp_k: f64,
    pub exhaust_velocity_mps: f64,
    pub exit_pressure_pa: f64,
    pub exit_mach: f64,
    #[serde(default)]
    pub compressor_inlet_total_temp_k: f64,
    #[serde(default)]
    pub precooler_heat_flow_w: f64,
    #[serde(default)]
    pub precooler_wall_heat_flow_w: f64,
    #[serde(default)]
    pub precooler_wall_temp_k: f64,
    #[serde(default)]
    pub coolant_outlet_temp_k: f64,
}

/// One ESTOC operating point (single active mode).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EstocPoint {
    pub mode: EstocMode,
    pub thrust_n: f64,
    pub fuel_flow_kg_s: f64,
    #[serde(default)]
    pub bulk_fuel_flow_kg_s: f64,
    #[serde(default)]
    pub boost_fuel_flow_kg_s: f64,
    /// LOX flow in rocket mode, 0 in air mode.
    pub oxidizer_flow_kg_s: f64,
    pub air_flow_kg_s: f64,
    /// Isp over all onboard propellant (fuel + oxidizer).
    pub isp_total_s: f64,
    pub exhaust_temp_k: f64,
    pub exhaust_velocity_mps: f64,
    pub exit_pressure_pa: f64,
    pub exit_mach: f64,
    #[serde(default)]
    pub compressor_inlet_total_temp_k: f64,
    #[serde(default)]
    pub precooler_heat_flow_w: f64,
    #[serde(default)]
    pub precooler_wall_heat_flow_w: f64,
    #[serde(default)]
    pub precooler_wall_temp_k: f64,
    #[serde(default)]
    pub coolant_outlet_temp_k: f64,
    #[serde(default)]
    pub precooler_saturated: bool,
}

impl EstocSpec {
    /// Validate authoring values (NaN fails closed).
    pub fn validate(&self) -> Result<(), PropulsionError> {
        if self.name.trim().is_empty() {
            return Err(PropulsionError::InvalidSpec(
                "engine name must not be empty".into(),
            ));
        }
        if !self.air.cycle.has_shaft() {
            return Err(PropulsionError::InvalidSpec(
                "combined cycle builds on a turbomachinery air path, not a ramjet/scramjet".into(),
            ));
        }
        if self.boost_coolant_fuel.is_some() && self.precooler.is_none() {
            return Err(PropulsionError::InvalidSpec(
                "boost/coolant fuel requires an ESTOC pre-cooler".into(),
            ));
        }
        if let Some(precooler) = self.precooler {
            precooler.validate()?;
        }
        if let Some(ejector) = self.ejector {
            ejector.validate()?;
            if !ejector.dry_mass_kg().is_finite() {
                return Err(PropulsionError::InvalidSpec(
                    "ejector geometry produces non-finite mass".into(),
                ));
            }
        }
        require_positive(self.rocket_chamber_pressure_pa, "rocket chamber pressure")?;
        if self.rocket_chamber_pressure_pa > ESTOC_MAX_ROCKET_PC_PA {
            return Err(PropulsionError::UnsupportedCombination(format!(
                "rocket pump feed caps at {:.1} MPa",
                ESTOC_MAX_ROCKET_PC_PA / 1.0e6
            )));
        }
        require_positive(self.rocket_throat_radius_m, "rocket throat radius")?;
        if let Some(ratio) = self.oxidizer_fuel_ratio
            && (!ratio.is_finite() || ratio <= 0.0)
        {
            return Err(PropulsionError::InvalidSpec(
                "oxidizer/fuel ratio must be finite and > 0".into(),
            ));
        }
        let switch_mach_hi = self.switch_mach_hi.unwrap_or(ESTOC_DEFAULT_SWITCH_MACH_HI);
        let switch_mach_lo = self.switch_mach_lo.unwrap_or(ESTOC_DEFAULT_SWITCH_MACH_LO);
        if !switch_mach_hi.is_finite()
            || !switch_mach_lo.is_finite()
            || switch_mach_hi <= switch_mach_lo
            || switch_mach_lo <= 0.0
        {
            return Err(PropulsionError::InvalidSpec(
                "switch band needs 0 < lo < hi".into(),
            ));
        }
        if let Some(tau) = self.transition_tau_s {
            require_positive(tau, "transition tau")?;
        }
        Ok(())
    }

    /// Hangar compile: air path first (sizes the shared nozzle), then the
    /// rocket chamber against the shared exit.
    pub fn compile(&self) -> Result<CompiledEstoc, PropulsionError> {
        self.validate()?;
        let bulk_fuel = self.bulk_fuel.unwrap_or(self.air.fuel);
        let mut air_spec = self.air.clone();
        air_spec.fuel = bulk_fuel;
        let air = air_spec.compile()?;
        let pair = bulk_fuel.lox_pair();
        let of_ratio = self
            .oxidizer_fuel_ratio
            .or(pair.reference_mixture_ratio())
            .expect("LOX pairs carry a reference ratio");
        let thermo = pair.thermo();
        let c_star = characteristic_velocity(&thermo);
        let throat_area_m2 =
            std::f64::consts::PI * self.rocket_throat_radius_m * self.rocket_throat_radius_m;
        let expansion = air.exit_area_core_m2 / throat_area_m2;
        if !(expansion >= 1.0) {
            return Err(PropulsionError::UnsupportedCombination(
                "rocket throat exceeds the shared nozzle exit; shrink the throat".into(),
            ));
        }
        let exit_mach = mach_from_area_ratio(thermo.gamma, expansion)?;
        let exit_pressure_ratio = (1.0 + (thermo.gamma - 1.0) / 2.0 * exit_mach * exit_mach)
            .powf(-thermo.gamma / (thermo.gamma - 1.0));
        let exit_temp_k =
            thermo.chamber_temp_k / (1.0 + (thermo.gamma - 1.0) / 2.0 * exit_mach * exit_mach);
        let exhaust_velocity_mps =
            exit_mach * (thermo.gamma * thermo.gas_constant_j_kg_k * exit_temp_k).sqrt();
        let exit = super::NozzleExitState {
            exit_mach,
            exit_pressure_ratio,
            exit_temp_k,
            exhaust_velocity_mps,
        };
        let divergence = 0.99;
        let mdot = self.rocket_chamber_pressure_pa * throat_area_m2 / c_star;
        let thrust_sl_n = thrust_coefficient(
            &thermo,
            self.rocket_chamber_pressure_pa,
            &exit,
            expansion,
            101_325.0,
            divergence,
        ) * self.rocket_chamber_pressure_pa
            * throat_area_m2;
        let thrust_vac_n = thrust_coefficient(
            &thermo,
            self.rocket_chamber_pressure_pa,
            &exit,
            expansion,
            0.0,
            divergence,
        ) * self.rocket_chamber_pressure_pa
            * throat_area_m2;
        // Rocket-side hardware only: chamber/injector/feed share of the
        // duct plus reinforcement of the shared structure (the nozzle,
        // ducting, and mounts already book in the air path — counted once).
        let rocket_extra_mass_kg =
            MASS_FIT_FEED_KG_PER_N * thrust_vac_n + ESTOC_REINFORCEMENT_FRACTION * air.dry_mass_kg;
        let ejector = self.ejector.map(|spec| CompiledEjector {
            spec,
            dry_mass_kg: spec.dry_mass_kg(),
        });
        let precooler_mass_kg = self.precooler.map_or(0.0, |spec| spec.wall_mass_kg);
        let ejector_mass_kg = ejector.map_or(0.0, |compiled| compiled.dry_mass_kg);
        let dry_mass_kg =
            air.dry_mass_kg + rocket_extra_mass_kg + precooler_mass_kg + ejector_mass_kg;
        Ok(CompiledEstoc {
            name: self.name.clone(),
            air,
            bulk_fuel,
            boost_coolant_fuel: self.boost_coolant_fuel,
            precooler: self.precooler,
            ejector,
            rocket_chamber_pressure_pa: self.rocket_chamber_pressure_pa,
            rocket_throat_area_m2: throat_area_m2,
            rocket_expansion_ratio: expansion,
            oxidizer_fuel_ratio: of_ratio,
            rocket_gamma: thermo.gamma,
            rocket_chamber_temp_k: thermo.chamber_temp_k,
            rocket_gas_constant: thermo.gas_constant_j_kg_k,
            rocket_c_star_mps: c_star,
            rocket_exit_mach: exit_mach,
            rocket_exit_temp_k: exit_temp_k,
            rocket_exhaust_velocity_mps: exhaust_velocity_mps,
            rocket_thrust_sl_n: thrust_sl_n,
            rocket_thrust_vac_n: thrust_vac_n,
            rocket_isp_sl_s: thrust_sl_n / (mdot * STANDARD_GRAVITY_MPS2),
            rocket_isp_vac_s: thrust_vac_n / (mdot * STANDARD_GRAVITY_MPS2),
            rocket_flow_kg_s: mdot,
            rocket_extra_mass_kg,
            dry_mass_kg,
            feed_pressure_required_pa: self.rocket_chamber_pressure_pa * 1.2,
            switch_mach_hi: self.switch_mach_hi.unwrap_or(ESTOC_DEFAULT_SWITCH_MACH_HI),
            switch_mach_lo: self.switch_mach_lo.unwrap_or(ESTOC_DEFAULT_SWITCH_MACH_LO),
            transition_tau_s: self
                .transition_tau_s
                .unwrap_or(ESTOC_DEFAULT_TRANSITION_TAU_S),
            min_throttle: 0.4,
        })
    }
}

impl CompiledEstoc {
    /// Mode selector (steady-analyzer convention: the air path is
    /// considered commanded — see [`Self::select_mode_inner`] for the
    /// runtime form).
    pub fn select_mode(
        &self,
        condition: &FlightCondition,
        air_point: &AirOperatingPoint,
        last_mode: EstocMode,
        manual: Option<EstocMode>,
    ) -> EstocMode {
        self.select_mode_inner(condition, air_point, last_mode, manual, true)
    }

    /// Manual selection wins. Automatic selection uses the solved thermal,
    /// combustion, drive, and net-thrust envelope; Mach hysteresis only
    /// delays a return from rocket to a still-viable air path.
    fn select_mode_inner(
        &self,
        condition: &FlightCondition,
        air_point: &AirOperatingPoint,
        last_mode: EstocMode,
        manual: Option<EstocMode>,
        air_commanded: bool,
    ) -> EstocMode {
        if let Some(mode) = manual {
            return mode;
        }
        if !air_commanded {
            return last_mode;
        }
        let oxygen_available = condition
            .composition
            .mass_fraction(crate::atmosphere::GasKind::Oxygen)
            > 1e-9;
        let within_precooler_limit = self.precooler.is_none_or(|precooler| {
            air_point.compressor_inlet_total_temp_k
                <= precooler.maximum_compressor_inlet_temp_k * (1.0 + 1e-9)
        });
        let air_viable = condition.ambient_pa > 0.0
            && oxygen_available
            && air_point.air_flow_kg_s > 0.0
            && air_point.lit
            && !air_point.drive_limited
            && !air_point.oxygen_limited
            && within_precooler_limit
            && air_point.thrust_n > 0.0;
        if air_viable {
            if condition.mach >= self.switch_mach_hi
                || (last_mode == EstocMode::Rocket && condition.mach > self.switch_mach_lo)
            {
                return EstocMode::Rocket;
            }
            return EstocMode::Air;
        }
        if !oxygen_available
            && self.ejector.is_some()
            && self.ejector_capture_flow_kg_s(condition) > 0.0
        {
            return EstocMode::Ejector;
        }
        EstocMode::Rocket
    }

    fn ejector_capture_flow_kg_s(&self, condition: &FlightCondition) -> f64 {
        let Some(ejector) = self.ejector else {
            return 0.0;
        };
        let density = condition.ambient_pa / (287.0 * condition.ambient_temp_k);
        density * ejector.spec.capture_area_m2 * condition.airspeed_mps
    }

    pub(super) fn conditioned_air_point(
        &self,
        condition: &FlightCondition,
        throttle: f64,
        shaft: JetShaftState,
        prev: Option<&EstocTransient>,
        dt_s: f64,
    ) -> Result<(AirOperatingPoint, f64, f64, ShaftBalance), PropulsionError> {
        let Some(precooler) = self.precooler else {
            let (point, balance) =
                self.air
                    .operating_point_at_spool(condition, throttle, shaft.spool_n, shaft.lit)?;
            return Ok((point, 0.0, 0.0, balance));
        };

        let (baseline, baseline_balance) =
            self.air
                .operating_point_at_spool(condition, throttle, shaft.spool_n, shaft.lit)?;
        let t_ram = condition.ambient_temp_k
            * (1.0 + (super::AIR_GAMMA - 1.0) * 0.5 * condition.mach * condition.mach);
        let wall_capacity_j_k = precooler.wall_mass_kg * precooler.wall_specific_heat_j_kg_k;
        let prior_wall_temp_k = prev
            .map(|state| state.precooler_wall_temp_k)
            .filter(|temperature| temperature.is_finite() && *temperature > 0.0)
            .unwrap_or(precooler.wall_initial_temp_k)
            .clamp(precooler.wall_initial_temp_k, precooler.wall_max_temp_k);
        let wall_headroom_w = if dt_s.is_finite() && dt_s > 0.0 {
            wall_capacity_j_k * (precooler.wall_max_temp_k - prior_wall_temp_k) / dt_s
        } else {
            0.0
        };
        let air_capacity_w_k = baseline.air_flow_kg_s * AIR_CP_J_KG_K;
        if throttle <= 0.0 || air_capacity_w_k <= 0.0 || t_ram <= 0.0 {
            return Ok((
                baseline,
                prior_wall_temp_k,
                prev.map_or(precooler.coolant_inlet_temp_k, |state| {
                    if state.coolant_outlet_temp_k.is_finite() && state.coolant_outlet_temp_k > 0.0
                    {
                        state.coolant_outlet_temp_k
                    } else {
                        precooler.coolant_inlet_temp_k
                    }
                }),
                baseline_balance,
            ));
        }

        let coolant_capacity_w_k = if self.boost_coolant_fuel.is_some() {
            precooler.maximum_coolant_flow_kg_s * precooler.coolant_specific_heat_j_kg_k
        } else {
            0.0
        };
        let wall_capacity_rate_w_k = if dt_s.is_finite() && dt_s > 0.0 {
            wall_capacity_j_k / dt_s
        } else {
            0.0
        };
        let cold_side_capacity_w_k = coolant_capacity_w_k + wall_capacity_rate_w_k;
        let cold_side_temp_k = if coolant_capacity_w_k > 0.0 {
            precooler.coolant_inlet_temp_k.min(prior_wall_temp_k)
        } else {
            prior_wall_temp_k
        };
        let min_capacity_w_k = air_capacity_w_k.min(cold_side_capacity_w_k);
        let exchanger_limit_w =
            precooler.effectiveness * min_capacity_w_k * (t_ram - cold_side_temp_k).max(0.0);
        let required_to_temperature_limit_w =
            air_capacity_w_k * (t_ram - precooler.maximum_compressor_inlet_temp_k).max(0.0);
        let exchanger_heat_limit_w = exchanger_limit_w
            .min(precooler.maximum_heat_flow_w)
            .min(required_to_temperature_limit_w);
        let coolant_delta_temp_k =
            precooler.coolant_max_outlet_temp_k - precooler.coolant_inlet_temp_k;
        let tit_k = self.air.turbine_inlet_temp_k * (0.55 + 0.45 * throttle);
        let boost_fuel = self.boost_coolant_fuel.unwrap_or(self.bulk_fuel);
        let core_flow_kg_s = baseline.air_flow_kg_s / (1.0 + self.air.bypass_ratio);
        let compressor_ratio =
            1.0 + (self.air.compressor_ratio - 1.0) * shaft.spool_n.clamp(0.0, 1.0).powi(2);
        let compressor_tau = compressor_ratio.powf(
            (super::AIR_GAMMA - 1.0) / (super::AIR_GAMMA * super::air::COMPRESSOR_POLY_EFFICIENCY),
        );
        let coolant_enthalpy_per_kg_j_kg =
            precooler.coolant_specific_heat_j_kg_k * coolant_delta_temp_k;
        let partition_heat = |total_heat_w: f64| {
            let compressor_temp_k = (t_ram - total_heat_w / air_capacity_w_k) * compressor_tau;
            let heat_before_fuel_w =
                (core_flow_kg_s * AIR_CP_J_KG_K * (tit_k - compressor_temp_k)).max(0.0);
            let coolant_energy_cap_w = if self.boost_coolant_fuel.is_some() {
                heat_before_fuel_w * coolant_enthalpy_per_kg_j_kg
                    / (COMBUSTOR_EFFICIENCY * boost_fuel.properties().0
                        + coolant_enthalpy_per_kg_j_kg)
            } else {
                0.0
            };
            let coolant_heat_w = total_heat_w
                .min(coolant_capacity_w_k * coolant_delta_temp_k)
                .min(coolant_energy_cap_w);
            let wall_heat_w = (total_heat_w - coolant_heat_w)
                .max(0.0)
                .min(wall_headroom_w);
            (coolant_heat_w, wall_heat_w)
        };
        let is_feasible = |total_heat_w: f64| {
            let (coolant_heat_w, wall_heat_w) = partition_heat(total_heat_w);
            total_heat_w <= coolant_heat_w + wall_heat_w + 1e-12 * total_heat_w.max(1.0)
        };
        let mut low_heat_w = 0.0;
        let mut high_heat_w = exchanger_heat_limit_w;
        if is_feasible(high_heat_w) {
            low_heat_w = high_heat_w;
        } else {
            for _ in 0..48 {
                let midpoint_heat_w = 0.5 * (low_heat_w + high_heat_w);
                if is_feasible(midpoint_heat_w) {
                    low_heat_w = midpoint_heat_w;
                } else {
                    high_heat_w = midpoint_heat_w;
                }
            }
        }
        let mut heat_flow_w = low_heat_w;
        let (coolant_heat_w, wall_heat_w) = partition_heat(heat_flow_w);
        heat_flow_w = coolant_heat_w + wall_heat_w;
        let boost_flow_kg_s = if coolant_heat_w > 0.0 {
            coolant_heat_w / (precooler.coolant_specific_heat_j_kg_k * coolant_delta_temp_k)
        } else {
            0.0
        }
        .min(precooler.maximum_coolant_flow_kg_s);
        let compressor_inlet_temp_k = (t_ram - heat_flow_w / air_capacity_w_k).max(1.0);
        let conditioning = AirCycleConditioning {
            compressor_inlet_total_temp_k: compressor_inlet_temp_k,
            compressor_pressure_recovery: precooler.pressure_recovery,
            boost_fuel,
            boost_fuel_flow_kg_s: boost_flow_kg_s,
            coolant_heat_flow_w: coolant_heat_w,
            wall_heat_flow_w: wall_heat_w,
        };
        let (point, balance) = if boost_flow_kg_s > 0.0 || wall_heat_w > 0.0 {
            self.air.operating_point_at_spool_conditioned(
                condition,
                throttle,
                shaft.spool_n,
                shaft.lit,
                conditioning,
            )?
        } else {
            (baseline, baseline_balance)
        };
        // The nozzle may rescale all matched flows. Advance the wall and
        // report coolant outlet from the heat actually used by that matched
        // operating point, not the pre-match exchanger request.
        let matched_wall_heat_w = point.precooler_wall_heat_flow_w;
        let matched_coolant_heat_w = (point.precooler_heat_flow_w - matched_wall_heat_w).max(0.0);
        let matched_coolant_flow_kg_s = point.boost_fuel_flow_kg_s;
        let coolant_outlet_temp_k = if matched_coolant_flow_kg_s > 0.0 {
            precooler.coolant_inlet_temp_k
                + matched_coolant_heat_w
                    / (matched_coolant_flow_kg_s * precooler.coolant_specific_heat_j_kg_k)
        } else {
            precooler.coolant_inlet_temp_k
        };
        let next_wall_temp_k = if dt_s.is_finite() && dt_s > 0.0 {
            (prior_wall_temp_k + matched_wall_heat_w * dt_s / wall_capacity_j_k)
                .min(precooler.wall_max_temp_k)
        } else {
            prior_wall_temp_k
        };
        Ok((point, next_wall_temp_k, coolant_outlet_temp_k, balance))
    }

    /// Steady rocket-mode point at throttle and ambient (chamber pressure
    /// scales linearly, same documented deep-throttle assumption as
    /// liquids; oxidizer/fuel split by the OF ratio).
    fn rocket_point(&self, throttle: f64, ambient_pa: f64) -> Result<EstocPoint, PropulsionError> {
        let thermo = PropellantThermo {
            gamma: self.rocket_gamma,
            chamber_temp_k: self.rocket_chamber_temp_k,
            gas_constant_j_kg_k: self.rocket_gas_constant,
            bulk_density_kg_m3: 0.0,
            characteristic_length_m: 0.0,
            reference_c_star_mps: 0.0,
        };
        let exit_ratio = (1.0
            + (thermo.gamma - 1.0) / 2.0 * self.rocket_exit_mach * self.rocket_exit_mach)
            .powf(-thermo.gamma / (thermo.gamma - 1.0));
        // Geometry-frozen by construction (matches the compiled ratio).
        let exit = NozzleExitState {
            exit_mach: self.rocket_exit_mach,
            exit_pressure_ratio: exit_ratio,
            exit_temp_k: self.rocket_exit_temp_k,
            exhaust_velocity_mps: self.rocket_exhaust_velocity_mps,
        };
        let chamber_pa = self.rocket_chamber_pressure_pa * throttle;
        let cf = thrust_coefficient(
            &thermo,
            chamber_pa,
            &exit,
            self.rocket_expansion_ratio,
            ambient_pa,
            0.99,
        );
        let thrust_n = cf * chamber_pa * self.rocket_throat_area_m2;
        let flow_kg_s = self.rocket_flow_kg_s * throttle;
        let fuel_flow_kg_s = flow_kg_s / (1.0 + self.oxidizer_fuel_ratio);
        let oxidizer_flow_kg_s = flow_kg_s - fuel_flow_kg_s;
        let exit_pressure_pa = exit_ratio * chamber_pa;
        Ok(EstocPoint {
            mode: EstocMode::Rocket,
            thrust_n,
            fuel_flow_kg_s,
            bulk_fuel_flow_kg_s: fuel_flow_kg_s,
            boost_fuel_flow_kg_s: 0.0,
            oxidizer_flow_kg_s,
            air_flow_kg_s: 0.0,
            isp_total_s: if flow_kg_s > 0.0 {
                thrust_n / (flow_kg_s * STANDARD_GRAVITY_MPS2)
            } else {
                0.0
            },
            exhaust_temp_k: self.rocket_exit_temp_k,
            exhaust_velocity_mps: self.rocket_exhaust_velocity_mps,
            exit_pressure_pa,
            exit_mach: self.rocket_exit_mach,
            compressor_inlet_total_temp_k: 0.0,
            precooler_heat_flow_w: 0.0,
            precooler_wall_heat_flow_w: 0.0,
            precooler_wall_temp_k: 0.0,
            coolant_outlet_temp_k: 0.0,
            precooler_saturated: false,
        })
    }

    fn ejector_point(
        &self,
        throttle: f64,
        condition: &FlightCondition,
    ) -> Result<EstocPoint, PropulsionError> {
        let ejector = self.ejector.ok_or_else(|| {
            PropulsionError::InvalidCommand("ESTOC ejector mode is not installed".into())
        })?;
        let entrained_air_kg_s = self.ejector_capture_flow_kg_s(condition);
        if entrained_air_kg_s <= 0.0 {
            return Err(PropulsionError::InvalidCommand(
                "ESTOC ejector needs nonzero captured atmospheric flow".into(),
            ));
        }
        let mut point = self.rocket_point(throttle, condition.ambient_pa)?;
        let motive_flow_kg_s = point.fuel_flow_kg_s + point.oxidizer_flow_kg_s;
        let mixed_flow_kg_s = motive_flow_kg_s + entrained_air_kg_s;
        let motive_kinetic_power_w = 0.5 * motive_flow_kg_s * point.exhaust_velocity_mps.powi(2);
        let mixed_exhaust_velocity_mps =
            (2.0 * motive_kinetic_power_w * ejector.spec.mixing_efficiency / mixed_flow_kg_s)
                .sqrt();
        // Preserve the rocket nozzle's pressure term, then close the mixed
        // stream momentum against captured air's incoming momentum.
        let pressure_thrust_n = point.thrust_n - motive_flow_kg_s * point.exhaust_velocity_mps;
        point.mode = EstocMode::Ejector;
        point.thrust_n = mixed_flow_kg_s * mixed_exhaust_velocity_mps
            - entrained_air_kg_s * condition.airspeed_mps
            + pressure_thrust_n;
        point.air_flow_kg_s = entrained_air_kg_s;
        point.isp_total_s = if motive_flow_kg_s > 0.0 {
            point.thrust_n / (motive_flow_kg_s * STANDARD_GRAVITY_MPS2)
        } else {
            0.0
        };
        point.exhaust_velocity_mps = mixed_exhaust_velocity_mps;
        point.exit_pressure_pa = condition.ambient_pa;
        point.exit_mach = mixed_exhaust_velocity_mps
            / (super::AIR_GAMMA * 287.0 * point.exhaust_temp_k.max(1.0)).sqrt();
        Ok(point)
    }

    /// Operating point with mode discipline and a self-consistent
    /// transition state: the full flow/thermodynamic snapshot approaches
    /// the selected mode's target first-order over `transition_tau_s`
    /// (valve/flow transient, documented), and Isp is recomputed from the
    /// smoothed flows so bookkeeping always holds. Thread the returned
    /// transient and `point.mode` back as `prev`/`last_mode`; `None` starts
    /// fresh at the exact target (editor/analyzer convention).
    ///
    /// Mode contract: manual selection always wins — including a manual
    /// `Air` in vacuum, which is honored and flames out cleanly (zero
    /// thrust, cause flags) rather than silently switching.
    ///
    /// The air path is evaluated at the runtime shaft state (`shaft`):
    /// spool speed schedules the cycle and `lit` gates ignition, so a
    /// windmilling or stopped core reports its real (unlit) state and
    /// mode selection falls back to rocket (or the installed ejector in
    /// an anoxic atmosphere) when air is commanded but dead. Analyzer callers
    /// pass a steady state ([`JetShaftState::running`] or the solved
    /// equilibrium).
    #[allow(clippy::too_many_arguments)]
    pub fn operating_point(
        &self,
        condition: &FlightCondition,
        throttle: f64,
        manual: Option<EstocMode>,
        last_mode: EstocMode,
        prev: Option<&EstocTransient>,
        dt_s: f64,
        shaft: JetShaftState,
    ) -> Result<(EstocPoint, EstocTransient), PropulsionError> {
        condition.validate()?;
        if !throttle.is_finite() || !(0.0..=1.0).contains(&throttle) {
            return Err(PropulsionError::InvalidCommand(
                "throttle must be finite in [0, 1]".into(),
            ));
        }
        if dt_s.is_nan() || dt_s < 0.0 {
            return Err(PropulsionError::InvalidCommand(
                "transition dt must be >= 0 or +infinity".into(),
            ));
        }
        if throttle == 0.0 {
            let air_point = self.air.operating_point(condition, 0.0)?;
            let mode = self.select_mode_inner(condition, &air_point, last_mode, manual, false);
            let wall_temp = self.precooler.map_or(0.0, |precooler| {
                prev.map_or(precooler.wall_initial_temp_k, |state| {
                    if state.precooler_wall_temp_k > 0.0 {
                        state.precooler_wall_temp_k
                    } else {
                        precooler.wall_initial_temp_k
                    }
                })
            });
            let coolant_temp = self.precooler.map_or(0.0, |precooler| {
                prev.map_or(precooler.coolant_inlet_temp_k, |state| {
                    if state.coolant_outlet_temp_k > 0.0 {
                        state.coolant_outlet_temp_k
                    } else {
                        precooler.coolant_inlet_temp_k
                    }
                })
            });
            let transient = EstocTransient {
                thrust_n: 0.0,
                fuel_flow_kg_s: 0.0,
                bulk_fuel_flow_kg_s: 0.0,
                boost_fuel_flow_kg_s: 0.0,
                oxidizer_flow_kg_s: 0.0,
                air_flow_kg_s: 0.0,
                exhaust_temp_k: condition.ambient_temp_k,
                exhaust_velocity_mps: 0.0,
                exit_pressure_pa: condition.ambient_pa,
                exit_mach: 0.0,
                compressor_inlet_total_temp_k: air_point.compressor_inlet_total_temp_k,
                precooler_heat_flow_w: 0.0,
                precooler_wall_heat_flow_w: 0.0,
                precooler_wall_temp_k: wall_temp,
                coolant_outlet_temp_k: coolant_temp,
            };
            return Ok((
                EstocPoint {
                    mode,
                    thrust_n: 0.0,
                    fuel_flow_kg_s: 0.0,
                    bulk_fuel_flow_kg_s: 0.0,
                    boost_fuel_flow_kg_s: 0.0,
                    oxidizer_flow_kg_s: 0.0,
                    air_flow_kg_s: 0.0,
                    isp_total_s: 0.0,
                    exhaust_temp_k: condition.ambient_temp_k,
                    exhaust_velocity_mps: 0.0,
                    exit_pressure_pa: condition.ambient_pa,
                    exit_mach: 0.0,
                    compressor_inlet_total_temp_k: air_point.compressor_inlet_total_temp_k,
                    precooler_heat_flow_w: 0.0,
                    precooler_wall_heat_flow_w: 0.0,
                    precooler_wall_temp_k: wall_temp,
                    coolant_outlet_temp_k: coolant_temp,
                    precooler_saturated: false,
                },
                transient,
            ));
        }
        let (air_point, cooled_wall_temp_k, coolant_outlet_temp_k, _) =
            self.conditioned_air_point(condition, throttle, shaft, prev, dt_s)?;
        let precooler_saturated = self.precooler.is_some_and(|precooler| {
            air_point.compressor_inlet_total_temp_k
                > precooler.maximum_compressor_inlet_temp_k * (1.0 + 1e-9)
        });
        let mode = self.select_mode(condition, &air_point, last_mode, manual);
        let idle_wall_temp_k = prev.map_or_else(
            || {
                self.precooler
                    .map_or(0.0, |precooler| precooler.wall_initial_temp_k)
            },
            |state| state.precooler_wall_temp_k,
        );
        let idle_coolant_temp_k = prev.map_or_else(
            || {
                self.precooler
                    .map_or(0.0, |precooler| precooler.coolant_inlet_temp_k)
            },
            |state| state.coolant_outlet_temp_k,
        );
        let target = match mode {
            EstocMode::Air => EstocTransient {
                thrust_n: air_point.thrust_n.max(0.0),
                fuel_flow_kg_s: air_point.fuel_flow_kg_s,
                bulk_fuel_flow_kg_s: air_point.bulk_fuel_flow_kg_s,
                boost_fuel_flow_kg_s: air_point.boost_fuel_flow_kg_s,
                oxidizer_flow_kg_s: 0.0,
                air_flow_kg_s: air_point.air_flow_kg_s,
                exhaust_temp_k: air_point.exhaust_temp_k,
                exhaust_velocity_mps: air_point.exhaust_velocity_mps,
                exit_pressure_pa: air_point.exit_pressure_pa,
                exit_mach: air_point.exit_mach,
                compressor_inlet_total_temp_k: air_point.compressor_inlet_total_temp_k,
                precooler_heat_flow_w: air_point.precooler_heat_flow_w,
                precooler_wall_heat_flow_w: air_point.precooler_wall_heat_flow_w,
                precooler_wall_temp_k: cooled_wall_temp_k,
                coolant_outlet_temp_k,
            },
            EstocMode::Rocket | EstocMode::Ejector => {
                let rocket = if mode == EstocMode::Ejector {
                    self.ejector_point(throttle, condition)?
                } else {
                    self.rocket_point(throttle, condition.ambient_pa)?
                };
                EstocTransient {
                    thrust_n: rocket.thrust_n,
                    fuel_flow_kg_s: rocket.fuel_flow_kg_s,
                    bulk_fuel_flow_kg_s: rocket.bulk_fuel_flow_kg_s,
                    boost_fuel_flow_kg_s: rocket.boost_fuel_flow_kg_s,
                    oxidizer_flow_kg_s: rocket.oxidizer_flow_kg_s,
                    air_flow_kg_s: rocket.air_flow_kg_s,
                    exhaust_temp_k: rocket.exhaust_temp_k,
                    exhaust_velocity_mps: rocket.exhaust_velocity_mps,
                    exit_pressure_pa: rocket.exit_pressure_pa,
                    exit_mach: rocket.exit_mach,
                    compressor_inlet_total_temp_k: air_point.compressor_inlet_total_temp_k,
                    precooler_heat_flow_w: 0.0,
                    precooler_wall_heat_flow_w: 0.0,
                    precooler_wall_temp_k: idle_wall_temp_k,
                    coolant_outlet_temp_k: idle_coolant_temp_k,
                }
            }
        };
        let smoothed = match prev {
            None => target,
            Some(previous) => {
                let alpha = 1.0 - (-dt_s / self.transition_tau_s).exp();
                let lerp = |old: f64, new: f64| old + (new - old) * alpha;
                EstocTransient {
                    thrust_n: lerp(previous.thrust_n, target.thrust_n),
                    fuel_flow_kg_s: lerp(previous.fuel_flow_kg_s, target.fuel_flow_kg_s),
                    bulk_fuel_flow_kg_s: lerp(
                        previous.bulk_fuel_flow_kg_s,
                        target.bulk_fuel_flow_kg_s,
                    ),
                    boost_fuel_flow_kg_s: lerp(
                        previous.boost_fuel_flow_kg_s,
                        target.boost_fuel_flow_kg_s,
                    ),
                    oxidizer_flow_kg_s: lerp(
                        previous.oxidizer_flow_kg_s,
                        target.oxidizer_flow_kg_s,
                    ),
                    air_flow_kg_s: lerp(previous.air_flow_kg_s, target.air_flow_kg_s),
                    exhaust_temp_k: lerp(previous.exhaust_temp_k, target.exhaust_temp_k),
                    exhaust_velocity_mps: lerp(
                        previous.exhaust_velocity_mps,
                        target.exhaust_velocity_mps,
                    ),
                    exit_pressure_pa: lerp(previous.exit_pressure_pa, target.exit_pressure_pa),
                    exit_mach: lerp(previous.exit_mach, target.exit_mach),
                    compressor_inlet_total_temp_k: lerp(
                        previous.compressor_inlet_total_temp_k,
                        target.compressor_inlet_total_temp_k,
                    ),
                    precooler_heat_flow_w: lerp(
                        previous.precooler_heat_flow_w,
                        target.precooler_heat_flow_w,
                    ),
                    precooler_wall_heat_flow_w: lerp(
                        previous.precooler_wall_heat_flow_w,
                        target.precooler_wall_heat_flow_w,
                    ),
                    precooler_wall_temp_k: target.precooler_wall_temp_k,
                    coolant_outlet_temp_k: target.coolant_outlet_temp_k,
                }
            }
        };
        let propellant_flow = smoothed.fuel_flow_kg_s + smoothed.oxidizer_flow_kg_s;
        Ok((
            EstocPoint {
                mode,
                thrust_n: smoothed.thrust_n,
                fuel_flow_kg_s: smoothed.fuel_flow_kg_s,
                bulk_fuel_flow_kg_s: smoothed.bulk_fuel_flow_kg_s,
                boost_fuel_flow_kg_s: smoothed.boost_fuel_flow_kg_s,
                oxidizer_flow_kg_s: smoothed.oxidizer_flow_kg_s,
                air_flow_kg_s: smoothed.air_flow_kg_s,
                isp_total_s: if propellant_flow > 0.0 {
                    smoothed.thrust_n / (propellant_flow * STANDARD_GRAVITY_MPS2)
                } else {
                    0.0
                },
                exhaust_temp_k: smoothed.exhaust_temp_k,
                exhaust_velocity_mps: smoothed.exhaust_velocity_mps,
                exit_pressure_pa: smoothed.exit_pressure_pa,
                exit_mach: smoothed.exit_mach,
                compressor_inlet_total_temp_k: smoothed.compressor_inlet_total_temp_k,
                precooler_heat_flow_w: smoothed.precooler_heat_flow_w,
                precooler_wall_heat_flow_w: smoothed.precooler_wall_heat_flow_w,
                precooler_wall_temp_k: smoothed.precooler_wall_temp_k,
                coolant_outlet_temp_k: smoothed.coolant_outlet_temp_k,
                precooler_saturated,
            },
            smoothed,
        ))
    }
}

/// One steady analyzer row for the ESTOC mode/thermal envelope.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct EstocAltitudePoint {
    pub altitude_m: f64,
    pub mach: f64,
    pub ambient_pa: f64,
    pub mode: EstocMode,
    pub thrust_n: f64,
    pub isp_total_s: f64,
    pub fuel_flow_kg_s: f64,
    pub bulk_fuel_flow_kg_s: f64,
    pub boost_fuel_flow_kg_s: f64,
    pub oxidizer_flow_kg_s: f64,
    pub air_flow_kg_s: f64,
    pub compressor_inlet_total_temp_k: f64,
    pub precooler_heat_flow_w: f64,
    pub precooler_saturated: bool,
}

/// Evaluate the steady ESTOC mode envelope over altitude × Mach.
/// The finite wall store is not credited at steady state; optional
/// flowing coolant/boost fuel remains available up to its authored limits.
pub fn analyze_estoc(
    engine: &CompiledEstoc,
    atmosphere: &crate::atmosphere::AtmosphereConfig,
    altitudes_m: &[f64],
    machs: &[f64],
    throttle: f64,
) -> Result<Vec<EstocAltitudePoint>, PropulsionError> {
    if altitudes_m.is_empty() || machs.is_empty() {
        return Err(PropulsionError::InvalidSpec(
            "ESTOC analyzer needs altitudes and Mach numbers".into(),
        ));
    }
    let mut rows = Vec::with_capacity(altitudes_m.len() * machs.len());
    for altitude_m in altitudes_m {
        if !altitude_m.is_finite() {
            return Err(PropulsionError::InvalidSpec(
                "ESTOC analyzer altitudes must be finite".into(),
            ));
        }
        let sample = atmosphere
            .sample(*altitude_m)
            .map_err(|error| PropulsionError::InvalidSpec(format!("atmosphere sample: {error}")))?;
        for mach in machs {
            if !mach.is_finite() || *mach < 0.0 {
                return Err(PropulsionError::InvalidSpec(
                    "ESTOC analyzer Mach numbers must be finite and >= 0".into(),
                ));
            }
            let condition =
                super::air::flight_condition(&sample, mach * sample.speed_of_sound_mps)?;
            let (point, _) = engine.operating_point(
                &condition,
                throttle,
                None,
                EstocMode::Air,
                None,
                f64::INFINITY,
                JetShaftState::running(&engine.air),
            )?;
            rows.push(EstocAltitudePoint {
                altitude_m: *altitude_m,
                mach: *mach,
                ambient_pa: sample.pressure_pa,
                mode: point.mode,
                thrust_n: point.thrust_n,
                isp_total_s: point.isp_total_s,
                fuel_flow_kg_s: point.fuel_flow_kg_s,
                bulk_fuel_flow_kg_s: point.bulk_fuel_flow_kg_s,
                boost_fuel_flow_kg_s: point.boost_fuel_flow_kg_s,
                oxidizer_flow_kg_s: point.oxidizer_flow_kg_s,
                air_flow_kg_s: point.air_flow_kg_s,
                compressor_inlet_total_temp_k: point.compressor_inlet_total_temp_k,
                precooler_heat_flow_w: point.precooler_heat_flow_w,
                precooler_saturated: point.precooler_saturated,
            });
        }
    }
    Ok(rows)
}

#[cfg(test)]
mod tests {
    use super::super::AIR_GAMMA;
    use super::super::{AirCycle, AirbreathingSpec, ChamberMaterial, JetFuel, ShaftSpec};
    use super::*;
    use crate::atmosphere::AtmosphereComposition;

    fn estoc_like() -> EstocSpec {
        EstocSpec {
            name: "ESTOC-1".into(),
            air: AirbreathingSpec {
                name: "estoc-air".into(),
                cycle: AirCycle::Turbojet,
                fuel: JetFuel::Kerosene,
                intake_area_m2: 0.9,
                intake: super::super::IntakeKind::Pitot,
                compressor_ratio: 12.0,
                bypass_ratio: 0.0,
                fan_pressure_ratio: 1.0,
                turbine_inlet_temp_k: 1500.0,
                afterburner: false,
                reheat_temp_k: 0.0,
                turbine_material: ChamberMaterial::nickel_superalloy(),
                spool_tau_s: 5.0,
                shaft: ShaftSpec::default(),
            },
            bulk_fuel: None,
            boost_coolant_fuel: None,
            precooler: None,
            ejector: None,
            rocket_chamber_pressure_pa: 7.0e6,
            rocket_throat_radius_m: 0.09,
            oxidizer_fuel_ratio: None,
            switch_mach_hi: None,
            switch_mach_lo: None,
            transition_tau_s: None,
        }
    }

    fn v6_estoc() -> EstocSpec {
        let mut spec = estoc_like();
        spec.air.intake_area_m2 = 0.05;
        spec.air.compressor_ratio = 1.5;
        spec.bulk_fuel = Some(JetFuel::Methane);
        spec.boost_coolant_fuel = Some(JetFuel::Hydrogen);
        spec.precooler = Some(EstocPrecoolerSpec {
            maximum_heat_flow_w: 20.0e6,
            effectiveness: 0.85,
            maximum_compressor_inlet_temp_k: 500.0,
            pressure_recovery: 0.98,
            wall_mass_kg: 500.0,
            wall_specific_heat_j_kg_k: 1_000.0,
            wall_initial_temp_k: 300.0,
            wall_max_temp_k: 800.0,
            coolant_inlet_temp_k: 20.0,
            coolant_max_outlet_temp_k: 400.0,
            coolant_specific_heat_j_kg_k: 14_000.0,
            maximum_coolant_flow_kg_s: 0.1,
        });
        spec.ejector = Some(EstocEjectorSpec {
            capture_area_m2: 0.06,
            mixing_length_m: 2.0,
            mixing_efficiency: 0.95,
            structure_density_kg_m3: 2_700.0,
            wall_thickness_m: 0.005,
        });
        spec
    }

    fn condition_at(mach: f64, ambient_pa: f64) -> FlightCondition {
        let temp = 288.15;
        let a = (AIR_GAMMA * 287.0 * temp).sqrt();
        FlightCondition {
            mach,
            ambient_pa,
            ambient_temp_k: temp,
            airspeed_mps: mach * a,
            composition: AtmosphereComposition::earth_air(),
        }
    }

    #[test]
    fn shared_nozzle_and_mass_book_once() {
        // Rocket expansion follows from the shared exit (no independent
        // nozzle); dry mass is air plus a small rocket-side extra, never
        // two full engines.
        let engine = estoc_like().compile().expect("estoc compiles");
        assert!(engine.rocket_expansion_ratio >= 1.0);
        let extra_fraction = engine.rocket_extra_mass_kg / engine.air.dry_mass_kg;
        assert!(
            (0.03..=0.20).contains(&extra_fraction),
            "rocket extra {extra_fraction:.3} outside the hardware band"
        );
        assert!(
            (engine.dry_mass_kg - engine.air.dry_mass_kg - engine.rocket_extra_mass_kg).abs()
                < 1e-9
        );
        assert!(engine.feed_pressure_required_pa > 0.0);
    }

    #[test]
    fn mode_discipline_with_hysteresis() {
        // Solved inlet/drive envelope selects modes; Mach hysteresis only
        // delays a rocket-to-air return while the air path is viable.
        let engine = EstocSpec {
            air: AirbreathingSpec {
                compressor_ratio: 1.5,
                ..estoc_like().air
            },
            ..estoc_like()
        }
        .compile()
        .expect("estoc compiles");
        let low_condition = condition_at(1.5, 101_325.0);
        let air_low = engine
            .air
            .operating_point(&low_condition, 1.0)
            .expect("air point");
        assert_eq!(
            engine.select_mode(&low_condition, &air_low, EstocMode::Air, None),
            EstocMode::Air
        );
        let high_condition = condition_at(4.5, 1000.0);
        let air_high = engine
            .air
            .operating_point(&high_condition, 1.0)
            .expect("high-Mach air point");
        assert_eq!(
            engine.select_mode(&high_condition, &air_high, EstocMode::Air, None),
            EstocMode::Rocket
        );
        assert_eq!(
            engine.select_mode(&condition_at(0.0, 0.0), &air_low, EstocMode::Air, None),
            EstocMode::Rocket
        );
        // Inside the band the last mode holds.
        let band_condition = condition_at(3.3, 5000.0);
        let air_band = engine
            .air
            .operating_point(&band_condition, 1.0)
            .expect("band air point");
        assert!(air_band.lit && !air_band.drive_limited && air_band.thrust_n > 0.0);
        assert_eq!(
            engine.select_mode(&band_condition, &air_band, EstocMode::Rocket, None),
            EstocMode::Rocket
        );
        assert_eq!(
            engine.select_mode(&band_condition, &air_band, EstocMode::Air, None),
            EstocMode::Air
        );
        assert_eq!(
            engine.select_mode(
                &high_condition,
                &air_high,
                EstocMode::Rocket,
                Some(EstocMode::Air)
            ),
            EstocMode::Air
        );

        let policy_engine = EstocSpec {
            air: AirbreathingSpec {
                compressor_ratio: 1.5,
                ..estoc_like().air
            },
            switch_mach_hi: Some(2.4),
            switch_mach_lo: Some(2.0),
            ..estoc_like()
        }
        .compile()
        .expect("policy-band ESTOC");
        let policy_condition = condition_at(2.5, 101_325.0);
        let policy_air_point = policy_engine
            .air
            .operating_point(&policy_condition, 1.0)
            .expect("viable point above policy threshold");
        assert!(policy_air_point.lit && !policy_air_point.drive_limited);
        assert_eq!(
            policy_engine.select_mode(&policy_condition, &policy_air_point, EstocMode::Air, None,),
            EstocMode::Rocket
        );
    }

    #[test]
    fn precooler_separates_bulk_and_warmed_boost_fuel_and_advances_wall_state() {
        let engine = v6_estoc().compile().expect("v6 ESTOC compiles");
        assert_eq!(engine.bulk_fuel, JetFuel::Methane);
        assert_eq!(engine.boost_coolant_fuel, Some(JetFuel::Hydrogen));
        let mut condition = condition_at(2.0, 101_325.0);
        condition.composition = AtmosphereComposition::earth_air();
        let (point, state) = engine
            .operating_point(
                &condition,
                1.0,
                None,
                EstocMode::Air,
                None,
                1.0,
                JetShaftState::running(&engine.air),
            )
            .expect("precooled point");
        assert_eq!(point.mode, EstocMode::Air);
        assert!(point.thrust_n > 0.0);
        assert!(point.precooler_heat_flow_w > 0.0);
        assert!(point.boost_fuel_flow_kg_s > 0.0);
        assert!(point.bulk_fuel_flow_kg_s > 0.0);
        assert!(
            (point.fuel_flow_kg_s - point.bulk_fuel_flow_kg_s - point.boost_fuel_flow_kg_s).abs()
                < 1e-12
        );
        assert!(point.compressor_inlet_total_temp_k < 519.0);
        assert!(point.precooler_heat_flow_w <= engine.precooler.unwrap().maximum_heat_flow_w);
        assert!(state.coolant_outlet_temp_k <= 400.0 + 1e-9);
        let compressor_tau = 1.5f64.powf(
            (super::super::AIR_GAMMA - 1.0)
                / (super::super::AIR_GAMMA * super::super::air::COMPRESSOR_POLY_EFFICIENCY),
        );
        let compressor_exit_k = point.compressor_inlet_total_temp_k * compressor_tau;
        let recovered_coolant_heat_w =
            point.precooler_heat_flow_w - point.precooler_wall_heat_flow_w;
        let fuel_heat_w = (point.bulk_fuel_flow_kg_s * JetFuel::Methane.properties().0
            + point.boost_fuel_flow_kg_s * JetFuel::Hydrogen.properties().0)
            * COMBUSTOR_EFFICIENCY;
        let expected_combustor_heat_w =
            point.air_flow_kg_s * AIR_CP_J_KG_K * (1_500.0 - compressor_exit_k);
        assert!(
            ((fuel_heat_w + recovered_coolant_heat_w) - expected_combustor_heat_w).abs()
                <= 1e-8 * expected_combustor_heat_w.abs().max(1.0)
        );

        let mut wall_only = v6_estoc();
        wall_only.boost_coolant_fuel = None;
        wall_only.air.compressor_ratio = 1.2;
        let wall_engine = wall_only.compile().expect("wall-store ESTOC");
        let hot_condition = condition_at(4.5, 10_000.0);
        let (hot_point, hot_state) = wall_engine
            .operating_point(
                &hot_condition,
                1.0,
                Some(EstocMode::Air),
                EstocMode::Air,
                None,
                100.0,
                JetShaftState::running(&wall_engine.air),
            )
            .expect("wall-limited hot point");
        assert!(hot_point.precooler_saturated);
        assert!(hot_state.precooler_wall_temp_k <= wall_engine.precooler.unwrap().wall_max_temp_k);
        let wall_capacity_j_k = wall_engine.precooler.unwrap().wall_mass_kg
            * wall_engine.precooler.unwrap().wall_specific_heat_j_kg_k;
        let expected_wall_temp_k = wall_engine.precooler.unwrap().wall_initial_temp_k
            + hot_point.precooler_wall_heat_flow_w * 100.0 / wall_capacity_j_k;
        assert!((hot_state.precooler_wall_temp_k - expected_wall_temp_k).abs() < 1e-9);

        let (automatic, _) = wall_engine
            .operating_point(
                &hot_condition,
                1.0,
                None,
                EstocMode::Air,
                Some(&hot_state),
                100.0,
                JetShaftState::running(&wall_engine.air),
            )
            .expect("automatic thermal fallback");
        assert_eq!(automatic.mode, EstocMode::Rocket);
    }

    #[test]
    fn estoc_precooler_work_is_included_in_runtime_shaft_balance() {
        let mut cooled_spec = v6_estoc();
        let cooled = cooled_spec.compile().expect("cooled ESTOC");
        cooled_spec.precooler = None;
        cooled_spec.boost_coolant_fuel = None;
        let uncooled = cooled_spec.compile().expect("uncooled ESTOC");
        let mount = |engine| crate::JetMount {
            name: "shaft-balance-mount".into(),
            engine: crate::CompiledJet::Estoc(Box::new(engine)),
            position_body_m: [0.0; 3],
            thrust_axis_body: [1.0, 0.0, 0.0],
            gimbal_range_rad: 0.0,
        };
        let cooled_mount = mount(cooled.clone());
        let uncooled_mount = mount(uncooled.clone());
        let mut cooled_shaft = JetShaftState::running(&cooled.air);
        cooled_shaft.spool_n = 0.5;
        let uncooled_shaft = JetShaftState {
            starter_charge_j: cooled_shaft.starter_charge_j,
            ..cooled_shaft
        };
        let command = |shaft| crate::JetCommand {
            manual: Some(EstocMode::Air),
            last_mode: EstocMode::Air,
            prev: None,
            dt_s: 0.1,
            shaft,
            starter_engaged: false,
            generator_load_w: 0.0,
        };
        let condition = condition_at(2.0, 101_325.0);
        let (cooled_point, _, cooled_next) = cooled_mount
            .estoc_point(1.0, &condition, &command(cooled_shaft))
            .expect("cooled runtime step");
        let (_, _, uncooled_next) = uncooled_mount
            .estoc_point(1.0, &condition, &command(uncooled_shaft))
            .expect("uncooled runtime step");
        assert!(cooled_point.precooler_heat_flow_w > 0.0);
        assert!(
            cooled_next.spool_n > uncooled_next.spool_n,
            "pre-cooler heat/work balance must affect spool integration"
        );
    }

    #[test]
    fn estoc_analyzer_reports_the_selected_steady_mode_and_thermal_point() {
        let engine = v6_estoc().compile().expect("v6 ESTOC");
        let atmosphere = crate::atmosphere::AtmosphereConfig::default();
        let sample = atmosphere.sample(0.0).expect("sea-level atmosphere");
        let condition = super::super::air::flight_condition(&sample, 0.0).expect("condition");
        let (expected, _) = engine
            .operating_point(
                &condition,
                1.0,
                None,
                EstocMode::Air,
                None,
                f64::INFINITY,
                JetShaftState::running(&engine.air),
            )
            .expect("steady ESTOC point");
        let rows =
            analyze_estoc(&engine, &atmosphere, &[0.0], &[0.0], 1.0).expect("ESTOC analyzer");
        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0].mode, expected.mode);
        assert_eq!(rows[0].thrust_n, expected.thrust_n);
        assert_eq!(
            rows[0].precooler_heat_flow_w,
            expected.precooler_heat_flow_w
        );
        assert!(analyze_estoc(&engine, &atmosphere, &[], &[0.0], 1.0).is_err());
    }

    #[test]
    fn anoxic_ejector_uses_captured_air_as_reaction_mass_with_rocket_propellant() {
        let engine = v6_estoc().compile().expect("v6 ESTOC compiles");
        let mut condition = condition_at(2.0, 101_325.0);
        condition.composition = AtmosphereComposition::from_mole_fractions(&[(
            crate::atmosphere::GasKind::Nitrogen,
            1.0,
        )])
        .expect("nitrogen atmosphere");
        let (point, _) = engine
            .operating_point(
                &condition,
                1.0,
                None,
                EstocMode::Air,
                None,
                1.0,
                JetShaftState::running(&engine.air),
            )
            .expect("anoxic ejector point");
        let rocket = engine
            .rocket_point(1.0, condition.ambient_pa)
            .expect("rocket comparison");
        assert_eq!(point.mode, EstocMode::Ejector);
        assert!(point.air_flow_kg_s > 0.0);
        assert!(point.thrust_n > rocket.thrust_n);
        assert!(
            (point.oxidizer_flow_kg_s / point.bulk_fuel_flow_kg_s - engine.oxidizer_fuel_ratio)
                .abs()
                < 1e-9
        );

        condition.airspeed_mps = 0.0;
        condition.mach = 0.0;
        let (no_capture, _) = engine
            .operating_point(
                &condition,
                1.0,
                None,
                EstocMode::Air,
                None,
                1.0,
                JetShaftState::running(&engine.air),
            )
            .expect("ejector falls back without captured flow");
        assert_eq!(no_capture.mode, EstocMode::Rocket);
        assert_eq!(no_capture.air_flow_kg_s, 0.0);
    }

    #[test]
    fn estoc_v6_rejects_unmatched_coolant_and_ejector_inputs() {
        let mut no_cooler = v6_estoc();
        no_cooler.precooler = None;
        assert!(no_cooler.compile().is_err());
        let mut impossible_coolant = v6_estoc();
        impossible_coolant
            .precooler
            .as_mut()
            .unwrap()
            .wall_initial_temp_k = 900.0;
        assert!(impossible_coolant.compile().is_err());
        let mut bad_ejector = v6_estoc();
        bad_ejector.ejector.as_mut().unwrap().mixing_efficiency = 1.1;
        assert!(bad_ejector.compile().is_err());
        let mut bad_upper_only = v6_estoc();
        bad_upper_only.switch_mach_hi = Some(2.0);
        assert!(bad_upper_only.compile().is_err());
        let mut bad_lower_only = v6_estoc();
        bad_lower_only.switch_mach_lo = Some(4.0);
        assert!(bad_lower_only.compile().is_err());
        let mut overflowing_capacity = v6_estoc();
        overflowing_capacity
            .precooler
            .as_mut()
            .unwrap()
            .wall_mass_kg = f64::MAX;
        overflowing_capacity
            .precooler
            .as_mut()
            .unwrap()
            .wall_specific_heat_j_kg_k = f64::MAX;
        assert!(overflowing_capacity.compile().is_err());
    }

    #[test]
    fn rocket_works_where_air_cannot() {
        // Vacuum rocket thrust is positive with modest shared-nozzle Isp;
        // at Mach 2 sea level air Isp dwarfs rocket Isp.
        let engine = estoc_like().compile().expect("estoc compiles");
        let vac = engine.rocket_point(1.0, 0.0).expect("vac rocket");
        assert!(vac.thrust_n > 0.0);
        assert!(
            (200.0..=350.0).contains(&vac.isp_total_s),
            "shared-nozzle rocket Isp {:.0} s outside the band",
            vac.isp_total_s
        );
        let air_m2 = engine
            .operating_point(
                &condition_at(2.0, 101_325.0),
                1.0,
                None,
                EstocMode::Air,
                None,
                1.0,
                JetShaftState::running(&engine.air),
            )
            .expect("air");
        assert_eq!(air_m2.0.mode, EstocMode::Air);
        let rocket_m2 = engine.rocket_point(1.0, 101_325.0).expect("sl rocket");
        assert!(air_m2.0.isp_total_s > 3.0 * rocket_m2.isp_total_s);
        // Oxidizer/fuel split respects the OF ratio.
        assert!(
            (vac.oxidizer_flow_kg_s / vac.fuel_flow_kg_s - engine.oxidizer_fuel_ratio).abs()
                / engine.oxidizer_fuel_ratio
                < 1e-9
        );
    }

    #[test]
    fn transition_evolves_consistent_state() {
        // Fresh evaluation hits the target exactly; a threaded transient
        // approaches it first-order across EVERY channel (not just
        // thrust), and the reported Isp always equals F/(mdot*g0) on the
        // smoothed snapshot — bookkeeping can never break mid-transition.
        // Long dt converges to the direct target.
        let engine = estoc_like().compile().expect("estoc compiles");
        let condition = condition_at(4.0, 2000.0);
        let (fresh, fresh_state) = engine
            .operating_point(
                &condition,
                1.0,
                None,
                EstocMode::Rocket,
                None,
                0.0,
                JetShaftState::running(&engine.air),
            )
            .expect("fresh");
        assert_eq!(fresh.mode, EstocMode::Rocket);
        let (direct, _) = engine
            .operating_point(
                &condition,
                1.0,
                Some(EstocMode::Rocket),
                EstocMode::Rocket,
                None,
                0.0,
                JetShaftState::running(&engine.air),
            )
            .expect("direct");
        assert!((fresh.thrust_n - direct.thrust_n).abs() / direct.thrust_n < 1e-12);
        // One tau from a cold snapshot: every channel partway, Isp exact.
        let cold = EstocTransient {
            thrust_n: 50_000.0,
            fuel_flow_kg_s: 5.0,
            bulk_fuel_flow_kg_s: 5.0,
            boost_fuel_flow_kg_s: 0.0,
            oxidizer_flow_kg_s: 0.0,
            air_flow_kg_s: 100.0,
            exhaust_temp_k: 800.0,
            exhaust_velocity_mps: 600.0,
            exit_pressure_pa: 50_000.0,
            exit_mach: 1.0,
            compressor_inlet_total_temp_k: 300.0,
            precooler_heat_flow_w: 0.0,
            precooler_wall_heat_flow_w: 0.0,
            precooler_wall_temp_k: 0.0,
            coolant_outlet_temp_k: 0.0,
        };
        let (mid, mid_state) = engine
            .operating_point(
                &condition,
                1.0,
                None,
                EstocMode::Air,
                Some(&cold),
                engine.transition_tau_s,
                JetShaftState::running(&engine.air),
            )
            .expect("mid");
        assert_eq!(mid.mode, EstocMode::Rocket);
        let alpha = 1.0 - (-1.0f64).exp();
        assert!(
            (mid.thrust_n - (50_000.0 + (direct.thrust_n - 50_000.0) * alpha)).abs()
                / direct.thrust_n
                < 1e-9
        );
        assert!(
            (mid.fuel_flow_kg_s - (5.0 + (direct.fuel_flow_kg_s - 5.0) * alpha)).abs()
                / direct.fuel_flow_kg_s.max(1e-9)
                < 1e-9
        );
        let propellant = mid.fuel_flow_kg_s + mid.oxidizer_flow_kg_s;
        assert!(
            (mid.isp_total_s - mid.thrust_n / (propellant * 9.80665)).abs() / mid.isp_total_s
                < 1e-12
        );
        // Converged: thread the snapshot until it lands on the target.
        let mut state = mid_state;
        let mut mode = mid.mode;
        for _ in 0..200 {
            let (point, next) = engine
                .operating_point(
                    &condition,
                    1.0,
                    None,
                    mode,
                    Some(&state),
                    engine.transition_tau_s,
                    JetShaftState::running(&engine.air),
                )
                .expect("converge");
            mode = point.mode;
            state = next;
        }
        assert!((state.thrust_n - direct.thrust_n).abs() / direct.thrust_n < 1e-6);
        assert_eq!(mode, EstocMode::Rocket);
        let _ = fresh_state;
    }

    #[test]
    fn fresh_command_evaluates_estoc_target() {
        // A fresh command has no previous snapshot, so the target is
        // returned directly. It remains JSON-safe for editor/network use.
        let engine = estoc_like().compile().expect("estoc compiles");
        let command = super::super::jet::JetCommand::fresh();
        assert!(command.dt_s.is_finite());
        let (point, _) = engine
            .operating_point(
                &condition_at(4.0, 2000.0),
                1.0,
                command.manual,
                command.last_mode,
                command.prev.as_ref(),
                command.dt_s,
                command.shaft,
            )
            .expect("fresh command evaluates");
        assert_eq!(point.mode, EstocMode::Rocket);
        assert!(point.thrust_n > 0.0);
    }

    #[test]
    fn manual_air_in_vacuum_flames_out_cleanly() {
        // Contract: manual selection always wins — including a manual Air
        // in vacuum, which is honored and flames out (zero thrust, cause
        // flags) rather than silently switching to rocket.
        let engine = estoc_like().compile().expect("estoc compiles");
        let vacuum = condition_at(0.0, 0.0);
        let (point, _) = engine
            .operating_point(
                &vacuum,
                1.0,
                Some(EstocMode::Air),
                EstocMode::Air,
                None,
                0.0,
                JetShaftState::running(&engine.air),
            )
            .expect("manual air");
        assert_eq!(point.mode, EstocMode::Air);
        assert_eq!(point.thrust_n, 0.0);
        assert_eq!(point.fuel_flow_kg_s, 0.0);
    }

    #[test]
    fn estoc_validation_refuses_garbage() {
        // Oversized rocket throat (expansion < 1), over-cap Pc, and a
        // ramjet air path are all refused with reasons.
        let big_throat = EstocSpec {
            rocket_throat_radius_m: 2.0,
            ..estoc_like()
        };
        assert!(big_throat.compile().is_err());
        let hot_pc = EstocSpec {
            rocket_chamber_pressure_pa: 100.0e6,
            ..estoc_like()
        };
        assert!(hot_pc.compile().is_err());
        let ram_air = EstocSpec {
            air: AirbreathingSpec {
                cycle: AirCycle::Ramjet,
                compressor_ratio: 1.0,
                ..estoc_like().air
            },
            ..estoc_like()
        };
        assert!(ram_air.compile().is_err());
    }
}
