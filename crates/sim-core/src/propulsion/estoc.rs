//! ESTOC combined-cycle engines: an air-breathing turbojet path plus a
//! closed-cycle rocket path sharing intake ducting, chamber, and nozzle
//! hardware — one engine, two flow paths, valve-selected.
//!
//! This fills the gameplay niche of switchable air/rocket engines under our
//! own name. Mode discipline is strict: air while the intake delivers,
//! rocket above the switch band, on dead air, or on manual command —
//! never both at once. The shared convergent nozzle caps rocket expansion (documented): rocket mode buys
//! thrust where air fails, not orbital efficiency.
//!
//! The rocket path reuses the LOX pair thermo at its reference mixture
//! ratio, the isentropic kernel, and the Summerfield flag. Air path is a
//! [`CompiledAirbreather`]; oxidizer is LOX from vehicle tanks.

// Validity guards use `!(x > 0.0)` so NaN fails closed (repo convention).
#![allow(clippy::neg_cmp_op_on_partial_ord)]

use serde::{Deserialize, Serialize};

use super::shaft::JetShaftState;
use super::{
    AirOperatingPoint, AirbreathingSpec, CompiledAirbreather, FlightCondition, JetFuel,
    MASS_FIT_FEED_KG_PER_N, NozzleExitState, Propellant, PropellantThermo, PropulsionError,
    STANDARD_GRAVITY_MPS2, characteristic_velocity, mach_from_area_ratio, require_positive,
    thrust_coefficient,
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
    pub oxidizer_flow_kg_s: f64,
    pub air_flow_kg_s: f64,
    pub exhaust_temp_k: f64,
    pub exhaust_velocity_mps: f64,
    pub exit_pressure_pa: f64,
    pub exit_mach: f64,
}

/// One ESTOC operating point (single active mode).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EstocPoint {
    pub mode: EstocMode,
    pub thrust_n: f64,
    pub fuel_flow_kg_s: f64,
    /// LOX flow in rocket mode, 0 in air mode.
    pub oxidizer_flow_kg_s: f64,
    pub air_flow_kg_s: f64,
    /// Isp over all onboard propellant (fuel + oxidizer).
    pub isp_total_s: f64,
    pub exhaust_temp_k: f64,
    pub exhaust_velocity_mps: f64,
    pub exit_pressure_pa: f64,
    pub exit_mach: f64,
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
        if let (Some(hi), Some(lo)) = (self.switch_mach_hi, self.switch_mach_lo)
            && (!hi.is_finite() || !lo.is_finite() || hi <= lo || lo <= 0.0)
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
        let air = self.air.compile()?;
        let pair = self.air.fuel.lox_pair();
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
        let dry_mass_kg = air.dry_mass_kg + rocket_extra_mass_kg;
        Ok(CompiledEstoc {
            name: self.name.clone(),
            air,
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

    /// Mode selector: manual wins; vacuum always rockets; above the high
    /// edge rockets; hysteresis holds rocket inside the band; a dead air
    /// path (stalled drive, anoxic air) falls back to rocket. When the
    /// air path is actually commanded (throttle > 0), an unlit core or
    /// zero intake flow is dead air too — the shaft has failed to light
    /// or the intake delivered nothing, so the valve falls back to
    /// rocket. Idle (`air_commanded = false`) skips that rule: a
    /// deliberately stopped core at zero throttle is not a failure and
    /// the reported mode stays Air (documented section 8.1 rule).
    /// Sustained intake starvation only limits (flagged) — switching on
    /// weakness is the pilot/FCU call, not the valve logic.
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
        if condition.ambient_pa <= 0.0 {
            return EstocMode::Rocket;
        }
        if condition.mach >= self.switch_mach_hi {
            return EstocMode::Rocket;
        }
        if last_mode == EstocMode::Rocket && condition.mach > self.switch_mach_lo {
            return EstocMode::Rocket;
        }
        if air_point.drive_limited || air_point.oxygen_limited {
            return EstocMode::Rocket;
        }
        if air_commanded && (!air_point.lit || air_point.air_flow_kg_s <= 0.0) {
            return EstocMode::Rocket;
        }
        EstocMode::Air
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
        })
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
    /// mode selection falls back to rocket when air is commanded but
    /// dead. Analyzer callers pass a steady state
    /// ([`JetShaftState::running`] or the solved equilibrium).
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
            let transient = EstocTransient {
                thrust_n: 0.0,
                fuel_flow_kg_s: 0.0,
                oxidizer_flow_kg_s: 0.0,
                air_flow_kg_s: 0.0,
                exhaust_temp_k: condition.ambient_temp_k,
                exhaust_velocity_mps: 0.0,
                exit_pressure_pa: condition.ambient_pa,
                exit_mach: 0.0,
            };
            return Ok((
                EstocPoint {
                    mode,
                    thrust_n: 0.0,
                    fuel_flow_kg_s: 0.0,
                    oxidizer_flow_kg_s: 0.0,
                    air_flow_kg_s: 0.0,
                    isp_total_s: 0.0,
                    exhaust_temp_k: condition.ambient_temp_k,
                    exhaust_velocity_mps: 0.0,
                    exit_pressure_pa: condition.ambient_pa,
                    exit_mach: 0.0,
                },
                transient,
            ));
        }
        let air_point = self
            .air
            .operating_point_at_spool(condition, throttle, shaft.spool_n, shaft.lit)?
            .0;
        let mode = self.select_mode(condition, &air_point, last_mode, manual);
        let target = match mode {
            EstocMode::Air => EstocTransient {
                thrust_n: air_point.thrust_n.max(0.0),
                fuel_flow_kg_s: air_point.fuel_flow_kg_s,
                oxidizer_flow_kg_s: 0.0,
                air_flow_kg_s: air_point.air_flow_kg_s,
                exhaust_temp_k: air_point.exhaust_temp_k,
                exhaust_velocity_mps: air_point.exhaust_velocity_mps,
                exit_pressure_pa: air_point.exit_pressure_pa,
                exit_mach: air_point.exit_mach,
            },
            EstocMode::Rocket => {
                let rocket = self.rocket_point(throttle, condition.ambient_pa)?;
                EstocTransient {
                    thrust_n: rocket.thrust_n,
                    fuel_flow_kg_s: rocket.fuel_flow_kg_s,
                    oxidizer_flow_kg_s: rocket.oxidizer_flow_kg_s,
                    air_flow_kg_s: 0.0,
                    exhaust_temp_k: rocket.exhaust_temp_k,
                    exhaust_velocity_mps: rocket.exhaust_velocity_mps,
                    exit_pressure_pa: rocket.exit_pressure_pa,
                    exit_mach: rocket.exit_mach,
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
                }
            }
        };
        let propellant_flow = smoothed.fuel_flow_kg_s + smoothed.oxidizer_flow_kg_s;
        Ok((
            EstocPoint {
                mode,
                thrust_n: smoothed.thrust_n,
                fuel_flow_kg_s: smoothed.fuel_flow_kg_s,
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
            },
            smoothed,
        ))
    }
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
            rocket_chamber_pressure_pa: 7.0e6,
            rocket_throat_radius_m: 0.09,
            oxidizer_fuel_ratio: None,
            switch_mach_hi: None,
            switch_mach_lo: None,
            transition_tau_s: None,
        }
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
        // Air low and slow, rocket past the band or in vacuum, hysteresis
        // inside the band, manual override everywhere.
        let engine = estoc_like().compile().expect("estoc compiles");
        let air_low = engine
            .air
            .operating_point(&condition_at(1.5, 101_325.0), 1.0)
            .expect("air point");
        assert_eq!(
            engine.select_mode(
                &condition_at(1.5, 101_325.0),
                &air_low,
                EstocMode::Air,
                None
            ),
            EstocMode::Air
        );
        assert_eq!(
            engine.select_mode(&condition_at(4.5, 1000.0), &air_low, EstocMode::Air, None),
            EstocMode::Rocket
        );
        assert_eq!(
            engine.select_mode(&condition_at(0.0, 0.0), &air_low, EstocMode::Air, None),
            EstocMode::Rocket
        );
        // Inside the band the last mode holds.
        assert_eq!(
            engine.select_mode(
                &condition_at(3.3, 5000.0),
                &air_low,
                EstocMode::Rocket,
                None
            ),
            EstocMode::Rocket
        );
        assert_eq!(
            engine.select_mode(&condition_at(3.3, 5000.0), &air_low, EstocMode::Air, None),
            EstocMode::Air
        );
        assert_eq!(
            engine.select_mode(
                &condition_at(4.5, 1000.0),
                &air_low,
                EstocMode::Rocket,
                Some(EstocMode::Air)
            ),
            EstocMode::Air
        );
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
            oxidizer_flow_kg_s: 0.0,
            air_flow_kg_s: 100.0,
            exhaust_temp_k: 800.0,
            exhaust_velocity_mps: 600.0,
            exit_pressure_pa: 50_000.0,
            exit_mach: 1.0,
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
