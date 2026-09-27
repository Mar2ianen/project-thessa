//! Multi-chamber propulsion systems: one shared feed (single turbopump
//! set, single gas-generator duct, common tanks) driving several
//! chamber/nozzle assemblies at their own stations — RD-170-style
//! clustering with differential throttle, per-chamber gimbals, and one
//! plume source per nozzle.
//!
//! The compile is native, not N single-engine compiles: shared
//! turbomachinery is booked once from total flow, while chambers carry
//! their own walls, nozzles, injectors, and gimbals. A single-chamber
//! system reproduces the standalone liquid compile exactly (pinned by
//! test); with linear feed hardware the totals scale linearly, and the
//! value of clustering is runtime authority, not mass magic.

use glam::DVec3;
use serde::{Deserialize, Serialize};

use crate::atmosphere::AtmosphereConfig;

use super::mount::gimbal_pair;
use super::{
    ChamberMaterial, CoolingMode, EngineCycle, EngineOperatingPoint, EnginePlumeState, EngineSpool,
    GimbalEffector, NozzleContour, NozzleExitState, Propellant, PropellantThermo, PropulsionError,
    STANDARD_GRAVITY_MPS2, characteristic_velocity, nozzle_exit, require_non_negative,
    require_positive, require_unit_interval, thrust_coefficient,
};
use super::{GG_DUCT_EXPANSION_RATIO, GG_PRESSURE_FRACTION};
use super::{MASS_FIT_FEED_KG_PER_N, MASS_FIT_GIMBAL_BASE_KG, MASS_FIT_GIMBAL_KG_PER_N};
use super::{MASS_FIT_HEAD_BASE_KG, MASS_FIT_HEAD_KG_PER_M2, MASS_FIT_MOUNT_KG_PER_N};
use super::{MASS_FIT_TURBO_KG_PER_FLOW_POWER, PRESSURE_SAFETY_FACTOR};
use super::{PUMP_EFFICIENCY, PUMP_SPECIFIC_POWER_W_PER_KG, TANK_PRESSURE_PA};

/// Maximum chambers per system (documented clustering bound).
pub const MAX_SYSTEM_CHAMBERS: usize = 16;

/// One chamber/nozzle assembly on the shared feed.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ChamberSpec {
    pub name: String,
    pub throat_radius_m: f64,
    pub expansion_ratio: f64,
    pub nozzle_length_m: f64,
    pub contour: NozzleContour,
    /// Chamber/nozzle station in vehicle body metres.
    pub position_body_m: [f64; 3],
    /// Unit thrust direction in vehicle body axes.
    pub thrust_axis_body: [f64; 3],
    /// Gimbal half-range (rad, 0 = fixed).
    pub gimbal_range_rad: f64,
}

/// Multi-chamber engine authoring: shared feed plus chamber list.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PropulsionSystemSpec {
    pub name: String,
    pub propellant: Propellant,
    pub cycle: EngineCycle,
    /// Shared design chamber pressure (Pa).
    pub chamber_pressure_pa: f64,
    /// Oxidizer-to-fuel ratio override; `None` = pair reference.
    pub mixture_ratio: Option<f64>,
    pub chamber_material: ChamberMaterial,
    pub cooling: CoolingMode,
    /// Characteristic length override (m); `None` = propellant reference.
    pub characteristic_length_m: Option<f64>,
    /// Minimum stable throttle; `None` = cycle floor.
    pub min_throttle: Option<f64>,
    /// Restartable after shutdown.
    pub restartable: bool,
    pub chambers: Vec<ChamberSpec>,
}

/// Hangar-compiled chamber: design scalars plus its dry-mass share
/// (walls, nozzle, injector, head, gimbal — no shared turbomachinery).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CompiledChamber {
    pub name: String,
    pub throat_area_m2: f64,
    pub throat_radius_m: f64,
    pub expansion_ratio: f64,
    pub exit_radius_m: f64,
    pub exit_area_m2: f64,
    pub divergence_factor: f64,
    pub exit_mach: f64,
    pub exit_pressure_pa: f64,
    pub exit_temp_k: f64,
    pub exhaust_velocity_mps: f64,
    pub thrust_sl_n: f64,
    pub thrust_vac_n: f64,
    pub flow_kg_s: f64,
    pub dry_mass_kg: f64,
    pub nozzle_mass_kg: f64,
    pub chamber_diameter_m: f64,
    pub chamber_length_m: f64,
    pub wall_thickness_m: f64,
    pub position_body_m: [f64; 3],
    pub thrust_axis_body: [f64; 3],
    pub gimbal_range_rad: f64,
}

/// Hangar-compiled propulsion system: shared feed solved once, chambers
/// solved individually, one plume source per nozzle at runtime.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CompiledPropulsionSystem {
    pub name: String,
    pub propellant: Propellant,
    pub cycle: EngineCycle,
    pub chamber_pressure_pa: f64,
    pub gamma: f64,
    pub chamber_temp_k: f64,
    pub gas_constant_j_kg_k: f64,
    pub c_star_mps: f64,
    pub chambers: Vec<CompiledChamber>,
    pub gg_bypass_fraction: f64,
    pub gg_thrust_sl_n: f64,
    pub gg_thrust_vac_n: f64,
    pub gg_exit_area_m2: f64,
    pub total_flow_kg_s: f64,
    pub total_thrust_sl_n: f64,
    pub total_thrust_vac_n: f64,
    pub total_isp_sl_s: f64,
    pub total_isp_vac_s: f64,
    pub dry_mass_kg: f64,
    pub pump_power_w: f64,
    pub feed_pressure_required_pa: f64,
    pub min_throttle: f64,
    pub spool_tau_s: f64,
    pub restartable: bool,
}

/// System operating point: totals plus one point per nozzle (same order),
/// which is what the plume renderer consumes per nozzle.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SystemOperatingPoint {
    pub thrust_n: f64,
    pub mass_flow_kg_s: f64,
    pub isp_s: f64,
    pub chambers: Vec<EngineOperatingPoint>,
}

/// One altitude row for a multi-chamber system at fixed per-chamber
/// throttles (editor analyzer contract, same shape as the single-engine
/// rows plus the chamber count for reference).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SystemAltitudePoint {
    pub altitude_m: f64,
    pub ambient_pa: f64,
    pub thrust_n: f64,
    pub isp_s: f64,
    pub mass_flow_kg_s: f64,
    pub separation_any: bool,
    pub chamber_count: usize,
}

impl ChamberSpec {
    /// Validate chamber authoring (NaN fails closed; axis must be unit).
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    pub fn validate(&self) -> Result<(), PropulsionError> {
        if self.name.trim().is_empty() {
            return Err(PropulsionError::InvalidSpec("chamber needs a name".into()));
        }
        require_positive(self.throat_radius_m, "chamber throat radius")?;
        if !(self.expansion_ratio >= 1.0) {
            return Err(PropulsionError::InvalidSpec(
                "chamber expansion ratio must be finite and >= 1".into(),
            ));
        }
        require_positive(self.nozzle_length_m, "chamber nozzle length")?;
        if self.position_body_m.iter().any(|v| !v.is_finite()) {
            return Err(PropulsionError::InvalidSpec(
                "chamber position must be finite".into(),
            ));
        }
        let axis = self.thrust_axis_body;
        if axis.iter().any(|v| !v.is_finite()) {
            return Err(PropulsionError::InvalidSpec(
                "chamber thrust axis must be finite".into(),
            ));
        }
        let norm = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
        if (norm - 1.0).abs() > 1e-9 {
            return Err(PropulsionError::InvalidSpec(
                "chamber thrust axis must be unit length".into(),
            ));
        }
        require_non_negative(self.gimbal_range_rad, "chamber gimbal range")?;
        if self.gimbal_range_rad > 0.35 {
            return Err(PropulsionError::InvalidSpec(
                "chamber gimbal above 0.35 rad needs an explicit actuator design".into(),
            ));
        }
        Ok(())
    }
}

impl PropulsionSystemSpec {
    /// Validate the system authoring (NaN fails closed).
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    pub fn validate(&self) -> Result<(), PropulsionError> {
        if self.name.trim().is_empty() {
            return Err(PropulsionError::InvalidSpec(
                "propulsion system needs a name".into(),
            ));
        }
        if self.propellant.is_solid() {
            return Err(PropulsionError::InvalidSpec(
                "multi-chamber systems are liquid-fed; solids cluster as separate motors".into(),
            ));
        }
        if matches!(
            self.propellant,
            Propellant::ColdGasNitrogen | Propellant::ColdGasHelium
        ) {
            return Err(PropulsionError::InvalidSpec(
                "cold gas has no chamber to cluster".into(),
            ));
        }
        if self.chambers.is_empty() || self.chambers.len() > MAX_SYSTEM_CHAMBERS {
            return Err(PropulsionError::InvalidSpec(format!(
                "chamber count must be in 1..={MAX_SYSTEM_CHAMBERS}"
            )));
        }
        require_positive(self.chamber_pressure_pa, "chamber pressure")?;
        self.chamber_material.validate()?;
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
        self.propellant.thermo_at_mixture(self.mixture_ratio)?;
        for chamber in &self.chambers {
            chamber.validate()?;
        }
        Ok(())
    }

    /// Hangar compile: shared feed solved once, chambers solved
    /// individually against the same design point.
    pub fn compile(&self) -> Result<CompiledPropulsionSystem, PropulsionError> {
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
        let l_star = self
            .characteristic_length_m
            .unwrap_or(thermo.characteristic_length_m);

        // Per-chamber design points against the shared Pc/thermo.
        let mut chambers = Vec::with_capacity(self.chambers.len());
        let mut total_throat_area_m2 = 0.0;
        for chamber in &self.chambers {
            let throat_area_m2 =
                std::f64::consts::PI * chamber.throat_radius_m * chamber.throat_radius_m;
            let exit_radius_m = chamber.throat_radius_m * chamber.expansion_ratio.sqrt();
            let exit_area_m2 = throat_area_m2 * chamber.expansion_ratio;
            let exit = nozzle_exit(&thermo, chamber.expansion_ratio)?;
            let wall_slope = (exit_radius_m - chamber.throat_radius_m) / chamber.nozzle_length_m;
            let conical = 0.5 * (1.0 + wall_slope.atan().cos());
            let divergence = match chamber.contour {
                NozzleContour::Conical => conical,
                NozzleContour::Bell => conical + (1.0 - conical) * 0.5,
                NozzleContour::Aerospike => 0.99,
            };
            let flow_kg_s = self.chamber_pressure_pa * throat_area_m2 / c_star;
            let thrust_sl_n = thrust_coefficient(
                &thermo,
                self.chamber_pressure_pa,
                &exit,
                chamber.expansion_ratio,
                101_325.0,
                divergence,
            ) * self.chamber_pressure_pa
                * throat_area_m2;
            let thrust_vac_n = thrust_coefficient(
                &thermo,
                self.chamber_pressure_pa,
                &exit,
                chamber.expansion_ratio,
                0.0,
                divergence,
            ) * self.chamber_pressure_pa
                * throat_area_m2;

            // Chamber-local mass: walls, nozzle, injector, head, gimbal.
            // Turbomachinery and the GG duct book once at system level.
            let chamber_volume_m3 = l_star * throat_area_m2;
            let chamber_diameter_m = (4.0 * chamber_volume_m3
                / (std::f64::consts::PI * super::CHAMBER_LENGTH_DIAMETER))
                .powf(1.0 / 3.0);
            let chamber_length_m = super::CHAMBER_LENGTH_DIAMETER * chamber_diameter_m;
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
            let slant_m = (chamber.nozzle_length_m * chamber.nozzle_length_m
                + wall_slope * wall_slope * chamber.nozzle_length_m * chamber.nozzle_length_m)
                .sqrt();
            let nozzle_area_m2 =
                std::f64::consts::PI * (chamber.throat_radius_m + exit_radius_m) * slant_m;
            let nozzle_thickness_m = wall_thickness_m
                * (chamber.throat_radius_m / chamber_diameter_m).min(1.0)
                * self.cooling.nozzle_wall_factor();
            let mut nozzle_kg =
                nozzle_area_m2 * nozzle_thickness_m * self.chamber_material.density_kg_m3;
            if chamber.contour == NozzleContour::Bell {
                nozzle_kg *= 0.85;
            }
            let head_kg = MASS_FIT_HEAD_BASE_KG + MASS_FIT_HEAD_KG_PER_M2 * throat_area_m2;
            let gimbal_kg = if chamber.gimbal_range_rad > 0.0 {
                MASS_FIT_GIMBAL_BASE_KG + MASS_FIT_GIMBAL_KG_PER_N * thrust_vac_n
            } else {
                0.0
            };
            total_throat_area_m2 += throat_area_m2;
            chambers.push(CompiledChamber {
                name: chamber.name.clone(),
                throat_area_m2,
                throat_radius_m: chamber.throat_radius_m,
                expansion_ratio: chamber.expansion_ratio,
                exit_radius_m,
                exit_area_m2,
                divergence_factor: divergence,
                exit_mach: exit.exit_mach,
                exit_pressure_pa: exit.exit_pressure_ratio * self.chamber_pressure_pa,
                exit_temp_k: exit.exit_temp_k,
                exhaust_velocity_mps: exit.exhaust_velocity_mps,
                thrust_sl_n,
                thrust_vac_n,
                flow_kg_s,
                dry_mass_kg: chamber_wall_kg + injector_kg + nozzle_kg + head_kg + gimbal_kg,
                nozzle_mass_kg: nozzle_kg,
                chamber_diameter_m,
                chamber_length_m,
                wall_thickness_m,
                position_body_m: chamber.position_body_m,
                thrust_axis_body: chamber.thrust_axis_body,
                gimbal_range_rad: chamber.gimbal_range_rad,
            });
        }

        // Shared feed: one GG duct on total bypass flow, one turbopump set
        // on total flow, mount/feed fits on total vacuum thrust.
        let total_chamber_flow_kg_s = self.chamber_pressure_pa * total_throat_area_m2 / c_star;
        let gg_flow_kg_s = limits.gg_bypass_fraction * total_chamber_flow_kg_s;
        let total_flow_kg_s = total_chamber_flow_kg_s + gg_flow_kg_s;
        let (gg_thrust_sl_n, gg_thrust_vac_n, gg_exit_area_m2) = if gg_flow_kg_s > 0.0 {
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
            (sl, vac, gg_throat_m2 * GG_DUCT_EXPANSION_RATIO)
        } else {
            (0.0, 0.0, 0.0)
        };
        let chambers_sl_n: f64 = chambers.iter().map(|c| c.thrust_sl_n).sum();
        let chambers_vac_n: f64 = chambers.iter().map(|c| c.thrust_vac_n).sum();
        let total_thrust_sl_n = chambers_sl_n + gg_thrust_sl_n;
        let total_thrust_vac_n = chambers_vac_n + gg_thrust_vac_n;
        let total_isp_sl_s = total_thrust_sl_n / (total_flow_kg_s * STANDARD_GRAVITY_MPS2);
        let total_isp_vac_s = total_thrust_vac_n / (total_flow_kg_s * STANDARD_GRAVITY_MPS2);

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
        let chambers_dry_kg: f64 = chambers.iter().map(|c| c.dry_mass_kg).sum();
        let dry_mass_kg = chambers_dry_kg
            + turbo_kg
            + pump_motor_kg
            + MASS_FIT_MOUNT_KG_PER_N * total_thrust_vac_n
            + MASS_FIT_FEED_KG_PER_N * total_thrust_vac_n;
        let pump_power_w = match self.cycle {
            EngineCycle::ElectricPump => {
                total_flow_kg_s * (self.chamber_pressure_pa - TANK_PRESSURE_PA).max(0.0)
                    / (thermo.bulk_density_kg_m3 * PUMP_EFFICIENCY)
            }
            _ => 0.0,
        };

        Ok(CompiledPropulsionSystem {
            name: self.name.clone(),
            propellant: self.propellant,
            cycle: self.cycle,
            chamber_pressure_pa: self.chamber_pressure_pa,
            gamma: thermo.gamma,
            chamber_temp_k: thermo.chamber_temp_k,
            gas_constant_j_kg_k: thermo.gas_constant_j_kg_k,
            c_star_mps: c_star,
            chambers,
            gg_bypass_fraction: limits.gg_bypass_fraction,
            gg_thrust_sl_n,
            gg_thrust_vac_n,
            gg_exit_area_m2,
            total_flow_kg_s,
            total_thrust_sl_n,
            total_thrust_vac_n,
            total_isp_sl_s,
            total_isp_vac_s,
            dry_mass_kg,
            pump_power_w,
            feed_pressure_required_pa: self.chamber_pressure_pa
                * (1.0 + super::INJECTOR_DROP_FRACTION),
            min_throttle: self.min_throttle.unwrap_or(limits.min_throttle),
            spool_tau_s: limits.spool_tau_s,
            restartable: self.restartable,
        })
    }
}

impl CompiledPropulsionSystem {
    fn thermo_ref(&self) -> PropellantThermo {
        PropellantThermo {
            gamma: self.gamma,
            chamber_temp_k: self.chamber_temp_k,
            gas_constant_j_kg_k: self.gas_constant_j_kg_k,
            bulk_density_kg_m3: 0.0,
            characteristic_length_m: 0.0,
            reference_c_star_mps: 0.0,
        }
    }

    fn chamber_exit(&self, chamber: &CompiledChamber) -> NozzleExitState {
        NozzleExitState {
            exit_mach: chamber.exit_mach,
            exit_pressure_ratio: chamber.exit_pressure_pa / self.chamber_pressure_pa,
            exit_temp_k: chamber.exit_temp_k,
            exhaust_velocity_mps: chamber.exhaust_velocity_mps,
        }
    }

    /// Steady-state operating point at per-chamber throttles and ambient.
    /// Each chamber scales its own pressure linearly (documented
    /// deep-throttle assumption); the shared GG duct follows total flow.
    pub fn operating_point(
        &self,
        throttles: &[f64],
        ambient_pa: f64,
    ) -> Result<SystemOperatingPoint, PropulsionError> {
        if !ambient_pa.is_finite() || ambient_pa < 0.0 {
            return Err(PropulsionError::InvalidCommand(
                "ambient pressure must be finite and >= 0".into(),
            ));
        }
        if throttles.len() != self.chambers.len() {
            return Err(PropulsionError::InvalidCommand(format!(
                "expected {} throttles, got {}",
                self.chambers.len(),
                throttles.len()
            )));
        }
        let thermo = self.thermo_ref();
        let mut points = Vec::with_capacity(self.chambers.len());
        let mut flow_kg_s = 0.0;
        let mut main_thrust_n = 0.0;
        for (chamber, throttle) in self.chambers.iter().zip(throttles) {
            if !throttle.is_finite() || !(0.0..=1.0).contains(throttle) {
                return Err(PropulsionError::InvalidCommand(
                    "chamber throttle must be finite in [0, 1]".into(),
                ));
            }
            if *throttle == 0.0 {
                points.push(EngineOperatingPoint {
                    thrust_n: 0.0,
                    mass_flow_kg_s: 0.0,
                    isp_s: 0.0,
                    exhaust_velocity_mps: chamber.exhaust_velocity_mps,
                    exit_pressure_pa: 0.0,
                    exit_temp_k: chamber.exit_temp_k,
                    exit_mach: chamber.exit_mach,
                    separation_risk: false,
                });
                continue;
            }
            let chamber_pa = self.chamber_pressure_pa * throttle;
            let exit_pressure_pa = chamber.exit_pressure_pa * throttle;
            let chamber_flow = chamber.flow_kg_s * throttle;
            let cf = thrust_coefficient(
                &thermo,
                chamber_pa,
                &self.chamber_exit(chamber),
                chamber.expansion_ratio,
                ambient_pa,
                chamber.divergence_factor,
            );
            let thrust_n = cf * chamber_pa * chamber.throat_area_m2;
            flow_kg_s += chamber_flow;
            main_thrust_n += thrust_n;
            points.push(EngineOperatingPoint {
                thrust_n,
                mass_flow_kg_s: chamber_flow,
                isp_s: thrust_n / (chamber_flow * STANDARD_GRAVITY_MPS2),
                exhaust_velocity_mps: chamber.exhaust_velocity_mps,
                exit_pressure_pa,
                exit_temp_k: chamber.exit_temp_k,
                exit_mach: chamber.exit_mach,
                separation_risk: exit_pressure_pa < super::SEPARATION_PRESSURE_RATIO * ambient_pa,
            });
        }
        // Shared duct follows the flow fraction (frozen duct geometry).
        let flow_fraction = if self.total_flow_kg_s > 0.0 {
            (flow_kg_s * (1.0 + self.gg_bypass_fraction) / self.total_flow_kg_s).min(1.0)
        } else {
            0.0
        };
        let gg_thrust_n =
            (self.gg_thrust_vac_n * flow_fraction - ambient_pa * self.gg_exit_area_m2).max(0.0);
        let thrust_n = main_thrust_n + gg_thrust_n;
        let total_flow = flow_kg_s * (1.0 + self.gg_bypass_fraction);
        Ok(SystemOperatingPoint {
            thrust_n,
            mass_flow_kg_s: total_flow,
            isp_s: if total_flow > 0.0 {
                thrust_n / (total_flow * STANDARD_GRAVITY_MPS2)
            } else {
                0.0
            },
            chambers: points,
        })
    }

    /// Plume-renderer inputs: one source per nozzle, same order as chambers.
    pub fn plume_states(&self, point: &SystemOperatingPoint) -> Vec<EnginePlumeState> {
        self.chambers
            .iter()
            .zip(point.chambers.iter())
            .map(|(chamber, chamber_point)| EnginePlumeState {
                exit_radius_m: chamber.exit_radius_m,
                mass_flow_kg_s: chamber_point.mass_flow_kg_s,
                exhaust_velocity_mps: chamber_point.exhaust_velocity_mps,
                exit_pressure_pa: chamber_point.exit_pressure_pa,
                exit_temp_k: chamber_point.exit_temp_k,
                exit_mach: chamber_point.exit_mach,
                propellant: self.propellant,
            })
            .collect()
    }

    /// Force/moment wrench in body axes at per-chamber throttles:
    /// differential throttle steers through station offsets for free. The
    /// shared GG duct (no station of its own) distributes proportionally,
    /// same rule as the vehicle total.
    pub fn wrench_body_n(
        &self,
        throttles: &[f64],
        ambient_pa: f64,
    ) -> Result<(DVec3, DVec3), PropulsionError> {
        let point = self.operating_point(throttles, ambient_pa)?;
        let main: f64 = point.chambers.iter().map(|c| c.thrust_n).sum();
        let scale = if main > 0.0 {
            point.thrust_n / main
        } else {
            1.0
        };
        let mut force = DVec3::ZERO;
        let mut moment = DVec3::ZERO;
        for (chamber, chamber_point) in self.chambers.iter().zip(point.chambers.iter()) {
            let thrust =
                DVec3::from_array(chamber.thrust_axis_body) * (chamber_point.thrust_n * scale);
            force += thrust;
            moment += DVec3::from_array(chamber.position_body_m).cross(thrust);
        }
        Ok((force, moment))
    }

    /// Gimbal authority of one chamber for the flight allocator.
    pub fn gimbal_authority(
        &self,
        chamber_index: usize,
        throttle: f64,
        ambient_pa: f64,
    ) -> Result<[GimbalEffector; 2], PropulsionError> {
        let chamber = self.chambers.get(chamber_index).ok_or_else(|| {
            PropulsionError::InvalidCommand(format!("no chamber at index {chamber_index}"))
        })?;
        let mut throttles = vec![0.0; self.chambers.len()];
        throttles[chamber_index] = throttle;
        let point = self.operating_point(&throttles, ambient_pa)?;
        let thrust =
            DVec3::from_array(chamber.thrust_axis_body) * point.chambers[chamber_index].thrust_n;
        Ok(gimbal_pair(
            chamber.position_body_m,
            chamber.thrust_axis_body,
            thrust.to_array(),
            chamber.gimbal_range_rad,
        ))
    }

    /// Cold per-chamber spool states (same shape as the single-engine
    /// runtime; advance each with [`CompiledPropulsionSystem::advance_chamber_spool`]).
    pub fn new_spools(&self) -> Vec<EngineSpool> {
        vec![
            EngineSpool {
                running: false,
                throttle_actual: 0.0,
                shots_remaining: if self.restartable { None } else { Some(1) },
                burn_elapsed_s: 0.0,
            };
            self.chambers.len()
        ]
    }

    /// Advance one chamber toward a throttle command: the same first-order
    /// spool law as the single-engine runtime, with the system floor/tau.
    pub fn advance_chamber_spool(
        &self,
        mut state: EngineSpool,
        throttle_cmd: f64,
        dt_s: f64,
    ) -> Result<EngineSpool, PropulsionError> {
        if !throttle_cmd.is_finite() || !(0.0..=1.0).contains(&throttle_cmd) {
            return Err(PropulsionError::InvalidCommand(
                "throttle command must be finite in [0, 1]".into(),
            ));
        }
        if !dt_s.is_finite() || dt_s < 0.0 {
            return Err(PropulsionError::InvalidCommand(
                "spool dt must be finite and >= 0".into(),
            ));
        }
        let target = if throttle_cmd == 0.0 {
            0.0
        } else {
            throttle_cmd.max(self.min_throttle)
        };
        if !state.running {
            if target == 0.0 {
                state.throttle_actual = 0.0;
                return Ok(state);
            }
            if let Some(shots) = state.shots_remaining {
                if shots == 0 {
                    return Err(PropulsionError::InvalidCommand(
                        "no ignition shots remaining".into(),
                    ));
                }
                state.shots_remaining = Some(shots - 1);
            }
            state.running = true;
        } else if target == 0.0 {
            state.running = false;
        }
        let alpha = 1.0 - (-dt_s / self.spool_tau_s).exp();
        state.throttle_actual += (target - state.throttle_actual) * alpha;
        if !state.running && state.throttle_actual < 1e-6 {
            state.throttle_actual = 0.0;
        }
        Ok(state)
    }

    /// Thrust/Isp versus altitude at fixed per-chamber throttles (editor
    /// analyzer contract for clustered engines).
    pub fn analyze_system_altitude(
        &self,
        atmosphere: &AtmosphereConfig,
        altitudes_m: &[f64],
        throttles: &[f64],
    ) -> Result<Vec<SystemAltitudePoint>, PropulsionError> {
        if altitudes_m.is_empty() {
            return Err(PropulsionError::InvalidSpec(
                "analyzer needs at least one altitude".into(),
            ));
        }
        let mut curve = Vec::with_capacity(altitudes_m.len());
        for altitude_m in altitudes_m {
            if !altitude_m.is_finite() {
                return Err(PropulsionError::InvalidSpec(
                    "analyzer altitudes must be finite".into(),
                ));
            }
            let sample = atmosphere.sample(*altitude_m).map_err(|error| {
                PropulsionError::InvalidSpec(format!("atmosphere sample: {error}"))
            })?;
            let point = self.operating_point(throttles, sample.pressure_pa)?;
            curve.push(SystemAltitudePoint {
                altitude_m: *altitude_m,
                ambient_pa: sample.pressure_pa,
                thrust_n: point.thrust_n,
                isp_s: point.isp_s,
                mass_flow_kg_s: point.mass_flow_kg_s,
                separation_any: point.chambers.iter().any(|c| c.separation_risk),
                chamber_count: self.chambers.len(),
            });
        }
        Ok(curve)
    }
}

/// One propulsion system installed on a vehicle. Chambers carry their own
/// body-frame stations, so the mount is just a name plus the compiled
/// system; mass aggregates per chamber at bake time.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SystemMount {
    pub name: String,
    pub system: CompiledPropulsionSystem,
}

impl SystemMount {
    /// Validate mount data (NaN fails closed through the chambers).
    pub fn validate(&self) -> Result<(), PropulsionError> {
        if self.name.trim().is_empty() {
            return Err(PropulsionError::InvalidSpec(
                "propulsion system mount needs a name".into(),
            ));
        }
        if self.system.chambers.is_empty() {
            return Err(PropulsionError::InvalidSpec(
                "mounted system has no chambers".into(),
            ));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::super::liquid::merlin_like;
    use super::super::{ChamberMaterial, CoolingMode, EngineCycle, Propellant};
    use super::*;

    fn twin_chamber(gimbal: f64) -> PropulsionSystemSpec {
        PropulsionSystemSpec {
            name: "twin".into(),
            propellant: Propellant::LoxRp1,
            cycle: EngineCycle::GasGenerator,
            chamber_pressure_pa: 9.7e6,
            mixture_ratio: None,
            chamber_material: ChamberMaterial::nickel_superalloy(),
            cooling: CoolingMode::Regenerative,
            characteristic_length_m: None,
            min_throttle: None,
            restartable: true,
            chambers: vec![
                ChamberSpec {
                    name: "a".into(),
                    throat_radius_m: 0.134,
                    expansion_ratio: 16.0,
                    nozzle_length_m: 1.5,
                    contour: NozzleContour::Bell,
                    position_body_m: [-3.0, 0.0, 0.5],
                    thrust_axis_body: [1.0, 0.0, 0.0],
                    gimbal_range_rad: gimbal,
                },
                ChamberSpec {
                    name: "b".into(),
                    throat_radius_m: 0.100,
                    expansion_ratio: 16.0,
                    nozzle_length_m: 1.2,
                    contour: NozzleContour::Bell,
                    position_body_m: [-3.0, 0.0, -0.5],
                    thrust_axis_body: [1.0, 0.0, 0.0],
                    gimbal_range_rad: gimbal,
                },
            ],
        }
    }

    #[test]
    fn single_chamber_matches_liquid_compile() {
        // One chamber through the system path must reproduce the standalone
        // liquid compile bit-for-bit: same kernels, same fits, single GG
        // duct, single turbopump set.
        let spec = PropulsionSystemSpec {
            chambers: vec![ChamberSpec {
                name: "only".into(),
                throat_radius_m: 0.134,
                expansion_ratio: 16.0,
                nozzle_length_m: 1.5,
                contour: NozzleContour::Bell,
                position_body_m: [0.0, 0.0, 0.0],
                thrust_axis_body: [1.0, 0.0, 0.0],
                gimbal_range_rad: 0.09,
            }],
            ..twin_chamber(0.09)
        };
        let system = spec.compile().expect("system compiles");
        let single = merlin_like().compile().expect("liquid compiles");
        for (a, b, label) in [
            (system.total_thrust_vac_n, single.thrust_vac_n, "vac thrust"),
            (system.total_thrust_sl_n, single.thrust_sl_n, "SL thrust"),
            (system.total_flow_kg_s, single.full_flow_kg_s, "flow"),
            (system.dry_mass_kg, single.dry_mass_kg, "dry mass"),
            (system.total_isp_vac_s, single.isp_vac_s, "vac Isp"),
        ] {
            let drift = (a - b).abs() / b.abs().max(1e-12);
            assert!(drift < 1e-9, "{label} drift {drift:e}");
        }
        // Runtime parity at full and half throttle too.
        let sys_full = system.operating_point(&[1.0], 0.0).expect("system full");
        let liq_full = super::super::CompiledEngine::Liquid(single.clone())
            .operating_point(1.0, 0.0, 0.0)
            .expect("liquid full");
        assert!((sys_full.thrust_n - liq_full.thrust_n).abs() / liq_full.thrust_n < 1e-9);
        let sys_half = system
            .operating_point(&[0.5], 101_325.0)
            .expect("system half");
        let liq_half = super::super::CompiledEngine::Liquid(single)
            .operating_point(0.5, 101_325.0, 0.0)
            .expect("liquid half");
        assert!((sys_half.thrust_n - liq_half.thrust_n).abs() / liq_half.thrust_n < 1e-9);
    }

    #[test]
    fn differential_throttle_steers_through_stations() {
        // Twin chambers 1 m apart in Z at full vs half: the moment about Y
        // equals the thrust difference times the arm, exactly. Staged
        // cycle (no GG duct) keeps force and moment on the same station
        // bookkeeping.
        let spec = PropulsionSystemSpec {
            cycle: EngineCycle::StagedCombustion,
            ..twin_chamber(0.0)
        };
        let system = spec.compile().expect("twin compiles");
        let point = system
            .operating_point(&[1.0, 0.5], 0.0)
            .expect("differential");
        let (force, moment) = system.wrench_body_n(&[1.0, 0.5], 0.0).expect("wrench");
        assert!((force.x - point.thrust_n).abs() / point.thrust_n < 1e-12);
        assert!(force.y.abs() < 1e-6 && force.z.abs() < 1e-6);
        let fa = point.chambers[0].thrust_n;
        let fb = point.chambers[1].thrust_n;
        // r_a = (…, +0.5z), r_b = (…, −0.5z): M_y = 0.5·(fa − fb).
        assert!((moment.y - 0.5 * (fa - fb)).abs() / fa < 1e-9);
        assert!(moment.x.abs() / fa < 1e-9 && moment.z.abs() / fa < 1e-9);
        // One plume source per nozzle, flows summing to the total.
        let plumes = system.plume_states(&point);
        assert_eq!(plumes.len(), 2);
        let plume_flow: f64 = plumes.iter().map(|p| p.mass_flow_kg_s).sum();
        assert!((plume_flow - point.mass_flow_kg_s).abs() / point.mass_flow_kg_s < 1e-9);
        // Gimbal authority of chamber a matches its own lever arm.
        let authority = system.gimbal_authority(0, 1.0, 0.0).expect("authority");
        assert_eq!(authority.len(), 2);
    }

    #[test]
    fn chamber_spools_stay_independent() {
        // Per-chamber spool states ignite and shut down independently under
        // the same first-order law as single engines.
        let system = twin_chamber(0.09).compile().expect("twin compiles");
        let mut spools = system.new_spools();
        assert_eq!(spools.len(), 2);
        spools[0] = system
            .advance_chamber_spool(spools[0], 1.0, 10.0)
            .expect("ignite a");
        assert!(spools[0].running);
        assert!(!spools[1].running);
        assert!((spools[0].throttle_actual - 1.0).abs() < 1e-6);
        spools[0] = system
            .advance_chamber_spool(spools[0], 0.0, 10.0)
            .expect("shutdown a");
        assert!(!spools[0].running);
        assert!(system.operating_point(&[1.0], 0.0).is_err());
    }

    #[test]
    fn system_validation_refuses_garbage() {
        let empty = PropulsionSystemSpec {
            chambers: vec![],
            ..twin_chamber(0.0)
        };
        assert!(empty.compile().is_err());
        let bad_axis = PropulsionSystemSpec {
            chambers: vec![ChamberSpec {
                thrust_axis_body: [2.0, 0.0, 0.0],
                ..twin_chamber(0.0).chambers[0].clone()
            }],
            ..twin_chamber(0.0)
        };
        assert!(bad_axis.compile().is_err());
        let solid_fuel = PropulsionSystemSpec {
            propellant: Propellant::SolidApcp,
            ..twin_chamber(0.0)
        };
        assert!(solid_fuel.compile().is_err());
    }
}
