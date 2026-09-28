//! Reaction control thrusters: monopropellant pulse engines, cold-gas
//! blowdown thrusters, and mounted clusters producing force/moment impulses.
//!
//! Monopropellant hydrazine compiles through the shared liquid path
//! (pressure-fed chamber, fixed full thrust — commands are on-times, not
//! throttle). Cold gas has no chamber to size, so it compiles on its own
//! path: rated at storage pressure, runtime thrust scales with actual inlet
//! pressure exactly (choked flow at frozen geometry). Pulse physics books a
//! triangular valve rise: short pulses deliver less impulse per propellant,
//! which is where the minimum impulse bit comes from — no efficiency curve
//! is fitted.
//!
//! Plume-family mapping for the game-side choke point: hydrazine reads as
//! hypergolic-like, nitrogen/helium as cold-gas.

// Validity guards use `!(x > 0.0)` so NaN fails closed (repo convention).
#![allow(clippy::neg_cmp_op_on_partial_ord)]

use glam::DVec3;
use serde::{Deserialize, Serialize};

use super::{
    ChamberMaterial, CompiledLiquid, CoolingMode, EngineCycle, EngineMount, EngineOperatingPoint,
    LiquidEngineSpec, NozzleContour, Propellant, PropellantThermo, PropulsionError,
    STANDARD_GRAVITY_MPS2, characteristic_velocity, nozzle_exit, require_non_negative,
    require_positive, thrust_coefficient,
};

/// Default valve rise time (s, documented typical RCS valve).
pub const RCS_DEFAULT_RISE_TIME_S: f64 = 0.005;
/// Default minimum valve on-time (s, documented typical RCS driver).
pub const RCS_DEFAULT_MIN_ON_TIME_S: f64 = 0.020;

/// One delivered pulse: impulse, booked propellant, and their ratio.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct RcsPulse {
    pub impulse_ns: f64,
    pub propellant_kg: f64,
    pub effective_isp_s: f64,
}

/// Triangular-rise pulse delivery shared by all RCS families: thrust rises
/// linearly over `rise_s`, propellant books the full open time.
fn deliver_pulse(thrust_n: f64, mass_flow_kg_s: f64, on_time_s: f64, rise_s: f64) -> RcsPulse {
    if on_time_s <= 0.0 {
        return RcsPulse {
            impulse_ns: 0.0,
            propellant_kg: 0.0,
            effective_isp_s: 0.0,
        };
    }
    let impulse_ns = if on_time_s <= rise_s {
        thrust_n * on_time_s * on_time_s / (2.0 * rise_s)
    } else {
        thrust_n * (on_time_s - rise_s / 2.0)
    };
    let propellant_kg = mass_flow_kg_s * on_time_s;
    RcsPulse {
        impulse_ns,
        propellant_kg,
        effective_isp_s: if propellant_kg > 0.0 {
            impulse_ns / (propellant_kg * STANDARD_GRAVITY_MPS2)
        } else {
            0.0
        },
    }
}

// ---------------------------------------------------------------------------
// Cold gas
// ---------------------------------------------------------------------------

/// Cold-gas thruster authoring: stored gas expanded through a nozzle.
/// Rated at `rated_pressure_pa`; runtime thrust follows actual inlet
/// pressure (tank blowdown) exactly.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ColdGasThrusterSpec {
    pub name: String,
    /// ColdGasNitrogen or ColdGasHelium (validated).
    pub gas: Propellant,
    /// Storage/stagnation temperature (K).
    pub storage_temp_k: f64,
    /// Rated inlet pressure (Pa).
    pub rated_pressure_pa: f64,
    pub throat_radius_m: f64,
    pub expansion_ratio: f64,
    pub nozzle_length_m: f64,
    pub contour: NozzleContour,
    pub material: ChamberMaterial,
    /// Valve rise time (s).
    pub valve_rise_time_s: f64,
    /// Minimum valve on-time (s, must clear the rise).
    pub min_on_time_s: f64,
}

/// Hangar-compiled cold-gas thruster.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CompiledColdGas {
    pub name: String,
    pub gas: Propellant,
    pub storage_temp_k: f64,
    pub rated_pressure_pa: f64,
    pub throat_area_m2: f64,
    pub expansion_ratio: f64,
    pub exit_radius_m: f64,
    pub divergence_factor: f64,
    pub gamma: f64,
    pub gas_constant_j_kg_k: f64,
    pub c_star_mps: f64,
    pub exit_mach: f64,
    pub exit_temp_k: f64,
    pub exhaust_velocity_mps: f64,
    pub rated_thrust_vac_n: f64,
    pub rated_thrust_sl_n: f64,
    pub rated_flow_kg_s: f64,
    pub rated_isp_vac_s: f64,
    pub dry_mass_kg: f64,
    pub valve_rise_time_s: f64,
    pub min_on_time_s: f64,
}

impl ColdGasThrusterSpec {
    /// Validate authoring values (NaN fails closed).
    pub fn validate(&self) -> Result<(), PropulsionError> {
        if self.name.trim().is_empty() {
            return Err(PropulsionError::InvalidSpec(
                "thruster name must not be empty".into(),
            ));
        }
        if !matches!(
            self.gas,
            Propellant::ColdGasNitrogen | Propellant::ColdGasHelium
        ) {
            return Err(PropulsionError::InvalidSpec(
                "cold-gas thruster needs a cold-gas propellant".into(),
            ));
        }
        require_positive(self.storage_temp_k, "storage temperature")?;
        require_positive(self.rated_pressure_pa, "rated pressure")?;
        require_positive(self.throat_radius_m, "throat radius")?;
        if !(self.expansion_ratio >= 1.0) {
            return Err(PropulsionError::InvalidSpec(
                "expansion ratio must be finite and >= 1".into(),
            ));
        }
        require_positive(self.nozzle_length_m, "nozzle length")?;
        self.material.validate()?;
        require_positive(self.valve_rise_time_s, "valve rise time")?;
        require_positive(self.min_on_time_s, "min on-time")?;
        if self.min_on_time_s < self.valve_rise_time_s {
            return Err(PropulsionError::InvalidSpec(
                "min on-time must clear the valve rise".into(),
            ));
        }
        Ok(())
    }

    /// Hangar compile at the rated inlet pressure.
    pub fn compile(&self) -> Result<CompiledColdGas, PropulsionError> {
        self.validate()?;
        let thermo = PropellantThermo {
            chamber_temp_k: self.storage_temp_k,
            ..self.gas.thermo()
        };
        let c_star = characteristic_velocity(&thermo);
        let throat_area_m2 = std::f64::consts::PI * self.throat_radius_m * self.throat_radius_m;
        let exit_radius_m = self.throat_radius_m * self.expansion_ratio.sqrt();
        let exit = nozzle_exit(&thermo, self.expansion_ratio)?;
        let wall_slope = (exit_radius_m - self.throat_radius_m) / self.nozzle_length_m;
        let conical = 0.5 * (1.0 + wall_slope.atan().cos());
        let divergence = match self.contour {
            NozzleContour::Conical => conical,
            NozzleContour::Bell => conical + (1.0 - conical) * 0.5,
            NozzleContour::Aerospike => 0.99,
        };
        let rated_flow_kg_s = self.rated_pressure_pa * throat_area_m2 / c_star;
        let rated_thrust_vac_n = thrust_coefficient(
            &thermo,
            self.rated_pressure_pa,
            &exit,
            self.expansion_ratio,
            0.0,
            divergence,
        ) * self.rated_pressure_pa
            * throat_area_m2;
        let rated_thrust_sl_n = thrust_coefficient(
            &thermo,
            self.rated_pressure_pa,
            &exit,
            self.expansion_ratio,
            101_325.0,
            divergence,
        ) * self.rated_pressure_pa
            * throat_area_m2;
        // Cold wall, no chamber: thin frustum at rating plus a documented
        // valve/manifold allowance.
        let slant_m = (self.nozzle_length_m * self.nozzle_length_m
            + (exit_radius_m - self.throat_radius_m).powi(2))
        .sqrt();
        let wall_m = self.rated_pressure_pa * self.throat_radius_m
            / (2.0 * self.material.yield_strength_pa)
            * 1.5
            * 0.5;
        let nozzle_kg = std::f64::consts::PI
            * (self.throat_radius_m + exit_radius_m)
            * slant_m
            * wall_m
            * self.material.density_kg_m3;
        let dry_mass_kg = nozzle_kg + 0.4;
        Ok(CompiledColdGas {
            name: self.name.clone(),
            gas: self.gas,
            storage_temp_k: self.storage_temp_k,
            rated_pressure_pa: self.rated_pressure_pa,
            throat_area_m2,
            expansion_ratio: self.expansion_ratio,
            exit_radius_m,
            divergence_factor: divergence,
            gamma: thermo.gamma,
            gas_constant_j_kg_k: thermo.gas_constant_j_kg_k,
            c_star_mps: c_star,
            exit_mach: exit.exit_mach,
            exit_temp_k: exit.exit_temp_k,
            exhaust_velocity_mps: exit.exhaust_velocity_mps,
            rated_thrust_vac_n,
            rated_thrust_sl_n,
            rated_flow_kg_s,
            rated_isp_vac_s: rated_thrust_vac_n / (rated_flow_kg_s * STANDARD_GRAVITY_MPS2),
            dry_mass_kg,
            valve_rise_time_s: self.valve_rise_time_s,
            min_on_time_s: self.min_on_time_s,
        })
    }
}

impl CompiledColdGas {
    fn thermo_ref(&self) -> PropellantThermo {
        PropellantThermo {
            gamma: self.gamma,
            chamber_temp_k: self.storage_temp_k,
            gas_constant_j_kg_k: self.gas_constant_j_kg_k,
            bulk_density_kg_m3: 0.0,
            characteristic_length_m: 0.0,
            reference_c_star_mps: 0.0,
        }
    }

    /// Steady operating point at inlet pressure and ambient. Exact choked
    /// scaling: thrust and flow track inlet pressure linearly at frozen
    /// geometry (pinned by test).
    pub fn steady_point(
        &self,
        inlet_pressure_pa: f64,
        ambient_pa: f64,
    ) -> Result<EngineOperatingPoint, PropulsionError> {
        if !(inlet_pressure_pa >= 0.0) {
            return Err(PropulsionError::InvalidCommand(
                "inlet pressure must be finite and >= 0".into(),
            ));
        }
        if !ambient_pa.is_finite() || ambient_pa < 0.0 {
            return Err(PropulsionError::InvalidCommand(
                "ambient pressure must be finite and >= 0".into(),
            ));
        }
        if inlet_pressure_pa == 0.0 {
            return Ok(EngineOperatingPoint {
                thrust_n: 0.0,
                mass_flow_kg_s: 0.0,
                isp_s: 0.0,
                exhaust_velocity_mps: self.exhaust_velocity_mps,
                exit_pressure_pa: 0.0,
                exit_temp_k: self.exit_temp_k,
                exit_mach: self.exit_mach,
                separation_risk: false,
            });
        }
        let thermo = self.thermo_ref();
        let exit = nozzle_exit(&thermo, self.expansion_ratio)?;
        let cf = thrust_coefficient(
            &thermo,
            inlet_pressure_pa,
            &exit,
            self.expansion_ratio,
            ambient_pa,
            self.divergence_factor,
        );
        let thrust_n = cf * inlet_pressure_pa * self.throat_area_m2;
        let flow_kg_s = inlet_pressure_pa * self.throat_area_m2 / self.c_star_mps;
        let exit_pressure_pa = exit.exit_pressure_ratio * inlet_pressure_pa;
        Ok(EngineOperatingPoint {
            thrust_n,
            mass_flow_kg_s: flow_kg_s,
            isp_s: thrust_n / (flow_kg_s * STANDARD_GRAVITY_MPS2),
            exhaust_velocity_mps: self.exhaust_velocity_mps,
            exit_pressure_pa,
            exit_temp_k: self.exit_temp_k,
            exit_mach: self.exit_mach,
            separation_risk: exit_pressure_pa < super::SEPARATION_PRESSURE_RATIO * ambient_pa,
        })
    }

    /// One valve pulse at inlet pressure and ambient.
    pub fn pulse(
        &self,
        on_time_s: f64,
        inlet_pressure_pa: f64,
        ambient_pa: f64,
    ) -> Result<RcsPulse, PropulsionError> {
        if !(on_time_s >= 0.0) {
            return Err(PropulsionError::InvalidCommand(
                "pulse on-time must be finite and >= 0".into(),
            ));
        }
        let steady = self.steady_point(inlet_pressure_pa, ambient_pa)?;
        Ok(deliver_pulse(
            steady.thrust_n,
            steady.mass_flow_kg_s,
            on_time_s,
            self.valve_rise_time_s,
        ))
    }

    /// Minimum impulse bit at rated pressure in vacuum.
    pub fn min_impulse_bit_ns(&self) -> f64 {
        self.pulse(self.min_on_time_s, self.rated_pressure_pa, 0.0)
            .map(|pulse| pulse.impulse_ns)
            .unwrap_or(0.0)
    }
}

// ---------------------------------------------------------------------------
// Monopropellant
// ---------------------------------------------------------------------------

/// Monopropellant hydrazine RCS authoring: catalyst-bed chamber compiled
/// through the shared liquid path at fixed full thrust.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MonopropThrusterSpec {
    pub name: String,
    /// Design chamber pressure (Pa).
    pub chamber_pressure_pa: f64,
    pub throat_radius_m: f64,
    pub expansion_ratio: f64,
    pub nozzle_length_m: f64,
    pub contour: NozzleContour,
    pub material: ChamberMaterial,
    pub valve_rise_time_s: f64,
    pub min_on_time_s: f64,
}

/// Hangar-compiled monopropellant thruster (fixed full thrust; commands
/// are on-times, never throttle).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CompiledMonoprop {
    pub engine: CompiledLiquid,
    pub valve_rise_time_s: f64,
    pub min_on_time_s: f64,
}

impl MonopropThrusterSpec {
    /// Validate authoring values (NaN fails closed).
    pub fn validate(&self) -> Result<(), PropulsionError> {
        if self.name.trim().is_empty() {
            return Err(PropulsionError::InvalidSpec(
                "thruster name must not be empty".into(),
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
        self.material.validate()?;
        require_positive(self.valve_rise_time_s, "valve rise time")?;
        require_positive(self.min_on_time_s, "min on-time")?;
        if self.min_on_time_s < self.valve_rise_time_s {
            return Err(PropulsionError::InvalidSpec(
                "min on-time must clear the valve rise".into(),
            ));
        }
        Ok(())
    }

    /// Hangar compile through the shared pressure-fed liquid path.
    pub fn compile(&self) -> Result<CompiledMonoprop, PropulsionError> {
        self.validate()?;
        let engine = LiquidEngineSpec {
            name: self.name.clone(),
            propellant: Propellant::MonopropHydrazine,
            cycle: EngineCycle::PressureFed,
            chamber_pressure_pa: self.chamber_pressure_pa,
            throat_radius_m: self.throat_radius_m,
            expansion_ratio: self.expansion_ratio,
            nozzle_length_m: self.nozzle_length_m,
            contour: self.contour,
            chamber_material: self.material,
            cooling: CoolingMode::Regenerative,
            mixture_ratio: None,
            characteristic_length_m: None,
            gimbal_range_rad: 0.0,
            min_throttle: Some(1.0),
            restartable: true,
        }
        .compile()?;
        Ok(CompiledMonoprop {
            engine,
            valve_rise_time_s: self.valve_rise_time_s,
            min_on_time_s: self.min_on_time_s,
        })
    }
}

impl CompiledMonoprop {
    /// Steady operating point at full thrust and ambient.
    pub fn steady_point(&self, ambient_pa: f64) -> Result<EngineOperatingPoint, PropulsionError> {
        super::CompiledEngine::Liquid(self.engine.clone()).operating_point(1.0, ambient_pa, 0.0)
    }

    /// One valve pulse at ambient.
    pub fn pulse(&self, on_time_s: f64, ambient_pa: f64) -> Result<RcsPulse, PropulsionError> {
        if !(on_time_s >= 0.0) {
            return Err(PropulsionError::InvalidCommand(
                "pulse on-time must be finite and >= 0".into(),
            ));
        }
        let steady = self.steady_point(ambient_pa)?;
        Ok(deliver_pulse(
            steady.thrust_n,
            steady.mass_flow_kg_s,
            on_time_s,
            self.valve_rise_time_s,
        ))
    }

    /// Minimum impulse bit in vacuum.
    pub fn min_impulse_bit_ns(&self) -> f64 {
        self.pulse(self.min_on_time_s, 0.0)
            .map(|pulse| pulse.impulse_ns)
            .unwrap_or(0.0)
    }
}

// ---------------------------------------------------------------------------
// Unified thrusters + clusters
// ---------------------------------------------------------------------------

/// One RCS thruster of either family for cluster assembly.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum RcsThruster {
    Monoprop(CompiledMonoprop),
    ColdGas(CompiledColdGas),
}

impl RcsThruster {
    /// Display name.
    pub fn name(&self) -> &str {
        match self {
            Self::Monoprop(thruster) => &thruster.engine.name,
            Self::ColdGas(thruster) => &thruster.name,
        }
    }

    /// Steady thrust (N) at inlet pressure (cold gas) and ambient.
    /// Monopropellant ignores inlet pressure (regulated feed, documented).
    pub fn steady_thrust_n(
        &self,
        inlet_pressure_pa: f64,
        ambient_pa: f64,
    ) -> Result<f64, PropulsionError> {
        match self {
            Self::Monoprop(thruster) => Ok(thruster.steady_point(ambient_pa)?.thrust_n),
            Self::ColdGas(thruster) => Ok(thruster
                .steady_point(inlet_pressure_pa, ambient_pa)?
                .thrust_n),
        }
    }

    /// One valve pulse at inlet pressure and ambient.
    pub fn pulse(
        &self,
        on_time_s: f64,
        inlet_pressure_pa: f64,
        ambient_pa: f64,
    ) -> Result<RcsPulse, PropulsionError> {
        match self {
            Self::Monoprop(thruster) => thruster.pulse(on_time_s, ambient_pa),
            Self::ColdGas(thruster) => thruster.pulse(on_time_s, inlet_pressure_pa, ambient_pa),
        }
    }

    /// Minimum impulse bit (N·s) at rated feed in vacuum.
    pub fn min_impulse_bit_ns(&self, rated_inlet_pa: f64) -> f64 {
        match self {
            Self::Monoprop(thruster) => thruster.min_impulse_bit_ns(),
            Self::ColdGas(thruster) => thruster
                .pulse(thruster.min_on_time_s, rated_inlet_pa, 0.0)
                .map(|pulse| pulse.impulse_ns)
                .unwrap_or(0.0),
        }
    }
}

/// One thruster installed on the airframe.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RcsMount {
    pub name: String,
    pub thruster: RcsThruster,
    /// Nozzle station in vehicle body metres.
    pub position_body_m: [f64; 3],
    /// Unit thrust direction in vehicle body axes.
    pub direction_body: [f64; 3],
}

impl RcsMount {
    /// Validate mount data (NaN fails closed; direction must be unit).
    pub fn validate(&self) -> Result<(), PropulsionError> {
        if self.name.trim().is_empty() {
            return Err(PropulsionError::InvalidSpec(
                "RCS mount needs a name".into(),
            ));
        }
        if self.position_body_m.iter().any(|v| !v.is_finite()) {
            return Err(PropulsionError::InvalidSpec(
                "RCS position must be finite".into(),
            ));
        }
        if self.direction_body.iter().any(|v| !v.is_finite()) {
            return Err(PropulsionError::InvalidSpec(
                "RCS direction must be finite".into(),
            ));
        }
        let norm = (self.direction_body[0] * self.direction_body[0]
            + self.direction_body[1] * self.direction_body[1]
            + self.direction_body[2] * self.direction_body[2])
            .sqrt();
        if (norm - 1.0).abs() > 1e-9 {
            return Err(PropulsionError::InvalidSpec(
                "RCS direction must be unit length".into(),
            ));
        }
        Ok(())
    }
}

/// A mounted RCS block: validated mounts firing coordinated pulses.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RcsCluster {
    pub mounts: Vec<RcsMount>,
}

impl RcsCluster {
    /// Validate the cluster (every mount; on-time arity checked per call).
    pub fn validate(&self) -> Result<(), PropulsionError> {
        if self.mounts.is_empty() {
            return Err(PropulsionError::InvalidSpec(
                "RCS cluster needs at least one mount".into(),
            ));
        }
        for mount in &self.mounts {
            mount.validate()?;
        }
        Ok(())
    }

    /// Delivered impulse for per-thruster on-times: force impulse (N·s)
    /// plus moment impulse about the body origin (N·m·s). Inlet pressure
    /// feeds the cold-gas members (shared bus, documented).
    pub fn impulse(
        &self,
        on_times_s: &[f64],
        inlet_pressure_pa: f64,
        ambient_pa: f64,
    ) -> Result<(DVec3, DVec3), PropulsionError> {
        self.validate()?;
        if on_times_s.len() != self.mounts.len() {
            return Err(PropulsionError::InvalidCommand(format!(
                "expected {} on-times, got {}",
                self.mounts.len(),
                on_times_s.len()
            )));
        }
        let mut force = DVec3::ZERO;
        let mut moment = DVec3::ZERO;
        for (mount, on_time_s) in self.mounts.iter().zip(on_times_s) {
            let delivered = mount
                .thruster
                .pulse(*on_time_s, inlet_pressure_pa, ambient_pa)?;
            let direction = DVec3::from_array(mount.direction_body);
            let position = DVec3::from_array(mount.position_body_m);
            let impulse = direction * delivered.impulse_ns;
            force += impulse;
            moment += position.cross(impulse);
        }
        Ok((force, moment))
    }

    /// PWM-average wrench for per-thruster duty cycles in [0, 1]:
    /// steady force (N) plus moment about the body origin (N·m).
    pub fn steady_wrench(
        &self,
        duties: &[f64],
        inlet_pressure_pa: f64,
        ambient_pa: f64,
    ) -> Result<(DVec3, DVec3), PropulsionError> {
        self.validate()?;
        if duties.len() != self.mounts.len() {
            return Err(PropulsionError::InvalidCommand(format!(
                "expected {} duties, got {}",
                self.mounts.len(),
                duties.len()
            )));
        }
        let mut force = DVec3::ZERO;
        let mut moment = DVec3::ZERO;
        for (mount, duty) in self.mounts.iter().zip(duties) {
            require_non_negative(*duty, "duty")?;
            if *duty > 1.0 || !duty.is_finite() {
                return Err(PropulsionError::InvalidCommand(
                    "duty must be finite in [0, 1]".into(),
                ));
            }
            let thrust_n = mount
                .thruster
                .steady_thrust_n(inlet_pressure_pa, ambient_pa)?
                * duty;
            let direction = DVec3::from_array(mount.direction_body);
            let position = DVec3::from_array(mount.position_body_m);
            let contribution = direction * thrust_n;
            force += contribution;
            moment += position.cross(contribution);
        }
        Ok((force, moment))
    }

    /// Convert cluster mounts into vehicle engine mounts for baking
    /// (monopropellant members only; cold gas carries no chamber model).
    /// Thrust axis equals the RCS direction; callers set stations.
    pub fn baked_monoprop_mounts(&self) -> Result<Vec<EngineMount>, PropulsionError> {
        self.validate()?;
        let mut mounts = Vec::with_capacity(self.mounts.len());
        for mount in &self.mounts {
            match &mount.thruster {
                RcsThruster::Monoprop(compiled) => mounts.push(EngineMount {
                    name: mount.name.clone(),
                    engine: super::CompiledEngine::Liquid(compiled.engine.clone()),
                    position_body_m: mount.position_body_m,
                    thrust_axis_body: mount.direction_body,
                }),
                RcsThruster::ColdGas(_) => {
                    return Err(PropulsionError::UnsupportedCombination(format!(
                        "cold-gas member {} has no baked chamber model",
                        mount.name
                    )));
                }
            }
        }
        Ok(mounts)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hydrazine_10n() -> CompiledMonoprop {
        MonopropThrusterSpec {
            name: "R-10".into(),
            chamber_pressure_pa: 1.0e6,
            throat_radius_m: 0.002,
            expansion_ratio: 60.0,
            nozzle_length_m: 0.06,
            contour: NozzleContour::Conical,
            material: ChamberMaterial::regen_alloy(),
            valve_rise_time_s: RCS_DEFAULT_RISE_TIME_S,
            min_on_time_s: RCS_DEFAULT_MIN_ON_TIME_S,
        }
        .compile()
        .expect("hydrazine compiles")
    }

    fn nitrogen_coldgas() -> CompiledColdGas {
        ColdGasThrusterSpec {
            name: "N2-11N".into(),
            gas: Propellant::ColdGasNitrogen,
            storage_temp_k: 300.0,
            rated_pressure_pa: 2.0e6,
            throat_radius_m: 0.001,
            expansion_ratio: 10.0,
            nozzle_length_m: 0.02,
            contour: NozzleContour::Conical,
            material: ChamberMaterial::regen_alloy(),
            valve_rise_time_s: RCS_DEFAULT_RISE_TIME_S,
            min_on_time_s: RCS_DEFAULT_MIN_ON_TIME_S,
        }
        .compile()
        .expect("cold gas compiles")
    }

    #[test]
    fn hydrazine_isp_band_and_mib() {
        // Monoprop hydrazine vacuum Isp sits in the 210-235 s band (high
        // expansion cone); the minimum impulse bit is positive and far
        // below a 1 s burn.
        let thruster = hydrazine_10n();
        let steady = thruster.steady_point(0.0).expect("steady");
        assert!(
            (210.0..=235.0).contains(&steady.isp_s),
            "hydrazine Isp {} s outside the band",
            steady.isp_s
        );
        let mib = thruster.min_impulse_bit_ns();
        assert!(mib > 0.0);
        let full = thruster.pulse(1.0, 0.0).expect("1 s pulse").impulse_ns;
        assert!(
            mib < 0.05 * full,
            "MIB must be a small fraction of a 1 s burn"
        );
    }

    #[test]
    fn cold_gas_nitrogen_band_and_pressure_scaling() {
        // N2 cold gas vacuum Isp sits in the 65-85 s band; thrust and flow
        // track inlet pressure exactly (choked flow, frozen geometry).
        let thruster = nitrogen_coldgas();
        assert!(
            (65.0..=85.0).contains(&thruster.rated_isp_vac_s),
            "N2 Isp {} s outside the band",
            thruster.rated_isp_vac_s
        );
        let full = thruster.steady_point(2.0e6, 0.0).expect("rated point");
        let half = thruster.steady_point(1.0e6, 0.0).expect("half pressure");
        assert!((half.thrust_n * 2.0 - full.thrust_n).abs() / full.thrust_n < 1e-12);
        assert!(
            (half.mass_flow_kg_s * 2.0 - full.mass_flow_kg_s).abs() / full.mass_flow_kg_s < 1e-12
        );
        assert!((half.isp_s - full.isp_s).abs() / full.isp_s < 1e-12);
        // Blowdown is monotonic: less tank pressure, less thrust.
        let low = thruster.steady_point(0.5e6, 0.0).expect("low");
        assert!(low.thrust_n < half.thrust_n);
    }

    #[test]
    fn pulse_rise_shape_and_long_pulse_efficiency() {
        // Below the rise the impulse grows quadratically; a 1 s pulse
        // recovers steady Isp to 0.5% (transient cost amortized).
        let thruster = hydrazine_10n();
        let tiny = thruster.pulse(0.0025, 0.0).expect("tiny");
        let half_tiny = thruster.pulse(0.00125, 0.0).expect("half tiny");
        assert!(
            (half_tiny.impulse_ns * 4.0 - tiny.impulse_ns).abs() / tiny.impulse_ns < 1e-9,
            "sub-rise pulses must scale quadratically"
        );
        let steady_isp = thruster.steady_point(0.0).expect("steady").isp_s;
        let long = thruster.pulse(1.0, 0.0).expect("long");
        assert!((long.effective_isp_s - steady_isp).abs() / steady_isp < 0.005);
        // Min on-time must clear the rise, and zero is a no-op.
        assert!(thruster.pulse(0.0, 0.0).expect("zero").impulse_ns == 0.0);
        let bad = MonopropThrusterSpec {
            name: "bad".into(),
            chamber_pressure_pa: 1.0e6,
            throat_radius_m: 0.002,
            expansion_ratio: 40.0,
            nozzle_length_m: 0.03,
            contour: NozzleContour::Conical,
            material: ChamberMaterial::regen_alloy(),
            valve_rise_time_s: RCS_DEFAULT_RISE_TIME_S,
            min_on_time_s: 0.001,
        };
        assert!(bad.compile().is_err());
    }

    #[test]
    fn opposed_pair_delivers_pure_couple() {
        // Two opposed thrusters on a 2 m arm: force cancels, moment doubles.
        let mono = RcsThruster::Monoprop(hydrazine_10n());
        let cluster = RcsCluster {
            mounts: vec![
                RcsMount {
                    name: "a".into(),
                    thruster: mono.clone(),
                    position_body_m: [0.0, 0.0, 1.0],
                    direction_body: [1.0, 0.0, 0.0],
                },
                RcsMount {
                    name: "b".into(),
                    thruster: mono,
                    position_body_m: [0.0, 0.0, -1.0],
                    direction_body: [-1.0, 0.0, 0.0],
                },
            ],
        };
        let (force, moment) = cluster.impulse(&[0.1, 0.1], 1.0e6, 0.0).expect("fire");
        assert!(force.length() < 1e-9, "opposed pair must cancel force");
        let single = hydrazine_10n().pulse(0.1, 0.0).expect("single").impulse_ns;
        assert!((moment.y - 2.0 * single).abs() / (2.0 * single) < 1e-9);
        assert!(moment.x.abs() < 1e-9 && moment.z.abs() < 1e-9);
        // Steady wrench at half duty is half the full wrench.
        let (half_f, _) = cluster
            .steady_wrench(&[0.5, 0.0], 1.0e6, 0.0)
            .expect("half");
        let (full_f, _) = cluster
            .steady_wrench(&[1.0, 0.0], 1.0e6, 0.0)
            .expect("full");
        assert!((half_f.length() * 2.0 - full_f.length()).abs() / full_f.length() < 1e-12);
    }

    #[test]
    fn cold_gas_rejected_from_liquid_path() {
        // Cold gas has no chamber to size: the liquid spec must refuse it.
        let spec = super::super::liquid::LiquidEngineSpec {
            name: "cold".into(),
            propellant: Propellant::ColdGasNitrogen,
            cycle: EngineCycle::PressureFed,
            chamber_pressure_pa: 1.0e6,
            throat_radius_m: 0.001,
            expansion_ratio: 5.0,
            nozzle_length_m: 0.02,
            contour: NozzleContour::Conical,
            chamber_material: ChamberMaterial::regen_alloy(),
            cooling: CoolingMode::Regenerative,
            mixture_ratio: None,
            characteristic_length_m: None,
            gimbal_range_rad: 0.0,
            min_throttle: None,
            restartable: true,
        };
        assert!(spec.compile().is_err());
    }
}
