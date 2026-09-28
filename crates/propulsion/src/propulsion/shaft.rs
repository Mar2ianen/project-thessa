//! Jet shaft runtime: starter topologies, light-off/self-sustain
//! hysteresis, windmill relight, generator load, and the normalized
//! spool-speed integrator (`docs/details/04` sections 8.1 and 18.8).
//!
//! The shaft is the rotating assembly of a single-spool jet: the
//! compressor/fan demand power from it, the turbine supplies power to
//! it, and optional accessories (starter, generator) push power in or
//! pull power out. Startup is therefore a solved power balance, not a
//! free state change: a starterless engine at zero airspeed receives no
//! free intake flow, because the compressor suction floor scales with
//! the actual spool speed and the shaft only spins when net shaft power
//! is positive.
//!
//! Model semantics (documented engineering bound):
//!
//! - spool speed is normalized (`spool_n` in [0, 1], 1 = design speed);
//! - dynamics integrate `dn/dt = net_w / (P_ref * spool_tau_s)`, a
//!   normalized form of torque = inertia x angular acceleration where
//!   `P_ref` (design turbine shaft power) times `spool_tau_s` plays the
//!   role of shaft energy storage — it calibrates the documented spool
//!   lag, it is not a measured rotor inertia;
//! - light-off requires `spool_n >= light_off_n` plus burnable air;
//!   once lit, the core flames out below `self_sustain_n` (hysteresis);
//! - starter charge is a single stored-energy reservoir (battery,
//!   compressed air, bootstrap propellant as J); wiring it to tank/bus
//!   resource graphs is deferred with those graphs;
//! - generator load enters the shaft balance directly: requested
//!   electrical power divided by efficiency is shaft drag, so an
//!   overloaded generator can stall a marginal spool.

// Validity guards use `!(x > 0.0)` so NaN fails closed (repo convention).
#![allow(clippy::neg_cmp_op_on_partial_ord)]

use serde::{Deserialize, Serialize};

use super::{
    AirCycle, AirOperatingPoint, CompiledAirbreather, FlightCondition, PropulsionError,
    require_positive,
};
use crate::StoredPropellant;

/// Bearing/accessory friction as a fraction of the reference shaft power
/// at `spool_n` cubed (documented loss channel; the cubic speed
/// dependence is the standard hydrodynamic-bearing form).
pub const SHAFT_FRICTION_FRACTION: f64 = 0.02;

/// Fraction of combustor heat release the turbine can extract as shaft
/// work at the design fuel flow, before design-point normalization
/// (documented calibration). Its only jobs are fixing the reference
/// power scale (`shaft_reference_power_w`, which sizes friction and the
/// spool dynamics) and keeping that scale physically plausible; the
/// design equilibrium itself is pinned exactly by the normalization
/// below, so this number cannot silently rescale engine performance.
pub const TURBINE_SHAFT_HEAT_FRACTION: f64 = 0.5;

/// Electric starter/generator drivetrain efficiency (documented).
pub const STARTER_ELECTRIC_EFFICIENCY: f64 = 0.85;
/// Air-turbine (pneumatic) starter efficiency (documented).
pub const STARTER_PNEUMATIC_EFFICIENCY: f64 = 0.70;
/// Rocket/gas-generator bootstrap turbine efficiency (documented).
pub const STARTER_ROCKET_EFFICIENCY: f64 = 0.40;

/// Starter topology (section 8.1): an authored hardware choice with
/// real mass and stored-energy consequences, independent of the
/// generator.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum StarterKind {
    /// Windmill/ram only: no starter hardware, no self-start at rest;
    /// in-flight relight only once inlet-driven rotation reaches
    /// light-off speed.
    #[default]
    None,
    /// Electrical bus -> motor/generator -> spool; zero-airspeed start.
    Electric,
    /// APU/ground-cart/cross-bleed air -> starter turbine -> spool.
    Pneumatic,
    /// Onboard propellant -> gas-generator/bootstrap turbine -> spool;
    /// starts independently of ambient airspeed.
    RocketBootstrap,
}

/// Rotor to which a shaft accessory is mechanically connected. In the
/// legacy single-spool topology both values address the one rotor; in a
/// multi-spool turbofan they select the HP core rotor or LP fan rotor.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ShaftSpool {
    LowPressure,
    #[default]
    HighPressure,
}

impl StarterKind {
    /// Stored-energy-to-shaft-power efficiency of this topology
    /// (documented per-family values).
    pub fn efficiency(self) -> f64 {
        match self {
            Self::None => 0.0,
            Self::Electric => STARTER_ELECTRIC_EFFICIENCY,
            Self::Pneumatic => STARTER_PNEUMATIC_EFFICIENCY,
            Self::RocketBootstrap => STARTER_ROCKET_EFFICIENCY,
        }
    }
}

/// Starter hardware spec: deliverable shaft power/torque, stored or
/// tank-backed energy, and installed mass. Tank-backed resources are drawn
/// through vehicle-core's ordinary resource allocator; the starter owns no
/// parallel inventory path.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct StarterSpec {
    pub kind: StarterKind,
    /// Deliverable shaft power while engaged (W).
    pub power_w: f64,
    /// Stored starter energy (J): battery, compressed-air, or bootstrap
    /// propellant chemical energy. Ignored (and required zero) for
    /// `StarterKind::None`.
    pub charge_j: f64,
    /// Optional tank-backed energy source. When present, `charge_j` must be
    /// zero and the live reservoir is derived from reachable vehicle tanks.
    #[serde(default)]
    pub resource: Option<StoredPropellant>,
    /// Rotor carrying starter torque (HP by default).
    #[serde(default)]
    pub attached_spool: ShaftSpool,
    /// Recoverable starter energy per kilogram of the selected stored
    /// resource (J/kg), explicitly authored for compressed-gas or bootstrap
    /// hardware. Zero when `resource` is absent.
    #[serde(default)]
    pub specific_energy_j_kg: f64,
    /// Optional rated shaft torque (N·m). When present, the physical shaft
    /// integrator uses this actuator torque with the authored rotor inertia.
    #[serde(default)]
    pub maximum_shaft_torque_nm: Option<f64>,
    /// Starter hardware mass (kg).
    pub mass_kg: f64,
}

impl Default for StarterSpec {
    fn default() -> Self {
        Self {
            kind: StarterKind::None,
            power_w: 0.0,
            charge_j: 0.0,
            resource: None,
            attached_spool: ShaftSpool::HighPressure,
            specific_energy_j_kg: 0.0,
            maximum_shaft_torque_nm: None,
            mass_kg: 0.0,
        }
    }
}

impl StarterSpec {
    /// Validate against the engine cycle (NaN fails closed; ramjets and
    /// scramjets refuse shaft hardware because they have no shaft).
    pub fn validate(&self, cycle: AirCycle) -> Result<(), PropulsionError> {
        if !cycle.has_shaft() && self.kind != StarterKind::None {
            return Err(PropulsionError::UnsupportedCombination(
                "ramjets and scramjets have no compressor/turbine shaft to start".into(),
            ));
        }
        match self.kind {
            StarterKind::None => {
                if self.power_w != 0.0
                    || self.charge_j != 0.0
                    || self.resource.is_some()
                    || self.specific_energy_j_kg != 0.0
                    || self.maximum_shaft_torque_nm.is_some()
                    || self.mass_kg != 0.0
                {
                    return Err(PropulsionError::InvalidSpec(
                        "no starter fitted, but starter power/charge/mass is non-zero".into(),
                    ));
                }
            }
            StarterKind::Electric | StarterKind::Pneumatic | StarterKind::RocketBootstrap => {
                require_positive(self.power_w, "starter power")?;
                require_positive(self.mass_kg, "starter mass")?;
                if let Some(torque) = self.maximum_shaft_torque_nm {
                    require_positive(torque, "starter shaft torque")?;
                }
                if let Some(resource) = self.resource {
                    if self.charge_j != 0.0 {
                        return Err(PropulsionError::InvalidSpec(
                            "tank-backed starter must set charge_j to zero".into(),
                        ));
                    }
                    require_positive(
                        self.specific_energy_j_kg,
                        "starter resource specific energy",
                    )?;
                    if self.kind == StarterKind::Pneumatic
                        && !matches!(
                            resource,
                            StoredPropellant::Nitrogen | StoredPropellant::Helium
                        )
                    {
                        return Err(PropulsionError::UnsupportedCombination(
                            "pneumatic starter resource must be stored nitrogen or helium".into(),
                        ));
                    }
                } else {
                    require_positive(self.charge_j, "starter charge")?;
                    if self.specific_energy_j_kg != 0.0 {
                        return Err(PropulsionError::InvalidSpec(
                            "starter specific energy requires a tank-backed resource".into(),
                        ));
                    }
                }
            }
        }
        Ok(())
    }
}

/// Shaft-mounted generator: independently optional (a running engine
/// creates no bus power just by running). `fitted = false` is the
/// documented "no generator" topology.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct GeneratorSpec {
    pub fitted: bool,
    /// Rated electrical output (W); also caps the requested load.
    pub power_w: f64,
    /// Shaft-to-electrical efficiency in (0, 1]; shaft draw is
    /// requested electrical power divided by it.
    pub efficiency: f64,
    /// Optional piecewise-linear efficiency map over normalized spool speed.
    /// When non-empty it replaces the scalar efficiency.
    #[serde(default)]
    pub efficiency_map: Vec<GeneratorEfficiencyPoint>,
    /// Optional lumped generator thermal model. Electrical output is limited
    /// so one implicit thermal step cannot cross its authored maximum.
    #[serde(default)]
    pub thermal: Option<GeneratorThermalSpec>,
    /// Rotor carrying generator reaction torque (HP by default).
    #[serde(default)]
    pub attached_spool: ShaftSpool,
    /// Optional shaft-side torque ceiling. The electrical output then falls
    /// with actual angular speed below the power rating.
    #[serde(default)]
    pub maximum_shaft_torque_nm: Option<f64>,
    /// Normalized spool speed below which the generator is offline
    /// (cut-in); it delivers nothing under it.
    pub cut_in_spool_n: f64,
    /// Generator mass (kg).
    pub mass_kg: f64,
}

/// Generator conversion efficiency at a normalized shaft speed.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GeneratorEfficiencyPoint {
    pub spool_n: f64,
    pub efficiency: f64,
}

/// Lumped winding/casing heat capacity and heat rejection to its local sink.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GeneratorThermalSpec {
    pub heat_capacity_j_k: f64,
    pub conductance_w_k: f64,
    pub initial_temperature_k: f64,
    pub maximum_temperature_k: f64,
}

impl GeneratorThermalSpec {
    pub fn validate(&self) -> Result<(), PropulsionError> {
        require_positive(self.heat_capacity_j_k, "generator heat capacity")?;
        require_positive(self.conductance_w_k, "generator thermal conductance")?;
        if !self.initial_temperature_k.is_finite()
            || self.initial_temperature_k <= 0.0
            || !self.maximum_temperature_k.is_finite()
            || self.maximum_temperature_k < self.initial_temperature_k
        {
            return Err(PropulsionError::InvalidSpec(
                "generator temperatures must be finite, positive, and initial <= maximum".into(),
            ));
        }
        Ok(())
    }
}

impl Default for GeneratorSpec {
    fn default() -> Self {
        Self {
            fitted: false,
            power_w: 0.0,
            efficiency: 0.85,
            efficiency_map: Vec::new(),
            thermal: None,
            attached_spool: ShaftSpool::HighPressure,
            maximum_shaft_torque_nm: None,
            cut_in_spool_n: 0.5,
            mass_kg: 0.0,
        }
    }
}

/// Two-spool turbofan rotor and turbine-work allocation. The low-pressure
/// rotor drives the fan; the high-pressure rotor drives the core compressor.
/// Fan inertia is reflected through the authored speed ratio and gearbox
/// efficiency. This is explicit hardware topology, not an extra spool-speed
/// coefficient applied to thrust.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct MultiSpoolSpec {
    pub low_pressure_design_speed_rad_s: f64,
    pub low_pressure_rotor_inertia_kg_m2: f64,
    pub fan_rotor_inertia_kg_m2: f64,
    pub high_pressure_design_speed_rad_s: f64,
    pub high_pressure_rotor_inertia_kg_m2: f64,
    /// Share of core turbine mechanical work delivered to the HP rotor; LP
    /// turbine work receives the remainder.
    pub high_pressure_turbine_power_fraction: f64,
    /// Fan angular speed / LP rotor angular speed. Values below one are
    /// reduction gears; one is direct drive.
    pub fan_gear_speed_ratio: f64,
    /// Mechanical power efficiency from LP rotor to fan.
    pub fan_gear_efficiency: f64,
}

impl MultiSpoolSpec {
    fn validate(self) -> Result<(), PropulsionError> {
        require_positive(
            self.low_pressure_design_speed_rad_s,
            "LP spool design speed",
        )?;
        require_positive(
            self.low_pressure_rotor_inertia_kg_m2,
            "LP spool rotor inertia",
        )?;
        if !self.fan_rotor_inertia_kg_m2.is_finite() || self.fan_rotor_inertia_kg_m2 < 0.0 {
            return Err(PropulsionError::InvalidSpec(
                "fan rotor inertia must be finite and >= 0".into(),
            ));
        }
        require_positive(
            self.high_pressure_design_speed_rad_s,
            "HP spool design speed",
        )?;
        require_positive(
            self.high_pressure_rotor_inertia_kg_m2,
            "HP spool rotor inertia",
        )?;
        if !self.high_pressure_turbine_power_fraction.is_finite()
            || !(0.0..=1.0).contains(&self.high_pressure_turbine_power_fraction)
            || self.high_pressure_turbine_power_fraction == 0.0
            || self.high_pressure_turbine_power_fraction == 1.0
        {
            return Err(PropulsionError::InvalidSpec(
                "HP turbine work fraction must be finite and strictly between 0 and 1".into(),
            ));
        }
        if !self.fan_gear_speed_ratio.is_finite()
            || self.fan_gear_speed_ratio <= 0.0
            || self.fan_gear_speed_ratio > 1.0
        {
            return Err(PropulsionError::InvalidSpec(
                "fan gear speed ratio must be finite in (0, 1]".into(),
            ));
        }
        if !self.fan_gear_efficiency.is_finite()
            || self.fan_gear_efficiency <= 0.0
            || self.fan_gear_efficiency > 1.0
        {
            return Err(PropulsionError::InvalidSpec(
                "fan gear efficiency must be finite in (0, 1]".into(),
            ));
        }
        Ok(())
    }

    fn low_pressure_effective_inertia(self) -> f64 {
        self.low_pressure_rotor_inertia_kg_m2
            + self.fan_rotor_inertia_kg_m2 * self.fan_gear_speed_ratio.powi(2)
    }
}

impl GeneratorSpec {
    /// Validate against the engine cycle (NaN fails closed; ramjets and
    /// scramjets refuse shaft hardware because they have no shaft).
    pub fn validate(&self, cycle: AirCycle) -> Result<(), PropulsionError> {
        if !cycle.has_shaft() && self.fitted {
            return Err(PropulsionError::UnsupportedCombination(
                "ramjets and scramjets have no shaft to drive a generator; use the vehicle bus"
                    .into(),
            ));
        }
        if !self.fitted {
            if self.power_w != 0.0
                || self.mass_kg != 0.0
                || !self.efficiency_map.is_empty()
                || self.thermal.is_some()
                || self.maximum_shaft_torque_nm.is_some()
            {
                return Err(PropulsionError::InvalidSpec(
                    "no generator fitted, but generator power/mass is non-zero".into(),
                ));
            }
            return Ok(());
        }
        require_positive(self.power_w, "generator power")?;
        require_positive(self.mass_kg, "generator mass")?;
        if !(self.efficiency > 0.0 && self.efficiency <= 1.0) {
            return Err(PropulsionError::InvalidSpec(
                "generator efficiency must be finite in (0, 1]".into(),
            ));
        }
        let mut previous_spool = None;
        for point in &self.efficiency_map {
            if !point.spool_n.is_finite()
                || !(0.0..=1.0).contains(&point.spool_n)
                || !point.efficiency.is_finite()
                || !(0.0 < point.efficiency && point.efficiency <= 1.0)
                || previous_spool.is_some_and(|previous| point.spool_n <= previous)
            {
                return Err(PropulsionError::InvalidSpec(
                    "generator efficiency-map speeds must be strictly increasing in [0, 1] with efficiencies in (0, 1]".into(),
                ));
            }
            previous_spool = Some(point.spool_n);
        }
        if let Some(thermal) = self.thermal {
            thermal.validate()?;
        }
        if let Some(torque) = self.maximum_shaft_torque_nm {
            require_positive(torque, "generator shaft torque")?;
        }
        if !self.cut_in_spool_n.is_finite() || !(0.0..=1.0).contains(&self.cut_in_spool_n) {
            return Err(PropulsionError::InvalidSpec(
                "generator cut-in spool must be finite in [0, 1]".into(),
            ));
        }
        Ok(())
    }

    fn efficiency_at(&self, spool_n: f64) -> f64 {
        if self.efficiency_map.is_empty() {
            return self.efficiency;
        }
        if spool_n <= self.efficiency_map[0].spool_n {
            return self.efficiency_map[0].efficiency;
        }
        for pair in self.efficiency_map.windows(2) {
            let [left, right] = pair else { unreachable!() };
            if spool_n <= right.spool_n {
                let fraction = (spool_n - left.spool_n) / (right.spool_n - left.spool_n);
                return left.efficiency + fraction * (right.efficiency - left.efficiency);
            }
        }
        self.efficiency_map
            .last()
            .expect("non-empty efficiency map")
            .efficiency
    }
}

/// Authored shaft topology of one gas turbine: starter, generator,
/// power-turbine takeoff, and the light-off/self-sustain thresholds
/// that decide when combustion can start and when it dies (sections 8.1/9).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ShaftSpec {
    pub starter: StarterSpec,
    pub generator: GeneratorSpec,
    /// Normalized spool speed at which ignition can light the core.
    pub light_off_n: f64,
    /// Normalized spool speed below which a lit core flames out
    /// (`<= light_off_n`: the gap is the relight hysteresis band).
    pub self_sustain_n: f64,
    /// Maximum share of combustor heat released as mechanical output by a
    /// downstream power turbine. Zero preserves a pure jet shaft. This is
    /// an energy budget, not a thrust multiplier; loaded cycle evaluation
    /// subtracts the corresponding gas enthalpy and pressure before the core
    /// nozzle.
    pub power_turbine_heat_fraction: f64,
    /// Rated shaft angular speed required by explicit torque-level hardware.
    #[serde(default)]
    pub design_speed_rad_s: Option<f64>,
    /// Rotor inertia for the torque-level shaft integrator (kg·m²).
    #[serde(default)]
    pub rotor_inertia_kg_m2: Option<f64>,
    /// Optional LP/HP turbofan shaft train. Omitted preserves the legacy
    /// single-spool air-path and integrator.
    #[serde(default)]
    pub multi_spool: Option<MultiSpoolSpec>,
}

impl Default for ShaftSpec {
    fn default() -> Self {
        Self {
            starter: StarterSpec::default(),
            generator: GeneratorSpec::default(),
            light_off_n: 0.15,
            self_sustain_n: 0.10,
            power_turbine_heat_fraction: 0.0,
            design_speed_rad_s: None,
            rotor_inertia_kg_m2: None,
            multi_spool: None,
        }
    }
}

impl ShaftSpec {
    /// Validate authoring values against the engine cycle (NaN fails
    /// closed; passive ramjets/scramjets may only carry the inert default shaft).
    pub fn validate(&self, cycle: AirCycle) -> Result<(), PropulsionError> {
        if !self.light_off_n.is_finite() || !(0.0..=1.0).contains(&self.light_off_n) {
            return Err(PropulsionError::InvalidSpec(
                "light-off spool must be finite in (0, 1]".into(),
            ));
        }
        if !(self.light_off_n > 0.0) {
            return Err(PropulsionError::InvalidSpec(
                "light-off spool must be finite in (0, 1]".into(),
            ));
        }
        if !self.self_sustain_n.is_finite() || !(self.self_sustain_n > 0.0) {
            return Err(PropulsionError::InvalidSpec(
                "self-sustain spool must be finite and > 0".into(),
            ));
        }
        if self.self_sustain_n > self.light_off_n {
            return Err(PropulsionError::InvalidSpec(
                "self-sustain spool must not exceed light-off spool".into(),
            ));
        }
        if !self.power_turbine_heat_fraction.is_finite()
            || !(0.0..=1.0).contains(&self.power_turbine_heat_fraction)
        {
            return Err(PropulsionError::InvalidSpec(
                "power-turbine heat fraction must be finite in [0, 1]".into(),
            ));
        }
        if !cycle.has_shaft() && self.power_turbine_heat_fraction != 0.0 {
            return Err(PropulsionError::UnsupportedCombination(
                "ramjets and scramjets have no turbine shaft or power takeoff".into(),
            ));
        }
        self.starter.validate(cycle)?;
        self.generator.validate(cycle)?;
        if let Some(multi_spool) = self.multi_spool {
            if cycle != AirCycle::Turbofan {
                return Err(PropulsionError::UnsupportedCombination(
                    "the LP/HP multi-spool topology currently requires a turbofan cycle".into(),
                ));
            }
            multi_spool.validate()?;
            if self.design_speed_rad_s.is_some() || self.rotor_inertia_kg_m2.is_some() {
                return Err(PropulsionError::InvalidSpec(
                    "multi-spool rotors use their own rated speeds and inertias".into(),
                ));
            }
            if self.starter.kind != StarterKind::None
                && self.starter.maximum_shaft_torque_nm.is_none()
            {
                return Err(PropulsionError::InvalidSpec(
                    "a multi-spool starter requires maximum_shaft_torque_nm".into(),
                ));
            }
            if self.generator.fitted && self.generator.maximum_shaft_torque_nm.is_none() {
                return Err(PropulsionError::InvalidSpec(
                    "a multi-spool generator requires maximum_shaft_torque_nm".into(),
                ));
            }
            if self.power_turbine_heat_fraction != 0.0 {
                return Err(PropulsionError::UnsupportedCombination(
                    "the LP/HP turbofan topology does not include a separate power turbine".into(),
                ));
            }
        }
        let torque_hardware = self.starter.maximum_shaft_torque_nm.is_some()
            || self.generator.maximum_shaft_torque_nm.is_some();
        let torque_integration = self.design_speed_rad_s.is_some()
            || self.rotor_inertia_kg_m2.is_some()
            || torque_hardware;
        if torque_integration
            && self.starter.kind != StarterKind::None
            && self.starter.maximum_shaft_torque_nm.is_none()
        {
            return Err(PropulsionError::InvalidSpec(
                "a fitted starter on a torque-integrated shaft requires maximum_shaft_torque_nm"
                    .into(),
            ));
        }
        if torque_integration
            && self.generator.fitted
            && self.generator.maximum_shaft_torque_nm.is_none()
        {
            return Err(PropulsionError::InvalidSpec(
                "a fitted generator on a torque-integrated shaft requires maximum_shaft_torque_nm"
                    .into(),
            ));
        }
        match (self.design_speed_rad_s, self.rotor_inertia_kg_m2) {
            (Some(speed), Some(inertia)) => {
                require_positive(speed, "shaft design speed")?;
                require_positive(inertia, "shaft rotor inertia")?;
            }
            (None, None) if !torque_hardware || self.multi_spool.is_some() => {}
            _ => {
                return Err(PropulsionError::InvalidSpec(
                    "torque-level shaft hardware requires both design_speed_rad_s and rotor_inertia_kg_m2".into(),
                ));
            }
        }
        Ok(())
    }
}

/// Shaft power balance of one cycle evaluation (all W): what the
/// accessories and compressor need versus what the turbine can supply
/// at the evaluated fuel flow. [`ShaftBalance::net_w`] is the power
/// left to accelerate (positive) or decelerate (negative) the spool.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ShaftBalance {
    /// Compressor + fan power drawn from the shaft.
    pub demand_w: f64,
    /// Compressor work assigned to the HP rotor at the evaluated operating
    /// point; equals `demand_w` on the legacy one-spool path.
    #[serde(default)]
    pub high_pressure_demand_w: f64,
    /// Fan work at the fan rotor before gearbox losses; zero for non-turbofans.
    #[serde(default)]
    pub low_pressure_fan_demand_w: f64,
    /// Net turbine work minus compressor/friction load on the HP rotor.
    /// Equals `net_w()` for the legacy single-spool model.
    #[serde(default)]
    pub high_pressure_net_w: f64,
    /// Net turbine work minus fan/gear/friction load on the LP rotor.
    #[serde(default)]
    pub low_pressure_net_w: f64,
    /// Turbine shaft power available at this fuel flow (0 when the core
    /// is not burning).
    pub capacity_w: f64,
    /// Gas enthalpy-limited output reserved for a downstream power turbine.
    #[serde(default)]
    pub power_takeoff_capacity_w: f64,
    /// Bearing/accessory friction loss at the evaluated spool speed.
    pub friction_w: f64,
}

impl ShaftBalance {
    /// Net shaft power (W): capacity minus demand and friction. Starter
    /// and generator terms are added by [`advance_jet_shaft`], which
    /// owns the command.
    pub fn net_w(&self) -> f64 {
        self.capacity_w - self.demand_w - self.friction_w
    }
}

/// Live jet shaft state: normalized spool speed, combustion state, and
/// remaining stored starter energy. Pure data; advance with
/// [`advance_jet_shaft`] using physics seconds (f64 durations — never
/// `Instant`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct JetShaftState {
    /// Normalized spool speed in [0, 1] (1 = design speed).
    pub spool_n: f64,
    /// Optional normalized LP turbine/fan rotor speed. `None` is the
    /// backwards-compatible single-spool state.
    #[serde(default)]
    pub low_pressure_spool_n: Option<f64>,
    /// True while the core sustains combustion (light-off reached,
    /// above self-sustain, burnable air present).
    pub lit: bool,
    /// Remaining stored starter energy (J). Constant for
    /// `StarterKind::None`.
    pub starter_charge_j: f64,
    /// Generator winding/casing temperature (K); used only when the fitted
    /// generator has a thermal model.
    #[serde(default = "default_generator_temperature_k")]
    pub generator_temperature_k: f64,
}

fn default_generator_temperature_k() -> f64 {
    293.15
}

impl JetShaftState {
    /// Stopped engine: no rotation, unlit, starter at full charge.
    /// Shafted engines only (ramjet/scramjet have no shaft state).
    pub fn cold(engine: &CompiledAirbreather) -> Self {
        Self {
            spool_n: 0.0,
            low_pressure_spool_n: engine.shaft.multi_spool.map(|_| 0.0),
            lit: false,
            starter_charge_j: engine.shaft.starter.charge_j,
            generator_temperature_k: engine
                .shaft
                .generator
                .thermal
                .map_or_else(default_generator_temperature_k, |thermal| {
                    thermal.initial_temperature_k
                }),
        }
    }

    /// Already-running engine at design spool: lit, full starter charge
    /// (a stopped starter does not recharge a running engine).
    pub fn running(engine: &CompiledAirbreather) -> Self {
        Self {
            spool_n: 1.0,
            low_pressure_spool_n: engine.shaft.multi_spool.map(|_| 1.0),
            lit: true,
            starter_charge_j: engine.shaft.starter.charge_j,
            generator_temperature_k: engine
                .shaft
                .generator
                .thermal
                .map_or_else(default_generator_temperature_k, |thermal| {
                    thermal.initial_temperature_k
                }),
        }
    }

    /// Name of the explicitly tank-backed starter draw consumer.
    pub fn starter_resource_consumer_name(mount_name: &str) -> String {
        format!("{mount_name}::starter")
    }
}

/// One shaft control step: throttle demand, starter engagement, and
/// requested electrical generator load (W).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ShaftCommand {
    /// Throttle demand in [0, 1]; 0 commands fuel off (shutdown),
    /// > 0 commands ignition/sustained operation.
    pub throttle: f64,
    /// Engage the fitted starter (refused at runtime when none is
    /// fitted).
    pub starter_engaged: bool,
    /// Requested electrical generator load (W), capped by the rated
    /// power and gated by cut-in speed.
    pub generator_load_w: f64,
}

/// Shaft telemetry for one advanced step: every power channel of the
/// balance plus the resulting state (debug/verification channel — the
/// effect is otherwise invisible without it).
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct ShaftTelemetry {
    pub spool_n: f64,
    pub lit: bool,
    /// True while the starter actually delivered shaft power this step.
    pub starter_active: bool,
    pub starter_shaft_power_w: f64,
    /// Stored energy consumed per second by the starter (W; electrical
    /// draw, pneumatic thermal power, or bootstrap chemical power).
    pub starter_draw_w: f64,
    /// Pneumatic input actually accepted from an external source (W).
    #[serde(default)]
    pub external_starter_supply_w: f64,
    pub starter_charge_j: f64,
    /// Electrical power delivered to the bus this step (W).
    pub generator_electrical_w: f64,
    /// Shaft power removed to produce it (W).
    pub generator_shaft_draw_w: f64,
    /// Generator temperature after this thermal step (K).
    pub generator_temperature_k: f64,
    pub demand_w: f64,
    pub capacity_w: f64,
    pub friction_w: f64,
    /// Net shaft power integrated this step (W), starter/generator
    /// included.
    pub net_w: f64,
}

/// Advance the jet shaft over `dt_s` physics seconds at `condition`.
///
/// Order of operations per step: evaluate the air path at the current
/// spool speed (fuel scheduled because throttle commands ignition),
/// decide light-off/flameout against the hysteresis thresholds, gate
/// capacity on the decided combustion state, add starter input and
/// subtract generator load, then integrate the normalized spool
/// dynamics. Returns the next state plus the full power telemetry.
///
/// Refuses: passive ramjet/scramjet cycles (no shaft), a starter engagement with no starter
/// fitted, and non-finite/out-of-range commands (NaN fails closed).
pub fn advance_jet_shaft(
    engine: &CompiledAirbreather,
    state: JetShaftState,
    command: &ShaftCommand,
    condition: &FlightCondition,
    dt_s: f64,
) -> Result<(JetShaftState, ShaftTelemetry), PropulsionError> {
    advance_jet_shaft_loaded(engine, state, command, condition, dt_s, 0.0)
}

/// [`advance_jet_shaft`] with an extra power take-off on the same shaft
/// (`extra_load_w`, watts): a geared propeller load for turboprops
/// (section 9). Zero reproduces the plain jet path exactly.
pub fn advance_jet_shaft_loaded(
    engine: &CompiledAirbreather,
    state: JetShaftState,
    command: &ShaftCommand,
    condition: &FlightCondition,
    dt_s: f64,
    extra_load_w: f64,
) -> Result<(JetShaftState, ShaftTelemetry), PropulsionError> {
    advance_jet_shaft_loaded_with_starter_power(
        engine,
        state,
        command,
        condition,
        dt_s,
        extra_load_w,
        0.0,
    )
}

/// [`advance_jet_shaft_loaded`] with externally supplied pneumatic starter
/// power. The supplier separately pays the corresponding bleed load.
pub fn advance_jet_shaft_loaded_with_starter_power(
    engine: &CompiledAirbreather,
    state: JetShaftState,
    command: &ShaftCommand,
    condition: &FlightCondition,
    dt_s: f64,
    extra_load_w: f64,
    pneumatic_starter_power_w: f64,
) -> Result<(JetShaftState, ShaftTelemetry), PropulsionError> {
    advance_jet_shaft_loaded_with_starter_power_and_spools(
        engine,
        state,
        command,
        condition,
        dt_s,
        extra_load_w,
        pneumatic_starter_power_w,
        |high_pressure_spool_n, low_pressure_spool_n, ignition| {
            if engine.shaft.multi_spool.is_some() {
                if extra_load_w > 0.0 {
                    return Err(PropulsionError::UnsupportedCombination(
                        "the multi-spool turbofan does not accept a separate power-turbine takeoff"
                            .into(),
                    ));
                }
                engine.operating_point_at_spools(
                    condition,
                    command.throttle,
                    high_pressure_spool_n,
                    low_pressure_spool_n,
                    ignition,
                )
            } else {
                engine.operating_point_at_spool_loaded(
                    condition,
                    command.throttle,
                    high_pressure_spool_n,
                    ignition,
                    extra_load_w,
                )
            }
        },
    )
}

/// Shaft integrator variant whose cycle balance and pneumatic starter supply
/// are supplied by the caller. ESTOC uses it to include its precooler-adjusted
/// compressor work in the same shaft balance that advances the spool.
pub(super) fn advance_jet_shaft_loaded_with_starter_power_and<F>(
    engine: &CompiledAirbreather,
    state: JetShaftState,
    command: &ShaftCommand,
    condition: &FlightCondition,
    dt_s: f64,
    extra_load_w: f64,
    pneumatic_starter_power_w: f64,
    evaluate: F,
) -> Result<(JetShaftState, ShaftTelemetry), PropulsionError>
where
    F: FnOnce(f64, bool) -> Result<(AirOperatingPoint, ShaftBalance), PropulsionError>,
{
    advance_jet_shaft_loaded_with_starter_power_and_spools(
        engine,
        state,
        command,
        condition,
        dt_s,
        extra_load_w,
        pneumatic_starter_power_w,
        |spool_n, _low_pressure_spool_n, ignition| evaluate(spool_n, ignition),
    )
}

/// Multi-spool-capable shaft integrator variant. The callback evaluates the
/// cycle with HP and LP normalized rotor speeds independently.
pub(super) fn advance_jet_shaft_loaded_with_starter_power_and_spools<F>(
    engine: &CompiledAirbreather,
    state: JetShaftState,
    command: &ShaftCommand,
    condition: &FlightCondition,
    dt_s: f64,
    extra_load_w: f64,
    pneumatic_starter_power_w: f64,
    evaluate: F,
) -> Result<(JetShaftState, ShaftTelemetry), PropulsionError>
where
    F: FnOnce(f64, f64, bool) -> Result<(AirOperatingPoint, ShaftBalance), PropulsionError>,
{
    if !engine.cycle.has_shaft() {
        return Err(PropulsionError::InvalidCommand(
            "ramjets and scramjets have no shaft to advance".into(),
        ));
    }
    if !(engine.shaft_reference_power_w > 0.0) {
        return Err(PropulsionError::InvalidCommand(
            "engine has no calibrated shaft".into(),
        ));
    }
    condition.validate()?;
    if !command.throttle.is_finite() || !(0.0..=1.0).contains(&command.throttle) {
        return Err(PropulsionError::InvalidCommand(
            "shaft throttle must be finite in [0, 1]".into(),
        ));
    }
    if !command.generator_load_w.is_finite() || command.generator_load_w < 0.0 {
        return Err(PropulsionError::InvalidCommand(
            "generator load must be finite and >= 0".into(),
        ));
    }
    if !pneumatic_starter_power_w.is_finite() || pneumatic_starter_power_w < 0.0 {
        return Err(PropulsionError::InvalidCommand(
            "pneumatic starter supply must be finite and >= 0".into(),
        ));
    }
    if !extra_load_w.is_finite() || extra_load_w < 0.0 {
        return Err(PropulsionError::InvalidCommand(
            "extra shaft load must be finite and >= 0".into(),
        ));
    }
    if extra_load_w > 0.0 && !state.lit {
        return Err(PropulsionError::InvalidCommand(
            "extra shaft load requires an already-lit engine".into(),
        ));
    }
    if engine.shaft.multi_spool.is_some() && extra_load_w > 0.0 {
        return Err(PropulsionError::UnsupportedCombination(
            "the multi-spool turbofan does not accept a separate power-turbine takeoff".into(),
        ));
    }
    if !dt_s.is_finite() || dt_s < 0.0 {
        return Err(PropulsionError::InvalidCommand(
            "shaft dt must be finite and >= 0".into(),
        ));
    }
    if !state.spool_n.is_finite() || !(0.0..=1.0).contains(&state.spool_n) {
        return Err(PropulsionError::InvalidCommand(
            "shaft state spool must be finite in [0, 1]".into(),
        ));
    }
    let low_pressure_spool_n = state.low_pressure_spool_n.unwrap_or(state.spool_n);
    if !low_pressure_spool_n.is_finite() || !(0.0..=1.0).contains(&low_pressure_spool_n) {
        return Err(PropulsionError::InvalidCommand(
            "LP shaft state spool must be finite in [0, 1]".into(),
        ));
    }
    if engine.shaft.multi_spool.is_some() && state.low_pressure_spool_n.is_none() {
        return Err(PropulsionError::InvalidCommand(
            "multi-spool engine state is missing its LP rotor speed".into(),
        ));
    }
    if engine.shaft.multi_spool.is_none() && state.low_pressure_spool_n.is_some() {
        return Err(PropulsionError::InvalidCommand(
            "single-spool engine state unexpectedly contains an LP rotor speed".into(),
        ));
    }
    if !state.starter_charge_j.is_finite() || state.starter_charge_j < 0.0 {
        return Err(PropulsionError::InvalidCommand(
            "shaft state starter charge must be finite and >= 0".into(),
        ));
    }
    if !state.generator_temperature_k.is_finite() || state.generator_temperature_k <= 0.0 {
        return Err(PropulsionError::InvalidCommand(
            "shaft state generator temperature must be finite and positive".into(),
        ));
    }
    if command.starter_engaged && engine.shaft.starter.kind == StarterKind::None {
        return Err(PropulsionError::InvalidCommand(
            "no starter fitted; the engine cannot be cranked".into(),
        ));
    }
    let spool_n = state.spool_n;
    let commanded = command.throttle > 0.0;

    // Air path at the current spool speed. Fuel is scheduled whenever
    // throttle commands it (a start attempt below light-off still asks
    // "could this light?"); capacity is gated on the decided state.
    let (point, balance) = evaluate(spool_n, low_pressure_spool_n, commanded)?;

    // Light-off / self-sustain hysteresis.
    let mut lit = state.lit;
    if !commanded || point.fuel_flow_kg_s <= 0.0 || point.air_flow_kg_s <= 0.0 {
        lit = false;
    } else if lit {
        if spool_n < engine.shaft.self_sustain_n {
            lit = false;
        }
    } else if spool_n >= engine.shaft.light_off_n {
        lit = true;
    }
    // A commanded power-turbine output is credited only when the core is
    // burning. The same output is booked as the external propeller load
    // below, so the two cancel while the extracted gas work is reflected in
    // the nozzle state and available-power limit.
    let capacity_w = if lit {
        balance.capacity_w + extra_load_w
    } else {
        0.0
    };
    let multi_spool = engine.shaft.multi_spool;
    let accessory_speed = |spool: ShaftSpool| match (multi_spool, spool) {
        (Some(_), ShaftSpool::LowPressure) => low_pressure_spool_n,
        _ => spool_n,
    };
    let accessory_design_speed = |spool: ShaftSpool| match (multi_spool, spool) {
        (Some(spools), ShaftSpool::LowPressure) => spools.low_pressure_design_speed_rad_s,
        (Some(spools), ShaftSpool::HighPressure) => spools.high_pressure_design_speed_rad_s,
        (None, _) => engine
            .shaft
            .design_speed_rad_s
            .unwrap_or(engine.shaft_reference_power_w.max(1.0)),
    };
    let accessory_inertia = |spool: ShaftSpool| match (multi_spool, spool) {
        (Some(spools), ShaftSpool::LowPressure) => spools.low_pressure_effective_inertia(),
        (Some(spools), ShaftSpool::HighPressure) => spools.high_pressure_rotor_inertia_kg_m2,
        (None, _) => engine.shaft.rotor_inertia_kg_m2.unwrap_or(1.0),
    };

    // Pneumatic power from an APU supplements the starter's onboard
    // compressed-air reserve. External bleed is already booked as a shaft
    // load at the supplying APU, so only the reserve portion is charged here.
    let mut starter_active = false;
    let mut starter_shaft_w = 0.0;
    let mut starter_draw_w = 0.0;
    let mut external_starter_supply_w = 0.0;
    if command.starter_engaged {
        let starter = &engine.shaft.starter;
        if pneumatic_starter_power_w > 0.0 && starter.kind != StarterKind::Pneumatic {
            return Err(PropulsionError::InvalidCommand(
                "external pneumatic supply requires a pneumatic starter".into(),
            ));
        }
        if starter.power_w > 0.0 && starter.maximum_shaft_torque_nm.is_none() {
            let efficiency = starter.kind.efficiency();
            if starter.kind == StarterKind::Pneumatic {
                external_starter_supply_w =
                    pneumatic_starter_power_w.min(starter.power_w / efficiency);
                starter_shaft_w = external_starter_supply_w * efficiency;
            }
            let remaining_starter_power_w = (starter.power_w - starter_shaft_w).max(0.0);
            let from_charge = if dt_s > 0.0 {
                state.starter_charge_j * efficiency / dt_s
            } else {
                f64::INFINITY
            };
            let charge_shaft_w = remaining_starter_power_w.min(from_charge);
            starter_shaft_w += charge_shaft_w;
            starter_draw_w = charge_shaft_w / efficiency;
            starter_active = starter_shaft_w > 0.0;
        }
    }

    // Generator: online above cut-in, capped at rated power, shaft-side
    // draw is the electrical request scaled by efficiency. The shaft
    // itself is never padded: an overdrawn load bogs the spool down.
    let generator = &engine.shaft.generator;
    let generator_spool_n = accessory_speed(generator.attached_spool);
    let generator_design_speed = accessory_design_speed(generator.attached_spool);
    let generator_angular_speed = generator_spool_n * generator_design_speed;
    let generator_efficiency = generator.efficiency_at(generator_spool_n);
    let mut generator_electrical_w = 0.0;
    let mut generator_shaft_w = 0.0;
    if generator.fitted && generator.power_w > 0.0 && generator_spool_n >= generator.cut_in_spool_n
    {
        let thermal_power_limit_w = generator.thermal.map_or(generator.power_w, |thermal| {
            if dt_s == 0.0 {
                return generator.power_w;
            }
            let heat_capacity = thermal.heat_capacity_j_k;
            let conductance = thermal.conductance_w_k;
            let maximum_loss_w = (((heat_capacity + dt_s * conductance)
                * thermal.maximum_temperature_k
                - heat_capacity * state.generator_temperature_k)
                / dt_s
                - conductance * condition.ambient_temp_k)
                .max(0.0);
            let loss_per_electrical_w = (1.0 / generator_efficiency - 1.0).max(0.0);
            if loss_per_electrical_w <= f64::EPSILON {
                generator.power_w
            } else {
                (maximum_loss_w / loss_per_electrical_w).min(generator.power_w)
            }
        });
        generator_electrical_w = command
            .generator_load_w
            .min(generator.power_w)
            .min(thermal_power_limit_w);
        if let Some(maximum_torque_nm) = generator.maximum_shaft_torque_nm {
            generator_electrical_w = generator_electrical_w
                .min(maximum_torque_nm * generator_angular_speed * generator_efficiency);
        }
        generator_shaft_w = generator_electrical_w / generator_efficiency;
    }

    let generator_temperature_k = if let Some(thermal) = generator.thermal {
        if dt_s == 0.0 {
            state.generator_temperature_k
        } else {
            let waste_heat_w = (generator_shaft_w - generator_electrical_w).max(0.0);
            (thermal.heat_capacity_j_k * state.generator_temperature_k
                + dt_s * (waste_heat_w + thermal.conductance_w_k * condition.ambient_temp_k))
                / (thermal.heat_capacity_j_k + dt_s * thermal.conductance_w_k)
        }
    } else {
        state.generator_temperature_k
    };

    let total_demand_w = balance.demand_w + extra_load_w;
    let mut net_w =
        capacity_w + starter_shaft_w - total_demand_w - balance.friction_w - generator_shaft_w;
    let mut low_spool_next = state.low_pressure_spool_n;
    let mut spool_next;
    if let Some(spools) = multi_spool {
        let high_design_speed = spools.high_pressure_design_speed_rad_s;
        let low_design_speed = spools.low_pressure_design_speed_rad_s;
        let high_inertia = spools.high_pressure_rotor_inertia_kg_m2;
        let low_inertia = spools.low_pressure_effective_inertia();
        let high_omega = spool_n * high_design_speed;
        let low_omega = low_pressure_spool_n * low_design_speed;
        let hp_fraction = spools.high_pressure_turbine_power_fraction;
        let high_capacity_w = capacity_w * hp_fraction;
        let low_capacity_w = capacity_w * (1.0 - hp_fraction);
        let high_friction_w = SHAFT_FRICTION_FRACTION
            * engine.shaft_reference_power_w
            * hp_fraction
            * spool_n.powi(3);
        let low_friction_w = SHAFT_FRICTION_FRACTION
            * engine.shaft_reference_power_w
            * (1.0 - hp_fraction)
            * low_pressure_spool_n.powi(3);
        let low_fan_demand_w = balance.low_pressure_fan_demand_w / spools.fan_gear_efficiency;
        let high_demand_torque_nm = if high_omega > 1.0e-9 {
            balance.high_pressure_demand_w / high_omega
        } else {
            0.0
        };
        let low_demand_torque_nm = if low_omega > 1.0e-9 {
            low_fan_demand_w / low_omega
        } else {
            0.0
        };
        let high_friction_torque_nm = if high_omega > 1.0e-9 {
            high_friction_w / high_omega
        } else {
            0.0
        };
        let low_friction_torque_nm = if low_omega > 1.0e-9 {
            low_friction_w / low_omega
        } else {
            0.0
        };
        // Turbine section power ratings define their design-speed torque;
        // power therefore falls with rotor speed and remains finite at rest.
        let mut high_base_torque_nm =
            high_capacity_w / high_design_speed - high_demand_torque_nm - high_friction_torque_nm;
        let mut low_base_torque_nm =
            low_capacity_w / low_design_speed - low_demand_torque_nm - low_friction_torque_nm;
        let generator_torque_nm = if generator_spool_n > 0.0 {
            generator_shaft_w / generator_angular_speed.max(1.0e-9)
        } else {
            0.0
        };
        match generator.attached_spool {
            ShaftSpool::HighPressure => high_base_torque_nm -= generator_torque_nm,
            ShaftSpool::LowPressure => low_base_torque_nm -= generator_torque_nm,
        }

        let starter = &engine.shaft.starter;
        let starter_spool_n = accessory_speed(starter.attached_spool);
        let starter_design_speed = accessory_design_speed(starter.attached_spool);
        let starter_omega = starter_spool_n * starter_design_speed;
        let starter_inertia = accessory_inertia(starter.attached_spool);
        let starter_base_torque_nm = match starter.attached_spool {
            ShaftSpool::HighPressure => high_base_torque_nm,
            ShaftSpool::LowPressure => low_base_torque_nm,
        };
        let mut starter_torque_nm = if command.starter_engaged {
            starter
                .maximum_shaft_torque_nm
                .unwrap_or(0.0)
                .min(starter.power_w / starter_omega.max(starter_design_speed * 1.0e-9))
        } else {
            0.0
        };
        if command.starter_engaged {
            let efficiency = starter.kind.efficiency();
            let available_energy_j = state.starter_charge_j + pneumatic_starter_power_w * dt_s;
            let starter_work_j = |torque_nm: f64| {
                let end_speed = (starter_omega
                    + (starter_base_torque_nm + torque_nm) * dt_s / starter_inertia)
                    .clamp(0.0, starter_design_speed);
                torque_nm * dt_s * (0.5 * (starter_omega + end_speed)).max(0.0)
            };
            if dt_s > 0.0 && starter_work_j(starter_torque_nm) / efficiency > available_energy_j {
                let mut low = 0.0;
                let mut high = starter_torque_nm;
                for _ in 0..48 {
                    let middle = 0.5 * (low + high);
                    if starter_work_j(middle) / efficiency <= available_energy_j {
                        low = middle;
                    } else {
                        high = middle;
                    }
                }
                starter_torque_nm = low;
            }
            let input_energy_j = if dt_s > 0.0 {
                starter_work_j(starter_torque_nm) / efficiency
            } else {
                0.0
            };
            let input_power_w = if dt_s > 0.0 {
                input_energy_j / dt_s
            } else {
                0.0
            };
            external_starter_supply_w = input_power_w.min(pneumatic_starter_power_w);
            starter_draw_w = (input_power_w - external_starter_supply_w).max(0.0);
            starter_shaft_w = if dt_s > 0.0 {
                starter_work_j(starter_torque_nm) / dt_s
            } else {
                0.0
            };
            starter_active = starter_torque_nm > 0.0;
        }
        match starter.attached_spool {
            ShaftSpool::HighPressure => high_base_torque_nm += starter_torque_nm,
            ShaftSpool::LowPressure => low_base_torque_nm += starter_torque_nm,
        }
        let high_end_speed =
            (high_omega + high_base_torque_nm * dt_s / high_inertia).clamp(0.0, high_design_speed);
        let low_end_speed =
            (low_omega + low_base_torque_nm * dt_s / low_inertia).clamp(0.0, low_design_speed);
        spool_next = high_end_speed / high_design_speed;
        low_spool_next = Some(low_end_speed / low_design_speed);
        let high_mean_speed = 0.5 * (high_omega + high_end_speed);
        let low_mean_speed = 0.5 * (low_omega + low_end_speed);
        net_w = high_base_torque_nm * high_mean_speed + low_base_torque_nm * low_mean_speed;
    } else if let (Some(design_speed), Some(rotor_inertia)) = (
        engine.shaft.design_speed_rad_s,
        engine.shaft.rotor_inertia_kg_m2,
    ) {
        let angular_speed = spool_n * design_speed;
        let base_engine_power_w = capacity_w - total_demand_w - balance.friction_w;
        let generator_torque_nm = if angular_speed > design_speed * 1.0e-9 {
            generator_shaft_w / angular_speed
        } else {
            0.0
        };
        let base_torque_nm = if angular_speed > design_speed * 1.0e-9 {
            base_engine_power_w / angular_speed - generator_torque_nm
        } else {
            0.0
        };
        let starter = &engine.shaft.starter;
        let mut starter_torque_nm = if command.starter_engaged {
            if let Some(maximum_torque_nm) = starter.maximum_shaft_torque_nm {
                maximum_torque_nm.min(starter.power_w / angular_speed.max(design_speed * 1.0e-9))
            } else {
                starter_shaft_w / design_speed
            }
        } else {
            0.0
        };

        if command.starter_engaged && starter.maximum_shaft_torque_nm.is_some() {
            let efficiency = starter.kind.efficiency();
            let available_energy_j = state.starter_charge_j + pneumatic_starter_power_w * dt_s;
            let starter_work_j = |torque_nm: f64| {
                let end_speed = (angular_speed
                    + (base_torque_nm + torque_nm) * dt_s / rotor_inertia)
                    .clamp(0.0, design_speed);
                torque_nm * dt_s * (0.5 * (angular_speed + end_speed)).max(0.0)
            };
            if dt_s > 0.0 && starter_work_j(starter_torque_nm) / efficiency > available_energy_j {
                let mut low = 0.0;
                let mut high = starter_torque_nm;
                for _ in 0..48 {
                    let middle = 0.5 * (low + high);
                    if starter_work_j(middle) / efficiency <= available_energy_j {
                        low = middle;
                    } else {
                        high = middle;
                    }
                }
                starter_torque_nm = low;
            }
            let input_energy_j = if dt_s > 0.0 {
                starter_work_j(starter_torque_nm) / efficiency
            } else {
                0.0
            };
            let input_power_w = if dt_s > 0.0 {
                input_energy_j / dt_s
            } else {
                0.0
            };
            external_starter_supply_w = input_power_w.min(pneumatic_starter_power_w);
            starter_draw_w = (input_power_w - external_starter_supply_w).max(0.0);
            starter_shaft_w = if dt_s > 0.0 {
                starter_work_j(starter_torque_nm) / dt_s
            } else {
                0.0
            };
            starter_active = starter_torque_nm > 0.0;
        }

        let net_torque_nm = base_torque_nm + starter_torque_nm;
        let end_speed =
            (angular_speed + net_torque_nm * dt_s / rotor_inertia).clamp(0.0, design_speed);
        net_w = base_engine_power_w + starter_shaft_w - generator_shaft_w;
        spool_next = end_speed / design_speed;
    } else {
        spool_next = spool_n + dt_s * net_w / (engine.shaft_reference_power_w * engine.spool_tau_s);
    }
    if !spool_next.is_finite() {
        return Err(PropulsionError::InvalidCommand(
            "shaft integration produced a non-finite spool speed".into(),
        ));
    }
    spool_next = spool_next.clamp(0.0, 1.0);

    let starter_charge_j = if starter_active {
        (state.starter_charge_j - starter_draw_w * dt_s).max(0.0)
    } else {
        state.starter_charge_j
    };

    Ok((
        JetShaftState {
            spool_n: spool_next,
            low_pressure_spool_n: low_spool_next,
            lit,
            starter_charge_j,
            generator_temperature_k,
        },
        ShaftTelemetry {
            spool_n: spool_next,
            lit,
            starter_active,
            starter_shaft_power_w: starter_shaft_w,
            starter_draw_w,
            external_starter_supply_w,
            starter_charge_j,
            generator_electrical_w,
            generator_shaft_draw_w: generator_shaft_w,
            generator_temperature_k,
            demand_w: total_demand_w,
            capacity_w,
            friction_w: balance.friction_w,
            net_w,
        },
    ))
}

/// Steady helpers used by docs/tests: balance without a command.
impl CompiledAirbreather {
    /// Shaft balance at an explicit spool speed and ignition command
    /// (no nozzle matching — the lean form used by the steady spool
    /// solver).
    pub fn shaft_balance(
        &self,
        condition: &FlightCondition,
        throttle: f64,
        spool_n: f64,
        ignition: bool,
    ) -> Result<ShaftBalance, PropulsionError> {
        Ok(self
            .operating_point_at_spool(condition, throttle, spool_n, ignition)?
            .1)
    }
}

#[cfg(test)]
mod tests {
    use super::super::{
        AirCycle, AirbreathingSpec, ChamberMaterial, IntakeKind, JetFuel, flight_condition,
    };
    use super::*;
    use crate::AtmosphereConfig;

    fn jet_spec(starter: StarterSpec, generator: GeneratorSpec) -> AirbreathingSpec {
        AirbreathingSpec {
            name: "shaft-test jet".into(),
            cycle: AirCycle::Turbojet,
            fuel: JetFuel::Kerosene,
            intake_area_m2: 0.5,
            intake: IntakeKind::Pitot,
            compressor_ratio: 8.0,
            bypass_ratio: 0.0,
            fan_pressure_ratio: 1.0,
            turbine_inlet_temp_k: 1400.0,
            afterburner: false,
            reheat_temp_k: 0.0,
            turbine_material: ChamberMaterial::nickel_superalloy(),
            spool_tau_s: 5.0,
            shaft: ShaftSpec {
                starter,
                generator,
                ..ShaftSpec::default()
            },
        }
    }

    fn sl_static() -> FlightCondition {
        flight_condition(
            &AtmosphereConfig::default().sample(0.0).expect("SL sample"),
            0.0,
        )
        .expect("static condition")
    }

    fn electric_starter() -> StarterSpec {
        StarterSpec {
            kind: StarterKind::Electric,
            power_w: 4.0e6,
            charge_j: 1.0e9,
            resource: None,
            attached_spool: ShaftSpool::HighPressure,
            specific_energy_j_kg: 0.0,
            maximum_shaft_torque_nm: None,
            mass_kg: 30.0,
        }
    }

    #[test]
    fn apu_pneumatic_bleed_cranks_without_spending_stored_reserve() {
        let starter = StarterSpec {
            kind: StarterKind::Pneumatic,
            power_w: 4.0e6,
            charge_j: 1.0e9,
            resource: None,
            attached_spool: ShaftSpool::HighPressure,
            specific_energy_j_kg: 0.0,
            maximum_shaft_torque_nm: None,
            mass_kg: 30.0,
        };
        let engine = jet_spec(starter, GeneratorSpec::default())
            .compile()
            .expect("pneumatic starter engine compiles");
        let condition = sl_static();
        let mut state = JetShaftState::cold(&engine);
        state.starter_charge_j = 0.0;
        let command = ShaftCommand {
            throttle: 1.0,
            starter_engaged: true,
            generator_load_w: 0.0,
        };
        let pneumatic_input_w = engine.shaft.starter.power_w / StarterKind::Pneumatic.efficiency();
        let (next, telemetry) = advance_jet_shaft_loaded_with_starter_power(
            &engine,
            state,
            &command,
            &condition,
            0.1,
            0.0,
            pneumatic_input_w,
        )
        .expect("APU bleed supplies the pneumatic starter");
        assert!(telemetry.starter_active);
        assert!((telemetry.external_starter_supply_w - pneumatic_input_w).abs() < 1e-9);
        assert!(telemetry.starter_shaft_power_w > 0.0);
        assert_eq!(telemetry.starter_draw_w, 0.0);
        assert_eq!(next.starter_charge_j, 0.0);
        assert!(next.spool_n > state.spool_n);
    }

    /// Integrate a shaft from `state` at fixed command until `steps`
    /// elapse or `done` reports completion; returns the last state and
    /// telemetry.
    fn integrate(
        engine: &CompiledAirbreather,
        mut state: JetShaftState,
        command: &ShaftCommand,
        condition: &FlightCondition,
        steps: usize,
        done: impl Fn(&JetShaftState) -> bool,
    ) -> (JetShaftState, ShaftTelemetry) {
        let mut telemetry = advance_jet_shaft(engine, state, command, condition, 0.0)
            .expect("telemetry probe")
            .1;
        for _ in 0..steps {
            if done(&state) {
                break;
            }
            let (next, next_telemetry) =
                advance_jet_shaft(engine, state, command, condition, 0.1).expect("step");
            state = next;
            telemetry = next_telemetry;
        }
        (state, telemetry)
    }

    #[test]
    fn starter_cold_start_lights_and_sustains() {
        // A fitted electric starter cranks a stopped engine from zero:
        // spool rises against demand, light-off trips at the authored
        // threshold, the starter is cut, and the core accelerates to
        // self-sustaining speed on its own power.
        let engine = jet_spec(electric_starter(), GeneratorSpec::default())
            .compile()
            .expect("compiles");
        let condition = sl_static();
        let mut state = JetShaftState::cold(&engine);
        assert_eq!(state.spool_n, 0.0);
        assert!(!state.lit);
        let command = ShaftCommand {
            throttle: 1.0,
            starter_engaged: true,
            generator_load_w: 0.0,
        };
        let mut lit_at_s = None;
        let mut elapsed = 0.0;
        let mut telemetry = advance_jet_shaft(&engine, state, &command, &condition, 0.0)
            .expect("probe")
            .1;
        for _ in 0..4000 {
            let (next, next_telemetry) =
                advance_jet_shaft(&engine, state, &command, &condition, 0.1).expect("start step");
            elapsed += 0.1;
            state = next;
            telemetry = next_telemetry;
            if lit_at_s.is_none() && state.lit {
                lit_at_s = Some(elapsed);
            }
            if state.lit && (state.spool_n - 1.0).abs() < 1e-3 && lit_at_s.is_some() {
                break;
            }
        }
        let lit_at = lit_at_s.expect("engine lights during crank");
        assert!(
            lit_at < 120.0,
            "light-off at {lit_at:.1} s is slower than the authored starter allows"
        );
        assert!(
            (state.spool_n - 1.0).abs() < 1e-3,
            "self-sustaining spool settles at design speed, got {}",
            state.spool_n
        );
        assert!(state.lit);
        assert!(
            state.starter_charge_j < 1.0e9,
            "cranking must spend stored starter energy"
        );
        assert!(telemetry.capacity_w > 0.0);
        assert!(telemetry.demand_w > 0.0);
        // After light-off the starter carried the spool, not the fire:
        // the crank window is bounded by light-off being reached first.
        assert!(lit_at > 0.0);
    }

    #[test]
    fn starterless_engine_cannot_start_at_rest() {
        // Deliberate starterless design: no hardware, so engaging the
        // starter command is refused, and at zero airspeed the engine
        // never spins or lights (no free intake flow: suction scales
        // with spool speed).
        let engine = jet_spec(StarterSpec::default(), GeneratorSpec::default())
            .compile()
            .expect("compiles");
        let condition = sl_static();
        let state = JetShaftState::cold(&engine);
        let engage = ShaftCommand {
            throttle: 1.0,
            starter_engaged: true,
            generator_load_w: 0.0,
        };
        assert!(
            advance_jet_shaft(&engine, state, &engage, &condition, 0.1).is_err(),
            "engaging an unfitted starter must be refused"
        );
        let run = ShaftCommand {
            throttle: 1.0,
            starter_engaged: false,
            generator_load_w: 0.0,
        };
        let (final_state, telemetry) = integrate(&engine, state, &run, &condition, 1000, |s| {
            s.lit || s.spool_n > 0.0
        });
        assert_eq!(final_state.spool_n, 0.0, "no airspeed, no windmilling");
        assert!(!final_state.lit);
        assert!(!telemetry.starter_active);
        assert_eq!(telemetry.capacity_w, 0.0);
    }

    #[test]
    fn windmill_relight_at_speed() {
        // Starterless relight: inlet-driven rotation already above
        // light-off lets the core light immediately and run on turbine
        // power alone.
        let engine = jet_spec(StarterSpec::default(), GeneratorSpec::default())
            .compile()
            .expect("compiles");
        let sample = AtmosphereConfig::default().sample(0.0).expect("SL");
        let condition =
            flight_condition(&sample, 0.8 * sample.speed_of_sound_mps).expect("cruise condition");
        let state = JetShaftState {
            spool_n: 0.6,
            low_pressure_spool_n: None,
            lit: false,
            starter_charge_j: 0.0,
            generator_temperature_k: 293.15,
        };
        let command = ShaftCommand {
            throttle: 1.0,
            starter_engaged: false,
            generator_load_w: 0.0,
        };
        let (next, telemetry) =
            advance_jet_shaft(&engine, state, &command, &condition, 0.1).expect("relight");
        assert!(
            next.lit,
            "windmilling above light-off with burnable air must relight"
        );
        assert!(telemetry.capacity_w > 0.0);
        assert!(telemetry.net_w > 0.0, "lit core accelerates the spool");
        let (settled, telemetry) = integrate(&engine, next, &command, &condition, 2000, |s| {
            (s.spool_n - 1.0).abs() < 1e-3
        });
        // The steady equilibrium here is NOT the sea-level static
        // design point: at Mach 0.8 the ram-heated corrected-flow
        // demand grows faster than turbine capacity, so the sustainable
        // spool sits below 1.0. The real invariants are: it settles
        // (net power ~ 0, documented error envelope), stays above
        // light-off, and keeps burning.
        assert!(
            (telemetry.net_w).abs() < 0.01 * engine.shaft_reference_power_w,
            "settled net {:.0} W not near equilibrium",
            telemetry.net_w
        );
        assert!(settled.spool_n > engine.shaft.light_off_n);
        assert!(settled.lit);
    }

    #[test]
    fn light_off_and_self_sustain_hysteresis() {
        // dt = 0 pins the spool so only the combustion state machine is
        // under test: unlit below light-off stays unlit even with fuel
        // scheduled; lit inside the hysteresis band survives; lit below
        // self-sustain flames out.
        let engine = jet_spec(electric_starter(), GeneratorSpec::default())
            .compile()
            .expect("compiles");
        let condition = sl_static();
        let command = ShaftCommand {
            throttle: 1.0,
            starter_engaged: false,
            generator_load_w: 0.0,
        };
        let band = JetShaftState {
            spool_n: 0.12, // between self_sustain (0.10) and light_off (0.15)
            low_pressure_spool_n: None,
            lit: false,
            starter_charge_j: 0.0,
            generator_temperature_k: 293.15,
        };
        let (attempt, _) =
            advance_jet_shaft(&engine, band, &command, &condition, 0.0).expect("attempt");
        assert!(!attempt.lit, "below light-off an unlit core cannot light");
        assert_eq!(attempt.spool_n, 0.12);

        let lit_band = JetShaftState { lit: true, ..band };
        let (held, telemetry) =
            advance_jet_shaft(&engine, lit_band, &command, &condition, 0.0).expect("hold");
        assert!(held.lit, "self-sustain hysteresis holds a lit core");

        let dying = JetShaftState {
            spool_n: 0.09, // below self_sustain
            low_pressure_spool_n: None,
            lit: true,
            starter_charge_j: 0.0,
            generator_temperature_k: 293.15,
        };
        let (flameout, telemetry_after) =
            advance_jet_shaft(&engine, dying, &command, &condition, 0.0).expect("flameout");
        assert!(!flameout.lit, "below self-sustain the core dies");
        assert_eq!(telemetry_after.capacity_w, 0.0);
        let _ = telemetry;

        // Commanded off (throttle 0) always shuts the core down.
        let shutdown_command = ShaftCommand {
            throttle: 0.0,
            starter_engaged: false,
            generator_load_w: 0.0,
        };
        let (shut, _) = advance_jet_shaft(&engine, lit_band, &shutdown_command, &condition, 0.0)
            .expect("shutdown");
        assert!(!shut.lit);
    }

    #[test]
    fn starter_charge_depletes_to_a_dead_crank() {
        // A tiny charge runs the starter dry: the delivered power is
        // energy-bounded from the first step, then the starter stops
        // contributing and the spool decays against demand.
        let engine = jet_spec(
            StarterSpec {
                kind: StarterKind::Electric,
                power_w: 4.0e6,
                charge_j: 1.0e5, // 0.1 MJ: seconds of cranking
                resource: None,
                attached_spool: ShaftSpool::HighPressure,
                specific_energy_j_kg: 0.0,
                maximum_shaft_torque_nm: None,
                mass_kg: 5.0,
            },
            GeneratorSpec::default(),
        )
        .compile()
        .expect("compiles");
        let condition = sl_static();
        let command = ShaftCommand {
            throttle: 1.0,
            starter_engaged: true,
            generator_load_w: 0.0,
        };
        let mut state = JetShaftState::cold(&engine);
        let mut first = true;
        for _ in 0..50 {
            let (next, telemetry) =
                advance_jet_shaft(&engine, state, &command, &condition, 1.0).expect("step");
            state = next;
            if first {
                // Energy-bounded: one second can draw at most the whole
                // stored charge (scaled to shaft side by efficiency).
                assert!(telemetry.starter_shaft_power_w <= 1.0e5 / 0.85 + 1e-9);
                first = false;
            }
            if state.starter_charge_j == 0.0 {
                break;
            }
        }
        assert_eq!(state.starter_charge_j, 0.0, "charge must run dry");
        let (dead, telemetry) =
            advance_jet_shaft(&engine, state, &command, &condition, 1.0).expect("dead crank");
        assert!(!telemetry.starter_active);
        assert_eq!(telemetry.starter_shaft_power_w, 0.0);
        assert!(telemetry.net_w < 0.0, "dead crank decays against demand");
        assert!(!dead.lit);
    }

    #[test]
    fn generator_load_and_cut_in_enter_the_balance() {
        // A fitted generator above cut-in drags the shaft (electrical
        // request / efficiency), so a loaded engine settles below its
        // unloaded equilibrium; below cut-in it delivers nothing.
        let generator = GeneratorSpec {
            fitted: true,
            power_w: 5.0e6,
            efficiency: 0.9,
            efficiency_map: Vec::new(),
            thermal: None,
            attached_spool: ShaftSpool::HighPressure,
            maximum_shaft_torque_nm: None,
            cut_in_spool_n: 0.5,
            mass_kg: 40.0,
        };
        let engine = jet_spec(StarterSpec::default(), generator)
            .compile()
            .expect("compiles");
        let condition = sl_static();
        let running = JetShaftState::running(&engine);
        let unloaded = ShaftCommand {
            throttle: 1.0,
            starter_engaged: false,
            generator_load_w: 0.0,
        };
        let loaded = ShaftCommand {
            generator_load_w: 5.0e6,
            ..unloaded
        };
        let (_, free_telemetry) =
            advance_jet_shaft(&engine, running, &unloaded, &condition, 0.0).expect("free");
        let (_, load_telemetry) =
            advance_jet_shaft(&engine, running, &loaded, &condition, 0.0).expect("loaded");
        assert_eq!(load_telemetry.generator_electrical_w, 5.0e6);
        assert!((load_telemetry.generator_shaft_draw_w - 5.0e6 / 0.9).abs() < 1.0);
        assert!(load_telemetry.net_w < free_telemetry.net_w);
        assert!(
            (load_telemetry.net_w - (free_telemetry.net_w - 5.0e6 / 0.9)).abs() < 1.0,
            "generator drag is exactly the electrical request scaled by efficiency"
        );

        // Below cut-in the generator is offline: no delivery, no drag.
        let slow = JetShaftState {
            spool_n: 0.3,
            ..running
        };
        let (_, cut_in) =
            advance_jet_shaft(&engine, slow, &loaded, &condition, 0.0).expect("below cut-in");
        assert_eq!(cut_in.generator_electrical_w, 0.0);
        assert_eq!(cut_in.generator_shaft_draw_w, 0.0);

        // Rating caps the request.
        let over = ShaftCommand {
            generator_load_w: 50.0e6,
            ..unloaded
        };
        let (_, capped) =
            advance_jet_shaft(&engine, running, &over, &condition, 0.0).expect("capped");
        assert_eq!(capped.generator_electrical_w, 5.0e6);
    }

    #[test]
    fn generator_efficiency_map_and_thermal_envelope_limit_real_bus_output() {
        let mut mapped = GeneratorSpec {
            fitted: true,
            power_w: 1_000.0,
            efficiency: 0.9,
            efficiency_map: vec![
                GeneratorEfficiencyPoint {
                    spool_n: 0.4,
                    efficiency: 0.5,
                },
                GeneratorEfficiencyPoint {
                    spool_n: 0.8,
                    efficiency: 0.9,
                },
            ],
            thermal: Some(GeneratorThermalSpec {
                heat_capacity_j_k: 100.0,
                conductance_w_k: 1.0,
                initial_temperature_k: 399.0,
                maximum_temperature_k: 400.0,
            }),
            attached_spool: ShaftSpool::HighPressure,
            maximum_shaft_torque_nm: None,
            cut_in_spool_n: 0.5,
            mass_kg: 2.0,
        };
        assert!((mapped.efficiency_at(0.6) - 0.7).abs() < 1.0e-12);
        let engine = jet_spec(StarterSpec::default(), mapped.clone())
            .compile()
            .expect("mapped, thermally rated generator");
        let condition = sl_static();
        let state = JetShaftState {
            spool_n: 0.6,
            ..JetShaftState::running(&engine)
        };
        let command = ShaftCommand {
            throttle: 1.0,
            starter_engaged: false,
            generator_load_w: 1_000.0,
        };
        let (next, limited) =
            advance_jet_shaft(&engine, state, &command, &condition, 1.0).expect("limited load");
        assert!(limited.generator_electrical_w < 1_000.0);
        assert!(limited.generator_electrical_w > 0.0);
        assert!(limited.generator_temperature_k <= 400.0 + 1.0e-10);
        assert_eq!(
            next.generator_temperature_k,
            limited.generator_temperature_k
        );

        mapped.efficiency_map = vec![
            GeneratorEfficiencyPoint {
                spool_n: 0.8,
                efficiency: 0.8,
            },
            GeneratorEfficiencyPoint {
                spool_n: 0.8,
                efficiency: 0.9,
            },
        ];
        assert!(jet_spec(StarterSpec::default(), mapped).compile().is_err());
    }

    #[test]
    fn torque_rated_starter_cranks_from_rest_and_generator_respects_torque_speed() {
        let mut starter = electric_starter();
        starter.power_w = 1_000.0;
        starter.charge_j = 10_000.0;
        starter.maximum_shaft_torque_nm = Some(100.0);
        let mut starter_spec = jet_spec(starter, GeneratorSpec::default());
        starter_spec.shaft.design_speed_rad_s = Some(1_000.0);
        starter_spec.shaft.rotor_inertia_kg_m2 = Some(10.0);
        let starter_engine = starter_spec.compile().expect("torque starter engine");
        let mut unbounded_starter = starter_spec.clone();
        if let StarterKind::Electric = unbounded_starter.shaft.starter.kind {
            unbounded_starter.shaft.starter.maximum_shaft_torque_nm = None;
        }
        assert!(unbounded_starter.compile().is_err());
        let condition = sl_static();
        let command = ShaftCommand {
            throttle: 1.0,
            starter_engaged: true,
            generator_load_w: 0.0,
        };
        let (cranked, starter_telemetry) = advance_jet_shaft(
            &starter_engine,
            JetShaftState::cold(&starter_engine),
            &command,
            &condition,
            0.1,
        )
        .expect("torque starter step");
        assert!(cranked.spool_n > 0.0);
        assert!(starter_telemetry.starter_shaft_power_w > 0.0);
        assert!(starter_telemetry.starter_draw_w > 0.0);

        let mut generator = GeneratorSpec {
            fitted: true,
            power_w: 10_000.0,
            efficiency: 0.8,
            efficiency_map: Vec::new(),
            thermal: None,
            attached_spool: ShaftSpool::HighPressure,
            maximum_shaft_torque_nm: Some(10.0),
            cut_in_spool_n: 0.5,
            mass_kg: 2.0,
        };
        generator.validate(AirCycle::Turbojet).expect("generator");
        let mut generator_spec = jet_spec(StarterSpec::default(), generator.clone());
        generator_spec.shaft.design_speed_rad_s = Some(1_000.0);
        generator_spec.shaft.rotor_inertia_kg_m2 = Some(10.0);
        let generator_engine = generator_spec.compile().expect("torque generator engine");
        let mut unbounded_generator = generator_spec.clone();
        unbounded_generator.shaft.generator.maximum_shaft_torque_nm = None;
        assert!(unbounded_generator.compile().is_err());
        let state = JetShaftState {
            spool_n: 0.6,
            ..JetShaftState::running(&generator_engine)
        };
        let load = ShaftCommand {
            throttle: 1.0,
            starter_engaged: false,
            generator_load_w: 10_000.0,
        };
        let (_, telemetry) = advance_jet_shaft(&generator_engine, state, &load, &condition, 0.0)
            .expect("torque-limited generator step");
        assert!((telemetry.generator_electrical_w - 4_800.0).abs() < 1.0e-9);
        assert!(telemetry.generator_shaft_draw_w <= 6_000.0 + 1.0e-9);

        generator.maximum_shaft_torque_nm = Some(f64::NAN);
        assert!(generator.validate(AirCycle::Turbojet).is_err());
    }

    #[test]
    fn two_spool_turbofan_tracks_independent_rotors_and_accessory_attachment() {
        let mut starter = electric_starter();
        starter.power_w = 12_000.0;
        starter.charge_j = 2_000_000.0;
        starter.maximum_shaft_torque_nm = Some(60.0);
        starter.attached_spool = ShaftSpool::LowPressure;
        let mut generator = GeneratorSpec {
            fitted: true,
            power_w: 20_000.0,
            efficiency: 0.9,
            maximum_shaft_torque_nm: Some(50.0),
            cut_in_spool_n: 0.5,
            mass_kg: 8.0,
            attached_spool: ShaftSpool::LowPressure,
            ..GeneratorSpec::default()
        };
        generator.thermal = None;
        let mut spec = jet_spec(starter, generator);
        spec.cycle = AirCycle::Turbofan;
        spec.bypass_ratio = 3.0;
        spec.fan_pressure_ratio = 1.5;
        let condition = sl_static();
        let mut calibration_spec = spec.clone();
        calibration_spec.shaft.starter.maximum_shaft_torque_nm = None;
        calibration_spec.shaft.generator.maximum_shaft_torque_nm = None;
        let calibration_engine = calibration_spec
            .compile()
            .expect("single-spool design calibration engine");
        let (_, design_balance) = calibration_engine
            .operating_point_at_spool(&condition, 1.0, 1.0, true)
            .expect("design operating point");
        let fan_gear_efficiency = 0.95;
        let hp_work_fraction = design_balance.high_pressure_demand_w
            / (design_balance.high_pressure_demand_w
                + design_balance.low_pressure_fan_demand_w / fan_gear_efficiency);
        spec.shaft.multi_spool = Some(MultiSpoolSpec {
            low_pressure_design_speed_rad_s: 500.0,
            low_pressure_rotor_inertia_kg_m2: 4.0,
            fan_rotor_inertia_kg_m2: 2.0,
            high_pressure_design_speed_rad_s: 1_000.0,
            high_pressure_rotor_inertia_kg_m2: 2.0,
            high_pressure_turbine_power_fraction: hp_work_fraction,
            fan_gear_speed_ratio: 0.5,
            fan_gear_efficiency,
        });
        let engine = spec.compile().expect("two-spool turbofan compiles");
        let mut unbalanced_spec = spec.clone();
        unbalanced_spec
            .shaft
            .multi_spool
            .as_mut()
            .unwrap()
            .high_pressure_turbine_power_fraction = 0.55;
        assert!(
            unbalanced_spec.compile().is_err(),
            "a turbine split that does not close both design rotor balances must be refused"
        );
        assert!(
            engine
                .operating_point_at_spool(&condition, 1.0, 1.0, true)
                .is_err()
        );
        let command = ShaftCommand {
            throttle: 1.0,
            starter_engaged: false,
            generator_load_w: 20_000.0,
        };
        let running = JetShaftState {
            low_pressure_spool_n: Some(0.8),
            ..JetShaftState::running(&engine)
        };
        let advance = |state, generator_load_w| {
            let command = ShaftCommand {
                generator_load_w,
                ..command
            };
            advance_jet_shaft_loaded_with_starter_power_and_spools(
                &engine,
                state,
                &command,
                &condition,
                0.01,
                0.0,
                0.0,
                |high_n, low_n, ignition| {
                    engine.operating_point_at_spools(
                        &condition,
                        command.throttle,
                        high_n,
                        low_n,
                        ignition,
                    )
                },
            )
            .expect("two-spool advance")
        };

        let (unloaded, _) = advance(running, 0.0);
        let (loaded, telemetry) = advance(running, 20_000.0);
        assert!(
            loaded.low_pressure_spool_n.unwrap() < unloaded.low_pressure_spool_n.unwrap(),
            "loaded LP={} unloaded LP={} generator={} W net={} W",
            loaded.low_pressure_spool_n.unwrap(),
            unloaded.low_pressure_spool_n.unwrap(),
            telemetry.generator_electrical_w,
            telemetry.net_w
        );
        assert!((loaded.spool_n - unloaded.spool_n).abs() < 1.0e-12);
        assert!(telemetry.generator_electrical_w > 0.0);

        let mut cold = JetShaftState::cold(&engine);
        cold.starter_charge_j = 2_000_000.0;
        let crank_command = ShaftCommand {
            throttle: 1.0,
            starter_engaged: true,
            generator_load_w: 0.0,
        };
        let (cranked, crank_telemetry) = advance_jet_shaft_loaded_with_starter_power_and_spools(
            &engine,
            cold,
            &crank_command,
            &condition,
            0.1,
            0.0,
            0.0,
            |high_n, low_n, ignition| {
                engine.operating_point_at_spools(
                    &condition,
                    crank_command.throttle,
                    high_n,
                    low_n,
                    ignition,
                )
            },
        )
        .expect("LP-mounted starter cranks the fan rotor");
        assert!(cranked.low_pressure_spool_n.unwrap() > 0.0);
        assert_eq!(cranked.spool_n, 0.0);
        assert!(crank_telemetry.starter_shaft_power_w > 0.0);

        let low_gear = engine.shaft.multi_spool.unwrap();
        assert!((low_gear.low_pressure_effective_inertia() - 4.5).abs() < 1.0e-12);
        let crank_from_rest = ShaftCommand {
            throttle: 0.0,
            starter_engaged: true,
            generator_load_w: 0.0,
        };
        let dt_s = 0.01;
        let (geared_crank, _) = advance_jet_shaft(
            &engine,
            JetShaftState::cold(&engine),
            &crank_from_rest,
            &condition,
            dt_s,
        )
        .expect("known torque-only LP spool step");
        let geared_omega =
            geared_crank.low_pressure_spool_n.unwrap() * low_gear.low_pressure_design_speed_rad_s;
        let expected_geared_omega = engine.shaft.starter.maximum_shaft_torque_nm.unwrap() * dt_s
            / low_gear.low_pressure_effective_inertia();
        assert!((geared_omega - expected_geared_omega).abs() < 1.0e-12);

        let mut direct_drive_spec = spec.clone();
        direct_drive_spec
            .shaft
            .multi_spool
            .as_mut()
            .unwrap()
            .fan_gear_speed_ratio = 1.0;
        let direct_drive = direct_drive_spec
            .compile()
            .expect("direct-drive turbofan compiles");
        let direct_gear = direct_drive.shaft.multi_spool.unwrap();
        let (direct_crank, _) = advance_jet_shaft(
            &direct_drive,
            JetShaftState::cold(&direct_drive),
            &crank_from_rest,
            &condition,
            dt_s,
        )
        .expect("direct-drive LP spool step");
        let direct_omega = direct_crank.low_pressure_spool_n.unwrap()
            * direct_gear.low_pressure_design_speed_rad_s;
        let expected_direct_omega = direct_drive.shaft.starter.maximum_shaft_torque_nm.unwrap()
            * dt_s
            / direct_gear.low_pressure_effective_inertia();
        assert!((direct_omega - expected_direct_omega).abs() < 1.0e-12);
        assert!(
            direct_omega < geared_omega,
            "fan inertia reflects through gear ratio squared"
        );

        let (slow_fan, _) = engine
            .operating_point_at_spools(&condition, 1.0, 1.0, 0.5, true)
            .expect("independent fan spool operating point");
        let (full_fan, _) = engine
            .operating_point_at_spools(&condition, 1.0, 1.0, 1.0, true)
            .expect("full fan speed operating point");
        assert!(slow_fan.air_flow_kg_s < full_fan.air_flow_kg_s);

        let static_sample = AtmosphereConfig::default()
            .sample(0.0)
            .expect("static atmosphere");
        let cruise_condition =
            flight_condition(&static_sample, 0.5 * static_sample.speed_of_sound_mps)
                .expect("cruise condition");
        let cruise = engine
            .operating_point(&cruise_condition, 0.85)
            .expect("coupled LP/HP steady equilibrium");
        let cruise_lp = cruise
            .low_pressure_spool_n
            .expect("steady point reports its fan spool");
        assert!(
            cruise.spool_n > 0.0 && cruise.spool_n < 1.0 && cruise_lp > 0.0 && cruise_lp < 1.0,
            "cruise equilibrium must solve both rotor speeds below redline: HP={} LP={}",
            cruise.spool_n,
            cruise_lp
        );
        let equilibrium = engine
            .shaft_balance_at_spools(&cruise_condition, 0.85, cruise.spool_n, cruise_lp, true)
            .expect("steady rotor balance");
        let tolerance_w = 1.0e-5 * equilibrium.capacity_w.max(1.0);
        assert!(equilibrium.high_pressure_net_w.abs() <= tolerance_w);
        assert!(equilibrium.low_pressure_net_w.abs() <= tolerance_w);

        let compiled_jet = super::super::CompiledJet::Air(Box::new(engine.clone()));
        let mount = super::super::JetMount {
            name: "two-spool-test".into(),
            engine: compiled_jet.clone(),
            position_body_m: [0.0; 3],
            thrust_axis_body: [1.0, 0.0, 0.0],
            gimbal_range_rad: 0.0,
        };
        let jet_command = super::super::JetCommand {
            dt_s: 0.1,
            starter_engaged: true,
            generator_load_w: 0.0,
            ..super::super::JetCommand::cold(&compiled_jet)
        };
        let (_, _, mount_state, _) = mount
            .estoc_point_with_telemetry(1.0, &condition, &jet_command)
            .expect("installed jet advances independent spool state");
        assert!(mount_state.low_pressure_spool_n.unwrap() > 0.0);
        assert_eq!(mount_state.spool_n, 0.0);
    }

    #[test]
    fn suction_scales_with_spool_speed() {
        // The v5 suction floor must follow actual compressor speed: a
        // stopped engine draws no air at zero airspeed, a spinning one
        // draws proportionally more.
        let engine = jet_spec(StarterSpec::default(), GeneratorSpec::default())
            .compile()
            .expect("compiles");
        let condition = sl_static();
        let stopped = engine
            .operating_point_at_spool(&condition, 1.0, 0.0, false)
            .expect("stopped")
            .0;
        assert_eq!(
            stopped.air_flow_kg_s, 0.0,
            "a stopped engine gets no free intake flow"
        );
        assert_eq!(stopped.thrust_n, 0.0);
        let mut previous = 0.0;
        for spool_n in [0.25, 0.5, 1.0] {
            let (point, balance) = engine
                .operating_point_at_spool(&condition, 1.0, spool_n, false)
                .expect("windmill");
            assert!(
                point.air_flow_kg_s > previous,
                "suction must grow with spool speed ({spool_n})"
            );
            assert!(point.suction_assisted, "labeled as suction-assisted");
            previous = point.air_flow_kg_s;
            assert!(balance.friction_w > 0.0, "spinning shaft books friction");
        }
        assert!(previous < engine.design_flow_kg_s + 1e-9);
    }

    #[test]
    fn steady_and_transient_equilibria_agree() {
        // The steady solver's equilibrium must be a fixed point of the
        // transient dynamics: advancing a shaft parked on the solved
        // spool barely moves it (pins bisection tolerance and the
        // normalization in one assertion).
        let engine = jet_spec(electric_starter(), GeneratorSpec::default())
            .compile()
            .expect("compiles");
        let condition = sl_static();
        for throttle in [1.0, 0.7] {
            let point = engine
                .operating_point(&condition, throttle)
                .expect("steady point");
            assert!(
                point.spool_n >= engine.shaft.light_off_n,
                "throttle {throttle} must find a sustainable spool"
            );
            assert!(point.lit);
            let state = JetShaftState {
                spool_n: point.spool_n,
                low_pressure_spool_n: None,
                lit: true,
                starter_charge_j: 0.0,
                generator_temperature_k: 293.15,
            };
            let command = ShaftCommand {
                throttle,
                starter_engaged: false,
                generator_load_w: 0.0,
            };
            let (next, _) =
                advance_jet_shaft(&engine, state, &command, &condition, 1.0).expect("consistency");
            assert!(
                (next.spool_n - point.spool_n).abs() < 1e-3,
                "steady spool {} vs transient {} at throttle {throttle}",
                point.spool_n,
                next.spool_n
            );
        }
    }

    #[test]
    fn design_point_equilibrium_is_exact_full_spool() {
        // Calibration anchor: at full throttle, sea level static, the
        // design point is an exact equilibrium — the solver returns
        // full spool and the reported thrust is the design thrust.
        let engine = jet_spec(electric_starter(), GeneratorSpec::default())
            .compile()
            .expect("compiles");
        let point = engine
            .operating_point(&sl_static(), 1.0)
            .expect("design point");
        assert!(
            point.spool_n >= 1.0 - 1e-6,
            "design equilibrium must sit at full spool, got {}",
            point.spool_n
        );
        assert!(point.lit);
        assert!(
            (point.thrust_n - engine.design_static_thrust_n).abs() / engine.design_static_thrust_n
                < 1e-6
        );
        assert!(point.suction_assisted);
    }

    #[test]
    fn shaft_validation_and_runtime_refusals() {
        // Ramjets refuse shaft hardware outright (no shaft), and the
        // inert default shaft is their only legal configuration.
        let ramjet = AirbreathingSpec {
            name: "ram".into(),
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
        assert!(ramjet.compile().is_ok(), "inert shaft on a ramjet is fine");
        let starter_ramjet = AirbreathingSpec {
            shaft: ShaftSpec {
                starter: electric_starter(),
                ..ShaftSpec::default()
            },
            ..ramjet.clone()
        };
        assert!(starter_ramjet.compile().is_err(), "starter on a ramjet");
        let generator_ramjet = AirbreathingSpec {
            shaft: ShaftSpec {
                generator: GeneratorSpec {
                    fitted: true,
                    power_w: 1.0e6,
                    efficiency: 0.9,
                    efficiency_map: Vec::new(),
                    thermal: None,
                    attached_spool: ShaftSpool::HighPressure,
                    maximum_shaft_torque_nm: None,
                    cut_in_spool_n: 0.5,
                    mass_kg: 20.0,
                },
                ..ShaftSpec::default()
            },
            ..ramjet.clone()
        };
        assert!(generator_ramjet.compile().is_err(), "generator on a ramjet");
        // Advancing a ramjet shaft is a programming error.
        let compiled_ramjet = ramjet.compile().expect("ramjet compiles");
        assert!(
            advance_jet_shaft(
                &compiled_ramjet,
                JetShaftState::running(&compiled_ramjet),
                &ShaftCommand {
                    throttle: 1.0,
                    starter_engaged: false,
                    generator_load_w: 0.0,
                },
                &sl_static(),
                0.1,
            )
            .is_err()
        );
        // Threshold sanity: self-sustain must sit at or below light-off.
        let bad_thresholds = AirbreathingSpec {
            shaft: ShaftSpec {
                light_off_n: 0.10,
                self_sustain_n: 0.20,
                ..ShaftSpec::default()
            },
            ..ramjet.clone()
        };
        assert!(bad_thresholds.compile().is_err());
        // NaN fails closed on commands.
        let engine = jet_spec(electric_starter(), GeneratorSpec::default())
            .compile()
            .expect("compiles");
        assert!(
            advance_jet_shaft(
                &engine,
                JetShaftState::cold(&engine),
                &ShaftCommand {
                    throttle: f64::NAN,
                    starter_engaged: false,
                    generator_load_w: 0.0,
                },
                &sl_static(),
                0.1,
            )
            .is_err()
        );
        assert!(
            advance_jet_shaft(
                &engine,
                JetShaftState::cold(&engine),
                &ShaftCommand {
                    throttle: 1.0,
                    starter_engaged: false,
                    generator_load_w: f64::INFINITY,
                },
                &sl_static(),
                0.1,
            )
            .is_err()
        );
    }

    #[test]
    fn vacuum_and_anoxic_conditions_never_light() {
        // No air and no oxygen both refuse light-off even with a
        // cranking starter and commanded ignition.
        let engine = jet_spec(electric_starter(), GeneratorSpec::default())
            .compile()
            .expect("compiles");
        let vacuum = FlightCondition {
            ambient_pa: 0.0,
            ..sl_static()
        };
        let anoxic = FlightCondition {
            composition: crate::atmosphere::AtmosphereComposition::anoxic(),
            ..sl_static()
        };
        let command = ShaftCommand {
            throttle: 1.0,
            starter_engaged: true,
            generator_load_w: 0.0,
        };
        for condition in [vacuum, anoxic] {
            let state = JetShaftState::cold(&engine);
            let (final_state, _) = integrate(&engine, state, &command, &condition, 200, |_| true);
            assert!(!final_state.lit, "no burnable air, no light-off");
        }
    }

    #[test]
    fn balance_helpers_expose_net_power() {
        // ShaftBalance::net_w is capacity minus demand and friction.
        let balance = ShaftBalance {
            demand_w: 60.0,
            high_pressure_demand_w: 60.0,
            low_pressure_fan_demand_w: 0.0,
            high_pressure_net_w: 30.0,
            low_pressure_net_w: 0.0,
            capacity_w: 100.0,
            power_takeoff_capacity_w: 0.0,
            friction_w: 10.0,
        };
        assert_eq!(balance.net_w(), 30.0);
        // A lean balance query matches the point-bearing one.
        let engine = jet_spec(electric_starter(), GeneratorSpec::default())
            .compile()
            .expect("compiles");
        let condition = sl_static();
        let lean = engine
            .shaft_balance(&condition, 1.0, 0.5, true)
            .expect("lean balance");
        let (_, full) = engine
            .operating_point_at_spool(&condition, 1.0, 0.5, true)
            .expect("full evaluation");
        assert!((lean.net_w() - full.net_w()).abs() < 1e-6);
    }

    #[test]
    fn loaded_shaft_zero_load_is_identical_and_bad_loads_are_refused() {
        let engine = jet_spec(electric_starter(), GeneratorSpec::default())
            .compile()
            .expect("compiles");
        let condition = sl_static();
        let state = JetShaftState::running(&engine);
        let command = ShaftCommand {
            throttle: 1.0,
            starter_engaged: false,
            generator_load_w: 0.0,
        };
        let plain = advance_jet_shaft(&engine, state, &command, &condition, 0.02)
            .expect("plain shaft step");
        let explicit_zero =
            advance_jet_shaft_loaded(&engine, state, &command, &condition, 0.02, 0.0)
                .expect("loaded shaft with zero takeoff");
        assert_eq!(plain, explicit_zero);
        assert!(
            advance_jet_shaft_loaded(&engine, state, &command, &condition, 0.02, -1.0).is_err()
        );
        assert!(
            advance_jet_shaft_loaded(&engine, state, &command, &condition, 0.02, f64::NAN).is_err()
        );
    }
}
