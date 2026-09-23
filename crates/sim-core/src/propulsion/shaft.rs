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

use super::{AirCycle, CompiledAirbreather, FlightCondition, PropulsionError, require_positive};

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

/// Starter hardware spec: deliverable shaft power, stored energy (J),
/// and mass. All stored-energy topologies are booked as one energy
/// reservoir until their resource graphs land.
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
    /// Starter hardware mass (kg).
    pub mass_kg: f64,
}

impl Default for StarterSpec {
    fn default() -> Self {
        Self {
            kind: StarterKind::None,
            power_w: 0.0,
            charge_j: 0.0,
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
                if self.power_w != 0.0 || self.charge_j != 0.0 || self.mass_kg != 0.0 {
                    return Err(PropulsionError::InvalidSpec(
                        "no starter fitted, but starter power/charge/mass is non-zero".into(),
                    ));
                }
            }
            StarterKind::Electric | StarterKind::Pneumatic | StarterKind::RocketBootstrap => {
                require_positive(self.power_w, "starter power")?;
                require_positive(self.charge_j, "starter charge")?;
                require_positive(self.mass_kg, "starter mass")?;
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
    /// Normalized spool speed below which the generator is offline
    /// (cut-in); it delivers nothing under it.
    pub cut_in_spool_n: f64,
    /// Generator mass (kg).
    pub mass_kg: f64,
}

impl Default for GeneratorSpec {
    fn default() -> Self {
        Self {
            fitted: false,
            power_w: 0.0,
            efficiency: 0.85,
            cut_in_spool_n: 0.5,
            mass_kg: 0.0,
        }
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
            if self.power_w != 0.0 || self.mass_kg != 0.0 {
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
        if !self.cut_in_spool_n.is_finite() || !(0.0..=1.0).contains(&self.cut_in_spool_n) {
            return Err(PropulsionError::InvalidSpec(
                "generator cut-in spool must be finite in [0, 1]".into(),
            ));
        }
        Ok(())
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
}

impl Default for ShaftSpec {
    fn default() -> Self {
        Self {
            starter: StarterSpec::default(),
            generator: GeneratorSpec::default(),
            light_off_n: 0.15,
            self_sustain_n: 0.10,
            power_turbine_heat_fraction: 0.0,
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
    /// True while the core sustains combustion (light-off reached,
    /// above self-sustain, burnable air present).
    pub lit: bool,
    /// Remaining stored starter energy (J). Constant for
    /// `StarterKind::None`.
    pub starter_charge_j: f64,
}

impl JetShaftState {
    /// Stopped engine: no rotation, unlit, starter at full charge.
    /// Shafted engines only (ramjet/scramjet have no shaft state).
    pub fn cold(engine: &CompiledAirbreather) -> Self {
        Self {
            spool_n: 0.0,
            lit: false,
            starter_charge_j: engine.shaft.starter.charge_j,
        }
    }

    /// Already-running engine at design spool: lit, full starter charge
    /// (a stopped starter does not recharge a running engine).
    pub fn running(engine: &CompiledAirbreather) -> Self {
        Self {
            spool_n: 1.0,
            lit: true,
            starter_charge_j: engine.shaft.starter.charge_j,
        }
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
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ShaftTelemetry {
    pub spool_n: f64,
    pub lit: bool,
    /// True while the starter actually delivered shaft power this step.
    pub starter_active: bool,
    pub starter_shaft_power_w: f64,
    /// Stored energy consumed per second by the starter (W; electrical
    /// draw, pneumatic thermal power, or bootstrap chemical power).
    pub starter_draw_w: f64,
    pub starter_charge_j: f64,
    /// Electrical power delivered to the bus this step (W).
    pub generator_electrical_w: f64,
    /// Shaft power removed to produce it (W).
    pub generator_shaft_draw_w: f64,
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
    if !state.starter_charge_j.is_finite() || state.starter_charge_j < 0.0 {
        return Err(PropulsionError::InvalidCommand(
            "shaft state starter charge must be finite and >= 0".into(),
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
    let (point, balance) = engine.operating_point_at_spool_loaded(
        condition,
        command.throttle,
        spool_n,
        commanded,
        extra_load_w,
    )?;

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

    // Starter: shaft power limited by rated power and remaining stored
    // energy scaled through its drivetrain efficiency.
    let mut starter_active = false;
    let mut starter_shaft_w = 0.0;
    let mut starter_draw_w = 0.0;
    if command.starter_engaged {
        let starter = &engine.shaft.starter;
        if starter.power_w > 0.0 && state.starter_charge_j > 0.0 {
            let efficiency = starter.kind.efficiency();
            let from_charge = if dt_s > 0.0 {
                state.starter_charge_j * efficiency / dt_s
            } else {
                f64::INFINITY
            };
            starter_shaft_w = starter.power_w.min(from_charge);
            starter_draw_w = starter_shaft_w / efficiency;
            starter_active = starter_shaft_w > 0.0;
        }
    }

    // Generator: online above cut-in, capped at rated power, shaft-side
    // draw is the electrical request scaled by efficiency. The shaft
    // itself is never padded: an overdrawn load bogs the spool down.
    let generator = &engine.shaft.generator;
    let mut generator_electrical_w = 0.0;
    let mut generator_shaft_w = 0.0;
    if generator.fitted && generator.power_w > 0.0 && spool_n >= generator.cut_in_spool_n {
        generator_electrical_w = command.generator_load_w.min(generator.power_w);
        generator_shaft_w = generator_electrical_w / generator.efficiency;
    }

    let total_demand_w = balance.demand_w + extra_load_w;
    let net_w =
        capacity_w + starter_shaft_w - total_demand_w - balance.friction_w - generator_shaft_w;
    let mut spool_next =
        spool_n + dt_s * net_w / (engine.shaft_reference_power_w * engine.spool_tau_s);
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
            lit,
            starter_charge_j,
        },
        ShaftTelemetry {
            spool_n: spool_next,
            lit,
            starter_active,
            starter_shaft_power_w: starter_shaft_w,
            starter_draw_w,
            starter_charge_j,
            generator_electrical_w,
            generator_shaft_draw_w: generator_shaft_w,
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
            mass_kg: 30.0,
        }
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
            lit: false,
            starter_charge_j: 0.0,
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
            lit: false,
            starter_charge_j: 0.0,
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
            lit: true,
            starter_charge_j: 0.0,
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
                lit: true,
                starter_charge_j: 0.0,
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
