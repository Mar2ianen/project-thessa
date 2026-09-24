//! Jet mounts: air-breathers and ESTOCs installed on a vehicle with body
//! stations, thrust axes, and gimbal ranges. Runtime evaluation is a pure
//! function of throttle, flight condition, and (for ESTOC) threaded mode
//! state — the vehicle stores no spool/mode memory.

use glam::DVec3;
use serde::{Deserialize, Serialize};

use super::GimbalEffector;
use super::mount::gimbal_pair;
use super::shaft::{JetShaftState, ShaftCommand, advance_jet_shaft, advance_jet_shaft_loaded_with};
use super::{
    CompiledAirbreather, CompiledEstoc, EnginePlumeState, EstocMode, EstocPoint, EstocTransient,
    FlightCondition, PropulsionError,
};

/// One installed jet of either family.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum CompiledJet {
    Air(Box<CompiledAirbreather>),
    Estoc(Box<CompiledEstoc>),
}

impl CompiledJet {
    /// Display name.
    pub fn name(&self) -> &str {
        match self {
            Self::Air(engine) => &engine.name,
            Self::Estoc(engine) => &engine.name,
        }
    }

    /// Dry mass in kg.
    pub fn dry_mass_kg(&self) -> f64 {
        match self {
            Self::Air(engine) => engine.dry_mass_kg,
            Self::Estoc(engine) => engine.dry_mass_kg,
        }
    }

    /// Spool time constant (s): the air-path compressor spool in both
    /// variants. Mode-transition lag is a separate dynamic (see
    /// [`CompiledJet::transition_tau_s`]).
    pub fn spool_tau_s(&self) -> f64 {
        match self {
            Self::Air(engine) => engine.spool_tau_s,
            Self::Estoc(engine) => engine.air.spool_tau_s,
        }
    }

    /// ESTOC mode-transition lag (s); `None` for plain air-breathers.
    pub fn transition_tau_s(&self) -> Option<f64> {
        match self {
            Self::Air(_) => None,
            Self::Estoc(engine) => Some(engine.transition_tau_s),
        }
    }
}

/// Jet command threading for stateless vehicle calls: manual override,
/// last mode, previous transient, timestep, and the live shaft state.
/// A fresh command has no previous transient, so it evaluates the
/// target directly, and carries an already-running shaft (analyzer
/// convention: the engine starts warm; `starter_charge_j` is a
/// `f64::MAX` sentinel meaning "starter untouched", since a running
/// engine never spends starter energy).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct JetCommand {
    pub manual: Option<EstocMode>,
    pub last_mode: EstocMode,
    /// Previous transition snapshot; `None` evaluates the fresh target.
    pub prev: Option<EstocTransient>,
    pub dt_s: f64,
    /// Live shaft state feeding the air path (spool speed, lit flag,
    /// starter charge). Plain air-breathers and the ESTOC air leg both
    /// schedule on it.
    pub shaft: JetShaftState,
    /// Starter engagement request for this step (advance with
    /// [`JetCommand::advance_shaft`]).
    pub starter_engaged: bool,
    /// Requested electrical generator load (W) for this step.
    pub generator_load_w: f64,
}

impl JetCommand {
    /// Fresh-start evaluation (direct target, no smoothing) with an
    /// already-running shaft.
    pub fn fresh() -> Self {
        Self {
            manual: None,
            last_mode: EstocMode::Air,
            prev: None,
            dt_s: 0.0,
            shaft: JetShaftState {
                spool_n: 1.0,
                lit: true,
                starter_charge_j: f64::MAX,
            },
            starter_engaged: false,
            generator_load_w: 0.0,
        }
    }

    /// Cold-start command: stopped, unlit shaft at full starter charge
    /// (per the engine's fitted starter topology).
    pub fn cold(engine: &CompiledJet) -> Self {
        let air = match engine {
            CompiledJet::Air(inner) => inner.as_ref(),
            CompiledJet::Estoc(inner) => &inner.air,
        };
        Self {
            shaft: JetShaftState::cold(air),
            starter_engaged: false,
            ..Self::fresh()
        }
    }

    /// Return the command to use on the next world tick after an
    /// operating-point evaluation. Manual selection and timestep are
    /// caller inputs; mode, the full transient snapshot, and the shaft
    /// state returned by [`JetMount::estoc_point`] are runtime state.
    pub fn with_state(
        &self,
        point: &EstocPoint,
        transient: EstocTransient,
        shaft: JetShaftState,
    ) -> Self {
        Self {
            last_mode: point.mode,
            prev: Some(transient),
            shaft,
            ..*self
        }
    }
}

/// One jet installed on the airframe.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JetMount {
    pub name: String,
    pub engine: CompiledJet,
    /// Nozzle station in vehicle body metres.
    pub position_body_m: [f64; 3],
    /// Unit thrust direction in vehicle body axes (usually +X).
    pub thrust_axis_body: [f64; 3],
    /// Gimbal half-range (rad, 0 = fixed).
    pub gimbal_range_rad: f64,
}

impl JetMount {
    /// The air-breather core of this mount (shared shaft plumbing).
    fn air_engine(&self) -> &CompiledAirbreather {
        match &self.engine {
            CompiledJet::Air(engine) => engine.as_ref(),
            CompiledJet::Estoc(engine) => &engine.air,
        }
    }

    /// Validate mount data (NaN fails closed; axis must be unit).
    pub fn validate(&self) -> Result<(), PropulsionError> {
        if self.name.trim().is_empty() {
            return Err(PropulsionError::InvalidSpec(
                "jet mount needs a name".into(),
            ));
        }
        if self.position_body_m.iter().any(|v| !v.is_finite()) {
            return Err(PropulsionError::InvalidSpec(
                "jet position must be finite".into(),
            ));
        }
        let axis = self.thrust_axis_body;
        if axis.iter().any(|v| !v.is_finite()) {
            return Err(PropulsionError::InvalidSpec(
                "jet axis must be finite".into(),
            ));
        }
        let norm = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
        if (norm - 1.0).abs() > 1e-9 {
            return Err(PropulsionError::InvalidSpec(
                "jet axis must be unit length".into(),
            ));
        }
        if !self.gimbal_range_rad.is_finite()
            || self.gimbal_range_rad < 0.0
            || self.gimbal_range_rad > 0.35
        {
            return Err(PropulsionError::InvalidSpec(
                "jet gimbal range must be finite in [0, 0.35]".into(),
            ));
        }
        Ok(())
    }

    /// Thrust magnitude (N) at throttle, condition, and jet command.
    pub fn thrust_n(
        &self,
        throttle: f64,
        condition: &FlightCondition,
        jet: &JetCommand,
    ) -> Result<f64, PropulsionError> {
        Ok(self.estoc_point(throttle, condition, jet)?.0.thrust_n)
    }

    /// Full point, updated transition snapshot, and next shaft state.
    /// The shaft is advanced one command step first (`command.dt_s`
    /// physics seconds, starter/generator honored), then the air path
    /// is evaluated at the resulting spool speed with `lit` as the
    /// ignition gate — so the returned state, point, and mode always
    /// agree. Thread all three forward via [`JetCommand::with_state`];
    /// `thrust_n`/`plume_state` discard the extras for one-shot calls.
    pub fn estoc_point(
        &self,
        throttle: f64,
        condition: &FlightCondition,
        jet: &JetCommand,
    ) -> Result<(EstocPoint, EstocTransient, JetShaftState), PropulsionError> {
        self.validate()?;
        let air_engine = self.air_engine();
        if !air_engine.cycle.has_shaft() {
            if jet.starter_engaged
                || !jet.generator_load_w.is_finite()
                || jet.generator_load_w != 0.0
            {
                return Err(PropulsionError::InvalidCommand(
                    "ramjet/scramjet mounts do not accept starter or generator commands".into(),
                ));
            }
            let point = air_engine.operating_point(condition, throttle)?;
            let snapshot = EstocTransient {
                thrust_n: point.thrust_n.max(0.0),
                fuel_flow_kg_s: point.fuel_flow_kg_s,
                bulk_fuel_flow_kg_s: point.bulk_fuel_flow_kg_s,
                boost_fuel_flow_kg_s: point.boost_fuel_flow_kg_s,
                oxidizer_flow_kg_s: 0.0,
                air_flow_kg_s: point.air_flow_kg_s,
                exhaust_temp_k: point.exhaust_temp_k,
                exhaust_velocity_mps: point.exhaust_velocity_mps,
                exit_pressure_pa: point.exit_pressure_pa,
                exit_mach: point.exit_mach,
                compressor_inlet_total_temp_k: point.compressor_inlet_total_temp_k,
                precooler_heat_flow_w: point.precooler_heat_flow_w,
                precooler_wall_heat_flow_w: point.precooler_wall_heat_flow_w,
                precooler_wall_temp_k: 0.0,
                coolant_outlet_temp_k: 0.0,
            };
            let mut shaft = JetShaftState::running(air_engine);
            shaft.lit = point.lit;
            return Ok((
                EstocPoint {
                    mode: EstocMode::Air,
                    thrust_n: snapshot.thrust_n,
                    fuel_flow_kg_s: snapshot.fuel_flow_kg_s,
                    bulk_fuel_flow_kg_s: snapshot.bulk_fuel_flow_kg_s,
                    boost_fuel_flow_kg_s: snapshot.boost_fuel_flow_kg_s,
                    oxidizer_flow_kg_s: 0.0,
                    air_flow_kg_s: snapshot.air_flow_kg_s,
                    isp_total_s: point.isp_s,
                    exhaust_temp_k: snapshot.exhaust_temp_k,
                    exhaust_velocity_mps: snapshot.exhaust_velocity_mps,
                    exit_pressure_pa: snapshot.exit_pressure_pa,
                    exit_mach: snapshot.exit_mach,
                    compressor_inlet_total_temp_k: snapshot.compressor_inlet_total_temp_k,
                    precooler_heat_flow_w: snapshot.precooler_heat_flow_w,
                    precooler_wall_heat_flow_w: snapshot.precooler_wall_heat_flow_w,
                    precooler_wall_temp_k: snapshot.precooler_wall_temp_k,
                    coolant_outlet_temp_k: snapshot.coolant_outlet_temp_k,
                    precooler_saturated: false,
                },
                snapshot,
                shaft,
            ));
        }
        let shaft_cmd = ShaftCommand {
            throttle,
            starter_engaged: jet.starter_engaged,
            generator_load_w: jet.generator_load_w,
        };
        let (shaft, _telemetry) = match &self.engine {
            CompiledJet::Estoc(estoc) => advance_jet_shaft_loaded_with(
                &estoc.air,
                jet.shaft,
                &shaft_cmd,
                condition,
                jet.dt_s,
                0.0,
                |spool_n, ignition| {
                    estoc
                        .conditioned_air_point(
                            condition,
                            throttle,
                            JetShaftState {
                                spool_n,
                                lit: ignition,
                                starter_charge_j: jet.shaft.starter_charge_j,
                            },
                            jet.prev.as_ref(),
                            jet.dt_s,
                        )
                        .map(|(point, _, _, balance)| (point, balance))
                },
            )?,
            CompiledJet::Air(_) => advance_jet_shaft(
                self.air_engine(),
                jet.shaft,
                &shaft_cmd,
                condition,
                jet.dt_s,
            )?,
        };
        match &self.engine {
            CompiledJet::Air(engine) => {
                let (point, _) = engine.operating_point_at_spool(
                    condition,
                    throttle,
                    shaft.spool_n,
                    shaft.lit,
                )?;
                let snapshot = EstocTransient {
                    thrust_n: point.thrust_n.max(0.0),
                    fuel_flow_kg_s: point.fuel_flow_kg_s,
                    bulk_fuel_flow_kg_s: point.bulk_fuel_flow_kg_s,
                    boost_fuel_flow_kg_s: point.boost_fuel_flow_kg_s,
                    oxidizer_flow_kg_s: 0.0,
                    air_flow_kg_s: point.air_flow_kg_s,
                    exhaust_temp_k: point.exhaust_temp_k,
                    exhaust_velocity_mps: point.exhaust_velocity_mps,
                    exit_pressure_pa: point.exit_pressure_pa,
                    exit_mach: point.exit_mach,
                    compressor_inlet_total_temp_k: point.compressor_inlet_total_temp_k,
                    precooler_heat_flow_w: point.precooler_heat_flow_w,
                    precooler_wall_heat_flow_w: point.precooler_wall_heat_flow_w,
                    precooler_wall_temp_k: 0.0,
                    coolant_outlet_temp_k: 0.0,
                };
                Ok((
                    EstocPoint {
                        mode: EstocMode::Air,
                        thrust_n: snapshot.thrust_n,
                        fuel_flow_kg_s: snapshot.fuel_flow_kg_s,
                        bulk_fuel_flow_kg_s: snapshot.bulk_fuel_flow_kg_s,
                        boost_fuel_flow_kg_s: snapshot.boost_fuel_flow_kg_s,
                        oxidizer_flow_kg_s: 0.0,
                        air_flow_kg_s: snapshot.air_flow_kg_s,
                        isp_total_s: point.isp_s,
                        exhaust_temp_k: snapshot.exhaust_temp_k,
                        exhaust_velocity_mps: snapshot.exhaust_velocity_mps,
                        exit_pressure_pa: snapshot.exit_pressure_pa,
                        exit_mach: snapshot.exit_mach,
                        compressor_inlet_total_temp_k: snapshot.compressor_inlet_total_temp_k,
                        precooler_heat_flow_w: snapshot.precooler_heat_flow_w,
                        precooler_wall_heat_flow_w: snapshot.precooler_wall_heat_flow_w,
                        precooler_wall_temp_k: snapshot.precooler_wall_temp_k,
                        coolant_outlet_temp_k: snapshot.coolant_outlet_temp_k,
                        precooler_saturated: false,
                    },
                    snapshot,
                    shaft,
                ))
            }
            CompiledJet::Estoc(engine) => {
                let (point, transient) = engine.operating_point(
                    condition,
                    throttle,
                    jet.manual,
                    jet.last_mode,
                    jet.prev.as_ref(),
                    jet.dt_s,
                    shaft,
                )?;
                Ok((point, transient, shaft))
            }
        }
    }

    /// Thrust vector in body axes (N).
    pub fn thrust_vector_body_n(
        &self,
        throttle: f64,
        condition: &FlightCondition,
        jet: &JetCommand,
    ) -> Result<[f64; 3], PropulsionError> {
        let thrust_n = self.thrust_n(throttle, condition, jet)?;
        Ok([
            self.thrust_axis_body[0] * thrust_n,
            self.thrust_axis_body[1] * thrust_n,
            self.thrust_axis_body[2] * thrust_n,
        ])
    }

    /// Plume-renderer input for the jet at throttle and condition (exit
    /// radius from the core nozzle area; ESTOC reports the active mode).
    /// Derived from [`JetMount::estoc_point`], so the plume always shows
    /// the same flow/state the thrust came from.
    pub fn plume_state(
        &self,
        throttle: f64,
        condition: &FlightCondition,
        jet: &JetCommand,
    ) -> Result<EnginePlumeState, PropulsionError> {
        self.validate()?;
        let (point, _, _) = self.estoc_point(throttle, condition, jet)?;
        let radius_source: &CompiledAirbreather = self.air_engine();
        Ok(EnginePlumeState {
            exit_radius_m: (radius_source.exit_area_core_m2 / std::f64::consts::PI).sqrt(),
            mass_flow_kg_s: point.fuel_flow_kg_s + point.oxidizer_flow_kg_s + point.air_flow_kg_s,
            exhaust_velocity_mps: point.exhaust_velocity_mps,
            exit_pressure_pa: point.exit_pressure_pa,
            exit_temp_k: point.exhaust_temp_k,
            exit_mach: point.exit_mach,
            propellant: radius_source.fuel.exhaust_propellant(),
        })
    }

    /// Gimbal authority pair for the flight allocator (shared math with
    /// rocket mounts; moments about the body origin).
    pub fn gimbal_authority(
        &self,
        throttle: f64,
        condition: &FlightCondition,
        jet: &JetCommand,
    ) -> Result<[GimbalEffector; 2], PropulsionError> {
        self.validate()?;
        let thrust_n = DVec3::from_array(self.thrust_vector_body_n(throttle, condition, jet)?);
        Ok(gimbal_pair(
            self.position_body_m,
            self.thrust_axis_body,
            thrust_n.to_array(),
            self.gimbal_range_rad,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::super::{
        AirCycle, AirbreathingSpec, ChamberMaterial, IntakeKind, JetFuel, ShaftSpec, StarterKind,
        StarterSpec,
    };
    use super::*;

    fn test_jet() -> JetMount {
        let engine = AirbreathingSpec {
            name: "test jet".into(),
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
            shaft: ShaftSpec::default(),
        }
        .compile()
        .expect("jet compiles");
        JetMount {
            name: "jet-a".into(),
            engine: CompiledJet::Air(Box::new(engine)),
            position_body_m: [-2.0, 0.0, 0.5],
            thrust_axis_body: [1.0, 0.0, 0.0],
            gimbal_range_rad: 0.0,
        }
    }

    fn sl_static() -> FlightCondition {
        FlightCondition {
            mach: 0.0,
            ambient_pa: 101_325.0,
            ambient_temp_k: 288.15,
            airspeed_mps: 0.0,
            composition: crate::atmosphere::AtmosphereComposition::earth_air(),
        }
    }

    #[test]
    fn jet_mount_thrust_and_gimbal() {
        // Mounted jet fires along its axis at static sea level; fixed
        // mount returns zero gimbal authority with well-defined axes.
        let mount = test_jet();
        let thrust = mount
            .thrust_vector_body_n(1.0, &sl_static(), &JetCommand::fresh())
            .expect("thrust");
        assert!(thrust[0] > 10_000.0);
        assert_eq!(thrust[1], 0.0);
        assert_eq!(thrust[2], 0.0);
        let authority = mount
            .gimbal_authority(1.0, &sl_static(), &JetCommand::fresh())
            .expect("authority");
        for effector in authority {
            assert_eq!(
                glam::DVec3::from_array(effector.moment_per_command_nm).length(),
                0.0
            );
        }
        // Off throttle is off; bad axis is refused.
        let off = mount
            .thrust_n(0.0, &sl_static(), &JetCommand::fresh())
            .expect("off");
        assert_eq!(off, 0.0);
        let mut bad = mount;
        bad.thrust_axis_body = [2.0, 0.0, 0.0];
        assert!(bad.validate().is_err());
    }

    #[test]
    fn scramjet_mount_uses_passive_cycle_and_refuses_shaft_accessories() {
        let engine = AirbreathingSpec {
            name: "mounted-scramjet".into(),
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
        }
        .compile()
        .expect("scramjet");
        let mount = JetMount {
            name: "scramjet-mount".into(),
            engine: CompiledJet::Air(Box::new(engine)),
            position_body_m: [0.0; 3],
            thrust_axis_body: [1.0, 0.0, 0.0],
            gimbal_range_rad: 0.0,
        };
        let sample = crate::AtmosphereConfig::default()
            .sample(20_000.0)
            .expect("20 km atmosphere");
        let condition =
            crate::flight_condition(&sample, 6.0 * sample.speed_of_sound_mps).expect("Mach 6");
        let command = JetCommand::fresh();
        let (point, _, shaft) = mount
            .estoc_point(1.0, &condition, &command)
            .expect("passive air path");
        assert!(point.thrust_n > 0.0);
        assert_eq!(shaft.spool_n, 1.0);

        let mut invalid = command;
        invalid.starter_engaged = true;
        assert!(mount.estoc_point(1.0, &condition, &invalid).is_err());
        invalid.starter_engaged = false;
        invalid.generator_load_w = 10.0;
        assert!(mount.estoc_point(1.0, &condition, &invalid).is_err());
    }

    #[test]
    fn cold_start_needs_a_starter_and_crank() {
        // A cold shaft refuses engagement when no starter is fitted;
        // with an electric starter the crank raises spool through
        // light-off, and the engine then self-sustains with the starter
        // released.
        let sl = sl_static();
        let mut cold = JetCommand::cold(&test_jet().engine);
        assert_eq!(cold.shaft.spool_n, 0.0);
        assert!(!cold.shaft.lit);
        cold.starter_engaged = true;
        assert!(
            test_jet().estoc_point(1.0, &sl, &cold).is_err(),
            "starterless engine must refuse engagement"
        );

        let engine = AirbreathingSpec {
            name: "starter jet".into(),
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
                starter: StarterSpec {
                    kind: StarterKind::Electric,
                    power_w: 4.0e6,
                    charge_j: 1.0e9,
                    mass_kg: 30.0,
                },
                ..ShaftSpec::default()
            },
        }
        .compile()
        .expect("starter jet compiles");
        let mount = JetMount {
            name: "starter-jet".into(),
            engine: CompiledJet::Air(Box::new(engine)),
            position_body_m: [-2.0, 0.0, 0.5],
            thrust_axis_body: [1.0, 0.0, 0.0],
            gimbal_range_rad: 0.0,
        };
        let mut command = JetCommand::cold(&mount.engine);
        command.starter_engaged = true;
        command.dt_s = 0.1;
        let mut lit = false;
        let mut spool = 0.0;
        for _ in 0..400 {
            let (_, _, shaft) = mount.estoc_point(1.0, &sl, &command).expect("crank");
            spool = shaft.spool_n;
            lit = shaft.lit;
            command.shaft = shaft;
            if lit {
                break;
            }
        }
        assert!(
            lit,
            "electric starter must light the core (spool {spool:.3})"
        );
        assert!(
            spool >= 0.15,
            "lit engine above light-off, spool {spool:.3}"
        );
        // Self-sustain: release the starter, keep throttle — stays lit
        // and keeps spooling up.
        command.starter_engaged = false;
        for _ in 0..200 {
            let (_, _, shaft) = mount.estoc_point(1.0, &sl, &command).expect("sustain");
            command.shaft = shaft;
        }
        assert!(command.shaft.lit, "self-sustaining after starter release");
        assert!(command.shaft.spool_n >= 0.15);
    }
}
