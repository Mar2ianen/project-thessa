//! Jet mounts: air-breathers and ESTOCs installed on a vehicle with body
//! stations, thrust axes, and gimbal ranges. Runtime evaluation is a pure
//! function of throttle, flight condition, and (for ESTOC) threaded mode
//! state — the vehicle stores no spool/mode memory.

use glam::DVec3;
use serde::{Deserialize, Serialize};

use super::GimbalEffector;
use super::mount::gimbal_pair;
use super::{
    CompiledAirbreather, CompiledEstoc, EnginePlumeState, EstocMode, EstocPoint, EstocTransient,
    FlightCondition, PropulsionError,
};

/// One installed jet of either family.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum CompiledJet {
    Air(CompiledAirbreather),
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

/// ESTOC command threading for stateless vehicle calls: manual override,
/// last mode, previous transient, and timestep. A fresh command has no
/// previous transient, so it evaluates the target directly.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct EstocCommand {
    pub manual: Option<EstocMode>,
    pub last_mode: EstocMode,
    /// Previous transition snapshot; `None` evaluates the fresh target.
    pub prev: Option<EstocTransient>,
    pub dt_s: f64,
}

impl EstocCommand {
    /// Fresh-start evaluation (direct target, no smoothing).
    pub fn fresh() -> Self {
        Self {
            manual: None,
            last_mode: EstocMode::Air,
            prev: None,
            dt_s: 0.0,
        }
    }

    /// Return the command to use on the next world tick after an ESTOC
    /// operating-point evaluation. Manual selection and timestep are caller
    /// inputs; mode and the full transient snapshot are runtime state.
    pub fn with_state(&self, point: &EstocPoint, transient: EstocTransient) -> Self {
        Self {
            last_mode: point.mode,
            prev: Some(transient),
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

    /// Thrust magnitude (N) at throttle, condition, and ESTOC command.
    pub fn thrust_n(
        &self,
        throttle: f64,
        condition: &FlightCondition,
        estoc: &EstocCommand,
    ) -> Result<f64, PropulsionError> {
        Ok(self.estoc_point(throttle, condition, estoc)?.0.thrust_n)
    }

    /// Full ESTOC point plus the updated transition snapshot (stateful
    /// callers thread both forward; `thrust_n`/`plume_state` discard the
    /// snapshot for one-shot calls).
    pub fn estoc_point(
        &self,
        throttle: f64,
        condition: &FlightCondition,
        estoc: &EstocCommand,
    ) -> Result<(EstocPoint, EstocTransient), PropulsionError> {
        self.validate()?;
        match &self.engine {
            CompiledJet::Air(engine) => {
                let point = engine.operating_point(condition, throttle)?;
                let snapshot = EstocTransient {
                    thrust_n: point.thrust_n.max(0.0),
                    fuel_flow_kg_s: point.fuel_flow_kg_s,
                    oxidizer_flow_kg_s: 0.0,
                    air_flow_kg_s: point.air_flow_kg_s,
                    exhaust_temp_k: point.exhaust_temp_k,
                    exhaust_velocity_mps: point.exhaust_velocity_mps,
                    exit_pressure_pa: point.exit_pressure_pa,
                    exit_mach: point.exit_mach,
                };
                Ok((
                    EstocPoint {
                        mode: EstocMode::Air,
                        thrust_n: snapshot.thrust_n,
                        fuel_flow_kg_s: snapshot.fuel_flow_kg_s,
                        oxidizer_flow_kg_s: 0.0,
                        air_flow_kg_s: snapshot.air_flow_kg_s,
                        isp_total_s: point.isp_s,
                        exhaust_temp_k: snapshot.exhaust_temp_k,
                        exhaust_velocity_mps: snapshot.exhaust_velocity_mps,
                        exit_pressure_pa: snapshot.exit_pressure_pa,
                        exit_mach: snapshot.exit_mach,
                    },
                    snapshot,
                ))
            }
            CompiledJet::Estoc(engine) => engine.operating_point(
                condition,
                throttle,
                estoc.manual,
                estoc.last_mode,
                estoc.prev.as_ref(),
                estoc.dt_s,
            ),
        }
    }

    /// Thrust vector in body axes (N).
    pub fn thrust_vector_body_n(
        &self,
        throttle: f64,
        condition: &FlightCondition,
        estoc: &EstocCommand,
    ) -> Result<[f64; 3], PropulsionError> {
        let thrust_n = self.thrust_n(throttle, condition, estoc)?;
        Ok([
            self.thrust_axis_body[0] * thrust_n,
            self.thrust_axis_body[1] * thrust_n,
            self.thrust_axis_body[2] * thrust_n,
        ])
    }

    /// Plume-renderer input for the jet at throttle and condition (exit
    /// radius from the core nozzle area; ESTOC reports the active mode).
    pub fn plume_state(
        &self,
        throttle: f64,
        condition: &FlightCondition,
        estoc: &EstocCommand,
    ) -> Result<EnginePlumeState, PropulsionError> {
        self.validate()?;
        match &self.engine {
            CompiledJet::Air(engine) => {
                let point = engine.operating_point(condition, throttle)?;
                Ok(EnginePlumeState {
                    exit_radius_m: (engine.exit_area_core_m2 / std::f64::consts::PI).sqrt(),
                    mass_flow_kg_s: point.air_flow_kg_s + point.fuel_flow_kg_s,
                    exhaust_velocity_mps: point.exhaust_velocity_mps,
                    exit_pressure_pa: point.exit_pressure_pa,
                    exit_temp_k: point.exhaust_temp_k,
                    exit_mach: point.exit_mach,
                    propellant: engine.fuel.exhaust_propellant(),
                })
            }
            CompiledJet::Estoc(engine) => {
                let (point, _) = engine.operating_point(
                    condition,
                    throttle,
                    estoc.manual,
                    estoc.last_mode,
                    estoc.prev.as_ref(),
                    estoc.dt_s,
                )?;
                Ok(EnginePlumeState {
                    exit_radius_m: (engine.air.exit_area_core_m2 / std::f64::consts::PI).sqrt(),
                    mass_flow_kg_s: point.fuel_flow_kg_s
                        + point.oxidizer_flow_kg_s
                        + point.air_flow_kg_s,
                    exhaust_velocity_mps: point.exhaust_velocity_mps,
                    exit_pressure_pa: point.exit_pressure_pa,
                    exit_temp_k: point.exhaust_temp_k,
                    exit_mach: point.exit_mach,
                    propellant: engine.air.fuel.exhaust_propellant(),
                })
            }
        }
    }

    /// Gimbal authority pair for the flight allocator (shared math with
    /// rocket mounts; moments about the body origin).
    pub fn gimbal_authority(
        &self,
        throttle: f64,
        condition: &FlightCondition,
        estoc: &EstocCommand,
    ) -> Result<[GimbalEffector; 2], PropulsionError> {
        self.validate()?;
        let thrust_n = DVec3::from_array(self.thrust_vector_body_n(throttle, condition, estoc)?);
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
    use super::super::{AirCycle, AirbreathingSpec, ChamberMaterial, IntakeKind, JetFuel};
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
        }
        .compile()
        .expect("jet compiles");
        JetMount {
            name: "jet-a".into(),
            engine: CompiledJet::Air(engine),
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
            oxygen_fraction: 0.232,
        }
    }

    #[test]
    fn jet_mount_thrust_and_gimbal() {
        // Mounted jet fires along its axis at static sea level; fixed
        // mount returns zero gimbal authority with well-defined axes.
        let mount = test_jet();
        let thrust = mount
            .thrust_vector_body_n(1.0, &sl_static(), &EstocCommand::fresh())
            .expect("thrust");
        assert!(thrust[0] > 10_000.0);
        assert_eq!(thrust[1], 0.0);
        assert_eq!(thrust[2], 0.0);
        let authority = mount
            .gimbal_authority(1.0, &sl_static(), &EstocCommand::fresh())
            .expect("authority");
        for effector in authority {
            assert_eq!(
                glam::DVec3::from_array(effector.moment_per_command_nm).length(),
                0.0
            );
        }
        // Off throttle is off; bad axis is refused.
        let off = mount
            .thrust_n(0.0, &sl_static(), &EstocCommand::fresh())
            .expect("off");
        assert_eq!(off, 0.0);
        let mut bad = mount;
        bad.thrust_axis_body = [2.0, 0.0, 0.0];
        assert!(bad.validate().is_err());
    }
}
