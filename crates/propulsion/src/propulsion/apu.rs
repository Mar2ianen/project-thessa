//! Auxiliary gas-turbine generator built on the shared airbreather and shaft
//! models. Fuel flow, starter energy, generator load, and spool response are
//! inherited from those physical subsystems rather than assigned a fixed APU
//! fuel coefficient.

use serde::{Deserialize, Serialize};

use super::{
    AirOperatingPoint, AirbreathingSpec, CompiledAirbreather, FlightCondition, JetFuel,
    JetShaftState, PropulsionError, ShaftCommand, ShaftTelemetry, advance_jet_shaft_loaded,
};

/// Authored APU gas turbine. Its airbreather must carry a fitted shaft
/// generator; start hardware remains the normal `ShaftSpec::starter` choice.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AuxiliaryPowerUnitSpec {
    pub name: String,
    pub engine: AirbreathingSpec,
}

/// Compiled gas-turbine APU reusing the standard compressor, combustor,
/// turbine, shaft, starter, and generator operating models.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CompiledAuxiliaryPowerUnit {
    pub name: String,
    pub engine: CompiledAirbreather,
    pub dry_mass_kg: f64,
}

/// One installed APU, including its body-frame mount and named tank-feed
/// endpoint. The electrical output joins the vehicle-wide ideal bus.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AuxiliaryPowerUnitMount {
    pub name: String,
    pub unit: CompiledAuxiliaryPowerUnit,
    pub position_body_m: [f64; 3],
    /// Unit direction of the exhaust thrust in body axes.
    #[serde(default = "default_thrust_axis_body")]
    pub thrust_axis_body: [f64; 3],
    /// Optional named assembly engine/feed port for tank reachability.
    #[serde(default)]
    pub feed_port_name: Option<String>,
}

impl AuxiliaryPowerUnitMount {
    pub fn validate(&self) -> Result<(), PropulsionError> {
        if self.name.trim().is_empty() {
            return Err(PropulsionError::InvalidSpec(
                "APU mount needs a name".into(),
            ));
        }
        if self.position_body_m.iter().any(|value| !value.is_finite()) {
            return Err(PropulsionError::InvalidSpec(
                "APU mount position must be finite".into(),
            ));
        }
        if self.thrust_axis_body.iter().any(|value| !value.is_finite()) {
            return Err(PropulsionError::InvalidSpec(
                "APU thrust axis must be finite".into(),
            ));
        }
        let axis_norm = self
            .thrust_axis_body
            .iter()
            .map(|value| value * value)
            .sum::<f64>()
            .sqrt();
        if (axis_norm - 1.0).abs() > 1.0e-9 {
            return Err(PropulsionError::InvalidSpec(
                "APU thrust axis must be unit length".into(),
            ));
        }
        if self
            .feed_port_name
            .as_ref()
            .is_some_and(|port| port.trim().is_empty())
        {
            return Err(PropulsionError::InvalidSpec(
                "APU feed-port name must not be empty".into(),
            ));
        }
        Ok(())
    }
}

fn default_thrust_axis_body() -> [f64; 3] {
    [1.0, 0.0, 0.0]
}

/// Persistent shaft state for one APU.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct AuxiliaryPowerUnitState {
    pub shaft: JetShaftState,
}

/// One fixed-step APU command. Requested generator load is clipped by the
/// fitted generator and acts as a real shaft load.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct AuxiliaryPowerUnitCommand {
    pub throttle: f64,
    pub starter_engaged: bool,
    pub generator_load_w: f64,
    /// Requested compressor/bleed take-off on the turbine shaft (W).
    pub pneumatic_bleed_power_w: f64,
    pub dt_s: f64,
}

/// APU output from one operating-point update.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AuxiliaryPowerUnitOperatingPoint {
    pub shaft_state: JetShaftState,
    pub shaft: ShaftTelemetry,
    pub air: AirOperatingPoint,
    pub electrical_power_w: f64,
    /// Pneumatic compressor shaft power reserved for engine starting (W).
    pub pneumatic_bleed_power_w: f64,
    pub fuel: JetFuel,
    pub fuel_flow_kg_s: f64,
}

impl AuxiliaryPowerUnitSpec {
    pub fn compile(self) -> Result<CompiledAuxiliaryPowerUnit, PropulsionError> {
        if self.name.trim().is_empty() {
            return Err(PropulsionError::InvalidSpec(
                "APU name must not be empty".into(),
            ));
        }
        if !self.engine.cycle.has_shaft() {
            return Err(PropulsionError::UnsupportedCombination(
                "an APU needs a compressor/turbine shaft; ramjet and scramjet cycles are unsupported".into(),
            ));
        }
        if !self.engine.shaft.generator.fitted || self.engine.shaft.generator.power_w <= 0.0 {
            return Err(PropulsionError::InvalidSpec(
                "an APU needs a fitted, positive-rated shaft generator".into(),
            ));
        }
        let engine = self.engine.compile()?;
        Ok(CompiledAuxiliaryPowerUnit {
            name: self.name,
            dry_mass_kg: engine.dry_mass_kg,
            engine,
        })
    }
}

impl CompiledAuxiliaryPowerUnit {
    pub fn initial_state(&self) -> AuxiliaryPowerUnitState {
        AuxiliaryPowerUnitState {
            shaft: JetShaftState::cold(&self.engine),
        }
    }

    pub fn advance(
        &self,
        state: AuxiliaryPowerUnitState,
        command: AuxiliaryPowerUnitCommand,
        condition: &FlightCondition,
    ) -> Result<(AuxiliaryPowerUnitState, AuxiliaryPowerUnitOperatingPoint), PropulsionError> {
        if !command.throttle.is_finite() || !(0.0..=1.0).contains(&command.throttle) {
            return Err(PropulsionError::InvalidCommand(
                "APU throttle must be finite and in [0, 1]".into(),
            ));
        }
        if !command.generator_load_w.is_finite() || command.generator_load_w < 0.0 {
            return Err(PropulsionError::InvalidCommand(
                "APU generator load must be finite and non-negative".into(),
            ));
        }
        let shaft_command = ShaftCommand {
            throttle: command.throttle,
            starter_engaged: command.starter_engaged,
            generator_load_w: command.generator_load_w,
        };
        if !command.pneumatic_bleed_power_w.is_finite() || command.pneumatic_bleed_power_w < 0.0 {
            return Err(PropulsionError::InvalidCommand(
                "APU pneumatic bleed load must be finite and non-negative".into(),
            ));
        }
        let (shaft_state, shaft) = advance_jet_shaft_loaded(
            &self.engine,
            state.shaft,
            &shaft_command,
            condition,
            command.dt_s,
            command.pneumatic_bleed_power_w,
        )?;
        let (air, _) = self.engine.operating_point_at_spool_loaded(
            condition,
            command.throttle,
            shaft_state.spool_n,
            shaft_state.lit,
            command.pneumatic_bleed_power_w,
        )?;
        let point = AuxiliaryPowerUnitOperatingPoint {
            shaft_state,
            electrical_power_w: shaft.generator_electrical_w,
            pneumatic_bleed_power_w: if shaft.lit {
                command.pneumatic_bleed_power_w
            } else {
                0.0
            },
            fuel: self.engine.fuel,
            fuel_flow_kg_s: air.fuel_flow_kg_s,
            shaft,
            air,
        };
        Ok((AuxiliaryPowerUnitState { shaft: shaft_state }, point))
    }
}

#[cfg(test)]
mod tests {
    use super::super::ShaftSpool;
    use super::*;
    use crate::propulsion::{
        AirCycle, ChamberMaterial, GeneratorSpec, IntakeKind, ShaftSpec, StarterKind, StarterSpec,
    };
    use crate::{AtmosphereConfig, flight_condition};

    fn spec(generator: GeneratorSpec) -> AuxiliaryPowerUnitSpec {
        AuxiliaryPowerUnitSpec {
            name: "service-apu".into(),
            engine: AirbreathingSpec {
                name: "service-apu-gas-generator".into(),
                cycle: AirCycle::Turbojet,
                fuel: JetFuel::Kerosene,
                intake_area_m2: 0.25,
                intake: IntakeKind::Pitot,
                compressor_ratio: 8.0,
                bypass_ratio: 0.0,
                fan_pressure_ratio: 1.0,
                turbine_inlet_temp_k: 1_350.0,
                afterburner: false,
                reheat_temp_k: 0.0,
                turbine_material: ChamberMaterial::nickel_superalloy(),
                spool_tau_s: 3.0,
                shaft: ShaftSpec {
                    starter: StarterSpec {
                        kind: StarterKind::Electric,
                        power_w: 20_000.0,
                        charge_j: 1.0e6,
                        resource: None,
                        attached_spool: ShaftSpool::HighPressure,
                        specific_energy_j_kg: 0.0,
                        maximum_shaft_torque_nm: None,
                        mass_kg: 3.0,
                    },
                    generator,
                    power_turbine_heat_fraction: 0.2,
                    ..ShaftSpec::default()
                },
            },
        }
    }

    #[test]
    fn apu_uses_the_shared_gas_turbine_shaft_and_reports_real_fuel_flow() {
        let apu = spec(GeneratorSpec {
            fitted: true,
            power_w: 20_000.0,
            efficiency: 0.9,
            efficiency_map: Vec::new(),
            thermal: None,
            attached_spool: ShaftSpool::HighPressure,
            maximum_shaft_torque_nm: None,
            cut_in_spool_n: 0.5,
            mass_kg: 4.0,
        })
        .compile()
        .expect("APU with fitted generator");
        let sample = AtmosphereConfig::default().sample(0.0).expect("sea level");
        let condition = flight_condition(&sample, 0.0).expect("static air condition");
        let state = AuxiliaryPowerUnitState {
            shaft: JetShaftState::running(&apu.engine),
        };
        let (next, point) = apu
            .advance(
                state,
                AuxiliaryPowerUnitCommand {
                    throttle: 1.0,
                    starter_engaged: false,
                    generator_load_w: 25_000.0,
                    pneumatic_bleed_power_w: 1_000.0,
                    dt_s: 0.1,
                },
                &condition,
            )
            .expect("loaded APU operating point");
        assert_eq!(point.electrical_power_w, 20_000.0);
        assert_eq!(point.shaft.generator_electrical_w, point.electrical_power_w);
        assert_eq!(point.pneumatic_bleed_power_w, 1_000.0);
        assert!(point.fuel_flow_kg_s > 0.0);
        assert_eq!(point.fuel_flow_kg_s, point.air.fuel_flow_kg_s);
        assert!(next.shaft.spool_n <= 1.0);
        assert!(apu.dry_mass_kg >= apu.engine.dry_mass_kg);
    }

    #[test]
    fn apu_refuses_a_missing_generator_or_shaftless_cycle() {
        assert!(spec(GeneratorSpec::default()).compile().is_err());
        let mut shaftless = spec(GeneratorSpec {
            fitted: true,
            power_w: 1_000.0,
            efficiency: 0.9,
            efficiency_map: Vec::new(),
            thermal: None,
            attached_spool: ShaftSpool::HighPressure,
            maximum_shaft_torque_nm: None,
            cut_in_spool_n: 0.5,
            mass_kg: 1.0,
        });
        shaftless.engine.cycle = AirCycle::Ramjet;
        assert!(shaftless.compile().is_err());
    }
}
