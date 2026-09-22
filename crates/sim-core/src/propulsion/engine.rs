//! Compiled engine runtime: throttle/ambient evaluation dispatch, thermal
//! power and depletion queries, and the plume-renderer handoff.

use serde::{Deserialize, Serialize};

use super::{
    CompiledLiquid, CompiledSolid, NozzleContour, Propellant, PropulsionError,
    SEPARATION_PRESSURE_RATIO, STANDARD_GRAVITY_MPS2, thrust_coefficient,
};

/// Compiled engine, either family. One representation for simple-mode
/// presets and advanced component authoring (v1 ships the simple path).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum CompiledEngine {
    Liquid(CompiledLiquid),
    Solid(CompiledSolid),
}

/// Instantaneous operating point: the single choke point between the
/// propulsion backend and thrust/mass-flow/plume consumers.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct EngineOperatingPoint {
    pub thrust_n: f64,
    pub mass_flow_kg_s: f64,
    pub isp_s: f64,
    pub exhaust_velocity_mps: f64,
    pub exit_pressure_pa: f64,
    pub exit_temp_k: f64,
    pub exit_mach: f64,
    /// True when Pe/Pa trips the Summerfield separation criterion.
    pub separation_risk: bool,
}

impl CompiledEngine {
    /// Engine display name.
    pub fn name(&self) -> &str {
        match self {
            Self::Liquid(engine) => &engine.name,
            Self::Solid(engine) => &engine.name,
        }
    }

    /// Dry (no propellant) mass in kg.
    pub fn dry_mass_kg(&self) -> f64 {
        match self {
            Self::Liquid(engine) => engine.dry_mass_kg,
            Self::Solid(engine) => engine.dry_mass_kg,
        }
    }

    /// Mass to aggregate into the vehicle budget at bake time: dry mass
    /// plus full solid propellant (grain burns off in flight; depletion
    /// wiring into the runtime mass model is deferred). Liquid propellant
    /// lives in tank parts, which are future vehicle components.
    pub fn bake_mass_kg(&self) -> f64 {
        match self {
            Self::Liquid(engine) => engine.dry_mass_kg,
            Self::Solid(engine) => engine.dry_mass_kg + engine.propellant_mass_kg,
        }
    }

    /// Full-throttle mass flow at the design point (kg/s).
    pub fn full_flow_kg_s(&self) -> f64 {
        match self {
            Self::Liquid(engine) => engine.full_flow_kg_s,
            Self::Solid(engine) => engine
                .burn_curve
                .iter()
                .map(|point| point.mass_flow_kg_s)
                .fold(0.0_f64, f64::max),
        }
    }

    /// Allowed throttle interval (solids are fixed at full by physics).
    pub fn throttle_range(&self) -> (f64, f64) {
        match self {
            Self::Liquid(engine) => (engine.min_throttle, 1.0),
            Self::Solid(_) => (1.0, 1.0),
        }
    }

    /// Spool/valve time constant (s). Solids ignite ballistically: 0.
    pub fn spool_tau_s(&self) -> f64 {
        match self {
            Self::Liquid(engine) => engine.spool_tau_s,
            Self::Solid(_) => 0.0,
        }
    }

    /// Steady-state operating point at effective throttle and ambient
    /// pressure. Liquids scale chamber pressure linearly with throttle
    /// (documented deep-throttle assumption: combustion efficiency held
    /// constant; real variation of a few percent is not modeled). Solids
    /// ignore throttle (must be 1) and read the burn trace at `burn_time_s`.
    pub fn operating_point(
        &self,
        throttle: f64,
        ambient_pa: f64,
        burn_time_s: f64,
    ) -> Result<EngineOperatingPoint, PropulsionError> {
        if !ambient_pa.is_finite() || ambient_pa < 0.0 {
            return Err(PropulsionError::InvalidCommand(
                "ambient pressure must be finite and >= 0".into(),
            ));
        }
        match self {
            Self::Liquid(engine) => {
                if !throttle.is_finite() || throttle < 0.0 || throttle > 1.0 {
                    return Err(PropulsionError::InvalidCommand(
                        "throttle must be finite in [0, 1]".into(),
                    ));
                }
                if throttle == 0.0 {
                    return Ok(EngineOperatingPoint::off(engine));
                }
                if engine.contour == NozzleContour::Aerospike {
                    // Altitude compensation: the free jet boundary tracks
                    // ambient, so the design vacuum thrust survives at any
                    // altitude minus base drag on the plug base.
                    let flow_kg_s = engine.full_flow_kg_s * throttle;
                    let thrust_n = (engine.thrust_vac_n * throttle
                        - ambient_pa * engine.aerospike_base_area_m2)
                        .max(0.0);
                    let isp_s = thrust_n / (flow_kg_s * STANDARD_GRAVITY_MPS2);
                    return Ok(EngineOperatingPoint {
                        thrust_n,
                        mass_flow_kg_s: flow_kg_s,
                        isp_s,
                        exhaust_velocity_mps: engine.exhaust_velocity_mps,
                        exit_pressure_pa: (engine.exit_pressure_pa * throttle).max(ambient_pa),
                        exit_temp_k: engine.exit_temp_k,
                        exit_mach: engine.exit_mach,
                        separation_risk: false,
                    });
                }
                let chamber_pa = engine.chamber_pressure_pa * throttle;
                let exit_pressure_pa = engine.exit_pressure_pa * throttle;
                let flow_kg_s = engine.full_flow_kg_s * throttle;
                // Main chamber (frozen exit Mach; pressure term exact).
                let thermo = engine.thermo_ref();
                let main_cf = thrust_coefficient(
                    &thermo,
                    chamber_pa,
                    &engine.exit_ref(),
                    engine.expansion_ratio,
                    ambient_pa,
                    engine.divergence_factor,
                );
                let mut thrust_n =
                    main_cf * chamber_pa * engine.throat_area_m2 * engine.kinetic_efficiency;
                thrust_n += engine.gg_thrust_at(throttle, ambient_pa);
                let isp_s = thrust_n / (flow_kg_s * STANDARD_GRAVITY_MPS2);
                Ok(EngineOperatingPoint {
                    thrust_n,
                    mass_flow_kg_s: flow_kg_s,
                    isp_s,
                    exhaust_velocity_mps: engine.exhaust_velocity_mps,
                    exit_pressure_pa,
                    exit_temp_k: engine.exit_temp_k,
                    exit_mach: engine.exit_mach,
                    separation_risk: exit_pressure_pa < SEPARATION_PRESSURE_RATIO * ambient_pa,
                })
            }
            Self::Solid(engine) => {
                if throttle != 1.0 {
                    return Err(PropulsionError::InvalidCommand(
                        "solid motors run at full thrust or off; partial throttle is not physical"
                            .into(),
                    ));
                }
                if !burn_time_s.is_finite() || burn_time_s < 0.0 {
                    return Err(PropulsionError::InvalidCommand(
                        "burn time must be finite and >= 0".into(),
                    ));
                }
                if burn_time_s >= engine.burn_time_s {
                    return Ok(EngineOperatingPoint::burned_out(engine));
                }
                let point = engine.interpolate(burn_time_s);
                if engine.contour == NozzleContour::Aerospike {
                    let thrust_n =
                        (point.thrust_vac_n - ambient_pa * engine.aerospike_base_area_m2).max(0.0);
                    return Ok(EngineOperatingPoint {
                        thrust_n,
                        mass_flow_kg_s: point.mass_flow_kg_s,
                        isp_s: thrust_n / (point.mass_flow_kg_s * STANDARD_GRAVITY_MPS2),
                        exhaust_velocity_mps: engine.exhaust_velocity_mps,
                        exit_pressure_pa: ambient_pa,
                        exit_temp_k: engine.exit_temp_k,
                        exit_mach: engine.exit_mach,
                        separation_risk: false,
                    });
                }
                let thermo = engine.thermo_ref();
                let cf = thrust_coefficient(
                    &thermo,
                    point.chamber_pa,
                    &engine.exit_state(),
                    engine.expansion_ratio,
                    ambient_pa,
                    engine.divergence_factor,
                );
                let thrust_n = cf * point.chamber_pa * engine.throat_area_m2;
                let exit_pressure_pa = point.chamber_pa * engine.exit_pressure_ratio();
                Ok(EngineOperatingPoint {
                    thrust_n,
                    mass_flow_kg_s: point.mass_flow_kg_s,
                    isp_s: thrust_n / (point.mass_flow_kg_s * STANDARD_GRAVITY_MPS2),
                    exhaust_velocity_mps: engine.exhaust_velocity_mps,
                    exit_pressure_pa,
                    exit_temp_k: engine.exit_temp_k,
                    exit_mach: engine.exit_mach,
                    separation_risk: exit_pressure_pa < SEPARATION_PRESSURE_RATIO * ambient_pa,
                })
            }
        }
    }

    /// Stagnation thermal power released at full throttle (W): mass flow
    /// times cp times chamber temperature. The future thermal graph
    /// consumes this; the nozzle converts part of it into exhaust kinetic
    /// power (see below). Liquids use design flow; solids the burn average.
    pub fn chamber_power_w(&self) -> f64 {
        match self {
            Self::Liquid(engine) => {
                let cp = engine.gamma * engine.gas_constant_j_kg_k / (engine.gamma - 1.0);
                engine.full_flow_kg_s * cp * engine.chamber_temp_k
            }
            Self::Solid(engine) => {
                let cp = engine.gamma * engine.gas_constant_j_kg_k / (engine.gamma - 1.0);
                let avg_flow_kg_s = engine.propellant_mass_kg / engine.burn_time_s;
                avg_flow_kg_s * cp * engine.chamber_temp_k
            }
        }
    }

    /// Exhaust kinetic power at full throttle in vacuum (W). Always below
    /// [`CompiledEngine::chamber_power_w`] (pinned by test): the difference
    /// is residual exhaust enthalpy plus (for GG cycles) duct losses.
    pub fn exhaust_kinetic_power_w(&self) -> f64 {
        match self {
            Self::Liquid(engine) => {
                0.5 * engine.full_flow_kg_s
                    * engine.exhaust_velocity_mps
                    * engine.exhaust_velocity_mps
            }
            Self::Solid(engine) => {
                let avg_flow_kg_s = engine.propellant_mass_kg / engine.burn_time_s;
                0.5 * avg_flow_kg_s * engine.exhaust_velocity_mps * engine.exhaust_velocity_mps
            }
        }
    }

    /// Solid propellant remaining at a burn clock (kg). Liquids return
    /// `None`: their propellant lives in tank parts (see `feed` module).
    pub fn propellant_remaining_kg(&self, burn_time_s: f64) -> Option<f64> {
        match self {
            Self::Liquid(_) => None,
            Self::Solid(engine) => {
                if !burn_time_s.is_finite() || burn_time_s <= 0.0 {
                    return Some(engine.propellant_mass_kg);
                }
                if burn_time_s >= engine.burn_time_s {
                    return Some(0.0);
                }
                let mut consumed_kg = 0.0;
                let mut prev = &engine.burn_curve[0];
                for point in engine.burn_curve.iter().skip(1) {
                    if point.time_s >= burn_time_s {
                        // Partial interval with linear flow: exact integral.
                        let span = (point.time_s - prev.time_s).max(1e-12);
                        let fraction = ((burn_time_s - prev.time_s) / span).clamp(0.0, 1.0);
                        let flow_now = prev.mass_flow_kg_s
                            + (point.mass_flow_kg_s - prev.mass_flow_kg_s) * fraction;
                        consumed_kg += 0.5
                            * (prev.mass_flow_kg_s + flow_now)
                            * (burn_time_s - prev.time_s).max(0.0);
                        break;
                    }
                    consumed_kg += 0.5
                        * (prev.mass_flow_kg_s + point.mass_flow_kg_s)
                        * (point.time_s - prev.time_s);
                    prev = point;
                }
                Some((engine.propellant_mass_kg - consumed_kg).max(0.0))
            }
        }
    }

    /// Plume-renderer input state for an operating point plus geometry.
    pub fn plume_state(&self, point: &EngineOperatingPoint) -> EnginePlumeState {
        match self {
            Self::Liquid(engine) => EnginePlumeState {
                exit_radius_m: engine.exit_radius_m,
                mass_flow_kg_s: point.mass_flow_kg_s,
                exhaust_velocity_mps: point.exhaust_velocity_mps,
                exit_pressure_pa: point.exit_pressure_pa,
                exit_temp_k: point.exit_temp_k,
                exit_mach: point.exit_mach,
                propellant: engine.propellant,
            },
            Self::Solid(engine) => EnginePlumeState {
                exit_radius_m: engine.exit_radius_m,
                mass_flow_kg_s: point.mass_flow_kg_s,
                exhaust_velocity_mps: point.exhaust_velocity_mps,
                exit_pressure_pa: point.exit_pressure_pa,
                exit_temp_k: point.exit_temp_k,
                exit_mach: point.exit_mach,
                propellant: engine.propellant,
            },
        }
    }
}

impl EngineOperatingPoint {
    fn off(engine: &CompiledLiquid) -> Self {
        Self {
            thrust_n: 0.0,
            mass_flow_kg_s: 0.0,
            isp_s: 0.0,
            exhaust_velocity_mps: engine.exhaust_velocity_mps,
            exit_pressure_pa: 0.0,
            exit_temp_k: engine.exit_temp_k,
            exit_mach: engine.exit_mach,
            separation_risk: false,
        }
    }

    fn burned_out(engine: &CompiledSolid) -> Self {
        Self {
            thrust_n: 0.0,
            mass_flow_kg_s: 0.0,
            isp_s: 0.0,
            exhaust_velocity_mps: engine.exhaust_velocity_mps,
            exit_pressure_pa: 0.0,
            exit_temp_k: engine.exit_temp_k,
            exit_mach: engine.exit_mach,
            separation_risk: false,
        }
    }
}

/// Nozzle/exhaust state for the plume renderer. Field-for-field compatible
/// with `thessa-plume-core` `PlumeSource`; the game-side choke point
/// `PlumeSource::from_engine_state` consumes exactly these values so neither
/// engine crate depends on the other.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct EnginePlumeState {
    pub exit_radius_m: f64,
    pub mass_flow_kg_s: f64,
    pub exhaust_velocity_mps: f64,
    pub exit_pressure_pa: f64,
    pub exit_temp_k: f64,
    pub exit_mach: f64,
    pub propellant: Propellant,
}

#[cfg(test)]
mod tests {
    use super::super::liquid::merlin_like;
    use super::super::{ChamberMaterial, NozzleContour, Propellant, SolidMotorSpec};
    use super::*;

    #[test]
    fn thermal_power_ordering_and_depletion() {
        // Energy ordering: kinetic exhaust power stays below released
        // chamber power (the gap is residual enthalpy + duct losses).
        // Depletion: remaining grain falls monotonically to exactly zero.
        let liquid = CompiledEngine::Liquid(merlin_like().compile().expect("compile"));
        assert!(liquid.exhaust_kinetic_power_w() > 0.0);
        assert!(liquid.exhaust_kinetic_power_w() < liquid.chamber_power_w());
        assert_eq!(liquid.propellant_remaining_kg(10.0), None);

        let (a, n) = SolidMotorSpec::apcp_ballistics();
        let solid = CompiledEngine::Solid(
            SolidMotorSpec {
                name: "depletion probe".into(),
                propellant: Propellant::SolidApcp,
                outer_radius_m: 0.5,
                core_radius_m: 0.32,
                segment_length_m: 1.5,
                segments: 2,
                burn_rate_coeff: a,
                burn_rate_exponent: n,
                throat_radius_m: 0.12,
                expansion_ratio: 8.0,
                nozzle_length_m: 0.8,
                contour: NozzleContour::Conical,
                casing_material: ChamberMaterial::nickel_superalloy(),
                inhibited_ends: true,
                segment_core_radii_m: None,
                gimbal_range_rad: 0.0,
                ignition_shots: 1,
            }
            .compile()
            .expect("solid"),
        );
        assert!(solid.exhaust_kinetic_power_w() < solid.chamber_power_w());
        let burn_time = match &solid {
            CompiledEngine::Solid(motor) => motor.burn_time_s,
            _ => 0.0,
        };
        let total = solid.propellant_remaining_kg(0.0).expect("grain");
        assert!(total > 0.0);
        let mut last = total;
        for step in 1..=10 {
            let remaining = solid
                .propellant_remaining_kg(burn_time * step as f64 / 10.0)
                .expect("grain");
            assert!(remaining <= last + total * 1e-9);
            last = remaining;
        }
        assert_eq!(solid.propellant_remaining_kg(burn_time), Some(0.0));
        assert_eq!(solid.propellant_remaining_kg(burn_time + 100.0), Some(0.0));
    }
}
