//! Compiled engine runtime: throttle/ambient evaluation dispatch, thermal
//! power and depletion queries, and the plume-renderer handoff.

use serde::{Deserialize, Serialize};

use super::{
    CompiledLiquid, CompiledSolid, NozzleContour, Propellant, PropulsionError,
    SEPARATION_PRESSURE_RATIO, STANDARD_GRAVITY_MPS2, require_non_negative, require_positive,
    require_unit_interval, thrust_coefficient,
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

    /// Fail-closed validation of a compiled engine: tampered or
    /// hand-edited baked data (zero divisors, out-of-range nozzle ratios,
    /// an empty or truncated burn trace) must be rejected before the
    /// operating-point formulas can turn it into NaN thrust. Mirrors the
    /// guarantees `compile()` establishes on the spec path; `EngineMount`
    /// runs this before every dispatch.
    pub fn validate(&self) -> Result<(), PropulsionError> {
        match self {
            Self::Liquid(engine) => {
                // Divisors: chamber pressure in the exit-pressure ratio
                // and thrust coefficient, full flow in Isp, gas constant
                // and (gamma - 1) in the thermal power, spool tau in the
                // valve advance.
                require_positive(engine.chamber_pressure_pa, "engine chamber pressure")?;
                require_positive(engine.throat_area_m2, "engine throat area")?;
                require_positive(engine.full_flow_kg_s, "engine full-throttle flow")?;
                require_positive(engine.gas_constant_j_kg_k, "engine gas constant")?;
                require_positive(engine.chamber_temp_k, "engine chamber temperature")?;
                require_positive(engine.spool_tau_s, "engine spool time constant")?;
                if !engine.gamma.is_finite() || engine.gamma <= 1.0 {
                    return Err(PropulsionError::InvalidSpec(
                        "engine gamma must be finite and > 1".into(),
                    ));
                }
                if !engine.expansion_ratio.is_finite() || engine.expansion_ratio < 1.0 {
                    return Err(PropulsionError::InvalidSpec(
                        "engine expansion ratio must be finite and >= 1".into(),
                    ));
                }
                // The frozen exit-pressure ratio is the powf base inside
                // the thrust coefficient's momentum sqrt: a non-positive
                // base goes NaN, a ratio above 1 takes sqrt of a negative.
                if !engine.exit_pressure_pa.is_finite()
                    || engine.exit_pressure_pa <= 0.0
                    || engine.exit_pressure_pa > engine.chamber_pressure_pa
                {
                    return Err(PropulsionError::InvalidSpec(
                        "engine exit pressure must be finite in (0, chamber pressure]".into(),
                    ));
                }
                if !engine.divergence_factor.is_finite()
                    || engine.divergence_factor <= 0.0
                    || engine.divergence_factor > 1.0
                {
                    return Err(PropulsionError::InvalidSpec(
                        "engine divergence factor must be finite in (0, 1]".into(),
                    ));
                }
                if !engine.kinetic_efficiency.is_finite()
                    || engine.kinetic_efficiency <= 0.0
                    || engine.kinetic_efficiency > 1.0
                {
                    return Err(PropulsionError::InvalidSpec(
                        "engine kinetic efficiency must be finite in (0, 1]".into(),
                    ));
                }
                require_unit_interval(engine.min_throttle, "engine minimum throttle")?;
                require_unit_interval(engine.gg_bypass_fraction, "engine gg bypass fraction")?;
                require_non_negative(engine.exhaust_velocity_mps, "engine exhaust velocity")?;
                require_non_negative(engine.exit_mach, "engine exit mach")?;
                require_non_negative(engine.dry_mass_kg, "engine dry mass")?;
                require_non_negative(engine.gimbal_range_rad, "engine gimbal range")?;
                // Remaining scalars that reach an operating point, the
                // plume handoff, or the vehicle mass budget.
                if [
                    engine.exit_temp_k,
                    engine.c_star_mps,
                    engine.thrust_sl_n,
                    engine.thrust_vac_n,
                    engine.isp_sl_s,
                    engine.isp_vac_s,
                    engine.gg_thrust_sl_n,
                    engine.gg_thrust_vac_n,
                    engine.gg_exit_area_m2,
                    engine.gg_isp_s,
                    engine.aerospike_base_area_m2,
                    engine.nozzle_wall_area_m2,
                    engine.pump_power_w,
                    engine.feed_pressure_required_pa,
                    engine.exit_radius_m,
                    engine.nozzle_length_m,
                ]
                .iter()
                .any(|value| !value.is_finite())
                {
                    return Err(PropulsionError::InvalidSpec(
                        "compiled liquid engine values must be finite".into(),
                    ));
                }
                Ok(())
            }
            Self::Solid(engine) => {
                require_positive(engine.throat_area_m2, "engine throat area")?;
                require_positive(engine.gas_constant_j_kg_k, "engine gas constant")?;
                require_positive(engine.chamber_temp_k, "engine chamber temperature")?;
                // The burn clock divides propellant into average flow in
                // the thermal power and gates the burned-out branch.
                require_positive(engine.burn_time_s, "engine burn time")?;
                require_non_negative(engine.propellant_mass_kg, "engine propellant mass")?;
                require_non_negative(engine.exhaust_velocity_mps, "engine exhaust velocity")?;
                require_non_negative(engine.exit_mach, "engine exit mach")?;
                require_non_negative(engine.dry_mass_kg, "engine dry mass")?;
                require_non_negative(engine.gimbal_range_rad, "engine gimbal range")?;
                if !engine.gamma.is_finite() || engine.gamma <= 1.0 {
                    return Err(PropulsionError::InvalidSpec(
                        "engine gamma must be finite and > 1".into(),
                    ));
                }
                if !engine.expansion_ratio.is_finite() || engine.expansion_ratio < 1.0 {
                    return Err(PropulsionError::InvalidSpec(
                        "engine expansion ratio must be finite and >= 1".into(),
                    ));
                }
                if !engine.divergence_factor.is_finite()
                    || engine.divergence_factor <= 0.0
                    || engine.divergence_factor > 1.0
                {
                    return Err(PropulsionError::InvalidSpec(
                        "engine divergence factor must be finite in (0, 1]".into(),
                    ));
                }
                if [
                    engine.total_impulse_ns,
                    engine.avg_isp_s,
                    engine.peak_pressure_pa,
                    engine.peak_thrust_sl_n,
                    engine.c_star_mps,
                    engine.exit_temp_k,
                    engine.aerospike_base_area_m2,
                    engine.throat_radius_m,
                    engine.exit_radius_m,
                    engine.grain_outer_radius_m,
                    engine.grain_length_m,
                ]
                .iter()
                .any(|value| !value.is_finite())
                {
                    return Err(PropulsionError::InvalidSpec(
                        "compiled solid engine values must be finite".into(),
                    ));
                }
                // Burn trace: `interpolate` output divides by chamber
                // pressure (thrust coefficient) and mass flow (Isp), so
                // every sample a query can land on must be burning. Only
                // the terminal burn-through sample may read zero, and it
                // must sit at or past the burn clock where the
                // burned-out branch takes over instead.
                let curve = &engine.burn_curve;
                let Some(first) = curve.first() else {
                    return Err(PropulsionError::InvalidSpec(
                        "solid engine burn curve must not be empty".into(),
                    ));
                };
                if first.chamber_pa <= 0.0 || first.mass_flow_kg_s <= 0.0 {
                    return Err(PropulsionError::InvalidSpec(
                        "solid engine must be burning at ignition".into(),
                    ));
                }
                for (index, point) in curve.iter().enumerate() {
                    if [
                        point.time_s,
                        point.web_burned_m,
                        point.burn_surface_area_m2,
                        point.port_area_m2,
                        point.port_perimeter_m,
                        point.chamber_pa,
                        point.mass_flow_kg_s,
                        point.thrust_sl_n,
                        point.thrust_vac_n,
                    ]
                    .iter()
                    .any(|value| !value.is_finite())
                    {
                        return Err(PropulsionError::InvalidSpec(
                            "solid burn curve points must be finite".into(),
                        ));
                    }
                    if point.time_s < 0.0 || point.chamber_pa < 0.0 || point.mass_flow_kg_s < 0.0 {
                        return Err(PropulsionError::InvalidSpec(
                            "solid burn curve points must be non-negative".into(),
                        ));
                    }
                    if index > 0 && point.time_s < curve[index - 1].time_s {
                        return Err(PropulsionError::InvalidSpec(
                            "solid burn curve time must be non-decreasing".into(),
                        ));
                    }
                    if index + 1 < curve.len()
                        && (point.chamber_pa <= 0.0 || point.mass_flow_kg_s <= 0.0)
                    {
                        return Err(PropulsionError::InvalidSpec(
                            "solid burn curve must stay lit between ignition and burn-through"
                                .into(),
                        ));
                    }
                }
                let last = curve.last().expect("non-empty burn curve");
                if (last.chamber_pa <= 0.0 || last.mass_flow_kg_s <= 0.0)
                    && last.time_s < engine.burn_time_s
                {
                    return Err(PropulsionError::InvalidSpec(
                        "solid burn-through sample must reach the burn clock".into(),
                    ));
                }
                Ok(())
            }
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
                // A motor with no burn duration releases no time-averaged
                // power; never divide by zero (NaN fails closed to 0 too).
                let avg_flow_kg_s = if engine.burn_time_s > 0.0 {
                    engine.propellant_mass_kg / engine.burn_time_s
                } else {
                    0.0
                };
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
                // Same guard as `chamber_power_w`: zero-duration motors
                // give zero average flow, never an infinite power.
                let avg_flow_kg_s = if engine.burn_time_s > 0.0 {
                    engine.propellant_mass_kg / engine.burn_time_s
                } else {
                    0.0
                };
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
                let Some(first_point) = engine.burn_curve.first() else {
                    // An empty curve integrates to zero flow (same fallback
                    // as `interpolate`): nothing is consumed yet.
                    return Some(engine.propellant_mass_kg);
                };
                let mut consumed_kg = 0.0;
                let mut prev = first_point;
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
    use super::super::{
        ChamberMaterial, NozzleContour, Propellant, SolidGrainGeometry, SolidMotorSpec,
    };
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
                grain_geometry: SolidGrainGeometry::Circular,
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

    /// A clean bake must validate; each tampered divisor/range that the
    /// operating-point formulas would turn into NaN thrust must not.
    #[test]
    fn compiled_liquid_validate_rejects_tampered_fields() {
        let clean = CompiledEngine::Liquid(merlin_like().compile().expect("compile"));
        clean.validate().expect("clean bake validates");

        let CompiledEngine::Liquid(mut tampered) = clean.clone() else {
            panic!("liquid variant");
        };
        let original_chamber_pa = tampered.chamber_pressure_pa;
        // Zero chamber pressure feeds the exit-pressure ratio (division)
        // and the thrust coefficient.
        tampered.chamber_pressure_pa = 0.0;
        assert!(matches!(
            CompiledEngine::Liquid(tampered.clone()).validate(),
            Err(PropulsionError::InvalidSpec(_))
        ));
        // NaN instead of a zero: `require_*` helpers reject both.
        tampered.chamber_pressure_pa = f64::NAN;
        assert!(matches!(
            CompiledEngine::Liquid(tampered.clone()).validate(),
            Err(PropulsionError::InvalidSpec(_))
        ));
        tampered.chamber_pressure_pa = original_chamber_pa;
        // Full flow divides thrust into Isp.
        tampered.full_flow_kg_s = 0.0;
        assert!(matches!(
            CompiledEngine::Liquid(tampered.clone()).validate(),
            Err(PropulsionError::InvalidSpec(_))
        ));
        tampered.full_flow_kg_s = 100.0;
        // gamma - 1 divides in the thrust coefficient and thermal power.
        tampered.gamma = 1.0;
        assert!(matches!(
            CompiledEngine::Liquid(tampered.clone()).validate(),
            Err(PropulsionError::InvalidSpec(_))
        ));
        tampered.gamma = 1.4;
        // Exit above chamber inverts the momentum sqrt term.
        tampered.exit_pressure_pa = tampered.chamber_pressure_pa * 1.5;
        assert!(matches!(
            CompiledEngine::Liquid(tampered.clone()).validate(),
            Err(PropulsionError::InvalidSpec(_))
        ));
        tampered.exit_pressure_pa = tampered.chamber_pressure_pa * 0.1;
        // Unit-interval fields gate throttling and GG thrust.
        tampered.min_throttle = 1.5;
        assert!(matches!(
            CompiledEngine::Liquid(tampered.clone()).validate(),
            Err(PropulsionError::InvalidSpec(_))
        ));
        tampered.min_throttle = 0.5;
        tampered.gg_bypass_fraction = f64::NAN;
        assert!(matches!(
            CompiledEngine::Liquid(tampered).validate(),
            Err(PropulsionError::InvalidSpec(_))
        ));
    }

    #[test]
    fn compiled_solid_validate_rejects_tampered_trace() {
        let clean = CompiledEngine::Solid(probe_solid());
        clean.validate().expect("clean bake validates");

        let CompiledEngine::Solid(mut tampered) = clean.clone() else {
            panic!("solid variant");
        };
        let original_burn_time_s = tampered.burn_time_s;
        let original_curve = tampered.burn_curve.clone();
        // The burn clock divides propellant into average flow.
        tampered.burn_time_s = 0.0;
        assert!(matches!(
            CompiledEngine::Solid(tampered.clone()).validate(),
            Err(PropulsionError::InvalidSpec(_))
        ));
        tampered.burn_time_s = original_burn_time_s;
        // An empty trace makes `interpolate` return zeros: chamber
        // pressure zero divides in the thrust coefficient.
        tampered.burn_curve.clear();
        assert!(matches!(
            CompiledEngine::Solid(tampered.clone()).validate(),
            Err(PropulsionError::InvalidSpec(_))
        ));
        tampered.burn_curve = original_curve.clone();
        // A dark sample in the middle of the trace is reachable by an
        // exact-time query and would divide by zero.
        tampered.burn_curve[1].chamber_pa = 0.0;
        assert!(matches!(
            CompiledEngine::Solid(tampered.clone()).validate(),
            Err(PropulsionError::InvalidSpec(_))
        ));
        tampered.burn_curve[1].chamber_pa = original_curve[1].chamber_pa;
        // NaN anywhere in the trace is not a physical burn state.
        tampered.burn_curve[0].mass_flow_kg_s = f64::NAN;
        assert!(matches!(
            CompiledEngine::Solid(tampered.clone()).validate(),
            Err(PropulsionError::InvalidSpec(_))
        ));
        tampered.burn_curve = original_curve;
        CompiledEngine::Solid(tampered)
            .validate()
            .expect("untampered bake still validates");
    }

    /// Grain probe for the validation tests: a plain two-segment APCP
    /// motor that compiles to a lit trace with a burn-through tail.
    fn probe_solid() -> CompiledSolid {
        let (a, n) = SolidMotorSpec::apcp_ballistics();
        SolidMotorSpec {
            name: "validate probe".into(),
            propellant: Propellant::SolidApcp,
            outer_radius_m: 0.5,
            core_radius_m: 0.32,
            grain_geometry: SolidGrainGeometry::Circular,
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
        .expect("solid")
    }
}
