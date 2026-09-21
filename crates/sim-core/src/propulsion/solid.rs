//! Solid motors: BATES cylindrical-port grains (uniform or stepped channel)
//! with equilibrium burn traces solved in closed form per station.

use serde::{Deserialize, Serialize};

use super::{
    AEROSPIKE_BASE_FRACTION, ChamberMaterial, MASS_FIT_FEED_KG_PER_N, MASS_FIT_GIMBAL_BASE_KG,
    MASS_FIT_GIMBAL_KG_PER_N, MASS_FIT_MOUNT_KG_PER_N, MASS_FIT_SOLID_IGNITER_KG, NozzleContour,
    NozzleExitState, PRESSURE_SAFETY_FACTOR, Propellant, PropellantThermo, PropulsionError,
    STANDARD_GRAVITY_MPS2, characteristic_velocity, nozzle_exit, require_non_negative,
    require_positive, thrust_coefficient,
};

/// BATES cylindrical-port grain authoring (Juno fuel-grain equivalent).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SolidMotorSpec {
    pub name: String,
    /// Ammonium-perchlorate composite today; the grain solver reads thermo
    /// from the pair so future chemistries plug in without a new solver.
    pub propellant: Propellant,
    /// Grain outer radius (m).
    pub outer_radius_m: f64,
    /// Initial port (core) radius (m).
    pub core_radius_m: f64,
    /// Grain length per segment (m).
    pub segment_length_m: f64,
    /// Number of segments.
    pub segments: u32,
    /// Burn-rate coefficient a in r = a * Pc^n (m/s/Pa^n).
    pub burn_rate_coeff: f64,
    /// Burn-rate exponent n (must be < 1 for stable equilibrium).
    pub burn_rate_exponent: f64,
    pub throat_radius_m: f64,
    pub expansion_ratio: f64,
    pub nozzle_length_m: f64,
    pub contour: NozzleContour,
    pub casing_material: ChamberMaterial,
    /// Inhibited segment ends (neutral trace); exposed ends burn too.
    pub inhibited_ends: bool,
    /// Per-segment initial port radii (m) for a stepped channel
    /// (boost-sustain shaping); `None` = uniform `core_radius_m`.
    pub segment_core_radii_m: Option<Vec<f64>>,
    pub gimbal_range_rad: f64,
    /// Ignition shots carried (solids are single-shot by default).
    pub ignition_shots: u32,
}

impl SolidMotorSpec {
    /// Reference APCP ballistics (documented typical: ~10 mm/s at 7 MPa).
    pub fn apcp_ballistics() -> (f64, f64) {
        (4.0e-5, 0.35)
    }

    /// Validate authoring values (NaN fails closed).
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    pub fn validate(&self) -> Result<(), PropulsionError> {
        if self.name.trim().is_empty() {
            return Err(PropulsionError::InvalidSpec(
                "motor name must not be empty".into(),
            ));
        }
        if !self.propellant.is_solid() {
            return Err(PropulsionError::InvalidSpec(
                "grain ballistics need a solid propellant".into(),
            ));
        }
        require_positive(self.outer_radius_m, "grain outer radius")?;
        require_positive(self.core_radius_m, "grain core radius")?;
        if !(self.core_radius_m < self.outer_radius_m) {
            return Err(PropulsionError::InvalidSpec(
                "grain core must be smaller than the outer radius".into(),
            ));
        }
        require_positive(self.segment_length_m, "grain segment length")?;
        if self.segments == 0 || self.segments > 16 {
            return Err(PropulsionError::InvalidSpec(
                "segments must be in 1..=16".into(),
            ));
        }
        require_positive(self.burn_rate_coeff, "burn-rate coefficient")?;
        if !(self.burn_rate_exponent > 0.0) || !(self.burn_rate_exponent < 1.0) {
            return Err(PropulsionError::InvalidSpec(
                "burn-rate exponent must be in (0, 1): n >= 1 has no stable pressure equilibrium"
                    .into(),
            ));
        }
        require_positive(self.throat_radius_m, "throat radius")?;
        if !(self.expansion_ratio >= 1.0) {
            return Err(PropulsionError::InvalidSpec(
                "expansion ratio must be finite and >= 1".into(),
            ));
        }
        require_positive(self.nozzle_length_m, "nozzle length")?;
        self.casing_material.validate()?;
        require_non_negative(self.gimbal_range_rad, "gimbal range")?;
        if let Some(radii) = &self.segment_core_radii_m {
            if radii.len() != self.segments as usize {
                return Err(PropulsionError::InvalidSpec(
                    "segment port radii must match the segment count".into(),
                ));
            }
            for radius_m in radii {
                require_positive(*radius_m, "segment port radius")?;
                if !(*radius_m < self.outer_radius_m) {
                    return Err(PropulsionError::InvalidSpec(
                        "segment port must be smaller than the outer radius".into(),
                    ));
                }
            }
        }
        if self.ignition_shots == 0 {
            return Err(PropulsionError::InvalidSpec(
                "a solid motor with zero shots can never ignite".into(),
            ));
        }
        Ok(())
    }

    /// Hangar compile: solve the equilibrium burn trace over the web.
    pub fn compile(&self) -> Result<CompiledSolid, PropulsionError> {
        self.validate()?;
        let thermo = self.propellant.thermo();
        let c_star = characteristic_velocity(&thermo);
        let throat_area_m2 = std::f64::consts::PI * self.throat_radius_m * self.throat_radius_m;
        let exit_radius_m = self.throat_radius_m * self.expansion_ratio.sqrt();
        let exit = nozzle_exit(&thermo, self.expansion_ratio)?;
        let wall_slope = (exit_radius_m - self.throat_radius_m) / self.nozzle_length_m;
        let conical_divergence = 0.5 * (1.0 + wall_slope.atan().cos());
        let divergence = match self.contour {
            NozzleContour::Conical => conical_divergence,
            NozzleContour::Bell => conical_divergence + (1.0 - conical_divergence) * 0.5,
            NozzleContour::Aerospike => 0.99,
        };

        // Web-burn trace: time-stepped equilibrium. Each segment regresses
        // its own port (stepped channels shape boost-sustain traces); the
        // common chamber pressure couples them through total burn area:
        // Pc = [Ab(t) a rho c* / At]^(1/(1-n)). Sliver after burn-through
        // is ignored (documented).
        let density = thermo.bulk_density_kg_m3;
        let core_radii: Vec<f64> = match &self.segment_core_radii_m {
            Some(radii) => radii.clone(),
            None => vec![self.core_radius_m; self.segments as usize],
        };
        let webs: Vec<f64> = core_radii
            .iter()
            .map(|core_m| self.outer_radius_m - core_m)
            .collect();
        let burn_area_at = |burned: &[f64]| -> f64 {
            let mut area_m2 = 0.0;
            for (index, core_m) in core_radii.iter().enumerate() {
                if burned[index] >= webs[index] {
                    continue;
                }
                let port_m = core_m + burned[index];
                area_m2 += std::f64::consts::PI * 2.0 * port_m * self.segment_length_m;
                if !self.inhibited_ends {
                    area_m2 += 2.0
                        * std::f64::consts::PI
                        * (self.outer_radius_m * self.outer_radius_m - port_m * port_m);
                }
            }
            area_m2
        };
        let pressure_at = |area_m2: f64| -> Result<f64, PropulsionError> {
            let chamber_pa = (area_m2 * self.burn_rate_coeff * density * c_star / throat_area_m2)
                .powf(1.0 / (1.0 - self.burn_rate_exponent));
            if !chamber_pa.is_finite() || chamber_pa <= 0.0 {
                return Err(PropulsionError::InvalidSpec(
                    "grain equilibrium pressure is non-physical".into(),
                ));
            }
            Ok(chamber_pa)
        };
        let point_at = |time_s: f64,
                        burned_now: &[f64],
                        burned_max_m: f64|
         -> Result<BurnPoint, PropulsionError> {
            let burn_area_m2 = burn_area_at(burned_now);
            let chamber_pa = pressure_at(burn_area_m2)?;
            let burn_rate_mps = self.burn_rate_coeff * chamber_pa.powf(self.burn_rate_exponent);
            let mass_flow_kg_s = density * burn_area_m2 * burn_rate_mps;
            let thrust_sl_n = thrust_coefficient(
                &thermo,
                chamber_pa,
                &exit,
                self.expansion_ratio,
                101_325.0,
                divergence,
            ) * chamber_pa
                * throat_area_m2;
            let thrust_vac_n = thrust_coefficient(
                &thermo,
                chamber_pa,
                &exit,
                self.expansion_ratio,
                0.0,
                divergence,
            ) * chamber_pa
                * throat_area_m2;
            Ok(BurnPoint {
                time_s,
                web_burned_m: burned_max_m,
                chamber_pa,
                mass_flow_kg_s,
                thrust_sl_n,
                thrust_vac_n,
            })
        };
        let initial_area_m2 = burn_area_at(&vec![0.0; core_radii.len()]);
        let initial_rate_mps =
            self.burn_rate_coeff * pressure_at(initial_area_m2)?.powf(self.burn_rate_exponent);
        let max_web_m = webs.iter().cloned().fold(0.0_f64, f64::max);
        let dt_s = max_web_m / initial_rate_mps / 400.0;
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(PropulsionError::InvalidSpec(
                "grain step sizing is non-physical".into(),
            ));
        }
        let mut burned = vec![0.0; core_radii.len()];
        let mut time_s = 0.0;
        let mut points = vec![point_at(0.0, &burned, 0.0)?];
        let mut peak_pressure_pa = 0.0_f64;
        let mut peak_thrust_n = 0.0_f64;
        loop {
            let burn_area_m2 = burn_area_at(&burned);
            if burn_area_m2 <= 0.0 {
                break;
            }
            let chamber_pa = pressure_at(burn_area_m2)?;
            let burn_rate_mps = self.burn_rate_coeff * chamber_pa.powf(self.burn_rate_exponent);
            let mut all_done = true;
            let mut burned_max_m = 0.0_f64;
            for (index, web_m) in webs.iter().enumerate() {
                if burned[index] < *web_m {
                    burned[index] = (burned[index] + burn_rate_mps * dt_s).min(*web_m);
                }
                burned_max_m = burned_max_m.max(burned[index]);
                if burned[index] < *web_m {
                    all_done = false;
                }
            }
            time_s += dt_s;
            // Burn-through: the trace terminates at zero thrust rather
            // than erroring on the empty grain.
            let point = if burn_area_at(&burned) <= 0.0 {
                BurnPoint {
                    time_s,
                    web_burned_m: burned_max_m,
                    chamber_pa: 0.0,
                    mass_flow_kg_s: 0.0,
                    thrust_sl_n: 0.0,
                    thrust_vac_n: 0.0,
                }
            } else {
                point_at(time_s, &burned, burned_max_m)?
            };
            peak_pressure_pa = peak_pressure_pa.max(point.chamber_pa);
            peak_thrust_n = peak_thrust_n.max(point.thrust_sl_n);
            points.push(point);
            if all_done {
                break;
            }
            if points.len() > 100_000 {
                return Err(PropulsionError::InvalidSpec(
                    "grain does not burn through".into(),
                ));
            }
        }
        peak_pressure_pa = peak_pressure_pa.max(points[0].chamber_pa);
        peak_thrust_n = peak_thrust_n.max(points[0].thrust_sl_n);
        // Integrate impulse and propellant over the recorded trace.
        let mut total_impulse_ns = 0.0;
        let mut propellant_kg = 0.0;
        for window in points.windows(2) {
            let (a, b) = (window[0], window[1]);
            let dt = b.time_s - a.time_s;
            total_impulse_ns += 0.5 * (a.thrust_vac_n + b.thrust_vac_n) * dt;
            propellant_kg += 0.5 * (a.mass_flow_kg_s + b.mass_flow_kg_s) * dt;
        }
        let burn_time_s = time_s;
        let avg_isp_s = total_impulse_ns / (propellant_kg * STANDARD_GRAVITY_MPS2);

        // Casing: cylinder + hemispherical-cap allowance + shared nozzle path.
        let grain_length_m = self.segment_length_m * self.segments as f64;
        let case_thickness_m = peak_pressure_pa * 2.0 * self.outer_radius_m
            / (2.0 * self.casing_material.yield_strength_pa)
            * PRESSURE_SAFETY_FACTOR;
        let case_kg = 2.0
            * std::f64::consts::PI
            * self.outer_radius_m
            * grain_length_m
            * case_thickness_m
            * self.casing_material.density_kg_m3
            * 1.3;
        let slant_m = (self.nozzle_length_m * self.nozzle_length_m
            + (exit_radius_m - self.throat_radius_m).powi(2))
        .sqrt();
        let mut nozzle_kg = std::f64::consts::PI
            * (self.throat_radius_m + exit_radius_m)
            * slant_m
            * case_thickness_m
            * 0.6
            * self.casing_material.density_kg_m3;
        if self.contour == NozzleContour::Bell {
            nozzle_kg *= 0.85;
        }
        if self.contour == NozzleContour::Aerospike {
            let spike_base_m = 0.6 * exit_radius_m;
            let spike_slant_m =
                (self.nozzle_length_m * self.nozzle_length_m + spike_base_m * spike_base_m).sqrt();
            nozzle_kg = nozzle_kg * 0.7
                + std::f64::consts::PI
                    * spike_base_m
                    * spike_slant_m
                    * case_thickness_m
                    * 0.6
                    * self.casing_material.density_kg_m3;
        }
        let gimbal_kg = if self.gimbal_range_rad > 0.0 {
            MASS_FIT_GIMBAL_BASE_KG + MASS_FIT_GIMBAL_KG_PER_N * peak_thrust_n
        } else {
            0.0
        };
        let dry_mass_kg = case_kg
            + nozzle_kg
            + gimbal_kg
            + MASS_FIT_SOLID_IGNITER_KG
            + MASS_FIT_MOUNT_KG_PER_N * peak_thrust_n
            + MASS_FIT_FEED_KG_PER_N * peak_thrust_n;

        Ok(CompiledSolid {
            name: self.name.clone(),
            propellant: self.propellant,
            throat_area_m2,
            throat_radius_m: self.throat_radius_m,
            expansion_ratio: self.expansion_ratio,
            exit_radius_m,
            divergence_factor: divergence,
            gamma: thermo.gamma,
            chamber_temp_k: thermo.chamber_temp_k,
            gas_constant_j_kg_k: thermo.gas_constant_j_kg_k,
            c_star_mps: c_star,
            exit_mach: exit.exit_mach,
            exit_temp_k: exit.exit_temp_k,
            exhaust_velocity_mps: exit.exhaust_velocity_mps,
            burn_curve: points,
            burn_time_s,
            total_impulse_ns,
            propellant_mass_kg: propellant_kg,
            avg_isp_s,
            peak_pressure_pa,
            peak_thrust_sl_n: peak_thrust_n,
            dry_mass_kg,
            grain_outer_radius_m: self.outer_radius_m,
            grain_length_m,
            ignition_shots: self.ignition_shots,
            gimbal_range_rad: self.gimbal_range_rad,
            contour: self.contour,
            aerospike_base_area_m2: match self.contour {
                NozzleContour::Aerospike => {
                    AEROSPIKE_BASE_FRACTION * throat_area_m2 * self.expansion_ratio
                }
                _ => 0.0,
            },
        })
    }
}

/// One web station of a compiled solid burn trace (sea-level + vacuum
/// thrust stored; altitude replay varies only the pressure term, which is
/// exact for a choked motor at frozen chamber pressure).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BurnPoint {
    pub time_s: f64,
    pub web_burned_m: f64,
    pub chamber_pa: f64,
    pub mass_flow_kg_s: f64,
    pub thrust_sl_n: f64,
    pub thrust_vac_n: f64,
}

/// Hangar-compiled solid motor with its equilibrium burn trace.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CompiledSolid {
    pub name: String,
    pub propellant: Propellant,
    pub throat_area_m2: f64,
    pub throat_radius_m: f64,
    pub expansion_ratio: f64,
    pub exit_radius_m: f64,
    pub divergence_factor: f64,
    pub gamma: f64,
    pub chamber_temp_k: f64,
    pub gas_constant_j_kg_k: f64,
    pub c_star_mps: f64,
    pub exit_mach: f64,
    pub exit_temp_k: f64,
    pub exhaust_velocity_mps: f64,
    pub burn_curve: Vec<BurnPoint>,
    pub burn_time_s: f64,
    pub total_impulse_ns: f64,
    pub propellant_mass_kg: f64,
    pub avg_isp_s: f64,
    pub peak_pressure_pa: f64,
    pub peak_thrust_sl_n: f64,
    pub dry_mass_kg: f64,
    pub grain_outer_radius_m: f64,
    pub grain_length_m: f64,
    pub ignition_shots: u32,
    pub gimbal_range_rad: f64,
    pub contour: NozzleContour,
    /// Aerospike base area exposed to ambient (m^2, 0 for bell/cone).
    pub aerospike_base_area_m2: f64,
}

impl CompiledSolid {
    pub(crate) fn thermo_ref(&self) -> PropellantThermo {
        PropellantThermo {
            gamma: self.gamma,
            chamber_temp_k: self.chamber_temp_k,
            gas_constant_j_kg_k: self.gas_constant_j_kg_k,
            bulk_density_kg_m3: 0.0,
            characteristic_length_m: 0.0,
            reference_c_star_mps: 0.0,
        }
    }

    /// Design-point exit pressure ratio (frozen nozzle geometry: the ratio
    /// depends only on gamma and expansion ratio, never on chamber pressure
    /// or ambient).
    pub(crate) fn exit_pressure_ratio(&self) -> f64 {
        (1.0 + (self.gamma - 1.0) / 2.0 * self.exit_mach * self.exit_mach)
            .powf(-self.gamma / (self.gamma - 1.0))
    }

    pub(crate) fn exit_state(&self) -> NozzleExitState {
        NozzleExitState {
            exit_mach: self.exit_mach,
            exit_pressure_ratio: self.exit_pressure_ratio(),
            exit_temp_k: self.exit_temp_k,
            exhaust_velocity_mps: self.exhaust_velocity_mps,
        }
    }

    pub(crate) fn interpolate(&self, burn_time_s: f64) -> BurnPoint {
        let curve = &self.burn_curve;
        if curve.is_empty() {
            return BurnPoint {
                time_s: 0.0,
                web_burned_m: 0.0,
                chamber_pa: 0.0,
                mass_flow_kg_s: 0.0,
                thrust_sl_n: 0.0,
                thrust_vac_n: 0.0,
            };
        }
        if burn_time_s <= 0.0 {
            return curve[0];
        }
        for window in curve.windows(2) {
            let (a, b) = (window[0], window[1]);
            if burn_time_s <= b.time_s {
                let span = (b.time_s - a.time_s).max(1e-12);
                let fraction = ((burn_time_s - a.time_s) / span).clamp(0.0, 1.0);
                return BurnPoint {
                    time_s: burn_time_s,
                    web_burned_m: a.web_burned_m + (b.web_burned_m - a.web_burned_m) * fraction,
                    chamber_pa: a.chamber_pa + (b.chamber_pa - a.chamber_pa) * fraction,
                    mass_flow_kg_s: a.mass_flow_kg_s
                        + (b.mass_flow_kg_s - a.mass_flow_kg_s) * fraction,
                    thrust_sl_n: a.thrust_sl_n + (b.thrust_sl_n - a.thrust_sl_n) * fraction,
                    thrust_vac_n: a.thrust_vac_n + (b.thrust_vac_n - a.thrust_vac_n) * fraction,
                };
            }
        }
        *curve.last().expect("non-empty burn curve")
    }
}

#[cfg(test)]
mod tests {
    use super::super::{CompiledEngine, STANDARD_GRAVITY_MPS2};
    use super::*;

    #[test]
    fn solid_burn_trace_is_physical() {
        // BATES grain: a cylindrical port opens as it burns, so the trace
        // is inherently progressive (thrust rises monotonically) — neutral
        // traces need star/finocyl grains, which are deferred. The test pins
        // the progressive signature, impulse/mass/Isp consistency, and the
        // choked-motor property that altitude replay changes only the
        // pressure term at frozen chamber pressure.
        let (a, n) = SolidMotorSpec::apcp_ballistics();
        let spec = SolidMotorSpec {
            name: "APCP booster".into(),
            propellant: Propellant::SolidApcp,
            outer_radius_m: 0.5,
            core_radius_m: 0.32,
            segment_length_m: 1.5,
            segments: 4,
            burn_rate_coeff: a,
            burn_rate_exponent: n,
            throat_radius_m: 0.12,
            expansion_ratio: 10.0,
            nozzle_length_m: 0.9,
            contour: NozzleContour::Conical,
            casing_material: ChamberMaterial::nickel_superalloy(),
            inhibited_ends: true,
            segment_core_radii_m: None,
            gimbal_range_rad: 0.0,
            ignition_shots: 1,
        };
        let motor = spec.compile().expect("solid compiles");
        assert!(motor.burn_time_s > 1.0 && motor.burn_time_s < 60.0);
        // Time-stepped trace: resolved (hundreds of stations) with
        // strictly increasing time.
        assert!(motor.burn_curve.len() >= 100);
        for window in motor.burn_curve.windows(2) {
            assert!(window[1].time_s > window[0].time_s);
        }
        // Progressive signature: vacuum thrust never decreases along the
        // web (the terminal point is the zero-thrust burn-through).
        let live = &motor.burn_curve[..motor.burn_curve.len() - 1];
        for window in live.windows(2) {
            assert!(
                window[1].thrust_vac_n >= window[0].thrust_vac_n,
                "cylindrical-port trace must be progressive"
            );
        }
        let mean_vac = motor.total_impulse_ns / motor.burn_time_s;
        let peak_vac = motor
            .burn_curve
            .iter()
            .map(|point| point.thrust_vac_n)
            .fold(0.0_f64, f64::max);
        assert!(
            peak_vac / mean_vac < 1.8,
            "chunky-port progressivity must stay bounded"
        );
        // Impulse = Isp * m_prop * g0 (internal consistency).
        let check = motor.avg_isp_s * motor.propellant_mass_kg * STANDARD_GRAVITY_MPS2;
        assert!((check - motor.total_impulse_ns).abs() / motor.total_impulse_ns < 1e-9);
        // Chamber pressure frozen vs ambient: SL and vacuum evaluation at
        // mid-burn differ only by the (Pe - Pa) Ae term.
        let engine = CompiledEngine::Solid(motor);
        let mid = engine
            .operating_point(1.0, 101_325.0, 1.0)
            .expect("mid-burn SL");
        let mid_vac = engine.operating_point(1.0, 0.0, 1.0).expect("mid-burn vac");
        assert!((mid.mass_flow_kg_s - mid_vac.mass_flow_kg_s).abs() / mid.mass_flow_kg_s < 1e-12);
        assert!(engine.operating_point(0.5, 0.0, 1.0).is_err());
        // Past burnout: zero thrust, no error.
        let done = engine.operating_point(1.0, 0.0, 1.0e6).expect("burned out");
        assert_eq!(done.thrust_n, 0.0);
    }

    #[test]
    fn stepped_ports_shape_boost_sustain() {
        // Stepped channel: a thin-web segment burns out early (boost) and
        // the thick-web segment sustains. The test pins the two-phase
        // signature plus integrated-vs-geometric propellant agreement.
        let (a, n) = SolidMotorSpec::apcp_ballistics();
        let spec = SolidMotorSpec {
            name: "boost-sustain".into(),
            propellant: Propellant::SolidApcp,
            outer_radius_m: 0.5,
            core_radius_m: 0.15,
            segment_length_m: 1.5,
            segments: 2,
            burn_rate_coeff: a,
            burn_rate_exponent: n,
            throat_radius_m: 0.12,
            expansion_ratio: 10.0,
            nozzle_length_m: 0.9,
            contour: NozzleContour::Conical,
            casing_material: ChamberMaterial::nickel_superalloy(),
            inhibited_ends: true,
            segment_core_radii_m: Some(vec![0.15, 0.35]),
            gimbal_range_rad: 0.0,
            ignition_shots: 1,
        };
        let motor = spec.compile().expect("stepped grain compiles");
        let engine = CompiledEngine::Solid(motor.clone());
        let peak = motor
            .burn_curve
            .iter()
            .map(|point| point.thrust_vac_n)
            .fold(0.0_f64, f64::max);
        let peak_time = motor
            .burn_curve
            .iter()
            .find(|point| point.thrust_vac_n >= peak * (1.0 - 1e-9))
            .expect("peak station")
            .time_s;
        assert!(
            peak_time < 0.4 * motor.burn_time_s,
            "boost peak must sit in the first 40% of the burn"
        );
        let late = engine
            .operating_point(1.0, 0.0, 0.85 * motor.burn_time_s)
            .expect("late sustain")
            .thrust_n;
        assert!(
            late < 0.55 * peak,
            "sustain phase must drop below 55% of boost peak"
        );
        // Integrated propellant agrees with the geometric grain mass.
        let geometric: f64 = [0.15, 0.35]
            .iter()
            .map(|core_m| std::f64::consts::PI * (0.5 * 0.5 - core_m * core_m) * 1.5 * 1770.0)
            .sum();
        let drift = (motor.propellant_mass_kg - geometric).abs() / geometric;
        assert!(drift < 0.02, "propellant drift {drift:e} too large");
        // Port-count mismatch is refused, not silently broadcast.
        let bad = SolidMotorSpec {
            segment_core_radii_m: Some(vec![0.2]),
            ..spec
        };
        assert!(bad.compile().is_err());
    }

    #[test]
    fn unstable_burn_exponent_is_rejected() {
        let (a, _) = SolidMotorSpec::apcp_ballistics();
        let spec = SolidMotorSpec {
            name: "unstable".into(),
            propellant: Propellant::SolidApcp,
            outer_radius_m: 0.5,
            core_radius_m: 0.15,
            segment_length_m: 1.5,
            segments: 2,
            burn_rate_coeff: a,
            burn_rate_exponent: 1.2,
            throat_radius_m: 0.12,
            expansion_ratio: 8.0,
            nozzle_length_m: 0.8,
            contour: NozzleContour::Conical,
            casing_material: ChamberMaterial::nickel_superalloy(),
            inhibited_ends: true,
            segment_core_radii_m: None,
            gimbal_range_rad: 0.0,
            ignition_shots: 1,
        };
        assert!(spec.compile().is_err());
    }
}
