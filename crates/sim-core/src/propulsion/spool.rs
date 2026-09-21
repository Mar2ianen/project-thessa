//! Spool / ignition runtime state: first-order throttle lag for liquids,
//! ballistic ignition plus burn clock for solids.

use serde::{Deserialize, Serialize};

use super::{CompiledEngine, PropulsionError};

/// Live engine state: ignition, spool lag, solid burn clock. Pure data;
/// advance with [`advance_spool`] using physics seconds (f64 durations, as
/// in the integrator kernels — never `Instant`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct EngineSpool {
    pub running: bool,
    pub throttle_actual: f64,
    pub shots_remaining: Option<u32>,
    pub burn_elapsed_s: f64,
}

impl EngineSpool {
    /// New engine: cold, full shots, zero burn clock.
    pub fn new(engine: &CompiledEngine) -> Self {
        Self {
            running: false,
            throttle_actual: 0.0,
            shots_remaining: match engine {
                CompiledEngine::Liquid(liquid) => {
                    if liquid.restartable {
                        None
                    } else {
                        Some(1)
                    }
                }
                CompiledEngine::Solid(solid) => Some(solid.ignition_shots),
            },
            burn_elapsed_s: 0.0,
        }
    }
}

/// Advance live state toward a throttle command over `dt_s`. First-order
/// spool lag for liquids; ballistic ignition + burn clock for solids.
pub fn advance_spool(
    engine: &CompiledEngine,
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
    match engine {
        CompiledEngine::Liquid(liquid) => {
            let (floor, _) = engine.throttle_range();
            let target = if throttle_cmd == 0.0 {
                0.0
            } else {
                throttle_cmd.max(floor)
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
            let alpha = 1.0 - (-dt_s / liquid.spool_tau_s).exp();
            state.throttle_actual += (target - state.throttle_actual) * alpha;
            if !state.running && state.throttle_actual < 1e-6 {
                state.throttle_actual = 0.0;
            }
            Ok(state)
        }
        CompiledEngine::Solid(solid) => {
            if throttle_cmd != 0.0 && throttle_cmd != 1.0 {
                return Err(PropulsionError::InvalidCommand(
                    "solid throttle commands are 0 (safe) or 1 (fire)".into(),
                ));
            }
            if state.burn_elapsed_s >= solid.burn_time_s {
                state.running = false;
                state.throttle_actual = 0.0;
                return Ok(state);
            }
            if !state.running {
                if throttle_cmd == 0.0 {
                    return Ok(state);
                }
                if let Some(shots) = state.shots_remaining {
                    if shots == 0 {
                        return Err(PropulsionError::InvalidCommand(
                            "motor already burned".into(),
                        ));
                    }
                    state.shots_remaining = Some(shots - 1);
                }
                state.running = true;
            }
            state.throttle_actual = 1.0;
            state.burn_elapsed_s = (state.burn_elapsed_s + dt_s).min(solid.burn_time_s);
            if state.burn_elapsed_s >= solid.burn_time_s {
                state.running = false;
                state.throttle_actual = 0.0;
            }
            Ok(state)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::super::liquid::merlin_like;
    use super::super::{ChamberMaterial, NozzleContour, Propellant, SolidMotorSpec};
    use super::*;

    #[test]
    fn spool_ignition_shutdown_and_solid_single_shot() {
        let liquid = CompiledEngine::Liquid(merlin_like().compile().expect("compile"));
        let mut state = EngineSpool::new(&liquid);
        assert!(state.shots_remaining.is_none());
        state = advance_spool(&liquid, state, 1.0, 10.0).expect("ignite");
        assert!(state.running);
        assert!((state.throttle_actual - 1.0).abs() < 1e-6);
        // Below the stability floor clamps up (documented), zero shuts down.
        state = advance_spool(&liquid, state, 0.05, 10.0).expect("clamp");
        assert!(state.running);
        assert!(state.throttle_actual >= 0.40 - 1e-9);
        state = advance_spool(&liquid, state, 0.0, 10.0).expect("shutdown");
        assert!(!state.running);

        let (a, n) = SolidMotorSpec::apcp_ballistics();
        let solid_spec = SolidMotorSpec {
            name: "single shot".into(),
            propellant: Propellant::SolidApcp,
            outer_radius_m: 0.3,
            core_radius_m: 0.1,
            segment_length_m: 1.0,
            segments: 2,
            burn_rate_coeff: a,
            burn_rate_exponent: n,
            throat_radius_m: 0.09,
            expansion_ratio: 8.0,
            nozzle_length_m: 0.6,
            contour: NozzleContour::Conical,
            casing_material: ChamberMaterial::nickel_superalloy(),
            inhibited_ends: true,
            segment_core_radii_m: None,
            gimbal_range_rad: 0.0,
            ignition_shots: 1,
        };
        let solid = CompiledEngine::Solid(solid_spec.compile().expect("solid"));
        let mut state = EngineSpool::new(&solid);
        assert!(advance_spool(&solid, state, 0.5, 0.1).is_err());
        state = advance_spool(&solid, state, 1.0, 0.1).expect("ignite");
        assert!(state.running && state.throttle_actual == 1.0);
        state = advance_spool(&solid, state, 1.0, 1.0e6).expect("burnout");
        assert!(!state.running && state.throttle_actual == 0.0);
        assert!(advance_spool(&solid, state, 1.0, 0.1).is_ok());
    }
}
