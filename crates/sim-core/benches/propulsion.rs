//! Propulsion backend benchmarks (`docs/details/04` section 17):
//! hangar compile cost plus runtime evaluation throughput for the editor
//! analyzer path (thrust/Isp sweeps over altitude x throttle) and the
//! solid burn-trace compile.
use std::{hint::black_box, time::Instant};

use thessa_sim_core::{
    AtmosphereConfig, ChamberMaterial, CompiledEngine, CoolingMode, EngineCycle, LiquidEngineSpec,
    NozzleContour, Propellant, SolidMotorSpec, analyze_altitude,
};

fn methalox_spec() -> LiquidEngineSpec {
    LiquidEngineSpec {
        name: "bench-methalox".into(),
        propellant: Propellant::LoxMethane,
        cycle: EngineCycle::GasGenerator,
        chamber_pressure_pa: 12.0e6,
        throat_radius_m: 0.15,
        expansion_ratio: 35.0,
        nozzle_length_m: 1.8,
        contour: NozzleContour::Bell,
        chamber_material: ChamberMaterial::nickel_superalloy(),
        cooling: CoolingMode::Regenerative,
        mixture_ratio: None,
        characteristic_length_m: None,
        gimbal_range_rad: 0.09,
        min_throttle: None,
        restartable: true,
    }
}

fn apcp_spec() -> SolidMotorSpec {
    let (a, n) = SolidMotorSpec::apcp_ballistics();
    SolidMotorSpec {
        name: "bench-apcp".into(),
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
    }
}

fn main() {
    // Hangar compile: one liquid + one solid.
    let start = Instant::now();
    let liquid = CompiledEngine::Liquid(methalox_spec().compile().expect("liquid"));
    let liquid_compile_us = start.elapsed().as_secs_f64() * 1.0e6;
    let start = Instant::now();
    let solid = CompiledEngine::Solid(apcp_spec().compile().expect("solid"));
    let solid_compile_us = start.elapsed().as_secs_f64() * 1.0e6;
    println!("liquid compile: {liquid_compile_us:.1} us, solid compile: {solid_compile_us:.1} us");

    // Analyzer sweep: 21 altitudes x 5 throttles (editor slider path).
    let atmosphere = AtmosphereConfig::default();
    let altitudes: Vec<f64> = (0..21).map(|k| k as f64 * 4000.0).collect();
    let throttles = [0.4, 0.55, 0.7, 0.85, 1.0];
    let iters = 200;
    let start = Instant::now();
    for _ in 0..iters {
        for throttle in throttles {
            let curve =
                analyze_altitude(&liquid, &atmosphere, &altitudes, throttle, 0.0).expect("analyze");
            black_box(curve);
        }
    }
    let evals = iters * throttles.len() * altitudes.len();
    let per_eval_ns = start.elapsed().as_secs_f64() * 1.0e9 / evals as f64;
    println!("analyzer point: {per_eval_ns:.1} ns/eval ({evals} evals)");

    // Solid altitude replay across the burn trace.
    let start = Instant::now();
    let steps = 64;
    for _ in 0..iters {
        for step in 0..=steps {
            let burn_time_s = solid_burn_time(&solid) * step as f64 / steps as f64;
            for ambient_pa in [101_325.0, 30_000.0, 0.0] {
                let point = solid
                    .operating_point(1.0, ambient_pa, burn_time_s)
                    .expect("solid point");
                black_box(point);
            }
        }
    }
    let evals = iters * (steps + 1) * 3;
    let per_eval_ns = start.elapsed().as_secs_f64() * 1.0e9 / evals as f64;
    println!("solid replay point: {per_eval_ns:.1} ns/eval ({evals} evals)");
}

fn solid_burn_time(engine: &CompiledEngine) -> f64 {
    match engine {
        CompiledEngine::Solid(solid) => solid.burn_time_s,
        CompiledEngine::Liquid(_) => 0.0,
    }
}
