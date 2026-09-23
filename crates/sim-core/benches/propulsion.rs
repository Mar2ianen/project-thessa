//! Propulsion backend benchmarks (`docs/details/04` section 17):
//! hangar compile cost plus runtime evaluation throughput for the editor
//! analyzer path (thrust/Isp sweeps over altitude x throttle), the solid
//! burn-trace compile, and the jet shaft runtime (steady spool
//! equilibrium solve + cold crank, section 8.1 / section 18.8).
use std::{hint::black_box, time::Instant};

use thessa_sim_core::{
    AirCycle, AirbreathingSpec, AtmosphereConfig, ChamberMaterial, CompiledEngine, CoolingMode,
    ElectricMotorSpec, EngineCycle, GasKind, IntakeKind, JetFuel, JetShaftState, LiquidEngineSpec,
    NozzleContour, PistonEngineSpec, Propellant, PropellerDriveSpec, PropellerSpec, ShaftCommand,
    ShaftPowerSourceSpec, ShaftSpec, SolidGrainGeometry, SolidMotorSpec, StarterKind, StarterSpec,
    TurbopropDriveSpec, advance_jet_shaft, analyze_airbreathing, analyze_altitude,
    analyze_propeller_drive, analyze_turboprop_drive, flight_condition,
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
        grain_geometry: SolidGrainGeometry::Circular,
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

    for (label, geometry) in [
        (
            "star",
            SolidGrainGeometry::Star {
                tip_count: 6,
                tip_radius_m: 0.30,
            },
        ),
        (
            "finocyl",
            SolidGrainGeometry::Finocyl {
                fin_count: 8,
                fin_tip_radius_m: 0.32,
                fin_width_rad: 0.24,
            },
        ),
    ] {
        let spec = SolidMotorSpec {
            name: format!("bench-{label}"),
            core_radius_m: 0.16,
            grain_geometry: geometry,
            ..apcp_spec()
        };
        let compile_iters = 10;
        let start = Instant::now();
        for _ in 0..compile_iters {
            black_box(spec.compile().expect("shaped solid compile"));
        }
        let compile_us = start.elapsed().as_secs_f64() * 1.0e6 / compile_iters as f64;
        println!(
            "{label} grain compile: {compile_us:.1} us ({} segments)",
            spec.segments
        );
    }

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

    // Jet shaft runtime (section 8.1): the steady spool equilibrium
    // solve behind `operating_point` (40-halving bisection) and a cold
    // crank to light-off on the runtime shaft machine.
    let jet = AirbreathingSpec {
        name: "bench-jet".into(),
        cycle: AirCycle::Turbojet,
        fuel: JetFuel::Kerosene,
        intake_area_m2: 0.9,
        intake: IntakeKind::Pitot,
        compressor_ratio: 12.0,
        bypass_ratio: 0.0,
        fan_pressure_ratio: 1.0,
        turbine_inlet_temp_k: 1500.0,
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
    .expect("bench jet");
    let sample = atmosphere.sample(0.0).expect("SL sample");
    let condition = flight_condition(&sample, 0.0).expect("condition");

    let start = Instant::now();
    for _ in 0..iters {
        black_box(jet.operating_point(&condition, 1.0).expect("steady point"));
    }
    let per_solve_us = start.elapsed().as_secs_f64() * 1.0e6 / iters as f64;
    println!("jet steady spool solve: {per_solve_us:.2} us/solve ({iters} solves)");

    let start = Instant::now();
    let mut state = JetShaftState::cold(&jet);
    let command = ShaftCommand {
        throttle: 1.0,
        starter_engaged: true,
        generator_load_w: 0.0,
    };
    let mut crank_steps = 0usize;
    while !state.lit && crank_steps < 4000 {
        state = advance_jet_shaft(&jet, state, &command, &condition, 0.1)
            .expect("shaft step")
            .0;
        crank_steps += 1;
    }
    assert!(
        state.lit,
        "bench starter must light the core within {} steps",
        crank_steps
    );
    let crank_ns = start.elapsed().as_secs_f64() * 1.0e9 / crank_steps as f64;
    println!(
        "jet cold crank to light-off: {crank_steps} steps ({:.1} sim s) at {crank_ns:.0} ns/step",
        crank_steps as f64 * 0.1
    );

    // Composition-aware atmosphere (section 10 / 18.9): oxidizer query
    // cost plus the analyzer sweep that stamps species into every sample.
    let composition = atmosphere.sample(0.0).expect("SL sample").composition;
    let queries = iters * 10_000;
    let start = Instant::now();
    let mut acc = 0.0;
    for _ in 0..queries {
        acc += composition.mass_fraction(GasKind::Oxygen);
    }
    black_box(acc);
    let per_query_ns = start.elapsed().as_secs_f64() * 1.0e9 / queries as f64;
    println!("composition mass-fraction query: {per_query_ns:.2} ns/query");

    let altitudes: Vec<f64> = (0..=10).map(|k| k as f64 * 8000.0).collect();
    let machs = [0.0, 0.5, 1.0, 2.0, 3.0];
    let warm_rows =
        analyze_airbreathing(&jet, &atmosphere, &altitudes, &machs, 1.0).expect("air grid");
    let start = Instant::now();
    for _ in 0..iters {
        black_box(
            analyze_airbreathing(&jet, &atmosphere, &altitudes, &machs, 1.0).expect("air grid"),
        );
    }
    let per_row_ns = start.elapsed().as_secs_f64() * 1.0e9 / (iters * warm_rows.len()) as f64;
    println!(
        "air analyzer row: {per_row_ns:.1} ns/row ({} rows/iter, species stamped per row)",
        warm_rows.len()
    );

    let scramjet = AirbreathingSpec {
        name: "bench-scramjet".into(),
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
    let scramjet_machs = [0.0, 1.0, 2.0, 4.0, 6.0, 8.0];
    let warm_rows = analyze_airbreathing(&scramjet, &atmosphere, &altitudes, &scramjet_machs, 1.0)
        .expect("scramjet analyzer");
    let start = Instant::now();
    for _ in 0..iters {
        black_box(
            analyze_airbreathing(&scramjet, &atmosphere, &altitudes, &scramjet_machs, 1.0)
                .expect("scramjet analyzer"),
        );
    }
    let per_row_ns = start.elapsed().as_secs_f64() * 1.0e9 / (iters * warm_rows.len()) as f64;
    println!(
        "scramjet analyzer row: {per_row_ns:.1} ns/row ({} rows/iter)",
        warm_rows.len()
    );

    // Shaft-power aircraft analyzer (section 9): electric and piston sources
    // driving the same ideal actuator-disk component over an 11-altitude ×
    // 4-airspeed target grid.
    let electric_drive = PropellerDriveSpec {
        propeller: PropellerSpec::default(),
        source: ShaftPowerSourceSpec::Electric(ElectricMotorSpec::default()),
        reduction_ratio: 2.0,
    }
    .compile()
    .expect("electric propeller drive");
    let piston_drive = PropellerDriveSpec {
        propeller: PropellerSpec::default(),
        source: ShaftPowerSourceSpec::Piston(PistonEngineSpec {
            cooling_capacity_w: 100_000.0,
            ..PistonEngineSpec::default()
        }),
        reduction_ratio: 1.0,
    }
    .compile()
    .expect("piston propeller drive");
    let source_rpm = 2_400.0;
    let airspeeds_mps = [0.0, 50.0, 100.0, 150.0];
    for (label, drive) in [("electric", &electric_drive), ("piston", &piston_drive)] {
        let warm_rows = analyze_propeller_drive(
            drive,
            &atmosphere,
            &altitudes,
            &airspeeds_mps,
            1.0,
            source_rpm,
        )
        .expect("shaft-power analyzer");
        let start = Instant::now();
        for _ in 0..iters {
            black_box(
                analyze_propeller_drive(
                    drive,
                    &atmosphere,
                    &altitudes,
                    &airspeeds_mps,
                    1.0,
                    source_rpm,
                )
                .expect("shaft-power analyzer"),
            );
        }
        let per_row_ns = start.elapsed().as_secs_f64() * 1.0e9 / (iters * warm_rows.len()) as f64;
        println!(
            "{label} propeller analyzer row: {per_row_ns:.1} ns/row ({} rows/iter)",
            warm_rows.len()
        );
    }

    let turboprop = TurbopropDriveSpec {
        air: AirbreathingSpec {
            name: "bench-turboprop-core".into(),
            cycle: AirCycle::Turbojet,
            fuel: JetFuel::Kerosene,
            intake_area_m2: 0.9,
            intake: IntakeKind::Pitot,
            compressor_ratio: 12.0,
            bypass_ratio: 0.0,
            fan_pressure_ratio: 1.0,
            turbine_inlet_temp_k: 1_500.0,
            afterburner: false,
            reheat_temp_k: 0.0,
            turbine_material: ChamberMaterial::nickel_superalloy(),
            spool_tau_s: 5.0,
            shaft: ShaftSpec {
                power_turbine_heat_fraction: 0.15,
                ..ShaftSpec::default()
            },
        },
        propeller: PropellerSpec::default(),
        shaft_rpm_at_full_spool: 12_000.0,
        reduction_ratio: 6.0,
        power_turbine_mass_kg: 45.0,
    }
    .compile()
    .expect("turboprop drive");
    let warm_rows = analyze_turboprop_drive(
        &turboprop,
        &atmosphere,
        &altitudes,
        &airspeeds_mps,
        1.0,
        0.25,
    )
    .expect("turboprop analyzer");
    let start = Instant::now();
    for _ in 0..iters {
        black_box(
            analyze_turboprop_drive(
                &turboprop,
                &atmosphere,
                &altitudes,
                &airspeeds_mps,
                1.0,
                0.25,
            )
            .expect("turboprop analyzer"),
        );
    }
    let per_row_ns = start.elapsed().as_secs_f64() * 1.0e9 / (iters * warm_rows.len()) as f64;
    println!(
        "turboprop analyzer row: {per_row_ns:.1} ns/row ({} rows/iter)",
        warm_rows.len()
    );
}

fn solid_burn_time(engine: &CompiledEngine) -> f64 {
    match engine {
        CompiledEngine::Solid(solid) => solid.burn_time_s,
        CompiledEngine::Liquid(_) => 0.0,
    }
}
