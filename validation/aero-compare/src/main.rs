use std::{error::Error, io::Cursor, process::Command};

use glam::{DMat3, DQuat, DVec3};
use thessa_sim_core::{
    AeroCase, AeroCoefficientTable, AeroConfig, AeroEnvironment, AeroGeometry, AeroModel,
    AeroPanel, AeroState, AtmosphereConfig, FlightStepInput, PanelAeroModel, RigidBodyProperties,
    RigidBodyState, integrate_rigid_body_duration,
};

struct Case {
    name: &'static str,
    case: AeroCase,
}

struct NormalizedCoefficients {
    lift: f64,
    drag: f64,
    pitch_moment: f64,
}

struct ReferenceResult {
    engine: String,
    model: String,
    mach: f64,
    alpha_deg: f64,
    lift: f64,
    drag: f64,
    pitch_moment: f64,
    lift_slope: f64,
    center_of_pressure_m: f64,
}

struct TrajectoryResult {
    mach: f64,
    speed_mps: f64,
    altitude_m: f64,
    alpha_deg: f64,
    dynamic_pressure_pa: f64,
}

const REFERENCE_SCRIPT: &str = include_str!("../reference_compare.py");

fn main() -> Result<(), Box<dyn Error>> {
    let cases = cases()?;
    let model = PanelAeroModel::new(AeroConfig::default())?;

    let trajectory = x15_trajectory()?;
    ensure_finite_trajectory("x15_like_6dof_proxy", &trajectory)?;
    print_x15_trajectory(&trajectory);

    println!("case,freestream_mps,mach,q_pa,force_x_n,force_z_n,moment_y_nm,cl,cd,cm");
    for case in &cases {
        let result = model.evaluate(&case.case)?;
        ensure_finite_result(case.name, &result)?;
        let coefficients = normalized_coefficients(&case.case, &result)?;
        ensure_finite_coefficients(case.name, &coefficients)?;
        println!(
            "{},{:.3},{:.5},{:.3},{:.3},{:.3},{:.3},{:.6},{:.6},{:.6}",
            case.name,
            case.case.state.velocity_body_mps.length(),
            result.mach,
            result.dynamic_pressure_pa,
            result.force_body_n.x,
            result.force_body_n.z,
            result.moment_body_nm.y,
            coefficients.lift,
            coefficients.drag,
            coefficients.pitch_moment,
        );
    }

    performance_smoke(&model, &cases)?;

    let strict_external = std::env::args().any(|argument| argument == "--require-external");
    let tools = [
        ("JSBSim", "jsbsim"),
        ("VSPAERO", "vspaero"),
        ("AVL", "avl"),
        ("OpenRocket", "openrocket"),
        ("SU2_CFD", "SU2_CFD"),
    ];
    let mut available = tools
        .iter()
        .filter(|(_, executable)| probe(executable))
        .map(|(name, _)| *name)
        .collect::<Vec<_>>();
    if probe_python_module("rocketpy") {
        available.push("RocketPy");
    }
    if probe_python_module("jsbsim") {
        available.push("JSBSim/Python");
    }
    if available.is_empty() {
        println!("external_engines=none (install optional validators to enable side-by-side runs)");
        if strict_external {
            return Err("--require-external requested, but no external engine is installed".into());
        }
    } else {
        println!("external_engines={}", available.join("|"));
    }

    let python = python_executable();
    if let Some(python) = python {
        println!(
            "comparison,case,engine,mach,alpha_deg,metric,ours,reference,abs_error,rel_error_pct"
        );
        let jsbsim_specs = [
            ("naca_like_subsonic_wing", "737"),
            ("x15_wing_only_proxy", "X15"),
            ("shuttle_hypersonic_entry_proxy", "Shuttle"),
        ];
        if python_module_available(&python, "jsbsim") {
            let reference = reference_trajectory_result(&python, "X15", 2.0, 5.0)?;
            print_scalar_comparison(
                "x15_like_6dof_proxy",
                "JSBSim/X15",
                "Mach",
                trajectory.mach,
                reference.mach,
            );
            print_scalar_comparison(
                "x15_like_6dof_proxy",
                "JSBSim/X15",
                "altitude_m",
                trajectory.altitude_m,
                reference.altitude_m,
            );
            print_scalar_comparison(
                "x15_like_6dof_proxy",
                "JSBSim/X15",
                "AoA_deg",
                trajectory.alpha_deg,
                reference.alpha_deg,
            );
            print_x15_imported_polar_comparison(&python)?;
            for (case_name, reference_model) in jsbsim_specs {
                let case = cases
                    .iter()
                    .find(|case| case.name == case_name)
                    .ok_or_else(|| format!("missing case {case_name}"))?;
                let result = model.evaluate(&case.case)?;
                let ours = normalized_coefficients(&case.case, &result)?;
                let reference = reference_result(
                    &python,
                    "jsbsim",
                    reference_model,
                    result.mach,
                    angle_of_attack_deg(&case.case),
                )?;
                print_comparison(case_name, &reference, "CL", ours.lift, reference.lift);
                print_comparison(case_name, &reference, "CD", ours.drag, reference.drag);
                print_comparison(
                    case_name,
                    &reference,
                    "CM",
                    ours.pitch_moment,
                    reference.pitch_moment,
                );
            }

            println!("reference_sweep,x15_wing_only_proxy,JSBSim/X15");
            for mach in [0.95, 1.1, 1.2, 1.5, 2.0, 3.0, 5.0] {
                let case = x15_wing_only_case(mach, 5.0)?;
                let result = model.evaluate(&case)?;
                let ours = normalized_coefficients(&case, &result)?;
                let reference = reference_result(&python, "jsbsim", "X15", mach, 5.0)?;
                print_comparison(
                    "x15_wing_only_proxy",
                    &reference,
                    "CL",
                    ours.lift,
                    reference.lift,
                );
                print_comparison(
                    "x15_wing_only_proxy",
                    &reference,
                    "CD",
                    ours.drag,
                    reference.drag,
                );
            }
        }

        if python_module_available(&python, "rocketpy") {
            let fin_config = AeroConfig {
                side_force_slope_per_rad: 0.0,
                ..AeroConfig::default()
            };
            let fin_model = PanelAeroModel::new(fin_config)?;
            for mach in [0.3, 0.8, 0.95, 1.1, 1.2, 2.0, 5.0] {
                let delta_alpha = 1.0e-4;
                let plus_case = rocket_fin_case(mach, delta_alpha)?;
                let minus_case = rocket_fin_case(mach, -delta_alpha)?;
                let plus_result = fin_model.evaluate(&plus_case)?;
                let minus_result = fin_model.evaluate(&minus_case)?;
                let rocket_reference_area_m2 = std::f64::consts::PI * 0.25_f64.powi(2);
                let plus = normalized_coefficients_with_reference(
                    &plus_case,
                    &plus_result,
                    rocket_reference_area_m2,
                    0.5,
                )?;
                let minus = normalized_coefficients_with_reference(
                    &minus_case,
                    &minus_result,
                    rocket_reference_area_m2,
                    0.5,
                )?;
                let ours_lift_slope = (plus.lift - minus.lift) / (2.0 * delta_alpha);
                let reference = reference_result(&python, "rocketpy", "", mach, 0.0)?;
                print_comparison(
                    "rocket_fin_set",
                    &reference,
                    "CL_alpha",
                    ours_lift_slope,
                    reference.lift_slope,
                );
                let ours_center_of_pressure_m =
                    -plus_result.moment_body_nm.y / plus_result.force_body_n.z;
                print_comparison(
                    "rocket_fin_set",
                    &reference,
                    "CP_m",
                    ours_center_of_pressure_m,
                    reference.center_of_pressure_m,
                );
            }
        }
    }
    println!("status=thessa_panel_regression_pass");
    Ok(())
}

fn print_x15_imported_polar_comparison(python: &str) -> Result<(), Box<dyn Error>> {
    let output = Command::new(python)
        .arg("-c")
        .arg(REFERENCE_SCRIPT)
        .arg("--reference")
        .arg("jsbsim")
        .arg("--model")
        .arg("X15")
        .arg("--mach")
        .arg("2.0")
        .arg("--alpha-deg")
        .arg("5.0")
        .arg("--polar")
        .output()?;
    if !output.status.success() {
        return Err(format!(
            "JSBSim polar export failed with status {}: {}",
            output.status,
            String::from_utf8_lossy(&output.stderr).trim()
        )
        .into());
    }
    let mut csv = String::from("mach,alpha_deg,cl,cd,cy,cm\n");
    for line in String::from_utf8_lossy(&output.stdout)
        .lines()
        .filter(|line| line.starts_with("polar,"))
    {
        let fields = line.split(',').collect::<Vec<_>>();
        if fields.len() != 8 {
            return Err(format!("invalid JSBSim polar row: {line}").into());
        }
        csv.push_str(&format!(
            "{},{},{},{},0,{}\n",
            fields[3], fields[4], fields[5], fields[6], fields[7]
        ));
    }
    let table = AeroCoefficientTable::from_csv(Cursor::new(csv))?;
    let model = PanelAeroModel::from_table(AeroConfig::default(), table)?;
    let case = x15_wing_only_case(2.0, 5.0)?;
    let result = model.evaluate_detailed(&case)?;
    let reference = reference_result(python, "jsbsim", "X15", 2.0, 5.0)?;
    let coefficients = normalized_coefficients(&case, &result)?;
    let table_pitch_moment = result
        .panel_loads
        .as_ref()
        .and_then(|loads| loads.first())
        .ok_or("detailed table evaluation did not return panel loads")?
        .coefficients
        .pitching_moment;
    print_comparison(
        "x15_imported_jsbsim_table",
        &reference,
        "CL",
        coefficients.lift,
        reference.lift,
    );
    print_comparison(
        "x15_imported_jsbsim_table",
        &reference,
        "CD",
        coefficients.drag,
        reference.drag,
    );
    print_comparison(
        "x15_imported_jsbsim_table",
        &reference,
        "CM",
        table_pitch_moment,
        reference.pitch_moment,
    );
    Ok(())
}

fn cases() -> Result<Vec<Case>, Box<dyn Error>> {
    let environment = AeroEnvironment::standard_sea_level();
    let wing = AeroGeometry::new(vec![AeroPanel::flat_plate(DVec3::ZERO, 20.0, 2.0)?])?;
    let rocket = AeroGeometry::new(vec![AeroPanel::flat_plate(DVec3::ZERO, 12.0, 4.0)?])?;
    let shuttle = AeroGeometry::new(vec![
        AeroPanel::flat_plate(DVec3::new(0.0, 0.0, 0.0), 45.0, 8.0)?,
        AeroPanel::flat_plate(DVec3::new(-4.0, 0.0, 0.0), 20.0, 5.0)?,
    ])?;
    let mut cases = vec![
        Case {
            name: "naca_like_subsonic_wing",
            case: AeroCase::new(
                AeroState::new(
                    DVec3::new(102.0, 0.0, 102.0 * 5.0_f64.to_radians().tan()),
                    DVec3::ZERO,
                ),
                environment,
                wing,
            )?,
        },
        Case {
            name: "rocket_transonic_axisymmetric_proxy",
            case: AeroCase::new(
                AeroState::new(
                    DVec3::new(0.95 * environment.speed_of_sound_mps, 0.0, 0.0),
                    DVec3::ZERO,
                ),
                environment,
                rocket,
            )?,
        },
        Case {
            name: "x15_wing_only_proxy",
            case: x15_wing_only_case(2.0, 5.0)?,
        },
        Case {
            name: "shuttle_hypersonic_entry_proxy",
            case: AeroCase::new(
                AeroState::new(
                    DVec3::new(
                        5.0 * environment.speed_of_sound_mps,
                        0.0,
                        5.0 * environment.speed_of_sound_mps * 20.0_f64.to_radians().tan(),
                    ),
                    DVec3::ZERO,
                ),
                environment,
                shuttle,
            )?,
        },
    ];
    for (name, mach) in [
        ("x15_mach_0_8", 0.8),
        ("x15_mach_1_0", 1.0),
        ("x15_mach_2_0", 2.0),
        ("x15_mach_6_0", 6.0),
    ] {
        cases.push(Case {
            name,
            case: x15_wing_only_case(mach, 5.0)?,
        });
    }
    Ok(cases)
}

fn performance_smoke(model: &PanelAeroModel, cases: &[Case]) -> Result<(), Box<dyn Error>> {
    let iterations = 25;
    let started = std::time::Instant::now();
    let mut completed = 0usize;
    for _ in 0..iterations {
        for case in cases {
            let result = model.evaluate(&case.case)?;
            ensure_finite_result(case.name, &result)?;
            completed += 1;
        }
    }
    let elapsed = started.elapsed();
    let default_limit_ms = 2_000u128;
    let limit_ms = std::env::var("THESSA_AERO_MAX_SMOKE_MS")
        .ok()
        .map(|value| value.parse::<u128>())
        .transpose()
        .map_err(|error| format!("invalid THESSA_AERO_MAX_SMOKE_MS: {error}"))?
        .unwrap_or(default_limit_ms);
    println!(
        "performance_smoke,cases={completed},elapsed_ms={},limit_ms={limit_ms}",
        elapsed.as_millis()
    );
    if elapsed.as_millis() > limit_ms {
        return Err(format!(
            "aero performance smoke exceeded {limit_ms} ms: {} ms for {completed} evaluations",
            elapsed.as_millis()
        )
        .into());
    }
    Ok(())
}

fn x15_wing_only_case(mach: f64, alpha_deg: f64) -> Result<AeroCase, Box<dyn Error>> {
    // Metrics copied as dimensions, not as coefficient data, from the
    // bundled JSBSim X15 model: Sw=200 ft^2, span=22.36 ft, c=10.27 ft.
    // The panel intentionally omits the fuselage and tail, so its comparison
    // reports the gap between a finite-wing proxy and the complete reference
    // aircraft rather than pretending to be an exact X-15 reconstruction.
    let area_m2: f64 = 200.0 * 0.09290304;
    let span_m: f64 = 22.36 * 0.3048;
    let chord_m: f64 = 10.27 * 0.3048;
    let aspect_ratio = span_m.powi(2) / area_m2;
    let atmosphere = AtmosphereConfig::default();
    let environment = atmosphere.aero_environment(24_384.0, DVec3::ZERO)?;
    let panel = AeroPanel::flat_plate(DVec3::ZERO, area_m2, chord_m)?
        .with_planform(span_m, aspect_ratio, 0.0, 1.0)?
        .with_thickness_ratio(0.04)?;
    let alpha_rad = alpha_deg.to_radians();
    Ok(AeroCase::new(
        AeroState::new(
            DVec3::new(
                mach * environment.speed_of_sound_mps * alpha_rad.cos(),
                0.0,
                mach * environment.speed_of_sound_mps * alpha_rad.sin(),
            ),
            DVec3::ZERO,
        ),
        environment,
        AeroGeometry::new(vec![panel])?,
    )?)
}

fn x15_flight_geometry() -> Result<AeroGeometry, Box<dyn Error>> {
    let wing_area_m2: f64 = 200.0 * 0.09290304;
    let wing_span_m: f64 = 22.36 * 0.3048;
    let wing_chord_m: f64 = 10.27 * 0.3048;
    let wing_aspect_ratio = wing_span_m.powi(2) / wing_area_m2;
    let wing = AeroPanel::flat_plate(DVec3::ZERO, wing_area_m2, wing_chord_m)?
        .with_planform(wing_span_m, wing_aspect_ratio, 0.0, 1.0)?
        .with_thickness_ratio(0.04)?;
    // A small horizontal tail gives the proxy a real longitudinal stability
    // path. It is deliberately an owned approximation, not a copied X-15
    // coefficient table.
    let tail = AeroPanel::flat_plate(DVec3::new(-4.0, 0.0, 0.0), 4.0, 1.2)?
        .with_planform(2.2, 1.21, 0.0, 1.0)?
        .with_lift_sign(-1.0)?
        .with_center_of_pressure(DVec3::new(-4.5, 0.0, 0.0))?;
    Ok(AeroGeometry::new(vec![wing, tail])?)
}

fn x15_trajectory() -> Result<TrajectoryResult, Box<dyn Error>> {
    let atmosphere = AtmosphereConfig::default();
    let geometry = x15_flight_geometry()?;
    let config = AeroConfig {
        pitching_moment_coefficient: -0.02,
        pitch_damping_coefficient: -4.0,
        ..AeroConfig::default()
    };
    let model = PanelAeroModel::new(config)?;
    let altitude_m = 24_384.0;
    let environment = atmosphere.aero_environment(altitude_m, DVec3::ZERO)?;
    let initial_mach = 2.0;
    let alpha_rad = 5.0_f64.to_radians();
    let speed_mps = initial_mach * environment.speed_of_sound_mps;
    let state = RigidBodyState::new(
        DVec3::new(0.0, 0.0, altitude_m),
        DVec3::new(
            speed_mps * alpha_rad.cos(),
            0.0,
            speed_mps * alpha_rad.sin(),
        ),
        DQuat::IDENTITY,
        DVec3::ZERO,
    )?;
    let properties = RigidBodyProperties::new(
        10_000.0,
        DMat3::from_diagonal(DVec3::new(80_000.0, 120_000.0, 100_000.0)),
    )?;
    let input = FlightStepInput::new(altitude_m, DVec3::new(0.0, 0.0, -atmosphere.gravity_mps2));
    let duration_s = 5.0;
    let (final_state, final_forces) = integrate_rigid_body_duration(
        &model, &geometry, atmosphere, state, properties, input, duration_s, 0.01,
    )?;
    let final_velocity_body =
        final_state.orientation_body_to_inertial.inverse() * final_state.velocity_inertial_mps;
    let final_speed_mps = final_state.velocity_inertial_mps.length();
    let final_alpha_deg = final_velocity_body
        .z
        .atan2(final_velocity_body.x)
        .to_degrees();
    Ok(TrajectoryResult {
        mach: final_forces.aero.mach,
        speed_mps: final_speed_mps,
        altitude_m: final_state.position_inertial_m.z,
        alpha_deg: final_alpha_deg,
        dynamic_pressure_pa: final_forces.aero.dynamic_pressure_pa,
    })
}

fn print_x15_trajectory(result: &TrajectoryResult) {
    println!(
        "trajectory,x15_like_6dof_proxy,duration_s=5.00,initial_mach=2.0000,final_mach={:.4},initial_speed_mps=596.080,final_speed_mps={:.3},initial_altitude_m=24384.0,final_altitude_m={:.1},final_q_pa={:.3},final_alpha_deg={:.4}",
        result.mach,
        result.speed_mps,
        result.altitude_m,
        result.dynamic_pressure_pa,
        result.alpha_deg,
    );
}

fn angle_of_attack_deg(case: &AeroCase) -> f64 {
    let relative_velocity = case.state.velocity_body_mps - case.environment.wind_velocity_body_mps;
    relative_velocity.z.atan2(relative_velocity.x).to_degrees()
}

fn normalized_coefficients(
    case: &AeroCase,
    result: &thessa_sim_core::AeroResult,
) -> Result<NormalizedCoefficients, Box<dyn Error>> {
    let reference_area_m2 = case
        .geometry
        .panels
        .iter()
        .map(|panel| panel.area_m2)
        .sum::<f64>();
    let reference_length_m = case
        .geometry
        .panels
        .iter()
        .map(|panel| panel.chord_m)
        .sum::<f64>()
        / case.geometry.panels.len() as f64;
    normalized_coefficients_with_reference(case, result, reference_area_m2, reference_length_m)
}

fn normalized_coefficients_with_reference(
    case: &AeroCase,
    result: &thessa_sim_core::AeroResult,
    reference_area_m2: f64,
    reference_length_m: f64,
) -> Result<NormalizedCoefficients, Box<dyn Error>> {
    let relative_velocity = case.state.velocity_body_mps - case.environment.wind_velocity_body_mps;
    let speed = relative_velocity.length();
    if speed <= f64::EPSILON {
        return Err("cannot normalize aerodynamic coefficients at zero speed".into());
    }
    let velocity_direction = relative_velocity / speed;
    let lift_direction = (DVec3::Z - velocity_direction * velocity_direction.z)
        .try_normalize()
        .ok_or("cannot construct a lift direction for this flow vector")?;
    let denominator = result.dynamic_pressure_pa * reference_area_m2;
    if denominator <= 0.0 || reference_length_m <= 0.0 {
        return Err("cannot normalize aerodynamic coefficients without dynamic pressure".into());
    }
    Ok(NormalizedCoefficients {
        lift: result.force_body_n.dot(lift_direction) / denominator,
        drag: -result.force_body_n.dot(velocity_direction) / denominator,
        pitch_moment: result.moment_body_nm.y / (denominator * reference_length_m),
    })
}

fn rocket_fin_case(mach: f64, alpha_rad: f64) -> Result<AeroCase, Box<dyn Error>> {
    let environment = AeroEnvironment::standard_sea_level();
    let speed = mach * environment.speed_of_sound_mps;
    let root_chord_m: f64 = 0.5;
    let tip_chord_m: f64 = 0.2;
    let fin_span_m: f64 = 0.25;
    let fin_area_m2 = 0.5 * (root_chord_m + tip_chord_m) * fin_span_m;
    let fin_position = DVec3::new(1.5, 0.0, 0.0);
    let sweep_length_m = root_chord_m - tip_chord_m;
    let sweep_angle_rad =
        ((sweep_length_m + 0.5 * tip_chord_m - 0.5 * root_chord_m) / fin_span_m).atan();
    let fin_cp_offset_m = (sweep_length_m / 3.0)
        * ((root_chord_m + 2.0 * tip_chord_m) / (root_chord_m + tip_chord_m))
        + (1.0 / 6.0)
            * (root_chord_m + tip_chord_m
                - root_chord_m * tip_chord_m / (root_chord_m + tip_chord_m));
    let center_of_pressure = fin_position - DVec3::X * fin_cp_offset_m;
    let fin_aspect_ratio = 2.0 * fin_span_m.powi(2) / fin_area_m2;
    let lift_interference_factor = 1.0 + 1.0 / ((fin_span_m + 0.25) / 0.25);
    let panels = [
        (DVec3::Y, "+Y fin"),
        (DVec3::new(0.0, -1.0, 0.0), "-Y fin"),
        (DVec3::Z, "+Z fin"),
        (DVec3::new(0.0, 0.0, -1.0), "-Z fin"),
    ]
    .into_iter()
    .map(|(lift_axis, _name)| {
        AeroPanel::new(fin_position, DVec3::X, lift_axis, fin_area_m2, 0.5)?
            .with_planform(
                fin_span_m,
                fin_aspect_ratio,
                sweep_angle_rad,
                lift_interference_factor,
            )?
            .with_center_of_pressure(center_of_pressure)
    })
    .collect::<Result<Vec<_>, _>>()?;
    Ok(AeroCase::new(
        AeroState::new(
            DVec3::new(speed * alpha_rad.cos(), 0.0, speed * alpha_rad.sin()),
            DVec3::ZERO,
        ),
        environment,
        AeroGeometry::new(panels)?,
    )?)
}

fn python_candidates() -> Vec<String> {
    let mut candidates = Vec::new();
    if let Ok(configured) = std::env::var("THESSA_AERO_PYTHON") {
        candidates.push(configured);
    }
    candidates.extend(
        [
            ".venv-aero/bin/python",
            "../../.venv-aero/bin/python",
            "python",
        ]
        .into_iter()
        .map(String::from),
    );
    candidates
}

fn python_module_available(python: &str, module: &str) -> bool {
    let import_statement = format!("import {module}");
    Command::new(python)
        .args(["-c", &import_statement])
        .output()
        .map(|output| output.status.success())
        .unwrap_or(false)
}

fn python_executable() -> Option<String> {
    python_candidates().into_iter().find(|candidate| {
        python_module_available(candidate, "jsbsim")
            || python_module_available(candidate, "rocketpy")
    })
}

fn reference_result(
    python: &str,
    reference: &str,
    model: &str,
    mach: f64,
    alpha_deg: f64,
) -> Result<ReferenceResult, Box<dyn Error>> {
    let output = Command::new(python)
        .arg("-c")
        .arg(REFERENCE_SCRIPT)
        .arg("--reference")
        .arg(reference)
        .arg("--model")
        .arg(model)
        .arg("--mach")
        .arg(mach.to_string())
        .arg("--alpha-deg")
        .arg(alpha_deg.to_string())
        .output()?;
    let stdout = String::from_utf8_lossy(&output.stdout);
    let result_line = stdout
        .lines()
        .find(|line| line.starts_with("result,"))
        .ok_or_else(|| {
            format!(
                "{reference} produced no result line (status {}, stderr: {})",
                output.status,
                String::from_utf8_lossy(&output.stderr).trim()
            )
        })?;
    let fields = result_line.split(',').collect::<Vec<_>>();
    if fields.len() != 10 {
        return Err(format!("invalid reference result: {result_line}").into());
    }
    if !output.status.success() {
        return Err(format!(
            "{reference} failed with status {}: {result_line}",
            output.status
        )
        .into());
    }
    Ok(ReferenceResult {
        engine: fields[1].to_string(),
        model: fields[2].to_string(),
        mach: parse_finite_float(fields[3], "reference Mach")?,
        alpha_deg: parse_finite_float(fields[4], "reference AoA")?,
        lift: parse_float(fields[5], "reference CL")?,
        drag: parse_float(fields[6], "reference CD")?,
        pitch_moment: parse_float(fields[7], "reference Cm")?,
        lift_slope: parse_float(fields[8], "reference CL_alpha")?,
        center_of_pressure_m: parse_float(fields[9], "reference CP")?,
    })
}

fn parse_float(value: &str, name: &str) -> Result<f64, Box<dyn Error>> {
    value
        .parse::<f64>()
        .map_err(|error| format!("{name} is not numeric ({value:?}): {error}").into())
}

fn parse_finite_float(value: &str, name: &str) -> Result<f64, Box<dyn Error>> {
    let parsed = parse_float(value, name)?;
    if !parsed.is_finite() {
        return Err(format!("{name} is non-finite: {value:?}").into());
    }
    Ok(parsed)
}

fn ensure_finite_result(
    case_name: &str,
    result: &thessa_sim_core::AeroResult,
) -> Result<(), Box<dyn Error>> {
    let finite = result.dynamic_pressure_pa.is_finite()
        && result.mach.is_finite()
        && result.reynolds_number.is_finite()
        && result.force_body_n.is_finite()
        && result.moment_body_nm.is_finite();
    if !finite {
        return Err(format!("{case_name} produced a non-finite aero result").into());
    }
    Ok(())
}

fn ensure_finite_coefficients(
    case_name: &str,
    coefficients: &NormalizedCoefficients,
) -> Result<(), Box<dyn Error>> {
    if !coefficients.lift.is_finite()
        || !coefficients.drag.is_finite()
        || !coefficients.pitch_moment.is_finite()
    {
        return Err(format!("{case_name} produced non-finite normalized coefficients").into());
    }
    Ok(())
}

fn ensure_finite_trajectory(
    case_name: &str,
    trajectory: &TrajectoryResult,
) -> Result<(), Box<dyn Error>> {
    if !trajectory.mach.is_finite()
        || !trajectory.speed_mps.is_finite()
        || !trajectory.altitude_m.is_finite()
        || !trajectory.alpha_deg.is_finite()
        || !trajectory.dynamic_pressure_pa.is_finite()
    {
        return Err(format!("{case_name} produced a non-finite trajectory result").into());
    }
    Ok(())
}

fn reference_trajectory_result(
    python: &str,
    model: &str,
    mach: f64,
    duration_s: f64,
) -> Result<TrajectoryResult, Box<dyn Error>> {
    let output = Command::new(python)
        .arg("-c")
        .arg(REFERENCE_SCRIPT)
        .arg("--reference")
        .arg("jsbsim")
        .arg("--model")
        .arg(model)
        .arg("--mach")
        .arg(mach.to_string())
        .arg("--alpha-deg")
        .arg("5.0")
        .arg("--trajectory")
        .arg("--duration-s")
        .arg(duration_s.to_string())
        .output()?;
    let stdout = String::from_utf8_lossy(&output.stdout);
    let result_line = stdout
        .lines()
        .find(|line| line.starts_with("trajectory_result,"))
        .ok_or_else(|| {
            format!(
                "{model} trajectory produced no result line (status {}, stderr: {})",
                output.status,
                String::from_utf8_lossy(&output.stderr).trim()
            )
        })?;
    if !output.status.success() {
        return Err(format!("{model} trajectory failed with status {}", output.status).into());
    }
    let fields = result_line.split(',').collect::<Vec<_>>();
    if fields.len() != 8 {
        return Err(format!("invalid trajectory result: {result_line}").into());
    }
    Ok(TrajectoryResult {
        mach: parse_finite_float(fields[3], "trajectory Mach")?,
        speed_mps: parse_finite_float(fields[4], "trajectory speed")?,
        altitude_m: parse_finite_float(fields[5], "trajectory altitude")?,
        alpha_deg: parse_finite_float(fields[6], "trajectory AoA")?,
        dynamic_pressure_pa: parse_finite_float(fields[7], "trajectory dynamic pressure")?,
    })
}

fn print_comparison(
    case_name: &str,
    reference: &ReferenceResult,
    metric: &str,
    ours: f64,
    expected: f64,
) {
    assert_finite_comparison(case_name, metric, ours, expected);
    let absolute_error = (ours - expected).abs();
    let relative_error_pct = if expected.abs() > 1.0e-12 {
        100.0 * absolute_error / expected.abs()
    } else {
        f64::NAN
    };
    println!(
        "comparison,{case_name},{}/{},{:.6},{:.6},{metric},{ours:.9},{expected:.9},{absolute_error:.9},{relative_error_pct:.3}",
        reference.engine, reference.model, reference.mach, reference.alpha_deg,
    );
}

fn print_scalar_comparison(case_name: &str, engine: &str, metric: &str, ours: f64, expected: f64) {
    assert_finite_comparison(case_name, metric, ours, expected);
    let absolute_error = (ours - expected).abs();
    let relative_error_pct = if expected.abs() > 1.0e-12 {
        100.0 * absolute_error / expected.abs()
    } else {
        f64::NAN
    };
    println!(
        "comparison,{case_name},{engine},0.0,0.0,{metric},{ours:.9},{expected:.9},{absolute_error:.9},{relative_error_pct:.3}"
    );
}

fn assert_finite_comparison(case_name: &str, metric: &str, ours: f64, expected: f64) {
    assert!(
        ours.is_finite() && expected.is_finite(),
        "non-finite comparison for {case_name} {metric}: ours={ours:?}, reference={expected:?}"
    );
}

fn probe(executable: &str) -> bool {
    Command::new(executable)
        .arg("--version")
        .output()
        .map(|output| output.status.success())
        .unwrap_or(false)
}

fn probe_python_module(module: &str) -> bool {
    python_candidates()
        .into_iter()
        .any(|executable| python_module_available(&executable, module))
}
