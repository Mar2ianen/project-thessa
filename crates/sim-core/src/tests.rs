use glam::{DQuat, DVec3};
use std::io::Cursor;

use super::*;

fn central_ephemeris(mu: f64) -> BakedEphemeris {
    BakedEphemeris::new(
        "TEST_EPOCH",
        vec![BakedBody::fixed(BodyId(0), "central", mu, 0.0)],
    )
    .expect("valid central test ephemeris")
}

fn two_body_binary(mu_primary: f64, mu_secondary: f64, separation_m: f64) -> BakedEphemeris {
    let total_mu = mu_primary + mu_secondary;
    let mean_motion = (total_mu / separation_m.powi(3)).sqrt();
    let primary_orbit = KeplerOrbit::new(
        total_mu,
        separation_m * mu_secondary / total_mu,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
    .and_then(|orbit| orbit.with_mean_motion(mean_motion))
    .expect("valid primary orbit");
    let secondary_orbit = KeplerOrbit::new(
        total_mu,
        separation_m * mu_primary / total_mu,
        0.0,
        0.0,
        0.0,
        0.0,
        std::f64::consts::PI,
    )
    .and_then(|orbit| orbit.with_mean_motion(mean_motion))
    .expect("valid secondary orbit");
    BakedEphemeris::new(
        "TEST_BINARY_EPOCH",
        vec![
            BakedBody::synthetic_barycenter(BodyId(0), "barycenter", total_mu, None, None),
            BakedBody::orbital(
                BodyId(1),
                "primary",
                mu_primary,
                0.0,
                BodyId(0),
                primary_orbit,
            ),
            BakedBody::orbital(
                BodyId(2),
                "secondary",
                mu_secondary,
                0.0,
                BodyId(0),
                secondary_orbit,
            ),
        ],
    )
    .expect("valid binary ephemeris")
}

#[test]
fn kepler_orbit_returns_analytic_periodic_state() {
    let mu = 3.986_004_418e14;
    let radius = 7_000_000.0;
    let ephemeris = central_ephemeris(mu);
    let field = GravityField::from_ephemeris(&ephemeris);
    let initial = TestParticleState {
        position: DVec3::new(radius, 0.0, 0.0),
        velocity: DVec3::new(0.0, (mu / radius).sqrt(), 0.0),
    };
    let period = TAU * (radius.powi(3) / mu).sqrt();
    let result = propagate_adaptive(
        &field,
        initial,
        SimTime::EPOCH,
        period,
        AdaptiveIntegratorConfig {
            initial_step_s: 30.0,
            min_step_s: 1.0e-5,
            max_step_s: 120.0,
            absolute_position_tolerance_m: 1.0e-4,
            absolute_velocity_tolerance_mps: 1.0e-7,
            relative_tolerance: 1.0e-11,
            max_steps: 100_000,
        },
    )
    .expect("circular orbit should propagate");
    assert!((result.end_time.seconds() - period).abs() < 1.0e-9);
    assert!(result.state.position.distance(initial.position) < 2.0);
    assert!(result.state.velocity.distance(initial.velocity) < 1.0e-3);
}

#[test]
fn eccentric_kepler_orbit_matches_analytic_periapsis_after_one_period() {
    let mu = 3.986_004_418e14;
    let semi_major_axis = 10_000_000.0;
    let eccentricity = 0.6;
    let orbit = KeplerOrbit::new(mu, semi_major_axis, eccentricity, 0.0, 0.0, 0.0, 0.0)
        .expect("valid eccentric orbit");
    let (position, velocity) = orbit
        .state_relative_at(SimTime::EPOCH)
        .expect("analytic periapsis");
    let ephemeris = central_ephemeris(mu);
    let field = GravityField::from_ephemeris(&ephemeris);
    let result = propagate_adaptive(
        &field,
        TestParticleState { position, velocity },
        SimTime::EPOCH,
        orbit.period_s(),
        AdaptiveIntegratorConfig {
            initial_step_s: 20.0,
            min_step_s: 1.0e-6,
            max_step_s: 90.0,
            absolute_position_tolerance_m: 1.0e-3,
            absolute_velocity_tolerance_mps: 1.0e-6,
            relative_tolerance: 1.0e-10,
            max_steps: 100_000,
        },
    )
    .expect("eccentric orbit should propagate");
    assert!(result.state.position.distance(position) < 1.0);
    assert!(result.state.velocity.distance(velocity) < 1.0e-3);
}

#[test]
fn velocity_verlet_bounds_energy_error_on_long_coast() {
    let mu = 3.986_004_418e14;
    let radius = 7_000_000.0;
    let ephemeris = central_ephemeris(mu);
    let field = GravityField::from_ephemeris(&ephemeris);
    let initial = TestParticleState {
        position: DVec3::new(radius, 0.0, 0.0),
        velocity: DVec3::new(0.0, (mu / radius).sqrt(), 0.0),
    };
    let initial_energy = 0.5 * initial.velocity.length_squared() - mu / radius;
    let period = TAU * (radius.powi(3) / mu).sqrt();
    let result = propagate_velocity_verlet(
        &field,
        initial,
        SimTime::EPOCH,
        period * 50.0,
        VerletConfig {
            step_s: 20.0,
            max_steps: 1_000_000,
        },
    )
    .expect("Verlet coast should propagate");
    let final_energy = 0.5 * result.state.velocity.length_squared()
        + field
            .potential(result.state.position, result.end_time)
            .expect("potential should be finite");
    assert!((final_energy - initial_energy).abs() / initial_energy.abs() < 1.0e-6);
}

#[test]
fn restricted_three_body_l4_has_near_zero_residual() {
    let mu_primary = 1.0e14;
    let mu_secondary = 1.0e13;
    let separation = 1.0e7;
    let ephemeris = two_body_binary(mu_primary, mu_secondary, separation);
    let field = GravityField::from_ephemeris(&ephemeris);
    let primary = ephemeris
        .body_state(BodyId(1), SimTime::EPOCH)
        .expect("primary state");
    let secondary = ephemeris
        .body_state(BodyId(2), SimTime::EPOCH)
        .expect("secondary state");
    let line = secondary.position_inertial - primary.position_inertial;
    let separation = line.length();
    let perpendicular = DVec3::new(-line.y, line.x, 0.0).normalize();
    let l4_position = (primary.position_inertial + secondary.position_inertial) * 0.5
        + perpendicular * separation * (3.0_f64.sqrt() * 0.5);
    let angular_rate = (mu_primary + mu_secondary).sqrt() / separation.powf(1.5);
    let expected_acceleration = -l4_position * angular_rate.powi(2);
    let actual_acceleration = field
        .acceleration(l4_position, SimTime::EPOCH)
        .expect("L4 gravity should be finite");
    assert!(
        (actual_acceleration - expected_acceleration).length() / actual_acceleration.length()
            < 1.0e-12
    );
}

#[test]
fn moving_secondary_enables_three_body_energy_exchange() {
    let mu_primary = 1.0e14;
    let mu_secondary = 1.0e12;
    let separation = 1.0e8;
    let ephemeris = two_body_binary(mu_primary, mu_secondary, separation);
    let field = GravityField::from_ephemeris(&ephemeris);
    let initial = TestParticleState {
        position: DVec3::new(1.8e8, -2.5e7, 0.0),
        velocity: DVec3::new(120.0, 560.0, 0.0),
    };
    let initial_energy =
        0.5 * initial.velocity.length_squared() - mu_primary / initial.position.length();
    let secondary_period = TAU * (separation.powi(3) / (mu_primary + mu_secondary)).sqrt();
    let result = propagate_adaptive(
        &field,
        initial,
        SimTime::EPOCH,
        secondary_period * 1.5,
        AdaptiveIntegratorConfig {
            initial_step_s: 100.0,
            min_step_s: 1.0e-5,
            max_step_s: 2_000.0,
            absolute_position_tolerance_m: 1.0e-2,
            absolute_velocity_tolerance_mps: 1.0e-5,
            relative_tolerance: 1.0e-9,
            max_steps: 100_000,
        },
    )
    .expect("three-body trajectory should propagate");
    let final_energy =
        0.5 * result.state.velocity.length_squared() - mu_primary / result.state.position.length();
    assert!((final_energy - initial_energy).abs() > 1.0e-7);
}

#[test]
fn ordered_impulsive_burn_schedule_is_deterministic() {
    let mu = 3.986_004_418e14;
    let ephemeris = central_ephemeris(mu);
    let field = GravityField::from_ephemeris(&ephemeris);
    let initial = TestParticleState {
        position: DVec3::new(7.0e6, 0.0, 0.0),
        velocity: DVec3::new(0.0, (mu / 7.0e6).sqrt(), 0.0),
    };
    let burns = [
        ImpulsiveBurn {
            time_s: 900.0,
            delta_v_mps: DVec3::new(0.0, 25.0, 3.0),
        },
        ImpulsiveBurn {
            time_s: 2_400.0,
            delta_v_mps: DVec3::new(-12.0, 0.0, 5.0),
        },
        ImpulsiveBurn {
            time_s: 5_000.0,
            delta_v_mps: DVec3::new(18.0, -8.0, -4.0),
        },
    ];
    let config = AdaptiveIntegratorConfig {
        max_step_s: 180.0,
        ..AdaptiveIntegratorConfig::default()
    };
    let first =
        propagate_adaptive_with_burns(&field, initial, SimTime::EPOCH, 8_000.0, &burns, config)
            .expect("burn schedule should propagate");
    let second =
        propagate_adaptive_with_burns(&field, initial, SimTime::EPOCH, 8_000.0, &burns, config)
            .expect("burn schedule replay should propagate");
    assert_eq!(first, second);
    assert_eq!(first.end_time, SimTime::EPOCH.offset(8_000.0));
    assert!(first.state.position.is_finite());
    assert!(first.state.velocity.is_finite());
}

#[test]
fn replay_and_parallel_batch_are_deterministic_and_ordered() {
    let ephemeris = central_ephemeris(3.986_004_418e14);
    let field = GravityField::from_ephemeris(&ephemeris);
    let initial = TestParticleState {
        position: DVec3::new(8.0e6, 2.0e6, 0.0),
        velocity: DVec3::new(-500.0, 7_000.0, 10.0),
    };
    let config = AdaptiveIntegratorConfig {
        initial_step_s: 17.0,
        min_step_s: 1.0e-6,
        max_step_s: 120.0,
        absolute_position_tolerance_m: 1.0e-3,
        absolute_velocity_tolerance_mps: 1.0e-6,
        relative_tolerance: 1.0e-10,
        max_steps: 100_000,
    };
    let first = propagate_adaptive(&field, initial, SimTime::EPOCH, 10_000.0, config)
        .expect("first replay");
    let second = propagate_adaptive(&field, initial, SimTime::EPOCH, 10_000.0, config)
        .expect("second replay");
    assert_eq!(first, second);

    let positions = vec![
        DVec3::new(8.0e6, 0.0, 0.0),
        DVec3::new(0.0, 9.0e6, 0.0),
        DVec3::new(-10.0e6, 0.0, 1.0e6),
    ];
    let batch = field
        .accelerations(&positions, SimTime::EPOCH)
        .expect("batch gravity");
    for (position, acceleration) in positions.iter().zip(batch) {
        assert_eq!(
            acceleration,
            field.acceleration(*position, SimTime::EPOCH).unwrap()
        );
    }
}

#[test]
fn explicit_state_vector_keeps_frame_label() {
    let state = StateVector::new(
        DVec3::new(1.0, 2.0, 3.0),
        DVec3::new(4.0, 5.0, 6.0),
        ReferenceFrame::BodyCenteredInertial(BodyId(7)),
    );
    assert_eq!(state.frame, ReferenceFrame::BodyCenteredInertial(BodyId(7)));
}

fn aero_test_case(speed_mps: f64, angle_of_attack_deg: f64) -> AeroCase {
    let panel = AeroPanel::flat_plate(DVec3::ZERO, 10.0, 2.0).expect("valid test panel");
    let alpha = angle_of_attack_deg.to_radians();
    let state = AeroState::new(
        DVec3::new(speed_mps, 0.0, speed_mps * alpha.tan()),
        DVec3::ZERO,
    );
    AeroCase::new(
        state,
        AeroEnvironment::standard_sea_level(),
        AeroGeometry::new(vec![panel]).expect("valid test geometry"),
    )
    .expect("valid test case")
}

#[test]
fn standard_atmosphere_matches_sea_level_and_layer_boundaries() {
    let atmosphere = AtmosphereConfig::default();
    let sea_level = atmosphere.sample(0.0).expect("sea-level sample");
    assert!((sea_level.temperature_k - 288.15).abs() < 1.0e-12);
    assert!((sea_level.pressure_pa - 101_325.0).abs() < 1.0e-9);
    assert!((sea_level.density_kg_m3 - 1.225_000_018).abs() < 1.0e-6);
    assert!((sea_level.speed_of_sound_mps - 340.293_988).abs() < 1.0e-3);
    assert!((sea_level.dynamic_viscosity_pa_s - 1.789_297_626e-5).abs() < 1.0e-10);

    let below_tropopause = atmosphere.sample(10_999.0).expect("below boundary");
    let above_tropopause = atmosphere.sample(11_001.0).expect("above boundary");
    assert!((below_tropopause.temperature_k - above_tropopause.temperature_k).abs() < 0.02);
    assert!((below_tropopause.pressure_pa - above_tropopause.pressure_pa).abs() < 10.0);
    assert!(above_tropopause.density_kg_m3 < sea_level.density_kg_m3);
    assert!(above_tropopause.speed_of_sound_mps < sea_level.speed_of_sound_mps);
}

#[test]
fn custom_atmosphere_propagates_world_sea_level_state_into_upper_layers() {
    let atmosphere =
        AtmosphereConfig::new(300.0, 90_000.0, 300.0, 1.3, 8.5).expect("valid custom atmosphere");
    let sea_level = atmosphere.sample(0.0).expect("custom sea level");
    let high_altitude = atmosphere.sample(20_000.0).expect("custom upper layer");
    assert!((sea_level.temperature_k - 300.0).abs() < 1.0e-12);
    assert!((sea_level.pressure_pa - 90_000.0).abs() < 1.0e-9);
    assert!(high_altitude.temperature_k < sea_level.temperature_k);
    assert!(high_altitude.pressure_pa < sea_level.pressure_pa);
    assert!(high_altitude.speed_of_sound_mps.is_finite());
}

#[test]
fn atmosphere_builds_supersonic_aero_environment_from_altitude() {
    let atmosphere = AtmosphereConfig::default();
    let environment = atmosphere
        .aero_environment(20_000.0, DVec3::ZERO)
        .expect("valid aero environment");
    let sample = atmosphere.sample(20_000.0).expect("valid sample");
    assert!((environment.density_kg_m3 - sample.density_kg_m3).abs() < 1.0e-15);
    assert!((environment.speed_of_sound_mps - sample.speed_of_sound_mps).abs() < 1.0e-12);
    let mach = sample.mach(680.0).expect("valid speed");
    assert!(mach > 2.2);
    assert!(mach < 2.4);
}

#[test]
fn aero_csv_table_imports_shuffled_mach_alpha_grid_and_cm() {
    let csv = "# mach,alpha_deg,cl,cd,cy,cm\n\
        2.0,5.0,0.4,0.08,0.0,-0.02\n\
        0.9,5.0,0.5,0.04,0.0,-0.03\n\
        2.0,-5.0,-0.4,0.08,0.0,0.02\n\
        0.9,-5.0,-0.5,0.04,0.0,0.03\n";
    let table = AeroCoefficientTable::from_csv(Cursor::new(csv)).expect("valid polar CSV");
    assert_eq!(table.mach_grid, vec![0.9, 2.0]);
    assert_eq!(table.alpha_grid_rad.len(), 2);
    let sample = table.sample(1.45, 0.0);
    assert!(sample.lift.abs() < 1.0e-12);
    assert!((sample.drag - 0.06).abs() < 1.0e-12);
    assert!(sample.pitching_moment.abs() < 1.0e-12);
}

#[test]
fn rigid_body_step_combines_aero_environment_extra_force_and_gravity() {
    let panel = AeroPanel::flat_plate(DVec3::ZERO, 10.0, 2.0).expect("valid aircraft panel");
    let geometry = AeroGeometry::new(vec![panel]).expect("valid geometry");
    let model = PanelAeroModel::new(AeroConfig {
        base_drag_coefficient: 0.0,
        induced_drag_factor: 0.0,
        wave_drag_coefficient: 0.0,
        ..AeroConfig::default()
    })
    .expect("valid aero model");
    let state = RigidBodyState::stationary(DVec3::ZERO);
    let state = RigidBodyState::new(
        state.position_inertial_m,
        DVec3::new(100.0, 0.0, 0.0),
        state.orientation_body_to_inertial,
        state.angular_velocity_body_rps,
    )
    .expect("valid flight state");
    let properties =
        RigidBodyProperties::new(1_000.0, glam::DMat3::from_diagonal(DVec3::splat(100.0)))
            .expect("valid mass properties");
    let mut input = FlightStepInput::new(0.0, DVec3::new(0.0, 0.0, -9.80665));
    input.extra_force_body_n = DVec3::new(1_000.0, 0.0, 0.0);
    let (next, forces) = integrate_rigid_body_step(
        &model,
        &geometry,
        AtmosphereConfig::default(),
        state,
        properties,
        input,
        0.1,
    )
    .expect("valid flight step");
    assert!((forces.acceleration_inertial_mps2.x - 1.0).abs() < 1.0e-12);
    assert!((forces.acceleration_inertial_mps2.z + 9.80665).abs() < 1.0e-12);
    assert!((next.velocity_inertial_mps.x - 100.1).abs() < 1.0e-12);
    assert!((next.velocity_inertial_mps.z + 0.980665).abs() < 1.0e-12);
    assert!((next.position_inertial_m.x - 10.01).abs() < 1.0e-12);
    assert!((next.position_inertial_m.z + 0.0980665).abs() < 1.0e-12);
    assert_eq!(next.orientation_body_to_inertial, DQuat::IDENTITY);
}

#[test]
fn atmosphere_rotation_adds_body_frame_air_velocity() {
    let atmosphere = AtmosphereConfig {
        body_rotation_rad_s: DVec3::new(0.0, 0.0, 1.0e-3),
        ..AtmosphereConfig::default()
    };
    let wind = atmosphere
        .rotating_wind_velocity_body_mps(DVec3::new(0.0, 1_000.0, 0.0), DVec3::ZERO)
        .expect("finite rotating wind");
    assert!((wind.x + 1.0).abs() < 1.0e-12);
    assert!(wind.y.abs() < 1.0e-12);
}

#[test]
fn aero_dynamic_pitch_damping_opposes_pitch_rate() {
    let panel = AeroPanel::flat_plate(DVec3::ZERO, 10.0, 2.0).expect("valid aircraft panel");
    let geometry = AeroGeometry::new(vec![panel]).expect("valid geometry");
    let model = PanelAeroModel::new(AeroConfig {
        base_drag_coefficient: 0.0,
        induced_drag_factor: 0.0,
        wave_drag_coefficient: 0.0,
        side_force_slope_per_rad: 0.0,
        pitch_damping_coefficient: -1.0,
        ..AeroConfig::default()
    })
    .expect("valid aero model");
    let state = AeroState::new(DVec3::new(100.0, 0.0, 0.0), DVec3::new(0.0, 0.2, 0.0));
    let case = AeroCase::new(state, AeroEnvironment::standard_sea_level(), geometry)
        .expect("valid aero case");
    let result = model.evaluate(&case).expect("finite damping result");
    assert!(result.moment_body_nm.y < 0.0);
}

#[test]
fn aero_signed_stabilizer_lift_restores_positive_alpha() {
    let panel = AeroPanel::flat_plate(DVec3::new(-4.0, 0.0, 0.0), 4.0, 1.2)
        .and_then(|panel| panel.with_lift_sign(-1.0))
        .expect("valid stabilizer panel");
    let case = AeroCase::new(
        AeroState::new(
            DVec3::new(100.0, 0.0, 100.0 * 5.0_f64.to_radians().tan()),
            DVec3::ZERO,
        ),
        AeroEnvironment::standard_sea_level(),
        AeroGeometry::new(vec![panel]).expect("valid geometry"),
    )
    .expect("valid aero case");
    let model = PanelAeroModel::new(AeroConfig {
        base_drag_coefficient: 0.0,
        induced_drag_factor: 0.0,
        wave_drag_coefficient: 0.0,
        ..AeroConfig::default()
    })
    .expect("valid aero model");
    let result = model.evaluate(&case).expect("finite stabilizer result");
    assert!(result.force_body_n.z < 0.0);
    assert!(result.moment_body_nm.y < 0.0);
}

#[test]
fn rigid_body_duration_uses_bounded_deterministic_substeps() {
    let panel = AeroPanel::flat_plate(DVec3::ZERO, 10.0, 2.0).expect("valid aircraft panel");
    let geometry = AeroGeometry::new(vec![panel]).expect("valid geometry");
    let model = PanelAeroModel::new(AeroConfig {
        base_drag_coefficient: 0.0,
        induced_drag_factor: 0.0,
        wave_drag_coefficient: 0.0,
        ..AeroConfig::default()
    })
    .expect("valid aero model");
    let state = RigidBodyState::new(
        DVec3::ZERO,
        DVec3::new(100.0, 0.0, 0.0),
        DQuat::IDENTITY,
        DVec3::ZERO,
    )
    .expect("valid state");
    let properties =
        RigidBodyProperties::new(1_000.0, glam::DMat3::from_diagonal(DVec3::splat(100.0)))
            .expect("valid properties");
    let mut input = FlightStepInput::new(0.0, DVec3::ZERO);
    input.extra_force_body_n = DVec3::new(1_000.0, 0.0, 0.0);
    let (one, _) = integrate_rigid_body_duration(
        &model,
        &geometry,
        AtmosphereConfig::default(),
        state,
        properties,
        input,
        0.1,
        0.02,
    )
    .expect("valid duration");
    let (two, _) = integrate_rigid_body_duration(
        &model,
        &geometry,
        AtmosphereConfig::default(),
        state,
        properties,
        input,
        0.1,
        0.02,
    )
    .expect("deterministic duration");
    assert_eq!(one, two);
    assert!((one.position_inertial_m.x - 10.006).abs() < 1.0e-12);
}

#[test]
fn vehicle_definition_supports_arbitrary_surfaces_and_control_channels() {
    let left_wing =
        AeroPanel::flat_plate(DVec3::new(0.0, -2.0, 0.0), 5.0, 2.0).expect("valid left wing");
    let right_wing =
        AeroPanel::flat_plate(DVec3::new(0.0, 2.0, 0.0), 5.0, 2.0).expect("valid right wing");
    let geometry = AeroGeometry::new(vec![left_wing, right_wing]).expect("valid geometry");
    let properties =
        RigidBodyProperties::new(1_000.0, glam::DMat3::from_diagonal(DVec3::splat(100.0)))
            .expect("valid properties");
    let elevator = ControlSurfaceDefinition::new(
        "elevator",
        vec![0, 1],
        -20.0_f64.to_radians(),
        15.0_f64.to_radians(),
    )
    .expect("valid elevator");
    let mut vehicle =
        VehicleDefinition::new("arbitrary-aircraft", geometry, properties, vec![elevator])
            .expect("valid vehicle definition");
    vehicle
        .apply_control_inputs(&[0.5])
        .expect("valid control input");
    assert!(
        (vehicle.aero_geometry.panels[0].control_deflection_rad - 7.5_f64.to_radians()).abs()
            < 1.0e-12
    );
    assert!(
        (vehicle.aero_geometry.panels[1].control_deflection_rad - 7.5_f64.to_radians()).abs()
            < 1.0e-12
    );
    assert!(vehicle.apply_control_inputs(&[1.1]).is_err());
    assert!(vehicle.apply_control_inputs(&[]).is_err());
}

#[test]
fn aero_zero_flow_has_no_force_or_moment() {
    let case = aero_test_case(0.0, 0.0);
    let model = PanelAeroModel::new(AeroConfig::default()).expect("valid aero model");
    let result = model.evaluate(&case).expect("zero-flow result");
    assert_eq!(result.force_body_n, DVec3::ZERO);
    assert_eq!(result.moment_body_nm, DVec3::ZERO);
    assert_eq!(result.dynamic_pressure_pa, 0.0);
    assert_eq!(result.mach, 0.0);
}

#[test]
fn aero_drag_and_lift_have_expected_signs() {
    let case = aero_test_case(100.0, 5.0);
    let model = PanelAeroModel::new(AeroConfig::default()).expect("valid aero model");
    let result = model.evaluate_detailed(&case).expect("finite aero result");
    assert!(result.force_body_n.x < 0.0, "drag must oppose +X travel");
    assert!(
        result.force_body_n.z > 0.0,
        "positive AoA must produce +Z lift"
    );
    assert!(
        result
            .panel_loads
            .as_ref()
            .is_some_and(|loads| loads.len() == 1)
    );
}

#[test]
fn aero_dynamic_pressure_scales_with_speed_squared() {
    let environment = AeroEnvironment::new(1.0, 1.0e9, 1.0e-5, DVec3::ZERO);
    let geometry = AeroGeometry::new(vec![
        AeroPanel::flat_plate(DVec3::ZERO, 10.0, 2.0).expect("valid panel"),
    ])
    .expect("valid geometry");
    let make_case = |speed_mps| {
        AeroCase::new(
            AeroState::new(DVec3::new(speed_mps, 0.0, speed_mps * 0.05), DVec3::ZERO),
            environment,
            geometry.clone(),
        )
        .expect("valid case")
    };
    let model = PanelAeroModel::new(AeroConfig {
        wave_drag_coefficient: 0.0,
        ..AeroConfig::default()
    })
    .expect("valid aero model");
    let slow = model.evaluate(&make_case(100.0)).expect("slow result");
    let fast = model.evaluate(&make_case(200.0)).expect("fast result");
    assert!((fast.dynamic_pressure_pa / slow.dynamic_pressure_pa - 4.0).abs() < 1.0e-12);
    assert!((fast.force_body_n.length() / slow.force_body_n.length() - 4.0).abs() < 1.0e-8);
}

#[test]
fn aero_local_rotation_contributes_omega_cross_r_velocity() {
    let panel =
        AeroPanel::flat_plate(DVec3::new(0.0, 0.0, 2.0), 1.0, 1.0).expect("valid offset panel");
    let case = AeroCase::new(
        AeroState::new(DVec3::ZERO, DVec3::new(0.0, 0.1, 0.0)),
        AeroEnvironment::standard_sea_level(),
        AeroGeometry::new(vec![panel]).expect("valid geometry"),
    )
    .expect("valid case");
    let model = PanelAeroModel::new(AeroConfig::default()).expect("valid aero model");
    let result = model
        .evaluate_detailed(&case)
        .expect("finite rotating result");
    let load = &result.panel_loads.expect("detailed load")[0];
    assert!((load.local_velocity_body_mps - DVec3::new(0.2, 0.0, 0.0)).length() < 1.0e-12);
    assert!(load.dynamic_pressure_pa > 0.0);
}

#[test]
fn aero_finite_planform_and_center_of_pressure_are_geometry_driven() {
    let fin_area_m2: f64 = 0.5 * (0.5 + 0.2) * 0.25;
    let sweep_rad = 0.5404195002705843;
    let panel = AeroPanel::new(
        DVec3::new(1.5, 0.0, 0.0),
        DVec3::X,
        DVec3::Z,
        fin_area_m2,
        0.5,
    )
    .expect("valid fin")
    .with_planform(0.25, 1.4285714285714286, sweep_rad, 1.5)
    .expect("valid fin planform")
    .with_center_of_pressure(DVec3::new(1.2785714285714285, 0.0, 0.0))
    .expect("valid fin center of pressure");
    let environment = AeroEnvironment::standard_sea_level();
    let model = PanelAeroModel::new(AeroConfig::default()).expect("valid aero model");
    let make_case = |alpha_rad: f64| {
        AeroCase::new(
            AeroState::new(
                DVec3::new(
                    0.95 * environment.speed_of_sound_mps * alpha_rad.cos(),
                    0.0,
                    0.95 * environment.speed_of_sound_mps * alpha_rad.sin(),
                ),
                DVec3::ZERO,
            ),
            environment,
            AeroGeometry::new(vec![panel]).expect("valid geometry"),
        )
        .expect("valid case")
    };
    let delta_alpha = 1.0e-4;
    let plus = model
        .evaluate_detailed(&make_case(delta_alpha))
        .expect("positive fin result");
    let minus = model
        .evaluate_detailed(&make_case(-delta_alpha))
        .expect("negative fin result");
    let plus_load = plus.panel_loads.as_ref().expect("positive detail")[0];
    let minus_load = minus.panel_loads.as_ref().expect("negative detail")[0];
    let measured_slope =
        (plus_load.coefficients.lift - minus_load.coefficients.lift) / (2.0 * delta_alpha);
    let compressible_2d_slope = 2.0 * std::f64::consts::PI / 0.6;
    let correlation_parameter =
        2.0 * std::f64::consts::PI * 1.4285714285714286 / (compressible_2d_slope * sweep_rad.cos());
    let expected_slope = 1.5 * (compressible_2d_slope * correlation_parameter * sweep_rad.cos())
        / (2.0 + correlation_parameter * (1.0 + (2.0 / correlation_parameter).powi(2)).sqrt());
    assert!((measured_slope - expected_slope).abs() < 1.0e-6);
    assert!((-plus.moment_body_nm.y / plus.force_body_n.z - 1.2785714285714285).abs() < 1.0e-12);
}

#[test]
fn aero_remains_finite_through_transonic_and_supersonic_regimes() {
    let model = PanelAeroModel::new(AeroConfig::default()).expect("valid aero model");
    for mach in [0.0, 0.3, 0.8, 0.99, 1.0, 1.01, 1.2, 2.0, 5.0] {
        let mut case = aero_test_case(mach * 340.294, 7.0);
        case.environment.speed_of_sound_mps = 340.294;
        let result = model.evaluate(&case).expect("finite regime result");
        assert!(
            result.force_body_n.is_finite(),
            "non-finite force at M={mach}"
        );
        assert!(
            result.moment_body_nm.is_finite(),
            "non-finite moment at M={mach}"
        );
        assert!(result.mach.is_finite());
    }
}

#[test]
fn aero_supersonic_wave_drag_follows_linearized_thin_airfoil_theory() {
    let alpha_rad = 2.0_f64.to_radians();
    let mut case = aero_test_case(2.0 * 340.294, alpha_rad.to_degrees());
    case.environment.speed_of_sound_mps = 340.294;
    let model = PanelAeroModel::new(AeroConfig {
        base_drag_coefficient: 0.0,
        induced_drag_factor: 0.0,
        wave_drag_coefficient: 0.0,
        ..AeroConfig::default()
    })
    .expect("valid supersonic aero model");
    let result = model
        .evaluate_detailed(&case)
        .expect("supersonic aero result");
    let load = &result.panel_loads.expect("supersonic detail")[0];
    let expected_wave_drag = AeroConfig::default().supersonic_lift_slope_factor * alpha_rad.powi(2)
        / (result.mach.powi(2) - 1.0).sqrt();
    assert!((load.coefficients.drag - expected_wave_drag).abs() < 1.0e-12);
    assert!(load.coefficients.drag > 0.0);
}

#[test]
fn aero_supersonic_thickness_drag_exists_at_zero_lift() {
    let mut panel = AeroPanel::flat_plate(DVec3::ZERO, 20.0, 2.0)
        .expect("valid aircraft panel")
        .with_thickness_ratio(0.04)
        .expect("valid thickness ratio");
    panel.planform_aspect_ratio = 6.0;
    let environment = AeroEnvironment::standard_sea_level();
    let case = AeroCase::new(
        AeroState::new(
            DVec3::new(2.0 * environment.speed_of_sound_mps, 0.0, 0.0),
            DVec3::ZERO,
        ),
        environment,
        AeroGeometry::new(vec![panel]).expect("valid geometry"),
    )
    .expect("valid supersonic case");
    let model = PanelAeroModel::new(AeroConfig {
        base_drag_coefficient: 0.0,
        induced_drag_factor: 0.0,
        wave_drag_coefficient: 0.0,
        ..AeroConfig::default()
    })
    .expect("valid model");
    let load = &model
        .evaluate_detailed(&case)
        .expect("supersonic result")
        .panel_loads
        .expect("detailed load")[0];
    let expected = 4.0 * 0.04_f64.powi(2) / (3.0_f64).sqrt();
    assert!((load.coefficients.drag - expected).abs() < 1.0e-12);
    assert!(load.coefficients.drag > 0.0);
}

#[test]
fn aero_static_pitching_moment_is_separate_from_center_of_pressure() {
    let panel = AeroPanel::flat_plate(DVec3::ZERO, 10.0, 2.0).expect("valid aircraft panel");
    let environment = AeroEnvironment::standard_sea_level();
    let case = AeroCase::new(
        AeroState::new(DVec3::new(100.0, 0.0, 0.0), DVec3::ZERO),
        environment,
        AeroGeometry::new(vec![panel]).expect("valid geometry"),
    )
    .expect("valid aircraft case");
    let model = PanelAeroModel::new(AeroConfig {
        base_drag_coefficient: 0.0,
        induced_drag_factor: 0.0,
        wave_drag_coefficient: 0.0,
        pitching_moment_coefficient: -0.05,
        ..AeroConfig::default()
    })
    .expect("valid model");
    let result = model.evaluate(&case).expect("aircraft result");
    let expected_moment = 0.5 * environment.density_kg_m3 * 100.0_f64.powi(2) * 10.0 * 2.0 * -0.05;
    assert!((result.moment_body_nm.y - expected_moment).abs() < 1.0e-9);
}

#[test]
fn aero_swept_surface_uses_normal_mach_for_supersonic_branch() {
    let environment = AeroEnvironment::standard_sea_level();
    let make_case = |sweep_rad| {
        let panel = AeroPanel::flat_plate(DVec3::ZERO, 20.0, 2.0)
            .expect("valid aircraft panel")
            .with_planform(10.0, 5.0, sweep_rad, 1.0)
            .expect("valid planform");
        let alpha_rad = 5.0_f64.to_radians();
        AeroCase::new(
            AeroState::new(
                DVec3::new(
                    2.0 * environment.speed_of_sound_mps * alpha_rad.cos(),
                    0.0,
                    2.0 * environment.speed_of_sound_mps * alpha_rad.sin(),
                ),
                DVec3::ZERO,
            ),
            environment,
            AeroGeometry::new(vec![panel]).expect("valid geometry"),
        )
        .expect("valid swept-wing case")
    };
    let model = PanelAeroModel::new(AeroConfig {
        base_drag_coefficient: 0.0,
        induced_drag_factor: 0.0,
        wave_drag_coefficient: 0.0,
        ..AeroConfig::default()
    })
    .expect("valid model");
    let unswept = model
        .evaluate_detailed(&make_case(0.0))
        .expect("unswept result");
    let swept = model
        .evaluate_detailed(&make_case(45.0_f64.to_radians()))
        .expect("swept result");
    let unswept_load = unswept.panel_loads.expect("unswept detail")[0];
    let swept_load = swept.panel_loads.expect("swept detail")[0];
    assert!(swept_load.coefficients.drag < unswept_load.coefficients.drag);
    assert_ne!(swept_load.coefficients.lift, unswept_load.coefficients.lift);
    assert!(swept_load.coefficients.drag.is_finite());
}

#[test]
fn aero_post_stall_lift_is_bounded_and_continuous() {
    let model = PanelAeroModel::new(AeroConfig::default()).expect("valid aero model");
    let before = model
        .evaluate(&aero_test_case(120.0, 17.9))
        .expect("pre-stall result");
    let after = model
        .evaluate_detailed(&aero_test_case(120.0, 18.1))
        .expect("post-stall result");
    assert!(after.force_body_n.z.is_finite());
    assert!(after.force_body_n.z.abs() <= before.force_body_n.z.abs() * 1.1);
    let load = &after.panel_loads.expect("post-stall detail")[0];
    assert!(load.coefficients.lift.abs() <= AeroConfig::default().max_lift_coefficient);
}

#[test]
fn aero_table_bilinear_interpolation_is_deterministic() {
    let table = AeroCoefficientTable::new(
        vec![0.0, 2.0],
        vec![-0.1, 0.1],
        vec![
            AeroCoefficients {
                lift: -1.0,
                drag: 1.0,
                side_force: 0.0,
                pitching_moment: -0.1,
            },
            AeroCoefficients {
                lift: 1.0,
                drag: 1.0,
                side_force: 0.0,
                pitching_moment: 0.1,
            },
            AeroCoefficients {
                lift: -2.0,
                drag: 2.0,
                side_force: 0.0,
                pitching_moment: -0.2,
            },
            AeroCoefficients {
                lift: 2.0,
                drag: 2.0,
                side_force: 0.0,
                pitching_moment: 0.2,
            },
        ],
    )
    .expect("valid coefficient table");
    let sample = table.sample(1.0, 0.0);
    assert_eq!(sample.lift, 0.0);
    assert_eq!(sample.drag, 1.5);
    assert_eq!(sample.pitching_moment, 0.0);
}

#[test]
fn aero_batch_preserves_order_and_replay() {
    let cases = vec![
        aero_test_case(80.0, -3.0),
        aero_test_case(120.0, 0.0),
        aero_test_case(250.0, 8.0),
    ];
    let model = PanelAeroModel::new(AeroConfig::default()).expect("valid aero model");
    let first = evaluate_batch(&model, &cases);
    let second = evaluate_batch(&model, &cases);
    assert_eq!(first, second);
    for (case, result) in cases.iter().zip(first) {
        let expected = model.evaluate(case).expect("single result");
        assert_eq!(result.expect("batch result"), expected);
    }
}

#[test]
fn x15_starter_profile_provides_surface_acceleration_margin() {
    let starter = X15StarterProfile::new().expect("X-15 starter profile");
    let model = PanelAeroModel::new(starter.aero_config).expect("X-15 aero model");
    let state = RigidBodyState::new(
        DVec3::ZERO,
        DVec3::new(
            starter.launch_forward_speed_mps,
            0.0,
            starter.launch_upward_speed_mps,
        ),
        DQuat::IDENTITY,
        DVec3::ZERO,
    )
    .expect("X-15 launch state");
    let mut input = FlightStepInput::new(0.0, DVec3::new(0.0, 0.0, -9.80665));
    input.extra_force_body_n = starter.thrust_body_n(0.35).expect("valid throttle");
    let forces = evaluate_flight_forces(
        &model,
        &starter.vehicle.aero_geometry,
        AtmosphereConfig::default(),
        state,
        starter.vehicle.mass_properties,
        input,
    )
    .expect("finite X-15 launch forces");
    assert!(forces.acceleration_inertial_mps2.is_finite());
    assert!(forces.acceleration_inertial_mps2.x > 5.0);
    assert!(forces.total_force_body_n.x > 0.0);
}

#[test]
fn aero_drag_uses_air_relative_speed_and_opposes_motion() {
    let environment = AeroEnvironment::new(1.0, 340.0, 1.8e-5, DVec3::new(50.0, 0.0, 0.0));
    let geometry = AeroGeometry::new(vec![
        AeroPanel::flat_plate(DVec3::ZERO, 10.0, 2.0).expect("valid panel"),
    ])
    .expect("valid geometry");
    let case = AeroCase::new(
        AeroState::new(DVec3::new(200.0, 0.0, 0.0), DVec3::ZERO),
        environment,
        geometry,
    )
    .expect("valid relative-flow case");
    let model = PanelAeroModel::new(AeroConfig {
        induced_drag_factor: 0.0,
        wave_drag_coefficient: 0.0,
        ..AeroConfig::default()
    })
    .expect("valid aero model");
    let result = model.evaluate(&case).expect("finite drag result");
    assert!((result.dynamic_pressure_pa - 11_250.0).abs() < 1.0e-9);
    assert!(result.force_body_n.x < 0.0);
}

#[test]
fn aero_transonic_drag_rise_is_bounded_and_continuous() {
    let model = PanelAeroModel::new(AeroConfig::default()).expect("valid aero model");
    let results = [0.99, 1.0, 1.01].map(|mach| {
        let mut case = aero_test_case(mach * 340.0, 5.0);
        case.environment.speed_of_sound_mps = 340.0;
        model.evaluate(&case).expect("finite transonic result")
    });
    for result in &results {
        assert!(result.force_body_n.is_finite());
        assert!(result.dynamic_pressure_pa.is_finite());
    }
    let magnitudes = results.map(|result| result.force_body_n.length());
    assert!(magnitudes[1] / magnitudes[0] < 1.4);
    assert!(magnitudes[2] / magnitudes[1] < 1.4);
}

#[test]
fn aero_rejects_finite_velocity_whose_magnitude_overflows() {
    let panel = AeroPanel::flat_plate(DVec3::ZERO, 1.0, 1.0).expect("valid panel");
    let case = AeroCase::new(
        AeroState::new(DVec3::splat(f64::MAX), DVec3::ZERO),
        AeroEnvironment::standard_sea_level(),
        AeroGeometry::new(vec![panel]).expect("valid geometry"),
    )
    .expect("components themselves are finite");
    let model = PanelAeroModel::new(AeroConfig::default()).expect("valid aero model");
    assert!(model.evaluate(&case).is_err());
}

#[test]
fn atmosphere_rejects_non_finite_derived_pressure_and_mach() {
    let mut sample = AtmosphereConfig::default()
        .sample(0.0)
        .expect("valid sample");
    assert!(sample.dynamic_pressure_pa(f64::MAX).is_err());
    sample.speed_of_sound_mps = 1.0e-300;
    assert!(sample.mach(f64::MAX).is_err());
}
