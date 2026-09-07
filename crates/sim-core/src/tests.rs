use glam::DVec3;

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
