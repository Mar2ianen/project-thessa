use super::*;

// ---- Finite-thrust propagation (integrator thrust arcs) ----

fn circular_state(mu: f64, radius: f64) -> TestParticleState {
    TestParticleState {
        position: DVec3::new(radius, 0.0, 0.0),
        velocity: DVec3::new(0.0, (mu / radius).sqrt(), 0.0),
    }
}

fn orbital_energy(mu: f64, state: TestParticleState) -> f64 {
    state.velocity.length_squared() / 2.0 - mu / state.position.length()
}

/// The epoch sampled inside a finite burn must follow the schedule: a burn
/// that starts `start_s` after departure runs against the field at its true
/// absolute time, not shifted back to the departure epoch. Fixed central
/// fields cannot see the shift (that is how this once regressed); the
/// moving binary here rotates enough between the two epochs to separate
/// correct from shifted by metres. Same physical burn encoded twice —
/// scheduled later from `EPOCH`, or started fresh at `EPOCH + offset` —
/// must land on the same state.
#[test]
fn thrust_arc_gravity_epoch_follows_schedule_offset() {
    let ephemeris = two_body_binary(3.0e14, 1.0e14, 5.0e7);
    let field = GravityField::from_ephemeris(&ephemeris);
    let mu = 4.0e14;
    let initial = circular_state(mu, 1.1e9);
    let mass = 20_000.0;
    let config = AdaptiveIntegratorConfig::default();
    let offset = 600.0;
    let burn_s = 120.0;
    let arc = ThrustArc {
        start_s: offset,
        duration_s: burn_s,
        direction: ThrustDirection::Inertial(DVec3::Y),
        throttle_01: 1.0,
        thrust_n: 5_000.0,
        mass_flow_kgs: 0.1,
    };
    let scheduled = propagate_adaptive_with_thrust(
        &field,
        initial,
        mass,
        SimTime::EPOCH,
        offset + burn_s,
        &[arc],
        config,
    )
    .expect("scheduled arc propagates");
    // Identical burn, same absolute window, encoded with start_s = 0 from
    // the arc-start epoch. Coast to the arc start with the same ballistic
    // stepper the scheduled run uses internally.
    let coast = propagate_adaptive(&field, initial, SimTime::EPOCH, offset, config)
        .expect("coast to arc start");
    let arc_at_origin = ThrustArc {
        start_s: 0.0,
        ..arc
    };
    let shifted = propagate_adaptive_with_thrust(
        &field,
        coast.state,
        mass,
        SimTime::EPOCH.offset(offset),
        burn_s,
        &[arc_at_origin],
        config,
    )
    .expect("offset arc propagates");
    let position_gap = (scheduled.state.position - shifted.state.position).length();
    let velocity_gap = (scheduled.state.velocity - shifted.state.velocity).length();
    assert!(
        position_gap < 1.0e-6,
        "burn scheduled at +{offset}s drifted {position_gap:e} m from the same burn encoded at its own epoch"
    );
    assert!(velocity_gap < 1.0e-9, "velocity gap {velocity_gap:e} m/s");
}

#[test]
fn thrust_arc_depletes_mass_in_closed_form() {
    let mu = 1.2e17;
    let ephemeris = central_ephemeris(mu);
    let field = GravityField::from_ephemeris(&ephemeris);
    let arc = ThrustArc {
        start_s: 100.0,
        duration_s: 1_000.0,
        direction: ThrustDirection::Inertial(DVec3::Y),
        throttle_01: 0.8,
        thrust_n: 1_000.0,
        mass_flow_kgs: 0.05,
    };
    let result = propagate_adaptive_with_thrust(
        &field,
        circular_state(mu, 1.1e9),
        20_000.0,
        SimTime::EPOCH,
        2_000.0,
        &[arc],
        AdaptiveIntegratorConfig::default(),
    )
    .expect("thrust arc propagates");
    // Closed form (the integrator never touches mass): m = m0 - mdot*t.
    assert!((result.final_mass_kg - (20_000.0 - 0.05 * 0.8 * 1_000.0)).abs() < 1e-9);
    assert!((result.end_time.seconds() - (SimTime::EPOCH.seconds() + 2_000.0)).abs() < 1e-6);
    assert!(result.state.position.is_finite());
}

#[test]
fn dead_thrust_arc_matches_ballistic() {
    // Zero throttle over the WHOLE horizon: the thrust stepper must
    // reproduce the ballistic stepper (same tableau, +0.0 thrust) — guards
    // the duplicated core. (A mid-horizon dead arc restarts the stepper
    // at the boundary, so it only agrees to tolerance — see below.)
    let mu = 1.2e17;
    let ephemeris = central_ephemeris(mu);
    let field = GravityField::from_ephemeris(&ephemeris);
    let initial = circular_state(mu, 1.1e9);
    let arc = ThrustArc {
        start_s: 0.0,
        duration_s: 50_000.0,
        direction: ThrustDirection::Inertial(DVec3::Y),
        throttle_01: 0.0,
        thrust_n: 1_000.0,
        mass_flow_kgs: 0.05,
    };
    let thrust = propagate_adaptive_with_thrust(
        &field,
        initial,
        20_000.0,
        SimTime::EPOCH,
        50_000.0,
        &[arc],
        AdaptiveIntegratorConfig::default(),
    )
    .expect("dead arc propagates");
    let ballistic = propagate_adaptive(
        &field,
        initial,
        SimTime::EPOCH,
        50_000.0,
        AdaptiveIntegratorConfig::default(),
    )
    .expect("ballistic propagates");
    assert_eq!(thrust.state, ballistic.state);
    assert_eq!(thrust.final_mass_kg, 20_000.0);
}

#[test]
fn segmented_dead_arcs_match_ballistic_to_tolerance() {
    // Mid-horizon dead arcs restart the adaptive stepper at boundaries
    // (fresh initial step), so agreement is to integration tolerance —
    // this guards the segmentation plumbing, not the tableau.
    let mu = 1.2e17;
    let ephemeris = central_ephemeris(mu);
    let field = GravityField::from_ephemeris(&ephemeris);
    let initial = circular_state(mu, 1.1e9);
    let arcs = [
        ThrustArc {
            start_s: 500.0,
            duration_s: 10_000.0,
            direction: ThrustDirection::Inertial(DVec3::Y),
            throttle_01: 0.0,
            thrust_n: 1_000.0,
            mass_flow_kgs: 0.05,
        },
        ThrustArc {
            start_s: 20_000.0,
            duration_s: 5_000.0,
            direction: ThrustDirection::Prograde,
            throttle_01: 0.0,
            thrust_n: 1_000.0,
            mass_flow_kgs: 0.05,
        },
    ];
    let thrust = propagate_adaptive_with_thrust(
        &field,
        initial,
        20_000.0,
        SimTime::EPOCH,
        50_000.0,
        &arcs,
        AdaptiveIntegratorConfig::default(),
    )
    .expect("dead arcs propagate");
    let ballistic = propagate_adaptive(
        &field,
        initial,
        SimTime::EPOCH,
        50_000.0,
        AdaptiveIntegratorConfig::default(),
    )
    .expect("ballistic propagates");
    assert!(thrust.state.position.distance(ballistic.state.position) < 1.0);
    assert_eq!(thrust.final_mass_kg, 20_000.0);
}

#[test]
fn prograde_arc_gains_orbital_energy() {
    // Five-day full-throttle prograde arc: energy must rise (spiral out),
    // mass must match closed form exactly.
    let mu = 1.2e17;
    let ephemeris = central_ephemeris(mu);
    let field = GravityField::from_ephemeris(&ephemeris);
    let initial = circular_state(mu, 1.1e9);
    let duration = 5.0 * 86_400.0;
    let arc = ThrustArc {
        start_s: 0.0,
        duration_s: duration,
        direction: ThrustDirection::Prograde,
        throttle_01: 1.0,
        thrust_n: 2.0,
        mass_flow_kgs: 2.0 / 30_000.0,
    };
    let result = propagate_adaptive_with_thrust(
        &field,
        initial,
        2_000.0,
        SimTime::EPOCH,
        duration,
        &[arc],
        AdaptiveIntegratorConfig::default(),
    )
    .expect("spiral propagates");
    assert!(orbital_energy(mu, result.state) > orbital_energy(mu, initial));
    assert!(result.state.position.length() > 1.1e9);
    let expected_mass = 2_000.0 - (2.0 / 30_000.0) * duration;
    assert!((result.final_mass_kg - expected_mass).abs() / expected_mass < 1e-12);
}

#[test]
fn thrust_schedule_validation_rejects_garbage() {
    let mu = 1.2e17;
    let ephemeris = central_ephemeris(mu);
    let field = GravityField::from_ephemeris(&ephemeris);
    let live = ThrustArc {
        start_s: 0.0,
        duration_s: 100.0,
        direction: ThrustDirection::Inertial(DVec3::X),
        throttle_01: 1.0,
        thrust_n: 1_000.0,
        mass_flow_kgs: 0.05,
    };
    // Propellant depleted by the arc.
    let thirsty = ThrustArc {
        mass_flow_kgs: 10.0,
        ..live
    };
    assert!(
        propagate_adaptive_with_thrust(
            &field,
            circular_state(mu, 1.1e9),
            100.0,
            SimTime::EPOCH,
            200.0,
            &[thirsty],
            AdaptiveIntegratorConfig::default(),
        )
        .is_err()
    );
    // Unordered arcs.
    let late = ThrustArc {
        start_s: 150.0,
        ..live
    };
    let early = ThrustArc {
        start_s: 50.0,
        ..live
    };
    assert!(
        propagate_adaptive_with_thrust(
            &field,
            circular_state(mu, 1.1e9),
            20_000.0,
            SimTime::EPOCH,
            500.0,
            &[late, early],
            AdaptiveIntegratorConfig::default(),
        )
        .is_err()
    );
    // Zero direction with a live engine.
    let blind = ThrustArc {
        direction: ThrustDirection::Inertial(DVec3::ZERO),
        ..live
    };
    assert!(
        propagate_adaptive_with_thrust(
            &field,
            circular_state(mu, 1.1e9),
            20_000.0,
            SimTime::EPOCH,
            200.0,
            &[blind],
            AdaptiveIntegratorConfig::default(),
        )
        .is_err()
    );
    // Arc past the horizon, non-positive mass.
    let past = ThrustArc {
        start_s: 150.0,
        duration_s: 100.0,
        ..live
    };
    assert!(
        propagate_adaptive_with_thrust(
            &field,
            circular_state(mu, 1.1e9),
            20_000.0,
            SimTime::EPOCH,
            200.0,
            &[past],
            AdaptiveIntegratorConfig::default(),
        )
        .is_err()
    );
    assert!(
        propagate_adaptive_with_thrust(
            &field,
            circular_state(mu, 1.1e9),
            0.0,
            SimTime::EPOCH,
            200.0,
            &[live],
            AdaptiveIntegratorConfig::default(),
        )
        .is_err()
    );
}

#[test]
fn rtn_basis_matches_circular_orbit() {
    // Circular orbit in the XY plane: R=+X, C=+Z, T=+Y (prograde).
    let (radial, transverse, cross) =
        rtn_basis(DVec3::new(1.1e9, 0.0, 0.0), DVec3::new(0.0, 10_460.0, 0.0))
            .expect("healthy orbit geometry");
    assert!((radial - DVec3::X).length() < 1e-12);
    assert!((transverse - DVec3::Y).length() < 1e-12);
    assert!((cross - DVec3::Z).length() < 1e-12);
    // Degenerate: at the center, or radial flight with no orbit plane.
    assert!(rtn_basis(DVec3::ZERO, DVec3::X).is_none());
    assert!(rtn_basis(DVec3::X, DVec3::X * 100.0).is_none());
    assert!(rtn_basis(DVec3::new(f64::NAN, 0.0, 0.0), DVec3::X).is_none());
}

#[test]
fn rtn_normal_arc_conserves_energy_turning_plane() {
    // Pure orbit-normal thrust does no work (a ⊥ v always): energy must be
    // conserved while the plane rotates. Proves the frame is truly normal,
    // not a mislabeled prograde.
    let mu = 1.2e17;
    let ephemeris = central_ephemeris(mu);
    let field = GravityField::from_ephemeris(&ephemeris);
    let initial = circular_state(mu, 1.1e9);
    let duration = 3.0 * 86_400.0;
    let arc = ThrustArc {
        start_s: 0.0,
        duration_s: duration,
        direction: ThrustDirection::Rtn {
            central: BodyId(0),
            radial: 0.0,
            transverse: 0.0,
            normal: 1.0,
        },
        throttle_01: 1.0,
        thrust_n: 5.0,
        mass_flow_kgs: 5.0 / 30_000.0,
    };
    let result = propagate_adaptive_with_thrust(
        &field,
        initial,
        2_000.0,
        SimTime::EPOCH,
        duration,
        &[arc],
        AdaptiveIntegratorConfig::default(),
    )
    .expect("normal arc propagates");
    let energy_before = orbital_energy(mu, initial);
    let energy_after = orbital_energy(mu, result.state);
    assert!(
        ((energy_after - energy_before) / energy_before).abs() < 1e-6,
        "energy drift {energy_before} -> {energy_after}"
    );
    // ...while the orbit normal genuinely moved (plane change happened).
    let normal_before = initial.position.cross(initial.velocity).normalize();
    let normal_after = result
        .state
        .position
        .cross(result.state.velocity)
        .normalize();
    let plane_change = normal_before.dot(normal_after).clamp(-1.0, 1.0).acos();
    assert!(plane_change > 1e-4, "plane must rotate, got {plane_change}");
}

#[test]
fn rtn_arc_validation_rejects_garbage() {
    let mu = 1.2e17;
    let ephemeris = central_ephemeris(mu);
    let field = GravityField::from_ephemeris(&ephemeris);
    let live = ThrustArc {
        start_s: 0.0,
        duration_s: 100.0,
        direction: ThrustDirection::Rtn {
            central: BodyId(0),
            radial: 0.0,
            transverse: 1.0,
            normal: 0.0,
        },
        throttle_01: 1.0,
        thrust_n: 1_000.0,
        mass_flow_kgs: 0.05,
    };
    // Unknown central body.
    let lost = ThrustArc {
        direction: ThrustDirection::Rtn {
            central: BodyId(99),
            radial: 0.0,
            transverse: 1.0,
            normal: 0.0,
        },
        ..live
    };
    assert!(
        propagate_adaptive_with_thrust(
            &field,
            circular_state(mu, 1.1e9),
            20_000.0,
            SimTime::EPOCH,
            100.0,
            &[lost],
            AdaptiveIntegratorConfig::default(),
        )
        .is_err()
    );
    // All-zero components with a live engine.
    let bland = ThrustArc {
        direction: ThrustDirection::Rtn {
            central: BodyId(0),
            radial: 0.0,
            transverse: 0.0,
            normal: 0.0,
        },
        ..live
    };
    assert!(
        propagate_adaptive_with_thrust(
            &field,
            circular_state(mu, 1.1e9),
            20_000.0,
            SimTime::EPOCH,
            100.0,
            &[bland],
            AdaptiveIntegratorConfig::default(),
        )
        .is_err()
    );
}

#[test]
fn prograde_steering_at_rest_is_an_error() {
    // Steering is undefined at zero velocity: honest error, never a
    // silent coast in an arbitrary direction.
    let mu = 1.2e17;
    let ephemeris = central_ephemeris(mu);
    let field = GravityField::from_ephemeris(&ephemeris);
    let rest = TestParticleState {
        position: DVec3::new(1.1e9, 0.0, 0.0),
        velocity: DVec3::ZERO,
    };
    let arc = ThrustArc {
        start_s: 0.0,
        duration_s: 100.0,
        direction: ThrustDirection::Prograde,
        throttle_01: 1.0,
        thrust_n: 1_000.0,
        mass_flow_kgs: 0.05,
    };
    assert!(
        propagate_adaptive_with_thrust(
            &field,
            rest,
            20_000.0,
            SimTime::EPOCH,
            100.0,
            &[arc],
            AdaptiveIntegratorConfig::default(),
        )
        .is_err()
    );
}

#[test]
fn sensitivity_matches_finite_difference_jacobian() {
    // Variational STM vs brute-force perturbations on a half-period LEO
    // arc: columns of Sr must equal d(r_end)/d(v0) within the
    // finite-difference truncation error. The analytic STM is the more
    // accurate side; the tolerance budgets the FD error, not the STM.
    let mu = 3.986_004_418e14;
    let ephemeris = central_ephemeris(mu);
    let field = GravityField::from_ephemeris(&ephemeris);
    let radius = 7_000_000.0;
    let initial = TestParticleState {
        position: DVec3::new(radius, 0.0, 0.0),
        velocity: DVec3::new(0.0, (mu / radius).sqrt(), 500.0),
    };
    let config = AdaptiveIntegratorConfig {
        initial_step_s: 30.0,
        min_step_s: 1.0e-6,
        max_step_s: 300.0,
        absolute_position_tolerance_m: 1.0e-4,
        absolute_velocity_tolerance_mps: 1.0e-7,
        relative_tolerance: 1.0e-11,
        max_steps: 100_000,
        dynamical_eta: None,
    };
    let duration = 3_000.0;
    let sens = propagate_adaptive_sensitivity(
        &field,
        initial,
        VelocitySensitivity::identity(),
        SimTime::EPOCH,
        duration,
        config,
    )
    .expect("sensitivity propagation");
    // The augmented trajectory must match the plain one bit-for-bit: same
    // coefficients, same step logic, same error control.
    let plain = propagate_adaptive(&field, initial, SimTime::EPOCH, duration, config)
        .expect("plain propagation");
    assert_eq!(sens.state, plain.state);
    assert_eq!(sens.stats, plain.stats);
    // Columns of Sr vs central differences (O(h^2) truncation, so the
    // comparison budgets the FD error, not the STM).
    let h = 0.5;
    for (axis, column) in [DVec3::X, DVec3::Y, DVec3::Z]
        .iter()
        .zip(sens.sensitivity.position.iter())
    {
        let plus = TestParticleState {
            position: initial.position,
            velocity: initial.velocity + *axis * h,
        };
        let minus = TestParticleState {
            position: initial.position,
            velocity: initial.velocity - *axis * h,
        };
        let end_plus = propagate_adaptive(&field, plus, SimTime::EPOCH, duration, config)
            .expect("perturbed propagation")
            .state
            .position;
        let end_minus = propagate_adaptive(&field, minus, SimTime::EPOCH, duration, config)
            .expect("perturbed propagation")
            .state
            .position;
        let fd = (end_plus - end_minus) / (2.0 * h);
        let scale = fd.length().max(1.0);
        assert!(
            (*column - fd).length() / scale < 1.0e-7,
            "stm column vs fd mismatch: {column:?} vs {fd:?}"
        );
    }
}

#[test]
fn dop853_converges_at_eighth_order() {
    // Order proof for the DOP853 tableau: on a smooth circular orbit with
    // the error controller parked (loose tolerances so steps ride the max
    // ceiling), halving the step must cut the period-closure error by ~2^8.
    // A mistyped coefficient would collapse this to a low order.
    // Accepts a wide band (64..1024) around 256 for higher-order terms.
    let mu = 3.986_004_418e14;
    let ephemeris = central_ephemeris(mu);
    let field = GravityField::from_ephemeris(&ephemeris);
    let radius = 7_000_000.0;
    let initial = TestParticleState {
        position: DVec3::new(radius, 0.0, 0.0),
        velocity: DVec3::new(0.0, (mu / radius).sqrt(), 0.0),
    };
    let period = TAU * (radius.powi(3) / mu).sqrt();
    let run = |max_step: f64| {
        propagate_adaptive_dop853(
            &field,
            initial,
            SimTime::EPOCH,
            period,
            AdaptiveIntegratorConfig {
                initial_step_s: max_step,
                min_step_s: 1.0e-6,
                max_step_s: max_step,
                absolute_position_tolerance_m: 1.0e6,
                absolute_velocity_tolerance_mps: 1.0e3,
                relative_tolerance: 1.0,
                max_steps: 100_000,
                dynamical_eta: None,
            },
        )
        .expect("dop853 fixed-step run")
        .state
        .position
        .distance(initial.position)
    };
    let coarse = run(200.0);
    let fine = run(100.0);
    assert!(
        coarse.is_finite() && fine.is_finite() && fine > 0.0,
        "both runs must close with finite nonzero error: {coarse:e} {fine:e}"
    );
    let ratio = coarse / fine;
    eprintln!("dop853 order check: err200={coarse:e} err100={fine:e} ratio={ratio:.1}");
    assert!(
        (64.0..=1024.0).contains(&ratio),
        "eighth-order halving must land near 256, got {ratio:.1}"
    );
}

#[test]
fn dop853_matches_dp5_on_a_transfer_arc() {
    // Cross-method agreement: DOP853 at tight tolerances must reproduce
    // the proven DP5 trajectory within the looser of the two error
    // budgets on a perturbed three-body arc.
    let mu_primary = 1.0e14;
    let mu_secondary = 1.0e12;
    let separation = 1.0e8;
    let ephemeris = two_body_binary(mu_primary, mu_secondary, separation);
    let field = GravityField::from_ephemeris(&ephemeris);
    let initial = TestParticleState {
        position: DVec3::new(1.8e8, -2.5e7, 0.0),
        velocity: DVec3::new(120.0, 560.0, 0.0),
    };
    let config = AdaptiveIntegratorConfig {
        initial_step_s: 100.0,
        min_step_s: 1.0e-5,
        max_step_s: 2_000.0,
        absolute_position_tolerance_m: 1.0e-2,
        absolute_velocity_tolerance_mps: 1.0e-5,
        relative_tolerance: 1.0e-9,
        max_steps: 100_000,
        dynamical_eta: None,
    };
    let duration = 200_000.0;
    let dp5 = propagate_adaptive(&field, initial, SimTime::EPOCH, duration, config)
        .expect("dp5 reference")
        .state;
    let dop = propagate_adaptive_dop853(&field, initial, SimTime::EPOCH, duration, config)
        .expect("dop853 candidate")
        .state;
    let pos_err = (dop.position - dp5.position).length();
    let vel_err = (dop.velocity - dp5.velocity).length();
    eprintln!("dop853-vs-dp5: dpos={pos_err:e} dvel={vel_err:e}");
    assert!(
        pos_err < 1.0,
        "position agreement within 1 m, got {pos_err:e}"
    );
    assert!(
        vel_err < 1.0e-3,
        "velocity agreement within 1 mm/s, got {vel_err:e}"
    );
}
