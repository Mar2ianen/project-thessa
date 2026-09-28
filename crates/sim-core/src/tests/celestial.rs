use super::*;

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
            dynamical_eta: None,
        },
    )
    .expect("circular orbit should propagate");
    assert!((result.end_time.seconds() - period).abs() < 1.0e-9);
    assert!(result.state.position.distance(initial.position) < 2.0);
    assert!(result.state.velocity.distance(initial.velocity) < 1.0e-3);
}

#[test]
fn dop853_adaptive_controller_honors_tolerance() {
    // Regression for the missing `* h` in the DOP853 error estimate: without
    // it the controller underestimates the error by ~h and tightening the
    // tolerance changes nothing. One circular period has analytic closure,
    // so tighter tolerance must shrink the closure error and cost steps.
    let mu = 3.986_004_418e14;
    let radius = 7_000_000.0;
    let ephemeris = central_ephemeris(mu);
    let field = GravityField::from_ephemeris(&ephemeris);
    let initial = TestParticleState {
        position: DVec3::new(radius, 0.0, 0.0),
        velocity: DVec3::new(0.0, (mu / radius).sqrt(), 0.0),
    };
    let period = TAU * (radius.powi(3) / mu).sqrt();
    let run = |pos_tol: f64, vel_tol: f64, rel: f64| {
        propagate_adaptive_dop853(
            &field,
            initial,
            SimTime::EPOCH,
            period,
            AdaptiveIntegratorConfig {
                initial_step_s: 30.0,
                min_step_s: 1.0e-5,
                max_step_s: 120.0,
                absolute_position_tolerance_m: pos_tol,
                absolute_velocity_tolerance_mps: vel_tol,
                relative_tolerance: rel,
                max_steps: 100_000,
                dynamical_eta: None,
            },
        )
        .expect("circular orbit should propagate")
    };
    let loose = run(1.0e-1, 1.0e-4, 1.0e-8);
    let tight = run(1.0e-4, 1.0e-7, 1.0e-11);
    let loose_err = loose.state.position.distance(initial.position);
    let tight_err = tight.state.position.distance(initial.position);
    eprintln!("dop853 closure: loose={loose_err:.3} m, tight={tight_err:.6} m");
    assert!(
        tight_err < loose_err,
        "tighter tolerance must shrink closure error"
    );
    assert!(
        tight_err < 2.0,
        "tight closure must be meters, got {tight_err}"
    );
    assert!(
        tight.stats.accepted_steps + tight.stats.rejected_steps
            > loose.stats.accepted_steps + loose.stats.rejected_steps,
        "tighter tolerance must cost steps"
    );
}

#[test]
fn dynamical_cap_reads_current_body_positions_after_rejected_steps() {
    // The RK scratch frame holds rejected stage timestamps past the retry
    // point; the cap must not read body positions from it. Moving binary
    // source, huge first step to force rejections, tight cap: completes,
    // rejects along the way, and agrees with the uncapped reference.
    let total_mu = 1.2e12;
    let separation = 2.0e7;
    let ephemeris = two_body_binary(1.0e12, 2.0e11, separation);
    let field = GravityField::from_ephemeris(&ephemeris);
    let radius = 4.0e7;
    let initial = TestParticleState {
        position: DVec3::new(radius, 0.0, 0.0),
        velocity: DVec3::new(0.0, (total_mu / radius).sqrt(), 0.0),
    };
    let duration = 200_000.0;
    let config = |eta: Option<f64>| AdaptiveIntegratorConfig {
        initial_step_s: 200_000.0,
        min_step_s: 1.0e-5,
        max_step_s: 200_000.0,
        absolute_position_tolerance_m: 1.0e-3,
        absolute_velocity_tolerance_mps: 1.0e-6,
        relative_tolerance: 1.0e-11,
        max_steps: 100_000,
        dynamical_eta: eta,
    };
    for propagate in [
        propagate_adaptive
            as fn(
                &GravityField,
                TestParticleState,
                SimTime,
                f64,
                AdaptiveIntegratorConfig,
            ) -> Result<PropagationResult, IntegratorError>,
        propagate_adaptive_dop853,
    ] {
        let capped = propagate(&field, initial, SimTime::EPOCH, duration, config(Some(0.5)))
            .expect("capped propagation completes");
        eprintln!(
            "capped stats: accepted={} rejected={}",
            capped.stats.accepted_steps, capped.stats.rejected_steps
        );
        assert!(
            capped.stats.rejected_steps > 0,
            "test must exercise the reject path"
        );
        let reference = propagate(&field, initial, SimTime::EPOCH, duration, config(None))
            .expect("reference propagation completes");
        let drift = capped.state.position.distance(reference.state.position);
        eprintln!("cap-vs-reference drift: {drift:.4} m");
        assert!(
            drift < 5.0,
            "capped run must track the reference, drift={drift}"
        );
    }
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
            dynamical_eta: None,
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
            dynamical_eta: None,
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
        dynamical_eta: None,
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
fn frame_batch_accelerations_match_direct_accumulation_bitwise() {
    let ephemeris = chain_ephemeris();
    let field = GravityField::from_ephemeris(&ephemeris);
    let positions = vec![
        DVec3::new(8.0e6, 0.0, 0.0),
        DVec3::new(0.0, 9.0e6, 0.0),
        DVec3::new(-10.0e6, 0.0, 1.0e6),
    ];
    let mut frame = EphemerisFrame::new();
    for seconds in [0.0, 8.0 / 120.0, 1_000_000.0] {
        let time = SimTime(seconds);
        let direct = field
            .accelerations(&positions, time)
            .expect("direct batch gravity");
        let states = frame.evaluate(&ephemeris, time).expect("frame states");
        let framed = field
            .accelerations_from_frame(&positions, states)
            .expect("frame batch gravity");
        assert_eq!(direct, framed, "frame batch must match at {seconds}s");
    }
}

fn two_binaries() -> BakedEphemeris {
    // Barycenter with two planet+moon pairs on opposite sides: the tree has
    // two internal children (one per pair), so a mid-range budget opens the
    // root while both pairs compete for the remaining budget.
    let orbit = |mu: f64, a: f64, m0: f64| {
        KeplerOrbit::new(mu, a, 0.0, 0.1, 0.2, 0.3, m0).expect("valid test orbit")
    };
    BakedEphemeris::new(
        "TEST_TWO_BINARIES",
        vec![
            BakedBody::synthetic_barycenter(BodyId(0), "barycenter", 4.4e14, None, None),
            BakedBody::orbital(
                BodyId(1),
                "planet-a",
                2.0e14,
                0.0,
                BodyId(0),
                orbit(4.4e14, 2.0e8, 0.0),
            ),
            BakedBody::orbital(
                BodyId(2),
                "moon-a",
                2.0e13,
                0.0,
                BodyId(1),
                orbit(2.2e14, 1.0e7, 0.0),
            ),
            BakedBody::orbital(
                BodyId(3),
                "planet-b",
                2.0e14,
                0.0,
                BodyId(0),
                orbit(4.4e14, 2.0e8, std::f64::consts::PI),
            ),
            BakedBody::orbital(
                BodyId(4),
                "moon-b",
                2.0e13,
                0.0,
                BodyId(3),
                orbit(2.2e14, 1.0e7, std::f64::consts::PI),
            ),
        ],
    )
    .expect("valid two-binaries ephemeris")
}

#[test]
fn tree_rejects_mismatched_frames_slice() {
    let ephemeris = two_body_binary(3.0e14, 1.0e14, 1.0e8);
    let tree = GravitySourceTree::build(&ephemeris).expect("tree builds");
    let mut frame = EphemerisFrame::new();
    let states = frame
        .evaluate(&ephemeris, SimTime::EPOCH)
        .expect("frame states");
    let frames = tree.resolve(states).expect("node frames");
    let position = DVec3::new(4.0e9, 0.0, 0.0);
    // Full frames evaluate fine.
    tree.evaluate(&frames, states, position, 1.0e-6)
        .expect("full frames evaluate");
    // A short slice (or one from another tree) fails open, never panics.
    let short = &frames[..frames.len() - 1];
    assert!(tree.evaluate(short, states, position, 1.0e-6).is_err());
    assert!(tree.evaluate(&[], states, position, 1.0e-6).is_err());
}

#[test]
fn tree_groups_binary_children_under_barycenter_node() {
    let mu_primary = 3.0e14;
    let mu_secondary = 1.0e14;
    let ephemeris = two_body_binary(mu_primary, mu_secondary, 1.0e8);
    let tree = GravitySourceTree::build(&ephemeris).expect("tree builds");
    // Barycenter aggregate + two source leaves.
    assert_eq!(tree.node_count(), 3);
    let root = &tree.nodes()[tree.roots()[0] as usize];
    assert_eq!(root.children.len(), 2);
    assert!((root.mu_total - (mu_primary + mu_secondary)).abs() <= 1.0);
    assert_eq!(root.own_mu, 0.0);
}

#[test]
fn tree_spends_remaining_budget_across_sibling_aggregates() {
    // Two planet+moon pairs: at a mid-range budget the root opens while
    // each pair alone would fit. The first pair spends most of the budget,
    // forcing the second open — the total posted bound must still hold.
    // (Per-node gating would accept both and post ~2x the budget.)
    let ephemeris = two_binaries();
    let field = GravityField::from_ephemeris(&ephemeris);
    let tree = GravitySourceTree::build(&ephemeris).expect("tree builds");
    assert_eq!(tree.node_count(), 5);
    let mut frame = EphemerisFrame::new();
    let time = SimTime::EPOCH;
    let states = frame.evaluate(&ephemeris, time).expect("frame states");
    let frames = tree.resolve(states).expect("node frames");
    let budget = 6.0e-9;
    let position = DVec3::new(1.0e10, 3.0e9, 0.0);
    let check_eval = |eval: &TreeEval, budget: f64| {
        assert!(
            eval.error_bound_mps2 <= budget,
            "posted {:e} exceeds allocated {budget:e}",
            eval.error_bound_mps2,
        );
        let exact = field.acceleration(position, time).expect("exact");
        let measured = (eval.acceleration - exact).length();
        assert!(
            measured <= eval.error_bound_mps2 * (1.0 + 1.0e-9),
            "measured {measured:e} exceeds posted {:e}",
            eval.error_bound_mps2,
        );
    };
    // Mid-range budget: with the quadrupole rung the root need not open —
    // it may accept at quad while children compete for the remainder.
    // Either way the posted bound holds and something nontrivial happens.
    let eval = tree
        .evaluate(&frames, states, position, budget)
        .expect("tree eval");
    check_eval(&eval, budget);
    assert!(
        eval.terms_quad + eval.terms_exact >= 1,
        "ladder must do real work at this budget"
    );
    // Zero budget forces the full open: every source exact, zero posted.
    // (Measured-vs-exact is ulp-level here, not bitwise: open leaves sum
    // in traversal order while the field sums in source order.)
    let open = tree
        .evaluate(&frames, states, position, 0.0)
        .expect("open eval");
    assert_eq!(open.error_bound_mps2, 0.0);
    assert!(
        open.nodes_visited >= 3,
        "open must traverse root and children"
    );
    let exact = field.acceleration(position, time).expect("exact");
    assert!(
        (open.acceleration - exact).length() <= 1e-12,
        "fully open tree must match the field to solver noise"
    );
}

#[test]
fn monopole_matches_explicit_children_within_posted_bound() {
    let mu_primary = 3.0e14;
    let mu_secondary = 1.0e14;
    let ephemeris = two_body_binary(mu_primary, mu_secondary, 1.0e8);
    let field = GravityField::from_ephemeris(&ephemeris);
    let tree = GravitySourceTree::build(&ephemeris).expect("tree builds");
    let mut frame = EphemerisFrame::new();
    let time = SimTime::EPOCH;
    let states = frame.evaluate(&ephemeris, time).expect("frame states");
    let frames = tree.resolve(states).expect("node frames");
    // Far-field points: the aggregate must be accepted and stay within its
    // own posted error bound (doc 23 sections 4, 17). The bound is
    // conservative by design (no cancellation accounting): at D/R ~ 20 it
    // already exceeds a 1e-6 budget and the node opens — that is covered by
    // the zero-budget case below. What the test pins is safety
    // (measured <= posted) plus the scaling law (bound/field shrinks with
    // distance) where acceptance happens.
    let mut last_ratio = f64::INFINITY;
    for distance in [5.0e9, 2.0e10] {
        let position = DVec3::new(distance, distance * 0.3, -distance * 0.17);
        let exact = field.acceleration(position, time).expect("exact gravity");
        let eval = tree
            .evaluate(&frames, states, position, 1.0e-6)
            .expect("tree gravity");
        assert_eq!(eval.terms_exact, 0, "far aggregate must be accepted");
        let measured = (eval.acceleration - exact).length();
        assert!(
            measured <= eval.error_bound_mps2 * (1.0 + 1.0e-9),
            "measured {measured:e} exceeds posted bound {:e} at {distance:e} m",
            eval.error_bound_mps2,
        );
        let ratio = eval.error_bound_mps2 / exact.length();
        assert!(
            ratio <= 0.05,
            "vacuous bound: ratio {ratio:e} at {distance:e} m",
        );
        assert!(
            ratio < last_ratio,
            "bound/field ratio must shrink with distance: {ratio:e} after {last_ratio:e}",
        );
        last_ratio = ratio;
    }
    // Zero budget opens everything: same physics as the exact path up to
    // summation order.
    let position = DVec3::new(4.0e9, 0.0, 0.0);
    let exact = field.acceleration(position, time).expect("exact gravity");
    let eval = tree
        .evaluate(&frames, states, position, 0.0)
        .expect("tree gravity");
    assert_eq!(eval.error_bound_mps2, 0.0);
    assert_eq!(eval.terms_exact, 2);
    assert!((eval.acceleration - exact).length() <= 1.0e-12);
}

mod quadrupole_tests {
    use super::*;
    use crate::{quadrupole_correction, quadrupole_error_estimate};

    fn outer(a: DVec3, b: DVec3) -> DMat3 {
        DMat3::from_cols(a * b.x, a * b.y, a * b.z)
    }

    #[test]
    fn correction_matches_direct_summation() {
        // Hand-built aggregates: exact sum minus monopole must equal the
        // closed form to higher-order leftovers. Symmetric case kills odd
        // orders (4th-order remainder); skewed case keeps 3rd order.
        let mu = 1.0e12;
        let pairs: Vec<(Vec<(DVec3, f64)>, DVec3)> = vec![
            // Two equal masses on ±x: analytic extra pull -6md²/R⁴ on axis.
            (
                vec![
                    (DVec3::new(1.0e7, 0.0, 0.0), mu),
                    (DVec3::new(-1.0e7, 0.0, 0.0), mu),
                ],
                DVec3::new(2.0e8, 0.0, 0.0),
            ),
            (
                vec![
                    (DVec3::new(1.0e7, 0.0, 0.0), mu),
                    (DVec3::new(-1.0e7, 0.0, 0.0), mu),
                ],
                DVec3::new(0.0, 2.0e8, 1.0e7),
            ),
            // Skewed triple: no symmetry to hide behind.
            (
                vec![
                    (DVec3::new(3.0e6, 1.0e6, 0.0), mu),
                    (DVec3::new(-2.0e6, 2.0e6, 1.0e6), 0.5 * mu),
                    (DVec3::new(0.0, -1.0e6, -2.0e6), 0.25 * mu),
                ],
                DVec3::new(1.0e8, 2.0e7, -1.5e7),
            ),
        ];
        for (members, probe) in pairs {
            let total_mu: f64 = members.iter().map(|(_, m)| m).sum();
            let barycenter = members.iter().map(|(pos, m)| *pos * *m).sum::<DVec3>() / total_mu;
            let mut second = DMat3::ZERO;
            let mut exact = DVec3::ZERO;
            for (pos, m) in &members {
                let offset = *pos - probe;
                let r2 = offset.length_squared();
                exact += offset * (*m / (r2 * r2.sqrt()));
                let d = *pos - barycenter;
                second += outer(d, d) * *m;
            }
            let offset = barycenter - probe;
            let monopole = offset * (total_mu / offset.length_squared().powf(1.5));
            let correction = quadrupole_correction(second, offset);
            let residual = (exact - monopole - correction).length();
            let scale = (exact - monopole).length().max(monopole.length() * 1e-12);
            assert!(
                residual / scale < 2e-2,
                "quadrupole must explain the non-monopole field to 2%, got {}",
                residual / scale
            );
            // And it must strictly improve over monopole alone.
            assert!(
                residual < (exact - monopole).length(),
                "correction must reduce the error"
            );
        }
    }

    #[test]
    fn two_mass_axis_matches_closed_form() {
        // Pin the exact constant: extra inward pull -6·m·d²/R⁴ on axis.
        let mu = 1.0e12;
        let d = 1.0e7;
        let second = outer(DVec3::new(d, 0.0, 0.0), DVec3::new(d, 0.0, 0.0)) * mu
            + outer(DVec3::new(-d, 0.0, 0.0), DVec3::new(-d, 0.0, 0.0)) * mu;
        let radius = 2.0e8;
        let correction = quadrupole_correction(second, DVec3::new(-radius, 0.0, 0.0));
        let expected = -6.0 * mu * d * d / radius.powi(4);
        assert!((correction.x - expected).abs() / expected.abs() < 1e-12);
        assert_eq!(correction.y, 0.0);
        assert_eq!(correction.z, 0.0);
    }

    #[test]
    fn node_frames_carry_exact_second_moments() {
        // Leaves sit on their own mass: zero moment. Internal nodes match a
        // hand accumulation from the same states.
        let ephemeris = two_body_binary(3.0e14, 1.0e14, 1.0e8);
        let tree = GravitySourceTree::build(&ephemeris).expect("tree builds");
        let mut frame = EphemerisFrame::new();
        let states = frame
            .evaluate(&ephemeris, SimTime::EPOCH)
            .expect("frame states");
        let frames = tree.resolve(states).expect("node frames");
        for (node_id, node) in tree.nodes().iter().enumerate() {
            let resolved = &frames[node_id];
            if node.children.is_empty() {
                assert_eq!(resolved.second_moment, DMat3::ZERO);
                continue;
            }
            let mut expected = DMat3::ZERO;
            // Walk the subtree masses directly from the ephemeris.
            let mut stack = vec![node_id as u32];
            while let Some(current) = stack.pop() {
                let current_node = &tree.nodes()[current as usize];
                if current_node.own_mu > 0.0 {
                    let pos = states[current_node.body.index()].position_inertial;
                    let d = pos - resolved.barycenter;
                    expected += outer(d, d) * current_node.own_mu;
                }
                stack.extend(current_node.children.iter().copied());
            }
            let diff = (expected - resolved.second_moment).col(0).length()
                + (expected - resolved.second_moment).col(1).length()
                + (expected - resolved.second_moment).col(2).length();
            let scale = expected
                .col(0)
                .length()
                .max(expected.col(1).length())
                .max(expected.col(2).length())
                .max(1.0);
            assert!(
                diff / scale < 1e-12,
                "second moment mismatch at node {node_id}"
            );
        }
    }

    #[test]
    fn quadrupole_rung_fires_between_monopole_and_open() {
        // Budget engineered from the root's own estimates: monopole rung
        // fails, quadrupole rung fits. The ladder must take the middle.
        let ephemeris = two_binaries();
        let field = GravityField::from_ephemeris(&ephemeris);
        let tree = GravitySourceTree::build(&ephemeris).expect("tree builds");
        let mut frame = EphemerisFrame::new();
        let time = SimTime::EPOCH;
        let states = frame.evaluate(&ephemeris, time).expect("frame states");
        let frames = tree.resolve(states).expect("node frames");
        let position = DVec3::new(1.0e10, 3.0e9, 0.0);
        let root_id = tree.roots()[0] as usize;
        let root = &tree.nodes()[root_id];
        let root_frame = &frames[root_id];
        let distance = (root_frame.barycenter - position).length();
        let clearance = distance - root_frame.radius_m;
        assert!(clearance > 0.0, "probe must stay outside the root ball");
        let mono = monopole_error_estimate(root.mu_total, root_frame.radius_m, clearance);
        let quad = quadrupole_error_estimate(root.mu_total, root_frame.radius_m, clearance);
        assert!(
            quad < mono,
            "test geometry must separate the rungs: mono={mono:e} quad={quad:e}"
        );
        let budget = (mono + quad) / 2.0;
        let eval = tree
            .evaluate(&frames, states, position, budget)
            .expect("tree eval");
        assert!(eval.terms_quad >= 1, "middle rung must fire");
        assert!(
            eval.error_bound_mps2 <= budget,
            "posted {:e} exceeds {budget:e}",
            eval.error_bound_mps2
        );
        let exact = field.acceleration(position, time).expect("exact");
        assert!(
            (eval.acceleration - exact).length() <= eval.error_bound_mps2 * (1.0 + 1e-9),
            "measured exceeds posted"
        );
    }

    #[test]
    fn quadrupole_bound_holds_across_geometry() {
        // Empirical soundness net for the K=128 remainder constant: sweep
        // near/mid/far targets across budgets and assert measured <= posted
        // <= budget everywhere. If the constant ever underestimates, this
        // fails rather than silently accepting a bad aggregate.
        let ephemeris = two_binaries();
        let field = GravityField::from_ephemeris(&ephemeris);
        let tree = GravitySourceTree::build(&ephemeris).expect("tree builds");
        let mut frame = EphemerisFrame::new();
        let time = SimTime::EPOCH;
        let states = frame.evaluate(&ephemeris, time).expect("frame states");
        let frames = tree.resolve(states).expect("node frames");
        let positions = [
            DVec3::new(4.0e9, 0.0, 0.0),
            DVec3::new(1.0e10, 3.0e9, 0.0),
            DVec3::new(-2.0e10, 1.0e10, 5.0e9),
            DVec3::new(1.0e11, -3.0e10, 2.0e10),
            DVec3::new(3.0e8, 1.0e8, 0.0),
        ];
        for position in positions {
            let exact = field.acceleration(position, time).expect("exact");
            for budget in [0.0, 1.0e-12, 1.0e-9, 1.0e-6, 1.0] {
                let eval = tree
                    .evaluate(&frames, states, position, budget)
                    .expect("tree eval");
                assert!(
                    eval.error_bound_mps2 <= budget,
                    "posted {:e} exceeds {budget:e} at {position:?}",
                    eval.error_bound_mps2
                );
                let measured = (eval.acceleration - exact).length();
                // Open leaves sum in traversal order (not source order):
                // ulp-level difference, not a bound violation.
                let tolerance = eval.error_bound_mps2.max(exact.length() * 1e-12);
                assert!(
                    measured <= tolerance * (1.0 + 1e-9),
                    "measured {measured:e} exceeds posted {:e} at {position:?} budget {budget:e}",
                    eval.error_bound_mps2
                );
            }
        }
    }
}

#[test]
fn tree_opens_aggregate_ball_for_close_targets() {
    let ephemeris = two_body_binary(3.0e14, 1.0e14, 1.0e8);
    let tree = GravitySourceTree::build(&ephemeris).expect("tree builds");
    let mut frame = EphemerisFrame::new();
    let states = frame
        .evaluate(&ephemeris, SimTime::EPOCH)
        .expect("frame states");
    let frames = tree.resolve(states).expect("node frames");
    // Sit on top of the secondary: inside the aggregate ball, so the tree
    // must open and evaluate both bodies exactly.
    let secondary = states[2].position_inertial + DVec3::new(1.0e6, 0.0, 0.0);
    let eval = tree
        .evaluate(&frames, states, secondary, 1.0e-6)
        .expect("tree gravity");
    assert_eq!(eval.error_bound_mps2, 0.0);
    assert_eq!(eval.terms_exact, 2);
    assert!(eval.nodes_visited >= 3);
}

#[test]
fn hessian_norm_matches_closed_form() {
    // Pins the HESSIAN_FROBENIUS_NORM derivation independently of the
    // tidal-tensor code: S = sum_ijk [3(d_ij n_k + d_ik n_j + d_jk n_i)
    // - 15 n_i n_j n_k]^2 must equal 90 for every unit direction.
    for direction in [
        DVec3::X,
        DVec3::Y,
        DVec3::Z,
        DVec3::new(1.0, 2.0, 3.0).normalize(),
        DVec3::new(-0.3, 0.8, 0.55).normalize(),
    ] {
        let n = [direction.x, direction.y, direction.z];
        let mut sum = 0.0;
        for i in 0..3 {
            for j in 0..3 {
                for k in 0..3 {
                    let delta = |a: usize, b: usize| f64::from(a == b);
                    let a = delta(i, j) * n[k] + delta(i, k) * n[j] + delta(j, k) * n[i];
                    let term = 3.0 * a - 15.0 * n[i] * n[j] * n[k];
                    sum += term * term;
                }
            }
        }
        assert!(
            (sum - 90.0).abs() <= 1.0e-9,
            "Frobenius sum {sum} != 90 for {direction:?}"
        );
    }
    assert!((HESSIAN_FROBENIUS_NORM * HESSIAN_FROBENIUS_NORM - 90.0).abs() <= 1.0e-9);
    assert!((HESSIAN_REMAINDER * 2.0 - HESSIAN_FROBENIUS_NORM).abs() == 0.0);
}

#[test]
fn patch_matches_exact_within_posted_bound() {
    let ephemeris = two_body_binary(3.0e14, 1.0e14, 1.0e8);
    let field = GravityField::from_ephemeris(&ephemeris);
    let mut frame = EphemerisFrame::new();
    let time = SimTime::EPOCH;
    let states = frame.evaluate(&ephemeris, time).expect("frame states");
    // Compact ball far from both bodies: everything absorbed, one patch.
    let center = DVec3::new(3.0e9, 1.0e9, 0.0);
    let positions: Vec<_> = (0..64)
        .map(|index| {
            let i = index as f64;
            center
                + DVec3::new((i * 12.9898).sin(), (i * 78.233).sin(), (i * 37.719).sin()) * 20_000.0
        })
        .collect();
    let config = CohortConfig {
        error_budget_mps2: 1.0e-9,
        ..Default::default()
    };
    let report = evaluate_cohorts(&ephemeris, states, &positions, config).expect("cohorts");
    assert_eq!(report.cohort_count, 1);
    assert_eq!(report.split_count, 0);
    assert!(report.error_bound_mps2 <= config.error_budget_mps2);
    let exact = field.accelerations(&positions, time).expect("exact batch");
    for (computed, reference) in report.accelerations.iter().zip(&exact) {
        let measured = (*computed - *reference).length();
        assert!(
            measured <= report.error_bound_mps2 * (1.0 + 1.0e-6),
            "patch error {measured:e} exceeds posted {:e}",
            report.error_bound_mps2,
        );
    }
}

#[test]
fn cohorts_split_before_bound_is_violated() {
    // Equal masses so each source contributes half the whole-ball bound:
    // at 60% of it both stay absorbed while their sum violates it.
    let ephemeris = two_body_binary(2.0e14, 2.0e14, 1.0e8);
    let field = GravityField::from_ephemeris(&ephemeris);
    let mut frame = EphemerisFrame::new();
    let time = SimTime::EPOCH;
    let states = frame.evaluate(&ephemeris, time).expect("frame states");
    // Stretched group in a smooth field: every source fits the budget
    // alone, but the summed remainder over the +-1e9 ball does not — so the
    // cohort must split (not go exact) until the child balls satisfy it.
    // Budget is calibrated at 60% of the whole-ball bound measured with an
    // effectively infinite budget.
    let center = DVec3::new(3.0e10, 0.0, 0.0);
    let positions: Vec<_> = (0..48)
        .map(|index| center + DVec3::new((index as f64 - 24.0 + 0.5) * 4.0e7, 0.0, 0.0))
        .collect();
    let probe = compile_patch(
        &ephemeris,
        states,
        &positions,
        CohortConfig {
            error_budget_mps2: 1.0e300,
            ..Default::default()
        },
    )
    .expect("probe patch compiles");
    assert!(
        probe.exact.is_empty(),
        "probe must absorb everything, got {:?}",
        probe.exact
    );
    let config = CohortConfig {
        error_budget_mps2: probe.error_bound_mps2 * 0.6,
        ..Default::default()
    };
    let report = evaluate_cohorts(&ephemeris, states, &positions, config).expect("cohorts");
    assert!(report.split_count > 0, "stretched group must split");
    assert!(report.error_bound_mps2 <= config.error_budget_mps2);
    let exact = field.accelerations(&positions, time).expect("exact batch");
    for (computed, reference) in report.accelerations.iter().zip(&exact) {
        let measured = (*computed - *reference).length();
        // The subsystem's whole point is a provable conservative bound:
        // hold the measured error to the posted bound, not 10x budget.
        assert!(
            measured <= report.error_bound_mps2 * (1.0 + 1.0e-6) + 1.0e-15,
            "cohort error {measured:e} escapes posted {:e}",
            report.error_bound_mps2,
        );
    }
}

#[test]
fn window_reuse_matches_fresh_within_budget() {
    let ephemeris = two_body_binary(3.0e14, 1.0e14, 1.0e8);
    let field = GravityField::from_ephemeris(&ephemeris);
    let config = CohortConfig {
        error_budget_mps2: 1.0e-9,
        ..Default::default()
    };
    let center = DVec3::new(3.0e9, 1.0e9, 0.0);
    let positions: Vec<_> = (0..32)
        .map(|index| {
            let i = index as f64;
            center + DVec3::new((i * 12.9898).sin(), (i * 78.233).sin(), 0.0) * 20_000.0
        })
        .collect();
    let mut frame = EphemerisFrame::new();
    let mut evaluator = CohortEvaluator::new();
    let states0 = frame
        .evaluate(&ephemeris, SimTime::EPOCH)
        .expect("frame t0")
        .to_vec();
    let first = evaluator
        .evaluate(&ephemeris, &states0, &positions, config)
        .expect("first eval builds window");
    assert!(!first.reused_window);
    // Sources moved for 10 s; the window must hold with a tighter achieved
    // bound, and stay within budget against the exact field at t1.
    let states1 = frame
        .evaluate(&ephemeris, SimTime(10.0))
        .expect("frame t1")
        .to_vec();
    let second = evaluator
        .evaluate(&ephemeris, &states1, &positions, config)
        .expect("second eval reuses window");
    let reused = second.reused_window;
    let bound = second.error_bound_mps2;
    let computed: Vec<_> = second.accelerations.to_vec();
    assert!(reused);
    assert_eq!(evaluator.reuses, 1);
    let exact = field
        .accelerations(&positions, SimTime(10.0))
        .expect("exact batch");
    for (computed, reference) in computed.iter().zip(&exact) {
        // Posted bound, not budget: the subsystem promises this number.
        let measured = (*computed - *reference).length();
        assert!(
            measured <= bound * (1.0 + 1.0e-6) + 1.0e-15,
            "reused window error {measured:e} escapes posted {bound:e}"
        );
    }
}

#[test]
fn window_rebuilds_when_group_leaves_ball() {
    let ephemeris = two_body_binary(3.0e14, 1.0e14, 1.0e8);
    let field = GravityField::from_ephemeris(&ephemeris);
    let config = CohortConfig {
        error_budget_mps2: 1.0e-9,
        ..Default::default()
    };
    let mut frame = EphemerisFrame::new();
    let states = frame
        .evaluate(&ephemeris, SimTime::EPOCH)
        .expect("frame states")
        .to_vec();
    let mut evaluator = CohortEvaluator::new();
    let near: Vec<_> = (0..16)
        .map(|index| DVec3::new(3.0e9 + index as f64 * 1_000.0, 0.0, 0.0))
        .collect();
    let first = evaluator
        .evaluate(&ephemeris, &states, &near, config)
        .expect("first eval");
    assert!(!first.reused_window);
    // Teleport the group across the system: the old ball cannot cover it,
    // so the evaluator must rebuild — and stay correct.
    let far: Vec<_> = (0..16)
        .map(|index| DVec3::new(-4.0e9 - index as f64 * 1_000.0, 0.0, 0.0))
        .collect();
    let second = evaluator
        .evaluate(&ephemeris, &states, &far, config)
        .expect("rebuild eval");
    let reused = second.reused_window;
    let bound = second.error_bound_mps2;
    let computed: Vec<_> = second.accelerations.to_vec();
    assert!(!reused);
    assert_eq!(evaluator.rebuilds, 2);
    let exact = field.accelerations(&far, SimTime::EPOCH).expect("exact");
    for (computed, reference) in computed.iter().zip(&exact) {
        let measured = (*computed - *reference).length();
        assert!(
            measured <= bound * (1.0 + 1.0e-6) + 1.0e-15,
            "rebuilt error {measured:e} escapes posted {bound:e}"
        );
    }
}

#[test]
fn classify_routes_near_host_to_exact() {
    let ephemeris = two_body_binary(3.0e14, 1.0e14, 1.0e8);
    let mut frame = EphemerisFrame::new();
    let states = frame
        .evaluate(&ephemeris, SimTime::EPOCH)
        .expect("frame states");
    // Ball around the secondary: it is inside, so it must be exact.
    let secondary = states[2].position_inertial;
    let group: Vec<_> = (0..8)
        .map(|index| secondary + DVec3::new(index as f64 * 100_000.0, 0.0, 0.0))
        .collect();
    let patch =
        compile_patch(&ephemeris, states, &group, CohortConfig::default()).expect("patch compiles");
    let exact_ids: Vec<_> = patch.exact.iter().map(|(body, _)| *body).collect();
    assert!(
        exact_ids.contains(&BodyId(2)),
        "secondary must be exact-near, got {exact_ids:?}"
    );
}

#[test]
fn evaluator_telemetry_sane_on_real_system() {
    let config_toml: SystemConfig =
        toml::from_str(include_str!("../../../../data/system.toml")).expect("system config");
    let ephemeris = config_toml.bake().expect("baked system");
    let home = ephemeris
        .body_state(ephemeris.body_id("thessa").unwrap(), SimTime::EPOCH)
        .unwrap();
    let center = home.position_inertial + DVec3::Z * 1e9;
    let positions: Vec<_> = (0..300)
        .map(|index| {
            let i = index as f64;
            center
                + DVec3::new(
                    (i * 12.9898).sin() * 50_000.0,
                    (i * 78.233).sin() * 50_000.0,
                    (i * 37.719).sin() * 50_000.0,
                )
        })
        .collect();
    let config = CohortConfig {
        error_budget_mps2: 1.0e-9,
        ..Default::default()
    };
    let mut frame = EphemerisFrame::new();
    let states = frame
        .evaluate(&ephemeris, SimTime::EPOCH)
        .expect("frame")
        .to_vec();
    let mut evaluator = CohortEvaluator::new();
    let first = evaluator
        .evaluate(&ephemeris, &states, &positions, config)
        .expect("first");
    let (reused, cohorts, splits, bound, exact, radius) = (
        first.reused_window,
        first.cohort_count,
        first.split_count,
        first.error_bound_mps2,
        first.exact_terms,
        first.max_radius_m,
    );
    assert!(!reused);
    assert_eq!((cohorts, splits), (1, 0));
    assert!(
        bound <= config.error_budget_mps2,
        "bound {bound:e} exceeds budget"
    );
    assert!(
        exact <= 300 * 22,
        "exact terms {exact} exceed 300 targets x 22 sources"
    );
    assert!(
        radius < 1.0e6,
        "radius {radius} insane for a +-50 km convoy"
    );
    let second = evaluator
        .evaluate(&ephemeris, &states, &positions, config)
        .expect("second");
    assert!(second.reused_window, "identical tick must reuse");
}

#[test]
fn window_reuse_holds_while_sources_drift_slowly() {
    // Controlled dynamics (binary period ~3e5 s, 10 s ticks): source drift
    // per tick is metres, so the temporal bound holds and nearly every tick
    // reuses the window — each one verified against exact. (On the real
    // system at 0.5 s ticks, fast-moon motion alone shifts the far field by
    // ~1e-8..1e-6 per tick, so a 1e-9 window correctly rebuilds instead.)
    let ephemeris = two_body_binary(3.0e14, 1.0e14, 1.0e8);
    let field = GravityField::from_ephemeris(&ephemeris);
    let config = CohortConfig {
        error_budget_mps2: 1.0e-9,
        ..Default::default()
    };
    let center = DVec3::new(3.0e9, 1.0e9, 0.0);
    let mut positions: Vec<_> = (0..32)
        .map(|index| {
            let i = index as f64;
            center + DVec3::new((i * 12.9898).sin(), (i * 78.233).sin(), 0.0) * 20_000.0
        })
        .collect();
    let mut velocities: Vec<_> = (0..32)
        .map(|index| {
            let i = index as f64;
            DVec3::new((i * 3.17).sin(), (i * 5.71).sin(), 0.0) * 2.0
        })
        .collect();
    let mut frame = EphemerisFrame::new();
    let mut evaluator = CohortEvaluator::new();
    for tick in 0..50 {
        let time = SimTime(tick as f64 * 10.0);
        let states = frame.evaluate(&ephemeris, time).expect("frame").to_vec();
        let eval = evaluator
            .evaluate(&ephemeris, &states, &positions, config)
            .expect("eval");
        let exact = field.accelerations(&positions, time).expect("exact");
        for (computed, reference) in eval.accelerations.iter().zip(&exact) {
            // Posted bound again — including on reused ticks, where the
            // temporal Lipschitz term is part of what is being checked.
            let measured = (*computed - *reference).length();
            assert!(
                measured <= eval.error_bound_mps2 * (1.0 + 1.0e-6) + 1.0e-15,
                "tick {tick}: measured {measured:e} escapes posted {:e}",
                eval.error_bound_mps2,
            );
        }
        let accels = eval.accelerations.to_vec();
        for (index, acceleration) in accels.iter().enumerate() {
            velocities[index] += *acceleration * 10.0;
            positions[index] += velocities[index] * 10.0;
        }
    }
    // Temporal staleness grows linearly to the budget, then a rebuild resets
    // the sawtooth: most ticks reuse, but the bound must actually bite.
    assert!(
        evaluator.reuses >= 35,
        "slow drift must reuse most ticks, got {}",
        evaluator.reuses
    );
    assert!(
        evaluator.rebuilds >= 5,
        "temporal bound must bite periodically, got {}",
        evaluator.rebuilds
    );
}

/// Deterministic xorshift64* for property tests: no new dependencies,
/// fixed seed, reproducible across runs and workers.
struct TestRng(u64);

impl TestRng {
    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (self.next_u64() as f64 / u64::MAX as f64) * (hi - lo)
    }

    fn unit(&mut self) -> DVec3 {
        loop {
            let direction = DVec3::new(
                self.range(-1.0, 1.0),
                self.range(-1.0, 1.0),
                self.range(-1.0, 1.0),
            );
            if direction.length_squared() > 1.0e-6 {
                return direction.normalize();
            }
        }
    }
}

#[test]
fn cohorts_hold_posted_bound_over_random_geometries() {
    // Dozens of deterministic source/target geometries across three
    // systems: every served acceleration must sit inside its posted bound,
    // and the posted bound inside the budget. Balls stay far from all
    // bodies (shell >= 1e9, radius <= 3e7, members within ~3e8), so no
    // singularities are possible by construction.
    let systems = [
        two_body_binary(2.0e14, 2.0e14, 1.0e8),
        two_body_binary(3.0e14, 1.0e14, 1.0e8),
        chain_ephemeris(),
    ];
    let config = CohortConfig {
        error_budget_mps2: 1.0e-9,
        ..Default::default()
    };
    let mut rng = TestRng(0x1234_5678_9ABC_DEF0);
    for (system_index, ephemeris) in systems.iter().enumerate() {
        let field = GravityField::from_ephemeris(ephemeris);
        let mut frame = EphemerisFrame::new();
        let states = frame
            .evaluate(ephemeris, SimTime::EPOCH)
            .expect("frame states");
        for case in 0..16 {
            let shell = 10.0_f64.powf(rng.range(9.0, 10.5));
            let center = rng.unit() * shell;
            let radius = 10.0_f64.powf(rng.range(3.0, 7.5));
            let count = 1 + (rng.next_u64() % 48) as usize;
            let positions: Vec<_> = (0..count)
                .map(|_| center + rng.unit() * rng.range(0.0, radius))
                .collect();
            let report =
                evaluate_cohorts(ephemeris, states, &positions, config).expect("cohorts evaluate");
            assert!(
                report.error_bound_mps2 <= config.error_budget_mps2,
                "system {system_index} case {case}: posted {:e} exceeds budget",
                report.error_bound_mps2,
            );
            let exact = field
                .accelerations(&positions, SimTime::EPOCH)
                .expect("exact batch");
            for (computed, reference) in report.accelerations.iter().zip(&exact) {
                let measured = (*computed - *reference).length();
                assert!(
                    measured <= report.error_bound_mps2 * (1.0 + 1.0e-6) + 1.0e-15,
                    "system {system_index} case {case}: measured {measured:e} escapes posted {:e}",
                    report.error_bound_mps2,
                );
            }
        }
    }
}

#[test]
fn validate_rejects_parent_cycle() {
    let orbit = |m0: f64| KeplerOrbit::new(1.0e14, 1.0e8, 0.0, 0.1, 0.2, 0.3, m0).unwrap();
    // 0 <-> 1 passes every per-body check (parents exist, orbits present)
    // yet would hang any parent-walking consumer: must fail fast here.
    let cyclic = BakedEphemeris::new(
        "TEST_CYCLE",
        vec![
            BakedBody::orbital(BodyId(0), "a", 1.0e13, 0.0, BodyId(1), orbit(0.0)),
            BakedBody::orbital(BodyId(1), "b", 1.0e13, 0.0, BodyId(0), orbit(1.0)),
        ],
    );
    assert!(matches!(cyclic, Err(EphemerisError::Cycle(_))));
    let self_loop = BakedEphemeris::new(
        "TEST_SELF_LOOP",
        vec![BakedBody::orbital(
            BodyId(0),
            "a",
            1.0e13,
            0.0,
            BodyId(0),
            orbit(0.0),
        )],
    );
    assert!(matches!(self_loop, Err(EphemerisError::Cycle(_))));
}

#[test]
fn validate_rejects_orbit_without_parent() {
    let orbit = KeplerOrbit::new(1.0e14, 1.0e8, 0.0, 0.1, 0.2, 0.3, 0.0).unwrap();
    let mut body = BakedBody::fixed(BodyId(0), "a", 1.0e13, 0.0);
    body.orbit = Some(orbit);
    let orphan = BakedEphemeris::new("TEST_ORPHAN_ORBIT", vec![body]);
    assert!(matches!(orphan, Err(EphemerisError::InvalidBody(_))));
}

#[test]
fn reuse_holds_posted_bound_over_random_epochs() {
    // Temporal twin of the static 48-geometry property test: random balls
    // evaluated at two epochs (sources drift between them), checking the
    // posted bound — fresh or reused — against exact at each epoch. Half
    // the cases use small epoch gaps (reuse likely), half large ones
    // (rebuild likely); the invariant holds on both paths.
    let ephemeris = two_body_binary(3.0e14, 1.0e14, 1.0e8);
    let field = GravityField::from_ephemeris(&ephemeris);
    let config = CohortConfig {
        error_budget_mps2: 1.0e-9,
        ..Default::default()
    };
    let mut rng = TestRng(0x0BAD_F00D_CAFE_1234);
    let mut frame = EphemerisFrame::new();
    for case in 0..24 {
        let shell = 10.0_f64.powf(rng.range(9.0, 10.0));
        let center = rng.unit() * shell;
        let radius = 10.0_f64.powf(rng.range(3.0, 5.0));
        let count = 1 + (rng.next_u64() % 24) as usize;
        let positions: Vec<_> = (0..count)
            .map(|_| center + rng.unit() * rng.range(0.0, radius))
            .collect();
        let t0 = rng.range(0.0, 200_000.0);
        let gap = if case % 2 == 0 {
            rng.range(5.0, 200.0)
        } else {
            rng.range(2_000.0, 20_000.0)
        };
        let mut evaluator = CohortEvaluator::new();
        for (epoch, label) in [(t0, "t0"), (t0 + gap, "t1")] {
            let states = frame
                .evaluate(&ephemeris, SimTime(epoch))
                .expect("frame states")
                .to_vec();
            let eval = evaluator
                .evaluate(&ephemeris, &states, &positions, config)
                .expect("eval");
            let bound = eval.error_bound_mps2;
            let computed: Vec<_> = eval.accelerations.to_vec();
            let exact = field
                .accelerations(&positions, SimTime(epoch))
                .expect("exact batch");
            for (computed, reference) in computed.iter().zip(&exact) {
                let measured = (*computed - *reference).length();
                assert!(
                    measured <= bound * (1.0 + 1.0e-6) + 1.0e-15,
                    "case {case} {label}: measured {measured:e} escapes posted {bound:e}"
                );
            }
        }
    }
}
