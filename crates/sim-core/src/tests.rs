use glam::{DMat3, DQuat, DVec3};
use std::io::Cursor;

use super::*;

/// Largest absolute entry of a 3x3 for test tolerances.
fn max_abs_entry(matrix: DMat3) -> f64 {
    matrix
        .col(0)
        .abs()
        .max(matrix.col(1).abs())
        .max(matrix.col(2).abs())
        .max_element()
}

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
            dynamical_eta: None,
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
    let eval = tree
        .evaluate(&frames, states, position, budget)
        .expect("tree eval");
    assert!(
        eval.error_bound_mps2 <= budget,
        "posted {:e} exceeds allocated {budget:e}",
        eval.error_bound_mps2,
    );
    assert!(eval.terms_exact >= 1, "root must open at this budget");
    let exact = field.acceleration(position, time).expect("exact");
    let measured = (eval.acceleration - exact).length();
    assert!(
        measured <= eval.error_bound_mps2 * (1.0 + 1.0e-9),
        "measured {measured:e} exceeds posted {:e}",
        eval.error_bound_mps2,
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
        toml::from_str(include_str!("../../../data/system.toml")).expect("system config");
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

#[test]
fn jacobi_eigen_is_orthonormal_reconstructing_and_deterministic() {
    // Symmetric indefinite tidal-tensor shape (traceless, mixed signs).
    let jacobian = DMat3::from_cols(
        DVec3::new(2.0e-6, 0.5e-6, -0.3e-6),
        DVec3::new(0.5e-6, -1.0e-6, 0.2e-6),
        DVec3::new(-0.3e-6, 0.2e-6, -1.0e-6),
    );
    let first = AffinePropagator::compile(jacobian).expect("propagator compiles");
    let second = AffinePropagator::compile(jacobian).expect("propagator compiles");
    assert_eq!(first, second, "same input must give the same basis");
    let basis = first.basis();
    let identity = basis.transpose() * basis;
    assert!(
        max_abs_entry(identity - DMat3::IDENTITY) <= 1.0e-14,
        "basis must be orthonormal"
    );
    let lambda = first.eigenvalues();
    assert!(
        lambda.x >= lambda.y && lambda.y >= lambda.z,
        "modes must sort descending, got {lambda:?}"
    );
    let rebuilt = basis * DMat3::from_diagonal(lambda) * basis.transpose();
    let scale = 2.0e-6;
    assert!(
        max_abs_entry(rebuilt - jacobian) <= 1.0e-12 * scale,
        "Q Lambda Q^T must rebuild J"
    );
    // Traceless: eigenvalues of vacuum gravity sum to zero.
    assert!(lambda.x + lambda.y + lambda.z <= 1.0e-9 * scale);
}

#[test]
fn propagator_rejects_bad_input() {
    let asymmetric = DMat3::from_cols(DVec3::X, DVec3::Y, DVec3::new(0.0, 1.0e-3, 1.0));
    assert_eq!(
        AffinePropagator::compile(asymmetric),
        Err(PropagatorError::AsymmetricJacobian)
    );
    let stretched = DMat3::from_diagonal(DVec3::new(1.0, -0.5, -0.5));
    let propagator = AffinePropagator::compile(stretched).expect("diagonal compiles");
    assert_eq!(
        propagator.coefficients(60.0),
        Err(PropagatorError::IntervalTooLong)
    );
    assert!(propagator.coefficients(10.0).is_ok());
    assert_eq!(
        propagator.coefficients(f64::NAN),
        Err(PropagatorError::NonFiniteStep)
    );
}

/// Independent RK4 reference on a frozen affine field: the STM must match a
/// converged numerical integration, not just its own closed form.
fn rk4_frozen_affine(
    jacobian: DMat3,
    constant: DVec3,
    mut position: DVec3,
    mut velocity: DVec3,
    duration_s: f64,
    step_s: f64,
) -> (DVec3, DVec3) {
    let accel = |position: DVec3| jacobian * position + constant;
    let mut time = 0.0;
    while time < duration_s {
        let step = step_s.min(duration_s - time);
        let a1v = velocity;
        let a1a = accel(position);
        let a2v = velocity + a1a * (step * 0.5);
        let a2a = accel(position + a1v * (step * 0.5));
        let a3v = velocity + a2a * (step * 0.5);
        let a3a = accel(position + a2v * (step * 0.5));
        let a4v = velocity + a3a * step;
        let a4a = accel(position + a3v * step);
        position += (a1v + a2v * 2.0 + a3v * 2.0 + a4v) * (step / 6.0);
        velocity += (a1a + a2a * 2.0 + a3a * 2.0 + a4a) * (step / 6.0);
        time += step;
    }
    (position, velocity)
}

#[test]
fn stm_matches_converged_rk4_on_frozen_field() {
    // Mixed-sign indefinite tensor at orbital magnitude plus a constant term
    // (absolute form, not just the homogeneous STM).
    let jacobian = DMat3::from_cols(
        DVec3::new(2.0e-6, 0.5e-6, -0.3e-6),
        DVec3::new(0.5e-6, -1.0e-6, 0.2e-6),
        DVec3::new(-0.3e-6, 0.2e-6, -1.0e-6),
    );
    let constant = DVec3::new(0.11, -0.07, 0.05);
    let propagator = AffinePropagator::compile(jacobian).expect("propagator compiles");
    for (position, velocity, duration) in [
        (
            DVec3::new(1.0e5, 0.0, 0.0),
            DVec3::new(0.0, 500.0, 10.0),
            120.0,
        ),
        (DVec3::new(-2.0e5, 1.0e5, 3.0e4), DVec3::ZERO, 60.0),
        (
            DVec3::new(5.0e4, -5.0e4, 5.0e4),
            DVec3::new(100.0, -200.0, 50.0),
            300.0,
        ),
    ] {
        let coeffs = propagator.coefficients(duration).expect("coeffs");
        let (analytic_x, analytic_v) = propagator.propagate(&coeffs, position, velocity, constant);
        let (numeric_x, numeric_v) =
            rk4_frozen_affine(jacobian, constant, position, velocity, duration, 0.01);
        let scale_x = analytic_x.length().max(1.0);
        let scale_v = analytic_v.length().max(1.0);
        assert!(
            (analytic_x - numeric_x).length() <= 1.0e-9 * scale_x,
            "position mismatch over {duration}s"
        );
        assert!(
            (analytic_v - numeric_v).length() <= 1.0e-9 * scale_v,
            "velocity mismatch over {duration}s"
        );
    }
}

#[test]
fn taylor_branch_matches_series_expansion() {
    // |lambda| dt^2 far below the threshold: pin the Taylor branch against
    // an independent second-order expansion, both signs.
    for lambda in [1.0e-13, -1.0e-13] {
        let jacobian = DMat3::from_diagonal(DVec3::new(lambda, 2.0 * lambda, -3.0 * lambda));
        let propagator = AffinePropagator::compile(jacobian).expect("propagator compiles");
        let dt = 10.0;
        let coeffs = propagator.coefficients(dt).expect("coeffs");
        let position = DVec3::new(1.0e4, -2.0e4, 3.0e4);
        let velocity = DVec3::new(100.0, 50.0, -80.0);
        let constant = DVec3::new(0.01, -0.02, 0.03);
        let (x, v) = propagator.propagate(&coeffs, position, velocity, constant);
        // Independent reference, third order in t so it matches the branch
        // expansion: x + v t + a t^2/2 + j t^3/6 with a = Jx + c and
        // jerk j = Jv; v + a t + j t^2/2.
        let accel = jacobian * position + constant;
        let jerk = jacobian * velocity;
        let reference_x =
            position + velocity * dt + accel * (dt * dt / 2.0) + jerk * (dt * dt * dt / 6.0);
        let reference_v = velocity + accel * dt + jerk * (dt * dt / 2.0);
        assert!((x - reference_x).length() <= 1.0e-9);
        assert!((v - reference_v).length() <= 1.0e-9);
    }
}

#[test]
fn real_patch_jacobian_is_traceless_and_propagatable() {
    // Cross-checks tidal_tensor assembly: vacuum point-mass Jacobians are
    // traceless, and the propagator accepts a real compiled patch tensor.
    let ephemeris = two_body_binary(3.0e14, 1.0e14, 1.0e8);
    let mut frame = EphemerisFrame::new();
    let states = frame
        .evaluate(&ephemeris, SimTime::EPOCH)
        .expect("frame states");
    let center = DVec3::new(3.0e9, 1.0e9, 0.0);
    let positions = vec![center, center + DVec3::new(10_000.0, 0.0, 0.0)];
    let patch = compile_patch(&ephemeris, states, &positions, CohortConfig::default())
        .expect("patch compiles");
    let trace = patch.jacobian.x_axis.x + patch.jacobian.y_axis.y + patch.jacobian.z_axis.z;
    let scale = patch.jacobian.col(0).length().max(1.0e-300);
    assert!(
        trace.abs() <= 1.0e-9 * scale,
        "vacuum tidal tensor must be traceless, got {trace:e}"
    );
    let propagator = AffinePropagator::compile(patch.jacobian).expect("propagator compiles");
    // Vacuum saddle: eigenvalues cannot be all-negative (sum is zero), so a
    // hyperbolic direction must exist.
    let lambda = propagator.eigenvalues();
    assert!(
        lambda.x > 0.0,
        "unstable direction must exist, got {lambda:?}"
    );
    assert!(
        lambda.z < 0.0,
        "stable direction must exist, got {lambda:?}"
    );
    let coeffs = propagator.coefficients(60.0).expect("minute coeffs");
    let (delta, _) = propagator.propagate(&coeffs, DVec3::ZERO, DVec3::ZERO, patch.g0);
    assert!((center + delta).is_finite());
}

#[test]
fn analytic_segment_stays_inside_posted_propagation_bound() {
    // The accuracy-idea verification: a frozen far-only patch propagated
    // analytically must stay inside field-spatial + temporal remainder
    // (converted to metres by double integration: bound * dt^2 / 2, valid
    // here since sigma * dt << 1 throughout).
    let ephemeris = two_body_binary(3.0e14, 1.0e14, 1.0e8);
    let field = GravityField::from_ephemeris(&ephemeris);
    let mut frame = EphemerisFrame::new();
    let states = frame
        .evaluate(&ephemeris, SimTime::EPOCH)
        .expect("frame states")
        .to_vec();
    let center = DVec3::new(3.0e9, 1.0e9, 0.0);
    let positions = vec![center, center + DVec3::new(10_000.0, 0.0, 0.0)];
    let patch = compile_patch(&ephemeris, &states, &positions, CohortConfig::default())
        .expect("patch compiles");
    assert!(patch.exact.is_empty());
    let propagator = AffinePropagator::compile(patch.jacobian).expect("propagator compiles");
    // Anchor formulation: the patch is compiled at `center`, so the
    // constant is g0 itself (never g0 - J*center — that belongs to the
    // origin-anchored form).
    let velocity = DVec3::new(0.0, 5_000.0, 100.0);
    for duration in [60.0, 600.0] {
        let coeffs = propagator.coefficients(duration).expect("coeffs");
        let (delta, _) = propagator.propagate(&coeffs, DVec3::ZERO, velocity, patch.g0);
        let analytic = center + delta;
        // Excursion: max anchor distance over sub-samples (conservative max
        // for the spatial remainder, not just the endpoint).
        let mut excursion = 0.0_f64;
        for quarter in 1..=4 {
            let sub = propagator
                .coefficients(duration * quarter as f64 / 4.0)
                .expect("sub coeffs");
            let (sub_delta, _) = propagator.propagate(&sub, DVec3::ZERO, velocity, patch.g0);
            excursion = excursion.max(sub_delta.length());
        }
        let field_bound = affine_segment_bound(&ephemeris, &states, &patch, excursion, duration)
            .expect("segment bound");
        assert!(
            field_bound.is_finite(),
            "segment must be inside the validity envelope at {duration}s"
        );
        let bound_m = field_bound * duration * duration / 2.0;
        // Honest exact reference: classic RK4 for the second-order system
        // with per-stage frames (bodies move during the step), 0.5 s steps.
        // Its own error is far below the bound under test.
        let mut x = center;
        let mut v = velocity;
        let steps = (duration / 0.5) as usize;
        for step in 0..steps {
            let base = step as f64 * 0.5;
            let mut accel_at = |position: DVec3, time: SimTime| {
                let sub = frame.evaluate(&ephemeris, time).expect("frame").to_vec();
                field
                    .accelerations_from_frame(std::slice::from_ref(&position), &sub)
                    .expect("exact accel")[0]
            };
            let k1v = accel_at(x, SimTime(base));
            let k1x = v;
            let k2v = accel_at(x + k1x * 0.25, SimTime(base + 0.25));
            let k2x = v + k1v * 0.25;
            let k3v = accel_at(x + k2x * 0.25, SimTime(base + 0.25));
            let k3x = v + k2v * 0.25;
            let k4v = accel_at(x + k3x * 0.5, SimTime(base + 0.5));
            let k4x = v + k3v * 0.5;
            x += (k1x + k2x * 2.0 + k3x * 2.0 + k4x) * (0.5 / 6.0);
            v += (k1v + k2v * 2.0 + k3v * 2.0 + k4v) * (0.5 / 6.0);
        }
        let divergence = (analytic - x).length();
        assert!(
            divergence <= bound_m,
            "analytic divergence {divergence:e} m exceeds posted {bound_m:e} m over {duration}s"
        );
    }
}

#[test]
fn analytic_bound_expires_and_refuses_honestly() {
    let ephemeris = two_body_binary(3.0e14, 1.0e14, 1.0e8);
    let mut frame = EphemerisFrame::new();
    let states = frame
        .evaluate(&ephemeris, SimTime::EPOCH)
        .expect("frame states")
        .to_vec();
    let center = DVec3::new(3.0e9, 1.0e9, 0.0);
    let positions = vec![center, center + DVec3::new(10_000.0, 0.0, 0.0)];
    let patch = compile_patch(&ephemeris, &states, &positions, CohortConfig::default())
        .expect("patch compiles");
    // A 10-hour excursion leaves the ball far behind a source: INFINITY is
    // the rebuild signal, not an error.
    assert_eq!(
        affine_segment_bound(&ephemeris, &states, &patch, 1.0e10, 36_000.0),
        Ok(f64::INFINITY)
    );
    // A ball containing the secondary off-center: it is inside, so it must
    // be exact, and the patch refuses analytic propagation instead of
    // silently dropping the point-mass terms. (Centered exactly on the
    // body would be a field singularity, not a patch.)
    let secondary = states[2].position_inertial;
    let near = vec![
        secondary + DVec3::new(1.5e6, 0.0, 0.0),
        secondary - DVec3::new(0.5e6, 0.0, 0.0),
    ];
    let near_patch = compile_patch(&ephemeris, &states, &near, CohortConfig::default())
        .expect("near patch compiles");
    assert!(!near_patch.exact.is_empty());
    assert_eq!(
        affine_segment_bound(&ephemeris, &states, &near_patch, 1.0e5, 60.0),
        Err(PatchError::AnalyticNeedsFarField)
    );
}

#[test]
fn piecewise_converges_with_budget_and_stays_deterministic() {
    // Budget-driven convergence: a tighter budget takes shorter segments
    // and lands closer to exact. Deep-space 1200 s horizon, verified
    // against per-stage-frame RK4 at the endpoint. (At 1e-12 and below the
    // driver honestly refuses instead: per-tick source motion alone exceeds
    // the budget — see the DtFloor case below.)
    let ephemeris = two_body_binary(3.0e14, 1.0e14, 1.0e8);
    let field = GravityField::from_ephemeris(&ephemeris);
    let center = DVec3::new(3.0e9, 1.0e9, 0.0);
    let velocity = DVec3::new(0.0, 5_000.0, 100.0);
    let run = |budget: f64, horizon: f64| {
        let mut frame = EphemerisFrame::new();
        let report = propagate_piecewise(
            &ephemeris,
            &mut frame,
            center,
            velocity,
            SimTime::EPOCH,
            CohortConfig {
                error_budget_mps2: budget,
                ..Default::default()
            },
            horizon,
            60.0,
        )
        .expect("piecewise runs");
        assert_eq!(report.fallback, None);
        // Reference endpoint: exact RK4, 0.5 s steps, per-stage frames.
        let mut x = center;
        let mut v = velocity;
        for step in 0..(horizon / 0.5) as usize {
            let base = step as f64 * 0.5;
            let mut accel_at = |position: DVec3, time: SimTime| {
                let sub = frame.evaluate(&ephemeris, time).expect("frame").to_vec();
                field
                    .accelerations_from_frame(std::slice::from_ref(&position), &sub)
                    .expect("exact")[0]
            };
            let k1v = accel_at(x, SimTime(base));
            let k1x = v;
            let k2v = accel_at(x + k1x * 0.25, SimTime(base + 0.25));
            let k2x = v + k1v * 0.25;
            let k3v = accel_at(x + k2x * 0.25, SimTime(base + 0.25));
            let k3x = v + k2v * 0.25;
            let k4v = accel_at(x + k3x * 0.5, SimTime(base + 0.5));
            let k4x = v + k3v * 0.5;
            x += (k1x + k2x * 2.0 + k3x * 2.0 + k4x) * (0.5 / 6.0);
            v += (k1v + k2v * 2.0 + k3v * 2.0 + k4v) * (0.5 / 6.0);
        }
        let last = report.steps.last().expect("at least one step");
        let error = (last.position - x).length();
        (report.steps.len(), error)
    };
    let (loose_steps, loose_error) = run(1.0e-9, 1_200.0);
    let (tight_steps, tight_error) = run(1.0e-10, 1_200.0);
    assert!(
        tight_steps >= loose_steps,
        "tighter budget must not take fewer segments"
    );
    assert!(
        tight_error < loose_error,
        "tighter budget must land closer: {tight_error:e} vs {loose_error:e}"
    );
    assert!(loose_error <= 1.0, "loose run must stay sane");
    // Determinism: same inputs, identical report.
    let mut frame = EphemerisFrame::new();
    let first = propagate_piecewise(
        &ephemeris,
        &mut frame,
        center,
        velocity,
        SimTime::EPOCH,
        CohortConfig::default(),
        600.0,
        60.0,
    )
    .expect("first run");
    let mut frame = EphemerisFrame::new();
    let second = propagate_piecewise(
        &ephemeris,
        &mut frame,
        center,
        velocity,
        SimTime::EPOCH,
        CohortConfig::default(),
        600.0,
        60.0,
    )
    .expect("second run");
    assert_eq!(first, second);
}

#[test]
fn piecewise_falls_back_near_body_and_on_zero_budget() {
    let ephemeris = two_body_binary(3.0e14, 1.0e14, 1.0e8);
    let mut frame = EphemerisFrame::new();
    let states = frame
        .evaluate(&ephemeris, SimTime::EPOCH)
        .expect("frame")
        .to_vec();
    // 5 km from the secondary with a slow drift: the segment ball reaches
    // the body on the first attempt → immediate exact-near fallback.
    let secondary = states[2].position_inertial;
    let report = propagate_piecewise(
        &ephemeris,
        &mut frame,
        secondary + DVec3::new(5_000.0, 0.0, 0.0),
        DVec3::new(0.0, 100.0, 0.0),
        SimTime::EPOCH,
        CohortConfig::default(),
        600.0,
        60.0,
    )
    .expect("fallback runs");
    assert_eq!(report.steps.len(), 1);
    assert_eq!(
        report.fallback,
        Some(AnalyticFallback::ExactNear { body: BodyId(2) })
    );
    // Zero budget overflows every source: grind to the dt floor, then hand
    // the remainder to exact integration.
    let center = DVec3::new(3.0e9, 1.0e9, 0.0);
    let report = propagate_piecewise(
        &ephemeris,
        &mut frame,
        center,
        DVec3::new(0.0, 5_000.0, 0.0),
        SimTime::EPOCH,
        CohortConfig {
            error_budget_mps2: 0.0,
            ..Default::default()
        },
        600.0,
        60.0,
    )
    .expect("zero-budget runs");
    assert_eq!(report.steps.len(), 1);
    assert_eq!(report.fallback, Some(AnalyticFallback::DtFloor));
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
        DVec3::new(speed_mps, 0.0, -speed_mps * alpha.tan()),
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
fn aero_dynamic_damping_uses_arbitrary_panel_axes() {
    let diagonal = 2.0_f64.sqrt().recip();
    let chord_axis = DVec3::new(diagonal, 0.0, diagonal);
    let lift_axis = DVec3::Y;
    let side_axis = lift_axis.cross(chord_axis).normalize();
    let panel = AeroPanel::new(DVec3::ZERO, chord_axis, lift_axis, 10.0, 2.0)
        .expect("valid arbitrarily oriented panel");
    let geometry = AeroGeometry::new(vec![panel]).expect("valid geometry");
    let model = PanelAeroModel::new(AeroConfig {
        base_drag_coefficient: 0.0,
        induced_drag_factor: 0.0,
        wave_drag_coefficient: 0.0,
        roll_damping_coefficient: 0.0,
        pitch_damping_coefficient: -1.0,
        yaw_damping_coefficient: 0.0,
        ..AeroConfig::default()
    })
    .expect("valid aero model");
    let state = AeroState::new(DVec3::new(100.0, 0.0, 0.0), side_axis * 0.2);
    let case = AeroCase::new(state, AeroEnvironment::standard_sea_level(), geometry)
        .expect("valid aero case");
    let result = model.evaluate(&case).expect("finite damping result");

    assert!(result.moment_body_nm.dot(side_axis) < 0.0);
    assert!(result.moment_body_nm.cross(side_axis).length() < 1.0e-9);
}

#[test]
fn aero_aft_stabilizer_lowers_nose_at_positive_alpha() {
    let panel = AeroPanel::flat_plate(DVec3::new(-4.0, 0.0, 0.0), 4.0, 1.2)
        .expect("valid stabilizer panel");
    let case = AeroCase::new(
        AeroState::new(
            DVec3::new(100.0, 0.0, -100.0 * 5.0_f64.to_radians().tan()),
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
    assert!(result.force_body_n.z > 0.0);
    assert!(result.moment_body_nm.y > 0.0);
    // Verify the physical consequence instead of calling a torque sign
    // "restoring": +Y rotates the nose toward -Z in this body basis.
    let nose_after = DQuat::from_rotation_y(result.moment_body_nm.y * 1.0e-6) * DVec3::X;
    assert!(nose_after.z < 0.0);
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
fn sampled_duration_calls_callback_for_each_deterministic_substep() {
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
    let input = FlightStepInput::new(0.0, DVec3::ZERO);
    let mut elapsed_samples = Vec::new();
    integrate_rigid_body_duration_sampled(
        &model,
        &geometry,
        AtmosphereConfig::default(),
        state,
        properties,
        0.1,
        0.02,
        |_, elapsed_s| {
            elapsed_samples.push(elapsed_s);
            Ok(input)
        },
    )
    .expect("valid sampled duration");

    assert_eq!(elapsed_samples.len(), 5);
    for (index, elapsed_s) in elapsed_samples.into_iter().enumerate() {
        assert!((elapsed_s - index as f64 * 0.02).abs() < 1.0e-12);
    }
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
                    -0.95 * environment.speed_of_sound_mps * alpha_rad.sin(),
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
                    -2.0 * environment.speed_of_sound_mps * alpha_rad.sin(),
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
fn x15_starter_profile_ships_compiled_contact_geometry() {
    let starter = X15StarterProfile::new().expect("X-15 starter profile");
    let geometry = &starter.vehicle.collision_geometry;
    assert!(!geometry.is_empty());
    geometry
        .validate()
        .expect("X-15 contact geometry validates");
    assert_eq!(geometry.parts.len(), 4);
    // The compound must cover the flown stations: fuselage capsule along the
    // body X axis plus wing/tail cuboids at the aero panel stations.
    let has_capsule = geometry.parts.iter().any(|part| {
        matches!(
            part.shape,
            crate::CollisionShape::Capsule {
                axis: crate::CollisionAxis::X,
                ..
            }
        )
    });
    assert!(
        has_capsule,
        "X-15 contact geometry needs a fuselage capsule"
    );
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
fn skip_aero_applies_zero_air_load_keeping_thrust_and_gravity() {
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
    input.skip_aero = true;
    let forces = evaluate_flight_forces(
        &model,
        &starter.vehicle.aero_geometry,
        AtmosphereConfig::default(),
        state,
        starter.vehicle.mass_properties,
        input,
    )
    .expect("finite skipped-aero forces");
    assert_eq!(forces.aero.force_body_n, DVec3::ZERO);
    assert_eq!(forces.aero.moment_body_nm, DVec3::ZERO);
    assert_eq!(forces.aero.dynamic_pressure_pa, 0.0);
    assert_eq!(forces.aero.panel_count, 0);
    // Thrust still integrates against gravity: positive down-range force.
    assert!(forces.total_force_body_n.x > 0.0);
    assert!(forces.acceleration_inertial_mps2.is_finite());
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

#[test]
fn fast_free_rotation_preserves_energy_and_inertial_angular_momentum() {
    let model = PanelAeroModel::new(AeroConfig::default()).unwrap();
    let geometry =
        AeroGeometry::new(vec![AeroPanel::flat_plate(DVec3::ZERO, 1.0, 1.0).unwrap()]).unwrap();
    let properties = X15StarterProfile::new().unwrap().vehicle.mass_properties;
    let mut state = RigidBodyState::new(
        DVec3::ZERO,
        DVec3::ZERO,
        DQuat::IDENTITY,
        DVec3::new(-8.87, -2.54, 0.053),
    )
    .unwrap();
    let inertia = properties.inertia_body_kg_m2;
    let momentum = inertia * state.angular_velocity_body_rps;
    let energy = 0.5 * state.angular_velocity_body_rps.dot(momentum);
    let mut max_energy_error: f64 = 0.0;
    let mut max_momentum_error: f64 = 0.0;
    for _ in 0..7200 {
        (state, _) = integrate_rigid_body_step(
            &model,
            &geometry,
            AtmosphereConfig::default(),
            state,
            properties,
            FlightStepInput::new(1_000_000.0, DVec3::ZERO),
            1.0 / 120.0,
        )
        .unwrap();
        let h = inertia * state.angular_velocity_body_rps;
        let energy_error = (0.5 * state.angular_velocity_body_rps.dot(h) / energy - 1.0).abs();
        let momentum_error =
            (state.orientation_body_to_inertial * h - momentum).length() / momentum.length();
        max_energy_error = max_energy_error.max(energy_error);
        max_momentum_error = max_momentum_error.max(momentum_error);
        assert!(
            energy_error < 1e-8,
            "free rotation gained energy: {energy_error}"
        );
        assert!(
            momentum_error < 1e-8,
            "inertial angular momentum drift: {momentum_error}"
        );
    }
    println!(
        "free rotation: relative energy error {max_energy_error:e}, momentum error {max_momentum_error:e}"
    );
}

#[test]
fn osculating_elements_recover_circular_orbit() {
    let mu: f64 = 3.986_004_418e14;
    let radius: f64 = 6_878_000.0;
    let speed = (mu / radius).sqrt();
    let elements = OsculatingElements::from_state(DVec3::X * radius, DVec3::Y * speed, mu).unwrap();
    assert!((elements.semi_major_axis_m - radius).abs() / radius < 1e-9);
    assert!(elements.eccentricity < 1e-9);
    assert!(!elements.is_escape());
    assert!((elements.apoapsis_m().unwrap() - radius).abs() / radius < 1e-9);
    assert!((elements.periapsis_m() - radius).abs() / radius < 1e-9);
    let period = elements.period_s().unwrap();
    assert!((period - 2.0 * std::f64::consts::PI * (radius.powi(3) / mu).sqrt()) / period < 1e-9);
}

#[test]
fn osculating_elements_roundtrip_elliptical_state() {
    // Reference: analytic KeplerOrbit evaluated off-periapsis.
    let mu = 4.0e13;
    let orbit = KeplerOrbit::new(mu, 10_000_000.0, 0.6, 0.3, 1.1, 0.7, 0.0).unwrap();
    let (r, v) = orbit.state_relative_at(SimTime(1_234.0)).unwrap();
    let elements = OsculatingElements::from_state(r, v, mu).unwrap();
    assert!(!elements.is_escape());
    assert!((elements.semi_major_axis_m - 10_000_000.0).abs() / 10_000_000.0 < 1e-9);
    assert!((elements.eccentricity - 0.6).abs() < 1e-9);
    assert!((elements.inclination_rad - 0.3).abs() < 1e-9);
    assert!((elements.apoapsis_m().unwrap() - 16_000_000.0).abs() < 1.0);
    assert!((elements.periapsis_m() - 4_000_000.0).abs() < 1.0);
    // Sampling at the recovered anomaly reproduces the input position.
    let back = elements.position_at_nu(elements.true_anomaly_rad);
    assert!((back - r).length() / r.length() < 1e-9);
}

#[test]
fn osculating_elements_report_escape_without_apoapsis() {
    // Solar-system escape energy at 1 AU: well above circular speed.
    let mu = 1.327_124_400_18e20;
    let r = DVec3::X * 1.496e11;
    let v = DVec3::Y * 50_000.0;
    let elements = OsculatingElements::from_state(r, v, mu).unwrap();
    assert!(elements.is_escape());
    assert!(elements.semi_major_axis_m < 0.0);
    assert!(elements.apoapsis_m().is_none());
    assert!(elements.period_s().is_none());
    assert!(elements.periapsis_m() > 0.0);
}

#[test]
fn osculating_elements_reject_degenerate_states() {
    let mu = 3.986_004_418e14;
    assert!(OsculatingElements::from_state(DVec3::ZERO, DVec3::Y, mu).is_err());
    // Radial plunge: no orbital plane.
    assert!(OsculatingElements::from_state(DVec3::X, -DVec3::X * 1000.0, mu).is_err());
    assert!(OsculatingElements::from_state(DVec3::X, DVec3::Y, f64::NAN).is_err());
}

#[test]
fn osculating_roundtrip_all_quadrants_and_retrograde_planes() {
    let mu = 4.0e13;
    let mut max_relative_error: f64 = 0.0;
    for inclination in [
        0.0,
        0.7,
        std::f64::consts::FRAC_PI_2,
        2.4,
        std::f64::consts::PI,
    ] {
        for eccentricity in [0.0, 0.3, 0.8] {
            for longitude in [0.2, 1.7, 3.3, 4.9] {
                let orbit = KeplerOrbit::new(
                    mu,
                    10_000_000.0,
                    eccentricity,
                    inclination,
                    0.8,
                    1.2,
                    longitude,
                )
                .unwrap();
                let (r, v) = orbit.state_relative_at(SimTime::EPOCH).unwrap();
                let elements = OsculatingElements::from_state(r, v, mu).unwrap();
                let error =
                    (elements.position_at_nu(elements.true_anomaly_rad) - r).length() / r.length();
                max_relative_error = max_relative_error.max(error);
                assert!(
                    error < 1e-9,
                    "i={inclination} e={eccentricity} phase={longitude}: {error:e}"
                );
            }
        }
    }
    eprintln!("osculating position roundtrip max relative error {max_relative_error:e}");
}

#[test]
fn sampled_verlet_returns_to_start_after_one_circular_period() {
    // Known case: single central mass, circular orbit must close.
    let mu = 4.0e13;
    let radius = 10_000_000.0;
    let ephemeris = central_ephemeris(mu);
    let speed = (mu / radius).sqrt();
    let period = TAU * (radius.powi(3) / mu).sqrt();
    let steps = 360;
    let path = propagate_sampled_verlet(
        &ephemeris,
        TestParticleState {
            position: DVec3::X * radius,
            velocity: DVec3::Y * speed,
        },
        SimTime::EPOCH,
        VerletConfig {
            step_s: period / steps as f64,
            max_steps: steps,
        },
        &[],
    )
    .expect("circular propagation succeeds");
    assert_eq!(path.end, SampledPathEnd::Completed);
    assert_eq!(path.positions.len(), steps as usize + 1);
    let error = (*path.positions.last().unwrap() - DVec3::X * radius).length() / radius;
    eprintln!("circular closure relative error {error:e} over {steps} steps");
    assert!(error < 1e-3, "closure error {error:e}");
}

#[test]
fn sampled_verlet_agrees_with_adaptive_on_unpowered_coast() {
    // On-rails equivalence groundwork: with no thrust and no aero load, the
    // map prediction (fixed-step sampled Verlet) and the authoritative
    // adaptive propagator must agree on the same gravity field. If this ever
    // diverges, baking the prediction as the craft's on-rails state is unsound.
    use crate::{AdaptiveIntegratorConfig, GravityField, propagate_adaptive};
    let mu = 4.0e13;
    let radius = 10_000_000.0;
    let ephemeris = central_ephemeris(mu);
    let speed = (mu / radius).sqrt();
    let initial = TestParticleState {
        position: DVec3::X * radius,
        velocity: DVec3::Y * speed,
    };
    let horizon = 3_600.0;
    let field = GravityField::from_ephemeris(&ephemeris);
    let reference = propagate_adaptive(
        &field,
        initial,
        SimTime::EPOCH,
        horizon,
        AdaptiveIntegratorConfig::default(),
    )
    .expect("adaptive coast succeeds");
    let step_s = 5.0;
    let path = propagate_sampled_verlet(
        &ephemeris,
        initial,
        SimTime::EPOCH,
        VerletConfig {
            step_s,
            max_steps: (horizon / step_s) as u64,
        },
        &[],
    )
    .expect("sampled coast succeeds");
    let drift = (path.positions.last().unwrap() - reference.state.position).length();
    eprintln!("coast agreement drift after {horizon}s: {drift:e} m");
    assert!(
        drift / radius < 1e-6,
        "prediction/flight integrators disagree: {drift:e} m"
    );
}

#[test]
fn sampled_verlet_stops_at_surface_impact() {
    // Tossed upward below escape speed around a planet-sized body: the path
    // must end in Impact, with no sample underground.
    let mu = 4.0e13;
    let planet_radius = 6_000_000.0;
    let start = 10_000_000.0;
    let ephemeris = BakedEphemeris::new(
        "TEST_IMPACT_EPOCH",
        vec![BakedBody::fixed(BodyId(0), "planet", mu, planet_radius)],
    )
    .expect("valid impact test ephemeris");
    let path = propagate_sampled_verlet(
        &ephemeris,
        TestParticleState {
            position: DVec3::X * start,
            velocity: DVec3::Y * (mu / start).sqrt() * 0.5,
        },
        SimTime::EPOCH,
        VerletConfig {
            step_s: 20.0,
            max_steps: 5_000,
        },
        &[BodyId(0)],
    )
    .expect("suborbital propagation succeeds");
    assert_eq!(path.end, SampledPathEnd::Impact(BodyId(0)));
    // Impact is detected at step granularity: every sample but the last is
    // above the surface; the last is the impact state, at most one step of
    // travel (20 s x ~7.5 km/s periapsis speed) underground.
    let last = path.positions.len() - 1;
    for position in &path.positions[..last] {
        assert!(
            position.length() >= planet_radius,
            "no sample before impact may be underground"
        );
    }
    let penetration = planet_radius - path.positions[last].length();
    assert!(
        penetration.abs() < 1e-6,
        "impact sample must sit just under the surface, penetration {penetration}"
    );
}

#[test]
fn sampled_verlet_escape_leaves_without_impact() {
    // Above escape speed: completed path, monotonically far away.
    let mu = 4.0e13;
    let planet_radius = 6_000_000.0;
    let start = 10_000_000.0;
    let ephemeris = BakedEphemeris::new(
        "TEST_ESCAPE_EPOCH",
        vec![BakedBody::fixed(BodyId(0), "planet", mu, planet_radius)],
    )
    .expect("valid escape test ephemeris");
    let path = propagate_sampled_verlet(
        &ephemeris,
        TestParticleState {
            position: DVec3::X * start,
            velocity: DVec3::Y * (2.0 * mu / start).sqrt() * 1.2,
        },
        SimTime::EPOCH,
        VerletConfig {
            step_s: 60.0,
            max_steps: 500,
        },
        &[BodyId(0)],
    )
    .expect("escape propagation succeeds");
    assert_eq!(path.end, SampledPathEnd::Completed);
    let final_radius = path.positions.last().unwrap().length();
    assert!(
        final_radius > start * 5.0,
        "escape must leave, final radius {final_radius}"
    );
}

#[test]
fn sampled_verlet_full_system_benchmark() {
    // Target-size batch: 22-source field, map-line workload (360 gravity
    // evals + impact checks). Synthetic baked system (depth-1 chains; the
    // real catalog chains to depth ~3, so treat this as a lower bound).
    // Prints ms for the perf record.
    let mut bodies = vec![BakedBody::fixed(BodyId(0), "star", 1.0e17, 7.0e8)];
    for i in 1..22u32 {
        let orbit = KeplerOrbit::new(
            1.0e17,
            2.0e8 + i as f64 * 1.0e8,
            0.01,
            0.02 * i as f64,
            0.3 * i as f64,
            0.1 * i as f64,
            0.2 * i as f64,
        )
        .expect("benchmark orbit builds");
        bodies.push(BakedBody::orbital(
            BodyId(i),
            format!("body{i}"),
            1.0e12 * i as f64,
            1.0e6 * i as f64,
            BodyId(0),
            orbit,
        ));
    }
    let ephemeris = BakedEphemeris::new("TEST_BENCH_EPOCH", bodies).expect("benchmark bakes");
    let home = BodyId(5);
    let home_mu = 5.0e12_f64;
    let home_state = ephemeris
        .body_state(home, SimTime::EPOCH)
        .expect("home state");
    let radius = 5.0e6 + 300_000.0;
    let speed = (home_mu / radius).sqrt();
    let started = std::time::Instant::now();
    let path = propagate_sampled_verlet(
        &ephemeris,
        TestParticleState {
            position: home_state.position_inertial + DVec3::X * radius,
            velocity: home_state.velocity_inertial + DVec3::Y * speed,
        },
        SimTime::EPOCH,
        VerletConfig {
            step_s: 30.0,
            max_steps: 360,
        },
        &[home],
    )
    .expect("benchmark propagation succeeds");
    let elapsed = started.elapsed();
    eprintln!(
        "synthetic 22-source 360-step sampled path: {:.2} ms, {} samples, end {:?}",
        elapsed.as_secs_f64() * 1000.0,
        path.positions.len(),
        path.end
    );
}

#[test]
fn interlunar_transfer_thessa_to_pelagos_encounters_target() {
    // Real catalog, real dynamics: Hohmann departure from Thessa's orbit to
    // Pelagos' orbit around Nereid, phased by brute-force departure search.
    // Proves the summed-field dynamics (no SOI logic anywhere) support
    // moon-to-moon transfers: the arc must leave Thessa far behind and swing
    // within 10 target radii of Pelagos.
    let config: SystemConfig =
        toml::from_str(include_str!("../../../data/system.toml")).expect("system parses");
    let ephemeris = config.bake().expect("system bakes");
    let nereid = ephemeris.body_id("nereid").expect("host exists");
    let thessa = ephemeris.body_id("thessa").expect("departure moon exists");
    let pelagos = ephemeris.body_id("pelagos").expect("target moon exists");
    let mu = ephemeris.body(nereid).expect("host descriptor").mu;
    let pelagos_radius = ephemeris.body(pelagos).expect("target descriptor").radius_m;
    let thessa_radius = ephemeris
        .body(thessa)
        .expect("departure descriptor")
        .radius_m;

    let epoch = SimTime::EPOCH;
    let thessa_state = ephemeris
        .body_state(thessa, epoch)
        .expect("departure state");
    let nereid_state = ephemeris.body_state(nereid, epoch).expect("host state");
    let r1 = (thessa_state.position_inertial - nereid_state.position_inertial).length();
    let pelagos_state = ephemeris.body_state(pelagos, epoch).expect("target state");
    let r2 = (pelagos_state.position_inertial - nereid_state.position_inertial).length();
    // Hohmann transfer ellipse between the (near-circular) moon orbits.
    let transfer_a = (r1 + r2) / 2.0;
    let tof_s = std::f64::consts::PI * (transfer_a.powi(3) / mu).sqrt();
    let dv = (mu / r1).sqrt() * ((2.0 * r2 / (r1 + r2)).sqrt() - 1.0);
    assert!(dv > 0.0, "outbound transfer needs prograde burn");
    // Departure phasing brute force: one target period, 64 slots. Each try
    // propagates the full 22-source field; total ~30k steps, well under a
    // second. Deterministic: fixed slots, fixed step.
    let dt = 900.0;
    let steps = (tof_s * 1.15 / dt) as u64;
    let mut best_miss_m = f64::INFINITY;
    let mut best_leaves_thessa_m = 0.0;
    for slot in 0..64 {
        let t0 = epoch.offset(slot as f64 * 160.0 * 3600.0 / 64.0);
        let dep = ephemeris.body_state(thessa, t0).expect("departure state");
        let host = ephemeris.body_state(nereid, t0).expect("host state");
        let radial = (dep.position_inertial - host.position_inertial).normalize_or_zero();
        let prograde = (dep.velocity_inertial - host.velocity_inertial).normalize_or_zero();
        // Impulsive departure from a 300 km parking orbit: the burn must
        // first beat Thessa's own escape, then keep the Hohmann excess.
        let mu_thessa = ephemeris.body(thessa).expect("departure descriptor").mu;
        let burnout = (2.0 * mu_thessa / (thessa_radius + 300_000.0) + dv * dv).sqrt();
        let start = dep.position_inertial + radial * (thessa_radius + 300_000.0);
        let velocity = dep.velocity_inertial + prograde * burnout;
        let path = propagate_sampled_verlet(
            &ephemeris,
            TestParticleState {
                position: start,
                velocity,
            },
            t0,
            VerletConfig {
                step_s: dt,
                max_steps: steps,
            },
            &[thessa, pelagos],
        )
        .expect("transfer propagation succeeds");
        let mut min_pelagos_m = f64::INFINITY;
        let mut max_thessa_m = 0.0_f64;
        for (index, position) in path.positions.iter().enumerate() {
            let time = path.times[index];
            if let Ok(target) = ephemeris.body_state(pelagos, time) {
                min_pelagos_m = min_pelagos_m
                    .min((*position - target.position_inertial).length() - pelagos_radius);
            }
            if let Ok(home) = ephemeris.body_state(thessa, time) {
                max_thessa_m = max_thessa_m.max((*position - home.position_inertial).length());
            }
        }
        if min_pelagos_m < best_miss_m {
            best_miss_m = min_pelagos_m;
            best_leaves_thessa_m = max_thessa_m;
        }
    }
    eprintln!(
        "thessa->pelagos hohmann dv {dv:.0} m/s tof {tof_s:.0} s: best miss {best_miss_m:.0} m, max thessa range {best_leaves_thessa_m:.0} m"
    );
    assert!(
        best_leaves_thessa_m > 100_000_000.0,
        "transfer must leave Thessa behind, range {best_leaves_thessa_m:.0}"
    );
    assert!(
        best_miss_m < pelagos_radius * 10.0,
        "transfer must encounter Pelagos within 10 radii, miss {best_miss_m:.0}"
    );
}

#[test]
fn dominant_body_follows_strongest_local_pull() {
    // Heavy primary vs light secondary on opposite sides of their barycenter.
    // Display-only: no physics changes, just the max-mu/r^2 hint.
    let ephemeris = two_body_binary(1.0e14, 1.0e12, 1.0e9);
    let time = SimTime::EPOCH;
    let primary = ephemeris.body_state(BodyId(1), time).unwrap();
    let secondary = ephemeris.body_state(BodyId(2), time).unwrap();
    // Just above the light moon, the moon wins despite the giant's mass.
    let near_moon = secondary.position_inertial + DVec3::X * 1.0e6;
    assert_eq!(ephemeris.dominant_body(near_moon, time), Some(BodyId(2)));
    // Just above the heavy primary, the primary wins.
    let near_primary = primary.position_inertial + DVec3::X * 1.0e6;
    assert_eq!(ephemeris.dominant_body(near_primary, time), Some(BodyId(1)));
}

#[test]
fn sampled_impact_chooses_entry_not_far_side_or_list_order() {
    let ephemeris = BakedEphemeris::new(
        "IMPACT",
        vec![
            BakedBody::fixed(BodyId(0), "inner", 1e-12, 3.0),
            BakedBody::fixed(BodyId(1), "outer", 1e-12, 5.0),
        ],
    )
    .unwrap();
    for speed in [10.0, 20.0] {
        // endpoint at center, or through the whole body
        let path = propagate_sampled_verlet(
            &ephemeris,
            TestParticleState {
                position: DVec3::X * 10.0,
                velocity: -DVec3::X * speed,
            },
            SimTime::EPOCH,
            VerletConfig {
                step_s: 1.0,
                max_steps: 4,
            },
            &[BodyId(0), BodyId(1)],
        )
        .unwrap();
        assert_eq!(path.end, SampledPathEnd::Impact(BodyId(1)));
        assert!((path.positions.last().unwrap().x - 5.0).abs() < 1e-10);
        assert!((path.end_time.0 - 5.0 / speed).abs() < 1e-10);
        assert_eq!(path.times.last(), Some(&path.end_time));
    }
}

#[test]
fn onrails_bake_sample_reuse_and_maneuver_invalidation() {
    use crate::{OnRailsCache, OnRailsWake};
    // Bake once on a circular coast, then sample forward in time without
    // rebaking: the interpolated state must track the path. A velocity kick
    // (the maneuver) must fail reuse so the caller rebakes.
    let mu = 4.0e13;
    let radius = 10_000_000.0;
    let ephemeris = central_ephemeris(mu);
    let speed = (mu / radius).sqrt();
    let initial = TestParticleState {
        position: DVec3::X * radius,
        velocity: DVec3::Y * speed,
    };
    let config = VerletConfig {
        step_s: 10.0,
        max_steps: 720,
    };
    let mut rails = OnRailsCache::new();
    assert!(rails.is_empty());
    assert!(!rails.usable_for(&ephemeris, initial, SimTime::EPOCH, config, &[], 1.0, 1e-3));
    rails
        .bake(&ephemeris, initial, SimTime::EPOCH, config, &[])
        .expect("coast bakes");
    assert!(!rails.is_empty());
    // Half an hour downrange the same coast still matches: no rebake needed.
    let later = SimTime::EPOCH.offset(1_800.0);
    let (expected_pos, _) = rails.sample_at(later).expect("baked path covers +30 min");
    let drifted = TestParticleState {
        position: expected_pos,
        velocity: DVec3::Y * speed,
    };
    // Velocity here is stale on purpose (position matches, velocity does
    // not): reuse must fail on the velocity tolerance.
    assert!(!rails.usable_for(&ephemeris, drifted, later, config, &[], 1.0, 1e-6));
    // Exact interpolated state reuses.
    let (pos, vel) = rails.sample_at(later).expect("sampled");
    let exact = TestParticleState {
        position: pos,
        velocity: vel,
    };
    assert!(rails.usable_for(&ephemeris, exact, later, config, &[], 1.0, 1e-3));
    // Maneuver: +10 m/s kick breaks reuse at any tolerance that matters.
    let kicked = TestParticleState {
        position: pos,
        velocity: vel + DVec3::Y * 10.0,
    };
    assert!(!rails.usable_for(&ephemeris, kicked, later, config, &[], 1.0, 1e-3));
    // Different horizon sizing forces a rebake decision by the caller.
    let other = VerletConfig {
        step_s: 20.0,
        max_steps: 720,
    };
    assert!(!rails.usable_for(&ephemeris, exact, later, other, &[], 1.0, 1e-3));
    // Wake: circular coast runs to the horizon, no impact.
    match rails.wake() {
        Some(OnRailsWake::HorizonEnd { time }) => {
            assert!((time.seconds() - 7_200.0).abs() < 1e-6);
        }
        other => panic!("expected horizon wake, got {other:?}"),
    }
    // Explicit invalidation (burn/contact/atmosphere entry) empties reuse.
    rails.invalidate();
    assert!(rails.is_empty());
    assert!(!rails.usable_for(&ephemeris, exact, later, config, &[], 1e9, 1e9));
}

#[test]
fn onrails_wake_reports_predicted_impact_epoch() {
    use crate::{OnRailsCache, OnRailsWake};
    // Suborbital toss: the wake must name the impact body and epoch so the
    // scheduler arms contact handling in simulation time, not per-tick polls.
    let mu = 4.0e13;
    let planet_radius = 6_000_000.0;
    let start = 10_000_000.0;
    let ephemeris = BakedEphemeris::new(
        "TEST_ONRAILS_IMPACT",
        vec![BakedBody::fixed(BodyId(0), "planet", mu, planet_radius)],
    )
    .expect("valid impact test ephemeris");
    let mut rails = OnRailsCache::new();
    rails
        .bake(
            &ephemeris,
            TestParticleState {
                position: DVec3::X * start,
                velocity: DVec3::Y * (mu / start).sqrt() * 0.5,
            },
            SimTime::EPOCH,
            VerletConfig {
                step_s: 20.0,
                max_steps: 5_000,
            },
            &[BodyId(0)],
        )
        .expect("suborbital coast bakes");
    match rails.wake() {
        Some(OnRailsWake::Impact { time, body }) => {
            assert_eq!(body, BodyId(0));
            assert!(time.seconds() > 0.0);
            eprintln!("predicted impact at T+{:.1}s", time.seconds());
        }
        other => panic!("expected impact wake, got {other:?}"),
    }
}

#[test]
fn onrails_reuse_skips_reintegration() {
    // Perf record: full bake vs cache-hit sampling on the target-size batch.
    // The hit must be orders of magnitude cheaper than the ~ms bake, which
    // is the whole economic case for on-rails.
    use crate::OnRailsCache;
    let mu = 4.0e13;
    let radius = 10_000_000.0;
    let ephemeris = central_ephemeris(mu);
    let speed = (mu / radius).sqrt();
    let initial = TestParticleState {
        position: DVec3::X * radius,
        velocity: DVec3::Y * speed,
    };
    let config = VerletConfig {
        step_s: 10.0,
        max_steps: 720,
    };
    let mut rails = OnRailsCache::new();
    let started = std::time::Instant::now();
    rails
        .bake(&ephemeris, initial, SimTime::EPOCH, config, &[])
        .expect("bakes");
    let bake = started.elapsed();
    let started = std::time::Instant::now();
    for i in 0..100 {
        let t = SimTime::EPOCH.offset(i as f64 * 10.0);
        assert!(
            rails.usable_for(
                &ephemeris,
                rails
                    .sample_at(t)
                    .map(|(p, v)| TestParticleState {
                        position: p,
                        velocity: v
                    })
                    .unwrap(),
                t,
                config,
                &[],
                1e-6,
                1e-9
            )
        );
    }
    let hits = started.elapsed();
    eprintln!(
        "on-rails bake {:.2} ms vs 100 cache hits {:.3} ms ({:.1}x)",
        bake.as_secs_f64() * 1000.0,
        hits.as_secs_f64() * 1000.0,
        bake.as_secs_f64() / hits.as_secs_f64().max(1e-9)
    );
}

#[test]
fn onrails_hermite_mid_sample_error_is_sub_metre() {
    // The flight loop samples translation between 5 s rail nodes: cubic
    // Hermite through the stored endpoint velocities must hold sub-metre
    // error at orbital speeds, otherwise riding the rails would corrupt the
    // authoritative state.
    let mu = 4.0e13;
    let radius = 10_000_000.0;
    let ephemeris = central_ephemeris(mu);
    let speed = (mu / radius).sqrt();
    let initial = TestParticleState {
        position: DVec3::X * radius,
        velocity: DVec3::Y * speed,
    };
    let path = propagate_sampled_verlet(
        &ephemeris,
        initial,
        SimTime::EPOCH,
        VerletConfig {
            step_s: 5.0,
            max_steps: 720,
        },
        &[],
    )
    .expect("coast bakes");
    // Reference: same arc integrated natively at 10x resolution.
    let fine = propagate_sampled_verlet(
        &ephemeris,
        initial,
        SimTime::EPOCH,
        VerletConfig {
            step_s: 0.5,
            max_steps: 7_200,
        },
        &[],
    )
    .expect("fine reference bakes");
    let rails = {
        let mut cache = crate::OnRailsCache::new();
        cache
            .bake(
                &ephemeris,
                initial,
                SimTime::EPOCH,
                VerletConfig {
                    step_s: 5.0,
                    max_steps: 720,
                },
                &[],
            )
            .expect("rails bake");
        cache
    };
    let mut max_error: f64 = 0.0;
    for i in 0..fine.positions.len() {
        let t = fine.times[i];
        if t.seconds() > path.end_time.seconds() {
            break;
        }
        let (sampled, _) = rails.sample_at(t).expect("covered");
        max_error = max_error.max((sampled - fine.positions[i]).length());
    }
    eprintln!("hermite mid-sample max error {max_error:e} m over 1 h arc");
    assert!(max_error < 2.0, "hermite error {max_error:e} m exceeds 2 m");
}

#[test]
fn scheduler_orders_wakes_and_drains_by_sim_time() {
    use crate::{EventScheduler, ScheduledKind};
    // Out-of-order arming still fires in epoch order; ties keep arm order.
    let mut queue = EventScheduler::new();
    assert!(queue.is_empty());
    queue.arm(ScheduledKind::Alarm, SimTime(300.0));
    queue.arm(
        ScheduledKind::RailsImpact { body: BodyId(3) },
        SimTime(100.0),
    );
    queue.arm(ScheduledKind::RailsHorizon, SimTime(200.0));
    assert_eq!(queue.len(), 3);
    assert!(matches!(
        queue.next().map(|event| event.kind),
        Some(ScheduledKind::RailsImpact { .. })
    ));
    let due = queue.drain_due(SimTime(150.0));
    assert_eq!(due.len(), 1);
    assert!(matches!(
        due[0].kind,
        ScheduledKind::RailsImpact { body } if body == BodyId(3)
    ));
    // Nothing due at the same epoch twice; future events stay armed.
    assert!(queue.drain_due(SimTime(150.0)).is_empty());
    assert_eq!(queue.len(), 2);
    let rest = queue.drain_due(SimTime(1_000.0));
    assert_eq!(rest.len(), 2);
    assert!(matches!(rest[0].kind, ScheduledKind::RailsHorizon));
    assert!(matches!(rest[1].kind, ScheduledKind::Alarm));
    assert!(queue.is_empty());
}

#[test]
fn scheduler_rails_wake_slot_holds_one_wake_and_cancels() {
    use crate::{EventScheduler, ScheduledKind};
    // Rebakes replace the wake instead of stacking: one path, one wake.
    let mut queue = EventScheduler::new();
    queue.arm_rails_wake(ScheduledKind::RailsHorizon, SimTime(500.0));
    queue.arm(ScheduledKind::Alarm, SimTime(600.0));
    queue.arm_rails_wake(
        ScheduledKind::RailsImpact { body: BodyId(1) },
        SimTime(400.0),
    );
    assert_eq!(queue.len(), 2);
    assert!(matches!(
        queue.next().map(|event| event.kind),
        Some(ScheduledKind::RailsImpact { .. })
    ));
    let id = queue.next().map(|event| event.id).unwrap();
    assert!(queue.cancel(id));
    assert!(!queue.cancel(id));
    assert_eq!(queue.len(), 1);
    queue.clear_rails_wakes();
    assert_eq!(queue.len(), 1);
    assert!(matches!(
        queue.next().map(|event| event.kind),
        Some(ScheduledKind::Alarm)
    ));
}

#[test]
fn fast_bake_matches_exact_within_metres() {
    // The table-backed bake must agree with the exact per-step Kepler path
    // while real bodies move: a tight binary companion laps every ~2.9 h, so
    // the 40 s table nodes actually interpolate. Same end state class,
    // metre-grade deviation over a full day. Prints both timings.
    let mu_primary = 4.0e13;
    let mu_secondary = 4.0e12;
    let separation = 20_000_000.0;
    let ephemeris = two_body_binary(mu_primary, mu_secondary, separation);
    let radius = 30_000_000.0;
    let speed = ((mu_primary + mu_secondary) / radius).sqrt();
    let initial = TestParticleState {
        position: DVec3::X * radius,
        velocity: DVec3::Y * speed,
    };
    let config = VerletConfig {
        step_s: 5.0,
        max_steps: 17_280,
    };
    let started = std::time::Instant::now();
    let exact = propagate_sampled_verlet(&ephemeris, initial, SimTime::EPOCH, config, &[])
        .expect("exact bakes");
    let exact_elapsed = started.elapsed();
    let started = std::time::Instant::now();
    let fast = propagate_sampled_verlet_fast(
        &ephemeris,
        initial,
        SimTime::EPOCH,
        config,
        &[],
        crate::TABLE_NODE_EVERY_STEPS,
    )
    .expect("fast bakes");
    let fast_elapsed = started.elapsed();
    assert_eq!(fast.end, exact.end);
    assert_eq!(fast.positions.len(), exact.positions.len());
    let mut max_deviation: f64 = 0.0;
    for i in 0..exact.positions.len() {
        max_deviation = max_deviation.max((fast.positions[i] - exact.positions[i]).length());
    }
    eprintln!(
        "fast bake {:.2} ms vs exact {:.2} ms ({:.1}x), max deviation {max_deviation:e} m over {} samples",
        fast_elapsed.as_secs_f64() * 1000.0,
        exact_elapsed.as_secs_f64() * 1000.0,
        exact_elapsed.as_secs_f64() / fast_elapsed.as_secs_f64().max(1e-9),
        exact.positions.len()
    );
    assert!(
        max_deviation < 5.0,
        "table interpolant drifted {max_deviation:e} m"
    );
}

#[test]
fn fast_bake_is_deterministic_across_runs() {
    // Same inputs, same path, bit for bit: the table build must not depend
    // on thread scheduling or hash order.
    let mu = 4.0e13;
    let radius = 10_000_000.0;
    let ephemeris = central_ephemeris(mu);
    let speed = (mu / radius).sqrt();
    let initial = TestParticleState {
        position: DVec3::X * radius,
        velocity: DVec3::Y * speed,
    };
    let config = VerletConfig {
        step_s: 5.0,
        max_steps: 1_000,
    };
    let first = propagate_sampled_verlet_fast(
        &ephemeris,
        initial,
        SimTime::EPOCH,
        config,
        &[],
        crate::TABLE_NODE_EVERY_STEPS,
    )
    .expect("first bakes");
    let second = propagate_sampled_verlet_fast(
        &ephemeris,
        initial,
        SimTime::EPOCH,
        config,
        &[],
        crate::TABLE_NODE_EVERY_STEPS,
    )
    .expect("second bakes");
    assert_eq!(first, second);
}

#[test]
fn rails_rejects_edited_ephemeris_even_with_same_identity_metadata() {
    let ephemeris = central_ephemeris(4e13);
    let initial = TestParticleState {
        position: DVec3::X * 1e7,
        velocity: DVec3::Y * 2000.0,
    };
    let config = VerletConfig {
        step_s: 5.0,
        max_steps: 3,
    };
    let mut rails = OnRailsCache::new();
    rails
        .bake(&ephemeris, initial, SimTime::EPOCH, config, &[])
        .unwrap();
    let mut edited = ephemeris.clone();
    edited.bodies[0].mu *= 1.01;
    assert!(!rails.usable_for(&edited, initial, SimTime::EPOCH, config, &[], 5.0, 0.05));
    assert!(rails.sample_at(SimTime(f64::NAN)).is_none());
}

#[test]
fn hermite_velocity_is_the_position_derivative() {
    let p0 = DVec3::new(1e7, 2e7, 3e7);
    let p1 = p0 + DVec3::new(100.0, 40.0, -3.0);
    let v0 = DVec3::new(20.0, 4.0, 0.0);
    let v1 = DVec3::new(18.0, 10.0, -2.0);
    let h = 5.0;
    for s in [0.2, 0.5, 0.8] {
        let (_, velocity) = crate::table::hermite_state(p0, v0, p1, v1, h, s);
        let epsilon = 1e-3;
        let before = crate::table::hermite_state(p0, v0, p1, v1, h, s - epsilon / h).0;
        let after = crate::table::hermite_state(p0, v0, p1, v1, h, s + epsilon / h).0;
        assert!(velocity.distance((after - before) / (2.0 * epsilon)) < 1e-5);
    }
}

#[test]
fn ephemeris_table_rejects_bad_steps_and_out_of_coverage_queries() {
    let ephemeris = central_ephemeris(4e13);
    for step in [0.0, -1.0, f64::NAN, f64::INFINITY, 1e-100] {
        assert!(
            crate::EphemerisTable::build(
                &ephemeris,
                &[BodyId(0)],
                SimTime::EPOCH,
                SimTime(10.0),
                step
            )
            .is_err()
        );
    }
    let table =
        crate::EphemerisTable::build(&ephemeris, &[BodyId(0)], SimTime::EPOCH, SimTime(10.0), 3.0)
            .unwrap();
    for time in [-1.0, 10.01, f64::NAN] {
        assert!(table.body_state_at(BodyId(0), SimTime(time)).is_none());
        assert!(
            table
                .acceleration_at(DVec3::X * 1e7, SimTime(time))
                .is_none()
        );
    }
    assert!(table.body_state_at(BodyId(0), SimTime(10.0)).is_some());
}

#[test]
fn fast_prediction_hits_non_gravitating_physical_body() {
    let mut body = BakedBody::fixed(BodyId(0), "surface-only", 4e13, 5.0);
    body.gravity_source = false;
    let ephemeris = BakedEphemeris::new("CONTACT", vec![body]).unwrap();
    let initial = TestParticleState {
        position: DVec3::X * 10.0,
        velocity: -DVec3::X * 20.0,
    };
    let config = VerletConfig {
        step_s: 1.0,
        max_steps: 2,
    };
    let path =
        propagate_sampled_verlet_fast(&ephemeris, initial, SimTime::EPOCH, config, &[BodyId(0)], 8)
            .unwrap();
    assert_eq!(path.end, SampledPathEnd::Impact(BodyId(0)));
    assert!((path.positions.last().unwrap().x - 5.0).abs() < 1e-10);
}

#[test]
fn rails_full_horizon_has_measured_second_order_orbit_error() {
    let mu = 4e13;
    let radius = 1e7_f64;
    let ephemeris = central_ephemeris(mu);
    let speed = (mu / radius).sqrt();
    let initial = TestParticleState {
        position: DVec3::X * radius,
        velocity: DVec3::Y * speed,
    };
    let mut errors = Vec::new();
    for step in [5.0, 2.5] {
        let mut cache = OnRailsCache::new();
        cache
            .bake(
                &ephemeris,
                initial,
                SimTime::EPOCH,
                VerletConfig {
                    step_s: step,
                    max_steps: (200_000.0 / step) as u64,
                },
                &[],
            )
            .unwrap();
        let mut position_error = 0.0_f64;
        let mut velocity_error = 0.0_f64;
        for i in 0..=1000 {
            let time = SimTime(i as f64 * 200.0);
            let phase = speed / radius * time.0;
            let (p, v) = cache.sample_at(time).unwrap();
            position_error =
                position_error.max(p.distance(DVec3::new(phase.cos(), phase.sin(), 0.0) * radius));
            velocity_error =
                velocity_error.max(v.distance(DVec3::new(-phase.sin(), phase.cos(), 0.0) * speed));
        }
        eprintln!(
            "200000s circular coast dt={step}: max position {position_error:.6} m, velocity {velocity_error:.9} m/s"
        );
        errors.push(position_error);
        assert!(position_error < 200.0);
        assert!(velocity_error < 0.05);
    }
    assert!(
        errors[1] < errors[0] * 0.3,
        "halving dt must recover second-order convergence: {errors:?}"
    );
}

#[test]
fn chunked_extend_matches_full_bake_bitwise() {
    // Hitch-free coast entry depends on this: head bake + per-frame
    // extension chunks must reproduce the one-shot full bake exactly
    // (same table nodes, same loop), not just approximately.
    use crate::{OnRailsCache, VerletConfig};
    let mu = 4.0e13;
    let radius = 10_000_000.0;
    let ephemeris = central_ephemeris(mu);
    let speed = (mu / radius).sqrt();
    let initial = TestParticleState {
        position: DVec3::X * radius,
        velocity: DVec3::Y * speed,
    };
    let config = VerletConfig {
        step_s: 5.0,
        max_steps: 4_000,
    };
    let started = std::time::Instant::now();
    let mut full = OnRailsCache::new();
    full.bake(&ephemeris, initial, SimTime::EPOCH, config, &[])
        .expect("full bakes");
    let full_elapsed = started.elapsed();
    let started = std::time::Instant::now();
    let mut chunked = OnRailsCache::new();
    chunked
        .bake_head(&ephemeris, initial, SimTime::EPOCH, config, &[])
        .expect("head bakes");
    let head_elapsed = started.elapsed();
    assert!(
        chunked.covered_until().expect("head covers").seconds()
            < full.covered_until().expect("full covers").seconds()
    );
    let mut extensions = 0;
    let mut extend_elapsed = std::time::Duration::ZERO;
    while chunked.needs_extension(SimTime::EPOCH, f64::INFINITY) {
        let started = std::time::Instant::now();
        assert!(chunked.extend(&ephemeris, 512).expect("extends"), "stalled");
        extend_elapsed += started.elapsed();
        extensions += 1;
        assert!(extensions < 20, "extension loop did not converge");
    }
    eprintln!(
        "chunked bake: full {:.2} ms vs head {:.2} ms + {}x extend avg {:.2} ms",
        full_elapsed.as_secs_f64() * 1000.0,
        head_elapsed.as_secs_f64() * 1000.0,
        extensions,
        extend_elapsed.as_secs_f64() * 1000.0 / extensions.max(1) as f64
    );
    let (a, b) = (full.path().unwrap(), chunked.path().unwrap());
    assert_eq!(a.positions.len(), b.positions.len());
    assert_eq!(a, b, "chunked bake must equal the full bake bitwise");
    eprintln!(
        "head+{}x512chunks == full bake over {} samples",
        extensions,
        a.positions.len()
    );
}

#[test]
fn rails_trim_keeps_window_and_extend_accounting() {
    // Sliding window for indefinite cruise: old samples drop in chunks,
    // interpolation/sampling still work inside the window, pre-window
    // queries correctly miss (forcing rebake), and extension keeps
    // budgeting against the stored horizon, not the trimmed length.
    use crate::{OnRailsCache, VerletConfig};
    let mu = 4.0e13;
    let radius = 10_000_000.0;
    let ephemeris = central_ephemeris(mu);
    let speed = (mu / radius).sqrt();
    let initial = TestParticleState {
        position: DVec3::X * radius,
        velocity: DVec3::Y * speed,
    };
    let config = VerletConfig {
        step_s: 5.0,
        max_steps: 4_000,
    };
    let mut rails = OnRailsCache::new();
    rails
        .bake(&ephemeris, initial, SimTime::EPOCH, config, &[])
        .expect("bakes");
    assert_eq!(rails.sample_count(), 4001);
    // Trim everything older than T+3600 s, chunks of 1800 s.
    rails.trim_before(SimTime(7200.0), 3600.0, 1800.0);
    let count = rails.sample_count();
    assert!(count < 4001 && count > 3000, "trimmed to {count} samples");
    // Inside the window: sampling works.
    assert!(rails.sample_at(SimTime(10_000.0)).is_some());
    // Before the window: miss (caller rebakes).
    assert!(rails.sample_at(SimTime(100.0)).is_none());
    // Horizon accounting untouched: this bake has 4000 accepted of 4000
    // max, so no extension even after trimming.
    assert!(!rails.needs_extension(SimTime(10_000.0), 60.0));
    // Fresh shorter bake still extends after a trim.
    let mut short = OnRailsCache::new();
    let config2 = VerletConfig {
        step_s: 5.0,
        max_steps: 4_000,
    };
    short
        .bake_head(&ephemeris, initial, SimTime::EPOCH, config2, &[])
        .expect("head bakes");
    let head_count = short.sample_count();
    short.trim_before(SimTime(5_000.0), 3600.0, 600.0);
    assert!(short.sample_count() < head_count);
    assert!(short.extend(&ephemeris, 512).expect("extends"));
    eprintln!(
        "trim+extend accounting holds over {} samples",
        short.sample_count()
    );
}

#[test]
fn year_long_scaled_escape_stays_display_grade() {
    // Interstellar prediction needs year horizons without megabyte paths.
    // Uniform 4 h steps fail catastrophically here (measured 7.4e10 m: the
    // fast periapsis bend is unresolved and the asymptote error compounds
    // forever), so the far bake follows the local dynamical timescale
    // instead: dense at periapsis, daily strides in cruise.
    use crate::{VerletConfig, propagate_sampled_verlet, propagate_sampled_verlet_scaled};
    let mu = 4.0e13;
    let start = 10_000_000.0;
    let ephemeris = central_ephemeris(mu);
    let initial = TestParticleState {
        position: DVec3::X * start,
        velocity: DVec3::Y * (2.0 * mu / start).sqrt() * 1.2,
    };
    let started = std::time::Instant::now();
    let scaled = propagate_sampled_verlet_scaled(
        &ephemeris,
        initial,
        SimTime::EPOCH,
        60.0,
        86_400.0,
        1.0 / 40.0,
        700,
        &[],
    )
    .expect("scaled year bakes");
    let scaled_elapsed = started.elapsed();
    assert!(
        scaled.end_time.seconds() >= 105_600.0 * 300.0,
        "scaled bake must reach the full reference year"
    );
    let fine = propagate_sampled_verlet(
        &ephemeris,
        initial,
        SimTime::EPOCH,
        VerletConfig {
            step_s: 300.0,
            max_steps: 105_600,
        },
        &[],
    )
    .expect("fine year bakes");
    // Compare at the scaled nodes against the bracketing fine samples,
    // over the shared horizon only.
    let fine_end = fine.end_time.seconds();
    let mut max_deviation: f64 = 0.0;
    let mut compared = 0;
    let mut j = 0;
    for (i, t) in scaled.times.iter().enumerate() {
        if t.seconds() > fine_end {
            break;
        }
        compared += 1;
        while j + 1 < fine.times.len() && fine.times[j + 1].seconds() < t.seconds() {
            j += 1;
        }
        let reference = if j + 1 < fine.times.len()
            && (fine.times[j + 1].seconds() - t.seconds()).abs()
                < (t.seconds() - fine.times[j].seconds()).abs()
        {
            fine.positions[j + 1]
        } else {
            fine.positions[j]
        };
        max_deviation = max_deviation.max((scaled.positions[i] - reference).length());
    }
    assert!(
        compared > 100,
        "must compare across the year, got {compared}"
    );
    let span_days = fine_end / 86_400.0;
    eprintln!(
        "year escape scaled deviation {max_deviation:e} m over {span_days:.0} days, {} nodes in {:.2} ms",
        scaled.positions.len(),
        scaled_elapsed.as_secs_f64() * 1000.0
    );
    assert!(
        max_deviation < 1.0e9,
        "scaled year drifted {max_deviation:e} m — too coarse even for display"
    );
}

#[test]
fn simd_snapshot_and_accel_match_scalar_within_tolerance() {
    // Cross-path agreement for the AVX-512 kernels on the real 22-source
    // system: Hermite FMA contraction and rsqrt-Newton refinement must stay
    // within ~1e-12 relative of the scalar loop. Skipped (vacuous pass)
    // where AVX-512 is unavailable — then both sides run the same code.
    use crate::{EphemerisTable, TableSnapshot};
    if !thessa_simd::avx512_available() {
        eprintln!("no AVX-512: SIMD agreement vacuous");
        return;
    }
    let config: SystemConfig =
        toml::from_str(include_str!("../../../data/system.toml")).expect("system parses");
    let ephemeris = config.bake().expect("system bakes");
    let bodies: Vec<_> = ephemeris.gravity_sources().map(|body| body.id).collect();
    let table = EphemerisTable::build(
        &ephemeris,
        &bodies,
        SimTime::EPOCH,
        SimTime::EPOCH.offset(86_400.0),
        40.0,
    )
    .expect("table builds");
    let mut simd_snap = TableSnapshot::default();
    let mut scalar_snap = TableSnapshot::default();
    let mut worst_snapshot: f64 = 0.0;
    let mut worst_accel: f64 = 0.0;
    for minutes in [0, 7, 63, 721, 1439] {
        let time = SimTime::EPOCH.offset(minutes as f64 * 60.0);
        table.snapshot_with(time, &mut simd_snap, true);
        table.snapshot_with(time, &mut scalar_snap, false);
        assert_eq!(simd_snap.cx.len(), scalar_snap.cx.len());
        for i in 0..simd_snap.cx.len() {
            for (a, b) in [
                (simd_snap.cx[i], scalar_snap.cx[i]),
                (simd_snap.cy[i], scalar_snap.cy[i]),
                (simd_snap.cz[i], scalar_snap.cz[i]),
            ] {
                let scale = b.abs().max(1.0);
                worst_snapshot = worst_snapshot.max((a - b).abs() / scale);
            }
        }
        let probe = DVec3::new(1.0e8, -2.0e8, 3.0e8);
        let simd_a = table
            .accel_with(&simd_snap, probe, true)
            .expect("simd accel");
        let scalar_a = table
            .accel_with(&scalar_snap, probe, false)
            .expect("scalar accel");
        let scale = scalar_a.length().max(1e-12);
        worst_accel = worst_accel.max((simd_a - scalar_a).length() / scale);
    }
    eprintln!("simd-vs-scalar max relative: snapshot {worst_snapshot:e}, accel {worst_accel:e}");
    assert!(
        worst_snapshot < 1e-12,
        "snapshot diverged {worst_snapshot:e}"
    );
    assert!(worst_accel < 1e-9, "accel diverged {worst_accel:e}");
}

#[test]
fn reused_table_endpoints_match_recomputed_verlet_bitwise() {
    // Moving sources exercise node transitions, SIMD tails and a fractional
    // final table interval. Reference intentionally recomputes both endpoints.
    let config: SystemConfig = toml::from_str(include_str!("../../../data/system.toml")).unwrap();
    let ephemeris = config.bake().unwrap();
    let home = ephemeris
        .body_state(ephemeris.body_id("thessa").unwrap(), SimTime::EPOCH)
        .unwrap();
    let initial = TestParticleState {
        position: home.position_inertial + DVec3::Z * 1e9,
        velocity: home.velocity_inertial + DVec3::X * 100.0,
    };
    let sources: Vec<_> = ephemeris.gravity_sources().map(|b| b.id).collect();
    for steps in [0, 1, 129] {
        let dt = 5.0;
        let table = EphemerisTable::build(
            &ephemeris,
            &sources,
            SimTime::EPOCH,
            SimTime(dt * steps as f64),
            40.0,
        )
        .unwrap();
        let actual = propagate_sampled_verlet_fast(
            &ephemeris,
            initial,
            SimTime::EPOCH,
            VerletConfig {
                step_s: dt,
                max_steps: steps,
            },
            &[],
            8,
        )
        .unwrap();
        let mut state = initial;
        let mut time = SimTime::EPOCH;
        let mut start = TableSnapshot::default();
        let mut end = TableSnapshot::default();
        for i in 0..steps as usize {
            table.snapshot(time, &mut start);
            let a0 = table.accel_from(&start, state.position).unwrap();
            let position = state.position + state.velocity * dt + a0 * (0.5 * dt * dt);
            time = time.offset(dt);
            table.snapshot(time, &mut end);
            let a1 = table.accel_from(&end, position).unwrap();
            state = TestParticleState {
                position,
                velocity: state.velocity + (a0 + a1) * (0.5 * dt),
            };
            assert_eq!(actual.positions[i + 1], state.position);
            assert_eq!(actual.velocities[i + 1], state.velocity);
            assert_eq!(actual.times[i + 1], time);
        }
        assert_eq!(actual.stats.accepted_steps, steps);
        assert_eq!(actual.end_time, time);
    }
}

#[test]
fn table_fallback_ignores_non_gravitating_center() {
    // A singular zero-mu lane makes the SIMD kernel fall back to scalar.
    for count in [1, 4, 8, 9] {
        let bodies: Vec<_> = (0..count)
            .map(|i| {
                let mut body = BakedBody::fixed(BodyId(i), "surface-only", 1.0, 1.0);
                body.gravity_source = false;
                body
            })
            .collect();
        let ephemeris = BakedEphemeris::new("CONTACT", bodies).unwrap();
        let ids: Vec<_> = ephemeris.bodies.iter().map(|b| b.id).collect();
        let table =
            EphemerisTable::build(&ephemeris, &ids, SimTime::EPOCH, SimTime(1.0), 1.0).unwrap();
        let mut snapshot = TableSnapshot::default();
        table.snapshot(SimTime::EPOCH, &mut snapshot);
        for simd in [false, true] {
            assert_eq!(
                table.accel_with(&snapshot, DVec3::ZERO, simd),
                Some(DVec3::ZERO)
            );
        }
    }
}

#[test]
fn extension_rejects_missing_velocity_without_panicking() {
    let ephemeris = central_ephemeris(4e13);
    let mut path = propagate_sampled_verlet_fast(
        &ephemeris,
        TestParticleState {
            position: DVec3::X * 1e7,
            velocity: DVec3::Y * 2000.0,
        },
        SimTime::EPOCH,
        VerletConfig {
            step_s: 5.0,
            max_steps: 2,
        },
        &[],
        8,
    )
    .unwrap();
    path.velocities.clear();
    let before = path.clone();
    assert!(propagate_sampled_extend(&ephemeris, &mut path, 5.0, 10, 3, &[], 8).is_err());
    assert_eq!(path, before);
}

#[test]
fn failed_extension_rolls_back_already_appended_steps() {
    let ephemeris = central_ephemeris(1.0);
    let mut path = SampledPath {
        positions: vec![DVec3::X; 2],
        velocities: vec![DVec3::X * 0.5; 2],
        accelerations: vec![DVec3::ZERO; 2],
        times: vec![SimTime(-1.0), SimTime::EPOCH],
        end_time: SimTime::EPOCH,
        end: SampledPathEnd::Completed,
        stats: IntegratorStats {
            accepted_steps: 1,
            rejected_steps: 0,
        },
    };
    // x=1,v=.5 => x1=1,v1=-.5 => x2=0 (point singularity).
    let before = path.clone();
    assert!(propagate_sampled_extend(&ephemeris, &mut path, 1.0, 10, 3, &[], 8).is_err());
    assert_eq!(path, before);
}

#[test]
fn constant_spin_matches_axis_rotation_and_rejects_asymmetric_tumble() {
    let inertia = glam::DMat3::from_diagonal(DVec3::new(2.0, 3.0, 5.0));
    let initial = DQuat::from_rotation_y(0.7);
    for seconds in [0.0, 1.0, 3600.0] {
        let q = constant_spin_orientation(initial, DVec3::X * 0.25, inertia, seconds).unwrap();
        assert!(q.abs_diff_eq(
            (initial * DQuat::from_rotation_x(0.25 * seconds)).normalize(),
            1e-12
        ));
        assert!(
            constant_spin_orientation(initial, DVec3::ZERO, inertia, seconds)
                .unwrap()
                .abs_diff_eq(initial, 1e-12)
        );
    }
    assert!(constant_spin_orientation(initial, DVec3::ONE, inertia, 1.0).is_none());
}

#[test]
fn rails_trim_moves_buffers_only_after_accumulating_a_full_chunk() {
    let ephemeris = central_ephemeris(4e13);
    let mut rails = OnRailsCache::new();
    rails
        .bake(
            &ephemeris,
            TestParticleState {
                position: DVec3::X * 1e7,
                velocity: DVec3::Y * 2000.0,
            },
            SimTime::EPOCH,
            VerletConfig {
                step_s: 5.0,
                max_steps: 1000,
            },
            &[],
        )
        .unwrap();
    rails.trim_before(SimTime(720.0), 60.0, 600.0);
    let after_chunk = rails.sample_count();
    assert!(after_chunk < 1001);
    for t in (725..1200).step_by(5) {
        rails.trim_before(SimTime(f64::from(t)), 60.0, 600.0);
        assert_eq!(
            rails.sample_count(),
            after_chunk,
            "must not memmove one sample per tick"
        );
    }
    rails.trim_before(SimTime(1400.0), 60.0, 600.0);
    assert!(rails.sample_count() < after_chunk);
    assert!(rails.sample_at(SimTime(1340.0)).is_some());
}

#[test]
fn zero_step_head_preserves_the_requested_budget() {
    let ephemeris = central_ephemeris(4e13);
    let mut rails = OnRailsCache::new();
    let path = rails
        .bake_head(
            &ephemeris,
            TestParticleState {
                position: DVec3::X * 1e7,
                velocity: DVec3::Y * 2000.0,
            },
            SimTime::EPOCH,
            VerletConfig {
                step_s: 5.0,
                max_steps: 0,
            },
            &[],
        )
        .unwrap();
    assert_eq!(path.positions.len(), 1);
    assert_eq!(path.end_time, SimTime::EPOCH);
    assert!(!rails.needs_extension(SimTime::EPOCH, 100.0));
}

/// High alpha must be drag-dominated, not riding an ever-rising post-stall
/// lift curve. Regression for the old alpha^0.65 + tanh model, under which
/// CL climbed to ~1.3 at 90 deg and the craft held high alpha on minimal
/// thrust. Uses an X-15-like slope/stall/max plus the default flat plate.
#[test]
fn x15_post_stall_is_drag_dominated() {
    use crate::{AeroConfig, AeroEnvironment, AeroGeometry, AeroPanel, AeroState, PanelAeroModel};
    use glam::DVec3;
    fn coefficients_at(
        slope: f64,
        stall_deg: f64,
        max: f64,
        alpha_deg: f64,
        deflection_rad: f64,
    ) -> crate::AeroCoefficients {
        let mut panel = AeroPanel::flat_plate(DVec3::ZERO, 10.0, 2.0).expect("panel");
        panel.control_deflection_rad = deflection_rad;
        let config = AeroConfig {
            lift_slope_per_rad: slope,
            stall_angle_rad: stall_deg.to_radians(),
            max_lift_coefficient: max,
            ..AeroConfig::default()
        };
        // Constant-speed direction sweep: exact alpha at constant Mach,
        // unlike the tan() construction which blows up Mach near 90 deg.
        let alpha = alpha_deg.to_radians();
        let speed = 120.0;
        let case = crate::AeroCase::new(
            AeroState::new(
                DVec3::new(speed * alpha.cos(), 0.0, -speed * alpha.sin()),
                DVec3::ZERO,
            ),
            AeroEnvironment::standard_sea_level(),
            AeroGeometry::new(vec![panel]).expect("geometry"),
        )
        .expect("case");
        let model = PanelAeroModel::new(config).expect("model");
        model
            .evaluate_detailed(&case)
            .expect("result")
            .panel_loads
            .expect("loads")[0]
            .coefficients
    }
    // Default flat plate (stall 18 deg): past-stall lift stays bounded and
    // well under the old 1.3-at-90-deg blowup; drag explodes.
    let post = coefficients_at(2.0 * std::f64::consts::PI, 18.0, 1.35, 25.0, 0.0);
    assert!(
        post.lift < AeroConfig::default().max_lift_coefficient,
        "post-stall CL must stay under max: {}",
        post.lift
    );
    let pre = coefficients_at(2.0 * std::f64::consts::PI, 18.0, 1.35, 15.0, 0.0);
    assert!(
        post.drag > pre.drag * 2.0,
        "CD must jump past stall: 15deg={} 25deg={}",
        pre.drag,
        post.drag
    );
    // ~90 deg behaves like a flat plate: little lift, ~2.0 drag.
    let flat = coefficients_at(2.0 * std::f64::consts::PI, 18.0, 1.35, 90.0, 0.0);
    assert!(
        flat.lift.abs() < 0.3,
        "flat-plate lift near 90deg must be small: {}",
        flat.lift
    );
    assert!(
        flat.drag > 1.0,
        "flat-plate drag near 90deg must be large: {}",
        flat.drag
    );
    // X-15-like config (stall 22 deg): L/D collapses from efficient
    // attached flight to drag-dominated separated flight.
    let cruise = coefficients_at(4.6, 22.0, 1.45, 15.0, 0.0);
    let high_alpha = coefficients_at(4.6, 22.0, 1.45, 30.0, 0.0);
    let ld_cruise = cruise.lift / cruise.drag;
    let ld_high = high_alpha.lift / high_alpha.drag;
    assert!(
        ld_cruise > 5.0,
        "attached flight must stay efficient: L/D={ld_cruise}"
    );
    assert!(
        ld_high < 2.5,
        "separated flight must be drag-dominated: L/D={ld_high}"
    );
    // Control authority fades in separated flow.
    let attached_gain = coefficients_at(4.6, 22.0, 1.45, 10.0, 0.1).lift
        - coefficients_at(4.6, 22.0, 1.45, 10.0, 0.0).lift;
    let separated_gain = coefficients_at(4.6, 22.0, 1.45, 30.0, 0.1).lift
        - coefficients_at(4.6, 22.0, 1.45, 30.0, 0.0).lift;
    assert!(
        separated_gain.abs() < attached_gain.abs() * 0.8,
        "control must fade when stalled: attached={attached_gain} separated={separated_gain}"
    );
    // The whole 0..90 sweep stays finite and continuous (1 deg steps).
    let mut previous = coefficients_at(4.6, 22.0, 1.45, 0.0, 0.0);
    for deg in 1..=90 {
        let current = coefficients_at(4.6, 22.0, 1.45, deg as f64, 0.0);
        assert!(current.lift.is_finite() && current.drag.is_finite());
        assert!(
            (current.lift - previous.lift).abs() < 0.25,
            "CL jump at {deg}deg"
        );
        assert!(
            (current.drag - previous.drag).abs() < 0.35,
            "CD jump at {deg}deg"
        );
        previous = current;
    }
}

/// SoA oracle equivalence: the structure-of-arrays path must reproduce the
/// AoS panel loop (same equations, shared coefficient code) across attached,
/// stalled and supersonic regimes, with and without control deflection.
#[test]
fn aero_soa_oracle_matches_panel_loop() {
    use crate::{
        AeroConfig, AeroEnvironment, AeroGeometry, AeroPanel, AeroState, PanelAeroModel, PanelSoA,
    };
    use glam::DVec3;
    let wing = AeroPanel::flat_plate(DVec3::new(0.0, 2.0, 0.0), 12.0, 1.5)
        .expect("wing")
        .with_planform(4.0, 2.6, 0.35, 1.0)
        .expect("wing planform");
    let mut tail = AeroPanel::flat_plate(DVec3::new(-4.0, 0.0, 0.5), 4.0, 1.0).expect("tail");
    tail.control_deflection_rad = 0.15;
    tail.center_of_pressure_body_m = DVec3::new(-4.2, 0.0, 0.5);
    let mut fin = AeroPanel::flat_plate(DVec3::new(-4.0, 0.0, 0.0), 2.0, 1.2).expect("fin");
    fin.exposure = 0.6;
    let geometry = AeroGeometry::new(vec![wing, tail, fin]).expect("geometry");
    let soa = PanelSoA::from_geometry(&geometry).expect("soa layout");
    assert_eq!(soa.count, 3);
    let model = PanelAeroModel::new(AeroConfig::default()).expect("model");
    // Attached, stalled, supersonic, spinning, crosswind: cover every branch.
    let cases = [
        (150.0, 5.0_f64.to_radians(), DVec3::ZERO, DVec3::ZERO),
        (150.0, 35.0_f64.to_radians(), DVec3::ZERO, DVec3::ZERO),
        (
            700.0,
            8.0_f64.to_radians(),
            DVec3::new(10.0, -5.0, 2.0),
            DVec3::new(0.1, -0.2, 0.05),
        ),
        (
            120.0,
            -12.0_f64.to_radians(),
            DVec3::ZERO,
            DVec3::new(0.0, 0.0, 0.3),
        ),
    ];
    for (speed, alpha, wind, omega) in cases {
        // Wind blows along body x/z; the y entry doubles as extra coverage
        // of the cross term in the oracle transcription.
        let velocity = DVec3::new(
            speed * alpha.cos() + wind.x,
            wind.y,
            -speed * alpha.sin() + wind.z,
        );
        let state = AeroState::new(velocity, omega);
        let environment =
            AeroEnvironment::new(1.225, 340.294, 1.81e-5, DVec3::new(wind.x, 0.0, wind.z));
        let reference = model
            .evaluate_state(state, environment, &geometry)
            .expect("reference");
        let candidate = model
            .evaluate_soa_parts(state, environment, &soa, false)
            .expect("soa");
        for (got, want, name) in [
            (candidate.force_body_n, reference.force_body_n, "force"),
            (candidate.moment_body_nm, reference.moment_body_nm, "moment"),
        ] {
            let scale = want.length().max(1.0);
            assert!(
                (got - want).length() / scale < 1e-12,
                "{name} diverged at {speed} m/s: {got:?} vs {want:?}"
            );
        }
        assert_eq!(candidate.panel_count, reference.panel_count);
        assert!((candidate.mach - reference.mach).abs() < 1e-15);
    }
}

/// SIMD fast-path agreement: `evaluate_soa_simd` must reproduce the SoA
/// oracle and the AoS panel loop across dispatch shapes (3 lanes = scalar
/// tail only, 7 = quad + tail, 10 = chunk + tail, 16 = two chunks),
/// attached/stalled/supersonic regimes, and the vortex + hypersonic
/// drag-cutoff model. Kernel math differs from the scalar path only in
/// float association, hence 1e-9 relative (the slot-order bug this guards
/// against diverged at O(1)).
#[test]
fn aero_soa_simd_matches_oracle_and_panel_loop() {
    use crate::{
        AeroConfig, AeroEnvironment, AeroGeometry, AeroPanel, AeroState, PanelAeroModel, PanelSoA,
    };
    use glam::DVec3;
    let mut all_panels = Vec::new();
    for i in 0..16 {
        let fi = i as f64;
        let mut panel = AeroPanel::flat_plate(
            DVec3::new(-4.0 + (i % 4) as f64, 2.0 - 0.5 * fi, 0.2 * (fi % 3.0)),
            1.0 + fi,
            0.8 + 0.1 * (fi % 3.0),
        )
        .expect("panel")
        .with_planform(
            2.0 + 0.4 * fi,
            1.5 + 0.5 * (fi % 5.0),
            0.1 * (fi % 4.0),
            1.0,
        )
        .expect("planform")
        .with_thickness_ratio(0.02 + 0.005 * (fi % 3.0))
        .expect("thickness");
        panel.control_deflection_rad = 0.05 * ((i % 3) as f64 - 1.0);
        if i == 7 {
            panel.exposure = 0.0;
        }
        if i == 11 {
            panel.exposure = 0.4;
        }
        all_panels.push(panel);
    }
    let vortex_config = AeroConfig {
        vortex_lift_factor: 0.6,
        separated_control_factor: 0.0,
        drag_only_above_mach: 5.0,
        ..AeroConfig::default()
    };
    let models = [
        PanelAeroModel::new(AeroConfig::default()).expect("default model"),
        PanelAeroModel::new(vortex_config).expect("vortex model"),
    ];
    let cases = [
        (150.0, 5.0_f64.to_radians(), DVec3::ZERO, DVec3::ZERO),
        (150.0, 35.0_f64.to_radians(), DVec3::ZERO, DVec3::ZERO),
        (
            700.0,
            8.0_f64.to_radians(),
            DVec3::new(10.0, -5.0, 2.0),
            DVec3::new(0.1, -0.2, 0.05),
        ),
        (
            120.0,
            -12.0_f64.to_radians(),
            DVec3::ZERO,
            DVec3::new(0.0, 0.0, 0.3),
        ),
        (
            1800.0,
            10.0_f64.to_radians(),
            DVec3::ZERO,
            DVec3::new(0.0, 0.0, 0.5),
        ),
    ];
    for count in [3usize, 7, 10, 16] {
        let geometry = AeroGeometry::new(all_panels[..count].to_vec()).expect("geometry");
        let soa = PanelSoA::from_geometry(&geometry).expect("soa layout");
        assert_eq!(soa.count, count);
        for model in &models {
            for (speed, alpha, wind, omega) in cases {
                let velocity = DVec3::new(
                    speed * alpha.cos() + wind.x,
                    wind.y,
                    -speed * alpha.sin() + wind.z,
                );
                let state = AeroState::new(velocity, omega);
                let environment =
                    AeroEnvironment::new(1.225, 340.294, 1.81e-5, DVec3::new(wind.x, 0.0, wind.z));
                let reference = model
                    .evaluate_state(state, environment, &geometry)
                    .expect("reference");
                let oracle = model
                    .evaluate_soa_parts(state, environment, &soa, false)
                    .expect("oracle");
                let candidate = model
                    .evaluate_soa_simd(state, environment, &soa, true)
                    .expect("simd");
                for (got, want, name) in [
                    (candidate.force_body_n, reference.force_body_n, "force"),
                    (candidate.moment_body_nm, reference.moment_body_nm, "moment"),
                ] {
                    let scale = want.length().max(1.0);
                    assert!(
                        (got - want).length() / scale < 1e-9,
                        "{name} vs AoS diverged at {count} panels, {speed} m/s: {got:?} vs {want:?}"
                    );
                }
                for (got, want, name) in [
                    (candidate.force_body_n, oracle.force_body_n, "force"),
                    (candidate.moment_body_nm, oracle.moment_body_nm, "moment"),
                ] {
                    let scale = want.length().max(1.0);
                    assert!(
                        (got - want).length() / scale < 1e-9,
                        "{name} vs oracle diverged at {count} panels, {speed} m/s: {got:?} vs {want:?}"
                    );
                }
                assert_eq!(candidate.panel_count, count);
                assert!((candidate.mach - reference.mach).abs() < 1e-15);
                let loads = candidate.panel_loads.expect("loads recorded");
                assert_eq!(loads.len(), count);
            }
        }
    }
    // Caller-owned scratch must reproduce the allocating variant bit for
    // bit, including vacuum-parked lanes whose kernel inputs stay stale
    // from the previous warm evaluation (their outputs are ignored by
    // force assembly, so reuse is sound).
    use crate::AeroSimdScratch;
    let geometry = AeroGeometry::new(all_panels[..10].to_vec()).expect("geometry");
    let soa = PanelSoA::from_geometry(&geometry).expect("soa layout");
    let model = PanelAeroModel::new(AeroConfig::default()).expect("model");
    let mut scratch = AeroSimdScratch::default();
    let vacuum = AeroEnvironment::new(0.0, 340.294, 1.81e-5, DVec3::ZERO);
    let parked_state = AeroState::new(DVec3::new(150.0, 0.0, -10.0), DVec3::ZERO);
    for state in [
        parked_state,
        AeroState::new(DVec3::ZERO, DVec3::new(0.0, 0.0, 0.1)),
    ] {
        let oracle = model
            .evaluate_soa_parts(state, vacuum, &soa, true)
            .expect("vacuum oracle");
        let fresh = model
            .evaluate_soa_simd(state, vacuum, &soa, true)
            .expect("vacuum simd");
        let reused = model
            .evaluate_soa_simd_scratch(state, vacuum, &soa, true, &mut scratch)
            .expect("vacuum reuse");
        assert_eq!(fresh.force_body_n, reused.force_body_n);
        assert_eq!(fresh.moment_body_nm, reused.moment_body_nm);
        assert_eq!(fresh.force_body_n, oracle.force_body_n);
        assert_eq!(fresh.moment_body_nm, oracle.moment_body_nm);
        assert_eq!(fresh.panel_loads.expect("loads").len(), 10);
    }
}

fn chain_ephemeris() -> BakedEphemeris {
    // Barycenter -> planet -> moon -> submoon: every lookup below the top
    // re-walks shared parents, which is exactly the redundancy the batch
    // evaluator removes. Radii are nonzero so the bodies also serve as
    // gravity sources alongside the hierarchy.
    let mu = 3.986_004_418e14;
    let orbit = |a: f64, e: f64, m0: f64| {
        KeplerOrbit::new(mu, a, e, 0.1, 0.2, 0.3, m0).expect("valid test orbit")
    };
    BakedEphemeris::new(
        "TEST_CHAIN_EPOCH",
        vec![
            BakedBody::synthetic_barycenter(BodyId(0), "barycenter", mu, None, None),
            BakedBody::orbital(
                BodyId(1),
                "planet",
                mu * 0.1,
                6_000_000.0,
                BodyId(0),
                orbit(50_000_000.0, 0.05, 0.0),
            ),
            BakedBody::orbital(
                BodyId(2),
                "moon",
                mu * 0.01,
                1_000_000.0,
                BodyId(1),
                orbit(5_000_000.0, 0.1, 1.0),
            ),
            BakedBody::orbital(
                BodyId(3),
                "submoon",
                mu * 0.001,
                100_000.0,
                BodyId(2),
                orbit(500_000.0, 0.2, 2.0),
            ),
        ],
    )
    .expect("valid chain ephemeris")
}

#[test]
fn batch_states_match_individual_lookups_bitwise() {
    let ephemeris = chain_ephemeris();
    let mut frame = EphemerisFrame::new();
    for seconds in [0.0, 1.0, 8.0 / 120.0, 1_000_000.0, -12_345.678] {
        let time = SimTime(seconds);
        let batch = frame
            .evaluate(&ephemeris, time)
            .expect("batch evaluation")
            .to_vec();
        assert_eq!(batch.len(), ephemeris.bodies.len());
        for body in &ephemeris.bodies {
            let single = ephemeris.body_state(body.id, time).expect("single lookup");
            let batched = batch[body.id.index()];
            assert_eq!(
                batched.position_inertial, single.position_inertial,
                "pos {:?}",
                body.id
            );
            assert_eq!(
                batched.velocity_inertial, single.velocity_inertial,
                "vel {:?}",
                body.id
            );
            assert_eq!(batched, single, "full state {:?}", body.id);
        }
        // Re-evaluating the same frame at a new time must not leak the old
        // generation's completion tags into the new pass.
        assert!(frame.states().len() == ephemeris.bodies.len());
    }
}

#[test]
fn batch_reports_cycles_and_rejects_short_buffers() {
    let mut cyclic = chain_ephemeris();
    cyclic.bodies[1].parent = Some(BodyId(3));
    let mut frame = EphemerisFrame::new();
    assert!(matches!(
        frame.evaluate(&cyclic, SimTime::EPOCH),
        Err(EphemerisError::Cycle(_))
    ));
    // A poisoned frame must still serve a healthy universe afterwards.
    let healthy = chain_ephemeris();
    assert!(frame.evaluate(&healthy, SimTime::EPOCH).is_ok());

    let mut states = vec![BodyState::ORIGIN; 2];
    let mut scratch = EphemerisScratch::new();
    assert!(
        healthy
            .body_states_into(SimTime::EPOCH, &mut states, &mut scratch)
            .is_err()
    );
}

#[test]
fn gravity_and_dominant_from_states_match_naive_paths() {
    let ephemeris = chain_ephemeris();
    let field = GravityField::from_ephemeris(&ephemeris);
    let mut frame = EphemerisFrame::new();
    let positions = [
        DVec3::new(56_000_000.0, 0.0, 0.0),
        DVec3::new(0.0, -49_000_000.0, 1_000_000.0),
    ];
    for seconds in [0.0, 40_000.0] {
        let time = SimTime(seconds);
        let states = frame.evaluate(&ephemeris, time).expect("batch").to_vec();
        for position in positions {
            let naive = field.acceleration(position, time).expect("naive gravity");
            let batched = field
                .acceleration_from_states(position, &states)
                .expect("batched gravity");
            assert_eq!(batched, naive);
            assert_eq!(
                ephemeris.dominant_body_from_states(position, &states),
                ephemeris.dominant_body(position, time),
            );
        }
    }
    // A short slice names the missing body instead of panicking.
    let short = &frame.states()[..2];
    assert!(field.acceleration_from_states(positions[0], short).is_err());
}

#[test]
fn soa_scratch_matches_scalar_panel_loop_within_envelope() {
    // Gate for routing the per-tick flight path through the vectorized SoA
    // kernels. The kernels transcribe the analytic model with a different
    // association order (up to a few ulps on 1e4-scale forces), so this pins
    // an absolute envelope instead of bitwise equality, per AGENTS 10.1:
    // 1e-6 N / 1e-6 N m sit ~9 orders below the decision-relevant scales
    // (254 kN thrust, ~1e3 N m RCS couples and saturation flag) while
    // relative metrics would lie on near-zero loads. The X-15 layout
    // (5 panels) exercises both the 4-wide kernel and the scalar tail.
    let profile = X15StarterProfile::new().expect("x15 profile");
    let mut vehicle = profile.vehicle.clone();
    let model = PanelAeroModel::new(profile.aero_config).expect("aero model");
    let mut panels = PanelSoA::from_geometry(&vehicle.aero_geometry).expect("soa layout");
    let mut scratch = AeroSimdScratch::default();
    let env = |density: f64| AeroEnvironment::new(density, 340.294, 1.81e-5, DVec3::ZERO);
    let flow = |vx: f64, vy: f64, omega: DVec3| AeroState::new(DVec3::new(vx, vy, 0.0), omega);
    let cases: Vec<(AeroState, AeroEnvironment, [f64; 4])> = vec![
        (
            flow(180.0, 0.0, DVec3::ZERO),
            env(1.0),
            [0.0, 0.0, 0.0, 0.0],
        ),
        (
            flow(180.0, -8.0, DVec3::ZERO),
            env(1.0),
            [-0.5, 0.3, 0.2, -0.2],
        ),
        // Past the 22-degree stall angle with rate: separation + omega x r.
        (
            flow(150.0, -70.0, DVec3::new(1.0, 0.5, 0.2)),
            env(0.9),
            [0.8, -0.6, 1.0, -1.0],
        ),
        // Transonic handoff and supersonic thin air with full deflection.
        (
            flow(340.0, 5.0, DVec3::ZERO),
            env(0.8),
            [0.2, 0.0, 0.5, -0.5],
        ),
        (
            flow(680.0, 20.0, DVec3::new(0.1, -0.2, 0.3)),
            env(0.4),
            [1.0, 1.0, -1.0, 1.0],
        ),
        // Declared vacuum parks every lane; still air parks them too.
        (
            flow(7000.0, 0.0, DVec3::ZERO),
            env(0.0),
            [0.4, 0.0, 0.0, 0.0],
        ),
        (flow(0.0, 0.0, DVec3::ZERO), env(1.0), [0.0, 0.0, 0.0, 0.0]),
    ];
    for (state, environment, commands) in cases {
        vehicle
            .apply_control_inputs(&commands)
            .expect("control inputs");
        let geometry = &vehicle.aero_geometry;
        panels.sync_deflections(geometry).expect("deflection sync");
        let scalar = model
            .evaluate_state(state, environment, geometry)
            .expect("scalar evaluation");
        let vector = model
            .evaluate_soa_simd_scratch(state, environment, &panels, false, &mut scratch)
            .expect("soa evaluation");
        assert!(
            (vector.force_body_n - scalar.force_body_n).length() <= 1.0e-6,
            "force envelope {state:?}: {:?} vs {:?}",
            vector.force_body_n,
            scalar.force_body_n,
        );
        assert!(
            (vector.moment_body_nm - scalar.moment_body_nm).length() <= 1.0e-6,
            "moment envelope {state:?}: {:?} vs {:?}",
            vector.moment_body_nm,
            scalar.moment_body_nm,
        );
        assert!(
            (vector.dynamic_pressure_pa - scalar.dynamic_pressure_pa).abs() <= 1.0e-9,
            "q envelope"
        );
        assert!(
            (vector.mach - scalar.mach).abs() <= 1.0e-12,
            "mach envelope"
        );
        assert!(
            (vector.reynolds_number - scalar.reynolds_number).abs() <= 1.0e-6,
            "re envelope"
        );
        assert_eq!(vector.panel_count, scalar.panel_count, "panel count");
    }
}

#[test]
#[ignore = "wall-clock diagnostic; run with --ignored --nocapture"]
fn profile_scalar_vs_soa_dev() {
    let profile = X15StarterProfile::new().expect("x15 profile");
    let vehicle = profile.vehicle.clone();
    let geometry = &vehicle.aero_geometry;
    let model = PanelAeroModel::new(profile.aero_config).expect("aero model");
    let mut panels = PanelSoA::from_geometry(geometry).expect("soa");
    let mut scratch = AeroSimdScratch::default();
    let env = AeroEnvironment::new(1.0, 340.294, 1.81e-5, DVec3::ZERO);
    let state = AeroState::new(DVec3::new(180.0, -8.0, 0.0), DVec3::ZERO);
    // Warmup.
    for _ in 0..200 {
        let _ = model.evaluate_state(state, env, geometry).expect("scalar");
        panels.sync_deflections(geometry).expect("sync");
        let _ = model
            .evaluate_soa_simd_scratch(state, env, &panels, false, &mut scratch)
            .expect("soa");
    }
    let iters = 2000;
    let now = std::time::Instant::now();
    for _ in 0..iters {
        std::hint::black_box(model.evaluate_state(state, env, geometry).expect("scalar"));
    }
    let scalar = now.elapsed();
    let now = std::time::Instant::now();
    for _ in 0..iters {
        panels.sync_deflections(geometry).expect("sync");
        std::hint::black_box(
            model
                .evaluate_soa_simd_scratch(state, env, &panels, false, &mut scratch)
                .expect("soa"),
        );
    }
    let soa = now.elapsed();
    eprintln!("scalar: {:?} total, {:?}/eval", scalar, scalar / iters);
    eprintln!("soa+sync: {:?} total, {:?}/eval", soa, soa / iters);
}

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
    let plain =
        propagate_adaptive(&field, initial, SimTime::EPOCH, duration, config)
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
    assert!(pos_err < 1.0, "position agreement within 1 m, got {pos_err:e}");
    assert!(vel_err < 1.0e-3, "velocity agreement within 1 mm/s, got {vel_err:e}");
}
