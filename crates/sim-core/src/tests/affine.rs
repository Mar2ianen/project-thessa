use super::*;

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

/// Excursion and source drift must be charged jointly. A target at the
/// excursion edge and a source drifting past it can meet inside the
/// interval — that IS the rebuild signal (INFINITY), not a finite bound.
/// The temporal clearance used to subtract only the drift, so an excursion
/// plus drift summing to more than the anchor distance still posted a
/// finite (and understated) bound.
#[test]
fn analytic_bound_expires_when_excursion_and_drift_close_the_gap() {
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
    // Sources sit within 1e8 m of the origin, so every anchor distance is
    // ~3.16e9 m: the 2.5e9 m excursion alone leaves ~6.6e8 m of clearance.
    // Orbit speeds are 500..1500 m/s, so over 2e6 s the sources displace
    // 1e9..3e9 m — drift alone closes the remaining gap.
    let excursion = 2.5e9;
    let interval_s = 2.0e6;
    let bound = affine_segment_bound(&ephemeris, &states, &patch, excursion, interval_s)
        .expect("bound evaluates");
    assert!(
        bound.is_infinite(),
        "excursion {excursion:e} m + drift over {interval_s}s must reach the anchor, \
         but the bound stayed finite at {bound:e}"
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
