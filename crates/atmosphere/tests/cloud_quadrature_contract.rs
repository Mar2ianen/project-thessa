//! Analytic reference tests for the *production* CPU mirror of cloud march A.
//!
//! These tests do not copy the implementation's previous-density recurrence.
//! Constant and affine density have closed-form Beer-Lambert solutions,
//! making it impossible to manufacture a matching reference by duplicating
//! the algorithm under test.
//!
//! Run:
//!   cargo test -p thessa-atmosphere --test cloud_quadrature_contract -- --nocapture
//!
//! The affine cases are intentionally RED on the reviewed base commit:
//! using a current/previous midpoint average introduces first-order bias.
//! Fix the numerical method, not the analytic reference or its tolerance.

use thessa_atmosphere::clouds::integrate_slab_gpu_trapezoidal_model;

fn assert_close(actual: f64, expected: f64, tolerance: f64, quantity: &str) {
    assert!(
        actual.is_finite() && (actual - expected).abs() <= tolerance,
        "{quantity}: actual={actual:.15}, expected={expected:.15}, abs_error={:.9e}, tolerance={tolerance}",
        (actual - expected).abs()
    );
}

#[test]
fn homogeneous_cloud_has_exact_beer_lambert_transmission() {
    // Analytical solution: tau = sigma_t * rho * distance.
    let rho = 0.4;
    let sigma = 0.35;
    let distance = 8.0;
    let expected_tau: f64 = sigma * rho * distance;

    for steps in [1_usize, 8, 24, 64] {
        let (transmission, optical_depth) =
            integrate_slab_gpu_trapezoidal_model(|_| rho, 0.0, distance, steps, sigma);
        assert_close(optical_depth, expected_tau, 1e-12, "uniform optical depth");
        assert_close(
            transmission,
            (-expected_tau).exp(),
            1e-12,
            "uniform transmission",
        );
    }
}

#[test]
fn affine_cloud_density_must_preserve_its_analytic_column_integral() {
    // Independent analytic oracle for rho(t) = 0.2 + 0.6t on [0,1]:
    // integral rho(t) dt = 0.2 + 0.6/2 = 0.5.
    // Both midpoint and correctly aligned trapezoidal integration are exact
    // for an affine function. Mixing successive midpoint samples is not.
    let extinction = 0.25;
    let expected_tau: f64 = extinction * 0.5;

    for steps in [4_usize, 8, 16, 24, 64] {
        let (transmission, depth) =
            integrate_slab_gpu_trapezoidal_model(|t| 0.2 + 0.6 * t, 0.0, 1.0, steps, extinction);

        assert_close(depth, expected_tau, 1e-12, "affine optical depth");
        assert_close(
            transmission,
            (-expected_tau).exp(),
            1e-12,
            "affine transmission",
        );
    }
}

#[test]
fn affine_integral_is_invariant_under_translation_of_ray_parameter() {
    // An identical physical profile on a shifted ray interval must
    // retain its analytic optical depth. This checks the actual t0/t1
    // coordinate semantics without using another numerical integrator.
    let extinction = 0.4;
    let expected_tau: f64 = 0.4 * (0.1 + 0.8 / 2.0) * 2.0;

    for t0 in [0.0_f64, 13.0, 1_000_000.0] {
        let (transmission, depth) = integrate_slab_gpu_trapezoidal_model(
            |t| 0.1 + 0.8 * ((t - t0) / 2.0),
            t0,
            t0 + 2.0,
            24,
            extinction,
        );
        assert_close(depth, expected_tau, 1e-9, "shifted affine optical depth");
        assert_close(
            transmission,
            (-expected_tau).exp(),
            1e-9,
            "shifted affine transmission",
        );
    }
}
