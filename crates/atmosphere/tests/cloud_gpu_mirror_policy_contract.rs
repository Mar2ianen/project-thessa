//! Independent CPU/GPU cloud-integrator policy regression tests.
//!
//! Test subject: the *public production* CPU mirror
//! `integrate_slab_gpu_trapezoidal_model` in thessa-atmosphere.
//!
//! Authoritative policy at review HEAD 2d381d (production shader):
//! `assets/shaders/cloud_volume.wgsl`, Backend A:
//!   if (density <= 0.001) { continue; }
//!   if (transmittance < 0.01) { transmittance = 0.0; break; }
//!
//! A shader change that intentionally alters these thresholds must update
//! the CPU mirror and these contract tests together. This is NOT a GPU test:
//! the GPU-specific values are pinned as source-backed expected behavior.
//!
//! Run:
//!   cargo test -p thessa-atmosphere --test cloud_gpu_mirror_policy_contract -- --nocapture
//!
//! On reviewed HEAD: three tests are expected RED. Do not weaken them;
//! fix the implementation or explicitly re-specify both CPU and GPU policy.

use thessa_atmosphere::clouds::integrate_slab_gpu_trapezoidal_model;

fn march_constant(density: f64, extinction: f64, length: f64, n: usize) -> (f64, f64) {
    integrate_slab_gpu_trapezoidal_model(|_| density, 0.0, length, n, extinction)
}

fn near(actual: f64, expected: f64, tolerance: f64, label: &str) {
    assert!(
        actual.is_finite() && (actual - expected).abs() <= tolerance,
        "{label}: actual={actual:.12}, expected={expected:.12}, abs_error={:.8e}",
        (actual - expected).abs(),
    );
}

#[test]
fn subthreshold_density_does_not_absorb_in_production_backend_a() {
    // WGSL: if (density <= 0.001) continue;
    // Using high extinction makes accidental CPU inclusion of tiny samples
    // produce a conspicuously WRONG transmittance instead of a rounding blip.
    let (transmittance, optical_depth) = march_constant(0.0008, 1000.0, 1.0, 24);
    near(transmittance, 1.0, 0.0, "subthreshold transmission");
    near(optical_depth, 0.0, 0.0, "subthreshold optical depth");
}

#[test]
fn density_equal_to_gpu_skip_boundary_is_skipped() {
    // Strict comparison in the shader is <=, not <.
    let (transmittance, optical_depth) = march_constant(0.001, 1000.0, 1.0, 24);
    near(transmittance, 1.0, 0.0, "boundary transmission");
    near(optical_depth, 0.0, 0.0, "boundary optical depth");
}

#[test]
fn density_above_gpu_skip_boundary_remains_absorbing() {
    let density = 0.0012_f64;
    let extinction = 1000.0;
    let expected_tau = density * extinction;

    let (transmittance, optical_depth) = march_constant(density, extinction, 1.0, 24);
    near(optical_depth, expected_tau, 1e-12, "above-boundary optical depth");
    near(transmittance, (-expected_tau).exp(), 1e-12, "above-boundary transmission");
}

#[test]
fn gpu_early_termination_sets_transmittance_exactly_zero() {
    // Production WGSL: if (transmittance < 0.01) { transmittance = 0; break; }
    // Full analytic homogeneous-cloud T = e^-5 = 0.0067379, so production
    // must terminate early and return T=0, whereas the reviewed CPU mirror
    // uses a different cutoff (< 1e-4) and incorrectly returns e^-5.
    let (transmittance, _) = march_constant(0.5, 1.0, 10.0, 24);
    near(transmittance, 0.0, 0.0, "GPU terminal transmission");
}

#[test]
fn optically_thin_cloud_retains_analytic_beer_lambert_transmission() {
    // A positive control ensures that matching GPU early-out behavior does
    // not accidentally clamp moderately transmitting cloud segments to zero.
    let expected_tau: f64 = 0.5 * 0.4 * 1.0;
    let (transmittance, optical_depth) = march_constant(0.5, 0.4, 1.0, 24);

    near(optical_depth, expected_tau, 1e-12, "thin-cloud optical depth");
    near(transmittance, (-expected_tau).exp(), 1e-12, "thin-cloud transmission");
}
