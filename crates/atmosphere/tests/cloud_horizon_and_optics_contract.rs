//! Second-wave independent review tests for geometry and cloud column optics.
//!
//! These call exported production functions directly. Expected geometric
//! distances and optical depths come from analytic sphere intersections and
//! calculus, *not* another copy of Thessa's raymarch implementation.
//!
//! Run:
//!   cargo test -p thessa-atmosphere --test cloud_horizon_and_optics_contract -- --nocapture
//!
//! On the old PR #7 base, the inward-at-surface and nonconstant-density tests
//! are expected to fail. The horizon tests should remain green after the
//! occlusion fix; that is their purpose (prevent a repair from hiding clouds).

use glam::DVec3;
use thessa_atmosphere::clouds::{
    clip_to_planet, integrate_slab_gpu_trapezoidal_model, intersect_shell, CloudSlab,
};

const PLANET_M: f64 = 3_200_000.0;
const BOT_M: f64 = PLANET_M + 25_600.0;
const TOP_M: f64 = PLANET_M + 54_720.0;

fn envelope() -> CloudSlab {
    CloudSlab::new(PLANET_M, BOT_M, TOP_M).unwrap()
}

fn visible(origin: DVec3, direction: DVec3, center: DVec3) -> Option<(f64, f64)> {
    let (t0, t1) = intersect_shell(origin, direction, center, envelope())?;
    clip_to_planet(t0, t1, origin, direction, center, PLANET_M)
}

fn near(actual: f64, expected: f64, tol: f64, label: &str) {
    assert!(
        actual.is_finite() && (actual - expected).abs() <= tol,
        "{label}: got {actual:.12}, expected {expected:.12}, diff={:.6e}",
        (actual - expected).abs(),
    );
}

#[test]
fn exact_surface_inward_occlusion_holds_for_each_planet_axis() {
    let center = DVec3::new(12_000_000.0, -22_000_000.0, 5_000_000.0);

    for axis in [
        DVec3::X,
        DVec3::NEG_X,
        DVec3::Y,
        DVec3::NEG_Y,
        DVec3::Z,
        DVec3::NEG_Z,
    ] {
        let origin = center + axis * PLANET_M;
        assert!(
            visible(origin, -axis, center).is_none(),
            "a ray launched inward from exact surface {axis:?} must not reveal far-side clouds",
        );
    }
}

#[test]
fn true_surface_tangent_must_keep_near_side_horizon_clouds() {
    let origin = DVec3::new(PLANET_M, 0.0, 0.0);
    let tangent = DVec3::Y;

    // Analytic intersection of a tangent ray from r=R with concentric
    // spheres r=R+h: t(h) = sqrt((R+h)^2 - R^2).
    let expected_entry = (BOT_M * BOT_M - PLANET_M * PLANET_M).sqrt();
    let expected_exit = (TOP_M * TOP_M - PLANET_M * PLANET_M).sqrt();
    let (t0, t1) = visible(origin, tangent, DVec3::ZERO)
        .expect("horizon-visible clouds must not be discarded as planet-occluded");

    near(t0, expected_entry, 1e-5, "surface tangent cloud entry");
    near(t1, expected_exit, 1e-5, "surface tangent cloud exit");
}

#[test]
fn orbital_nadir_must_keep_only_frontside_cloud_interval() {
    let altitude_m = 200_000.0;
    let origin = DVec3::new(PLANET_M + altitude_m, 0.0, 0.0);
    let (t0, t1) = visible(origin, DVec3::NEG_X, DVec3::ZERO)
        .expect("orbiter above clouds must see its near-side deck");

    // Purely radial distances, no numerical sphere-intersection oracle.
    near(t0, altitude_m - (TOP_M - PLANET_M), 1e-5, "orbital front entry");
    near(t1, altitude_m - (BOT_M - PLANET_M), 1e-5, "orbital front exit");
}

#[test]
fn orbital_grazing_chord_matches_closed_form_and_misses_solid_planet() {
    let cam_r = PLANET_M + 200_000.0;
    let impact_r = (BOT_M + TOP_M) * 0.5;
    let sin_theta = impact_r / cam_r;
    let direction = DVec3::new(-(1.0 - sin_theta * sin_theta).sqrt(), sin_theta, 0.0);
    let origin = DVec3::new(cam_r, 0.0, 0.0);

    let (t0, t1) = visible(origin, direction, DVec3::ZERO)
        .expect("limb ray whose impact parameter exceeds planetary radius sees clouds");
    let expected_chord = 2.0 * (TOP_M * TOP_M - impact_r * impact_r).sqrt();
    near(t1 - t0, expected_chord, 1e-4, "orbital tangent chord");
    assert!(t0 > 0.0, "clouds must start ahead of the observer");
}

#[test]
fn descending_affine_density_integrates_to_analytic_optical_depth() {
    // rho(t)=0.9-0.6t, so integral_0^1 rho dt = 0.6.
    // The old previous-midpoint recurrence biases in the opposite direction
    // compared to the increasing affine test in the first review PR.
    let extinction = 0.25_f64;
    let expected_tau = 0.6 * extinction;
    for n in [2_usize, 8, 24, 64] {
        let (t, tau) = integrate_slab_gpu_trapezoidal_model(
            |x| 0.9 - 0.6 * x,
            0.0,
            1.0,
            n,
            extinction,
        );
        near(tau, expected_tau, 1e-12, "descending affine tau");
        near(t, (-expected_tau).exp(), 1e-12, "descending affine transmission");
    }
}

#[test]
fn oscillating_density_has_analytic_mean_at_every_tested_step_budget() {
    // rho(t) = 0.5 + 0.25 sin(2*pi*t), strictly positive on [0,1].
    // Its exact integral is 0.5. Midpoint samples sum to the exact integral
    // for these even budgets; previous-midpoint blending breaks this identity.
    let extinction = 0.25_f64;
    let expected_tau = 0.5 * extinction;

    for n in [8_usize, 24, 64] {
        let (t, tau) = integrate_slab_gpu_trapezoidal_model(
            |x| 0.5 + 0.25 * (std::f64::consts::TAU * x).sin(),
            0.0,
            1.0,
            n,
            extinction,
        );
        near(tau, expected_tau, 1e-12, "periodic density tau");
        near(t, (-expected_tau).exp(), 1e-12, "periodic density transmission");
    }
}
