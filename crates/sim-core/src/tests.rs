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

#[path = "tests/celestial.rs"]
mod celestial;

#[path = "tests/affine.rs"]
mod affine;

#[path = "tests/aerodynamics.rs"]
mod aerodynamics;

#[path = "tests/rails.rs"]
mod rails;

#[path = "tests/aero_kernels.rs"]
mod aero_kernels;

#[path = "tests/thrust_arcs.rs"]
mod thrust_arcs;

#[path = "tests/harmonics.rs"]
mod harmonics;

#[path = "tests/vehicle_effectors.rs"]
mod vehicle_effectors;
