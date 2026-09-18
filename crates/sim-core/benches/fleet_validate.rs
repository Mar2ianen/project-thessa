//! Lockstep validation for the cohort layer (not a KPI bench): exact and
//! cohort gravity advance side by side, and every tick checks the per-target
//! acceleration error against the posted bound — panicking with tick/index
//! detail on violation. Tracks peak state divergence over the run alongside
//! the final one, so partial error compensation cannot hide behind a small
//! endpoint number. Especially important for window reuse, whose staleness
//! accumulates mid-run and resets on rebuild.
use std::hint::black_box;

use glam::DVec3;
use thessa_sim_core::{
    CohortConfig, CohortEvaluator, EphemerisFrame, GravityField, SimTime, SystemConfig,
};

const TICKS: usize = 2_000;
const STEP_S: f64 = 0.5;
const COUNT: usize = 64;
const BUDGET: f64 = 1.0e-9;

fn validate_scenario(
    name: &str,
    field: &GravityField,
    ephemeris: &thessa_sim_core::BakedEphemeris,
    anchor: DVec3,
    base_velocity: DVec3,
) {
    let mut positions: Vec<_> = (0..COUNT)
        .map(|index| {
            let i = index as f64;
            anchor
                + DVec3::new(
                    (i * 12.9898).sin() * 25_000.0,
                    (i * 78.233).sin() * 25_000.0,
                    (i * 37.719).sin() * 25_000.0,
                )
        })
        .collect();
    let mut velocities = vec![base_velocity; COUNT];
    let mut exact_positions = positions.clone();
    let mut exact_velocities = velocities.clone();
    let config = CohortConfig {
        error_budget_mps2: BUDGET,
        ..Default::default()
    };
    let mut frame = EphemerisFrame::new();
    let mut evaluator = CohortEvaluator::new();
    let mut peak_accel_error = 0.0_f64;
    let mut peak_dx = 0.0_f64;
    let mut peak_dv = 0.0_f64;
    let mut peak_posted = 0.0_f64;
    let mut time = SimTime::EPOCH;
    for tick in 0..TICKS {
        time = SimTime(time.0 + STEP_S);
        let states = frame.evaluate(ephemeris, time).expect("frame states");
        let eval = evaluator
            .evaluate(
                black_box(ephemeris),
                black_box(states),
                black_box(&positions),
                config,
            )
            .expect("cohort eval");
        let exact = field
            .accelerations_from_frame(black_box(&positions), black_box(states))
            .expect("exact batch");
        peak_posted = peak_posted.max(eval.error_bound_mps2);
        for (index, (computed, reference)) in eval.accelerations.iter().zip(&exact).enumerate() {
            let error = (*computed - *reference).length();
            peak_accel_error = peak_accel_error.max(error);
            assert!(
                error <= eval.error_bound_mps2 * (1.0 + 1.0e-6) + 1.0e-15,
                "{name} tick {tick} target {index}: accel error {error:e} escapes posted {:e}",
                eval.error_bound_mps2,
            );
            exact_velocities[index] += *reference * STEP_S;
            exact_positions[index] += exact_velocities[index] * STEP_S;
            velocities[index] += *computed * STEP_S;
            positions[index] += velocities[index] * STEP_S;
            peak_dx = peak_dx.max((positions[index] - exact_positions[index]).length());
            peak_dv = peak_dv.max((velocities[index] - exact_velocities[index]).length());
        }
        black_box(&positions);
    }
    // NOTE: peak dx/dv is the validation metric; the 40k KPI bench reports
    // final-state divergence only (see its header).
    println!(
        "{name} x{COUNT} x{TICKS} ticks: peak accel error {peak_accel_error:e} (posted max {peak_posted:e}) | peak dx {peak_dx:.6} m, peak dv {peak_dv:.9} m/s | reuses {}, rebuilds {}",
        evaluator.reuses, evaluator.rebuilds,
    );
}

fn main() {
    let config: SystemConfig = toml::from_str(include_str!("../../../data/system.toml")).unwrap();
    let ephemeris = config.bake().unwrap();
    let field = GravityField::from_ephemeris(&ephemeris);
    let home = ephemeris
        .body_state(ephemeris.body_id("thessa").unwrap(), SimTime::EPOCH)
        .unwrap();
    let thessa_radius_m = ephemeris
        .bodies
        .iter()
        .find(|body| body.name == "thessa")
        .expect("thessa body")
        .radius_m;
    let mu = ephemeris
        .bodies
        .iter()
        .find(|body| body.name == "thessa")
        .expect("thessa body")
        .mu;
    validate_scenario(
        "deep-space cruise",
        &field,
        &ephemeris,
        home.position_inertial + DVec3::Z * 1e9,
        home.velocity_inertial + DVec3::X * 100.0,
    );
    let radius = thessa_radius_m + 500_000.0;
    validate_scenario(
        "low-orbit 500km",
        &field,
        &ephemeris,
        home.position_inertial + DVec3::new(radius, 0.0, 0.0),
        home.velocity_inertial + DVec3::new(0.0, (mu / radius).sqrt(), 0.0),
    );
}
