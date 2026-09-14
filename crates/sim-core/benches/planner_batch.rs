//! Planner candidate search through one shared patch (docs/23_GRAVITY_FIELD_COHORTS.md
//! section 12): hundreds/thousands of candidate burns share the same start
//! epoch, start region and celestial configuration. Instead of compiling
//! gravity independently per candidate, one patch serves them all; staged
//! search runs a loose first pass, then tighter patches for survivors.
//!
//! No maneuver planner exists yet (ManeuverNode is reserved), so this bench
//! measures the workload shape the planner will have: N candidate positions
//! at one epoch through one patch compiler, against the exact batch, with
//! max error pinned against the posted bound.
use std::{hint::black_box, time::Instant};

use glam::DVec3;
use thessa_sim_core::{
    CohortConfig, CohortEvaluator, EphemerisFrame, GravityField, SimTime, SystemConfig,
};

fn candidates(center: DVec3, span_m: f64, count: usize) -> Vec<DVec3> {
    (0..count)
        .map(|index| {
            let i = index as f64;
            center
                + DVec3::new(
                    (i * 12.9898).sin() * span_m,
                    (i * 78.233).sin() * span_m,
                    (i * 37.719).sin() * span_m,
                )
        })
        .collect()
}

fn main() {
    let config: SystemConfig = toml::from_str(include_str!("../../../data/system.toml")).unwrap();
    let ephemeris = config.bake().unwrap();
    let field = GravityField::from_ephemeris(&ephemeris);
    let time = SimTime::EPOCH;
    let mut frame = EphemerisFrame::new();
    let states = frame
        .evaluate(&ephemeris, time)
        .expect("frame states")
        .to_vec();
    let home = ephemeris
        .body_state(ephemeris.body_id("thessa").unwrap(), time)
        .unwrap();
    // Candidate cloud where a planner would search: deep-space corridor.
    let center = home.position_inertial + DVec3::Z * 1e9;

    for count in [1_000, 10_000] {
        let positions = candidates(center, 200_000.0, count);
        // Reference: exact framed batch, timed.
        let started = Instant::now();
        let exact = field
            .accelerations_from_frame(black_box(&positions), black_box(&states))
            .expect("exact batch");
        let exact_elapsed = started.elapsed();
        for budget in [1.0e-6, 1.0e-9] {
            let config = CohortConfig {
                error_budget_mps2: budget,
                ..Default::default()
            };
            let mut evaluator = CohortEvaluator::new();
            let started = Instant::now();
            let eval = evaluator
                .evaluate(
                    black_box(&ephemeris),
                    black_box(&states),
                    black_box(&positions),
                    config,
                )
                .expect("candidate batch");
            let elapsed = started.elapsed();
            let mut max_error = 0.0_f64;
            for (computed, reference) in eval.accelerations.iter().zip(&exact) {
                max_error = max_error.max((*computed - *reference).length());
            }
            assert!(
                max_error <= eval.error_bound_mps2 * (1.0 + 1.0e-6),
                "candidate error {max_error:e} escapes posted {:e}",
                eval.error_bound_mps2,
            );
            println!(
                "planner x{count} budget {budget:e}: shared patch {elapsed:?} ({:.1} ns/candidate, {} cohorts, {} splits, posted {:e}) vs exact {exact_elapsed:?} (x{:.1}) | max error {max_error:e} m/s^2",
                elapsed.as_secs_f64() * 1e9 / count as f64,
                eval.cohort_count,
                eval.split_count,
                eval.error_bound_mps2,
                exact_elapsed.as_secs_f64() / elapsed.as_secs_f64(),
            );
        }
    }
}
