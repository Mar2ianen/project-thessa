//! End-to-end fleet propagation: 40 000 ticks x N vehicles through full
//! source accumulation (docs/23_GRAVITY_FIELD_COHORTS.md section 16.2).
//!
//! Each tick re-evaluates the ephemeris frame at the new epoch (bodies move),
//! accumulates gravity for every target, and advances all vehicles with one
//! symplectic Euler step. Paths:
//!
//!   - direct: `GravityField::accelerations` (per-target ephemeris lookups);
//!   - framed: one `EphemerisFrame::evaluate` per tick plus
//!     `accelerations_from_frame`;
//!   - cohort: shared affine patches with exact-near terms.
//!
//! The framed path must reproduce the direct path bitwise (asserted on final
//! states): same physics, representation-only change. Main KPI is simulated
//! seconds per wall second at matched trajectory error.
use std::{hint::black_box, time::Instant};

use glam::DVec3;
use thessa_sim_core::{
    CohortConfig, CohortEvaluator, EphemerisFrame, GravityField, SimTime, SystemConfig,
};

const TICKS: usize = 40_000;
const STEP_S: f64 = 0.5;
/// Untimed warmup ticks before every measured path: frequency ramp, caches,
/// allocator and thread-pool state otherwise bias whichever path runs first
/// (measured up to 2x on the microsecond-scale cohort/framed ticks).
const WARMUP_TICKS: usize = 500;

#[derive(Clone, Copy, PartialEq, Debug)]
enum Path {
    Direct,
    Framed,
    Cohort { budget_mps2: f64 },
}

struct Fleet {
    positions: Vec<DVec3>,
    velocities: Vec<DVec3>,
}

fn integrate_step(fleet: &mut Fleet, accelerations: &[DVec3]) {
    for (index, acceleration) in accelerations.iter().enumerate() {
        fleet.velocities[index] += *acceleration * STEP_S;
        fleet.positions[index] += fleet.velocities[index] * STEP_S;
    }
    black_box(&fleet.positions);
}

fn propagate(
    field: &GravityField,
    ephemeris: &thessa_sim_core::BakedEphemeris,
    fleet: &mut Fleet,
    path: Path,
    ticks: usize,
) -> CohortStats {
    let mut frame = EphemerisFrame::new();
    // Persistent across all 40k ticks: scratch reuse + patch window.
    let mut evaluator = CohortEvaluator::new();
    let mut framed_scratch: Vec<DVec3> = Vec::new();
    let mut time = SimTime::EPOCH;
    let mut stats = CohortStats::default();
    // Symplectic Euler straight from the borrowed batch output: no copy.
    for _ in 0..ticks {
        time = SimTime(time.0 + STEP_S);
        match path {
            Path::Direct => {
                let accelerations = field
                    .accelerations(black_box(&fleet.positions), time)
                    .expect("finite direct accelerations");
                integrate_step(fleet, &accelerations);
            }
            Path::Framed => {
                let states = frame
                    .evaluate(ephemeris, time)
                    .expect("finite frame states");
                field
                    .accelerations_from_frame_into(
                        black_box(&fleet.positions),
                        black_box(states),
                        &mut framed_scratch,
                    )
                    .expect("finite frame accelerations");
                integrate_step(fleet, &framed_scratch);
            }
            Path::Cohort { budget_mps2 } => {
                let states = frame
                    .evaluate(ephemeris, time)
                    .expect("finite frame states");
                let config = CohortConfig {
                    error_budget_mps2: budget_mps2,
                    ..Default::default()
                };
                match evaluator.evaluate(
                    black_box(ephemeris),
                    black_box(states),
                    black_box(&fleet.positions),
                    config,
                ) {
                    Ok(eval) => {
                        stats.cohorts += eval.cohort_count as u64;
                        stats.splits += eval.split_count as u64;
                        stats.error_bound = stats.error_bound.max(eval.error_bound_mps2);
                        stats.exact_terms += eval.exact_terms;
                        stats.max_radius_m = stats.max_radius_m.max(eval.max_radius_m);
                        stats.reuses += u64::from(eval.reused_window);
                        integrate_step(fleet, eval.accelerations);
                    }
                    // Fail open by contract: exact frame path on patch error,
                    // with its exact work counted honestly.
                    Err(_) => {
                        stats.fallbacks += 1;
                        stats.exact_terms += (fleet.positions.len() * field.source_count()) as u64;
                        field
                            .accelerations_from_frame_into(
                                &fleet.positions,
                                states,
                                &mut framed_scratch,
                            )
                            .expect("finite fallback accelerations");
                        integrate_step(fleet, &framed_scratch);
                    }
                }
            }
        }
    }
    stats
}

#[derive(Default)]
struct CohortStats {
    cohorts: u64,
    splits: u64,
    fallbacks: u64,
    error_bound: f64,
    exact_terms: u64,
    max_radius_m: f64,
    reuses: u64,
}

fn main() {
    // Optional thread count (doc 23 section 16 scaling axis): the global
    // pool must be built before the first parallel op.
    // Usage: `fleet_prop [size] [threads] [rev]`.
    let args: Vec<String> = std::env::args().collect();
    if let Some(threads) = args.get(2).and_then(|arg| arg.parse::<usize>().ok()) {
        rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build_global()
            .expect("global rayon pool");
    }
    println!("rayon threads: {}", rayon::current_num_threads());

    let config: SystemConfig = toml::from_str(include_str!("../../../data/system.toml")).unwrap();
    let ephemeris = config.bake().unwrap();
    let field = GravityField::from_ephemeris(&ephemeris);
    println!("sources: {}", field.source_count());

    let home = ephemeris
        .body_state(ephemeris.body_id("thessa").unwrap(), SimTime::EPOCH)
        .unwrap();
    let mu = ephemeris
        .bodies
        .iter()
        .find(|body| body.name == "thessa")
        .expect("thessa body")
        .mu;

    // Optional size filter for hypothesis iteration: `fleet_prop 300`.
    // Optional "rev" third arg reverses path order (order-effect bound):
    // `fleet_prop 300 1 rev`. Usage: `fleet_prop [size] [threads] [rev]`.
    let only: Option<usize> = args.get(1).and_then(|arg| arg.parse().ok());
    let reverse = args.get(3).is_some_and(|arg| arg == "rev");

    // Scenario 1 (doc 16.2): deep-space cruise — weak gradients, the convoy
    // holds and one patch serves the whole run.
    run_scenario(
        "deep-space cruise",
        &field,
        &ephemeris,
        home.position_inertial + DVec3::Z * 1e9,
        home.velocity_inertial + DVec3::X * 100.0,
        &[128, 300, 1_000],
        only,
        reverse,
    );
    // Scenarios 2-4: true low orbits at 500/750/1000 km altitude over the    // body radius — strong gradient, exact-near pressure, Kepler shear.
    // Floor justification (Thessa: R = 3200 km, 1.2 bar N2/O2, 0.5 g per the
    // `mass_earth` comment in system.toml): H ~= 287*288/4.9 ~= 16.9 km,
    // sea-level rho ~= 1.45 kg/m^3, so the 1e-10 kg/m^3 declared-vacuum
    // cutoff sits at ~= 395 km. 100 km (rho ~ 4e-3) and even 300 km
    // (rho ~ 3e-8) are atmosphere, not ballistic regime: at orbital speed
    // their drag dwarfs the 1e-9 gravity budget the bench pins. 500 km has
    // ~100 km (~6 H) of margin below the cutoff across plausible T.
    // (The old 6.9e6 m anchor was ~3700 km up — not a low orbit at all.)
    let thessa_radius_m = ephemeris
        .bodies
        .iter()
        .find(|body| body.name == "thessa")
        .expect("thessa body")
        .radius_m;
    for altitude_km in [500, 750, 1_000] {
        let radius = thessa_radius_m + altitude_km as f64 * 1000.0;
        run_scenario(
            &format!("low-orbit {}km", altitude_km),
            &field,
            &ephemeris,
            home.position_inertial + DVec3::new(radius, 0.0, 0.0),
            home.velocity_inertial + DVec3::new(0.0, (mu / radius).sqrt(), 0.0),
            &[300],
            only,
            reverse,
        );
    }
}

fn convoy(anchor: DVec3, base_velocity: DVec3, count: usize) -> Fleet {
    let mut positions = Vec::with_capacity(count);
    let mut velocities = Vec::with_capacity(count);
    for index in 0..count {
        let i = index as f64;
        positions.push(
            anchor
                + DVec3::new(
                    (i * 12.9898).sin() * 25_000.0,
                    (i * 78.233).sin() * 25_000.0,
                    (i * 37.719).sin() * 25_000.0,
                ),
        );
        velocities.push(
            base_velocity
                + DVec3::new(
                    (i * 3.17).sin() * 0.5,
                    (i * 5.71).sin() * 0.5,
                    (i * 9.13).sin() * 0.5,
                ),
        );
    }
    Fleet {
        positions,
        velocities,
    }
}

/// One benchmark scenario: fixed anchor/velocity, several fleet sizes.
/// Eight params read better than a struct here (all call sites pass them
/// positionally once); the library itself stays warning-clean.
#[allow(clippy::too_many_arguments)]
fn run_scenario(
    name: &str,
    field: &GravityField,
    ephemeris: &thessa_sim_core::BakedEphemeris,
    anchor: DVec3,
    base_velocity: DVec3,
    sizes: &[usize],
    only: Option<usize>,
    reverse: bool,
) {
    // Warm the Rayon pool; not part of the measurement.
    let mut warm = convoy(anchor, base_velocity, 8);
    propagate(field, ephemeris, &mut warm, Path::Framed, WARMUP_TICKS);

    for &count in sizes {
        if only.is_some_and(|want| want != count) {
            continue;
        }
        let mut results = Vec::new();
        let mut paths = [
            Path::Direct,
            Path::Framed,
            Path::Cohort {
                budget_mps2: 1.0e-9,
            },
        ];
        // Order-effect bound: reversed runs must reproduce the same ratios
        // within noise; path order is fixed otherwise for comparability.
        if reverse {
            paths.reverse();
        }
        for path in paths {
            // Same-path warmup first: frequency/caches/allocator otherwise
            // bias whichever path runs first by up to 2x on these ticks.
            let mut warmup = convoy(anchor, base_velocity, count);
            propagate(field, ephemeris, &mut warmup, path, WARMUP_TICKS);
            let mut fleet = convoy(anchor, base_velocity, count);
            let started = Instant::now();
            let stats = propagate(field, ephemeris, &mut fleet, path, TICKS);
            let elapsed = started.elapsed();
            let per_tick = elapsed / TICKS as u32;
            results.push((path, elapsed, per_tick, fleet, stats));
        }
        let find = |want: Path| {
            results
                .iter()
                .find(|(path, _, _, _, _)| *path == want)
                .expect("path measured")
        };
        let (_, elapsed_a, _, fleet_a, _) = find(Path::Direct);
        let (_, elapsed_b, per_tick_b, fleet_b, _) = find(Path::Framed);
        let (_, elapsed_c, per_tick_c, fleet_c, stats_c) = find(Path::Cohort {
            budget_mps2: 1.0e-9,
        });
        assert_eq!(
            fleet_a.positions, fleet_b.positions,
            "framed propagation must match direct bitwise (x{count})"
        );
        assert_eq!(
            fleet_a.velocities, fleet_b.velocities,
            "framed velocities must match direct bitwise (x{count})"
        );
        // Cohort divergence vs exact in decision-relevant absolute units
        // (doc 23 section 17): metres, m/s over the full 40k-tick run.
        let mut max_dx = 0.0_f64;
        let mut max_dv = 0.0_f64;
        for index in 0..count {
            max_dx = max_dx.max((fleet_c.positions[index] - fleet_a.positions[index]).length());
            max_dv = max_dv.max((fleet_c.velocities[index] - fleet_a.velocities[index]).length());
        }
        let simulated_s = TICKS as f64 * STEP_S;
        println!(
            "{name} x{count} x{TICKS} ticks: direct {elapsed_a:?} | framed {elapsed_b:?} ({per_tick_b:?}/tick) | cohort {elapsed_c:?} ({per_tick_c:?}/tick, x{:.2} vs direct, {:.1} sim-s/wall-s) | cohorts/tick {:.1}, splits {}, fallbacks {}, reused {:.1}%, posted bound {:e} m/s^2, exact/target {:.1}/{}, max radius {:.1} km | divergence: {max_dx:.4} m, {max_dv:.6} m/s",
            elapsed_a.as_secs_f64() / elapsed_c.as_secs_f64(),
            simulated_s / elapsed_c.as_secs_f64(),
            stats_c.cohorts as f64 / TICKS as f64,
            stats_c.splits,
            stats_c.fallbacks,
            stats_c.reuses as f64 / TICKS as f64 * 100.0,
            stats_c.error_bound,
            stats_c.exact_terms as f64 / TICKS as f64 / count as f64,
            field.source_count(),
            stats_c.max_radius_m / 1000.0,
        );
    }
}
