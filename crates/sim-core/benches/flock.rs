//! Flock benchmarks per docs/23_GRAVITY_FIELD_COHORTS.md section 16.2.
//!
//! Fleet workloads (not single-vehicle microbenchmarks):
//!  1. gravity flock: 1/16/64/128/300/1000 targets x the full system
//!     ephemeris, two scenarios — compact convoy in deep space and convoy
//!     near Thessa with near sources. Reports ephemeris sampling (one
//!     `EphemerisFrame::evaluate` per tick) separately from per-target
//!     source accumulation, for both the direct path and the frame path
//!     (doc section 19, step 1).
//!  2. 6-DoF flock: N vehicles x 16 panels through the full
//!     `integrate_rigid_body_step` under Rayon — the atmospheric tick cost.
use std::{hint::black_box, time::Instant};

use glam::{DMat3, DQuat, DVec3};
use rayon::prelude::*;
use thessa_sim_core::{
    AeroConfig, AeroGeometry, AeroPanel, AtmosphereConfig, BakedEphemeris, BodyState, CohortConfig,
    EphemerisFrame, FlightStepInput, GravityField, GravitySourceTree, PanelAeroModel,
    RigidBodyProperties, RigidBodyState, SimTime, SystemConfig, evaluate_cohorts,
    integrate_rigid_body_step,
};
/// Compact convoy: tight spatial cluster (100 km box), shared epoch.
fn convoy_positions(center: DVec3, count: usize) -> Vec<DVec3> {
    (0..count)
        .map(|index| {
            let i = index as f64;
            center
                + DVec3::new(
                    (i * 12.9898).sin() * 50_000.0,
                    (i * 78.233).sin() * 50_000.0,
                    (i * 37.719).sin() * 50_000.0,
                )
        })
        .collect()
}

fn bench_gravity_scenario(
    name: &str,
    field: &GravityField,
    ephemeris: &BakedEphemeris,
    center: DVec3,
    time: SimTime,
    counts: &[usize],
) {
    println!("--- gravity {name} ({} sources) ---", field.source_count());
    for &count in counts {
        let positions = convoy_positions(center, count);
        // Warm the Rayon pool once; it is not part of the measurement.
        let _ = field
            .accelerations(black_box(&positions), time)
            .expect("finite warmup accelerations");

        // (a) ephemeris sampling: once per tick regardless of target count.
        let mut frame = EphemerisFrame::new();
        let frame_iters = 20;
        let started = Instant::now();
        for _ in 0..frame_iters {
            let states: &[BodyState] = frame
                .evaluate(black_box(ephemeris), time)
                .expect("finite frame states");
            black_box(states);
        }
        let frame_per_tick = started.elapsed() / frame_iters;

        // (b) direct path: per-target accumulation + per-target lookups.
        let iters = 10;
        let started = Instant::now();
        for _ in 0..iters {
            let out = field
                .accelerations(black_box(&positions), time)
                .expect("finite direct accelerations");
            black_box(out);
        }
        let direct_per_tick = started.elapsed() / iters as u32;

        // (c) frame path: same accumulation over one shared frame.
        let started = Instant::now();
        for _ in 0..iters {
            let states = frame
                .evaluate(ephemeris, time)
                .expect("finite frame states");
            let out = field
                .accelerations_from_frame(black_box(&positions), black_box(states))
                .expect("finite frame accelerations");
            black_box(out);
        }
        let framed_per_tick = started.elapsed() / iters as u32;

        // (d) cohort path: one shared patch per tick (budget 1e-9 m/s^2).
        let started = Instant::now();
        let mut cohorts = 0_usize;
        let mut exact_terms = 0_u64;
        for _ in 0..iters {
            let states = frame
                .evaluate(ephemeris, time)
                .expect("finite frame states");
            let report = evaluate_cohorts(
                black_box(ephemeris),
                black_box(states),
                black_box(&positions),
                CohortConfig {
                    error_budget_mps2: 1.0e-9,
                    ..Default::default()
                },
            )
            .expect("finite cohort accelerations");
            cohorts = report.cohort_count;
            exact_terms = report.exact_terms;
            black_box(report);
        }
        let cohort_per_tick = started.elapsed() / iters as u32;

        println!(
            "{name} x{count}: frame {frame_per_tick:?}/tick | direct {direct_per_tick:?}/tick ({:.1} ns/target) | framed {framed_per_tick:?}/tick ({:.1} ns/target) | cohort {cohort_per_tick:?}/tick ({:.1} ns/target, {cohorts} cohorts, exact {:.1}/22)",
            direct_per_tick.as_secs_f64() * 1e9 / count as f64,
            framed_per_tick.as_secs_f64() * 1e9 / count as f64,
            cohort_per_tick.as_secs_f64() * 1e9 / count as f64,
            exact_terms as f64 / count as f64,
        );

        // (e) tree opening pressure (doc 23 step 5 verdict input): serial
        // hierarchy traversals, nodes visited + exact terms per target.
        let tree = GravitySourceTree::build(ephemeris).expect("source tree");
        for budget in [1.0e-9, 1.0e-12] {
            let states = frame.evaluate(ephemeris, time).expect("tree frame states");
            let frames = tree.resolve(ephemeris, states).expect("node frames");
            let mut visited = 0_u64;
            let mut exact_terms = 0_u64;
            for position in &positions {
                let eval = tree
                    .evaluate(&frames, states, *position, budget)
                    .expect("tree eval");
                visited += eval.nodes_visited as u64;
                exact_terms += eval.terms_exact as u64;
            }
            println!(
                "{name} x{count} tree budget {budget:e}: nodes/target {:.1}, exact/target {:.1}/{}",
                visited as f64 / count as f64,
                exact_terms as f64 / count as f64,
                tree.node_count(),
            );
        }
    }
}

fn bench_flock_6dof(vehicles: usize, panels: usize, iters: usize) {
    let panel_list: Vec<_> = (0..panels)
        .map(|index| {
            let x = (index as f64 - panels as f64 / 2.0) * 0.4;
            AeroPanel::flat_plate(DVec3::new(x, 0.0, 0.0), 1.5, 1.0).expect("valid panel")
        })
        .collect();
    let geometry = AeroGeometry::new(panel_list).expect("valid geometry");
    let model = PanelAeroModel::new(AeroConfig::default()).expect("valid aero model");
    let atmosphere = AtmosphereConfig::default();
    let properties = RigidBodyProperties::new(1_000.0, DMat3::from_diagonal(DVec3::splat(1_000.0)))
        .expect("valid mass properties");
    let input = FlightStepInput::new(10_000.0, DVec3::new(0.0, 0.0, -9.80665));
    let states: Vec<_> = (0..vehicles)
        .map(|index| {
            RigidBodyState::new(
                DVec3::new(index as f64 * 50.0, 0.0, 10_000.0),
                DVec3::new(350.0 + (index % 64) as f64 * 0.25, 0.0, 12.0),
                DQuat::IDENTITY,
                DVec3::ZERO,
            )
            .expect("valid state")
        })
        .collect();
    let started = Instant::now();
    for _ in 0..iters {
        let next_states: Vec<_> = states
            .par_iter()
            .map(|state| {
                integrate_rigid_body_step(
                    black_box(&model),
                    black_box(&geometry),
                    atmosphere,
                    *state,
                    properties,
                    input,
                    1.0 / 120.0,
                )
                .expect("finite flight step")
                .0
            })
            .collect();
        black_box(next_states);
    }
    let per_tick = started.elapsed() / iters as u32;
    println!(
        "6-DoF flock {vehicles} vehicles x {panels} panels: {per_tick:?} per tick ({:.1} us/vehicle)",
        per_tick.as_secs_f64() * 1e6 / vehicles as f64,
    );
}

/// H0 (doc 23 section 11 open question): kernel-level tail share for 22
/// sources — full 8+8+4+2scalar cascade vs padded-to-24 full lanes vs pure
/// scalar. Timing only; padding *semantics* (None-conditions, -0.0, order)
/// are the separate open question in the doc.
fn bench_simd_tail() {
    const SOURCES: usize = 22;
    const PADDED: usize = 24;
    let (px, py, pz) = (6.9e6, 1.0e5, 0.0);
    let mut cx = vec![0.0; PADDED];
    let mut cy = vec![0.0; PADDED];
    let mut cz = vec![0.0; PADDED];
    let mut mu = vec![0.0; PADDED];
    for i in 0..SOURCES {
        let angle = i as f64 * 2.399963;
        let radius = 1.0e8 + i as f64 * 1.0e7;
        cx[i] = radius * angle.cos();
        cy[i] = radius * angle.sin();
        cz[i] = i as f64 * 1.0e6;
        mu[i] = 4.0e13 + i as f64 * 1.0e12;
    }
    // Pad lanes: zero mu, finite far-away centers (timing only).
    for i in SOURCES..PADDED {
        cx[i] = 1.0e13;
        cy[i] = -1.0e13;
        cz[i] = 1.0e13;
        mu[i] = 0.0;
    }
    let iters = 200_000;
    fn time_it(iters: u32, mut eval: impl FnMut() -> (f64, f64, f64)) -> std::time::Duration {
        let started = Instant::now();
        for _ in 0..iters {
            black_box(eval());
        }
        started.elapsed() / iters
    }
    let scalar_term = |index: usize, total: (f64, f64, f64)| {
        let dx = cx[index] - px;
        let dy = cy[index] - py;
        let dz = cz[index] - pz;
        let d2 = dx * dx + dy * dy + dz * dz;
        let inverse = d2.sqrt().recip();
        let t = mu[index] * inverse.powi(3);
        (total.0 + dx * t, total.1 + dy * t, total.2 + dz * t)
    };
    let cascade = time_it(iters, || {
        let mut total = (0.0, 0.0, 0.0);
        assert!(thessa_simd::gravity_chunk(
            &cx, &cy, &cz, &mu, 0, px, py, pz, &mut total
        ));
        assert!(thessa_simd::gravity_chunk(
            &cx, &cy, &cz, &mu, 8, px, py, pz, &mut total
        ));
        assert!(thessa_simd::gravity_quad(
            &cx, &cy, &cz, &mu, 16, px, py, pz, &mut total
        ));
        let mut t = total;
        for index in 20..SOURCES {
            t = scalar_term(index, t);
        }
        t
    });
    let padded = time_it(iters, || {
        let mut total = (0.0, 0.0, 0.0);
        assert!(thessa_simd::gravity_chunk(
            &cx, &cy, &cz, &mu, 0, px, py, pz, &mut total
        ));
        assert!(thessa_simd::gravity_chunk(
            &cx, &cy, &cz, &mu, 8, px, py, pz, &mut total
        ));
        assert!(thessa_simd::gravity_chunk(
            &cx, &cy, &cz, &mu, 16, px, py, pz, &mut total
        ));
        total
    });
    let scalar = time_it(iters, || {
        let mut t = (0.0, 0.0, 0.0);
        for index in 0..SOURCES {
            t = scalar_term(index, t);
        }
        t
    });
    println!(
        "simd tail 22 sources: cascade 8+8+4+2scalar {cascade:?}/eval | padded 3x8 {padded:?}/eval | scalar {scalar:?}/eval",
    );
}

/// Micro: sequential cost of one patch evaluation (affine + exact-near) vs
/// one full 22-term framed accumulation — isolates per-target math from
/// Rayon dispatch / allocation overhead of the batch paths.
fn bench_patch_micro(
    ephemeris: &BakedEphemeris,
    states: &[BodyState],
    field: &GravityField,
    time: SimTime,
) {
    use thessa_sim_core::{compile_patch, evaluate_patch};
    let center = DVec3::new(1.0e9, 3.0e8, 0.0);
    let positions: Vec<_> = (0..256)
        .map(|index| center + DVec3::new(index as f64 * 200.0, 0.0, -(index as f64) * 100.0))
        .collect();
    let patch = compile_patch(
        ephemeris,
        states,
        &positions,
        CohortConfig {
            error_budget_mps2: 1.0e-9,
            ..Default::default()
        },
    )
    .expect("micro patch compiles");
    println!(
        "micro patch: radius {:.1} km, exact-near {}, bound {:e} m/s^2",
        patch.radius_m / 1000.0,
        patch.exact.len(),
        patch.error_bound_mps2,
    );
    let iters: u32 = 200_000;
    let started = Instant::now();
    for iteration in 0..iters {
        let position = positions[iteration as usize % positions.len()];
        black_box(evaluate_patch(&patch, states, black_box(position)).expect("patch eval"));
    }
    println!(
        "micro sequential evaluate_patch: {:?}/eval",
        started.elapsed() / iters
    );
    let started = Instant::now();
    for iteration in 0..iters {
        let position = positions[iteration as usize % positions.len()];
        black_box(
            field
                .acceleration(black_box(position), time)
                .expect("exact eval"),
        );
    }
    println!(
        "micro sequential full exact (22 src): {:?}/eval",
        started.elapsed() / iters
    );
}

fn main() {
    let config: SystemConfig = toml::from_str(include_str!("../../../data/system.toml")).unwrap();
    let ephemeris = config.bake().unwrap();
    let field = GravityField::from_ephemeris(&ephemeris);
    let time = SimTime::EPOCH;
    let counts = [1, 16, 64, 128, 300, 1_000];

    // Scenario 1 (doc 16.2): compact convoy in deep space — 1e9 m above the
    // home body along +Z, far from every near source.
    let home = ephemeris
        .body_state(ephemeris.body_id("thessa").unwrap(), time)
        .unwrap();
    bench_gravity_scenario(
        "deep-space convoy",
        &field,
        &ephemeris,
        home.position_inertial + DVec3::Z * 1e9,
        time,
        &counts,
    );

    // Scenario 2 (doc 16.2): convoy near Thessa with 1-3 exact-near sources —
    // low orbit band where the near field dominates.
    let low_orbit = home.position_inertial + DVec3::new(6.9e6, 0.0, 0.0);
    bench_gravity_scenario(
        "near-body convoy",
        &field,
        &ephemeris,
        low_orbit,
        time,
        &counts,
    );

    for vehicles in [256, 1_000] {
        bench_flock_6dof(vehicles, 16, 10);
    }

    bench_simd_tail();
    let mut micro_frame = EphemerisFrame::new();
    let micro_states = micro_frame
        .evaluate(&ephemeris, time)
        .expect("micro frame states")
        .to_vec();
    bench_patch_micro(&ephemeris, &micro_states, &field, time);
}
