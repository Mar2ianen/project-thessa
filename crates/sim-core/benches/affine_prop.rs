//! Benchmarks A/B from the analytic-propagation note, section 15.
//!
//! A: frozen affine field — analytic STM vs Velocity Verlet vs RK4 wall
//! time for 1/100/1k/10k candidates, with the STM as the exact oracle for
//! the frozen linear field.
//!
//! B: real deep-space patch, no exact-near sources — analytic frozen-patch
//! segments vs exact gravity integration over increasing segment lengths,
//! with divergence checked against the posted propagation bound.
use std::{hint::black_box, time::Instant};

use glam::{DMat3, DVec3};
use thessa_sim_core::{
    AffinePropagator, CohortConfig, EphemerisFrame, GravityField, SimTime, SystemConfig,
    affine_segment_bound, compile_patch,
};

fn frozen_candidates(count: usize) -> (Vec<DVec3>, Vec<DVec3>) {
    let positions: Vec<_> = (0..count)
        .map(|index| {
            let i = index as f64;
            DVec3::new(
                1.0e5 + (i * 12.9898).sin() * 2.0e4,
                (i * 78.233).sin() * 2.0e4,
                (i * 37.719).sin() * 2.0e4,
            )
        })
        .collect();
    let velocities: Vec<_> = (0..count)
        .map(|index| {
            let i = index as f64;
            DVec3::new(
                500.0 + (i * 3.17).sin() * 20.0,
                (i * 5.71).sin() * 20.0,
                10.0 + (i * 9.13).sin() * 5.0,
            )
        })
        .collect();
    (positions, velocities)
}

fn bench_a() {
    // Mixed-sign indefinite tensor at orbital magnitude, plus constant.
    let jacobian = DMat3::from_cols(
        DVec3::new(2.0e-6, 0.5e-6, -0.3e-6),
        DVec3::new(0.5e-6, -1.0e-6, 0.2e-6),
        DVec3::new(-0.3e-6, 0.2e-6, -1.0e-6),
    );
    let constant = DVec3::new(0.11, -0.07, 0.05);
    let propagator = AffinePropagator::compile(jacobian).expect("propagator");
    let duration = 120.0;
    let coeffs = propagator.coefficients(duration).expect("coeffs");
    let accel = |position: DVec3| jacobian * position + constant;
    for count in [1, 100, 1_000, 10_000] {
        let (positions, velocities) = frozen_candidates(count);
        // STM (oracle).
        let started = Instant::now();
        for _ in 0..20 {
            for (position, velocity) in positions.iter().zip(&velocities) {
                black_box(propagator.propagate(
                    black_box(&coeffs),
                    black_box(*position),
                    black_box(*velocity),
                    black_box(constant),
                ));
            }
        }
        let stm = started.elapsed() / 20;
        // Velocity Verlet, 500 microsteps.
        let steps = 500;
        let step = duration / steps as f64;
        let started = Instant::now();
        for _ in 0..20 {
            for (position, velocity) in positions.iter().zip(&velocities) {
                let (mut x, mut v) = (*position, *velocity);
                for _ in 0..steps {
                    v += accel(x) * (step * 0.5);
                    x += v * step;
                    v += accel(x) * (step * 0.5);
                }
                black_box((x, v));
            }
        }
        let verlet = started.elapsed() / 20;
        // RK4, 100 microsteps.
        let steps = 100;
        let step = duration / steps as f64;
        let started = Instant::now();
        for _ in 0..20 {
            for (position, velocity) in positions.iter().zip(&velocities) {
                let (mut x, mut v) = (*position, *velocity);
                for _ in 0..steps {
                    let a1 = accel(x);
                    let k1x = v;
                    let a2 = accel(x + k1x * (step * 0.5));
                    let k2x = v + a1 * (step * 0.5);
                    let a3 = accel(x + k2x * (step * 0.5));
                    let k3x = v + a2 * (step * 0.5);
                    let a4 = accel(x + k3x * step);
                    let k4x = v + a3 * step;
                    x += (k1x + k2x * 2.0 + k3x * 2.0 + k4x) * (step / 6.0);
                    v += (a1 + a2 * 2.0 + a3 * 2.0 + a4) * (step / 6.0);
                }
                black_box((x, v));
            }
        }
        let rk4 = started.elapsed() / 20;
        println!(
            "affine oracle x{count}: STM {stm:?} ({:.1} ns/cand) | Verlet500 {verlet:?} | RK4-100 {rk4:?}",
            stm.as_secs_f64() * 1e9 / count as f64,
        );
    }
}

fn bench_b() {
    let config: SystemConfig = toml::from_str(include_str!("../../../data/system.toml")).unwrap();
    let ephemeris = config.bake().unwrap();
    let field = GravityField::from_ephemeris(&ephemeris);
    let mut frame = EphemerisFrame::new();
    let states = frame
        .evaluate(&ephemeris, SimTime::EPOCH)
        .expect("frame")
        .to_vec();
    let home = ephemeris
        .body_state(ephemeris.body_id("thessa").unwrap(), SimTime::EPOCH)
        .unwrap();
    let center = home.position_inertial + DVec3::Z * 1e9;
    let positions = vec![center, center + DVec3::new(10_000.0, 0.0, 0.0)];
    let patch = compile_patch(&ephemeris, &states, &positions, CohortConfig::default())
        .expect("patch compiles");
    assert!(patch.exact.is_empty());
    let propagator = AffinePropagator::compile(patch.jacobian).expect("propagator");
    let constant = patch.g0 - patch.jacobian * center;
    let velocity = DVec3::new(0.0, 5_000.0, 100.0);
    for duration in [60.0, 600.0, 3_600.0] {
        let coeffs = propagator.coefficients(duration).expect("coeffs");
        let started = Instant::now();
        let (analytic, _) = propagator.propagate(&coeffs, center, velocity, constant);
        let analytic_elapsed = started.elapsed();
        let mut excursion = 0.0_f64;
        for quarter in 1..=4 {
            let sub = propagator
                .coefficients(duration * quarter as f64 / 4.0)
                .expect("sub coeffs");
            let (point, _) = propagator.propagate(&sub, center, velocity, constant);
            excursion = excursion.max((point - center).length());
        }
        let field_bound = affine_segment_bound(&ephemeris, &states, &patch, excursion, duration)
            .expect("segment bound");
        // Exact reference: RK4, 0.5 s steps, per-stage frames.
        let started = Instant::now();
        let mut x = center;
        let mut v = velocity;
        let steps = (duration / 0.5) as usize;
        for step in 0..steps {
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
        let exact_elapsed = started.elapsed();
        let divergence = (analytic - x).length();
        let bound_m = field_bound * duration * duration / 2.0;
        println!(
            "real patch segment {duration}s: analytic {analytic_elapsed:?} vs exact-RK4 {exact_elapsed:?} (x{:.0}) | excursion {:.1} km | divergence {divergence:e} m vs posted {bound_m:e} m (margin x{:.0})",
            exact_elapsed.as_secs_f64() / analytic_elapsed.as_secs_f64().max(1e-12),
            excursion / 1000.0,
            bound_m / divergence.max(1e-300),
        );
        assert!(
            divergence <= bound_m,
            "segment divergence escapes posted bound"
        );
    }
}

fn main() {
    bench_a();
    bench_b();
}
