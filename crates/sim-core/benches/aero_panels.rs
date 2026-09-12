//! Per-panel evaluation cost: AoS loop vs SoA oracle vs SIMD fast path.
//! Run with `cargo bench -p thessa-sim-core --bench aero_panels`.
use std::{hint::black_box, time::Instant};
use thessa_sim_core::*;

fn main() {
    let starter = X15StarterProfile::new().unwrap();
    let model = PanelAeroModel::new(starter.aero_config).unwrap();
    let geometry = starter.vehicle.aero_geometry;
    let soa = PanelSoA::from_geometry(&geometry).unwrap();
    eprintln!("panels: {}", soa.count);
    let state = AeroState::new(
        glam::DVec3::new(200.0, 0.0, -40.0),
        glam::DVec3::new(0.01, 0.02, 0.0),
    );
    let env = AtmosphereConfig::default()
        .aero_environment(3000.0f64.max(0.0), glam::DVec3::ZERO)
        .unwrap();
    let n = 20_000;
    let started = Instant::now();
    for _ in 0..n {
        black_box(model.evaluate_state(state, env, &geometry).unwrap());
    }
    let aos = started.elapsed().as_secs_f64() * 1e6 / n as f64;
    let started = Instant::now();
    for _ in 0..n {
        black_box(model.evaluate_soa_parts(state, env, &soa, false).unwrap());
    }
    let soa_us = started.elapsed().as_secs_f64() * 1e6 / n as f64;
    let started = Instant::now();
    for _ in 0..n {
        black_box(model.evaluate_soa_simd(state, env, &soa, false).unwrap());
    }
    let simd_us = started.elapsed().as_secs_f64() * 1e6 / n as f64;
    let mut scratch = AeroSimdScratch::default();
    let started = Instant::now();
    for _ in 0..n {
        black_box(
            model
                .evaluate_soa_simd_scratch(state, env, &soa, false, &mut scratch)
                .unwrap(),
        );
    }
    let reuse_us = started.elapsed().as_secs_f64() * 1e6 / n as f64;
    println!(
        "AoS loop: {aos:.2} us/eval, SoA oracle: {soa_us:.2} us/eval, SoA SIMD: {simd_us:.2} us/eval, SIMD reuse: {reuse_us:.2} us/eval"
    );

    // Synthetic 16-panel wing set: engages two 8-wide kernels per eval.
    let mut panels = Vec::new();
    for i in 0..16 {
        let fi = i as f64;
        panels.push(
            AeroPanel::flat_plate(
                glam::DVec3::new(-4.0 + (i % 4) as f64, 2.0 - 0.5 * fi, 0.0),
                1.0 + fi,
                0.8 + 0.1 * (fi % 3.0),
            )
            .unwrap()
            .with_planform(
                2.0 + 0.4 * fi,
                1.5 + 0.5 * (fi % 5.0),
                0.1 * (fi % 4.0),
                1.0,
            )
            .unwrap(),
        );
    }
    let big = AeroGeometry::new(panels).unwrap();
    let big_soa = PanelSoA::from_geometry(&big).unwrap();
    let started = Instant::now();
    for _ in 0..n {
        black_box(model.evaluate_state(state, env, &big).unwrap());
    }
    let big_aos = started.elapsed().as_secs_f64() * 1e6 / n as f64;
    let started = Instant::now();
    for _ in 0..n {
        black_box(
            model
                .evaluate_soa_parts(state, env, &big_soa, false)
                .unwrap(),
        );
    }
    let big_soa_us = started.elapsed().as_secs_f64() * 1e6 / n as f64;
    let started = Instant::now();
    for _ in 0..n {
        black_box(
            model
                .evaluate_soa_simd(state, env, &big_soa, false)
                .unwrap(),
        );
    }
    let big_simd_us = started.elapsed().as_secs_f64() * 1e6 / n as f64;
    let mut big_scratch = AeroSimdScratch::default();
    let started = Instant::now();
    for _ in 0..n {
        black_box(
            model
                .evaluate_soa_simd_scratch(state, env, &big_soa, false, &mut big_scratch)
                .unwrap(),
        );
    }
    let big_reuse_us = started.elapsed().as_secs_f64() * 1e6 / n as f64;
    println!(
        "16 panels — AoS: {big_aos:.2} us/eval, oracle: {big_soa_us:.2} us/eval, SIMD: {big_simd_us:.2} us/eval, SIMD reuse: {big_reuse_us:.2} us/eval"
    );
}
