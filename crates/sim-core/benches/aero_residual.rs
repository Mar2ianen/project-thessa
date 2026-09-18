//! Residual aerodynamic coefficient-table prototype benchmark.
//!
//! Measures canonical f64 table sampling against the scalar residual reference
//! path from docs/43. This is intentionally not a SIMD result; it establishes
//! storage density and the decode overhead that AVX2/AVX-512 work must beat.
//!
//! Run with:
//! cargo bench -p thessa-sim-core --bench aero_residual

use std::{hint::black_box, time::Instant};

use thessa_sim_core::{
    AeroCoefficientTable, AeroCoefficients, AeroResidualBudget, AeroResidualTable,
};

fn build_table(mach_count: usize, alpha_count: usize) -> AeroCoefficientTable {
    let mach_grid = (0..mach_count)
        .map(|i| 6.0 * i as f64 / (mach_count - 1) as f64)
        .collect::<Vec<_>>();
    let alpha_grid_rad = (0..alpha_count)
        .map(|i| -1.2 + 2.4 * i as f64 / (alpha_count - 1) as f64)
        .collect::<Vec<_>>();
    let mut samples = Vec::with_capacity(mach_count * alpha_count);
    for &mach in &mach_grid {
        for &alpha in &alpha_grid_rad {
            // Smooth almost everywhere, deliberately sharper around transonic
            // and stall-like bands so adaptive tiles do not get a toy input.
            let stall = ((alpha.abs() - 0.32) / 0.10).clamp(0.0, 1.0);
            let stall = stall * stall * (3.0 - 2.0 * stall);
            let trans = ((mach - 0.78) / 0.40).clamp(0.0, 1.0);
            let trans = trans * trans * (3.0 - 2.0 * trans);
            let attached = 4.8 * alpha / (1.0 + (3.2 * alpha).powi(2)).sqrt();
            let separated = 0.9 * (2.0 * alpha).sin();
            let lift = attached + (separated - attached) * stall;
            let drag = 0.025
                + 0.08 * attached * attached * (1.0 - stall)
                + 1.7 * stall * alpha.sin().powi(2)
                + 0.18 * trans * (0.12 + 1.5 * alpha * alpha);
            let side_force = 0.03 * alpha.sin() * (1.0 + 0.05 * mach);
            let pitching_moment = -0.08 * alpha * (1.0 - 0.6 * stall);
            samples.push(AeroCoefficients {
                lift,
                drag,
                side_force,
                pitching_moment,
            });
        }
    }
    AeroCoefficientTable::new(mach_grid, alpha_grid_rad, samples).unwrap()
}

fn time_samples(
    iters: usize,
    mut sample: impl FnMut(f64, f64) -> AeroCoefficients,
) -> std::time::Duration {
    let started = Instant::now();
    for i in 0..iters {
        let mach = (i.wrapping_mul(17) % 10_000) as f64 * 6.0 / 9_999.0;
        let alpha = -1.2 + (i.wrapping_mul(7919) % 10_000) as f64 * 2.4 / 9_999.0;
        black_box(sample(black_box(mach), black_box(alpha)));
    }
    started.elapsed()
}

fn main() {
    let canonical = build_table(257, 257);
    let encode_started = Instant::now();
    let packed =
        AeroResidualTable::encode(&canonical, AeroResidualBudget::uniform(1.0e-4)).unwrap();
    let encode_time = encode_started.elapsed();
    let stats = packed.stats();

    println!(
        "aero residual 257x257: raw={} KiB logical={} KiB payload={} KiB ratio={:.2}x",
        stats.raw_coefficient_bytes / 1024,
        stats.logical_resident_bytes / 1024,
        stats.payload_bytes / 1024,
        stats.raw_coefficient_bytes as f64 / stats.logical_resident_bytes as f64,
    );
    println!(
        "tiles={} r8={} r16={} raw64={} encode={:?} max_err={:?}",
        stats.tiles,
        stats.residual8_tiles,
        stats.residual16_tiles,
        stats.raw64_tiles,
        encode_time,
        stats.max_error,
    );

    let iters = 1_000_000;
    let canonical_time = time_samples(iters, |mach, alpha| canonical.sample(mach, alpha));
    let packed_time = time_samples(iters, |mach, alpha| packed.sample(mach, alpha));
    println!(
        "sample x{iters}: canonical={:?} ({:.1} ns/sample) residual-scalar={:?} ({:.1} ns/sample, x{:.2})",
        canonical_time,
        canonical_time.as_secs_f64() * 1e9 / iters as f64,
        packed_time,
        packed_time.as_secs_f64() * 1e9 / iters as f64,
        packed_time.as_secs_f64() / canonical_time.as_secs_f64(),
    );
}
