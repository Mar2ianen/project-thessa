//! Slab-march oracle cost: shaped-profile integration at view budgets.
//! Run with `cargo bench -p thessa-atmosphere --bench cloud_slab`.

use std::{hint::black_box, time::Instant};

use thessa_atmosphere::clouds::{CloudSlab, integrate_slab, intersect_shell, vertical_profile};

fn main() {
    for side in [32, 64, 128] {
        let mut times = Vec::new();
        for _ in 0..5 {
            let start = Instant::now();
            let detail = thessa_atmosphere::clouds::bake_density_detail(0x5eed2026, side).unwrap();
            times.push(start.elapsed().as_secs_f64() * 1000.0);
            black_box(detail);
        }
        times.sort_by(f64::total_cmp);
        println!(
            "cloud_density_bake side={side} samples=5 median_ms={:.3}",
            times[2]
        );
    }
    let slab = CloudSlab::new(3_200_000.0, 3_200_000.0 * 1.008, 3_200_000.0 * 1.014).unwrap();
    let origin = glam::DVec3::new(0.0, 0.0, 4_000_000.0);
    let dir = glam::DVec3::new(0.0, 0.0, -1.0);
    let (t0, t1) = intersect_shell(origin, dir, glam::DVec3::ZERO, slab).expect("hits deck");
    let field = |t: f64| vertical_profile((t - t0) / (t1 - t0));
    // Warm up.
    let (tx, _) = integrate_slab(field, t0, t1, 16, 4.0 / slab.thickness_m());
    black_box(tx);

    for steps in [8u32, 16] {
        let iterations = 20000;
        let started = Instant::now();
        let mut checksum = 0.0;
        for _ in 0..iterations {
            let (t, _) = integrate_slab(field, t0, t1, steps, 4.0 / slab.thickness_m());
            checksum += t;
            black_box(t);
        }
        let elapsed = started.elapsed();
        println!(
            "integrate_slab steps={steps} x{iterations}: {:.3} ms total, {:.1} ns/iter, checksum={checksum:.6}",
            elapsed.as_secs_f64() * 1000.0,
            elapsed.as_secs_f64() * 1e9 / iterations as f64,
        );
    }
}
