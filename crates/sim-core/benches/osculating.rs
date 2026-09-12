//! State-to-elements cost for a 256-vehicle map update, no renderer involved.
use std::{hint::black_box, time::Instant};
use thessa_sim_core::{KeplerOrbit, OsculatingElements, SimTime};
fn main() {
    let mu = 4.0e13;
    let states: Vec<_> = (0..256)
        .map(|i| {
            KeplerOrbit::new(
                mu,
                10_000_000.0,
                (i % 9) as f64 * 0.1,
                (i % 8) as f64 * std::f64::consts::PI / 7.0,
                0.8,
                1.2,
                i as f64 * 0.03,
            )
            .unwrap()
            .state_relative_at(SimTime::EPOCH)
            .unwrap()
        })
        .collect();
    let mut times = Vec::with_capacity(2000);
    for iteration in 0..2100 {
        let start = Instant::now();
        for &(r, v) in &states {
            black_box(OsculatingElements::from_state(black_box(r), black_box(v), mu).unwrap());
        }
        if iteration >= 100 {
            times.push(start.elapsed().as_secs_f64() * 1e6);
        }
    }
    times.sort_by(f64::total_cmp);
    println!(
        "256 orbital states: p50 {:.3} us; p95 {:.3} us; p99 {:.3} us",
        times[1000], times[1900], times[1980]
    );
}
