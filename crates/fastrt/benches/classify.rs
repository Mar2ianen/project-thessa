//! FastRT Stage 1 policy throughput: `classify_frame` over synthetic fleets.
//! Run with `cargo bench -p thessa-fastrt --bench classify`.
//! Pins ns/candidate so a policy change shows its linear-scan price.

use std::{hint::black_box, time::Instant};

use thessa_fastrt::{Bounds3, Candidate, Policy, classify_frame};

fn lcg(state: &mut u64) -> f64 {
    *state = state
        .wrapping_mul(6364136223846793005)
        .wrapping_add(1442695040888963407);
    ((*state >> 11) as f64) / ((1u64 << 53) as f64)
}

fn fleet(n: usize, seed: u64) -> Vec<Candidate> {
    let mut rng = seed;
    (0..n as u64)
        .map(|key| {
            let c = [
                lcg(&mut rng) * 2000.0 - 1000.0,
                lcg(&mut rng) * 200.0,
                lcg(&mut rng) * 2000.0 - 1000.0,
            ];
            let e = 1.0 + lcg(&mut rng) * 50.0;
            let min = [c[0] - e, c[1] - e, c[2] - e];
            let max = [c[0] + e, c[1] + e, c[2] + e];
            Candidate {
                key,
                bounds: Bounds3::new(min, max).unwrap(),
                visible: lcg(&mut rng) > 0.05,
                rt_effects: (lcg(&mut rng) * 8.0) as u32,
                full_tris: (lcg(&mut rng) * 200_000.0) as u64,
                proxy_tris: (lcg(&mut rng) * 2_000.0) as u64,
            }
        })
        .collect()
}

fn main() {
    let policy = Policy::new(20_000);
    for (n, iters) in [(10_000usize, 50usize), (100_000, 8)] {
        let fleet = fleet(n, 0xC1A551F7);
        // Warm up (also shakes out first-sort allocator paths).
        black_box(classify_frame(black_box(&fleet), &policy));
        let started = Instant::now();
        let mut classes = 0usize;
        for _ in 0..iters {
            classes += classify_frame(black_box(&fleet), &policy).len();
        }
        let ns_per_candidate = started.elapsed().as_nanos() as f64 / (iters as f64 * n as f64);
        println!("classify_frame n={n}: {ns_per_candidate:.1} ns/candidate ({classes} classified)");
    }
}
