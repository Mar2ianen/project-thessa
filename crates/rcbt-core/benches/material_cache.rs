//! CPU slot-assignment baseline, not GPU uploads or renderer FPS.
//! Request construction, cache initialization and validation are outside the
//! timed region; allocations performed by update_retaining are included.
use std::{hint::black_box, time::Instant};
use thessa_rcbt_core::material_cache::SlotCache;

fn measure(capacity: usize, scenario: &str, samples: usize) {
    let resident: Vec<_> = (1..=capacity as u64).map(|id| (id, 1)).collect();
    let replacements = (capacity / 16).max(1);
    let mut alternate = resident.clone();
    for (index, page) in alternate[capacity - replacements..].iter_mut().enumerate() {
        page.0 = capacity as u64 + index as u64 + 1;
    }
    let empty = SlotCache::new(capacity).unwrap();
    let mut cache = empty.clone();
    cache.update_retaining(&resident);
    // Warm the instruction/data path; not reported as samples.
    for _ in 0..4 {
        black_box(cache.update_retaining(&resident));
    }
    let mut times = Vec::with_capacity(samples);
    for index in 0..samples {
        if scenario == "cold" {
            cache = empty.clone();
        }
        let request = if scenario == "churn_1_in_16" && index % 2 == 0 {
            &alternate
        } else {
            &resident
        };
        let start = Instant::now();
        let changes = black_box(cache.update_retaining(black_box(request)));
        times.push(start.elapsed().as_nanos());
        let expected = match scenario {
            "cold" => capacity,
            "churn_1_in_16" => replacements,
            _ => 0,
        };
        assert_eq!(changes.len(), expected);
        // Pin real residency, not just a fast no-op returning the right count.
        for &(id, generation) in request {
            let slot = cache.slot(id).expect("requested page resident");
            assert_eq!(cache.entry(slot).unwrap().generation, generation);
        }
        assert_eq!(cache.len(), capacity);
    }
    times.sort_unstable();
    let percentile = |p: usize| times[(samples * p).div_ceil(100).saturating_sub(1)];
    println!(
        "{capacity},{scenario},{samples},{},{},{},{}",
        percentile(50),
        percentile(95),
        percentile(99),
        times[samples - 1]
    );
}

fn main() {
    let samples = std::env::var("THESSA_CACHE_BENCH_SAMPLES")
        .map(|n| n.parse::<usize>().expect("positive sample count"))
        .unwrap_or(31);
    assert!(samples > 0);
    println!("capacity,scenario,samples,median_ns,p95_ns,p99_ns,max_ns");
    for capacity in [64, 256, 1024, 4096] {
        for scenario in ["warm", "cold", "churn_1_in_16"] {
            measure(capacity, scenario, samples);
        }
    }
}
