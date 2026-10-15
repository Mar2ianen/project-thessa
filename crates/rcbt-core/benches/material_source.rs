//! Source-lifecycle microbenchmark, not terrain rendering or whole-client FPS.
use std::{hint::black_box, time::Instant};
use thessa_rcbt_core::{material_page::CbtMaterialPage, material_source::MaterialPageStore};

fn measure(label: &str, count: usize, mut operation: impl FnMut()) {
    let mut samples = Vec::with_capacity(count);
    for _ in 0..count {
        let start = Instant::now();
        operation();
        samples.push(start.elapsed().as_nanos());
    }
    samples.sort_unstable();
    println!(
        "{label}: n={count} median_ns={} p95_ns={}",
        samples[count / 2],
        samples[count * 95 / 100]
    );
}

fn main() {
    let page = CbtMaterialPage::from_rgba8(vec![64; 128 * 128 * 4]).unwrap();
    let mut source = MaterialPageStore::default();
    source.set_pages((32768..33792).map(|id| {
        (
            id,
            CbtMaterialPage::from_rgba8(vec![(id % 256) as u8; 128 * 128 * 4]).unwrap(),
        )
    }));
    source.set_priority((32768..33280).collect());
    measure(
        "1024 sources / 512 priorities: shared snapshot",
        10000,
        || {
            black_box(source.clone());
        },
    );
    measure("1024 sources: clean budget check", 10000, || {
        source.trim_to_budget(1024)
    });
    measure(
        "1024 sources: four completed pages + retained snapshot COW",
        1000,
        || {
            let snapshot = source.clone();
            source.set_pages((32768..32772).map(|id| (id, page.clone())));
            black_box(snapshot);
        },
    );
    measure(
        "1024 sources: evict/reinsert four pages + retained snapshot",
        1000,
        || {
            let snapshot = source.clone();
            source.remove_pages(33788..33792);
            source.set_pages((33788..33792).map(|id| (id, page.clone())));
            black_box(snapshot);
        },
    );
}
