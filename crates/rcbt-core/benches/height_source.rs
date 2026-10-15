//! Height snapshot/publication metadata, not geometry generation or game FPS.
use std::{hint::black_box, time::Instant};
use thessa_rcbt_core::{HeightPage, height_source::HeightPageStore};

fn measure<T>(label: &str, n: usize, mut prepare: impl FnMut() -> T, mut operation: impl FnMut(T)) {
    let mut times = Vec::with_capacity(n);
    for _ in 0..n {
        let input = prepare();
        let start = Instant::now();
        operation(input);
        times.push(start.elapsed().as_nanos());
    }
    times.sort_unstable();
    println!(
        "{label}: n={n} median_ns={} p95_ns={}",
        times[n / 2],
        times[n * 95 / 100]
    );
}

fn main() {
    let a = HeightPage::bake(&[10.0; 33 * 33], 33, 0.5).unwrap();
    let b = HeightPage::bake(&[20.0; 33 * 33], 33, 0.5).unwrap();
    let mut source = HeightPageStore::default();
    source.set_pages((32768..33792).map(|id| (id, a.clone())));
    measure(
        "1024 33x33 pages: shared snapshot",
        10000,
        || (),
        |()| {
            black_box(source.clone());
        },
    );
    measure(
        "1024 pages: eight equal completions",
        1000,
        || (32768..32776).map(|id| (id, a.clone())).collect::<Vec<_>>(),
        |pages| {
            source.set_pages(pages);
        },
    );
    let mut alternate = false;
    measure(
        "1024 pages: eight dirty completions with retained snapshot",
        1000,
        || {
            alternate = !alternate;
            let page = if alternate { &b } else { &a };
            (32768..32776)
                .map(|id| (id, page.clone()))
                .collect::<Vec<_>>()
        },
        |pages| {
            let old = source.clone();
            source.set_pages(pages);
            black_box(old);
        },
    );
    measure(
        "1024 pages: eight evictions/reinsertions with retained snapshot",
        1000,
        || (33784..33792).map(|id| (id, a.clone())).collect::<Vec<_>>(),
        |pages| {
            let old = source.clone();
            source.remove_pages(33784..33792);
            source.set_pages(pages);
            black_box(old);
        },
    );
}
