//! Phase D residency benchmark (doc §9): insert/access throughput,
//! eviction workload hit rates under a byte budget, and dirty-update cost
//! versus full re-upload.
//!
//! Run with `cargo bench -p thessa-microstore-core --bench residency`.

use std::{hint::black_box, time::Instant};

use thessa_microstore_core::{EncodeMode, EncodedPage, ResidencyCache, fixtures};

const MODE: EncodeMode = EncodeMode::Adaptive { max_abs_error: 2.0 };
const ITERS: usize = 200;

fn main() {
    // 64 distinct 64x64 pages (~4-19 KB each adaptive).
    let pages: Vec<EncodedPage> = (0..64u64)
        .map(|s| EncodedPage::encode(&fixtures::noise(64, 64, s + 1), MODE))
        .collect();
    let page_bytes: usize = pages.iter().map(EncodedPage::encoded_bytes).sum();
    println!("residency: 64 pages, {page_bytes} B total adaptive");

    // Insert throughput into an unbounded cache.
    let started = Instant::now();
    for _ in 0..ITERS {
        let mut cache = ResidencyCache::new(u64::MAX);
        for (i, page) in pages.iter().enumerate() {
            cache.insert(black_box(i as u64), black_box(page.clone()));
        }
        black_box(cache);
    }
    println!(
        "insert 64 pages: {:.3} ms/iter",
        started.elapsed().as_secs_f64() * 1000.0 / ITERS as f64
    );

    // Cyclic access over all pages with room for ~1/4: steady-state hit
    // rate of a cyclic scan larger than the cache.
    let budget = (page_bytes / 4) as u64;
    let mut cache = ResidencyCache::new(budget);
    for (i, page) in pages.iter().enumerate() {
        cache.insert(i as u64, page.clone());
    }
    let started = Instant::now();
    let rounds = 20;
    for _ in 0..rounds {
        for i in 0..64u64 {
            black_box(cache.get(black_box(i)));
        }
    }
    let t = cache.telemetry();
    println!(
        "cyclic scan x{rounds} over budget/4: hit_rate={:.3}, evictions={}, resident={} B / budget {budget} B, texels/MiB={:.0}",
        t.hit_rate().unwrap_or(-1.0),
        t.evictions,
        t.resident_bytes,
        t.texels_per_mib(),
    );
    println!(
        "lookup throughput: {:.1} ns/get",
        started.elapsed().as_secs_f64() * 1e9 / (rounds * 64) as f64
    );

    // Working-set access (16 hot pages in a 64-page cache budget): hits
    // should dominate after warmup.
    let mut cache = ResidencyCache::new(u64::MAX);
    for (i, page) in pages.iter().enumerate() {
        cache.insert(i as u64, page.clone());
    }
    for _ in 0..5 {
        for i in 0..16u64 {
            cache.get(i);
        }
    }
    let t = cache.telemetry();
    println!(
        "hot-16 working set: hit_rate={:.3} (expect ~1.0)",
        t.hit_rate().unwrap_or(-1.0)
    );

    // Dirty update cost: one-texel change vs full page re-upload bytes.
    let mut cache = ResidencyCache::new(u64::MAX);
    cache.insert(0, pages[0].clone());
    cache.mark_clean(0).expect("resident");
    let mut data = fixtures::noise(64, 64, 1).data;
    data[1000] = data[1000].wrapping_add(60);
    let field = thessa_microstore_core::ScalarField::new(64, 64, data).expect("extent");
    let started = Instant::now();
    for _ in 0..ITERS {
        let n = cache
            .update(0, black_box(&field), black_box(MODE))
            .expect("resident");
        black_box(n);
        let ranges = cache.flush(0).expect("resident");
        black_box(ranges);
        cache.mark_clean(0).expect("resident");
        // Restore the pristine page for the next iteration.
        cache.insert(0, black_box(pages[0].clone()));
        cache.mark_clean(0).expect("resident");
    }
    let full = pages[0].encoded_bytes();
    let dirty = {
        cache.insert(0, pages[0].clone());
        cache.mark_clean(0).expect("resident");
        cache.update(0, &field, MODE).expect("resident");
        match cache.flush(0).expect("resident") {
            thessa_microstore_core::FlushPayload::Incremental { patches } => {
                patches.iter().map(|r| r.bytes.len()).sum()
            }
            thessa_microstore_core::FlushPayload::Full { page_bytes, table } => {
                page_bytes.len() + table.len() * 4
            }
        }
    };
    println!(
        "dirty update: {:.3} ms/iter, dirty {dirty} B vs full re-upload {full} B ({:.2}x)",
        started.elapsed().as_secs_f64() * 1000.0 / ITERS as f64,
        full as f64 / dirty.max(1) as f64,
    );

    // Rung change (lossless insert, lossy update): layout shifts, so the
    // flush must fall back to a full page + table reupload.
    let mut cache = ResidencyCache::new(u64::MAX);
    let raw = thessa_microstore_core::EncodedPage::encode(
        &field,
        thessa_microstore_core::EncodeMode::Residual8,
    );
    cache.insert(0, raw);
    cache.mark_clean(0).expect("resident");
    cache.update(0, &field, MODE).expect("resident");
    match cache.flush(0).expect("resident") {
        thessa_microstore_core::FlushPayload::Full { page_bytes, table } => {
            println!(
                "rung-change update: Full fallback {} B page + {} B table",
                page_bytes.len(),
                table.len() * 4,
            );
        }
        thessa_microstore_core::FlushPayload::Incremental { .. } => {
            println!("rung-change update: ERROR, expected Full fallback");
        }
    }
}
