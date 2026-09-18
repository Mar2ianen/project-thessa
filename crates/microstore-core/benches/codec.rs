//! Phase A codec benchmark matrix (doc §16):
//! fixtures x (raw baseline, fixed Residual8, fixed Residual4,
//! adaptive 4/8) with throughput, residency, and error columns.
//!
//! Run with `cargo bench -p thessa-microstore-core --bench codec`.

use std::{hint::black_box, time::Instant};

use thessa_microstore_core::{EncodeMode, EncodedPage, fixtures, measure};

const SIZE: u32 = 128;
const ADAPTIVE_BUDGET: f64 = 2.0;
const ITERS: usize = 300;

fn bench_mode(name: &str, mode: EncodeMode) {
    println!("mode {name}:");
    println!(
        "{:>10} {:>10} {:>10} {:>10} {:>10} {:>10} {:>10}",
        "fixture", "ms/enc", "ms/dec", "MiB/s", "B/tex", "maxerr", "rms"
    );
    for (fixture_name, field) in fixtures::all(SIZE, SIZE) {
        // Warm up and verify once.
        let page = EncodedPage::encode(&field, mode);
        let decoded = page.decode();
        let stats = measure(&field, &decoded);
        let bytes = page.encoded_bytes();

        let started = Instant::now();
        for _ in 0..ITERS {
            let page = EncodedPage::encode(black_box(&field), mode);
            black_box(page);
        }
        let enc_ms = started.elapsed().as_secs_f64() * 1000.0 / ITERS as f64;

        let page = EncodedPage::encode(&field, mode);
        let started = Instant::now();
        for _ in 0..ITERS {
            let decoded = black_box(&page).decode();
            black_box(decoded);
        }
        let dec_ms = started.elapsed().as_secs_f64() * 1000.0 / ITERS as f64;

        let texels = (SIZE as f64) * (SIZE as f64);
        // Combined codec throughput in mebi-texels per second.
        let mib_s = texels * 2.0 / ((enc_ms + dec_ms) / 1000.0) / 1_048_576.0;
        println!(
            "{:>10} {:>10.3} {:>10.3} {:>10.1} {:>10.3} {:>10.1} {:>10.3}",
            fixture_name,
            enc_ms,
            dec_ms,
            mib_s,
            bytes as f64 / texels,
            stats.max_abs,
            stats.rms,
        );
    }
}

fn main() {
    println!("microstore Phase A: {SIZE}x{SIZE} scalar field, {ITERS} iters");
    bench_mode("raw8", EncodeMode::Raw8);
    bench_mode("residual8", EncodeMode::Residual8);
    bench_mode("residual4", EncodeMode::Residual4);
    bench_mode(
        "adaptive2.0",
        EncodeMode::Adaptive {
            max_abs_error: ADAPTIVE_BUDGET,
        },
    );
}
