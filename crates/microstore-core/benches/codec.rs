//! Phase A/C codec benchmark matrix (doc §16):
//! fixtures x (raw baseline, fixed Residual8/6/4/2, adaptive ladder)
//! with throughput, residency, header share, and error columns, plus a
//! three-channel color section and two page extents under comparison.
//!
//! Run with `cargo bench -p thessa-microstore-core --bench codec`.

use std::{hint::black_box, time::Instant};

use thessa_microstore_core::{
    ColorField, ColorPage, EncodeMode, EncodedPage, fixtures, measure, measure_linear,
};

const ADAPTIVE_BUDGET: f64 = 2.0;
const ITERS: usize = 300;

fn bench_mode(size: u32, name: &str, mode: EncodeMode) {
    println!("mode {name} @ {size}x{size}:");
    println!(
        "{:>10} {:>10} {:>10} {:>10} {:>10} {:>10} {:>10} {:>10}",
        "fixture", "ms/enc", "ms/dec", "MiB/s", "B/tex", "hdr%", "maxerr", "rms"
    );
    for (fixture_name, field) in fixtures::all(size, size) {
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

        let texels = (size as f64) * (size as f64);
        // Combined codec throughput in mebi-texels per second.
        let mib_s = texels * 2.0 / ((enc_ms + dec_ms) / 1000.0) / 1_048_576.0;
        println!(
            "{:>10} {:>10.3} {:>10.3} {:>10.1} {:>10.3} {:>10.1} {:>10.1} {:>10.3}",
            fixture_name,
            enc_ms,
            dec_ms,
            mib_s,
            bytes as f64 / texels,
            page.header_fraction() * 100.0,
            stats.max_abs,
            stats.rms,
        );
    }
}

fn bench_color(size: u32) {
    // One RGB page: coast (R), noise (G), gradient (B) planes.
    let field = ColorField::new([
        fixtures::coast(size, size, 0xC0),
        fixtures::noise(size, size, 0x10),
        fixtures::gradient(size, size),
    ])
    .expect("color planes share extent");
    println!("color rgb @ {size}x{size}:");
    println!(
        "{:>10} {:>10} {:>10} {:>10} {:>10} {:>10}",
        "mode", "ms/enc", "B/tex", "lin-max", "lin-rms", "byte-max"
    );
    for (name, mode) in [
        ("raw8", EncodeMode::Raw8),
        (
            "adapt2.0",
            EncodeMode::Adaptive {
                max_abs_error: ADAPTIVE_BUDGET,
            },
        ),
    ] {
        let page = ColorPage::encode(&field, mode);
        let decoded = page.decode();
        let linear = measure_linear(&field.channels, &decoded);
        let worst_byte = field
            .channels
            .iter()
            .zip(decoded.iter())
            .map(|(a, b)| measure(a, b).max_abs)
            .fold(0.0f64, f64::max);

        let started = Instant::now();
        for _ in 0..ITERS {
            let page = ColorPage::encode(black_box(&field), mode);
            black_box(page);
        }
        let enc_ms = started.elapsed().as_secs_f64() * 1000.0 / ITERS as f64;
        println!(
            "{:>10} {:>10.3} {:>10.3} {:>10.4} {:>10.4} {:>10.1}",
            name,
            enc_ms,
            page.bytes_per_texel(),
            linear.max_abs,
            linear.rms,
            worst_byte,
        );
    }

    // Pluggable metric: same field, linear-light budget.
    {
        use thessa_microstore_core::ColorBudget;
        let page = ColorPage::encode_with_budget(&field, ColorBudget::Linear(0.01));
        let decoded = page.decode();
        let linear = measure_linear(&field.channels, &decoded);
        let started = Instant::now();
        for _ in 0..ITERS {
            let page = ColorPage::encode_with_budget(black_box(&field), ColorBudget::Linear(0.01));
            black_box(page);
        }
        let enc_ms = started.elapsed().as_secs_f64() * 1000.0 / ITERS as f64;
        println!(
            "{:>10} {:>10.3} {:>10.3} {:>10.4} {:>10.4} {:>10.1}",
            "lin0.01",
            enc_ms,
            page.bytes_per_texel(),
            linear.max_abs,
            linear.rms,
            field
                .channels
                .iter()
                .zip(decoded.iter())
                .map(|(a, b)| measure(a, b).max_abs)
                .fold(0.0f64, f64::max),
        );
    }
}

fn bench_mips(size: u32) {
    // Full chain cost: build + adaptive encode + total bytes vs base.
    let field = fixtures::noise(size, size, 0x51AB);
    let chain = thessa_microstore_core::MipChain::build(&field, 8);
    let started = Instant::now();
    for _ in 0..ITERS {
        let chain = thessa_microstore_core::MipChain::build(black_box(&field), 8);
        let pages: Vec<_> = chain
            .levels
            .iter()
            .map(|level| {
                EncodedPage::encode(
                    level,
                    EncodeMode::Adaptive {
                        max_abs_error: ADAPTIVE_BUDGET,
                    },
                )
            })
            .collect();
        black_box(pages);
    }
    let ms = started.elapsed().as_secs_f64() * 1000.0 / ITERS as f64;
    let pages: Vec<_> = chain
        .levels
        .iter()
        .map(|level| {
            EncodedPage::encode(
                level,
                EncodeMode::Adaptive {
                    max_abs_error: ADAPTIVE_BUDGET,
                },
            )
        })
        .collect();
    let total: usize = pages.iter().map(EncodedPage::encoded_bytes).sum();
    println!(
        "mips {} levels @ {size}x{size}: build+encode {:.3} ms, chain {total} B vs base {} B ({:.2}x)",
        pages.len(),
        ms,
        pages[0].encoded_bytes(),
        total as f64 / pages[0].encoded_bytes() as f64,
    );
}

fn main() {
    for size in [128, 256] {
        println!("microstore Phase A/C: {size}x{size} scalar field, {ITERS} iters");
        bench_mode(size, "raw8", EncodeMode::Raw8);
        bench_mode(size, "residual8", EncodeMode::Residual8);
        bench_mode(size, "residual6", EncodeMode::Residual6);
        bench_mode(size, "residual4", EncodeMode::Residual4);
        bench_mode(size, "residual2", EncodeMode::Residual2);
        bench_mode(
            size,
            "adaptive2.0",
            EncodeMode::Adaptive {
                max_abs_error: ADAPTIVE_BUDGET,
            },
        );
        bench_color(size);
        bench_mips(size);
    }
}
