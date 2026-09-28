//! Game material storage benchmark (doc 41, §12 DoD item 5).
//!
//! Target-size batches for the CBT material path: 4 pages is one streaming
//! frame (`material_budget` in `apps/client/src/terrain.rs`), 32 pages is a
//! full scene cover. Each page is a real `CbtMaterialPage` (linear-light mip
//! averaging included): 128x128 RGBA base + 8-level mip chain, 87,380 B raw.
//!
//! The measured path is exactly what `material_storage = "microstore_compact"`
//! runs at upload time: `encode_material_level` per mip (budget 2.0 code
//! levels) + interleave decode back to RGBA shadow pages.
//!
//! Run with `cargo bench -p thessa-bevy-rcbt --bench material_storage
//! --features render`.

use std::{hint::black_box, time::Instant};

use thessa_bevy_rcbt::{CbtMaterialPage, material_microstore::encode_material_level};

const BUDGET: f64 = 2.0;
const PAGE: u32 = 128;
const RAW_PAGE_BYTES: usize = 87_380;

/// Deterministic strata+grain RGBA, same family as the `rock_rgba` test
/// fixture but self-contained (bench builds must not depend on test code).
fn rock_rgba(size: u32, seed: u64) -> Vec<u8> {
    let mut s = seed | 1;
    let mut next = || {
        s = s.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = s;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        (z ^ (z >> 31)) as f64 / u64::MAX as f64
    };
    let mut out = Vec::with_capacity(size as usize * size as usize * 4);
    for y in 0..size {
        for x in 0..size {
            let band = ((y / 16) % 2) as f64;
            let grain = next();
            let r = (90.0 + band * 60.0 + grain * 40.0).clamp(0.0, 255.0) as u8;
            let g = (70.0 + band * 40.0 + grain * 30.0).clamp(0.0, 255.0) as u8;
            let b = (50.0 + grain * 60.0 + (x % 7) as f64 * 3.0).clamp(0.0, 255.0) as u8;
            let a = (110.0 + band * 80.0).clamp(0.0, 255.0) as u8;
            out.extend_from_slice(&[r, g, b, a]);
        }
    }
    out
}

fn bench_batch(pages: &[CbtMaterialPage], label: &str) {
    // Warm up once and pin correctness before timing.
    let mut wire = 0usize;
    let mut worst = 0u8;
    for page in pages {
        let (mut w, mut h) = (PAGE, PAGE);
        for mip in page.mips().iter() {
            let level = encode_material_level(mip, w, h, BUDGET).expect("encodes");
            wire += level.encoded_bytes();
            let [r, g, b] = level.color.decode();
            let a = level.roughness.decode();
            for (i, ((rr, gg), (bb, aa))) in r
                .data
                .iter()
                .zip(g.data.iter())
                .zip(b.data.iter().zip(a.data.iter()))
                .enumerate()
            {
                for (o, d) in [*rr, *gg, *bb, *aa].iter().zip([
                    mip[i * 4],
                    mip[i * 4 + 1],
                    mip[i * 4 + 2],
                    mip[i * 4 + 3],
                ]) {
                    worst = worst.max(o.abs_diff(d));
                    assert!(o.abs_diff(d) <= 2, "over budget");
                }
            }
            w = (w / 2).max(1);
            h = (h / 2).max(1);
        }
    }
    let raw = pages.len() * RAW_PAGE_BYTES;
    assert!(wire < raw, "compact must beat raw RGBA");

    let iters = if pages.len() <= 4 { 50 } else { 10 };
    let started = Instant::now();
    for _ in 0..iters {
        for page in pages {
            let (mut w, mut h) = (PAGE, PAGE);
            for mip in page.mips().iter() {
                let level = encode_material_level(black_box(mip), w, h, BUDGET).expect("encodes");
                black_box(level);
                w = (w / 2).max(1);
                h = (h / 2).max(1);
            }
        }
    }
    let enc_ms = started.elapsed().as_secs_f64() * 1000.0 / iters as f64;

    // Decode timing: re-encode once, then interleave-decode the held levels.
    let held: Vec<_> = pages
        .iter()
        .map(|page| {
            let (mut w, mut h) = (PAGE, PAGE);
            let mut levels = Vec::with_capacity(page.mips().len());
            for mip in page.mips().iter() {
                levels.push(encode_material_level(mip, w, h, BUDGET).expect("encodes"));
                w = (w / 2).max(1);
                h = (h / 2).max(1);
            }
            levels
        })
        .collect();
    let started = Instant::now();
    for _ in 0..iters {
        for levels in &held {
            for level in black_box(levels) {
                let [r, g, b] = level.color.decode();
                let a = level.roughness.decode();
                let mut rgba = Vec::with_capacity(r.data.len() * 4);
                for i in 0..r.data.len() {
                    rgba.extend_from_slice(&[r.data[i], g.data[i], b.data[i], a.data[i]]);
                }
                black_box(rgba);
            }
        }
    }
    let dec_ms = started.elapsed().as_secs_f64() * 1000.0 / iters as f64;
    let total_s = (enc_ms + dec_ms) / 1000.0;
    let mib_s = raw as f64 / total_s.max(1e-9) / 1_048_576.0;

    println!(
        "{label}: {} pages, wire {wire} B vs raw {raw} B ({:.2}x), worst {worst}, \
         encode {enc_ms:.2} ms ({:.2} ms/page), decode {dec_ms:.2} ms, throughput {mib_s:.1} MiB/s raw-equiv",
        pages.len(),
        wire as f64 / raw as f64,
        enc_ms / pages.len() as f64,
    );
}

fn main() {
    println!("game material storage batches (budget {BUDGET}, {RAW_PAGE_BYTES} B raw/page):");
    let pages4: Vec<_> = (0..4)
        .map(|i| CbtMaterialPage::from_rgba8(rock_rgba(PAGE, 0x5EED + i)).expect("page builds"))
        .collect();
    bench_batch(&pages4, "stream-frame");
    let pages32: Vec<_> = (0..32)
        .map(|i| CbtMaterialPage::from_rgba8(rock_rgba(PAGE, 0xC0FF + i)).expect("page builds"))
        .collect();
    bench_batch(&pages32, "scene-cover ");
}
