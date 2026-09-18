//! Phase B/C telemetry (doc 41 §9): GPU decode parity cost against
//! upload/residency savings, plus an A/B line against the current RGBA
//! material path (128x128x4 B base level, 8 mip levels total).
//!
//! The milliseconds below are dispatch + synchronous readback of whole
//! pages (parity oracle), not material-shader sample cost.
//!
//! Run with `cargo bench -p thessa-rcbt-wgpu --bench microstore`.
//! Prints SKIP without a GPU adapter.

use std::{hint::black_box, sync::Arc, time::Instant};

use thessa_microstore_core::{EncodeMode, EncodedPage, fixtures, measure};
use thessa_microstore_core::{MipChain, sample_rounded};
use thessa_rcbt_wgpu::microstore::MicrostoreDecode;
use thessa_rcbt_wgpu::microstore_sample::{GpuMips, PackedSampler};

const ITERS: usize = 30;
const ADAPTIVE_BUDGET: f64 = 2.0;
/// Full 8-level mip chain of a 128x128 RGBA8 material page
/// (see `CbtMaterialPage::byte_len`).
const RGBA_MIP_PAGE_BYTES: usize = 87_380;
/// Base level only, for the no-mips-yet comparison line.
const RGBA_BASE_BYTES: usize = 65_536;
/// Matches the decoder upload path (one spare word past the payload).
const UPLOAD_SLACK_BYTES: usize = 4;

fn main() {
    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
    let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
        power_preference: wgpu::PowerPreference::HighPerformance,
        compatible_surface: None,
        force_fallback_adapter: false,
    }));
    let Ok(adapter) = adapter else {
        println!("SKIP: no GPU adapter available");
        return;
    };
    let info = adapter.get_info();
    println!("adapter: {} ({:?})", info.name, info.backend);
    let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
        label: Some("microstore-bench"),
        required_features: wgpu::Features::empty(),
        required_limits: wgpu::Limits::default(),
        experimental_features: wgpu::ExperimentalFeatures::default(),
        memory_hints: wgpu::MemoryHints::Performance,
        trace: wgpu::Trace::Off,
    }))
    .expect("bench device");
    let device = Arc::new(device);
    let queue = Arc::new(queue);
    let decoder = MicrostoreDecode::new(device.clone(), queue.clone());

    println!("microstore Phase B/C: scalar fields, {ITERS} iters (dispatch+readback)");
    println!(
        "{:>10} {:>10} {:>10} {:>10} {:>10} {:>10} {:>10}",
        "size.fixture.mode", "gpu-ms", "up-B", "res-B", "B/tex", "maxerr", "hdr%"
    );
    let modes: Vec<(&str, EncodeMode)> = vec![
        ("raw8", EncodeMode::Raw8),
        ("r8", EncodeMode::Residual8),
        ("r6", EncodeMode::Residual6),
        ("r4", EncodeMode::Residual4),
        ("r2", EncodeMode::Residual2),
        (
            "adapt",
            EncodeMode::Adaptive {
                max_abs_error: ADAPTIVE_BUDGET,
            },
        ),
    ];
    for size in [128u32, 256u32] {
        for (fixture_name, field) in fixtures::all(size, size) {
            for (mode_name, mode) in &modes {
                let page = EncodedPage::encode(&field, *mode);
                // Parity against the CPU reference before timing.
                let gpu = decoder.decode_page(&page).expect("gpu decode");
                assert_eq!(gpu, page.decode().data, "{fixture_name}.{mode_name} parity");
                let stats = measure(&field, &page.decode());

                let started = Instant::now();
                for _ in 0..ITERS {
                    let out = decoder.decode_page(black_box(&page)).expect("gpu decode");
                    black_box(out);
                }
                let ms = started.elapsed().as_secs_f64() * 1000.0 / ITERS as f64;
                // Upload = padded page + decoder slack + offset table +
                // params; residency adds the u32-per-texel decode target
                // a sampler would read.
                let up = page.encoded_bytes().div_ceil(4) * 4
                    + UPLOAD_SLACK_BYTES
                    + page.blocks.len() * 4
                    + 16;
                let texels = size as usize * size as usize;
                let resident = up + texels * 4;
                println!(
                    "{size}.{fixture_name}.{mode_name:>6} {:>10.3} {:>10} {:>10} {:>10.3} {:>10.1} {:>10.1}",
                    ms,
                    up,
                    resident,
                    page.bytes_per_texel(),
                    stats.max_abs,
                    page.header_fraction() * 100.0,
                );
            }
        }
    }

    // A/B against the current path. Honest accounting both sides:    // microstore pays padded wire + offset table + params per channel
    // (see upload_size), while CbtMaterialPage carries a full 8-level mip
    // chain (128²+64²+...+1² texels x RGBA = 87,380 B), not just the base
    // level. Microstore has no mips and no hardware filtering yet, so this
    // is upload telemetry, not an apples-to-apples quality comparison.
    println!(
        "A/B vs current material page ({RGBA_MIP_PAGE_BYTES} B with mips; {RGBA_BASE_BYTES} B base level only):"
    );
    let field = fixtures::noise(128, 128, 0xA8);
    let page = EncodedPage::encode(
        &field,
        EncodeMode::Adaptive {
            max_abs_error: ADAPTIVE_BUDGET,
        },
    );
    let up = thessa_microstore_core::wgsl::upload_size(&page);
    let four_channels = up.gpu_upload_bytes() * 4;
    println!(
        "4x adaptive channels: {four_channels} B upload ({:.2}x of full-mip RGBA, {:.2}x of base level)",
        four_channels as f64 / 87_380.0,
        four_channels as f64 / 65_536.0,
    );

    // Mip chain on GPU: decode once, then downsample level by level.
    // Compares against the CPU chain for parity and reports per-level ms.
    println!("gpu mip chain (128x128 noise, decode + levels):");
    let mips = GpuMips::new(device.clone(), queue.clone());
    let level0: Vec<u32> = field.data.iter().map(|v| *v as u32).collect();
    let cpu_chain = MipChain::build(&field, 7);
    let mut src = level0;
    let (mut w, mut h) = (128u32, 128u32);
    for (level, expect) in cpu_chain.levels.iter().skip(1).enumerate() {
        let started = Instant::now();
        let mut gpu = Vec::new();
        for _ in 0..ITERS {
            let (dw, dh, out) = mips
                .downsample(black_box(&src), black_box(w), black_box(h))
                .expect("mip");
            black_box(&out);
            gpu = out;
            assert_eq!((dw, dh), (expect.width, expect.height));
        }
        let ms = started.elapsed().as_secs_f64() * 1000.0 / ITERS as f64;
        let want: Vec<u32> = expect.data.iter().map(|v| *v as u32).collect();
        let parity = if gpu == want { "exact" } else { "MISMATCH" };
        println!(
            "  level {} {w}x{h} -> {}x{}: {ms:.3} ms [{parity}]",
            level + 1,
            expect.width,
            expect.height
        );
        src = gpu;
        (w, h) = (expect.width, expect.height);
    }

    // Packed sample-time filtering throughput on real data.
    println!("packed sample-time filtering (256 UVs x modes, coast 64x64):");
    let sampler = PackedSampler::new(device.clone(), queue.clone());
    let coast = fixtures::coast(64, 64, 0x5CA7);
    let mut uvs = Vec::with_capacity(256);
    let mut s = 0x5A4Eu64;
    for _ in 0..256 {
        s = s.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = s;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        let f = (z ^ (z >> 31)) as f64 / u64::MAX as f64;
        s = s.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z2 = s;
        z2 = (z2 ^ (z2 >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z2 = (z2 ^ (z2 >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        let f2 = (z2 ^ (z2 >> 31)) as f64 / u64::MAX as f64;
        uvs.push([f as f32, f2 as f32]);
    }
    for (mode_name, mode) in [
        ("r8", EncodeMode::Residual8),
        ("r4", EncodeMode::Residual4),
        (
            "adapt",
            EncodeMode::Adaptive {
                max_abs_error: ADAPTIVE_BUDGET,
            },
        ),
    ] {
        let page = EncodedPage::encode(&coast, mode);
        let decoded = page.decode();
        // Parity first (<= 1 level), then time it.
        let gpu = sampler.sample(&page, &uvs).expect("sample");
        let worst = uvs
            .iter()
            .zip(gpu.iter())
            .map(|(uv, got)| (*got as i32 - sample_rounded(&decoded, uv[0], uv[1]) as i32).abs())
            .max()
            .unwrap_or(-1);
        let started = Instant::now();
        for _ in 0..ITERS {
            let out = sampler
                .sample(black_box(&page), black_box(&uvs))
                .expect("sample");
            black_box(out);
        }
        let ms = started.elapsed().as_secs_f64() * 1000.0 / ITERS as f64;
        println!(
            "  {mode_name:>6}: {:.3} ms / 256 samples ({:.1} M samples/s), worst drift {worst} level",
            ms,
            256.0 / (ms / 1000.0) / 1e6,
        );
    }
}
