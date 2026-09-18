//! Phase B telemetry (doc 41 §9): GPU sample-time decode cost against
//! upload/residency savings, plus an A/B line against the current RGBA
//! material path (128x128x4 B, kept as the fallback).
//!
//! Run with `cargo bench -p thessa-rcbt-wgpu --bench microstore`.
//! Prints SKIP without a GPU adapter.

use std::{hint::black_box, sync::Arc, time::Instant};

use thessa_microstore_core::{EncodeMode, EncodedPage, fixtures, measure};
use thessa_rcbt_wgpu::microstore::MicrostoreDecode;

const ITERS: usize = 30;
const ADAPTIVE_BUDGET: f64 = 2.0;
/// Current material page: 128x128 RGBA8 (see `CbtMaterialPage`).
const RGBA_PAGE_BYTES: usize = 128 * 128 * 4;

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
    let decoder = MicrostoreDecode::new(Arc::new(device), Arc::new(queue));

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
                // Upload = padded page + offset table + params; residency adds
                // the u32-per-texel decode target a sampler would read.
                let up = page.encoded_bytes().div_ceil(4) * 4 + page.blocks.len() * 4 + 16;
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

    // A/B against the current path: four microstore channels (albedo RGB +
    // roughness, one scalar page each) versus one RGBA8 material page.
    println!("A/B vs current RGBA material path ({RGBA_PAGE_BYTES} B):");
    let field = fixtures::noise(128, 128, 0xA8);
    let page = EncodedPage::encode(
        &field,
        EncodeMode::Adaptive {
            max_abs_error: ADAPTIVE_BUDGET,
        },
    );
    let four_channels = page.encoded_bytes() * 4;
    println!(
        "4x adaptive channels: {four_channels} B upload ({:.2}x of RGBA), residency is decode-target dependent",
        four_channels as f64 / RGBA_PAGE_BYTES as f64
    );
}
