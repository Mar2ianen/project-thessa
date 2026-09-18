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
use thessa_rcbt_wgpu::microstore::MicrostoreDecode;

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

    // A/B against the current path. Honest accounting both sides:
    // microstore pays padded wire + offset table + params per channel
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
}
