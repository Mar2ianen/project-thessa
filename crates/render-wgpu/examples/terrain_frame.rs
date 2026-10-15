//! Standalone headless CBT frame, screenshot and latency benchmark. No Bevy.
//! cargo run --release -p thessa-render-wgpu --example terrain_frame -- output.png

use glam::DVec3;
use std::{error::Error, fs::File, io::BufWriter, time::Instant};
use thessa_rcbt_core::{
    HeightPage,
    tile_anchor::{TileAnchor, TileKey},
};
use thessa_render_wgpu::{
    Backend,
    terrain::{OffscreenTarget, TerrainPatch, TerrainRenderer},
    view::ViewParams,
};

fn main() -> Result<(), Box<dyn Error>> {
    let path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "terrain.png".into());
    let backend = Backend::bring_up()?;
    let width = 1280;
    let height = 720;
    let mut pages = Vec::new();
    let mut anchors = Vec::new();
    for y in 0..8 {
        for x in 0..8 {
            anchors.push(
                TileAnchor::new(TileKey::new(4, 10, 508 + x, 508 + y).unwrap(), 6_371_000.0)
                    .unwrap(),
            );
            let samples = (0..33 * 33)
                .map(|i| {
                    let u = (x as f64 + (i % 33) as f64 / 32.0) / 8.0;
                    let v = (y as f64 + (i / 33) as f64 / 32.0) / 8.0;
                    100.0
                        + 3500.0 * (-((u - 0.4).powi(2) + (v - 0.55).powi(2)) / 0.012).exp()
                        + 1700.0 * (-((u - 0.65).powi(2) + (v - 0.4).powi(2)) / 0.02).exp()
                })
                .collect::<Vec<_>>();
            pages.push(HeightPage::bake(&samples, 33, 0.05)?);
        }
    }
    let patches = anchors
        .iter()
        .zip(&pages)
        .map(|(a, p)| TerrainPatch {
            anchor: *a,
            page: p,
        })
        .collect::<Vec<_>>();
    let start = Instant::now();
    let mut renderer = TerrainRenderer::new(&backend, &patches, OffscreenTarget::FORMAT)?;
    backend.device.poll(wgpu::PollType::wait_indefinitely())?;
    let setup_ms = start.elapsed().as_secs_f64() * 1000.0;
    let target = OffscreenTarget::new(&backend, width, height)?;
    let eye = DVec3::new(0.0, -55_000.0, 6_371_000.0 + 45_000.0);
    let params = ViewParams {
        world_from_view: glam::dcamera::rh::view::look_at_mat4(
            eye,
            DVec3::new(0.0, 0.0, 6_371_000.0),
            DVec3::Y,
        )
        .inverse(),
        clip_from_view: glam::dcamera::rh::proj::directx::perspective(
            0.9,
            width as f64 / height as f64,
            1.0,
            1e7,
        ),
        world_position_m: eye,
        exposure: 1.0,
    };
    let mut timings = Vec::new();
    for frame in 0..150 {
        let start = Instant::now();
        renderer.update_view(&backend, &params)?;
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        renderer.encode(&mut encoder, &target.color, &target.depth);
        backend.queue.submit([encoder.finish()]);
        backend.device.poll(wgpu::PollType::wait_indefinitely())?;
        if frame >= 30 {
            timings.push(start.elapsed().as_secs_f64() * 1000.0);
        }
    }
    timings.sort_by(f64::total_cmp);
    let median = timings[timings.len() / 2];
    let p95 = timings[timings.len() * 95 / 100];
    let draw = renderer.read_draw(&backend)?;
    let pixels = target.read_rgba8(&backend)?;
    let mut png = png::Encoder::new(BufWriter::new(File::create(&path)?), width, height);
    png.set_color(png::ColorType::Rgba);
    png.set_depth(png::BitDepth::Eight);
    png.write_header()?.write_image_data(&pixels)?;
    println!(
        "adapter={} patches={} triangles={} setup_ms={setup_ms:.2}",
        backend.adapter_name,
        patches.len(),
        draw[0] / 3
    );
    println!(
        "offscreen {width}x{height}, 120 frames, camera upload + classify + indirect raster + GPU completion; no readback"
    );
    println!(
        "median_ms={median:.3} p95_ms={p95:.3} reciprocal_median_fps={:.1} screenshot={path}",
        1000.0 / median
    );
    for dirty_count in [1, patches.len()] {
        let updates = pages
            .iter()
            .take(dirty_count)
            .enumerate()
            .map(|(i, p)| (i as u32, p))
            .collect::<Vec<_>>();
        let mut times = Vec::new();
        for i in 0..130 {
            let start = Instant::now();
            renderer.update_pages(&backend, &updates)?;
            backend.device.poll(wgpu::PollType::wait_indefinitely())?;
            if i >= 30 {
                times.push(start.elapsed().as_secs_f64() * 1000.0);
            }
        }
        times.sort_by(f64::total_cmp);
        println!(
            "quantized upload + geometry + GPU completion: dirty={dirty_count} median_ms={:.3} p95_ms={:.3}",
            times[50], times[95]
        );
    }
    Ok(())
}
