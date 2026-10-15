//! Migration viewer: canonical frozen terrain, owned compute/raster/materials,
//! winit presentation/input. No Bevy, networking, HUD, or production effect parity.
#![forbid(unsafe_code)]
mod scene;
mod window;

use std::{error::Error, time::Instant};
use thessa_render_wgpu::{Backend, terrain::OffscreenTarget};

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args().skip(1);
    let mut headless = None;
    let mut frame_limit = None;
    let mut graphics_path = "graphics.toml".to_string();
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--headless" => headless = Some(args.next().ok_or("--headless requires a PNG path")?),
            "--frames" => {
                frame_limit = Some(
                    args.next()
                        .ok_or("--frames requires a count")?
                        .parse::<u32>()?,
                )
            }
            "--graphics" => graphics_path = args.next().ok_or("--graphics requires a TOML path")?,
            "--help" => {
                println!(
                    "thessa-terrain-native [--headless image.png] [--frames N] [--graphics graphics.toml]\nWindow: Left/Right orbit; Up/Down zoom; Esc exit.\nStatic canonical terrain migration viewer, not the complete game client.\nOnly manual exposure and vsync settings are consumed; other effects are unported."
                );
                return Ok(());
            }
            _ => return Err(format!("unknown argument: {arg}").into()),
        }
    }
    if frame_limit == Some(0) {
        return Err("--frames must be non-zero".into());
    }
    let started = Instant::now();
    let requested =
        thessa_graphics::RequestedGraphics::from_toml(&std::fs::read_to_string(&graphics_path)?)?;
    eprintln!(
        "migration viewer consumes manual exposure/vsync only; atmosphere, shadows, HDR/tonemapping, RT, automatic exposure and other effects are unported"
    );
    let scene = scene::Scene::load(requested)?;
    eprintln!(
        "canonical CPU scene build: {:.2}s (excluded from frame benchmark)",
        started.elapsed().as_secs_f64()
    );
    if let Some(path) = headless {
        let backend = Backend::bring_up()?;
        let target = OffscreenTarget::new(&backend, 1280, 720)?;
        let renderer = scene.renderer(&backend, OffscreenTarget::FORMAT)?;
        let resolved = thessa_graphics::ResolvedGraphicsSettings::from_requested(
            &scene.requested,
            &backend.caps,
        );
        let exposure = manual_exposure(resolved.exposure_ev100);
        let mut timings = Vec::new();
        let count = frame_limit.unwrap_or(120);
        for frame in 0..u64::from(count) + 30 {
            let start = Instant::now();
            renderer.update_view(&backend, &scene.view(1280, 720, exposure))?;
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
        let pixels = target.read_rgba8(&backend)?;
        let mut png = png::Encoder::new(
            std::io::BufWriter::new(std::fs::File::create(&path)?),
            1280,
            720,
        );
        png.set_color(png::ColorType::Rgba);
        png.set_depth(png::BitDepth::Eight);
        png.write_header()?.write_image_data(&pixels)?;
        println!(
            "adapter={} canonical_tiles={} drawn_triangles={} frames={count} median_ms={median:.3} p95_ms={p95:.3} reciprocal_median_fps={:.1} screenshot={path}",
            backend.adapter_name,
            scene.anchors.len(),
            renderer.read_draw(&backend)?[0] / 3,
            1000.0 / median
        );
        println!(
            "Static canonical snapshot, 1280x720 offscreen: camera upload + classify + raster + GPU completion; excludes setup/material upload/readback; NOT game FPS."
        );
        return Ok(());
    }
    window::run(scene, frame_limit)
}

/// Same manual-camera convention as the current Bevy PBR host (EV100).
fn manual_exposure(ev100: f32) -> f32 {
    1.0 / (1.2 * 2.0f32.powf(ev100))
}
