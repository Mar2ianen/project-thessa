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
    let mut prestream_seconds = 3.0;
    let mut sweep = false;
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
            "--prestream-seconds" => {
                prestream_seconds = args
                    .next()
                    .ok_or("--prestream-seconds requires seconds")?
                    .parse::<f64>()?
            }
            "--sweep" => sweep = true,
            "--help" => {
                println!(
                    "thessa-terrain-native [--headless image.png] [--frames N] [--graphics graphics.toml] [--prestream-seconds 3] [--sweep]\nWindow: Left/Right orbit; Up/Down zoom; Esc exit.\nHeadless: --sweep advances orbit by 0.006 rad per rendered frame; pre-stream defaults to 3s.\nLive canonical terrain streaming viewer, not the complete game client.\nOnly manual exposure and vsync settings are consumed; other effects are unported."
                );
                return Ok(());
            }
            _ => return Err(format!("unknown argument: {arg}").into()),
        }
    }
    if frame_limit == Some(0) {
        return Err("--frames must be non-zero".into());
    }
    if !prestream_seconds.is_finite() || !(0.0..=3600.0).contains(&prestream_seconds) {
        return Err("--prestream-seconds must be finite and within 0..=3600".into());
    }
    if headless.is_none() && (sweep || prestream_seconds != 3.0) {
        return Err("--sweep and --prestream-seconds require --headless".into());
    }
    let started = Instant::now();
    let requested =
        thessa_graphics::RequestedGraphics::from_toml(&std::fs::read_to_string(&graphics_path)?)?;
    eprintln!(
        "migration viewer consumes manual exposure/vsync only; atmosphere, shadows, HDR/tonemapping, RT, automatic exposure and other effects are unported"
    );
    let mut scene = scene::Scene::load(requested)?;
    eprintln!(
        "canonical CPU scene build: {:.2}s (excluded from frame benchmark)",
        started.elapsed().as_secs_f64()
    );
    if let Some(path) = headless {
        let backend = Backend::bring_up()?;
        let target = OffscreenTarget::new(&backend, 1280, 720)?;
        let mut renderer = scene.renderer(&backend, OffscreenTarget::FORMAT)?;
        let resolved = thessa_graphics::ResolvedGraphicsSettings::from_requested(
            &scene.requested,
            &backend.caps,
        );
        let exposure = manual_exposure(resolved.exposure_ev100);
        let stream_started = Instant::now();
        while stream_started.elapsed().as_secs_f64() < prestream_seconds {
            scene.tick(
                &backend,
                &mut renderer,
                1280,
                720,
                exposure,
                stream_started.elapsed().as_secs_f64(),
            )?;
            std::thread::sleep(std::time::Duration::from_millis(2));
        }
        let mut timings = Vec::new();
        let mut streaming_timings = Vec::new();
        let mut measured_selections = 0;
        let mut measured_publications = 0;
        let count = frame_limit.unwrap_or(120);
        for frame in 0..u64::from(count) + 30 {
            let start = Instant::now();
            if sweep {
                scene.angle = frame as f64 * 0.006;
            }
            let update = scene.tick(
                &backend,
                &mut renderer,
                1280,
                720,
                exposure,
                stream_started.elapsed().as_secs_f64(),
            )?;
            let stream_ms = start.elapsed().as_secs_f64() * 1000.0;
            renderer.update_view(&backend, &scene.view(1280, 720, exposure))?;
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            renderer.encode(&mut encoder, &target.color, &target.depth);
            backend.queue.submit([encoder.finish()]);
            backend.device.poll(wgpu::PollType::wait_indefinitely())?;
            if frame >= 30 {
                measured_selections += u64::from(update.selected);
                measured_publications += u64::from(update.geometry_changed);
                timings.push(start.elapsed().as_secs_f64() * 1000.0);
                streaming_timings.push(stream_ms);
            }
        }
        timings.sort_by(f64::total_cmp);
        streaming_timings.sort_by(f64::total_cmp);
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
            "adapter={} canonical_tiles={} drawn_triangles={} frames={count} median_ms={median:.3} p95_ms={p95:.3} p99_ms={:.3} max_ms={:.3} reciprocal_median_fps={:.1} screenshot={path}",
            backend.adapter_name,
            scene.stream.visible().len(),
            renderer.read_draw(&backend)?[0] / 3,
            timings[timings.len() * 99 / 100],
            timings.last().unwrap(),
            1000.0 / median
        );
        println!(
            "Live canonical stream, 1280x720 offscreen: selection/poll/publication + camera upload + classify + raster + GPU completion; excludes setup/{prestream_seconds}s pre-stream/30 warmup frames/readback; NOT game FPS. sweep={sweep}"
        );
        println!(
            "geometry_publications={} regenerated_patches={} cache_entries={:?} pending_jobs={:?} wanted={}",
            scene.publications,
            scene.regenerated,
            scene.stream.cache_entries(),
            scene.stream.pending_jobs(),
            scene.stream.wanted_count()
        );
        println!(
            "stream_cpu_median_ms={:.3} stream_cpu_p95_ms={:.3} stream_cpu_p99_ms={:.3} stream_cpu_max_ms={:.3} material_upload_bytes={} capacity_rejections={} final_angle_rad={:.3} measured_selections={measured_selections} measured_publications={measured_publications}",
            streaming_timings[streaming_timings.len() / 2],
            streaming_timings[streaming_timings.len() * 95 / 100],
            streaming_timings[streaming_timings.len() * 99 / 100],
            streaming_timings.last().unwrap(),
            scene.material_upload_bytes,
            scene.capacity_rejections,
            scene.angle
        );
        return Ok(());
    }
    window::run(scene, frame_limit)
}

/// Same manual-camera convention as the current Bevy PBR host (EV100).
fn manual_exposure(ev100: f32) -> f32 {
    1.0 / (1.2 * 2.0f32.powf(ev100))
}
