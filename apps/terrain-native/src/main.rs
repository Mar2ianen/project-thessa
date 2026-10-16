//! Migration viewer: canonical terrain, owned compute/raster/materials,
//! winit presentation/input and optional egui HUD. No Bevy or full client parity.
#![forbid(unsafe_code)]
mod celestial;
#[cfg(feature = "hud")]
mod hud;
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
    let mut hud_override = None;
    let mut sim_time = 0.0_f64;
    let mut sim_step = 0.0_f64;
    let mut vehicle_preview = false;
    let mut live_flight = false;
    let mut chase = false;
    let mut flight_step = None;
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
            "--vehicle-preview" => vehicle_preview = true,
            "--flight" => live_flight = true,
            "--chase" => chase = true,
            "--flight-step" => {
                flight_step = Some(
                    args.next()
                        .ok_or("--flight-step requires seconds per frame")?
                        .parse::<f64>()?,
                )
            }
            "--sim-time" => sim_time = args.next().ok_or("--sim-time requires seconds")?.parse()?,
            "--sim-step" => {
                sim_step = args
                    .next()
                    .ok_or("--sim-step requires seconds per frame")?
                    .parse()?
            }
            "--hud" => hud_override = Some(true),
            "--no-hud" => hud_override = Some(false),
            "--help" => {
                println!(
                    "thessa-terrain-native [--headless image.png] [--frames N] [--graphics graphics.toml] [--prestream-seconds 3] [--sweep] [--sim-time 0] [--sim-step 0] [--hud|--no-hud]\nWindow: egui HUD on by default (with hud feature); Left/Right orbit; Up/Down zoom; Esc exit when UI does not capture input.\nHeadless: HUD off by default; --hud includes it in capture/timings. --sweep advances orbit by 0.006 rad per rendered frame; pre-stream defaults to 3s.\n--sim-time samples baked ephemerides/flight terrain spin at seconds from epoch; --sim-step (headless only) advances sample time per rendered frame, including 30 warmup frames. Defaults preserve the frozen epoch. This is presentation validation, not server warp.\nLive canonical terrain streaming viewer, not the complete game client.\nOnly manual exposure and vsync settings are consumed; other effects are unported."
                );
                println!(
                    "--vehicle-preview loads the shared X15 mesh source at the exact initial flight pose (epoch only); diagnostic preview, not live flight. Camera distance: 8..180m, reset 32m.\nX15: North American X-15 Plane by cmoreau, CC BY 4.0; assets/models/north_american_x-15_plane.LICENSE.txt contains the source/license links."
                );
                println!(
                    "--flight runs the existing local FlightAuthority from canonical COAST launch with Navball attitude hold and tracking orbit camera. No flight-input/network/controller-UI parity. Requires --sim-time 0 --sim-step 0, incompatible with --vehicle-preview.\nHeadless --flight-step SECONDS (default 1/60, finite 0<step<=0.1) requests simulation elapsed per frame, including 30 warmup frames; pre-stream stays frozen. Window uses elapsed wall demand clamped to 0.1s and the old local client's 8ms work budget; no new physics dt or warp."
                );
                println!(
                    "--chase frames the vehicle with the shared pilot chase camera (follow craft attitude) instead of the survey orbit. Requires --flight or --vehicle-preview.\nWindow keys: Left/Right yaw orbit; Up/Down zoom; V chase toggle; Backquote reset camera; right-drag orbit; middle-drag pan; wheel zoom (all ignored while the HUD captures input). Esc exits."
                );
                return Ok(());
            }
            _ => return Err(format!("unknown argument: {arg}").into()),
        }
    }
    if frame_limit == Some(0) {
        return Err("--frames must be non-zero".into());
    }
    if live_flight && (vehicle_preview || sim_time != 0.0 || sim_step != 0.0) {
        return Err("--flight requires --sim-time 0 --sim-step 0 and no --vehicle-preview".into());
    }
    if chase && !live_flight && !vehicle_preview {
        return Err("--chase requires --flight or --vehicle-preview".into());
    }
    if flight_step.is_some() && (!live_flight || headless.is_none()) {
        return Err("--flight-step requires --flight and --headless".into());
    }
    let flight_step = flight_step.unwrap_or(1.0 / 60.0);
    if !flight_step.is_finite() || flight_step <= 0.0 || flight_step > 0.1 {
        return Err("--flight-step must be finite and within 0<step<=0.1".into());
    }
    if vehicle_preview && (sim_time != 0.0 || sim_step != 0.0) {
        return Err("--vehicle-preview requires epoch --sim-time 0 and --sim-step 0".into());
    }
    if !prestream_seconds.is_finite() || !(0.0..=3600.0).contains(&prestream_seconds) {
        return Err("--prestream-seconds must be finite and within 0..=3600".into());
    }
    if !sim_time.is_finite()
        || !sim_step.is_finite()
        || !(sim_time + sim_step * (f64::from(frame_limit.unwrap_or(120)) + 29.0)).is_finite()
    {
        return Err("presentation sample times must be finite".into());
    }
    if headless.is_none() && (sweep || prestream_seconds != 3.0 || sim_step != 0.0) {
        return Err("--sweep, --prestream-seconds and --sim-step require --headless".into());
    }
    let hud_enabled = hud_override.unwrap_or(cfg!(feature = "hud") && headless.is_none());
    if hud_enabled && !cfg!(feature = "hud") {
        return Err("--hud requires the hud Cargo feature (enabled by default)".into());
    }
    let started = Instant::now();
    let requested =
        thessa_graphics::RequestedGraphics::from_toml(&std::fs::read_to_string(&graphics_path)?)?;
    eprintln!(
        "migration viewer consumes manual exposure/vsync only; atmosphere, shadows, HDR/tonemapping, RT, automatic exposure and other effects are unported"
    );
    let mut scene = scene::Scene::load(requested)?;
    scene.set_presentation_time(thessa_sim_core::SimTime(sim_time))?;
    if vehicle_preview {
        scene.enable_vehicle_preview()?;
    }
    if live_flight {
        scene.enable_live_flight()?;
    }
    if chase {
        scene.camera.set_chase(true);
    }
    eprintln!(
        "canonical CPU scene build: {:.2}s (excluded from frame benchmark)",
        started.elapsed().as_secs_f64()
    );
    if let Some(path) = headless {
        let backend = Backend::bring_up()?;
        let target = OffscreenTarget::new(&backend, 1280, 720)?;
        let mut renderer = scene.renderer(&backend, OffscreenTarget::FORMAT)?;
        let mut mesh_renderer = scene.mesh_renderer(&backend, OffscreenTarget::FORMAT)?;
        #[cfg(feature = "hud")]
        let mut hud = hud_enabled.then(|| hud::Hud::new(&backend, OffscreenTarget::FORMAT));
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
        #[cfg(feature = "hud")]
        let mut previous_host_ms = 0.0;
        let count = frame_limit.unwrap_or(120);
        for frame in 0..u64::from(count) + 30 {
            let start = Instant::now();
            if live_flight {
                scene.advance_flight(flight_step, None)?;
            } else {
                scene.set_presentation_time(thessa_sim_core::SimTime(
                    sim_time + sim_step * frame as f64,
                ))?;
            }
            if sweep {
                if scene.vehicle_preview() {
                    scene.camera.orbit_yaw(0.006);
                } else {
                    scene.angle = frame as f64 * 0.006;
                }
            }
            #[cfg(feature = "hud")]
            let hud_output = hud.as_ref().map(|hud| {
                let mut controls = hud::Controls::from_scene(&scene);
                // This path has no platform input. It exercises exactly the
                // same widgets and GPU overlay as the native window.
                hud.run(
                    egui::RawInput {
                        screen_rect: Some(egui::Rect::from_min_size(
                            egui::Pos2::ZERO,
                            egui::vec2(1280.0, 720.0),
                        )),
                        time: Some(stream_started.elapsed().as_secs_f64()),
                        ..Default::default()
                    },
                    &mut controls,
                    &hud::Stats::from_scene(&scene, frame, previous_host_ms),
                )
            });
            let stream_start = Instant::now();
            let update = scene.tick(
                &backend,
                &mut renderer,
                1280,
                720,
                exposure,
                stream_started.elapsed().as_secs_f64(),
            )?;
            let stream_ms = stream_start.elapsed().as_secs_f64() * 1000.0;
            renderer.update_view(&backend, &scene.view(1280, 720, exposure))?;
            if let Some(mesh) = &mut mesh_renderer {
                scene.update_mesh(&backend, mesh, 1280, 720, exposure)?;
            }
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            renderer.encode(&mut encoder, &target.color, &target.depth);
            if let Some(mesh) = &mesh_renderer {
                mesh.encode(&mut encoder, &target.color, &target.depth);
            }
            #[cfg(feature = "hud")]
            let commands = if let (Some(hud), Some(output)) = (&mut hud, hud_output) {
                hud.encode(&backend, &mut encoder, &target.color, output, [1280, 720])
            } else {
                Vec::new()
            };
            #[cfg(not(feature = "hud"))]
            let commands = Vec::<wgpu::CommandBuffer>::new();
            backend.queue.submit(
                commands
                    .into_iter()
                    .chain(std::iter::once(encoder.finish())),
            );
            backend.device.poll(wgpu::PollType::wait_indefinitely())?;
            #[cfg(feature = "hud")]
            {
                previous_host_ms = start.elapsed().as_secs_f64() * 1000.0;
            }
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
            "Live canonical stream, 1280x720 offscreen: selection/poll/publication + camera upload + classify + raster + optional HUD + GPU completion; excludes setup/{prestream_seconds}s pre-stream/30 warmup frames/readback; NOT game FPS. sweep={sweep} hud={hud_enabled}"
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
        println!(
            "presentation_sim_time_s={} sim_step_s={sim_step} body_inertial_m={:?}",
            scene.sample.time.0, scene.sample.position_inertial_m
        );
        if let Some(mesh) = &mesh_renderer {
            println!(
                "mesh_submitted={:?}; drawn_triangles above counts terrain only; live_flight={live_flight}",
                mesh.draw_stats()
            );
        }
        if let Some(status) = scene.flight_telemetry() {
            println!(
                "flight_step_s={flight_step} {status}; full local physics advance included in frame timings, not isolated renderer cost"
            );
        }
        return Ok(());
    }
    window::run(scene, frame_limit, hud_enabled)
}

/// Same manual-camera convention as the current Bevy PBR host (EV100).
fn manual_exposure(ev100: f32) -> f32 {
    1.0 / (1.2 * 2.0f32.powf(ev100))
}
