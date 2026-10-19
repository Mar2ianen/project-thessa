//! Migration viewer: canonical terrain, owned compute/raster/materials,
//! winit presentation/input and optional egui HUD. No Bevy or full client parity.
#![forbid(unsafe_code)]
mod celestial;
#[cfg(feature = "hud")]
mod contact_preview;
mod environment;
mod flyby;
#[cfg(feature = "hud")]
mod hud;
#[cfg(feature = "hud")]
mod map;
mod perf;
mod scene;
mod sky_layers;
mod window;

mod audio;
use std::{error::Error, time::Instant};
use thessa_render_wgpu::{Backend, terrain::OffscreenTarget};

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args().skip(1);
    let mut headless = None;
    let mut perf_capture = None;
    let mut frame_limit = None;
    let mut graphics_path = "graphics.toml".to_string();
    let mut prestream_seconds = 3.0;
    let mut sweep = false;
    let mut hud_override = None;
    let mut map_open = false;
    let mut sim_time = 0.0_f64;
    let mut sim_step = 0.0_f64;
    let mut vehicle_preview = false;
    let mut live_flight = false;
    let mut server_flight = false;
    let mut server_path = None;
    let mut server_timeout = 5.0_f64;
    let mut chase = false;
    let mut survey_site = None;
    let mut throttle_preset = None;
    let mut engine_on = false;
    let mut flight_step = None;
    let mut width = 1280u32;
    let mut height = 720u32;
    let mut audio = audio::Options::default();
    let mut warp = None;
    let mut paused = false;
    let mut flyby = false;
    let mut maneuver_kick = None;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--mute" => audio.muted = true,
            "--audio-cabin" => audio.cabin = true,
            "--audio-demo" => audio.demo = true,
            "--paused" => paused = true,
            "--flyby" => flyby = true,
            "--maneuver-kick" => {
                maneuver_kick = Some(
                    args.next()
                        .ok_or("--maneuver-kick requires a prograde dv in m/s")?
                        .parse::<f64>()?,
                )
            }
            "--warp" => {
                warp = Some(
                    args.next()
                        .ok_or("--warp requires a factor")?
                        .parse::<f64>()?,
                )
            }
            "--width" => {
                width = args.next().ok_or("--width requires pixels")?.parse()?;
            }
            "--height" => {
                height = args.next().ok_or("--height requires pixels")?.parse()?;
            }
            "--headless" => headless = Some(args.next().ok_or("--headless requires a PNG path")?),
            "--perf-capture" => {
                perf_capture = Some(std::path::PathBuf::from(
                    args.next().ok_or("--perf-capture requires a JSON path")?,
                ))
            }
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
            "--server-flight" => server_flight = true,
            "--server" => {
                server_path = Some(std::path::PathBuf::from(
                    args.next().ok_or("--server requires an executable path")?,
                ))
            }
            "--server-timeout" => {
                server_timeout = args
                    .next()
                    .ok_or("--server-timeout requires seconds")?
                    .parse()?
            }
            "--chase" => chase = true,
            "--site" => {
                survey_site = Some(
                    args.next()
                        .ok_or("--site requires 0, 1 or 2")?
                        .parse::<usize>()?,
                );
            }
            "--throttle" => {
                throttle_preset = Some(
                    args.next()
                        .ok_or("--throttle requires a 0..=1 fraction")?
                        .parse::<f64>()?,
                )
            }
            "--engine-on" => engine_on = true,
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
            "--map" => map_open = true,
            "--no-hud" => hud_override = Some(false),
            "--help" => {
                println!(
                    "thessa-terrain-native [--headless image.png] [--frames N] [--graphics graphics.toml] [--prestream-seconds 3] [--sweep] [--sim-time 0] [--sim-step 0] [--hud|--no-hud]\nWindow: egui HUD on by default (with hud feature); Left/Right orbit; Up/Down zoom; Esc exit when UI does not capture input.\nHeadless: HUD off by default; --hud includes it in capture/timings. --sweep advances orbit by 0.006 rad per rendered frame; pre-stream defaults to 3s.\n--sim-time samples baked ephemerides/flight terrain spin and the optional spectral sea at seconds from epoch; --sim-step (headless only) advances sample time per rendered frame, including 30 warmup frames. Defaults preserve the frozen epoch. This is presentation validation, not server warp.\nLive canonical terrain streaming viewer, not the complete game client.\nConsumes exposure/vsync, retained fleet mesh/material settings, shared raster shadows, textured mesh ray queries with resident terrain membership, opt-in conforming topology and spectral ocean, shared sky/environment and manual HDR/tonemapping. Native stars/source discs and depth-clipped clouds/aurora are implemented, including shared cloud shadows/reflections. Aerial perspective, primary terrain/ocean scene-object RT reflections, automatic exposure and bloom remain unported."
                );
                println!(
                    "--vehicle-preview loads the shared X15 mesh source at the exact initial flight pose (epoch only); diagnostic preview, not live flight. Camera distance: 8..180m, reset 32m.\nX15: North American X-15 Plane by cmoreau, CC BY 4.0; assets/models/north_american_x-15_plane.LICENSE.txt contains the source/license links."
                );
                println!(
                    "--perf-capture PATH.json requires --headless; writes thessa-perf JSON/CSV (last 1800 measured frames). Opt-in GPU command timestamps when supported, otherwise explicit unavailable. No profiler queries/buffers in normal runs. Mapping/capture writing excluded from frame samples; diagnostic scheduling can still perturb performance."
                );
                println!(
                    "--flight runs the existing local FlightAuthority from canonical COAST launch with Navball attitude hold and tracking orbit camera. No complete controller-UI parity. Requires --sim-time 0 --sim-step 0, incompatible with --vehicle-preview.\nHeadless --flight-step SECONDS (default 1/60, finite 0<step<=0.1) requests simulation elapsed per frame, including 30 warmup frames; pre-stream stays frozen. Window uses elapsed wall demand clamped to 0.1s and the old local client's 8ms work budget; no new physics dt or warp."
                );
                println!(
                    "--server-flight flies PRIMARY on the shared embedded stdio server (mutually exclusive with --flight/--vehicle-preview). --server PATH selects the executable; default: thessa-server beside the viewer. --server-timeout SECONDS (default 5, 0<seconds<=60) bounds handshake and snapshot silence. No local physics or fallback; a failed session freezes the last complete presentation. Scene time is receive-time interpolated; instruments show the newest authoritative sample. Server mode uses wall time, including headless runs; --flight-step is local-only."
                );
                println!(
                    "--chase frames the vehicle with the shared pilot chase camera (follow craft attitude) instead of the survey orbit. Requires --flight, --server-flight or --vehicle-preview.\nWindow keys: Left/Right yaw orbit; Up/Down zoom; V chase toggle; Backquote reset camera; right-drag orbit; middle-drag pan; wheel zoom (all ignored while the HUD captures input). Esc exits."
                );
                println!(
                    "Survey: --site 0|1|2 selects COAST/HIGHLANDS/VOLCANIC (default 1), incompatible with flight modes or --vehicle-preview. Shared quaternion orbit/free look, distance 80..2000000m (default/reset 5000m). Window F6 cycles sites; right-drag orbit; Shift-right-drag free look; wheel zoom; Backquote resets. Selecting a site resets distance and free look, preserving orbit."
                );
                println!(
                    "--throttle FRACTION (0..=1, requires --flight or --server-flight) presets throttle intent before the first frame. --engine-on arms engine intent. Window flight keys: WASDQE stick, Shift/Ctrl throttle slew, X cut, Z full, Space engine, C cycle control law, T SAS toggle, hold F to temporarily invert SAS, CapsLock precision; R/Y/G/P toggle RCS/wheels/gear/chutes. Mouse Aim uses the cursor with a 6% deadband; stick keys take over. Camera drag, UI capture, focus loss and pause cannot also steer. Server input acceptance means queued, not acknowledged; systems checkboxes show demands."
                );
                println!(
                    "Audio: --mute skips output-device creation; --audio-cabin selects the PRIMARY-attached diagnostic cabin. --audio-demo enables the delayed Mach-cone fixture (flight SimTime when flying, isolated elapsed-time fixture otherwise). No audio device is opened by headless capture, regardless of flags."
                );
                println!(
                    "Flight time: --paused and --warp 0..131072 require --flight or --server-flight. Pause/warp scale admitted simulation time, never solver dt. Window: F8 pause, [/] warp half/double, Backspace relaunch at the canonical site without rewinding time. Arrow keys remain camera controls; Space remains engine toggle. HUD separates intent from server acknowledgement. Lost server links require restarting the session, not a local reset."
                );
                println!(
                    "Map: --map starts the native egui system map (requires HUD). M toggles map/pilot ownership; flight continues with neutral manual axes and retained throttle/SAS. Six baked scopes, body/PRIMARY framing, orbit/pan/zoom and dominant-body Ap/Pe/escape/wake diagnostics. The path comes only from physics JIT, through its shared certified coast-batch interval/guards; no hypothetical zero-thrust/zero-aero forecast. Powered/live executable JIT futures and remote path transport remain unimplemented, explicitly unavailable rather than fabricated. Maneuver interaction is not yet ported."
                );
                println!(
                    "HUD keys (with HUD feature/enabled): F1 controls help, F2 flight HUD visibility (map remains usable), F3 telemetry/fleet panels. Help owns manual/camera input without pausing or cutting the engine; F1 restores visible help from hidden HUD. Hidden flight instruments release their GPU callback resource. These are session-only visibility changes, not graphics requests or authority acknowledgements."
                );
                println!(
                    "Flight instruments: dominant-body readouts; session-only Surface/Air/Orbital/Target speed cues and Datum/AGL selectors. Attitude stays local. Air/load/orbit and wheel/canopy details are available in the deck; absent remote air/force observations remain unavailable. Foreign bodies never borrow home rotation/air/terrain. Target inputs are not supplied yet."
                );
                println!(
                    "Coastal flyby: --flyby requires unpaused x1 --flight or --server-flight without --map; F7 / HUD toggles. Captures datum target and shore heading, selects Navball/SAS/engine/chase, writes normal pilot stick/throttle only. Manual takeover, map/help/focus loss, pause/warp/relaunch or failed/stale flight cancels without auto-resume. Not obstacle avoidance or a general autopilot."
                );
                println!(
                    "--maneuver-kick MPS queues one typed ExecuteManeuver prograde kick (1<dv<=2000 m/s, requires --server-flight) after the first acknowledged snapshot. The server executes the plan; the client never steps locally or fabricates burn observations. A rejected plan surfaces in the server wake notice. Sent at most once per session; local flight has no executor and is rejected."
                );
                return Ok(());
            }
            _ => return Err(format!("unknown argument: {arg}").into()),
        }
    }
    if frame_limit == Some(0) {
        return Err("--frames must be non-zero".into());
    }
    if perf_capture.is_some() && headless.is_none() {
        return Err("--perf-capture requires --headless".into());
    }
    if perf_capture
        .as_ref()
        .is_some_and(|path| path.extension().and_then(|s| s.to_str()) != Some("json"))
    {
        return Err("--perf-capture requires a .json path (CSV is written beside it)".into());
    }
    if let (Some(capture), Some(image)) = (&perf_capture, &headless) {
        let image = std::path::Path::new(image);
        if image == capture || image == capture.with_extension("csv") {
            return Err("--perf-capture requires paths separate from the screenshot".into());
        }
    }
    if server_flight && live_flight {
        return Err("--server-flight and --flight are mutually exclusive".into());
    }
    if !server_timeout.is_finite() || server_timeout <= 0.0 || server_timeout > 60.0 {
        return Err("--server-timeout must be finite and within 0<seconds<=60".into());
    }
    if !server_flight && (server_path.is_some() || server_timeout != 5.0) {
        return Err("--server and --server-timeout require --server-flight".into());
    }
    if (live_flight || server_flight) && (vehicle_preview || sim_time != 0.0 || sim_step != 0.0) {
        let mode = if server_flight {
            "--server-flight"
        } else {
            "--flight"
        };
        return Err(
            format!("{mode} requires --sim-time 0 --sim-step 0 and no --vehicle-preview").into(),
        );
    }
    let live_flight = live_flight || server_flight;
    if flyby && (!live_flight || paused || warp.is_some_and(|w| w != 1.0) || map_open) {
        return Err(
            "--flyby requires unpaused x1 --flight or --server-flight without --map".into(),
        );
    }
    if let Some(dv) = maneuver_kick {
        // Retained fixture semantics: embedded wire only; the server executes
        // the plan. Local mode has no maneuver executor, so it stays rejected
        // instead of silently doing nothing.
        if !server_flight {
            return Err(
                "--maneuver-kick requires --server-flight; local mode has no executor".into(),
            );
        }
        if !dv.is_finite() || dv <= 1.0 || dv > 2000.0 {
            return Err("--maneuver-kick dv must be finite and within 1<dv<=2000 m/s".into());
        }
    }
    if (paused || warp.is_some()) && !live_flight {
        return Err("--paused/--warp require --flight or --server-flight".into());
    }
    if warp.is_some_and(|factor| {
        !factor.is_finite() || !(0.0..=thessa_flight_net::MAX_TIME_WARP).contains(&factor)
    }) {
        return Err("warp must be finite and within 0..=131072".into());
    }
    if chase && !live_flight && !vehicle_preview {
        return Err("--chase requires --flight, --server-flight or --vehicle-preview".into());
    }
    if survey_site.is_some_and(|site| site >= thessa_flight_authority::SURVEY_SITE_LABELS.len()) {
        return Err("--site requires 0, 1 or 2".into());
    }
    if survey_site.is_some() && (live_flight || vehicle_preview) {
        return Err(
            "--site is survey-only; incompatible with flight modes or --vehicle-preview".into(),
        );
    }
    if throttle_preset.is_some() && !live_flight {
        return Err("--throttle requires --flight or --server-flight".into());
    }
    if engine_on && !live_flight {
        return Err("--engine-on requires --flight or --server-flight".into());
    }
    if let Some(throttle) = throttle_preset
        && (!throttle.is_finite() || !(0.0..=1.0).contains(&throttle))
    {
        return Err("--throttle must be finite and within 0..=1".into());
    }
    if flight_step.is_some() && (!live_flight || server_flight || headless.is_none()) {
        return Err("--flight-step requires local --flight and --headless".into());
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
    if map_open && !hud_enabled {
        return Err("--map requires HUD; use --hud for headless map captures".into());
    }
    if hud_enabled && !cfg!(feature = "hud") {
        return Err("--hud requires the hud Cargo feature (enabled by default)".into());
    }
    let started = Instant::now();
    let requested =
        thessa_graphics::RequestedGraphics::from_toml(&std::fs::read_to_string(&graphics_path)?)?;
    eprintln!(
        "migration viewer consumes exposure/vsync, retained fleet meshes, opaque material/reflection settings, shared raster shadows, textured mesh ray queries with resident terrain membership, opt-in conforming topology and spectral ocean, shared sky/environment with source discs, stars, depth-clipped clouds/aurora and manual HDR/tonemapping; aerial perspective, primary terrain/ocean scene-object RT reflections, automatic exposure and bloom remain unported"
    );
    let mut scene = scene::Scene::load(requested)?;
    #[cfg(feature = "hud")]
    scene.set_map_open(map_open);
    if let Some(site) = survey_site {
        scene.select_site(site);
    }
    scene.set_presentation_time(thessa_sim_core::SimTime(sim_time))?;
    if vehicle_preview {
        scene.enable_vehicle_preview()?;
    }
    if server_flight {
        scene.enable_server_flight(
            server_path.as_deref(),
            std::time::Duration::from_secs_f64(server_timeout),
        )?;
    } else if live_flight {
        scene.enable_live_flight()?;
    }
    scene.request_pause(paused);
    if let Some(warp) = warp {
        scene.request_warp(warp)?;
    }
    if chase {
        scene.camera.set_chase(true);
    }
    if let Some(throttle) = throttle_preset {
        scene.input.throttle = throttle;
    }
    if engine_on {
        scene.input.engine_active = true;
    }
    if server_flight {
        // Queue presets before GPU startup/pre-stream; otherwise the already
        // running child would keep constructor throttle until the first draw.
        scene.push_live_input(0.0, false, false);
        scene.advance_flight(0.0, None)?;
    }
    if flyby {
        // One-shot startup request, never recurring environment auto-engage.
        if server_flight {
            let deadline = Instant::now() + std::time::Duration::from_secs_f64(server_timeout);
            while scene.server_snapshot_count() == Some(0) && !scene.flight_stopped() {
                scene.push_live_input(0.0, false, false);
                scene.advance_flight(0.0, None)?;
                if Instant::now() >= deadline {
                    return Err("flyby waiting for first server snapshot timed out".into());
                }
                std::thread::sleep(std::time::Duration::from_millis(2));
            }
        }
        scene.set_flyby(true)?;
    }
    if let Some(dv) = maneuver_kick {
        // One-shot typed ExecuteManeuver over the wire; the server owns the
        // plan. Wait for the first acknowledged snapshot, then retry only
        // while the link is alive and the craft has relative motion.
        let deadline = Instant::now() + std::time::Duration::from_secs_f64(server_timeout);
        loop {
            if scene.flight_stopped() {
                return Err("flight stopped before the maneuver kick could be sent".into());
            }
            match scene.request_maneuver_kick(dv) {
                Ok(true) => break,
                Ok(false) => {}
                Err(error) => return Err(error.into()),
            }
            scene.push_live_input(0.0, false, false);
            scene.advance_flight(0.0, None)?;
            if Instant::now() >= deadline {
                return Err(
                    "maneuver kick waiting for an acknowledged moving flight timed out".into(),
                );
            }
            std::thread::sleep(std::time::Duration::from_millis(2));
        }
        eprintln!("maneuver kick queued: prograde {dv:.1} m/s, server-executed plan");
    }
    eprintln!(
        "canonical CPU scene build: {:.2}s (excluded from frame benchmark)",
        started.elapsed().as_secs_f64()
    );
    if let Some(path) = headless {
        let backend = if perf_capture.is_some() {
            Backend::bring_up_for_graphics_profiled(&scene.requested)?
        } else {
            Backend::bring_up_for_graphics(&scene.requested)?
        };
        let mut perf = perf_capture
            .as_ref()
            .map(|_| perf::HeadlessPerf::new(&backend));
        let target = OffscreenTarget::new(&backend, width, height)?;
        let resolved = backend.resolve_graphics(&scene.requested);
        if scene.requested.renderer.hdr && !resolved.hdr {
            eprintln!("native HDR target unsupported: using direct LDR fallback");
        }
        let display = thessa_render_wgpu::display::DisplayTransform::new(
            &backend,
            &resolved,
            OffscreenTarget::FORMAT,
            width,
            height,
        )?;
        let mut renderer = scene.renderer(&backend, display.scene_format())?;
        let sky_layers = scene.sky_layers(&backend, display.scene_format(), &mut renderer);
        println!(
            "native_sky_layers enabled={} storage_bytes={}",
            sky_layers.enabled(),
            sky_layers.storage_bytes()
        );
        let mut mesh_renderer =
            scene.mesh_renderer_with_shadows(&backend, display.scene_format(), &renderer)?;
        #[cfg(feature = "hud")]
        let mut hud = hud_enabled.then(|| hud::Hud::new(&backend, OffscreenTarget::FORMAT));
        println!(
            "native_display hdr={} scene_format={:?} target_bytes={} tonemapping={} exposure=manual auto_exposure_applied=false bloom_applied=false",
            display.enabled(),
            display.scene_format(),
            display.target_bytes(),
            if display.enabled() {
                "reinhard"
            } else {
                "none"
            }
        );
        let exposure = manual_exposure(resolved.exposure_ev100);
        let stream_started = Instant::now();
        while stream_started.elapsed().as_secs_f64() < prestream_seconds {
            scene.tick(
                &backend,
                &mut renderer,
                width,
                height,
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
        let mut last_flight_frame = Instant::now();
        for frame in 0..u64::from(count) + 30 {
            let start = Instant::now();
            if live_flight {
                let elapsed = if server_flight {
                    start
                        .duration_since(last_flight_frame)
                        .as_secs_f64()
                        .min(0.1)
                } else {
                    flight_step
                };
                last_flight_frame = start;
                scene.push_live_input(elapsed, false, false);
                scene.advance_flight(elapsed, None)?;
            } else {
                scene.set_presentation_time(thessa_sim_core::SimTime(
                    sim_time + sim_step * frame as f64,
                ))?;
            }
            if sweep {
                if scene.vehicle_preview() {
                    scene.camera.orbit_yaw(0.006);
                } else {
                    scene
                        .survey
                        .set_orbit(glam::Quat::from_rotation_y(frame as f32 * 0.006));
                }
            }
            #[cfg(feature = "hud")]
            let hud_output = hud.as_mut().map(|hud| {
                let mut controls = hud::Controls::from_scene(&scene);
                // This path has no platform input. It exercises exactly the
                // same widgets and GPU overlay as the native window.
                hud.run(
                    egui::RawInput {
                        screen_rect: Some(egui::Rect::from_min_size(
                            egui::Pos2::ZERO,
                            egui::vec2(width as f32, height as f32),
                        )),
                        // Script the presentation-only UI clock as well: wall
                        // speed must not leave a captured window half faded.
                        // This never advances physics, sky or audio SimTime.
                        time: Some(frame as f64 / 60.0),
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
                width,
                height,
                exposure,
                stream_started.elapsed().as_secs_f64(),
            )?;
            let stream_ms = stream_start.elapsed().as_secs_f64() * 1000.0;
            renderer.update_view_for_extent(
                &backend,
                &scene.view(width, height, exposure),
                [width, height],
            )?;
            renderer.update_ocean(&backend, scene.sample.time.0)?;
            scene.update_sky_layers(&backend, &sky_layers, width, height, exposure)?;
            renderer.update_shadows(
                &backend,
                scene.view(width, height, exposure).world_position_m,
                &scene.lighting(),
            )?;
            if let Some(mesh) = &mut mesh_renderer {
                scene.update_mesh(&backend, mesh, width, height, exposure)?;
                mesh.sync_rt_terrain(&backend, &renderer)?;
            }
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            if let Some(perf) = &perf {
                perf.mark(&mut encoder, 0);
            }
            renderer.encode_surface(&mut encoder);
            renderer.encode_shadows(
                &mut encoder,
                mesh_renderer.as_ref().and_then(scene::SceneMeshes::opaque),
            );
            if let Some(perf) = &perf {
                perf.mark(&mut encoder, 1);
            }
            if let Some(mesh) = &mut mesh_renderer {
                mesh.encode_rt(&mut encoder);
            }
            if let Some(perf) = &perf {
                perf.mark(&mut encoder, 2);
            }
            renderer.encode_scene_ops(
                &mut encoder,
                display.scene_color(&target.color),
                &target.depth,
                mesh_renderer.as_ref().and_then(scene::SceneMeshes::opaque),
                sky_layers.enabled(),
            );
            sky_layers.encode(
                &backend,
                &mut encoder,
                display.scene_color(&target.color),
                &target.depth,
            );
            if let Some(perf) = &perf {
                perf.mark(&mut encoder, 3);
            }
            display.encode(&mut encoder, &target.color);
            if let Some(perf) = &perf {
                perf.mark(&mut encoder, 4);
            }
            #[cfg(feature = "hud")]
            let commands = if let (Some(hud), Some(output)) = (&mut hud, hud_output) {
                hud.encode(
                    &backend,
                    &mut encoder,
                    &target.color,
                    output,
                    [width, height],
                )
            } else {
                Vec::new()
            };
            #[cfg(not(feature = "hud"))]
            let commands = Vec::<wgpu::CommandBuffer>::new();
            if let Some(perf) = &perf {
                perf.mark(&mut encoder, 5);
                perf.resolve(&mut encoder);
            }
            backend.queue.submit(
                commands
                    .into_iter()
                    .chain(std::iter::once(encoder.finish())),
            );
            let cpu_submit_s = perf.as_ref().map(|_| start.elapsed().as_secs_f64());
            backend.device.poll(wgpu::PollType::wait_indefinitely())?;
            let wall_s = start.elapsed().as_secs_f64();
            #[cfg(feature = "hud")]
            {
                previous_host_ms = start.elapsed().as_secs_f64() * 1000.0;
            }
            if frame >= 30 {
                measured_selections += u64::from(update.selected);
                measured_publications += u64::from(update.geometry_changed);
                timings.push(wall_s * 1000.0);
                streaming_timings.push(stream_ms);
                if let Some(perf) = &mut perf {
                    perf.record(
                        &backend,
                        &scene,
                        mesh_renderer.as_ref(),
                        wall_s,
                        cpu_submit_s.unwrap(),
                        stream_ms / 1000.0,
                    )?;
                }
            }
        }
        timings.sort_by(f64::total_cmp);
        if let (Some(perf), Some(path)) = (&perf, &perf_capture) {
            perf.write(path,&backend,&scene,&display,&sky_layers,width,height,format!("terrain-native vehicle_preview={vehicle_preview} live_flight={live_flight} server_flight={server_flight} sweep={sweep} hud={hud_enabled} prestream_s={prestream_seconds} sim_step_s={sim_step} clock={:?}",scene.flight_clock()))?;
        }
        streaming_timings.sort_by(f64::total_cmp);
        let median = timings[timings.len() / 2];
        let p95 = timings[timings.len() * 95 / 100];
        let pixels = target.read_rgba8(&backend)?;
        let mut png = png::Encoder::new(
            std::io::BufWriter::new(std::fs::File::create(&path)?),
            width,
            height,
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
            "Live canonical stream, {width}x{height} offscreen: selection/poll/publication + camera upload + classify + raster + optional HUD + GPU completion; excludes setup/{prestream_seconds}s pre-stream/30 warmup frames/readback; NOT game FPS. sweep={sweep} hud={hud_enabled}"
        );
        println!(
            "terrain_topology={} topology_bytes={} topology_revision={} fallback={}; conforming drawn_triangles includes zero-area free/virtual allocation slots, not live-leaf count",
            if renderer.conforming_surface().is_some() {
                "conforming"
            } else {
                "patches"
            },
            renderer.topology_storage_bytes(),
            renderer
                .conforming_surface()
                .map_or(0, |s| s.publication().revision),
            renderer.topology_reason().unwrap_or("none")
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
            f64::from(scene.survey.orbit().to_euler(glam::EulerRot::YXZ).0)
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
            println!(
                "fleet_mesh_source_publications={} epoch_withheld={} size_culled={}; retained neutral shells at PRIMARY presentation epoch, no extrapolation",
                mesh.source_publications, mesh.epoch_withheld, mesh.size_culled
            );
        }
        println!(
            "shadow_atlas_bytes={} shadow_active_layers={} shadow_rendered_passes={} terrain_shadow_draws={:?} material_quality={:?} specular_reflections={}",
            renderer.shadows().map_or(0, |s| s.storage_bytes()),
            renderer.shadows().map_or(0, |s| s.active_passes()),
            renderer.last_shadow_passes(),
            renderer.last_shadow_draws(),
            resolved.surface_quality,
            resolved.specular_reflections
        );
        println!(
            "sky_background={} shared_environment_publication=true; single-scattering field, native stars/source discs and depth-clipped clouds/aurora; no aerial perspective",
            renderer.sky_enabled()
        );
        let ocean = renderer.ocean_stats();
        println!(
            "spectral_ocean=size={} evaluations={} last_fft_dispatches={} field_bytes={} near_bytes={} near_patches={} absolute_height_bound_m={:.6} fallback={}",
            ocean.size,
            ocean.evaluations,
            ocean.fft_passes,
            ocean.storage_bytes,
            ocean.geometry_bytes,
            ocean.deformed_patches,
            ocean.height_bound_m,
            renderer.ocean_reason().unwrap_or("none")
        );
        if let Some(mesh) = mesh_renderer.as_ref().and_then(scene::SceneMeshes::opaque) {
            let stats = mesh.rt_stats();
            if stats.active {
                println!(
                    "ray_tracing=active tlas_instances={} traced_triangles={} rt_bytes={} as_builds={} terrain_instances={} terrain_blas_builds={}; resident selected terrain + opaque meshes; raster atlas retained",
                    stats.instances,
                    stats.triangles,
                    stats.bytes,
                    stats.passes,
                    stats.terrain_instances,
                    stats.terrain_blas_builds
                );
            } else {
                println!(
                    "ray_tracing=inactive reason={}; environment-only reflection fallback",
                    mesh.rt_reason().unwrap_or(if stats.bytes > 0 {
                        "no admitted live ray instances"
                    } else {
                        "ray tracing not requested"
                    })
                );
            }
            if let Some(reason) = mesh.rt_terrain_reason() {
                println!("ray_terrain_fallback={reason}");
            }
            let (enabled, width, height, storage_bytes, uploads) = mesh.environment_stats();
            println!(
                "renderer_environment=enabled={enabled} grid={}x{} storage_bytes={storage_bytes} uploads={uploads}",
                width, height
            );
        }
        match scene.environment_stats() {
            Some(field) if field.enabled => println!(
                "environment=enabled grid={}x{} view_steps={} sun_steps={} ambient_lux=[{:.2},{:.2},{:.2}] evaluations={} skipped={} storage_bytes={} last_evaluation_ms={:.2} aurora_steps={}; shared sky/emission field, sampled by background, raster and RT reflection terms",
                field.width,
                field.height,
                field.view_steps,
                field.sun_steps,
                field.ambient_lux[0],
                field.ambient_lux[1],
                field.ambient_lux[2],
                field.evaluated,
                field.skipped,
                field.storage_bytes,
                field.last_evaluation_ms,
                field.aurora_steps
            ),
            Some(_) => println!(
                "environment=disabled; isotropic ambient_lux/PI reflection term, no sky integration"
            ),
            None => println!("environment=unavailable; body has no atmosphere provider"),
        }
        if let Some(status) = scene.flight_telemetry() {
            if server_flight {
                println!(
                    "{status}; server runs in wall time; snapshot adoption/presentation included in frame timings; child CPU work is concurrent, not isolated renderer/network cost"
                );
            } else {
                println!(
                    "flight_step_s={flight_step} {status}; full local physics advance included in frame timings, not isolated renderer cost"
                );
            }
        }
        return Ok(());
    }
    window::run(scene, frame_limit, hud_enabled, width, height, audio)
}

/// Same manual-camera convention as the current Bevy PBR host (EV100).
fn manual_exposure(ev100: f32) -> f32 {
    thessa_render_core::display::manual_exposure(ev100)
}
