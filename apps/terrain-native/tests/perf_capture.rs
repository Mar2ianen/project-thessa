//! Native adapter contract: reuse the portable schema, never label CPU submit
//! as GPU time; no temporary files outside the crate's ignored target tree.
#[test]
fn map_cli_requires_an_available_enabled_hud_before_loading_the_world() {
    let mut cases = vec![
        vec!["--map", "--no-hud"],
        vec!["--map", "--headless", "unused.png"],
    ];
    if !cfg!(feature = "hud") {
        cases.push(vec!["--map", "--hud"]);
    }
    for args in cases {
        let output = std::process::Command::new(env!("CARGO_BIN_EXE_thessa-terrain-native"))
            .args(args)
            .output()
            .unwrap();
        assert!(!output.status.success());
        let error = String::from_utf8_lossy(&output.stderr);
        assert!(error.contains("requires"), "{error}");
        assert!(!error.contains("canonical Thessa:"));
    }
}

#[test]
#[cfg(feature = "hud")]
#[ignore = "hardware: native map through actual silent CLI, HUD pixels and authoritative clock counters"]
fn native_map_cli_renders_real_system_overlay_without_pausing_or_opening_audio() {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    let scratch = root
        .join("target/test-tmp")
        .join(format!("native-map-{}", std::process::id()));
    std::fs::create_dir_all(&scratch).unwrap();
    let graphics = scratch.join("graphics.toml");
    std::fs::write(&graphics,"version=1\npreset=\"custom\"\n[renderer]\nray_tracing=\"off\"\nhdr=false\n[atmosphere]\nenabled=false\nsky_dome=false\nstars=false\n[upper_atmosphere]\naurora=false\nairglow=false\n[clouds]\nenabled=false\n[ocean]\nenabled=false\n[shadows]\nenabled=false\n").unwrap();
    let image = scratch.join("map.png");
    let json = scratch.join("map.json");
    let output = std::process::Command::new(env!("CARGO_BIN_EXE_thessa-terrain-native"))
        .current_dir(root.join("../.."))
        .args([
            "--map",
            "--hud",
            "--flight",
            "--flight-step",
            "0.025",
            "--frames",
            "2",
            "--width",
            "1280",
            "--height",
            "800",
            "--mute",
            "--prestream-seconds",
            "0",
            "--graphics",
        ])
        .arg(graphics)
        .arg("--headless")
        .arg(&image)
        .arg("--perf-capture")
        .arg(&json)
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(!String::from_utf8_lossy(&output.stderr).contains("native_audio=active"));
    let capture =
        thessa_perf::PerfCapture::from_json(&std::fs::read_to_string(json).unwrap()).unwrap();
    assert_eq!(
        capture.metadata.graphics.resolved["native_map_open"],
        "true"
    );
    assert_eq!(capture.frames.len(), 2);
    for frame in &capture.frames {
        assert_eq!(frame.sim.steps_this_frame, 3);
        assert!((frame.sim.sim_time_advanced_s - 0.025).abs() < 1e-12);
    }
    assert!((capture.frames[1].sim_time_s - 0.8).abs() < 1e-10);
    let mut reader = png::Decoder::new(std::io::BufReader::new(
        std::fs::File::open(&image).unwrap(),
    ))
    .read_info()
    .unwrap();
    let mut pixels = vec![0; reader.output_buffer_size().unwrap()];
    let info = reader.next_frame(&mut pixels).unwrap();
    assert_eq!(info.color_type, png::ColorType::Rgba);
    let canvas_pixels = pixels[..info.buffer_size()]
        .as_chunks::<4>()
        .0
        .iter()
        // egui's display dithering may change the encoded value by one LSB;
        // this is a map-presence oracle, not an exact shader-color contract.
        .filter(|p| {
            p[..3]
                .iter()
                .zip([5_u8, 9, 18])
                .all(|(a, b)| a.abs_diff(b) <= 1)
        })
        .count();
    assert!(
        canvas_pixels > 30000,
        "actual map canvas absent: {canvas_pixels}"
    );
    println!(
        "native_map_cli canvas_pixels={canvas_pixels} time_s=0.8 fixed_steps_per_frame=3 audio=silent image={}",
        image.display()
    );
}

#[test]
fn flight_time_cli_rejects_invalid_or_nonflight_requests_before_startup() {
    for args in [
        vec!["--paused"],
        vec!["--warp", "2"],
        vec!["--flight", "--warp", "NaN"],
        vec!["--flight", "--warp", "-1"],
        vec!["--flight", "--warp", "131073"],
        vec!["--flight", "--warp", "inf"],
    ] {
        let output = std::process::Command::new(env!("CARGO_BIN_EXE_thessa-terrain-native"))
            .args(args)
            .output()
            .unwrap();
        assert!(!output.status.success());
        let error = String::from_utf8_lossy(&output.stderr);
        assert!(
            error.contains("warp") || error.contains("--paused"),
            "{error}"
        );
        assert!(
            !error.contains("canonical Thessa:"),
            "invalid inputs must fail before loading the world"
        );
    }
}

#[test]
#[ignore = "hardware: actual native CLI pause/warp and performance budget records, no audio"]
fn headless_flight_time_requests_preserve_fixed_steps_and_truthful_capture_counters() {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    let scratch = root
        .join("target/test-tmp")
        .join(format!("native-clock-{}", std::process::id()));
    std::fs::create_dir_all(&scratch).unwrap();
    let graphics = scratch.join("graphics.toml");
    std::fs::write(&graphics,"version=1\npreset=\"custom\"\n[renderer]\nray_tracing=\"off\"\nhdr=false\n[atmosphere]\nenabled=false\nsky_dome=false\nstars=false\n[upper_atmosphere]\naurora=false\nairglow=false\n[clouds]\nenabled=false\n[ocean]\nenabled=false\n[shadows]\nenabled=false\n").unwrap();
    for paused in [false, true] {
        let json = scratch.join(format!("clock-{paused}.json"));
        let mut command = std::process::Command::new(env!("CARGO_BIN_EXE_thessa-terrain-native"));
        command
            .current_dir(root.join("../.."))
            .args([
                "--flight",
                "--warp",
                "2",
                "--flight-step",
                "0.025",
                "--frames",
                "2",
                "--width",
                "320",
                "--height",
                "180",
                "--no-hud",
                "--mute",
                "--prestream-seconds",
                "0",
                "--graphics",
            ])
            .arg(&graphics)
            .arg("--headless")
            .arg(json.with_extension("png"))
            .arg("--perf-capture")
            .arg(&json);
        if paused {
            command.arg("--paused");
        }
        let output = command.output().unwrap();
        assert!(
            output.status.success(),
            "{}",
            String::from_utf8_lossy(&output.stderr)
        );
        assert!(!String::from_utf8_lossy(&output.stderr).contains("native_audio=active"));
        let capture =
            thessa_perf::PerfCapture::from_json(&std::fs::read_to_string(json).unwrap()).unwrap();
        assert_eq!(capture.frames.len(), 2);
        for frame in &capture.frames {
            assert_eq!(frame.sim.fixed_dt_s, 1.0 / 120.0);
            assert_eq!(frame.sim.steps_this_frame, if paused { 0 } else { 6 });
            assert!(
                (frame.sim.sim_time_advanced_s - if paused { 0.0 } else { 0.05 }).abs() < 1e-12
            );
            assert_eq!(frame.sim.requested_warp, if paused { 0.0 } else { 2.0 });
            assert!(
                (frame.sim.effective_warp - frame.sim.sim_time_advanced_s / frame.frame_wall_s)
                    .abs()
                    < 1e-10
            );
        }
        assert!(
            (capture.frames[1].sim_time_s - if paused { 0.0 } else { 1.6 }).abs() < 1e-10,
            "30 warmup+2 measured deterministic frames"
        );
        println!(
            "native_clock_capture paused={paused} fixed_steps={} time_s={} samples=2 scratch={}",
            capture.frames[1].sim.steps_this_frame,
            capture.frames[1].sim_time_s,
            scratch.display()
        );
    }
}

#[test]
fn capture_cli_rejects_window_mode_and_non_json_paths_before_startup() {
    for args in [
        vec!["--perf-capture", "trace.json"],
        vec!["--headless", "image.png", "--perf-capture", "trace.csv"],
        vec!["--headless", "trace.json", "--perf-capture", "trace.json"],
    ] {
        let output = std::process::Command::new(env!("CARGO_BIN_EXE_thessa-terrain-native"))
            .args(args)
            .output()
            .unwrap();
        assert!(!output.status.success());
        assert!(String::from_utf8_lossy(&output.stderr).contains("--perf-capture requires"));
    }
}

#[test]
#[ignore = "hardware: requires a wgpu adapter, serial offscreen perf capture"]
fn native_headless_capture_exports_measured_frames_and_capability_truth() {
    for (hdr, topology) in [(false, false), (true, false), (true, true)] {
        check_capture(hdr, topology);
    }
}

fn check_capture(hdr: bool, topology: bool) {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    let scratch = root.join("target/test-tmp").join(format!(
        "native-perf-{}-{hdr}-{topology}",
        std::process::id()
    ));
    std::fs::create_dir_all(&scratch).unwrap();
    let graphics = scratch.join("graphics.toml");
    std::fs::write(&graphics,format!("version=1\n[renderer]\nhdr={hdr}\nray_tracing=\"off\"\nauto_exposure=true\nbloom=true\n[terrain_topology]\nenabled={topology}\nquality=\"low\"\n")).unwrap();
    let json = scratch.join("capture.json");
    let output = std::process::Command::new(env!("CARGO_BIN_EXE_thessa-terrain-native"))
        .current_dir(root.join("../.."))
        .arg("--graphics")
        .arg(&graphics)
        .args(["--no-hud", "--headless"])
        .arg(scratch.join("image.png"))
        .args(["--perf-capture"])
        .arg(&json)
        .args([
            "--frames",
            "3",
            "--prestream-seconds",
            "0",
            "--audio-cabin",
            "--audio-demo",
        ])
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "stdout={} stderr={}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    let capture =
        thessa_perf::PerfCapture::from_json(&std::fs::read_to_string(&json).unwrap()).unwrap();
    assert_eq!(capture.frames.len(), 3);
    assert_eq!(capture.metadata.window_frames, 3);
    assert_eq!(capture.metadata.display.width, 1280);
    assert!(capture.metadata.platform.gpu.is_some());
    let backend =
        thessa_render_wgpu::Backend::bring_up_for_graphics_profiled(&Default::default()).unwrap();
    let supported = thessa_render_wgpu::timing::GpuIntervals::new(&backend, 6).is_some();
    let names = [
        "render.surface_and_shadows",
        "render.rt_as_build",
        "render.scene_classify_and_raster",
        "render.display_transform",
        "render.ui",
    ];
    for frame in &capture.frames {
        assert!(frame.frame_cpu_s <= frame.frame_wall_s);
        assert_eq!(frame.gpu.available, supported);
        if supported {
            assert!(!frame.gpu.scopes.contains_key("render.terrain_and_sky"));
            assert!(!frame.gpu.scopes.contains_key("render.mesh_and_rt"));
            assert!(frame.gpu.frame_s.unwrap() > 0.0);
            assert!(names.iter().all(|name| {
                frame
                    .gpu
                    .scopes
                    .get(*name)
                    .is_some_and(|v| v.is_finite() && *v >= 0.0)
            }));
            let sum: f64 = frame.gpu.scopes.values().sum();
            assert!((sum - frame.gpu.frame_s.unwrap()).abs() < 1e-10);
        } else {
            assert!(frame.gpu.frame_s.is_none());
            assert!(frame.gpu.scopes.is_empty());
        }
    }
    let csv = std::fs::read_to_string(json.with_extension("csv")).unwrap();
    assert_eq!(csv.lines().count(), 4);
    if supported {
        assert!(csv.contains("gpu_scope:render.scene_classify_and_raster_ms"));
    }
    assert_eq!(
        capture
            .metadata
            .graphics
            .resolved
            .get("gpu_scope_layout")
            .map(String::as_str),
        Some("native-topology-v5-sky")
    );
    if topology
        && thessa_render_wgpu::probe_capabilities(&backend.adapter).terrain_conforming_supported
            == Some(true)
    {
        let output = String::from_utf8_lossy(&output.stdout);
        assert!(output.contains("terrain_topology=conforming"));
        assert!(
            output.contains("topology_revision=33"),
            "30 warmups + 3 measured emitted publications"
        );
    }
    let enabled = hdr
        && thessa_render_wgpu::probe_capabilities(&backend.adapter).hdr_render_target_supported
            == Some(true);
    let meta = &capture.metadata.graphics.resolved;
    assert_eq!(meta["native_hdr"], enabled.to_string());
    assert_eq!(
        meta["native_hdr_target_bytes"],
        if enabled {
            (1280u64 * 720 * 8).to_string()
        } else {
            "0".into()
        }
    );
    assert_eq!(
        meta["native_tonemapping"],
        if enabled { "reinhard" } else { "none" }
    );
    assert_eq!(meta["native_auto_exposure_applied"], "false");
    assert_eq!(meta["native_bloom_applied"], "false");
    assert_eq!(
        meta["native_scene_format"],
        if enabled {
            "Rgba16Float"
        } else {
            "Rgba8UnormSrgb"
        }
    );
}
