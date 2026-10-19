//! Native adapter contract: reuse the portable schema, never label CPU submit
//! as GPU time; no temporary files outside the crate's ignored target tree.
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
