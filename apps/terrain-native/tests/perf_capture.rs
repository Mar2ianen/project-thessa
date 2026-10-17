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
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    let scratch = root
        .join("target/test-tmp")
        .join(format!("native-perf-{}", std::process::id()));
    std::fs::create_dir_all(&scratch).unwrap();
    let json = scratch.join("capture.json");
    let output = std::process::Command::new(env!("CARGO_BIN_EXE_thessa-terrain-native"))
        .current_dir(root.join("../.."))
        .args(["--no-hud", "--headless"])
        .arg(scratch.join("image.png"))
        .args(["--perf-capture"])
        .arg(&json)
        .args(["--frames", "3", "--prestream-seconds", "0"])
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
    let supported = thessa_render_wgpu::timing::GpuIntervals::new(&backend, 5).is_some();
    let names = [
        "render.shadows",
        "render.terrain_and_sky",
        "render.mesh_and_rt",
        "render.ui",
    ];
    for frame in &capture.frames {
        assert!(frame.frame_cpu_s <= frame.frame_wall_s);
        assert_eq!(frame.gpu.available, supported);
        if supported {
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
        assert!(csv.contains("gpu_scope:render.terrain_and_sky_ms"));
    }
}
