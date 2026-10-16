//! Native lifecycle regression: clipboard workers must be joined while winit's
//! borrowed Wayland display is still alive, not after run_app destroys it.
#![cfg(feature = "hud")]

#[test]
#[ignore = "requires a native Wayland/X11 display and wgpu adapter"]
fn hud_and_terrain_only_windows_exit_cleanly() {
    for hud in ["--hud", "--no-hud"] {
        let output = std::process::Command::new(env!("CARGO_BIN_EXE_thessa-terrain-native"))
            .current_dir(std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../.."))
            .args([hud, "--frames", "8"])
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "{hud}: {:?}\nstdout: {}\nstderr: {}",
            output.status,
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr),
        );
        assert!(String::from_utf8_lossy(&output.stdout).contains("window_presented_frames=8"));
    }
}

#[test]
#[ignore = "requires a native Wayland/X11 display and wgpu adapter"]
fn live_flight_windows_advance_and_exit_with_or_without_hud() {
    for hud in ["--hud", "--no-hud"] {
        let output = std::process::Command::new(env!("CARGO_BIN_EXE_thessa-terrain-native"))
            .current_dir(std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../.."))
            .args(["--flight", hud, "--frames", "16"])
            .output()
            .unwrap();
        let stdout = String::from_utf8_lossy(&output.stdout);
        assert!(
            output.status.success(),
            "{hud}: {:?}\nstdout: {stdout}\nstderr: {}",
            output.status,
            String::from_utf8_lossy(&output.stderr),
        );
        assert!(stdout.contains("window_presented_frames=16"));
        assert!(stdout.contains("stopped=false"));
        let value = |key: &str| {
            stdout
                .split_once(key)
                .unwrap()
                .1
                .split_whitespace()
                .next()
                .unwrap()
                .parse::<f64>()
                .unwrap()
        };
        let authority = value("authority_time_s=");
        assert!(authority > 0.0);
        assert_eq!(authority, value("presentation_time_s="));
    }
}
