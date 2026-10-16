//! Native lifecycle regression: clipboard workers must be joined while winit's
//! borrowed Wayland display is still alive, not after run_app destroys it.
#![cfg(feature = "hud")]

#[test]
#[ignore = "requires a native display, wgpu adapter and built sibling thessa-server"]
fn server_flight_window_adopts_snapshots_and_exits_cleanly() {
    let output = std::process::Command::new(env!("CARGO_BIN_EXE_thessa-terrain-native"))
        .current_dir(std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../.."))
        .args([
            "--server-flight",
            "--chase",
            "--throttle",
            "0",
            "--hud",
            "--frames",
            "90",
        ])
        .output()
        .unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(
        output.status.success(),
        "stdout: {stdout}\nstderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    for expected in [
        "window_presented_frames=90",
        "source=server",
        "local_steps=0",
        "stopped=false",
        "throttle=0.000",
        "link_closed=false",
    ] {
        assert!(stdout.contains(expected), "missing {expected}: {stdout}");
    }
    let number = |key: &str| {
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
    assert!(number("received_snapshots=") > 1.0);
    assert!(number("authority_time_s=") > 0.0);
    assert!(number("presentation_time_s=") <= number("authority_time_s="));
}

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
fn live_flight_throttle_preset_reaches_the_authority() {
    let output = std::process::Command::new(env!("CARGO_BIN_EXE_thessa-terrain-native"))
        .current_dir(std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../.."))
        .args([
            "--flight",
            "--throttle",
            "1",
            "--engine-on",
            "--hud",
            "--frames",
            "16",
        ])
        .output()
        .unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(
        output.status.success(),
        "--throttle: {:?}\nstdout: {stdout}\nstderr: {}",
        output.status,
        String::from_utf8_lossy(&output.stderr),
    );
    assert!(stdout.contains("window_presented_frames=16"));
    assert!(stdout.contains("throttle=1.000"));
    assert!(stdout.contains("engine=true"));
    assert!(stdout.contains("stopped=false"));
}

#[test]
#[ignore = "requires a native Wayland/X11 display and wgpu adapter"]
fn live_flight_chase_windows_advance_and_exit() {
    let output = std::process::Command::new(env!("CARGO_BIN_EXE_thessa-terrain-native"))
        .current_dir(std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../.."))
        .args(["--flight", "--chase", "--hud", "--frames", "16"])
        .output()
        .unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(
        output.status.success(),
        "--chase: {:?}\nstdout: {stdout}\nstderr: {}",
        output.status,
        String::from_utf8_lossy(&output.stderr),
    );
    assert!(stdout.contains("window_presented_frames=16"));
    assert!(stdout.contains("stopped=false"));
    assert!(stdout.contains("chase=true"));
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
