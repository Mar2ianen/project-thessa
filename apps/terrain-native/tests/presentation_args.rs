//! Presentation CLI rejects invalid samples before loading assets or a GPU.
use std::process::Command;

#[test]
fn rejects_nonfinite_or_window_stepped_samples_before_loading_graphics() {
    for (args, message) in [
        (
            vec!["--sim-time", "NaN"],
            "presentation sample times must be finite",
        ),
        (
            vec!["--sim-time", "inf"],
            "presentation sample times must be finite",
        ),
        (
            vec!["--sim-step", "NaN"],
            "presentation sample times must be finite",
        ),
        (
            vec!["--sim-time", "1e308", "--sim-step", "1e308"],
            "presentation sample times must be finite",
        ),
        (vec!["--sim-step", "1"], "require --headless"),
        (vec!["--flight", "--sim-time", "1"], "--flight requires"),
        (vec!["--flight", "--vehicle-preview"], "--flight requires"),
        (vec!["--chase"], "--chase requires"),
        (vec!["--flyby"], "--flyby requires"),
        (vec!["--flight", "--flyby", "--paused"], "--flyby requires"),
        (
            vec!["--flight", "--flyby", "--warp", "4"],
            "--flyby requires",
        ),
        (vec!["--flight", "--flyby", "--map"], "--flyby requires"),
        (vec!["--site"], "--site requires"),
        (vec!["--site", "3"], "--site requires"),
        (vec!["--site", "0", "--flight"], "--site is survey-only"),
        (
            vec!["--site", "1", "--vehicle-preview"],
            "--site is survey-only",
        ),
        (vec!["--throttle", "0.5"], "--throttle requires"),
        (vec!["--engine-on"], "--engine-on requires"),
        (
            vec!["--flight", "--headless", "unused.png", "--throttle", "NaN"],
            "--throttle must be finite",
        ),
        (
            vec!["--flight", "--headless", "unused.png", "--throttle", "2"],
            "--throttle must be finite",
        ),
        (
            vec!["--flight", "--headless", "unused.png", "--throttle", "-0.5"],
            "--throttle must be finite",
        ),
        (vec!["--flight-step", "0.01", "--chase"], "--chase requires"),
        (vec!["--flight-step", "0.01"], "--flight-step requires"),
        (
            vec!["--flight", "--flight-step", "0.01"],
            "--flight-step requires",
        ),
        (
            vec![
                "--flight",
                "--headless",
                "unused.png",
                "--flight-step",
                "NaN",
            ],
            "--flight-step must be finite",
        ),
        (
            vec!["--flight", "--headless", "unused.png", "--flight-step", "0"],
            "--flight-step must be finite",
        ),
        (
            vec![
                "--flight",
                "--headless",
                "unused.png",
                "--flight-step",
                "-1",
            ],
            "--flight-step must be finite",
        ),
        (
            vec![
                "--flight",
                "--headless",
                "unused.png",
                "--flight-step",
                "0.2",
            ],
            "--flight-step must be finite",
        ),
        (
            vec!["--vehicle-preview", "--sim-time", "1"],
            "--vehicle-preview requires epoch",
        ),
        (
            vec!["--vehicle-preview", "--sim-step", "1"],
            "--vehicle-preview requires epoch",
        ),
        (vec!["--sim-time"], "--sim-time requires seconds"),
        (vec!["--sim-step"], "--sim-step requires seconds per frame"),
    ] {
        let output = Command::new(env!("CARGO_BIN_EXE_thessa-terrain-native"))
            .args(["--graphics", "not-a-graphics-config-do-not-create.toml"])
            .args(&args)
            .output()
            .unwrap();
        assert!(!output.status.success(), "unexpected success: {args:?}");
        assert!(
            String::from_utf8_lossy(&output.stderr).contains(message),
            "{args:?}: {}",
            String::from_utf8_lossy(&output.stderr)
        );
    }
}

#[test]
fn help_exposes_presentation_sampling_and_warmup_boundary() {
    let output = Command::new(env!("CARGO_BIN_EXE_thessa-terrain-native"))
        .arg("--help")
        .output()
        .unwrap();
    assert!(output.status.success());
    let text = String::from_utf8_lossy(&output.stdout);
    for phrase in [
        "--sim-time",
        "--sim-step",
        "--flight",
        "--flight-step",
        "--chase",
        "chase camera",
        "--throttle",
        "--engine-on",
        "--flyby",
        "F7",
        "without auto-resume",
        "WASDQE",
        "COAST",
        "--site",
        "F6",
        "Shift-right-drag",
        "30 warmup frames",
        "not server warp",
    ] {
        assert!(text.contains(phrase), "missing help: {phrase}");
    }
}

#[test]
#[ignore = "requires a wgpu adapter; silent headless CLI flight fixture"]
fn flyby_cli_flies_the_real_native_authority_without_audio_devices() {
    let directory =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("target/test-tmp/flyby-cli");
    std::fs::create_dir_all(&directory).unwrap();
    let image = directory.join("flight.png");
    let output = Command::new(env!("CARGO_BIN_EXE_thessa-terrain-native"))
        .current_dir(std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../.."))
        .args([
            "--flight",
            "--flyby",
            "--throttle",
            "0",
            "--hud",
            "--mute",
            "--headless",
        ])
        .arg(&image)
        .args([
            "--frames",
            "2",
            "--flight-step",
            "0.025",
            "--prestream-seconds",
            "0.1",
        ])
        .output()
        .unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(output.status.success(), "{stdout}\n{stderr}");
    for expected in [
        "flyby_active=true",
        "source=local",
        "chase=true",
        "control_mode=Navball",
        "engine=true",
        "stopped=false",
        "capacity_rejections=0",
    ] {
        assert!(stdout.contains(expected), "missing {expected}: {stdout}");
    }
    let throttle: f64 = stdout
        .split_once(" throttle=")
        .unwrap()
        .1
        .split_whitespace()
        .next()
        .unwrap()
        .parse()
        .unwrap();
    assert!(
        throttle > 0.0 && throttle < 1.0,
        "slew must come from ordinary pilot intent"
    );
    assert!(image.is_file());
    assert!(
        !stderr.contains("native_audio device="),
        "captures must never open audio devices"
    );
}
