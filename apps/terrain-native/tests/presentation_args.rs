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
        "COAST",
        "30 warmup frames",
        "not server warp",
    ] {
        assert!(text.contains(phrase), "missing help: {phrase}");
    }
}
