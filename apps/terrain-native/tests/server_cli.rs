#[test]
fn server_mode_rejects_conflicting_or_nonfinite_options_before_scene_setup() {
    for args in [
        vec!["--server-flight", "--flight"],
        vec!["--server-flight", "--vehicle-preview"],
        vec!["--server-flight", "--site", "0"],
        vec!["--server-flight", "--sim-time", "10"],
        vec![
            "--server-flight",
            "--headless",
            "unused.png",
            "--flight-step",
            "0.01",
        ],
        vec!["--server-flight", "--server-timeout", "NaN"],
        vec!["--server-flight", "--server-timeout", "0"],
        vec!["--server-flight", "--server-timeout", "61"],
        vec!["--server", "missing"],
        vec!["--server-timeout", "2"],
    ] {
        let output = std::process::Command::new(env!("CARGO_BIN_EXE_thessa-terrain-native"))
            .args(&args)
            .output()
            .unwrap();
        assert!(!output.status.success(), "{args:?}");
        let stderr = String::from_utf8_lossy(&output.stderr);
        assert!(
            !stderr.contains("canonical Thessa"),
            "validation ran after setup: {stderr}"
        );
        assert!(stderr.contains("Error:"), "{args:?}: {stderr}");
    }
}
