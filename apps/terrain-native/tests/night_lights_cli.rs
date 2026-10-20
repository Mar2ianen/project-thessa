//! Actual game-owned mask extraction through the silent native host. Numerical
//! emission and coordinate oracles live in render-wgpu's primary-terrain tests.
#[test]
#[ignore = "hardware: all native night-mask tiers/disabled/zero gain, no audio, unchanged authoritative flight telemetry"]
fn native_night_masks_publish_once_without_touching_authoritative_flight() {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    let scratch = root
        .join("target/test-tmp")
        .join(format!("native-night-{}", std::process::id()));
    std::fs::create_dir_all(&scratch).unwrap();
    let mut authority = None;
    for (name, enabled, quality, intensity, bytes) in [
        ("off", false, "medium", 1.0, 0),
        ("zero", true, "high", 0.0, 0),
        ("low", true, "low", 1.0, 8192),
        ("medium", true, "medium", 1.0, 32768),
        ("high", true, "high", 1.0, 131072),
    ] {
        let graphics = scratch.join(format!("{name}.toml"));
        std::fs::write(&graphics, format!("version=1\npreset=\"custom\"\n[renderer]\nray_tracing=\"off\"\nhdr=true\n[atmosphere]\nenabled=false\nsky_dome=false\nstars=false\n[upper_atmosphere]\naurora=false\nairglow=false\n[clouds]\nenabled=false\n[ocean]\nenabled=false\n[shadows]\nenabled=false\n[engine_plume]\nenabled=false\n[night_lights]\nenabled={enabled}\nquality=\"{quality}\"\nintensity={intensity}\n")).unwrap();
        let image = scratch.join(format!("{name}.png"));
        let output = std::process::Command::new(env!("CARGO_BIN_EXE_thessa-terrain-native"))
            .current_dir(root.join("../.."))
            .args([
                "--flight",
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
            .arg(&image)
            .output()
            .unwrap();
        let stderr = String::from_utf8_lossy(&output.stderr);
        assert!(output.status.success(), "{name}: {stderr}");
        assert!(!stderr.contains("native_audio=active"));
        assert_eq!(
            stderr.matches("native_night_bake ").count(),
            usize::from(bytes > 0),
            "one startup bake, never per frame: {stderr}"
        );
        let stdout = String::from_utf8_lossy(&output.stdout);
        let state = stdout
            .lines()
            .find(|l| l.starts_with("flight_step_s="))
            .unwrap()
            .to_string();
        if let Some(previous) = &authority {
            assert_eq!(&state, previous);
        } else {
            authority = Some(state);
        }
        let counters = stdout
            .lines()
            .find(|l| l.starts_with("native_night_lights "))
            .unwrap();
        assert!(
            counters.contains(&format!("storage_bytes={bytes} uploaded_bytes={bytes}")),
            "{counters}"
        );
        if bytes == 0 {
            assert!(counters.contains("extent=None"));
        }
        assert!(image.metadata().unwrap().len() > 1000);
        eprintln!(
            "native_night_cli tier={name} {counters}; authority=identical audio=silent scratch={}",
            scratch.display()
        );
    }
}
