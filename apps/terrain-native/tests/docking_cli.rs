//! The actual native host must render solved states without opening audio.
#[test]
fn docking_rejects_mixed_authority_or_invalid_replay_before_world_loading() {
    for args in [
        vec!["--docking-demo", "--flight"],
        vec!["--docking-demo", "--map"],
        vec!["--docking-time", "2"],
        vec![
            "--docking-demo",
            "--docking-time",
            "NaN",
            "--headless",
            "unused.png",
        ],
        vec![
            "--docking-demo",
            "--docking-step",
            "1",
            "--headless",
            "unused.png",
        ],
        vec!["--docking-demo", "--sim-time", "2"],
        vec!["--docking-demo", "--vehicle-preview"],
    ] {
        let output = std::process::Command::new(env!("CARGO_BIN_EXE_thessa-terrain-native"))
            .args(args)
            .output()
            .unwrap();
        assert!(!output.status.success());
        let error = String::from_utf8_lossy(&output.stderr);
        assert!(error.contains("docking"), "{error}");
        assert!(
            !error.contains("canonical CPU scene build"),
            "must reject before loading assets"
        );
    }
}

#[test]
#[ignore = "hardware: silent native docking replay at approach, equalization and completed load/undock"]
fn native_docking_cli_renders_the_retained_cad_and_solved_two_body_replay() {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    let scratch = root
        .join("target/test-tmp")
        .join(format!("native-docking-{}", std::process::id()));
    std::fs::create_dir_all(&scratch).unwrap();
    let mut images = Vec::new();
    for (time, state, joints, finished) in [
        ("0", "SoftCapture", "0", "false"),
        ("4.5", "OuterStructureEngaged", "1", "false"),
        ("5.5", "PressureEqualized", "1", "false"),
        ("8", "Free", "0", "true"),
    ] {
        let image = scratch.join(format!("{time}.png"));
        let output = std::process::Command::new(env!("CARGO_BIN_EXE_thessa-terrain-native"))
            .current_dir(root.join("../.."))
            .args([
                "--docking-demo",
                "--docking-time",
                time,
                "--docking-step",
                "0",
                "--frames",
                "1",
                "--width",
                "640",
                "--height",
                "360",
                "--no-hud",
                "--mute",
                "--prestream-seconds",
                "0",
                "--headless",
            ])
            .arg(&image)
            .output()
            .unwrap();
        let error = String::from_utf8_lossy(&output.stderr);
        assert!(output.status.success(), "{error}");
        assert!(!error.contains("native_audio=active"));
        assert!(
            error.contains("native_plume allocated=false draws=0"),
            "fixture is not a starter engine"
        );
        let stdout = String::from_utf8_lossy(&output.stdout);
        let telemetry = stdout
            .lines()
            .find(|s| s.starts_with("native_docking "))
            .unwrap();
        for field in [
            format!("state={state}"),
            format!("joints={joints}"),
            format!("finished={finished}"),
        ] {
            assert!(telemetry.contains(&field), "{telemetry}");
        }
        assert!(
            stdout.contains("mesh_submitted=MeshDrawStats { instances: 2,"),
            "{stdout}"
        );
        let mut reader = png::Decoder::new(std::io::BufReader::new(
            std::fs::File::open(&image).unwrap(),
        ))
        .read_info()
        .unwrap();
        let mut pixels = vec![0; reader.output_buffer_size().unwrap()];
        let info = reader.next_frame(&mut pixels).unwrap();
        pixels.truncate(info.buffer_size());
        images.push(pixels);
        eprintln!("{telemetry}; audio=silent scratch={}", scratch.display());
    }
    let changed = images[0]
        .as_chunks::<4>()
        .0
        .iter()
        .zip(images[3].as_chunks::<4>().0)
        .filter(|(a, b)| a[..3].iter().zip(&b[..3]).any(|(a, b)| a.abs_diff(*b) > 8))
        .count();
    assert!(
        changed > 100,
        "solved approach/load must visibly move the craft: {changed}"
    );
    eprintln!(
        "native_docking_cli changed_pixels={changed}; deterministic solved-state evidence, not aesthetic acceptance"
    );
    #[cfg(feature = "hud")]
    {
        let image = scratch.join("inspector.png");
        let output = std::process::Command::new(env!("CARGO_BIN_EXE_thessa-terrain-native"))
            .current_dir(root.join("../.."))
            .args([
                "--docking-demo",
                "--docking-time",
                "5.5",
                "--docking-step",
                "0",
                "--frames",
                "1",
                "--width",
                "960",
                "--height",
                "540",
                "--hud",
                "--mute",
                "--prestream-seconds",
                "0",
                "--headless",
            ])
            .arg(&image)
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "{}",
            String::from_utf8_lossy(&output.stderr)
        );
        let stdout = String::from_utf8_lossy(&output.stdout);
        assert!(stdout.contains("state=PressureEqualized") && stdout.contains("joints=1"));
        assert!(image.metadata().unwrap().len() > 1000);
    }
}
