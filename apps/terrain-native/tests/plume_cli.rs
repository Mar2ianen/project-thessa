//! Actual silent native-host extraction and rendering, not an aesthetic test.
#[test]
#[ignore = "hardware: Low/Medium/High/disabled native CLI render and identical authoritative flight state"]
fn native_plume_tiers_render_without_mutating_flight_or_opening_audio() {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    let scratch = root
        .join("target/test-tmp")
        .join(format!("native-plume-{}", std::process::id()));
    std::fs::create_dir_all(&scratch).unwrap();
    let mut reference_state = None;
    let mut images = Vec::new();
    for (name, enabled, tier, steps) in [
        ("off", false, "medium", 0),
        ("low", true, "low", 0),
        ("medium", true, "medium", 8),
        ("high", true, "high", 14),
    ] {
        let graphics = scratch.join(format!("{name}.toml"));
        std::fs::write(&graphics, format!("version=1\npreset=\"custom\"\n[renderer]\nray_tracing=\"off\"\nhdr=false\n[atmosphere]\nenabled=false\nsky_dome=false\nstars=false\n[upper_atmosphere]\naurora=false\nairglow=false\n[clouds]\nenabled=false\n[ocean]\nenabled=false\n[shadows]\nenabled=false\n[engine_plume]\nenabled={enabled}\nquality=\"{tier}\"\nflicker=false\n")).unwrap();
        let image = scratch.join(format!("{name}.png"));
        let output = std::process::Command::new(env!("CARGO_BIN_EXE_thessa-terrain-native"))
            .current_dir(root.join("../.."))
            .args([
                "--flight",
                "--throttle",
                "0.5",
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
        let error = String::from_utf8_lossy(&output.stderr);
        assert!(output.status.success(), "{name}: {error}");
        assert!(!error.contains("native_audio=active"));
        assert!(
            error.contains(&format!(
                "native_plume allocated={enabled} draws={} march_steps={steps}",
                u32::from(enabled)
            )),
            "{error}"
        );
        let stdout = String::from_utf8_lossy(&output.stdout);
        let state = stdout
            .lines()
            .find(|line| line.starts_with("flight_step_s="))
            .unwrap()
            .to_owned();
        if let Some(reference) = &reference_state {
            assert_eq!(&state, reference);
        } else {
            reference_state = Some(state);
        }
        let mut reader = png::Decoder::new(std::io::BufReader::new(
            std::fs::File::open(&image).unwrap(),
        ))
        .read_info()
        .unwrap();
        let mut pixels = vec![0; reader.output_buffer_size().unwrap()];
        let info = reader.next_frame(&mut pixels).unwrap();
        pixels.truncate(info.buffer_size());
        images.push(pixels);
    }
    for (i, name) in ["low", "medium", "high"].into_iter().enumerate() {
        let changed = images[0]
            .as_chunks::<4>()
            .0
            .iter()
            .zip(images[i + 1].as_chunks::<4>().0)
            .filter(|(a, b)| a[..3].iter().zip(&b[..3]).any(|(a, b)| a.abs_diff(*b) > 4))
            .count();
        assert!(
            changed > 15,
            "{name}: no visible plume contribution ({changed} pixels)"
        );
        eprintln!(
            "native_plume_cli tier={name} changed_pixels={changed} authority=identical audio=silent scratch={}",
            scratch.display()
        );
    }
}
