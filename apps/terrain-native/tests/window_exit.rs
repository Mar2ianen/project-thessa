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
