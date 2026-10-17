//! Sky-background numerics and fixed-source cost. No image-preference claim.
use glam::{DMat4, DQuat, DVec3};
use thessa_rcbt_core::{
    HeightPage,
    tile_anchor::{TileAnchor, TileKey},
};
use thessa_render_core::environment::EnvironmentRadiance;
use thessa_render_wgpu::{
    Backend, BackendError,
    environment::EnvironmentBinding,
    terrain::{OffscreenTarget, TerrainPatch, TerrainRenderer},
    view::ViewParams,
};

fn backend() -> Option<Backend> {
    match Backend::bring_up() {
        Ok(b) => Some(b),
        Err(BackendError::NoAdapter) => {
            eprintln!("SKIP: no GPU adapter");
            None
        }
        Err(e) => panic!("{e}"),
    }
}

fn renderer(backend: &Backend, empty: bool) -> (TerrainRenderer, TileAnchor) {
    let anchor = TileAnchor::new(TileKey::new(4, 10, 512, 512).unwrap(), 6_000_000.0).unwrap();
    let page = HeightPage::bake(&[0.0; 4], 2, 0.001).unwrap();
    let mut terrain = TerrainRenderer::new(
        backend,
        &[TerrainPatch {
            anchor,
            page: &page,
        }],
        OffscreenTarget::FORMAT,
    )
    .unwrap();
    if empty {
        terrain.publish_partition(backend, &[]).unwrap();
    }
    (terrain, anchor)
}

fn field(width: u32, height: u32) -> EnvironmentRadiance {
    EnvironmentRadiance::new(
        width,
        height,
        (0..width * height)
            .map(|i| {
                let x = (i % width) as f32 / width as f32;
                let y = (i / width) as f32 / height as f32;
                [0.03 + 0.7 * x, 0.05 + 0.5 * y, 0.1 + 0.3 * x * y]
            })
            .collect(),
    )
    .unwrap()
}

fn view(rotation: DQuat, position: DVec3, exposure: f32) -> ViewParams {
    ViewParams {
        world_from_view: DMat4::from_rotation_translation(rotation, position),
        world_position_m: position,
        clip_from_view: glam::dcamera::rh::proj::directx::perspective(1.1, 65.0 / 33.0, 0.1, 1e6),
        exposure,
    }
}

fn draw(
    backend: &Backend,
    terrain: &mut TerrainRenderer,
    target: &OffscreenTarget,
    view: &ViewParams,
) -> Vec<u8> {
    terrain.update_view(backend, view).unwrap();
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    terrain.encode(&mut encoder, &target.color, &target.depth);
    backend.queue.submit([encoder.finish()]);
    target.read_rgba8(backend).unwrap()
}

fn srgb(value: f32) -> u8 {
    let v = if value <= 0.003_130_8 {
        12.92 * value
    } else {
        1.055 * value.powf(1.0 / 2.4) - 0.055
    };
    (v.clamp(0.0, 1.0) * 255.0).round() as u8
}

#[test]
fn sky_matches_cpu_directional_sampler_under_rotation_projection_and_large_translation() {
    let Some(backend) = backend() else {
        return;
    };
    let (mut terrain, _) = renderer(&backend, true);
    let target = OffscreenTarget::new(&backend, 65, 33).unwrap();
    let mut binding = EnvironmentBinding::new(&backend.device);
    for (width, height) in [(8, 4), (16, 8), (32, 16), (8, 4)] {
        let grid = field(width, height);
        binding.update(&backend, Some(&grid));
        terrain.share_environment(&backend, Some(&binding));
        assert!(terrain.sky_enabled());
        for (rotation, position, infinite) in [
            (DQuat::IDENTITY, DVec3::ZERO, false),
            (
                DQuat::from_rotation_y(2.9) * DQuat::from_rotation_x(0.6),
                DVec3::new(1e12, -2e11, 7e10),
                false,
            ),
            (DQuat::from_rotation_x(-1.5), DVec3::ZERO, true),
        ] {
            let mut camera = view(rotation, position, 0.7);
            if infinite {
                camera.clip_from_view =
                    glam::dcamera::rh::proj::directx::perspective_infinite(1.1, 65.0 / 33.0, 0.1);
            }
            let pixels = draw(&backend, &mut terrain, &target, &camera);
            for (i, pixel) in pixels.as_chunks::<4>().0.iter().enumerate() {
                let ndc = glam::DVec4::new(
                    2.0 * (i % 65) as f64 / 65.0 + 1.0 / 65.0 - 1.0,
                    1.0 - 2.0 * (i / 65) as f64 / 33.0 - 1.0 / 33.0,
                    0.5,
                    1.0,
                );
                let p = camera.clip_from_view.inverse() * ndc;
                let dir = rotation * (p.truncate() / p.w).normalize();
                let expected = grid
                    .sample(dir.as_vec3().to_array())
                    .map(|v| srgb(v * camera.exposure));
                for c in 0..3 {
                    assert!(
                        pixel[c].abs_diff(expected[c]) <= 2,
                        "{width}x{height} pixel={i} c={c} actual={pixel:?} expected={expected:?}"
                    );
                }
                assert_eq!(pixel[3], 255);
            }
        }
    }
    assert_eq!(binding.uploads(), 4, "camera motion never uploads the grid");
}

#[test]
fn background_does_not_overwrite_terrain_and_disabling_restores_exact_fallback() {
    let Some(backend) = backend() else {
        return;
    };
    let (mut terrain, anchor) = renderer(&backend, false);
    let center = DVec3::from(anchor.anchor_body_m);
    let eye = center + DVec3::from(anchor.normal) * 7000.0;
    let mut camera = view(DQuat::IDENTITY, eye, 0.5);
    camera.world_from_view = glam::dcamera::rh::view::look_at_mat4(eye, center, DVec3::Y).inverse();
    let target = OffscreenTarget::new(&backend, 65, 33).unwrap();
    let baseline = draw(&backend, &mut terrain, &target, &camera);
    let mut binding = EnvironmentBinding::new(&backend.device);
    let grid = EnvironmentRadiance::new(2, 2, vec![[0.8, 0.02, 0.1]; 4]).unwrap();
    binding.update(&backend, Some(&grid));
    terrain.share_environment(&backend, Some(&binding));
    let sky = draw(&backend, &mut terrain, &target, &camera);
    let clear = baseline[..4].to_vec();
    let mut covered = 0;
    let mut background = 0;
    for (before, after) in baseline
        .as_chunks::<4>()
        .0
        .iter()
        .zip(sky.as_chunks::<4>().0)
    {
        if before.as_slice() == clear {
            assert_ne!(before, after);
            background += 1;
        } else {
            assert_eq!(before, after, "sky overwrote foreground depth");
            covered += 1;
        }
    }
    assert!(
        covered > 100 && background > 100,
        "covered={covered} background={background}"
    );
    binding.update(&backend, None);
    terrain.share_environment(&backend, Some(&binding));
    assert!(!terrain.sky_enabled());
    assert_eq!(binding.storage_bytes(), 0);
    assert_eq!(draw(&backend, &mut terrain, &target, &camera), baseline);
}

#[test]
#[ignore = "hardware: fixed-source sky background cost"]
fn fixed_source_sky_background_cost() {
    let backend = Backend::bring_up().expect("GPU adapter");
    let (mut terrain, _) = renderer(&backend, true);
    let target = OffscreenTarget::new(&backend, 1280, 720).unwrap();
    terrain
        .update_view(&backend, &view(DQuat::IDENTITY, DVec3::ZERO, 0.5))
        .unwrap();
    let mut binding = EnvironmentBinding::new(&backend.device);
    binding.update(&backend, Some(&field(32, 16)));
    for repeat in 0..3 {
        for enabled in if repeat % 2 == 0 {
            [false, true]
        } else {
            [true, false]
        } {
            terrain.share_environment(&backend, enabled.then_some(&binding));
            let mut times = Vec::new();
            for frame in 0..150 {
                let start = std::time::Instant::now();
                let mut encoder = backend.device.create_command_encoder(&Default::default());
                terrain.encode(&mut encoder, &target.color, &target.depth);
                backend.queue.submit([encoder.finish()]);
                backend
                    .device
                    .poll(wgpu::PollType::wait_indefinitely())
                    .unwrap();
                if frame >= 30 {
                    times.push(start.elapsed().as_secs_f64() * 1000.0);
                }
            }
            times.sort_by(f64::total_cmp);
            eprintln!(
                "fixed_sky adapter={} repeat={repeat} enabled={enabled} resolution=1280x720 grid=32x16 measured=120 median_ms={:.6} p95_ms={:.6} uploads={} storage_bytes={}; CPU encode/submit + GPU completion, no readback or CPU scattering; full uncovered frame worst-case sky fill",
                backend.adapter_name,
                times[60],
                times[114],
                binding.uploads(),
                binding.storage_bytes()
            );
        }
    }
    assert_eq!(binding.uploads(), 1);
}
