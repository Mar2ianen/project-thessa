//! GPU energy/occlusion/lifecycle oracles, not subjective visual approval.
use glam::{DMat4, DQuat, DVec3, Mat4, Vec3};
use thessa_plume_core::{AxialProfile, AxialStation, ExpansionRegime, integrate_ray};
use thessa_render_wgpu::{
    Backend,
    plume::{ConeDraw, PlumeFrame, PlumeRenderer, VolumeDraw},
    terrain::{OffscreenTarget, TerrainRenderer},
    view::ViewParams,
};

fn view(position: DVec3, rotation: DQuat) -> ViewParams {
    ViewParams {
        world_from_view: DMat4::from_rotation_translation(rotation, position),
        world_position_m: position,
        clip_from_view: glam::dcamera::rh::proj::directx::perspective(0.9, 1.0, 0.1, 1e6),
        exposure: 1.0,
    }
}

fn volume(steps: f32) -> VolumeDraw {
    VolumeDraw {
        origin_len: [0.0, -2.0, -10.0, 4.0],
        axis_radius: [0.0, 1.0, 0.0, 1.6],
        animation: [steps, 0.0, 0.0, 0.0],
        diamonds: true,
        tail_dissolve: false,
        profile: vec![
            [0.0, 1.0, 0.15, 1.0, 0.06, 0.03, 0.015, 0.0],
            [4.0, 1.0, 0.15, 1.0, 0.06, 0.03, 0.015, 0.0],
        ]
        .into(),
    }
}

fn render(
    backend: &Backend,
    plume: &PlumeRenderer,
    target: &OffscreenTarget,
    depth: f32,
) -> Vec<u8> {
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    {
        let _pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: &target.color,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color::TRANSPARENT),
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: &target.depth,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(depth),
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            }),
            ..Default::default()
        });
    }
    plume.encode(&mut encoder, &target.color, &target.depth);
    backend.queue.submit([encoder.finish()]);
    target.read_rgba8(backend).unwrap()
}

fn linear(byte: u8) -> f64 {
    let value = f64::from(byte) / 255.0;
    if value <= 0.04045 {
        value / 12.92
    } else {
        ((value + 0.055) / 1.055).powf(2.4)
    }
}

#[test]
#[ignore = "hardware: actual raster energy against CPU slab oracle, depth and camera lifecycle"]
fn volume_energy_matches_cpu_slab_with_depth_and_camera_lifecycle() {
    let backend = Backend::bring_up().unwrap();
    let target = OffscreenTarget::new(&backend, 65, 65).unwrap();
    let mut plume = PlumeRenderer::new(&backend, OffscreenTarget::FORMAT, true).unwrap();
    // Same axial publication and bounded chord as CPU, without residual
    // deformation/tail dissolve. Pin absolute linear RGB / alpha error.
    let station = |z_m| AxialStation {
        z_m,
        radius_m: 1.0,
        center_density_kg_m3: 1.0,
        center_temp_k: 1000.0,
        extinction_per_m: 0.15,
        emission_rgb: [0.06, 0.03, 0.015],
        shock: 1.0,
    };
    let profile = AxialProfile {
        stations: vec![station(0.0), station(4.0)],
        regime: ExpansionRegime::Matched,
        length_m: 4.0,
    };
    let (transmittance, expected) = integrate_ray(&profile, 2.0, 1.6, 256);
    let center = (32 * 65 + 32) * 4;
    for steps in [8.0, 14.0] {
        let draw = volume(steps);
        for position in [DVec3::ZERO, DVec3::new(1e12, -2e12, 3e12)] {
            plume
                .update(
                    &backend,
                    &view(position, DQuat::IDENTITY),
                    Some(PlumeFrame::Volume(draw.clone())),
                )
                .unwrap();
            assert_eq!(
                plume.stats().profile_upload_bytes,
                if position == DVec3::ZERO { 64 } else { 0 }
            );
            let bytes = render(&backend, &plume, &target, 1.0);
            for channel in 0..3 {
                let actual = linear(bytes[center + channel]);
                assert!(
                    (actual - expected[channel]).abs() < 0.003,
                    "steps={steps} c={channel} actual={actual} expected={}",
                    expected[channel]
                );
            }
            let alpha = f64::from(bytes[center + 3]) / 255.0;
            assert!(
                (alpha - (1.0 - transmittance)).abs() < 0.003,
                "alpha={alpha} CPU={}",
                1.0 - transmittance
            );
            assert!(bytes[center] > 30, "oracle must witness a real draw");
            assert!(
                render(&backend, &plume, &target, 0.0)
                    .iter()
                    .all(|v| *v == 0),
                "opaque foreground depth must occlude every plume fragment"
            );
        }
        // Identical field commit, different view: camera uniforms must still
        // update or the old plume stays in the frame after turning away.
        plume
            .update(
                &backend,
                &view(DVec3::ZERO, DQuat::from_rotation_y(std::f64::consts::PI)),
                Some(PlumeFrame::Volume(draw)),
            )
            .unwrap();
        assert!(
            render(&backend, &plume, &target, 1.0)
                .iter()
                .all(|v| *v == 0)
        );
    }
    plume
        .update(&backend, &view(DVec3::ZERO, DQuat::IDENTITY), None)
        .unwrap();
    assert!(
        render(&backend, &plume, &target, 1.0)
            .iter()
            .all(|v| *v == 0)
    );
}

#[test]
#[ignore = "hardware: finite axial/inside rays and opaque intersection within the field"]
fn axial_inside_and_partial_depth_rays_are_not_discarded() {
    let backend = Backend::bring_up().unwrap();
    let target = OffscreenTarget::new(&backend, 65, 65).unwrap();
    let camera = view(DVec3::ZERO, DQuat::IDENTITY);
    let mut plume = PlumeRenderer::new(&backend, OffscreenTarget::FORMAT, true).unwrap();
    let mut draw = volume(14.0);
    draw.origin_len = [0.0, 0.0, -5.0, 4.0];
    draw.axis_radius = [0.0, 0.0, -1.0, 1.6];
    let center = (32 * 65 + 32) * 4;
    plume
        .update(&backend, &camera, Some(PlumeFrame::Volume(draw.clone())))
        .unwrap();
    let full = render(&backend, &plume, &target, 1.0);
    assert!(
        full[center] > 70,
        "exact axial rays must integrate, not vanish"
    );
    let depth = |distance: f64| {
        let p = camera.clip_from_view * glam::DVec4::new(0.0, 0.0, -distance, 1.0);
        (p.z / p.w) as f32
    };
    let half = render(&backend, &plume, &target, depth(7.0));
    assert!(
        half[center] > 20 && half[center] < full[center],
        "opaque hit inside the field clips the integral"
    );
    assert!(
        render(&backend, &plume, &target, depth(4.0))
            .iter()
            .all(|v| *v == 0)
    );
    // Eye midway through a field that straddles the near plane. There is no
    // whiteout-prevention alpha fudge; finite Beer-Lambert transport applies.
    draw.origin_len = [0.0, 0.0, 2.0, 4.0];
    plume
        .update(&backend, &camera, Some(PlumeFrame::Volume(draw)))
        .unwrap();
    let inside = render(&backend, &plume, &target, 1.0);
    assert!(inside[center] > 20 && inside[center] < full[center]);
}

#[test]
#[ignore = "hardware: actual 48-station atmospheric/vacuum publications against CPU oracle"]
fn published_profile_matches_cpu_without_fitted_extinction_or_hue() {
    use thessa_plume_core::{
        ExhaustFamily, PlumeEnvironment, PlumeSource, RigidTransform, profile::build_axial_profile,
    };
    let backend = Backend::bring_up().unwrap();
    let target = OffscreenTarget::new(&backend, 65, 65).unwrap();
    let camera = view(DVec3::ZERO, DQuat::IDENTITY);
    let mut plume = PlumeRenderer::new(&backend, OffscreenTarget::FORMAT, true).unwrap();
    let mut max_error = 0.0_f64;
    for pressure_pa in [101_325.0, 5000.0, 0.0] {
        for throttle in [0.05, 0.3, 1.0] {
            let source = PlumeSource {
                nozzle_to_vehicle: RigidTransform::IDENTITY,
                exit_radius_m: 0.55,
                mass_flow_kg_s: 480.0,
                exhaust_velocity_mps: 3550.0,
                exit_pressure_pa: 120_000.0,
                exit_temperature_k: 1850.0,
                exit_mach: 3.5,
                throttle,
                exhaust: ExhaustFamily::Methalox,
            };
            let env = PlumeEnvironment {
                pressure_pa,
                density_kg_m3: 0.0,
                temperature_k: 280.0,
                oxygen_fraction: 0.0,
                flow_velocity_local_mps: [0.0; 3],
            };
            let mut profile = build_axial_profile(&source, &env, 48).unwrap();
            let z = profile.length_m * 0.25;
            let bound = profile
                .stations
                .iter()
                .map(|s| s.radius_m)
                .fold(0.0, f64::max)
                * 1.6;
            let (_, reference) = integrate_ray(&profile, z, bound, 256);
            // Common radiometric normalization prevents SDR saturation from
            // hiding errors in the vacuum cases. Geometry/transport unchanged.
            let normalization = 0.25 / reference.into_iter().fold(0.0, f64::max).max(1e-20);
            for s in &mut profile.stations {
                s.emission_rgb = s.emission_rgb.map(|v| v * normalization);
            }
            let (_, expected) = integrate_ray(&profile, z, bound, 256);
            let transport: std::sync::Arc<[[f32; 8]]> = profile
                .stations
                .iter()
                .map(|s| {
                    [
                        s.z_m as f32,
                        s.radius_m as f32,
                        s.extinction_per_m as f32,
                        s.shock as f32,
                        s.emission_rgb[0] as f32,
                        s.emission_rgb[1] as f32,
                        s.emission_rgb[2] as f32,
                        0.0,
                    ]
                })
                .collect();
            for steps in [8.0, 14.0] {
                let draw = VolumeDraw {
                    origin_len: [
                        0.0,
                        -z as f32,
                        -(bound * 4.0) as f32,
                        profile.length_m as f32,
                    ],
                    axis_radius: [0.0, 1.0, 0.0, bound as f32],
                    animation: [steps, 0.0, 0.0, 0.0],
                    diamonds: true,
                    tail_dissolve: false,
                    profile: transport.clone(),
                };
                plume
                    .update(&backend, &camera, Some(PlumeFrame::Volume(draw)))
                    .unwrap();
                let actual = render(&backend, &plume, &target, 1.0);
                let center = (32 * 65 + 32) * 4;
                for c in 0..3 {
                    let error = (linear(actual[center + c]) - expected[c]).abs();
                    max_error = max_error.max(error);
                    assert!(
                        error < 0.02,
                        "p={pressure_pa} throttle={throttle} steps={steps} c={c} error={error}"
                    );
                }
            }
        }
    }
    eprintln!(
        "plume_profile_oracle max_absolute_linear_rgb_error={max_error:.6} envelope=0.02 cases=18"
    );
}

#[test]
#[ignore = "hardware: Low impostor upload, alpha and zero-cost disabled resource lifecycle"]
fn impostor_draws_and_disabled_terrain_releases_the_consumer() {
    let backend = Backend::bring_up().unwrap();
    let target = OffscreenTarget::new(&backend, 65, 65).unwrap();
    let camera = view(DVec3::ZERO, DQuat::IDENTITY);
    let frame = PlumeFrame::Cone(ConeDraw {
        model: Mat4::from_translation(Vec3::new(0.0, 0.0, -10.0)).to_cols_array(),
        texture: vec![255; 32 * 256 * 4].into(),
    });
    for diamonds in [false, true] {
        let mut plume = PlumeRenderer::new(&backend, OffscreenTarget::FORMAT, diamonds).unwrap();
        assert!(plume.is_live_for(OffscreenTarget::FORMAT, diamonds));
        assert!(!plume.is_live_for(wgpu::TextureFormat::Rgba16Float, diamonds));
        plume
            .update(&backend, &camera, Some(frame.clone()))
            .unwrap();
        let bytes = render(&backend, &plume, &target, 1.0);
        assert!(bytes.as_chunks::<4>().0.iter().any(|p| p[0] > 50));
        assert_eq!(
            &bytes[..4],
            &[0; 4],
            "transparent texture edges must not emit"
        );
    }
    let anchor = thessa_rcbt_core::tile_anchor::TileAnchor::new(
        thessa_rcbt_core::tile_anchor::TileKey::new(4, 0, 0, 0).unwrap(),
        1000.0,
    )
    .unwrap();
    let page = thessa_rcbt_core::HeightPage::bake(&[0.0; 4], 2, 0.001).unwrap();
    let mut terrain = TerrainRenderer::new(
        &backend,
        &[thessa_render_wgpu::terrain::TerrainPatch {
            anchor,
            page: &page,
        }],
        OffscreenTarget::FORMAT,
    )
    .unwrap();
    let mut settings = backend.resolve_graphics(&thessa_graphics::RequestedGraphics::default());
    terrain
        .update_plume(&backend, &camera, &settings, None)
        .unwrap();
    assert!(!terrain.plume_allocated());
    terrain
        .update_plume(&backend, &camera, &settings, Some(frame.clone()))
        .unwrap();
    assert!(terrain.plume_allocated());
    terrain.publish_partition(&backend, &[]).unwrap();
    terrain.update_view(&backend, &camera).unwrap();
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    // A downstream plume forces a valid depth store even if no atmospheric
    // layer requested it. This is the real unified-scene integration path.
    terrain.encode_scene_ops(&mut encoder, &target.color, &target.depth, None, false);
    backend.queue.submit([encoder.finish()]);
    assert!(
        target
            .read_rgba8(&backend)
            .unwrap()
            .as_chunks::<4>()
            .0
            .iter()
            .any(|p| p[0] > 100)
    );
    settings.plume_enabled = false;
    terrain
        .update_plume(&backend, &camera, &settings, Some(frame))
        .unwrap();
    assert!(!terrain.plume_allocated());
    let mut hdr = PlumeRenderer::new(&backend, wgpu::TextureFormat::Rgba16Float, true).unwrap();
    hdr.update(&backend, &camera, Some(PlumeFrame::Volume(volume(14.0))))
        .unwrap();
}

#[test]
#[ignore = "hardware diagnostic: true GPU timestamps for one 1280x720 native plume consumer"]
fn plume_gpu_cost() {
    use thessa_render_wgpu::timing::GpuIntervals;
    let backend = Backend::bring_up_for_graphics_profiled(&Default::default()).unwrap();
    let Some(timing) = GpuIntervals::new(&backend, 2) else {
        eprintln!("native_plume_gpu unavailable=timestamp_capability");
        return;
    };
    let target = OffscreenTarget::new(&backend, 1280, 720).unwrap();
    let mut camera = view(DVec3::ZERO, DQuat::IDENTITY);
    camera.clip_from_view =
        glam::dcamera::rh::proj::directx::perspective(0.9, 1280.0 / 720.0, 0.1, 1e6);
    let mut plume = PlumeRenderer::new(&backend, OffscreenTarget::FORMAT, true).unwrap();
    for steps in [8.0, 14.0] {
        plume
            .update(&backend, &camera, Some(PlumeFrame::Volume(volume(steps))))
            .unwrap();
        // Initialize color/depth once, outside the timed interval. No readback
        // or CPU wall duration is reported as GPU work.
        render(&backend, &plume, &target, 1.0);
        let mut samples = Vec::new();
        for i in 0..36 {
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            timing.mark(&mut encoder, 0);
            plume.encode(&mut encoder, &target.color, &target.depth);
            timing.mark(&mut encoder, 1);
            timing.resolve(&mut encoder);
            backend.queue.submit([encoder.finish()]);
            let seconds = timing.read(&backend).unwrap()[0];
            if i >= 4 {
                samples.push(seconds);
            }
        }
        samples.sort_by(f64::total_cmp);
        eprintln!(
            "native_plume_gpu adapter={:?} sources=1 extent=1280x720 steps={steps} samples=32 median_ms={:.6} p95_ms={:.6}",
            backend.adapter_name,
            samples[16] * 1000.0,
            samples[30] * 1000.0
        );
    }
}
