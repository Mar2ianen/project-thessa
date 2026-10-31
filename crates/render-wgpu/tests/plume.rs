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
        radial_reference: [1.0, 0.0, 0.0],
        centerline: std::sync::Arc::from([]),
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

fn clear(
    encoder: &mut wgpu::CommandEncoder,
    color: &wgpu::TextureView,
    depth: &wgpu::TextureView,
    depth_value: f32,
) {
    {
        let _pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: color,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color::TRANSPARENT),
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: depth,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(depth_value),
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            }),
            ..Default::default()
        });
    }
}

fn render(
    backend: &Backend,
    plume: &PlumeRenderer,
    target: &OffscreenTarget,
    depth: f32,
) -> Vec<u8> {
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    clear(&mut encoder, &target.color, &target.depth, depth);
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

fn hdr_target(backend: &Backend, width: u32, height: u32) -> wgpu::Texture {
    backend.device.create_texture(&wgpu::TextureDescriptor {
        label: Some("plume-exposure-oracle"),
        size: wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba16Float,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
        view_formats: &[],
    })
}

fn half(v: u16) -> f64 {
    assert_eq!(v & 0x8000, 0, "negative HDR");
    let exponent = (v >> 10) & 31;
    assert_ne!(exponent, 31, "non-finite HDR");
    let fraction = f64::from(v & 1023);
    if exponent == 0 {
        fraction * 2.0_f64.powi(-24)
    } else {
        (1024.0 + fraction) * 2.0_f64.powi(i32::from(exponent) - 25)
    }
}

#[test]
#[ignore = "hardware: Low and Volume HDR pre-exposure follows camera without changing opacity"]
fn both_representations_follow_scene_exposure_with_cached_fields() {
    let backend = Backend::bring_up().unwrap();
    eprintln!(
        "plume_exposure adapter={:?} backend={:?}",
        backend.adapter_name,
        backend.adapter.get_info().backend
    );
    let target = OffscreenTarget::new(&backend, 65, 65).unwrap();
    let texture = hdr_target(&backend, 65, 65);
    let color = texture.create_view(&Default::default());
    let mut camera = view(DVec3::ZERO, DQuat::IDENTITY);
    let cone = PlumeFrame::Cone(ConeDraw {
        model: Mat4::from_translation(Vec3::new(0.0, 0.0, -10.0)).to_cols_array(),
        texture: [64, 128, 192, 128].repeat(32 * 256).into(),
    });
    let mut witnessed_hdr = false;
    for frame in [PlumeFrame::Volume(volume(14.0)), cone] {
        let mut plume =
            PlumeRenderer::new(&backend, wgpu::TextureFormat::Rgba16Float, true).unwrap();
        let mut baseline = Vec::new();
        for gain in [1.0, 0.25, 0.0, 4.0] {
            camera.exposure = gain;
            plume
                .update(&backend, &camera, Some(frame.clone()))
                .unwrap();
            if gain != 1.0 {
                assert_eq!(plume.stats().profile_upload_bytes, 0);
                assert_eq!(plume.stats().texture_upload_bytes, 0);
            }
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            clear(&mut encoder, &color, &target.depth, 1.0);
            plume.encode(&mut encoder, &color, &target.depth);
            backend.queue.submit([encoder.finish()]);
            let bytes = thessa_render_wgpu::readback::read_color(&backend, &texture).unwrap();
            let words: Vec<_> = bytes
                .as_chunks::<2>()
                .0
                .iter()
                .map(|b| u16::from_le_bytes([b[0], b[1]]))
                .collect();
            if gain == 1.0 {
                assert!(
                    words
                        .as_chunks::<4>()
                        .0
                        .iter()
                        .any(|p| p[0] != 0 && p[3] != 0)
                );
                baseline = words;
                continue;
            }
            for (pixel, reference) in words
                .as_chunks::<4>()
                .0
                .iter()
                .zip(baseline.as_chunks::<4>().0.iter())
            {
                assert_eq!(pixel[3], reference[3], "exposure must preserve opacity");
                for c in 0..3 {
                    let expected = half(reference[c]) * f64::from(gain);
                    let actual = half(pixel[c]);
                    witnessed_hdr |= actual > 1.0;
                    assert!(
                        (actual - expected).abs() <= expected.abs() * 0.002 + 1e-6,
                        "gain={gain} channel={c} actual={actual} expected={expected}"
                    );
                }
            }
        }
    }
    assert!(
        witnessed_hdr,
        "HDR radiance must survive without SDR clipping"
    );
}

#[test]
#[ignore = "hardware: homogeneous emitting/absorbing axial chord versus closed-form transport"]
fn homogeneous_axial_transport_is_independent_of_step_budget() {
    let backend = Backend::bring_up().unwrap();
    eprintln!(
        "plume_segment adapter={:?} backend={:?}",
        backend.adapter_name,
        backend.adapter.get_info().backend
    );
    let target = OffscreenTarget::new(&backend, 65, 65).unwrap();
    let hdr = hdr_target(&backend, 65, 65);
    let color = hdr.create_view(&Default::default());
    let camera = view(DVec3::ZERO, DQuat::IDENTITY);
    let mut plume = PlumeRenderer::new(&backend, wgpu::TextureFormat::Rgba16Float, true).unwrap();
    let mut max_error = 0.0_f64;
    for optical_depth in [0.0_f64, 1e-8, 1e-4, 0.01, 0.1, 1.0, 10.0, 100.0] {
        for steps in [1.0, 8.0, 14.0, 16.0] {
            let mut draw = volume(steps);
            draw.origin_len = [0.0, 0.0, -5.0, 4.0];
            draw.axis_radius = [0.0, 0.0, -1.0, 1.6];
            for s in std::sync::Arc::make_mut(&mut draw.profile) {
                s[2] = (optical_depth / (4.0 * 1.18)) as f32;
                s[4..7].copy_from_slice(&[0.3, 0.15, 0.07]);
            }
            plume
                .update(&backend, &camera, Some(PlumeFrame::Volume(draw)))
                .unwrap();
            for length in [4.0_f64, 2.0] {
                let p = camera.clip_from_view * glam::DVec4::new(0.0, 0.0, -(5.0 + length), 1.0);
                let depth = if length == 4.0 {
                    1.0
                } else {
                    (p.z / p.w) as f32
                };
                let mut encoder = backend.device.create_command_encoder(&Default::default());
                clear(&mut encoder, &color, &target.depth, depth);
                plume.encode(&mut encoder, &color, &target.depth);
                backend.queue.submit([encoder.finish()]);
                let bytes = thessa_render_wgpu::readback::read_color(&backend, &hdr).unwrap();
                let index = (32 * 65 + 32) * 8;
                let actual: [f64; 4] = std::array::from_fn(|c| {
                    half(u16::from_le_bytes(
                        bytes[index + c * 2..index + c * 2 + 2].try_into().unwrap(),
                    ))
                });
                let tau = optical_depth * length / 4.0;
                let alpha = -(-tau).exp_m1();
                let factor = if tau == 0.0 {
                    length
                } else {
                    length * alpha / tau
                };
                let expected = [
                    0.3 * 1.18 * factor,
                    0.15 * 1.18 * factor,
                    0.07 * 1.18 * factor,
                    alpha,
                ];
                for c in 0..4 {
                    let error = (actual[c] - expected[c]).abs();
                    max_error = max_error.max(error);
                    assert!(
                        error <= expected[c].abs() * 0.002 + 2e-6,
                        "tau={tau} steps={steps} length={length} channel={c} actual={} expected={}",
                        actual[c],
                        expected[c]
                    );
                }
            }
        }
    }
    eprintln!("plume_segment cases=64 max_absolute_rgba_error={max_error}");
}

#[test]
#[ignore = "hardware: frozen analytic volume controls for an independent Cycles integrator; new THESSA_BENCH_OUTPUT required"]
fn frozen_plume_volume_reference() {
    let out = std::path::PathBuf::from(
        std::env::var_os("THESSA_BENCH_OUTPUT").expect("output directory required"),
    );
    std::fs::create_dir(&out).unwrap();
    let backend = Backend::bring_up().unwrap();
    let target = OffscreenTarget::new(&backend, 129, 129).unwrap();
    let hdr = hdr_target(&backend, 129, 129);
    let color = hdr.create_view(&Default::default());
    let camera = view(DVec3::ZERO, DQuat::IDENTITY);
    let mut plume = PlumeRenderer::new(&backend, wgpu::TextureFormat::Rgba16Float, true).unwrap();
    for (name, axial, extinction) in [
        ("axial-zero", true, 0.0),
        ("axial-one", true, 1.0 / (4.0 * 1.18)),
        ("axial-ten", true, 10.0 / (4.0 * 1.18)),
        ("side", false, 0.15),
    ] {
        let mut draw = volume(14.0);
        if axial {
            draw.origin_len = [0.0, 0.0, -5.0, 4.0];
            draw.axis_radius = [0.0, 0.0, -1.0, 1.6];
        }
        for s in std::sync::Arc::make_mut(&mut draw.profile) {
            s[2] = extinction;
            s[4..7].copy_from_slice(&[0.3, 0.15, 0.07]);
        }
        plume
            .update(&backend, &camera, Some(PlumeFrame::Volume(draw.clone())))
            .unwrap();
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        clear(&mut encoder, &color, &target.depth, 1.0);
        plume.encode(&mut encoder, &color, &target.depth);
        backend.queue.submit([encoder.finish()]);
        let directory = out.join(name);
        std::fs::create_dir(&directory).unwrap();
        std::fs::write(
            directory.join("linear.rgba16f"),
            thessa_render_wgpu::readback::read_color(&backend, &hdr).unwrap(),
        )
        .unwrap();
        let metadata = format!(
            "{{\"id\":\"analytic-plume-volume-v1\",\"case\":\"{name}\",\"extent\":[129,129],\"units\":\"metres\",\"origin_len\":{:?},\"axis_radius\":{:?},\"profile\":{:?},\"steps\":14,\"exposure_gain\":1.0,\"erosion\":0,\"tail_dissolve\":false,\"diamonds\":true,\"world_from_view\":{:?},\"clip_from_view\":{:?},\"backend\":\"{:?}\"}}\n",
            draw.origin_len,
            draw.axis_radius,
            draw.profile,
            camera.world_from_view.to_cols_array(),
            camera.clip_from_view.to_cols_array(),
            backend.adapter.get_info().backend
        );
        std::fs::write(directory.join("scene.json"), metadata).unwrap();
    }
    eprintln!(
        "plume_volume_reference adapter={:?} backend={:?} cases=4 extent=129x129 source=analytic-control-not-a-calibrated-engine",
        backend.adapter_name,
        backend.adapter.get_info().backend
    );
}

/// Model-generated fields are transport controls, not calibrated engine firings.
#[test]
#[ignore = "hardware: compiled-engine pressure/throttle/view gallery; new THESSA_BENCH_OUTPUT required"]
fn frozen_compiled_plume_reference() {
    compiled_reference(false);
}

#[test]
#[ignore = "hardware: three curved compiled-engine controls for stock Cycles; new output required"]
fn frozen_curved_plume_reference() {
    compiled_reference(true);
}

fn compiled_reference(curved: bool) {
    use thessa_plume_core::{
        PlumeEnvironment, RigidTransform,
        source::{PropellantFamily, engine_plume_source},
    };
    use thessa_propulsion::{
        ChamberMaterial, CompiledEngine, CoolingMode, EngineCycle, LiquidEngineSpec, NozzleContour,
        Propellant,
    };
    let out = std::path::PathBuf::from(
        std::env::var_os("THESSA_BENCH_OUTPUT").expect("output directory required"),
    );
    std::fs::create_dir(&out).unwrap();
    let spec = LiquidEngineSpec {
        name: "compiled-methalox-transport-control".into(),
        propellant: Propellant::LoxMethane,
        cycle: EngineCycle::GasGenerator,
        chamber_pressure_pa: 9.7e6,
        throat_radius_m: 0.08,
        expansion_ratio: 18.0,
        nozzle_length_m: 0.9,
        contour: NozzleContour::Bell,
        chamber_material: ChamberMaterial::nickel_superalloy(),
        cooling: CoolingMode::Regenerative,
        mixture_ratio: None,
        characteristic_length_m: None,
        gimbal_range_rad: 0.0,
        min_throttle: None,
        restartable: true,
    };
    let engine = CompiledEngine::Liquid(spec.compile().unwrap());
    let backend = Backend::bring_up().unwrap();
    let target = OffscreenTarget::new(&backend, 129, 129).unwrap();
    let hdr = hdr_target(&backend, 129, 129);
    let color = hdr.create_view(&Default::default());
    let camera = view(DVec3::ZERO, DQuat::IDENTITY);
    let mut renderer =
        PlumeRenderer::new(&backend, wgpu::TextureFormat::Rgba16Float, true).unwrap();
    for (name, pressure, throttle, angle) in [
        ("sea-full", Some(101_325.0), 1.0, 0),
        ("sea-half", Some(101_325.0), 0.5, 0),
        ("matched-full", None, 1.0, 0),
        ("matched-half", None, 0.5, 0),
        ("thin-full", Some(1_000.0), 1.0, 0),
        ("thin-half", Some(1_000.0), 0.5, 0),
        ("matched-axial", None, 1.0, 1),
        ("matched-inside", None, 1.0, 2),
        ("matched-oblique", None, 1.0, 3),
    ] {
        if curved && !["sea-full", "thin-full", "matched-inside"].contains(&name) {
            continue;
        }
        let ambient = pressure.unwrap_or_else(|| {
            engine
                .operating_point(throttle, 0.0, 0.0)
                .unwrap()
                .exit_pressure_pa
        });
        let point = engine.operating_point(throttle, ambient, 0.0).unwrap();
        let state = engine.plume_state(&point);
        let source = engine_plume_source(
            RigidTransform::IDENTITY,
            state.exit_radius_m,
            state.mass_flow_kg_s,
            state.exhaust_velocity_mps,
            state.exit_pressure_pa,
            state.exit_temp_k,
            state.exit_mach,
            1.0,
            PropellantFamily::Methalox,
        );
        let env = PlumeEnvironment {
            pressure_pa: ambient,
            density_kg_m3: ambient / (287.05 * 288.15),
            temperature_k: 288.15,
            oxygen_fraction: 0.0,
            flow_velocity_local_mps: [0.0; 3],
        };
        let profile = thessa_plume_core::profile::build_axial_profile(&source, &env, 48).unwrap();
        let first = profile.stations[0];
        let expected_density = point.mass_flow_kg_s
            / (point.exhaust_velocity_mps * std::f64::consts::PI * source.exit_radius_m.powi(2));
        assert!(
            (first.center_density_kg_m3 / expected_density - 1.0).abs() < 1e-12,
            "actual flow must not be throttled twice"
        );
        let len = profile.length_m as f32;
        let radius = profile
            .stations
            .iter()
            .map(|s| s.radius_m as f32)
            .fold(0.0, f32::max)
            * 1.6;
        let distance = len.max(radius * 2.0) * 1.5;
        let (origin, axis) = match angle {
            1 => ([0.0, 0.0, -distance], [0.0, 0.0, -1.0]),
            2 => ([0.0, 0.0, len * 0.5], [0.0, 0.0, -1.0]),
            3 => ([0.0, -0.3 * len, -distance + 0.4 * len], [0.0, 0.6, -0.8]),
            _ => ([0.0, -len * 0.5, -distance], [0.0, 1.0, 0.0]),
        };
        let centerline = if curved {
            let mut motion_env = env;
            motion_env.flow_velocity_local_mps = [450.0, -180.0, 0.0];
            let rotation = DQuat::from_rotation_arc(
                DVec3::Z,
                DVec3::new(axis[0] as f64, axis[1] as f64, axis[2] as f64),
            );
            thessa_plume_core::motion::centerline(
                &source,
                &motion_env,
                [0.8, 1.2, 0.4],
                profile.length_m,
                9,
            )
            .unwrap()
            .iter()
            .map(|p| {
                let point = DVec3::new(origin[0] as f64, origin[1] as f64, origin[2] as f64)
                    + rotation * DVec3::new(p[0], p[1], p[2]);
                [point.x as f32, point.y as f32, point.z as f32, p[3] as f32]
            })
            .collect::<Vec<_>>()
            .into()
        } else {
            std::sync::Arc::from([])
        };
        let draw = VolumeDraw {
            radial_reference: [1.0, 0.0, 0.0],
            centerline,
            origin_len: [origin[0], origin[1], origin[2], len],
            axis_radius: [axis[0], axis[1], axis[2], radius],
            animation: [14.0, 0.0, 0.0, 0.0],
            diamonds: true,
            tail_dissolve: false,
            profile: profile
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
                .collect(),
        };
        let directory = out.join(name);
        std::fs::create_dir(&directory).unwrap();
        for steps in [14, 16] {
            let mut current = draw.clone();
            current.animation[0] = steps as f32;
            renderer
                .update(&backend, &camera, Some(PlumeFrame::Volume(current)))
                .unwrap();
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            clear(&mut encoder, &color, &target.depth, 1.0);
            renderer.encode(&mut encoder, &color, &target.depth);
            backend.queue.submit([encoder.finish()]);
            let name = if steps == 14 {
                "linear.rgba16f"
            } else {
                "steps16.rgba16f"
            };
            std::fs::write(
                directory.join(name),
                thessa_render_wgpu::readback::read_color(&backend, &hdr).unwrap(),
            )
            .unwrap();
        }
        let metadata = serde_json::json!({
            "id":if curved {"curved-plume-volume-v1"} else {"compiled-plume-volume-v1"}, "case":name, "shock_radial_mode":"axisymmetric-j0-v1", "radial_weight_mode":"gaussian-v2", "extent":[129,129], "units":"metres",
            "origin_len":draw.origin_len, "axis_radius":draw.axis_radius, "profile":&*draw.profile,
            "centerline":&*draw.centerline,
            "curve_metric":"emitted-length-v1",
            "motion":if curved { serde_json::json!({"angular_velocity_nozzle_rps":[0.8,1.2,0.4],"air_velocity_nozzle_mps":[450.0,-180.0,0.0],"ambient_density_kg_m3":env.density_kg_m3}) } else {serde_json::Value::Null},
            "steps":14, "exposure_gain":1.0, "erosion":0, "tail_dissolve":false, "diamonds":true,
            "world_from_view":camera.world_from_view.to_cols_array(), "clip_from_view":camera.clip_from_view.to_cols_array(),
            "engine_spec":spec, "operating_point":point, "effective_throttle":throttle,
            "source_throttle":source.throttle, "exit_radius_m":state.exit_radius_m,
            "ambient_pressure_pa":ambient, "ambient_temperature_k":env.temperature_k,
            "model_regime":format!("{:?}", profile.regime), "backend":format!("{:?}",backend.adapter.get_info().backend)
        });
        std::fs::write(
            directory.join("scene.json"),
            serde_json::to_vec_pretty(&metadata).unwrap(),
        )
        .unwrap();
    }
    eprintln!(
        "compiled_plume_reference adapter={:?} curved={curved} cases=9-or-3 extent=129x129 steps=14/16 calibration=none",
        backend.adapter_name
    );
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
    let mut unordered = volume(8.0);
    std::sync::Arc::make_mut(&mut unordered.profile).swap(0, 1);
    assert!(
        plume
            .update(
                &backend,
                &view(DVec3::ZERO, DQuat::IDENTITY),
                Some(PlumeFrame::Volume(unordered))
            )
            .is_err()
    );
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
                    radial_reference: [1.0, 0.0, 0.0],
                    centerline: std::sync::Arc::from([]),
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

#[test]
#[ignore = "hardware: fixed-work HDR plume lookup ABBA; requires new THESSA_BENCH_OUTPUT directory"]
fn published_profile_lookup_cost() {
    profile_cost(false);
}

#[test]
#[ignore = "hardware: fixed-work 3D trajectory plume cost; new THESSA_BENCH_OUTPUT directory required"]
fn curved_plume_gpu_cost() {
    profile_cost(true);
}

fn profile_cost(curved: bool) {
    use std::{io::Write, time::Instant};
    use thessa_plume_core::{
        ExhaustFamily, PlumeEnvironment, PlumeSource, RigidTransform, profile::build_axial_profile,
    };
    use thessa_render_wgpu::timing::GpuIntervals;
    let out = std::path::PathBuf::from(
        std::env::var_os("THESSA_BENCH_OUTPUT").expect("output directory required"),
    );
    std::fs::create_dir(&out).unwrap();
    let backend = Backend::bring_up_for_graphics_profiled(&Default::default()).unwrap();
    let timing = GpuIntervals::new(&backend, 2).expect("timestamps required");
    let target = OffscreenTarget::new(&backend, 1280, 720).unwrap();
    let hdr = backend.device.create_texture(&wgpu::TextureDescriptor {
        label: Some("plume-lookup-hdr"),
        size: wgpu::Extent3d {
            width: 1280,
            height: 720,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba16Float,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
        view_formats: &[],
    });
    let color = hdr.create_view(&Default::default());
    let mut camera = view(DVec3::ZERO, DQuat::IDENTITY);
    camera.clip_from_view =
        glam::dcamera::rh::proj::directx::perspective(0.9, 1280.0 / 720.0, 0.1, 1e6);
    let mut plume = PlumeRenderer::new(&backend, wgpu::TextureFormat::Rgba16Float, true).unwrap();
    let mut csv = std::fs::File::create_new(out.join("frames.csv")).unwrap();
    writeln!(csv, "case,frame,cpu_submit_ms,wall_ms,gpu_ms").unwrap();
    // Both production-sized profiles, near-nozzle and whole-field side views,
    // plus the publication limits. No scalar fitted to the measured pixels.
    let cases = [
        (101_325.0, 48, 0.5, 8.0),
        (101_325.0, 48, 0.5, 14.0),
        (0.0, 48, 0.5, 8.0),
        (0.0, 48, 0.5, 14.0),
        (101_325.0, 48, 0.02, 8.0),
        (101_325.0, 48, 0.02, 14.0),
        (0.0, 48, 0.02, 8.0),
        (0.0, 48, 0.02, 14.0),
        (101_325.0, 2, 0.5, 14.0),
        (101_325.0, 64, 0.5, 14.0),
    ];
    for (case, (pressure_pa, count, center_fraction, steps)) in cases.into_iter().enumerate() {
        let source = PlumeSource {
            nozzle_to_vehicle: RigidTransform::IDENTITY,
            exit_radius_m: 0.55,
            mass_flow_kg_s: 480.0,
            exhaust_velocity_mps: 3550.0,
            exit_pressure_pa: 120_000.0,
            exit_temperature_k: 1850.0,
            exit_mach: 3.5,
            throttle: 1.0,
            exhaust: ExhaustFamily::Methalox,
        };
        let env = PlumeEnvironment {
            pressure_pa,
            density_kg_m3: 0.0,
            temperature_k: 280.0,
            oxygen_fraction: 0.0,
            flow_velocity_local_mps: [0.0; 3],
        };
        let profile = build_axial_profile(&source, &env, count).unwrap();
        let z = profile.length_m * center_fraction;
        let radius = profile
            .stations
            .iter()
            .map(|s| s.radius_m)
            .fold(0.0, f64::max)
            * 1.6;
        let distance = if center_fraction < 0.1 {
            profile.evaluate(z).unwrap().radius_m * 4.0
        } else {
            profile.length_m * 1.1
        };
        let publication: std::sync::Arc<[[f32; 8]]> = profile
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
        std::fs::write(
            out.join(format!("case-{case}.stations.f32")),
            publication
                .iter()
                .flatten()
                .flat_map(|v| v.to_le_bytes())
                .collect::<Vec<_>>(),
        )
        .unwrap();
        let centerline = if curved {
            let mut moving_env = env;
            moving_env.density_kg_m3 = if pressure_pa > 0.0 { 1.2 } else { 0.0 };
            moving_env.flow_velocity_local_mps = [150.0, -60.0, 0.0];
            thessa_plume_core::motion::centerline(
                &source,
                &moving_env,
                [0.8, 0.5, 0.2],
                profile.length_m,
                if steps < 10.0 { 9 } else { 17 },
            )
            .unwrap()
            .into_iter()
            .map(|p| {
                [
                    p[0] as f32,
                    (p[2] - z) as f32,
                    (-p[1] - distance) as f32,
                    p[3] as f32,
                ]
            })
            .collect::<Vec<_>>()
            .into()
        } else {
            std::sync::Arc::from([])
        };
        let draw = VolumeDraw {
            radial_reference: [1.0, 0.0, 0.0],
            centerline,
            origin_len: [0.0, -z as f32, -distance as f32, profile.length_m as f32],
            axis_radius: [0.0, 1.0, 0.0, radius as f32],
            animation: [steps, 0.0, 0.0, 0.0],
            diamonds: true,
            tail_dissolve: false,
            profile: publication,
        };
        plume
            .update(&backend, &camera, Some(PlumeFrame::Volume(draw)))
            .unwrap();
        for frame in 0..100 {
            let started = Instant::now();
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            {
                let _clear = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                    color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                        view: &color,
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
                            load: wgpu::LoadOp::Clear(1.0),
                            store: wgpu::StoreOp::Store,
                        }),
                        stencil_ops: None,
                    }),
                    ..Default::default()
                });
            }
            timing.mark(&mut encoder, 0);
            plume.encode(&mut encoder, &color, &target.depth);
            timing.mark(&mut encoder, 1);
            timing.resolve(&mut encoder);
            backend.queue.submit([encoder.finish()]);
            let cpu = started.elapsed().as_secs_f64() * 1000.0;
            backend
                .device
                .poll(wgpu::PollType::wait_indefinitely())
                .unwrap();
            let wall = started.elapsed().as_secs_f64() * 1000.0;
            let gpu = timing.read(&backend).unwrap()[0] * 1000.0;
            if frame >= 20 {
                writeln!(csv, "{case},{},{cpu},{wall},{gpu}", frame - 20).unwrap();
            }
        }
        let pixels = thessa_render_wgpu::readback::read_color(&backend, &hdr).unwrap();
        let mut signal = false;
        for bytes in pixels.as_chunks::<2>().0.iter() {
            let half = u16::from_le_bytes([bytes[0], bytes[1]]);
            assert_ne!(half & 0x7c00, 0x7c00, "non-finite HDR");
            signal |= half & 0x7fff != 0;
        }
        assert!(signal, "fixture must witness an actual plume");
        std::fs::write(out.join(format!("case-{case}.rgba16f")), pixels).unwrap();
        eprintln!(
            "plume_lookup adapter={:?} curved={curved} case={case} pressure_pa={pressure_pa} stations={count} center_fraction={center_fraction} steps={steps} length_m={} radius_bound_m={radius} camera_distance_m={distance} extent=1280x720 format=RGBA16Float source=provisional-methalox throttle=1 time=0 erosion=0 tail_dissolve=false warmup=20 samples=80",
            backend.adapter_name, profile.length_m
        );
    }
}

#[test]
#[ignore = "hardware: curved 3D field from six rotations vs independent dense world-space ray integration"]
fn curved_volume_matches_dense_world_space_integration() {
    let backend = Backend::bring_up().unwrap();
    let target = OffscreenTarget::new(&backend, 65, 65).unwrap();
    let texture = hdr_target(&backend, 65, 65);
    let color = texture.create_view(&Default::default());
    let camera = view(DVec3::ZERO, DQuat::IDENTITY);
    let mut renderer =
        PlumeRenderer::new(&backend, wgpu::TextureFormat::Rgba16Float, true).unwrap();
    let path = [
        DVec3::new(-0.7, -2.0, 0.5),
        DVec3::new(0.0, -0.8, -0.3),
        DVec3::new(0.9, 0.6, -0.1),
        DVec3::new(-0.1, 2.0, 0.7),
    ];
    let profile = AxialProfile {
        length_m: 4.0,
        regime: ExpansionRegime::Matched,
        stations: [0.0, 4.0]
            .map(|z_m| AxialStation {
                z_m,
                radius_m: 0.65,
                center_density_kg_m3: 1.0,
                center_temp_k: 1800.0,
                extinction_per_m: 0.15,
                emission_rgb: [0.06, 0.03, 0.015],
                shock: 1.0,
            })
            .to_vec(),
    };
    let mut max_error = 0.0_f64;
    for (case, rotation) in [
        DQuat::IDENTITY,
        DQuat::from_rotation_y(0.7),
        DQuat::from_rotation_x(0.9),
        DQuat::from_rotation_z(1.2),
        DQuat::from_rotation_y(1.6),
        DQuat::from_rotation_x(2.2),
        DQuat::IDENTITY,
        DQuat::IDENTITY,
    ]
    .into_iter()
    .enumerate()
    {
        let folded = [
            DVec3::new(0.0, -1.0, -5.0),
            DVec3::new(0.0, 1.0, -5.0),
            DVec3::new(5.0, 1.0, -10.0),
            DVec3::new(5.0, -1.0, -15.0),
            DVec3::new(0.0, -1.0, -15.0),
            DVec3::new(0.0, 1.0, -15.0),
        ];
        let path: &[DVec3] = if case == 7 { &folded } else { &path };
        let centers: Vec<_> = path
            .iter()
            .enumerate()
            .map(|(i, p)| {
                let p = rotation * *p
                    + if case >= 6 {
                        DVec3::ZERO
                    } else {
                        DVec3::new(0.0, 0.0, -10.0)
                    };
                [
                    p.x as f32,
                    p.y as f32,
                    p.z as f32,
                    4.0 * i as f32 / (path.len() - 1) as f32,
                ]
            })
            .collect();
        let mut draw = volume(14.0);
        draw.centerline = centers.clone().into();
        draw.origin_len = [centers[0][0], centers[0][1], centers[0][2], 4.0];
        draw.profile = vec![
            [0.0, 0.65, 0.15, 1.0, 0.06, 0.03, 0.015, 0.0],
            [4.0, 0.65, 0.15, 1.0, 0.06, 0.03, 0.015, 0.0],
        ]
        .into();
        renderer
            .update(&backend, &camera, Some(PlumeFrame::Volume(draw)))
            .unwrap();
        for clipped in [false, true] {
            let plane_m = if case == 6 { 0.3 } else { 9.5 };
            let depth_value = if clipped {
                camera
                    .clip_from_view
                    .project_point3(DVec3::new(0.0, 0.0, -plane_m))
                    .z as f32
            } else {
                1.0
            };
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            clear(&mut encoder, &color, &target.depth, depth_value);
            renderer.encode(&mut encoder, &color, &target.depth);
            backend.queue.submit([encoder.finish()]);
            let pixels = thessa_render_wgpu::readback::read_color(&backend, &texture).unwrap();
            let actual: Vec<_> = pixels
                .as_chunks::<2>()
                .0
                .iter()
                .map(|p| half(u16::from_le_bytes(*p)))
                .collect();
            let mut signal = 0.0_f64;
            for y in (20..45).step_by(4) {
                for x in (20..45).step_by(4) {
                    let rd = DVec3::new(
                        (2.0 * (x as f64 + 0.5) / 65.0 - 1.0) * (0.45_f64).tan(),
                        (1.0 - 2.0 * (y as f64 + 0.5) / 65.0) * (0.45_f64).tan(),
                        -1.0,
                    )
                    .normalize();
                    let dt = (if clipped { plane_m / -rd.z } else { 25.0 }) / 8192.0;
                    let (mut trans, mut rgb) = (1.0, [0.0; 3]);
                    for step in 0..8192 {
                        let p = rd * ((step as f64 + 0.5) * dt);
                        let mut nearest = None;
                        let mut best = 1.6_f64.powi(2);
                        for (j, pair) in centers.windows(2).enumerate() {
                            let a =
                                DVec3::new(pair[0][0] as f64, pair[0][1] as f64, pair[0][2] as f64);
                            let b =
                                DVec3::new(pair[1][0] as f64, pair[1][1] as f64, pair[1][2] as f64);
                            let ab = b - a;
                            let t = (p - a).dot(ab) / ab.length_squared();
                            if (j == 0 && t < 0.0) || (j == centers.len() - 2 && t > 1.0) {
                                continue;
                            }
                            let t = t.clamp(0.0, 1.0);
                            let d2 = (p - a - ab * t).length_squared();
                            if d2 < best {
                                best = d2;
                                nearest = Some((
                                    pair[0][3] as f64 * (1.0 - t) + pair[1][3] as f64 * t,
                                    d2.sqrt(),
                                    (pair[1][3] - pair[0][3]) as f64 / ab.length(),
                                ));
                            }
                        }
                        if let Some((z, r, metric)) = nearest {
                            let mut sample = thessa_plume_core::sample_medium(&profile, z, r);
                            sample.extinction_per_m *= metric;
                            sample.emission_rgb = sample.emission_rgb.map(|v| v * metric);
                            let tau = sample.extinction_per_m * dt;
                            let length = if sample.extinction_per_m > 0.0 {
                                -(-tau).exp_m1() / sample.extinction_per_m
                            } else {
                                dt
                            };
                            for (c, v) in rgb.iter_mut().enumerate() {
                                *v += trans * sample.emission_rgb[c] * length;
                            }
                            trans *= (-tau).exp();
                        }
                    }
                    let expected = [rgb[0], rgb[1], rgb[2], 1.0 - trans];
                    for (c, expected) in expected.into_iter().enumerate() {
                        let v = actual[(y * 65 + x) * 4 + c];
                        let error = (v - expected).abs();
                        max_error = max_error.max(error);
                        assert!(
                            error < 0.012,
                            "case={case} pixel={x},{y} channel={c} actual={v} expected={expected}"
                        );
                        signal = signal.max(v);
                    }
                }
            }
            if !clipped {
                assert!(signal > 0.03, "nonempty 3D view required");
            }
        }
        let mut invalid = volume(14.0);
        invalid.centerline = vec![[0.0; 4]; 2].into();
        assert!(
            renderer
                .update(&backend, &camera, Some(PlumeFrame::Volume(invalid)))
                .is_err()
        );
        assert!(!renderer.active());
    }
    eprintln!(
        "curved_volume adapter={:?} views=8 depth_variants=2 probe_rays=784 dense_steps=8192 max_abs_rgba_error={max_error}",
        backend.adapter_name
    );
}

#[test]
#[ignore = "hardware: 3D shell is covariant under world/camera rotations and follows nozzle roll"]
fn curved_turbulence_uses_the_nozzle_frame() {
    let backend = Backend::bring_up().unwrap();
    let target = OffscreenTarget::new(&backend, 65, 65).unwrap();
    let texture = hdr_target(&backend, 65, 65);
    let color = texture.create_view(&Default::default());
    let mut renderer =
        PlumeRenderer::new(&backend, wgpu::TextureFormat::Rgba16Float, true).unwrap();
    let path = [
        Vec3::new(0.0, -2.0, -10.0),
        Vec3::new(0.4, 0.0, -9.5),
        Vec3::new(0.0, 2.0, -10.0),
    ];
    let mut baseline: Vec<f64> = Vec::new();
    let mut max_error = 0.0_f64;
    for (case, rotation) in [
        DQuat::IDENTITY,
        DQuat::from_rotation_y(0.8),
        DQuat::from_rotation_x(1.1),
        DQuat::from_rotation_z(0.6),
        DQuat::IDENTITY,
    ]
    .into_iter()
    .enumerate()
    {
        let mut draw = volume(14.0);
        draw.centerline = path
            .iter()
            .enumerate()
            .map(|(i, p)| {
                let point = rotation * p.as_dvec3();
                [
                    point.x as f32,
                    point.y as f32,
                    point.z as f32,
                    i as f32 * 2.0,
                ]
            })
            .collect::<Vec<_>>()
            .into();
        draw.origin_len[..3].copy_from_slice(&draw.centerline[0][..3]);
        draw.axis_radius[..3].copy_from_slice(&(rotation * DVec3::Y).as_vec3().to_array());
        draw.radial_reference = (rotation * if case == 4 { -DVec3::X } else { DVec3::X })
            .as_vec3()
            .to_array();
        draw.animation = [14.0, 2.0, 0.4, 150.0];
        renderer
            .update(
                &backend,
                &view(DVec3::ZERO, rotation),
                Some(PlumeFrame::Volume(draw)),
            )
            .unwrap();
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        clear(&mut encoder, &color, &target.depth, 1.0);
        renderer.encode(&mut encoder, &color, &target.depth);
        backend.queue.submit([encoder.finish()]);
        let raw = thessa_render_wgpu::readback::read_color(&backend, &texture).unwrap();
        let pixels: Vec<_> = raw
            .as_chunks::<2>()
            .0
            .iter()
            .map(|b| half(u16::from_le_bytes(*b)))
            .collect();
        if case == 0 {
            baseline = pixels;
            continue;
        }
        let error = baseline
            .iter()
            .zip(&pixels)
            .map(|(a, b)| (a - b).abs())
            .fold(0.0, f64::max);
        if case == 4 {
            assert!(
                error > 0.001,
                "rolling nozzle must rotate asymmetric shell structure"
            );
        } else {
            max_error = max_error.max(error);
            assert!(error < 0.002, "case={case} rotation error={error}");
        }
    }
    eprintln!(
        "curved_turbulence adapter={:?} common_rotation_max_rgba_error={max_error}",
        backend.adapter_name
    );
}
