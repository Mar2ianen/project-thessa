use glam::{DMat4, DQuat, DVec3};
use thessa_graphics::Quality;
use thessa_rcbt_core::lighting::SurfaceLighting;
use thessa_render_core::{ViewParams, mesh::*};
use thessa_render_wgpu::{
    Backend,
    mesh::{MeshRenderer, SurfaceSettings},
    terrain::OffscreenTarget,
};

fn quad(z: f32, material: usize) -> Primitive {
    Primitive {
        ambient_visibility: vec![],
        material,
        vertices: [[-1., -1., z], [1., -1., z], [1., 1., z], [-1., 1., z]]
            .map(|position| Vertex {
                position,
                normal: [0., 0., 1.],
                uv: [0.; 2],
            })
            .to_vec(),
        indices: vec![0, 1, 2, 0, 2, 3],
    }
}
fn source() -> MeshAsset {
    MeshAsset {
        primitives: vec![quad(-0.25, 0), quad(-0.75, 1), quad(-0.5, 2)],
        materials: vec![
            Material {
                alpha_mode: AlphaMode::Blend,
                base_color: [0., 0., 0., 0.5],
                emission_rgb: [0., 2., 0.],
                double_sided: true,
                ..Default::default()
            },
            Material {
                base_color: [0., 0., 0., 0.],
                emission_rgb: [0., 0., 1.],
                double_sided: true,
                ..Default::default()
            },
            Material {
                alpha_mode: AlphaMode::Blend,
                base_color: [0., 0., 0., 0.25],
                emission_rgb: [4., 0., 0.],
                double_sided: false,
                ..Default::default()
            },
        ],
        textures: vec![],
        images: vec![],
    }
}
fn renderer(backend: &Backend, asset: &MeshAsset, quality: Quality) -> MeshRenderer {
    let mut renderer = MeshRenderer::with_surface(
        backend,
        asset,
        SurfaceSettings {
            format: wgpu::TextureFormat::Rgba16Float,
            capacity: 3,
            quality,
            reflections: false,
            shadows: None,
            ray_tracing: None,
        },
    )
    .unwrap();
    renderer
        .update(
            backend,
            &ViewParams {
                world_from_view: DMat4::IDENTITY,
                world_position_m: DVec3::ZERO,
                clip_from_view: DMat4::from_scale(DVec3::new(1., 1., -1.)),
                exposure: 0.5,
            },
            &SurfaceLighting::default(),
            &[MeshInstance {
                position_m: DVec3::ZERO,
                rotation: DQuat::IDENTITY,
                scale: 1.,
            }],
        )
        .unwrap();
    renderer
}
fn hdr_target(backend: &Backend, width: u32, height: u32) -> wgpu::Texture {
    backend.device.create_texture(&wgpu::TextureDescriptor {
        label: Some("transparency-hdr-oracle"),
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
fn pixel(backend: &Backend, meshes: &[&MeshRenderer], global: bool) -> [u16; 4] {
    composition(backend, meshes, global).0
}
fn composition(
    backend: &Backend,
    meshes: &[&MeshRenderer],
    global: bool,
) -> ([u16; 4], Option<usize>) {
    let target = OffscreenTarget::new(backend, 16, 16).unwrap();
    let hdr = hdr_target(backend, 16, 16);
    let color = hdr.create_view(&Default::default());
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    let mut transparent_draws = None;
    {
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: &color,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: &target.depth,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(1.),
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            }),
            ..Default::default()
        });
        if global {
            transparent_draws = Some(MeshRenderer::draw_scene_into_pass(meshes, &mut pass));
        } else {
            meshes[0].draw_into_pass(&mut pass);
        }
    }
    backend.queue.submit([encoder.finish()]);
    let bytes = thessa_render_wgpu::readback::read_color(backend, &hdr).unwrap();
    let pixel = std::array::from_fn(|i| {
        u16::from_le_bytes(
            bytes[(8 * 16 + 8) * 8 + i * 2..(8 * 16 + 8) * 8 + i * 2 + 2]
                .try_into()
                .unwrap(),
        )
    });
    (pixel, transparent_draws)
}

#[test]
fn hdr_emission_and_alpha_over_are_order_independent_for_separated_surfaces() {
    let backend = Backend::bring_up().unwrap();
    let temporal = thessa_graphics::ResolvedGraphicsSettings {
        temporal_taa: true,
        ..Default::default()
    };
    assert!(
        matches!(MeshRenderer::with_surface(&backend, &source(), SurfaceSettings {
        format: wgpu::TextureFormat::Rgba16Float, capacity: 3, quality: Quality::High,
        reflections: false, shadows: None, ray_tracing: Some(&temporal),
    }), Err(thessa_render_wgpu::BackendError::Device(reason)) if reason.contains("BLEND temporal"))
    );
    for quality in [Quality::Low, Quality::High] {
        for reverse in [false, true] {
            let mut asset = source();
            if reverse {
                asset.primitives.reverse();
            }
            let r = renderer(&backend, &asset, quality);
            // (.5 green) over (.5 red) over (.5 blue), with alpha .5 and .25:
            // final pre-exposed RGB=(.25,.5,.1875), alpha one, exact half floats.
            assert_eq!(
                pixel(&backend, &[&r], false),
                [0x3400, 0x3800, 0x3200, 0x3c00]
            );
            assert_eq!(r.draw_stats().draw_commands, 3);
        }
    }
    let asset = source();
    let mut front = asset.clone();
    front.primitives = vec![asset.primitives[0].clone()];
    let mut back = asset.clone();
    back.primitives = asset.primitives[1..].to_vec();
    let front = renderer(&backend, &front, Quality::Low);
    let back = renderer(&backend, &back, Quality::Low);
    assert_eq!(
        pixel(&backend, &[&front, &back], true),
        [0x3400, 0x3800, 0x3200, 0x3c00]
    );
    assert_eq!(
        pixel(&backend, &[&back, &front], true),
        [0x3400, 0x3800, 0x3200, 0x3c00]
    );
    let mut emitter = source();
    emitter.primitives = vec![quad(-0.25, 0)];
    emitter.materials[0].alpha_mode = AlphaMode::Opaque;
    emitter.materials[0].emission_rgb = [4., 0., 0.];
    let emitter = renderer(&backend, &emitter, Quality::High);
    assert_eq!(
        pixel(&backend, &[&emitter], false),
        [0x4000, 0, 0, 0x3c00],
        "emission must survive above one, without lighting"
    );
}

#[test]
fn moved_culled_and_removed_instances_refresh_the_transparent_queue() {
    let backend = Backend::bring_up().unwrap();
    let mut mesh = renderer(&backend, &source(), Quality::High);
    let view = ViewParams {
        world_from_view: DMat4::IDENTITY,
        world_position_m: DVec3::ZERO,
        clip_from_view: DMat4::from_scale(DVec3::new(1., 1., -1.)),
        exposure: 0.5,
    };
    let mut instances = vec![
        MeshInstance {
            position_m: DVec3::ZERO,
            rotation: DQuat::IDENTITY,
            scale: 1.,
        };
        3
    ];
    for red_position in [
        DVec3::new(0., 0., 0.4),
        DVec3::ZERO,
        DVec3::new(10., 0., 0.),
    ] {
        instances[2].position_m = red_position;
        for _ in 0..2 {
            mesh.update_selected(
                &backend,
                &view,
                &SurfaceLighting::default(),
                &instances,
                Some(&[0..1, 1..2, 2..3]),
            )
            .unwrap();
            let expected = if red_position.z > 0. {
                [0x3800, 0x3600, 0x3200, 0x3c00]
            } else if red_position.x > 0. {
                [0, 0x3800, 0x3400, 0x3c00]
            } else {
                [0x3400, 0x3800, 0x3200, 0x3c00]
            };
            assert_eq!(pixel(&backend, &[&mesh], false), expected);
        }
    }
    mesh.update_selected(
        &backend,
        &view,
        &SurfaceLighting::default(),
        &[],
        Some(&[0..0, 0..0, 0..0]),
    )
    .unwrap();
    assert_eq!(mesh.draw_stats().draw_commands, 0);
    assert_eq!(pixel(&backend, &[&mesh], false), [0, 0, 0, 0x3c00]);
}

#[test]
#[ignore = "requires the pinned CC-BY-4.0 Khronos asset; set THESSA_REFERENCE_ASSET_CACHE after reference_assets.py ingestion"]
fn original_khronos_emissive_strength_renders_all_five_hdr_levels() {
    let path = std::path::PathBuf::from(std::env::var_os("THESSA_REFERENCE_ASSET_CACHE").expect("external cache required"))
        .join("khronos/EmissiveStrengthTest/8f492390ddc7a2ae221a1369ef560589cf8c70fb/original/EmissiveStrengthTest.glb");
    let bytes = std::fs::read(path).unwrap();
    assert_eq!(bytes.len(), 10668);
    assert!(thessa_render_assets::decode_glb(&bytes, Default::default()).is_err());
    let asset = thessa_render_assets::decode_glb_surfaces(&bytes, Default::default()).unwrap();
    assert_eq!(asset.primitives.len(), 6);
    assert_eq!(
        asset
            .primitives
            .iter()
            .map(|p| p.indices.len() / 3)
            .sum::<usize>(),
        90
    );
    let backend = Backend::bring_up().unwrap();
    for quality in [Quality::Low, Quality::High] {
        let mut mesh = renderer(&backend, &asset, quality);
        for (level, x) in [-6., -3., 0., 3., 6.].into_iter().enumerate() {
            let eye = DVec3::new(x, 0., 5.);
            mesh.update(
                &backend,
                &ViewParams {
                    world_from_view: DMat4::from_translation(eye),
                    world_position_m: eye,
                    clip_from_view: DMat4::from_scale(DVec3::new(1., 1., -0.1)),
                    exposure: 0.5,
                },
                &SurfaceLighting::default(),
                &[MeshInstance {
                    position_m: DVec3::ZERO,
                    rotation: DQuat::IDENTITY,
                    scale: 1.,
                }],
            )
            .unwrap();
            let actual = pixel(&backend, &[&mesh], false);
            // RGB=[0.1,0.5,0.9] × strength × exposure; powers of two shift the
            // half-float exponent exactly. One ULP permits shader rounding.
            let expected = [0x2a66u16, 0x3400, 0x3733].map(|bits| bits + (level as u16) * 0x400);
            for i in 0..3 {
                assert!(
                    actual[i].abs_diff(expected[i]) <= 1,
                    "{quality:?} level={level} actual={actual:?} expected={expected:?}"
                );
            }
            assert_eq!(actual[3], 0x3c00);
            eprintln!(
                "khronos_emission quality={quality:?} strength={} rgba16_bits={actual:?}",
                1 << level
            );
        }
    }
}

#[test]
fn transparent_instance_runs_preserve_order_gaps_and_cross_consumer_interleaving() {
    let backend = Backend::bring_up().unwrap();
    let mut asset = source();
    let mut primitive = quad(-0.25, 0);
    for (vertex, uv) in primitive
        .vertices
        .iter_mut()
        .zip([[0., 1.], [1., 1.], [1., 0.], [0., 0.]])
    {
        vertex.uv = uv;
    }
    asset.primitives = vec![primitive];
    asset.materials.truncate(1);
    asset.materials[0].base_color = [1.; 4];
    asset.materials[0].metallic = 0.;
    asset.materials[0].emission_rgb = [0.; 3];
    asset.materials[0].base_texture = Some(0);
    asset.textures = vec![Texture {
        image: 0,
        wrap: [Wrap::Clamp; 2],
        linear: false,
        min_linear: false,
    }];
    asset.images = vec![Image {
        width: 2,
        height: 1,
        rgba8: vec![255, 0, 0, 128, 0, 255, 0, 128],
    }];
    let mut combined = renderer(&backend, &asset, Quality::Low);
    let mut first = renderer(&backend, &asset, Quality::Low);
    let mut second = renderer(&backend, &asset, Quality::Low);
    let view = ViewParams {
        world_from_view: DMat4::IDENTITY,
        world_position_m: DVec3::ZERO,
        clip_from_view: DMat4::from_scale(DVec3::new(1., 1., -1.)),
        exposure: 0.5,
    };
    let light = SurfaceLighting {
        ambient_lux: [std::f32::consts::PI; 4],
        ..Default::default()
    };
    let mut poses = [
        MeshInstance {
            position_m: DVec3::new(-0.4, 0., -0.25),
            rotation: DQuat::IDENTITY,
            scale: 1.,
        },
        MeshInstance {
            position_m: DVec3::new(0.4, 0., 0.),
            rotation: DQuat::IDENTITY,
            scale: 1.,
        },
    ];
    let mut samples = vec![];
    for reverse in [false, true] {
        if reverse {
            poses[0].position_m.z = 0.;
            poses[1].position_m.z = -0.25;
        }
        combined.update(&backend, &view, &light, &poses).unwrap();
        first.update(&backend, &view, &light, &poses[..1]).unwrap();
        second.update(&backend, &view, &light, &poses[1..]).unwrap();
        let image = pixel(&backend, &[&combined], false);
        assert!(
            image[0] > 0 && image[1] > 0,
            "both differently coloured instances must contribute"
        );
        assert_eq!(image, pixel(&backend, &[&first, &second], true));
        assert_eq!(
            combined.draw_stats().draw_calls,
            if reverse { 2 } else { 1 }
        );
        samples.push(image);
    }
    assert_ne!(
        samples[0], samples[1],
        "alpha-over must remain order-dependent"
    );
    poses[0].position_m.z = -0.25;
    poses[1].position_m.z = 0.;
    let mut middle_asset = source();
    middle_asset.primitives = vec![quad(-0.375, 0)];
    middle_asset.materials[0].emission_rgb = [0., 0., 1.];
    let middle = renderer(&backend, &middle_asset, Quality::Low);
    combined.update(&backend, &view, &light, &poses).unwrap();
    first.update(&backend, &view, &light, &poses[..1]).unwrap();
    second.update(&backend, &view, &light, &poses[1..]).unwrap();
    let merged = composition(&backend, &[&combined, &middle], true);
    assert_eq!(
        merged.1,
        Some(3),
        "global depth interleaving must split the local run"
    );
    assert_eq!(
        merged.0,
        pixel(&backend, &[&first, &middle, &second], true),
        "another consumer between two depths must split the local instanced run"
    );
    let mut outside = poses[0];
    outside.position_m.x = 10.;
    combined
        .update(&backend, &view, &light, &[poses[0], outside, poses[1]])
        .unwrap();
    assert_eq!(
        combined.draw_stats().draw_calls,
        2,
        "culled instance IDs must not be included in a run"
    );
    assert_eq!(pixel(&backend, &[&combined], false), samples[0]);
}

#[test]
#[ignore = "hardware fixed-work transparent submission cost; set THESSA_BENCH_OUTPUT to a new directory"]
fn fixed_transparent_instance_submission_cost() {
    use std::{io::Write, time::Instant};
    let out = std::path::PathBuf::from(
        std::env::var_os("THESSA_BENCH_OUTPUT").expect("output directory required"),
    );
    std::fs::create_dir(&out).unwrap();
    let backend = Backend::bring_up_for_graphics_profiled(&Default::default()).unwrap();
    let mut asset = source();
    asset.primitives = vec![quad(-0.25, 0)];
    asset.materials.truncate(1);
    asset.materials[0].emission_rgb = [0.1, 0.5, 0.9];
    let instances: Vec<_> = (0..8192)
        .map(|i| MeshInstance {
            position_m: DVec3::new(
                -1. + 2. * (f64::from(i % 128) + 0.5) / 128.,
                -1. + 2. * (f64::from(i / 128) + 0.5) / 64.,
                -0.5,
            ),
            rotation: DQuat::IDENTITY,
            scale: 0.9 / 128.,
        })
        .collect();
    let view = ViewParams {
        world_from_view: DMat4::IDENTITY,
        world_position_m: DVec3::ZERO,
        clip_from_view: DMat4::from_scale(DVec3::new(1., 1., -1.)),
        exposure: 0.5,
    };
    let target = OffscreenTarget::new(&backend, 1280, 720).unwrap();
    let hdr = hdr_target(&backend, 1280, 720);
    let color = hdr.create_view(&Default::default());
    let mut mesh = MeshRenderer::with_surface(
        &backend,
        &asset,
        SurfaceSettings {
            format: wgpu::TextureFormat::Rgba16Float,
            capacity: 8192,
            quality: Quality::High,
            reflections: false,
            shadows: None,
            ray_tracing: None,
        },
    )
    .unwrap();
    let timing =
        thessa_render_wgpu::timing::GpuIntervals::new(&backend, 2).expect("timestamps required");
    let mut csv = std::fs::File::create_new(out.join("frames.csv")).unwrap();
    writeln!(csv, "frame,cpu_submit_ms,wall_ms,gpu_ms").unwrap();
    for frame in 0..150 {
        let started = Instant::now();
        mesh.update(&backend, &view, &SurfaceLighting::default(), &instances)
            .unwrap();
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        {
            let _pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &color,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &target.depth,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                ..Default::default()
            });
        }
        timing.mark(&mut encoder, 0);
        mesh.encode(&mut encoder, &color, &target.depth);
        timing.mark(&mut encoder, 1);
        timing.resolve(&mut encoder);
        backend.queue.submit([encoder.finish()]);
        let cpu = started.elapsed().as_secs_f64() * 1000.;
        backend
            .device
            .poll(wgpu::PollType::wait_indefinitely())
            .unwrap();
        let wall = started.elapsed().as_secs_f64() * 1000.;
        let gpu = timing.read(&backend).unwrap()[0] * 1000.;
        if frame >= 30 {
            writeln!(csv, "{},{cpu},{wall},{gpu}", frame - 30).unwrap();
        }
    }
    let stats = mesh.draw_stats();
    assert_eq!(stats.instances, 8192);
    assert_eq!(stats.triangles, 16384);
    std::fs::write(
        out.join("linear.rgba16f"),
        thessa_render_wgpu::readback::read_color(&backend, &hdr).unwrap(),
    )
    .unwrap();
    eprintln!(
        "transparent_submission adapter={:?} instances={} triangles={} calls={} commands={} extent=1280x720 format=RGBA16Float quality=High lighting=zero alpha=0.5 exposure=0.5 frames=120 warmup=30; timestamp-map/readback outside samples",
        backend.adapter_name,
        stats.instances,
        stats.triangles,
        stats.draw_calls,
        stats.draw_commands
    );
}
