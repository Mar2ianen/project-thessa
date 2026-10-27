use glam::{DMat4, DQuat, DVec3};
use thessa_rcbt_core::lighting::SurfaceLighting;
use thessa_render_core::mesh::*;
use thessa_render_wgpu::{Backend, mesh::MeshRenderer, terrain::OffscreenTarget, view::ViewParams};

fn source() -> MeshAsset {
    MeshAsset {
        primitives: vec![Primitive {
            ambient_visibility: vec![],
            vertices: [[-0.3, -0.3, 0.5], [0.3, -0.3, 0.5], [0.0, 0.3, 0.5]]
                .map(|position| Vertex {
                    position,
                    normal: [0.0, 0.0, 1.0],
                    uv: [0.5; 2],
                })
                .to_vec(),
            indices: vec![0, 1, 2],
            material: 0,
        }],
        materials: vec![Material {
            base_texture: Some(0),
            // This oracle isolates diffuse sRGB sampling. glTF's material
            // default is a metal, which has no diffuse term with reflection off.
            metallic: 0.0,
            double_sided: true,
            ..Default::default()
        }],
        textures: vec![Texture {
            image: 0,
            wrap: [Wrap::Repeat; 2],
            linear: false,
            min_linear: false,
        }],
        images: vec![Image {
            width: 1,
            height: 1,
            rgba8: vec![128, 64, 32, 255],
        }],
    }
}
fn view(camera: DVec3) -> ViewParams {
    ViewParams {
        world_from_view: DMat4::from_translation(camera),
        world_position_m: camera,
        clip_from_view: DMat4::IDENTITY,
        exposure: 1.0,
    }
}
#[test]
fn textured_instances_move_rebase_keep_depth_and_reject_failed_updates_atomically() {
    let backend = Backend::bring_up().unwrap();
    let target = OffscreenTarget::new(&backend, 129, 129).unwrap();
    let mut renderer = MeshRenderer::new(&backend, &source(), OffscreenTarget::FORMAT, 2).unwrap();
    let light = SurfaceLighting {
        ambient_lux: [
            std::f32::consts::PI,
            std::f32::consts::PI,
            std::f32::consts::PI,
            0.0,
        ],
        ..Default::default()
    };
    let instance = MeshInstance {
        position_m: DVec3::ZERO,
        rotation: DQuat::IDENTITY,
        scale: 1.0,
    };
    let render = |renderer: &mut MeshRenderer, depth: f32| {
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        {
            let _pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: None,
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &target.color,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.0,
                            g: 0.0,
                            b: 1.0,
                            a: 1.0,
                        }),
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
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
        }
        renderer.encode(&mut encoder, &target.color, &target.depth);
        backend.queue.submit([encoder.finish()]);
        target.read_rgba8(&backend).unwrap()
    };
    renderer
        .update(&backend, &view(DVec3::ZERO), &light, &[instance])
        .unwrap();
    let pixels = render(&mut renderer, 1.0);
    assert_eq!(renderer.draw_stats().triangles, 1);
    let indirect = backend
        .device
        .features()
        .contains(wgpu::Features::INDIRECT_FIRST_INSTANCE)
        && backend
            .adapter
            .get_downlevel_capabilities()
            .flags
            .contains(wgpu::DownlevelFlags::INDIRECT_EXECUTION);
    assert_eq!(
        renderer.draw_stats().frame_upload_bytes,
        640 + usize::from(indirect) * 20
    );
    renderer
        .update(
            &backend,
            &view(DVec3::ZERO),
            &light,
            &[MeshInstance {
                rotation: DQuat::from_rotation_z(1.0),
                ..instance
            }],
        )
        .unwrap();
    assert_ne!(
        render(&mut renderer, 1.0),
        pixels,
        "rotation must reach the GPU instance transform"
    );
    renderer
        .update(&backend, &view(DVec3::ZERO), &light, &[instance])
        .unwrap();
    let center = (64 * 129 + 64) * 4;
    assert_eq!(&pixels[center..center + 4], &[128, 64, 32, 255]);
    // Cover the all-two-sided specialization and mixed-material fallback with
    // both triangle windings, including a two-sided primitive in a mixed asset.
    for (double_sided, mixed) in [(false, false), (true, false), (true, true)] {
        for reverse in [false, true] {
            let mut asset = source();
            asset.materials[0].double_sided = double_sided;
            if mixed {
                asset.materials.push(Material::default());
            }
            if reverse {
                asset.primitives[0].indices.reverse();
            }
            let mut sided =
                MeshRenderer::new(&backend, &asset, OffscreenTarget::FORMAT, 1).unwrap();
            sided
                .update(&backend, &view(DVec3::ZERO), &light, &[instance])
                .unwrap();
            let rgba = render(&mut sided, 1.0);
            assert_eq!(
                &rgba[center..center + 4],
                if reverse && !double_sided {
                    &[0, 0, 255, 255]
                } else {
                    &[128, 64, 32, 255]
                },
                "double_sided={double_sided} mixed={mixed} reverse={reverse}",
            );
        }
    }
    // The native plant adapter reuses this exact base-texture path as an
    // organ palette: linear filtering at a constant texel-centre UV must not
    // bleed green foliage into the stem, or vice versa, even with minification.
    let foliage = [43, 71, 26, 255];
    let stem = [74, 64, 43, 255];
    for (u, expected) in [(0.25, foliage), (0.75, stem)] {
        let mut palette = source();
        palette.images[0].width = 2;
        palette.images[0].rgba8 = [foliage, stem].concat();
        palette.textures[0].wrap = [Wrap::Clamp; 2];
        palette.textures[0].linear = true;
        palette.textures[0].min_linear = true;
        for vertex in &mut palette.primitives[0].vertices {
            vertex.uv = [u, 0.5];
        }
        let mut palette_renderer =
            MeshRenderer::new(&backend, &palette, OffscreenTarget::FORMAT, 1).unwrap();
        for scale in [1.0, 0.05] {
            palette_renderer
                .update(
                    &backend,
                    &view(DVec3::ZERO),
                    &light,
                    &[MeshInstance { scale, ..instance }],
                )
                .unwrap();
            let rgba = render(&mut palette_renderer, 1.0);
            for (&actual, expected) in rgba[center..center + 4].iter().zip(expected) {
                assert!(
                    actual.abs_diff(expected) <= 1,
                    "palette must preserve its organ swatch at scale={scale}: {:?}",
                    &rgba[center..center + 4]
                );
            }
        }
    }
    assert_eq!(&pixels[..4], &[0, 0, 255, 255]);
    assert_eq!(
        &render(&mut renderer, 0.4)[center..center + 4],
        &[0, 0, 255, 255],
        "must respect pre-existing terrain depth"
    );
    let far = DVec3::new(1.5e11, -1e12, 2e12);
    renderer
        .update(
            &backend,
            &view(far),
            &light,
            &[MeshInstance {
                position_m: far,
                ..instance
            }],
        )
        .unwrap();
    assert_eq!(render(&mut renderer, 1.0), pixels);
    let moved = MeshInstance {
        position_m: far + DVec3::new(0.5, 0.0, 0.0),
        ..instance
    };
    renderer
        .update(&backend, &view(far), &light, &[moved])
        .unwrap();
    let moving = render(&mut renderer, 1.0);
    assert_ne!(moving, pixels);
    let mut invalid_view = view(far);
    invalid_view.exposure = f32::NAN;
    assert!(
        renderer
            .update(&backend, &invalid_view, &light, &[instance])
            .is_err()
    );
    assert_eq!(render(&mut renderer, 1.0), moving);
    let mut invalid_light = light;
    invalid_light.ambient_lux[0] = f32::NAN;
    assert!(
        renderer
            .update(&backend, &view(far), &invalid_light, &[instance])
            .is_err()
    );
    assert_eq!(render(&mut renderer, 1.0), moving);
    let bad = MeshInstance {
        scale: f64::NAN,
        ..instance
    };
    assert!(
        renderer
            .update(&backend, &view(far), &light, &[instance, bad])
            .is_err()
    );
    assert_eq!(render(&mut renderer, 1.0), moving);
    assert!(
        renderer
            .update(&backend, &view(far), &light, &[instance; 3])
            .is_err()
    );
    assert_eq!(render(&mut renderer, 1.0), moving);
    renderer
        .update(
            &backend,
            &view(DVec3::ZERO),
            &light,
            &[
                MeshInstance {
                    position_m: DVec3::new(-0.5, 0.0, 0.0),
                    ..instance
                },
                MeshInstance {
                    position_m: DVec3::new(0.5, 0.0, 0.0),
                    ..instance
                },
            ],
        )
        .unwrap();
    let multiple = render(&mut renderer, 1.0);
    assert_eq!(renderer.draw_stats().instances, 2);
    assert_eq!(renderer.draw_stats().draw_calls, 1);
    let changed = |p: &[u8]| {
        p.as_chunks::<4>()
            .0
            .iter()
            .filter(|p| p != &&[0, 0, 255, 255])
            .count()
    };
    assert!(changed(&multiple) > changed(&pixels));
    renderer
        .update(&backend, &view(DVec3::ZERO), &light, &[])
        .unwrap();
    assert_eq!(changed(&render(&mut renderer, 1.0)), 0);
}

#[test]
fn dielectric_fresnel_parameter_reaches_the_primary_shader() {
    let backend = Backend::bring_up().unwrap();
    let target = OffscreenTarget::new(&backend, 65, 65).unwrap();
    for f0 in [((1.333_f32 - 1.0) / (1.333 + 1.0)).powi(2), 0.04, 0.25] {
        let mut asset = source();
        asset.materials[0].base_texture = None;
        asset.materials[0].dielectric_f0 = f0;
        for v in &mut asset.primitives[0].vertices {
            v.normal = [0.0, 0.0, -1.0];
        }
        let mut renderer = MeshRenderer::with_surface(
            &backend,
            &asset,
            thessa_render_wgpu::mesh::SurfaceSettings {
                format: OffscreenTarget::FORMAT,
                capacity: 1,
                quality: thessa_graphics::Quality::High,
                reflections: false,
                shadows: None,
                ray_tracing: None,
            },
        )
        .unwrap();
        let light = SurfaceLighting {
            ambient_lux: [
                std::f32::consts::PI,
                std::f32::consts::PI,
                std::f32::consts::PI,
                0.0,
            ],
            ..Default::default()
        };
        renderer
            .update(
                &backend,
                &view(DVec3::ZERO),
                &light,
                &[MeshInstance {
                    position_m: DVec3::ZERO,
                    rotation: DQuat::IDENTITY,
                    scale: 1.0,
                }],
            )
            .unwrap();
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        {
            let _pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: None,
                color_attachments: &[],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &target.depth,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
        }
        renderer.encode(&mut encoder, &target.color, &target.depth);
        backend.queue.submit([encoder.finish()]);
        let pixels = target.read_rgba8(&backend).unwrap();
        let linear = 1.0 - f64::from(f0);
        let expected = ((1.055 * linear.powf(1.0 / 2.4) - 0.055) * 255.0).round() as u8;
        assert!(
            pixels[(32 * 65 + 32) * 4].abs_diff(expected) <= 1,
            "F0={f0}: {} vs {expected}",
            pixels[(32 * 65 + 32) * 4]
        );
    }
}
