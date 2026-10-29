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
fn pixel(backend: &Backend, meshes: &[&MeshRenderer], global: bool) -> [u16; 4] {
    let target = OffscreenTarget::new(backend, 16, 16).unwrap();
    let hdr = backend.device.create_texture(&wgpu::TextureDescriptor {
        label: Some("transparency-hdr-oracle"),
        size: wgpu::Extent3d {
            width: 16,
            height: 16,
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
    let mut encoder = backend.device.create_command_encoder(&Default::default());
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
            MeshRenderer::draw_scene_into_pass(meshes, &mut pass);
        } else {
            meshes[0].draw_into_pass(&mut pass);
        }
    }
    backend.queue.submit([encoder.finish()]);
    let bytes = thessa_render_wgpu::readback::read_color(backend, &hdr).unwrap();
    std::array::from_fn(|i| {
        u16::from_le_bytes(
            bytes[(8 * 16 + 8) * 8 + i * 2..(8 * 16 + 8) * 8 + i * 2 + 2]
                .try_into()
                .unwrap(),
        )
    })
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
