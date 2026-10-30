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
        vertices: [
            ([-1., -1., z], [0., 1.]),
            ([1., -1., z], [1., 1.]),
            ([1., 1., z], [1., 0.]),
            ([-1., 1., z], [0., 0.]),
        ]
        .map(|(position, uv)| Vertex {
            position,
            normal: [0., 0., 1.],
            uv,
        })
        .to_vec(),
        indices: vec![0, 1, 2, 0, 2, 3],
    }
}

#[test]
fn masked_coverage_preserves_holes_depth_factors_and_opaque_alpha_semantics() {
    let backend = Backend::bring_up().unwrap();
    let mut asset = MeshAsset {
        // Background is deliberately drawn last: transparent texels must not
        // write foreground depth and prevent the blue surface from appearing.
        primitives: vec![quad(-0.25, 0), quad(-0.75, 1)],
        materials: vec![
            Material {
                base_texture: Some(0),
                alpha_mode: AlphaMode::Mask,
                alpha_cutoff: 0.5,
                base_color: [1., 1., 1., 0.5],
                metallic: 0.,
                double_sided: true,
                ..Default::default()
            },
            Material {
                base_color: [0., 0., 1., 0.],
                metallic: 0.,
                double_sided: false,
                ..Default::default()
            },
        ],
        textures: vec![Texture {
            image: 0,
            wrap: [Wrap::Clamp; 2],
            linear: false,
            min_linear: false,
        }],
        images: vec![Image {
            width: 2,
            height: 1,
            rgba8: vec![255, 0, 0, 0, 255, 0, 0, 255],
        }],
    };
    let view = ViewParams {
        world_from_view: DMat4::IDENTITY,
        world_position_m: DVec3::ZERO,
        clip_from_view: DMat4::from_scale(DVec3::new(1., 1., -1.)),
        exposure: 4.,
    };
    let light = SurfaceLighting {
        ambient_lux: [std::f32::consts::PI; 4],
        ..Default::default()
    };
    asset.materials.push(Material {
        double_sided: false,
        ..asset.materials[0].clone()
    });
    for (quality, double_sided) in [
        (Quality::Low, true),
        (Quality::High, true),
        (Quality::High, false),
    ] {
        let masked_material = if double_sided { 0 } else { 2 };
        asset.primitives[0].material = masked_material;
        for (cutoff, factor, expected) in [
            (0.5, 0.5, [[0, 0, 255], [255, 0, 0]]),
            (0.5, 0.49, [[0, 0, 255], [0, 0, 255]]),
            (0., 0., [[255, 0, 0], [255, 0, 0]]),
            (1.5, 1., [[0, 0, 255], [0, 0, 255]]),
        ] {
            asset.materials[masked_material].alpha_cutoff = cutoff;
            asset.materials[masked_material].base_color[3] = factor;
            let mut renderer = MeshRenderer::with_surface(
                &backend,
                &asset,
                SurfaceSettings {
                    format: OffscreenTarget::FORMAT,
                    capacity: 1,
                    quality,
                    reflections: false,
                    shadows: None,
                    ray_tracing: None,
                },
            )
            .unwrap();
            renderer
                .update(
                    &backend,
                    &view,
                    &light,
                    &[MeshInstance {
                        position_m: DVec3::ZERO,
                        rotation: DQuat::IDENTITY,
                        scale: 1.,
                    }],
                )
                .unwrap();
            let target = OffscreenTarget::new(&backend, 32, 16).unwrap();
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            {
                let _clear = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                    color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                        view: &target.color,
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
            renderer.encode(&mut encoder, &target.color, &target.depth);
            backend.queue.submit([encoder.finish()]);
            let pixels = target.read_rgba8(&backend).unwrap();
            for (x, color) in [8, 24].into_iter().zip(expected) {
                let index = (8 * 32 + x) * 4;
                for (actual, expected) in pixels[index..index + 3].iter().zip(color) {
                    assert!(
                        actual.abs_diff(expected) <= 1,
                        "quality={quality:?} cutoff={cutoff} factor={factor}: {:?} vs {color:?}",
                        &pixels[index..index + 4]
                    );
                }
            }
        }
    }
    let settings = thessa_graphics::ResolvedGraphicsSettings {
        shadow_enabled: true,
        native_shadow_map_size: 64,
        ..Default::default()
    };
    let atlas = thessa_render_wgpu::shadows::ShadowAtlas::new(&backend, &settings).unwrap();
    let result = MeshRenderer::with_surface(
        &backend,
        &asset,
        SurfaceSettings {
            format: OffscreenTarget::FORMAT,
            capacity: 1,
            quality: Quality::High,
            reflections: false,
            shadows: atlas.as_ref(),
            ray_tracing: None,
        },
    );
    assert!(
        matches!(result,Err(thessa_render_wgpu::BackendError::Device(reason)) if reason.contains("MASK shadow"))
    );
    let rt = thessa_graphics::ResolvedGraphicsSettings {
        ray_tracing: thessa_graphics::ResolvedRayTracing::Local,
        rt_vehicles: true,
        ..Default::default()
    };
    assert!(
        matches!(thessa_render_wgpu::ray_scene::plan(&backend,&asset,Some(&rt),1,Quality::High,true),
        thessa_render_wgpu::ray_scene::Enable::Unavailable(reason) if reason.contains("MASK/BLEND ray"))
    );
}
