//! Actual mesh coverage and single-sample depth, not a screenshot preference.
use glam::{DMat4, DQuat, DVec3};
use thessa_graphics::{Quality, RayTracingRequest, RequestedGraphics};
use thessa_render_core::{ViewParams, mesh::*};
use thessa_render_wgpu::{
    Backend,
    mesh::{MeshRenderer, SurfaceSettings},
    msaa::SceneMsaa,
    terrain::OffscreenTarget,
};

#[test]
fn thin_mesh_coverage_resolves_and_depth_keeps_nearest_covered_sample() {
    let mut request = RequestedGraphics::default();
    request.renderer.ray_tracing = RayTracingRequest::Off;
    request.renderer.specular_reflections = false;
    request.shadows.enabled = false;
    let backend = Backend::bring_up_for_graphics(&request).unwrap();
    let mut column_values = vec![];
    for samples in [1, 4] {
        request.renderer.msaa_samples = samples;
        let settings = backend.resolve_graphics(&request);
        assert_eq!(
            settings.raster_msaa_samples, samples,
            "local hardware must exercise the requested path"
        );
        let mut msaa =
            SceneMsaa::new(&backend, &settings, OffscreenTarget::FORMAT, [64, 64]).unwrap();
        assert_eq!(msaa.is_some(), samples == 4);
        if let Some(msaa) = &mut msaa {
            assert_eq!(msaa.target_bytes(), 64 * 64 * 32);
            assert!(msaa.resize(&backend, [0, 64]).is_err());
            assert_eq!(msaa.target_bytes(), 64 * 64 * 32);
            msaa.resize(&backend, [32, 32]).unwrap();
            msaa.resize(&backend, [64, 64]).unwrap();
        }
        let asset = MeshAsset {
            primitives: vec![Primitive {
                ambient_visibility: vec![],
                vertices: [[32.25, 8.0], [32.75, 8.0], [32.75, 56.0], [32.25, 56.0]]
                    .map(|p| Vertex {
                        position: [p[0] / 32.0 - 1.0, 1.0 - p[1] / 32.0, 0.4],
                        normal: [0.0, 0.0, 1.0],
                        uv: [0.0; 2],
                    })
                    .to_vec(),
                indices: vec![0, 2, 1, 0, 3, 2],
                material: 0,
            }],
            materials: vec![Material {
                base_color: [0.5, 0.5, 0.5, 1.0],
                metallic: 0.0,
                ..Default::default()
            }],
            textures: vec![],
            images: vec![],
        };
        let mut mesh = MeshRenderer::with_surface(
            &backend,
            &asset,
            SurfaceSettings {
                format: OffscreenTarget::FORMAT,
                capacity: 1,
                quality: Quality::Low,
                reflections: false,
                shadows: None,
                ray_tracing: Some(&settings),
            },
        )
        .unwrap();
        mesh.update(
            &backend,
            &ViewParams {
                world_from_view: DMat4::IDENTITY,
                clip_from_view: DMat4::IDENTITY,
                world_position_m: DVec3::ZERO,
                exposure: 1.0,
            },
            &thessa_rcbt_core::lighting::SurfaceLighting {
                ambient_lux: [std::f32::consts::PI; 4],
                directions: [[0.0; 4]; 3],
                colors_lux: [[0.0; 4]; 3],
            },
            &[MeshInstance {
                position_m: DVec3::ZERO,
                rotation: DQuat::IDENTITY,
                scale: 1.0,
            }],
        )
        .unwrap();
        let target = OffscreenTarget::new(&backend, 64, 64).unwrap();
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("thin-coverage-oracle"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: msaa.as_ref().map_or(&target.color, |s| &s.color),
                    depth_slice: None,
                    resolve_target: msaa.as_ref().map(|_| &target.color),
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: msaa.as_ref().map_or(&target.depth, |s| s.depth()),
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                ..Default::default()
            });
            mesh.draw_into_pass(&mut pass);
        }
        if let Some(msaa) = &msaa {
            msaa.resolve_depth(&mut encoder, &target.depth);
        }
        let staging = backend.device.create_buffer(&wgpu::BufferDescriptor {
            label: None,
            size: 64 * 64 * 4,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        encoder.copy_texture_to_buffer(
            wgpu::TexelCopyTextureInfo {
                aspect: wgpu::TextureAspect::DepthOnly,
                ..target.depth_texture.as_image_copy()
            },
            wgpu::TexelCopyBufferInfo {
                buffer: &staging,
                layout: wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(256),
                    rows_per_image: Some(64),
                },
            },
            wgpu::Extent3d {
                width: 64,
                height: 64,
                depth_or_array_layers: 1,
            },
        );
        backend.queue.submit([encoder.finish()]);
        let image = target.read_rgba8(&backend).unwrap();
        let colour = image[((32 * 64 + 32) * 4) as usize];
        column_values.push(colour);
        let slice = staging.slice(..);
        let (tx, rx) = std::sync::mpsc::channel();
        slice.map_async(wgpu::MapMode::Read, move |r| {
            tx.send(r).unwrap();
        });
        backend
            .device
            .poll(wgpu::PollType::wait_indefinitely())
            .unwrap();
        rx.recv().unwrap().unwrap();
        let data = slice.get_mapped_range().unwrap();
        let depth_at = |x, y| {
            f32::from_le_bytes(
                data[((y * 64 + x) * 4) as usize..((y * 64 + x) * 4 + 4) as usize]
                    .try_into()
                    .unwrap(),
            )
        };
        assert!((depth_at(32, 32) - 0.4).abs() < 1e-6);
        assert_eq!(depth_at(31, 32), 1.0);
    }
    assert_eq!(column_values[0], 188);
    assert!(
        (134..=140).contains(&column_values[1]),
        "half-pixel coverage must average in linear light: {column_values:?}"
    );
}
