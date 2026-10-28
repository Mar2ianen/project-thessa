// SPDX-License-Identifier: MIT
//! Compare the real primary pass with its original, unfiltered draw sequence.
use super::*;
use crate::terrain::OffscreenTarget;
use glam::{DMat4, DQuat, DVec3};
use thessa_render_core::{mesh::*, temporal::JitterOffset};

fn capture(
    mesh: &MeshRenderer,
    backend: &Backend,
    target: &OffscreenTarget,
    cull: bool,
) -> (Vec<u8>, Vec<u8>) {
    let width = 257u32;
    let height = 193u32;
    let stride = (width * 4).next_multiple_of(wgpu::COPY_BYTES_PER_ROW_ALIGNMENT);
    let buffer = backend.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("primary-oracle-depth"),
        size: u64::from(stride * height),
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    {
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("primary-oracle"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: &target.color,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color {
                        r: 0.05,
                        g: 0.08,
                        b: 0.12,
                        a: 1.0,
                    }),
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
            timestamp_writes: None,
            occlusion_query_set: None,
            multiview_mask: None,
        });
        mesh.draw_with_visibility(&mut pass, cull);
    }
    encoder.copy_texture_to_buffer(
        target.depth_texture.as_image_copy(),
        wgpu::TexelCopyBufferInfo {
            buffer: &buffer,
            layout: wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(stride),
                rows_per_image: Some(height),
            },
        },
        wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        },
    );
    backend.queue.submit([encoder.finish()]);
    let padded = crate::terrain::map_buffer(backend, &buffer).unwrap();
    let depth = padded
        .chunks_exact(stride as usize)
        .flat_map(|row| row[..width as usize * 4].iter().copied())
        .collect();
    (target.read_rgba8(backend).unwrap(), depth)
}

#[test]
#[ignore = "software Vulkan: exact primary colour/depth versus all original instance ranges"]
fn primary_frustum_matches_full_draws_at_large_origins_and_jitter() {
    assert!(
        !std::path::Path::new("/dev/dri").exists(),
        "use device-isolated software Vulkan"
    );
    let backend = Backend::bring_up().unwrap();
    assert_eq!(
        backend.adapter.get_info().device_type,
        wgpu::DeviceType::Cpu
    );
    let target = OffscreenTarget::new(&backend, 257, 193).unwrap();
    let primitive = thessa_render_core::mesh::Primitive {
        vertices: [[-0.5, -0.3, 0.0], [0.5, -0.3, 0.0], [0.0, 0.7, 0.0]]
            .map(|position| Vertex {
                position,
                normal: [0.0, 0.0, 1.0],
                uv: [0.0; 2],
            })
            .to_vec(),
        ambient_visibility: vec![],
        indices: vec![0, 1, 2],
        material: 0,
    };
    let source = MeshAsset {
        primitives: vec![primitive.clone(), primitive],
        materials: vec![Material {
            base_color: [0.15, 0.35, 0.08, 1.0],
            double_sided: true,
            ..Default::default()
        }],
        textures: vec![],
        images: vec![],
    };
    let mut mesh = MeshRenderer::new(&backend, &source, OffscreenTarget::FORMAT, 192).unwrap();
    let mut direct = MeshRenderer::new(&backend, &source, OffscreenTarget::FORMAT, 192).unwrap();
    direct.primary_indirect_supported = false;
    // Exercise cached uploaded-model bounds while the independent direct/full
    // oracle retains the original transform-per-test path and same shader.
    mesh.cache_bounds = true;
    let batch_supported = mesh.primary_indirect_supported;
    let mut witnessed_batching = false;
    let lights = SurfaceLighting {
        ambient_lux: [0.9, 0.8, 0.7, 0.0],
        ..Default::default()
    };
    let mut witnessed_rejection = false;
    for origin in [
        DVec3::ZERO,
        DVec3::new(3.2e6, -2e6, 4e6),
        DVec3::new(3e12, -2e12, 1e12),
    ] {
        let mut instances: Vec<_> = (0..192)
            .map(|i| {
                let a = i as f64 * 0.47;
                MeshInstance {
                    position_m: origin
                        + DVec3::new(a.sin() * 18.0, (a * 0.7).cos() * 4.0, a.cos() * 18.0),
                    rotation: DQuat::from_rotation_y(a * 0.3),
                    scale: 0.1 + (i % 9) as f64 * 0.6,
                }
            })
            .collect();
        instances[0] = MeshInstance {
            position_m: origin + DVec3::new(0.0, 0.0, -2.0),
            rotation: DQuat::IDENTITY,
            scale: 5.0,
        };
        for yaw in [0.0, 0.4, 1.7] {
            let view = ViewParams {
                world_position_m: origin,
                world_from_view: DMat4::from_rotation_translation(
                    DQuat::from_rotation_y(yaw),
                    origin,
                ),
                clip_from_view: glam::dcamera::rh::proj::directx::perspective(
                    0.9f64,
                    257.0 / 193.0,
                    0.1,
                    500.0,
                ),
                exposure: 1.0,
            };
            for jitter in [
                JitterOffset::ZERO,
                JitterOffset {
                    pixels: [0.49, -0.49],
                },
            ] {
                mesh.update_selected(
                    &backend,
                    &view,
                    &lights,
                    &instances,
                    Some(&[0..128, 64..192]),
                )
                .unwrap();
                mesh.update_primary_jitter(&backend, &view, [257, 193], jitter)
                    .unwrap();
                direct
                    .update_selected(
                        &backend,
                        &view,
                        &lights,
                        &instances,
                        Some(&[0..128, 64..192]),
                    )
                    .unwrap();
                direct
                    .update_primary_jitter(&backend, &view, [257, 193], jitter)
                    .unwrap();
                let stats = mesh.draw_stats();
                assert_eq!(stats.triangles, direct.draw_stats().triangles);
                witnessed_batching |= stats.draw_calls < direct.draw_stats().draw_calls;
                assert_eq!(stats.candidate_instances, 192);
                assert_eq!(stats.candidate_triangles, 256);
                assert!(stats.instances > 0 && stats.triangles > 0);
                witnessed_rejection |= stats.triangles < stats.candidate_triangles;
                let full = capture(&mesh, &backend, &target, false);
                let culled = capture(&mesh, &backend, &target, true);
                assert_eq!(
                    full, culled,
                    "origin={origin:?} yaw={yaw} jitter={jitter:?}"
                );
                assert_eq!(
                    full,
                    capture(&direct, &backend, &target, true),
                    "direct fallback"
                );
                let before = mesh.draw_stats();
                assert!(
                    mesh.update_selected(
                        &backend,
                        &view,
                        &lights,
                        &instances,
                        Some(&[0..193, 0..1])
                    )
                    .is_err()
                );
                assert_eq!(
                    mesh.draw_stats(),
                    before,
                    "invalid publication must retain its draw plan"
                );
            }
        }
        mesh.update_selected(
            &backend,
            &ViewParams {
                world_position_m: origin,
                world_from_view: DMat4::from_translation(origin),
                clip_from_view: glam::dcamera::rh::proj::directx::perspective(0.9, 1.0, 0.1, 500.0),
                exposure: 1.0,
            },
            &lights,
            &instances,
            Some(&[0..64, 128..192]),
        )
        .unwrap();
        assert_eq!(
            mesh.draw_stats().candidate_triangles,
            128,
            "range invalidation"
        );
        instances[0].position_m += DVec3::new(200.0, 0.0, 0.0);
        let moved_view = ViewParams {
            world_position_m: origin,
            world_from_view: DMat4::from_translation(origin),
            clip_from_view: glam::dcamera::rh::proj::directx::perspective(0.9, 1.0, 0.1, 500.0),
            exposure: 1.0,
        };
        mesh.update_selected(
            &backend,
            &moved_view,
            &lights,
            &instances,
            Some(&[0..64, 128..192]),
        )
        .unwrap();
        assert_eq!(
            capture(&mesh, &backend, &target, false),
            capture(&mesh, &backend, &target, true),
            "instance invalidation"
        );
        let mut invalid = instances.clone();
        invalid[128].scale = f64::NAN;
        let before_failure = mesh.draw_stats();
        assert!(
            mesh.update_selected(
                &backend,
                &moved_view,
                &lights,
                &invalid,
                Some(&[0..64, 128..192])
            )
            .is_err()
        );
        assert_eq!(mesh.draw_stats(), before_failure);
        mesh.update_selected(
            &backend,
            &moved_view,
            &lights,
            &instances,
            Some(&[0..64, 128..192]),
        )
        .unwrap();
        assert_eq!(
            mesh.draw_stats().frame_upload_bytes,
            576,
            "unchanged source/plan must reuse GPU buffers"
        );
        mesh.update(
            &backend,
            &ViewParams {
                world_position_m: origin,
                world_from_view: DMat4::from_translation(origin),
                clip_from_view: glam::dcamera::rh::proj::directx::perspective(
                    0.9f64, 1.0, 0.1, 500.0,
                ),
                exposure: 1.0,
            },
            &lights,
            &[],
        )
        .unwrap();
        assert_eq!(mesh.draw_stats().instances, 0);
        assert_eq!(mesh.draw_stats().triangles, 0);
    }
    assert!(witnessed_rejection);
    assert!(!batch_supported || witnessed_batching);
    eprintln!(
        "exact primary colour/depth: indirect_supported={batch_supported}, witnessed_batching={witnessed_batching}"
    );
}
