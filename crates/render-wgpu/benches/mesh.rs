//! Identical source/target, clear-only floor vs one/32 static mesh instances.
//! Full host upload/encode/submit/device-completion latency, not GPU timestamp
//! time or displayed FPS. No readback, terrain, UI, streaming or asset IO.
use glam::{DMat4, DQuat, DVec3};
use std::time::Instant;
use thessa_rcbt_core::lighting::SurfaceLighting;
use thessa_render_core::{ViewParams, mesh::MeshInstance};
use thessa_render_wgpu::{Backend, mesh::MeshRenderer, terrain::OffscreenTarget};
fn main() {
    let backend = Backend::bring_up().expect("mesh benchmark requires an adapter");
    let source = thessa_render_assets::decode_glb(
        include_bytes!("../../../assets/models/north_american_x-15_plane.glb"),
        Default::default(),
    )
    .unwrap();
    let target = OffscreenTarget::new(&backend, 1280, 720).unwrap();
    let mut renderer = MeshRenderer::new(&backend, &source, OffscreenTarget::FORMAT, 32).unwrap();
    let view = ViewParams {
        world_from_view: DMat4::IDENTITY,
        world_position_m: DVec3::ZERO,
        clip_from_view: DMat4::IDENTITY,
        exposure: 1.0,
    };
    let instances: Vec<_> = (0..32)
        .map(|i| MeshInstance {
            position_m: DVec3::new(
                -0.875 + (i % 8) as f64 * 0.25,
                -0.75 + (i / 8) as f64 * 0.5,
                0.5,
            ),
            rotation: DQuat::from_rotation_x(0.8),
            scale: 0.012,
        })
        .collect();
    eprintln!(
        "adapter: {}; source: 78465 triangles, source mip only",
        backend.adapter_name
    );
    for count in [0, 1, 32] {
        let mut times = Vec::new();
        for frame in 0..150 {
            let start = Instant::now();
            renderer
                .update(
                    &backend,
                    &view,
                    &SurfaceLighting::default(),
                    &instances[..count],
                )
                .unwrap();
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            {
                let _pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                    label: None,
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
            backend
                .device
                .poll(wgpu::PollType::wait_indefinitely())
                .unwrap();
            if frame >= 30 {
                times.push(start.elapsed().as_secs_f64() * 1000.0);
            }
        }
        times.sort_by(f64::total_cmp);
        println!(
            "instances={count} triangles={} median_ms={:.3} p95_ms={:.3} n=120 warmup=30",
            count * 78465,
            times[60],
            times[114]
        );
    }
}
