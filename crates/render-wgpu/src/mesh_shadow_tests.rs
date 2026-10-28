//! Actual multi-layer direct/indirect shadow equivalence, including mixed LODs.
use super::*;
use crate::terrain::{OffscreenTarget, TerrainCapacity, TerrainPatch, TerrainRenderer};
use glam::{DMat4, DQuat, DVec3};
use thessa_render_core::mesh::*;

#[test]
#[ignore = "software Vulkan: shadow indirect/direct depth and colour across layers, mixed geometry and budget fallback"]
fn shadow_indirect_matches_direct_for_every_layer_and_reuses_uploads() {
    assert!(!std::path::Path::new("/dev/dri").exists());
    let backend = Backend::bring_up().unwrap();
    assert_eq!(
        backend.adapter.get_info().device_type,
        wgpu::DeviceType::Cpu
    );
    let mut request = thessa_graphics::RequestedGraphics::default();
    request.renderer.ray_tracing = thessa_graphics::RayTracingRequest::Off;
    request.shadows.map_size = 512;
    request.shadows.max_distance_m = 1200.0;
    request.shadows.first_cascade_distance_m = Some(25.0);
    request.shadows.mesh_lod_error_texels = 0.5;
    request.shadows.receiver_culling = true;
    request.shadows.raster_sample_culling = true;
    let resolved = backend.resolve_graphics(&request);
    let page = thessa_rcbt_core::HeightPage::bake(&[0.0; 4], 2, 0.001).unwrap();
    let mut terrain = TerrainRenderer::with_capacity_and_lighting(
        &backend,
        &[TerrainPatch {
            anchor: thessa_rcbt_core::tile_anchor::TileAnchor::new(
                thessa_rcbt_core::tile_anchor::TileKey::new(0, 0, 0, 0).unwrap(),
                1000.0,
            )
            .unwrap(),
            page: &page,
        }],
        OffscreenTarget::FORMAT,
        TerrainCapacity {
            patches: 1,
            height_grid_size: 2,
            material_layers: 1,
        },
        Some(&resolved),
    )
    .unwrap();
    terrain.publish_partition(&backend, &[]).unwrap();
    let primitive = thessa_render_core::mesh::Primitive {
        ambient_visibility: vec![],
        vertices: [
            [-0.5, -0.5, -4.0],
            [0.5, -0.5, -4.0],
            [0.5, 0.5, -4.0],
            [-0.5, 0.5, -4.0],
        ]
        .map(|position| Vertex {
            position,
            normal: [0.0, 0.0, 1.0],
            uv: [0.0; 2],
        })
        .to_vec(),
        indices: vec![0, 1, 2, 0, 2, 3, 0, 1, 2, 0, 2, 3],
        material: 0,
    };
    let source = MeshAsset {
        primitives: vec![primitive.clone(), primitive],
        materials: vec![Material {
            base_color: [0.1, 0.8, 0.2, 1.0],
            double_sided: true,
            metallic: 0.0,
            ..Default::default()
        }],
        textures: vec![],
        images: vec![],
    };
    let settings = || SurfaceSettings {
        format: OffscreenTarget::FORMAT,
        capacity: 64,
        quality: thessa_graphics::Quality::High,
        reflections: false,
        shadows: terrain.shadows(),
        ray_tracing: Some(&resolved),
    };
    let mut batched =
        MeshRenderer::with_surface_and_terrain(&backend, &source, settings(), None).unwrap();
    let mut direct =
        MeshRenderer::with_surface_and_terrain(&backend, &source, settings(), None).unwrap();
    assert!(
        batched.primary_indirect_supported,
        "pinned software adapter must exercise indexed indirect first_instance"
    );
    direct.primary_indirect_supported = false;
    let mut lods: Vec<_> = (0..2)
        .map(|primitive| ShadowMeshLod {
            primitive,
            positions: vec![[-0.5, -0.5, -4.0], [0.5, -0.5, -4.0], [0.5, 0.5, -4.0]],
            indices: vec![0, 1, 2],
            max_error_m: 0.71,
        })
        .collect();
    {
        let primitive = 0;
        lods.push(ShadowMeshLod {
            primitive,
            positions: source.primitives[primitive]
                .vertices
                .iter()
                .map(|v| v.position)
                .collect(),
            indices: vec![0, 1, 2, 0, 2, 3],
            max_error_m: 0.0,
        });
    }
    batched.set_shadow_lods(&backend, &lods).unwrap();
    direct.set_shadow_lods(&backend, &lods).unwrap();
    let target = OffscreenTarget::new(&backend, 257, 193).unwrap();
    let lights = SurfaceLighting {
        directions: [
            DVec3::Z,
            DVec3::new(0.6, 0.2, 0.7).normalize(),
            DVec3::new(-0.2, 0.8, 0.4).normalize(),
        ]
        .map(|d| [d.x as f32, d.y as f32, d.z as f32, 0.0]),
        colors_lux: [[3.0; 4]; 3],
        ambient_lux: [0.2; 4],
    };
    let capture = |terrain: &mut TerrainRenderer, mesh: &MeshRenderer, samples: bool| {
        terrain.invalidate_shadows();
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        if samples {
            terrain.encode_shadows(&mut encoder, Some(mesh));
        } else {
            terrain.encode_shadows_without_sample_culling(&mut encoder, Some(mesh));
        }
        let stats = terrain.last_shadow_draws();
        terrain.encode_scene(&mut encoder, &target.color, &target.depth, Some(mesh));
        backend.queue.submit([encoder.finish()]);
        (
            terrain.shadows().unwrap().read_depth(&backend).unwrap(),
            target.read_rgba8(&backend).unwrap(),
            stats,
        )
    };
    let mut witnessed_samples = false;
    let mut witnessed_lod = false;
    let mut witnessed_batch = false;
    let mut witnessed_full = false;
    let mut witnessed_fine = false;
    let mut witnessed_coarse = false;
    for (case, origin) in [DVec3::ZERO, DVec3::new(3e12, -2e12, 1e12)]
        .into_iter()
        .enumerate()
    {
        let view = ViewParams {
            world_from_view: DMat4::from_translation(origin),
            world_position_m: origin,
            clip_from_view: glam::dcamera::rh::proj::directx::perspective(
                1.1,
                257.0 / 193.0,
                0.1,
                1200.0,
            ),
            exposure: 0.5,
        };
        let instances: Vec<_> = (0..64)
            .map(|i| {
                let t = i as f64 * 1.7;
                MeshInstance {
                    position_m: origin
                        + DVec3::new(
                            t.sin() * 90.0,
                            (t * 0.3).cos() * 10.0,
                            -40.0 + (t * 0.7).cos() * 50.0,
                        ),
                    rotation: DQuat::from_rotation_y(t * 0.2),
                    scale: 0.2 + (i % 8) as f64 * 0.5,
                }
            })
            .collect();
        terrain.update_view(&backend, &view).unwrap();
        terrain.update_lighting(&backend, &lights).unwrap();
        terrain.update_shadows(&backend, origin, &lights).unwrap();
        batched
            .update(&backend, &view, &lights, &instances)
            .unwrap();
        direct.primary_indirect_supported = false;
        direct.shadow_command_budget = resolved.shadow_mesh_command_budget_bytes;
        direct.update(&backend, &view, &lights, &instances).unwrap();
        assert!(batched.shadow_indirect_storage_bytes() > 0);
        assert!(
            batched.shadow_indirect_storage_bytes() <= resolved.shadow_mesh_command_budget_bytes
        );
        assert_eq!(direct.shadow_indirect_storage_bytes(), 0);
        let unculled = capture(&mut terrain, &batched, false);
        assert_eq!(unculled.2.mesh_sample_rejected_instances, 0);
        let a = capture(&mut terrain, &batched, true);
        assert_eq!(
            unculled.0, a.0,
            "sample culling changed depth at case {case}"
        );
        assert_eq!(
            unculled.1, a.1,
            "sample culling changed colour at case {case}"
        );
        assert!(a.2.mesh_submitted_triangles < unculled.2.mesh_submitted_triangles);
        witnessed_samples |= a.2.mesh_sample_rejected_instances > 0;
        {
            let commands = batched.shadow_commands.lock().unwrap();
            for data in &commands.uploaded {
                for command in data.as_chunks::<20>().0 {
                    let count = u32::from_le_bytes(command[..4].try_into().unwrap());
                    witnessed_full |= count == 12;
                    witnessed_fine |= count == 6;
                    witnessed_coarse |= count == 3;
                }
            }
        }
        let b = capture(&mut terrain, &direct, true);
        assert_eq!(a.0, b.0, "direct/indirect atlas differs at case {case}");
        assert_eq!(
            a.1, b.1,
            "direct/indirect final colour differs at case {case}"
        );
        assert!(a.2.mesh_indirect && !b.2.mesh_indirect);
        assert_eq!(a.2.mesh_submitted_triangles, b.2.mesh_submitted_triangles);
        assert_eq!(a.2.mesh_draw_commands, b.2.mesh_draw_commands);
        assert_eq!(b.2.mesh_draw_calls, b.2.mesh_draw_commands);
        witnessed_lod |= a.2.mesh_lod_instances > 0;
        witnessed_batch |= a.2.mesh_draw_calls < a.2.mesh_draw_commands;
        if case == 0 {
            assert!(a.2.mesh_indirect_upload_bytes > 0);
        }
        let reused = capture(&mut terrain, &batched, true);
        assert_eq!(a.0, reused.0);
        assert_eq!(a.1, reused.1);
        assert_eq!(reused.2.mesh_indirect_upload_bytes, 0);
        // A supported device with an undersized command budget uses direct
        // draws without changing geometry, depth or instance identities.
        direct.primary_indirect_supported = true;
        direct.shadow_command_budget = 1;
        direct.update(&backend, &view, &lights, &instances).unwrap();
        assert_eq!(direct.shadow_indirect_storage_bytes(), 0);
        let fallback = capture(&mut terrain, &direct, true);
        assert_eq!(a.0, fallback.0);
        assert_eq!(a.1, fallback.1);
        let budget = terrain.shadows().unwrap().command_available.clone();
        let unused = *budget.lock().unwrap();
        assert_eq!(
            unused + batched.shadow_indirect_storage_bytes(),
            resolved.shadow_mesh_command_budget_bytes
        );
        *budget.lock().unwrap() = 0;
        direct.shadow_command_budget = resolved.shadow_mesh_command_budget_bytes;
        direct.update(&backend, &view, &lights, &instances).unwrap();
        assert_eq!(direct.shadow_indirect_storage_bytes(), 0);
        *budget.lock().unwrap() = unused;
        eprintln!(
            "shadow_indirect case={case} triangles={} commands={} batches={} upload_bytes={} storage_bytes={} sample_rejected={} no_sample_triangles={}",
            a.2.mesh_submitted_triangles,
            a.2.mesh_draw_commands,
            a.2.mesh_draw_calls,
            a.2.mesh_indirect_upload_bytes,
            batched.shadow_indirect_storage_bytes(),
            a.2.mesh_sample_rejected_instances,
            unculled.2.mesh_submitted_triangles
        );
    }
    assert!(
        witnessed_samples
            && witnessed_lod
            && witnessed_batch
            && witnessed_full
            && witnessed_fine
            && witnessed_coarse
    );
    let budget = terrain.shadows().unwrap().command_available.clone();
    drop(batched);
    drop(direct);
    assert_eq!(
        *budget.lock().unwrap(),
        resolved.shadow_mesh_command_budget_bytes
    );
}
