//! Regression from the Pass 1/2/3 audit: real GPU quiescence and invalidations.
use glam::{DMat4, DQuat, DVec3};
use thessa_graphics::{Quality, RayTracingRequest, RequestedGraphics};
use thessa_rcbt_core::{
    HeightPage,
    lighting::SurfaceLighting,
    tile_anchor::{TileAnchor, TileKey},
};
use thessa_render_core::mesh::{Material, MeshAsset, MeshInstance, Primitive, Vertex};
use thessa_render_wgpu::{
    Backend,
    mesh::{MeshRenderer, SurfaceSettings},
    terrain::{OffscreenTarget, TerrainCapacity, TerrainPatch, TerrainRenderer},
    view::ViewParams,
};

#[test]
#[ignore = "hardware: quiescent topology invalidations and shared shadow cache equivalence"]
fn optimization_review_reproduces_lod_and_shadow_invalidations() {
    let mut request = RequestedGraphics::default();
    request.renderer.ray_tracing = RayTracingRequest::Off;
    request.terrain_topology.enabled = true;
    request.terrain_topology.quality = Quality::Low;
    request.shadows.cascades = 1;
    request.shadows.max_distance_m = 100.0;
    request.shadows.map_size = 512;
    let backend = Backend::bring_up_for_graphics(&request).unwrap();
    let settings = backend.resolve_graphics(&request);
    let page = HeightPage::bake(&[0.0; 4], 2, 0.001).unwrap();
    let patches = (0..6)
        .map(|face| TerrainPatch {
            anchor: TileAnchor::new(TileKey::new(face, 0, 0, 0).unwrap(), 1000.0).unwrap(),
            page: &page,
        })
        .collect::<Vec<_>>();
    let mut terrain = TerrainRenderer::with_capacity_and_lighting(
        &backend,
        &patches,
        OffscreenTarget::FORMAT,
        TerrainCapacity {
            patches: 6,
            height_grid_size: 2,
            material_layers: 1,
        },
        Some(&settings),
    )
    .unwrap();
    let eye = DVec3::new(0.0, 0.0, 1010.0);
    let view = |camera: DVec3, rotation: DQuat| ViewParams {
        world_from_view: DMat4::from_rotation_translation(rotation, camera),
        world_position_m: camera,
        clip_from_view: glam::dcamera::rh::proj::directx::perspective(1.0, 1.0, 0.1, 3000.0),
        exposure: 1.0,
    };
    let light = SurfaceLighting {
        ambient_lux: [0.1, 0.1, 0.1, 0.0],
        directions: [[0.0, 0.0, 1.0, 0.0]; 3],
        colors_lux: [[1.0, 1.0, 1.0, 0.0], [0.0; 4], [0.0; 4]],
    };
    let asset = MeshAsset {
        primitives: vec![Primitive {
            vertices: [
                [-20.0, -20.0, 0.0],
                [20.0, -20.0, 0.0],
                [20.0, 20.0, 0.0],
                [-20.0, 20.0, 0.0],
            ]
            .map(|position| Vertex {
                position,
                normal: [0.0, 0.0, 1.0],
                uv: [0.0; 2],
            })
            .to_vec(),
            indices: vec![0, 1, 2, 0, 2, 3],
            material: 0,
        }],
        materials: vec![Material::default()],
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
            shadows: terrain.shadows(),
            ray_tracing: None,
        },
    )
    .unwrap();
    let instances = [MeshInstance {
        position_m: DVec3::new(0.0, 0.0, 1006.0),
        rotation: DQuat::IDENTITY,
        scale: 1.0,
    }];
    for _ in 0..150 {
        terrain
            .update_view_for_extent(&backend, &view(eye, DQuat::IDENTITY), [257, 257])
            .unwrap();
        terrain.update_shadows(&backend, eye, &light).unwrap();
        mesh.update(&backend, &view(eye, DQuat::IDENTITY), &light, &instances)
            .unwrap();
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        terrain.encode_surface(&mut encoder);
        terrain.encode_shadows(&mut encoder, Some(&mesh));
        backend.queue.submit([encoder.finish()]);
        backend
            .device
            .poll(wgpu::PollType::wait_indefinitely())
            .unwrap();
    }
    let settled = terrain.conforming_surface().unwrap().publication();
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    terrain.encode_surface(&mut encoder);
    backend.queue.submit([encoder.finish()]);
    assert_eq!(
        settled.revision,
        terrain.conforming_surface().unwrap().publication().revision,
        "unchanged completed zero-request wave must sleep"
    );
    let before = terrain.shadows().unwrap().read_depth(&backend).unwrap();
    assert!(
        before
            .as_chunks::<4>()
            .0
            .iter()
            .any(|p| f32::from_le_bytes(*p) < 1.0)
    );
    // A static world-space mesh moves in camera-relative caster coordinates.
    // Its world transform alone cannot key a camera-relative shadow instance.
    let moved = eye + DVec3::X;
    terrain
        .update_view_for_extent(&backend, &view(moved, DQuat::IDENTITY), [257, 257])
        .unwrap();
    terrain.update_shadows(&backend, moved, &light).unwrap();
    mesh.update(&backend, &view(moved, DQuat::IDENTITY), &light, &instances)
        .unwrap();
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    terrain.encode_surface(&mut encoder);
    terrain.encode_shadows(&mut encoder, Some(&mesh));
    backend.queue.submit([encoder.finish()]);
    let passes = terrain.last_shadow_passes();
    let cached = terrain.shadows().unwrap().read_depth(&backend).unwrap();
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    terrain.encode_shadows_reference(&mut encoder, Some(&mesh));
    backend.queue.submit([encoder.finish()]);
    let reference = terrain.shadows().unwrap().read_depth(&backend).unwrap();
    let changed_depth = cached
        .as_chunks::<4>()
        .0
        .iter()
        .zip(reference.as_chunks::<4>().0)
        .filter(|(a, b)| a != b)
        .count();
    println!(
        "AUDIT shadow: revision={} anchor={:?} actual_camera={moved:?} cached_passes={passes} forced_reference_changed_depth_texels={changed_depth}",
        settled.revision, settled.origin_m
    );
    // Trigger a viewport transition after the patch's fixed counter expires.
    terrain
        .update_view_for_extent(&backend, &view(moved, DQuat::IDENTITY), [4096, 4096])
        .unwrap();
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    terrain.encode_surface(&mut encoder);
    backend.queue.submit([encoder.finish()]);
    let resized = terrain.conforming_surface().unwrap().publication();
    println!(
        "AUDIT viewport: settled_epoch={} resized_epoch={} (257 -> 4096 pixels)",
        settled.revision, resized.revision
    );
    terrain
        .update_view_for_extent(
            &backend,
            &view(moved, DQuat::from_rotation_y(std::f64::consts::PI)),
            [4096, 4096],
        )
        .unwrap();
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    terrain.encode_surface(&mut encoder);
    backend.queue.submit([encoder.finish()]);
    let rotated = terrain.conforming_surface().unwrap().publication();
    println!(
        "AUDIT rotation: resized_epoch={} rotated_epoch={} (180 degrees, same position)",
        resized.revision, rotated.revision
    );
    assert!(
        resized.revision > settled.revision
            && rotated.revision > resized.revision
            && changed_depth == 0,
        "optimizer fails required viewport/orientation invalidation or cached-shadow/reference equivalence; see AUDIT diagnostics"
    );
    assert_eq!(
        terrain.conforming_surface().unwrap().publication().origin_m,
        moved.to_array()
    );
}
