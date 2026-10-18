//! Production native topology selection, not a diagnostic custom draw pipeline.
use glam::DVec3;
use thessa_graphics::{Quality, RayTracingRequest, RequestedGraphics};
use thessa_rcbt_core::{
    HeightPage,
    lighting::SurfaceLighting,
    material_page::CbtMaterialPage,
    tile_anchor::{TileAnchor, TileKey},
};
use thessa_render_wgpu::{
    Backend,
    terrain::{OffscreenTarget, TerrainCapacity, TerrainPatch, TerrainRenderer},
    view::ViewParams,
};

#[test]
#[ignore = "hardware: production persistent topology raster/shadow/ocean publication and disabled fallback"]
fn native_topology_settings_select_shared_production_consumers() {
    let mut requested = RequestedGraphics::default();
    requested.renderer.ray_tracing = RayTracingRequest::Off;
    requested.terrain_topology.enabled = true;
    requested.terrain_topology.quality = Quality::Low;
    requested.shadows.map_size = 512;
    requested.shadows.cascades = 1;
    requested.shadows.max_distance_m = 12000.0;
    let backend = Backend::bring_up_for_graphics(&requested).unwrap();
    let settings = backend.resolve_graphics(&requested);
    assert_eq!(settings.terrain_topology.capacity, 32768);
    let radius = 6371000.0;
    let page = HeightPage::bake(&[0.0; 4], 2, 0.001).unwrap();
    let patches = (0..6)
        .map(|face| TerrainPatch {
            anchor: TileAnchor::new(TileKey::new(face, 0, 0, 0).unwrap(), radius).unwrap(),
            page: &page,
        })
        .collect::<Vec<_>>();
    let capacity = TerrainCapacity {
        patches: 6,
        height_grid_size: 2,
        material_layers: 6,
    };
    let mut terrain = TerrainRenderer::with_capacity_and_lighting(
        &backend,
        &patches,
        OffscreenTarget::FORMAT,
        capacity,
        Some(&settings),
    )
    .unwrap();
    assert!(terrain.topology_reason().is_none());
    assert!(terrain.topology_storage_bytes() > 32768 * 144);
    let target = OffscreenTarget::new(&backend, 257, 257).unwrap();
    let lights = SurfaceLighting {
        ambient_lux: [1.0, 1.0, 1.0, 0.0],
        directions: [[0.0, 0.0, 1.0, 0.0]; 3],
        colors_lux: [[0.0; 4]; 3],
    };
    terrain.update_lighting(&backend, &lights).unwrap();
    let camera = DVec3::new(0.0, 0.0, 2.5 * radius);
    let view = |camera: DVec3| ViewParams {
        world_from_view: glam::dcamera::rh::view::look_at_mat4(camera, DVec3::ZERO, DVec3::Y)
            .inverse(),
        clip_from_view: glam::dcamera::rh::proj::directx::perspective(0.7, 1.0, 1.0, 1e8),
        world_position_m: camera,
        exposure: 1.0,
    };
    assert!(
        terrain.update_view(&backend, &view(camera)).is_err(),
        "no implicit viewport for LOD"
    );
    let publish = |terrain: &mut TerrainRenderer, color: [u8; 4], generation| {
        let ids = (8..14).collect::<Vec<_>>();
        let pages = ids
            .iter()
            .map(|&id| {
                (
                    id,
                    (
                        generation,
                        CbtMaterialPage::from_rgba8(color.repeat(128 * 128)).unwrap(),
                    ),
                )
            })
            .collect();
        terrain.update_materials(&backend, &ids, &pages).unwrap();
    };
    let draw = |terrain: &mut TerrainRenderer, camera: DVec3, emit: bool| {
        terrain
            .update_view_for_extent(&backend, &view(camera), [257, 257])
            .unwrap();
        terrain.update_shadows(&backend, camera, &lights).unwrap();
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        if emit {
            terrain.encode_surface(&mut encoder);
        }
        terrain.encode_shadows(&mut encoder, None);
        terrain.encode(&mut encoder, &target.color, &target.depth);
        backend.queue.submit([encoder.finish()]);
        target.read_rgba8(&backend).unwrap()
    };
    publish(&mut terrain, [255, 0, 0, 255], 1);
    let initial = draw(&mut terrain, camera, true);
    let center = |bytes: &[u8]| {
        let offset = (128 * 257 + 128) * 4;
        [bytes[offset], bytes[offset + 1], bytes[offset + 2]]
    };
    assert!(
        center(&initial)[0] > 100 && center(&initial)[1] < 5,
        "canonical raster {:?}",
        center(&initial)
    );
    assert_eq!(
        terrain.conforming_surface().unwrap().publication().revision,
        1
    );
    let mut final_red = initial;
    for _ in 0..8 {
        final_red = draw(&mut terrain, camera, true);
    }
    let revision = terrain.conforming_surface().unwrap().publication();
    assert_eq!(
        final_red,
        draw(&mut terrain, camera, false),
        "retained emission must produce exactly the same raster"
    );
    assert_eq!(
        terrain.last_shadow_passes(),
        0,
        "retained epoch permits shadow caching"
    );
    assert_eq!(
        terrain.conforming_surface().unwrap().publication(),
        revision
    );
    publish(&mut terrain, [0, 255, 0, 255], 2);
    let green = draw(&mut terrain, camera, false);
    assert!(center(&green)[1] > 100 && center(&green)[0] < 5);
    let shifted = draw(&mut terrain, camera + DVec3::new(1000.0, 0.0, 0.0), true);
    assert!(center(&shifted)[1] > 100);
    assert_eq!(
        terrain.conforming_surface().unwrap().publication().origin_m,
        (camera + DVec3::new(1000.0, 0.0, 0.0)).to_array()
    );
    // No active sun above: activate one and ensure the production shadow path
    // actually casts full conforming allocation slots, not legacy grid patches.
    let sun = SurfaceLighting {
        colors_lux: [[1.0, 1.0, 1.0, 0.0], [0.0; 4], [0.0; 4]],
        ..lights
    };
    terrain
        .update_shadows(&backend, camera + DVec3::new(1000.0, 0.0, 0.0), &sun)
        .unwrap();
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    terrain.encode_shadows(&mut encoder, None);
    backend.queue.submit([encoder.finish()]);
    assert_eq!(terrain.last_shadow_draws().submitted_slots, 32768);
    assert_eq!(terrain.last_shadow_passes(), 1);
    let depth = terrain.shadows().unwrap().read_depth(&backend).unwrap();
    assert!(
        depth
            .as_chunks::<4>()
            .0
            .iter()
            .any(|p| f32::from_le_bytes(*p) < 1.0)
    );
    // Cache reuse must also work with an actual lit atlas layer, not merely
    // the zero-sun early-out above. A new epoch must force it to cast again.
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    terrain.encode_shadows(&mut encoder, None);
    backend.queue.submit([encoder.finish()]);
    assert_eq!(terrain.last_shadow_passes(), 0);
    assert_eq!(
        terrain.shadows().unwrap().read_depth(&backend).unwrap(),
        depth
    );
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    terrain.encode_surface(&mut encoder);
    terrain.encode_shadows(&mut encoder, None);
    backend.queue.submit([encoder.finish()]);
    assert_eq!(terrain.last_shadow_passes(), 1);
    assert_eq!(terrain.last_shadow_draws().submitted_slots, 32768);
    let mut off = settings.clone();
    off.terrain_topology = Default::default();
    let legacy = TerrainRenderer::with_capacity_and_lighting(
        &backend,
        &patches,
        OffscreenTarget::FORMAT,
        capacity,
        Some(&off),
    )
    .unwrap();
    assert!(legacy.conforming_surface().is_none());
    assert_eq!(legacy.topology_storage_bytes(), 0);
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    legacy.encode_surface(&mut encoder);
    backend.queue.submit([encoder.finish()]);
    // Existing resolved ocean policy, same production raster and emitter.
    let mut water = settings.clone();
    water.ocean.enabled = true;
    water.ocean.geometry_patches = 16;
    water.ocean_size = 32;
    let sea = thessa_render_core::ocean::SeaState {
        gravity_m_s2: 9.81,
        depth_m: 100.0,
        wind_speed_m_s: 10.0,
        wind_direction_rad: 0.0,
        significant_height_m: 2.0,
        seed: 17,
    };
    let mut ocean = TerrainRenderer::with_capacity_lighting_and_ocean(
        &backend,
        &patches,
        OffscreenTarget::FORMAT,
        capacity,
        Some(&water),
        Some(sea),
    )
    .unwrap();
    publish(&mut ocean, [20, 40, 80, 30], 1);
    ocean.update_lighting(&backend, &lights).unwrap();
    ocean
        .update_view_for_extent(&backend, &view(camera), [257, 257])
        .unwrap();
    ocean.update_ocean(&backend, 0.0).unwrap();
    let water0 = draw(&mut ocean, camera, true);
    assert_ne!(center(&water0), [0; 3]);
    ocean.update_ocean(&backend, 1.0).unwrap();
    let water1 = draw(&mut ocean, camera, true);
    assert_ne!(
        water0, water1,
        "spectral optics must be live in the selected raster"
    );
    println!(
        "production_topology adapter={} capacity=32768 epochs={} bytes={} Earth-scale raster/material/rebase/shared-shadow/ocean/disabled oracles; no Bevy visual-parity or whole-client completion claim",
        backend.adapter_name,
        terrain.conforming_surface().unwrap().publication().revision,
        terrain.topology_storage_bytes()
    );
}
