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
#[ignore = "hardware: lit convex planetary terrain must not acquire triangular self-shadow acne"]
fn conforming_planetary_receiver_matches_unoccluded_reference() {
    let mut request = RequestedGraphics::default();
    request.renderer.ray_tracing = RayTracingRequest::Off;
    request.terrain_topology.enabled = true;
    request.terrain_topology.quality = Quality::Low;
    request.shadows.enabled = true;
    request.shadows.cascades = 2;
    request.shadows.map_size = 512;
    request.shadows.max_distance_m = 4000.0;
    let backend = Backend::bring_up_for_graphics(&request).unwrap();
    let settings = backend.resolve_graphics(&request);
    let radius = 3_200_000.0;
    let page = HeightPage::bake(&[0.0; 4], 2, 0.001).unwrap();
    let patches = (0..6)
        .map(|face| TerrainPatch {
            anchor: TileAnchor::new(TileKey::new(face, 0, 0, 0).unwrap(), radius).unwrap(),
            page: &page,
        })
        .collect::<Vec<_>>();
    let camera = DVec3::new(0.0, radius + 100.0, 0.0);
    let view = ViewParams {
        world_from_view: glam::dcamera::rh::view::look_at_mat4(
            camera,
            DVec3::new(0.0, radius, -300.0),
            DVec3::Y,
        )
        .inverse(),
        clip_from_view: glam::dcamera::rh::proj::directx::perspective(0.9, 1.0, 1.0, 1e8),
        world_position_m: camera,
        exposure: 1.0,
    };
    let sun = DVec3::new(0.6, 0.5, 0.4).normalize();
    let lights = SurfaceLighting {
        ambient_lux: [0.02, 0.02, 0.02, 0.0],
        directions: [[sun.x as f32, sun.y as f32, sun.z as f32, 0.0]; 3],
        colors_lux: [[1.0, 1.0, 1.0, 0.0], [0.0; 4], [0.0; 4]],
    };
    let target = OffscreenTarget::new(&backend, 257, 257).unwrap();
    let render = |shadows: bool, occluder: bool| {
        let mut settings = settings.clone();
        settings.shadow_enabled = shadows;
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
        terrain.update_lighting(&backend, &lights).unwrap();
        let asset = thessa_render_core::mesh::MeshAsset {
            primitives: vec![thessa_render_core::mesh::Primitive {
                ambient_visibility: vec![],
                vertices: [
                    [-30.0, 0.0, -30.0],
                    [30.0, 0.0, -30.0],
                    [30.0, 0.0, 30.0],
                    [-30.0, 0.0, 30.0],
                ]
                .map(|position| thessa_render_core::mesh::Vertex {
                    position,
                    normal: [0.0, 1.0, 0.0],
                    uv: [0.0; 2],
                })
                .to_vec(),
                indices: vec![0, 1, 2, 0, 2, 3],
                material: 0,
            }],
            materials: vec![Default::default()],
            textures: vec![],
            images: vec![],
        };
        let mut mesh = thessa_render_wgpu::mesh::MeshRenderer::with_surface(
            &backend,
            &asset,
            thessa_render_wgpu::mesh::SurfaceSettings {
                format: OffscreenTarget::FORMAT,
                capacity: 1,
                quality: Quality::Low,
                reflections: false,
                shadows: terrain.shadows(),
                ray_tracing: None,
            },
        )
        .unwrap();
        mesh.update(
            &backend,
            &view,
            &lights,
            &[thessa_render_core::mesh::MeshInstance {
                position_m: DVec3::new(0.0, radius + 20.0, -180.0),
                rotation: glam::DQuat::IDENTITY,
                scale: 1.0,
            }],
        )
        .unwrap();
        for _ in 0..48 {
            terrain
                .update_view_for_extent(&backend, &view, [257, 257])
                .unwrap();
            terrain.update_shadows(&backend, camera, &lights).unwrap();
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            terrain.encode_surface(&mut encoder);
            terrain.encode_shadows(&mut encoder, occluder.then_some(&mesh));
            terrain.encode(&mut encoder, &target.color, &target.depth);
            backend.queue.submit([encoder.finish()]);
            backend
                .device
                .poll(wgpu::PollType::wait_indefinitely())
                .unwrap();
        }
        target.read_rgba8(&backend).unwrap()
    };
    let reference = render(false, false);
    let actual = render(true, false);
    let mut tested = 0;
    let mut dark = 0;
    let mut max_delta = 0u8;
    for y in 0..257 {
        for x in 0..257 {
            let i = (y * 257 + x) * 4;
            if reference[i + 1] <= 50 {
                continue;
            }
            let delta = reference[i + 1].saturating_sub(actual[i + 1]);
            max_delta = max_delta.max(delta);
            dark += usize::from(delta > 3);
            tested += 1;
        }
    }
    assert!(tested > 23870, "fixture must cover lit terrain");
    println!(
        "conforming_convex_shadow adapter={} tested={tested} dark={dark} max_srgb8_delta={max_delta}; 3-byte envelope, no occluders, no visual preference claim",
        backend.adapter_name
    );
    assert_eq!(
        dark, 0,
        "a lit convex surface has no foreground self-occluder"
    );
    let occluded = render(true, true);
    // Independent local-plane ray projection of the 20m-high opaque square's
    // shadow centre. Planetary sag here is <1cm, well inside this 60m square.
    let point = DVec3::new(0.0, radius, -180.0) - DVec3::new(sun.x, 0.0, sun.z) * (20.0 / sun.y);
    let clip = (view.clip_from_view * view.world_from_view.inverse()).project_point3(point);
    let x = ((clip.x * 0.5 + 0.5) * 257.0) as usize;
    let y = ((0.5 - clip.y * 0.5) * 257.0) as usize;
    assert!(x > 1 && x < 256 && y > 1 && y < 256);
    let i = (y * 257 + x) * 4 + 1;
    println!(
        "conforming_real_shadow pixel=({x},{y}) unoccluded={} occluded={}",
        reference[i], occluded[i]
    );
    assert!(
        occluded[i] + 20 < reference[i],
        "receiver-plane correction must retain a real mesh shadow"
    );
}

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

#[test]
#[ignore = "hardware: production conforming raster must preserve subtexel L22-L24 material motion in all cube charts"]
fn conforming_material_coordinates_keep_centimetre_detail() {
    use wgpu::util::DeviceExt;
    let backend = Backend::bring_up().unwrap();
    let settings = thessa_graphics::ResolvedGraphicsSettings {
        surface_quality: thessa_graphics::Quality::High,
        terrain_topology: thessa_graphics::ResolvedTerrainTopology {
            capacity: 4096,
            waves: 1,
            split_area_px: 256.0,
            merge_area_px: 32.0,
        },
        terrain_ground_detail: true,
        terrain_normal_maps: false,
        specular_reflections: false,
        shadow_enabled: false,
        ..Default::default()
    };
    let height = HeightPage::bake(&[0.0; 4], 2, 0.001).unwrap();
    let rgba = (0..128 * 128)
        .flat_map(|i| [if i % 128 < 64 { 32 } else { 224 }, 0, 0, 255])
        .collect();
    let material = CbtMaterialPage::from_rgba8(rgba).unwrap();
    let target = OffscreenTarget::new(&backend, 65, 65).unwrap();
    // Inject two exact CPU-projected triangles into the production stream. This
    // isolates raster precision from convergence policy, with no alternate raster.
    let module = backend.device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: None,
        source: wgpu::ShaderSource::Wgsl("@group(0) @binding(0) var<storage,read> fixture:array<vec4<u32>>; @group(0) @binding(1) var<storage,read_write> vertices:array<vec4<u32>>; @group(0) @binding(2) var<storage,read> draw_indices:array<u32>; @compute @workgroup_size(64) fn fill(@builtin(global_invocation_id) id:vec3<u32>){ if(id.x<arrayLength(&vertices)){var v=vec4(0u);let first=draw_indices[0]*9u;let second=draw_indices[1]*9u;if(id.x>=first && id.x<first+9u){v=fixture[id.x-first];}if(id.x>=second && id.x<second+9u){v=fixture[id.x-second+9u];}vertices[id.x]=v;}}".into()),
    });
    let pipeline = backend
        .device
        .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: None,
            layout: None,
            module: &module,
            entry_point: Some("fill"),
            compilation_options: Default::default(),
            cache: None,
        });
    for (face, level) in (0..6).flat_map(|face| [22, 23, 24].map(|level| (face, level))) {
        let scale = 1 << (level - 17);
        let key = TileKey::new(face, level, 113_491 * scale, 95_110 * scale).unwrap();
        let anchor = TileAnchor::new(key, 3_200_000.0).unwrap();
        let geometry_anchor = TileAnchor::new(
            TileKey::new(face, 22, key.x >> (level - 22), key.y >> (level - 22)).unwrap(),
            3_200_000.0,
        )
        .unwrap();
        let mut terrain = TerrainRenderer::with_capacity_and_lighting(
            &backend,
            &[TerrainPatch {
                anchor: geometry_anchor,
                page: &height,
            }],
            OffscreenTarget::FORMAT,
            TerrainCapacity {
                patches: 1,
                height_grid_size: 2,
                material_layers: 1,
            },
            Some(&settings),
        )
        .unwrap();
        terrain
            .update_lighting(
                &backend,
                &SurfaceLighting {
                    ambient_lux: [std::f32::consts::PI; 4],
                    directions: [[0.0; 4]; 3],
                    colors_lux: [[0.0; 4]; 3],
                },
            )
            .unwrap();
        let id = thessa_rcbt_core::conforming_surface::page_id(geometry_anchor.tile).unwrap()
            << (2 * (level - 22));
        terrain
            .update_materials(
                &backend,
                &[id],
                &[(id, (1, material.clone()))].into_iter().collect(),
            )
            .unwrap();
        let mut reds = Vec::new();
        for step in -4..=4 {
            let u = 0.5 + f64::from(step) / 4096.0;
            let center = DVec3::from(anchor.project_body_m([u, 0.5], 0.0));
            let eye = center + DVec3::from(anchor.normal) * 2.0;
            let view = ViewParams {
                world_from_view: glam::dcamera::rh::view::look_at_mat4(
                    eye,
                    center,
                    DVec3::from(anchor.raw_axis_v),
                )
                .inverse(),
                clip_from_view: glam::dcamera::rh::proj::directx::perspective(
                    0.01 / f64::from(scale),
                    1.0,
                    0.01,
                    1000.0,
                ),
                world_position_m: eye,
                exposure: 1.0,
            };
            terrain
                .update_view_for_extent(&backend, &view, [65, 65])
                .unwrap();
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            terrain.encode_surface(&mut encoder);
            let mut words = Vec::new();
            for uv in [
                [0.0, 0.0],
                [1.0, 0.0],
                [0.0, 1.0],
                [1.0, 1.0],
                [0.0, 1.0],
                [1.0, 0.0],
            ] {
                let body = DVec3::from(anchor.project_body_m(uv, 0.0));
                let position = (body - eye).as_vec3();
                words.extend([
                    position.x.to_bits(),
                    position.y.to_bits(),
                    position.z.to_bits(),
                    0,
                ]);
                let normal = body.normalize().as_vec3();
                words.extend([
                    normal.x.to_bits(),
                    normal.y.to_bits(),
                    normal.z.to_bits(),
                    (face as f32).to_bits(),
                ]);
                let span = 536870912u32 >> level;
                words.extend([
                    (key.x + uv[0] as u32) * span,
                    (key.y + uv[1] as u32) * span,
                    0,
                    u32::from(level),
                ]);
            }
            let fixture = backend
                .device
                .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: None,
                    contents: &words
                        .into_iter()
                        .flat_map(u32::to_le_bytes)
                        .collect::<Vec<_>>(),
                    usage: wgpu::BufferUsages::STORAGE,
                });
            let vertices = &terrain.conforming_surface().unwrap().vertices;
            let bind = backend
                .device
                .create_bind_group(&wgpu::BindGroupDescriptor {
                    label: None,
                    layout: &pipeline.get_bind_group_layout(0),
                    entries: &[
                        wgpu::BindGroupEntry {
                            binding: 0,
                            resource: fixture.as_entire_binding(),
                        },
                        wgpu::BindGroupEntry {
                            binding: 1,
                            resource: vertices.as_entire_binding(),
                        },
                        wgpu::BindGroupEntry {
                            binding: 2,
                            resource: terrain
                                .conforming_surface()
                                .unwrap()
                                .active_indices
                                .as_entire_binding(),
                        },
                    ],
                });
            {
                let mut pass = encoder.begin_compute_pass(&Default::default());
                pass.set_pipeline(&pipeline);
                pass.set_bind_group(0, &bind, &[]);
                pass.dispatch_workgroups((vertices.size() / 16).div_ceil(64) as u32, 1, 1);
            }
            terrain.encode(&mut encoder, &target.color, &target.depth);
            backend.queue.submit([encoder.finish()]);
            let pixels = target.read_rgba8(&backend).unwrap();
            let actual = pixels[(32 * 65 + 32) * 4];
            let decode = |v: f64| ((v / 255.0 + 0.055) / 1.055).powf(2.4);
            let t = 0.5 + (u - 0.5) * 125.0;
            let linear = decode(32.0) * (1.0 - t) + decode(224.0) * t;
            let expected = ((1.055 * linear.powf(1.0 / 2.4) - 0.055) * 255.0).round() as u8;
            assert!(
                actual.abs_diff(expected) <= 2,
                "face={face} step={step} actual={actual} expected={expected}"
            );
            reds.push(actual);
        }
        assert!(reds.windows(2).all(|p| p[0] < p[1]), "face={face} {reds:?}");
    }
}

#[test]
#[ignore = "hardware: sparse overlapping height pages feed persistent geometry, never a patch partition"]
fn persistent_height_publication_accepts_partial_children_with_root_fallback() {
    use thessa_render_wgpu::terrain::ResidentTerrainPatch;
    let backend = Backend::bring_up().unwrap();
    let settings = thessa_graphics::ResolvedGraphicsSettings {
        terrain_topology: thessa_graphics::ResolvedTerrainTopology {
            capacity: 4096,
            waves: 1,
            split_area_px: 1024.0,
            merge_area_px: 128.0,
        },
        shadow_enabled: false,
        specular_reflections: false,
        ..Default::default()
    };
    let radius = 3_200_000.0;
    let root_page = HeightPage::bake(&[0.0; 4], 2, 0.001).unwrap();
    let child_page = HeightPage::bake(&[20.0; 4], 2, 0.001).unwrap();
    let roots: Vec<_> = (0..6)
        .map(|face| TerrainPatch {
            anchor: TileAnchor::new(TileKey::new(face, 0, 0, 0).unwrap(), radius).unwrap(),
            page: &root_page,
        })
        .collect();
    let capacity = TerrainCapacity {
        patches: 7,
        height_grid_size: 2,
        material_layers: 1,
    };
    let mut terrain = TerrainRenderer::with_capacity_and_lighting(
        &backend,
        &roots,
        OffscreenTarget::FORMAT,
        capacity,
        Some(&settings),
    )
    .unwrap();
    let mut sparse: Vec<_> = roots
        .iter()
        .map(|p| ResidentTerrainPatch {
            anchor: p.anchor,
            page: p.page,
            generation: 1,
        })
        .collect();
    sparse.push(ResidentTerrainPatch {
        anchor: TileAnchor::new(TileKey::new(4, 1, 0, 0).unwrap(), radius).unwrap(),
        page: &child_page,
        generation: 1,
    });
    assert_eq!(
        terrain.publish_partition(&backend, &sparse).unwrap().active,
        7
    );
    let camera = DVec3::new(0.0, 0.0, radius * 2.5);
    let view = ViewParams {
        world_from_view: glam::dcamera::rh::view::look_at_mat4(camera, DVec3::ZERO, DVec3::Y)
            .inverse(),
        clip_from_view: glam::dcamera::rh::proj::directx::perspective(0.9, 1.0, 1.0, 1e8),
        world_position_m: camera,
        exposure: 1.0,
    };
    terrain
        .update_view_for_extent(&backend, &view, [65, 65])
        .unwrap();
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    terrain.encode_surface(&mut encoder);
    backend.queue.submit([encoder.finish()]);
    let vertices = terrain.read_vertices(&backend).unwrap();
    assert!(
        vertices
            .as_chunks::<12>()
            .0
            .iter()
            .any(|v| v[3] == 20.0 && v[7] == 4.0)
    );
    assert!(
        vertices
            .as_chunks::<12>()
            .0
            .iter()
            .any(|v| v[3] == 0.0 && v[4..7].iter().any(|x| *x != 0.0))
    );
    let mut legacy_settings = settings.clone();
    legacy_settings.terrain_topology = Default::default();
    let mut legacy = TerrainRenderer::with_capacity_and_lighting(
        &backend,
        &roots,
        OffscreenTarget::FORMAT,
        capacity,
        Some(&legacy_settings),
    )
    .unwrap();
    assert!(legacy.publish_partition(&backend, &sparse).is_err());
}

#[test]
#[ignore = "hardware: close flat/elevated surfaces must settle without exhausting the persistent pool"]
fn flat_surface_converges_at_zero_and_large_datum_elevations() {
    let backend = Backend::bring_up().unwrap();
    let settings = thessa_graphics::ResolvedGraphicsSettings {
        terrain_topology: thessa_graphics::ResolvedTerrainTopology {
            capacity: 16384,
            waves: 4,
            split_area_px: 64.0,
            merge_area_px: 8.0,
        },
        shadow_enabled: false,
        specular_reflections: false,
        ..Default::default()
    };
    for elevation in [0.0, 500.0] {
        let radius = 3_200_000.0;
        let page = HeightPage::bake(&[elevation; 4], 2, 0.001).unwrap();
        let roots: Vec<_> = (0..6)
            .map(|face| TerrainPatch {
                anchor: TileAnchor::new(TileKey::new(face, 0, 0, 0).unwrap(), radius).unwrap(),
                page: &page,
            })
            .collect();
        let mut terrain = TerrainRenderer::with_capacity_and_lighting(
            &backend,
            &roots,
            OffscreenTarget::FORMAT,
            TerrainCapacity {
                patches: 6,
                height_grid_size: 2,
                material_layers: 1,
            },
            Some(&settings),
        )
        .unwrap();
        let target = DVec3::new(0.0, 0.0, radius + elevation);
        let eye = target + DVec3::Z * 2.0;
        let view = ViewParams {
            world_from_view: glam::dcamera::rh::view::look_at_mat4(eye, target, DVec3::Y).inverse(),
            clip_from_view: glam::dcamera::rh::proj::directx::perspective(0.9, 1.0, 0.1, 1e8),
            world_position_m: eye,
            exposure: 1.0,
        };
        for _ in 0..128 {
            terrain
                .update_view_for_extent(&backend, &view, [129, 129])
                .unwrap();
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            terrain.encode_surface(&mut encoder);
            backend.queue.submit([encoder.finish()]);
            backend
                .device
                .poll(wgpu::PollType::wait_indefinitely())
                .unwrap();
        }
        let state = terrain.read_topology_state(&backend).unwrap().unwrap();
        println!(
            "flat_cbt elevation_m={elevation} live={} split={} merge={} rejected={} last_commit={} last_request={} epochs={}",
            state[8] / 3,
            state[5],
            state[6],
            state[7],
            state[1],
            state[2],
            terrain.conforming_surface().unwrap().publication().revision
        );
        assert_eq!(
            (state[1], state[2], state[7]),
            (0, 0, 0),
            "settlement requires completed zero/zero without exhaustion"
        );
        assert!(state[5] > 0 && state[8] / 3 < 16384);
        assert_eq!(terrain.read_draw(&backend).unwrap(), state[8..12]);

        assert!(terrain.conforming_surface().unwrap().publication().revision < 128);
    }
}

#[path = "support/cbt.rs"]
mod cbt_oracle;

#[test]
#[ignore = "hardware: production ground-orbit-ground topology, canonical vertices and bounded capacity recovery; new THESSA_BENCH_OUTPUT required"]
fn production_ground_orbit_ground_preserves_closed_canonical_surface() {
    use thessa_rcbt_core::{Node, conforming::cube_vertices, conforming_surface::SurfaceAddress};
    use thessa_render_wgpu::timing::GpuIntervals;
    let out = std::path::PathBuf::from(
        std::env::var_os("THESSA_BENCH_OUTPUT").expect("new output directory required"),
    );
    std::fs::create_dir(&out).unwrap();
    let mut request = RequestedGraphics::default();
    request.renderer.ray_tracing = RayTracingRequest::Off;
    let backend = Backend::bring_up_for_graphics_profiled(&request).unwrap();
    let timing = GpuIntervals::new(&backend, 3);
    let radius = 3_200_000.0;
    let anchors: Vec<_> = (0..6)
        .map(|face| TileAnchor::new(TileKey::new(face, 0, 0, 0).unwrap(), radius).unwrap())
        .collect();
    let pages: Vec<_> = anchors
        .iter()
        .map(|anchor| {
            let samples: Vec<_> = (0..17 * 17)
                .map(|i| {
                    let direction =
                        DVec3::from_array(anchor.project_body_m(
                            [f64::from(i % 17) / 16.0, f64::from(i / 17) / 16.0],
                            0.0,
                        ))
                        .normalize();
                    80.0 * direction.x.powi(2)
                        + 50.0 * direction.y.powi(2)
                        + 30.0 * (4.0 * direction.z).sin()
                })
                .collect();
            HeightPage::bake(&samples, 17, 0.001).unwrap()
        })
        .collect();
    for (i, page) in pages.iter().enumerate() {
        std::fs::write(out.join(format!("face-{i}.rhpg")), page.to_bytes()).unwrap();
    }
    let patches: Vec<_> = anchors
        .iter()
        .zip(&pages)
        .map(|(anchor, page)| TerrainPatch {
            anchor: *anchor,
            page,
        })
        .collect();
    let settings = thessa_graphics::ResolvedGraphicsSettings {
        terrain_topology: thessa_graphics::ResolvedTerrainTopology {
            capacity: 131072,
            waves: 4,
            split_area_px: 64.0,
            merge_area_px: 8.0,
        },
        shadow_enabled: false,
        specular_reflections: false,
        ..Default::default()
    };
    let make = |settings: &thessa_graphics::ResolvedGraphicsSettings| {
        TerrainRenderer::with_capacity_and_lighting(
            &backend,
            &patches,
            OffscreenTarget::FORMAT,
            TerrainCapacity {
                patches: 6,
                height_grid_size: 17,
                material_layers: 1,
            },
            Some(settings),
        )
        .unwrap()
    };
    let canonical = |cube: [i64; 3]| {
        let address = SurfaceAddress::from_cube(cube).unwrap();
        let (_, uv) = address.tile(0).unwrap();
        let height = pages[address.face as usize].sample(uv[0] as f32, uv[1] as f32);
        (
            DVec3::from_array(anchors[address.face as usize].project_body_m(uv, f64::from(height))),
            height,
        )
    };
    let view_at = |phase: f64, altitude: f64| {
        let direction = DVec3::new(
            0.18 * (phase * std::f64::consts::TAU).sin(),
            0.08 * (phase * std::f64::consts::TAU).sin(),
            1.0,
        )
        .normalize();
        let surface_height = f64::from(pages[4].sample(
            (direction.x / direction.z * 0.5 + 0.5) as f32,
            (direction.y / direction.z * 0.5 + 0.5) as f32,
        ));
        let eye = direction * (radius + surface_height + altitude);
        ViewParams {
            world_from_view: glam::dcamera::rh::view::look_at_mat4(eye, DVec3::ZERO, DVec3::Y)
                .inverse(),
            clip_from_view: glam::dcamera::rh::proj::directx::perspective(0.9, 1.0, 0.1, 1e8),
            world_position_m: eye,
            exposure: 1.0,
        }
    };
    let target = OffscreenTarget::new(&backend, 129, 129).unwrap();
    let mut terrain = make(&settings);
    let mut csv = String::from(
        "frame,phase,altitude_m,triangles,splits,merges,capacity_rejections,last_committed,last_requested,epoch,topology_emit_gpu_ms,raster_gpu_ms\n",
    );
    let mut ground_epochs = [0; 2];
    let mut ground_counts = [0; 2];
    let mut max_near = 0.0f64;
    let mut max_whole = 0.0f64;
    for frame in 0..224 {
        let (phase, altitude, label) = if frame < 64 {
            (0.0, 2.0, "initial-settle")
        } else if frame < 160 {
            let phase = f64::from(frame - 64) / 95.0;
            let altitude = 2.0
                * ((radius / 1.0_f64).ln() * (std::f64::consts::PI * phase).sin().max(0.0)).exp();
            (phase, altitude, "continuous-flight")
        } else {
            (0.0, 2.0, "return-settle")
        };
        let view = view_at(phase, altitude);
        terrain
            .update_view_for_extent(&backend, &view, [129, 129])
            .unwrap();
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        if let Some(t) = &timing {
            t.mark(&mut encoder, 0);
        }
        terrain.encode_surface(&mut encoder);
        if let Some(t) = &timing {
            t.mark(&mut encoder, 1);
        }
        terrain.encode(&mut encoder, &target.color, &target.depth);
        if let Some(t) = &timing {
            t.mark(&mut encoder, 2);
            t.resolve(&mut encoder);
        }
        backend.queue.submit([encoder.finish()]);
        let elapsed = timing.as_ref().map(|t| t.read(&backend).unwrap());
        let records = terrain.read_topology_records(&backend).unwrap().unwrap();
        cbt_oracle::assert_closed_snapshot(&records);
        let live = records
            .iter()
            .filter(|r| {
                Node::from_heap_id(u64::from(r.address[0]) | (u64::from(r.address[1]) << 32))
                    .ok()
                    .and_then(cube_vertices)
                    .is_some()
            })
            .count();
        let state = terrain.read_topology_state(&backend).unwrap().unwrap();
        // The four reserved virtual roots are submitted as degenerate slots.
        let active = records
            .iter()
            .filter(|r| r.address[0] != 0 || r.address[1] != 0)
            .count();
        assert_eq!(active - live, 4);
        assert_eq!(
            state[8] as usize,
            active * 3,
            "indirect membership at frame {frame}"
        );
        assert_eq!(terrain.read_draw(&backend).unwrap(), state[8..12]);
        let publication = terrain.conforming_surface().unwrap().publication();
        assert_eq!(publication.origin_m, view.world_position_m.to_array());
        assert!(
            state[7] == 0,
            "normal-budget route must not masquerade as converged after exhaustion: frame={frame} state={state:?}"
        );
        if frame % 16 == 0 || frame == 63 || frame == 223 {
            let vertices = terrain.read_vertices(&backend).unwrap();
            let mut shared = std::collections::BTreeMap::new();
            for (slot, r) in records.iter().enumerate() {
                let id = u64::from(r.address[0]) | (u64::from(r.address[1]) << 32);
                let Some(cube) = Node::from_heap_id(id).ok().and_then(cube_vertices) else {
                    assert!(
                        vertices[slot * 36..slot * 36 + 36]
                            .iter()
                            .all(|v| v.to_bits() == 0),
                        "published inactive slot must be zero for shadow/RT consumers: frame={frame} slot={slot}"
                    );
                    continue;
                };
                for (corner, cube) in cube.into_iter().enumerate() {
                    let index = (slot * 3 + corner) * 12;
                    let actual = DVec3::new(
                        f64::from(vertices[index]),
                        f64::from(vertices[index + 1]),
                        f64::from(vertices[index + 2]),
                    );
                    let (expected, height) = canonical(cube);
                    let expected = expected - view.world_position_m;
                    let error = (actual - expected).abs().max_element();
                    max_whole = max_whole.max(error);
                    assert!(
                        error < 8.0,
                        "whole-body coordinate error {error} at frame {frame}"
                    );
                    if expected.length() < 10_000.0 {
                        max_near = max_near.max(error);
                        assert!(
                            error < 0.02,
                            "near-camera coordinate error {error} at frame {frame}"
                        );
                    }
                    assert!(
                        (vertices[index + 3] - height).abs() < 0.005,
                        "canonical height changed with camera"
                    );
                    let bits = [
                        vertices[index].to_bits(),
                        vertices[index + 1].to_bits(),
                        vertices[index + 2].to_bits(),
                    ];
                    if let Some(previous) = shared.insert(cube, bits) {
                        assert_eq!(previous, bits, "shared production vertex crack");
                    }
                }
            }
        }
        if frame == 63 || frame == 223 {
            assert_eq!(
                (state[1], state[2]),
                (0, 0),
                "settled endpoint retains pending work"
            );
            let index = usize::from(frame == 223);
            ground_epochs[index] = publication.revision;
            ground_counts[index] = live;
            std::fs::write(
                out.join(format!("ground-{index}.rgba8")),
                target.read_rgba8(&backend).unwrap(),
            )
            .unwrap();
        }
        let ms = elapsed.map(|v| [v[0] * 1000.0, v[1] * 1000.0]);
        csv.push_str(&format!(
            "{frame},{label},{altitude},{live},{},{},{},{},{},{},{},{}\n",
            state[5],
            state[6],
            state[7],
            state[1],
            state[2],
            publication.revision,
            ms.map_or(String::new(), |v| v[0].to_string()),
            ms.map_or(String::new(), |v| v[1].to_string())
        ));
    }
    assert!(
        ground_epochs[1] > ground_epochs[0],
        "settled topology failed to wake for flight"
    );
    let publication = terrain.conforming_surface().unwrap().publication();
    for _ in 0..4 {
        terrain
            .update_view_for_extent(&backend, &view_at(0.0, 2.0), [129, 129])
            .unwrap();
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        terrain.encode_surface(&mut encoder);
        backend.queue.submit([encoder.finish()]);
        backend
            .device
            .poll(wgpu::PollType::wait_indefinitely())
            .unwrap();
    }
    assert_eq!(
        terrain.conforming_surface().unwrap().publication(),
        publication,
        "settled view must retain its publication"
    );
    assert_eq!(
        std::fs::read(out.join("ground-0.rgba8")).unwrap(),
        std::fs::read(out.join("ground-1.rgba8")).unwrap(),
        "return to the same frozen ground view must reproduce the raster"
    );
    std::fs::write(out.join("trajectory.csv"), csv).unwrap();
    let record = serde_json::json!({"id":"production-cbt-ground-orbit-ground-v1","radius_m":radius,"extent":[129,129],"capacity":settings.terrain_topology.capacity,"waves":4,"split_area_px":64,"merge_area_px":8,"height_grid":17,"height_error_budget_m":0.001,"frames":224,"closed_topology_frames":224,"ground_triangles":ground_counts,"ground_epochs":ground_epochs,"max_near_vertex_error_m":max_near,"max_whole_vertex_error_m":max_whole,"storage_bytes":terrain.topology_storage_bytes(),"gpu_timing_available":timing.is_some(),"adapter":backend.adapter_name,"scope":"production topology/canonical geometry/raster; frozen root residency; not dense tessellation error, streaming, shadow/ocean/RT cost or whole-client performance"});
    std::fs::write(
        out.join("trajectory.json"),
        serde_json::to_vec_pretty(&record).unwrap(),
    )
    .unwrap();
    // Same frozen source, deliberately insufficient topology capacity.
    // Exhaustion must retain pending requests and a closed mesh, then recover.
    let mut bounded_settings = settings.clone();
    bounded_settings.terrain_topology.capacity = 4096;
    let mut bounded = make(&bounded_settings);
    let large = OffscreenTarget::new(&backend, 513, 513).unwrap();
    let mut bounded_rows = Vec::new();
    let mut exhausted = None;
    let mut recovered = None;
    for frame in 0..192 {
        let altitude = if frame < 96 { 2.0 } else { radius * 2.0 };
        let view = view_at(0.0, altitude);
        bounded
            .update_view_for_extent(&backend, &view, [513, 513])
            .unwrap();
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        bounded.encode_surface(&mut encoder);
        bounded.encode(&mut encoder, &large.color, &large.depth);
        backend.queue.submit([encoder.finish()]);
        let state = bounded.read_topology_state(&backend).unwrap().unwrap();
        cbt_oracle::assert_closed_snapshot(
            &bounded.read_topology_records(&backend).unwrap().unwrap(),
        );
        assert!(state[8] / 3 <= 4096);
        bounded_rows.push(serde_json::json!({"frame":frame,"altitude_m":altitude,"state":state,"epoch":bounded.conforming_surface().unwrap().publication().revision}));
        if frame == 95 {
            assert!(
                state[7] > 0 && state[2] > 0,
                "exhausted pool must not report zero pending requests: {state:?}"
            );
            exhausted = Some(state);
        }
        if frame == 191 {
            assert_eq!(
                (state[1], state[2]),
                (0, 0),
                "far view failed to recover: {state:?}"
            );
            recovered = Some(state);
        }
    }
    let before = bounded.conforming_surface().unwrap().publication();
    bounded
        .update_view_for_extent(&backend, &view_at(0.0, 2.0), [513, 513])
        .unwrap();
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    bounded.encode_surface(&mut encoder);
    backend.queue.submit([encoder.finish()]);
    let awake = bounded.read_topology_state(&backend).unwrap().unwrap();
    assert!(
        bounded.conforming_surface().unwrap().publication().revision > before.revision
            && awake[2] > 0,
        "recovered topology must wake on the next close view"
    );
    cbt_oracle::assert_closed_snapshot(&bounded.read_topology_records(&backend).unwrap().unwrap());
    let recovery = serde_json::json!({"capacity":4096,"extent":[513,513],"exhausted_state":exhausted,"recovered_state":recovered,"reawakened_state":awake,"rows":bounded_rows,"scope":"same frozen six root pages; bounded overload control, not equal-error performance comparison"});
    std::fs::write(
        out.join("exhaustion.json"),
        serde_json::to_vec_pretty(&recovery).unwrap(),
    )
    .unwrap();
    eprintln!("production_cbt_trajectory {record}");
}
