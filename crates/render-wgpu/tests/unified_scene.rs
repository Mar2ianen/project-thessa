//! Same-resident-scene split/unified correctness and scoped A/B diagnostics.
//! No async streaming, game-FPS claim or subjective visual acceptance.
use glam::{DMat4, DQuat, DVec3};
use thessa_graphics::{Quality, RayTracingRequest, RequestedGraphics};
use thessa_rcbt_core::{
    HeightPage,
    lighting::SurfaceLighting,
    tile_anchor::{TileAnchor, TileKey},
};
use thessa_render_core::{ViewParams, environment::EnvironmentRadiance, mesh::*};
use thessa_render_wgpu::{
    Backend,
    environment::EnvironmentBinding,
    mesh::{MeshRenderer, SurfaceSettings},
    terrain::{OffscreenTarget, TerrainCapacity, TerrainPatch, TerrainRenderer},
    timing::GpuIntervals,
};

struct Scene {
    terrain: TerrainRenderer,
    mesh: MeshRenderer,
    view: ViewParams,
    lights: SurfaceLighting,
    instances: Vec<MeshInstance>,
}

fn equal_bytes(label: &str, a: &[u8], b: &[u8]) {
    assert!(
        a == b,
        "{label}: lengths {}/{} first mismatch {:?}",
        a.len(),
        b.len(),
        a.iter().zip(b).position(|(a, b)| a != b)
    );
}

fn capture_shadow_choice(
    scene: &mut Scene,
    backend: &Backend,
    target: &OffscreenTarget,
    cull: bool,
) -> (
    Vec<u8>,
    Vec<u8>,
    thessa_render_wgpu::shadows::ShadowDrawStats,
) {
    scene.terrain.invalidate_shadows();
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    if cull {
        scene
            .terrain
            .encode_shadows(&mut encoder, Some(&scene.mesh));
    } else {
        scene
            .terrain
            .encode_shadows_reference(&mut encoder, Some(&scene.mesh));
    }
    let stats = scene.terrain.last_shadow_draws();
    scene.mesh.encode_rt(&mut encoder);
    scene.terrain.encode_scene(
        &mut encoder,
        &target.color,
        &target.depth,
        Some(&scene.mesh),
    );
    backend.queue.submit([encoder.finish()]);
    let depth = scene
        .terrain
        .shadows()
        .unwrap()
        .read_depth(backend)
        .unwrap();
    (depth, target.read_rgba8(backend).unwrap(), stats)
}

#[test]
#[ignore = "hardware: culling versus original full-caster depth atlas and final image, including lifecycle"]
fn shadow_culling_matches_full_casters_not_just_split_unified() {
    let backend = Backend::bring_up().unwrap();
    let target = OffscreenTarget::new(&backend, 257, 193).unwrap();
    let mut scene = Scene::new(
        &backend,
        RayTracingRequest::Off,
        false,
        1024,
        64,
        OffscreenTarget::FORMAT,
    );
    let mut witnessed_rejection = false;
    let mut witnessed_depth = false;
    for (i, camera) in [
        DVec3::new(0.0, 0.0, 1010.0),
        DVec3::new(12.3, -7.2, 1010.0),
        DVec3::new(0.0, 0.0, -1010.0),
        DVec3::new(3e12, -2e12, 1e12),
    ]
    .into_iter()
    .enumerate()
    {
        scene.view.world_position_m = camera;
        scene.view.world_from_view = DMat4::from_translation(camera);
        let dirs = [
            DVec3::Z,
            DVec3::new(0.7, 0.2, 0.4).normalize(),
            DVec3::new(1.0, 0.0, 1e-4).normalize(),
        ];
        scene.lights.directions = dirs.map(|d| [d.x as f32, d.y as f32, d.z as f32, 0.0]);
        scene.lights.colors_lux = [[std::f32::consts::PI; 4]; 3];
        if i == 2 {
            scene.lights.colors_lux[1] = [0.0; 4];
        }
        scene.update(&backend);
        let a = capture_shadow_choice(&mut scene, &backend, &target, false);
        let b = capture_shadow_choice(&mut scene, &backend, &target, true);
        equal_bytes("full/culled depth", &a.0, &b.0);
        equal_bytes("full/culled color", &a.1, &b.1);
        witnessed_rejection |= b.2.submitted_slots < b.2.candidate_slots;
        witnessed_depth |= a.0.as_chunks::<4>().0.iter().any(|p| {
            let z = f32::from_le_bytes(*p);
            z > 0.0 && z < 1.0
        });
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        scene
            .terrain
            .encode_shadows(&mut encoder, Some(&scene.mesh));
        assert_eq!(scene.terrain.last_shadow_passes(), 0);
        assert_eq!(scene.terrain.last_shadow_draws(), Default::default());
    }
    assert!(
        witnessed_rejection && witnessed_depth,
        "oracle must test real caster rejection and nonempty depth"
    );
    // This higher origin leaves the old flat tile's sphere outside the first
    // light's depth volume; the new tall page enters it. A stale bound must fail.
    scene.view.world_position_m = DVec3::new(0.0, 0.0, 1250.0);
    scene.view.world_from_view = DMat4::from_translation(scene.view.world_position_m);
    scene.update(&backend);
    let before = capture_shadow_choice(&mut scene, &backend, &target, false);
    let high = HeightPage::bake(&[80.0, 120.0, -10.0, 40.0], 2, 0.01).unwrap();
    scene
        .terrain
        .update_pages(&backend, &[(528, &high)])
        .unwrap();
    scene.update(&backend);
    let a = capture_shadow_choice(&mut scene, &backend, &target, false);
    assert_ne!(
        before.0, a.0,
        "height replacement must visibly change depth"
    );
    let b = capture_shadow_choice(&mut scene, &backend, &target, true);
    equal_bytes("height replacement depth", &a.0, &b.0);
    assert!(
        scene
            .terrain
            .update_pages(&backend, &[(528, &high), (4096, &high)])
            .is_err()
    );
    let after = capture_shadow_choice(&mut scene, &backend, &target, true);
    equal_bytes("rejected height publication", &b.0, &after.0);
    for count in [4, 0, 2] {
        let pages = (0..count)
            .map(|i| thessa_render_wgpu::terrain::ResidentTerrainPatch {
                anchor: TileAnchor::new(TileKey::new(4, 5, 15 + i, 16).unwrap(), 1000.0).unwrap(),
                page: &high,
                generation: 7,
            })
            .collect::<Vec<_>>();
        scene.terrain.publish_partition(&backend, &pages).unwrap();
        scene.update(&backend);
        let a = capture_shadow_choice(&mut scene, &backend, &target, false);
        let b = capture_shadow_choice(&mut scene, &backend, &target, true);
        equal_bytes("partition shrink/empty/reuse depth", &a.0, &b.0);
        equal_bytes("partition color", &a.1, &b.1);
    }
    let mut ocean = Scene::new(
        &backend,
        RayTracingRequest::Off,
        true,
        1024,
        64,
        OffscreenTarget::FORMAT,
    );
    let a = capture_shadow_choice(&mut ocean, &backend, &target, false);
    let b = capture_shadow_choice(&mut ocean, &backend, &target, true);
    equal_bytes("ocean fail-open atlas", &a.0, &b.0);
    assert_eq!(
        b.2.submitted_slots, b.2.candidate_slots,
        "ocean must retain full caster coverage"
    );
    let mut ocean = Scene::new(
        &backend,
        RayTracingRequest::Off,
        true,
        1,
        1,
        OffscreenTarget::FORMAT,
    );
    assert_eq!(
        ocean.terrain.ocean_stats().deformed_patches,
        1,
        "test actual displaced vertices, not just optical-only fallback"
    );
    for time in [0.0, 2.0, 0.0] {
        ocean.terrain.update_ocean(&backend, time).unwrap();
        let a = capture_shadow_choice(&mut ocean, &backend, &target, false);
        let b = capture_shadow_choice(&mut ocean, &backend, &target, true);
        equal_bytes("displaced ocean fail-open atlas", &a.0, &b.0);
        equal_bytes("displaced ocean color", &a.1, &b.1);
        assert_eq!(b.2.submitted_slots, b.2.candidate_slots);
    }
}

#[test]
#[ignore = "hardware: release same-tree full/culled caster ABAB BABA diagnostic, no streaming or game FPS"]
fn full_versus_culled_shadow_cost() {
    let mut request = RequestedGraphics::default();
    request.renderer.ray_tracing = RayTracingRequest::Off;
    let backend = Backend::bring_up_for_graphics_profiled(&request).unwrap();
    let mut scene = Scene::new(
        &backend,
        RayTracingRequest::Off,
        false,
        1024,
        64,
        OffscreenTarget::FORMAT,
    );
    let target = OffscreenTarget::new(&backend, 1280, 720).unwrap();
    let gpu = GpuIntervals::new(&backend, 3).unwrap();
    for moving in [false, true] {
        let mut reference: Option<(Vec<u8>, Vec<u8>)> = None;
        for (block, cull) in [false, true, false, true, true, false, true, false]
            .into_iter()
            .enumerate()
        {
            let mut shadows = Vec::new();
            let mut total = Vec::new();
            let mut wall = Vec::new();
            let mut slots = 0;
            let mut draws = 0;
            for i in 0..330 {
                let start = std::time::Instant::now();
                scene.view.world_position_m = DVec3::new(
                    if moving {
                        (i as f64 * 0.01).sin() * 0.1
                    } else {
                        0.0
                    },
                    0.0,
                    1010.0,
                );
                scene.view.world_from_view = DMat4::from_translation(scene.view.world_position_m);
                scene.update(&backend);
                // Both static modes use the same exact cache after their first
                // redraw. Moving frames invalidate through ordinary view work.
                let mut encoder = backend.device.create_command_encoder(&Default::default());
                gpu.mark(&mut encoder, 0);
                if cull {
                    scene
                        .terrain
                        .encode_shadows(&mut encoder, Some(&scene.mesh));
                } else {
                    // Cache reference results as production does, while keeping
                    // culling disabled on actual redraws.
                    if moving || i == 0 {
                        scene
                            .terrain
                            .encode_shadows_reference(&mut encoder, Some(&scene.mesh));
                    } else {
                        scene
                            .terrain
                            .encode_shadows(&mut encoder, Some(&scene.mesh));
                    }
                }
                gpu.mark(&mut encoder, 1);
                scene.mesh.encode_rt(&mut encoder);
                scene.terrain.encode_scene(
                    &mut encoder,
                    &target.color,
                    &target.depth,
                    Some(&scene.mesh),
                );
                gpu.mark(&mut encoder, 2);
                gpu.resolve(&mut encoder);
                backend.queue.submit([encoder.finish()]);
                backend
                    .device
                    .poll(wgpu::PollType::wait_indefinitely())
                    .unwrap();
                let elapsed = start.elapsed().as_secs_f64() * 1000.0;
                let times = gpu.read(&backend).unwrap();
                if i >= 30 {
                    shadows.push(times[0] * 1000.0);
                    total.push(times.iter().sum::<f64>() * 1000.0);
                    wall.push(elapsed);
                    slots += scene.terrain.last_shadow_draws().submitted_slots;
                    draws += scene.terrain.last_shadow_draws().draw_calls;
                }
            }
            let depth = scene
                .terrain
                .shadows()
                .unwrap()
                .read_depth(&backend)
                .unwrap();
            let pixels = target.read_rgba8(&backend).unwrap();
            if let Some((a, b)) = &reference {
                equal_bytes("benchmark atlas", a, &depth);
                equal_bytes("benchmark color", b, &pixels);
            } else {
                reference = Some((depth, pixels));
            }
            shadows.sort_by(f64::total_cmp);
            total.sort_by(f64::total_cmp);
            wall.sort_by(f64::total_cmp);
            println!(
                "shadow_culling adapter={:?} block={block} cull={cull} moving={moving} patches=1024 meshes=64 scene=1280x720 atlas_bytes={} samples=300 warmup=30 shadow_p50_ms={} total_gpu_p50_ms={} total_gpu_p95_ms={} total_gpu_p99_ms={} wall_p50_ms={} submitted_slots_total={slots} draw_calls_total={draws}; exact full/culled atlas+color at block endpoint; same resident scene, CPU updates+encode/submit/GPU completion in wall, readbacks/source setup excluded; no game FPS or complete renderer claim",
                backend.adapter_name,
                scene.terrain.shadows().unwrap().storage_bytes(),
                shadows[150],
                total[150],
                total[285],
                total[297],
                wall[150]
            );
            scene.terrain.invalidate_shadows();
        }
    }
}
impl Scene {
    fn new(
        backend: &Backend,
        mode: RayTracingRequest,
        ocean: bool,
        patch_count: u32,
        mesh_count: u32,
        format: wgpu::TextureFormat,
    ) -> Self {
        let mut requested = RequestedGraphics::default();
        requested.renderer.ray_tracing = mode;
        requested.shadows.map_size = 256;
        requested.shadows.max_distance_m = 100.0;
        requested.ocean.enabled = ocean;
        requested.ocean.quality = Quality::Low;
        requested.ocean.significant_height_m = 0.1;
        let settings = backend.resolve_graphics(&requested);
        let page = HeightPage::bake(&[0.0; 4], 2, 0.001).unwrap();
        let patches = (0..patch_count)
            .map(|i| TerrainPatch {
                anchor: TileAnchor::new(
                    if patch_count == 1 {
                        TileKey::new(4, 7, 64, 64).unwrap()
                    } else {
                        TileKey::new(4, 5, i % 32, i / 32).unwrap()
                    },
                    1000.0,
                )
                .unwrap(),
                page: &page,
            })
            .collect::<Vec<_>>();
        let mut terrain = TerrainRenderer::with_capacity_lighting_and_ocean(
            backend,
            &patches,
            format,
            TerrainCapacity {
                patches: patch_count,
                height_grid_size: 2,
                material_layers: 1,
            },
            Some(&settings),
            Some(thessa_render_core::ocean::SeaState {
                gravity_m_s2: 9.81,
                depth_m: 100.0,
                wind_speed_m_s: 10.0,
                wind_direction_rad: 0.0,
                significant_height_m: 0.1,
                seed: 17,
            }),
        )
        .unwrap();
        let mut environment = EnvironmentBinding::new(&backend.device);
        environment.update(
            backend,
            Some(&EnvironmentRadiance::new(2, 2, vec![[0.1, 0.02, 0.3]; 4]).unwrap()),
        );
        terrain.share_environment(backend, Some(&environment));
        let source = MeshAsset {
            primitives: vec![Primitive {
                vertices: [
                    [-0.5, -0.5, -4.0],
                    [0.5, -0.5, -4.0],
                    [0.5, 0.5, -4.0],
                    [-0.5, 0.5, -4.0],
                ]
                .map(|position| Vertex {
                    position,
                    normal: [0.0, 0.0, 1.0],
                    uv: [0.5; 2],
                })
                .to_vec(),
                indices: vec![0, 1, 2, 0, 2, 3],
                material: 0,
            }],
            materials: vec![Material {
                base_color: [0.05, 0.8, 0.1, 1.0],
                metallic: 0.0,
                roughness: 0.7,
                ..Default::default()
            }],
            textures: vec![],
            images: vec![],
        };
        let mut mesh = MeshRenderer::with_surface_and_terrain(
            backend,
            &source,
            SurfaceSettings {
                format,
                capacity: mesh_count as usize,
                quality: Quality::High,
                reflections: true,
                shadows: terrain.shadows(),
                ray_tracing: Some(&settings),
            },
            terrain.ray_source().as_ref(),
        )
        .unwrap();
        mesh.share_environment(backend, &environment);
        let origin = DVec3::new(0.0, 0.0, 1010.0);
        let view = ViewParams {
            world_from_view: DMat4::from_translation(origin),
            world_position_m: origin,
            clip_from_view: glam::dcamera::rh::proj::directx::perspective(1.1, 1.0, 0.1, 100.0),
            exposure: 0.5,
        };
        let lights = SurfaceLighting {
            directions: [[0.0, 0.0, 1.0, 0.0]; 3],
            colors_lux: [[std::f32::consts::PI; 4]; 3],
            ..Default::default()
        };
        let instances = (0..mesh_count)
            .map(|i| MeshInstance {
                position_m: origin
                    + if mesh_count == 1 {
                        DVec3::ZERO
                    } else {
                        DVec3::new(f64::from(i % 8) - 3.5, f64::from(i / 8) - 3.5, 0.0)
                    },
                rotation: DQuat::IDENTITY,
                scale: 1.0,
            })
            .collect();
        let mut scene = Self {
            terrain,
            mesh,
            view,
            lights,
            instances,
        };
        scene.update(backend);
        scene
    }
    fn update(&mut self, backend: &Backend) {
        self.terrain.update_view(backend, &self.view).unwrap();
        self.terrain.update_lighting(backend, &self.lights).unwrap();
        self.terrain.update_ocean(backend, 1.25).unwrap();
        self.terrain
            .update_shadows(backend, self.view.world_position_m, &self.lights)
            .unwrap();
        self.mesh
            .update(backend, &self.view, &self.lights, &self.instances)
            .unwrap();
        self.mesh.sync_rt_terrain(backend, &self.terrain).unwrap();
    }
    fn encode(
        &mut self,
        encoder: &mut wgpu::CommandEncoder,
        target: &OffscreenTarget,
        unified: bool,
    ) {
        self.encode_views(encoder, &target.color, &target.depth, unified);
    }
    fn encode_views(
        &mut self,
        encoder: &mut wgpu::CommandEncoder,
        color: &wgpu::TextureView,
        depth: &wgpu::TextureView,
        unified: bool,
    ) {
        self.terrain.encode_shadows(encoder, Some(&self.mesh));
        if unified {
            self.mesh.encode_rt(encoder);
            self.terrain
                .encode_scene(encoder, color, depth, Some(&self.mesh));
        } else {
            self.terrain.encode(encoder, color, depth);
            self.mesh.encode(encoder, color, depth);
        }
    }
    fn pixels(&mut self, backend: &Backend, target: &OffscreenTarget, unified: bool) -> Vec<u8> {
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        self.encode(&mut encoder, target, unified);
        backend.queue.submit([encoder.finish()]);
        target.read_rgba8(backend).unwrap()
    }
}

#[test]
#[ignore = "hardware: split/unified depth, sky, shadows, ocean and ray-query oracle"]
fn split_and_unified_match_with_live_and_empty_meshes() {
    let mut request = RequestedGraphics::default();
    request.renderer.ray_tracing = RayTracingRequest::Local;
    let backend = Backend::bring_up_for_graphics(&request).unwrap();
    for mode in [RayTracingRequest::Off, RayTracingRequest::Local] {
        for ocean in [false, true] {
            let mut scene = Scene::new(&backend, mode, ocean, 1, 1, OffscreenTarget::FORMAT);
            for size in [129, 256] {
                let target = OffscreenTarget::new(&backend, size, size).unwrap();
                for (translation, rotation) in [
                    (DVec3::ZERO, DQuat::IDENTITY),
                    (DVec3::new(0.2, 0.1, 0.0), DQuat::from_rotation_z(0.3)),
                ] {
                    scene.instances[0].position_m += translation;
                    scene.instances[0].rotation = rotation;
                    scene.update(&backend);
                    // B/A/B/A exercises both first-build directions and cached depth.
                    let b = scene.pixels(&backend, &target, true);
                    if mode == RayTracingRequest::Local {
                        assert!(
                            scene.mesh.rt_stats().active,
                            "oracle must actually build and bind its RT scene"
                        );
                    }
                    assert_eq!(b, scene.pixels(&backend, &target, false));
                    scene.terrain.invalidate_shadows();
                    assert_eq!(b, scene.pixels(&backend, &target, true));
                    assert_eq!(b, scene.pixels(&backend, &target, false));
                    let at = ((size / 2 * size + size / 2) * 4) as usize;
                    assert!(
                        b[at + 1] > b[at] && b[at + 1] > b[at + 2],
                        "foreground green mesh must remain visible: mode={mode:?} ocean={ocean} size={size} center={:?}",
                        &b[at..at + 4]
                    );
                    assert!(
                        b.as_chunks::<4>()
                            .0
                            .iter()
                            .filter(|p| p[2] > p[0] && p[0] > p[1].saturating_add(5))
                            .count()
                            > 100,
                        "fixture must exercise uncovered sky, not only foreground pixels"
                    );
                }
                scene
                    .mesh
                    .update(&backend, &scene.view, &scene.lights, &[])
                    .unwrap();
                scene.terrain.invalidate_shadows();
                let a = scene.pixels(&backend, &target, false);
                assert_eq!(a, scene.pixels(&backend, &target, true));
                assert_eq!(
                    scene.mesh.rt_stats().passes,
                    0,
                    "empty mesh cannot add AS work"
                );
                scene.update(&backend);
            }
        }
    }
}

#[test]
#[ignore = "hardware: release fixed-resident ABAB split/unified command-interval diagnostic"]
fn fixed_resident_split_unified_cost() {
    let mut request = RequestedGraphics::default();
    request.renderer.ray_tracing = RayTracingRequest::Off;
    let backend = Backend::bring_up_for_graphics_profiled(&request).unwrap();
    let mut scene = Scene::new(
        &backend,
        RayTracingRequest::Off,
        false,
        1024,
        64,
        OffscreenTarget::FORMAT,
    );
    let target = OffscreenTarget::new(&backend, 1280, 720).unwrap();
    let gpu = GpuIntervals::new(&backend, 2).expect("hardware diagnostic needs timestamps");
    for moving in [false, true] {
        let mut reference = None;
        // Counterbalance the second ABAB sequence to expose order/clock drift.
        for (block, unified) in [false, true, false, true, true, false, true, false]
            .into_iter()
            .enumerate()
        {
            let mut wall = Vec::new();
            let mut times = Vec::new();
            for i in 0..630 {
                // Identical camera path and resident buffers in all eight blocks;
                // camera changes include anchor/shadow CPU work and redraws.
                scene.view.world_position_m = DVec3::new(
                    if moving {
                        (i as f64 * 0.01).sin() * 0.1
                    } else {
                        0.0
                    },
                    0.0,
                    1010.0,
                );
                scene.view.world_from_view = DMat4::from_translation(scene.view.world_position_m);
                let start = std::time::Instant::now();
                scene.update(&backend);
                let mut encoder = backend.device.create_command_encoder(&Default::default());
                gpu.mark(&mut encoder, 0);
                scene.encode(&mut encoder, &target, unified);
                gpu.mark(&mut encoder, 1);
                gpu.resolve(&mut encoder);
                backend.queue.submit([encoder.finish()]);
                backend
                    .device
                    .poll(wgpu::PollType::wait_indefinitely())
                    .unwrap();
                let elapsed = start.elapsed().as_secs_f64() * 1000.0;
                let value = gpu.read(&backend).unwrap()[0] * 1000.0;
                if i >= 30 {
                    wall.push(elapsed);
                    times.push(value);
                }
            }
            let pixels = target.read_rgba8(&backend).unwrap();
            if let Some(reference) = &reference {
                assert_eq!(&pixels, reference);
            } else {
                reference = Some(pixels);
            }
            wall.sort_by(f64::total_cmp);
            times.sort_by(f64::total_cmp);
            println!(
                "resident_abab adapter={:?} block={block} unified={unified} moving={moving} patches=1024 mesh_instances=64 resolution=1280x720 warmup=30 samples=600 wall_p50_ms={} wall_p95_ms={} wall_p99_ms={} gpu_p50_ms={} gpu_p95_ms={} gpu_p99_ms={}; wall=CPU view/light/ocean(no-op)/shadow/mesh updates+encode/submit/GPU completion; GPU=shadow+classification+raster commands, queue writes excluded; no streaming/readback/compilation in samples, RT off, synthetic quads not game FPS",
                backend.adapter_name,
                wall[300],
                wall[570],
                wall[594],
                times[300],
                times[570],
                times[594]
            );
        }
    }
}

#[test]
#[ignore = "hardware: native HDR split/unified scene with actual sky, mesh, RT and ocean"]
fn hdr_scene_preserves_pass_equivalence_and_exposure() {
    let mut request = RequestedGraphics::default();
    request.renderer.ray_tracing = RayTracingRequest::Local;
    let backend = Backend::bring_up_for_graphics(&request).unwrap();
    let settings = backend.resolve_graphics(&request);
    let display = thessa_render_wgpu::display::DisplayTransform::new(
        &backend,
        &settings,
        OffscreenTarget::FORMAT,
        129,
        129,
    )
    .unwrap();
    assert!(display.enabled());
    for mode in [RayTracingRequest::Off, RayTracingRequest::Local] {
        for ocean in [false, true] {
            let mut scene = Scene::new(&backend, mode, ocean, 1, 1, display.scene_format());
            let target = OffscreenTarget::new(&backend, 129, 129).unwrap();
            let mut reference = None;
            for exposure in [0.5, 0.25, 0.5] {
                scene.view.exposure = exposure;
                scene.update(&backend);
                let mut a = None;
                for unified in [true, false, true, false] {
                    let mut encoder = backend.device.create_command_encoder(&Default::default());
                    scene.encode_views(
                        &mut encoder,
                        display.scene_color(&target.color),
                        &target.depth,
                        unified,
                    );
                    display.encode(&mut encoder, &target.color);
                    backend.queue.submit([encoder.finish()]);
                    let b = target.read_rgba8(&backend).unwrap();
                    if let Some(a) = &a {
                        assert_eq!(a, &b);
                    } else {
                        a = Some(b);
                    }
                }
                let a = a.unwrap();
                if exposure == 0.5 {
                    if let Some(reference) = &reference {
                        assert_eq!(
                            reference, &a,
                            "manual exposure rewind must be deterministic"
                        );
                    } else {
                        reference = Some(a);
                    }
                } else {
                    assert_ne!(
                        reference.as_ref().unwrap(),
                        &a,
                        "manual exposure must actually affect the scene"
                    );
                }
            }
        }
    }
}
