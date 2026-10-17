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
