//! Native ray-query reflection oracles.
//!
//! These pin the RT contract, not just "RT runs": scene geometry replaces the
//! environment term at the reflected pixel, a ray that hits nothing reproduces
//! the raster environment term bit-for-bit, and RT off is exactly the raster
//! render with no acceleration-structure work.
use glam::{DMat4, DQuat, DVec3};
use thessa_graphics::{RayTracingRequest, RequestedGraphics, ResolvedGraphicsSettings};
use thessa_rcbt_core::{
    HeightPage,
    lighting::SurfaceLighting,
    tile_anchor::{TileAnchor, TileKey},
};
use thessa_render_core::environment::EnvironmentRadiance;
use thessa_render_core::mesh::*;
use thessa_render_wgpu::{
    Backend, BackendError,
    mesh::{MeshRenderer, SurfaceSettings},
    terrain::{OffscreenTarget, TerrainCapacity, TerrainPatch, TerrainRenderer},
    view::ViewParams,
};

const SIZE: u32 = 129;

/// Camera at `origin` looking down -Z, matching the existing surface oracles.
fn camera(origin: DVec3) -> ViewParams {
    ViewParams {
        world_from_view: DMat4::from_translation(origin),
        world_position_m: origin,
        clip_from_view: glam::dcamera::rh::proj::directx::perspective(
            std::f64::consts::FRAC_PI_3,
            1.0,
            0.1,
            100.0,
        ),
        exposure: 0.5,
    }
}

fn quad(radius: f32, material: usize) -> Primitive {
    Primitive {
        vertices: [
            [-radius, -radius, 0.0],
            [radius, -radius, 0.0],
            [radius, radius, 0.0],
            [-radius, radius, 0.0],
        ]
        .map(|position| Vertex {
            position,
            normal: [0.0, 0.0, 1.0],
            uv: [0.5; 2],
        })
        .to_vec(),
        indices: vec![0, 1, 2, 0, 2, 3],
        material,
    }
}

/// A white perfect mirror at 45° and a red diffuse quad in its reflected
/// direction (-X). Both are camera-relative geometry.
fn source() -> MeshAsset {
    MeshAsset {
        primitives: vec![quad(1.2, 0), quad(1.5, 1)],
        materials: vec![
            Material {
                base_color: [1.0; 4],
                metallic: 1.0,
                roughness: 0.0,
                ..Default::default()
            },
            Material {
                base_color: [1.0, 0.0, 0.0, 1.0],
                metallic: 0.0,
                roughness: 1.0,
                ..Default::default()
            },
        ],
        textures: vec![],
        images: vec![],
    }
}

fn instances() -> Vec<MeshInstance> {
    vec![
        MeshInstance {
            position_m: DVec3::new(0.0, 0.0, -2.0),
            rotation: DQuat::from_rotation_y(-std::f64::consts::FRAC_PI_4),
            scale: 1.0,
        },
        MeshInstance {
            position_m: DVec3::new(-3.0, 0.0, -2.0),
            rotation: DQuat::from_rotation_y(std::f64::consts::FRAC_PI_2),
            scale: 1.0,
        },
    ]
}

/// One sun along +X, so the target quad (normal +X) is lit and the mirror is
/// not: any red at the mirror pixel can only arrive through the reflection.
fn lighting(ambient_lux: [f32; 3]) -> SurfaceLighting {
    SurfaceLighting {
        ambient_lux: [ambient_lux[0], ambient_lux[1], ambient_lux[2], 0.0],
        directions: [[1.0, 0.0, 0.0, 0.0], [0.0; 4], [0.0; 4]],
        colors_lux: [[std::f32::consts::PI; 4], [0.0; 4], [0.0; 4]],
    }
}

fn requested(mode: RayTracingRequest) -> RequestedGraphics {
    let mut requested = RequestedGraphics::default();
    requested.renderer.ray_tracing = mode;
    // Reflection/material oracles isolate radiance from scene visibility.
    requested.shadows.enabled = false;
    requested
}

fn backend(mode: RayTracingRequest) -> Option<Backend> {
    match Backend::bring_up_for_graphics(&requested(mode)) {
        Ok(backend) => Some(backend),
        Err(BackendError::NoAdapter) => {
            eprintln!("SKIP: no GPU adapter");
            None
        }
        // Explicit RT on an incapable adapter must fail loudly; the oracle
        // then reports a skip rather than pretending to have tested RT.
        Err(BackendError::Device(reason)) if reason.contains("lacks ray query") => {
            eprintln!("SKIP: adapter lacks ray query: {reason}");
            None
        }
        Err(error) => panic!("{error}"),
    }
}

fn renderer(
    backend: &Backend,
    settings: &ResolvedGraphicsSettings,
) -> Result<MeshRenderer, BackendError> {
    MeshRenderer::with_surface(
        backend,
        &source(),
        SurfaceSettings {
            format: OffscreenTarget::FORMAT,
            capacity: 2,
            quality: settings.surface_quality,
            reflections: settings.specular_reflections,
            shadows: None,
            ray_tracing: Some(settings),
        },
    )
}

fn draw(
    backend: &Backend,
    mesh: &mut MeshRenderer,
    mesh_instances: &[MeshInstance],
    ranges: &[std::ops::Range<u32>],
    lights: &SurfaceLighting,
) -> Vec<u8> {
    draw_at(
        backend,
        mesh,
        mesh_instances,
        ranges,
        lights,
        &camera(DVec3::ZERO),
    )
}

fn draw_at(
    backend: &Backend,
    mesh: &mut MeshRenderer,
    mesh_instances: &[MeshInstance],
    ranges: &[std::ops::Range<u32>],
    lights: &SurfaceLighting,
    view: &ViewParams,
) -> Vec<u8> {
    let target = OffscreenTarget::new(backend, SIZE, SIZE).unwrap();
    mesh.update_selected(backend, view, lights, mesh_instances, Some(ranges))
        .unwrap();
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    clear_target(&mut encoder, &target);
    mesh.encode(&mut encoder, &target.color, &target.depth);
    backend.queue.submit([encoder.finish()]);
    target.read_rgba8(backend).unwrap()
}

fn clear_target(encoder: &mut wgpu::CommandEncoder, target: &OffscreenTarget) {
    {
        let _pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
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
            ..Default::default()
        });
    }
}

/// Screen-center pixel, where the mirror quad sits.
fn center(bytes: &[u8]) -> [u8; 3] {
    let at = ((SIZE / 2) * SIZE + SIZE / 2) as usize * 4;
    [bytes[at], bytes[at + 1], bytes[at + 2]]
}

struct TerrainMirror {
    terrain: TerrainRenderer,
    mesh: MeshRenderer,
    page: HeightPage,
    anchor: TileAnchor,
    origin: DVec3,
    instances: Vec<MeshInstance>,
}

impl TerrainMirror {
    fn new(backend: &Backend, settings: &ResolvedGraphicsSettings, diffuse: bool) -> Self {
        Self::with_capacity(backend, settings, diffuse, 2)
    }
    fn with_capacity(
        backend: &Backend,
        settings: &ResolvedGraphicsSettings,
        diffuse: bool,
        capacity: u32,
    ) -> Self {
        let anchor = TileAnchor::new(TileKey::new(0, 7, 64, 64).unwrap(), 1000.0).unwrap();
        let page = HeightPage::bake(&[0.0; 4], 2, 0.001).unwrap();
        let sea = thessa_render_core::ocean::SeaState {
            gravity_m_s2: 6.4,
            depth_m: settings.ocean.depth_m,
            wind_speed_m_s: settings.ocean.wind_speed_m_s,
            wind_direction_rad: settings.ocean.wind_direction_rad,
            significant_height_m: settings.ocean.significant_height_m,
            seed: settings.ocean.seed,
        };
        let terrain = TerrainRenderer::with_capacity_lighting_and_ocean(
            backend,
            &[TerrainPatch {
                anchor,
                page: &page,
            }],
            OffscreenTarget::FORMAT,
            TerrainCapacity {
                patches: capacity,
                height_grid_size: 2,
                material_layers: 1,
            },
            Some(settings),
            Some(sea),
        )
        .unwrap();
        let mut asset = source();
        if diffuse {
            asset.materials[0].metallic = 0.0;
            asset.materials[0].roughness = 1.0;
        }
        let mesh = MeshRenderer::with_surface_and_terrain(
            backend,
            &asset,
            SurfaceSettings {
                format: OffscreenTarget::FORMAT,
                capacity: 1,
                quality: settings.surface_quality,
                reflections: settings.specular_reflections,
                shadows: None,
                ray_tracing: Some(settings),
            },
            terrain.ray_source().as_ref(),
        )
        .unwrap();
        let origin = DVec3::from_array(anchor.anchor_body_m) + DVec3::new(3.0, 0.0, 2.0);
        let mut instances = instances();
        instances.truncate(1);
        instances[0].position_m += origin;
        Self {
            terrain,
            mesh,
            page,
            anchor,
            origin,
            instances,
        }
    }

    fn material(&mut self, backend: &Backend, color: [u8; 4], generation: u64) {
        let slot = self.anchor.tile;
        let mut node = 8 + u64::from(slot.face);
        for bit in (0..slot.level).rev() {
            node = (node << 2) | u64::from(((slot.x >> bit) & 1) * 2 + ((slot.y >> bit) & 1));
        }
        let page =
            thessa_rcbt_core::material_page::CbtMaterialPage::from_rgba8(color.repeat(128 * 128))
                .unwrap();
        self.terrain
            .update_materials(backend, &[node], &[(node, (generation, page))].into())
            .unwrap();
    }

    fn draw(&mut self, backend: &Backend, lights: &SurfaceLighting) -> Vec<u8> {
        self.mesh.sync_rt_terrain(backend, &self.terrain).unwrap();
        draw_at(
            backend,
            &mut self.mesh,
            &self.instances,
            &[0..1, 0..0],
            lights,
            &camera(self.origin),
        )
    }
}

#[test]
fn resident_terrain_hits_share_materials_and_dirty_only_blas() {
    let Some(backend) = backend(RayTracingRequest::Local) else {
        return;
    };
    let settings = backend.resolve_graphics(&requested(RayTracingRequest::Local));
    let mut scene = TerrainMirror::new(&backend, &settings, false);
    scene.material(&backend, [255, 0, 0, 255], 1);
    let lights = lighting([0.0; 3]);
    let red = scene.draw(&backend, &lights);
    let pixel = center(&red);
    // White dielectric GGX contributes ~15 encoded levels on the dark channels;
    // diffuse red is ~185 at this sun/exposure, within two 8-bit levels.
    assert!(
        (183..=187).contains(&pixel[0])
            && (13..=17).contains(&pixel[1])
            && (13..=17).contains(&pixel[2]),
        "terrain radiance: {pixel:?}"
    );
    assert_eq!(scene.mesh.rt_stats().terrain_instances, 1);
    assert_eq!(scene.mesh.rt_stats().terrain_blas_builds, 1);
    assert!(scene.mesh.rt_terrain_reason().is_none());
    assert_eq!(scene.draw(&backend, &lights), red, "static reuse is exact");
    assert_eq!(scene.mesh.rt_stats().passes, 0);
    // Replacing an immutable fleet/mesh source must not reconstruct unchanged
    // planetary BLASes. The new TLAS still builds once with retained handles.
    let mut replacement = MeshRenderer::with_surface_and_terrain(
        &backend,
        &source(),
        SurfaceSettings {
            format: OffscreenTarget::FORMAT,
            capacity: 1,
            quality: settings.surface_quality,
            reflections: settings.specular_reflections,
            shadows: None,
            ray_tracing: Some(&settings),
        },
        scene.terrain.ray_source().as_ref(),
    )
    .unwrap();
    assert!(replacement.adopt_rt_terrain(&mut scene.mesh));
    scene.mesh = replacement;
    assert_eq!(scene.draw(&backend, &lights), red);
    assert_eq!(scene.mesh.rt_stats().terrain_blas_builds, 0);
    assert_eq!(scene.mesh.rt_stats().passes, 1);
    // The same bytes, deliberately regenerated, must not change intersections.
    scene
        .terrain
        .update_pages(&backend, &[(0, &scene.page)])
        .unwrap();
    assert_eq!(scene.draw(&backend, &lights), red);
    assert_eq!(scene.mesh.rt_stats().terrain_blas_builds, 1);
    // Material-only publication is visible through shared raster resources,
    // without rebuilding geometry, hit buffers, or the TLAS.
    scene.material(&backend, [0, 0, 255, 255], 2);
    let blue = center(&scene.draw(&backend, &lights));
    assert!(
        (183..=187).contains(&blue[2])
            && (13..=17).contains(&blue[0])
            && (13..=17).contains(&blue[1]),
        "material update: {blue:?}"
    );
    assert_eq!(scene.mesh.rt_stats().passes, 0);
    scene.origin.x += 0.125;
    scene.draw(&backend, &lights);
    assert_eq!(
        scene.mesh.rt_stats().terrain_blas_builds,
        0,
        "camera only changes TLAS"
    );
    assert_eq!(scene.mesh.rt_stats().passes, 1);
    // Move the patch outside the outgoing ray; a stale BLAS would stay blue.
    let raised = HeightPage::bake(&[10.0; 4], 2, 0.001).unwrap();
    scene
        .terrain
        .update_pages(&backend, &[(0, &raised)])
        .unwrap();
    let miss = center(&scene.draw(&backend, &lights));
    assert_eq!(
        miss, [0; 3],
        "dirty geometry must replace old intersections"
    );
    assert_eq!(scene.mesh.rt_stats().terrain_blas_builds, 1);
    scene.terrain.publish_partition(&backend, &[]).unwrap();
    scene.draw(&backend, &lights);
    assert_eq!(scene.mesh.rt_stats().terrain_instances, 0);
    assert_eq!(scene.mesh.rt_stats().terrain_blas_builds, 0);
    assert_eq!(scene.mesh.rt_stats().instances, 1);
}

#[test]
fn terrain_budget_rejects_whole_partition_and_recovers_without_stale_blockers() {
    let Some(backend) = backend(RayTracingRequest::Local) else {
        return;
    };
    let mut settings = backend.resolve_graphics(&requested(RayTracingRequest::Local));
    settings.rt_max_instances = 3; // two reserved mesh pairings + one terrain.
    let mut scene = TerrainMirror::new(&backend, &settings, false);
    scene.material(&backend, [255, 0, 0, 255], 1);
    let lights = lighting([0.0; 3]);
    let enabled = scene.draw(&backend, &lights);
    assert_eq!(scene.mesh.rt_stats().terrain_instances, 1);
    let other = TileAnchor::new(TileKey::new(0, 7, 64, 65).unwrap(), 1000.0).unwrap();
    let resident = |anchor| thessa_render_wgpu::terrain::ResidentTerrainPatch {
        anchor,
        page: &scene.page,
        generation: 0,
    };
    scene
        .terrain
        .publish_partition(&backend, &[resident(scene.anchor), resident(other)])
        .unwrap();
    assert_eq!(center(&scene.draw(&backend, &lights)), [0; 3]);
    assert_eq!(scene.mesh.rt_stats().terrain_instances, 0);
    assert_eq!(scene.mesh.rt_stats().terrain_blas_builds, 0);
    assert!(
        scene
            .mesh
            .rt_terrain_reason()
            .unwrap()
            .contains("partition has 2")
    );
    scene
        .terrain
        .publish_partition(
            &backend,
            &[thessa_render_wgpu::terrain::ResidentTerrainPatch {
                anchor: scene.anchor,
                page: &scene.page,
                generation: 0,
            }],
        )
        .unwrap();
    assert_eq!(scene.draw(&backend, &lights), enabled);
    assert_eq!(scene.mesh.rt_stats().terrain_blas_builds, 1);
    assert!(scene.mesh.rt_terrain_reason().is_none());
    // Disabled terrain participation adds no ray-source capability/resources.
    settings.rt_terrain = false;
    let off = TerrainMirror::new(&backend, &settings, false);
    assert!(off.terrain.ray_source().is_none());
}

#[test]
fn shadow_only_terrain_blocks_primary_mesh_without_hit_tables() {
    let Some(backend) = backend(RayTracingRequest::Local) else {
        return;
    };
    let mut request = requested(RayTracingRequest::Local);
    request.shadows.enabled = true;
    request.renderer.specular_reflections = false;
    let settings = backend.resolve_graphics(&request);
    let mut scene = TerrainMirror::new(&backend, &settings, true);
    let mut lights = lighting([0.0; 3]);
    lights.directions[0][0] = -1.0;
    assert_eq!(center(&scene.draw(&backend, &lights)), [0; 3]);
    assert_eq!(scene.mesh.rt_stats().terrain_blas_builds, 1);
    scene.terrain.publish_partition(&backend, &[]).unwrap();
    let unblocked = center(&scene.draw(&backend, &lights));
    assert!(
        unblocked[0] > 130,
        "retired terrain cannot keep shadowing: {unblocked:?}"
    );
    assert_eq!(scene.mesh.rt_stats().terrain_instances, 0);
}

#[test]
fn terrain_membership_is_independent_of_vehicle_participation_and_empty_tlas_is_safe() {
    let Some(backend) = backend(RayTracingRequest::Local) else {
        return;
    };
    let mut settings = backend.resolve_graphics(&requested(RayTracingRequest::Local));
    settings.rt_vehicles = false;
    let mut scene = TerrainMirror::new(&backend, &settings, false);
    scene.material(&backend, [255, 0, 0, 255], 1);
    let lights = lighting([0.0; 3]);
    assert!(center(&scene.draw(&backend, &lights))[0] > 170);
    assert_eq!(scene.mesh.rt_stats().instances, 1);
    assert_eq!(
        scene.mesh.rt_stats().triangles,
        2048,
        "no vehicle geometry charged/traced"
    );
    scene.terrain.publish_partition(&backend, &[]).unwrap();
    assert_eq!(center(&scene.draw(&backend, &lights)), [0; 3]);
    assert_eq!(scene.mesh.rt_stats().instances, 0);
    assert_eq!(
        scene.mesh.rt_stats().passes,
        1,
        "retire the previous blocker TLAS"
    );
    scene.draw(&backend, &lights);
    assert_eq!(scene.mesh.rt_stats().passes, 0);
    // The first ever query may also see an empty admitted partition.
    let mut empty = TerrainMirror::new(&backend, &settings, false);
    empty.terrain.publish_partition(&backend, &[]).unwrap();
    assert_eq!(center(&empty.draw(&backend, &lights)), [0; 3]);
    assert_eq!(empty.mesh.rt_stats().passes, 1);
}

#[test]
fn ocean_hits_use_shared_deformed_grid_and_spectral_normals_without_global_rebuilds() {
    let Some(backend) = backend(RayTracingRequest::Local) else {
        return;
    };
    let mut request = requested(RayTracingRequest::Local);
    request.ocean.enabled = true;
    request.ocean.quality = thessa_graphics::Quality::Low;
    request.ocean.significant_height_m = 0.4;
    let settings = backend.resolve_graphics(&request);
    let mut scene = TerrainMirror::new(&backend, &settings, false);
    scene.material(&backend, [20, 40, 180, 30], 1);
    scene
        .terrain
        .update_view(&backend, &camera(scene.origin))
        .unwrap();
    let datum = scene.terrain.read_vertices(&backend).unwrap();
    scene.terrain.update_ocean(&backend, 0.0).unwrap();
    let waves = scene.terrain.read_vertices(&backend).unwrap();
    assert_ne!(waves, datum);
    let lights = lighting([0.0; 3]);
    let first = scene.draw(&backend, &lights);
    assert_eq!(scene.mesh.rt_stats().terrain_blas_builds, 1);
    scene.terrain.update_ocean(&backend, 0.0).unwrap();
    assert_eq!(scene.draw(&backend, &lights), first);
    assert_eq!(scene.mesh.rt_stats().passes, 0);
    scene.terrain.update_ocean(&backend, 1.0).unwrap();
    let second = scene.draw(&backend, &lights);
    assert_ne!(
        first, second,
        "shared ray hit normals/displacement must evolve"
    );
    assert_eq!(scene.mesh.rt_stats().terrain_blas_builds, 1);
    scene.terrain.update_ocean(&backend, 0.0).unwrap();
    assert_eq!(
        scene.draw(&backend, &lights),
        first,
        "ray shading rewind is deterministic"
    );
}

#[test]
#[ignore = "requires a ray-query adapter; release bounded ocean deformation/AS/trace cost diagnostic"]
fn ocean_near_geometry_and_ray_cost_diagnostic() {
    use thessa_render_wgpu::timing::GpuIntervals;
    let mut request = requested(RayTracingRequest::Local);
    request.ocean.enabled = true;
    request.ocean.quality = thessa_graphics::Quality::High;
    request.ocean.geometry_patches = 16;
    let backend = Backend::bring_up_for_graphics_profiled(&request).unwrap();
    let settings = backend.resolve_graphics(&request);
    for count in [1, 16] {
        let mut scene = TerrainMirror::with_capacity(&backend, &settings, false, 16);
        let anchors: Vec<_> = (0..count)
            .map(|i| {
                TileAnchor::new(TileKey::new(0, 7, 64 + i % 4, 64 + i / 4).unwrap(), 1000.0)
                    .unwrap()
            })
            .collect();
        scene
            .terrain
            .publish_partition(
                &backend,
                &anchors
                    .iter()
                    .map(
                        |&anchor| thessa_render_wgpu::terrain::ResidentTerrainPatch {
                            anchor,
                            page: &scene.page,
                            generation: 0,
                        },
                    )
                    .collect::<Vec<_>>(),
            )
            .unwrap();
        scene.material(&backend, [20, 40, 180, 30], 1);
        scene
            .terrain
            .update_view(&backend, &camera(scene.origin))
            .unwrap();
        let geometry_timing = GpuIntervals::new(&backend, 2);
        let ray_timing = GpuIntervals::new(&backend, 3);
        let target = OffscreenTarget::new(&backend, 1280, 720).unwrap();
        let lights = lighting([0.0; 3]);
        let mut geometry = Vec::new();
        let mut builds = Vec::new();
        let mut trace = Vec::new();
        for frame in 0..80 {
            scene
                .terrain
                .update_ocean_profiled(&backend, frame as f64 / 60.0, geometry_timing.as_ref())
                .unwrap();
            scene
                .mesh
                .sync_rt_terrain(&backend, &scene.terrain)
                .unwrap();
            scene
                .mesh
                .update_selected(
                    &backend,
                    &camera(scene.origin),
                    &lights,
                    &scene.instances,
                    Some(&[0..1, 0..0]),
                )
                .unwrap();
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            clear_target(&mut encoder, &target);
            scene.mesh.encode_profiled(
                &mut encoder,
                &target.color,
                &target.depth,
                ray_timing.as_ref(),
            );
            backend.queue.submit([encoder.finish()]);
            backend
                .device
                .poll(wgpu::PollType::wait_indefinitely())
                .unwrap();
            let g = geometry_timing
                .as_ref()
                .map(|t| t.read(&backend).unwrap()[0] * 1000.0);
            let r = ray_timing.as_ref().map(|t| t.read(&backend).unwrap());
            if frame >= 20 {
                assert_eq!(scene.mesh.rt_stats().terrain_blas_builds, count);
                if let Some(v) = g {
                    geometry.push(v);
                }
                if let Some(v) = r {
                    builds.push(v[0] * 1000.0);
                    trace.push(v[1] * 1000.0);
                }
            }
        }
        geometry.sort_by(f64::total_cmp);
        builds.sort_by(f64::total_cmp);
        trace.sort_by(f64::total_cmp);
        eprintln!(
            "ocean_near_cost adapter={:?} slots={count} size=128 layers=9 extent=1280x720 samples=60 geometry_gpu_p50_ms={:?} as_gpu_p50_ms={:?} mesh_trace_gpu_p50_ms={:?} near_bytes={}",
            backend.adapter_name,
            geometry.get(30),
            builds.get(30),
            trace.get(30),
            scene.terrain.ocean_stats().geometry_bytes
        );
    }
}

#[test]
#[ignore = "requires a ray-query adapter; release fixed-workload cost diagnostic"]
fn terrain_ray_dirty_build_cost_diagnostic() {
    use thessa_render_wgpu::timing::GpuIntervals;
    let backend =
        Backend::bring_up_for_graphics_profiled(&requested(RayTracingRequest::Local)).unwrap();
    let mut settings = backend.resolve_graphics(&requested(RayTracingRequest::Local));
    settings.rt_scene_memory_bytes = 128 * 1024 * 1024;
    settings.rt_max_instances = 2048;
    let page = HeightPage::bake(&[0.0; 4], 2, 0.001).unwrap();
    let patches: Vec<_> = (0..1024)
        .map(|i| TerrainPatch {
            anchor: TileAnchor::new(
                TileKey::new(0, 17, 65536 + i % 32, 65536 + i / 32).unwrap(),
                6_371_000.0,
            )
            .unwrap(),
            page: &page,
        })
        .collect();
    let mut terrain = TerrainRenderer::with_capacity_and_lighting(
        &backend,
        &patches,
        OffscreenTarget::FORMAT,
        TerrainCapacity {
            patches: 1024,
            height_grid_size: 2,
            material_layers: 1,
        },
        Some(&settings),
    )
    .unwrap();
    let mut mesh = MeshRenderer::with_surface_and_terrain(
        &backend,
        &source(),
        SurfaceSettings {
            format: OffscreenTarget::FORMAT,
            capacity: 1,
            quality: settings.surface_quality,
            reflections: true,
            shadows: None,
            ray_tracing: Some(&settings),
        },
        terrain.ray_source().as_ref(),
    )
    .unwrap();
    let origin = DVec3::from_array(patches[0].anchor.anchor_body_m) + DVec3::new(3.0, 0.0, 2.0);
    let view = camera(origin);
    let mut instances = instances();
    instances.truncate(1);
    instances[0].position_m += origin;
    let lights = lighting([0.0; 3]);
    let target = OffscreenTarget::new(&backend, 1280, 720).unwrap();
    let timing = GpuIntervals::new(&backend, 2);
    let mut reference = None;
    for (name, dirty_count) in [("cached", 0), ("one_dirty", 1), ("all_dirty", 1024)] {
        let updates: Vec<_> = (0..dirty_count).map(|i| (i, &page)).collect();
        let mut host = Vec::new();
        let mut gpu = Vec::new();
        let mut prepare = Vec::new();
        for frame in 0..35 {
            terrain.update_pages(&backend, &updates).unwrap();
            // Exclude height regeneration from this AS+mesh diagnostic. GPU
            // timestamps below include build/trace barriers, not queue writes.
            backend
                .device
                .poll(wgpu::PollType::wait_indefinitely())
                .unwrap();
            let start = std::time::Instant::now();
            mesh.sync_rt_terrain(&backend, &terrain).unwrap();
            mesh.update_selected(&backend, &view, &lights, &instances, Some(&[0..1, 0..0]))
                .unwrap();
            let prep_ms = start.elapsed().as_secs_f64() * 1000.0;
            let start = std::time::Instant::now();
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            clear_target(&mut encoder, &target);
            if let Some(t) = &timing {
                t.mark(&mut encoder, 0);
            }
            mesh.encode(&mut encoder, &target.color, &target.depth);
            if let Some(t) = &timing {
                t.mark(&mut encoder, 1);
                t.resolve(&mut encoder);
            }
            backend.queue.submit([encoder.finish()]);
            backend
                .device
                .poll(wgpu::PollType::wait_indefinitely())
                .unwrap();
            let host_ms = start.elapsed().as_secs_f64() * 1000.0;
            let gpu_ms = timing
                .as_ref()
                .map(|t| t.read(&backend).unwrap()[0] * 1000.0);
            if frame >= 5 {
                assert_eq!(mesh.rt_stats().terrain_instances, 1024);
                assert_eq!(mesh.rt_stats().terrain_blas_builds, dirty_count);
                assert_eq!(mesh.rt_stats().passes, u32::from(dirty_count > 0));
                host.push(host_ms);
                prepare.push(prep_ms);
                if let Some(ms) = gpu_ms {
                    gpu.push(ms);
                }
            }
        }
        let pixels = target.read_rgba8(&backend).unwrap();
        if let Some(previous) = &reference {
            assert_eq!(previous, &pixels, "cached/dirty intersections differ");
        } else {
            reference = Some(pixels);
        }
        host.sort_by(f64::total_cmp);
        prepare.sort_by(f64::total_cmp);
        gpu.sort_by(f64::total_cmp);
        eprintln!(
            "terrain_rt_cost adapter={:?} mode={name} patches=1024 triangles=2097152 extent=1280x720 samples=30 prepare_p50_ms={} host_p50_ms={} host_p95_ms={} gpu_p50_ms={:?} planning_bytes={} (height generation and readback excluded)",
            backend.adapter_name,
            prepare[15],
            host[15],
            host[28],
            gpu.get(15),
            mesh.rt_stats().bytes
        );
    }
}

#[test]
fn reflection_replaces_environment_with_scene_geometry() {
    let Some(backend) = backend(RayTracingRequest::Auto) else {
        return;
    };
    let settings = backend.resolve_graphics(&requested(RayTracingRequest::Auto));
    assert!(
        settings.ray_tracing.is_active(),
        "this rig resolves RT off: {}",
        settings.notes.join("; ")
    );
    let mut mesh = renderer(&backend, &settings).unwrap();
    let stats = mesh.rt_stats();
    assert!(mesh.rt_reason().is_none(), "{:?}", mesh.rt_reason());
    assert_eq!(stats.triangles, 4);
    assert_eq!(
        stats.bytes, 1584,
        "48-byte vertices + exact hit tables + AS/pair planning terms"
    );
    assert_eq!(stats.instances, 0, "RT instances are published by update");
    let mirror = draw(
        &backend,
        &mut mesh,
        &instances(),
        &[0..1, 1..2],
        &lighting([0.0; 3]),
    );
    assert_eq!(mesh.rt_stats().instances, 2);
    assert!(mesh.rt_stats().active);
    let reflected = center(&mirror);
    // Normal incidence, roughness=1: diffuse red 0.96 plus white GGX
    // 0.04/4, then exposure 0.5. The white lobe is physical, not texture noise.
    assert_rgb_near(reflected, [0.485, 0.005, 0.005]);
    // The same instance set with RT resolved off has no environment light to
    // reflect, so the mirror pixel is exactly black.
    let off = backend.resolve_graphics(&requested(RayTracingRequest::Off));
    let mut raster = renderer(&backend, &off).unwrap();
    let plain = draw(
        &backend,
        &mut raster,
        &instances(),
        &[0..1, 1..2],
        &lighting([0.0; 3]),
    );
    assert_eq!(center(&plain), [0, 0, 0]);
}

fn srgb8(linear: f32) -> u8 {
    let encoded = if linear <= 0.003_130_8 {
        12.92 * linear
    } else {
        1.055 * linear.powf(1.0 / 2.4) - 0.055
    };
    (encoded.clamp(0.0, 1.0) * 255.0).round() as u8
}

fn assert_rgb_near(actual: [u8; 3], linear: [f32; 3]) {
    let expected = linear.map(srgb8);
    for channel in 0..3 {
        assert!(
            actual[channel].abs_diff(expected[channel]) <= 2,
            "actual={actual:?}, expected={expected:?}, channel={channel}"
        );
    }
}

fn textured_target() -> MeshAsset {
    let mut asset = source();
    asset.materials[1].base_color = [1.0; 4];
    for (vertex, uv) in asset.primitives[1].vertices.iter_mut().zip([
        [0.0, 0.0],
        [1.0, 0.0],
        [1.0, 1.0],
        [0.0, 1.0],
    ]) {
        vertex.uv = uv;
    }
    asset
}

fn map(asset: &mut MeshAsset, width: u32, height: u32, rgba8: Vec<u8>, linear: bool) -> usize {
    let image = asset.images.len();
    asset.images.push(Image {
        width,
        height,
        rgba8,
    });
    let id = asset.textures.len();
    asset.textures.push(Texture {
        image,
        wrap: [Wrap::Clamp; 2],
        linear,
        min_linear: linear,
    });
    id
}

fn reflected_sample(
    backend: &Backend,
    settings: &ResolvedGraphicsSettings,
    asset: &MeshAsset,
    placed: &[MeshInstance],
    lights: &SurfaceLighting,
) -> [u8; 3] {
    let mut renderer = MeshRenderer::with_surface(
        backend,
        asset,
        SurfaceSettings {
            format: OffscreenTarget::FORMAT,
            capacity: 2,
            quality: settings.surface_quality,
            reflections: true,
            shadows: None,
            ray_tracing: Some(settings),
        },
    )
    .unwrap();
    let pixel = center(&draw(backend, &mut renderer, placed, &[0..1, 1..2], lights));
    assert!(renderer.rt_stats().active, "{:?}", renderer.rt_reason());
    pixel
}

#[test]
fn reflected_uvs_use_srgb_decode_before_bilinear_filtering() {
    let Some(backend) = backend(RayTracingRequest::Local) else {
        return;
    };
    let settings = backend.resolve_graphics(&requested(RayTracingRequest::Local));
    let mut asset = textured_target();
    let id = map(
        &mut asset,
        2,
        2,
        vec![
            128, 0, 0, 255, 0, 64, 0, 255, 0, 0, 255, 255, 255, 255, 255, 255,
        ],
        true,
    );
    asset.materials[1].base_texture = Some(id);
    let mut placed = instances();
    placed[1].position_m.y = 0.75; // reflection hits UV=(0.5,0.25), not UV=(0.5,0.5).
    let pixel = reflected_sample(&backend, &settings, &asset, &placed, &lighting([0.0; 3]));
    let decode = |byte: f32| ((byte / 255.0 + 0.055) / 1.055).powf(2.4);
    assert_rgb_near(
        pixel,
        [
            0.96 * decode(128.0) * 0.25 + 0.005,
            0.96 * decode(64.0) * 0.25 + 0.005,
            0.005,
        ],
    );
}

#[test]
fn reflected_texture_wrap_modes_and_colour_spaces_match_raster_resources() {
    let Some(backend) = backend(RayTracingRequest::Local) else {
        return;
    };
    let settings = backend.resolve_graphics(&requested(RayTracingRequest::Local));
    let mut asset = textured_target();
    let id = map(
        &mut asset,
        2,
        1,
        vec![255, 0, 0, 255, 0, 255, 0, 255],
        false,
    );
    asset.materials[1].base_texture = Some(id);
    for (u, wrap, red) in [
        (1.25, Wrap::Repeat, true),
        (1.25, Wrap::Mirror, false),
        (1.25, Wrap::Clamp, false),
        (-0.25, Wrap::Repeat, false),
        (-0.25, Wrap::Mirror, true),
        (-0.25, Wrap::Clamp, true),
    ] {
        asset.textures[id].wrap = [wrap; 2];
        for v in &mut asset.primitives[1].vertices {
            v.uv = [u, 0.5];
        }
        let pixel = reflected_sample(
            &backend,
            &settings,
            &asset,
            &instances(),
            &lighting([0.0; 3]),
        );
        assert_rgb_near(
            pixel,
            if red {
                [0.485, 0.005, 0.005]
            } else {
                [0.005, 0.485, 0.005]
            },
        );
    }
    // The SAME source image/texture can be sRGB base colour and linear MR:
    // sharing only by image ID (without colour space) would corrupt one role.
    asset.images[0] = Image {
        width: 1,
        height: 1,
        rgba8: vec![128, 128, 128, 255],
    };
    asset.materials[1].metallic = 1.0;
    asset.materials[1].metallic_roughness_texture = Some(id);
    let mr = 128.0_f32 / 255.0;
    let albedo = ((mr + 0.055) / 1.055).powf(2.4);
    let fresnel = 0.04 * (1.0 - mr) + albedo * mr;
    let expected = ((1.0 - fresnel) * (1.0 - mr) * albedo + fresnel / (4.0 * mr.powi(4))) * 0.5;
    assert_rgb_near(
        reflected_sample(
            &backend,
            &settings,
            &asset,
            &instances(),
            &lighting([0.0; 3]),
        ),
        [expected; 3],
    );
}

#[test]
fn reflected_mr_channels_and_quality_control_hit_shading() {
    let Some(backend) = backend(RayTracingRequest::Local) else {
        return;
    };
    let settings = backend.resolve_graphics(&requested(RayTracingRequest::Local));
    let mut asset = textured_target();
    asset.materials[1].base_color = [0.5, 0.5, 0.5, 1.0];
    asset.materials[1].metallic = 1.0;
    let id = map(&mut asset, 1, 1, vec![0, 255, 0, 255], false);
    asset.materials[1].metallic_roughness_texture = Some(id);
    let dielectric = reflected_sample(
        &backend,
        &settings,
        &asset,
        &instances(),
        &lighting([0.0; 3]),
    );
    assert_rgb_near(dielectric, [0.245; 3]); // factor 1 * B=0, roughness G=1.
    asset.images[0].rgba8[2] = 255;
    let metal = reflected_sample(
        &backend,
        &settings,
        &asset,
        &instances(),
        &lighting([0.0; 3]),
    );
    assert_rgb_near(metal, [0.0625; 3]);
    asset.images[0].rgba8[1] = 128;
    let smooth = reflected_sample(
        &backend,
        &settings,
        &asset,
        &instances(),
        &lighting([0.0; 3]),
    );
    assert!(
        smooth[0] > metal[0] + 100,
        "roughness must change GGX, {smooth:?} vs {metal:?}"
    );
    for quality in [
        thessa_graphics::Quality::Low,
        thessa_graphics::Quality::Medium,
    ] {
        let mut cheaper = settings.clone();
        cheaper.surface_quality = quality;
        let fallback = reflected_sample(
            &backend,
            &cheaper,
            &asset,
            &instances(),
            &lighting([0.0; 3]),
        );
        assert_rgb_near(fallback, [0.0625; 3]); // MR texture omitted, factors preserved.
    }
}

#[test]
fn hit_roughness_yields_to_environment_instead_of_tracing_a_perfect_mirror() {
    let Some(backend) = backend(RayTracingRequest::Local) else {
        return;
    };
    let settings = backend.resolve_graphics(&requested(RayTracingRequest::Local));
    let mut asset = textured_target();
    asset.materials[1].metallic = 1.0;
    let id = map(&mut asset, 1, 1, vec![0, 255, 255, 255], false);
    asset.materials[1].metallic_roughness_texture = Some(id);
    let lights = SurfaceLighting {
        ambient_lux: [std::f32::consts::PI; 4],
        ..Default::default()
    };
    assert_rgb_near(
        reflected_sample(&backend, &settings, &asset, &instances(), &lights),
        [0.5; 3],
    );
    asset.images[0].rgba8[1] = 0;
    // At roughness zero the ray returns to the primary mirror, exhausting the
    // two-hit chain. It must not receive the rough hit's environment reduction.
    assert_eq!(
        reflected_sample(&backend, &settings, &asset, &instances(), &lights),
        [0; 3]
    );
}

#[test]
fn reflected_ao_and_normal_maps_obey_the_existing_material_contract() {
    let Some(backend) = backend(RayTracingRequest::Local) else {
        return;
    };
    let settings = backend.resolve_graphics(&requested(RayTracingRequest::Local));
    let mut primary = source();
    let ao = map(&mut primary, 1, 1, vec![0, 0, 0, 255], false);
    primary.materials[0].occlusion_texture = Some(ao);
    assert_eq!(
        reflected_sample(
            &backend,
            &settings,
            &primary,
            &instances(),
            &lighting([0.0; 3])
        ),
        [0; 3]
    );
    primary.materials[0].occlusion_strength = 0.5;
    assert_rgb_near(
        reflected_sample(
            &backend,
            &settings,
            &primary,
            &instances(),
            &lighting([0.0; 3]),
        ),
        [0.2425, 0.0025, 0.0025],
    );
    let mut asset = textured_target();
    let id = map(&mut asset, 1, 1, vec![0, 0, 0, 255], false);
    asset.materials[1].occlusion_texture = Some(id);
    let ambient = SurfaceLighting {
        ambient_lux: [std::f32::consts::PI; 4],
        ..Default::default()
    };
    assert_eq!(
        reflected_sample(&backend, &settings, &asset, &instances(), &ambient),
        [0; 3]
    );
    asset.materials[1].occlusion_strength = 0.5;
    assert_rgb_near(
        reflected_sample(&backend, &settings, &asset, &instances(), &ambient),
        [0.25; 3],
    );
    asset.materials[1].occlusion_texture = None;
    let normal = map(&mut asset, 1, 1, vec![255, 128, 128, 255], false);
    asset.materials[1].normal_texture = Some(normal);
    let grazed = reflected_sample(
        &backend,
        &settings,
        &asset,
        &instances(),
        &lighting([0.0; 3]),
    );
    assert!(
        grazed[0] < 24,
        "normal map should turn the hit away from +X sun, {grazed:?}"
    );
    // Degenerate UVs use the unperturbed smooth normal, as the raster path does.
    for vertex in &mut asset.primitives[1].vertices {
        vertex.uv = [0.5; 2];
    }
    assert_rgb_near(
        reflected_sample(
            &backend,
            &settings,
            &asset,
            &instances(),
            &lighting([0.0; 3]),
        ),
        [0.485; 3],
    );
}

#[test]
fn scene_ray_shadows_work_without_reflections_or_an_atlas() {
    let Some(backend) = backend(RayTracingRequest::Local) else {
        return;
    };
    let mut asset = textured_target();
    asset.materials[0].metallic = 0.0;
    asset.materials[0].roughness = 1.0;
    asset.primitives[0] = quad(1.0, 0);
    asset.primitives[1] = quad(0.25, 1);
    let placed = [
        MeshInstance {
            position_m: DVec3::new(0.0, 0.0, -2.0),
            rotation: DQuat::IDENTITY,
            scale: 1.0,
        },
        MeshInstance {
            position_m: DVec3::new(1.0, 0.0, -1.0),
            rotation: DQuat::IDENTITY,
            scale: 1.0,
        },
    ];
    let d = std::f32::consts::FRAC_1_SQRT_2;
    let lights = SurfaceLighting {
        directions: [[d, 0.0, d, 0.0], [0.0; 4], [0.0; 4]],
        colors_lux: [[std::f32::consts::PI; 4], [0.0; 4], [0.0; 4]],
        ..Default::default()
    };
    for (mode, shadows, distance) in [
        (RayTracingRequest::Local, true, 100.0),
        (RayTracingRequest::Local, true, 0.5),
        (RayTracingRequest::Local, false, 100.0),
        (RayTracingRequest::Off, true, 100.0),
    ] {
        let mut request = requested(mode);
        request.renderer.specular_reflections = false;
        request.shadows.enabled = shadows;
        request.raytracing.max_distance_m = distance;
        let settings = backend.resolve_graphics(&request);
        let mut mesh = MeshRenderer::with_surface(
            &backend,
            &asset,
            SurfaceSettings {
                format: OffscreenTarget::FORMAT,
                capacity: 2,
                quality: settings.surface_quality,
                reflections: false,
                shadows: None,
                ray_tracing: Some(&settings),
            },
        )
        .unwrap();
        let pixel = center(&draw(&backend, &mut mesh, &placed, &[0..1, 1..2], &lights));
        let tracing = mode == RayTracingRequest::Local && shadows;
        if tracing && distance > 2.0 {
            assert_eq!(
                pixel, [0; 3],
                "off-camera scene geometry blocks stellar rays"
            );
        } else {
            assert_rgb_near(pixel, [0.48 * d; 3]);
        }
        if tracing {
            assert!(mesh.rt_stats().active);
            assert_eq!(
                mesh.rt_stats().bytes,
                944,
                "shadow-only keeps 16-byte positions and no hit tables/pairs/maps"
            );
        } else {
            assert_eq!(
                mesh.rt_stats().bytes,
                0,
                "both RT consumers disabled or RT off"
            );
        }
    }
}

#[test]
fn shadow_rays_occlude_a_reflected_hit_without_occluding_the_reflection_ray() {
    let Some(backend) = backend(RayTracingRequest::Local) else {
        return;
    };
    let mut asset = source();
    asset.materials.push(Material {
        base_color: [0.0, 0.0, 0.0, 1.0],
        metallic: 0.0,
        roughness: 1.0,
        ..Default::default()
    });
    asset.primitives.push(quad(0.25, 2));
    let mut placed = instances();
    placed.push(MeshInstance {
        position_m: DVec3::new(-1.5, 1.5, -2.0),
        rotation: DQuat::from_rotation_y(std::f64::consts::FRAC_PI_2),
        scale: 1.0,
    });
    let d = std::f32::consts::FRAC_1_SQRT_2;
    let mut lights = lighting([0.0; 3]);
    lights.directions[0] = [d, d, 0.0, 0.0];
    for shadows in [false, true] {
        let mut request = requested(RayTracingRequest::Local);
        request.shadows.enabled = shadows;
        let settings = backend.resolve_graphics(&request);
        let mut mesh = MeshRenderer::with_surface(
            &backend,
            &asset,
            SurfaceSettings {
                format: OffscreenTarget::FORMAT,
                capacity: 3,
                quality: settings.surface_quality,
                reflections: true,
                shadows: None,
                ray_tracing: Some(&settings),
            },
        )
        .unwrap();
        let pixel = center(&draw(
            &backend,
            &mut mesh,
            &placed,
            &[0..1, 1..2, 2..3],
            &lights,
        ));
        if shadows {
            assert_eq!(pixel, [0; 3]);
        } else {
            assert!(pixel[0] > 100 && pixel[0] > pixel[1] * 4, "{pixel:?}");
        }
    }
}

#[test]
fn too_many_hit_texture_bindings_report_raster_fallback_instead_of_truncating() {
    let Some(backend) = backend(RayTracingRequest::Local) else {
        return;
    };
    let settings = backend.resolve_graphics(&requested(RayTracingRequest::Local));
    let mut asset = source();
    let count = backend
        .device
        .limits()
        .max_sampled_textures_per_shader_stage
        .min(backend.device.limits().max_samplers_per_shader_stage)
        .saturating_sub(5)
        + 1;
    for _ in 0..count {
        let id = map(&mut asset, 1, 1, vec![255; 4], false);
        let material = asset.materials.len();
        asset.materials.push(Material {
            base_texture: Some(id),
            ..Default::default()
        });
        asset.primitives.push(quad(0.1, material));
    }
    let mesh = MeshRenderer::with_surface(
        &backend,
        &asset,
        SurfaceSettings {
            format: OffscreenTarget::FORMAT,
            capacity: 2,
            quality: settings.surface_quality,
            reflections: true,
            shadows: None,
            ray_tracing: Some(&settings),
        },
    )
    .unwrap();
    assert_eq!(mesh.rt_stats().bytes, 0);
    assert!(mesh.rt_reason().unwrap().contains("material bindings"));
    let mut shadows = requested(RayTracingRequest::Local);
    shadows.renderer.specular_reflections = false;
    shadows.shadows.enabled = true;
    let shadows = backend.resolve_graphics(&shadows);
    let shadow_only = MeshRenderer::with_surface(
        &backend,
        &asset,
        SurfaceSettings {
            format: OffscreenTarget::FORMAT,
            capacity: 2,
            quality: shadows.surface_quality,
            reflections: false,
            shadows: None,
            ray_tracing: Some(&shadows),
        },
    )
    .unwrap();
    assert!(
        shadow_only.rt_reason().is_none(),
        "shadow-only needs no material bindings: {:?}",
        shadow_only.rt_reason()
    );
    assert!(shadow_only.rt_stats().bytes > 0);
}

#[test]
fn miss_reproduces_the_raster_environment_term_exactly() {
    let Some(backend) = backend(RayTracingRequest::Auto) else {
        return;
    };
    let settings = backend.resolve_graphics(&requested(RayTracingRequest::Auto));
    if !settings.ray_tracing.is_active() {
        return;
    }
    let lights = lighting([std::f32::consts::PI; 3]);
    let mirror_only = &instances()[..1];
    // Only the mirror instance draws the mirror primitive; the target
    // primitive has no instance, so every reflection ray leaves the scene.
    let ranges = [0..1u32, 0..0];
    let mut with_rt = renderer(&backend, &settings).unwrap();
    let traced = draw(&backend, &mut with_rt, mirror_only, &ranges, &lights);
    assert_eq!(with_rt.rt_stats().instances, 1);
    let off = backend.resolve_graphics(&requested(RayTracingRequest::Off));
    let mut without_rt = renderer(&backend, &off).unwrap();
    let plain = draw(&backend, &mut without_rt, mirror_only, &ranges, &lights);
    assert_eq!(traced, plain);
    assert_eq!(without_rt.rt_stats().passes, 0);
}

#[test]
fn ray_tracing_off_publishes_no_acceleration_structure_work() {
    let Some(backend) = backend(RayTracingRequest::Off) else {
        return;
    };
    let settings = backend.resolve_graphics(&requested(RayTracingRequest::Off));
    assert!(!settings.ray_tracing.is_active());
    let mut mesh = renderer(&backend, &settings).unwrap();
    assert!(mesh.rt_reason().is_none());
    let image = draw(
        &backend,
        &mut mesh,
        &instances(),
        &[0..1, 1..2],
        &lighting([0.0; 3]),
    );
    let stats = mesh.rt_stats();
    assert!(!stats.active);
    assert_eq!(stats.passes, 0);
    assert_eq!(stats.bytes, 0);
    // Raster-only means no RT bind group, so the shader must produce exactly
    // the environment-only reflection: black without ambient light.
    assert_eq!(center(&image), [0, 0, 0]);
}

/// In-process RT cost probe. The same device, scene, lights and shadow atlas
/// are rendered twice inside one process — RT resolved off, then RT resolved
/// local — so the difference is a frame-time measurement instead of the
/// cross-run capture noise that made the preview PNGs useless as evidence.
/// The mirror quad is screen-filling, so its reflective pixels are traversed
/// and shaded rather than mostly idling on the miss path.
#[test]
#[ignore = "hardware: requires a ray-query capable adapter"]
fn fixed_source_ray_query_frame_cost() {
    const WIDTH: u32 = 1280;
    const HEIGHT: u32 = 720;
    let Some(backend) = backend(RayTracingRequest::Local) else {
        return;
    };
    let target = OffscreenTarget::new(&backend, WIDTH, HEIGHT).unwrap();
    let page = HeightPage::bake(&[0.0; 4], 2, 0.001).unwrap();
    let patches = (0..16)
        .map(|i| TerrainPatch {
            anchor: TileAnchor::new(TileKey::new(4, 5, i % 4, i / 4).unwrap(), 1000.0).unwrap(),
            page: &page,
        })
        .collect::<Vec<_>>();
    // The mesh objects sit well above the terrain plane: otherwise the
    // terrain's own depth occludes the hit shading and the probe would
    // measure a traversal whose result is invisible. Ambient keeps the hit
    // term nonzero even where the mirror shadows the target.
    let origin = DVec3::new(0.0, 50.0, 1010.0);
    let view = camera(origin);
    let lights = lighting([std::f32::consts::PI; 3]);
    // Screen-filling mirror at 45° plus a large target in its reflected
    // direction, so nearly every covered pixel commits a hit.
    let scene = MeshAsset {
        primitives: vec![quad(60.0, 0), quad(60.0, 1)],
        materials: source().materials,
        textures: vec![],
        images: vec![],
    };
    let placed = vec![
        MeshInstance {
            position_m: origin + DVec3::new(0.0, 0.0, -2.0),
            rotation: DQuat::from_rotation_y(-std::f64::consts::FRAC_PI_4),
            scale: 1.0,
        },
        MeshInstance {
            position_m: origin + DVec3::new(-60.0, 0.0, -2.0),
            rotation: DQuat::from_rotation_y(std::f64::consts::FRAC_PI_2),
            scale: 1.0,
        },
    ];
    let ranges = [0..1u32, 1..2];
    let mut images = Vec::new();
    for mode in [RayTracingRequest::Off, RayTracingRequest::Local] {
        let mut request = requested(mode);
        request.shadows.enabled = true;
        let settings = backend.resolve_graphics(&request);
        let mut terrain = TerrainRenderer::with_capacity_and_lighting(
            &backend,
            &patches,
            OffscreenTarget::FORMAT,
            TerrainCapacity {
                patches: 16,
                height_grid_size: 2,
                material_layers: 1,
            },
            Some(&settings),
        )
        .unwrap();
        terrain.update_view(&backend, &view).unwrap();
        terrain.update_lighting(&backend, &lights).unwrap();
        terrain.update_shadows(&backend, origin, &lights).unwrap();
        let mut mesh = MeshRenderer::with_surface(
            &backend,
            &scene,
            SurfaceSettings {
                format: OffscreenTarget::FORMAT,
                capacity: 2,
                quality: settings.surface_quality,
                reflections: settings.specular_reflections,
                shadows: terrain.shadows(),
                ray_tracing: Some(&settings),
            },
        )
        .unwrap();
        let mut timings = Vec::new();
        for frame in 0..150 {
            mesh.update_selected(&backend, &view, &lights, &placed, Some(&ranges))
                .unwrap();
            let started = std::time::Instant::now();
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            terrain.encode_shadows(&mut encoder, Some(&mesh));
            terrain.encode(&mut encoder, &target.color, &target.depth);
            mesh.encode(&mut encoder, &target.color, &target.depth);
            backend.queue.submit([encoder.finish()]);
            backend
                .device
                .poll(wgpu::PollType::wait_indefinitely())
                .unwrap();
            if frame >= 30 {
                timings.push(started.elapsed().as_secs_f64() * 1000.0);
            }
        }
        timings.sort_by(f64::total_cmp);
        let stats = mesh.rt_stats();
        println!(
            "fixed_source_ray_query adapter={} mode={mode:?} resolution={WIDTH}x{HEIGHT} rt_instances={} rt_triangles={} rt_scene_bytes={} last_frame_as_builds={} atlas_bytes={} measured_frames=120 median_ms={:.3} p95_ms={:.3}; CPU encode/submit + GPU completion incl. the shadow atlas, not isolated GPU timestamps or game FPS",
            backend.adapter_name,
            stats.instances,
            stats.triangles,
            stats.bytes,
            stats.passes,
            terrain.shadows().map_or(0, |s| s.storage_bytes()),
            timings[60],
            timings[114]
        );
        if !settings.ray_tracing.is_active() {
            assert_eq!(stats.bytes, 0, "RT off must allocate no RT scene");
            assert_eq!(stats.passes, 0, "RT off must build nothing");
        }
        images.push(target.read_rgba8(&backend).unwrap());
    }
    let changed = images[0]
        .as_chunks::<4>()
        .0
        .iter()
        .zip(images[1].as_chunks::<4>().0)
        .filter(|(off, local)| off != local)
        .count();
    println!(
        "fixed_source_ray_query rt_on_vs_off_changed_pixels={changed} of {}",
        WIDTH as usize * HEIGHT as usize
    );
    assert!(
        changed > 10_000,
        "the cost probe must actually exercise reflections, changed={changed}"
    );
}

/// A validated environment grid built from a texel function. `column == 0` and
/// `column == 3` of a 4-wide grid straddle -X, which is the mirror's reflected
/// direction in this fixture.
fn environment(
    width: u32,
    height: u32,
    texel: impl Fn(u32, u32) -> [f32; 3],
) -> EnvironmentRadiance {
    let mut texels = Vec::with_capacity((width * height) as usize);
    for row in 0..height {
        for column in 0..width {
            texels.push(texel(column, row));
        }
    }
    EnvironmentRadiance::new(width, height, texels).expect("valid test grid")
}

/// Not calling `update_environment` must be exactly the disabled state, which
/// is exactly the legacy isotropic reflection term.
#[test]
fn disabled_environment_is_byte_identical_to_the_unset_renderer() {
    let Some(backend) = backend(RayTracingRequest::Off) else {
        return;
    };
    let settings = backend.resolve_graphics(&requested(RayTracingRequest::Off));
    let lights = lighting([std::f32::consts::PI; 3]);
    let mut untouched = renderer(&backend, &settings).unwrap();
    let reference = draw(
        &backend,
        &mut untouched,
        &instances(),
        &[0..1, 1..2],
        &lights,
    );
    let (enabled, width, height, storage, uploads) = untouched.environment_stats();
    assert!(!enabled, "an unset renderer must start disabled");
    assert_eq!((width, height, storage, uploads), (0, 0, 0, 0));

    let mut cleared = renderer(&backend, &settings).unwrap();
    cleared.update_environment(&backend, None);
    let image = draw(&backend, &mut cleared, &instances(), &[0..1, 1..2], &lights);
    assert_eq!(reference, image);
    let (enabled, width, height, storage, uploads) = cleared.environment_stats();
    assert!(!enabled);
    assert_eq!((width, height, storage), (0, 0, 0));
    assert_eq!(uploads, 0, "already disabled performs no metadata write");
}

/// A uniform field whose texels are `ambient_lux / PI` is the legacy isotropic
/// environment term, so binding it must not change the frame beyond bilinear
/// mix rounding. This is the contract that lets a physical sky field replace
/// the constant without a separate code path.
#[test]
fn uniform_field_reproduces_the_isotropic_environment_term() {
    let Some(backend) = backend(RayTracingRequest::Off) else {
        return;
    };
    let settings = backend.resolve_graphics(&requested(RayTracingRequest::Off));
    let ambient = 1.0_f32;
    let lights = lighting([ambient; 3]);
    let mut raster = renderer(&backend, &settings).unwrap();
    let reference = draw(&backend, &mut raster, &instances(), &[0..1, 1..2], &lights);

    let mut fielded = renderer(&backend, &settings).unwrap();
    let radiance = ambient / std::f32::consts::PI;
    let grid = environment(4, 2, |_, _| [radiance; 3]);
    assert!((grid.ambient_lux()[0] - ambient).abs() < 1e-4);
    fielded.update_environment(&backend, Some(&grid));
    let (enabled, width, height, storage, uploads) = fielded.environment_stats();
    assert!(enabled);
    assert_eq!((width, height), (4, 2));
    assert_eq!(storage, 4 * 2 * 16);
    assert_eq!(uploads, 1);
    assert!((fielded.environment_ambient_lux()[0] - ambient).abs() < 1e-4);
    let image = draw(&backend, &mut fielded, &instances(), &[0..1, 1..2], &lights);

    let max = image
        .iter()
        .zip(&reference)
        .map(|(a, b)| a.abs_diff(*b))
        .max()
        .unwrap();
    let differing = image.iter().zip(&reference).filter(|(a, b)| a != b).count();
    println!(
        "uniform_field max_channel_delta={max} differing_bytes={differing} of {}",
        reference.len()
    );
    assert!(
        max <= 1,
        "uniform field must match the legacy term, max {max}"
    );
    assert!(differing * 16 < reference.len(), "differing {differing}");
    assert_eq!(center(&image), center(&reference));
}

/// Reflections read the field: a field that carries radiance only on the
/// mirror's reflected side shows up as that radiance, while the same scene
/// without a field reflects nothing.
#[test]
fn mirror_reflects_the_field_radiance_of_its_reflected_direction() {
    let Some(backend) = backend(RayTracingRequest::Off) else {
        return;
    };
    let settings = backend.resolve_graphics(&requested(RayTracingRequest::Off));
    // No direct lights at all: the mirror pixel can only come from the field.
    let lights = lighting([0.0; 3]);
    let grid = environment(4, 2, |column, _| {
        if column == 0 || column == 3 {
            [1.0, 0.0, 0.0]
        } else {
            [0.0, 0.0, 1.0]
        }
    });
    let mut fielded = renderer(&backend, &settings).unwrap();
    fielded.update_environment(&backend, Some(&grid));
    let image = draw(&backend, &mut fielded, &instances(), &[0..1, 1..2], &lights);
    let reflected = center(&image);
    assert!(
        reflected[0] > 32 && reflected[2] < 8,
        "mirror must show the -X field radiance, got {reflected:?}"
    );
    // The +X side of the field is blue, so the pixel is not just "some field".
    assert!(reflected[0] > reflected[2] * 4, "{reflected:?}");

    let mut plain = renderer(&backend, &settings).unwrap();
    let none = draw(&backend, &mut plain, &instances(), &[0..1, 1..2], &lights);
    assert_eq!(center(&none), [0, 0, 0]);
}

/// The RT miss path must land on the same directional field value as the
/// raster term: with a directional field bound, RT on and RT off are still
/// byte-identical when nothing is hit.
#[test]
fn miss_reproduces_the_raster_directional_field_exactly() {
    let Some(backend) = backend(RayTracingRequest::Auto) else {
        return;
    };
    let settings = backend.resolve_graphics(&requested(RayTracingRequest::Auto));
    if !settings.ray_tracing.is_active() {
        return;
    }
    let lights = lighting([std::f32::consts::PI; 3]);
    let mirror_only = &instances()[..1];
    let ranges = [0..1u32, 0..0];
    let grid = environment(4, 2, |column, _| {
        if column == 0 || column == 3 {
            [1.0, 0.5, 0.0]
        } else {
            [0.0, 0.5, 1.0]
        }
    });
    let mut with_rt = renderer(&backend, &settings).unwrap();
    with_rt.update_environment(&backend, Some(&grid));
    let traced = draw(&backend, &mut with_rt, mirror_only, &ranges, &lights);
    assert_eq!(with_rt.rt_stats().instances, 1);
    let off = backend.resolve_graphics(&requested(RayTracingRequest::Off));
    let mut without_rt = renderer(&backend, &off).unwrap();
    without_rt.update_environment(&backend, Some(&grid));
    let plain = draw(&backend, &mut without_rt, mirror_only, &ranges, &lights);
    assert_eq!(traced, plain);
    assert_eq!(without_rt.rt_stats().passes, 0);
}

/// White metal has Fresnel=1 and cannot catch a squared Fresnel weight.
/// Exercise the original miss bug with dielectric F0=0.04, partial mirror
/// fade and a non-unit AO texture, for both uniform and directional fields.
#[test]
fn dielectric_miss_applies_fresnel_and_occlusion_once() {
    let Some(backend) = backend(RayTracingRequest::Auto) else {
        return;
    };
    let local = backend.resolve_graphics(&requested(RayTracingRequest::Auto));
    assert!(local.ray_tracing.is_active());
    let off = backend.resolve_graphics(&requested(RayTracingRequest::Off));
    let mut asset = source();
    asset.materials[0].metallic = 0.0;
    asset.materials[0].roughness = 0.33;
    asset.materials[0].occlusion_texture = Some(0);
    asset.textures.push(Texture {
        image: 0,
        wrap: [Wrap::Clamp; 2],
        linear: false,
        min_linear: false,
    });
    asset.images.push(Image {
        width: 1,
        height: 1,
        rgba8: vec![128, 128, 128, 255],
    });
    let lights = SurfaceLighting {
        ambient_lux: [std::f32::consts::PI; 4],
        ..Default::default()
    };
    for directional in [false, true] {
        let grid = environment(8, 4, |column, row| {
            if directional {
                [0.2 + column as f32 * 0.1, 0.25, 0.1 + row as f32 * 0.2]
            } else {
                [1.0; 3]
            }
        });
        let images: Vec<_> = [&off, &local]
            .into_iter()
            .map(|settings| {
                let mut mesh = MeshRenderer::with_surface(
                    &backend,
                    &asset,
                    SurfaceSettings {
                        format: OffscreenTarget::FORMAT,
                        capacity: 2,
                        quality: settings.surface_quality,
                        reflections: true,
                        shadows: None,
                        ray_tracing: Some(settings),
                    },
                )
                .unwrap();
                mesh.update_environment(&backend, Some(&grid));
                draw(
                    &backend,
                    &mut mesh,
                    &instances()[..1],
                    &[0..1, 0..0],
                    &lights,
                )
            })
            .collect();
        let max_delta = images[0]
            .iter()
            .zip(&images[1])
            .map(|(a, b)| a.abs_diff(*b))
            .max()
            .unwrap();
        assert!(
            max_delta <= 1,
            "directional={directional}, max delta={max_delta}"
        );
        assert!(center(&images[0])[0] > 8, "non-vacuous dielectric oracle");
    }
}

#[test]
fn disabling_a_previously_bound_environment_releases_the_grid() {
    let Some(backend) = backend(RayTracingRequest::Off) else {
        return;
    };
    let settings = backend.resolve_graphics(&requested(RayTracingRequest::Off));
    let mut mesh = renderer(&backend, &settings).unwrap();
    let lights = lighting([std::f32::consts::PI; 3]);
    let reference = draw(&backend, &mut mesh, &instances(), &[0..1, 1..2], &lights);
    for size in [4, 32, 8] {
        let grid = environment(size, size / 2, |_, _| [0.0, 0.0, 1.0]);
        mesh.update_environment(&backend, Some(&grid));
        let colored = draw(&backend, &mut mesh, &instances(), &[0..1, 1..2], &lights);
        assert_ne!(reference, colored);
    }
    mesh.update_environment(&backend, None);
    assert_eq!(mesh.environment_stats().3, 0);
    let disabled = draw(&backend, &mut mesh, &instances(), &[0..1, 1..2], &lights);
    assert_eq!(reference, disabled);
    let uploads = mesh.environment_stats().4;
    mesh.update_environment(&backend, None);
    assert_eq!(mesh.environment_stats().4, uploads);
}
