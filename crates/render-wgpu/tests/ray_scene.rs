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
    let target = OffscreenTarget::new(backend, SIZE, SIZE).unwrap();
    mesh.update_selected(
        backend,
        &camera(DVec3::ZERO),
        lights,
        mesh_instances,
        Some(ranges),
    )
    .unwrap();
    let mut encoder = backend.device.create_command_encoder(&Default::default());
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
    mesh.encode(&mut encoder, &target.color, &target.depth);
    backend.queue.submit([encoder.finish()]);
    target.read_rgba8(backend).unwrap()
}

/// Screen-center pixel, where the mirror quad sits.
fn center(bytes: &[u8]) -> [u8; 3] {
    let at = ((SIZE / 2) * SIZE + SIZE / 2) as usize * 4;
    [bytes[at], bytes[at + 1], bytes[at + 2]]
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
    assert!(
        reflected[0] > 32 && reflected[1] < 8 && reflected[2] < 8,
        "mirror must show the red quad it faces, got {reflected:?}"
    );
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
        let settings = backend.resolve_graphics(&requested(mode));
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
