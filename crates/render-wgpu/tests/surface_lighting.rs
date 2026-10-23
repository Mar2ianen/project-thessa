use glam::{DMat4, DQuat, DVec3};
use thessa_graphics::{Capabilities, Quality, RequestedGraphics, ResolvedGraphicsSettings};
use thessa_rcbt_core::{
    HeightPage,
    lighting::SurfaceLighting,
    tile_anchor::{TileAnchor, TileKey},
};
use thessa_render_core::{ViewParams, mesh::*};
use thessa_render_wgpu::{
    Backend, BackendError,
    mesh::MeshRenderer,
    terrain::{OffscreenTarget, TerrainCapacity, TerrainPatch, TerrainRenderer},
};

fn backend() -> Option<Backend> {
    match Backend::bring_up() {
        Ok(backend) => Some(backend),
        Err(BackendError::NoAdapter) => {
            eprintln!("SKIP: no GPU adapter");
            None
        }
        Err(error) => panic!("{error}"),
    }
}

fn quad(x: f32, z: f32, r: f32) -> Primitive {
    Primitive {
        vertices: [
            [-r + x, -r, z],
            [r + x, -r, z],
            [r + x, r, z],
            [-r + x, r, z],
        ]
        .map(|position| Vertex {
            position,
            normal: [0.0, 0.0, 1.0],
            uv: [0.5; 2],
        })
        .to_vec(),
        indices: vec![0, 1, 2, 0, 2, 3],
        material: 0,
    }
}
fn source() -> MeshAsset {
    MeshAsset {
        primitives: vec![quad(0.0, -4.0, 3.0), quad(1.0, -2.0, 0.6)],
        materials: vec![Material {
            metallic: 0.0,
            ..Default::default()
        }],
        textures: vec![],
        images: vec![],
    }
}
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
fn draw(
    backend: &Backend,
    mesh: &mut MeshRenderer,
    mut terrain: Option<&mut TerrainRenderer>,
) -> Vec<u8> {
    let target = OffscreenTarget::new(backend, 129, 129).unwrap();
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    if let Some(terrain) = terrain.as_deref_mut() {
        terrain.encode_shadows(&mut encoder, Some(mesh));
    }
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
    if let Some(terrain) = terrain {
        terrain.encode(&mut encoder, &target.color, &target.depth);
    }
    mesh.encode(&mut encoder, &target.color, &target.depth);
    backend.queue.submit([encoder.finish()]);
    target.read_rgba8(backend).unwrap()
}

fn cached_matches_redraw(
    backend: &Backend,
    mesh: &mut MeshRenderer,
    terrain: &mut TerrainRenderer,
) -> Vec<u8> {
    let first = draw(backend, mesh, Some(terrain));
    assert!(
        terrain.last_shadow_passes() > 0,
        "changed inputs must refresh depth"
    );
    let cached = draw(backend, mesh, Some(terrain));
    assert_eq!(terrain.last_shadow_passes(), 0);
    assert_eq!(
        cached, first,
        "cached depth differs from just-rendered depth"
    );
    terrain.invalidate_shadows();
    let forced = draw(backend, mesh, Some(terrain));
    assert!(terrain.last_shadow_passes() > 0);
    assert_eq!(
        forced, cached,
        "exact cache must be byte-identical to forced redraw"
    );
    first
}

#[test]
fn shared_shadow_atlas_occludes_each_light_independently_and_preserves_large_origin_precision() {
    shared_shadow_atlas_fixture(None);
}

#[test]
fn explicit_near_shadow_cover_keeps_shared_occlusion_cache_and_large_origin_precision() {
    shared_shadow_atlas_fixture(Some(3.0));
}

fn shared_shadow_atlas_fixture(first_m: Option<f64>) {
    let Some(backend) = backend() else {
        return;
    };
    let mut requested = RequestedGraphics::default();
    requested.shadows.cascades = 2;
    requested.shadows.map_size = 512;
    requested.shadows.normal_bias_m = 0.005;
    requested.shadows.quality = Quality::Low;
    let mut settings =
        ResolvedGraphicsSettings::from_requested(&requested, &Capabilities::unknown());
    settings.shadow_max_distance_m = 10.0;
    settings.shadow_first_cascade_distance_m = first_m;
    let page = HeightPage::bake(&[0.0; 4], 2, 0.001).unwrap();
    let patch = TerrainPatch {
        anchor: TileAnchor::new(TileKey::new(0, 0, 0, 0).unwrap(), 1000.0).unwrap(),
        page: &page,
    };
    let mut terrain = TerrainRenderer::with_capacity_and_lighting(
        &backend,
        &[patch],
        OffscreenTarget::FORMAT,
        TerrainCapacity {
            patches: 1,
            height_grid_size: 2,
            material_layers: 1,
        },
        Some(&settings),
    )
    .unwrap();
    assert_eq!(
        terrain.shadows().unwrap().storage_bytes(),
        3 * 2 * 512 * 512 * 4,
        "explicit near cover must not change the atlas allocation"
    );
    assert_eq!(
        terrain.shadows().unwrap().caster_distance_m(),
        30.0,
        "finite far caster cover is unchanged"
    );
    terrain.publish_partition(&backend, &[]).unwrap();
    let sun = DVec3::new(1.0, 0.0, 1.0).normalize();
    let other = DVec3::new(-1.0, 0.0, 1.0).normalize();
    let lights = SurfaceLighting {
        directions: [
            sun.as_vec3().extend(0.0).to_array(),
            other.as_vec3().extend(0.0).to_array(),
            [0.0; 4],
        ],
        colors_lux: [
            [std::f32::consts::PI, 0.0, 0.0, 0.0],
            [0.0, std::f32::consts::PI, 0.0, 0.0],
            [0.0; 4],
        ],
        ..Default::default()
    };
    let mesh_source = source();
    let mut mesh = MeshRenderer::with_surface(
        &backend,
        &mesh_source,
        thessa_render_wgpu::mesh::SurfaceSettings {
            format: OffscreenTarget::FORMAT,
            capacity: 2,
            quality: Quality::Medium,
            reflections: false,
            shadows: terrain.shadows(),
            ray_tracing: None,
        },
    )
    .unwrap();
    let sample =
        |bytes: &[u8], x: usize| bytes[(64 * 129 + x) * 4..(64 * 129 + x) * 4 + 3].to_vec();
    for origin in [DVec3::ZERO, DVec3::new(3e12, -2e12, 1e12)] {
        let view = camera(origin);
        terrain.update_view(&backend, &view).unwrap();
        terrain.update_shadows(&backend, origin, &lights).unwrap();
        mesh.update(
            &backend,
            &view,
            &lights,
            &[MeshInstance {
                position_m: origin,
                rotation: DQuat::IDENTITY,
                scale: 1.0,
            }],
        )
        .unwrap();
        let image = cached_matches_redraw(&backend, &mut mesh, &mut terrain);
        // Resubmitting identical instance/range bytes must preserve the cache.
        mesh.update(
            &backend,
            &view,
            &lights,
            &[MeshInstance {
                position_m: origin,
                rotation: DQuat::IDENTITY,
                scale: 1.0,
            }],
        )
        .unwrap();
        assert_eq!(draw(&backend, &mut mesh, Some(&mut terrain)), image);
        assert_eq!(terrain.last_shadow_passes(), 0);
        terrain.publish_partition(&backend, &[]).unwrap();
        assert_eq!(draw(&backend, &mut mesh, Some(&mut terrain)), image);
        assert_eq!(
            terrain.last_shadow_passes(),
            0,
            "identical partition publication retains depth"
        );
        let mut replacement = MeshRenderer::with_surface(
            &backend,
            &mesh_source,
            thessa_render_wgpu::mesh::SurfaceSettings {
                format: OffscreenTarget::FORMAT,
                capacity: 2,
                quality: Quality::Medium,
                reflections: false,
                shadows: terrain.shadows(),
                ray_tracing: None,
            },
        )
        .unwrap();
        replacement
            .update(
                &backend,
                &view,
                &lights,
                &[MeshInstance {
                    position_m: origin,
                    rotation: DQuat::IDENTITY,
                    scale: 1.0,
                }],
            )
            .unwrap();
        assert_eq!(
            cached_matches_redraw(&backend, &mut replacement, &mut terrain),
            image,
            "replacement source has its own cache identity even at the same instance revision"
        );
        mesh = replacement;
        let blocked = sample(&image, 36);
        let clear = sample(&image, 64);
        assert!(
            blocked[0] < 10 && blocked[1] > 100,
            "red source occluded, independent green source stays: {blocked:?}"
        );
        assert!(
            clear[0] > 100 && clear[1] > 100,
            "unoccluded receiver: {clear:?}"
        );
        assert_eq!(terrain.shadows().unwrap().active_passes(), 4);
        assert_eq!(
            terrain.shadows().unwrap().storage_bytes(),
            6 * 512 * 512 * 4
        );
        for (offset, ranges) in [
            (0.4, [0..1, 1..2]),
            (0.9, [0..1, 1..2]),
            (0.9, [0..1, 1..1]),
        ] {
            let placed = [
                MeshInstance {
                    position_m: origin,
                    rotation: DQuat::IDENTITY,
                    scale: 1.0,
                },
                MeshInstance {
                    position_m: origin + DVec3::X * offset,
                    rotation: DQuat::IDENTITY,
                    scale: 1.0,
                },
            ];
            mesh.update_selected(&backend, &view, &lights, &placed, Some(&ranges))
                .unwrap();
            cached_matches_redraw(&backend, &mut mesh, &mut terrain);
        }
        // Light intensity changes do not change opaque visibility; a new
        // direction or source activation does, even with unchanged casters.
        let mut changed_lights = lights;
        changed_lights.directions[0] = DVec3::new(0.3, 0.0, 1.0)
            .normalize()
            .as_vec3()
            .extend(0.0)
            .to_array();
        terrain
            .update_shadows(&backend, origin, &changed_lights)
            .unwrap();
        cached_matches_redraw(&backend, &mut mesh, &mut terrain);
        changed_lights.colors_lux[1] = [0.0; 4];
        terrain
            .update_shadows(&backend, origin, &changed_lights)
            .unwrap();
        cached_matches_redraw(&backend, &mut mesh, &mut terrain);
    }
    settings.shadow_enabled = false;
    assert!(
        thessa_render_wgpu::shadows::ShadowAtlas::new(&backend, &settings)
            .unwrap()
            .is_none(),
        "disabled allocates no atlas/pipelines"
    );
}

#[test]
fn smooth_mesh_shading_normals_do_not_change_the_shadow_receiver_plane() {
    let backend = Backend::bring_up().expect("requires the local hardware adapter");
    let mut requested = RequestedGraphics::default();
    requested.shadows.cascades = 1;
    requested.shadows.map_size = 512;
    requested.shadows.normal_bias_m = 0.0;
    requested.shadows.quality = Quality::High;
    let mut settings =
        ResolvedGraphicsSettings::from_requested(&requested, &Capabilities::unknown());
    settings.shadow_max_distance_m = 10.0;
    let page = HeightPage::bake(&[0.0; 4], 2, 0.001).unwrap();
    let patch = TerrainPatch {
        anchor: TileAnchor::new(TileKey::new(0, 0, 0, 0).unwrap(), 1000.0).unwrap(),
        page: &page,
    };
    let mut terrain = TerrainRenderer::with_capacity_and_lighting(
        &backend,
        &[patch],
        OffscreenTarget::FORMAT,
        TerrainCapacity {
            patches: 1,
            height_grid_size: 2,
            material_layers: 1,
        },
        Some(&settings),
    )
    .unwrap();
    terrain.publish_partition(&backend, &[]).unwrap();
    let mut source = source();
    source.primitives.truncate(1);
    for vertex in &mut source.primitives[0].vertices {
        // Flat z=-4 geometry, deliberately tilted/smooth interpolated normals.
        // The analytic receiver plane is still z=-4 everywhere, with no blocker.
        let x = vertex.position[0] / 3.0 * 0.6;
        vertex.normal = [x, 0.0, (1.0 - x * x).sqrt()];
    }
    let sun = DVec3::new(1.0, 0.2, 1.0).normalize();
    let lights = SurfaceLighting {
        directions: [sun.as_vec3().extend(0.0).to_array(), [0.0; 4], [0.0; 4]],
        colors_lux: [[std::f32::consts::PI; 4], [0.0; 4], [0.0; 4]],
        ..Default::default()
    };
    let render = |shadows: bool, terrain: &mut TerrainRenderer| {
        let mut mesh = MeshRenderer::with_surface(
            &backend,
            &source,
            thessa_render_wgpu::mesh::SurfaceSettings {
                format: OffscreenTarget::FORMAT,
                capacity: 1,
                quality: Quality::Medium,
                reflections: false,
                shadows: if shadows { terrain.shadows() } else { None },
                ray_tracing: None,
            },
        )
        .unwrap();
        let view = camera(DVec3::ZERO);
        terrain.update_view(&backend, &view).unwrap();
        terrain
            .update_shadows(&backend, DVec3::ZERO, &lights)
            .unwrap();
        mesh.update(
            &backend,
            &view,
            &lights,
            &[MeshInstance {
                position_m: DVec3::ZERO,
                rotation: DQuat::IDENTITY,
                scale: 1.0,
            }],
        )
        .unwrap();
        if shadows {
            draw(&backend, &mut mesh, Some(terrain))
        } else {
            draw(&backend, &mut mesh, None)
        }
    };
    let reference = render(false, &mut terrain);
    let actual = render(true, &mut terrain);
    let max_error = reference
        .iter()
        .zip(&actual)
        .map(|(a, b)| a.abs_diff(*b))
        .max()
        .unwrap();
    assert!(
        max_error <= 1,
        "unoccluded smooth-normal receiver differs by {max_error} sRGB codes with shadows enabled"
    );
}

#[test]
fn pbr_uses_linear_metal_roughness_maps_and_reflection_disable_has_no_hidden_specular() {
    let Some(backend) = backend() else {
        return;
    };
    let mut source = source();
    source.primitives.truncate(1);
    source.materials[0] = Material {
        base_color: [0.5, 0.25, 0.125, 1.0],
        metallic: 1.0,
        roughness: 0.5,
        ..Default::default()
    };
    let light = SurfaceLighting {
        directions: [[0.0, 0.0, 1.0, 0.0], [0.0; 4], [0.0; 4]],
        colors_lux: [[std::f32::consts::PI; 4], [0.0; 4], [0.0; 4]],
        ..Default::default()
    };
    let instance = MeshInstance {
        position_m: DVec3::ZERO,
        rotation: DQuat::IDENTITY,
        scale: 1.0,
    };
    let render = |source: &MeshAsset, quality: Quality, reflections: bool| {
        let mut renderer = MeshRenderer::with_surface(
            &backend,
            source,
            thessa_render_wgpu::mesh::SurfaceSettings {
                format: OffscreenTarget::FORMAT,
                capacity: 1,
                quality,
                reflections,
                shadows: None,
                ray_tracing: None,
            },
        )
        .unwrap();
        renderer
            .update(&backend, &camera(DVec3::ZERO), &light, &[instance])
            .unwrap();
        draw(&backend, &mut renderer, None)
    };
    let center = |bytes: &[u8]| bytes[(64 * 129 + 64) * 4..(64 * 129 + 64) * 4 + 3].to_vec();
    assert_eq!(
        center(&render(&source, Quality::Medium, false)),
        vec![0, 0, 0],
        "a metal has no diffuse substitute when reflections are off"
    );
    let rough = render(&source, Quality::Medium, true);
    // At N=V=L, roughness=.5 gives D=16/PI, G=1 and F=base.
    // E=PI and exposure=.5 therefore yield linear RGB=(1,.5,.25).
    for (actual, expected) in center(&rough).into_iter().zip([255u8, 188, 137]) {
        assert!(
            actual.abs_diff(expected) <= 2,
            "normal-incidence GGX oracle within two sRGB codes"
        );
    }
    source.materials[0].roughness = 128.0 / 255.0;
    let factors = render(&source, Quality::Medium, true);
    source.materials[0].roughness = 1.0;
    source.materials[0].metallic_roughness_texture = Some(0);
    source.textures.push(Texture {
        image: 0,
        wrap: [Wrap::Repeat; 2],
        linear: false,
        min_linear: false,
    });
    source.images.push(Image {
        width: 1,
        height: 1,
        rgba8: vec![0, 128, 255, 255],
    });
    assert_eq!(
        factors,
        render(&source, Quality::High, true),
        "roughness G and metallic B are linear data, not sRGB or the R channel"
    );
    assert!(center(&rough)[0] > 0);
    assert_eq!(
        center(&render(&source, Quality::Low, false)),
        vec![0, 0, 0],
        "low quality must not invent a metal diffuse term when reflections are disabled"
    );
    assert!(
        center(&render(&source, Quality::Low, true))[0] > 0,
        "low retains source reflection"
    );
    // Degenerate UVs must retain a finite geometric normal, not introduce NaN.
    source.materials[0].normal_texture = Some(0);
    assert!(center(&render(&source, Quality::High, true))[0] > 0);
}

#[test]
fn resident_terrain_casts_onto_mesh_without_using_the_camera_visibility_list() {
    let Some(backend) = backend() else {
        return;
    };
    let mut request = RequestedGraphics::default();
    request.shadows.cascades = 1;
    request.shadows.map_size = 512;
    request.shadows.quality = Quality::Low;
    request.shadows.normal_bias_m = 0.001;
    let mut settings = ResolvedGraphicsSettings::from_requested(&request, &Capabilities::unknown());
    settings.shadow_max_distance_m = 10.0;
    let anchor = TileAnchor::new(TileKey::new(4, 11, 1024, 1024).unwrap(), 1000.0).unwrap();
    let origin = DVec3::from_array(anchor.anchor_body_m) + DVec3::new(-1.0, 0.0, 2.0);
    let page = HeightPage::bake(&[0.0; 4], 2, 0.001).unwrap();
    let mut terrain = TerrainRenderer::with_capacity_and_lighting(
        &backend,
        &[TerrainPatch {
            anchor,
            page: &page,
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
    let direction = DVec3::new(1.0, 0.0, 1.0).normalize();
    let lights = SurfaceLighting {
        directions: [
            direction.as_vec3().extend(0.0).to_array(),
            [0.0; 4],
            [0.0; 4],
        ],
        colors_lux: [[std::f32::consts::PI; 4], [0.0; 4], [0.0; 4]],
        ..Default::default()
    };
    let mut source = source();
    source.primitives.truncate(1);
    let mut mesh = MeshRenderer::with_surface(
        &backend,
        &source,
        thessa_render_wgpu::mesh::SurfaceSettings {
            format: OffscreenTarget::FORMAT,
            capacity: 1,
            quality: Quality::Medium,
            reflections: false,
            shadows: terrain.shadows(),
            ray_tracing: None,
        },
    )
    .unwrap();
    let view = camera(origin);
    terrain.update_view(&backend, &view).unwrap();
    terrain.update_shadows(&backend, origin, &lights).unwrap();
    mesh.update(
        &backend,
        &view,
        &lights,
        &[MeshInstance {
            position_m: origin,
            rotation: DQuat::IDENTITY,
            scale: 1.0,
        }],
    )
    .unwrap();
    let image = cached_matches_redraw(&backend, &mut mesh, &mut terrain);
    let pixel = |x: usize| image[(64 * 129 + x) * 4];
    assert!(
        pixel(36) < 10,
        "resident terrain occludes receiver: {}",
        pixel(36)
    );
    assert!(
        pixel(64) > 100,
        "receiver outside terrain shadow remains lit: {}",
        pixel(64)
    );
    let changed_page = HeightPage::bake(&[0.01; 4], 2, 0.001).unwrap();
    terrain
        .update_pages(&backend, &[(0, &changed_page)])
        .unwrap();
    cached_matches_redraw(&backend, &mut mesh, &mut terrain);
    // Retire the page: stale generated vertex slots must no longer cast.
    terrain.publish_partition(&backend, &[]).unwrap();
    let retired = cached_matches_redraw(&backend, &mut mesh, &mut terrain);
    assert!(retired[(64 * 129 + 36) * 4] > 100);
}

#[test]
fn normal_maps_change_the_tangent_frame_and_ao_only_attenuates_indirect_light() {
    let Some(backend) = backend() else {
        return;
    };
    let mut source = source();
    source.primitives.truncate(1);
    for (vertex, uv) in source.primitives[0].vertices.iter_mut().zip([
        [0.0, 0.0],
        [1.0, 0.0],
        [1.0, 1.0],
        [0.0, 1.0],
    ]) {
        vertex.uv = uv;
    }
    let mut lights = SurfaceLighting {
        directions: [[0.0, 0.0, 1.0, 0.0], [0.0; 4], [0.0; 4]],
        colors_lux: [[std::f32::consts::PI; 4], [0.0; 4], [0.0; 4]],
        ..Default::default()
    };
    let render = |source: &MeshAsset, lights: &SurfaceLighting| {
        let mut mesh = MeshRenderer::with_surface(
            &backend,
            source,
            thessa_render_wgpu::mesh::SurfaceSettings {
                format: OffscreenTarget::FORMAT,
                capacity: 1,
                quality: Quality::High,
                reflections: false,
                shadows: None,
                ray_tracing: None,
            },
        )
        .unwrap();
        mesh.update(
            &backend,
            &camera(DVec3::ZERO),
            lights,
            &[MeshInstance {
                position_m: DVec3::ZERO,
                rotation: DQuat::IDENTITY,
                scale: 1.0,
            }],
        )
        .unwrap();
        draw(&backend, &mut mesh, None)
    };
    let direct = render(&source, &lights);
    source.textures.push(Texture {
        image: 0,
        wrap: [Wrap::Clamp; 2],
        linear: false,
        min_linear: false,
    });
    source.images.push(Image {
        width: 1,
        height: 1,
        rgba8: vec![255, 128, 128, 255],
    });
    source.materials[0].normal_texture = Some(0);
    let tilted = render(&source, &lights);
    let center = (64 * 129 + 64) * 4;
    assert!(
        tilted[center] < 30 && direct[center] > 180,
        "tangent-space +X is almost perpendicular to +Z light"
    );
    source.materials[0].normal_scale = 0.0;
    assert_eq!(direct, render(&source, &lights));
    source.materials[0].normal_texture = None;
    source.images[0].rgba8 = vec![0, 255, 255, 255];
    source.materials[0].occlusion_texture = Some(0);
    lights.ambient_lux = [std::f32::consts::PI; 4];
    assert_eq!(
        direct,
        render(&source, &lights),
        "AO=0 removes ambient, never direct illuminance"
    );
    source.materials[0].occlusion_strength = 0.0;
    assert!(render(&source, &lights)[center] > direct[center]);
}

/// Fixed-source diagnostic, not a production FPS gate. No async streaming,
/// readback, shader compilation or source publication is inside the timer.
#[test]
#[ignore = "requires a wgpu adapter; fixed 1024-patch/64-instance frame diagnostic"]
fn fixed_source_shadow_frame_cost() {
    let backend = Backend::bring_up().unwrap();
    let target = OffscreenTarget::new(&backend, 1280, 720).unwrap();
    let page = HeightPage::bake(&[0.0; 4], 2, 0.001).unwrap();
    let patches = (0..1024)
        .map(|i| TerrainPatch {
            anchor: TileAnchor::new(TileKey::new(4, 5, i % 32, i / 32).unwrap(), 1000.0).unwrap(),
            page: &page,
        })
        .collect::<Vec<_>>();
    let origin = DVec3::new(0.0, 0.0, 1010.0);
    let view = camera(origin);
    let mut source = source();
    source.primitives.truncate(1);
    let instances = (0..64)
        .map(|i| MeshInstance {
            position_m: origin + DVec3::new(f64::from(i % 8) - 3.5, f64::from(i / 8) - 3.5, -1.0),
            rotation: DQuat::IDENTITY,
            scale: 0.1,
        })
        .collect::<Vec<_>>();
    let lights = SurfaceLighting {
        directions: [[0.0, 0.0, 1.0, 0.0]; 3],
        colors_lux: [[std::f32::consts::PI; 4]; 3],
        ..Default::default()
    };
    let mut reference = None;
    for (trial, (enabled, force_redraw)) in [
        (false, false),
        (true, true),
        (true, false),
        (true, false),
        (true, true),
        (true, true),
        (true, false),
    ]
    .into_iter()
    .enumerate()
    {
        let mut request = RequestedGraphics::default();
        request.shadows.enabled = enabled;
        let settings = ResolvedGraphicsSettings::from_requested(&request, &Capabilities::unknown());
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
        terrain.update_view(&backend, &view).unwrap();
        terrain.update_lighting(&backend, &lights).unwrap();
        terrain.update_shadows(&backend, origin, &lights).unwrap();
        let mut mesh = MeshRenderer::with_surface(
            &backend,
            &source,
            thessa_render_wgpu::mesh::SurfaceSettings {
                format: OffscreenTarget::FORMAT,
                capacity: 64,
                quality: Quality::High,
                reflections: true,
                shadows: terrain.shadows(),
                ray_tracing: None,
            },
        )
        .unwrap();
        mesh.update(&backend, &view, &lights, &instances).unwrap();
        let mut timings = Vec::new();
        for i in 0..150 {
            if force_redraw {
                terrain.invalidate_shadows();
            }
            let start = std::time::Instant::now();
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            terrain.encode_shadows(&mut encoder, Some(&mesh));
            terrain.encode(&mut encoder, &target.color, &target.depth);
            mesh.encode(&mut encoder, &target.color, &target.depth);
            backend.queue.submit([encoder.finish()]);
            backend
                .device
                .poll(wgpu::PollType::wait_indefinitely())
                .unwrap();
            if i >= 30 {
                timings.push(start.elapsed().as_secs_f64() * 1000.0);
            }
        }
        timings.sort_by(f64::total_cmp);
        println!(
            "fixed_source adapter={} trial={trial} shadows={enabled} force_redraw={force_redraw} patches=1024 instances=64 resolution=1280x720 atlas_bytes={} active_layers={} last_rendered_passes={} measured_frames=120 median_ms={:.3} p95_ms={:.3}; CPU encode/submit + GPU completion, not isolated GPU timestamps or game FPS",
            backend.adapter_name,
            terrain.shadows().map_or(0, |s| s.storage_bytes()),
            terrain.shadows().map_or(0, |s| s.active_passes()),
            terrain.last_shadow_passes(),
            timings[60],
            timings[114]
        );
        if enabled {
            let pixels = target.read_rgba8(&backend).unwrap();
            if let Some(reference) = &reference {
                assert_eq!(
                    &pixels, reference,
                    "forced/cached shadow depth must render identical frames"
                );
            } else {
                reference = Some(pixels);
            }
            assert_eq!(
                terrain.last_shadow_passes(),
                if force_redraw { 12 } else { 0 }
            );
        }
    }
}
