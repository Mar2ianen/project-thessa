use glam::{DMat4, DQuat, DVec3};
use thessa_graphics::Quality;
use thessa_rcbt_core::lighting::SurfaceLighting;
use thessa_render_core::{ViewParams, mesh::*};
use thessa_render_wgpu::{
    Backend,
    mesh::{MeshRenderer, SurfaceSettings},
    terrain::OffscreenTarget,
};

fn quad(z: f32, material: usize) -> Primitive {
    Primitive {
        ambient_visibility: vec![],
        material,
        vertices: [
            ([-1., -1., z], [0., 1.]),
            ([1., -1., z], [1., 1.]),
            ([1., 1., z], [1., 0.]),
            ([-1., 1., z], [0., 0.]),
        ]
        .map(|(position, uv)| Vertex {
            position,
            normal: [0., 0., 1.],
            uv,
        })
        .to_vec(),
        indices: vec![0, 1, 2, 0, 2, 3],
    }
}

#[test]
fn masked_coverage_preserves_holes_depth_factors_and_opaque_alpha_semantics() {
    let backend = Backend::bring_up().unwrap();
    let mut asset = MeshAsset {
        // Background is deliberately drawn last: transparent texels must not
        // write foreground depth and prevent the blue surface from appearing.
        primitives: vec![quad(-0.25, 0), quad(-0.75, 1)],
        materials: vec![
            Material {
                base_texture: Some(0),
                alpha_mode: AlphaMode::Mask,
                alpha_cutoff: 0.5,
                base_color: [1., 1., 1., 0.5],
                metallic: 0.,
                double_sided: true,
                ..Default::default()
            },
            Material {
                base_color: [0., 0., 1., 0.],
                metallic: 0.,
                double_sided: false,
                ..Default::default()
            },
        ],
        textures: vec![Texture {
            image: 0,
            wrap: [Wrap::Clamp; 2],
            linear: false,
            min_linear: false,
        }],
        images: vec![Image {
            width: 2,
            height: 1,
            rgba8: vec![255, 0, 0, 0, 255, 0, 0, 255],
        }],
    };
    let view = ViewParams {
        world_from_view: DMat4::IDENTITY,
        world_position_m: DVec3::ZERO,
        clip_from_view: DMat4::from_scale(DVec3::new(1., 1., -1.)),
        exposure: 4.,
    };
    let light = SurfaceLighting {
        ambient_lux: [std::f32::consts::PI; 4],
        ..Default::default()
    };
    asset.materials.push(Material {
        double_sided: false,
        ..asset.materials[0].clone()
    });
    for (quality, double_sided) in [
        (Quality::Low, true),
        (Quality::High, true),
        (Quality::High, false),
    ] {
        let masked_material = if double_sided { 0 } else { 2 };
        asset.primitives[0].material = masked_material;
        for (cutoff, factor, expected) in [
            (0.5, 0.5, [[0, 0, 255], [255, 0, 0]]),
            (0.5, 0.49, [[0, 0, 255], [0, 0, 255]]),
            (0., 0., [[255, 0, 0], [255, 0, 0]]),
            (1.5, 1., [[0, 0, 255], [0, 0, 255]]),
        ] {
            asset.materials[masked_material].alpha_cutoff = cutoff;
            asset.materials[masked_material].base_color[3] = factor;
            let mut renderer = MeshRenderer::with_surface(
                &backend,
                &asset,
                SurfaceSettings {
                    format: OffscreenTarget::FORMAT,
                    capacity: 1,
                    quality,
                    reflections: false,
                    shadows: None,
                    ray_tracing: None,
                },
            )
            .unwrap();
            renderer
                .update(
                    &backend,
                    &view,
                    &light,
                    &[MeshInstance {
                        position_m: DVec3::ZERO,
                        rotation: DQuat::IDENTITY,
                        scale: 1.,
                    }],
                )
                .unwrap();
            let target = OffscreenTarget::new(&backend, 32, 16).unwrap();
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            {
                let _clear = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
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
                            load: wgpu::LoadOp::Clear(1.),
                            store: wgpu::StoreOp::Store,
                        }),
                        stencil_ops: None,
                    }),
                    ..Default::default()
                });
            }
            renderer.encode(&mut encoder, &target.color, &target.depth);
            backend.queue.submit([encoder.finish()]);
            let pixels = target.read_rgba8(&backend).unwrap();
            for (x, color) in [8, 24].into_iter().zip(expected) {
                let index = (8 * 32 + x) * 4;
                for (actual, expected) in pixels[index..index + 3].iter().zip(color) {
                    assert!(
                        actual.abs_diff(expected) <= 1,
                        "quality={quality:?} cutoff={cutoff} factor={factor}: {:?} vs {color:?}",
                        &pixels[index..index + 4]
                    );
                }
            }
        }
    }
    let settings = thessa_graphics::ResolvedGraphicsSettings {
        shadow_enabled: true,
        native_shadow_map_size: 64,
        ..Default::default()
    };
    let atlas = thessa_render_wgpu::shadows::ShadowAtlas::new(&backend, &settings).unwrap();
    let result = MeshRenderer::with_surface(
        &backend,
        &asset,
        SurfaceSettings {
            format: OffscreenTarget::FORMAT,
            capacity: 1,
            quality: Quality::High,
            reflections: false,
            shadows: atlas.as_ref(),
            ray_tracing: None,
        },
    );
    assert!(result.is_ok(), "MASK uses texture-aware shadow coverage");
    let rt = thessa_graphics::ResolvedGraphicsSettings {
        ray_tracing: thessa_graphics::ResolvedRayTracing::Local,
        rt_vehicles: true,
        ..Default::default()
    };
    assert!(
        matches!(thessa_render_wgpu::ray_scene::plan(&backend,&asset,Some(&rt),1,Quality::High,true),
        thessa_render_wgpu::ray_scene::Enable::Unavailable(reason) if reason.contains("device negotiated without ray query"))
    );
}

#[test]
#[ignore = "Vulkan: alpha-aware depth versus actual cut geometry, all layers and direct/indirect paths"]
fn masked_shadows_match_real_cut_geometry_and_clear_removed_sources() {
    use thessa_render_wgpu::terrain::{TerrainCapacity, TerrainPatch, TerrainRenderer};
    let backend = Backend::bring_up().unwrap();
    eprintln!("masked shadow adapter={:?}", backend.adapter.get_info());
    let settings = thessa_graphics::ResolvedGraphicsSettings {
        shadow_enabled: true,
        native_shadow_map_size: 128,
        shadow_cascades: 1,
        shadow_max_distance_m: 5.0,
        shadow_first_cascade_distance_m: None,
        shadow_receiver_culling: false,
        shadow_raster_sample_guard_texels: None,
        shadow_mesh_lod_error_texels: 0.0,
        ..Default::default()
    };
    let page = thessa_rcbt_core::HeightPage::bake(&[0.; 4], 2, 0.001).unwrap();
    let mut terrain = TerrainRenderer::with_capacity_and_lighting(
        &backend,
        &[TerrainPatch {
            anchor: thessa_rcbt_core::tile_anchor::TileAnchor::new(
                thessa_rcbt_core::tile_anchor::TileKey::new(0, 0, 0, 0).unwrap(),
                1000.,
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
        Some(&settings),
    )
    .unwrap();
    terrain.publish_partition(&backend, &[]).unwrap();
    let view = ViewParams {
        world_from_view: DMat4::IDENTITY,
        world_position_m: DVec3::ZERO,
        clip_from_view: DMat4::from_scale(DVec3::new(1., 1., -0.1)),
        exposure: 1.,
    };
    let lights = SurfaceLighting {
        directions: [[0., 0., 1., 0.]; 3],
        colors_lux: [[1.; 4]; 3],
        ambient_lux: [0.; 4],
    };
    terrain.update_view(&backend, &view).unwrap();
    terrain
        .update_shadows(&backend, DVec3::ZERO, &lights)
        .unwrap();
    let instance = MeshInstance {
        position_m: DVec3::ZERO,
        rotation: DQuat::IDENTITY,
        scale: 1.,
    };
    let build =
        |atlas: Option<&thessa_render_wgpu::shadows::ShadowAtlas>, masked: bool, case: usize| {
            let mut p = quad(-4., 0);
            if !masked && matches!(case, 0 | 3) {
                for v in &mut p.vertices {
                    v.position[0] = v.position[0].max(0.);
                }
            }
            if !masked && matches!(case, 1 | 4) {
                for v in &mut p.vertices {
                    v.position[0] += 1000.;
                }
            }
            let asset = MeshAsset {
                primitives: vec![p],
                materials: vec![Material {
                    base_color: [
                        1.,
                        1.,
                        1.,
                        if case == 1 {
                            0.4
                        } else if case == 3 {
                            0.5
                        } else {
                            1.
                        },
                    ],
                    base_texture: Some(0),
                    alpha_mode: if masked {
                        AlphaMode::Mask
                    } else {
                        AlphaMode::Opaque
                    },
                    alpha_cutoff: if case == 2 {
                        0.
                    } else if case == 4 {
                        1.1
                    } else {
                        0.5
                    },
                    double_sided: true,
                    metallic: 0.,
                    ..Default::default()
                }],
                textures: vec![Texture {
                    image: 0,
                    wrap: [Wrap::Clamp; 2],
                    linear: false,
                    min_linear: false,
                }],
                images: vec![Image {
                    width: 2,
                    height: 1,
                    rgba8: vec![255, 255, 255, 0, 255, 255, 255, 255],
                }],
            };
            // Empty index buffers are not a geometry source; remove the primitive.
            let asset = if asset.primitives[0].indices.is_empty() {
                MeshAsset {
                    primitives: vec![],
                    ..asset
                }
            } else {
                asset
            };
            let mut mesh = MeshRenderer::with_surface(
                &backend,
                &asset,
                SurfaceSettings {
                    format: OffscreenTarget::FORMAT,
                    capacity: 1,
                    quality: Quality::High,
                    reflections: false,
                    shadows: atlas,
                    ray_tracing: None,
                },
            )
            .unwrap();
            mesh.update(&backend, &view, &lights, &[instance]).unwrap();
            mesh
        };
    let capture = |terrain: &mut TerrainRenderer, mesh: &MeshRenderer, direct: bool| {
        terrain.invalidate_shadows();
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        if direct {
            terrain.encode_shadows_reference(&mut encoder, Some(mesh));
        } else {
            terrain.encode_shadows(&mut encoder, Some(mesh));
        }
        backend.queue.submit([encoder.finish()]);
        terrain.shadows().unwrap().read_depth(&backend).unwrap()
    };
    for case in 0..5 {
        let mask = build(terrain.shadows(), true, case);
        let actual = capture(&mut terrain, &mask, false);
        let expected_mesh = build(terrain.shadows(), false, case);
        let expected = capture(&mut terrain, &expected_mesh, false);
        assert_eq!(actual, expected, "cut geometry depth case={case}");
        assert_eq!(
            actual,
            capture(&mut terrain, &mask, true),
            "direct/indirect case={case}"
        );
        if case == 0 {
            assert!(
                actual
                    .as_chunks::<4>()
                    .0
                    .iter()
                    .any(|b| f32::from_le_bytes(*b) < 1.)
            );
        }
    }
    eprintln!(
        "MASK shadow: five cutoff/factor cases, three layers, exact geometric and direct/indirect agreement"
    );
}

#[test]
#[ignore = "Vulkan: filtered alpha retains fractional MSAA coverage while single-sample MASK remains exact"]
fn filtered_foliage_alpha_uses_msaa_coverage_without_blending() {
    use thessa_render_wgpu::msaa::SceneMsaa;
    let mut request = thessa_graphics::RequestedGraphics::default();
    request.renderer.ray_tracing = thessa_graphics::RayTracingRequest::Off;
    request.shadows.enabled = false;
    request.temporal.taa = false;
    request.temporal.motion_vectors = false;
    let backend = Backend::bring_up_for_graphics(&request).unwrap();
    let view = ViewParams {
        world_from_view: DMat4::IDENTITY,
        clip_from_view: DMat4::IDENTITY,
        world_position_m: DVec3::ZERO,
        exposure: 1.,
    };
    let light = SurfaceLighting {
        ambient_lux: [std::f32::consts::PI; 4],
        ..Default::default()
    };
    for samples in [1, 4] {
        request.renderer.msaa_samples = samples;
        let settings = backend.resolve_graphics(&request);
        assert_eq!(settings.raster_msaa_samples, samples);
        let msaa = SceneMsaa::new(&backend, &settings, OffscreenTarget::FORMAT, [32, 16]).unwrap();
        let mut codes = Vec::new();
        for alpha in [0, 64, 128, 192, 255] {
            let asset = MeshAsset {
                primitives: vec![quad(0.4, 0)],
                materials: vec![Material {
                    alpha_mode: AlphaMode::Mask,
                    alpha_cutoff: 0.5,
                    base_texture: Some(0),
                    metallic: 0.,
                    double_sided: true,
                    ..Default::default()
                }],
                textures: vec![Texture {
                    image: 0,
                    wrap: [Wrap::Clamp; 2],
                    linear: true,
                    min_linear: true,
                }],
                images: vec![Image {
                    width: 1,
                    height: 1,
                    rgba8: vec![255, 255, 255, alpha],
                }],
            };
            let mut mesh = MeshRenderer::with_surface(
                &backend,
                &asset,
                SurfaceSettings {
                    format: OffscreenTarget::FORMAT,
                    capacity: 1,
                    quality: Quality::Low,
                    reflections: false,
                    shadows: None,
                    ray_tracing: Some(&settings),
                },
            )
            .unwrap();
            mesh.update(
                &backend,
                &view,
                &light,
                &[MeshInstance {
                    position_m: DVec3::ZERO,
                    rotation: DQuat::IDENTITY,
                    scale: 1.,
                }],
            )
            .unwrap();
            let target = OffscreenTarget::new(&backend, 32, 16).unwrap();
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            {
                let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                    color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                        view: msaa.as_ref().map_or(&target.color, |m| &m.color),
                        depth_slice: None,
                        resolve_target: msaa.as_ref().map(|_| &target.color),
                        ops: wgpu::Operations {
                            load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                            store: wgpu::StoreOp::Store,
                        },
                    })],
                    depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                        view: msaa.as_ref().map_or(&target.depth, |m| m.depth()),
                        depth_ops: Some(wgpu::Operations {
                            load: wgpu::LoadOp::Clear(1.),
                            store: wgpu::StoreOp::Store,
                        }),
                        stencil_ops: None,
                    }),
                    ..Default::default()
                });
                mesh.draw_into_pass(&mut pass);
            }
            backend.queue.submit([encoder.finish()]);
            let rgba = target.read_rgba8(&backend).unwrap();
            codes.push(rgba[(8 * 32 + 16) * 4]);
        }
        if samples == 1 {
            assert_eq!(codes, [0, 0, 255, 255, 255]);
        } else {
            assert_eq!(codes[0], 0);
            assert_eq!(codes[4], 255);
            assert!(
                codes.windows(2).all(|w| w[0] < w[1]),
                "fractional alpha must survive minification: {codes:?}"
            );
            assert!(
                (180..=195).contains(&codes[2]),
                "half area in linear light: {codes:?}"
            );
        }
        eprintln!("MASK samples={samples} alpha=[0,64,128,192,255] resolved_srgb={codes:?}");
    }
}

#[test]
#[ignore = "Vulkan: a two-sided mapped leaf must reverse its complete shading normal"]
fn two_sided_normal_map_reverses_before_lighting_on_back_faces() {
    let backend = Backend::bring_up().unwrap();
    let source = MeshAsset {
        primitives: vec![quad(0., 0)],
        materials: vec![Material {
            base_color: [0.25, 0.55, 0.1, 1.],
            normal_texture: Some(0),
            metallic: 0.,
            double_sided: true,
            ..Default::default()
        }],
        textures: vec![Texture {
            image: 0,
            wrap: [Wrap::Clamp; 2],
            linear: false,
            min_linear: false,
        }],
        images: vec![Image {
            width: 1,
            height: 1,
            rgba8: vec![230, 152, 192, 255],
        }],
    };
    let mut colors = Vec::new();
    for side in [1.0_f64, -1.0] {
        let eye = DVec3::Z * (side * 3.);
        let view = ViewParams {
            world_from_view: glam::dcamera::rh::view::look_at_mat4(eye, DVec3::ZERO, DVec3::Y)
                .inverse(),
            clip_from_view: glam::dcamera::rh::proj::directx::perspective(0.8, 1., 0.1, 10.),
            world_position_m: eye,
            exposure: 0.5,
        };
        let direction = (DVec3::X + DVec3::Z * 0.5).normalize() * side;
        let light = SurfaceLighting {
            directions: [
                [
                    direction.x as f32,
                    direction.y as f32,
                    direction.z as f32,
                    0.,
                ],
                [0.; 4],
                [0.; 4],
            ],
            colors_lux: [[3.; 4], [0.; 4], [0.; 4]],
            ambient_lux: [0.2; 4],
        };
        let mut mesh = MeshRenderer::with_surface(
            &backend,
            &source,
            SurfaceSettings {
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
            &view,
            &light,
            &[MeshInstance {
                position_m: DVec3::ZERO,
                rotation: DQuat::IDENTITY,
                scale: 1.,
            }],
        )
        .unwrap();
        let target = OffscreenTarget::new(&backend, 33, 33).unwrap();
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
                        load: wgpu::LoadOp::Clear(1.),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                ..Default::default()
            });
        }
        mesh.encode(&mut encoder, &target.color, &target.depth);
        backend.queue.submit([encoder.finish()]);
        let bytes = target.read_rgba8(&backend).unwrap();
        colors.push(bytes[(16 * 33 + 16) * 4..(16 * 33 + 16) * 4 + 3].to_vec());
    }
    eprintln!("normal-map leaf front/back RGB={colors:?}");
    assert!(colors[0][1] > 80, "fixture must see the mapped light");
    for c in 0..3 {
        assert!(
            colors[0][c].abs_diff(colors[1][c]) <= 1,
            "complete mapped normal reversal: {colors:?}"
        );
    }
}
