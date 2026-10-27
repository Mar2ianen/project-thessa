// SPDX-License-Identifier: GPL-3.0-or-later
//! Single-specimen anatomy/material inspection using the real native producer.
//! Neutral QA lighting/support, not a coastal/reference acceptance scene.
use super::*;
use thessa_render_wgpu::{
    Backend,
    mesh::{MeshRenderer, SurfaceSettings},
    terrain::{OffscreenTarget, TerrainCapacity, TerrainPatch, TerrainRenderer},
    view::ViewParams,
};

#[test]
#[ignore = "software Vulkan: actual plant specimens and LODs; fresh output directory required"]
fn capture_actual_plant_specimens() {
    assert!(
        !std::path::Path::new("/dev/dri").exists(),
        "isolate device files with the software-only Vulkan launcher before bring-up"
    );
    let out = std::path::PathBuf::from(
        std::env::var_os("THESSA_PLANT_PREVIEW_DIR")
            .expect("set THESSA_PLANT_PREVIEW_DIR to a NEW directory"),
    );
    std::fs::create_dir(&out).unwrap();
    let mut request = thessa_graphics::RequestedGraphics::default();
    request.renderer.ray_tracing = thessa_graphics::RayTracingRequest::Off;
    request.renderer.surface_quality = thessa_graphics::Quality::High;
    request.renderer.specular_reflections = false;
    request.renderer.msaa_samples = 4;
    request.shadows.cascades = 1;
    request.shadows.map_size = 2048;
    request.shadows.max_distance_m = 50.0;
    request.renderer.shadow_memory_mib = 64;
    request.ocean.enabled = false;
    let backend = Backend::bring_up_for_graphics(&request).unwrap();
    assert_eq!(
        backend.adapter.get_info().device_type,
        wgpu::DeviceType::Cpu,
        "this diagnostic is software-only; use the isolated CPU-ICD launcher"
    );
    let settings = backend.resolve_graphics(&request);
    let display = thessa_render_wgpu::display::DisplayTransform::new(
        &backend,
        &settings,
        OffscreenTarget::FORMAT,
        960,
        800,
    )
    .unwrap();
    let msaa = thessa_render_wgpu::msaa::SceneMsaa::new(
        &backend,
        &settings,
        display.scene_format(),
        [960, 800],
    )
    .unwrap();
    let page = thessa_rcbt_core::HeightPage::bake(&[0.0; 4], 2, 0.001).unwrap();
    let patch = TerrainPatch {
        anchor: thessa_rcbt_core::tile_anchor::TileAnchor::new(
            thessa_rcbt_core::tile_anchor::TileKey::new(0, 18, 131072, 131072).unwrap(),
            1_000_000.0,
        )
        .unwrap(),
        page: &page,
    };
    let mut terrain = TerrainRenderer::with_capacity_and_lighting(
        &backend,
        &[patch],
        display.scene_format(),
        TerrainCapacity {
            patches: 1,
            height_grid_size: 2,
            material_layers: 1,
        },
        Some(&settings),
    )
    .unwrap();
    let sun = DVec3::new(0.35, 0.75, 0.56).normalize();
    let lights = thessa_rcbt_core::lighting::SurfaceLighting {
        ambient_lux: [2.8, 3.4, 4.0, 0.0],
        directions: [
            [sun.x as f32, sun.y as f32, sun.z as f32, 0.0],
            [0.0; 4],
            [0.0; 4],
        ],
        colors_lux: [[7.6, 8.0, 7.2, 0.0], [0.0; 4], [0.0; 4]],
    };
    let mut trace =
        crate::gpu_trace::GpuTrace::create(&out.join("stages.toml"), 1_000_000).unwrap();
    trace
        .event(
            0,
            "fixture",
            "adapter",
            &format!("{:?}", backend.adapter.get_info()),
        )
        .unwrap();
    let mut records = Vec::new();
    let mut frame = 0;
    for kind in [
        thessa_worldgen_rocky::scatter::ScatterKind::FernClump,
        thessa_worldgen_rocky::scatter::ScatterKind::LycopsidTree,
    ] {
        let tree = kind == thessa_worldgen_rocky::scatter::ScatterKind::LycopsidTree;
        for form in 0..3 {
            let id = (0..1000)
                .map(thessa_worldgen_rocky::scatter::ScatterId)
                .find(|&id| thessa_worldgen_rocky::offline::plant_form(id, kind) == form)
                .unwrap();
            for reduced in [false, true] {
                let item = ScatterItem {
                    id,
                    kind,
                    direction: DVec3::Y.to_array(),
                    ground_height_m: 0.0,
                    scale: 1.0,
                    yaw_rad: 0.0,
                };
                let mut p = convert_with_detail(
                    vec![item],
                    3_200_000.0,
                    reduced.then_some((DVec3::X * 3_200_000.0, 48.0, 0.0)),
                )
                .unwrap();
                assert_eq!(p.instances.len(), 1);
                let material = p.asset.materials.len();
                p.asset.materials.push(Material {
                    base_color: [0.08, 0.065, 0.045, 1.0],
                    metallic: 0.0,
                    roughness: 0.8,
                    double_sided: true,
                    ..Default::default()
                });
                p.asset.primitives.push(Primitive {
                    ambient_visibility: vec![],
                    vertices: [
                        [-15.0, -0.004, -15.0],
                        [-15.0, -0.004, 15.0],
                        [15.0, -0.004, 15.0],
                        [15.0, -0.004, -15.0],
                    ]
                    .map(|position| Vertex {
                        position,
                        normal: [0.0, 1.0, 0.0],
                        uv: [0.0; 2],
                    })
                    .to_vec(),
                    indices: vec![0, 1, 2, 0, 2, 3],
                    material,
                });
                p.ranges.push(1..2);
                let instances = [MeshInstance {
                    position_m: DVec3::ZERO,
                    rotation: DQuat::IDENTITY,
                    scale: 1.0,
                }; 2];
                let mut mesh = MeshRenderer::with_surface(
                    &backend,
                    &p.asset,
                    SurfaceSettings {
                        format: display.scene_format(),
                        capacity: 2,
                        quality: settings.surface_quality,
                        reflections: false,
                        shadows: terrain.shadows(),
                        ray_tracing: Some(&settings),
                    },
                )
                .unwrap();
                let (eye, target) = if tree {
                    (DVec3::new(12.0, 8.5, 15.0), DVec3::new(0.0, 5.0, 0.0))
                } else if form == 1 {
                    (DVec3::new(1.3, 0.9, 1.9), DVec3::new(0.0, 0.35, 0.0))
                } else {
                    (DVec3::new(2.5, 1.8, 3.5), DVec3::new(0.0, 0.75, 0.0))
                };
                let view = ViewParams {
                    world_position_m: eye,
                    world_from_view: glam::dcamera::rh::view::look_at_mat4(eye, target, DVec3::Y)
                        .inverse(),
                    clip_from_view: glam::dcamera::rh::proj::directx::perspective(
                        0.80, 1.2, 0.1, 100.0,
                    ),
                    exposure: 1.0,
                };
                mesh.update_selected(&backend, &view, &lights, &instances, Some(&p.ranges))
                    .unwrap();
                terrain.update_view(&backend, &view).unwrap();
                terrain.update_lighting(&backend, &lights).unwrap();
                terrain.update_shadows(&backend, eye, &lights).unwrap();
                terrain.invalidate_shadows();
                let name = format!(
                    "{}-form{form}-{}",
                    if tree { "tree" } else { "fern" },
                    if reduced { "reduced" } else { "full" }
                );
                trace.event(frame,&name,"view",&format!("eye={eye:?} target={target:?}; neutral QA light, real producer/maps/AO, plain support plane")).unwrap();
                trace.workload(frame, Some(mesh.draw_stats()), 1).unwrap();
                let image = OffscreenTarget::new(&backend, 960, 800).unwrap();
                let mut encoder = backend.device.create_command_encoder(&Default::default());
                terrain.encode_shadows(&mut encoder, Some(&mesh));
                trace
                    .checkpoint(
                        &backend,
                        &mut encoder,
                        frame,
                        "specimen_shadow",
                        terrain.last_shadow_draws().mesh_submitted_triangles,
                    )
                    .unwrap();
                mesh.encode_rt(&mut encoder);
                {
                    let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                        color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                            view: msaa
                                .as_ref()
                                .map_or(display.scene_color(&image.color), |m| &m.color),
                            depth_slice: None,
                            resolve_target: msaa
                                .as_ref()
                                .map(|_| display.scene_color(&image.color)),
                            ops: wgpu::Operations {
                                load: wgpu::LoadOp::Clear(wgpu::Color {
                                    r: 0.22,
                                    g: 0.26,
                                    b: 0.27,
                                    a: 1.0,
                                }),
                                store: wgpu::StoreOp::Store,
                            },
                        })],
                        depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                            view: msaa.as_ref().map_or(&image.depth, |m| m.depth()),
                            depth_ops: Some(wgpu::Operations {
                                load: wgpu::LoadOp::Clear(1.0),
                                store: wgpu::StoreOp::Store,
                            }),
                            stencil_ops: None,
                        }),
                        ..Default::default()
                    });
                    mesh.draw_into_pass(&mut pass);
                }
                display.encode(&mut encoder, &image.color);
                trace
                    .checkpoint(
                        &backend,
                        &mut encoder,
                        frame,
                        "specimen_primary",
                        mesh.draw_stats().triangles,
                    )
                    .unwrap();
                let rgba = image.read_rgba8(&backend).unwrap();
                crate::write_rgba_png(&out.join(format!("{name}.png")), 960, 800, &rgba).unwrap();
                records.push(format!(
                    "{name}: {:?}; shadow={:?}",
                    mesh.draw_stats(),
                    terrain.last_shadow_draws()
                ));
                trace
                    .event(frame, &name, "captured", records.last().unwrap())
                    .unwrap();
                frame += 1;
            }
        }
    }
    std::fs::write(out.join("specimens.txt"), records.join("\n")).unwrap();
    assert_eq!(frame, 12);
}
