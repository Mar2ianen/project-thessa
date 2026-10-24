//! Hardware oracle for the static camera-reprojection motion-vector pass.
//! The fixture is a depth-only screen gradient, so the expected vectors are
//! computed independently on the CPU from the same matrices and depth.

use glam::{DMat4, DVec3, Vec3};
use thessa_render_core::temporal::world_from_depth;
use thessa_render_wgpu::{
    Backend, BackendError,
    motion::{BYTES_PER_PIXEL, FORMAT, MotionParams, MotionVectorPass},
    view::{ViewParams, ViewUniformBlock},
};
use wgpu::util::DeviceExt;

const SIZE: u32 = 64;

fn backend() -> Option<Backend> {
    match Backend::bring_up() {
        Ok(backend) => Some(backend),
        Err(BackendError::NoAdapter) => {
            eprintln!("SKIP: no adapter");
            None
        }
        Err(error) => panic!("{error}"),
    }
}

fn depth_pipeline(backend: &Backend) -> wgpu::RenderPipeline {
    let module = backend
        .device
        .create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("motion-depth-fixture"),
            source: wgpu::ShaderSource::Wgsl(
                r#"
@vertex fn vertex(@builtin(vertex_index) i: u32) -> @builtin(position) vec4<f32> {
    let p = array<vec2<f32>, 3>(vec2(-1.0, -1.0), vec2(3.0, -1.0), vec2(-1.0, 3.0));
    return vec4<f32>(p[i], 0.0, 1.0);
}
@fragment fn fragment(@builtin(position) p: vec4<f32>) -> @builtin(frag_depth) f32 {
    if (p.x < 40.0) { return 1.0; }
    return 0.25 + 0.5 * (p.x / 64.0);
}
"#
                .into(),
            ),
        });
    backend
        .device
        .create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("motion-depth-fixture"),
            layout: None,
            vertex: wgpu::VertexState {
                module: &module,
                entry_point: Some("vertex"),
                compilation_options: Default::default(),
                buffers: &[],
            },
            fragment: Some(wgpu::FragmentState {
                module: &module,
                entry_point: Some("fragment"),
                compilation_options: Default::default(),
                targets: &[],
            }),
            primitive: Default::default(),
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: Some(true),
                depth_compare: Some(wgpu::CompareFunction::Always),
                stencil: Default::default(),
                bias: Default::default(),
            }),
            multisample: Default::default(),
            multiview_mask: None,
            cache: None,
        })
}

#[test]
fn static_motion_matches_cpu_reprojection_and_marks_sky_invalid() {
    let Some(backend) = backend() else {
        return;
    };
    let depth = backend.device.create_texture(&wgpu::TextureDescriptor {
        label: Some("motion-depth-fixture"),
        size: wgpu::Extent3d {
            width: SIZE,
            height: SIZE,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Depth32Float,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
        view_formats: &[],
    });
    let depth_view = depth.create_view(&Default::default());
    let output = backend.device.create_texture(&wgpu::TextureDescriptor {
        label: Some("motion-vectors"),
        size: wgpu::Extent3d {
            width: SIZE,
            height: SIZE,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: FORMAT,
        usage: wgpu::TextureUsages::STORAGE_BINDING | wgpu::TextureUsages::COPY_SRC,
        view_formats: &[],
    });
    let output_view = output.create_view(&Default::default());
    let pixels = (0..SIZE * SIZE)
        .flat_map(|i| [0, 0, 0, if i % SIZE >= 56 { 0 } else { 255 }])
        .collect::<Vec<_>>();
    let mask = backend.device.create_texture_with_data(
        &backend.queue,
        &wgpu::TextureDescriptor {
            label: Some("static-dynamic-mask-fixture"),
            size: wgpu::Extent3d {
                width: SIZE,
                height: SIZE,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8Unorm,
            usage: wgpu::TextureUsages::TEXTURE_BINDING,
            view_formats: &[],
        },
        wgpu::util::TextureDataOrder::LayerMajor,
        &pixels,
    );
    let mask_view = mask.create_view(&Default::default());
    let pipeline = depth_pipeline(&backend);
    let pass = MotionVectorPass::new(&backend);

    let projection = glam::dcamera::rh::proj::directx::perspective(1.0, 1.5, 0.1, 1_000.0);
    let make_view = |x: f64| ViewParams {
        world_from_view: DMat4::from_translation(DVec3::new(x, 0.0, 10.0)),
        clip_from_view: projection,
        world_position_m: DVec3::new(x, 0.0, 10.0),
        exposure: 1.0,
    };
    let current = ViewUniformBlock::new(&make_view(0.0)).unwrap();
    let previous = ViewUniformBlock::new(&make_view(0.1)).unwrap();
    let params = MotionParams {
        current_world_from_clip: current.world_from_clip,
        previous_clip_from_world: previous.clip_from_world,
        camera_offset: Vec3::new(-0.1, 0.0, 0.0),
        extent: [SIZE, SIZE],
        previous_valid: true,
        current_jitter: thessa_render_core::temporal::JitterOffset::ZERO,
    };

    let mut encoder = backend.device.create_command_encoder(&Default::default());
    {
        let mut render = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("motion-depth-fixture"),
            color_attachments: &[],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: &depth_view,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(1.0),
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            }),
            ..Default::default()
        });
        render.set_pipeline(&pipeline);
        render.draw(0..3, 0..1);
    }
    pass.encode(
        &backend,
        &mut encoder,
        &depth_view,
        &mask_view,
        &output_view,
        params,
    )
    .unwrap();

    let stride = SIZE * BYTES_PER_PIXEL;
    let readback = backend.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("motion-readback"),
        size: u64::from(stride * SIZE),
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    encoder.copy_texture_to_buffer(
        wgpu::TexelCopyTextureInfo {
            texture: &output,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        wgpu::TexelCopyBufferInfo {
            buffer: &readback,
            layout: wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(stride),
                rows_per_image: Some(SIZE),
            },
        },
        wgpu::Extent3d {
            width: SIZE,
            height: SIZE,
            depth_or_array_layers: 1,
        },
    );
    backend.queue.submit([encoder.finish()]);
    let (tx, rx) = std::sync::mpsc::channel();
    readback.map_async(wgpu::MapMode::Read, .., move |result| {
        tx.send(result).unwrap();
    });
    backend
        .device
        .poll(wgpu::PollType::wait_indefinitely())
        .unwrap();
    rx.recv().unwrap().unwrap();
    let data = readback.get_mapped_range(..).unwrap();

    let decode = |x: u32, y: u32| {
        let at = (y * stride + x * BYTES_PER_PIXEL) as usize;
        let word = |offset: usize| {
            half_to_f32(u16::from_le_bytes([
                data[at + offset],
                data[at + offset + 1],
            ]))
        };
        [word(0), word(2), word(4), word(6)]
    };

    // Sky/far-depth samples carry no finite terrain history.
    let invalid = decode(10, 10);
    assert_eq!(&invalid[..2], &[0.0, 0.0]);
    assert!(invalid[3] < 0.5);
    assert_eq!(
        decode(60, 50),
        [0.0; 4],
        "dynamic samples must be invalid even at finite depth"
    );

    let (px, py) = (50, 50);
    let gpu = decode(px, py);
    assert!(gpu[3] > 0.5);
    let uv = [f64::from(px) + 0.5, f64::from(py) + 0.5].map(|v| v / f64::from(SIZE));
    let depth_value = 0.25 + 0.5 * (f64::from(px) + 0.5) / f64::from(SIZE);
    let world = world_from_depth(current.world_from_clip.as_dmat4(), uv, depth_value).unwrap();
    let world_previous = world + DVec3::new(-0.1, 0.0, 0.0);
    let clip = previous.clip_from_world.as_dmat4() * world_previous.extend(1.0);
    let previous_uv = [clip.x / clip.w * 0.5 + 0.5, 0.5 - clip.y / clip.w * 0.5];
    let expected = [previous_uv[0] - uv[0], previous_uv[1] - uv[1]];
    assert!(
        (f64::from(gpu[0]) - expected[0]).abs() < 3e-4,
        "gpu={gpu:?} expected={expected:?}"
    );
    assert!(
        (f64::from(gpu[1]) - expected[1]).abs() < 3e-4,
        "gpu={gpu:?} expected={expected:?}"
    );

    drop(data);
    readback.unmap();
}

/// Software conversion for the test readback; the shader writes binary16.
fn half_to_f32(bits: u16) -> f32 {
    let sign = u32::from(bits & 0x8000) << 16;
    let exponent = u32::from((bits >> 10) & 0x1f);
    let mantissa = u32::from(bits & 0x03ff);
    let value = if exponent == 0 {
        if mantissa == 0 {
            sign
        } else {
            let mut mantissa = mantissa;
            let mut exponent = 127 - 15 + 1;
            while mantissa & 0x0400 == 0 {
                mantissa <<= 1;
                exponent -= 1;
            }
            sign | (exponent << 23) | ((mantissa & 0x03ff) << 13)
        }
    } else if exponent == 0x1f {
        sign | 0x7f80_0000 | (mantissa << 13)
    } else {
        sign | ((exponent + 127 - 15) << 23) | (mantissa << 13)
    };
    f32::from_bits(value)
}

fn read_texture(backend: &Backend, texture: &wgpu::Texture, bytes_per_pixel: u32) -> Vec<u8> {
    let stride = (texture.width() * bytes_per_pixel).div_ceil(256) * 256;
    let buffer = backend.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("scene-motion-oracle-readback"),
        size: u64::from(stride * texture.height()),
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    encoder.copy_texture_to_buffer(
        texture.as_image_copy(),
        wgpu::TexelCopyBufferInfo {
            buffer: &buffer,
            layout: wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(stride),
                rows_per_image: Some(texture.height()),
            },
        },
        texture.size(),
    );
    backend.queue.submit([encoder.finish()]);
    let (tx, rx) = std::sync::mpsc::channel();
    buffer.map_async(wgpu::MapMode::Read, .., move |result| {
        tx.send(result).unwrap()
    });
    backend
        .device
        .poll(wgpu::PollType::wait_indefinitely())
        .unwrap();
    rx.recv().unwrap().unwrap();
    let data = buffer.get_mapped_range(..).unwrap();
    let result = data
        .chunks_exact(stride as usize)
        .flat_map(|row| {
            row[..(texture.width() * bytes_per_pixel) as usize]
                .iter()
                .copied()
        })
        .collect();
    drop(data);
    buffer.unmap();
    result
}

fn halves(bytes: &[u8]) -> Vec<[f32; 4]> {
    bytes
        .as_chunks::<8>()
        .0
        .iter()
        .map(|pixel| {
            std::array::from_fn(|c| {
                half_to_f32(u16::from_le_bytes([pixel[2 * c], pixel[2 * c + 1]]))
            })
        })
        .collect()
}

/// Real primary shaders, including the material's early low-quality return,
/// stored forward-Z scene depth, spectral displacement, and conforming updates.
/// The independent oracle uses f64 source views + the actual quantized depth,
/// not the GPU uniform inverse or triangle/TileKey identity.
#[test]
fn scene_motion_masks_dynamics_matches_f64_and_resets_without_changing_rgb() {
    scene_oracle(false);
}

#[test]
fn jittered_scene_depth_matches_f64_and_static_camera_has_zero_motion() {
    scene_oracle(true);
}

fn scene_oracle(jitter_enabled: bool) {
    use thessa_graphics::{Quality, RayTracingRequest, RequestedGraphics};
    use thessa_rcbt_core::{
        HeightPage,
        lighting::SurfaceLighting,
        tile_anchor::{TileAnchor, TileKey},
    };
    use thessa_render_core::mesh::*;
    use thessa_render_wgpu::{
        display::HDR_FORMAT,
        mesh::{MeshRenderer, SurfaceSettings},
        motion::SceneMotion,
        terrain::{TerrainCapacity, TerrainPatch, TerrainRenderer},
    };
    let Some(backend) = backend() else {
        return;
    };
    let make_target = |format| {
        backend.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("scene-motion-oracle"),
            size: wgpu::Extent3d {
                width: SIZE,
                height: SIZE,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT
                | wgpu::TextureUsages::TEXTURE_BINDING
                | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        })
    };
    let color = make_target(HDR_FORMAT);
    let depth = make_target(wgpu::TextureFormat::Depth32Float);
    let color_view = color.create_view(&Default::default());
    let depth_view = depth.create_view(&Default::default());
    let radius = 3_200_000.0;
    let mut request = RequestedGraphics::default();
    request.renderer.ray_tracing = RayTracingRequest::Off;
    request.renderer.specular_reflections = false;
    request.renderer.terrain_normal_maps = false;
    request.temporal.jitter = jitter_enabled;
    request.shadows.enabled = false;
    request.terrain_topology.quality = Quality::Low;
    request.ocean.quality = Quality::Low;
    request.ocean.significant_height_m = 0.1;
    request.ocean.geometry_distance_m = 20000.0;
    let source = MeshAsset {
        primitives: vec![Primitive {
            ambient_visibility: vec![],
            vertices: [[-0.5, -0.5, -4.0], [0.5, -0.5, -4.0], [0.0, 0.5, -4.0]]
                .map(|position| Vertex {
                    position,
                    normal: [0.0, 0.0, 1.0],
                    uv: [0.0; 2],
                })
                .to_vec(),
            indices: vec![0, 1, 2],
            material: 0,
        }],
        materials: vec![Material {
            double_sided: true,
            ..Default::default()
        }],
        textures: vec![],
        images: vec![],
    };
    let lights = SurfaceLighting {
        ambient_lux: [1.0; 4],
        ..Default::default()
    };
    for conforming in [false, true] {
        request.terrain_topology.enabled = conforming;
        for (ocean, agl, mesh_quality, water_material_loaded, static_mesh) in [
            (false, 2.0, Quality::Low, false, false),
            (false, 50.0, Quality::High, false, false),
            (false, 400_000.0, Quality::Low, false, false),
            (true, 50.0, Quality::High, false, false),
            (true, 50.0, Quality::Low, true, false),
            (false, 2.0, Quality::High, false, true),
        ] {
            request.ocean.enabled = ocean;
            let page = HeightPage::bake(&[if ocean { 0.0 } else { 100.0 }; 4], 2, 0.001).unwrap();
            let keys = if !conforming && ocean {
                // Near pages resolve spectral displacement; root pages reduce
                // the wave field to its zero-mean DC mode by footprint policy.
                (511..513)
                    .flat_map(|y| (511..513).map(move |x| TileKey::new(4, 10, x, y).unwrap()))
                    .collect::<Vec<_>>()
            } else {
                (0..6)
                    .map(|face| TileKey::new(face, 0, 0, 0).unwrap())
                    .collect()
            };
            let patches = keys
                .into_iter()
                .map(|key| TerrainPatch {
                    anchor: TileAnchor::new(key, radius).unwrap(),
                    page: &page,
                })
                .collect::<Vec<_>>();
            let make_view = |x: f64| ViewParams {
                world_from_view: DMat4::from_translation(DVec3::new(
                    x,
                    0.0,
                    radius + if ocean { 0.0 } else { 100.0 } + agl,
                )),
                world_position_m: DVec3::new(
                    x,
                    0.0,
                    radius + if ocean { 0.0 } else { 100.0 } + agl,
                ),
                clip_from_view: glam::dcamera::rh::proj::directx::perspective(
                    0.9,
                    1.0,
                    if agl < 3.0 { 0.1 } else { 1.0 },
                    1e8,
                ),
                exposure: 1.0,
            };
            let mut reference = None;
            for enabled in [false, true] {
                request.temporal.motion_vectors = enabled;
                let settings = backend.resolve_graphics(&request);
                let mut motion = SceneMotion::new(&backend, &settings, [SIZE, SIZE]).unwrap();
                assert_eq!(motion.is_some(), enabled);
                let mut terrain = TerrainRenderer::with_capacity_lighting_and_ocean(
                    &backend,
                    &patches,
                    HDR_FORMAT,
                    TerrainCapacity {
                        patches: 6,
                        height_grid_size: 2,
                        material_layers: 6,
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
                if water_material_loaded {
                    let ids = (8..14).collect::<Vec<_>>();
                    let pages = ids
                        .iter()
                        .map(|&id| {
                            (
                                id,
                                (
                                    1,
                                    thessa_rcbt_core::material_page::CbtMaterialPage::from_rgba8(
                                        [10, 40, 80, 0].repeat(128 * 128),
                                    )
                                    .unwrap(),
                                ),
                            )
                        })
                        .collect();
                    terrain.update_materials(&backend, &ids, &pages).unwrap();
                }
                let build = if static_mesh {
                    MeshRenderer::with_static_surface_and_terrain
                } else {
                    MeshRenderer::with_surface_and_terrain
                };
                let mut mesh = build(
                    &backend,
                    &source,
                    SurfaceSettings {
                        format: HDR_FORMAT,
                        capacity: 1,
                        quality: mesh_quality,
                        reflections: false,
                        shadows: None,
                        ray_tracing: Some(&settings),
                    },
                    None,
                )
                .unwrap();
                terrain.update_lighting(&backend, &lights).unwrap();
                let previous = make_view(0.0);
                // Settle the actual persistent CBT, not a special test raster.
                for _ in 0..24 {
                    terrain
                        .update_view_for_extent(&backend, &previous, [SIZE, SIZE])
                        .unwrap();
                    terrain.update_ocean(&backend, 0.0).unwrap();
                    let mut encoder = backend.device.create_command_encoder(&Default::default());
                    terrain.encode_surface(&mut encoder);
                    backend.queue.submit([encoder.finish()]);
                    backend
                        .device
                        .poll(wgpu::PollType::wait_indefinitely())
                        .unwrap();
                }
                if ocean {
                    assert!(terrain.ocean_stats().geometry_bytes > 0);
                    if !conforming {
                        assert!(terrain.ocean_stats().deformed_patches > 0);
                    }
                }
                let mut current = make_view(agl * 0.005);
                current.world_from_view *= DMat4::from_rotation_y(0.003);
                let mut rgb = Vec::new();
                for frame in 0..7 {
                    let mut view = if frame == 0 { previous } else { current };
                    if frame >= 5 {
                        view.exposure = 2.0;
                    }
                    if frame == 6 {
                        view.clip_from_view.x_axis.x *= 1.01;
                    }
                    let jitter = motion
                        .as_ref()
                        .map_or(thessa_render_core::temporal::JitterOffset::ZERO, |m| {
                            m.jitter()
                        });
                    if let Some(motion) = &motion {
                        let index = match frame {
                            0 | 3 | 4 => 0,
                            1 | 5 => 1,
                            _ => 2,
                        };
                        assert_eq!(motion.submitted_frames(), index);
                        assert_eq!(
                            jitter,
                            motion.jitter(),
                            "querying a sample must not advance it"
                        );
                        assert_eq!(
                            jitter,
                            if jitter_enabled {
                                thessa_render_core::temporal::JitterOffset::halton(index)
                            } else {
                                thessa_render_core::temporal::JitterOffset::ZERO
                            }
                        );
                    }
                    terrain
                        .update_view_jittered(&backend, &view, [SIZE, SIZE], jitter)
                        .unwrap();
                    terrain.update_ocean(&backend, frame as f64 * 0.1).unwrap();
                    mesh.update(
                        &backend,
                        &view,
                        &lights,
                        &[MeshInstance {
                            position_m: if static_mesh {
                                previous.world_position_m
                            } else {
                                view.world_position_m
                            },
                            rotation: glam::DQuat::IDENTITY,
                            scale: if agl < 3.0 { 0.1 } else { 1.0 },
                        }],
                    )
                    .unwrap();
                    mesh.update_primary_jitter(&backend, &view, [SIZE, SIZE], jitter)
                        .unwrap();
                    let mut encoder = backend.device.create_command_encoder(&Default::default());
                    terrain.encode_surface(&mut encoder);
                    // false deliberately: resolved diagnostics MUST force Store.
                    terrain.encode_opaque_scene_ops(
                        &mut encoder,
                        &color_view,
                        &depth_view,
                        Some(&mesh),
                        !enabled,
                    );
                    if let Some(motion) = &motion {
                        motion
                            .encode(&backend, &mut encoder, &depth_view, &color_view, &view)
                            .unwrap();
                    }
                    terrain.encode_plume(&mut encoder, &color_view, &depth_view);
                    backend.queue.submit([encoder.finish()]);
                    let scene_bytes = read_texture(&backend, &color, 8);
                    rgb.extend(
                        scene_bytes
                            .as_chunks::<8>()
                            .0
                            .iter()
                            .flat_map(|p| p[..6].iter().copied()),
                    );
                    if let Some(motion) = &mut motion {
                        assert_eq!(motion.target_bytes(), u64::from(SIZE * SIZE * 8));
                        let vectors = halves(&read_texture(&backend, motion.texture(), 8));
                        if frame != 1 && frame != 2 {
                            assert!(
                                vectors.iter().all(|&p| p == [0.0; 4]),
                                "initial, uncommitted, reset/resize frames must be invalid"
                            );
                        } else {
                            let scene = halves(&scene_bytes);
                            let depths = read_texture(&backend, &depth, 4);
                            let mut valid = 0;
                            let mut invalid_finite = 0;
                            let mut max_error_px = 0.0f64;
                            for (i, &gpu) in vectors.iter().enumerate() {
                                let z = f32::from_le_bytes(
                                    depths[4 * i..4 * i + 4].try_into().unwrap(),
                                );
                                if z >= 1.0 || scene[i][3] < 1.0 {
                                    assert_eq!(gpu, [0.0; 4]);
                                    invalid_finite += usize::from(z < 1.0);
                                    continue;
                                }
                                valid += 1;
                                assert_eq!(gpu[3], 1.0);
                                let uv = [
                                    (i as u32 % SIZE) as f64 + 0.5,
                                    (i as u32 / SIZE) as f64 + 0.5,
                                ]
                                .map(|v| v / SIZE as f64);
                                let world = world_from_depth(
                                    view.world_from_view
                                        * jitter
                                            .apply(view.clip_from_view, [SIZE, SIZE])
                                            .unwrap()
                                            .inverse(),
                                    uv,
                                    f64::from(z),
                                )
                                .unwrap();
                                let source = if frame == 1 { previous } else { current };
                                let clip = source.clip_from_view
                                    * source.world_from_view.inverse()
                                    * world.extend(1.0);
                                let expected = [
                                    clip.x / clip.w * 0.5 + 0.5 - uv[0]
                                        + jitter.pixels[0] / SIZE as f64,
                                    0.5 - clip.y / clip.w * 0.5 - uv[1]
                                        + jitter.pixels[1] / SIZE as f64,
                                ];
                                for c in 0..2 {
                                    if frame == 2 {
                                        assert!(
                                            f64::from(gpu[c]).abs() * (SIZE as f64) < 0.02,
                                            "stationary camera must not report jitter as motion"
                                        );
                                    }
                                    max_error_px = max_error_px
                                        .max((f64::from(gpu[c]) - expected[c]).abs() * SIZE as f64);
                                }
                            }
                            if static_mesh {
                                assert_eq!(
                                    invalid_finite, 0,
                                    "static plants must admit camera reprojection"
                                );
                            } else {
                                assert!(invalid_finite > 10, "moving mesh must remain masked");
                            }
                            if ocean {
                                assert_eq!(
                                    valid, 0,
                                    "displaced ocean must never publish static motion even with rough fallback material"
                                );
                            } else {
                                assert!(valid > 1000);
                            }
                            assert!(
                                max_error_px < 0.02,
                                "planet-scale f64 oracle error {max_error_px} px"
                            );
                            println!(
                                "scene_motion static_mesh={static_mesh} jitter={jitter_enabled} frame={frame} conforming={conforming} ocean={ocean} camera_altitude_above_page_m={agl} valid={valid} invalid_finite={invalid_finite} max_error_px={max_error_px:.6}"
                            );
                        }
                        // Frame 0 intentionally remains uncommitted until its
                        // readback above. Encode does not advance the source.
                        if frame == 0 {
                            motion.commit(previous);
                        }
                        if frame == 1 {
                            motion.commit(current);
                        }
                        if frame == 2 {
                            motion.reset();
                        }
                        if frame == 3 {
                            motion.commit(current);
                            assert!(motion.resize(&backend, [0, SIZE]).is_err());
                            motion.resize(&backend, [32, 32]).unwrap();
                            motion.resize(&backend, [SIZE, SIZE]).unwrap();
                        }
                        if frame >= 4 {
                            motion.commit(view);
                        }
                    }
                }
                if enabled && !jitter_enabled {
                    assert_eq!(
                        reference.as_ref().unwrap(),
                        &rgb,
                        "motion diagnostics must not change even HDR RGB bits"
                    );
                } else if !enabled {
                    reference = Some(rgb);
                } else {
                    assert_ne!(
                        reference.as_ref().unwrap(),
                        &rgb,
                        "jitter diagnostic must affect primary visibility, not silently remain unused"
                    );
                }
            }
        }
    }
}
