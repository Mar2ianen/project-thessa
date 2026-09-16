//! Execute the production WGSL and read back its results. These tests require
//! a wgpu adapter (a software Vulkan adapter is sufficient).
use super::*;
use wgpu::util::DeviceExt;

struct Gpu {
    device: wgpu::Device,
    queue: wgpu::Queue,
}

impl Gpu {
    fn new() -> Self {
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
        let adapter = pollster::block_on(instance.request_adapter(&Default::default()))
            .expect("GPU regression tests require a wgpu adapter");
        eprintln!("CBT regression adapter: {:?}", adapter.get_info());
        let (device, queue) =
            pollster::block_on(adapter.request_device(&Default::default())).unwrap();
        Self { device, queue }
    }

    fn buffer(&self, words: &[u32], uniform: bool) -> wgpu::Buffer {
        let bytes: Vec<_> = words.iter().flat_map(|word| word.to_le_bytes()).collect();
        self.device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("cbt-test-data"),
                contents: &bytes,
                usage: wgpu::BufferUsages::COPY_SRC
                    | wgpu::BufferUsages::COPY_DST
                    | if uniform {
                        wgpu::BufferUsages::UNIFORM
                    } else {
                        wgpu::BufferUsages::STORAGE
                    },
            })
    }

    fn dispatch(
        &self,
        shader: &str,
        entries: &[(&str, u32)],
        buffers: &[&wgpu::Buffer],
        uniforms: &[u32],
        writable: &[u32],
    ) {
        let module = self
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("cbt-production-wgsl"),
                source: wgpu::ShaderSource::Wgsl(shader.into()),
            });
        let layout_entries: Vec<_> = buffers
            .iter()
            .enumerate()
            .map(|(binding, _)| wgpu::BindGroupLayoutEntry {
                binding: binding as u32,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: if uniforms.contains(&(binding as u32)) {
                        wgpu::BufferBindingType::Uniform
                    } else {
                        wgpu::BufferBindingType::Storage {
                            read_only: !writable.contains(&(binding as u32)),
                        }
                    },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            })
            .collect();
        let layout = self
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: None,
                entries: &layout_entries,
            });
        let pipeline_layout = self
            .device
            .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: None,
                bind_group_layouts: &[Some(&layout)],
                immediate_size: 0,
            });
        let bindings: Vec<_> = buffers
            .iter()
            .enumerate()
            .map(|(binding, buffer)| wgpu::BindGroupEntry {
                binding: binding as u32,
                resource: buffer.as_entire_binding(),
            })
            .collect();
        let group = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: None,
            layout: &layout,
            entries: &bindings,
        });
        let mut encoder = self.device.create_command_encoder(&Default::default());
        for (entry, groups) in entries {
            let pipeline = self
                .device
                .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                    label: Some(entry),
                    layout: Some(&pipeline_layout),
                    module: &module,
                    entry_point: Some(entry),
                    compilation_options: Default::default(),
                    cache: None,
                });
            let mut pass = encoder.begin_compute_pass(&Default::default());
            pass.set_pipeline(&pipeline);
            pass.set_bind_group(0, &group, &[]);
            pass.dispatch_workgroups(*groups, 1, 1);
        }
        self.queue.submit([encoder.finish()]);
    }

    fn read(&self, buffer: &wgpu::Buffer) -> Vec<u32> {
        let staging = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: None,
            size: buffer.size(),
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let mut encoder = self.device.create_command_encoder(&Default::default());
        encoder.copy_buffer_to_buffer(buffer, 0, &staging, 0, buffer.size());
        self.queue.submit([encoder.finish()]);
        let (tx, rx) = std::sync::mpsc::channel();
        staging
            .slice(..)
            .map_async(wgpu::MapMode::Read, move |result| tx.send(result).unwrap());
        self.device
            .poll(wgpu::PollType::wait_indefinitely())
            .unwrap();
        rx.recv().unwrap().unwrap();
        let words = staging
            .slice(..)
            .get_mapped_range()
            .chunks_exact(4)
            .map(|bytes| u32::from_le_bytes(bytes.try_into().unwrap()))
            .collect();
        staging.unmap();
        words
    }
}

#[test]
#[ignore = "requires a wgpu adapter; run with --ignored --nocapture"]
fn gpu_geometry_matches_cube_sphere_and_has_finite_border_normals() {
    let gpu = Gpu::new();
    for radius in [1_737_400_f32, 3_200_000_f32, 6_371_000_f32] {
        check_geometry(&gpu, radius);
    }
}

fn check_geometry(gpu: &Gpu, radius: f32) {
    let mut records = Vec::new();
    let mut tiles = Vec::new();
    for level in [0, 17] {
        for face in 0..6_u32 {
            let x = if level == 0 { 0 } else { 57_213_u32 };
            let y = if level == 0 { 0 } else { 89_319_u32 };
            let mut id = 8 + u64::from(face);
            for bit in (0..level).rev() {
                id = (id << 2) | u64::from(((x >> bit) & 1) * 2 + ((y >> bit) & 1));
            }
            records.extend([
                id as u32,
                (id >> 32) as u32,
                3 + level * 2,
                tiles.len() as u32,
            ]);
            tiles.push((face, level, x, y));
        }
    }
    let count = tiles.len();
    let anchors: Vec<_> = tiles
        .iter()
        .map(|&(face, level, x, y)| {
            crate::precision::TileAnchor::new(
                crate::precision::TileKey::new(face as u8, level as u8, x, y).unwrap(),
                f64::from(radius),
            )
            .unwrap()
        })
        .collect();
    let frame_words: Vec<_> = anchors
        .iter()
        .flat_map(|anchor| {
            let frame = anchor.to_gpu([0.0; 3]).unwrap();
            [
                frame.anchor_hi_m,
                frame.anchor_lo_m,
                frame.raw_center,
                frame.raw_axis_u,
                frame.raw_axis_v,
                frame.normal,
                frame.radius_half_extent_len,
            ]
            .into_iter()
            .flatten()
            .map(f32::to_bits)
        })
        .collect();
    let frames = gpu.buffer(&frame_words, false);
    let leaves = gpu.buffer(&records, false);
    let patches = gpu.buffer(&vec![0; count * 4], false);
    let page = HeightPage::bake(&[120.0, 123.0, 121.0, 125.0], 2, 0.001).unwrap();
    let mut packed = Vec::new();
    pack_page_residuals(&page, &mut packed);
    let flat = HeightPage::bake(&[123.0; 4], 2, 0.001).unwrap();
    let flat_offset = pack_page_residuals(&flat, &mut packed);
    let metadata_words: Vec<_> = tiles
        .iter()
        .flat_map(|(face, _, _, _)| {
            let (p, offset) = if face % 2 == 0 {
                (&page, 0)
            } else {
                (&flat, flat_offset)
            };
            [
                p.base_height_m().to_bits(),
                p.residual_scale_m().to_bits(),
                2,
                offset,
            ]
        })
        .collect();
    let metadata = gpu.buffer(&metadata_words, false);
    let residuals = gpu.buffer(&packed, false);
    let vertices = gpu.buffer(&vec![0; count * GPU_VERTEX_COUNT_PER_PATCH * 8], false);
    let params = gpu.buffer(
        &[
            count as u32,
            GPU_VERTEX_COUNT_PER_PATCH as u32,
            radius.to_bits(),
            0,
        ],
        true,
    );
    gpu.dispatch(
        CBT_GEOMETRY_WGSL,
        &[(
            "build_geometry",
            (count as u32 * GPU_VERTEX_COUNT_PER_PATCH as u32).div_ceil(64),
        )],
        &[
            &leaves, &patches, &metadata, &residuals, &vertices, &params, &frames,
        ],
        &[5],
        &[1, 4],
    );
    let output = gpu.read(&vertices);
    for (ordinal, (face, level, x, y)) in tiles.iter().copied().enumerate() {
        for gy in 0..GPU_GRID_SIZE {
            for gx in 0..GPU_GRID_SIZE {
                let a = 2.0 * (x as f64 + gx as f64 / 32.0) / 2_f64.powi(level as i32) - 1.0;
                let b = 2.0 * (y as f64 + gy as f64 / 32.0) / 2_f64.powi(level as i32) - 1.0;
                let raw = match face {
                    0 => [1.0, b, -a],
                    1 => [-1.0, b, a],
                    2 => [a, 1.0, -b],
                    3 => [a, -1.0, b],
                    4 => [a, b, 1.0],
                    _ => [-a, b, -1.0],
                };
                let length = raw.iter().map(|x| x * x).sum::<f64>().sqrt();
                let dir = raw.map(|x| x / length);
                let offset = (ordinal * GPU_VERTEX_COUNT_PER_PATCH + gy * GPU_GRID_SIZE + gx) * 8;
                let actual: Vec<_> = output[offset..offset + 8]
                    .iter()
                    .map(|word| f32::from_bits(*word) as f64)
                    .collect();
                let source = if face % 2 == 0 { &page } else { &flat };
                let height = f64::from(source.sample(gx as f32 / 32.0, gy as f32 / 32.0));
                let error = (0..3)
                    .map(|i| {
                        (actual[i] + anchors[ordinal].anchor_body_m[i]
                            - dir[i] * (f64::from(radius) + height))
                            .powi(2)
                    })
                    .sum::<f64>()
                    .sqrt();
                assert!(
                    error < if level == 0 { 2.0 } else { 0.001 },
                    "R={radius} face={face} L{level} ({gx},{gy}) error={error}"
                );
                let norm = actual[4..7].iter().map(|x| x * x).sum::<f64>().sqrt();
                assert!(
                    (norm - 1.0).abs() < 1e-5,
                    "invalid border normal at {ordinal}:{gx},{gy}: {actual:?}"
                );
                let outward = (0..3).map(|i| actual[i + 4] * dir[i]).sum::<f64>();
                assert!(outward > 0.8, "inward/unstable normal: {outward}");
                if face % 2 == 1 {
                    let error = (0..3)
                        .map(|i| (actual[i + 4] - dir[i]).powi(2))
                        .sum::<f64>()
                        .sqrt();
                    assert!(
                        error < 1e-5,
                        "flat-surface normal must not acquire a false slope: {error}"
                    );
                }
            }
        }
    }
}

#[test]
#[ignore = "requires a wgpu adapter; run with --ignored --nocapture"]
fn gpu_classifier_uses_position_stride_for_every_leaf() {
    let gpu = Gpu::new();
    let metadata = gpu.buffer(&[0, 0, 33, 0].repeat(3), false);
    let mut positions = Vec::new();
    for x in [10_f32, 0_f32, -10_f32] {
        for _ in 0..GPU_VERTEX_COUNT_PER_PATCH {
            positions.extend([x, 0.0, 0.5, 1.0, 0.0, 1.0, 0.0, 0.0].map(f32::to_bits));
        }
    }
    let vertices = gpu.buffer(&positions, false);
    let triangles = gpu.buffer(&vec![0; 3 * GPU_TRIANGLE_COUNT_PER_PATCH * 4], false);
    let count = gpu.buffer(&[0; 4], false);
    let draw = gpu.buffer(&[0; 4], false);
    let identity = [
        1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.,
    ]
    .map(f32::to_bits);
    let view = gpu.buffer(&identity, true);
    let transform = gpu.buffer(&identity, true);
    let params = gpu.buffer(
        &[3, GPU_VERTEX_COUNT_PER_PATCH as u32, 1_f32.to_bits(), 0],
        true,
    );
    let leaves = gpu.buffer(&[8, 0, 3, 0, 9, 0, 3, 1, 10, 0, 3, 2], false);
    let frames = gpu.buffer(&[0; 3 * 28], false);
    gpu.dispatch(
        CBT_CLASSIFY_WGSL,
        &[
            ("reset_active", 1),
            ("classify_active", 3),
            ("finalize_active", 1),
        ],
        &[
            &metadata, &vertices, &triangles, &count, &draw, &view, &transform, &params, &leaves,
            &frames,
        ],
        &[5, 6, 7],
        &[2, 3, 4],
    );
    // A zero-span patch uses step 8: 4x4x2 surface + 4x4x2 skirt triangles.
    assert_eq!(gpu.read(&draw), [192, 1, 0, 0]);
    assert_eq!(gpu.read(&count)[0], 64);
    let triangles = gpu.read(&triangles);
    for record in triangles[..64 * 4].chunks_exact(4) {
        assert_eq!(record[0], 1, "only the middle leaf is visible");
        assert!(
            record[1..]
                .iter()
                .all(|index| (index & 0x7fffffff) < GPU_VERTEX_COUNT_PER_PATCH as u32)
        );
    }
}

#[test]
#[ignore = "requires a wgpu adapter; run with --ignored --nocapture"]
fn gpu_surface_uv_matches_north_first_east_positive_bake() {
    let gpu = Gpu::new();
    let function = &CBT_RASTER_WGSL
        [CBT_RASTER_WGSL.find("fn surface_uv").unwrap()..CBT_RASTER_WGSL.find("@vertex").unwrap()];
    let shader = format!(
        r#"
        {function}
        @group(0) @binding(0) var<storage, read> directions: array<vec4<f32>>;
        @group(0) @binding(1) var<storage, read_write> result: array<vec4<f32>>;
        @compute @workgroup_size(1)
        fn check_uv(@builtin(global_invocation_id) id: vec3<u32>) {{
            result[id.x] = vec4(surface_uv(directions[id.x].xyz), 0.0, 0.0);
        }}
    "#
    );
    let input = [
        1., 0., 0., 0., 0., 0., -1., 0., 0., 1., 0., 0., 0., -1., 0., 0.,
    ]
    .map(f32::to_bits);
    let directions = gpu.buffer(&input, false);
    let result = gpu.buffer(&[0; 16], false);
    gpu.dispatch(
        &shader,
        &[("check_uv", 4)],
        &[&directions, &result],
        &[],
        &[1],
    );
    let result = gpu.read(&result);
    for (row, expected) in [[0.0, 0.5], [0.25, 0.5], [0.0, 0.001], [0.0, 0.999]]
        .into_iter()
        .enumerate()
    {
        for axis in 0..2 {
            let actual = f32::from_bits(result[row * 4 + axis]);
            assert!(
                (actual - expected[axis]).abs() < 1e-5,
                "row={row} axis={axis}: {actual}"
            );
        }
    }
}

#[test]
#[ignore = "requires a wgpu adapter; run with --ignored --nocapture"]
fn gpu_ocean_reflection_is_finite_and_filters_unresolved_waves() {
    let gpu = Gpu::new();
    let shader = format!(
        "{}\n{}",
        include_str!("ocean.wgsl"),
        r#"
@group(0) @binding(0) var<storage, read_write> result: array<vec4<f32>>;
@compute @workgroup_size(1)
fn check() {
    let up = vec3(1.0, 0.0, 0.0);
    let rotation = mat3x3<f32>(vec3(1.0,0.0,0.0), vec3(0.0,1.0,0.0), vec3(0.0,0.0,1.0));
    let wave = vec4(0.1, 100000.0, 1.73, 0.5);
    result[0] = vec4(ocean_wave_normal(up, up, rotation, vec2(0.1, 0.2), 1.0, wave), 1.0);
    result[1] = vec4(ocean_schlick(0.0204, 1.0), ocean_schlick(0.0204, 0.0), 0.0, 1.0);
    result[2] = vec4(ocean_sun_reflection(up, up, -up, vec3(89000.0), 0.16, 0.0204), 1.0);
    result[3] = vec4(ocean_sun_reflection(up, up, up, vec3(89000.0), 0.16, 0.0204), 1.0);
    // Local radial up is +X, so this must sample zenith, independent of +Y.
    result[4] = vec4(ocean_sky_radiance(up, up, vec3(3.0), vec3(2.0), vec3(1.0)), 1.0);
    result[5] = vec4(ocean_wave_normal(up, up, rotation, vec2(0.1, 0.2), 0.0, wave), 1.0);
}
"#
    );
    let output = gpu.buffer(&[0; 24], false);
    gpu.dispatch(&shader, &[("check", 1)], &[&output], &[], &[0]);
    let values: Vec<_> = gpu.read(&output).into_iter().map(f32::from_bits).collect();
    assert!(values.iter().all(|x| x.is_finite()));
    assert_eq!(&values[0..3], &[1.0, 0.0, 0.0]);
    assert!((values[4] - 0.0204).abs() < 1e-6);
    assert_eq!(values[5], 1.0);
    assert_eq!(&values[8..11], &[0.0; 3]);
    assert!(values[12..15].iter().all(|x| *x > 0.0));
    assert_eq!(&values[16..19], &[3.0; 3]);
    let norm = values[20..23].iter().map(|x| x * x).sum::<f32>().sqrt();
    assert!((norm - 1.0).abs() < 1e-5);
    assert!(values[20] > 0.95);
}

#[test]
#[ignore = "requires a wgpu adapter; run with --ignored --nocapture"]
fn gpu_tile_transform_keeps_near_camera_precision_after_body_rotation() {
    use bevy::math::{DMat4, DQuat, DVec3};
    let gpu = Gpu::new();
    let anchor = crate::precision::TileAnchor::new(
        crate::precision::TileKey::new(2, 17, 57_213, 89_319).unwrap(),
        6_371_000.0,
    )
    .unwrap();
    let rotation = DQuat::from_rotation_y(0.731) * DQuat::from_rotation_z(-0.29);
    let anchor_body = DVec3::from_array(anchor.anchor_body_m);
    let origin = rotation * anchor_body + DVec3::new(17.125, -31.75, 0.123456);
    let transform = DMat4::from_rotation_translation(rotation, -origin);
    let relative = transform.transform_point3(anchor_body).to_array();
    let mut frame = anchor.to_gpu([0.0; 3]).unwrap();
    for i in 0..3 {
        frame.anchor_hi_m[i] = relative[i] as f32;
        frame.anchor_lo_m[i] = (relative[i] - f64::from(frame.anchor_hi_m[i])) as f32;
    }
    let frames = gpu.buffer(
        &frame
            .words()
            .into_iter()
            .flatten()
            .map(f32::to_bits)
            .collect::<Vec<_>>(),
        false,
    );
    let matrix = gpu.buffer(
        &transform.to_cols_array().map(|x| (x as f32).to_bits()),
        true,
    );
    let output = gpu.buffer(&[0; 4], false);
    let shader = format!(
        "{}\n{}\n{}",
        include_str!("precision.wgsl"),
        include_str!("tile_frame.wgsl"),
        r#"
@group(0) @binding(0) var<storage, read> frames: array<CbtTileFrame>;
@group(0) @binding(1) var<uniform> transform: mat4x4<f32>;
@group(0) @binding(2) var<storage, read_write> result: array<vec4<f32>>;
@compute @workgroup_size(1)
fn check() { result[0] = cbt_render_position(frames[0], vec3(12.25, -2.125, 33.03125), transform); }
"#
    );
    gpu.dispatch(
        &shader,
        &[("check", 1)],
        &[&frames, &matrix, &output],
        &[1],
        &[2],
    );
    let values: Vec<_> = gpu.read(&output).into_iter().map(f32::from_bits).collect();
    let expected = transform.transform_point3(anchor_body + DVec3::new(12.25, -2.125, 33.03125));
    let actual = DVec3::new(
        f64::from(values[0]),
        f64::from(values[1]),
        f64::from(values[2]),
    );
    assert!(
        (actual - expected).length() < 0.0001,
        "actual={actual} expected={expected}"
    );
}

/// Execute the actual mesh entry body as compute, replacing only its stage IO.
/// This tests all lanes/outputs even on adapters without native mesh shaders.
#[cfg(feature = "mesh-shaders")]
#[test]
#[ignore = "requires a wgpu adapter; run with --include-ignored"]
fn gpu_mesh_emission_initializes_every_vertex_and_primitive() {
    let gpu = Gpu::new();
    let mut shader = CBT_MESH_WGSL.split("@fragment").next().unwrap().to_owned();
    shader = shader.replace("enable wgpu_mesh_shader;", "");
    for attribute in [
        "@builtin(position)",
        "@location(0)",
        "@builtin(triangle_indices)",
        "@builtin(vertex_count)",
        "@builtin(primitive_count)",
        "@builtin(vertices)",
        "@builtin(primitives)",
    ] {
        shader = shader.replace(attribute, "");
    }
    shader = shader
        .replace("@mesh(mesh_output)", "@compute")
        .replace(
            "var<workgroup> mesh_output: MeshOutput;",
            "@group(0) @binding(7) var<storage, read_write> mesh_output: MeshOutput;",
        )
        .replace("@group(1) @binding(0)", "@group(0) @binding(1)")
        .replace("@group(1) @binding(1)", "@group(0) @binding(5)")
        .replace(
            "let meshlet = workgroup.x;",
            "let meshlet = params._padding;",
        );
    let radius = 6_371_000.0f32;
    let anchor = crate::precision::TileAnchor::new(
        crate::precision::TileKey::new(4, 17, 65537, 65539).unwrap(),
        radius as f64,
    )
    .unwrap();
    let frames = gpu.buffer(
        &anchor
            .to_gpu(anchor.anchor_body_m)
            .unwrap()
            .words()
            .into_iter()
            .flatten()
            .map(f32::to_bits)
            .collect::<Vec<_>>(),
        false,
    );
    let identity = [
        1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.,
    ]
    .map(f32::to_bits);
    let matrix = gpu.buffer(&identity, true);
    let leaves = gpu.buffer(&[0; 4], false);
    let residuals = gpu.buffer(&[0; 2], false);
    let metadata = gpu.buffer(&[120.0f32.to_bits(), 1.0f32.to_bits(), 2, 0], false);
    // vec3 members align to 16 bytes: header16, vertex32, primitive16.
    let sentinel = 0x7fc00001;
    let words = 4 + 81 * 8 + 128 * 4;
    for meshlet in 0..16 {
        let output = gpu.buffer(&vec![sentinel; words], false);
        let params = gpu.buffer(&[1, 1089, radius.to_bits(), meshlet], true);
        gpu.dispatch(
            &shader,
            &[("build_mesh", 1)],
            &[
                &leaves, &matrix, &metadata, &residuals, &frames, &matrix, &params, &output,
            ],
            &[1, 5, 6],
            &[7],
        );
        let result = gpu.read(&output);
        assert_eq!(&result[..2], &[81, 128]);
        for i in 0..81 {
            let vertex: Vec<_> = result[4 + i * 8..4 + i * 8 + 7]
                .iter()
                .copied()
                .map(f32::from_bits)
                .collect();
            assert!(vertex.iter().all(|v| v.is_finite()), "unwritten vertex {i}");
            let uv = [
                ((meshlet % 4) * 8 + i as u32 % 9) as f64 / 32.,
                ((meshlet / 4) * 8 + i as u32 / 9) as f64 / 32.,
            ];
            let expected = anchor.project_body_m(uv, 120.0);
            for axis in 0..3 {
                assert!(
                    (vertex[axis] as f64 - (expected[axis] - anchor.anchor_body_m[axis])).abs()
                        < 0.001,
                    "meshlet {meshlet} vertex {i} axis {axis}: {}",
                    vertex[axis]
                );
            }
            assert!((vertex[4..7].iter().map(|v| v * v).sum::<f32>() - 1.).abs() < 1e-5);
        }
        let mut area = 0.;
        for i in 0..128 {
            let tri = &result[4 + 81 * 8 + i * 4..4 + 81 * 8 + i * 4 + 3];
            assert!(
                tri.iter().all(|v| *v < 81),
                "unwritten primitive {i}: {tri:?}"
            );
            let xy: Vec<_> = tri
                .iter()
                .map(|v| [(v % 9) as f32, (v / 9) as f32])
                .collect();
            let signed = (xy[1][0] - xy[0][0]) * (xy[2][1] - xy[0][1])
                - (xy[1][1] - xy[0][1]) * (xy[2][0] - xy[0][0]);
            assert_eq!(signed, 1.);
            area += signed * 0.5;
        }
        assert_eq!(area, 64.);
    }
    // Missing height data must emit zero counts rather than partially valid IO.
    let absent = gpu.buffer(&[0; 4], false);
    let output = gpu.buffer(&vec![sentinel; words], false);
    let params = gpu.buffer(&[1, 1089, radius.to_bits(), 0], true);
    gpu.dispatch(
        &shader,
        &[("build_mesh", 1)],
        &[
            &leaves, &matrix, &absent, &residuals, &frames, &matrix, &params, &output,
        ],
        &[1, 5, 6],
        &[7],
    );
    assert_eq!(&gpu.read(&output)[..2], &[0, 0]);
}
