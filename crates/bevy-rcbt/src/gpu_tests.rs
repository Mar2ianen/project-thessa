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
            .as_chunks::<4>()
            .0
            .iter()
            .map(|bytes| u32::from_le_bytes(*bytes))
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
            count as u32,
        ],
        true,
    );
    // Full-cover dispatch: every ordinal is dirty.
    let dirty: Vec<u32> = (0..count as u32).collect();
    let dirty_ordinals = gpu.buffer(&dirty, false);
    gpu.dispatch(
        CBT_GEOMETRY_WGSL,
        &[(
            "build_geometry",
            (count as u32 * GPU_VERTEX_COUNT_PER_PATCH as u32).div_ceil(64),
        )],
        &[
            &leaves,
            &patches,
            &metadata,
            &residuals,
            &vertices,
            &params,
            &frames,
            &dirty_ordinals,
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
fn gpu_geometry_dirty_dispatch_regenerates_only_dirty_leaves() {
    // One arriving page must regenerate one patch: full-cover dispatch,
    // then a single-ordinal dispatch over the same vertex buffer. Untouched
    // leaves stay bit-identical; the dirty leaf moves with its new page.
    let gpu = Gpu::new();
    let radius = 6_371_000_f32;
    let count = 3_usize;
    let records: Vec<u32> = (0..count as u32)
        .flat_map(|ordinal| [8 + ordinal, 0, 3, ordinal])
        .collect();
    let leaves = gpu.buffer(&records, false);
    let patches = gpu.buffer(&vec![0; count * 4], false);
    let anchors: Vec<_> = (0..3)
        .map(|face| {
            crate::precision::TileAnchor::new(
                crate::precision::TileKey::new(face, 0, 0, 0).unwrap(),
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
    // Leaf 1 starts flat at 100 m; the update lifts it to 110 m.
    let flat = HeightPage::bake(&[100.0; 4], 2, 0.001).unwrap();
    let lifted = HeightPage::bake(&[110.0; 4], 2, 0.001).unwrap();
    let mut packed = Vec::new();
    pack_page_residuals(&flat, &mut packed);
    let lifted_offset = pack_page_residuals(&lifted, &mut packed);
    let metadata_for = |lifted_active: bool| {
        (0..count)
            .flat_map(|ordinal| {
                let page = if lifted_active && ordinal == 1 {
                    &lifted
                } else {
                    &flat
                };
                let offset = if lifted_active && ordinal == 1 {
                    lifted_offset
                } else {
                    0
                };
                [
                    page.base_height_m().to_bits(),
                    page.residual_scale_m().to_bits(),
                    2,
                    offset,
                ]
            })
            .collect::<Vec<_>>()
    };
    let vertices = gpu.buffer(&vec![0; count * GPU_VERTEX_COUNT_PER_PATCH * 8], false);
    let run_dispatch = |gpu: &Gpu,
                        metadata: &wgpu::Buffer,
                        residuals: &wgpu::Buffer,
                        dirty: &[u32],
                        vertices: &wgpu::Buffer| {
        let dirty_ordinals = gpu.buffer(dirty, false);
        // The dirty count sizes the dispatch guard; the ordinal list
        // addresses the ordinal-indexed vertex buffer.
        let params = gpu.buffer(
            &[
                count as u32,
                GPU_VERTEX_COUNT_PER_PATCH as u32,
                radius.to_bits(),
                dirty.len() as u32,
            ],
            true,
        );
        gpu.dispatch(
            CBT_GEOMETRY_WGSL,
            &[(
                "build_geometry",
                (dirty.len() as u32 * GPU_VERTEX_COUNT_PER_PATCH as u32).div_ceil(64),
            )],
            &[
                &leaves,
                &patches,
                metadata,
                residuals,
                vertices,
                &params,
                &frames,
                &dirty_ordinals,
            ],
            &[5],
            &[1, 4],
        );
    };
    // Full cover first.
    let metadata = gpu.buffer(&metadata_for(false), false);
    let residuals = gpu.buffer(&packed, false);
    let all: Vec<u32> = (0..count as u32).collect();
    run_dispatch(&gpu, &metadata, &residuals, &all, &vertices);
    let before = gpu.read(&vertices);
    // Single-ordinal dispatch with the lifted page for leaf 1.
    let metadata_lifted = gpu.buffer(&metadata_for(true), false);
    run_dispatch(&gpu, &metadata_lifted, &residuals, &[1], &vertices);
    let after = gpu.read(&vertices);
    let span = GPU_VERTEX_COUNT_PER_PATCH * 8;
    for ordinal in 0..count {
        let (old, new) = (
            &before[ordinal * span..(ordinal + 1) * span],
            &after[ordinal * span..(ordinal + 1) * span],
        );
        if ordinal == 1 {
            assert_ne!(old, new, "dirty leaf 1 must regenerate");
            // Height channel (word 7 of each vertex) moves ~10 m.
            let drift: f32 = old
                .as_chunks::<8>()
                .0
                .iter()
                .zip(new.as_chunks::<8>().0.iter())
                .map(|(a, b)| (f32::from_bits(b[7]) - f32::from_bits(a[7])).abs())
                .sum::<f32>()
                / GPU_VERTEX_COUNT_PER_PATCH as f32;
            assert!(
                (drift - 10.0).abs() < 0.5,
                "leaf 1 height must follow its page, drift={drift}"
            );
        } else {
            assert_eq!(old, new, "clean leaf {ordinal} must stay bit-identical");
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
    let draw = gpu.buffer(&[0; 8], false);
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
    let history = gpu.buffer(&[0; 3 * 4], false);
    gpu.dispatch(
        CBT_CLASSIFY_WGSL,
        &[
            ("reset_active", 1),
            ("classify_active", 3),
            ("finalize_active", 1),
        ],
        &[
            &metadata, &vertices, &triangles, &count, &draw, &view, &transform, &params, &leaves,
            &frames, &history,
        ],
        &[5, 6, 7],
        &[2, 3, 4, 10],
    );
    // A zero-span patch uses step 8: 4x4x2 surface + 4x4x2 skirt triangles.
    assert_eq!(gpu.read(&draw), [192, 1, 0, 0, 2, 1, 1, 0]);
    assert_eq!(gpu.read(&count)[0], 64);
    // Mesh arguments fan out at 256 groups/row, including an empty draw.
    for total in [0u32, 1, 31, 32, 33, 8192, 8193] {
        gpu.queue.write_buffer(&count, 0, &total.to_le_bytes());
        gpu.dispatch(
            CBT_CLASSIFY_WGSL,
            &[("finalize_active", 1)],
            &[
                &metadata, &vertices, &triangles, &count, &draw, &view, &transform, &params,
                &leaves, &frames, &history,
            ],
            &[5, 6, 7],
            &[2, 3, 4, 10],
        );
        let groups = total.div_ceil(32);
        assert_eq!(
            gpu.read(&draw),
            [
                total * 3,
                1,
                0,
                0,
                groups.min(256),
                groups.div_ceil(256).max(1),
                1,
                0
            ]
        );
    }
    let triangles = gpu.read(&triangles);
    for record in triangles[..64 * 4].as_chunks::<4>().0.iter() {
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
fn gpu_classifier_grid_step_history_has_hysteresis_and_identity_reset() {
    let gpu = Gpu::new();
    let metadata = gpu.buffer(&[0, 0, 33, 0], false);
    let identity = [
        1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.,
    ]
    .map(f32::to_bits);
    let view = gpu.buffer(&identity, true);
    let transform = gpu.buffer(&identity, true);
    let params = gpu.buffer(
        &[1, GPU_VERTEX_COUNT_PER_PATCH as u32, 1_f32.to_bits(), 0],
        true,
    );
    let frames = gpu.buffer(&[0; 28], false);
    let history = gpu.buffer(&[0; 4], false);

    let vertices_for = |span: f32| {
        let mut words = Vec::with_capacity(GPU_VERTEX_COUNT_PER_PATCH * 8);
        for y in 0..33 {
            for x in 0..33 {
                words.extend(
                    [
                        x as f32 / 32.0 * span,
                        y as f32 / 32.0 * span,
                        0.5,
                        1.0,
                        0.0,
                        0.0,
                        1.0,
                        0.0,
                    ]
                    .map(f32::to_bits),
                );
            }
        }
        gpu.buffer(&words, false)
    };
    let dispatch = |vertices: &wgpu::Buffer, leaf_id: u32| {
        let leaves = gpu.buffer(&[leaf_id, 0, 3, 0], false);
        let triangles = gpu.buffer(&vec![0; 3 * GPU_TRIANGLE_COUNT_PER_PATCH * 4], false);
        let count = gpu.buffer(&[0; 4], false);
        let draw = gpu.buffer(&[0; 8], false);
        gpu.dispatch(
            CBT_CLASSIFY_WGSL,
            &[
                ("reset_active", 1),
                ("classify_active", 1),
                ("finalize_active", 1),
            ],
            &[
                &metadata, vertices, &triangles, &count, &draw, &view, &transform, &params,
                &leaves, &frames, &history,
            ],
            &[5, 6, 7],
            &[2, 3, 4, 10],
        );
        (draw, leaves)
    };

    let (draw, _) = dispatch(&vertices_for(0.039), 8);
    assert_eq!(gpu.read(&draw)[0], 576, "initial step 4");
    assert_eq!(gpu.read(&history), [8, 0, 4, 0]);

    let (draw, _) = dispatch(&vertices_for(0.041), 8);
    assert_eq!(gpu.read(&draw)[0], 576, "jitter must retain step 4");
    assert_eq!(gpu.read(&history), [8, 0, 4, 0]);

    let (draw, _) = dispatch(&vertices_for(0.2), 8);
    assert_eq!(
        gpu.read(&draw)[0],
        6912,
        "meaningful crossing refines to step 1"
    );
    assert_eq!(gpu.read(&history), [8, 0, 1, 0]);

    let (draw, _) = dispatch(&vertices_for(0.039), 9);
    assert_eq!(
        gpu.read(&draw)[0],
        576,
        "new leaf identity must reset history"
    );
    assert_eq!(gpu.read(&history), [9, 0, 4, 0]);
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
    // Production land path shares this dielectric kernel (F0=0.04), but
    // consumes the land material's roughness rather than the ocean setting.
    result[6] = vec4(ocean_sun_reflection(up, up, up, vec3(89000.0), 0.45, 0.04), 1.0);
    result[7] = vec4(ocean_sun_reflection(up, up, up, vec3(89000.0), 0.95, 0.04), 1.0);
    result[8] = vec4(ocean_sun_reflection(up, up, -up, vec3(89000.0), 0.45, 0.04), 1.0);
}
"#
    );
    let output = gpu.buffer(&[0; 36], false);
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
    assert!(
        values[24] > values[28] * 10.0,
        "land roughness must change the specular lobe"
    );
    assert_eq!(
        &values[32..35],
        &[0.0; 3],
        "no reflection from back-facing sunlight"
    );
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
    for (i, (hi, rel)) in frame
        .anchor_hi_m
        .iter_mut()
        .zip(relative.iter())
        .enumerate()
    {
        *hi = *rel as f32;
        frame.anchor_lo_m[i] = (*rel - f64::from(*hi)) as f32;
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
///
/// The shim keeps every production binding number (0..16) and the 256-wide
/// fan-out `(group.y * 256u + group.x) * 32u` verbatim, so a production
/// binding renumber or fan-out change breaks this test instead of being
/// masked by a rewritten shader. Only mesh/vertex stage attributes (invalid
/// in compute), the `enable wgpu_mesh_shader` directive (needs native mesh
/// support), the `@mesh` entry attribute and the workgroup-only output
/// variable are adapted: per-workgroup outputs move to a NEW storage array
/// at binding 17, indexed by the same fan-out id. Unused fragment bindings
/// (textures, lighting, directory) keep their production numbers with dummy
/// contents; the mesh path never samples them. This tests all lanes/outputs
/// even on adapters without native mesh shaders.
#[cfg(feature = "mesh-shaders")]
#[test]
#[ignore = "requires a wgpu adapter; run with --include-ignored"]
fn gpu_mesh_emission_initializes_every_vertex_and_primitive() {
    let gpu = Gpu::new();
    let mut shader = format!(
        "{}\n{}",
        CBT_RASTER_WGSL.split("@fragment").next().unwrap(),
        include_str!("mesh.wgsl")
    );
    // Compute adapters reject the mesh enable directive; the `build_mesh`
    // body under test does not depend on it.
    shader = shader.replace("enable wgpu_mesh_shader;", "");
    // Strip mesh/vertex-only attributes, which are invalid in a compute
    // entry. Stage IO only: member order and binding numbers are unchanged.
    for attribute in [
        "@builtin(position)",
        "@location(0)",
        "@location(1)",
        "@location(2)",
        "@location(3)",
        "@location(4)",
        "@location(5)",
        "@interpolate(flat)",
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
            "@group(0) @binding(17) var<storage, read_write> mesh_outputs: array<MeshOutput>;",
        )
        .replace("@vertex\nfn vertex(\n    @builtin(vertex_index) vertex_index: u32,\n) -> VertexOutput {\n    return cbt_vertex(vertex_index);\n}", "")
        // Fan-out kept verbatim; per-workgroup outputs are indexed by the
        // same meshlet id (`first / 32u`) instead of workgroup memory.
        .replace("mesh_output.", "mesh_outputs[first / 32u].");
    // Pin the production contract: every buffer/texture binding and the
    // fan-out must survive the shim, so a production renumber fails here.
    for binding in 0..=16 {
        assert!(
            shader.contains(&format!("@binding({binding})")),
            "production binding {binding} must survive the compute shim"
        );
    }
    assert!(
        shader.contains("(group.y * 256u + group.x) * 32u"),
        "production mesh fan-out must survive the compute shim"
    );
    assert!(
        shader.contains("material_directory"),
        "production material bindings must survive the compute shim"
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
    let mut generated = Vec::new();
    for y in 0..33 {
        for x in 0..33 {
            let p = bevy::math::DVec3::from_array(
                anchor.project_body_m([x as f64 / 32., y as f64 / 32.], 120.),
            ) - bevy::math::DVec3::from_array(anchor.anchor_body_m);
            generated.extend(
                [p.x as f32, p.y as f32, p.z as f32, 1., 0., 1., 0., 120.].map(f32::to_bits),
            );
        }
    }
    let vertices = gpu.buffer(&generated, false);
    let mut view_words = vec![0; 116];
    view_words[..16].copy_from_slice(&identity);
    let view = gpu.buffer(&view_words, true);
    let triangle_words: Vec<u32> = (0..513)
        .flat_map(|i| {
            let base = (i / 2 / 32) * 33 + i / 2 % 32;
            if i % 2 == 0 {
                [0, base, base + 1, base + 33]
            } else {
                [0, base + 1, base + 34, base + 33]
            }
        })
        .collect();
    let triangles = gpu.buffer(&triangle_words, false);
    let slots = gpu.buffer(&[11, 1.0f32.to_bits(), 0, 0], false);
    let draw = gpu.buffer(&[512 * 3, 1, 0, 0, 16, 1, 1, 0], false);
    let metadata = gpu.buffer(&[120.0f32.to_bits(), 1.0f32.to_bits(), 2, 0], false);
    // `_padding` is unused padding in production; the meshlet id comes only
    // from the workgroup id via the production fan-out above.
    let params = gpu.buffer(&[1, 1089, radius.to_bits(), 0], true);
    // Dummy contents for fragment-only production bindings the mesh path
    // never samples. Binding numbers stay production-exact.
    let lighting = gpu.buffer(&[0; 40], true);
    let directory = gpu.buffer(&[0; 4], false);
    let dummy_texture = |array_layers: u32| {
        gpu.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("mesh-test-dummy"),
            size: wgpu::Extent3d {
                width: 1,
                height: 1,
                depth_or_array_layers: array_layers,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8Unorm,
            usage: wgpu::TextureUsages::TEXTURE_BINDING,
            view_formats: &[],
        })
    };
    let albedo_texture = dummy_texture(1);
    let albedo_view = albedo_texture.create_view(&Default::default());
    let roughness_texture = dummy_texture(1);
    let roughness_view = roughness_texture.create_view(&Default::default());
    let material_texture = dummy_texture(1);
    let material_view = material_texture.create_view(&wgpu::TextureViewDescriptor {
        dimension: Some(wgpu::TextureViewDimension::D2Array),
        ..Default::default()
    });
    let sampler = gpu.device.create_sampler(&wgpu::SamplerDescriptor {
        mag_filter: wgpu::FilterMode::Linear,
        min_filter: wgpu::FilterMode::Linear,
        ..Default::default()
    });
    // Explicit layout at production binding numbers (0..16 buffers/textures
    // plus the shim output at 17). Compute visibility: the test entry is
    // compute even though production raster uses vertex/fragment stages.
    let storage = |binding: u32, read_only: bool| wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Storage { read_only },
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    };
    let uniform = |binding: u32| wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Uniform,
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    };
    let texture =
        |binding: u32, dimension: wgpu::TextureViewDimension| wgpu::BindGroupLayoutEntry {
            binding,
            visibility: wgpu::ShaderStages::COMPUTE,
            ty: wgpu::BindingType::Texture {
                sample_type: wgpu::TextureSampleType::Float { filterable: true },
                view_dimension: dimension,
                multisampled: false,
            },
            count: None,
        };
    let sampler_entry = |binding: u32| wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
        count: None,
    };
    let bind_layout = gpu
        .device
        .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("mesh-production-bindings"),
            entries: &[
                uniform(0),
                uniform(1),
                storage(2, true),
                storage(3, true),
                uniform(4),
                storage(5, true),
                texture(6, wgpu::TextureViewDimension::D2),
                sampler_entry(7),
                storage(8, true),
                uniform(9),
                texture(10, wgpu::TextureViewDimension::D2),
                storage(11, true),
                texture(12, wgpu::TextureViewDimension::D2Array),
                sampler_entry(13),
                storage(14, true),
                storage(15, true),
                storage(16, true),
                storage(17, false),
            ],
        });
    let module = gpu
        .device
        .create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("cbt-production-mesh-shim"),
            source: wgpu::ShaderSource::Wgsl(shader.into()),
        });
    let pipeline_layout = gpu
        .device
        .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: None,
            bind_group_layouts: &[Some(&bind_layout)],
            immediate_size: 0,
        });
    let pipeline = gpu
        .device
        .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("build_mesh"),
            layout: Some(&pipeline_layout),
            module: &module,
            entry_point: Some("build_mesh"),
            compilation_options: Default::default(),
            cache: None,
        });
    // vec3 members align to 16 bytes: header16, shared vertex80, primitive16.
    let sentinel = 0x7fc00001;
    let words = 4 + 96 * 20 + 32 * 4;
    // One dispatch per draw case with the production fan-out workgroup
    // count (16 meshlets for 512 triangles, 17 with the tail). Each
    // workgroup writes its own `mesh_outputs[meshlet]` slice.
    let run_meshlets = |triangles: &wgpu::Buffer, draw: &wgpu::Buffer, meshlets: u32| -> Vec<u32> {
        let output = gpu.buffer(&vec![sentinel; meshlets as usize * words], false);
        let group = gpu.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: None,
            layout: &bind_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: view.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: matrix.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: vertices.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: metadata.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 4,
                    resource: params.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 5,
                    resource: triangles.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 6,
                    resource: wgpu::BindingResource::TextureView(&albedo_view),
                },
                wgpu::BindGroupEntry {
                    binding: 7,
                    resource: wgpu::BindingResource::Sampler(&sampler),
                },
                wgpu::BindGroupEntry {
                    binding: 8,
                    resource: leaves.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 9,
                    resource: lighting.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 10,
                    resource: wgpu::BindingResource::TextureView(&roughness_view),
                },
                wgpu::BindGroupEntry {
                    binding: 11,
                    resource: frames.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 12,
                    resource: wgpu::BindingResource::TextureView(&material_view),
                },
                wgpu::BindGroupEntry {
                    binding: 13,
                    resource: wgpu::BindingResource::Sampler(&sampler),
                },
                wgpu::BindGroupEntry {
                    binding: 14,
                    resource: slots.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 15,
                    resource: draw.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 16,
                    resource: directory.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 17,
                    resource: output.as_entire_binding(),
                },
            ],
        });
        let mut encoder = gpu.device.create_command_encoder(&Default::default());
        {
            let mut pass = encoder.begin_compute_pass(&Default::default());
            pass.set_pipeline(&pipeline);
            pass.set_bind_group(0, &group, &[]);
            pass.dispatch_workgroups(meshlets, 1, 1);
        }
        gpu.queue.submit([encoder.finish()]);
        gpu.read(&output)
    };
    let check_meshlet = |all: &[u32], meshlet: usize, triangles: usize| {
        let result = &all[meshlet * words..(meshlet + 1) * words];
        assert_eq!(&result[..2], &[96, 32], "meshlet {meshlet} header");
        for i in 0..96 {
            let vertex: Vec<_> = result[4 + i * 20..4 + i * 20 + 7]
                .iter()
                .copied()
                .map(f32::from_bits)
                .collect();
            assert!(
                vertex.iter().all(|v| v.is_finite()),
                "unwritten meshlet {meshlet} vertex {i}"
            );
            let local = triangle_words[(meshlet * 32 + i / 3) * 4 + 1 + i % 3];
            let uv = [(local % 33) as f64 / 32., (local / 33) as f64 / 32.];
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
        for i in 0..32 {
            let tri = &result[4 + 96 * 20 + i * 4..4 + 96 * 20 + i * 4 + 3];
            assert_eq!(tri, [i as u32 * 3, i as u32 * 3 + 1, i as u32 * 3 + 2]);
        }
        let _ = triangles;
    };
    let full = run_meshlets(&triangles, &draw, 16);
    for meshlet in 0..16 {
        check_meshlet(&full, meshlet, 512);
    }
    // An empty classified stream must emit zero counts, not stale geometry.
    let absent = gpu.buffer(&[0; 8], false);
    let empty = run_meshlets(&triangles, &absent, 1);
    assert_eq!(&empty[..2], &[0, 0]);

    // A partial final meshlet must never emit its uninitialized tail.
    let tail_draw = gpu.buffer(&[513 * 3, 1, 0, 0, 17, 1, 1, 0], false);
    let tailed = run_meshlets(&triangles, &tail_draw, 17);
    for meshlet in 0..16 {
        assert_eq!(
            &tailed[meshlet * words..meshlet * words + 2],
            &[96, 32],
            "full meshlet {meshlet} beside the tail"
        );
    }
    let result = &tailed[16 * words..17 * words];
    assert_eq!(&result[..2], &[3, 1]);
    assert_eq!(&result[4 + 96 * 20..4 + 96 * 20 + 3], &[0, 1, 2]);
    assert_eq!(
        result[4 + 18],
        11,
        "shared material slot must survive mesh IO"
    );

    // Skirts use the same radial offset and exclude ocean shading by height.
    let mut skirt_words = triangle_words.clone();
    skirt_words[512 * 4 + 2] |= 0x80000000;
    let skirt_triangles = gpu.buffer(&skirt_words, false);
    let skirted = run_meshlets(&skirt_triangles, &tail_draw, 17);
    let skirt = &skirted[16 * words..17 * words];
    assert_eq!(f32::from_bits(skirt[4 + 20 + 15]), 1e9);
    let distance = (0..3)
        .map(|axis| {
            let delta =
                f32::from_bits(skirt[4 + 20 + axis]) - f32::from_bits(result[4 + 20 + axis]);
            delta * delta
        })
        .sum::<f32>()
        .sqrt();
    assert!(
        (distance - 256.).abs() < 0.001,
        "bounded skirt depth: {distance}"
    );
}

#[test]
#[ignore = "requires a wgpu adapter; run with --ignored"]
fn gpu_material_directory_uses_world_addresses_on_all_cube_faces() {
    let gpu = Gpu::new();
    let mut inputs = Vec::new();
    let mut entries = Vec::new();
    for face in 0..6 {
        for level in [0, 17] {
            let (x, y) = if level == 0 { (0, 0) } else { (65537, 65539) };
            let key = crate::precision::TileKey::new(face, level, x, y).unwrap();
            let anchor = crate::precision::TileAnchor::new(key, 3_200_000.).unwrap();
            let d = anchor.project_body_m([0.25, 0.75], 0.);
            inputs.extend(
                [
                    d[0] as f32 / 3_200_000.,
                    d[1] as f32 / 3_200_000.,
                    d[2] as f32 / 3_200_000.,
                ]
                .map(f32::to_bits),
            );
            inputs.push(level as u32);
            let mut id = 8u64 + u64::from(face);
            for bit in (0..level).rev() {
                id = (id << 2) | u64::from(((x >> bit) & 1) * 2 + ((y >> bit) & 1));
            }
            entries.push(crate::material_cache::SlotEntry {
                node_id: id,
                slot: entries.len() as u32,
                generation: 1,
            });
        }
    }
    let words: Vec<_> = crate::material_cache::material_directory(entries.iter().copied())
        .into_iter()
        .flatten()
        .collect();
    let directory = gpu.buffer(&words, false);
    let input = gpu.buffer(&inputs, false);
    let output = gpu.buffer(&vec![0; entries.len() * 4], false);
    let shader = format!(
        "{}\n{}",
        include_str!("material_sample.wgsl")
            .split("// Gradients")
            .next()
            .unwrap()
            .replace("@binding(16)", "@binding(0)"),
        r#"
@group(0) @binding(1) var<storage, read> points: array<vec4<u32>>;
@group(0) @binding(2) var<storage, read_write> result: array<vec4<u32>>;
@compute @workgroup_size(64)
fn check(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= arrayLength(&points)) { return; }
    let point = points[gid.x];
    let address = material_address(bitcast<vec3<f32>>(point.xyz));
    let id = material_node(address.face, vec2<u32>(address.uv * exp2(f32(point.w))), point.w);
    result[gid.x] = vec4(address.face, id, material_layer(id));
}
"#
    );
    gpu.dispatch(
        &shader,
        &[("check", 1)],
        &[&directory, &input, &output],
        &[],
        &[2],
    );
    for (i, record) in gpu.read(&output).as_chunks::<4>().0.iter().enumerate() {
        assert_eq!(
            *record,
            [
                (i / 2) as u32,
                entries[i].node_id as u32,
                (entries[i].node_id >> 32) as u32,
                i as u32
            ]
        );
    }
}

#[test]
#[ignore = "requires a wgpu adapter; run with --ignored"]
fn gpu_material_missing_neighbours_fade_without_erasing_complete_edges() {
    let gpu = Gpu::new();
    let shader = format!(
        "{}\n{}",
        include_str!("material_sample.wgsl")
            .split("// Gradients")
            .next()
            .unwrap()
            .replace("@binding(16)", "@binding(0)"),
        r#"
@group(0) @binding(1) var<storage, read> points: array<vec4<f32>>;
@group(0) @binding(2) var<storage, read_write> result: array<u32>;
@compute @workgroup_size(64)
fn check(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= arrayLength(&points)) { return; }
    let point = points[gid.x];
    result[gid.x] = bitcast<u32>(material_resident_weight(
        MaterialAddress(point.xy, u32(point.z)), u32(point.w), 0.032));
}
"#,
    );
    // One level-2 page, with samples at left edge, centre, corner and
    // progressively inside the missing-neighbour transition band.
    let points: Vec<u32> = [0.0f32, 0.008, 0.016, 0.024, 0.032, 0.5]
        .into_iter()
        .flat_map(|u| [(1.0 + u) / 4.0, 1.5 / 4.0, 4.0, 2.0])
        .chain([0.25, 0.25, 4.0, 2.0])
        // A face edge must also see the missing page on the adjacent face.
        .chain([0.0, 0.375, 4.0, 2.0])
        .map(f32::to_bits)
        .collect();
    let input = gpu.buffer(&points, false);
    let output = gpu.buffer(&[0; 8], false);
    let id = |face: u64, x: u64, y: u64| {
        let mut id = 8u64 + face;
        for bit in (0..2).rev() {
            id = (id << 2) | (((x >> bit) & 1) * 2 + ((y >> bit) & 1));
        }
        id
    };
    for complete in [false, true] {
        let entries: Vec<_> = (0u64..6)
            .flat_map(|face| (0u64..4).flat_map(move |x| (0u64..4).map(move |y| (face, x, y))))
            .filter(|&(face, x, y)| complete || (face == 4 && x == 1 && y == 1))
            .map(|(face, x, y)| crate::material_cache::SlotEntry {
                node_id: id(face, x, y),
                slot: 0,
                generation: 1,
            })
            .collect();
        let words: Vec<_> = crate::material_cache::material_directory(entries)
            .into_iter()
            .flatten()
            .collect();
        let directory = gpu.buffer(&words, false);
        gpu.dispatch(
            &shader,
            &[("check", 1)],
            &[&directory, &input, &output],
            &[],
            &[2],
        );
        let weights: Vec<_> = gpu.read(&output).into_iter().map(f32::from_bits).collect();
        if complete {
            assert!(weights.iter().all(|w| (*w - 1.0).abs() < 1e-6));
        } else {
            assert_eq!(weights[0], 0.0, "missing neighbour shares coarse edge");
            assert_eq!(weights[5], 1.0, "interior preserves detail");
            assert_eq!(weights[6], 0.0, "missing diagonal shares coarse corner");
            assert_eq!(weights[7], 0.0, "cube-face edge also fades");
            for pair in weights[..6].windows(2) {
                assert!(pair[0] <= pair[1], "monotone residency transition");
            }
            assert!((weights[2] - 0.5).abs() < 1e-5);
        }
    }
}

/// Full game-data roundtrip on hardware: a real `CbtMaterialPage` (linear-
/// light mip averaging included) through microstore encode plus GPU decode
/// for every mip level and every channel. This is the integration proof
/// the prototype was built for: game constructor bytes on a real adapter.
#[ignore = "requires a wgpu adapter; run with --ignored --nocapture"]
#[test]
fn game_material_mips_decode_on_hardware() {
    use crate::material_microstore::rock_rgba;
    use thessa_microstore_core::{EncodeMode, EncodedPage, ScalarField};
    use thessa_rcbt_wgpu::microstore::MicrostoreDecode;

    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
    let adapter = pollster::block_on(instance.request_adapter(&Default::default()))
        .expect("material GPU test requires a wgpu adapter");
    eprintln!("material regression adapter: {:?}", adapter.get_info());
    let (device, queue) = pollster::block_on(adapter.request_device(&Default::default())).unwrap();
    let decoder = MicrostoreDecode::new(std::sync::Arc::new(device), std::sync::Arc::new(queue));

    let rgba = rock_rgba(crate::material_pages::MATERIAL_PAGE_SIZE, 0x9A7E);
    let page = crate::material_pages::CbtMaterialPage::from_rgba8(rgba).expect("real page");
    assert_eq!(page.mips.len(), 8);
    let mut total_wire = 0usize;
    let mut worst = 0u8;
    for (level, mip) in page.mips.iter().enumerate() {
        let size = crate::material_pages::MATERIAL_PAGE_SIZE >> level;
        assert_eq!(
            mip.len(),
            size as usize * size as usize * 4,
            "level {level}"
        );
        for channel in 0..4 {
            let plane: Vec<u8> = mip
                .as_chunks::<4>()
                .0
                .iter()
                .map(|px| px[channel])
                .collect();
            let field = ScalarField::new(size, size, plane).expect("plane extent");
            let encoded = EncodedPage::encode(&field, EncodeMode::Adaptive { max_abs_error: 2.0 });
            total_wire += encoded.encoded_bytes();
            let gpu = decoder.decode_page(&encoded).expect("gpu decode");
            let cpu = encoded.decode();
            assert_eq!(gpu.len(), cpu.data.len(), "level {level} ch {channel} len");
            for (g, c) in gpu.iter().zip(cpu.data.iter()) {
                worst = worst.max(g.abs_diff(*c));
            }
            assert_eq!(gpu, cpu.data, "level {level} ch {channel} bit-exact");
        }
    }
    eprintln!("8 game mips x 4 channels on hardware: {total_wire} B wire, worst drift {worst}");
}

/// Compact storage path on hardware: the exact `encode_material_level`
/// (ColorPage RGB + roughness) + `MaterialArray::decode_material_levels`
/// pre-decode used by `material_storage = "microstore_compact"`, with every
/// underlying `EncodedPage` verified bit-exact against the GPU decoder.
/// Sampling stays identical because the texture upload sees only the
/// decoded RGBA; this pins the residency bytes are GPU-decodable.
#[ignore = "requires a wgpu adapter; run with --ignored --nocapture"]
#[test]
fn microstore_compact_levels_match_gpu_decode() {
    use crate::material_microstore::{encode_material_level, rock_rgba};
    use thessa_rcbt_wgpu::microstore::MicrostoreDecode;

    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
    let adapter = pollster::block_on(instance.request_adapter(&Default::default()))
        .expect("compact storage GPU test requires a wgpu adapter");
    eprintln!("compact storage adapter: {:?}", adapter.get_info());
    let (device, queue) = pollster::block_on(adapter.request_device(&Default::default())).unwrap();
    let decoder = MicrostoreDecode::new(std::sync::Arc::new(device), std::sync::Arc::new(queue));

    let rgba = rock_rgba(crate::material_pages::MATERIAL_PAGE_SIZE, 0xC0FFEE);
    let page = crate::material_pages::CbtMaterialPage::from_rgba8(rgba).expect("real page");
    let mut wire = 0usize;
    let mut worst_vs_orig = 0u8;
    for (level, mip) in page.mips.iter().enumerate() {
        let size = crate::material_pages::MATERIAL_PAGE_SIZE >> level;
        let encoded = encode_material_level(mip, size, size, 2.0).expect("encodes");
        wire += encoded.encoded_bytes();
        // Every stored page must GPU-decode bit-exact vs CPU.
        let cpu_planes = [
            encoded.color.channels[0].decode(),
            encoded.color.channels[1].decode(),
            encoded.color.channels[2].decode(),
            encoded.roughness.decode(),
        ];
        let gpu_planes = [
            decoder
                .decode_page(&encoded.color.channels[0])
                .expect("gpu r"),
            decoder
                .decode_page(&encoded.color.channels[1])
                .expect("gpu g"),
            decoder
                .decode_page(&encoded.color.channels[2])
                .expect("gpu b"),
            decoder.decode_page(&encoded.roughness).expect("gpu a"),
        ];
        for (ch, (gpu, cpu)) in gpu_planes.iter().zip(cpu_planes.iter()).enumerate() {
            assert_eq!(gpu.len(), cpu.data.len(), "level {level} ch {ch} len");
            assert_eq!(gpu, &cpu.data, "level {level} ch {ch} bit-exact");
        }
        // The pre-decode upload path interleaves those exact planes.
        let decoded = material_render::MaterialArray::decode_material_levels(&[encoded]);
        assert_eq!(decoded.len(), 1);
        let back = &decoded[0];
        assert_eq!(back.len(), mip.len(), "level {level} rgba len");
        for (i, (o, d)) in mip.iter().zip(back.iter()).enumerate() {
            worst_vs_orig = worst_vs_orig.max(o.abs_diff(*d));
            assert!(o.abs_diff(*d) <= 2, "level {level} byte {i}: {o} vs {d}");
        }
    }
    let raw: usize = page.mips.iter().map(Vec::len).sum();
    eprintln!("compact storage on hardware: {wire} B wire vs {raw} B raw, worst {worst_vs_orig}");
    assert!(wire < raw, "compact must beat raw RGBA");
}

#[test]
#[ignore = "requires a wgpu adapter; run with --ignored --nocapture"]
fn raster_surface_shader_validates_on_hardware() {
    let gpu = Gpu::new();
    // Full raster module (vertex + fragment), including the cloud-shadow
    // and city-lights blocks. wgpu reports naga validation failures
    // through the error scope, so pop it and fail closed.
    let scope = gpu.device.push_error_scope(wgpu::ErrorFilter::Validation);
    let _module = gpu
        .device
        .create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("cbt-raster-surface-validation"),
            source: wgpu::ShaderSource::Wgsl(super::CBT_RASTER_WGSL.into()),
        });
    let err = pollster::block_on(scope.pop());
    assert!(err.is_none(), "raster WGSL validation failed: {err:?}");
}

#[test]
#[ignore = "requires a wgpu adapter; run with --ignored --nocapture"]
fn cloud_optical_kernel_matches_cpu_reference_on_hardware() {
    let gpu = Gpu::new();

    // Sliced directly from the production WGSL shader assets/shaders/cloud_volume.wgsl:
    // Ensures the hardware test evaluates the exact production shader functions without duplication.
    const PROD_WGSL: &str = include_str!("../../../assets/shaders/cloud_volume.wgsl");
    let start_idx = PROD_WGSL
        .find("fn powder_scatter_term")
        .expect("powder_scatter_term in cloud_volume.wgsl");
    let end_idx = PROD_WGSL
        .find("fn hash12")
        .expect("hash12 in cloud_volume.wgsl");
    let prod_functions = &PROD_WGSL[start_idx..end_idx];

    let shader = format!(
        r#"
@group(0) @binding(0) var<storage, read> in_params: array<f32>;
@group(0) @binding(1) var<storage, read_write> out_results: array<f32>;

{prod_functions}

@compute @workgroup_size(64)
fn test_optical_kernel(@builtin(global_invocation_id) gid: vec3<u32>) {{
    let idx = gid.x;
    if (idx >= arrayLength(&in_params)) {{
        return;
    }}
    let param = in_params[idx];
    let bp = beer_powder(param);
    let ps = powder_scatter_term(param);
    let phase = dual_lobe_phase(param * 0.2 - 1.0);
    let hgrad = convective_cumulus_gradient(param * 0.1);

    let base_out = idx * 4u;
    out_results[base_out + 0u] = bp;
    out_results[base_out + 1u] = ps;
    out_results[base_out + 2u] = phase;
    out_results[base_out + 3u] = hgrad;
}}
"#
    );

    let num_samples = 256;
    let inputs: Vec<f32> = (0..num_samples)
        .map(|i| (i as f32) / ((num_samples - 1) as f32) * 10.0)
        .collect();
    let in_words: Vec<u32> = inputs.iter().map(|&x| x.to_bits()).collect();
    let out_words: Vec<u32> = vec![0u32; num_samples * 4];

    let in_buf = gpu.buffer(&in_words, false);
    let out_buf = gpu.buffer(&out_words, false);

    gpu.dispatch(
        &shader,
        &[("test_optical_kernel", 4)],
        &[&in_buf, &out_buf],
        &[],
        &[1],
    );

    let readback_words = gpu.read(&out_buf);
    let readback_floats: Vec<f32> = readback_words
        .iter()
        .map(|&w| f32::from_bits(w))
        .collect();

    // Verify GPU hardware results directly against production Rust oracle functions in thessa_atmosphere::clouds:
    for (i, &param) in inputs.iter().enumerate() {
        let base = i * 4;
        let gpu_bp = readback_floats[base];
        let gpu_ps = readback_floats[base + 1];
        let gpu_phase = readback_floats[base + 2];
        let gpu_hgrad = readback_floats[base + 3];

        // 1. Beer-powder
        let cpu_bp = thessa_atmosphere::clouds::beer_powder(param as f64) as f32;
        assert!(
            (gpu_bp - cpu_bp).abs() < 1e-5,
            "sample {i} param {param}: gpu_bp {gpu_bp} vs cpu_bp {cpu_bp}"
        );

        // 2. Powder scatter
        let cpu_ps = thessa_atmosphere::clouds::powder_scatter_term(param as f64) as f32;
        assert!(
            (gpu_ps - cpu_ps).abs() < 1e-5,
            "sample {i} param {param}: gpu_ps {gpu_ps} vs cpu_ps {cpu_ps}"
        );

        // 3. Dual-lobe phase
        let cos_theta = param * 0.2 - 1.0;
        let cpu_phase = thessa_atmosphere::clouds::dual_lobe_phase(cos_theta as f64) as f32;
        assert!(
            (gpu_phase - cpu_phase).abs() < 1e-4,
            "sample {i} cos_theta {cos_theta}: gpu_phase {gpu_phase} vs cpu_phase {cpu_phase}"
        );

        // 4. Convective gradient
        let h01 = param * 0.1;
        let cpu_hgrad = thessa_atmosphere::clouds::convective_cumulus_gradient(h01 as f64) as f32;
        assert!(
            (gpu_hgrad - cpu_hgrad).abs() < 1e-5,
            "sample {i} h01 {h01}: gpu_hgrad {gpu_hgrad} vs cpu_hgrad {cpu_hgrad}"
        );
    }
}

#[test]
#[ignore = "requires a wgpu adapter; run with --ignored --nocapture"]
fn cloud_noise_kernel_matches_cpu_perlin_on_hardware() {
    let gpu = Gpu::new();

    // Sliced directly from the production WGSL shader assets/shaders/cloud_volume.wgsl:
    // Extracts hash33, hash_grad, and perlin3d functions.
    const PROD_WGSL: &str = include_str!("../../../assets/shaders/cloud_volume.wgsl");
    let start_idx = PROD_WGSL
        .find("fn hash33")
        .expect("hash33 in cloud_volume.wgsl");
    let end_idx = PROD_WGSL
        .find("const ROT_OCTAVE")
        .expect("ROT_OCTAVE in cloud_volume.wgsl");
    let prod_functions = &PROD_WGSL[start_idx..end_idx];

    let shader = format!(
        r#"
@group(0) @binding(0) var<storage, read> in_points: array<vec4<f32>>;
@group(0) @binding(1) var<storage, read_write> out_results: array<vec4<f32>>;

{prod_functions}

@compute @workgroup_size(64)
fn test_noise_kernel(@builtin(global_invocation_id) gid: vec3<u32>) {{
    let idx = gid.x;
    if (idx >= arrayLength(&in_points)) {{
        return;
    }}
    let p = in_points[idx].xyz;
    let n = perlin3d(p);
    let h = hash33(p);
    let g = hash_grad(p);
    // Write out perlin3d in vec4 0, and hash_grad in vec4 1:
    let base = idx * 2u;
    out_results[base + 0u] = vec4<f32>(n, h.x, h.y, h.z);
    out_results[base + 1u] = vec4<f32>(g.x, g.y, g.z, 0.0);
}}
"#
    );

    // Replicate CPU reference oracle using fract_gl() strictly matching WGSL fract(x) = x - floor(x):
    fn cpu_hash33(p: glam::Vec3) -> glam::Vec3 {
        let mut q = (p * glam::Vec3::new(0.1031, 0.1030, 0.0973)).fract_gl();
        let dot_val = q.dot(glam::Vec3::new(q.y, q.z, q.x) + glam::Vec3::splat(33.33));
        q += glam::Vec3::splat(dot_val);
        let val = (glam::Vec3::new(
            (q.x + q.y) * q.z,
            (q.x + q.z) * q.y,
            (q.y + q.z) * q.x,
        ))
        .fract_gl();
        val * 2.0 - glam::Vec3::ONE
    }

    fn cpu_hash_grad(p: glam::Vec3) -> glam::Vec3 {
        let h = cpu_hash33(p);
        (h + glam::Vec3::splat(1e-5)).normalize()
    }

    fn cpu_perlin3d(p: glam::Vec3) -> f32 {
        let i = p.floor();
        let f = p.fract_gl();
        let u = f * f * f * (f * (f * 6.0 - 15.0) + 10.0);

        let g000 = cpu_hash_grad(i + glam::Vec3::new(0.0, 0.0, 0.0));
        let g100 = cpu_hash_grad(i + glam::Vec3::new(1.0, 0.0, 0.0));
        let g010 = cpu_hash_grad(i + glam::Vec3::new(0.0, 1.0, 0.0));
        let g110 = cpu_hash_grad(i + glam::Vec3::new(1.0, 1.0, 0.0));
        let g001 = cpu_hash_grad(i + glam::Vec3::new(0.0, 0.0, 1.0));
        let g101 = cpu_hash_grad(i + glam::Vec3::new(1.0, 0.0, 1.0));
        let g011 = cpu_hash_grad(i + glam::Vec3::new(0.0, 1.0, 1.0));
        let g111 = cpu_hash_grad(i + glam::Vec3::new(1.0, 1.0, 1.0));

        let v000 = g000.dot(f - glam::Vec3::new(0.0, 0.0, 0.0));
        let v100 = g100.dot(f - glam::Vec3::new(1.0, 0.0, 0.0));
        let v010 = g010.dot(f - glam::Vec3::new(0.0, 1.0, 0.0));
        let v110 = g110.dot(f - glam::Vec3::new(1.0, 1.0, 0.0));
        let v001 = g001.dot(f - glam::Vec3::new(0.0, 0.0, 1.0));
        let v101 = g101.dot(f - glam::Vec3::new(1.0, 0.0, 1.0));
        let v011 = g011.dot(f - glam::Vec3::new(0.0, 1.0, 1.0));
        let v111 = g111.dot(f - glam::Vec3::new(1.0, 1.0, 1.0));

        let vx00 = v000 + u.x * (v100 - v000);
        let vx10 = v010 + u.x * (v110 - v010);
        let vx01 = v001 + u.x * (v101 - v001);
        let vx11 = v011 + u.x * (v111 - v011);
        let vxy0 = vx00 + u.y * (vx10 - vx00);
        let vxy1 = vx01 + u.y * (vx11 - vx01);
        vxy0 + u.z * (vxy1 - vxy0)
    }

    let num_samples = 256;
    let mut test_points: Vec<glam::Vec3> = Vec::with_capacity(num_samples);
    for i in 0..num_samples {
        // Test diverse ranges including planetary scale:
        // Samples span [-50, +50] km local cells up to [-1500, +1500] km planet-wide chords:
        let scale = if i < 128 { 0.001 } else { 0.03 };
        let x = ((i * 1973 % 100_000) as f32) * scale - 50.0 * (scale * 1000.0);
        let y = ((i * 9277 % 100_000) as f32) * scale - 50.0 * (scale * 1000.0);
        let z = ((i * 2699 % 100_000) as f32) * scale - 50.0 * (scale * 1000.0);
        test_points.push(glam::Vec3::new(x, y, z));
    }

    let in_words: Vec<u32> = test_points
        .iter()
        .flat_map(|p| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits(), 0u32])
        .collect();
    let out_words: Vec<u32> = vec![0u32; num_samples * 8];

    let in_buf = gpu.buffer(&in_words, false);
    let out_buf = gpu.buffer(&out_words, false);

    gpu.dispatch(
        &shader,
        &[("test_noise_kernel", 4)],
        &[&in_buf, &out_buf],
        &[],
        &[1],
    );

    let readback_words = gpu.read(&out_buf);
    let readback_floats: Vec<f32> = readback_words
        .iter()
        .map(|&w| f32::from_bits(w))
        .collect();

    let mut max_diff_perlin = 0.0f32;
    let mut max_diff_hash = 0.0f32;
    let mut max_diff_grad = 0.0f32;
    let mut worst_sample = 0;
    let mut worst_gpu_p = 0.0f32;
    let mut worst_cpu_p = 0.0f32;

    for (i, &p) in test_points.iter().enumerate() {
        let base = i * 8;
        let gpu_perlin = readback_floats[base];
        let gpu_hash = glam::Vec3::new(
            readback_floats[base + 1],
            readback_floats[base + 2],
            readback_floats[base + 3],
        );
        let gpu_grad = glam::Vec3::new(
            readback_floats[base + 4],
            readback_floats[base + 5],
            readback_floats[base + 6],
        );

        assert!(gpu_perlin.is_finite(), "sample {i} point {p:?}: GPU perlin is non-finite {gpu_perlin}");
        assert!(gpu_hash.is_finite(), "sample {i} point {p:?}: GPU hash is non-finite {gpu_hash:?}");
        assert!(gpu_grad.is_finite(), "sample {i} point {p:?}: GPU grad is non-finite {gpu_grad:?}");

        let cpu_perlin = cpu_perlin3d(p);
        let cpu_hash = cpu_hash33(p);
        let cpu_grad = cpu_hash_grad(p);

        assert!(cpu_perlin.is_finite(), "sample {i} point {p:?}: CPU perlin is non-finite {cpu_perlin}");

        let diff_p = (gpu_perlin - cpu_perlin).abs();
        let diff_h = (gpu_hash - cpu_hash).abs().max_element();
        let diff_g = (gpu_grad - cpu_grad).abs().max_element();

        if diff_p > max_diff_perlin {
            max_diff_perlin = diff_p;
            worst_sample = i;
            worst_gpu_p = gpu_perlin;
            worst_cpu_p = cpu_perlin;
        }
        if diff_h > max_diff_hash {
            max_diff_hash = diff_h;
        }
        if diff_g > max_diff_grad {
            max_diff_grad = diff_g;
        }
    }

    let worst_p = test_points[worst_sample];
    eprintln!(
        "cloud_noise_kernel diagnostic: max_diff_perlin={max_diff_perlin} (sample {worst_sample} point {worst_p:?}, gpu={worst_gpu_p}, cpu={worst_cpu_p}), max_diff_hash={max_diff_hash}, max_diff_grad={max_diff_grad}"
    );

    // Across planetary chords (up to +/- 1500 km), hash33 multiplies values of order ~5000.
    // In IEEE 754 f32, values near 5000 allocate 13 bits of mantissa to the integer part,
    // leaving 11 bits for fract() (resolution 2^-11 = 1/2048).
    // An 8-ULP difference between GPU FMA (v_fma_f32) and CPU un-fused math yields exactly
    // 8 * 2^-11 = 2^-8 = 1/256 in fract, and 2^-7 = 0.0078125 in hash33.
    // Propagated through quintic interpolation, the maximum bound between GPU and CPU is < 0.010:
    assert!(
        max_diff_perlin < 0.010,
        "max_diff_perlin {max_diff_perlin} exceeds 0.010 threshold at point {worst_p:?}"
    );
}

fn get_cloud_volume_wgsl_slice() -> &'static str {
    const PROD_WGSL: &str = include_str!("../../../assets/shaders/cloud_volume.wgsl");
    let start_idx = PROD_WGSL
        .find("const CLOUD_MARCH_MAX")
        .expect("const CLOUD_MARCH_MAX in cloud_volume.wgsl");
    let end_idx = PROD_WGSL
        .find("@fragment")
        .expect("@fragment in cloud_volume.wgsl");
    &PROD_WGSL[start_idx..end_idx]
}

fn create_gpu_coverage_texture(
    gpu: &Gpu,
    w: u32,
    h: u32,
    tex_bytes: &[u8],
) -> (wgpu::Texture, wgpu::TextureView, wgpu::Sampler) {
    let texture = gpu.device.create_texture(&wgpu::TextureDescriptor {
        label: Some("test-coverage-tex"),
        size: wgpu::Extent3d {
            width: w,
            height: h,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba8Unorm,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });

    gpu.queue.write_texture(
        wgpu::TexelCopyTextureInfo {
            texture: &texture,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        tex_bytes,
        wgpu::TexelCopyBufferLayout {
            offset: 0,
            bytes_per_row: Some(w * 4),
            rows_per_image: Some(h),
        },
        wgpu::Extent3d {
            width: w,
            height: h,
            depth_or_array_layers: 1,
        },
    );

    let view = texture.create_view(&Default::default());
    let sampler = gpu.device.create_sampler(&wgpu::SamplerDescriptor {
        address_mode_u: wgpu::AddressMode::Repeat,
        address_mode_v: wgpu::AddressMode::ClampToEdge,
        mag_filter: wgpu::FilterMode::Linear,
        min_filter: wgpu::FilterMode::Linear,
        ..Default::default()
    });

    (texture, view, sampler)
}

#[allow(clippy::too_many_arguments)]
fn pack_cloud_uniform_words(
    planet_radius_m: f32,
    bot_radius: f32,
    top_radius: f32,
    extinction: f32,
    steps: f32,
    sun_dir: glam::Vec3,
    sun_vis: f32,
    opacity: f32,
    drift: f32,
    sun_span: f32,
    detail_amp: f32,
    march_top: f32,
    pathtrace: bool,
) -> Vec<u32> {
    let mut words = vec![0u32; 64];
    let floats: [f32; 24] = [
        0.0, 0.0, 0.0, planet_radius_m,
        bot_radius, top_radius, extinction, steps,
        sun_dir.x, sun_dir.y, sun_dir.z, sun_vis,
        0.0, 0.0, 0.0, 1.0,
        opacity, drift, sun_span, detail_amp,
        0.0, 0.05, march_top, if pathtrace { 1.0 } else { 0.0 },
    ];
    for (i, &f) in floats.iter().enumerate() {
        words[i] = f.to_bits();
    }
    words
}

#[allow(clippy::too_many_arguments)]
fn execute_cloud_compute(
    gpu: &Gpu,
    shader_source: &str,
    entry_point: &str,
    uniform_buf: &wgpu::Buffer,
    coverage_view: &wgpu::TextureView,
    coverage_sampler: &wgpu::Sampler,
    in_buf: &wgpu::Buffer,
    out_buf: &wgpu::Buffer,
    workgroups: u32,
) {
    let uniform_layout = |binding: u32| wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Uniform,
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    };
    let storage_layout = |binding: u32, read_only: bool| wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Storage { read_only },
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    };
    let texture_layout = |binding: u32| wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Texture {
            sample_type: wgpu::TextureSampleType::Float { filterable: true },
            view_dimension: wgpu::TextureViewDimension::D2,
            multisampled: false,
        },
        count: None,
    };
    let sampler_layout = |binding: u32| wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
        count: None,
    };

    let bind_layout = gpu
        .device
        .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("cloud-bind-layout"),
            entries: &[
                uniform_layout(0),
                texture_layout(1),
                sampler_layout(2),
                storage_layout(3, true),
                storage_layout(4, false),
            ],
        });

    let bind_group = gpu.device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("cloud-bind-group"),
        layout: &bind_layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform_buf.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::TextureView(coverage_view),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: wgpu::BindingResource::Sampler(coverage_sampler),
            },
            wgpu::BindGroupEntry {
                binding: 3,
                resource: in_buf.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 4,
                resource: out_buf.as_entire_binding(),
            },
        ],
    });

    let module = gpu
        .device
        .create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some(entry_point),
            source: wgpu::ShaderSource::Wgsl(shader_source.into()),
        });
    let pipeline_layout = gpu
        .device
        .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: None,
            bind_group_layouts: &[Some(&bind_layout)],
            immediate_size: 0,
        });
    let pipeline = gpu
        .device
        .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some(entry_point),
            layout: Some(&pipeline_layout),
            module: &module,
            entry_point: Some(entry_point),
            compilation_options: Default::default(),
            cache: None,
        });

    let mut encoder = gpu.device.create_command_encoder(&Default::default());
    {
        let mut pass = encoder.begin_compute_pass(&Default::default());
        pass.set_pipeline(&pipeline);
        pass.set_bind_group(0, &bind_group, &[]);
        pass.dispatch_workgroups(workgroups, 1, 1);
    }
    gpu.queue.submit([encoder.finish()]);
}

fn build_cloud_test_wgsl() -> String {
    let prod_slice = get_cloud_volume_wgsl_slice();
    format!(
        r#"
struct CloudUniforms {{
    center_radius: vec4<f32>,
    slab: vec4<f32>,
    sun: vec4<f32>,
    deck_to_local: vec4<f32>,
    params: vec4<f32>,
    shape: vec4<f32>,
}}

@group(0) @binding(0) var<uniform> cloud: CloudUniforms;
@group(0) @binding(1) var coverage_tex: texture_2d<f32>;
@group(0) @binding(2) var coverage_sampler: sampler;
@group(0) @binding(3) var<storage, read> in_samples: array<vec4<f32>>;
@group(0) @binding(4) var<storage, read_write> out_results: array<vec4<f32>>;

{prod_slice}

@compute @workgroup_size(64)
fn test_slab_density(@builtin(global_invocation_id) gid: vec3<u32>) {{
    let idx = gid.x;
    if (idx >= arrayLength(&in_samples)) {{
        return;
    }}
    let s = in_samples[idx];
    let p_norm = s.xyz;
    let step_km = s.w;
    let drift = vec2<f32>(cloud.params.y, cloud.params.y * 0.7);
    let weather = vec3<f32>(0.5, 0.5, 0.5);

    let d_full = slab_density(p_norm, drift, weather, step_km, true);
    let d_fast = slab_density(p_norm, drift, weather, step_km, false);

    out_results[idx] = vec4<f32>(d_full, d_fast, weather.x, 0.0);
}}

@compute @workgroup_size(64)
fn test_raymarch(@builtin(global_invocation_id) gid: vec3<u32>) {{
    let idx = gid.x;
    let total_rays = arrayLength(&in_samples) / 2u;
    if (idx >= total_rays) {{
        return;
    }}

    let ro = in_samples[idx * 2u].xyz;
    let rd_param = in_samples[idx * 2u + 1u];
    let rd = rd_param.xyz;
    let steps_override = rd_param.w;

    let center = cloud.center_radius.xyz;
    let radius = max(cloud.center_radius.w, 1e-6);
    let r_cam = length(ro);
    let bot = cloud.slab.x;
    let top = cloud.slab.y;
    let march_top = cloud.shape.z;
    let ext = max(cloud.slab.z, 0.0);
    let steps = select(max(i32(cloud.slab.w + 0.5), 1), max(i32(steps_override + 0.5), 1), steps_override > 0.5);
    let drift = vec2<f32>(cloud.params.y, cloud.params.y * 0.32);

    let b = dot(ro, rd);

    let disc_top = b * b - (r_cam * r_cam - march_top * march_top);
    if (disc_top < 0.0) {{
        out_results[idx * 2u] = vec4<f32>(0.0, 0.0, 0.0, 0.0);
        out_results[idx * 2u + 1u] = vec4<f32>(0.0, 0.0, 0.0, 0.0);
        return;
    }}
    let sq_top = sqrt(disc_top);
    let t_top0 = -b - sq_top;
    let t_top1 = -b + sq_top;
    if (t_top1 <= 0.0) {{
        out_results[idx * 2u] = vec4<f32>(0.0, 0.0, 0.0, 0.0);
        out_results[idx * 2u + 1u] = vec4<f32>(0.0, 0.0, 0.0, 0.0);
        return;
    }}

    var t0 = max(t_top0, 0.0);
    var t1 = t_top1;

    let disc_bot = b * b - (r_cam * r_cam - bot * bot);
    if (disc_bot > 0.0) {{
        let sq_bot = sqrt(disc_bot);
        let t_bot0 = -b - sq_bot;
        let t_bot1 = -b + sq_bot;

        if (r_cam < bot) {{
            if (t_bot1 > 0.0) {{
                t0 = max(t0, t_bot1);
            }}
        }} else {{
            if (t_bot0 > 0.0) {{
                t1 = min(t1, t_bot0);
            }}
        }}
    }}

    let disc_planet = b * b - (r_cam * r_cam - 1.0);
    if (disc_planet > 0.0) {{
        let t_planet0 = -b - sqrt(disc_planet);
        if (t_planet0 > 1e-5) {{
            t1 = min(t1, t_planet0);
        }} else if (b <= 0.0) {{
            out_results[idx * 2u] = vec4<f32>(0.0, 0.0, 0.0, 0.0);
            out_results[idx * 2u + 1u] = vec4<f32>(0.0, 0.0, 0.0, 0.0);
            return;
        }}
    }}

    let geometric_tangent_chord = 2.0 * sqrt(max(march_top * march_top - bot * bot, 1e-4));
    let r_low = march_top;
    let r_high = march_top + 0.05;
    let altitude_weight = smoothstep(r_low, r_high, r_cam);
    let max_chord = mix(min(geometric_tangent_chord, 0.025), geometric_tangent_chord, altitude_weight);
    t1 = min(t1, t0 + max_chord);

    let chord = max(t1 - t0, 0.0);
    let radius_km = radius * 0.001;
    let chord_km = chord * radius_km;

    if (t1 <= t0 + 1e-5) {{
        out_results[idx * 2u] = vec4<f32>(0.0, 0.0, 0.0, 0.0);
        out_results[idx * 2u + 1u] = vec4<f32>(t0, t1, chord_km, 0.0);
        return;
    }}

    let sun_dir = normalize(cloud.sun.xyz + vec3<f32>(1e-9, 0.0, 0.0));
    let sun_vis = clamp(cloud.sun.w, 0.0, 1.0);
    let cos_theta = dot(rd, sun_dir);
    let thick = max(top - bot, 1e-6);
    let inv_radius = 1.0 / radius;
    let shadow_span = max(cloud.params.z / 0.8, 0.05);

    let w_entry = weather_at(ro + rd * t0, drift);
    let w_exit = weather_at(ro + rd * t1, drift);
    var transmittance = 1.0;
    var scat = vec3<f32>(0.0);

    let dt = chord / f32(steps);
    let step_km = dt * radius_km;
    let dither = 0.5;
    var prev_density = -1.0;

    let loop_limit = min(steps, 256);
    for (var i = 0; i < 256; i += 1) {{
        if (i >= loop_limit) {{
            break;
        }}
        let t_current = t0 + (f32(i) + dither) * dt;
        if (t_current >= t1) {{
            break;
        }}

        let p = ro + rd * t_current;
        let f = clamp((t_current - t0) / chord, 0.0, 1.0);
        let w2 = mix(w_entry, w_exit, f);
        let weather = vec3<f32>(w2.x, 0.0, w2.y);

        let density = slab_density(p, drift, weather, step_km, true);
        if (density <= 0.001) {{
            prev_density = -1.0;
            continue;
        }}

        let integ_density = select((density + prev_density) * 0.5, density, prev_density < 0.0);
        prev_density = density;

        let h01 = clamp((length(p) - bot) / thick, 0.0, 1.0);

        let shadow_d0 = 180.0 * inv_radius * shadow_span;
        let shadow_d1 = 650.0 * inv_radius * shadow_span;
        let shadow_d2 = 1800.0 * inv_radius * shadow_span;

        let s0 = slab_density(p + sun_dir * shadow_d0, drift, weather, 0.80, false);
        let s1 = slab_density(p + sun_dir * shadow_d1, drift, weather, 2.20, false);
        let s2 = slab_density(p + sun_dir * shadow_d2, drift, weather, 5.00, false);

        let sun_depth = (s0 * 0.65 + s1 * 0.90 + s2 * 1.50) * 2.8;

        let direct_attenuation = beer_powder(sun_depth);
        let ms1 = exp(-sun_depth * 0.32) * 0.35;
        let ms2 = exp(-sun_depth * 0.10) * 0.12;

        let up = p / max(length(p), 1e-6);
        let elev = dot(up, sun_dir);
        let day = smoothstep(-0.06, 0.16, elev) * sun_vis;
        let tint = mix(vec3<f32>(1.0, 0.45, 0.18), vec3<f32>(1.0, 0.98, 0.92), smoothstep(-0.02, 0.35, elev));

        let phase_direct = dual_lobe_phase(cos_theta);
        let phase_bounce = 0.50 * hg_phase(0.35, cos_theta) + 0.50 * 0.079577;

        let sun_scatter = direct_attenuation * (0.85 + 4.2 * phase_direct) + ms1 * (0.50 + 1.6 * phase_bounce) + ms2 * 0.22;
        let sun_radiance = tint * (day * sun_scatter * 2.8);

        let sky_amb = vec3<f32>(0.34, 0.50, 0.84) * (0.08 + 0.92 * day);
        let ground_amb = vec3<f32>(0.16, 0.20, 0.15) * (0.08 + 0.92 * day);
        let height_occ = smoothstep(0.04, 0.40, h01);
        let core_occ = transmittance * transmittance;
        let amb_occ = (0.20 + 0.80 * height_occ) * (0.12 + 0.88 * core_occ);
        let ambient_radiance = mix(ground_amb, sky_amb, h01 * 0.75 + 0.25) * amb_occ * 0.45;

        let step_ext = integ_density * ext * dt;
        let step_trans = exp(-step_ext);
        let point_light = sun_radiance + ambient_radiance;
        scat += transmittance * point_light * (1.0 - step_trans);
        transmittance *= step_trans;

        if (transmittance < 0.01) {{
            transmittance = 0.0;
            break;
        }}
    }}

    let alpha = clamp(1.0 - transmittance, 0.0, 1.0);
    out_results[idx * 2u] = vec4<f32>(scat, alpha);
    out_results[idx * 2u + 1u] = vec4<f32>(t0, t1, chord_km, step_km);
}}

@compute @workgroup_size(64)
fn test_solar_shadow(@builtin(global_invocation_id) gid: vec3<u32>) {{
    let idx = gid.x;
    let total_samples = arrayLength(&in_samples) / 2u;
    if (idx >= total_samples) {{
        return;
    }}

    let p = in_samples[idx * 2u].xyz;
    let sun_dir = normalize(in_samples[idx * 2u + 1u].xyz);

    let radius = max(cloud.center_radius.w, 1e-6);
    let radius_km = radius * 0.001;
    let inv_radius = 1.0 / radius;
    let top = cloud.slab.y;
    let ext = max(cloud.slab.z, 0.0);
    let shadow_span = max(cloud.params.z / 0.8, 0.05);
    let drift = vec2<f32>(cloud.params.y, cloud.params.y * 0.32);
    let weather = vec3<f32>(0.5, 0.0, 0.5);

    let shadow_d0 = 180.0 * inv_radius * shadow_span;
    let shadow_d1 = 650.0 * inv_radius * shadow_span;
    let shadow_d2 = 1800.0 * inv_radius * shadow_span;

    let s0 = slab_density(p + sun_dir * shadow_d0, drift, weather, 0.80, false);
    let s1 = slab_density(p + sun_dir * shadow_d1, drift, weather, 2.20, false);
    let s2 = slab_density(p + sun_dir * shadow_d2, drift, weather, 5.00, false);

    let sun_depth = (s0 * 0.65 + s1 * 0.90 + s2 * 1.50) * 2.8;
    let t_3tap = beer_powder(sun_depth);

    let r_p = length(p);
    let b = dot(p, sun_dir);
    let disc = b * b - (r_p * r_p - top * top);
    var t_top = 0.0;
    if (disc > 0.0) {{
        t_top = max(-b + sqrt(disc), 0.0);
    }}

    let n_steps = 32;
    let dt = t_top / f32(n_steps);
    let step_km = dt * radius_km;
    var tau_march = 0.0;
    for (var i = 0; i < n_steps; i += 1) {{
        let t_s = (f32(i) + 0.5) * dt;
        let p_s = p + sun_dir * t_s;
        let d = slab_density(p_s, drift, weather, step_km, false);
        tau_march += d * ext * dt;
    }}
    let t_march = exp(-tau_march);

    out_results[idx * 2u] = vec4<f32>(t_3tap, t_march, sun_depth, tau_march);
    out_results[idx * 2u + 1u] = vec4<f32>(t_top * radius_km, step_km, s0, s1);
}}
"#
    )
}

/// Helper to bake a 64x64 synthetic island coverage map texture.
fn make_synthetic_island_tex() -> (u32, Vec<u8>) {
    let tex_size = 64u32;
    let mut tex_bytes = vec![0u8; (tex_size * tex_size * 4) as usize];
    for y in 0..tex_size {
        for x in 0..tex_size {
            let u = (x as f32 + 0.5) / (tex_size as f32);
            let v = (y as f32 + 0.5) / (tex_size as f32);
            let du = (u - 0.5).abs();
            let dv = (v - 0.5).abs();
            let dist = (du * du + dv * dv).sqrt();
            let cov = (1.0 - (dist / 0.25)).clamp(0.0, 1.0) * 0.85;
            let alpha = (cov * 255.0).round() as u8;
            let offset = ((y * tex_size + x) * 4) as usize;
            tex_bytes[offset] = 255;
            tex_bytes[offset + 1] = 255;
            tex_bytes[offset + 2] = 255;
            tex_bytes[offset + 3] = alpha;
        }
    }
    (tex_size, tex_bytes)
}

#[test]
#[ignore = "requires a wgpu adapter; run with --ignored --nocapture"]
fn synthetic_fixture_slab_density_and_lod_characterization_on_hardware() {
    let gpu = Gpu::new();
    let (tex_size, tex_bytes) = make_synthetic_island_tex();
    let (_tex, view, sampler) = create_gpu_coverage_texture(&gpu, tex_size, tex_size, &tex_bytes);

    let uniform_words = pack_cloud_uniform_words(
        3_200_000.0,
        1.000625,
        1.0028125,
        42.0,
        24.0,
        glam::Vec3::new(0.0, 1.0, 0.0),
        1.0,
        1.0,
        0.0,
        0.8,
        1.6,
        1.0028125,
        false,
    );
    let uniform_buf = gpu.buffer(&uniform_words, true);

    let step_sizes = [0.5f32, 2.0f32, 6.0f32, 12.0f32];
    let r_bottom = 1.000625f32;
    let r_top = 1.0028125f32;
    let r_span = r_top - r_bottom;
    let h_steps = 32;

    let mut samples: Vec<[f32; 4]> = Vec::new();
    // Group A: Vertical column through cloud island center (local dir [-1, 0, 0] -> u=0.5, v=0.5)
    for &step_km in &step_sizes {
        for i in 0..h_steps {
            let h01 = (i as f32 + 0.5) / (h_steps as f32);
            let r = r_bottom + h01 * r_span;
            samples.push([-r, 0.0, 0.0, step_km]);
        }
    }
    // Group B: Vertical column in clear sky (local dir [1, 0, 0] -> u=0.0, v=0.5)
    for &step_km in &step_sizes {
        for i in 0..h_steps {
            let h01 = (i as f32 + 0.5) / (h_steps as f32);
            let r = r_bottom + h01 * r_span;
            samples.push([r, 0.0, 0.0, step_km]);
        }
    }

    let num_samples = samples.len();
    let in_words: Vec<u32> = samples
        .iter()
        .flat_map(|s| [s[0].to_bits(), s[1].to_bits(), s[2].to_bits(), s[3].to_bits()])
        .collect();
    let out_words = vec![0u32; num_samples * 4];

    let in_buf = gpu.buffer(&in_words, false);
    let out_buf = gpu.buffer(&out_words, false);

    let shader = build_cloud_test_wgsl();
    execute_cloud_compute(
        &gpu,
        &shader,
        "test_slab_density",
        &uniform_buf,
        &view,
        &sampler,
        &in_buf,
        &out_buf,
        num_samples.div_ceil(64) as u32,
    );

    let readback_words = gpu.read(&out_buf);
    let readback_floats: Vec<f32> = readback_words.iter().map(|&w| f32::from_bits(w)).collect();

    // Assert Group B clear sky strictly zero density and tau:
    let group_b_offset = step_sizes.len() * h_steps;
    for i in 0..(step_sizes.len() * h_steps) {
        let sample_idx = group_b_offset + i;
        let d_full = readback_floats[sample_idx * 4];
        assert_eq!(d_full, 0.0, "Clear sky sample {sample_idx} must have strictly zero density");
    }

    // Quadrature analysis of Group A:
    // Discretization interval: slab thickness 7.0 km over 32 midpoint intervals.
    // ds = 7.0 / 32 km.
    // True optical depth tau = sigma_t * integral(rho * dt) = sigma_t * (integral(rho * ds) / R_planet).
    // For R = 3200 km, ext = 42: factor is (42.0 / 3200.0) = 0.013125.
    eprintln!("\n=== [GPU FUNCTION TESTED] Synthetic Fixture Column Density & Optical Depth ===");
    let slab_thickness_km = 7.0f32;
    let ds_km = slab_thickness_km / (h_steps as f32);
    let ext = 42.0f32;
    let r_planet_km = 3200.0f32;
    let mut column_dens_integrals = Vec::new();
    let mut column_tau = Vec::new();
    let mut column_tau_fast = Vec::new();

    for (step_idx, &step_km) in step_sizes.iter().enumerate() {
        let offset = step_idx * h_steps;
        let mut sum_density = 0.0f32;
        let mut sum_density_fast = 0.0f32;
        let mut active_count = 0;
        for i in 0..h_steps {
            let sample_idx = offset + i;
            let d = readback_floats[sample_idx * 4];
            let d_fast = readback_floats[sample_idx * 4 + 1];
            sum_density += d;
            sum_density_fast += d_fast;
            if d > 0.001 {
                active_count += 1;
            }
        }
        let density_integral_km = sum_density * ds_km;
        let density_integral_fast_km = sum_density_fast * ds_km;
        let tau = density_integral_km * (ext / r_planet_km);
        let tau_fast = density_integral_fast_km * (ext / r_planet_km);
        let transmittance = (-tau).exp();
        let alpha = 1.0 - transmittance;

        column_dens_integrals.push(density_integral_km);
        column_tau.push(tau);
        column_tau_fast.push(tau_fast);

        eprintln!(
            "Step {:>4.1} km | Active: {:>2}/{} | Dens Integral: {:>7.4} km | Tau: {:.4} | Transmittance: {:.4} | Alpha: {:.4}",
            step_km, active_count, h_steps, density_integral_km, tau, transmittance, alpha
        );
        assert!(tau > 0.1, "Column optical depth must be non-zero in cloud core");
        assert!(active_count >= 10, "Cloud body must span significant height range");
    }

    let int_0_5 = column_dens_integrals[0];
    let int_12 = column_dens_integrals[3];
    let rel_var = (int_12 - int_0_5).abs() / int_0_5;
    eprintln!("Density line integral variation across 24x step span: {:.2}%", rel_var * 100.0);
    assert!(rel_var < 0.20, "Filter response variation ({rel_var:.3}) must be < 20%");

    for (step_idx, (&tau, &tau_f)) in column_tau.iter().zip(&column_tau_fast).enumerate() {
        let shadow_err = (tau_f - tau).abs() / tau;
        eprintln!(
            "Step {:>4.1} km | Shadow fast-path optical depth discrepancy: {:.2}%",
            step_sizes[step_idx], shadow_err * 100.0
        );
        assert!(shadow_err < 0.25, "Shadow fast-path error {shadow_err:.3} exceeds 25%");
    }
    eprintln!("[GPU FUNCTION TESTED] synthetic_fixture_slab_density_and_lod_characterization_on_hardware: PASSED\n");
}

#[test]
#[ignore = "requires a wgpu adapter; run with --ignored --nocapture"]
fn fine_resolution_cloud_boundary_silhouette_stability_on_hardware() {
    let gpu = Gpu::new();
    let (tex_size, tex_bytes) = make_synthetic_island_tex();
    let (_tex, view, sampler) = create_gpu_coverage_texture(&gpu, tex_size, tex_size, &tex_bytes);

    let uniform_words = pack_cloud_uniform_words(
        3_200_000.0,
        1.000625,
        1.0028125,
        42.0,
        24.0,
        glam::Vec3::new(0.0, 1.0, 0.0),
        1.0,
        1.0,
        0.0,
        0.8,
        1.6,
        1.0028125,
        false,
    );
    let uniform_buf = gpu.buffer(&uniform_words, true);

    let r_bottom = 1.000625f32;
    let r_top = 1.0028125f32;
    let r_mid = r_bottom + 0.40 * (r_top - r_bottom);

    // 1. Coarse probe along equator to locate the cloud edge crossing rho=0.05:
    let n_probe = 32;
    let u_probe_start = 0.60f32;
    let u_probe_end = 0.72f32;
    let mut probe_samples = Vec::new();
    for i in 0..n_probe {
        let u = u_probe_start + (u_probe_end - u_probe_start) * (i as f32) / ((n_probe - 1) as f32);
        let theta = u * std::f32::consts::TAU;
        probe_samples.push([theta.cos() * r_mid, theta.sin() * r_mid, 0.0, 0.5]);
    }
    let probe_in_words: Vec<u32> = probe_samples
        .iter()
        .flat_map(|s| [s[0].to_bits(), s[1].to_bits(), s[2].to_bits(), s[3].to_bits()])
        .collect();
    let probe_out_words = vec![0u32; n_probe * 4];
    let probe_in_buf = gpu.buffer(&probe_in_words, false);
    let probe_out_buf = gpu.buffer(&probe_out_words, false);

    let shader = build_cloud_test_wgsl();
    execute_cloud_compute(
        &gpu,
        &shader,
        "test_slab_density",
        &uniform_buf,
        &view,
        &sampler,
        &probe_in_buf,
        &probe_out_buf,
        (n_probe as u32).div_ceil(64),
    );
    let probe_readback = gpu.read(&probe_out_buf);
    let probe_floats: Vec<f32> = probe_readback.iter().map(|&w| f32::from_bits(w)).collect();
    for i in 0..n_probe {
        let u = u_probe_start + (u_probe_end - u_probe_start) * (i as f32) / ((n_probe - 1) as f32);
        eprintln!("probe {i}: u={u:.4}, d={}", probe_floats[i * 4]);
    }

    let mut u_seg_start = 0.0f32;
    let mut u_seg_end = 0.0f32;
    let du_probe = (u_probe_end - u_probe_start) / ((n_probe - 1) as f32);
    for i in 0..(n_probe - 1) {
        let d0 = probe_floats[i * 4];
        let d1 = probe_floats[(i + 1) * 4];
        if (d0 >= 0.05 && d1 < 0.05) || (d0 < 0.05 && d1 >= 0.05) {
            u_seg_start = u_probe_start + (i as f32) * du_probe;
            u_seg_end = u_probe_start + ((i + 1) as f32) * du_probe;
            break;
        }
    }
    assert!(u_seg_end > u_seg_start, "coarse probe must locate cloud boundary segment");

    // Stage 2 refinement probe on the detected [u_seg_start, u_seg_end] segment (spans ~78 km):
    let n_refine = 32;
    let mut refine_samples = Vec::new();
    for i in 0..n_refine {
        let u = u_seg_start + (u_seg_end - u_seg_start) * (i as f32) / ((n_refine - 1) as f32);
        let theta = u * std::f32::consts::TAU;
        refine_samples.push([theta.cos() * r_mid, theta.sin() * r_mid, 0.0, 0.5]);
    }
    let refine_in_words: Vec<u32> = refine_samples
        .iter()
        .flat_map(|s| [s[0].to_bits(), s[1].to_bits(), s[2].to_bits(), s[3].to_bits()])
        .collect();
    let refine_out_words = vec![0u32; n_refine * 4];
    let refine_in_buf = gpu.buffer(&refine_in_words, false);
    let refine_out_buf = gpu.buffer(&refine_out_words, false);

    execute_cloud_compute(
        &gpu,
        &shader,
        "test_slab_density",
        &uniform_buf,
        &view,
        &sampler,
        &refine_in_buf,
        &refine_out_buf,
        (n_refine as u32).div_ceil(64),
    );
    let refine_readback = gpu.read(&refine_out_buf);
    let refine_floats: Vec<f32> = refine_readback.iter().map(|&w| f32::from_bits(w)).collect();

    let du_refine = (u_seg_end - u_seg_start) / ((n_refine - 1) as f32);
    let mut u_boundary = None;
    for i in 0..(n_refine - 1) {
        let d0 = refine_floats[i * 4];
        let d1 = refine_floats[(i + 1) * 4];
        if (d0 >= 0.05 && d1 < 0.05) || (d0 < 0.05 && d1 >= 0.05) {
            let frac = (0.05 - d0) / (d1 - d0);
            let u = u_seg_start + (i as f32 + frac) * du_refine;
            u_boundary = Some(u);
            break;
        }
    }
    let u_edge = u_boundary.expect("refinement probe must locate cloud boundary crossing");

    // 2. High-resolution localized 30.0 km sweep centered at the detected cloud boundary:
    // N = 128 points -> dx = 30.0 / 127 ≈ 236.2 metres per point.
    let sweep_span_km = 30.0f32;
    let n_points = 128;
    let dx_km = sweep_span_km / ((n_points - 1) as f32);
    let arc_span = sweep_span_km / 3200.0f32;
    let theta_center = u_edge * std::f32::consts::TAU;

    let step_sizes = [0.5f32, 2.0f32, 6.0f32];
    let mut fine_samples = Vec::new();

    for &step_km in &step_sizes {
        for i in 0..n_points {
            let frac = (i as f32) / ((n_points - 1) as f32) - 0.5f32;
            let theta = theta_center + frac * arc_span;
            let dir_x = theta.cos();
            let dir_y = theta.sin();
            fine_samples.push([dir_x * r_mid, dir_y * r_mid, 0.0, step_km]);
        }
    }

    let in_words: Vec<u32> = fine_samples
        .iter()
        .flat_map(|s| [s[0].to_bits(), s[1].to_bits(), s[2].to_bits(), s[3].to_bits()])
        .collect();
    let out_words = vec![0u32; fine_samples.len() * 4];

    let in_buf = gpu.buffer(&in_words, false);
    let out_buf = gpu.buffer(&out_words, false);

    execute_cloud_compute(
        &gpu,
        &shader,
        "test_slab_density",
        &uniform_buf,
        &view,
        &sampler,
        &in_buf,
        &out_buf,
        fine_samples.len().div_ceil(64) as u32,
    );

    let readback_words = gpu.read(&out_buf);
    let readback_floats: Vec<f32> = readback_words.iter().map(|&w| f32::from_bits(w)).collect();

    eprintln!("Coarse probe detected u_edge = {:.6}", u_edge);
    eprintln!("\n=== [GPU FUNCTION TESTED] Fine-Resolution Cloud Boundary Stability (30 km sweep, dx ≈ 236 m) ===");
    let mut boundary_positions_km = Vec::new();

    for (step_idx, &step_km) in step_sizes.iter().enumerate() {
        let offset = step_idx * n_points;
        let d_start = readback_floats[offset * 4];
        let d_end = readback_floats[(offset + n_points - 1) * 4];
        eprintln!("Step {:>4.1} km | d_start = {:.4}, d_end = {:.4}", step_km, d_start, d_end);
        let mut crossing_km = None;
        for i in 0..(n_points - 1) {
            let d0 = readback_floats[(offset + i) * 4];
            let d1 = readback_floats[(offset + i + 1) * 4];
            if (d0 >= 0.05 && d1 < 0.05) || (d0 < 0.05 && d1 >= 0.05) {
                let frac = (0.05 - d0) / (d1 - d0);
                let pos_km = (i as f32 + frac) * dx_km;
                crossing_km = Some(pos_km);
                break;
            }
        }
        let cross = crossing_km.expect("fine sweep boundary contour crossing rho=0.05 must exist in 30 km sweep");
        boundary_positions_km.push(cross);
        eprintln!(
            "Step {:>4.1} km | Boundary position x = {:>6.3} km (from start of 30 km sweep)",
            step_km, cross
        );
    }

    let ref_cross = boundary_positions_km[0];
    for (step_idx, &cross) in boundary_positions_km.iter().enumerate().skip(1) {
        let shift_km = (cross - ref_cross).abs();
        eprintln!(
            "Step {:>4.1} km | Physical Boundary Shift: {:>6.3} km (<= 1.5 km required)",
            step_sizes[step_idx], shift_km
        );
        assert!(
            shift_km <= 1.5,
            "Boundary shift {shift_km:.3} km exceeds 1.5 km tolerance at step {}",
            step_sizes[step_idx]
        );
    }
    eprintln!("[GPU FUNCTION TESTED] fine_resolution_cloud_boundary_silhouette_stability_on_hardware: PASSED\n");
}

#[test]
#[ignore = "requires a wgpu adapter; run with --ignored --nocapture"]
fn production_thessa_fixture_cloud_render_and_quadrature_on_hardware() {
    let gpu = Gpu::new();

    // Canonical production coverage texture baked via thessa_atmosphere::clouds::bake_clouds:
    let tex_w = 256u32;
    let tex_h = 128u32;
    let tex_bytes = thessa_atmosphere::clouds::bake_clouds(
        0xC10D_0000_0000_0001,
        tex_w,
        tex_h,
        0.55,
        1.0,
        (1.0, 1.0),
    );
    let (_tex, view, sampler) = create_gpu_coverage_texture(&gpu, tex_w, tex_h, &tex_bytes);

    // Canonical Thessa planetary configuration:
    // Planet radius = 3200 km (3,200,000 m).
    // Low-deck slab: 25.6 km to 44.8 km (1.008 to 1.014 radii). Thickness = 19.2 km.
    // March ceiling = 54.72 km (1.0171 radii).
    // Production extinction CLOUD_EXT_LOW = 1000.0 per radius.
    let r_planet = 3_200_000.0f32;
    let bot = 1.008f32;
    let top = 1.014f32;
    let march_top = 1.0171f32;
    let ext = 1000.0f32;
    let steps = 24.0f32;

    // Direct unit vectors derived from the canonical bake texture:
    // Peak cloud core (coverage ~ 0.77, alpha 196 at (57, 78)):
    let dir_peak = glam::Vec3::new(0.1488, 0.9262, -0.346).normalize();
    // Clear sky (North pole polar region, coverage strictly 0):
    let dir_clear = glam::Vec3::new(0.0, 0.0, 1.0);
    let right = glam::Vec3::new(0.0, 0.0, 1.0).cross(dir_peak).normalize();

    let uniform_words = pack_cloud_uniform_words(
        r_planet,
        bot,
        top,
        ext,
        steps,
        glam::Vec3::new(0.0, 1.0, 0.5).normalize(),
        1.0,
        1.0,
        0.0,
        0.8,
        1.6,
        march_top,
        false,
    );
    let uniform_buf = gpu.buffer(&uniform_words, true);

    // 5 Canonical Viewing Regimes:
    // 0: Surface Zenith (overcast cloud core: cam at r=1.0, rd pointing straight up into overcast core)
    // 1: Surface Zenith (clear sky: cam at r=1.0, rd pointing straight up into clear oceanic sky)
    // 2: In-Cloud Observer (cam at r=1.011, h=35.2 km inside deck, viewing horizontally)
    // 3: Low-Sun / Sunset Regime (sun elevation 15 deg, view directed at sun for dual-lobe silver lining)
    // 4: Orbit Limb (cam in orbit at r=1.080, h=256 km, tangent ray with coarse step ds ≈ 36 km)
    let in_cloud_ro = dir_peak * 1.011;
    let sun_15 = (dir_peak * 15.0_f32.to_radians().sin() + right * 15.0_f32.to_radians().cos()).normalize();
    let r_cam = 1.080f32;
    let r_graze = (bot + top) * 0.5;
    let sin_theta = (r_graze / r_cam).clamp(0.0, 1.0);
    let cos_theta = (1.0 - sin_theta * sin_theta).sqrt();
    let orb_ro = dir_peak * r_cam;
    let orb_rd = (-dir_peak * cos_theta + right * sin_theta).normalize();

    let ray_data: Vec<[f32; 4]> = vec![
        // Regime 0: Overcast core zenith (cam at surface under peak cloud core, looking up)
        [dir_peak.x, dir_peak.y, dir_peak.z, 0.0],
        [dir_peak.x, dir_peak.y, dir_peak.z, 0.0],
        // Regime 1: Clear sky zenith (cam at surface under clear polar sky, looking up)
        [dir_clear.x, dir_clear.y, dir_clear.z, 0.0],
        [dir_clear.x, dir_clear.y, dir_clear.z, 0.0],
        // Regime 2: In-cloud observer at h = 35.2 km (r = 1.011) looking horizontally
        [in_cloud_ro.x, in_cloud_ro.y, in_cloud_ro.z, 0.0],
        [right.x, right.y, right.z, 0.0],
        // Regime 3: Sunset silver-lining forward scatter (viewing towards 15° sun)
        [dir_peak.x * 1.0001, dir_peak.y * 1.0001, dir_peak.z * 1.0001, 0.0],
        [sun_15.x, sun_15.y, sun_15.z, 0.0],
        // Regime 4: Orbital insertion (r = 1.080, 256 km altitude) looking at horizon tangent
        [orb_ro.x, orb_ro.y, orb_ro.z, 0.0],
        [orb_rd.x, orb_rd.y, orb_rd.z, 0.0],
    ];

    let num_rays = ray_data.len() / 2;
    let in_words: Vec<u32> = ray_data
        .iter()
        .flat_map(|r| [r[0].to_bits(), r[1].to_bits(), r[2].to_bits(), r[3].to_bits()])
        .collect();
    let out_words = vec![0u32; num_rays * 8];

    let in_buf = gpu.buffer(&in_words, false);
    let out_buf = gpu.buffer(&out_words, false);

    let shader = build_cloud_test_wgsl();
    execute_cloud_compute(
        &gpu,
        &shader,
        "test_raymarch",
        &uniform_buf,
        &view,
        &sampler,
        &in_buf,
        &out_buf,
        num_rays.div_ceil(64) as u32,
    );

    let readback_words = gpu.read(&out_buf);
    let readback_floats: Vec<f32> = readback_words.iter().map(|&w| f32::from_bits(w)).collect();

    eprintln!("\n=== [GPU RENDER VERIFIED] Production Thessa Fixture across 5 Viewing Regimes ===");
    let regime_names = [
        "Surface Zenith (Overcast Core)",
        "Surface Zenith (Clear Air)",
        "In-Cloud Observer (h=35.2 km)",
        "Low-Sun Silver Lining (15 deg)",
        "Orbital Limb Horizon (h=256 km)",
    ];

    for (idx, &name) in regime_names.iter().enumerate().take(num_rays) {
        let base = idx * 8;
        let scat = glam::Vec3::new(readback_floats[base], readback_floats[base + 1], readback_floats[base + 2]);
        let alpha = readback_floats[base + 3];
        let chord_km = readback_floats[base + 6];
        let step_km = readback_floats[base + 7];

        assert!(scat.is_finite(), "regime {idx} scat non-finite: {scat:?}");
        assert!(alpha.is_finite(), "regime {idx} alpha non-finite: {alpha}");
        assert!((0.0..=1.0).contains(&alpha), "regime {idx} alpha out of [0, 1]: {alpha}");

        eprintln!(
            "Regime {}: {:<32} | Chord: {:>6.2} km | Step: {:>5.2} km | Alpha: {:>6.4} | Scat: ({:.3}, {:.3}, {:.3})",
            idx, name, chord_km, step_km, alpha, scat.x, scat.y, scat.z
        );
    }

    // Regime 0: Surface Zenith Overcast:
    let r0_alpha = readback_floats[3];
    assert!(
        r0_alpha > 0.95,
        "Overcast core must be essentially opaque (alpha > 0.95, got {r0_alpha:.4})"
    );

    // Regime 1: Surface Zenith Clear Sky:
    let r1_alpha = readback_floats[8 + 3];
    assert_eq!(
        r1_alpha, 0.0,
        "Clear sky zenith must have strictly zero opacity"
    );

    // Regime 2: In-Cloud Observer:
    let r2_t0 = readback_floats[16 + 4];
    assert_eq!(
        r2_t0, 0.0,
        "In-cloud observer must start raymarch immediately at t0=0"
    );

    // Regime 3: Sunset Silver Lining:
    let r3_scat = glam::Vec3::new(readback_floats[24], readback_floats[25], readback_floats[26]);
    assert!(
        r3_scat.length() > 0.05,
        "Forward scattering in sunset regime must produce bright radiance"
    );

    // Regime 4: Orbital Insertion:
    let r4_chord_km = readback_floats[32 + 6];
    assert!(
        r4_chord_km > 300.0,
        "Orbital horizon tangent chord must be long (> 300 km, got {r4_chord_km:.1} km)"
    );

    eprintln!("[GPU RENDER VERIFIED] production_thessa_fixture_cloud_render_and_quadrature_on_hardware: PASSED\n");
}

#[test]
#[ignore = "requires a wgpu adapter; run with --ignored --nocapture"]
fn production_raymarch_quadrature_convergence_on_hardware() {
    let gpu = Gpu::new();
    let tex_w = 256u32;
    let tex_h = 128u32;
    let tex_bytes = thessa_atmosphere::clouds::bake_clouds(
        0xC10D_0000_0000_0001,
        tex_w,
        tex_h,
        0.55,
        1.0,
        (1.0, 1.0),
    );
    let (_tex, view, sampler) = create_gpu_coverage_texture(&gpu, tex_w, tex_h, &tex_bytes);

    let uniform_words = pack_cloud_uniform_words(
        3_200_000.0,
        1.008,
        1.014,
        1000.0,
        24.0,
        glam::Vec3::new(0.0, 1.0, 0.0),
        1.0,
        1.0,
        0.0,
        0.8,
        1.6,
        1.0171,
        false,
    );
    let uniform_buf = gpu.buffer(&uniform_words, true);

    // Slant ray directed through the canonical cloud core:
    let dir_peak = glam::Vec3::new(0.1488, 0.9262, -0.346).normalize();
    let right = glam::Vec3::new(0.0, 0.0, 1.0).cross(dir_peak).normalize();
    let slant_rd = (dir_peak * 0.866 + right * 0.50).normalize();

    // Variable step marching: N = [96, 48, 24, 12, 6] steps
    // (step sizes approx 0.25, 0.50, 1.00, 2.00, 4.00 km)
    let step_counts = [96.0f32, 48.0f32, 24.0f32, 12.0f32, 6.0f32];
    let mut ray_data = Vec::new();
    for &n_steps in &step_counts {
        ray_data.push([dir_peak.x, dir_peak.y, dir_peak.z, 0.0]); // ro at surface
        ray_data.push([slant_rd.x, slant_rd.y, slant_rd.z, n_steps]); // rd, steps_override
    }

    let num_rays = step_counts.len();
    let in_words: Vec<u32> = ray_data
        .iter()
        .flat_map(|r| [r[0].to_bits(), r[1].to_bits(), r[2].to_bits(), r[3].to_bits()])
        .collect();
    let out_words = vec![0u32; num_rays * 8];

    let in_buf = gpu.buffer(&in_words, false);
    let out_buf = gpu.buffer(&out_words, false);

    let shader = build_cloud_test_wgsl();
    execute_cloud_compute(
        &gpu,
        &shader,
        "test_raymarch",
        &uniform_buf,
        &view,
        &sampler,
        &in_buf,
        &out_buf,
        num_rays.div_ceil(64) as u32,
    );

    let readback_words = gpu.read(&out_buf);
    let readback_floats: Vec<f32> = readback_words.iter().map(|&w| f32::from_bits(w)).collect();

    eprintln!("\n=== [GPU RENDER VERIFIED] Production Raymarch Quadrature Convergence ===");
    let mut transmittances = Vec::new();
    let mut radiances = Vec::new();

    for (idx, &n_steps) in step_counts.iter().enumerate() {
        let base = idx * 8;
        let scat = glam::Vec3::new(readback_floats[base], readback_floats[base + 1], readback_floats[base + 2]);
        let alpha = readback_floats[base + 3];
        let chord_km = readback_floats[base + 6];
        let step_km = readback_floats[base + 7];
        let trans = 1.0 - alpha;
        transmittances.push(trans);
        radiances.push(scat.length());

        eprintln!(
            "Steps: {:>2} (ds = {:>4.2} km) | Chord: {:>5.2} km | Transmittance: {:.4} | Alpha: {:.4} | Radiance: {:.4}",
            n_steps as u32, step_km, chord_km, trans, alpha, scat.length()
        );
    }

    let ref_trans = transmittances[0]; // N=96 reference
    for (idx, &n_steps) in step_counts.iter().enumerate().skip(1) {
        let err = (transmittances[idx] - ref_trans).abs();
        eprintln!(
            "Steps {:>2} | Transmittance error vs N=96 reference: {:.4}",
            n_steps as u32, err
        );
        if n_steps >= 24.0 {
            assert!(
                err < 0.08,
                "N={n_steps} error {err:.4} exceeds 0.08 tolerance vs N=96 reference"
            );
        } else if n_steps >= 12.0 {
            assert!(
                err < 0.15,
                "N={n_steps} error {err:.4} exceeds 0.15 tolerance vs N=96 reference"
            );
        }
    }
    eprintln!("[GPU RENDER VERIFIED] production_raymarch_quadrature_convergence_on_hardware: PASSED\n");
}

#[test]
#[ignore = "requires a wgpu adapter; run with --ignored --nocapture"]
fn production_solar_shadow_transmittance_on_hardware() {
    let gpu = Gpu::new();
    let tex_w = 256u32;
    let tex_h = 128u32;
    let tex_bytes = thessa_atmosphere::clouds::bake_clouds(
        0xC10D_0000_0000_0001,
        tex_w,
        tex_h,
        0.55,
        1.0,
        (1.0, 1.0),
    );
    let (_tex, view, sampler) = create_gpu_coverage_texture(&gpu, tex_w, tex_h, &tex_bytes);

    let uniform_words = pack_cloud_uniform_words(
        3_200_000.0,
        1.008,
        1.014,
        1000.0,
        24.0,
        glam::Vec3::new(0.0, 1.0, 0.0),
        1.0,
        1.0,
        0.0,
        0.8,
        1.6,
        1.0171,
        false,
    );
    let uniform_buf = gpu.buffer(&uniform_words, true);

    // In-cloud sample point p at h01 = 0.35 inside the canonical cumulus core:
    let dir_peak = glam::Vec3::new(0.1488, 0.9262, -0.346).normalize();
    let right = glam::Vec3::new(0.0, 0.0, 1.0).cross(dir_peak).normalize();
    let r_p = 1.008 + 0.35 * (1.014 - 1.008);
    let p = dir_peak * r_p;

    // Three sun elevation angles relative to the local zenith:
    // 0: Noon (sun straight up at 90° elevation along local zenith dir_peak)
    // 1: Afternoon (sun at 45° elevation)
    // 2: Sunset (sun at 15° elevation)
    let sun_noon = dir_peak;
    let sun_45 = (dir_peak * 45.0_f32.to_radians().sin() + right * 45.0_f32.to_radians().cos()).normalize();
    let sun_15 = (dir_peak * 15.0_f32.to_radians().sin() + right * 15.0_f32.to_radians().cos()).normalize();
    let sun_dirs = [sun_noon, sun_45, sun_15];

    let mut sample_data = Vec::new();
    for &dir in &sun_dirs {
        sample_data.push([p.x, p.y, p.z, 0.0]);
        sample_data.push([dir.x, dir.y, dir.z, 0.0]);
    }

    let num_samples = sun_dirs.len();
    let in_words: Vec<u32> = sample_data
        .iter()
        .flat_map(|r| [r[0].to_bits(), r[1].to_bits(), r[2].to_bits(), r[3].to_bits()])
        .collect();
    let out_words = vec![0u32; num_samples * 8];

    let in_buf = gpu.buffer(&in_words, false);
    let out_buf = gpu.buffer(&out_words, false);

    let shader = build_cloud_test_wgsl();
    execute_cloud_compute(
        &gpu,
        &shader,
        "test_solar_shadow",
        &uniform_buf,
        &view,
        &sampler,
        &in_buf,
        &out_buf,
        num_samples.div_ceil(64) as u32,
    );

    let readback_words = gpu.read(&out_buf);
    let readback_floats: Vec<f32> = readback_words.iter().map(|&w| f32::from_bits(w)).collect();

    eprintln!("\n=== [GPU FUNCTION TESTED] Production Solar Shadow Transmittance along Sun Vectors ===");
    let angles = ["Noon (90 deg)", "Afternoon (45 deg)", "Sunset (15 deg)"];
    let mut trans_3tap = Vec::new();
    let mut trans_march = Vec::new();

    let mut taus_march = Vec::new();
    let mut march_dists = Vec::new();

    for (idx, &angle_name) in angles.iter().enumerate().take(num_samples) {
        let base = idx * 8;
        let t_3tap = readback_floats[base];
        let t_march = readback_floats[base + 1];
        let sun_depth = readback_floats[base + 2];
        let tau_march = readback_floats[base + 3];
        let march_dist_km = readback_floats[base + 4];

        trans_3tap.push(t_3tap);
        trans_march.push(t_march);
        taus_march.push(tau_march);
        march_dists.push(march_dist_km);

        eprintln!(
            "Angle: {:<18} | March Dist: {:>5.2} km | T(3-tap): {:.4} (depth {:.2}) | T(March): {:.4} (tau {:.2})",
            angle_name, march_dist_km, t_3tap, sun_depth, t_march, tau_march
        );
    }

    // Physical monotonicity assertion: oblique sun travels longer distance through slab,
    // so path distance and multi-step optical depth tau_march strictly increase:
    assert!(
        march_dists[0] < march_dists[1] && march_dists[1] < march_dists[2],
        "Sun ray path distance through slab must strictly increase as sun descends: {:?}",
        march_dists
    );
    assert!(
        taus_march[0] < taus_march[1] && taus_march[1] < taus_march[2],
        "Multi-step march optical depth must strictly increase as sun descends: {:?}",
        taus_march
    );
    assert!(
        trans_march[0] >= trans_march[1] && trans_march[1] >= trans_march[2],
        "Multi-step march transmittance must monotonically decrease as sun descends: {:?}",
        trans_march
    );
    for i in 0..num_samples {
        assert!(
            trans_march[i] <= 0.05,
            "Deep cloud core must be strongly shadowed (trans_march <= 0.05, got {})",
            trans_march[i]
        );
        assert!(
            trans_3tap[i] <= 0.05,
            "Deep cloud core must be strongly shadowed in 3-tap approximation (trans_3tap <= 0.05, got {})",
            trans_3tap[i]
        );
    }

    eprintln!("[GPU FUNCTION TESTED] production_solar_shadow_transmittance_on_hardware: PASSED\n");
}

#[test]
#[ignore = "requires a wgpu adapter; run with --ignored --nocapture"]
fn production_camera_ascent_continuity_across_full_transition_on_hardware() {
    let gpu = Gpu::new();
    let tex_w = 256u32;
    let tex_h = 128u32;
    let tex_bytes = thessa_atmosphere::clouds::bake_clouds(
        0xC10D_0000_0000_0001,
        tex_w,
        tex_h,
        0.55,
        1.0,
        (1.0, 1.0),
    );
    let (_tex, view, sampler) = create_gpu_coverage_texture(&gpu, tex_w, tex_h, &tex_bytes);

    let uniform_words = pack_cloud_uniform_words(
        3_200_000.0,
        1.008,
        1.014,
        1000.0,
        24.0,
        glam::Vec3::new(0.0, 1.0, 0.0),
        1.0,
        1.0,
        0.0,
        0.8,
        1.6,
        1.0171,
        false,
    );
    let uniform_buf = gpu.buffer(&uniform_words, true);

    // Camera ascending from 50 km to 220 km in 35 steps (delta_h = 5.0 km) on actual Vulkan GPU:
    let n_alts = 35;
    let h_min = 50.0f32;
    let h_max = 220.0f32;
    let delta_h = (h_max - h_min) / ((n_alts - 1) as f32);

    let mut ray_data = Vec::new();
    let mut alt_km_list = Vec::new();

    let r_graze = (1.008f32 + 1.014f32) * 0.5;

    for i in 0..n_alts {
        let h_km = h_min + (i as f32) * delta_h;
        alt_km_list.push(h_km);
        let r_cam = 1.0 + (h_km / 3200.0);
        let sin_theta = (r_graze / r_cam).clamp(0.0, 1.0);
        let cos_theta = (1.0 - sin_theta * sin_theta).sqrt();
        ray_data.push([0.0, -r_cam, 0.0, 0.0]); // ro
        ray_data.push([sin_theta, cos_theta, 0.0, 0.0]); // rd: aimed at grazing tangent
    }

    let in_words: Vec<u32> = ray_data
        .iter()
        .flat_map(|r| [r[0].to_bits(), r[1].to_bits(), r[2].to_bits(), r[3].to_bits()])
        .collect();
    let out_words = vec![0u32; n_alts * 8];

    let in_buf = gpu.buffer(&in_words, false);
    let out_buf = gpu.buffer(&out_words, false);

    let shader = build_cloud_test_wgsl();
    execute_cloud_compute(
        &gpu,
        &shader,
        "test_raymarch",
        &uniform_buf,
        &view,
        &sampler,
        &in_buf,
        &out_buf,
        (n_alts as u32).div_ceil(64),
    );

    let readback_words = gpu.read(&out_buf);
    let readback_floats: Vec<f32> = readback_words.iter().map(|&w| f32::from_bits(w)).collect();

    eprintln!("\n=== [GPU RENDER VERIFIED] Production Camera Ascent Continuity (50–220 km on Hardware) ===");
    let mut chords = Vec::new();
    let mut steps = Vec::new();
    let mut alphas = Vec::new();

    for (i, &alt_km) in alt_km_list.iter().enumerate().take(n_alts) {
        let base = i * 8;
        let alpha = readback_floats[base + 3];
        let chord_km = readback_floats[base + 6];
        let step_km = readback_floats[base + 7];
        chords.push(chord_km);
        steps.push(step_km);
        alphas.push(alpha);

        if i % 5 == 0 || i == n_alts - 1 {
            eprintln!(
                "Alt: {:>5.1} km | Chord: {:>6.2} km | Step: {:>5.2} km | Rendered Alpha: {:.4}",
                alt_km, chord_km, step_km, alpha
            );
        }
    }

    // Verify discrete derivatives:
    let max_chord_rate = 8.0f32; // km/km
    let max_step_rate = 0.35f32; // km/km

    for i in 0..(n_alts - 1) {
        let dh = alt_km_list[i + 1] - alt_km_list[i];
        let d_chord = (chords[i + 1] - chords[i]).abs() / dh;
        let d_step = (steps[i + 1] - steps[i]).abs() / dh;

        assert!(
            d_chord <= max_chord_rate,
            "Discrete chord rate {d_chord:.2} km/km at alt {} km exceeds limit {max_chord_rate}",
            alt_km_list[i]
        );
        assert!(
            d_step <= max_step_rate,
            "Discrete step rate {d_step:.3} km/km at alt {} km exceeds limit {max_step_rate}",
            alt_km_list[i]
        );
        assert!(
            alphas[i].is_finite() && (0.0..=1.0).contains(&alphas[i]),
            "Rendered alpha {} at alt {} km must be finite and within [0, 1]",
            alphas[i], alt_km_list[i]
        );
    }
    assert!(
        alphas[n_alts - 1].is_finite() && (0.0..=1.0).contains(&alphas[n_alts - 1]),
        "Rendered alpha at final alt must be finite and within [0, 1]"
    );
    eprintln!("[GPU RENDER VERIFIED] production_camera_ascent_continuity_across_full_transition_on_hardware: PASSED\n");
}


