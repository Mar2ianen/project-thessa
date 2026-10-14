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

#[test]
#[ignore = "requires a wgpu adapter; run with --ignored --nocapture"]
fn production_slab_density_and_lod_continuity_on_hardware() {
    let gpu = Gpu::new();

    // Sliced directly from the production WGSL shader assets/shaders/cloud_volume.wgsl:
    // Extracts quat_rotate, shell_mesh_uv, convective_cumulus_gradient, perlin3d,
    // cauliflower_fbm, edge_wisps, slab_density and all intermediate procedural math.
    const PROD_WGSL: &str = include_str!("../../../assets/shaders/cloud_volume.wgsl");
    let start_idx = PROD_WGSL
        .find("fn quat_rotate")
        .expect("quat_rotate in cloud_volume.wgsl");
    let end_idx = PROD_WGSL
        .find("@fragment")
        .expect("@fragment in cloud_volume.wgsl");
    let prod_functions = &PROD_WGSL[start_idx..end_idx];

    let shader = format!(
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

{prod_functions}

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

    out_results[idx] = vec4<f32>(d_full, d_fast, 0.0, 0.0);
}}
"#
    );

    // 1. Create coverage weather map texture (64x64 RGBA8Unorm) with realistic cloud island:
    let tex_size = 64u32;
    let mut tex_bytes = vec![0u8; (tex_size * tex_size * 4) as usize];
    for y in 0..tex_size {
        for x in 0..tex_size {
            let u = (x as f32 + 0.5) / (tex_size as f32);
            let v = (y as f32 + 0.5) / (tex_size as f32);
            // Center island at u=0.5, v=0.5 with radius 0.25 (coverage 0.8 at core, falls off to 0):
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

    let coverage_texture = gpu.device.create_texture(&wgpu::TextureDescriptor {
        label: Some("test-coverage-tex"),
        size: wgpu::Extent3d {
            width: tex_size,
            height: tex_size,
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
            texture: &coverage_texture,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        &tex_bytes,
        wgpu::TexelCopyBufferLayout {
            offset: 0,
            bytes_per_row: Some(tex_size * 4),
            rows_per_image: Some(tex_size),
        },
        wgpu::Extent3d {
            width: tex_size,
            height: tex_size,
            depth_or_array_layers: 1,
        },
    );

    let coverage_view = coverage_texture.create_view(&Default::default());
    let coverage_sampler = gpu.device.create_sampler(&wgpu::SamplerDescriptor {
        address_mode_u: wgpu::AddressMode::Repeat,
        address_mode_v: wgpu::AddressMode::ClampToEdge,
        mag_filter: wgpu::FilterMode::Linear,
        min_filter: wgpu::FilterMode::Linear,
        ..Default::default()
    });

    // 2. Uniform buffer with realistic Thessa planetary parameters:
    let mut cloud_uniform_words = vec![0u32; 64]; // 256 bytes padded
    let cloud_uniform_floats: [f32; 24] = [
        0.0, 0.0, 0.0, 3_200_000.0,       // center_radius
        1.000625, 1.0028125, 42.0, 24.0,   // slab (2 to 9 km above 3200 km planet)
        0.0, 1.0, 0.0, 1.0,                // sun
        0.0, 0.0, 0.0, 1.0,                // deck_to_local (identity quat)
        1.0, 0.0, 0.003, 1.6,              // params (opacity 1.0, detail 1.6)
        0.0, 0.05, 1.0, 0.0,               // shape (cumulus deck 0, cov_floor 0.05)
    ];
    for (i, &f) in cloud_uniform_floats.iter().enumerate() {
        cloud_uniform_words[i] = f.to_bits();
    }
    let uniform_buf = gpu.buffer(&cloud_uniform_words, true);

    // 3. Generate test sample points:
    let step_sizes = [0.5f32, 2.0f32, 6.0f32, 12.0f32];
    let r_bottom = 1.000625f32;
    let r_top = 1.0028125f32;
    let r_span = r_top - r_bottom;

    let mut samples: Vec<[f32; 4]> = Vec::new();

    // Group A: Vertical column through cloud island center (local dir [-1, 0, 0] -> u=0.5, v=0.5)
    // 32 height samples per step size:
    let h_steps = 32;
    for &step_km in &step_sizes {
        for i in 0..h_steps {
            let h01 = (i as f32) / ((h_steps - 1) as f32) * 1.1; // 0.0 to 1.1 (spans below LCL to above top)
            let r = r_bottom + h01 * r_span;
            samples.push([-r, 0.0, 0.0, step_km]);
        }
    }

    // Group B: Vertical column in clear sky (local dir [1, 0, 0] -> u=0.0, v=0.5, outside cloud)
    for &step_km in &step_sizes {
        for i in 0..h_steps {
            let h01 = (i as f32) / ((h_steps - 1) as f32);
            let r = r_bottom + h01 * r_span;
            samples.push([r, 0.0, 0.0, step_km]);
        }
    }

    // Group C: Horizontal slice across cloud edge at h01 = 0.40 (64 points across boundary)
    let boundary_steps = 64;
    let r_mid = r_bottom + 0.40 * r_span;
    for &step_km in &step_sizes {
        for i in 0..boundary_steps {
            // Sweep u from 0.30 to 0.70:
            let u = 0.30 + 0.40 * (i as f32) / ((boundary_steps - 1) as f32);
            let angle = u * std::f32::consts::TAU;
            let dir_x = angle.cos();
            let dir_y = angle.sin();
            samples.push([dir_x * r_mid, dir_y * r_mid, 0.0, step_km]);
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

    // 4. Build pipeline & dispatch:
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
            label: Some("slab-density-bind-layout"),
            entries: &[
                uniform_layout(0),
                texture_layout(1),
                sampler_layout(2),
                storage_layout(3, true),
                storage_layout(4, false),
            ],
        });

    let bind_group = gpu.device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("slab-density-bind-group"),
        layout: &bind_layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform_buf.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::TextureView(&coverage_view),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: wgpu::BindingResource::Sampler(&coverage_sampler),
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
            label: Some("cbt-production-slab-density-wgsl"),
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
            label: Some("test_slab_density"),
            layout: Some(&pipeline_layout),
            module: &module,
            entry_point: Some("test_slab_density"),
            compilation_options: Default::default(),
            cache: None,
        });

    let mut encoder = gpu.device.create_command_encoder(&Default::default());
    {
        let mut pass = encoder.begin_compute_pass(&Default::default());
        pass.set_pipeline(&pipeline);
        pass.set_bind_group(0, &bind_group, &[]);
        pass.dispatch_workgroups(num_samples.div_ceil(64) as u32, 1, 1);
    }
    gpu.queue.submit([encoder.finish()]);

    let readback_words = gpu.read(&out_buf);
    let readback_floats: Vec<f32> = readback_words
        .iter()
        .map(|&w| f32::from_bits(w))
        .collect();

    // 5. Assertions on hardware results:
    for i in 0..num_samples {
        let base = i * 4;
        let d_full = readback_floats[base];
        let d_fast = readback_floats[base + 1];
        assert!(d_full.is_finite(), "sample {i} d_full non-finite: {d_full}");
        assert!(d_fast.is_finite(), "sample {i} d_fast non-finite: {d_fast}");
        assert!(d_full >= 0.0, "sample {i} d_full negative: {d_full}");
        assert!(d_fast >= 0.0, "sample {i} d_fast negative: {d_fast}");
    }

    // Check Group B: Clear sky produces strictly zero density:
    let group_b_offset = step_sizes.len() * h_steps;
    for i in 0..(step_sizes.len() * h_steps) {
        let sample_idx = group_b_offset + i;
        let d_full = readback_floats[sample_idx * 4];
        assert_eq!(
            d_full, 0.0,
            "Clear sky sample {sample_idx} must have strictly zero density"
        );
    }

    // Check Group A: Column optical depth stability across step sizes (LOD):
    eprintln!("\n=== Production WGSL slab_density Hardware Column Optical Depth ===");
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
        let tau = sum_density * (7.0 / (h_steps as f32)); // Optical depth across 7 km slab
        let tau_fast = sum_density_fast * (7.0 / (h_steps as f32));
        column_tau.push(tau);
        column_tau_fast.push(tau_fast);
        eprintln!(
            "Step {:>4.1} km | Active Samples: {:>2}/{} | Tau(Full) = {:.4} | Tau(Fast Shadow) = {:.4}",
            step_km, active_count, h_steps, tau, tau_fast
        );
        assert!(tau > 0.5, "Column optical depth must be significant in cloud center");
        assert!(active_count >= 10, "Cloud body must span significant height range");
    }

    // Verify bounded optical depth variation across a 24x change in step size (0.5 to 12.0 km):
    let tau_0_5 = column_tau[0];
    let tau_12 = column_tau[3];
    let rel_tau_diff = (tau_12 - tau_0_5).abs() / tau_0_5;
    eprintln!("Optical depth variation across 24x step span: {:.2}%", rel_tau_diff * 100.0);
    assert!(
        rel_tau_diff < 0.35,
        "Column optical depth variation ({rel_tau_diff:.3}) must be < 35% across 24x step change"
    );

    // Verify shadow fast-path optical depth tracks full-path within 25%:
    for (step_idx, (&tau, &tau_f)) in column_tau.iter().zip(&column_tau_fast).enumerate() {
        let shadow_err = (tau_f - tau).abs() / tau;
        eprintln!(
            "Step {:>4.1} km | Shadow fast-path optical depth relative discrepancy = {:.2}%",
            step_sizes[step_idx], shadow_err * 100.0
        );
        assert!(
            shadow_err < 0.25,
            "Shadow fast-path error {shadow_err:.3} exceeds 25% at step {}", step_sizes[step_idx]
        );
    }

    // Check Group C: Boundary silhouette stability across step sizes:
    eprintln!("\n=== Cloud Silhouette Edge Position Across LODs ===");
    let group_c_offset = group_b_offset + step_sizes.len() * h_steps;
    let mut boundary_entries = Vec::new();
    for (step_idx, &step_km) in step_sizes.iter().enumerate() {
        let offset = group_c_offset + step_idx * boundary_steps;
        let mut first_cloud_idx = None;
        let mut last_cloud_idx = None;
        for i in 0..boundary_steps {
            let sample_idx = offset + i;
            let d = readback_floats[sample_idx * 4];
            if d > 0.001 {
                if first_cloud_idx.is_none() {
                    first_cloud_idx = Some(i);
                }
                last_cloud_idx = Some(i);
            }
        }
        let first = first_cloud_idx.expect("cloud entry boundary");
        let last = last_cloud_idx.expect("cloud exit boundary");
        boundary_entries.push((first, last));
        eprintln!(
            "Step {:>4.1} km | Cloud Boundary Range: index [{:>2} .. {:>2}] (span {})",
            step_km, first, last, last - first + 1
        );
    }

    // Boundary entry and exit must not shift by more than 3 sample points (~4.7% of boundary sweep) across LODs:
    // (At 0.5 km, high-frequency edge wisps extend boundary by ~1-2 cells, which smoothly contract as step increases).
    let (ref_first, ref_last) = boundary_entries[0];
    for (step_idx, &(first, last)) in boundary_entries.iter().enumerate().skip(1) {
        let d_first = (first as i32 - ref_first as i32).abs();
        let d_last = (last as i32 - ref_last as i32).abs();
        assert!(
            d_first <= 3,
            "Boundary entry shifted by {d_first} cells at step {}", step_sizes[step_idx]
        );
        assert!(
            d_last <= 3,
            "Boundary exit shifted by {d_last} cells at step {}", step_sizes[step_idx]
        );
    }
}
