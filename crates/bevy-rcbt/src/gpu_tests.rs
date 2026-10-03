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
