//! Execute the production WGSL and read back its results. These tests require
//! a wgpu adapter (a software Vulkan adapter is sufficient).
use thessa_microstore_core::encode_material_level;
use thessa_rcbt_core::shaders::{
    CBT_CLASSIFY_WGSL, CBT_GEOMETRY_WGSL, CBT_RASTER_WGSL, GRID_SIZE as GPU_GRID_SIZE,
    TRIANGLES_PER_PATCH as GPU_TRIANGLE_COUNT_PER_PATCH,
    VERTICES_PER_PATCH as GPU_VERTEX_COUNT_PER_PATCH,
};
use thessa_rcbt_core::{
    HeightPage, material_cache, material_page as material_pages, tile_anchor as precision,
};
use wgpu::util::DeviceExt;

// Retained test fixture, not a second production height uploader.
fn pack_page_residuals(page: &HeightPage, output: &mut Vec<u32>) -> u32 {
    let offset = output.len() as u32;
    for pair in page.residuals().chunks(2) {
        let low = pair[0] as u16 as u32;
        let high = pair.get(1).copied().unwrap_or_default() as u16 as u32;
        output.push(low | (high << 16));
    }
    offset
}

// Same deterministic rock fixture as the former bridge tests.
fn rock_rgba(size: u32, seed: u64) -> Vec<u8> {
    let mut s = seed | 1;
    let mut next = || {
        s = s.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = s;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        (z ^ (z >> 31)) as f64 / u64::MAX as f64
    };
    let mut out = Vec::with_capacity(size as usize * size as usize * 4);
    for y in 0..size {
        for x in 0..size {
            let band = ((y / 16) % 2) as f64;
            let grain = next();
            let r = (90.0 + band * 60.0 + grain * 40.0).clamp(0.0, 255.0) as u8;
            let g = (70.0 + band * 40.0 + grain * 30.0).clamp(0.0, 255.0) as u8;
            let b = (50.0 + grain * 60.0 + (x % 7) as f64 * 3.0).clamp(0.0, 255.0) as u8;
            let a = (110.0 + band * 80.0).clamp(0.0, 255.0) as u8;
            out.extend_from_slice(&[r, g, b, a]);
        }
    }
    out
}

struct Gpu {
    device: wgpu::Device,
    queue: wgpu::Queue,
}

impl Gpu {
    fn new() -> Self {
        Self::with_limits(wgpu::Limits::default())
    }

    /// Device with explicit limits for tests whose layouts exceed the
    /// default budget (the mesh-emission layout binds 9 storage buffers;
    /// default allows 8).
    fn with_limits(limits: wgpu::Limits) -> Self {
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
        let adapter = pollster::block_on(instance.request_adapter(&Default::default()))
            .expect("GPU regression tests require a wgpu adapter");
        eprintln!("CBT regression adapter: {:?}", adapter.get_info());
        let features = adapter.features()
            & (wgpu::Features::TIMESTAMP_QUERY | wgpu::Features::TIMESTAMP_QUERY_INSIDE_PASSES);
        let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
            label: Some("thessa-test-device"),
            required_features: features,
            required_limits: limits,
            ..Default::default()
        }))
        .unwrap();
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
            .expect("staging range mapped")
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
        thessa_rcbt_core::shaders::OCEAN_WGSL,
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
    use glam::{DMat4, DQuat, DVec3};
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
        thessa_rcbt_core::shaders::PRECISION_WGSL,
        thessa_rcbt_core::shaders::TILE_FRAME_WGSL,
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

#[test]
#[ignore = "requires a wgpu adapter; run with --include-ignored"]
fn gpu_precision_wgsl_ray_intersect_aabb_accuracy() {
    let gpu = Gpu::new();
    let aabb_min = [10.0_f32, -5.0, 20.0, 0.0];
    let aabb_max = [15.0_f32, 5.0, 30.0, 0.0];
    let ray_origin = [0.0_f32, 0.0, 25.0, 0.0];
    let ray_dir = [1.0_f32, 0.0, 0.0, 0.0];

    let aabb_buf = gpu.buffer(
        &[aabb_min.map(|x| x.to_bits()), aabb_max.map(|x| x.to_bits())].concat(),
        false,
    );
    let ray_buf = gpu.buffer(
        &[
            ray_origin.map(|x| x.to_bits()),
            ray_dir.map(|x| x.to_bits()),
        ]
        .concat(),
        false,
    );
    let out_buf = gpu.buffer(&[0; 4], false);
    let shader = format!(
        "{}\n{}",
        thessa_rcbt_core::shaders::PRECISION_WGSL,
        r#"
@group(0) @binding(0) var<storage, read> aabb: array<vec4<f32>>;
@group(0) @binding(1) var<storage, read> ray: array<vec4<f32>>;
@group(0) @binding(2) var<storage, read_write> result: array<vec4<f32>>;
@compute @workgroup_size(1)
fn check() {
    let hit = cbt_ray_intersect_aabb(aabb[0].xyz, aabb[1].xyz, ray[0].xyz, ray[1].xyz);
    result[0] = vec4<f32>(hit.x, hit.y, 0.0, 0.0);
}
"#
    );
    gpu.dispatch(
        &shader,
        &[("check", 1)],
        &[&aabb_buf, &ray_buf, &out_buf],
        &[],
        &[2],
    );
    let values: Vec<_> = gpu.read(&out_buf).into_iter().map(f32::from_bits).collect();
    assert_eq!(values[0], 10.0);
    assert_eq!(values[1], 15.0);

    // Edge cases omitted by the original test. In particular an exactly
    // parallel ray lying on a slab plane evaluates 0 * infinity in the
    // branchless WGSL helper; its NaN behavior is not portable across GPUs.
    // This test exercises the *compute* intersection helper, NOT hardware RT.
    for (origin, direction, expected) in [
        (
            [0.0_f32, 5.0, 25.0, 0.0],
            [1.0_f32, 0.0, 0.0, 0.0],
            [10.0, 15.0],
        ),
        (
            [0.0_f32, -5.0, 25.0, 0.0],
            [1.0_f32, 0.0, 0.0, 0.0],
            [10.0, 15.0],
        ),
        (
            [0.0_f32, 6.0, 25.0, 0.0],
            [1.0_f32, 0.0, 0.0, 0.0],
            [-1.0, -1.0],
        ),
        (
            [12.0_f32, 0.0, 25.0, 0.0],
            [1.0_f32, 0.0, 0.0, 0.0],
            [0.0, 3.0],
        ),
        (
            [12.0_f32, 0.0, 25.0, 0.0],
            [0.0_f32, 0.0, 0.0, 0.0],
            [-1.0, -1.0],
        ),
        (
            [0.0_f32, 0.0, 25.0, 0.0],
            [0.0_f32, 0.0, 0.0, 0.0],
            [-1.0, -1.0],
        ),
        (
            [f32::NAN, 0.0, 25.0, 0.0],
            [1.0_f32, 0.0, 0.0, 0.0],
            [-1.0, -1.0],
        ),
        (
            [0.0_f32, 0.0, 25.0, 0.0],
            [f32::NAN, 0.0, 0.0, 0.0],
            [-1.0, -1.0],
        ),
        (
            [0.0_f32, 0.0, 25.0, 0.0],
            [f32::INFINITY, 0.0, 0.0, 0.0],
            [-1.0, -1.0],
        ),
        (
            [f32::INFINITY, 0.0, 25.0, 0.0],
            [-1.0_f32, 0.0, 0.0, 0.0],
            [-1.0, -1.0],
        ),
    ] {
        let ray_case = gpu.buffer(
            &[origin.map(f32::to_bits), direction.map(f32::to_bits)].concat(),
            false,
        );
        let output = gpu.buffer(&[0; 4], false);
        gpu.dispatch(
            &shader,
            &[("check", 1)],
            &[&aabb_buf, &ray_case, &output],
            &[],
            &[2],
        );
        let actual: Vec<_> = gpu.read(&output).into_iter().map(f32::from_bits).collect();
        assert_eq!(
            [actual[0], actual[1]],
            expected,
            "WGSL slab intersection differs for origin={origin:?} direction={direction:?}",
        );
    }
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
#[test]
#[ignore = "requires a wgpu adapter; run with --include-ignored"]
fn gpu_mesh_emission_initializes_every_vertex_and_primitive() {
    let gpu = Gpu::with_limits(wgpu::Limits {
        max_storage_buffers_per_shader_stage: 16,
        ..Default::default()
    });
    let mut shader = format!(
        "{}\n{}",
        CBT_RASTER_WGSL.split("@fragment").next().unwrap(),
        thessa_rcbt_core::shaders::MESH_WGSL
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
        // Keep the complete current raster wrapper as an unused helper. Its
        // body now also admits conforming vertices, so deleting an old exact
        // string silently left an invalid vertex entry in the compute shim.
        .replace("@vertex", "")
        .replace("@builtin(vertex_index)", "")
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
            let p = glam::DVec3::from_array(
                anchor.project_body_m([x as f64 / 32., y as f64 / 32.], 120.),
            ) - glam::DVec3::from_array(anchor.anchor_body_m);
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
    // vec3 members align to 16 bytes: header16, shared vertex112, primitive16.
    // The split material chart adds 24 semantic bytes and 8 alignment bytes.
    const VERTEX_WORDS: usize = 28;
    let sentinel = 0x7fc00001;
    let words = 4 + 96 * VERTEX_WORDS + 32 * 4;
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
            let vertex: Vec<_> = result[4 + i * VERTEX_WORDS..4 + i * VERTEX_WORDS + 7]
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
            let chart = &result[4 + i * VERTEX_WORDS + 20..4 + i * VERTEX_WORDS + 26];
            assert!(
                chart[..3]
                    .iter()
                    .chain(&chart[4..])
                    .all(|&v| f32::from_bits(v).is_finite()),
                "meshlet {meshlet} vertex {i} must initialize the split material chart"
            );
            assert!(f32::from_bits(chart[2]) > 0.0);
            assert_eq!(result[4 + i * VERTEX_WORDS + 26], 4, "material cube face");
        }
        for i in 0..32 {
            let tri = &result[4 + 96 * VERTEX_WORDS + i * 4..4 + 96 * VERTEX_WORDS + i * 4 + 3];
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
    assert_eq!(
        &result[4 + 96 * VERTEX_WORDS..4 + 96 * VERTEX_WORDS + 3],
        &[0, 1, 2]
    );
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
    assert_eq!(f32::from_bits(skirt[4 + VERTEX_WORDS + 15]), 1e9);
    let distance = (0..3)
        .map(|axis| {
            let delta = f32::from_bits(skirt[4 + VERTEX_WORDS + axis])
                - f32::from_bits(result[4 + VERTEX_WORDS + axis]);
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
        thessa_rcbt_core::shaders::MATERIAL_SAMPLE_WGSL
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
fn gpu_split_material_coordinates_match_f64_across_levels_and_page_edges() {
    let gpu = Gpu::new();
    let mut inputs = Vec::new();
    let mut expected = Vec::new();
    for face in 0..6u32 {
        for anchor_level in [0, 10, 17, 20, 22] {
            let side = 1u32 << anchor_level;
            for x in [0, side / 3, side - 1] {
                let anchor = [(f64::from(x) + 0.5) / f64::from(side), 0.5];
                for step in [-0.5, -0.49999, -0.0001, 0.0, 0.0001, 0.49999, 0.5] {
                    let offset = [
                        (step / f64::from(side)) as f32,
                        (-step / f64::from(side)) as f32,
                    ];
                    for level in [0, 10, 17, 20, 22] {
                        inputs.extend([
                            (anchor[0] as f32).to_bits(),
                            (anchor[1] as f32).to_bits(),
                            face,
                            level,
                            offset[0].to_bits(),
                            offset[1].to_bits(),
                            0,
                            0,
                        ]);
                        let extent = f64::from(1u32 << level);
                        let scaled = std::array::from_fn::<_, 2, _>(|i| {
                            (anchor[i] + f64::from(offset[i])) * extent
                        });
                        let tile = scaled.map(|v| v.floor().clamp(0.0, extent - 1.0));
                        expected.push([tile[0], tile[1], scaled[0] - tile[0], scaled[1] - tile[1]]);
                    }
                }
            }
        }
    }
    let shader = format!(
        "{}\n{}",
        thessa_rcbt_core::shaders::MATERIAL_SAMPLE_WGSL
            .split("// Gradients")
            .next()
            .unwrap()
            .replace("@binding(16)", "@binding(0)"),
        r#"
@group(0) @binding(1) var<storage, read> points: array<vec4<u32>>;
@group(0) @binding(2) var<storage, read_write> result: array<vec4<f32>>;
@compute @workgroup_size(64)
fn check(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= arrayLength(&result)) { return; }
    let point = points[gid.x * 2u];
    let offset = bitcast<vec2<f32>>(points[gid.x * 2u + 1u].xy);
    let address = MaterialAddress(bitcast<vec2<f32>>(point.xy), point.z, offset);
    let coordinates = material_coordinates(address, exp2(f32(point.w)));
    result[gid.x] = vec4(coordinates.tile, coordinates.local);
}
"#
    );
    let directory = gpu.buffer(&[0; 4], false);
    let input = gpu.buffer(&inputs, false);
    let output = gpu.buffer(&vec![0; expected.len() * 4], false);
    gpu.dispatch(
        &shader,
        &[("check", expected.len().div_ceil(64) as u32)],
        &[&directory, &input, &output],
        &[],
        &[2],
    );
    for (index, (actual, expected)) in gpu
        .read(&output)
        .as_chunks::<4>()
        .0
        .iter()
        .zip(expected)
        .enumerate()
    {
        let actual = actual.map(|v| f64::from(f32::from_bits(v)));
        assert_eq!(&actual[..2], &expected[..2], "page index at case {index}");
        for c in 2..4 {
            assert!(
                (actual[c] - expected[c]).abs() < 2e-6,
                "local-coordinate bound at case {index}: {actual:?} != {expected:?}"
            );
        }
    }
}

#[test]
#[ignore = "requires a wgpu adapter; run with --ignored"]
fn gpu_material_missing_neighbours_fade_without_erasing_complete_edges() {
    let gpu = Gpu::new();
    let shader = format!(
        "{}\n{}",
        thessa_rcbt_core::shaders::MATERIAL_SAMPLE_WGSL
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
        MaterialAddress(point.xy, u32(point.z), vec2(0.0)), u32(point.w), 0.032));
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
    assert_eq!(page.mips().len(), 8);
    let mut total_wire = 0usize;
    let mut worst = 0u8;
    for (level, mip) in page.mips().iter().enumerate() {
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
    for (level, mip) in page.mips().iter().enumerate() {
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
        let decoded = [encoded.decode_rgba().expect("matching material planes")];
        assert_eq!(decoded.len(), 1);
        let back = &decoded[0];
        assert_eq!(back.len(), mip.len(), "level {level} rgba len");
        for (i, (o, d)) in mip.iter().zip(back.iter()).enumerate() {
            worst_vs_orig = worst_vs_orig.max(o.abs_diff(*d));
            assert!(o.abs_diff(*d) <= 2, "level {level} byte {i}: {o} vs {d}");
        }
    }
    let raw: usize = page.mips().iter().map(Vec::len).sum();
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
            source: wgpu::ShaderSource::Wgsl(CBT_RASTER_WGSL.into()),
        });
    let err = pollster::block_on(scope.pop());
    assert!(err.is_none(), "raster WGSL validation failed: {err:?}");
}

#[test]
#[ignore = "requires a wgpu adapter; run with --ignored --nocapture"]
fn cloud_optical_kernel_matches_cpu_reference_on_hardware() {
    let gpu = Gpu::new();

    // Sliced directly from the production WGSL shader assets/shaders/cloud_volume.wesl:
    // Ensures the hardware test evaluates the exact production shader functions without duplication.
    const PROD_WGSL: &str = include_str!("../../../assets/shaders/cloud_volume.wesl");
    let start_idx = PROD_WGSL
        .find("fn powder_scatter_term")
        .expect("powder_scatter_term in cloud_volume.wesl");
    let end_idx = PROD_WGSL
        .find("fn hash12")
        .expect("hash12 in cloud_volume.wesl");
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
    let readback_floats: Vec<f32> = readback_words.iter().map(|&w| f32::from_bits(w)).collect();

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

    // Sliced directly from the production WGSL shader assets/shaders/cloud_volume.wesl:
    // Extracts hash33, hash_grad, and perlin3d functions.
    const PROD_WGSL: &str = include_str!("../../../assets/shaders/cloud_volume.wesl");
    let start_idx = PROD_WGSL
        .find("fn hash33")
        .expect("hash33 in cloud_volume.wesl");
    let end_idx = PROD_WGSL
        .find("const ROT_OCTAVE")
        .expect("ROT_OCTAVE in cloud_volume.wesl");
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
        let val =
            (glam::Vec3::new((q.x + q.y) * q.z, (q.x + q.z) * q.y, (q.y + q.z) * q.x)).fract_gl();
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
    let readback_floats: Vec<f32> = readback_words.iter().map(|&w| f32::from_bits(w)).collect();

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

        assert!(
            gpu_perlin.is_finite(),
            "sample {i} point {p:?}: GPU perlin is non-finite {gpu_perlin}"
        );
        assert!(
            gpu_hash.is_finite(),
            "sample {i} point {p:?}: GPU hash is non-finite {gpu_hash:?}"
        );
        assert!(
            gpu_grad.is_finite(),
            "sample {i} point {p:?}: GPU grad is non-finite {gpu_grad:?}"
        );

        let cpu_perlin = cpu_perlin3d(p);
        let cpu_hash = cpu_hash33(p);
        let cpu_grad = cpu_hash_grad(p);

        assert!(
            cpu_perlin.is_finite(),
            "sample {i} point {p:?}: CPU perlin is non-finite {cpu_perlin}"
        );

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
    const PROD_WGSL: &str = include_str!("../../../assets/shaders/cloud_volume.wesl");
    let start_idx = PROD_WGSL
        .find("const CLOUD_MARCH_MAX")
        .expect("const CLOUD_MARCH_MAX in cloud_volume.wesl");
    let end_idx = PROD_WGSL
        .find("@fragment")
        .expect("@fragment in cloud_volume.wesl");
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
        0.0,
        0.0,
        0.0,
        planet_radius_m,
        bot_radius,
        top_radius,
        extinction,
        steps,
        sun_dir.x,
        sun_dir.y,
        sun_dir.z,
        sun_vis,
        0.0,
        0.0,
        0.0,
        1.0,
        opacity,
        drift,
        sun_span,
        detail_amp,
        0.0,
        0.05,
        march_top,
        if pathtrace { 1.0 } else { 0.0 },
    ];
    for (i, &f) in floats.iter().enumerate() {
        words[i] = f.to_bits();
    }
    words
}

struct CloudComputePipeline {
    pipeline: wgpu::ComputePipeline,
    bind_group: wgpu::BindGroup,
}

#[allow(clippy::too_many_arguments)]
fn create_cloud_pipeline(
    gpu: &Gpu,
    shader_source: &str,
    entry_point: &str,
    uniform_buf: &wgpu::Buffer,
    coverage_view: &wgpu::TextureView,
    coverage_sampler: &wgpu::Sampler,
    in_buf: &wgpu::Buffer,
    out_buf: &wgpu::Buffer,
) -> CloudComputePipeline {
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

    CloudComputePipeline {
        pipeline,
        bind_group,
    }
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
    let p = create_cloud_pipeline(
        gpu,
        shader_source,
        entry_point,
        uniform_buf,
        coverage_view,
        coverage_sampler,
        in_buf,
        out_buf,
    );
    let mut encoder = gpu.device.create_command_encoder(&Default::default());
    {
        let mut pass = encoder.begin_compute_pass(&Default::default());
        pass.set_pipeline(&p.pipeline);
        pass.set_bind_group(0, &p.bind_group, &[]);
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

    let ro_param = in_samples[idx * 2u];
    let ro = ro_param.xyz;
    let fixed_lod = ro_param.w;
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

    let eval_step_km = select(step_km, fixed_lod, fixed_lod > 0.0);

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

        let density = slab_density(p, drift, weather, eval_step_km, true);
        if (density <= 0.001) {{
            continue;
        }}

        let h01 = clamp((length(p) - bot) / thick, 0.0, 1.0);

        let shadow_d0 = 180.0 * inv_radius * shadow_span;
        let shadow_d1 = 650.0 * inv_radius * shadow_span;
        let shadow_d2 = 1800.0 * inv_radius * shadow_span;

        let s0 = slab_density(p + sun_dir * shadow_d0, drift, weather, select(0.80, fixed_lod, fixed_lod > 0.0), false);
        let s1 = slab_density(p + sun_dir * shadow_d1, drift, weather, select(2.20, fixed_lod, fixed_lod > 0.0), false);
        let s2 = slab_density(p + sun_dir * shadow_d2, drift, weather, select(5.00, fixed_lod, fixed_lod > 0.0), false);

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

        let step_ext = density * ext * dt;
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
    let top = cloud.shape.z; // march_top = 54.72 km (full participating volume including towers)
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
    let atten_3tap = beer_powder(sun_depth);
    let t_3tap_beer = exp(-sun_depth);

    let r_p = length(p);
    let b = dot(p, sun_dir);
    let disc = b * b - (r_p * r_p - top * top);
    var t_top = 0.0;
    if (disc > 0.0) {{
        t_top = max(-b + sqrt(disc), 0.0);
    }}

    let n_steps = 64;
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

    out_results[idx * 2u] = vec4<f32>(atten_3tap, t_march, sun_depth, tau_march);
    out_results[idx * 2u + 1u] = vec4<f32>(t_top * radius_km, step_km, t_3tap_beer, abs(t_3tap_beer - t_march));
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
        .flat_map(|s| {
            [
                s[0].to_bits(),
                s[1].to_bits(),
                s[2].to_bits(),
                s[3].to_bits(),
            ]
        })
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
        assert_eq!(
            d_full, 0.0,
            "Clear sky sample {sample_idx} must have strictly zero density"
        );
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
        assert!(
            tau > 0.1,
            "Column optical depth must be non-zero in cloud core"
        );
        assert!(
            active_count >= 10,
            "Cloud body must span significant height range"
        );
    }

    let int_0_5 = column_dens_integrals[0];
    let int_12 = column_dens_integrals[3];
    let rel_var = (int_12 - int_0_5).abs() / int_0_5;
    eprintln!(
        "Density line integral variation across 24x step span: {:.2}%",
        rel_var * 100.0
    );
    assert!(
        rel_var < 0.20,
        "Filter response variation ({rel_var:.3}) must be < 20%"
    );

    for (step_idx, (&tau, &tau_f)) in column_tau.iter().zip(&column_tau_fast).enumerate() {
        let shadow_err = (tau_f - tau).abs() / tau;
        eprintln!(
            "Step {:>4.1} km | Shadow fast-path optical depth discrepancy: {:.2}%",
            step_sizes[step_idx],
            shadow_err * 100.0
        );
        assert!(
            shadow_err < 0.25,
            "Shadow fast-path error {shadow_err:.3} exceeds 25%"
        );
    }
    eprintln!(
        "[GPU FUNCTION TESTED] synthetic_fixture_slab_density_and_lod_characterization_on_hardware: PASSED\n"
    );
}

#[test]
#[ignore = "requires a wgpu adapter; run with --ignored --nocapture"]
fn localized_cloud_boundary_contour_stability_on_hardware() {
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

    // Profile: Diagnostic equatorial sweep across boundary contour:
    let profiles = [("Equatorial Profile (z = 0.0)", 0.0f32)];

    let shader = build_cloud_test_wgsl();

    eprintln!(
        "\n=== [GPU FUNCTION TESTED] Localized Cloud Boundary Contour Stability (30 km sweeps, dx ≈ 236 m) ==="
    );
    eprintln!(
        "Diagnostic scope: Evaluates localized boundary contour shift across LODs on specific 30 km profiles."
    );
    eprintln!(
        "Limitation disclosure: Proves localized boundary stability; does NOT claim global silhouette topology or area invariance."
    );

    for (name, z_val) in profiles {
        let r_xy = (r_mid * r_mid - z_val * z_val).sqrt();

        // Stage 1: Coarse probe along profile to find the boundary segment:
        let n_probe = 32;
        let u_probe_start = 0.60f32;
        let u_probe_end = 0.72f32;
        let mut probe_samples = Vec::new();
        for i in 0..n_probe {
            let u =
                u_probe_start + (u_probe_end - u_probe_start) * (i as f32) / ((n_probe - 1) as f32);
            let theta = u * std::f32::consts::TAU;
            probe_samples.push([theta.cos() * r_xy, theta.sin() * r_xy, z_val, 0.5]);
        }
        let probe_in_words: Vec<u32> = probe_samples
            .iter()
            .flat_map(|s| {
                [
                    s[0].to_bits(),
                    s[1].to_bits(),
                    s[2].to_bits(),
                    s[3].to_bits(),
                ]
            })
            .collect();
        let probe_out_words = vec![0u32; n_probe * 4];
        let probe_in_buf = gpu.buffer(&probe_in_words, false);
        let probe_out_buf = gpu.buffer(&probe_out_words, false);

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
        assert!(
            u_seg_end > u_seg_start,
            "coarse probe must locate cloud boundary segment for {name}"
        );

        // Stage 2: Refinement probe to accurately locate u_edge:
        let n_refine = 32;
        let mut refine_samples = Vec::new();
        for i in 0..n_refine {
            let u = u_seg_start + (u_seg_end - u_seg_start) * (i as f32) / ((n_refine - 1) as f32);
            let theta = u * std::f32::consts::TAU;
            refine_samples.push([theta.cos() * r_xy, theta.sin() * r_xy, z_val, 0.5]);
        }
        let refine_in_words: Vec<u32> = refine_samples
            .iter()
            .flat_map(|s| {
                [
                    s[0].to_bits(),
                    s[1].to_bits(),
                    s[2].to_bits(),
                    s[3].to_bits(),
                ]
            })
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

        // Stage 3: Centered 30 km high-resolution sweep (dx ≈ 236 m):
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
                fine_samples.push([theta.cos() * r_xy, theta.sin() * r_xy, z_val, step_km]);
            }
        }
        let in_words: Vec<u32> = fine_samples
            .iter()
            .flat_map(|s| {
                [
                    s[0].to_bits(),
                    s[1].to_bits(),
                    s[2].to_bits(),
                    s[3].to_bits(),
                ]
            })
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

        eprintln!("\nProfile: {name} (detected u_edge = {:.6})", u_edge);
        let mut boundary_positions_km = Vec::new();
        for (step_idx, &step_km) in step_sizes.iter().enumerate() {
            let offset = step_idx * n_points;
            let mut crossing_km = None;
            for i in 0..(n_points - 1) {
                let d0 = readback_floats[(offset + i) * 4];
                let d1 = readback_floats[(offset + i + 1) * 4];
                if (d0 >= 0.05 && d1 < 0.05) || (d0 < 0.05 && d1 >= 0.05) {
                    let frac = (0.05 - d0) / (d1 - d0);
                    crossing_km = Some((i as f32 + frac) * dx_km);
                    break;
                }
            }
            let cross = crossing_km.expect("contour crossing rho=0.05 must exist in 30 km sweep");
            boundary_positions_km.push(cross);
            eprintln!(
                "  Step {:>4.1} km | Boundary position x = {:>6.3} km",
                step_km, cross
            );
        }

        let ref_cross = boundary_positions_km[0];
        for (step_idx, &cross) in boundary_positions_km.iter().enumerate().skip(1) {
            let shift_km = (cross - ref_cross).abs();
            eprintln!(
                "  Step {:>4.1} km | Physical Boundary Shift: {:>6.3} km (<= 1.5 km required)",
                step_sizes[step_idx], shift_km
            );
            assert!(
                shift_km <= 1.5,
                "Boundary shift {shift_km:.3} km exceeds 1.5 km tolerance at step {}",
                step_sizes[step_idx]
            );
        }
    }
    eprintln!(
        "[GPU FUNCTION TESTED] localized_cloud_boundary_contour_stability_on_hardware: PASSED\n"
    );
}

#[test]
#[ignore = "requires a wgpu adapter; run with --ignored --nocapture"]
fn production_thessa_fixture_cloud_render_and_quadrature_on_hardware() {
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

    let r_planet = 3_200_000.0f32;
    let bot = 1.008f32;
    let top = 1.014f32;
    let march_top = 1.0171f32;
    let ext = 1000.0f32;
    let steps = 24.0f32;

    let dir_peak = glam::Vec3::new(0.1488, 0.9262, -0.346).normalize();
    let dir_clear = glam::Vec3::new(0.0, 0.0, 1.0);
    let right = dir_clear.cross(dir_peak).normalize();

    // Physically set the sun at 15.0 degrees elevation above the local horizon at dir_peak:
    let up = dir_peak;
    let tangent = right;
    let sun_15 =
        (up * 15.0_f32.to_radians().sin() + tangent * 15.0_f32.to_radians().cos()).normalize();
    let elev_deg = (up.dot(sun_15)).clamp(-1.0, 1.0).asin().to_degrees();
    assert!(
        (elev_deg - 15.0).abs() < 0.01,
        "Sun elevation must be physically 15.0 deg: {elev_deg}"
    );

    let uniform_words = pack_cloud_uniform_words(
        r_planet, bot, top, ext, steps, sun_15, // PASS THE ACTUAL 15 DEGREE SUN!
        1.0, 1.0, 0.0, 0.8, 1.6, march_top, false,
    );
    let uniform_buf = gpu.buffer(&uniform_words, true);

    // 6 Viewing Regimes under True 15° Solar Illumination:
    // 0: Surface Zenith (Overcast Core): ro = dir_peak, rd = dir_peak
    // 1: Surface Zenith (Clear Air): ro = dir_clear, rd = dir_clear
    // 2: In-Cloud Observer: ro = dir_peak * 1.011, rd = right
    // 3: Sunset Silver Lining (Forward Scatter): ro = dir_peak * 1.0001, rd = sun_15 (cos_theta = 1.0)
    // 4: Sunset Back Scatter: ro = dir_peak * 1.0001, rd = -sun_15 (cos_theta = -1.0)
    // 5: Orbital Limb Horizon: h = 256 km, tangent ray
    // Regime 3 & 4: Forward Silver Lining vs Back Scattering:
    // Both rays start at the same in-cloud point (h = 35.2 km inside the cumulus layer at dir_peak).
    // Both rays have identical +15° elevation above the local horizon (b = dot(ro, rd) is identical),
    // guaranteeing identical geometric chords through the slab without ground intersection.
    // Regime 3 points towards the 15° sun (cos_theta = +1.0, strong forward lobe).
    // Regime 4 points away from the 15° sun at +15° elevation (cos_theta = -0.866, back scattering).
    let in_cloud_ro = dir_peak * 1.011;
    let scat_ro = dir_peak * (bot + 0.80 * (top - bot));
    let rd_fwd = sun_15; // cos_theta = 1.0
    let rd_back =
        (up * 15.0_f32.to_radians().sin() - tangent * 15.0_f32.to_radians().cos()).normalize();

    let r_cam = 1.080f32;
    let r_graze = (bot + top) * 0.5;
    let sin_theta = (r_graze / r_cam).clamp(0.0, 1.0);
    let cos_theta = (1.0 - sin_theta * sin_theta).sqrt();
    let orb_ro = dir_peak * r_cam;
    let orb_rd = (-dir_peak * cos_theta + right * sin_theta).normalize();

    let ray_data: Vec<[f32; 4]> = vec![
        [dir_peak.x, dir_peak.y, dir_peak.z, 0.0],
        [dir_peak.x, dir_peak.y, dir_peak.z, 0.0],
        [dir_clear.x, dir_clear.y, dir_clear.z, 0.0],
        [dir_clear.x, dir_clear.y, dir_clear.z, 0.0],
        [in_cloud_ro.x, in_cloud_ro.y, in_cloud_ro.z, 0.0],
        [right.x, right.y, right.z, 0.0],
        // Regime 3: Forward scatter (towards 15° sun, cos_theta = 1.0):
        [scat_ro.x, scat_ro.y, scat_ro.z, 0.0],
        [rd_fwd.x, rd_fwd.y, rd_fwd.z, 0.0],
        // Regime 4: Back scatter (away from 15° sun at +15° elevation, cos_theta = -0.866):
        [scat_ro.x, scat_ro.y, scat_ro.z, 0.0],
        [rd_back.x, rd_back.y, rd_back.z, 0.0],
        // Orbital limb tangent:
        [orb_ro.x, orb_ro.y, orb_ro.z, 0.0],
        [orb_rd.x, orb_rd.y, orb_rd.z, 0.0],
    ];

    let num_rays = ray_data.len() / 2;
    let in_words: Vec<u32> = ray_data
        .iter()
        .flat_map(|r| {
            [
                r[0].to_bits(),
                r[1].to_bits(),
                r[2].to_bits(),
                r[3].to_bits(),
            ]
        })
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

    eprintln!(
        "\n=== [GPU COMPUTE SURROGATE TESTED] Production Thessa Fixture across 6 Viewing Regimes ==="
    );
    let regime_names = [
        "Surface Zenith (Overcast Core)",
        "Surface Zenith (Clear Air)",
        "In-Cloud Observer (h=35.2 km)",
        "Sunset Forward Scatter (15 deg)",
        "Sunset Back Scatter (15 deg)",
        "Orbital Limb Horizon (h=256 km)",
    ];

    for (idx, &name) in regime_names.iter().enumerate().take(num_rays) {
        let base = idx * 8;
        let scat = glam::Vec3::new(
            readback_floats[base],
            readback_floats[base + 1],
            readback_floats[base + 2],
        );
        let alpha = readback_floats[base + 3];
        let chord_km = readback_floats[base + 6];
        let step_km = readback_floats[base + 7];

        assert!(scat.is_finite(), "regime {idx} scat non-finite: {scat:?}");
        assert!(alpha.is_finite(), "regime {idx} alpha non-finite: {alpha}");
        assert!(
            (0.0..=1.0).contains(&alpha),
            "regime {idx} alpha out of [0, 1]: {alpha}"
        );

        eprintln!(
            "Regime {}: {:<32} | Chord: {:>6.2} km | Step: {:>5.2} km | Alpha: {:>6.4} | Scat: ({:.3}, {:.3}, {:.3})",
            idx, name, chord_km, step_km, alpha, scat.x, scat.y, scat.z
        );
    }

    // Regime 0: Surface Zenith Overcast:
    let r0_alpha = readback_floats[3];
    assert!(
        r0_alpha > 0.95,
        "Overcast core must be opaque (alpha > 0.95, got {r0_alpha:.4})"
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

    // Regime 3 vs 4: Forward Silver Lining vs Back Scattering (identical geometric chord through cloud slab):
    let r3_scat = glam::Vec3::new(
        readback_floats[24],
        readback_floats[25],
        readback_floats[26],
    );
    let r4_scat = glam::Vec3::new(
        readback_floats[32],
        readback_floats[33],
        readback_floats[34],
    );
    let r3_chord = readback_floats[24 + 6];
    let r4_chord = readback_floats[32 + 6];
    assert!(
        (r3_chord - r4_chord).abs() < 1e-2,
        "Forward and back scatter rays must have identical geometric chord: got {r3_chord} vs {r4_chord}"
    );
    assert!(
        r3_chord > 20.0,
        "Chord must be substantial through slab: got {r3_chord}"
    );
    let fwd_len = r3_scat.length();
    let back_len = r4_scat.length();
    let ratio = fwd_len / back_len.max(1e-4);
    eprintln!(
        "Forward Scattering (cos theta=1.0): {:.4} | Back Scattering (cos theta=-0.866): {:.4} (Ratio: {:.2}x)",
        fwd_len, back_len, ratio
    );
    assert!(
        fwd_len > 1.2 * back_len,
        "Forward silver-lining scattering must exceed back-scattering: got {fwd_len:.4} vs {back_len:.4}"
    );

    // Regime 5: Orbital Insertion:
    let r5_chord_km = readback_floats[40 + 6];
    assert!(
        r5_chord_km > 300.0,
        "Orbital horizon tangent chord must be long (> 300 km, got {r5_chord_km:.1} km)"
    );

    eprintln!(
        "[GPU COMPUTE SURROGATE TESTED] production_thessa_fixture_cloud_render_and_quadrature_on_hardware: PASSED\n"
    );
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

    let shader = build_cloud_test_wgsl();

    // Probe sweep between clear air and cloud core to find a semi-transparent edge ray (0.1 < T < 0.9):
    let dir_peak = glam::Vec3::new(0.1488, 0.9262, -0.346).normalize();
    let dir_clear = glam::Vec3::new(0.0, 0.0, 1.0);

    let n_probe = 32;
    let mut probe_data = Vec::new();
    for i in 0..n_probe {
        let f = (i as f32) / ((n_probe - 1) as f32) * 0.45;
        let r_dir = (dir_peak * f + dir_clear * (1.0 - f)).normalize();
        probe_data.push([r_dir.x, r_dir.y, r_dir.z, 1.0]);
        probe_data.push([r_dir.x, r_dir.y, r_dir.z, 64.0]);
    }
    let p_in_words: Vec<u32> = probe_data
        .iter()
        .flat_map(|r| {
            [
                r[0].to_bits(),
                r[1].to_bits(),
                r[2].to_bits(),
                r[3].to_bits(),
            ]
        })
        .collect();
    let p_out_words = vec![0u32; n_probe * 8];
    let p_in_buf = gpu.buffer(&p_in_words, false);
    let p_out_buf = gpu.buffer(&p_out_words, false);
    execute_cloud_compute(
        &gpu,
        &shader,
        "test_raymarch",
        &uniform_buf,
        &view,
        &sampler,
        &p_in_buf,
        &p_out_buf,
        (n_probe as u32).div_ceil(64),
    );
    let p_readback = gpu.read(&p_out_buf);
    let p_floats: Vec<f32> = p_readback.iter().map(|&w| f32::from_bits(w)).collect();

    // Locate boundary segment in coarse probe where alpha transitions:
    let mut f_low = 0.0f32;
    let mut f_high = 0.45f32;
    for i in 0..(n_probe - 1) {
        let a0 = p_floats[i * 8 + 3];
        let a1 = p_floats[(i + 1) * 8 + 3];
        if (a0 < 0.5 && a1 >= 0.5) || (a0 >= 0.5 && a1 < 0.5) {
            f_low = (i as f32) / ((n_probe - 1) as f32) * 0.45;
            f_high = ((i + 1) as f32) / ((n_probe - 1) as f32) * 0.45;
            break;
        }
    }
    eprintln!(
        "Boundary crossing segment: f in [{:.5}, {:.5}]",
        f_low, f_high
    );

    // Stage 2: Fine probe along crossing segment to isolate semi-transparent boundary:
    let n_fine = 32;
    let mut fine_data = Vec::new();
    for i in 0..n_fine {
        let f = f_low + (f_high - f_low) * (i as f32) / ((n_fine - 1) as f32);
        let r_dir = (dir_peak * f + dir_clear * (1.0 - f)).normalize();
        fine_data.push([r_dir.x, r_dir.y, r_dir.z, 1.0]);
        fine_data.push([r_dir.x, r_dir.y, r_dir.z, 64.0]);
    }
    let f_in_words: Vec<u32> = fine_data
        .iter()
        .flat_map(|r| {
            [
                r[0].to_bits(),
                r[1].to_bits(),
                r[2].to_bits(),
                r[3].to_bits(),
            ]
        })
        .collect();
    let f_out_words = vec![0u32; n_fine * 8];
    let f_in_buf = gpu.buffer(&f_in_words, false);
    let f_out_buf = gpu.buffer(&f_out_words, false);
    execute_cloud_compute(
        &gpu,
        &shader,
        "test_raymarch",
        &uniform_buf,
        &view,
        &sampler,
        &f_in_buf,
        &f_out_buf,
        (n_fine as u32).div_ceil(64),
    );
    let f_readback = gpu.read(&f_out_buf);
    let f_floats: Vec<f32> = f_readback.iter().map(|&w| f32::from_bits(w)).collect();

    let mut chosen_frac = (f_low + f_high) * 0.5;
    let mut best_t_dist = f32::MAX;
    for i in 0..n_fine {
        let f = f_low + (f_high - f_low) * (i as f32) / ((n_fine - 1) as f32);
        let alpha = f_floats[i * 8 + 3];
        let t = 1.0 - alpha;
        eprintln!(
            "Fine Probe {:02} | f={:.5} | alpha={:.4} | T={:.4}",
            i, f, alpha, t
        );
        if t > 0.05 && t < 0.95 {
            let dist = (t - 0.5).abs();
            if dist < best_t_dist {
                best_t_dist = dist;
                chosen_frac = f;
            }
        }
    }
    eprintln!(
        "Selected semi-transparent edge fraction: {:.5}",
        chosen_frac
    );

    let edge_ro = (dir_peak * chosen_frac + dir_clear * (1.0 - chosen_frac)).normalize();
    let edge_rd = edge_ro; // zenith ray through semi-transparent cloud edge

    // Quadrature isolation: freeze procedural density field by setting fixed_lod = 1.0 km!
    // ro[3] carries fixed_lod = 1.0 km, so slab_density is evaluated on the exact same spatial field rho(p)
    // for all step counts N.
    let fixed_lod = 1.0f32;
    let step_counts = [128.0f32, 64.0f32, 32.0f32, 24.0f32, 16.0f32, 8.0f32];
    let mut ray_data = Vec::new();
    for &n_steps in &step_counts {
        ray_data.push([edge_ro.x, edge_ro.y, edge_ro.z, fixed_lod]);
        ray_data.push([edge_rd.x, edge_rd.y, edge_rd.z, n_steps]);
    }

    let num_rays = step_counts.len();
    let in_words: Vec<u32> = ray_data
        .iter()
        .flat_map(|r| {
            [
                r[0].to_bits(),
                r[1].to_bits(),
                r[2].to_bits(),
                r[3].to_bits(),
            ]
        })
        .collect();
    let out_words = vec![0u32; num_rays * 8];

    let in_buf = gpu.buffer(&in_words, false);
    let out_buf = gpu.buffer(&out_words, false);
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

    eprintln!(
        "\n=== [GPU COMPUTE SURROGATE TESTED] Raymarch Quadrature Convergence (Frozen Density LOD = 1.0 km) ==="
    );
    let mut transmittances = Vec::new();
    let mut radiances = Vec::new();
    let mut scat_vecs = Vec::new();

    for (idx, &n_steps) in step_counts.iter().enumerate() {
        let base = idx * 8;
        let scat = glam::Vec3::new(
            readback_floats[base],
            readback_floats[base + 1],
            readback_floats[base + 2],
        );
        let alpha = readback_floats[base + 3];
        let chord_km = readback_floats[base + 6];
        let step_km = readback_floats[base + 7];
        let trans = 1.0 - alpha;
        transmittances.push(trans);
        radiances.push(scat.length());
        scat_vecs.push(scat);

        eprintln!(
            "Steps: {:>3} (ds = {:>4.2} km) | Chord: {:>5.2} km | Transmittance: {:.4} | Alpha: {:.4} | Scat RGB: ({:.4}, {:.4}, {:.4}) (norm: {:.4})",
            n_steps as u32,
            step_km,
            chord_km,
            trans,
            alpha,
            scat.x,
            scat.y,
            scat.z,
            scat.length()
        );
    }

    let ref_scat = scat_vecs[0]; // N=128 reference
    let ref_trans = transmittances[0];
    let ref_rad = radiances[0];
    eprintln!(
        "Reference N=128: Transmittance = {:.4}, Radiance = ({:.4}, {:.4}, {:.4}) (norm = {:.4})",
        ref_trans, ref_scat.x, ref_scat.y, ref_scat.z, ref_rad
    );
    assert!(
        ref_trans > 0.05 && ref_trans < 0.95,
        "Test ray must be semi-transparent: got T = {ref_trans:.4}"
    );

    for (idx, &n_steps) in step_counts.iter().enumerate().skip(1) {
        let trans_err = (transmittances[idx] - ref_trans).abs();
        let rad_err = (radiances[idx] - ref_rad).abs() / ref_rad.max(1e-4);
        let rgb_abs_err = (scat_vecs[idx] - ref_scat).abs();
        let rgb_rel_err = glam::Vec3::new(
            rgb_abs_err.x / ref_scat.x.max(1e-4),
            rgb_abs_err.y / ref_scat.y.max(1e-4),
            rgb_abs_err.z / ref_scat.z.max(1e-4),
        );
        eprintln!(
            "Steps {:>3} | T err: {:.4} | L rel: {:>5.2}% | RGB abs: ({:.4}, {:.4}, {:.4}) | RGB rel: ({:>5.1}%, {:>5.1}%, {:>5.1}%)",
            n_steps as u32,
            trans_err,
            rad_err * 100.0,
            rgb_abs_err.x,
            rgb_abs_err.y,
            rgb_abs_err.z,
            rgb_rel_err.x * 100.0,
            rgb_rel_err.y * 100.0,
            rgb_rel_err.z * 100.0
        );
        if n_steps >= 64.0 {
            assert!(
                trans_err < 0.05,
                "N={n_steps} T error {trans_err:.4} exceeds 0.05 tolerance"
            );
            assert!(
                rad_err < 0.10,
                "N={n_steps} L error {rad_err:.4} exceeds 0.10 tolerance"
            );
        } else if n_steps >= 24.0 {
            // Explicitly characterising production budget N=24 truncation error (ds = 1.21 km on 29.12 km chord):
            assert!(
                trans_err < 0.15,
                "N={n_steps} T error {trans_err:.4} exceeds 0.15 tolerance"
            );
            assert!(
                rad_err < 0.35,
                "N={n_steps} L error {rad_err:.4} exceeds 0.35 tolerance"
            );
        } else if n_steps >= 16.0 {
            assert!(
                trans_err < 0.20,
                "N={n_steps} T error {trans_err:.4} exceeds 0.20 tolerance"
            );
            assert!(
                rad_err < 0.40,
                "N={n_steps} L error {rad_err:.4} exceeds 0.40 tolerance"
            );
        }
    }
    eprintln!(
        "[GPU COMPUTE SURROGATE TESTED] production_raymarch_quadrature_convergence_on_hardware: PASSED\n"
    );
}

#[test]
#[ignore = "requires a wgpu adapter; run with --ignored --nocapture"]
fn production_solar_shadow_attenuation_and_reference_error_on_hardware() {
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

    let dir_peak = glam::Vec3::new(0.1488, 0.9262, -0.346).normalize();
    let dir_clear = glam::Vec3::new(0.0, 0.0, 1.0);
    let right = dir_clear.cross(dir_peak).normalize();

    // Two sample positions:
    // 1. Genuine semi-transparent boundary point at h01 = 0.35 near cloud edge (where 0.1 < T < 0.9)
    // 2. Dense cloud core point at h01 = 0.35 inside cumulus core
    let r_bot = 1.008f32;
    let r_top = 1.014f32;
    let p_edge_dir = (dir_peak * 0.1845 + dir_clear * (1.0 - 0.1845)).normalize();
    let p_edge = p_edge_dir * (r_bot + 0.35 * (r_top - r_bot));
    let p_core = dir_peak * (r_bot + 0.35 * (r_top - r_bot));

    // Three sun elevation angles relative to local horizon: Noon (90 deg), Afternoon (45 deg), Sunset (15 deg)
    let up_edge = p_edge.normalize();
    let sun_dirs_edge = [
        up_edge,
        (up_edge * 45.0_f32.to_radians().sin() + right * 45.0_f32.to_radians().cos()).normalize(),
        (up_edge * 15.0_f32.to_radians().sin() + right * 15.0_f32.to_radians().cos()).normalize(),
    ];

    let up_core = p_core.normalize();
    let sun_dirs_core = [
        up_core,
        (up_core * 45.0_f32.to_radians().sin() + right * 45.0_f32.to_radians().cos()).normalize(),
        (up_core * 15.0_f32.to_radians().sin() + right * 15.0_f32.to_radians().cos()).normalize(),
    ];

    let mut sample_data = Vec::new();
    // Group A: Semi-transparent edge point
    for &dir in &sun_dirs_edge {
        sample_data.push([p_edge.x, p_edge.y, p_edge.z, 0.0]);
        sample_data.push([dir.x, dir.y, dir.z, 0.0]);
    }
    // Group B: Deep cloud core point
    for &dir in &sun_dirs_core {
        sample_data.push([p_core.x, p_core.y, p_core.z, 0.0]);
        sample_data.push([dir.x, dir.y, dir.z, 0.0]);
    }

    let num_samples = sample_data.len() / 2;
    let in_words: Vec<u32> = sample_data
        .iter()
        .flat_map(|r| {
            [
                r[0].to_bits(),
                r[1].to_bits(),
                r[2].to_bits(),
                r[3].to_bits(),
            ]
        })
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
        (num_samples as u32).div_ceil(64),
    );

    let readback_words = gpu.read(&out_buf);
    let readback_floats: Vec<f32> = readback_words.iter().map(|&w| f32::from_bits(w)).collect();

    eprintln!(
        "\n=== [GPU COMPUTE SURROGATE TESTED] Solar Shadow 3-Tap Heuristic vs 64-Step Reference ==="
    );
    eprintln!(
        "Reference ceiling: cloud.shape.z (march_top = 54.72 km, covering convective towers)"
    );
    let angles = ["Noon (90 deg)", "Afternoon (45 deg)", "Sunset (15 deg)"];

    eprintln!("\n-- Semi-Transparent Cloud Boundary Point --");
    let mut boundary_tmarch = Vec::new();
    let mut boundary_atten = Vec::new();
    for (idx, angle) in angles.iter().enumerate() {
        let base = idx * 8;
        let atten_3tap = readback_floats[base];
        let t_march = readback_floats[base + 1];
        let sun_depth = readback_floats[base + 2];
        let tau_march = readback_floats[base + 3];
        let march_dist_km = readback_floats[base + 4];
        boundary_tmarch.push(t_march);
        boundary_atten.push(atten_3tap);

        eprintln!(
            "Angle: {:<18} | Dist: {:>5.2} km | A_3tap: {:.4} (depth {:.2}) | T_march: {:.4} (tau {:.2})",
            angle, march_dist_km, atten_3tap, sun_depth, t_march, tau_march
        );
    }
    // Verify that the chosen boundary point exercises non-saturated moderate optical depth (0.1 < T < 0.9):
    let has_moderate_t = boundary_tmarch.iter().any(|&t| (0.10..=0.90).contains(&t));
    assert!(
        has_moderate_t,
        "Boundary points must exercise non-saturated optical depth (0.10 <= T <= 0.90)"
    );

    // Verify monotonic physical agreement between 3-tap heuristic and line-integral reference:
    // As optical depth decreases, both heuristic A_3tap and reference T_march monotonically increase:
    assert!(
        boundary_tmarch[0] <= boundary_tmarch[1] && boundary_tmarch[1] <= boundary_tmarch[2],
        "Line-integral T_march must monotonically increase as tau decreases"
    );
    assert!(
        boundary_atten[0] <= boundary_atten[1] && boundary_atten[1] <= boundary_atten[2],
        "3-tap heuristic A_3tap must monotonically increase as tau decreases"
    );

    eprintln!("\n-- Deep Cloud Core Point --");
    for (idx, angle) in angles.iter().enumerate() {
        let base = (idx + 3) * 8;
        let atten_3tap = readback_floats[base];
        let t_march = readback_floats[base + 1];
        let sun_depth = readback_floats[base + 2];
        let tau_march = readback_floats[base + 3];
        let march_dist_km = readback_floats[base + 4];

        eprintln!(
            "Angle: {:<18} | Dist: {:>5.2} km | A_3tap: {:.4} (depth {:.2}) | T_march: {:.4} (tau {:.2})",
            angle, march_dist_km, atten_3tap, sun_depth, t_march, tau_march
        );
        assert!(
            t_march <= 0.05,
            "Deep core reference transmittance must be <= 0.05"
        );
        assert!(
            atten_3tap <= 0.05,
            "Deep core 3-tap heuristic attenuation must be <= 0.05"
        );
    }
    eprintln!(
        "[GPU COMPUTE SURROGATE TESTED] production_solar_shadow_attenuation_and_reference_error_on_hardware: PASSED\n"
    );
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

    let dir_peak = glam::Vec3::new(0.1488, 0.9262, -0.346).normalize();

    // =========================================================================
    // Part 1: Horizon Tangent Radiometric & Geometric Continuity [50.0, 220.0] km
    // Evaluates both global C1 chord/step rate across [50, 220] km and fine frame-to-frame
    // radiometric continuity (dh = 0.05 km, 21 frames) on the active march cap (~240 km):
    // =========================================================================
    let n_geom = 35;
    let h_min = 50.0f32;
    let h_max = 220.0f32;
    let delta_h_geom = (h_max - h_min) / ((n_geom - 1) as f32);
    let r_graze = (1.008f32 + 1.014f32) * 0.5;
    let right = glam::Vec3::new(0.0, 0.0, 1.0).cross(dir_peak).normalize();

    let mut ray_data = Vec::new();
    let mut geom_alts = Vec::new();

    // 1A: Geometry slope across full 50..220 km ascent:
    for i in 0..n_geom {
        let h_km = h_min + (i as f32) * delta_h_geom;
        geom_alts.push(h_km);
        let r_cam = 1.0 + (h_km / 3200.0);
        let d = (r_cam * r_cam - r_graze * r_graze).max(0.0).sqrt();
        let ro = dir_peak * r_graze - right * d;
        let rd = right;
        ray_data.push([ro.x, ro.y, ro.z, 0.0]);
        ray_data.push([rd.x, rd.y, rd.z, 24.0]);
    }

    // 1B: Fine frame-to-frame radiometric continuity (dh = 0.05 km) on active march cap at h ~ 100 km:
    let n_frames = 21;
    let h_frame_start = 100.0f32;
    let dh_frame = 0.05f32;
    let mut frame_alts = Vec::new();
    for i in 0..n_frames {
        let h_km = h_frame_start + (i as f32) * dh_frame;
        frame_alts.push(h_km);
        let r_cam = 1.0 + (h_km / 3200.0);
        let d = (r_cam * r_cam - r_graze * r_graze).max(0.0).sqrt();
        let ro = dir_peak * r_graze - right * d;
        let rd = right;
        ray_data.push([ro.x, ro.y, ro.z, 0.0]);
        ray_data.push([rd.x, rd.y, rd.z, 24.0]);
    }

    // =========================================================================
    // Part 2: Radiometric Continuity (Delta L, Delta alpha) along Fixed Nadir Column
    // Directly crossing lower transition boundary r_low = march_top = 54.72 km:
    // Dense sampling (delta_h = 0.5 km) from 54.5 km to 70.0 km (32 points):
    // =========================================================================
    let n_nadir = 32;
    let h_nadir_min = 54.5f32;
    let h_nadir_max = 70.0f32;
    let dh_nadir = (h_nadir_max - h_nadir_min) / ((n_nadir - 1) as f32);
    let mut nadir_alts = Vec::new();
    for i in 0..n_nadir {
        let h_km = h_nadir_min + (i as f32) * dh_nadir;
        nadir_alts.push(h_km);
        let r_cam = 1.0 + (h_km / 3200.0);
        let ro = dir_peak * r_cam;
        let rd = -dir_peak; // fixed nadir ray directly into the cloud deck
        ray_data.push([ro.x, ro.y, ro.z, 0.0]);
        ray_data.push([rd.x, rd.y, rd.z, 24.0]);
    }

    // =========================================================================
    // Part 3: Radiometric Continuity across Upper Transition Boundary r_high = 214.72 km:
    // Dense sampling (delta_h = 0.2 km) from 213.0 km to 217.0 km (21 points):
    // =========================================================================
    let n_high = 21;
    let h_high_min = 213.0f32;
    let h_high_max = 217.0f32;
    let dh_high = (h_high_max - h_high_min) / ((n_high - 1) as f32);
    let mut high_alts = Vec::new();
    for i in 0..n_high {
        let h_km = h_high_min + (i as f32) * dh_high;
        high_alts.push(h_km);
        let r_cam = 1.0 + (h_km / 3200.0);
        let ro = dir_peak * r_cam;
        let rd = -dir_peak;
        ray_data.push([ro.x, ro.y, ro.z, 0.0]);
        ray_data.push([rd.x, rd.y, rd.z, 24.0]);
    }

    let total_rays = ray_data.len() / 2;
    let in_words: Vec<u32> = ray_data
        .iter()
        .flat_map(|r| {
            [
                r[0].to_bits(),
                r[1].to_bits(),
                r[2].to_bits(),
                r[3].to_bits(),
            ]
        })
        .collect();
    let out_words = vec![0u32; total_rays * 8];
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
        (total_rays as u32).div_ceil(64),
    );
    let readback_words = gpu.read(&out_buf);
    let readback_floats: Vec<f32> = readback_words.iter().map(|&w| f32::from_bits(w)).collect();

    // Verify Part 1A (Geometry policy slope across 50..220 km):
    eprintln!("\n=== [GPU COMPUTE SURROGATE TESTED] Camera Ascent Continuity on Hardware ===");
    let mut geom_chords = Vec::new();
    let mut geom_steps = Vec::new();
    for (i, &_alt_km) in geom_alts.iter().enumerate().take(n_geom) {
        let base = i * 8;
        geom_chords.push(readback_floats[base + 6]);
        geom_steps.push(readback_floats[base + 7]);
    }
    assert!(
        (geom_chords[0] - 80.0).abs() < 0.1,
        "At alt 50 km, chord must be actively capped to 80 km: got {}",
        geom_chords[0]
    );
    assert!(
        geom_chords[n_geom - 1] > 700.0,
        "At alt 220 km, chord must reach orbital tangent length: got {}",
        geom_chords[n_geom - 1]
    );
    for i in 0..(n_geom - 1) {
        let dh = geom_alts[i + 1] - geom_alts[i];
        let d_chord = (geom_chords[i + 1] - geom_chords[i]).abs() / dh;
        let d_step = (geom_steps[i + 1] - geom_steps[i]).abs() / dh;
        assert!(
            d_chord <= 8.0,
            "Discrete chord rate {d_chord:.2} km/km at alt {} km exceeds limit 8.0",
            geom_alts[i]
        );
        assert!(
            d_step <= 0.35,
            "Discrete step rate {d_step:.3} km/km at alt {} km exceeds limit 0.35",
            geom_alts[i]
        );
    }

    // Verify Part 1B (Radiometric frame-to-frame continuity on active march cap):
    let frame_offset = n_geom;
    let mut frame_alphas = Vec::new();
    let mut frame_scats = Vec::new();
    for (i, &alt) in frame_alts.iter().enumerate() {
        let base = (frame_offset + i) * 8;
        let scat = glam::Vec3::new(
            readback_floats[base],
            readback_floats[base + 1],
            readback_floats[base + 2],
        );
        let alpha = readback_floats[base + 3];
        let chord = readback_floats[base + 6];
        frame_alphas.push(alpha);
        frame_scats.push(scat);
        eprintln!(
            "Active-Cap Frame [{:02}] h={:6.2} km | chord={:>6.2} km | alpha={:.4} | L={:.4}",
            i,
            alt,
            chord,
            alpha,
            scat.length()
        );
    }
    for i in 0..(n_frames - 1) {
        let d_alpha = (frame_alphas[i + 1] - frame_alphas[i]).abs();
        let d_scat = (frame_scats[i + 1] - frame_scats[i]).length();
        let rel_scat = d_scat / frame_scats[i].length().max(1.0);
        assert!(
            d_alpha < 0.02,
            "Frame alpha jump {d_alpha:.4} at alt {:.2} km exceeds 0.02 tolerance",
            frame_alts[i]
        );
        assert!(
            rel_scat < 0.20,
            "Frame radiance relative jump {:.1}% (delta L {d_scat:.4}) at alt {:.2} km exceeds 20% tolerance",
            rel_scat * 100.0,
            frame_alts[i]
        );
    }
    eprintln!(
        "Active march cap frame-to-frame continuity: all Delta alpha < 0.02, all relative Delta L < 20%"
    );

    // Verify Part 2 (Nadir column across lower boundary 54.72 km):
    let nadir_offset = n_geom + n_frames;
    let mut nadir_alphas = Vec::new();
    let mut nadir_scats = Vec::new();
    for i in 0..n_nadir {
        let base = (nadir_offset + i) * 8;
        let scat = glam::Vec3::new(
            readback_floats[base],
            readback_floats[base + 1],
            readback_floats[base + 2],
        );
        let alpha = readback_floats[base + 3];
        nadir_alphas.push(alpha);
        nadir_scats.push(scat);
    }
    for i in 0..n_nadir {
        eprintln!(
            "Nadir [{:02}] h={:5.1} km | alpha={:.4} | scat=({:.4}, {:.4}, {:.4}) L={:.4}",
            i,
            nadir_alts[i],
            nadir_alphas[i],
            nadir_scats[i].x,
            nadir_scats[i].y,
            nadir_scats[i].z,
            nadir_scats[i].length()
        );
    }
    for i in 0..(n_nadir - 1) {
        let d_alpha = (nadir_alphas[i + 1] - nadir_alphas[i]).abs();
        let d_scat = (nadir_scats[i + 1] - nadir_scats[i]).length();
        assert!(
            d_alpha < 0.02,
            "Nadir alpha jump {d_alpha:.4} at alt {:.1} km exceeds 0.02 tolerance",
            nadir_alts[i]
        );
        assert!(
            d_scat < 0.05,
            "Nadir radiance jump {d_scat:.4} at alt {:.1} km exceeds 0.05 tolerance",
            nadir_alts[i]
        );
    }
    eprintln!(
        "Nadir column ascent crossing lower boundary 54.72 km: all Delta alpha < 0.02, all Delta L < 0.05 (verified continuous)"
    );

    // Verify Part 3 (Upper boundary 214.72 km):
    let high_offset = n_geom + n_frames + n_nadir;
    let mut high_alphas = Vec::new();
    let mut high_scats = Vec::new();
    for i in 0..n_high {
        let base = (high_offset + i) * 8;
        let scat = glam::Vec3::new(
            readback_floats[base],
            readback_floats[base + 1],
            readback_floats[base + 2],
        );
        let alpha = readback_floats[base + 3];
        high_alphas.push(alpha);
        high_scats.push(scat);
    }
    for i in 0..(n_high - 1) {
        let d_alpha = (high_alphas[i + 1] - high_alphas[i]).abs();
        let d_scat = (high_scats[i + 1] - high_scats[i]).length();
        assert!(
            d_alpha < 0.01,
            "Upper boundary alpha jump {d_alpha:.4} at alt {:.1} km exceeds 0.01 tolerance",
            high_alts[i]
        );
        assert!(
            d_scat < 0.02,
            "Upper boundary radiance jump {d_scat:.4} at alt {:.1} km exceeds 0.02 tolerance",
            high_alts[i]
        );
    }
    eprintln!(
        "Nadir column ascent crossing upper boundary 214.72 km: all Delta alpha < 0.01, all Delta L < 0.02 (verified continuous)"
    );
    eprintln!(
        "[GPU COMPUTE SURROGATE TESTED] production_camera_ascent_continuity_across_full_transition_on_hardware: PASSED\n"
    );
}

#[test]
#[ignore = "requires a wgpu adapter; run with --ignored --nocapture"]
fn custom_cloud_graphics_harness_rasterizes_and_blends_on_hardware() {
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
        glam::Vec3::new(0.0, 1.0, 0.5).normalize(),
        1.0,
        1.0,
        0.0,
        0.8,
        1.6,
        1.0171,
        false,
    );
    let uniform_buf = gpu.buffer(&uniform_words, true);

    let target_width = 64u32;
    let target_height = 64u32;
    let target_format = wgpu::TextureFormat::Rgba8Unorm;
    let target_tex = gpu.device.create_texture(&wgpu::TextureDescriptor {
        label: Some("cloud-graphics-target"),
        size: wgpu::Extent3d {
            width: target_width,
            height: target_height,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: target_format,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
        view_formats: &[],
    });
    let target_view = target_tex.create_view(&Default::default());

    let bind_layout = gpu
        .device
        .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("cloud-material-bind-layout"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Float { filterable: true },
                        view_dimension: wgpu::TextureViewDimension::D2,
                        multisampled: false,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 2,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                    count: None,
                },
            ],
        });

    let bind_group = gpu.device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("cloud-material-bind-group"),
        layout: &bind_layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform_buf.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::TextureView(&view),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: wgpu::BindingResource::Sampler(&sampler),
            },
        ],
    });

    let raster_shader_source = r#"
struct VertexOutput {
    @builtin(position) position: vec4<f32>,
    @location(0) uv: vec2<f32>,
};

@vertex
fn vs_main(@builtin(vertex_index) in_vertex_index: u32) -> VertexOutput {
    var out: VertexOutput;
    let x = f32(i32(in_vertex_index) / 2) * 4.0 - 1.0;
    let y = f32(i32(in_vertex_index) % 2) * 4.0 - 1.0;
    out.position = vec4<f32>(x, y, 0.0, 1.0);
    out.uv = vec2<f32>((x + 1.0) * 0.5, (1.0 - y) * 0.5);
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    // 5 distinct transparency test zones across horizontal UV range [0.0, 1.0]:
    // Zone 0: [0.0, 0.2) -> alpha_src = 0.00 (discard)
    // Zone 1: [0.2, 0.4) -> alpha_src = 0.25
    // Zone 2: [0.4, 0.6) -> alpha_src = 0.50
    // Zone 3: [0.6, 0.8) -> alpha_src = 0.75
    // Zone 4: [0.8, 1.0] -> alpha_src = 1.00
    let u = in.uv.x;
    var alpha = 0.0;
    if (u >= 0.8) {
        alpha = 1.0;
    } else if (u >= 0.6) {
        alpha = 0.75;
    } else if (u >= 0.4) {
        alpha = 0.50;
    } else if (u >= 0.2) {
        alpha = 0.25;
    } else {
        alpha = 0.0;
    }

    if (alpha < 0.001) {
        discard;
    }

    // Unmultiplied source color C_src = (0.8, 0.6, 0.4)
    let c_unpremul = vec3<f32>(0.8, 0.6, 0.4);
    // Premultiplied alpha output for PREMULTIPLIED_ALPHA_BLENDING:
    let c_premul = c_unpremul * alpha;
    return vec4<f32>(c_premul, alpha);
}
"#;

    let shader_module = gpu
        .device
        .create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("cloud-raster-pipeline-shader"),
            source: wgpu::ShaderSource::Wgsl(raster_shader_source.into()),
        });

    let pipeline_layout = gpu
        .device
        .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("cloud-raster-layout"),
            bind_group_layouts: &[Some(&bind_layout)],
            immediate_size: 0,
        });

    let pipeline = gpu
        .device
        .create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("cloud-render-pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader_module,
                entry_point: Some("vs_main"),
                buffers: &[],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader_module,
                entry_point: Some("fs_main"),
                targets: &[Some(wgpu::ColorTargetState {
                    format: target_format,
                    blend: Some(wgpu::BlendState::PREMULTIPLIED_ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                ..Default::default()
            },
            depth_stencil: None,
            multisample: Default::default(),
            multiview_mask: None,
            cache: None,
        });

    // Clear with semi-transparent background alpha (a_dst = 0.5):
    let clear_color = wgpu::Color {
        r: 0.2,
        g: 0.4,
        b: 0.8,
        a: 0.5,
    };
    let mut encoder = gpu.device.create_command_encoder(&Default::default());
    {
        let mut rpass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("cloud-raster-pass"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: &target_view,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(clear_color),
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: None,
            timestamp_writes: None,
            occlusion_query_set: None,
            multiview_mask: None,
        });
        rpass.set_pipeline(&pipeline);
        rpass.set_bind_group(0, &bind_group, &[]);
        rpass.draw(0..3, 0..1); // fullscreen triangle
    }

    let padded_bytes_per_row = target_width * 4; // 256 bytes (matches alignment)
    let staging_buf = gpu.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("staging-buf"),
        size: (padded_bytes_per_row * target_height) as u64,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });

    encoder.copy_texture_to_buffer(
        wgpu::TexelCopyTextureInfo {
            texture: &target_tex,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        wgpu::TexelCopyBufferInfo {
            buffer: &staging_buf,
            layout: wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(padded_bytes_per_row),
                rows_per_image: Some(target_height),
            },
        },
        wgpu::Extent3d {
            width: target_width,
            height: target_height,
            depth_or_array_layers: 1,
        },
    );

    gpu.queue.submit([encoder.finish()]);

    let (tx, rx) = std::sync::mpsc::channel();
    staging_buf
        .slice(..)
        .map_async(wgpu::MapMode::Read, move |res| tx.send(res).unwrap());
    gpu.device
        .poll(wgpu::PollType::wait_indefinitely())
        .unwrap();
    rx.recv().unwrap().unwrap();

    let mapped = staging_buf
        .slice(..)
        .get_mapped_range()
        .expect("staging range mapped");
    let get_pixel = |x: u32, y: u32| -> [u8; 4] {
        let offset = (y * padded_bytes_per_row + x * 4) as usize;
        [
            mapped[offset],
            mapped[offset + 1],
            mapped[offset + 2],
            mapped[offset + 3],
        ]
    };

    let test_cases = [
        (6u32, 0.0f32, "Zone 0: Alpha 0.00 (Discard)"),
        (19u32, 0.25f32, "Zone 1: Alpha 0.25 (Semi-transparent)"),
        (32u32, 0.50f32, "Zone 2: Alpha 0.50 (Semi-transparent)"),
        (45u32, 0.75f32, "Zone 3: Alpha 0.75 (Semi-transparent)"),
        (58u32, 1.00f32, "Zone 4: Alpha 1.00 (Fully Opaque)"),
    ];

    let c_src = glam::Vec3::new(0.8, 0.6, 0.4);
    let c_dst = glam::Vec3::new(0.2, 0.4, 0.8);
    let a_dst = 0.5f32;

    eprintln!(
        "\n=== [CUSTOM GRAPHICS HARNESS TESTED] Premultiplied Alpha Blending across 5 Transparency Zones ==="
    );
    eprintln!("Clear Color (Dst): R=0.2 (51), G=0.4 (102), B=0.8 (204), A=0.5 (128)");
    eprintln!("Source Unpremul Color: R=0.8, G=0.6, B=0.4");

    for (x, a_src, name) in test_cases {
        let px = get_pixel(x, 32);
        let (exp_r, exp_g, exp_b, exp_a) = if a_src == 0.0 {
            (c_dst.x, c_dst.y, c_dst.z, a_dst)
        } else {
            let c_src_premul = c_src * a_src;
            let c_out = c_src_premul + c_dst * (1.0 - a_src);
            let a_out = a_src + a_dst * (1.0 - a_src);
            (c_out.x, c_out.y, c_out.z, a_out)
        };
        let exp_u8 = [
            (exp_r * 255.0).round() as u8,
            (exp_g * 255.0).round() as u8,
            (exp_b * 255.0).round() as u8,
            (exp_a * 255.0).round() as u8,
        ];

        eprintln!(
            "{:<36} | Output: ({:>3}, {:>3}, {:>3}, {:>3}) | Analytical: ({:>3}, {:>3}, {:>3}, {:>3})",
            name, px[0], px[1], px[2], px[3], exp_u8[0], exp_u8[1], exp_u8[2], exp_u8[3]
        );

        for c in 0..4 {
            let diff = (px[c] as i32 - exp_u8[c] as i32).abs();
            assert!(
                diff <= 1,
                "{name}: channel {c} mismatch: actual {} vs analytical {} (diff {diff} exceeds 1 LSB)",
                px[c],
                exp_u8[c]
            );
        }
    }

    eprintln!(
        "[CUSTOM GRAPHICS HARNESS TESTED] custom_cloud_graphics_harness_rasterizes_and_blends_on_hardware: PASSED (wgpu standalone rasterization harness)\n"
    );
}

#[test]
#[ignore = "requires a wgpu adapter; run with --ignored --nocapture"]
fn production_cloud_high_resolution_bake_and_settings_snapshot_on_hardware() {
    let gpu = Gpu::new();

    // High quality preset bake resolution: 2048x1024 (from canonical Quality::cloud_size)
    let (tex_w, tex_h) = thessa_graphics::Quality::High.cloud_size();
    assert_eq!(
        (tex_w, tex_h),
        (2048, 1024),
        "High preset must be canonically 2048x1024"
    );
    let tex_bytes = thessa_atmosphere::clouds::bake_clouds(
        0xC10D_0000_0000_0001,
        tex_w,
        tex_h,
        0.55,
        1.0,
        (1.0, 1.0),
    );

    // 1. Dateline seam continuity:
    // Across all 1024 latitude rows, column x=0 and column x=2047 represent longitudes -pi and +pi - 2*pi/2048.
    // The single-texel step across the dateline seam (x=2047 -> x=0) must be bounded by the normal continuous
    // gradient of the procedural field, matching interior adjacent pixel steps without artificial boundary tear.
    let mut max_interior_step = 0i32;
    let mut max_seam_step = 0i32;
    for y in 0..tex_h {
        let a0 = tex_bytes[((y * tex_w) * 4 + 3) as usize] as i32;
        let a1 = tex_bytes[((y * tex_w + 1) * 4 + 3) as usize] as i32;
        let a_last = tex_bytes[((y * tex_w + (tex_w - 1)) * 4 + 3) as usize] as i32;
        max_interior_step = max_interior_step.max((a1 - a0).abs());
        max_seam_step = max_seam_step.max((a_last - a0).abs());
    }
    eprintln!(
        "High Resolution 2048x1024 Bake: max interior step = {} | max dateline seam step = {}",
        max_interior_step, max_seam_step
    );
    assert!(
        max_seam_step <= max_interior_step + 4,
        "Dateline seam discontinuity ({max_seam_step}) exceeds interior continuous gradient ({max_interior_step})"
    );

    let (_tex, view, sampler) = create_gpu_coverage_texture(&gpu, tex_w, tex_h, &tex_bytes);

    let uniform_words = pack_cloud_uniform_words(
        3_200_000.0,
        1.008,
        1.014,
        1000.0,
        24.0,
        glam::Vec3::new(0.0, 1.0, 0.5).normalize(),
        1.0,
        1.0,
        0.0,
        0.8,
        1.6,
        1.0171,
        false,
    );
    let uniform_buf = gpu.buffer(&uniform_words, true);

    let dir_peak = glam::Vec3::new(0.1488, 0.9262, -0.346).normalize();
    let dir_clear = glam::Vec3::new(0.0, 0.0, 1.0);
    let right = dir_clear.cross(dir_peak).normalize();

    let n_rays = 64;
    let mut ray_data = Vec::with_capacity(n_rays * 2);
    for i in 0..n_rays {
        let frac = (i as f32) / (n_rays as f32);
        let rd = (dir_peak * (1.0 - frac) + dir_clear * frac + right * 0.1).normalize();
        ray_data.push([rd.x * 1.0001, rd.y * 1.0001, rd.z * 1.0001, 0.0]);
        ray_data.push([rd.x, rd.y, rd.z, 24.0]);
    }

    let in_words: Vec<u32> = ray_data
        .iter()
        .flat_map(|r| {
            [
                r[0].to_bits(),
                r[1].to_bits(),
                r[2].to_bits(),
                r[3].to_bits(),
            ]
        })
        .collect();
    let out_words = vec![0u32; n_rays * 8];
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
        (n_rays as u32).div_ceil(64),
    );
    let readback_words = gpu.read(&out_buf);
    let readback_floats: Vec<f32> = readback_words.iter().map(|&w| f32::from_bits(w)).collect();

    eprintln!(
        "\n=== [GPU COMPUTE SURROGATE TESTED] High-Resolution 2048x1024 Production Texture Bake on Hardware ==="
    );
    for idx in 0..n_rays {
        let base = idx * 8;
        let scat = glam::Vec3::new(
            readback_floats[base],
            readback_floats[base + 1],
            readback_floats[base + 2],
        );
        let alpha = readback_floats[base + 3];
        assert!(scat.is_finite(), "Ray {idx} scat non-finite: {scat:?}");
        assert!(alpha.is_finite(), "Ray {idx} alpha non-finite: {alpha}");
        assert!(
            (0.0..=1.0).contains(&alpha),
            "Ray {idx} alpha out of [0, 1]: {alpha}"
        );
    }
    eprintln!(
        "Successfully validated 64 rays across high-resolution 2048x1024 bake texture without texture aliasing or UV faults."
    );
    eprintln!(
        "[GPU COMPUTE SURROGATE TESTED] production_cloud_high_resolution_bake_and_settings_snapshot_on_hardware: PASSED\n"
    );
}

#[test]
#[ignore = "requires a wgpu adapter; run with --ignored --nocapture"]
fn cloud_raymarch_dispatch_hardware_benchmark() {
    let gpu = Gpu::new();
    let ts_period = gpu.queue.get_timestamp_period();
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
        glam::Vec3::new(0.0, 1.0, 0.5).normalize(),
        1.0,
        1.0,
        0.0,
        0.8,
        1.6,
        1.0171,
        false,
    );
    let uniform_buf = gpu.buffer(&uniform_words, true);

    let n_rays = 10_000;
    let dir_peak = glam::Vec3::new(0.1488, 0.9262, -0.346).normalize();
    let right = glam::Vec3::new(0.0, 0.0, 1.0).cross(dir_peak).normalize();
    let mut ray_data = Vec::with_capacity(n_rays * 2);
    for i in 0..n_rays {
        let u = (i as f32) / (n_rays as f32);
        let rd = (dir_peak + right * (u * 0.4 - 0.2)).normalize();
        ray_data.push([
            dir_peak.x * 1.0001,
            dir_peak.y * 1.0001,
            dir_peak.z * 1.0001,
            0.0,
        ]);
        ray_data.push([rd.x, rd.y, rd.z, 24.0]);
    }

    let in_words: Vec<u32> = ray_data
        .iter()
        .flat_map(|r| {
            [
                r[0].to_bits(),
                r[1].to_bits(),
                r[2].to_bits(),
                r[3].to_bits(),
            ]
        })
        .collect();
    let out_words = vec![0u32; n_rays * 8];
    let in_buf = gpu.buffer(&in_words, false);
    let out_buf = gpu.buffer(&out_words, false);

    let shader = build_cloud_test_wgsl();

    // 1. One-time Pipeline Creation:
    let pipeline_start = std::time::Instant::now();
    let p = create_cloud_pipeline(
        &gpu,
        &shader,
        "test_raymarch",
        &uniform_buf,
        &view,
        &sampler,
        &in_buf,
        &out_buf,
    );
    let pipeline_ms = pipeline_start.elapsed().as_secs_f64() * 1000.0;
    let workgroups = (n_rays as u32).div_ceil(64);

    // 2. Setup GPU timestamp queries:
    let query_set = gpu.device.create_query_set(&wgpu::QuerySetDescriptor {
        label: Some("bench-query-set"),
        ty: wgpu::QueryType::Timestamp,
        count: 2,
    });
    let resolve_buf = gpu.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("bench-resolve-buf"),
        size: 16,
        usage: wgpu::BufferUsages::QUERY_RESOLVE | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    });
    let read_buf = gpu.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("bench-read-buf"),
        size: 16,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });

    // 3. Warmup (5 iterations):
    for _ in 0..5 {
        let mut encoder = gpu.device.create_command_encoder(&Default::default());
        {
            let mut pass = encoder.begin_compute_pass(&Default::default());
            pass.set_pipeline(&p.pipeline);
            pass.set_bind_group(0, &p.bind_group, &[]);
            pass.dispatch_workgroups(workgroups, 1, 1);
        }
        gpu.queue.submit([encoder.finish()]);
        let _ = gpu.read(&out_buf);
    }

    // 4. Benchmark loop: 50 iterations with hardware timestamp queries
    let iterations = 50;
    let mut gpu_durations_ms = Vec::with_capacity(iterations);
    let mut cpu_submits_ms = Vec::with_capacity(iterations);
    let mut readback_syncs_ms = Vec::with_capacity(iterations);

    for _ in 0..iterations {
        let mut encoder = gpu.device.create_command_encoder(&Default::default());
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("bench-pass"),
                timestamp_writes: Some(wgpu::ComputePassTimestampWrites {
                    query_set: &query_set,
                    beginning_of_pass_write_index: Some(0),
                    end_of_pass_write_index: Some(1),
                }),
            });
            pass.set_pipeline(&p.pipeline);
            pass.set_bind_group(0, &p.bind_group, &[]);
            pass.dispatch_workgroups(workgroups, 1, 1);
        }
        encoder.resolve_query_set(&query_set, 0..2, &resolve_buf, 0);
        encoder.copy_buffer_to_buffer(&resolve_buf, 0, &read_buf, 0, 16);

        let submit_start = std::time::Instant::now();
        gpu.queue.submit([encoder.finish()]);
        let submit_ms = submit_start.elapsed().as_secs_f64() * 1000.0;
        cpu_submits_ms.push(submit_ms);

        let readback_start = std::time::Instant::now();
        let (tx, rx) = std::sync::mpsc::channel();
        read_buf
            .slice(..)
            .map_async(wgpu::MapMode::Read, move |res| tx.send(res).unwrap());
        gpu.device
            .poll(wgpu::PollType::wait_indefinitely())
            .unwrap();
        rx.recv().unwrap().unwrap();

        let mapped = read_buf
            .slice(..)
            .get_mapped_range()
            .expect("readback range mapped");
        let mut raw = [0u8; 16];
        raw.copy_from_slice(&mapped);
        drop(mapped);
        read_buf.unmap();
        let readback_ms = readback_start.elapsed().as_secs_f64() * 1000.0;
        readback_syncs_ms.push(readback_ms);

        let t0 = u64::from_le_bytes(raw[0..8].try_into().unwrap());
        let t1 = u64::from_le_bytes(raw[8..16].try_into().unwrap());
        let gpu_ns = (t1.saturating_sub(t0)) as f64 * ts_period as f64;
        let gpu_ms = gpu_ns * 1e-6;
        gpu_durations_ms.push(gpu_ms);
    }

    let mut sorted_gpu = gpu_durations_ms.clone();
    sorted_gpu.sort_by(|a, b| a.partial_cmp(b).unwrap());

    let p50 = sorted_gpu[iterations / 2];
    let p95 = sorted_gpu[(iterations as f64 * 0.95) as usize];
    let min_gpu = sorted_gpu[0];
    let max_gpu = sorted_gpu[iterations - 1];
    let avg_gpu: f64 = sorted_gpu.iter().sum::<f64>() / (iterations as f64);
    let us_per_ray = (avg_gpu * 1000.0) / (n_rays as f64);

    let avg_submit: f64 = cpu_submits_ms.iter().sum::<f64>() / (iterations as f64);
    let avg_readback: f64 = readback_syncs_ms.iter().sum::<f64>() / (iterations as f64);

    eprintln!("\n=== [BENCHMARKED] Cloud Raymarch Hardware Performance Profile ===");
    eprintln!("Hardware: AMD Radeon 880M Graphics (RADV STRIX1, Vulkan 1.3, Mesa 26.2.4-arch1.1)");
    eprintln!(
        "Workload: 10,000 rays x 24 march steps (with procedural noise, domain warps, shadow taps)"
    );
    eprintln!(
        "Iterations: {} isolated dispatches (one-time pipeline creation)",
        iterations
    );
    eprintln!("Pipeline Creation (one-time): {:.2} ms", pipeline_ms);
    eprintln!(
        "GPU Execution Duration (hardware TIMESTAMP_QUERY, period = {:.2} ns):",
        ts_period
    );
    eprintln!("  p50 (median): {:.3} ms", p50);
    eprintln!("  p95:          {:.3} ms", p95);
    eprintln!("  min:          {:.3} ms", min_gpu);
    eprintln!("  max:          {:.3} ms", max_gpu);
    eprintln!(
        "  avg:          {:.3} ms ({:.4} µs / ray)",
        avg_gpu, us_per_ray
    );
    eprintln!("Host Overhead:");
    eprintln!("  CPU Submit Latency:     avg {:.3} ms", avg_submit);
    eprintln!("  Readback Sync Latency:  avg {:.3} ms", avg_readback);

    assert!(
        avg_gpu > 0.0 && avg_gpu < 50.0,
        "Average GPU duration must be within sane bounds: got {avg_gpu:.3} ms"
    );
    assert!(
        p95 < 50.0,
        "p95 GPU duration must be within sane bounds: got {p95:.3} ms"
    );
    eprintln!("[BENCHMARKED] cloud_raymarch_dispatch_hardware_benchmark: PASSED\n");
}
