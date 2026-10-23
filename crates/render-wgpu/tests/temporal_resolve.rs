//! Hardware oracle for the temporal-resolve compute pass.
//!
//! The fixture is intentionally simple: a checkerboard current image, constant
//! depth and flat previous history. Expected pixels come from the independent
//! `render-core` resolve contract, not from a screenshot preference.

use glam::{DMat4, DVec3, Vec3};
use thessa_render_core::temporal::JitterOffset;
use thessa_render_core::temporal_resolve::{
    DEFAULT_COLOR_SIGMA, DEFAULT_MIN_CURRENT_WEIGHT, HistoryCandidate, HistoryGrid, ResolveInput,
    ResolvePolicy, resolve,
};
use thessa_render_wgpu::{
    Backend, BackendError,
    temporal_resolve::{
        HISTORY_BYTES_PER_PIXEL, HISTORY_FORMAT, ResolveParams, TemporalResolvePass,
    },
    view::{ViewParams, ViewUniformBlock},
};
use wgpu::util::DeviceExt;

const SIZE: u32 = 16;

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

fn view() -> (ViewParams, ViewUniformBlock, ViewUniformBlock, JitterOffset) {
    let extent = [SIZE, SIZE];
    let params = ViewParams {
        world_from_view: DMat4::IDENTITY,
        clip_from_view: glam::dcamera::rh::proj::directx::perspective(1.0, 1.5, 0.1, 1_000.0),
        world_position_m: DVec3::ZERO,
        exposure: 1.0,
    };
    let jitter = JitterOffset::new([0.25, -0.25]);
    let current = ViewUniformBlock::new_jittered(&params, DVec3::ZERO, jitter, extent).unwrap();
    let previous = ViewUniformBlock::new(&params).unwrap();
    (params, current, previous, jitter)
}

fn params() -> ResolveParams {
    let (_, current, previous, jitter) = view();
    ResolveParams {
        current_world_from_clip: current.world_from_clip,
        previous_clip_from_world: previous.unjittered_clip_from_world,
        camera_offset: Vec3::ZERO,
        extent: [SIZE, SIZE],
        previous_valid: true,
        current_jitter: jitter,
        previous_jitter: JitterOffset::ZERO,
        depth_tolerance: 1e-4,
        color_sigma: DEFAULT_COLOR_SIGMA,
        min_current_weight: DEFAULT_MIN_CURRENT_WEIGHT,
        history_grid: HistoryGrid::ResolvedUnjittered,
        max_history_age: 64,
        neighborhood_width: 3,
    }
}

fn checker(x: u32, y: u32) -> [f32; 4] {
    let value = if (x + y).is_multiple_of(2) { 0.5 } else { 0.75 };
    [value, value, value, 1.0]
}

fn flat(value: f32, alpha: f32) -> [f32; 4] {
    [value, value, value, alpha]
}

fn half_bits(value: f32) -> u16 {
    let bits = value.to_bits();
    let sign = ((bits >> 16) & 0x8000) as u16;
    let exponent = ((bits >> 23) & 0xff) as i32;
    if exponent == 0 {
        return sign;
    }
    let half_exponent = exponent - 127 + 15;
    if half_exponent <= 0 {
        return sign;
    }
    if half_exponent >= 0x1f {
        return sign | 0x7c00;
    }
    sign | ((half_exponent as u16) << 10) | (((bits >> 13) & 0x03ff) as u16)
}

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

fn rgba16_data(values: impl Iterator<Item = [f32; 4]>) -> Vec<u8> {
    values
        .flat_map(|value| {
            value
                .into_iter()
                .flat_map(|channel| half_bits(channel).to_le_bytes())
        })
        .collect()
}

fn rgba16_texture(
    backend: &Backend,
    label: &str,
    values: impl Iterator<Item = [f32; 4]>,
) -> wgpu::Texture {
    backend.device.create_texture_with_data(
        &backend.queue,
        &wgpu::TextureDescriptor {
            label: Some(label),
            size: wgpu::Extent3d {
                width: SIZE,
                height: SIZE,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: HISTORY_FORMAT,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        },
        wgpu::util::TextureDataOrder::LayerMajor,
        &rgba16_data(values),
    )
}

fn depth_texture(backend: &Backend, label: &str, value: f32) -> wgpu::Texture {
    let texture = backend.device.create_texture(&wgpu::TextureDescriptor {
        label: Some(label),
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
    let module = backend
        .device
        .create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("temporal-resolve-depth-fixture"),
            source: wgpu::ShaderSource::Wgsl(
                format!(
                    r#"
@vertex fn vertex(@builtin(vertex_index) i: u32) -> @builtin(position) vec4<f32> {{
    let p = array<vec2<f32>, 3>(vec2(-1.0, -1.0), vec2(3.0, -1.0), vec2(-1.0, 3.0));
    return vec4<f32>(p[i], 0.0, 1.0);
}}
@fragment fn fragment() -> @builtin(frag_depth) f32 {{ return {:?}; }}
"#,
                    value
                )
                .into(),
            ),
        });
    let pipeline = backend
        .device
        .create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("temporal-resolve-depth-fixture"),
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
        });
    let view = texture.create_view(&Default::default());
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    {
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("temporal-resolve-depth-fixture"),
            color_attachments: &[],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: &view,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(1.0),
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            }),
            ..Default::default()
        });
        pass.set_pipeline(&pipeline);
        pass.draw(0..3, 0..1);
    }
    backend.queue.submit([encoder.finish()]);
    texture
}

fn read_texture(backend: &Backend, texture: &wgpu::Texture) -> Vec<u8> {
    let stride = (texture.width() * HISTORY_BYTES_PER_PIXEL)
        .div_ceil(wgpu::COPY_BYTES_PER_ROW_ALIGNMENT)
        * wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
    let buffer = backend.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("temporal-resolve-readback"),
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
            row[..(texture.width() * HISTORY_BYTES_PER_PIXEL) as usize]
                .iter()
                .copied()
        })
        .collect();
    drop(data);
    buffer.unmap();
    result
}

fn neighbors(x: u32, y: u32) -> [[f32; 3]; 9] {
    let mut values = [[0.0; 3]; 9];
    let mut at = 0;
    for dy in -1i32..=1 {
        for dx in -1i32..=1 {
            let px = (x as i32 + dx).clamp(0, SIZE as i32 - 1) as u32;
            let py = (y as i32 + dy).clamp(0, SIZE as i32 - 1) as u32;
            let color = checker(px, py);
            values[at] = [color[0], color[1], color[2]];
            at += 1;
        }
    }
    values
}

fn expected_pixel(x: u32, y: u32, previous_depth: f64, motion_valid: bool) -> ([f32; 3], u32) {
    let mut candidate = HistoryCandidate {
        color: [0.625; 3],
        age: 2,
        color_uv: [0.5, 0.5],
        depth_uv: [0.5, 0.5],
        sampled_depth: previous_depth,
        expected_depth: 0.5,
        depth_tolerance: 1e-4,
    };
    if !motion_valid {
        candidate.sampled_depth = 0.5;
    }
    let output = resolve(
        ResolveInput {
            current: [checker(x, y)[0]; 3],
            neighbors: neighbors(x, y),
            motion_valid,
            epoch_valid: true,
            history: Some(candidate),
        },
        ResolvePolicy::default(),
    );
    (output.color, output.age)
}

fn run(
    backend: &Backend,
    previous_depth_value: f32,
    motion_valid: bool,
    min_current_weight: f32,
    invalid_history_pixel: Option<[u32; 2]>,
) -> Vec<([f32; 3], u32)> {
    let current = rgba16_texture(backend, "temporal-current", {
        let mut values = Vec::with_capacity((SIZE * SIZE) as usize);
        for y in 0..SIZE {
            for x in 0..SIZE {
                let mut value = checker(x, y);
                value[3] = f32::from(motion_valid);
                values.push(value);
            }
        }
        values.into_iter()
    });
    let history = rgba16_texture(
        backend,
        "temporal-history",
        (0..SIZE).flat_map(|y| {
            (0..SIZE).map(move |x| {
                flat(
                    0.625,
                    if invalid_history_pixel == Some([x, y]) {
                        0.0
                    } else {
                        2.0
                    },
                )
            })
        }),
    );
    let motion = rgba16_texture(
        backend,
        "temporal-motion",
        std::iter::repeat_n(
            [0.0, 0.0, 0.0, if motion_valid { 1.0 } else { 0.0 }],
            (SIZE * SIZE) as usize,
        ),
    );
    let current_depth = depth_texture(backend, "temporal-current-depth", 0.5);
    let previous_depth = depth_texture(backend, "temporal-previous-depth", previous_depth_value);
    let output = backend.device.create_texture(&wgpu::TextureDescriptor {
        label: Some("temporal-resolve-output"),
        size: wgpu::Extent3d {
            width: SIZE,
            height: SIZE,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: HISTORY_FORMAT,
        usage: wgpu::TextureUsages::STORAGE_BINDING | wgpu::TextureUsages::COPY_SRC,
        view_formats: &[],
    });
    let current_view = current.create_view(&Default::default());
    let history_view = history.create_view(&Default::default());
    let motion_view = motion.create_view(&Default::default());
    let current_depth_view = current_depth.create_view(&Default::default());
    let previous_depth_view = previous_depth.create_view(&Default::default());
    let output_view = output.create_view(&Default::default());
    let pass = TemporalResolvePass::new(backend);
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    pass.encode(
        backend,
        &mut encoder,
        &current_view,
        &current_depth_view,
        &motion_view,
        &history_view,
        &previous_depth_view,
        &output_view,
        ResolveParams {
            min_current_weight,
            ..params()
        },
    )
    .unwrap();
    backend.queue.submit([encoder.finish()]);
    let bytes = read_texture(backend, &output);
    bytes
        .as_chunks::<{ HISTORY_BYTES_PER_PIXEL as usize }>()
        .0
        .iter()
        .map(|pixel| {
            let channel = |index: usize| {
                half_to_f32(u16::from_le_bytes(
                    pixel[index * 2..index * 2 + 2].try_into().unwrap(),
                ))
            };
            ([channel(0), channel(1), channel(2)], channel(3) as u32)
        })
        .collect()
}

#[test]
fn valid_history_matches_the_cpu_contract_and_clips_before_blending() {
    let Some(backend) = backend() else {
        return;
    };
    let actual = run(&backend, 0.5, true, DEFAULT_MIN_CURRENT_WEIGHT, None);
    for y in 0..SIZE {
        for x in 0..SIZE {
            let (expected, age) = expected_pixel(x, y, 0.5, true);
            let (actual, actual_age) = actual[(y * SIZE + x) as usize];
            for channel in 0..3 {
                assert!(
                    (actual[channel] - expected[channel]).abs() < 2e-3,
                    "pixel ({x},{y}) channel {channel}: gpu={actual:?} cpu={expected:?}"
                );
            }
            assert_eq!(actual_age, age);
        }
    }
}

#[test]
fn depth_disocclusion_and_invalid_motion_fall_back_to_current() {
    let Some(backend) = backend() else {
        return;
    };
    for (previous_depth, motion_valid) in [(0.6, true), (0.5, false)] {
        let actual = run(
            &backend,
            previous_depth,
            motion_valid,
            DEFAULT_MIN_CURRENT_WEIGHT,
            None,
        );
        for y in 0..SIZE {
            for x in 0..SIZE {
                let (expected, age) = expected_pixel(x, y, f64::from(previous_depth), motion_valid);
                let (actual, actual_age) = actual[(y * SIZE + x) as usize];
                for channel in 0..3 {
                    assert!(
                        (actual[channel] - expected[channel]).abs() < 2e-3,
                        "previous_depth={previous_depth} motion={motion_valid} pixel ({x},{y}): gpu={actual:?} cpu={expected:?}"
                    );
                }
                assert_eq!(actual_age, age);
            }
        }
    }
}

#[test]
fn fully_responsive_policy_matches_cpu_current_and_reset_age() {
    let Some(backend) = backend() else {
        return;
    };
    let actual = run(&backend, 0.5, true, 1.0, None);
    for y in 0..SIZE {
        for x in 0..SIZE {
            let expected = checker(x, y)[0];
            let (color, age) = actual[(y * SIZE + x) as usize];
            assert_eq!(age, 1);
            assert!(color.into_iter().all(|v| (v - expected).abs() < 2e-3));
        }
    }
}

#[test]
fn masked_history_taps_cannot_bleed_through_bilinear_sampling() {
    let Some(backend) = backend() else {
        return;
    };
    let actual = run(
        &backend,
        0.5,
        true,
        DEFAULT_MIN_CURRENT_WEIGHT,
        Some([8, 8]),
    );
    // Jitter [-0.25,+0.25] in history texel coordinates makes these four
    // footprints touch the masked source, even when the nearest tap is valid.
    for (x, y) in [(8, 7), (9, 7), (8, 8), (9, 8)] {
        let (color, age) = actual[(y * SIZE + x) as usize];
        assert_eq!(age, 1, "masked history footprint at ({x},{y})");
        assert_eq!(color, [checker(x, y)[0]; 3]);
    }
    assert_eq!(actual[(4 * SIZE + 4) as usize].1, 3);
}
