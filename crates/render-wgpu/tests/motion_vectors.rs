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
    pass.encode(&backend, &mut encoder, &depth_view, &output_view, params)
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
