//! Actual post pass versus a neutral CPU oracle, not aesthetic acceptance.
use thessa_render_wgpu::{
    Backend, display::DisplayTransform, terrain::OffscreenTarget, timing::GpuIntervals,
};

const COLORS: [[f32; 3]; 8] = [
    [0.0; 3],
    [0.003; 3],
    [0.18; 3],
    [1.0; 3],
    [4.0; 3],
    [16.0; 3],
    [0.25, 2.0, 8.0],
    [-1.0, 0.5, 65504.0],
];

fn pattern(backend: &Backend, format: wgpu::TextureFormat) -> wgpu::RenderPipeline {
    let colors = COLORS
        .map(|c| format!("vec3<f32>({:?},{:?},{:?})", c[0], c[1], c[2]))
        .join(",");
    let source = format!(
        "@vertex fn vertex(@builtin(vertex_index) i:u32)->@builtin(position) vec4<f32>{{let p=array<vec2<f32>,3>(vec2(-1.0,-1.0),vec2(3.0,-1.0),vec2(-1.0,3.0));return vec4(p[i],0.0,1.0);}} @fragment fn fragment(@builtin(position) p:vec4<f32>)->@location(0) vec4<f32>{{let colors=array<vec3<f32>,8>({colors});return vec4(colors[u32(p.x)%8u],1.0);}}"
    );
    let module = backend
        .device
        .create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("display-oracle-pattern"),
            source: wgpu::ShaderSource::Wgsl(source.into()),
        });
    backend
        .device
        .create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("display-oracle-pattern"),
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
                targets: &[Some(wgpu::ColorTargetState {
                    format,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: Default::default(),
            depth_stencil: None,
            multisample: Default::default(),
            multiview_mask: None,
            cache: None,
        })
}
fn draw_pattern(
    encoder: &mut wgpu::CommandEncoder,
    view: &wgpu::TextureView,
    pipeline: &wgpu::RenderPipeline,
) {
    let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
        color_attachments: &[Some(wgpu::RenderPassColorAttachment {
            view,
            depth_slice: None,
            resolve_target: None,
            ops: wgpu::Operations {
                load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                store: wgpu::StoreOp::Store,
            },
        })],
        ..Default::default()
    });
    pass.set_pipeline(pipeline);
    pass.draw(0..3, 0..1);
}
fn srgb(v: f32) -> u8 {
    let x = if v <= 0.0031308 {
        12.92 * v
    } else {
        1.055 * v.powf(1.0 / 2.4) - 0.055
    };
    (x.clamp(0.0, 1.0) * 255.0).round() as u8
}

#[test]
#[ignore = "hardware: native HDR transfer/quantization, highlight detail, resize and disabled identity"]
fn display_matches_cpu_oracle_and_disabled_is_exact() {
    let backend = Backend::bring_up().unwrap();
    for mode in [
        thessa_graphics::ToneMapping::Reinhard,
        thessa_graphics::ToneMapping::PbrNeutral,
    ] {
        for enabled in [false, true] {
            let mut requested = thessa_graphics::RequestedGraphics::default();
            requested.renderer.hdr = enabled;
            requested.renderer.tone_mapping = mode;
            let settings = backend.resolve_graphics(&requested);
            let mut display =
                DisplayTransform::new(&backend, &settings, OffscreenTarget::FORMAT, 257, 17)
                    .unwrap();
            assert_eq!(display.enabled(), enabled);
            let pipeline = pattern(&backend, display.scene_format());
            for (width, height) in [(257, 17), (33, 65), (257, 17)] {
                display.resize(&backend, width, height).unwrap();
                assert_eq!(
                    display.target_bytes(),
                    if enabled {
                        u64::from(width) * u64::from(height) * 8
                    } else {
                        0
                    }
                );
                assert!(display.resize(&backend, 0, height).is_err());
                assert!(
                    display
                        .resize(
                            &backend,
                            backend.device.limits().max_texture_dimension_2d + 1,
                            height
                        )
                        .is_err()
                );
                assert_eq!(
                    display.target_bytes(),
                    if enabled {
                        u64::from(width) * u64::from(height) * 8
                    } else {
                        0
                    }
                );
                let target = OffscreenTarget::new(&backend, width, height).unwrap();
                let render = |post: bool| {
                    let mut encoder = backend.device.create_command_encoder(&Default::default());
                    draw_pattern(&mut encoder, display.scene_color(&target.color), &pipeline);
                    if post {
                        display.encode(&mut encoder, &target.color);
                    }
                    backend.queue.submit([encoder.finish()]);
                    target.read_rgba8(&backend).unwrap()
                };
                let pixels = render(true);
                if !enabled {
                    assert_eq!(
                        pixels,
                        render(false),
                        "HDR disabled must be byte-identical direct rendering"
                    );
                }
                for y in 0..height {
                    for x in 0..width {
                        let rgb = COLORS[x as usize % 8];
                        let rgb = if enabled {
                            match mode {
                                thessa_graphics::ToneMapping::Reinhard => {
                                    thessa_render_core::display::reinhard(rgb)
                                }
                                thessa_graphics::ToneMapping::PbrNeutral => {
                                    thessa_render_core::display::pbr_neutral(rgb)
                                }
                            }
                        } else {
                            rgb
                        };
                        let at = ((y * width + x) * 4) as usize;
                        for c in 0..3 {
                            assert!(
                                pixels[at + c].abs_diff(srgb(rgb[c])) <= 1,
                                "8-bit sRGB error >1 code: hdr={enabled} ({x},{y}) channel={c} got={} expected={}",
                                pixels[at + c],
                                srgb(rgb[c])
                            );
                        }
                        assert_eq!(pixels[at + 3], 255);
                    }
                }
                assert_eq!(
                    pixels,
                    render(true),
                    "unchanged/paused presentation has no temporal drift"
                );
                let gray = |column: usize| pixels[column * 4];
                if enabled {
                    assert!(
                        gray(3) < gray(4) && gray(4) < gray(5),
                        "1/4/16 highlights must remain distinct"
                    );
                } else {
                    assert_eq!(gray(3), gray(5));
                }
            }
        }
    }

    // Non-sRGB SDR surfaces require exactly one explicit output transfer.
    let settings = backend.resolve_graphics(&Default::default());
    let display = DisplayTransform::new(
        &backend,
        &settings,
        wgpu::TextureFormat::Rgba8Unorm,
        257,
        17,
    )
    .unwrap();
    let target =
        OffscreenTarget::with_format(&backend, 257, 17, wgpu::TextureFormat::Rgba8Unorm).unwrap();
    let pipeline = pattern(&backend, display.scene_format());
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    draw_pattern(&mut encoder, display.scene_color(&target.color), &pipeline);
    display.encode(&mut encoder, &target.color);
    backend.queue.submit([encoder.finish()]);
    let pixels = target.read_rgba8(&backend).unwrap();
    for x in 0..257 {
        let rgb = thessa_render_core::display::reinhard(COLORS[x % 8]);
        for c in 0..3 {
            assert!(pixels[x * 4 + c].abs_diff(srgb(rgb[c])) <= 1);
        }
    }
}

#[test]
#[ignore = "hardware: display pass standalone cost; release; no game-FPS comparison"]
fn display_pass_cost() {
    let backend = Backend::bring_up_for_graphics_profiled(&Default::default()).unwrap();
    let settings = backend.resolve_graphics(&Default::default());
    for [width, height] in [[1280, 720], [1920, 1080], [2560, 1440]] {
        let display =
            DisplayTransform::new(&backend, &settings, OffscreenTarget::FORMAT, width, height)
                .unwrap();
        let output = OffscreenTarget::new(&backend, width, height).unwrap();
        let pipeline = pattern(&backend, display.scene_format());
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        draw_pattern(&mut encoder, display.scene_color(&output.color), &pipeline);
        backend.queue.submit([encoder.finish()]);
        let timing = GpuIntervals::new(&backend, 2).expect("hardware timestamp diagnostic");
        let mut gpu = Vec::new();
        let mut wall = Vec::new();
        for i in 0..230 {
            let start = std::time::Instant::now();
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            timing.mark(&mut encoder, 0);
            display.encode(&mut encoder, &output.color);
            timing.mark(&mut encoder, 1);
            timing.resolve(&mut encoder);
            backend.queue.submit([encoder.finish()]);
            backend
                .device
                .poll(wgpu::PollType::wait_indefinitely())
                .unwrap();
            let elapsed = start.elapsed().as_secs_f64() * 1000.0;
            let value = timing.read(&backend).unwrap()[0] * 1000.0;
            if i >= 30 {
                gpu.push(value);
                wall.push(elapsed);
            }
        }
        gpu.sort_by(f64::total_cmp);
        wall.sort_by(f64::total_cmp);
        println!(
            "display_pass adapter={:?} extent={width}x{height} target_bytes={} samples=200 warmup=30 gpu_p50_ms={} gpu_p95_ms={} gpu_p99_ms={} host_wait_p50_ms={}; HDR sample+Reinhard+sRGB only, source pattern prefilled, uploads/scene/UI/readback/setup excluded; not game FPS",
            backend.adapter_name,
            display.target_bytes(),
            gpu[100],
            gpu[190],
            gpu[198],
            wall[100]
        );
    }
}
