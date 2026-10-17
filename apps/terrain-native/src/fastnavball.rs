//! Native-pixel attitude sphere. Only static graticules may be baked; attitude,
//! markers, labels and roll are live at the presentation cadence, even paused.
//! Resources belong to the existing egui renderer/device. No scene copy, extra
//! target, texture upload per attitude change or 30 Hz CPU raster rebuild.
use thessa_flight_authority::navball::InstrumentFrame;
use thessa_graphics::{InstrumentLayout, NavballRenderer, ResolvedInstruments};
use wgpu::util::DeviceExt;

#[derive(Clone, Copy)]
pub struct Paint {
    pub frame: InstrumentFrame,
    pub settings: ResolvedInstruments,
    pub format: wgpu::TextureFormat,
}

pub struct FastNavball {
    pipeline: wgpu::RenderPipeline,
    bind: wgpu::BindGroup,
    uniform: wgpu::Buffer,
    key: (wgpu::TextureFormat, NavballRenderer, u32),
    last: Option<[u8; 48]>,
}

// Identical angular grid spacing in both GPU variants; the baked path stores
// filtered coverage rather than a low-resolution attitude-dependent image.
const ANALYTIC_GRID: &str = r#"
fn grid_coverage(uv:vec2<f32>,gx:vec2<f32>,gy:vec2<f32>)->f32{
    let width=max((abs(gx)+abs(gy))*vec2(6.28318530718,3.14159265359),vec2(0.0015));
    let gap=abs(fract((uv-vec2(0.5))*vec2(12.0,18.0)+0.5)-0.5)*vec2(6.28318530718/12.0,3.14159265359/18.0);
    let lines=vec2(1.0)-smoothstep(width*0.4,width*0.9,gap);
    return max(lines.x,lines.y);
}"#;
const BAKED_GRID: &str = r#"
@group(0) @binding(1) var graticule:texture_2d<f32>;
@group(0) @binding(2) var grid_sampler:sampler;
fn grid_coverage(uv:vec2<f32>,gx:vec2<f32>,gy:vec2<f32>)->f32{return textureSampleGrad(graticule,grid_sampler,uv,gx,gy).r;}
"#;

fn atlas(width: u32) -> Vec<u8> {
    let mut data = Vec::new();
    let height = width / 2;
    for y in 0..height {
        for x in 0..width {
            let mut value = 0.0_f64;
            for (dx, dy) in [(0.25, 0.25), (0.75, 0.25), (0.25, 0.75), (0.75, 0.75)] {
                let u = (x as f64 + dx) / width as f64 - 0.5;
                let v = (y as f64 + dy) / height as f64 - 0.5;
                let longitude =
                    ((u * 12.0 + 0.5).rem_euclid(1.0) - 0.5).abs() * std::f64::consts::TAU / 12.0;
                let latitude =
                    ((v * 18.0 + 0.5).rem_euclid(1.0) - 0.5).abs() * std::f64::consts::PI / 18.0;
                value += (1.0 - longitude.min(latitude) / 0.006).clamp(0.0, 1.0);
            }
            data.push((value * 255.0 / 4.0).round() as u8);
        }
    }
    let mut size = [width, height];
    let mut previous = 0;
    while size[0] > 1 || size[1] > 1 {
        let next = [(size[0] / 2).max(1), (size[1] / 2).max(1)];
        let offset = data.len();
        for y in 0..next[1] {
            for x in 0..next[0] {
                let mut sum = 0u32;
                for dy in 0..2 {
                    for dx in 0..2 {
                        let sx = (x * 2 + dx).min(size[0] - 1);
                        let sy = (y * 2 + dy).min(size[1] - 1);
                        sum += u32::from(data[previous + (sy * size[0] + sx) as usize]);
                    }
                }
                data.push(((sum + 2) / 4) as u8);
            }
        }
        previous = offset;
        size = next;
    }
    data
}

impl FastNavball {
    fn new(device: &wgpu::Device, queue: &wgpu::Queue, paint: Paint) -> Self {
        let baked = paint.settings.renderer == NavballRenderer::Baked;
        let code = format!(
            "const TARGET_SRGB:bool={};\n{}",
            paint.format.is_srgb(),
            include_str!("fastnavball.wgsl").replace(
                "// GRID_IMPLEMENTATION",
                if baked { BAKED_GRID } else { ANALYTIC_GRID }
            )
        );
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("fastnavball"),
            source: wgpu::ShaderSource::Wgsl(code.into()),
        });
        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("fastnavball-native-pixel"),
            layout: None,
            vertex: wgpu::VertexState {
                module: &module,
                entry_point: Some("vertex"),
                compilation_options: Default::default(),
                buffers: &[],
            },
            primitive: Default::default(),
            depth_stencil: None,
            multisample: Default::default(),
            fragment: Some(wgpu::FragmentState {
                module: &module,
                entry_point: Some("fragment"),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format: paint.format,
                    blend: Some(wgpu::BlendState::PREMULTIPLIED_ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            multiview_mask: None,
            cache: None,
        });
        let uniform = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("navball-attitude-48-bytes"),
            contents: &[0; 48],
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });
        let texture = baked.then(|| {
            let width = paint.settings.atlas_width;
            device
                .create_texture_with_data(
                    queue,
                    &wgpu::TextureDescriptor {
                        label: Some("navball-static-graticule"),
                        size: wgpu::Extent3d {
                            width,
                            height: width / 2,
                            depth_or_array_layers: 1,
                        },
                        mip_level_count: width.ilog2() + 1,
                        sample_count: 1,
                        dimension: wgpu::TextureDimension::D2,
                        format: wgpu::TextureFormat::R8Unorm,
                        usage: wgpu::TextureUsages::TEXTURE_BINDING,
                        view_formats: &[],
                    },
                    wgpu::util::TextureDataOrder::LayerMajor,
                    &atlas(width),
                )
                .create_view(&Default::default())
        });
        let sampler = baked.then(|| {
            device.create_sampler(&wgpu::SamplerDescriptor {
                label: Some("navball-grid-mips"),
                address_mode_u: wgpu::AddressMode::Repeat,
                address_mode_v: wgpu::AddressMode::ClampToEdge,
                mag_filter: wgpu::FilterMode::Linear,
                min_filter: wgpu::FilterMode::Linear,
                mipmap_filter: wgpu::MipmapFilterMode::Linear,
                ..Default::default()
            })
        });
        let mut entries = vec![wgpu::BindGroupEntry {
            binding: 0,
            resource: uniform.as_entire_binding(),
        }];
        if let (Some(texture), Some(sampler)) = (&texture, &sampler) {
            entries.extend([
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::TextureView(texture),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: wgpu::BindingResource::Sampler(sampler),
                },
            ]);
        }
        let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("navball-field"),
            layout: &pipeline.get_bind_group_layout(0),
            entries: &entries,
        });
        Self {
            pipeline,
            bind,
            uniform,
            key: (
                paint.format,
                paint.settings.renderer,
                paint.settings.atlas_width,
            ),
            last: None,
        }
    }
    fn update(&mut self, queue: &wgpu::Queue, paint: Paint) {
        let mut words = [0.0_f32; 12];
        words[0..3].copy_from_slice(&paint.frame.north.as_vec3().to_array());
        words[4..7].copy_from_slice(&paint.frame.east.as_vec3().to_array());
        words[8..11].copy_from_slice(&paint.frame.up.as_vec3().to_array());
        words[3] = f32::from(paint.settings.layout == InstrumentLayout::Rocket);
        let data = words.map(f32::to_le_bytes);
        let bytes = *data.as_flattened().first_chunk::<48>().unwrap();
        if self.last != Some(bytes) {
            queue.write_buffer(&self.uniform, 0, &bytes);
            self.last = Some(bytes);
        }
    }
    fn draw(&self, pass: &mut wgpu::RenderPass<'_>) {
        pass.set_pipeline(&self.pipeline);
        pass.set_bind_group(0, &self.bind, &[]);
        pass.draw(0..6, 0..1);
    }
}

impl egui_wgpu::CallbackTrait for Paint {
    fn prepare(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        _screen: &egui_wgpu::ScreenDescriptor,
        _encoder: &mut wgpu::CommandEncoder,
        resources: &mut egui_wgpu::CallbackResources,
    ) -> Vec<wgpu::CommandBuffer> {
        let key = (
            self.format,
            self.settings.renderer,
            self.settings.atlas_width,
        );
        if resources.get::<FastNavball>().is_none_or(|r| r.key != key) {
            resources.insert(FastNavball::new(device, queue, *self));
        }
        resources
            .get_mut::<FastNavball>()
            .unwrap()
            .update(queue, *self);
        Vec::new()
    }
    fn paint(
        &self,
        _info: egui::PaintCallbackInfo,
        pass: &mut wgpu::RenderPass<'static>,
        resources: &egui_wgpu::CallbackResources,
    ) {
        resources.get::<FastNavball>().unwrap().draw(pass);
    }
}

impl Paint {
    pub fn show(self, ui: &mut egui::Ui, side: f32) {
        let (rect, _) = ui.allocate_exact_size(egui::vec2(side, side), egui::Sense::hover());
        let center = rect.center();
        let radius = side * 0.44;
        let disk = egui::Rect::from_center_size(center, egui::Vec2::splat(radius * 2.0));
        let painter = ui.painter();
        painter.circle_filled(center, side * 0.499, egui::Color32::from_rgb(14, 20, 24));
        painter.add(egui_wgpu::Callback::new_paint_callback(disk, self));
        painter.circle_stroke(center, radius, egui::Stroke::new(1.5, egui::Color32::GRAY));
        for degree in (0..360).step_by(10) {
            let a = (degree as f32).to_radians();
            let direction = egui::vec2(a.sin(), -a.cos());
            painter.line_segment(
                [
                    center + direction * (radius + 3.0),
                    center + direction * (radius + if degree % 30 == 0 { 10.0 } else { 6.0 }),
                ],
                egui::Stroke::new(1.0, egui::Color32::LIGHT_GRAY),
            );
        }
        let gold = egui::Color32::from_rgb(255, 191, 66);
        let roll = (self.frame.up.y as f32).atan2(self.frame.up.z as f32);
        let tip = egui::vec2(roll.sin(), -roll.cos());
        painter.circle_filled(center + tip * (radius + 12.0), 3.0, gold);
        for degrees in (0..360).step_by(30) {
            let a = (degrees as f64).to_radians();
            let dir = self.frame.north * a.cos() + self.frame.east * a.sin();
            if dir.x < 0.28 {
                continue;
            }
            if let Some(p) = InstrumentFrame::project(dir) {
                let at = center + egui::vec2(p[0], p[1]) * radius;
                let text = match degrees {
                    0 => "N".to_owned(),
                    90 => "E".to_owned(),
                    180 => "S".to_owned(),
                    270 => "W".to_owned(),
                    _ => format!("{degrees:03}"),
                };
                painter.text(
                    at + egui::vec2(1.0, 1.0),
                    egui::Align2::CENTER_CENTER,
                    &text,
                    egui::FontId::monospace(side * 0.041),
                    egui::Color32::BLACK,
                );
                painter.text(
                    at,
                    egui::Align2::CENTER_CENTER,
                    text,
                    egui::FontId::monospace(side * 0.041),
                    egui::Color32::WHITE,
                );
            }
        }
        let meridian = (glam::DVec3::X - self.frame.up * self.frame.up.x)
            .try_normalize()
            .unwrap_or(self.frame.north);
        for degrees in [-60.0_f64, -30.0, 30.0, 60.0] {
            let a = degrees.to_radians();
            let direction = meridian * a.cos() + self.frame.up * a.sin();
            if direction.x < 0.22 {
                continue;
            }
            if let Some(p) = InstrumentFrame::project(direction) {
                let at = center + egui::vec2(p[0] + 0.12, p[1]) * radius;
                let text = format!("{degrees:+.0}");
                painter.text(
                    at + egui::vec2(1.0, 1.0),
                    egui::Align2::LEFT_CENTER,
                    &text,
                    egui::FontId::monospace(side * 0.039),
                    egui::Color32::BLACK,
                );
                painter.text(
                    at,
                    egui::Align2::LEFT_CENTER,
                    text,
                    egui::FontId::monospace(side * 0.039),
                    egui::Color32::WHITE,
                );
            }
        }
        for (direction, color, retro) in [
            (self.frame.prograde, egui::Color32::LIGHT_GREEN, false),
            (
                self.frame.prograde.map(|v| -v),
                egui::Color32::LIGHT_GREEN,
                true,
            ),
            (Some(self.frame.up), egui::Color32::LIGHT_BLUE, false),
        ] {
            if let Some(point) = direction.and_then(InstrumentFrame::project) {
                let at = center + egui::vec2(point[0], point[1]) * radius;
                painter.circle_stroke(at, side * 0.022, egui::Stroke::new(2.0, color));
                for angle in [0.0_f32, 2.094395, 4.18879] {
                    let d = egui::vec2(angle.sin(), angle.cos());
                    painter.line_segment(
                        [at + d * side * 0.023, at + d * side * 0.04],
                        egui::Stroke::new(1.5, color),
                    );
                }
                if retro {
                    painter.line_segment(
                        [at - egui::vec2(4.0, 4.0), at + egui::vec2(4.0, 4.0)],
                        egui::Stroke::new(1.5, color),
                    );
                }
            }
        }
        for sign in [-1.0, 1.0] {
            painter.line_segment(
                [
                    center + egui::vec2(sign * side * 0.035, 0.0),
                    center + egui::vec2(sign * side * 0.13, 0.0),
                ],
                egui::Stroke::new(2.5, gold),
            );
        }
        painter.circle_stroke(center, side * 0.015, egui::Stroke::new(2.0, gold));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::DVec3;
    use thessa_render_wgpu::{Backend, terrain::OffscreenTarget};

    fn paint(renderer: NavballRenderer, layout: InstrumentLayout) -> Paint {
        Paint {
            frame: InstrumentFrame::new(DVec3::Z, DVec3::X, DVec3::X),
            settings: thessa_graphics::resolve_instruments(thessa_graphics::InstrumentSettings {
                navball_renderer: renderer,
                layout,
                ..Default::default()
            }),
            format: OffscreenTarget::FORMAT,
        }
    }
    fn draw(backend: &Backend, gpu: &mut FastNavball, paint: Paint, size: u32) -> Vec<u8> {
        let target = OffscreenTarget::new(backend, size, size).unwrap();
        gpu.update(&backend.queue, paint);
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &target.color,
                    resolve_target: None,
                    depth_slice: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.1,
                            g: 0.2,
                            b: 0.3,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                ..Default::default()
            });
            gpu.draw(&mut pass);
        }
        backend.queue.submit([encoder.finish()]);
        target.read_rgba8(backend).unwrap()
    }

    #[test]
    fn baked_graticule_has_complete_mips_with_periodic_meridian_coverage() {
        for width in [512, 1024, 2048] {
            let data = atlas(width);
            let bytes: u32 = (0..=width.ilog2())
                .map(|l| (width >> l).max(1) * ((width / 2) >> l).max(1))
                .sum();
            assert_eq!(data.len(), bytes as usize);
            for y in 0..width / 2 {
                let start = (y * width) as usize;
                assert!(data[start].abs_diff(data[start + width as usize - 1]) <= 1);
            }
            assert!(data.iter().any(|&v| v > 150));
            assert!(data.contains(&0));
        }
    }

    #[test]
    #[ignore = "requires a wgpu adapter; native-resolution navball numeric/render oracle"]
    fn native_pixel_ball_is_live_in_heading_pitch_roll_and_both_palettes() {
        let backend = Backend::bring_up().unwrap();
        for renderer in [NavballRenderer::Analytic, NavballRenderer::Baked] {
            for layout in [InstrumentLayout::Aircraft, InstrumentLayout::Rocket] {
                let p = paint(renderer, layout);
                let mut gpu = FastNavball::new(&backend.device, &backend.queue, p);
                let reference = draw(&backend, &mut gpu, p, 256);
                assert_eq!(
                    reference,
                    draw(&backend, &mut gpu, p, 256),
                    "paused input is deterministic"
                );
                for (axis, angle) in [
                    (DVec3::X, 0.7),
                    (DVec3::Y, 0.7),
                    (DVec3::Z, 0.7),
                    (DVec3::Y, std::f64::consts::FRAC_PI_2),
                ] {
                    let q = glam::DQuat::from_axis_angle(axis, angle);
                    let moved = Paint {
                        frame: InstrumentFrame::new(q * DVec3::Z, q * DVec3::X, DVec3::X),
                        ..p
                    };
                    let image = draw(&backend, &mut gpu, moved, 256);
                    assert_ne!(
                        reference, image,
                        "instrument must respond to all attitude axes, including heading"
                    );
                    assert_eq!(
                        &reference[..4],
                        &image[..4],
                        "disk cannot clear transparent corners"
                    );
                }
                assert_eq!(
                    reference,
                    draw(&backend, &mut gpu, p, 256),
                    "rewind must restore bytes"
                );
                let vertical = Paint {
                    frame: InstrumentFrame::new(DVec3::X, DVec3::Z, DVec3::ZERO),
                    ..p
                };
                let odd = draw(&backend, &mut gpu, vertical, 257);
                assert!(
                    odd[((128 * 257 + 128) * 4) as usize..][..3]
                        .iter()
                        .any(|&v| v > 10),
                    "exact pole-centre pixel must remain a finite visible colour"
                );
                let seam = |offset: f64| {
                    let q = glam::DQuat::from_rotation_z(std::f64::consts::PI + offset);
                    Paint {
                        frame: InstrumentFrame::new(q * DVec3::Z, q * DVec3::X, DVec3::X),
                        ..p
                    }
                };
                let left = draw(&backend, &mut gpu, seam(-1e-6), 256);
                let right = draw(&backend, &mut gpu, seam(1e-6), 256);
                assert!(
                    left.iter().zip(&right).all(|(a, b)| a.abs_diff(*b) <= 4),
                    "atan2 branch cut must not create a mip/AA seam"
                );
                let large = draw(&backend, &mut gpu, p, 512);
                assert_eq!(large.len(), 512 * 512 * 4);
                // Hemisphere colours remain readable away from the grid.
                let upper = &large[((125 * 512 + 271) * 4)..][..3];
                let lower = &large[((391 * 512 + 271) * 4)..][..3];
                if layout == InstrumentLayout::Aircraft {
                    assert!(upper[2] > upper[0] && lower[0] > lower[2]);
                } else {
                    assert!(
                        upper.iter().map(|&v| u32::from(v)).sum::<u32>()
                            > lower.iter().map(|&v| u32::from(v)).sum::<u32>()
                    );
                }
            }
        }
    }

    #[test]
    #[ignore = "requires a wgpu adapter; release CPU128 versus native GPU analytic/baked cost diagnostic"]
    fn navball_cost_diagnostic() {
        use thessa_render_wgpu::timing::GpuIntervals;
        let backend = Backend::bring_up_for_graphics_profiled(&Default::default()).unwrap();
        let mut cpu = Vec::new();
        for i in 0..140 {
            let q = glam::DQuat::from_rotation_y(i as f64 * 0.001);
            let start = std::time::Instant::now();
            std::hint::black_box(thessa_flight_authority::navball::navball_pixels(
                q * DVec3::Z,
            ));
            if i >= 20 {
                cpu.push(start.elapsed().as_secs_f64() * 1000.0);
            }
        }
        cpu.sort_by(f64::total_cmp);
        eprintln!(
            "navball_cpu128 samples=120 pixel_generation_p50_ms={} p95_ms={} upload_bytes_per_rebuild=65536 (upload/draw excluded)",
            cpu[60], cpu[114]
        );
        for renderer in [NavballRenderer::Analytic, NavballRenderer::Baked] {
            for side in [256, 512] {
                let mut p = paint(renderer, InstrumentLayout::Rocket);
                let init = std::time::Instant::now();
                let mut gpu = FastNavball::new(&backend.device, &backend.queue, p);
                let init_ms = init.elapsed().as_secs_f64() * 1000.0;
                let target = OffscreenTarget::new(&backend, side, side).unwrap();
                let timing = GpuIntervals::new(&backend, 2);
                let mut times = Vec::new();
                let mut host = Vec::new();
                for i in 0..140 {
                    let q = glam::DQuat::from_rotation_y(i as f64 * 0.001);
                    p.frame = InstrumentFrame::new(q * DVec3::Z, q * DVec3::X, DVec3::X);
                    let start = std::time::Instant::now();
                    gpu.update(&backend.queue, p);
                    let mut encoder = backend.device.create_command_encoder(&Default::default());
                    if let Some(t) = &timing {
                        t.mark(&mut encoder, 0);
                    }
                    {
                        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                                view: &target.color,
                                resolve_target: None,
                                depth_slice: None,
                                ops: wgpu::Operations {
                                    load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                                    store: wgpu::StoreOp::Store,
                                },
                            })],
                            ..Default::default()
                        });
                        gpu.draw(&mut pass);
                    }
                    if let Some(t) = &timing {
                        t.mark(&mut encoder, 1);
                        t.resolve(&mut encoder);
                    }
                    backend.queue.submit([encoder.finish()]);
                    backend
                        .device
                        .poll(wgpu::PollType::wait_indefinitely())
                        .unwrap();
                    let wall = start.elapsed().as_secs_f64() * 1000.0;
                    let value = timing
                        .as_ref()
                        .map(|t| t.read(&backend).unwrap()[0] * 1000.0);
                    if i >= 20 {
                        host.push(wall);
                        if let Some(v) = value {
                            times.push(v);
                        }
                    }
                }
                host.sort_by(f64::total_cmp);
                times.sort_by(f64::total_cmp);
                eprintln!(
                    "navball_gpu adapter={:?} mode={renderer:?} side={side} samples=120 gpu_p50_ms={:?} gpu_p95_ms={:?} host_p50_ms={} init_host_ms={} attitude_upload_bytes=48 static_atlas_bytes={} (clear+quad only, labels/markers/egui excluded; not matched quality with CPU128)",
                    backend.adapter_name,
                    times.get(60),
                    times.get(114),
                    host[60],
                    init_ms,
                    if renderer == NavballRenderer::Baked {
                        atlas(p.settings.atlas_width).len()
                    } else {
                        0
                    }
                );
            }
        }
    }
}
