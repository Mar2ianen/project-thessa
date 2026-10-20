//! Native plume raster consumer. The published axial stations, not a fitted
//! extinction/hue approximation, supply the medium. A color-load pass reads
//! the existing scene depth directly; no depth copy or global mode change.
use crate::{
    Backend, BackendError,
    view::{ViewParams, ViewUniformBlock},
};
use glam::{Vec3, Vec4};
use std::sync::Arc;
use wgpu::util::DeviceExt;

const MAX_STATIONS: usize = 64;
const FRAME_BYTES: u64 = 160;
const DRAW_BYTES: u64 = 80;
const COMMON: &str = include_str!("plume_common.wgsl");

/// GPU transport: axial distance/radius/extinction/shock, then emission RGB
/// and padding. This is a backend payload, not a simulation API.
pub type ProfileStation = [f32; 8];

#[derive(Debug, Clone, PartialEq)]
pub struct VolumeDraw {
    /// Camera-relative origin and physical field length (metres).
    pub origin_len: [f32; 4],
    /// Unit exhaust axis and conservative radial integration bound (metres).
    pub axis_radius: [f32; 4],
    /// March steps, simulation time, shell erosion strength, advection speed.
    pub animation: [f32; 4],
    pub diamonds: bool,
    pub tail_dissolve: bool,
    /// Immutable publication; cloned by handle, uploaded only on change.
    pub profile: Arc<[ProfileStation]>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ConeDraw {
    pub model: [f32; 16],
    /// Caller-owned 32x256 sRGB RGBA8 impostor; game optical bakes stay in
    /// the GPL application, not the MIT renderer.
    pub texture: Arc<[u8]>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum PlumeFrame {
    Volume(VolumeDraw),
    Cone(ConeDraw),
}

/// Work submitted by the last update/encode, never fabricated GPU time.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct PlumeStats {
    pub draw_calls: u32,
    pub march_steps: u32,
    pub frame_upload_bytes: u64,
    pub profile_upload_bytes: u64,
    pub texture_upload_bytes: u64,
}

pub struct PlumeRenderer {
    device: Arc<wgpu::Device>,
    format: wgpu::TextureFormat,
    diamonds: bool,
    volume_pipeline: wgpu::RenderPipeline,
    cone_pipeline: wgpu::RenderPipeline,
    global_layout: wgpu::BindGroupLayout,
    frame: wgpu::Buffer,
    draw: wgpu::Buffer,
    profile: wgpu::Buffer,
    volume_bind: wgpu::BindGroup,
    cone_bind: wgpu::BindGroup,
    cone_vertices: wgpu::Buffer,
    cone_count: u32,
    published: Option<Arc<[ProfileStation]>>,
    impostor: wgpu::Texture,
    published_impostor: Option<Arc<[u8]>>,
    active: Option<PlumeFrame>,
    stats: PlumeStats,
}

fn uniform(binding: u32, size: u64) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Uniform,
            has_dynamic_offset: false,
            min_binding_size: wgpu::BufferSize::new(size),
        },
        count: None,
    }
}

pub fn volume_shader_source() -> String {
    format!("{COMMON}\n{}", include_str!("plume_volume.wgsl"))
}

pub fn cone_shader_source() -> String {
    format!("{COMMON}\n{}", include_str!("plume_cone.wgsl"))
}

impl PlumeRenderer {
    /// Lazy creation: never call for a disabled effect or an absent source.
    pub fn new(
        backend: &Backend,
        format: wgpu::TextureFormat,
        diamonds: bool,
    ) -> Result<Self, BackendError> {
        let device = &backend.device;
        let global_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("plume-camera-depth"),
            entries: &[
                uniform(0, FRAME_BYTES),
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Depth,
                        view_dimension: wgpu::TextureViewDimension::D2,
                        multisampled: false,
                    },
                    count: None,
                },
            ],
        });
        let volume_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("plume-profile"),
            entries: &[
                uniform(0, DRAW_BYTES),
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
            ],
        });
        let cone_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("plume-impostor"),
            entries: &[
                uniform(0, 64),
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
        let make_buffer = |label, size, usage| {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size,
                usage: usage | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            })
        };
        let frame = make_buffer("plume-camera", FRAME_BYTES, wgpu::BufferUsages::UNIFORM);
        let draw = make_buffer("plume-draw", DRAW_BYTES, wgpu::BufferUsages::UNIFORM);
        let profile = make_buffer(
            "plume-profile",
            (MAX_STATIONS * 32) as u64,
            wgpu::BufferUsages::STORAGE,
        );
        let volume_bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("plume-profile"),
            layout: &volume_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: draw.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: profile.as_entire_binding(),
                },
            ],
        });
        let texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("plume-impostor"),
            size: wgpu::Extent3d {
                width: 32,
                height: 256,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8UnormSrgb,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        });
        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            ..Default::default()
        });
        let cone_bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("plume-impostor"),
            layout: &cone_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: draw.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::TextureView(
                        &texture.create_view(&Default::default()),
                    ),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: wgpu::BindingResource::Sampler(&sampler),
                },
            ],
        });
        let volume_pipeline = pipeline(
            device,
            format,
            &volume_shader_source(),
            &global_layout,
            &volume_layout,
            false,
        );
        let cone_pipeline = pipeline(
            device,
            format,
            &cone_shader_source(),
            &global_layout,
            &cone_layout,
            true,
        );
        let cone = impostor_cone_vertices();
        let cone_count = cone.len() as u32 / 5;
        let cone_vertices = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("plume-cone"),
            contents: &bytes(cone),
            usage: wgpu::BufferUsages::VERTEX,
        });
        Ok(Self {
            device: backend.device.clone(),
            format,
            diamonds,
            volume_pipeline,
            cone_pipeline,
            global_layout,
            frame,
            draw,
            profile,
            volume_bind,
            cone_bind,
            cone_vertices,
            cone_count,
            published: None,
            impostor: texture,
            published_impostor: None,
            active: None,
            stats: PlumeStats::default(),
        })
    }

    pub fn is_live_for(&self, format: wgpu::TextureFormat, diamonds: bool) -> bool {
        self.format == format && self.diamonds == diamonds
    }

    pub fn stats(&self) -> PlumeStats {
        self.stats
    }
    pub fn active(&self) -> bool {
        self.active.is_some()
    }

    pub fn update(
        &mut self,
        backend: &Backend,
        params: &ViewParams,
        frame: Option<PlumeFrame>,
    ) -> Result<(), BackendError> {
        self.stats = PlumeStats::default();
        self.active = None;
        let Some(frame) = frame else {
            return Ok(());
        };
        let view = ViewUniformBlock::new(params)
            .ok_or_else(|| BackendError::Device("invalid plume camera".into()))?;
        let frame_bytes = bytes(
            view.clip_from_world
                .to_cols_array()
                .into_iter()
                .chain(view.world_from_clip.to_cols_array())
                .chain(view.world_position.extend(0.0).to_array())
                .chain([0.0; 4]),
        );
        let draw_bytes = match &frame {
            PlumeFrame::Volume(draw) => {
                if !(2..=MAX_STATIONS).contains(&draw.profile.len())
                    || !draw.profile.iter().flatten().all(|v| v.is_finite())
                {
                    return Err(BackendError::Device(
                        "invalid plume axial publication".into(),
                    ));
                }
                if self
                    .published
                    .as_ref()
                    .is_none_or(|p| !Arc::ptr_eq(p, &draw.profile))
                {
                    let data = bytes(draw.profile.iter().flatten().copied());
                    backend.queue.write_buffer(&self.profile, 0, &data);
                    self.stats.profile_upload_bytes = data.len() as u64;
                    self.published = Some(draw.profile.clone());
                }
                let Some(rect) = clip_rect(&view, draw) else {
                    return Ok(());
                };
                self.stats.march_steps = draw.animation[0].clamp(1.0, 16.0) as u32;
                bytes(
                    rect.into_iter()
                        .chain(draw.origin_len)
                        .chain(draw.axis_radius)
                        .chain([
                            draw.profile.len() as f32,
                            draw.animation[0],
                            draw.animation[1],
                            draw.animation[2],
                        ])
                        .chain([
                            draw.animation[3],
                            f32::from(draw.diamonds),
                            f32::from(draw.tail_dissolve),
                            0.0,
                        ]),
                )
            }
            PlumeFrame::Cone(draw) => {
                if draw.texture.len() != 32 * 256 * 4 {
                    return Err(BackendError::Device(
                        "plume impostor must be 32x256 RGBA8".into(),
                    ));
                }
                if self
                    .published_impostor
                    .as_ref()
                    .is_none_or(|p| !Arc::ptr_eq(p, &draw.texture))
                {
                    backend.queue.write_texture(
                        self.impostor.as_image_copy(),
                        &draw.texture,
                        wgpu::TexelCopyBufferLayout {
                            offset: 0,
                            bytes_per_row: Some(128),
                            rows_per_image: Some(256),
                        },
                        self.impostor.size(),
                    );
                    self.stats.texture_upload_bytes = draw.texture.len() as u64;
                    self.published_impostor = Some(draw.texture.clone());
                }
                bytes(draw.model)
            }
        };
        backend.queue.write_buffer(&self.frame, 0, &frame_bytes);
        backend.queue.write_buffer(&self.draw, 0, &draw_bytes);
        self.stats.frame_upload_bytes = (frame_bytes.len() + draw_bytes.len()) as u64;
        self.stats.draw_calls = 1;
        self.active = Some(frame);
        Ok(())
    }

    /// Load the opaque/sky color, sample its stored depth, never write depth.
    /// This additional pass exists only for an actual active plume.
    pub fn encode(
        &self,
        encoder: &mut wgpu::CommandEncoder,
        color: &wgpu::TextureView,
        depth: &wgpu::TextureView,
    ) {
        let Some(active) = &self.active else {
            return;
        };
        let global = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("plume-camera-depth"),
            layout: &self.global_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: self.frame.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::TextureView(depth),
                },
            ],
        });
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("native-plume-depth-clipped"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: color,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Load,
                    store: wgpu::StoreOp::Store,
                },
            })],
            ..Default::default()
        });
        pass.set_bind_group(0, &global, &[]);
        match active {
            PlumeFrame::Volume(_) => {
                pass.set_pipeline(&self.volume_pipeline);
                pass.set_bind_group(1, &self.volume_bind, &[]);
                pass.draw(0..6, 0..1);
            }
            PlumeFrame::Cone(_) => {
                pass.set_pipeline(&self.cone_pipeline);
                pass.set_bind_group(1, &self.cone_bind, &[]);
                pass.set_vertex_buffer(0, self.cone_vertices.slice(..));
                pass.draw(0..self.cone_count, 0..1);
            }
        }
    }
}

fn pipeline(
    device: &wgpu::Device,
    format: wgpu::TextureFormat,
    source: &str,
    global: &wgpu::BindGroupLayout,
    local: &wgpu::BindGroupLayout,
    cone: bool,
) -> wgpu::RenderPipeline {
    let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("native-plume"),
        source: wgpu::ShaderSource::Wgsl(source.into()),
    });
    let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("native-plume"),
        bind_group_layouts: &[Some(global), Some(local)],
        immediate_size: 0,
    });
    let vertices = [Some(wgpu::VertexBufferLayout {
        array_stride: 20,
        step_mode: wgpu::VertexStepMode::Vertex,
        attributes: &wgpu::vertex_attr_array![0=>Float32x3,1=>Float32x2],
    })];
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("native-plume"),
        layout: Some(&layout),
        vertex: wgpu::VertexState {
            module: &module,
            entry_point: Some("vertex"),
            compilation_options: Default::default(),
            buffers: if cone { &vertices } else { &[] },
        },
        primitive: wgpu::PrimitiveState {
            cull_mode: None,
            ..Default::default()
        },
        depth_stencil: None,
        multisample: Default::default(),
        fragment: Some(wgpu::FragmentState {
            module: &module,
            entry_point: Some("fragment"),
            compilation_options: Default::default(),
            targets: &[Some(wgpu::ColorTargetState {
                format,
                blend: Some(wgpu::BlendState::PREMULTIPLIED_ALPHA_BLENDING),
                write_mask: wgpu::ColorWrites::ALL,
            })],
        }),
        multiview_mask: None,
        cache: None,
    })
}

/// Conservative projected box around the finite cylindrical field. Near-plane
/// crossings/inside views use full-screen coverage, never discard axial rays.
fn clip_rect(view: &ViewUniformBlock, draw: &VolumeDraw) -> Option<[f32; 4]> {
    let origin = Vec3::from_slice(&draw.origin_len);
    let axis = Vec3::from_slice(&draw.axis_radius);
    let end = origin + axis * draw.origin_len[3];
    let r = Vec3::splat(draw.axis_radius[3]);
    let min = origin.min(end) - r;
    let max = origin.max(end) + r;
    let mut rect = Vec4::new(
        f32::INFINITY,
        f32::INFINITY,
        f32::NEG_INFINITY,
        f32::NEG_INFINITY,
    );
    let mut front = false;
    let mut crossing = false;
    for i in 0..8 {
        let p = Vec3::new(
            if i & 1 == 0 { min.x } else { max.x },
            if i & 2 == 0 { min.y } else { max.y },
            if i & 4 == 0 { min.z } else { max.z },
        );
        let clip = view.clip_from_world * p.extend(1.0);
        if clip.w <= 0.0 || clip.z <= 0.0 {
            crossing = true;
            continue;
        }
        front = true;
        let ndc = clip.truncate() / clip.w;
        rect.x = rect.x.min(ndc.x);
        rect.y = rect.y.min(ndc.y);
        rect.z = rect.z.max(ndc.x);
        rect.w = rect.w.max(ndc.y);
    }
    if !front {
        return None;
    }
    if crossing {
        return Some([-1.0, -1.0, 1.0, 1.0]);
    }
    if rect.x > 1.0 || rect.y > 1.0 || rect.z < -1.0 || rect.w < -1.0 {
        return None;
    }
    Some([
        rect.x.max(-1.0),
        rect.y.max(-1.0),
        rect.z.min(1.0),
        rect.w.min(1.0),
    ])
}

fn bytes(values: impl IntoIterator<Item = f32>) -> Vec<u8> {
    values.into_iter().flat_map(f32::to_le_bytes).collect()
}

fn impostor_cone_vertices() -> Vec<f32> {
    let mut vertices = Vec::with_capacity(32 * 6 * 5);
    for i in 0..32 {
        let a = i as f32 * std::f32::consts::TAU / 32.0;
        let b = (i + 1) as f32 * std::f32::consts::TAU / 32.0;
        let va = [a.cos(), -0.5, a.sin(), i as f32 / 32.0, 0.0];
        let vb = [b.cos(), -0.5, b.sin(), (i + 1) as f32 / 32.0, 0.0];
        for v in [
            va,
            vb,
            [0.0, 0.5, 0.0, (i as f32 + 0.5) / 32.0, 1.0],
            [0.0, -0.5, 0.0, 0.5, 0.0],
            vb,
            va,
        ] {
            vertices.extend(v);
        }
    }
    vertices
}

#[cfg(test)]
mod tests {
    #[test]
    fn projected_bound_rejects_offscreen_without_clamping_it_to_an_edge_draw() {
        use super::*;
        let camera = ViewParams {
            world_from_view: glam::DMat4::IDENTITY,
            world_position_m: glam::DVec3::ZERO,
            clip_from_view: glam::dcamera::rh::proj::directx::perspective(0.9, 1.0, 0.1, 1000.0),
            exposure: 1.0,
        };
        let view = ViewUniformBlock::new(&camera).unwrap();
        let mut draw = VolumeDraw {
            origin_len: [1000.0, 0.0, -10.0, 4.0],
            axis_radius: [0.0, 1.0, 0.0, 1.0],
            animation: [8.0, 0.0, 0.0, 0.0],
            diamonds: false,
            tail_dissolve: false,
            profile: Arc::from([]),
        };
        assert!(clip_rect(&view, &draw).is_none());
        draw.origin_len[0] = 0.0;
        assert!(clip_rect(&view, &draw).is_some());
        draw.origin_len[2] = 10.0;
        assert!(clip_rect(&view, &draw).is_none());
        draw.origin_len = [0.0, -2.0, 0.0, 4.0];
        assert_eq!(clip_rect(&view, &draw), Some([-1.0, -1.0, 1.0, 1.0]));
    }

    #[test]
    fn shaders_need_no_native_capabilities() {
        for source in [super::volume_shader_source(), super::cone_shader_source()] {
            let module = naga::front::wgsl::parse_str(&source)
                .unwrap_or_else(|e| panic!("{}", e.emit_to_string(&source)));
            naga::valid::Validator::new(
                naga::valid::ValidationFlags::all(),
                naga::valid::Capabilities::empty(),
            )
            .validate(&module)
            .unwrap();
        }
    }
}
