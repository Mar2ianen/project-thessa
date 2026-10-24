//! Native SDR presentation of the existing pre-exposed linear scene.
//! HDR-off is resource-free. HDR-on adds one RGBA16Float attachment and one
//! fullscreen pass; the caller draws display-referred UI afterwards.
use crate::{Backend, BackendError};

pub const HDR_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba16Float;

pub struct DisplayTransform {
    hdr: Option<Hdr>,
    output_format: wgpu::TextureFormat,
    extent: [u32; 2],
    tone_mapping: thessa_graphics::ToneMapping,
}
struct Hdr {
    texture: wgpu::Texture,
    view: wgpu::TextureView,
    pipeline: wgpu::RenderPipeline,
    bind: wgpu::BindGroup,
}

impl DisplayTransform {
    /// Settings must come from the active backend's capability resolver. This
    /// slice is manual exposure only; auto exposure/bloom are not implemented.
    pub fn new(
        backend: &Backend,
        settings: &thessa_graphics::ResolvedGraphicsSettings,
        output_format: wgpu::TextureFormat,
        width: u32,
        height: u32,
    ) -> Result<Self, BackendError> {
        validate_extent(backend, width, height)?;
        let hdr = if settings.hdr {
            if !matches!(
                output_format,
                wgpu::TextureFormat::Rgba8Unorm
                    | wgpu::TextureFormat::Rgba8UnormSrgb
                    | wgpu::TextureFormat::Bgra8Unorm
                    | wgpu::TextureFormat::Bgra8UnormSrgb
            ) {
                return Err(BackendError::Device(
                    "native display requires an 8-bit SDR output attachment".into(),
                ));
            }
            if crate::probe_capabilities(&backend.adapter).hdr_render_target_supported != Some(true)
            {
                return Err(BackendError::Device(
                    "HDR not supported; resolve settings before constructing display".into(),
                ));
            }
            let module = backend
                .device
                .create_shader_module(wgpu::ShaderModuleDescriptor {
                    label: Some("native-display-reinhard"),
                    source: wgpu::ShaderSource::Wgsl(
                        include_str!("display.wgsl")
                            .replace(
                                "OUTPUT_IS_SRGB",
                                if output_format.is_srgb() {
                                    "true"
                                } else {
                                    "false"
                                },
                            )
                            .into(),
                    ),
                });
            let pipeline = backend
                .device
                .create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                    label: Some("native-display-reinhard"),
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
                        compilation_options: wgpu::PipelineCompilationOptions {
                            constants: &[(
                                "TONEMAP_OPERATOR",
                                if settings.tone_mapping == thessa_graphics::ToneMapping::PbrNeutral
                                {
                                    1.0
                                } else {
                                    0.0
                                },
                            )],
                            ..Default::default()
                        },
                        targets: &[Some(wgpu::ColorTargetState {
                            format: output_format,
                            blend: None,
                            write_mask: wgpu::ColorWrites::ALL,
                        })],
                    }),
                    primitive: Default::default(),
                    depth_stencil: None,
                    multisample: Default::default(),
                    multiview_mask: None,
                    cache: None,
                });
            let texture = target(backend, width, height);
            let view = texture.create_view(&Default::default());
            let bind = bind(backend, &pipeline, &view);
            Some(Hdr {
                texture,
                view,
                pipeline,
                bind,
            })
        } else {
            None
        };
        Ok(Self {
            hdr,
            output_format,
            extent: [width, height],
            tone_mapping: settings.tone_mapping,
        })
    }

    pub fn scene_format(&self) -> wgpu::TextureFormat {
        if self.hdr.is_some() {
            HDR_FORMAT
        } else {
            self.output_format
        }
    }
    /// Pre-exposed, scene-linear target. LDR borrows the output directly.
    pub fn scene_color<'a>(&'a self, output: &'a wgpu::TextureView) -> &'a wgpu::TextureView {
        self.hdr.as_ref().map_or(output, |hdr| &hdr.view)
    }
    pub fn scene_texture(&self) -> Option<&wgpu::Texture> {
        self.hdr.as_ref().map(|hdr| &hdr.texture)
    }
    pub fn target_bytes(&self) -> u64 {
        if self.hdr.is_some() {
            u64::from(self.extent[0]) * u64::from(self.extent[1]) * 8
        } else {
            0
        }
    }
    pub fn enabled(&self) -> bool {
        self.hdr.is_some()
    }

    pub fn tone_mapping(&self) -> &'static str {
        if self.enabled() {
            self.tone_mapping.as_str()
        } else {
            "none"
        }
    }
    /// Zero/minimized or inadmissible resize leaves all live resources intact.
    /// Only target and bind group change; pipelines/scene geometry stay retained.
    pub fn resize(
        &mut self,
        backend: &Backend,
        width: u32,
        height: u32,
    ) -> Result<(), BackendError> {
        validate_extent(backend, width, height)?;
        if self.extent == [width, height] {
            return Ok(());
        }
        if let Some(hdr) = &mut self.hdr {
            let texture = target(backend, width, height);
            let view = texture.create_view(&Default::default());
            hdr.texture = texture;
            hdr.bind = bind(backend, &hdr.pipeline, &view);
            hdr.view = view;
        }
        self.extent = [width, height];
        Ok(())
    }
    /// Call after scene raster and before UI. No CPU work/upload/allocation on
    /// the inactive path, and no host readback or temporal state on either path.
    pub fn encode(&self, encoder: &mut wgpu::CommandEncoder, output: &wgpu::TextureView) {
        let Some(hdr) = &self.hdr else {
            return;
        };
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("native-display-reinhard"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: output,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                    store: wgpu::StoreOp::Store,
                },
            })],
            ..Default::default()
        });
        pass.set_pipeline(&hdr.pipeline);
        pass.set_bind_group(0, &hdr.bind, &[]);
        pass.draw(0..3, 0..1);
    }
}

fn validate_extent(backend: &Backend, width: u32, height: u32) -> Result<(), BackendError> {
    let max = backend.device.limits().max_texture_dimension_2d;
    if width == 0 || height == 0 || width > max || height > max {
        Err(BackendError::Device("invalid display extent".into()))
    } else {
        Ok(())
    }
}
fn target(backend: &Backend, width: u32, height: u32) -> wgpu::Texture {
    backend.device.create_texture(&wgpu::TextureDescriptor {
        label: Some("native-pre-exposed-hdr"),
        size: wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: HDR_FORMAT,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT
            | wgpu::TextureUsages::TEXTURE_BINDING
            | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    })
}
fn bind(
    backend: &Backend,
    pipeline: &wgpu::RenderPipeline,
    view: &wgpu::TextureView,
) -> wgpu::BindGroup {
    backend
        .device
        .create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("native-display-reinhard"),
            layout: &pipeline.get_bind_group_layout(0),
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::TextureView(view),
            }],
        })
}
