//! Optional opaque coverage targets. Colour resolves in hardware; finite depth
//! resolves to the nearest covered sample for existing single-sample consumers.
use crate::{Backend, BackendError};

pub struct SceneMsaa {
    pub color: wgpu::TextureView,
    depth: wgpu::TextureView,
    depth_bind: wgpu::BindGroup,
    depth_pipeline: wgpu::RenderPipeline,
    format: wgpu::TextureFormat,
    extent: [u32; 2],
}
impl SceneMsaa {
    pub fn new(
        backend: &Backend,
        settings: &thessa_graphics::ResolvedGraphicsSettings,
        format: wgpu::TextureFormat,
        extent: [u32; 2],
    ) -> Result<Option<Self>, BackendError> {
        if settings.raster_msaa_samples == 1 {
            return Ok(None);
        }
        if settings.raster_msaa_samples != 4 {
            return Err(BackendError::Device(
                "unsupported resolved MSAA count".into(),
            ));
        }
        Self::allocate(backend, format, extent).map(Some)
    }
    fn allocate(
        backend: &Backend,
        format: wgpu::TextureFormat,
        extent: [u32; 2],
    ) -> Result<Self, BackendError> {
        if extent.contains(&0)
            || extent
                .iter()
                .any(|&v| v > backend.device.limits().max_texture_dimension_2d)
        {
            return Err(BackendError::Device("invalid MSAA target extent".into()));
        }
        let make = |label, format, usage| {
            backend
                .device
                .create_texture(&wgpu::TextureDescriptor {
                    label: Some(label),
                    size: wgpu::Extent3d {
                        width: extent[0],
                        height: extent[1],
                        depth_or_array_layers: 1,
                    },
                    mip_level_count: 1,
                    sample_count: 4,
                    dimension: wgpu::TextureDimension::D2,
                    format,
                    usage,
                    view_formats: &[],
                })
                .create_view(&Default::default())
        };
        let color = make(
            "scene-msaa-color",
            format,
            wgpu::TextureUsages::RENDER_ATTACHMENT,
        );
        let depth = make(
            "scene-msaa-depth",
            wgpu::TextureFormat::Depth32Float,
            wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
        );
        let module=backend.device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label:Some("scene-msaa-nearest-depth"),source:wgpu::ShaderSource::Wgsl(r#"
@group(0) @binding(0) var source_depth:texture_depth_multisampled_2d;
@vertex fn vertex(@builtin(vertex_index) i:u32)->@builtin(position) vec4<f32> {
    let p=array<vec2<f32>,3>(vec2(-1.0,-1.0),vec2(3.0,-1.0),vec2(-1.0,3.0));return vec4(p[i],0.0,1.0);
}
@fragment fn fragment(@builtin(position) p:vec4<f32>)->@builtin(frag_depth) f32 {
    var nearest=1.0;for(var i=0;i<4;i++){nearest=min(nearest,textureLoad(source_depth,vec2<i32>(p.xy),i));}return nearest;
}
"#.into())});
        let depth_pipeline =
            backend
                .device
                .create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                    label: Some("scene-msaa-nearest-depth"),
                    layout: None,
                    vertex: wgpu::VertexState {
                        module: &module,
                        entry_point: Some("vertex"),
                        compilation_options: Default::default(),
                        buffers: &[],
                    },
                    primitive: Default::default(),
                    depth_stencil: Some(wgpu::DepthStencilState {
                        format: wgpu::TextureFormat::Depth32Float,
                        depth_write_enabled: Some(true),
                        depth_compare: Some(wgpu::CompareFunction::Always),
                        stencil: Default::default(),
                        bias: Default::default(),
                    }),
                    multisample: Default::default(),
                    fragment: Some(wgpu::FragmentState {
                        module: &module,
                        entry_point: Some("fragment"),
                        compilation_options: Default::default(),
                        targets: &[],
                    }),
                    multiview_mask: None,
                    cache: None,
                });
        let depth_bind = backend
            .device
            .create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("scene-msaa-nearest-depth"),
                layout: &depth_pipeline.get_bind_group_layout(0),
                entries: &[wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(&depth),
                }],
            });
        Ok(Self {
            color,
            depth,
            depth_bind,
            depth_pipeline,
            format,
            extent,
        })
    }
    pub fn depth(&self) -> &wgpu::TextureView {
        &self.depth
    }
    pub fn target_bytes(&self) -> u64 {
        let color_bytes = if self.format == wgpu::TextureFormat::Rgba16Float {
            8
        } else {
            4
        };
        u64::from(self.extent[0]) * u64::from(self.extent[1]) * 4 * (color_bytes + 4)
    }
    pub fn resize(&mut self, backend: &Backend, extent: [u32; 2]) -> Result<(), BackendError> {
        if self.extent != extent {
            *self = Self::allocate(backend, self.format, extent)?;
        }
        Ok(())
    }
    pub fn resolve_depth(&self, encoder: &mut wgpu::CommandEncoder, target: &wgpu::TextureView) {
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("scene-msaa-nearest-depth"),
            color_attachments: &[],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: target,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(1.0),
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            }),
            ..Default::default()
        });
        pass.set_pipeline(&self.depth_pipeline);
        pass.set_bind_group(0, &self.depth_bind, &[]);
        pass.draw(0..3, 0..1);
    }
}
