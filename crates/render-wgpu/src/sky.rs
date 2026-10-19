//! Lazy far-depth consumer of the shared environment field. No extra render
//! target, pass, depth copy, CPU scattering, or per-frame upload.
use crate::environment::EnvironmentBinding;

pub(crate) struct SkyBackground {
    pipeline: wgpu::RenderPipeline,
    bind: wgpu::BindGroup,
}

impl SkyBackground {
    pub fn new(
        device: &wgpu::Device,
        format: wgpu::TextureFormat,
        view: &wgpu::Buffer,
        environment: &EnvironmentBinding,
    ) -> Self {
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("shared-sky-background"),
            source: wgpu::ShaderSource::Wgsl(
                format!(
                    "{}\n{}\n{}",
                    thessa_rcbt_core::shaders::OCEAN_WGSL,
                    crate::environment::shader(0),
                    include_str!("sky.wgsl")
                )
                .into(),
            ),
        });
        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("shared-sky-background"),
            layout: None,
            vertex: wgpu::VertexState {
                module: &module,
                entry_point: Some("sky_vertex"),
                compilation_options: Default::default(),
                buffers: &[],
            },
            primitive: Default::default(),
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: Some(false),
                depth_compare: Some(wgpu::CompareFunction::LessEqual),
                stencil: Default::default(),
                bias: Default::default(),
            }),
            multisample: Default::default(),
            fragment: Some(wgpu::FragmentState {
                module: &module,
                entry_point: Some("sky_fragment"),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            multiview_mask: None,
            cache: None,
        });
        let bind = Self::bind(device, &pipeline, view, environment);
        Self { pipeline, bind }
    }

    fn bind(
        device: &wgpu::Device,
        pipeline: &wgpu::RenderPipeline,
        view: &wgpu::Buffer,
        environment: &EnvironmentBinding,
    ) -> wgpu::BindGroup {
        let [meta, texels] = environment.background_entries();
        device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("shared-sky-background"),
            layout: &pipeline.get_bind_group_layout(0),
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: view.as_entire_binding(),
                },
                meta,
                texels,
            ],
        })
    }

    pub fn rebind(
        &mut self,
        device: &wgpu::Device,
        view: &wgpu::Buffer,
        environment: &EnvironmentBinding,
    ) {
        self.bind = Self::bind(device, &self.pipeline, view, environment);
    }

    pub fn draw(&self, pass: &mut wgpu::RenderPass<'_>) {
        pass.set_pipeline(&self.pipeline);
        pass.set_bind_group(0, &self.bind, &[]);
        pass.draw(0..3, 0..1);
    }
}
