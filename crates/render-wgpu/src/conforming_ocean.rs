//! Diagnostic geometry adapter; borrows the existing spectral sea and material
//! residency. No FFT, streamer, datum backup or wave model is introduced here.
use crate::{Backend, BackendError, material::MaterialBinding, ocean::OceanField};
use thessa_graphics::ResolvedGraphicsSettings;
use wgpu::util::DeviceExt;

pub(crate) struct ConformingOcean {
    pub extent: wgpu::Buffer,
    policy: wgpu::Buffer,
    measure: wgpu::ComputePipeline,
    deform: wgpu::ComputePipeline,
    bind: wgpu::BindGroup,
    field_bind: wgpu::BindGroup,
}

impl ConformingOcean {
    pub fn new(
        backend: &Backend,
        vertices: &wgpu::Buffer,
        settings: &ResolvedGraphicsSettings,
        field: Option<&OceanField>,
        materials: &MaterialBinding,
    ) -> Result<Option<Self>, BackendError> {
        let ocean = &settings.ocean;
        if !ocean.enabled || ocean.geometry_patches == 0 || ocean.geometry_distance_m == 0.0 {
            return Ok(None);
        }
        if ocean.geometry_patches > 64
            || !ocean.geometry_distance_m.is_finite()
            || !(0.0..=20_000.0).contains(&ocean.geometry_distance_m)
        {
            return Err(BackendError::Device(
                "invalid conforming ocean geometry policy".into(),
            ));
        }
        let field = field.ok_or_else(|| {
            BackendError::Device("conforming ocean needs the published spectral field".into())
        })?;
        if field.stats().size != settings.ocean_size {
            return Err(BackendError::Device(
                "conforming ocean field/resolved size mismatch".into(),
            ));
        }
        let device = &backend.device;
        let extent = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("cbt-ocean-extent"),
            size: 16,
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_DST
                | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        let policy = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("cbt-ocean-policy"),
            contents: [
                (ocean.geometry_distance_m as f32).to_bits(),
                ocean.geometry_patches * 2048,
                0,
                0,
            ]
            .map(u32::to_le_bytes)
            .as_flattened(),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("cbt-ocean"),
            source: wgpu::ShaderSource::Wgsl(shader_source().into()),
        });
        let entries = (0..6)
            .map(|binding| wgpu::BindGroupLayoutEntry {
                binding,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: match binding {
                    3 => wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Float { filterable: true },
                        view_dimension: wgpu::TextureViewDimension::D2Array,
                        multisampled: false,
                    },
                    4 => wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                    _ => wgpu::BindingType::Buffer {
                        ty: if binding == 2 {
                            wgpu::BufferBindingType::Uniform
                        } else {
                            wgpu::BufferBindingType::Storage {
                                read_only: binding == 5,
                            }
                        },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                },
                count: None,
            })
            .collect::<Vec<_>>();
        let sources = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("cbt-ocean-sources"),
            entries: &entries,
        });
        let field_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("cbt-ocean-field"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Float { filterable: true },
                        view_dimension: wgpu::TextureViewDimension::D2Array,
                        multisampled: false,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 2,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                    count: None,
                },
            ],
        });
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: None,
            bind_group_layouts: &[Some(&sources), Some(&field_layout)],
            immediate_size: 0,
        });
        let pipeline = |name| {
            device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some(name),
                layout: Some(&layout),
                module: &module,
                entry_point: Some(name),
                compilation_options: Default::default(),
                cache: None,
            })
        };
        let measure = pipeline("measure_ocean_extent");
        let deform = pipeline("deform_conforming_ocean");
        let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("cbt-ocean-sources"),
            layout: &sources,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: vertices.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: extent.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: policy.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: wgpu::BindingResource::TextureView(&materials.texture),
                },
                wgpu::BindGroupEntry {
                    binding: 4,
                    resource: wgpu::BindingResource::Sampler(&materials.sampler),
                },
                wgpu::BindGroupEntry {
                    binding: 5,
                    resource: materials.directory.as_entire_binding(),
                },
            ],
        });
        let field_bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("cbt-ocean-field"),
            layout: &field_layout,
            entries: &field.binding().entries(),
        });
        Ok(Some(Self {
            extent,
            policy,
            measure,
            deform,
            bind,
            field_bind,
        }))
    }

    pub fn encode(&self, encoder: &mut wgpu::CommandEncoder, capacity: u32) {
        encoder.clear_buffer(&self.extent, 0, None);
        // End the reduction pass before any vertex writer starts.
        for (pipeline, count) in [(&self.measure, capacity), (&self.deform, capacity * 3)] {
            let mut pass = encoder.begin_compute_pass(&Default::default());
            pass.set_pipeline(pipeline);
            pass.set_bind_group(0, &self.bind, &[]);
            pass.set_bind_group(1, &self.field_bind, &[]);
            pass.dispatch_workgroups(count.div_ceil(64), 1, 1);
        }
    }

    pub fn storage_bytes(&self) -> u64 {
        self.extent.size() + self.policy.size()
    }
}

pub(crate) fn shader_source() -> String {
    format!(
        "{}\n{}\n{}",
        crate::ocean::geometry_material_shader(5),
        crate::ocean::shader(1),
        include_str!("conforming_surface_ocean.wgsl")
    )
}
