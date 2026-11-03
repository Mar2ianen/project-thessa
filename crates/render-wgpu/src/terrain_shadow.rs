//! Primary land/water consumes the existing mesh scene; no second accelerator.
use crate::{
    Backend,
    mesh::layout::{buffer, entry, sampler, texture},
};
use wgpu::{BindingType as Ty, ShaderStages as Stages, TextureViewDimension as Dimension};

pub(crate) const SHADOW_HOOK: &str = "fn cbt_shadow_visibility(p:vec3<f32>,n:vec3<f32>,sun:u32)->f32 { return rt_visibility(p,n,lighting.directions[sun].xyz,shadow_visibility(p,n,sun)*cloud_terrain_sun_visibility(p,sun),primary_shadow_params.x,primary_shadow_params.y); }";
const PARAMS: &str = "@group(3) @binding(1) var<uniform> primary_shadow_params:vec4<f32>;";

pub(crate) fn scene_layout(device: &wgpu::Device) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("primary-shadow-scene-layout"),
        entries: &[
            entry(
                0,
                Stages::FRAGMENT,
                Ty::AccelerationStructure {
                    vertex_return: false,
                },
            ),
            buffer(1, Stages::FRAGMENT, true),
        ],
    })
}

pub(crate) fn raster_pipeline(
    device: &wgpu::Device,
    module: &wgpu::ShaderModule,
    layout: Option<&wgpu::PipelineLayout>,
    format: wgpu::TextureFormat,
    samples: u32,
    conforming: bool,
    ambient_output: bool,
) -> wgpu::RenderPipeline {
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("cbt-raster"),
        layout,
        vertex: wgpu::VertexState {
            module,
            entry_point: Some("vertex"),
            compilation_options: wgpu::PipelineCompilationOptions {
                constants: &[("CBT_CONFORMING", f64::from(conforming))],
                ..Default::default()
            },
            buffers: &[],
        },
        primitive: wgpu::PrimitiveState {
            cull_mode: if conforming {
                Some(wgpu::Face::Back)
            } else {
                None
            },
            ..Default::default()
        },
        depth_stencil: Some(wgpu::DepthStencilState {
            format: wgpu::TextureFormat::Depth32Float,
            depth_write_enabled: Some(true),
            depth_compare: Some(wgpu::CompareFunction::Less),
            stencil: Default::default(),
            bias: Default::default(),
        }),
        multisample: wgpu::MultisampleState {
            count: samples,
            ..Default::default()
        },
        fragment: Some(wgpu::FragmentState {
            module,
            entry_point: Some("fragment"),
            compilation_options: Default::default(),
            targets: &crate::ambient_visibility::color_targets(format, ambient_output, None),
        }),
        multiview_mask: None,
        cache: None,
    })
}

pub(crate) fn supported(limits: &wgpu::Limits, ocean: bool, sky: bool) -> bool {
    limits.max_bind_groups >= 4
        && limits.max_acceleration_structures_per_shader_stage >= 1
        && limits.max_storage_buffers_per_shader_stage >= 8
        && limits.max_uniform_buffers_per_shader_stage >= 7 + u32::from(ocean) + u32::from(sky)
        && limits.max_sampled_textures_per_shader_stage >= 10 + u32::from(ocean)
        && limits.max_samplers_per_shader_stage >= 5 + u32::from(ocean)
}

pub(crate) struct PrimaryShadow {
    pub pipeline: wgpu::RenderPipeline,
    device: std::sync::Arc<wgpu::Device>,
    source: String,
    format: wgpu::TextureFormat,
    samples: u32,
    conforming: bool,
    layouts: [wgpu::BindGroupLayout; 3],
    masked: std::cell::RefCell<Option<(String, wgpu::RenderPipeline)>>,
    pub geometry: wgpu::BindGroup,
    pub atlas: wgpu::BindGroup,
    pub environment: wgpu::BindGroup,
}
impl PrimaryShadow {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        backend: &Backend,
        raster_source: &str,
        format: wgpu::TextureFormat,
        samples: u32,
        conforming: bool,
        geometry: &[wgpu::BindGroupEntry<'_>],
        atlas: &crate::shadows::ShadowAtlas,
        environment: &[wgpu::BindGroupEntry<'_>],
        ocean: bool,
        sky: bool,
    ) -> Self {
        let device = &backend.device;
        let mut entries = Vec::new();
        for binding in [0, 1, 4, 9] {
            entries.push(buffer(binding, Stages::VERTEX_FRAGMENT, true));
        }
        for binding in [2, 3, 5, 8, 11] {
            entries.push(buffer(binding, Stages::VERTEX, false));
        }
        for binding in [14, 16, 20] {
            entries.push(buffer(binding, Stages::VERTEX_FRAGMENT, false));
        }
        for binding in [6, 10, 12, 17, 18, 19, 22] {
            entries.push(texture(
                binding,
                if [12, 19, 22].contains(&binding) {
                    Dimension::D2Array
                } else {
                    Dimension::D2
                },
            ));
        }
        if geometry.iter().any(|e| e.binding == 21) {
            entries.push(buffer(21, Stages::FRAGMENT, true));
        }
        entries.extend([sampler(7), sampler(13)]);
        let geometry_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("primary-shadow-terrain-layout"),
            entries: &entries,
        });
        let atlas_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("primary-shadow-atlas-layout"),
            entries: &[
                buffer(0, Stages::FRAGMENT, true),
                entry(
                    1,
                    Stages::FRAGMENT,
                    Ty::Texture {
                        sample_type: wgpu::TextureSampleType::Depth,
                        view_dimension: Dimension::D2Array,
                        multisampled: false,
                    },
                ),
                entry(
                    2,
                    Stages::FRAGMENT,
                    Ty::Sampler(wgpu::SamplerBindingType::Comparison),
                ),
            ],
        });
        let mut environment_entries = vec![
            buffer(5, Stages::FRAGMENT, true),
            texture(6, Dimension::D2),
            sampler(7),
            texture(10, Dimension::D3),
            sampler(11),
        ];
        if ocean {
            environment_entries.extend([
                buffer(0, Stages::FRAGMENT, true),
                texture(1, Dimension::D2Array),
                sampler(2),
            ]);
        }
        if sky {
            environment_entries.extend([
                buffer(3, Stages::FRAGMENT, true),
                buffer(4, Stages::FRAGMENT, false),
            ]);
        }
        let environment_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("primary-shadow-environment-layout"),
                entries: &environment_entries,
            });
        let scene_layout = scene_layout(device);
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("primary-shadow-pipeline-layout"),
            bind_group_layouts: &[
                Some(&geometry_layout),
                Some(&atlas_layout),
                Some(&environment_layout),
                Some(&scene_layout),
            ],
            immediate_size: 0,
        });
        let source = format!(
            "enable wgpu_ray_query;\n{}\n{PARAMS}\n{}\n{raster_source}",
            format_args!(
                "{}\n{}",
                thessa_fastrt::visibility::QUERY_POLICY_WGSL,
                crate::ray_scene::QUERY_SHADER
                    .replace("// CANDIDATE_TRAVERSAL", "rayQueryProceed(&query);")
            ),
            crate::shadows::receiver_shader(1)
        );
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("primary-shadow-terrain"),
            source: wgpu::ShaderSource::Wgsl(source.into()),
        });
        let pipeline = raster_pipeline(
            device,
            &module,
            Some(&layout),
            format,
            samples,
            conforming,
            raster_source.contains("struct AmbientSurfaceOutput"),
        );
        let geometry = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("primary-shadow-terrain"),
            layout: &geometry_layout,
            entries: geometry,
        });
        let environment = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("primary-shadow-environment"),
            layout: &environment_layout,
            entries: environment,
        });
        Self {
            device: backend.device.clone(),
            source: raster_source.to_owned(),
            format,
            samples,
            conforming,
            layouts: [geometry_layout, atlas_layout.clone(), environment_layout],
            masked: std::cell::RefCell::new(None),
            pipeline,
            geometry,
            environment,
            atlas: atlas.receiver(backend, &atlas_layout),
        }
    }

    pub fn masked_pipeline(
        &self,
        schema: Option<&crate::ray_scene::PrimarySchema>,
    ) -> std::cell::Ref<'_, Option<(String, wgpu::RenderPipeline)>> {
        if let Some(schema) = schema {
            let mut cached = self.masked.borrow_mut();
            if cached.as_ref().is_none_or(|(key, _)| key != &schema.shader) {
                let source = format!(
                    "enable wgpu_ray_query;\n{}\n{}\n{}\n{}",
                    schema.shader,
                    PARAMS.replace(
                        "@binding(1)",
                        &format!("@binding({})", schema.params_binding)
                    ),
                    crate::shadows::receiver_shader(1),
                    self.source
                );
                let module = self
                    .device
                    .create_shader_module(wgpu::ShaderModuleDescriptor {
                        label: Some("masked-primary-shadow"),
                        source: wgpu::ShaderSource::Wgsl(source.into()),
                    });
                let layout = self
                    .device
                    .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                        label: Some("masked-primary-shadow"),
                        bind_group_layouts: &[
                            Some(&self.layouts[0]),
                            Some(&self.layouts[1]),
                            Some(&self.layouts[2]),
                            Some(&schema.layout),
                        ],
                        immediate_size: 0,
                    });
                let pipeline = raster_pipeline(
                    &self.device,
                    &module,
                    Some(&layout),
                    self.format,
                    self.samples,
                    self.conforming,
                    self.source.contains("struct AmbientSurfaceOutput"),
                );
                *cached = Some((schema.shader.clone(), pipeline));
            }
        }
        self.masked.borrow()
    }

    pub fn rebind_environment(
        &mut self,
        device: &wgpu::Device,
        entries: &[wgpu::BindGroupEntry<'_>],
    ) {
        self.environment = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("primary-shadow-environment"),
            layout: &self.pipeline.get_bind_group_layout(2),
            entries,
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use wgpu::util::DeviceExt;

    #[test]
    fn unsupported_receiver_limits_keep_the_existing_raster_path() {
        let mut limits = wgpu::Limits {
            max_acceleration_structures_per_shader_stage: 1,
            ..Default::default()
        };
        assert!(supported(&limits, true, true));
        limits.max_uniform_buffers_per_shader_stage = 8;
        assert!(!supported(&limits, true, true));
        assert!(supported(&limits, true, false));
        limits.max_acceleration_structures_per_shader_stage = 0;
        assert!(!supported(&limits, false, false));
    }

    #[test]
    #[ignore = "GPU: FastRT scalar visibility policy vs f64 oracle, no RT feature required"]
    fn fastrt_visibility_policy_matches_f64_reference() {
        let backend = Backend::bring_up().unwrap();
        let inputs: [[f32; 4]; 12] = [
            [0., 1., 0.01, 100.],
            [1., -0.5, 0.01, 100.],
            [1., 0., 0., 100.],
            [1., 0.01, 1., 100.],
            [1., 0.5, 0.01, 100.],
            [0.5, 0.5, 0.01, 100.],
            [1., 1., 0., 100.],
            [f32::NAN, 1., 0.01, 100.],
            [1., f32::INFINITY, 0.01, 100.],
            [1., 1., -0.01, 100.],
            [1., 1., 0.01, f32::INFINITY],
            [1., 1., 0.01, 0.],
        ];
        let input = backend
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("visibility-policy-input"),
                contents: &inputs
                    .iter()
                    .flatten()
                    .flat_map(|x| x.to_le_bytes())
                    .collect::<Vec<_>>(),
                usage: wgpu::BufferUsages::STORAGE,
            });
        let output = backend.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("visibility-policy-output"),
            size: inputs.len() as u64 * 4,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        let module = backend.device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label:Some("fastrt-visibility-policy"),source:wgpu::ShaderSource::Wgsl(format!(
                "{}\n@group(0) @binding(0) var<storage,read> inputs:array<vec4<f32>>;\n@group(0) @binding(1) var<storage,read_write> outputs:array<f32>;\n@compute @workgroup_size(1) fn probe(@builtin(global_invocation_id) id:vec3<u32>){{let v=inputs[id.x];outputs[id.x]=fastrt_visibility_min_distance(v.x,v.y,v.z,v.w);}}",
                thessa_fastrt::visibility::QUERY_POLICY_WGSL).into()),
        });
        let pipeline = backend
            .device
            .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some("fastrt-visibility-policy"),
                layout: None,
                module: &module,
                entry_point: Some("probe"),
                compilation_options: Default::default(),
                cache: None,
            });
        let bind = backend
            .device
            .create_bind_group(&wgpu::BindGroupDescriptor {
                label: None,
                layout: &pipeline.get_bind_group_layout(0),
                entries: &[
                    wgpu::BindGroupEntry {
                        binding: 0,
                        resource: input.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 1,
                        resource: output.as_entire_binding(),
                    },
                ],
            });
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        {
            let mut pass = encoder.begin_compute_pass(&Default::default());
            pass.set_pipeline(&pipeline);
            pass.set_bind_group(0, &bind, &[]);
            pass.dispatch_workgroups(inputs.len() as u32, 1, 1);
        }
        backend.queue.submit([encoder.finish()]);
        let bytes = crate::terrain::read_buffer(&backend, &output).unwrap();
        for (record, bytes) in inputs.iter().zip(bytes.as_chunks::<4>().0) {
            let value = f32::from_le_bytes(*bytes);
            let [v, c, b, d] = record.map(f64::from);
            let expected = thessa_fastrt::visibility::query_min_distance(v, c, b, d)
                .ok()
                .flatten();
            match expected {
                None => assert_eq!(value, -1., "{record:?}"),
                Some(expected) => assert!(
                    (f64::from(value) - expected).abs() < 1e-7,
                    "{record:?}: {value} vs {expected}"
                ),
            }
        }
        eprintln!(
            "FastRT visibility policy adapter={} inputs={}",
            backend.adapter_name,
            inputs.len()
        );
    }
}
