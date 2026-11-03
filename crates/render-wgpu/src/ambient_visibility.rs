//! Optional current-frame diffuse-sky receiver and screen-space visibility.
//! Direct light, specular, emission and air scattering are never modulated.
use crate::{
    Backend, BackendError,
    view::{ViewParams, ViewUniformBlock},
};
use thessa_graphics::ResolvedGraphicsSettings;

pub(crate) fn color_targets(
    format: wgpu::TextureFormat,
    ambient: bool,
    blend: Option<wgpu::BlendState>,
) -> Vec<Option<wgpu::ColorTargetState>> {
    let mut targets = vec![Some(wgpu::ColorTargetState {
        format,
        blend,
        write_mask: wgpu::ColorWrites::ALL,
    })];
    if ambient {
        targets.push(Some(wgpu::ColorTargetState {
            format: wgpu::TextureFormat::Rgba16Float,
            blend: None,
            write_mask: wgpu::ColorWrites::ALL,
        }));
    }
    targets
}
const OUTPUT: &str = "struct AmbientSurfaceOutput { @location(0) beauty:vec4<f32>, @location(1) diffuse_sky:vec4<f32> };\n";

pub(crate) fn mesh_shader(source: String) -> String {
    OUTPUT.to_owned()
        + &source
            .replace(
                "ambient_component:vec3<f32>,baked_visibility:f32)->vec4<f32>",
                "ambient_component:vec3<f32>,baked_visibility:f32)->AmbientSurfaceOutput",
            )
            .replace(
                "front:bool)->@location(0) vec4<f32>",
                "front:bool)->AmbientSurfaceOutput",
            )
            .replace(
                "return vec4(radiance*view.exposure,",
                "return AmbientSurfaceOutput(vec4(radiance*view.exposure,",
            )
            .replace(
                "1.0);\n}\n@fragment",
                "1.0),vec4(ambient_component*view.exposure,clamp(baked_visibility,0.0,1.0)));\n}\n@fragment",
            )
            .replace(
                "0.0);\n}\n@fragment",
                "0.0),vec4(ambient_component*view.exposure,clamp(baked_visibility,0.0,1.0)));\n}\n@fragment",
            )
            .replace(
                "material.color.a==1.0));\n}\n@fragment",
                "material.color.a==1.0)),vec4(ambient_component*view.exposure,clamp(baked_visibility,0.0,1.0)));\n}\n@fragment",
            )
}

pub(crate) fn terrain_shader(source: String) -> String {
    let (prefix, fragment) = source
        .split_once("fn fragment(input: VertexOutput) -> @location(0) vec4<f32> {")
        .expect("terrain fragment ABI");
    // Debug returns precede material evaluation and carry no diffuse-sky term.
    let fragment=fragment.replace("    // AMBIENT_COMPONENT_OUTPUT", "    exported_ambient=ambient_component*(1.0-water_material)*view.exposure*env_haze_transmittance(input.render_position);\n    // AMBIENT_COMPONENT_OUTPUT");
    let mut wrapped = String::new();
    let mut tail = fragment.as_str();
    while let Some(at) = tail.find("return vec4(") {
        wrapped.push_str(&tail[..at]);
        let start = at + "return ".len();
        let mut end = start;
        let mut nesting = 0;
        for (i, c) in tail[start..].char_indices() {
            if c == '(' {
                nesting += 1;
            }
            if c == ')' {
                nesting -= 1;
                if nesting == 0 {
                    end = start + i + 1;
                    break;
                }
            }
        }
        assert!(end > start, "terrain return ABI");
        wrapped.push_str("return AmbientSurfaceOutput(");
        wrapped.push_str(&tail[start..end]);
        wrapped.push_str(",vec4(exported_ambient,1.0))");
        tail = &tail[end..];
    }
    wrapped.push_str(tail);
    format!(
        "{OUTPUT}{prefix}fn fragment(input: VertexOutput) -> AmbientSurfaceOutput {{\nvar exported_ambient=vec3(0.0);{wrapped}"
    )
}

pub struct AmbientVisibility {
    pub texture: wgpu::Texture,
    view: wgpu::TextureView,
    multisampled: Option<wgpu::TextureView>,
    pipeline: wgpu::RenderPipeline,
    params: wgpu::Buffer,
    binding: Option<(
        wgpu::TextureView,
        Option<wgpu::TextureView>,
        wgpu::BindGroup,
    )>,
    settings: ResolvedGraphicsSettings,
    extent: [u32; 2],
}
impl AmbientVisibility {
    pub fn new(
        backend: &Backend,
        settings: &ResolvedGraphicsSettings,
        extent: [u32; 2],
    ) -> Result<Option<Self>, BackendError> {
        if settings.ambient_visibility_samples == 0 {
            return Ok(None);
        }
        let bad = || {
            BackendError::Device("ambient visibility requires resolved HDR, 8/16/32 samples, finite radius and valid extent".into())
        };
        if !settings.hdr
            || ![1, 4].contains(&settings.raster_msaa_samples)
            || ![8, 16, 32].contains(&settings.ambient_visibility_samples)
            || !settings.ambient_visibility_radius_m.is_finite()
            || !(0.05..=20.).contains(&settings.ambient_visibility_radius_m)
            || extent.contains(&0)
            || extent
                .iter()
                .any(|&v| v > backend.device.limits().max_texture_dimension_2d)
        {
            return Err(bad());
        }
        let make = |samples| {
            backend.device.create_texture(&wgpu::TextureDescriptor {
                label: Some("diffuse-sky-receiver"),
                size: wgpu::Extent3d {
                    width: extent[0],
                    height: extent[1],
                    depth_or_array_layers: 1,
                },
                mip_level_count: 1,
                sample_count: samples,
                dimension: wgpu::TextureDimension::D2,
                format: wgpu::TextureFormat::Rgba16Float,
                usage: wgpu::TextureUsages::RENDER_ATTACHMENT
                    | wgpu::TextureUsages::TEXTURE_BINDING
                    | if samples == 1 {
                        wgpu::TextureUsages::COPY_SRC
                    } else {
                        wgpu::TextureUsages::empty()
                    },
                view_formats: &[],
            })
        };
        let texture = make(1);
        let view = texture.create_view(&Default::default());
        let multisampled = (settings.raster_msaa_samples > 1)
            .then(|| make(settings.raster_msaa_samples).create_view(&Default::default()));
        let mut source = include_str!("ambient_visibility.wgsl").replace(
            "SAMPLE_COUNT",
            &settings.ambient_visibility_samples.to_string(),
        );
        if multisampled.is_some() {
            source=source.replace("// MSAA_BINDING","@group(0) @binding(3) var coverage_depth:texture_depth_multisampled_2d;")
                .replace("// COVERAGE_CHECK", "var near=1.0;var far=0.0;for(var i=0;i<4;i++){let d=textureLoad(coverage_depth,pixel,i);near=min(near,d);far=max(far,d);}\nif(near<1.0 && (far>=1.0 || length(position(uv,far)-position(uv,near))>max(0.002,pixel_span))){return vec4(0.0);}");
        }
        let module = backend
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("diffuse-sky-visibility"),
                source: wgpu::ShaderSource::Wgsl(source.into()),
            });
        let pipeline = backend
            .device
            .create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some("diffuse-sky-visibility"),
                layout: None,
                vertex: wgpu::VertexState {
                    module: &module,
                    entry_point: Some("vertex"),
                    compilation_options: Default::default(),
                    buffers: &[],
                },
                primitive: Default::default(),
                depth_stencil: None,
                multisample: wgpu::MultisampleState {
                    count: settings.raster_msaa_samples,
                    ..Default::default()
                },
                fragment: Some(wgpu::FragmentState {
                    module: &module,
                    entry_point: Some("fragment"),
                    compilation_options: Default::default(),
                    targets: &[Some(wgpu::ColorTargetState {
                        format: wgpu::TextureFormat::Rgba16Float,
                        blend: Some(wgpu::BlendState {
                            color: wgpu::BlendComponent {
                                src_factor: wgpu::BlendFactor::One,
                                dst_factor: wgpu::BlendFactor::One,
                                operation: wgpu::BlendOperation::ReverseSubtract,
                            },
                            alpha: wgpu::BlendComponent {
                                src_factor: wgpu::BlendFactor::Zero,
                                dst_factor: wgpu::BlendFactor::One,
                                operation: wgpu::BlendOperation::Add,
                            },
                        }),
                        write_mask: wgpu::ColorWrites::ALL,
                    })],
                }),
                multiview_mask: None,
                cache: None,
            });
        let params = backend.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("diffuse-sky-visibility-params"),
            size: 144,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        Ok(Some(Self {
            texture,
            view,
            multisampled,
            pipeline,
            params,
            binding: None,
            settings: settings.clone(),
            extent,
        }))
    }
    pub fn attachment(&self) -> wgpu::RenderPassColorAttachment<'_> {
        wgpu::RenderPassColorAttachment {
            view: self.multisampled.as_ref().unwrap_or(&self.view),
            depth_slice: None,
            resolve_target: self.multisampled.as_ref().map(|_| &self.view),
            ops: wgpu::Operations {
                load: wgpu::LoadOp::Clear(wgpu::Color::TRANSPARENT),
                store: wgpu::StoreOp::Store,
            },
        }
    }
    pub fn target_bytes(&self) -> u64 {
        u64::from(self.extent[0])
            * u64::from(self.extent[1])
            * 8
            * (if self.multisampled.is_some() { 5 } else { 1 })
            + 144
    }
    pub fn resize(&mut self, backend: &Backend, extent: [u32; 2]) -> Result<(), BackendError> {
        if extent != self.extent {
            *self = Self::new(backend, &self.settings, extent)?.unwrap();
        }
        Ok(())
    }
    #[allow(clippy::too_many_arguments)]
    pub fn encode(
        &mut self,
        backend: &Backend,
        encoder: &mut wgpu::CommandEncoder,
        color: &wgpu::TextureView,
        resolve: Option<&wgpu::TextureView>,
        depth: &wgpu::TextureView,
        multisampled_depth: Option<&wgpu::TextureView>,
        view: &ViewParams,
        jitter: thessa_render_core::temporal::JitterOffset,
        enabled: bool,
    ) -> Result<(), BackendError> {
        let block =
            ViewUniformBlock::new_jittered(view, view.world_position_m, jitter, self.extent)
                .ok_or_else(|| BackendError::Device("invalid ambient visibility view".into()))?;
        let bytes = block
            .clip_from_world
            .to_cols_array()
            .into_iter()
            .chain(block.world_from_clip.to_cols_array())
            .chain([
                self.settings.ambient_visibility_radius_m,
                if enabled { 1.0 } else { 0.0 },
                self.extent[0] as f32,
                self.extent[1] as f32,
            ])
            .flat_map(f32::to_le_bytes)
            .collect::<Vec<_>>();
        backend.queue.write_buffer(&self.params, 0, &bytes);
        if self
            .binding
            .as_ref()
            .is_none_or(|(d, m, _)| d != depth || m.as_ref() != multisampled_depth)
        {
            let mut entries = vec![
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: self.params.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::TextureView(depth),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: wgpu::BindingResource::TextureView(&self.view),
                },
            ];
            if let Some(d) = multisampled_depth {
                entries.push(wgpu::BindGroupEntry {
                    binding: 3,
                    resource: wgpu::BindingResource::TextureView(d),
                });
            }
            self.binding = Some((
                depth.clone(),
                multisampled_depth.cloned(),
                backend
                    .device
                    .create_bind_group(&wgpu::BindGroupDescriptor {
                        label: Some("diffuse-sky-visibility"),
                        layout: &self.pipeline.get_bind_group_layout(0),
                        entries: &entries,
                    }),
            ));
        }
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("diffuse-sky-visibility"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: color,
                depth_slice: None,
                resolve_target: resolve,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Load,
                    store: wgpu::StoreOp::Store,
                },
            })],
            ..Default::default()
        });
        pass.set_pipeline(&self.pipeline);
        pass.set_bind_group(0, &self.binding.as_ref().unwrap().2, &[]);
        pass.draw(0..3, 0..1);
        Ok(())
    }
}
