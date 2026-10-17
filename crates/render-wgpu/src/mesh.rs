//! Persistent opaque textured mesh consumer. Loads the caller's existing
//! color/depth targets after terrain; no scene copy, readback or extra device.
use crate::{
    Backend, BackendError,
    view::{ViewParams, ViewUniformBlock},
};
use thessa_rcbt_core::lighting::SurfaceLighting;
use thessa_render_core::mesh::{MeshAsset, MeshInstance, Wrap};
use wgpu::util::DeviceExt;

struct Primitive {
    vertices: wgpu::Buffer,
    indices: wgpu::Buffer,
    count: u32,
    material: usize,
}
/// Submitted workload, not visibility results or GPU timings.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MeshDrawStats {
    pub instances: u32,
    pub draw_calls: usize,
    pub triangles: u64,
    /// Uniform + instance bytes written by one successful `update`.
    pub frame_upload_bytes: usize,
}
/// Surface/lighting configuration for a native mesh renderer. Bundling the
/// resolved settings in one struct keeps the constructor readable as shadows,
/// material quality and ray tracing join it.
pub struct SurfaceSettings<'a> {
    pub format: wgpu::TextureFormat,
    /// Rigid-instance capacity.
    pub capacity: usize,
    pub quality: thessa_graphics::Quality,
    /// Specular reflections (terrain and mesh) on/off.
    pub reflections: bool,
    pub shadows: Option<&'a crate::shadows::ShadowAtlas>,
    /// Resolved graphics settings; `None` keeps the raster-only path.
    pub ray_tracing: Option<&'a thessa_graphics::ResolvedGraphicsSettings>,
}

pub struct MeshRenderer {
    shadow_receiver: Option<wgpu::BindGroup>,
    shadow_caster: Option<wgpu::BindGroup>,
    shadow_instances: Vec<u8>,
    shadow_revision: u64,
    rt: Option<crate::ray_scene::RayScene>,
    rt_reason: Option<String>,
    environment: crate::environment::EnvironmentBinding,
    pipeline: wgpu::RenderPipeline,
    global: wgpu::BindGroup,
    materials: Vec<wgpu::BindGroup>,
    primitives: Vec<Primitive>,
    view: wgpu::Buffer,
    lighting: wgpu::Buffer,
    instances: wgpu::Buffer,
    capacity: usize,
    count: u32,
    staging: Vec<u8>,
    ranges: Vec<std::ops::Range<u32>>,
}
fn fail(s: &str) -> BackendError {
    BackendError::Device(s.into())
}

/// Shared frame bind group: view, lighting, instances and the environment
/// field. Rebuilt when the environment texel buffer is reallocated, because a
/// bind group captures the buffers it was created with.
fn frame_bind_group(
    device: &wgpu::Device,
    pipeline: &wgpu::RenderPipeline,
    view: &wgpu::Buffer,
    lighting: &wgpu::Buffer,
    instances: &wgpu::Buffer,
    environment: &crate::environment::EnvironmentBinding,
) -> wgpu::BindGroup {
    let [environment_meta, environment_texels] = environment.entries();
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("mesh-frame"),
        layout: &pipeline.get_bind_group_layout(0),
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: view.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: lighting.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: instances.as_entire_binding(),
            },
            environment_meta,
            environment_texels,
        ],
    })
}
fn words(values: impl IntoIterator<Item = f32>) -> Vec<u8> {
    values.into_iter().flat_map(f32::to_le_bytes).collect()
}

pub(crate) fn material_texture(
    backend: &Backend,
    source: &MeshAsset,
    id: Option<usize>,
    fallback: [u8; 4],
    srgb: bool,
    cache: &mut std::collections::BTreeMap<(Option<usize>, [u8; 4], bool), wgpu::TextureView>,
) -> (wgpu::TextureView, wgpu::Sampler) {
    let t = id.map(|i| source.textures[i]);
    let image = t.map(|t| t.image);
    let texture = cache
        .entry((image, if image.is_some() { [0; 4] } else { fallback }, srgb))
        .or_insert_with(|| {
            let (width, height, pixels) = image.map_or((1, 1, fallback.as_slice()), |i| {
                let i = &source.images[i];
                (i.width, i.height, i.rgba8.as_slice())
            });
            let texture = backend.device.create_texture(&wgpu::TextureDescriptor {
                label: Some("mesh-material-texture"),
                size: wgpu::Extent3d {
                    width,
                    height,
                    depth_or_array_layers: 1,
                },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: if srgb {
                    wgpu::TextureFormat::Rgba8UnormSrgb
                } else {
                    wgpu::TextureFormat::Rgba8Unorm
                },
                usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
                view_formats: &[],
            });
            backend.queue.write_texture(
                texture.as_image_copy(),
                pixels,
                wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(width * 4),
                    rows_per_image: Some(height),
                },
                texture.size(),
            );
            texture.create_view(&Default::default())
        })
        .clone();
    let wrap = |w| match w {
        Wrap::Clamp => wgpu::AddressMode::ClampToEdge,
        Wrap::Mirror => wgpu::AddressMode::MirrorRepeat,
        Wrap::Repeat => wgpu::AddressMode::Repeat,
    };
    let sampler = backend.device.create_sampler(&wgpu::SamplerDescriptor {
        address_mode_u: wrap(t.map_or(Wrap::Repeat, |t| t.wrap[0])),
        address_mode_v: wrap(t.map_or(Wrap::Repeat, |t| t.wrap[1])),
        mag_filter: if t.is_none_or(|t| t.linear) {
            wgpu::FilterMode::Linear
        } else {
            wgpu::FilterMode::Nearest
        },
        min_filter: if t.is_none_or(|t| t.min_linear) {
            wgpu::FilterMode::Linear
        } else {
            wgpu::FilterMode::Nearest
        },
        ..Default::default()
    });
    (texture, sampler)
}
impl MeshRenderer {
    /// Ray-tracing accounting for telemetry; inactive when RT is off or was
    /// rejected, with `rt_reason` explaining a rejection.
    pub fn rt_stats(&self) -> crate::ray_scene::RayStats {
        self.rt.as_ref().map_or_else(
            crate::ray_scene::RayStats::default,
            crate::ray_scene::RayScene::stats,
        )
    }

    pub fn rt_reason(&self) -> Option<&str> {
        self.rt_reason.as_deref()
    }

    pub fn rt_terrain_reason(&self) -> Option<&str> {
        self.rt
            .as_ref()
            .and_then(crate::ray_scene::RayScene::terrain_reason)
    }

    /// Publish terrain membership after mesh updates, before encoding. Height
    /// generation submissions precede these builds on the same queue.
    pub fn sync_rt_terrain(
        &mut self,
        backend: &Backend,
        terrain: &crate::terrain::TerrainRenderer,
    ) -> Result<(), BackendError> {
        if let Some(rt) = &mut self.rt {
            rt.sync_terrain(backend, terrain)?;
        }
        Ok(())
    }

    /// Reuse unchanged terrain BLASes when only the immutable mesh asset was
    /// replaced (for example, a new fleet appearance epoch).
    pub fn adopt_rt_terrain(&mut self, previous: &mut Self) -> bool {
        match (&mut self.rt, &mut previous.rt) {
            (Some(next), Some(old)) => next.adopt_terrain(old),
            _ => false,
        }
    }

    /// Bind the radiance environment field, or `None` for the disabled state
    /// that reproduces the legacy isotropic `ambient_lux / PI` reflection
    /// term exactly. No pipeline or layout rebuild; buffer growth/release
    /// rebuilds the frame bind group. Disabled retains only a 16-byte texel
    /// placeholder and the metadata uniform.
    pub fn update_environment(
        &mut self,
        backend: &Backend,
        field: Option<&thessa_render_core::environment::EnvironmentRadiance>,
    ) {
        if self.environment.update(backend, field) {
            self.global = frame_bind_group(
                &backend.device,
                &self.pipeline,
                &self.view,
                &self.lighting,
                &self.instances,
                &self.environment,
            );
        }
    }

    /// Adopt shared scene buffers, without staging or duplicating the grid.
    /// Call after publication (and on renderer replacement), not every frame.
    pub fn share_environment(
        &mut self,
        backend: &Backend,
        binding: &crate::environment::EnvironmentBinding,
    ) {
        self.environment = binding.clone();
        self.global = frame_bind_group(
            &backend.device,
            &self.pipeline,
            &self.view,
            &self.lighting,
            &self.instances,
            &self.environment,
        );
    }

    /// Bound environment field: `(enabled, width, height, storage_bytes, uploads)`.
    pub fn environment_stats(&self) -> (bool, u32, u32, u64, u64) {
        let (width, height) = self.environment.dimensions();
        (
            self.environment.enabled(),
            width,
            height,
            self.environment.storage_bytes(),
            self.environment.uploads(),
        )
    }

    /// Cos-weighted hemispherical irradiance of the bound field, in lux;
    /// zero while disabled.
    pub fn environment_ambient_lux(&self) -> [f32; 3] {
        self.environment.ambient_lux()
    }

    pub fn draw_stats(&self) -> MeshDrawStats {
        MeshDrawStats {
            instances: self.count,
            draw_calls: self.ranges.iter().filter(|r| !r.is_empty()).count(),
            triangles: self
                .primitives
                .iter()
                .zip(&self.ranges)
                .map(|(p, range)| u64::from(p.count) / 3 * u64::from(range.end - range.start))
                .sum::<u64>(),
            frame_upload_bytes: 464 + 112 + self.count as usize * 64,
        }
    }
    pub fn new(
        backend: &Backend,
        source: &MeshAsset,
        format: wgpu::TextureFormat,
        capacity: usize,
    ) -> Result<Self, BackendError> {
        Self::with_surface(
            backend,
            source,
            SurfaceSettings {
                format,
                capacity,
                quality: thessa_graphics::Quality::Low,
                reflections: false,
                shadows: None,
                ray_tracing: None,
            },
        )
    }

    pub fn with_surface(
        backend: &Backend,
        source: &MeshAsset,
        settings: SurfaceSettings<'_>,
    ) -> Result<Self, BackendError> {
        Self::with_surface_and_terrain(backend, source, settings, None)
    }

    pub fn with_surface_and_terrain(
        backend: &Backend,
        source: &MeshAsset,
        settings: SurfaceSettings<'_>,
        terrain: Option<&crate::terrain_ray::TerrainRaySource>,
    ) -> Result<Self, BackendError> {
        let SurfaceSettings {
            format,
            capacity,
            quality,
            reflections,
            shadows,
            ray_tracing,
        } = settings;
        source.validate().map_err(fail)?;
        // Feasibility (device feature, scene sizes, RT memory budget) is
        // decided before the pipeline exists, because the shader include
        // depends on it.
        let rt = crate::ray_scene::plan_with_terrain(
            backend,
            source,
            ray_tracing,
            capacity,
            quality,
            reflections,
            terrain,
        );
        let rt_reason = match &rt {
            crate::ray_scene::Enable::Unavailable(reason) => Some(reason.clone()),
            _ => None,
        };
        let device = &backend.device;
        if capacity == 0
            || capacity > 65536
            || capacity as u64 * 64 > device.limits().max_storage_buffer_binding_size
            || source.images.iter().any(|i| {
                i.width > device.limits().max_texture_dimension_2d
                    || i.height > device.limits().max_texture_dimension_2d
            })
            || source.primitives.iter().any(|p| {
                p.vertices.len() as u64 * 32 > device.limits().max_buffer_size
                    || p.indices.len() as u64 * 4 > device.limits().max_buffer_size
                    || p.indices.len() > u32::MAX as usize
            })
        {
            return Err(fail("mesh capacity/source exceeds device limits"));
        }
        let buffer = |data: &[u8], usage| {
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("mesh-source"),
                contents: data,
                usage,
            })
        };
        let view = buffer(
            &[0; 464],
            wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        );
        let lighting = buffer(
            &[0; 112],
            wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        );
        let instances = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("rigid-instances"),
            size: capacity as u64 * 64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let rt_include = match &rt {
            crate::ray_scene::Enable::Ready(plan) => crate::ray_scene::shader(plan),
            _ => crate::ray_scene::fallback_shader(),
        };
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("owned-mesh"),
            source: wgpu::ShaderSource::Wgsl(
                format!(
                    "{}\n{}\n{}\n{}\n{}",
                    rt_include,
                    thessa_rcbt_core::shaders::OCEAN_WGSL,
                    include_str!("environment.wgsl"),
                    shadows.map_or_else(
                        || crate::shadows::NO_SHADOWS.into(),
                        |_| crate::shadows::receiver_shader(2)
                    ),
                    include_str!("mesh.wgsl")
                )
                .into(),
            ),
        });
        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("owned-mesh"),
            layout: None,
            vertex: wgpu::VertexState {
                module: &module,
                entry_point: Some("vertex"),
                compilation_options: Default::default(),
                buffers: &[Some(wgpu::VertexBufferLayout {
                    array_stride: 32,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &wgpu::vertex_attr_array![0=>Float32x3,1=>Float32x3,2=>Float32x2],
                })],
            },
            primitive: wgpu::PrimitiveState {
                cull_mode: None,
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: Some(true),
                depth_compare: Some(wgpu::CompareFunction::Less),
                stencil: Default::default(),
                bias: Default::default(),
            }),
            multisample: Default::default(),
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
            multiview_mask: None,
            cache: None,
        });
        let environment = crate::environment::EnvironmentBinding::new(device);
        let global = frame_bind_group(
            device,
            &pipeline,
            &view,
            &lighting,
            &instances,
            &environment,
        );
        let mut textures = std::collections::BTreeMap::new();
        let mut materials = Vec::new();
        for material in &source.materials {
            let (texture, sampler) = material_texture(
                backend,
                source,
                material.base_texture,
                [255; 4],
                true,
                &mut textures,
            );
            let use_maps = quality == thessa_graphics::Quality::High;
            let (mr, mr_sampler) = material_texture(
                backend,
                source,
                use_maps
                    .then_some(material.metallic_roughness_texture)
                    .flatten(),
                [255; 4],
                false,
                &mut textures,
            );
            let (normal, normal_sampler) = material_texture(
                backend,
                source,
                use_maps.then_some(material.normal_texture).flatten(),
                [128, 128, 255, 255],
                false,
                &mut textures,
            );
            let (ao, ao_sampler) = material_texture(
                backend,
                source,
                use_maps.then_some(material.occlusion_texture).flatten(),
                [255; 4],
                false,
                &mut textures,
            );
            let mut data = words(material.base_color);
            data.extend(u32::from(material.double_sided).to_le_bytes());
            data.extend(
                (match quality {
                    thessa_graphics::Quality::Low => 0u32,
                    thessa_graphics::Quality::Medium => 1,
                    thessa_graphics::Quality::High => 2,
                })
                .to_le_bytes(),
            );
            data.extend(u32::from(reflections).to_le_bytes());
            data.extend(u32::from(use_maps && material.normal_texture.is_some()).to_le_bytes());
            data.extend(words([
                material.metallic,
                material.roughness,
                material.normal_scale,
                material.occlusion_strength,
            ]));
            let params = buffer(&data, wgpu::BufferUsages::UNIFORM);
            materials.push(device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("mesh-material"),
                layout: &pipeline.get_bind_group_layout(1),
                entries: &[
                    wgpu::BindGroupEntry {
                        binding: 0,
                        resource: params.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 1,
                        resource: wgpu::BindingResource::TextureView(&texture),
                    },
                    wgpu::BindGroupEntry {
                        binding: 2,
                        resource: wgpu::BindingResource::Sampler(&sampler),
                    },
                    wgpu::BindGroupEntry {
                        binding: 3,
                        resource: wgpu::BindingResource::TextureView(&mr),
                    },
                    wgpu::BindGroupEntry {
                        binding: 4,
                        resource: wgpu::BindingResource::Sampler(&mr_sampler),
                    },
                    wgpu::BindGroupEntry {
                        binding: 5,
                        resource: wgpu::BindingResource::TextureView(&normal),
                    },
                    wgpu::BindGroupEntry {
                        binding: 6,
                        resource: wgpu::BindingResource::Sampler(&normal_sampler),
                    },
                    wgpu::BindGroupEntry {
                        binding: 7,
                        resource: wgpu::BindingResource::TextureView(&ao),
                    },
                    wgpu::BindGroupEntry {
                        binding: 8,
                        resource: wgpu::BindingResource::Sampler(&ao_sampler),
                    },
                ],
            }));
        }
        let primitives: Vec<_> = source
            .primitives
            .iter()
            .map(|p| Primitive {
                vertices: buffer(
                    &words(
                        p.vertices
                            .iter()
                            .flat_map(|v| v.position.into_iter().chain(v.normal).chain(v.uv)),
                    ),
                    wgpu::BufferUsages::VERTEX,
                ),
                indices: buffer(
                    &p.indices
                        .iter()
                        .flat_map(|i| i.to_le_bytes())
                        .collect::<Vec<_>>(),
                    wgpu::BufferUsages::INDEX,
                ),
                count: p.indices.len() as u32,
                material: p.material,
            })
            .collect();
        let rt = match rt {
            crate::ray_scene::Enable::Ready(plan) => Some(crate::ray_scene::RayScene::new(
                backend,
                *plan,
                &pipeline.get_bind_group_layout(3),
                source,
                &mut textures,
            )?),
            _ => None,
        };
        Ok(Self {
            shadow_instances: Vec::new(),
            shadow_revision: 0,
            rt,
            rt_reason,
            environment,
            shadow_receiver: shadows
                .map(|s| s.receiver(backend, &pipeline.get_bind_group_layout(2))),
            shadow_caster: shadows.map(|s| {
                backend
                    .device
                    .create_bind_group(&wgpu::BindGroupDescriptor {
                        label: Some("mesh-shadow-source"),
                        layout: &s.mesh_layout,
                        entries: &[wgpu::BindGroupEntry {
                            binding: 0,
                            resource: instances.as_entire_binding(),
                        }],
                    })
            }),
            ranges: vec![0..0; primitives.len()],
            pipeline,
            global,
            materials,
            primitives,
            view,
            lighting,
            instances,
            capacity,
            count: 0,
            staging: Vec::with_capacity(capacity * 64),
        })
    }
    /// Validate the complete frame before any write; errors retain the previous
    /// draw count and data. Empty publication clears instances without a pass.
    pub fn update(
        &mut self,
        backend: &Backend,
        view: &ViewParams,
        lighting: &SurfaceLighting,
        instances: &[MeshInstance],
    ) -> Result<(), BackendError> {
        self.update_selected(backend, view, lighting, instances, None)
    }

    /// Optional per-primitive instance ranges allow heterogeneous sources in
    /// the existing single pass/pipeline. Validate before publishing any bytes.
    pub fn update_selected(
        &mut self,
        backend: &Backend,
        view: &ViewParams,
        lighting: &SurfaceLighting,
        instances: &[MeshInstance],
        ranges: Option<&[std::ops::Range<u32>]>,
    ) -> Result<(), BackendError> {
        if let Some(ranges) = ranges
            && (ranges.len() != self.primitives.len()
                || ranges
                    .iter()
                    .any(|r| r.start > r.end || r.end as usize > instances.len()))
        {
            return Err(fail("invalid mesh primitive instance ranges"));
        }
        if instances.len() > self.capacity {
            return Err(fail("mesh instance capacity exceeded"));
        }
        let block = ViewUniformBlock::new(view).ok_or_else(|| fail("invalid mesh camera"))?;
        let lights = lighting
            .records()
            .ok_or_else(|| fail("invalid mesh lighting"))?;
        self.staging.clear();
        for instance in instances {
            let matrix = instance
                .camera_relative_matrix(view.world_position_m)
                .ok_or_else(|| fail("invalid rigid mesh instance"))?;
            self.staging.extend(
                matrix
                    .to_cols_array()
                    .into_iter()
                    .flat_map(f32::to_le_bytes),
            );
        }
        backend.queue.write_buffer(&self.view, 0, &block.to_bytes());
        let mut light_bytes = [0u8; 112];
        for (out, word) in light_bytes
            .as_chunks_mut::<4>()
            .0
            .iter_mut()
            .zip(lights.into_iter().flatten())
        {
            out.copy_from_slice(&word.to_le_bytes());
        }
        backend.queue.write_buffer(&self.lighting, 0, &light_bytes);
        if !self.staging.is_empty() {
            backend
                .queue
                .write_buffer(&self.instances, 0, &self.staging);
        }
        self.count = instances.len() as u32;
        let mut shadow_changed = false;
        for (i, range) in self.ranges.iter_mut().enumerate() {
            let next = ranges.map_or(0..self.count, |ranges| ranges[i].clone());
            shadow_changed |= *range != next;
            *range = next;
        }
        if self.shadow_caster.is_some() {
            if self.shadow_instances != self.staging {
                self.shadow_instances.clone_from(&self.staging);
                shadow_changed = true;
            }
            if shadow_changed {
                self.shadow_revision = self.shadow_revision.wrapping_add(1);
            }
        }
        if let Some(rt) = &mut self.rt {
            rt.update(
                &backend.queue,
                view.world_position_m,
                instances,
                &self.ranges,
            )?;
        }
        Ok(())
    }

    pub(crate) fn shadow_key(&self) -> Option<(wgpu::Buffer, u64)> {
        self.shadow_caster
            .as_ref()
            .map(|_| (self.instances.clone(), self.shadow_revision))
    }
    /// Encode acceleration-structure builds (TLAS/BLAS) outside the render pass,
    /// before draw_into_pass(). Empty draw sets do no AS work.
    pub fn encode_rt(&mut self, encoder: &mut wgpu::CommandEncoder) {
        if self.count == 0 || self.ranges.iter().all(std::ops::Range::is_empty) {
            return;
        }
        if let Some(rt) = &mut self.rt {
            rt.encode(encoder);
        }
    }

    pub fn encode(
        &mut self,
        encoder: &mut wgpu::CommandEncoder,
        color: &wgpu::TextureView,
        depth: &wgpu::TextureView,
    ) {
        self.encode_profiled(encoder, color, depth, None);
    }

    /// Optional three-mark hardware diagnostic: AS builds then mesh draws.
    /// No query writes when no mesh is drawable. Caller supplies three marks,
    /// completes submission and reads; ordinary encoding allocates no profiler.
    pub fn encode_profiled(
        &mut self,
        encoder: &mut wgpu::CommandEncoder,
        color: &wgpu::TextureView,
        depth: &wgpu::TextureView,
        timing: Option<&crate::timing::GpuIntervals>,
    ) {
        if self.count == 0 || self.ranges.iter().all(std::ops::Range::is_empty) {
            return;
        }
        // Acceleration-structure builds are encoder-level state and must
        // precede the pass that traces them.
        if let Some(t) = timing {
            t.mark(encoder, 0);
        }
        self.encode_rt(encoder);
        if let Some(t) = timing {
            t.mark(encoder, 1);
        }
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("owned-rigid-mesh"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: color,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Load,
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: depth,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Load,
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
            multiview_mask: None,
        });
        self.draw_into_pass(&mut pass);
        drop(pass);
        if let Some(t) = timing {
            t.mark(encoder, 2);
            t.resolve(encoder);
        }
    }

    /// Draw rigid mesh instances directly into an existing compatible render pass.
    /// Caller must first encode_rt() outside that pass. Attachments must match
    /// construction (color format, Depth32Float and single-sample raster).
    pub fn draw_into_pass<'a>(&'a self, pass: &mut wgpu::RenderPass<'a>) {
        if self.count == 0 || self.ranges.iter().all(std::ops::Range::is_empty) {
            return;
        }
        pass.set_pipeline(&self.pipeline);
        pass.set_bind_group(0, &self.global, &[]);
        if let Some(receiver) = &self.shadow_receiver {
            pass.set_bind_group(2, receiver, &[]);
        }
        if let Some(rt) = &self.rt {
            pass.set_bind_group(3, rt.bind_group(), &[]);
        }
        for (p, range) in self.primitives.iter().zip(&self.ranges) {
            if range.is_empty() {
                continue;
            }
            pass.set_bind_group(1, &self.materials[p.material], &[]);
            pass.set_vertex_buffer(0, p.vertices.slice(..));
            pass.set_index_buffer(p.indices.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..p.count, 0, range.clone());
        }
    }

    pub(crate) fn cast_shadow(
        &self,
        pass: &mut wgpu::RenderPass<'_>,
        atlas: &crate::shadows::ShadowAtlas,
    ) {
        let Some(bind) = &self.shadow_caster else {
            return;
        };
        pass.set_pipeline(&atlas.mesh_pipeline);
        pass.set_bind_group(1, bind, &[]);
        for (p, range) in self.primitives.iter().zip(&self.ranges) {
            if range.is_empty() {
                continue;
            }
            pass.set_vertex_buffer(0, p.vertices.slice(..));
            pass.set_index_buffer(p.indices.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..p.count, 0, range.clone());
        }
    }
}
