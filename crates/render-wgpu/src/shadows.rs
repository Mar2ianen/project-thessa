//! Portable shared raster shadow atlas. Existing generated terrain and opaque
//! mesh buffers cast directly: no CPU mesh rebuild, scene copy or readback.
use crate::{Backend, BackendError, mesh::MeshRenderer, terrain::TerrainRenderer};
use thessa_graphics::ResolvedGraphicsSettings;
use thessa_rcbt_core::lighting::SurfaceLighting;
use wgpu::util::DeviceExt;

/// Last actual depth encoding, summed over active layers. Candidate slots count
/// the admitted slot extent (including holes); cache hits report all zeroes.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct ShadowDrawStats {
    pub candidate_slots: u64,
    pub submitted_slots: u64,
    pub draw_calls: u64,
}

#[derive(Clone)]
pub struct ShadowAtlas {
    pub(crate) mesh_pipeline: wgpu::RenderPipeline,
    pub(crate) terrain_pipeline: wgpu::RenderPipeline,
    pub(crate) conforming_pipeline: wgpu::RenderPipeline,
    pub(crate) mesh_layout: wgpu::BindGroupLayout,
    pub(crate) terrain_layout: wgpu::BindGroupLayout,
    pub(crate) conforming_layout: wgpu::BindGroupLayout,
    texture: wgpu::TextureView,
    depth_texture: wgpu::Texture,
    sampler: wgpu::Sampler,
    frame: wgpu::Buffer,
    layers: Vec<(wgpu::TextureView, wgpu::Buffer, wgpu::BindGroup)>,
    radii: Vec<f64>,
    size: u32,
    normal_bias_m: f64,
    filter: u32,
    active: [bool; 3],
    matrices: Option<[[f32; 16]; 12]>,
    depth_revision: u64,
}

pub fn receiver_shader(group: u32) -> String {
    include_str!("shadows.wgsl").replace("SHADOW_GROUP", &group.to_string())
}

pub const NO_SHADOWS: &str =
    "fn shadow_visibility(p:vec3<f32>,n:vec3<f32>,sun:u32)->f32 { return 1.0; }";

impl ShadowAtlas {
    pub fn new(
        backend: &Backend,
        settings: &ResolvedGraphicsSettings,
    ) -> Result<Option<Self>, BackendError> {
        if !settings.shadow_enabled {
            return Ok(None);
        }
        let device = &backend.device;
        let size = settings
            .native_shadow_map_size
            .min(device.limits().max_texture_dimension_2d);
        if size < 64
            || !size.is_power_of_two()
            || settings.shadow_filter_radius > 2
            || !settings.shadow_normal_bias_m.is_finite()
            || !(0.0..=50.0).contains(&settings.shadow_normal_bias_m)
        {
            return Err(BackendError::Device(
                "invalid shadow resolution/filter/bias".into(),
            ));
        }
        let radii = thessa_render_core::shadow::cascade_radii(
            settings.shadow_max_distance_m,
            settings.shadow_cascades,
        )
        .ok_or_else(|| BackendError::Device("invalid shadow cover".into()))?;
        let layer_count = 3 * radii.len() as u32;
        if layer_count > device.limits().max_texture_array_layers {
            return Err(BackendError::Device(
                "shadow atlas exceeds array-layer capability".into(),
            ));
        }
        let depth = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("shared-directional-shadow-atlas"),
            size: wgpu::Extent3d {
                width: size,
                height: size,
                depth_or_array_layers: layer_count,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Depth32Float,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT
                | wgpu::TextureUsages::TEXTURE_BINDING
                | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let matrix_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("shadow-layer"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let storage_layout = |count| {
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("shadow-sources"),
                entries: &(0..count)
                    .map(|binding| wgpu::BindGroupLayoutEntry {
                        binding,
                        visibility: wgpu::ShaderStages::VERTEX,
                        ty: wgpu::BindingType::Buffer {
                            ty: if binding == 3 {
                                wgpu::BufferBindingType::Uniform
                            } else {
                                wgpu::BufferBindingType::Storage { read_only: true }
                            },
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    })
                    .collect::<Vec<_>>(),
            })
        };
        let mesh_layout = storage_layout(1);
        let terrain_layout = storage_layout(4);
        // The CBT caster reads one storage binding: the canonical surface
        // vertex stream. binding 0 is storage here (only binding 3 is uniform).
        let conforming_layout = storage_layout(1);
        let pipeline = |code: String, layout: &wgpu::BindGroupLayout, mesh: bool| {
            let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("shadow-caster"),
                source: wgpu::ShaderSource::Wgsl(code.into()),
            });
            let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: None,
                bind_group_layouts: &[Some(&matrix_layout), Some(layout)],
                immediate_size: 0,
            });
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some("shadow-caster"),
                layout: Some(&layout),
                vertex: wgpu::VertexState {
                    module: &module,
                    entry_point: Some("vertex"),
                    compilation_options: Default::default(),
                    buffers: if mesh {
                        &[Some(wgpu::VertexBufferLayout {
                            array_stride: 32,
                            step_mode: wgpu::VertexStepMode::Vertex,
                            attributes: &wgpu::vertex_attr_array![0=>Float32x3],
                        })]
                    } else {
                        &[]
                    },
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
                    bias: wgpu::DepthBiasState {
                        constant: 1,
                        slope_scale: 1.0,
                        clamp: 0.0,
                    },
                }),
                multisample: Default::default(),
                fragment: None,
                multiview_mask: None,
                cache: None,
            })
        };
        let mesh_pipeline = pipeline(include_str!("shadow_mesh.wgsl").into(), &mesh_layout, true);
        let terrain_pipeline = pipeline(
            format!(
                "{}\n{}\n{}",
                thessa_rcbt_core::shaders::PRECISION_WGSL,
                thessa_rcbt_core::shaders::TILE_FRAME_WGSL,
                include_str!("shadow_terrain.wgsl")
            ),
            &terrain_layout,
            false,
        );
        let conforming_pipeline = pipeline(
            include_str!("shadow_conforming.wgsl").into(),
            &conforming_layout,
            false,
        );
        let layers = (0..layer_count)
            .map(|i| {
                let uniform = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("shadow-layer-matrix"),
                    contents: &[0; 64],
                    usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                });
                let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
                    label: None,
                    layout: &matrix_layout,
                    entries: &[wgpu::BindGroupEntry {
                        binding: 0,
                        resource: uniform.as_entire_binding(),
                    }],
                });
                let view = depth.create_view(&wgpu::TextureViewDescriptor {
                    dimension: Some(wgpu::TextureViewDimension::D2),
                    base_array_layer: i,
                    array_layer_count: Some(1),
                    ..Default::default()
                });
                (view, uniform, bind)
            })
            .collect();
        Ok(Some(Self {
            mesh_pipeline,
            terrain_pipeline,
            conforming_pipeline,
            mesh_layout,
            terrain_layout,
            conforming_layout,
            texture: depth.create_view(&wgpu::TextureViewDescriptor {
                dimension: Some(wgpu::TextureViewDimension::D2Array),
                ..Default::default()
            }),
            depth_texture: depth,
            sampler: device.create_sampler(&wgpu::SamplerDescriptor {
                compare: Some(wgpu::CompareFunction::LessEqual),
                ..Default::default()
            }),
            frame: device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("shadow-frame"),
                contents: &[0; 800],
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            }),
            layers,
            radii,
            size,
            normal_bias_m: settings.shadow_normal_bias_m,
            filter: settings.shadow_filter_radius,
            active: [false; 3],
            matrices: None,
            depth_revision: 0,
        }))
    }
    pub fn storage_bytes(&self) -> u64 {
        u64::from(self.size).pow(2) * self.layers.len() as u64 * 4
    }
    pub fn active_passes(&self) -> usize {
        self.active.iter().filter(|&&a| a).count() * self.radii.len()
    }

    pub(crate) fn depth_revision(&self) -> u64 {
        self.depth_revision
    }
    /// Conservative enclosing sphere for the finite light-view caster volume.
    pub fn caster_distance_m(&self) -> f64 {
        3.0 * self.radii.last().copied().unwrap_or(0.0)
    }
    fn layer_matrix(&self, layer: usize) -> Option<glam::Mat4> {
        self.matrices
            .as_ref()
            .and_then(|m| m.get(layer))
            .map(glam::Mat4::from_cols_array)
    }
    /// Explicit diagnostic readback of all Depth32Float atlas layers, little-
    /// endian raw f32 bytes. Never called by ordinary window/headless frames.
    pub fn read_depth(&self, backend: &Backend) -> Result<Vec<u8>, BackendError> {
        let bytes = u64::from(self.size) * u64::from(self.size) * self.layers.len() as u64 * 4;
        let staging = backend.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("shadow-depth-oracle"),
            size: bytes,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        // Power-of-two atlas size >=64: four-byte rows are already 256-aligned.
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        encoder.copy_texture_to_buffer(
            wgpu::TexelCopyTextureInfo {
                texture: &self.depth_texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::DepthOnly,
            },
            wgpu::TexelCopyBufferInfo {
                buffer: &staging,
                layout: wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(self.size * 4),
                    rows_per_image: Some(self.size),
                },
            },
            wgpu::Extent3d {
                width: self.size,
                height: self.size,
                depth_or_array_layers: self.layers.len() as u32,
            },
        );
        backend.queue.submit([encoder.finish()]);
        crate::terrain::map_buffer(backend, &staging)
    }
    pub(crate) fn receiver(
        &self,
        backend: &Backend,
        layout: &wgpu::BindGroupLayout,
    ) -> wgpu::BindGroup {
        backend
            .device
            .create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("shadow-receiver"),
                layout,
                entries: &[
                    wgpu::BindGroupEntry {
                        binding: 0,
                        resource: self.frame.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 1,
                        resource: wgpu::BindingResource::TextureView(&self.texture),
                    },
                    wgpu::BindGroupEntry {
                        binding: 2,
                        resource: wgpu::BindingResource::Sampler(&self.sampler),
                    },
                ],
            })
    }
    pub fn update(
        &mut self,
        backend: &Backend,
        camera: glam::DVec3,
        lights: &SurfaceLighting,
    ) -> Result<(), BackendError> {
        if lights.records().is_none() || !camera.is_finite() {
            return Err(BackendError::Device("invalid shadow frame".into()));
        }
        let mut matrices = [[0.0f32; 16]; 12];
        let mut active = [false; 3];
        for sun in 0..3 {
            active[sun] = lights.colors_lux[sun][..3].iter().any(|&c| c > 0.0);
            if !active[sun] {
                continue;
            }
            let d = lights.directions[sun];
            let direction = glam::DVec3::new(f64::from(d[0]), f64::from(d[1]), f64::from(d[2]));
            for (i, &radius) in self.radii.iter().enumerate() {
                let matrix = thessa_render_core::shadow::directional_clip(
                    camera,
                    direction,
                    radius,
                    *self.radii.last().unwrap(),
                    self.size,
                )
                .ok_or_else(|| BackendError::Device("invalid shadow projection".into()))?
                .as_mat4();
                matrices[sun * self.radii.len() + i] = matrix.to_cols_array();
            }
        }
        if self.matrices.as_ref() != Some(&matrices) || active != self.active {
            // Fixed shader ABI: 12 mat4s and eight scalars. Stack encoding
            // avoids transient vectors, and cached frames serialize nothing.
            let mut frame_bytes = [0; 800];
            let words = matrices
                .iter()
                .flatten()
                .copied()
                .chain((0..4).map(|i| self.radii.get(i).copied().unwrap_or(0.0) as f32))
                .chain([
                    self.radii.len() as f32,
                    self.normal_bias_m as f32,
                    self.filter as f32,
                    1.0 / self.size as f32,
                ]);
            for (chunk, val) in frame_bytes.as_chunks_mut::<4>().0.iter_mut().zip(words) {
                *chunk = val.to_le_bytes();
            }
            backend.queue.write_buffer(&self.frame, 0, &frame_bytes);
            for (i, (_, uniform, _)) in self.layers.iter().enumerate() {
                backend
                    .queue
                    .write_buffer(uniform, 0, &frame_bytes[i * 64..(i + 1) * 64]);
            }
            self.matrices = Some(matrices);
            self.depth_revision = self.depth_revision.wrapping_add(1);
        }
        self.active = active;
        Ok(())
    }
    pub fn encode(
        &self,
        encoder: &mut wgpu::CommandEncoder,
        terrain: &TerrainRenderer,
        meshes: Option<&MeshRenderer>,
    ) {
        self.encode_casters(encoder, terrain, meshes, true);
    }

    pub(crate) fn encode_casters(
        &self,
        encoder: &mut wgpu::CommandEncoder,
        terrain: &TerrainRenderer,
        meshes: Option<&MeshRenderer>,
        cull: bool,
    ) -> ShadowDrawStats {
        let mut total = ShadowDrawStats::default();
        for (i, (view, _, bind)) in self.layers.iter().enumerate() {
            if !self.active[i / self.radii.len()] {
                continue;
            }
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("shared-shadow-layer"),
                color_attachments: &[],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                ..Default::default()
            });
            pass.set_bind_group(0, bind, &[]);
            let stats = terrain.cast_shadow(
                &mut pass,
                self,
                if cull { self.layer_matrix(i) } else { None },
            );
            total.candidate_slots += stats.candidate_slots;
            total.submitted_slots += stats.submitted_slots;
            total.draw_calls += stats.draw_calls;
            if let Some(meshes) = meshes {
                meshes.cast_shadow(&mut pass, self);
            }
        }
        total
    }

    /// Diagnostic-only CBT caster: draw the canonical per-bisector surface
    /// vertex stream into every active layer. Emission must already be encoded
    /// after topology publication and before this call. Not a production path;
    /// the legacy patch caster remains the only one `encode_shadows` uses.
    pub fn encode_conforming_casters(
        &self,
        backend: &Backend,
        encoder: &mut wgpu::CommandEncoder,
        surface: &crate::conforming_surface::ConformingSurface,
    ) -> ShadowDrawStats {
        let mut total = ShadowDrawStats::default();
        for (i, (view, _, matrix_bind)) in self.layers.iter().enumerate() {
            if !self.active[i / self.radii.len()] {
                continue;
            }
            // Per-layer bind group 0 already wraps that layer's matrix uniform.
            let caster = backend
                .device
                .create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some("cbt-shadow-caster"),
                    layout: &self.conforming_layout,
                    entries: &[wgpu::BindGroupEntry {
                        binding: 0,
                        resource: surface.vertices.as_entire_binding(),
                    }],
                });
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("cbt-shadow-layer"),
                color_attachments: &[],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                ..Default::default()
            });
            pass.set_bind_group(0, matrix_bind, &[]);
            pass.set_pipeline(&self.conforming_pipeline);
            pass.set_bind_group(1, &caster, &[]);
            pass.draw(0..surface.slot_count() * 3, 0..1);
            total.candidate_slots += u64::from(surface.slot_count());
            total.submitted_slots += u64::from(surface.slot_count());
            total.draw_calls += 1;
        }
        total
    }
}
