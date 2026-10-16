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
pub struct MeshRenderer {
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
fn words(values: impl IntoIterator<Item = f32>) -> Vec<u8> {
    values.into_iter().flat_map(f32::to_le_bytes).collect()
}
impl MeshRenderer {
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
        source.validate().map_err(fail)?;
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
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("owned-mesh"),
            source: wgpu::ShaderSource::Wgsl(include_str!("mesh.wgsl").into()),
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
        let global = device.create_bind_group(&wgpu::BindGroupDescriptor {
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
            ],
        });
        let mut textures = std::collections::BTreeMap::new();
        let mut materials = Vec::new();
        for material in &source.materials {
            let (image_id, width, height, pixels) = if let Some(t) = material.base_texture {
                let i = &source.images[source.textures[t].image];
                (
                    Some(source.textures[t].image),
                    i.width,
                    i.height,
                    i.rgba8.as_slice(),
                )
            } else {
                (None, 1, 1, [255u8; 4].as_slice())
            };
            let texture = textures.entry(image_id).or_insert_with(|| {
                let texture = device.create_texture(&wgpu::TextureDescriptor {
                    label: Some("mesh-albedo-srgb"),
                    size: wgpu::Extent3d {
                        width,
                        height,
                        depth_or_array_layers: 1,
                    },
                    mip_level_count: 1,
                    sample_count: 1,
                    dimension: wgpu::TextureDimension::D2,
                    format: wgpu::TextureFormat::Rgba8UnormSrgb,
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
            });
            let wrap = |w| match w {
                Wrap::Clamp => wgpu::AddressMode::ClampToEdge,
                Wrap::Mirror => wgpu::AddressMode::MirrorRepeat,
                Wrap::Repeat => wgpu::AddressMode::Repeat,
            };
            let t = material.base_texture.map(|i| source.textures[i]);
            let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
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
            let mut data = words(material.base_color);
            data.extend(u32::from(material.double_sided).to_le_bytes());
            data.extend([0; 12]);
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
                        resource: wgpu::BindingResource::TextureView(texture),
                    },
                    wgpu::BindGroupEntry {
                        binding: 2,
                        resource: wgpu::BindingResource::Sampler(&sampler),
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
        Ok(Self {
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
        for (i, range) in self.ranges.iter_mut().enumerate() {
            *range = ranges.map_or(0..self.count, |ranges| ranges[i].clone());
        }
        Ok(())
    }
    pub fn encode(
        &self,
        encoder: &mut wgpu::CommandEncoder,
        color: &wgpu::TextureView,
        depth: &wgpu::TextureView,
    ) {
        if self.count == 0 || self.ranges.iter().all(std::ops::Range::is_empty) {
            return;
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
        pass.set_pipeline(&self.pipeline);
        pass.set_bind_group(0, &self.global, &[]);
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
}
