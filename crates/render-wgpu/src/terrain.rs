//! Owned CBT geometry → visibility/LOD/compaction → indirect raster passes.
//!
//! Uses the production portable WGSL, packed HeightPage and TileAnchor ABI;
//! no Bevy dependency or CPU mesh construction. This first consumer supports
//! a body-fixed camera, streamed material pages and a fallback globe color,
//! not the full game scene.

use thessa_rcbt_core::{HeightPage, shaders, tile_anchor::TileAnchor};
use wgpu::util::DeviceExt;

use crate::{
    Backend, BackendError,
    view::{ViewParams, ViewUniformBlock},
};

pub const VERTICES_PER_PATCH: u32 = shaders::VERTICES_PER_PATCH as u32;
const TRIANGLES_PER_PATCH: u64 = shaders::TRIANGLES_PER_PATCH as u64;

/// Borrowed source pages. Quantized heights are uploaded without expansion.
pub struct TerrainPatch<'a> {
    pub anchor: TileAnchor,
    pub page: &'a HeightPage,
}

/// Persistent GPU terrain working set. Geometry is generated once at upload;
/// camera-only frames update anchors/view and rerun classification, not geometry.
pub struct TerrainRenderer {
    lighting: wgpu::Buffer,
    materials: crate::material::MaterialArray,
    node_ids: Vec<u64>,
    anchors: Vec<TileAnchor>,
    frames: wgpu::Buffer,
    view: wgpu::Buffer,
    vertices: wgpu::Buffer,
    draw: wgpu::Buffer,
    geometry: wgpu::ComputePipeline,
    geometry_bind: wgpu::BindGroup,
    params: wgpu::Buffer,
    dirty: wgpu::Buffer,
    metadata: wgpu::Buffer,
    residuals: wgpu::Buffer,
    page_slots: Vec<(u64, u64)>,
    radius: f32,
    classify: [wgpu::ComputePipeline; 3],
    classify_bind: wgpu::BindGroup,
    raster: wgpu::RenderPipeline,
    raster_bind: wgpu::BindGroup,
}

fn bytes(words: impl IntoIterator<Item = u32>) -> Vec<u8> {
    words.into_iter().flat_map(u32::to_le_bytes).collect()
}

fn upload(
    device: &wgpu::Device,
    label: &str,
    data: &[u8],
    usage: wgpu::BufferUsages,
) -> wgpu::Buffer {
    device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some(label),
        contents: data,
        usage,
    })
}

fn storage(device: &wgpu::Device, size: u64, extra: wgpu::BufferUsages) -> wgpu::Buffer {
    device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("cbt-working-set"),
        size,
        usage: wgpu::BufferUsages::STORAGE | extra,
        mapped_at_creation: false,
    })
}

fn buffer_layout(
    device: &wgpu::Device,
    count: u32,
    uniforms: &[u32],
    writable: &[u32],
) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("cbt-compute-layout"),
        entries: &(0..count)
            .map(|binding| wgpu::BindGroupLayoutEntry {
                binding,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: if uniforms.contains(&binding) {
                        wgpu::BufferBindingType::Uniform
                    } else {
                        wgpu::BufferBindingType::Storage {
                            read_only: !writable.contains(&binding),
                        }
                    },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            })
            .collect::<Vec<_>>(),
    })
}

fn bind_buffers(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    buffers: &[&wgpu::Buffer],
) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("cbt-compute-bind"),
        layout,
        entries: &buffers
            .iter()
            .enumerate()
            .map(|(binding, buffer)| wgpu::BindGroupEntry {
                binding: binding as u32,
                resource: buffer.as_entire_binding(),
            })
            .collect::<Vec<_>>(),
    })
}

fn pipeline(
    device: &wgpu::Device,
    module: &wgpu::ShaderModule,
    layout: &wgpu::PipelineLayout,
    entry: &str,
) -> wgpu::ComputePipeline {
    device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some(entry),
        layout: Some(layout),
        module,
        entry_point: Some(entry),
        compilation_options: Default::default(),
        cache: None,
    })
}

impl TerrainRenderer {
    /// Construct on an owned backend. All source tiles must belong to the same
    /// body/radius. Bounds are checked before creating any GPU buffers.
    pub fn new(
        backend: &Backend,
        patches: &[TerrainPatch<'_>],
        format: wgpu::TextureFormat,
    ) -> Result<Self, BackendError> {
        Self::with_material_capacity(backend, patches, format, 1)
    }

    /// Allocate a bounded raw material working set. Capacity is explicit rather
    /// than an adapter-dependent hidden allocation; each layer holds eight mips.
    pub fn with_material_capacity(
        backend: &Backend,
        patches: &[TerrainPatch<'_>],
        format: wgpu::TextureFormat,
        material_layers: u32,
    ) -> Result<Self, BackendError> {
        let device = &backend.device;
        let fail = |s: &str| BackendError::Device(s.into());
        let Some(first) = patches.first() else {
            return Err(fail("terrain needs at least one patch"));
        };
        let radius = first.anchor.radius_m;
        let count = patches.len() as u64;
        let limits = device.limits();
        let max_storage = limits
            .max_storage_buffer_binding_size
            .min(limits.max_buffer_size);
        let vertex_bytes = count
            .checked_mul(VERTICES_PER_PATCH as u64 * 32)
            .ok_or_else(|| fail("terrain size overflow"))?;
        let triangle_bytes = count
            .checked_mul(TRIANGLES_PER_PATCH * 16)
            .ok_or_else(|| fail("terrain size overflow"))?;
        let residual_bytes = patches
            .iter()
            .try_fold(0u64, |sum, p| {
                sum.checked_add((p.page.residuals().len() as u64).div_ceil(2) * 4)
            })
            .ok_or_else(|| fail("terrain residual size overflow"))?;
        if count > u32::MAX as u64 / VERTICES_PER_PATCH as u64
            || count > limits.max_compute_workgroups_per_dimension as u64
            || (count * VERTICES_PER_PATCH as u64).div_ceil(64)
                > limits.max_compute_workgroups_per_dimension as u64
            || vertex_bytes > max_storage
            || triangle_bytes > max_storage
            || residual_bytes > max_storage
            || limits.max_storage_buffers_per_shader_stage < 8
            || patches
                .iter()
                .any(|p| p.anchor.radius_m != radius || p.anchor.tile.level > 30)
        {
            return Err(fail(
                "terrain working set exceeds device limits or mixes body radii",
            ));
        }
        let anchors: Vec<_> = patches.iter().map(|p| p.anchor).collect();
        let frames_data = frame_bytes(&anchors, [0.0; 3])?;
        let materials =
            crate::material::MaterialArray::new(backend, material_layers, patches.len())?;
        let mut node_ids = Vec::with_capacity(patches.len());
        let mut residuals = Vec::with_capacity(residual_bytes as usize);
        let mut page_slots = Vec::with_capacity(patches.len());
        let mut metadata = Vec::with_capacity(patches.len() * 4);
        let mut leaves = Vec::with_capacity(patches.len() * 4);
        for (ordinal, p) in patches.iter().enumerate() {
            let key = p.anchor.tile;
            let mut id = 8 + u64::from(key.face);
            for bit in (0..key.level).rev() {
                id = (id << 2) | u64::from(((key.x >> bit) & 1) * 2 + ((key.y >> bit) & 1));
            }
            node_ids.push(id);
            leaves.extend([
                id as u32,
                (id >> 32) as u32,
                3 + 2 * u32::from(key.level),
                ordinal as u32,
            ]);
            metadata.extend([
                p.page.base_height_m().to_bits(),
                p.page.residual_scale_m().to_bits(),
                p.page.grid_size(),
                (residuals.len() / 4) as u32,
            ]);
            page_slots.push((
                residuals.len() as u64,
                (p.page.residuals().len() as u64).div_ceil(2) * 4,
            ));
            residuals.extend(p.page.residual_bytes());
            residuals.resize(residuals.len().div_ceil(4) * 4, 0);
        }
        let ro = wgpu::BufferUsages::STORAGE;
        let leaves = upload(device, "cbt-leaves", &bytes(leaves), ro);
        let metadata = upload(
            device,
            "cbt-pages",
            &bytes(metadata),
            ro | wgpu::BufferUsages::COPY_DST,
        );
        let residuals = upload(
            device,
            "cbt-residuals",
            &residuals,
            ro | wgpu::BufferUsages::COPY_DST,
        );
        let frames = upload(
            device,
            "cbt-tile-frames",
            &frames_data,
            ro | wgpu::BufferUsages::COPY_DST,
        );
        let vertices = storage(device, vertex_bytes, wgpu::BufferUsages::COPY_SRC);
        let patch_records = storage(device, count * 16, wgpu::BufferUsages::empty());
        let params_data = bytes([
            count as u32,
            VERTICES_PER_PATCH,
            (radius as f32).to_bits(),
            count as u32,
        ]);
        let params = upload(
            device,
            "cbt-params",
            &params_data,
            wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        );
        let dirty = upload(
            device,
            "cbt-initial-dirty",
            &bytes(0..count as u32),
            ro | wgpu::BufferUsages::COPY_DST,
        );
        let geometry_layout = buffer_layout(device, 8, &[5], &[1, 4]);
        let geometry_bind = bind_buffers(
            device,
            &geometry_layout,
            &[
                &leaves,
                &patch_records,
                &metadata,
                &residuals,
                &vertices,
                &params,
                &frames,
                &dirty,
            ],
        );
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("cbt-geometry"),
            bind_group_layouts: &[Some(&geometry_layout)],
            immediate_size: 0,
        });
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("cbt-geometry"),
            source: wgpu::ShaderSource::Wgsl(shaders::CBT_GEOMETRY_WGSL.into()),
        });
        let geometry = pipeline(device, &module, &layout, "build_geometry");
        let mut encoder = device.create_command_encoder(&Default::default());
        {
            let mut pass = encoder.begin_compute_pass(&Default::default());
            pass.set_pipeline(&geometry);
            pass.set_bind_group(0, &geometry_bind, &[]);
            pass.dispatch_workgroups((count as u32 * VERTICES_PER_PATCH).div_ceil(64), 1, 1);
        }
        backend.queue.submit([encoder.finish()]);
        let view = upload(
            device,
            "cbt-view",
            &[0; ViewUniformBlock::BYTE_LEN],
            wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        );
        let transform = upload(
            device,
            "cbt-body-rotation",
            &bytes(glam::Mat4::IDENTITY.to_cols_array().map(f32::to_bits)),
            wgpu::BufferUsages::UNIFORM,
        );
        let triangles = storage(device, triangle_bytes, wgpu::BufferUsages::empty());
        let active = storage(device, 16, wgpu::BufferUsages::empty());
        let draw = storage(
            device,
            32,
            wgpu::BufferUsages::INDIRECT | wgpu::BufferUsages::COPY_SRC,
        );
        let history = storage(device, count * 16, wgpu::BufferUsages::empty());
        let classify_layout = buffer_layout(device, 11, &[5, 6, 7], &[2, 3, 4, 10]);
        let classify_bind = bind_buffers(
            device,
            &classify_layout,
            &[
                &metadata, &vertices, &triangles, &active, &draw, &view, &transform, &params,
                &leaves, &frames, &history,
            ],
        );
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("cbt-classify"),
            bind_group_layouts: &[Some(&classify_layout)],
            immediate_size: 0,
        });
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("cbt-classify"),
            source: wgpu::ShaderSource::Wgsl(shaders::CBT_CLASSIFY_WGSL.into()),
        });
        let classify = ["reset_active", "classify_active", "finalize_active"]
            .map(|entry| pipeline(device, &module, &layout, entry));
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("cbt-raster"),
            source: wgpu::ShaderSource::Wgsl(shaders::CBT_RASTER_WGSL.into()),
        });
        let raster = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("cbt-raster"),
            layout: None,
            vertex: wgpu::VertexState {
                module: &module,
                entry_point: Some("vertex"),
                compilation_options: Default::default(),
                buffers: &[],
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
        // Globe asset loading and effect settings remain in the client.
        let globe = solid_texture(backend, [100, 130, 70, 255], false);
        let roughness = solid_texture(backend, [255; 4], false);
        let black = solid_texture(backend, [0; 4], false);
        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            ..Default::default()
        });
        let mut light = [0.0_f32; 56];
        light[..3].fill(0.15);
        light[4..7].copy_from_slice(&[0.8, 0.5, 0.3]);
        light[16..19].fill(2.5);
        let lighting = upload(
            device,
            "cbt-lighting",
            &bytes(light.map(f32::to_bits)),
            wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        );
        let raster_layout = raster.get_bind_group_layout(0);
        let entries = [
            (0, view.as_entire_binding()),
            (1, transform.as_entire_binding()),
            (2, vertices.as_entire_binding()),
            (3, metadata.as_entire_binding()),
            (4, params.as_entire_binding()),
            (5, triangles.as_entire_binding()),
            (6, wgpu::BindingResource::TextureView(&globe)),
            (7, wgpu::BindingResource::Sampler(&sampler)),
            (8, leaves.as_entire_binding()),
            (9, lighting.as_entire_binding()),
            (10, wgpu::BindingResource::TextureView(&roughness)),
            (11, frames.as_entire_binding()),
            (12, wgpu::BindingResource::TextureView(&materials.view)),
            (13, wgpu::BindingResource::Sampler(&materials.sampler)),
            (14, materials.slots.as_entire_binding()),
            (16, materials.directory.as_entire_binding()),
            (17, wgpu::BindingResource::TextureView(&black)),
            (18, wgpu::BindingResource::TextureView(&black)),
        ]
        .into_iter()
        .map(|(binding, resource)| wgpu::BindGroupEntry { binding, resource })
        .collect::<Vec<_>>();
        let raster_bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("cbt-raster-bind"),
            layout: &raster_layout,
            entries: &entries,
        });
        Ok(Self {
            lighting,
            materials,
            node_ids,
            anchors,
            frames,
            view,
            vertices,
            draw,
            geometry,
            geometry_bind,
            params,
            dirty,
            metadata,
            residuals,
            page_slots,
            radius: radius as f32,
            classify,
            classify_bind,
            raster,
            raster_bind,
        })
    }

    /// Publish a complete material snapshot with independent ordered demand.
    /// Stable generations/priority reordering do not upload texture pages.
    pub fn update_materials(
        &mut self,
        backend: &Backend,
        demand: &[u64],
        pages: &crate::material::MaterialPages,
    ) -> Result<crate::material::MaterialUpload, BackendError> {
        self.materials
            .prepare(backend, &self.node_ids, demand, pages)
    }

    /// Publish direct multi-source illumination; effects stay disabled. Only
    /// the shared seven-record lighting prefix changes, with atomic validation.
    pub fn update_lighting(
        &self,
        backend: &Backend,
        lighting: &thessa_rcbt_core::lighting::SurfaceLighting,
    ) -> Result<(), BackendError> {
        let records = lighting
            .records()
            .ok_or_else(|| BackendError::Device("invalid terrain lighting".into()))?;
        backend.queue.write_buffer(
            &self.lighting,
            0,
            &bytes(records.into_iter().flatten().map(f32::to_bits)),
        );
        Ok(())
    }

    /// Publish changed quantized pages into stable slots and regenerate only
    /// their GPU patches. Reuses pipelines/buffers; no CPU mesh or readback.
    /// Updates must have unique valid ordinals and fit the original slot.
    /// All updates are validated before any GPU mutation.
    pub fn update_pages(
        &self,
        backend: &Backend,
        updates: &[(u32, &HeightPage)],
    ) -> Result<(), BackendError> {
        if updates.is_empty() {
            return Ok(());
        }
        let mut ordinals = updates.iter().map(|(i, _)| *i).collect::<Vec<_>>();
        ordinals.sort_unstable();
        if ordinals.windows(2).any(|w| w[0] == w[1])
            || updates.iter().any(|(i, page)| {
                self.page_slots
                    .get(*i as usize)
                    .is_none_or(|(_, capacity)| {
                        (page.residuals().len() as u64).div_ceil(2) * 4 > *capacity
                    })
            })
        {
            return Err(BackendError::Device(
                "invalid/duplicate terrain ordinal or page exceeds stable slot".into(),
            ));
        }
        for (i, page) in updates {
            let (offset, _) = self.page_slots[*i as usize];
            let mut data = page.residual_bytes();
            data.resize(data.len().div_ceil(4) * 4, 0);
            backend.queue.write_buffer(&self.residuals, offset, &data);
            backend.queue.write_buffer(
                &self.metadata,
                u64::from(*i) * 16,
                &bytes([
                    page.base_height_m().to_bits(),
                    page.residual_scale_m().to_bits(),
                    page.grid_size(),
                    (offset / 4) as u32,
                ]),
            );
        }
        backend.queue.write_buffer(&self.dirty, 0, &bytes(ordinals));
        backend.queue.write_buffer(
            &self.params,
            0,
            &bytes([
                self.anchors.len() as u32,
                VERTICES_PER_PATCH,
                self.radius.to_bits(),
                updates.len() as u32,
            ]),
        );
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        {
            let mut pass = encoder.begin_compute_pass(&Default::default());
            pass.set_pipeline(&self.geometry);
            pass.set_bind_group(0, &self.geometry_bind, &[]);
            pass.dispatch_workgroups(
                (updates.len() as u32 * VERTICES_PER_PATCH).div_ceil(64),
                1,
                1,
            );
        }
        backend.queue.submit([encoder.finish()]);
        Ok(())
    }

    /// Update body-fixed camera uniforms. Tile anchors and shader world space
    /// share exactly this f64 camera origin. No terrain regeneration/readback.
    pub fn update_view(&self, backend: &Backend, params: &ViewParams) -> Result<(), BackendError> {
        let block = ViewUniformBlock::new(params)
            .ok_or_else(|| BackendError::Device("invalid camera uniforms".into()))?;
        let frames = frame_bytes(&self.anchors, params.world_position_m.to_array())?;
        backend.queue.write_buffer(&self.view, 0, &block.to_bytes());
        backend.queue.write_buffer(&self.frames, 0, &frames);
        Ok(())
    }

    /// Encode a full GPU-owned visibility/triangle-LOD/indirect raster frame.
    /// Target format must match construction; depth must be Depth32Float.
    pub fn encode(
        &self,
        encoder: &mut wgpu::CommandEncoder,
        color: &wgpu::TextureView,
        depth: &wgpu::TextureView,
    ) {
        {
            let mut pass = encoder.begin_compute_pass(&Default::default());
            pass.set_bind_group(0, &self.classify_bind, &[]);
            for (i, pipeline) in self.classify.iter().enumerate() {
                pass.set_pipeline(pipeline);
                pass.dispatch_workgroups(if i == 1 { self.anchors.len() as u32 } else { 1 }, 1, 1);
            }
        }
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("owned-cbt-frame"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: color,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color {
                        r: 0.015,
                        g: 0.025,
                        b: 0.05,
                        a: 1.0,
                    }),
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: depth,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(1.0),
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
            multiview_mask: None,
        });
        pass.set_pipeline(&self.raster);
        pass.set_bind_group(0, &self.raster_bind, &[]);
        pass.draw_indirect(&self.draw, 0);
    }

    /// Diagnostic only: tile-local xyz/w followed by normal xyz/height.
    pub fn read_vertices(&self, backend: &Backend) -> Result<Vec<f32>, BackendError> {
        Ok(read_buffer(backend, &self.vertices)?
            .as_chunks::<4>()
            .0
            .iter()
            .map(|b| f32::from_le_bytes(*b))
            .collect())
    }

    /// Diagnostic only; never needed to submit an ordinary frame.
    pub fn read_draw(&self, backend: &Backend) -> Result<Vec<u32>, BackendError> {
        Ok(read_buffer(backend, &self.draw)?
            .as_chunks::<4>()
            .0
            .iter()
            .map(|b| u32::from_le_bytes(*b))
            .collect())
    }
}

fn frame_bytes(anchors: &[TileAnchor], camera: [f64; 3]) -> Result<Vec<u8>, BackendError> {
    let mut out = Vec::with_capacity(anchors.len() * 112);
    for anchor in anchors {
        let frame = anchor
            .to_gpu(camera)
            .ok_or_else(|| BackendError::Device("invalid terrain frame".into()))?;
        out.extend(bytes(frame.words().into_iter().flatten().map(f32::to_bits)));
    }
    Ok(out)
}

fn solid_texture(backend: &Backend, rgba: [u8; 4], array: bool) -> wgpu::TextureView {
    let texture = backend.device.create_texture_with_data(
        &backend.queue,
        &wgpu::TextureDescriptor {
            label: Some("cbt-fallback-material"),
            size: wgpu::Extent3d {
                width: 1,
                height: 1,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8Unorm,
            usage: wgpu::TextureUsages::TEXTURE_BINDING,
            view_formats: &[],
        },
        wgpu::util::TextureDataOrder::LayerMajor,
        &rgba,
    );
    texture.create_view(&wgpu::TextureViewDescriptor {
        dimension: Some(if array {
            wgpu::TextureViewDimension::D2Array
        } else {
            wgpu::TextureViewDimension::D2
        }),
        ..Default::default()
    })
}

fn read_buffer(backend: &Backend, source: &wgpu::Buffer) -> Result<Vec<u8>, BackendError> {
    let staging = backend.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("cbt-diagnostic-readback"),
        size: source.size(),
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    encoder.copy_buffer_to_buffer(source, 0, &staging, 0, source.size());
    backend.queue.submit([encoder.finish()]);
    map_buffer(backend, &staging)
}

fn map_buffer(backend: &Backend, staging: &wgpu::Buffer) -> Result<Vec<u8>, BackendError> {
    let (tx, rx) = std::sync::mpsc::channel();
    staging
        .slice(..)
        .map_async(wgpu::MapMode::Read, move |result| {
            let _ = tx.send(result);
        });
    backend
        .device
        .poll(wgpu::PollType::wait_indefinitely())
        .map_err(|e| BackendError::Device(e.to_string()))?;
    rx.recv()
        .map_err(|e| BackendError::Device(e.to_string()))?
        .map_err(|e| BackendError::Device(e.to_string()))?;
    let view = staging
        .slice(..)
        .get_mapped_range()
        .map_err(|e| BackendError::Device(e.to_string()))?;
    let out = view.to_vec();
    drop(view);
    staging.unmap();
    Ok(out)
}

/// Persistent color/depth targets; reuse across frames. Screenshot readback is
/// explicit and is not part of the steady-state rendering benchmark.
pub struct OffscreenTarget {
    pub color: wgpu::TextureView,
    pub depth: wgpu::TextureView,
    texture: wgpu::Texture,
    width: u32,
    height: u32,
}

impl OffscreenTarget {
    pub const FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8UnormSrgb;

    pub fn new(backend: &Backend, width: u32, height: u32) -> Result<Self, BackendError> {
        let max = backend.device.limits().max_texture_dimension_2d;
        if width == 0 || height == 0 || width > max || height > max {
            return Err(BackendError::Device("invalid offscreen extent".into()));
        }
        let make = |format, usage| {
            backend.device.create_texture(&wgpu::TextureDescriptor {
                label: Some("cbt-offscreen"),
                size: wgpu::Extent3d {
                    width,
                    height,
                    depth_or_array_layers: 1,
                },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format,
                usage,
                view_formats: &[],
            })
        };
        let texture = make(
            Self::FORMAT,
            wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
        );
        let depth = make(
            wgpu::TextureFormat::Depth32Float,
            wgpu::TextureUsages::RENDER_ATTACHMENT,
        )
        .create_view(&Default::default());
        let color = texture.create_view(&Default::default());
        Ok(Self {
            color,
            depth,
            texture,
            width,
            height,
        })
    }

    pub fn read_rgba8(&self, backend: &Backend) -> Result<Vec<u8>, BackendError> {
        let stride = (self.width * 4).div_ceil(wgpu::COPY_BYTES_PER_ROW_ALIGNMENT)
            * wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
        let staging = backend.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("cbt-screenshot"),
            size: stride as u64 * self.height as u64,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        encoder.copy_texture_to_buffer(
            self.texture.as_image_copy(),
            wgpu::TexelCopyBufferInfo {
                buffer: &staging,
                layout: wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(stride),
                    rows_per_image: Some(self.height),
                },
            },
            wgpu::Extent3d {
                width: self.width,
                height: self.height,
                depth_or_array_layers: 1,
            },
        );
        backend.queue.submit([encoder.finish()]);
        let padded = map_buffer(backend, &staging)?;
        Ok(padded
            .chunks_exact(stride as usize)
            .flat_map(|row| row[..self.width as usize * 4].iter().copied())
            .collect())
    }
}
