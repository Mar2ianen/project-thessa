//! Joint optical transport. Source capacity is explicit and never silently truncated.
use super::{
    Backend, BackendError, DRAW_BYTES, FRAME_BYTES, MAX_CENTERS, MAX_STATIONS, PlumeStats,
    VolumeDraw, bytes, clip_rect, encode_volume, validate_volume,
};
use std::sync::Arc;

use crate::view::ViewUniformBlock;
pub(super) const MAX_FIELDS: usize = super::MAX_JOINT_FIELDS;

pub(super) fn shader_source() -> String {
    // Reuse the single-source station interpolation, shock and swept geometry.
    // Private invocation state selects a source; no mutable storage is shared.
    let source = super::volume_shader_source()
        .replace("@group(1) @binding(0) var<uniform> draw: Draw;",
            "var<private> draw: Draw;\nvar<private> station_base: u32;\nvar<private> center_base: u32;")
        .replace("stations[", "stations[station_base + ")
        .replace("centers[", "centers[center_base + ");
    format!("{source}\n{}", include_str!("plume_multi.wgsl"))
}

pub(super) struct MultiVolume {
    device: Arc<wgpu::Device>,
    pipeline: wgpu::RenderPipeline,
    frame: wgpu::Buffer,
    config: wgpu::Buffer,
    draws: wgpu::Buffer,
    stations: wgpu::Buffer,
    centers: wgpu::Buffer,
    bind: wgpu::BindGroup,
}

impl MultiVolume {
    pub(super) fn new(
        backend: &Backend,
        format: wgpu::TextureFormat,
        global: &wgpu::BindGroupLayout,
    ) -> Self {
        let device = &backend.device;
        let buffer = |label, size, usage| {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size,
                usage: usage | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            })
        };
        let frame = buffer(
            "plume-multi-camera",
            FRAME_BYTES,
            wgpu::BufferUsages::UNIFORM,
        );
        let config = buffer("plume-multi-config", 32, wgpu::BufferUsages::UNIFORM);
        let draws = buffer(
            "plume-multi-sources",
            MAX_FIELDS as u64 * DRAW_BYTES,
            wgpu::BufferUsages::STORAGE,
        );
        let stations = buffer(
            "plume-multi-stations",
            (MAX_FIELDS * MAX_STATIONS * 32) as u64,
            wgpu::BufferUsages::STORAGE,
        );
        let centers = buffer(
            "plume-multi-centers",
            (MAX_FIELDS * MAX_CENTERS * 16) as u64,
            wgpu::BufferUsages::STORAGE,
        );
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("plume-multi"),
            entries: &[super::uniform(0, 32), storage(1), storage(2), storage(3)],
        });
        let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("plume-multi"),
            layout: &layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: config.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: stations.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: centers.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: draws.as_entire_binding(),
                },
            ],
        });
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("joint-plume-volume"),
            source: wgpu::ShaderSource::Wgsl(shader_source().into()),
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("joint-plume-volume"),
            bind_group_layouts: &[Some(global), Some(&layout)],
            immediate_size: 0,
        });
        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("joint-plume-volume"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &module,
                entry_point: Some("multi_vertex"),
                compilation_options: Default::default(),
                buffers: &[],
            },
            primitive: wgpu::PrimitiveState {
                cull_mode: None,
                ..Default::default()
            },
            depth_stencil: None,
            multisample: Default::default(),
            fragment: Some(wgpu::FragmentState {
                module: &module,
                entry_point: Some("multi_fragment"),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format,
                    blend: Some(wgpu::BlendState::PREMULTIPLIED_ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            multiview_mask: None,
            cache: None,
        });
        Self {
            device: device.clone(),
            pipeline,
            frame,
            config,
            draws,
            stations,
            centers,
            bind,
        }
    }

    pub(super) fn update(
        &mut self,
        backend: &Backend,
        params: &super::ViewParams,
        sources: &[VolumeDraw],
    ) -> Result<PlumeStats, BackendError> {
        if sources.len() > MAX_FIELDS {
            return Err(BackendError::Device(format!(
                "{} plume fields exceed joint transport capacity {MAX_FIELDS}",
                sources.len()
            )));
        }
        let view = ViewUniformBlock::new(params)
            .ok_or_else(|| BackendError::Device("invalid joint plume camera".into()))?;
        // Validate the complete transaction before the first queue write.
        for source in sources {
            validate_volume(source)?;
        }
        let visible: Vec<_> = sources
            .iter()
            .filter_map(|draw| clip_rect(&view, draw).map(|rect| (draw, rect)))
            .collect();
        if visible.is_empty() {
            return Ok(PlumeStats::default());
        }
        let mut union = [
            f32::INFINITY,
            f32::INFINITY,
            f32::NEG_INFINITY,
            f32::NEG_INFINITY,
        ];
        let mut draw_bytes = Vec::new();
        let mut profiles = Vec::new();
        let mut centers = Vec::new();
        let mut stats = PlumeStats {
            draw_calls: 1,
            source_count: visible.len() as u32,
            ..Default::default()
        };
        for (draw, rect) in &visible {
            union[0] = union[0].min(rect[0]);
            union[1] = union[1].min(rect[1]);
            union[2] = union[2].max(rect[2]);
            union[3] = union[3].max(rect[3]);
            draw_bytes.extend(encode_volume(draw, *rect));
            profiles.extend(bytes(draw.profile.iter().flatten().copied()));
            profiles.resize(profiles.len() + (MAX_STATIONS - draw.profile.len()) * 32, 0);
            centers.extend(bytes(draw.centerline.iter().flatten().copied()));
            centers.resize(
                centers.len() + (MAX_CENTERS - draw.centerline.len()) * 16,
                0,
            );
            stats.march_steps = stats
                .march_steps
                .max(draw.animation[0].clamp(1., 16.) as u32);
            stats.march_step_limit = 64;
            stats.pixel_ray_limit = stats.pixel_ray_limit.max(draw.pixel_grid * draw.pixel_grid);
            stats.centerline_segments += draw.centerline.len().saturating_sub(1) as u32;
        }
        let camera = bytes(
            view.clip_from_world
                .to_cols_array()
                .into_iter()
                .chain(view.world_from_clip.to_cols_array())
                .chain(view.world_position.extend(0.).to_array())
                .chain([view.exposure, 0., 0., 0.]),
        );
        let config = bytes(union.into_iter().chain([visible.len() as f32, 0., 0., 0.]));
        backend.queue.write_buffer(&self.frame, 0, &camera);
        backend.queue.write_buffer(&self.config, 0, &config);
        backend.queue.write_buffer(&self.draws, 0, &draw_bytes);
        backend.queue.write_buffer(&self.stations, 0, &profiles);
        backend.queue.write_buffer(&self.centers, 0, &centers);
        stats.frame_upload_bytes = (camera.len() + config.len() + draw_bytes.len()) as u64;
        stats.profile_upload_bytes = profiles.len() as u64;
        stats.centerline_upload_bytes = centers.len() as u64;
        Ok(stats)
    }

    pub(super) fn encode(
        &self,
        encoder: &mut wgpu::CommandEncoder,
        color: &wgpu::TextureView,
        depth: &wgpu::TextureView,
        global: &wgpu::BindGroupLayout,
    ) {
        let bind = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("joint-plume-camera"),
            layout: global,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: self.frame.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::TextureView(depth),
                },
            ],
        });
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("joint-plume-depth-clipped"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: color,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Load,
                    store: wgpu::StoreOp::Store,
                },
            })],
            ..Default::default()
        });
        pass.set_pipeline(&self.pipeline);
        pass.set_bind_group(0, &bind, &[]);
        pass.set_bind_group(1, &self.bind, &[]);
        pass.draw(0..6, 0..1);
    }
}

fn storage(binding: u32) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::FRAGMENT,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Storage { read_only: true },
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn joint_shader_is_valid_wgsl() {
        let source = super::shader_source();
        let module = naga::front::wgsl::parse_str(&source)
            .unwrap_or_else(|e| panic!("{}", e.emit_to_string(&source)));
        naga::valid::Validator::new(
            naga::valid::ValidationFlags::all(),
            naga::valid::Capabilities::all(),
        )
        .validate(&module)
        .unwrap();
    }
}
