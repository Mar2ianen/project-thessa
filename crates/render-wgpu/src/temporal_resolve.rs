//! GPU implementation of the backend-neutral temporal-resolve contract.
//!
//! The pass is deliberately stateless. It does not allocate colour history,
//! copy previous depth or alter presentation; those are scene/presentation
//! owners. Inputs must be the opaque pre-overlay scene, current stored depth,
//! motion vectors and a previous resolved colour/depth pair. Output alpha is the
//! accumulated frame age, not the scene's static-surface mask.

use crate::{Backend, BackendError};
use glam::{Mat4, Vec3};
use thessa_render_core::temporal::JitterOffset;
use thessa_render_core::temporal_resolve::{
    DEFAULT_COLOR_SIGMA, DEFAULT_MIN_HISTORY_WEIGHT, HistoryGrid,
};

pub const HISTORY_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba16Float;
pub const HISTORY_BYTES_PER_PIXEL: u32 = 8;

/// Inputs for one resolve dispatch.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ResolveParams {
    /// Inverse of the jittered current projection in current camera-relative space.
    pub current_world_from_clip: Mat4,
    /// Previous unjittered clip-from-world matrix in previous camera-relative space.
    pub previous_clip_from_world: Mat4,
    /// `current_camera_body - previous_camera_body` in the shared body frame.
    pub camera_offset: Vec3,
    pub extent: [u32; 2],
    pub previous_valid: bool,
    pub current_jitter: JitterOffset,
    pub previous_jitter: JitterOffset,
    /// Explicit normalized-depth tolerance derived by the caller.
    pub depth_tolerance: f32,
    pub color_sigma: f32,
    pub min_history_weight: f32,
    pub history_grid: HistoryGrid,
}

impl ResolveParams {
    pub const BYTE_LEN: usize = 192;

    fn to_bytes(self) -> [u8; Self::BYTE_LEN] {
        let mut out = [0u8; Self::BYTE_LEN];
        let mut at = 0;
        for matrix in [self.current_world_from_clip, self.previous_clip_from_world] {
            for word in matrix.to_cols_array() {
                out[at..at + 4].copy_from_slice(&word.to_le_bytes());
                at += 4;
            }
        }
        for word in [
            self.camera_offset.x,
            self.camera_offset.y,
            self.camera_offset.z,
            f32::from(self.previous_valid),
            self.extent[0] as f32,
            self.extent[1] as f32,
            self.current_jitter.pixels[0] as f32,
            self.current_jitter.pixels[1] as f32,
            self.previous_jitter.pixels[0] as f32,
            self.previous_jitter.pixels[1] as f32,
            self.depth_tolerance,
            self.color_sigma,
            self.min_history_weight,
            match self.history_grid {
                HistoryGrid::ResolvedUnjittered => 0.0,
                HistoryGrid::JitteredScene => 1.0,
            },
        ] {
            out[at..at + 4].copy_from_slice(&word.to_le_bytes());
            at += 4;
        }
        debug_assert!(at <= Self::BYTE_LEN);
        out
    }

    fn validate(self) -> Result<(), BackendError> {
        if !self.current_world_from_clip.is_finite()
            || !self.previous_clip_from_world.is_finite()
            || !self.camera_offset.is_finite()
            || self.extent.contains(&0)
            || self.current_jitter.ndc(self.extent).is_none()
            || self.previous_jitter.ndc(self.extent).is_none()
            || !self.depth_tolerance.is_finite()
            || self.depth_tolerance < 0.0
            || !self.color_sigma.is_finite()
            || self.color_sigma < 0.0
            || !self.min_history_weight.is_finite()
            || !(0.0..=1.0).contains(&self.min_history_weight)
        {
            return Err(BackendError::Device(
                "invalid temporal-resolve parameters".into(),
            ));
        }
        Ok(())
    }
}

impl Default for ResolveParams {
    fn default() -> Self {
        Self {
            current_world_from_clip: Mat4::IDENTITY,
            previous_clip_from_world: Mat4::IDENTITY,
            camera_offset: Vec3::ZERO,
            extent: [1, 1],
            previous_valid: false,
            current_jitter: JitterOffset::ZERO,
            previous_jitter: JitterOffset::ZERO,
            depth_tolerance: 1e-4,
            color_sigma: DEFAULT_COLOR_SIGMA,
            min_history_weight: DEFAULT_MIN_HISTORY_WEIGHT,
            history_grid: HistoryGrid::ResolvedUnjittered,
        }
    }
}

pub struct TemporalResolvePass {
    pipeline: wgpu::ComputePipeline,
    layout: wgpu::BindGroupLayout,
    params: wgpu::Buffer,
    sampler: wgpu::Sampler,
}

impl TemporalResolvePass {
    pub fn new(backend: &Backend) -> Self {
        let module = backend
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("temporal-resolve"),
                source: wgpu::ShaderSource::Wgsl(include_str!("temporal_resolve.wgsl").into()),
            });
        let layout = backend
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("temporal-resolve"),
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
                            view_dimension: wgpu::TextureViewDimension::D2,
                            multisampled: false,
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry {
                        binding: 2,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Texture {
                            sample_type: wgpu::TextureSampleType::Depth,
                            view_dimension: wgpu::TextureViewDimension::D2,
                            multisampled: false,
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry {
                        binding: 3,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Texture {
                            sample_type: wgpu::TextureSampleType::Float { filterable: true },
                            view_dimension: wgpu::TextureViewDimension::D2,
                            multisampled: false,
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry {
                        binding: 4,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Texture {
                            sample_type: wgpu::TextureSampleType::Float { filterable: true },
                            view_dimension: wgpu::TextureViewDimension::D2,
                            multisampled: false,
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry {
                        binding: 5,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Texture {
                            sample_type: wgpu::TextureSampleType::Depth,
                            view_dimension: wgpu::TextureViewDimension::D2,
                            multisampled: false,
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry {
                        binding: 6,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry {
                        binding: 7,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::StorageTexture {
                            access: wgpu::StorageTextureAccess::WriteOnly,
                            format: HISTORY_FORMAT,
                            view_dimension: wgpu::TextureViewDimension::D2,
                        },
                        count: None,
                    },
                ],
            });
        let pipeline_layout =
            backend
                .device
                .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                    label: Some("temporal-resolve"),
                    bind_group_layouts: &[Some(&layout)],
                    immediate_size: 0,
                });
        let pipeline = backend
            .device
            .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some("temporal-resolve"),
                layout: Some(&pipeline_layout),
                module: &module,
                entry_point: Some("main"),
                compilation_options: Default::default(),
                cache: None,
            });
        let params = backend.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("temporal-resolve-params"),
            size: ResolveParams::BYTE_LEN as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let sampler = backend.device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("temporal-resolve-history"),
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            address_mode_w: wgpu::AddressMode::ClampToEdge,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            mipmap_filter: wgpu::MipmapFilterMode::Nearest,
            ..Default::default()
        });
        Self {
            pipeline,
            layout,
            params,
            sampler,
        }
    }

    /// Encode one dispatch. `history` and `previous_depth` must describe the
    /// previous submitted frame at the same extent. The caller owns ping-pong,
    /// reset and copy-to-history policy.
    #[allow(clippy::too_many_arguments)]
    pub fn encode(
        &self,
        backend: &Backend,
        encoder: &mut wgpu::CommandEncoder,
        current_color: &wgpu::TextureView,
        current_depth: &wgpu::TextureView,
        motion: &wgpu::TextureView,
        history: &wgpu::TextureView,
        previous_depth: &wgpu::TextureView,
        output: &wgpu::TextureView,
        params: ResolveParams,
    ) -> Result<(), BackendError> {
        params.validate()?;
        backend
            .queue
            .write_buffer(&self.params, 0, &params.to_bytes());
        let bind = backend
            .device
            .create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("temporal-resolve"),
                layout: &self.layout,
                entries: &[
                    wgpu::BindGroupEntry {
                        binding: 0,
                        resource: self.params.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 1,
                        resource: wgpu::BindingResource::TextureView(current_color),
                    },
                    wgpu::BindGroupEntry {
                        binding: 2,
                        resource: wgpu::BindingResource::TextureView(current_depth),
                    },
                    wgpu::BindGroupEntry {
                        binding: 3,
                        resource: wgpu::BindingResource::TextureView(motion),
                    },
                    wgpu::BindGroupEntry {
                        binding: 4,
                        resource: wgpu::BindingResource::TextureView(history),
                    },
                    wgpu::BindGroupEntry {
                        binding: 5,
                        resource: wgpu::BindingResource::TextureView(previous_depth),
                    },
                    wgpu::BindGroupEntry {
                        binding: 6,
                        resource: wgpu::BindingResource::Sampler(&self.sampler),
                    },
                    wgpu::BindGroupEntry {
                        binding: 7,
                        resource: wgpu::BindingResource::TextureView(output),
                    },
                ],
            });
        let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor::default());
        pass.set_pipeline(&self.pipeline);
        pass.set_bind_group(0, &bind, &[]);
        pass.dispatch_workgroups(
            params.extent[0].div_ceil(8),
            params.extent[1].div_ceil(8),
            1,
        );
        Ok(())
    }
}

/// Allocate a resolved/history texture with the storage and readback usages the
/// pass expects. This is a convenience for scene owners and tests; it does not
/// retain the texture or define reset policy.
pub fn history_target(backend: &Backend, extent: [u32; 2]) -> Result<wgpu::Texture, BackendError> {
    let max = backend.device.limits().max_texture_dimension_2d;
    if extent.iter().any(|&n| n == 0 || n > max) {
        return Err(BackendError::Device(
            "invalid temporal-resolve extent".into(),
        ));
    }
    Ok(backend.device.create_texture(&wgpu::TextureDescriptor {
        label: Some("native-temporal-history"),
        size: wgpu::Extent3d {
            width: extent[0],
            height: extent[1],
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: HISTORY_FORMAT,
        usage: wgpu::TextureUsages::STORAGE_BINDING
            | wgpu::TextureUsages::TEXTURE_BINDING
            | wgpu::TextureUsages::COPY_SRC,
        view_formats: &[],
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn uniform_layout_is_stable_and_validation_is_explicit() {
        let bytes = ResolveParams::default().to_bytes();
        assert_eq!(bytes.len(), ResolveParams::BYTE_LEN);
        let word = |at: usize| f32::from_le_bytes(bytes[at..at + 4].try_into().unwrap());
        assert_eq!(word(64), 1.0);
        assert_eq!(word(128), 0.0);
        assert_eq!(word(140), 0.0);
        assert_eq!(word(144), 1.0);
        assert_eq!(word(148), 1.0);
        assert_eq!(word(152), 0.0);
        assert_eq!(word(168), 1e-4);
        assert_eq!(word(176), DEFAULT_MIN_HISTORY_WEIGHT);
        assert_eq!(word(180), 0.0);
        assert!(ResolveParams::default().validate().is_ok());

        let mut invalid = ResolveParams::default();
        invalid.extent[0] = 0;
        assert!(invalid.validate().is_err());
        invalid = ResolveParams::default();
        invalid.current_jitter = JitterOffset::new([0.5001, 0.0]);
        assert!(invalid.validate().is_err());
        invalid = ResolveParams::default();
        invalid.depth_tolerance = f32::NAN;
        assert!(invalid.validate().is_err());
        invalid = ResolveParams::default();
        invalid.color_sigma = -1.0;
        assert!(invalid.validate().is_err());
        invalid = ResolveParams::default();
        invalid.min_history_weight = 1.01;
        assert!(invalid.validate().is_err());
        invalid = ResolveParams::default();
        invalid.camera_offset.x = f32::INFINITY;
        assert!(invalid.validate().is_err());
    }
}
