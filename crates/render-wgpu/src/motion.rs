//! Screen-space motion vectors for static, camera-relative surfaces.
//!
//! This pass deliberately reconstructs from depth instead of triangle
//! identity. It therefore covers planetary/CBT and body-fixed geometry across
//! CBT split/merge as long as the represented surface itself did not move.
//! Displaced ocean, moving vehicles and deforming meshes need an object-motion
//! term (or an explicit invalid mask) and must not be treated as static.
//!
//! Output is `RGBA16Float`: `xy` is `previous_uv - current_uv`, `z` is zero and
//! `w` is `1` for a valid vector or `0` for an invalid/sky sample. The alpha
//! channel avoids overloading a legitimate zero vector with "discard history".
//!
//! Scene-colour alpha is the explicit dynamic-surface mask: static land writes
//! `1`, displaced ocean and moving/deforming meshes write `0`. The pass runs
//! before plume/sky overlays so those are never mistaken for static terrain.

use crate::{Backend, BackendError};
use glam::{Mat4, Vec3};

pub const FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba16Float;
pub const BYTES_PER_PIXEL: u32 = 8;

/// Inputs for one motion-vector encoding.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotionParams {
    /// Inverse of the current **unjittered** clip-from-world matrix, in the
    /// current camera-relative frame.
    pub current_world_from_clip: Mat4,
    /// Previous **unjittered** clip-from-world matrix, in the previous
    /// camera-relative frame.
    pub previous_clip_from_world: Mat4,
    /// `current_camera_body - previous_camera_body` in the shared body frame.
    /// This is a small per-frame offset; do not pass absolute planetary
    /// coordinates through an `f32` uniform.
    pub camera_offset: Vec3,
    pub extent: [u32; 2],
    /// False on first frame, after resize/cut, or without a committed source.
    pub previous_valid: bool,
}

impl MotionParams {
    pub const BYTE_LEN: usize = 160;

    fn to_bytes(self) -> [u8; Self::BYTE_LEN] {
        let mut out = [0u8; Self::BYTE_LEN];
        let mut at = 0;
        for m in [self.current_world_from_clip, self.previous_clip_from_world] {
            for word in m.to_cols_array() {
                out[at..at + 4].copy_from_slice(&word.to_le_bytes());
                at += 4;
            }
        }
        for word in [
            self.camera_offset.x,
            self.camera_offset.y,
            self.camera_offset.z,
            0.0,
            self.extent[0] as f32,
            self.extent[1] as f32,
            f32::from(self.previous_valid),
            0.0,
        ] {
            out[at..at + 4].copy_from_slice(&word.to_le_bytes());
            at += 4;
        }
        debug_assert_eq!(at, Self::BYTE_LEN);
        out
    }

    fn validate(self) -> Result<(), BackendError> {
        if !self.current_world_from_clip.is_finite()
            || !self.previous_clip_from_world.is_finite()
            || !self.camera_offset.is_finite()
            || self.extent[0] == 0
            || self.extent[1] == 0
        {
            return Err(BackendError::Device(
                "invalid motion-vector parameters".into(),
            ));
        }
        Ok(())
    }
}

pub struct MotionVectorPass {
    pipeline: wgpu::ComputePipeline,
    layout: wgpu::BindGroupLayout,
    params: wgpu::Buffer,
}

impl MotionVectorPass {
    pub fn new(backend: &Backend) -> Self {
        let module = backend
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("temporal-motion-vectors"),
                source: wgpu::ShaderSource::Wgsl(include_str!("motion_vectors.wgsl").into()),
            });
        let layout = backend
            .device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("temporal-motion-vectors"),
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
                            sample_type: wgpu::TextureSampleType::Depth,
                            view_dimension: wgpu::TextureViewDimension::D2,
                            multisampled: false,
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry {
                        binding: 2,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Texture {
                            sample_type: wgpu::TextureSampleType::Float { filterable: false },
                            view_dimension: wgpu::TextureViewDimension::D2,
                            multisampled: false,
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry {
                        binding: 3,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::StorageTexture {
                            access: wgpu::StorageTextureAccess::WriteOnly,
                            format: FORMAT,
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
                    label: Some("temporal-motion-vectors"),
                    bind_group_layouts: &[Some(&layout)],
                    immediate_size: 0,
                });
        let pipeline = backend
            .device
            .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some("temporal-motion-vectors"),
                layout: Some(&pipeline_layout),
                module: &module,
                entry_point: Some("main"),
                compilation_options: Default::default(),
                cache: None,
            });
        let params = backend.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("temporal-motion-params"),
            size: MotionParams::BYTE_LEN as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        Self {
            pipeline,
            layout,
            params,
        }
    }

    /// Encode one dispatch. `depth` must be the current frame's stored depth and
    /// `scene_color` the matching pre-overlay scene colour (alpha is the
    /// static-surface mask). `output` must be `FORMAT` with
    /// `STORAGE_BINDING | COPY_SRC` usage. The caller owns lifetime and history;
    /// this pass keeps none.
    pub fn encode(
        &self,
        backend: &Backend,
        encoder: &mut wgpu::CommandEncoder,
        depth: &wgpu::TextureView,
        scene_color: &wgpu::TextureView,
        output: &wgpu::TextureView,
        params: MotionParams,
    ) -> Result<(), BackendError> {
        params.validate()?;
        backend
            .queue
            .write_buffer(&self.params, 0, &params.to_bytes());
        let bind = backend
            .device
            .create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("temporal-motion-vectors"),
                layout: &self.layout,
                entries: &[
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
                        resource: wgpu::BindingResource::TextureView(scene_color),
                    },
                    wgpu::BindGroupEntry {
                        binding: 3,
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

/// Optional scene-owned diagnostics. Stores one *submitted* camera, not colour
/// history. Encoding never advances the previous-frame state; call `commit`
/// only after submitting the real frame, never after a generated/repeated one.
pub struct SceneMotion {
    pass: MotionVectorPass,
    output: wgpu::Texture,
    view: wgpu::TextureView,
    extent: [u32; 2],
    previous: Option<crate::view::ViewParams>,
}

impl SceneMotion {
    pub fn new(
        backend: &Backend,
        settings: &thessa_graphics::ResolvedGraphicsSettings,
        extent: [u32; 2],
    ) -> Result<Option<Self>, BackendError> {
        if !settings.temporal_motion_vectors {
            return Ok(None);
        }
        if !settings.hdr {
            return Err(BackendError::Device(
                "resolve motion settings with HDR first".into(),
            ));
        }
        let output = motion_target(backend, extent)?;
        let view = output.create_view(&Default::default());
        Ok(Some(Self {
            pass: MotionVectorPass::new(backend),
            output,
            view,
            extent,
            previous: None,
        }))
    }

    pub fn target_bytes(&self) -> u64 {
        u64::from(self.extent[0]) * u64::from(self.extent[1]) * u64::from(BYTES_PER_PIXEL)
    }

    /// Diagnostic/adapter access; no host readback occurs in the normal loop.
    pub fn texture(&self) -> &wgpu::Texture {
        &self.output
    }

    pub fn reset(&mut self) {
        self.previous = None;
    }

    /// Invalid extents preserve live resources. Even an unchanged valid extent
    /// resets previous motion (surface reconfiguration may hide a camera cut).
    pub fn resize(&mut self, backend: &Backend, extent: [u32; 2]) -> Result<(), BackendError> {
        if extent != self.extent {
            let output = motion_target(backend, extent)?;
            self.view = output.create_view(&Default::default());
            self.output = output;
            self.extent = extent;
        }
        self.reset();
        Ok(())
    }

    /// The caller must encode stored opaque depth/colour first and translucent
    /// overlays afterwards. Matrices and origin deltas are derived from f64
    /// body-fixed views; no absolute planetary f32 translation is uploaded.
    pub fn encode(
        &self,
        backend: &Backend,
        encoder: &mut wgpu::CommandEncoder,
        depth: &wgpu::TextureView,
        scene_color: &wgpu::TextureView,
        current: &crate::view::ViewParams,
    ) -> Result<(), BackendError> {
        let block = crate::view::ViewUniformBlock::new(current)
            .ok_or_else(|| BackendError::Device("invalid current motion view".into()))?;
        let previous = self.previous.as_ref().unwrap_or(current);
        let previous_block = crate::view::ViewUniformBlock::new(previous)
            .ok_or_else(|| BackendError::Device("invalid previous motion view".into()))?;
        self.pass.encode(
            backend,
            encoder,
            depth,
            scene_color,
            &self.view,
            MotionParams {
                current_world_from_clip: block.world_from_clip,
                previous_clip_from_world: previous_block.unjittered_clip_from_world,
                camera_offset: (current.world_position_m - previous.world_position_m).as_vec3(),
                extent: self.extent,
                previous_valid: self.previous.is_some()
                    && previous.clip_from_view == current.clip_from_view
                    && previous.exposure == current.exposure,
            },
        )
    }

    /// Commit only a successfully submitted primary-visibility frame.
    pub fn commit(&mut self, view: crate::view::ViewParams) {
        self.previous = Some(view);
    }
}

fn motion_target(backend: &Backend, extent: [u32; 2]) -> Result<wgpu::Texture, BackendError> {
    let max = backend.device.limits().max_texture_dimension_2d;
    if extent.iter().any(|&n| n == 0 || n > max) {
        return Err(BackendError::Device("invalid motion target extent".into()));
    }
    Ok(backend.device.create_texture(&wgpu::TextureDescriptor {
        label: Some("native-scene-motion"),
        size: wgpu::Extent3d {
            width: extent[0],
            height: extent[1],
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: FORMAT,
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
    fn shader_needs_no_native_capabilities() {
        let source = include_str!("motion_vectors.wgsl");
        let module = naga::front::wgsl::parse_str(source)
            .unwrap_or_else(|e| panic!("{}", e.emit_to_string(source)));
        naga::valid::Validator::new(
            naga::valid::ValidationFlags::all(),
            naga::valid::Capabilities::empty(),
        )
        .validate(&module)
        .unwrap();
    }

    fn params() -> MotionParams {
        MotionParams {
            current_world_from_clip: Mat4::from_translation(Vec3::new(1.0, 2.0, 3.0)),
            previous_clip_from_world: Mat4::from_scale(Vec3::splat(2.0)),
            camera_offset: Vec3::new(-0.25, 0.5, -0.75),
            extent: [1920, 1080],
            previous_valid: true,
        }
    }

    #[test]
    fn uniform_layout_and_validation_are_stable() {
        let bytes = params().to_bytes();
        let word = |at: usize| f32::from_le_bytes(bytes[at..at + 4].try_into().unwrap());
        assert_eq!(bytes.len(), MotionParams::BYTE_LEN);
        assert_eq!(word(48), 1.0);
        assert_eq!(word(52), 2.0);
        assert_eq!(word(56), 3.0);
        assert_eq!(word(64), 2.0);
        assert_eq!(word(128), -0.25);
        assert_eq!(word(132), 0.5);
        assert_eq!(word(136), -0.75);
        assert_eq!(word(144), 1920.0);
        assert_eq!(word(148), 1080.0);
        assert_eq!(word(152), 1.0);
        assert!(params().validate().is_ok());

        let mut invalid = params();
        invalid.extent[0] = 0;
        assert!(invalid.validate().is_err());
        invalid = params();
        invalid.current_world_from_clip.x_axis.x = f32::NAN;
        assert!(invalid.validate().is_err());
    }
}
