//! Scene-owned history lifecycle around the shared stateless resolve pass.
//! Only a submitted real frame commits history; overlays never enter it.
use crate::{
    Backend, BackendError,
    motion::SceneMotion,
    temporal_resolve::{self, ResolveParams, TemporalResolvePass},
    view::{ViewParams, ViewUniformBlock},
};

/// Publication/cut tokens supplied by the scene; camera-relative re-emission
/// alone must not change them.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SceneHistoryEpoch {
    pub terrain: [u64; 2],
    pub plants: u64,
    pub presentation: u64,
    pub view: u64,
}

pub struct SceneTemporal {
    pass: TemporalResolvePass,
    color: [wgpu::Texture; 2],
    views: [wgpu::TextureView; 2],
    depth: wgpu::Texture,
    depth_view: wgpu::TextureView,
    extent: [u32; 2],
    index: usize,
    epoch: Option<SceneHistoryEpoch>,
    pending: Option<(SceneHistoryEpoch, bool)>,
    submitted: u64,
    reset_frames: u64,
    max_history_age: u32,
    neighborhood_width: u32,
}

impl SceneTemporal {
    pub fn new(
        backend: &Backend,
        settings: &thessa_graphics::ResolvedGraphicsSettings,
        extent: [u32; 2],
    ) -> Result<Option<Self>, BackendError> {
        if !settings.temporal_taa {
            return Ok(None);
        }
        if !settings.hdr || !settings.temporal_motion_vectors || !settings.temporal_jitter {
            return Err(BackendError::Device(
                "resolve TAA with HDR, motion and jitter first".into(),
            ));
        }
        let (color, views, depth, depth_view) = targets(backend, extent)?;
        Ok(Some(Self {
            pass: TemporalResolvePass::new(backend),
            color,
            views,
            depth,
            depth_view,
            extent,
            index: 0,
            epoch: None,
            pending: None,
            submitted: 0,
            reset_frames: 0,
            max_history_age: match settings.temporal_quality {
                thessa_graphics::Quality::Low => 4,
                thessa_graphics::Quality::Medium => 8,
                thessa_graphics::Quality::High => 32,
            },
            neighborhood_width: if settings.temporal_quality == thessa_graphics::Quality::Low {
                2
            } else {
                3
            },
        }))
    }

    pub fn target_bytes(&self) -> u64 {
        u64::from(self.extent[0]) * u64::from(self.extent[1]) * 20
    }
    pub fn submitted_frames(&self) -> u64 {
        self.submitted
    }
    /// Submitted frames with globally invalid history, not pixel rejections.
    pub fn reset_frames(&self) -> u64 {
        self.reset_frames
    }

    pub fn history_age_limit(&self) -> u32 {
        self.max_history_age
    }

    pub fn neighborhood_width(&self) -> u32 {
        self.neighborhood_width
    }

    pub fn resize(&mut self, backend: &Backend, extent: [u32; 2]) -> Result<(), BackendError> {
        let (color, views, depth, depth_view) = targets(backend, extent)?;
        self.color = color;
        self.views = views;
        self.depth = depth;
        self.depth_view = depth_view;
        self.extent = extent;
        self.epoch = None;
        self.pending = None;
        self.submitted = 0;
        self.reset_frames = 0;
        self.index = 0;
        Ok(())
    }

    /// Encode after opaque motion, before plume/sky/UI. Copy back for existing
    /// composition consumers while keeping the unoverlaid output as history.
    #[allow(clippy::too_many_arguments)]
    pub fn encode(
        &mut self,
        backend: &Backend,
        encoder: &mut wgpu::CommandEncoder,
        scene: &wgpu::Texture,
        depth: &wgpu::Texture,
        motion: &SceneMotion,
        view: &ViewParams,
        epoch: SceneHistoryEpoch,
    ) -> Result<(), BackendError> {
        let block = ViewUniformBlock::new_jittered(
            view,
            view.world_position_m,
            motion.jitter(),
            self.extent,
        )
        .ok_or_else(|| BackendError::Device("invalid temporal current view".into()))?;
        let previous = motion.previous_view().unwrap_or(view);
        let previous_block = ViewUniformBlock::new(previous)
            .ok_or_else(|| BackendError::Device("invalid temporal previous view".into()))?;
        let valid = self.epoch == Some(epoch)
            && motion.previous_view().is_some()
            && previous.clip_from_view == view.clip_from_view
            && previous.exposure == view.exposure;
        self.pass.encode(
            backend,
            encoder,
            &scene.create_view(&Default::default()),
            &depth.create_view(&Default::default()),
            motion.texture_view(),
            &self.views[1 - self.index],
            &self.depth_view,
            &self.views[self.index],
            ResolveParams {
                current_world_from_clip: block.world_from_clip,
                previous_clip_from_world: previous_block.unjittered_clip_from_world,
                camera_offset: (view.world_position_m - previous.world_position_m).as_vec3(),
                extent: self.extent,
                previous_valid: valid,
                current_jitter: motion.jitter(),
                previous_jitter: motion.previous_jitter(),
                // shortcut: 16 epsilon in normalized Depth32Float, replace with
                // a measured metric envelope before production TAA acceptance.
                depth_tolerance: 16.0 * f32::EPSILON,
                max_history_age: self.max_history_age,
                neighborhood_width: self.neighborhood_width,
                ..Default::default()
            },
        )?;
        let size = wgpu::Extent3d {
            width: self.extent[0],
            height: self.extent[1],
            depth_or_array_layers: 1,
        };
        encoder.copy_texture_to_texture(
            self.color[self.index].as_image_copy(),
            scene.as_image_copy(),
            size,
        );
        encoder.copy_texture_to_texture(depth.as_image_copy(), self.depth.as_image_copy(), size);
        self.pending = Some((epoch, valid));
        Ok(())
    }

    pub fn commit(&mut self) {
        if let Some((epoch, valid)) = self.pending.take() {
            self.epoch = Some(epoch);
            self.reset_frames += u64::from(!valid);
            self.index = 1 - self.index;
            self.submitted = self.submitted.wrapping_add(1);
        }
    }
}

type Targets = (
    [wgpu::Texture; 2],
    [wgpu::TextureView; 2],
    wgpu::Texture,
    wgpu::TextureView,
);
fn targets(backend: &Backend, extent: [u32; 2]) -> Result<Targets, BackendError> {
    let color = [
        temporal_resolve::history_target(backend, extent)?,
        temporal_resolve::history_target(backend, extent)?,
    ];
    let views = color.each_ref().map(|t| t.create_view(&Default::default()));
    let depth = backend.device.create_texture(&wgpu::TextureDescriptor {
        label: Some("temporal-previous-depth"),
        size: wgpu::Extent3d {
            width: extent[0],
            height: extent[1],
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Depth32Float,
        usage: wgpu::TextureUsages::COPY_DST | wgpu::TextureUsages::TEXTURE_BINDING,
        view_formats: &[],
    });
    let depth_view = depth.create_view(&Default::default());
    Ok((color, views, depth, depth_view))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{display::DisplayTransform, terrain::OffscreenTarget};
    use glam::{DMat4, DVec3};

    #[test]
    #[ignore = "hardware: scene temporal ping-pong, overlay exclusion, submitted-only history, epochs and resize"]
    fn submitted_history_excludes_overlays_and_resets_without_advancing_a_clock() {
        let backend = Backend::bring_up().unwrap();
        let mut request = thessa_graphics::RequestedGraphics::default();
        let off = backend.resolve_graphics(&request);
        assert!(
            SceneTemporal::new(&backend, &off, [0, 0])
                .unwrap()
                .is_none()
        );
        request.temporal.taa = true;
        request.temporal.quality = thessa_graphics::Quality::Low;
        let settings = backend.resolve_graphics(&request);
        assert!(settings.temporal_taa);
        let extent = [4, 4];
        let display =
            DisplayTransform::new(&backend, &settings, OffscreenTarget::FORMAT, 4, 4).unwrap();
        let target = OffscreenTarget::new(&backend, 4, 4).unwrap();
        let mut motion = SceneMotion::new(&backend, &settings, extent)
            .unwrap()
            .unwrap();
        let mut temporal = SceneTemporal::new(&backend, &settings, extent)
            .unwrap()
            .unwrap();
        assert_eq!(temporal.target_bytes(), 16 * 20);
        assert_eq!(temporal.history_age_limit(), 4);
        assert_eq!(temporal.neighborhood_width(), 2);
        let mut view = ViewParams {
            world_from_view: DMat4::IDENTITY,
            world_position_m: DVec3::ZERO,
            clip_from_view: DMat4::IDENTITY,
            exposure: 1.0,
        };
        let clear = |encoder: &mut wgpu::CommandEncoder, color, depth| {
            let _pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: display.scene_color(&target.color),
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(color),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: if depth {
                    Some(wgpu::RenderPassDepthStencilAttachment {
                        view: &target.depth,
                        depth_ops: Some(wgpu::Operations {
                            load: wgpu::LoadOp::Clear(0.5),
                            store: wgpu::StoreOp::Store,
                        }),
                        stencil_ops: None,
                    })
                } else {
                    None
                },
                ..Default::default()
            });
        };
        let color = wgpu::Color {
            r: 0.25,
            g: 0.5,
            b: 0.75,
            a: 1.0,
        };
        for (frame, epoch, age) in [
            (0, 1, 1),
            (1, 1, 2),
            (2, 2, 1),
            (3, 2, 1),
            (4, 2, 1),
            (5, 2, 1),
            (6, 2, 1),
            (7, 2, 1),
            (8, 2, 1),
            // Explicit invalid mask: shown but age zero, then valid again.
            (9, 2, 0),
            (10, 2, 1),
        ] {
            let mut epoch = SceneHistoryEpoch {
                plants: epoch,
                ..Default::default()
            };
            if frame == 3 {
                view.exposure = 2.0;
            }
            if frame >= 4 {
                epoch.presentation = 1;
            }
            if frame >= 5 {
                epoch.view = 1;
            }
            if frame >= 6 {
                epoch.terrain[0] = 1;
            }
            if frame >= 7 {
                epoch.terrain[1] = 1;
            }
            if frame == 8 {
                view.clip_from_view = DMat4::from_scale(DVec3::new(0.9, 0.9, 1.0));
            }
            let frame_color = if frame == 9 {
                wgpu::Color { a: 0.0, ..color }
            } else {
                color
            };
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            clear(&mut encoder, frame_color, true);
            motion
                .encode(
                    &backend,
                    &mut encoder,
                    &target.depth,
                    display.scene_color(&target.color),
                    &view,
                )
                .unwrap();
            temporal
                .encode(
                    &backend,
                    &mut encoder,
                    display.scene_texture().unwrap(),
                    &target.depth_texture,
                    &motion,
                    &view,
                    epoch,
                )
                .unwrap();
            assert_eq!(temporal.submitted_frames(), frame);
            if frame == 0 {
                // Discard an encoded frame: no submission, no history/sample advance.
                drop(encoder);
                assert_eq!(motion.submitted_frames(), 0);
                encoder = backend.device.create_command_encoder(&Default::default());
                clear(&mut encoder, frame_color, true);
                motion
                    .encode(
                        &backend,
                        &mut encoder,
                        &target.depth,
                        display.scene_color(&target.color),
                        &view,
                    )
                    .unwrap();
                temporal
                    .encode(
                        &backend,
                        &mut encoder,
                        display.scene_texture().unwrap(),
                        &target.depth_texture,
                        &motion,
                        &view,
                        epoch,
                    )
                    .unwrap();
            }
            // Fake an opaque red overlay after the history snapshot.
            clear(&mut encoder, wgpu::Color::RED, false);
            backend.queue.submit([encoder.finish()]);
            temporal.commit();
            motion.commit(view);
            let readback = backend.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("temporal-lifecycle-readback"),
                size: 1024,
                usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            encoder.copy_texture_to_buffer(
                temporal.color[1 - temporal.index].as_image_copy(),
                wgpu::TexelCopyBufferInfo {
                    buffer: &readback,
                    layout: wgpu::TexelCopyBufferLayout {
                        offset: 0,
                        bytes_per_row: Some(256),
                        rows_per_image: Some(4),
                    },
                },
                wgpu::Extent3d {
                    width: 4,
                    height: 4,
                    depth_or_array_layers: 1,
                },
            );
            backend.queue.submit([encoder.finish()]);
            let bytes = crate::terrain::map_buffer(&backend, &readback).unwrap();
            for y in 0..4 {
                for x in 0..4 {
                    let p = y * 256 + x * 8;
                    let words = std::array::from_fn::<_, 4, _>(|i| {
                        u16::from_le_bytes([bytes[p + i * 2], bytes[p + i * 2 + 1]])
                    });
                    assert_eq!(
                        words,
                        [
                            0x3400,
                            0x3800,
                            0x3a00,
                            match age {
                                0 => 0x0000,
                                1 => 0x3c00,
                                _ => 0x4000,
                            }
                        ],
                        "history must exclude overlays, mask invalid pixels and reset age"
                    );
                }
            }
        }
        let bytes = temporal.target_bytes();
        assert_eq!(temporal.reset_frames(), 8);
        temporal.commit();
        assert_eq!(temporal.submitted_frames(), 11);
        assert!(temporal.resize(&backend, [0, 4]).is_err());
        assert_eq!(temporal.target_bytes(), bytes);
        temporal.resize(&backend, [8, 4]).unwrap();
        assert_eq!(temporal.submitted_frames(), 0);
        assert!(temporal.epoch.is_none() && temporal.pending.is_none());
        assert_eq!(temporal.target_bytes(), 32 * 20);
    }
}
