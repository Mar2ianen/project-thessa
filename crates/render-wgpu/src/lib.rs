//! Thessa-owned wgpu backend bring-up (headless-first).
//!
//! This is the R1 foundation: `Instance → Adapter → Device` without Bevy,
//! a hardware capability probe feeding the existing
//! [`thessa_graphics::Capabilities`] resolver input, and the first
//! frame primitives (clear + readback), plus [`terrain`]'s owned CBT
//! compute/classification/indirect raster frame. Headless bring-up uses
//! `InstanceDescriptor::new_without_display_handle()`; [`Backend::for_surface`]
//! also selects a compatible adapter/device for a platform-owned surface.
//! The native terrain migration viewer owns window lifecycle and presentation;
//! it is not the complete game/platform replacement.
//!
//! Capability semantics (mirror of `resolve_terrain_capabilities`, but
//! hardware-truth instead of post-negotiation):
//!
//! - `is_discrete_gpu`: adapter device type, exactly as the client reads it;
//! - `ray_query_supported`: adapter exposes `EXPERIMENTAL_RAY_QUERY`;
//! - `terrain_mesh_supported`: mesh-shader feature plus the terrain mesh
//!   limits. Thresholds (96 output vertices / 32 primitives / 32
//!   invocations / 7 storage buffers) duplicate
//!   `bevy-rcbt/src/render.rs::{MESHLET_*, mesh_shader_supported}` — the
//!   single source of truth until the terrain mesh path moves off the
//!   bridge, at which point the constants hoist here.
//!
//! Device negotiation: [`Backend::bring_up`] requests stable features only,
//! so no `unsafe` experimental-feature consent lives in the default path
//! (this crate is `forbid(unsafe_code)`). The consent itself is isolated in
//! `thessa-wgpu-exp`; [`Backend::bring_up_experimental`] negotiates exactly
//! the adapter-exposed interesting subset through it. Limits stay default in
//! this step; per-pipeline limit negotiation arrives with the pipeline cache.

#![forbid(unsafe_code)]

use std::sync::Arc;

use thessa_graphics::Capabilities;

pub mod clouds;
mod conforming_ocean;
pub mod conforming_surface;
pub mod display;
pub mod environment;
pub mod material;
pub mod mesh;
pub mod ocean;
pub mod plume;
pub mod ray_scene;
pub mod shadows;
mod sky;
pub mod terrain;
pub mod terrain_ray;
mod terrain_topology;
pub mod timing;
pub mod view;

/// Mesh limits the terrain mesh-shader path needs. Duplicates the bridge
/// (`bevy-rcbt/src/render.rs`); see module docs for the hoist plan.
const MESH_MIN_OUTPUT_VERTICES: u32 = 96;
const MESH_MIN_OUTPUT_PRIMITIVES: u32 = 32;
const MESH_MIN_INVOCATIONS: u32 = 32;
const MESH_MIN_STORAGE_BUFFERS: u32 = 7;

/// Bring-up failure.
#[derive(Debug, Clone, PartialEq)]
pub enum BackendError {
    /// No wgpu adapter (headless CI, no GPU). Callers SKIP on this.
    NoAdapter,
    /// Device/queue/pipeline failure with context.
    Device(String),
}

impl std::fmt::Display for BackendError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoAdapter => write!(f, "thessa-render-wgpu: no adapter"),
            Self::Device(detail) => write!(f, "thessa-render-wgpu device failure: {detail}"),
        }
    }
}

impl std::error::Error for BackendError {}

/// Owned backend: instance (kept alive), adapter, device, queue, and the
/// probed hardware capabilities.
pub struct Backend {
    _instance: wgpu::Instance,
    /// Human-readable adapter name for logs and bench headers.
    pub adapter_name: String,
    pub adapter: wgpu::Adapter,
    pub device: Arc<wgpu::Device>,
    pub queue: Arc<wgpu::Queue>,
    pub caps: Capabilities,
}

/// Device limits for one bring-up. Byte-identical to `Limits::default()`
/// unless ray tracing was negotiated, in which case the acceleration-structure
/// budgets come from the adapter instead of staying at the zero-valued
/// defaults (which would reject any RT pipeline at layout derivation).
fn limits_for(adapter: &wgpu::Adapter, features: wgpu::Features) -> wgpu::Limits {
    let mut limits = wgpu::Limits::default();
    if !features.contains(wgpu::Features::EXPERIMENTAL_RAY_QUERY) {
        return limits;
    }
    let caps = adapter.limits();
    limits.max_acceleration_structures_per_shader_stage =
        caps.max_acceleration_structures_per_shader_stage;
    limits.max_blas_primitive_count = caps.max_blas_primitive_count;
    limits.max_blas_geometry_count = caps.max_blas_geometry_count;
    limits.max_tlas_instance_count = caps.max_tlas_instance_count;
    // Mesh hit tables + shared environment + borrowed terrain geometry and
    // its material directory. Lesser devices retain the mesh-only RT path.
    limits.max_storage_buffers_per_shader_stage = caps
        .max_storage_buffers_per_shader_stage
        .clamp(limits.max_storage_buffers_per_shader_stage, 9);
    limits.max_buffers_and_acceleration_structures_per_shader_stage = limits
        .max_buffers_and_acceleration_structures_per_shader_stage
        .max(caps.max_buffers_and_acceleration_structures_per_shader_stage);
    limits
}

impl Backend {
    /// Headless bring-up on the high-performance adapter. Succeeds anywhere
    /// a wgpu adapter exists; returns [`BackendError::NoAdapter`] otherwise.
    /// Stable features only — see [`Backend::bring_up_experimental`] for the
    /// negotiated path.
    pub fn bring_up() -> Result<Self, BackendError> {
        Self::bring_up_inner(None)
    }

    /// Bring-up negotiating the adapter-exposed experimental subset (mesh
    /// shaders, ray query) through the isolated consent in
    /// `thessa-wgpu-exp`. Hardware truth and device state agree afterwards
    /// by construction; use it for paths that build mesh/RT pipelines.
    pub fn bring_up_experimental() -> Result<Self, BackendError> {
        Self::bring_up_inner(Some(thessa_wgpu_exp::experimental_consent()))
    }

    fn bring_up_inner(consent: Option<wgpu::ExperimentalFeatures>) -> Result<Self, BackendError> {
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
        Self::request(instance, None, consent, None, false)
    }

    /// Stable device selected for a caller-owned window surface. The platform
    /// supplies the instance/display handle; no window toolkit leaks into this
    /// backend. Uses the same capability/device path as headless rendering.
    pub fn for_surface(
        instance: wgpu::Instance,
        surface: &wgpu::Surface<'_>,
    ) -> Result<Self, BackendError> {
        Self::request(instance, Some(surface), None, None, false)
    }

    pub fn bring_up_for_graphics(
        requested: &thessa_graphics::RequestedGraphics,
    ) -> Result<Self, BackendError> {
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
        Self::request(instance, None, None, Some(requested), false)
    }

    /// Opt-in diagnostic device. Missing timestamp capability degrades to
    /// unavailable timings, never an invented CPU-derived GPU duration.
    pub fn bring_up_for_graphics_profiled(
        requested: &thessa_graphics::RequestedGraphics,
    ) -> Result<Self, BackendError> {
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
        Self::request(instance, None, None, Some(requested), true)
    }
    pub fn for_surface_graphics(
        instance: wgpu::Instance,
        surface: &wgpu::Surface<'_>,
        requested: &thessa_graphics::RequestedGraphics,
    ) -> Result<Self, BackendError> {
        Self::request(instance, Some(surface), None, Some(requested), false)
    }
    /// Capability resolution must describe the negotiated device, not just
    /// the adapter's unrequested experimental features.
    pub fn resolve_graphics(
        &self,
        requested: &thessa_graphics::RequestedGraphics,
    ) -> thessa_graphics::ResolvedGraphicsSettings {
        let mut caps = self.caps;
        caps.ray_query_supported = Some(
            self.device
                .features()
                .contains(wgpu::Features::EXPERIMENTAL_RAY_QUERY),
        );
        thessa_graphics::ResolvedGraphicsSettings::from_requested(requested, &caps)
    }

    fn request(
        instance: wgpu::Instance,
        surface: Option<&wgpu::Surface<'_>>,
        consent: Option<wgpu::ExperimentalFeatures>,
        graphics: Option<&thessa_graphics::RequestedGraphics>,
        profiling: bool,
    ) -> Result<Self, BackendError> {
        let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            compatible_surface: surface,
            force_fallback_adapter: false,
            // Headless compute: exact adapter limits, no fingerprint bucketing.
            apply_limit_buckets: false,
        }))
        .map_err(|_| BackendError::NoAdapter)?;
        let info = adapter.get_info();
        let (mut features, mut experimental) = match consent {
            Some(token) => (thessa_wgpu_exp::supported_experimental(&adapter), token),
            // Stable-only: experimental consent is never taken implicitly.
            None => (
                wgpu::Features::empty(),
                wgpu::ExperimentalFeatures::disabled(),
            ),
        };
        let timing = timing::required_features();
        if profiling && adapter.features().contains(timing) {
            features |= timing;
        }
        if let Some(settings) = graphics {
            use thessa_graphics::RayTracingRequest;
            let supported = adapter
                .features()
                .contains(wgpu::Features::EXPERIMENTAL_RAY_QUERY);
            let forced = matches!(
                settings.renderer.ray_tracing,
                RayTracingRequest::Local | RayTracingRequest::Full
            );
            if forced && !supported {
                return Err(BackendError::Device("explicit native RT requested, adapter lacks ray query; use ray_tracing=off or auto".into()));
            }
            if supported
                && settings.renderer.ray_tracing != RayTracingRequest::Off
                && (settings.renderer.specular_reflections || settings.shadows.enabled)
            {
                features |= wgpu::Features::EXPERIMENTAL_RAY_QUERY;
                experimental = thessa_wgpu_exp::experimental_consent();
            }
        }
        let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
            label: Some("thessa-render-backend"),
            required_features: features,
            required_limits: limits_for(&adapter, features),
            experimental_features: experimental,
            memory_hints: wgpu::MemoryHints::Performance,
            trace: wgpu::Trace::Off,
        }))
        .map_err(|e| BackendError::Device(format!("request_device failed: {e:?}")))?;
        let caps = probe_capabilities(&adapter);
        Ok(Self {
            _instance: instance,
            adapter_name: info.name,
            adapter,
            device: Arc::new(device),
            queue: Arc::new(queue),
            caps,
        })
    }

    /// Clear an offscreen RGBA8 target and read it back. Exists to prove the
    /// backend drives the GPU end to end (pass + copy + map) with exact
    /// pixels; the R3 render loop builds on this submission shape.
    pub fn clear_readback(
        &self,
        width: u32,
        height: u32,
        color: [f64; 4],
    ) -> Result<Vec<u8>, BackendError> {
        if width == 0 || height == 0 {
            return Err(BackendError::Device("clear size must be nonzero".into()));
        }
        if width > 4096 || height > 4096 {
            return Err(BackendError::Device("clear size capped at 4096".into()));
        }
        let texture = self.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("thessa-backend-clear"),
            size: wgpu::Extent3d {
                width,
                height,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8UnormSrgb,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
        let row_bytes = width as u64 * 4;
        let row_stride = row_bytes.div_ceil(wgpu::COPY_BYTES_PER_ROW_ALIGNMENT as u64)
            * wgpu::COPY_BYTES_PER_ROW_ALIGNMENT as u64;
        let staging = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("thessa-backend-clear-staging"),
            size: row_stride * height as u64,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("thessa-backend-clear-encoder"),
            });
        {
            let _pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("thessa-backend-clear-pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: color[0],
                            g: color[1],
                            b: color[2],
                            a: color[3],
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
        }
        encoder.copy_texture_to_buffer(
            wgpu::TexelCopyTextureInfo {
                texture: &texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::TexelCopyBufferInfo {
                buffer: &staging,
                layout: wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(row_stride as u32),
                    rows_per_image: Some(height),
                },
            },
            wgpu::Extent3d {
                width,
                height,
                depth_or_array_layers: 1,
            },
        );
        self.queue.submit(Some(encoder.finish()));
        let (tx, rx) = std::sync::mpsc::channel();
        staging
            .slice(..)
            .map_async(wgpu::MapMode::Read, move |result| {
                let _ = tx.send(result);
            });
        self.device
            .poll(wgpu::PollType::wait_indefinitely())
            .map_err(|e| BackendError::Device(format!("device poll failed: {e:?}")))?;
        rx.recv()
            .map_err(|_| BackendError::Device("staging map channel closed".into()))?
            .map_err(|e| BackendError::Device(format!("staging map failed: {e:?}")))?;
        let view = staging
            .slice(..)
            .get_mapped_range()
            .map_err(|e| BackendError::Device(format!("staging map range failed: {e:?}")))?;
        let mut out = Vec::with_capacity((row_bytes * height as u64) as usize);
        for row in view.chunks_exact(row_stride as usize) {
            out.extend_from_slice(&row[..row_bytes as usize]);
        }
        drop(view);
        staging.unmap();
        Ok(out)
    }

    /// Upload bytes to a device buffer and read them back. The future
    /// frame-publish path stages snapshot payloads exactly this way; this
    /// primitive exists so byte parity is testable today, before any render
    /// loop consumes it. Empty input returns empty without touching the GPU;
    /// inputs above 256 MiB are rejected.
    pub fn roundtrip(&self, bytes: &[u8]) -> Result<Vec<u8>, BackendError> {
        if bytes.is_empty() {
            return Ok(Vec::new());
        }
        if bytes.len() as u64 > 1 << 28 {
            return Err(BackendError::Device(
                "roundtrip size capped at 256 MiB".into(),
            ));
        }
        // `write_buffer` demands 4-byte alignment: pad the upload, then
        // truncate the readback to the caller's length.
        let padded = bytes.len().div_ceil(4) * 4;
        let buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("thessa-backend-roundtrip"),
            size: padded as u64,
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_SRC
                | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let staging = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("thessa-backend-roundtrip-staging"),
            size: padded as u64,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let mut upload = Vec::with_capacity(padded);
        upload.extend_from_slice(bytes);
        upload.resize(padded, 0);
        self.queue.write_buffer(&buffer, 0, &upload);
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("thessa-backend-roundtrip-encoder"),
            });
        encoder.copy_buffer_to_buffer(&buffer, 0, &staging, 0, padded as u64);
        self.queue.submit(Some(encoder.finish()));
        let (tx, rx) = std::sync::mpsc::channel();
        staging
            .slice(..)
            .map_async(wgpu::MapMode::Read, move |result| {
                let _ = tx.send(result);
            });
        self.device
            .poll(wgpu::PollType::wait_indefinitely())
            .map_err(|e| BackendError::Device(format!("device poll failed: {e:?}")))?;
        rx.recv()
            .map_err(|_| BackendError::Device("staging map channel closed".into()))?
            .map_err(|e| BackendError::Device(format!("staging map failed: {e:?}")))?;
        let view = staging
            .slice(..)
            .get_mapped_range()
            .map_err(|e| BackendError::Device(format!("staging map range failed: {e:?}")))?;
        let mut out = view.to_vec();
        drop(view);
        staging.unmap();
        out.truncate(bytes.len());
        Ok(out)
    }
}

/// Hardware capability probe feeding [`Capabilities`]. Reads the adapter
/// (hardware truth), not the negotiated device.
pub fn probe_capabilities(adapter: &wgpu::Adapter) -> Capabilities {
    let features = adapter.features();
    let limits = adapter.limits();
    let mesh = features.contains(wgpu::Features::EXPERIMENTAL_MESH_SHADER)
        && limits.max_mesh_output_vertices >= MESH_MIN_OUTPUT_VERTICES
        && limits.max_mesh_output_primitives >= MESH_MIN_OUTPUT_PRIMITIVES
        && limits.max_mesh_invocations_per_workgroup >= MESH_MIN_INVOCATIONS
        && limits.max_mesh_invocations_per_dimension >= MESH_MIN_INVOCATIONS
        && limits.max_storage_buffers_per_shader_stage >= MESH_MIN_STORAGE_BUFFERS;
    Capabilities {
        terrain_conforming_supported: Some(
            limits.max_storage_buffers_per_shader_stage >= 8
                && limits.max_storage_buffer_binding_size >= 131072 * 144,
        ),
        hdr_render_target_supported: Some(
            adapter
                .get_texture_format_features(wgpu::TextureFormat::Rgba16Float)
                .allowed_usages
                .contains(
                    wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
                ),
        ),
        terrain_mesh_supported: Some(mesh),
        ray_query_supported: Some(features.contains(wgpu::Features::EXPERIMENTAL_RAY_QUERY)),
        is_discrete_gpu: Some(adapter.get_info().device_type == wgpu::DeviceType::DiscreteGpu),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn backend() -> Option<Backend> {
        match Backend::bring_up() {
            Ok(backend) => Some(backend),
            Err(BackendError::NoAdapter) => {
                eprintln!("SKIP: no GPU adapter for render-wgpu backend test");
                None
            }
            Err(other) => panic!("bring-up failed: {other}"),
        }
    }

    #[test]
    fn bring_up_reports_coherent_capabilities() {
        let Some(backend) = backend() else {
            return;
        };
        eprintln!("adapter: {}", backend.adapter_name);
        // Probe is deterministic: same adapter, same answer twice.
        assert_eq!(backend.caps, probe_capabilities(&backend.adapter));
        // Bring-up is stable-only by construction: the device must not
        // carry experimental bits implicitly. Per-pipeline negotiation
        // takes that consent later, explicitly.
        assert!(
            !backend
                .device
                .features()
                .contains(wgpu::Features::EXPERIMENTAL_RAY_QUERY)
        );
        // This rig is an integrated AMD APU: documents the expectation
        // explicitly instead of asserting a fixed GPU class.
        eprintln!(
            "caps: mesh={:?} ray_query={:?} discrete={:?}",
            backend.caps.terrain_mesh_supported,
            backend.caps.ray_query_supported,
            backend.caps.is_discrete_gpu
        );
    }

    #[test]
    fn experimental_bring_up_carries_probed_bits() {
        let Some(stable) = backend() else {
            return;
        };
        let experimental = match Backend::bring_up_experimental() {
            Ok(backend) => backend,
            Err(BackendError::NoAdapter) => {
                eprintln!("SKIP: no GPU adapter for experimental bring-up test");
                return;
            }
            Err(other) => panic!("experimental bring-up failed: {other}"),
        };
        // Same adapter class: capabilities agree across both paths.
        assert_eq!(experimental.caps, stable.caps);
        // The negotiated device carries exactly the adapter-exposed
        // interesting subset — consent path verified on real hardware.
        let want = thessa_wgpu_exp::supported_experimental(&experimental.adapter);
        assert!(
            experimental.device.features().contains(want),
            "experimental device must carry {want:?}"
        );
    }

    #[test]
    fn clear_black_and_white_read_back_exact() {
        let Some(backend) = backend() else {
            return;
        };
        // sRGB endpoints are exact under the transfer function; mid-tones
        // are not asserted here (0.5 linear is not 128 sRGB).
        for (color, expected) in [
            ([0.0, 0.0, 0.0, 1.0], [0u8, 0, 0, 255]),
            ([1.0, 1.0, 1.0, 1.0], [255u8, 255, 255, 255]),
        ] {
            let pixels = backend
                .clear_readback(64, 64, color)
                .expect("clear+readback runs");
            assert_eq!(pixels.len(), 64 * 64 * 4);
            let (quads, _) = pixels.as_chunks::<4>();
            for quad in quads {
                assert_eq!(
                    *quad, expected,
                    "clear {color:?} must read back {expected:?}"
                );
            }
        }
    }

    #[test]
    fn roundtrip_preserves_bytes_including_unaligned() {
        let Some(backend) = backend() else {
            return;
        };
        assert!(backend.roundtrip(&[]).expect("empty roundtrips").is_empty());
        // Unaligned sizes plus a 1 MiB pseudo-random payload.
        let payload: Vec<u8> = (0..1_000_003u32)
            .map(|i| (i.wrapping_mul(2654435761) >> 13) as u8)
            .collect();
        for len in [1usize, 3, 7, 1000, 1_000_003] {
            let back = backend.roundtrip(&payload[..len]).expect("roundtrip runs");
            assert_eq!(back, payload[..len], "byte parity at len {len}");
        }
    }

    #[test]
    fn clear_unaligned_rows_are_tightly_packed() {
        let Some(backend) = backend() else { return };
        for (width, height) in [(1, 1), (1, 9), (63, 3), (65, 7), (257, 2)] {
            let pixels = backend
                .clear_readback(width, height, [1.0, 0.0, 0.0, 1.0])
                .expect("padded texture copy");
            assert_eq!(pixels.len(), width as usize * height as usize * 4);
            assert!(
                pixels
                    .as_chunks::<4>()
                    .0
                    .iter()
                    .all(|p| *p == [255, 0, 0, 255])
            );
        }
    }

    #[test]
    fn clear_rejects_degenerate_sizes() {
        let Some(backend) = backend() else {
            return;
        };
        assert!(backend.clear_readback(0, 64, [0.0; 4]).is_err());
        assert!(backend.clear_readback(64, 0, [0.0; 4]).is_err());
        assert!(backend.clear_readback(8192, 64, [0.0; 4]).is_err());
    }
}
