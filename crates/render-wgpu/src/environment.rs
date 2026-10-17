//! GPU binding for the backend-neutral radiance environment field.
//!
//! Transport only: the layout contract and the numerics live in
//! `thessa_render_core::environment` (CPU) and `environment.wgsl` (sampling).
//! A renderer always binds both entries; the field is enabled by a metadata
//! flag instead of by rebuilding pipelines. Buffer growth or release rebuilds
//! the frame bind group; same-size updates only write buffers.
use thessa_render_core::environment::EnvironmentRadiance;

/// `vec4<u32>` header + `vec4<f32>` ambient, matching `environment.wgsl`.
const META_BYTES: u64 = 32;
/// `array<vec4<f32>>` texel stride.
const TEXEL_BYTES: u64 = 16;
/// Placeholder texel so the storage binding is always valid.
const PLACEHOLDER_TEXELS: u64 = 1;

/// Bound environment field and its reported diagnostics.
/// Clones share GPU buffers (including same-size writes); they are publication
/// snapshots, not independent mutable fields. One owner updates the field and
/// consumers must re-adopt its clone after growth/release or metadata changes.
#[derive(Clone)]
pub struct EnvironmentBinding {
    meta: wgpu::Buffer,
    texels: wgpu::Buffer,
    enabled: bool,
    width: u32,
    height: u32,
    ambient_lux: [f32; 3],
    capacity_texels: u64,
    uploads: u64,
}

impl EnvironmentBinding {
    /// Allocate the disabled state: no field, so every consumer reproduces the
    /// legacy isotropic environment term exactly.
    pub fn new(device: &wgpu::Device) -> Self {
        let meta = buffer(device, META_BYTES, "environment-meta");
        let texels = buffer(
            device,
            PLACEHOLDER_TEXELS * TEXEL_BYTES,
            "environment-texels",
        );
        Self {
            meta,
            texels,
            enabled: false,
            width: 0,
            height: 0,
            ambient_lux: [0.0; 3],
            capacity_texels: PLACEHOLDER_TEXELS,
            uploads: 0,
        }
    }

    /// Bind `field`, or the disabled state for `None`. Returns `true` when the
    /// texel buffer had to be reallocated, in which case the caller must
    /// rebuild any bind group that referenced the previous buffer.
    pub fn update(
        &mut self,
        backend: &crate::Backend,
        field: Option<&EnvironmentRadiance>,
    ) -> bool {
        let Some(field) = field else {
            if !self.enabled {
                return false;
            }
            backend
                .queue
                .write_buffer(&self.meta, 0, &meta_bytes(false, 0, 0, [0.0; 3]));
            self.enabled = false;
            self.width = 0;
            self.height = 0;
            self.ambient_lux = [0.0; 3];
            self.uploads += 1;
            self.texels = buffer(&backend.device, TEXEL_BYTES, "environment-placeholder");
            self.capacity_texels = PLACEHOLDER_TEXELS;
            return true;
        };
        let mut reallocated = false;
        let required = field.texels().len() as u64;
        if required > self.capacity_texels {
            let capacity = required;
            self.texels = buffer(
                &backend.device,
                capacity * TEXEL_BYTES,
                "environment-texels",
            );
            self.capacity_texels = capacity;
            reallocated = true;
        }
        let mut staged = Vec::with_capacity(field.texels().len() * TEXEL_BYTES as usize);
        for texel in field.texels() {
            for value in texel {
                staged.extend_from_slice(&value.to_le_bytes());
            }
            staged.extend_from_slice(&0.0_f32.to_le_bytes());
        }
        backend.queue.write_buffer(&self.texels, 0, &staged);
        backend.queue.write_buffer(
            &self.meta,
            0,
            &meta_bytes(true, field.width(), field.height(), field.ambient_lux()),
        );
        self.enabled = true;
        self.width = field.width();
        self.height = field.height();
        self.ambient_lux = field.ambient_lux();
        self.uploads += 1;
        reallocated
    }

    /// Bind group entries 3 and 4 of the shared frame group.
    pub fn entries(&self) -> [wgpu::BindGroupEntry<'_>; 2] {
        [
            wgpu::BindGroupEntry {
                binding: 3,
                resource: self.meta.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 4,
                resource: self.texels.as_entire_binding(),
            },
        ]
    }

    pub fn enabled(&self) -> bool {
        self.enabled
    }

    /// Bound grid dimensions, `(0, 0)` while disabled.
    pub fn dimensions(&self) -> (u32, u32) {
        (self.width, self.height)
    }

    /// Cos-weighted hemispherical irradiance of the bound field, in lux.
    pub fn ambient_lux(&self) -> [f32; 3] {
        self.ambient_lux
    }

    /// Reserved grid storage excluding the metadata and 16-byte placeholder.
    pub fn storage_bytes(&self) -> u64 {
        if self.enabled {
            self.capacity_texels * TEXEL_BYTES
        } else {
            0
        }
    }

    /// Actual upload batches (repeated disable is a no-op). A renderer reports
    /// this to distinguish revision updates from per-frame staging.
    pub fn uploads(&self) -> u64 {
        self.uploads
    }
}

fn meta_bytes(enabled: bool, width: u32, height: u32, ambient_lux: [f32; 3]) -> [u8; 32] {
    let mut bytes = [0_u8; META_BYTES as usize];
    let header = [u32::from(enabled), width, height, 0];
    for (index, word) in header.iter().enumerate() {
        bytes[index * 4..index * 4 + 4].copy_from_slice(&word.to_le_bytes());
    }
    for (index, value) in ambient_lux.iter().enumerate() {
        let offset = 16 + index * 4;
        bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
    }
    bytes
}

fn buffer(device: &wgpu::Device, size: u64, label: &str) -> wgpu::Buffer {
    device.create_buffer(&wgpu::BufferDescriptor {
        label: Some(label),
        size: size.max(4),
        usage: wgpu::BufferUsages::STORAGE
            | wgpu::BufferUsages::UNIFORM
            | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn meta_layout_matches_the_shader() {
        let bytes = meta_bytes(true, 32, 16, [1.0, 2.0, 3.0]);
        assert_eq!(bytes.len(), 32);
        assert_eq!(u32::from_le_bytes(bytes[0..4].try_into().unwrap()), 1);
        assert_eq!(u32::from_le_bytes(bytes[4..8].try_into().unwrap()), 32);
        assert_eq!(u32::from_le_bytes(bytes[8..12].try_into().unwrap()), 16);
        assert_eq!(u32::from_le_bytes(bytes[12..16].try_into().unwrap()), 0);
        assert_eq!(f32::from_le_bytes(bytes[16..20].try_into().unwrap()), 1.0);
        assert_eq!(f32::from_le_bytes(bytes[20..24].try_into().unwrap()), 2.0);
        assert_eq!(f32::from_le_bytes(bytes[24..28].try_into().unwrap()), 3.0);
        assert_eq!(f32::from_le_bytes(bytes[28..32].try_into().unwrap()), 0.0);
    }

    #[test]
    fn placeholder_meta_is_disabled() {
        let bytes = meta_bytes(false, 0, 0, [0.0; 3]);
        assert_eq!(u32::from_le_bytes(bytes[0..4].try_into().unwrap()), 0);
    }
}
