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
    clouds: crate::clouds::CloudBinding,
    background: Option<Background>,
    meta: wgpu::Buffer,
    texels: wgpu::Buffer,
    enabled: bool,
    width: u32,
    height: u32,
    ambient_lux: [f32; 3],
    capacity_texels: u64,
    uploads: u64,
}

#[derive(Clone)]
struct Background {
    meta: wgpu::Buffer,
    texels: wgpu::Buffer,
    count: u64,
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
            clouds: crate::clouds::CloudBinding::disabled(device),
            background: None,
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
            self.background = None;
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
    pub fn base_entries(&self) -> [wgpu::BindGroupEntry<'_>; 2] {
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

    /// Optional primary-only clear field. Reflections and irradiance continue
    /// sampling the total field; the primary volume pass supplies ordered emission.
    pub fn update_background(
        &mut self,
        backend: &crate::Backend,
        field: Option<&EnvironmentRadiance>,
    ) {
        let Some(field) = field else {
            self.background = None;
            return;
        };
        let count = field.texels().len() as u64;
        let background = self.background.get_or_insert_with(|| Background {
            meta: buffer(&backend.device, META_BYTES, "background-meta"),
            texels: buffer(&backend.device, count * TEXEL_BYTES, "background-texels"),
            count,
        });
        if background.count != count {
            background.texels = buffer(&backend.device, count * TEXEL_BYTES, "background-texels");
            background.count = count;
        }
        let data: Vec<_> = field
            .texels()
            .iter()
            .flat_map(|t| t.iter().copied().chain([0.0]))
            .flat_map(f32::to_le_bytes)
            .collect();
        backend.queue.write_buffer(&background.texels, 0, &data);
        backend.queue.write_buffer(
            &background.meta,
            0,
            &meta_bytes(true, field.width(), field.height(), field.ambient_lux()),
        );
    }

    pub fn background_entries(&self) -> [wgpu::BindGroupEntry<'_>; 2] {
        self.background.as_ref().map_or_else(
            || self.base_entries(),
            |b| {
                [
                    wgpu::BindGroupEntry {
                        binding: 3,
                        resource: b.meta.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 4,
                        resource: b.texels.as_entire_binding(),
                    },
                ]
            },
        )
    }

    pub fn enabled(&self) -> bool {
        self.enabled
    }

    pub fn entries(&self) -> Vec<wgpu::BindGroupEntry<'_>> {
        self.base_entries()
            .into_iter()
            .chain(self.clouds.entries())
            .collect()
    }

    /// Consumers must rebind after replacing handles; same-size frame writes
    /// are visible to all adopted clones without another publication.
    pub fn set_clouds(&mut self, binding: crate::clouds::CloudBinding) {
        self.clouds = binding;
    }

    pub fn cloud_entries(&self) -> [wgpu::BindGroupEntry<'_>; 3] {
        self.clouds.entries()
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
                + self
                    .background
                    .as_ref()
                    .map_or(0, |b| b.count * TEXEL_BYTES + META_BYTES)
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

pub(crate) fn shader(group: u32) -> String {
    format!(
        "{}\n{}",
        include_str!("environment.wgsl"),
        crate::clouds::SHADER
    )
    .replace("@group(0)", &format!("@group({group})"))
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

    #[test]
    #[ignore = "hardware: environment probe keeps a bounded sky floor at N.V=0 and 1e-7"]
    fn grazing_floor_is_not_the_old_zero_early_return() {
        let backend = crate::Backend::bring_up().unwrap();
        let field = EnvironmentRadiance::new(8, 4, vec![[100.0; 3]; 32]).unwrap();
        let mut binding = EnvironmentBinding::new(&backend.device);
        binding.update(&backend, Some(&field));
        // The probe calls the actual shared WGSL function, not a Rust mirror.
        // N.V=0 and 1e-7 returned exactly zero before the horizon guard (the
        // old early-out covered the first, and the grazing quadrature the
        // second); the floor must keep both finite and non-zero.
        let source = format!(
            "{}\n{}\n{}",
            include_str!("../../rcbt-core/src/shaders/ocean.wgsl"),
            shader(0),
            r#"
@group(1) @binding(0) var<storage, read_write> probe_out: array<vec4<f32>>;

@compute @workgroup_size(1)
fn probe() {
    let n = vec3<f32>(0.0, 0.0, 1.0);
    let f0 = vec3<f32>(0.04, 0.04, 0.04);
    let ambient = vec3<f32>(100.0, 100.0, 100.0);
    let e0 = vec3<f32>(1.0, 0.0, 0.0);
    let e1 = normalize(vec3<f32>(1.0, 0.0, 1.0e-7));
    let e2 = normalize(vec3<f32>(0.9987492, 0.0, 0.05));
    let e3 = normalize(vec3<f32>(0.9797959, 0.0, 0.2));
    probe_out[0] = vec4<f32>(env_specular_from(vec3<f32>(0.0), n, e0, 0.5, f0, ambient, 16u, vec3<u32>(0u, 0u, 0u)), 1.0);
    probe_out[1] = vec4<f32>(env_specular_from(vec3<f32>(0.0), n, e1, 0.5, f0, ambient, 16u, vec3<u32>(0u, 0u, 0u)), 1.0);
    probe_out[2] = vec4<f32>(env_specular_from(vec3<f32>(0.0), n, e2, 0.5, f0, ambient, 16u, vec3<u32>(0u, 0u, 0u)), 1.0);
    probe_out[3] = vec4<f32>(env_specular_from(vec3<f32>(0.0), n, e3, 0.5, f0, ambient, 16u, vec3<u32>(0u, 0u, 0u)), 1.0);
}
"#
        );
        let module = backend
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("environment-grazing-probe"),
                source: wgpu::ShaderSource::Wgsl(source.into()),
            });
        let pipeline = backend
            .device
            .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some("environment-grazing-probe"),
                layout: None,
                module: &module,
                entry_point: Some("probe"),
                compilation_options: Default::default(),
                cache: None,
            });
        let group0 = backend
            .device
            .create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("environment-grazing-probe-field"),
                layout: &pipeline.get_bind_group_layout(0),
                entries: &binding.entries(),
            });
        let output = backend.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("environment-grazing-probe-output"),
            size: 4 * 16,
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_SRC
                | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let group1 = backend
            .device
            .create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("environment-grazing-probe-output-binding"),
                layout: &pipeline.get_bind_group_layout(1),
                entries: &[wgpu::BindGroupEntry {
                    binding: 0,
                    resource: output.as_entire_binding(),
                }],
            });
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        {
            let mut pass = encoder.begin_compute_pass(&Default::default());
            pass.set_pipeline(&pipeline);
            pass.set_bind_group(0, &group0, &[]);
            pass.set_bind_group(1, &group1, &[]);
            pass.dispatch_workgroups(1, 1, 1);
        }
        let staging = backend.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("environment-grazing-probe-staging"),
            size: 4 * 16,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        encoder.copy_buffer_to_buffer(&output, 0, &staging, 0, 4 * 16);
        backend.queue.submit([encoder.finish()]);
        let (tx, rx) = std::sync::mpsc::channel();
        staging
            .slice(..)
            .map_async(wgpu::MapMode::Read, move |result| {
                let _ = tx.send(result);
            });
        backend
            .device
            .poll(wgpu::PollType::wait_indefinitely())
            .unwrap();
        rx.recv().unwrap().unwrap();
        let mapped = staging.slice(..).get_mapped_range().unwrap();
        let values: Vec<[f32; 4]> = mapped
            .as_chunks::<16>()
            .0
            .iter()
            .map(|chunk| {
                std::array::from_fn(|i| {
                    f32::from_le_bytes(chunk[i * 4..i * 4 + 4].try_into().unwrap())
                })
            })
            .collect();
        drop(mapped);
        staging.unmap();
        for (index, label) in ["N.V=0", "N.V=1e-7", "N.V=0.05"].into_iter().enumerate() {
            let radiance = values[index][0];
            assert!(
                radiance.is_finite() && radiance > 0.0,
                "{label} is not a bounded non-zero horizon value: {radiance}"
            );
            assert!(
                radiance <= 100.01,
                "{label} exceeds the uniform sky radiance: {radiance}"
            );
            eprintln!("terrain_specular_floor {label} radiance={radiance}");
        }
        assert!(values[3][0].is_finite(), "{:?}", values[3]);
    }
}
