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

    pub fn cloud_entries(&self) -> [wgpu::BindGroupEntry<'_>; 5] {
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
    #[ignore = "hardware: exact grazing GGX limit vs independent solid-angle quadrature, no sky floor"]
    fn grazing_limit_matches_the_brdf_without_a_radiance_floor() {
        let backend = crate::Backend::bring_up().unwrap();
        let field = EnvironmentRadiance::new(8, 4, vec![[100.0; 3]; 32]).unwrap();
        let mut binding = EnvironmentBinding::new(&backend.device);
        binding.update(&backend, Some(&field));
        // The probe calls the actual shared WGSL function, not a Rust mirror.
        // These angles expose the cancelled 0/0 in the visibility ratio.
        // Merely requiring a nonzero result would also accept the old, much
        // too bright horizon floor; compare with the literal BRDF instead.
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
        let values = probe(&backend, &binding, source, 4, [1, 1, 1]);
        for (index, nv) in [0.0, 1e-7, 0.05, 0.2].into_iter().enumerate() {
            let radiance = values[index][0];
            let reference = grazing_brdf_reference(0.5, nv, 1024) * 100.0;
            let coarser = grazing_brdf_reference(0.5, nv, 512) * 100.0;
            assert!(
                (reference - coarser).abs() < 0.01,
                "oracle did not converge"
            );
            assert!(
                radiance.is_finite() && (f64::from(radiance) - reference).abs() < 0.25,
                "N.V={nv} actual={radiance} independent={reference}"
            );
            eprintln!("terrain_specular_limit nv={nv} radiance={radiance} reference={reference}");
        }
        assert!(
            (values[0][0] - values[1][0]).abs() < 0.0001,
            "exact and near-grazing limits must be continuous"
        );
    }

    #[test]
    #[ignore = "hardware: one-sided shared GGX rejects back-facing shading normals without altering the finite grazing limit"]
    fn environment_ggx_has_no_support_below_the_shading_hemisphere() {
        let backend = crate::Backend::bring_up().unwrap();
        let field = EnvironmentRadiance::new(8, 4, vec![[100.0; 3]; 32]).unwrap();
        let mut binding = EnvironmentBinding::new(&backend.device);
        binding.update(&backend, Some(&field));
        let mut body = String::new();
        let mut cases = Vec::new();
        for nv in [-1.0_f64, -0.5, -0.1, -0.01, -1e-7] {
            for roughness in [0.25, 0.5, 1.0] {
                for samples in [4, 8, 16] {
                    let index = cases.len();
                    let x = (1.0 - nv * nv).sqrt();
                    body.push_str(&format!("let r{index}=env_specular_response(vec3(0.0),vec3(0.0,0.0,1.0),normalize(vec3({x:?},0.0,{nv:?})),{roughness:?},vec3(0.04,0.5,0.9),vec3(100.0),{samples}u,vec3<u32>(0u));probe_out[{index}]=vec4(r{index}.radiance,r{index}.reflectance.r);\n"));
                    cases.push((nv, roughness, samples));
                }
            }
        }
        let source = format!(
            "{}\n{}\n@group(1) @binding(0) var<storage,read_write> probe_out:array<vec4<f32>>;\n@compute @workgroup_size(1) fn probe(){{{body}}}",
            include_str!("../../rcbt-core/src/shaders/ocean.wgsl"),
            shader(0)
        );
        let values = probe(&backend, &binding, source, cases.len() as u64, [1, 1, 1]);
        let mut invalid = Vec::new();
        for ((nv, roughness, samples), value) in cases.into_iter().zip(values) {
            if nv == -0.1 && samples == 16 {
                eprintln!(
                    "GGX hemisphere nv={nv} roughness={roughness} samples={samples} value={value:?}"
                );
            }
            if value != [0.0; 4] {
                invalid.push((nv, roughness, samples, value));
            }
        }
        assert!(
            invalid.is_empty(),
            "one-sided GGX has no outgoing support; {} invalid cases, first={:?}",
            invalid.len(),
            invalid.first()
        );
    }

    fn probe(
        backend: &crate::Backend,
        binding: &EnvironmentBinding,
        source: String,
        count: u64,
        workgroups: [u32; 3],
    ) -> Vec<[f32; 4]> {
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
            size: count * 16,
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
            pass.dispatch_workgroups(workgroups[0], workgroups[1], workgroups[2]);
        }
        let staging = backend.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("environment-grazing-probe-staging"),
            size: count * 16,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        encoder.copy_buffer_to_buffer(&output, 0, &staging, 0, count * 16);
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
        values
    }

    #[test]
    #[ignore = "hardware: actual WGSL two-sided substrate reciprocity and collimated-source energy"]
    fn substrate_response_is_reciprocal_and_does_not_create_directional_energy() {
        let backend = crate::Backend::bring_up().unwrap();
        let field = EnvironmentRadiance::new(8, 4, vec![[1.0; 3]; 32]).unwrap();
        let mut binding = EnvironmentBinding::new(&backend.device);
        binding.update(&backend, Some(&field));
        // The 128x256 outgoing solid-angle grid is unrelated to the production
        // VNDF sampling. Integrate actual direct WGSL plus the substrate term.
        for roughness in [0.45, 0.6, 1.0] {
            let average_reference = (0..16)
                .map(|i| {
                    grazing_brdf_reference(roughness, ((f64::from(i) + 0.5) / 16.0).sqrt(), 256)
                })
                .sum::<f64>()
                / 16.0;
            for nl in [0.01, 0.2, 1.0] {
                for samples in [4, 8, 16] {
                    let body = format!(
                        r#"
@group(1) @binding(0) var<storage,read_write> probe_out:array<vec4<f32>>;
@compute @workgroup_size(8,8)
fn probe(@builtin(global_invocation_id) id:vec3<u32>){{
    let nv=(f32(id.x)+0.5)/128.0;
    let phi=(f32(id.y)+0.5)*ENV_TAU/256.0;
    let n=vec3(0.0,0.0,1.0);
    let v=vec3(sqrt(1.0-nv*nv)*cos(phi),sqrt(1.0-nv*nv)*sin(phi),nv);
    let nl:f32={nl};let l=vec3(sqrt(1.0-nl*nl),0.0,nl);let r:f32={roughness};
    let response=env_specular_response(vec3(0.0),n,v,r,vec3(0.04),vec3(1.0),{samples}u,vec3<u32>(0u));
    let rv=response.reflectance.r;
    let rl=env_directional_reflectance(n,l,r,vec3(0.04),{samples}u).r;
    let diffuse=(1.0-rv)*(1.0-rl)/OCEAN_PI;
    let spec=ocean_sun_reflection(n,v,l,vec3(1.0),r,0.04).r/nl;
    let reverse=ocean_sun_reflection(n,l,v,vec3(1.0),r,0.04).r/nv;
    probe_out[id.y*128u+id.x]=vec4((spec+diffuse)*nv,abs(spec-reverse),rv,response.radiance.r);
    if(id.x==0u && id.y==0u){{probe_out[32768]=vec4(env_average_reflectance(r,vec3(0.04),{samples}u),1.0);}}
}}
"#
                    );
                    let source = format!(
                        "{}\n{}\n{body}",
                        include_str!("../../rcbt-core/src/shaders/ocean.wgsl"),
                        shader(0)
                    );
                    let values = probe(&backend, &binding, source, 128 * 256 + 1, [16, 32, 1]);
                    let average = f64::from(values[32768][0]);
                    assert!(
                        (average - average_reference).abs() < 0.01,
                        "ambient r={roughness} samples={samples} actual={average} reference={average_reference}"
                    );
                    let values = &values[..32768];
                    let energy: f64 = values.iter().map(|v| f64::from(v[0])).sum::<f64>()
                        * std::f64::consts::TAU
                        / (128.0 * 256.0);
                    let reciprocal_error = values.iter().map(|v| v[1]).fold(0.0_f32, f32::max);
                    // Absolute incident-energy envelope, not strict certification
                    // of finite quadrature over every possible material/direction.
                    assert!(
                        energy.is_finite() && energy <= 1.005,
                        "r={roughness} nl={nl} samples={samples} energy={energy}"
                    );
                    assert!(
                        reciprocal_error < 0.0001,
                        "reciprocity error={reciprocal_error}"
                    );
                    eprintln!(
                        "substrate_energy r={roughness} nl={nl} samples={samples} outgoing={energy} reciprocity_error={reciprocal_error} average={average} independent_average={average_reference}"
                    );
                }
            }
        }
    }

    #[test]
    #[ignore = "hardware: anisotropic-sky grazing GGX versus independent converged solid-angle integration"]
    fn anisotropic_grazing_environment_matches_independent_reference() {
        use glam::DVec3;
        let backend = crate::Backend::bring_up().unwrap();
        // Synthetic oblique sky: dark lower hemisphere and a smooth bright
        // lobe. Its analytic radiance is bounded by 100; this is not a captured
        // canonical sky or a general HDR error guarantee.
        let up = DVec3::new(-0.886, 0.4, -0.235).normalize();
        let tangent = up.cross(DVec3::Y).normalize();
        let bright = (up * 0.2 + tangent).normalize();
        let mut texels = Vec::new();
        for y in 0..16 {
            let elevation = ((f64::from(y) + 0.5) / 16.0 - 0.5) * std::f64::consts::PI;
            for x in 0..32 {
                let phi = ((f64::from(x) + 0.5) / 32.0 - 0.5) * std::f64::consts::TAU;
                let d = DVec3::new(
                    elevation.cos() * phi.cos(),
                    elevation.sin(),
                    elevation.cos() * phi.sin(),
                );
                let radiance = if d.dot(up) > 0.0 {
                    5.0 + 95.0 * (20.0 * (d.dot(bright) - 1.0)).exp()
                } else {
                    0.0
                };
                texels.push([radiance as f32; 3]);
            }
        }
        let field = EnvironmentRadiance::new(32, 16, texels).unwrap();
        let mut binding = EnvironmentBinding::new(&backend.device);
        binding.update(&backend, Some(&field));
        let mut body = String::new();
        let mut cases = Vec::new();
        for slope in [-0.05, 0.0, 0.05] {
            let normal = (up + tangent * slope).normalize();
            for nv in [0.001_f64, 0.02, 0.2] {
                let t = (tangent - normal * normal.dot(tangent)).normalize();
                let eye = normal * nv + t * (1.0 - nv * nv).sqrt();
                for roughness in [0.6_f64, 0.85, 1.0] {
                    let i = cases.len();
                    let n = normal.to_array();
                    let v = eye.to_array();
                    body.push_str(&format!(
                        r#"
let n{i}=normalize(vec3({:?},{:?},{:?}));
let v{i}=normalize(vec3({:?},{:?},{:?}));
let a{i}=env_specular_response(vec3(0.0),n{i},v{i},{roughness:?},vec3(0.04),vec3(0.0),16u,vec3<u32>(0u));
let b{i}=env_specular_response(vec3(0.0),n{i},v{i},{roughness:?},vec3(0.04),vec3(0.0),4096u,vec3<u32>(0u));
probe_out[{i}]=vec4(a{i}.radiance.r,b{i}.radiance.r,a{i}.reflectance.r,b{i}.reflectance.r);
"#,
                        n[0], n[1], n[2], v[0], v[1], v[2]
                    ));
                    cases.push((normal, eye, nv, roughness));
                }
            }
        }
        let source = format!(
            "{}\n{}\n@group(1) @binding(0) var<storage,read_write> probe_out:array<vec4<f32>>;\n@compute @workgroup_size(1) fn probe(){{{body}}}",
            include_str!("../../rcbt-core/src/shaders/ocean.wgsl"),
            shader(0)
        );
        let values = probe(&backend, &binding, source, cases.len() as u64, [1, 1, 1]);
        for ((normal, eye, nv, r), v) in cases.into_iter().zip(values) {
            // Uniform incident solid-angle midpoint quadrature, independent of
            // VNDF sampling/PDF. Sample the same published bilinear grid on CPU
            // so this tests the integrator rather than sky-grid discretization.
            let reference = |size: usize| {
                let t = normal.cross(DVec3::Y).normalize();
                let b = normal.cross(t);
                let a2 = (r * r).max(0.0025).powi(2);
                let k = (r + 1.0).powi(2) / 8.0;
                let mut sum = 0.0;
                for y in 0..size {
                    let mu = (y as f64 + 0.5) / size as f64;
                    let sine = (1.0 - mu * mu).sqrt();
                    for x in 0..size * 2 {
                        let phi = std::f64::consts::TAU * (x as f64 + 0.5) / (size * 2) as f64;
                        let light = normal * mu + (t * phi.cos() + b * phi.sin()) * sine;
                        let half = (eye + light).normalize();
                        let nh = normal.dot(half);
                        let d = a2 / (std::f64::consts::PI * (nh * nh * (a2 - 1.0) + 1.0).powi(2));
                        let fresnel = 0.04 + 0.96 * (1.0 - eye.dot(half)).powi(5);
                        let weight =
                            d * fresnel * mu / (4.0 * (nv * (1.0 - k) + k) * (mu * (1.0 - k) + k));
                        sum += weight * f64::from(field.sample(light.as_vec3().to_array())[0]);
                    }
                }
                sum * std::f64::consts::TAU / (2 * size * size) as f64
            };
            let expected = reference(512);
            let coarser = reference(256);
            eprintln!(
                "anisotropic_grazing nv={nv} r={r} n={normal:?} production16={} dense4096={} solid_angle512={expected} convergence={} error16={} error4096={}",
                v[0],
                v[1],
                (expected - coarser).abs(),
                (f64::from(v[0]) - expected).abs(),
                (f64::from(v[1]) - expected).abs()
            );
            assert!(
                (expected - coarser).abs() < 0.00003,
                "independent reference convergence"
            );
            assert!(
                (f64::from(v[1]) - expected).abs() < 0.0001,
                "dense GGX must match the independent reference to 1e-6 peak source radiance"
            );
            assert!(
                (f64::from(v[0]) - expected).abs() < 0.04,
                "16 samples must remain within the fixture's 4e-4 peak-source envelope"
            );
        }
    }

    // Integrate D*G*F/(4*N.V) over incident solid angle, independent of the
    // shader's visible-normal importance sampler and sampling PDF. Gv/N.V has
    // a finite limit 1/k; no epsilon or artificial light source is needed.
    fn grazing_brdf_reference(r: f64, nv: f64, n: usize) -> f64 {
        let ex = (1.0 - nv * nv).sqrt();
        let a2 = (r * r).max(0.0025).powi(2);
        let k = (r + 1.0).powi(2) / 8.0;
        let mut sum = 0.0;
        for y in 0..n {
            let mu = (y as f64 + 0.5) / n as f64;
            let sine = (1.0 - mu * mu).sqrt();
            let gl = mu / (mu * (1.0 - k) + k);
            for x in 0..n {
                let phi = std::f64::consts::TAU * (x as f64 + 0.5) / n as f64;
                let h =
                    glam::DVec3::new(ex + sine * phi.cos(), sine * phi.sin(), nv + mu).normalize();
                let d = a2 / (std::f64::consts::PI * (h.z * h.z * (a2 - 1.0) + 1.0).powi(2));
                let f = 0.04 + 0.96 * (1.0 - ex * h.x - nv * h.z).powi(5);
                sum += d * gl * f / (4.0 * (nv * (1.0 - k) + k));
            }
        }
        sum * std::f64::consts::TAU / (n * n) as f64
    }
}
