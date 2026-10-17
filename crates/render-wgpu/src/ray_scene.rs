//! Native ray-query scene for opaque object reflections.
//!
//! `mesh.wgsl` calls the named hook `rt_reflection(...)`. When ray tracing
//! resolves on, the negotiated device carries `EXPERIMENTAL_RAY_QUERY`, and
//! the scene fits the resolved RT memory budget, this module owns the
//! acceleration structures behind that hook:
//!
//! - one BLAS per primitive, referencing a consolidated position/index copy so
//!   the raster vertex/index buffers keep their existing usage and layout. A
//!   TLAS instance references a whole BLAS, so per-primitive instancing
//!   requires per-primitive BLASes; a single shared BLAS would place every
//!   primitive at every instance transform;
//! - a TLAS whose instances are the renderer's live camera-relative mesh
//!   instances, rebuilt only when a transform, range, or pairing changes, and
//!   whose `custom_data` carries the primitive that range selects;
//! - the fragment bind group (group 3) and the per-primitive material
//!   records used to shade committed hits.
//!
//! Scope is deliberately narrow: scene-object specular reflections. Terrain,
//! atmosphere and clouds keep their raster/map paths, hit shading uses the
//! material's linear base color and the geometric normal (no UV or texture
//! sampling), and a ray that hits nothing returns the caller's isotropic
//! environment term — RT never invents reflected radiance.

use crate::{Backend, BackendError};
use std::ops::Range;
use thessa_graphics::ResolvedGraphicsSettings;
use thessa_render_core::mesh::{MeshAsset, MeshInstance};
use wgpu::util::DeviceExt;

/// Roughness above which the single mirror sample yields completely to the
/// isotropic environment term. A mirror direction cannot represent a wide
/// specular lobe, so this is an explicit bounded reduction; inside the range
/// the hit is shaded with the existing illumination model.
pub const ROUGHNESS_CUTOFF: f32 = 0.6;

/// Planning estimate for one primitive's bottom-level acceleration structure.
/// The budget is a decision boundary, not an allocation: the driver's own size
/// query decides the real resource, and this term bounds its descriptor and
/// node overhead so a huge primitive list cannot pass the budget unnoticed.
const BLAS_OVERHEAD_BYTES: u64 = 256;

/// One TLAS instance record: 3x4 transform, BLAS address, custom index, mask.
const INSTANCE_RECORD_BYTES: u64 = 64;

/// One pair record read by the shader: primitive index plus the three rows of
/// the instance's camera-relative transform, so hit positions and normals can
/// be moved out of asset-local space without relying on driver-side
/// object-to-world conventions.
const PAIR_RECORD_BYTES: u64 = 64;

/// Per-frame accounting for telemetry and captures.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RayStats {
    /// The TLAS has been built and carries at least one instance.
    pub active: bool,
    /// TLAS instances (primitive × mesh-instance pairs) in the last update.
    pub instances: u32,
    pub triangles: u64,
    /// Owned RT storage: positions, indices, ranges, material records.
    pub bytes: u64,
    /// Acceleration-structure builds issued by the last `encode`.
    pub passes: u32,
}

/// One primitive's placement inside the consolidated RT buffers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Placement {
    index_base: u32,
    index_count: u32,
    vertex_base: u32,
    vertex_count: u32,
}

/// Feasibility decision taken before the pipeline (and its group-3 layout)
/// exists, because the shader include depends on it.
#[derive(Debug)]
pub enum Enable {
    /// RT not requested, or no consumer (reflections off).
    Off,
    /// Requested but infeasible on this device/scene; the raster path stays.
    Unavailable(String),
    Ready(Plan),
}

/// Validated scene layout plus the resolved RT parameters baked into the
/// shader. Opaque to callers.
#[derive(Debug)]
pub struct Plan {
    placements: Vec<Placement>,
    ranges: Vec<[u32; 4]>,
    /// One record per primitive (RT geometry index): base color rgb, metallic,
    /// roughness. Primitives reference materials, so records are flattened
    /// here instead of indexing the shared material list from the shader.
    records: Vec<[f32; 8]>,
    triangles: u64,
    bytes: u64,
    instances: u32,
    bias_m: f32,
    max_distance_m: f32,
    bounces: u32,
}

/// Decide whether this scene can trace at all. Never panics; every rejection
/// is reported, so a raster run can log exactly why RT stayed off.
pub fn plan(
    backend: &Backend,
    source: &MeshAsset,
    resolved: Option<&ResolvedGraphicsSettings>,
    capacity: usize,
) -> Enable {
    let Some(resolved) = resolved else {
        return Enable::Off;
    };
    if !resolved.ray_tracing.is_active() || !resolved.rt_vehicles {
        return Enable::Off;
    }
    if !resolved.specular_reflections {
        return Enable::Off;
    }
    if !backend
        .device
        .features()
        .contains(wgpu::Features::EXPERIMENTAL_RAY_QUERY)
    {
        return Enable::Unavailable("device negotiated without ray query".into());
    }
    let limits = backend.device.limits();
    if limits.max_acceleration_structures_per_shader_stage == 0 {
        return Enable::Unavailable("device advertises no acceleration-structure bindings".into());
    }
    if source.primitives.is_empty() || capacity == 0 {
        return Enable::Unavailable("no mesh geometry to trace".into());
    }
    let mut placements = Vec::with_capacity(source.primitives.len());
    let mut vertices = 0u64;
    let mut indices = 0u64;
    let mut triangles = 0u64;
    for (i, primitive) in source.primitives.iter().enumerate() {
        if primitive.indices.is_empty() || primitive.indices.len() % 3 != 0 {
            return Enable::Unavailable(format!("primitive {i} has no complete triangles"));
        }
        placements.push(Placement {
            index_base: indices as u32,
            index_count: primitive.indices.len() as u32,
            vertex_base: vertices as u32,
            vertex_count: primitive.vertices.len() as u32,
        });
        vertices += primitive.vertices.len() as u64;
        indices += primitive.indices.len() as u64;
        triangles += primitive.indices.len() as u64 / 3;
    }
    if vertices > u64::from(u32::MAX) || indices > u64::from(u32::MAX) {
        return Enable::Unavailable("RT geometry exceeds 32-bit indices".into());
    }
    // One BLAS per primitive keeps per-primitive instancing possible, so the
    // device limit applies per primitive, not to the scene total.
    if let Some(primitive) = source
        .primitives
        .iter()
        .find(|p| p.indices.len() as u64 / 3 > u64::from(limits.max_blas_primitive_count))
    {
        return Enable::Unavailable(format!(
            "RT primitive has {} triangles, device limit is {}",
            primitive.indices.len() / 3,
            limits.max_blas_primitive_count
        ));
    }
    let instances = capacity
        .saturating_mul(source.primitives.len())
        .min(resolved.rt_max_instances as usize)
        .max(1);
    if instances as u32 > limits.max_tlas_instance_count {
        return Enable::Unavailable(format!(
            "RT instances exceed device limit ({})",
            limits.max_tlas_instance_count
        ));
    }
    let bytes = vertices * 16
        + indices * 4
        + placements.len() as u64 * BLAS_OVERHEAD_BYTES
        + instances as u64 * (INSTANCE_RECORD_BYTES + PAIR_RECORD_BYTES);
    if bytes > resolved.rt_scene_memory_bytes {
        return Enable::Unavailable(format!(
            "RT scene needs {bytes} bytes, budget is {}",
            resolved.rt_scene_memory_bytes
        ));
    }
    if !resolved.rt_max_distance_m.is_finite() || resolved.rt_max_distance_m <= 0.0 {
        return Enable::Unavailable("invalid RT distance budget".into());
    }
    let ranges = placements
        .iter()
        .map(|p| [p.index_base, p.vertex_base, p.index_count, 0])
        .collect();
    let records = source
        .primitives
        .iter()
        .map(|primitive| {
            let material = source.materials.get(primitive.material);
            let color = material.map_or([0.0; 4], |m| m.base_color);
            let roughness = material.map_or(1.0, |m| m.roughness);
            let metallic = material.map_or(0.0, |m| m.metallic);
            [
                color[0], color[1], color[2], metallic, roughness, 0.0, 0.0, 0.0,
            ]
        })
        .collect::<Vec<[f32; 8]>>();
    Enable::Ready(Plan {
        placements,
        ranges,
        records,
        triangles,
        bytes,
        instances: instances as u32,
        bias_m: resolved.rt_ray_bias_m,
        max_distance_m: resolved.rt_max_distance_m.clamp(0.01, 1.0e7) as f32,
        bounces: resolved.rt_reflection_bounces,
    })
}

/// Shader include for the RT build: `enable` directive, resolved constants,
/// then the hook implementation. Must be composed before any other WGSL.
pub fn shader(plan: &Plan) -> String {
    format!(
        "enable wgpu_ray_query;\nconst RT_BIAS:f32={};\nconst RT_TMAX:f32={};\nconst RT_BOUNCES:u32={};\nconst RT_ROUGHNESS_CUTOFF:f32={};\n{}",
        plan.bias_m,
        plan.max_distance_m,
        plan.bounces,
        ROUGHNESS_CUTOFF,
        include_str!("ray_scene.wgsl"),
    )
}

/// Raster substitute for the same hook: exactly the environment term the RT
/// miss path reproduces, so toggling RT cannot change unreflected shading.
pub const FALLBACK: &str = "fn rt_reflection(position:vec3<f32>,normal:vec3<f32>,eye:vec3<f32>,f0:vec3<f32>,fresnel:vec3<f32>,roughness:f32,ambient:vec3<f32>,occlusion:f32)->vec3<f32>{return ambient/OCEAN_PI*fresnel*occlusion;}\n";

/// Owned acceleration structures, hit tables, and the group-3 bind group.
pub struct RayScene {
    /// One bottom-level structure per primitive; TLAS instances select one.
    blases: Vec<wgpu::Blas>,
    sizes: Vec<wgpu::BlasTriangleGeometrySizeDescriptor>,
    tlas: wgpu::Tlas,
    bind: wgpu::BindGroup,
    positions: wgpu::Buffer,
    indices: wgpu::Buffer,
    placements: Vec<Placement>,
    transforms: Vec<[f32; 12]>,
    previous: Vec<[f32; 12]>,
    /// Primitive (and BLAS) index per TLAS slot, and the shader-side pairing.
    primitives: Vec<u32>,
    previous_primitives: Vec<u32>,
    pairs: wgpu::Buffer,
    built: bool,
    dirty: bool,
    triangles: u64,
    bytes: u64,
    passes: u32,
}

impl RayScene {
    pub fn new(
        backend: &Backend,
        plan: Plan,
        layout: &wgpu::BindGroupLayout,
        source: &MeshAsset,
    ) -> Result<Self, BackendError> {
        let device = &backend.device;
        let positions = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("rt-positions"),
            contents: &f32_bytes(source.primitives.iter().flat_map(|p| {
                p.vertices
                    .iter()
                    .flat_map(|v| [v.position[0], v.position[1], v.position[2], 0.0])
            })),
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::BLAS_INPUT,
        });
        let indices = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("rt-indices"),
            contents: &u32_bytes(
                source
                    .primitives
                    .iter()
                    .flat_map(|p| p.indices.iter().copied()),
            ),
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::BLAS_INPUT,
        });
        let ranges = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("rt-ranges"),
            contents: &u32_bytes(plan.ranges.iter().flatten().copied()),
            usage: wgpu::BufferUsages::STORAGE,
        });
        let materials = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("rt-materials"),
            contents: &f32_bytes(plan.records.iter().flatten().copied()),
            usage: wgpu::BufferUsages::STORAGE,
        });
        let sizes = plan
            .placements
            .iter()
            .map(|placement| wgpu::BlasTriangleGeometrySizeDescriptor {
                vertex_format: wgpu::VertexFormat::Float32x3,
                vertex_count: placement.vertex_count,
                index_format: Some(wgpu::IndexFormat::Uint32),
                index_count: Some(placement.index_count),
                // A derived ray query has no any-hit stage, so a candidate
                // intersection of non-opaque geometry is never committed and
                // every traversal silently misses. Scene primitives are solid
                // until alpha-tested materials join the trace.
                flags: wgpu::AccelerationStructureGeometryFlags::OPAQUE,
            })
            .collect::<Vec<_>>();
        let blases = sizes
            .iter()
            .map(|size| {
                device.create_blas(
                    &wgpu::CreateBlasDescriptor {
                        label: Some("scene-blas"),
                        flags: wgpu::AccelerationStructureFlags::PREFER_FAST_TRACE,
                        update_mode: wgpu::AccelerationStructureUpdateMode::Build,
                    },
                    wgpu::BlasGeometrySizeDescriptors::Triangles {
                        descriptors: vec![size.clone()],
                    },
                )
            })
            .collect::<Vec<_>>();
        let pairs = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("rt-pairs"),
            size: plan.instances as u64 * PAIR_RECORD_BYTES,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let tlas = device.create_tlas(&wgpu::CreateTlasDescriptor {
            label: Some("scene-tlas"),
            max_instances: plan.instances,
            flags: wgpu::AccelerationStructureFlags::PREFER_FAST_TRACE,
            update_mode: wgpu::AccelerationStructureUpdateMode::Build,
        });
        let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("rt-scene"),
            layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::AccelerationStructure(&tlas),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: materials.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: ranges.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: positions.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 4,
                    resource: indices.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 5,
                    resource: pairs.as_entire_binding(),
                },
            ],
        });
        Ok(Self {
            blases,
            sizes,
            tlas,
            bind,
            positions,
            indices,
            placements: plan.placements,
            transforms: Vec::new(),
            previous: Vec::new(),
            primitives: Vec::new(),
            previous_primitives: Vec::new(),
            pairs,
            built: false,
            dirty: false,
            triangles: plan.triangles,
            bytes: plan.bytes,
            passes: 0,
        })
    }

    pub fn bind_group(&self) -> &wgpu::BindGroup {
        &self.bind
    }

    pub fn stats(&self) -> RayStats {
        RayStats {
            active: self.built && !self.transforms.is_empty(),
            instances: self.transforms.len() as u32,
            triangles: self.triangles,
            bytes: self.bytes,
            passes: self.passes,
        }
    }

    /// Refresh TLAS instances from the renderer's live instances and the
    /// per-primitive instance ranges. Invalid transforms are rejected before
    /// any state changes; an unchanged transform list costs no rebuild.
    pub fn update(
        &mut self,
        queue: &wgpu::Queue,
        camera_m: glam::DVec3,
        instances: &[MeshInstance],
        ranges: &[Range<u32>],
    ) -> Result<u32, BackendError> {
        if ranges.len() != self.placements.len() {
            return Err(BackendError::Device("RT range count mismatch".into()));
        }
        self.transforms.clear();
        self.primitives.clear();
        for (primitive, range) in ranges.iter().enumerate() {
            for i in range.clone() {
                let Some(instance) = instances.get(i as usize) else {
                    return Err(BackendError::Device("RT instance out of bounds".into()));
                };
                let matrix = instance
                    .camera_relative_matrix(camera_m)
                    .ok_or_else(|| BackendError::Device("invalid RT instance transform".into()))?;
                let (r0, r1, r2) = (matrix.row(0), matrix.row(1), matrix.row(2));
                self.transforms.push([
                    r0.x, r0.y, r0.z, r0.w, r1.x, r1.y, r1.z, r1.w, r2.x, r2.y, r2.z, r2.w,
                ]);
                self.primitives.push(primitive as u32);
            }
        }
        if self.transforms.len() > self.tlas.get().len() {
            return Err(BackendError::Device("RT instance capacity exceeded".into()));
        }
        if self.transforms != self.previous || self.primitives != self.previous_primitives {
            // The shader reads the same pairing it traces against: slot index
            // in `custom_data`, primitive and camera-relative rows in `pairs`.
            for (slot, transform) in self.transforms.iter().enumerate() {
                self.tlas[slot] = Some(wgpu::TlasInstance::new(
                    &self.blases[self.primitives[slot] as usize],
                    *transform,
                    slot as u32,
                    0xFF,
                ));
            }
            for slot in self.transforms.len()..self.tlas.get().len() {
                self.tlas[slot] = None;
            }
            let mut staging = Vec::with_capacity(self.transforms.len() * 64);
            for (transform, primitive) in self.transforms.iter().zip(&self.primitives) {
                staging.extend_from_slice(&primitive.to_le_bytes());
                staging.extend_from_slice(&[0; 12]);
                for row in 0..3 {
                    for value in &transform[row * 4..row * 4 + 4] {
                        staging.extend_from_slice(&value.to_le_bytes());
                    }
                }
            }
            if !staging.is_empty() {
                queue.write_buffer(&self.pairs, 0, &staging);
            }
            self.previous.clear();
            self.previous.extend_from_slice(&self.transforms);
            self.previous_primitives.clear();
            self.previous_primitives.extend_from_slice(&self.primitives);
            self.dirty = true;
        }
        Ok(self.transforms.len() as u32)
    }

    /// Build the BLAS once and the TLAS whenever instances changed. Must run
    /// in the frame encoder before the mesh pass. An empty instance set
    /// leaves the structure unbuilt: no draw can query it in that state.
    pub fn encode(&mut self, encoder: &mut wgpu::CommandEncoder) -> u32 {
        self.passes = 0;
        let build_blas = !self.built;
        if self.transforms.is_empty() || (!build_blas && !self.dirty) {
            return 0;
        }
        {
            let entries = if build_blas {
                self.blases
                    .iter()
                    .zip(self.sizes.iter().zip(&self.placements))
                    .map(|(blas, (size, placement))| wgpu::BlasBuildEntry {
                        blas,
                        geometry: wgpu::BlasGeometries::TriangleGeometries(vec![
                            wgpu::BlasTriangleGeometry {
                                size,
                                vertex_buffer: &self.positions,
                                first_vertex: placement.vertex_base,
                                vertex_stride: 16,
                                index_buffer: Some(&self.indices),
                                first_index: Some(placement.index_base),
                                transform_buffer: None,
                                transform_buffer_offset: None,
                            },
                        ]),
                    })
                    .collect::<Vec<_>>()
            } else {
                Vec::new()
            };
            encoder.build_acceleration_structures(entries.iter(), [&self.tlas]);
        }
        self.built = true;
        self.dirty = false;
        self.passes = 1;
        1
    }
}

fn f32_bytes(values: impl IntoIterator<Item = f32>) -> Vec<u8> {
    values.into_iter().flat_map(f32::to_le_bytes).collect()
}

fn u32_bytes(values: impl IntoIterator<Item = u32>) -> Vec<u8> {
    values.into_iter().flat_map(u32::to_le_bytes).collect()
}
