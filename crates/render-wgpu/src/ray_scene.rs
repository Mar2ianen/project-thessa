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
//! Meshes can also borrow the resident generated terrain pool: its stable
//! slots own persistent BLASes and only dirty geometry is rebuilt. Atmosphere
//! and clouds keep their raster/map paths. Mesh hits interpolate UVs and
//! normals and sample the raster material textures with explicit LOD zero.
//! A miss samples the shared environment; RT never invents reflected radiance.

use crate::terrain_ray::{TERRAIN_TAG, TerrainRaySource, TerrainRays};
use crate::{Backend, BackendError};
use std::ops::Range;
use thessa_graphics::{Quality, ResolvedGraphicsSettings};
use thessa_render_core::mesh::{MeshAsset, MeshInstance};
use wgpu::util::DeviceExt;

/// Roughness above which the single mirror sample yields completely to the
/// environment term. A mirror direction cannot represent a wide
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
/// Three padded vec4s: position, smooth normal, UV. Also the BLAS stride.
const VERTEX_BYTES: u64 = 48;
/// Colour/metallic, roughness/normal-scale/AO, four texture indices.
const MATERIAL_BYTES: u64 = 48;

/// Per-frame accounting for telemetry and captures.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RayStats {
    /// The TLAS has been built and carries at least one instance.
    pub active: bool,
    /// TLAS instances (primitive × mesh-instance pairs) in the last update.
    pub instances: u32,
    pub triangles: u64,
    /// RT planning bytes, including hit attributes/tables and AS estimates;
    /// shared raster textures are not duplicated or counted again.
    pub bytes: u64,
    /// Acceleration-structure builds issued by the last `encode`.
    pub passes: u32,
    pub terrain_instances: u32,
    pub terrain_blas_builds: u32,
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
    /// RT not requested, or neither reflections nor shadows consumes it.
    Off,
    /// Requested but infeasible on this device/scene; the raster path stays.
    Unavailable(String),
    Ready(Box<Plan>),
}

/// Validated scene layout plus the resolved RT parameters baked into the
/// shader. Opaque to callers.
#[derive(Debug)]
pub struct Plan {
    mesh_enabled: bool,
    terrain: Option<(TerrainRaySource, usize)>,
    terrain_reason: Option<String>,
    placements: Vec<Placement>,
    ranges: Vec<[u32; 4]>,
    records: Vec<u8>,
    /// Unique (source texture ID, sRGB) bindings, shared with raster resources.
    textures: Vec<(usize, bool)>,
    quality: Quality,
    shadow_rays: bool,
    reflections: bool,
    vertex_stride: u64,
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
    quality: Quality,
    reflections: bool,
) -> Enable {
    plan_inner(
        backend,
        source,
        resolved,
        capacity,
        quality,
        reflections,
        false,
    )
}

fn plan_inner(
    backend: &Backend,
    source: &MeshAsset,
    resolved: Option<&ResolvedGraphicsSettings>,
    capacity: usize,
    quality: Quality,
    reflections: bool,
    terrain_consumer: bool,
) -> Enable {
    let Some(resolved) = resolved else {
        return Enable::Off;
    };
    if !resolved.ray_tracing.is_active() || (!resolved.rt_vehicles && !terrain_consumer) {
        return Enable::Off;
    }
    if !reflections && !resolved.shadow_enabled {
        return Enable::Off;
    }
    if let Err(reason) = source.validate() {
        return Enable::Unavailable(format!("invalid RT source: {reason}"));
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
    let mesh_enabled = resolved.rt_vehicles;
    if !mesh_enabled {
        vertices = 0;
        indices = 0;
        triangles = 0;
    }
    if vertices > u64::from(u32::MAX) || indices > u64::from(u32::MAX) {
        return Enable::Unavailable("RT geometry exceeds 32-bit indices".into());
    }
    // One BLAS per primitive keeps per-primitive instancing possible, so the
    // device limit applies per primitive, not to the scene total.
    if let Some(primitive) = source
        .primitives
        .iter()
        .filter(|_| mesh_enabled)
        .find(|p| p.indices.len() as u64 / 3 > u64::from(limits.max_blas_primitive_count))
    {
        return Enable::Unavailable(format!(
            "RT primitive has {} triangles, device limit is {}",
            primitive.indices.len() / 3,
            limits.max_blas_primitive_count
        ));
    }
    let instances = if mesh_enabled {
        capacity
            .saturating_mul(source.primitives.len())
            .min(resolved.rt_max_instances as usize)
            .max(1)
    } else {
        0
    };
    if instances as u32 > limits.max_tlas_instance_count {
        return Enable::Unavailable(format!(
            "RT instances exceed device limit ({})",
            limits.max_tlas_instance_count
        ));
    }
    let mut textures = Vec::new();
    let mut records = Vec::new();
    for primitive in source
        .primitives
        .iter()
        .filter(|_| reflections && mesh_enabled)
    {
        let material = &source.materials[primitive.material];
        records.extend(f32_bytes([
            material.base_color[0],
            material.base_color[1],
            material.base_color[2],
            material.metallic,
            material.roughness,
            material.normal_scale,
            material.occlusion_strength,
            0.0,
        ]));
        let maps = [
            material.base_texture,
            material.metallic_roughness_texture,
            material.normal_texture,
            material.occlusion_texture,
        ];
        for (channel, texture) in maps.into_iter().enumerate() {
            let texture = texture.filter(|_| channel == 0 || quality == Quality::High);
            let index = texture.map_or(u32::MAX, |id| {
                let key = (id, channel == 0);
                if let Some(index) = textures.iter().position(|entry| *entry == key) {
                    index as u32
                } else {
                    textures.push(key);
                    (textures.len() - 1) as u32
                }
            });
            records.extend(index.to_le_bytes());
        }
    }
    // Static texture/sampler switches need no optional bindless features.
    // Reserve the primary material's four maps and the shadow atlas. Reject
    // rather than silently sampling another material if the scene cannot fit.
    if textures.len() as u32 + 5 > limits.max_sampled_textures_per_shader_stage
        || textures.len() as u32 + 5 > limits.max_samplers_per_shader_stage
    {
        return Enable::Unavailable(format!(
            "RT material bindings need {} textures/samplers plus 5 raster/shadow bindings, exceeding device limits",
            textures.len()
        ));
    }
    let vertex_stride = if reflections { VERTEX_BYTES } else { 16 };
    let hit_table_bytes = if reflections {
        placements.len() as u64 * (16 + MATERIAL_BYTES)
    } else {
        0
    };
    let pair_bytes = if reflections { PAIR_RECORD_BYTES } else { 0 };
    let storage_sizes = [
        vertices * vertex_stride,
        indices * 4,
        placements.len() as u64 * 16 * u64::from(reflections && mesh_enabled),
        records.len() as u64,
        instances as u64 * pair_bytes,
    ];
    if storage_sizes.into_iter().any(|bytes| {
        bytes > limits.max_buffer_size || bytes > limits.max_storage_buffer_binding_size
    }) {
        return Enable::Unavailable("RT consolidated storage exceeds device limits".into());
    }
    let bytes = if mesh_enabled {
        vertices * vertex_stride
            + indices * 4
            + hit_table_bytes
            + placements.len() as u64 * BLAS_OVERHEAD_BYTES
            + instances as u64 * (INSTANCE_RECORD_BYTES + pair_bytes)
    } else {
        // Shader-required placeholders, no mesh geometry or structures.
        if reflections {
            48 + 4 + 16 + 48
        } else {
            16 + 4
        }
    };
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
    Enable::Ready(Box::new(Plan {
        mesh_enabled,
        terrain: None,
        terrain_reason: None,
        placements,
        ranges,
        records,
        textures,
        quality,
        shadow_rays: resolved.shadow_enabled,
        reflections,
        vertex_stride,
        triangles,
        bytes,
        instances: instances as u32,
        bias_m: resolved.rt_ray_bias_m,
        max_distance_m: resolved.rt_max_distance_m.clamp(0.01, 1.0e7) as f32,
        bounces: resolved.rt_reflection_bounces,
    }))
}

/// Extend the existing scene plan with a borrowed generated terrain pool.
/// A partition is admitted whole at publication, never partially truncated.
pub fn plan_with_terrain(
    backend: &Backend,
    source: &MeshAsset,
    resolved: Option<&ResolvedGraphicsSettings>,
    capacity: usize,
    quality: Quality,
    reflections: bool,
    terrain: Option<&TerrainRaySource>,
) -> Enable {
    let mut result = plan_inner(
        backend,
        source,
        resolved,
        capacity,
        quality,
        reflections,
        terrain.is_some() && resolved.is_some_and(|r| r.rt_terrain),
    );
    if let (Enable::Ready(plan), Some(settings), Some(terrain)) = (&mut result, resolved, terrain)
        && settings.rt_terrain
    {
        let limits = backend.device.limits();
        let available = settings
            .rt_scene_memory_bytes
            .saturating_sub(plan.bytes)
            .saturating_sub(crate::terrain_ray::INDEX_BYTES);
        let slots = (available / crate::terrain_ray::SLOT_BYTES)
            .min(terrain.capacity as u64)
            .min(u64::from(
                settings.rt_max_instances.saturating_sub(plan.instances),
            ))
            .min(u64::from(
                limits
                    .max_tlas_instance_count
                    .saturating_sub(plan.instances),
            )) as usize;
        if slots == 0 {
            plan.terrain_reason =
                Some("RT terrain has no instance/memory capacity; raster terrain fallback".into());
        } else if reflections
            && (limits.max_storage_buffers_per_shader_stage < 9
                || plan.textures.len() as u32 + 6 + u32::from(terrain.ocean.is_some())
                    > limits.max_sampled_textures_per_shader_stage
                || plan.textures.len() as u32 + 6 + u32::from(terrain.ocean.is_some())
                    > limits.max_samplers_per_shader_stage)
        {
            plan.terrain_reason = Some(
                "RT terrain hit bindings exceed device limits; raster terrain fallback".into(),
            );
        } else if limits.max_blas_primitive_count < 2048 {
            plan.terrain_reason = Some(
                "RT terrain grid exceeds BLAS primitive limit; raster terrain fallback".into(),
            );
        } else {
            plan.instances += slots as u32;
            plan.bytes +=
                slots as u64 * crate::terrain_ray::SLOT_BYTES + crate::terrain_ray::INDEX_BYTES;
            plan.terrain = Some((terrain.clone(), slots));
        }
    }
    if let Enable::Ready(plan) = &result
        && !plan.mesh_enabled
        && plan.terrain.is_none()
    {
        return Enable::Unavailable(
            plan.terrain_reason
                .clone()
                .unwrap_or_else(|| "no admitted ray geometry".into()),
        );
    }
    result
}

/// Shader include for the RT build: `enable` directive, resolved constants,
/// then the hook implementation. Must be composed before any other WGSL.
pub fn shader(plan: &Plan) -> String {
    let mut sampling = String::new();
    let map_base = if plan.terrain.is_some() && plan.reflections {
        10
    } else {
        6
    };
    for index in 0..plan.textures.len() {
        sampling.push_str(&format!(
            "@group(3) @binding({}) var rt_texture_{index}:texture_2d<f32>;\n@group(3) @binding({}) var rt_sampler_{index}:sampler;\n",
            map_base + index * 2, map_base + 1 + index * 2,
        ));
    }
    sampling.push_str("fn rt_sample(index:u32,uv:vec2<f32>)->vec4<f32>{switch index{\n");
    for index in 0..plan.textures.len() {
        sampling.push_str(&format!("case {index}u:{{return textureSampleLevel(rt_texture_{index},rt_sampler_{index},uv,0.0);}}\n"));
    }
    sampling.push_str("default:{return vec4(1.0);}}}\n");
    if let Some((terrain, _)) = &plan.terrain
        && plan.reflections
    {
        sampling.push_str(&format!(
            "const RT_TERRAIN_RADIUS:f32={};\n",
            terrain.radius_m as f32
        ));
        let [r, g, b, _] = crate::terrain::FALLBACK_ALBEDO;
        sampling.push_str(&format!(
            "const RT_TERRAIN_FALLBACK:vec3<f32>=vec3({},{},{});\n",
            f32::from(r) / 255.0,
            f32::from(g) / 255.0,
            f32::from(b) / 255.0
        ));
        sampling.push_str(
            &thessa_rcbt_core::shaders::MATERIAL_SAMPLE_WGSL
                .replace("@group(0) @binding(16)", "@group(3) @binding(7)"),
        );
        let hit=include_str!("terrain_ray.wgsl").replace("// OCEAN_HIT_HOOK",if terrain.ocean.is_some(){
            "let water=ocean_material_weight(n0.w*bary.x+n1.w*bary.y+n2.w*bary.z,roughness);\nif(water>0.0){let wave=ocean_spectral(position,direction,0.0);normal=normalize(mix(normal,direction-wave.gradient,water));normal=select(normal,-normal,dot(normal,geometry)<0.0);roughness=mix(roughness,clamp(pow(pow(ocean_frame.optics.y,4.0)+wave.variance,0.25),0.02,1.0),water);dielectric=mix(0.04,ocean_frame.optics.x,water);}"
        }else{""});
        if terrain.ocean.is_some() {
            let base = map_base + plan.textures.len() * 2;
            sampling.push_str(
                &crate::ocean::shader(3)
                    .replace("@binding(0)", &format!("@binding({base})"))
                    .replace("@binding(1)", &format!("@binding({})", base + 1))
                    .replace("@binding(2)", &format!("@binding({})", base + 2)),
            );
        }
        sampling.push_str(&hit);
    }
    let hit_shader = include_str!("ray_scene.wgsl").replace("// TERRAIN_HIT_HOOK", if plan.terrain.is_some() {
        "if ((pair.words.x & 0x80000000u) != 0u) { return rt_terrain_surface(hit, pair, ray_direction); }"
    } else { "" });
    format!(
        "enable wgpu_ray_query;\nconst RT_BIAS:f32={};\nconst RT_TMAX:f32={};\nconst RT_BOUNCES:u32={};\nconst RT_ROUGHNESS_CUTOFF:f32={};\nconst RT_LOW_QUALITY:bool={};\nconst RT_SHADOWS:bool={};\n{}\n{}\n{}",
        plan.bias_m,
        plan.max_distance_m,
        plan.bounces,
        ROUGHNESS_CUTOFF,
        plan.quality == Quality::Low,
        plan.shadow_rays,
        sampling,
        include_str!("ray_query.wgsl"),
        if plan.reflections {
            &hit_shader
        } else {
            REFLECTION_FALLBACK
        },
    )
}

/// Raster substitute for the same hook: exactly the environment term the RT
/// miss path reproduces, in the same operand order, so toggling RT cannot
/// change unreflected shading. `env_radiance` is the shared environment field
/// (a real sky radiance grid when one is bound, the isotropic `ambient_lux /
/// PI` otherwise).
const REFLECTION_FALLBACK: &str = "fn rt_reflection(position:vec3<f32>,normal:vec3<f32>,eye:vec3<f32>,f0:vec3<f32>,fresnel:vec3<f32>,roughness:f32,ambient:vec3<f32>,occlusion:f32)->vec3<f32>{return env_radiance(reflect(-eye,normal),ambient)*fresnel*occlusion;}\n";

pub fn fallback_shader() -> String {
    format!(
        "{REFLECTION_FALLBACK}fn surface_visibility(position:vec3<f32>,normal:vec3<f32>,sun:u32)->f32{{return shadow_visibility(position,normal,sun);}}\n"
    )
}

/// Owned acceleration structures, hit tables, and the group-3 bind group.
pub struct RayScene {
    mesh_enabled: bool,
    terrain: Option<TerrainRays>,
    terrain_reason: Option<String>,
    queue: std::sync::Arc<wgpu::Queue>,
    mesh_count: usize,
    camera_m: glam::DVec3,
    terrain_blas_builds: u32,
    /// One bottom-level structure per primitive; TLAS instances select one.
    blases: Vec<wgpu::Blas>,
    sizes: Vec<wgpu::BlasTriangleGeometrySizeDescriptor>,
    tlas: wgpu::Tlas,
    bind: wgpu::BindGroup,
    positions: wgpu::Buffer,
    indices: wgpu::Buffer,
    vertex_stride: u64,
    placements: Vec<Placement>,
    transforms: Vec<[f32; 12]>,
    previous: Vec<[f32; 12]>,
    /// Primitive (and BLAS) index per TLAS slot, and the shader-side pairing.
    primitives: Vec<u32>,
    previous_primitives: Vec<u32>,
    pairs: Option<wgpu::Buffer>,
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
        textures: &mut std::collections::BTreeMap<
            (Option<usize>, [u8; 4], bool),
            wgpu::TextureView,
        >,
    ) -> Result<Self, BackendError> {
        let device = &backend.device;
        let positions = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("rt-positions"),
            contents: &if plan.mesh_enabled {
                f32_bytes(source.primitives.iter().flat_map(|p| {
                    p.vertices.iter().flat_map(|v| {
                        [
                            v.position[0],
                            v.position[1],
                            v.position[2],
                            0.0,
                            v.normal[0],
                            v.normal[1],
                            v.normal[2],
                            0.0,
                            v.uv[0],
                            v.uv[1],
                            0.0,
                            0.0,
                        ]
                        .into_iter()
                        .take(plan.vertex_stride as usize / 4)
                    })
                }))
            } else {
                vec![0; plan.vertex_stride as usize]
            },
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::BLAS_INPUT,
        });
        let indices = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("rt-indices"),
            contents: &if plan.mesh_enabled {
                u32_bytes(
                    source
                        .primitives
                        .iter()
                        .flat_map(|p| p.indices.iter().copied()),
                )
            } else {
                vec![0; 4]
            },
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::BLAS_INPUT,
        });
        let ranges = plan.reflections.then(|| {
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("rt-ranges"),
                contents: &if plan.mesh_enabled {
                    u32_bytes(plan.ranges.iter().flatten().copied())
                } else {
                    vec![0; 16]
                },
                usage: wgpu::BufferUsages::STORAGE,
            })
        });
        let materials = plan.reflections.then(|| {
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("rt-materials"),
                contents: if plan.mesh_enabled {
                    &plan.records
                } else {
                    &[0; 48]
                },
                usage: wgpu::BufferUsages::STORAGE,
            })
        });
        let sizes = plan
            .placements
            .iter()
            .filter(|_| plan.mesh_enabled)
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
        let pairs = plan.reflections.then(|| {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("rt-pairs"),
                size: plan.instances as u64 * PAIR_RECORD_BYTES,
                usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            })
        });
        let tlas = device.create_tlas(&wgpu::CreateTlasDescriptor {
            label: Some("scene-tlas"),
            max_instances: plan.instances,
            flags: wgpu::AccelerationStructureFlags::PREFER_FAST_TRACE,
            update_mode: wgpu::AccelerationStructureUpdateMode::Build,
        });
        let maps: Vec<_> = plan
            .textures
            .iter()
            .map(|&(id, srgb)| {
                crate::mesh::material_texture(backend, source, Some(id), [255; 4], srgb, textures)
            })
            .collect();
        let terrain = plan
            .terrain
            .map(|(source, capacity)| TerrainRays::new(device, source, capacity));
        let mut entries = vec![wgpu::BindGroupEntry {
            binding: 0,
            resource: wgpu::BindingResource::AccelerationStructure(&tlas),
        }];
        if let (Some(materials), Some(ranges), Some(pairs)) = (&materials, &ranges, &pairs) {
            entries.extend([
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
            ]);
            if let Some(terrain) = &terrain {
                entries.extend([
                    wgpu::BindGroupEntry {
                        binding: 6,
                        resource: terrain.source.vertices.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 7,
                        resource: terrain.source.directory.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 8,
                        resource: wgpu::BindingResource::TextureView(&terrain.source.material),
                    },
                    wgpu::BindGroupEntry {
                        binding: 9,
                        resource: wgpu::BindingResource::Sampler(&terrain.source.sampler),
                    },
                ]);
            }
        }
        let map_base = if terrain.is_some() && plan.reflections {
            10
        } else {
            6
        };
        for (index, (view, sampler)) in maps.iter().enumerate() {
            entries.push(wgpu::BindGroupEntry {
                binding: map_base + index as u32 * 2,
                resource: wgpu::BindingResource::TextureView(view),
            });
            entries.push(wgpu::BindGroupEntry {
                binding: map_base + 1 + index as u32 * 2,
                resource: wgpu::BindingResource::Sampler(sampler),
            });
        }
        if plan.reflections
            && let Some(ocean) = terrain.as_ref().and_then(|t| t.source.ocean.as_ref())
        {
            let base = map_base + maps.len() as u32 * 2;
            entries.extend(ocean.entries().into_iter().map(|mut e| {
                e.binding += base;
                e
            }));
        }
        let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("rt-scene"),
            layout,
            entries: &entries,
        });
        Ok(Self {
            mesh_enabled: plan.mesh_enabled,
            terrain,
            terrain_reason: plan.terrain_reason,
            queue: backend.queue.clone(),
            mesh_count: 0,
            camera_m: glam::DVec3::ZERO,
            terrain_blas_builds: 0,
            blases,
            sizes,
            tlas,
            bind,
            positions,
            indices,
            vertex_stride: plan.vertex_stride,
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
            triangles: self.triangles
                + self
                    .terrain
                    .as_ref()
                    .map_or(0, |t| t.active.len() as u64 * 2048),
            bytes: self.bytes,
            passes: self.passes,
            terrain_instances: self.terrain.as_ref().map_or(0, |t| t.active.len() as u32),
            terrain_blas_builds: self.terrain_blas_builds,
        }
    }

    pub fn terrain_reason(&self) -> Option<&str> {
        self.terrain
            .as_ref()
            .and_then(|t| t.reason.as_deref())
            .or(self.terrain_reason.as_deref())
    }

    pub fn sync_terrain(
        &mut self,
        backend: &Backend,
        terrain: &crate::terrain::TerrainRenderer,
    ) -> Result<(), BackendError> {
        if let Some(pool) = &mut self.terrain {
            pool.sync(backend, terrain)?;
        }
        Ok(())
    }

    /// Carry terrain-owned geometry across an unrelated immutable mesh-source
    /// replacement. Bindings already reference the same borrowed buffers; only
    /// the new mesh BLASes/TLAS need building. Admission is rechecked by sync.
    pub fn adopt_terrain(&mut self, previous: &mut Self) -> bool {
        let (Some(next), Some(old)) = (&self.terrain, &previous.terrain) else {
            return false;
        };
        if next.source.vertices != old.source.vertices || old.active.len() > next.capacity {
            return false;
        }
        let capacity = next.capacity;
        self.terrain = previous.terrain.take();
        self.terrain.as_mut().unwrap().capacity = capacity;
        true
    }

    /// Refresh TLAS instances from the renderer's live instances and the
    /// per-primitive instance ranges. Invalid transforms are rejected before
    /// any state changes; an unchanged transform list costs no rebuild.
    pub fn update(
        &mut self,
        _queue: &wgpu::Queue,
        camera_m: glam::DVec3,
        instances: &[MeshInstance],
        ranges: &[Range<u32>],
    ) -> Result<u32, BackendError> {
        if ranges.len() != self.placements.len() {
            return Err(BackendError::Device("RT range count mismatch".into()));
        }
        self.transforms.clear();
        self.primitives.clear();
        for (primitive, range) in ranges.iter().enumerate().filter(|_| self.mesh_enabled) {
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
        self.mesh_count = self.transforms.len();
        self.camera_m = camera_m;
        self.passes = 0;
        self.terrain_blas_builds = 0;
        Ok(self.mesh_count as u32)
    }

    fn prepare(&mut self) {
        self.transforms.truncate(self.mesh_count);
        self.primitives.truncate(self.mesh_count);
        if let Some(terrain) = &self.terrain {
            for &(slot, anchor) in &terrain.active {
                let translation =
                    (glam::DVec3::from_array(anchor.anchor_body_m) - self.camera_m).as_vec3();
                self.transforms.push([
                    1.0,
                    0.0,
                    0.0,
                    translation.x,
                    0.0,
                    1.0,
                    0.0,
                    translation.y,
                    0.0,
                    0.0,
                    1.0,
                    translation.z,
                ]);
                self.primitives.push(TERRAIN_TAG | slot as u32);
            }
        }
        if self.transforms != self.previous
            || self.primitives != self.previous_primitives
            || self.terrain.as_ref().is_some_and(|t| !t.dirty.is_empty())
        {
            // The shader reads the same pairing it traces against: slot index
            // in `custom_data`, primitive and camera-relative rows in `pairs`.
            for (slot, transform) in self.transforms.iter().enumerate() {
                self.tlas[slot] = Some(wgpu::TlasInstance::new(
                    if self.primitives[slot] & TERRAIN_TAG == 0 {
                        &self.blases[self.primitives[slot] as usize]
                    } else {
                        self.terrain.as_ref().unwrap().slots
                            [(self.primitives[slot] & !TERRAIN_TAG) as usize]
                            .as_ref()
                            .unwrap()
                    },
                    *transform,
                    slot as u32,
                    0xFF,
                ));
            }
            for slot in self.transforms.len()..self.tlas.get().len() {
                self.tlas[slot] = None;
            }
            if let Some(pairs) = &self.pairs {
                let mut staging = Vec::with_capacity(self.transforms.len() * 64);
                for (slot, (transform, primitive)) in
                    self.transforms.iter().zip(&self.primitives).enumerate()
                {
                    staging.extend_from_slice(&primitive.to_le_bytes());
                    if primitive & TERRAIN_TAG != 0 {
                        let pool = self.terrain.as_ref().unwrap();
                        let (_, anchor) = &pool.active[slot - self.mesh_count];
                        staging.extend(f32_bytes(anchor.normal.map(|v| v as f32)));
                    } else {
                        staging.extend_from_slice(&[0; 12]);
                    }
                    for row in 0..3 {
                        for value in &transform[row * 4..row * 4 + 4] {
                            staging.extend_from_slice(&value.to_le_bytes());
                        }
                    }
                }
                if !staging.is_empty() {
                    self.queue.write_buffer(pairs, 0, &staging);
                }
            }
            self.previous.clear();
            self.previous.extend_from_slice(&self.transforms);
            self.previous_primitives.clear();
            self.previous_primitives.extend_from_slice(&self.primitives);
            self.dirty = true;
        }
    }

    /// Build the BLAS once and the TLAS whenever instances changed. Must run
    /// in the frame encoder before the mesh pass. Terrain-only scenes can
    /// become empty: build an empty TLAS so primary receivers can safely miss.
    pub fn encode(&mut self, encoder: &mut wgpu::CommandEncoder) -> u32 {
        self.passes = 0;
        self.terrain_blas_builds = 0;
        self.prepare();
        let build_blas = !self.built;
        if !build_blas && !self.dirty {
            return 0;
        }
        {
            let mut entries = if build_blas {
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
                                vertex_stride: self.vertex_stride,
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
            if let Some(terrain) = &self.terrain {
                self.terrain_blas_builds = terrain.dirty.len() as u32;
                entries.extend(terrain.builds());
            }
            encoder.build_acceleration_structures(entries.iter(), [&self.tlas]);
        }
        self.built = true;
        self.dirty = false;
        if let Some(terrain) = &mut self.terrain {
            terrain.mark_built();
        }
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
