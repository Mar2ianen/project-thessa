//! Owned CBT geometry → visibility/LOD/compaction → indirect raster passes.
//!
//! Uses the production portable WGSL, packed HeightPage and TileAnchor ABI;
//! no Bevy dependency or CPU mesh construction. This first consumer supports
//! a body-fixed camera, streamed material pages and a fallback globe color,
//! not the full game scene.

use thessa_rcbt_core::material_cache::SlotCache;
use thessa_rcbt_core::{HeightPage, shaders, tile_anchor::TileAnchor};
use wgpu::util::DeviceExt;

use crate::{
    Backend, BackendError,
    view::{ViewParams, ViewUniformBlock},
};

pub const VERTICES_PER_PATCH: u32 = shaders::VERTICES_PER_PATCH as u32;
const TRIANGLES_PER_PATCH: u64 = shaders::TRIANGLES_PER_PATCH as u64;
pub(crate) const FALLBACK_ALBEDO: [u8; 4] = [100, 130, 70, 255];

/// Borrowed source pages. Quantized heights are uploaded without expansion.
pub struct TerrainPatch<'a> {
    pub anchor: TileAnchor,
    pub page: &'a HeightPage,
}

/// Stable source identity; generations change whenever a page's bytes change.
pub struct ResidentTerrainPatch<'a> {
    pub anchor: TileAnchor,
    pub page: &'a HeightPage,
    pub generation: u64,
}

#[derive(Clone, Copy, Debug)]
struct PatchShadowBound {
    center_body_m: glam::DVec3,
    radius_m: f64,
}

impl PatchShadowBound {
    fn new(anchor: &TileAnchor, page: &HeightPage) -> Option<Self> {
        let canonical = TileAnchor::new(anchor.tile, anchor.radius_m)?;
        // Public custom TileAnchor frames are admitted by the renderer, but
        // without a canonical geometric bound they must fail open here.
        if anchor.anchor_body_m != canonical.anchor_body_m
            || anchor.to_gpu([0.0; 3])? != canonical.to_gpu([0.0; 3])?
        {
            return None;
        }
        // On a cube face |raw|>=1: normalization is 1-Lipschitz along the
        // center-to-sample segment; |delta|<=sqrt(2)*half_extent. Encloses the
        // whole curved surface, unlike a sampled nine-point AABB. Bound actual
        // residual decoding/lerps, not original pre-quantization height extrema.
        let height = (f64::from(page.base_height_m()).abs()
            + 32768.0 * f64::from(page.residual_scale_m()).abs())
            * (1.0 + 32.0 * f64::from(f32::EPSILON));
        let scale = canonical.radius_m + height;
        if !(128.0 * f64::from(f32::MIN_POSITIVE)..f64::from(f32::MAX) / 128.0).contains(&scale) {
            return None;
        }
        let radius_m = std::f64::consts::SQRT_2 * canonical.radius_m * canonical.half_extent
            + height
            + 64.0 * f64::from(f32::EPSILON) * scale;
        Some(Self {
            center_body_m: glam::DVec3::from_array(canonical.anchor_body_m),
            radius_m,
        })
    }

    fn intersects(&self, camera: [f64; 3], matrix: glam::Mat4) -> bool {
        let relative = self.center_body_m - glam::DVec3::from_array(camera);
        // Anchor/local reconstruction uncertainty, then the independently
        // bounded f32 clip dot products. Larger error retains more casters.
        let rounding = 64.0 * f64::from(f32::EPSILON) * relative.abs().max_element();
        thessa_render_core::shadow::sphere_intersects_clip(
            matrix.as_dmat4(),
            relative,
            self.radius_m + rounding,
        )
    }
}

#[derive(Debug, Clone, Copy)]
pub struct TerrainCapacity {
    pub patches: u32,
    pub height_grid_size: u32,
    pub material_layers: u32,
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct TerrainPublication {
    pub active: usize,
    pub regenerated: usize,
    pub retained: usize,
    pub dispatch_slots: usize,
}

/// Static R8 optical-mask residency, not authoritative settlement state or
/// emitted watts. A missing source stays dark even when allocation is enabled.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct NightLightsStats {
    pub extent: Option<[u32; 2]>,
    pub uploaded_bytes: u64,
    pub storage_bytes: u64,
}

#[derive(PartialEq)]
struct ShadowCacheKey {
    lights: u64,
    meshes: Option<(wgpu::Buffer, u64)>,
    surface: u64,
}

/// Persistent GPU terrain working set. Regular-patch geometry is generated at
/// page publication; opt-in conforming geometry is emitted at encode_surface.
pub struct TerrainRenderer {
    motion_vectors: bool,
    topology: Option<crate::terrain_topology::TerrainTopology>,
    topology_reason: Option<String>,
    ocean: Option<crate::ocean::OceanField>,
    ocean_geometry: Option<crate::ocean::OceanGeometry>,
    ocean_bind: Option<wgpu::BindGroup>,
    ocean_environment: Option<crate::environment::EnvironmentBinding>,
    land_environment_samples: u32,
    ocean_settings: thessa_graphics::OceanSettings,
    ocean_dirty: bool,
    ocean_last_camera: Option<[f64; 3]>,
    ocean_reason: Option<String>,
    ocean_possible: Vec<Option<(f64, f64)>>,
    /// Per-slot resident height extent (min, max) in body metres, including
    /// each page's residual error bound. Drives the conservative CBT LOD
    /// height bound; independent of transient render refinement.
    resident_height_extent: Vec<Option<(f64, f64)>>,
    shadow_cache: Option<ShadowCacheKey>,
    last_shadow_passes: usize,
    last_shadow_draws: crate::shadows::ShadowDrawStats,
    sky: Option<crate::sky::SkyBackground>,
    sky_visible: bool,
    /// Enabled plume paths; disabled settings drop everything, frames without
    /// an active engine only skip the draws.
    plume: Option<crate::plume::PlumeRenderer>,
    format: wgpu::TextureFormat,
    shadows: Option<crate::shadows::ShadowAtlas>,
    shadow_receiver: Option<wgpu::BindGroup>,
    shadow_caster: Option<wgpu::BindGroup>,
    shadow_bounds: Vec<Option<PatchShadowBound>>,
    residency: SlotCache,
    manual_updates: Vec<bool>,
    active_extent: usize,
    camera: [f64; 3],
    body_radius_m: f64,
    leaves: wgpu::Buffer,
    history: wgpu::Buffer,
    lighting: wgpu::Buffer,
    materials: crate::material::MaterialArray,
    node_ids: Vec<u64>,
    ray_revisions: Vec<u64>,
    anchors: Vec<TileAnchor>,
    frames: wgpu::Buffer,
    frame_staging: Vec<u8>,
    view: wgpu::Buffer,
    /// Separate primary projection only in the opt-in jitter diagnostic. The
    /// classifier, topology policy and clear sky retain `view` without jitter.
    primary_view: Option<wgpu::Buffer>,
    vertices: wgpu::Buffer,
    draw: wgpu::Buffer,
    geometry: wgpu::ComputePipeline,
    geometry_bind: wgpu::BindGroup,
    params: wgpu::Buffer,
    dirty: wgpu::Buffer,
    metadata: wgpu::Buffer,
    surface_source: Option<crate::conforming_surface::HeightSurfaceSource>,
    residuals: wgpu::Buffer,
    page_slots: Vec<(u64, u64)>,
    radius: f32,
    classify: [wgpu::ComputePipeline; 3],
    classify_bind: wgpu::BindGroup,
    raster: wgpu::RenderPipeline,
    raster_bind: wgpu::BindGroup,
    night_mask: Option<wgpu::Texture>,
    night_intensity: f32,
    night_uploaded_bytes: u64,
}

fn bytes(words: impl IntoIterator<Item = u32>) -> Vec<u8> {
    words.into_iter().flat_map(u32::to_le_bytes).collect()
}

fn upload(
    device: &wgpu::Device,
    label: &str,
    data: &[u8],
    usage: wgpu::BufferUsages,
) -> wgpu::Buffer {
    device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some(label),
        contents: data,
        usage,
    })
}

fn storage(device: &wgpu::Device, size: u64, extra: wgpu::BufferUsages) -> wgpu::Buffer {
    device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("cbt-working-set"),
        size,
        usage: wgpu::BufferUsages::STORAGE | extra,
        mapped_at_creation: false,
    })
}

fn buffer_layout(
    device: &wgpu::Device,
    count: u32,
    uniforms: &[u32],
    writable: &[u32],
) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("cbt-compute-layout"),
        entries: &(0..count)
            .map(|binding| wgpu::BindGroupLayoutEntry {
                binding,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: if uniforms.contains(&binding) {
                        wgpu::BufferBindingType::Uniform
                    } else {
                        wgpu::BufferBindingType::Storage {
                            read_only: !writable.contains(&binding),
                        }
                    },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            })
            .collect::<Vec<_>>(),
    })
}

fn bind_buffers(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    buffers: &[&wgpu::Buffer],
) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("cbt-compute-bind"),
        layout,
        entries: &buffers
            .iter()
            .enumerate()
            .map(|(binding, buffer)| wgpu::BindGroupEntry {
                binding: binding as u32,
                resource: buffer.as_entire_binding(),
            })
            .collect::<Vec<_>>(),
    })
}

fn pipeline(
    device: &wgpu::Device,
    module: &wgpu::ShaderModule,
    layout: &wgpu::PipelineLayout,
    entry: &str,
    constants: &[(&str, f64)],
) -> wgpu::ComputePipeline {
    device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some(entry),
        layout: Some(layout),
        module,
        entry_point: Some(entry),
        compilation_options: wgpu::PipelineCompilationOptions {
            constants,
            ..Default::default()
        },
        cache: None,
    })
}

impl TerrainRenderer {
    /// Construct on an owned backend. All source tiles must belong to the same
    /// body/radius. Bounds are checked before creating any GPU buffers.
    pub fn new(
        backend: &Backend,
        patches: &[TerrainPatch<'_>],
        format: wgpu::TextureFormat,
    ) -> Result<Self, BackendError> {
        Self::with_material_capacity(backend, patches, format, 1)
    }

    /// Allocate a bounded raw material working set. Capacity is explicit rather
    /// than an adapter-dependent hidden allocation; each layer holds eight mips.
    pub fn with_material_capacity(
        backend: &Backend,
        patches: &[TerrainPatch<'_>],
        format: wgpu::TextureFormat,
        material_layers: u32,
    ) -> Result<Self, BackendError> {
        Self::allocate(
            backend,
            patches,
            format,
            material_layers,
            patches.len(),
            None,
            None,
            None,
        )
    }

    /// Allocate a reusable geometry pool; unused slots are not dispatched.
    /// Height slots have a fixed maximum grid capacity and never grow in-frame.
    pub fn with_capacity(
        backend: &Backend,
        patches: &[TerrainPatch<'_>],
        format: wgpu::TextureFormat,
        capacity: TerrainCapacity,
    ) -> Result<Self, BackendError> {
        Self::with_capacity_and_lighting(backend, patches, format, capacity, None)
    }

    pub fn with_capacity_and_lighting(
        backend: &Backend,
        patches: &[TerrainPatch<'_>],
        format: wgpu::TextureFormat,
        capacity: TerrainCapacity,
        settings: Option<&thessa_graphics::ResolvedGraphicsSettings>,
    ) -> Result<Self, BackendError> {
        Self::with_capacity_lighting_and_ocean(backend, patches, format, capacity, settings, None)
    }

    pub fn with_capacity_lighting_and_ocean(
        backend: &Backend,
        patches: &[TerrainPatch<'_>],
        format: wgpu::TextureFormat,
        capacity: TerrainCapacity,
        settings: Option<&thessa_graphics::ResolvedGraphicsSettings>,
        sea: Option<thessa_render_core::ocean::SeaState>,
    ) -> Result<Self, BackendError> {
        if capacity.height_grid_size < 2 || capacity.height_grid_size > 4096 {
            return Err(BackendError::Device(
                "invalid terrain height slot grid".into(),
            ));
        }
        let page_bytes = (u64::from(capacity.height_grid_size).pow(2) * 2).div_ceil(4) * 4;
        Self::allocate(
            backend,
            patches,
            format,
            capacity.material_layers,
            capacity.patches as usize,
            Some(page_bytes),
            settings,
            sea,
        )
    }

    // One allocation boundary with independent source/capacity and optional
    // physical sea inputs; not a second configuration model.
    #[allow(clippy::too_many_arguments)]
    fn allocate(
        backend: &Backend,
        patches: &[TerrainPatch<'_>],
        format: wgpu::TextureFormat,
        material_layers: u32,
        capacity: usize,
        page_capacity: Option<u64>,
        settings: Option<&thessa_graphics::ResolvedGraphicsSettings>,
        sea: Option<thessa_render_core::ocean::SeaState>,
    ) -> Result<Self, BackendError> {
        let device = &backend.device;
        let topology_requested = settings.is_some_and(|s| s.terrain_topology.capacity > 0);
        let topology_supported = device.limits().max_storage_buffers_per_shader_stage >= 8
            && settings.is_none_or(|s| {
                u64::from(s.terrain_topology.capacity) * 144
                    <= device.limits().max_storage_buffer_binding_size
            });
        let topology_enabled = topology_requested && topology_supported;
        let shadows = if let Some(settings) = settings {
            crate::shadows::ShadowAtlas::new(backend, settings)?
        } else {
            None
        };
        let fail = |s: &str| BackendError::Device(s.into());
        let Some(first) = patches.first() else {
            return Err(fail("terrain needs at least one patch"));
        };
        let radius = first.anchor.radius_m;
        let ocean_settings = settings.map(|s| s.ocean.clone()).unwrap_or_default();
        let ocean = if ocean_settings.enabled {
            if ocean_settings.geometry_patches > 64
                || !(0.0..=20_000.0).contains(&ocean_settings.geometry_distance_m)
            {
                return Err(fail("invalid ocean near-field capacity/distance"));
            }
            let sea = sea
                .ok_or_else(|| fail("spectral ocean requires explicit local gravity/sea state"))?;
            Some(crate::ocean::OceanField::new(
                backend,
                sea,
                settings.unwrap().ocean_size,
            )?)
        } else {
            None
        };
        let count = capacity as u64;
        let limits = device.limits();
        let max_storage = limits
            .max_storage_buffer_binding_size
            .min(limits.max_buffer_size);
        let vertex_bytes = if topology_enabled {
            32
        } else {
            count
                .checked_mul(VERTICES_PER_PATCH as u64 * 32)
                .ok_or_else(|| fail("terrain size overflow"))?
        };
        let triangle_bytes = if topology_enabled {
            16
        } else {
            count
                .checked_mul(TRIANGLES_PER_PATCH * 16)
                .ok_or_else(|| fail("terrain size overflow"))?
        };
        let residual_bytes = if let Some(size) = page_capacity {
            count.checked_mul(size)
        } else {
            patches.iter().try_fold(0u64, |sum, p| {
                sum.checked_add((p.page.residuals().len() as u64).div_ceil(2) * 4)
            })
        }
        .ok_or_else(|| fail("terrain residual size overflow"))?;
        if capacity < patches.len()
            || capacity == 0
            || patches.iter().any(|p| {
                page_capacity
                    .is_some_and(|size| (p.page.residuals().len() as u64).div_ceil(2) * 4 > size)
            })
            || count > u32::MAX as u64 / VERTICES_PER_PATCH as u64
            || count > limits.max_compute_workgroups_per_dimension as u64
            || (count * VERTICES_PER_PATCH as u64).div_ceil(64)
                > limits.max_compute_workgroups_per_dimension as u64
            || vertex_bytes > max_storage
            || triangle_bytes > max_storage
            || residual_bytes > max_storage
            || limits.max_storage_buffers_per_shader_stage < 8
            || patches
                .iter()
                .any(|p| p.anchor.radius_m != radius || tile_node_id(&p.anchor).is_none())
        {
            return Err(fail(
                "terrain working set exceeds device limits or mixes body radii",
            ));
        }
        let mut anchors: Vec<_> = patches.iter().map(|p| p.anchor).collect();
        anchors.resize(capacity, first.anchor);
        let frames_data = frame_bytes(&anchors, [0.0; 3])?;
        let normal_maps = settings.is_none_or(|s| s.terrain_normal_maps);
        let materials =
            crate::material::MaterialArray::new(backend, material_layers, capacity, normal_maps)?;
        let mut node_ids = Vec::with_capacity(capacity);
        let mut residuals = Vec::with_capacity(residual_bytes as usize);
        let mut page_slots = Vec::with_capacity(patches.len());
        let mut metadata = Vec::with_capacity(patches.len() * 4);
        let mut leaves = Vec::with_capacity(patches.len() * 4);
        for (ordinal, p) in patches.iter().enumerate() {
            let key = p.anchor.tile;
            let id = tile_node_id(&p.anchor).ok_or_else(|| fail("invalid terrain tile"))?;
            node_ids.push(id);
            leaves.extend([
                id as u32,
                (id >> 32) as u32,
                3 + 2 * u32::from(key.level),
                ordinal as u32,
            ]);
            metadata.extend([
                p.page.base_height_m().to_bits(),
                p.page.residual_scale_m().to_bits(),
                p.page.grid_size(),
                (residuals.len() / 4) as u32,
            ]);
            page_slots.push((
                residuals.len() as u64,
                page_capacity.unwrap_or((p.page.residuals().len() as u64).div_ceil(2) * 4),
            ));
            residuals.extend(p.page.residual_bytes());
            residuals.resize(residuals.len().div_ceil(4) * 4, 0);
            if let Some(size) = page_capacity {
                residuals.resize((page_slots[ordinal].0 + size) as usize, 0);
            }
        }
        let distinct: std::collections::BTreeSet<_> = node_ids.iter().copied().collect();
        if distinct.len() != patches.len() {
            return Err(fail("duplicate terrain tile"));
        }
        let mut residency = SlotCache::new(capacity).map_err(|e| fail(&e.to_string()))?;
        residency.update(&node_ids.iter().map(|id| (*id, 0)).collect::<Vec<_>>());
        for _ in patches.len()..capacity {
            let size = page_capacity.expect("spare capacity needs bounded height slots");
            page_slots.push((residuals.len() as u64, size));
            residuals.resize(residuals.len() + size as usize, 0);
        }
        node_ids.resize(capacity, 0);
        leaves.resize(capacity * 4, 0);
        metadata.resize(capacity * 4, 0);
        let ro = wgpu::BufferUsages::STORAGE;
        let leaves = upload(
            device,
            "cbt-leaves",
            &bytes(leaves),
            ro | wgpu::BufferUsages::COPY_DST,
        );
        let metadata = upload(
            device,
            "cbt-pages",
            &bytes(metadata),
            ro | wgpu::BufferUsages::COPY_DST,
        );
        let residuals = upload(
            device,
            "cbt-residuals",
            &residuals,
            ro | wgpu::BufferUsages::COPY_DST,
        );
        let frames = upload(
            device,
            "cbt-tile-frames",
            &frames_data,
            ro | wgpu::BufferUsages::COPY_DST,
        );
        let ray_usage = if settings.is_some_and(|s| s.rt_terrain)
            && device
                .features()
                .contains(wgpu::Features::EXPERIMENTAL_RAY_QUERY)
        {
            wgpu::BufferUsages::BLAS_INPUT
        } else {
            wgpu::BufferUsages::empty()
        };
        let vertices = storage(
            device,
            vertex_bytes,
            wgpu::BufferUsages::COPY_SRC
                | if ocean.is_some() {
                    wgpu::BufferUsages::COPY_DST
                } else {
                    wgpu::BufferUsages::empty()
                }
                | ray_usage,
        );
        let patch_records = storage(device, count * 16, wgpu::BufferUsages::empty());
        let params_data = bytes([
            patches.len() as u32,
            VERTICES_PER_PATCH,
            (radius as f32).to_bits(),
            patches.len() as u32,
        ]);
        let params = upload(
            device,
            "cbt-params",
            &params_data,
            wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        );
        let dirty = upload(
            device,
            "cbt-initial-dirty",
            &bytes((0..capacity as u32).map(|i| if (i as usize) < patches.len() { i } else { 0 })),
            ro | wgpu::BufferUsages::COPY_DST,
        );
        let geometry_layout = buffer_layout(device, 8, &[5], &[1, 4]);
        let geometry_bind = bind_buffers(
            device,
            &geometry_layout,
            &[
                &leaves,
                &patch_records,
                &metadata,
                &residuals,
                &vertices,
                &params,
                &frames,
                &dirty,
            ],
        );
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("cbt-geometry"),
            bind_group_layouts: &[Some(&geometry_layout)],
            immediate_size: 0,
        });
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("cbt-geometry"),
            source: wgpu::ShaderSource::Wgsl(shaders::CBT_GEOMETRY_WGSL.into()),
        });
        let geometry = pipeline(device, &module, &layout, "build_geometry", &[]);
        let mut encoder = device.create_command_encoder(&Default::default());
        if !topology_enabled {
            let mut pass = encoder.begin_compute_pass(&Default::default());
            pass.set_pipeline(&geometry);
            pass.set_bind_group(0, &geometry_bind, &[]);
            pass.dispatch_workgroups(
                (patches.len() as u32 * VERTICES_PER_PATCH).div_ceil(64),
                1,
                1,
            );
        }
        backend.queue.submit([encoder.finish()]);
        let view = upload(
            device,
            "cbt-view",
            &[0; ViewUniformBlock::BYTE_LEN],
            wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        );
        let primary_view = settings.filter(|s| s.temporal_jitter).map(|_| {
            upload(
                device,
                "cbt-primary-view",
                &[0; ViewUniformBlock::BYTE_LEN],
                wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            )
        });
        let transform = upload(
            device,
            "cbt-body-rotation",
            &bytes(glam::Mat4::IDENTITY.to_cols_array().map(f32::to_bits)),
            wgpu::BufferUsages::UNIFORM,
        );
        let triangles = storage(device, triangle_bytes, wgpu::BufferUsages::empty());
        let active = storage(device, 16, wgpu::BufferUsages::empty());
        let draw = storage(
            device,
            32,
            wgpu::BufferUsages::INDIRECT | wgpu::BufferUsages::COPY_SRC,
        );
        let history = storage(device, count * 16, wgpu::BufferUsages::COPY_DST);
        let classify_layout = buffer_layout(device, 11, &[5, 6, 7], &[2, 3, 4, 10]);
        let classify_bind = bind_buffers(
            device,
            &classify_layout,
            &[
                &metadata, &vertices, &triangles, &active, &draw, &view, &transform, &params,
                &leaves, &frames, &history,
            ],
        );
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("cbt-classify"),
            bind_group_layouts: &[Some(&classify_layout)],
            immediate_size: 0,
        });
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("cbt-classify"),
            source: wgpu::ShaderSource::Wgsl(shaders::CBT_CLASSIFY_WGSL.into()),
        });
        let classify = ["reset_active", "classify_active", "finalize_active"].map(|entry| {
            pipeline(
                device,
                &module,
                &layout,
                entry,
                &[(
                    "cbt_displacement_bound_m",
                    ocean.as_ref().map_or(0.0, |f| f.stats().height_bound_m),
                )],
            )
        });
        let reflections = settings.is_none_or(|s| s.specular_reflections);
        let land_environment_samples = settings.map_or(16, |s| s.terrain_environment_samples());
        let [cloud_directions, cloud_steps, cloud_sun_steps] =
            settings.map_or([4, 4, 1], |s| s.terrain_environment_cloud_budget());
        let topology_reason = (topology_requested && !topology_supported).then(|| {
            "conforming terrain exceeds negotiated device limits; using regular patches".into()
        });
        let mut resident_height_extent = Vec::with_capacity(capacity);
        resident_height_extent.extend(patches.iter().map(|p| Some(height_extent(p.page))));
        resident_height_extent.resize(capacity, None);
        let surface_source = if topology_requested && topology_supported {
            Some(crate::conforming_surface::HeightSurfaceSource::new(
                backend,
                &metadata,
                &residuals,
                radius,
                &node_ids,
                &resident_height_extent,
            )?)
        } else {
            None
        };
        let topology = if let Some(height) = &surface_source {
            Some(crate::terrain_topology::TerrainTopology::new(
                backend,
                settings.unwrap(),
                height,
                &materials.binding(),
                ocean.as_ref(),
                shadows.as_ref(),
            )?)
        } else {
            None
        };
        let mut hooks = format!(
            "fn cbt_shadow_visibility(p:vec3<f32>,n:vec3<f32>,sun:u32)->f32 {{ return shadow_visibility(p,n,sun)*cloud_terrain_sun_visibility(p,sun); }}\nfn cbt_specular_enabled()->bool {{ return {reflections}; }}\n"
        );
        hooks.push_str(&if reflections {
            format!("fn cbt_land_environment(p:vec3<f32>,n:vec3<f32>,v:vec3<f32>,r:f32,a:vec3<f32>)->vec4<f32>{{let response=env_specular_response(p,n,v,r,vec3(0.04),a,{land_environment_samples}u,vec3<u32>({cloud_directions}u,{cloud_steps}u,{cloud_sun_steps}u));return vec4(response.radiance,response.reflectance.r);}}\nfn cbt_land_transmission(n:vec3<f32>,l:vec3<f32>,r:f32)->f32{{return 1.0-env_directional_reflectance(n,l,r,vec3(0.04),{land_environment_samples}u).r;}}\nfn cbt_land_ambient_transmission(r:f32)->f32{{return 1.0-env_average_reflectance(r,vec3(0.04),{land_environment_samples}u).r;}}\n")
        } else {
            "fn cbt_land_environment(p:vec3<f32>,n:vec3<f32>,v:vec3<f32>,r:f32,a:vec3<f32>)->vec4<f32>{return vec4(0.0);}\nfn cbt_land_transmission(n:vec3<f32>,l:vec3<f32>,r:f32)->f32{return 1.0;}\nfn cbt_land_ambient_transmission(r:f32)->f32{return 1.0;}\n".into()
        });
        hooks.push_str(if ocean.is_some(){include_str!("ocean_raster.wgsl")}else{
            "fn cbt_spectral_ocean(p:vec3<f32>,n:vec3<f32>,d:vec3<f32>,h:f32,r:f32,a:vec3<f32>,f:f32)->vec4<f32>{return vec4(0.0);}"
        });
        if ocean.is_some() {
            hooks.push_str(&crate::ocean::shader(2));
        }
        if ocean.is_some() || reflections {
            hooks.push_str(&crate::environment::shader(2));
        } else {
            hooks.push_str(&crate::clouds::SHADER.replace("@group(0)", "@group(2)"));
        }
        // Displacement samples material at vertices, independently of this
        // fragment's filtered material. Conservatively mask the whole height
        // admission band, including rough/dry fallback fragments at its edges.
        let temporal_alpha = if settings.is_some_and(|s| s.temporal_motion_vectors) {
            format!(
                "return vec4(radiance * view.exposure, select(1.0, 0.0, max(water_material, spectral_ocean.a) > 0.0{}));",
                if ocean.is_some() && ocean_settings.geometry_patches > 0 {
                    " || input.height_m < 1.0"
                } else {
                    ""
                }
            )
        } else {
            "return vec4(radiance * view.exposure, 1.0);".into()
        };
        let raster_source = shaders::CBT_RASTER_WGSL
            // Only the opt-in native diagnostic repurposes scene alpha. The
            // shared/legacy shader and disabled native output remain intact.
            .replace(
                "return vec4(radiance * view.exposure, 1.0);",
                &temporal_alpha,
            )
            .replacen(shaders::RASTER_SURFACE_HOOKS_WGSL, &hooks, 1)
            .replace(
                "const MATERIAL_NORMALS_ENABLED: bool = false;",
                &format!("const MATERIAL_NORMALS_ENABLED: bool = {normal_maps};"),
            )
            .replace(
                "const MATERIAL_NORMAL_MIP_BIAS: f32 = 0.0;",
                &format!(
                    "const MATERIAL_NORMAL_MIP_BIAS: f32 = {}.0;",
                    settings.map_or(0, |s| s.terrain_normal_mip_bias())
                ),
            );
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("cbt-raster"),
            source: wgpu::ShaderSource::Wgsl(
                format!(
                    "{}\n{}",
                    shadows.as_ref().map_or_else(
                        || crate::shadows::NO_SHADOWS.into(),
                        |_| crate::shadows::receiver_shader(1)
                    ),
                    raster_source
                )
                .into(),
            ),
        });
        let raster = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("cbt-raster"),
            layout: None,
            vertex: wgpu::VertexState {
                module: &module,
                entry_point: Some("vertex"),
                compilation_options: wgpu::PipelineCompilationOptions {
                    constants: &[("CBT_CONFORMING", f64::from(topology.is_some()))],
                    ..Default::default()
                },
                buffers: &[],
            },
            primitive: wgpu::PrimitiveState {
                cull_mode: if topology.is_some() {
                    Some(wgpu::Face::Back)
                } else {
                    None
                },
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: Some(true),
                depth_compare: Some(wgpu::CompareFunction::Less),
                stencil: Default::default(),
                bias: Default::default(),
            }),
            multisample: Default::default(),
            fragment: Some(wgpu::FragmentState {
                module: &module,
                entry_point: Some("fragment"),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            multiview_mask: None,
            cache: None,
        });
        // Globe asset loading and effect settings remain in the client.
        let globe = solid_texture(backend, FALLBACK_ALBEDO, false);
        let roughness = solid_texture(backend, [255; 4], false);
        let black = solid_texture(backend, [0; 4], false);
        let night_mask = settings
            .and_then(|s| s.night_light_mask_size())
            .map(|[width, height]| {
                device.create_texture(&wgpu::TextureDescriptor {
                    label: Some("terrain-night-mask"),
                    size: wgpu::Extent3d {
                        width,
                        height,
                        depth_or_array_layers: 1,
                    },
                    mip_level_count: 1,
                    sample_count: 1,
                    dimension: wgpu::TextureDimension::D2,
                    format: wgpu::TextureFormat::R8Unorm,
                    usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
                    view_formats: &[],
                })
            });
        let night_view = night_mask
            .as_ref()
            .map(|t| t.create_view(&Default::default()));
        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            address_mode_u: wgpu::AddressMode::Repeat,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            ..Default::default()
        });
        let mut light = [0.0_f32; 56];
        light[..3].fill(0.15);
        light[4..7].copy_from_slice(&[0.8, 0.5, 0.3]);
        light[16..19].fill(2.5);
        let lighting = upload(
            device,
            "cbt-lighting",
            &bytes(light.map(f32::to_bits)),
            wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        );
        let raster_layout = raster.get_bind_group_layout(0);
        let entries = [
            (
                0,
                primary_view.as_ref().unwrap_or(&view).as_entire_binding(),
            ),
            (1, transform.as_entire_binding()),
            (
                2,
                topology
                    .as_ref()
                    .map_or(&vertices, |t| &t.surface.vertices)
                    .as_entire_binding(),
            ),
            (3, metadata.as_entire_binding()),
            (4, params.as_entire_binding()),
            (5, triangles.as_entire_binding()),
            (6, wgpu::BindingResource::TextureView(&globe)),
            (7, wgpu::BindingResource::Sampler(&sampler)),
            (8, leaves.as_entire_binding()),
            (9, lighting.as_entire_binding()),
            (10, wgpu::BindingResource::TextureView(&roughness)),
            (11, frames.as_entire_binding()),
            (12, wgpu::BindingResource::TextureView(&materials.view)),
            (13, wgpu::BindingResource::Sampler(&materials.sampler)),
            (14, materials.slots.as_entire_binding()),
            (16, materials.directory.as_entire_binding()),
            (17, wgpu::BindingResource::TextureView(&black)),
            (
                18,
                wgpu::BindingResource::TextureView(night_view.as_ref().unwrap_or(&black)),
            ),
            (
                19,
                wgpu::BindingResource::TextureView(&materials.normal_view),
            ),
            (20, materials.normal_directory.as_entire_binding()),
        ]
        .into_iter()
        .map(|(binding, resource)| wgpu::BindGroupEntry { binding, resource })
        .collect::<Vec<_>>();
        let raster_bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("cbt-raster-bind"),
            layout: &raster_layout,
            entries: &entries,
        });
        let environment = crate::environment::EnvironmentBinding::new(device);
        let ocean_bind = Some({
            let mut entries = ocean
                .as_ref()
                .map_or_else(Vec::new, |field| field.binding().entries().to_vec());
            if ocean.is_some() || reflections {
                entries.extend(environment.entries());
            } else {
                entries.extend(environment.cloud_entries());
            }
            device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("ocean-raster"),
                layout: &raster.get_bind_group_layout(2),
                entries: &entries,
            })
        });
        let ocean_environment = Some(environment);
        let ocean_geometry = if let Some(field) = &ocean
            && ocean_settings.geometry_patches > 0
            && topology.is_none()
        {
            Some(crate::ocean::OceanGeometry::new(
                backend,
                ocean_settings.geometry_patches as usize,
                &vertices,
                &frames,
                &materials,
                field,
            ))
        } else {
            None
        };
        Ok(Self {
            topology,
            topology_reason,
            ocean,
            ocean_geometry,
            ocean_bind,
            ocean_environment,
            land_environment_samples,
            ocean_settings,
            ocean_dirty: true,
            ocean_last_camera: None,
            ocean_reason: None,
            ocean_possible: (0..if settings.is_some_and(|s| s.ocean.enabled) {
                capacity
            } else {
                0
            })
                .map(|i| patches.get(i).and_then(|p| ocean_height_range(p.page)))
                .collect(),
            resident_height_extent,
            shadow_cache: None,
            last_shadow_passes: 0,
            last_shadow_draws: Default::default(),
            sky: None,
            plume: None,
            motion_vectors: settings.is_some_and(|s| s.temporal_motion_vectors),
            sky_visible: settings.is_none_or(|s| s.sky_dome),
            format,
            shadow_receiver: shadows
                .as_ref()
                .map(|s| s.receiver(backend, &raster.get_bind_group_layout(1))),
            shadow_caster: shadows.as_ref().map(|s| {
                device.create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some("terrain-shadow-source"),
                    layout: &s.terrain_layout,
                    entries: &[
                        wgpu::BindGroupEntry {
                            binding: 0,
                            resource: vertices.as_entire_binding(),
                        },
                        wgpu::BindGroupEntry {
                            binding: 1,
                            resource: frames.as_entire_binding(),
                        },
                        wgpu::BindGroupEntry {
                            binding: 2,
                            resource: metadata.as_entire_binding(),
                        },
                        wgpu::BindGroupEntry {
                            binding: 3,
                            resource: transform.as_entire_binding(),
                        },
                    ],
                })
            }),
            shadow_bounds: {
                let mut bounds = Vec::new();
                if shadows.is_some() {
                    bounds.reserve_exact(capacity);
                    bounds.extend(
                        patches
                            .iter()
                            .map(|p| PatchShadowBound::new(&p.anchor, p.page)),
                    );
                    bounds.resize(capacity, None);
                }
                bounds
            },
            shadows,
            residency,
            manual_updates: vec![false; capacity],
            active_extent: patches.len(),
            camera: [0.0; 3],
            body_radius_m: radius,
            leaves,
            history,
            lighting,
            materials,
            node_ids,
            ray_revisions: vec![1; capacity],
            anchors,
            frames,
            frame_staging: frames_data,
            view,
            primary_view,
            vertices,
            draw,
            geometry,
            geometry_bind,
            params,
            dirty,
            metadata,
            surface_source,
            residuals,
            page_slots,
            radius: radius as f32,
            classify,
            classify_bind,
            raster,
            raster_bind,
            night_mask,
            night_intensity: settings.map_or(0.0, |s| s.night_lights_intensity),
            night_uploaded_bytes: 0,
        })
    }

    /// Publish a complete material snapshot with independent ordered demand.
    /// Stable generations/priority reordering do not upload texture pages.
    pub fn update_materials(
        &mut self,
        backend: &Backend,
        demand: &[u64],
        pages: &crate::material::MaterialPages,
    ) -> Result<crate::material::MaterialUpload, BackendError> {
        let upload = self
            .materials
            .prepare(backend, &self.node_ids, demand, pages)?;
        self.ocean_dirty |= upload.changed;
        if upload.changed
            && let Some(topology) = &self.topology
        {
            topology.mark_dirty();
        }
        Ok(upload)
    }

    /// Publish direct multi-source illumination; effects stay disabled. Only
    /// the shared seven-record lighting prefix changes, with atomic validation.
    pub fn update_lighting(
        &self,
        backend: &Backend,
        lighting: &thessa_rcbt_core::lighting::SurfaceLighting,
    ) -> Result<(), BackendError> {
        let records = lighting
            .records()
            .ok_or_else(|| BackendError::Device("invalid terrain lighting".into()))?;
        backend.queue.write_buffer(
            &self.lighting,
            0,
            &bytes(records.into_iter().flatten().map(f32::to_bits)),
        );
        Ok(())
    }

    /// Adopt a complete caller-owned optical mask. Dimensions follow resolved
    /// quality; validate before touching texture/uniform bytes. The same shader
    /// consumes it on regular and conforming primary terrain, without a pass.
    pub fn update_night_mask(
        &mut self,
        backend: &Backend,
        pixels: &[u8],
    ) -> Result<(), BackendError> {
        let texture = self
            .night_mask
            .as_ref()
            .ok_or_else(|| BackendError::Device("night mask is disabled".into()))?;
        if pixels.len() as u64 != u64::from(texture.width()) * u64::from(texture.height()) {
            return Err(BackendError::Device(
                "night mask byte length does not match resolved extent".into(),
            ));
        }
        backend.queue.write_texture(
            texture.as_image_copy(),
            pixels,
            wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(texture.width()),
                rows_per_image: Some(texture.height()),
            },
            texture.size(),
        );
        backend.queue.write_buffer(
            &self.lighting,
            52 * 4,
            &bytes([self.night_intensity, 1.0, 0.0, 0.0].map(f32::to_bits)),
        );
        self.night_uploaded_bytes += pixels.len() as u64;
        Ok(())
    }

    pub fn night_lights_stats(&self) -> NightLightsStats {
        NightLightsStats {
            extent: self.night_mask.as_ref().map(|t| [t.width(), t.height()]),
            uploaded_bytes: self.night_uploaded_bytes,
            storage_bytes: self
                .night_mask
                .as_ref()
                .map_or(0, |t| u64::from(t.width()) * u64::from(t.height())),
        }
    }

    pub fn shadows(&self) -> Option<&crate::shadows::ShadowAtlas> {
        self.shadows.as_ref()
    }

    /// Explicit ordered frame boundary: topology transactions, canonical datum
    /// and optional ocean deformation precede shadow/RT/raster consumption.
    /// Regular-patch mode is a no-op. Re-encoding advances the publication epoch.
    pub fn encode_surface(&self, encoder: &mut wgpu::CommandEncoder) {
        if let Some(topology) = &self.topology {
            topology.encode(encoder);
        }
    }
    pub fn conforming_surface(&self) -> Option<&crate::conforming_surface::ConformingSurface> {
        self.topology.as_ref().map(|t| &t.surface)
    }
    pub fn topology_storage_bytes(&self) -> u64 {
        self.topology.as_ref().map_or(0, |t| t.storage_bytes())
    }
    pub fn topology_reason(&self) -> Option<&str> {
        self.topology_reason.as_deref()
    }

    pub fn update_shadows(
        &mut self,
        backend: &Backend,
        camera: glam::DVec3,
        lights: &thessa_rcbt_core::lighting::SurfaceLighting,
    ) -> Result<(), BackendError> {
        if let Some(shadows) = &mut self.shadows {
            shadows.update(backend, camera, lights)?;
        }
        Ok(())
    }

    pub fn encode_shadows(
        &mut self,
        encoder: &mut wgpu::CommandEncoder,
        meshes: Option<&crate::mesh::MeshRenderer>,
    ) {
        self.encode_shadows_impl(encoder, meshes, true);
    }

    /// Diagnostic oracle: force the original full resident caster draw. This
    /// is not a graphics/quality toggle and never runs in production callers.
    pub fn encode_shadows_reference(
        &mut self,
        encoder: &mut wgpu::CommandEncoder,
        meshes: Option<&crate::mesh::MeshRenderer>,
    ) {
        self.invalidate_shadows();
        self.encode_shadows_impl(encoder, meshes, false);
        self.invalidate_shadows();
    }

    fn encode_shadows_impl(
        &mut self,
        encoder: &mut wgpu::CommandEncoder,
        meshes: Option<&crate::mesh::MeshRenderer>,
        cull: bool,
    ) {
        self.last_shadow_passes = 0;
        self.last_shadow_draws = Default::default();
        if let Some(shadows) = &self.shadows {
            let key = ShadowCacheKey {
                lights: shadows.depth_revision(),
                meshes: meshes.and_then(crate::mesh::MeshRenderer::shadow_key),
                surface: self
                    .topology
                    .as_ref()
                    .map_or(0, |t| t.surface.publication().revision),
            };
            if self.shadow_cache.as_ref() == Some(&key) {
                return;
            }
            self.last_shadow_draws = shadows.encode_casters(encoder, self, meshes, cull);
            self.last_shadow_passes = shadows.active_passes();
            self.shadow_cache = Some(key);
        }
    }

    /// Last encoded depth-layer draws (zero on an exact cached frame), unlike
    /// `ShadowAtlas::active_passes`, which counts potential active layers.
    pub fn last_shadow_passes(&self) -> usize {
        self.last_shadow_passes
    }

    pub fn last_shadow_draws(&self) -> crate::shadows::ShadowDrawStats {
        self.last_shadow_draws
    }

    /// Explicit invalidation for diagnostics/external source changes. Ordinary
    /// camera, page, membership, mesh and light updates invalidate themselves.
    pub fn invalidate_shadows(&mut self) {
        self.shadow_cache = None;
    }

    pub(crate) fn cast_shadow(
        &self,
        pass: &mut wgpu::RenderPass<'_>,
        atlas: &crate::shadows::ShadowAtlas,
        clip: Option<glam::Mat4>,
    ) -> crate::shadows::ShadowDrawStats {
        if let Some(topology) = &self.topology {
            if topology.surface.publication().revision == 0 {
                return Default::default();
            }
            if let Some(bind) = &topology.shadow {
                pass.set_pipeline(&atlas.conforming_pipeline);
                pass.set_bind_group(1, bind, &[]);
                pass.draw(0..topology.surface.slot_count() * 3, 0..1);
                return crate::shadows::ShadowDrawStats {
                    candidate_slots: u64::from(topology.surface.slot_count()),
                    submitted_slots: u64::from(topology.surface.slot_count()),
                    draw_calls: 1,
                };
            }
            return Default::default();
        }
        if self.active_extent == 0 {
            return Default::default();
        }
        let Some(bind) = &self.shadow_caster else {
            return Default::default();
        };
        pass.set_pipeline(&atlas.terrain_pipeline);
        pass.set_bind_group(1, bind, &[]);

        let mut stats = crate::shadows::ShadowDrawStats {
            candidate_slots: self.active_extent as u64,
            ..Default::default()
        };
        // Spectral geometry has its own deformation envelope and lifecycle.
        // Until integrated here, ocean-enabled terrain always draws in full.
        let Some(clip) = clip.filter(|_| self.ocean.is_none()) else {
            pass.draw(0..self.active_extent as u32 * 6144, 0..1);
            stats.submitted_slots = stats.candidate_slots;
            stats.draw_calls = 1;
            return stats;
        };
        let mut range_start: Option<u32> = None;
        for ordinal in 0..self.active_extent {
            let visible = self.node_ids[ordinal] != 0
                && self
                    .shadow_bounds
                    .get(ordinal)
                    .and_then(Option::as_ref)
                    .is_none_or(|bound| bound.intersects(self.camera, clip));
            if visible {
                range_start.get_or_insert(ordinal as u32);
                stats.submitted_slots += 1;
            } else if let Some(start) = range_start.take() {
                pass.draw(start * 6144..ordinal as u32 * 6144, 0..1);
                stats.draw_calls += 1;
            }
        }
        if let Some(start) = range_start {
            pass.draw(start * 6144..self.active_extent as u32 * 6144, 0..1);
            stats.draw_calls += 1;
        }
        stats
    }

    /// Replace the active partition atomically after CPU validation. Retained
    /// tiles keep slots/history; reuse clears history and inactive metadata.
    /// No pipelines/buffers are recreated, no clean geometry is regenerated.
    pub fn publish_partition(
        &mut self,
        backend: &Backend,
        patches: &[ResidentTerrainPatch<'_>],
    ) -> Result<TerrainPublication, BackendError> {
        let fail =
            || BackendError::Device("invalid, duplicate or over-capacity terrain partition".into());
        if patches.len() > self.anchors.len() {
            return Err(fail());
        }
        let mut requested = Vec::with_capacity(patches.len());
        let mut sources = std::collections::BTreeMap::new();
        for (index, patch) in patches.iter().enumerate() {
            if patch.anchor.radius_m != self.body_radius_m {
                return Err(fail());
            }
            if self.surface_source.is_some() && !valid_surface_extent(patch.page) {
                return Err(fail());
            }
            let id = tile_node_id(&patch.anchor).ok_or_else(fail)?;
            if sources.insert(id, index).is_some() {
                return Err(fail());
            }
            requested.push((id, patch.generation));
        }
        // Distinct IDs can still overlap when one names another's ancestor.
        for &(id, _) in &requested {
            let mut parent = id >> 2;
            while parent >= 8 {
                if sources.contains_key(&parent) {
                    return Err(fail());
                }
                parent >>= 2;
            }
        }
        let mut residency = self.residency.clone();
        let changes = residency.update(&requested);
        let changed: std::collections::BTreeSet<_> = changes.iter().map(|c| c.slot).collect();
        let mut anchors = self.anchors.clone();
        let mut ids = vec![0; self.node_ids.len()];
        let mut leaves = vec![[0; 4]; self.node_ids.len()];
        let mut dirty = Vec::new();
        let mut reset = Vec::new();
        let mut retained = 0;
        for entry in residency.iter() {
            let patch = &patches[sources[&entry.node_id]];
            let slot = entry.slot as usize;
            let size = (patch.page.residuals().len() as u64).div_ceil(2) * 4;
            if size > self.page_slots[slot].1 {
                return Err(fail());
            }
            let frame = patch.anchor.to_gpu(self.camera).ok_or_else(fail)?;
            let old_frame = anchors[slot].to_gpu(self.camera).ok_or_else(fail)?;
            let reuse = self.node_ids[slot] != entry.node_id;
            if reuse {
                reset.push(slot);
            } else {
                retained += 1;
            }
            if changed.contains(&entry.slot)
                || reuse
                || self.manual_updates[slot]
                || frame != old_frame
            {
                dirty.push((entry.slot, patch.page));
                if !reuse {
                    reset.push(slot);
                }
            }
            anchors[slot] = patch.anchor;
            ids[slot] = entry.node_id;
            leaves[slot] = [
                entry.node_id as u32,
                (entry.node_id >> 32) as u32,
                3 + 2 * u32::from(patch.anchor.tile.level),
                entry.slot,
            ];
        }
        let active_extent = ids.iter().rposition(|id| *id != 0).map_or(0, |i| i + 1);
        encode_frames(
            &mut self.frame_staging,
            &anchors[..active_extent],
            self.camera,
        )?;
        // From here all fallible validation has completed. Queue ordering keeps
        // the previous submitted draw intact and the next draw wholly new.
        if ids != self.node_ids || !dirty.is_empty() {
            self.invalidate_shadows();
            self.ocean_dirty = true;
            if let Some(topology) = &self.topology {
                topology.mark_dirty();
            }
        }
        for (slot, (&old, &new)) in self.node_ids.iter().zip(&ids).enumerate() {
            if old != 0 && new == 0 {
                backend
                    .queue
                    .write_buffer(&self.metadata, slot as u64 * 16, &[0; 16]);
            }
        }
        for slot in reset {
            backend
                .queue
                .write_buffer(&self.history, slot as u64 * 16, &[0; 16]);
        }
        backend
            .queue
            .write_buffer(&self.leaves, 0, &bytes(leaves.into_iter().flatten()));
        if !self.frame_staging.is_empty() {
            backend
                .queue
                .write_buffer(&self.frames, 0, &self.frame_staging);
        }
        self.anchors = anchors;
        self.node_ids = ids;
        self.residency = residency;
        self.active_extent = active_extent;
        if let Some(source) = &self.surface_source {
            source.publish(backend, &self.node_ids)?;
        }
        self.upload_pages(backend, &dirty);
        for (bound, id) in self.shadow_bounds.iter_mut().zip(&self.node_ids) {
            if *id == 0 {
                *bound = None;
            }
        }
        for (extent, id) in self.resident_height_extent.iter_mut().zip(&self.node_ids) {
            if *id == 0 {
                *extent = None;
            }
        }
        self.manual_updates.fill(false);
        Ok(TerrainPublication {
            active: patches.len(),
            regenerated: dirty.len(),
            retained,
            dispatch_slots: active_extent,
        })
    }

    pub fn slot(&self, node_id: u64) -> Option<u32> {
        self.residency.slot(node_id)
    }

    /// Publish changed quantized pages into stable slots and regenerate only
    /// their GPU patches. Reuses pipelines/buffers; no CPU mesh or readback.
    /// Updates must have unique valid ordinals and fit the original slot.
    /// All updates are validated before any GPU mutation.
    pub fn update_pages(
        &mut self,
        backend: &Backend,
        updates: &[(u32, &HeightPage)],
    ) -> Result<(), BackendError> {
        if updates.is_empty() {
            return Ok(());
        }
        let mut ordinals = updates.iter().map(|(i, _)| *i).collect::<Vec<_>>();
        ordinals.sort_unstable();
        if ordinals.windows(2).any(|w| w[0] == w[1])
            || updates.iter().any(|(i, page)| {
                if self.surface_source.is_some() && !valid_surface_extent(page) {
                    return true;
                }
                self.page_slots
                    .get(*i as usize)
                    .is_none_or(|(_, capacity)| {
                        self.node_ids[*i as usize] == 0
                            || (page.residuals().len() as u64).div_ceil(2) * 4 > *capacity
                    })
            })
        {
            return Err(BackendError::Device(
                "invalid/duplicate terrain ordinal or page exceeds stable slot".into(),
            ));
        }
        for &(slot, _) in updates {
            self.manual_updates[slot as usize] = true;
        }
        self.invalidate_shadows();
        self.upload_pages(backend, updates);
        Ok(())
    }

    fn upload_pages(&mut self, backend: &Backend, updates: &[(u32, &HeightPage)]) {
        self.ocean_dirty |= !updates.is_empty();
        for (i, page) in updates {
            let idx = *i as usize;
            if let Some(bound) = self.shadow_bounds.get_mut(idx) {
                *bound = PatchShadowBound::new(&self.anchors[idx], page);
            }
            if let Some(possible) = self.ocean_possible.get_mut(idx) {
                *possible = ocean_height_range(page);
            }
            self.resident_height_extent[idx] = Some(height_extent(page));
            self.ray_revisions[idx] = self.ray_revisions[idx].wrapping_add(1).max(1);
            let (offset, _) = self.page_slots[idx];
            let mut data = page.residual_bytes();
            data.resize(data.len().div_ceil(4) * 4, 0);
            backend.queue.write_buffer(&self.residuals, offset, &data);
            backend.queue.write_buffer(
                &self.metadata,
                u64::from(*i) * 16,
                &bytes([
                    page.base_height_m().to_bits(),
                    page.residual_scale_m().to_bits(),
                    page.grid_size(),
                    (offset / 4) as u32,
                ]),
            );
        }
        if let Some(source) = &self.surface_source {
            source.publish_bounds(backend, &self.node_ids, &self.resident_height_extent);
        }
        if !updates.is_empty() {
            backend
                .queue
                .write_buffer(&self.dirty, 0, &bytes(updates.iter().map(|(i, _)| *i)));
        }
        backend.queue.write_buffer(
            &self.params,
            0,
            &bytes([
                self.active_extent as u32,
                VERTICES_PER_PATCH,
                self.radius.to_bits(),
                updates.len() as u32,
            ]),
        );
        if updates.is_empty() {
            return;
        }
        if let Some(topology) = &self.topology {
            topology.mark_dirty();
            return;
        }
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        {
            let mut pass = encoder.begin_compute_pass(&Default::default());
            pass.set_pipeline(&self.geometry);
            pass.set_bind_group(0, &self.geometry_bind, &[]);
            pass.dispatch_workgroups(
                (updates.len() as u32 * VERTICES_PER_PATCH).div_ceil(64),
                1,
                1,
            );
        }
        backend.queue.submit([encoder.finish()]);
    }

    /// Update body-fixed camera uniforms. Tile anchors and shader world space
    /// share exactly this f64 camera origin. No terrain regeneration/readback.
    pub fn update_view(
        &mut self,
        backend: &Backend,
        params: &ViewParams,
    ) -> Result<(), BackendError> {
        if self.topology.is_some() {
            return Err(BackendError::Device(
                "conforming terrain requires update_view_for_extent".into(),
            ));
        }
        self.update_view_for_extent(backend, params, [1, 1])
    }

    /// View and actual scaled render-target extent for persistent LOD policy.
    /// Call after page/material publication and before encode_surface.
    pub fn update_view_for_extent(
        &mut self,
        backend: &Backend,
        params: &ViewParams,
        extent: [u32; 2],
    ) -> Result<(), BackendError> {
        self.update_view_jittered(
            backend,
            params,
            extent,
            thessa_render_core::temporal::JitterOffset::ZERO,
        )
    }

    /// Primary raster jitter only. LOD/classification, sky and secondary passes
    /// keep the unjittered view. Nonzero jitter requires resolved jitter settings
    /// at construction; rejection occurs before any camera-buffer write.
    pub fn update_view_jittered(
        &mut self,
        backend: &Backend,
        params: &ViewParams,
        extent: [u32; 2],
        jitter: thessa_render_core::temporal::JitterOffset,
    ) -> Result<(), BackendError> {
        if !jitter.is_zero() && self.primary_view.is_none() {
            return Err(BackendError::Device(
                "primary jitter is not resolved for this terrain renderer".into(),
            ));
        }
        let primary = if self.primary_view.is_some() {
            Some(
                ViewUniformBlock::new_jittered(params, params.world_position_m, jitter, extent)
                    .ok_or_else(|| {
                        BackendError::Device("invalid primary camera uniforms".into())
                    })?,
            )
        } else {
            None
        };
        let camera = params.world_position_m.to_array();
        let block = ViewUniformBlock::new(params)
            .ok_or_else(|| BackendError::Device("invalid camera uniforms".into()))?;
        if let Some(topology) = &self.topology {
            let ocean_bound = if self.ocean_settings.geometry_patches > 0 {
                self.ocean
                    .as_ref()
                    .map_or(0.0, |f| f.stats().height_bound_m as f32)
            } else {
                0.0
            };
            topology.update_view(backend, &block, camera, extent, ocean_bound)?;
        }
        if camera != self.camera {
            // Projection/rotation/exposure changes do not change tile-relative
            // anchors. Publications already upload anchors at self.camera.
            encode_frames(
                &mut self.frame_staging,
                &self.anchors[..self.active_extent],
                camera,
            )?;
            if !self.frame_staging.is_empty() {
                backend
                    .queue
                    .write_buffer(&self.frames, 0, &self.frame_staging);
            }
            if self.topology.is_none() {
                self.invalidate_shadows();
            }
        }
        backend.queue.write_buffer(&self.view, 0, &block.to_bytes());
        if let (Some(buffer), Some(primary)) = (&self.primary_view, primary) {
            backend.queue.write_buffer(buffer, 0, &primary.to_bytes());
        }
        self.camera = camera;
        Ok(())
    }

    /// Publish the scene's shared sky buffers. Disabled drops the consumer:
    /// no background draw, extra target, or retained sky allocation/pipeline.
    pub fn share_environment(
        &mut self,
        backend: &Backend,
        environment: Option<&crate::environment::EnvironmentBinding>,
    ) {
        {
            let environment = environment
                .cloned()
                .unwrap_or_else(|| crate::environment::EnvironmentBinding::new(&backend.device));
            let mut entries = self
                .ocean
                .as_ref()
                .map_or_else(Vec::new, |field| field.binding().entries().to_vec());
            if self.ocean.is_some() || self.land_environment_samples > 0 {
                entries.extend(environment.entries());
            } else {
                entries.extend(environment.cloud_entries());
            }
            self.ocean_bind = Some(
                backend
                    .device
                    .create_bind_group(&wgpu::BindGroupDescriptor {
                        label: Some("ocean-raster-environment"),
                        layout: &self.raster.get_bind_group_layout(2),
                        entries: &entries,
                    }),
            );
            self.ocean_environment = Some(environment);
        }
        if let Some(environment) = environment.filter(|field| field.enabled() && self.sky_visible) {
            if let Some(sky) = &mut self.sky {
                sky.rebind(&backend.device, &self.view, environment);
            } else {
                self.sky = Some(crate::sky::SkyBackground::new(
                    &backend.device,
                    self.format,
                    &self.view,
                    environment,
                ));
            }
        } else {
            self.sky = None;
        }
    }

    pub fn sky_enabled(&self) -> bool {
        self.sky.is_some()
    }

    /// Applied clear-sky quadrature budget (cloud columns use the smaller
    /// resolved reflection budget); zero eliminates the
    /// consumer at shader creation, without a texture, pass or per-frame work.
    pub fn land_environment_samples(&self) -> u32 {
        self.land_environment_samples
    }

    /// Lazy plume consumer. Disabled settings release all GPU resources;
    /// absent engine draws do not allocate a consumer in the first place.
    pub fn update_plume(
        &mut self,
        backend: &Backend,
        view: &ViewParams,
        settings: &thessa_graphics::ResolvedGraphicsSettings,
        frame: Option<crate::plume::PlumeFrame>,
    ) -> Result<(), BackendError> {
        if !settings.plume_enabled {
            self.plume = None;
            return Ok(());
        }
        if frame.is_some()
            && self
                .plume
                .as_ref()
                .is_none_or(|p| !p.is_live_for(self.format, settings.plume_diamonds))
        {
            self.plume = Some(crate::plume::PlumeRenderer::new(
                backend,
                self.format,
                settings.plume_diamonds,
            )?);
        }
        if let Some(plume) = &mut self.plume {
            plume.update(backend, view, frame)?;
        }
        Ok(())
    }

    pub fn plume_allocated(&self) -> bool {
        self.plume.is_some()
    }

    pub fn plume_stats(&self) -> crate::plume::PlumeStats {
        self.plume
            .as_ref()
            .map_or_else(Default::default, |p| p.stats())
    }

    pub fn ocean_stats(&self) -> crate::ocean::OceanStats {
        let mut stats = self
            .ocean
            .as_ref()
            .map_or_else(Default::default, |f| f.stats());
        if let Some(geometry) = &self.ocean_geometry {
            stats.deformed_patches = geometry.selected.len() as u32;
            stats.geometry_bytes =
                geometry.original.size() + geometry.slots.size() + geometry.params.size();
        }
        if let Some(topology) = &self.topology {
            stats.geometry_bytes += topology.surface.ocean_storage_bytes();
        }
        stats
    }
    pub fn ocean_reason(&self) -> Option<&str> {
        self.ocean_reason.as_deref()
    }

    /// Publish one simulation sample after height/material/camera updates and
    /// before shadow encoding and mesh RT synchronization. Restores datum
    /// backups first; never accumulates displacement or expands CPU geometry.
    pub fn update_ocean(&mut self, backend: &Backend, time_s: f64) -> Result<(), BackendError> {
        self.update_ocean_profiled(backend, time_s, None)
    }

    /// Two-mark diagnostic for near-slot restoration/capture/deformation only;
    /// FFT publication precedes this interval. Caller owns completion/readback.
    pub fn update_ocean_profiled(
        &mut self,
        backend: &Backend,
        time_s: f64,
        timing: Option<&crate::timing::GpuIntervals>,
    ) -> Result<(), BackendError> {
        let Some(field) = &mut self.ocean else {
            return Ok(());
        };
        let evolved = field.update(backend, time_s)?;
        if evolved
            && self.ocean_settings.geometry_patches > 0
            && let Some(topology) = &self.topology
        {
            topology.mark_dirty();
        }
        field.update_frame(
            &backend.queue,
            glam::DVec3::from_array(self.camera),
            self.body_radius_m,
        )?;
        let Some(geometry) = &mut self.ocean_geometry else {
            return Ok(());
        };
        if !evolved
            && !self.ocean_dirty
            && self.ocean_last_camera == Some(self.camera)
            && geometry
                .selected
                .iter()
                .all(|&(slot, rev)| self.ray_revisions[slot] == rev)
        {
            return Ok(());
        }
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        if let Some(t) = timing {
            t.mark(&mut encoder, 0);
        }
        let mut geometry_changed = !geometry.selected.is_empty();
        let slot_bytes = u64::from(VERTICES_PER_PATCH) * 32;
        for (backup, &(slot, revision)) in geometry.selected.iter().enumerate() {
            backend
                .queue
                .write_buffer(&self.history, slot as u64 * 16 + 12, &0u32.to_le_bytes());
            if self.ray_revisions[slot] == revision {
                encoder.copy_buffer_to_buffer(
                    &geometry.original,
                    backup as u64 * slot_bytes,
                    &self.vertices,
                    slot as u64 * slot_bytes,
                    slot_bytes,
                );
                self.ray_revisions[slot] = self.ray_revisions[slot].wrapping_add(1);
            }
        }
        let camera = glam::DVec3::from_array(self.camera);
        let distance = self.ocean_settings.geometry_distance_m;
        let mut selected = Vec::new();
        for (slot, anchor) in self
            .anchors
            .iter()
            .enumerate()
            .filter(|(i, _)| self.node_ids[*i] != 0 && self.ocean_possible[*i].is_some())
        {
            let width = 2.0 * self.body_radius_m / 2f64.powi(i32::from(anchor.tile.level));
            // At >= one longest period per filtered cell the zero-mean
            // height field reduces to its DC mode; only optical variance stays.
            if width / 32.0 * 2.0 >= thessa_render_core::ocean::PERIODS_M[0] {
                continue;
            }
            let (low, high) = self.ocean_possible[slot].unwrap();
            let (min, max) = anchor.aabb_body_m(low, high);
            // Reuse the nine-point surface bound, padded by a normalized-cube
            // Hessian envelope (||H|| <= 6 for |raw| >= 1). width²/R exceeds
            // its bilinear sample error 1.5*R*half_extent².
            let padding = (width / self.body_radius_m).powi(2)
                * (self.body_radius_m + low)
                    .abs()
                    .max((self.body_radius_m + high).abs());
            let closest = camera.clamp(
                glam::DVec3::from_array(min) - glam::DVec3::splat(padding),
                glam::DVec3::from_array(max) + glam::DVec3::splat(padding),
            );
            if (closest - camera).length() < distance {
                selected.push(slot);
            }
        }
        let reason = if selected.len() > geometry.capacity {
            Some(format!(
                "ocean near partition has {} patches, budget admits {}; filtered optical-only fallback",
                selected.len(),
                geometry.capacity
            ))
        } else {
            None
        };
        if reason != self.ocean_reason {
            if let Some(reason) = &reason {
                eprintln!("{reason}");
            }
            self.ocean_reason = reason;
        }
        geometry.selected.clear();
        if self.ocean_reason.is_none() && distance > 0.0 {
            geometry_changed |= !selected.is_empty();
            for (backup, &slot) in selected.iter().enumerate() {
                backend.queue.write_buffer(
                    &self.history,
                    slot as u64 * 16 + 12,
                    &1u32.to_le_bytes(),
                );
                encoder.copy_buffer_to_buffer(
                    &self.vertices,
                    slot as u64 * slot_bytes,
                    &geometry.original,
                    backup as u64 * slot_bytes,
                    slot_bytes,
                );
                self.ray_revisions[slot] = self.ray_revisions[slot].wrapping_add(1);
                geometry.selected.push((slot, self.ray_revisions[slot]));
            }
            if !selected.is_empty() {
                backend.queue.write_buffer(
                    &geometry.slots,
                    0,
                    &bytes(selected.iter().map(|&s| s as u32)),
                );
                backend.queue.write_buffer(
                    &geometry.params,
                    0,
                    &bytes([
                        selected.len() as u32,
                        self.radius.to_bits(),
                        (distance as f32).to_bits(),
                        0,
                    ]),
                );
                let mut pass = encoder.begin_compute_pass(&Default::default());
                pass.set_pipeline(&geometry.pipeline);
                pass.set_bind_group(0, &geometry.bind, &[]);
                pass.set_bind_group(1, &geometry.field_bind, &[]);
                pass.dispatch_workgroups(
                    (selected.len() as u32 * VERTICES_PER_PATCH).div_ceil(64),
                    1,
                    1,
                );
            }
        }
        if let Some(t) = timing {
            t.mark(&mut encoder, 1);
            t.resolve(&mut encoder);
        }
        if geometry_changed || timing.is_some() {
            backend.queue.submit([encoder.finish()]);
        }
        self.ocean_dirty = false;
        self.ocean_last_camera = Some(self.camera);
        if geometry_changed {
            self.invalidate_shadows();
        }
        Ok(())
    }

    /// Lazily borrow the existing height publication for the persistent CBT
    /// experiment. No directory allocation or publication cost while unused.
    pub fn height_surface_source(
        &mut self,
        backend: &Backend,
    ) -> Result<crate::conforming_surface::HeightSurfaceSource, BackendError> {
        if self.surface_source.is_none() {
            self.surface_source = Some(crate::conforming_surface::HeightSurfaceSource::new(
                backend,
                &self.metadata,
                &self.residuals,
                self.body_radius_m,
                &self.node_ids,
                &self.resident_height_extent,
            )?);
        }
        Ok(self.surface_source.as_ref().unwrap().clone())
    }

    /// Share existing canonical material residency without a geometry-specific
    /// layer assignment, second cache or texture upload.
    pub fn material_binding(&self) -> crate::material::MaterialBinding {
        self.materials.binding()
    }

    /// Conservative surface-height bound for CBT LOD classification, in
    /// unit-sphere chord units (`height_m / body_radius_m`). Derived from the
    /// widest resident page extent including residual error, so relief is
    /// charged for the screen area it adds. Zero when no page is resident, and
    /// clamped to one chord because the classifier works on a unit sphere.
    pub fn height_bound_chord(&self) -> f32 {
        let radius = self.body_radius_m;
        if !radius.is_finite() || radius <= 0.0 {
            return 0.0;
        }
        let widest = self.resident_height_extent.iter().flatten().fold(
            None,
            |acc: Option<(f64, f64)>, &(low, high)| {
                Some(match acc {
                    None => (low, high),
                    Some((a, b)) => (a.min(low), b.max(high)),
                })
            },
        );
        let Some((low, high)) = widest else {
            return 0.0;
        };
        (low.abs().max(high.abs()) / radius).min(1.0) as f32
    }

    pub fn ray_source(&self) -> Option<crate::terrain_ray::TerrainRaySource> {
        if let Some(topology) = &self.topology {
            return topology.surface.ray_source(&self.materials.binding());
        }
        self.vertices
            .usage()
            .contains(wgpu::BufferUsages::BLAS_INPUT)
            .then(|| crate::terrain_ray::TerrainRaySource {
                ocean: self.ocean.as_ref().map(|f| f.binding().clone()),
                vertices: self.vertices.clone(),
                directory: self.materials.directory.clone(),
                material: self.materials.view.clone(),
                sampler: self.materials.sampler.clone(),
                radius_m: self.body_radius_m,
                capacity: self.node_ids.len(),
                conforming: None,
                conforming_configuration: 0,
            })
    }

    pub(crate) fn ray_publication(&self) -> (&wgpu::Buffer, &[u64], &[u64], &[TileAnchor]) {
        (
            &self.vertices,
            &self.node_ids,
            &self.ray_revisions,
            &self.anchors,
        )
    }

    /// Encode a full GPU-owned visibility/triangle-LOD/indirect raster frame,
    /// optionally drawing rigid scene meshes in the same render pass to eliminate
    /// LoadOp::Load and attachment store/load overhead.
    /// Conforming mode requires encode_surface() first, before shadows/RT.
    /// Color formats must match both renderers; depth is single-sample
    /// Depth32Float. Call mesh.encode_rt() before opening this pass if present.
    pub fn encode_scene(
        &self,
        encoder: &mut wgpu::CommandEncoder,
        color: &wgpu::TextureView,
        depth: &wgpu::TextureView,
        mesh: Option<&crate::mesh::MeshRenderer>,
    ) {
        self.encode_scene_ops(encoder, color, depth, mesh, true);
    }

    /// Unified scene pass with explicit control over depth buffer store.
    /// Passing `store_depth: false` permits `StoreOp::Discard` when neither
    /// resolved motion diagnostics nor an active plume consumes scene depth.
    pub fn encode_scene_ops(
        &self,
        encoder: &mut wgpu::CommandEncoder,
        color: &wgpu::TextureView,
        depth: &wgpu::TextureView,
        mesh: Option<&crate::mesh::MeshRenderer>,
        store_depth: bool,
    ) {
        self.encode_opaque_scene_ops(encoder, color, depth, mesh, store_depth);
        self.encode_plume(encoder, color, depth);
    }

    /// Opaque scene plus far-depth sky background, without translucent plume.
    /// Motion diagnostics must run after this pass with `store_depth = true`,
    /// before plume or cloud overlays can contaminate the static-surface mask.
    pub fn encode_opaque_scene_ops(
        &self,
        encoder: &mut wgpu::CommandEncoder,
        color: &wgpu::TextureView,
        depth: &wgpu::TextureView,
        mesh: Option<&crate::mesh::MeshRenderer>,
        store_depth: bool,
    ) {
        if self.topology.is_none() {
            let mut pass = encoder.begin_compute_pass(&Default::default());
            pass.set_bind_group(0, &self.classify_bind, &[]);
            for (i, pipeline) in self.classify.iter().enumerate() {
                pass.set_pipeline(pipeline);
                if i != 1 || self.active_extent > 0 {
                    pass.dispatch_workgroups(
                        if i == 1 { self.active_extent as u32 } else { 1 },
                        1,
                        1,
                    );
                }
            }
        }
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("owned-unified-scene-frame"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: color,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color {
                        r: 0.015,
                        g: 0.025,
                        b: 0.05,
                        a: 1.0,
                    }),
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: depth,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(1.0),
                    store: if store_depth
                        || self.motion_vectors
                        || self.plume.as_ref().is_some_and(|p| p.active())
                    {
                        wgpu::StoreOp::Store
                    } else {
                        wgpu::StoreOp::Discard
                    },
                }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
            multiview_mask: None,
        });
        pass.set_pipeline(&self.raster);
        pass.set_bind_group(0, &self.raster_bind, &[]);
        if let Some(receiver) = &self.shadow_receiver {
            pass.set_bind_group(1, receiver, &[]);
        }
        if let Some(ocean) = &self.ocean_bind {
            pass.set_bind_group(2, ocean, &[]);
        }
        if let Some(topology) = &self.topology {
            if topology.surface.publication().revision != 0 {
                pass.draw(0..topology.surface.slot_count() * 3, 0..1);
            }
        } else {
            pass.draw_indirect(&self.draw, 0);
        }
        if let Some(mesh) = mesh {
            mesh.draw_into_pass(&mut pass);
        }
        // Far depth, no depth writes: terrain and opaque meshes reject covered
        // pixels before sky sampling. UI remains a separate overlay pass.
        if let Some(sky) = &self.sky {
            sky.draw(&mut pass);
        }
        drop(pass);
    }

    /// Translucent, depth-clipped plume, after opaque motion diagnostics.
    pub fn encode_plume(
        &self,
        encoder: &mut wgpu::CommandEncoder,
        color: &wgpu::TextureView,
        depth: &wgpu::TextureView,
    ) {
        if let Some(plume) = &self.plume {
            plume.encode(encoder, color, depth);
        }
    }

    /// Raster the current publication. Conforming mode requires encode_surface
    /// before any shadows/RT/raster consumers; regular mode classifies here.
    /// Target format must match construction; depth must be Depth32Float.
    pub fn encode(
        &self,
        encoder: &mut wgpu::CommandEncoder,
        color: &wgpu::TextureView,
        depth: &wgpu::TextureView,
    ) {
        self.encode_scene(encoder, color, depth, None);
    }

    /// Diagnostic only: legacy tile-local xyz/w + normal xyz/height, or the
    /// canonical 48-byte ABI when conforming_surface() is present.
    pub fn read_vertices(&self, backend: &Backend) -> Result<Vec<f32>, BackendError> {
        Ok(read_buffer(
            backend,
            self.topology
                .as_ref()
                .map_or(&self.vertices, |t| &t.surface.vertices),
        )?
        .as_chunks::<4>()
        .0
        .iter()
        .map(|b| f32::from_le_bytes(*b))
        .collect())
    }

    /// Diagnostic only; never needed to submit an ordinary frame.
    pub fn read_draw(&self, backend: &Backend) -> Result<Vec<u32>, BackendError> {
        if let Some(topology) = &self.topology {
            // Submitted allocation triangles include zero-area free slots;
            // callers must not describe this count as the live-leaf count.
            return Ok(vec![topology.surface.slot_count() * 3, 1, 0, 0]);
        }
        Ok(read_buffer(backend, &self.draw)?
            .as_chunks::<4>()
            .0
            .iter()
            .map(|b| u32::from_le_bytes(*b))
            .collect())
    }
}

fn ocean_height_range(page: &HeightPage) -> Option<(f64, f64)> {
    let min = f64::from(page.min_height_m() - page.max_residual_error_m());
    let max = f64::from(page.max_height_m() + page.max_residual_error_m()).min(1.0);
    (min < 1.0).then_some((min, max))
}

/// Resident height extent for one page, including its residual error bound so
/// the CBT LOD height bound stays conservative.
fn height_extent(page: &HeightPage) -> (f64, f64) {
    (
        f64::from(page.min_height_m()) - f64::from(page.max_residual_error_m()),
        f64::from(page.max_height_m()) + f64::from(page.max_residual_error_m()),
    )
}

fn valid_surface_extent(page: &HeightPage) -> bool {
    let (low, high) = height_extent(page);
    low.is_finite()
        && high.is_finite()
        && low <= high
        && low.abs().max(high.abs()) <= f64::from(f32::MAX)
}

fn frame_bytes(anchors: &[TileAnchor], camera: [f64; 3]) -> Result<Vec<u8>, BackendError> {
    let mut out = Vec::with_capacity(anchors.len() * 112);
    encode_frames(&mut out, anchors, camera)?;
    Ok(out)
}

fn encode_frames(
    out: &mut Vec<u8>,
    anchors: &[TileAnchor],
    camera: [f64; 3],
) -> Result<(), BackendError> {
    out.clear();
    for anchor in anchors {
        let frame = anchor
            .to_gpu(camera)
            .ok_or_else(|| BackendError::Device("invalid terrain frame".into()))?;
        // Encode one fixed shader record before extending the destination.
        // Appending four bytes at a time inhibits bulk-store optimization.
        let encoded = frame.words().map(|word| word.map(f32::to_le_bytes));
        out.extend_from_slice(encoded.as_flattened().as_flattened());
    }
    Ok(())
}

fn tile_node_id(anchor: &TileAnchor) -> Option<u64> {
    thessa_rcbt_core::conforming_surface::page_id(anchor.tile)
}

fn solid_texture(backend: &Backend, rgba: [u8; 4], array: bool) -> wgpu::TextureView {
    let texture = backend.device.create_texture_with_data(
        &backend.queue,
        &wgpu::TextureDescriptor {
            label: Some("cbt-fallback-material"),
            size: wgpu::Extent3d {
                width: 1,
                height: 1,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8Unorm,
            usage: wgpu::TextureUsages::TEXTURE_BINDING,
            view_formats: &[],
        },
        wgpu::util::TextureDataOrder::LayerMajor,
        &rgba,
    );
    texture.create_view(&wgpu::TextureViewDescriptor {
        dimension: Some(if array {
            wgpu::TextureViewDimension::D2Array
        } else {
            wgpu::TextureViewDimension::D2
        }),
        ..Default::default()
    })
}

fn read_buffer(backend: &Backend, source: &wgpu::Buffer) -> Result<Vec<u8>, BackendError> {
    let staging = backend.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("cbt-diagnostic-readback"),
        size: source.size(),
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    encoder.copy_buffer_to_buffer(source, 0, &staging, 0, source.size());
    backend.queue.submit([encoder.finish()]);
    map_buffer(backend, &staging)
}

pub(crate) fn map_buffer(
    backend: &Backend,
    staging: &wgpu::Buffer,
) -> Result<Vec<u8>, BackendError> {
    let (tx, rx) = std::sync::mpsc::channel();
    staging
        .slice(..)
        .map_async(wgpu::MapMode::Read, move |result| {
            let _ = tx.send(result);
        });
    backend
        .device
        .poll(wgpu::PollType::wait_indefinitely())
        .map_err(|e| BackendError::Device(e.to_string()))?;
    rx.recv()
        .map_err(|e| BackendError::Device(e.to_string()))?
        .map_err(|e| BackendError::Device(e.to_string()))?;
    let view = staging
        .slice(..)
        .get_mapped_range()
        .map_err(|e| BackendError::Device(e.to_string()))?;
    let out = view.to_vec();
    drop(view);
    staging.unmap();
    Ok(out)
}

/// Persistent color/depth targets; reuse across frames. Screenshot readback is
/// explicit and is not part of the steady-state rendering benchmark.
pub struct OffscreenTarget {
    pub color: wgpu::TextureView,
    pub depth: wgpu::TextureView,
    texture: wgpu::Texture,
    width: u32,
    height: u32,
}

impl OffscreenTarget {
    pub const FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8UnormSrgb;

    pub fn new(backend: &Backend, width: u32, height: u32) -> Result<Self, BackendError> {
        Self::with_format(backend, width, height, Self::FORMAT)
    }

    /// RGBA8 capture target. The unorm variant tests shader-side SDR transfer;
    /// read_rgba8 returns stored bytes without inserting a color transform.
    pub fn with_format(
        backend: &Backend,
        width: u32,
        height: u32,
        format: wgpu::TextureFormat,
    ) -> Result<Self, BackendError> {
        if !matches!(
            format,
            wgpu::TextureFormat::Rgba8Unorm | wgpu::TextureFormat::Rgba8UnormSrgb
        ) {
            return Err(BackendError::Device(
                "offscreen capture requires RGBA8 output".into(),
            ));
        }
        let max = backend.device.limits().max_texture_dimension_2d;
        if width == 0 || height == 0 || width > max || height > max {
            return Err(BackendError::Device("invalid offscreen extent".into()));
        }
        let make = |format, usage| {
            backend.device.create_texture(&wgpu::TextureDescriptor {
                label: Some("cbt-offscreen"),
                size: wgpu::Extent3d {
                    width,
                    height,
                    depth_or_array_layers: 1,
                },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format,
                usage,
                view_formats: &[],
            })
        };
        let texture = make(
            format,
            wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
        );
        let depth = make(
            wgpu::TextureFormat::Depth32Float,
            wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
        )
        .create_view(&Default::default());
        let color = texture.create_view(&Default::default());
        Ok(Self {
            color,
            depth,
            texture,
            width,
            height,
        })
    }

    pub fn read_rgba8(&self, backend: &Backend) -> Result<Vec<u8>, BackendError> {
        let stride = (self.width * 4).div_ceil(wgpu::COPY_BYTES_PER_ROW_ALIGNMENT)
            * wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
        let staging = backend.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("cbt-screenshot"),
            size: stride as u64 * self.height as u64,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        encoder.copy_texture_to_buffer(
            self.texture.as_image_copy(),
            wgpu::TexelCopyBufferInfo {
                buffer: &staging,
                layout: wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(stride),
                    rows_per_image: Some(self.height),
                },
            },
            wgpu::Extent3d {
                width: self.width,
                height: self.height,
                depth_or_array_layers: 1,
            },
        );
        backend.queue.submit([encoder.finish()]);
        let padded = map_buffer(backend, &staging)?;
        Ok(padded
            .chunks_exact(stride as usize)
            .flat_map(|row| row[..self.width as usize * 4].iter().copied())
            .collect())
    }
}

#[cfg(test)]
#[path = "terrain_frame_tests.rs"]
mod frame_encoding_tests;
