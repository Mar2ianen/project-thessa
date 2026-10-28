//! Persistent opaque textured mesh consumer. Loads the caller's existing
//! color/depth targets after terrain; no scene copy, readback or extra device.
use crate::{
    Backend, BackendError,
    view::{ViewParams, ViewUniformBlock},
};
use thessa_rcbt_core::lighting::SurfaceLighting;
use thessa_render_core::mesh::{MeshAsset, MeshInstance, Wrap};
use wgpu::util::DeviceExt;

struct ShadowPrimitive {
    vertices: wgpu::Buffer,
    indices: wgpu::Buffer,
    count: u32,
    max_error_m: f64,
}

struct Primitive {
    bounds: Option<crate::mesh_visibility::Bounds>,
    shadow: Option<ShadowPrimitive>,
    ambient: wgpu::Buffer,
    vertices: wgpu::Buffer,
    indices: wgpu::Buffer,
    count: u32,
    material: usize,
}
/// Primary submitted/candidate workload; not GPU timing or occlusion results.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct MeshDrawStats {
    /// Unique instance indices with at least one submitted primary primitive.
    pub instances: u32,
    /// Retained instances before primary frustum rejection; shadow/RT sources remain.
    pub candidate_instances: u32,
    pub candidate_triangles: u64,
    pub draw_calls: usize,
    /// Logical indexed commands; an indirect API call can batch several.
    pub draw_commands: usize,
    pub triangles: u64,
    /// Actual uniform/instance/indirect bytes written by the latest successful update,
    /// plus any subsequent primary jitter upload. Reused instance data is zero.
    pub frame_upload_bytes: usize,
}
/// Surface/lighting configuration for a native mesh renderer. Bundling the
/// resolved settings in one struct keeps the constructor readable as shadows,
/// material quality and ray tracing join it.
pub struct SurfaceSettings<'a> {
    pub format: wgpu::TextureFormat,
    /// Rigid-instance capacity.
    pub capacity: usize,
    pub quality: thessa_graphics::Quality,
    /// Specular reflections (terrain and mesh) on/off.
    pub reflections: bool,
    pub shadows: Option<&'a crate::shadows::ShadowAtlas>,
    /// Resolved graphics settings for RT and the optional temporal mask;
    /// `None` keeps raster-only rendering with ordinary opaque alpha.
    pub ray_tracing: Option<&'a thessa_graphics::ResolvedGraphicsSettings>,
}

#[derive(Default)]
struct ShadowCommands {
    commands: Vec<u8>,
    alternate: Vec<u8>,
    batches: Vec<(usize, bool, std::ops::Range<u32>)>,
    uploaded: Vec<Vec<u8>>,
}

pub struct MeshRenderer {
    shadow_queue: Option<std::sync::Arc<wgpu::Queue>>,
    shadow_command_budget: u64,
    shadow_indirect: Option<crate::shadows::MeshCommandStorage>,
    shadow_available: Option<std::sync::Arc<std::sync::Mutex<u64>>>,
    shadow_layer_capacity: usize,
    shadow_commands: std::sync::Mutex<ShadowCommands>,
    shadow_receiver: Option<wgpu::BindGroup>,
    shadow_caster: Option<wgpu::BindGroup>,
    uploaded_instances: Vec<u8>,
    uploaded_origin: Option<glam::DVec3>,
    last_instances: Vec<thessa_render_core::mesh::MeshInstance>,
    shadow_revision: u64,
    rt: Option<crate::ray_scene::RayScene>,
    rt_reason: Option<String>,
    environment: crate::environment::EnvironmentBinding,
    pipeline: wgpu::RenderPipeline,
    global: wgpu::BindGroup,
    materials: Vec<wgpu::BindGroup>,
    primitives: Vec<Primitive>,
    world_bounds: Vec<Vec<Option<crate::mesh_visibility::WorldBounds>>>,
    cache_bounds: bool,
    view: wgpu::Buffer,
    lighting: wgpu::Buffer,
    instances: wgpu::Buffer,
    capacity: usize,
    count: u32,
    staging: Vec<u8>,
    ranges: Vec<std::ops::Range<u32>>,
    primary_runs: Vec<Vec<std::ops::Range<u32>>>,
    primary_seen: Vec<bool>,
    primary_stats: MeshDrawStats,
    primary_view: Option<(glam::DVec3, glam::Mat4)>,
    primary_indirect_supported: bool,
    primary_indirect: Option<wgpu::Buffer>,
    primary_indirect_capacity: usize,
    primary_indirect_bytes: Vec<u8>,
    primary_indirect_staging: Vec<u8>,
    primary_indirect_ranges: Vec<std::ops::Range<u32>>,
}
fn fail(s: &str) -> BackendError {
    BackendError::Device(s.into())
}

/// Shared frame bind group: view, lighting, instances and the environment
/// field. Rebuilt when the environment texel buffer is reallocated, because a
/// bind group captures the buffers it was created with.
fn frame_bind_group(
    device: &wgpu::Device,
    pipeline: &wgpu::RenderPipeline,
    view: &wgpu::Buffer,
    lighting: &wgpu::Buffer,
    instances: &wgpu::Buffer,
    environment: &crate::environment::EnvironmentBinding,
) -> wgpu::BindGroup {
    let mut entries = vec![
        wgpu::BindGroupEntry {
            binding: 0,
            resource: view.as_entire_binding(),
        },
        wgpu::BindGroupEntry {
            binding: 1,
            resource: lighting.as_entire_binding(),
        },
        wgpu::BindGroupEntry {
            binding: 2,
            resource: instances.as_entire_binding(),
        },
    ];
    entries.extend(environment.entries());
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("mesh-frame"),
        layout: &pipeline.get_bind_group_layout(0),
        entries: &entries,
    })
}
fn words(values: impl IntoIterator<Item = f32>) -> Vec<u8> {
    values.into_iter().flat_map(f32::to_le_bytes).collect()
}

pub(crate) fn material_texture(
    backend: &Backend,
    source: &MeshAsset,
    id: Option<usize>,
    fallback: [u8; 4],
    srgb: bool,
    cache: &mut std::collections::BTreeMap<(Option<usize>, [u8; 4], bool), wgpu::TextureView>,
) -> (wgpu::TextureView, wgpu::Sampler) {
    let t = id.map(|i| source.textures[i]);
    let image = t.map(|t| t.image);
    let texture = cache
        .entry((image, if image.is_some() { [0; 4] } else { fallback }, srgb))
        .or_insert_with(|| {
            let (width, height, pixels) = image.map_or((1, 1, fallback.as_slice()), |i| {
                let i = &source.images[i];
                (i.width, i.height, i.rgba8.as_slice())
            });
            let levels = thessa_render_core::mesh::Image {
                width,
                height,
                rgba8: pixels.to_vec(),
            }
            .mip_chain(srgb)
            .expect("validated mesh image");
            let texture = backend.device.create_texture(&wgpu::TextureDescriptor {
                label: Some("mesh-material-texture"),
                size: wgpu::Extent3d {
                    width,
                    height,
                    depth_or_array_layers: 1,
                },
                mip_level_count: levels.len() as u32,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: if srgb {
                    wgpu::TextureFormat::Rgba8UnormSrgb
                } else {
                    wgpu::TextureFormat::Rgba8Unorm
                },
                usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
                view_formats: &[],
            });
            for (level, image) in levels.iter().enumerate() {
                backend.queue.write_texture(
                    wgpu::TexelCopyTextureInfo {
                        mip_level: level as u32,
                        ..texture.as_image_copy()
                    },
                    &image.rgba8,
                    wgpu::TexelCopyBufferLayout {
                        offset: 0,
                        bytes_per_row: Some(image.width * 4),
                        rows_per_image: Some(image.height),
                    },
                    wgpu::Extent3d {
                        width: image.width,
                        height: image.height,
                        depth_or_array_layers: 1,
                    },
                );
            }
            texture.create_view(&Default::default())
        })
        .clone();
    let wrap = |w| match w {
        Wrap::Clamp => wgpu::AddressMode::ClampToEdge,
        Wrap::Mirror => wgpu::AddressMode::MirrorRepeat,
        Wrap::Repeat => wgpu::AddressMode::Repeat,
    };
    let sampler = backend.device.create_sampler(&wgpu::SamplerDescriptor {
        address_mode_u: wrap(t.map_or(Wrap::Repeat, |t| t.wrap[0])),
        address_mode_v: wrap(t.map_or(Wrap::Repeat, |t| t.wrap[1])),
        mag_filter: if t.is_none_or(|t| t.linear) {
            wgpu::FilterMode::Linear
        } else {
            wgpu::FilterMode::Nearest
        },
        mipmap_filter: if t.is_none_or(|t| t.min_linear) {
            wgpu::MipmapFilterMode::Linear
        } else {
            wgpu::MipmapFilterMode::Nearest
        },
        min_filter: if t.is_none_or(|t| t.min_linear) {
            wgpu::FilterMode::Linear
        } else {
            wgpu::FilterMode::Nearest
        },
        ..Default::default()
    });
    (texture, sampler)
}
impl MeshRenderer {
    /// Ray-tracing accounting for telemetry; inactive when RT is off or was
    /// rejected, with `rt_reason` explaining a rejection.
    pub fn rt_stats(&self) -> crate::ray_scene::RayStats {
        self.rt.as_ref().map_or_else(
            crate::ray_scene::RayStats::default,
            crate::ray_scene::RayScene::stats,
        )
    }

    pub fn rt_reason(&self) -> Option<&str> {
        self.rt_reason.as_deref()
    }

    pub fn rt_terrain_reason(&self) -> Option<&str> {
        self.rt
            .as_ref()
            .and_then(crate::ray_scene::RayScene::terrain_reason)
    }

    /// Publish generated-patch membership after mesh updates, before encoding.
    /// Matching conforming sources follow their emission epoch automatically;
    /// this call only validates their identity and performs no patch sync. Height
    /// generation submissions precede these builds on the same queue.
    pub fn sync_rt_terrain(
        &mut self,
        backend: &Backend,
        terrain: &crate::terrain::TerrainRenderer,
    ) -> Result<(), BackendError> {
        if let Some(rt) = &mut self.rt {
            rt.sync_terrain(backend, terrain)?;
        }
        Ok(())
    }

    /// Reuse unchanged terrain BLASes when only the immutable mesh asset was
    /// replaced (for example, a new fleet appearance epoch).
    pub fn adopt_rt_terrain(&mut self, previous: &mut Self) -> bool {
        match (&mut self.rt, &mut previous.rt) {
            (Some(next), Some(old)) => next.adopt_terrain(old),
            _ => false,
        }
    }

    /// Bind the radiance environment field, or `None` for the disabled state
    /// that reproduces the legacy isotropic `ambient_lux / PI` reflection
    /// term exactly. No pipeline or layout rebuild; buffer growth/release
    /// rebuilds the frame bind group. Disabled retains only a 16-byte texel
    /// placeholder and the metadata uniform.
    pub fn update_environment(
        &mut self,
        backend: &Backend,
        field: Option<&thessa_render_core::environment::EnvironmentRadiance>,
    ) {
        if self.environment.update(backend, field) {
            self.global = frame_bind_group(
                &backend.device,
                &self.pipeline,
                &self.view,
                &self.lighting,
                &self.instances,
                &self.environment,
            );
        }
    }

    /// Adopt shared scene buffers, without staging or duplicating the grid.
    /// Call after publication (and on renderer replacement), not every frame.
    pub fn share_environment(
        &mut self,
        backend: &Backend,
        binding: &crate::environment::EnvironmentBinding,
    ) {
        self.environment = binding.clone();
        self.global = frame_bind_group(
            &backend.device,
            &self.pipeline,
            &self.view,
            &self.lighting,
            &self.instances,
            &self.environment,
        );
    }

    /// Bound environment field: `(enabled, width, height, storage_bytes, uploads)`.
    pub fn environment_stats(&self) -> (bool, u32, u32, u64, u64) {
        let (width, height) = self.environment.dimensions();
        (
            self.environment.enabled(),
            width,
            height,
            self.environment.storage_bytes(),
            self.environment.uploads(),
        )
    }

    /// Cos-weighted hemispherical irradiance of the bound field, in lux;
    /// zero while disabled.
    pub fn environment_ambient_lux(&self) -> [f32; 3] {
        self.environment.ambient_lux()
    }

    pub fn draw_stats(&self) -> MeshDrawStats {
        self.primary_stats
    }

    fn prepare_primary(&mut self, backend: &Backend, clip: glam::Mat4) -> usize {
        self.primary_seen.resize(self.count as usize, false);
        self.primary_seen.fill(false);
        let mut stats = MeshDrawStats {
            candidate_instances: self.count,
            frame_upload_bytes: self.primary_stats.frame_upload_bytes,
            ..Default::default()
        };
        for (primitive_index, ((primitive, range), runs)) in self
            .primitives
            .iter()
            .zip(&self.ranges)
            .zip(&mut self.primary_runs)
            .enumerate()
        {
            runs.clear();
            let triangles = u64::from(primitive.count) / 3;
            stats.candidate_triangles += triangles * u64::from(range.end - range.start);
            let visible = |i: u32| {
                if let Some(Some(world)) =
                    self.world_bounds[primitive_index].get((i - range.start) as usize)
                {
                    return world.intersects(clip, 0.0);
                }
                primitive.bounds.is_none_or(|bounds| {
                    self.staging
                        .get(i as usize * 64..i as usize * 64 + 64)
                        .is_none_or(|bytes| bounds.intersects_uploaded(clip, bytes))
                })
            };
            for run in crate::mesh_visibility::visible_runs(range.clone(), visible) {
                self.primary_seen[run.start as usize..run.end as usize].fill(true);
                stats.triangles += triangles * u64::from(run.end - run.start);
                runs.push(run);
            }
            stats.draw_commands += runs.len();
        }
        stats.instances = self.primary_seen.iter().filter(|&&seen| seen).count() as u32;
        let uploaded = self.prepare_primary_indirect(backend);
        stats.draw_calls = if self.primary_indirect.is_some() {
            self.primary_runs.iter().filter(|r| !r.is_empty()).count()
        } else {
            stats.draw_commands
        };
        self.primary_stats = stats;
        uploaded
    }

    fn prepare_primary_indirect(&mut self, backend: &Backend) -> usize {
        if !self.primary_indirect_supported {
            return 0;
        }
        self.primary_indirect_staging.clear();
        for ((primitive, runs), range) in self
            .primitives
            .iter()
            .zip(&self.primary_runs)
            .zip(&mut self.primary_indirect_ranges)
        {
            let first = (self.primary_indirect_staging.len() / 20) as u32;
            for run in runs {
                self.primary_indirect_staging.extend(
                    [primitive.count, run.end - run.start, 0, 0, run.start]
                        .into_iter()
                        .flat_map(u32::to_le_bytes),
                );
            }
            *range = first..(self.primary_indirect_staging.len() / 20) as u32;
        }
        let needed = self.primary_indirect_staging.len();
        if needed == 0 {
            return 0;
        }
        let limit = backend.device.limits().max_buffer_size;
        if needed as u64 > limit || needed / 20 > u32::MAX as usize {
            self.primary_indirect = None;
            self.primary_indirect_capacity = 0;
            return 0;
        }
        let grew = needed > self.primary_indirect_capacity || self.primary_indirect.is_none();
        if grew {
            self.primary_indirect_capacity =
                (needed.next_power_of_two() as u64).min(limit) as usize;
            self.primary_indirect = Some(backend.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("mesh-primary-indirect"),
                size: self.primary_indirect_capacity as u64,
                usage: wgpu::BufferUsages::INDIRECT | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }));
        }
        if grew || self.primary_indirect_bytes != self.primary_indirect_staging {
            backend.queue.write_buffer(
                self.primary_indirect.as_ref().unwrap(),
                0,
                &self.primary_indirect_staging,
            );
            self.primary_indirect_bytes
                .clone_from(&self.primary_indirect_staging);
            needed
        } else {
            0
        }
    }

    pub fn new(
        backend: &Backend,
        source: &MeshAsset,
        format: wgpu::TextureFormat,
        capacity: usize,
    ) -> Result<Self, BackendError> {
        Self::with_surface(
            backend,
            source,
            SurfaceSettings {
                format,
                capacity,
                quality: thessa_graphics::Quality::Low,
                reflections: false,
                shadows: None,
                ray_tracing: None,
            },
        )
    }

    pub fn with_surface(
        backend: &Backend,
        source: &MeshAsset,
        settings: SurfaceSettings<'_>,
    ) -> Result<Self, BackendError> {
        Self::with_surface_and_terrain(backend, source, settings, None)
    }

    pub fn with_surface_and_terrain(
        backend: &Backend,
        source: &MeshAsset,
        settings: SurfaceSettings<'_>,
        terrain: Option<&crate::terrain_ray::TerrainRaySource>,
    ) -> Result<Self, BackendError> {
        Self::build_surface(backend, source, settings, terrain, false)
    }

    /// Camera-only reprojection for immutable body-fixed geometry. The caller
    /// must reset scene history whenever membership, geometry or poses change.
    /// Moving/deforming meshes must use `with_surface_and_terrain` instead.
    pub fn with_static_surface_and_terrain(
        backend: &Backend,
        source: &MeshAsset,
        settings: SurfaceSettings<'_>,
        terrain: Option<&crate::terrain_ray::TerrainRaySource>,
    ) -> Result<Self, BackendError> {
        Self::build_surface(backend, source, settings, terrain, true)
    }

    fn build_surface(
        backend: &Backend,
        source: &MeshAsset,
        settings: SurfaceSettings<'_>,
        terrain: Option<&crate::terrain_ray::TerrainRaySource>,
        static_geometry: bool,
    ) -> Result<Self, BackendError> {
        let SurfaceSettings {
            format,
            capacity,
            quality,
            reflections,
            shadows,
            ray_tracing,
        } = settings;
        source.validate().map_err(fail)?;
        // Feasibility (device feature, scene sizes, RT memory budget) is
        // decided before the pipeline exists, because the shader include
        // depends on it.
        let rt = crate::ray_scene::plan_with_terrain(
            backend,
            source,
            ray_tracing,
            capacity,
            quality,
            reflections,
            terrain,
        );
        let rt_reason = match &rt {
            crate::ray_scene::Enable::Unavailable(reason) => Some(reason.clone()),
            _ => None,
        };
        let device = &backend.device;
        if capacity == 0
            || capacity > 65536
            || capacity as u64 * 64 > device.limits().max_storage_buffer_binding_size
            || source.images.iter().any(|i| {
                i.width > device.limits().max_texture_dimension_2d
                    || i.height > device.limits().max_texture_dimension_2d
            })
            || source.primitives.iter().any(|p| {
                p.vertices.len() as u64 * 32 > device.limits().max_buffer_size
                    || p.indices.len() as u64 * 4 > device.limits().max_buffer_size
                    || p.indices.len() > u32::MAX as usize
            })
        {
            return Err(fail("mesh capacity/source exceeds device limits"));
        }
        let buffer = |data: &[u8], usage| {
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("mesh-source"),
                contents: data,
                usage,
            })
        };
        let view = buffer(
            &[0; 464],
            wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        );
        let lighting = buffer(
            &[0; 112],
            wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        );
        let instances = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("rigid-instances"),
            size: capacity as u64 * 64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let rt_include = match &rt {
            crate::ray_scene::Enable::Ready(plan) => crate::ray_scene::shader(plan),
            _ => crate::ray_scene::fallback_shader(),
        };
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("owned-mesh"),
            source: wgpu::ShaderSource::Wgsl(
                format!(
                    "{}\n{}\n{}\n{}\n{}",
                    rt_include,
                    thessa_rcbt_core::shaders::OCEAN_WGSL,
                    crate::environment::shader(0),
                    shadows.map_or_else(
                        || crate::shadows::NO_SHADOWS.into(),
                        |_| crate::shadows::receiver_shader(2)
                    ),
                    // Unknown/moving meshes remain motion-invalid. Immutable
                    // geometry shares terrain's camera-only reconstruction.
                    include_str!("mesh.wgsl").replace(
                        "view.exposure,1.0)",
                        if !static_geometry
                            && ray_tracing.is_some_and(|s| s.temporal_motion_vectors)
                        {
                            "view.exposure,0.0)"
                        } else {
                            "view.exposure,1.0)"
                        },
                    )
                )
                .into(),
            ),
        });
        let has_vertex_ao = source
            .primitives
            .iter()
            .any(|p| !p.ambient_visibility.is_empty());
        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("owned-mesh"),
            layout: None,
            vertex: wgpu::VertexState {
                module: &module,
                entry_point: Some("vertex"),
                compilation_options: Default::default(),
                buffers: &[Some(wgpu::VertexBufferLayout {
                    array_stride: 32,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &wgpu::vertex_attr_array![0=>Float32x3,1=>Float32x3,2=>Float32x2],
                }), Some(wgpu::VertexBufferLayout {
                    array_stride: if has_vertex_ao { 4 } else { 0 },
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &wgpu::vertex_attr_array![3=>Float32],
                })],
            },
            primitive: wgpu::PrimitiveState {
                cull_mode: None,
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: Some(true),
                depth_compare: Some(wgpu::CompareFunction::Less),
                stencil: Default::default(),
                bias: Default::default(),
            }),
            multisample: wgpu::MultisampleState { count: ray_tracing.map_or(1,|settings| settings.raster_msaa_samples), ..Default::default() },
            fragment: Some(wgpu::FragmentState {
                module: &module,
                entry_point: Some("fragment"),
                // Opaque two-sided populations need no discard. Specialization
                // lets the driver reject hidden foliage before expensive shading.
                compilation_options: wgpu::PipelineCompilationOptions {
                    constants: &[
                        ("MESH_ALL_DOUBLE_SIDED", f64::from(source.materials.iter().all(|m| m.double_sided))),
                        ("MESH_ENV_SAMPLES", match quality { thessa_graphics::Quality::Low => 4.0, thessa_graphics::Quality::Medium => 8.0, thessa_graphics::Quality::High => 16.0 }),
                        ("MESH_CLOUD_DIRECTIONS", match quality { thessa_graphics::Quality::Low => 1.0, thessa_graphics::Quality::Medium => 2.0, thessa_graphics::Quality::High => 4.0 }),
                    ],
                    ..Default::default()
                },
                targets: &[Some(wgpu::ColorTargetState {
                    format,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            multiview_mask: None,
            cache: None,
        });
        let environment = crate::environment::EnvironmentBinding::new(device);
        let global = frame_bind_group(
            device,
            &pipeline,
            &view,
            &lighting,
            &instances,
            &environment,
        );
        let mut textures = std::collections::BTreeMap::new();
        let mut materials = Vec::new();
        for material in &source.materials {
            let (texture, sampler) = material_texture(
                backend,
                source,
                material.base_texture,
                [255; 4],
                true,
                &mut textures,
            );
            let use_maps = quality == thessa_graphics::Quality::High;
            let (mr, mr_sampler) = material_texture(
                backend,
                source,
                use_maps
                    .then_some(material.metallic_roughness_texture)
                    .flatten(),
                [255; 4],
                false,
                &mut textures,
            );
            let (normal, normal_sampler) = material_texture(
                backend,
                source,
                use_maps.then_some(material.normal_texture).flatten(),
                [128, 128, 255, 255],
                false,
                &mut textures,
            );
            let (ao, ao_sampler) = material_texture(
                backend,
                source,
                use_maps.then_some(material.occlusion_texture).flatten(),
                [255; 4],
                false,
                &mut textures,
            );
            let mut data = words(material.base_color);
            data.extend(u32::from(material.double_sided).to_le_bytes());
            data.extend(
                (match quality {
                    thessa_graphics::Quality::Low => 0u32,
                    thessa_graphics::Quality::Medium => 1,
                    thessa_graphics::Quality::High => 2,
                })
                .to_le_bytes(),
            );
            data.extend(u32::from(reflections).to_le_bytes());
            data.extend(u32::from(use_maps && material.normal_texture.is_some()).to_le_bytes());
            data.extend(words([
                material.metallic,
                material.roughness,
                material.normal_scale,
                material.occlusion_strength,
            ]));
            data.extend(words([
                material.diffuse_transmission,
                material.dielectric_f0,
                0.0,
                0.0,
            ]));
            let params = buffer(&data, wgpu::BufferUsages::UNIFORM);
            materials.push(device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("mesh-material"),
                layout: &pipeline.get_bind_group_layout(1),
                entries: &[
                    wgpu::BindGroupEntry {
                        binding: 0,
                        resource: params.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 1,
                        resource: wgpu::BindingResource::TextureView(&texture),
                    },
                    wgpu::BindGroupEntry {
                        binding: 2,
                        resource: wgpu::BindingResource::Sampler(&sampler),
                    },
                    wgpu::BindGroupEntry {
                        binding: 3,
                        resource: wgpu::BindingResource::TextureView(&mr),
                    },
                    wgpu::BindGroupEntry {
                        binding: 4,
                        resource: wgpu::BindingResource::Sampler(&mr_sampler),
                    },
                    wgpu::BindGroupEntry {
                        binding: 5,
                        resource: wgpu::BindingResource::TextureView(&normal),
                    },
                    wgpu::BindGroupEntry {
                        binding: 6,
                        resource: wgpu::BindingResource::Sampler(&normal_sampler),
                    },
                    wgpu::BindGroupEntry {
                        binding: 7,
                        resource: wgpu::BindingResource::TextureView(&ao),
                    },
                    wgpu::BindGroupEntry {
                        binding: 8,
                        resource: wgpu::BindingResource::Sampler(&ao_sampler),
                    },
                ],
            }));
        }
        let primitives: Vec<_> = source
            .primitives
            .iter()
            .map(|p| Primitive {
                shadow: None,
                bounds: crate::mesh_visibility::Bounds::new(p.vertices.iter().map(|v| v.position)),
                ambient: buffer(
                    &words(if p.ambient_visibility.is_empty() {
                        vec![1.0; if has_vertex_ao { p.vertices.len() } else { 1 }]
                    } else {
                        p.ambient_visibility.clone()
                    }),
                    wgpu::BufferUsages::VERTEX,
                ),
                vertices: buffer(
                    &words(
                        p.vertices
                            .iter()
                            .flat_map(|v| v.position.into_iter().chain(v.normal).chain(v.uv)),
                    ),
                    wgpu::BufferUsages::VERTEX,
                ),
                indices: buffer(
                    &p.indices
                        .iter()
                        .flat_map(|i| i.to_le_bytes())
                        .collect::<Vec<_>>(),
                    wgpu::BufferUsages::INDEX,
                ),
                count: p.indices.len() as u32,
                material: p.material,
            })
            .collect();
        let rt = match rt {
            crate::ray_scene::Enable::Ready(plan) => Some(crate::ray_scene::RayScene::new(
                backend,
                *plan,
                &pipeline.get_bind_group_layout(3),
                source,
                &mut textures,
            )?),
            _ => None,
        };
        Ok(Self {
            shadow_queue: shadows.map(|_| backend.queue.clone()),
            shadow_command_budget: if shadows.is_some() {
                ray_tracing.map_or(0, |s| s.shadow_mesh_command_budget_bytes)
            } else {
                0
            },
            shadow_available: shadows.map(|s| s.command_available.clone()),
            shadow_indirect: None,
            shadow_layer_capacity: 0,
            shadow_commands: Default::default(),
            uploaded_instances: Vec::new(),
            uploaded_origin: None,
            last_instances: Vec::new(),
            shadow_revision: 0,
            rt,
            rt_reason,
            environment,
            shadow_receiver: shadows
                .map(|s| s.receiver(backend, &pipeline.get_bind_group_layout(2))),
            shadow_caster: shadows.map(|s| {
                backend
                    .device
                    .create_bind_group(&wgpu::BindGroupDescriptor {
                        label: Some("mesh-shadow-source"),
                        layout: &s.mesh_layout,
                        entries: &[wgpu::BindGroupEntry {
                            binding: 0,
                            resource: instances.as_entire_binding(),
                        }],
                    })
            }),
            world_bounds: vec![Vec::new(); primitives.len()],
            cache_bounds: shadows.is_some(),
            primary_runs: vec![Vec::new(); primitives.len()],
            primary_seen: Vec::with_capacity(capacity),
            primary_stats: Default::default(),
            primary_view: None,
            primary_indirect_supported: backend
                .device
                .features()
                .contains(wgpu::Features::INDIRECT_FIRST_INSTANCE)
                && backend
                    .adapter
                    .get_downlevel_capabilities()
                    .flags
                    .contains(wgpu::DownlevelFlags::INDIRECT_EXECUTION),
            primary_indirect: None,
            primary_indirect_capacity: 0,
            primary_indirect_bytes: Vec::new(),
            primary_indirect_staging: Vec::new(),
            primary_indirect_ranges: vec![0..0; primitives.len()],
            ranges: vec![0..0; primitives.len()],
            pipeline,
            global,
            materials,
            primitives,
            view,
            lighting,
            instances,
            capacity,
            count: 0,
            staging: Vec::with_capacity(capacity * 64),
        })
    }
    fn prepare_world_bounds(&mut self) {
        if !self.cache_bounds {
            return;
        }
        for ((p, range), cache) in self
            .primitives
            .iter()
            .zip(&self.ranges)
            .zip(&mut self.world_bounds)
        {
            cache.clear();
            cache.extend(range.clone().map(|i| {
                p.bounds.and_then(|bounds| {
                    self.uploaded_instances
                        .get(i as usize * 64..i as usize * 64 + 64)
                        .and_then(|bytes| bounds.uploaded_world(bytes))
                })
            }));
        }
    }
    pub(crate) fn matches_receiver_view(&self, clip: glam::Mat4, origin: [f64; 3]) -> bool {
        self.primary_view == Some((glam::DVec3::from_array(origin), clip))
    }
    pub(crate) fn receiver_error_m(&self) -> Option<f64> {
        self.world_bounds
            .iter()
            .flatten()
            .try_fold(0.0_f64, |error, world| {
                let bound = world.as_ref()?.receiver_error_m();
                bound.is_finite().then_some(error.max(bound))
            })
    }
    pub fn shadow_bounds_cpu_storage_bytes(&self) -> usize {
        self.world_bounds
            .iter()
            .map(|c| {
                c.capacity() * std::mem::size_of::<Option<crate::mesh_visibility::WorldBounds>>()
            })
            .sum::<usize>()
            + self.world_bounds.capacity()
                * std::mem::size_of::<Vec<Option<crate::mesh_visibility::WorldBounds>>>()
    }

    /// Allocate within the resolved unused atlas budget, using validated range
    /// lengths as a hard upper bound on one command per instance per layer.
    fn prepare_shadow_indirect(&mut self, backend: &Backend) {
        if self.shadow_command_budget == 0
            || !self.primary_indirect_supported
            || self.shadow_queue.is_none()
        {
            self.shadow_indirect = None;
            self.shadow_layer_capacity = 0;
            return;
        }
        let needed = self.ranges.iter().try_fold(0_usize, |sum, r| {
            sum.checked_add((r.end - r.start) as usize)
        });
        let available = self.shadow_available.as_ref().unwrap().clone();
        let shared_capacity = *available.lock().unwrap_or_else(|p| p.into_inner())
            + self.shadow_indirect_storage_bytes();
        let max_capacity = self
            .shadow_command_budget
            .min(shared_capacity)
            .min(backend.device.limits().max_buffer_size)
            / (12 * 20);
        let Some(needed) =
            needed.filter(|&n| n > 0 && n as u64 <= max_capacity && n <= u32::MAX as usize)
        else {
            self.shadow_indirect = None;
            self.shadow_layer_capacity = 0;
            return;
        };
        if self.shadow_indirect.is_none() || needed > self.shadow_layer_capacity {
            self.shadow_layer_capacity = needed.next_power_of_two().min(max_capacity as usize);
            self.shadow_indirect = None; // Return the previous reservation before growing.
            let size = (self.shadow_layer_capacity * 12 * 20) as u64;
            let mut remaining = available.lock().unwrap_or_else(|p| p.into_inner());
            if *remaining < size {
                self.shadow_layer_capacity = 0;
                return;
            }
            *remaining -= size;
            drop(remaining);
            self.shadow_indirect = Some(crate::shadows::MeshCommandStorage {
                available,
                buffer: backend.device.create_buffer(&wgpu::BufferDescriptor {
                    // Queue writes precede the entire submission: each layer owns
                    // a disjoint command region until this frame is submitted.
                    label: Some("mesh-shadow-indirect"),
                    size,
                    usage: wgpu::BufferUsages::INDIRECT | wgpu::BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                }),
            });
            self.shadow_commands
                .get_mut()
                .unwrap_or_else(|p| p.into_inner())
                .uploaded = vec![Vec::new(); 12];
        }
    }

    pub fn shadow_indirect_storage_bytes(&self) -> u64 {
        self.shadow_indirect.as_ref().map_or(0, |s| s.buffer.size())
    }

    /// Validate the complete frame before any write; errors retain the previous
    /// draw count and data. Empty publication clears instances without a pass.
    pub fn update(
        &mut self,
        backend: &Backend,
        view: &ViewParams,
        lighting: &SurfaceLighting,
        instances: &[MeshInstance],
    ) -> Result<(), BackendError> {
        self.update_selected(backend, view, lighting, instances, None)
    }

    /// Optional per-primitive instance ranges allow heterogeneous sources in
    /// the existing single pass/pipeline. Validate before publishing any bytes.
    pub fn update_selected(
        &mut self,
        backend: &Backend,
        view: &ViewParams,
        lighting: &SurfaceLighting,
        instances: &[MeshInstance],
        ranges: Option<&[std::ops::Range<u32>]>,
    ) -> Result<(), BackendError> {
        if let Some(ranges) = ranges
            && (ranges.len() != self.primitives.len()
                || ranges
                    .iter()
                    .any(|r| r.start > r.end || r.end as usize > instances.len()))
        {
            return Err(fail("invalid mesh primitive instance ranges"));
        }
        if instances.len() > self.capacity {
            return Err(fail("mesh instance capacity exceeded"));
        }
        let block = ViewUniformBlock::new(view).ok_or_else(|| fail("invalid mesh camera"))?;
        let lights = lighting
            .records()
            .ok_or_else(|| fail("invalid mesh lighting"))?;
        if self.uploaded_origin == Some(view.world_position_m)
            && self.last_instances.as_slice() == instances
        {
            // A failed prior publication may have left partial scratch bytes.
            self.staging.clone_from(&self.uploaded_instances);
        } else {
            self.staging.clear();
            for instance in instances {
                let matrix = instance
                    .camera_relative_matrix(view.world_position_m)
                    .ok_or_else(|| fail("invalid rigid mesh instance"))?;
                self.staging.extend(
                    matrix
                        .to_cols_array()
                        .into_iter()
                        .flat_map(f32::to_le_bytes),
                );
            }
        }
        backend.queue.write_buffer(&self.view, 0, &block.to_bytes());
        let mut light_bytes = [0u8; 112];
        for (out, word) in light_bytes
            .as_chunks_mut::<4>()
            .0
            .iter_mut()
            .zip(lights.into_iter().flatten())
        {
            out.copy_from_slice(&word.to_le_bytes());
        }
        backend.queue.write_buffer(&self.lighting, 0, &light_bytes);
        let uploaded_changed = self.uploaded_instances != self.staging;
        let upload_bytes = if uploaded_changed {
            self.staging.len()
        } else {
            0
        };
        if uploaded_changed && !self.staging.is_empty() {
            backend
                .queue
                .write_buffer(&self.instances, 0, &self.staging);
        }
        self.uploaded_origin = Some(view.world_position_m);
        self.count = instances.len() as u32;
        let mut shadow_changed = false;
        for (i, range) in self.ranges.iter_mut().enumerate() {
            let next = ranges.map_or(0..self.count, |ranges| ranges[i].clone());
            shadow_changed |= *range != next;
            *range = next;
        }
        if self.last_instances.as_slice() != instances {
            self.last_instances.clear();
            self.last_instances.extend_from_slice(instances);
            shadow_changed = true;
        }
        if self.shadow_caster.is_some() {
            shadow_changed |= uploaded_changed;
            if shadow_changed {
                self.shadow_revision = self.shadow_revision.wrapping_add(1);
            }
        }
        if uploaded_changed {
            self.uploaded_instances.clone_from(&self.staging);
        }
        if shadow_changed || uploaded_changed {
            self.prepare_world_bounds();
        }
        self.prepare_shadow_indirect(backend);
        let primary_view = (view.world_position_m, block.clip_from_world);
        let primary_upload = if shadow_changed || self.primary_view != Some(primary_view) {
            let bytes = self.prepare_primary(backend, block.clip_from_world);
            self.primary_view = Some(primary_view);
            bytes
        } else {
            0
        };
        self.primary_stats.frame_upload_bytes = 464 + 112 + upload_bytes + primary_upload;
        if let Some(rt) = &mut self.rt {
            rt.update(
                &backend.queue,
                view.world_position_m,
                instances,
                &self.ranges,
            )?;
        }
        Ok(())
    }

    pub(crate) fn shadow_key(&self) -> Option<(wgpu::Buffer, u64)> {
        self.shadow_caster
            .as_ref()
            .map(|_| (self.instances.clone(), self.shadow_revision))
    }

    /// Override only primary camera sampling after the ordinary instance update,
    /// before encoding. Does not touch instance/shadow/RT state. The zero-jitter
    /// path needs no extra upload because `update` already publishes that view.
    pub fn update_primary_jitter(
        &mut self,
        backend: &Backend,
        view: &ViewParams,
        extent: [u32; 2],
        jitter: thessa_render_core::temporal::JitterOffset,
    ) -> Result<(), BackendError> {
        if jitter.is_zero() {
            return Ok(());
        }
        let block = ViewUniformBlock::new_jittered(view, view.world_position_m, jitter, extent)
            .ok_or_else(|| fail("invalid primary mesh camera"))?;
        backend.queue.write_buffer(&self.view, 0, &block.to_bytes());
        let primary_view = (view.world_position_m, block.clip_from_world);
        let primary_upload = if self.primary_view != Some(primary_view) {
            let bytes = self.prepare_primary(backend, block.clip_from_world);
            self.primary_view = Some(primary_view);
            bytes
        } else {
            0
        };
        self.primary_stats.frame_upload_bytes += 464 + primary_upload;
        Ok(())
    }
    /// Encode acceleration-structure builds (TLAS/BLAS) outside the render pass,
    /// before draw_into_pass(). Empty draw sets do no AS work.
    pub fn encode_rt(&mut self, encoder: &mut wgpu::CommandEncoder) {
        if self.count == 0 || self.ranges.iter().all(std::ops::Range::is_empty) {
            return;
        }
        if let Some(rt) = &mut self.rt {
            rt.encode(encoder);
        }
    }

    pub fn encode(
        &mut self,
        encoder: &mut wgpu::CommandEncoder,
        color: &wgpu::TextureView,
        depth: &wgpu::TextureView,
    ) {
        self.encode_profiled(encoder, color, depth, None);
    }

    /// Optional three-mark hardware diagnostic: AS builds then mesh draws.
    /// No query writes when no mesh is drawable. Caller supplies three marks,
    /// completes submission and reads; ordinary encoding allocates no profiler.
    pub fn encode_profiled(
        &mut self,
        encoder: &mut wgpu::CommandEncoder,
        color: &wgpu::TextureView,
        depth: &wgpu::TextureView,
        timing: Option<&crate::timing::GpuIntervals>,
    ) {
        if self.count == 0 || self.ranges.iter().all(std::ops::Range::is_empty) {
            return;
        }
        // Acceleration-structure builds are encoder-level state and must
        // precede the pass that traces them.
        if let Some(t) = timing {
            t.mark(encoder, 0);
        }
        self.encode_rt(encoder);
        if let Some(t) = timing {
            t.mark(encoder, 1);
        }
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("owned-rigid-mesh"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: color,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Load,
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: depth,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Load,
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
            multiview_mask: None,
        });
        self.draw_into_pass(&mut pass);
        drop(pass);
        if let Some(t) = timing {
            t.mark(encoder, 2);
            t.resolve(encoder);
        }
    }

    /// Draw rigid mesh instances directly into an existing compatible render pass.
    /// Caller must first encode_rt() outside that pass. Attachments must match
    /// construction (color format, Depth32Float and single-sample raster).
    pub fn draw_into_pass<'a>(&'a self, pass: &mut wgpu::RenderPass<'a>) {
        self.draw_with_visibility(pass, true);
    }

    fn draw_with_visibility<'a>(&'a self, pass: &mut wgpu::RenderPass<'a>, cull: bool) {
        if self.count == 0 || self.ranges.iter().all(std::ops::Range::is_empty) {
            return;
        }
        pass.set_pipeline(&self.pipeline);
        pass.set_bind_group(0, &self.global, &[]);
        if let Some(receiver) = &self.shadow_receiver {
            pass.set_bind_group(2, receiver, &[]);
        }
        if let Some(rt) = &self.rt {
            pass.set_bind_group(3, rt.bind_group(), &[]);
        }
        for (i, ((p, range), visible)) in self
            .primitives
            .iter()
            .zip(&self.ranges)
            .zip(&self.primary_runs)
            .enumerate()
        {
            let runs = if cull {
                visible.as_slice()
            } else {
                std::slice::from_ref(range)
            };
            if runs.is_empty() || range.is_empty() {
                continue;
            }
            pass.set_bind_group(1, &self.materials[p.material], &[]);
            pass.set_vertex_buffer(0, p.vertices.slice(..));
            pass.set_vertex_buffer(1, p.ambient.slice(..));
            pass.set_index_buffer(p.indices.slice(..), wgpu::IndexFormat::Uint32);
            if let Some(indirect) = self.primary_indirect.as_ref().filter(|_| cull) {
                let range = &self.primary_indirect_ranges[i];
                pass.multi_draw_indexed_indirect(
                    indirect,
                    u64::from(range.start) * 20,
                    range.end - range.start,
                );
            } else {
                for run in runs {
                    pass.draw_indexed(0..p.count, 0, run.clone());
                }
            }
        }
    }

    /// Atomically replace optional depth-only geometry. The caller certifies
    /// max_error_m against the original primitive, in both surface directions.
    pub fn set_shadow_lods(
        &mut self,
        backend: &Backend,
        lods: &[thessa_render_core::mesh::ShadowMeshLod],
    ) -> Result<(), BackendError> {
        let mut seen = std::collections::BTreeSet::new();
        for lod in lods {
            let Some(p) = self.primitives.get(lod.primitive) else {
                return Err(fail("shadow LOD primitive out of range"));
            };
            if !seen.insert(lod.primitive)
                || !lod.max_error_m.is_finite()
                || lod.max_error_m < 0.0
                || lod.positions.len() as u64 * 12 > backend.device.limits().max_buffer_size
                || lod.indices.len() as u64 * 4 > backend.device.limits().max_buffer_size
                || lod.positions.len() > u32::MAX as usize
                || lod.positions.is_empty()
                || lod.positions.iter().flatten().any(|v| !v.is_finite())
                || lod.indices.is_empty()
                || !lod.indices.len().is_multiple_of(3)
                || lod.indices.len() >= p.count as usize
                || lod
                    .indices
                    .iter()
                    .any(|&i| i as usize >= lod.positions.len())
                || p.bounds.is_none()
            {
                return Err(fail("invalid depth-only mesh LOD"));
            }
        }
        let mut replacements: Vec<Option<ShadowPrimitive>> =
            (0..self.primitives.len()).map(|_| None).collect();
        for (i, lod) in lods.iter().enumerate() {
            let buffer = |contents: &[u8], usage| {
                backend
                    .device
                    .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                        label: Some("mesh-shadow-lod"),
                        contents,
                        usage,
                    })
            };
            let shared = lods[..i]
                .iter()
                .find(|a| a.same_geometry(lod))
                .and_then(|a| replacements[a.primitive].as_ref());
            let (vertices, indices) = if let Some(shared) = shared {
                (shared.vertices.clone(), shared.indices.clone())
            } else {
                (
                    buffer(
                        &words(lod.positions.iter().flatten().copied()),
                        wgpu::BufferUsages::VERTEX,
                    ),
                    buffer(
                        &lod.indices
                            .iter()
                            .flat_map(|i| i.to_le_bytes())
                            .collect::<Vec<_>>(),
                        wgpu::BufferUsages::INDEX,
                    ),
                )
            };
            replacements[lod.primitive] = Some(ShadowPrimitive {
                vertices,
                indices,
                count: lod.indices.len() as u32,
                max_error_m: lod.max_error_m,
            });
        }
        for (p, shadow) in self.primitives.iter_mut().zip(replacements) {
            p.shadow = shadow;
        }
        self.shadow_revision = self.shadow_revision.wrapping_add(1);
        Ok(())
    }

    pub(crate) fn cast_shadow(
        &self,
        pass: &mut wgpu::RenderPass<'_>,
        atlas: &crate::shadows::ShadowAtlas,
        clip: Option<glam::Mat4>,
        visibility_clip: Option<&thessa_render_core::shadow::ReceiverVolume>,
        layer: usize,
        mut observe: impl FnMut(usize, crate::shadows::ShadowDrawStats),
    ) -> crate::shadows::ShadowDrawStats {
        let mut total = crate::shadows::ShadowDrawStats::default();
        let Some(bind) = &self.shadow_caster else {
            return total;
        };
        let projection = if atlas.mesh_lod_pipeline.is_some() && atlas.mesh_lod_error_texels > 0.0 {
            clip.and_then(|c| crate::mesh_visibility::LightProjection::new(c, atlas.map_size()))
        } else {
            None
        };
        let indirect = clip.is_some() && layer < 12 && self.shadow_indirect.is_some();
        let mut commands = indirect.then(|| {
            self.shadow_commands
                .lock()
                .unwrap_or_else(|p| p.into_inner())
        });
        if let Some(s) = commands.as_mut() {
            s.commands.clear();
            s.batches.clear();
        }
        pass.set_pipeline(&atlas.mesh_pipeline);
        pass.set_bind_group(1, bind, &[]);
        for (primitive, (p, range)) in self.primitives.iter().zip(&self.ranges).enumerate() {
            let mut stats = crate::shadows::ShadowDrawStats::default();
            if range.is_empty() {
                continue;
            }
            let first_command = commands.as_ref().map_or(0, |s| s.commands.len() / 20) as u32;
            if let Some(s) = commands.as_mut() {
                s.alternate.clear();
            }
            stats.mesh_indirect = indirect;
            let triangles = u64::from(p.count) / 3;
            stats.mesh_candidate_instances += u64::from(range.end - range.start);
            stats.mesh_candidate_triangles += triangles * u64::from(range.end - range.start);
            let selection = |i: u32| -> Option<(bool, f64)> {
                let Some(Some(world)) =
                    self.world_bounds[primitive].get((i - range.start) as usize)
                else {
                    return Some((false, 0.0)); // Missing/unbounded data keeps full geometry.
                };
                let max_error = p.shadow.as_ref().map_or(0.0, |p| p.max_error_m);
                if let Some(volume) = visibility_clip
                    && !world.intersects_receivers(volume, max_error)
                {
                    return None;
                }
                let error = projection.and_then(|projection| {
                    p.shadow
                        .as_ref()
                        .and_then(|lod| world.projected_error(projection, lod.max_error_m))
                });
                let lod = error.is_some_and(|e| e <= atlas.mesh_lod_error_texels);
                if visibility_clip.is_none()
                    && let Some(clip) = clip
                    && !world.intersects(clip, if lod { max_error } else { 0.0 })
                {
                    return None;
                }
                Some((lod, if lod { error.unwrap() } else { 0.0 }))
            };
            let mut run: Option<(u32, u32, bool)> = None;
            let mut draw = |start, end, lod| {
                let (vertices, indices, count) = if lod {
                    let p = p.shadow.as_ref().unwrap();
                    (&p.vertices, &p.indices, p.count)
                } else {
                    (&p.vertices, &p.indices, p.count)
                };
                if let Some(s) = commands.as_mut() {
                    let target = if lod {
                        &mut s.alternate
                    } else {
                        &mut s.commands
                    };
                    target.extend(
                        [count, end - start, 0, 0, start]
                            .into_iter()
                            .flat_map(u32::to_le_bytes),
                    );
                } else {
                    pass.set_pipeline(if lod {
                        atlas.mesh_lod_pipeline.as_ref().unwrap()
                    } else {
                        &atlas.mesh_pipeline
                    });
                    pass.set_vertex_buffer(0, vertices.slice(..));
                    pass.set_index_buffer(indices.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..count, 0, start..end);
                    stats.mesh_draw_calls += 1;
                }
                stats.mesh_draw_commands += 1;
                stats.mesh_submitted_instances += u64::from(end - start);
                stats.mesh_submitted_triangles += u64::from(count / 3) * u64::from(end - start);
                if lod {
                    stats.mesh_lod_instances += u64::from(end - start);
                    stats.mesh_lod_triangles_saved +=
                        (triangles - u64::from(count / 3)) * u64::from(end - start);
                }
            };
            for i in range.clone() {
                let selected = selection(i);
                if let Some((_, error)) = selected {
                    stats.mesh_lod_max_error_microtexels = stats
                        .mesh_lod_max_error_microtexels
                        .max((error * 1_000_000.0).ceil() as u64);
                }
                match (run, selected) {
                    (Some((start, _, a)), Some((b, _))) if a == b => run = Some((start, i + 1, a)),
                    (Some((start, end, lod)), selected) => {
                        draw(start, end, lod);
                        run = selected.map(|(lod, _)| (i, i + 1, lod));
                    }
                    (None, selected) => run = selected.map(|(lod, _)| (i, i + 1, lod)),
                }
            }
            if let Some((start, end, lod)) = run {
                draw(start, end, lod);
            }
            if let Some(s) = commands.as_mut() {
                let s = &mut **s;
                let full_end = (s.commands.len() / 20) as u32;
                if first_command < full_end {
                    s.batches.push((primitive, false, first_command..full_end));
                    stats.mesh_draw_calls += 1;
                }
                s.commands.append(&mut s.alternate);
                let end = (s.commands.len() / 20) as u32;
                if full_end < end {
                    s.batches.push((primitive, true, full_end..end));
                    stats.mesh_draw_calls += 1;
                }
            }
            observe(primitive, stats);
            total.accumulate(stats);
        }
        if let Some(s) = commands.as_mut() {
            let buffer = &self.shadow_indirect.as_ref().unwrap().buffer;
            let base = (layer * self.shadow_layer_capacity * 20) as u64;
            let ShadowCommands {
                commands,
                uploaded,
                batches,
                ..
            } = &mut **s;
            debug_assert!(commands.len() <= self.shadow_layer_capacity * 20);
            if uploaded[layer] != *commands {
                if !commands.is_empty() {
                    self.shadow_queue
                        .as_ref()
                        .unwrap()
                        .write_buffer(buffer, base, commands);
                }
                total.mesh_indirect_upload_bytes = commands.len() as u64;
                uploaded[layer].clone_from(commands);
            }
            for (primitive, lod, range) in batches {
                let p = &self.primitives[*primitive];
                let (vertices, indices) = if *lod {
                    let p = p.shadow.as_ref().unwrap();
                    (&p.vertices, &p.indices)
                } else {
                    (&p.vertices, &p.indices)
                };
                pass.set_pipeline(if *lod {
                    atlas.mesh_lod_pipeline.as_ref().unwrap()
                } else {
                    &atlas.mesh_pipeline
                });
                pass.set_vertex_buffer(0, vertices.slice(..));
                pass.set_index_buffer(indices.slice(..), wgpu::IndexFormat::Uint32);
                pass.multi_draw_indexed_indirect(
                    buffer,
                    base + u64::from(range.start) * 20,
                    range.end - range.start,
                );
            }
        }
        total
    }
}

#[cfg(test)]
#[path = "mesh_primary_tests.rs"]
mod primary_tests;

#[cfg(test)]
#[path = "mesh_shadow_tests.rs"]
mod shadow_tests;
