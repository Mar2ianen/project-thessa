//! Requested -> resolved graphics settings (spec section 17).
//!
//! Separate user intent from runtime capability:
//!
//! ```text
//! graphics.toml -> requested -> preset expansion
//!               -> capability resolution -> ResolvedGraphicsSettings
//!               -> renderer + perf capture metadata
//! ```
//!
//! Unsupported high-end features degrade cleanly instead of failing startup,
//! except explicit RT requests which fail fast: silently dropping a mode the
//! user deliberately enabled would be worse than a clear startup error.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::settings::{
    BackendRequest, Quality, RayTracingRequest, RequestedGraphics, TerrainRenderRequest,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct ResolvedInstruments {
    pub enabled: bool,
    pub layout: crate::settings::InstrumentLayout,
    pub renderer: crate::settings::NavballRenderer,
    pub diameter_points: u32,
    pub atlas_width: u32,
}
impl Default for ResolvedInstruments {
    fn default() -> Self {
        resolve_instruments(Default::default())
    }
}

/// Shared resolver for the native HUD and the full graphics settings. Both
/// paths require only baseline portable fragment/uniform/R8 texture support.
/// Auto selects the texture-free analytic representation; baking is opt-in.
pub fn resolve_instruments(request: crate::settings::InstrumentSettings) -> ResolvedInstruments {
    use crate::settings::NavballRenderer;
    ResolvedInstruments {
        enabled: request.enabled,
        layout: request.layout,
        renderer: match request.navball_renderer {
            NavballRenderer::Auto => NavballRenderer::Analytic,
            v => v,
        },
        diameter_points: if !request.enabled {
            0
        } else {
            match request.quality {
                Quality::Low => 192,
                Quality::Medium => 256,
                Quality::High => 320,
            }
        },
        atlas_width: if !request.enabled || request.navball_renderer != NavballRenderer::Baked {
            0
        } else {
            match request.quality {
                Quality::Low => 512,
                Quality::Medium => 1024,
                Quality::High => 2048,
            }
        },
    }
}

/// Resolved ray-tracing mode. There is no single boolean: `local` buys RT
/// where it has the highest visual value first (spec section 18).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ResolvedRayTracing {
    #[default]
    Off,
    Local,
    Full,
}

impl ResolvedRayTracing {
    pub fn is_active(self) -> bool {
        self != Self::Off
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Off => "off",
            Self::Local => "local",
            Self::Full => "full",
        }
    }
}

/// Resolved render backend after capability mapping.
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct ResolvedBackend {
    /// Backend actually requested from wgpu (`auto` lets wgpu choose).
    pub name: String,
}

/// Terrain raster path selected for this client run.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ResolvedTerrainRender {
    #[default]
    Cpu,
    GpuIndexed,
    GpuMesh,
}

impl ResolvedTerrainRender {
    pub fn is_gpu(self) -> bool {
        matches!(self, Self::GpuIndexed | Self::GpuMesh)
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Cpu => "cpu",
            Self::GpuIndexed => "gpu_indexed",
            Self::GpuMesh => "gpu_mesh",
        }
    }
}

/// Material page storage selected for this client run.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ResolvedMaterialStorage {
    #[default]
    RgbaArray,
    MicrostoreCompact,
}

impl ResolvedMaterialStorage {
    /// Whether pages upload through the compact microstore path.
    pub fn is_microstore(self) -> bool {
        matches!(self, Self::MicrostoreCompact)
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::RgbaArray => "rgba_array",
            Self::MicrostoreCompact => "microstore_compact",
        }
    }
}

/// Active representation used for rendering clouds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ActiveCloudRepresentation {
    #[default]
    Disabled,
    Shell,
    VolumeMarch,
    VolumeStochastic,
}

impl ActiveCloudRepresentation {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Disabled => "disabled",
            Self::Shell => "shell",
            Self::VolumeMarch => "volume_march",
            Self::VolumeStochastic => "volume_stochastic",
        }
    }
}

/// Runtime capability input for resolution.
///
/// Adapter details are only known after renderer init, which happens after
/// plugin registration, so pre-init callers pass `None` (unknown). Unknown
/// never resolves to an enabled RT mode: `auto` stays raster, explicit
/// requests are kept and fail fast at device creation with a clear note.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Capabilities {
    /// Host implements compact material residency (not merely the GPU codec).
    /// Unknown preserves the legacy pre-init path; native must report false
    /// until its production material cache actually consumes compact pages.
    pub material_compact_supported: Option<bool>,
    /// Persistent topology's eight compute storage bindings are available.
    pub terrain_conforming_supported: Option<bool>,
    /// Enabled mesh feature and sufficient terrain shader limits on the device.
    pub terrain_mesh_supported: Option<bool>,
    /// `Some(true)` when the adapter exposes ray-query features.
    pub ray_query_supported: Option<bool>,
    /// `Some(true)` when the adapter is a discrete GPU.
    pub is_discrete_gpu: Option<bool>,
    /// RGBA16Float can be rendered and sampled for a native HDR scene target.
    pub hdr_render_target_supported: Option<bool>,
}

impl Capabilities {
    pub fn unknown() -> Self {
        Self {
            material_compact_supported: None,
            terrain_conforming_supported: None,
            ray_query_supported: None,
            is_discrete_gpu: None,
            terrain_mesh_supported: None,
            hdr_render_target_supported: None,
        }
    }
}

/// Fully resolved settings consumed by the renderer and perf captures.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct ResolvedTerrainTopology {
    /// Zero disables all persistent-topology resources and dispatches.
    pub capacity: u32,
    pub waves: u32,
    pub split_area_px: f32,
    pub merge_area_px: f32,
}
impl ResolvedTerrainTopology {
    fn resolve(
        request: crate::settings::TerrainTopologySettings,
        supported: Option<bool>,
        notes: &mut Vec<String>,
    ) -> Self {
        if !request.enabled {
            return Self::default();
        }
        if supported == Some(false) {
            notes.push(
                "conforming terrain storage/compute limits unsupported; using regular patches"
                    .into(),
            );
            return Self::default();
        }
        let (capacity, waves, split_area_px) = match request.quality {
            Quality::Low => (32768, 1, 1024.0),
            Quality::Medium => (65536, 2, 256.0),
            Quality::High => (131072, 4, 64.0),
        };
        Self {
            capacity,
            waves,
            split_area_px,
            merge_area_px: split_area_px / 8.0,
        }
    }
}

/// Fully resolved settings consumed by the renderer and perf captures.
/// Quality controls budgets (steps, LUT sizes), never physics.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ResolvedGraphicsSettings {
    /// Prepared survey-only plant policy. False must skip sampler, jobs and
    /// mesh work; resolving these values does not allocate instance resources.
    #[serde(default)]
    pub surface_plants_enabled: bool,
    /// Nested visual density subset, never a change to habitat, height or IDs.
    #[serde(default)]
    pub surface_plants_density_scale: f64,
    /// One fixed L14 window contains 4096 canonical candidate cells.
    #[serde(default)]
    pub surface_plants_max_candidates: u64,
    /// Output ceiling, not a reservation or a target population count.
    #[serde(default)]
    pub surface_plants_max_items: u32,
    #[serde(default)]
    pub terrain_topology: ResolvedTerrainTopology,
    pub instruments: ResolvedInstruments,
    pub ocean: crate::settings::OceanSettings,
    pub ocean_size: u32,
    pub rt_scene_memory_bytes: u64,
    pub rt_max_instances: u32,
    pub rt_reflection_bounces: u32,
    pub rt_reflection_samples: u32,
    pub rt_ray_bias_m: f32,
    pub surface_quality: Quality,
    #[serde(default = "crate::settings::default_true")]
    pub terrain_normal_maps: bool,
    #[serde(default)]
    pub terrain_ground_detail: bool,
    pub specular_reflections: bool,
    pub native_shadow_map_size: u32,
    pub shadow_filter_radius: u32,
    pub fleet_meshes: bool,
    /// Screen-size culling fallback, not a change to physical geometry.
    pub fleet_min_pixels: f32,
    pub preset_label: String,
    pub backend: ResolvedBackend,
    pub terrain: ResolvedTerrainRender,
    pub terrain_mesh_cells: u32,
    pub material_storage: ResolvedMaterialStorage,
    pub ray_tracing: ResolvedRayTracing,
    pub resolution_scale: f32,
    pub vsync: bool,
    pub hdr: bool,
    pub bloom: bool,
    pub exposure_ev100: f32,
    pub auto_exposure: bool,
    pub atmosphere_enabled: bool,
    pub atmosphere_quality: Quality,
    pub atmosphere_steps: u32,
    pub aerial_perspective: bool,
    pub multiple_scattering: bool,
    pub eclipses: bool,
    pub multi_star: bool,
    pub limb_scattering: bool,
    pub sky_dome: bool,
    pub stars: bool,
    pub stars_quality: Quality,
    pub atmosphere_density_scale: f32,
    pub airglow: bool,
    pub aurora_shell: bool,
    pub aurora_lighting: bool,
    pub aurora_quality: crate::settings::AuroraQuality,
    pub aurora_intensity: f32,
    pub aurora_animate: bool,
    pub clouds_enabled: bool,
    pub clouds_quality: Quality,
    /// Numerical integrator for the volume path (March vs Stochastic).
    pub cloud_integrator: crate::settings::CloudIntegratorRequest,
    /// Volume-march the low-deck slab (false = shell impostors). Resolved
    /// from `volumetric && enabled`, with Low quality falling back to shells.
    pub clouds_volumetric: bool,
    /// View-march steps for the volume path (shells ignore it).
    pub clouds_steps: u32,
    pub clouds_layers: u32,
    pub clouds_coverage: f32,
    pub clouds_opacity: f32,
    pub clouds_animate: bool,
    pub clouds_cast_shadows: bool,
    pub night_lights_enabled: bool,
    pub night_lights_quality: Quality,
    pub night_lights_intensity: f32,
    pub gas_giant_enabled: bool,
    pub gas_giant_quality: Quality,
    pub gas_giant_animate: bool,
    pub gas_giant_limb: bool,
    pub plume_enabled: bool,
    pub plume_quality: Quality,
    pub plume_diamonds: bool,
    pub plume_flicker: bool,
    pub plume_light: bool,
    pub rt_terrain: bool,
    pub rt_vehicles: bool,
    pub rt_landmarks: bool,
    pub rt_atmosphere_queries: bool,
    pub rt_clouds: bool,
    pub rt_max_distance_m: f64,
    pub shadow_enabled: bool,
    pub shadow_cascades: u32,
    pub shadow_max_distance_m: f64,
    pub shadow_map_size: u32,
    pub shadow_normal_bias_m: f64,
    /// Human-readable fallback notes, also stored in captures.
    #[serde(default)]
    pub notes: Vec<String>,
}

impl Default for ResolvedGraphicsSettings {
    fn default() -> Self {
        Self::from_requested(&RequestedGraphics::default(), &Capabilities::unknown())
    }
}

#[cfg(test)]
mod terrain_topology_tests {
    use super::*;
    #[test]
    fn ground_detail_falls_back_until_conforming_coordinates_are_precise() {
        let mut request = RequestedGraphics::default();
        request.renderer.terrain_ground_detail = true;
        request.terrain_topology.enabled = true;
        let settings = ResolvedGraphicsSettings::from_requested(&request, &Capabilities::unknown());
        assert!(!settings.terrain_ground_detail);
        assert_eq!(settings.terrain_material_max_level(), 17);
        assert!(
            settings
                .notes
                .iter()
                .any(|note| note.contains("ground detail requires regular-patch"))
        );
        let fallback = ResolvedGraphicsSettings::from_requested(
            &request,
            &Capabilities {
                terrain_conforming_supported: Some(false),
                ..Default::default()
            },
        );
        assert!(fallback.terrain_ground_detail);
        assert!(fallback.terrain_material_max_level() > 17);
    }
    #[test]
    fn normal_map_policy_and_filter_tiers_roundtrip() {
        for (quality, bias) in [(Quality::Low, 2), (Quality::Medium, 1), (Quality::High, 0)] {
            let mut request = RequestedGraphics::default();
            request.renderer.surface_quality = quality;
            for enabled in [true, false] {
                request.renderer.terrain_normal_maps = enabled;
                request.renderer.terrain_ground_detail = enabled;
                let resolved =
                    ResolvedGraphicsSettings::from_requested(&request, &Capabilities::unknown());
                assert_eq!(resolved.terrain_normal_maps, enabled);
                assert_eq!(resolved.terrain_ground_detail, enabled);
                assert_eq!(
                    resolved.terrain_material_max_level(),
                    if enabled {
                        match quality {
                            Quality::Low => 20,
                            Quality::Medium => 21,
                            Quality::High => 22,
                        }
                    } else {
                        17
                    }
                );
                assert_eq!(
                    resolved.as_meta_map()["terrain_ground_detail"],
                    enabled.to_string()
                );
                assert_eq!(resolved.terrain_normal_mip_bias(), bias);
                assert_eq!(
                    resolved.as_meta_map()["terrain_normal_maps"],
                    enabled.to_string()
                );
                assert_eq!(
                    RequestedGraphics::from_toml(&toml::to_string(&request).unwrap()).unwrap(),
                    request
                );
            }
        }
    }
    #[test]
    fn topology_intent_budgets_fallback_and_metadata_roundtrip() {
        let request = RequestedGraphics::from_toml(
            "version=1\n[terrain_topology]\nenabled=true\nquality='low'\n",
        )
        .unwrap();
        let unknown = ResolvedGraphicsSettings::from_requested(&request, &Capabilities::unknown());
        assert_eq!(unknown.terrain_topology.capacity, 32768);
        assert_eq!(unknown.terrain_topology.waves, 1);
        assert_eq!(unknown.terrain_topology.split_area_px, 1024.0);
        assert_eq!(unknown.terrain_topology.merge_area_px, 128.0);
        let caps = Capabilities {
            terrain_conforming_supported: Some(false),
            ..Default::default()
        };
        let fallback = ResolvedGraphicsSettings::from_requested(&request, &caps);
        assert_eq!(
            fallback.terrain_topology,
            ResolvedTerrainTopology::default()
        );
        assert!(fallback.notes.iter().any(|n| n.contains("conforming")));
        for (quality, capacity, waves, split) in [
            (Quality::Low, 32768, 1, 1024.0),
            (Quality::Medium, 65536, 2, 256.0),
            (Quality::High, 131072, 4, 64.0),
        ] {
            let mut request = request.clone();
            request.terrain_topology.quality = quality;
            let resolved = ResolvedGraphicsSettings::from_requested(
                &request,
                &Capabilities {
                    terrain_conforming_supported: Some(true),
                    ..Default::default()
                },
            );
            assert_eq!(
                resolved.terrain_topology,
                ResolvedTerrainTopology {
                    capacity,
                    waves,
                    split_area_px: split,
                    merge_area_px: split / 8.0
                }
            );
            request.terrain_topology.enabled = false;
            assert_eq!(
                ResolvedGraphicsSettings::from_requested(&request, &Capabilities::unknown())
                    .terrain_topology,
                ResolvedTerrainTopology::default()
            );
            let roundtrip =
                RequestedGraphics::from_toml(&toml::to_string(&request).unwrap()).unwrap();
            assert_eq!(roundtrip, request);
        }
        assert!(
            crate::describe()
                .iter()
                .any(|m| m.path == "terrain_topology.enabled" && m.restart_required)
        );
    }
}

impl ResolvedGraphicsSettings {
    /// Native page refinement ceiling; existing residency budgets are unchanged.
    pub fn terrain_material_max_level(&self) -> u8 {
        if !self.terrain_ground_detail {
            return 17;
        }
        match self.surface_quality {
            Quality::Low => 20,
            Quality::Medium => 21,
            Quality::High => 22,
        }
    }
    /// Extra normal-map mip filtering; coarser tiers reduce texture bandwidth
    /// and resolved detail without changing the canonical surface.
    pub fn terrain_normal_mip_bias(&self) -> u32 {
        match self.surface_quality {
            Quality::Low => 2,
            Quality::Medium => 1,
            Quality::High => 0,
        }
    }

    /// Native terrain GGX environment quadrature. Shares the requested surface
    /// quality/specular switch; zero compiles out this consumer entirely.
    pub fn terrain_environment_samples(&self) -> u32 {
        if !self.specular_reflections {
            return 0;
        }
        match self.surface_quality {
            Quality::Low => 4,
            Quality::Medium => 8,
            Quality::High => 16,
        }
    }

    /// Reflected-cloud (directions, view steps, sun steps) budget. Primary
    /// clouds and existing mesh/ocean reflections retain their own budgets.
    pub fn terrain_environment_cloud_budget(&self) -> [u32; 3] {
        if !self.specular_reflections || !self.clouds_enabled {
            return [0; 3];
        }
        let count = match self.surface_quality {
            Quality::Low => 1,
            Quality::Medium => 2,
            Quality::High => 4,
        };
        [count, count.min(self.clouds_steps.max(1)), 1]
    }

    /// Static inhabited-land optical mask budget; disabled/zero-gain paths
    /// have no producer or texture allocation beyond a shared black binding.
    pub fn night_light_mask_size(&self) -> Option<[u32; 2]> {
        if !self.night_lights_enabled
            || !self.night_lights_intensity.is_finite()
            || self.night_lights_intensity <= 0.0
        {
            return None;
        }
        Some(match self.night_lights_quality {
            Quality::Low => [128, 64],
            Quality::Medium => [256, 128],
            Quality::High => [512, 256],
        })
    }

    /// Resolve requested settings through capabilities. Never panics; explicit
    /// RT requests on incapable hardware are preserved (fail-fast downstream)
    /// with an explanatory note, everything else degrades cleanly.
    pub fn from_requested(requested: &RequestedGraphics, caps: &Capabilities) -> Self {
        let mut notes = Vec::new();
        let hdr = requested.renderer.hdr && caps.hdr_render_target_supported != Some(false);
        if requested.renderer.hdr && !hdr {
            notes.push(
                "HDR scene target unsupported: using direct LDR without postprocessing".into(),
            );
        }

        let backend_name = match requested.renderer.backend {
            BackendRequest::Auto => "auto".to_string(),
            BackendRequest::Vulkan => "vulkan".to_string(),
            BackendRequest::Metal => "metal".to_string(),
            BackendRequest::Webgpu => "webgpu".to_string(),
            BackendRequest::Dx12 => {
                notes.push(
                    "backend dx12 requested but deferred pending ADR; using auto".to_string(),
                );
                "auto".to_string()
            }
        };

        let ray_tracing = match requested.renderer.ray_tracing {
            RayTracingRequest::Off => ResolvedRayTracing::Off,
            RayTracingRequest::Local => {
                Self::note_explicit_rt(caps, &mut notes);
                ResolvedRayTracing::Local
            }
            RayTracingRequest::Full => {
                Self::note_explicit_rt(caps, &mut notes);
                ResolvedRayTracing::Full
            }
            RayTracingRequest::Auto => match caps.ray_query_supported {
                Some(true) => {
                    notes.push("auto: ray-query capable adapter, enabling local RT".to_string());
                    ResolvedRayTracing::Local
                }
                _ => {
                    notes.push(
                        "auto: RT capability unknown/unavailable pre-init, using raster"
                            .to_string(),
                    );
                    ResolvedRayTracing::Off
                }
            },
        };

        // Participation flags only matter when RT is active; clear them
        // otherwise so captures never claim RT work on a raster run.
        let rt_on = ray_tracing.is_active();
        if !rt_on
            && (requested.raytracing.terrain
                || requested.raytracing.vehicles
                || requested.raytracing.atmosphere)
        {
            notes.push("RT participation flags ignored while ray tracing is off".to_string());
        }

        let terrain = match requested.renderer.terrain {
            TerrainRenderRequest::Cpu => ResolvedTerrainRender::Cpu,
            TerrainRenderRequest::GpuIndexed => ResolvedTerrainRender::GpuIndexed,
            TerrainRenderRequest::GpuAuto | TerrainRenderRequest::GpuMesh => {
                if caps.terrain_mesh_supported == Some(true) {
                    ResolvedTerrainRender::GpuMesh
                } else {
                    notes.push(
                        "terrain mesh capability unknown/unavailable; using GPU indexed".into(),
                    );
                    ResolvedTerrainRender::GpuIndexed
                }
            }
        };
        let mut native_shadow_map_size = requested
            .shadows
            .map_size
            .clamp(512, 8192)
            .next_power_of_two();
        let layers = 3 * requested.shadows.cascades.clamp(1, 4);
        let budget = u64::from(requested.renderer.shadow_memory_mib.clamp(4, 2048)) * 1024 * 1024;
        while u64::from(native_shadow_map_size).pow(2) * u64::from(layers) * 4 > budget {
            native_shadow_map_size /= 2;
        }
        if requested.shadows.enabled && native_shadow_map_size < requested.shadows.map_size {
            notes.push(format!(
                "native shadow atlas memory budget resolves map size to {native_shadow_map_size}"
            ));
        }
        let terrain_topology = ResolvedTerrainTopology::resolve(
            requested.terrain_topology,
            caps.terrain_conforming_supported,
            &mut notes,
        );
        let terrain_ground_detail =
            requested.renderer.terrain_ground_detail && terrain_topology.capacity == 0;
        if requested.renderer.terrain_ground_detail && !terrain_ground_detail {
            notes.push("experimental ground detail requires regular-patch local coordinates; conforming terrain retains L17 materials".into());
        }
        Self {
            surface_plants_enabled: requested.surface_plants.enabled,
            surface_plants_density_scale: if !requested.surface_plants.enabled {
                0.0
            } else {
                match requested.surface_plants.quality {
                    Quality::Low => 0.25,
                    Quality::Medium => 0.5,
                    Quality::High => 1.0,
                }
            },
            surface_plants_max_candidates: if requested.surface_plants.enabled {
                4096
            } else {
                0
            },
            surface_plants_max_items: if requested.surface_plants.enabled {
                4096
            } else {
                0
            },
            instruments: resolve_instruments(requested.instruments),
            ocean: requested.ocean.clone(),
            ocean_size: match requested.ocean.quality {
                Quality::Low => 32,
                Quality::Medium => 64,
                Quality::High => 128,
            },
            preset_label: requested.effective_preset_label().to_string(),
            rt_scene_memory_bytes: u64::from(requested.raytracing.scene_memory_mib.clamp(1, 128))
                * 1024
                * 1024,
            rt_max_instances: requested.raytracing.max_instances.clamp(1, 16384),
            rt_reflection_bounces: requested.raytracing.reflection_bounces.clamp(1, 4),
            rt_reflection_samples: match requested.renderer.surface_quality {
                Quality::Low => 1,
                Quality::Medium => 2,
                Quality::High => 4,
            },
            rt_ray_bias_m: if requested.raytracing.ray_bias_m.is_finite() {
                requested.raytracing.ray_bias_m.clamp(0.0001, 1.0)
            } else {
                0.01
            },
            native_shadow_map_size,
            shadow_filter_radius: match requested.shadows.quality {
                Quality::Low => 0,
                Quality::Medium => 1,
                Quality::High => 2,
            },
            surface_quality: requested.renderer.surface_quality,
            terrain_normal_maps: requested.renderer.terrain_normal_maps,
            terrain_ground_detail,
            specular_reflections: requested.renderer.specular_reflections,
            backend: ResolvedBackend { name: backend_name },
            terrain,
            terrain_topology,
            terrain_mesh_cells: requested.renderer.terrain_mesh_cells.clamp(8, 64),
            material_storage: match requested.renderer.material_storage {
                crate::settings::MaterialStorageRequest::RgbaArray => {
                    ResolvedMaterialStorage::RgbaArray
                }
                crate::settings::MaterialStorageRequest::MicrostoreCompact => {
                    if caps.material_compact_supported == Some(false) {
                        notes.push("Compact material residency is unavailable in this host; using rgba_array (no compact encode/decode work)".into());
                        ResolvedMaterialStorage::RgbaArray
                    } else {
                        ResolvedMaterialStorage::MicrostoreCompact
                    }
                }
            },
            ray_tracing,
            resolution_scale: requested.renderer.resolution_scale,
            vsync: requested.renderer.vsync,
            hdr,
            bloom: requested.renderer.bloom,
            exposure_ev100: if requested.renderer.exposure_ev100.is_finite() {
                requested.renderer.exposure_ev100.clamp(-16.0, 32.0)
            } else {
                13.0
            },
            fleet_meshes: requested.renderer.fleet_meshes,
            fleet_min_pixels: match requested.renderer.fleet_quality {
                Quality::Low => 8.0,
                Quality::Medium => 2.0,
                Quality::High => 0.5,
            },
            auto_exposure: requested.renderer.auto_exposure && hdr,
            atmosphere_enabled: requested.atmosphere.enabled,
            atmosphere_quality: requested.atmosphere.quality,
            atmosphere_steps: requested.atmosphere.ray_steps,
            aerial_perspective: requested.atmosphere.aerial_perspective,
            multiple_scattering: requested.atmosphere.multiple_scattering,
            eclipses: requested.atmosphere.eclipses,
            multi_star: requested.atmosphere.multi_star,
            limb_scattering: requested.atmosphere.limb_scattering,
            sky_dome: requested.atmosphere.sky_dome,
            stars: requested.atmosphere.stars,
            stars_quality: requested.atmosphere.stars_quality,
            atmosphere_density_scale: requested.atmosphere.density_scale.clamp(0.0, 2.0),
            airglow: requested.upper_atmosphere.airglow,
            aurora_shell: requested.upper_atmosphere.aurora,
            aurora_lighting: requested.upper_atmosphere.aurora_lighting && rt_on,
            aurora_quality: requested.upper_atmosphere.aurora_quality,
            aurora_intensity: requested.upper_atmosphere.aurora_intensity.clamp(0.0, 4.0),
            aurora_animate: requested.upper_atmosphere.aurora_animate,
            clouds_enabled: requested.clouds.enabled,
            clouds_quality: requested.clouds.quality,
            cloud_integrator: requested.clouds.integrator,
            clouds_volumetric: Self::resolve_cloud_volume(requested, &mut notes),
            clouds_steps: Self::resolve_cloud_steps(requested, &mut notes),
            clouds_layers: requested.clouds.layers.clamp(1, 2),
            clouds_coverage: requested.clouds.coverage.clamp(0.0, 1.0),
            clouds_opacity: requested.clouds.opacity.clamp(0.0, 1.0),
            clouds_animate: requested.clouds.animate,
            clouds_cast_shadows: requested.clouds.cast_shadows && requested.clouds.enabled,
            night_lights_enabled: requested.night_lights.enabled,
            night_lights_quality: requested.night_lights.quality,
            night_lights_intensity: requested.night_lights.intensity.clamp(0.0, 2.0),
            gas_giant_enabled: requested.gas_giant.enabled,
            gas_giant_quality: requested.gas_giant.quality,
            gas_giant_animate: requested.gas_giant.animate_bands,
            gas_giant_limb: requested.gas_giant.limb_darkening,
            plume_enabled: requested.engine_plume.enabled,
            plume_quality: requested.engine_plume.quality,
            plume_diamonds: requested.engine_plume.mach_diamonds,
            plume_flicker: requested.engine_plume.flicker,
            plume_light: requested.engine_plume.light,
            rt_terrain: requested.raytracing.terrain && rt_on,
            rt_vehicles: requested.raytracing.vehicles && rt_on,
            rt_landmarks: requested.raytracing.landmarks && rt_on,
            rt_atmosphere_queries: requested.raytracing.atmosphere && rt_on,
            rt_clouds: requested.raytracing.clouds && rt_on,
            rt_max_distance_m: requested.raytracing.max_distance_m,
            shadow_enabled: requested.shadows.enabled,
            shadow_cascades: requested.shadows.cascades.clamp(1, 4),
            shadow_max_distance_m: requested.shadows.max_distance_m.clamp(100.0, 100_000.0),
            shadow_map_size: requested.shadows.map_size.clamp(512, 8192),
            shadow_normal_bias_m: requested.shadows.normal_bias_m.clamp(0.0, 50.0),
            notes,
        }
    }

    /// Volume path needs the same shader loop bound as
    /// `assets/shaders/cloud_volume.wesl` (`CLOUD_MARCH_MAX`).
    const CLOUD_MARCH_STEP_CAP: u32 = 24;

    fn resolve_cloud_volume(requested: &RequestedGraphics, notes: &mut Vec<String>) -> bool {
        if !requested.clouds.volumetric || !requested.clouds.enabled {
            return false;
        }
        if requested.clouds.quality == Quality::Low {
            notes.push(
                "volumetric clouds requested at low quality; using shell decks (low = shell fallback)"
                    .to_string(),
            );
            return false;
        }
        true
    }

    fn resolve_cloud_steps(requested: &RequestedGraphics, notes: &mut Vec<String>) -> u32 {
        let steps = requested.clouds.ray_steps.clamp(4, 128);
        if steps > Self::CLOUD_MARCH_STEP_CAP {
            notes.push(format!(
                "cloud ray_steps {steps} capped at {} for the volume march",
                Self::CLOUD_MARCH_STEP_CAP
            ));
            Self::CLOUD_MARCH_STEP_CAP
        } else {
            steps
        }
    }

    fn note_explicit_rt(caps: &Capabilities, notes: &mut Vec<String>) {
        match caps.ray_query_supported {
            Some(false) => notes.push(
                "RT explicitly requested but adapter lacks ray query; device creation is expected to fail — set ray_tracing=\"off\" to fall back to raster"
                    .to_string(),
            ),
            None => notes.push(
                "RT explicitly requested; capability unknown pre-init, failing fast if unsupported"
                    .to_string(),
            ),
            Some(true) => {}
        }
    }
}

/// Pure resolution of requested cloud integrator and optional environment override string.
/// Zero allocations, pure enum mapping.
pub fn resolve_cloud_integrator(
    requested: crate::settings::CloudIntegratorRequest,
    env_override: Option<&str>,
) -> crate::settings::CloudIntegratorRequest {
    if let Some(val) = env_override {
        if val.eq_ignore_ascii_case("pt")
            || val.eq_ignore_ascii_case("stochastic")
            || val == "1"
            || val.eq_ignore_ascii_case("pathtrace")
        {
            return crate::settings::CloudIntegratorRequest::Stochastic;
        }
        if val.eq_ignore_ascii_case("march") || val == "0" || val.eq_ignore_ascii_case("raymarch") {
            return crate::settings::CloudIntegratorRequest::March;
        }
    }
    requested
}

/// Single immutable source of truth for the `THESSA_CLOUD_BACKEND` environment override.
///
/// Cached via `std::sync::OnceLock` so repeated calls across frames or across systems
/// never perform redundant allocations or `std::env::var` queries, and maintain strict consistency.
pub fn cloud_backend_override_env() -> Option<&'static str> {
    static BACKEND_OVERRIDE: std::sync::OnceLock<Option<String>> = std::sync::OnceLock::new();
    BACKEND_OVERRIDE
        .get_or_init(|| std::env::var("THESSA_CLOUD_BACKEND").ok())
        .as_deref()
}

impl ResolvedGraphicsSettings {
    /// Evaluates the true runtime representation selected for clouds:
    /// Disabled when clouds are off, Shell when volumetric is off,
    /// and VolumeMarch / VolumeStochastic when volume marching.
    ///
    /// Respects the `THESSA_CLOUD_BACKEND` environment variable override if set
    /// ("stochastic" / "pt" / "1" -> VolumeStochastic, "march" / "0" -> VolumeMarch).
    pub fn active_cloud_representation(&self) -> ActiveCloudRepresentation {
        self.active_cloud_representation_with_override(cloud_backend_override_env())
    }

    /// Evaluates the runtime representation with an optional explicit environment override string.
    pub fn active_cloud_representation_with_override(
        &self,
        override_env: Option<&str>,
    ) -> ActiveCloudRepresentation {
        if !self.clouds_enabled {
            ActiveCloudRepresentation::Disabled
        } else if !self.clouds_volumetric {
            ActiveCloudRepresentation::Shell
        } else {
            match resolve_cloud_integrator(self.cloud_integrator, override_env) {
                crate::settings::CloudIntegratorRequest::March => {
                    ActiveCloudRepresentation::VolumeMarch
                }
                crate::settings::CloudIntegratorRequest::Stochastic => {
                    ActiveCloudRepresentation::VolumeStochastic
                }
            }
        }
    }

    /// Flat string map for perf capture metadata (`resolved` section).
    pub fn as_meta_map(&self) -> BTreeMap<String, String> {
        let mut map = BTreeMap::new();
        map.insert(
            "surface_plants_enabled".into(),
            self.surface_plants_enabled.to_string(),
        );
        map.insert(
            "surface_plants_density_scale".into(),
            self.surface_plants_density_scale.to_string(),
        );
        map.insert(
            "surface_plants_max_candidates".into(),
            self.surface_plants_max_candidates.to_string(),
        );
        map.insert(
            "surface_plants_max_items".into(),
            self.surface_plants_max_items.to_string(),
        );
        map.insert(
            "surface_quality".into(),
            format!("{:?}", self.surface_quality).to_lowercase(),
        );
        map.insert(
            "terrain_normal_maps".into(),
            self.terrain_normal_maps.to_string(),
        );
        map.insert(
            "terrain_ground_detail".into(),
            self.terrain_ground_detail.to_string(),
        );
        map.insert(
            "terrain_material_max_level".into(),
            self.terrain_material_max_level().to_string(),
        );
        map.insert(
            "terrain_normal_mip_bias".into(),
            self.terrain_normal_mip_bias().to_string(),
        );
        map.insert(
            "specular_reflections".into(),
            self.specular_reflections.to_string(),
        );
        map.insert(
            "native_shadow_map_size".into(),
            self.native_shadow_map_size.to_string(),
        );
        map.insert(
            "shadow_filter_radius".into(),
            self.shadow_filter_radius.to_string(),
        );
        map.insert("fleet_meshes".into(), self.fleet_meshes.to_string());
        map.insert("fleet_min_pixels".into(), self.fleet_min_pixels.to_string());
        map.insert("auto_exposure".into(), self.auto_exposure.to_string());
        map.insert("instruments".into(), self.instruments.enabled.to_string());
        map.insert(
            "instrument_layout".into(),
            format!("{:?}", self.instruments.layout).to_lowercase(),
        );
        map.insert(
            "navball_diameter_points".into(),
            self.instruments.diameter_points.to_string(),
        );
        map.insert(
            "navball_atlas_width".into(),
            self.instruments.atlas_width.to_string(),
        );
        map.insert(
            "navball_renderer".into(),
            format!("{:?}", self.instruments.renderer).to_lowercase(),
        );
        map.insert("ocean".into(), self.ocean.enabled.to_string());
        map.insert("ocean_size".into(), self.ocean_size.to_string());
        map.insert(
            "terrain_topology_capacity".into(),
            self.terrain_topology.capacity.to_string(),
        );
        map.insert(
            "terrain_topology_waves".into(),
            self.terrain_topology.waves.to_string(),
        );
        map.insert(
            "terrain_topology_split_area_px".into(),
            self.terrain_topology.split_area_px.to_string(),
        );
        map.insert(
            "terrain_topology_merge_area_px".into(),
            self.terrain_topology.merge_area_px.to_string(),
        );
        map.insert(
            "ocean_geometry_patches".into(),
            self.ocean.geometry_patches.to_string(),
        );
        map.insert(
            "ocean_geometry_distance_m".into(),
            self.ocean.geometry_distance_m.to_string(),
        );
        map.insert("preset".to_string(), self.preset_label.clone());
        map.insert("backend".to_string(), self.backend.name.clone());
        map.insert("terrain".to_string(), self.terrain.as_str().to_string());
        map.insert(
            "material_storage".to_string(),
            self.material_storage.as_str().to_string(),
        );
        map.insert(
            "terrain_mesh_cells".to_string(),
            self.terrain_mesh_cells.to_string(),
        );
        map.insert(
            "ray_tracing".to_string(),
            self.ray_tracing.as_str().to_string(),
        );
        map.insert(
            "resolution_scale".to_string(),
            format!("{:.3}", self.resolution_scale),
        );
        map.insert("vsync".to_string(), self.vsync.to_string());
        map.insert("hdr".to_string(), self.hdr.to_string());
        map.insert("bloom".to_string(), self.bloom.to_string());
        map.insert(
            "exposure_ev100".to_string(),
            format!("{:.1}", self.exposure_ev100),
        );
        map.insert(
            "atmosphere_quality".to_string(),
            format!("{:?}", self.atmosphere_quality).to_lowercase(),
        );
        map.insert(
            "atmosphere_steps".to_string(),
            self.atmosphere_steps.to_string(),
        );
        map.insert("eclipses".to_string(), self.eclipses.to_string());
        map.insert("stars".into(), self.stars.to_string());
        map.insert(
            "stars_quality".into(),
            format!("{:?}", self.stars_quality).to_lowercase(),
        );
        map.insert("multi_star".to_string(), self.multi_star.to_string());
        map.insert(
            "rt_max_distance_m".to_string(),
            format!("{:.0}", self.rt_max_distance_m),
        );
        map.insert(
            "rt_scene_memory_bytes".to_string(),
            self.rt_scene_memory_bytes.to_string(),
        );
        map.insert(
            "rt_max_instances".to_string(),
            self.rt_max_instances.to_string(),
        );
        map.insert(
            "rt_reflection_bounces".to_string(),
            self.rt_reflection_bounces.to_string(),
        );
        map.insert("shadows".to_string(), self.shadow_enabled.to_string());
        map.insert(
            "shadow_cascades".to_string(),
            self.shadow_cascades.to_string(),
        );
        map.insert(
            "shadow_map_size".to_string(),
            self.shadow_map_size.to_string(),
        );
        map.insert("clouds".to_string(), self.clouds_enabled.to_string());
        map.insert(
            "active_cloud_representation".to_string(),
            self.active_cloud_representation().as_str().to_string(),
        );
        map.insert(
            "clouds_volumetric".to_string(),
            self.clouds_volumetric.to_string(),
        );
        map.insert(
            "cloud_integrator".to_string(),
            format!("{:?}", self.cloud_integrator).to_lowercase(),
        );
        map.insert("cloud_steps".to_string(), self.clouds_steps.to_string());
        map.insert("cloud_layers".to_string(), self.clouds_layers.to_string());
        map.insert(
            "cloud_shadows".to_string(),
            self.clouds_cast_shadows.to_string(),
        );
        map.insert(
            "night_lights".to_string(),
            self.night_lights_enabled.to_string(),
        );
        map.insert("gas_giant".to_string(), self.gas_giant_enabled.to_string());
        map.insert("plume".to_string(), self.plume_enabled.to_string());
        map.insert("aurora".to_string(), self.aurora_shell.to_string());
        map.insert(
            "aurora_intensity".to_string(),
            format!("{:.2}", self.aurora_intensity),
        );
        for (i, note) in self.notes.iter().enumerate() {
            map.insert(format!("note_{i}"), note.clone());
        }
        map
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::settings::RequestedGraphics;

    #[test]
    fn surface_plants_tiers_disable_metadata_and_isolation() {
        for supported in [None, Some(false), Some(true)] {
            let caps = Capabilities {
                material_compact_supported: supported,
                terrain_conforming_supported: supported,
                terrain_mesh_supported: supported,
                ray_query_supported: supported,
                is_discrete_gpu: supported,
                hdr_render_target_supported: supported,
            };
            // Exclude preset-label changes so complete resolved equality pins
            // isolation, including RT, shadow, topology and fallback notes.
            let mut request = RequestedGraphics {
                preset: crate::Preset::Custom,
                ..Default::default()
            };
            let baseline = ResolvedGraphicsSettings::from_requested(&request, &caps);
            assert!(!baseline.surface_plants_enabled);
            assert_eq!(baseline.surface_plants_density_scale, 0.0);
            assert_eq!(baseline.surface_plants_max_candidates, 0);
            assert_eq!(baseline.surface_plants_max_items, 0);
            for (quality, density) in [
                (Quality::Low, 0.25),
                (Quality::Medium, 0.5),
                (Quality::High, 1.0),
            ] {
                request.surface_plants.quality = quality;
                for enabled in [false, true] {
                    request.surface_plants.enabled = enabled;
                    let resolved = ResolvedGraphicsSettings::from_requested(&request, &caps);
                    let expected_density = if enabled { density } else { 0.0 };
                    let expected_budget = if enabled { 4096 } else { 0 };
                    assert_eq!(resolved.surface_plants_enabled, enabled);
                    assert_eq!(resolved.surface_plants_density_scale, expected_density);
                    assert_eq!(resolved.surface_plants_max_candidates, expected_budget);
                    assert_eq!(resolved.surface_plants_max_items, expected_budget as u32);
                    let meta = resolved.as_meta_map();
                    assert_eq!(meta["surface_plants_enabled"], enabled.to_string());
                    assert_eq!(
                        meta["surface_plants_density_scale"],
                        expected_density.to_string()
                    );
                    assert_eq!(
                        meta["surface_plants_max_candidates"],
                        expected_budget.to_string()
                    );
                    assert_eq!(
                        meta["surface_plants_max_items"],
                        expected_budget.to_string()
                    );
                    assert_eq!(
                        toml::from_str::<ResolvedGraphicsSettings>(
                            &toml::to_string(&resolved).unwrap()
                        )
                        .unwrap(),
                        resolved
                    );
                    let mut unrelated = resolved;
                    unrelated.surface_plants_enabled = false;
                    unrelated.surface_plants_density_scale = 0.0;
                    unrelated.surface_plants_max_candidates = 0;
                    unrelated.surface_plants_max_items = 0;
                    assert_eq!(unrelated, baseline);
                }
            }
        }
    }

    #[test]
    fn surface_plants_legacy_resolved_capture_defaults_to_zero_work() {
        let baseline = ResolvedGraphicsSettings::default();
        let mut legacy = toml::Value::try_from(&baseline).unwrap();
        for key in [
            "surface_plants_enabled",
            "surface_plants_density_scale",
            "surface_plants_max_candidates",
            "surface_plants_max_items",
        ] {
            legacy.as_table_mut().unwrap().remove(key).unwrap();
        }
        let restored: ResolvedGraphicsSettings = legacy.try_into().unwrap();
        assert_eq!(restored, baseline);
    }

    #[test]
    fn terrain_environment_quadrature_follows_resolved_material_policy() {
        for (quality, samples) in [(Quality::Low, 4), (Quality::Medium, 8), (Quality::High, 16)] {
            let mut request = RequestedGraphics::default();
            request.renderer.surface_quality = quality;
            request.renderer.specular_reflections = true;
            request.clouds.enabled = true;
            request.clouds.volumetric = true;
            request.clouds.quality = Quality::High;
            request.clouds.ray_steps = 24;
            let resolved =
                ResolvedGraphicsSettings::from_requested(&request, &Capabilities::unknown());
            assert_eq!(resolved.terrain_environment_samples(), samples);
            assert_eq!(
                resolved.terrain_environment_cloud_budget(),
                [samples / 4, samples / 4, 1]
            );
            request.renderer.specular_reflections = false;
            let off = ResolvedGraphicsSettings::from_requested(&request, &Capabilities::unknown());
            assert_eq!(off.terrain_environment_samples(), 0);
            assert_eq!(off.terrain_environment_cloud_budget(), [0; 3]);
            assert_eq!(off.surface_quality, resolved.surface_quality);
            request.renderer.specular_reflections = true;
            request.clouds.enabled = false;
            let clear =
                ResolvedGraphicsSettings::from_requested(&request, &Capabilities::unknown());
            assert_eq!(clear.terrain_environment_cloud_budget(), [0; 3]);
            assert_eq!(clear.terrain_environment_samples(), samples);
        }
    }

    #[test]
    fn fleet_quality_is_a_resolved_budget_and_disabled_stays_disabled() {
        for (quality, pixels) in [
            (Quality::Low, 8.0),
            (Quality::Medium, 2.0),
            (Quality::High, 0.5),
        ] {
            let mut requested = RequestedGraphics::default();
            requested.renderer.fleet_quality = quality;
            let resolved =
                ResolvedGraphicsSettings::from_requested(&requested, &Capabilities::unknown());
            assert_eq!(resolved.fleet_min_pixels, pixels);
            assert!(resolved.fleet_meshes);
            requested.renderer.fleet_meshes = false;
            let resolved =
                ResolvedGraphicsSettings::from_requested(&requested, &Capabilities::unknown());
            assert!(!resolved.fleet_meshes);
            assert_eq!(resolved.as_meta_map()["fleet_meshes"], "false");
        }
        let requested = RequestedGraphics::from_toml(
            "[renderer]\nfleet_meshes = false\nfleet_quality = 'low'\n",
        )
        .unwrap();
        assert!(!requested.renderer.fleet_meshes);
        assert_eq!(requested.renderer.fleet_quality, Quality::Low);
    }

    #[test]
    fn auto_resolves_to_raster_when_capability_unknown() {
        let resolved = ResolvedGraphicsSettings::from_requested(
            &RequestedGraphics::default(),
            &Capabilities::unknown(),
        );
        assert_eq!(resolved.ray_tracing, ResolvedRayTracing::Off);
        assert!(!resolved.rt_terrain);
        assert!(resolved.notes.iter().any(|n| n.contains("raster")));
    }

    #[test]
    fn explicit_local_survives_with_fail_fast_note() {
        let mut requested = RequestedGraphics::default();
        requested.renderer.ray_tracing = RayTracingRequest::Local;
        let resolved =
            ResolvedGraphicsSettings::from_requested(&requested, &Capabilities::unknown());
        assert_eq!(resolved.ray_tracing, ResolvedRayTracing::Local);
        assert!(resolved.rt_terrain);
        assert!(resolved.notes.iter().any(|n| n.contains("fail")));
    }

    #[test]
    fn known_capable_adapter_enables_local_on_auto() {
        let caps = Capabilities {
            ray_query_supported: Some(true),
            is_discrete_gpu: Some(true),
            ..Default::default()
        };
        let resolved =
            ResolvedGraphicsSettings::from_requested(&RequestedGraphics::default(), &caps);
        assert_eq!(resolved.ray_tracing, ResolvedRayTracing::Local);
    }

    #[test]
    fn hdr_capability_fallback_and_exposure_validation() {
        let mut requested = RequestedGraphics::default();
        requested.renderer.auto_exposure = true;
        for enabled in [false, true] {
            for supported in [None, Some(false), Some(true)] {
                requested.renderer.hdr = enabled;
                let caps = Capabilities {
                    hdr_render_target_supported: supported,
                    ..Default::default()
                };
                let resolved = ResolvedGraphicsSettings::from_requested(&requested, &caps);
                let expected = enabled && supported != Some(false);
                assert_eq!(resolved.hdr, expected);
                assert_eq!(resolved.auto_exposure, expected);
                assert_eq!(
                    resolved
                        .notes
                        .iter()
                        .any(|n| n.contains("HDR scene target unsupported")),
                    enabled && !expected
                );
            }
        }
        for (input, expected) in [
            (f32::NAN, 13.0),
            (f32::INFINITY, 13.0),
            (-100.0, -16.0),
            (50.0, 32.0),
            (13.5, 13.5),
        ] {
            requested.renderer.exposure_ev100 = input;
            let resolved =
                ResolvedGraphicsSettings::from_requested(&requested, &Capabilities::unknown());
            assert_eq!(resolved.exposure_ev100, expected);
        }
    }

    #[test]
    fn density_scale_defaults_to_one_and_clamps() {
        let resolved = ResolvedGraphicsSettings::from_requested(
            &RequestedGraphics::default(),
            &Capabilities::unknown(),
        );
        assert_eq!(resolved.atmosphere_density_scale, 1.0);
        let mut requested = RequestedGraphics::default();
        requested.atmosphere.density_scale = 9.0;
        let resolved =
            ResolvedGraphicsSettings::from_requested(&requested, &Capabilities::unknown());
        assert_eq!(resolved.atmosphere_density_scale, 2.0);
        requested.atmosphere.density_scale = -1.0;
        let resolved =
            ResolvedGraphicsSettings::from_requested(&requested, &Capabilities::unknown());
        assert_eq!(resolved.atmosphere_density_scale, 0.0);
    }

    #[test]
    fn dx12_request_falls_back_to_auto_with_note() {
        let mut requested = RequestedGraphics::default();
        requested.renderer.backend = BackendRequest::Dx12;
        let resolved =
            ResolvedGraphicsSettings::from_requested(&requested, &Capabilities::unknown());
        assert_eq!(resolved.backend.name, "auto");
        assert!(resolved.notes.iter().any(|n| n.contains("dx12")));
    }

    #[test]
    fn shadow_budgets_resolve_with_defensive_clamps() {
        let requested = RequestedGraphics::default();
        let resolved =
            ResolvedGraphicsSettings::from_requested(&requested, &Capabilities::unknown());
        assert!(resolved.shadow_enabled);
        assert_eq!(resolved.shadow_cascades, 4);
        assert_eq!(resolved.shadow_map_size, 4096);
        assert!((resolved.shadow_max_distance_m - 12_000.0).abs() < 1e-9);
        let meta = resolved.as_meta_map();
        assert_eq!(meta["shadows"], "true");
        assert_eq!(meta["shadow_cascades"], "4");
    }

    #[test]
    fn native_surface_tiers_and_atlas_memory_budget_are_explicit_and_round_trip() {
        let mut requested = RequestedGraphics::default();
        for (preset, quality, filter) in [
            (crate::Preset::Low, Quality::Low, 0),
            (crate::Preset::Medium, Quality::Medium, 1),
            (crate::Preset::High, Quality::High, 2),
        ] {
            requested.apply_preset(preset);
            let resolved =
                ResolvedGraphicsSettings::from_requested(&requested, &Capabilities::unknown());
            assert_eq!(resolved.surface_quality, quality);
            assert_eq!(resolved.shadow_filter_radius, filter);
        }
        requested.renderer.specular_reflections = false;
        requested.renderer.shadow_memory_mib = 4;
        requested.shadows.map_size = 8192;
        let resolved =
            ResolvedGraphicsSettings::from_requested(&requested, &Capabilities::unknown());
        assert!(!resolved.specular_reflections);
        assert!(
            u64::from(resolved.native_shadow_map_size).pow(2)
                * 3
                * u64::from(resolved.shadow_cascades)
                * 4
                <= 4 * 1024 * 1024
        );
        assert!(resolved.notes.iter().any(|n| n.contains("memory budget")));
        assert_eq!(
            RequestedGraphics::from_toml(&toml::to_string(&requested).unwrap()).unwrap(),
            requested
        );
        assert_eq!(resolved.as_meta_map()["specular_reflections"], "false");
    }

    #[test]
    fn rt_scene_budget_instances_bounces_and_bias_are_clamped_and_round_trip() {
        let mut requested = RequestedGraphics::default();
        requested.renderer.ray_tracing = RayTracingRequest::Local;
        requested.raytracing.scene_memory_mib = 0;
        requested.raytracing.max_instances = 0;
        requested.raytracing.reflection_bounces = 99;
        requested.raytracing.ray_bias_m = -1.0;
        let resolved =
            ResolvedGraphicsSettings::from_requested(&requested, &Capabilities::unknown());
        assert_eq!(resolved.rt_scene_memory_bytes, 1024 * 1024);
        assert_eq!(resolved.rt_max_instances, 1);
        assert_eq!(resolved.rt_reflection_bounces, 4);
        assert_eq!(resolved.rt_ray_bias_m, 0.0001);
        assert!(resolved.rt_vehicles);
        assert_eq!(
            resolved.as_meta_map()["rt_scene_memory_bytes"],
            (1024 * 1024).to_string()
        );
        // Explicit RT on an incapable adapter stays visible and is not
        // silently downgraded; the device path fails fast instead.
        let mut requested = RequestedGraphics::default();
        requested.renderer.ray_tracing = RayTracingRequest::Full;
        let resolved = ResolvedGraphicsSettings::from_requested(
            &requested,
            &Capabilities {
                ray_query_supported: Some(false),
                ..Capabilities::unknown()
            },
        );
        assert_eq!(resolved.ray_tracing, ResolvedRayTracing::Full);
        // An explicit request is not silently downgraded to raster: the
        // participation flags stay set and the device path fails fast.
        assert!(resolved.rt_vehicles);
        assert!(resolved.notes.iter().any(|n| n.contains("lacks ray query")));
    }

    #[test]
    fn terrain_mode_is_resolved_without_mesh_shader_features() {
        let mut requested = RequestedGraphics::default();
        requested.renderer.terrain = TerrainRenderRequest::GpuIndexed;
        let resolved =
            ResolvedGraphicsSettings::from_requested(&requested, &Capabilities::unknown());
        assert_eq!(resolved.terrain, ResolvedTerrainRender::GpuIndexed);
        assert_eq!(resolved.terrain_mesh_cells, 24);
        assert!(resolved.terrain.is_gpu());
        assert_eq!(resolved.as_meta_map()["terrain"], "gpu_indexed");
        assert_eq!(resolved.as_meta_map()["terrain_mesh_cells"], "24");
    }

    #[test]
    fn cloud_volume_resolves_with_low_fallback_and_step_cap() {
        use crate::settings::Quality;
        let caps = Capabilities::unknown();
        // Off by default (requested default is shell decks).
        let resolved =
            ResolvedGraphicsSettings::from_requested(&RequestedGraphics::default(), &caps);
        assert!(!resolved.clouds_volumetric);
        // Requested volume on Medium survives with user steps.
        let mut requested = RequestedGraphics::default();
        requested.clouds.enabled = true;
        requested.clouds.volumetric = true;
        requested.clouds.quality = Quality::Medium;
        requested.clouds.ray_steps = 12;
        let resolved = ResolvedGraphicsSettings::from_requested(&requested, &caps);
        assert!(resolved.clouds_volumetric);
        assert_eq!(resolved.clouds_steps, 12);
        // Low quality falls back to shells with a note.
        requested.clouds.quality = Quality::Low;
        let resolved = ResolvedGraphicsSettings::from_requested(&requested, &caps);
        assert!(!resolved.clouds_volumetric);
        assert!(resolved.notes.iter().any(|n| n.contains("shell fallback")));
        // Disabled clouds never march, even when requested.
        requested.clouds.enabled = false;
        requested.clouds.quality = Quality::High;
        let resolved = ResolvedGraphicsSettings::from_requested(&requested, &caps);
        assert!(!resolved.clouds_volumetric);
        // Huge step counts cap at the shader loop bound with a note.
        requested.clouds.enabled = true;
        requested.clouds.ray_steps = 128;
        let resolved = ResolvedGraphicsSettings::from_requested(&requested, &caps);
        assert_eq!(resolved.clouds_steps, 24);
        assert!(resolved.notes.iter().any(|n| n.contains("capped at 24")));
        let meta = resolved.as_meta_map();
        assert_eq!(meta["clouds_volumetric"], "true");
        assert_eq!(meta["cloud_steps"], "24");
    }

    #[test]
    fn mesh_terrain_resolves_by_capability_with_indexed_fallback() {
        for mode in [TerrainRenderRequest::GpuAuto, TerrainRenderRequest::GpuMesh] {
            let mut requested = RequestedGraphics::default();
            requested.renderer.terrain = mode;
            for supported in [None, Some(false), Some(true)] {
                let caps = Capabilities {
                    terrain_mesh_supported: supported,
                    ..Default::default()
                };
                let resolved = ResolvedGraphicsSettings::from_requested(&requested, &caps);
                assert_eq!(
                    resolved.terrain,
                    if supported == Some(true) {
                        ResolvedTerrainRender::GpuMesh
                    } else {
                        ResolvedTerrainRender::GpuIndexed
                    }
                );
                assert_eq!(
                    resolved
                        .notes
                        .iter()
                        .any(|note| note.starts_with("terrain mesh capability")),
                    supported != Some(true)
                );
            }
        }
    }
}

#[cfg(test)]
mod storage_tests {
    use super::*;
    use crate::settings::{MaterialStorageRequest, RequestedGraphics};

    #[test]
    fn unsupported_host_resolves_compact_to_the_explicit_raw_fallback() {
        let mut requested = RequestedGraphics::default();
        requested.renderer.material_storage = MaterialStorageRequest::MicrostoreCompact;
        for supported in [Some(false), Some(true), None] {
            let caps = Capabilities {
                material_compact_supported: supported,
                ..Default::default()
            };
            let resolved = ResolvedGraphicsSettings::from_requested(&requested, &caps);
            assert_eq!(
                resolved.material_storage.is_microstore(),
                supported != Some(false)
            );
            assert_eq!(
                resolved
                    .notes
                    .iter()
                    .any(|n| n.contains("Compact material residency")),
                supported == Some(false)
            );
        }
        requested.renderer.material_storage = MaterialStorageRequest::RgbaArray;
        let resolved = ResolvedGraphicsSettings::from_requested(
            &requested,
            &Capabilities {
                material_compact_supported: Some(false),
                ..Default::default()
            },
        );
        assert!(
            !resolved
                .notes
                .iter()
                .any(|n| n.contains("Compact material residency"))
        );
    }

    #[test]
    fn storage_defaults_to_rgba_array() {
        let resolved = ResolvedGraphicsSettings::from_requested(
            &RequestedGraphics::default(),
            &Capabilities::unknown(),
        );
        assert_eq!(
            resolved.material_storage,
            ResolvedMaterialStorage::RgbaArray
        );
        assert!(!resolved.material_storage.is_microstore());
    }

    #[test]
    fn explicit_compact_survives_resolution_with_telemetry() {
        let mut requested = RequestedGraphics::default();
        requested.renderer.material_storage = MaterialStorageRequest::MicrostoreCompact;
        let resolved =
            ResolvedGraphicsSettings::from_requested(&requested, &Capabilities::unknown());
        assert_eq!(
            resolved.material_storage,
            ResolvedMaterialStorage::MicrostoreCompact
        );
        assert!(resolved.material_storage.is_microstore());
        assert_eq!(
            resolved
                .as_meta_map()
                .get("material_storage")
                .map(String::as_str),
            Some("microstore_compact")
        );
    }

    #[test]
    #[allow(clippy::field_reassign_with_default)]
    fn active_cloud_representation_truth_matrix() {
        let mut r = ResolvedGraphicsSettings::default();

        // 1. Clouds disabled -> Disabled representation regardless of volume/integrator:
        r.clouds_enabled = false;
        r.clouds_volumetric = true;
        r.cloud_integrator = crate::settings::CloudIntegratorRequest::March;
        assert_eq!(
            r.active_cloud_representation(),
            ActiveCloudRepresentation::Disabled
        );
        assert_eq!(r.active_cloud_representation().as_str(), "disabled");
        assert_eq!(
            r.as_meta_map()
                .get("active_cloud_representation")
                .map(String::as_str),
            Some("disabled")
        );

        // 2. Clouds enabled, volumetric false -> Shell representation:
        r.clouds_enabled = true;
        r.clouds_volumetric = false;
        assert_eq!(
            r.active_cloud_representation(),
            ActiveCloudRepresentation::Shell
        );
        assert_eq!(r.active_cloud_representation().as_str(), "shell");
        assert_eq!(
            r.as_meta_map()
                .get("active_cloud_representation")
                .map(String::as_str),
            Some("shell")
        );

        // 3. Clouds enabled, volumetric true, March -> VolumeMarch:
        r.clouds_volumetric = true;
        r.cloud_integrator = crate::settings::CloudIntegratorRequest::March;
        assert_eq!(
            r.active_cloud_representation(),
            ActiveCloudRepresentation::VolumeMarch
        );
        assert_eq!(r.active_cloud_representation().as_str(), "volume_march");

        // 4. Clouds enabled, volumetric true, Stochastic -> VolumeStochastic:
        r.cloud_integrator = crate::settings::CloudIntegratorRequest::Stochastic;
        assert_eq!(
            r.active_cloud_representation_with_override(None),
            ActiveCloudRepresentation::VolumeStochastic
        );
        assert_eq!(
            r.active_cloud_representation_with_override(None).as_str(),
            "volume_stochastic"
        );

        // 5. Explicit overrides via THESSA_CLOUD_BACKEND env values:
        // "pt" forces VolumeStochastic even when settings specify March:
        r.cloud_integrator = crate::settings::CloudIntegratorRequest::March;
        assert_eq!(
            r.active_cloud_representation_with_override(Some("pt")),
            ActiveCloudRepresentation::VolumeStochastic
        );
        // "march" forces VolumeMarch even when settings specify Stochastic:
        r.cloud_integrator = crate::settings::CloudIntegratorRequest::Stochastic;
        assert_eq!(
            r.active_cloud_representation_with_override(Some("march")),
            ActiveCloudRepresentation::VolumeMarch
        );
    }
}
