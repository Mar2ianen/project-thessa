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
    /// Enabled mesh feature and sufficient terrain shader limits on the device.
    pub terrain_mesh_supported: Option<bool>,
    /// `Some(true)` when the adapter exposes ray-query features.
    pub ray_query_supported: Option<bool>,
    /// `Some(true)` when the adapter is a discrete GPU.
    pub is_discrete_gpu: Option<bool>,
}

impl Capabilities {
    pub fn unknown() -> Self {
        Self {
            ray_query_supported: None,
            is_discrete_gpu: None,
            terrain_mesh_supported: None,
        }
    }
}

/// Fully resolved settings consumed by the renderer and perf captures.
/// Quality controls budgets (steps, LUT sizes), never physics.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ResolvedGraphicsSettings {
    pub ocean: crate::settings::OceanSettings,
    pub ocean_size: u32,
    pub rt_scene_memory_bytes: u64,
    pub rt_max_instances: u32,
    pub rt_reflection_bounces: u32,
    pub rt_reflection_samples: u32,
    pub rt_ray_bias_m: f32,
    pub surface_quality: Quality,
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

impl ResolvedGraphicsSettings {
    /// Resolve requested settings through capabilities. Never panics; explicit
    /// RT requests on incapable hardware are preserved (fail-fast downstream)
    /// with an explanatory note, everything else degrades cleanly.
    pub fn from_requested(requested: &RequestedGraphics, caps: &Capabilities) -> Self {
        let mut notes = Vec::new();

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
        Self {
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
            specular_reflections: requested.renderer.specular_reflections,
            backend: ResolvedBackend { name: backend_name },
            terrain,
            terrain_mesh_cells: requested.renderer.terrain_mesh_cells.clamp(8, 64),
            material_storage: match requested.renderer.material_storage {
                crate::settings::MaterialStorageRequest::RgbaArray => {
                    ResolvedMaterialStorage::RgbaArray
                }
                crate::settings::MaterialStorageRequest::MicrostoreCompact => {
                    ResolvedMaterialStorage::MicrostoreCompact
                }
            },
            ray_tracing,
            resolution_scale: requested.renderer.resolution_scale,
            vsync: requested.renderer.vsync,
            hdr: requested.renderer.hdr,
            bloom: requested.renderer.bloom,
            exposure_ev100: requested.renderer.exposure_ev100,
            fleet_meshes: requested.renderer.fleet_meshes,
            fleet_min_pixels: match requested.renderer.fleet_quality {
                Quality::Low => 8.0,
                Quality::Medium => 2.0,
                Quality::High => 0.5,
            },
            auto_exposure: requested.renderer.auto_exposure && requested.renderer.hdr,
            atmosphere_enabled: requested.atmosphere.enabled,
            atmosphere_quality: requested.atmosphere.quality,
            atmosphere_steps: requested.atmosphere.ray_steps,
            aerial_perspective: requested.atmosphere.aerial_perspective,
            multiple_scattering: requested.atmosphere.multiple_scattering,
            eclipses: requested.atmosphere.eclipses,
            multi_star: requested.atmosphere.multi_star,
            limb_scattering: requested.atmosphere.limb_scattering,
            sky_dome: requested.atmosphere.sky_dome,
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
            "surface_quality".into(),
            format!("{:?}", self.surface_quality).to_lowercase(),
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
        map.insert("ocean".into(), self.ocean.enabled.to_string());
        map.insert("ocean_size".into(), self.ocean_size.to_string());
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
