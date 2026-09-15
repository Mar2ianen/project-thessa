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
}

impl ResolvedTerrainRender {
    pub fn is_gpu(self) -> bool {
        matches!(self, Self::GpuIndexed)
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Cpu => "cpu",
            Self::GpuIndexed => "gpu_indexed",
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
        }
    }
}

/// Fully resolved settings consumed by the renderer and perf captures.
/// Quality controls budgets (steps, LUT sizes), never physics.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ResolvedGraphicsSettings {
    pub preset_label: String,
    pub backend: ResolvedBackend,
    pub terrain: ResolvedTerrainRender,
    pub terrain_mesh_cells: u32,
    pub ray_tracing: ResolvedRayTracing,
    pub resolution_scale: f32,
    pub vsync: bool,
    pub hdr: bool,
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
    pub clouds_layers: u32,
    pub clouds_coverage: f32,
    pub clouds_opacity: f32,
    pub clouds_animate: bool,
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

        Self {
            preset_label: requested.effective_preset_label().to_string(),
            backend: ResolvedBackend { name: backend_name },
            terrain: match requested.renderer.terrain {
                TerrainRenderRequest::Cpu => ResolvedTerrainRender::Cpu,
                TerrainRenderRequest::GpuIndexed => ResolvedTerrainRender::GpuIndexed,
            },
            terrain_mesh_cells: requested.renderer.terrain_mesh_cells.clamp(8, 64),
            ray_tracing,
            resolution_scale: requested.renderer.resolution_scale,
            vsync: requested.renderer.vsync,
            hdr: requested.renderer.hdr,
            exposure_ev100: requested.renderer.exposure_ev100,
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
            clouds_layers: requested.clouds.layers.clamp(1, 2),
            clouds_coverage: requested.clouds.coverage.clamp(0.0, 1.0),
            clouds_opacity: requested.clouds.opacity.clamp(0.0, 1.0),
            clouds_animate: requested.clouds.animate,
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

    /// Flat string map for perf capture metadata (`resolved` section).
    pub fn as_meta_map(&self) -> BTreeMap<String, String> {
        let mut map = BTreeMap::new();
        map.insert("auto_exposure".into(), self.auto_exposure.to_string());
        map.insert("preset".to_string(), self.preset_label.clone());
        map.insert("backend".to_string(), self.backend.name.clone());
        map.insert("terrain".to_string(), self.terrain.as_str().to_string());
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
            "cloud_layers".to_string(),
            self.clouds_layers.to_string(),
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
}
