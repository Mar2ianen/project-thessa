//! Requested graphics settings: TOML schema, presets, validation.
//!
//! `graphics.toml` is user intent. Presets are bulk writes over the same
//! fields, never hidden alternative state: when a value diverges from its
//! preset expansion the configuration reports `custom` (spec section 15).

use serde::{Deserialize, Serialize};

/// Bulk starting points. Every preset expands to explicit values; editing any
/// derived value flips the reported preset to [`Preset::Custom`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Preset {
    Low,
    Medium,
    #[default]
    High,
    Ultra,
    Custom,
}

impl Preset {
    pub fn from_str_name(name: &str) -> Option<Self> {
        match name {
            "low" => Some(Self::Low),
            "medium" => Some(Self::Medium),
            "high" => Some(Self::High),
            "ultra" => Some(Self::Ultra),
            "custom" => Some(Self::Custom),
            _ => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Low => "low",
            Self::Medium => "medium",
            Self::High => "high",
            Self::Ultra => "ultra",
            Self::Custom => "custom",
        }
    }
}

/// Quality selector for budget-only scaling (never physics).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Quality {
    Low,
    #[default]
    Medium,
    High,
}

/// Requested render backend. `dx12` is accepted by the parser but resolves to
/// `auto` with a note until a dedicated ADR allows direct DX paths.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum BackendRequest {
    #[default]
    Auto,
    Vulkan,
    Metal,
    Dx12,
    Webgpu,
}

/// Terrain raster consumer used by the normal client.
///
/// The CPU path remains the portable default. The indexed GPU path consumes
/// the same CBT pages without requiring experimental mesh-shader features;
/// hardware mesh shaders stay an isolated crate-level experiment rather than
/// a normal game setting.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TerrainRenderRequest {
    #[default]
    Cpu,
    GpuIndexed,
}

/// Material page storage for the CBT terrain path.
///
/// The CPU path remains the portable default. The compact path holds
/// microstore-encoded pages in residency and decodes them at upload time;
/// sampling (texture format, mips, filtering) is identical either way, so
/// this is a storage/upload tradeoff, never a visual mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MaterialStorageRequest {
    #[default]
    RgbaArray,
    MicrostoreCompact,
}

impl MaterialStorageRequest {
    /// TOML name used in graphics.toml.
    pub fn from_str_name(name: &str) -> Option<Self> {
        match name {
            "rgba_array" => Some(Self::RgbaArray),
            "microstore_compact" => Some(Self::MicrostoreCompact),
            _ => None,
        }
    }
}
/// Requested ray-tracing mode (spec section 18). Avoid a single boolean:
/// `local` buys RT where it has the highest local visual value, `full`
/// enables everything within budget, `auto` resolves by capability.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum RayTracingRequest {
    Off,
    Local,
    Full,
    #[default]
    Auto,
}

impl RayTracingRequest {
    pub fn from_str_name(name: &str) -> Option<Self> {
        match name {
            "off" => Some(Self::Off),
            "local" => Some(Self::Local),
            "full" => Some(Self::Full),
            "auto" => Some(Self::Auto),
            _ => None,
        }
    }
}

fn default_true() -> bool {
    true
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RendererSettings {
    #[serde(default)]
    pub backend: BackendRequest,
    #[serde(default)]
    pub ray_tracing: RayTracingRequest,
    #[serde(default)]
    pub terrain: TerrainRenderRequest,
    /// Material page storage: raw RGBA array (default) or compact
    /// microstore residency with decode at upload time.
    #[serde(default)]
    pub material_storage: MaterialStorageRequest,
    /// CPU terrain vertices per tile edge. The indexed GPU path has a fixed
    /// 33x33 page contract and therefore uses 32 internally.
    #[serde(default = "default_terrain_mesh_cells")]
    pub terrain_mesh_cells: u32,
    #[serde(default = "default_resolution_scale")]
    pub resolution_scale: f32,
    #[serde(default = "default_true")]
    pub vsync: bool,
    #[serde(default = "default_true")]
    pub hdr: bool,
    /// Bloom is an optional post-process; terrain readability wins by default.
    #[serde(default)]
    pub bloom: bool,
    /// Adapt the camera to day/night/eclipses using scene luminance.
    #[serde(default = "default_true")]
    pub auto_exposure: bool,
    /// Manual EV100 used when auto exposure is disabled.
    #[serde(default = "default_exposure")]
    pub exposure_ev100: f32,
}

fn default_exposure() -> f32 {
    13.0
}

fn default_resolution_scale() -> f32 {
    1.0
}

fn default_terrain_mesh_cells() -> u32 {
    24
}

impl Default for RendererSettings {
    fn default() -> Self {
        Self {
            backend: BackendRequest::Auto,
            ray_tracing: RayTracingRequest::Auto,
            terrain: TerrainRenderRequest::Cpu,
            material_storage: MaterialStorageRequest::RgbaArray,
            terrain_mesh_cells: default_terrain_mesh_cells(),
            resolution_scale: 1.0,
            vsync: true,
            hdr: true,
            bloom: false,
            exposure_ev100: 13.0,
            // Deterministic manual exposure until the AE metering curve is
            // tuned against real HDR scenes (a constant -2.47 curve only
            // darkens everything).
            auto_exposure: false,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AtmosphereSettings {
    #[serde(default = "default_true")]
    pub enabled: bool,
    #[serde(default)]
    pub quality: Quality,
    #[serde(default = "default_true")]
    pub aerial_perspective: bool,
    #[serde(default)]
    pub multiple_scattering: bool,
    #[serde(default = "default_true")]
    pub eclipses: bool,
    #[serde(default = "default_true")]
    pub multi_star: bool,
    #[serde(default = "default_true")]
    pub limb_scattering: bool,
    #[serde(default = "default_true")]
    pub sky_dome: bool,
    #[serde(default = "default_ray_steps")]
    pub ray_steps: u32,
    /// Visual optical-depth multiplier for the scattering medium only
    /// (renderer appearance, never physics). 1.0 = physical betas from
    /// pressure; 0.0 = transparent shell/sky. Range enforced at resolve.
    #[serde(default = "default_density_scale")]
    pub density_scale: f32,
}

fn default_density_scale() -> f32 {
    1.0
}

fn default_ray_steps() -> u32 {
    24
}

impl Default for AtmosphereSettings {
    fn default() -> Self {
        Self {
            enabled: true,
            quality: Quality::High,
            aerial_perspective: true,
            multiple_scattering: false,
            eclipses: true,
            multi_star: true,
            limb_scattering: true,
            sky_dome: true,
            ray_steps: 24,
            density_scale: 1.0,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CloudSettings {
    #[serde(default)]
    pub enabled: bool,
    #[serde(default)]
    pub quality: Quality,
    #[serde(default)]
    pub volumetric: bool,
    #[serde(default = "default_cloud_steps")]
    pub ray_steps: u32,
    #[serde(default)]
    pub cast_shadows: bool,
    /// Cheap shell layers: 1 = single deck, 2 = low deck + cirrus.
    #[serde(default = "default_cloud_layers")]
    pub layers: u32,
    /// Coverage threshold over the procedural fBm field (0 = overcast, 1 = clear).
    #[serde(default = "default_cloud_coverage")]
    pub coverage: f32,
    /// Alpha gain over the shell texture.
    #[serde(default = "default_cloud_opacity")]
    pub opacity: f32,
    /// Rotate decks differentially; off = static (cheapest, still shaded).
    #[serde(default = "default_true")]
    pub animate: bool,
}

fn default_cloud_steps() -> u32 {
    16
}

fn default_cloud_layers() -> u32 {
    1
}

fn default_cloud_coverage() -> f32 {
    0.45
}

fn default_cloud_opacity() -> f32 {
    0.9
}

impl Default for CloudSettings {
    fn default() -> Self {
        Self {
            enabled: false,
            quality: Quality::Medium,
            volumetric: false,
            ray_steps: 16,
            cast_shadows: false,
            layers: default_cloud_layers(),
            coverage: default_cloud_coverage(),
            opacity: default_cloud_opacity(),
            animate: true,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum AuroraQuality {
    #[default]
    Low,
    High,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct UpperAtmosphereSettings {
    #[serde(default = "default_true")]
    pub airglow: bool,
    #[serde(default = "default_true")]
    pub aurora: bool,
    #[serde(default)]
    pub aurora_quality: AuroraQuality,
    #[serde(default)]
    pub aurora_lighting: bool,
    /// Emission gain over the analytic oval model (appearance only).
    #[serde(default = "default_aurora_intensity")]
    pub aurora_intensity: f32,
    /// Slide curtains with sim time; off = static oval (cheapest).
    #[serde(default = "default_true")]
    pub aurora_animate: bool,
}

fn default_aurora_intensity() -> f32 {
    1.0
}

impl Default for UpperAtmosphereSettings {
    fn default() -> Self {
        Self {
            airglow: true,
            aurora: true,
            aurora_quality: AuroraQuality::Low,
            aurora_lighting: false,
            aurora_intensity: default_aurora_intensity(),
            aurora_animate: true,
        }
    }
}

/// Cheap gas-giant look: procedural band textures baked once on the CPU,
/// no custom shader, no per-frame cost beyond a slow mesh spin.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GasGiantSettings {
    #[serde(default = "default_true")]
    pub enabled: bool,
    #[serde(default)]
    pub quality: Quality,
    /// Slow band drift / storm rotation; off = static (cheapest).
    #[serde(default = "default_true")]
    pub animate_bands: bool,
    /// Boost belt/zone contrast baked into the texture (appearance only).
    #[serde(default = "default_true")]
    pub limb_darkening: bool,
}

impl Default for GasGiantSettings {
    fn default() -> Self {
        Self {
            enabled: true,
            quality: Quality::Medium,
            animate_bands: true,
            limb_darkening: true,
        }
    }
}

/// Cheap realistic plume: cone mesh + baked gradient/Mach-diamond texture +
/// flicker + one optional point light. No particles, no volumetrics.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EnginePlumeSettings {
    #[serde(default = "default_true")]
    pub enabled: bool,
    #[serde(default)]
    pub quality: Quality,
    /// Mach-diamond bands baked into the emissive texture.
    #[serde(default = "default_true")]
    pub mach_diamonds: bool,
    /// Throttle-driven flicker; off = steady plume (cheapest).
    #[serde(default = "default_true")]
    pub flicker: bool,
    /// One point light at the nozzle; off saves a forward light.
    #[serde(default = "default_true")]
    pub light: bool,
}

impl Default for EnginePlumeSettings {
    fn default() -> Self {
        Self {
            enabled: true,
            quality: Quality::Medium,
            mach_diamonds: true,
            flicker: true,
            light: true,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RaytracingParticipation {
    #[serde(default = "default_true")]
    pub terrain: bool,
    #[serde(default = "default_true")]
    pub vehicles: bool,
    #[serde(default)]
    pub landmarks: bool,
    #[serde(default = "default_true")]
    pub atmosphere: bool,
    #[serde(default)]
    pub clouds: bool,
    #[serde(default = "default_rt_distance")]
    pub max_distance_m: f64,
}

fn default_rt_distance() -> f64 {
    500_000.0
}

impl Default for RaytracingParticipation {
    fn default() -> Self {
        Self {
            terrain: true,
            vehicles: true,
            landmarks: false,
            atmosphere: true,
            clouds: false,
            max_distance_m: 500_000.0,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct DebugSettings {
    #[serde(default)]
    pub show_atmosphere_bounds: bool,
    #[serde(default)]
    pub show_rt_proxies: bool,
}

/// Sun shadow map cascade coverage. The engine default ends at 150 m, so a
/// survey camera at kilometres would see no self-shadowing at all; these
/// bounds carry planetary-scale cover explicitly instead of magic numbers
/// in the client.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ShadowSettings {
    /// Master switch for the raster shadow path (RT lighting ignores it).
    #[serde(default = "default_true")]
    pub enabled: bool,
    #[serde(default)]
    pub quality: Quality,
    /// Cascade count, 1..=4.
    #[serde(default = "default_shadow_cascades")]
    pub cascades: u32,
    /// Far end of cascade cover in metres. Near stays sub-metre for pilot
    /// detail; the first cascade ends at max/40.
    #[serde(default = "default_shadow_distance")]
    pub max_distance_m: f64,
    /// Shadow texel grid per cascade (power of two, 512..=8192).
    #[serde(default = "default_shadow_map_size")]
    pub map_size: u32,
    /// Normal offset in metres against acne on 32 m mesh cells.
    #[serde(default = "default_shadow_normal_bias")]
    pub normal_bias_m: f64,
}

fn default_shadow_cascades() -> u32 {
    4
}

fn default_shadow_distance() -> f64 {
    12_000.0
}

fn default_shadow_map_size() -> u32 {
    4096
}

fn default_shadow_normal_bias() -> f64 {
    1.5
}

impl Default for ShadowSettings {
    fn default() -> Self {
        Self {
            enabled: true,
            quality: Quality::High,
            cascades: default_shadow_cascades(),
            max_distance_m: default_shadow_distance(),
            map_size: default_shadow_map_size(),
            normal_bias_m: default_shadow_normal_bias(),
        }
    }
}

/// Requested graphics configuration as parsed from `graphics.toml`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RequestedGraphics {
    #[serde(default = "default_config_version")]
    pub version: u32,
    #[serde(default)]
    pub preset: Preset,
    #[serde(default)]
    pub renderer: RendererSettings,
    #[serde(default)]
    pub atmosphere: AtmosphereSettings,
    #[serde(default)]
    pub clouds: CloudSettings,
    #[serde(default)]
    pub upper_atmosphere: UpperAtmosphereSettings,
    #[serde(default)]
    pub raytracing: RaytracingParticipation,
    #[serde(default)]
    pub shadows: ShadowSettings,
    #[serde(default)]
    pub gas_giant: GasGiantSettings,
    #[serde(default)]
    pub engine_plume: EnginePlumeSettings,
    #[serde(default)]
    pub debug: DebugSettings,
}

fn default_config_version() -> u32 {
    1
}

impl Default for RequestedGraphics {
    fn default() -> Self {
        Self {
            version: 1,
            preset: Preset::High,
            renderer: RendererSettings::default(),
            atmosphere: AtmosphereSettings::default(),
            clouds: CloudSettings::default(),
            upper_atmosphere: UpperAtmosphereSettings::default(),
            raytracing: RaytracingParticipation::default(),
            shadows: ShadowSettings::default(),
            gas_giant: GasGiantSettings::default(),
            engine_plume: EnginePlumeSettings::default(),
            debug: DebugSettings::default(),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConfigError {
    UnsupportedVersion { got: u32 },
    Parse(String),
    InvalidValue { path: &'static str, detail: String },
}

impl std::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::UnsupportedVersion { got } => {
                write!(f, "unsupported graphics.toml version {got} (expected 1)")
            }
            Self::Parse(detail) => write!(f, "cannot parse graphics.toml: {detail}"),
            Self::InvalidValue { path, detail } => {
                write!(f, "invalid graphics setting {path}: {detail}")
            }
        }
    }
}

impl std::error::Error for ConfigError {}

impl RequestedGraphics {
    /// Parse TOML text. Unknown keys are ignored so future settings and the
    /// GUI round-trip do not break older builds.
    pub fn from_toml(text: &str) -> Result<Self, ConfigError> {
        let config: Self = toml::from_str(text).map_err(|e| ConfigError::Parse(e.to_string()))?;
        config.validate()?;
        Ok(config)
    }

    fn validate(&self) -> Result<(), ConfigError> {
        if self.version != 1 {
            return Err(ConfigError::UnsupportedVersion { got: self.version });
        }
        if !(0.25..=4.0).contains(&self.renderer.resolution_scale) {
            return Err(ConfigError::InvalidValue {
                path: "renderer.resolution_scale",
                detail: format!(
                    "expected 0.25..=4.0, got {}",
                    self.renderer.resolution_scale
                ),
            });
        }
        if !(8..=64).contains(&self.renderer.terrain_mesh_cells) {
            return Err(ConfigError::InvalidValue {
                path: "renderer.terrain_mesh_cells",
                detail: format!("expected 8..=64, got {}", self.renderer.terrain_mesh_cells),
            });
        }
        if !(1.0..=20.0).contains(&self.renderer.exposure_ev100) {
            return Err(ConfigError::InvalidValue {
                path: "renderer.exposure_ev100",
                detail: format!("expected 1..=20, got {}", self.renderer.exposure_ev100),
            });
        }
        if !(4..=128).contains(&self.atmosphere.ray_steps) {
            return Err(ConfigError::InvalidValue {
                path: "atmosphere.ray_steps",
                detail: format!("expected 4..=128, got {}", self.atmosphere.ray_steps),
            });
        }
        if !(4..=128).contains(&self.clouds.ray_steps) {
            return Err(ConfigError::InvalidValue {
                path: "clouds.ray_steps",
                detail: format!("expected 4..=128, got {}", self.clouds.ray_steps),
            });
        }
        if !(1..=2).contains(&self.clouds.layers) {
            return Err(ConfigError::InvalidValue {
                path: "clouds.layers",
                detail: format!("expected 1..=2, got {}", self.clouds.layers),
            });
        }
        if !(0.0..=1.0).contains(&self.clouds.coverage) {
            return Err(ConfigError::InvalidValue {
                path: "clouds.coverage",
                detail: format!("expected 0..=1, got {}", self.clouds.coverage),
            });
        }
        if !(0.0..=1.0).contains(&self.clouds.opacity) {
            return Err(ConfigError::InvalidValue {
                path: "clouds.opacity",
                detail: format!("expected 0..=1, got {}", self.clouds.opacity),
            });
        }
        if !(0.0..=4.0).contains(&self.upper_atmosphere.aurora_intensity) {
            return Err(ConfigError::InvalidValue {
                path: "upper_atmosphere.aurora_intensity",
                detail: format!(
                    "expected 0..=4, got {}",
                    self.upper_atmosphere.aurora_intensity
                ),
            });
        }
        if !self.raytracing.max_distance_m.is_finite() || self.raytracing.max_distance_m < 0.0 {
            return Err(ConfigError::InvalidValue {
                path: "raytracing.max_distance_m",
                detail: "expected a finite value >= 0".to_string(),
            });
        }
        if !(1..=4).contains(&self.shadows.cascades) {
            return Err(ConfigError::InvalidValue {
                path: "shadows.cascades",
                detail: format!("expected 1..=4, got {}", self.shadows.cascades),
            });
        }
        if !(100.0..=100_000.0).contains(&self.shadows.max_distance_m) {
            return Err(ConfigError::InvalidValue {
                path: "shadows.max_distance_m",
                detail: format!("expected 100..=100000, got {}", self.shadows.max_distance_m),
            });
        }
        if !(512..=8192).contains(&self.shadows.map_size)
            || !self.shadows.map_size.is_power_of_two()
        {
            return Err(ConfigError::InvalidValue {
                path: "shadows.map_size",
                detail: format!(
                    "expected a power of two in 512..=8192, got {}",
                    self.shadows.map_size
                ),
            });
        }
        if !(0.0..=50.0).contains(&self.shadows.normal_bias_m) {
            return Err(ConfigError::InvalidValue {
                path: "shadows.normal_bias_m",
                detail: format!("expected 0..=50, got {}", self.shadows.normal_bias_m),
            });
        }
        Ok(())
    }

    /// Expand a preset over the current values. Preset fields that carry
    /// budget meaning are overwritten; the preset label is stored as-is.
    pub fn apply_preset(&mut self, preset: Preset) {
        self.preset = preset;
        // Bloom is deliberately opt-in for every preset: it costs a full
        // mip-chain at native resolution and does not improve terrain detail.
        self.renderer.bloom = false;
        match preset {
            Preset::Low => {
                self.renderer.resolution_scale = 0.75;
                self.renderer.ray_tracing = RayTracingRequest::Off;
                self.renderer.terrain_mesh_cells = 16;
                self.atmosphere.quality = Quality::Low;
                self.atmosphere.ray_steps = 8;
                self.atmosphere.aerial_perspective = false;
                self.atmosphere.limb_scattering = true;
                self.clouds.enabled = false;
                self.clouds.layers = 1;
                self.clouds.animate = false;
                self.gas_giant.quality = Quality::Low;
                self.gas_giant.animate_bands = false;
                self.engine_plume.quality = Quality::Low;
                self.engine_plume.light = false;
                self.engine_plume.mach_diamonds = false;
                self.engine_plume.flicker = false;
                self.shadows.quality = Quality::Low;
                self.shadows.cascades = 2;
                self.shadows.max_distance_m = 3000.0;
                self.shadows.map_size = 1024;
                self.shadows.normal_bias_m = 2.0;
                self.upper_atmosphere.aurora_quality = AuroraQuality::Low;
                self.upper_atmosphere.aurora_lighting = false;
            }
            Preset::Medium => {
                self.renderer.resolution_scale = 1.0;
                self.renderer.ray_tracing = RayTracingRequest::Auto;
                self.renderer.terrain_mesh_cells = 20;
                self.atmosphere.quality = Quality::Medium;
                self.atmosphere.ray_steps = 16;
                self.atmosphere.aerial_perspective = true;
                self.clouds.enabled = false;
                self.clouds.layers = 1;
                self.clouds.animate = true;
                self.gas_giant.quality = Quality::Medium;
                self.gas_giant.animate_bands = true;
                self.engine_plume.quality = Quality::Medium;
                self.engine_plume.light = true;
                self.engine_plume.mach_diamonds = true;
                self.engine_plume.flicker = true;
                self.shadows.quality = Quality::Medium;
                self.shadows.cascades = 4;
                self.shadows.max_distance_m = 6000.0;
                self.shadows.map_size = 2048;
                self.shadows.normal_bias_m = 1.5;
                self.upper_atmosphere.aurora_quality = AuroraQuality::Low;
            }
            Preset::High => {
                self.renderer.resolution_scale = 1.0;
                self.renderer.ray_tracing = RayTracingRequest::Auto;
                self.renderer.terrain_mesh_cells = 24;
                self.atmosphere.quality = Quality::High;
                self.atmosphere.ray_steps = 24;
                self.atmosphere.aerial_perspective = true;
                self.clouds.enabled = false;
                self.clouds.layers = 1;
                self.clouds.animate = true;
                self.gas_giant.quality = Quality::High;
                self.gas_giant.animate_bands = true;
                self.engine_plume.quality = Quality::High;
                self.engine_plume.light = true;
                self.engine_plume.mach_diamonds = true;
                self.engine_plume.flicker = true;
                self.shadows.quality = Quality::High;
                self.shadows.cascades = 4;
                self.shadows.max_distance_m = 12_000.0;
                self.shadows.map_size = 4096;
                self.shadows.normal_bias_m = 1.5;
                self.upper_atmosphere.aurora_quality = AuroraQuality::Low;
                self.upper_atmosphere.aurora_intensity = 1.0;
                self.upper_atmosphere.aurora_animate = true;
            }
            Preset::Ultra => {
                self.renderer.resolution_scale = 1.0;
                self.renderer.ray_tracing = RayTracingRequest::Auto;
                self.renderer.terrain_mesh_cells = 32;
                self.atmosphere.quality = Quality::High;
                self.atmosphere.ray_steps = 32;
                self.atmosphere.aerial_perspective = true;
                self.clouds.enabled = false;
                self.clouds.layers = 2;
                self.clouds.animate = true;
                self.gas_giant.quality = Quality::High;
                self.gas_giant.animate_bands = true;
                self.engine_plume.quality = Quality::High;
                self.engine_plume.light = true;
                self.engine_plume.mach_diamonds = true;
                self.engine_plume.flicker = true;
                self.shadows.quality = Quality::High;
                self.shadows.cascades = 4;
                self.shadows.max_distance_m = 20_000.0;
                self.shadows.map_size = 4096;
                self.shadows.normal_bias_m = 1.0;
                self.upper_atmosphere.aurora_quality = AuroraQuality::High;
                self.upper_atmosphere.aurora_intensity = 1.2;
                self.upper_atmosphere.aurora_animate = true;
            }
            Preset::Custom => {}
        }
    }

    /// True when budget-carrying values diverge from the named preset
    /// expansion, in which case UIs should display `custom`.
    pub fn is_custom(&self) -> bool {
        if self.preset == Preset::Custom {
            return true;
        }
        let mut expanded = self.clone();
        expanded.apply_preset(self.preset);
        expanded.renderer.backend = self.renderer.backend;
        expanded.preset = Preset::Custom;
        let mut current = self.clone();
        current.preset = Preset::Custom;
        // Backend and terrain requests are intent, not preset budgets: ignore
        // them for custom detection.
        current.renderer.backend = expanded.renderer.backend;
        current.renderer.terrain = expanded.renderer.terrain;
        current != expanded
    }

    /// Effective preset label for display and capture metadata.
    pub fn effective_preset_label(&self) -> &'static str {
        if self.is_custom() {
            "custom"
        } else {
            self.preset.as_str()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn checked_in_config_parses() {
        let text = include_str!("../../../graphics.toml");
        let config = RequestedGraphics::from_toml(text).unwrap();
        assert_eq!(config.version, 1);
        assert_eq!(config.renderer.terrain, TerrainRenderRequest::Cpu);
    }

    #[test]
    fn presets_expand_and_custom_detected() {
        let mut config = RequestedGraphics::default();
        config.apply_preset(Preset::Low);
        assert_eq!(config.atmosphere.ray_steps, 8);
        assert!(!config.is_custom());
        config.atmosphere.ray_steps = 64;
        assert!(config.is_custom());
        assert_eq!(config.effective_preset_label(), "custom");
    }

    #[test]
    fn invalid_values_rejected() {
        let bad = "preset = \"high\"\n[renderer]\nresolution_scale = 99.0\n";
        assert!(RequestedGraphics::from_toml(bad).is_err());
        let bad_steps = "preset = \"high\"\n[atmosphere]\nray_steps = 2\n";
        assert!(RequestedGraphics::from_toml(bad_steps).is_err());
        let bad_cloud_steps = "preset = \"high\"\n[clouds]\nray_steps = 2\n";
        assert!(RequestedGraphics::from_toml(bad_cloud_steps).is_err());
        let bad_terrain_cells = "preset = \"high\"\n[renderer]\nterrain_mesh_cells = 4\n";
        assert!(RequestedGraphics::from_toml(bad_terrain_cells).is_err());
        for value in ["nan", "inf", "-inf"] {
            let bad_rt_distance = format!("[raytracing]\nmax_distance_m = {value}\n");
            assert!(RequestedGraphics::from_toml(&bad_rt_distance).is_err());
        }
    }

    #[test]
    fn shadow_budgets_expand_with_presets_and_validate() {
        let mut config = RequestedGraphics::default();
        config.apply_preset(Preset::Low);
        assert_eq!(config.shadows.cascades, 2);
        assert_eq!(config.shadows.map_size, 1024);
        assert!(!config.is_custom());
        config.apply_preset(Preset::High);
        assert_eq!(config.shadows.cascades, 4);
        assert_eq!(config.shadows.max_distance_m, 12_000.0);
        assert!(!config.is_custom());
        for bad in [
            "preset = \"high\"\n[shadows]\ncascades = 9\n",
            "preset = \"high\"\n[shadows]\nmap_size = 1000\n",
            "preset = \"high\"\n[shadows]\nmax_distance_m = -5.0\n",
            "preset = \"high\"\n[shadows]\nnormal_bias_m = 500.0\n",
        ] {
            assert!(RequestedGraphics::from_toml(bad).is_err(), "{bad}");
        }
        // Missing section still parses (old configs without shadows).
        let legacy = "preset = \"high\"\n[renderer]\nresolution_scale = 1.0\n";
        let parsed = RequestedGraphics::from_toml(legacy).unwrap();
        assert_eq!(parsed.shadows.cascades, 4);
        assert!(parsed.shadows.enabled);
    }

    #[test]
    fn unknown_keys_ignored_for_forward_compat() {
        let text = "preset = \"high\"\n[renderer]\nray_tracing = \"auto\"\nflux_capacitor = true\n";
        assert!(RequestedGraphics::from_toml(text).is_ok());
    }
}

#[cfg(test)]
mod storage_tests {
    use super::*;

    #[test]
    fn storage_names_round_trip() {
        assert_eq!(
            MaterialStorageRequest::from_str_name("rgba_array"),
            Some(MaterialStorageRequest::RgbaArray)
        );
        assert_eq!(
            MaterialStorageRequest::from_str_name("microstore_compact"),
            Some(MaterialStorageRequest::MicrostoreCompact)
        );
        assert_eq!(MaterialStorageRequest::from_str_name("bogus"), None);
        assert_eq!(
            MaterialStorageRequest::default(),
            MaterialStorageRequest::RgbaArray
        );
    }

    #[test]
    fn storage_parses_from_toml() {
        let config: RequestedGraphics =
            toml::from_str("version = 1\n[renderer]\nmaterial_storage = \"microstore_compact\"\n")
                .expect("parses");
        assert_eq!(
            config.renderer.material_storage,
            MaterialStorageRequest::MicrostoreCompact
        );
    }
}
