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

impl Quality {
    /// Canonical bake dimensions (width, height) for whole-planet weather maps.
    ///
    /// - Low: 512 x 256
    /// - Medium: 1024 x 512
    /// - High: 2048 x 1024
    #[inline]
    pub const fn cloud_size(self) -> (u32, u32) {
        match self {
            Self::Low => (512, 256),
            Self::Medium => (1024, 512),
            Self::High => (2048, 1024),
        }
    }
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
/// optional hardware mesh shaders share its geometry and material contract.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TerrainRenderRequest {
    #[default]
    Cpu,
    GpuIndexed,
    /// Prefer mesh shaders when supported, otherwise use indexed GPU draw.
    GpuAuto,
    /// Request mesh draw; unsupported devices degrade with a note.
    GpuMesh,
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

pub(crate) fn default_true() -> bool {
    true
}

/// Native temporal signal diagnostics. No history or resolve is enabled yet.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct TemporalSettings {
    /// Produce native-resolution camera motion with explicit invalid masks.
    /// Requires HDR; disabled allocates no motion target or pipeline.
    #[serde(default)]
    pub motion_vectors: bool,
    /// Diagnostic primary-visibility Halton jitter, not antialiasing. Requires
    /// motion diagnostics; without a resolve the image deliberately trembles.
    #[serde(default)]
    pub jitter: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RendererSettings {
    /// Native material quality: Low approximates diffuse/ambient Fresnel by F0.
    #[serde(default)]
    pub surface_quality: Quality,
    /// Native primary terrain normal plane; restart required for residency.
    #[serde(default = "default_true")]
    pub terrain_normal_maps: bool,
    #[serde(default = "default_true")]
    pub specular_reflections: bool,
    /// Maximum native depth-atlas storage across three lights and all cascades.
    #[serde(default = "default_shadow_memory")]
    pub shadow_memory_mib: u32,
    /// Native retained fleet meshes; no effect on authoritative simulation.
    #[serde(default = "default_true")]
    pub fleet_meshes: bool,
    #[serde(default)]
    pub fleet_quality: Quality,
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

fn default_shadow_memory() -> u32 {
    64
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
            surface_quality: Quality::High,
            terrain_normal_maps: true,
            specular_reflections: true,
            shadow_memory_mib: default_shadow_memory(),
            fleet_meshes: true,
            fleet_quality: Quality::High,
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
    /// Native deterministic distant-star catalogue, independent of scattering.
    #[serde(default = "default_true")]
    pub stars: bool,
    #[serde(default)]
    pub stars_quality: Quality,
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
            stars: true,
            stars_quality: Quality::Medium,
            ray_steps: 24,
            density_scale: 1.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum CloudIntegratorRequest {
    /// Real-time stratified ray marching with space skipping (bounded real-time budget on iGPU / Radeon 880M).
    #[default]
    March,
    /// Stochastic scattering approximation with 1-sample Woodcock free-flight estimator.
    Stochastic,
}

impl CloudIntegratorRequest {
    pub fn from_str_name(name: &str) -> Option<Self> {
        match name.to_ascii_lowercase().as_str() {
            "march" | "raymarch" | "0" => Some(Self::March),
            "stochastic" | "1" => Some(Self::Stochastic),
            // Deprecated legacy aliases for Stochastic:
            "pathtrace" | "pt" => Some(Self::Stochastic),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CloudSettings {
    #[serde(default)]
    pub enabled: bool,
    #[serde(default)]
    pub quality: Quality,
    /// Numerical integrator for the volume path: stratified ray march or stochastic scattering.
    #[serde(default)]
    pub integrator: CloudIntegratorRequest,
    /// March view rays through the low-deck slab instead of alpha-blending
    /// shell impostors. Low quality keeps the shell fallback; Medium/High
    /// march with `ray_steps`. The cirrus deck stays a shell in both paths.
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
    0.55
}

fn default_cloud_opacity() -> f32 {
    0.9
}

impl Default for CloudSettings {
    fn default() -> Self {
        Self {
            enabled: false,
            quality: Quality::Medium,
            integrator: CloudIntegratorRequest::default(),
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

/// Night-side city lights: warm emissive clusters over inhabited land,
/// gated by the terminator in-shader. Bake resolution only; the mask is a
/// static optical proxy, never simulated activity.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NightLightsSettings {
    #[serde(default = "default_true")]
    pub enabled: bool,
    #[serde(default)]
    pub quality: Quality,
    /// Emissive gain over the city mask (0 = dark, 2 = bright metro cores).
    #[serde(default = "default_night_lights_intensity")]
    pub intensity: f32,
}

fn default_night_lights_intensity() -> f32 {
    1.0
}

impl Default for NightLightsSettings {
    fn default() -> Self {
        Self {
            enabled: true,
            quality: Quality::Medium,
            intensity: default_night_lights_intensity(),
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

/// Engine plume quality and optional effects. Low uses the impostor cone;
/// Medium/High integrate the analytic field through a volume ribbon.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EnginePlumeSettings {
    #[serde(default = "default_true")]
    pub enabled: bool,
    #[serde(default)]
    pub quality: Quality,
    /// Enable field-derived shock diamonds (and bands on the Low impostor).
    #[serde(default = "default_true")]
    pub mach_diamonds: bool,
    /// Enable plume animation; off freezes the field-time input.
    #[serde(default = "default_true")]
    pub flicker: bool,
    /// Legacy field-derived nozzle point light; the native renderer currently
    /// reports this consumer as unavailable instead of fabricating lighting.
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
    /// Native hit-attribute/texture storage, not driver-owned AS memory.
    #[serde(default = "default_rt_memory")]
    pub scene_memory_mib: u32,
    #[serde(default = "default_rt_instances")]
    pub max_instances: u32,
    #[serde(default = "default_rt_bounces")]
    pub reflection_bounces: u32,
    #[serde(default = "default_ray_bias")]
    pub ray_bias_m: f32,
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
fn default_rt_memory() -> u32 {
    32
}
fn default_rt_instances() -> u32 {
    4096
}
fn default_rt_bounces() -> u32 {
    2
}
fn default_ray_bias() -> f32 {
    0.01
}

impl Default for RaytracingParticipation {
    fn default() -> Self {
        Self {
            terrain: true,
            scene_memory_mib: default_rt_memory(),
            max_instances: default_rt_instances(),
            reflection_bounces: default_rt_bounces(),
            ray_bias_m: default_ray_bias(),
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

/// Render-only finite-depth spectral sea. State is shared across quality tiers;
/// quality changes frequency sampling, never authoritative fluid/contact state.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct OceanSettings {
    pub enabled: bool,
    pub quality: Quality,
    pub wind_speed_m_s: f64,
    pub wind_direction_rad: f64,
    pub significant_height_m: f64,
    pub depth_m: f64,
    pub seed: u64,
    /// Bounded near-field deformation slots; zero is optical-only fallback.
    pub geometry_patches: u32,
    pub geometry_distance_m: f64,
}

impl Default for OceanSettings {
    fn default() -> Self {
        Self {
            enabled: false,
            quality: Quality::High,
            wind_speed_m_s: 10.0,
            wind_direction_rad: 0.0,
            significant_height_m: 2.0,
            depth_m: 100.0,
            seed: 17,
            geometry_patches: 16,
            geometry_distance_m: 300.0,
        }
    }
}

/// Presentation layout only; independent of the active flight-control law.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum InstrumentLayout {
    #[default]
    Aircraft,
    Rocket,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NavballRenderer {
    #[default]
    Auto,
    Analytic,
    Baked,
}

/// Native instruments only. Layout never selects a flight-control law.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct InstrumentSettings {
    pub enabled: bool,
    pub quality: Quality,
    pub layout: InstrumentLayout,
    pub navball_renderer: NavballRenderer,
}
impl Default for InstrumentSettings {
    fn default() -> Self {
        Self {
            enabled: true,
            quality: Quality::High,
            layout: InstrumentLayout::Aircraft,
            navball_renderer: NavballRenderer::Auto,
        }
    }
}

/// Native topology quality changes representation budgets, never simulation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct TerrainTopologySettings {
    /// Native persistent conforming bisectors; false keeps regular patches.
    pub enabled: bool,
    pub quality: Quality,
}
impl Default for TerrainTopologySettings {
    fn default() -> Self {
        Self {
            enabled: false,
            quality: Quality::High,
        }
    }
}

/// Requested graphics configuration as parsed from `graphics.toml`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RequestedGraphics {
    #[serde(default)]
    pub terrain_topology: TerrainTopologySettings,
    #[serde(default)]
    pub instruments: InstrumentSettings,
    #[serde(default)]
    pub ocean: OceanSettings,
    #[serde(default = "default_config_version")]
    pub version: u32,
    #[serde(default)]
    pub preset: Preset,
    #[serde(default)]
    pub renderer: RendererSettings,
    #[serde(default)]
    pub temporal: TemporalSettings,
    #[serde(default)]
    pub atmosphere: AtmosphereSettings,
    #[serde(default)]
    pub clouds: CloudSettings,
    #[serde(default)]
    pub night_lights: NightLightsSettings,
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
            terrain_topology: TerrainTopologySettings::default(),
            instruments: InstrumentSettings::default(),
            ocean: OceanSettings::default(),
            version: 1,
            preset: Preset::High,
            renderer: RendererSettings::default(),
            temporal: TemporalSettings::default(),
            atmosphere: AtmosphereSettings::default(),
            clouds: CloudSettings::default(),
            night_lights: NightLightsSettings::default(),
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
        for (path, value, low, high) in [
            (
                "ocean.wind_speed_m_s",
                self.ocean.wind_speed_m_s,
                0.0,
                100.0,
            ),
            (
                "ocean.wind_direction_rad",
                self.ocean.wind_direction_rad,
                -1e6,
                1e6,
            ),
            (
                "ocean.significant_height_m",
                self.ocean.significant_height_m,
                0.0,
                20.0,
            ),
            ("ocean.depth_m", self.ocean.depth_m, 0.1, 100_000.0),
            (
                "ocean.geometry_distance_m",
                self.ocean.geometry_distance_m,
                0.0,
                20_000.0,
            ),
        ] {
            if !(low..=high).contains(&value) {
                return Err(ConfigError::InvalidValue {
                    path,
                    detail: format!("expected finite {low}..={high}"),
                });
            }
        }
        if self.ocean.geometry_patches > 64 {
            return Err(ConfigError::InvalidValue {
                path: "ocean.geometry_patches",
                detail: "expected 0..=64".into(),
            });
        }
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
        if !(0.0..=2.0).contains(&self.atmosphere.density_scale) {
            return Err(ConfigError::InvalidValue {
                path: "atmosphere.density_scale",
                detail: format!("expected 0..=2, got {}", self.atmosphere.density_scale),
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
        if !(0.0..=2.0).contains(&self.night_lights.intensity) {
            return Err(ConfigError::InvalidValue {
                path: "night_lights.intensity",
                detail: format!("expected 0..=2, got {}", self.night_lights.intensity),
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
        if !(0.0..=1_000_000_000.0).contains(&self.raytracing.max_distance_m) {
            return Err(ConfigError::InvalidValue {
                path: "raytracing.max_distance_m",
                detail: format!(
                    "expected a finite value in 0..=1000000000, got {}",
                    self.raytracing.max_distance_m
                ),
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
        if preset != Preset::Custom {
            self.ocean.quality = match preset {
                Preset::Low => Quality::Low,
                Preset::Medium => Quality::Medium,
                _ => Quality::High,
            };
            self.instruments.quality = self.ocean.quality;
            self.terrain_topology.quality = self.ocean.quality;
        }
        match preset {
            Preset::Low => {
                self.renderer.surface_quality = Quality::Low;
                self.renderer.fleet_quality = Quality::Low;
                self.renderer.resolution_scale = 0.75;
                self.renderer.ray_tracing = RayTracingRequest::Off;
                self.renderer.terrain_mesh_cells = 16;
                self.atmosphere.quality = Quality::Low;
                self.atmosphere.stars_quality = Quality::Low;
                self.atmosphere.ray_steps = 8;
                self.atmosphere.aerial_perspective = false;
                self.atmosphere.limb_scattering = true;
                self.clouds.enabled = false;
                self.clouds.layers = 1;
                self.clouds.animate = false;
                self.clouds.cast_shadows = false;
                self.night_lights.enabled = true;
                self.night_lights.quality = Quality::Low;
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
                self.renderer.surface_quality = Quality::Medium;
                self.renderer.fleet_quality = Quality::Medium;
                self.renderer.resolution_scale = 1.0;
                self.renderer.ray_tracing = RayTracingRequest::Auto;
                self.renderer.terrain_mesh_cells = 20;
                self.atmosphere.quality = Quality::Medium;
                self.atmosphere.stars_quality = Quality::Medium;
                self.atmosphere.ray_steps = 16;
                self.atmosphere.aerial_perspective = true;
                self.clouds.enabled = false;
                self.clouds.layers = 1;
                self.clouds.animate = true;
                self.clouds.cast_shadows = false;
                self.night_lights.enabled = true;
                self.night_lights.quality = Quality::Medium;
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
                self.renderer.surface_quality = Quality::High;
                self.renderer.fleet_quality = Quality::High;
                self.renderer.resolution_scale = 1.0;
                self.renderer.ray_tracing = RayTracingRequest::Auto;
                self.renderer.terrain_mesh_cells = 24;
                self.atmosphere.quality = Quality::High;
                self.atmosphere.stars_quality = Quality::High;
                self.atmosphere.ray_steps = 24;
                self.atmosphere.aerial_perspective = true;
                self.clouds.enabled = false;
                self.clouds.layers = 1;
                self.clouds.animate = true;
                self.clouds.cast_shadows = true;
                self.night_lights.enabled = true;
                self.night_lights.quality = Quality::Medium;
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
                self.renderer.surface_quality = Quality::High;
                self.renderer.fleet_quality = Quality::High;
                self.renderer.resolution_scale = 1.0;
                self.renderer.ray_tracing = RayTracingRequest::Auto;
                self.renderer.terrain_mesh_cells = 32;
                self.atmosphere.quality = Quality::High;
                self.atmosphere.stars_quality = Quality::High;
                self.atmosphere.ray_steps = 32;
                self.atmosphere.aerial_perspective = true;
                self.clouds.enabled = false;
                self.clouds.layers = 2;
                self.clouds.animate = true;
                self.clouds.cast_shadows = true;
                self.night_lights.enabled = true;
                self.night_lights.quality = Quality::High;
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
        assert_eq!(config.renderer.terrain, TerrainRenderRequest::GpuAuto);
    }

    #[test]
    fn instrument_choices_roundtrip_and_resolve_without_changing_flight_state() {
        let request = RequestedGraphics::from_toml(
            "[instruments]\nlayout='rocket'\nnavball_renderer='baked'\nquality='medium'\n",
        )
        .unwrap();
        assert_eq!(request.instruments.layout, InstrumentLayout::Rocket);
        let resolved = crate::resolve_instruments(request.instruments);
        assert_eq!(resolved.atlas_width, 1024);
        assert_eq!(resolved.diameter_points, 256);
        let text = toml::to_string(&request).unwrap();
        assert_eq!(RequestedGraphics::from_toml(&text).unwrap(), request);
        let disabled = crate::resolve_instruments(InstrumentSettings {
            enabled: false,
            ..request.instruments
        });
        assert_eq!(disabled.diameter_points, 0);
        assert_eq!(disabled.atlas_width, 0);
        assert_eq!(
            crate::resolve_instruments(InstrumentSettings::default()).renderer,
            NavballRenderer::Analytic
        );
        assert!(RequestedGraphics::from_toml("[instruments]\nlayout='bogus'").is_err());
    }

    #[test]
    fn ocean_config_roundtrips_validates_and_presets_keep_physical_inputs() {
        let config=RequestedGraphics::from_toml("preset='high'\n[ocean]\nenabled=true\nquality='low'\nwind_speed_m_s=13.0\ndepth_m=9.0\nseed=8\n").unwrap();
        assert!(config.ocean.enabled);
        assert_eq!(config.ocean.quality, Quality::Low);
        let text = toml::to_string(&config).unwrap();
        assert_eq!(RequestedGraphics::from_toml(&text).unwrap(), config);
        let mut selected = config.clone();
        selected.apply_preset(Preset::High);
        assert_eq!(selected.ocean.wind_speed_m_s, 13.0);
        assert_eq!(selected.ocean.depth_m, 9.0);
        assert_eq!(selected.ocean.seed, 8);
        assert!(selected.ocean.enabled);
        assert_eq!(selected.ocean.quality, Quality::High);
        for assignment in [
            "wind_speed_m_s=nan",
            "significant_height_m=-1.0",
            "depth_m=0.0",
            "geometry_patches=65",
            "geometry_distance_m=inf",
        ] {
            assert!(
                RequestedGraphics::from_toml(&format!("[ocean]\n{assignment}")).is_err(),
                "{assignment}"
            );
        }
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
        for value in ["-0.1", "2.1", "nan", "inf"] {
            let bad_density_scale = format!("[atmosphere]\ndensity_scale = {value}\n");
            assert!(RequestedGraphics::from_toml(&bad_density_scale).is_err());
        }
        for value in ["nan", "inf", "-inf"] {
            let bad_rt_distance = format!("[raytracing]\nmax_distance_m = {value}\n");
            assert!(RequestedGraphics::from_toml(&bad_rt_distance).is_err());
        }
        let bad_rt_distance = "[raytracing]\nmax_distance_m = 1000000001.0\n";
        assert!(RequestedGraphics::from_toml(bad_rt_distance).is_err());
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
