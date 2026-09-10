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
    #[serde(default = "default_resolution_scale")]
    pub resolution_scale: f32,
    #[serde(default = "default_true")]
    pub vsync: bool,
    #[serde(default = "default_true")]
    pub hdr: bool,
    /// Camera exposure compensation. Raw-sun + atmosphere scenes need ~13.
    #[serde(default = "default_exposure")]
    pub exposure_ev100: f32,
}

fn default_exposure() -> f32 {
    13.0
}

fn default_resolution_scale() -> f32 {
    1.0
}

impl Default for RendererSettings {
    fn default() -> Self {
        Self {
            backend: BackendRequest::Auto,
            ray_tracing: RayTracingRequest::Auto,
            resolution_scale: 1.0,
            vsync: true,
            hdr: true,
            exposure_ev100: 13.0,
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
}

fn default_cloud_steps() -> u32 {
    16
}

impl Default for CloudSettings {
    fn default() -> Self {
        Self {
            enabled: false,
            quality: Quality::Medium,
            volumetric: false,
            ray_steps: 16,
            cast_shadows: false,
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
}

impl Default for UpperAtmosphereSettings {
    fn default() -> Self {
        Self {
            airglow: true,
            aurora: true,
            aurora_quality: AuroraQuality::Low,
            aurora_lighting: false,
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
        if self.raytracing.max_distance_m < 0.0 {
            return Err(ConfigError::InvalidValue {
                path: "raytracing.max_distance_m",
                detail: "expected >= 0".to_string(),
            });
        }
        Ok(())
    }

    /// Expand a preset over the current values. Preset fields that carry
    /// budget meaning are overwritten; the preset label is stored as-is.
    pub fn apply_preset(&mut self, preset: Preset) {
        self.preset = preset;
        match preset {
            Preset::Low => {
                self.renderer.resolution_scale = 0.75;
                self.renderer.ray_tracing = RayTracingRequest::Off;
                self.atmosphere.quality = Quality::Low;
                self.atmosphere.ray_steps = 8;
                self.atmosphere.aerial_perspective = false;
                self.atmosphere.limb_scattering = true;
                self.clouds.enabled = false;
                self.upper_atmosphere.aurora_quality = AuroraQuality::Low;
                self.upper_atmosphere.aurora_lighting = false;
            }
            Preset::Medium => {
                self.renderer.resolution_scale = 1.0;
                self.renderer.ray_tracing = RayTracingRequest::Auto;
                self.atmosphere.quality = Quality::Medium;
                self.atmosphere.ray_steps = 16;
                self.atmosphere.aerial_perspective = true;
                self.clouds.enabled = false;
                self.upper_atmosphere.aurora_quality = AuroraQuality::Low;
            }
            Preset::High => {
                self.renderer.resolution_scale = 1.0;
                self.renderer.ray_tracing = RayTracingRequest::Auto;
                self.atmosphere.quality = Quality::High;
                self.atmosphere.ray_steps = 24;
                self.atmosphere.aerial_perspective = true;
                self.clouds.enabled = false;
                self.upper_atmosphere.aurora_quality = AuroraQuality::Low;
            }
            Preset::Ultra => {
                self.renderer.resolution_scale = 1.0;
                self.renderer.ray_tracing = RayTracingRequest::Auto;
                self.atmosphere.quality = Quality::High;
                self.atmosphere.ray_steps = 32;
                self.atmosphere.aerial_perspective = true;
                self.clouds.enabled = false;
                self.upper_atmosphere.aurora_quality = AuroraQuality::High;
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
        // Backend request is intent, not budget: ignore it for custom detection.
        current.renderer.backend = expanded.renderer.backend;
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
    }

    #[test]
    fn unknown_keys_ignored_for_forward_compat() {
        let text = "preset = \"high\"\n[renderer]\nray_tracing = \"auto\"\nflux_capacitor = true\n";
        assert!(RequestedGraphics::from_toml(text).is_ok());
    }
}
