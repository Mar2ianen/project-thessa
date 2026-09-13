//! Machine-readable setting descriptors for the future GUI (spec section 16).
//!
//! The GUI parses/edits the actual `graphics.toml` model. Each descriptor
//! carries path, label, type, range, units, visibility and restart
//! requirements so the GUI never hides the real model behind opaque presets.

use serde::{Deserialize, Serialize};

/// Setting value kind for typed GUI editors.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SettingKind {
    Bool,
    Float,
    Int,
    Enum,
}

/// One editable setting with GUI metadata.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SettingMeta {
    /// TOML path, e.g. `atmosphere.ray_steps`.
    pub path: &'static str,
    pub label: &'static str,
    pub kind: SettingKind,
    /// Allowed values for enums, `min..=max` text for numerics.
    pub range: &'static str,
    pub step: &'static str,
    pub unit: &'static str,
    pub advanced: bool,
    pub restart_required: bool,
    pub description: &'static str,
}

/// Descriptors for the core settings. Budgets editable at runtime are
/// `restart_required: false`; backend/mode switches need restart.
pub fn describe() -> Vec<SettingMeta> {
    vec![
        SettingMeta {
            path: "preset",
            label: "Graphics preset",
            kind: SettingKind::Enum,
            range: "low | medium | high | ultra | custom",
            step: "-",
            unit: "-",
            advanced: false,
            restart_required: false,
            description: "Bulk starting point; manual edits report custom.",
        },
        SettingMeta {
            path: "renderer.backend",
            label: "Render backend",
            kind: SettingKind::Enum,
            range: "auto | vulkan | metal | webgpu",
            step: "-",
            unit: "-",
            advanced: true,
            restart_required: true,
            description: "wgpu backend preference. Direct DX paths need an ADR.",
        },
        SettingMeta {
            path: "renderer.ray_tracing",
            label: "Ray tracing mode",
            kind: SettingKind::Enum,
            range: "off | local | full | auto",
            step: "-",
            unit: "-",
            advanced: false,
            restart_required: true,
            description: "Experimental Solari path. Explicit modes fail fast without RT hardware.",
        },
        SettingMeta {
            path: "renderer.resolution_scale",
            label: "Resolution scale",
            kind: SettingKind::Float,
            range: "0.25..=4.0",
            step: "0.05",
            unit: "×",
            advanced: false,
            restart_required: false,
            description: "Render resolution multiplier.",
        },
        SettingMeta {
            path: "renderer.vsync",
            label: "VSync",
            kind: SettingKind::Bool,
            range: "true | false",
            step: "-",
            unit: "-",
            advanced: false,
            restart_required: false,
            description: "Vertical sync.",
        },
        SettingMeta {
            path: "renderer.auto_exposure",
            label: "Adaptive exposure",
            kind: SettingKind::Bool,
            range: "true | false",
            step: "-",
            unit: "-",
            advanced: false,
            restart_required: true,
            description: "Adapt camera brightness to daylight, night and eclipses; disable for manual EV100.",
        },
        SettingMeta {
            path: "renderer.exposure_ev100",
            label: "Exposure",
            kind: SettingKind::Float,
            range: "1..=20",
            step: "0.5",
            unit: "EV100",
            advanced: true,
            restart_required: false,
            description: "Camera exposure compensation for raw-sun scenes.",
        },
        SettingMeta {
            path: "atmosphere.enabled",
            label: "Atmosphere",
            kind: SettingKind::Bool,
            range: "true | false",
            step: "-",
            unit: "-",
            advanced: false,
            restart_required: false,
            description: "Master switch for visual atmosphere.",
        },
        SettingMeta {
            path: "atmosphere.quality",
            label: "Atmosphere quality",
            kind: SettingKind::Enum,
            range: "low | medium | high",
            step: "-",
            unit: "-",
            advanced: false,
            restart_required: false,
            description: "Budgets only; never changes physics.",
        },
        SettingMeta {
            path: "atmosphere.ray_steps",
            label: "Atmosphere ray steps",
            kind: SettingKind::Int,
            range: "4..=128",
            step: "4",
            unit: "steps",
            advanced: true,
            restart_required: false,
            description: "View-march steps for CPU optical queries.",
        },
        SettingMeta {
            path: "atmosphere.eclipses",
            label: "Eclipse darkening",
            kind: SettingKind::Bool,
            range: "true | false",
            step: "-",
            unit: "-",
            advanced: false,
            restart_required: false,
            description: "Geometric eclipse dimming of direct light and sky.",
        },
        SettingMeta {
            path: "atmosphere.multi_star",
            label: "Multi-star lighting",
            kind: SettingKind::Bool,
            range: "true | false",
            step: "-",
            unit: "-",
            advanced: false,
            restart_required: false,
            description: "Accept secondary-star contribution in the lighting interface.",
        },
        SettingMeta {
            path: "upper_atmosphere.airglow",
            label: "Airglow",
            kind: SettingKind::Bool,
            range: "true | false",
            step: "-",
            unit: "-",
            advanced: true,
            restart_required: false,
            description: "Weak night-side upper-atmosphere emission.",
        },
        SettingMeta {
            path: "upper_atmosphere.aurora",
            label: "Aurora shell",
            kind: SettingKind::Bool,
            range: "true | false",
            step: "-",
            unit: "-",
            advanced: true,
            restart_required: false,
            description: "Auroral emission shell, independent of scattering.",
        },
        SettingMeta {
            path: "raytracing.max_distance_m",
            label: "RT max distance",
            kind: SettingKind::Float,
            range: "0..=1e9",
            step: "10000",
            unit: "m",
            advanced: true,
            restart_required: false,
            description: "Budget cap for ray-traced participation.",
        },
        SettingMeta {
            path: "debug.show_atmosphere_bounds",
            label: "Show atmosphere bounds",
            kind: SettingKind::Bool,
            range: "true | false",
            step: "-",
            unit: "-",
            advanced: true,
            restart_required: false,
            description: "Debug visualization of the optical shell.",
        },
        SettingMeta {
            path: "shadows.enabled",
            label: "Sun shadows",
            kind: SettingKind::Bool,
            range: "true | false",
            step: "-",
            unit: "-",
            advanced: false,
            restart_required: false,
            description: "Master switch for the raster shadow path.",
        },
        SettingMeta {
            path: "shadows.cascades",
            label: "Shadow cascades",
            kind: SettingKind::Int,
            range: "1..=4",
            step: "1",
            unit: "-",
            advanced: true,
            restart_required: false,
            description: "Cascade count over the shadow cover range.",
        },
        SettingMeta {
            path: "shadows.max_distance_m",
            label: "Shadow cover distance",
            kind: SettingKind::Float,
            range: "100..=100000",
            step: "500",
            unit: "m",
            advanced: true,
            restart_required: false,
            description: "Far end of sun-shadow cascade cover.",
        },
        SettingMeta {
            path: "shadows.map_size",
            label: "Shadow map size",
            kind: SettingKind::Int,
            range: "512 | 1024 | 2048 | 4096 | 8192",
            step: "-",
            unit: "px",
            advanced: true,
            restart_required: true,
            description: "Shadow texel grid per cascade (power of two).",
        },
        SettingMeta {
            path: "shadows.normal_bias_m",
            label: "Shadow normal bias",
            kind: SettingKind::Float,
            range: "0..=50",
            step: "0.5",
            unit: "m",
            advanced: true,
            restart_required: false,
            description: "Normal offset against acne on coarse mesh.",
        },
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn descriptors_cover_mode_switch_and_budgets() {
        let metas = describe();
        let paths: Vec<_> = metas.iter().map(|m| m.path).collect();
        assert!(paths.contains(&"renderer.ray_tracing"));
        assert!(paths.contains(&"atmosphere.ray_steps"));
        let rt = metas
            .iter()
            .find(|m| m.path == "renderer.ray_tracing")
            .unwrap();
        assert!(rt.restart_required);
        let steps = metas
            .iter()
            .find(|m| m.path == "atmosphere.ray_steps")
            .unwrap();
        assert!(!steps.restart_required);
        assert!(!paths.is_empty());
    }
}
