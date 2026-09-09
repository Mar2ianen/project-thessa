//! World recipe manifest: planet + gores + layers + terrain/climate/readability.
//!
//! New `[terrain]`, `[climate]`, `[readability]` and `[[features]]` sections
//! are optional with deterministic defaults, so old manifests keep parsing.

use serde::{Deserialize, Serialize};

use crate::{erosion::ErosionKnobs, features::PlacedFeature, tectonics::Boundary};

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct Manifest {
    pub planet: Planet,
    pub gores: Gores,
    pub layers: Vec<Layer>,
    #[serde(default)]
    pub terrain: TerrainRecipe,
    #[serde(default)]
    pub climate: ClimateRecipe,
    #[serde(default)]
    pub readability: ReadabilityRecipe,
    /// Plate boundaries (tectonics-lite). Empty = no tectonic uplift.
    #[serde(default)]
    pub tectonics: Vec<Boundary>,
    /// Erosion passes applied during bake.
    #[serde(default)]
    pub erosion: ErosionRecipe,
    /// Optional sea-level calibration target (ocean fraction 0..1).
    /// None keeps the physical datum untouched (old manifests, Moon).
    #[serde(default)]
    pub ocean_target: Option<f64>,
    #[serde(default)]
    pub features: Vec<PlacedFeature>,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct Planet {
    pub name: String,
    pub kind: String,
    pub datum_radius_m: f64,
    pub height_min_m: f64,
    pub height_max_m: f64,
    pub seed: u64,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct Gores {
    pub count: u32,
    pub overlap_fraction: f64,
    #[serde(default = "default_true")]
    pub polar_caps: bool,
}

fn default_true() -> bool {
    true
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct Layer {
    pub name: String,
    pub file: String,
    pub prompt: String,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct TerrainRecipe {
    pub mountain_coverage: f64,
    pub crater_density: f64,
    pub tectonic_activity: f64,
    pub volcanism: f64,
    pub erosion: f64,
    pub canyon_strength: f64,
    pub roughness: f64,
}

impl Default for TerrainRecipe {
    fn default() -> Self {
        Self {
            mountain_coverage: 0.3,
            crater_density: 0.3,
            tectonic_activity: 0.3,
            volcanism: 0.2,
            erosion: 0.4,
            canyon_strength: 0.3,
            roughness: 0.4,
        }
    }
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct ClimateRecipe {
    pub polar_extent: f64,
    pub aridity: f64,
    pub glaciation: f64,
    /// Facing-side ocean bias 0..1 (depresses sub-Nereid longitudes).
    #[serde(default)]
    pub nereid_ocean_bias: f64,
    /// Anti-Nereid continental bias 0..1 (raises far-side land).
    #[serde(default)]
    pub anti_nereid_land_bias: f64,
}

impl Default for ClimateRecipe {
    fn default() -> Self {
        Self {
            polar_extent: 0.15,
            aridity: 0.4,
            glaciation: 0.2,
            nereid_ocean_bias: 0.0,
            anti_nereid_land_bias: 0.0,
        }
    }
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct ReadabilityRecipe {
    pub macro_feature_strength: f64,
    pub biome_min_scale_km: f64,
    pub landmark_density: f64,
}

impl Default for ReadabilityRecipe {
    fn default() -> Self {
        Self {
            macro_feature_strength: 1.0,
            biome_min_scale_km: 150.0,
            landmark_density: 0.5,
        }
    }
}

/// Erosion passes (simulation-lite, deterministic).
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct ErosionRecipe {
    pub thermal_iters: u32,
    pub talus_deg: f64,
    pub droplets: u32,
    pub droplet_steps: u32,
}

impl Default for ErosionRecipe {
    fn default() -> Self {
        let d = ErosionKnobs::default();
        Self {
            thermal_iters: d.thermal_iters,
            talus_deg: d.talus_deg,
            droplets: d.droplets,
            droplet_steps: d.droplet_steps,
        }
    }
}

pub const CANONICAL_LAYERS: [&str; 7] = [
    "height",
    "albedo",
    "biomes",
    "roughness",
    "normal",
    "hydrology",
    "minerals",
];

pub fn validate_manifest(manifest: &Manifest) -> Result<(), String> {
    if manifest.planet.kind.trim().to_lowercase() != "rocky" {
        return Err(format!(
            "planet kind must be \"rocky\" for this tool, got {:?}",
            manifest.planet.kind
        ));
    }
    if manifest.planet.name.trim().is_empty() {
        return Err("planet name must not be empty".into());
    }
    if !manifest.planet.datum_radius_m.is_finite() || manifest.planet.datum_radius_m <= 0.0 {
        return Err("datum_radius_m must be finite and positive".into());
    }
    if !manifest.planet.height_min_m.is_finite()
        || !manifest.planet.height_max_m.is_finite()
        || manifest
            .planet
            .height_min_m
            .partial_cmp(&manifest.planet.height_max_m)
            != Some(std::cmp::Ordering::Less)
    {
        return Err("need height_min_m < height_max_m, both finite".into());
    }
    if !(4..=16).contains(&manifest.gores.count) {
        return Err("gores.count must be 4..=16 orange slices".into());
    }
    if !manifest.gores.overlap_fraction.is_finite()
        || !(0.0..=0.25).contains(&manifest.gores.overlap_fraction)
    {
        return Err("gores.overlap_fraction must be within 0..=0.25".into());
    }
    if manifest.layers.len() != CANONICAL_LAYERS.len() {
        return Err(format!(
            "need exactly {} layers, got {}",
            CANONICAL_LAYERS.len(),
            manifest.layers.len()
        ));
    }
    for expected in CANONICAL_LAYERS {
        if !manifest.layers.iter().any(|l| l.name == expected) {
            return Err(format!("missing canonical layer {expected:?}"));
        }
    }
    for layer in &manifest.layers {
        if layer.file.trim().is_empty() || layer.prompt.trim().is_empty() {
            return Err(format!("layer {:?} needs file and prompt", layer.name));
        }
    }
    for recipe in [
        manifest.terrain.mountain_coverage,
        manifest.terrain.crater_density,
        manifest.terrain.tectonic_activity,
        manifest.terrain.volcanism,
        manifest.terrain.erosion,
        manifest.terrain.canyon_strength,
        manifest.terrain.roughness,
        manifest.climate.polar_extent,
        manifest.climate.aridity,
        manifest.climate.glaciation,
        manifest.climate.nereid_ocean_bias,
        manifest.climate.anti_nereid_land_bias,
        manifest.readability.macro_feature_strength,
        manifest.readability.landmark_density,
    ] {
        if !recipe.is_finite() || !(0.0..=1.0).contains(&recipe) {
            return Err("recipe fractions must be within 0..=1".into());
        }
    }
    if !manifest.readability.biome_min_scale_km.is_finite()
        || manifest.readability.biome_min_scale_km <= 0.0
    {
        return Err("biome_min_scale_km must be positive".into());
    }
    if manifest
        .ocean_target
        .is_some_and(|t| !t.is_finite() || !(0.05..=0.95).contains(&t))
    {
        return Err("ocean_target must be within 0.05..=0.95".into());
    }
    for feature in &manifest.features {
        feature.validate()?;
    }
    for boundary in &manifest.tectonics {
        boundary.validate()?;
    }
    if manifest.erosion.talus_deg <= 0.0 || manifest.erosion.talus_deg >= 60.0 {
        return Err("erosion.talus_deg must be within (0, 60)".into());
    }
    Ok(())
}
