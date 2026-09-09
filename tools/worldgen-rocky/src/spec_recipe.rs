//! Thessa v0.2 spec recipe: `worldgen_recipe.toml` + `thessa_v02.toml`.
//!
//! Parses the generator-facing recipe, validates it against the body design,
//! and deterministically places `[[feature]]` specs onto the globe.
//! Same seed => same placement, bit for bit.

use serde::{Deserialize, Serialize};

use crate::{
    features::{Feature, PlacedFeature},
    rng,
};

#[derive(Debug, Clone, Deserialize)]
pub struct SpecRecipe {
    pub planet: SpecPlanet,
    #[serde(default)]
    pub readability: SpecReadability,
    #[serde(default)]
    pub terrain: SpecTerrain,
    #[serde(default)]
    pub climate_proxy: SpecClimate,
    #[serde(default)]
    pub geothermal: SpecGeothermal,
    #[serde(default)]
    pub feature: Vec<SpecFeature>,
    #[serde(default)]
    pub preview: SpecPreview,
}

#[derive(Debug, Clone, Deserialize)]
pub struct SpecPlanet {
    pub id: String,
    pub seed: u64,
    pub datum_radius_m: f64,
    pub height_min_m: f64,
    pub height_max_m: f64,
    #[serde(default = "default_map_w")]
    pub global_map_width: u32,
    #[serde(default = "default_map_h")]
    pub global_map_height: u32,
}

fn default_map_w() -> u32 {
    1920
}
fn default_map_h() -> u32 {
    1080
}

#[derive(Debug, Clone, Deserialize)]
pub struct SpecReadability {
    pub preview_width: u32,
    pub preview_height: u32,
    pub major_region_min_scale_km: f64,
    pub macro_feature_strength: f64,
}

impl Default for SpecReadability {
    fn default() -> Self {
        Self {
            preview_width: 480,
            preview_height: 270,
            major_region_min_scale_km: 250.0,
            macro_feature_strength: 0.85,
        }
    }
}

#[derive(Debug, Clone, Deserialize, Default)]
pub struct SpecTerrain {
    #[serde(default)]
    pub mountain_coverage: f64,
    #[serde(default)]
    pub crater_density: f64,
    #[serde(default)]
    pub volcanism: f64,
    #[serde(default)]
    pub erosion_strength: f64,
    #[serde(default)]
    pub glaciation: f64,
    #[serde(default)]
    pub rift_activity: f64,
    #[serde(default)]
    pub roughness_macro: f64,
    #[serde(default)]
    pub plateau_coverage: f64,
}

#[derive(Debug, Clone, Deserialize, Default)]
pub struct SpecClimate {
    #[serde(default)]
    pub nereid_facing_ocean_bias: f64,
    #[serde(default)]
    pub anti_nereid_continentality_bias: f64,
    #[serde(default)]
    pub polar_cooling_strength: f64,
    #[serde(default)]
    pub elevation_cooling_strength: f64,
    #[serde(default)]
    pub eclipse_cooling_strength: f64,
    #[serde(default)]
    pub geothermal_local_warming_strength: f64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct SpecGeothermal {
    pub major_provinces_min: u32,
    pub major_provinces_max: u32,
    pub secondary_fields_min: u32,
    pub secondary_fields_max: u32,
}

impl Default for SpecGeothermal {
    fn default() -> Self {
        Self {
            major_provinces_min: 2,
            major_provinces_max: 4,
            secondary_fields_min: 4,
            secondary_fields_max: 10,
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct SpecPreview {
    pub export_height: bool,
    pub export_biome: bool,
    pub export_geology: bool,
    pub export_hydrology: bool,
    pub export_geothermal: bool,
    pub export_eclipse_exposure: bool,
    pub export_continentality: bool,
}

impl Default for SpecPreview {
    fn default() -> Self {
        Self {
            export_height: true,
            export_biome: true,
            export_geology: true,
            export_hydrology: true,
            export_geothermal: true,
            export_eclipse_exposure: true,
            export_continentality: true,
        }
    }
}

/// One `[[feature]]` entry. Only the fields each kind needs are read;
/// unknown extras are ignored by serde defaulting.
#[derive(Debug, Clone, Deserialize)]
pub struct SpecFeature {
    pub kind: String,
    #[serde(default)]
    pub count: Option<u32>,
    #[serde(default)]
    pub count_min: Option<u32>,
    #[serde(default)]
    pub count_max: Option<u32>,
    #[serde(default)]
    pub diameter_km_min: Option<f64>,
    #[serde(default)]
    pub diameter_km_max: Option<f64>,
    #[serde(default)]
    pub length_km_min: Option<f64>,
    #[serde(default)]
    pub length_km_max: Option<f64>,
    #[serde(default)]
    pub width_km_min: Option<f64>,
    #[serde(default)]
    pub width_km_max: Option<f64>,
    #[serde(default)]
    pub depth_km_min: Option<f64>,
    #[serde(default)]
    pub depth_km_max: Option<f64>,
    #[serde(default)]
    pub height_km_min: Option<f64>,
    #[serde(default)]
    pub height_km_max: Option<f64>,
    #[serde(default)]
    pub peak_height_km_min: Option<f64>,
    #[serde(default)]
    pub peak_height_km_max: Option<f64>,
    #[serde(default)]
    pub chain_length_km_min: Option<f64>,
    #[serde(default)]
    pub chain_length_km_max: Option<f64>,
    #[serde(default)]
    pub central_peak_probability: Option<f64>,
    #[serde(default)]
    pub prefer_anti_nereid: bool,
    #[serde(default)]
    pub prefer_high_latitude: bool,
    #[serde(default)]
    pub landmark_priority: f64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct BodyFile {
    pub body: BodyBlock,
}

#[derive(Debug, Clone, Deserialize)]
pub struct BodyBlock {
    pub mean_radius_km: f64,
    pub height_min_m: f64,
    pub height_max_m: f64,
}

/// Validate recipe + body file agreement (radius, height range).
pub fn validate_spec(recipe: &SpecRecipe, body: &BodyFile) -> Result<(), String> {
    if recipe.planet.id.trim().is_empty() {
        return Err("planet id must not be empty".into());
    }
    if (body.body.mean_radius_km * 1000.0 - recipe.planet.datum_radius_m).abs() > 1.0 {
        return Err("recipe datum_radius_m disagrees with body mean_radius_km".into());
    }
    if (body.body.height_min_m - recipe.planet.height_min_m).abs() > 1e-9
        || (body.body.height_max_m - recipe.planet.height_max_m).abs() > 1e-9
    {
        return Err("recipe height range disagrees with body file".into());
    }
    if recipe.readability.preview_width != 480 || recipe.readability.preview_height != 270 {
        return Err("readability preview must be 480x270 per spec".into());
    }
    for f in &recipe.feature {
        let (lo, hi) = count_range(f);
        if lo > hi {
            return Err(format!("feature {} has inverted count range", f.kind));
        }
        for (name, pair) in [
            ("diameter", (f.diameter_km_min, f.diameter_km_max)),
            ("length", (f.length_km_min, f.length_km_max)),
            ("width", (f.width_km_min, f.width_km_max)),
            ("depth", (f.depth_km_min, f.depth_km_max)),
        ] {
            if let (Some(a), Some(b)) = pair {
                let ordered = a.partial_cmp(&0.0) == Some(std::cmp::Ordering::Greater)
                    && b.partial_cmp(&a) != Some(std::cmp::Ordering::Less);
                if !ordered {
                    return Err(format!("feature {} bad {name} range", f.kind));
                }
            }
        }
    }
    Ok(())
}

fn count_range(f: &SpecFeature) -> (u32, u32) {
    if let Some(c) = f.count {
        (c, c)
    } else {
        (
            f.count_min.unwrap_or(1),
            f.count_max.unwrap_or(f.count_min.unwrap_or(1)),
        )
    }
}

fn sample_range(seed: u64, ch: u32, lo: f64, hi: f64) -> f64 {
    lo + rng::hash01(seed, ch, 0, 0) * (hi - lo).max(0.0)
}

/// Deterministic placement of all `[[feature]]` specs.
/// Major landmarks (priority >= 0.8) keep >= 12 deg separation.
pub fn place_spec_features(recipe: &SpecRecipe) -> Result<Vec<PlacedFeature>, String> {
    let mut out = Vec::new();
    let mut channel: u32 = 1000;
    for spec in &recipe.feature {
        let (lo, hi) = count_range(spec);
        let span = hi.saturating_sub(lo) + 1;
        let n = lo
            + (rng::hash01(recipe.planet.seed, channel, 7, 7) * span as f64) as u32 % span.max(1);
        channel += 1;
        for i in 0..n {
            let seed = recipe.planet.seed ^ ((channel as u64) << 32 | i as u64);
            channel += 1;
            let (lat, lon) = pick_site(recipe.planet.seed, seed, spec, &out)?;
            let feature = build_feature(spec, seed)?;
            let id = format!("{}-{i}", spec.kind.replace('_', "-"));
            out.push(PlacedFeature {
                id,
                seed,
                lat_deg: lat,
                lon_deg: lon,
                rotation_rad: rng::hash01(seed, 900, 0, 0) * std::f64::consts::TAU,
                feature,
            });
        }
    }
    Ok(out)
}

fn pick_site(
    seed: u64,
    item_seed: u64,
    spec: &SpecFeature,
    placed: &[PlacedFeature],
) -> Result<(f64, f64), String> {
    for attempt in 0..24 {
        let ch = 800 + attempt;
        let mut lat = rng::hash11(seed ^ item_seed, ch, 1, 0) * 60.0;
        let mut lon = rng::hash11(seed ^ item_seed, ch, 0, 1) * 170.0;
        if spec.prefer_high_latitude {
            lat = lat.signum().max(0.2) * (45.0 + rng::hash01(seed ^ item_seed, ch, 2, 2) * 30.0);
            let _ = &mut lon;
        }
        if spec.prefer_anti_nereid {
            // Anti-Nereid side is lon +/-180: fold longitude outward.
            lon = 180.0 - rng::hash01(seed ^ item_seed, ch, 3, 3) * 120.0;
            if rng::hash01(seed ^ item_seed, ch, 4, 4) < 0.5 {
                lon = -lon;
            }
        }
        if spec.landmark_priority < 0.8 {
            return Ok((lat.clamp(-80.0, 80.0), lon));
        }
        // Separation for major landmarks.
        let clear = placed.iter().all(|p| {
            let dlat = (lat - p.lat_deg).abs();
            let mut dlon = (lon - p.lon_deg).abs();
            if dlon > 180.0 {
                dlon = 360.0 - dlon;
            }
            (dlat.powi(2) + dlon.powi(2)).sqrt() > 12.0
        });
        if clear {
            return Ok((lat.clamp(-80.0, 80.0), lon));
        }
    }
    Err(format!("could not separate landmark {}", spec.kind))
}

fn km_range(seed: u64, ch: u32, lo: Option<f64>, hi: Option<f64>, fallback: f64) -> f64 {
    match (lo, hi) {
        (Some(a), Some(b)) => sample_range(seed, ch, a, b) * 1000.0,
        _ => fallback,
    }
}

fn build_feature(spec: &SpecFeature, seed: u64) -> Result<Feature, String> {
    Ok(match spec.kind.as_str() {
        "impact_basin" => Feature::ImpactBasin {
            radius_m: km_range(
                seed,
                1,
                spec.diameter_km_min,
                spec.diameter_km_max,
                1_500_000.0,
            ) / 2.0,
            depth_m: km_range(seed, 2, spec.depth_km_min, spec.depth_km_max, 3500.0),
            rings: 3,
        },
        "mountain_arc" | "glaciated_mountain_coast" => Feature::MountainRange {
            length_m: km_range(seed, 3, spec.length_km_min, spec.length_km_max, 1_200_000.0),
            width_m: km_range(seed, 4, spec.width_km_min, spec.width_km_max, 200_000.0),
            height_m: km_range(
                seed,
                5,
                spec.peak_height_km_min.or(spec.height_km_min),
                spec.peak_height_km_max.or(spec.height_km_max),
                6500.0,
            ),
            branches: 5,
        },
        "rift_canyon_system" => Feature::Canyon {
            length_m: km_range(seed, 6, spec.length_km_min, spec.length_km_max, 900_000.0),
            width_m: km_range(seed, 7, spec.width_km_min, spec.width_km_max, 80_000.0),
            depth_m: km_range(seed, 8, spec.depth_km_min, spec.depth_km_max, 2500.0),
            tributaries: 4,
        },
        "volcanic_province" => Feature::VolcanicProvince {
            radius_m: km_range(
                seed,
                9,
                spec.diameter_km_min,
                spec.diameter_km_max,
                600_000.0,
            ) / 2.0,
            swell_m: 1500.0,
            shields: 5,
        },
        "shield_volcano_caldera_complex" => Feature::ShieldVolcano {
            radius_m: km_range(
                seed,
                10,
                spec.diameter_km_min,
                spec.diameter_km_max,
                180_000.0,
            ) / 2.0,
            height_m: km_range(seed, 11, spec.height_km_min, spec.height_km_max, 4500.0),
            caldera: true,
        },
        "large_crater" => {
            let rim = km_range(
                seed,
                12,
                spec.diameter_km_min,
                spec.diameter_km_max,
                200_000.0,
            ) / 2.0;
            Feature::Crater {
                rim_radius_m: rim,
                depth_m: rim * 0.12,
                central_peak: rng::hash01(seed, 13, 0, 0)
                    < spec.central_peak_probability.unwrap_or(0.65),
                ejecta: true,
            }
        }
        "dry_plateau_salt_basin_complex" => Feature::SaltBasin {
            radius_m: km_range(
                seed,
                14,
                spec.diameter_km_min,
                spec.diameter_km_max,
                600_000.0,
            ) / 2.0,
            depth_m: 700.0,
        },
        "archipelago" => {
            let chain = km_range(
                seed,
                15,
                spec.chain_length_km_min,
                spec.chain_length_km_max,
                500_000.0,
            );
            Feature::Archipelago {
                chain_length_m: chain,
                island_radius_m: (chain / 12.0).max(15_000.0),
                islands: 6,
            }
        }
        other => return Err(format!("unknown spec feature kind {other}")),
    })
}

/// Physical size constraints per kind (diameter/length in metres).
pub fn feature_size_ok(kind: &str, f: &Feature) -> bool {
    let (len_m, min_ok, max_ok) = match (kind, f) {
        ("impact_basin", Feature::ImpactBasin { radius_m, .. }) => {
            (radius_m * 2.0, 1_200_000.0, 2_000_000.0)
        }
        ("mountain_arc", Feature::MountainRange { length_m, .. }) => {
            (*length_m, 1_000_000.0, 1_800_000.0)
        }
        ("rift_canyon_system", Feature::Canyon { length_m, .. }) => {
            (*length_m, 600_000.0, 1_400_000.0)
        }
        ("volcanic_province", Feature::VolcanicProvince { radius_m, .. }) => {
            (radius_m * 2.0, 400_000.0, 900_000.0)
        }
        _ => return true,
    };
    len_m >= min_ok && len_m <= max_ok
}

#[derive(Debug, Clone, Serialize)]
pub struct SpecSummary {
    pub placed: usize,
    pub kinds: Vec<(String, usize)>,
}

use crate::manifest::{
    ClimateRecipe, ErosionRecipe, Gores, Layer, Manifest, Planet, ReadabilityRecipe, TerrainRecipe,
};

/// Build a runnable [`Manifest`] from the spec recipe.
pub fn manifest_from_spec(recipe: &SpecRecipe) -> Result<Manifest, String> {
    let features = place_spec_features(recipe)?;
    let layers = [
        "height",
        "albedo",
        "biomes",
        "roughness",
        "normal",
        "hydrology",
        "minerals",
    ]
    .iter()
    .map(|name| Layer {
        name: name.to_string(),
        file: format!("{name}_gore_{{i}}.png"),
        prompt: format!("prompts/{name}.md"),
    })
    .collect();
    Ok(Manifest {
        planet: Planet {
            name: recipe.planet.id.clone(),
            kind: "rocky".into(),
            datum_radius_m: recipe.planet.datum_radius_m,
            height_min_m: recipe.planet.height_min_m,
            height_max_m: recipe.planet.height_max_m,
            seed: recipe.planet.seed,
        },
        gores: Gores {
            count: 8,
            overlap_fraction: 0.06,
            polar_caps: true,
        },
        layers,
        terrain: TerrainRecipe {
            mountain_coverage: recipe.terrain.mountain_coverage,
            crater_density: recipe.terrain.crater_density,
            tectonic_activity: 0.5,
            volcanism: recipe.terrain.volcanism,
            erosion: recipe.terrain.erosion_strength,
            canyon_strength: recipe.terrain.rift_activity,
            roughness: recipe.terrain.roughness_macro,
        },
        climate: ClimateRecipe {
            polar_extent: 0.12 + recipe.terrain.glaciation * 0.2,
            aridity: 0.35 + 0.6 * recipe.climate_proxy.anti_nereid_continentality_bias,
            glaciation: recipe.terrain.glaciation,
            nereid_ocean_bias: recipe.climate_proxy.nereid_facing_ocean_bias,
            anti_nereid_land_bias: recipe.climate_proxy.anti_nereid_continentality_bias,
        },
        readability: ReadabilityRecipe {
            macro_feature_strength: recipe.readability.macro_feature_strength,
            biome_min_scale_km: recipe.readability.major_region_min_scale_km,
            landmark_density: 0.7,
        },
        tectonics: Vec::new(),
        erosion: ErosionRecipe {
            thermal_iters: (4.0 + recipe.terrain.erosion_strength * 12.0) as u32,
            talus_deg: 34.0,
            droplets: 4000,
            droplet_steps: 64,
        },
        // Spec ocean target 0.52..0.68: calibrate datum to the midpoint.
        ocean_target: Some(0.60),
        features,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn load() -> (SpecRecipe, BodyFile) {
        let recipe: SpecRecipe =
            toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml"))
                .expect("recipe parses");
        let body: BodyFile = toml::from_str(include_str!("../../../data/worldgen/thessa_v02.toml"))
            .expect("body parses");
        (recipe, body)
    }

    #[test]
    fn spec_files_validate_together() {
        let (recipe, body) = load();
        validate_spec(&recipe, &body).expect("spec valid");
        assert_eq!(recipe.feature.len(), 9);
    }

    #[test]
    fn placement_is_deterministic_and_sized() {
        let (recipe, _) = load();
        let a = place_spec_features(&recipe).expect("place");
        let b = place_spec_features(&recipe).expect("place");
        assert_eq!(a.len(), b.len());
        assert_eq!(a, b);
        // Landmark counts from the recipe ranges.
        assert!((8..=30).contains(&a.len()));
        for f in &a {
            f.validate().expect("placed feature valid");
        }
        // Giant basin within the 1200-2000 km class.
        let basin = a
            .iter()
            .find(|f| f.id.starts_with("impact-basin"))
            .expect("basin placed");
        if let Feature::ImpactBasin { radius_m, .. } = &basin.feature {
            assert!((1_200_000.0..=2_000_000.0).contains(&(radius_m * 2.0)));
        } else {
            panic!("basin kind mismatch");
        }
        for f in &a {
            // Major landmarks must obey the spec size classes.
            let kind = f.id.rsplit_once('-').map(|(k, _)| k).unwrap_or(&f.id);
            let kind = kind.replace('-', "_");
            if [
                "impact_basin",
                "mountain_arc",
                "rift_canyon_system",
                "volcanic_province",
            ]
            .contains(&kind.as_str())
            {
                assert!(
                    feature_size_ok(&kind, &f.feature),
                    "{} {:?}",
                    f.id,
                    f.feature
                );
            }
        }
    }

    #[test]
    fn different_seeds_differ() {
        let (mut recipe, _) = load();
        let a = place_spec_features(&recipe).expect("a");
        recipe.planet.seed = 999;
        let b = place_spec_features(&recipe).expect("b");
        assert_ne!(a, b);
    }

    #[test]
    fn spec_ocean_fraction_within_target() {
        let (recipe, body) = load();
        crate::spec_recipe::validate_spec(&recipe, &body).expect("valid");
        let manifest = manifest_from_spec(&recipe).expect("manifest");
        let report = crate::bake::bake_report(&manifest, 4.0).expect("bake");
        assert!(report.errors.is_empty(), "{:?}", report.errors);
        let ocean_fraction = report.ocean_cells as f64 / report.cells as f64;
        assert!(
            (0.50..=0.70).contains(&ocean_fraction),
            "ocean fraction {ocean_fraction}"
        );
    }
}
