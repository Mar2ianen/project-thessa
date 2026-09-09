//! Bake pipeline: validate -> evaluate -> derive -> consistency-check.
//!
//! Authority split: macro height + broad regions are imported/authored;
//! normal, hydrology, roughness, minerals and final albedo are DERIVED.
//! AI-provided derived layers are hints at most and never override physics.

use std::collections::HashMap;

use serde::{Deserialize, Serialize};

use crate::{
    biomes::{Biome, Geology, SiteClass, parse_biome},
    climate::{continentality_steps, nereid_influence},
    erosion::{ErosionKnobs, erode},
    height::encode_height_01,
    hydro::{HeightGrid, WaterClass, classify_water},
    manifest::Manifest,
    minerals::{MineralRichness, sample_minerals},
    tectonics::eval_uplift_m,
    terrain::{TerrainKnobs, eval_macro_m, eval_meso_m},
};

/// Coarse site classification from height + latitude + recipe + landmarks.
/// Deterministic; fine detail never changes the primary biome.
/// Landmark features override the height-derived biome inside their reach so
/// giant craters, volcanoes and canyons stay visible from orbit.
pub fn classify_site_coarse(
    manifest: &Manifest,
    lat_deg: f64,
    lon_deg: f64,
    height_m: f64,
) -> SiteClass {
    let polar = lat_deg.abs() / 90.0;
    if height_m < 0.0 {
        let shelf = height_m > -800.0;
        let biome = if shelf {
            Biome::ShallowSea
        } else {
            Biome::DeepOcean
        };
        return SiteClass::new(biome, Geology::OceanicCrust);
    }
    if polar > 1.0 - manifest.climate.polar_extent
        || (polar > 0.6 && manifest.climate.glaciation > 0.55)
    {
        return SiteClass::new(Biome::PolarIceCap, Geology::GlacialTill);
    }
    if height_m > 6000.0 {
        return SiteClass::new(Biome::AlpinePeaks, Geology::ContinentalCrust);
    }
    if height_m > 2500.0 {
        let volcanic =
            manifest.terrain.volcanism > 0.5 && ((lat_deg * 3.7 + height_m * 0.001).sin() > 0.3);
        if volcanic {
            return SiteClass::new(Biome::VolcanicField, Geology::Basaltic);
        }
        return SiteClass::new(Biome::MountainRange, Geology::ContinentalCrust);
    }
    if manifest.climate.aridity > 0.6 && polar < 0.5 && height_m < 1500.0 {
        return SiteClass::new(Biome::SandDesert, Geology::Sedimentary);
    }
    let mut site = SiteClass::new(Biome::RockyPlain, Geology::Regolith);
    // Landmark override: strongest in-reach feature wins the biome.
    let mut best = 0.0;
    for feature in &manifest.features {
        let reach = feature.reach_m();
        let dlat_m = (lat_deg - feature.lat_deg).to_radians() * manifest.planet.datum_radius_m;
        let mut dlon = (lon_deg - feature.lon_deg).to_radians();
        if dlon > std::f64::consts::PI {
            dlon -= 2.0 * std::f64::consts::PI;
        }
        if dlon < -std::f64::consts::PI {
            dlon += 2.0 * std::f64::consts::PI;
        }
        let dlon_m = dlon * manifest.planet.datum_radius_m * lat_deg.to_radians().cos().max(0.05);
        let dist = (dlat_m.powi(2) + dlon_m.powi(2)).sqrt();
        if dist < reach {
            let weight = 1.0 - dist / reach;
            if weight > best {
                best = weight;
                site = feature_site(feature);
            }
        }
    }
    site
}

/// Biome/geology implied by a landmark feature.
fn feature_site(feature: &crate::features::PlacedFeature) -> SiteClass {
    use crate::features::Feature;
    match &feature.feature {
        Feature::Crater { .. } => SiteClass::new(Biome::ComplexCrater, Geology::ImpactBreccia),
        Feature::ImpactBasin { .. } => SiteClass::new(Biome::ImpactBasin, Geology::ImpactBreccia),
        Feature::ShieldVolcano { .. } => SiteClass::new(Biome::ShieldVolcano, Geology::Basaltic),
        Feature::Caldera { .. } => SiteClass::new(Biome::Caldera, Geology::Basaltic),
        Feature::LavaField { .. } => SiteClass::new(Biome::LavaFlow, Geology::Basaltic),
        Feature::Canyon { .. } => SiteClass::new(Biome::CanyonProvince, Geology::Sedimentary),
        Feature::GlacierValley { .. } => SiteClass::new(Biome::Glacier, Geology::GlacialTill)
            .with_tags(crate::biomes::FeatureTag::Glacier, None),
        Feature::DuneField { .. } => SiteClass::new(Biome::DuneField, Geology::Sedimentary),
        Feature::MountainRange { .. } | Feature::RidgeChain { .. } => {
            SiteClass::new(Biome::MountainRange, Geology::ContinentalCrust)
        }
        Feature::Plateau { .. } => SiteClass::new(Biome::Plateau, Geology::ContinentalCrust),
        Feature::Escarpment { .. } => SiteClass::new(Biome::Escarpment, Geology::Sedimentary),
        Feature::Archipelago { .. } => SiteClass::new(Biome::Archipelago, Geology::Basaltic),
        Feature::VolcanicProvince { .. } => SiteClass::new(Biome::VolcanicField, Geology::Basaltic),
        Feature::SaltBasin { .. } => SiteClass::new(Biome::SaltFlat, Geology::Evaporite),
    }
}

/// Knobs from the world recipe.
pub fn knobs_from_manifest(manifest: &Manifest) -> TerrainKnobs {
    TerrainKnobs {
        mountain_coverage: manifest.terrain.mountain_coverage,
        crater_density: manifest.terrain.crater_density,
        volcanism: manifest.terrain.volcanism,
        erosion: manifest.terrain.erosion,
        canyon_strength: manifest.terrain.canyon_strength,
        base_roughness_m: 50.0 + 300.0 * manifest.terrain.roughness,
    }
}

/// Evaluate sim-first height grid:
/// provinces + tectonic uplift + landmark features, then erosion passes.
/// Micro detail stays runtime-only.
pub fn evaluate_height_grid(manifest: &Manifest, step_deg: f64) -> Result<HeightGrid, String> {
    evaluate_height_grid_steps(manifest, step_deg, step_deg)
}

/// Same with independent latitude/longitude steps (map exports).
pub fn evaluate_height_grid_steps(
    manifest: &Manifest,
    step_lat_deg: f64,
    step_lon_deg: f64,
) -> Result<HeightGrid, String> {
    for step in [step_lat_deg, step_lon_deg] {
        if !step.is_finite() || step <= 0.0 || step > 10.0 {
            return Err("grid steps must be within (0, 10] degrees".into());
        }
    }
    let knobs = knobs_from_manifest(manifest);
    let strength = manifest.readability.macro_feature_strength;
    let mut lats = Vec::new();
    let mut lat = -90.0;
    while lat <= 90.0 {
        lats.push(lat);
        lat += step_lat_deg;
    }
    let mut lons = Vec::new();
    let mut lon = -180.0;
    while lon < 180.0 {
        lons.push(lon);
        lon += step_lon_deg;
    }
    let mut grid = HeightGrid::new(lats, lons, manifest.planet.datum_radius_m);
    for (r, lat) in grid.lats.clone().iter().enumerate() {
        for (c, lon) in grid.lons.clone().iter().enumerate() {
            let macro_h = eval_macro_m(
                &manifest.features,
                *lat,
                *lon,
                manifest.planet.datum_radius_m,
            ) * strength;
            let meso_h = eval_meso_m(
                manifest.planet.seed,
                knobs,
                *lat,
                *lon,
                manifest.planet.datum_radius_m,
            );
            let tectonic = eval_uplift_m(
                &manifest.tectonics,
                *lat,
                *lon,
                manifest.planet.datum_radius_m,
            ) * strength;
            // Hemispheric design bias: facing side more oceanic, far side
            // more continental. Physical metres, recipe-driven.
            let facing = crate::climate::nereid_influence(*lon);
            let hemi_bias = -4500.0 * manifest.climate.nereid_ocean_bias * facing
                + 3600.0 * manifest.climate.anti_nereid_land_bias * (1.0 - facing);
            let h = tectonic + macro_h + meso_h + hemi_bias;
            grid.h[r][c] = h;
        }
    }
    // Erosion is causal: mountains shed talus, droplets carve downhill.
    erode(
        &mut grid,
        ErosionKnobs {
            thermal_iters: manifest.erosion.thermal_iters,
            talus_deg: manifest.erosion.talus_deg,
            droplets: manifest.erosion.droplets,
            droplet_steps: manifest.erosion.droplet_steps,
        },
    );
    for row in grid.h.iter_mut() {
        for h in row.iter_mut() {
            *h = h.clamp(manifest.planet.height_min_m, manifest.planet.height_max_m);
        }
    }
    if let Some(target) = manifest.ocean_target {
        calibrate_sea_level(&mut grid, target);
        // Re-clamp rails after the datum shift (affects few cells).
        for row in grid.h.iter_mut() {
            for h in row.iter_mut() {
                *h = h.clamp(manifest.planet.height_min_m, manifest.planet.height_max_m);
            }
        }
    }
    Ok(grid)
}

/// Calibrate sea level: uniform shift so the ocean fraction hits `target`.
/// Deterministic (total-order sort). Documented datum adjustment, not physics.
fn calibrate_sea_level(grid: &mut HeightGrid, target: f64) {
    let mut sorted: Vec<f64> = grid.h.iter().flatten().copied().collect();
    sorted.sort_by(f64::total_cmp);
    if sorted.is_empty() {
        return;
    }
    let idx = (target.clamp(0.05, 0.95) * (sorted.len() - 1) as f64).round() as usize;
    let offset = -sorted[idx.min(sorted.len() - 1)];
    for row in grid.h.iter_mut() {
        for h in row.iter_mut() {
            *h += offset;
        }
    }
}

/// Derive tangent-space normal via central differences, in metres.
pub fn derive_normal(grid: &HeightGrid, r: usize, c: usize) -> (f64, f64, f64) {
    let cols = grid.cols();
    let rows = grid.rows();
    let cp = grid.h[r][c.min(cols - 1)];
    let (dy_m, dx_m) = grid.cell_m(r);
    let hx = (grid.h[r][(c + 1) % cols] - grid.h[r][(c + cols - 1) % cols]) / (2.0 * dx_m);
    let r_up = if r > 0 { r - 1 } else { r };
    let r_dn = if r + 1 < rows { r + 1 } else { r };
    let hy = (grid.h[r_dn][c] - grid.h[r_up][c]) / (2.0 * dy_m.max(1.0));
    let _ = cp;
    let inv = 1.0 / (hx * hx + hy * hy + 1.0).sqrt();
    (-hx * inv, -hy * inv, inv)
}

/// Two-pass water classification with driver-based local dryness:
/// ocean mask first, then continentality => per-cell aridity for lakes/salt.
pub fn classify_water_driven(grid: &HeightGrid, manifest: &Manifest) -> Vec<Vec<WaterClass>> {
    let arid0 = vec![vec![0.5; grid.cols()]; grid.rows()];
    let water0 = classify_water(grid, &arid0, manifest.climate.glaciation);
    let dist = continentality_steps(grid, &water0);
    let cell_m = 2.0 * std::f64::consts::PI * manifest.planet.datum_radius_m / grid.cols() as f64;
    let arid: Vec<Vec<f64>> = dist
        .iter()
        .enumerate()
        .map(|(r, row)| {
            row.iter()
                .enumerate()
                .map(|(c, d)| {
                    let continentality = (*d as f64 * cell_m / 2_500_000.0).clamp(0.0, 1.0);
                    let facing = nereid_influence(grid.lons[c]);
                    let _ = grid.lats[r];
                    (0.5 * manifest.climate.aridity + 0.6 * continentality - 0.35 * facing)
                        .clamp(0.0, 1.0)
                })
                .collect()
        })
        .collect();
    classify_water(grid, &arid, manifest.climate.glaciation)
}
/// Consistency report: derived layers must agree with height.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConsistencyReport {
    pub cells: usize,
    pub ocean_cells: usize,
    pub lake_cells: usize,
    pub river_cells: usize,
    pub salt_cells: usize,
    pub ice_cells: usize,
    pub min_height_m: f64,
    pub max_height_m: f64,
    pub biome_histogram: HashMap<String, usize>,
    pub mineral_hotspots: usize,
    pub errors: Vec<String>,
}

pub fn bake_report(manifest: &Manifest, step_deg: f64) -> Result<ConsistencyReport, String> {
    let grid = evaluate_height_grid(manifest, step_deg)?;
    let water = classify_water_driven(&grid, manifest);
    let mut report = ConsistencyReport {
        cells: grid.rows() * grid.cols(),
        ocean_cells: 0,
        lake_cells: 0,
        river_cells: 0,
        salt_cells: 0,
        ice_cells: 0,
        min_height_m: f64::INFINITY,
        max_height_m: f64::NEG_INFINITY,
        biome_histogram: HashMap::new(),
        mineral_hotspots: 0,
        errors: Vec::new(),
    };
    for r in 0..grid.rows() {
        for c in 0..grid.cols() {
            let h = grid.h[r][c];
            report.min_height_m = report.min_height_m.min(h);
            report.max_height_m = report.max_height_m.max(h);
            match water[r][c] {
                WaterClass::Ocean => report.ocean_cells += 1,
                WaterClass::Lake => report.lake_cells += 1,
                WaterClass::River => report.river_cells += 1,
                WaterClass::SaltFlat => report.salt_cells += 1,
                WaterClass::Ice => report.ice_cells += 1,
                WaterClass::Land => {}
            }
            // Water/height agreement: ocean IFF below datum.
            if (water[r][c] == WaterClass::Ocean) != (h < 0.0) {
                report
                    .errors
                    .push(format!("water/height mismatch at ({r},{c})"));
            }
            let site = classify_site_coarse(manifest, grid.lats[r], grid.lons[c], h);
            let key = format!("{:?}", site.biome);
            // Biome labels are legal by construction; re-parse to prove it.
            let canonical = format!("{:?}", site.biome);
            let snake = to_snake(&canonical);
            if parse_biome(&snake).is_err() {
                report
                    .errors
                    .push(format!("illegal biome label {canonical}"));
            }
            *report.biome_histogram.entry(key).or_insert(0) += 1;
            if r % 4 == 0 && c % 4 == 0 {
                let m = sample_minerals(
                    manifest.planet.seed,
                    site.geology,
                    h,
                    0.2,
                    1e9,
                    manifest.climate.aridity,
                    manifest.climate.glaciation,
                    r as i64,
                    c as i64,
                );
                if m.iron > 0.2 || m.rare > 0.2 || m.volatile > 0.2 {
                    report.mineral_hotspots += 1;
                }
                let _ = MineralRichness {
                    iron: 0.0,
                    rare: 0.0,
                    volatile: 0.0,
                };
            }
        }
    }
    // Height must respect manifest range.
    if report.min_height_m < manifest.planet.height_min_m - 1e-6
        || report.max_height_m > manifest.planet.height_max_m + 1e-6
    {
        report.errors.push("height outside manifest range".into());
    }
    // Grayscale round-trip sanity for extremes.
    let g_min = encode_height_01(
        report.min_height_m,
        manifest.planet.height_min_m,
        manifest.planet.height_max_m,
    );
    let g_max = encode_height_01(
        report.max_height_m,
        manifest.planet.height_min_m,
        manifest.planet.height_max_m,
    );
    if !(0.0..=1.0).contains(&g_min) || !(0.0..=1.0).contains(&g_max) {
        report.errors.push("height encoding out of range".into());
    }
    Ok(report)
}

fn to_snake(debug_name: &str) -> String {
    let mut out = String::new();
    for (i, ch) in debug_name.chars().enumerate() {
        if ch.is_ascii_uppercase() {
            if i > 0 {
                out.push('_');
            }
            out.push(ch.to_ascii_lowercase());
        } else {
            out.push(ch);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::features::{Feature, PlacedFeature};

    fn demo_manifest() -> Manifest {
        let toml = include_str!("../../../data/worldgen/example_rocky.toml");
        toml::from_str(toml).expect("example parses")
    }

    #[test]
    fn bake_report_is_consistent() {
        let m = demo_manifest();
        let report = bake_report(&m, 10.0).expect("bake");
        assert!(report.errors.is_empty(), "{:?}", report.errors);
        assert!(report.cells > 0);
        assert!(report.min_height_m >= m.planet.height_min_m - 1e-6);
    }

    #[test]
    fn derived_normal_points_up_on_flat() {
        let m = demo_manifest();
        let grid = evaluate_height_grid(&m, 10.0).expect("grid");
        // Flat ocean far from features: normal ~ +Z.
        let (nx, ny, nz) = derive_normal(&grid, grid.rows() / 2, 0);
        assert!(nz > 0.9, "{nx} {ny} {nz}");
        assert!(nx.is_finite() && ny.is_finite() && nz.is_finite());
    }

    #[test]
    fn landmark_raises_terrain() {
        let mut m = demo_manifest();
        m.features.push(PlacedFeature {
            id: "olympus".into(),
            seed: 1,
            lat_deg: 10.0,
            lon_deg: 20.0,
            rotation_rad: 0.0,
            feature: Feature::ShieldVolcano {
                radius_m: 300_000.0,
                height_m: 9000.0,
                caldera: true,
            },
        });
        let grid = evaluate_height_grid(&m, 5.0).expect("grid");
        let peak = grid
            .h
            .iter()
            .flatten()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(peak > 4000.0, "landmark visible: {peak}");
    }

    #[test]
    fn same_seed_same_bake() {
        let m = demo_manifest();
        let a = bake_report(&m, 10.0).expect("a");
        let b = bake_report(&m, 10.0).expect("b");
        assert_eq!(a.ocean_cells, b.ocean_cells);
        assert_eq!(a.biome_histogram, b.biome_histogram);
    }
}

#[cfg(test)]
mod landmark_tests {
    use super::*;
    use crate::features::{Feature, PlacedFeature};

    #[test]
    fn landmark_overrides_plain_biome() {
        let toml = include_str!("../../../data/worldgen/example_rocky.toml");
        let mut manifest: Manifest = toml::from_str(toml).expect("parse");
        manifest.features.push(PlacedFeature {
            id: "big-eye".into(),
            seed: 1,
            lat_deg: 0.0,
            lon_deg: 0.0,
            rotation_rad: 0.0,
            feature: Feature::ImpactBasin {
                radius_m: 500_000.0,
                depth_m: 4000.0,
                rings: 2,
            },
        });
        let at = classify_site_coarse(&manifest, 0.0, 0.0, 500.0);
        assert_eq!(at.biome, Biome::ImpactBasin);
        let far = classify_site_coarse(&manifest, 0.0, 60.0, 500.0);
        assert_ne!(far.biome, Biome::ImpactBasin);
    }
}
