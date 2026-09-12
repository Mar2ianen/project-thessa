//! One canonical deterministic global planetary terrain field.
//!
//! ```text
//! TerrainSample sample(direction_on_sphere, min_wavelength_m)
//! ```
//!
//! Tiles/chunks are only consumers/cache units: the same physical point
//! always yields the same terrain regardless of tile ID, camera, LOD,
//! sampling order or raster resolution. Coarse context grids (ocean,
//! moisture, geothermal normalization) are caches, not the terrain itself.

use serde::{Deserialize, Serialize};

use crate::{
    biomes::{Biome, FeatureTag, Geology, SiteClass},
    climate::{continentality_metres, drivers_at, nereid_influence},
    geothermal::{GeothermalProvince, geothermal_activity},
    hydro::{HeightGrid, WaterClass, classify_water},
    landmarks::LandmarkZone,
    sphere::{dir_from_latlon, latlon_from_dir},
    tectonics::{Boundary, eval_uplift_m},
    terrain::{
        MESO_BANDS_M, TerrainKnobs, eval_band_m, eval_macro_m, eval_macro_provinces_m, meso_amp,
    },
};

/// Physical planet + recipe parameters (no raster anywhere).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlanetParams {
    pub name: String,
    pub seed: u64,
    pub radius_m: f64,
    pub height_min_m: f64,
    pub height_max_m: f64,
    pub knobs: KnobsSerde,
    pub macro_strength: f64,
    pub polar_extent: f64,
    pub aridity: f64,
    pub glaciation: f64,
    pub volcanism: f64,
    pub nereid_ocean_bias: f64,
    pub anti_nereid_land_bias: f64,
    pub eclipse_strength: f64,
    /// Nominal global mean tidal flux, W/m2 (normalization target).
    pub nominal_flux_w_m2: f64,
    pub ocean_target: Option<f64>,
}

/// Serde-friendly knobs mirror.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct KnobsSerde {
    pub mountain_coverage: f64,
    pub crater_density: f64,
    pub volcanism: f64,
    pub erosion: f64,
    pub canyon_strength: f64,
    pub base_roughness_m: f64,
}

impl From<KnobsSerde> for TerrainKnobs {
    fn from(k: KnobsSerde) -> Self {
        Self {
            mountain_coverage: k.mountain_coverage,
            crater_density: k.crater_density,
            volcanism: k.volcanism,
            erosion: k.erosion,
            canyon_strength: k.canyon_strength,
            base_roughness_m: k.base_roughness_m,
        }
    }
}

/// Full semantic sample for renderer/runtime consumers.
#[derive(Debug, Clone)]
pub struct TerrainSample {
    pub height_m: f64,
    pub macro_height_m: f64,
    pub procedural_height_m: f64,
    pub biome: Biome,
    pub geology: Geology,
    pub tag0: Option<FeatureTag>,
    pub tag1: Option<FeatureTag>,
    /// Slope magnitude estimate (rise/run) at the requested wavelength.
    pub slope_hint: f64,
    /// Normalized geothermal flux, W/m2.
    pub geothermal_flux_w_m2: f64,
    pub moisture01: f64,
    pub temperature_k: f64,
    pub continentality01: f64,
    pub eclipse_exposure01: f64,
}

/// The global field. Build once, sample anywhere.
pub struct PlanetField {
    pub params: PlanetParams,
    pub features: Vec<crate::features::PlacedFeature>,
    pub tectonics: Vec<Boundary>,
    pub provinces: Vec<GeothermalProvince>,
    pub landmarks: Vec<LandmarkZone>,
    geo_norm: f64,
    context: ContextGrid,
    /// Datum calibration belongs to the canonical field, not just a preview.
    pub sea_offset_m: f64,
}

#[derive(Default)]
struct ContextGrid {
    lats: Vec<f64>,
    lons: Vec<f64>,
    water: Vec<Vec<WaterClass>>,
    arid: Vec<Vec<f64>>,
    ocean_dist_m: Vec<Vec<f64>>,
}

impl PlanetField {
    /// Build the field, evaluating the coarse context cache (2 deg).
    /// Erosion-corrected macro heights seed the context; fine detail stays analytic.
    pub fn build(
        params: PlanetParams,
        features: Vec<crate::features::PlacedFeature>,
        tectonics: Vec<Boundary>,
        provinces: Vec<GeothermalProvince>,
        landmarks: Vec<LandmarkZone>,
    ) -> Result<Self, String> {
        validate_params(&params)?;
        let geo_norm = normalize_geothermal(&provinces, params.nominal_flux_w_m2, params.radius_m);
        let mut field = Self {
            params,
            features,
            tectonics,
            provinces,
            landmarks,
            geo_norm,
            context: ContextGrid::default(),
            sea_offset_m: 0.0,
        };
        let (context, offset) = build_context(&field);
        field.context = context;
        field.sea_offset_m = offset;
        Ok(field)
    }

    /// Canonical sample. Pure function of (direction, min_wavelength).
    /// Only bands with wavelength >= min_wavelength_m contribute, so a
    /// coarse sample is always the low-frequency prefix of a finer one.
    pub fn sample(&self, dir: [f64; 3], min_wavelength_m: f64) -> TerrainSample {
        self.sample_impl(dir, min_wavelength_m, true)
    }

    /// Material sampling when the consumer already calculates a mesh/texture normal.
    pub fn sample_surface(&self, dir: [f64; 3], min_wavelength_m: f64) -> TerrainSample {
        self.sample_impl(dir, min_wavelength_m, false)
    }

    fn sample_impl(&self, dir: [f64; 3], min_wavelength_m: f64, slope: bool) -> TerrainSample {
        let (height, macro_h, detail_h) = self.height_parts(dir, min_wavelength_m);
        let (lat, lon) = latlon_from_dir(dir);
        // Context lookup (nearest coarse cell).
        let (_, _, ocean_dist) = self.context_at(lat, lon);
        let geo_w = geothermal_activity(&self.provinces, lat, lon, self.params.radius_m, 0.0, 0.0);
        let flux = geo_w * self.geo_norm;
        let drivers = drivers_at(
            lat,
            lon,
            height,
            ocean_dist,
            self.params.eclipse_strength,
            geo_w.min(1.0),
        );
        let site = classify_with_context(&self.params, &self.features, lat, lon, height, flux);
        // Slope hint from analytic neighbours at the requested wavelength.
        let slope_hint = if slope {
            self.slope_hint(dir, min_wavelength_m.max(1.0))
        } else {
            0.0
        };
        let regional = crate::rng::fbm3(
            self.params.seed,
            770,
            dir[0] * 6.0,
            dir[1] * 6.0,
            dir[2] * 6.0,
            3,
        );
        let moisture = (drivers.moisture * 0.65 + 0.15 + regional * 0.45
            - self.params.aridity * 0.16)
            .clamp(0.0, 1.0);
        let temperature_k = 294.0
            - 38.0 * dir[1].powi(2)
            - height.max(0.0) * 0.005
            - (1.0 - drivers.eclipse_exposure) * 12.0
            + regional * 3.0;
        TerrainSample {
            height_m: height,
            macro_height_m: macro_h,
            procedural_height_m: detail_h,
            biome: site.biome,
            geology: site.geology,
            tag0: site.tag0,
            tag1: site.tag1,
            slope_hint,
            geothermal_flux_w_m2: flux,
            moisture01: moisture,
            temperature_k,
            continentality01: drivers.continentality,
            eclipse_exposure01: drivers.eclipse_exposure,
        }
    }

    /// Allocation-free canonical height for mesh vertices and contact queries.
    pub fn height_m(&self, dir: [f64; 3], min_wavelength_m: f64) -> f64 {
        self.height_parts(dir, min_wavelength_m).0
    }

    fn height_parts(&self, dir: [f64; 3], min_wavelength_m: f64) -> (f64, f64, f64) {
        let min_wl = min_wavelength_m.max(1.0);
        let (lat, lon) = latlon_from_dir(dir);
        let knobs: TerrainKnobs = self.params.knobs.into();
        let strength = self.params.macro_strength;
        let macro_h = (eval_macro_m(&self.features, lat, lon, self.params.radius_m)
            + eval_macro_provinces_m(self.params.seed, knobs, dir, self.params.radius_m)
            + eval_uplift_m(&self.tectonics, lat, lon, self.params.radius_m)
            + hemi_bias(&self.params, lon))
            * strength;
        let meso_h: f64 = MESO_BANDS_M
            .iter()
            .enumerate()
            .filter(|(_, wl)| **wl >= min_wl)
            .map(|(i, wl)| {
                eval_band_m(
                    self.params.seed,
                    64 + i,
                    dir,
                    self.params.radius_m,
                    *wl,
                    meso_amp(*wl, knobs),
                )
            })
            .sum();
        let micro_h = crate::terrain::eval_micro_dir_m(
            self.params.seed,
            knobs,
            dir,
            self.params.radius_m,
            min_wl,
        );
        let mountain_mask = crate::appearance::smooth(800.0, 3000.0, macro_h);
        let mountain_detail: f64 = [(8000.0, 1100.0), (4000.0, 450.0)]
            .into_iter()
            .filter(|(wl, _)| *wl >= min_wl)
            .enumerate()
            .map(|(i, (wl, amp))| {
                let p = dir.map(|v| v * self.params.radius_m / wl);
                let n =
                    crate::rng::value_noise3(self.params.seed, 1900 + i as u32, p[0], p[1], p[2]);
                ((1.0 - n.abs()).powi(3) - 0.4) * amp * mountain_mask
            })
            .sum();
        let meso_h = meso_h + mountain_detail;
        let mut height = macro_h + meso_h + micro_h - self.sea_offset_m;
        // Authored landmark overrides (delta-based, datum-robust).
        for zone in &self.landmarks {
            height += zone.height_delta(dir, self.params.radius_m, &|d| {
                self.base_height(d)
                    + crate::terrain::eval_micro_dir_m(
                        self.params.seed,
                        knobs,
                        d,
                        self.params.radius_m,
                        250.0,
                    )
            });
        }
        height = height.clamp(self.params.height_min_m, self.params.height_max_m);
        (height, macro_h - self.sea_offset_m, meso_h + micro_h)
    }

    /// Base height without landmarks (used by landmark blending).
    fn base_height(&self, dir: [f64; 3]) -> f64 {
        let (lat, lon) = latlon_from_dir(dir);
        let knobs: TerrainKnobs = self.params.knobs.into();
        (eval_macro_m(&self.features, lat, lon, self.params.radius_m)
            + eval_macro_provinces_m(self.params.seed, knobs, dir, self.params.radius_m)
            + eval_uplift_m(&self.tectonics, lat, lon, self.params.radius_m)
            + hemi_bias(&self.params, lon))
            * self.params.macro_strength
            + crate::terrain::eval_meso_m(self.params.seed, knobs, lat, lon, self.params.radius_m)
    }

    fn slope_hint(&self, dir: [f64; 3], wavelength_m: f64) -> f64 {
        // Offset along two tangent axes by half a wavelength.
        let (lat, lon) = latlon_from_dir(dir);
        let d_deg = (wavelength_m * 0.5 / self.params.radius_m).to_degrees();
        let h0 = self.base_height(dir);
        let hx = self.base_height(dir_from_latlon(lat, lon + d_deg));
        let hy = self.base_height(dir_from_latlon((lat + d_deg).clamp(-90.0, 90.0), lon));
        let dx = (wavelength_m * 0.5).max(1.0);
        (((hx - h0) / dx).powi(2) + ((hy - h0) / dx).powi(2)).sqrt()
    }

    fn context_at(&self, lat: f64, lon: f64) -> (WaterClass, f64, f64) {
        let r = regular_index(lat, -90.0, 2.0, self.context.lats.len(), false);
        let c = regular_index(lon, -180.0, 2.0, self.context.lons.len(), true);
        (
            self.context.water[r][c],
            self.context.arid[r][c],
            self.context.ocean_dist_m[r][c],
        )
    }
}

// The context is a regular 2 degree grid. Lower cell wins exact ties,
// matching the former scan, with longitude periodic at the seam.
fn regular_index(value: f64, start: f64, step: f64, count: usize, wrap: bool) -> usize {
    let position = if wrap {
        (value - start).rem_euclid(step * count as f64) / step
    } else {
        (value - start) / step
    };
    let nearest = (position - 0.5).ceil();
    if wrap {
        (nearest as i64).rem_euclid(count as i64) as usize
    } else {
        nearest.clamp(0.0, (count - 1) as f64) as usize
    }
}

fn hemi_bias(params: &PlanetParams, lon_deg: f64) -> f64 {
    let facing = nereid_influence(lon_deg);
    -4500.0 * params.nereid_ocean_bias * facing
        + 3600.0 * params.anti_nereid_land_bias * (1.0 - facing)
}

fn validate_params(params: &PlanetParams) -> Result<(), String> {
    if params
        .ocean_target
        .is_some_and(|t| !t.is_finite() || !(0.0..=1.0).contains(&t))
    {
        return Err("ocean_target must be finite and within 0..=1".into());
    }
    if params.name.trim().is_empty() {
        return Err("planet name must not be empty".into());
    }
    if !params.radius_m.is_finite() || params.radius_m <= 0.0 {
        return Err("radius_m must be positive".into());
    }
    if !params.height_min_m.is_finite()
        || !params.height_max_m.is_finite()
        || params.height_min_m.partial_cmp(&params.height_max_m) != Some(std::cmp::Ordering::Less)
    {
        return Err("need height_min_m < height_max_m".into());
    }
    if !params.nominal_flux_w_m2.is_finite() || params.nominal_flux_w_m2 < 0.0 {
        return Err("nominal_flux_w_m2 must be finite non-negative".into());
    }
    Ok(())
}

/// Build the coarse context cache: macro+meso heights, water, aridity,
/// ocean distance. Deterministic; resolution fixed so caches never leak
/// into analytic results.
fn build_context(field: &PlanetField) -> (ContextGrid, f64) {
    let params = &field.params;
    let step = 2.0;
    let mut lats = Vec::new();
    let mut lat = -90.0;
    while lat <= 90.0 {
        lats.push(lat);
        lat += step;
    }
    let mut lons = Vec::new();
    let mut lon = -180.0;
    while lon < 180.0 {
        lons.push(lon);
        lon += step;
    }
    let mut grid = HeightGrid::new(lats.clone(), lons.clone(), params.radius_m);
    for (r, la) in lats.iter().enumerate() {
        for (c, lo) in lons.iter().enumerate() {
            // Same height evaluator as every consumer, before datum calibration.
            grid.h[r][c] = field.height_m(dir_from_latlon(*la, *lo), 1.0);
        }
    }
    let sea_offset_m = params
        .ocean_target
        .map(|target| {
            let mut samples: Vec<_> = grid
                .h
                .iter()
                .enumerate()
                .flat_map(|(r, row)| {
                    let weight = lats[r].to_radians().cos().max(0.0);
                    row.iter().map(move |height| (*height, weight))
                })
                .collect();
            samples.sort_by(|a, b| a.0.total_cmp(&b.0));
            let target_weight = target * samples.iter().map(|s| s.1).sum::<f64>();
            let mut accumulated = 0.0;
            let mut datum = 0.0;
            for (height, weight) in samples {
                accumulated += weight;
                datum = height;
                if accumulated >= target_weight {
                    break;
                }
            }
            datum
        })
        .unwrap_or(0.0);
    for row in &mut grid.h {
        for height in row {
            *height -= sea_offset_m;
        }
    }
    let arid0 = vec![vec![0.5; grid.cols()]; grid.rows()];
    let water0 = classify_water(&grid, &arid0, params.glaciation);
    let dist_m = continentality_metres(&grid, &water0);
    let mut arid = vec![vec![0.0; grid.cols()]; grid.rows()];
    let mut ocean_dist_m = vec![vec![0.0; grid.cols()]; grid.rows()];
    for (r, row) in dist_m.iter().enumerate() {
        for (c, d) in row.iter().enumerate() {
            let cont = (d / 2_500_000.0).clamp(0.0, 1.0);
            let facing = nereid_influence(grid.lons[c]);
            arid[r][c] = (0.5 * params.aridity + 0.6 * cont - 0.35 * facing).clamp(0.0, 1.0);
            ocean_dist_m[r][c] = *d;
        }
    }
    let water = classify_water(&grid, &arid, params.glaciation);
    (
        ContextGrid {
            lats,
            lons,
            water,
            arid,
            ocean_dist_m,
        },
        sea_offset_m,
    )
}

/// Normalize the geothermal weight field so its spherical area mean equals
/// the nominal flux. Fibonacci-sphere area sampling, deterministic.
fn normalize_geothermal(provinces: &[GeothermalProvince], nominal_flux: f64, radius_m: f64) -> f64 {
    const SAMPLES: usize = 4096;
    let mut sum = 0.0;
    for i in 0..SAMPLES {
        // Fibonacci lattice: equal-area, deterministic, no RNG.
        let y = 1.0 - 2.0 * (i as f64 + 0.5) / SAMPLES as f64;
        let r = (1.0 - y * y).max(0.0).sqrt();
        let phi = i as f64 * std::f64::consts::PI * (3.0 - 5.0_f64.sqrt());
        let dir = [r * phi.cos(), y, r * phi.sin()];
        let (lat, lon) = latlon_from_dir(dir);
        sum += geothermal_activity(provinces, lat, lon, radius_m, 0.0, 0.0);
    }
    let mean_weight = sum / SAMPLES as f64;
    if mean_weight > 1e-12 {
        nominal_flux / mean_weight
    } else {
        0.0
    }
}

/// Site classification shared by field sampling (context-aware).
pub fn classify_with_context(
    params: &PlanetParams,
    features: &[crate::features::PlacedFeature],
    lat: f64,
    lon: f64,
    height_m: f64,
    geo_flux: f64,
) -> SiteClass {
    // Reuse the bake classifier semantics via a lightweight manifest mirror.
    // NOTE: kept in sync with bake::classify_site_coarse by construction test.
    crate::bake::classify_for_field(
        lat,
        lon,
        height_m,
        geo_flux,
        params.polar_extent,
        params.glaciation,
        params.volcanism,
        params.aridity,
        features,
        params.radius_m,
    )
}

/// Build a [`PlanetField`] from a dev manifest (provinces defaulted).
pub fn field_from_manifest(manifest: &crate::manifest::Manifest) -> Result<PlanetField, String> {
    let params = PlanetParams {
        name: manifest.planet.name.clone(),
        seed: manifest.planet.seed,
        radius_m: manifest.planet.datum_radius_m,
        height_min_m: manifest.planet.height_min_m,
        height_max_m: manifest.planet.height_max_m,
        knobs: KnobsSerde {
            mountain_coverage: manifest.terrain.mountain_coverage,
            crater_density: manifest.terrain.crater_density,
            volcanism: manifest.terrain.volcanism,
            erosion: manifest.terrain.erosion,
            canyon_strength: manifest.terrain.canyon_strength,
            base_roughness_m: 50.0 + 300.0 * manifest.terrain.roughness,
        },
        macro_strength: manifest.readability.macro_feature_strength,
        polar_extent: manifest.climate.polar_extent,
        aridity: manifest.climate.aridity,
        glaciation: manifest.climate.glaciation,
        volcanism: manifest.terrain.volcanism,
        nereid_ocean_bias: manifest.climate.nereid_ocean_bias,
        anti_nereid_land_bias: manifest.climate.anti_nereid_land_bias,
        eclipse_strength: 0.28,
        nominal_flux_w_m2: 0.146,
        ocean_target: manifest.ocean_target,
    };
    let hot_spots: Vec<(f64, f64)> = manifest
        .features
        .iter()
        .filter_map(|f| {
            use crate::features::Feature;
            match &f.feature {
                Feature::VolcanicProvince { .. }
                | Feature::ShieldVolcano { .. }
                | Feature::Canyon { .. } => Some((f.lat_deg, f.lon_deg)),
                _ => None,
            }
        })
        .collect();
    let provinces = crate::geothermal::default_provinces(manifest.planet.seed, &hot_spots);
    PlanetField::build(
        params,
        manifest.features.clone(),
        manifest.tectonics.clone(),
        provinces,
        manifest.landmark_zones.clone(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        landmarks::{LandmarkMode, LandmarkZone},
        sphere::dir_from_latlon,
        terrain::{MESO_BANDS_M, MICRO_BANDS_M, eval_band_m, micro_amp},
    };

    fn test_params() -> PlanetParams {
        PlanetParams {
            name: "test".into(),
            seed: 77,
            radius_m: 3_200_000.0,
            height_min_m: -8000.0,
            height_max_m: 12000.0,
            knobs: KnobsSerde {
                mountain_coverage: 0.3,
                crater_density: 0.3,
                volcanism: 0.2,
                erosion: 0.4,
                canyon_strength: 0.3,
                base_roughness_m: 150.0,
            },
            macro_strength: 1.0,
            polar_extent: 0.15,
            aridity: 0.4,
            glaciation: 0.2,
            volcanism: 0.2,
            nereid_ocean_bias: 0.0,
            anti_nereid_land_bias: 0.0,
            eclipse_strength: 0.28,
            nominal_flux_w_m2: 0.146,
            ocean_target: None,
        }
    }

    fn test_field() -> PlanetField {
        PlanetField::build(test_params(), vec![], vec![], vec![], vec![]).expect("field builds")
    }

    #[test]
    fn same_point_same_sample() {
        let field = test_field();
        let dir = dir_from_latlon(12.0, -40.0);
        let a = field.sample(dir, 100.0);
        let b = field.sample(dir, 100.0);
        assert_eq!(a.height_m, b.height_m);
        assert_eq!(a.biome, b.biome);
        assert!(a.height_m.is_finite() && a.slope_hint.is_finite());
    }

    #[test]
    fn separate_builds_agree_tile_independence() {
        // No tile/chunk identity enters sampling: two builds agree exactly.
        let a = test_field();
        let b = test_field();
        let dir = dir_from_latlon(-33.0, 77.0);
        assert_eq!(a.sample(dir, 50.0).height_m, b.sample(dir, 50.0).height_m);
    }

    #[test]
    fn seam_identical_pm180() {
        let field = test_field();
        let a = field.sample(dir_from_latlon(10.0, 180.0), 100.0);
        let b = field.sample(dir_from_latlon(10.0, -180.0), 100.0);
        assert_eq!(a.height_m, b.height_m);
        assert_eq!(a.biome, b.biome);
    }

    #[test]
    fn pole_invariant_over_longitude() {
        let field = test_field();
        let p0 = field.sample(dir_from_latlon(90.0, 0.0), 100.0);
        for lon in [-150.0, -45.0, 60.0, 179.0] {
            let p = field.sample(dir_from_latlon(90.0, lon), 100.0);
            assert_eq!(p0.height_m, p.height_m, "lon {lon}");
        }
    }

    #[test]
    fn coarse_lod_is_prefix_of_fine_lod() {
        // sample(64000) must equal the manual low-frequency sum; the finer
        // sample only ADDS shorter bands.
        let field = test_field();
        let dir = dir_from_latlon(5.0, 5.0);
        let knobs: TerrainKnobs = test_params().knobs.into();
        let coarse = field.sample(dir, 64_000.0);
        let mut expected = 0.0;
        for (i, wl) in MESO_BANDS_M.iter().enumerate() {
            if *wl >= 64_000.0 {
                expected += eval_band_m(
                    77,
                    64 + i,
                    dir,
                    3_200_000.0,
                    *wl,
                    crate::terrain::meso_amp(*wl, knobs),
                );
            }
        }
        for (i, wl) in MICRO_BANDS_M.iter().enumerate() {
            if *wl >= 64_000.0 {
                expected += eval_band_m(77, 128 + i, dir, 3_200_000.0, *wl, micro_amp(*wl, knobs));
            }
        }
        // Macro part: features(0) + provinces + uplift(0) + bias(0).
        let provinces = crate::terrain::eval_macro_provinces_m(77, knobs, dir, 3_200_000.0);
        assert!((coarse.procedural_height_m - expected).abs() < 1e-9);
        assert!((coarse.macro_height_m - provinces).abs() < 1e-9);
        let fine = field.sample(dir, 100.0);
        assert!(
            (fine.height_m - coarse.height_m).abs() > 1e-9,
            "finer LOD adds detail"
        );
        assert!(fine.height_m.is_finite());
    }

    #[test]
    fn geothermal_mean_matches_nominal_flux() {
        let provinces = crate::geothermal::place_provinces(7, 3, 6, &[], &[]);
        let params = test_params();
        let field = PlanetField::build(params, vec![], vec![], provinces, vec![]).expect("build");
        // Area-weighted (Fibonacci) mean over the sphere.
        const N: usize = 2048;
        let mut sum = 0.0;
        for i in 0..N {
            let y = 1.0 - 2.0 * (i as f64 + 0.5) / N as f64;
            let r = (1.0 - y * y).max(0.0).sqrt();
            let phi = i as f64 * std::f64::consts::PI * (3.0 - 5.0_f64.sqrt());
            let s = field.sample([r * phi.cos(), y, r * phi.sin()], 1000.0);
            sum += s.geothermal_flux_w_m2;
            assert!(s.geothermal_flux_w_m2.is_finite() && s.geothermal_flux_w_m2 >= 0.0);
        }
        let mean = sum / N as f64;
        assert!((mean - 0.146).abs() / 0.146 < 0.10, "mean flux {mean}");
    }

    #[test]
    fn landmark_center_is_authoritative() {
        let zone = LandmarkZone {
            id: "dome".into(),
            center_lat_deg: 10.0,
            center_lon_deg: 20.0,
            radius_m: 60_000.0,
            blend_width_m: 20_000.0,
            mode: LandmarkMode::ProceduralOverride,
            asset: None,
            surface_tags: vec![],
            descriptor: None,
            amplitude_m: 800.0,
        };
        let params = test_params();
        let plain =
            PlanetField::build(params.clone(), vec![], vec![], vec![], vec![]).expect("plain");
        let zoned = PlanetField::build(params, vec![], vec![], vec![], vec![zone]).expect("zoned");
        let dir = dir_from_latlon(10.0, 20.0);
        let a = plain.sample(dir, 100.0).height_m;
        let b = zoned.sample(dir, 100.0).height_m;
        // Dome peak adds exactly amplitude_m at the center.
        assert!((b - a - 800.0).abs() < 1e-6, "{a} vs {b}");
    }

    #[test]
    fn sample_semantics_in_range() {
        let field = test_field();
        let s = field.sample(dir_from_latlon(-20.0, 40.0), 250.0);
        assert!((0.0..=1.0).contains(&s.moisture01));
        assert!((0.0..=1.0).contains(&s.continentality01));
        assert!((0.0..=1.0).contains(&s.eclipse_exposure01));
        assert!(s.slope_hint.is_finite() && s.slope_hint >= 0.0);
    }
    #[test]
    fn ocean_datum_is_shared_by_height_surface_and_full_samples() {
        let mut params = test_params();
        params.ocean_target = Some(0.6);
        let f = PlanetField::build(params, vec![], vec![], vec![], vec![]).unwrap();
        let mut ocean = 0;
        for i in 0..4096 {
            let y = 1.0 - 2.0 * (i as f64 + 0.5) / 4096.0;
            let angle = i as f64 * 2.399963229728653;
            let r = (1.0 - y * y).sqrt();
            let dir = [r * angle.cos(), y, r * angle.sin()];
            let h = f.height_m(dir, 1.0);
            assert_eq!(h, f.sample_surface(dir, 1.0).height_m);
            if i % 64 == 0 {
                assert_eq!(h, f.sample(dir, 1.0).height_m);
            }
            ocean += usize::from(h < 0.0);
        }
        let fraction = ocean as f64 / 4096.0;
        assert!((fraction - 0.6).abs() < 0.025, "ocean area {fraction}");
    }
    #[test]
    fn context_arithmetic_wraps_seam_and_clamps_poles() {
        assert_eq!(regular_index(180.0, -180.0, 2.0, 180, true), 0);
        assert_eq!(regular_index(-540.0, -180.0, 2.0, 180, true), 0);
        assert_eq!(regular_index(90.0, -90.0, 2.0, 91, false), 90);
        assert_eq!(regular_index(-90.0, -90.0, 2.0, 91, false), 0);
        for n in 0..180 {
            assert_eq!(
                regular_index(-180.0 + n as f64 * 2.0, -180.0, 2.0, 180, true),
                n
            );
        }
    }
}
