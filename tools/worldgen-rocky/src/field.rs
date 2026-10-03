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
    hydro::{HeightGrid, WaterClass},
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
    #[serde(default)]
    pub hydrology: crate::hydro::HydrologyRecipe,
    #[serde(default)]
    pub surface_climate: crate::climate::SurfaceClimate,
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
    /// Tangent-plane height Laplacian, 1/m; positive in hollows, negative on
    /// crests. Zero when the consumer has not requested a geometry stencil.
    pub curvature_per_m: f64,
    /// Actual regional final-minus-source displacement, not a noise mask.
    pub erosion_displacement_m: f64,
    /// Positive fine-region final-minus-source displacement; a deposit proxy,
    /// not the unrelated coarse DEM's net displacement or a snow inventory.
    pub local_deposition_m: f64,
    /// Normalized geothermal flux, W/m2.
    pub geothermal_flux_w_m2: f64,
    pub moisture01: f64,
    /// Terrain-derived regional saturation suitability, not local water depth.
    pub wetland_potential01: f64,
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
    /// Derived inhabited-region descriptors; no buildings or city simulation.
    pub inhabited_regions: Vec<crate::civilization::InhabitedRegion>,
    geo_norm: f64,
    context: ContextGrid,
    /// Datum calibration belongs to the canonical field, not just a preview.
    pub sea_offset_m: f64,
    /// Static area-mean calibration of the recipe climate proxy only.
    temperature_offset_k: f64,
    /// Offline erosion delta only; fine spectral relief remains analytic.
    frozen_erosion: Option<FrozenErosion>,
    civilization: Option<crate::civilization::CivilizationRecipe>,
}

struct FrozenErosion {
    step_deg: f64,
    delta_m: Vec<Vec<f64>>,
    regional: Vec<crate::offline::RegionalErosion>,
}

#[derive(Default)]
struct ContextGrid {
    lats: Vec<f64>,
    lons: Vec<f64>,
    ocean_dist_m: Vec<Vec<f64>>,
    moisture01: Vec<Vec<f64>>,
    salt_spill_m: Vec<Vec<Option<f64>>>,
    hydrology_step_deg: f64,
    wetland_support: Vec<Vec<f64>>,
    rivers: Vec<crate::hydro::RiverReach>,
    lakes: Vec<crate::hydro::LakeBasin>,
}

impl PlanetField {
    pub fn has_frozen_erosion(&self) -> bool {
        self.frozen_erosion.is_some()
    }

    /// Stable source identity. A frozen erosion bake may not cross recipes,
    /// seeds, body geometry, landmarks or calibrated datum/climate contracts.
    pub fn source_signature(&self) -> Result<String, String> {
        serde_json::to_string(&serde_json::json!({
            "contract": "thessa-analytic-source-v1",
            "params": self.params, "features": self.features,
            "tectonics": self.tectonics, "provinces": self.provinces,
            "landmarks": self.landmarks, "sea_offset_m": self.sea_offset_m,
            "temperature_offset_k": self.temperature_offset_k,
        }))
        .map_err(|e| e.to_string())
    }

    /// Consume an existing offline bake without executing erosion at runtime.
    /// Only its final-minus-source displacement is applied, so metre-scale
    /// analytic detail and native mesh/contact sampling keep the same path.
    pub fn with_frozen_erosion(
        mut self,
        world: &crate::offline::FrozenSurface,
    ) -> Result<Self, String> {
        world.validate()?;
        if self.has_frozen_erosion() {
            return Err("frozen erosion is already attached".into());
        }
        let source = world
            .source
            .as_ref()
            .ok_or("historical surface has no erosion source reference")?;
        // Cargo feature unification can enable serde_json/preserve_order in
        // the client while the offline CLI uses sorted maps. Object ordering
        // and whitespace are not source semantics; numbers/arrays still match
        // exactly, without tolerating a changed recipe or calibrated datum.
        let baked_signature: serde_json::Value = serde_json::from_str(&source.signature)
            .map_err(|e| format!("invalid frozen source identity: {e}"))?;
        let live_signature: serde_json::Value = serde_json::from_str(&self.source_signature()?)
            .map_err(|e| format!("invalid live source identity: {e}"))?;
        if baked_signature != live_signature
            || world.seed != self.params.seed
            || world.radius_m != self.params.radius_m
        {
            return Err("frozen erosion source does not match the canonical field".into());
        }
        let delta_m = world
            .heights_m
            .iter()
            .zip(&source.heights_m)
            .map(|(final_row, original)| {
                final_row
                    .iter()
                    .zip(original)
                    .map(|(h, old)| h - old)
                    .collect()
            })
            .collect();
        self.frozen_erosion = Some(FrozenErosion {
            step_deg: world.recipe.grid_step_deg,
            delta_m,
            regional: world.regional_erosion.clone(),
        });
        let (context, _) = build_context(&self);
        self.context = context;
        // Retain drainage from the actual final offline DEM, not a second,
        // coarser rerouting that would erase smaller basins and tributaries.
        self.context.rivers = world.rivers.clone();
        self.context.lakes = world.lakes.clone();
        self.context.hydrology_step_deg = world.recipe.grid_step_deg;
        self.context.salt_spill_m = vec![vec![None; world.lons.len()]; world.lats.len()];
        for lake in &world.lakes {
            if lake.salt_flat {
                for [r, c] in &lake.cells {
                    self.context.salt_spill_m[*r][*c] = Some(lake.surface_height_m);
                }
            }
        }
        if let Some(recipe) = self.civilization {
            self.inhabited_regions = crate::civilization::derive_regions(&self, recipe)?;
        }
        Ok(self)
    }

    /// Stable macro drainage descriptors shared by all world-layer consumers.
    /// Source spacing is reported by `hydrology_grid_step_deg`; these edges
    /// are not detailed river channels or water surfaces.
    pub fn rivers(&self) -> &[crate::hydro::RiverReach] {
        &self.context.rivers
    }

    /// Connected regional lake/salt basins; not metre-scale shoreline geometry.
    pub fn lake_basins(&self) -> &[crate::hydro::LakeBasin] {
        &self.context.lakes
    }

    pub fn hydrology_grid_step_deg(&self) -> f64 {
        self.context.hydrology_step_deg
    }

    pub fn regional_erosion(&self) -> &[crate::offline::RegionalErosion] {
        self.frozen_erosion
            .as_ref()
            .map_or(&[], |erosion| erosion.regional.as_slice())
    }

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
            inhabited_regions: Vec::new(),
            geo_norm,
            context: ContextGrid::default(),
            sea_offset_m: 0.0,
            temperature_offset_k: 0.0,
            frozen_erosion: None,
            civilization: None,
        };
        let (context, offset) = build_context(&field);
        field.context = context;
        field.sea_offset_m = offset;
        if let Some(target) = field.params.surface_climate.target_mean_temperature_k {
            let mut sum = 0.0;
            const N: usize = 4096;
            for i in 0..N {
                let y = 1.0 - 2.0 * (i as f64 + 0.5) / N as f64;
                let r = (1.0 - y * y).sqrt();
                let angle = i as f64 * 2.399963229728653;
                sum += field
                    .sample_surface([r * angle.cos(), y, r * angle.sin()], 32.0)
                    .temperature_k;
            }
            field.temperature_offset_k = target - sum / N as f64;
            // Balance needs the calibrated temperature, not the provisional
            // field. Preserve the existing datum during this context refresh.
            field.context = build_context(&field).0;
        }
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

    /// Surface sample from a shared [`height_prefix_m`](Self::height_prefix_m):
    /// identical to [`sample_surface`](Self::sample_surface) up to the final
    /// ulp, but the ~2 us macro path runs once per texel instead of twice
    /// (fine sample plus mesh-grid residual).
    pub fn sample_surface_from_prefix(
        &self,
        dir: [f64; 3],
        prefix_m: f64,
        macro_h: f64,
        min_wavelength_m: f64,
    ) -> TerrainSample {
        let min_wl = min_wavelength_m.max(1.0);
        let knobs: TerrainKnobs = self.params.knobs.into();
        let (meso_h, micro_h) = self.detail_parts_m(dir, min_wl, knobs, macro_h);
        let height = prefix_m + meso_h + micro_h - self.sea_offset_m;
        let height = self
            .regional_height(dir, min_wl, height, macro_h)
            .clamp(self.params.height_min_m, self.params.height_max_m);
        self.finish_sample(dir, height, macro_h, meso_h + micro_h, 0.0)
    }

    fn sample_impl(&self, dir: [f64; 3], min_wavelength_m: f64, slope: bool) -> TerrainSample {
        let (height, macro_h, detail_h) = self.height_parts(dir, min_wavelength_m);
        let slope_hint = if slope {
            self.slope_hint(dir, min_wavelength_m.max(1.0))
        } else {
            0.0
        };
        self.finish_sample(dir, height, macro_h, detail_h, slope_hint)
    }

    /// Shared sample tail: classification, climate drivers and material
    /// state from a resolved height. Bitwise identical to the legacy
    /// monolith for identical inputs.
    fn finish_sample(
        &self,
        dir: [f64; 3],
        height: f64,
        macro_h: f64,
        detail_h: f64,
        slope_hint: f64,
    ) -> TerrainSample {
        let (lat, lon) = latlon_from_dir(dir);
        // Interpolate continuous drivers, never discrete biome IDs.
        let ocean_dist = if height < 0.0 {
            0.0
        } else {
            self.context_scalar_at(&self.context.ocean_dist_m, lat, lon)
        };
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
        let mut site = classify_with_context(&self.params, &self.features, lat, lon, height, flux);
        let moisture = if height < 0.0 {
            1.0
        } else {
            self.context_scalar_at(&self.context.moisture01, lat, lon)
        };
        let temperature_k = self.temperature_for_height(dir, height, flux);
        // Bilinear regional support avoids painting nearest-cell rectangles.
        // Ocean and highland samples cannot become wetlands through interpolation.
        let wetland_potential01 = if height < 0.0 {
            0.0
        } else {
            self.context_wetland_at(lat, lon)
                * (1.0 - crate::appearance::smooth(150.0, 400.0, height))
        };
        // Salinity suitability belongs to a dry closed depression, not the
        // circular reach of an authored plateau or a generic high-altitude site.
        let r = regular_index(
            lat,
            -90.0,
            self.context.hydrology_step_deg,
            self.context.salt_spill_m.len(),
            false,
        );
        let c = regular_index(
            lon,
            -180.0,
            self.context.hydrology_step_deg,
            self.context.salt_spill_m[0].len(),
            true,
        );
        if height >= 0.0 && self.context.salt_spill_m[r][c].is_some_and(|spill| height < spill) {
            site = SiteClass::new(Biome::SaltFlat, Geology::Evaporite);
        }
        site = crate::ecology::classify_ecological_site(
            site,
            height,
            temperature_k,
            moisture,
            wetland_potential01,
            drivers.continentality,
        );
        TerrainSample {
            height_m: height,
            macro_height_m: macro_h,
            procedural_height_m: detail_h,
            biome: site.biome,
            geology: site.geology,
            tag0: site.tag0,
            tag1: site.tag1,
            slope_hint,
            curvature_per_m: 0.0,
            erosion_displacement_m: self.erosion_delta_at(lat, lon)
                + self
                    .regional_erosion()
                    .iter()
                    .map(|p| p.displacement_at(lat, lon))
                    .sum::<f64>(),
            local_deposition_m: self
                .regional_erosion()
                .iter()
                .map(|p| p.displacement_at(lat, lon).max(0.0))
                .sum(),
            geothermal_flux_w_m2: flux,
            moisture01: moisture,
            wetland_potential01,
            temperature_k,
            continentality01: drivers.continentality,
            eclipse_exposure01: drivers.eclipse_exposure,
        }
    }

    /// Shared calibrated climate proxy at a supplied (possibly eroded) height.
    pub(crate) fn temperature_for_height(&self, dir: [f64; 3], height: f64, flux: f64) -> f64 {
        let (_, lon) = latlon_from_dir(dir);
        let regional = crate::rng::fbm3(
            self.params.seed,
            770,
            dir[0] * 6.0,
            dir[1] * 6.0,
            dir[2] * 6.0,
            3,
        );
        let climate = self.params.surface_climate;
        let mut temperature = 294.0
            - 38.0 * climate.polar_cooling_strength * dir[1].powi(2)
            - height.max(0.0) * 0.005 * climate.elevation_cooling_strength
            - (1.0 - crate::climate::eclipse_exposure(lon, self.params.eclipse_strength)) * 12.0
            + regional * 3.0;
        if climate.geothermal_local_warming_strength > 0.0 {
            const SIGMA: f64 = 5.670374419e-8;
            let warmed = (temperature.powi(4) + flux / SIGMA).sqrt().sqrt();
            temperature += (warmed - temperature) * climate.geothermal_local_warming_strength;
        }
        temperature + self.temperature_offset_k
    }

    /// Allocation-free canonical height for mesh vertices and contact queries.
    pub fn height_m(&self, dir: [f64; 3], min_wavelength_m: f64) -> f64 {
        self.height_parts(dir, min_wavelength_m).0
    }

    /// Fixed physical-scale geometry of the actual canonical surface,
    /// including authored landmarks and attached erosion. Returns rise/run
    /// and the tangent height Laplacian (1/m, positive in concave hollows).
    /// Consumers choose a scale explicitly, never inherit a page's mesh LOD.
    pub fn surface_geometry(&self, dir: [f64; 3], scale_m: f64) -> (f64, f64) {
        let (prefix, macro_h) = self.height_prefix_m(dir);
        self.surface_geometry_from_prefix(dir, prefix, macro_h, scale_m)
    }

    pub(crate) fn surface_geometry_from_prefix(
        &self,
        dir: [f64; 3],
        prefix: f64,
        macro_h: f64,
        scale_m: f64,
    ) -> (f64, f64) {
        let scale_m = scale_m.max(2.0);
        let distance = scale_m * 0.5;
        let (east, north, _) = crate::sphere::enu_basis(dir);
        let (sin, cos) = (distance / self.params.radius_m).sin_cos();
        let offset = |axis: [f64; 3], sign: f64| {
            std::array::from_fn(|i| dir[i] * cos + axis[i] * sin * sign)
        };
        stencil_geometry(
            [
                self.height_from_prefix(dir, prefix, macro_h, scale_m),
                self.height_m(offset(east, 1.0), scale_m),
                self.height_m(offset(east, -1.0), scale_m),
                self.height_m(offset(north, 1.0), scale_m),
                self.height_m(offset(north, -1.0), scale_m),
            ],
            distance,
        )
    }

    /// Cutoff-independent macro term: features, provinces, uplift and
    /// hemispheric bias. The ~2 us bulk of a height evaluation (~15 noise
    /// evals in the province warp alone); texture builders evaluate it once
    /// per texel and share it between the fine sample and the mesh-grid
    /// residual instead of paying twice.
    fn macro_term_m(&self, dir: [f64; 3], lat: f64, lon: f64, knobs: TerrainKnobs) -> f64 {
        (eval_macro_m(&self.features, lat, lon, self.params.radius_m)
            + eval_macro_provinces_m(self.params.seed, knobs, dir, self.params.radius_m)
            + eval_uplift_m(&self.tectonics, lat, lon, self.params.radius_m)
            + hemi_bias(&self.params, lon))
            * self.params.macro_strength
            + self.erosion_delta_at(lat, lon)
    }

    /// Cutoff-dependent detail: meso bands, mountain detail octaves, micro.
    /// Returns `(meso_sum, micro_h)` in the exact combination order the
    /// legacy monolith used, so splits stay bitwise identical.
    fn detail_parts_m(
        &self,
        dir: [f64; 3],
        min_wl: f64,
        knobs: TerrainKnobs,
        macro_h: f64,
    ) -> (f64, f64) {
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
        // Frozen displacement must not alter the amplitudes of the retained
        // analytic fine bands: otherwise applying a delta would add fresh relief.
        let source_macro = if self.has_frozen_erosion() {
            let (lat, lon) = latlon_from_dir(dir);
            macro_h - self.erosion_delta_at(lat, lon)
        } else {
            macro_h
        };
        let mountain_mask = crate::appearance::smooth(800.0, 3000.0, source_macro);
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
        (meso_h + mountain_detail, micro_h)
    }

    /// Landmark deltas: cutoff-independent (blend weight plus dir-only
    /// procedural domes), shared like the macro term.
    fn landmark_sum_m(&self, dir: [f64; 3], knobs: TerrainKnobs) -> f64 {
        let mut sum = 0.0;
        for zone in &self.landmarks {
            sum += zone.height_delta(dir, self.params.radius_m, &|d| {
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
        sum
    }

    /// Shared cutoff-independent prefix: `(macro + landmarks, macro)`.
    /// Texture builders compute it once per texel and feed both the fine
    /// sample and the mesh-grid residual through [`height_from_prefix`](Self::height_from_prefix).
    /// The macro value rides along so the residual never re-evaluates the
    /// ~2 us macro path for its mountain mask.
    pub fn height_prefix_m(&self, dir: [f64; 3]) -> (f64, f64) {
        let (lat, lon) = latlon_from_dir(dir);
        let knobs: TerrainKnobs = self.params.knobs.into();
        let macro_h = self.macro_term_m(dir, lat, lon, knobs);
        (macro_h + self.landmark_sum_m(dir, knobs), macro_h)
    }

    /// Height at `min_wavelength_m` from a shared [`height_prefix_m`](Self::height_prefix_m).
    /// Bitwise identical to [`height_m`](Self::height_m) up to the final
    /// ulp (the prefix crosses one extra rounding when split out).
    pub fn height_from_prefix(
        &self,
        dir: [f64; 3],
        prefix_m: f64,
        macro_h: f64,
        min_wavelength_m: f64,
    ) -> f64 {
        let min_wl = min_wavelength_m.max(1.0);
        let knobs: TerrainKnobs = self.params.knobs.into();
        let (meso_h, micro_h) = self.detail_parts_m(dir, min_wl, knobs, macro_h);
        let height = prefix_m + meso_h + micro_h - self.sea_offset_m;
        let height = self.regional_height(dir, min_wl, height, macro_h);
        height.clamp(self.params.height_min_m, self.params.height_max_m)
    }

    fn height_parts(&self, dir: [f64; 3], min_wavelength_m: f64) -> (f64, f64, f64) {
        self.height_parts_impl(dir, min_wavelength_m, true)
    }

    /// Geological parent used for stable survey scoring and bake diagnostics.
    /// Never use this as a collision/launch height: those use the final field.
    pub fn regional_parent_sample(&self, dir: [f64; 3], min_wavelength_m: f64) -> TerrainSample {
        let (height, macro_h, detail_h) = self.height_parts_impl(dir, min_wavelength_m, false);
        self.finish_sample(dir, height, macro_h, detail_h, 0.0)
    }

    fn height_parts_impl(
        &self,
        dir: [f64; 3],
        min_wavelength_m: f64,
        regional: bool,
    ) -> (f64, f64, f64) {
        let min_wl = min_wavelength_m.max(1.0);
        let (lat, lon) = latlon_from_dir(dir);
        let knobs: TerrainKnobs = self.params.knobs.into();
        let macro_h = self.macro_term_m(dir, lat, lon, knobs);
        let (meso_h, micro_h) = self.detail_parts_m(dir, min_wl, knobs, macro_h);
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
        if regional {
            height = self.regional_height(dir, min_wl, height, macro_h);
        }
        height = height.clamp(self.params.height_min_m, self.params.height_max_m);
        (height, macro_h - self.sea_offset_m, meso_h + micro_h)
    }

    fn regional_height(&self, dir: [f64; 3], min_wl: f64, mut height: f64, macro_h: f64) -> f64 {
        let patches = self.regional_erosion();
        if patches.is_empty() {
            return height;
        }
        let (lat, lon) = latlon_from_dir(dir);
        for patch in patches {
            let displacement = patch.displacement_at(lat, lon);
            if displacement <= 0.0 || min_wl >= patch.source_min_wavelength_m {
                height += displacement;
            } else {
                // Finite-depth infill: exposed peaks receive less sediment,
                // hollows receive more. A zero/thin deposit cannot erase an
                // entire unresolved depression merely because it is concave.
                // Resolved-grid volume is authoritative for this offline proxy;
                // a sub-cell transport/inventory solve is not represented here.
                let knobs = self.params.knobs.into();
                let (a, b) = self.detail_parts_m(dir, min_wl, knobs, macro_h);
                let (c, d) =
                    self.detail_parts_m(dir, patch.source_min_wavelength_m, knobs, macro_h);
                let fine_relief = (a + b) - (c + d);
                let remaining = fine_relief.signum() * (fine_relief.abs() - displacement).max(0.0);
                height += displacement + remaining - fine_relief;
            }
        }
        height
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

    /// Estimate the material slope at a physical scale.  Keeping this scale
    /// independent of the requesting tile makes biome transitions stable as
    /// a tile crosses an LOD boundary.
    pub(crate) fn slope_hint(&self, dir: [f64; 3], wavelength_m: f64) -> f64 {
        let (lat, lon) = latlon_from_dir(dir);
        let knobs: TerrainKnobs = self.params.knobs.into();
        self.slope_hint_from_macro(dir, wavelength_m, self.macro_term_m(dir, lat, lon, knobs))
    }

    /// Same material slope, reusing a macro sample already computed by a page.
    pub(crate) fn slope_hint_from_macro(
        &self,
        dir: [f64; 3],
        wavelength_m: f64,
        macro_h: f64,
    ) -> f64 {
        // Offset along two orthonormal tangent axes by half a wavelength.
        // Latitude/longitude offsets are ill-conditioned near the poles:
        // longitude's metres-per-degree tends to zero there and used to make
        // the material slope latitude-dependent.
        let up = [0.0, 1.0, 0.0];
        let mut east = [
            up[1] * dir[2] - up[2] * dir[1],
            up[2] * dir[0] - up[0] * dir[2],
            up[0] * dir[1] - up[1] * dir[0],
        ];
        let east_norm = (east[0] * east[0] + east[1] * east[1] + east[2] * east[2]).sqrt();
        if east_norm < 1.0e-12 {
            east = [1.0, 0.0, 0.0];
        } else {
            east = east.map(|v| v / east_norm);
        }
        let mut north = [
            dir[1] * east[2] - dir[2] * east[1],
            dir[2] * east[0] - dir[0] * east[2],
            dir[0] * east[1] - dir[1] * east[0],
        ];
        let north_norm = (north[0] * north[0] + north[1] * north[1] + north[2] * north[2]).sqrt();
        north = north.map(|v| v / north_norm);
        let angle = (wavelength_m * 0.5 / self.params.radius_m).max(1.0 / self.params.radius_m);
        let (sin_angle, cos_angle) = angle.sin_cos();
        let offset = |tangent: [f64; 3]| {
            std::array::from_fn(|i| dir[i] * cos_angle + tangent[i] * sin_angle)
        };
        let (lat, lon) = latlon_from_dir(dir);
        let h0 = macro_h
            + crate::terrain::eval_meso_m(
                self.params.seed,
                self.params.knobs.into(),
                lat,
                lon,
                self.params.radius_m,
            );
        let neighbour_height = |d| {
            let (lat, lon) = latlon_from_dir(d);
            self.base_height(d) + self.erosion_delta_at(lat, lon)
        };
        let hx = neighbour_height(offset(east));
        let hy = neighbour_height(offset(north));
        let dx = (wavelength_m * 0.5).max(1.0);
        (((hx - h0) / dx).powi(2) + ((hy - h0) / dx).powi(2)).sqrt()
    }

    fn context_wetland_at(&self, lat: f64, lon: f64) -> f64 {
        self.context_scalar_at(&self.context.wetland_support, lat, lon)
    }

    fn erosion_delta_at(&self, lat: f64, lon: f64) -> f64 {
        let Some(relief) = &self.frozen_erosion else {
            return 0.0;
        };
        let rows = relief.delta_m.len();
        let cols = relief.delta_m[0].len();
        let y = ((lat + 90.0) / relief.step_deg).clamp(0.0, (rows - 1) as f64);
        let x = (lon + 180.0).rem_euclid(360.0) / relief.step_deg;
        let (r, c) = (y.floor() as usize, x.floor() as usize);
        let nr = (r + 1).min(rows - 1);
        let nc = (c + 1) % cols;
        let (u, v) = (x.fract(), y.fract());
        let a = relief.delta_m[r][c];
        let d = relief.delta_m[nr][nc];
        // Same triangulation as the frozen DEM and shoreline extractor.
        if v <= u {
            a * (1.0 - u) + relief.delta_m[r][nc] * (u - v) + d * v
        } else {
            a * (1.0 - v) + d * u + relief.delta_m[nr][c] * (v - u)
        }
    }

    fn context_scalar_at(&self, values: &[Vec<f64>], lat: f64, lon: f64) -> f64 {
        let rows = self.context.lats.len();
        let cols = self.context.lons.len();
        let y = ((lat + 90.0) / 2.0).clamp(0.0, (rows - 1) as f64);
        let x = (lon + 180.0).rem_euclid(360.0) / 2.0;
        let (r, c) = (y.floor() as usize, x.floor() as usize);
        let (fy, fx) = (y - r as f64, x - c as f64);
        let blend = |row: usize| lerp_context(values[row][c], values[row][(c + 1) % cols], fx);
        lerp_context(blend(r), blend((r + 1).min(rows - 1)), fy)
    }
}

fn stencil_geometry(h: [f64; 5], distance_m: f64) -> (f64, f64) {
    let gx = (h[1] - h[2]) / (2.0 * distance_m);
    let gy = (h[3] - h[4]) / (2.0 * distance_m);
    let laplacian =
        ((h[1] - h[0]) + (h[2] - h[0]) + (h[3] - h[0]) + (h[4] - h[0])) / distance_m.powi(2);
    (gx.hypot(gy), laplacian)
}

// The context is a regular 2 degree grid. Lower cell wins exact ties,
// matching the former scan, with longitude periodic at the seam.
fn lerp_context(a: f64, b: f64, t: f64) -> f64 {
    if t == 0.0 {
        a
    } else if t == 1.0 {
        b
    } else {
        a * (1.0 - t) + b * t
    }
}

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
    params.hydrology.validate()?;
    params.surface_climate.validate()?;
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
    let sea_offset_m = if !field.context.lats.is_empty() || field.has_frozen_erosion() {
        0.0
    } else {
        params
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
            .unwrap_or(0.0)
    };
    for row in &mut grid.h {
        for height in row {
            *height -= sea_offset_m;
        }
    }
    let water0: Vec<Vec<WaterClass>> = grid
        .h
        .iter()
        .map(|row| {
            row.iter()
                .map(|h| {
                    if *h < 0.0 {
                        WaterClass::Ocean
                    } else {
                        WaterClass::Land
                    }
                })
                .collect()
        })
        .collect();
    let dist_m = continentality_metres(&grid, &water0);
    let mut moisture01 = crate::climate::moisture_grid(&grid, params.surface_climate);
    let arid: Vec<Vec<f64>> = moisture01
        .iter()
        .map(|row| row.iter().map(|m| 1.0 - m).collect())
        .collect();
    let mut ocean_dist_m = vec![vec![0.0; grid.cols()]; grid.rows()];
    for (r, row) in dist_m.iter().enumerate() {
        for (c, d) in row.iter().enumerate() {
            ocean_dist_m[r][c] = *d;
        }
    }
    let temperatures: Vec<Vec<f64>> = lats
        .iter()
        .enumerate()
        .map(|(r, lat)| {
            lons.iter()
                .enumerate()
                .map(|(c, lon)| {
                    let dir = dir_from_latlon(*lat, *lon);
                    let flux = geothermal_activity(
                        &field.provinces,
                        *lat,
                        *lon,
                        params.radius_m,
                        0.0,
                        0.0,
                    ) * field.geo_norm;
                    field.temperature_for_height(dir, grid.h[r][c], flux)
                })
                .collect()
        })
        .collect();
    let lakes: Vec<_> = crate::hydro::lake_basins_with_balance(
        &grid,
        &moisture01,
        &temperatures,
        params.surface_climate,
    )
    .expect("validated climate and matching context dimensions")
    .into_iter()
    .filter(|b| {
        if b.salt_flat {
            params.hydrology.closed_basin_salt_flats
        } else {
            params.hydrology.allow_lakes
        }
    })
    .collect();
    let mut salt_spill_m = vec![vec![None; grid.cols()]; grid.rows()];
    for lake in &lakes {
        if lake.salt_flat {
            for [r, c] in &lake.cells {
                salt_spill_m[*r][*c] = Some(lake.surface_height_m);
            }
        }
    }
    let depressions = crate::hydro::depression_depth(&grid);
    let catchments = crate::terrain_fields::flow_catchment_area_m2(&grid);
    let rivers = if params.hydrology.route_rivers_downhill {
        crate::hydro::river_reaches(
            &grid,
            &catchments,
            params.hydrology.river_min_catchment_area_m2,
        )
        .expect("validated hydrology and matching context dimensions")
    } else {
        Vec::new()
    };
    let mut wetland_support: Vec<Vec<f64>> = grid
        .h
        .iter()
        .enumerate()
        .map(|(r, row)| {
            row.iter()
                .enumerate()
                .map(|(c, h)| {
                    if *h < 0.0 {
                        0.0
                    } else {
                        crate::hydro::wetland_potential01(
                            crate::terrain_fields::slope_at(&grid, r, c),
                            depressions[r][c],
                            catchments[r][c],
                            arid[r][c],
                        )
                    }
                })
                .collect()
        })
        .collect();
    // Each pole is one physical point, not distinct longitude sectors. Use
    // the cap mean at the endpoints so interpolation converges consistently.
    for r in [0, grid.rows() - 1] {
        let mean = wetland_support[r].iter().sum::<f64>() / grid.cols() as f64;
        wetland_support[r].fill(mean);
        let distance_mean = ocean_dist_m[r].iter().sum::<f64>() / grid.cols() as f64;
        ocean_dist_m[r].fill(distance_mean);
        let moisture_mean = moisture01[r].iter().sum::<f64>() / grid.cols() as f64;
        moisture01[r].fill(moisture_mean);
    }
    (
        ContextGrid {
            lats,
            lons,
            ocean_dist_m,
            moisture01,
            salt_spill_m,
            hydrology_step_deg: step,
            wetland_support,
            rivers,
            lakes,
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
    crate::manifest::validate_manifest(manifest)?;
    let params = PlanetParams {
        hydrology: manifest.hydrology,
        surface_climate: manifest.climate.surface,
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
        eclipse_strength: manifest.climate.surface.eclipse_cooling_strength,
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
    let provinces = manifest
        .geothermal
        .place(manifest.planet.seed, &hot_spots)?;
    let mut field = PlanetField::build(
        params,
        manifest.features.clone(),
        manifest.tectonics.clone(),
        provinces,
        manifest.landmark_zones.clone(),
    )?;
    if let Some(recipe) = manifest.civilization {
        field.civilization = Some(recipe);
        field.inhabited_regions = crate::civilization::derive_regions(&field, recipe)?;
    }
    Ok(field)
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
            hydrology: crate::hydro::HydrologyRecipe::default(),
            surface_climate: crate::climate::SurfaceClimate::default(),
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
    fn surface_geometry_stencil_recovers_planes_and_curvature_sign() {
        let (slope, curvature) = stencil_geometry([3.0, 7.0, -1.0, 9.0, -3.0], 2.0);
        assert!((slope - 13.0_f64.sqrt()).abs() < 1e-12);
        assert_eq!(curvature, 0.0);
        assert_eq!(stencil_geometry([0.0, 4.0, 4.0, 4.0, 4.0], 2.0), (0.0, 4.0));
        assert_eq!(
            stencil_geometry([0.0, -4.0, -4.0, -4.0, -4.0], 2.0),
            (0.0, -4.0)
        );
    }

    #[test]
    fn physical_surface_geometry_reuses_prefix_and_is_finite_at_poles_and_seam() {
        let field = test_field();
        for (lat, lon) in [(90.0, 0.0), (-90.0, 0.0), (20.0, 180.0), (20.0, -180.0)] {
            let dir = dir_from_latlon(lat, lon);
            let direct = field.surface_geometry(dir, 256.0);
            let (prefix, macro_h) = field.height_prefix_m(dir);
            assert_eq!(
                direct,
                field.surface_geometry_from_prefix(dir, prefix, macro_h, 256.0)
            );
            assert!(direct.0.is_finite() && direct.1.is_finite());
            assert!(direct.0 >= 0.0);
        }
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
    fn material_slope_is_finite_and_coordinate_stable_at_pole() {
        let field = test_field();
        let a = field.slope_hint(dir_from_latlon(90.0, 0.0), 256.0);
        let b = field.slope_hint(dir_from_latlon(90.0, 137.0), 256.0);
        assert!(a.is_finite() && b.is_finite());
        assert!((a - b).abs() < 1.0e-10, "a={a}, b={b}");
    }

    #[test]
    fn material_slope_reuses_the_same_canonical_macro_sample() {
        let field = test_field();
        for (lat, lon) in [(0.0, 0.0), (25.0, 179.0), (-70.0, -179.0), (90.0, 0.0)] {
            let dir = dir_from_latlon(lat, lon);
            let (_, macro_h) = field.height_prefix_m(dir);
            assert_eq!(
                field.slope_hint(dir, 256.0),
                field.slope_hint_from_macro(dir, 256.0, macro_h)
            );
        }
    }

    #[test]
    fn wetland_context_is_interpolated_periodic_and_prefix_stable() {
        let mut field = test_field();
        for lat in [-90.0, 90.0] {
            let pole = field.context_wetland_at(lat, 0.0);
            for lon in [-180.0, -45.0, 75.0, 180.0] {
                assert!((field.context_wetland_at(lat, lon) - pole).abs() < 1e-12);
            }
        }
        let r = 45;
        for row in &mut field.context.wetland_support {
            row.fill(0.0);
        }
        field.context.wetland_support[r][179] = 1.0;
        field.context.wetland_support[r][0] = 1.0;
        assert_eq!(
            field.context_wetland_at(0.0, -180.0),
            field.context_wetland_at(0.0, 180.0)
        );
        assert!((field.context_wetland_at(0.0, -179.0) - 0.5).abs() < 1e-12);
        assert!(
            (field.context_wetland_at(0.0, -179.0 - 1e-6)
                - field.context_wetland_at(0.0, -179.0 + 1e-6))
            .abs()
                < 2e-6
        );
        let dir = dir_from_latlon(0.0, -179.0);
        let sample = field.sample_surface(dir, 32.0);
        let (prefix, macro_h) = field.height_prefix_m(dir);
        let reused = field.sample_surface_from_prefix(dir, prefix, macro_h, 32.0);
        assert_eq!(sample.wetland_potential01, reused.wetland_potential01);
        for i in 0..512 {
            let y = 1.0 - 2.0 * (i as f64 + 0.5) / 512.0;
            let a = i as f64 * 2.399963229728653;
            let radius = (1.0 - y * y).sqrt();
            let s = field.sample_surface([radius * a.cos(), y, radius * a.sin()], 32.0);
            assert!((0.0..=1.0).contains(&s.wetland_potential01));
            if s.height_m < 0.0 || s.height_m >= 400.0 {
                assert_eq!(s.wetland_potential01, 0.0);
            }
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

    #[test]
    fn continuous_ocean_driver_does_not_jump_at_context_cell_boundaries() {
        let mut field = test_field();
        for row in &mut field.context.ocean_dist_m {
            row.fill(100_000.0);
        }
        field.context.ocean_dist_m[45][90] = 0.0;
        assert_eq!(
            field.context_scalar_at(&field.context.ocean_dist_m, 0.0, 1.0),
            50_000.0
        );
        let left = field.context_scalar_at(&field.context.ocean_dist_m, 0.0, 1.0 - 1e-6);
        let right = field.context_scalar_at(&field.context.ocean_dist_m, 0.0, 1.0 + 1e-6);
        assert!((right - left).abs() < 0.11);
        assert_eq!(
            lerp_context(f64::INFINITY, f64::INFINITY, 0.0),
            f64::INFINITY
        );
        assert_eq!(
            lerp_context(f64::INFINITY, f64::INFINITY, 0.5),
            f64::INFINITY
        );
    }
}

#[cfg(test)]
mod prefix_regression_tests {
    use super::*;
    use crate::{
        spec_recipe::{SpecRecipe, manifest_from_spec},
        sphere::dir_from_latlon,
    };

    fn field() -> PlanetField {
        let recipe: SpecRecipe =
            toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml")).unwrap();
        field_from_manifest(&manifest_from_spec(&recipe).unwrap()).unwrap()
    }

    #[test]
    fn shared_prefix_reproduces_canonical_heights() {
        // The texture fast path must not fork the terrain: prefix-shared
        // heights track height_m to ~1 ulp, samples to a few ulp (different
        // association order only). Larger drift means the split changed the
        // math, and tiles would disagree with physics/contact queries.
        let field = field();
        let mut worst_h: f64 = 0.0;
        let mut worst_sample: f64 = 0.0;
        for i in 0..512 {
            let dir = dir_from_latlon(-60.0 + i as f64 * 0.2, 20.0 + i as f64 * 0.13);
            for min_wl in [2.0, 32.0, 500.0] {
                let (prefix, macro_h) = field.height_prefix_m(dir);
                let shared = field.height_from_prefix(dir, prefix, macro_h, min_wl);
                let exact = field.height_m(dir, min_wl);
                let scale = exact.abs().max(1.0);
                worst_h = worst_h.max((shared - exact).abs() / scale);
                let a = field.sample_surface_from_prefix(dir, prefix, macro_h, 32.0);
                let b = field.sample_surface(dir, 32.0);
                let sscale = b.height_m.abs().max(1.0);
                worst_sample = worst_sample.max((a.height_m - b.height_m).abs() / sscale);
                assert_eq!(a.biome, b.biome, "prefix path changed classification");
            }
        }
        eprintln!("prefix height drift {worst_h:e}, sample drift {worst_sample:e}");
        assert!(worst_h < 1e-12, "prefix height drifted {worst_h:e}");
        assert!(
            worst_sample < 1e-12,
            "prefix sample drifted {worst_sample:e}"
        );
    }
}

/// Declared obstacle heights around a site: the data an unobserved craft
/// (or a future autopilot landing / impact predictor) needs instead of
/// visual tiles. Full physics reads the same field through the same
/// [`PlanetField::height_m`]; only mesh/texture/normal synthesis is
/// skipped. Heights are exact at the sampled points; the track coverage proof
/// is geometric. A caller that also supplies a separately validated slope
/// bound can build an explicit withstand proof.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ObstacleReport {
    pub center_dir: [f64; 3],
    pub radius_m: f64,
    pub grid_step_m: f64,
    /// Number of samples on one side of the square grid. The square contains
    /// the requested disc, so this plus `grid_step_m` is an auditable layout
    /// proof rather than an opaque sample count.
    pub grid_side: u32,
    /// Worst-case distance from any point in the sampled square to its nearest
    /// grid sample. This proves geometric sample coverage only; it is not yet
    /// a bound on unresolved terrain relief.
    pub sample_cover_radius_m: f64,
    pub samples: u32,
    pub center_height_m: f64,
    pub max_height_m: f64,
    pub min_height_m: f64,
    /// Max grid slope (rise/run) between orthogonal neighbors.
    pub max_slope: f64,
}

/// Conservative terrain-clearance proof built on top of an obstacle report.
/// The caller supplies a separately validated upper bound for terrain slope;
/// `ObstacleReport::max_slope` is only an observed lower bound from adjacent
/// samples and is therefore never silently treated as a global guarantee.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ObstacleWithstandProof {
    pub evaluated_reports: u32,
    pub altitude_m: f64,
    pub footprint_radius_m: f64,
    pub slope_bound: f64,
    pub max_sampled_height_m: f64,
    pub unresolved_relief_bound_m: f64,
    pub conservative_max_height_m: f64,
    pub clearance_m: f64,
}

impl ObstacleWithstandProof {
    pub fn withstands(&self) -> bool {
        self.clearance_m >= 0.0
    }
}

impl ObstacleReport {
    /// Prove a conservative altitude clearance for this report. The proof is
    /// valid when `slope_bound` is an externally certified upper bound on the
    /// terrain's rise/run over the report and the vehicle footprint. The
    /// report's sample cover radius plus the footprint radius bounds the
    /// farthest unresolved point that can affect the vehicle.
    pub fn withstand_proof(
        &self,
        altitude_m: f64,
        footprint_radius_m: f64,
        slope_bound: f64,
    ) -> Result<ObstacleWithstandProof, String> {
        validate_withstand_inputs(altitude_m, footprint_radius_m, slope_bound)?;
        if slope_bound + 1.0e-12 < self.max_slope {
            return Err(format!(
                "withstand slope bound {slope_bound} is below observed report slope {}",
                self.max_slope
            ));
        }
        Ok(build_withstand_proof(
            1,
            altitude_m,
            footprint_radius_m,
            slope_bound,
            self.max_height_m,
            self.sample_cover_radius_m,
        ))
    }
}

/// Geometric coverage evidence for a sequence of ground-track directions.
/// The track is covered when every consecutive pair of centers is no farther
/// apart than two obstacle-report radii. A caller can add an explicit
/// withstand/error bound with [`ObstacleTrackCertificate::withstand_proof`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ObstacleCoverageProof {
    pub track_points: u32,
    pub obstacle_radius_m: f64,
    /// Tangent-plane report radius mapped back to the spherical surface.
    pub obstacle_geodesic_radius_m: f64,
    pub max_center_spacing_m: f64,
    pub allowed_center_spacing_m: f64,
    pub grid_step_m: f64,
    pub grid_side: u32,
    pub sample_cover_radius_m: f64,
}

impl ObstacleCoverageProof {
    pub fn covers_track(&self) -> bool {
        self.max_center_spacing_m
            <= self.allowed_center_spacing_m + 1.0e-9 * self.allowed_center_spacing_m.max(1.0)
    }
}

/// Obstacle reports collected along one track. `coverage` is the proof that
/// the report discs cover the piecewise-geodesic centerline; the max/min values
/// summarize the same reports for a caller that needs a conservative terrain
/// ceiling.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ObstacleTrackCertificate {
    pub reports: Vec<ObstacleReport>,
    pub coverage: ObstacleCoverageProof,
    pub max_height_m: f64,
    pub min_height_m: f64,
}

impl ObstacleTrackCertificate {
    pub fn covers_track(&self) -> bool {
        self.coverage.covers_track()
    }

    /// Apply one externally validated slope bound to every report in the
    /// covered track and return the worst conservative clearance.
    pub fn withstand_proof(
        &self,
        altitude_m: f64,
        footprint_radius_m: f64,
        slope_bound: f64,
    ) -> Result<ObstacleWithstandProof, String> {
        validate_withstand_inputs(altitude_m, footprint_radius_m, slope_bound)?;
        let observed_slope = self
            .reports
            .iter()
            .map(|report| report.max_slope)
            .fold(0.0, f64::max);
        if slope_bound + 1.0e-12 < observed_slope {
            return Err(format!(
                "withstand slope bound {slope_bound} is below observed track slope {observed_slope}"
            ));
        }
        let max_cover_radius = self
            .reports
            .iter()
            .map(|report| report.sample_cover_radius_m)
            .fold(0.0, f64::max);
        Ok(build_withstand_proof(
            self.reports.len() as u32,
            altitude_m,
            footprint_radius_m,
            slope_bound,
            self.max_height_m,
            max_cover_radius,
        ))
    }
}

fn validate_withstand_inputs(
    altitude_m: f64,
    footprint_radius_m: f64,
    slope_bound: f64,
) -> Result<(), String> {
    if !altitude_m.is_finite() {
        return Err(format!(
            "withstand altitude must be finite, got {altitude_m}"
        ));
    }
    if !footprint_radius_m.is_finite() || footprint_radius_m < 0.0 {
        return Err(format!(
            "withstand footprint radius must be finite and non-negative, got {footprint_radius_m}"
        ));
    }
    if !slope_bound.is_finite() || slope_bound < 0.0 {
        return Err(format!(
            "withstand slope bound must be finite and non-negative, got {slope_bound}"
        ));
    }
    Ok(())
}

fn build_withstand_proof(
    evaluated_reports: u32,
    altitude_m: f64,
    footprint_radius_m: f64,
    slope_bound: f64,
    max_sampled_height_m: f64,
    sample_cover_radius_m: f64,
) -> ObstacleWithstandProof {
    let unresolved_relief_bound_m = slope_bound * (sample_cover_radius_m + footprint_radius_m);
    let conservative_max_height_m = max_sampled_height_m + unresolved_relief_bound_m;
    ObstacleWithstandProof {
        evaluated_reports,
        altitude_m,
        footprint_radius_m,
        slope_bound,
        max_sampled_height_m,
        unresolved_relief_bound_m,
        conservative_max_height_m,
        clearance_m: altitude_m - conservative_max_height_m,
    }
}

impl PlanetField {
    /// Sample obstacle heights on a tangent-plane disc around `center_dir`.
    /// Grid step is `max(32 m, radius/32)` with the sampling wavelength
    /// matched to it, so the grid resolves what the samples contain;
    /// side count caps at 129 (under 17k samples, one-time cost).
    pub fn declare_obstacles(
        &self,
        center_dir: [f64; 3],
        radius_m: f64,
    ) -> Result<ObstacleReport, String> {
        if !radius_m.is_finite() || radius_m < 0.0 {
            return Err(format!(
                "obstacle radius must be finite and non-negative, got {radius_m}"
            ));
        }
        let center_len_sq = center_dir[0] * center_dir[0]
            + center_dir[1] * center_dir[1]
            + center_dir[2] * center_dir[2];
        if !center_len_sq.is_finite() || center_len_sq <= 0.0 {
            return Err("obstacle center direction must be finite and nonzero".to_string());
        }
        let center = [
            center_dir[0] / center_len_sq.sqrt(),
            center_dir[1] / center_len_sq.sqrt(),
            center_dir[2] / center_len_sq.sqrt(),
        ];
        // Tangent basis: deterministic pick, orthogonalized exactly.
        let helper = if center[0].abs() < 0.9 {
            [1.0, 0.0, 0.0]
        } else {
            [0.0, 1.0, 0.0]
        };
        let mut e1 = [
            center[1] * helper[2] - center[2] * helper[1],
            center[2] * helper[0] - center[0] * helper[2],
            center[0] * helper[1] - center[1] * helper[0],
        ];
        let e1_len = (e1[0] * e1[0] + e1[1] * e1[1] + e1[2] * e1[2]).sqrt();
        e1 = [e1[0] / e1_len, e1[1] / e1_len, e1[2] / e1_len];
        let e2 = [
            center[1] * e1[2] - center[2] * e1[1],
            center[2] * e1[0] - center[0] * e1[2],
            center[0] * e1[1] - center[1] * e1[0],
        ];
        let mut step = (32.0f64).max(radius_m / 32.0);
        let mut half = (radius_m / step).ceil() as usize;
        if 2 * half + 1 > 129 {
            half = 64;
            step = radius_m / 64.0;
        }
        let min_wavelength_m = step.max(32.0);
        let side = 2 * half + 1;
        let mut heights = Vec::with_capacity(side * side);
        let mut max_height_m = f64::NEG_INFINITY;
        let mut min_height_m = f64::INFINITY;
        for iy in 0..side {
            for ix in 0..side {
                let dx = (ix as f64 - half as f64) * step;
                let dy = (iy as f64 - half as f64) * step;
                let p = [
                    center[0] * self.params.radius_m + e1[0] * dx + e2[0] * dy,
                    center[1] * self.params.radius_m + e1[1] * dx + e2[1] * dy,
                    center[2] * self.params.radius_m + e1[2] * dx + e2[2] * dy,
                ];
                let len = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt();
                let dir = [p[0] / len, p[1] / len, p[2] / len];
                let h = self.height_m(dir, min_wavelength_m);
                if !h.is_finite() {
                    return Err("obstacle sampling hit non-finite height".to_string());
                }
                max_height_m = max_height_m.max(h);
                min_height_m = min_height_m.min(h);
                heights.push(h);
            }
        }
        let center_height_m = heights[half * side + half];
        let mut max_slope: f64 = 0.0;
        for iy in 0..side {
            for ix in 0..side {
                let h = heights[iy * side + ix];
                if ix + 1 < side {
                    max_slope = max_slope.max(((heights[iy * side + ix + 1] - h) / step).abs());
                }
                if iy + 1 < side {
                    max_slope = max_slope.max(((heights[(iy + 1) * side + ix] - h) / step).abs());
                }
            }
        }
        Ok(ObstacleReport {
            center_dir: center,
            radius_m,
            grid_step_m: step,
            grid_side: side as u32,
            sample_cover_radius_m: if side == 1 {
                0.0
            } else {
                step * 0.5_f64.sqrt()
            },
            samples: heights.len() as u32,
            center_height_m,
            max_height_m,
            min_height_m,
            max_slope,
        })
    }

    /// Declare obstacles along a ground track and return explicit geometric
    /// coverage evidence. Each report samples a tangent-plane square that
    /// contains its obstacle disc. Consecutive report centers must be close
    /// enough for those discs to cover the complete piecewise-geodesic track.
    /// This does not claim that unresolved sub-grid relief can withstand a
    /// vehicle; that is the next certification layer.
    pub fn certify_obstacle_track(
        &self,
        track_dirs: &[[f64; 3]],
        obstacle_radius_m: f64,
    ) -> Result<ObstacleTrackCertificate, String> {
        if track_dirs.is_empty() {
            return Err("obstacle track must contain at least one point".into());
        }
        if !obstacle_radius_m.is_finite() || obstacle_radius_m < 0.0 {
            return Err(format!(
                "obstacle track radius must be finite and non-negative, got {obstacle_radius_m}"
            ));
        }
        let mut normalized = Vec::with_capacity(track_dirs.len());
        for direction in track_dirs {
            let length_sq = direction[0] * direction[0]
                + direction[1] * direction[1]
                + direction[2] * direction[2];
            if !length_sq.is_finite() || length_sq <= 0.0 {
                return Err("obstacle track contains a degenerate direction".into());
            }
            let length = length_sq.sqrt();
            normalized.push([
                direction[0] / length,
                direction[1] / length,
                direction[2] / length,
            ]);
        }
        let mut max_center_spacing_m: f64 = 0.0;
        for pair in normalized.windows(2) {
            let dot = (pair[0][0] * pair[1][0] + pair[0][1] * pair[1][1] + pair[0][2] * pair[1][2])
                .clamp(-1.0, 1.0);
            max_center_spacing_m = max_center_spacing_m.max(self.params.radius_m * dot.acos());
        }
        let reports: Vec<_> = normalized
            .iter()
            .map(|direction| self.declare_obstacles(*direction, obstacle_radius_m))
            .collect::<Result<_, _>>()?;
        let first = reports.first().expect("track has at least one report");
        let coverage = ObstacleCoverageProof {
            track_points: normalized.len() as u32,
            obstacle_radius_m,
            max_center_spacing_m,
            obstacle_geodesic_radius_m: self.params.radius_m
                * (obstacle_radius_m / self.params.radius_m).atan(),
            allowed_center_spacing_m: 2.0
                * self.params.radius_m
                * (obstacle_radius_m / self.params.radius_m).atan(),
            grid_step_m: first.grid_step_m,
            grid_side: first.grid_side,
            sample_cover_radius_m: first.sample_cover_radius_m,
        };
        if !coverage.covers_track() {
            return Err(format!(
                "obstacle track sampling gap {:.3} m exceeds covered spacing {:.3} m",
                coverage.max_center_spacing_m, coverage.allowed_center_spacing_m
            ));
        }
        Ok(ObstacleTrackCertificate {
            max_height_m: reports
                .iter()
                .map(|report| report.max_height_m)
                .fold(f64::NEG_INFINITY, f64::max),
            min_height_m: reports
                .iter()
                .map(|report| report.min_height_m)
                .fold(f64::INFINITY, f64::min),
            reports,
            coverage,
        })
    }
}

#[cfg(test)]
mod obstacle_tests {
    use super::*;

    fn field() -> PlanetField {
        let recipe: crate::spec_recipe::SpecRecipe =
            toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml")).unwrap();
        crate::field::field_from_manifest(&crate::spec_recipe::manifest_from_spec(&recipe).unwrap())
            .unwrap()
    }

    #[test]
    fn obstacle_report_is_deterministic_and_bounds_center() {
        let field = field();
        let raw: [f64; 3] = [0.3, 0.8, 0.5];
        let len: f64 = (raw[0] * raw[0] + raw[1] * raw[1] + raw[2] * raw[2]).sqrt();
        let dir = [raw[0] / len, raw[1] / len, raw[2] / len];
        let a = field.declare_obstacles(dir, 2000.0).expect("report");
        let b = field.declare_obstacles(dir, 2000.0).expect("report");
        assert_eq!(a, b);
        assert!(a.max_height_m >= a.center_height_m);
        assert!(a.min_height_m <= a.center_height_m);
        assert!(a.max_slope >= 0.0);
        assert!(a.samples > 100);
        // Center agrees with a direct sample at comparable wavelength.
        let direct = field.height_m(dir, 32.0);
        assert!(
            (a.center_height_m - direct).abs() <= 1.0,
            "center {} vs direct {direct}",
            a.center_height_m
        );
    }

    #[test]
    fn obstacle_report_exposes_geometric_sampling_coverage() {
        let field = field();
        let report = field
            .declare_obstacles([0.0, 1.0, 0.0], 2_000.0)
            .expect("report");
        assert_eq!(report.grid_side * report.grid_side, report.samples);
        assert!(report.sample_cover_radius_m > 0.0);
        assert!(report.sample_cover_radius_m <= report.grid_step_m);
    }

    #[test]
    fn obstacle_report_withstand_proof_separates_slope_bound_from_observed_samples() {
        let field = field();
        let report = field
            .declare_obstacles([0.0, 1.0, 0.0], 2_000.0)
            .expect("report");
        let slope_bound = report.max_slope + 0.25;
        let safe_altitude =
            report.max_height_m + slope_bound * (report.sample_cover_radius_m + 5.0) + 1.0;
        let proof = report
            .withstand_proof(safe_altitude, 5.0, slope_bound)
            .expect("withstand proof");
        assert!(proof.withstands());
        assert_eq!(proof.evaluated_reports, 1);
        assert!(proof.unresolved_relief_bound_m > 0.0);
        assert!((proof.clearance_m - 1.0).abs() < 1.0e-9);

        let unsafe_proof = report
            .withstand_proof(proof.conservative_max_height_m - 1.0, 5.0, slope_bound)
            .expect("proof remains a valid negative result");
        assert!(!unsafe_proof.withstands());
        assert!(
            report
                .withstand_proof(safe_altitude, 5.0, report.max_slope - 1.0e-9)
                .is_err()
        );
    }

    #[test]
    fn obstacle_track_withstand_proof_uses_the_worst_report() {
        let field = field();
        let center = [0.0, 1.0, 0.0];
        let small_turn = [0.001, (1.0_f64 - 0.001_f64.powi(2)).sqrt(), 0.0];
        let certificate = field
            .certify_obstacle_track(&[center, small_turn], 20_000.0)
            .expect("covered track");
        let slope_bound = certificate
            .reports
            .iter()
            .map(|report| report.max_slope)
            .fold(0.0, f64::max)
            + 0.5;
        let altitude = certificate.max_height_m
            + slope_bound * (certificate.coverage.sample_cover_radius_m + 10.0)
            + 1.0;
        let proof = certificate
            .withstand_proof(altitude, 10.0, slope_bound)
            .expect("track withstand proof");
        assert_eq!(proof.evaluated_reports, 2);
        assert!(proof.withstands());
    }

    #[test]
    fn obstacle_track_certificate_proves_centerline_coverage() {
        let field = field();
        let center = [0.0, 1.0, 0.0];
        let small_turn = [0.001, (1.0_f64 - 0.001_f64.powi(2)).sqrt(), 0.0];
        let certificate = field
            .certify_obstacle_track(&[center, small_turn], 20_000.0)
            .expect("covered track");
        assert!(certificate.covers_track());
        assert_eq!(certificate.coverage.track_points, 2);
        assert_eq!(certificate.reports.len(), 2);
        assert!(certificate.coverage.max_center_spacing_m < 40_000.0);
        assert!(certificate.max_height_m >= certificate.min_height_m);

        let uncovered = field.certify_obstacle_track(&[center, [1.0, 0.0, 0.0]], 20_000.0);
        assert!(uncovered.is_err(), "a wide sampling gap must not certify");
    }

    #[test]
    fn obstacle_report_rejects_degenerate_inputs() {
        let field = field();
        assert!(field.declare_obstacles([0.0, 0.0, 0.0], 100.0).is_err());
        assert!(
            field
                .declare_obstacles([f64::NAN, 0.0, 0.0], 100.0)
                .is_err()
        );
        assert!(field.declare_obstacles([0.0, 1.0, 0.0], -1.0).is_err());
        assert!(
            field
                .declare_obstacles([0.0, 1.0, 0.0], f64::INFINITY)
                .is_err()
        );
        // Zero radius is a single-point report, not an error.
        let point = field
            .declare_obstacles([0.0, 1.0, 0.0], 0.0)
            .expect("point");
        assert_eq!(point.samples, 1);
        assert_eq!(point.max_slope, 0.0);
    }

    #[test]
    fn obstacle_max_covers_known_highland_site() {
        // A 50 km declaration around high ground must see multi-kilometre
        // relief (otherwise low batches would certify through mountains).
        let field = field();
        // Deterministic high ground: scan a fixed latitude circle for elevation.
        let mut best = (f64::NEG_INFINITY, [0.0f64, 1.0, 0.0]);
        for i in 0..360 {
            let lon = i as f64 * std::f64::consts::TAU / 360.0;
            let raw = [0.5 * lon.cos(), 0.5, 0.5 * lon.sin()];
            let len: f64 = (raw[0] * raw[0] + raw[1] * raw[1] + raw[2] * raw[2]).sqrt();
            let dir = [raw[0] / len, raw[1] / len, raw[2] / len];
            let h = field.height_m(dir, 32.0);
            if h > best.0 {
                best = (h, dir);
            }
        }
        eprintln!("highest meridian sample: {:.0} m", best.0);
        let report = field.declare_obstacles(best.1, 50_000.0).expect("report");
        assert!(
            report.max_height_m >= best.0,
            "report max {} below sampled {}",
            report.max_height_m,
            best.0
        );
        assert!(
            report.max_height_m > 1000.0,
            "relief {}",
            report.max_height_m
        );
    }
}
