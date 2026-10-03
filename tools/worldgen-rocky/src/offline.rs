//! Offline surface compilation. Loading/sampling the artifact never runs erosion,
//! drainage routing or vegetation placement. The live analytic field is a source,
//! not a substitute for these frozen results.
use crate::{field::PlanetField, hydro::HeightGrid, lod::TileKey};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct OfflineRecipe {
    pub grid_step_deg: f64,
    pub runoff: crate::erosion::RunoffErosion,
    /// Explicit close-range vegetation regions; no global billion-plant bake.
    pub vegetation_tiles: Vec<TileKey>,
}

impl Default for OfflineRecipe {
    fn default() -> Self {
        Self {
            grid_step_deg: 0.5,
            runoff: Default::default(),
            vegetation_tiles: Vec::new(),
        }
    }
}

impl OfflineRecipe {
    pub fn validate(&self) -> Result<(), String> {
        self.runoff.validate()?;
        let n = 180.0 / self.grid_step_deg;
        if !self.grid_step_deg.is_finite()
            || !(0.125..=10.0).contains(&self.grid_step_deg)
            || (n - n.round()).abs() > 1e-8
            || self.vegetation_tiles.len() > 256
        {
            return Err("offline grid step must divide 180 degrees, within 0.125..10; at most 256 vegetation tiles".into());
        }
        for tile in &self.vegetation_tiles {
            if tile.face >= 6
                || !(10..=20).contains(&tile.level)
                || tile.x >= 1u32 << tile.level
                || tile.y >= 1u32 << tile.level
            {
                return Err("offline vegetation needs valid cube tiles at levels 10..20".into());
            }
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FrozenSurface {
    pub schema: u32,
    pub seed: u64,
    pub radius_m: f64,
    pub recipe: OfflineRecipe,
    pub lats: Vec<f64>,
    pub lons: Vec<f64>,
    pub heights_m: Vec<Vec<f64>>,
    /// Pre-erosion reference and source identity, absent on historical schema 2.
    #[serde(default)]
    pub source: Option<FrozenSource>,
    /// Frozen unlit sRGB plus perceptual roughness. No runtime material synthesis.
    pub material_rgba: Vec<Vec<[u8; 4]>>,
    pub ecological_cover: Vec<Vec<crate::ecology::VegetationCover>>,
    pub rivers: Vec<crate::hydro::RiverReach>,
    pub lakes: Vec<crate::hydro::LakeBasin>,
    pub shore_segments: Vec<[[f64; 3]; 2]>,
    pub vegetation: Vec<PlantInstance>,
    pub prototypes: Vec<PlantMesh>,
    pub tiles: Vec<FrozenTile>,
    pub sediment_volume_error_m3: f64,
    /// Schema 4: finer erosion of selected geographic regions, on this parent
    /// surface. The global DEM/material snapshots are intentionally retained.
    #[serde(default)]
    pub regional_erosion: Vec<RegionalErosion>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegionalErosionRecipe {
    pub id: String,
    pub center_lat_deg: f64,
    pub center_lon_deg: f64,
    pub spacing_m: f64,
    pub cells: usize,
    pub runoff: crate::erosion::RunoffErosion,
}

impl RegionalErosionRecipe {
    pub fn validate(&self, radius_m: f64) -> Result<(), String> {
        self.runoff.validate()?;
        let half_lat = (self.cells as f64 * self.spacing_m * 0.5 / radius_m).to_degrees();
        if self.id.is_empty()
            || !self.center_lat_deg.is_finite()
            || !self.center_lon_deg.is_finite()
            || self.center_lat_deg.abs() + half_lat >= 85.0
            || !(-180.0..=180.0).contains(&self.center_lon_deg)
            || !self.spacing_m.is_finite()
            || !(32.0..=2000.0).contains(&self.spacing_m)
            || !(16..=1024).contains(&self.cells)
            || !self.cells.is_multiple_of(2)
        {
            return Err("regional erosion needs an even 16..1024-cell grid, 32..2000 m spacing, finite nonpolar center and an id".into());
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegionalErosion {
    pub recipe: RegionalErosionRecipe,
    pub lats: Vec<f64>,
    /// Unwrapped around the recipe's center, including dateline-crossing grids.
    pub lons: Vec<f64>,
    pub source_min_wavelength_m: f64,
    pub source_heights_m: Vec<Vec<f64>>,
    pub displacement_m: Vec<Vec<f64>>,
    pub exported_sediment_m3: f64,
    pub sediment_volume_error_m3: f64,
}

fn regional_domains_overlap(
    a: &RegionalErosionRecipe,
    b: &RegionalErosionRecipe,
    radius_m: f64,
) -> bool {
    let half_lat =
        |p: &RegionalErosionRecipe| (p.cells as f64 * p.spacing_m * 0.5 / radius_m).to_degrees();
    let (ay, by) = (half_lat(a), half_lat(b));
    let longitude_distance =
        ((a.center_lon_deg - b.center_lon_deg + 180.0).rem_euclid(360.0) - 180.0).abs();
    (a.center_lat_deg - b.center_lat_deg).abs() <= ay + by
        && longitude_distance
            <= ay / a.center_lat_deg.to_radians().cos() + by / b.center_lat_deg.to_radians().cos()
}

impl RegionalErosion {
    pub fn displacement_at(&self, lat: f64, lon: f64) -> f64 {
        let lon = self.recipe.center_lon_deg
            + (lon - self.recipe.center_lon_deg + 180.0).rem_euclid(360.0)
            - 180.0;
        if lat < self.lats[0]
            || lat > *self.lats.last().unwrap()
            || lon < self.lons[0]
            || lon > *self.lons.last().unwrap()
        {
            return 0.0;
        }
        let y = (lat - self.lats[0]) / (self.lats[1] - self.lats[0]);
        let x = (lon - self.lons[0]) / (self.lons[1] - self.lons[0]);
        let r = (y.floor() as usize).min(self.lats.len() - 2);
        let c = (x.floor() as usize).min(self.lons.len() - 2);
        let (u, v) = (x - c as f64, y - r as f64);
        let d = &self.displacement_m;
        if v <= u {
            d[r][c] * (1.0 - u) + d[r][c + 1] * (u - v) + d[r + 1][c + 1] * v
        } else {
            d[r][c] * (1.0 - v) + d[r + 1][c + 1] * u + d[r + 1][c] * (v - u)
        }
    }

    fn validate(&self, radius_m: f64) -> Result<(), String> {
        self.recipe.validate(radius_m)?;
        let n = self.recipe.cells + 1;
        let dy = (self.recipe.spacing_m / radius_m).to_degrees();
        let dx = dy / self.recipe.center_lat_deg.to_radians().cos();
        if self.lats.len() != n
            || self.lons.len() != n
            || self.source_min_wavelength_m != 2.0 * self.recipe.spacing_m
            || !self.exported_sediment_m3.is_finite()
            || self.exported_sediment_m3 < 0.0
            || !self.sediment_volume_error_m3.is_finite()
            || [&self.source_heights_m, &self.displacement_m]
                .into_iter()
                .any(|grid| {
                    grid.len() != n
                        || grid
                            .iter()
                            .any(|row| row.len() != n || row.iter().any(|h| !h.is_finite()))
                })
        {
            return Err("invalid regional erosion geometry or sediment budget".into());
        }
        for i in 0..n {
            let offset = i as f64 - self.recipe.cells as f64 * 0.5;
            if !self.lats[i].is_finite()
                || !self.lons[i].is_finite()
                || (self.lats[i] - self.recipe.center_lat_deg - offset * dy).abs() > 1e-9
                || (self.lons[i] - self.recipe.center_lon_deg - offset * dx).abs() > 1e-9
            {
                return Err("regional erosion coordinates disagree with physical spacing".into());
            }
            if self.displacement_m[0][i] != 0.0
                || self.displacement_m[n - 1][i] != 0.0
                || self.displacement_m[i][0] != 0.0
                || self.displacement_m[i][n - 1] != 0.0
            {
                return Err("regional open-boundary displacement must be zero".into());
            }
        }
        let net_volume: f64 = self
            .displacement_m
            .iter()
            .enumerate()
            .map(|(r, row)| {
                let lat = self.lats[r].to_radians();
                let area = radius_m.powi(2)
                    * dx.to_radians()
                    * ((lat + dy.to_radians() * 0.5).sin() - (lat - dy.to_radians() * 0.5).sin());
                row.iter().sum::<f64>() * area
            })
            .sum();
        let transported: f64 = self
            .displacement_m
            .iter()
            .enumerate()
            .map(|(r, row)| {
                let lat = self.lats[r].to_radians();
                let area = radius_m.powi(2)
                    * dx.to_radians()
                    * ((lat + dy.to_radians() * 0.5).sin() - (lat - dy.to_radians() * 0.5).sin());
                row.iter().map(|h| h.abs()).sum::<f64>() * area
            })
            .sum();
        let tolerance = 1e-3 + (transported + self.exported_sediment_m3) * 1e-9;
        if (net_volume + self.exported_sediment_m3).abs() > tolerance
            || (net_volume + self.exported_sediment_m3 - self.sediment_volume_error_m3).abs()
                > tolerance
        {
            return Err(
                "regional sediment volume disagrees with displacement and exported load".into(),
            );
        }
        Ok(())
    }
}

/// Extend an existing validated global bake; never repeat global generation.
pub fn refine_regions(
    field: &PlanetField,
    world: &mut FrozenSurface,
    recipes: &[RegionalErosionRecipe],
) -> Result<(), String> {
    world.validate()?;
    if !field.has_frozen_erosion()
        || !field.regional_erosion().is_empty()
        || !world.regional_erosion.is_empty()
        || recipes.is_empty()
        || recipes.len() > 16
    {
        return Err("regional refinement needs an attached parent, 1..16 regions and no existing refinement".into());
    }
    let expected: serde_json::Value = serde_json::from_str(
        &world
            .source
            .as_ref()
            .ok_or("refinement needs source reference")?
            .signature,
    )
    .map_err(|e| e.to_string())?;
    let actual: serde_json::Value =
        serde_json::from_str(&field.source_signature()?).map_err(|e| e.to_string())?;
    if expected != actual {
        return Err("regional refinement parent/source mismatch".into());
    }
    for (i, a) in recipes.iter().enumerate() {
        a.validate(world.radius_m)?;
        for b in &recipes[..i] {
            if a.id == b.id || regional_domains_overlap(a, b, world.radius_m) {
                return Err(
                    "regional erosion domains must have unique ids and disjoint context aprons"
                        .into(),
                );
            }
        }
    }
    for (r, lat) in world.lats.iter().enumerate() {
        for (c, lon) in world.lons.iter().enumerate() {
            let dir = crate::sphere::dir_from_latlon(*lat, *lon);
            if (field.height_m(dir, world.source.as_ref().unwrap().min_wavelength_m)
                - world.heights_m[r][c])
                .abs()
                > 1e-6
            {
                return Err("regional refinement field is not the supplied parent DEM".into());
            }
        }
    }
    let mut patches = Vec::new();
    for recipe in recipes {
        recipe.validate(world.radius_m)?;
        let dy = (recipe.spacing_m / world.radius_m).to_degrees();
        let dx = dy / recipe.center_lat_deg.to_radians().cos();
        let coordinates = |center: f64, step: f64| {
            (0..=recipe.cells)
                .map(|i| center + (i as f64 - recipe.cells as f64 * 0.5) * step)
                .collect::<Vec<_>>()
        };
        let mut grid = HeightGrid::regional(
            coordinates(recipe.center_lat_deg, dy),
            coordinates(recipe.center_lon_deg, dx),
            world.radius_m,
        );
        let mut runoff = grid.h.clone();
        let cutoff = recipe.spacing_m * 2.0;
        for (r, row) in grid.h.iter_mut().enumerate() {
            for (c, h) in row.iter_mut().enumerate() {
                let dir = crate::sphere::dir_from_latlon(grid.lats[r], grid.lons[c]);
                let sample = field.sample_surface(dir, cutoff);
                *h = sample.height_m;
                runoff[r][c] = field
                    .params
                    .surface_climate
                    .runoff_m_yr(sample.moisture01, sample.temperature_k);
            }
        }
        let source = grid.h.clone();
        let exported = crate::erosion::erode_runoff_budget(&mut grid, &runoff, recipe.runoff)?;
        let displacement: Vec<Vec<_>> = grid
            .h
            .iter()
            .zip(&source)
            .map(|(final_row, original)| {
                final_row.iter().zip(original).map(|(a, b)| a - b).collect()
            })
            .collect();
        let net_volume: f64 = displacement
            .iter()
            .enumerate()
            .map(|(r, row)| row.iter().sum::<f64>() * crate::terrain_fields::cell_area_m2(&grid, r))
            .sum();
        patches.push(RegionalErosion {
            recipe: recipe.clone(),
            lats: grid.lats,
            lons: grid.lons,
            source_min_wavelength_m: cutoff,
            source_heights_m: source,
            displacement_m: displacement,
            exported_sediment_m3: exported,
            sediment_volume_error_m3: net_volume + exported,
        });
    }
    world.schema = 4;
    world.regional_erosion = patches;
    world.validate()
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FrozenSource {
    /// Deterministic serialized field contract, not a process-specific hash.
    pub signature: String,
    pub heights_m: Vec<Vec<f64>>,
    pub min_wavelength_m: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlantInstance {
    pub kind: crate::scatter::ScatterKind,
    pub direction: [f64; 3],
    pub ground_height_m: f64,
    pub scale: f64,
    pub yaw_rad: f64,
}

/// Indexed prototype geometry, metre-local and renderer-neutral. Consumers
/// upload it through the existing native mesh path; no runtime plant generation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlantMesh {
    pub kind: crate::scatter::ScatterKind,
    pub positions: Vec<[f32; 3]>,
    pub normals: Vec<[f32; 3]>,
    pub indices: Vec<u32>,
    pub albedo_srgb: [f32; 3],
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FrozenTile {
    pub key: TileKey,
    /// Existing compact native RCBT HeightPage wire format, 33x33 vertices.
    pub height_page: Vec<u8>,
    /// Existing 128x128 RGBA material-page contract, including sampling border.
    pub material_rgba: Vec<u8>,
}

pub fn compile(field: &PlanetField, recipe: OfflineRecipe) -> Result<FrozenSurface, String> {
    recipe.validate()?;
    if field.has_frozen_erosion() {
        return Err("offline compilation requires an uneroded source field".into());
    }
    let rows = (180.0 / recipe.grid_step_deg).round() as usize + 1;
    let cols = 2 * (rows - 1);
    let lats: Vec<_> = (0..rows)
        .map(|r| -90.0 + r as f64 * recipe.grid_step_deg)
        .collect();
    let lons: Vec<_> = (0..cols)
        .map(|c| -180.0 + c as f64 * recipe.grid_step_deg)
        .collect();
    let mut grid = HeightGrid::new(lats.clone(), lons.clone(), field.params.radius_m);
    let mut runoff = vec![vec![0.0; cols]; rows];
    for r in 0..rows {
        for c in 0..cols {
            let dir = crate::sphere::dir_from_latlon(lats[r], lons[c]);
            let sample = field.sample_surface(dir, 32.0);
            grid.h[r][c] = sample.height_m;
            // Static climate proxy, not measured rain: moist land supplies
            // runoff, cold ground suppresses liquid flow. All units explicit.
            runoff[r][c] = field
                .params
                .surface_climate
                .runoff_m_yr(sample.moisture01, sample.temperature_k);
        }
    }
    let initial_volume = volume(&grid);
    let source = FrozenSource {
        signature: field.source_signature()?,
        heights_m: grid.h.clone(),
        min_wavelength_m: 32.0,
    };
    crate::erosion::erode_runoff(&mut grid, &runoff, recipe.runoff)?;
    finish_compile(field, recipe, grid, source, initial_volume)
}

fn volume(grid: &HeightGrid) -> f64 {
    grid.h
        .iter()
        .enumerate()
        .map(|(r, row)| row.iter().sum::<f64>() * crate::terrain_fields::cell_area_m2(grid, r))
        .sum()
}

fn bake_native_tiles(world: &FrozenSurface) -> Result<Vec<FrozenTile>, String> {
    let mut keys: std::collections::BTreeSet<_> = (0..6).map(TileKey::root).collect();
    for tile in &world.recipe.vegetation_tiles {
        let mut next = Some(*tile);
        while let Some(key) = next {
            keys.insert(key);
            next = key.parent();
        }
    }
    if keys.len() > 1024 {
        return Err("offline native-page budget exceeds 1024 tiles".into());
    }
    keys.into_iter()
        .map(|key| {
            let mut heights = Vec::with_capacity(33 * 33);
            for y in 0..33 {
                for x in 0..33 {
                    heights.push(
                        world
                            .height_m(key.direction(x as f64 / 32.0, y as f64 / 32.0))
                            .max(0.0),
                    );
                }
            }
            let height_page = thessa_rcbt_core::HeightPage::bake(&heights, 33, 0.25)
                .map_err(|e| format!("offline native height page: {e}"))?
                .to_bytes();
            let mut material_rgba = Vec::with_capacity(128 * 128 * 4);
            for y in 0..128 {
                for x in 0..128 {
                    material_rgba.extend(world.material_at(
                        key.direction((x as f64 - 1.0) / 125.0, (y as f64 - 1.0) / 125.0),
                    ));
                }
            }
            Ok(FrozenTile {
                key,
                height_page,
                material_rgba,
            })
        })
        .collect()
}

fn finish_compile(
    field: &PlanetField,
    recipe: OfflineRecipe,
    grid: HeightGrid,
    source: FrozenSource,
    initial_volume: f64,
) -> Result<FrozenSurface, String> {
    // Final terrain controls the ocean supply and mountain barriers. Do not
    // retain a stale climate mask from the pre-erosion DEM.
    let moisture = crate::climate::moisture_grid(&grid, field.params.surface_climate);
    let temperatures: Vec<Vec<f64>> = grid
        .lats
        .iter()
        .enumerate()
        .map(|(r, lat)| {
            grid.lons
                .iter()
                .enumerate()
                .map(|(c, lon)| {
                    let dir = crate::sphere::dir_from_latlon(*lat, *lon);
                    let flux = field.sample_surface(dir, 32.0).geothermal_flux_w_m2;
                    field.temperature_for_height(dir, grid.h[r][c], flux)
                })
                .collect()
        })
        .collect();
    let catchments = crate::terrain_fields::flow_catchment_area_m2(&grid);
    let rivers = if field.params.hydrology.route_rivers_downhill {
        crate::hydro::river_reaches(
            &grid,
            &catchments,
            field.params.hydrology.river_min_catchment_area_m2,
        )?
    } else {
        Vec::new()
    };
    let lakes = crate::hydro::lake_basins_with_balance(
        &grid,
        &moisture,
        &temperatures,
        field.params.surface_climate,
    )?
    .into_iter()
    .filter(|b| {
        if b.salt_flat {
            field.params.hydrology.closed_basin_salt_flats
        } else {
            field.params.hydrology.allow_lakes
        }
    })
    .collect();
    let mut world = FrozenSurface {
        schema: 3,
        source: Some(source),
        seed: field.params.seed,
        radius_m: grid.datum_radius_m,
        shore_segments: shoreline(&grid),
        rivers,
        lakes,
        sediment_volume_error_m3: volume(&grid) - initial_volume,
        regional_erosion: Vec::new(),
        lats: grid.lats,
        lons: grid.lons,
        heights_m: grid.h,
        material_rgba: Vec::new(),
        ecological_cover: Vec::new(),
        vegetation: Vec::new(),
        prototypes: plant_prototypes(),
        tiles: Vec::new(),
        recipe,
    };
    (world.material_rgba, world.ecological_cover) =
        bake_materials(field, &world, &catchments, &moisture);
    world.vegetation = bake_plants(field, &world);
    world.tiles = bake_native_tiles(&world)?;
    Ok(world)
}

impl FrozenSurface {
    fn cover_at(&self, dir: [f64; 3]) -> crate::ecology::VegetationCover {
        let (lat, lon) = crate::sphere::latlon_from_dir(dir);
        let y = ((lat + 90.0) / self.recipe.grid_step_deg).clamp(0.0, (self.lats.len() - 1) as f64);
        let x = (lon + 180.0).rem_euclid(360.0) / self.recipe.grid_step_deg;
        let (r, c) = (y.floor() as usize, x.floor() as usize);
        let nr = (r + 1).min(self.lats.len() - 1);
        let nc = (c + 1) % self.lons.len();
        let values = [
            self.ecological_cover[r][c],
            self.ecological_cover[r][nc],
            self.ecological_cover[nr][c],
            self.ecological_cover[nr][nc],
        ];
        let (u, v) = (x.fract(), y.fract());
        let weights = [(1.0 - u) * (1.0 - v), u * (1.0 - v), (1.0 - u) * v, u * v];
        crate::ecology::VegetationCover {
            ground01: values
                .iter()
                .zip(weights)
                .map(|(s, w)| s.ground01 * w)
                .sum(),
            canopy01: values
                .iter()
                .zip(weights)
                .map(|(s, w)| s.canopy01 * w)
                .sum(),
            reeds01: values.iter().zip(weights).map(|(s, w)| s.reeds01 * w).sum(),
        }
    }

    /// Sampling stored channels only. RGB interpolation is in linear light;
    /// roughness is linear. This is texture filtering, not material synthesis.
    pub fn material_at(&self, direction: [f64; 3]) -> [u8; 4] {
        let (lat, lon) = crate::sphere::latlon_from_dir(direction);
        let y = ((lat + 90.0) / self.recipe.grid_step_deg).clamp(0.0, (self.lats.len() - 1) as f64);
        let x = (lon + 180.0).rem_euclid(360.0) / self.recipe.grid_step_deg;
        let (r, c) = (y.floor() as usize, x.floor() as usize);
        let nr = (r + 1).min(self.lats.len() - 1);
        let nc = (c + 1) % self.lons.len();
        let samples = [
            self.material_rgba[r][c],
            self.material_rgba[r][nc],
            self.material_rgba[nr][c],
            self.material_rgba[nr][nc],
        ];
        let (u, v) = (x.fract(), y.fract());
        let weights = [(1.0 - u) * (1.0 - v), u * (1.0 - v), (1.0 - u) * v, u * v];
        std::array::from_fn(|ch| {
            let sum: f64 = samples
                .iter()
                .zip(weights)
                .map(|(s, w)| {
                    let value = s[ch] as f64 / 255.0;
                    w * if ch < 3 {
                        if value <= 0.04045 {
                            value / 12.92
                        } else {
                            ((value + 0.055) / 1.055).powf(2.4)
                        }
                    } else {
                        value
                    }
                })
                .sum();
            let encoded = if ch < 3 {
                if sum <= 0.0031308 {
                    sum * 12.92
                } else {
                    1.055 * sum.powf(1.0 / 2.4) - 0.055
                }
            } else {
                sum
            };
            (encoded.clamp(0.0, 1.0) * 255.0).round() as u8
        })
    }

    pub fn validate(&self) -> Result<(), String> {
        self.recipe.validate()?;
        let rows = (180.0 / self.recipe.grid_step_deg).round() as usize + 1;
        let cols = 2 * (rows - 1);
        if !matches!(self.schema, 2..=4)
            || !self.radius_m.is_finite()
            || self.radius_m <= 0.0
            || self.lats.len() != rows
            || self.lons.len() != cols
            || self.heights_m.len() != rows
            || self
                .heights_m
                .iter()
                .any(|row| row.len() != cols || row.iter().any(|v| !v.is_finite()))
        {
            return Err("invalid frozen surface schema, dimensions or heights".into());
        }
        if self.schema >= 3 && self.source.is_none() {
            return Err("schema 3 requires the pre-erosion source contract".into());
        }
        if self.regional_erosion.len() > 16
            || (self.schema < 4 && !self.regional_erosion.is_empty())
        {
            return Err("regional erosion requires schema 4 and at most 16 patches".into());
        }
        for patch in &self.regional_erosion {
            patch.validate(self.radius_m)?;
        }
        for (i, a) in self.regional_erosion.iter().enumerate() {
            for b in &self.regional_erosion[..i] {
                if a.recipe.id == b.recipe.id
                    || regional_domains_overlap(&a.recipe, &b.recipe, self.radius_m)
                {
                    return Err(
                        "regional erosion domains must have unique ids and disjoint context aprons"
                            .into(),
                    );
                }
            }
        }
        if let Some(source) = &self.source
            && (source.signature.is_empty()
                || !source.min_wavelength_m.is_finite()
                || source.min_wavelength_m < 1.0
                || source.heights_m.len() != rows
                || source
                    .heights_m
                    .iter()
                    .any(|row| row.len() != cols || row.iter().any(|h| !h.is_finite())))
        {
            return Err("invalid frozen source reference".into());
        }
        if self.material_rgba.len() != rows
            || self.material_rgba.iter().any(|row| row.len() != cols)
        {
            return Err("frozen materials must match final terrain dimensions".into());
        }
        if self.ecological_cover.len() != rows
            || self.ecological_cover.iter().any(|row| {
                row.len() != cols
                    || row.iter().any(|c| {
                        !c.total().is_finite()
                            || c.total() > 1.0 + 1e-12
                            || c.ground01 < 0.0
                            || c.canopy01 < 0.0
                            || c.reeds01 < 0.0
                    })
            })
        {
            return Err("invalid frozen ecological coverage".into());
        }
        for (i, lat) in self.lats.iter().enumerate() {
            if !lat.is_finite() || (*lat + 90.0 - i as f64 * self.recipe.grid_step_deg).abs() > 1e-8
            {
                return Err("frozen latitudes do not match the declared grid".into());
            }
        }
        for (i, lon) in self.lons.iter().enumerate() {
            if !lon.is_finite()
                || (*lon + 180.0 - i as f64 * self.recipe.grid_step_deg).abs() > 1e-8
            {
                return Err("frozen longitudes do not match the declared grid".into());
            }
        }
        if !self.sediment_volume_error_m3.is_finite()
            || self.prototypes.len() != 3
            || self.vegetation.len() > 256 * 4000
        {
            return Err("invalid frozen geometry budget or sediment counter".into());
        }
        for mesh in &self.prototypes {
            validate_mesh(mesh)?;
        }
        if self.tiles.len() > 1024 {
            return Err("frozen native-page budget exceeded".into());
        }
        for tile in &self.tiles {
            if tile.key.face >= 6
                || tile.key.level > 20
                || tile.key.x >= 1u32 << tile.key.level
                || tile.key.y >= 1u32 << tile.key.level
                || tile.material_rgba.len() != 128 * 128 * 4
            {
                return Err("invalid frozen native page address/layout".into());
            }
            thessa_rcbt_core::HeightPage::from_bytes(&tile.height_page)
                .map_err(|e| format!("invalid frozen native height page: {e}"))?;
        }
        for reach in &self.rivers {
            let [r, c] = reach.source_cell;
            let [nr, nc] = reach.receiver_cell;
            if r >= rows
                || nr >= rows
                || c >= cols
                || nc >= cols
                || reach.start_height_m != self.heights_m[r][c]
                || reach.end_height_m != self.heights_m[nr][nc]
                || reach.end_height_m >= reach.start_height_m
                || !unit_dir(reach.start_dir)
                || !unit_dir(reach.end_dir)
            {
                return Err("frozen river does not match final terrain".into());
            }
        }
        for lake in &self.lakes {
            if !lake.surface_height_m.is_finite()
                || !lake.area_m2.is_finite()
                || lake.area_m2 <= 0.0
                || !lake.capacity_m3.is_finite()
                || lake.capacity_m3 < 0.0
                || lake.cells.is_empty()
                || !lake.aridity01.is_finite()
                || !(0.0..=1.0).contains(&lake.aridity01)
                || lake.cells.iter().any(|[r, c]| *r >= rows || *c >= cols)
                || lake.inflow_m3_yr.is_some() != lake.evaporation_m3_yr.is_some()
                || [lake.inflow_m3_yr, lake.evaporation_m3_yr]
                    .into_iter()
                    .flatten()
                    .any(|v| !v.is_finite() || v < 0.0)
            {
                return Err("invalid frozen lake geometry or annual water balance".into());
            }
        }
        if self
            .shore_segments
            .iter()
            .flatten()
            .any(|dir| !unit_dir(*dir))
        {
            return Err("invalid frozen shore direction".into());
        }
        for plant in &self.vegetation {
            if !unit_dir(plant.direction)
                || !plant.ground_height_m.is_finite()
                || !plant.scale.is_finite()
                || plant.scale <= 0.0
                || !plant.yaw_rad.is_finite()
                || !self.prototypes.iter().any(|m| m.kind == plant.kind)
            {
                return Err("invalid frozen plant instance".into());
            }
        }
        Ok(())
    }

    pub fn write(&self, path: &std::path::Path) -> Result<(), String> {
        self.validate()?;
        let mut partial_name = path.as_os_str().to_os_string();
        partial_name.push(".partial");
        let partial = std::path::PathBuf::from(partial_name);
        let file = std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&partial)
            .map_err(|e| e.to_string())?;
        let mut encoder = flate2::write::GzEncoder::new(
            std::io::BufWriter::new(file),
            flate2::Compression::default(),
        );
        let result = (|| {
            use std::io::Write;
            serde_json::to_writer(&mut encoder, self).map_err(|e| e.to_string())?;
            encoder
                .finish()
                .map_err(|e| e.to_string())?
                .flush()
                .map_err(|e| e.to_string())?;
            // Atomic publication, and unlike rename this never overwrites a
            // completed artifact. The partial file is owned by this invocation.
            std::fs::hard_link(&partial, path).map_err(|e| e.to_string())
        })();
        let _ = std::fs::remove_file(partial);
        result
    }

    /// Load frozen data only; no PlanetField, noise or generation is required.
    pub fn load(path: &std::path::Path) -> Result<Self, String> {
        let file = std::fs::File::open(path).map_err(|e| e.to_string())?;
        Self::read_gzip(std::io::BufReader::new(file))
    }

    /// Load the same validated wire format from bundled bytes or a stream.
    /// The decoded-size limit is identical to the filesystem loader.
    pub fn read_gzip(reader: impl std::io::Read) -> Result<Self, String> {
        use std::io::Read;
        let reader = flate2::read::GzDecoder::new(reader);
        let mut bytes = Vec::new();
        reader
            .take(512 * 1024 * 1024 + 1)
            .read_to_end(&mut bytes)
            .map_err(|e| e.to_string())?;
        if bytes.len() > 512 * 1024 * 1024 {
            return Err("frozen surface exceeds 512 MiB decoded limit".into());
        }
        let result: Self = serde_json::from_slice(&bytes).map_err(|e| e.to_string())?;
        result.validate()?;
        Ok(result)
    }

    /// Read-only periodic interpolation of frozen terrain. No erosion or RNG.
    pub fn height_m(&self, direction: [f64; 3]) -> f64 {
        let (lat, lon) = crate::sphere::latlon_from_dir(direction);
        let y = ((lat + 90.0) / self.recipe.grid_step_deg).clamp(0.0, (self.lats.len() - 1) as f64);
        let x = (lon + 180.0).rem_euclid(360.0) / self.recipe.grid_step_deg;
        let (r, c) = (y.floor() as usize, x.floor() as usize);
        let nr = (r + 1).min(self.lats.len() - 1);
        let nc = (c + 1) % self.lons.len();
        let (u, v) = (x.fract(), y.fract());
        let a = self.heights_m[r][c];
        let d = self.heights_m[nr][nc];
        if v <= u {
            a * (1.0 - u) + self.heights_m[r][nc] * (u - v) + d * v
        } else {
            a * (1.0 - v) + d * u + self.heights_m[nr][c] * (v - u)
        }
    }
}

/// Sea-level intersections of the frozen triangulated DEM, not random coast
/// decoration. The same diagonal is used by read-only height sampling.
fn shoreline(grid: &HeightGrid) -> Vec<[[f64; 3]; 2]> {
    let mut segments = Vec::new();
    for r in 0..grid.rows() - 1 {
        for c in 0..grid.cols() {
            let nc = (c + 1) % grid.cols();
            let lon = grid.lons[c];
            let next_lon = if nc == 0 {
                grid.lons[0] + 360.0
            } else {
                grid.lons[nc]
            };
            let a = [grid.lats[r], lon, grid.h[r][c]];
            let b = [grid.lats[r], next_lon, grid.h[r][nc]];
            let d = [grid.lats[r + 1], next_lon, grid.h[r + 1][nc]];
            let e = [grid.lats[r + 1], lon, grid.h[r + 1][c]];
            for tri in [[a, b, d], [a, d, e]] {
                let mut crossings = Vec::new();
                for i in 0..3 {
                    let (p, q) = (tri[i], tri[(i + 1) % 3]);
                    if (p[2] < 0.0) != (q[2] < 0.0) {
                        let t = p[2] / (p[2] - q[2]);
                        crossings.push(crate::sphere::dir_from_latlon(
                            p[0] + (q[0] - p[0]) * t,
                            p[1] + (q[1] - p[1]) * t,
                        ));
                    }
                }
                if crossings.len() == 2 {
                    segments.push([crossings[0], crossings[1]]);
                }
            }
        }
    }
    segments
}

fn bake_plants(field: &PlanetField, world: &FrozenSurface) -> Vec<PlantInstance> {
    let mut tiles = world.recipe.vegetation_tiles.clone();
    tiles.sort();
    tiles.dedup();
    let mut plants = Vec::new();
    for tile in tiles {
        let span = tile.span_m(world.radius_m);
        let center = field.sample_surface(tile.direction(0.5, 0.5), 32.0);
        let ctx = crate::scatter::ScatterContext {
            planet_seed: world.seed,
            tile_id: crate::lod::cbt_node_for_tile(tile).unwrap().id(),
            biome: center.biome,
            geology: center.geology,
            slope01: 0.0,
            rocky01: 0.0,
            tile_size_m: span,
        };
        // Reuse the existing placement kernel. Maximum cover creates candidate
        // locations; each location is then rejected using its own final terrain.
        let candidates = crate::scatter::scatter_vegetation(
            ctx,
            crate::ecology::VegetationCover {
                ground01: 1.0,
                canopy01: 0.0,
                reeds01: 0.0,
            },
        );
        for (i, candidate) in candidates.into_iter().enumerate() {
            let dir = tile.direction(candidate.x_m / span, candidate.y_m / span);
            let ground_height = world.height_m(dir);
            if inside_lake(world, dir, ground_height) {
                continue;
            }
            let cover = world.cover_at(dir);
            let selection = crate::rng::hash01(world.seed ^ ctx.tile_id, 450, i as i64, 0);
            if selection >= cover.total() || ground_height < 2.0 || frozen_slope(world, dir) > 0.25
            {
                continue;
            }
            let kind = if selection < cover.reeds01 {
                crate::scatter::ScatterKind::ReedClump
            } else if selection < cover.reeds01 + cover.canopy01 {
                crate::scatter::ScatterKind::LycopsidTree
            } else {
                crate::scatter::ScatterKind::FernClump
            };
            plants.push(PlantInstance {
                kind,
                direction: dir,
                ground_height_m: ground_height,
                scale: candidate.scale,
                yaw_rad: candidate.yaw_rad,
            });
        }
    }
    plants
}

type MaterialAndCover = (Vec<Vec<[u8; 4]>>, Vec<Vec<crate::ecology::VegetationCover>>);

fn bake_materials(
    field: &PlanetField,
    world: &FrozenSurface,
    catchments: &[Vec<f64>],
    moisture: &[Vec<f64>],
) -> MaterialAndCover {
    let mut grid = HeightGrid::new(world.lats.clone(), world.lons.clone(), world.radius_m);
    grid.h = world.heights_m.clone();
    let depth = crate::hydro::depression_depth(&grid);
    let mut salt = vec![vec![false; grid.cols()]; grid.rows()];
    for lake in &world.lakes {
        if lake.salt_flat {
            for [r, c] in &lake.cells {
                salt[*r][*c] = grid.h[*r][*c] < lake.surface_height_m;
            }
        }
    }
    world
        .heights_m
        .iter()
        .enumerate()
        .map(|(r, row)| {
            let (pixels, cover): (Vec<_>, Vec<_>) = row
                .iter()
                .enumerate()
                .map(|(c, height)| {
                    let dir = crate::sphere::dir_from_latlon(world.lats[r], world.lons[c]);
                    let mut sample = field.sample_surface(dir, 32.0);
                    sample.temperature_k =
                        field.temperature_for_height(dir, *height, sample.geothermal_flux_w_m2);
                    sample.height_m = *height;
                    sample.moisture01 = moisture[r][c];
                    sample.slope_hint = crate::terrain_fields::slope_at(&grid, r, c);
                    sample.wetland_potential01 = if *height >= 0.0 && *height < 400.0 {
                        crate::hydro::wetland_potential01(
                            sample.slope_hint,
                            depth[r][c],
                            catchments[r][c],
                            1.0 - sample.moisture01,
                        )
                    } else {
                        0.0
                    };
                    let texel_m = grid.cell_m(r).0.max(grid.cell_m(r).1);
                    let mut site = crate::field::classify_with_context(
                        &field.params,
                        &field.features,
                        world.lats[r],
                        world.lons[c],
                        *height,
                        sample.geothermal_flux_w_m2,
                    );
                    if salt[r][c] {
                        site = crate::biomes::SiteClass::new(
                            crate::biomes::Biome::SaltFlat,
                            crate::biomes::Geology::Evaporite,
                        );
                    }
                    site = crate::ecology::classify_ecological_site(
                        site,
                        *height,
                        sample.temperature_k,
                        sample.moisture01,
                        sample.wetland_potential01,
                        sample.continentality01,
                    );
                    sample.biome = site.biome;
                    sample.geology = site.geology;
                    sample.tag0 = site.tag0;
                    sample.tag1 = site.tag1;
                    let material = crate::appearance::surface_appearance_filtered(
                        field, &sample, dir, texel_m,
                    );
                    let mut rgba = [0; 4];
                    for (target, channel) in rgba[..3].iter_mut().zip(material.albedo_srgb) {
                        *target = (channel * 255.0).round() as u8;
                    }
                    rgba[3] = (material.roughness * 255.0).round() as u8;
                    (
                        rgba,
                        crate::ecology::vegetation_cover(&sample, sample.slope_hint),
                    )
                })
                .unzip();
            (pixels, cover)
        })
        .unzip()
}

fn frozen_slope(world: &FrozenSurface, dir: [f64; 3]) -> f64 {
    let (east, north, _) = crate::sphere::enu_basis(dir);
    let angle = 32.0 / world.radius_m;
    let h = world.height_m(dir);
    let delta = |axis: [f64; 3]| {
        world.height_m(std::array::from_fn(|i| {
            dir[i] * angle.cos() + axis[i] * angle.sin()
        })) - h
    };
    delta(east).hypot(delta(north)) / 32.0
}

fn inside_lake(world: &FrozenSurface, dir: [f64; 3], ground_height: f64) -> bool {
    let (lat, lon) = crate::sphere::latlon_from_dir(dir);
    let r = ((lat + 90.0) / world.recipe.grid_step_deg).round() as usize;
    let c = ((lon + 180.0).rem_euclid(360.0) / world.recipe.grid_step_deg).round() as usize
        % world.lons.len();
    world.lakes.iter().any(|lake| {
        !lake.salt_flat
            && ground_height < lake.surface_height_m
            && lake
                .cells
                .binary_search(&[r.min(world.lats.len() - 1), c])
                .is_ok()
    })
}

fn plant_prototypes() -> Vec<PlantMesh> {
    use crate::scatter::ScatterKind;
    [
        ScatterKind::FernClump,
        ScatterKind::ReedClump,
        ScatterKind::LycopsidTree,
    ]
    .into_iter()
    .map(|kind| {
        let mut mesh = PlantMesh {
            kind,
            positions: Vec::new(),
            normals: Vec::new(),
            indices: Vec::new(),
            albedo_srgb: [0.16, 0.30, 0.12],
        };
        match kind {
            ScatterKind::FernClump => {
                for i in 0..8 {
                    fern_frond(&mut mesh, i as f64 * std::f64::consts::TAU / 8.0);
                }
            }
            ScatterKind::ReedClump => {
                for i in 0..5 {
                    let a = i as f64 * 2.399963;
                    stem(
                        &mut mesh,
                        [a.cos() * 0.2, 0.0, a.sin() * 0.2],
                        0.025,
                        1.5 + i as f64 * 0.1,
                    );
                }
            }
            ScatterKind::LycopsidTree => {
                stem(&mut mesh, [0.0; 3], 0.3, 9.0);
                for i in 0..12 {
                    tree_crown(&mut mesh, i as f64 * std::f64::consts::TAU / 12.0);
                }
            }
            _ => unreachable!(),
        }
        mesh
    })
    .collect()
}

fn triangle(mesh: &mut PlantMesh, points: [[f64; 3]; 3]) {
    let ab = crate::lod::sub(points[1], points[0]);
    let ac = crate::lod::sub(points[2], points[0]);
    let n = crate::lod::normalize(crate::lod::cross(ab, ac)).map(|v| v as f32);
    let start = mesh.positions.len() as u32;
    mesh.positions.extend(points.map(|p| p.map(|v| v as f32)));
    mesh.normals.extend([n; 3]);
    mesh.indices.extend([start, start + 1, start + 2]);
}

fn unit_dir(dir: [f64; 3]) -> bool {
    dir.iter().all(|v| v.is_finite()) && (crate::lod::dot(dir, dir) - 1.0).abs() < 1e-8
}

fn validate_mesh(mesh: &PlantMesh) -> Result<(), String> {
    if mesh.positions.is_empty()
        || mesh.positions.len() > 4096
        || mesh.positions.len() != mesh.normals.len()
        || !mesh.indices.len().is_multiple_of(3)
        || mesh
            .indices
            .iter()
            .any(|i| *i as usize >= mesh.positions.len())
        || mesh.positions.iter().flatten().any(|v| !v.is_finite())
        || mesh.normals.iter().flatten().any(|v| !v.is_finite())
    {
        return Err("invalid baked indexed plant mesh".into());
    }
    Ok(())
}

fn stem(mesh: &mut PlantMesh, origin: [f64; 3], radius: f64, height: f64) {
    for i in 0..6 {
        let a = i as f64 * std::f64::consts::TAU / 6.0;
        let b = (i + 1) as f64 * std::f64::consts::TAU / 6.0;
        let p = |angle: f64, y: f64| {
            [
                origin[0] + radius * angle.cos(),
                origin[1] + y,
                origin[2] + radius * angle.sin(),
            ]
        };
        triangle(mesh, [p(a, 0.0), p(a, height), p(b, height)]);
        triangle(mesh, [p(a, 0.0), p(b, height), p(b, 0.0)]);
    }
}

fn fern_frond(mesh: &mut PlantMesh, angle: f64) {
    for i in 0..5 {
        let a = i as f64 / 5.0;
        let b = (i + 1) as f64 / 5.0;
        let p = |t: f64, side: f64| {
            let length = t * 1.2;
            let width = (t * std::f64::consts::PI).sin() * 0.15 * side;
            [
                length * angle.cos() - width * angle.sin(),
                0.1 + (t * std::f64::consts::PI).sin() * 0.6,
                length * angle.sin() + width * angle.cos(),
            ]
        };
        if i == 4 {
            triangle(mesh, [p(a, -1.0), p(b, 0.0), p(a, 1.0)]);
        } else {
            triangle(mesh, [p(a, -1.0), p(b, -1.0), p(b, 1.0)]);
            if i > 0 {
                triangle(mesh, [p(a, -1.0), p(b, 1.0), p(a, 1.0)]);
            }
        }
    }
}

fn tree_crown(mesh: &mut PlantMesh, angle: f64) {
    let p = |radius: f64, yaw: f64, y: f64| [radius * yaw.cos(), y, radius * yaw.sin()];
    triangle(
        mesh,
        [
            p(0.0, angle, 10.0),
            p(2.5, angle, 7.0),
            p(2.5, angle + std::f64::consts::TAU / 12.0, 7.0),
        ],
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn regional_refinement_reuses_parent_and_enters_every_height_path() {
        let mut world = world();
        let parent = field().with_frozen_erosion(&world).unwrap();
        let global = world.heights_m.clone();
        let recipe = RegionalErosionRecipe {
            id: "test-highland".into(),
            center_lat_deg: 38.0,
            center_lon_deg: -171.0,
            spacing_m: 256.0,
            cells: 16,
            runoff: crate::erosion::RunoffErosion {
                iterations: 8,
                years_per_iteration: 10000.0,
                ..Default::default()
            },
        };
        refine_regions(&parent, &mut world, &[recipe]).unwrap();
        assert_eq!(
            world.heights_m, global,
            "global bake must not be repeated/modified"
        );
        let refined = field().with_frozen_erosion(&world).unwrap();
        let patch = &world.regional_erosion[0];
        let mut changed = false;
        for r in 0..patch.lats.len() {
            for c in 0..patch.lons.len() {
                let dir = crate::sphere::dir_from_latlon(patch.lats[r], patch.lons[c]);
                let h = refined.height_m(dir, patch.source_min_wavelength_m);
                assert!(
                    (h - patch.source_heights_m[r][c] - patch.displacement_m[r][c]).abs() < 1e-6
                );
                let (prefix, macro_h) = refined.height_prefix_m(dir);
                let fine = refined.height_m(dir, 32.0);
                let before = parent.height_m(dir, 32.0);
                let displacement = patch.displacement_m[r][c];
                let expected = if displacement <= 0.0 {
                    before + displacement
                } else {
                    let reference = parent.height_m(dir, patch.source_min_wavelength_m);
                    let fine_relief = before - reference;
                    reference
                        + displacement
                        + fine_relief.signum() * (fine_relief.abs() - displacement).max(0.0)
                };
                assert!(
                    (fine - expected).abs() < 1e-6,
                    "finite deposition must bury, not re-add, fine source relief: r={r} c={c} d={displacement} before={before} fine={fine} expected={expected}"
                );
                assert!(
                    (refined.height_from_prefix(dir, prefix, macro_h, 32.0) - fine).abs() < 1e-8
                );
                assert_eq!(refined.sample_surface(dir, 32.0).height_m, fine);
                assert!(
                    (refined
                        .sample_surface_from_prefix(dir, prefix, macro_h, 32.0)
                        .height_m
                        - fine)
                        .abs()
                        < 1e-8
                );
                changed |= patch.displacement_m[r][c].abs() > 1e-8;
            }
        }
        assert!(changed);
        assert!((patch.sediment_volume_error_m3).abs() < 1.0);
        for (lat, lon) in [(0.0, 0.0), (38.0, -169.0)] {
            let dir = crate::sphere::dir_from_latlon(lat, lon);
            assert_eq!(refined.height_m(dir, 32.0), parent.height_m(dir, 32.0));
        }
        let mut wrong = world.clone();
        wrong.regional_erosion[0].displacement_m[0][0] = 1.0;
        assert!(wrong.validate().is_err());
        wrong = world.clone();
        wrong.regional_erosion[0].lons[0] = f64::NAN;
        assert!(wrong.validate().is_err());
        wrong = world.clone();
        let mut duplicate = wrong.regional_erosion[0].clone();
        duplicate.recipe.id = "overlap".into();
        wrong.regional_erosion.push(duplicate);
        assert!(wrong.validate().is_err());
        assert!(refine_regions(&refined, &mut world, &[]).is_err());
    }

    #[test]
    fn regional_coordinates_and_sampling_cross_dateline_without_wrap_shortcuts() {
        let mut world = world();
        let parent = field().with_frozen_erosion(&world).unwrap();
        let recipe = RegionalErosionRecipe {
            id: "dateline".into(),
            center_lat_deg: 30.0,
            center_lon_deg: 180.0,
            spacing_m: 256.0,
            cells: 16,
            runoff: crate::erosion::RunoffErosion {
                iterations: 1,
                erodibility: 0.0,
                ..Default::default()
            },
        };
        refine_regions(&parent, &mut world, &[recipe]).unwrap();
        let patch = &world.regional_erosion[0];
        assert_eq!(
            patch.displacement_at(30.0, 180.0),
            patch.displacement_at(30.0, -180.0)
        );
        assert_eq!(patch.displacement_at(30.0, 0.0), 0.0);
        assert!(
            RegionalErosionRecipe {
                center_lat_deg: 90.0,
                ..patch.recipe.clone()
            }
            .validate(world.radius_m)
            .is_err()
        );
    }

    fn field() -> PlanetField {
        let spec =
            toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml")).unwrap();
        crate::field::field_from_manifest(&crate::spec_recipe::manifest_from_spec(&spec).unwrap())
            .unwrap()
    }

    #[test]
    fn frozen_erosion_enters_all_canonical_height_paths_without_removing_fine_relief() {
        let world = world();
        let raw = field();
        let eroded = field().with_frozen_erosion(&world).unwrap();
        assert!(eroded.has_frozen_erosion());
        assert_eq!(eroded.rivers(), world.rivers);
        assert_eq!(eroded.lake_basins(), world.lakes);
        assert_eq!(eroded.hydrology_grid_step_deg(), world.recipe.grid_step_deg);
        assert_eq!(
            eroded
                .inhabited_regions
                .iter()
                .map(|r| r.population)
                .sum::<u64>(),
            150_000_000
        );
        let mut changed = 0;
        for (r, lat) in world.lats.iter().enumerate() {
            for (c, lon) in world.lons.iter().enumerate() {
                let dir = crate::sphere::dir_from_latlon(*lat, *lon);
                let correction = world.heights_m[r][c] - raw.height_m(dir, 32.0);
                changed += usize::from(correction.abs() > 1e-6);
                assert!((eroded.height_m(dir, 32.0) - world.heights_m[r][c]).abs() < 1e-8);
                let fine = eroded.height_m(dir, 2.0);
                assert!((fine - raw.height_m(dir, 2.0) - correction).abs() < 1e-8);
                let (prefix, macro_h) = eroded.height_prefix_m(dir);
                assert!((eroded.height_from_prefix(dir, prefix, macro_h, 2.0) - fine).abs() < 1e-8);
                let sample = eroded.sample_surface_from_prefix(dir, prefix, macro_h, 2.0);
                assert!((sample.height_m - fine).abs() < 1e-8);
            }
        }
        assert!(changed > 0);
        assert_eq!(eroded.sea_offset_m, raw.sea_offset_m);
        assert!(compile(&eroded, OfflineRecipe::default()).is_err());
        assert!(eroded.with_frozen_erosion(&world).is_err());
        let mut wrong = world.clone();
        wrong
            .source
            .as_mut()
            .unwrap()
            .signature
            .push_str("different source");
        assert!(field().with_frozen_erosion(&wrong).is_err());
        let mut historical = world;
        historical.schema = 2;
        historical.source = None;
        historical.validate().unwrap();
        assert!(field().with_frozen_erosion(&historical).is_err());
    }

    #[test]
    fn frozen_source_identity_is_semantic_but_still_rejects_changed_parameters() {
        let mut world = world();
        let source = world.source.as_mut().unwrap();
        let mut identity: serde_json::Value = serde_json::from_str(&source.signature).unwrap();
        // Reverse the top-level members and use different whitespace. This
        // also covers the CLI/client preserve_order feature-unification case.
        source.signature = format!(
            "{{\n{}\n}}",
            identity
                .as_object()
                .unwrap()
                .iter()
                .rev()
                .map(|(key, value)| format!("{}: {}", serde_json::to_string(key).unwrap(), value))
                .collect::<Vec<_>>()
                .join(",\n")
        );
        assert!(field().with_frozen_erosion(&world).is_ok());
        identity["params"]["radius_m"] = serde_json::json!(3_200_001.0);
        world.source.as_mut().unwrap().signature = identity.to_string();
        assert!(field().with_frozen_erosion(&world).is_err());
        assert!(FrozenSurface::read_gzip(&b"not gzip"[..]).is_err());
    }

    fn world() -> FrozenSurface {
        let spec =
            toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml")).unwrap();
        let field = crate::field::field_from_manifest(
            &crate::spec_recipe::manifest_from_spec(&spec).unwrap(),
        )
        .unwrap();
        compile(
            &field,
            OfflineRecipe {
                grid_step_deg: 10.0,
                runoff: crate::erosion::RunoffErosion {
                    iterations: 2,
                    ..Default::default()
                },
                vegetation_tiles: vec![TileKey {
                    face: 0,
                    level: 13,
                    x: 4096,
                    y: 4096,
                }],
            },
        )
        .unwrap()
    }

    #[test]
    fn baked_shore_and_rivers_agree_with_final_terrain_and_preserve_poles() {
        let w = world();
        w.validate().unwrap();
        assert!(!w.shore_segments.is_empty());
        for segment in &w.shore_segments {
            for dir in segment {
                assert!(w.height_m(*dir).abs() < 1e-7);
            }
        }
        for reach in &w.rivers {
            assert!(reach.end_height_m < reach.start_height_m);
            assert!((w.height_m(reach.start_dir) - reach.start_height_m).abs() < 1e-8);
            assert!((w.height_m(reach.end_dir) - reach.end_height_m).abs() < 1e-8);
        }
        for lat in [-90.0, 90.0] {
            let h = w.height_m(crate::sphere::dir_from_latlon(lat, 0.0));
            for lon in [-180.0, -50.0, 80.0, 180.0] {
                assert!((w.height_m(crate::sphere::dir_from_latlon(lat, lon)) - h).abs() < 1e-8);
            }
        }
    }

    #[test]
    fn frozen_load_is_generation_free_and_does_not_overwrite_completed_bakes() {
        let w = world();
        let directory = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(format!(
            "target/test-tmp/offline-roundtrip-{}",
            std::process::id()
        ));
        std::fs::create_dir_all(&directory).unwrap();
        let path = directory.join("surface.json.gz");
        w.write(&path).unwrap();
        assert!(w.write(&path).is_err());
        let loaded = FrozenSurface::load(&path).unwrap();
        let attached = field().with_frozen_erosion(&loaded).unwrap();
        assert!(attached.has_frozen_erosion());
        assert_eq!(loaded.heights_m, w.heights_m);
        assert_eq!(loaded.vegetation, w.vegetation);
        assert_eq!(loaded.prototypes, w.prototypes);
        assert_eq!(loaded.rivers, w.rivers);
        std::fs::remove_file(path).unwrap();
        std::fs::remove_dir(directory).unwrap();
    }

    #[test]
    fn baked_plants_have_valid_indexed_geometry_and_reject_broken_recipes() {
        for mesh in plant_prototypes() {
            validate_mesh(&mesh).unwrap();
            for normal in mesh.normals {
                let len = normal.iter().map(|v| v * v).sum::<f32>();
                assert!((len - 1.0).abs() < 1e-5);
            }
        }
        for step in [0.0, f64::NAN, 0.01, 1.3] {
            assert!(
                OfflineRecipe {
                    grid_step_deg: step,
                    ..Default::default()
                }
                .validate()
                .is_err()
            );
        }
        let mut w = world();
        w.lats[1] = 0.0;
        assert!(w.validate().is_err());
    }
}
