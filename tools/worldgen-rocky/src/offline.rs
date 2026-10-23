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
    /// Shared scatter contract; budgets apply to the entire requested union.
    pub scatter: crate::scatter::ScatterPolicy,
}

impl Default for OfflineRecipe {
    fn default() -> Self {
        Self {
            grid_step_deg: 0.5,
            runoff: Default::default(),
            vegetation_tiles: Vec::new(),
            scatter: Default::default(),
        }
    }
}

impl OfflineRecipe {
    pub fn validate(&self) -> Result<(), String> {
        self.runoff.validate()?;
        self.scatter.validate().map_err(|e| e.to_string())?;
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
    /// Absent on historical schema-2/3/4 bakes; new bakes retain world addresses.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub id: Option<crate::scatter::ScatterId>,
    pub kind: crate::scatter::ScatterKind,
    pub direction: [f64; 3],
    pub ground_height_m: f64,
    pub scale: f64,
    pub yaw_rad: f64,
}

/// Indexed prototype geometry, metre-local and renderer-neutral, reusable by
/// offline bakes and native mesh publication; not vegetation growth or physics.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlantMesh {
    pub kind: crate::scatter::ScatterKind,
    pub positions: Vec<[f32; 3]>,
    pub normals: Vec<[f32; 3]>,
    pub indices: Vec<u32>,
    pub albedo_srgb: [f32; 3],
    /// Optional separate stem reflectance. Historical bakes are single-material.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stem: Option<PlantStem>,
}

// Bounded optical source payload, not a planet-wide instance allocation.
// The forked crown has 7,272 split vertices; no runtime tessellation is used.
const MAX_PLANT_PROTOTYPE_VERTICES: usize = 8192;

/// The prototype emitter writes stems before foliage, with separate vertices.
/// This is optical asset metadata, not wood growth, strength or collision state.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlantStem {
    pub vertex_count: u32,
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
    world.vegetation = bake_plants(&world)?;
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
                            || c.total() > 1.0 + crate::ecology::COVER_SUM_TOLERANCE
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
            || self.vegetation.len() > self.recipe.scatter.max_items
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
        if self.vegetation.iter().any(|p| p.id.is_some())
            && (self.vegetation.iter().any(|p| p.id.is_none())
                || self
                    .vegetation
                    .iter()
                    .any(|p| p.id.unwrap().0 & 1 != 1 || p.id.unwrap().cell().is_none())
                || self.vegetation.windows(2).any(|p| p[0].id >= p[1].id))
        {
            return Err("addressed frozen plants need unique ascending plant IDs".into());
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

#[cfg(test)]
#[path = "scatter_offline_tests.rs"]
mod scatter_tests;

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

fn bake_plants(world: &FrozenSurface) -> Result<Vec<PlantInstance>, String> {
    let mut tiles = world.recipe.vegetation_tiles.clone();
    tiles.sort();
    tiles.dedup();
    // Ancestors precede descendants in TileKey order. Remove overlapping query
    // windows before budgeting; do not instantiate a planet-global population.
    let mut covered = std::collections::BTreeSet::new();
    tiles.retain(|tile| {
        let mut parent = tile.parent();
        while let Some(key) = parent {
            if covered.contains(&key) {
                return false;
            }
            parent = key.parent();
        }
        covered.insert(*tile);
        true
    });
    let mut plants = Vec::new();
    let mut remaining = world.recipe.scatter;
    for tile in tiles {
        let batch = crate::scatter::scatter_region(
            world.seed,
            world.radius_m,
            tile,
            crate::scatter::ScatterPopulation::Plants,
            remaining,
            |dir| {
                let ground_height_m = world.height_m(dir);
                // Frozen ecological coverage was derived from final terrain,
                // not from this analytic source's old centre-biome sample.
                crate::scatter::ScatterSurface::plants(
                    ground_height_m,
                    frozen_slope(world, dir),
                    frozen_water_height(world, dir),
                    world.cover_at(dir),
                )
            },
        )
        .map_err(|e| e.to_string())?;
        remaining.max_candidates -= batch.candidates_visited;
        remaining.max_items -= batch.items.len();
        if batch.items.len() > plants.capacity() - plants.len() {
            let additional = batch
                .items
                .len()
                .max(plants.capacity().max(1024))
                .min(world.recipe.scatter.max_items - plants.len());
            crate::scatter::try_reserve_items(&mut plants, additional)
                .map_err(|e| e.to_string())?;
        }
        plants.extend(batch.items.into_iter().map(|item| PlantInstance {
            id: Some(item.id),
            kind: item.kind,
            direction: item.direction,
            ground_height_m: item.ground_height_m,
            scale: item.scale,
            yaw_rad: item.yaw_rad,
        }));
    }
    plants.sort_unstable_by_key(|plant| plant.id);
    Ok(plants)
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
    let delta = |axis: [f64; 3]| {
        let forward = world.height_m(std::array::from_fn(|i| {
            dir[i] * angle.cos() + axis[i] * angle.sin()
        }));
        let backward = world.height_m(std::array::from_fn(|i| {
            dir[i] * angle.cos() - axis[i] * angle.sin()
        }));
        (forward - backward) / 64.0
    };
    delta(east).hypot(delta(north))
}

fn frozen_water_height(world: &FrozenSurface, dir: [f64; 3]) -> Option<f64> {
    Some(crate::field::basin_water_height_m(
        &world.lakes,
        world.recipe.grid_step_deg,
        world.lats.len(),
        world.lons.len(),
        dir,
    ))
}

pub fn plant_prototypes() -> Vec<PlantMesh> {
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
            stem: None,
        };
        match kind {
            ScatterKind::FernClump => {
                mesh.albedo_srgb = [0.18, 0.38, 0.10];
                fern_clump(&mut mesh);
            }
            ScatterKind::ReedClump => {
                mesh.albedo_srgb = [0.33, 0.35, 0.11];
                reed_clump(&mut mesh);
            }
            ScatterKind::LycopsidTree => {
                mesh.albedo_srgb = [0.17, 0.28, 0.10];
                lycopsid_tree(&mut mesh);
            }
            _ => unreachable!(),
        }
        mesh
    })
    .collect()
}

const GOLDEN_ANGLE: f64 = 2.399_963_229_728_653;

/// Grounded fern rosette with paired pinnae along thirteen arching rachises.
fn fern_clump(mesh: &mut PlantMesh) {
    const FRONDS: usize = 13;
    // Explicit non-random per-frond variation (length, arch height, base height).
    const LENGTH_M: [f64; FRONDS] = [
        0.86, 1.04, 0.92, 1.10, 0.80, 0.98, 0.75, 1.06, 0.88, 1.00, 0.72, 0.94, 0.82,
    ];
    const HEIGHT_M: [f64; FRONDS] = [
        0.56, 0.64, 0.52, 0.60, 0.58, 0.66, 0.54, 0.62, 0.50, 0.58, 0.60, 0.55, 0.57,
    ];
    const BASE_Y_M: [f64; FRONDS] = [
        0.0, 0.02, 0.01, 0.03, 0.015, 0.005, 0.025, 0.0, 0.018, 0.008, 0.022, 0.012, 0.006,
    ];
    for i in 0..FRONDS {
        fern_frond(
            mesh,
            0.37 + i as f64 * GOLDEN_ANGLE,
            LENGTH_M[i],
            HEIGHT_M[i],
            BASE_Y_M[i],
        );
    }
}

/// Tapered reed bed: ten curved blades around a low tussock instead of
/// six-sided needles. Blade bases stay on local Y = 0.
fn reed_clump(mesh: &mut PlantMesh) {
    const STEMS: usize = 10;
    const HEIGHT_M: [f64; STEMS] = [1.72, 1.95, 1.50, 1.85, 2.00, 1.62, 1.90, 1.40, 1.78, 1.66];
    const LEAN_M: [f64; STEMS] = [0.24, 0.36, 0.18, 0.30, 0.42, 0.22, 0.34, 0.20, 0.28, 0.26];
    const BASE_R_M: [f64; STEMS] = [0.10, 0.03, 0.15, 0.06, 0.11, 0.02, 0.14, 0.08, 0.05, 0.12];
    const WIDTH_M: [f64; STEMS] = [
        0.040, 0.052, 0.038, 0.048, 0.055, 0.042, 0.050, 0.036, 0.046, 0.035,
    ];
    for i in 0..STEMS {
        reed_blade(
            mesh,
            0.9 + i as f64 * GOLDEN_ANGLE,
            BASE_R_M[i],
            LEAN_M[i],
            HEIGHT_M[i],
            WIDTH_M[i],
        );
    }
    // Six short outward blades close the base silhouette without bulk.
    for i in 0..6 {
        let (radial, lateral) = azimuth_frame(0.4 + i as f64 * std::f64::consts::TAU / 6.0);
        let nodes: [[f64; 3]; 3] = std::array::from_fn(|k| {
            let (radius, y) = [(0.02, 0.0), (0.14, 0.10), (0.26, 0.21)][k];
            offset(scaled(radial, radius), [0.0, y, 0.0])
        });
        ribbon(
            mesh,
            &nodes,
            &[0.030, 0.075, 0.009],
            lateral,
            [0.0, 1.0, 0.0],
        );
    }
}

/// Authored arborescent lycopsid proxy: two ascending forks, each dividing
/// again into two leafy terminals. Not a taxonomic reconstruction or growth law.
fn lycopsid_tree(mesh: &mut PlantMesh) {
    const TRUNK_RINGS: [(f64, f64); 6] = [
        (0.0, 0.44),
        (0.22, 0.34),
        (0.60, 0.30),
        (3.0, 0.28),
        (5.7, 0.205),
        (6.5, 0.14),
    ];
    tapered_limb(mesh, [0.0; 3], [0.0, 1.0, 0.0], 16, &TRUNK_RINGS);
    let mut terminals = Vec::with_capacity(4);
    for fork in 0..2 {
        let angle = 0.42 + fork as f64 * std::f64::consts::PI;
        let (radial, _) = azimuth_frame(angle);
        let tilt = 0.52f64;
        let axis = offset(scaled(radial, tilt.sin()), [0.0, tilt.cos(), 0.0]);
        let base = offset(scaled(radial, 0.06), [0.0, 6.26, 0.0]);
        let length = [2.10, 2.30][fork];
        tapered_limb(
            mesh,
            base,
            axis,
            12,
            &[(0.0, 0.155), (length * 0.48, 0.105), (length, 0.062)],
        );
        let joint = offset(base, scaled(axis, length));
        for split in 0..2 {
            let tip_angle = angle + [-0.64, 0.64][split];
            let (radial, _) = azimuth_frame(tip_angle);
            let tilt = 0.70f64;
            let axis = offset(scaled(radial, tilt.sin()), [0.0, tilt.cos(), 0.0]);
            tapered_limb(
                mesh,
                joint,
                axis,
                12,
                &[(0.0, 0.067), (0.42, 0.041), (0.82, 0.018)],
            );
            let terminal = offset(joint, scaled(axis, 0.82));
            tapered_limb(
                mesh,
                terminal,
                [0.0, 1.0, 0.0],
                8,
                &[(0.0, 0.018), (0.15, 0.010)],
            );
            terminals.push((terminal, tip_angle));
        }
    }
    mesh.stem = Some(PlantStem {
        vertex_count: mesh.positions.len() as u32,
        // Authored weathered stem reflectance, not a measured fossil spectrum.
        albedo_srgb: [0.29, 0.25, 0.17],
    });
    for (base, phase) in terminals {
        for i in 0..24 {
            let reach = 1.35 + 0.50 * (i as f64 * 1.71).sin().powi(2);
            // Three short internodes carry differently aged leaf cohorts; the
            // broad leaves retain real gaps and enter raster/shadows/RT alike.
            let base = offset(base, [0.0, (i % 3) as f64 * 0.075, 0.0]);
            crown_frond(mesh, phase + i as f64 * GOLDEN_ANGLE, base, reach);
        }
    }
}

/// One arching rachis with seven opposed pairs of separate tapered pinnae.
/// Gaps are geometry shared by primary raster, shadows and optional RT.
fn fern_frond(mesh: &mut PlantMesh, angle: f64, length_m: f64, height_m: f64, base_y_m: f64) {
    let (radial, lateral) = azimuth_frame(angle);
    let at = |t: f64| {
        offset(
            scaled(radial, 0.04 + t * length_m),
            [
                0.0,
                base_y_m + (t * std::f64::consts::PI * 0.8).sin() * height_m,
                0.0,
            ],
        )
    };
    let nodes: [[f64; 3]; 13] = std::array::from_fn(|i| at(i as f64 / 12.0));
    let widths = std::array::from_fn::<_, 13, _>(|i| 0.006 * (1.0 - 0.85 * i as f64 / 12.0));
    ribbon(mesh, &nodes, &widths, lateral, [0.0, 1.0, 0.0]);
    for pair in 0..7 {
        let t = 0.14 + pair as f64 * 0.12;
        let length = length_m * 0.23 * (std::f64::consts::PI * t).sin().powf(0.65);
        for side in [-1.0, 1.0] {
            let attachment = at(t);
            let nodes: [[f64; 3]; 4] = std::array::from_fn(|i| {
                let s = [0.0, 0.28, 0.65, 1.0][i];
                offset(
                    attachment,
                    offset(
                        scaled(lateral, side * length * s),
                        offset(
                            scaled(radial, length * s * 0.28),
                            [0.0, length * (std::f64::consts::PI * s).sin() * 0.12, 0.0],
                        ),
                    ),
                )
            });
            ribbon(
                mesh,
                &nodes,
                &[0.003, length * 0.24, length * 0.16, 0.0008],
                radial,
                [0.0, 1.0, 0.0],
            );
        }
    }
}

/// One tapered reed blade: wide at the base, narrow at the drooping tip.
fn reed_blade(
    mesh: &mut PlantMesh,
    angle: f64,
    base_radius: f64,
    lean_m: f64,
    height_m: f64,
    width_m: f64,
) {
    const T: [f64; 7] = [0.0, 0.17, 0.34, 0.51, 0.68, 0.84, 1.0];
    const HEIGHT_T: [f64; 7] = [0.0, 0.20, 0.40, 0.60, 0.75, 0.82, 0.60];
    const LEAN_T: [f64; 7] = [0.0, 0.05, 0.15, 0.35, 0.65, 0.95, 1.8];
    let (radial, lateral) = azimuth_frame(angle);
    let nodes: [[f64; 3]; 7] = std::array::from_fn(|i| {
        offset(
            scaled(radial, base_radius + LEAN_T[i] * lean_m),
            [0.0, HEIGHT_T[i] * height_m, 0.0],
        )
    });
    let widths = T.map(|t| width_m * (1.0 - 0.85 * t));
    // Follow the upper side continuously across the arch and drooping tip.
    ribbon(mesh, &nodes, &widths, lateral, [0.0, 1.0, 0.0]);
}

/// One arching crown frond: rises from the trunk, then droops outward. Broad
/// enough to present a real silhouette from both above and eye level.
fn crown_frond(mesh: &mut PlantMesh, angle: f64, base: [f64; 3], reach_m: f64) {
    const RADIAL: [f64; 5] = [0.0, 0.20, 0.45, 0.74, 1.0];
    const ARC: [f64; 5] = [0.0, 0.26, 0.30, 0.10, -0.38];
    const WIDTH: [f64; 5] = [0.006, 0.075, 0.105, 0.065, 0.001];
    let (radial, lateral) = azimuth_frame(angle);
    let nodes: [[f64; 3]; 5] = std::array::from_fn(|i| {
        offset(
            base,
            offset(
                scaled(radial, RADIAL[i] * reach_m),
                [0.0, ARC[i] * reach_m, 0.0],
            ),
        )
    });
    let widths = WIDTH.map(|w| w * reach_m);
    ribbon(mesh, &nodes, &widths, lateral, [0.0, 1.0, 0.0]);
}

fn offset(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|i| a[i] + b[i])
}

fn scaled(v: [f64; 3], s: f64) -> [f64; 3] {
    v.map(|c| c * s)
}

/// Horizontal unit radial and lateral directions for one azimuth about +Y.
fn azimuth_frame(angle: f64) -> ([f64; 3], [f64; 3]) {
    let (sin, cos) = angle.sin_cos();
    ([cos, 0.0, sin], [-sin, 0.0, cos])
}

/// Flat tapered strip through `nodes`. `lateral` is a unit horizontal axis and
/// `hint` selects which side of the sheet receives the derived face normals.
fn ribbon(
    mesh: &mut PlantMesh,
    nodes: &[[f64; 3]],
    half_widths: &[f64],
    lateral: [f64; 3],
    hint: [f64; 3],
) {
    debug_assert_eq!(nodes.len(), half_widths.len());
    for i in 0..nodes.len() - 1 {
        let left = offset(nodes[i], scaled(lateral, half_widths[i]));
        let right = offset(nodes[i], scaled(lateral, -half_widths[i]));
        let next_right = offset(nodes[i + 1], scaled(lateral, -half_widths[i + 1]));
        let next_left = offset(nodes[i + 1], scaled(lateral, half_widths[i + 1]));
        oriented_triangle(mesh, [left, right, next_right], hint);
        oriented_triangle(mesh, [left, next_right, next_left], hint);
    }
}

/// Tapered polygonal limb with cone-surface normals: smooth circumferential
/// shading, but deliberate axial ring creases. The terminal cap closes forks.
fn tapered_limb(
    mesh: &mut PlantMesh,
    base: [f64; 3],
    axis: [f64; 3],
    sides: usize,
    rings: &[(f64, f64)],
) {
    let axis = crate::lod::normalize(axis);
    let reference = if axis[1].abs() < 0.9 {
        [0.0, 1.0, 0.0]
    } else {
        [1.0, 0.0, 0.0]
    };
    let u = crate::lod::normalize(crate::lod::cross(reference, axis));
    let v = crate::lod::cross(axis, u);
    let at = |distance: f64, radius: f64, angle: f64| {
        offset(
            offset(base, scaled(axis, distance)),
            offset(
                scaled(u, radius * angle.cos()),
                scaled(v, radius * angle.sin()),
            ),
        )
    };
    for band in rings.windows(2) {
        let (start_d, start_r) = band[0];
        let (end_d, end_r) = band[1];
        let first = mesh.positions.len();
        for i in 0..sides {
            let a = i as f64 * std::f64::consts::TAU / sides as f64;
            let b = (i + 1) as f64 * std::f64::consts::TAU / sides as f64;
            let mid = (a + b) * 0.5;
            let hint = offset(scaled(u, mid.cos()), scaled(v, mid.sin()));
            oriented_triangle(
                mesh,
                [
                    at(start_d, start_r, a),
                    at(start_d, start_r, b),
                    at(end_d, end_r, b),
                ],
                hint,
            );
            oriented_triangle(
                mesh,
                [
                    at(start_d, start_r, a),
                    at(end_d, end_r, b),
                    at(end_d, end_r, a),
                ],
                hint,
            );
        }
        let slope = (end_r - start_r) / (end_d - start_d);
        for (point, normal) in mesh.positions[first..]
            .iter()
            .zip(&mut mesh.normals[first..])
        {
            let relative = crate::lod::sub(point.map(f64::from), base);
            let distance = crate::lod::dot(relative, axis);
            let radial = crate::lod::normalize(crate::lod::sub(relative, scaled(axis, distance)));
            *normal = crate::lod::normalize(crate::lod::sub(radial, scaled(axis, slope)))
                .map(|v| v as f32);
        }
    }
    let &(distance, radius) = rings.last().unwrap();
    let center = offset(base, scaled(axis, distance));
    for i in 0..sides {
        let a = i as f64 * std::f64::consts::TAU / sides as f64;
        let b = (i + 1) as f64 * std::f64::consts::TAU / sides as f64;
        oriented_triangle(
            mesh,
            [center, at(distance, radius, a), at(distance, radius, b)],
            axis,
        );
    }
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
        || mesh.positions.len() > MAX_PLANT_PROTOTYPE_VERTICES
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
    if let Some(stem) = &mesh.stem {
        let count = stem.vertex_count as usize;
        if count == 0
            || count >= mesh.positions.len()
            || stem
                .albedo_srgb
                .iter()
                .any(|v| !v.is_finite() || !(0.0..=1.0).contains(v))
            || mesh.indices.as_chunks::<3>().0.iter().any(|triangle| {
                let stem_corners = triangle.iter().filter(|&&i| i < stem.vertex_count).count();
                stem_corners != 0 && stem_corners != 3
            })
        {
            return Err("invalid plant stem material boundary".into());
        }
    }
    Ok(())
}

fn face_normal(points: [[f64; 3]; 3]) -> [f64; 3] {
    crate::lod::cross(
        crate::lod::sub(points[1], points[0]),
        crate::lod::sub(points[2], points[0]),
    )
}

/// Emit a triangle whose winding makes the cross-product normal agree with
/// `hint`. Normals are still derived from the emitted winding, never assigned.
fn oriented_triangle(mesh: &mut PlantMesh, mut points: [[f64; 3]; 3], hint: [f64; 3]) {
    if crate::lod::dot(face_normal(points), hint) < 0.0 {
        points.swap(1, 2);
    }
    triangle(mesh, points);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scatter::ScatterKind;

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
                scatter: Default::default(),
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
    fn plant_stem_palette_is_additive_and_never_interpolates_across_an_organ() {
        for mesh in plant_prototypes() {
            if let Some(stem) = &mesh.stem {
                assert_eq!(mesh.kind, ScatterKind::LycopsidTree);
                assert!(
                    stem.vertex_count > 0 && (stem.vertex_count as usize) < mesh.positions.len()
                );
                assert_ne!(stem.albedo_srgb, mesh.albedo_srgb);
                for triangle in mesh.indices.as_chunks::<3>().0 {
                    assert!(
                        triangle.iter().all(|&v| v < stem.vertex_count)
                            || triangle.iter().all(|&v| v >= stem.vertex_count)
                    );
                }
                let mut broken = mesh.clone();
                broken.stem.as_mut().unwrap().vertex_count = 1;
                assert!(validate_mesh(&broken).is_err());
                broken = mesh.clone();
                broken.stem.as_mut().unwrap().albedo_srgb[0] = f32::NAN;
                assert!(validate_mesh(&broken).is_err());
            } else {
                assert_ne!(mesh.kind, ScatterKind::LycopsidTree);
            }
            let value = serde_json::to_value(&mesh).unwrap();
            let roundtrip: PlantMesh = serde_json::from_value(value.clone()).unwrap();
            assert_eq!(roundtrip, mesh);
            let mut legacy = value;
            legacy.as_object_mut().unwrap().remove("stem");
            let legacy: PlantMesh = serde_json::from_value(legacy).unwrap();
            assert_eq!(legacy.stem, None);
            assert_eq!(legacy.positions, mesh.positions);
            assert_eq!(legacy.normals, mesh.normals);
            assert_eq!(legacy.indices, mesh.indices);
            assert_eq!(legacy.albedo_srgb, mesh.albedo_srgb);
            validate_mesh(&legacy).unwrap();
        }
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
        let mut oversized = plant_prototypes().remove(0);
        oversized
            .positions
            .resize(MAX_PLANT_PROTOTYPE_VERTICES + 1, [0.0; 3]);
        oversized
            .normals
            .resize(MAX_PLANT_PROTOTYPE_VERTICES + 1, [0.0, 1.0, 0.0]);
        assert!(validate_mesh(&oversized).is_err());
        let mut w = world();
        w.lats[1] = 0.0;
        assert!(w.validate().is_err());
    }

    /// Proto-type triangles as f64 world-local points plus the normal derived
    /// from their winding; independent of the stored f32 normals.
    fn prototype_triangles(mesh: &PlantMesh) -> Vec<(u32, [[f64; 3]; 3], [f64; 3])> {
        mesh.indices
            .chunks(3)
            .map(|index| {
                let points = [0, 1, 2].map(|k| mesh.positions[index[k] as usize].map(f64::from));
                let cross = face_normal(points);
                let length = crate::lod::dot(cross, cross).sqrt();
                (index[0], points, cross.map(|v| v / length))
            })
            .collect()
    }

    fn centroid(points: [[f64; 3]; 3]) -> [f64; 3] {
        scaled(
            points.iter().fold([0.0; 3], |sum, p| offset(sum, *p)),
            1.0 / 3.0,
        )
    }

    fn radial_of(point: [f64; 3]) -> [f64; 3] {
        let radial = [point[0], 0.0, point[2]];
        crate::lod::normalize(radial)
    }

    // Independent orthographic ray/triangle coverage in the ground plane.
    fn covered_from_above(mesh: &PlantMesh, x: f64, z: f64) -> bool {
        mesh.indices.as_chunks::<3>().0.iter().any(|indices| {
            let p = indices.map(|i| mesh.positions[i as usize].map(f64::from));
            let edge =
                |a: [f64; 3], b: [f64; 3]| (b[0] - a[0]) * (z - a[2]) - (b[2] - a[2]) * (x - a[0]);
            let sides = [edge(p[0], p[1]), edge(p[1], p[2]), edge(p[2], p[0])];
            sides.iter().all(|v| *v >= -1e-10) || sides.iter().all(|v| *v <= 1e-10)
        })
    }

    #[test]
    fn broad_crown_leaf_has_a_readable_lamina_and_a_tapered_tip() {
        let mut mesh = plant_prototypes().remove(2);
        mesh.positions.clear();
        mesh.normals.clear();
        mesh.indices.clear();
        mesh.stem = None;
        crown_frond(&mut mesh, 0.0, [0.0; 3], 2.0);
        // A 2 m leaf has a 42 cm lamina here: real surface, not a thin wire.
        assert!(covered_from_above(&mesh, 0.9, 0.18));
        assert!(covered_from_above(&mesh, 0.9, -0.18));
        assert!(!covered_from_above(&mesh, 0.9, 0.24));
        assert!(covered_from_above(&mesh, 1.95, 0.0));
        assert!(!covered_from_above(&mesh, 1.95, 0.04));
    }

    #[test]
    fn pinnate_fern_has_geometric_gaps_instead_of_a_serrated_solid_sheet() {
        let mut mesh = plant_prototypes().remove(0);
        mesh.positions.clear();
        mesh.normals.clear();
        mesh.indices.clear();
        fern_frond(&mut mesh, 0.0, 1.0, 0.6, 0.0);
        // Independent vertical ray/triangle test in XZ. Paired pinnae attach
        // every 0.12 m along this one-metre rachis, with true air between them.
        let covered = |x, z| covered_from_above(&mesh, x, z);
        assert!(covered(0.577, 0.095), "pinna must present real leaf area");
        assert!(
            !covered(0.625, 0.095),
            "gap must not cast a sheet shadow or opaque RT hit"
        );
        assert!(
            covered(0.625, 0.0),
            "rachis must remain connected across pinna gaps"
        );
    }

    #[test]
    fn conical_limb_normals_and_chord_error_match_the_analytic_surface() {
        let mut mesh = plant_prototypes().remove(2);
        mesh.positions.clear();
        mesh.normals.clear();
        mesh.indices.clear();
        mesh.stem = None;
        tapered_limb(
            &mut mesh,
            [0.0; 3],
            [0.0, 1.0, 0.0],
            16,
            &[(0.0, 0.4), (2.0, 0.2)],
        );
        // First 32 triangles are the sides, then 16 close the tip.
        for (points, normals) in mesh.positions[..96]
            .as_chunks::<3>()
            .0
            .iter()
            .zip(mesh.normals[..96].as_chunks::<3>().0)
        {
            for (p, n) in points.iter().zip(normals) {
                let p = p.map(f64::from);
                let r = p[0].hypot(p[2]);
                let expected = crate::lod::normalize([p[0] / r, 0.1, p[2] / r]);
                let error = crate::lod::sub(n.map(f64::from), expected);
                assert!(crate::lod::dot(error, error).sqrt() < 2e-7);
            }
            let middle = centroid(points.map(|p| p.map(f64::from)));
            let analytic_radius = 0.4 - middle[1] * 0.1;
            let chord_error = analytic_radius - middle[0].hypot(middle[2]);
            assert!((-1e-7..=0.008).contains(&chord_error));
        }
        let reference = mesh.clone();
        for angle in [-0.3f64, 0.3] {
            let (sin, cos) = angle.sin_cos();
            let rotate = |v: [f64; 3]| [v[0] * cos + v[1] * sin, -v[0] * sin + v[1] * cos, v[2]];
            let base = [2.0, 8.0, -1.0];
            mesh.positions.clear();
            mesh.normals.clear();
            mesh.indices.clear();
            tapered_limb(
                &mut mesh,
                base,
                rotate([0.0, 1.0, 0.0]),
                16,
                &[(0.0, 0.4), (2.0, 0.2)],
            );
            for ((p, n), (rp, rn)) in mesh
                .positions
                .iter()
                .zip(&mesh.normals)
                .zip(reference.positions.iter().zip(&reference.normals))
            {
                let expected = offset(base, rotate(rp.map(f64::from)));
                let position_error = crate::lod::sub(p.map(f64::from), expected);
                assert!(crate::lod::dot(position_error, position_error).sqrt() < 1e-6);
                let normal_error = crate::lod::sub(n.map(f64::from), rotate(rn.map(f64::from)));
                assert!(crate::lod::dot(normal_error, normal_error).sqrt() < 3e-6);
            }
        }
    }

    #[test]
    fn plant_prototypes_are_bounded_non_degenerate_and_deterministic() {
        let prototypes = plant_prototypes();
        assert_eq!(prototypes.len(), 3);
        // No RNG or external state: repeated production is bit-identical.
        assert_eq!(plant_prototypes(), prototypes);
        for mesh in &prototypes {
            validate_mesh(mesh).unwrap();
            assert_eq!(mesh.normals.len(), mesh.positions.len());
            assert!(mesh.positions.len() <= MAX_PLANT_PROTOTYPE_VERTICES);
            let triangles = mesh.indices.len() / 3;
            let (minimum, ceiling) = match mesh.kind {
                ScatterKind::FernClump => (1400, 1450),
                ScatterKind::ReedClump => (120, 170),
                ScatterKind::LycopsidTree => (1400, 1450),
                kind => panic!("unexpected prototype kind {kind:?}"),
            };
            assert!(
                (minimum..=ceiling).contains(&triangles),
                "{:?} has {triangles} triangles outside the pinned {minimum}..={ceiling}",
                mesh.kind
            );
            let foliage_area: f64 = prototype_triangles(mesh)
                .into_iter()
                .filter(|(first, _, _)| mesh.stem.as_ref().is_none_or(|s| *first >= s.vertex_count))
                .map(|(_, points, _)| {
                    let n = face_normal(points);
                    crate::lod::dot(n, n).sqrt() * 0.5
                })
                .sum();
            if mesh.kind == ScatterKind::LycopsidTree {
                assert!(
                    (35.0..50.0).contains(&foliage_area),
                    "authored crown area: {foliage_area} m²"
                );
            }
            for (first, points, normal) in prototype_triangles(mesh) {
                assert!(
                    points.iter().flatten().all(|v| v.is_finite()),
                    "{:?} has a non-finite vertex",
                    mesh.kind
                );
                assert!(
                    (crate::lod::dot(normal, normal) - 1.0).abs() < 1e-9,
                    "{:?} triangle normal is not unit length",
                    mesh.kind
                );
                // Twice the area in m²; centimetre twig caps and small pinnae
                // are real geometry, but collapsed/duplicate vertices are not.
                let double_area = crate::lod::dot(face_normal(points), normal);
                let minimum = if mesh.kind == ScatterKind::ReedClump {
                    2e-4
                } else {
                    2e-6
                };
                assert!(
                    double_area > minimum,
                    "{:?} has a degenerate triangle (area {:.3e})",
                    mesh.kind,
                    double_area * 0.5
                );
                // Leaf sheets remain face-normal shaded. Stem normals describe
                // smooth cone sides (>=8 sides), never inward or zero vectors.
                let stem = mesh.stem.as_ref().is_some_and(|s| first < s.vertex_count);
                for index in first..first + 3 {
                    let stored = mesh.normals[index as usize].map(f64::from);
                    assert!(crate::lod::dot(stored, normal) > if stem { 0.90 } else { 1.0 - 1e-6 });
                }
            }
        }
    }

    #[test]
    fn plant_prototype_extents_and_fern_roots_stay_on_local_ground() {
        // Pinned representation limits, in metres: the fern rosette must sit on
        // local Y = 0, reeds stay in 1.2-2.0 m, forked crowns stay near 9 m.
        for mesh in plant_prototypes() {
            let min_y = mesh
                .positions
                .iter()
                .map(|p| f64::from(p[1]))
                .fold(f64::INFINITY, f64::min);
            let max_y = mesh
                .positions
                .iter()
                .map(|p| f64::from(p[1]))
                .fold(f64::NEG_INFINITY, f64::max);
            let max_reach = mesh
                .positions
                .iter()
                .map(|p| f64::from(p[0]).hypot(f64::from(p[2])))
                .fold(0.0, f64::max);
            let (min_height, max_height, max_extent) = match mesh.kind {
                ScatterKind::FernClump => (0.45, 0.75, 1.35),
                ScatterKind::ReedClump => (1.20, 2.00, 1.00),
                ScatterKind::LycopsidTree => (8.70, 10.20, 3.90),
                kind => panic!("unexpected prototype kind {kind:?}"),
            };
            // No ground offset is invented: every prototype base meets local Y=0.
            assert!(
                min_y > -0.01,
                "{:?} dips below its root plane by {min_y}",
                mesh.kind
            );
            assert!(
                min_y <= 0.02,
                "{:?} floats {min_y} m above its root plane",
                mesh.kind
            );
            assert!(
                (min_height..=max_height).contains(&max_y),
                "{:?} height {max_y} outside {min_height}..={max_height}",
                mesh.kind
            );
            assert!(
                max_reach <= max_extent,
                "{:?} reach {max_reach} exceeds {max_extent}",
                mesh.kind
            );
        }
    }

    #[test]
    fn plant_foliage_normals_face_up_and_crowns_face_outward() {
        // Organ metadata, not an arbitrary height/radius, owns the tree's
        // foliage classification now that spreading forks reach into its crown.
        for mesh in plant_prototypes() {
            let max_y = mesh
                .positions
                .iter()
                .map(|p| f64::from(p[1]))
                .fold(f64::NEG_INFINITY, f64::max);
            for (first, points, normal) in prototype_triangles(&mesh) {
                let middle = centroid(points);
                let stem = mesh.stem.as_ref().is_some_and(|s| first < s.vertex_count);
                if middle[1] > 0.4 * max_y && !stem {
                    assert!(
                        normal[1] > -0.25,
                        "{:?} upper surface faces down at {middle:?}: {normal:?}",
                        mesh.kind
                    );
                }
                if mesh.kind != ScatterKind::LycopsidTree {
                    continue;
                }
                if !stem {
                    // Arching fronds rise before they droop, so their upper
                    // surface may tilt inward near the trunk. What must never
                    // happen is an upper-side triangle facing the ground.
                    assert!(
                        normal[1] > 0.35,
                        "crown frond faces down at {middle:?}: {normal:?}"
                    );
                } else if middle[1] < 3.0 {
                    let radial = radial_of(middle);
                    assert!(
                        crate::lod::dot(normal, radial) > 0.5,
                        "trunk side faces inward at {middle:?}: {normal:?}"
                    );
                }
            }
        }
    }
}
