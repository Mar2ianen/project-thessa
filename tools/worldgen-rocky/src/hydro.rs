//! Terrain-driven hydrology MVP.
//!
//! - ocean: height below water datum (sea level = 0 m by construction)
//! - lakes: only inside closed depressions (deterministic fill test)
//! - rivers: steepest-descent walk on the height grid; NEVER uphill
//! - salt flats: closed dry basins (no outlet, arid hint)
//! - glaciers: handled by biome tags, they follow valleys from terrain
//!
//! Operates on a regular lat/lon grid in metres. Grid spacing is derived
//! from degrees + datum radius, never from source image pixels.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WaterClass {
    Land,
    Ocean,
    Lake,
    River,
    SaltFlat,
    Ice,
}

/// Regional depression at its spill level, not a resolved shoreline or a
/// actual water inventory. Optional annual balance proxies refine salt
/// suitability; dry basins retain the same geological geometry.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct LakeBasin {
    pub id: usize,
    pub cells: Vec<[usize; 2]>,
    pub surface_height_m: f64,
    pub max_depth_m: f64,
    pub area_m2: f64,
    pub capacity_m3: f64,
    pub aridity01: f64,
    pub salt_flat: bool,
    /// Annual proxy at spill extent, not an actual lake water inventory.
    #[serde(default)]
    pub inflow_m3_yr: Option<f64>,
    #[serde(default)]
    pub evaporation_m3_yr: Option<f64>,
}

/// Connected depressions share one level water surface. Longitude wraps.
/// Area and capacity use spherical SI cell areas, not raster-cell counts.
pub fn lake_basins(grid: &HeightGrid, arid: &[Vec<f64>]) -> Vec<LakeBasin> {
    let filled = filled_dem(grid);
    let mut visited = vec![vec![false; grid.cols()]; grid.rows()];
    let mut basins = Vec::new();
    for r in 0..grid.rows() {
        for c in 0..grid.cols() {
            if visited[r][c] || grid.h[r][c] < 0.0 || filled[r][c] <= grid.h[r][c] {
                continue;
            }
            let basin = collect_basin(grid, arid, &filled, &mut visited, r, c);
            if basin.max_depth_m > 60.0 {
                basins.push(LakeBasin {
                    id: basins.len(),
                    ..basin
                });
            }
        }
    }
    basins
}

/// Refine salt suitability using routed liquid inflow versus evaporation at
/// spill extent. Boundary inflow is counted once, not at every downstream cell.
pub fn lake_basins_with_balance(
    grid: &HeightGrid,
    moisture: &[Vec<f64>],
    temperatures_k: &[Vec<f64>],
    climate: crate::climate::SurfaceClimate,
) -> Result<Vec<LakeBasin>, String> {
    climate.validate()?;
    for (values, is_moisture) in [(moisture, true), (temperatures_k, false)] {
        if values.len() != grid.rows()
            || values.iter().any(|row| {
                row.len() != grid.cols()
                    || row.iter().any(|v| {
                        !v.is_finite()
                            || if is_moisture {
                                !(0.0..=1.0).contains(v)
                            } else {
                                *v <= 0.0
                            }
                    })
            })
        {
            return Err("lake balance needs matching finite moisture and temperature grids".into());
        }
    }
    let arid: Vec<Vec<f64>> = moisture
        .iter()
        .map(|row| row.iter().map(|m| 1.0 - m).collect())
        .collect();
    let mut basins = lake_basins(grid, &arid);
    let mut owner = vec![vec![None; grid.cols()]; grid.rows()];
    let mut source = vec![vec![0.0; grid.cols()]; grid.rows()];
    for (r, row) in source.iter_mut().enumerate() {
        let area = crate::terrain_fields::cell_area_m2(grid, r);
        for (c, flow) in row.iter_mut().enumerate() {
            if grid.h[r][c] >= 0.0 {
                *flow = area * climate.runoff_m_yr(moisture[r][c], temperatures_k[r][c]);
            }
        }
    }
    for basin in &mut basins {
        let (mut inflow, mut evaporation) = (0.0, 0.0);
        for [r, c] in &basin.cells {
            owner[*r][*c] = Some(basin.id);
            inflow += source[*r][*c];
            evaporation += crate::terrain_fields::cell_area_m2(grid, *r)
                * climate.evaporation_m_yr(temperatures_k[*r][*c]);
        }
        basin.inflow_m3_yr = Some(inflow);
        basin.evaporation_m3_yr = Some(evaporation);
    }
    let discharge = crate::terrain_fields::accumulate_downhill(grid, source, true);
    for (r, row) in discharge.iter().enumerate() {
        for (c, flow) in row.iter().enumerate() {
            if grid.h[r][c] < 0.0 {
                continue;
            }
            if let Some((nr, nc)) = crate::terrain_fields::downstream_cell(grid, r, c)
                && let Some(id) = owner[nr][nc]
                && owner[r][c] != Some(id)
            {
                *basins[id].inflow_m3_yr.as_mut().unwrap() += flow;
            }
        }
    }
    for basin in &mut basins {
        let evaporation = basin.evaporation_m3_yr.unwrap();
        basin.salt_flat = evaporation > basin.inflow_m3_yr.unwrap();
    }
    Ok(basins)
}

#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct HydrologyRecipe {
    pub allow_lakes: bool,
    pub closed_basin_salt_flats: bool,
    pub route_rivers_downhill: bool,
    pub river_min_catchment_area_m2: f64,
}

impl Default for HydrologyRecipe {
    fn default() -> Self {
        Self {
            allow_lakes: true,
            closed_basin_salt_flats: true,
            route_rivers_downhill: true,
            river_min_catchment_area_m2: 1.0e11,
        }
    }
}

impl HydrologyRecipe {
    pub fn validate(self) -> Result<(), String> {
        if !self.river_min_catchment_area_m2.is_finite() || self.river_min_catchment_area_m2 <= 0.0
        {
            return Err("river catchment threshold must be finite and positive".into());
        }
        Ok(())
    }
}

fn collect_basin(
    grid: &HeightGrid,
    arid: &[Vec<f64>],
    filled: &[Vec<f64>],
    visited: &mut [Vec<bool>],
    r: usize,
    c: usize,
) -> LakeBasin {
    let level = filled[r][c];
    let mut basin = LakeBasin {
        id: 0,
        cells: Vec::new(),
        surface_height_m: level,
        max_depth_m: 0.0,
        area_m2: 0.0,
        capacity_m3: 0.0,
        aridity01: 0.0,
        salt_flat: false,
        inflow_m3_yr: None,
        evaporation_m3_yr: None,
    };
    let mut stack = vec![(r, c)];
    visited[r][c] = true;
    while let Some((r, c)) = stack.pop() {
        let depth = level - grid.h[r][c];
        let area = crate::terrain_fields::cell_area_m2(grid, r);
        basin.cells.push([r, c]);
        basin.max_depth_m = basin.max_depth_m.max(depth);
        basin.area_m2 += area;
        basin.capacity_m3 += area * depth;
        basin.aridity01 += area * arid[r][c];
        for (nr, nc) in grid.adjacent(r, c) {
            if !visited[nr][nc]
                && grid.h[nr][nc] >= 0.0
                && filled[nr][nc] == level
                && grid.h[nr][nc] < level
            {
                visited[nr][nc] = true;
                stack.push((nr, nc));
            }
        }
    }
    basin.cells.sort_unstable();
    basin.aridity01 /= basin.area_m2;
    basin.salt_flat = basin.aridity01 > 0.55;
    basin
}

/// Coarse terrain-derived drainage edge, not a resolved river water surface.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct RiverReach {
    pub source_cell: [usize; 2],
    pub receiver_cell: [usize; 2],
    pub start_dir: [f64; 3],
    pub end_dir: [f64; 3],
    pub start_height_m: f64,
    pub end_height_m: f64,
    pub catchment_area_m2: f64,
    pub ocean_outlet: bool,
}

/// Extract connected reaches using the same receiver and catchment field as
/// wetland support. Threshold is physical area, never a random seed per cell.
pub fn river_reaches(
    grid: &HeightGrid,
    catchments: &[Vec<f64>],
    min_area_m2: f64,
) -> Result<Vec<RiverReach>, String> {
    if !min_area_m2.is_finite() || min_area_m2 <= 0.0 {
        return Err("river catchment threshold must be finite and positive".into());
    }
    if catchments.len() != grid.rows() || catchments.iter().any(|row| row.len() != grid.cols()) {
        return Err("river catchment dimensions must match the height grid".into());
    }
    let mut reaches = Vec::new();
    for (r, row) in catchments.iter().enumerate() {
        for (c, area) in row.iter().enumerate() {
            if !area.is_finite() || *area < 0.0 {
                return Err("catchment areas must be finite and nonnegative".into());
            }
            if grid.h[r][c] < 0.0 || *area < min_area_m2 {
                continue;
            }
            let Some((nr, nc)) = crate::terrain_fields::downstream_cell(grid, r, c) else {
                continue;
            };
            reaches.push(RiverReach {
                source_cell: [r, c],
                receiver_cell: [nr, nc],
                start_dir: crate::sphere::dir_from_latlon(grid.lats[r], grid.lons[c]),
                end_dir: crate::sphere::dir_from_latlon(grid.lats[nr], grid.lons[nc]),
                start_height_m: grid.h[r][c],
                end_height_m: grid.h[nr][nc],
                catchment_area_m2: *area,
                ocean_outlet: grid.h[nr][nc] < 0.0,
            });
        }
    }
    Ok(reaches)
}

/// Regional wetland suitability, not a water surface or a local flood solver.
/// Shallow retained water and convergent drainage favour saturation; steep
/// slopes, deep lakes and dry climate suppress it. Inputs are terrain-derived.
pub fn wetland_potential01(slope: f64, depression_m: f64, catchment_m2: f64, aridity: f64) -> f64 {
    let smooth = crate::appearance::smooth;
    let flat = 1.0 - smooth(0.002, 0.02, slope);
    let shallow = smooth(0.5, 5.0, depression_m) * (1.0 - smooth(30.0, 60.0, depression_m));
    let drainage = smooth(1.0e10, 1.0e11, catchment_m2);
    let wet = 1.0 - smooth(0.35, 0.65, aridity);
    flat * shallow.max(drainage) * wet
}

/// Simple height grid with geographic extent.
#[derive(Debug, Clone)]
pub struct HeightGrid {
    pub lats: Vec<f64>,
    pub lons: Vec<f64>,
    pub h: Vec<Vec<f64>>,
    pub datum_radius_m: f64,
    /// Global grids wrap longitude; regional refinement has real side edges.
    pub longitude_periodic: bool,
}

impl HeightGrid {
    pub fn new(lats: Vec<f64>, lons: Vec<f64>, datum_radius_m: f64) -> Self {
        let h = vec![vec![0.0; lons.len()]; lats.len()];
        Self {
            lats,
            lons,
            h,
            datum_radius_m,
            longitude_periodic: true,
        }
    }

    pub fn regional(lats: Vec<f64>, lons: Vec<f64>, datum_radius_m: f64) -> Self {
        let mut grid = Self::new(lats, lons, datum_radius_m);
        grid.longitude_periodic = false;
        grid
    }

    pub fn is_open_edge(&self, r: usize, c: usize) -> bool {
        !self.longitude_periodic
            && (r == 0 || c == 0 || r + 1 == self.rows() || c + 1 == self.cols())
    }

    /// Regional drainage also resolves diagonal descent, at its true distance.
    pub fn drainage_adjacent(&self, r: usize, c: usize) -> impl Iterator<Item = (usize, usize)> {
        let diagonal = [(-1isize, -1isize), (-1, 1), (1, -1), (1, 1)]
            .into_iter()
            .filter_map(move |(dr, dc)| {
                if self.longitude_periodic {
                    return None;
                }
                let (nr, nc) = (r.checked_add_signed(dr)?, c.checked_add_signed(dc)?);
                (nr < self.rows() && nc < self.cols()).then_some((nr, nc))
            });
        self.adjacent(r, c).chain(diagonal)
    }

    /// Fixed-order four-connected neighbours, without spurious regional wrap.
    pub fn adjacent(&self, r: usize, c: usize) -> impl Iterator<Item = (usize, usize)> {
        let left = if c > 0 {
            Some(c - 1)
        } else if self.longitude_periodic {
            Some(self.cols() - 1)
        } else {
            None
        };
        let right = if c + 1 < self.cols() {
            Some(c + 1)
        } else if self.longitude_periodic {
            Some(0)
        } else {
            None
        };
        [
            r.checked_sub(1).map(|nr| (nr, c)),
            (r + 1 < self.rows()).then_some((r + 1, c)),
            left.map(|nc| (r, nc)),
            right.map(|nc| (r, nc)),
        ]
        .into_iter()
        .flatten()
    }

    pub fn rows(&self) -> usize {
        self.lats.len()
    }
    pub fn cols(&self) -> usize {
        self.lons.len()
    }

    /// Cell size in metres (local approx).
    pub fn cell_m(&self, row: usize) -> (f64, f64) {
        let dlat = if self.rows() > 1 {
            (self.lats[1] - self.lats[0]).abs().to_radians() * self.datum_radius_m
        } else {
            1000.0
        };
        let dlon = if self.cols() > 1 {
            (self.lons[1] - self.lons[0]).abs().to_radians()
                * self.datum_radius_m
                * self.lats[row].to_radians().cos().max(0.05)
        } else {
            1000.0
        };
        (dlat, dlon)
    }
}

/// Classify each cell. `arid_hint01` (0 wet .. 1 arid) and `ice_hint01` come
/// from the climate recipe + latitude; the terrain decides the rest.
/// Classify each cell. `arid01` is a per-cell dryness grid (0 wet .. 1 arid),
/// typically built from continentality + moisture drivers; `ice_hint01` stays
/// global (polar extent comes from latitude + recipe).
pub fn classify_water(
    grid: &HeightGrid,
    arid01: &[Vec<f64>],
    ice_hint01: f64,
) -> Vec<Vec<WaterClass>> {
    classify_water_with_recipe(grid, arid01, ice_hint01, HydrologyRecipe::default())
}

pub fn classify_water_with_recipe(
    grid: &HeightGrid,
    arid01: &[Vec<f64>],
    ice_hint01: f64,
    recipe: HydrologyRecipe,
) -> Vec<Vec<WaterClass>> {
    let basins = lake_basins(grid, arid01);
    classify_water_with_basins(grid, &basins, ice_hint01, recipe)
}

pub(crate) fn classify_water_with_basins(
    grid: &HeightGrid,
    basins: &[LakeBasin],
    ice_hint01: f64,
    recipe: HydrologyRecipe,
) -> Vec<Vec<WaterClass>> {
    let (rows, cols) = (grid.rows(), grid.cols());
    let mut out = vec![vec![WaterClass::Land; cols]; rows];
    for (r, row) in out.iter_mut().enumerate() {
        let polar = grid.lats[r].abs() / 90.0;
        for (c, water) in row.iter_mut().enumerate() {
            let h = grid.h[r][c];
            if h < 0.0 {
                *water = WaterClass::Ocean;
            } else if polar > 0.82 || (polar > 0.6 && ice_hint01 > 0.5) {
                *water = WaterClass::Ice;
            }
        }
    }
    // One basin-wide climate decision, including shallow margins of deep lakes.
    for basin in basins {
        let class = if basin.salt_flat && recipe.closed_basin_salt_flats {
            WaterClass::SaltFlat
        } else if !basin.salt_flat && recipe.allow_lakes {
            WaterClass::Lake
        } else {
            continue;
        };
        for [r, c] in &basin.cells {
            if out[*r][*c] == WaterClass::Land {
                out[*r][*c] = class;
            }
        }
    }
    // The same SI catchment threshold and receivers as canonical river reaches.
    if recipe.route_rivers_downhill {
        let catchments = crate::terrain_fields::flow_catchment_area_m2(grid);
        for reach in river_reaches(grid, &catchments, recipe.river_min_catchment_area_m2)
            .expect("valid hydrology recipe and matching grid")
        {
            let [r, c] = reach.source_cell;
            if out[r][c] == WaterClass::Land {
                out[r][c] = WaterClass::River;
            }
        }
    }
    out
}

#[cfg(test)]
fn neighbors(rows: usize, cols: usize, r: usize, c: usize) -> Vec<(usize, usize)> {
    let mut v = Vec::with_capacity(8);
    for dr in -1i64..=1 {
        for dc in -1i64..=1 {
            if dr == 0 && dc == 0 {
                continue;
            }
            let nr = r as i64 + dr;
            // Longitude wraps (gores stitch around the globe).
            let nc = (c as i64 + dc).rem_euclid(cols as i64);
            if nr >= 0 && nr < rows as i64 {
                v.push((nr as usize, nc as usize));
            }
        }
    }
    v
}

/// Priority-flood fill (deterministic): `filled[r][c]` is the spill height.
/// Cells where `filled - h` is large sit in true closed depressions.
/// Pole rows and ocean cells are outlets; longitude wraps.
fn filled_dem(grid: &HeightGrid) -> Vec<Vec<f64>> {
    use std::collections::BinaryHeap;
    #[derive(PartialEq, Eq)]
    struct Item {
        key: i64,
        r: usize,
        c: usize,
    }
    impl Ord for Item {
        fn cmp(&self, other: &Self) -> std::cmp::Ordering {
            // Min-heap by height, then position: fully deterministic.
            other
                .key
                .cmp(&self.key)
                .then(other.r.cmp(&self.r))
                .then(other.c.cmp(&self.c))
        }
    }
    impl PartialOrd for Item {
        fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
            Some(self.cmp(other))
        }
    }
    let (rows, cols) = (grid.rows(), grid.cols());
    let mut filled = vec![vec![f64::INFINITY; cols]; rows];
    let mut visited = vec![vec![false; cols]; rows];
    let mut heap = BinaryHeap::new();
    let seed_outlet = |filled: &mut Vec<Vec<f64>>,
                       visited: &mut Vec<Vec<bool>>,
                       heap: &mut BinaryHeap<Item>,
                       r: usize,
                       c: usize| {
        filled[r][c] = grid.h[r][c];
        visited[r][c] = true;
        heap.push(Item {
            key: height_order_key(grid.h[r][c]),
            r,
            c,
        });
    };
    for c in 0..cols {
        for r in [0, rows - 1] {
            seed_outlet(&mut filled, &mut visited, &mut heap, r, c);
        }
    }
    for r in 0..rows {
        for c in 0..cols {
            if (grid.h[r][c] < 0.0 || grid.is_open_edge(r, c)) && !visited[r][c] {
                seed_outlet(&mut filled, &mut visited, &mut heap, r, c);
            }
        }
    }
    if heap.is_empty() {
        // No outlet at all (all-land test grids): fall back to global minimum.
        let mut min = (0usize, 0usize);
        for r in 0..rows {
            for c in 0..cols {
                if grid.h[r][c] < grid.h[min.0][min.1] {
                    min = (r, c);
                }
            }
        }
        seed_outlet(&mut filled, &mut visited, &mut heap, min.0, min.1);
    }
    while let Some(Item { r, c, .. }) = heap.pop() {
        for (nr, nc) in grid.adjacent(r, c) {
            if visited[nr][nc] {
                continue;
            }
            visited[nr][nc] = true;
            // A standing lake is level: do not add fictitious depth on flats.
            filled[nr][nc] = grid.h[nr][nc].max(filled[r][c]);
            heap.push(Item {
                key: height_order_key(filled[nr][nc]),
                r: nr,
                c: nc,
            });
        }
    }
    filled
}

// Preserve the full f64 height ordering (including negative elevations).
fn height_order_key(height: f64) -> i64 {
    let bits = height.to_bits() as i64;
    bits ^ (((bits >> 63) as u64 >> 1) as i64)
}

/// Depression depth in metres: how deep a cell sits below its spill point.
pub fn depression_depth(grid: &HeightGrid) -> Vec<Vec<f64>> {
    let filled = filled_dem(grid);
    grid.h
        .iter()
        .zip(filled.iter())
        .map(|(hrow, frow)| {
            hrow.iter()
                .zip(frow.iter())
                .map(|(h, f)| (f - h).max(0.0))
                .collect()
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lake_balance_counts_catchment_inflow_once_and_rejects_frozen_salt_playas() {
        let mut grid =
            HeightGrid::new(vec![-1.0, 0.0, 1.0], vec![-180.0, -90.0, 0.0, 90.0], 1000.0);
        grid.h = vec![vec![500.0; 4]; 3];
        grid.h[1] = vec![100.0, 500.0, 500.0, 200.0];
        let wet = vec![vec![1.0; 4]; 3];
        let dry = vec![vec![0.0; 4]; 3];
        let warm = vec![vec![278.0; 4]; 3];
        let cold = vec![vec![260.0; 4]; 3];
        let climate = crate::climate::SurfaceClimate::default();
        let lakes = lake_basins_with_balance(&grid, &wet, &warm, climate).unwrap();
        let lake = &lakes[0];
        assert!(!lake.salt_flat);
        // Four-neighbour routing: cap rows contribute only columns 0 and 3;
        // the middle row contributes all four (including local basin rain).
        let expected = (2.0 * crate::terrain_fields::cell_area_m2(&grid, 0)
            + 4.0 * crate::terrain_fields::cell_area_m2(&grid, 1)
            + 2.0 * crate::terrain_fields::cell_area_m2(&grid, 2))
            * climate.precipitation_m_yr;
        assert!(
            (lake.inflow_m3_yr.unwrap() - expected).abs() < expected * 1e-12,
            "inflow {} expected {expected}, cells {:?}",
            lake.inflow_m3_yr.unwrap(),
            lake.cells
        );
        assert!((lake.evaporation_m3_yr.unwrap() - lake.area_m2 * 0.6).abs() < 1e-8);
        assert!(lake_basins_with_balance(&grid, &dry, &warm, climate).unwrap()[0].salt_flat);
        assert!(!lake_basins_with_balance(&grid, &dry, &cold, climate).unwrap()[0].salt_flat);
        assert!(
            lake_basins_with_balance(&grid, &vec![vec![f64::NAN; 4]; 3], &warm, climate).is_err()
        );
    }

    #[test]
    fn level_lakes_wrap_the_seam_and_have_spherical_capacity() {
        let mut g = HeightGrid::new(vec![-2.0, 0.0, 2.0], vec![-180.0, -90.0, 0.0, 90.0], 1000.0);
        g.h = vec![vec![500.0; 4]; 3];
        g.h[1] = vec![100.0, 500.0, 500.0, 200.0];
        let wet = vec![vec![0.1; 4]; 3];
        let basins = lake_basins(&g, &wet);
        assert_eq!(basins.len(), 1);
        let lake = &basins[0];
        assert_eq!(lake.cells, vec![[1, 0], [1, 3]]);
        assert_eq!(lake.surface_height_m, 500.0);
        assert_eq!(lake.max_depth_m, 400.0);
        let area = crate::terrain_fields::cell_area_m2(&g, 1);
        assert!((lake.area_m2 - 2.0 * area).abs() < 1e-8);
        assert!((lake.capacity_m3 - 700.0 * area).abs() < 1e-6);
        assert!(!lake.salt_flat);
        assert!(lake_basins(&g, &vec![vec![0.9; 4]; 3])[0].salt_flat);
        assert_eq!(basins, lake_basins(&g, &wet));
        g.h = vec![vec![500.0; 4]; 3];
        assert!(lake_basins(&g, &wet).is_empty());
        assert!(depression_depth(&g).iter().flatten().all(|d| *d == 0.0));
    }

    #[test]
    fn reaches_reuse_downhill_receivers_and_connect_to_ocean() {
        let g = cone_grid();
        let catchments = crate::terrain_fields::flow_catchment_area_m2(&g);
        let a = river_reaches(&g, &catchments, 1.0).unwrap();
        assert_eq!(a, river_reaches(&g, &catchments, 1.0).unwrap());
        assert!(a.iter().any(|reach| reach.ocean_outlet));
        for reach in &a {
            assert!(reach.start_height_m >= 0.0 && reach.end_height_m < reach.start_height_m);
            let [r, c] = reach.source_cell;
            let [nr, nc] = reach.receiver_cell;
            assert_eq!(
                crate::terrain_fields::downstream_cell(&g, r, c),
                Some((nr, nc))
            );
            assert!(catchments[nr][nc] >= reach.catchment_area_m2);
            if !reach.ocean_outlet {
                assert!(
                    a.iter().any(|next| next.source_cell == reach.receiver_cell)
                        || crate::terrain_fields::downstream_cell(&g, nr, nc).is_none()
                );
            }
        }
        for threshold in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert!(river_reaches(&g, &catchments, threshold).is_err());
        }
        assert!(river_reaches(&g, &[], 1.0).is_err());
    }

    #[test]
    fn wetland_support_requires_water_retention_or_drainage_and_gentle_wet_ground() {
        assert_eq!(wetland_potential01(0.0, 0.0, 0.0, 0.2), 0.0);
        assert_eq!(wetland_potential01(0.0, 10.0, 0.0, 0.2), 1.0);
        assert_eq!(wetland_potential01(0.0, 0.0, 1.0e11, 0.2), 1.0);
        assert_eq!(wetland_potential01(0.1, 10.0, 1.0e11, 0.2), 0.0);
        assert_eq!(wetland_potential01(0.0, 10.0, 1.0e11, 0.9), 0.0);
        assert_eq!(wetland_potential01(0.0, 100.0, 0.0, 0.2), 0.0);
    }

    #[test]
    fn water_classification_respects_the_same_river_threshold_and_disable_switch() {
        let g = cone_grid();
        let arid = vec![vec![0.2; g.cols()]; g.rows()];
        let recipe = HydrologyRecipe {
            river_min_catchment_area_m2: 1.0,
            ..Default::default()
        };
        let water = classify_water_with_recipe(&g, &arid, 0.0, recipe);
        let reaches =
            river_reaches(&g, &crate::terrain_fields::flow_catchment_area_m2(&g), 1.0).unwrap();
        assert!(!reaches.is_empty());
        for reach in reaches {
            let [r, c] = reach.source_cell;
            assert_eq!(water[r][c], WaterClass::River);
        }
        let disabled = classify_water_with_recipe(
            &g,
            &arid,
            0.0,
            HydrologyRecipe {
                route_rivers_downhill: false,
                ..recipe
            },
        );
        assert!(disabled.iter().flatten().all(|w| *w != WaterClass::River));
    }

    fn cone_grid() -> HeightGrid {
        // 7x7 cone peak at center, ocean ring outside.
        let lats: Vec<f64> = (0..7).map(|i| 10.0 - i as f64).collect();
        let lons: Vec<f64> = (0..7).map(|i| -20.0 + i as f64).collect();
        let mut g = HeightGrid::new(lats, lons, 3_200_000.0);
        for (r, row) in g.h.iter_mut().enumerate() {
            for (c, h) in row.iter_mut().enumerate() {
                let d = ((r as f64 - 3.0).powi(2) + (c as f64 - 3.0).powi(2)).sqrt();
                *h = if d > 2.5 { -500.0 } else { 3000.0 - d * 900.0 };
            }
        }
        g
    }

    #[test]
    fn ocean_below_datum_land_above() {
        let g = cone_grid();
        let arid = vec![vec![0.3; g.cols()]; g.rows()];
        let w = classify_water(&g, &arid, 0.0);
        assert_eq!(w[0][0], WaterClass::Ocean);
        assert!(matches!(w[3][3], WaterClass::Land | WaterClass::River));
    }

    #[test]
    fn rivers_never_flow_uphill() {
        let g = cone_grid();
        let arid = vec![vec![0.3; g.cols()]; g.rows()];
        let w = classify_water(&g, &arid, 0.0);
        // Every river cell must have a strictly lower neighbor or touch water.
        for r in 0..g.rows() {
            for c in 0..g.cols() {
                if w[r][c] != WaterClass::River {
                    continue;
                }
                let h = g.h[r][c];
                let drains = neighbors(g.rows(), g.cols(), r, c)
                    .into_iter()
                    .any(|(nr, nc)| {
                        g.h[nr][nc] < h - 1e-9
                            || matches!(
                                w[nr][nc],
                                WaterClass::Ocean | WaterClass::Lake | WaterClass::River
                            )
                    });
                assert!(drains, "river at ({r},{c}) has no downhill path");
            }
        }
    }

    #[test]
    fn flat_plain_has_no_lakes() {
        let lats: Vec<f64> = (0..6).map(|i| 30.0 - i as f64).collect();
        let lons: Vec<f64> = (0..6).map(|i| i as f64 * 2.0).collect();
        let mut g = HeightGrid::new(lats, lons, 3_200_000.0);
        for row in g.h.iter_mut() {
            for h in row.iter_mut() {
                *h = 400.0;
            }
        }
        g.h[0][0] = -100.0; // one ocean outlet cell
        let arid = vec![vec![0.2; g.cols()]; g.rows()];
        let w = classify_water(&g, &arid, 0.0);
        assert!(w.iter().flatten().all(|c| !matches!(c, WaterClass::Lake)));
    }

    #[test]
    fn closed_basin_becomes_lake_or_salt() {
        // Bowl with rim above datum everywhere around.
        let lats: Vec<f64> = (0..5).map(|i| 5.0 - i as f64).collect();
        let lons: Vec<f64> = (0..5).map(|i| i as f64).collect();
        let mut g = HeightGrid::new(lats, lons, 3_200_000.0);
        for (r, row) in g.h.iter_mut().enumerate() {
            for (c, h) in row.iter_mut().enumerate() {
                let d = ((r as f64 - 2.0).powi(2) + (c as f64 - 2.0).powi(2)).sqrt();
                *h = 1500.0 + d * 400.0 - if d < 0.5 { 1400.0 } else { 0.0 };
            }
        }
        let wet_arid = vec![vec![0.1; g.cols()]; g.rows()];
        let wet = classify_water(&g, &wet_arid, 0.0);
        assert_eq!(wet[2][2], WaterClass::Lake);
        let disabled = HydrologyRecipe {
            allow_lakes: false,
            route_rivers_downhill: false,
            ..Default::default()
        };
        assert_eq!(
            classify_water_with_recipe(&g, &wet_arid, 0.0, disabled)[2][2],
            WaterClass::Land
        );
        let dry_arid = vec![vec![0.9; g.cols()]; g.rows()];
        let dry = classify_water(&g, &dry_arid, 0.0);
        assert_eq!(dry[2][2], WaterClass::SaltFlat);
        let disabled = HydrologyRecipe {
            closed_basin_salt_flats: false,
            ..disabled
        };
        assert_eq!(
            classify_water_with_recipe(&g, &dry_arid, 0.0, disabled)[2][2],
            WaterClass::Land
        );
    }
}
