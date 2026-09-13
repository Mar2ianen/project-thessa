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

/// Simple height grid with geographic extent.
#[derive(Debug, Clone)]
pub struct HeightGrid {
    pub lats: Vec<f64>,
    pub lons: Vec<f64>,
    pub h: Vec<Vec<f64>>,
    pub datum_radius_m: f64,
}

impl HeightGrid {
    pub fn new(lats: Vec<f64>, lons: Vec<f64>, datum_radius_m: f64) -> Self {
        let h = vec![vec![0.0; lons.len()]; lats.len()];
        Self {
            lats,
            lons,
            h,
            datum_radius_m,
        }
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
    let (rows, cols) = (grid.rows(), grid.cols());
    // True closed depressions via one deterministic priority-flood pass.
    let depth = depression_depth(grid);
    let mut out = vec![vec![WaterClass::Land; cols]; rows];
    for r in 0..rows {
        let polar = grid.lats[r].abs() / 90.0;
        for c in 0..cols {
            let h = grid.h[r][c];
            if h < 0.0 {
                out[r][c] = WaterClass::Ocean;
            } else if polar > 0.82 || (polar > 0.6 && ice_hint01 > 0.5) {
                out[r][c] = WaterClass::Ice;
            } else if depth[r][c] > 60.0 {
                // Only substantial closed depressions hold water; shallow
                // noise pits stay land (drained as wetlands by rivers).
                // Dry + deep => salt flat, wet + deep => lake.
                if arid01[r][c] > 0.55 {
                    out[r][c] = WaterClass::SaltFlat;
                } else {
                    out[r][c] = WaterClass::Lake;
                }
            }
        }
    }
    // Rivers: steepest descent from high land cells, carved downhill only.
    for r in 0..rows {
        for c in 0..cols {
            if out[r][c] == WaterClass::Land && grid.h[r][c] > 200.0 && river_seed(grid, r, c) {
                trace_river(grid, &mut out, r, c);
            }
        }
    }
    out
}

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
            key: (grid.h[r][c] * 1000.0) as i64,
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
            if grid.h[r][c] < 0.0 && !visited[r][c] {
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
        let nb = [
            (r.wrapping_sub(1), c),
            (r + 1, c),
            (r, (c + cols - 1) % cols),
            (r, (c + 1) % cols),
        ];
        for (nr, nc) in nb {
            if nr >= rows || visited[nr][nc] {
                continue;
            }
            visited[nr][nc] = true;
            // Spill height: never below the cell itself, epsilon above pour point.
            filled[nr][nc] = grid.h[nr][nc].max(filled[r][c] + 1e-3);
            heap.push(Item {
                key: (filled[nr][nc] * 1000.0) as i64,
                r: nr,
                c: nc,
            });
        }
    }
    filled
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

fn river_seed(_grid: &HeightGrid, r: usize, c: usize) -> bool {
    // Sparse deterministic seeds on high ground (hash of coords).
    let h = (r as u64)
        .wrapping_mul(0xBF58_476D_1CE4_E5B9)
        .wrapping_add((c as u64).wrapping_mul(0x94D0_49BB_1331_11EB));
    h.is_multiple_of(37)
}

/// Walk steepest descent marking River. Stops at ocean/lake/edge/local pit.
/// By construction each step goes to a strictly lower cell: never uphill.
fn trace_river(grid: &HeightGrid, out: &mut [Vec<WaterClass>], mut r: usize, mut c: usize) {
    let (rows, cols) = (grid.rows(), grid.cols());
    for _ in 0..256 {
        match out[r][c] {
            WaterClass::Ocean | WaterClass::Lake => return,
            WaterClass::River => return,
            _ => {}
        }
        let h = grid.h[r][c];
        let mut best: Option<(usize, usize)> = None;
        let mut best_h = h;
        for (nr, nc) in neighbors(rows, cols, r, c) {
            // Deterministic tie-break: first strictly-lower in scan order wins.
            if grid.h[nr][nc] < best_h - 1e-9 {
                best_h = grid.h[nr][nc];
                best = Some((nr, nc));
            }
        }
        match best {
            Some((nr, nc)) => {
                if out[r][c] == WaterClass::Land {
                    out[r][c] = WaterClass::River;
                }
                r = nr;
                c = nc;
            }
            None => return, // local pit: river ends (endorheic), no uphill step taken
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
        let dry_arid = vec![vec![0.9; g.cols()]; g.rows()];
        let dry = classify_water(&g, &dry_arid, 0.0);
        assert_eq!(dry[2][2], WaterClass::SaltFlat);
    }
}
