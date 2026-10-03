//! Slope / curvature / drainage helpers derived from final height.
//!
//! All helpers read the baked height grid in metres. Nothing here invents
//! terrain; it only measures it for biome, scatter and preview consumers.

use crate::hydro::HeightGrid;

/// Slope magnitude 0..1+ (rise over run) at a cell.
pub fn slope_at(grid: &HeightGrid, r: usize, c: usize) -> f64 {
    let cols = grid.cols();
    let rows = grid.rows();
    let (dy_m, dx_m) = grid.cell_m(r);
    let (left, right) = if grid.longitude_periodic {
        ((c + cols - 1) % cols, (c + 1) % cols)
    } else {
        (c.saturating_sub(1), (c + 1).min(cols - 1))
    };
    let x_span = if grid.longitude_periodic {
        2.0
    } else {
        right.abs_diff(left).max(1) as f64
    };
    let hx = (grid.h[r][right] - grid.h[r][left]) / (x_span * dx_m.max(1.0));
    let r_up = r.saturating_sub(1);
    let r_dn = (r + 1).min(rows - 1);
    let y_span = if grid.longitude_periodic {
        2.0
    } else {
        r_dn.abs_diff(r_up).max(1) as f64
    };
    let hy = (grid.h[r_dn][c] - grid.h[r_up][c]) / (y_span * dy_m.max(1.0));
    (hx * hx + hy * hy).sqrt()
}

/// Mean curvature (Laplacian / cell area): >0 pit/mound, <0 ridge.
/// Scale: 1/metres. Sign convention documented for scatter use.
pub fn curvature_at(grid: &HeightGrid, r: usize, c: usize) -> f64 {
    let cols = grid.cols();
    let rows = grid.rows();
    let (dy_m, dx_m) = grid.cell_m(r);
    let h = grid.h[r][c];
    let (left, right) = if grid.longitude_periodic {
        ((c + cols - 1) % cols, (c + 1) % cols)
    } else {
        (c.saturating_sub(1), (c + 1).min(cols - 1))
    };
    let lap = (grid.h[r][right] + grid.h[r][left] - 2.0 * h) / dx_m.max(1.0).powi(2)
        + (grid.h[(r + 1).min(rows - 1)][c] + grid.h[r.saturating_sub(1)][c] - 2.0 * h)
            / dy_m.max(1.0).powi(2);
    if lap.is_finite() { lap } else { 0.0 }
}

/// Four-neighbour downhill flow accumulation (cell counts, deterministic order).
/// Each cell routes all its water to the lowest neighbour; accumulation
/// counts how many cells drain through it. Rivers = high accumulation.
pub fn flow_accumulation(grid: &HeightGrid) -> Vec<Vec<u32>> {
    accumulate_downhill(grid, vec![vec![1.0; grid.cols()]; grid.rows()], false)
        .into_iter()
        .map(|row| {
            row.into_iter()
                .map(|v| v.min(u32::MAX as f64) as u32)
                .collect()
        })
        .collect()
}

/// Land catchment area in SI, with latitude-dependent spherical cell area.
/// Ocean cells terminate drainage and do not contribute fictitious runoff.
pub fn flow_catchment_area_m2(grid: &HeightGrid) -> Vec<Vec<f64>> {
    let area = grid
        .h
        .iter()
        .enumerate()
        .map(|(r, row)| {
            let cell_area = cell_area_m2(grid, r);
            row.iter()
                .map(|h| if *h >= 0.0 { cell_area } else { 0.0 })
                .collect()
        })
        .collect();
    accumulate_downhill(grid, area, true)
}

/// Exact spherical area of a regular latitude/longitude cell, pole-clipped.
pub fn cell_area_m2(grid: &HeightGrid, row: usize) -> f64 {
    let half_lat = if grid.rows() > 1 {
        (grid.lats[1] - grid.lats[0]).abs() * 0.5
    } else {
        0.5
    };
    let dlon = if grid.cols() > 1 {
        (grid.lons[1] - grid.lons[0]).abs().to_radians()
    } else {
        1.0_f64.to_radians()
    };
    let low = (grid.lats[row] - half_lat).max(-90.0).to_radians();
    let high = (grid.lats[row] + half_lat).min(90.0).to_radians();
    grid.datum_radius_m.powi(2) * dlon * (high.sin() - low.sin())
}

/// Accumulate an extensive source (area, runoff volume/year, etc.) along the
/// existing receiver graph. Ocean cells terminate the transport.
pub fn accumulate_downhill(
    grid: &HeightGrid,
    mut acc: Vec<Vec<f64>>,
    stop_at_ocean: bool,
) -> Vec<Vec<f64>> {
    let (rows, cols) = (grid.rows(), grid.cols());
    // Process cells highest-first so upstream accumulates before routing.
    let mut order: Vec<(usize, usize)> = (0..rows)
        .flat_map(|r| (0..cols).map(move |c| (r, c)))
        .collect();
    order.sort_by(|a, b| {
        grid.h[b.0][b.1]
            .partial_cmp(&grid.h[a.0][a.1])
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    for (r, c) in order {
        let h = grid.h[r][c];
        if stop_at_ocean && h < 0.0 {
            continue;
        }
        let best = downstream_cell(grid, r, c);
        if let Some((nr, nc)) = best {
            acc[nr][nc] += acc[r][c];
        }
    }
    acc
}

/// Shared strictly-downhill receiver. Fixed scan order gives stable ties;
/// longitude wraps, flat cells and closed pits have no receiver.
pub fn downstream_cell(grid: &HeightGrid, r: usize, c: usize) -> Option<(usize, usize)> {
    if grid.is_open_edge(r, c) {
        return None;
    }
    let mut best = None;
    let mut best_h = grid.h[r][c];
    let mut best_slope = 0.0;
    for (nr, nc) in grid.drainage_adjacent(r, c) {
        if !grid.longitude_periodic {
            let (dy, dx) = grid.cell_m(r);
            let distance = ((nr.abs_diff(r) as f64 * dy).powi(2)
                + (nc.abs_diff(c) as f64 * dx).powi(2))
            .sqrt();
            let slope = (grid.h[r][c] - grid.h[nr][nc]) / distance.max(1.0);
            if slope > best_slope {
                best = Some((nr, nc));
                best_slope = slope;
            }
            continue;
        }
        if grid.h[nr][nc] < best_h {
            best_h = grid.h[nr][nc];
            best = Some((nr, nc));
        }
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cone() -> HeightGrid {
        let lats: Vec<f64> = (0..7).map(|i| 3.0 - i as f64).collect();
        let lons: Vec<f64> = (0..7).map(|i| -3.0 + i as f64).collect();
        let mut g = HeightGrid::new(lats, lons, 3_200_000.0);
        for (r, row) in g.h.iter_mut().enumerate() {
            for (c, h) in row.iter_mut().enumerate() {
                let d = ((r as f64 - 3.0).powi(2) + (c as f64 - 3.0).powi(2)).sqrt();
                *h = 5000.0 - d * 1000.0;
            }
        }
        g
    }

    #[test]
    fn slope_zero_at_peak_positive_on_flank() {
        let g = cone();
        assert!(slope_at(&g, 3, 3) < 0.01);
        assert!(slope_at(&g, 3, 5) > 0.005);
    }

    #[test]
    fn accumulation_collects_downhill() {
        let g = cone();
        let acc = flow_accumulation(&g);
        assert!(acc[3][0] > acc[3][3]);
        assert!(acc.iter().flatten().all(|v| *v >= 1));
    }

    #[test]
    fn catchments_use_spherical_area_and_stop_at_ocean() {
        let mut g = HeightGrid::new(
            vec![-60.0, 0.0, 60.0],
            vec![0.0, 90.0, 180.0, 270.0],
            1000.0,
        );
        g.h = vec![vec![100.0; 4]; 3];
        let area = flow_catchment_area_m2(&g);
        assert!(area[1][0] > area[0][0] * 1.9);
        let total: f64 = area.iter().flatten().sum();
        assert!((total - 4.0 * std::f64::consts::PI * 1.0e6).abs() < 1e-6);
        let mut coast = HeightGrid::new(vec![0.0], vec![0.0, 90.0, 180.0, 270.0], 1000.0);
        coast.h[0] = vec![-0.5, 20.0, -1.0, -2.0];
        let acc = flow_catchment_area_m2(&coast);
        assert!(acc[0][2] > 0.0);
        assert_eq!(acc[0][3], 0.0, "ocean must not route catchments onward");
    }

    #[test]
    fn no_nan_inf() {
        let g = cone();
        for r in 0..g.rows() {
            for c in 0..g.cols() {
                assert!(slope_at(&g, r, c).is_finite());
                assert!(curvature_at(&g, r, c).is_finite());
            }
        }
    }
}
