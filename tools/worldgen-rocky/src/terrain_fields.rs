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
    let hx = (grid.h[r][(c + 1) % cols] - grid.h[r][(c + cols - 1) % cols]) / (2.0 * dx_m.max(1.0));
    let r_up = r.saturating_sub(1);
    let r_dn = (r + 1).min(rows - 1);
    let hy = (grid.h[r_dn][c] - grid.h[r_up][c]) / (2.0 * dy_m.max(1.0));
    (hx * hx + hy * hy).sqrt()
}

/// Mean curvature (Laplacian / cell area): >0 pit/mound, <0 ridge.
/// Scale: 1/metres. Sign convention documented for scatter use.
pub fn curvature_at(grid: &HeightGrid, r: usize, c: usize) -> f64 {
    let cols = grid.cols();
    let rows = grid.rows();
    let (dy_m, dx_m) = grid.cell_m(r);
    let h = grid.h[r][c];
    let lap = (grid.h[r][(c + 1) % cols] + grid.h[r][(c + cols - 1) % cols] - 2.0 * h)
        / dx_m.max(1.0).powi(2)
        + (grid.h[(r + 1).min(rows - 1)][c] + grid.h[r.saturating_sub(1)][c] - 2.0 * h)
            / dy_m.max(1.0).powi(2);
    if lap.is_finite() { lap } else { 0.0 }
}

/// Simple D8 flow accumulation (cell counts, deterministic order).
/// Each cell routes all its water to the lowest neighbour; accumulation
/// counts how many cells drain through it. Rivers = high accumulation.
pub fn flow_accumulation(grid: &HeightGrid) -> Vec<Vec<u32>> {
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
    let mut acc = vec![vec![1u32; cols]; rows];
    for (r, c) in order {
        let h = grid.h[r][c];
        let mut best: Option<(usize, usize)> = None;
        let mut best_h = h;
        // Fixed scan order => deterministic ties.
        let nb = [
            (r.saturating_sub(1), c),
            ((r + 1).min(rows - 1), c),
            (r, (c + cols - 1) % cols),
            (r, (c + 1) % cols),
        ];
        for (nr, nc) in nb {
            if grid.h[nr][nc] < best_h {
                best_h = grid.h[nr][nc];
                best = Some((nr, nc));
            }
        }
        if let Some((nr, nc)) = best {
            acc[nr][nc] = acc[nr][nc].saturating_add(acc[r][c]);
        }
    }
    acc
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
        // Edges receive drainage from the peak.
        assert!(acc[3][0] > acc[3][3]);
        assert!(acc.iter().flatten().all(|v| *v >= 1));
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
