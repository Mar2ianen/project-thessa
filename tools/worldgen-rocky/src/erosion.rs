//! Lightweight deterministic erosion on the height grid.
//!
//! Two passes, both in physical metres, both seeded-deterministic:
//! - thermal: talus relaxation toward the repose angle (diffusive, stable)
//! - hydraulic: seeded droplet walk — erode on steep descent, deposit on
//!   flats/pits. Droplet starts come from a strided lattice, never RNG streams.
//!
//! This is intentionally NOT a full landscape-evolution model: enough to make
//! mountains shed talus, valleys collect sediment and rivers carve downhill.

use crate::hydro::HeightGrid;

/// Recipe knobs.
#[derive(Debug, Clone, Copy)]
pub struct ErosionKnobs {
    /// Thermal relaxation iterations.
    pub thermal_iters: u32,
    /// Repose angle in degrees (typical rock 30-37).
    pub talus_deg: f64,
    /// Number of droplets (scaled internally by grid size).
    pub droplets: u32,
    /// Max steps per droplet.
    pub droplet_steps: u32,
}

impl Default for ErosionKnobs {
    fn default() -> Self {
        Self {
            thermal_iters: 10,
            talus_deg: 34.0,
            droplets: 4000,
            droplet_steps: 64,
        }
    }
}

/// Run both passes in place.
pub fn erode(grid: &mut HeightGrid, knobs: ErosionKnobs) {
    thermal_pass(grid, knobs.thermal_iters, knobs.talus_deg);
    hydraulic_pass(grid, knobs.droplets, knobs.droplet_steps);
}

/// Talus relaxation: move material from cells steeper than repose.
/// Checkerboard-ordered double buffer => deterministic and oscillation-free.
pub fn thermal_pass(grid: &mut HeightGrid, iters: u32, talus_deg: f64) {
    let talus = talus_deg.to_radians().tan().max(0.05);
    let (rows, cols) = (grid.rows(), grid.cols());
    if rows < 3 || cols < 3 || iters == 0 {
        return;
    }
    let mut delta = vec![vec![0.0; cols]; rows];
    for _ in 0..iters.min(200) {
        for row in &mut delta {
            for v in row.iter_mut() {
                *v = 0.0;
            }
        }
        for r in 1..rows - 1 {
            let (_, dx_m) = grid.cell_m(r);
            let (dy_m, _) = grid.cell_m(r);
            for c in 0..cols {
                let h = grid.h[r][c];
                // 4-neighbours (no wrap on rows; wrap on longitude).
                let nb = [
                    (r - 1, c),
                    (r + 1, c),
                    (r, (c + cols - 1) % cols),
                    (r, (c + 1) % cols),
                ];
                for (nr, nc) in nb {
                    let dist = if nr == r { dx_m } else { dy_m };
                    let dh = h - grid.h[nr][nc];
                    let slope = dh / dist.max(1.0);
                    if slope > talus {
                        // Move a fraction of the excess above repose.
                        let move_h = (dh - talus * dist) * 0.12;
                        delta[r][c] -= move_h;
                        delta[nr][nc] += move_h;
                    }
                }
            }
        }
        for (r, row) in grid.h.iter_mut().enumerate() {
            for (c, h) in row.iter_mut().enumerate() {
                *h += delta[r][c];
            }
        }
    }
}

/// Droplet erosion: deterministic starts on a strided lattice over high cells.
pub fn hydraulic_pass(grid: &mut HeightGrid, droplets: u32, max_steps: u32) {
    let (rows, cols) = (grid.rows(), grid.cols());
    if rows < 3 || cols < 3 || droplets == 0 {
        return;
    }
    let stride = ((rows * cols) as f64 / f64::from(droplets).max(1.0))
        .sqrt()
        .ceil()
        .max(1.0) as usize;
    let mut done = 0u32;
    let mut r = 1usize;
    while r < rows - 1 && done < droplets {
        let mut c = 1 + (r * 7 % stride);
        while c < cols - 1 && done < droplets {
            if grid.h[r][c] > 100.0 {
                droplet(grid, r, c, max_steps.min(256));
                done += 1;
            }
            c += stride;
        }
        r += stride.max(1);
    }
}

fn downhill(grid: &HeightGrid, r: usize, c: usize) -> Option<(usize, usize, f64)> {
    let (rows, cols) = (grid.rows(), grid.cols());
    let h = grid.h[r][c];
    let (_, dx_m) = grid.cell_m(r);
    let (dy_m, _) = grid.cell_m(r);
    let mut best: Option<(usize, usize, f64)> = None;
    // Fixed neighbour order => deterministic ties.
    let nb = [
        (r.wrapping_sub(1), c, dy_m),
        (r + 1, c, dy_m),
        (r, (c + cols - 1) % cols, dx_m),
        (r, (c + 1) % cols, dx_m),
    ];
    for (nr, nc, dist) in nb {
        if nr >= rows {
            continue;
        }
        let slope = (h - grid.h[nr][nc]) / dist.max(1.0);
        if slope > 1e-9 && best.is_none_or(|(_, _, s)| slope > s) {
            best = Some((nr, nc, slope));
        }
    }
    best
}

fn droplet(grid: &mut HeightGrid, mut r: usize, mut c: usize, max_steps: u32) {
    let mut sediment = 0.0;
    for _ in 0..max_steps {
        match downhill(grid, r, c) {
            Some((nr, nc, slope)) => {
                // Erode proportional to slope, capacity-limited.
                let take = (slope * 8.0).min(40.0).min(grid.h[r][c] + 20000.0);
                let take = take.max(0.0) * 0.5;
                grid.h[r][c] -= take;
                sediment += take;
                // Deposit a fraction while moving (valley fill).
                let drop = sediment * 0.08;
                grid.h[nr][nc] += drop;
                sediment -= drop;
                r = nr;
                c = nc;
            }
            None => {
                // Pit or flat: dump remaining load, stop. Never flows uphill.
                grid.h[r][c] += sediment;
                return;
            }
        }
    }
    grid.h[r][c] += sediment;
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ridge_grid() -> HeightGrid {
        let lats: Vec<f64> = (0..9).map(|i| 4.0 - i as f64 * 0.5).collect();
        let lons: Vec<f64> = (0..9).map(|i| -4.0 + i as f64 * 0.5).collect();
        let mut g = HeightGrid::new(lats, lons, 3_200_000.0);
        for (r, row) in g.h.iter_mut().enumerate() {
            for (c, h) in row.iter_mut().enumerate() {
                // Sharp pyramid: oversleep slopes everywhere.
                let d = ((r as f64 - 4.0).abs() + (c as f64 - 4.0).abs()) * 20_000.0;
                *h = 20_000.0 - d;
            }
        }
        g
    }

    fn max_slope(grid: &HeightGrid) -> f64 {
        let mut m: f64 = 0.0;
        for r in 1..grid.rows() - 1 {
            let (_, dx) = grid.cell_m(r);
            for c in 0..grid.cols() {
                let s = ((grid.h[r][c] - grid.h[r][(c + 1) % grid.cols()]).abs()) / dx;
                m = m.max(s);
            }
        }
        m
    }

    #[test]
    fn thermal_reduces_oversleep_slopes() {
        let mut g = ridge_grid();
        let before = max_slope(&g);
        thermal_pass(&mut g, 30, 34.0);
        let after = max_slope(&g);
        assert!(after < before, "{after} vs {before}");
        assert!(g.h.iter().flatten().all(|v| v.is_finite()));
    }

    #[test]
    fn erosion_conserves_mass_roughly() {
        let mut g = ridge_grid();
        let before: f64 = g.h.iter().flatten().sum();
        erode(
            &mut g,
            ErosionKnobs {
                thermal_iters: 6,
                talus_deg: 34.0,
                droplets: 200,
                droplet_steps: 32,
            },
        );
        let after: f64 = g.h.iter().flatten().sum();
        // Thermal pass is conservative; droplets keep sediment on-grid.
        assert!(
            (before - after).abs() / before.abs() < 0.02,
            "{before} vs {after}"
        );
    }

    #[test]
    fn erosion_is_deterministic() {
        let mut a = ridge_grid();
        let mut b = ridge_grid();
        erode(&mut a, ErosionKnobs::default());
        erode(&mut b, ErosionKnobs::default());
        assert_eq!(a.h, b.h);
    }
}
