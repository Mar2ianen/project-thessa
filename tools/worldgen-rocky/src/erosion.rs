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

/// Offline landscape-evolution proxy. Runoff drives incision; transported
/// sediment is measured in cubic metres and deposited on-grid, including sea.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct RunoffErosion {
    pub iterations: u32,
    pub years_per_iteration: f64,
    /// Stream-power erodibility: m/year divided by sqrt(m3/year) and slope.
    pub erodibility: f64,
    /// Length over which a moving sediment load settles, in metres.
    pub sediment_transport_m: f64,
}

impl Default for RunoffErosion {
    fn default() -> Self {
        Self {
            iterations: 32,
            years_per_iteration: 1000.0,
            erodibility: 0.00002,
            sediment_transport_m: 50_000.0,
        }
    }
}

impl RunoffErosion {
    pub fn validate(self) -> Result<(), String> {
        if self.iterations > 256
            || !self.years_per_iteration.is_finite()
            || !(0.0..=100_000.0).contains(&self.years_per_iteration)
            || !self.erodibility.is_finite()
            || !(0.0..=0.01).contains(&self.erodibility)
            || !self.sediment_transport_m.is_finite()
            || self.sediment_transport_m <= 0.0
        {
            return Err("invalid bounded offline runoff erosion recipe".into());
        }
        Ok(())
    }
}

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

/// Run conservative runoff incision and sediment transport in place.
pub fn erode_runoff(
    grid: &mut HeightGrid,
    runoff_m_yr: &[Vec<f64>],
    recipe: RunoffErosion,
) -> Result<(), String> {
    erode_runoff_budget(grid, runoff_m_yr, recipe).map(|_| ())
}

/// Same runoff pipeline, reporting sediment crossing regional open edges.
/// Global periodic grids retain their original closed-domain semantics.
pub fn erode_runoff_budget(
    grid: &mut HeightGrid,
    runoff_m_yr: &[Vec<f64>],
    recipe: RunoffErosion,
) -> Result<f64, String> {
    recipe.validate()?;
    if runoff_m_yr.len() != grid.rows()
        || runoff_m_yr
            .iter()
            .any(|row| row.len() != grid.cols() || row.iter().any(|v| !v.is_finite() || *v < 0.0))
    {
        return Err("runoff must be a matching finite nonnegative SI grid".into());
    }
    let mut exported_volume_m3 = 0.0;
    for _ in 0..recipe.iterations {
        let source = runoff_m_yr
            .iter()
            .enumerate()
            .map(|(r, row)| {
                let area = crate::terrain_fields::cell_area_m2(grid, r);
                row.iter()
                    .enumerate()
                    .map(|(c, rain)| {
                        if grid.h[r][c] >= 0.0 && !grid.is_open_edge(r, c) {
                            rain * area
                        } else {
                            0.0
                        }
                    })
                    .collect()
            })
            .collect();
        let discharge = crate::terrain_fields::accumulate_downhill(grid, source, true);
        exported_volume_m3 += runoff_step(grid, &discharge, recipe);
    }
    Ok(exported_volume_m3)
}

/// Legacy thermal and droplet passes, separate from SI runoff evolution.
pub fn erode(grid: &mut HeightGrid, knobs: ErosionKnobs) {
    thermal_pass(grid, knobs.thermal_iters, knobs.talus_deg);
    hydraulic_pass(grid, knobs.droplets, knobs.droplet_steps);
}

/// Talus relaxation: move material from cells steeper than repose.
fn runoff_step(grid: &mut HeightGrid, discharge: &[Vec<f64>], recipe: RunoffErosion) -> f64 {
    let mut order: Vec<_> = (0..grid.rows())
        .flat_map(|r| (0..grid.cols()).map(move |c| (r, c)))
        .collect();
    order.sort_by(|a, b| grid.h[b.0][b.1].total_cmp(&grid.h[a.0][a.1]).then(a.cmp(b)));
    let mut sediment = vec![vec![0.0; grid.cols()]; grid.rows()];
    let mut delta = sediment.clone();
    let mut exported = 0.0;
    for (r, c) in order {
        if grid.is_open_edge(r, c) {
            exported += sediment[r][c];
            continue;
        }
        let area = crate::terrain_fields::cell_area_m2(grid, r);
        if grid.h[r][c] < 0.0 {
            delta[r][c] += sediment[r][c] / area;
            continue;
        }
        let Some((nr, nc)) = crate::terrain_fields::downstream_cell(grid, r, c) else {
            delta[r][c] += sediment[r][c] / area;
            continue;
        };
        let distance = crate::sphere::great_circle_m(
            crate::sphere::dir_from_latlon(grid.lats[r], grid.lons[c]),
            crate::sphere::dir_from_latlon(grid.lats[nr], grid.lons[nc]),
            grid.datum_radius_m,
        )
        .max(1.0);
        let drop = grid.h[r][c] - grid.h[nr][nc];
        let take = (recipe.erodibility * discharge[r][c].sqrt() * drop / distance
            * recipe.years_per_iteration)
            .min(drop * 0.25)
            .max(0.0);
        delta[r][c] -= take;
        let load = sediment[r][c] + take * area;
        let settled = load * (1.0 - (-distance / recipe.sediment_transport_m).exp());
        let receiver_area = crate::terrain_fields::cell_area_m2(grid, nr);
        if grid.is_open_edge(nr, nc) {
            sediment[nr][nc] += load;
        } else {
            delta[nr][nc] += settled / receiver_area;
            sediment[nr][nc] += load - settled;
        }
    }
    for (row, change) in grid.h.iter_mut().zip(delta) {
        for (h, dh) in row.iter_mut().zip(change) {
            *h += dh;
        }
    }
    // A pole is one physical point. Cap cells have equal area, so replacing
    // sector heights by their mean preserves sediment volume and pole identity.
    for r in [0, grid.rows() - 1] {
        if grid.lats[r].abs() == 90.0 {
            let mean = grid.h[r].iter().sum::<f64>() / grid.cols() as f64;
            grid.h[r].fill(mean);
        }
    }
    exported
}

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

    #[test]
    fn regional_runoff_preserves_edges_and_accounts_for_exported_volume() {
        let mut grid = HeightGrid::regional(
            (0..17).map(|i| 30.0 + i as f64 * 0.01).collect(),
            (0..17).map(|i| 179.9 + i as f64 * 0.01).collect(),
            3_200_000.0,
        );
        for (r, row) in grid.h.iter_mut().enumerate() {
            for (c, h) in row.iter_mut().enumerate() {
                *h = 1000.0 - 20.0 * r as f64 - 12.0 * c as f64;
            }
        }
        let source = grid.h.clone();
        let exported = erode_runoff_budget(
            &mut grid,
            &vec![vec![0.5; 17]; 17],
            RunoffErosion {
                iterations: 8,
                years_per_iteration: 1000.0,
                ..Default::default()
            },
        )
        .unwrap();
        let mut change = 0.0;
        for (r, row) in grid.h.iter().enumerate() {
            let area = crate::terrain_fields::cell_area_m2(&grid, r);
            for (c, h) in row.iter().enumerate() {
                if grid.is_open_edge(r, c) {
                    assert_eq!(*h, source[r][c]);
                }
                change += (*h - source[r][c]) * area;
            }
        }
        assert!(exported > 0.0);
        assert!(
            (change + exported).abs() < exported * 1e-9 + 1e-3,
            "volume error: {}",
            change + exported
        );
        assert_eq!(
            crate::terrain_fields::downstream_cell(&grid, 8, 8),
            Some((9, 9))
        );
        assert_eq!(crate::terrain_fields::downstream_cell(&grid, 8, 0), None);
    }

    #[test]
    fn old_crater_rim_erodes_and_deposits_into_its_closed_floor() {
        let mut grid = HeightGrid::new(
            (0..9).map(|r| -0.4 + r as f64 * 0.1).collect(),
            (0..9).map(|c| -0.4 + c as f64 * 0.1).collect(),
            3_200_000.0,
        );
        for (r, row) in grid.h.iter_mut().enumerate() {
            for (c, h) in row.iter_mut().enumerate() {
                let radius = (r as f64 - 4.0).hypot(c as f64 - 4.0);
                *h = 100.0 + 900.0 * (-(radius - 2.0).powi(2) * 2.0).exp();
            }
        }
        let original = grid.clone();
        let rain = vec![vec![0.8; 9]; 9];
        let initial = material_volume(&grid);
        erode_runoff(
            &mut grid,
            &rain,
            RunoffErosion {
                years_per_iteration: 100_000.0,
                ..Default::default()
            },
        )
        .unwrap();
        assert!(grid.h[4][6] < original.h[4][6]);
        assert!(grid.h[4][4] > original.h[4][4]);
        assert!(grid.h[4][6] - grid.h[4][4] < original.h[4][6] - original.h[4][4]);
        assert!((material_volume(&grid) - initial).abs() < initial.abs() * 1e-12);
    }

    fn material_volume(grid: &HeightGrid) -> f64 {
        grid.h
            .iter()
            .enumerate()
            .map(|(r, row)| row.iter().sum::<f64>() * crate::terrain_fields::cell_area_m2(grid, r))
            .sum()
    }

    #[test]
    fn offline_runoff_incises_high_ground_and_conserves_spherical_sediment_volume() {
        let mut grid = ridge_grid();
        let before = grid.clone();
        let rain = vec![vec![0.8; grid.cols()]; grid.rows()];
        let initial_volume = material_volume(&grid);
        erode_runoff(&mut grid, &rain, RunoffErosion::default()).unwrap();
        let drift = (material_volume(&grid) - initial_volume).abs();
        assert!(
            drift < initial_volume.abs() * 1e-12,
            "volume drift {drift} m3"
        );
        assert!(grid.h[4][4] < before.h[4][4], "runoff must erode the ridge");
        assert!(
            grid.h
                .iter()
                .flatten()
                .zip(before.h.iter().flatten())
                .any(|(a, b)| a > b)
        );
        let mut duplicate = before.clone();
        erode_runoff(&mut duplicate, &rain, RunoffErosion::default()).unwrap();
        assert_eq!(grid.h, duplicate.h);
        let mut dry = before.clone();
        let no_rain = vec![vec![0.0; dry.cols()]; dry.rows()];
        erode_runoff(&mut dry, &no_rain, RunoffErosion::default()).unwrap();
        assert_eq!(dry.h, before.h);
    }

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
