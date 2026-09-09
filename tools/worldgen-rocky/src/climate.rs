//! Heuristic climate-driver masks (NOT a GCM).
//!
//! Four macro drivers from the Thessa spec plus normal ones:
//! - eclipse_exposure: how much Asterion-A light survives Nereid eclipses
//!   (sub-Nereid longitudes lose midday heating near eclipse season)
//! - nereid_influence: IR + reflected light, maritime damping on the facing side
//! - continentality: distance-to-ocean in metres, derived from terrain
//! - geothermal: from `geothermal.rs`, consumed here as local warming
//!
//! Plus latitude, elevation cooling, moisture proxy, circulation proxy.

use crate::hydro::{HeightGrid, WaterClass};

/// Sub-Nereid longitude for a synchronously rotating moon: faces the giant.
/// Convention: lon 0 faces Nereid; anti-Nereid side is lon +/-180.
pub const SUB_NEREID_LON_DEG: f64 = 0.0;

/// Eclipse exposure 0..1: 1 = full stellar heating, lower near the
/// sub-Nereid point where midday eclipses bite (eclipse season geometry
/// is not frozen, so this is a smooth heuristic band, not a hard shadow).
pub fn eclipse_exposure(lon_deg: f64, eclipse_strength: f64) -> f64 {
    let mut d = (lon_deg - SUB_NEREID_LON_DEG).to_radians();
    while d > std::f64::consts::PI {
        d -= 2.0 * std::f64::consts::PI;
    }
    while d < -std::f64::consts::PI {
        d += 2.0 * std::f64::consts::PI;
    }
    // Gaussian dip around the facing meridian, ~60 deg wide.
    let dip = (-(d / 1.05).powi(2)).exp();
    (1.0 - eclipse_strength * 0.45 * dip).clamp(0.0, 1.0)
}

/// Nereid radiative/maritime influence 0..1, peaks at sub-Nereid point.
pub fn nereid_influence(lon_deg: f64) -> f64 {
    let mut d = (lon_deg - SUB_NEREID_LON_DEG).to_radians();
    while d > std::f64::consts::PI {
        d -= 2.0 * std::f64::consts::PI;
    }
    while d < -std::f64::consts::PI {
        d += 2.0 * std::f64::consts::PI;
    }
    ((d.cos() + 1.0) / 2.0).powf(1.5)
}

/// Continentality 0 (open ocean air) .. 1 (deep interior) from a BFS over
/// land cells, measured in grid steps converted to metres by the caller.
/// Deterministic: no RNG involved.
pub fn continentality_steps(grid: &HeightGrid, water: &[Vec<WaterClass>]) -> Vec<Vec<u32>> {
    use std::collections::VecDeque;
    let (rows, cols) = (grid.rows(), grid.cols());
    let mut dist = vec![vec![u32::MAX; cols]; rows];
    let mut queue = VecDeque::new();
    for r in 0..rows {
        for c in 0..cols {
            if matches!(water[r][c], WaterClass::Ocean) {
                dist[r][c] = 0;
                queue.push_back((r, c));
            }
        }
    }
    while let Some((r, c)) = queue.pop_front() {
        let nd = dist[r][c].saturating_add(1);
        // 4-neighbours + longitude wrap.
        let nb = [
            (r.wrapping_sub(1), c),
            (r + 1, c),
            (r, (c + cols - 1) % cols),
            (r, (c + 1) % cols),
        ];
        for (nr, nc) in nb {
            if nr < rows && dist[nr][nc] == u32::MAX {
                dist[nr][nc] = nd;
                queue.push_back((nr, nc));
            }
        }
    }
    dist
}

/// Full driver bundle at one site. All 0..1 unless noted.
#[derive(Debug, Clone, Copy)]
pub struct ClimateDrivers {
    pub eclipse_exposure: f64,
    pub nereid_influence: f64,
    /// 0 ocean air .. 1 deep interior (normalized by 2500 km).
    pub continentality: f64,
    /// 0 equator .. 1 pole.
    pub polar: f64,
    /// Elevation cooling 0..1 (lapse-rate proxy, 12 km => 1).
    pub elevation: f64,
    /// Moisture proxy 0..1: ocean nearness + facing-side storm tracks.
    pub moisture: f64,
    /// Geothermal local warming 0..1 (from geothermal field).
    pub geothermal: f64,
}

#[allow(clippy::too_many_arguments)]
pub fn drivers_at(
    lat_deg: f64,
    lon_deg: f64,
    height_m: f64,
    ocean_dist_m: f64,
    eclipse_strength: f64,
    geothermal: f64,
) -> ClimateDrivers {
    let polar = (lat_deg.abs() / 90.0).clamp(0.0, 1.0);
    let continentality = (ocean_dist_m / 2_500_000.0).clamp(0.0, 1.0);
    let elevation = (height_m.max(0.0) / 12_000.0).clamp(0.0, 1.0);
    let facing = nereid_influence(lon_deg);
    // Wetter on the facing side + near coasts, drier deep inland.
    let moisture = ((1.0 - continentality) * 0.7 + facing * 0.3).clamp(0.0, 1.0);
    ClimateDrivers {
        eclipse_exposure: eclipse_exposure(lon_deg, eclipse_strength),
        nereid_influence: facing,
        continentality,
        polar,
        elevation,
        moisture,
        geothermal: geothermal.clamp(0.0, 1.0),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn facing_side_gets_less_eclipse_more_nereid() {
        assert!(eclipse_exposure(0.0, 0.5) < eclipse_exposure(180.0, 0.5));
        assert!(nereid_influence(0.0) > nereid_influence(180.0));
        assert!((nereid_influence(180.0)).abs() < 1e-12);
    }

    #[test]
    fn continentality_zero_at_ocean_grows_inland() {
        let lats: Vec<f64> = (0..5).map(|i| 2.0 - i as f64).collect();
        let lons: Vec<f64> = (0..7).map(|i| i as f64).collect();
        let mut grid = HeightGrid::new(lats, lons, 3_200_000.0);
        for row in grid.h.iter_mut() {
            for h in row.iter_mut() {
                *h = 1000.0;
            }
            row[0] = -500.0; // west ocean column
        }
        let water: Vec<Vec<WaterClass>> = grid
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
        let dist = continentality_steps(&grid, &water);
        assert_eq!(dist[2][0], 0);
        // Longitude wraps, so mid-landmass (col 3) is farthest from the ocean.
        assert!(dist[2][3] > dist[2][1], "{:?}", dist[2]);
    }

    #[test]
    fn drivers_stay_in_range() {
        let d = drivers_at(-30.0, 45.0, 2500.0, 800_000.0, 0.28, 0.1);
        for v in [
            d.eclipse_exposure,
            d.nereid_influence,
            d.continentality,
            d.polar,
            d.elevation,
            d.moisture,
            d.geothermal,
        ] {
            assert!((0.0..=1.0).contains(&v) && v.is_finite());
        }
    }
}
