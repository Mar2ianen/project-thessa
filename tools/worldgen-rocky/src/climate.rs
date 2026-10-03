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

/// Recipe-controlled climate proxy, not atmospheric flight physics or a GCM.
/// Missing settings reproduce the legacy temperature field.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct SurfaceClimate {
    pub target_mean_temperature_k: Option<f64>,
    pub polar_cooling_strength: f64,
    pub elevation_cooling_strength: f64,
    pub eclipse_cooling_strength: f64,
    pub geothermal_local_warming_strength: f64,
    /// E-folding travel distance for ocean-supplied moisture, in metres.
    pub moisture_transport_m: f64,
    /// Cumulative ascent over which orographic condensation removes 1-1/e.
    pub rainout_height_m: f64,
    /// Facing-side cloud retention bias in the static climate proxy.
    pub high_cloud_wet_region_bias: f64,
    /// Annual water supply represented by unit moisture, in metres/year.
    pub precipitation_m_yr: f64,
    /// Open-water potential evaporation at 278 K, in metres/year.
    pub evaporation_m_yr_at_278k: f64,
}

impl Default for SurfaceClimate {
    fn default() -> Self {
        Self {
            target_mean_temperature_k: None,
            polar_cooling_strength: 1.0,
            elevation_cooling_strength: 1.0,
            eclipse_cooling_strength: 0.28,
            geothermal_local_warming_strength: 0.0,
            moisture_transport_m: 1_200_000.0,
            rainout_height_m: 1800.0,
            high_cloud_wet_region_bias: 0.0,
            precipitation_m_yr: 1.0,
            evaporation_m_yr_at_278k: 0.6,
        }
    }
}

impl SurfaceClimate {
    pub fn validate(self) -> Result<(), String> {
        if self
            .target_mean_temperature_k
            .is_some_and(|v| !v.is_finite() || v <= 0.0)
        {
            return Err("climate target mean temperature must be finite and positive".into());
        }
        for strength in [
            self.polar_cooling_strength,
            self.elevation_cooling_strength,
            self.eclipse_cooling_strength,
            self.geothermal_local_warming_strength,
            self.high_cloud_wet_region_bias,
        ] {
            if !strength.is_finite() || !(0.0..=1.0).contains(&strength) {
                return Err("climate proxy strengths must be finite within 0..=1".into());
            }
        }
        for length in [self.moisture_transport_m, self.rainout_height_m] {
            if !length.is_finite() || length <= 0.0 {
                return Err("climate moisture lengths must be finite and positive metres".into());
            }
        }
        for flux in [self.precipitation_m_yr, self.evaporation_m_yr_at_278k] {
            if !flux.is_finite() || flux < 0.0 {
                return Err("climate annual water fluxes must be finite and nonnegative".into());
            }
        }
        Ok(())
    }
}

impl SurfaceClimate {
    /// Annual liquid-flow proxy, not measured weather or a water inventory.
    pub fn runoff_m_yr(self, moisture01: f64, temperature_k: f64) -> f64 {
        self.precipitation_m_yr
            * moisture01.powi(2)
            * crate::appearance::smooth(258.0, 278.0, temperature_k)
    }

    /// Saturation-vapour-pressure scaling (Clausius-Clapeyron). Persistent
    /// freezing excludes a liquid evaporative playa; ice/sublimation is not modeled.
    pub fn evaporation_m_yr(self, temperature_k: f64) -> f64 {
        if temperature_k <= 273.15 {
            return 0.0;
        }
        self.evaporation_m_yr_at_278k
            * (45_000.0 / 8.314462618 * (1.0 / 278.0 - 1.0 / temperature_k)).exp()
    }
}

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

/// Continentality in METRES from the nearest ocean, via deterministic
/// Dijkstra over the grid with physical edge lengths (dy_m / dx_m per row).
/// A polar cell never counts the same as an equatorial one: edge weights
/// shrink with cos(latitude). Longitude wraps; pole rows are dead ends.
/// Deterministic: heap ordered by (distance bits, row, col).
pub fn continentality_metres(grid: &HeightGrid, water: &[Vec<WaterClass>]) -> Vec<Vec<f64>> {
    use std::collections::BinaryHeap;
    #[derive(PartialEq)]
    struct Item {
        dist_bits: u64,
        r: usize,
        c: usize,
    }
    impl Eq for Item {}
    impl Ord for Item {
        fn cmp(&self, other: &Self) -> std::cmp::Ordering {
            other
                .dist_bits
                .cmp(&self.dist_bits)
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
    let mut dist = vec![vec![f64::INFINITY; cols]; rows];
    let mut heap = BinaryHeap::new();
    for r in 0..rows {
        for c in 0..cols {
            if matches!(water[r][c], WaterClass::Ocean) {
                dist[r][c] = 0.0;
                heap.push(Item { dist_bits: 0, r, c });
            }
        }
    }
    while let Some(Item { r, c, .. }) = heap.pop() {
        let here = dist[r][c];
        let (dy_m, dx_m) = grid.cell_m(r);
        let nb = [
            (r.wrapping_sub(1), c, dy_m),
            (r + 1, c, dy_m),
            (r, (c + cols - 1) % cols, dx_m),
            (r, (c + 1) % cols, dx_m),
        ];
        for (nr, nc, w) in nb {
            if nr >= rows {
                continue;
            }
            let nd = here + w;
            if nd < dist[nr][nc] {
                dist[nr][nc] = nd;
                heap.push(Item {
                    dist_bits: nd.to_bits(),
                    r: nr,
                    c: nc,
                });
            }
        }
    }
    dist
}

/// Ocean-supplied moisture surviving travel and cumulative orographic ascent.
/// Shortest attenuation paths can go around mountains; descent does not restore
/// condensed water. This is a static, isotropic transport proxy, not a GCM or a
/// precipitation inventory. The grid must contain datum-calibrated heights.
pub fn moisture_grid(grid: &HeightGrid, recipe: SurfaceClimate) -> Vec<Vec<f64>> {
    use std::{cmp::Reverse, collections::BinaryHeap};
    let (rows, cols) = (grid.rows(), grid.cols());
    let mut cost = vec![vec![f64::INFINITY; cols]; rows];
    let mut heap = BinaryHeap::new();
    for (r, row) in grid.h.iter().enumerate() {
        for (c, h) in row.iter().enumerate() {
            if *h < 0.0 {
                cost[r][c] = 0.0;
                heap.push(Reverse((0u64, r, c)));
            }
        }
    }
    while let Some(Reverse((bits, r, c))) = heap.pop() {
        let here = f64::from_bits(bits);
        if here != cost[r][c] {
            continue;
        }
        let start = crate::sphere::dir_from_latlon(grid.lats[r], grid.lons[c]);
        for (nr, nc) in [
            (r.saturating_sub(1), c),
            ((r + 1).min(rows - 1), c),
            (r, (c + cols - 1) % cols),
            (r, (c + 1) % cols),
        ] {
            let end = crate::sphere::dir_from_latlon(grid.lats[nr], grid.lons[nc]);
            let distance = crate::sphere::great_circle_m(start, end, grid.datum_radius_m);
            let retention =
                1.0 + recipe.high_cloud_wet_region_bias * nereid_influence(grid.lons[nc]);
            let ascent = (grid.h[nr][nc].max(0.0) - grid.h[r][c].max(0.0)).max(0.0);
            let next = here
                + distance / (recipe.moisture_transport_m * retention)
                + ascent / recipe.rainout_height_m;
            if next < cost[nr][nc] {
                cost[nr][nc] = next;
                heap.push(Reverse((next.to_bits(), nr, nc)));
            }
        }
    }
    cost.into_iter()
        .map(|row| row.into_iter().map(|v| (-v).exp()).collect())
        .collect()
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
    fn moisture_transport_decays_in_metres_and_descent_does_not_restore_rainout() {
        let mut grid = HeightGrid::new(vec![0.0], (0..12).map(|i| i as f64).collect(), 3_200_000.0);
        grid.h[0].fill(0.0);
        grid.h[0][0] = -100.0;
        let recipe = SurfaceClimate {
            moisture_transport_m: 300_000.0,
            ..Default::default()
        };
        let flat = moisture_grid(&grid, recipe);
        let distance = 3.0_f64.to_radians() * grid.datum_radius_m;
        assert!((flat[0][3] - (-distance / recipe.moisture_transport_m).exp()).abs() < 1e-12);
        // Both directions to the interior cross a ridge. A low basin behind
        // them remains dry even though its elevation is back at sea level.
        grid.h[0][1] = 4000.0;
        grid.h[0][11] = 4000.0;
        let shadow = moisture_grid(&grid, recipe);
        assert!(
            (shadow[0][3] / flat[0][3] - (-4000.0 / recipe.rainout_height_m).exp()).abs() < 1e-12
        );
        assert_eq!(shadow[0][0], 1.0);
        assert_eq!(shadow, moisture_grid(&grid, recipe));
        grid.h[0].fill(100.0);
        assert!(
            moisture_grid(&grid, recipe)
                .iter()
                .flatten()
                .all(|m| *m == 0.0)
        );
    }

    #[test]
    fn moisture_proxy_preserves_seam_pole_identity_and_validates_si_lengths() {
        let mut grid = HeightGrid::new(
            vec![-90.0, 0.0, 90.0],
            vec![-180.0, -90.0, 0.0, 90.0],
            3_200_000.0,
        );
        grid.h = vec![vec![0.0; 4]; 3];
        grid.h[1][0] = -1.0;
        let moisture = moisture_grid(&grid, SurfaceClimate::default());
        assert!((moisture[1][1] - moisture[1][3]).abs() < 1e-12);
        assert!(moisture.iter().flatten().all(|m| (0.0..=1.0).contains(m)));
        for row in [&moisture[0], &moisture[2]] {
            assert!(row.iter().all(|m| (m - row[0]).abs() < 1e-12));
        }
        for length in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert!(
                SurfaceClimate {
                    moisture_transport_m: length,
                    ..Default::default()
                }
                .validate()
                .is_err()
            );
            assert!(
                SurfaceClimate {
                    rainout_height_m: length,
                    ..Default::default()
                }
                .validate()
                .is_err()
            );
        }
    }

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
        let dist = continentality_metres(&grid, &water);
        assert_eq!(dist[2][0], 0.0);
        // Longitude wraps, so mid-landmass (col 3) is farthest from the ocean.
        assert!(dist[2][3] > dist[2][1], "{:?}", dist[2]);
        // Physical edge lengths: the neighbour cell is one dx_m away.
        let (_, dx_m) = grid.cell_m(2);
        assert!((dist[2][1] - dx_m).abs() < 1.0, "{:?}", dist[2]);
        assert!(dist.iter().flatten().all(|d| d.is_finite()));
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
