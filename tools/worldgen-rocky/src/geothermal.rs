//! Geothermal activity field: localized provinces, never uniform heating.
//!
//! The global mean tidal flux (~0.146 W/m2) is NOT spread evenly. Activity
//! concentrates into major provinces (rift/volcanic), secondary fields and
//! many weak spots. Consumers: hot springs, fumaroles, sulfur, hydrothermal
//! deposits, local snow suppression.

use crate::rng;

#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct ProvinceRecipe {
    pub major_min: u32,
    pub major_max: u32,
    pub secondary_min: u32,
    pub secondary_max: u32,
}

impl Default for ProvinceRecipe {
    fn default() -> Self {
        Self {
            major_min: 3,
            major_max: 3,
            secondary_min: 6,
            secondary_max: 6,
        }
    }
}

impl ProvinceRecipe {
    pub fn validate(self) -> Result<(), String> {
        if self.major_min > self.major_max
            || self.secondary_min > self.secondary_max
            || self.major_max > 32
            || self.secondary_max > 128
        {
            return Err(
                "geothermal province counts need ordered ranges (major <= 32, secondary <= 128)"
                    .into(),
            );
        }
        Ok(())
    }

    pub fn place(
        self,
        seed: u64,
        hot_spots: &[(f64, f64)],
    ) -> Result<Vec<GeothermalProvince>, String> {
        self.validate()?;
        let count = |min: u32, max: u32, channel| {
            min + (rng::hash01(seed, channel, 0, 0) * f64::from(max - min + 1)) as u32
        };
        let lats: Vec<_> = hot_spots.iter().map(|p| p.0).collect();
        let lons: Vec<_> = hot_spots.iter().map(|p| p.1).collect();
        Ok(place_provinces(
            seed,
            count(self.major_min, self.major_max, 507),
            count(self.secondary_min, self.secondary_max, 517),
            &lats,
            &lons,
        ))
    }
}

/// One geothermal province. Flux in W/m2 at the center.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct GeothermalProvince {
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub radius_m: f64,
    pub flux_w_m2: f64,
}

/// Activity 0..1 at a site from provinces + volcanic/rift proximity.
/// `volcanic01`/`rift01` are 0..1 proximities supplied by the caller.
pub fn geothermal_activity(
    provinces: &[GeothermalProvince],
    lat_deg: f64,
    lon_deg: f64,
    radius_m: f64,
    volcanic01: f64,
    rift01: f64,
) -> f64 {
    let mut flux = 0.0;
    for p in provinces {
        let dlat = (lat_deg - p.lat_deg).to_radians() * radius_m;
        let mut dlon = (lon_deg - p.lon_deg).to_radians();
        while dlon > std::f64::consts::PI {
            dlon -= 2.0 * std::f64::consts::PI;
        }
        while dlon < -std::f64::consts::PI {
            dlon += 2.0 * std::f64::consts::PI;
        }
        let dlon_m = dlon * radius_m * lat_deg.to_radians().cos().max(0.05);
        let d = (dlat.powi(2) + dlon_m.powi(2)).sqrt();
        flux += p.flux_w_m2 * (-(d / p.radius_m).powi(2)).exp();
    }
    // Secondary texture near volcanic/rift geology even without a province.
    flux += 0.25 * volcanic01 + 0.20 * rift01;
    // Normalize: 3 W/m2 local => fully active.
    (flux / 3.0).clamp(0.0, 1.0)
}

/// Deterministic province placement from a seed:
/// `major` strong provinces, `secondary` medium fields. Weak spots are
/// implicit noise handled by callers via `rng`.
pub fn place_provinces(
    seed: u64,
    major: u32,
    secondary: u32,
    prefer_lats: &[f64],
    prefer_lons: &[f64],
) -> Vec<GeothermalProvince> {
    let mut out = Vec::new();
    for i in 0..major {
        let k = i as i64;
        // A preferred site is a coordinate pair, not independently sampled
        // latitude/longitude lists (which could move heat to another region).
        let pair_count = prefer_lats.len().min(prefer_lons.len());
        let site = (pair_count > 0)
            .then(|| (rng::hash01(seed, 501, k, 0) * pair_count as f64) as usize % pair_count);
        let lat = if prefer_lats.is_empty() {
            rng::hash11(seed, 501, k, 0) * 50.0
        } else {
            prefer_lats[site.unwrap_or_else(|| {
                (rng::hash01(seed, 501, k, 0) * prefer_lats.len() as f64) as usize
                    % prefer_lats.len()
            })] + rng::hash11(seed, 502, k, 0) * 12.0
        };
        let lon = if prefer_lons.is_empty() {
            rng::hash11(seed, 503, k, 1) * 150.0
        } else {
            prefer_lons[site.unwrap_or_else(|| {
                (rng::hash01(seed, 503, k, 1) * prefer_lons.len() as f64) as usize
                    % prefer_lons.len()
            })] + rng::hash11(seed, 504, k, 1) * 15.0
        };
        out.push(GeothermalProvince {
            lat_deg: lat.clamp(-70.0, 70.0),
            lon_deg: (lon + 180.0).rem_euclid(360.0) - 180.0,
            radius_m: 250_000.0 + rng::hash01(seed, 505, k, 2) * 350_000.0,
            flux_w_m2: 2.0 + rng::hash01(seed, 506, k, 3) * 3.0,
        });
    }
    for i in 0..secondary {
        let k = i as i64 + 100;
        out.push(GeothermalProvince {
            lat_deg: (rng::hash11(seed, 511, k, 0) * 60.0).clamp(-70.0, 70.0),
            lon_deg: rng::hash11(seed, 512, k, 1) * 170.0,
            radius_m: 80_000.0 + rng::hash01(seed, 513, k, 2) * 120_000.0,
            flux_w_m2: 0.6 + rng::hash01(seed, 514, k, 3) * 0.9,
        });
    }
    out
}

/// Default province set: 3 major preferring volcanic/rift landmark positions,
/// 6 secondary spread globally. Deterministic from (seed, feature sites).
pub fn default_provinces(seed: u64, hot_spots: &[(f64, f64)]) -> Vec<GeothermalProvince> {
    let mut lats: Vec<f64> = hot_spots.iter().map(|p| p.0).collect();
    let mut lons: Vec<f64> = hot_spots.iter().map(|p| p.1).collect();
    if lats.is_empty() {
        lats = vec![10.0, -20.0, 35.0];
        lons = vec![30.0, -60.0, 150.0];
    }
    place_provinces(seed, 3, 6, &lats, &lons)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn province_center_is_hot_edge_is_not() {
        let provinces = vec![GeothermalProvince {
            lat_deg: 0.0,
            lon_deg: 0.0,
            radius_m: 300_000.0,
            flux_w_m2: 4.0,
        }];
        let hot = geothermal_activity(&provinces, 0.0, 0.0, 3_200_000.0, 0.0, 0.0);
        let cold = geothermal_activity(&provinces, 0.0, 60.0, 3_200_000.0, 0.0, 0.0);
        assert!(hot > 0.9, "{hot}");
        assert!(cold < 0.05, "{cold}");
    }

    #[test]
    fn placement_counts_and_determinism() {
        let a = place_provinces(7, 3, 6, &[], &[]);
        let b = place_provinces(7, 3, 6, &[], &[]);
        assert_eq!(a.len(), 9);
        assert_eq!(a[0].lat_deg, b[0].lat_deg);
        let c = place_provinces(8, 3, 6, &[], &[]);
        assert!((a[0].lat_deg - c[0].lat_deg).abs() > 1e-9);
    }

    #[test]
    fn preferred_sites_keep_coordinate_pairs_and_do_not_flip_hemispheres() {
        let sites = [(-40.0, -160.0), (40.0, 160.0)];
        let provinces = place_provinces(7, 32, 0, &[-40.0, 40.0], &[-160.0, 160.0]);
        for province in provinces {
            assert!(
                sites.iter().any(|&(lat, lon)| {
                    let dlon = (province.lon_deg - lon + 180.0).rem_euclid(360.0) - 180.0;
                    (province.lat_deg - lat).abs() <= 12.0 && dlon.abs() <= 15.0
                }),
                "displaced province: {province:?}"
            );
        }
    }
}
