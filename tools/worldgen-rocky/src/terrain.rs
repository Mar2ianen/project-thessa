//! Terrain evaluation in stable physical spectral bands.
//!
//! MACRO ~125-2000 km: authored provinces (bands) + landmark features.
//! MESO  ~4-64 km:     relief bands.
//! MICRO ~32 m-2 km:   detail bands.
//!
//! All bands sample 3D deterministic noise on the unit sphere:
//!   q = dir * radius_m / wavelength_m
//! so there is no equirectangular seam, no pole pinching, and physical
//! wavelengths are uniform over the globe.
//!
//! LOD rule: `sample(dir, min_wavelength)` sums exactly the bands with
//! wavelength >= min_wavelength. A coarse sample is therefore always the
//! low-frequency PREFIX of a finer sample, never a different surface.

use crate::{
    features::{PlacedFeature, eval_feature_height_m},
    rng,
    sphere::dir_from_latlon,
};

/// Macro province bands, metres.
pub const MACRO_BANDS_M: [f64; 5] = [2_000_000.0, 1_000_000.0, 500_000.0, 250_000.0, 125_000.0];
/// Meso relief bands, metres.
pub const MESO_BANDS_M: [f64; 5] = [64_000.0, 32_000.0, 16_000.0, 8_000.0, 4_000.0];
/// Micro detail bands, metres.
pub const MICRO_BANDS_M: [f64; 7] = [2000.0, 1000.0, 500.0, 250.0, 125.0, 64.0, 32.0];

/// Recipe knobs (fractions 0..1 unless noted).
#[derive(Debug, Clone, Copy)]
pub struct TerrainKnobs {
    pub mountain_coverage: f64,
    pub crater_density: f64,
    pub volcanism: f64,
    pub erosion: f64,
    pub canyon_strength: f64,
    pub base_roughness_m: f64,
}

impl Default for TerrainKnobs {
    fn default() -> Self {
        Self {
            mountain_coverage: 0.3,
            crater_density: 0.3,
            volcanism: 0.2,
            erosion: 0.4,
            canyon_strength: 0.3,
            base_roughness_m: 150.0,
        }
    }
}

fn band_channel(band_index: usize) -> u32 {
    1000 + band_index as u32 * 131
}

/// One spectral band value in [-amp, +amp].
pub fn eval_band_m(
    seed: u64,
    band_index: usize,
    dir: [f64; 3],
    radius_m: f64,
    wavelength_m: f64,
    amplitude_m: f64,
) -> f64 {
    let q = [
        dir[0] * radius_m / wavelength_m,
        dir[1] * radius_m / wavelength_m,
        dir[2] * radius_m / wavelength_m,
    ];
    amplitude_m * rng::value_noise3(seed, band_channel(band_index), q[0], q[1], q[2])
}

fn macro_amp(wavelength_m: f64, knobs: TerrainKnobs) -> f64 {
    wavelength_m * 0.0012 * (0.4 + knobs.mountain_coverage)
}

pub(crate) fn meso_amp(wavelength_m: f64, knobs: TerrainKnobs) -> f64 {
    wavelength_m * 0.004 * (0.3 + 0.7 * knobs.mountain_coverage) * (1.0 - 0.5 * knobs.erosion)
}

pub(crate) fn micro_amp(wavelength_m: f64, knobs: TerrainKnobs) -> f64 {
    (wavelength_m * 0.03).min(knobs.base_roughness_m.max(1.0) * 0.5)
}

/// Evaluate MACRO band: landmark features + broad provinces (spherical).
pub fn eval_macro_m(features: &[PlacedFeature], lat_deg: f64, lon_deg: f64, radius_m: f64) -> f64 {
    features
        .iter()
        .map(|f| eval_feature_height_m(f, lat_deg, lon_deg, radius_m))
        .sum()
}

/// Macro province noise (spherical bands), without discrete features.
pub fn eval_macro_provinces_m(seed: u64, knobs: TerrainKnobs, dir: [f64; 3], radius_m: f64) -> f64 {
    MACRO_BANDS_M
        .iter()
        .enumerate()
        .map(|(i, wl)| eval_band_m(seed, i, dir, radius_m, *wl, macro_amp(*wl, knobs)))
        .sum()
}

/// Evaluate MESO band: relief at 4-64 km wavelengths (spherical).
pub fn eval_meso_m(
    seed: u64,
    knobs: TerrainKnobs,
    lat_deg: f64,
    lon_deg: f64,
    radius_m: f64,
) -> f64 {
    let dir = dir_from_latlon(lat_deg, lon_deg);
    MESO_BANDS_M
        .iter()
        .enumerate()
        .map(|(i, wl)| eval_band_m(seed, 64 + i, dir, radius_m, *wl, meso_amp(*wl, knobs)))
        .sum()
}

/// Evaluate MICRO band down to `detail_scale_m` minimum wavelength.
/// Smaller `detail_scale_m` ADDS shorter bands; shared bands are identical.
pub fn eval_micro_m(
    seed: u64,
    knobs: TerrainKnobs,
    lat_deg: f64,
    lon_deg: f64,
    radius_m: f64,
    detail_scale_m: f64,
) -> f64 {
    let dir = dir_from_latlon(lat_deg, lon_deg);
    eval_micro_dir_m(seed, knobs, dir, radius_m, detail_scale_m)
}

/// Direction-based micro evaluation (tile/renderer friendly).
pub fn eval_micro_dir_m(
    seed: u64,
    knobs: TerrainKnobs,
    dir: [f64; 3],
    radius_m: f64,
    min_wavelength_m: f64,
) -> f64 {
    MICRO_BANDS_M
        .iter()
        .enumerate()
        .filter(|(_, wl)| **wl >= min_wavelength_m)
        .map(|(i, wl)| eval_band_m(seed, 128 + i, dir, radius_m, *wl, micro_amp(*wl, knobs)))
        .sum()
}

/// Full stack. Pure function of inputs.
pub fn eval_terrain_m(
    seed: u64,
    knobs: TerrainKnobs,
    features: &[PlacedFeature],
    lat_deg: f64,
    lon_deg: f64,
    radius_m: f64,
    detail_scale_m: f64,
) -> f64 {
    let dir = dir_from_latlon(lat_deg, lon_deg);
    eval_macro_m(features, lat_deg, lon_deg, radius_m)
        + eval_macro_provinces_m(seed, knobs, dir, radius_m)
        + eval_meso_m(seed, knobs, lat_deg, lon_deg, radius_m)
        + eval_micro_m(seed, knobs, lat_deg, lon_deg, radius_m, detail_scale_m)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{features::Feature, sphere::dir_from_latlon};

    fn knobs() -> TerrainKnobs {
        TerrainKnobs::default()
    }

    #[test]
    fn empty_features_macro_is_zero() {
        assert_eq!(eval_macro_m(&[], 10.0, 20.0, 3_200_000.0), 0.0);
    }

    #[test]
    fn stack_is_deterministic_finite_and_seed_sensitive() {
        let feats = vec![crate::features::PlacedFeature {
            id: "m".into(),
            seed: 3,
            lat_deg: 10.0,
            lon_deg: 10.0,
            rotation_rad: 0.0,
            feature: Feature::Plateau {
                radius_m: 200_000.0,
                height_m: 1500.0,
            },
        }];
        let a = eval_terrain_m(7, knobs(), &feats, 10.0, 10.0, 3_200_000.0, 250.0);
        assert_eq!(
            a,
            eval_terrain_m(7, knobs(), &feats, 10.0, 10.0, 3_200_000.0, 250.0)
        );
        assert!(a.is_finite());
        let b = eval_terrain_m(8, knobs(), &feats, 10.0, 10.0, 3_200_000.0, 250.0);
        assert!((a - b).abs() > 1e-9);
    }

    #[test]
    fn finer_detail_adds_bands_without_changing_shared_ones() {
        // LOD prefix property: fine == coarse + exactly the shorter bands.
        let dir = dir_from_latlon(5.0, 5.0);
        let coarse = eval_micro_dir_m(7, knobs(), dir, 3_200_000.0, 250.0);
        let fine = eval_micro_dir_m(7, knobs(), dir, 3_200_000.0, 60.0);
        let mut added = 0.0;
        for (i, wl) in MICRO_BANDS_M.iter().enumerate() {
            if *wl < 250.0 && *wl >= 60.0 {
                added += eval_band_m(7, 128 + i, dir, 3_200_000.0, *wl, micro_amp(*wl, knobs()));
            }
        }
        assert!(
            (fine - coarse - added).abs() < 1e-9,
            "{fine} vs {coarse} + {added}"
        );
        assert!((fine - coarse).abs() > 1e-9);
    }

    #[test]
    fn seam_and_poles_are_continuous() {
        // Same physical point via different longitudes => identical height.
        let a = eval_meso_m(7, knobs(), 10.0, 180.0, 3_200_000.0);
        let b = eval_meso_m(7, knobs(), 10.0, -180.0, 3_200_000.0);
        assert_eq!(a, b);
        // Pole is longitude-invariant.
        let p0 = eval_meso_m(7, knobs(), 90.0, 0.0, 3_200_000.0);
        for lon in [-150.0, -45.0, 60.0, 179.0] {
            assert_eq!(p0, eval_meso_m(7, knobs(), 90.0, lon, 3_200_000.0));
        }
    }

    #[test]
    fn finer_detail_scale_changes_micro_but_not_macro() {
        let feats: Vec<crate::features::PlacedFeature> = vec![];
        let m1 = eval_macro_m(&feats, 5.0, 5.0, 3_200_000.0);
        let m2 = eval_macro_m(&feats, 5.0, 5.0, 3_200_000.0);
        assert_eq!(m1, m2);
        let micro1 = eval_micro_m(7, knobs(), 5.0, 5.0, 3_200_000.0, 250.0);
        let micro2 = eval_micro_m(7, knobs(), 5.0, 5.0, 3_200_000.0, 60.0);
        assert!((micro1 - micro2).abs() > 1e-9);
    }
}
