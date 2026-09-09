//! Terrain evaluation in three explicit physical scale bands.
//!
//! MACRO ~100-2000 km: authored landmark features + broad provinces.
//! MESO  ~5-200 km:    secondary relief tied to recipe + geology.
//! MICRO ~0.01-5 km:   deterministic small detail, physical wavelengths.
//!
//! Nothing here knows about pixels. Frequencies are 1/metres.

use crate::{
    features::{PlacedFeature, eval_feature_height_m},
    rng,
};

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

/// Evaluate MACRO band: sum of landmark features + broad provinces.
pub fn eval_macro_m(features: &[PlacedFeature], lat_deg: f64, lon_deg: f64, radius_m: f64) -> f64 {
    features
        .iter()
        .map(|f| eval_feature_height_m(f, lat_deg, lon_deg, radius_m))
        .sum()
}

/// Evaluate MESO band: relief at 5-200 km wavelengths, modulated by knobs.
/// Deterministic fBm sampled in physical surface metres.
pub fn eval_meso_m(
    seed: u64,
    knobs: TerrainKnobs,
    lat_deg: f64,
    lon_deg: f64,
    radius_m: f64,
) -> f64 {
    // Surface metres from angular coords (equirect local approx is fine for noise input).
    let x = lon_deg.to_radians() * radius_m;
    let y = lat_deg.to_radians() * radius_m;
    // 40 km base wavelength: two octaves cover ~5-80 km.
    let hills = rng::fbm(seed, 101, x / 40_000.0, y / 40_000.0, 2);
    let amp = 1200.0 * (0.3 + 0.7 * knobs.mountain_coverage) * (1.0 - 0.5 * knobs.erosion);
    hills * amp
}

/// Evaluate MICRO band: centimetres-to-km detail at explicit wavelengths.
pub fn eval_micro_m(
    seed: u64,
    knobs: TerrainKnobs,
    lat_deg: f64,
    lon_deg: f64,
    radius_m: f64,
    detail_scale_m: f64,
) -> f64 {
    let scale = detail_scale_m.clamp(0.05, 5000.0);
    let x = lon_deg.to_radians() * radius_m / scale;
    let y = lat_deg.to_radians() * radius_m / scale;
    let amp = (0.35 * scale.min(500.0)).min(knobs.base_roughness_m.max(1.0));
    rng::fbm(seed, 102, x, y, 4) * amp
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
    eval_macro_m(features, lat_deg, lon_deg, radius_m)
        + eval_meso_m(seed, knobs, lat_deg, lon_deg, radius_m)
        + eval_micro_m(seed, knobs, lat_deg, lon_deg, radius_m, detail_scale_m)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::features::Feature;

    fn knobs() -> TerrainKnobs {
        TerrainKnobs::default()
    }

    #[test]
    fn empty_features_macro_is_zero() {
        assert_eq!(eval_macro_m(&[], 10.0, 20.0, 3_200_000.0), 0.0);
    }

    #[test]
    fn stack_is_deterministic_finite_and_seed_sensitive() {
        let feats = vec![PlacedFeature {
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
    fn finer_detail_scale_changes_micro_but_not_macro() {
        let feats: Vec<PlacedFeature> = vec![];
        let m1 = eval_macro_m(&feats, 5.0, 5.0, 3_200_000.0);
        let m2 = eval_macro_m(&feats, 5.0, 5.0, 3_200_000.0);
        assert_eq!(m1, m2);
        let micro1 = eval_micro_m(7, knobs(), 5.0, 5.0, 3_200_000.0, 250.0);
        let micro2 = eval_micro_m(7, knobs(), 5.0, 5.0, 3_200_000.0, 60.0);
        assert!((micro1 - micro2).abs() > 1e-9);
    }
}
