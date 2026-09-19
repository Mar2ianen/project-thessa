//! Continuous surface materials derived from climate and terrain.
//! Biome/feature IDs remain discrete semantic data; they are not painted as
//! flat colour disks. No sunlight is baked into the albedo.
use crate::{
    field::{PlanetField, TerrainSample},
    rng,
};

#[derive(Clone, Copy, Debug)]
pub struct SurfaceAppearance {
    pub albedo_srgb: [f32; 3],
    pub roughness: f32,
    pub vegetation: f32,
    pub snow: f32,
}
fn mix(a: [f64; 3], b: [f64; 3], t: f64) -> [f64; 3] {
    std::array::from_fn(|i| a[i] + (b[i] - a[i]) * t.clamp(0.0, 1.0))
}
pub(crate) fn smooth(a: f64, b: f64, v: f64) -> f64 {
    let t = ((v - a) / (b - a)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Deterministic material-only grain. This is deliberately separate from the
/// canonical height field: it gives close terrain a readable surface pattern
/// without inventing collision relief or changing authoritative queries.
///
/// Range is roughly [-1, 1] (sum of weighted value-noise octaves).
pub fn surface_grain(field: &PlanetField, dir: [f64; 3]) -> f64 {
    [(8.0, 0.42), (32.0, 0.32), (128.0, 0.20), (512.0, 0.10)]
        .into_iter()
        .enumerate()
        .map(|(band, (scale, weight))| {
            weight
                * rng::value_noise3(
                    field.params.seed,
                    741 + band as u32 * 17,
                    dir[0] * field.params.radius_m / scale,
                    dir[1] * field.params.radius_m / scale,
                    dir[2] * field.params.radius_m / scale,
                )
        })
        .sum()
}

/// The normal-map path must discard grain finer than two texels, otherwise a
/// low-resolution tile aliases its own detail instead of filtering it.
pub(crate) fn surface_grain_height(
    field: &PlanetField,
    dir: [f64; 3],
    texel_wavelength_m: f64,
) -> f64 {
    [(8.0, 0.42), (32.0, 0.32), (128.0, 0.20), (512.0, 0.10)]
        .into_iter()
        .enumerate()
        .filter(|(_, (scale, _))| *scale >= texel_wavelength_m * 2.0)
        .map(|(band, (scale, weight))| {
            let q = dir.map(|v| v * field.params.radius_m / scale);
            weight
                * 2.0
                * rng::value_noise3(field.params.seed, 741 + band as u32 * 17, q[0], q[1], q[2])
        })
        .sum()
}

pub fn surface_appearance(
    field: &PlanetField,
    sample: &TerrainSample,
    dir: [f64; 3],
) -> SurfaceAppearance {
    let noise = |channel, scale, octaves| {
        rng::fbm3(
            field.params.seed,
            channel,
            dir[0] * field.params.radius_m / scale,
            dir[1] * field.params.radius_m / scale,
            dir[2] * field.params.radius_m / scale,
            octaves,
        )
    };
    let regional = noise(731, 360_000.0, 3);
    let variation = noise(733, 65_000.0, 3);
    // Independent micro-relief noise (24 m + 12 m octaves): deliberately NOT
    // the cover grain — sharing one signal for threshold patches and final
    // brightness partially cancels (opposite signs), muting both.
    let micro = noise(739, 24.0, 2);
    let grain = surface_grain(field, dir);
    let h = sample.height_m;
    // Snow cover breaks into drifts and thaw patches: the grain rides the
    // threshold (±0.7 grain ~= ±4 K) so a uniform sub-zero plain still reads
    // as structured snow instead of a flat fill. Classification inputs
    // (height/moisture/temperature/slope) are untouched — only the visual
    // coverage and the final albedo carry the pattern.
    let snow = smooth(
        276.0,
        264.0,
        sample.temperature_k + regional * 5.0 + variation * 2.0 + grain * 6.0,
    );
    if h < 0.0 {
        let shelf = (-h / 500.0).clamp(0.0, 1.0).sqrt();
        let c = mix([0.12, 0.42, 0.43], [0.018, 0.095, 0.19], shelf);
        let c = mix(c, [0.014, 0.044, 0.11], smooth(600.0, 6000.0, -h));
        let ice = smooth(263.0, 253.0, sample.temperature_k + regional * 6.0);
        return SurfaceAppearance {
            albedo_srgb: mix(c, [0.75, 0.84, 0.86], ice)
                .map(|x| (x * (1.0 + grain * 0.10)).clamp(0.0, 1.0) as f32),
            roughness: (0.16 + 0.58 * ice) as f32,
            vegetation: 0.0,
            snow: ice as f32,
        };
    }
    let moisture = (sample.moisture01 + regional * 0.24).clamp(0.0, 1.0);
    let vegetation = smooth(0.25, 0.60, moisture)
        * smooth(268.0, 283.0, sample.temperature_k)
        * (1.0 - smooth(0.35, 0.85, sample.slope_hint));
    let dry = mix(
        [0.57, 0.46, 0.29],
        [0.39, 0.37, 0.30],
        smooth(0.1, 0.55, moisture),
    );
    let green = mix(
        [0.32, 0.40, 0.21],
        [0.10, 0.23, 0.15],
        smooth(0.50, 0.85, moisture),
    );
    let mut color = mix(dry, green, vegetation);
    let rock = smooth(1900.0, 4400.0, h).max(smooth(0.35, 0.8, sample.slope_hint));
    color = mix(color, [0.36, 0.37, 0.36], rock);
    // Beach is a height band, not a circular feature footprint.
    color = mix([0.66, 0.64, 0.48], color, smooth(3.0, 45.0, h));
    color = color.map(|c| c * (1.0 + variation * 0.14 + regional * 0.08 + grain * 0.24));
    let snow_cover = snow * (1.0 - smooth(0.7, 1.4, sample.slope_hint) * 0.8);
    color = mix(color, [0.88, 0.92, 0.94], snow_cover);
    // Frost on grass: the sub-zero band where snow cover is partial or thin.
    // Below ~258 K snow does the talking; above ~278 K there is nothing to
    // freeze. In between, moisture-gated patches of pale crystals settle over
    // whatever the cover left — the classic -7 C morning. Classification
    // inputs are untouched; only the visual albedo/roughness carry it.
    let frost_band = smooth(278.0, 270.0, sample.temperature_k)
        * (1.0 - smooth(262.0, 254.0, sample.temperature_k));
    let frost = frost_band
        * (0.25 + 0.75 * sample.moisture01)
        * smooth(-0.3, 0.5, micro)
        // White on white is invisible: deep snow needs no frost, and the mix
        // below would only iron out the snow's own micro-relief.
        * (1.0 - snow_cover);
    color = mix(color, [0.78, 0.83, 0.88], (frost * 0.75).clamp(0.0, 1.0));
    // Micro-relief brightness on the final albedo: overlapping covers (snow,
    // rock) would otherwise mute the pre-mix grain to invisibility. Uses the
    // independent micro signal, so it adds instead of fighting the patches.
    // ±8% stays clear of the sRGB ceiling on bright snow.
    color = color.map(|c| (c * (1.0 + micro * 0.08)).clamp(0.0, 1.0));
    SurfaceAppearance {
        albedo_srgb: color.map(|c| c.clamp(0.0, 1.0) as f32),
        // Frost crystals glitter: pull roughness down where they settle.
        roughness: (0.92 - 0.30 * frost.clamp(0.0, 1.0)) as f32,
        vegetation: vegetation as f32,
        snow: snow as f32,
    }
}

#[cfg(test)]
mod frost_tests {
    use super::*;
    use crate::biomes::{Biome, FeatureTag, Geology};

    fn field() -> PlanetField {
        let recipe =
            toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml")).unwrap();
        crate::field::field_from_manifest(&crate::spec_recipe::manifest_from_spec(&recipe).unwrap())
            .unwrap()
    }

    fn sample(temperature_k: f64, moisture01: f64) -> TerrainSample {
        TerrainSample {
            height_m: 500.0,
            macro_height_m: 500.0,
            procedural_height_m: 0.0,
            biome: Biome::RockyPlain,
            geology: Geology::ContinentalCrust,
            tag0: None,
            tag1: None,
            slope_hint: 0.1,
            geothermal_flux_w_m2: 0.08,
            moisture01,
            temperature_k,
            continentality01: 0.5,
            eclipse_exposure01: 1.0,
        }
    }

    fn dirs() -> Vec<[f64; 3]> {
        // Fixed spread of unit directions: noise is identical across samples
        // at the same dir, so temperature/moisture differences are pure.
        (0..8)
            .map(|i| {
                let a = i as f64 * std::f64::consts::TAU / 8.0;
                [a.cos(), 0.2, a.sin()]
            })
            .map(|d| {
                let l = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
                [d[0] / l, d[1] / l, d[2] / l]
            })
            .collect()
    }

    #[test]
    fn frost_settles_only_in_the_subzero_band_and_leaves_classification() {
        let field = field();
        for dir in dirs() {
            // Warm and deep-cold: no frost band, baseline roughness exactly.
            for temp in [300.0, 240.0] {
                let out = surface_appearance(&field, &sample(temp, 1.0), dir);
                assert_eq!(out.roughness, 0.92, "no frost outside the band at {temp} K");
            }
            // Classification inputs untouched by the visual frost.
            assert_eq!(
                surface_appearance(&field, &sample(300.0, 1.0), dir).snow,
                0.0
            );
            assert_eq!(
                surface_appearance(&field, &sample(240.0, 1.0), dir).snow,
                1.0
            );
        }
        // In the band (-7 C morning) frost must actually settle: compare
        // moist vs dry at the same dirs. Snow cover is moisture-independent,
        // so the means differ only through the frost moisture gate
        // (0.25 + 0.75 * moisture) — a 4x cleaner signal than an absolute
        // threshold against the noisy snow edge.
        // Probe at the warm edge of the band (270 K): the frost band is
        // still fully 1 there while the snow edge (~270 K center) has only
        // half closed, so the moisture gate reads through snow suppression.
        let mean = |moisture: f64| {
            dirs()
                .iter()
                .map(|dir| {
                    surface_appearance(&field, &sample(270.0, moisture), *dir).roughness as f64
                })
                .sum::<f64>()
                / 8.0
        };
        let (moist, dry) = (mean(1.0), mean(0.0));
        assert!(
            moist < dry - 0.01,
            "moisture must gate frost glitter, moist={moist:.4} dry={dry:.4}"
        );
        // Frost never raises roughness anywhere.
        for dir in dirs() {
            for temp in [250.0, 260.0, 266.0, 272.0, 280.0] {
                let out = surface_appearance(&field, &sample(temp, 1.0), dir);
                assert!(
                    out.roughness <= 0.92,
                    "frost only lowers roughness at {temp} K"
                );
            }
        }
    }
}
