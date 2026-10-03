//! Continuous surface materials derived from climate and terrain.
//! Biome/feature IDs remain discrete semantic data; they are not painted as
//! flat colour disks. No sunlight is baked into the albedo.
use crate::{
    biomes::Geology,
    field::{PlanetField, TerrainSample},
    rng,
};

/// Approximate unlit substrate reflectance, not the biome/debug colour legend.
/// Weathered loose cover and exposed rock remain separate optical materials.
fn substrate_albedo(geology: Geology) -> ([f64; 3], [f64; 3]) {
    match geology {
        Geology::Basaltic | Geology::OceanicCrust => ([0.20, 0.21, 0.22], [0.32, 0.29, 0.25]),
        Geology::FelsicHighland => ([0.56, 0.54, 0.49], [0.51, 0.46, 0.37]),
        Geology::ContinentalCrust => ([0.42, 0.43, 0.42], [0.43, 0.36, 0.27]),
        Geology::Sedimentary => ([0.48, 0.43, 0.35], [0.49, 0.40, 0.28]),
        Geology::ImpactBreccia => ([0.34, 0.32, 0.30], [0.40, 0.34, 0.28]),
        Geology::Evaporite => ([0.77, 0.75, 0.69], [0.64, 0.59, 0.47]),
        Geology::GlacialTill => ([0.40, 0.43, 0.44], [0.48, 0.46, 0.40]),
        Geology::Regolith => ([0.35, 0.34, 0.31], [0.44, 0.37, 0.28]),
        Geology::Hydrothermal => ([0.39, 0.35, 0.29], [0.48, 0.37, 0.22]),
    }
}

#[derive(Clone, Copy, Debug)]
pub struct SurfaceAppearance {
    pub albedo_srgb: [f32; 3],
    pub roughness: f32,
    pub vegetation: f32,
    pub snow: f32,
    /// Supply/geometry-limited visual cover, separate from cold suitability.
    pub snow_cover: f32,
    /// Optical frost mixing weight; zero outside the frost temperature band.
    pub frost_cover: f32,
}
fn land_roughness(vegetation: f64, exposed: f64, damp: f64, texture: f64, frost: f64) -> f32 {
    (0.88 + vegetation * 0.06 - exposed * 0.18 - damp * (1.0 - vegetation) * 0.12
        + texture * exposed * 0.04
        - 0.25 * frost.clamp(0.0, 1.0))
    .clamp(0.45, 0.98) as f32
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
    surface_grain_filtered(field, dir, 0.0)
}

fn band_weight(scale_m: f64, texel_m: f64) -> f64 {
    1.0 - smooth(scale_m * 0.25, scale_m * 0.5, texel_m)
}

fn surface_grain_filtered(field: &PlanetField, dir: [f64; 3], texel_m: f64) -> f64 {
    [(8.0, 0.42), (32.0, 0.32), (128.0, 0.20), (512.0, 0.10)]
        .into_iter()
        .enumerate()
        .filter(|(_, (scale, _))| band_weight(*scale, texel_m) > 0.0)
        .map(|(band, (scale, weight))| {
            weight
                * band_weight(scale, texel_m)
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
    surface_appearance_filtered(field, sample, dir, 0.0)
}

/// Appearance-only spectral filtering; canonical height and semantics stay fixed.
pub fn surface_appearance_filtered(
    field: &PlanetField,
    sample: &TerrainSample,
    dir: [f64; 3],
    texel_m: f64,
) -> SurfaceAppearance {
    let noise = |channel, scale, octaves| {
        if texel_m > 0.0 {
            let mut sum = 0.0;
            let mut norm = 0.0;
            let (mut amplitude, mut frequency) = (1.0, 1.0);
            for octave in 0..octaves {
                let weight = band_weight(scale / frequency, texel_m);
                if weight > 0.0 {
                    let q = dir.map(|v| v * field.params.radius_m / scale * frequency);
                    sum += amplitude
                        * weight
                        * rng::value_noise3(
                            field.params.seed,
                            channel + octave * 7919,
                            q[0],
                            q[1],
                            q[2],
                        );
                }
                norm += amplitude;
                amplitude *= 0.5;
                frequency *= 2.03;
            }
            return sum / norm;
        }
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
    // Geometry and the actual frozen displacement, not another random mask.
    // Curvature is measured at 256 m by material consumers; scaling it by
    // that span yields a dimensionless change in slope. Displacement remains
    // regional evidence, not a resolved local sediment thickness/inventory.
    let crest = smooth(0.02, 0.25, -sample.curvature_per_m * 256.0);
    let hollow = smooth(0.02, 0.25, sample.curvature_per_m * 256.0);
    let eroded = smooth(0.0, 40.0, -sample.erosion_displacement_m);
    let deposited = smooth(
        0.0,
        40.0,
        sample.erosion_displacement_m.max(sample.local_deposition_m),
    );
    let gentle = 1.0 - smooth(0.25, 0.7, sample.slope_hint);
    let loose_cover = gentle * hollow.max(deposited);
    // Regional displacement is not a local snow inventory. Curvature may
    // bias exposure weakly, but must not strip every convex crest into an
    // artificial contour line; actual steepness remains the main exposure.
    let exposed =
        smooth(0.35, 0.85, sample.slope_hint).max(crest * eroded * (1.0 - loose_cover) * 0.15);
    // Accumulation smooths the small material bands; exposed rock retains
    // them. This changes appearance only, never collision/height or biomes.
    let texture_gain = 1.0 - loose_cover * 0.65;
    let micro = noise(739, 24.0, 2) * texture_gain;
    let grain = surface_grain_filtered(field, dir, texel_m) * texture_gain;
    // Weathering/cover structure needs intermediate physical scales; tiny
    // grain alone disappears into the average at a survey camera. Exposures
    // and loose deposits do not receive the same uniformly smeared pattern.
    let weathering = noise(751, 384.0, 3);
    let outcrop_texture = noise(757, 96.0, 3);
    let cover_texture = noise(761, 192.0, 3);
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
            snow_cover: ice as f32,
            frost_cover: 0.0,
        };
    }
    let moisture = (sample.moisture01 + regional * 0.24).clamp(0.0, 1.0);
    let ecological_cover = crate::ecology::vegetation_cover(sample, sample.slope_hint);
    let vegetation = ecological_cover.total();
    let (stone, weathered_soil) = substrate_albedo(sample.geology);
    let soil = weathered_soil.map(|c| c * (1.0 + weathering * 0.18));
    let damp = smooth(0.35, 0.95, moisture);
    let dry = soil.map(|c| c * (1.0 - damp * 0.25));
    // The existing ecology state distinguishes canopy, ground plants and reeds.
    // Do not collapse all three into one full-coverage green paint bucket.
    // These are filtered optical cover proxies, not rendered tree geometry.
    let grass = mix([0.33, 0.35, 0.19], [0.20, 0.29, 0.16], damp);
    let canopy = mix([0.18, 0.26, 0.13], [0.09, 0.19, 0.12], damp);
    let reeds = mix([0.31, 0.33, 0.18], [0.18, 0.27, 0.16], damp);
    let plant = std::array::from_fn(|i| {
        (grass[i] * ecological_cover.ground01
            + canopy[i] * ecological_cover.canopy01
            + reeds[i] * ecological_cover.reeds01)
            / vegetation.max(1e-9)
    });
    let plant = plant.map(|c| c * (1.0 + cover_texture * 0.22));
    let mut color = mix(dry, plant, vegetation);
    // Saturated lowland soil / vegetation, not invented pond geometry. The
    // regional drainage source is continuous; local slopes reject cliff faces.
    let wetland = sample.wetland_potential01 * (1.0 - smooth(0.02, 0.1, sample.slope_hint));
    let wet_ground = mix([0.20, 0.23, 0.16], [0.10, 0.26, 0.16], vegetation);
    color = mix(color, wet_ground, wetland);
    // Elevation alone cannot replace every highland substrate with the same
    // grey rock. Actual exposure and retained ecological cover choose the mix.
    let rock = exposed.max(smooth(1900.0, 4400.0, h) * (1.0 - vegetation) * (1.0 - loose_cover));
    let stone = stone.map(|c| c * (1.0 + outcrop_texture * 0.20 + weathering * 0.12));
    color = mix(color, stone, rock);
    // Gentle concave/depositional ground carries loose weathered material;
    // steep convex outcrops retain the underlying rock. No slope-independent
    // white salt overlay is introduced.
    color = mix(color, soil, loose_cover * (1.0 - vegetation) * 0.55);
    // Beach is a height band, not a circular feature footprint.
    color = mix([0.66, 0.64, 0.48], color, smooth(3.0, 45.0, h));
    color = color.map(|c| c * (1.0 + variation * 0.14 + regional * 0.08 + grain * 0.24));
    // A cold dry plateau cannot acquire the same thick cover as a snowy wet
    // mountain. One annual precipitation budget, converted from water to
    // settled snow (300 kg/m³), provides a static availability proxy. The
    // source's 32 m relief amplitude is the cover-depth scale, not a colour
    // multiplier. This is not a dynamic snowpack/seasonal weather solver.
    let snow_depth_m = field.params.surface_climate.precipitation_m_yr
        * sample.moisture01.powi(2)
        * (1.0 - smooth(258.0, 278.0, sample.temperature_k))
        * (1000.0 / 300.0);
    let roughness_depth_m = crate::terrain::micro_amp(32.0, field.params.knobs.into()).max(0.01);
    let snow_available = (snow_depth_m / roughness_depth_m).clamp(0.0, 1.0);
    let snow_cover = (snow * snow_available * (1.0 + hollow * moisture * 0.3)).clamp(0.0, 1.0)
        * (1.0 - exposed * 0.9);
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
        roughness: land_roughness(vegetation, exposed, damp, outcrop_texture, frost),
        vegetation: vegetation as f32,
        snow: snow as f32,
        snow_cover: snow_cover as f32,
        frost_cover: (frost * 0.75).clamp(0.0, 1.0) as f32,
    }
}

#[cfg(test)]
mod frost_tests {
    use super::*;

    #[test]
    fn cold_dry_ground_does_not_receive_unlimited_snow_cover() {
        let field = field();
        for dir in dirs() {
            let dry = sample(268.0, 0.12);
            let wet = sample(268.0, 0.9);
            let a = surface_appearance(&field, &dry, dir);
            let b = surface_appearance(&field, &wet, dir);
            assert!(
                a.snow_cover < b.snow_cover * 0.1,
                "snow must depend on its precipitation supply: dry={} wet={}",
                a.snow_cover,
                b.snow_cover
            );
            assert_eq!(
                a.snow, b.snow,
                "climate suitability remains separate from finite cover"
            );
        }
    }

    #[test]
    fn exposed_substrates_keep_geological_reflectance_instead_of_one_grey_fill() {
        let field = field();
        for dir in dirs() {
            let mut basalt = sample(285.0, 0.0);
            basalt.slope_hint = 1.0;
            basalt.geology = Geology::Basaltic;
            let mut felsic = basalt.clone();
            felsic.geology = Geology::FelsicHighland;
            let a = surface_appearance(&field, &basalt, dir);
            let b = surface_appearance(&field, &felsic, dir);
            assert!(a.albedo_srgb.iter().sum::<f32>() + 0.4 < b.albedo_srgb.iter().sum::<f32>());
            assert_eq!(basalt.height_m, felsic.height_m);
            assert_eq!(a.vegetation, b.vegetation);
            assert!((0.45..=0.98).contains(&a.roughness));
        }
    }

    #[test]
    fn intermediate_material_patterns_are_filtered_not_aliased_at_coarse_texels() {
        let field = field();
        let mut sample = sample(285.0, 0.0);
        sample.slope_hint = 0.8;
        let mut fine = Vec::new();
        let mut coarse = Vec::new();
        for i in 0..128 {
            let dir = crate::sphere::dir_from_latlon(23.0, 165.0 + i as f64 * 0.0001);
            fine.push(surface_appearance_filtered(&field, &sample, dir, 2.0).albedo_srgb[0] as f64);
            coarse.push(
                surface_appearance_filtered(&field, &sample, dir, 10000.0).albedo_srgb[0] as f64,
            );
        }
        let spread = |values: &[f64]| {
            values.iter().copied().fold(f64::NEG_INFINITY, f64::max)
                - values.iter().copied().fold(f64::INFINITY, f64::min)
        };
        assert!(
            spread(&fine) > 0.005,
            "resolved rock needs material structure"
        );
        assert!(
            spread(&coarse) < spread(&fine) * 0.1,
            "coarse pixels must not retain unresolved mottling"
        );
    }
    use crate::biomes::{Biome, Geology};

    fn field() -> PlanetField {
        let recipe =
            toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml")).unwrap();
        crate::field::field_from_manifest(&crate::spec_recipe::manifest_from_spec(&recipe).unwrap())
            .unwrap()
    }

    #[test]
    fn coarse_appearance_filters_unresolved_grain_without_changing_semantics() {
        let field = field();
        let dir = crate::sphere::dir_from_latlon(20.0, 40.0);
        let sample = field.sample_surface(dir, 32.0);
        let fine = surface_appearance(&field, &sample, dir);
        let zero = surface_appearance_filtered(&field, &sample, dir, 0.0);
        assert_eq!(fine.albedo_srgb, zero.albedo_srgb);
        assert_eq!(fine.roughness, zero.roughness);
        assert_eq!(surface_grain_filtered(&field, dir, 1000.0), 0.0);
        let before = (
            sample.height_m,
            sample.biome,
            sample.geology,
            sample.temperature_k,
        );
        let coarse = surface_appearance_filtered(&field, &sample, dir, 5000.0);
        assert!(
            coarse
                .albedo_srgb
                .iter()
                .all(|v| v.is_finite() && (0.0..=1.0).contains(v))
        );
        assert_eq!(
            before,
            (
                sample.height_m,
                sample.biome,
                sample.geology,
                sample.temperature_k
            )
        );
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
            curvature_per_m: 0.0,
            erosion_displacement_m: 0.0,
            local_deposition_m: 0.0,
            geothermal_flux_w_m2: 0.08,
            moisture01,
            wetland_potential01: 0.0,
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
    fn geomorphology_exposes_eroded_crests_and_smooths_depositional_hollows() {
        let field = field();
        for dir in dirs() {
            let mut crest = sample(260.0, 0.7);
            crest.curvature_per_m = -0.002;
            crest.erosion_displacement_m = -100.0;
            let mut hollow = crest.clone();
            hollow.curvature_per_m = 0.002;
            hollow.erosion_displacement_m = 100.0;
            let a = surface_appearance(&field, &crest, dir);
            let b = surface_appearance(&field, &hollow, dir);
            let difference = b.albedo_srgb.iter().sum::<f32>() - a.albedo_srgb.iter().sum::<f32>();
            assert!(
                difference > 0.03 && difference < 0.5,
                "subtle accumulation, not a dark contour: {difference}"
            );
            assert_eq!(crest.height_m, hollow.height_m);
            assert_eq!(crest.biome, hollow.biome);
            assert_eq!(crest.temperature_k, hollow.temperature_k);
            assert_eq!(a.snow, b.snow, "climate suitability is not relabelled");
        }
    }

    #[test]
    fn frost_settles_only_in_the_subzero_band_and_leaves_classification() {
        let field = field();
        for dir in dirs() {
            // Warm and deep-cold: no frost, regardless of substrate roughness.
            for temp in [300.0, 240.0] {
                let out = surface_appearance(&field, &sample(temp, 1.0), dir);
                assert_eq!(
                    out.frost_cover, 0.0,
                    "no frost outside the band at {temp} K"
                );
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
        // Inspect frost directly: snow availability and substrate roughness
        // also depend on moisture, so neither is an isolated frost signal.
        // Probe at the warm edge of the band (270 K): the frost band is
        // still fully 1 there while the snow edge (~270 K center) has only
        // half closed, so the moisture gate reads through snow suppression.
        let mean = |moisture: f64| {
            dirs()
                .iter()
                .map(|dir| {
                    surface_appearance(&field, &sample(270.0, moisture), *dir).frost_cover as f64
                })
                .sum::<f64>()
                / 8.0
        };
        let (moist, dry) = (mean(1.0), mean(0.0));
        assert!(
            moist > dry + 0.01,
            "moisture must gate frost glitter, moist={moist:.4} dry={dry:.4}"
        );
        // Compare identical substrate/cover inputs, not the former constant
        // baseline: forest can be rougher than exposed stone.
        for vegetation in [0.0, 0.5, 1.0] {
            for exposed in [0.0, 0.5, 1.0] {
                let baseline = land_roughness(vegetation, exposed, 0.7, 0.2, 0.0);
                assert!(land_roughness(vegetation, exposed, 0.7, 0.2, 0.8) <= baseline);
            }
        }
    }
}
