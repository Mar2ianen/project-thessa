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
    let h = sample.height_m;
    let snow = smooth(
        276.0,
        264.0,
        sample.temperature_k + regional * 5.0 + variation * 2.0,
    );
    if h < 0.0 {
        let shelf = (-h / 500.0).clamp(0.0, 1.0).sqrt();
        let c = mix([0.12, 0.42, 0.43], [0.018, 0.095, 0.19], shelf);
        let c = mix(c, [0.014, 0.044, 0.11], smooth(600.0, 6000.0, -h));
        let ice = smooth(263.0, 253.0, sample.temperature_k + regional * 6.0);
        return SurfaceAppearance {
            albedo_srgb: mix(c, [0.75, 0.84, 0.86], ice).map(|x| x as f32),
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
    color = color.map(|c| c * (1.0 + variation * 0.14 + regional * 0.08));
    color = mix(
        color,
        [0.88, 0.92, 0.94],
        snow * (1.0 - smooth(0.7, 1.4, sample.slope_hint) * 0.8),
    );
    SurfaceAppearance {
        albedo_srgb: color.map(|c| c.clamp(0.0, 1.0) as f32),
        roughness: 0.92,
        vegetation: vegetation as f32,
        snow: snow as f32,
    }
}
