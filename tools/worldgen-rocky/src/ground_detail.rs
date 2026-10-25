//! Optional body-fixed optical soil scans; no authoritative collision state.
//! Source scales/licenses: assets/textures/{forest-ground-04,mud-litter-01}.LICENSE.txt.
use std::{io::Read, sync::OnceLock};

pub const SWATCH_WIDTH_M: f64 = 3.15;
pub const HEIGHT_RANGE_M: f32 = 0.060;
const SIDE: usize = 512;
const RAW_BYTES: usize = 8 + SIDE * SIDE * 6;

#[derive(Clone, Copy, Debug)]
pub(crate) struct GroundSample {
    pub albedo_linear: [f32; 3],
    pub roughness: f32,
    pub height_m: f32,
}

struct Mip {
    side: usize,
    // RGB linear reflectance, linear perceptual roughness, relative height (m).
    pixels: Vec<[f32; 5]>,
}

struct Texture(Vec<Mip>);
static TEXTURE: OnceLock<Texture> = OnceLock::new();
static LITTER: OnceLock<Texture> = OnceLock::new();
const LITTER_WIDTH_M: f64 = 1.3;
const LITTER_HEIGHT_RANGE_M: f32 = 0.025;
const LITTER_MEAN: GroundSample = GroundSample {
    albedo_linear: [0.13887769, 0.10027028, 0.03611909],
    roughness: 0.9196433,
    height_m: -0.0054620567,
};

pub(crate) fn decode_srgb(v: f64) -> f64 {
    if v <= 0.04045 {
        v / 12.92
    } else {
        ((v + 0.055) / 1.055).powf(2.4)
    }
}

pub(crate) fn encode_srgb(v: f64) -> f64 {
    if v <= 0.0031308 {
        v * 12.92
    } else {
        1.055 * v.powf(1.0 / 2.4) - 0.055
    }
}

/// Last mip after intrinsic forest-substrate processing. Coarse pages use the
/// same material mean, not legacy canopy colour. Pinned against the asset below.
fn swatch_mean() -> GroundSample {
    GroundSample {
        albedo_linear: [0.089_765_41, 0.062_935_11, 0.039_580_63],
        roughness: 0.807_225_05,
        height_m: -0.0068553747,
    }
}

pub(crate) fn mean_sample() -> GroundSample {
    let a = swatch_mean();
    let w = 0.475_f32;
    GroundSample {
        albedo_linear: std::array::from_fn(|i| {
            a.albedo_linear[i] * (1.0 - w) + LITTER_MEAN.albedo_linear[i] * w
        }),
        roughness: a.roughness * (1.0 - w) + LITTER_MEAN.roughness * w,
        height_m: a.height_m,
    }
}

impl Texture {
    fn decode() -> Self {
        Self::decode_source(
            include_bytes!("../../../assets/textures/forest-ground-04.bin.gz"),
            HEIGHT_RANGE_M,
            true,
        )
    }
    fn decode_litter() -> Self {
        Self::decode_source(
            include_bytes!("../../../assets/textures/mud-litter-01.bin.gz"),
            LITTER_HEIGHT_RANGE_M,
            false,
        )
    }
    fn decode_source(bytes: &[u8], height_range_m: f32, transfer: bool) -> Self {
        let mut raw = Vec::new();
        flate2::read::GzDecoder::new(bytes)
            .take(RAW_BYTES as u64 + 1)
            .read_to_end(&mut raw)
            .expect("embedded ground material gzip");
        assert_eq!(raw.len(), RAW_BYTES, "bounded embedded material payload");
        assert_eq!(&raw[..8], &[b'F', b'G', b'0', b'1', 0, 2, 0, 0]);
        let pixels = raw[8..]
            .as_chunks::<6>()
            .0
            .iter()
            .map(|p| {
                let rgb = std::array::from_fn(|i| decode_srgb(f64::from(p[i]) / 255.0) as f32);
                let roughness = f32::from(p[3]) / 255.0;
                let (rgb, roughness) = if transfer {
                    forest_substrate(rgb, roughness)
                } else {
                    (rgb, roughness)
                };
                [
                    rgb[0],
                    rgb[1],
                    rgb[2],
                    roughness,
                    (f32::from(u16::from_le_bytes([p[4], p[5]])) / 65535.0 - 0.5) * height_range_m,
                ]
            })
            .collect();
        let mut mips = vec![Mip { side: SIDE, pixels }];
        while mips.last().unwrap().side > 1 {
            let previous = mips.last().unwrap();
            let side = previous.side / 2;
            let pixels = (0..side * side)
                .map(|i| {
                    let x = i % side * 2;
                    let y = i / side * 2;
                    std::array::from_fn(|c| {
                        (previous.pixels[y * previous.side + x][c]
                            + previous.pixels[y * previous.side + x + 1][c]
                            + previous.pixels[(y + 1) * previous.side + x][c]
                            + previous.pixels[(y + 1) * previous.side + x + 1][c])
                            * 0.25
                    })
                })
                .collect();
            mips.push(Mip { side, pixels });
        }
        Self(mips)
    }

    fn sample(&self, uv: [f64; 2], lod: f64) -> [f32; 5] {
        let lod = lod.clamp(0.0, (self.0.len() - 1) as f64);
        let a = self.0[lod.floor() as usize].sample(uv);
        let b = self.0[lod.ceil() as usize].sample(uv);
        std::array::from_fn(|c| a[c] + (b[c] - a[c]) * lod.fract() as f32)
    }
    #[cfg(test)]
    fn sample_stochastic(&self, uv: [f64; 2], lod: f64, seed: u64, projection: u32) -> [f32; 5] {
        self.sample_stochastic_width(uv, lod, seed, projection, SWATCH_WIDTH_M)
    }
    fn sample_stochastic_width(
        &self,
        uv: [f64; 2],
        lod: f64,
        seed: u64,
        projection: u32,
        width_m: f64,
    ) -> [f32; 5] {
        let cell = uv.map(|v| v.floor() as i64);
        // A 30 cm transition keeps most of each patch unblurred. Adjacent
        // cells share corner sources, so colour and physical height agree.
        let half_blend = 0.15 / width_m;
        let t = uv
            .map(|v| crate::appearance::smooth(0.5 - half_blend, 0.5 + half_blend, v - v.floor()));
        let mut result = [0.0; 5];
        for y in 0..2 {
            for x in 0..2 {
                let weight = (if x == 0 { 1.0 - t[0] } else { t[0] })
                    * (if y == 0 { 1.0 - t[1] } else { t[1] });
                if weight == 0.0 {
                    continue;
                }
                let hash = |channel| {
                    crate::rng::hash01(
                        seed,
                        1780 + projection * 8 + channel,
                        cell[0] + x,
                        cell[1] + y,
                    )
                };
                let rotated = match ((hash(0) * 4.0) as u32).min(3) {
                    0 => uv,
                    1 => [-uv[1], uv[0]],
                    2 => [-uv[0], -uv[1]],
                    _ => [uv[1], -uv[0]],
                };
                let value = self.sample([rotated[0] + hash(1), rotated[1] + hash(2)], lod);
                for (out, value) in result.iter_mut().zip(value) {
                    *out += value * weight as f32;
                }
            }
        }
        result
    }
}

impl Mip {
    fn sample(&self, uv: [f64; 2]) -> [f32; 5] {
        let p = uv.map(|v| v.rem_euclid(1.0) * self.side as f64 - 0.5);
        let xy = p.map(|v| v.floor() as i64);
        let [u, v] = p.map(|v| v.fract().rem_euclid(1.0) as f32);
        let at = |dx: i64, dy: i64| {
            self.pixels[((xy[1] + dy).rem_euclid(self.side as i64) as usize) * self.side
                + (xy[0] + dx).rem_euclid(self.side as i64) as usize]
        };
        let [a, b, c, d] = [at(0, 0), at(1, 0), at(0, 1), at(1, 1)];
        std::array::from_fn(|i| {
            (a[i] * (1.0 - u) + b[i] * u) * (1.0 - v) + (c[i] * (1.0 - u) + d[i] * u) * v
        })
    }
}

/// Triplanar body-space projection avoids cube-face/latitude seams. Texture
/// phase is seed-owned, never page-, worker-, camera- or geometry-LOD-owned.
pub(crate) fn sample(dir: [f64; 3], radius_m: f64, seed: u64, texel_m: f64) -> GroundSample {
    let q = dir.map(|v| v * radius_m / 8.0);
    let field = crate::rng::perlin3(seed, 6200, q[0], q[1], q[2]);
    let w = (0.1
        + 0.75
            * crate::appearance::smooth(-0.25, 0.30, field)
            * crate::appearance::band_weight(8.0, texel_m)
        + 0.375 * (1.0 - crate::appearance::band_weight(8.0, texel_m))) as f32;
    let a = if texel_m >= SWATCH_WIDTH_M {
        swatch_mean()
    } else {
        sample_texture(
            TEXTURE.get_or_init(Texture::decode),
            dir,
            radius_m,
            seed,
            texel_m,
            SWATCH_WIDTH_M,
        )
    };
    let b = if texel_m >= LITTER_WIDTH_M {
        LITTER_MEAN
    } else {
        sample_texture(
            LITTER.get_or_init(Texture::decode_litter),
            dir,
            radius_m,
            seed ^ 0x6c6974746572,
            texel_m,
            LITTER_WIDTH_M,
        )
    };
    let mean = swatch_mean().height_m;
    GroundSample {
        albedo_linear: std::array::from_fn(|i| {
            a.albedo_linear[i] * (1.0 - w) + b.albedo_linear[i] * w
        }),
        roughness: a.roughness * (1.0 - w) + b.roughness * w,
        // Center both physical sources independently: changing litter coverage
        // must not invent a bump from unrelated displacement-map zero points.
        height_m: mean + (a.height_m - mean) * (1.0 - w) + (b.height_m - LITTER_MEAN.height_m) * w,
    }
}
fn sample_texture(
    texture: &Texture,
    dir: [f64; 3],
    radius_m: f64,
    seed: u64,
    texel_m: f64,
    width_m: f64,
) -> GroundSample {
    let p: [f64; 3] = std::array::from_fn(|i| {
        dir[i] * radius_m / width_m + crate::rng::hash01(seed, 1741 + i as u32, 0, 0)
    });
    let weights = dir.map(|v| v.abs().powi(8));
    let sum: f64 = weights.iter().sum();
    // A mip texel integrates one footprint width. Doubling it here adds a
    // blur before the resident page receives its own derivative filtering.
    let lod = (texel_m * SIDE as f64 / width_m).max(1.0).log2();
    let projections = [[p[2], p[1]], [p[0], p[2]], [p[0], p[1]]];
    let mut value = [0.0; 5];
    for (projection, (uv, weight)) in projections.into_iter().zip(weights).enumerate() {
        if weight / sum < 1e-8 {
            continue;
        }
        for (out, sample) in value.iter_mut().zip(texture.sample_stochastic_width(
            uv,
            lod,
            seed,
            projection as u32,
            width_m,
        )) {
            *out += sample * (weight / sum) as f32;
        }
    }
    GroundSample {
        albedo_linear: [value[0], value[1], value[2]],
        roughness: value[3],
        height_m: value[4],
    }
}

// Transform intrinsic pigment/mineral response before filtering: applying a
// nonlinear brightness mask to an already averaged texel changes its mean.
fn forest_substrate(rgb: [f32; 3], roughness: f32) -> ([f32; 3], f32) {
    let luminance = |v: [f32; 3]| {
        0.2126 * f64::from(v[0]) + 0.7152 * f64::from(v[1]) + 0.0722 * f64::from(v[2])
    };
    let mineral = crate::appearance::smooth(
        0.90,
        1.35,
        luminance(rgb) / luminance([0.15293027, 0.10844741, 0.06403417]),
    );
    let color = std::array::from_fn(|i| {
        let absorption = [0.28, 0.24, 0.25][i];
        (f64::from(rgb[i]) * (absorption + (1.0 - absorption) * mineral)) as f32
    });
    (
        color,
        (f64::from(roughness) * (1.0 - mineral * 0.35)) as f32,
    )
}

/// Body-fixed optical cover, not authoritative water storage or moss biology.
#[derive(Clone, Copy, Debug, Default)]
pub struct GroundCover {
    pub wetness: f64,
    pub moss: f64,
    pub moss_height_m: f64,
}

pub fn cover(
    q: [f64; 3],
    seed: u64,
    texel_m: f64,
    damp: f64,
    canopy: f64,
    source_height_m: f64,
) -> GroundCover {
    use crate::appearance::{band_weight, smooth};
    let noise = |channel, wavelength, perlin| {
        let p = q.map(|v| v / wavelength);
        let value = if perlin {
            crate::rng::perlin3(seed, channel, p[0], p[1], p[2])
        } else {
            crate::rng::value_noise3(seed, channel, p[0], p[1], p[2])
        };
        value * band_weight(wavelength, texel_m)
    };
    let moss_field = noise(6143, 0.85, true) * 0.65 + noise(6148, 0.25, false) * 0.35;
    let moss = smooth(0.28, 0.58, damp)
        * (0.45 + 0.55 * smooth(0.02, 0.20, canopy))
        * smooth(-0.12, 0.18, moss_field)
        * band_weight(0.85, texel_m)
        * 0.85;
    // A connected cushion and smaller hummocks share the pigment's world phase.
    let cushion = moss * moss * (0.035 + 0.025 * smooth(-0.25, 0.25, noise(6160, 0.18, true)));
    // Moisture availability is canonical; scan hollows and metre-scale retention
    // distribute that availability. This is not a rain/runoff integrator.
    let retention = smooth(-0.30, 0.25, noise(6161, 2.4, true));
    let hollows = 1.0 - smooth(-0.018, 0.012, source_height_m);
    GroundCover {
        wetness: (damp * (0.50 + 0.35 * retention + 0.35 * hollows)).clamp(0.0, 1.0),
        moss,
        moss_height_m: cushion,
    }
}

/// Authored forest-floor optical layers. Wetness is a material response to the
/// canonical moisture field, not a standing-water or drainage simulation.
pub(crate) fn forest_floor(
    ground: GroundSample,
    q: [f64; 3],
    seed: u64,
    texel_m: f64,
    damp: f64,
    canopy: f64,
) -> GroundSample {
    use crate::appearance::{band_weight, smooth};
    let noise = |channel, wavelength, perlin| {
        let p = q.map(|v| v / wavelength);
        let value = if perlin {
            crate::rng::perlin3(seed, channel, p[0], p[1], p[2])
        } else {
            crate::rng::value_noise3(seed, channel, p[0], p[1], p[2])
        };
        value * band_weight(wavelength, texel_m)
    };
    let mineral_patch = smooth(0.05, 0.45, noise(6146, 3.2, true));
    let fine_mineral = smooth(0.52, 0.82, noise(6147, 0.22, false)) * 0.25;
    let cover = cover(q, seed, texel_m, damp, canopy, f64::from(ground.height_m));
    let wet = cover.wetness;
    let moss = cover.moss;
    let coarse_litter =
        smooth(0.08, 0.58, noise(6142, 1.6, true)) * (0.40 + 0.60 * smooth(0.04, 0.25, canopy));
    let fine_litter =
        smooth(0.12, 0.62, noise(6151, 0.45, true)) * (0.35 + 0.65 * smooth(0.02, 0.20, canopy));
    let litter = (coarse_litter * 0.50 + fine_litter * 0.45).clamp(0.0, 0.76);
    let albedo_linear = std::array::from_fn(|i| {
        let wet_darkening = [0.52, 0.52, 0.46][i];
        let mineral_factor = 1.0 + mineral_patch * 0.15 + fine_mineral * [0.15, 0.18, 0.28][i];
        let substrate =
            f64::from(ground.albedo_linear[i]) * mineral_factor * (1.0 - wet * wet_darkening);
        // Pigment coverage retains scan contrast instead of painting a flat swatch.
        let texture_detail = (f64::from(ground.albedo_linear[i])
            / f64::from(mean_sample().albedo_linear[i]))
        .clamp(0.5, 1.5);
        let moss_albedo = [0.045, 0.058, 0.028][i] * texture_detail;
        let covered = substrate * (1.0 - moss) + moss_albedo * moss;
        let litter_albedo = [0.062, 0.049, 0.035][i] * texture_detail;
        (covered * (1.0 - litter) + litter_albedo * litter).clamp(0.0, 1.0) as f32
    });
    let dry = f64::from(ground.roughness);
    let wet_roughness = dry * (1.0 - wet) + (0.18 * (1.0 - fine_mineral * 0.20)) * wet;
    let moss_roughness = wet_roughness * (1.0 - moss) + 0.92 * moss;
    GroundSample {
        albedo_linear,
        roughness: (moss_roughness * (1.0 - litter) + 0.75 * litter).clamp(0.22, 0.98) as f32,
        // Optical pigments/wetness do not amplify the scan's physical slope.
        height_m: ground.height_m,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn native_scan_footprint_does_not_add_an_extra_mip_of_blur() {
        let texture = Texture::decode();
        let seed = 42;
        let r = 3_200_000.0;
        let uv = [
            crate::rng::hash01(seed, 1743, 0, 0),
            crate::rng::hash01(seed, 1742, 0, 0),
        ];
        let expected = texture.sample_stochastic(uv, 0.0, seed, 0);
        let actual = sample_texture(
            &texture,
            [1.0, 0.0, 0.0],
            r,
            seed,
            SWATCH_WIDTH_M / SIDE as f64,
            SWATCH_WIDTH_M,
        );
        for (a, e) in actual.albedo_linear.into_iter().zip(expected) {
            assert!(
                (a - e).abs() < 1e-7,
                "native scan footprint must retain mip zero"
            );
        }
    }

    #[test]
    fn cover_retains_hollow_wetness_and_bounded_resolved_cushions() {
        let q = [137.0, 29.0, -18.0];
        let low = cover(q, 77, 0.012, 0.9, 0.6, -0.025);
        let high = cover(q, 77, 0.012, 0.9, 0.6, 0.025);
        assert!(low.wetness > high.wetness);
        let mut maximum = 0.0_f64;
        for i in 0..400 {
            let q = [
                137.0 + (i % 20) as f64 * 0.13,
                29.0,
                -18.0 + (i / 20) as f64 * 0.13,
            ];
            let c = cover(q, 77, 0.012, 0.9, 0.6, 0.0);
            assert!((0.0..=0.06).contains(&c.moss_height_m));
            maximum = maximum.max(c.moss_height_m);
            let coarse = cover(q, 77, 2.0, 0.9, 0.6, 0.0);
            assert_eq!(coarse.moss_height_m, 0.0);
            assert_eq!(cover(q, 77, 0.012, 0.0, 0.6, 0.0).moss_height_m, 0.0);
        }
        assert!(maximum > 0.015);
    }
    #[test]
    fn imported_mud_litter_preserves_means_and_world_patch_continuity() {
        let tex = Texture::decode_litter();
        let last = tex.0.last().unwrap().pixels[0];
        let expected = [
            LITTER_MEAN.albedo_linear[0],
            LITTER_MEAN.albedo_linear[1],
            LITTER_MEAN.albedo_linear[2],
            LITTER_MEAN.roughness,
            LITTER_MEAN.height_m,
        ];
        for (a, b) in last.into_iter().zip(expected) {
            assert!((a - b).abs() < 2e-7);
        }
        let r = 3_200_000.0;
        let mut min = 1.0_f32;
        let mut max = 0.0_f32;
        for x in 0..160 {
            let dir = crate::lod::normalize([1.0, x as f64 * 0.5 / r, 0.137 / r]);
            let a = sample(dir, r, 42, 0.01);
            let b = sample(
                crate::lod::normalize([1.0, (x as f64 * 0.5 + 1e-6) / r, 0.137 / r]),
                r,
                42,
                0.01,
            );
            for (a, b) in a.albedo_linear.into_iter().zip(b.albedo_linear) {
                assert!((a - b).abs() < 0.0001);
            }
            assert!((a.height_m - b.height_m).abs() < 0.00001);
            min = min.min(a.albedo_linear[1]);
            max = max.max(a.albedo_linear[1]);
        }
        assert!(
            max - min > 0.03,
            "ground must contain distinct correlated material variation"
        );
    }

    #[test]
    fn stochastic_patches_are_continuous_without_swatch_repetition() {
        let texture = Texture::decode();
        for axis in 0..2 {
            for boundary in [-17.0, -0.5, 0.0, 0.5, 1.0, 39.0] {
                let mut a = [0.137, 0.283];
                a[axis] = boundary - 1e-9;
                let mut b = a;
                b[axis] = boundary + 1e-9;
                for (a, b) in texture
                    .sample_stochastic(a, 0.0, 42, 0)
                    .into_iter()
                    .zip(texture.sample_stochastic(b, 0.0, 42, 0))
                {
                    assert!((a - b).abs() < 1e-5, "patch seam at {boundary}");
                }
            }
        }
        let mut difference = 0.0;
        for i in 0..64 {
            let uv = [i as f64 * 0.137, -i as f64 * 0.283];
            let a = texture.sample_stochastic(uv, 0.0, 42, 0);
            let b = texture.sample_stochastic([uv[0] + 1.0, uv[1]], 0.0, 42, 0);
            assert_eq!(a, texture.sample_stochastic(uv, 0.0, 42, 0));
            difference += (a[0] - b[0]).abs();
        }
        assert!(
            difference > 0.1,
            "one-swatch translation must not reproduce the same stones"
        );
    }

    #[test]
    fn mineral_transfer_is_continuous_before_mip_averaging() {
        let raw_mean = [0.15293027, 0.10844741, 0.06403417];
        let mut previous: Option<([f32; 3], f32)> = None;
        for step in 0..=2000 {
            let rgb = raw_mean.map(|v| v * (0.5 + step as f32 / 2000.0));
            let current = forest_substrate(rgb, 0.9);
            if let Some(previous) = previous {
                for (a, b) in current.0.into_iter().zip(previous.0) {
                    assert!((a - b).abs() < 0.001);
                }
                assert!((current.1 - previous.1).abs() < 0.001);
            }
            previous = Some(current);
        }
        let dark = forest_substrate(raw_mean.map(|v| v * 0.5), 0.9);
        let bright = forest_substrate(raw_mean.map(|v| v * 1.5), 0.9);
        let wrong = forest_substrate(raw_mean, 0.9);
        assert!(
            ((dark.0[0] + bright.0[0]) * 0.5 - wrong.0[0]).abs() > 0.03,
            "filtering before the nonlinear transfer loses the stone fraction"
        );
    }

    #[test]
    fn optical_layers_are_continuous_and_preserve_source_height() {
        let mean = mean_sample();
        let q = [137.0, 29.0, -18.0];
        for damp in [0.0, 0.5, 1.0] {
            for canopy in [0.0, 0.5, 1.0] {
                for height_m in [-HEIGHT_RANGE_M * 0.5, 0.0, HEIGHT_RANGE_M * 0.5] {
                    let mut previous: Option<GroundSample> = None;
                    for step in 0..=600 {
                        let ground = GroundSample {
                            albedo_linear: mean
                                .albedo_linear
                                .map(|v| v * (0.5 + step as f32 / 600.0)),
                            roughness: 0.9,
                            height_m,
                        };
                        let floor = forest_floor(ground, q, 77, 0.012, damp, canopy);
                        assert_eq!(floor.height_m, height_m);
                        assert!((0.22..=0.98).contains(&floor.roughness));
                        assert!(
                            floor
                                .albedo_linear
                                .iter()
                                .all(|v| v.is_finite() && (0.0..=1.0).contains(v))
                        );
                        if let Some(previous) = previous {
                            for (a, b) in
                                floor.albedo_linear.into_iter().zip(previous.albedo_linear)
                            {
                                assert!(
                                    (a - b).abs() < 0.002,
                                    "mineral boundary must be continuous"
                                );
                            }
                        }
                        previous = Some(floor);
                    }
                }
            }
        }
        let dry = forest_floor(mean, q, 77, 0.012, 0.0, 0.0);
        let wet = forest_floor(mean, q, 77, 0.012, 1.0, 0.0);
        assert!(wet.roughness < dry.roughness);
        assert!(
            wet.albedo_linear
                .into_iter()
                .zip(dry.albedo_linear)
                .all(|(w, d)| w < d)
        );
        // At coarse resolution every procedural pigment band is filtered out.
        let a = forest_floor(mean, q, 77, 2.0, 0.5, 0.5);
        let b = forest_floor(mean, [0.0, 1000.0, 8.0], 77, 2.0, 0.5, 0.5);
        assert_eq!(a.albedo_linear, b.albedo_linear);
        assert_eq!(a.roughness, b.roughness);
    }

    #[test]
    fn damp_moss_forms_resolved_patches_without_erasing_scan_detail() {
        let mean = mean_sample();
        let mut green_ratio = [f32::INFINITY, 0.0_f32];
        let mut patch = [0.0; 3];
        for i in 0..400 {
            let q = [
                137.0 + (i % 20) as f64 * 0.13,
                29.0,
                -18.0 + (i / 20) as f64 * 0.13,
            ];
            let floor = forest_floor(mean, q, 77, 0.012, 0.9, 0.6);
            let ratio = floor.albedo_linear[1] / floor.albedo_linear[0];
            green_ratio[0] = green_ratio[0].min(ratio);
            if ratio > green_ratio[1] {
                green_ratio[1] = ratio;
                patch = q;
            }
        }
        assert!(
            green_ratio[1] > 1.0,
            "damp coastal ground needs visible moss patches"
        );
        assert!(
            green_ratio[1] - green_ratio[0] > 0.2,
            "coverage must be spatially varied"
        );
        let dark = forest_floor(
            GroundSample {
                albedo_linear: mean.albedo_linear.map(|v| v * 0.8),
                ..mean
            },
            patch,
            77,
            0.012,
            0.9,
            0.6,
        );
        let bright = forest_floor(
            GroundSample {
                albedo_linear: mean.albedo_linear.map(|v| v * 1.2),
                ..mean
            },
            patch,
            77,
            0.012,
            0.9,
            0.6,
        );
        assert!(
            bright
                .albedo_linear
                .into_iter()
                .zip(dark.albedo_linear)
                .all(|(a, b)| a > b * 1.1)
        );
        assert_eq!(bright.height_m, dark.height_m);
    }

    #[test]
    fn swatch_mips_preserve_linear_means_and_wrap_continuously() {
        let texture = Texture::decode();
        assert_eq!(texture.0.len(), 10);
        let base = &texture.0[0];
        for c in 0..5 {
            let mean =
                base.pixels.iter().map(|p| f64::from(p[c])).sum::<f64>() / base.pixels.len() as f64;
            assert!((f64::from(texture.0.last().unwrap().pixels[0][c]) - mean).abs() < 2e-7);
        }
        for lod in [0.0, 2.5, 9.0] {
            assert_eq!(
                texture.sample([0.125, 0.25], lod),
                texture.sample([1.125, -0.75], lod)
            );
            let a = texture.sample([-1e-8, 0.25], lod);
            let b = texture.sample([1e-8, 0.25], lod);
            for (a, b) in a.into_iter().zip(b) {
                assert!((a - b).abs() < 1e-4);
            }
        }
        let mean = swatch_mean();
        let expected = [
            mean.albedo_linear[0],
            mean.albedo_linear[1],
            mean.albedo_linear[2],
            mean.roughness,
            mean.height_m,
        ];
        for (a, b) in texture.0.last().unwrap().pixels[0]
            .into_iter()
            .zip(expected)
        {
            assert!((a - b).abs() < 2e-7);
        }
        assert_eq!(
            sample([1.0, 0.0, 0.0], 3_200_000.0, 42, 16.0).albedo_linear,
            mean_sample().albedo_linear
        );
    }

    #[test]
    fn world_fixed_material_is_bounded_and_continuous_across_cube_edges() {
        for face in 0..6 {
            for i in 0..64 {
                let dir = crate::lod::TileKey::root(face).direction(i as f64 / 63.0, 0.5);
                let a = sample(dir, 3_200_000.0, 42, 0.024);
                let b = sample(dir, 3_200_000.0, 42, 0.024);
                assert_eq!(a.albedo_linear, b.albedo_linear);
                assert!(
                    a.albedo_linear
                        .iter()
                        .all(|v| v.is_finite() && (0.0..=1.0).contains(v))
                );
                assert!((0.0..=1.0).contains(&a.roughness));
                assert!(a.height_m.abs() <= HEIGHT_RANGE_M * 0.5);
            }
        }
        // Approach the same signed cube edge from its two incident charts.
        // A chart-indexed texture would jump here; this projection has no face ID.
        for axis in 0..3 {
            for sign in [-1.0, 1.0] {
                let mut a = [0.25, 0.25, 0.25];
                a[axis] = sign;
                a[(axis + 1) % 3] = 1.0 - 1e-12;
                let mut b = a;
                b[(axis + 1) % 3] = 1.0 + 1e-12;
                let unit = |v: [f64; 3]| {
                    let length = v.iter().map(|c| c * c).sum::<f64>().sqrt();
                    v.map(|c| c / length)
                };
                let a = sample(unit(a), 3_200_000.0, 42, 0.024);
                let b = sample(unit(b), 3_200_000.0, 42, 0.024);
                for (a, b) in a.albedo_linear.into_iter().zip(b.albedo_linear) {
                    assert!((a - b).abs() < 0.002, "cube-edge reflectance discontinuity");
                }
                assert!((a.height_m - b.height_m).abs() < 2e-5);
            }
        }
    }
}
