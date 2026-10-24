//! Optional, body-fixed optical soil/gravel swatch. No terrain or collision state.
//! Source and authored scales: assets/textures/forest-ground-04.LICENSE.txt.
use std::{io::Read, sync::OnceLock};

pub const SWATCH_WIDTH_M: f64 = 3.15;
pub const HEIGHT_RANGE_M: f32 = 0.018;
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

/// Last mip of the imported swatch. Coarse pages use the *same* material mean,
/// not the legacy canopy colour. Pinned against the decoded asset below.
pub(crate) fn mean_sample() -> GroundSample {
    GroundSample {
        albedo_linear: [0.15293027, 0.10844741, 0.06403417],
        roughness: 0.90125257,
        height_m: -0.0020566124,
    }
}

impl Texture {
    fn decode() -> Self {
        let mut raw = Vec::new();
        flate2::read::GzDecoder::new(
            &include_bytes!("../../../assets/textures/forest-ground-04.bin.gz")[..],
        )
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
                [
                    decode_srgb(f64::from(p[0]) / 255.0) as f32,
                    decode_srgb(f64::from(p[1]) / 255.0) as f32,
                    decode_srgb(f64::from(p[2]) / 255.0) as f32,
                    f32::from(p[3]) / 255.0,
                    (f32::from(u16::from_le_bytes([p[4], p[5]])) / 65535.0 - 0.5) * HEIGHT_RANGE_M,
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
    if texel_m * 2.0 >= SWATCH_WIDTH_M {
        return mean_sample();
    }
    let texture = TEXTURE.get_or_init(Texture::decode);
    let p: [f64; 3] = std::array::from_fn(|i| {
        dir[i] * radius_m / SWATCH_WIDTH_M + crate::rng::hash01(seed, 1741 + i as u32, 0, 0)
    });
    let weights = dir.map(|v| v.abs().powi(8));
    let sum: f64 = weights.iter().sum();
    // Filter source wavelengths shorter than two destination texels.
    let lod = (2.0 * texel_m * SIDE as f64 / SWATCH_WIDTH_M)
        .max(1.0)
        .log2();
    let projections = [[p[2], p[1]], [p[0], p[2]], [p[0], p[1]]];
    let mut value = [0.0; 5];
    for (uv, weight) in projections.into_iter().zip(weights) {
        if weight / sum < 1e-8 {
            continue;
        }
        for (out, sample) in value.iter_mut().zip(texture.sample(uv, lod)) {
            *out += sample * (weight / sum) as f32;
        }
    }
    GroundSample {
        albedo_linear: [value[0], value[1], value[2]],
        roughness: value[3],
        height_m: value[4],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
        let mean = mean_sample();
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
            sample([1.0, 0.0, 0.0], 3_200_000.0, 42, 3.125).albedo_linear,
            mean.albedo_linear
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
